#![no_std]
#![allow(dead_code)]
use embedded_hal::blocking::{delay::DelayMs, i2c};

mod icm20689_abstractions;
pub use abs::{AccelConfig, DLPFBandwidth, GyroConfig};
use icm20689_abstractions as abs;

// macro to transform the coordinate frame of the measurements
macro_rules! turn {
    ($T:expr, $vec:expr) => {
        $T[0] * ($vec[0] as f64) + $T[1] * ($vec[1] as f64) + $T[2] * ($vec[2] as f64)
    };
}

macro_rules! read_hl {
    ($buffer:expr, $start_bit_index:expr) => {
        ((($buffer[$start_bit_index] as u16) << 8) | ($buffer[$start_bit_index + 1] as u16)) as i16
    };
}

struct Calibration {
    scale: f64,
    num_samples: usize,
    BD: [f64; 3],
    B: [f64; 3],
    S: [f64; 3],
    min: [f64; 3],
    max: [f64; 3],
}

impl Default for Calibration {
    fn default() -> Self {
        Self {
            scale: 0f64,
            num_samples: 100usize,
            BD: [0f64, 0f64, 0f64],
            B: [0f64, 0f64, 0f64],
            S: [1f64, 1f64, 1f64],
            min: [0f64, 0f64, 0f64],
            max: [0f64, 0f64, 0f64],
        }
    }
}

#[derive(Default)]
pub struct Measurement {
    calibration: Calibration,
    counts: [i16; 3],
    pub values: [f64; 3],
}

#[derive(Debug)]
pub enum ICMError {
    I2cReadError,
    I2cWriteError,
    ICMIDError,
}

pub struct ICM20689<I>
where
    I: i2c::Write + i2c::Read,
{
    i2c: I,
    icm_address: u8,

    accel_range: AccelConfig,
    pub accel_measurement: Measurement,

    gyro_range: GyroConfig,
    pub gyro_measurement: Measurement,

    temp_measurement: f64,

    bandwidth: DLPFBandwidth,
    srd: u8,
}

impl<I> ICM20689<I>
where
    I: i2c::Write + i2c::Read + i2c::WriteRead,
{
    fn write_reg(&mut self, addr: u8, data: u8) -> Result<(), ICMError> {
        if let Ok(()) = self.i2c.write(self.icm_address, &[addr, data]) {
            return Ok(());
        } else {
            Err(ICMError::I2cWriteError)
        }
    }

    fn read_reg(&mut self, addr: u8, buf: &mut u8) -> Result<(), ICMError> {
        if let Ok(()) = self.i2c.write_read(self.icm_address, &[addr], &mut [*buf]) {
            Ok(())
        } else {
            Err(ICMError::I2cReadError)
        }
    }

    fn read_registers(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), ICMError> {
        if let Ok(()) = self.i2c.write_read(self.icm_address, &[addr], buf) {
            Ok(())
        } else {
            Err(ICMError::I2cReadError)
        }
    }

    pub fn new(i2c: I, icm_address: u8) -> Self {
        Self {
            i2c,
            icm_address,

            accel_range: AccelConfig::ACCEL_RANGE_16G,
            accel_measurement: Measurement::default(),

            gyro_range: GyroConfig::GYRO_RANGE_2000DPS,
            gyro_measurement: Measurement::default(),

            temp_measurement: 0f64,

            bandwidth: DLPFBandwidth::DLPF_BANDWIDTH_MAX,
            srd: 0u8,
        }
    }

    pub fn init<D>(&mut self, delay: &mut D) -> Result<(), ICMError>
    where
        D: DelayMs<u16>,
    {
        delay.delay_ms(80);

        // telling the ICM to choose the best clock source
        if let Result::Ok(()) = self.i2c.write(
            self.icm_address,
            &[abs::power_management::PWR_MGMT_1, abs::CLOCK_SEL_PLL],
        ) {
            // resetting the ICM
            self.write_reg(abs::power_management::PWR_MGMT_1, abs::PWR_RESET)?;
            delay.delay_ms(2);

            // telling the ICM to choose the best clock source
            self.write_reg(abs::power_management::PWR_MGMT_1, abs::CLOCK_SEL_PLL)?;

            // check the whoami register of the ICM
            if let Ok(icm_id) = self.whoami() {
                if icm_id != 152 {
                    // println!("bypassing icmiderror for now");
                    // return Err(ICMError::ICMIDError);
                }
            } else {
                return Err(ICMError::I2cReadError);
            }

            // enable the sensor
            self.write_reg(abs::power_management::PWR_MGMT_2, abs::SENSOR_EN)?;

            // set accelerometer and gyroscope ranges
            self.set_accel_range(self.accel_range)?;
            self.set_gyro_range(self.gyro_range)?;

            // TODO tune these bandwidths experimentally check page 39 to 41 in documentation
            self.set_dlpf_bandwidth(self.bandwidth)?;
            self.set_srd(self.srd)?;

            // TODO calibrate gyro and accelerometer
            Ok(())
        } else {
            // if there's an error, wait for 40 millis and try again
            delay.delay_ms(40);
            self.init(delay)
        }
    }

    pub fn set_accel_range(&mut self, range: abs::AccelConfig) -> Result<(), ICMError> {
        self.accel_range = range;
        self.write_reg(abs::configuration::ACCEL_CONFIG, self.accel_range as u8)?;
        self.accel_measurement.calibration.scale = (abs::g / 32767.5f64)
            * match range {
                abs::AccelConfig::ACCEL_RANGE_2G => 2f64,
                abs::AccelConfig::ACCEL_RANGE_4G => 4f64,
                abs::AccelConfig::ACCEL_RANGE_8G => 8f64,
                abs::AccelConfig::ACCEL_RANGE_16G => 16f64,
            };
        Ok(())
    }

    pub fn set_gyro_range(&mut self, range: abs::GyroConfig) -> Result<(), ICMError> {
        self.gyro_range = range;
        self.write_reg(abs::configuration::GYRO_CONFIG, self.gyro_range as u8)?;
        self.gyro_measurement.calibration.scale = (abs::d2r / 32767.5f64)
            * match range {
                abs::GyroConfig::GYRO_RANGE_250DPS => 250f64,
                abs::GyroConfig::GYRO_RANGE_500DPS => 500f64,
                abs::GyroConfig::GYRO_RANGE_1000DPS => 1000f64,
                abs::GyroConfig::GYRO_RANGE_2000DPS => 2000f64,
            };
        Ok(())
    }

    pub fn set_dlpf_bandwidth(&mut self, bandwidth: abs::DLPFBandwidth) -> Result<(), ICMError> {
        match bandwidth {
            abs::DLPFBandwidth::DLPF_BANDWIDTH_MAX => {
                self.write_reg(
                    abs::configuration::ACCEL_CONFIG_2,
                    abs::AccelDLPFBW::ACCEL_DLPF_1046HZ as u8,
                )?;
                let mut gyro_config_value: u8 = 0u8;
                self.read_reg(abs::configuration::GYRO_CONFIG, &mut gyro_config_value)?;
                self.write_reg(
                    abs::configuration::GYRO_CONFIG,
                    ((gyro_config_value & 0xfc) | abs::GYRO_FCHOICE_B_8173HZ),
                )?;
            }
            abs::DLPFBandwidth::DLPF_BANDWIDTH_218HZ => {
                self.write_reg(
                    abs::configuration::ACCEL_CONFIG_2,
                    abs::AccelDLPFBW::ACCEL_DLPF_218HZ as u8,
                )?;
                self.write_reg(
                    abs::configuration::CONFIG,
                    abs::GyroDLPFBW::GYRO_DLPF_250HZ as u8,
                )?;
            }
            abs::DLPFBandwidth::DLPF_BANDWIDTH_99HZ => {
                self.write_reg(
                    abs::configuration::ACCEL_CONFIG_2,
                    abs::AccelDLPFBW::ACCEL_DLPF_99HZ as u8,
                )?;
                self.write_reg(
                    abs::configuration::CONFIG,
                    abs::GyroDLPFBW::GYRO_DLPF_92HZ as u8,
                )?;
            }
            abs::DLPFBandwidth::DLPF_BANDWIDTH_45HZ => {
                self.write_reg(
                    abs::configuration::ACCEL_CONFIG_2,
                    abs::AccelDLPFBW::ACCEL_DLPF_45HZ as u8,
                )?;
                self.write_reg(
                    abs::configuration::CONFIG,
                    abs::GyroDLPFBW::GYRO_DLPF_41HZ as u8,
                )?;
            }
            abs::DLPFBandwidth::DLPF_BANDWIDTH_21HZ => {
                self.write_reg(
                    abs::configuration::ACCEL_CONFIG_2,
                    abs::AccelDLPFBW::ACCEL_DLPF_21HZ as u8,
                )?;
                self.write_reg(
                    abs::configuration::CONFIG,
                    abs::GyroDLPFBW::GYRO_DLPF_20HZ as u8,
                )?;
            }
            abs::DLPFBandwidth::DLPF_BANDWIDTH_10HZ => {
                self.write_reg(
                    abs::configuration::ACCEL_CONFIG_2,
                    abs::AccelDLPFBW::ACCEL_DLPF_10HZ as u8,
                )?;
                self.write_reg(
                    abs::configuration::CONFIG,
                    abs::GyroDLPFBW::GYRO_DLPF_10HZ as u8,
                )?;
            }
            abs::DLPFBandwidth::DLPF_BANDWIDTH_5HZ => {
                self.write_reg(
                    abs::configuration::ACCEL_CONFIG_2,
                    abs::AccelDLPFBW::ACCEL_DLPF_5HZ as u8,
                )?;
                self.write_reg(
                    abs::configuration::CONFIG,
                    abs::GyroDLPFBW::GYRO_DLPF_5HZ as u8,
                )?;
            }
        }
        Ok(())
    }

    pub fn set_srd(&mut self, srd: u8) -> Result<(), ICMError> {
        self.write_reg(abs::SMPLRT_DIV, 19u8)?;
        self.write_reg(abs::SMPLRT_DIV, srd)?;
        self.srd = srd;
        Ok(())
    }

    pub fn read_sensor(&mut self) -> Result<(), ICMError> {
        let mut buffer: [u8; 15] = [0u8; 15];
        self.read_registers(abs::IMU_OUT, &mut buffer)?;
        // self.accel_measurement.counts[0] = (((buffer[0] as u16) << 8) | (buffer[1] as u16)) as i16;
        self.accel_measurement.counts[0] = read_hl!(buffer, 0usize);
        self.accel_measurement.counts[1] = read_hl!(buffer, 2usize);
        self.accel_measurement.counts[2] = read_hl!(buffer, 4usize);
        let temp_counts: i16 = read_hl!(buffer, 6usize);
        self.gyro_measurement.counts[0] = read_hl!(buffer, 8usize);
        self.gyro_measurement.counts[1] = read_hl!(buffer, 10usize);
        self.gyro_measurement.counts[2] = read_hl!(buffer, 12usize);

        let acc_counts = &self.accel_measurement.counts;
        let acc_values = &mut self.accel_measurement.values;
        let acc_calib = &self.accel_measurement.calibration;
        acc_values[0] =
            (turn!(abs::tX, acc_counts) * acc_calib.scale - acc_calib.B[0] * acc_calib.S[0]) as f64;
        acc_values[1] =
            (turn!(abs::tY, acc_counts) * acc_calib.scale - acc_calib.B[1] * acc_calib.S[1]) as f64;
        acc_values[2] =
            (turn!(abs::tZ, acc_counts) * acc_calib.scale - acc_calib.B[2] * acc_calib.S[2]) as f64;

        self.temp_measurement =
            (((temp_counts as f64) - abs::temp_offset) / abs::temp_scale) + abs::temp_offset;

        let gyro_counts = &self.gyro_measurement.counts;
        let gyro_values = &mut self.gyro_measurement.values;
        let gyro_calib = &self.gyro_measurement.calibration;
        gyro_values[0] = (turn!(abs::tX, gyro_counts) * gyro_calib.scale - gyro_calib.B[0]) as f64;
        gyro_values[1] = (turn!(abs::tY, gyro_counts) * gyro_calib.scale - gyro_calib.B[1]) as f64;
        gyro_values[2] = (turn!(abs::tZ, gyro_counts) * gyro_calib.scale - gyro_calib.B[2]) as f64;

        Ok(())
    }

    pub fn calibrate_gyro<D>(&mut self, delay: &mut D) -> Result<(), ICMError>
    where
        D: DelayMs<u16>,
    {
        let old_gyro_range = self.gyro_range.clone();
        self.set_gyro_range(abs::GyroConfig::GYRO_RANGE_250DPS)?;

        let old_bandwidth = self.bandwidth.clone();
        self.set_dlpf_bandwidth(DLPFBandwidth::DLPF_BANDWIDTH_21HZ)?;

        let old_srd = self.srd.clone();
        self.set_srd(19u8)?;

        self.gyro_measurement.calibration.num_samples = 100;
        self.gyro_measurement.calibration.BD[0] = 0f64;
        self.gyro_measurement.calibration.BD[1] = 0f64;
        self.gyro_measurement.calibration.BD[2] = 0f64;
        for _ in 0..(self.gyro_measurement.calibration.num_samples) {
            self.read_sensor()?;
            let calib = &mut self.gyro_measurement.calibration;
            let values = &self.gyro_measurement.values;
            calib.BD[0] += (values[0] + calib.B[0]) / (calib.num_samples as f64);
            calib.BD[1] += (values[1] + calib.B[1]) / (calib.num_samples as f64);
            calib.BD[2] += (values[2] + calib.B[2]) / (calib.num_samples as f64);
            delay.delay_ms(20);
        }

        let calib = &mut self.gyro_measurement.calibration;
        calib.B[0] = calib.BD[0];
        calib.B[1] = calib.BD[1];
        calib.B[2] = calib.BD[2];

        self.set_gyro_range(old_gyro_range)?;
        self.set_dlpf_bandwidth(old_bandwidth)?;
        self.set_srd(old_srd)?;
        Ok(())
    }

    // TODO add function to calibrate accelerometer
    pub fn calibrate_accel<D>(&mut self, delay: &mut D) -> Result<(), ICMError>
    where
        D: DelayMs<u16>
    {
        let old_accel_range = self.accel_range.clone();
        self.set_accel_range(abs::AccelConfig::ACCEL_RANGE_2G)?;

        let old_bandwidth = self.bandwidth.clone();
        self.set_dlpf_bandwidth(DLPFBandwidth::DLPF_BANDWIDTH_21HZ)?;

        let old_srd = self.srd.clone();
        self.set_srd(19u8)?;

        let calib = &mut self.accel_measurement.calibration;
        calib.num_samples = 100;

        calib.BD[0] = 0f64;
        calib.BD[1] = 0f64;
        calib.BD[2] = 0f64;
        drop(calib);
        for _ in 0..(self.accel_measurement.calibration.num_samples) {
            self.read_sensor()?;
            let calib = &mut self.accel_measurement.calibration;
            let values = &self.accel_measurement.values;
            calib.BD[0] += (values[0] / calib.S[0] + calib.B[0])/(calib.num_samples as f64);
            calib.BD[1] += (values[1] / calib.S[1] + calib.B[1])/(calib.num_samples as f64);
            calib.BD[2] += (values[2] / calib.S[2] + calib.B[2])/(calib.num_samples as f64);
            delay.delay_ms(20);
        }

        let calib = &mut self.accel_measurement.calibration;

        if calib.BD[0] > 9.0 {
            calib.max[0] = calib.BD[0];
        }
        if calib.BD[1] > 9.0 {
            calib.max[1] = calib.BD[1];
        }
        if calib.BD[2] > 9.0 {
            calib.max[2] = calib.BD[2];
        }

        if calib.BD[0] < -9.0 {
            calib.min[0] = calib.BD[0];
        }
        if calib.BD[1] < -9.0 {
            calib.min[1] = calib.BD[1];
        }
        if calib.BD[2] < -9.0 {
            calib.min[2] = calib.BD[2];
        }

        if ((calib.min[0] > 9.0) || (calib.min[0] < -9.0)) &&
            ((calib.max[0] > 9.0) || (calib.max[0]) < -9.0) {
            calib.B[0] = (calib.min[0] + calib.max[0]) / 2.0;
            calib.S[0] = abs::g / calib.B[0];
        }
        if ((calib.min[1] > 9.0) || (calib.min[1] < -9.0)) &&
            ((calib.max[1] > 9.0) || (calib.max[1]) < -9.0) {
            calib.B[1] = (calib.min[1] + calib.max[1]) / 2.0;
            calib.S[1] = abs::g / calib.B[1];
        }
        if ((calib.min[2] > 9.0) || (calib.min[2] < -9.0)) &&
            ((calib.max[2] > 9.0) || (calib.max[2]) < -9.0) {
            calib.B[2] = (calib.min[2] + calib.max[2]) / 2.0;
            calib.S[2] = abs::g / calib.B[2];
        }

        self.set_accel_range(old_accel_range)?;
        self.set_dlpf_bandwidth(old_bandwidth)?;
        self.set_srd(old_srd)?;
        Ok(())
    }



    pub fn whoami(&mut self) -> Result<u8, ICMError> {
        let whoami_addr: u8 = abs::WHO_AM_I;
        let mut id: u8 = 0x00;
        self.read_reg(whoami_addr, &mut id)?;
        Ok(id)
    }
}

/*
2. TODO add functions to get the values measured
3. TODO add functionality to preload calibration values
*/
