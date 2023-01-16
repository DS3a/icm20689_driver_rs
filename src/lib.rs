#![no_std]
#![allow(dead_code)]
use embedded_hal::blocking::{delay::DelayMs, i2c};

mod icm20689_abstractions;
pub use abs::{AccelConfig, GyroConfig};
use icm20689_abstractions as abs;

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

    accel_scale: f64,
    accel_range: AccelConfig,

    gyro_scale: f64,
    gyro_range: GyroConfig,
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

    pub fn new(i2c: I, icm_address: u8) -> Self {
        Self {
            i2c,
            icm_address,
            accel_scale: 0f64, // will be set to the actual scale when init() is called
            accel_range: AccelConfig::ACCEL_RANGE_16G,

            gyro_scale: 0f64,
            gyro_range: GyroConfig::GYRO_RANGE_2000DPS,
        }
    }

    pub fn init<D>(&mut self, delay: &mut D) -> Result<(), ICMError>
    where
        D: DelayMs<u16>,
    {
        delay.delay_ms(80);
        // TODO check documentation pg 48

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
                    return Err(ICMError::ICMIDError);
                }
            } else {
                return Err(ICMError::I2cReadError);
            }

            // enable the sensor
            self.write_reg(abs::power_management::PWR_MGMT_2, abs::SENSOR_EN)?;

            // set accelerometer and gyroscope ranges
            self.set_accel_range(self.accel_range)?;
            self.set_gyro_range(self.gyro_range)?;

            // set LPF for accelerometer to 0x8 for now
            // TODO tune these bandwidths experimentally check page 39 to 41 in documentation
            self.write_reg(abs::configuration::ACCEL_CONFIG_2, 0x8)?;
            self.write_reg(abs::configuration::CONFIG, 0x0)?;
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
        self.accel_scale = (abs::g / 32767.5f64)
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
        self.gyro_scale = (abs::d2r / 32767.5f64)
            * match range {
                abs::GyroConfig::GYRO_RANGE_250DPS => 250f64,
                abs::GyroConfig::GYRO_RANGE_500DPS => 500f64,
                abs::GyroConfig::GYRO_RANGE_1000DPS => 1000f64,
                abs::GyroConfig::GYRO_RANGE_2000DPS => 2000f64,
            };
        Ok(())
    }

    pub fn set_DLPF_bandwidth(&mut self, bandwidth: abs::DLPFBandwidth) -> Result<(), ICMError> {
        match (bandwidth) {
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

    // TODO add function to read accelerometer
    // TODO add function to read gyroscope
    // TODO add function to read temperature

    pub fn calibrate_gyro(&mut self) -> Result<(), ICMError> {
        self.write_reg(
            abs::configuration::GYRO_CONFIG,
            abs::GyroConfig::GYRO_RANGE_250DPS as u8,
        ); // temporarily

        Ok(())
    }

    fn whoami(&mut self) -> Result<u8, ICMError> {
        let whoami_addr: u8 = abs::WHO_AM_I;
        let mut id: u8 = 0x00;
        self.read_reg(whoami_addr, &mut id)?;
        Ok(id)
    }
}

/*
 TODO add measure functions
 1. TODO add calibration functions to find bias and scale
 2. TODO add transformation matrix that can be set by user
 3. TODO read the accelerometer and gyro output registers, process it blah blah
*/
