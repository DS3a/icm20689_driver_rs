#![no_std]
#![allow(dead_code)]
use embedded_hal::blocking::{delay::DelayMs, i2c};

mod icm20689_abstractions;
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
    gyro_scale: f64,
}

impl<I> ICM20689<I>
where
    I: i2c::Write + i2c::Read,
{
    pub fn new(i2c: I, icm_address: u8) -> Self {
        Self {
            i2c,
            icm_address,
            accel_scale: 0f64,
            gyro_scale: 0f64,
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
            self.set_accel_range(abs::AccelConfig::ACCEL_RANGE_16G)?;
            self.set_gyro_range(abs::GyroConfig::GYRO_RANGE_2000DPS)?;

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
        self.write_reg(
            abs::configuration::ACCEL_CONFIG,
            range as u8
        )?;
        self.accel_scale = (abs::g / 32767.5f64) * match range {
            abs::AccelConfig::ACCEL_RANGE_2G => 2f64,
            abs::AccelConfig::ACCEL_RANGE_4G => 4f64,
            abs::AccelConfig::ACCEL_RANGE_8G => 8f64,
            abs::AccelConfig::ACCEL_RANGE_16G => 16f64,
        };
        Ok(())
    }

    pub fn set_gyro_range(&mut self, range: abs::GyroConfig) -> Result<(), ICMError> {
        self.write_reg(
            abs::configuration::GYRO_CONFIG,
            range as u8
        )?;
        self.gyro_scale = (abs::d2r / 32767.5f64) * match range {
            abs::GyroConfig::GYRO_RANGE_250DPS => 250f64,
            abs::GyroConfig::GYRO_RANGE_500DPS => 500f64,
            abs::GyroConfig::GYRO_RANGE_1000DPS => 1000f64,
            abs::GyroConfig::GYRO_RANGE_2000DPS => 2000f64,
        };
        Ok(())
    }

    fn write_reg(&mut self, addr: u8, data: u8) -> Result<(), ICMError> {
        if let Ok(()) = self.i2c.write(self.icm_address, &[addr, data]) {
            return Ok(());
        } else {
            Err(ICMError::I2cWriteError)
        }
    }

    fn read_reg(&mut self, addr: u8, buf: &mut u8) -> Result<(), ICMError> {
        if let Ok(()) = self.i2c.read(self.icm_address, &mut [addr, *buf]) {
            Ok(())
        } else {
            Err(ICMError::I2cReadError)
        }
    }

    pub fn whoami(&mut self) -> Result<u8, ICMError> {
        let whoami_addr: u8 = abs::WHO_AM_I;
        let mut id: u8 = 0x00;
        self.read_reg(whoami_addr, &mut id)?;
        Ok(id)
    }
}


// TODO check out the reading thingi
// TODO add measure functions
