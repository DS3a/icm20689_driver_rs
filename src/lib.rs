#![no_std]
#![allow(dead_code)]
use embedded_hal::blocking::{delay::DelayMs, i2c};

mod icm20689_abstractions;
use icm20689_abstractions as abs;

pub struct ICM20689<I>
where
    I: i2c::Write + i2c::Read,
{
    i2c: I,
    icm_address: u8,
}

impl<I> ICM20689<I>
where
    I: i2c::Write + i2c::Read,
{
    pub fn new(i2c: I, icm_address: u8) -> Self {
        Self { i2c, icm_address }
    }

    pub fn init<D>(&mut self, delay: &mut D) -> bool
    where
        D: DelayMs<u16>,
    {
        delay.delay_ms(80);
        // TODO check documentation pg 48
        if let Result::Ok(()) = self.i2c.write(self.icm_address, &[abs::power_management::PWR_MGMT_1, abs::CLOCK_SEL_PLL]) {

            // continue
            true
        } else {
            delay.delay_ms(40);
            self.init(delay)
        }

        // set Power_management_1(0x6B) to choose the best clock source
        //  reset the icm20689
        // wait for reset (delay)
        // set power mgmt_1 to choose the best clock source again
        // enable the sensor (write 0x00 to pwr_mgmt_2 (0x6C))

        // select accelerometer range
        // select gyroscope range
        // select accelerometer and gyroscope scale based on the selected range

        // TODO study the low pass filter
    }
}
