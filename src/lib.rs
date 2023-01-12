#![no_std]
#![allow(dead_code)]
use embedded_hal::blocking::{delay::DelayMs, i2c};

mod icm20689_abstractions;
use icm20689_abstractions::accelerometer_measurements;

pub struct ICM20689<I>
where
    I: i2c::Write + i2c::Read,
{
    i2c: I,
    address: u8,
}

impl<I> ICM20689<I> where I: i2c::Write + i2c::Read {
    pub fn new(i2c: I, address: u8) -> Self {
        Self {
            i2c,
            address,
        }
    }

    pub fn init(&self) {
        // TODO check documentation pg 48
        // set Power_management_1(0x6B) to choose the best clock source
        // reset the icm20689
        // wait for reset (delay)
        // set power mgmt_1 to choose the best clock source again
        // enable the sensor (write 0x00 to pwr_mgmt_2 (0x6C))

        // select accelerometer range
        // select gyroscope range
        // select accelerometer and gyroscope scale based on the selected range

        // TODO study the low pass filter
    }
}
