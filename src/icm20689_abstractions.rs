#![allow(dead_code)]
macro_rules! register {
    ($name:ident, $address:expr) => {
        pub const $name: u8 = $address;
    };
}

macro_rules! register_value {
    ($name:ident, $address:expr) => {
        pub const $name: u8 = $address;
    };
}

pub const g: f64 = 9.807;
pub const d2r: f64 = 3.14159265359f64/180.0f64;
pub const r2d: f64 = 1f64/d2r;
pub const I2C_RATE: u32 = 400_000;

register!(WHO_AM_I, 0x75);

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum GyroConfig {
    GYRO_RANGE_250DPS = 0x00,
    GYRO_RANGE_500DPS = 0x08,
    GYRO_RANGE_1000DPS = 0x10,
    GYRO_RANGE_2000DPS = 0x18,
}

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum AccelConfig {
    ACCEL_RANGE_2G = 0x00,
    ACCEL_RANGE_4G = 0x08,
    ACCEL_RANGE_8G = 0x10,
    ACCEL_RANGE_16G = 0x18,
}

pub mod power_management {
    register!(PWR_MGMT_1, 0x6b);
    register!(PWR_MGMT_2, 0x6c);
}

register_value!(CLOCK_SEL_PLL, 0x01); // to be written to PWR_MGMT_1
register_value!(PWR_RESET, 0x80); // to be written to PWR_MGMT_1
register_value!(SENSOR_EN, 0x00); // to be written to PWR_MGMT_2

pub mod configuration {
    register!(CONFIG, 0x1a);
    register!(GYRO_CONFIG, 0x1b);
    register!(ACCEL_CONFIG, 0x1c);
    register!(ACCEL_CONFIG_2, 0x1d);
}

pub mod gyroscope_measurements {
    register!(GYRO_XOUT_H, 0x43);
    register!(GYRO_XOUT_L, 0x44);
    register!(GYRO_YOUT_H, 0x45);
    register!(GYRO_YOUT_L, 0x46);
    register!(GYRO_ZOUT_H, 0x47);
    register!(GYRO_ZOUT_L, 0x48);
}

pub mod accelerometer_measurements {
    register!(ACCEL_XOUT_H, 0x3b);
    register!(ACCEL_XOUT_L, 0x3c);
    register!(ACCEL_YOUT_H, 0x3d);
    register!(ACCEL_YOUT_L, 0x3e);
    register!(ACCEL_ZOUT_H, 0x3f);
    register!(ACCEL_ZOUT_L, 0x40);
}
