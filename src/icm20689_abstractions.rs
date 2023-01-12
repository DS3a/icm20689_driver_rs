#![allow(dead_code)]
macro_rules! register {
    ($name:ident, $address:expr) => {
        pub const $name: u8 = $address;
    };
}

const g: f64 = 9.807;
const I2C_RATE: u32 = 400_000;

pub enum GyroConfig {
    GYRO_RANGE_250DPS = 0x00,
    GYRO_RANGE_500DPS = 0x08,
    GYRO_RANGE_1000DPS = 0x10,
    GYRO_RANGE_2000DPS = 0x18,
}

pub enum AccelConfig {
    ACCEL_RANGE_2G = 0x00,
    ACCEL_RANGE_4G = 0x08,
    ACCEL_RANGE_8G = 0x10,
    ACCEL_RANGE_16G = 0x18,
}

pub mod configuration {
    register!(GYRO_CONFIG, 0x1b);
    register!(ACCEL_CONFIG, 0x1c);
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
