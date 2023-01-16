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
pub const d2r: f64 = 3.14159265359f64 / 180.0f64; // convert degrees to radian
pub const r2d: f64 = 1f64 / d2r; // convert radian to degrees
pub const I2C_RATE: u32 = 400_000;

register!(WHO_AM_I, 0x75);
register!(SMPLRT_DIV, 0x19);

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum GyroConfig {
    GYRO_RANGE_250DPS = 0x00,
    GYRO_RANGE_500DPS = 0x08,
    GYRO_RANGE_1000DPS = 0x10,
    GYRO_RANGE_2000DPS = 0x18,
}

register_value!(GYRO_FCHOICE_B_8173HZ, 0x01);
register_value!(GYRO_FCHoice_b_3281HZ, 0x10);

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum AccelConfig {
    ACCEL_RANGE_2G = 0x00,
    ACCEL_RANGE_4G = 0x08,
    ACCEL_RANGE_8G = 0x10,
    ACCEL_RANGE_16G = 0x18,
}

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum AccelDLPFBW {
    ACCEL_DLPF_218HZ = 0x01,
    ACCEL_DLPF_99HZ = 0x02,
    ACCEL_DLPF_45HZ = 0x03,
    ACCEL_DLPF_21HZ = 0x04,
    ACCEL_DLPF_10HZ = 0x05,
    ACCEL_DLPF_5HZ = 0x06,
    ACCEL_DLPF_420HZ = 0x07,
    ACCEL_DLPF_1046HZ = 0x08,
}

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum GyroDLPFBW {
    GYRO_DLPF_250HZ = 0x00,
    GYRO_DLPF_176HZ = 0x01,
    GYRO_DLPF_92HZ = 0x02,
    GYRO_DLPF_41HZ = 0x03,
    GYRO_DLPF_20HZ = 0x04,
    GYRO_DLPF_10HZ = 0x05,
    GYRO_DLPF_5HZ = 0x06,
}

#[derive(Clone, Copy)]
pub enum DLPFBandwidth {
    DLPF_BANDWIDTH_MAX,
    DLPF_BANDWIDTH_218HZ,
    DLPF_BANDWIDTH_99HZ,
    DLPF_BANDWIDTH_45HZ,
    DLPF_BANDWIDTH_21HZ,
    DLPF_BANDWIDTH_10HZ,
    DLPF_BANDWIDTH_5HZ,
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

// can be used to read the entire IMU
register!(IMU_OUT, accelerometer_measurements::ACCEL_XOUT_H);
