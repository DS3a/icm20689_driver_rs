use embedded_hal::blocking::delay::DelayMs;
use esp_idf_hal::delay::FreeRtos;

use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
// use embedded_hal::delay::DelayUs;
use esp_idf_hal::i2c::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use icm20689_driver_rs::{ICMError, ICM20689};

const ICM20689_ADDR: u8 = 0x68;

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();

    let i2c = peripherals.i2c0;
    let sda = peripherals.pins.gpio6;
    let scl = peripherals.pins.gpio7;

    println!("starting the i2c configuration");
    let config = I2cConfig::new().baudrate(400.kHz().into());
    let mut i2c = I2cDriver::new(i2c, sda, scl, &config).unwrap();

    let mut icm = ICM20689::new(i2c, ICM20689_ADDR);

    let mut delay = FreeRtos {};

    println!("initializing the ICM");
    icm.init(&mut delay);
    loop {
        match icm.whoami() {
            Ok(id) => {
                println!("got the id {:?}", id);
                println!("calibrating gyroscope");
                match icm.calibrate_gyro(&mut delay) {
                    Ok(()) => {
                        println!("The gyroscope was calibrated successfully");
                        println!("calibrating accelerometer now");
                        match icm.calibrate_accel(&mut delay) {
                            Ok(()) => {
                                println!("accelerometer calibrated successfully");
                                println!("setting the bw to the lowest gain");
                                icm.set_dlpf_bandwidth(icm20689_driver_rs::DLPFBandwidth::DLPF_BANDWIDTH_5HZ).unwrap();
                                loop {
                                    println!("now proceeding to read sensor");
                                    match icm.read_sensor() {
                                        Ok(()) => {
                                            println!("sensor read successfully, the gyro values are : ");
                                            icm.read_gyroscope();
                                            println!("{:?}", icm.gyro_measurement.values);
                                            println!("the accelerometer values are : ");
                                            println!("{:?}", icm.accel_measurement.values);
                                            delay.delay_ms(500u16);
                                        },
                                        Err(err) => {
                                            println!("unable to read sensor due to {:?}\nresetting and trying again", err);
                                            break;
                                        }
                                    }
                                }
                            },
                            Err(err) => println!("unable to calibrate accelerometer due to {:?}", err)
                        }
                   },
                    Err(err) => println!("The gyroscope was unable to calibrate due to {:?}", err)
                }
            },
            Err(err) => {
                println!("unable to get the ID from the icm due to {:?}", err);
            }
        }
        println!("Hello, world!");
        delay.delay_ms(1000 as u16);
    }
}
