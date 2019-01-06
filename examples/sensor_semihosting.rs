#![no_main]
#![no_std]

extern crate cortex_m_rt;
extern crate panic_halt;

extern crate nrf51_sensor_tag;

use nrf51_sensor_tag::cortex_m;
use nrf51_sensor_tag::hal::delay::Delay;
use nrf51_sensor_tag::hal::i2c::I2c;
use nrf51_sensor_tag::hal::prelude::*;

use cortex_m::peripheral::Peripherals;
use cortex_m_rt::entry;

extern crate bmp180;
use bmp180::{BMP180, Oversampling};

extern crate cortex_m_semihosting;
use cortex_m_semihosting::hio;
use core::fmt::Write;

#[entry]
fn main() -> ! {
    if let (Some(p), Some(_cp)) = (nrf51_sensor_tag::Peripherals::take(), Peripherals::take()) {
        let mut stdout = match hio::hstdout() {
            Ok(fd) => fd,
            Err(()) => loop { },
        };

        let mut delay = Delay::new(p.TIMER0);

        let gpio = p.GPIO.split();
        let scl = gpio.pin10.into_open_drain_input().downgrade();
        let sda = gpio.pin9.into_open_drain_input().downgrade();
        let i2c = I2c::i2c1(p.TWI1, sda, scl);

        let mut bmp180 = BMP180::new(i2c, delay).unwrap();

        loop {
            let (temp, pressure) = bmp180.temperature_and_pressure(Oversampling::O1).unwrap();
            write!(stdout, "Temp: {} Pressure: {}\n", temp, pressure).unwrap();
        }
    }

    loop {
        continue;
    }
}
