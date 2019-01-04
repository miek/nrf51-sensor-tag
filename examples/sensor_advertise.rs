#![no_main]
#![no_std]

extern crate cortex_m_rt;
extern crate byteorder;
extern crate panic_halt;

extern crate nrf51_sensor_tag;

use nrf51_sensor_tag::{cortex_m, FICR, RADIO};
use nrf51_sensor_tag::hal::delay::Delay;
use nrf51_sensor_tag::hal::i2c::I2c;
use nrf51_sensor_tag::hal::prelude::*;

use byteorder::{ByteOrder, BigEndian, LittleEndian};

use cortex_m::peripheral::Peripherals;
use cortex_m_rt::entry;

extern crate bmp180;
use bmp180::{BMP180, Oversampling};

extern crate cortex_m_semihosting;
use cortex_m_semihosting::hio;
use core::fmt::Write;

const MAX_PAYLOAD: usize = 254 - (1 + 1 + 6 + 1);

#[repr(C, packed)]
struct Packet {
    header: u8,
    length: u8,
    adv_a: [u8; 6],
    payload_length: u8,
    payload: [u8; MAX_PAYLOAD],
}

struct BLEAdvertiser {
    packet: Packet,
    radio: RADIO,
}

impl BLEAdvertiser {
    fn new(radio: RADIO, ficr: &FICR) -> BLEAdvertiser {
        let mut packet = Packet {
            header: 0x42,
            length: 0x16,
            adv_a: [0u8; 6],
            payload_length: 0,
            payload: [0; MAX_PAYLOAD],
        };

        LittleEndian::write_u16(&mut packet.adv_a[4..6], ficr.deviceid[1].read().bits() as u16 | 0xc000);
        LittleEndian::write_u32(&mut packet.adv_a[0..4], ficr.deviceid[0].read().bits());

        let addr: u32 = 0x8e89bed6;

        radio.shorts.write(|w| w.ready_start().enabled().end_disable().enabled());
        radio.frequency.write(|w| unsafe { w.bits(2) });
        radio.mode.write(|w| w.mode().ble_1mbit());
        radio.pcnf0.write(|w| unsafe {
            w.s0len().bit(true)
             .lflen().bits(8)
             .s1len().bits(0)
        });
        radio.pcnf1.write(|w| unsafe {
            w.maxlen().bits(0xFF)
             .balen().bits(3)
             .whiteen().enabled()
        });
        radio.base0.write(|w| unsafe { w.bits((addr << 8) & 0xFFFFFF00) });
        radio.prefix0.write(|w| unsafe { w.bits(addr >> 24) });
        radio.crccnf.write(|w| w.skipaddr().skip().len().three());
        radio.crcinit.write(|w| unsafe { w.bits(0x555555) });
        radio.crcpoly.write(|w| unsafe { w.bits(0x100065B)});

        radio.datawhiteiv.write(|w| unsafe { w.bits(37) });
        BLEAdvertiser{ packet, radio }        
    }

    fn advertise(&mut self, payload: &[u8]) -> () {
        let payload_len = payload.len();
        self.packet.length = payload_len as u8 + 7;
        self.packet.payload_length = payload_len as u8; 
        self.packet.payload[..payload_len].clone_from_slice(payload);
        self.radio.packetptr.write(|w| unsafe { w.bits(&self.packet as *const _ as u32) });
        self.radio.tasks_txen.write(|w| unsafe { w.bits(1) });
    }

}

fn ruuvi_v3_temp(temp: i32) -> (u8, u8) {
    let temp_int = temp / 10;
    let temp_hundredths = (temp % 10) * 10;
    let temp_int = if temp_int < 0 {
        (temp_int * -1) | 0x80
    } else {
        temp_int
    };
    (temp_int as u8, temp_hundredths as u8)
}

#[entry]
fn main() -> ! {
    if let (Some(p), Some(_cp)) = (nrf51_sensor_tag::Peripherals::take(), Peripherals::take()) {
        let mut stdout = match hio::hstdout() {
            Ok(fd) => fd,
            Err(()) => loop { },
        };

        p.CLOCK.tasks_hfclkstart.write(|w| unsafe { w.bits(1) });

        let mut ble = BLEAdvertiser::new(p.RADIO, &p.FICR);

        let mut delay = Delay::new(p.TIMER0);

        let gpio = p.GPIO.split();
        let scl = gpio.pin10.into_open_drain_input().downgrade();
        let sda = gpio.pin9.into_open_drain_input().downgrade();
        let i2c = I2c::i2c1(p.TWI1, sda, scl, p.PPI);

        let mut bmp180 = BMP180::new(i2c, delay).unwrap();

        let mut payload = [0u8; 17];
        payload[0] = 0xff; // Manuf. specific data
        LittleEndian::write_u16(&mut payload[1..3], 0x0499); // Ruuvi
        payload[3] = 3; // Data format 3

        loop {
            let (temp, pressure) = bmp180.temperature_and_pressure(Oversampling::O1).unwrap();
            let temp_conv = ruuvi_v3_temp(temp);
            payload[5] = temp_conv.0;
            payload[6] = temp_conv.1;
            BigEndian::write_u16(&mut payload[7..9], (pressure - 50000) as u16);
            ble.advertise(&payload);
        }
    }

    loop {
        continue;
    }
}

