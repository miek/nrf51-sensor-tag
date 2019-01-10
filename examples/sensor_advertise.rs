#![no_main]
#![no_std]

extern crate cortex_m_rt;
extern crate byteorder;
extern crate panic_halt;

extern crate nrf51_sensor_tag;

use nrf51_sensor_tag::cortex_m;
use nrf51_sensor_tag::nrf51::{interrupt, ADC, FICR, RADIO};
use nrf51_sensor_tag::hal::delay::Delay;
use nrf51_sensor_tag::hal::i2c::I2c;
use nrf51_sensor_tag::hal::prelude::*;

use byteorder::{ByteOrder, BigEndian, LittleEndian};

use cortex_m::peripheral::Peripherals;
use cortex_m_rt::entry;

extern crate bmp180;
use bmp180::{BMP180, Oversampling};

struct BatteryMonitor {
    adc: ADC,
}

impl BatteryMonitor {
    fn new(adc: ADC) -> BatteryMonitor {
        adc.config.write(|w|
            w.res()._10bit()
             .inpsel().supply_one_third_prescaling()
             .refsel().vbg()
             .psel().disabled()
             .extrefsel().none()
        );
        BatteryMonitor{ adc }
    }

    fn measure(&mut self) -> u16 {
        self.adc.events_end.write(|w| w.event().clear());
        self.adc.tasks_start.write(|w| w.task().trigger());
        while self.adc.events_end.read().event().is_clear() {}
        let result = self.adc.result.read().bits();
        ((result * 3600) / 1024) as u16
    }
}

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

        if ficr.overrideen.read().ble_1mbit().bit_is_set() {
            radio.override0.write(|w| unsafe { w.bits(ficr.ble_1mbit[0].read().bits()) });
            radio.override1.write(|w| unsafe { w.bits(ficr.ble_1mbit[1].read().bits()) });
            radio.override2.write(|w| unsafe { w.bits(ficr.ble_1mbit[2].read().bits()) });
            radio.override3.write(|w| unsafe { w.bits(ficr.ble_1mbit[3].read().bits()) });
            radio.override4.write(|w| unsafe { w.bits(ficr.ble_1mbit[4].read().bits()) });
        }

        BLEAdvertiser{ packet, radio }        
    }

    fn set_channel(&mut self, channel: u8) -> () {
        let freq = match channel {
            37 => Some(2),
            38 => Some(26),
            39 => Some(80),
            _ => None,
        };
        if let Some(freq) = freq {
            self.radio.datawhiteiv.write(|w| unsafe { w.bits(channel as u32) });
            self.radio.frequency.write(|w| unsafe { w.bits(freq) });
        };
    }

    fn advertise(&mut self, payload: &[u8]) -> () {
        let payload_len = payload.len();
        self.packet.length = payload_len as u8 + 7;
        self.packet.payload_length = payload_len as u8; 
        self.packet.payload[..payload_len].clone_from_slice(payload);
        self.radio.packetptr.write(|w| unsafe { w.bits(&self.packet as *const _ as u32) });
        self.radio.tasks_txen.write(|w| w.task().trigger());
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
    if let (Some(p), Some(mut cp)) = (nrf51_sensor_tag::Peripherals::take(), Peripherals::take()) {
        p.CLOCK.events_hfclkstarted.write(|w| w.event().clear());
        p.CLOCK.tasks_hfclkstart.write(|w| w.task().trigger() );
        while p.CLOCK.events_hfclkstarted.read().event().is_clear() {}

        p.CLOCK.lfclksrc.write(|w| w.src().xtal());
        p.CLOCK.events_lfclkstarted.write(|w| w.event().clear());
        p.CLOCK.tasks_lfclkstart.write(|w| w.task().trigger());
        while p.CLOCK.events_lfclkstarted.read().event().is_clear() {}

        let mut ble = BLEAdvertiser::new(p.RADIO, &p.FICR);

        let mut delay = Delay::new(p.TIMER0);

        let gpio = p.GPIO.split();
        let scl = gpio.pin10.into_open_drain_input().downgrade();
        let sda = gpio.pin9.into_open_drain_input().downgrade();
        let i2c = I2c::i2c1(p.TWI1, sda, scl);

        let mut bmp180 = BMP180::new(i2c, delay).unwrap();
        let mut bat = BatteryMonitor::new(p.ADC);

        let mut payload = [0u8; 17];
        payload[0] = 0xff; // Manuf. specific data
        LittleEndian::write_u16(&mut payload[1..3], 0x0499); // Ruuvi
        payload[3] = 3; // Data format 3

        let rtc = p.RTC0;
        rtc.cc[0].write(|w| unsafe { w.bits(100) }); // 1s @ 10ms tick rate
        rtc.evtenset.write(|w| w.compare0().set());
        rtc.intenset.write(|w| w.compare0().set());
        rtc.prescaler.write(|w| unsafe { w.bits(327) }); // ~10ms tick rate
        cp.NVIC.enable(nrf51_sensor_tag::Interrupt::RTC0);
        nrf51_sensor_tag::NVIC::unpend(nrf51_sensor_tag::Interrupt::RTC0);

        let mut led = gpio.pin17.into_push_pull_output();
        led.set_high();

        loop {
            for channel in 37..40 {
                let (temp, pressure) = bmp180.temperature_and_pressure(Oversampling::O1).unwrap();
                let temp_conv = ruuvi_v3_temp(temp);
                payload[5] = temp_conv.0;
                payload[6] = temp_conv.1;
                BigEndian::write_u16(&mut payload[7..9], (pressure - 50000) as u16);
                let millivolts = bat.measure();
                BigEndian::write_u16(&mut payload[15..17], millivolts);
                led.set_low();
                ble.set_channel(channel);
                ble.advertise(&payload);
                led.set_high();

                rtc.tasks_stop.write(|w| w.task().trigger());
                rtc.tasks_clear.write(|w| w.task().trigger());
                rtc.events_compare[0].write(|w| w.event().clear());
                rtc.tasks_start.write(|w| w.task().trigger());
                cortex_m::asm::wfe();
            }
        }
    }

    loop {
        continue;
    }
}

#[interrupt]
fn RTC0() {
    unsafe {
        let event_compare: *mut u32 = 0x4000B140 as *mut u32;
        *event_compare = 0;
    }
}
