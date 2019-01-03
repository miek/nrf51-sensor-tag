#![no_main]
#![no_std]

extern crate cortex_m_rt;
extern crate panic_halt;
extern crate nrf51_sensor_tag as sensortag;

use cortex_m_rt::entry;

use sensortag::hal::delay::Delay;
use sensortag::hal::prelude::*;
use sensortag::hal::rng;

#[repr(C, packed)]
struct Packet {
    header: u8,
    length: u8,
    adv_a: [u8; 6],
    payload: [u8; 16],
}

#[entry]
fn main() -> ! {
    if let Some(p) = sensortag::Peripherals::take() {
        let mut delay = Delay::new(p.TIMER0);

        p.CLOCK.tasks_hfclkstart.write(|w| unsafe { w.bits(1) });

        let mut rng = rng::Rng::new(p.RNG);
        let mut packet = Packet {
            header: 0x42,
            length: 0x16,
            adv_a: [0xEF, 0xFF, 0xC0, 0xAA, 0x18, 0x00],
            payload: [0x0F, 0x09, 0x79, 0x61, 0x79, 0x20, 0x62, 0x6c, 0x75, 0x65, 0x74, 0x6f, 0x6f, 0x74, 0x68, 0x21],
         };

        let addr: u32 = 0x8e89bed6;

        p.RADIO.shorts.write(|w| w.ready_start().enabled().end_disable().enabled());
        p.RADIO.packetptr.write(|w| unsafe { w.bits(&packet as *const _ as u32) });
        p.RADIO.frequency.write(|w| unsafe { w.bits(2) });
        p.RADIO.mode.write(|w| w.mode().ble_1mbit());
        p.RADIO.pcnf0.write(|w| unsafe {
            w.s0len().bit(true)
             .lflen().bits(8)
             .s1len().bits(0)
        });
        p.RADIO.pcnf1.write(|w| unsafe {
            w.maxlen().bits(0xFF)
             .balen().bits(3)
             .whiteen().enabled()
        });
        p.RADIO.base0.write(|w| unsafe { w.bits((addr << 8) & 0xFFFFFF00) });
        p.RADIO.prefix0.write(|w| unsafe { w.bits(addr >> 24) });
        p.RADIO.crccnf.write(|w| w.skipaddr().skip().len().three());
        p.RADIO.crcinit.write(|w| unsafe { w.bits(0x555555) });
        p.RADIO.crcpoly.write(|w| unsafe { w.bits(0x100065B)});

        p.RADIO.datawhiteiv.write(|w| unsafe { w.bits(37) });
        

        loop {
            p.RADIO.tasks_txen.write(|w| unsafe { w.bits(1) });
            delay.delay_ms(1000_u16);
            rng.read(&mut packet.adv_a).ok();
        }
    }

    loop {}
}
