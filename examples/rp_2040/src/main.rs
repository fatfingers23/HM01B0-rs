#![no_std]
#![no_main]

use cortex_m::prelude::_embedded_hal_blocking_i2c_Write;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::i2c::{self, Config, InterruptHandler};
use embassy_rp::peripherals::I2C0;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{Common, Irq, Pio, PioPin, ShiftDirection, StateMachine};
use embassy_time::{Duration, Timer};
use hm01b0::{DataBits, PictureSize, HM01B0};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C0_IRQ => InterruptHandler<I2C0>;
});

bind_interrupts!(struct PioIrqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    //Set up I2C to set settings for the camera
    let sda = p.PIN_4;
    let scl = p.PIN_5;
    // let mut i2c = i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, Config::default());
    let mut i2c = i2c::I2c::new_blocking(p.I2C0, scl, sda, Config::default());

    let _pio = Pio::new(p.PIO0, PioIrqs);

    info!("Before init");
    // HM01B0::test();
    let hm01b0 = HM01B0::new(&mut i2c, PictureSize::Size320x320, DataBits::Bits1);
    // info!("{:?}", hm01b0);
    info!("After init");
    let delay = Duration::from_secs(1);

    // loop {
    //     info!("I work");
    //     Timer::after(delay).await;
    // }
}



