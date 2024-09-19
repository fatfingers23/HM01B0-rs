#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::pac::spi::Spi;
use embassy_time::{Duration, Timer};
use hm01b0::HM01B0;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_14;
    let scl = p.PIN_15;
    let hm01b0 = HM01B0::new(p.I2C0, sda, scl);

    let delay = Duration::from_secs(1);

    loop {
        info!("I work");
        Timer::after(delay).await;
    }
}
