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
use embedded_hal_1::i2c::{I2c, SevenBitAddress};
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

    //setup PIO to read data from the camera
    // embassy_rp::pio::Config

    let _pio = Pio::new(p.PIO0, PioIrqs);

    info!("Before init");
    hm01b0_read_reg16(&mut i2c, 0x0000, 0x24);
    info!("After init");
    let delay = Duration::from_secs(1);

    // loop {
    //     info!("I work");
    //     Timer::after(delay).await;
    // }
}

fn write_u16_be(bytes: &mut [u8; 2], value: u16) {
    bytes[0] = (value >> 8) as u8;   // Higher byte
    bytes[1] = (value & 0xFF) as u8; // Lower byte
}

fn read_u16_be(bytes: &[u8; 2]) -> u16 {
    ((bytes[0] as u16) << 8) | (bytes[1] as u16)
}


 fn hm01b0_read_reg16<I2C: I2c>(
    i2c: &mut I2C,
    address: u16,
    i2c_address: SevenBitAddress,
)  {
    let mut address_bytes = [0u8; 2];
    let mut result_bytes = [0xffu8; 2];

    // Manually write u16 to the byte array in little-endian format
    write_u16_be(&mut address_bytes, address);

    // Write the address to the sensor
    i2c.write(i2c_address, &address_bytes).unwrap();

    // Read 2 bytes from the sensor
    i2c.read(i2c_address, &mut result_bytes).unwrap();
    // Manually read the u16 value from the result bytes in little-endian format
    let result = read_u16_be(&result_bytes);
    info!("{:?}", result);

}

