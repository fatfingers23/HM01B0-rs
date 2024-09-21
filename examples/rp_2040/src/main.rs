#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::i2c::{self, Config, InterruptHandler};
use embassy_rp::peripherals::I2C0;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::Pio;
use embassy_time::Duration;
use hm01b0::{DataBits, PictureSize, HM01B0};
use pio::WaitSource::GPIO;
use pio::{InSource, JmpCondition, MovDestination, MovOperation, MovSource, SetDestination};
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
    let mut i2c = i2c::I2c::new_async(p.I2C0, scl, sda, Irqs, Config::default());
    // let mut i2c = i2c::I2c::new_blocking(p.I2C0, scl, sda, Config::default());

    let _pio = Pio::new(p.PIO0, PioIrqs);

    // let mut c: embassy_rp::pwm::Config = get_mclk_pwm_config();
    // let mut pwm = Pwm::new_output_b(p.PWM_SLICE4, p.PIN_10, c.clone());

    info!("Before init");
    let data_bits = DataBits::Bits1;

    let hm01b0 = HM01B0::new(&mut i2c, PictureSize::Size320x320, data_bits, None, None).await;
    info!("After init");

    info!("Before Pio");
    let num_border_px = 2;
    let vsync_pin = 6;
    let hsync_pin = 7;
    let pclk_pin = 8;
    let data_pin_base = 9;

    let mut assembler = pio::Assembler::<34>::new();
    let mut wrap_target = assembler.label();
    let mut wrap_source = assembler.label();

    let mut ins_4_target = assembler.label();
    let mut ins_4_source = assembler.label();

    let mut ins_10_target = assembler.label();
    let mut ins_10_source = assembler.label();

    let mut ins_13_target = assembler.label();

    assembler.pull(false, true);
    assembler.wait(0, GPIO, vsync_pin, false);
    assembler.wait(1, GPIO, vsync_pin, false);
    assembler.set(SetDestination::Y, num_border_px - 1);
    assembler.bind(&mut ins_4_target);
    assembler.wait(1, GPIO, hsync_pin, false);
    assembler.wait(0, GPIO, hsync_pin, false);
    assembler.jmp(JmpCondition::YDecNonZero, &mut ins_4_target);
    assembler.bind(&mut wrap_target);
    assembler.mov(MovDestination::X, MovOperation::None, MovSource::OSR);
    assembler.bind(&mut ins_10_target);
    assembler.wait(1, GPIO, hsync_pin, false);
    assembler.set(
        SetDestination::Y,
        num_border_px * hm01b0.num_pclk_per_px - 1,
    );
    assembler.wait(1, GPIO, pclk_pin, false);
    assembler.wait(0, GPIO, pclk_pin, false);
    assembler.jmp(JmpCondition::YDecNonZero, &mut ins_10_target);
    assembler.jmp(JmpCondition::YDecNonZero, &mut ins_13_target);
    assembler.wait(1, GPIO, pclk_pin, false);
    assembler.r#in(InSource::PINS, data_bits.to_u8());
    assembler.wait(0, GPIO, pclk_pin, false);
    assembler.jmp(JmpCondition::XDecNonZero, &mut ins_13_target);
    assembler.wait(0, GPIO, hsync_pin, false);
    assembler.bind(&mut wrap_source);

    info!("After Pio");

    let delay = Duration::from_secs(1);

    // loop {
    //     info!("I work");
    //     Timer::after(delay).await;
    // }
}
