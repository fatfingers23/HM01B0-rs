#![no_std]
#![no_main]

use defmt::export::u32;
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::gpio::Level;
use embassy_rp::i2c::{self, Config, InterruptHandler};
use embassy_rp::peripherals::I2C0;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{Direction, Pio, ShiftConfig, ShiftDirection};
use embassy_rp::{bind_interrupts, dma, Peripheral};
use embassy_time::{Duration, Timer};
use fixed::prelude::ToFixed;
use fixed_macro::types::U56F8;
use hm01b0::{DataBits, PictureSize, HM01B0};
use pio::WaitSource::GPIO;
use pio::{
    Assembler, InSource, JmpCondition, MovDestination, MovOperation, MovSource, SetDestination,
    WaitSource,
};
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

    // let mut c: embassy_rp::pwm::Config = get_mclk_pwm_config();
    // let mut pwm = Pwm::new_output_b(p.PWM_SLICE4, p.PIN_10, c.clone());

    info!("Before init");
    let data_bits = DataBits::Bits1;

    let mut hm01b0 = HM01B0::new(&mut i2c, PictureSize::Size160x120, data_bits, None, None).await;
    hm01b0.set_coarse_integration(2).await;
    info!("After init");

    info!("Before Pio");
    let num_border_px = 2;
    let vsync_pin = 6;
    let hsync_pin = 7;
    let pclk_pin = 8;
    let data_pin_base = 9;
    let num_pclk_per_px = 1; // Assuming this is defined somewhere in your context

    let pio = p.PIO0;
    let Pio {
        mut common,
        sm0: mut sm,
        ..
    } = Pio::new(pio, PioIrqs);

    let mut assembler = Assembler::<32>::new();
    let mut wrap_target = assembler.label();
    let mut wrap_source = assembler.label();

    let mut ins_4_target = assembler.label();
    let mut ins_10_target = assembler.label();
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
        (num_border_px * hm01b0.num_pclk_per_px - 1) as u8,
    );
    assembler.wait(1, GPIO, pclk_pin, false);
    assembler.wait(0, GPIO, pclk_pin, false);
    assembler.jmp(JmpCondition::YDecNonZero, &mut ins_10_target);
    assembler.bind(&mut ins_13_target);
    assembler.wait(1, GPIO, pclk_pin, false);
    assembler.r#in(InSource::PINS, data_bits.to_u8());
    assembler.wait(0, GPIO, pclk_pin, false);
    assembler.jmp(JmpCondition::XDecNonZero, &mut ins_13_target);
    assembler.wait(0, GPIO, hsync_pin, false);
    assembler.bind(&mut wrap_source);
    let pio_program = assembler.assemble_with_wrap(wrap_source, wrap_target);
    pio_program.code.iter().for_each(|x| {
        info!("{:?}", x);
    });

    // pio_program.wrap.target;

    // Load the PIO program into the PIO block
    let mut cfg = embassy_rp::pio::Config::default();

    cfg.use_program(&common.load_program(&pio_program), &[]);
    cfg.clock_divider = (U56F8!(125_000_000) / U56F8!(10_000)).to_fixed();
    cfg.shift_in = ShiftConfig {
        auto_fill: true,
        threshold: 32,
        direction: ShiftDirection::Right,
    };
    // cfg.shift_out = ShiftConfig {
    //     auto_fill: true,
    //     threshold: 32,
    //     direction: ShiftDirection::Left,
    // };

    sm.set_config(&cfg);
    sm.set_enable(true);
    // Initialize GPIO pins for PIO
    info!("After Pio");

    let vsync = common.make_pio_pin(p.PIN_6);
    let hsync = common.make_pio_pin(p.PIN_7);
    let pclk = common.make_pio_pin(p.PIN_8);
    sm.set_pin_dirs(Direction::In, &[&vsync, &hsync, &pclk]);

    // Configure DMA
    // PIO and DMA configuration
    let mut dma_out_ref = p.DMA_CH0.into_ref();
    let mut dma_in_ref = p.DMA_CH1.into_ref();
    let length = 160 * 120; // Assuming 160x120 image size for the example
    let mut buffer: [u8; 160 * 120] = [0; 160 * 120];

    // loop {
    sm.restart();
    info!("before dma read");
    info!("{:?}", sm.is_enabled());
    let rx = sm.rx();
    let is_it_full = rx.empty();
    let idk = rx.try_pull();
    info!("{:?}", idk);

    // rx.wait_pull(dma_in_ref.reborrow(), &mut buffer).await;
    // info!("its working?");
    // }
}
