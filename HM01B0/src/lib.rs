#![no_std]

use crate::Addresses::{ModeSelect, SoftwareReset};
use defmt::*;
use embassy_rp::gpio::Output;
use embassy_rp::gpio::Pin;
use embassy_rp::i2c::Error;
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{Pio, StateMachine};
use embassy_rp::pwm::Pwm;
use embassy_rp::{pwm, PeripheralRef};
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;
use export::debug;
use fixed::prelude::ToFixed;

const HM01B0_ADDR: u8 = 0x24;

pub enum Addresses {
    BitControl = 0x3059,
    BinningMode = 0x0390,
    OSCClockDiver = 0x3060,
    FrameLengthLines = 0x0340,
    GroupParameterHold = 0x0104,
    IntegrationTimeInLines = 0x0202,
    LineLength = 0x0342,
    ModeSelect = 0x0100,
    ModelId = 0x0000,
    QVGAEnable = 0x3010,
    ReadoutX = 0x0383,
    ReadoutY = 0x0387,
    SoftwareReset = 0x0103,
}

//TODO untested
pub fn get_mclk_pwm_config() -> pwm::Config {
    let mut config: pwm::Config = Default::default();
    config.divider = 1.25.to_fixed();
    config.compare_a = 2;
    config.compare_b = 0;
    config.top = 3;
    config
}

pub enum PioInstance {
    pio0,
    pio1,
}

pub enum PictureSize {
    Size320x320,
    Size320x240,
    Size160x120,
}

//TODO find a better name for this and what its called or at lest an explanation from the data sheet
#[derive(Clone, Copy)]
pub enum DataBits {
    Bits8,
    Bits4,
    Bits1,
}

impl DataBits {
    pub fn to_u8(self) -> u8 {
        match self {
            DataBits::Bits8 => 8,
            DataBits::Bits4 => 4,
            DataBits::Bits1 => 1,
        }
    }
}

#[derive(Clone)]
struct Config {
    readout_x_val: u8,           // 0x0383
    readout_y_val: u8,           // 0x0387
    binning_mode_val: u8,        // 0x0390
    qvga_win_en_val: u8,         // 0x3010
    frame_length_lines_val: u16, // 0x0340
    line_length_pclk_val: u16,   // 0x0342
    num_border_px: u8,
    // bit_control_val: u8,         // 0x3059
}

pub struct HM01B0<'a, I>
where
    I: I2c,
{
    i2c: &'a mut I,
    // pio: Pio<'_, P>,
    size: PictureSize,
    config: Config,
    pub num_pclk_per_px: u8,
}

impl<'a, I> HM01B0<'a, I>
where
    I: I2c,
{
    pub async fn new(
        i2c: &'a mut I,
        size: PictureSize,
        data_bits: DataBits,
        reset_pin: Option<Output<'a>>,
        mclk_pwm: Option<Pwm<'a>>,
    ) -> Self {
        let config = match size {
            PictureSize::Size320x320 => Config {
                readout_x_val: 0x01,
                readout_y_val: 0x01,
                binning_mode_val: 0x00,
                qvga_win_en_val: 0x00,
                frame_length_lines_val: 0x0158,
                line_length_pclk_val: 0x0178,
                num_border_px: 2,
            },
            PictureSize::Size320x240 => Config {
                readout_x_val: 0x01,
                readout_y_val: 0x01,
                binning_mode_val: 0x00,
                qvga_win_en_val: 0x01,
                frame_length_lines_val: 0x0104,
                line_length_pclk_val: 0x0178,
                num_border_px: 2,
            },
            PictureSize::Size160x120 => Config {
                readout_x_val: 0x03,
                readout_y_val: 0x03,
                binning_mode_val: 0x03,
                qvga_win_en_val: 0x01,
                frame_length_lines_val: 0x0080,
                line_length_pclk_val: 0x00D7,
                num_border_px: 2,
            },
        };

        if let Some(mut reset) = reset_pin {
            reset.set_low();
            Timer::after(Duration::from_millis(100)).await;
            reset.set_high();
        }

        //I think mclk pin setup only needs the config and like this before init on your program
        //Sorry don't have that model camera to test yet
        // let mut c: embassy_rp::pwm::Config = get_mclk_pwm_config();
        // let mut pwm = Pwm::new_output_b(p.PWM_SLICE4, p.PIN_10, c.clone());

        let num_pclk_per_px = match data_bits {
            DataBits::Bits8 => 1,
            DataBits::Bits4 => 2,
            DataBits::Bits1 => 8,
        };

        let mut hm01b0 = Self {
            i2c,
            size,
            config: config.clone(),
            num_pclk_per_px,
        };

        match hm01b0.read_reg16(Addresses::ModelId as u16).await {
            Ok(model) => {
                debug!("Camera Model ID: {:?}", model);
                if model != 0x01b0 {
                    error!("Invalid model id!")
                }
            }
            Err(e) => {
                error!("{:?}", e)
            }
        }

        hm01b0.reset().await;

        let bit_control = match data_bits {
            DataBits::Bits8 => 0x02,
            DataBits::Bits4 => 0x42,
            DataBits::Bits1 => 0x22,
        };

        //Print config values as u8 not hex

        debug!("Bit Control: {:?}", bit_control);
        debug!("Readout X: {:?}", config.readout_x_val);
        debug!("Readout Y: {:?}", config.readout_y_val);
        debug!("Binning Mode: {:?}", config.binning_mode_val);
        debug!("QVGA Enable: {:?}", config.qvga_win_en_val);
        debug!("Frame Length Lines: {:?}", config.frame_length_lines_val);
        debug!("Line Length PCLK: {:?}", config.line_length_pclk_val);
        debug!("INTEGRATION_H: {:?}", config.line_length_pclk_val / 2);

        // //Set camera config to the device
        hm01b0
            .write_reg8(Addresses::BitControl as u16, bit_control)
            .await
            .unwrap();

        hm01b0
            .write_reg8(Addresses::ReadoutX as u16, config.readout_x_val)
            .await
            .unwrap();

        hm01b0
            .write_reg8(Addresses::ReadoutY as u16, config.readout_y_val)
            .await
            .unwrap();

        hm01b0
            .write_reg8(Addresses::BinningMode as u16, config.binning_mode_val)
            .await
            .unwrap();

        hm01b0
            .write_reg8(Addresses::QVGAEnable as u16, config.qvga_win_en_val)
            .await
            .unwrap();

        hm01b0
            .write_reg16(
                Addresses::FrameLengthLines as u16,
                config.frame_length_lines_val,
            )
            .await
            .unwrap();

        hm01b0
            .write_reg16(Addresses::LineLength as u16, config.line_length_pclk_val)
            .await
            .unwrap();

        // OSC_CLK_DIV
        hm01b0
            .write_reg8(Addresses::OSCClockDiver as u16, 0x08 | 0)
            .await
            .unwrap();

        // INTEGRATION_H
        hm01b0
            .write_reg8(
                Addresses::IntegrationTimeInLines as u16,
                (config.line_length_pclk_val / 2) as u8,
            )
            .await
            .unwrap();

        // GRP_PARAM_HOLD
        hm01b0
            .write_reg8(Addresses::GroupParameterHold as u16, 0x01)
            .await
            .unwrap();

        hm01b0
    }

    pub async fn set_coarse_integration(&mut self, mut lines: u16) {
        if (lines < 2) {
            lines = 2;
        }

        lines -= 2;

        self.write_reg16(Addresses::IntegrationTimeInLines as u16, lines)
            .await
            .unwrap();

        self.write_reg16(Addresses::GroupParameterHold as u16, 0x01)
            .await
            .unwrap();
    }

    async fn hm01b0_read_frame(
        &mut self,
        pio: &mut Pio<'_, PIO0>,
        sm: &mut StateMachine<'_, PIO0, 0>,
        dma: PeripheralRef<'_, DMA_CH0>,
        buffer: &mut [u8],
        width: usize,
        num_pclk_per_px: usize,
    ) {
        // Initialize PIO state machine with HM01B0 program
        sm.restart();
        sm.clear_fifos();

        // Configure PIO state machine
        // let config = ShiftConfig {
        //     auto_fill: false,
        //     direction: ShiftDirection::Right,
        //     threshold: 8,
        // };
        // sm. = config;
        // sm.set_enabled(true);
        sm.set_enable(true);
        // Prepare DMA transfer from PIO RX FIFO to the buffer
        // let dreq = pio.di get_dreq(false); // False for RX direction

        // let dma_config = dma::Config::default()
        //     .increment_source(false) // PIO RX FIFO, no increment
        //     .increment_dest(true) // Buffer, increment
        //     .data_size(dma::WordSize::Byte)
        //     .dreq(dreq);
        //
        // dma.start_transfer(dma_config, pio.rx_fifo(), buffer).await;

        // Push the number of pixels to read into the PIO
        let pixels_to_read = width * num_pclk_per_px - 1;
        //     sm.tx().push_blocking(pixels_to_read as u32);
        //
        //     // Enable the HM01B0 sensor by writing to its register (simulated here)
        //     hm01b0_write_reg8(0x0100, 0x01).await; // MODE_SELECT: Enable sensor
        //
        //     // Wait for DMA transfer to complete
        //     yield_now().await; // To give control to the executor while waiting
        //
        //     // Disable the PIO state machine after DMA completes
        //     sm.set_enabled(false);
        //
        //     // Disable the HM01B0 sensor (simulated here)
        //     hm01b0_write_reg8(0x0100, 0x00).await; // MODE_SELECT: Disable sensor
    }

    async fn reset(&mut self) {
        self.write_reg8(SoftwareReset as u16, 0x01).await.unwrap();

        for attempt in 1..=10 {
            match self.read_reg8(ModeSelect as u16).await {
                Ok(result) => {
                    debug!("Status: {:?}", result);
                    if result == 0x00 {
                        break;
                    }
                }
                Err(e) => {
                    debug!("Attempt {}: {:?}", attempt, e);
                    if attempt == 10 {
                        error!("Failed to reset HM01B0 after 10 attempts");
                    }
                }
            }
            Timer::after(Duration::from_millis(100)).await;
        }
    }

    async fn read_reg16(&mut self, address: u16) -> Result<u16, Error> {
        let mut address_bytes = [0u8; 2];
        let mut result_bytes = [0xffu8; 2];

        address_bytes[0] = (address >> 8) as u8; // Higher byte
        address_bytes[1] = (address & 0xFF) as u8; // Lower byte

        // Write the address to the sensor
        self.i2c.write(HM01B0_ADDR, &address_bytes).await.unwrap();

        // Read 2 bytes from the sensor
        self.i2c.read(HM01B0_ADDR, &mut result_bytes).await.unwrap();

        // Manually read the u16 value from the result bytes
        let result = ((result_bytes[0] as u16) << 8) | (result_bytes[1] as u16);
        Ok(result)
    }

    fn reverse_u16(&mut self, value: u16) -> u16 {
        (value << 8) | (value >> 8)
    }

    async fn write_reg16(&mut self, address: u16, value: u16) -> Result<(), Error> {
        let mut data = [0u8; 4];

        // Write the 16-bit address
        data[0] = (address >> 8) as u8; // High byte of address
        data[1] = (address & 0xFF) as u8; // Low byte of address

        data[2] = (value >> 8) as u8;
        data[3] = (value & 0xFF) as u8;

        self.i2c.write(HM01B0_ADDR, &data).await.unwrap();
        Ok(())
    }

    pub async fn read_reg8(&mut self, address: u16) -> Result<u8, Error> {
        let mut address_bytes = [0u8; 2];
        let mut result = [0xffu8; 1];

        // Write the 16-bit address in
        address_bytes[0] = (address >> 8) as u8; // High byte
        address_bytes[1] = (address & 0xFF) as u8; // Low byte

        // Write the address first
        self.i2c.write(HM01B0_ADDR, &address_bytes).await.unwrap();

        // Read 1 byte from the device
        self.i2c.read(HM01B0_ADDR, &mut result).await.unwrap();

        Ok(result[0])
    }

    pub async fn write_reg8(&mut self, address: u16, value: u8) -> Result<(), Error> {
        let mut data = [0u8; 3];

        // Reverse the 16-bit address to match the __REV16 behavior in C
        data[0] = (address >> 8) as u8; // High byte of address
        data[1] = (address & 0xFF) as u8; // Low byte of address

        // Write the 8-bit value
        data[2] = value;

        // Debugging: print the binary of data (similar to C code)

        // for i in 0..3 {
        //     info!("{:08b}", data[i]);
        // }

        self.i2c
            .write(HM01B0_ADDR, &[address as u8, value])
            .await
            .unwrap();
        Ok(())
    }
}
