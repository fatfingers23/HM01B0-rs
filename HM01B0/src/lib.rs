#![no_std]

// use embassy_rp::peripherals::{PIO0, PIO1};
use crate::Addresses::{ModeSelect, SoftwareReset};
use defmt::*;
use embassy_rp::gpio::{Output, Pin};
use embassy_rp::i2c::Error;
use embassy_rp::pwm;
use embassy_rp::pwm::Pwm;
use embassy_time::{Duration, Timer};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::i2c::I2c;
use fixed::prelude::ToFixed;
use fixed::types::extra::U4;
use fixed::FixedU16;

const HM01B0_ADDR: u8 = 0x24;

pub enum Addresses {
    BitControl = 0x3059,
    BinningMode = 0x0390,
    FrameLengthLines = 0x0340,
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
pub enum DataBits {
    Bits8,
    Bits4,
    Bits1,
}

struct I2CConfig {
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
            PictureSize::Size320x320 => I2CConfig {
                readout_x_val: 0x01,
                readout_y_val: 0x01,
                binning_mode_val: 0x00,
                qvga_win_en_val: 0x00,
                frame_length_lines_val: 0x0158,
                line_length_pclk_val: 0x0178,
                num_border_px: 2,
            },
            PictureSize::Size320x240 => I2CConfig {
                readout_x_val: 0x01,
                readout_y_val: 0x01,
                binning_mode_val: 0x00,
                qvga_win_en_val: 0x01,
                frame_length_lines_val: 0x0104,
                line_length_pclk_val: 0x0178,
                num_border_px: 2,
            },
            PictureSize::Size160x120 => I2CConfig {
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

        let mut hm01b0 = Self { i2c, size };

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

        //Set camera config to the device
        hm01b0
            .write_reg8(Addresses::BitControl as u16, bit_control)
            .await
            .unwrap();

        hm01b0
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

    fn write_u16(&mut self, bytes: &mut [u8; 2], value: u16) {
        bytes[0] = (value >> 8) as u8; // Higher byte
        bytes[1] = (value & 0xFF) as u8; // Lower byte
    }

    fn read_u16(&mut self, bytes: &[u8; 2]) -> u16 {
        ((bytes[0] as u16) << 8) | (bytes[1] as u16)
    }

    async fn read_reg16(&mut self, address: u16) -> Result<u16, Error> {
        let mut address_bytes = [0u8; 2];
        let mut result_bytes = [0xffu8; 2];

        self.write_u16(&mut address_bytes, address);

        // Write the address to the sensor
        self.i2c.write(HM01B0_ADDR, &address_bytes).await.unwrap();

        // Read 2 bytes from the sensor
        self.i2c.read(HM01B0_ADDR, &mut result_bytes).await.unwrap();
        // Manually read the u16 value from the result bytes
        Ok(self.read_u16(&result_bytes))
    }

    async fn read_reg8(&mut self, address: u16) -> Result<u8, Error> {
        let mut address_bytes = [0u8; 2];
        let mut result = [0xffu8; 1];

        // Write the 16-bit address in big-endian format
        address_bytes[0] = (address >> 8) as u8; // High byte
        address_bytes[1] = (address & 0xFF) as u8; // Low byte

        // Write the address first
        self.i2c.write(HM01B0_ADDR, &address_bytes).await.unwrap();

        // Read 1 byte from the device
        self.i2c.read(HM01B0_ADDR, &mut result).await.unwrap();

        Ok(result[0])
    }

    async fn write_reg8(&mut self, address: u16, value: u8) -> Result<(), Error> {
        let mut data = [0u8; 3];

        // Write the 16-bit address in big-endian format
        data[0] = (address >> 8) as u8; // High byte
        data[1] = (address & 0xFF) as u8; // Low byte

        // Add the 8-bit value to the buffer
        data[2] = value;

        self.i2c.write(HM01B0_ADDR, &data).await.unwrap();
        Ok(())
    }
}
