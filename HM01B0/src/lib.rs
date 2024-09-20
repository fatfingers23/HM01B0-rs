#![no_std]

use core::ops::Add;
use defmt::*;
use embassy_rp::flash::Instance;
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::rom_data::float_funcs::int_to_float;
use embassy_rp::{pac::pio::Pio, Peripherals};
use embedded_hal::i2c::ErrorType;
use embedded_hal_async::i2c::I2c;
use log::info;
use crate::Addresses::ModelId;

const HM01B0_ADDR: u8 = 0x24;

pub enum Addresses {
    ModelId = 0x0000,
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

pub struct HM01B0<I>
where
    I: I2c,
{
    i2c: I,
    // pio: Pio<'_, P>,
    size: PictureSize,
}

impl<I> HM01B0<I>
where
    I: I2c
{
    pub async fn new(mut i2c: I, size: PictureSize, data_bits: DataBits) -> Self where <I as embedded_hal::i2c::ErrorType>::Error: Format
    {
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

        //TODO not needed for POC, but dont forget to check it out. C code
        // if (config->reset_pin > -1) {
        //     gpio_init(config->reset_pin);
        //     gpio_set_dir(config->reset_pin, GPIO_OUT);
        //     gpio_put(config->reset_pin, 0);
        //     sleep_ms(100);
        //     gpio_put(config->reset_pin, 1);
        // }

        // if (config->mclk_pin > -1) {
        //     gpio_set_function(config->mclk_pin, GPIO_FUNC_PWM);
        //     uint mclk_slice_num = pwm_gpio_to_slice_num(config->mclk_pin);
        //     uint mclk_channel = pwm_gpio_to_channel(config->mclk_pin);

        //     // PWM @ ~25 MHz, 50% duty cycle
        //     pwm_set_clkdiv(mclk_slice_num, 1.25);
        //     pwm_set_wrap(mclk_slice_num, 3);
        //     pwm_set_chan_level(mclk_slice_num, mclk_channel, 2);
        //     pwm_set_enabled(mclk_slice_num, true);
        // }
        info!("Configured");

        // i2c.write(HM01B0_ADDR, &[Addresses::ModelId as u8, 0x0000]).await.unwrap();
        // info!("after write");
        // i2c.read(HM01B0_ADDR, &mut result ).await.unwrap();
        // let idk =  (result[0] as u16) << 8 | (result[0] as u16);
        // 
        // info!("{:?}", idk);
        // info!("After read");

        let mut address_bytes = [0u8; 2];
        let mut result_bytes = [0xffu8; 2];

        // Manually write u16 to the byte array in little-endian format
        write_u16_le(&mut address_bytes, Addresses::ModelId as u16);

        // Write the address to the sensor
        i2c.write(HM01B0_ADDR, &address_bytes).await.unwrap();

        // Read 2 bytes from the sensor
        i2c.read(HM01B0_ADDR, &mut result_bytes).await.unwrap();
        info!("{:?}", read_u16_le(&result_bytes));

        // let model_read = i2c.write_read(HM01B0_ADDR, &[Addresses::ModelId as u8], &mut result).await;

        // match model_read {
        //     Ok(_) => {
        //         info!("{:?}", result);
        //     }
        //     Err(e) => {
        //         error!("{:?}", e)
        //     }
        // }

        Self { i2c, size }
    }

}
fn write_u16_le(bytes: &mut [u8; 2], value: u16) {
    bytes[0] = (value & 0xFF) as u8;        // Lower byte
    bytes[1] = (value >> 8) as u8;          // Higher byte
}

fn read_u16_le(bytes: &[u8; 2]) -> u16 {
    (bytes[1] as u16) << 8 | (bytes[0] as u16)
}

