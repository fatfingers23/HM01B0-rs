#![no_std]

// use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::i2c::Error;
use embedded_hal_async::i2c::I2c;
use defmt::*;

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
    I: I2c
{

    pub async fn new(i2c: &'a mut I, size: PictureSize, data_bits: DataBits) -> Self
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

        let mut hm01b0 = Self { i2c, size };
        
        match hm01b0.hm01b0_read_reg16(Addresses::ModelId as u16).await {
            Ok(model) => {
                debug!("Camera Model ID: {:?}", model);
                if model != 0x01b0{
                    error!("Invalid model id!")
                }
            } Err(e) => {
                error!("{:?}", e)
            },
        }
        hm01b0
    }

    fn write_u16(&mut self, bytes: &mut [u8; 2], value: u16) {
        bytes[0] = (value >> 8) as u8;   // Higher byte
        bytes[1] = (value & 0xFF) as u8; // Lower byte
    }

    fn read_u16(&mut self, bytes: &[u8; 2]) -> u16 {
        ((bytes[0] as u16) << 8) | (bytes[1] as u16)
    }

     async fn hm01b0_read_reg16(
        &mut self,
        address: u16
    ) -> Result<u16, Error> {
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
}


// let model_read_result = hm01b0_read_reg16(&mut i2c, ModelId as u16);



