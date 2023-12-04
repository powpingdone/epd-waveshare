//! A simple Driver for the Waveshare 7.3 inch (F) E-Ink Display via SPI
//!
//! # References
//!
//! - [Datasheet](https://www.waveshare.com/wiki/5.65inch_e-Paper_Module_(F))
//! - [Waveshare C driver](https://github.com/waveshare/e-Paper/blob/master/RaspberryPi%26JetsonNano/c/lib/e-Paper/EPD_5in65f.c)
//! - [Waveshare Python driver](https://github.com/waveshare/e-Paper/blob/master/RaspberryPi%26JetsonNano/python/lib/waveshare_epd/epd5in65f.py)

use embedded_hal::{
    delay::DelayUs,
    digital::{InputPin, OutputPin},
    spi::SpiDevice,
};

use crate::color::OctColor;
use crate::interface::DisplayInterface;
use crate::traits::{InternalWiAdditions, RefreshLut, WaveshareDisplay};

pub(crate) mod command;
use self::command::Command;
use crate::buffer_len;

/// Full size buffer for use with the 7.3f EPD
#[cfg(feature = "graphics")]
pub type Display7in3f = crate::graphics::Display<
    WIDTH,
    HEIGHT,
    false,
    { buffer_len(WIDTH as usize, HEIGHT as usize * 4) },
    OctColor,
>;

/// Width of the display
pub const WIDTH: u32 = 800;
/// Height of the display
pub const HEIGHT: u32 = 480;
/// Default Background Color
pub const DEFAULT_BACKGROUND_COLOR: OctColor = OctColor::White;
/// Default mode of writing data (single byte vs blockwise)
const SINGLE_BYTE_WRITE: bool = true;

/// Epd5in65f driver
///
pub struct Epd7in3f<SPI, BUSY, DC, RST, DELAY> {
    /// Connection Interface
    interface: DisplayInterface<SPI, BUSY, DC, RST, DELAY, SINGLE_BYTE_WRITE>,
    /// Background Color
    color: OctColor,
}

impl<SPI, BUSY, DC, RST, DELAY> InternalWiAdditions<SPI, BUSY, DC, RST, DELAY>
    for Epd7in3f<SPI, BUSY, DC, RST, DELAY>
where
    SPI: SpiDevice,
    BUSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
    DELAY: DelayUs,
{
    fn init(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        // Reset the device
        self.interface.reset(delay, 10_000, 2_000);

        // 0xAA CMDH 0x49 0x55 0x20 0x08 0x09 0x18
        self.cmd_with_data(spi, Command::PowerSetting, &[0x3F, 0x00, 0x32, 0x2A, 0x0E, 0x2A])?;
        self.cmd_with_data(spi, Command::PanelSetting, &[0x5F, 0x69])?;
        self.cmd_with_data(spi, Command::PowerOffSequenceSetting, &[0x00, 0x54, 0x00, 0x44])?;
        // 0x05 BTST1(BoosterSoftStart1) 0x40 0x1F 0x1F 0x22
        self.cmd_with_data(spi, Command::BoosterSoftStart2, &[0x6F, 0x1F, 0x1F, 0x2C])?;
        // 0x08 BTST3(BoosterSoftStart3) 0x6F 0x1F 0x1F 0x22
        // 0x13 IPC 0x00 0x04 
        self.cmd_with_data(spi, Command::PllControl, &[0x3C])?;
        self.cmd_with_data(spi, Command::TemperatureCalibration, &[0x00])?;
        self.update_vcom(spi)?;
        self.cmd_with_data(spi, Command::TconSetting, &[0x02, 0x00])?;
        self.send_resolution(spi)?;
        self.cmd_with_data(spi, Command::VcmDcSetting, &[0x1E])?;
        // 0x84 T_VDCS 0x00
        // 0x86 AGID 0x00
        self.cmd_with_data(spi, Command::FlashMode, &[0x2F])?;
        // 0xE0 CCSET 0x00
        // 0xE6 TSSET 0x00

        delay.delay_us(100_000);

        self.update_vcom(spi)?;
        Ok(())
    }
}

impl<SPI, BUSY, DC, RST, DELAY> WaveshareDisplay<SPI, BUSY, DC, RST, DELAY>
    for Epd7in3f<SPI, BUSY, DC, RST, DELAY>
where
    SPI: SpiDevice,
    BUSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
    DELAY: DelayUs,
{
    type DisplayColor = OctColor;
    fn new(
        spi: &mut SPI,
        busy: BUSY,
        dc: DC,
        rst: RST,
        delay: &mut DELAY,
        delay_us: Option<u32>,
    ) -> Result<Self, SPI::Error> {
        let interface = DisplayInterface::new(busy, dc, rst, delay_us);
        let color = DEFAULT_BACKGROUND_COLOR;

        let mut epd = Epd7in3f { interface, color };

        epd.init(spi, delay)?;

        Ok(epd)
    }

    fn wake_up(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.init(spi, delay)
    }

    fn sleep(&mut self, spi: &mut SPI, _delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.cmd_with_data(spi, Command::DeepSleep, &[0xA5])?;
        Ok(())
    }

    fn update_frame(
        &mut self,
        spi: &mut SPI,
        buffer: &[u8],
        delay: &mut DELAY,
    ) -> Result<(), SPI::Error> {
        self.wait_until_idle(spi, delay)?;
        self.update_vcom(spi)?;
        self.send_resolution(spi)?;
        self.cmd_with_data(spi, Command::DataStartTransmission1, buffer)?;
        Ok(())
    }

    fn update_partial_frame(
        &mut self,
        _spi: &mut SPI,
        _delay: &mut DELAY,
        _buffer: &[u8],
        _x: u32,
        _y: u32,
        _width: u32,
        _height: u32,
    ) -> Result<(), SPI::Error> {
        unimplemented!();
    }

    fn display_frame(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.wait_until_idle(spi, delay)?;
        self.command(spi, Command::PowerOn)?;
        self.wait_until_idle(spi, delay)?;
        self.command(spi, Command::DisplayRefresh)?;
        self.wait_until_idle(spi, delay)?;
        self.command(spi, Command::PowerOff)?;
        self.wait_busy_low(delay);
        Ok(())
    }

    fn update_and_display_frame(
        &mut self,
        spi: &mut SPI,
        buffer: &[u8],
        delay: &mut DELAY,
    ) -> Result<(), SPI::Error> {
        self.update_frame(spi, buffer, delay)?;
        self.display_frame(spi, delay)?;
        Ok(())
    }

    fn clear_frame(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        let bg = OctColor::colors_byte(self.color, self.color);
        self.wait_until_idle(spi, delay)?;
        self.update_vcom(spi)?;
        self.send_resolution(spi)?;
        self.command(spi, Command::DataStartTransmission1)?;
        self.interface.data_x_times(spi, bg, WIDTH * HEIGHT / 2)?;
        self.display_frame(spi, delay)?;
        Ok(())
    }

    fn set_background_color(&mut self, color: OctColor) {
        self.color = color;
    }

    fn background_color(&self) -> &OctColor {
        &self.color
    }

    fn width(&self) -> u32 {
        WIDTH
    }

    fn height(&self) -> u32 {
        HEIGHT
    }

    fn set_lut(
        &mut self,
        _spi: &mut SPI,
        _delay: &mut DELAY,
        _refresh_rate: Option<RefreshLut>,
    ) -> Result<(), SPI::Error> {
        unimplemented!();
    }

    fn wait_until_idle(&mut self, _spi: &mut SPI, delay: &mut DELAY) -> Result<(), SPI::Error> {
        self.interface.wait_until_idle(delay, true);
        Ok(())
    }
}

impl<SPI, BUSY, DC, RST, DELAY> Epd7in3f<SPI, BUSY, DC, RST, DELAY>
where
    SPI: SpiDevice,
    BUSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
    DELAY: DelayUs,
{
    fn command(&mut self, spi: &mut SPI, command: Command) -> Result<(), SPI::Error> {
        self.interface.cmd(spi, command)
    }

    fn send_data(&mut self, spi: &mut SPI, data: &[u8]) -> Result<(), SPI::Error> {
        self.interface.data(spi, data)
    }

    fn cmd_with_data(
        &mut self,
        spi: &mut SPI,
        command: Command,
        data: &[u8],
    ) -> Result<(), SPI::Error> {
        self.interface.cmd_with_data(spi, command, data)
    }

    fn wait_busy_low(&mut self, delay: &mut DELAY) {
        self.interface.wait_until_idle(delay, false);
    }
    fn send_resolution(&mut self, spi: &mut SPI) -> Result<(), SPI::Error> {
        let w = self.width();
        let h = self.height();

        self.command(spi, Command::TconResolution)?;
        self.send_data(spi, &[(w >> 8) as u8])?;
        self.send_data(spi, &[w as u8])?;
        self.send_data(spi, &[(h >> 8) as u8])?;
        self.send_data(spi, &[h as u8])
    }

    fn update_vcom(&mut self, spi: &mut SPI) -> Result<(), SPI::Error> {
        let bg_color = (self.color.get_nibble() & 0b111) << 5;
        self.cmd_with_data(spi, Command::VcomAndDataIntervalSetting, &[0x17 | bg_color])?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn epd_size() {
        assert_eq!(WIDTH, 600);
        assert_eq!(HEIGHT, 448);
        assert_eq!(DEFAULT_BACKGROUND_COLOR, OctColor::White);
    }
}
