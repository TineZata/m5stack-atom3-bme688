use std::thread;
use std::time::Duration;

use anyhow::{Context, Result};
use bme680::{
    Bme680, I2CAddress, IIRFilterSize, OversamplingSetting, PowerMode, SettingsBuilder,
};
use embedded_graphics::mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Baseline, Text};
use esp_idf_hal::delay::Ets;
use esp_idf_hal::gpio::{AnyInputPin, PinDriver};
use esp_idf_hal::i2c::{config::Config as I2cConfig, I2cDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::spi::{
    config::{Config as SpiConfig, DriverConfig as SpiDriverConfig, MODE_0},
    SpiDeviceDriver,
};
use log::info;
use mipidsi::models::GC9107;
use mipidsi::options::ColorOrder;
use mipidsi::Builder;
use mipidsi::interface::SpiInterface;

const LCD_W: u32 = 128;
const LCD_H: u32 = 128;

fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().context("Peripherals unavailable")?;

    let i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio2,
        peripherals.pins.gpio1,
        &I2cConfig::new().baudrate(400.kHz().into()),
    )
    .context("Failed to init I2C")?;

    let mut delay = Ets;
    let mut bme = Bme680::init(i2c, &mut delay, I2CAddress::Primary)
        .map_err(|e| anyhow::anyhow!("Failed to init BME688/BME680 at 0x76: {:?}", e))?;

    let bme_settings = SettingsBuilder::new()
        .with_humidity_oversampling(OversamplingSetting::OS2x)
        .with_pressure_oversampling(OversamplingSetting::OS4x)
        .with_temperature_oversampling(OversamplingSetting::OS8x)
        .with_temperature_filter(IIRFilterSize::Size3)
        .with_gas_measurement(Duration::from_millis(150), 320, 25)
        .with_run_gas(true)
        .build();

    bme.set_sensor_settings(&mut delay, bme_settings)
        .map_err(|e| anyhow::anyhow!("Failed to apply BME settings: {:?}", e))?;

    let profile_dur = bme
        .get_profile_dur(&bme_settings.0)
        .map_err(|e| anyhow::anyhow!("Failed to calculate BME profile duration: {:?}", e))?;

    let mut lcd_bl = PinDriver::output(peripherals.pins.gpio38)?;
    lcd_bl.set_high()?;

    let lcd_dc = PinDriver::output(peripherals.pins.gpio33)?;
    let lcd_rst = PinDriver::output(peripherals.pins.gpio34)?;

    let spi = SpiDeviceDriver::new_single(
        peripherals.spi3,
        peripherals.pins.gpio17,
        peripherals.pins.gpio21,
        Option::<AnyInputPin>::None,
        Some(peripherals.pins.gpio15),
        &SpiDriverConfig::new(),
        &SpiConfig::new().baudrate(40.MHz().into()).data_mode(MODE_0),
    )
    .context("Failed to init LCD SPI")?;

    let mut spi_buffer = [0u8; 256]; // Smaller buffer for display operations
    let di = SpiInterface::new(spi, lcd_dc, &mut spi_buffer);

    let mut display = Builder::new(GC9107, di)
        .reset_pin(lcd_rst)
        .display_size(LCD_W as u16, LCD_H as u16)
        .color_order(ColorOrder::Bgr)
        .init(&mut delay)
        .map_err(|e| anyhow::anyhow!("Failed to init GC9107 display: {:?}", e))?;

    render_lines(
        &mut display,
        &["AtomS3 + BME688", "Booting...", "", "Waiting data..."],
    )
    .context("Initial LCD draw failed")?;

    let mut gas_baseline_ohm: Option<f32> = None;

    loop {
        bme.set_sensor_mode(&mut delay, PowerMode::ForcedMode)
            .map_err(|e| anyhow::anyhow!("Failed to trigger forced BME measurement: {:?}", e))?;

        thread::sleep(profile_dur);

        let (data, _status) = bme
            .get_sensor_data(&mut delay)
            .map_err(|e| anyhow::anyhow!("Failed to read BME measurement: {:?}", e))?;

        let temperature_c = data.temperature_celsius() as f32;
        let pressure_hpa = data.pressure_hpa() as f32;
        let humidity_pct = data.humidity_percent() as f32;
        let gas_ohm = data.gas_resistance_ohm();

        let baseline = gas_baseline_ohm.get_or_insert(gas_ohm as f32);
        *baseline = (*baseline * 0.99) + ((gas_ohm as f32) * 0.01);

        let voc_proxy_index = ((*baseline / gas_ohm as f32) * 100.0).clamp(25.0, 500.0);
        let quality = if voc_proxy_index < 80.0 {
            "Great"
        } else if voc_proxy_index < 140.0 {
            "Good"
        } else if voc_proxy_index < 220.0 {
            "Moderate"
        } else {
            "Poor"
        };

        info!(
            "T:{:.1}C P:{:.1}hPa H:{:.1}% Gas:{}Ohm VOCx:{:.0} ({})",
            temperature_c,
            pressure_hpa,
            humidity_pct,
            gas_ohm,
            voc_proxy_index,
            quality
        );

        let l0 = "AtomS3 + BME688".to_owned();
        let l1 = format!("T {:>5.1} C", temperature_c);
        let l2 = format!("P {:>6.1} hPa", pressure_hpa);
        let l3 = format!("H {:>6.1} %", humidity_pct);
        let l4 = format!("Gas {:>6} ohm", gas_ohm);
        let l5 = format!("VOCx {:>5.0} {}", voc_proxy_index, quality);

        render_lines(
            &mut display,
            &[l0.as_str(), l1.as_str(), l2.as_str(), l3.as_str(), l4.as_str(), l5.as_str()],
        )
        .context("LCD draw failed")?;

        thread::sleep(Duration::from_millis(1000));
    }
}

fn render_lines<D>(display: &mut D, lines: &[&str]) -> Result<()>
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: core::fmt::Debug,
{
    let bg = PrimitiveStyle::with_fill(Rgb565::BLACK);
    Rectangle::new(Point::new(0, 0), Size::new(LCD_W, LCD_H))
        .into_styled(bg)
        .draw(display)
        .map_err(|e| anyhow::anyhow!("clear failed: {e:?}"))?;

    let style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(Rgb565::GREEN)
        .build();

    let mut y = 3;
    for line in lines {
        Text::with_baseline(line, Point::new(3, y), style, Baseline::Top)
            .draw(display)
            .map_err(|e| anyhow::anyhow!("text draw failed: {e:?}"))?;
        y += 11;
    }

    Ok(())
}
