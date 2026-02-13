use std::thread;
use std::time::Duration;

use anyhow::{Context, Result};
use embedded_graphics::mono_font::{ascii::FONT_10X20, MonoTextStyle};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Alignment, Text};
use embedded_hal::spi::MODE_0;  // Changed from MODE_3 to MODE_0 (per M5GFX)
use esp_idf_hal::delay::Ets;
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::spi::{config::Config, SpiDeviceDriver, SpiDriverConfig};
use log::info;
use mipidsi::{Builder, models::ST7789};  // Note: AtomS3 actually uses GC9107, but mipidsi doesn't support it
use mipidsi::interface::SpiInterface;

fn draw_sample<T>(lcd: &mut T)
where
    T: DrawTarget<Color = Rgb565>,
{
    // Create a border style  
    let border_stroke = PrimitiveStyle::with_stroke(Rgb565::BLUE, 4);

    // Draw border around the screen (128x128)
    Rectangle::new(Point::new(0, 0), Size::new(128, 128))
        .into_styled(border_stroke)
        .draw(lcd)
        .ok();

    // Create text style
    let character_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    let text = "AtomS3 Rust!";

    // Draw text centered
    Text::with_alignment(text, Point::new(64, 64), character_style, Alignment::Center)
        .draw(lcd)
        .ok();
}

fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("=== AtomS3 Display Test ===");

    let peripherals = Peripherals::take().context("Peripherals unavailable")?;
    
    // Enable backlight - GPIO16 per M5GFX official implementation
    info!("Enabling backlight on GPIO16");
    let mut backlight_16 = PinDriver::output(peripherals.pins.gpio16)?;
    backlight_16.set_high()?;
    
    thread::sleep(Duration::from_millis(100));

    backlight_16.set_high()?;
    /* 
    // Setup pins for AtomS3 LCD
    let pin_sclk = peripherals.pins.gpio17;
    let pin_mosi = peripherals.pins.gpio21;
    let pin_cs = peripherals.pins.gpio15;
    let pin_dc = peripherals.pins.gpio33;
    
    // Setup reset pin
    let mut lcd_reset_pin = PinDriver::output(peripherals.pins.gpio34)?;

    info!("Issue LCD Reset");
    lcd_reset_pin.set_low()?;
    thread::sleep(Duration::from_millis(100));
    lcd_reset_pin.set_high()?;
    thread::sleep(Duration::from_millis(200));

    info!("SPI Master initialization with MODE_0");

    let spi_config = Config::new()
        .baudrate(26.MHz().into())
        .data_mode(MODE_0);  // Changed to MODE_0 per M5GFX

    let device = SpiDeviceDriver::new_single(
        peripherals.spi2,
        pin_sclk,
        pin_mosi,
        Option::<esp_idf_hal::gpio::AnyInputPin>::None,
        Some(pin_cs),
        &SpiDriverConfig::new(),
        &spi_config,
    )
    .context("Failed to create SPI device")?;

    let pin_dc_out = PinDriver::output(pin_dc)?;

    info!("SPI Display interface");
    // Create display interface with buffer
    let mut buffer = [0u8; 512];
    let di = SpiInterface::new(device, pin_dc_out, &mut buffer);

    let mut delay = Ets;

    info!("Display initialization");
    let mut display = Builder::new(ST7789, di)
        .reset_pin(lcd_reset_pin)
        .init(&mut delay)
        .map_err(|e| anyhow::anyhow!("Display init failed: {:?}", e))?;
    
    backlight_16.set_high()?;

    info!("Display initialized successfully!");

    // Clear screen to red
    info!("Filling screen with RED...");
    display.clear(Rgb565::RED)
        .map_err(|e| anyhow::anyhow!("Clear RED failed: {:?}", e))?;
    
    info!("RED should be visible now!");
    thread::sleep(Duration::from_secs(2));

    // Clear screen to cyan
    info!("Filling screen with CYAN...");
    display.clear(Rgb565::CYAN)
        .map_err(|e| anyhow::anyhow!("Clear CYAN failed: {:?}", e))?;
    
    info!("CYAN visible!");
    thread::sleep(Duration::from_secs(2));

    // Draw sample graphics
    display.clear(Rgb565::BLACK)
        .map_err(|e| anyhow::anyhow!("Clear failed: {:?}", e))?;
    
    draw_sample(&mut display);

    info!("Sample graphics drawn!");
    */
    // Keep display on
    loop {
        thread::sleep(Duration::from_secs(1));
    }
}
