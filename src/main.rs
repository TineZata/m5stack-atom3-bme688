use std::thread;
use std::time::Duration;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Primitive, PrimitiveStyle};
use embedded_hal::spi::MODE_0;
use esp_idf_hal::delay::Ets;
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::spi::{config::Config, SpiDeviceDriver, SpiDriverConfig};
use log::{info, error};
use mipidsi::{Builder, models::ST7789};
use mipidsi::interface::SpiInterface;
use mipidsi::options::{Orientation, Rotation};

// BME688/BME680 Register addresses
const BME688_ADDR: u8 = 0x76;
const BME688_ADDR_ALT: u8 = 0x77;  // Alternate address
const BME688_CHIP_ID_REG: u8 = 0xD0;
const BME688_RESET_REG: u8 = 0xE0;
const BME688_STATUS_REG: u8 = 0x1D;
const BME688_MEAS_STATUS_REG: u8 = 0x1D;  // Shows if measurement is in progress
const BME688_CTRL_HUM_REG: u8 = 0x72;
const BME688_CTRL_MEAS_REG: u8 = 0x74;
const BME688_CONFIG_REG: u8 = 0x75;
const BME688_PRESS_MSB_REG: u8 = 0xF7;

// Expected chip ID
const BME688_CHIP_ID: u8 = 0x61;

// Scan I2C bus for devices
fn i2c_scan(i2c: &mut I2cDriver) {
    info!("Scanning I2C bus...");
    let mut found = 0;
    for addr in 0x08..=0x77 {
        let mut dummy = [0u8; 1];
        if i2c.read(addr, &mut dummy, 100).is_ok() {
            info!("  Found device at address: 0x{:02X}", addr);
            found += 1;
        }
    }
    if found == 0 {
        error!("No I2C devices found!");
    } else {
        info!("Found {} I2C device(s)", found);
    }
}

// Minimal 5x7 pixel font (stored as 5 bytes per character)
// Each byte is a column, bits are pixels (1 = draw pixel)
const FONT_5X7: [[u8; 5]; 96] = [
    [0x00, 0x00, 0x00, 0x00, 0x00], // Space (32)
    [0x00, 0x00, 0x5F, 0x00, 0x00], // !
    [0x00, 0x07, 0x00, 0x07, 0x00], // "
    [0x14, 0x7F, 0x14, 0x7F, 0x14], // #
    [0x24, 0x2A, 0x7F, 0x2A, 0x12], // $
    [0x23, 0x13, 0x08, 0x64, 0x62], // %
    [0x36, 0x49, 0x55, 0x22, 0x50], // &
    [0x00, 0x05, 0x03, 0x00, 0x00], // '
    [0x00, 0x1C, 0x22, 0x41, 0x00], // (
    [0x00, 0x41, 0x22, 0x1C, 0x00], // )
    [0x14, 0x08, 0x3E, 0x08, 0x14], // *
    [0x08, 0x08, 0x3E, 0x08, 0x08], // +
    [0x00, 0x50, 0x30, 0x00, 0x00], // ,
    [0x08, 0x08, 0x08, 0x08, 0x08], // -
    [0x00, 0x60, 0x60, 0x00, 0x00], // .
    [0x20, 0x10, 0x08, 0x04, 0x02], // /
    [0x3E, 0x51, 0x49, 0x45, 0x3E], // 0
    [0x00, 0x42, 0x7F, 0x40, 0x00], // 1
    [0x42, 0x61, 0x51, 0x49, 0x46], // 2
    [0x21, 0x41, 0x45, 0x4B, 0x31], // 3
    [0x18, 0x14, 0x12, 0x7F, 0x10], // 4
    [0x27, 0x45, 0x45, 0x45, 0x39], // 5
    [0x3C, 0x4A, 0x49, 0x49, 0x30], // 6
    [0x01, 0x71, 0x09, 0x05, 0x03], // 7
    [0x36, 0x49, 0x49, 0x49, 0x36], // 8
    [0x06, 0x49, 0x49, 0x29, 0x1E], // 9
    [0x00, 0x36, 0x36, 0x00, 0x00], // :
    [0x00, 0x56, 0x36, 0x00, 0x00], // ;
    [0x08, 0x14, 0x22, 0x41, 0x00], // <
    [0x14, 0x14, 0x14, 0x14, 0x14], // =
    [0x00, 0x41, 0x22, 0x14, 0x08], // >
    [0x02, 0x01, 0x51, 0x09, 0x06], // ?
    [0x32, 0x49, 0x79, 0x41, 0x3E], // @
    [0x7E, 0x11, 0x11, 0x11, 0x7E], // A
    [0x7F, 0x49, 0x49, 0x49, 0x36], // B
    [0x3E, 0x41, 0x41, 0x41, 0x22], // C
    [0x7F, 0x41, 0x41, 0x22, 0x1C], // D
    [0x7F, 0x49, 0x49, 0x49, 0x41], // E
    [0x7F, 0x09, 0x09, 0x09, 0x01], // F
    [0x3E, 0x41, 0x49, 0x49, 0x7A], // G
    [0x7F, 0x08, 0x08, 0x08, 0x7F], // H
    [0x00, 0x41, 0x7F, 0x41, 0x00], // I
    [0x20, 0x40, 0x41, 0x3F, 0x01], // J
    [0x7F, 0x08, 0x14, 0x22, 0x41], // K
    [0x7F, 0x40, 0x40, 0x40, 0x40], // L
    [0x7F, 0x02, 0x0C, 0x02, 0x7F], // M
    [0x7F, 0x04, 0x08, 0x10, 0x7F], // N
    [0x3E, 0x41, 0x41, 0x41, 0x3E], // O
    [0x7F, 0x09, 0x09, 0x09, 0x06], // P
    [0x3E, 0x41, 0x51, 0x21, 0x5E], // Q
    [0x7F, 0x09, 0x19, 0x29, 0x46], // R
    [0x46, 0x49, 0x49, 0x49, 0x31], // S
    [0x01, 0x01, 0x7F, 0x01, 0x01], // T
    [0x3F, 0x40, 0x40, 0x40, 0x3F], // U
    [0x1F, 0x20, 0x40, 0x20, 0x1F], // V
    [0x3F, 0x40, 0x38, 0x40, 0x3F], // W
    [0x63, 0x14, 0x08, 0x14, 0x63], // X
    [0x07, 0x08, 0x70, 0x08, 0x07], // Y
    [0x61, 0x51, 0x49, 0x45, 0x43], // Z
    [0x00, 0x7F, 0x41, 0x41, 0x00], // [
    [0x02, 0x04, 0x08, 0x10, 0x20], // \
    [0x00, 0x41, 0x41, 0x7F, 0x00], // ]
    [0x04, 0x02, 0x01, 0x02, 0x04], // ^
    [0x40, 0x40, 0x40, 0x40, 0x40], // _
    [0x00, 0x01, 0x02, 0x04, 0x00], // `
    [0x20, 0x54, 0x54, 0x54, 0x78], // a
    [0x7F, 0x48, 0x44, 0x44, 0x38], // b
    [0x38, 0x44, 0x44, 0x44, 0x20], // c
    [0x38, 0x44, 0x44, 0x48, 0x7F], // d
    [0x38, 0x54, 0x54, 0x54, 0x18], // e
    [0x08, 0x7E, 0x09, 0x01, 0x02], // f
    [0x0C, 0x52, 0x52, 0x52, 0x3E], // g
    [0x7F, 0x08, 0x04, 0x04, 0x78], // h
    [0x00, 0x44, 0x7D, 0x40, 0x00], // i
    [0x20, 0x40, 0x44, 0x3D, 0x00], // j
    [0x7F, 0x10, 0x28, 0x44, 0x00], // k
    [0x00, 0x41, 0x7F, 0x40, 0x00], // l
    [0x7C, 0x04, 0x18, 0x04, 0x78], // m
    [0x7C, 0x08, 0x04, 0x04, 0x78], // n
    [0x38, 0x44, 0x44, 0x44, 0x38], // o
    [0x7C, 0x14, 0x14, 0x14, 0x08], // p
    [0x08, 0x14, 0x14, 0x18, 0x7C], // q
    [0x7C, 0x08, 0x04, 0x04, 0x08], // r
    [0x48, 0x54, 0x54, 0x54, 0x20], // s
    [0x04, 0x3F, 0x44, 0x40, 0x20], // t
    [0x3C, 0x40, 0x40, 0x20, 0x7C], // u
    [0x1C, 0x20, 0x40, 0x20, 0x1C], // v
    [0x3C, 0x40, 0x30, 0x40, 0x3C], // w
    [0x44, 0x28, 0x10, 0x28, 0x44], // x
    [0x0C, 0x50, 0x50, 0x50, 0x3C], // y
    [0x44, 0x64, 0x54, 0x4C, 0x44], // z
    [0x00, 0x08, 0x36, 0x41, 0x00], // {
    [0x00, 0x00, 0x7F, 0x00, 0x00], // |
    [0x00, 0x41, 0x36, 0x08, 0x00], // }
    [0x08, 0x08, 0x2A, 0x1C, 0x08], // ~
    [0x00, 0x00, 0x00, 0x00, 0x00], // DEL
];

// Draw a single character at x,y using minimal stack
fn draw_char<D>(display: &mut D, x: i32, y: i32, ch: char, color: Rgb565) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let code = ch as usize;
    if code < 32 || code > 127 {
        return Ok(()); // Skip non-printable
    }
    
    let glyph = &FONT_5X7[code - 32];
    
    // Draw each column of the character
    for col in 0..5 {
        let column_data = glyph[col];
        // Draw each pixel in the column
        for row in 0..8 {
            if (column_data & (1 << row)) != 0 {
                let px = embedded_graphics::primitives::Rectangle::new(
                    Point::new(x + col as i32, y + row as i32),
                    embedded_graphics::geometry::Size::new(1, 1)
                )
                .into_styled(PrimitiveStyle::with_fill(color));
                px.draw(display)?;
            }
        }
    }
    
    Ok(())
}

// Draw text string character-by-character (minimal stack usage)
fn draw_text<D>(display: &mut D, x: i32, y: i32, text: &str, color: Rgb565) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let mut offset = 0;
    for ch in text.chars() {
        draw_char(display, x + offset, y, ch, color)?;
        offset += 6; // 5px width + 1px spacing
    }
    Ok(())
}

// I2C helper functions for BME688 (with extended delays for stability)
fn bme688_read_reg(i2c: &mut I2cDriver, addr: u8, reg: u8) -> Result<u8, esp_idf_svc::sys::EspError> {
    let mut buf = [0u8; 1];
    i2c.write_read(addr, &[reg], &mut buf, 1000)?;
    thread::sleep(Duration::from_millis(10));  // Increased delay
    Ok(buf[0])
}

fn bme688_write_reg(i2c: &mut I2cDriver, addr: u8, reg: u8, value: u8) -> Result<(), esp_idf_svc::sys::EspError> {
    i2c.write(addr, &[reg, value], 1000)?;
    thread::sleep(Duration::from_millis(10));  // Increased delay
    Ok(())
}

fn bme688_read_burst(i2c: &mut I2cDriver, addr: u8, reg: u8, buf: &mut [u8]) -> Result<(), esp_idf_svc::sys::EspError> {
    i2c.write_read(addr, &[reg], buf, 1000)?;
    thread::sleep(Duration::from_millis(10));  // Increased delay
    Ok(())
}

// Initialize BME688 sensor
fn bme688_init(i2c: &mut I2cDriver, addr: u8) -> Result<(), esp_idf_svc::sys::EspError> {
    // Read chip ID
    let chip_id = bme688_read_reg(i2c, addr, BME688_CHIP_ID_REG)?;
    info!("BME688 Chip ID: 0x{:02X} (expected 0x61)", chip_id);
    
    if chip_id != BME688_CHIP_ID {
        error!("BME688 chip ID mismatch! Got 0x{:02X}, expected 0x61", chip_id);
        // Just continue anyway, might be BME680 which is compatible
    }
    
    // Soft reset
    info!("Resetting BME688...");
    bme688_write_reg(i2c, addr, BME688_RESET_REG, 0xB6)?;
    thread::sleep(Duration::from_millis(100));  // Wait for reset
    
    // Configure humidity oversampling (2x)
    bme688_write_reg(i2c, addr, BME688_CTRL_HUM_REG, 0x02)?;
    thread::sleep(Duration::from_millis(20));
    
    // Configure temperature and pressure oversampling, and set to forced mode
    // [7:5] = temperature oversampling (010 = 2x)
    // [4:2] = pressure oversampling (101 = 16x)  
    // [1:0] = power mode (01 = forced mode)
    bme688_write_reg(i2c, addr, BME688_CTRL_MEAS_REG, 0x55)?;
    thread::sleep(Duration::from_millis(20));
    
    // Configure IIR filter (coefficient 3)
    bme688_write_reg(i2c, addr, BME688_CONFIG_REG, 0x10)?;
    thread::sleep(Duration::from_millis(20));
    
    info!("BME688 initialized successfully");
    Ok(())
}

// Read raw sensor data from BME688
fn bme688_read_raw(i2c: &mut I2cDriver, addr: u8) -> Result<(u32, u32, u16), esp_idf_svc::sys::EspError> {
    // Trigger measurement (forced mode)
    bme688_write_reg(i2c, addr, BME688_CTRL_MEAS_REG, 0x55)?;
    
    // Wait for measurement with status checking (16x pressure oversampling needs time)
    thread::sleep(Duration::from_millis(100));
    
    // Poll status register to check if measurement is ready (max 30 attempts)
    for _ in 0..30 {
        let status = bme688_read_reg(i2c, addr, BME688_MEAS_STATUS_REG)?;
        if (status & 0x80) == 0 {  // Bit 7 = 0 means measurement complete
            break;
        }
        thread::sleep(Duration::from_millis(20));
    }
    
    // Read 8 bytes starting from pressure MSB register
    static mut DATA_BUF: [u8; 8] = [0u8; 8];
    unsafe {
        bme688_read_burst(i2c, addr, BME688_PRESS_MSB_REG, &mut DATA_BUF)?;
        
        // Parse pressure (20-bit)
        let press_raw = ((DATA_BUF[0] as u32) << 12) | ((DATA_BUF[1] as u32) << 4) | ((DATA_BUF[2] as u32) >> 4);
        
        // Parse temperature (20-bit)
        let temp_raw = ((DATA_BUF[3] as u32) << 12) | ((DATA_BUF[4] as u32) << 4) | ((DATA_BUF[5] as u32) >> 4);
        
        // Parse humidity (16-bit)
        let hum_raw = ((DATA_BUF[6] as u16) << 8) | (DATA_BUF[7] as u16);
        
        Ok((press_raw, temp_raw, hum_raw))
    }
}

fn main() {
    // Initialize system
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("=== AtomS3 Display Clear Test ===");
    
    let peripherals = Peripherals::take().expect("Failed to get peripherals");
    info!("Peripherals OK!");

    // Enable backlight - GPIO16 confirmed working
    info!("Enabling backlight on GPIO16");
    let mut backlight = PinDriver::output(peripherals.pins.gpio16).expect("backlight pin");
    backlight.set_high().expect("backlight on");
    
    thread::sleep(Duration::from_millis(100));
    
    // Setup LCD pins
    let pin_sclk = peripherals.pins.gpio17;
    let pin_mosi = peripherals.pins.gpio21;
    let pin_cs = peripherals.pins.gpio15;
    let pin_dc = peripherals.pins.gpio33;
    let mut lcd_reset = PinDriver::output(peripherals.pins.gpio34).expect("reset pin");

    // Reset LCD
    info!("LCD Reset");
    lcd_reset.set_low().expect("reset low");
    thread::sleep(Duration::from_millis(100));
    lcd_reset.set_high().expect("reset high");
    thread::sleep(Duration::from_millis(200));

    // Setup SPI with MODE_0 (per M5GFX)
    info!("SPI init with MODE_0");
    let spi_config = Config::new()
        .baudrate(26.MHz().into())
        .data_mode(MODE_0);

    let device = SpiDeviceDriver::new_single(
        peripherals.spi2,
        pin_sclk,
        pin_mosi,
        Option::<esp_idf_hal::gpio::AnyInputPin>::None,
        Some(pin_cs),
        &SpiDriverConfig::new(),
        &spi_config,
    ).expect("SPI device");

    let pin_dc_out = PinDriver::output(pin_dc).expect("DC pin");
    
    // Create display interface with buffer
    let mut buffer = [0u8; 512];
    let di = SpiInterface::new(device, pin_dc_out, &mut buffer);

    let mut delay = Ets;

    info!("Initializing display");
    let mut display = Builder::new(ST7789, di)
        .reset_pin(lcd_reset)
        .display_size(128, 128)  // AtomS3 is 128x128
        .display_offset(0, 0)    // No offset
        .init(&mut delay)
        .expect("display init");

    
    // Clear screen to black
    info!("Clearing display to black");
    display.clear(Rgb565::BLACK).expect("clear");
    
    info!("Display cleared! Should see solid black screen.");
    display.set_orientation(Orientation::default().rotate(Rotation::Deg180)).unwrap();
    
    // Initialize I2C for BME688 sensor (native ESP-IDF I2C)
    info!("Initializing I2C for BME688 (SDA=GPIO2, SCL=GPIO1)");
    let mut i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio2,  // SDA
        peripherals.pins.gpio1,  // SCL
        &I2cConfig::new().baudrate(100.kHz().into()),
    ).expect("I2C driver init");
    
    // Scan I2C bus first
    i2c_scan(&mut i2c);
    
    // Try to initialize BME688 sensor with native register access
    info!("Initializing BME688 sensor...");
    let sensor_addr = match bme688_init(&mut i2c, BME688_ADDR) {
        Ok(_) => {
            info!("BME688 found at 0x{:02X}!", BME688_ADDR);
            BME688_ADDR
        }
        Err(e) => {
            error!("BME688 not found at 0x76, trying 0x77... Error: {:?}", e);
            match bme688_init(&mut i2c, BME688_ADDR_ALT) {
                Ok(_) => {
                    info!("BME688 found at 0x{:02X}!", BME688_ADDR_ALT);
                    BME688_ADDR_ALT
                }
                Err(e2) => {
                    error!("BME688 init failed at both addresses: {:?}", e2);
                    display.clear(Rgb565::BLACK).expect("clear");
                    draw_text(&mut display, 10, 20, "No BME688", Rgb565::RED).unwrap();
                    draw_text(&mut display, 10, 40, "Check wiring", Rgb565::YELLOW).unwrap();
                    draw_text(&mut display, 10, 60, "SDA=GPIO2", Rgb565::WHITE).unwrap();
                    draw_text(&mut display, 10, 80, "SCL=GPIO1", Rgb565::WHITE).unwrap();
                    loop {
                        thread::sleep(Duration::from_secs(1));
                    }
                }
            }
        }
    };
    
    display.clear(Rgb565::BLACK).expect("clear");
    draw_text(&mut display, 10, 10, "BME688 Ready!", Rgb565::GREEN).unwrap();
    thread::sleep(Duration::from_millis(1000));
    
    // Main sensor reading loop
    info!("Starting sensor loop");
    
    loop {
        // Read raw sensor data
        let result = bme688_read_raw(&mut i2c, sensor_addr);
        
        if let Ok((p, t, h)) = result {
            // Calculate values (as integers to avoid float formatting overhead)
            let temp_c = ((t as f32) / 5120.0 - 50.0) as i32;
            let press_val = ((p as f32) / 256.0) as u32;
            let hum_pct = ((h as f32) / 512.0) as u32;
            
            // Clear and update display (using heapless for less stack usage)
            display.clear(Rgb565::BLACK).ok();
            draw_text(&mut display, 5, 5, "BME688", Rgb565::WHITE).ok();
            
            // Format strings using heapless (stack-allocated, minimal overhead)
            use core::fmt::Write;
            let mut temp_str = heapless::String::<16>::new();
            let mut press_str = heapless::String::<16>::new();
            let mut hum_str = heapless::String::<16>::new();
            
            let _ = write!(temp_str, "{}C", temp_c);
            let _ = write!(press_str, "{}", press_val);
            let _ = write!(hum_str, "{}%", hum_pct);
            
            // Display
            draw_text(&mut display, 5, 25, "T:", Rgb565::GREEN).ok();
            draw_text(&mut display, 25, 25, temp_str.as_str(), Rgb565::GREEN).ok();
            
            draw_text(&mut display, 5, 45, "P:", Rgb565::CYAN).ok();
            draw_text(&mut display, 25, 45, press_str.as_str(), Rgb565::CYAN).ok();
            
            draw_text(&mut display, 5, 65, "H:", Rgb565::YELLOW).ok();
            draw_text(&mut display, 25, 65, hum_str.as_str(), Rgb565::YELLOW).ok();
        } else {
            error!("Read failed");
            display.clear(Rgb565::BLACK).ok();
            draw_text(&mut display, 10, 30, "Read Error", Rgb565::RED).ok();
        }
        
        // Longer wait between readings for stability
        thread::sleep(Duration::from_millis(2000));
    }
}
