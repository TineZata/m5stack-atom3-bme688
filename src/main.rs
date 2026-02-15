use std::thread;
use std::time::Duration;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Primitive, PrimitiveStyle};
use embedded_hal::spi::MODE_0;
use esp_idf_hal::delay::Ets;
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::ledc::{LedcDriver, LedcTimerDriver, config::TimerConfig};
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
const BME688_PRESS_MSB_REG: u8 = 0x1F;  // Field 0 press_msb (NOT 0xF7 which is BME280)
const BME688_CTRL_GAS_0_REG: u8 = 0x70;
const BME688_CTRL_GAS_1_REG: u8 = 0x71;
const BME688_GAS_R_MSB_LOW_REG: u8 = 0x2A;  // Field 0 gas_r_msb (BME680 / low-gas variant)
const BME688_GAS_R_MSB_HIGH_REG: u8 = 0x2C; // Field 0 gas_r_msb (BME688 / high-gas variant)
const BME688_RES_HEAT_0_REG: u8 = 0x5A;
const BME688_GAS_WAIT_0_REG: u8 = 0x64;

// Expected chip ID
const BME688_CHIP_ID: u8 = 0x61;

/// Gas range lookup table for resistance calculation (from Bosch datasheet)
const GAS_RANGE_K1: [f32; 16] = [
    1.0, 1.0, 1.0, 1.0, 1.0, 0.99, 1.0, 0.992,
    1.0, 1.0, 0.998, 0.995, 1.0, 0.99, 1.0, 1.0,
];
const GAS_RANGE_K2: [f32; 16] = [
    8000000.0, 4000000.0, 2000000.0, 1000000.0,
    499500.4995, 248262.1648, 125000.0, 63004.03226,
    31281.28128, 15625.0, 7812.5, 3906.25,
    1953.125, 976.5625, 488.28125, 244.140625,
];

// Backlight brightness: constant 50% PWM to save power
const BACKLIGHT_PCT: u32 = 50;

// Total cycle time between sensor reads (seconds between display updates)
const CYCLE_INTERVAL_MS: u64 = 3_000;

/// Factory calibration coefficients read from the BME688's NVM.
/// These are unique per chip and required for accurate readings.
#[derive(Debug, Default)]
struct Bme688Calib {
    // Temperature
    par_t1: u16,
    par_t2: i16,
    par_t3: i8,
    // Pressure
    par_p1: u16,
    par_p2: i16,
    par_p3: i8,
    par_p4: i16,
    par_p5: i16,
    par_p6: i8,
    par_p7: i8,
    par_p8: i16,
    par_p9: i16,
    par_p10: u8,
    // Humidity
    par_h1: u16,
    par_h2: u16,
    par_h3: i8,
    par_h4: i8,
    par_h5: i8,
    par_h6: u8,
    par_h7: i8,
    // Gas heater
    par_gh1: i8,
    par_gh2: i16,
    par_gh3: i8,
    res_heat_range: u8,
    res_heat_val: i8,
    range_sw_err: i8,
    // Chip variant: 0x00 = BME680 (gas low), 0x01 = BME688 (gas high)
    variant_id: u8,
    // Intermediate temperature value used in pressure/humidity compensation
    t_fine: f32,
}

/// Read calibration data from BME688 NVM registers
fn bme688_read_calib(i2c: &mut I2cDriver, addr: u8) -> Result<Bme688Calib, esp_idf_svc::sys::EspError> {
    // Read calibration bank 1: registers 0x89..0xA1 (25 bytes)
    let mut coeff1 = [0u8; 25];
    bme688_read_burst(i2c, addr, 0x89, &mut coeff1)?;

    // Read calibration bank 2: registers 0xE1..0xF0 (16 bytes)
    let mut coeff2 = [0u8; 16];
    bme688_read_burst(i2c, addr, 0xE1, &mut coeff2)?;

    let calib = Bme688Calib {
        // Temperature calibration
        // par_t1 lives in bank 2 at registers 0xE9 (LSB) / 0xEA (MSB)
        par_t1: (coeff2[9] as u16) << 8 | coeff2[8] as u16,
        // par_t2 at 0x8A (LSB) / 0x8B (MSB) => coeff1[1] / coeff1[2]
        par_t2: ((coeff1[2] as u16) << 8 | coeff1[1] as u16) as i16,
        // par_t3 at 0x8C => coeff1[3]
        par_t3: coeff1[3] as i8,

        // Pressure calibration (from bank 1)
        // par_p1 at 0x8E (LSB) / 0x8F (MSB) => coeff1[5] / coeff1[6]
        par_p1: (coeff1[6] as u16) << 8 | coeff1[5] as u16,
        // par_p2 at 0x90 (LSB) / 0x91 (MSB) => coeff1[7] / coeff1[8]
        par_p2: ((coeff1[8] as u16) << 8 | coeff1[7] as u16) as i16,
        // par_p3 at 0x92 => coeff1[9]
        par_p3: coeff1[9] as i8,
        par_p4: ((coeff1[12] as u16) << 8 | coeff1[11] as u16) as i16,
        par_p5: ((coeff1[14] as u16) << 8 | coeff1[13] as u16) as i16,
        // par_p6 at 0x98 => coeff1[15], par_p7 at 0x99 => coeff1[16]
        par_p6: coeff1[15] as i8,
        par_p7: coeff1[16] as i8,
        par_p8: ((coeff1[20] as u16) << 8 | coeff1[19] as u16) as i16,
        par_p9: ((coeff1[22] as u16) << 8 | coeff1[21] as u16) as i16,
        par_p10: coeff1[23],

        // Humidity calibration (from bank 2)
        // par_h1: MSB at 0xE3 (coeff2[2]), LSB nibble at 0xE2 lower (coeff2[1])
        par_h1: ((coeff2[2] as u16) << 4) | ((coeff2[1] as u16) & 0x0F),
        // par_h2: MSB at 0xE1 (coeff2[0]), LSB nibble at 0xE2 upper (coeff2[1])
        par_h2: ((coeff2[0] as u16) << 4) | ((coeff2[1] as u16) >> 4),
        par_h3: coeff2[3] as i8,
        par_h4: coeff2[4] as i8,
        par_h5: coeff2[5] as i8,
        par_h6: coeff2[6],
        par_h7: coeff2[7] as i8,

        // Gas heater calibration
        // par_gh1 at 0xED => coeff2[12], par_gh2 at 0xEB/0xEC => coeff2[10..12], par_gh3 at 0xEE => coeff2[13]
        par_gh1: coeff2[12] as i8,
        par_gh2: ((coeff2[11] as u16) << 8 | coeff2[10] as u16) as i16,
        par_gh3: coeff2[13] as i8,
        // res_heat_range at register 0x02 bits [5:4]
        res_heat_range: (bme688_read_reg(i2c, addr, 0x02)? >> 4) & 0x03,
        // res_heat_val at register 0x00
        res_heat_val: bme688_read_reg(i2c, addr, 0x00)? as i8,
        // range_sw_err at register 0x04 bits [7:4] (signed)
        range_sw_err: ((bme688_read_reg(i2c, addr, 0x04)? as i8) >> 4),
        // variant_id at register 0xF0: 0x00=BME680, 0x01=BME688
        variant_id: bme688_read_reg(i2c, addr, 0xF0)?,

        t_fine: 0.0,
    };

    info!("BME688 calibration: T1={} T2={} T3={}", calib.par_t1, calib.par_t2, calib.par_t3);
    info!("BME688 calibration: P1={} P2={} P5={}", calib.par_p1, calib.par_p2, calib.par_p5);
    info!("BME688 calibration: H1={} H2={}", calib.par_h1, calib.par_h2);
    info!("BME688 gas calib: GH1={} GH2={} GH3={} range={} val={} err={} variant=0x{:02X}",
          calib.par_gh1, calib.par_gh2, calib.par_gh3,
          calib.res_heat_range, calib.res_heat_val, calib.range_sw_err, calib.variant_id);

    Ok(calib)
}

/// Compensate raw temperature using Bosch algorithm.
/// Returns temperature in °C * 100 (e.g. 2170 = 21.70°C)
fn compensate_temperature(calib: &mut Bme688Calib, temp_adc: u32) -> i32 {
    let var1 = ((temp_adc as f32 / 16384.0) - (calib.par_t1 as f32 / 1024.0))
        * calib.par_t2 as f32;
    let var2 = ((temp_adc as f32 / 131072.0) - (calib.par_t1 as f32 / 8192.0))
        * ((temp_adc as f32 / 131072.0) - (calib.par_t1 as f32 / 8192.0))
        * (calib.par_t3 as f32 * 16.0);
    calib.t_fine = var1 + var2;
    (calib.t_fine / 5120.0 * 100.0) as i32
}

/// Compensate raw pressure using Bosch algorithm.
/// Returns pressure in Pa * 100 (e.g. 10010000 = 1001.00 hPa)
fn compensate_pressure(calib: &Bme688Calib, press_adc: u32) -> u32 {
    let var1 = (calib.t_fine / 2.0) - 64000.0;
    let var2 = var1 * var1 * (calib.par_p6 as f32 / 131072.0);
    let var2 = var2 + var1 * calib.par_p5 as f32 * 2.0;
    let var2 = (var2 / 4.0) + (calib.par_p4 as f32 * 65536.0);
    let var1 = (((calib.par_p3 as f32 * var1 * var1) / 16384.0)
        + (calib.par_p2 as f32 * var1)) / 524288.0;
    let var1 = (1.0 + (var1 / 32768.0)) * calib.par_p1 as f32;

    if var1 == 0.0 {
        return 0;
    }

    let press = 1048576.0 - press_adc as f32;
    let press = ((press - (var2 / 4096.0)) * 6250.0) / var1;
    let var1 = (calib.par_p9 as f32 * press * press) / 2147483648.0;
    let var2 = press * (calib.par_p8 as f32 / 32768.0);
    let var3 = (press / 256.0) * (press / 256.0) * (press / 256.0)
        * (calib.par_p10 as f32 / 131072.0);
    let press = press + (var1 + var2 + var3 + (calib.par_p7 as f32 * 128.0)) / 16.0;

    // Return Pa * 100
    (press * 100.0) as u32
}

/// Compensate raw humidity using Bosch algorithm.
/// Returns humidity in %RH * 100 (e.g. 4800 = 48.00%)
fn compensate_humidity(calib: &Bme688Calib, hum_adc: u16) -> u32 {
    let temp_comp = calib.t_fine / 5120.0;
    let var1 = hum_adc as f32
        - ((calib.par_h1 as f32 * 16.0)
            + ((calib.par_h3 as f32 / 2.0) * temp_comp));
    let var2 = var1
        * ((calib.par_h2 as f32 / 262144.0)
            * (1.0
                + ((calib.par_h4 as f32 / 16384.0) * temp_comp)
                + ((calib.par_h5 as f32 / 1048576.0) * temp_comp * temp_comp)));
    let var3 = calib.par_h6 as f32 / 16384.0;
    let var4 = calib.par_h7 as f32 / 2097152.0;
    let hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);

    let hum = if hum > 100.0 { 100.0 } else if hum < 0.0 { 0.0 } else { hum };
    // Return %RH * 100
    (hum * 100.0) as u32
}

/// Calculate the gas heater resistance register value for a target temperature.
/// amb_temp is the ambient temperature in °C (integer).
fn calc_res_heat(calib: &Bme688Calib, target_temp: u16, amb_temp: i32) -> u8 {
    // Bosch reference: bme68x.c calc_res_heat()
    let var1 = ((amb_temp as f32 * calib.par_gh3 as f32) / 1000.0) * 256.0;
    let var2 = (calib.par_gh1 as f32 + 784.0)
        * ((((calib.par_gh2 as f32 + 154009.0) * target_temp as f32 * 5.0) / 100.0)
            + 3276800.0)
        / 10.0;
    let var3 = var1 + (var2 / 2.0);
    let var4 = var3 / (calib.res_heat_range as f32 + 4.0);
    let var5 = (131.0 * calib.res_heat_val as f32) + 65536.0;
    let res_heat_x100 = ((var4 / var5) - 250.0) * 34.0;
    ((res_heat_x100 + 50.0) / 100.0) as u8
}

/// Calculate gas resistance in Ohms from ADC value and gas range.
/// BME688 (variant 0x01) uses a simplified formula; BME680 uses lookup tables.
fn calc_gas_resistance(gas_adc: u16, gas_range: u8, variant_id: u8, range_sw_err: i8) -> f32 {
    if variant_id == 0x01 {
        // BME688 (high gas variant) — simplified formula from Bosch BME68x API
        let var1 = (262144u32 >> gas_range) as f32;
        let var2 = (gas_adc as f32 - 512.0) * 3.0 + 4096.0;
        1000000.0 * var1 / var2
    } else {
        // BME680 (low gas variant) — uses lookup tables
        let gas_range_f = (1u32 << gas_range) as f32;
        let var1 = 1340.0 + (5.0 * range_sw_err as f32);
        let var2 = var1 * (1.0 + GAS_RANGE_K1[gas_range as usize] / 100.0);
        let var3 = 1.0 + (GAS_RANGE_K2[gas_range as usize] / gas_adc as f32);
        1.0 / (var3 * 0.000000125 * gas_range_f * var2)
    }
}

/// Map gas resistance to a simple Indoor Air Quality (IAQ) score 0..500.
/// Lower resistance = more VOCs = worse air. This is a heuristic estimation;
/// Bosch's official BSEC library uses a proprietary algorithm.
///
/// Rough mapping:  >250 kOhm => excellent (IAQ ~25)
///                 50-250 kOhm => good (IAQ 25-100)
///                 10-50 kOhm => moderate (IAQ 100-200)
///                 5-10 kOhm => poor (IAQ 200-300)
///                 <5 kOhm => very poor (IAQ 300-500)
fn estimate_iaq(gas_resistance: f32, humidity: f32) -> u16 {
    // Gas contribution (75% weight) - logarithmic mapping
    let gas_score = if gas_resistance >= 250000.0 {
        0.0  // excellent
    } else if gas_resistance >= 50000.0 {
        // 250k..50k => 0..100
        let ratio = (250000.0 - gas_resistance) / 200000.0;
        ratio * 100.0
    } else if gas_resistance >= 10000.0 {
        // 50k..10k => 100..200
        let ratio = (50000.0 - gas_resistance) / 40000.0;
        100.0 + ratio * 100.0
    } else if gas_resistance >= 5000.0 {
        // 10k..5k => 200..300
        let ratio = (10000.0 - gas_resistance) / 5000.0;
        200.0 + ratio * 100.0
    } else {
        // <5k => 300..500
        let ratio = ((5000.0 - gas_resistance) / 5000.0).min(1.0);
        300.0 + ratio * 200.0
    };

    // Humidity contribution (25% weight) - optimal is ~40% RH
    let hum_offset = (humidity - 40.0).abs();
    let hum_score = if hum_offset <= 10.0 {
        0.0
    } else {
        (hum_offset - 10.0) * 2.0  // penalty for deviation
    };

    let iaq = (gas_score * 0.75 + hum_score * 0.25).max(0.0).min(500.0);
    iaq as u16
}

/// Get IAQ label text for a given IAQ score
fn iaq_label(iaq: u16) -> &'static str {
    match iaq {
        0..=50 => "Excellent",
        51..=100 => "Good",
        101..=150 => "Lightly Polluted",
        151..=200 => "Moderately Polluted",
        201..=250 => "Heavily Polluted",
        251..=350 => "Severely Polluted",
        _ => "Hazardous",
    }
}

/// Get IAQ display color
fn iaq_color(iaq: u16) -> Rgb565 {
    match iaq {
        0..=50 => Rgb565::GREEN,
        51..=100 => Rgb565::new(12, 63, 0),   // light green
        101..=150 => Rgb565::YELLOW,
        151..=200 => Rgb565::new(31, 30, 0),   // orange
        201..=300 => Rgb565::RED,
        _ => Rgb565::new(16, 0, 16),           // purple
    }
}


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
    
    // Configure temperature and pressure oversampling (stay in sleep mode)
    // [7:5] = temperature oversampling (010 = 2x)
    // [4:2] = pressure oversampling (101 = 16x)  
    // [1:0] = power mode (00 = sleep) — forced-mode trigger happens in read_raw
    bme688_write_reg(i2c, addr, BME688_CTRL_MEAS_REG, 0x54)?;
    thread::sleep(Duration::from_millis(20));
    
    // Configure IIR filter (coefficient 3)
    bme688_write_reg(i2c, addr, BME688_CONFIG_REG, 0x10)?;
    thread::sleep(Duration::from_millis(20));
    
    // Don't disable heater (ctrl_gas_0 = 0x00)
    bme688_write_reg(i2c, addr, BME688_CTRL_GAS_0_REG, 0x00)?;
    thread::sleep(Duration::from_millis(20));
    
    // Note: ctrl_gas_1 (run_gas) is set after calibration read,
    // since BME680 vs BME688 require different run_gas bit values.
    
    info!("BME688 initialized successfully");
    Ok(())
}

/// Configure gas heater for a measurement cycle.
/// Must be called after calibration is read and before each forced-mode trigger.
fn bme688_setup_gas_heater(i2c: &mut I2cDriver, addr: u8, calib: &Bme688Calib, amb_temp: i32) -> Result<(), esp_idf_svc::sys::EspError> {
    // Target heater temperature 300°C per datasheet §3.2 example
    let res_heat = calc_res_heat(calib, 300, amb_temp);
    bme688_write_reg(i2c, addr, BME688_RES_HEAT_0_REG, res_heat)?;
    // Gas wait time: 0x59 = 100ms per datasheet §3.2 example
    // Encoding: [7:6] = multiplication factor (00=1x, 01=4x, 10=16x, 11=64x), [5:0] = base
    // 0x59 = 0b01_011001 => factor=4x, base=25 => 4*25 = 100ms
    bme688_write_reg(i2c, addr, BME688_GAS_WAIT_0_REG, 0x59)?;
    // Enable gas: run_gas for BME680 (0x10) or BME688 (0x20), nb_conv=0
    let run_gas = if calib.variant_id == 0x01 { 0x20u8 } else { 0x10u8 };
    bme688_write_reg(i2c, addr, BME688_CTRL_GAS_1_REG, run_gas)?;
    info!("Gas heater configured: res_heat={}, target=300C, wait=100ms, run_gas=0x{:02X}, variant=0x{:02X}", res_heat, run_gas, calib.variant_id);
    Ok(())
}

// Read raw sensor data from BME688 (including gas)
// Returns (press_raw, temp_raw, hum_raw, gas_adc, gas_range, gas_valid, heat_stab)
fn bme688_read_raw(i2c: &mut I2cDriver, addr: u8, variant_id: u8) -> Result<(u32, u32, u16, u16, u8, bool, bool), esp_idf_svc::sys::EspError> {
    // Re-arm gas measurement before triggering forced mode.
    // BME688 (variant 0x01) uses run_gas bit 5; BME680 uses bit 4.
    let run_gas = if variant_id == 0x01 { 0x20u8 } else { 0x10u8 };
    bme688_write_reg(i2c, addr, BME688_CTRL_GAS_1_REG, run_gas)?;

    // Trigger measurement (forced mode) - must re-assert forced mode each cycle
    bme688_write_reg(i2c, addr, BME688_CTRL_MEAS_REG, 0x55)?;
    
    // Wait for measurement with status checking (16x pressure oversampling + gas heating needs time)
    thread::sleep(Duration::from_millis(250));
    
    // Poll status register to check if measurement is ready (max 50 attempts)
    for _ in 0..50 {
        let status = bme688_read_reg(i2c, addr, BME688_MEAS_STATUS_REG)?;
        if (status & 0x80) != 0 {  // Bit 7 = 1 means new data ready
            break;
        }
        thread::sleep(Duration::from_millis(20));
    }
    
    // Read TPH: 8 bytes starting from press_msb (0x1F..0x26)
    static mut DATA_BUF: [u8; 8] = [0u8; 8];
    // Gas registers differ by variant:
    //   BME680 (low-gas):  0x2A-0x2B
    //   BME688 (high-gas): 0x2C-0x2D
    static mut GAS_BUF: [u8; 2] = [0u8; 2];
    let gas_msb_reg = if variant_id == 0x01 {
        BME688_GAS_R_MSB_HIGH_REG
    } else {
        BME688_GAS_R_MSB_LOW_REG
    };
    unsafe {
        bme688_read_burst(i2c, addr, BME688_PRESS_MSB_REG, &mut DATA_BUF)?;
        bme688_read_burst(i2c, addr, gas_msb_reg, &mut GAS_BUF)?;
        
        // Parse pressure (20-bit)
        let press_raw = ((DATA_BUF[0] as u32) << 12) | ((DATA_BUF[1] as u32) << 4) | ((DATA_BUF[2] as u32) >> 4);
        
        // Parse temperature (20-bit)
        let temp_raw = ((DATA_BUF[3] as u32) << 12) | ((DATA_BUF[4] as u32) << 4) | ((DATA_BUF[5] as u32) >> 4);
        
        // Parse humidity (16-bit)
        let hum_raw = ((DATA_BUF[6] as u16) << 8) | (DATA_BUF[7] as u16);
        
        // Parse gas resistance (10-bit ADC) and gas range (4-bit)
        // gas_r_msb[7:0] = gas ADC [9:2]
        // gas_r_lsb: [7:6]=gas_r<1:0>, [5]=gas_valid_r, [4]=heat_stab_r, [3:0]=gas_range_r
        let gas_adc = ((GAS_BUF[0] as u16) << 2) | ((GAS_BUF[1] as u16) >> 6);
        let gas_range = GAS_BUF[1] & 0x0F;
        let gas_valid = (GAS_BUF[1] & 0x20) != 0;   // bit 5: gas measurement completed
        let heat_stab = (GAS_BUF[1] & 0x10) != 0;    // bit 4: heater reached target temp
        
        info!("Gas raw: reg=0x{:02X} regs=[0x{:02X},0x{:02X}] adc={} range={} valid={} heat_stab={}",
              gas_msb_reg, GAS_BUF[0], GAS_BUF[1], gas_adc, gas_range, gas_valid, heat_stab);
        
        Ok((press_raw, temp_raw, hum_raw, gas_adc, gas_range, gas_valid, heat_stab))
    }
}

fn log_memory_stats(stage: &str) {
    unsafe {
        use esp_idf_svc::sys::{
            heap_caps_get_free_size,
            heap_caps_get_minimum_free_size,
            uxTaskGetStackHighWaterMark,
            xTaskGetCurrentTaskHandle,
            CONFIG_ESP_MAIN_TASK_STACK_SIZE,
            MALLOC_CAP_8BIT,
        };

        let stack_free_min = uxTaskGetStackHighWaterMark(xTaskGetCurrentTaskHandle()) as usize;
        let stack_cfg = CONFIG_ESP_MAIN_TASK_STACK_SIZE as usize;
        let stack_peak_used = stack_cfg.saturating_sub(stack_free_min);

        let heap_free = heap_caps_get_free_size(MALLOC_CAP_8BIT) as usize;
        let heap_min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT) as usize;

        info!("===== MEMORY: {} =====", stage);
        info!("Main stack configured: {} bytes", stack_cfg);
        info!("Main stack min-free watermark: {} bytes", stack_free_min);
        let peak_tenths_pct = if stack_cfg > 0 {
            (stack_peak_used * 1000) / stack_cfg
        } else {
            0
        };
        info!(
            "Main stack peak usage: {} bytes ({}.{:01}%)",
            stack_peak_used,
            peak_tenths_pct / 10,
            peak_tenths_pct % 10
        );
        info!("Heap free now (8-bit): {} bytes", heap_free);
        info!("Heap minimum ever free (8-bit): {} bytes", heap_min_free);
        info!("==========================");
    }
}

fn main() {
    // Initialize system
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("=== AtomS3 Display Clear Test ===");
    
        log_memory_stats("startup");
    
    let peripherals = Peripherals::take().expect("Failed to get peripherals");
    info!("Peripherals OK!");
    
    log_memory_stats("after Peripherals::take");

    // Setup backlight as PWM at 50% to save power
    info!("Enabling backlight PWM on GPIO16 at {}%", BACKLIGHT_PCT);
    let timer_config = TimerConfig::default().frequency(1.kHz().into());
    let timer = LedcTimerDriver::new(peripherals.ledc.timer0, &timer_config).expect("LEDC timer");
    let mut backlight = LedcDriver::new(peripherals.ledc.channel0, timer, peripherals.pins.gpio16).expect("LEDC backlight");
    let max_duty = backlight.get_max_duty();
    backlight.set_duty(max_duty * BACKLIGHT_PCT / 100).expect("backlight on");
    
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

    // Setup SPI with MODE_0 (per AtomS3/M5GFX behavior)
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
    let mut buffer = Box::new([0u8; 512]);
    let di = SpiInterface::new(device, pin_dc_out, buffer.as_mut());

    let mut delay = Ets;

    log_memory_stats("before display init");

    info!("Initializing display");
    let mut display = Builder::new(ST7789, di)
        .reset_pin(lcd_reset)
        .display_size(128, 128)  // AtomS3 is 128x128
        .display_offset(0, 0)    // No offset
        .init(&mut delay)
        .expect("display init");

    log_memory_stats("after display init");
    
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
    
    // Read factory calibration coefficients from chip NVM
    info!("Reading BME688 calibration data...");
    let mut calib = bme688_read_calib(&mut i2c, sensor_addr)
        .expect("Failed to read BME688 calibration data");

    // Configure gas heater (use 25°C as initial ambient estimate)
    info!("Configuring gas heater...");
    bme688_setup_gas_heater(&mut i2c, sensor_addr, &calib, 25)
        .expect("Failed to configure gas heater");

    display.clear(Rgb565::BLACK).expect("clear");
    draw_text(&mut display, 10, 10, "BME688 Ready!", Rgb565::GREEN).unwrap();
    draw_text(&mut display, 10, 30, "Gas warming up", Rgb565::YELLOW).unwrap();
    
    log_memory_stats("before main loop");
    
    thread::sleep(Duration::from_millis(1000));
    
    // Main sensor reading loop
    info!("Starting sensor loop");
    
    let mut loop_count: u32 = 0;
    let mut last_amb_temp: i32 = 25;  // Track ambient temp for heater recalibration
    info!("Power saving: backlight {}%, cycle {}ms", BACKLIGHT_PCT, CYCLE_INTERVAL_MS);
    
    loop {
        // Print stack info every 10 iterations
        if loop_count % 10 == 0 {
            log_memory_stats("sensor loop");
        }
        loop_count += 1;
        
        // Read raw sensor data (now includes gas)
        let result = bme688_read_raw(&mut i2c, sensor_addr, calib.variant_id);
        
        if let Ok((p, t, h, gas_adc, gas_range, gas_valid, heat_stab)) = result {
            // Compensate using Bosch algorithms with factory calibration
            let temp_100 = compensate_temperature(&mut calib, t);  // °C * 100
            let press_100 = compensate_pressure(&calib, p);        // Pa * 100
            let hum_100 = compensate_humidity(&calib, h);          // %RH * 100

            // Convert pressure from Pa to hPa (mbar)
            let press_hpa_100 = press_100 / 100;  // hPa * 100

            // Gas reading is valid only when measurement completed AND heater was stable
            let gas_ok = gas_valid && heat_stab;

            // Calculate gas resistance
            let gas_resistance = if gas_ok && gas_adc > 0 {
                calc_gas_resistance(gas_adc, gas_range, calib.variant_id, calib.range_sw_err)
            } else {
                0.0
            };

            // Calculate IAQ
            let humidity_pct = hum_100 as f32 / 100.0;
            let iaq = if gas_ok && gas_resistance > 0.0 {
                estimate_iaq(gas_resistance, humidity_pct)
            } else {
                999  // Invalid / warming up
            };

            // Recalibrate heater with actual ambient temp every 30 cycles
            let current_temp = temp_100 / 100;
            if loop_count % 30 == 1 && (current_temp - last_amb_temp).abs() > 2 {
                last_amb_temp = current_temp;
                let _ = bme688_setup_gas_heater(&mut i2c, sensor_addr, &calib, current_temp);
            }

            let gas_kohm = gas_resistance / 1000.0;
            info!(
                "T={}.{:02}C  P={}.{:02}hPa  H={}.{:02}%  Gas={:.1}kOhm  IAQ={} ({}){} gv={} hs={}",
                temp_100 / 100, (temp_100 % 100).abs(),
                press_hpa_100 / 100, press_hpa_100 % 100,
                hum_100 / 100, hum_100 % 100,
                gas_kohm, iaq, iaq_label(iaq),
                if !gas_ok {
                    if !gas_valid { " [gas invalid]" }
                    else { " [heater not stable]" }
                } else { "" },
                gas_valid as u8, heat_stab as u8,
            );

            // Clear and update display
            display.clear(Rgb565::BLACK).ok();
            
            // Format with one decimal place using heapless buffer
            use core::fmt::Write;
            let mut line_buf = heapless::String::<20>::new();
            
            // Temperature: e.g. "T: 21.7C"
            line_buf.clear();
            let t_whole = temp_100 / 100;
            let t_frac = ((temp_100 % 100).abs()) / 10;
            let _ = write!(line_buf, "{}.{}C", t_whole, t_frac);
            draw_text(&mut display, 5, 5, "T:", Rgb565::GREEN).ok();
            draw_text(&mut display, 25, 5, line_buf.as_str(), Rgb565::GREEN).ok();
            
            // Pressure: e.g. "P: 1001.0"
            line_buf.clear();
            let p_whole = press_hpa_100 / 100;
            let p_frac = (press_hpa_100 % 100) / 10;
            let _ = write!(line_buf, "{}.{}", p_whole, p_frac);
            draw_text(&mut display, 5, 22, "P:", Rgb565::CYAN).ok();
            draw_text(&mut display, 25, 22, line_buf.as_str(), Rgb565::CYAN).ok();
            
            // Humidity: e.g. "H: 48.0%"
            line_buf.clear();
            let h_whole = hum_100 / 100;
            let h_frac = (hum_100 % 100) / 10;
            let _ = write!(line_buf, "{}.{}%", h_whole, h_frac);
            draw_text(&mut display, 5, 39, "H:", Rgb565::YELLOW).ok();
            draw_text(&mut display, 25, 39, line_buf.as_str(), Rgb565::YELLOW).ok();

            // Gas resistance in kOhm
            line_buf.clear();
            if gas_ok && gas_resistance > 0.0 {
                if gas_kohm >= 1000.0 {
                    let _ = write!(line_buf, "{:.0}kOhm", gas_kohm);
                } else if gas_kohm >= 100.0 {
                    let _ = write!(line_buf, "{:.0}kOhm", gas_kohm);
                } else {
                    let _ = write!(line_buf, "{:.1}kOhm", gas_kohm);
                }
            } else {
                let _ = write!(line_buf, "---");
            }
            draw_text(&mut display, 5, 56, "G:", Rgb565::new(16, 32, 16)).ok();
            draw_text(&mut display, 25, 56, line_buf.as_str(), Rgb565::new(16, 32, 16)).ok();

            // IAQ score and label
            line_buf.clear();
            if iaq < 999 {
                let _ = write!(line_buf, "{}", iaq);
            } else {
                let _ = write!(line_buf, "---");
            }
            let iaq_c = if iaq < 999 { iaq_color(iaq) } else { Rgb565::new(16, 16, 16) };
            draw_text(&mut display, 5, 73, "IAQ:", iaq_c).ok();
            draw_text(&mut display, 31, 73, line_buf.as_str(), iaq_c).ok();

            // IAQ label on next line
            let label = if iaq < 999 { iaq_label(iaq) } else { "Warming up" };
            draw_text(&mut display, 5, 90, label, iaq_c).ok();

            // Gas re-arm is now handled inside bme688_read_raw before each forced-mode trigger
        } else {
            error!("Read failed");
            display.clear(Rgb565::BLACK).ok();
            draw_text(&mut display, 10, 30, "Read Error", Rgb565::RED).ok();
        }

        // Wait between measurement cycles — gives heater time to stabilise
        // and prevents display flicker from constant clear+redraw
        thread::sleep(Duration::from_millis(CYCLE_INTERVAL_MS));
    }
}
