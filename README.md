# M5Stack AtomS3 + BME688 (Rust)

Rust starter project for reading a BME688 over I2C and showing values on AtomS3's built-in display.

## What it displays

- Temperature (°C)
- Pressure (hPa)
- Humidity (%)
- Gas resistance (Ohms)
- VOC proxy index (`VOCx`) derived from gas resistance trend

> `BME688` does **not** provide direct concentration values for specific gases (CO2, NOx, etc.) without Bosch BSEC. This project shows raw gas resistance plus a practical VOC trend proxy.

## Hardware assumptions

Default pin map in `src/main.rs`:

- **BME688 (external Port / HY2.0):** `SDA=GPIO2`, `SCL=GPIO1`, address `0x76`
- **AtomS3 LCD (GC9107 controller, using ST7789 driver in compatibility mode):**
  - `SCLK=GPIO17`
  - `MOSI=GPIO21`
  - `CS=GPIO15`
  - `DC=GPIO33`
  - `RST=GPIO34`
  - `BL=GPIO16` ← **Backlight (confirmed from M5GFX source)**
  - **SPI Mode:** `MODE_0` (per official M5Stack implementation)

**Note:** The AtomS3 uses a GC9107 display controller, but since `mipidsi` doesn't support GC9107, we use the ST7789 driver in compatibility mode with SPI MODE_0.

If your wiring differs, edit the constants in `src/main.rs`.

## 1) One-time toolchain setup (Windows PowerShell)

```powershell
cargo install espup --locked
espup install
cargo install ldproxy espflash cargo-espflash --locked
```

Open a **new terminal** after `espup install`.

Then verify toolchain environment:

```powershell
rustc -Vv
```

## 2) Build

```powershell
cargo build
```

If you still see `can't find crate for core` on `xtensa-esp32s3-espidf`, your shell has not loaded the ESP toolchain environment yet. Open a brand-new VS Code terminal after running `espup install`.

## 3) Flash + monitor

```powershell
cargo espflash flash --monitor --chip esp32s3
```

If auto-reset does not enter bootloader:

- Hold the AtomS3 reset button for ~2 seconds until green LED indicates download mode.
- Run flash command again.

## VS Code

This repo includes tasks for:

- Build firmware
- Flash firmware
- Flash + monitor

Use **Terminal → Run Task**.

## Notes

- Refresh interval is ~1 second.
- Gas/VOC stabilization takes a few minutes after power-up.
- This project sets `CARGO_TARGET_DIR=C:/t/m5bme` in `.cargo/config.toml` to avoid Windows `esp-idf-sys` long-path build failures.
