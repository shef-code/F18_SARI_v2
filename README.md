# F18_SARI_v2

F/A-18C Standby Attitude Reference Indicator (SARI) for DCS World, driven by DCS-BIOS and drawn on a Waveshare 2.8" round LCD.

The globe, bank pointer, slip ball, rate of turn, pitch-adjust aircraft symbol, OFF flag and ILS needles all work. The physical controls (cage knob, test button) are not implemented yet.

<p>
<img width="360" alt="Simulated display: climbing turn with the ILS needles in view" src="drawings/sari-simulated-ils.png" />
<img width="360" alt="Simulated display: level with the OFF flag in view" src="drawings/sari-simulated-off.png" />
</p>

*Simulated screenshots, composed from the image arrays in this repo with the same layout as the sketch. Left: climbing turn with the ILS needles in view. Right: level, OFF flag in view. They are shown the way you see the mounted display.*

<img width="771" height="1028" alt="The gauge running on the Waveshare display" src="https://github.com/user-attachments/assets/8b370739-d4d0-4343-816a-e55da60d9a37" />

## Hardware

Waveshare **ESP32-S3-LCD-2.8C**, the 2.8" round LCD **without** touchscreen.

| | |
|---|---|
| MCU | ESP32-S3R8, dual core, 240 MHz |
| Memory | 16 MB flash, 8 MB octal PSRAM |
| Display | 2.8" round, 480x480, ST7701 driver, 16-bit RGB parallel interface |
| USB | USB-C through a CH343P USB-to-UART chip (UART0, GPIO43/44) |
| IO expander | TCA9554PWR on I2C (GPIO7 SCL, GPIO15 SDA): LCD reset, LCD CS, buzzer |
| Onboard, not used here | QMI8658 IMU, PCF85063 RTC, TF card slot, buzzer |

Waveshare information for this display:

- [Product page](https://www.waveshare.com/esp32-s3-touch-lcd-2.8c.htm?sku=30254)
- [Wiki: ESP32-S3-LCD-2.8C](https://www.waveshare.com/wiki/ESP32-S3-LCD-2.8C) (pinout, dimensions, Arduino and ESP-IDF setup)
- [Schematic](https://files.waveshare.com/wiki/ESP32-S3-Touch-LCD-2.8C/ESP32-S3-Touch-LCD-2.8C_schematic_diagram.pdf) (filed under the touch model's name)
- [Demo code](https://files.waveshare.com/wiki/ESP32-S3-LCD-2.8C/ESP32-S3-LCD-2.8C-Demo.zip) (`Display_ST7701`, `TCA9554PWR` and `I2C_Driver` in this repo started from it)
- [ST7701S datasheet](https://files.waveshare.com/wiki/common/ST7701S_SPEC_V1.4.pdf)

### The display is mounted upside down

The USB connectors would interfere with the test button and the Cage/Uncage knob, so the display is mounted upside down. I believe this lets the display and electronics clear the Open Hornet frame, but I cannot confirm it, because the frame is too large for my 3D printer's bed.

The flip is baked in: every bitmap is stored rotated 180 degrees and the screen coordinates in the sketch are mirrored. LVGL's rotation setting is not used. If you mount the display the right way up, the artwork and the offsets both have to change.

## Requirements

Versions used in the current build:

| Component | Version | Where to get it |
|---|---|---|
| ESP32 by Espressif (board package) | 3.3.10 | Boards Manager, with `https://espressif.github.io/arduino-esp32/package_esp32_index.json` as an additional boards manager URL |
| lvgl | 8.4.0 | Arduino IDE → Library Manager → search LVGL → install |
| DCS-BIOS FP-Fork | 0.3.9 | [DCS-Skunkworks/dcs-bios-arduino-library](https://github.com/DCS-Skunkworks/dcs-bios-arduino-library) |
| Wire | 3.3.10 | Comes with the ESP32 board package |

The sketch folder has its own `lv_conf.h`, which is the one the build uses.

The Waveshare wiki tells you to install their offline `esp32-XIP` board package and LVGL 8.3.10 for their demos. This sketch does not need those; it builds with the stock Espressif package and the settings below.

## Board settings

Board: **ESP32S3 Dev Module**. Waveshare only shows its settings in screenshots for its own board package, so these were worked out from the hardware and confirmed by flashing.

| Setting | Value | Why |
|---|---|---|
| Flash Size | 16MB (128Mb) | The board has 16 MB |
| PSRAM | OPI PSRAM | The frame buffers and the globe texture live in the 8 MB octal PSRAM. The display does not work without it |
| Partition Scheme | 16M Flash (3MB APP/9.9MB FATFS) | The sketch is about 1.5 MB, too big for the default 1.2 MB app partition |
| USB CDC On Boot | Disabled | USB-C goes through the CH343 to UART0, so `Serial` has to be UART0 for DCS-BIOS |
| Flash Mode | QIO 80MHz | |
| CPU Frequency | 240MHz (WiFi) | |

The same settings as an `arduino-cli` FQBN:

```
esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=default,FlashMode=qio,CPUFreq=240
```

## Connecting to DCS

The board talks to DCS-BIOS over its USB-C port at 250000 baud (the DCS-BIOS default serial mode). Run the DCS-BIOS serial connection script for the board's COM port, the same as for any other DCS-BIOS Arduino panel.

DCS-BIOS controls used, all from the F/A-18C module:

| Identifier | Address | Drives |
|---|---|---|
| `SAI_PITCH` | 0x74e4 | Globe pitch |
| `SAI_BANK` | 0x74e6 | Globe roll and bank pointer |
| `SAI_ATT_WARNING_FLAG` | 0x74e8 | OFF flag |
| `SAI_MAN_PITCH_ADJ` | 0x74ea | Aircraft symbol height |
| `SAI_SLIP_BALL` | 0x74ec | Slip ball |
| `SAI_RATE_OF_TURN` | 0x74ee | Rate of turn indicator |
| `SAI_POINTER_VER` | 0x756a | ILS vertical needle |
| `SAI_POINTER_HOR` | 0x756c | ILS horizontal needle |

## Testing without DCS

Set `SARI_DEMO_MODE` at the top of `F18_SARI_v2.ino` to animate the gauge on the bench with no PC connected:

| Value | Behaviour |
|---|---|
| 0 | Off. Normal operation, reads DCS-BIOS |
| 1 | Pitch sweep and continuous roll |
| 2 | Pitch sweep only |
| 3 | Continuous roll only |

The slip ball, rate of turn, aircraft symbol, OFF flag and ILS needles move in every demo mode. DCS-BIOS is not read while a demo mode is on, so set it back to 0 for use with DCS.

For an on-screen FPS readout, set `SARI_PERF_MONITOR` to 1 in `lv_conf.h`.

## How it draws

The ESP32-S3 has no GPU, so the instrument is 2D bitmap compositing.

- `GlobeRenderer.h` draws the bezel and the attitude globe in one pass. It samples a flat 300x1080 pitch tape (6 px per degree) one scanline at a time, rotated by the bank angle, and mirrors the tape over the pole past 90 degrees of pitch. The aircraft symbol and the ILS needles are drawn in the same pass, over the globe and under the bezel.
- LVGL draws the remaining sprites on top: bank pointer, slip ball, rate of turn and OFF flag.
- The panel has three frame buffers in PSRAM. Each finished frame is handed to the panel whole, so there is no tearing.
- The pixel clock is lowered to 16 MHz (about 57 Hz refresh), which leaves more CPU time for drawing.

Frame rate is currently about 20 FPS with everything moving. DCS-BIOS sends about 30 updates per second, so 30 FPS is the target.

## Files

| File | What it is |
|---|---|
| `F18_SARI_v2.ino` | Gauge logic, LVGL setup, DCS-BIOS callbacks, demo mode |
| `GlobeRenderer.h` | Custom renderer for the bezel, globe, aircraft symbol and ILS needles |
| `Display_ST7701.cpp/.h` | ST7701 panel init and RGB panel driver |
| `TCA9554PWR.cpp/.h`, `I2C_Driver.cpp/.h` | IO expander driver |
| `lv_conf.h` | LVGL configuration |
| `SARIBackground.c`, `SARIGlobe.c` | Bezel and globe pitch tape |
| `Bank.c`, `SARIBug.c`, `SARICaged.c`, `SlipBall.c`, `RateOfTurn.c` | Bank pointer, aircraft symbol, OFF flag, slip ball, rate of turn |
| `VerticalPointer.c`, `HorizontalPointer.c` | ILS needles |

## Related

- [F18-RadarAltimeter](https://github.com/shef-code/F18-RadarAltimeter): my F/A-18C radar altimeter on the Waveshare 1.85" round LCD.
