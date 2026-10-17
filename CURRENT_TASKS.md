# F18 SARI v2 — Current Tasks

Handoff file for whoever works on this next (any AI model, or Jason). It is the single source of truth for what is known, what is planned, and what is done. Read sections 1–3 before touching code.

Last full code review: 2026-10-07, branch `SARI-8bit-optimization`, commit `c0982b8`.

---

## 1. How to use this file

1. Read sections 2 and 3 (context and verified facts). Do not re-derive them.
2. Pick the first task in section 5 whose status is `[ ]`, top to bottom, unless Jason says otherwise.
3. Do one task at a time. Keep each task to its own commit.
4. When you finish or stop, update the task's status and add one line to the session log (section 8).
5. Compile every change before handing it over (section 2, "Building"). An AI agent can compile freely, but uploads only when Jason asks for one in that session. The SARI board is on COM11 (reported by Jason 2026-10-07), but about 15 serial devices are attached to this PC and Windows can renumber ports, so before any upload run `board list` and confirm with Jason that COM11 is still the SARI board. Jason reports what he sees on the hardware. A task is only `[x]` after Jason confirms it on hardware. Use `[~]` for "code written and compiling, waiting on hardware test". There is no Python on this PC.
6. If a task's premise turns out to be wrong, say so in the task and the log. Do not quietly work around it.

Status marks: `[ ]` not started, `[~]` code written and awaiting hardware test, `[x]` confirmed on hardware, `[-]` dropped (give the reason).

---

## 2. Project context

- **What it is:** a replica of the F/A-18C Standby Attitude Reference Indicator (SARI) as seen in DCS World, driven by DCS-BIOS.
- **Goal:** look as close to the DCS instrument as possible at a smooth frame rate. The ESP32 has no GPU, so everything is 2D bitmap compositing.
- **Hardware:** Waveshare ESP32-S3-LCD-2.8C: ESP32-S3, 480x480 round ST7701 panel on a 16-bit RGB parallel interface, frame buffer in PSRAM, TCA9554 I2C IO expander, no touch.
- **The display is mounted upside down** (the USB connectors would otherwise hit the knob). This is handled by storing every bitmap rotated 180 degrees and mirroring coordinates in code. It is not handled by LVGL rotation (see fact V4).
- **Software:** Arduino-ESP32 core 3.3.10, LVGL 8.4.0, DCS-BIOS Arduino library 0.3.10, compiler flag `-Os`.
- **Files:**
  - `F18_SARI_v2.ino`: all gauge logic, the LVGL setup, the DCS-BIOS callbacks.
  - `Display_ST7701.cpp/.h`: panel init and the RGB panel driver (Waveshare demo code).
  - `TCA9554PWR.cpp/.h`, `I2C_Driver.cpp/.h`: IO expander (Waveshare demo code).
  - `lv_conf.h`: the LVGL config that is really in use. The ESP32 core adds the sketch folder to the include path, and there is no other `lv_conf.h` beside the library.
  - `*.c`: LVGL image arrays (table below).

### Building

The Arduino IDE's bundled `arduino-cli` (1.2.0) is used from VS Code. Board settings live in `sketch.yaml` and the VS Code tasks in `.vscode/tasks.json` (Terminal > Run Task, or Ctrl+Shift+B to compile).

Command line, from the sketch folder (PowerShell):

```
& "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" --config-file "$env:USERPROFILE\.arduinoIDE\arduino-cli.yaml" compile .
```

Add `--upload` to flash; the port (COM11) comes from `sketch.yaml`, or override it with `--port COMx`. The `--config-file` argument is required; it points the CLI at the same core and libraries the IDE uses.

Board settings (board "ESP32S3 Dev Module"). Waveshare's wiki does not list them; they are inferred from the hardware (ESP32-S3R8, 16 MB flash, USB-C wired to a CH343 USB-UART chip) and confirmed to compile on 2026-10-07. Not yet confirmed by flashing.

| Setting | Value | Why |
|---|---|---|
| Flash Size | 16MB | Board has 16 MB |
| PSRAM | OPI PSRAM | S3R8 has 8 MB octal PSRAM; the frame buffer needs it |
| Partition Scheme | 16M Flash (3MB APP/9.9MB FATFS) | Sketch is 1.45 MB, too big for the 1.2 MB default |
| USB CDC On Boot | Disabled | USB-C goes through the CH343 to UART0, so `Serial` must be UART0 for DCS-BIOS |
| Flash Mode | QIO 80MHz | |
| CPU Frequency | 240MHz | |

Baseline size at commit `c0982b8`: sketch 1,453,160 bytes (46% of 3 MB), globals 258,924 bytes (79% of internal RAM, 68 KB free). Internal RAM is tight, so watch that figure.

### Board pins (from the Waveshare wiki)

- Used by the panel: GPIO 1, 2, 3, 5, 6, 8-14, 17, 18, 21, 38-41, 45-48.
- I2C bus: GPIO7 (SCL), GPIO15 (SDA). Shared by the TCA9554 expander, the IMU and the RTC, and brought out on a header.
- UART0: GPIO43 (TX), GPIO44 (RX). This is the DCS-BIOS port.
- TCA9554 expander: all eight pins are used on the board and none are brought out (EXIO1 LCD reset, EXIO2 touch reset, EXIO3 LCD CS, EXIO4 SD CS, EXIO5/6 IMU interrupts, EXIO7 RTC interrupt, EXIO8 buzzer).
- **Free and reachable on the 12-pin header: GPIO0, GPIO19, GPIO20.** GPIO0 is the boot strapping pin and must not be held low at power-up. GPIO19/20 are the native USB data lines, free because USB goes through the CH343.
- Not on any header: GPIO4 (battery ADC), GPIO16 (touch interrupt, unused on the non-touch board), GPIO42 (SD card).
- Onboard and unused by this project: QMI8658 IMU, PCF85063 RTC, SD slot, buzzer.

### Images

| File | Format | Size (px) | Used | Notes |
|---|---|---|---|---|
| `SARIBackground.c` | INDEXED_8BIT, alpha | 480x480 | yes | Bezel. Transparent round window, x 104..378, y 133..407 (275x275, centre 241,270) |
| `SARIGlobe.c` | TRUE_COLOR (RGB565) | 300x1080 | yes | Flat pitch tape, 6 px per degree, covers -90..+90 only. 648 KB |
| `Bank.c` | TRUE_COLOR_ALPHA | 22x312 | yes | Only a 20x22 triangle at the top (x 1..20, y 3..24) is opaque. The rest is empty |
| `SARIBug.c` | INDEXED_8BIT | 179x40 | yes | Aircraft symbol |
| `SARICaged.c` | INDEXED_8BIT | 49x144 | yes | OFF flag. Rotated in code (see B1) |
| `SlipBall.c` | INDEXED_8BIT | 26x27 | yes | |
| `RateOfTurn.c` | INDEXED_8BIT | 30x29 | yes | |
| `HorizontalPointer.c` | ALPHA_8BIT | 235x8 | **no** | ILS needle, never referenced |
| `VerticalPointer.c` | INDEXED_8BIT | 8x235 | **no** | ILS needle, never referenced |

### DCS-BIOS controls for this instrument

Taken from Jason's installed `FA-18C_hornet.json`. All gauge outputs are 0..65535, mask `0xffff`, shift 0.

| Identifier | Address | In code | Purpose |
|---|---|---|---|
| `SAI_PITCH` | 0x74e4 | yes | Pitch |
| `SAI_BANK` | 0x74e6 | yes | Bank |
| `SAI_ATT_WARNING_FLAG` | 0x74e8 | yes | OFF flag position |
| `SAI_MAN_PITCH_ADJ` | 0x74ea | yes | Aircraft symbol height |
| `SAI_SLIP_BALL` | 0x74ec | yes | Slip ball |
| `SAI_RATE_OF_TURN` | 0x74ee | yes | Rate of turn |
| `SAI_POINTER_VER` | 0x756a | **no** | ILS vertical needle |
| `SAI_POINTER_HOR` | 0x756c | **no** | ILS horizontal needle |
| `SAI_CAGE` | input, 0x747e mask 0x0400 | **no** | Pull to cage (selector) |
| `SAI_SET` | input, 0x74e2 | **no** | Adjust knob (variable_step) |
| `SAI_TEST_BTN` | input, 0x747e mask 0x0200 | **no** | Test button |
| `INSTR_INT_LT` | 0x7560 | **no** | Instrument lighting level |
| `INST_PNL_DIMMER` | 0x7546 | **no** | INST PNL dimmer knob position |

---

## 3. Verified facts (checked against the installed library source)

**V1. DCS-BIOS sends about 30 updates per second.** Rendering faster than 30 fps shows the same data twice unless values are interpolated (task A2). The realistic first target is a steady 30 fps with no dropped updates.

**V2. LVGL is capped at about 33 fps by config.** `LV_DISP_DEF_REFR_PERIOD` is 30 ms in `lv_conf.h`. A 30 ms timer running against a 33 ms data stream also causes periodic judder.

**V3. LVGL 8.4 cannot rotate indexed images.** Indexed images are decoded one row at a time (`lvgl/src/draw/lv_draw_img.c`, the "line-by-line" branch near line 301), and that branch does not support rotation or zoom. Only TRUE_COLOR, TRUE_COLOR_ALPHA, RGB565A8 and ALPHA_8BIT C arrays take the whole-image path that can be transformed. Indexed images also allocate and rebuild a 256-entry palette every time they are drawn, because `LV_IMG_CACHE_DEF_SIZE` is 0.

**V4. `disp_drv.rotated = LV_DISP_ROT_180` does nothing.** LVGL only rotates when `sw_rotate = 1` (`lvgl/src/core/lv_refr.c` line 1293). The 180 degree flip comes entirely from the pre-rotated bitmaps and mirrored coordinates. Do not set `sw_rotate`: it would flip everything a second time and cost CPU.

**V5. What LVGL redraws each frame.** Any pitch or bank change invalidates the 275x275 globe holder. For that area LVGL then (a) rotates the globe with antialiasing, (b) alpha-blends the background over it row by row through the indexed decoder, even though most of that area is the fully transparent window, and (c) rotates the 22x312 bank image, whose rotated bounding box is up to about 312x312 and is almost all transparent. That is roughly three per-pixel passes over about 75,000 pixels to show one textured disc and a small triangle.

**V6. The panel refreshes at about 108 Hz.** PCLK is 30 MHz and the total frame is 548x508 clocks. With the bounce buffer enabled, an interrupt copies the whole 460 KB frame out of PSRAM on every refresh, about 50 MB/s, on the same core that runs LVGL (the core that called `esp_lcd_new_rgb_panel`, which is the Arduino loop core).

**V7. DCS-BIOS serial is polled, not interrupt driven.** `DcsBios::loop()` drains `Serial` only between frames. At 250000 baud about 25 bytes arrive per millisecond. The Arduino-ESP32 default receive buffer is 256 bytes, so any frame that takes longer than about 10 ms to render can overflow it. The parser then loses sync and drops updates until the next sync frame.

**V8. `DcsBios::setup()` calls `Serial.begin(250000)` itself.** The sketch's own `Serial.begin(115200)` and `delay(1000)` are redundant.

**V9. The previous commit (`e4b33e8`) had every image as TRUE_COLOR_ALPHA.** Commit `c0982b8` converted most of them to INDEXED_8BIT. That saved flash but moved them onto the slower row-by-row draw path (V3) and likely broke the OFF flag rotation (B1).

---

## 4. Open questions for Jason

Answer these inline when you can. Tasks that depend on one say so.

- Q1. ANSWERED 2026-10-07: Jason did not record the old settings. The inferred settings in section 2 "Building" were flashed and the gauge displays normally.
- Q2. ANSWERED 2026-10-08: it never swung as an indexed image (see B1); fixed. Original question: Does the OFF flag still swing in and out correctly on the current commit, or has it looked wrong since the 8-bit conversion? (B1)
- Q3. ANSWERED 2026-10-07: Jason is not sure; `32782` may be a typo. Settle it by measurement (B5).
- Q4. PARTLY ANSWERED 2026-10-07: only GPIO0, GPIO19 and GPIO20 are free (section 2 "Board pins"). Still open: what hardware Jason wants for the knob and button (F3).
- Q5. Should the backlight follow the cockpit instrument lighting, and what is the minimum brightness you want in a dark cockpit? (F2)
- Q6. Which DCS-BIOS library do you intend to build with? Both `DCS-BIOS` 0.3.10 and `DCS-BIOS_FP-Fork` 0.3.9 are installed and both provide `DcsBios.h`.
- Q7. OK to delete `README.zip` from the repo? (C1)
- Q8. What do the real F/A-18C SAI cage knob and TEST button do? Until Jason says, locally: cage pulled puts the OFF flag in view, test held centres the ILS needles. (F3)

---

## 5. Tasks

Order matters. Phase 0 gives numbers, so that every later change can be judged by measurement instead of by feel.

### Phase 0 — Measure first

**[x] M1. Bench demo mode.** Confirmed on hardware (Jason, 2026-10-08): demo mode runs and all SARI elements animate. Code written 2026-10-07, compiles in all modes.
`#define SARI_DEMO_MODE` at the top of `F18_SARI_v2.ino`: 0 = off (normal, reads DCS-BIOS), 1 = pitch sweep + roll, 2 = pitch only, 3 = roll only. Slip ball, rate of turn, pitch-adjust bug and OFF flag animate in every demo mode. Data is fed at ~30 Hz, the same rate as DCS-BIOS. Leave it at 0 when committing.
Done when: with the flag at 1 the gauge animates on the bench with no PC connected, and with 0 behaviour is unchanged.

**[~] M2. Frame-rate readout.** Code written 2026-10-07, compiles, not yet flashed.
`#define SARI_PERF_MONITOR` in `lv_conf.h` (0 off, 1 on). It must be set in `lv_conf.h`, not the `.ino`, because the LVGL library is compiled separately. The readout sits at the bottom centre of the screen (upside down on the physical display, so top centre as you look at it); the round panel hides the corners. Leave it at 0 when committing.
Done when: the baseline table below is filled in.

`LV_TICK_CUSTOM` is now 1 (LVGL reads `millis()` itself) and the manual `lv_tick_inc` is gone from `loop()`. Before this fix the readout always showed 33 FPS and 0% CPU, because LVGL measures render time with its tick counter and the sketch only advanced the tick between frames, so every draw measured as zero time. Do not trust any FPS number taken before 2026-10-07 evening.

**Baseline measurements (Jason fills these in).** Set `SARI_DEMO_MODE` and `SARI_PERF_MONITOR`, upload, wait 20 s, read FPS and CPU %. Commit `c0982b8` plus M1/M2 only.

| Demo mode | FPS | CPU % | Notes |
|---|---|---|---|
| 1 (pitch + roll), after P0 + P1 + P5 (16 MHz) + P6 (panel on core 0) + images in PSRAM | 10-12 | 97-98 | Jason, 2026-10-08, video `IMG_9853.MOV`. Pitch-only measured over serial at about 18-24 FPS with the same build. Tearing still visible in some frames |
| 2 (pitch only), after P0 | 7 | 96-97 | Jason, 2026-10-08, video `IMG_9848.MOV`. Only a small gain; tearing still visible in some frames |
| 2 (pitch only) | 5-6 | 95+ | Jason, 2026-10-07, after the LV_TICK_CUSTOM fix. Far below the 30 fps target |
| 3 (roll only) | "33" most of the time, 3-5 otherwise. Real rate about 3-5 | 99 (98 during the dips) | Jason, 2026-10-08. The 33 is not real, see note below |
| 1 (pitch + roll) | "33" most of the time, 3-5 otherwise. Real rate about 3-5 | 98-99 | Jason, 2026-10-08, video `IMG_9847.MOV`. Tearing and ghosting visible |

**Note on the "33 FPS" readout.** In `lvgl/src/core/lv_refr.c` (lines 397-410) the monitor shows the cap (`fps_limit`, 33) whenever it counted no frame over 5000 px in its 300 ms window. So 33 means frames are slower than 300 ms, not that the rate is 33. Treat only the low numbers as real. All three modes are therefore about 3-6 FPS, and the target is 30. Video `IMG_9847.MOV` (mode 1) shows steps of about 3-5 per second plus tearing (two attitudes overlaid in one frame).

### Phase 1 — Bugs

**[x] B1. OFF flag may not rotate correctly as an indexed image.** (V3, V9, Q2) Fixed and confirmed by Jason in demo mode 2026-10-08. Premise confirmed in the LVGL source (`lv_draw_img.c`, line-by-line branch): each row of an indexed image is rotated on its own and clipped to the unrotated box, so the flag only drew properly at exactly 0 degrees. `SARICaged.c` was converted in place from the indexed data to TRUE_COLOR_ALPHA (same pixels; not taken from `e4b33e8`, which predates the 180 degree bitmap rotation check).
Update 2026-10-07: in Jason's video of the old build the flag does appear to swing in at the right edge, so the premise below may be wrong. Check it closely on a frame-by-frame basis before changing anything. Original note:
`SARICaged.c` is now INDEXED_8BIT but is rotated with `lv_img_set_angle`.
Fix: restore the TRUE_COLOR_ALPHA version with `git checkout e4b33e8 -- SARICaged.c` (same 49x144 size), or re-export it as TRUE_COLOR_ALPHA or RGB565A8.
Done when: the flag swings smoothly through its full travel in demo mode.

**[x] B2. OFF flag animation starts from the wrong angle.** Confirmed by Jason in demo mode 2026-10-08: the `lv_anim` block is gone and the angle is set directly. `F18_SARI_v2.ino` lines 142-162.
`current_caged_angle` is set to the new target as soon as an animation starts, so the next animation starts from the previous target and not from where the flag is actually drawn. DCS already sends this value as a smooth sweep, so each update restarts a 280 ms ease and the flag lags and jumps. Also, setup sets the angle to `900` while the callback works in `-900..0`.
Fix: remove the `lv_anim` block and call `lv_img_set_angle(caged_img, target)` directly. This also removes the function-pointer cast on line 155.
Done when: the flag tracks DCS without lag or jumps.

**[x] B3. Serial receive buffer is too small.** (V7, V8) Confirmed by Jason 2026-10-08 in DCS: no issues. `Serial.begin(115200)` is kept only in demo mode, for the `SARI_PROFILE` output.
In `setup()`, delete `Serial.begin(115200)` and `delay(1000)`, and call `Serial.setRxBufferSize(4096)` immediately before `DcsBios::setup()`. It must run before `begin`.
Done when: no visible stalls or snaps in the gauge during fast manoeuvres in DCS.

**[ ] B4. Driver `printf` calls share the DCS-BIOS port.**
`Display_ST7701.cpp`, `TCA9554PWR.cpp` and `I2C_Driver.cpp` print error text with `printf`, which goes out on the same serial port as DCS-BIOS messages. Remove them or wrap them in a debug define.

**[ ] B5. Centre-value constants are inconsistent.** (Q3) Lines 44-45, 103-140.
`DCS_MID` is 32782 and `DCS_HALF` is 32768. The pitch-adjust callback subtracts one and divides by the other. Jason does not know whether 32782 was deliberate. Settle it by reading the real values: with the aircraft level, wings level and in coordinated flight, note what DCS-BIOS reports for `SAI_PITCH`, `SAI_BANK`, `SAI_SLIP_BALL` and `SAI_RATE_OF_TURN` (the DCS-BIOS control reference page shows live values). If they sit at 32767/32768, use 32768 everywhere. The difference is 14 counts, under 0.1 degree, so this is not visible either way. Then use one constant. Remove the `(DCS_MID != 0) ?` and `(DCS_HALF != 0) ?` checks, which are always true.

**[~] B6. Globe is not aligned with the bezel window.** Fixed 2026-10-09 after Jason photographed the sliver, waiting on his check. Done differently from the note below, to leave the picture where it is: the renderer's window rectangle is the old holder plus a 2 px margin on every side (`WIN_MARGIN` in `GlobeRenderer.h`), which covers the whole hole and keeps the same centre, and the screen background is black. The globe is therefore still centred on (239, 272) while the hole is centred on (241, 270); moving it would be a separate, visible change. Original note:
The holder is placed at x 102..376, y 135..409. The window in the bezel is x 104..378, y 133..407. That leaves a thin sliver, about 2 px, at one edge of the window where the screen background shows through, and the default LVGL screen background is near-white.
Fix: position the holder with `lv_obj_set_pos(holder, 104, 133)`, drop the `lv_obj_center` / `lv_obj_set_y` pair, and set the screen background to black (`lv_obj_set_style_bg_color(scr, lv_color_black(), 0)`).
Done when: no light sliver at the window edge at any attitude.

**[ ] B7. Steep pitch shows a blank area.**
The globe texture covers only -90..+90 degrees (540 px each side of centre) and the window radius is 137 px, so beyond about 67 degrees of pitch the window runs off the end of the texture.
Short-term fix: the black screen background from B6. Proper fix: wrap over the pole in task A1, or extend the texture.
Done when: pitching straight up or down never shows an empty region.

**[ ] B8. Remove the no-op rotation line.** (V4)
Delete `disp_drv.rotated = LV_DISP_ROT_180;` (line 195) and leave a one-line comment saying the flip is baked into the bitmaps.

### Phase 2 — Cheap performance wins, keeping LVGL

Measure after each one (M2) and log the result. Keep changes that help and revert ones that do not.

**[-] P0. Remove the permanent 180 degree rotation of the globe.** DROPPED 2026-10-08: worked (picture correct) but pitch-only only went from 5-6 to 7 FPS, and Jason saw more glitching with it than on the baseline. Reverted. Mechanism for the extra glitching not understood.
Found 2026-10-07. DCS reports level flight as bank 32768, which `onSaiBankChange` turns into 1800 (180 degrees), and that angle is applied to the globe. The globe texture is stored the other way up, so at level flight LVGL is always running the slow rotate-and-resample path over the whole window, even when the aircraft is not rolling. That explains why pitch-only runs at only 5-6 FPS.
Fix: re-export `SARIGlobe.c` rotated 180 degrees (this is the same kind of re-export as the earlier "rotated all bitmaps" commit), and set the globe angle to `bank_x10 - 1800` wrapped to 0..3599. At level flight the angle is then 0 and LVGL takes its plain-copy path. Update `update_globe()` so the pitch direction still comes out right. Leave `bank_img` as it is until this is checked.
Done when: pitch-only FPS rises sharply, and the globe looks identical to before at several pitch and bank values.

**[x] P1. Crop the bank pointer to its triangle.** Confirmed by Jason 2026-10-08. (V5c) Expected: large.
Re-export `Bank.c` as a 22x28 image holding only the triangle (rows 0..27 of the current image). Keep the object's top-left where it is now (`lv_obj_set_pos(bank_img, 229, 117)`, replacing the `lv_obj_align` call, which would re-centre the shorter image) and set the pivot to the same screen point as before: `lv_img_set_pivot(bank_img, 11, 156)`. LVGL allows a pivot outside the image. The redraw then covers about 35x35 px, down from about 312x312.
Done when: the pointer follows the same arc as before and FPS in the roll test improves.

**[ ] P2. Take the bezel off the per-frame alpha-blend path.** (V5b) Expected: large.
Option A (try first): re-export `SARIBackground.c` as `LV_IMG_CF_RGB565A8`, which has a dedicated fast path in LVGL 8.4 (`lv_draw_sw_img.c` line 85). Costs about 691 KB of flash.
Option B: make the bezel opaque TRUE_COLOR (plain copy, the fastest path), put the globe holder above it in z-order, and clip the holder to a circle with `lv_obj_set_style_radius(holder, LV_RADIUS_CIRCLE, 0)` and `lv_obj_set_style_clip_corner(holder, true, 0)`. This removes the blend pass, but the soft inner edge of the bezel is replaced by LVGL's own antialiased circle.
Done when: FPS improves in the pitch-only test with no visible change at the window edge.

**[ ] P3. Lower the LVGL refresh period.** (V2)
Set `LV_DISP_DEF_REFR_PERIOD` to 10 in `lv_conf.h`.

**[ ] P4. Turn off antialiasing on the globe only.**
`lv_img_set_antialias(globe_img, false);` after creating it. The rotation then samples one source pixel per output pixel. Check whether the pitch lines look acceptably clean while rolling. Leave antialiasing on for the small sprites.

**[x] P5. Lower the pixel clock.** (V6) Confirmed by Jason 2026-10-08: 16 MHz (about 57 Hz), no flicker, no drift, roughly doubles the frame rate. Must be with the bounce buffer on and the panel on the original core.
In `Display_ST7701.h` set `ESP_PANEL_LCD_RGB_TIMING_FREQ_HZ` to 18 MHz (about 65 Hz) and then try 16 MHz (about 57 Hz). This roughly halves the interrupt copy load.
Done when: FPS improves and the picture shows no flicker, drift or shimmer. Revert if it does.

**[-] P6. Move the panel interrupt to the other core.** (V6) DROPPED 2026-10-08: it raised the frame rate a lot, but the whole picture then drifted sideways and overlapped itself (video `IMG_9855.MOV`), unreadable at 30 MHz and visibly shifting at 16 MHz. With the interrupt on its own core, LVGL's flash and PSRAM reads compete with it for the memory bus and the panel is starved. On the same core the interrupt always pre-empts LVGL. Do not retry without a different way of feeding the panel. Original text:
Call `LCD_Init()` from a short one-shot FreeRTOS task pinned to core 0 and wait for it to finish before continuing `setup()`. The refresh interrupt then runs on core 0 and LVGL keeps core 1.

**[ ] P7. Config and compiler tuning.** Update 2026-10-08: the `-O2` `build_opt.h` part was tried and gave no measurable change (39-52 frames per 2 s either way), so skip it. The `lv_conf.h` settings are untested.
In `lv_conf.h`: `LV_ATTRIBUTE_FAST_MEM` to `IRAM_ATTR`, `LV_MEMCPY_MEMSET_STD` to 1, `LV_MEM_CUSTOM` to 1. Add a `build_opt.h` file to the sketch folder containing `-O2` (the core copies it into the build). Check the verbose compile output to confirm `-O2` appears after `-Os`.

**[~] P8. Restore the small sprites to a directly drawable format.** (V3) Done 2026-10-08, waiting on Jason's look: `SARIBug.c`, `SlipBall.c` and `RateOfTurn.c` converted in place to TRUE_COLOR_ALPHA (same pixels, +14 KB flash). Gain is small: mode 1 went from 36-46 to 37-48 frames per 2 s.
`SARIBug`, `SlipBall` and `RateOfTurn` are indexed, so each redraw rebuilds a palette and decodes row by row. They are tiny (under 22 KB total as TRUE_COLOR_ALPHA). Convert them back, or to RGB565A8, and measure. Low priority.

**[x] P9. Stop screen tearing.** Confirmed by Jason 2026-10-08: two panel frame buffers, flush copies into the hidden one and swaps at frame end; see `my_disp_flush`, `refresh_display` and `LCD_showFrameBuffer`. Original text:
Flushes are not synchronised to the panel's vertical sync, so a fast-rolling horizon can tear. Options: use two frame buffers and swap on vsync (`num_fbs = 2` with LVGL `direct_mode`), or wait on a vsync semaphore before flushing. Only do this if Jason sees tearing once the frame rate is up.

### Phase 3 — Alternative approach, if Phase 2 is not enough

**[~] A1. Custom globe renderer.** Written and confirmed correct on hardware 2026-10-08 (`GlobeRenderer.h`), about 18 FPS in mode 1; the 30 FPS target is not reached yet. Recommended if the Phase 2 result is below a steady 30 fps.
Replace LVGL's rotation of the globe with a purpose-built scanline renderer. LVGL does generic work this instrument does not need.
- Precompute, per screen row, the first and last x of the circular window (from the bezel alpha). Only about 59,000 pixels are ever drawn.
- Per frame, compute sin and cos of bank once in 16.16 fixed point. For each row compute the starting texture coordinate, then step it by a constant per pixel (two adds), fetch the RGB565 texel, and write it. No per-pixel multiply, no alpha, no masks.
- Blend only the roughly 800 antialiased edge pixels of the bezel from a precomputed list.
- Pole wrap, which fixes B7 properly: when the texture row runs past either end, mirror it (`v = 2*edge - v`, `u = width - u`).
- Optional true-ball look: before the rotation step, scale each pixel's offset from the window centre by a precomputed factor `asin(r/R) / (r/R)`. This makes the pitch lines bunch toward the rim the way a sphere does, at the cost of one table lookup and two multiplies per pixel. This is the closest a 2D renderer gets to the DCS 3D ball.
- Keep LVGL for the sprites, or draw them directly too. If LVGL stays, do the globe in a custom draw event on the holder so it writes into LVGL's draw buffer.
Done when: the roll test holds 30 fps or better with antialias-quality lines.

**[ ] A2. Interpolate between DCS-BIOS updates.** (V1)
Once rendering is faster than 30 fps, ease pitch and bank toward their latest targets every rendered frame. Handle the bank wrap at 0/360 by taking the short way round. This is what makes motion look fluid.

**[ ] A3. Alternative library (not recommended as the first move).**
LovyanGFX 1.2.7 is installed and supports this panel type with rotated sprite pushes. It is a full rewrite of the display layer for a gain that A1 gets with less change.

### Phase 4 — Missing features

**[~] F1. ILS needles.** Written 2026-10-08, running in demo mode, waiting on Jason. How it is built (differs from the original note below): with the custom renderer the bezel and globe are one object, so the needles are drawn inside it (`add_needle`, `set_needle_pos` and step 6 of `draw()` in `GlobeRenderer.h`), over the globe and under the bezel; a needle slid past the window edge disappears behind the bezel. They are not shown with the LVGL fallback globe. `ILS_TRAVEL_PX` (150), `ILS_VER_DIR`, `ILS_HOR_DIR` and `ILS_COLOUR` (0x9A682C, taken from the vertical needle image) in the sketch are guesses: the mapping assumed is centred at 32768 and linear to out-of-view at both ends of the range. Still needed from Jason, comparing with DCS: which way each needle moves, how far a full-scale deflection should go, where it parks, the colour, and whether the two bitmaps need the 180 degree flip (they look symmetric). No frame-rate cost measured (37-48 frames per 2 s). Original note:
Add `IntegerBuffer`s for `SAI_POINTER_VER` (0x756a) and `SAI_POINTER_HOR` (0x756c). The bitmaps exist (`VerticalPointer.c`, `HorizontalPointer.c`). Z-order: above the globe, below the bezel. Move them by translation (x for the vertical needle, y for the horizontal one). Check that the two bitmaps were rotated 180 degrees like the rest. `HorizontalPointer` is ALPHA_8BIT, so give it a colour with `lv_obj_set_style_img_recolor` and `lv_obj_set_style_img_recolor_opa`. Jason needs to supply the parked position and full-deflection travel by comparing with DCS.
Done when: both needles match DCS with the ILS tuned, and park out of view otherwise.

**[ ] F2. Backlight follows cockpit lighting.** (Q5)
Add an `IntegerBuffer` for `INSTR_INT_LT` (0x7560) and map it to `Set_Backlight()` with a minimum floor. Today the backlight is fixed at 50.

**[~] F3. Physical controls.** Code written and flashed 2026-10-16 (`Controls.h`, hooked into setup and loop). Jason confirmed the controls work with no DCS connected (adjust knob, cage, test). Not yet tested against DCS: the messages sent to DCS (`SAI_SET`, `SAI_CAGE`, `SAI_TEST_BTN`, `STBY_PRESS_ALT`) and the knob directions in that case (`ENC1_REVERSE`, `ENC2_REVERSE`). Local adjust direction was reversed at Jason's request. What the real cage and test do is still to be worked out (Q8). (Q4)
`SAI_CAGE` (pull switch), `SAI_SET` (rotary encoder, variable_step), `SAI_TEST_BTN` (push button). That is four inputs and only three GPIOs are free (0, 19, 20; see "Board pins"). Two ways to do it:
- Option A, I2C (recommended): put the encoder and switches on the I2C header (GPIO7/15) using an Adafruit seesaw rotary encoder breakout (the seesaw library is already installed) or a PCF8574 expander. No free GPIO is used. The DCS-BIOS `Switch2Pos` and `RotaryEncoder` classes read real pins, so poll the I2C device in `loop()` and call `DcsBios::tryToSendDcsBiosMessage("SAI_SET", "+3200")` and the like directly. Check the new device's address against the ones already on the bus: TCA9554 at 0x20, QMI8658 IMU, PCF85063 RTC.
- Controls board update 2026-10-16: a second rotary encoder was added to the PCF8574 board (`drawings/sari-controls-pcb.svg` and `drawings/sari-controls-protoboard.svg`), on 3-pin connector J4: P6 = A, P5 = B, push switch not used (P4 and P7 are spare). Encoder 1 stays on P0 = A, P1 = B, P2 = switch, P3 = test button. No firmware reads it yet, and it is not decided which DCS-BIOS control it drives.
- Firmware (`Controls.h`): PCF8574 at 0x21 polled about every 1 ms by a task on the other core. Encoder 1 sends `SAI_SET` +3200/-3200, its switch (P2) sends `SAI_CAGE` 1 when pressed and 0 when released, the test button (P3) sends `SAI_TEST_BTN` 1/0, encoder 2 sends `STBY_PRESS_ALT` +3200/-3200. Tunables at the top of the file: steps per click, reverse flags. Disabled in demo mode.
- Night/NVG tint (2026-10-16, compiled, not yet flashed or tested in DCS): `COCKKPIT_LIGHT_MODE_SW` (0x74C8, mask 0x600, shift 9; inferred from Jason's report: 0 DAY, 1 NITE, 2 NVG; NVG is the same switch, another position) NITE and NVG turn the display green in `my_disp_flush`, NVG at `NVG_BRIGHTNESS` (0.7) of NITE, and `INST_PNL_DIMMER` (0x7546) sets the backlight between `NIGHT_MIN_LEVEL` (0.20) and `NIGHT_BACKLIGHT_MAX`; day backlight stays 50. Not exercised by the no-DCS demo.
- No-DCS behaviour (2026-10-16, replaces the earlier auto demo): at power-up the gauge shows a normal level display (OFF flag out of view, ILS needles out of view). While no DCS-BIOS bytes arrive (`DCS_LIVE_MS` 3 s), the controls work locally: adjust knob moves the aircraft symbol, cage switch puts the OFF flag in view while pulled, test button centres the ILS needles while held. After `DEMO_AFTER_MS` (5 min from power-up or from the last byte) the demo starts; it ends at the first DCS byte, restoring the last DCS values. This supersedes the F4 idea of showing the OFF flag at boot. `Controls.h` has a `SARI_CONTROLS_DEBUG` build flag that prints an I2C scan and the raw pin byte (0x20, 0x21, 0x51, 0x6B were found on 2026-10-16, so the PCF8574 answers at 0x21).
- Option B, direct pins: encoder on GPIO19 and GPIO20 with the standard `DcsBios::RotaryEncoder`, cage switch on GPIO0, and no test button. The cage knob must not be in the "pulled" (low) position at power-up, or the board enters download mode.

**[-] F4. Power-off and no-data state.** Superseded 2026-10-16: Jason wants a normal display at power-up (OFF flag out of view), not the OFF flag. See the "No-DCS behaviour" bullet under F3. Still open from this item: the gauge freezes on its last values when DCS stops, and which end of `SAI_ATT_WARNING_FLAG` means "flag in view" is only inferred (65535 in view, 0 out of view, from the local flag behaviour Jason accepted).
Show the OFF flag at boot until the first DCS-BIOS data arrives, and again if no data arrives for a few seconds. Today the gauge freezes on its last values when DCS stops. First confirm which end of the `SAI_ATT_WARNING_FLAG` range means "flag in view".

**[~] F6. Night and NVG lighting.** Written 2026-10-16. NITE and NVG draw the display in green (`green_of` in `my_disp_flush`, only when `night_tint` is set), and the INST PNL knob (`INST_PNL_DIMMER`, 0x7546) sets the backlight. Mode switch `COCKKPIT_LIGHT_MODE_SW` (0x74C8, mask 0x600, shift 9): values are inferred, not read from DCS (0 DAY, 1 NITE, 2 NVG); Jason reported NITE tinted and NVG not under the first guess (0 NVG, 1 NITE, 2 DAY). Constants in the night section of the sketch: `DAY_BACKLIGHT` 50, `NIGHT_BACKLIGHT_MAX` 60, `NIGHT_MIN_LEVEL` 0.20 (knob fully off), `NVG_BRIGHTNESS` 0.7 (NVG 30% darker than NITE). Jason is still feeling out the limits. NOT fully flashed: the board has the 0.8 NVG / minimum-4 build; the 0.7 / 20% build compiles but COM11 was in use. Done when Jason confirms DAY, NITE and NVG look right in DCS. Related to F2, which is still open (it follows `INSTR_INT_LT` in day mode).

**[ ] F5. Pitch ladder accuracy.**
The globe texture carries only the 30 degree numerals plus CLIMB and DIVE. Compare with the DCS instrument and add any missing numerals and markings to the artwork.

### Phase 5 — Cleanup

**[ ] C1. Remove `README.zip`.** (Q7) 1.37 MB stale snapshot of an older version, including an unrelated `verticleVelocityIndicator.c`. It stays recoverable from git history.

**[ ] C2. `F18_SARI_v2.ino`.**
- Header comment says "v3" and "FINAL".
- `pivot_x` / `pivot_y` are file-level globals used only inside `setup()`.
- `#include <math.h>` is redundant with `Arduino.h`.
- Add `lv_obj_clear_flag(holder, LV_OBJ_FLAG_SCROLLABLE)`, so LVGL stops treating the holder as a scroll container whose child is larger than itself.
- `update_globe()` runs twice in a frame when pitch and bank both change. Harmless, but a single call from `loop()` guarded by a dirty flag is tidier.

**[ ] C3. `Display_ST7701.cpp/.h`.**
- `example_on_vsync_event` is an empty stub and the registration code is commented out (reuse it for P9 or delete it).
- Duplicate `#include` lines in the header.
- `PWM_Channel` and `Dutyfactor` are unused.
- `Light < 0` on a `uint8_t` is always false.
- `Backlight_Init()` sets brightness to 50 and `setup()` sets it to 50 again.

**[ ] C4. `TCA9554PWR.cpp/.h` and `I2C_Driver.cpp/.h`.**
- Header line 37 declares `I2C_Read_EXIO` a second time where `I2C_Write_EXIO` was meant.
- `Read_EXIOS` has its default argument on the definition, not the declaration.
- `Mode_EXIO`, `Set_Toggle`, `Set_EXIOS`, `Read_EXIO` are unused.
- `I2C_Read` and `I2C_Write` are unused, and they return `-1` from a `bool` function, so failure reads as `true`.

**[ ] C5. `lv_conf.h`.** Turn off the widgets, extra themes and layouts that are not used (everything except `LV_USE_IMG` and its `LV_USE_LABEL` dependency). This reduces build time and flash size, not frame time.

**[ ] C6. README.** Record the build settings from Q1, the library versions from section 2, and which DCS-BIOS library is intended (Q6).

---

## 6. Recommended order

M1, M2, then B1 to B4 and B6 (small, low risk), then P1, P2, P3 (the largest expected gains), then measure. If the roll test is at a steady 30 fps, go to F1. If it is not, do P4 to P7, and if that still falls short, do A1.

## 7. Things not to do

- Do not set `sw_rotate` or try to "fix" the display rotation (V4).
- Do not convert the globe or any rotated image to an indexed format (V3).
- Do not print debug text to `Serial` while DCS-BIOS is active.
- Do not move the LVGL draw buffer into PSRAM. It is in internal RAM on purpose, which is faster.

## 8. Session log

Newest first. One line per session: date, who or which model, what changed, what was measured, what is next.

- 2026-10-17 — Claude (Sonnet): SESSION END. Uncommitted work on branch `SARI-8bit-optimization` (still at `04181ef`): new `Controls.h`, edits to `F18_SARI_v2.ino`, `README.md`, new `drawings/`, this file. DONE this session: (1) controls board drawings (`drawings/sari-controls-pcb.svg` and `-protoboard.svg` plus PNGs, rendered with headless Edge; there is no Python) now show a second encoder on J4 (3-pin: P6 A, P5 B, P4 and P7 spare) and part drawings for J2 (encoder, pins 1 GND, 2 GND, 3 A, 4 B, 5 SW), J4 and J1 (6x6 tactile switch); (2) `Controls.h` reads the PCF8574 at 0x21 from a task on the other core and sends the DCS-BIOS messages, only when DCS is live; (3) no-DCS behaviour in the sketch: normal display at power-up, local controls (adjust knob moves the aircraft symbol, cage shows the OFF flag, test centres the ILS needles, the flag and needles glide at `GLIDE_PER_MS`), demo only after `DEMO_AFTER_MS` (5 min) without DCS bytes; the vertical needle parks on the left (value 0), the horizontal needle at 65535; (4) night/NVG tint (F6). FLASHED and confirmed by Jason: controls and no-DCS behaviour. FLASHED, unconfirmed: night tint with NVG 0.8 and minimum backlight 4. COMPILES BUT NOT FLASHED: NVG 0.7 and `NIGHT_MIN_LEVEL` 0.20, because COM11 was held by another program (the DCS-BIOS Hub probably; it must be disconnected from COM11 before esptool can open it, and DCS-BIOS data does not reach the board while esptool holds the port). The SARI board is COM11, confirmed by Jason this session. NEXT: flash the pending build, then Jason checks DAY, NITE and NVG in DCS (fix the value order in `onLightModeChange` if wrong), and tests the controls against DCS. Pitfalls met: the scratch edits with perl turn `\n` inside C strings into real newlines (use the Edit tool for C code with escapes); `Controls.h` has an optional `SARI_CONTROLS_DEBUG` build flag (I2C scan and raw pin byte over the DCS port at 250000 baud; build with `--build-property "compiler.cpp.extra_flags=-DSARI_CONTROLS_DEBUG=1"`); a temporary red/blue mode-diagnostic build was flashed once and removed again at Jason's request, so do not flash diagnostic builds without asking. Demo modes 1-3 (`SARI_DEMO_MODE`) were not recompiled after the demo rewrite; only mode 0 was.
- 2026-10-10 — Claude (Opus): SESSION END. No sketch code changed, nothing flashed, nothing committed. Branch `SARI-8bit-optimization` still at `04181ef`; the working tree has a rewritten `README.md` and a new `docs/` folder. (1) README (C6, written, waiting on Jason's read): hardware table, Waveshare links, board settings and FQBN, library versions, DCS-BIOS control table, demo modes, two simulated screenshots (`drawings/sari-simulated-ils.png`, `drawings/sari-simulated-off.png`). Versions verified by compile: core 3.3.10, lvgl 8.4.0, Wire 3.3.10, and the build uses `DCS-BIOS_FP-Fork` 0.3.9, not `DCS-BIOS` 0.3.10 (answers what Q6 builds with today; section 2 still says 0.3.10). Sketch 1,476,516 bytes, globals 262,236. (2) F3 design, drawings only, no code and no hardware yet: Jason wants cage, test button and the set knob, using a generic 5-pin encoder (A, common, B, plus a 2-pin push switch used for cage) and KF2510 2.54 mm connectors. Chosen part: a bare PCF8574 on the I2C header at address 0x21 (A0 high; 0x20 is the onboard TCA9554), P0 encoder A, P1 encoder B, P2 switch, P3 test button, P4-P7 spare (room for a second encoder). The once-per-frame poll (about 45 ms) is too slow for an encoder, so the plan is a background task reading the expander at about 1 kHz, counting steps, and the loop sending `SAI_SET` `+3200`/`-3200`. Fallback if I2C polling misses steps: encoder A/B on GPIO19/20 with the pulse counter. Drawings: `drawings/sari-controls-pcb.svg/.png` (33 x 33 mm board) and `drawings/sari-controls-protoboard.svg/.png` (breadboard-style protoboard, the one Jason will build; its connector pin order differs from the PCB drawing). (3) Cluster interconnect research (Jason wants fewer USB cables and COM ports for the SARI plus his airspeed, altimeter and VVI gauges on Waveshare ESP32-S3-LCD-1.85 boards), from the Waveshare schematics, not tried on hardware: the 1.85 has one native-USB port and UART0 (GPIO43/44) always on a 4-pin header, and no 5 V input pin; the 2.8C disconnects its UART header whenever its CH343 USB-C port has power, and the header is live when the board is powered from the native USB-C port or the 5 V header pin. Recommended to Jason: one 3.3 V USB-serial adapter feeding all four UART headers (one COM port), the 1.85 sketches built with USB CDC On Boot disabled, only the SARI sending back. Alternative: the ESP-NOW hub project already on this PC (`Documents\Arduino\hornet-esp32-gauges`, `Documents\Arduino\ESP32_HUB`); risk for this sketch is internal RAM (about 65 KB free) and frame rate. Jason has not chosen. Scratchpad helpers are not kept: `render.pl` (draws the simulated screenshots from the image arrays) and `proto.pl` (draws the protoboard SVG); SVG to PNG was done with headless Edge. Next: Jason's choice on the interconnect and whether to write the PCF8574 code; F1 and B6 are still `[~]` from the last session.

- 2026-10-09 — Claude (Opus): SESSION END. Branch `SARI-8bit-optimization` at `04181ef`, working tree clean apart from the untracked `CURRENT_TASKS.md`, `sketch.yaml`, `.vscode/`. The board on COM11 runs that commit as the normal DCS build (demo, readout and profile off). Open with Jason: whether his "looks good" covers the horizontal needle's direction, full-scale travel and parked positions, needle colour, the aircraft symbol under the bezel and the sliver; F1 and B6 stay `[~]` until he says. Next after that: frame rate (time the LVGL sprite share by hiding one sprite at a time), or B4/B5.

- 2026-10-09 — Claude (Opus): Jason rechecked against DCS after the `ILS_VER_DIR` flip: "looks good". Committed the working tree as `04181ef` (F1 needles, aircraft symbol as a renderer layer, B6), one commit because the three changes share the same lines in two files. The board runs this commit as the normal DCS build. F1 and B6 left at `[~]`: Jason's "looks good" did not say whether it covers the horizontal needle's direction, the full-scale travel (`ILS_TRAVEL_PX` 150), the colour, the aircraft symbol under the bezel and the sliver; asked him. Next: his answer, then mark F1/B6 `[x]`; after that the frame-rate work (time the LVGL sprite share by hiding one sprite at a time) or B4/B5.

- 2026-10-09 — Claude (Opus): first DCS comparison of the needles: Jason reports the vertical needle moves opposite to DCS and parks on the left. `ILS_VER_DIR` changed from +1 to -1 (it should now park on the right, assuming "left" described ours), compiled (1,476,516 bytes) and flashed to COM11. Waiting on Jason's recheck; horizontal needle direction, travel and colour still not reported. Not committed.

- 2026-10-09 — Claude (Opus): resumed. No source change. Compiled the working tree as the normal DCS build (demo off, readout off, profile off): 1,476,512 bytes, globals 262,236. Flashed to COM11 with esptool after Jason confirmed the port; hash verified. The board now talks to DCS-BIOS again. Working tree still not committed (ILS needles F1, aircraft symbol as a renderer layer, B6). Next: Jason compares the needles with DCS with an ILS tuned (direction of each needle, full-scale travel, parked position, colour), and also checks the layer order, the aircraft symbol under the bezel and the sliver fix; then set `ILS_TRAVEL_PX`, `ILS_VER_DIR`, `ILS_HOR_DIR`, `ILS_COLOUR` and commit.

- 2026-10-09 — Claude (Opus): SESSION END. Branch `SARI-8bit-optimization` at `0468e8f` (commits this session: `20af078` flat-run fill and inlining, `64ec9f0` three frame buffers and B3, `f21f7de` OFF flag B1/B2, `0468e8f` small sprites P8). NOT committed, in the working tree (`F18_SARI_v2.ino`, `GlobeRenderer.h`), compiling, demo and readout off in the source: the ILS needles (F1), the aircraft symbol as a renderer layer under the needles, and the B6 sliver fix. Jason has seen the needles in the demo ("looks good" apart from the order, since changed) but has not yet confirmed the new order, the aircraft symbol now being under the bezel, or the sliver fix. The board runs the demo build of that working tree (mode 1, profile on, 115200 on COM11), so it will not talk to DCS-BIOS until a normal build is flashed. Next, in order: (1) Jason's check of those three things, then commit; (2) flash the normal build and compare the needles with DCS with an ILS tuned, then set `ILS_TRAVEL_PX`, `ILS_VER_DIR`, `ILS_HOR_DIR`, `ILS_COLOUR`; (3) open question from Jason's photo: whether the light grey band in the bezel image should be repainted; (4) frame rate is about 22 FPS in demo mode 1; the LVGL sprite share (about 16 ms of a 45 ms frame) is the largest untouched part, and the next step there is to time it by hiding one sprite at a time. Scratchpad helpers are not kept: an indexed-to-TRUE_COLOR_ALPHA image converter (perl), the instrumented profiling copy of the sketch, and the build/flash/read script. Git Bash has perl, which is what replaced the missing Python for image work.

- 2026-10-09 — Claude (Opus): Jason sent a photo of a thin white sliver along the left edge of the globe as he looks at it. Cause is B6 (see there): the last two columns and rows of the bezel hole were outside the renderer's rectangle and showed LVGL's near-white screen background. Fixed with a margin and a black background. Checked the bezel image at the same time: the light grey band around the upper half of the window as mounted (screen y 271-410, both sides, ending in a straight edge) is part of `SARIBackground.c` itself, not a drawing fault. Board runs the demo build with the fix. Not committed (working tree: ILS needles, aircraft symbol as a layer, B6).

- 2026-10-09 — Claude (Opus): Jason checked the needles in the demo: they look good but must pass over the aircraft symbol, not under it. The renderer's needles became general layers (`add_layer`, `set_layer_pos`, drawn in the order added) and the aircraft symbol is now the first layer, under the two needles; it is no longer an LVGL image when the renderer is in use, and so it is now also under the bezel. Compiles (1,477,420 bytes with profile on). Mode 1: 38-49 frames per 2 s. Board runs this demo build. Not committed. Next: Jason confirms the order, then the DCS comparison for the F1 constants.

- 2026-10-08 — Claude (Opus): committed P8 (`0468e8f`). Wrote F1, the ILS needles (see F1 for how and for what is still a guess); `SAI_POINTER_VER` and `SAI_POINTER_HOR` are now read, and demo mode sweeps both needles through their full travel. Compiles (1,477,268 bytes with profile on). Mode 1: 37-48 frames per 2 s, unchanged. Board runs this demo build (mode 1, profile on). Not committed. Next: Jason checks the needles in the demo, then against DCS with an ILS tuned, and the constants are set from that.

- 2026-10-08 — Claude (Opus): P8 at Jason's request: the three small sprites converted to TRUE_COLOR_ALPHA with the scratchpad converter (same one as the OFF flag). Mode 1, frames per 2 s: 37-48, typically 43-44 (before: 36-46, typically 41), so about 5% at best and close to the run-to-run spread. The sprite format was not the main sprite cost. Board runs this demo build (mode 1, profile on). Not committed. Remaining per frame, about 45 ms: renderer about 18, copy to frame buffer about 10, LVGL about 16. Untested ideas for the LVGL share: the bank pointer and OFF flag are rotated by LVGL every frame in the demo; each sprite move makes LVGL redraw the renderer's scene under it as a separate area; drawing the sprites inside the renderer would remove both.

- 2026-10-08 — Claude (Opus): Jason confirmed the OFF flag in demo mode ("looks good"). B1 and B2 marked done and committed. Not yet seen against live DCS. At Jason's request the board stays on the demo build (mode 1, profile on, 115200), so it will not talk to DCS-BIOS until the normal build is flashed back.

- 2026-10-08 — Claude (Opus): committed `20af078` (flat-run fill, inlining) and `64ec9f0` (three frame buffers, B3). Then B1 and B2 (see those tasks): `SARICaged.c` is now TRUE_COLOR_ALPHA and the flag angle is set directly. Compiles (1,454,908 bytes). Mode 1 with the flag now really rotating: 36-46 frames per 2 s, about the same as before. The board runs the demo build (mode 1, profile on) for Jason to check the flag: it should swing smoothly between hanging over the right side of the window and lying level; when level, the part near its pivot is inside the round glass, over the bezel, so check whether anything shows there that should not. Not committed. Next: Jason's verdict, flash the DCS build back, then Q2 can be closed.

- 2026-10-08 — Claude (Opus): Jason confirmed the three-buffer picture in demo mode ("looks good so far"). Then looked for where the renderer's 17-24 ms goes, with an instrumented copy of the sketch in the scratchpad (pixel counters, a calibration loop, a scanline micro-benchmark, disassembly). Findings: (1) the panel interrupt is not the cause: a fixed internal-RAM workload runs only about 10% slower with the panel running than before `LCD_Init()`; my earlier guess was wrong. (2) At `-Os` the compiler did not inline `mix565`, `sample_inside` and friends, so a blended pixel cost about 1.1 us. `GlobeRenderer.h` now forces them inline and is built at `-O2` (`#pragma GCC optimize`): micro-benchmark, 55,000 px: flat sky 2.4 ms to 1.3 ms, all-blended row 60 ms to 15 ms. In the real frame the gain is small: renderer 17-19 ms, frames per 2 s 39-52 (typically 43-45, about 22 FPS). (3) Tried and dropped: repacking the texture into internal RAM as palette + per-row spans (slower per sample than reading PSRAM directly: 455 against 331 ns); `-O2` for the whole build through `build_opt.h` (the first half of P7; no change, +40 KB). (4) Per frame in mode 1, per 2 s of 43 frames: renderer about 800 ms, of which about 240 ms is bezel copy and edge pixels; copy to frame buffer 430 ms; the rest, about 700 ms, is LVGL drawing the sprites. In the window about 31,000 px per frame are cheap coarse fills, 17,000 are short fine runs and 9,000 are blended, although only 1.4% of the texture's texels are not background, so most of the fine and blended work is spent on plain colour near a line. The demo also swings the OFF flag constantly, which real flight does not. Working tree: flat-run fill, B3, three frame buffers, inlining; board flashed with that as the normal DCS build (demo off). Not committed. Ideas left, biggest first: the LVGL sprite share (OFF flag B1/B2, P8, drawing sprites in the renderer); a distance map in place of the two flat maps (each 4x4 block stores how many blocks away the nearest detail is, so runs are not cut at block edges); run-fill for rows over the pole (still pixel by pixel, about 430 ns each).

- 2026-10-08 — Claude (Opus): measured both changes, mode 1 over serial, frames per 2 s (before: 35-39). Flat-run fill alone: 35-40, renderer still 17-24 ms per frame, so it gained nothing measurable; the renderer's cost is not the per-pixel lookup. Three frame buffers on top: 36-51, typically 42-46 (about 21-23 FPS); the swap wait fell from about 6.5 ms to under 0.1 ms per frame, resync about 1 ms. Per frame now roughly 45 ms: renderer 17-24, copy to frame buffer 10, LVGL sprites and the rest about 13. The renderer's time went up slightly when the swap wait went away, which points at the panel interrupt taking a large share of the core (time it used to take during the wait now lands inside the measured sections). Board left on the three-buffer demo build (mode 1, profile on, 115200) for Jason to check for tearing and ghosting. Not committed. Next: find where the renderer's 17-24 ms goes (count pixels per path in the profile build; check whether `flat_map` landed in internal RAM or PSRAM), then the OFF flag.

- 2026-10-08 — Claude (Opus): three frame buffers written, compiles (1,439,012 bytes, globals 262,044), not flashed, not committed. `ESP_PANEL_LCD_RGB_FRAME_BUF_NUM` is 3. `LCD_showFrameBuffer` no longer waits for the panel to switch; the sketch rotates `fb_front` / `fb_spare` / `fb_back`, and `LCD_waitForSpareFrameBuffer()` at the start of `refresh_display()` only waits if the panel has not finished a frame since the show call before the last one (it counts frames in the panel interrupt). The buffer drawn into is now two frames old, so the resync covers the areas of the last two frames (`stale_areas[2]`). To check on hardware: no tearing, no ghosting or leftover sprite trails (slip ball, rate of turn, bank pointer, OFF flag), and the frame count in the profile output. The board still runs the DCS build with two buffers. Notes: this build picks `DCS-BIOS_FP-Fork` over `DCS-BIOS` for `DcsBios.h` (compiler message; bears on Q6). `arduino-cli compile --output-dir` also writes a `build/` folder into the sketch folder; delete it afterwards. Do not rewrite `F18_SARI_v2.ino` with PowerShell `Get-Content`/`Set-Content`: it holds non-ASCII characters and PowerShell 5.1 corrupts them (use the Edit tool).

- 2026-10-08 — Claude (Opus): flashed the normal build (demo off, readout off, flat-run fill + B3) to COM11 with esptool; Jason confirmed COM11 (a second CH343 board is on COM20). First test of the custom renderer against live DCS: Jason reports no issues. B3 marked done. The flat-run fill is confirmed for picture only; its frame-time gain is not measured yet. Not committed.

- 2026-10-08 — Claude (Opus): two changes written, both compile (1,438,840 bytes, globals 261,932), neither flashed nor measured nor committed. (1) A1 flat-run fill in `GlobeRenderer.h`: a second flat-colour map with 16x16 blocks (`coarse_map`) and `draw_inside_run`, which fills every pixel of a scanline that stays in the same flat block in one go instead of one lookup per pixel. It only applies to runs wholly inside the texture; rows that cross the pole (beyond about 67 degrees of pitch) still go pixel by pixel. To check on hardware: picture identical to before, in particular no wrong-coloured specks along the horizon line and pitch lines while rolling, and the renderer time (was 20 ms per frame) in the `SARI_PROFILE` output. (2) B3. The board still runs the old demo build. Next: flash and measure (1), then the live DCS test, then three frame buffers, then the OFF flag.

- 2026-10-08 — Claude (Opus): SESSION END. Branch `SARI-8bit-optimization` at `9720e1f`, working tree clean apart from the untracked `CURRENT_TASKS.md`, `sketch.yaml`, `.vscode/`. The board is still running the demo build (mode 1, readout on); the committed source has demo and readout off. The renderer has only been tested in demo mode, not against live DCS. Next, in order: (1) test against DCS; (2) fill flat runs in `GlobeRenderer.h` (renderer is 20 ms of a 53 ms frame); (3) three frame buffers to drop the 6.5 ms swap wait; (4) OFF flag (B1, B2). Helper scripts (profiling over serial, image crop/rotate) were in the session scratchpad and are not kept; the profile build is `-DSARI_PROFILE=1` with demo mode on, read at 115200 on COM11.
- 2026-10-08 — Claude (Opus): Jason confirmed the latest build: bank pointer right, no flicker, tearing or ghosting, 14-21 FPS on the readout. Committed as the commit after `03964b6` (renderer, cropped bank pointer, resync skip), demo and readout off. Note: the on-screen CPU % now reads 0 because the refresh runs outside `lv_timer_handler`; the FPS number is still real. A1 and P1 confirmed on hardware.
- 2026-10-08 — Claude (Opus): Jason confirmed the A1 picture: looks right, stable, smooth lines, 7-17 FPS. Then, mode 1 over serial, frames per 2 s: bounce buffer off at 16 MHz 21-24 (no real gain, so the interrupt is not the bottleneck at 16 MHz; bounce buffer back on, my earlier 60% estimate was wrong); flat-colour shortcut added to the pole (checked) sampling path 22-25; bank pointer cropped to 22x28 (P1 redone, `Bank.c`) 27-29; resync now skips whatever the coming refresh will redraw anyway (`refresh_display`) 35-39, about 18-19 FPS. Per frame now roughly: renderer 20 ms, copy to hidden buffer 10 ms, swap wait 6.5 ms, LVGL sprites 14 ms, resync under 1 ms. Flashed for Jason to check (bank pointer arc, no ghosting from the resync skip). Not committed. Ideas left: fill flat runs of the globe in one go instead of per pixel; three frame buffers to drop the swap wait; the OFF flag is a rotated indexed image (B1/V3).
- 2026-10-08 — Claude (Opus): committed the checkpoint as `03964b6` (P9 + P5 + M1/M2). Wrote A1 as `GlobeRenderer.h`, switched on by `SARI_GLOBE_RENDERER` in the sketch (0 = old LVGL globe). One full-screen LVGL object draws the bezel (pre-converted to RGB565 in PSRAM at boot) and samples the globe texture per scanline with bilinear filtering, pole mirroring past +/-90 degrees, and a 4x4-block flat-colour map in internal RAM (the texture is 99% two colours). Sprites are still LVGL images on top. Mode 1 over serial: 18-23 frames per 2 s (about 10 FPS; LVGL globe was 11-15 per 2 s). Renderer itself takes 30-45 ms per frame; the flat map did not speed it up, so the limit is CPU time: at 16 MHz the bounce interrupt copies about 26 MB/s of a roughly 42 MB/s PSRAM read ceiling on the same core. Flashed for Jason to check the picture. Not committed. Ideas left: bounce buffer off at 16 MHz (it drifted at 30 MHz, untested at 16), a lower pixel clock still, cropping the bank pointer (its redraw box covers the whole window).
- 2026-10-08 — Claude (Opus): Jason confirmed 16 MHz shows no flicker. P5 and P9 marked done. Not committed yet.
- 2026-10-08 — Claude (Opus): bounce buffer off gave severe sideways drift (Jason video), so it is back on; do not turn it off. With bounce on and the two-buffer swap, Jason confirmed: picture stable, tearing gone (P9 works). Mode 1, frames per 2 s over serial: 4-8 at 30 MHz, 11-15 at 16 MHz. Flashed 16 MHz (P5 alone, panel on the original core) for Jason to check for flicker. Working tree: P9 + 16 MHz, demo mode 1, readout on, nothing committed. Next: if no flicker keep 16 MHz, then A1.
- 2026-10-08 — Claude (Opus): P9 written and flashed, waiting on Jason. Final form: panel has two frame buffers (`ESP_PANEL_LCD_RGB_FRAME_BUF_NUM` 2); LVGL still renders into its internal buffer; the flush copies into the hidden frame buffer and swaps at frame end (`LCD_showFrameBuffer`, waits for the panel `on_frame_buf_complete` signal); before each refresh the areas drawn last frame are copied from the shown buffer to the hidden one. The sketch now runs the refresh itself (`refresh_display()`, LVGL refresh timer deleted). Bounce buffer set to 0 (panel reads PSRAM by DMA). Measured, mode 1, frames per 2 s: LVGL direct mode 4-8 (rendering into PSRAM is slow); copy-and-swap with bounce buffer on 4-7 (PSRAM writes only about 6-9 MB/s because the bounce interrupt eats the core); redraw instead of copy for the resync 3-6; bounce buffer off 11-12; bounce off plus copy resync 13-16 (about 7 FPS, baseline 3-4). Per frame then: copy 10 ms, swap 13 ms, resync 17 ms, LVGL render about 90 ms. Key finding: with the bounce buffer on, the panel interrupt (about 50 MB/s PSRAM copy on the LVGL core) is the main thing starving LVGL. Risk to check: Waveshare enabled the bounce buffer to avoid screen drift. Next: Jason checks for tearing and drift, then A1.
- 2026-10-08 — Claude (Opus): at Jason's request reverted the sketch folder to the baseline (`c0982b8`) plus M1, M2 and the `SARI_PROFILE` timing code (off by default), and flashed it. P0, P1, P2, P4, P5, P6 and the PSRAM copies are all out. `SARI_DEMO_MODE` is 1 and `SARI_PERF_MONITOR` is 1 in the working tree; set both to 0 before committing. `arduino-cli --upload` still fails on this board; flash with esptool: `esptool --chip esp32s3 --port COM11 --baud 460800 write-flash -z 0x10000 <build>/F18_SARI_v2.ino.bin`. Next: P9 (two frame buffers, swap on vsync) for the glitching, then A1 for frame rate.
- 2026-10-08 — Claude (Opus): bisected the glitching by eye with Jason, flashing builds from a scratch copy. Baseline (HEAD + M1/M2): Jason sees no flicker. P0 + P1: glitches. P0 only: glitches. So P0 (pre-rotated globe) is implicated; P1 alone and the PSRAM copies alone are untested. Caveat: the baseline video `IMG_9847.MOV` also showed torn frames, so the baseline is not tear-free, only less visible. Mechanism for P0 not understood. PSRAM image copies removed from the working tree. Working tree is P0 + P1, demo mode 1, readout on; board has P0 only. Nothing committed.
- 2026-10-08 — Claude (Opus): Jason reported jagged pitch lines (P4), flicker and image shifting; `IMG_9855.MOV` showed full sideways drift. Reverted P4 (antialias back on), P5 (back to 30 MHz) and P6 (panel back on the loop core). Working tree now: P0, P1, globe and bezel in PSRAM, 30 MHz, original core. Mode 1 over serial: 7-14 frames per 2 s (about 4-7 FPS). The earlier 10-15 FPS figures all depended on P6 and are not usable. Next: confirm the picture is stable again, then A1.
- 2026-10-08 — Claude (Sonnet): profiled over serial (temporary `SARI_PROFILE` code in the sketch, off by default; I read COM11 with a PowerShell SerialPort). Measured: flash to RAM 18 MB/s, PSRAM to RAM 42, RAM to PSRAM 24.5, RAM to RAM 365. LVGL render is about 80% of frame time, flush about 18%. Hiding either the globe or the bezel alone reaches the 33 fps timer cap, so their combined masked blending is the cost. Results (pitch-only, frames per 2 s): baseline about 16; P2 option A (RGB565A8 bezel, 7-8 FPS) no gain, reverted; P1 (crop bank) no gain in pitch-only but needed for roll; 16 MHz pixel clock about 29 (P5); panel init on core 0 about 33 (P6); bounce buffer off no change, reverted; globe and bezel copied into PSRAM at boot 36-48. Mode 1 on screen: 10-12 FPS (18-24 frames per 2 s over serial). P4 (globe antialias off): 23-30 frames per 2 s, about 12-15 FPS, waiting on Jason's view of the line quality. Nothing committed. Next: P3, then A1.
- 2026-10-08 — Claude (Sonnet): did P0 (rotated `SARIGlobe.c` with a PowerShell script in the scratchpad, changed the angle and pivot in the sketch). Flashed demo mode 2: 7 FPS, CPU 96-97%, tearing still visible occasionally in `IMG_9848.MOV`. `arduino-cli` upload failed three times at sector 0x100000 ("Serial data stream stopped"); flashing the app image with esptool directly at 460800 worked. Next: P2 (bezel blend), P1 (bank pointer), P3.
- 2026-10-08 — Claude (Sonnet): flashed demo modes 3 then 1 to COM11 (Jason confirmed the port). Both read "33" most of the time and 3-5 otherwise. Found in the LVGL source that 33 is the fallback cap, not a measurement, so the real rate is 3-5 FPS in every mode. Video IMG_9847.MOV shows slow steps and tearing. `SARI_DEMO_MODE` is left at 1 in the working tree. Next: P0, then P1, P2, P3, measuring after each.
- 2026-10-08 — Claude (Sonnet): Jason confirmed the demo mode runs on hardware with all elements moving (M1 `[x]`). FPS/CPU counters work after the tick fix. Baseline modes 3 and 1 still to be measured. Next: decide between measuring them and starting P0.
- 2026-10-07 — Claude (Sonnet): installed ffmpeg (winget, Gyan.FFmpeg) and reviewed Jason's video `C:\Users\jason\Videos\IMG_9841.MOV` (old build). Findings: (1) visible tearing: bands of the picture shifted sideways and doubled lines on the bezel for single frames, which fits LVGL writing into the frame buffer while the panel scans it (P9, P5, P6); (2) a diagonal stripe pattern over the globe window in some frames, possibly camera moire, unconfirmed (Jason to say whether it is visible by eye); (3) the globe looks pale white-cyan rather than the deep blue in the image file, also unconfirmed, possibly camera exposure (Jason to say what colour it is by eye); (4) the OFF flag does move. Fixed the LVGL tick (see M2). Pitch-only baseline 5-6 FPS, CPU 95%+. Found P0. Next: baseline modes 3 and 1.
- 2026-10-07 — Claude (Sonnet): flashed the unchanged baseline to COM11; Jason confirmed the gauge looks normal, so the inferred board settings are good (Q1 closed). Wrote M1 and M2 (all four demo modes compile). Not yet flashed. Next: Jason (or Claude, when asked) flashes demo mode 2, 3, 1 with the perf monitor on and fills in the baseline table.
- 2026-10-07 — Claude (Opus): read the Waveshare wiki for pins and settings; added `sketch.yaml` and `.vscode/tasks.json`; test compile passed (1,453,160 bytes). Nothing uploaded. Next: Jason flashes once with the new settings to confirm them, then M1.
- 2026-10-07 — Claude (Opus): full code review, created this file. No code changed. Baseline FPS not yet measured. Next: M1.
