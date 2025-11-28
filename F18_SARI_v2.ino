// F18_SARI_v2 - Draw the Needle bitmap with LVGL on ST7701 (ESP32-S3 RGB)
// - Uses your I2C_Driver, TCA9554PWR, and Display_ST7701 helpers
// - Displays the needle image from Needle.c and lets you rotate it
//
// NOTE: In lv_conf.h, ensure the config is enabled (#if 1), LV_COLOR_DEPTH=16, LV_USE_IMG=1.

#include <Arduino.h>
#include <lvgl.h>

#include "I2C_Driver.h"
#include "TCA9554PWR.h"
#include "Display_ST7701.h"   // LCD_Init(), LCD_addWindow(), Backlight_Init(), Set_Backlight()

// Pull in the C symbol for the needle image descriptor defined in Needle.c
extern "C" {
  extern const lv_img_dsc_t Needle;  // width=27, height=350, LV_IMG_CF_TRUE_COLOR_ALPHA
}

// ===== Resolution (from your ST7701 config) =====
#ifndef ESP_PANEL_LCD_WIDTH
  #define ESP_PANEL_LCD_WIDTH  480
#endif
#ifndef ESP_PANEL_LCD_HEIGHT
  #define ESP_PANEL_LCD_HEIGHT 480
#endif
#define DISP_WIDTH   ESP_PANEL_LCD_WIDTH
#define DISP_HEIGHT  ESP_PANEL_LCD_HEIGHT

// ===== LVGL draw buffer =====
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[DISP_WIDTH * 40];   // ~38 KB (40 lines @ RGB565)

// ===== LVGL -> Panel flush bridge (LVGL 8.4: non-const color pointer) =====
static void my_disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
  uint16_t x1 = (uint16_t)area->x1;
  uint16_t y1 = (uint16_t)area->y1;
  uint16_t x2 = (uint16_t)area->x2;
  uint16_t y2 = (uint16_t)area->y2;

  // Your driver expects raw RGB565 bytes; it adjusts end coords to exclusive internally
  LCD_addWindow(x1, y1, x2, y2, (uint8_t*)color_p);

  lv_disp_flush_ready(drv);
}

// ===== Optional: simple demo rotation =====
static lv_obj_t *needle_img = nullptr;
static int16_t angle_tenths = 0;  // LVGL angle is in 0.1° units

void setup() {
  Serial.begin(115200);

  // --- I/O / buzzer off ---
  I2C_Init();
  TCA9554PWR_Init(0x00);     // all EXIO as outputs
  Set_EXIO(EXIO_PIN8, Low);  // keep buzzer off (on EXIO8)

  // --- Panel + backlight ---
  LCD_Init();
  Backlight_Init();
  Set_Backlight(100);

  // --- LVGL core + display registration ---
  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf1, nullptr, DISP_WIDTH * 40);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res  = DISP_WIDTH;
  disp_drv.ver_res  = DISP_HEIGHT;
  disp_drv.draw_buf = &draw_buf;
  disp_drv.flush_cb = my_disp_flush;
  lv_disp_drv_register(&disp_drv);

  // --- Screen + needle image ---
  lv_obj_t *scr = lv_obj_create(nullptr);
  lv_disp_load_scr(scr);

  needle_img = lv_img_create(scr);
  lv_img_set_src(needle_img, &Needle);  // Bitmap from Needle.c
  lv_obj_center(needle_img);

  // Set a pivot so the needle rotates around its tail (tweak to taste)
  // Needle is 27x350; pivot near bottom-center:
  lv_img_set_pivot(needle_img, 13, 330);     // X ~ center (0..26), Y near bottom (0..349)
  lv_img_set_angle(needle_img, 0);           // 0.1° units (e.g., 900 = 90 degrees)
}

void loop() {
  // LVGL timing
  lv_tick_inc(5);
  lv_timer_handler();
  delay(5);

  // --- Optional: slow sweep demo (comment out for static needle) ---
  // angle_tenths ranges roughly -3000..3000 (~-300°..300°) as a demo
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last >= 20) { // update every 20 ms
    last = now;
    angle_tenths += 5;    // +0.5°/sweep step (adjust to your liking)
    if (angle_tenths > 3000) angle_tenths = -3000;
    lv_img_set_angle(needle_img, angle_tenths);
  }
}
