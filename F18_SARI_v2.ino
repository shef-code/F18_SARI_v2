// F18_SARI_v3 — FINAL, 100% COMPILING VERSION (CLEANED / OPTIMIZED)
// ESP32-S3 + ST7701 + LVGL8 + DCS-BIOS (Arduino-ESP32 2.x/3.x)

#define DCSBIOS_DEFAULT_SERIAL
#define DCSBIOS_DISABLE_SERVO

#include <Arduino.h>
#include <DcsBios.h>
#include <lvgl.h>
#include <math.h>
#include "I2C_Driver.h"
#include "TCA9554PWR.h"
#include "Display_ST7701.h"

extern "C" {
  extern const lv_img_dsc_t SARIBackground, SARIBug, RateOfTurn, SlipBall;
  extern const lv_img_dsc_t Bank, SARIGlobe, SARICaged;
}

// ────────────────────────────── Config ──────────────────────────────
constexpr int  DISP_W = 480, DISP_H = 480;

constexpr int  BUG_Y_OFFSET       = 17;
constexpr int  GLOBE_Y_OFFSET     = 33;
constexpr int  BANK_Y_OFFSET      = 33;
constexpr int  CAGED_X_OFFSET     = -157;
constexpr int  CAGED_Y_OFFSET     = 96;

constexpr int  PITCH_DIR          = +1;
constexpr float PITCH_SCALE       = 1.0f;

constexpr int  SLIP_MAX_X_PX      = 90;
constexpr int  SLIP_BASE_Y        = -150;
constexpr float SLIP_Y_LIFT       = 7.0f;

constexpr int  ROT_MAX_X_PX       = 75;
constexpr int  ROT_BASE_Y         = -200;

constexpr int  GLOBE_CAL_X        = 0;
constexpr int  GLOBE_CAL_Y        = 0;

constexpr int BUG_MAX_TRAVEL_PX = 70;    

constexpr uint16_t DCS_MID  = 32782;
constexpr uint16_t DCS_HALF = 32768;

// ────────────────────────────── LVGL Objects ──────────────────────────────
static lv_obj_t *globe_img = nullptr;
static lv_obj_t *bank_img  = nullptr;
static lv_obj_t *bug_img   = nullptr;
static lv_obj_t *slip_img  = nullptr;
static lv_obj_t *rot_img   = nullptr;
static lv_obj_t *caged_img = nullptr;

// ────────────────────────────── State ──────────────────────────────
static int16_t pitch_x10 = 0;
static int16_t bank_x10  = 0;

static int pivot_x = 0, pivot_y = 0;
static int base_globe_x = 0, base_globe_y = 0;
static int pitch_px_per_deg = 0;

// ────────────────────────────── Helpers ──────────────────────────────
static inline int mapInt(uint16_t v, uint16_t inMin, uint16_t inMax, int outMin, int outMax) {
  if (v <= inMin) return outMin;
  if (v >= inMax) return outMax;
  return outMin + (int)((uint32_t)(v - inMin) * (uint64_t)(outMax - outMin) / (inMax - inMin));
}

// ────────────────────────────── Globe Update ──────────────────────────────
static inline void update_globe() {
  if (!globe_img) return;

  const float pitch_px = (pitch_x10 / 10.0f) * (float)PITCH_DIR * (float)pitch_px_per_deg;
  const float bank_rad = (float)bank_x10 * 0.0017453292519943296f; // deg*10 -> radians

  const float s = sinf(bank_rad);
  const float c = cosf(bank_rad);

  const int dx = lrintf(-s * pitch_px);
  const int dy = lrintf( c * pitch_px);

  lv_obj_set_pos(globe_img, base_globe_x + dx, base_globe_y + dy);
}

// ────────────────────────────── DCS-BIOS CALLBACKS (unsigned int!) ──────────────────────────────
void onSaiPitchChange(unsigned int v) {
  pitch_x10 = mapInt((uint16_t)v, 0, 65535, -900, 900);
  update_globe();
}
DcsBios::IntegerBuffer saiPitchBuffer(0x74e4, 0xffff, 0, onSaiPitchChange);

void onSaiBankChange(unsigned int v) {
  bank_x10 = mapInt((uint16_t)v, 0, 65535, 0, 3600);
  const int16_t angle = (int16_t)(bank_x10 % 3600); // keep your original behavior
  lv_img_set_angle(bank_img,  angle);
  lv_img_set_angle(globe_img, angle);
  update_globe();
}
DcsBios::IntegerBuffer saiBankBuffer(0x74e6, 0xffff, 0, onSaiBankChange);

void onSaiSlipBallChange(unsigned int v) {
  int delta = (int)v - (int)DCS_MID;
  int x = (DCS_MID != 0) ? (delta * SLIP_MAX_X_PX / (int)DCS_MID) : 0;

  // reverse left/right (keep)
  x = -x;

  x = constrain(x, -SLIP_MAX_X_PX, SLIP_MAX_X_PX);
  const float ratio = fabsf((float)x) / (float)SLIP_MAX_X_PX;
  const int y = SLIP_BASE_Y + (int)(ratio * SLIP_Y_LIFT);

  lv_obj_align(slip_img, LV_ALIGN_CENTER, x, y);
}
DcsBios::IntegerBuffer saiSlipBallBuffer(0x74ec, 0xffff, 0, onSaiSlipBallChange);

void onSaiRateOfTurnChange(unsigned int v) {
  int delta = (int)v - (int)DCS_MID;
  int x = (DCS_MID != 0) ? (delta * ROT_MAX_X_PX / (int)DCS_MID) : 0;

  // reverse left/right (keep)
  x = -x;

  x = constrain(x, -ROT_MAX_X_PX, ROT_MAX_X_PX);
  lv_obj_align(rot_img, LV_ALIGN_CENTER, x, ROT_BASE_Y);
}
DcsBios::IntegerBuffer saiRateOfTurnBuffer(0x74ee, 0xffff, 0, onSaiRateOfTurnChange);

void onSaiManPitchAdjChange(unsigned int v) {
  int delta = (int)v - (int)DCS_MID;

  // Map DCS full deflection to ±70 px
  int y = (DCS_HALF != 0) ? (delta * BUG_MAX_TRAVEL_PX / (int)DCS_HALF) : 0;

  // Clamp and apply your neutral offset
  y = constrain(y, -BUG_MAX_TRAVEL_PX, BUG_MAX_TRAVEL_PX) + BUG_Y_OFFSET;

  lv_obj_align(bug_img, LV_ALIGN_CENTER, 0, y);
}
DcsBios::IntegerBuffer saiManPitchAdjBuffer(0x74ea, 0xffff, 0, onSaiManPitchAdjChange);

static int16_t current_caged_angle = -900;
void onSaiAttWarningFlagChange(unsigned int v) {
  const int16_t target = (int16_t)mapInt((uint16_t)v, 0, 65535, -900, 0);

  if (abs(target - current_caged_angle) < 8) {
    lv_img_set_angle(caged_img, target);
    current_caged_angle = target;
    return;
  }

  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, caged_img);
  lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_img_set_angle);
  lv_anim_set_values(&a, current_caged_angle, target);
  lv_anim_set_time(&a, 280);
  lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
  lv_anim_start(&a);

  current_caged_angle = target;
}
DcsBios::IntegerBuffer saiAttWarningFlagBuffer(0x74e8, 0xffff, 0, onSaiAttWarningFlagChange);

// ────────────────────────────── Display Flush ──────────────────────────────
static void my_disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *buf) {
  LCD_addWindow(area->x1, area->y1, area->x2, area->y2, (uint8_t*)buf);
  lv_disp_flush_ready(drv);
}

// ────────────────────────────── Setup ──────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  I2C_Init();
  TCA9554PWR_Init(0x00);
  Set_EXIO(EXIO_PIN8, Low);    // Turns off buzzer
  LCD_Init();
  Backlight_Init();
  Set_Backlight(50);

  lv_init();

  static lv_color_t buf[DISP_W * 192];
  static lv_disp_draw_buf_t draw_buf;
  lv_disp_draw_buf_init(&draw_buf, buf, nullptr, DISP_W * 192);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = DISP_W;
  disp_drv.ver_res = DISP_H;
  disp_drv.draw_buf = &draw_buf;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.rotated = LV_DISP_ROT_180;
  lv_disp_drv_register(&disp_drv);

  lv_obj_t *scr = lv_scr_act();

  // Globe pitch scale (keep original math)
  const int globe_w = SARIGlobe.header.w;
  const int globe_h = SARIGlobe.header.h;
  pitch_px_per_deg = lrintf(((float)globe_h / 180.0f) * PITCH_SCALE);

  constexpr int GLOBE_VP_W = 275;
  constexpr int GLOBE_VP_H = 275;

  lv_obj_t *holder = lv_obj_create(scr);
  lv_obj_remove_style_all(holder);
  lv_obj_set_size(holder, GLOBE_VP_W, GLOBE_VP_H);
  lv_obj_center(holder);
  lv_obj_set_y(holder, GLOBE_Y_OFFSET);
  lv_obj_set_scrollbar_mode(holder, LV_SCROLLBAR_MODE_OFF); // keep

  pivot_x = globe_w / 2 + GLOBE_CAL_X;
  pivot_y = globe_h / 2 + GLOBE_CAL_Y;

  base_globe_x = (GLOBE_VP_W / 2) - pivot_x;
  base_globe_y = (GLOBE_VP_H / 2) - pivot_y;

  globe_img = lv_img_create(holder);
  lv_img_set_src(globe_img, &SARIGlobe);
  lv_img_set_pivot(globe_img, pivot_x, pivot_y);
  lv_obj_set_pos(globe_img, base_globe_x, base_globe_y);

  lv_obj_t *bg = lv_img_create(scr);
  lv_img_set_src(bg, &SARIBackground);
  lv_obj_center(bg);

  bank_img = lv_img_create(scr);
  lv_img_set_src(bank_img, &Bank);
  lv_obj_align(bank_img, LV_ALIGN_CENTER, 0, BANK_Y_OFFSET);
  lv_img_set_pivot(bank_img, Bank.header.w / 2, Bank.header.h / 2);

  bug_img  = lv_img_create(scr); lv_img_set_src(bug_img,  &SARIBug);     lv_obj_align(bug_img,  LV_ALIGN_CENTER, 0, BUG_Y_OFFSET);
  slip_img = lv_img_create(scr); lv_img_set_src(slip_img, &SlipBall);    lv_obj_align(slip_img, LV_ALIGN_CENTER, 0, SLIP_BASE_Y);
  rot_img  = lv_img_create(scr); lv_img_set_src(rot_img,  &RateOfTurn);  lv_obj_align(rot_img,  LV_ALIGN_CENTER, 0, ROT_BASE_Y);

  caged_img = lv_img_create(scr);
  lv_img_set_src(caged_img, &SARICaged);
  lv_obj_align(caged_img, LV_ALIGN_CENTER, CAGED_X_OFFSET, CAGED_Y_OFFSET);
  lv_img_set_pivot(caged_img, SARICaged.header.w, SARICaged.header.h);
  lv_img_set_angle(caged_img, 900);

  DcsBios::setup();

  // Force neutral position (keep)
  onSaiPitchChange(DCS_MID);
  onSaiBankChange(DCS_MID);
  onSaiSlipBallChange(DCS_MID);
  onSaiRateOfTurnChange(DCS_MID);
  onSaiManPitchAdjChange(DCS_MID);
  onSaiAttWarningFlagChange(65535);
}

void loop() {
  static uint32_t last_ms = 0;
  const uint32_t ms = millis();
  lv_tick_inc(ms - last_ms);
  last_ms = ms;

  lv_timer_handler();
  DcsBios::loop();
}
