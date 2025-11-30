// F18_SARI_v2 - LVGL + DCS-BIOS (ESP32-S3 RGB, ST7701)
// - SlipBall & RateOfTurn: L/R ±150 px
// - SARIBug: U/D ±150 px
//
// lv_conf.h: LV_COLOR_DEPTH=16, LV_USE_IMG=1, LV_DRAW_COMPLEX=1, LV_USE_IMG_TRANSFORM=1

#define DCSBIOS_DEFAULT_SERIAL
#define DCSBIOS_DISABLE_SERVO

#include <DcsBios.h>
#include <Arduino.h>
#include <lvgl.h>

#include "I2C_Driver.h"
#include "TCA9554PWR.h"
#include "Display_ST7701.h"   // LCD_Init(), LCD_addWindow(), Backlight_Init(), Set_Backlight()

extern "C" {
  extern const lv_img_dsc_t SARIBackground;
  extern const lv_img_dsc_t SARIBug;
  extern const lv_img_dsc_t RateOfTurn;
  extern const lv_img_dsc_t SlipBall;
  extern const lv_img_dsc_t Bank;
  extern const lv_img_dsc_t SARIGlobe;
}

// ===== UI objects =====
static lv_obj_t *bug_img        = nullptr; 
static lv_obj_t *slipBall_img   = nullptr; 
static lv_obj_t *rateOfTurn_img = nullptr; 
static lv_obj_t *bank_img       = nullptr;  
static lv_obj_t *globe_img      = nullptr;  


// ===== Resolution (from your ST7701 config) =====
#define DISP_WIDTH   480
#define DISP_HEIGHT  480

// ===== LVGL draw buffer =====
static lv_disp_draw_buf_t draw_buf;

// Use a single buffer of 64 lines (≈ 480 * 64 * 2 bytes = ~61 KB)
#define BUF_LINES 64
static lv_color_t buf1[DISP_WIDTH * BUF_LINES];

// ===== LVGL -> Panel flush bridge =====
static void my_disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
  uint16_t x1 = (uint16_t)area->x1;
  uint16_t y1 = (uint16_t)area->y1;
  uint16_t x2 = (uint16_t)area->x2;
  uint16_t y2 = (uint16_t)area->y2;
  LCD_addWindow(x1, y1, x2, y2, (uint8_t*)color_p);
  lv_disp_flush_ready(drv);
}


// ===== PITCH → vertical scroll config =====
// Map ±90° pitch to ±180 px movement (2 px per degree). Tweak to taste.
static constexpr int  PITCH_MAX_DEG      = 90;   // clamp range
static constexpr int  PITCH_PX_PER_DEG   = 2;    // pixels per degree
static constexpr int  PITCH_MAX_PX       = PITCH_MAX_DEG * PITCH_PX_PER_DEG;


// ================= DCS-BIOS CALLBACKS =================
// Each callback has its own inline mapping constants and clamp helper.

// Slip/Skid ball → move SlipBall left/right around y=160
void onSaiSlipBallChange(unsigned int newValue) {
  // ===== Mapping config (inline) =====
  static constexpr long MID_CODE   = 32782;   // center code
  static constexpr long HALF_SPAN  = 32768;   // half-range
  static constexpr int  MAX_X_PX   = 90;     // ±90 px horizontal travel
  static constexpr int  BASE_Y     = 160;     // baseline vertical position
  static constexpr float MAX_Y_UP  = 7.0f;    // total upward shift at max deflection (px)
  auto clampInt = [](int v, int lo, int hi){ return (v < lo) ? lo : (v > hi) ? hi : v; };

  // compute horizontal offset
  long delta = (long)newValue - MID_CODE;
  long x = (HALF_SPAN != 0) ? (delta * MAX_X_PX) / HALF_SPAN : 0;
  x = clampInt((int)x, -MAX_X_PX, MAX_X_PX);

  // compute small vertical shift (up 5 px at extremes)
  // use absolute value so center=0 px, edge=-5 px
  float ratio = (float)abs(x) / (float)MAX_X_PX;
  int y = BASE_Y - (int)(ratio * MAX_Y_UP);

  lv_obj_align(slipBall_img, LV_ALIGN_CENTER, x, y);
}
// raw address buffer (keep your values)
DcsBios::IntegerBuffer saiSlipBallBuffer(0x74ec, 0xffff, 0, onSaiSlipBallChange);

// Rate of Turn needle → move RateOfTurn left/right around y=215
void onSaiBankChange(unsigned int newValue) 
{
  // Map 0..65535 → 0..3600 (0.1° units)
  int angle_tenths = map((int)newValue, 0, 65535, 0, 3600);

  // Shift by 180° to match DCS (wrap to 0..359.9°)
  angle_tenths = (angle_tenths + 1800) % 3600;

  lv_img_set_angle(bank_img, angle_tenths);
  lv_img_set_angle(globe_img, angle_tenths);
}
DcsBios::IntegerBuffer saiBankBuffer(0x74e6, 0xffff, 0, onSaiBankChange);

void onSaiRateOfTurnChange(unsigned int newValue) 
{
  // ===== Mapping config (inline) =====
  static constexpr long MID_CODE   = 32782;
  static constexpr long HALF_SPAN  = 32768;
  static constexpr int  MAX_PX     = 75;
  auto clampInt = [](int v, int lo, int hi){ return (v < lo) ? lo : (v > hi) ? hi : v; };

  long delta = (long)newValue - MID_CODE;
  long x = (HALF_SPAN != 0) ? (delta * MAX_PX) / HALF_SPAN : 0;
  lv_obj_align(rateOfTurn_img, LV_ALIGN_CENTER, clampInt((int)x, -MAX_PX, MAX_PX), 215);
}
DcsBios::IntegerBuffer saiRateOfTurnBuffer(0x74ee, 0xffff, 0, onSaiRateOfTurnChange);


// Manual Pitch Adj (your “bug”) → move SARIBug up/down around center
void onSaiManPitchAdjChange(unsigned int newValue) {
  // ===== Mapping config (inline) =====

  static constexpr long MID_CODE   = 32782;
  static constexpr long HALF_SPAN  = 32768;
  static constexpr int  MAX_PX     = 35;     // ±40 px travel (reduced from 150)
  auto clampInt = [](int v, int lo, int hi){ return (v < lo) ? lo : (v > hi) ? hi : v; };

  long delta = (long)newValue - MID_CODE;
  long y = (HALF_SPAN != 0) ? (delta * MAX_PX) / HALF_SPAN : 0;
  y = -y;  // invert direction
  lv_obj_align(bug_img, LV_ALIGN_CENTER, 0, clampInt((int)y, -MAX_PX, MAX_PX));
}
DcsBios::IntegerBuffer saiManPitchAdjBuffer(0x74ea, 0xffff, 0, onSaiManPitchAdjChange);


void onSaiPitchChange(unsigned int newValue) 
{
  // DCS-BIOS 0..65535 → pitch in tenths of a degree, mapped to -90.0..+90.0
  // (If your actual range is wider/narrower, adjust PITCH_MAX_DEG above.)
  long tenths = map((int)newValue, 0, 65535, -PITCH_MAX_DEG * 10, PITCH_MAX_DEG * 10);

  // Convert to pixels and clamp
  long y_px = (tenths * PITCH_PX_PER_DEG) / 10;   // tenths° → px
  if (y_px < -PITCH_MAX_PX) y_px = -PITCH_MAX_PX;
  if (y_px >  PITCH_MAX_PX) y_px =  PITCH_MAX_PX;

  // Move the globe relative to screen center (positive = down)
  // Rotation still comes from onSaiBankChange()
  lv_obj_align(globe_img, LV_ALIGN_CENTER, 0, (int)y_px);
}
DcsBios::IntegerBuffer saiPitchBuffer(0x74e4, 0xffff, 0, onSaiPitchChange);


// ======================================================

void setup() {
  Serial.begin(115200);

  // --- I/O / buzzer off ---
  I2C_Init();
  TCA9554PWR_Init(0x00);
  Set_EXIO(EXIO_PIN8, Low);

  // --- Panel + backlight ---
  LCD_Init();
  Backlight_Init();
  Set_Backlight(70);


// LVGL init
lv_init();
lv_disp_draw_buf_init(&draw_buf, buf1, nullptr, DISP_WIDTH * BUF_LINES);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res  = DISP_WIDTH;
  disp_drv.ver_res  = DISP_HEIGHT;
  disp_drv.draw_buf = &draw_buf;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.sw_rotate    = 1;
  disp_drv.full_refresh = 0;
  disp_drv.rotated      = LV_DISP_ROT_180;
  lv_disp_drv_register(&disp_drv);

  // --- Screen ---
  lv_obj_t *scr = lv_obj_create(nullptr);
  lv_disp_load_scr(scr);

  // --- Set background to black ---
  lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);


  // --- Globe (rotates with bank) ---
  globe_img = lv_img_create(scr);
  lv_img_set_src(globe_img, &SARIGlobe);
  lv_obj_center(globe_img);
  lv_img_set_pivot(globe_img, SARIGlobe.header.w / 2, SARIGlobe.header.h / 2); // center pivot
  lv_obj_move_background(globe_img); // keep it behind foreground elements (optional)

  
  // --- Background (ball art behind everything, if desired) ---
  lv_obj_t *sari_bg = lv_img_create(scr);
  lv_img_set_src(sari_bg, &SARIBackground);
  lv_obj_center(sari_bg);

  // --- SARI Bug (moves up/down) ---
  bug_img = lv_img_create(scr);
  lv_img_set_src(bug_img, &SARIBug);
  lv_obj_center(bug_img);

  // --- Slip Ball (moves left/right at y=160) ---
  slipBall_img = lv_img_create(scr);
  lv_img_set_src(slipBall_img, &SlipBall);
  lv_obj_align(slipBall_img, LV_ALIGN_CENTER, 0, 160);

  // --- Rate of Turn (moves left/right at y=215) ---
  rateOfTurn_img = lv_img_create(scr);
  lv_img_set_src(rateOfTurn_img, &RateOfTurn);
  lv_obj_align(rateOfTurn_img, LV_ALIGN_CENTER, 0, 215);


  // --- Bank image (rotates around its own center) ---
  bank_img = lv_img_create(scr);
  lv_img_set_src(bank_img, &Bank);
  lv_obj_center(bank_img);   // Center on the screen
  lv_img_set_pivot(bank_img, Bank.header.w /2, Bank.header.h / 2);  // center pivot



  // IMPORTANT: start DCS-BIOS *after* objects exist so early callbacks are safe
  DcsBios::setup();

  // Initialize to center code so nothing jumps
  const unsigned int INIT = 32782U;
  onSaiSlipBallChange(INIT);
  //onSaiBankChange(INIT);
  onSaiManPitchAdjChange(INIT);
}

void loop() {
  // LVGL tick
  lv_tick_inc(1);
  lv_timer_handler();
  delay(1);

  // DCS-BIOS pump
  DcsBios::loop();
}
