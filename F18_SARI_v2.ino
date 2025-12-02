// F18_SARI_v2 — Simplified (fixed viewport, roll-compensated pitch, 3 px/°)
// ESP32-S3 + ST7701 480x480 + LVGL8 + DCS-BIOS
// Globe art: 300x900 (vertical −90..+90°)

#define DCSBIOS_DEFAULT_SERIAL
#define DCSBIOS_DISABLE_SERVO

#include <DcsBios.h>
#include <Arduino.h>
#include <lvgl.h>
#include <math.h>

#include "I2C_Driver.h"
#include "TCA9554PWR.h"
#include "Display_ST7701.h"

extern "C" {
  extern const lv_img_dsc_t SARIBackground;
  extern const lv_img_dsc_t SARIBug;
  extern const lv_img_dsc_t RateOfTurn;
  extern const lv_img_dsc_t SlipBall;
  extern const lv_img_dsc_t Bank;
  extern const lv_img_dsc_t SARIGlobe;
  extern const lv_img_dsc_t VerticalPointer;
  extern const lv_img_dsc_t HorizontalPointer;
  extern const lv_img_dsc_t SARICaged;
      
}

// ----------------- Config -----------------
#define DISP_WIDTH   480
#define DISP_HEIGHT  480

static constexpr int  BUG_Y_OFFSET   = -18;   // -18
static constexpr int  GLOBE_Y_OFFSET = -33;   // -33

static constexpr int  HP_X_OFFSET = -50;      // parked = -50 x -203
static constexpr int  HP_Y_OFFSET = -50;     //  test = -50 x -50

static constexpr int  VP_X_OFFSET = -27;      // parked = -27 x -69 
static constexpr int  VP_Y_OFFSET = -80;      // test = -27 x -240

static constexpr int  SARICAGED_X_OFFSET = 157;  // caged = 155 x -95
static constexpr int  SARICAGED_Y_OFFSET = -96;  // uncaged =

static constexpr int  PITCH_MAX_DEG  = 90;
static constexpr int  PITCH_DIR      = +1;     // +1 = globe moves down on pitch up

// 300x900 art => 900/180 = 5 px/°. We want half of 6 → 3 px/°  ==> scale = 0.6
static constexpr float PITCH_SCALE   = 0.6f;   // 3 px/°
static int             PITCH_PX_PER_DEG = 3;   // set in setup from art * scale
static int             PITCH_MAX_PX      = PITCH_MAX_DEG * 3;

static constexpr int   SLIP_MAX_X_PX  = 90;
static constexpr int   SLIP_BASE_Y    = 152;
static constexpr float SLIP_MAX_Y_UP  = 7.0f;
static constexpr int   ROT_MAX_X_PX   = 75;
static constexpr int   ROT_BASE_Y     = 200;

static constexpr long  DCS_MID_CODE   = 32782;
static constexpr long  DCS_HALF_SPAN  = 32768;

// Pivot fine-tune relative to image center (adjust if your disc isn’t perfectly centered in the PNG)
static constexpr int   GLOBE_CAL_X    = 0;
static constexpr int   GLOBE_CAL_Y    = 0;

// ----------------- LVGL state -----------------
static lv_disp_draw_buf_t draw_buf;
#define BUF_LINES 64
static lv_color_t buf1[DISP_WIDTH * BUF_LINES];

static lv_obj_t *scr          = nullptr;
static lv_obj_t *bug_img      = nullptr;
static lv_obj_t *slipBall_img = nullptr;
static lv_obj_t *rateOfTurn_img = nullptr;
static lv_obj_t *bank_img     = nullptr;
static lv_obj_t *globe_holder = nullptr;  // fixed viewport
static lv_obj_t *globe_img    = nullptr;  // rotated & translated child
static lv_obj_t *verticalPointer_img    = nullptr;
static lv_obj_t *horizontalPointer_img    = nullptr;
static lv_obj_t *sariCaged_img    = nullptr;

// Geometry/cache
static int globe_w = 0, globe_h = 0;
static int globe_center_x = 0, globe_base_y = 0;

// Attitude (tenths of degrees)
static int current_pitch_x10 = 0;
static int current_bank_x10  = 0;

// ----------------- Utils -----------------
static inline int clampInt(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

static void my_disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
  LCD_addWindow((uint16_t)area->x1, (uint16_t)area->y1,
                (uint16_t)area->x2, (uint16_t)area->y2, (uint8_t*)color_p);
  lv_disp_flush_ready(drv);
}

// ----------------- Core: roll-compensated pitch -----------------
static void update_globe_position() {
  const int pitch_pixels = clampInt(PITCH_DIR * (current_pitch_x10 * PITCH_PX_PER_DEG) / 10,
                                    -PITCH_MAX_PX, PITCH_MAX_PX);

  const int pivot_x = (globe_w / 2) + GLOBE_CAL_X;
  const int pivot_y = (globe_h / 2) + GLOBE_CAL_Y;
  const int base_img_x = (globe_w / 2) - pivot_x;   // pivot at holder center
  const int base_img_y = (globe_h / 2) - pivot_y;

  // Translate along bitmap’s local vertical axis to keep motion “up” relative to the rolled globe
  const float bank_rad = current_bank_x10 * 0.0017453292519943296f; // pi/1800
  const int dx = (int)lrintf(-sinf(bank_rad) * pitch_pixels);
  const int dy = (int)lrintf( cosf(bank_rad) * pitch_pixels);

  lv_obj_set_pos(globe_img, base_img_x + dx, base_img_y + dy);
}

// ----------------- DCS-BIOS callbacks -----------------
void onSaiSlipBallChange(unsigned int newValue) {
  long delta = (long)newValue - DCS_MID_CODE;
  long x = (DCS_HALF_SPAN != 0) ? (delta * SLIP_MAX_X_PX) / DCS_HALF_SPAN : 0;
  x = clampInt((int)x, -SLIP_MAX_X_PX, SLIP_MAX_X_PX);
  float ratio = (float)abs((int)x) / (float)SLIP_MAX_X_PX;
  int y = SLIP_BASE_Y - (int)(ratio * SLIP_MAX_Y_UP);
  lv_obj_align(slipBall_img, LV_ALIGN_CENTER, (int)x, y);
}
DcsBios::IntegerBuffer saiSlipBallBuffer(0x74ec, 0xffff, 0, onSaiSlipBallChange);

void onSaiBankChange(unsigned int newValue) {
  int angle_tenths = map((int)newValue, 0, 65535, 0, 3600);
  angle_tenths = (angle_tenths + 1800) % 3600;  // panel orientation
  current_bank_x10 = angle_tenths;

  lv_img_set_angle(bank_img, angle_tenths);
  lv_img_set_angle(globe_img, angle_tenths);
  update_globe_position();
}
DcsBios::IntegerBuffer saiBankBuffer(0x74e6, 0xffff, 0, onSaiBankChange);

void onSaiRateOfTurnChange(unsigned int newValue) {
  long delta = (long)newValue - DCS_MID_CODE;
  long x = (DCS_HALF_SPAN != 0) ? (delta * ROT_MAX_X_PX) / DCS_HALF_SPAN : 0;
  x = clampInt((int)x, -ROT_MAX_X_PX, ROT_MAX_X_PX);
  lv_obj_align(rateOfTurn_img, LV_ALIGN_CENTER, (int)x, ROT_BASE_Y);
}
DcsBios::IntegerBuffer saiRateOfTurnBuffer(0x74ee, 0xffff, 0, onSaiRateOfTurnChange);

void onSaiManPitchAdjChange(unsigned int newValue) {
  static constexpr int MAX_PX = 35;
  long delta = (long)newValue - DCS_MID_CODE;
  long y = (DCS_HALF_SPAN != 0) ? (delta * MAX_PX) / DCS_HALF_SPAN : 0;
  y = -y;
  int y_out = clampInt((int)y, -MAX_PX, MAX_PX) + BUG_Y_OFFSET;
  lv_obj_align(bug_img, LV_ALIGN_CENTER, 0, y_out);
}
DcsBios::IntegerBuffer saiManPitchAdjBuffer(0x74ea, 0xffff, 0, onSaiManPitchAdjChange);

void onSaiPitchChange(unsigned int newValue) {
  long tenths = map((int)newValue, 0, 65535, -PITCH_MAX_DEG * 10, PITCH_MAX_DEG * 10);
  current_pitch_x10 = (int)tenths;
  update_globe_position();
}
DcsBios::IntegerBuffer saiPitchBuffer(0x74e4, 0xffff, 0, onSaiPitchChange);


void onSaiAttWarningFlagChange(unsigned int newValue) 
{

  static int16_t last_angle = 0;


  // Map 0..65535 → 0..-900 (tenths of degrees)
  int32_t target = -( (int32_t)newValue * 900 ) / 65535;

  // Skip if change is tiny
  if (abs(target - last_angle) < 5) return;

  // Cancel any in-flight animation
  lv_anim_del(sariCaged_img, (lv_anim_exec_xcb_t)lv_img_set_angle);

  // Animate 0→-900 smoothly
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, sariCaged_img);
  lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_img_set_angle);
  lv_anim_set_values(&a, last_angle, target);
  lv_anim_set_time(&a, 250);                      // ms duration
  lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
  lv_anim_start(&a);

  last_angle = (int16_t)target;
}
DcsBios::IntegerBuffer saiAttWarningFlagBuffer(0x74e8, 0xffff, 0, onSaiAttWarningFlagChange);


// ----------------- Setup / Loop -----------------
void setup() {
  Serial.begin(115200);

  I2C_Init();
  TCA9554PWR_Init(0x00);
  Set_EXIO(EXIO_PIN8, Low);   // silence buzzer

  LCD_Init();
  Backlight_Init();
  Set_Backlight(70);

  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf1, nullptr, DISP_WIDTH * BUF_LINES);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res      = DISP_WIDTH;
  disp_drv.ver_res      = DISP_HEIGHT;
  disp_drv.draw_buf     = &draw_buf;
  disp_drv.flush_cb     = my_disp_flush;
  disp_drv.sw_rotate    = 1;
  disp_drv.full_refresh = 0;
  disp_drv.rotated      = LV_DISP_ROT_180;   // panel upside down
  lv_disp_drv_register(&disp_drv);

  scr = lv_obj_create(nullptr);
  lv_disp_load_scr(scr);
  lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

  globe_w = SARIGlobe.header.w;   // 300
  globe_h = SARIGlobe.header.h;   // 900

  PITCH_PX_PER_DEG = (int)lrintf((globe_h / 180.0f) * PITCH_SCALE); // 900/180 * 0.6 = 3
  PITCH_MAX_PX     = PITCH_MAX_DEG * PITCH_PX_PER_DEG;              // 270

  globe_holder = lv_obj_create(scr);
  lv_obj_remove_style_all(globe_holder);
  lv_obj_set_size(globe_holder, globe_w, globe_h);
  globe_center_x = (DISP_WIDTH  - globe_w) / 2;
  globe_base_y   = (DISP_HEIGHT - globe_h) / 2 + GLOBE_Y_OFFSET;
  lv_obj_set_pos(globe_holder, globe_center_x, globe_base_y);

  globe_img = lv_img_create(globe_holder);
  lv_img_set_src(globe_img, &SARIGlobe);
  const int pivot_x = (globe_w / 2) + GLOBE_CAL_X;
  const int pivot_y = (globe_h / 2) + GLOBE_CAL_Y;
  lv_img_set_pivot(globe_img, pivot_x, pivot_y);
  lv_obj_set_pos(globe_img, (globe_w / 2) - pivot_x, (globe_h / 2) - pivot_y);

  lv_obj_t *sari_bg = lv_img_create(scr);
  lv_img_set_src(sari_bg, &SARIBackground);
  lv_obj_center(sari_bg);

  bug_img = lv_img_create(scr);
  lv_img_set_src(bug_img, &SARIBug);
  lv_obj_align(bug_img, LV_ALIGN_CENTER, 0, BUG_Y_OFFSET);

  slipBall_img = lv_img_create(scr);
  lv_img_set_src(slipBall_img, &SlipBall);
  lv_obj_align(slipBall_img, LV_ALIGN_CENTER, 0, SLIP_BASE_Y);

  rateOfTurn_img = lv_img_create(scr);
  lv_img_set_src(rateOfTurn_img, &RateOfTurn);
  lv_obj_align(rateOfTurn_img, LV_ALIGN_CENTER, 0, ROT_BASE_Y);

  bank_img = lv_img_create(scr);
  lv_img_set_src(bank_img, &Bank);
  lv_obj_center(bank_img);
  lv_img_set_pivot(bank_img, Bank.header.w / 2, Bank.header.h / 2);

  verticalPointer_img = lv_img_create(scr);
  lv_img_set_src(verticalPointer_img, &VerticalPointer);
  lv_obj_align(verticalPointer_img, LV_ALIGN_CENTER, VP_X_OFFSET, VP_Y_OFFSET);

  horizontalPointer_img = lv_img_create(scr);
  lv_img_set_src(horizontalPointer_img, &HorizontalPointer);
  lv_obj_align(horizontalPointer_img, LV_ALIGN_CENTER, HP_X_OFFSET, HP_Y_OFFSET);

  sariCaged_img = lv_img_create(scr);
  lv_img_set_src(sariCaged_img, &SARICaged);
  lv_obj_align(sariCaged_img, LV_ALIGN_CENTER, SARICAGED_X_OFFSET, SARICAGED_Y_OFFSET);
  // Set pivot to the top-right corner
  lv_img_set_pivot(sariCaged_img, SARICaged.header.w, 0);

  DcsBios::setup();

  const unsigned int INIT = DCS_MID_CODE;
  onSaiSlipBallChange(INIT);
  onSaiRateOfTurnChange(INIT);
  onSaiManPitchAdjChange(INIT);
  onSaiPitchChange(INIT);
  onSaiBankChange(INIT);


}

void loop() {
  lv_tick_inc(5);
  lv_timer_handler();
  delay(5);
  DcsBios::loop();


}
