// F18_SARI_v3 — FINAL, 100% COMPILING VERSION (CLEANED / OPTIMIZED)
// ESP32-S3 + ST7701 + LVGL8 + DCS-BIOS (Arduino-ESP32 2.x/3.x)

#define DCSBIOS_DEFAULT_SERIAL
#define DCSBIOS_DISABLE_SERVO

// While DCS is not connected the gauge shows a normal level display (the last DCS values, once it has had any), and
// the physical controls work on it: the adjust knob moves the aircraft symbol, the cage switch puts the OFF flag in
// view while it is pulled, the test button centres the ILS needles while it is held.
// Demo mode: the gauge drives itself with synthetic data.
//   0 = normal operation: reads DCS-BIOS, and starts the demo (variant 1) once no DCS-BIOS data has arrived for
//       DEMO_AFTER_MS (5 minutes, counted from power-up), leaving it again as soon as DCS connects
// The other values force the demo on for testing and FPS measurement, and never read DCS-BIOS:
//   1 = pitch sweep + continuous roll
//   2 = pitch sweep only
//   3 = continuous roll only
// Slip ball, rate of turn, OFF flag and ILS needles animate in every demo mode.
// For FPS numbers also set SARI_PERF_MONITOR to 1 in lv_conf.h.
#define SARI_DEMO_MODE 0

// Globe drawing: 1 = custom renderer (GlobeRenderer.h), 0 = LVGL rotated image with the bezel blended over it.
#define SARI_GLOBE_RENDERER 1

#include <Arduino.h>
#include <DcsBios.h>
#include <lvgl.h>
#include <math.h>
#include "I2C_Driver.h"
#include "TCA9554PWR.h"
#include "Display_ST7701.h"
#include "GlobeRenderer.h"
#include "Controls.h"

extern "C" {
  extern const lv_img_dsc_t SARIBackground, SARIBug, RateOfTurn, SlipBall;
  extern const lv_img_dsc_t Bank, SARIGlobe, SARICaged;
  extern const lv_img_dsc_t VerticalPointer, HorizontalPointer;
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

// ILS needles. A needle is centred at the middle of the DCS range and slides ILS_TRAVEL_PX each way; at either end
// of the range it is out of the window, behind the bezel. Positive direction is +x / +y on the LVGL screen, which
// is mounted upside down. Travel and directions are first guesses, to be set by comparing with DCS (task F1).
constexpr int  ILS_TRAVEL_PX      = 150;
constexpr int  ILS_VER_DIR        = -1;   // vertical needle (moves sideways)
constexpr int  ILS_HOR_DIR        = +1;    // horizontal needle (moves up and down)
constexpr uint32_t ILS_COLOUR     = 0x9A682C;   // horizontal needle; the vertical needle image carries this colour

constexpr uint16_t DCS_MID  = 32782;
constexpr uint16_t DCS_HALF = 32768;
constexpr uint32_t DCS_LIVE_MS = 3000;       // DCS-BIOS bytes within this time = DCS is connected
constexpr uint32_t DEMO_AFTER_MS = 5UL * 60UL * 1000UL;   // no DCS-BIOS bytes for this long = run the demo

// ────────────────────────────── LVGL Objects ──────────────────────────────
static lv_obj_t *globe_img = nullptr;   // only when the LVGL globe is in use
static bool use_globe_renderer = false;
static int layer_bug = -1;              // the aircraft symbol as a globe renderer layer, under the ILS needles
static lv_obj_t *bank_img  = nullptr;
static lv_obj_t *bug_img   = nullptr;   // only when the aircraft symbol is not drawn by the globe renderer
static lv_obj_t *slip_img  = nullptr;
static lv_obj_t *rot_img   = nullptr;
static lv_obj_t *caged_img = nullptr;

// ────────────────────────────── State ──────────────────────────────
static int16_t pitch_x10 = 0;
static int16_t bank_x10  = 0;

// DCS is connected / the gauge is running its demo
static bool dcs_live = false;
static bool demo_active = (SARI_DEMO_MODE != 0);
// True while the sketch itself is driving the callbacks (controls, demo), so they are not taken for DCS data
static bool in_local = false;

// The last value DCS sent for each input, so the gauge can go back to it when DCS connects. Callbacks run for local
// values too, so those are not recorded.
enum LiveIdx { L_PITCH, L_BANK, L_SLIP, L_RATE, L_ADJ, L_FLAG, L_VER, L_HOR, L_COUNT };
static uint16_t live_val[L_COUNT];
static inline void note(LiveIdx i, unsigned int v) { if (!in_local) live_val[i] = (uint16_t)v; }

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
  const float pitch_px = (pitch_x10 / 10.0f) * (float)PITCH_DIR * (float)pitch_px_per_deg;
  const float bank_rad = (float)bank_x10 * 0.0017453292519943296f; // deg*10 -> radians

  if (use_globe_renderer) {
    globe_renderer::set_attitude(pitch_px, bank_rad);
    return;
  }
  if (!globe_img) return;

  const float s = sinf(bank_rad);
  const float c = cosf(bank_rad);

  const int dx = lrintf(-s * pitch_px);
  const int dy = lrintf( c * pitch_px);

  lv_obj_set_pos(globe_img, base_globe_x + dx, base_globe_y + dy);
}

// ────────────────────────────── DCS-BIOS CALLBACKS (unsigned int!) ──────────────────────────────
void onSaiPitchChange(unsigned int v) {
  note(L_PITCH, v);
  pitch_x10 = mapInt((uint16_t)v, 0, 65535, -900, 900);
  update_globe();
}
DcsBios::IntegerBuffer saiPitchBuffer(0x74e4, 0xffff, 0, onSaiPitchChange);

void onSaiBankChange(unsigned int v) {
  note(L_BANK, v);
  bank_x10 = mapInt((uint16_t)v, 0, 65535, 0, 3600);
  const int16_t angle = (int16_t)(bank_x10 % 3600); // keep your original behavior
  lv_img_set_angle(bank_img,  angle);
  if (globe_img) lv_img_set_angle(globe_img, angle);
  update_globe();
}
DcsBios::IntegerBuffer saiBankBuffer(0x74e6, 0xffff, 0, onSaiBankChange);

void onSaiSlipBallChange(unsigned int v) {
  note(L_SLIP, v);
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
  note(L_RATE, v);
  int delta = (int)v - (int)DCS_MID;
  int x = (DCS_MID != 0) ? (delta * ROT_MAX_X_PX / (int)DCS_MID) : 0;

  // reverse left/right (keep)
  x = -x;

  x = constrain(x, -ROT_MAX_X_PX, ROT_MAX_X_PX);
  lv_obj_align(rot_img, LV_ALIGN_CENTER, x, ROT_BASE_Y);
}
DcsBios::IntegerBuffer saiRateOfTurnBuffer(0x74ee, 0xffff, 0, onSaiRateOfTurnChange);

void onSaiManPitchAdjChange(unsigned int v) {
  note(L_ADJ, v);
  int delta = (int)v - (int)DCS_MID;

  // Map DCS full deflection to ±70 px
  int y = (DCS_HALF != 0) ? (delta * BUG_MAX_TRAVEL_PX / (int)DCS_HALF) : 0;

  // Clamp and apply your neutral offset
  y = constrain(y, -BUG_MAX_TRAVEL_PX, BUG_MAX_TRAVEL_PX) + BUG_Y_OFFSET;

  if (bug_img) lv_obj_align(bug_img, LV_ALIGN_CENTER, 0, y);
  else globe_renderer::set_layer_pos(layer_bug, (DISP_W - SARIBug.header.w) / 2 - globe_renderer::win_x,
                                     (DISP_H - SARIBug.header.h) / 2 + y - globe_renderer::win_y);
}
DcsBios::IntegerBuffer saiManPitchAdjBuffer(0x74ea, 0xffff, 0, onSaiManPitchAdjChange);

// DCS already sends the flag position as a smooth sweep, so it is applied as it arrives.
void onSaiAttWarningFlagChange(unsigned int v) {
  note(L_FLAG, v);
  lv_img_set_angle(caged_img, (int16_t)mapInt((uint16_t)v, 0, 65535, -900, 0));
}
DcsBios::IntegerBuffer saiAttWarningFlagBuffer(0x74e8, 0xffff, 0, onSaiAttWarningFlagChange);

// ILS needles (drawn by the globe renderer, so they are not shown with the LVGL globe)
static int needle_ver = -1, needle_hor = -1;
static inline int ilsOffsetPx(unsigned int v) {
  return ((int)v - (int)DCS_HALF) * ILS_TRAVEL_PX / (int)DCS_HALF;
}
void onSaiPointerVerChange(unsigned int v) {
  note(L_VER, v);
  globe_renderer::set_layer_pos(needle_ver,
                                (globe_renderer::WIN_W - VerticalPointer.header.w) / 2 + ILS_VER_DIR * ilsOffsetPx(v),
                                (globe_renderer::WIN_H - VerticalPointer.header.h) / 2);
}
DcsBios::IntegerBuffer saiPointerVerBuffer(0x756a, 0xffff, 0, onSaiPointerVerChange);

void onSaiPointerHorChange(unsigned int v) {
  note(L_HOR, v);
  globe_renderer::set_layer_pos(needle_hor,
                                (globe_renderer::WIN_W - HorizontalPointer.header.w) / 2,
                                (globe_renderer::WIN_H - HorizontalPointer.header.h) / 2 + ILS_HOR_DIR * ilsOffsetPx(v));
}
DcsBios::IntegerBuffer saiPointerHorBuffer(0x756c, 0xffff, 0, onSaiPointerHorChange);

// Night and NVG lighting. In NITE and NVG the display is drawn in green (my_disp_flush) and the INST PNL knob sets
// its brightness through the backlight. NVG is the same green at NVG_BRIGHTNESS of NITE, so it is not too bright
// through goggles. From what the switch reports in the aircraft its values are DAY 0, NITE 1, NVG 2; anything else
// is treated as DAY.
constexpr uint8_t DAY_BACKLIGHT       = 50;
constexpr uint8_t NIGHT_BACKLIGHT_MAX = 60;    // knob fully on
constexpr float   NIGHT_MIN_LEVEL     = 0.05f; // knob fully off, as a fraction of the above
constexpr float   NVG_BRIGHTNESS      = 0.7f;
constexpr unsigned int LIGHT_MODE_NITE = 1, LIGHT_MODE_NVG = 2;
static bool night_tint = false, nvg_mode = false;
static uint16_t inst_pnl_level = 65535;

static void apply_backlight() {
  if (!night_tint) { Set_Backlight(DAY_BACKLIGHT); return; }
  float level = NIGHT_BACKLIGHT_MAX * (NIGHT_MIN_LEVEL + (1.0f - NIGHT_MIN_LEVEL) * inst_pnl_level / 65535.0f);
  if (nvg_mode) level *= NVG_BRIGHTNESS;
  Set_Backlight((uint8_t)(level + 0.5f));
}

void onLightModeChange(unsigned int v) {
  const bool night = (v == LIGHT_MODE_NITE || v == LIGHT_MODE_NVG);
  nvg_mode = (v == LIGHT_MODE_NVG);
  apply_backlight();
  if (night == night_tint) return;
  night_tint = night;
  apply_backlight();
  lv_obj_invalidate(lv_scr_act());              // redraw everything in the new colours
}
DcsBios::IntegerBuffer lightModeBuffer(0x74c8, 0x0600, 9, onLightModeChange);

void onInstPnlDimmerChange(unsigned int v) {
  inst_pnl_level = (uint16_t)v;
  if (night_tint) apply_backlight();
}
DcsBios::IntegerBuffer instPnlDimmerBuffer(0x7546, 0xffff, 0, onInstPnlDimmerChange);

// RGB565 pixel to a green of the same brightness
static inline uint16_t green_of(uint16_t p) {
  const uint32_t lum = (((p >> 11) * 154) + (((p >> 5) & 63) * 150) + ((p & 31) * 58)) >> 8;   // 0..62
  return (uint16_t)(((lum >> 4) << 11) | (lum << 5) | (lum >> 3));
}

// ────────────────────────────── Display Flush ──────────────────────────────
#ifndef SARI_PROFILE
#define SARI_PROFILE 0   // 1 = print render/flush timing to Serial every 2 s (demo mode only: Serial is the DCS-BIOS port)
#endif
#if SARI_PROFILE
static uint32_t prof_flush_us = 0, prof_flush_px = 0, prof_flush_n = 0;
static uint32_t prof_copy_us = 0, prof_show_us = 0, prof_sync_us = 0;
#endif
// The panel has three frame buffers: the one handed to the panel last (fb_front), the one handed to it before
// that (fb_spare), which it may still be showing until its next refresh, and one to draw into (fb_back). LVGL
// renders into its own buffer in fast internal RAM, and this copies each finished piece into fb_back. After the
// last piece of a frame fb_back is handed to the panel, which switches to it at its next refresh, so nothing
// half-drawn reaches the screen, and drawing carries on in the third buffer without waiting for that switch.
// The buffer drawn into next is two frames old, so it is missing whatever the last two frames drew. Those areas
// are remembered here and refresh_display() copies them across before the next frame is drawn.
static lv_color_t *fb_front = nullptr, *fb_back = nullptr, *fb_spare = nullptr;
constexpr int MAX_FRAME_AREAS = 12;
static lv_area_t frame_areas[MAX_FRAME_AREAS];   // areas drawn into fb_back during the current frame
static int frame_area_cnt = 0;
static lv_area_t stale_areas[2][MAX_FRAME_AREAS];   // areas drawn by the last frame [0] and the one before [1]
static int stale_area_cnt[2] = { 0, 0 };
static bool frame_shown = false;

static void my_disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *buf) {
#if SARI_PROFILE
  const uint32_t t0 = micros();
#endif
  const int w = lv_area_get_width(area);
  lv_color_t *dst = fb_back + area->y1 * DISP_W + area->x1;
  if (night_tint) {
    for (int y = area->y1; y <= area->y2; y++) {
      for (int x = 0; x < w; x++) dst[x].full = green_of(buf[x].full);
      dst += DISP_W;
      buf += w;
    }
  } else {
    for (int y = area->y1; y <= area->y2; y++) {
      memcpy(dst, buf, w * sizeof(lv_color_t));
      dst += DISP_W;
      buf += w;
    }
  }
#if SARI_PROFILE
  prof_copy_us += micros() - t0;
#endif
  if (frame_area_cnt < MAX_FRAME_AREAS) frame_areas[frame_area_cnt++] = *area;
  else _lv_area_join(&frame_areas[MAX_FRAME_AREAS - 1], &frame_areas[MAX_FRAME_AREAS - 1], area);

  if (lv_disp_flush_is_last(drv)) {
#if SARI_PROFILE
    const uint32_t t1 = micros();
#endif
    LCD_showFrameBuffer(fb_back);
#if SARI_PROFILE
    prof_show_us += micros() - t1;
#endif
    lv_color_t *shown = fb_back;
    fb_back = fb_spare;
    fb_spare = fb_front;
    fb_front = shown;
    frame_shown = true;
  }
#if SARI_PROFILE
  prof_flush_us += micros() - t0;
  prof_flush_px += (uint32_t)lv_area_get_width(area) * lv_area_get_height(area);
  prof_flush_n++;
#endif
  lv_disp_flush_ready(drv);
}

// Runs one LVGL screen refresh. The sketch drives this itself (LVGL's own refresh timer is removed in setup)
// so that the hidden frame buffer can be brought up to date first.
// Copies a rectangle from the shown frame buffer to the hidden one. Does nothing for an empty rectangle.
static void sync_rect(int x1, int y1, int x2, int y2) {
  if (x2 < x1 || y2 < y1) return;
  const size_t bytes = (x2 - x1 + 1) * sizeof(lv_color_t);
  for (int y = y1; y <= y2; y++) memcpy(fb_back + y * DISP_W + x1, fb_front + y * DISP_W + x1, bytes);
}

static void refresh_display() {
  lv_disp_t *disp = lv_disp_get_default();
  if (disp->inv_p == 0) return;                    // nothing changed, leave both frame buffers alone

  // Bring the hidden buffer up to date with what the last frame drew into the other one
#if SARI_PROFILE
  const uint32_t t0 = micros();
#endif
  LCD_waitForSpareFrameBuffer();                   // the panel may still be reading fb_back
  for (int i = 0; i < stale_area_cnt[0] + stale_area_cnt[1]; i++) {
    const lv_area_t *a = i < stale_area_cnt[0] ? &stale_areas[0][i] : &stale_areas[1][i - stale_area_cnt[0]];
    // Whatever this refresh is about to redraw anyway need not be copied. Find the pending redraw area that
    // overlaps this stale area most, and copy only the up to four strips of the stale area outside it.
    lv_area_t cut;
    int32_t best = 0;
    for (int j = 0; j < disp->inv_p; j++) {
      lv_area_t common;
      if (!_lv_area_intersect(&common, a, &disp->inv_areas[j])) continue;
      const int32_t size = (int32_t)lv_area_get_size(&common);
      if (size > best) { best = size; cut = common; }
    }
    if (best == 0) { sync_rect(a->x1, a->y1, a->x2, a->y2); continue; }
    sync_rect(a->x1, a->y1, a->x2, cut.y1 - 1);              // above
    sync_rect(a->x1, cut.y2 + 1, a->x2, a->y2);              // below
    sync_rect(a->x1, cut.y1, cut.x1 - 1, cut.y2);            // left
    sync_rect(cut.x2 + 1, cut.y1, a->x2, cut.y2);            // right
  }
#if SARI_PROFILE
  prof_sync_us += micros() - t0;
#endif
  frame_area_cnt = 0;
  frame_shown = false;

  _lv_disp_refr_timer(nullptr);

  if (frame_shown) {
    memcpy(stale_areas[1], stale_areas[0], sizeof(lv_area_t) * stale_area_cnt[0]);
    stale_area_cnt[1] = stale_area_cnt[0];
    memcpy(stale_areas[0], frame_areas, sizeof(lv_area_t) * frame_area_cnt);
    stale_area_cnt[0] = frame_area_cnt;
  }
}

// ────────────────────────────── Setup ──────────────────────────────
void setup() {
#if SARI_DEMO_MODE
  Serial.begin(115200);        // only for the SARI_PROFILE output; DcsBios::setup() opens the port otherwise
#endif

  I2C_Init();
  TCA9554PWR_Init(0x00);
  Set_EXIO(EXIO_PIN8, Low);    // Turns off buzzer
  LCD_Init();
  Backlight_Init();
  Set_Backlight(50);

  lv_init();

  void *fb0 = nullptr, *fb1 = nullptr, *fb2 = nullptr;
  LCD_getFrameBuffers(&fb0, &fb1, &fb2);
  fb_front = (lv_color_t *)fb0;   // the panel starts by showing its first frame buffer
  fb_back  = (lv_color_t *)fb1;
  fb_spare = (lv_color_t *)fb2;

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
  lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
  // The sketch runs the screen refresh itself, see refresh_display()
  lv_timer_del(disp->refr_timer);
  disp->refr_timer = nullptr;

  lv_obj_t *scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);   // anything the bezel leaves uncovered shows black, not LVGL's near-white

  // Globe pitch scale (keep original math)
  const int globe_w = SARIGlobe.header.w;
  const int globe_h = SARIGlobe.header.h;
  pitch_px_per_deg = lrintf(((float)globe_h / 180.0f) * PITCH_SCALE);

  constexpr int GLOBE_VP_W = 275;
  constexpr int GLOBE_VP_H = 275;

  pivot_x = globe_w / 2 + GLOBE_CAL_X;
  pivot_y = globe_h / 2 + GLOBE_CAL_Y;

#if SARI_GLOBE_RENDERER
  // One object draws the bezel and the globe. Same window rectangle as the LVGL holder below.
  use_globe_renderer = globe_renderer::create(scr, &SARIBackground, &SARIGlobe,
                                              (DISP_W - GLOBE_VP_W) / 2, (DISP_H - GLOBE_VP_H) / 2 + GLOBE_Y_OFFSET,
                                              pivot_x, pivot_y) != nullptr;
  // Layers are drawn in this order, so the needles pass over the aircraft symbol
  layer_bug  = globe_renderer::add_layer(&SARIBug, lv_color_black());
  needle_ver = globe_renderer::add_layer(&VerticalPointer, lv_color_hex(ILS_COLOUR));
  needle_hor = globe_renderer::add_layer(&HorizontalPointer, lv_color_hex(ILS_COLOUR));
#endif
  if (!use_globe_renderer) {
    lv_obj_t *holder = lv_obj_create(scr);
    lv_obj_remove_style_all(holder);
    lv_obj_set_size(holder, GLOBE_VP_W, GLOBE_VP_H);
    lv_obj_center(holder);
    lv_obj_set_y(holder, GLOBE_Y_OFFSET);
    lv_obj_set_scrollbar_mode(holder, LV_SCROLLBAR_MODE_OFF); // keep

    base_globe_x = (GLOBE_VP_W / 2) - pivot_x;
    base_globe_y = (GLOBE_VP_H / 2) - pivot_y;

    globe_img = lv_img_create(holder);
    lv_img_set_src(globe_img, &SARIGlobe);
    lv_img_set_pivot(globe_img, pivot_x, pivot_y);
    lv_obj_set_pos(globe_img, base_globe_x, base_globe_y);

    lv_obj_t *bg = lv_img_create(scr);
    lv_img_set_src(bg, &SARIBackground);
    lv_obj_center(bg);
  }

  bank_img = lv_img_create(scr);
  lv_img_set_src(bank_img, &Bank);
  // Bank.c holds only the pointer triangle (22x28), cut from the top of the original 22x312 image. It is placed
  // where the top of that image was and pivots about where its centre was, which is outside this image, so the
  // pointer follows the same arc while LVGL only redraws a small area.
  constexpr int BANK_FULL_H = 312;
  lv_obj_set_pos(bank_img, (DISP_W - Bank.header.w) / 2, (DISP_H - BANK_FULL_H) / 2 + BANK_Y_OFFSET);
  lv_img_set_pivot(bank_img, Bank.header.w / 2, BANK_FULL_H / 2);

  if (layer_bug < 0) { bug_img = lv_img_create(scr); lv_img_set_src(bug_img, &SARIBug); lv_obj_align(bug_img, LV_ALIGN_CENTER, 0, BUG_Y_OFFSET); }
  slip_img = lv_img_create(scr); lv_img_set_src(slip_img, &SlipBall);    lv_obj_align(slip_img, LV_ALIGN_CENTER, 0, SLIP_BASE_Y);
  rot_img  = lv_img_create(scr); lv_img_set_src(rot_img,  &RateOfTurn);  lv_obj_align(rot_img,  LV_ALIGN_CENTER, 0, ROT_BASE_Y);

  caged_img = lv_img_create(scr);
  lv_img_set_src(caged_img, &SARICaged);
  lv_obj_align(caged_img, LV_ALIGN_CENTER, CAGED_X_OFFSET, CAGED_Y_OFFSET);
  lv_img_set_pivot(caged_img, SARICaged.header.w, SARICaged.header.h);

#if !SARI_DEMO_MODE
  // DCS-BIOS is only read between frames, and at 250000 baud a frame's worth of data overflows the default
  // 256-byte buffer. Must be set before DcsBios::setup(), which opens the port.
  Serial.setRxBufferSize(4096);
  DcsBios::setup();
#endif
  controls::begin();

  // Force neutral position (keep)
  onSaiPitchChange(DCS_MID);
  onSaiBankChange(DCS_MID);
  onSaiSlipBallChange(DCS_MID);
  onSaiRateOfTurnChange(DCS_MID);
  onSaiManPitchAdjChange(DCS_MID);
  onSaiAttWarningFlagChange(0);      // OFF flag out of view
  onSaiPointerVerChange(0);       // out of view (on the left) until DCS says otherwise
  onSaiPointerHorChange(65535);
}

// Puts the gauge back to what DCS last sent (or the neutral display, if it never sent anything).
static void restore_live() {
  onSaiPitchChange(live_val[L_PITCH]);
  onSaiBankChange(live_val[L_BANK]);
  onSaiSlipBallChange(live_val[L_SLIP]);
  onSaiRateOfTurnChange(live_val[L_RATE]);
  onSaiManPitchAdjChange(live_val[L_ADJ]);
  onSaiAttWarningFlagChange(live_val[L_FLAG]);
  onSaiPointerVerChange(live_val[L_VER]);
  onSaiPointerHorChange(live_val[L_HOR]);
}

// Demo variant: 1 = pitch sweep + continuous roll, 2 = pitch sweep only, 3 = continuous roll only
#if SARI_DEMO_MODE
constexpr int DEMO_VARIANT = SARI_DEMO_MODE;
#else
constexpr int DEMO_VARIANT = 1;
#endif
constexpr int ADJ_STEP = 2809;              // per click of the adjust knob, as it is sent to DCS
// Turning the knob clockwise moves the symbol this way while DCS is not connected; the minus sign in local_controls() sets it

// Controls while DCS is not connected (the demo included). The adjust knob moves the aircraft symbol, the cage switch
// puts the OFF flag in view while it is pulled, and the test button centres the ILS needles while it is held. The
// flag and needles glide to their targets, and the demo only sets their targets, so they never jump.
// Everything else stays as it was shown.
static uint16_t local_adj = DCS_MID;        // aircraft symbol position
static bool local_force = true;             // apply the symbol position again
constexpr float GLIDE_PER_MS = 65535.0f / 500.0f;   // full travel of the flag or a needle in 0.5 s

struct Glide {
  float cur;
  int shown;
  void sync(uint16_t v) { cur = v; shown = v; }
  // Moves toward target; returns true when the displayed (whole) value changed.
  bool step(float target, float max_move) {
    const float d = target - cur;
    cur = (d > max_move) ? cur + max_move : (d < -max_move) ? cur - max_move : target;
    const int now = (int)(cur + 0.5f);
    if (now == shown) return false;
    shown = now;
    return true;
  }
};
static Glide g_flag = {0, 0}, g_ver = {0, 0}, g_hor = {65535, 65535};
static float demo_flag = 0, demo_ver = 0, demo_hor = 65535;   // targets the demo animates

// The glide values follow the last DCS values, so leaving DCS (or the demo) starts from what is on the screen
static void sync_glide() {
  g_flag.sync(live_val[L_FLAG]);
  g_ver.sync(live_val[L_VER]);
  g_hor.sync(live_val[L_HOR]);
}

static void local_controls(uint32_t ms) {
  static uint32_t last_ms = 0;
  uint32_t dt = ms - last_ms;
  if (dt > 50) dt = 50;                      // the first call after DCS (or a long frame) must not jump
  const float max_move = dt * GLIDE_PER_MS;
  last_ms = ms;

  const int clicks = controls::enc1.clicks.exchange(0);
  controls::enc2.clicks.store(0);            // the altimeter setting has nothing to show here
  const bool caged = controls::cage.stable.load();
  const bool testing = controls::test.stable.load();

  if (clicks || local_force) {
    local_adj = (uint16_t)constrain((int)local_adj - clicks * ADJ_STEP, 0, 65535);
    onSaiManPitchAdjChange(local_adj);
    local_force = false;
  }
  const float flag_target = caged ? 65535.0f : (demo_active ? demo_flag : (float)live_val[L_FLAG]);
  const float ver_target = testing ? (float)DCS_HALF : (demo_active ? demo_ver : (float)live_val[L_VER]);
  const float hor_target = testing ? (float)DCS_HALF : (demo_active ? demo_hor : (float)live_val[L_HOR]);
  if (g_flag.step(flag_target, max_move)) onSaiAttWarningFlagChange(g_flag.shown);
  if (g_ver.step(ver_target, max_move)) onSaiPointerVerChange(g_ver.shown);
  if (g_hor.step(hor_target, max_move)) onSaiPointerHorChange(g_hor.shown);
}

// Feeds the callbacks at ~30 Hz, the rate DCS-BIOS sends, so the load matches real use.
static void demo_step(uint32_t ms) {
  static uint32_t last_demo_ms = 0;
  if (ms - last_demo_ms < 33) return;
  last_demo_ms = ms;

  const float t = ms / 1000.0f;

  // pitch: full sweep in about 25 s
  if (DEMO_VARIANT != 3) onSaiPitchChange((unsigned int)(32768.0f + 32000.0f * sinf(t * 0.25f)));
  // bank: one full revolution every 12 s
  if (DEMO_VARIANT != 2) onSaiBankChange((unsigned int)(fmodf(t / 12.0f, 1.0f) * 65535.0f));
  onSaiSlipBallChange((unsigned int)(32768.0f + 32000.0f * sinf(t * 0.7f)));
  onSaiRateOfTurnChange((unsigned int)(32768.0f + 32000.0f * sinf(t * 0.5f)));
  // OFF flag: swings across its full travel once every 8 s
  demo_flag = 32768.0f + 32767.0f * sinf(t * 0.785f);
  // ILS needles: each slides across its full travel, out of view at both ends
  demo_ver = 32768.0f + 32767.0f * sinf(t * 0.6f);
  demo_hor = 32768.0f + 32767.0f * sinf(t * 0.45f);
}

// Runs LVGL's timers (animations) and, at LVGL's refresh period, one screen refresh.
static void run_lvgl() {
  static uint32_t last_refresh_ms = 0;
  lv_timer_handler();
  const uint32_t now = millis();
  if (now - last_refresh_ms >= LV_DISP_DEF_REFR_PERIOD) {
    last_refresh_ms = now;
    refresh_display();
  }
}

void loop() {
  // LVGL reads millis() itself (LV_TICK_CUSTOM in lv_conf.h), so its tick keeps running during a draw.
#if !SARI_DEMO_MODE
  // DCS is connected while DCS-BIOS bytes keep arriving. Until then (and for a while after they stop) the display stays
  // as it is, with the local controls on it; after DEMO_AFTER_MS without any bytes the demo starts.
  static uint32_t last_rx_ms = 0;          // counts from power-up until the first byte
  static bool have_rx = false;
  if (Serial.available()) { last_rx_ms = millis(); have_rx = true; }
  const uint32_t quiet_ms = millis() - last_rx_ms;
  const bool live_now = have_rx && quiet_ms < DCS_LIVE_MS;
  const bool demo_now = !live_now && quiet_ms >= DEMO_AFTER_MS;
  if (live_now != dcs_live) {
    dcs_live = live_now;
    controls::enc1.clicks.store(0);
    controls::enc2.clicks.store(0);
    if (dcs_live) { demo_active = false; restore_live(); }
    else { local_adj = live_val[L_ADJ]; sync_glide(); }   // carry on from where DCS left things
    local_force = true;
  }
  if (!dcs_live && demo_now != demo_active) {
    demo_active = demo_now;
    if (!demo_active) { restore_live(); sync_glide(); }
    local_force = true;
  }
#endif
  if (!dcs_live) {
    in_local = true;
    if (demo_active) demo_step(millis());
    local_controls(millis());
    in_local = false;
  }

#if SARI_PROFILE
  static uint32_t p_last = 0, p_lvgl_us = 0, p_frames = 0;
  const uint32_t h0 = micros();
  run_lvgl();
  p_lvgl_us += micros() - h0;
  if (frame_shown) { p_frames++; frame_shown = false; }
  if (millis() - p_last >= 2000) {
    Serial.printf("PROF frames=%lu, lvgl=%lu us, flush=%lu us (copy=%lu, swap=%lu), sync=%lu us, globe=%lu us, %lu flushes, %lu px\n",
                  (unsigned long)p_frames, (unsigned long)p_lvgl_us, (unsigned long)prof_flush_us,
                  (unsigned long)prof_copy_us, (unsigned long)prof_show_us, (unsigned long)prof_sync_us, (unsigned long)globe_renderer::prof_draw_us,
                  (unsigned long)prof_flush_n, (unsigned long)prof_flush_px);
    p_last = millis(); p_lvgl_us = p_frames = 0;
    prof_flush_us = prof_flush_px = prof_flush_n = prof_copy_us = prof_show_us = prof_sync_us = 0; globe_renderer::prof_draw_us = 0;
  }
#else
  run_lvgl();
#endif
#if !SARI_DEMO_MODE
  DcsBios::loop();
  if (dcs_live) controls::service();
#endif
}
