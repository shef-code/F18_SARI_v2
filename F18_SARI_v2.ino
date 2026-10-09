// F18_SARI_v3 — FINAL, 100% COMPILING VERSION (CLEANED / OPTIMIZED)
// ESP32-S3 + ST7701 + LVGL8 + DCS-BIOS (Arduino-ESP32 2.x/3.x)

#define DCSBIOS_DEFAULT_SERIAL
#define DCSBIOS_DISABLE_SERVO

// Bench demo: drive the gauge with synthetic data instead of DCS-BIOS (for testing and FPS measurement).
//   0 = off (normal operation, reads DCS-BIOS)
//   1 = pitch sweep + continuous roll
//   2 = pitch sweep only
//   3 = continuous roll only
// Slip ball, rate of turn, pitch-adjust bug and the OFF flag animate in every demo mode.
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
  pitch_x10 = mapInt((uint16_t)v, 0, 65535, -900, 900);
  update_globe();
}
DcsBios::IntegerBuffer saiPitchBuffer(0x74e4, 0xffff, 0, onSaiPitchChange);

void onSaiBankChange(unsigned int v) {
  bank_x10 = mapInt((uint16_t)v, 0, 65535, 0, 3600);
  const int16_t angle = (int16_t)(bank_x10 % 3600); // keep your original behavior
  lv_img_set_angle(bank_img,  angle);
  if (globe_img) lv_img_set_angle(globe_img, angle);
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

  if (bug_img) lv_obj_align(bug_img, LV_ALIGN_CENTER, 0, y);
  else globe_renderer::set_layer_pos(layer_bug, (DISP_W - SARIBug.header.w) / 2 - globe_renderer::win_x,
                                     (DISP_H - SARIBug.header.h) / 2 + y - globe_renderer::win_y);
}
DcsBios::IntegerBuffer saiManPitchAdjBuffer(0x74ea, 0xffff, 0, onSaiManPitchAdjChange);

// DCS already sends the flag position as a smooth sweep, so it is applied as it arrives.
void onSaiAttWarningFlagChange(unsigned int v) {
  lv_img_set_angle(caged_img, (int16_t)mapInt((uint16_t)v, 0, 65535, -900, 0));
}
DcsBios::IntegerBuffer saiAttWarningFlagBuffer(0x74e8, 0xffff, 0, onSaiAttWarningFlagChange);

// ILS needles (drawn by the globe renderer, so they are not shown with the LVGL globe)
static int needle_ver = -1, needle_hor = -1;
static inline int ilsOffsetPx(unsigned int v) {
  return ((int)v - (int)DCS_HALF) * ILS_TRAVEL_PX / (int)DCS_HALF;
}
void onSaiPointerVerChange(unsigned int v) {
  globe_renderer::set_layer_pos(needle_ver,
                                (globe_renderer::WIN_W - VerticalPointer.header.w) / 2 + ILS_VER_DIR * ilsOffsetPx(v),
                                (globe_renderer::WIN_H - VerticalPointer.header.h) / 2);
}
DcsBios::IntegerBuffer saiPointerVerBuffer(0x756a, 0xffff, 0, onSaiPointerVerChange);

void onSaiPointerHorChange(unsigned int v) {
  globe_renderer::set_layer_pos(needle_hor,
                                (globe_renderer::WIN_W - HorizontalPointer.header.w) / 2,
                                (globe_renderer::WIN_H - HorizontalPointer.header.h) / 2 + ILS_HOR_DIR * ilsOffsetPx(v));
}
DcsBios::IntegerBuffer saiPointerHorBuffer(0x756c, 0xffff, 0, onSaiPointerHorChange);

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
  for (int y = area->y1; y <= area->y2; y++) {
    memcpy(dst, buf, w * sizeof(lv_color_t));
    dst += DISP_W;
    buf += w;
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

  // Force neutral position (keep)
  onSaiPitchChange(DCS_MID);
  onSaiBankChange(DCS_MID);
  onSaiSlipBallChange(DCS_MID);
  onSaiRateOfTurnChange(DCS_MID);
  onSaiManPitchAdjChange(DCS_MID);
  onSaiAttWarningFlagChange(65535);
  onSaiPointerVerChange(65535);   // out of view until DCS says otherwise
  onSaiPointerHorChange(65535);
}

#if SARI_DEMO_MODE
// Feeds the callbacks at ~30 Hz, the rate DCS-BIOS sends, so the load matches real use.
static void demo_step(uint32_t ms) {
  static uint32_t last_demo_ms = 0;
  if (ms - last_demo_ms < 33) return;
  last_demo_ms = ms;

  const float t = ms / 1000.0f;

#if SARI_DEMO_MODE != 3
  // pitch: full sweep in about 25 s
  onSaiPitchChange((unsigned int)(32768.0f + 32000.0f * sinf(t * 0.25f)));
#endif
#if SARI_DEMO_MODE != 2
  // bank: one full revolution every 12 s
  onSaiBankChange((unsigned int)(fmodf(t / 12.0f, 1.0f) * 65535.0f));
#endif
  onSaiSlipBallChange((unsigned int)(32768.0f + 32000.0f * sinf(t * 0.7f)));
  onSaiRateOfTurnChange((unsigned int)(32768.0f + 32000.0f * sinf(t * 0.5f)));
  onSaiManPitchAdjChange((unsigned int)(32768.0f + 32000.0f * sinf(t * 0.3f)));
  // OFF flag: swings across its full travel once every 8 s
  onSaiAttWarningFlagChange((unsigned int)(32768.0f + 32767.0f * sinf(t * 0.785f)));
  // ILS needles: each slides across its full travel, out of view at both ends
  onSaiPointerVerChange((unsigned int)(32768.0f + 32767.0f * sinf(t * 0.6f)));
  onSaiPointerHorChange((unsigned int)(32768.0f + 32767.0f * sinf(t * 0.45f)));
}
#endif

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
#if SARI_DEMO_MODE
  demo_step(millis());
#endif

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
#endif
}
