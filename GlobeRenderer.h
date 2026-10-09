// Custom renderer for the bezel and the attitude globe (task A1 in CURRENT_TASKS.md).
//
// LVGL's generic path drew the globe as a rotated image and then alpha-blended the whole bezel image over it.
// This draws both in one pass from one LVGL object that covers the screen:
//   - outside the globe window it copies the bezel, which is prepared once at boot as plain RGB565;
//   - inside the window it samples the globe texture directly, one scanline at a time, with the texture
//     coordinate stepped by a constant per pixel;
//   - only the few soft-edge pixels of the window are blended.
// The small sprites (aircraft symbol, bank pointer, slip ball, rate of turn, OFF flag) stay ordinary LVGL images
// drawn on top.
//
// The picture is meant to match the LVGL version: same window position, same pivot, same pitch scale.
// One deliberate difference: past +/-90 degrees of pitch the texture is mirrored over the pole instead of
// running off its end (task B7).
#pragma once
#include <Arduino.h>
#include <lvgl.h>
#include <math.h>

namespace globe_renderer {

constexpr int SCR_W = 480, SCR_H = 480;
constexpr int WIN_W = 275, WIN_H = 275;   // globe window rectangle (the old LVGL "holder")

static lv_obj_t *scene = nullptr;
static int win_x = 0, win_y = 0;          // top-left of the window rectangle on screen

// Bezel, prepared at boot (PSRAM)
static uint16_t *bezel_rgb = nullptr;     // SCR_W x SCR_H. Outside the window rectangle: bezel over the screen colour.
                                          // Inside it: the raw bezel colour, to be blended with the globe by bezel_a.
static uint8_t  *bezel_a   = nullptr;     // WIN_W x WIN_H bezel alpha inside the window rectangle (255 = bezel only)

// Per window row: where the bezel stops being opaque, and the run where it is fully transparent.
//   x <  open_l or x >  open_r : bezel only
//   open_l <= x < clear_l      : blended edge
//   clear_l <= x <= clear_r    : globe only
//   clear_r <  x <= open_r     : blended edge
static int16_t open_l[WIN_H], open_r[WIN_H], clear_l[WIN_H], clear_r[WIN_H];

// Globe texture
static const uint16_t *tex = nullptr;
static int tex_w = 0, tex_h = 0;
static int tex_pivot_x = 0, tex_pivot_y = 0;

// Flat-colour map. Most of the globe is plain sky or ground. Each entry covers a 4x4 block of texels and says
// whether that block, plus a one-texel border on every side (what a bilinear sample can reach), is one colour. It lives in
// internal RAM, so those pixels are drawn without touching the texture in PSRAM.
constexpr int FLAT_SHIFT = 2;
constexpr uint8_t FLAT_NONE = 255;
static uint8_t *flat_map = nullptr;       // flat_w x flat_h, index into flat_colour or FLAT_NONE
static int flat_w = 0, flat_h = 0;
static uint16_t flat_colour[FLAT_NONE];

// Attitude for the next draw
static float att_pitch_px = 0.0f;         // texture rows the globe is shifted by
static float att_bank_rad = 0.0f;

static uint32_t prof_draw_us = 0;         // time spent in draw(), for SARI_PROFILE

// Blend two RGB565 colours. w is the weight of b, 0..32.
static inline uint16_t mix565(uint32_t a, uint32_t b, uint32_t w) {
  a = (a | (a << 16)) & 0x07E0F81F;
  b = (b | (b << 16)) & 0x07E0F81F;
  const uint32_t r = (a + (((b - a) * w) >> 5)) & 0x07E0F81F;
  return (uint16_t)(r | (r >> 16));
}

// Texel fetch that is safe for any coordinate. Past the top or bottom of the texture the view has gone over
// the pole, so the texture is mirrored (and flipped left to right).
static inline uint16_t texel(int u, int v) {
  if (v < 0)           { v = -1 - v;            u = tex_w - 1 - u; }
  else if (v >= tex_h) { v = 2 * tex_h - 1 - v; u = tex_w - 1 - u; }
  if (v < 0) v = 0; else if (v >= tex_h) v = tex_h - 1;
  if (u < 0) u = 0; else if (u >= tex_w) u = tex_w - 1;
  return tex[v * tex_w + u];
}

// Bilinear sample at a 16.16 fixed-point texture coordinate, any coordinate.
static inline uint16_t sample_checked(int32_t u, int32_t v) {
  const int ui = u >> 16, vi = v >> 16;
  {
    // Flat-colour shortcut, on the mirrored and clamped position of the base texel
    int mu = ui, mv = vi;
    if (mv < 0)           { mv = -1 - mv;            mu = tex_w - 1 - mu; }
    else if (mv >= tex_h) { mv = 2 * tex_h - 1 - mv; mu = tex_w - 1 - mu; }
    if (mv < 0) mv = 0; else if (mv >= tex_h) mv = tex_h - 1;
    if (mu < 0) mu = 0; else if (mu >= tex_w) mu = tex_w - 1;
    const uint8_t k = flat_map[(mv >> FLAT_SHIFT) * flat_w + (mu >> FLAT_SHIFT)];
    if (k != FLAT_NONE) return flat_colour[k];
  }
  const uint16_t c00 = texel(ui, vi), c01 = texel(ui + 1, vi), c10 = texel(ui, vi + 1), c11 = texel(ui + 1, vi + 1);
  if (c00 == c01 && c00 == c10 && c00 == c11) return c00;
  const uint32_t fu = (u >> 11) & 31, fv = (v >> 11) & 31;
  return mix565(mix565(c00, c01, fu), mix565(c10, c11, fu), fv);
}

// Same, for coordinates known to be at least one texel inside the texture on every side.
static inline uint16_t sample_inside(int32_t u, int32_t v) {
  const int ui = u >> 16, vi = v >> 16;
  const uint8_t k = flat_map[(vi >> FLAT_SHIFT) * flat_w + (ui >> FLAT_SHIFT)];
  if (k != FLAT_NONE) return flat_colour[k];                // flat sky or ground: no texture read at all
  const uint16_t *p = tex + vi * tex_w + ui;
  const uint16_t c00 = p[0], c01 = p[1], c10 = p[tex_w], c11 = p[tex_w + 1];
  if (c00 == c01 && c00 == c10 && c00 == c11) return c00;
  const uint32_t fu = (u >> 11) & 31, fv = (v >> 11) & 31;
  return mix565(mix565(c00, c01, fu), mix565(c10, c11, fu), fv);
}

static inline bool inside_texture(int32_t u, int32_t v) {
  const int ui = u >> 16, vi = v >> 16;
  return ui >= 0 && ui < tex_w - 1 && vi >= 0 && vi < tex_h - 1;
}

static void draw(lv_event_t *e) {
  const lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target(e);

  if (code == LV_EVENT_COVER_CHECK) {
    // This object paints every pixel of its area, so LVGL need not draw anything underneath it.
    lv_cover_check_info_t *info = (lv_cover_check_info_t *)lv_event_get_param(e);
    if (info->res != LV_COVER_RES_MASKED) {
      info->res = _lv_area_is_in(info->area, &obj->coords, 0) ? LV_COVER_RES_COVER : LV_COVER_RES_NOT_COVER;
    }
    return;
  }
  if (code != LV_EVENT_DRAW_MAIN) return;

  lv_draw_ctx_t *ctx = lv_event_get_draw_ctx(e);
  lv_area_t a;
  if (!_lv_area_intersect(&a, ctx->clip_area, &obj->coords)) return;
  const uint32_t prof_t0 = micros();

  uint16_t *const buf = (uint16_t *)ctx->buf;
  const int buf_w = lv_area_get_width(ctx->buf_area);
  const int buf_x = ctx->buf_area->x1, buf_y = ctx->buf_area->y1;

  // Texture coordinate of a window pixel (lx, ly), matching what the LVGL image rotation did:
  // rotate about the window centre by the bank angle, then shift along the globe's own axis by the pitch.
  const float s = sinf(att_bank_rad), c = cosf(att_bank_rad);
  const float qx0 = -(float)(WIN_W / 2) + s * att_pitch_px;   // offset of pixel (0,0) from the shifted pivot
  const float qy0 = -(float)(WIN_H / 2) - c * att_pitch_px;
  const int32_t u00 = (int32_t)lrintf(((float)tex_pivot_x + qx0 * c + qy0 * s) * 65536.0f);
  const int32_t v00 = (int32_t)lrintf(((float)tex_pivot_y - qx0 * s + qy0 * c) * 65536.0f);
  const int32_t du_dx = (int32_t)lrintf(c * 65536.0f),  dv_dx = (int32_t)lrintf(-s * 65536.0f);
  const int32_t du_dy = (int32_t)lrintf(s * 65536.0f),  dv_dy = (int32_t)lrintf(c * 65536.0f);

  for (int y = a.y1; y <= a.y2; y++) {
    uint16_t *dst = buf + (y - buf_y) * buf_w + (a.x1 - buf_x);
    const uint16_t *bz = bezel_rgb + y * SCR_W + a.x1;
    const int ly = y - win_y;

    if (ly < 0 || ly >= WIN_H || a.x2 < win_x || a.x1 >= win_x + WIN_W) {
      memcpy(dst, bz, (a.x2 - a.x1 + 1) * sizeof(uint16_t));   // row is all bezel
      continue;
    }

    // Columns of this row in window coordinates, clipped to the redraw area
    const int lx_first = max(a.x1 - win_x, 0), lx_last = min(a.x2 - win_x, WIN_W - 1);
    if (a.x1 < win_x) {                                        // bezel left of the window rectangle
      const int n = win_x - a.x1;
      memcpy(dst, bz, n * sizeof(uint16_t));
      dst += n; bz += n;
    }

    const int ol = open_l[ly], orr = open_r[ly], cl = clear_l[ly], cr = clear_r[ly];
    const uint8_t *al = bezel_a + ly * WIN_W;
    int32_t u = u00 + ly * du_dy + lx_first * du_dx;
    int32_t v = v00 + ly * dv_dy + lx_first * dv_dx;
    int lx = lx_first;

    // 1. opaque bezel on the left
    int end = min(ol - 1, lx_last);
    if (lx <= end) {
      const int n = end - lx + 1;
      memcpy(dst, bz, n * sizeof(uint16_t));
      dst += n; bz += n; lx += n; u += n * du_dx; v += n * dv_dx;
    }
    // 2. soft edge on the left
    end = min(cl - 1, lx_last);
    for (; lx <= end; lx++, dst++, bz++, u += du_dx, v += dv_dx) {
      const uint32_t w = (al[lx] + 4) >> 3;                    // bezel weight 0..32
      *dst = (w >= 32) ? *bz : mix565(sample_checked(u, v), *bz, w);
    }
    // 3. globe only
    end = min(cr, lx_last);
    if (lx <= end) {
      const int n = end - lx + 1;
      const int32_t u_end = u + (n - 1) * du_dx, v_end = v + (n - 1) * dv_dx;
      if (inside_texture(u, v) && inside_texture(u_end, v_end)) {
        // The coordinate moves in a straight line, so if both ends are inside the texture the whole run is.
        for (int i = 0; i < n; i++, u += du_dx, v += dv_dx) dst[i] = sample_inside(u, v);
      } else {
        for (int i = 0; i < n; i++, u += du_dx, v += dv_dx) dst[i] = sample_checked(u, v);
      }
      dst += n; bz += n; lx += n;
    }
    // 4. soft edge on the right
    end = min(orr, lx_last);
    for (; lx <= end; lx++, dst++, bz++, u += du_dx, v += dv_dx) {
      const uint32_t w = (al[lx] + 4) >> 3;
      *dst = (w >= 32) ? *bz : mix565(sample_checked(u, v), *bz, w);
    }
    // 5. opaque bezel on the right, then the bezel beyond the window rectangle
    const int rest = a.x2 - (win_x + lx) + 1;
    if (rest > 0) memcpy(dst, bz, rest * sizeof(uint16_t));
  }
  prof_draw_us += micros() - prof_t0;
}

// Builds flat_map from the texture.
static bool prepare_flat_map() {
  flat_w = (tex_w + (1 << FLAT_SHIFT) - 1) >> FLAT_SHIFT;
  flat_h = (tex_h + (1 << FLAT_SHIFT) - 1) >> FLAT_SHIFT;
  flat_map = (uint8_t *)heap_caps_malloc(flat_w * flat_h, MALLOC_CAP_INTERNAL);
  if (!flat_map) flat_map = (uint8_t *)heap_caps_malloc(flat_w * flat_h, MALLOC_CAP_SPIRAM);
  if (!flat_map) return false;
  int colours = 0;
  for (int by = 0; by < flat_h; by++) {
    for (int bx = 0; bx < flat_w; bx++) {
      const int x0 = bx << FLAT_SHIFT, y0 = by << FLAT_SHIFT;
      // one texel of border on every side, so the neighbours a bilinear sample reads are covered in any direction
      const int xa = max(x0 - 1, 0), ya = max(y0 - 1, 0);
      const int x1 = min(x0 + (1 << FLAT_SHIFT), tex_w - 1), y1 = min(y0 + (1 << FLAT_SHIFT), tex_h - 1);
      const uint16_t c = tex[y0 * tex_w + x0];
      bool flat = true;
      for (int y = ya; y <= y1 && flat; y++)
        for (int x = xa; x <= x1; x++)
          if (tex[y * tex_w + x] != c) { flat = false; break; }
      uint8_t k = FLAT_NONE;
      if (flat) {
        int i = 0;
        while (i < colours && flat_colour[i] != c) i++;
        if (i == colours && colours < FLAT_NONE) flat_colour[colours++] = c;
        if (i < colours) k = (uint8_t)i;
      }
      flat_map[by * flat_w + bx] = k;
    }
  }
  return true;
}

// Builds the bezel tables from the INDEXED_8BIT bezel image (256-entry B,G,R,A palette, then one byte per pixel).
static bool prepare_bezel(const lv_img_dsc_t *bezel, lv_color_t screen_colour) {
  if (bezel->header.cf != LV_IMG_CF_INDEXED_8BIT || bezel->header.w != SCR_W || bezel->header.h != SCR_H) return false;
  bezel_rgb = (uint16_t *)heap_caps_malloc(SCR_W * SCR_H * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
  bezel_a   = (uint8_t *)heap_caps_malloc(WIN_W * WIN_H, MALLOC_CAP_SPIRAM);
  if (!bezel_rgb || !bezel_a) return false;

  const uint8_t *pal = bezel->data;
  const uint8_t *idx = bezel->data + 256 * 4;
  const uint16_t scr565 = lv_color_to16(screen_colour);

  for (int y = 0; y < SCR_H; y++) {
    const int ly = y - win_y;
    const bool win_row = ly >= 0 && ly < WIN_H;
    if (win_row) { open_l[ly] = WIN_W; open_r[ly] = -1; }
    for (int x = 0; x < SCR_W; x++) {
      const uint8_t *p = pal + idx[y * SCR_W + x] * 4;
      const uint16_t c = (uint16_t)(((p[2] >> 3) << 11) | ((p[1] >> 2) << 5) | (p[0] >> 3));
      const uint8_t alpha = p[3];
      const int lx = x - win_x;
      if (win_row && lx >= 0 && lx < WIN_W) {
        bezel_rgb[y * SCR_W + x] = c;
        bezel_a[ly * WIN_W + lx] = alpha;
        if (alpha < 255) { if (lx < open_l[ly]) open_l[ly] = lx; open_r[ly] = lx; }
      } else {
        bezel_rgb[y * SCR_W + x] = mix565(scr565, c, (alpha + 4) >> 3);
      }
    }
    if (!win_row) continue;

    // Longest fully transparent run in this row. Everything else between open_l and open_r is treated as edge.
    int best_l = 0, best_r = -1, run_l = -1;
    for (int lx = 0; lx <= WIN_W; lx++) {
      const bool clear = lx < WIN_W && bezel_a[ly * WIN_W + lx] == 0;
      if (clear && run_l < 0) run_l = lx;
      if (!clear && run_l >= 0) {
        if (lx - 1 - run_l > best_r - best_l) { best_l = run_l; best_r = lx - 1; }
        run_l = -1;
      }
    }
    if (best_r < best_l) { best_l = open_r[ly] + 1; best_r = open_r[ly]; }   // no clear run: all edge
    clear_l[ly] = best_l;
    clear_r[ly] = best_r;
  }
  return true;
}

// Creates the scene object on `parent`. Returns nullptr if it cannot be set up (the caller then uses LVGL images).
// window_x / window_y: top-left of the globe window rectangle. pivot_x / pivot_y: texel shown at the window centre
// at zero pitch.
static lv_obj_t *create(lv_obj_t *parent, const lv_img_dsc_t *bezel, const lv_img_dsc_t *globe,
                        int window_x, int window_y, int pivot_x, int pivot_y) {
  if (globe->header.cf != LV_IMG_CF_TRUE_COLOR || sizeof(lv_color_t) != 2) return nullptr;
  win_x = window_x; win_y = window_y;
  tex_w = globe->header.w; tex_h = globe->header.h;
  tex_pivot_x = pivot_x; tex_pivot_y = pivot_y;

  // The texture is read as 16-bit words, so it needs an even address. Flash data is not guaranteed to have one,
  // and PSRAM also reads faster than flash on this board (42 against 18 MB/s), so use a PSRAM copy when possible.
  uint16_t *copy = (uint16_t *)heap_caps_malloc(globe->data_size, MALLOC_CAP_SPIRAM);
  if (copy) {
    memcpy(copy, globe->data, globe->data_size);
    tex = copy;
  } else if (((uintptr_t)globe->data & 1) == 0) {
    tex = (const uint16_t *)globe->data;
  } else {
    return nullptr;
  }

  if (!prepare_flat_map()) return nullptr;
  if (!prepare_bezel(bezel, lv_obj_get_style_bg_color(parent, LV_PART_MAIN))) return nullptr;

  scene = lv_obj_create(parent);
  lv_obj_remove_style_all(scene);
  lv_obj_set_pos(scene, 0, 0);
  lv_obj_set_size(scene, SCR_W, SCR_H);
  lv_obj_clear_flag(scene, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(scene, draw, LV_EVENT_COVER_CHECK, nullptr);
  lv_obj_add_event_cb(scene, draw, LV_EVENT_DRAW_MAIN, nullptr);
  return scene;
}

// Sets the attitude and schedules a redraw of the window if it changed.
static void set_attitude(float pitch_px, float bank_rad) {
  if (!scene || (pitch_px == att_pitch_px && bank_rad == att_bank_rad)) return;
  att_pitch_px = pitch_px;
  att_bank_rad = bank_rad;
  lv_area_t win = { (lv_coord_t)win_x, (lv_coord_t)win_y, (lv_coord_t)(win_x + WIN_W - 1), (lv_coord_t)(win_y + WIN_H - 1) };
  lv_obj_invalidate_area(scene, &win);
}

}  // namespace globe_renderer
