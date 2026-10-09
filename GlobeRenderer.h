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

// The sketch is built with -Os, which leaves the small per-pixel helpers below as real function calls (measured at
// about 1 microsecond per textured pixel). This file is built for speed instead, and the helpers are forced inline.
#pragma GCC push_options
#pragma GCC optimize("O2")
#define GLOBE_INLINE static inline __attribute__((always_inline))

namespace globe_renderer {

constexpr int SCR_W = 480, SCR_H = 480;
// Globe window rectangle: the old LVGL "holder" (275 x 275) plus a margin on every side. The hole in the bezel
// image is not centred on the holder (it sits 2 px to one side and 2 px up), so without the margin its last two
// columns and rows fell outside the rectangle and showed the screen background as a thin light sliver.
// The margin keeps the centre, and with it the whole picture, where it was.
constexpr int WIN_MARGIN = 2;
constexpr int WIN_W = 275 + 2 * WIN_MARGIN, WIN_H = 275 + 2 * WIN_MARGIN;

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
static int flat_colours = 0;

// The same map again with 16x16 blocks. A pixel that lands in a flat block here lets the renderer fill every
// following pixel of the scanline that is still in the block, up to 16 at a time, with one lookup.
constexpr int COARSE_SHIFT = 4;
static uint8_t *coarse_map = nullptr;     // coarse_w x coarse_h
static int coarse_w = 0, coarse_h = 0;

// Layers: images drawn over the globe and under the bezel, in the order they were added (the aircraft symbol, then
// the ILS needles on top of it). One that is moved out past the edge of the window disappears behind the bezel.
// Each is an upright image that only ever slides, never turns.
struct Layer {
  uint16_t *rgb;                          // w x h colours
  uint8_t  *a;                            // w x h alpha
  int w, h;
  int x, y;                               // top-left in window coordinates
};
constexpr int MAX_LAYERS = 3;
static Layer layers[MAX_LAYERS];
static int layer_n = 0;

// Attitude for the next draw
static float att_pitch_px = 0.0f;         // texture rows the globe is shifted by
static float att_bank_rad = 0.0f;

static uint32_t prof_draw_us = 0;         // time spent in draw(), for SARI_PROFILE

// Blend two RGB565 colours. w is the weight of b, 0..32.
GLOBE_INLINE uint16_t mix565(uint32_t a, uint32_t b, uint32_t w) {
  a = (a | (a << 16)) & 0x07E0F81F;
  b = (b | (b << 16)) & 0x07E0F81F;
  const uint32_t r = (a + (((b - a) * w) >> 5)) & 0x07E0F81F;
  return (uint16_t)(r | (r >> 16));
}

// Texel fetch that is safe for any coordinate. Past the top or bottom of the texture the view has gone over
// the pole, so the texture is mirrored (and flipped left to right).
GLOBE_INLINE uint16_t texel(int u, int v) {
  if (v < 0)           { v = -1 - v;            u = tex_w - 1 - u; }
  else if (v >= tex_h) { v = 2 * tex_h - 1 - v; u = tex_w - 1 - u; }
  if (v < 0) v = 0; else if (v >= tex_h) v = tex_h - 1;
  if (u < 0) u = 0; else if (u >= tex_w) u = tex_w - 1;
  return tex[v * tex_w + u];
}

// Bilinear sample at a 16.16 fixed-point texture coordinate, any coordinate.
GLOBE_INLINE uint16_t sample_checked(int32_t u, int32_t v) {
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

// Bilinear sample straight from the texture, for coordinates known to be at least one texel inside it on
// every side.
GLOBE_INLINE uint16_t sample_inside(int32_t u, int32_t v) {
  const uint16_t *p = tex + (v >> 16) * tex_w + (u >> 16);
  const uint16_t c00 = p[0], c01 = p[1], c10 = p[tex_w], c11 = p[tex_w + 1];
  if (c00 == c01 && c00 == c10 && c00 == c11) return c00;
  const uint32_t fu = (u >> 11) & 31, fv = (v >> 11) & 31;
  return mix565(mix565(c00, c01, fu), mix565(c10, c11, fu), fv);
}

// Draws n pixels of one scanline whose texture coordinates all lie inside the texture (see sample_inside).
// The coordinate moves at most one texel per pixel along each axis, so a pixel that is `d` texels away from the
// edge of its block, in the direction of travel, is followed by at least `d` more pixels in the same block.
// In a flat block that whole run is one colour and is filled without looking anything else up.
GLOBE_INLINE void draw_inside_run(uint16_t *dst, int n, int32_t u, int32_t v, int32_t du, int32_t dv) {
  int i = 0;
  while (i < n) {
    const int ui = u >> 16, vi = v >> 16;
    int mask = (1 << COARSE_SHIFT) - 1;
    uint8_t k = coarse_map[(vi >> COARSE_SHIFT) * coarse_w + (ui >> COARSE_SHIFT)];
    if (k == FLAT_NONE) {
      mask = (1 << FLAT_SHIFT) - 1;
      k = flat_map[(vi >> FLAT_SHIFT) * flat_w + (ui >> FLAT_SHIFT)];
      if (k == FLAT_NONE) {
        dst[i++] = sample_inside(u, v);
        u += du; v += dv;
        continue;
      }
    }
    int run = n - i;
    if (du > 0) run = min(run, mask - (ui & mask) + 1); else if (du < 0) run = min(run, (ui & mask) + 1);
    if (dv > 0) run = min(run, mask - (vi & mask) + 1); else if (dv < 0) run = min(run, (vi & mask) + 1);
    const uint16_t colour = flat_colour[k];
    for (const int end = i + run; i < end; i++) dst[i] = colour;
    u += run * du; v += run * dv;
  }
}

GLOBE_INLINE bool inside_texture(int32_t u, int32_t v) {
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
    uint16_t *const win_dst = dst - lx_first;                  // this row's pixel for window column 0
    const uint16_t *const win_bz = bz - lx_first;
    const int32_t u_row = u00 + ly * du_dy, v_row = v00 + ly * dv_dy;
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
        draw_inside_run(dst, n, u, v, du_dx, dv_dx);
      } else {
        int32_t uu = u, vv = v;
        for (int i = 0; i < n; i++, uu += du_dx, vv += dv_dx) dst[i] = sample_checked(uu, vv);
      }
      dst += n; bz += n; lx += n; u += n * du_dx; v += n * dv_dx;
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

    // 6. layers, over the globe pixels just drawn. In the soft edge the globe is sampled again, so that the
    //    order stays globe, layer, bezel (a second layer there does not see the first; it is a pixel or two).
    for (int i = 0; i < layer_n; i++) {
      const Layer &nd = layers[i];
      const int ry = ly - nd.y;
      if (ry < 0 || ry >= nd.h) continue;
      const int xa = max(max(nd.x, ol), lx_first), xb = min(min(nd.x + nd.w - 1, orr), lx_last);
      for (int x = xa; x <= xb; x++) {
        const int k = ry * nd.w + (x - nd.x);
        const uint32_t na = (nd.a[k] + 4) >> 3;                // layer weight 0..32
        if (na == 0) continue;
        if (x >= cl && x <= cr) {
          win_dst[x] = mix565(win_dst[x], nd.rgb[k], na);
        } else {
          const uint32_t w = (al[x] + 4) >> 3;                 // bezel weight
          if (w >= 32) continue;
          const uint16_t under = mix565(sample_checked(u_row + x * du_dx, v_row + x * dv_dx), nd.rgb[k], na);
          win_dst[x] = mix565(under, win_bz[x], w);
        }
      }
    }
  }
  prof_draw_us += micros() - prof_t0;
}

// Builds a flat-colour map of the texture with blocks of (1 << shift) texels a side. Returns nullptr if there is
// no memory for it.
static uint8_t *build_flat_map(int shift, int &map_w, int &map_h) {
  map_w = (tex_w + (1 << shift) - 1) >> shift;
  map_h = (tex_h + (1 << shift) - 1) >> shift;
  uint8_t *map = (uint8_t *)heap_caps_malloc(map_w * map_h, MALLOC_CAP_INTERNAL);
  if (!map) map = (uint8_t *)heap_caps_malloc(map_w * map_h, MALLOC_CAP_SPIRAM);
  if (!map) return nullptr;
  int &colours = flat_colours;
  for (int by = 0; by < map_h; by++) {
    for (int bx = 0; bx < map_w; bx++) {
      const int x0 = bx << shift, y0 = by << shift;
      // one texel of border on every side, so the neighbours a bilinear sample reads are covered in any direction
      const int xa = max(x0 - 1, 0), ya = max(y0 - 1, 0);
      const int x1 = min(x0 + (1 << shift), tex_w - 1), y1 = min(y0 + (1 << shift), tex_h - 1);
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
      map[by * map_w + bx] = k;
    }
  }
  return map;
}

static bool prepare_flat_maps() {
  flat_map   = build_flat_map(FLAT_SHIFT, flat_w, flat_h);
  coarse_map = build_flat_map(COARSE_SHIFT, coarse_w, coarse_h);
  return flat_map && coarse_map;
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
// window_x / window_y: top-left of the globe window rectangle, before the margin is added. pivot_x / pivot_y: texel shown at the window centre
// at zero pitch.
static lv_obj_t *create(lv_obj_t *parent, const lv_img_dsc_t *bezel, const lv_img_dsc_t *globe,
                        int window_x, int window_y, int pivot_x, int pivot_y) {
  if (globe->header.cf != LV_IMG_CF_TRUE_COLOR || sizeof(lv_color_t) != 2) return nullptr;
  win_x = window_x - WIN_MARGIN; win_y = window_y - WIN_MARGIN;
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

  if (!prepare_flat_maps()) return nullptr;
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

// Schedules a redraw of the globe window.
static void invalidate_window() {
  lv_area_t win = { (lv_coord_t)win_x, (lv_coord_t)win_y, (lv_coord_t)(win_x + WIN_W - 1), (lv_coord_t)(win_y + WIN_H - 1) };
  lv_obj_invalidate_area(scene, &win);
}

// Adds a layer from a TRUE_COLOR_ALPHA or INDEXED_8BIT image (its own colours) or an ALPHA_8BIT image (drawn in
// `colour`). It starts out of view. Returns its number for set_layer_pos, or -1 if it cannot be set up.
static int add_layer(const lv_img_dsc_t *img, lv_color_t colour) {
  if (!scene || layer_n == MAX_LAYERS) return -1;
  const lv_img_cf_t cf = img->header.cf;
  const bool indexed = cf == LV_IMG_CF_INDEXED_8BIT;
  if (!indexed && cf != LV_IMG_CF_ALPHA_8BIT && cf != LV_IMG_CF_TRUE_COLOR_ALPHA) return -1;
  Layer &nd = layers[layer_n];
  nd.w = img->header.w; nd.h = img->header.h;
  const int px = nd.w * nd.h;
  nd.rgb = (uint16_t *)heap_caps_malloc(px * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
  nd.a   = (uint8_t *)heap_caps_malloc(px, MALLOC_CAP_INTERNAL);
  if (!nd.rgb || !nd.a) { free(nd.rgb); free(nd.a); return -1; }
  const uint8_t *pal = img->data, *data = indexed ? img->data + 256 * 4 : img->data;
  for (int i = 0; i < px; i++) {
    if (indexed) {
      const uint8_t *p = pal + data[i] * 4;                    // B, G, R, A
      nd.rgb[i] = (uint16_t)(((p[2] >> 3) << 11) | ((p[1] >> 2) << 5) | (p[0] >> 3));
      nd.a[i] = p[3];
    } else if (cf == LV_IMG_CF_TRUE_COLOR_ALPHA) {
      const uint8_t *p = data + i * 3;                         // two colour bytes, then alpha
      nd.rgb[i] = LV_COLOR_16_SWAP ? (uint16_t)((p[0] << 8) | p[1]) : (uint16_t)(p[0] | (p[1] << 8));
      nd.a[i] = p[2];
    } else {
      nd.rgb[i] = lv_color_to16(colour);
      nd.a[i] = data[i];
    }
  }
  nd.x = WIN_W; nd.y = WIN_H;
  return layer_n++;
}

// Moves a layer. x, y: top-left of its image in window coordinates.
static void set_layer_pos(int layer, int x, int y) {
  if (layer < 0 || layer >= layer_n || (layers[layer].x == x && layers[layer].y == y)) return;
  layers[layer].x = x; layers[layer].y = y;
  invalidate_window();
}

// Sets the attitude and schedules a redraw of the window if it changed.
static void set_attitude(float pitch_px, float bank_rad) {
  if (!scene || (pitch_px == att_pitch_px && bank_rad == att_bank_rad)) return;
  att_pitch_px = pitch_px;
  att_bank_rad = bank_rad;
  invalidate_window();
}

}  // namespace globe_renderer

#undef GLOBE_INLINE
#pragma GCC pop_options
