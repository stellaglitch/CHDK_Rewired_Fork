// Live preview of the bends - an approximation, drawn over the live view.
// See include/bend_preview.h.
//
// The previous live bend wrote into Canon's viewport buffer and never worked:
// the pipeline refills that buffer by DMA many times a frame, so the bend was
// overwritten before it could be seen (raw.c, and docs/LIVEVIEW_NOTES.md).
// This does not touch the viewport. It reads it, and draws on the bitmap
// plane - the layer the OSD and the multiple exposure ghost live on, which
// nothing refills behind our back - and four things shape it:
//
//   It is a model of the sensor, not the sensor. The live view is 8 bit YUV
//   that Canon has already demosaiced, white balanced and tone mapped. To put
//   the bends through what they were written for, it is turned back into a
//   small Bayer mosaic at the sensor's own depth: decoded to linear light,
//   divided by the sensor's daylight white so green clips first the way it
//   does on the real chip, and packed exactly as the frame buffer is. Both
//   engines then run on that unchanged - the matrix through the same compiled
//   table and bus evaluator the capture uses, the chain through bendx_apply().
//
//   It is small. 176x120 photosites against 2888x2136, so an effect whose
//   geometry is set in sensor rows or bytes - an address line, a stride, a
//   row period - lands at the same place in the value domain but at a
//   different scale on screen. Value effects (the bit routing, Melt's
//   colours, the clamp) look much as they will; geometric ones are a hint.
//   The review after the shot is the exact picture, and remains the judge.
//
//   The palette is Canon's. Fifteen distinct colours on these bodies, so the
//   result is dithered to them, and only where the bends changed something:
//   everywhere else is left transparent and the real live view shows through
//   in full colour and at full rate. A Melt streak or one bent region sits
//   on an untouched picture; a matrix that changes every value covers it.
//
//   It is slow. One step every BPV_MS: build a frame, then paint it in two
//   halves - about two frames a second, which is what fits beside Canon's
//   own work in the GUI task. Canon also paints over this plane, which is why
//   it is repainted continuously rather than once.

#include "platform.h"
#include "camera_info.h"
#include "conf.h"
#include "gui.h"
#include "gui_draw.h"
#include "viewport.h"
#include "raw_buffer.h"
#include "bend.h"
#include "bend_seg.h"
#include "bendx.h"
#include "bend_preview.h"

//-------------------------------------------------------------------

#define BPV_W       176     // photosites across: a whole number of packed
                            // groups, and 352 screen columns at 2x
#define BPV_H       120     // photosites down: 240 rows at 2x
#define BPV_NB      CAM_SENSOR_BITS_PER_PIXEL
#define BPV_ROWLEN  ((BPV_W * BPV_NB) / 8)
#define BPV_MAX     ((1 << BPV_NB) - 1)
#define BPV_KEEP    8       // 8-bit difference below which a pixel counts as
                            // unchanged and the live view is left showing

static unsigned char  *bpv_raw;     // BPV_H rows, packed as the frame buffer is
static unsigned char  *bpv_orig;    // BPV_W*BPV_H, unbent value in 8 bits
static unsigned short *bpv_lut;     // the matrix's value table
static unsigned char  *bpv_scr;     // bendx scratch, grown on demand
static unsigned        bpv_scr_len;
static bend_cc_t       bpv_cc;

#define BPV_CUBE    16              // levels per channel in the colour cube
static unsigned char  *bpv_cube;    // BPV_CUBE^3 palette bytes - see bpv_cube_build
static unsigned        bpv_cube_sum;// checksum of the palette it was built for
static unsigned char   bpv_prgb[256][3];    // what each palette byte looks like
static short           bpv_err[2][BPV_W + 2][3];   // error diffusion, two rows

static int      bpv_step;           // 0 build, 1 paint top, 2 paint bottom
static int      bpv_have;           // bpv_raw holds a finished frame
static int      bpv_painted;        // something of ours is on screen
static unsigned bpv_frame;          // BUS_FRAME, counting preview frames
static unsigned bpv_white[3];       // sensor daylight white, green = 1000
static int      bpv_g_even, bpv_r_odd;

// Linear light, /4096, from an 8-bit display value - and back. Gamma 2:
// near enough to Canon's curve for a fifteen-colour picture, and exact to
// invert with a square and a square root, which this CPU has neither of for
// anything fancier.
static unsigned short bpv_lin[256];
static unsigned char  bpv_enc[257];     // index = linear / 16

//-------------------------------------------------------------------

void bend_preview_neutral(unsigned *out)
{
    // The DNG colour matrix maps XYZ to camera RGB, so D65 white through it is
    // the sensor's own idea of white. Left at zero if the port's matrix is
    // missing or nonsense.
    static const int d65[3] = { 9505, 10000, 10890 };   // XYZ, /10000
    const int *m = camera_sensor.color_matrix1;
    int s[3], r, k;

    out[0] = out[1] = out[2] = 0;
    for (r = 0; r < 3; r++)
    {
        s[r] = 0;
        for (k = 0; k < 3; k++)
        {
            int num = m[2 * (3 * r + k)], den = m[2 * (3 * r + k) + 1], coef;
            if (den <= 0) return;
            coef = (den >= 1000) ? num / (den / 1000) : num * (1000 / den);
            s[r] += coef * d65[k] / 10000;          // per mille
        }
        if (s[r] <= 0) return;
    }
    for (k = 0; k < 3; k++) out[k] = (unsigned)(s[k] * 1000 / s[1]);
}

//-------------------------------------------------------------------

static void bpv_free(void)
{
    if (bpv_raw)  { free(bpv_raw);  bpv_raw  = 0; }
    if (bpv_orig) { free(bpv_orig); bpv_orig = 0; }
    if (bpv_lut)  { free(bpv_lut);  bpv_lut  = 0; }
    if (bpv_scr)  { free(bpv_scr);  bpv_scr  = 0; bpv_scr_len = 0; }
    if (bpv_cube) { free(bpv_cube); bpv_cube = 0; }
    bpv_have = 0;
    bpv_step = 0;
}

static int bpv_alloc(void)
{
    unsigned i;

    if (bpv_raw && bpv_orig && bpv_lut) return 1;

    bpv_raw  = malloc(BPV_ROWLEN * BPV_H);
    bpv_orig = malloc(BPV_W * BPV_H);
    bpv_lut  = malloc((BPV_MAX + 1) * sizeof(unsigned short));
    if (!bpv_raw || !bpv_orig || !bpv_lut) { bpv_free(); return 0; }

    for (i = 0; i < 256; i++) bpv_lin[i] = (unsigned short)((i * i * 4096u) / (255u * 255u));
    for (i = 0; i <= 256; i++)
    {
        // Integer square root of i/256, scaled to 255.
        unsigned t = i * 255u * 255u / 256u, r = 0;
        while ((r + 1) * (r + 1) <= t) r++;
        bpv_enc[i] = (unsigned char)r;
    }

    bend_preview_neutral(bpv_white);
    if (!bpv_white[0] || !bpv_white[1] || !bpv_white[2])
        bpv_white[0] = bpv_white[1] = bpv_white[2] = 1000;

    bpv_g_even = (camera_sensor.cfa_pattern == 0x01020001 || camera_sensor.cfa_pattern == 0x01000201);
    bpv_r_odd  = (camera_sensor.cfa_pattern == 0x00010102 || camera_sensor.cfa_pattern == 0x01000201);
    return 1;
}

// 0 red, 1 green, 2 blue, for one photosite.
static int bpv_chan(int x, int y)
{
    if ((((x + y) & 1) ^ bpv_g_even) != 0) return 1;
    return ((y & 1) == bpv_r_odd) ? 0 : 2;
}

//-------------------------------------------------------------------
// The live view, into the mosaic.
//
// UYVYYY - four pixels in six bytes, U and V signed - as on every body this
// builds for (gbc_feed.c has the layout). Nearest sample, and only the one
// channel each photosite would have seen, so no pixel is converted whole.

static int bpv_grab(void)
{
    static short col[BPV_W];
    static short row[BPV_H];
    const unsigned char *vid;
    unsigned black = camera_sensor.black_level;
    unsigned white = camera_sensor.white_level;
    unsigned span;
    int stride, rows, src_w, x, y;

    if (camera_info.state.mode_play) return 0;
    vid = (const unsigned char *)vid_get_viewport_active_buffer();
    if (!vid) return 0;

    stride = vid_get_viewport_byte_width();
    rows   = (int)vid_get_viewport_height() * vid_get_viewport_yscale();
    if (stride < 6 || stride > 4096 || rows < 2 || rows > 1024) return 0;
    src_w = (stride / 6) * 4;
    if (white <= black || white > BPV_MAX) white = BPV_MAX;
    if (black >= white) black = 0;
    span = white - black;

    // Photosite x covers screen columns 2x, 2x+1; the viewport spans the
    // whole screen width, so that is the same fraction of it.
    for (x = 0; x < BPV_W; x++)
    {
        int sx = (2 * x * src_w) / camera_screen.width;
        if (sx >= src_w) sx = src_w - 1;
        col[x] = (short)sx;
    }
    for (y = 0; y < BPV_H; y++)
    {
        int sy = (2 * y * rows) / camera_screen.height;
        if (sy >= rows) sy = rows - 1;
        row[y] = (short)sy;
    }

    for (y = 0; y < BPV_H; y++)
    {
        const unsigned char *src = vid + (unsigned)row[y] * (unsigned)stride;
        unsigned char *dst = bpv_raw + y * BPV_ROWLEN;
        unsigned char *o8  = bpv_orig + y * BPV_W;

        for (x = 0; x < BPV_W; x++)
        {
            static const unsigned char yoff[4] = { 1, 3, 4, 5 };
            int sx = col[x];
            const unsigned char *g = src + (sx >> 2) * 6;
            int Y = g[yoff[sx & 3]];
            int U = (signed char)g[0];
            int V = (signed char)g[2];
            int ch = bpv_chan(x, y), c;
            unsigned v;

            if      (ch == 0) c = Y + ((V * 1436) >> 10);
            else if (ch == 1) c = Y - ((U * 352 + V * 731) >> 10);
            else              c = Y + ((U * 1815) >> 10);
            if (c < 0) c = 0;
            if (c > 255) c = 255;

            // Linear, then under the sensor's white: green reaches the top of
            // the range at display white, red and blue fall short of it, as
            // they do on the chip.
            v = ((unsigned)bpv_lin[c] * span) >> 12;
            v = black + (v * bpv_white[ch]) / 1000u;
            if (v > BPV_MAX) v = BPV_MAX;

            bendx_set(dst, (unsigned)x, v, BPV_NB);
            o8[x] = (unsigned char)(v >> (BPV_NB - 8));
        }
    }
    return 1;
}

//-------------------------------------------------------------------
// The matrix. bb_run() in raw.c, over the mosaic instead of the sensor: the
// same compiled table, the same bus evaluator and the same segment spans.
// Positions handed to the evaluator are scaled up to sensor coordinates, so
// the clock buses run at roughly the rate they will on the shot.

static void bpv_bend(void)
{
    int layout, size, n, seg;
    unsigned sxs = camera_sensor.raw_rowpix / BPV_W;
    unsigned sys = camera_sensor.raw_rows / BPV_H;
    unsigned black = camera_sensor.black_level & BPV_MAX;

    bend_seg_sanitize(&conf.bend_segs, BPV_NB);
    layout = conf.bend_segs.layout;
    size   = conf.bend_segs.size;
    n      = bend_seg_count(layout);
    if (!sxs) sxs = 1;
    if (!sys) sys = 1;

    for (seg = 0; seg < n; seg++)
    {
        // A copy: simplifying the camera's own matrix is the capture's call.
        bend_t b = *bend_seg_conf(seg);
        int period, chan, x, y;

        bend_sanitize(&b, BPV_NB);
        if (conf.bitbend_simple) bend_simplify(&b);
        bend_compile(&b, &bpv_cc, bpv_lut, BPV_NB);
        if (bpv_cc.is_identity) continue;

        period = b.row_period;
        chan   = b.bayer;

        for (y = 0; y < BPV_H; y++)
        {
            unsigned char segs[BEND_SEG_SPANS];
            unsigned short xend[BEND_SEG_SPANS];
            unsigned char *rowp = bpv_raw + y * BPV_ROWLEN;
            bend_src_t src;
            unsigned noise_state;
            int nsp, i;

            if (period > 1 && (y % period) != 0) continue;
            nsp = bend_seg_spans(layout, size, y, BPV_W, BPV_H, segs, xend);

            src.frame = bpv_frame;
            src.ob    = black;      // the mosaic has no masked columns
            src.above = src.tap = src.noise = 0;
            noise_state = bend_trash_seed(&b, (unsigned)y, src.ob);

            for (i = 0; i < nsp; i++)
            {
                int x0 = (i > 0) ? xend[i - 1] : 0;
                int x1 = xend[i];

                if (segs[i] != (unsigned char)seg) continue;
                src.prev = (x0 > 0) ? bendx_get(rowp, (unsigned)(x0 - 1), BPV_NB) : 0;

                for (x = x0; x < x1; x++)
                {
                    unsigned v;

                    if (chan && (int)(((y & 1) << 1) | (x & 1)) != chan - 1) continue;
                    v = bendx_get(rowp, (unsigned)x, BPV_NB);

                    if (bpv_cc.is_pure)
                    {
                        bendx_set(rowp, (unsigned)x, bpv_lut[v], BPV_NB);
                        continue;
                    }
                    if (bpv_cc.bus_used & (1u << BUS_NOISE))
                        src.noise = bend_trash_step(&b, &noise_state, (unsigned)x * sxs);
                    if (bpv_cc.bus_used & (1u << BUS_VHOLD))
                        src.above = (y > 0) ? bendx_get(rowp - BPV_ROWLEN, (unsigned)x, BPV_NB) : 0;
                    if (bpv_cc.bus_used & (1u << BUS_TAP))
                    {
                        // The tap is in sensor pixels; at this scale most taps
                        // land on the neighbour, which is what they look like.
                        int tx = x + (int)b.tap_dx / (int)sxs;
                        int ty = y + (int)b.tap_dy / (int)sys;
                        if (tx < 0) tx = 0;
                        if (ty < 0) ty = 0;
                        if (tx >= BPV_W) tx = BPV_W - 1;
                        if (ty >= BPV_H) ty = BPV_H - 1;
                        src.tap = bendx_get(bpv_raw + ty * BPV_ROWLEN, (unsigned)tx, BPV_NB);
                    }
                    bendx_set(rowp, (unsigned)x,
                              bend_eval(&bpv_cc, v, x * (int)sxs, y * (int)sys, &src), BPV_NB);
                    src.prev = v;
                }
            }
        }
    }
}

//-------------------------------------------------------------------
// The chain. raw_bendx(), over the mosaic.

#ifdef CAM_BEND_EXPERIMENTAL
static void bpv_bendx(void)
{
    bendx_chain_t *c = &conf.bendx;
    bendx_buf_t buf;
    bendx_env_t env;
    unsigned need;
    int i, n;

    bendx_chain_sanitize(c);
    n = bendx_chain_count(c);
    if (n <= 0) return;

    need = bendx_chain_scratch_bytes(c, BPV_ROWLEN);
    if (need > bpv_scr_len)
    {
        if (bpv_scr) free(bpv_scr);
        bpv_scr = malloc(need);
        bpv_scr_len = bpv_scr ? need : 0;
        if (!bpv_scr) return;
    }

    buf.base   = bpv_raw;
    buf.rowlen = BPV_ROWLEN;
    buf.rowpix = BPV_W;
    buf.rows   = BPV_H;
    buf.ob_x   = 0;
    buf.nbits  = BPV_NB;

#ifdef ROMBASEADDR
    env.rom     = (const unsigned char *)(ROMBASEADDR);
    env.rom_len = 0x00100000;
#else
    env.rom     = 0;
    env.rom_len = 0;
#endif
    {
        unsigned jl = 0;
        env.jpg     = (const unsigned char *)hook_jpeg_buffer(&jl);
        env.jpg_len = jl;
    }
    env.frame  = bpv_frame;
    env.black  = camera_sensor.black_level;
    env.white  = camera_sensor.white_level;
    env.g_even = (unsigned)bpv_g_even;
    env.r_odd  = (unsigned)bpv_r_odd;
    for (i = 0; i < 3; i++) env.neutral[i] = bpv_white[i];

    for (i = 0; i < n; i++)
        bendx_apply(&c->item[i], &buf, &env, bpv_scr);
}
#endif

//-------------------------------------------------------------------
// Painting.
//
// The colours are the camera's own. The first version of this matched against
// CHDK's twenty colour *names* with RGB guessed from the names, and the names
// are wrong on these bodies - "blue" is navy, record-mode "yellow" is orange,
// "dark green" is olive (include/theme.h has the whole story) - so the preview
// painted colours the picture did not have.
//
// So the palette is read live, the way "Dump palette to card" reads it: four
// bytes an entry, V U Y and a 2-bit opacity, and every fully opaque entry is
// a candidate. Nearest-colour against a hundred-odd entries per pixel would
// cost more than the rest of the preview put together, so the search is done
// once into a 16x16x16 cube and each pixel is one lookup. The cube is rebuilt
// only when the palette's contents change - record and playback load
// different ones.
//
// If the port cannot hand over its palette, the fallback is the model in
// include/theme.h: each byte is a 50/50 blend of two of sixteen base entries,
// nine of which are known. That gives forty-five colours, all real.


// Base entries of the blend model: nibble and approximate colour.
static const unsigned char bpv_base[][4] = {
    { 0x1, 255, 255, 255 },     // white
    { 0x2, 220,  40,  40 },     // red
    { 0x3, 128, 128, 128 },     // grey
    { 0x4,  64,  64,  64 },     // dark grey
    { 0x5,  50, 190,  50 },     // green
    { 0x6, 230, 230,  50 },     // yellow, a greenish one
    { 0xd,  40,  80, 230 },     // blue
    { 0xe, 245, 130,  30 },     // orange
    { 0xf,   0,   0,   0 },     // black
};
#define BPV_BASE_N  (int)(sizeof(bpv_base) / sizeof(bpv_base[0]))

static int bpv_cube_build(void)
{
    static unsigned char ci[256];   // candidate palette bytes
    static unsigned char cr[256], cg[256], cb[256];
    const unsigned char *pal = (const unsigned char *)vid_get_bitmap_active_palette();
    int size = vid_get_palette_size();
    unsigned sum = 0;
    int n = 0, i, j, r, g, b;

    if (pal && size >= 4 && size <= 4096)
    {
        for (i = 0; i < size; i++) sum = sum * 31u + pal[i];
        if (bpv_cube && sum == bpv_cube_sum) return 1;

        // Entry 0 is transparent whatever it holds, and is what unchanged
        // pixels are drawn with - never a colour.
        for (i = 1; i * 4 + 3 < size && i < 256; i++)
        {
            int v = pal[i * 4], u = pal[i * 4 + 1], yy = pal[i * 4 + 2];
            int su = (u > 127) ? u - 256 : u, sv = (v > 127) ? v - 256 : v;
            if ((pal[i * 4 + 3] & 3) != 3) continue;
            r = yy + (359 * sv) / 256;
            g = yy - (88 * su) / 256 - (183 * sv) / 256;
            b = yy + (454 * su) / 256;
            ci[n] = (unsigned char)i;
            cr[n] = (unsigned char)((r < 0) ? 0 : (r > 255) ? 255 : r);
            cg[n] = (unsigned char)((g < 0) ? 0 : (g > 255) ? 255 : g);
            cb[n] = (unsigned char)((b < 0) ? 0 : (b > 255) ? 255 : b);
            n++;
        }
    }
    else
    {
        if (bpv_cube && bpv_cube_sum == 0) return 1;
    }

    // Too few opaque entries to be a real palette, or none to read: the blend
    // model instead.
    if (n < 8)
    {
        n = 0;
        sum = 0;
        for (i = 0; i < BPV_BASE_N; i++)
            for (j = i; j < BPV_BASE_N; j++)
            {
                ci[n] = (unsigned char)((bpv_base[i][0] << 4) | bpv_base[j][0]);
                cr[n] = (unsigned char)((bpv_base[i][1] + bpv_base[j][1]) / 2);
                cg[n] = (unsigned char)((bpv_base[i][2] + bpv_base[j][2]) / 2);
                cb[n] = (unsigned char)((bpv_base[i][3] + bpv_base[j][3]) / 2);
                n++;
            }
    }

    if (!bpv_cube) bpv_cube = malloc(BPV_CUBE * BPV_CUBE * BPV_CUBE);
    if (!bpv_cube) return 0;

    for (r = 0; r < BPV_CUBE; r++)
    for (g = 0; g < BPV_CUBE; g++)
    for (b = 0; b < BPV_CUBE; b++)
    {
        int qr = r * 17, qg = g * 17, qb = b * 17;     // 0 .. 255
        int best = 0, bestd = 0x7fffffff;
        for (i = 0; i < n; i++)
        {
            int dr = qr - cr[i], dg = qg - cg[i], db = qb - cb[i];
            // Slightly toward green, where the eye sees most of the
            // difference between two near colours - not so far that a
            // distant hue never gets its blue-heavy neighbours picked.
            int d = 3 * dr * dr + 4 * dg * dg + 3 * db * db;
            if (d < bestd) { bestd = d; best = i; }
        }
        bpv_cube[(r * BPV_CUBE + g) * BPV_CUBE + b] = ci[best];
    }
    for (i = 0; i < n; i++)
    {
        bpv_prgb[ci[i]][0] = cr[i];
        bpv_prgb[ci[i]][1] = cg[i];
        bpv_prgb[ci[i]][2] = cb[i];
    }
    bpv_cube_sum = sum;
    return 1;
}

static void bpv_paint(int y0, int y1)
{
    unsigned black = camera_sensor.black_level;
    unsigned white = camera_sensor.white_level;
    unsigned span, shift = BPV_NB - 8;
    int x, y;

    if (!bpv_cube_build()) return;

    if (white <= black || white > BPV_MAX) white = BPV_MAX;
    if (black >= white) black = 0;
    span = white - black;

    // Floyd-Steinberg. The palette has no magenta and no hot pink, so a fixed
    // dither between neighbouring levels still lands those on plain red; with
    // the error carried forward, a pink area becomes red, pink and purple
    // pixels that average to the pink. Two rows of error, and the painting is
    // split across ticks top half then bottom half, so the row a half ends on
    // carries straight into the next - a fresh frame starts it at zero.
    if (y0 == 0)
    {
        short *e = &bpv_err[0][0][0];
        for (x = 0; x < (BPV_W + 2) * 3; x++) e[x] = 0;
    }

    for (y = y0; y < y1; y++)
    {
        // The 2x2 window below and right of each photosite holds one red, one
        // blue and two greens whatever its phase - the cheapest demosaic
        // there is, and enough at this size.
        int wy = (y < BPV_H - 1) ? y : y - 1;
        short (*cur)[3] = bpv_err[y & 1];
        short (*nxt)[3] = bpv_err[(y + 1) & 1];

        {
            short *e = &nxt[0][0];
            for (x = 0; x < (BPV_W + 2) * 3; x++) e[x] = 0;
        }

        for (x = 0; x < BPV_W; x++)
        {
            int wx = (x < BPV_W - 1) ? x : x - 1;
            unsigned sum[3] = { 0, 0, 0 };
            int cnt[3] = { 0, 0, 0 };
            int changed = 0, j;
            color cl;

            for (j = 0; j < 4; j++)
            {
                int px = wx + (j & 1), py = wy + (j >> 1);
                unsigned v = bendx_get(bpv_raw + py * BPV_ROWLEN, (unsigned)px, BPV_NB);
                int d = (int)(v >> shift) - (int)bpv_orig[py * BPV_W + px];
                int ch = bpv_chan(px, py);

                if (d >= BPV_KEEP || d <= -BPV_KEEP) changed = 1;
                sum[ch] += (v > black) ? v - black : 0;
                cnt[ch]++;
            }

            if (!changed)
                cl = COLOR_TRANSPARENT;
            else
            {
                int want[3], q[3], k;

                for (k = 0; k < 3; k++)
                {
                    // Undo the sensor's white, then display gamma, then add
                    // the error the pixels before this one left behind.
                    unsigned lin = cnt[k] ? sum[k] / (unsigned)cnt[k] : 0;
                    lin = ((lin << 12) / span) * 1000u / bpv_white[k];
                    if (lin > 4096u) lin = 4096u;
                    want[k] = bpv_enc[lin >> 4] + cur[x + 1][k];
                    if (want[k] < 0) want[k] = 0;
                    if (want[k] > 255) want[k] = 255;
                    q[k] = (want[k] * 3855 + 32896) >> 16;    // round(x * 15 / 255)
                }
                cl = bpv_cube[(q[0] * BPV_CUBE + q[1]) * BPV_CUBE + q[2]];

                for (k = 0; k < 3; k++)
                {
                    // Capped, so one hopeless colour - a hue the palette
                    // has nothing near - does not smear across the row.
                    int e = want[k] - (int)bpv_prgb[cl & 0xff][k];
                    if (e >  128) e =  128;
                    if (e < -128) e = -128;
                    cur[x + 2][k] = (short)(cur[x + 2][k] + (e * 7) / 16);
                    nxt[x    ][k] = (short)(nxt[x    ][k] + (e * 3) / 16);
                    nxt[x + 1][k] = (short)(nxt[x + 1][k] + (e * 5) / 16);
                    nxt[x + 2][k] = (short)(nxt[x + 2][k] + e / 16);
                }
            }

            draw_pixel(2 * x,     2 * y,     cl);
            draw_pixel(2 * x + 1, 2 * y,     cl);
            draw_pixel(2 * x,     2 * y + 1, cl);
            draw_pixel(2 * x + 1, 2 * y + 1, cl);
        }
    }
}

//-------------------------------------------------------------------

int bend_preview_wanted(void)
{
    if (!conf.bend_preview) return 0;
    if (conf.bitbend_enable && bend_seg_any()) return 1;
#ifdef CAM_BEND_EXPERIMENTAL
    if (conf.bendx_enable && bendx_chain_count(&conf.bendx) > 0) return 1;
#endif
    return 0;
}

int bend_preview_draw(int force)
{
    if (!bpv_alloc()) return 0;

    // A screen that has just been erased gets the whole of the last frame
    // back at once; a preview that fades in a half at a time after every
    // menu would read as a fault.
    if (force && bpv_have)
    {
        bpv_paint(0, BPV_H);
        bpv_painted = 1;
        return 1;
    }

    switch (bpv_step)
    {
    case 0:
        if (!bpv_grab()) return 0;
        bpv_frame++;
        if (conf.bitbend_enable) bpv_bend();
#ifdef CAM_BEND_EXPERIMENTAL
        if (conf.bendx_enable) bpv_bendx();
#endif
        bpv_have = 1;
        bpv_step = 1;
        return 0;
    case 1:
        bpv_paint(0, BPV_H / 2);
        bpv_step = 2;
        break;
    default:
        bpv_paint(BPV_H / 2, BPV_H);
        bpv_step = 0;
        break;
    }
    bpv_painted = 1;
    return 1;
}

void bend_preview_idle(int wiped)
{
    if (bpv_painted)
    {
        bpv_painted = 0;
        // A shot wipes the whole plane itself (posd_hide_now); asking for a
        // restore on top of that would repaint into Canon's review.
        if (!wiped) gui_set_need_restore();
    }
    if (!conf.bend_preview || camera_info.state.mode_play) bpv_free();
    // Whatever was half painted is stale by the time the preview is back.
    bpv_step = 0;
}
