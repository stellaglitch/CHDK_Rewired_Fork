//-------------------------------------------------------------------
// Experimental profiles - see include/bendx.h and docs/EXPERIMENTAL_EFFECTS.md
//
// One named failure of the path between the ADC and the JPEG encoder, applied
// to the frame buffer after the routing matrix in bend.c has finished with the
// bus. No CHDK dependencies, so tools/bendx_selftest.c can compile it on the
// host and prove the bounds - which matters more here than it does for bend.c,
// because these effects address the buffer themselves instead of being handed
// one pixel at a time.
//
// Every write in this file lands inside buf->base[0 .. rows*rowlen). Every
// read of a foreign buffer lands inside the length the caller declared for it.
// Those two sentences are the whole safety argument, and the host test exists
// to keep them true.
//-------------------------------------------------------------------

#include "camera.h"
#include "bendx.h"

#ifdef CAM_BEND_EXPERIMENTAL

//-------------------------------------------------------------------

const char * const bendx_mix_names[BXMIX_COUNT] = {
    "Replace", "XOR", "OR", "AND"
};

// At most 12 characters - the picker column is 20 and has to hold a marker,
// the name and the amount.
static const char * const bx_names[BX_COUNT] = {
    "Off",
    "Byte order",       // BX_ENDIAN
    "Bit slip",         // BX_SLIP
    "Sync tear",        // BX_TEAR
    "Row addr",         // BX_ADDR
    "Col addr",         // BX_COLADDR
    "DMA stride",       // BX_STRIDE
    "Refresh",          // BX_REFRESH
    "Bus echo",         // BX_ECHO
    "Line mem",         // BX_LINEMEM
    "Field flip",       // BX_FIELD
    "ROM bus",          // BX_ROMBUS
    "Clock slip",       // BX_CLOCK
    "CFA phase",        // BX_CFA
    "Clamp run",        // BX_CLAMP
    "Bloom",            // BX_BLOOM
    "Pixel sort",       // BX_SORT
    "Lock loss",        // BX_LOCK
    "JPEG bus",         // BX_JPGBUS
    "Melt",             // BX_MELT
    "Aerochrome",       // BX_AERO
    "Sky rain",         // BX_RAIN
};

// One line each, for the menu. What the hardware fault is, not what the code
// does - the code is the imitation and the fault is the thing.
static const char * const bx_descs[BX_COUNT] = {
    "No experimental profile",
    "Bytes latched in the wrong order",
    "Row read back off its bit boundary",
    "Bit slip growing down the frame",
    "A row address line stuck",
    "A column address line stuck",
    "DMA stride mis-programmed",
    "DRAM refresh missed - bits decay",
    "The bus reflected back on itself",
    "Line memory holding the last row",
    "Field phase inverted",
    "Program ROM on the data lanes",
    "Pixel clock at the wrong rate",
    "Demosaic on the wrong Bayer phase",
    "Black level clamp loop unlocked",
    "Saturation into the V register",
    "Runs sorted along the row",
    "Timing losing lock and re-finding",
    "The JPEG buffer on the data lanes",
    "Highlights draining down the frame",
    "Kodak's false-colour infrared film",
    "Thin streaks falling from the clouds",
};

//-------------------------------------------------------------------

static unsigned xs32(unsigned s)
{
    if (!s) s = 0x9e3779b9u;
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
}

// Bytes per packed group, and pixels in it. The frame buffer is not a linear
// bit stream: eight pixels share ten bytes at 10bpp and four share six at
// 12bpp, in an order that is not ascending. That is why the byte-domain
// effects below scramble rather than shift, and why they look like a memory
// fault rather than like a filter.
static unsigned bx_gsz(int nbits)  { return (nbits == 10) ? 10u : 6u; }
static unsigned bx_gpix(int nbits) { return (nbits == 10) ?  8u : 4u; }

// Pixels the pixel-domain effects may address.
//
// A packed group is indivisible: writing any pixel of it touches all of its
// bytes, so a row whose pixel count is not a whole number of groups has a tail
// that cannot be written without reaching past the end of the row - and on the
// last row of the frame, past the end of the buffer. Both cameras here divide
// exactly (3152/8 and 3720/4) so nothing is lost, but rowpix is a per-camera
// constant and the next port may not be so tidy. The tail keeps its own value,
// which is at most three pixels of the far right edge.
static unsigned bx_wholepix(const bendx_buf_t *b)
{
    return b->rowpix - (b->rowpix % bx_gpix(b->nbits));
}

//-------------------------------------------------------------------
// Packed accessors. Lifted from get_raw_pixel/set_raw_pixel in core/raw.c and
// made row-relative; tools/bendx_selftest.c checks them against the originals
// for every phase at both depths.

unsigned bendx_get(const unsigned char *p, unsigned x, int nbits)
{
    if (nbits == 10)
    {
        const unsigned char *a = p + (x / 8) * 10;
        switch (x % 8) {
            case 0: return ((0x3fc & (((unsigned)a[1]) << 2)) | (a[0] >> 6));
            case 1: return ((0x3f0 & (((unsigned)a[0]) << 4)) | (a[3] >> 4));
            case 2: return ((0x3c0 & (((unsigned)a[3]) << 6)) | (a[2] >> 2));
            case 3: return ((0x300 & (((unsigned)a[2]) << 8)) | (a[5]));
            case 4: return ((0x3fc & (((unsigned)a[4]) << 2)) | (a[7] >> 6));
            case 5: return ((0x3f0 & (((unsigned)a[7]) << 4)) | (a[6] >> 4));
            case 6: return ((0x3c0 & (((unsigned)a[6]) << 6)) | (a[9] >> 2));
            case 7: return ((0x300 & (((unsigned)a[9]) << 8)) | (a[8]));
        }
    }
    else
    {
        const unsigned char *a = p + (x / 4) * 6;
        switch (x % 4) {
            case 0: return (((unsigned)a[1])        << 4) | (a[0] >> 4);
            case 1: return (((unsigned)(a[0]&0x0f)) << 8) | (a[3]);
            case 2: return (((unsigned)a[2])        << 4) | (a[5] >> 4);
            case 3: return (((unsigned)(a[5]&0x0f)) << 8) | (a[4]);
        }
    }
    return 0;
}

void bendx_set(unsigned char *p, unsigned x, unsigned v, int nbits)
{
    if (nbits == 10)
    {
        unsigned char *a = p + (x / 8) * 10;
        switch (x % 8) {
            case 0: a[0]=(a[0]&0x3F)|(unsigned char)(v<<6); a[1]=(unsigned char)(v>>2);                       break;
            case 1: a[0]=(a[0]&0xC0)|(unsigned char)(v>>4); a[3]=(a[3]&0x0F)|(unsigned char)(v<<4);           break;
            case 2: a[2]=(a[2]&0x03)|(unsigned char)(v<<2); a[3]=(a[3]&0xF0)|(unsigned char)(v>>6);           break;
            case 3: a[2]=(a[2]&0xFC)|(unsigned char)(v>>8); a[5]=(unsigned char)v;                            break;
            case 4: a[4]=(unsigned char)(v>>2);             a[7]=(a[7]&0x3F)|(unsigned char)(v<<6);           break;
            case 5: a[6]=(a[6]&0x0F)|(unsigned char)(v<<4); a[7]=(a[7]&0xC0)|(unsigned char)(v>>4);           break;
            case 6: a[6]=(a[6]&0xF0)|(unsigned char)(v>>6); a[9]=(a[9]&0x03)|(unsigned char)(v<<2);           break;
            case 7: a[8]=(unsigned char)v;                  a[9]=(a[9]&0xFC)|(unsigned char)(v>>8);           break;
        }
    }
    else
    {
        unsigned char *a = p + (x / 4) * 6;
        switch (x % 4) {
            case 0: a[0]=(a[0]&0x0F)|(unsigned char)(v<<4); a[1]=(unsigned char)(v>>4);                       break;
            case 1: a[0]=(a[0]&0xF0)|(unsigned char)(v>>8); a[3]=(unsigned char)v;                            break;
            case 2: a[2]=(unsigned char)(v>>4);             a[5]=(a[5]&0x0F)|(unsigned char)(v<<4);           break;
            case 3: a[4]=(unsigned char)v;                  a[5]=(a[5]&0xF0)|(unsigned char)(v>>8);           break;
        }
    }
}

//-------------------------------------------------------------------
// Lanes and mixing.
//
// lanes is a mask over the eight data lanes of the frame buffer's memory bus,
// not over the ADC bits - after packing those two are not the same thing, and
// pretending they were would make the mask do something different on every
// pixel phase. In the pixel-domain effects at the bottom of the file it is
// used as a bit-plane mask instead, over the top eight planes, because there
// the value is unpacked and planes are what exist.

static unsigned char bx_lanes(const bendx_t *x)
{
    return x->lanes ? x->lanes : 0xff;
}

static unsigned char bx_mixb(unsigned char old, unsigned char nw, int mix)
{
    switch (mix)
    {
    case BXMIX_XOR: return (unsigned char)(old ^ nw);
    case BXMIX_OR:  return (unsigned char)(old | nw);
    case BXMIX_AND: return (unsigned char)(old & nw);
    default:        return nw;
    }
}

static unsigned char bx_merge(unsigned char old, unsigned char nw,
                              unsigned char lanes, int mix)
{
    unsigned char v = bx_mixb(old, nw, mix);
    return (unsigned char)((old & ~lanes) | (v & lanes));
}

// Same, in the value domain.
static unsigned bx_mixv(unsigned old, unsigned nw, unsigned mask, int mix)
{
    unsigned v;
    switch (mix)
    {
    case BXMIX_XOR: v = old ^ nw; break;
    case BXMIX_OR:  v = old | nw; break;
    case BXMIX_AND: v = old & nw; break;
    default:        v = nw;       break;
    }
    return (old & ~mask) | (v & mask);
}

// Plane mask for the pixel-domain effects. 0 means every plane, which is the
// default and the only setting most profiles ever want.
static unsigned bx_planes(const bendx_t *x, int nbits)
{
    unsigned all = (1u << nbits) - 1u;
    if (!x->lanes) return all;
    return ((unsigned)x->lanes << (nbits - 8)) & all;
}

// Row banding. rowmod 0 and 1 mean every row; otherwise the effect is on for
// rowmod rows and off for rowmod rows, so a profile can be left visibly
// interleaved with the clean picture instead of covering it.
static int bx_on(const bendx_t *x, unsigned y)
{
    unsigned m = x->rowmod;
    if (m < 2) return 1;
    return (y % (m * 2u)) < m;
}

//-------------------------------------------------------------------
// Ranges. Both inclusive, and 0 means the knob does nothing for this effect.

int bendx_amount_max(int kind)
{
    switch (kind)
    {
    case BX_ENDIAN:   return 4;     // which reordering
    case BX_SLIP:     return 15;    // bits, minus one
    case BX_TEAR:     return 15;    // bits per row, minus one
    case BX_ADDR:     return 11;    // which row address line
    case BX_COLADDR:  return 11;    // which column address line
    case BX_STRIDE:   return 63;    // bytes of stride error, minus one
    case BX_REFRESH:  return 15;    // severity
    case BX_ECHO:     return 63;    // delay in bytes, minus one
    case BX_LINEMEM:  return 63;    // period, minus two
    case BX_FIELD:    return 63;    // block height, minus two
    case BX_ROMBUS:   return 7;     // texture scale, as a shift
    case BX_CLOCK:    return 15;    // rate error
    case BX_CFA:      return 7;     // pixels across
    case BX_CLAMP:    return 15;    // clamp gain
    case BX_BLOOM:    return 15;    // how low the overflow threshold sits
    case BX_SORT:     return 15;    // threshold the runs are taken above
    case BX_LOCK:     return 15;    // how often lock is lost
    case BX_JPGBUS:   return 7;     // texture scale, as a shift
    case BX_MELT:     return 15;    // how far below clipping a highlight starts
    case BX_AERO:     return 15;    // how strongly foliage glows
    case BX_RAIN:     return 15;    // how many streaks
    }
    return 0;
}

int bendx_reach_max(int kind)
{
    switch (kind)
    {
    case BX_SLIP:     return 32;    // whole bytes on top of the bit rotation
    case BX_TEAR:     return 32;    // starting offset
    case BX_STRIDE:   return 15;    // coarse multiplier on the stride error
    case BX_ECHO:     return 15;    // coarse multiplier on the delay
    case BX_LINEMEM:  return 15;    // how many rows are held
    case BX_ROMBUS:   return 15;    // how fast the ROM window walks
    case BX_CLOCK:    return 7;     // wobble period, as a shift
    case BX_CFA:      return 7;     // rows down
    case BX_CLAMP:    return 15;    // how far away the wrong OB row is
    case BX_BLOOM:    return 15;    // how far the smear carries down
    case BX_SORT:     return 15;    // run length, and which way up - see below
    case BX_LOCK:     return 15;    // how far it jumps when it slips
    case BX_JPGBUS:   return 15;    // how fast the window walks
    case BX_MELT:     return 15;    // how far the streaks run
    case BX_AERO:     return 15;    // saturation of the dye layers
    case BX_RAIN:     return 15;    // how far they fall
    }
    return 0;
}

int bendx_tint_max(int kind)
{
    if (kind == BX_MELT || kind == BX_RAIN) return BXTINT_LOCAL;
    if (kind == BX_AERO) return BXTINT_RED;
    return 0;
}

int bendx_protect_max(int kind)
{
    return (kind == BX_MELT || kind == BX_RAIN) ? BXPROT_SKY : 0;
}

const char *bendx_protect_name(const bendx_t *x)
{
    if (bendx_protect_max(x->kind) <= 0) return "-";
    return (x->protect == BXPROT_SKY) ? "Sky only" : "Off";
}

const char *bendx_tint_name(const bendx_t *x)
{
    static const char * const melt[] = { "Classic", "Image", "Local" };
    static const char * const aero[] = { "Yellow 12", "Orange 21", "Red 25" };
    int max = bendx_tint_max(x->kind);
    int t = (x->tint <= max) ? x->tint : 0;

    if (max <= 0) return "-";
    return (x->kind == BX_AERO) ? aero[t] : melt[t];
}

int bendx_is_slow(int kind)
{
    switch (kind)
    {
    case BX_CLOCK: case BX_CFA:   case BX_CLAMP:
    case BX_BLOOM: case BX_SORT:  case BX_MELT: case BX_AERO: case BX_RAIN:
        return 1;
    }
    return 0;
}

int bendx_needs_env(int kind)
{
    return (kind == BX_ROMBUS || kind == BX_JPGBUS);
}

//-------------------------------------------------------------------
// How much working space an effect wants. Two rows unless it says otherwise.

#define BX_SORT_BUCKETS     4096u   // one per 12 bit value

unsigned bendx_scratch_bytes(int kind, unsigned rowlen)
{
    unsigned two = rowlen * BENDX_SCRATCH_ROWS;

    if (kind == BX_SORT)
    {
        // A counting sort over the whole value range. At 10bpp two rows are
        // 7880 bytes and this is 8192, so the default is not enough and the
        // caller has to be told - which is the entire reason this function
        // exists rather than a constant.
        unsigned hist = BX_SORT_BUCKETS * sizeof(unsigned short);
        return (hist > two) ? hist : two;
    }
    if (kind == BX_MELT || kind == BX_RAIN)
    {
        // Four shorts per two-pixel cell is four bytes per pixel, and a
        // pixel is at least 1.25 bytes of row at either depth, so four rows
        // always cover it. Then four bytes of alignment
        // and the colour grid: BXM_GH rows of blocks 64 cells wide, one more
        // for the whole frame, and one row more for the blend of the two grid
        // rows around the current row - sixteen bytes a block. A cell is at
        // least 2.5 bytes of row, so rowlen / 128 + 2 columns always covers it.
        return rowlen * 4u + 4u + 16u * (17u * (rowlen / 128u + 2u) + 1u);
    }
    return two;
}

unsigned bendx_chain_scratch_bytes(const bendx_chain_t *c, unsigned rowlen)
{
    unsigned most = rowlen * BENDX_SCRATCH_ROWS;
    int i, n = bendx_chain_count(c);

    for (i = 0; i < n; i++)
    {
        unsigned want = bendx_scratch_bytes(c->item[i].kind, rowlen);
        if (want > most) most = want;
    }
    return most;
}

const char *bendx_name(int kind)
{
    if (kind < 0 || kind >= BX_COUNT) kind = BX_OFF;
    return bx_names[kind];
}

const char *bendx_desc(int kind)
{
    if (kind < 0 || kind >= BX_COUNT) kind = BX_OFF;
    return bx_descs[kind];
}

//-------------------------------------------------------------------

void bendx_reset(bendx_t *x)
{
    x->kind   = BX_OFF;
    x->amount = 0;
    x->reach  = 0;
    x->lanes  = 0;
    x->mix    = BXMIX_SET;
    x->rowmod = 0;
    x->tint    = BXTINT_CLASSIC;
    x->protect = BXPROT_OFF;
    x->seed   = 0xb1a5edu;
}

int bendx_is_off(const bendx_t *x)
{
    return x->kind == BX_OFF || x->kind >= BX_COUNT;
}

// Change the effect, and start its knobs somewhere worth looking at.
//
// The knobs have to be reset: they are one pair of bytes that means "eleven
// bits of slip" under one effect and "address line eleven" under the next, so
// carrying them over is a setting that changed itself. Both UIs go through
// here so that they cannot disagree about that.
//
// The mix does not reset to Replace for the foreign buses. At Replace and full
// lanes, ROM bus and the two display buses overwrite every byte of the frame -
// the photograph is not damaged, it is gone, and there is nothing on the back
// of the camera to suggest which knob to turn to get it back. XOR keeps the
// picture and puts the foreign buffer through it, which is the effect people
// mean when they ask for this one; Replace is still there, one item down the
// menu, for when it is what you want.
void bendx_set_kind(bendx_t *x, int kind)
{
    unsigned keep = x->seed;

    if (kind < 0 || kind >= BX_COUNT) kind = BX_OFF;

    bendx_reset(x);
    x->seed = keep ? keep : x->seed;
    x->kind = (unsigned char)kind;
    if (bendx_needs_env(kind)) x->mix = BXMIX_XOR;
    // Sky rain is for keeping the subject out of it; starting it with the
    // subject unprotected would show the one look it was not made for.
    if (kind == BX_RAIN) x->protect = BXPROT_SKY;

    bendx_sanitize(x);
}

void bendx_sanitize(bendx_t *x)
{
    int m;

    if (x->kind >= BX_COUNT) { bendx_reset(x); return; }

    m = bendx_amount_max(x->kind);
    if (x->amount > m) x->amount = (unsigned char)m;
    m = bendx_reach_max(x->kind);
    if (x->reach > m) x->reach = (unsigned char)m;

    if (x->mix >= BXMIX_COUNT) x->mix = BXMIX_SET;
    if (x->rowmod > 64)        x->rowmod = 0;
    if (x->tint > bendx_tint_max(x->kind)) x->tint = BXTINT_CLASSIC;
    if (x->protect > bendx_protect_max(x->kind)) x->protect = BXPROT_OFF;
    if (!x->seed)              x->seed = 0xb1a5edu;
}

// The amount in the effect's own units, because "7" means seven different
// things across this list and the picker has room to say which.
void bendx_label(const bendx_t *x, char *buf)
{
    static const char * const endian[] = { "byte", "word", "grp", "nib", "rev" };
    unsigned v, div;
    int i = 0;

    buf[0] = 0;
    switch (x->kind)
    {
    case BX_OFF:
        return;

    case BX_ENDIAN:
        {
            const char *s = endian[(x->amount <= 4) ? x->amount : 0];
            while (*s) buf[i++] = *s++;
            buf[i] = 0;
        }
        return;

    case BX_SLIP:     v = (unsigned)x->amount + 1u + (unsigned)x->reach * 8u;  break;
    case BX_TEAR:     v = (unsigned)x->amount + 1u;                            break;
    case BX_ADDR:
    case BX_COLADDR:  v = 1u << x->amount;                                     break;
    case BX_STRIDE:
    case BX_ECHO:     v = (unsigned)x->amount + 1u + (unsigned)x->reach * 64u; break;
    case BX_LINEMEM:
    case BX_FIELD:    v = (unsigned)x->amount + 2u;                            break;
    default:          v = (unsigned)x->amount;                                 break;
    }

    // No sprintf here: this file is shared with the host test and with a
    // redraw path, and four digits do not justify pulling in the formatter.
    for (div = 1u; v / div >= 10u; div *= 10u) ;
    for (; div; div /= 10u) buf[i++] = (char)('0' + (v / div) % 10u);
    buf[i] = 0;
}

//-------------------------------------------------------------------
// The chain. See the layout note in bendx.h: item[0..n-1] are locked and
// item[n] is live, so the two are one list and "lock it" is n++.

void bendx_chain_reset(bendx_chain_t *c)
{
    int i;
    for (i = 0; i < BENDX_CHAIN_MAX; i++) bendx_reset(&c->item[i]);
    c->n = 0;
    c->pad[0] = c->pad[1] = c->pad[2] = 0;
}

void bendx_chain_sanitize(bendx_chain_t *c)
{
    int i;

    if (c->n > BENDX_CHAIN_MAX) c->n = 0;
    for (i = 0; i < BENDX_CHAIN_MAX; i++) bendx_sanitize(&c->item[i]);

    // A locked slot holding Off is a hole, and a hole makes the position
    // markers in the picker lie about what runs when. Close them here rather
    // than defending against them at every use: this is the only place the
    // invariant "item[0..n-1] are all live effects" is established, and it is
    // reachable from a config block written by any earlier build.
    for (i = 0; i < (int)c->n; )
    {
        if (bendx_is_off(&c->item[i])) bendx_chain_remove(c, i);
        else                           i++;
    }
}

int bendx_chain_count(const bendx_chain_t *c)
{
    int n = c->n;
    if (n < BENDX_CHAIN_MAX && !bendx_is_off(&c->item[n])) n++;
    return n;
}

bendx_t *bendx_chain_live(bendx_chain_t *c)
{
    return (c->n < BENDX_CHAIN_MAX) ? &c->item[c->n] : 0;
}

const bendx_t *bendx_chain_live_const(const bendx_chain_t *c)
{
    return (c->n < BENDX_CHAIN_MAX) ? &c->item[c->n] : 0;
}

int bendx_chain_find(const bendx_chain_t *c, int kind)
{
    int i, n = bendx_chain_count(c);

    if (kind == BX_OFF) return -1;
    for (i = 0; i < n; i++)
        if (c->item[i].kind == kind) return i;
    return -1;
}

int bendx_chain_lock(bendx_chain_t *c)
{
    if (c->n >= BENDX_CHAIN_MAX)      return 0;     // nothing left to open
    if (bendx_is_off(&c->item[c->n])) return 0;     // nothing live to freeze

    c->n++;
    // The slot that just became live has whatever an earlier chain left in it.
    if (c->n < BENDX_CHAIN_MAX) bendx_reset(&c->item[c->n]);
    return 1;
}

int bendx_chain_remove(bendx_chain_t *c, int i)
{
    int k;

    if (i < 0 || i >= BENDX_CHAIN_MAX) return 0;

    if (i >= (int)c->n)
    {
        // The live slot. Nothing to close up - it just goes back to Off.
        if (i == (int)c->n) { bendx_reset(&c->item[i]); return 1; }
        return 0;
    }

    // Shift the rest down, live slot included, so tuning is not disturbed by
    // removing something above it.
    for (k = i; k < BENDX_CHAIN_MAX - 1; k++) c->item[k] = c->item[k + 1];
    bendx_reset(&c->item[BENDX_CHAIN_MAX - 1]);
    c->n--;
    return 1;
}

int bendx_chain_slow(const bendx_chain_t *c)
{
    int i, n = bendx_chain_count(c);
    for (i = 0; i < n; i++)
        if (bendx_is_slow(c->item[i].kind)) return 1;
    return 0;
}

//-------------------------------------------------------------------
// Reroll the knobs, never the effect. The effect is the thing you chose; the
// knobs are the thing you are hunting through.

unsigned bendx_random(bendx_t *x, unsigned seed)
{
    unsigned s = xs32(seed);
    int am = bendx_amount_max(x->kind);
    int rm = bendx_reach_max(x->kind);

    x->seed = s;

    if (am > 0) { s = xs32(s); x->amount = (unsigned char)(s % (unsigned)(am + 1)); }
    if (rm > 0) { s = xs32(s); x->reach  = (unsigned char)(s % (unsigned)(rm + 1)); }

    // Full lanes most of the time. A partial lane mask is a good effect and a
    // bad default: three quarters of the masks make the damage too quiet to
    // see on the back of the camera, which is where it gets judged.
    s = xs32(s);
    x->lanes = (s % 100u < 70u) ? 0 : (unsigned char)(s >> 8);

    s = xs32(s);
    if (bendx_needs_env(x->kind))
    {
        // Never Replace on a foreign bus, for the reason in bendx_set_kind():
        // it does not damage the photograph, it removes it. A reroll that can
        // land on "no picture at all" is a reroll people stop pressing.
        x->mix = (unsigned char)(BXMIX_XOR + s % (BXMIX_COUNT - 1));
    }
    else if (s % 100u < 55u) x->mix = BXMIX_SET;
    else                     x->mix = (unsigned char)(s % BXMIX_COUNT);

    s = xs32(s);
    x->rowmod = (s % 100u < 70u) ? 0 : (unsigned char)(1 + s % 32u);

    bendx_sanitize(x);
    return s;
}

//-------------------------------------------------------------------
// Byte-domain effects.
//
// All of these walk the packed bytes in place, one row at a time, with at most
// two rows of scratch. That is one linear pass over the buffer - the same
// order of work the pure path in raw.c does - which is why they are the ones
// that can be left on.

// Rotate a row left by n bits, wrapping. The unit of the fault is the bit,
// because that is what a read that started early recovers: the bytes are
// intact and the boundaries are not.
// Optional UI service callback, called from inside the per-row loops below.
//
// Each profile is one pass over the whole sensor and there was no service point
// anywhere inside it, so an experimental chain held the display task for the
// entire pass - measured on the A470 at ~14s, which is exactly how long the
// persistent overlay stayed off the screen after a shot. The bend engine in
// core/raw.c already services every 64 rows for the same reason; this gives the
// experimental engine the same courtesy.
//
// Kept as a setter rather than a member of bendx_env_t so that the host
// self-tests, which build their own env on the stack, cannot leave it holding a
// stack value. Null until a caller sets it, which is what the self-tests get.
static void (*bx_service)(void);

void bendx_set_service(void (*fn)(void))
{
    bx_service = fn;
}

// Cheap enough to sit in every row loop: one test against a null pointer, and
// the call itself only every 64th row.
#define BX_TICK(y) do { if (bx_service && (((y) & 0x3f) == 0)) bx_service(); } while (0)

static void bx_rot(unsigned char *dst, const unsigned char *src,
                   unsigned len, unsigned bits)
{
    unsigned bo, bi, i;

    if (!len) return;
    bits %= len * 8u;
    bo = bits >> 3;
    bi = bits & 7u;

    if (!bi)
    {
        for (i = 0; i < len; i++) dst[i] = src[(i + bo) % len];
        return;
    }
    for (i = 0; i < len; i++)
    {
        unsigned a = src[(i + bo) % len];
        unsigned b = src[(i + bo + 1u) % len];
        dst[i] = (unsigned char)(((a << bi) | (b >> (8u - bi))) & 0xff);
    }
}

static void bx_blend_row(unsigned char *row, const unsigned char *nw,
                         unsigned len, unsigned char lanes, int mix)
{
    unsigned i;
    for (i = 0; i < len; i++) row[i] = bx_merge(row[i], nw[i], lanes, mix);
}

static void bx_endian(const bendx_t *x, const bendx_buf_t *b,
                      unsigned char *s0, unsigned char *s1)
{
    unsigned char lanes = bx_lanes(x);
    unsigned gsz = bx_gsz(b->nbits);
    unsigned len = b->rowlen, y, i;

    for (y = 0; y < b->rows; y++)
    {
        BX_TICK(y);
        unsigned char *row = b->base + (unsigned)y * len;
        if (!bx_on(x, y)) continue;

        for (i = 0; i < len; i++) s0[i] = row[i];

        switch (x->amount)
        {
        case 0:     // adjacent bytes swapped - a halfword latched backwards
            for (i = 0; i + 1 < len; i += 2)
            { s1[i] = s0[i+1]; s1[i+1] = s0[i]; }
            if (len & 1) s1[len-1] = s0[len-1];
            break;

        case 1:     // whole word reversed - the wrong endianness outright
            for (i = 0; i + 3 < len; i += 4)
            { s1[i]=s0[i+3]; s1[i+1]=s0[i+2]; s1[i+2]=s0[i+1]; s1[i+3]=s0[i]; }
            for (; i < len; i++) s1[i] = s0[i];
            break;

        case 2:     // the packed pixel group reversed, so the phases invert
            for (i = 0; i + gsz <= len; i += gsz)
            {
                unsigned k;
                for (k = 0; k < gsz; k++) s1[i+k] = s0[i + gsz - 1 - k];
            }
            for (; i < len; i++) s1[i] = s0[i];
            break;

        case 3:     // nibbles swapped - half a byte lane out of step
            for (i = 0; i < len; i++)
                s1[i] = (unsigned char)((s0[i] << 4) | (s0[i] >> 4));
            break;

        default:    // every byte bit-reversed - the lanes wired back to front
            for (i = 0; i < len; i++)
            {
                unsigned v = s0[i];
                v = ((v & 0xaa) >> 1) | ((v & 0x55) << 1);
                v = ((v & 0xcc) >> 2) | ((v & 0x33) << 2);
                v = ((v & 0xf0) >> 4) | ((v & 0x0f) << 4);
                s1[i] = (unsigned char)v;
            }
            break;
        }

        bx_blend_row(row, s1, len, lanes, x->mix);
    }
}

static void bx_slip(const bendx_t *x, const bendx_buf_t *b,
                    unsigned char *s0, unsigned char *s1)
{
    unsigned char lanes = bx_lanes(x);
    unsigned len = b->rowlen, y, i;
    unsigned bits = (unsigned)x->amount + 1u + (unsigned)x->reach * 8u;

    for (y = 0; y < b->rows; y++)
    {
        BX_TICK(y);
        unsigned char *row = b->base + (unsigned)y * len;
        if (!bx_on(x, y)) continue;
        for (i = 0; i < len; i++) s0[i] = row[i];
        bx_rot(s1, s0, len, bits);
        bx_blend_row(row, s1, len, lanes, x->mix);
    }
}

static void bx_tear(const bendx_t *x, const bendx_buf_t *b,
                    unsigned char *s0, unsigned char *s1)
{
    unsigned char lanes = bx_lanes(x);
    unsigned len = b->rowlen, y, i;
    unsigned per = (unsigned)x->amount + 1u;
    unsigned bits = (unsigned)x->reach * 8u;

    for (y = 0; y < b->rows; y++)
    {
        BX_TICK(y);
        unsigned char *row = b->base + (unsigned)y * len;
        bits += per;                    // grows whether or not the row is on,
                                        // so banding cuts the tear instead of
                                        // pausing it
        if (!bx_on(x, y)) continue;
        for (i = 0; i < len; i++) s0[i] = row[i];
        bx_rot(s1, s0, len, bits);
        bx_blend_row(row, s1, len, lanes, x->mix);
    }
}

// A stuck address line does not corrupt data, it fetches the wrong data: the
// two rows whose numbers differ only in that bit become the same row, or -
// since both halves of the pair are driven - trade places.
static void bx_addr(const bendx_t *x, const bendx_buf_t *b, unsigned char *s0)
{
    unsigned char lanes = bx_lanes(x);
    unsigned len = b->rowlen;
    unsigned step = 1u << x->amount;
    unsigned y, i;

    if (step >= b->rows) return;

    for (y = 0; y < b->rows; y++)
    {
        BX_TICK(y);
        unsigned char *ra, *rb;
        unsigned y2 = y | step;

        if (y & step) continue;         // handled as the low half of its pair
        if (y2 >= b->rows) continue;
        if (!bx_on(x, y)) continue;

        ra = b->base + (unsigned)y  * len;
        rb = b->base + (unsigned)y2 * len;
        for (i = 0; i < len; i++) s0[i] = ra[i];
        for (i = 0; i < len; i++)
        {
            unsigned char a = s0[i], c = rb[i];
            ra[i] = bx_merge(a, c, lanes, x->mix);
            rb[i] = bx_merge(c, a, lanes, x->mix);
        }
    }
}

// The same fault on the other axis. The unit is the packed group rather than
// the byte, so the blocks that trade places land on pixel boundaries and the
// result reads as displaced image rather than as noise.
static void bx_coladdr(const bendx_t *x, const bendx_buf_t *b)
{
    unsigned char lanes = bx_lanes(x);
    unsigned gsz = bx_gsz(b->nbits);
    unsigned len = b->rowlen;
    unsigned ngroups = len / gsz;
    unsigned step = 1u << x->amount;
    unsigned y, g, k;

    if (step >= ngroups) return;

    for (y = 0; y < b->rows; y++)
    {
        BX_TICK(y);
        unsigned char *row = b->base + (unsigned)y * len;
        if (!bx_on(x, y)) continue;

        for (g = 0; g < ngroups; g++)
        {
            unsigned char *ga, *gb;
            unsigned g2 = g | step;

            if (g & step) continue;
            if (g2 >= ngroups) continue;

            ga = row + g  * gsz;
            gb = row + g2 * gsz;
            for (k = 0; k < gsz; k++)
            {
                unsigned char a = ga[k], c = gb[k];
                ga[k] = bx_merge(a, c, lanes, x->mix);
                gb[k] = bx_merge(c, a, lanes, x->mix);
            }
        }
    }
}

// Stride is the one number the DMA engine needs that the picture does not
// contain. Get it wrong by d bytes and row y is fetched from d*y bytes into
// the frame - the image shears, and keeps shearing, and eventually wraps.
//
// Rows are rewritten in ascending order and every source lies at or after the
// row being written, so the source is still untouched when it is read. Once
// the offset wraps past the end the source is a row this pass already bent,
// which is the feedback a real runaway stride has and is wanted.
static void bx_stride(const bendx_t *x, const bendx_buf_t *b, unsigned char *s0)
{
    unsigned char lanes = bx_lanes(x);
    unsigned len = b->rowlen;
    unsigned total = len * b->rows;
    unsigned d = (unsigned)x->amount + 1u + (unsigned)x->reach * 64u;
    unsigned y, i;

    if (!total) return;

    for (y = 0; y < b->rows; y++)
    {
        BX_TICK(y);
        unsigned char *row = b->base + (unsigned)y * len;
        // y * (len + d) cannot overflow 32 bits: the largest sensor here is
        // 2772 rows of 5580 bytes and d tops out at 1024.
        unsigned src = (y * (len + d)) % total;

        if (!bx_on(x, y)) continue;

        if (src + len <= total)
            for (i = 0; i < len; i++) s0[i] = b->base[src + i];
        else
        {
            unsigned head = total - src;
            for (i = 0; i < head; i++)     s0[i] = b->base[src + i];
            for (; i < len; i++)           s0[i] = b->base[i - head];
        }
        bx_blend_row(row, s0, len, lanes, x->mix);
    }
}

// Refresh is the only thing keeping a DRAM cell at the value it was written.
// Miss it and cells drift to whichever rail their sense amp favours - and the
// longer since the write, the more of them have gone, which is why this gets
// worse down the frame rather than being uniform.
static void bx_refresh(const bendx_t *x, const bendx_buf_t *b)
{
    unsigned char lanes = bx_lanes(x);
    unsigned len = b->rowlen;
    unsigned y, i;
    unsigned sev = (unsigned)x->amount + 1u;
    int to_one = (x->mix == BXMIX_OR);

    if (!b->rows) return;

    for (y = 0; y < b->rows; y++)
    {
        BX_TICK(y);
        unsigned char *row = b->base + (unsigned)y * len;
        unsigned st = xs32(x->seed ^ (y * 0x9e3779b9u));
        // 0 at the top of the frame, sev/16 of all cells by the bottom.
        // Divided before the multiply so the whole thing stays in 32 bits.
        unsigned thr = (sev * ((y * 0x10000u) / b->rows)) / 16u;

        if (!bx_on(x, y)) continue;

        for (i = 0; i < len; i++)
        {
            st = xs32(st);
            if ((st & 0xffffu) >= thr) continue;
            row[i] = to_one ? (unsigned char)(row[i] | lanes)
                            : (unsigned char)(row[i] & ~lanes);
        }
    }
}

// A bus that is not terminated reflects, and what comes back arrives late and
// lands on top of what is being driven now. The delay reaches back past the
// start of the row into the one before it, because the scan does not stop at
// the row boundary and neither does the reflection.
static void bx_echo(const bendx_t *x, const bendx_buf_t *b,
                    unsigned char *s0, unsigned char *s1)
{
    unsigned char lanes = bx_lanes(x);
    unsigned len = b->rowlen;
    unsigned d = (unsigned)x->amount + 1u + (unsigned)x->reach * 64u;
    unsigned char *cur = s0, *prev = s1;
    int have_prev = 0;
    unsigned y, i;

    if (d > len) d = len;

    for (y = 0; y < b->rows; y++)
    {
        BX_TICK(y);
        unsigned char *row = b->base + (unsigned)y * len;
        unsigned char *t;

        for (i = 0; i < len; i++) cur[i] = row[i];

        if (bx_on(x, y))
        {
            for (i = 0; i < len; i++)
            {
                unsigned char src;
                if (i >= d)            src = cur[i - d];
                else if (have_prev)    src = prev[len + i - d];
                else                   continue;   // nothing has been driven yet
                row[i] = bx_merge(row[i], src, lanes, x->mix);
            }
        }

        // cur holds this row as it arrived, which is what the next row's
        // reflection has to see - not the version we just wrote over it.
        t = prev; prev = cur; cur = t;
        have_prev = 1;
    }
}

// Line memory is one row deep and it is what everything downstream reads. Stop
// clocking it and it keeps presenting the last row it latched, for as long as
// the fault lasts.
static void bx_linemem(const bendx_t *x, const bendx_buf_t *b)
{
    unsigned char lanes = bx_lanes(x);
    unsigned len = b->rowlen;
    unsigned period = (unsigned)x->amount + 2u;
    unsigned hold   = (unsigned)x->reach + 1u;
    unsigned y, i;

    if (hold >= period) hold = period - 1u;

    for (y = 0; y < b->rows; y++)
    {
        BX_TICK(y);
        unsigned char *row, *src;
        unsigned pos = y % period;
        unsigned sy;

        if (pos < period - hold) continue;      // still clocking normally
        if (!bx_on(x, y)) continue;

        sy = y - pos + (period - hold) - 1u;    // the last row it latched
        row = b->base + (unsigned)y  * len;
        src = b->base + (unsigned)sy * len;
        for (i = 0; i < len; i++) row[i] = bx_merge(row[i], src[i], lanes, x->mix);
    }
}

// The two fields of an interlaced read arrive in an order the timing generator
// decides. Invert the phase and the block comes back bottom-up; at a block of
// two that is the classic odd/even swap, and at larger blocks it is what a
// field buffer read backwards looks like.
static void bx_field(const bendx_t *x, const bendx_buf_t *b, unsigned char *s0)
{
    unsigned char lanes = bx_lanes(x);
    unsigned len = b->rowlen;
    unsigned blk = (unsigned)x->amount + 2u;
    unsigned y0, k, i;

    for (y0 = 0; y0 < b->rows; y0 += blk)
    {
        unsigned n = (y0 + blk <= b->rows) ? blk : (b->rows - y0);

        if (!bx_on(x, y0)) continue;

        for (k = 0; k < n / 2u; k++)
        {
            unsigned char *ra = b->base + (unsigned)(y0 + k)         * len;
            unsigned char *rb = b->base + (unsigned)(y0 + n - 1 - k) * len;
            for (i = 0; i < len; i++) s0[i] = ra[i];
            for (i = 0; i < len; i++)
            {
                unsigned char a = s0[i], c = rb[i];
                ra[i] = bx_merge(a, c, lanes, x->mix);
                rb[i] = bx_merge(c, a, lanes, x->mix);
            }
        }
    }
}

//-------------------------------------------------------------------
// Foreign buses.
//
// Something else in the camera driving the frame buffer's data lanes. On a
// bench this is a probe left on the wrong pad; here it is a second buffer read
// straight through, at whatever address it happens to live.
//
// Read-only on the far side, always, and bounded by the length the caller
// declared. raw.c derives all three lengths from accessors the port already
// has, so this file never contains an address.

// A flat slab of memory read straight through. Used for the program ROM and
// for the JPEG encoder's buffer, which want identical treatment and produce
// completely different pictures: one is ARM code and Canon's string tables,
// the other is entropy-coded image data, and they look nothing like each other
// on the data lanes.
static void bx_membus(const bendx_t *x, const bendx_buf_t *b,
                      const unsigned char *mem, unsigned mem_len)
{
    unsigned char lanes = bx_lanes(x);
    unsigned len = b->rowlen;
    unsigned shift = x->amount;             // repeat each byte 2^shift
    unsigned span  = (len >> shift) + 2u;   // source bytes one row consumes
    unsigned walk  = ((unsigned)x->reach + 1u) * 61u;
    unsigned window, off, y, i;

    if (!mem || mem_len <= span) return;

    window = mem_len - span;
    off = x->seed % window;

    for (y = 0; y < b->rows; y++)
    {
        BX_TICK(y);
        unsigned char *row = b->base + (unsigned)y * len;
        const unsigned char *r = mem + (off + (unsigned)y * walk) % window;

        if (!bx_on(x, y)) continue;
        for (i = 0; i < len; i++)
            row[i] = bx_merge(row[i], r[i >> shift], lanes, x->mix);
    }
}

//-------------------------------------------------------------------
// Pixel-domain effects.
//
// These unpack, so they cost what the bussed path in raw.c costs - which on
// these bodies is the difference between a shot that saves and a shot you wait
// out. bendx_is_slow() reports it and the picker marks it; none of them can be
// expressed on the packed bytes, because the packing is not in scan order.

// The pixel clock is what says where one pixel ends and the next begins. Run
// it at the wrong rate against a fixed line length and the row stretches or
// compresses, and since the rate is never wrong by a constant, it wobbles.
static void bx_clock(const bendx_t *x, const bendx_buf_t *b, unsigned char *s0)
{
    unsigned mask = bx_planes(x, b->nbits);
    unsigned len = b->rowlen;
    unsigned err = ((unsigned)x->amount + 1u) * 128u;
    unsigned period = 1u << (x->reach + 2);
    unsigned npix = bx_wholepix(b);
    unsigned y, i;

    if (!npix) return;

    for (y = 0; y < b->rows; y++)
    {
        BX_TICK(y);
        unsigned char *row = b->base + (unsigned)y * len;
        unsigned ratio;
        unsigned pos;

        if (!bx_on(x, y)) continue;

        // Triangle over `period` rows, so the rate drifts one way and back
        // rather than jumping - a PLL hunting, not a PLL switched. period is
        // 1 << (reach + 2), so its half is never zero.
        pos = y % period;
        if (pos >= period / 2u) pos = period - pos - 1u;
        ratio = 65536u + (err * pos * 2u) / (period / 2u);

        for (i = 0; i < len; i++) s0[i] = row[i];

        for (i = 0; i < npix; i++)
        {
            // i * ratio stays inside 32 bits: 3720 pixels by at most 67584.
            unsigned sx = (i * ratio) >> 16;
            unsigned v;
            sx %= npix;                 // the row folds back rather than
                                        // smearing its last pixel
            v = bendx_get(s0, sx, b->nbits);
            bendx_set(row, i, bx_mixv(bendx_get(s0, i, b->nbits), v, mask, x->mix),
                      b->nbits);
        }
    }
}

// Canon's demosaic assumes a fixed Bayer phase. Move the image under it by an
// odd number of pixels or rows and every red site is read as green - the
// geometry is untouched and the colour is destroyed, which is a thing only a
// CFA can do.
static void bx_cfa(const bendx_t *x, const bendx_buf_t *b, unsigned char *s0)
{
    unsigned mask = bx_planes(x, b->nbits);
    unsigned len = b->rowlen;
    unsigned dx = x->amount, dy = x->reach;
    unsigned npix = bx_wholepix(b);
    unsigned y, i;

    if (!dx && !dy) dx = 1;             // a phase shift of nothing is not one
    if (!npix || !b->rows) return;

    for (y = 0; y < b->rows; y++)
    {
        BX_TICK(y);
        unsigned char *row = b->base + (unsigned)y * len;
        unsigned sy = y + dy;
        const unsigned char *src;

        if (!bx_on(x, y)) continue;

        // Ascending, and the source is always at or below us, so it has not
        // been rewritten yet. Past the last row there is nothing to read, so
        // the bottom dy rows keep their own phase.
        if (sy >= b->rows) continue;
        if (dy == 0)
        {
            for (i = 0; i < len; i++) s0[i] = row[i];
            src = s0;
        }
        else
            src = b->base + (unsigned)sy * len;

        for (i = 0; i < npix; i++)
        {
            unsigned sx = (i + dx) % npix;
            unsigned v = bendx_get(src, sx, b->nbits);
            bendx_set(row, i, bx_mixv(bendx_get(row, i, b->nbits), v, mask, x->mix),
                      b->nbits);
        }
    }
}

// The clamp loop measures the masked columns at the start of each row and
// subtracts what it finds, which is how a CCD's dark current is kept out of
// the picture. Feed it the measurement from a different row and it corrects
// for a row that is not there - so the correction becomes the signal, and it
// bands, and the banding tracks real dark current rather than a pattern.
static void bx_clamp(const bendx_t *x, const bendx_buf_t *b,
                     const bendx_env_t *env)
{
    unsigned mask = bx_planes(x, b->nbits);
    unsigned len = b->rowlen;
    unsigned vmax = (1u << b->nbits) - 1u;
    unsigned white = env->white ? env->white : vmax;
    unsigned gain = (unsigned)x->amount + 1u;
    unsigned away = ((unsigned)x->reach + 1u) * 8u;
    unsigned npix = bx_wholepix(b);
    unsigned y, i;

    if (b->ob_x >= npix || !b->rows) return;
    if (white > vmax) white = vmax;

    for (y = 0; y < b->rows; y++)
    {
        BX_TICK(y);
        unsigned char *row = b->base + (unsigned)y * len;
        unsigned oy = (y + away) % b->rows;
        unsigned ob = bendx_get(b->base + (unsigned)oy * len, b->ob_x, b->nbits);
        int corr;

        if (!bx_on(x, y)) continue;

        corr = (int)((ob * gain) >> 3) - (int)env->black;

        for (i = 0; i < npix; i++)
        {
            unsigned v = bendx_get(row, i, b->nbits);
            int d = (int)v - corr;
            if (d < 0) d = 0;
            if (d > (int)white) d = (int)white;
            bendx_set(row, i, bx_mixv(v, (unsigned)d, mask, x->mix), b->nbits);
        }
    }
}

//-------------------------------------------------------------------
// Sensor physics.
//
// Everything above this point breaks something between the sensor and the
// JPEG. Bloom breaks the sensor and is what you see on an old CCD pointed at
// something bright.

// Blooming.
//
// A well that fills past capacity spills into the vertical transfer register,
// and everything clocked through that register afterwards carries the spill.
// On a CCD that is the white column standing out of every specular highlight -
// the single most recognisable "this is not a CMOS sensor" artefact there is.
//
// One accumulator per column, carried down the frame, which fits in the two
// rows of scratch every effect already gets: a column short is two bytes and
// there are fewer pixels in a row than there are bytes in two of them.
static void bx_bloom(const bendx_t *x, const bendx_buf_t *b, unsigned char *s)
{
    unsigned mask = bx_planes(x, b->nbits);
    unsigned len = b->rowlen;
    unsigned npix = bx_wholepix(b);
    unsigned vmax = (1u << b->nbits) - 1u;
    unsigned short *acc = (unsigned short *)(void *)s;
    // Threshold walks down from the top of the range, so a low amount spills
    // only from what is genuinely clipped and a high one from anything bright.
    unsigned thr = vmax - ((vmax * (unsigned)x->amount) / 20u);
    unsigned decay = 200u + (unsigned)x->reach * 3u;    // /256 kept per row
    unsigned y, i;

    if (!npix) return;
    if (decay > 255u) decay = 255u;
    for (i = 0; i < npix; i++) acc[i] = 0;

    for (y = 0; y < b->rows; y++)
    {
        BX_TICK(y);
        unsigned char *row = b->base + (unsigned)y * len;

        for (i = 0; i < npix; i++)
        {
            unsigned v = bendx_get(row, i, b->nbits);
            unsigned a = acc[i];
            unsigned o;

            if (v > thr) a += (v - thr);
            else         a = (a * decay) >> 8;
            if (a > vmax) a = vmax;
            acc[i] = (unsigned short)a;

            // The accumulator is carried whether or not this row is in scope,
            // so banding cuts holes in the smear rather than resetting it.
            if (!bx_on(x, y)) continue;

            o = v + a;
            if (o > vmax) o = vmax;
            bendx_set(row, i, bx_mixv(v, o, mask, x->mix), b->nbits);
        }
    }
}

//-------------------------------------------------------------------
// Processes.
//
// Pixel sorting is not a hardware failure. It is here because it looks good,
// and the honest place to say so is here rather than in a hardware story
// invented to justify it.
//
// Both are Bayer-aware in a way the fault effects are not, and for a reason
// that is not aesthetic: a fault moves charge or bits, and the mosaic goes
// along for the ride. A process moves *pixels*, and moving a pixel one place
// puts a red value on a green site - so Canon's demosaic, which runs after us
// and assumes a fixed phase, turns the result into colour mud rather than into
// the effect. It therefore moves in steps of two, which is the distance
// between neighbours of the same colour.

// Pixel sorting.
//
// Runs of pixels above a threshold, put in order along the row. Two passes per
// row, one per Bayer phase, stepping two at a time so red only ever sorts with
// red - see the note above.
//
// Counting sort, not insertion sort. Insertion is what every pixel sort is
// written with and it is quadratic; on a 3720 pixel row over 2772 rows that is
// thirteen billion comparisons, which is not slow, it is never. Counting sort
// is one pass to tally and one to write back, and the value range is only 4096
// wide because that is what the sensor produces.
static void bx_sort(const bendx_t *x, const bendx_buf_t *b, unsigned char *s)
{
    unsigned mask = bx_planes(x, b->nbits);
    unsigned len = b->rowlen;
    unsigned npix = bx_wholepix(b);
    unsigned vmax = (1u << b->nbits) - 1u;
    unsigned short *hist = (unsigned short *)(void *)s;
    unsigned thr = (vmax * (unsigned)x->amount) / 18u;
    // The low three bits set the shortest run worth sorting; the top bit turns
    // the sort over. One byte, two knobs, because there is no third field and
    // a sort that only runs one way up is half an effect.
    unsigned minrun = (((unsigned)x->reach & 7u) + 1u) * 8u;
    int down = (x->reach >= 8);
    unsigned phase, y, i;

    if (npix < 4) return;
    for (i = 0; i < BX_SORT_BUCKETS; i++) hist[i] = 0;

    for (y = 0; y < b->rows; y++)
    {
        BX_TICK(y);
        unsigned char *row = b->base + (unsigned)y * len;

        if (!bx_on(x, y)) continue;

        for (phase = 0; phase < 2; phase++)
        {
            unsigned i0 = phase;

            while (i0 < npix)
            {
                unsigned i1, n, lo, hi, v, w;

                // Find a run of same-phase pixels that are all above the
                // threshold. That mask is where nearly all of the look comes
                // from - sorting everything just gives a gradient.
                while (i0 < npix && bendx_get(row, i0, b->nbits) <= thr) i0 += 2;
                if (i0 >= npix) break;
                for (i1 = i0; i1 < npix && bendx_get(row, i1, b->nbits) > thr; i1 += 2) ;

                n = (i1 - i0) / 2u;
                if (n < minrun) { i0 = i1 + 2; continue; }

                lo = vmax; hi = 0;
                for (i = i0; i < i1; i += 2)
                {
                    v = bendx_get(row, i, b->nbits);
                    hist[v]++;
                    if (v < lo) lo = v;
                    if (v > hi) hi = v;
                }

                // Write back in order, and clear only the buckets this run
                // touched - clearing all 4096 per run is what would make this
                // quadratic again by the back door.
                w = i0;
                if (!down)
                {
                    for (v = lo; v <= hi; v++)
                        while (hist[v]--)
                        { unsigned o = bendx_get(row, w, b->nbits);
                          bendx_set(row, w, bx_mixv(o, v, mask, x->mix), b->nbits); w += 2; }
                }
                else
                {
                    for (v = hi; ; v--)
                    {
                        while (hist[v]--)
                        { unsigned o = bendx_get(row, w, b->nbits);
                          bendx_set(row, w, bx_mixv(o, v, mask, x->mix), b->nbits); w += 2; }
                        if (v == lo) break;
                    }
                }
                // hist[v]-- leaves every touched bucket at 0xffff, not 0.
                for (v = lo; v <= hi; v++) hist[v] = 0;

                i0 = i1 + 2;
            }
        }
    }
}

//-------------------------------------------------------------------
// Timing losing lock.
//
// BX_SLIP is a constant offset and BX_TEAR is one that grows. This is the
// third thing a clock recovery loop does when it cannot hold: it sits locked
// for a while, slips to somewhere else, holds there, slips again. Discrete,
// unpredictable, and stable in between - which reads completely differently
// from either of the other two even though all three rotate a row.
static void bx_lock(const bendx_t *x, const bendx_buf_t *b,
                    unsigned char *s0, unsigned char *s1)
{
    unsigned char lanes = bx_lanes(x);
    unsigned len = b->rowlen;
    unsigned rate = (unsigned)x->amount + 1u;       // slips per hundred rows
    unsigned span = ((unsigned)x->reach + 1u) * 8u; // how far it lands
    unsigned st = xs32(x->seed);
    unsigned bits = 0;
    unsigned y, i;

    for (y = 0; y < b->rows; y++)
    {
        BX_TICK(y);
        unsigned char *row = b->base + (unsigned)y * len;

        st = xs32(st);
        if (st % 100u < rate)
        {
            st = xs32(st);
            bits = st % (span + 1u);
        }

        if (!bx_on(x, y) || !bits) continue;
        for (i = 0; i < len; i++) s0[i] = row[i];
        bx_rot(s1, s0, len, bits);
        bx_blend_row(row, s1, len, lanes, x->mix);
    }
}

//-------------------------------------------------------------------
// Highlight melt.
//
// Taken from photographs, not from a fault - the same footing as the sort.
// What the photographs show, and what each part of this is for:
//
//   - every highlight trails a streak straight down, the length varying
//     column to column, finely ragged at the bottom but following the shape
//     of the highlight above in the large - a pillar leaves a U-shaped tip;
//   - it is white over bright parts of the scene and pink over dark ones;
//   - it ends white, then a thin cyan line, then a thick, solid green band.
//
// The colours are made in the raw, not painted on. The streak carries red and
// blue at the highlight's level and green well below it, and each site keeps
// whichever is brighter, the streak or the scene. Canon's white balance does
// the rest: over a dark scene red and blue clip and green does not, which is
// pink; over a bright one the scene's own green fills in, which is white. At
// the end green comes up to full a little before red stops (white), red stops
// before blue (cyan), and green carries on after both (the green band).
//
// The streak is carried in cells two pixels wide, so both sites of a Bayer
// pair stay together and the mosaic is never split down the middle of a
// streak; demosaicing a half-streak is colour mud.
//
// Per cell: the level of the highlight that started it, how far down its own
// length it has got, and how far it moves per row. Positions are in units of
// the streak's length so that the shape is the same for a short streak and a
// long one, and so that there is one division per streak rather than one per
// pixel. That matters on a CPU with no divide instruction and six million
// pixels.

#define BXM_ONE     16384u          // a streak's whole length
#define BXM_TOP     (BXM_ONE / 6u)  // white fading to pink over this much
#define BXM_GPRE    (BXM_ONE / 10u) // green back to full this far before the end
#define BXM_GMID    4096u           // green in the body, of the highlight: 0.25
#define BXM_END     (BXM_ONE + BXM_ONE / 3u)

static unsigned bxm_body(unsigned u)
{
    // 1 - u^16: flat for nearly all of the length, then a hard end.
    unsigned w = (u * u) >> 14;
    w = (w * w) >> 14;
    w = (w * w) >> 14;
    w = (w * w) >> 14;
    return BXM_ONE - w;
}

// Length factor for a cell, /256: a smooth swell across groups of 32 cells so
// the ends follow broad shapes, plus a little per-cell rag.
static unsigned bxm_shape(unsigned seed, unsigned c)
{
    unsigned n = c >> 5, t = c & 31u;
    unsigned a = 176u + (xs32(seed ^ (n * 0x85ebca6bu)) & 127u);
    unsigned z = 176u + (xs32(seed ^ ((n + 1u) * 0x85ebca6bu)) & 127u);
    // Smoothstep between the two, /1024, so the swells are rounded rather
    // than straight-sided Vs.
    unsigned sm = (t * t * (96u - 2u * t)) >> 5;
    unsigned f = (a * (1024u - sm) + z * sm) >> 10;          // 0.69 .. 1.18
    return (f * (248u + (xs32(seed ^ (c * 0x9e3779b9u)) & 15u))) >> 8;
}

//-------------------------------------------------------------------
// Melt colours.
//
// A palette is six levels, /BXM_ONE of the highlight: the body's red, green
// and blue, then the end band's. Classic is the pair the reference photos
// show. The other two modes take a pair of complementary colours from the
// photograph being bent: the band is a colour found in it and the body is
// that colour's complement, so pink-and-green is just what a green scene
// gives - which is also what they fall back to when the scene has no colour.
//
// The colour is measured from a sparse sample of the frame before anything
// is drawn, in a grid of blocks BXM_GW cells wide and BXM_GH deep. Colour in
// the raw is meaningless on its own - green always reads high, because the
// sensor is most sensitive to it - so every block is judged against a white
// reference: the sensor's daylight white from the port's DNG colour matrix
// (env->neutral), or failing that the frame's average weighted by
// brightness. Never the plain average: a frame that is half blue sky averages
// blue, and then the sky counts as grey. The same reference turns the chosen
// colour back into raw levels, so that after Canon's white balance it comes
// out roughly as chosen - roughly, because Canon balances for the light it
// thinks it saw, and in shade or indoors that is not daylight.
//
//   Image: one pair for the whole frame. The band is the frame's strongest
//          colour - the blocks' colours, each weighted by how saturated it is -
//          and the body its complement. Blue sky gives orange streaks with a
//          blue end; a red wall, cyan with a red end.
//   Local: each block's own colour, so a streak changes colour as it runs
//          down: orange over sky, magenta over trees. Where a block is grey
//          the frame's colour fills in, so neutral areas do not go white,
//          and a dark block counts for less, because its colour is mostly
//          noise. Blocks are blended into each other, never stepped.

#define BXM_GW      64u             // palette block width, in cells
#define BXM_GH      16u             // palette blocks down the frame
#define BXM_SAT     512             // full chroma, as the L1 norm of a /256 deviation
#define BXM_LGAIN   3               // how much a block's own colour is pushed

static const unsigned short bxm_classic[6] = {
    BXM_ONE, BXM_GMID, BXM_ONE,     // body: red and blue clip, green low - pink
    0,       BXM_ONE,  0,           // band: green alone
};

static unsigned bxm_grid_cols(unsigned ncell)
{
    return ncell / BXM_GW + 1u;
}

static int bxm_abs(int v) { return (v < 0) ? -v : v; }

static int bxm_l1(const int *c)
{
    return bxm_abs(c[0]) + bxm_abs(c[1]) + bxm_abs(c[2]);
}

// A chroma deviation (sums to ~0, L1 <= BXM_SAT) to a palette: band toward
// it, body away from it, each scaled to full, then into raw levels.
static void bxm_pair(const int *c, const unsigned *m, unsigned short *pal)
{
    unsigned mmax = m[0];
    int side, k;

    if (m[1] > mmax) mmax = m[1];
    if (m[2] > mmax) mmax = m[2];

    for (side = 0; side < 2; side++)
    {
        int col[3], top = 1;

        for (k = 0; k < 3; k++)
        {
            col[k] = 256 + (side ? c[k] : -c[k]);
            if (col[k] < 0) col[k] = 0;
            if (col[k] > top) top = col[k];
        }
        for (k = 0; k < 3; k++)
        {
            unsigned want = ((unsigned)col[k] * 256u) / (unsigned)top;   // 0 .. 256
            pal[side * 3 + k] = (unsigned short)((want * 64u * m[k]) / mmax);
        }
    }
}

// A block's colour: each channel divided by white's, then as a /256
// deviation from their mean, so it sums to about zero and does not depend on
// how bright the block is. Scaled down when the block is dark - below half of
// gref, the frame's typical brightness, its colour is increasingly noise.
// Returns 0 if the block had too few samples to say.
static int bxm_chroma(const unsigned *blk, const unsigned *n, unsigned gref, int *d)
{
    unsigned br = blk[3] & 0xffffu, bb = blk[3] >> 16;
    unsigned bg, conf;
    int r[3], avg, k;

    if (br < 4u || bb < 4u) return 0;
    bg = blk[1] / (br + bb);
    r[0] = (int)(((blk[0] / br) * 1024u) / n[0]);
    r[1] = (int)((bg * 1024u) / n[1]);
    r[2] = (int)(((blk[2] / bb) * 1024u) / n[2]);
    avg = (r[0] + r[1] + r[2]) / 3;
    if (avg <= 0) return 0;
    conf = (bg * 512u) / (gref ? gref : 1u);
    if (conf > 256u) conf = 256u;
    for (k = 0; k < 3; k++)
    {
        d[k] = ((r[k] - avg) * 256) / avg;
        if (d[k] >  512) d[k] =  512;
        if (d[k] < -512) d[k] = -512;
        d[k] = (d[k] * (int)conf) / 256;
    }
    return 1;
}

// Sample the frame and fill the grid with palettes. The grid is BXM_GH rows
// of gx blocks, eight shorts a block, then one more block for the frame's
// own pair. Returns 0 if the frame is too dark to measure; the caller then
// uses Classic.
static int bxm_palette(const bendx_t *x, const bendx_buf_t *b,
                       const bendx_env_t *env, unsigned ncell, unsigned thr,
                       unsigned black, unsigned *g, unsigned gx)
{
    unsigned nblk = gx * BXM_GH;
    // Brightness-weighted sums for the white reference: red times green on
    // red rows, green squared, blue times green on blue rows, and the green
    // weights on each row phase. All /1024, so a whole frame fits.
    unsigned sr = 0, sg = 0, sb = 0, wr = 0, wb = 0;
    unsigned m[3], wt[3], y, c, i;
    int dom[3] = { 0, 0, 0 };
    int k;

    for (i = 0; i < (nblk + 1u) * 4u; i++) g[i] = 0;

    // Two rows in eight, so both Bayer row phases are seen, and every other
    // cell. Highlights are left out - they are what streaks, not the scene.
    for (y = 0; y < b->rows; y++)
    {
        BX_TICK(y);
        const unsigned char *row;
        unsigned gk, by;
        int red;

        if ((y & 7u) >= 2u) continue;
        row = b->base + (unsigned)y * b->rowlen;
        gk  = ((y & 1u) ^ (env->g_even ? 0u : 1u));
        red = ((y & 1u) == (env->r_odd ? 1u : 0u));
        by  = (y * BXM_GH) / b->rows;

        for (c = 0; c < ncell; c += 2u)
        {
            unsigned v[2], gv, nv, *blk;

            v[0] = bendx_get(row, c * 2u,      b->nbits);
            v[1] = bendx_get(row, c * 2u + 1u, b->nbits);
            if (v[0] > thr || v[1] > thr) continue;
            if (v[gk] <= black + 2u) continue;

            gv = (v[gk] - black) >> 2;
            nv = (v[gk ^ 1u] > black) ? (v[gk ^ 1u] - black) >> 2 : 0u;
            blk = g + 4u * (by * gx + c / BXM_GW);
            blk[1] += gv;
            if (red) { blk[0] += nv; blk[3] += 1u;       sr += (nv * gv) >> 10; wr += gv; }
            else     { blk[2] += nv; blk[3] += 0x10000u; sb += (nv * gv) >> 10; wb += gv; }
            sg += (gv * gv) >> 10;
        }
    }

    // The white reference, /1024 like the sums. Too dark to say anything
    // and the caller falls back to Classic.
    wr >>= 10; wb >>= 10;
    if (wr < 4u || wb < 4u) return 0;
    m[0] = sr / wr;
    m[1] = sg / (wr + wb);
    m[2] = sb / wb;
    if (m[0] < 2u || m[1] < 2u || m[2] < 2u) return 0;

    // White: the sensor's own daylight white if the port knows it, else what
    // was just measured.
    if (env->neutral[0] && env->neutral[1] && env->neutral[2])
        for (k = 0; k < 3; k++) wt[k] = env->neutral[k];
    else
        for (k = 0; k < 3; k++) wt[k] = m[k];

    // The frame's strongest colour: every block's, weighted by how saturated
    // it is.
    for (i = 0; i < nblk; i++)
    {
        int d[3], w;

        if (!bxm_chroma(g + i * 4u, wt, m[1], d)) continue;
        // One wildly coloured block - a sign, a lamp - must not outvote the
        // frame, and must not overflow the sum.
        w = bxm_l1(d);
        if (w > 1024) { for (k = 0; k < 3; k++) d[k] = d[k] * 1024 / w; w = 1024; }
        w /= 16;
        for (k = 0; k < 3; k++) dom[k] += d[k] * w;
    }

    // Normalised to full chroma - unless there is next to none, in which case
    // green, which makes the frame's pair the classic pink and green.
    while (bxm_l1(dom) > 65536) for (k = 0; k < 3; k++) dom[k] /= 2;
    if (bxm_l1(dom) < 64) { dom[0] = -128; dom[1] = 256; dom[2] = -128; }
    else { int n = bxm_l1(dom); for (k = 0; k < 3; k++) dom[k] = dom[k] * BXM_SAT / n; }

    bxm_pair(dom, wt, (unsigned short *)(void *)(g + nblk * 4u));
    if (x->tint != BXTINT_LOCAL) return 1;

    for (i = 0; i < nblk; i++)
    {
        unsigned *blk = g + i * 4u;
        int d[3] = { 0, 0, 0 }, n;

        if (bxm_chroma(blk, wt, m[1], d))
            for (k = 0; k < 3; k++) d[k] *= BXM_LGAIN;
        n = bxm_l1(d);
        if (n > BXM_SAT) { for (k = 0; k < 3; k++) d[k] = d[k] * BXM_SAT / n; n = BXM_SAT; }
        // Whatever saturation the block lacks, the frame's colour supplies.
        for (k = 0; k < 3; k++) d[k] += dom[k] * (BXM_SAT - n) / BXM_SAT;

        // The block's sums are read; its sixteen bytes become its palette.
        bxm_pair(d, wt, (unsigned short *)(void *)blk);
    }
    return 1;
}

//-------------------------------------------------------------------
// Sky only.
//
// What a painting effect may paint over when the subject is protected: the
// sky and the clouds in it, judged from one Bayer quad as it was before
// anything was painted - its green and one of red or blue from this row, the
// other from the row above (bx_melt keeps that per cell, unbent). Worked in
// linear light against the sensor's white, so neutral means neutral.
//
//   1  blue sky:  blue clearly above red and not below green, and not black
//   2  cloud:     bright, and close to neutral
//   0  anything else - which is the subject: bronze, skin, leaves, a lamp
//
// A colour test, not a picture of the subject, so it has the limits of one:
// a blue shirt counts as sky and a white wall as cloud. Against open sky -
// the picture this is for - those are rarely in the way.

static int bxm_sky(unsigned g, unsigned xv, unsigned prevx, int red,
                   unsigned black, const unsigned *inv, int *lin)
{
    unsigned rv = red ? xv : prevx, bv = red ? prevx : xv;
    int r  = (int)((((rv > black) ? rv - black : 0u) * inv[0]) >> 12);
    int gg = (int)((((g  > black) ? g  - black : 0u) * inv[1]) >> 12);
    int bb = (int)((((bv > black) ? bv - black : 0u) * inv[2]) >> 12);
    int mx, mn, lum;

    if (lin) { lin[0] = r; lin[1] = gg; lin[2] = bb; }
    // Blue by a margin as well as a ratio, and not near black: in the
    // shadows noise alone makes blue beat red half the time, and a black lamp
    // would count as sky.
    if (bb * 16 > r * 19 && bb * 16 >= gg * 15 && bb > 200 && bb - r > 48) return 1;

    mx = (r > gg) ? r : gg;  if (bb > mx) mx = bb;
    mn = (r < gg) ? r : gg;  if (bb < mn) mn = bb;
    lum = (2 * r + 5 * gg + bb) >> 3;
    if (lum > 1600 && (mx - mn) * 4 < mx) return 2;
    return 0;
}

// Blend the Local palette's two grid rows around row y into rowpal - the
// same as bx_melt does inline.
static void bxm_rowpal(unsigned y, unsigned rows, const unsigned *grid,
                       unsigned gx, unsigned short *rowpal)
{
    int fy = (int)((y * BXM_GH * 256u) / rows) - 128;
    unsigned by0 = (fy > 0) ? (unsigned)fy >> 8 : 0u;
    unsigned by1 = (by0 + 1u < BXM_GH) ? by0 + 1u : by0;
    unsigned t = (fy > 0) ? (unsigned)fy & 255u : 0u;
    const unsigned short *p0 = (const unsigned short *)(const void *)(grid + by0 * gx * 4u);
    const unsigned short *p1 = (const unsigned short *)(const void *)(grid + by1 * gx * 4u);
    unsigned c;

    for (c = 0; c < gx * 8u; c++)
        rowpal[c] = (unsigned short)((p0[c] * (256u - t) + p1[c] * t) >> 8);
}

//-------------------------------------------------------------------
// Sky rain.
//
// Thin streaks of flat colour falling out of the clouds - taken from a
// photograph, like Melt, and on the same footing. Where Melt drains every
// highlight in a curtain with a white top and a green end, this is sparse:
//
//   - a streak starts only inside a cloud, a few cells wide, one colour all
//     the way down - mostly the first colour, one in four the second;
//   - lengths are skewed short, so most are dashes and a few run far;
//   - starts cluster in curtains under some clouds and thin out elsewhere,
//     from a smooth density field across the frame;
//   - the clouds themselves pick up violet and pink blotches in their
//     mid-tones, where the reference's are.
//
// Per cell: which colour it is streaking (pk, 0 = none) and how many rows are
// left (pu). Starts are decided per unit of four cells and per eight rows, so
// the cells of one streak start together and it reads as one line.
//
//   amount  density, reach  length, tint  colours (Classic is the
//   reference's pink and periwinkle; Image and Local as for Melt), protect
//   sky only - on by default, so the subject stays clear.

static void bxm_rain(const bendx_t *x, const bendx_buf_t *b, const bendx_env_t *env,
                     unsigned ncell, unsigned lbase, unsigned black, unsigned vmax,
                     const unsigned *span, const unsigned *inv,
                     const unsigned short *frame_pal, int local,
                     const unsigned *grid, unsigned gx, unsigned short *rowpal,
                     unsigned short *pk, unsigned short *pu, unsigned short *pv)
{
    // Linear /4096: the reference's pink, then its periwinkle.
    static const unsigned short classic[6] = { 3277, 1147, 1720,  1352, 1352, 3523 };
    unsigned mask = bx_planes(x, b->nbits);
    int protect = (x->protect == BXPROT_SKY);
    unsigned dens = (unsigned)x->amount + 3u;
    int tinted = (frame_pal != bxm_classic);
    unsigned y, c;

    for (y = 0; y < b->rows; y++)
    {
        BX_TICK(y);
        unsigned char *row = b->base + y * b->rowlen;
        unsigned gk = ((y & 1u) ^ (env->g_even ? 0u : 1u));
        int red = ((y & 1u) == (env->r_odd ? 1u : 0u));
        int on = bx_on(x, y);

        if (local) bxm_rowpal(y, b->rows, grid, gx, rowpal);

        for (c = 0; c < ncell; c++)
        {
            unsigned i = c * 2u, v[2], prevx, k;
            int cls = -1, lin[3];

            v[0] = bendx_get(row, i,      b->nbits);
            v[1] = bendx_get(row, i + 1u, b->nbits);
            prevx = y ? pv[c] : v[gk ^ 1u];
            pv[c] = (unsigned short)v[gk ^ 1u];

            // A start, if this cell is idle, inside a cloud, and its unit's
            // draw for this eight-row slot comes up.
            if (!pk[c])
            {
                // Units of eight cells, a streak two to seven of them wide -
                // a few pixels on the finished picture, as in the reference.
                unsigned unit = c >> 3;
                unsigned hu = xs32(x->seed ^ (unit * 0x9e3779b9u) ^ ((y >> 3) * 0x85ebca6bu));
                unsigned w = 2u + ((hu >> 4) & 3u) + ((hu >> 6) & 2u);

                if ((c & 7u) < w)
                {
                    unsigned f = bxm_shape(x->seed ^ 0x2545f491u, unit);
                    unsigned thresh = (dens * ((f * f) >> 8) * 27u) >> 3;

                    if ((hu & 0xffffu) < thresh)
                    {
                        cls = bxm_sky(v[gk], v[gk ^ 1u], prevx, red, black, inv, lin);
                        if (cls == 2)
                        {
                            // Skewed short, up to twice the reach: mostly
                            // dashes, a few that fall a long way.
                            unsigned r = 40u + ((hu >> 16) & 215u);
                            unsigned L = (lbase * 2u * r * r) >> 16;
                            L = (L * (248u + (xs32(x->seed ^ (c * 0x27d4eb2du)) & 15u))) >> 8;
                            if (L < 4u) L = 4u;
                            if (L > 65535u) L = 65535u;
                            pk[c] = (unsigned short)((((hu >> 24) & 7u) >= 6u) ? 2u : 1u);
                            pu[c] = (unsigned short)L;
                        }
                    }
                }
            }

            if (pk[c])
            {
                int paint = on;

                if (paint && protect)
                {
                    if (cls < 0) cls = bxm_sky(v[gk], v[gk ^ 1u], prevx, red, black, inv, lin);
                    paint = (cls != 0);
                }

                if (paint)
                {
                    unsigned short pal[6];
                    unsigned base = (pk[c] == 2u) ? 3u : 0u;

                    if (!tinted)
                        for (k = 0; k < 6u; k++) pal[k] = classic[k];
                    else
                    {
                        const unsigned short *src = frame_pal;
                        unsigned short blend[6];
                        if (local)
                        {
                            int fx = (int)(c * (256u / BXM_GW)) - 128;
                            unsigned bx0 = (fx > 0) ? (unsigned)fx >> 8 : 0u;
                            unsigned bx1 = (bx0 + 1u < gx) ? bx0 + 1u : bx0;
                            unsigned t = (fx > 0) ? (unsigned)fx & 255u : 0u;
                            for (k = 0; k < 6u; k++)
                                blend[k] = (unsigned short)((rowpal[bx0 * 8u + k] * (256u - t)
                                                           + rowpal[bx1 * 8u + k] * t) >> 8);
                            src = blend;
                        }
                        for (k = 0; k < 6u; k++) pal[k] = src[k];
                    }

                    for (k = 0; k < 2u; k++)
                    {
                        unsigned ch = (k == gk) ? 1u : (red ? 0u : 2u);
                        unsigned o;

                        // Classic is linear light against the sensor's white.
                        // A palette is already raw, relative to a highlight;
                        // a rain colour is a flat level, four fifths of the
                        // range (span[1] is the whole range: green's white is
                        // the reference).
                        if (!tinted) o = black + ((pal[base + ch] * span[ch]) >> 12);
                        else         o = black + ((((pal[base + ch] * 3277u) >> 14) * span[1]) >> 14);
                        if (o > vmax) o = vmax;
                        bendx_set(row, i + k, bx_mixv(v[k], o, mask, x->mix), b->nbits);
                    }
                }

                if (--pu[c] == 0) pk[c] = 0;
                continue;
            }

            // Not streaking: the cloud tint, in blotches, strongest in the
            // cloud's mid-tones and gone in its brightest and darkest parts.
            if (!on) continue;
            if (cls < 0) cls = bxm_sky(v[gk], v[gk ^ 1u], prevx, red, black, inv, lin);
            if (cls == 2)
            {
                int lum = (2 * lin[0] + 5 * lin[1] + lin[2]) >> 3;
                unsigned hb = xs32(x->seed ^ ((c >> 3) * 0x68e31da4u) ^ ((y >> 4) * 0xb5297a4du));
                unsigned n = hb & 255u;
                int wgt, s;

                if (lum <= 800 || lum >= 4000 || n <= 140u) continue;
                wgt = (lum < 2400) ? ((lum - 800) * 256) / 1600 : ((4000 - lum) * 256) / 1600;
                s = ((int)(n - 140u) * 2 * wgt) >> 8;           // 0 .. 230

                for (k = 0; k < 2u; k++)
                {
                    int violet = (int)((hb >> 8) & 1u);
                    unsigned ch = (k == gk) ? 1u : (red ? 0u : 2u);
                    int f;
                    unsigned o;

                    if (ch == 1u)      f = 256 - ((s * 90) >> 8);
                    else if (ch == 0u) f = 256 + ((s * (violet ? 26 : 77)) >> 8);
                    else               f = 256 + ((s * (violet ? 77 : 31)) >> 8);

                    o = (v[k] > black) ? v[k] - black : 0u;
                    o = black + ((o * (unsigned)f) >> 8);
                    if (o > vmax) o = vmax;
                    bendx_set(row, i + k, bx_mixv(v[k], o, mask, x->mix), b->nbits);
                }
            }
        }
    }
}

static void bx_melt(const bendx_t *x, const bendx_buf_t *b,
                    const bendx_env_t *env, unsigned char *s)
{
    unsigned mask = bx_planes(x, b->nbits);
    unsigned len = b->rowlen;
    unsigned npix = bx_wholepix(b);
    unsigned ncell = npix / 2u;
    unsigned vmax = (1u << b->nbits) - 1u;
    unsigned white = env->white ? env->white : vmax;
    unsigned black = env->black;
    // See bendx_scratch_bytes: 4 * ncell shorts, then the palette grid.
    unsigned short *pk = (unsigned short *)(void *)s;   // level above black, 0 = idle
    unsigned short *pu = pk + ncell;                    // position, BXM_ONE = length
    unsigned short *du = pu + ncell;                    // step per row, 0 = in the source
    unsigned short *pv = du + ncell;                    // last row's red or blue, unbent
    unsigned *grid = (unsigned *)(void *)(s + ((8u * ncell + 3u) & ~3u));
    unsigned gx = bxm_grid_cols(ncell);
    const unsigned short *frame_pal = bxm_classic;
    // The blended row, after the grid and the frame's own pair.
    unsigned short *rowpal = (unsigned short *)(void *)(grid + (gx * BXM_GH + 1u) * 4u);
    int local = 0;
    int rain = (x->kind == BX_RAIN);
    int protect = (x->protect == BXPROT_SKY);
    unsigned span[3], inv[3];
    unsigned thr, lbase, y, c;
    int k0;

    if (!ncell) return;
    if (white > vmax) white = vmax;
    if (black >= white) black = 0;

    // amount 0 takes only what is clipped or nearly; 15 takes the top third.
    // Rain's amount is its density, and only the palette sampler reads thr
    // there - to leave the clipped highlights out of the scene's colour.
    thr = white - ((white - black) * (rain ? 1u : (unsigned)x->amount + 1u)) / 48u;
    // reach 0 is about a twentieth of the frame, 15 under a third.
    lbase = (b->rows * ((unsigned)x->reach + 3u)) / 64u;
    if (lbase < 8u) lbase = 8u;

    // Linear light /4096 per channel, the sensor's white at 4096 in each -
    // what the sky test and Rain's flat colours are worked in.
    for (k0 = 0; k0 < 3; k0++)
    {
        unsigned n = env->neutral[k0] ? env->neutral[k0] : 1000u;
        span[k0] = ((white - black) * n) / 1000u;
        if (!span[k0]) span[k0] = 1;
        inv[k0] = (1u << 24) / span[k0];
    }

    if (x->tint != BXTINT_CLASSIC
     && bxm_palette(x, b, env, ncell, thr, black, grid, gx))
    {
        frame_pal = (const unsigned short *)(const void *)(grid + gx * BXM_GH * 4u);
        local = (x->tint == BXTINT_LOCAL);
    }

    for (c = 0; c < ncell; c++) pk[c] = pu[c] = du[c] = pv[c] = 0;

    if (rain)
    {
        bxm_rain(x, b, env, ncell, lbase, black, vmax, span, inv,
                 frame_pal, local, grid, gx, rowpal, pk, pu, pv);
        return;
    }

    for (y = 0; y < b->rows; y++)
    {
        BX_TICK(y);
        unsigned char *row = b->base + (unsigned)y * len;
        // Which pixel of the cell is green on this row, and whether the other
        // one is red or blue.
        unsigned gk = ((y & 1u) ^ (env->g_even ? 0u : 1u));
        int red = ((y & 1u) == (env->r_odd ? 1u : 0u));
        int on = bx_on(x, y);

        // Local: blend the two grid rows whose centres this row lies between
        // into the row buffer, so colours change smoothly down the frame.
        if (local)
        {
            int fy = (int)((y * BXM_GH * 256u) / b->rows) - 128;
            unsigned by0 = (fy > 0) ? (unsigned)fy >> 8 : 0u;
            unsigned by1 = (by0 + 1u < BXM_GH) ? by0 + 1u : by0;
            unsigned t = (fy > 0) ? (unsigned)fy & 255u : 0u;
            const unsigned short *p0 = (const unsigned short *)(const void *)(grid + by0 * gx * 4u);
            const unsigned short *p1 = (const unsigned short *)(const void *)(grid + by1 * gx * 4u);

            for (c = 0; c < gx * 8u; c++)
                rowpal[c] = (unsigned short)((p0[c] * (256u - t) + p1[c] * t) >> 8);
        }

        for (c = 0; c < ncell; c++)
        {
            unsigned i = c * 2u;
            unsigned v[2], k, u, h, p, bu, rc;
            unsigned short pal[6];

            unsigned prevx;

            v[0] = bendx_get(row, i,      b->nbits);
            v[1] = bendx_get(row, i + 1u, b->nbits);

            // The other colour of this quad, from the row above as it was
            // before anything here painted it - so the sky test below sees
            // the scene, not a streak. Kept for every cell, every row.
            prevx = y ? pv[c] : v[gk ^ 1u];
            pv[c] = (unsigned short)v[gk ^ 1u];

            // In a highlight: (re)start the streak from here.
            if (v[0] > thr || v[1] > thr)
            {
                unsigned m = (v[0] > v[1]) ? v[0] : v[1];
                pk[c] = (unsigned short)(m - black);
                pu[c] = 0;
                du[c] = 0;
                continue;
            }
            if (!pk[c]) continue;

            // First row out of the highlight: fix this streak's length. A
            // highlight only just over the threshold runs half as far as one
            // that is clipped.
            if (!du[c])
            {
                unsigned room = white - thr + 1u;
                unsigned over = (unsigned)pk[c] + black;
                unsigned L;

                over = (over > thr) ? over - thr : 0u;
                L = (lbase * bxm_shape(x->seed, c)) >> 8;
                L = (L * (128u + (over * 128u) / room)) >> 8;
                if (L < 4u) L = 4u;
                p = (BXM_ONE + L / 2u) / L;
                du[c] = (unsigned short)(p ? p : 1u);
            }

            u = (unsigned)pu[c] + du[c];
            if (u >= BXM_END) { pk[c] = 0; continue; }
            pu[c] = (unsigned short)u;

            // Carried whether or not the row is in scope, as in the bloom.
            if (!on) continue;

            // Protected: the streak goes on down, but behind the subject.
            if (protect && !bxm_sky(v[gk], v[gk ^ 1u], prevx, red, black, inv, 0))
                continue;

            p = pk[c];
            h = xs32(x->seed ^ 0x5bd1e995u ^ (c * 0x9e3779b9u));
            // End band: twenty-four to fifty-five rows past the end, never
            // more than a third of the streak. Red stops two to five rows early.
            bu = du[c] * (24u + ((h >> 8) & 31u));
            if (bu > BXM_ONE / 3u) bu = BXM_ONE / 3u;
            rc = du[c] * (2u + ((h >> 16) & 3u));
            // Local takes the colour of where the streak is now, not where it
            // started, so it changes as it runs down; blended across the two
            // blocks whose centres this cell lies between.
            if (local)
            {
                int fx = (int)(c * (256u / BXM_GW)) - 128;
                unsigned bx0 = (fx > 0) ? (unsigned)fx >> 8 : 0u;
                unsigned bx1 = (bx0 + 1u < gx) ? bx0 + 1u : bx0;
                unsigned t = (fx > 0) ? (unsigned)fx & 255u : 0u;

                for (k = 0; k < 6u; k++)
                    pal[k] = (unsigned short)((rowpal[bx0 * 8u + k] * (256u - t)
                                             + rowpal[bx1 * 8u + k] * t) >> 8);
            }
            else
                for (k = 0; k < 6u; k++) pal[k] = frame_pal[k];

            for (k = 0; k < 2u; k++)
            {
                unsigned ch = (k == gk) ? 1u : (red ? 0u : 2u);
                unsigned un = (ch == 0u) ? u + rc : u;
                unsigned q = 0, o;

                // Body: white where it leaves the highlight, fading to the
                // body colour, with a hard end.
                if (un < BXM_ONE)
                {
                    unsigned base = pal[ch];
                    if (u < BXM_TOP)
                        base = BXM_ONE - (((BXM_ONE - base) * u * 6u) >> 14);
                    q = (base * bxm_body(un)) >> 14;
                }
                // End band, from a little before the body ends.
                if (u + BXM_GPRE >= BXM_ONE && u < BXM_ONE + bu && pal[3u + ch] > q)
                    q = pal[3u + ch];

                o = black + ((p * q) >> 14);
                if (o > vmax) o = vmax;
                if (o > v[k])
                    bendx_set(row, i + k, bx_mixv(v[k], o, mask, x->mix), b->nbits);
            }
        }
    }
}

//-------------------------------------------------------------------
// Aerochrome.
//
// Kodak Aerochrome III 1443 was a false-colour reversal film with three layers
// that see the wrong things: the one that ends up red is sensitive to near
// infrared, the one that ends up green to red, the one that ends up blue to
// green. Every layer is also blue-sensitive, so it was shot through a yellow
// filter that throws blue away. Living leaves reflect near infrared about
// three times as strongly as green, so foliage comes out magenta to red, a
// blue sky comes out deep blue, red things come out yellow and skin waxy.
//
// The A540 sees no infrared at all - there is a hot mirror over the sensor -
// so the infrared layer is an estimate, and that is the one honest limit of
// this: it is made from how much greener than red and blue each patch is,
// which is chlorophyll's visible signature. Green paint will glow like a
// tree; a dry or autumn leaf will not, though on film it would.
//
// Per Bayer quad, so every site has all three colours to work from: the quad
// is white balanced against the sensor's daylight white, put through the
// layer mapping, saturated - the film was punchy, and the dyes are what make
// it look like itself - and written back site by site. The greens keep their
// own difference from the quad's mean, which is most of the detail; red and
// blue are single per quad anyway.
//
//   amount  how strongly foliage glows: the infrared gain on greenness
//   reach   dye saturation
//   tint    the filter: yellow 12 as Kodak recommended, orange 21 and red 25
//           darkening the green-sensitive (blue-printing) layer, as they did

static void bx_aero(const bendx_t *x, const bendx_buf_t *b, const bendx_env_t *env)
{
    static const int filt[3] = { 256, 166, 90 };    // blue-printing layer, /256
    unsigned mask = bx_planes(x, b->nbits);
    unsigned len = b->rowlen;
    unsigned npix = bx_wholepix(b) & ~1u;
    unsigned vmax = (1u << b->nbits) - 1u;
    unsigned white = env->white ? env->white : vmax;
    unsigned black = env->black;
    unsigned span[3], inv[3];
    int gain = 48 + 8 * (int)x->amount;             // /16: 3 .. 10.5
    int sat  = 358 + 26 * (int)x->reach;            // /256: 1.4 .. 2.9
    int bf   = filt[(x->tint <= BXTINT_RED) ? x->tint : 0];
    unsigned y, i;
    int k;

    if (npix < 2u || b->rows < 2u) return;
    if (white > vmax) white = vmax;
    if (black >= white) black = 0;

    // Linear light /4096 per channel, with the sensor's white at 4096 in each.
    for (k = 0; k < 3; k++)
    {
        unsigned n = env->neutral[k] ? env->neutral[k] : 1000u;
        span[k] = ((white - black) * n) / 1000u;
        if (!span[k]) span[k] = 1;
        inv[k] = (1u << 24) / span[k];
    }

    for (y = 0; y + 1u < b->rows; y += 2u)
    {
        unsigned char *row[2];
        int ch[4];

        BX_TICK(y);
        if (!bx_on(x, y)) continue;
        row[0] = b->base + y * len;
        row[1] = row[0] + len;

        // The four sites of a quad, in order (0,0) (1,0) (0,1) (1,1): which
        // colour each is. The same for every quad in the row pair.
        for (k = 0; k < 4; k++)
        {
            unsigned sx = (unsigned)(k & 1), sy = y + (unsigned)(k >> 1);
            if ((((sx + sy) & 1u) ^ (env->g_even ? 1u : 0u)) != 0) ch[k] = 1;
            else ch[k] = ((sy & 1u) == (env->r_odd ? 1u : 0u)) ? 0 : 2;
        }

        for (i = 0; i < npix; i += 2u)
        {
            unsigned v[4];
            int lin[4], c[3] = { 0, 0, 0 }, o[3], gsum = 0, gn = 0, veg, sky, yl;

            for (k = 0; k < 4; k++)
            {
                v[k] = bendx_get(row[k >> 1], i + (unsigned)(k & 1), b->nbits);
                lin[k] = (int)((((v[k] > black) ? v[k] - black : 0u) * inv[ch[k]]) >> 12);
                if (ch[k] == 1) { gsum += lin[k]; gn++; }
                else c[ch[k]] = lin[k];
            }
            c[1] = gn ? gsum / gn : 0;

            // The layers. Red prints the infrared estimate, green prints red,
            // blue prints green through the filter.
            veg = c[1] - ((c[0] > c[2]) ? c[0] : c[2]);
            if (veg < 0) veg = 0;
            // Blue sky and water carry next to no infrared - scattered light
            // is short-wave, and water absorbs it - which is what makes them
            // the film's deep blue rather than violet. Blueness takes it away.
            sky = c[2] - ((c[0] > c[1]) ? c[0] : c[1]);
            if (sky < 0) sky = 0;
            o[0] = c[0] + ((gain * veg) >> 4) - sky;
            if (o[0] < 0) o[0] = 0;
            o[1] = c[0];
            o[2] = (c[1] * bf) >> 8;

            // Dye saturation, about the layer mix's own luminance.
            yl = (2 * o[0] + 5 * o[1] + o[2]) >> 3;
            for (k = 0; k < 3; k++)
            {
                o[k] = yl + (((o[k] - yl) * sat) >> 8);
                if (o[k] < 0) o[k] = 0;
                if (o[k] > 8191) o[k] = 8191;
            }

            for (k = 0; k < 4; k++)
            {
                int out = o[ch[k]];
                unsigned w;

                // Each green keeps its own offset from the quad's mean.
                if (ch[k] == 1) out += lin[k] - c[1];
                if (out < 0) out = 0;
                w = black + (((unsigned)out * span[ch[k]]) >> 12);
                if (w > vmax) w = vmax;
                bendx_set(row[k >> 1], i + (unsigned)(k & 1),
                          bx_mixv(v[k], w, mask, x->mix), b->nbits);
            }
        }
    }
}

//-------------------------------------------------------------------

void bendx_apply(const bendx_t *x, const bendx_buf_t *buf,
                 const bendx_env_t *env, unsigned char *scratch)
{
    unsigned char *s0, *s1;

    if (!x || !buf || !env || !scratch) return;
    if (!buf->base || !buf->rowlen || !buf->rows) return;
    if (buf->nbits != 10 && buf->nbits != 12) return;
    if (bendx_is_off(x)) return;

    s0 = scratch;
    s1 = scratch + buf->rowlen;

    switch (x->kind)
    {
    case BX_ENDIAN:  bx_endian(x, buf, s0, s1);                             break;
    case BX_SLIP:    bx_slip(x, buf, s0, s1);                               break;
    case BX_TEAR:    bx_tear(x, buf, s0, s1);                               break;
    case BX_ADDR:    bx_addr(x, buf, s0);                                   break;
    case BX_COLADDR: bx_coladdr(x, buf);                                    break;
    case BX_STRIDE:  bx_stride(x, buf, s0);                                 break;
    case BX_REFRESH: bx_refresh(x, buf);                                    break;
    case BX_ECHO:    bx_echo(x, buf, s0, s1);                               break;
    case BX_LINEMEM: bx_linemem(x, buf);                                    break;
    case BX_FIELD:   bx_field(x, buf, s0);                                  break;
    case BX_ROMBUS:  bx_membus(x, buf, env->rom, env->rom_len);             break;
    case BX_CLOCK:   bx_clock(x, buf, s0);                                  break;
    case BX_CFA:     bx_cfa(x, buf, s0);                                    break;
    case BX_CLAMP:   bx_clamp(x, buf, env);                                 break;
    case BX_BLOOM:   bx_bloom(x, buf, s0);                                  break;
    case BX_SORT:    bx_sort(x, buf, s0);                                   break;
    case BX_LOCK:    bx_lock(x, buf, s0, s1);                               break;
    case BX_JPGBUS:  bx_membus(x, buf, env->jpg, env->jpg_len);             break;
    case BX_MELT:    bx_melt(x, buf, env, s0);                              break;
    case BX_AERO:    bx_aero(x, buf, env);                                  break;
    case BX_RAIN:    bx_melt(x, buf, env, s0);                              break;
    default: break;
    }
}

#else

// The image recipe module is shared by every camera and imports these four
// read-only helpers through the global module export table. Cameras without
// the experimental engine still need resolvable exports even though they can
// neither run nor restore a profile chain. Report the only state those builds
// support: experimental off.
const char *bendx_name(int kind)
{
    (void)kind;
    return "Off";
}

void bendx_label(const bendx_t *x, char *buf)
{
    (void)x;
    buf[0] = 0;
}

void bendx_chain_sanitize(bendx_chain_t *c)
{
    c->n = 0;
}

int bendx_chain_count(const bendx_chain_t *c)
{
    (void)c;
    return 0;
}

#endif // CAM_BEND_EXPERIMENTAL
