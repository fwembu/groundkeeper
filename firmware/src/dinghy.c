/* SPDX-License-Identifier: GPL-3.0-only */
/* Dinghy: the smallest FM-1 firmware to build your own on. A sine wave on the keys, and that is all.
 * This is the file to change. Everything below it (platform.c) is done: updates, the USB rescue,
 * MIDI (USB, the TRS jack), USB audio, keys, knobs, LEDs, the screen, flash storage; the FM-1 and the SMK-37.
 *
 *   keys       play sines, 4 voices; the notes go out over USB-MIDI too, on the MIDI channel
 *   OCT- OCT+  the octave (kept in flash)
 *   PRESETS    the MIDI channel: OMNI (in: all, out: 1) or 1..16 (kept in flash)
 *   MASTER     the volume (platform.c)
 *   MIDI in    notes from USB or the TRS jack play the sines, with their velocity; a MIDI clock shows its tempo
 *              and blinks PLAY on the beat, START / STOP light it
 *   SMK-37     the keys with velocity, the outer keys too, the pads (white keys F3..G5, lit while they sound), the
 *              pitch wheel bends +-2 semitones
 *
 * Rules of the road (the chip has no FPU and no libgcc): integers only, no float, no 64-bit division.
 * tests/host.c runs this file on a computer: the screen as a PNG, the sound as a WAV.
 * What platform.c offers: docs/API.md. The chip's rules: docs/RULES.md. */
#define APP_NAME "GROUNDKEEPER"                /* the title, the rescue and the crash screen */
static void app_idle(void);
#define LCD_IDLE app_idle                       /* while the screen's DMA runs: MIDI in (platform.c) */
#include "platform.c"
#if BOARD_SMK37
#include "smk_wheel.h"
#endif

/* ---- the sound (runs in the audio ISR: keep it short, no flash, no screen) */
#define NV 4u                                   /* voices */
static struct {
    volatile uint32_t inc;                      /* phase step per frame: f * 2^32 / 44100 (unbent) */
    volatile int32_t gate;                      /* target level, Q15: 0 = released */
    int32_t amp;                                /* the level now: glides to gate, no clicks */
    uint32_t ph, age;
    uint8_t note;
} voice[NV];
static volatile uint32_t bend_q16 = 65536;      /* the pitch wheel: the steps times this / 65536 */

static int32_t sine(uint32_t ph)                /* a full cycle over 2^32 -> -32767..32767 (a refined parabola, ~0.1 %) */
{
    int32_t x = (int32_t)ph >> 16, ax = x < 0 ? -x : x, y, ay;
    y = (x * (32768 - ax)) >> 13;               /* 4x(1-|x|), Q15 */
    ay = y < 0 ? -y : y;
    y += ((((y * ay) >> 15) - y) * 7373) >> 15; /* + 0.225 (y|y| - y) */
    return y > 32767 ? 32767 : y < -32767 ? -32767 : y;
}

/* ---- Groundkeeper's voices (audio ISR; the decisions are in the main loop)
 * DV voices, each two oscillators a few cents apart (a slow beat) in one of five timbres, with its own slow "breath".
 * The main loop sets a voice's target pitch; here the pitch glides there over a second or two.
 * Two engines. DRONE: a voice that is part of the network grows in (ATTACK) and holds; one that is not withers away
 * (RELEASE). BLOOM: a voice sounds only when the main loop triggers it: it swells to its peak (ATTACK), then fades
 * (RELEASE), and leaves silence until the next one. */
#include "gk_tables.h"
#define DV 6u                                   /* the most voices the network can grow (density 6) */
static struct {
    volatile uint32_t tinc;                     /* target phase step (main loop writes) */
    uint32_t inc;                               /* the step now: glides to tinc */
    uint32_t pa, pb, lfo, vph;                  /* the oscillators' phases, the breath's, the vibrato's */
    volatile uint32_t lfo_inc, vib_inc;         /* breath and vibrato speeds, per block */
    volatile int32_t vgate;                     /* DRONE: 1 = part of the network (main loop writes) */
    volatile uint32_t trig;                     /* BLOOM: +1 = bloom now (main loop writes, after peak and the note) */
    volatile int32_t peak;                      /* BLOOM: how loud this bloom gets, Q15 */
    uint32_t tseen;                             /* (the last trig the audio saw) */
    int32_t vamp, glast, nlp;                   /* its level now (Q15), its gain at the end of the last block, MIST's breath */
    uint8_t stage;                              /* BLOOM: 1 rising, else falling */
    volatile uint8_t note, band;                /* the note, and its wavetable band */
} drone[DV];
#define NT 5u                                   /* timbres: MOSS, MIST, CHOIR, ORGAN, REED */
static volatile uint32_t tim_now;               /* the timbre (main loop writes) */
static uint32_t tim_prev, tim_xf = 32767;       /* the crossfade from the last one, Q15 (audio) */
static volatile uint32_t engine;                /* 0 DRONE, 1 BLOOM (main loop writes) */
static volatile uint32_t env_att = 63, env_rel = 218;   /* the envelope (ATT_INC, REL_K values; main loop writes) */
static volatile uint32_t wob_depth;             /* WOBBLE: Q16 of the pitch (main loop writes) */
static volatile int32_t drone_on;               /* 1: growing / sounding, 0: fading to silence */
static int32_t drone_lvl;                       /* the drone's level now, Q15: fades in and out */
static uint32_t wob_ph, aud_rng = 0x6C8E9CF5u;
static int32_t wob_fl;                          /* the wobble's random part (flutter) */
static int16_t noise[256];                      /* a block of white noise (MIST's breath) */
#define DRONE_GAIN 4600                         /* per voice, Q15: six voices with partials stay under full scale */
static const uint8_t DETUNE[NT] = {10, 10, 8, 11, 10};   /* the second oscillator: + inc >> this (10: 1.7 cents) */
static const uint16_t TGAIN[NT] = {32767, 30000, 40000, 37000, 39000};   /* level per timbre (by RMS, the bright ones a little under) */
static const uint16_t VIB[NT] = {0, 0, 150, 0, 90};   /* vibrato depth, Q16 of the pitch (CHOIR ~0.23 %, REED ~0.14 %) */

static uint32_t xs32(uint32_t *r) { uint32_t x = *r; x ^= x << 13; x ^= x >> 17; x ^= x << 5; return *r = x; }
static int32_t wt(const int16_t *t, uint32_t ph)   /* a wavetable, linearly interpolated */
{
    uint32_t i = ph >> 23, f = (ph >> 9) & 0x3FFFu;
    int32_t a = t[i];
    return a + (((t[i + 1u] - a) * (int32_t)f) >> 14);
}
static int32_t shape(uint32_t t, uint32_t pa, uint32_t pb, uint32_t band, int32_t *nlp, int32_t nz)   /* ~+-32767 */
{
    switch (t) {
    case 1u: {                                  /* MIST: the pair an octave up, the fundamental faint, breath around it */
        int32_t s = (((sine(pa << 1) + sine(pb << 1)) * 13107) >> 15) + ((sine(pa) * 5461) >> 15) + (sine(pa * 3u) >> 4);
        *nlp += ((nz - *nlp) * 6000) >> 15;     /* noise below ~1.4 kHz ... */
        return s + ((*nlp * sine(pa << 1)) >> 14);   /* ... carried up to the octave: a breathy band around the tone */
    }
    case 2u: return (wt(CHOIR_WT[band], pa) + wt(CHOIR_WT[band], pb)) >> 1;   /* CHOIR: an "ah", two singers */
    case 3u: return (wt(ORGAN_WT, pa) + wt(ORGAN_WT, pb)) >> 1;                /* ORGAN: drawbars */
    case 4u: return (wt(REED_WT[band], pa) + wt(REED_WT[band], pb)) >> 1;     /* REED: a muted oboe */
    default: return (sine(pa) + sine(pb) + (sine(pa << 1) >> 2)) >> 1;        /* MOSS: two sines and a quiet octave */
    }
}

static void drone_audio(int32_t *bus, uint32_t n)   /* adds the voices, mono, to bus */
{
    uint32_t v, i, eng = engine, att = env_att, rel = env_rel, tn = tim_now, tp;
    int32_t lvl = drone_lvl, wob;
    if (drone_on) { lvl += 48; if (lvl > 32767) lvl = 32767; }   /* fade in ~4 s */
    else { lvl -= 32; if (lvl < 0) lvl = 0; }                     /* fade out ~6 s */
    drone_lvl = lvl;
    if (tn != tim_prev && tim_xf == 32767) tim_xf = 0;   /* (prev: the old one, now: the new one) */
    if (tim_xf < 32767) {                       /* a new timbre: crossfade over ~0.2 s */
        tim_xf += 1024;
        if (tim_xf >= 32767) { tim_xf = 32767; tim_prev = tn; }
    }
    tp = tim_prev;
    if (!lvl) return;                           /* silent: costs nothing */
    wob_ph += 13710000u;                        /* WOBBLE: a wow at ~0.55 Hz and a slow random flutter */
    wob_fl += ((int32_t)(xs32(&aud_rng) >> 17) - 16384 - wob_fl) >> 4;
    wob = (int32_t)(((int64_t)((sine(wob_ph) * 21299 >> 15) + wob_fl) * (int32_t)wob_depth) >> 15);   /* Q16 */
    if (tn == 1u || tp == 1u)
        for (i = 0; i < n; i++) noise[i] = (int16_t)(xs32(&aud_rng) >> 16);
    for (v = 0; v < DV; v++) {
        uint32_t pa = drone[v].pa, pb = drone[v].pb, inc = drone[v].inc, tinc = drone[v].tinc, incm, incb, band = drone[v].band;
        int32_t breath, g, g0 = drone[v].glast, step, va = drone[v].vamp, va0 = va, mod;
        if (eng == 0u) {                        /* DRONE: grow in, hold; wither when out of the network */
            if (drone[v].vgate) { va += (int32_t)att; if (va > 32767) va = 32767; }
            else va -= ((va * (int32_t)rel) >> 15) + 1;
        } else {                                /* BLOOM: rise to the peak, then fall */
            if (drone[v].trig != drone[v].tseen) { drone[v].tseen = drone[v].trig; drone[v].stage = 1; inc = tinc; }
            if (drone[v].stage == 1u) {
                va += (int32_t)att;
                if (va >= drone[v].peak) { va = drone[v].peak; drone[v].stage = 0; }
            } else va -= ((va * (int32_t)rel) >> 15) + 1;
        }
        if (va < 0) va = 0;
        drone[v].vamp = va;
        if (!va && !va0) { drone[v].inc = tinc; drone[v].glast = 0; continue; }   /* silent: no glide when it sounds again */
        inc += (uint32_t)((int32_t)(tinc - inc) >> 7);   /* glide: ~1.5 s to settle */
        if ((int32_t)(tinc - inc) > -128 && (int32_t)(tinc - inc) < 128) inc = tinc;
        drone[v].inc = inc;
        drone[v].vph += drone[v].vib_inc;       /* the pitch: the glide, the wobble, the timbre's vibrato */
        mod = wob + ((sine(drone[v].vph) * (int32_t)VIB[tn]) >> 15);
        incm = inc + (uint32_t)(int32_t)(((int64_t)inc * mod) >> 16);
        incb = incm + (incm >> DETUNE[tn]);
        drone[v].lfo += drone[v].lfo_inc;
        breath = 24576 + ((sine(drone[v].lfo) * 8192) >> 15);   /* 0.5 .. 1.0 */
        g = (((((((DRONE_GAIN * breath) >> 15) * (int32_t)TGAIN[tn]) >> 15) * lvl) >> 15) * va) >> 15;
        if (eng) g <<= 1;                       /* BLOOM: a voice or two at a time, so each twice as loud */
        step = (g - g0) / (int32_t)n;           /* the gain ramps across the block: no steps, even on a fast attack */
        for (i = 0; i < n; i++) {
            int32_t s;
            pa += incm;
            pb += incb;
            s = shape(tn, pa, pb, band, &drone[v].nlp, noise[i]);
            if (tim_xf < 32767)
                s = (s * (int32_t)(tim_xf >> 1) + shape(tp, pa, pb, band, &drone[v].nlp, noise[i]) * (int32_t)((32767u - tim_xf) >> 1)) >> 14;
            g0 += step;
            bus[i] += (s * g0) >> 15;
        }
        drone[v].glast = g;
        drone[v].pa = pa;
        drone[v].pb = pb;
    }
}

/* ---- the sound around the drone (audio ISR): a resonant low-pass, then a reverb, then dry + wet out in stereo
 * FILTER: a state-variable low-pass (Chamberlin), run twice a sample so it stays stable up to 12 kHz; its states carry
 * 4 extra bits so the low cutoffs stay smooth. RES: its Q, with the input turned down as the resonance rises.
 * REVERB: four delay lines (50..95 ms) that feed back into each other through a Hadamard matrix, each with a soft
 * low-pass (a forest, not a tiled room), after two allpasses that blur the attack; L and R take different lines.
 * SPACE sets the feedback (a decay of 0.6 s .. 16 s), MIX dry against wet. The coefficients glide (no zipper noise),
 * and a soft limiter keeps a resonant peak from clipping. The tail runs on ~20 s after the drone fades, then rests. */
#ifdef DINGHY_HOST
#define POOL
#else
#define POOL __attribute__((section(".pool")))   /* (the pool: big buffers, zeroed at boot) */
#endif
/* (computed on a computer, tools in the commit message: no float on the chip) */
static const uint16_t CUT_F[25] = {187, 227, 275, 335, 406, 494, 600, 728, 885, 1074, 1305, 1585, 1925, 2338, 2839, 3448, 4187, 5084, 6172, 7491, 9090, 11023, 13359, 16171, 27167};   /* 2 sin(pi fc / 2FS), Q15: 80 Hz .. 7 kHz in 24 steps, then OPEN (12 kHz) */
static const char *const CUT_LABEL[25] = {"80", "100", "120", "140", "170", "210", "260", "310", "380", "460", "560", "680", "820", "1.0k", "1.2k", "1.5k", "1.8k", "2.2k", "2.6k", "3.2k", "3.9k", "4.7k", "5.8k", "7.0k", "OPEN"};
static const uint16_t RES_Q[16] = {46348, 40681, 35707, 31342, 27510, 24146, 21194, 18603, 16328, 14332, 12580, 11041, 9691, 8507, 7466, 6554};   /* 1/Q, Q15: Q 0.71 .. 5 */
static const uint16_t RES_COMP[16] = {32768, 31676, 30518, 29297, 28020, 26694, 25329, 23935, 22522, 21103, 19689, 18293, 16926, 15598, 14318, 13094};   /* input gain, Q15: the resonance's boost taken back */
static const uint16_t SPACE_G[16] = {14228, 16764, 19125, 21260, 23148, 24785, 26183, 27364, 28351, 29169, 29844, 30398, 30850, 31218, 31517, 31759};   /* feedback, Q15: a decay of 0.6 s .. 16 s */
static const char *const SPACE_LABEL[16] = {"0.6s", "0.7s", "0.9s", "1.2s", "1.4s", "1.8s", "2.2s", "2.8s", "3.5s", "4.3s", "5.4s", "6.7s", "8.3s", "10s", "13s", "16s"};

#define NCUT 25u
#define NRES 16u
#define NSPACE 16u
#define NMIX 21u                                /* 0 .. 100 % in 5 % steps */
static volatile uint32_t fx_cut = NCUT - 1u, fx_res, fx_space = 8, fx_mix = 5;   /* (main loop writes) */
static int32_t f_now = 27167, q_now = 46348, c_now = 32768, g_now, m_now;   /* the gliding coefficients */
static int32_t sv_lp, sv_bp;                    /* the filter's states (x16) */
#define RV0 2203u
#define RV1 2851u
#define RV2 3517u
#define RV3 4211u
#define AP0 142u
#define AP1 379u
static int16_t rv0[RV0] POOL, rv1[RV1] POOL, rv2[RV2] POOL, rv3[RV3] POOL, ap0[AP0] POOL, ap1[AP1] POOL;
static uint32_t ri0, ri1, ri2, ri3, ai0, ai1, rv_rest;
static int32_t dm0, dm1, dm2, dm3;              /* the lines' damping states */
#define RV_DAMP 18022                           /* the lines' low-pass, Q15: ~5.6 kHz */
#define RV_DIFF 20480                           /* the allpasses' gain, Q15: 0.625 */
#define RV_TAIL (44100u * 20u)                  /* frames the reverb runs on after its input goes quiet */

/* TEXTURE (FX 2), after the reverb: HISS, a noise coloured by TONE (a one-pole low-pass below 0, a high-pass above),
 * L and R apart; CRACKLE, little clicks at random, each panned at random; both follow the drone's fade. */
static volatile uint32_t tex_hiss, tex_tone = 5, tex_crk;   /* (main loop writes) */
static uint32_t tex_rng = 0x9E3779B9u;
static int32_t tx_l, tx_r, ck_l, ck_r;          /* the noise filters' states, the crackle now */

static int32_t sat16(int32_t x) { return x > 32767 ? 32767 : x < -32767 ? -32767 : x; }
static int32_t glide(int32_t now, int32_t to) { int32_t d = to - now; return d > -8 && d < 8 ? to : now + (d >> 3); }
static int32_t soft(int32_t x)                  /* above 3/4 of full scale, a quarter as steep */
{
    return x > 24576 ? 24576 + ((x - 24576) >> 2) : x < -24576 ? -24576 + ((x + 24576) >> 2) : x;
}

static void fx_audio(const int32_t *bus, int32_t *out, uint32_t n, int quiet_in)
{
    uint32_t i, k;
    int32_t f, q, c, g, m, dry;
    f_now = glide(f_now, CUT_F[fx_cut]);
    q_now = glide(q_now, RES_Q[fx_res]);
    c_now = glide(c_now, RES_COMP[fx_res]);
    g_now = glide(g_now, SPACE_G[fx_space]);
    m_now = glide(m_now, (int32_t)fx_mix * 1638);   /* 0 .. 32760 */
    if (quiet_in) {
        if (rv_rest >= RV_TAIL) { sv_lp = sv_bp = 0; return; }   /* all quiet: costs nothing */
        rv_rest += n;
    } else rv_rest = 0;
    f = f_now; q = q_now; c = c_now; g = g_now; m = m_now;
    dry = 32767 - (m >> 1);                     /* at MIX 100 % the dry is half */
    {
        uint32_t tn = tex_tone, ct = tex_crk;
        int32_t ta = TONE_A[tn], hg = (int32_t)(((int64_t)tex_hiss * TONE_COMP[tn] >> 15) * drone_lvl >> 15);
        int32_t cg = drone_lvl;
        if (hg || ct || ck_l || ck_r) for (i = 0; i < n; i++) {   /* (written to out here; the drone adds below) */
            uint32_t rr = xs32(&tex_rng);
            int32_t nl = (int16_t)(rr >> 16), nr = (int16_t)rr, hl, hr;
            tx_l += ((nl - tx_l) * ta) >> 15;
            tx_r += ((nr - tx_r) * ta) >> 15;
            if (tn < 5u) { hl = tx_l; hr = tx_r; }
            else if (tn > 5u) { hl = nl - tx_l; hr = nr - tx_r; }
            else { hl = nl; hr = nr; }
            hl = (int32_t)(((int64_t)hl * hg) >> 15);
            hr = (int32_t)(((int64_t)hr * hg) >> 15);
            if (ct && xs32(&tex_rng) < ct) {    /* a crackle: 1500 .. 9500, either sign, panned */
                uint32_t r2 = xs32(&tex_rng), pan = r2 & 0x7FFFu;
                int32_t amp = (int32_t)(1500u + (r2 >> 19)) * ((r2 & 0x8000u) ? 1 : -1);
                amp = (amp * cg) >> 15;
                ck_l += (amp * (int32_t)(32767u - pan)) >> 15;
                ck_r += (amp * (int32_t)pan) >> 15;
            }
            out[2u * i] += hl + ck_l;
            out[2u * i + 1u] += hr + ck_r;
            ck_l = (ck_l * 22938) >> 15;        /* (each click dies in ~0.3 ms; */
            ck_r = (ck_r * 22938) >> 15;
            if (ck_l > -8 && ck_l < 8) ck_l = 0;    /*  to zero, not to -1 for ever) */
            if (ck_r > -8 && ck_r < 8) ck_r = 0;
        }
    }
    for (i = 0; i < n; i++) {
        int32_t x = (int32_t)(((int64_t)bus[i] * c) >> 15) << 4, y, l = 0, r = 0;
        for (k = 0; k < 2u; k++) {              /* the filter, twice a sample */
            int32_t hp;
            sv_lp += (int32_t)(((int64_t)f * sv_bp) >> 15);
            hp = x - sv_lp - (int32_t)(((int64_t)q * sv_bp) >> 15);
            sv_bp += (int32_t)(((int64_t)f * hp) >> 15);
        }
        y = sv_lp >> 4;
        if (m) {                                /* the reverb */
            int32_t in = y >> 2, v, t, y0, y1, y2, y3, a, b, cc, e;
            v = in - ((RV_DIFF * ap0[ai0]) >> 15); t = ap0[ai0] + ((RV_DIFF * v) >> 15); ap0[ai0] = (int16_t)sat16(v);
            if (++ai0 == AP0) ai0 = 0;
            v = t - ((RV_DIFF * ap1[ai1]) >> 15); in = ap1[ai1] + ((RV_DIFF * v) >> 15); ap1[ai1] = (int16_t)sat16(v);
            if (++ai1 == AP1) ai1 = 0;
            y0 = rv0[ri0]; y1 = rv1[ri1]; y2 = rv2[ri2]; y3 = rv3[ri3];
            dm0 += ((y0 - dm0) * RV_DAMP + 16384) >> 15; dm1 += ((y1 - dm1) * RV_DAMP + 16384) >> 15;   /* (rounded: */
            dm2 += ((y2 - dm2) * RV_DAMP + 16384) >> 15; dm3 += ((y3 - dm3) * RV_DAMP + 16384) >> 15;   /*  no drift) */
            a = dm0 + dm1; b = dm0 - dm1; cc = dm2 + dm3; e = dm2 - dm3;   /* Hadamard: (a+c, b+e, a-c, b-e) / 2 */
            rv0[ri0] = (int16_t)sat16(in + ((((b + e) >> 1) * g + 16384) >> 15));   /* (Hadamard, then rotated a */
            rv1[ri1] = (int16_t)sat16(-in + ((((a - cc) >> 1) * g + 16384) >> 15)); /*  line: a Hadamard alone */
            rv2[ri2] = (int16_t)sat16(in + ((((b - e) >> 1) * g + 16384) >> 15));   /*  swaps two patterns back */
            rv3[ri3] = (int16_t)sat16(-in + ((((a + cc) >> 1) * g + 16384) >> 15)); /*  and forth; this spreads) */
            if (++ri0 == RV0) ri0 = 0;
            if (++ri1 == RV1) ri1 = 0;
            if (++ri2 == RV2) ri2 = 0;
            if (++ri3 == RV3) ri3 = 0;
            l = ((y0 + y1 - y2 - y3) * (m >> 1)) >> 15;   /* L and R: two sign patterns across all four lines, */
            r = ((y0 - y1 - y2 + y3) * (m >> 1)) >> 15;   /* orthogonal to the input's: decorrelated */
        }
        y = (y * dry) >> 15;
        out[2u * i] += soft(y + l);
        out[2u * i + 1u] += soft(y + r);
    }
}

#define BUS_N 256u
static int32_t bus[BUS_N];
#ifndef DINGHY_HOST
static volatile uint32_t cpu_max_us;            /* the audio ISR's longest run since the main loop last looked */
#endif

static void app_audio(int32_t *out, uint32_t n)
{
    uint32_t v, i;
#ifndef DINGHY_HOST
    uint32_t t0 = fm1_ticks(), us;
#endif
    if (n > BUS_N) n = BUS_N;
    for (i = 0; i < n; i++) bus[i] = 0;
    drone_audio(bus, n);
    fx_audio(bus, out, n, drone_lvl == 0);
    for (v = 0; v < NV; v++) {
        uint32_t ph = voice[v].ph, step = (uint32_t)(((uint64_t)voice[v].inc * bend_q16) >> 16);   /* (int64 multiply: fine) */
        int32_t amp = voice[v].amp, g = voice[v].gate;
        if (!amp && !g)
            continue;                           /* silent: costs nothing */
        for (i = 0; i < n; i++) {
            int32_t s;
            if (amp < g) { amp += 48; if (amp > g) amp = g; }        /* attack ~5 ms */
            else if (amp > g) { amp -= 6; if (amp < g) amp = g; }    /* release ~45 ms */
            ph += step;
            s = (sine(ph) * amp) >> 15;
            out[2u * i] += s;                   /* L */
            out[2u * i + 1u] += s;              /* R */
        }
        voice[v].ph = ph;
        voice[v].amp = amp;
    }
#ifndef DINGHY_HOST
    us = (fm1_ticks() - t0) / FM1_TICKS_PER_US;
    if (us > cpu_max_us) cpu_max_us = us;
#endif
}

/* ---- notes (main loop) */
static const uint32_t INC_C9[12] = {            /* MIDI 120..131 (C9..B9); lower octaves shift right */
    815363807u, 863847862u, 915214929u, 969636441u, 1027294024u, 1088380105u,
    1153098554u, 1221665363u, 1294309365u, 1371273005u, 1452813141u, 1539201906u};
static uint32_t ages, last_note = 255;

static void play(uint32_t note, uint32_t vel)
{
    uint32_t v, pick = 0, best = 0xFFFFFFFFu;
    if (note > 127u || !vel) return;
    for (v = 0; v < NV; v++) {                  /* the same note, else a free voice, else the oldest */
        uint32_t score = voice[v].note == note && voice[v].gate ? 0u
                       : voice[v].gate ? 0x80000000u + voice[v].age : (uint32_t)voice[v].amp + 1u;
        if (score < best) { best = score; pick = v; }
    }
    voice[pick].note = (uint8_t)note;
    voice[pick].age = ++ages;
    voice[pick].inc = INC_C9[note % 12u] >> (10u - note / 12u);
    voice[pick].gate = (int32_t)(vel * 90u);    /* 127 -> 11430: four voices stay under full scale */
    last_note = note;
}
static void stop(uint32_t note)
{
    uint32_t v;
    for (v = 0; v < NV; v++)
        if (voice[v].note == note)
            voice[v].gate = 0;
}
static int quiet(void)                          /* nothing sounds, not even a release */
{
    uint32_t v;
    if (drone_on || drone_lvl) return 0;        /* the drone is sounding or fading: no flash erase now */
    for (v = 0; v < NV; v++)
        if (voice[v].gate || voice[v].amp) return 0;
    return 1;
}

/* ---- Groundkeeper's growth (main loop): which chord the network holds, and how its voices drift.
 * Six chords in C major, in a ring where neighbours share three notes, so every step is a small one.
 * Every few seconds one voice moves to a note of the current chord; every few moves the chord steps along the ring. */
static const uint8_t CHORD[6][5] = {            /* in C, as pitch classes: root, three chord tones, a colour (9th; 11th on iii) */
    {0, 4, 7, 11, 2}, {9, 0, 4, 7, 11}, {5, 9, 0, 4, 7}, {2, 5, 9, 0, 4}, {7, 11, 2, 5, 9}, {4, 7, 11, 2, 9}};
static const char *const QUAL[6] = {"maj7", "m7", "maj7", "m7", "7", "m7"};   /* I vi IV ii V iii */
static const uint8_t FIFTHS[12] = {0, 7, 2, 9, 4, 11, 6, 1, 8, 3, 10, 5};   /* the keys round the circle of fifths: C G D ... F */
static const char *const KEY_NAME[12] = {"C", "G", "D", "A", "E", "B", "F#", "Db", "Ab", "Eb", "Bb", "F"};
static const uint8_t WIN_C[DV] = {46, 58, 63, 68, 73, 78};   /* each voice's window centre at full SPREAD: E2..E3 up to C5..C6 */
static const uint8_t RANK[DV] = {0, 2, 1, 3, 4, 5};   /* the order voices grow in: the root, the middle, the low, the high, the colours */
static uint32_t grow_rng = 0x2545F491u, chord_now, key_now, moves_left, grow_ver, density = 4u, pulse_ms;
static int32_t spread = 8, pitch;               /* SPREAD 0..8 (8: the windows above), PITCH -12..+12 semitones */
static uint32_t chord_every = 5u;               /* CHORDS: pulses a chord (0: HOLD) */
static uint32_t bloom_t[DV];                    /* BLOOM: when each voice last bloomed (fm1_ms) */

static uint32_t rnd(uint32_t n)                 /* 0 .. n-1 (xorshift32) */
{
    grow_rng ^= grow_rng << 13; grow_rng ^= grow_rng >> 17; grow_rng ^= grow_rng << 5;
    return n ? grow_rng % n : 0u;
}
static uint32_t note_inc(uint32_t note) { return INC_C9[note % 12u] >> (10u - note / 12u); }
static uint32_t cpc(uint32_t c, uint32_t k) { return (CHORD[c][k] + FIFTHS[key_now]) % 12u; }   /* chord c's k-th pitch class, in the key now */
static int alive(uint32_t v) { return RANK[v] < density; }
static uint32_t win_lo(uint32_t v)              /* the lowest note of voice v's window (12 semitones wide) */
{
    return (uint32_t)(64 + pitch + ((int32_t)WIN_C[v] - 64) * spread / 8 - 6);   /* SPREAD 0: every window at E4..E5 */
}
static uint32_t new_moves(void) { return chord_every < 3u ? chord_every : chord_every - 1u + rnd(3u); }   /* (a little give) */
static int in_chord(uint32_t note, uint32_t c, uint32_t v)   /* a note voice v may hold: voices 4, 5 may hold the colour */
{
    uint32_t k, nk = v >= 4u ? 5u : 4u;
    for (k = 0; k < nk; k++) if (note % 12u == cpc(c, k)) return 1;
    return 0;
}
static int fits(uint32_t v, uint32_t c)         /* voice v's note belongs to chord c (the bass: only the root), in its window */
{
    uint32_t n = drone[v].note;
    if (n < win_lo(v) || n > win_lo(v) + 12u) return 0;
    return v == 0u ? n % 12u == cpc(c, 0) : in_chord(n, c, v);
}
static int held_by_other(uint32_t pc, uint32_t v)   /* another living upper voice already has this pitch class */
{
    uint32_t w;
    for (w = 1; w < DV; w++) if (w != v && alive(w) && drone[w].note % 12u == pc) return 1;
    return 0;
}
static uint32_t pick_note(uint32_t v, uint32_t c, int fresh)   /* a chord note in v's range, near where v is */
{
    uint32_t n, pass, best = drone[v].note, bestd = 0xFFFFu;
    for (pass = 0; pass < 3u && bestd == 0xFFFFu; pass++)   /* 0: every rule; 1: may double a pitch class; 2: may stay */
        for (n = win_lo(v); n <= win_lo(v) + 12u; n++) {
            uint32_t d = n > drone[v].note ? n - drone[v].note : drone[v].note - n;
            if (v == 0u ? n % 12u != cpc(c, 0) : !in_chord(n, c, v) || (pass == 0u && held_by_other(n % 12u, v))) continue;
            if (fresh && pass < 2u && n == drone[v].note) continue;   /* a drift: somewhere new */
            d = d * 4u + rnd(6u);               /* nearest, with a little chance */
            if (d < bestd) { bestd = d; best = n; }
        }
    return best;
}
static void voice_to(uint32_t v, uint32_t note)
{
    drone[v].band = (uint8_t)(note < WT_BAND0 ? 0u : (note - WT_BAND0) / WT_BAND_STEP < WT_NBAND ? (note - WT_BAND0) / WT_BAND_STEP : WT_NBAND - 1u);
    drone[v].note = (uint8_t)note;
    drone[v].tinc = note_inc(note);
    grow_ver++;
}
static void set_density(uint32_t d)             /* 1..6: voices grow in (planted on a chord note) or wither, in RANK order */
{
    uint32_t v;
    density = d;
    for (v = 0; v < DV; v++) {
        int32_t on = alive(v) && drone_on;
        if (on && !drone[v].vgate)              /* a new thread: planted where it fits the chord now */
            voice_to(v, pick_note(v, chord_now, 0));
        drone[v].vgate = on;
    }
    grow_ver++;
}
static void revoice(void)                       /* SPREAD or PITCH moved: every living voice to a chord note in its new window */
{
    uint32_t v;
    for (v = 0; v < DV; v++)
        if (alive(v)) voice_to(v, pick_note(v, chord_now, 0));
#ifdef DINGHY_HOST
    printf("%7u ms  revoice (spread %d, pitch %+d):", (unsigned)fm1_ms, (int)spread, (int)pitch);
    for (v = 0; v < DV; v++) if (alive(v)) printf(" %u", (unsigned)drone[v].note);
    printf("\n");
#endif
}
static void key_step(int32_t dir)               /* one step round the circle of fifths (+1 sharpwards), through a pivot */
{
    uint32_t old[4], c, k, j, best = chord_now, bests = 0;
    for (k = 0; k < 4u; k++) old[k] = cpc(chord_now, k);
    key_now = (key_now + (dir > 0 ? 1u : 11u)) % 12u;
    for (c = 0; c < 6u; c++) {                  /* the new key's chord sharing the most with the one sounding: often the same */
        uint32_t sc = 0;
        for (k = 0; k < 4u; k++) for (j = 0; j < 4u; j++) sc += cpc(c, k) == old[j];
        sc = sc * 2u + (cpc(c, 0) == old[0]);   /* (the same root breaks a tie) */
        if (sc > bests) { bests = sc; best = c; }
    }
    chord_now = best;
#ifdef DINGHY_HOST
    printf("%7u ms  season: key %s, pivot chord %u\n", (unsigned)fm1_ms, KEY_NAME[key_now], (unsigned)best);
#endif
    moves_left = new_moves();                   /* rest on the pivot a while before the new key's chords move on */
    grow_ver++;
}
static void bloom(void)                         /* BLOOM: one living voice blooms on a note of the chord */
{
    uint32_t v, n = 0, pick = DV;
    for (v = 0; v < DV; v++)                    /* a voice that has nearly faded is free */
        if (alive(v) && drone[v].vamp < 1500) n++;
    if (n) {                                    /* a free one, at random */
        n = rnd(n);
        for (v = 0; v < DV; v++)
            if (alive(v) && drone[v].vamp < 1500 && !n--) { pick = v; break; }
    } else                                      /* none free: the one that bloomed longest ago */
        for (v = 0; v < DV; v++)
            if (alive(v) && (pick == DV || (int32_t)(bloom_t[v] - bloom_t[pick]) < 0)) pick = v;
    if (pick == DV) return;
    voice_to(pick, pick_note(pick, chord_now, 1));
    drone[pick].peak = 20000 + (int32_t)rnd(12768u);   /* each bloom a little different */
    bloom_t[pick] = fm1_ms;
    drone[pick].trig++;                         /* (last: the audio starts it with the note and peak above) */
}
static void pulse_move(void)                    /* a pulse through the network: one voice moves (or blooms); now and then the chord */
{
    uint32_t v, w, n;
    pulse_ms = fm1_ms;
    if (chord_every && !moves_left) {           /* the network shifts: a neighbour on the ring (CHORDS: HOLD never) */
        chord_now = (chord_now + (rnd(3u) ? 1u : 5u)) % 6u;
        moves_left = new_moves();
    }
    if (moves_left) moves_left--;
    if (engine) { bloom(); return; }
    for (w = 0, v = DV; w < DV && v == DV; w++) /* a living voice outside the chord (or its window) moves first, the bass first */
        if (alive(w) && !fits(w, chord_now)) v = w;
    if (v < DV) { voice_to(v, pick_note(v, chord_now, 0)); return; }
    for (w = 1, n = 0; w < DV; w++) n += (uint32_t)alive(w);
    if (!n) return;                             /* only the root: nothing to drift */
    n = rnd(n);                                 /* all in the chord: one living upper voice drifts to a new voicing */
    for (w = 1; w < DV; w++)
        if (alive(w) && !n--) { voice_to(w, pick_note(w, chord_now, 1)); return; }
}

/* ---- the LFOs (main loop, per ms; nothing here runs in the audio ISR)
 * PULSE moves the voicings: a move each time its wave rises through the middle. SINE: regular; RANDOM: a smooth random
 * wave (a new target each quarter cycle, a raised-cosine glide to it), so the moves come irregularly; MIX: the two summed.
 * SEASON (OFF, 15 s .. 10 min in 15 s steps) moves the key: a slow random wave; each time it peaks high or dips low, the key takes a step round the circle
 * of fifths. It keeps travelling the same way, turning back a third of the time, so over the hours it wanders through
 * the keys (high = sharpwards, low = flatwards made a pendulum between two keys). Its cycle is 0.7 of the setting: a
 * key change comes about once per setting, on average. */
typedef struct { uint32_t ph; int32_t a, b; uint8_t seg, armed, zone; } lfo_t;   /* seg: log2 of random targets a cycle */
static lfo_t pulse = {0, 0, 0, 2, 0, 0}, season = {0, 0, 0, 1, 0, 0};
static uint32_t lfo_t0;
static int32_t season_dir = 1;                  /* +1: travelling sharpwards round the circle, -1: flatwards */
#define NPULSE 12u
static const uint16_t PULSE_S[NPULSE] = {0, 2, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64};   /* seconds a cycle; 0: STILL */
#define NSHAPE 3u
static const char *const SHAPE_NAME[NSHAPE] = {"SINE", "MIX", "RANDOM"};
#define NSEASON 41u                             /* OFF, then 15 s .. 10 min in 15 s steps */
#define SEASON_S(i) ((uint32_t)(i) * 15u)

static int lfo_advance(lfo_t *l, uint32_t dt, uint32_t ms)   /* 1: a new random segment began (pick the next target) */
{
    uint32_t old = l->ph;
    if (!ms) return 0;
    l->ph += (4294967295u / ms) * dt;
    return (old >> (32u - l->seg)) != (l->ph >> (32u - l->seg));
}
static void lfo_target(lfo_t *l) { l->a = l->b; l->b = (int32_t)rnd(65535u) - 32767; }
static int32_t lfo_random(const lfo_t *l)       /* -32767..32767: from a to b along a raised cosine */
{
    uint32_t u = (l->ph << l->seg) >> 1;        /* where we are in this segment, 0..2^31 */
    int32_t w = (32767 - sine(u + 0x40000000u)) >> 1;   /* 0 .. 32767 */
    return l->a + ((((l->b - l->a) >> 1) * w) >> 14);
}
static void lfo_reset(void)                     /* PLAY: both waves start from the middle */
{
    pulse.ph = season.ph = 0;
    pulse.a = pulse.b = season.a = season.b = 0;
    pulse.armed = season.zone = 0;
    lfo_target(&pulse);
    season_dir = rnd(2u) ? 1 : -1;
    lfo_t0 = fm1_ms;
}
static int lfo_tick(uint32_t pul, uint32_t shp, uint32_t sea)   /* returns 1 when SEASON moved the key */
{
    uint32_t dt = fm1_ms - lfo_t0;
    int32_t v;
    int moved = 0;
    lfo_t0 = fm1_ms;
    if (!drone_on || !dt) return 0;
    if (dt > 500u) dt = 500u;                   /* (after a stall: no leap) */
    if (lfo_advance(&pulse, dt, PULSE_S[pul] * 1000u)) lfo_target(&pulse);
    if (PULSE_S[pul]) {
        v = shp == 0u ? sine(pulse.ph) : shp == 2u ? lfo_random(&pulse) : (sine(pulse.ph) + lfo_random(&pulse)) >> 1;
        if (v < -4000) pulse.armed = 1;         /* (below the middle first: one move per rise) */
        else if (v > 0 && pulse.armed) { pulse.armed = 0; pulse_move(); }
    }
    if (lfo_advance(&season, dt, SEASON_S(sea) * 700u)) lfo_target(&season);
    if (sea) {
        v = lfo_random(&season);
        int ev = 0;
        if (season.zone != 1u && v > 16384) { season.zone = 1; ev = 1; }
        else if (season.zone != 2u && v < -16384) { season.zone = 2; ev = 1; }
        else if ((season.zone == 1u && v < 8192) || (season.zone == 2u && v > -8192)) season.zone = 0;
        if (ev) {                               /* a peak or a dip: a step, mostly onward */
            if (!rnd(3u)) season_dir = -season_dir;
            key_step(season_dir);
            moved = 1;
        }
    }
    return moved;
}

static void grow_start(void)                    /* PLAY: plant the network on the key's home chord */
{
    uint32_t v;
    grow_rng ^= fm1_ms * 2654435761u;
    if (!grow_rng) grow_rng = 1;
    chord_now = 0;
    for (v = 0; v < DV; v++) {
        drone[v].note = (uint8_t)(win_lo(v) + 6u);
        drone[v].vib_inc = 4294967295u / 172u * (9u + rnd(3u)) / 2u;   /* a vibrato of 4.5 .. 5.5 Hz (per block) */
        drone[v].vgate = 0;
        drone[v].lfo_inc = 4294967295u / (172u * (7u + rnd(13u)));   /* a breath every 7..19 s */
    }
    moves_left = new_moves();
    lfo_reset();
    drone_on = 1;
    set_density(density);
}
static void grow_stop(void)                     /* PLAY again: the whole network fades back into the ground */
{
    drone_on = 0;                               /* (the voices keep their gates: the master fade does it) */
    grow_ver++;
}

/* ---- the screen: the name, the last note big, the keyboard with what sounds, the octave */
#define C_ACC RGB(120, 170, 90)                 /* moss green */
#define C_KEY RGB(210, 210, 210)
#define C_OFF RGB(90, 90, 90)
#define KEY0 53u                                /* key 0 = F3 */
static int black(uint32_t note) { return (0x54Au >> (note % 12u)) & 1u; }

static void draw_note(void)
{
    static const char *const NAME[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    char s[8] = "-";
    if (drone_on) {                             /* the chord: its root big, the accidental and the quality small */
        static const char *const SHARP[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
        static const char *const FLAT[12] = {"C", "Db", "D", "Eb", "E", "F", "Gb", "G", "Ab", "A", "Bb", "B"};
        const char *nm = (key_now >= 7u ? FLAT : SHARP)[cpc(chord_now, 0)], *q = QUAL[chord_now];
        char root[2] = {nm[0], 0};
        int32_t x = (240 - (text_w(&FONT_L, root) + text_w(&FONT_S, nm + 1) + 2 + text_w(&FONT_S, q))) / 2;
        cv_begin(240, 32, C_BLACK);
        x = cv_text(x, 0, &FONT_L, root, C_ACC);
        x = cv_text(x, 1, &FONT_S, nm + 1, C_ACC) + 2;   /* # or b, raised */
        cv_text(x, 15, &FONT_S, q, C_ACC);      /* maj7, m7, 7: on the line */
        cv_blit(0, 82);
        return;
    }
    if (last_note > 127u) { lcd_fill(0, 76, 240, 44, C_BLACK); draw_text_box(0, 90, 240, &FONT_S, "press PLAY to grow", C_OFF, 1); return; }
    if (last_note < 128u) {
        const char *p = NAME[last_note % 12u];
        uint32_t i = 0;
        while (*p) s[i++] = *p++;
        if (last_note >= 12u) s[i++] = (char)('0' + last_note / 12u - 1u);
        s[i] = 0;
    }
    draw_text_box(0, 82, 240, &FONT_L, s, C_WHITE, 1);
}

static void draw_keys(uint32_t lit)             /* bit k: key k sounds */
{
    uint32_t k, w = 0;
    cv_begin(240, 56, C_BLACK);
    for (k = 0; k < 27u; k++)                   /* 16 white keys of 14 px from x = 8 */
        if (!black(KEY0 + k)) {
            cv_rect(8 + (int32_t)w * 14, 0, 13, 56, lit >> k & 1u ? C_ACC : C_KEY);
            w++;
        }
    for (k = 0, w = 0; k < 27u; k++)            /* the black ones on the line between two white */
        if (black(KEY0 + k))
            cv_rect(8 + (int32_t)w * 14 - 5, 0, 9, 34, lit >> k & 1u ? C_ACC : C_BLACK);
        else
            w++;
    cv_blit(0, 140);
}

/* ---- the settings: one byte each, kept in flash; the F_ order is the struct's */
#define NCHG 11u
static const uint8_t CHG_N[NCHG] = {0, 1, 2, 3, 4, 5, 6, 8, 10, 12, 16};   /* CHORDS: pulses a chord; 0: HOLD */
#define NATT 14u
#define NREL 14u
static struct { uint8_t oct, ch, tim, den, key, pul, shp, sea, cut, res, spc, mix, eng, spr, pit, chg, att[2], rel[2], his, ton, crk, wob; } set;
enum { F_OCT, F_CH, F_TIM, F_DEN, F_KEY, F_PUL, F_SHP, F_SEA, F_CUT, F_RES, F_SPC, F_MIX, F_ENG, F_SPR, F_PIT, F_CHG,
       F_ATT, F_ATT1, F_REL, F_REL1, F_HIS, F_TON, F_CRK, F_WOB, F_N, F_CPU = F_N };
#define CFG ((uint8_t *)&set)
static const uint8_t CFG_MIN[F_N] = {0, 0, 0, 1};   /* (the rest: 0) */
static const uint8_t CFG_MAX[F_N] = {255, 16, NT - 1u, DV, 11, NPULSE - 1u, NSHAPE - 1u, NSEASON - 1u, NCUT - 1u, NRES - 1u,
    NSPACE - 1u, NMIX - 1u, 1, 8, 24, NCHG - 1u, NATT - 1u, NATT - 1u, NREL - 1u, NREL - 1u, 20, 10, 20, 10};
static const uint8_t CFG_DEF[F_N] = {0, 0, 0, 4, 0, 4, 1, 0, NCUT - 1u, 0, 8, 5, 0, 8, 12, 5,
    10, 4, 10, 8, 0, 5, 0, 0};                  /* DRONE: attack 3 s, release 6 s; BLOOM: 350 ms, 3 s */
static const char *const F_NAME[F_N + 1] = {"OCT", "CH", "TIMBRE", "DENSITY", "KEY", "PULSE", "SHAPE", "SEASON", "FILTER", "RES",
    "SPACE", "MIX", "ENGINE", "SPREAD", "PITCH", "CHORDS", "ATTACK", "ATTACK", "RELEASE", "RELEASE", "HISS", "TONE", "CRACKLE",
    "WOBBLE", "CPU"};
/* the pages: HOME, SEL (the engine), LFO, FX 1, FX 2; F_ATT / F_REL mean the engine's own */
enum { P_HOME, P_SEL, P_LFO, P_FX1, P_FX2, NPAGE };
static const uint8_t PAGE_F[NPAGE][4] = {{F_DEN, F_SPR, F_PIT, F_CHG}, {F_ENG, F_TIM, F_ATT, F_REL}, {F_PUL, F_SHP, F_SEA, F_CPU},
                                         {F_CUT, F_RES, F_SPC, F_MIX}, {F_HIS, F_TON, F_CRK, F_WOB}};
static uint32_t page_field(uint32_t page, uint32_t k)
{
    uint32_t f = PAGE_F[page][k];
    return f == F_ATT || f == F_REL ? f + set.eng : f;
}
static void apply(void)                         /* the settings to the sound */
{
    fx_cut = set.cut; fx_res = set.res; fx_space = set.spc; fx_mix = set.mix;
    tim_now = set.tim;
    engine = set.eng;
    env_att = ATT_INC[set.att[set.eng]];
    env_rel = REL_K[set.rel[set.eng]];
    spread = set.spr;
    pitch = (int32_t)set.pit - 12;
    chord_every = CHG_N[set.chg];
    tex_hiss = HISS_G[set.his]; tex_tone = set.ton; tex_crk = CRK_T[set.crk];
    wob_depth = WOB_D[set.wob];
}

static const char *const TIM_NAME[NT] = {"MOSS", "MIST", "CHOIR", "ORGAN", "REED"};
static const char *const ENG_NAME[2] = {"DRONE", "BLOOM"};
static void put(char **p, const char *a) { while (*a) *(*p)++ = *a++; }
static void put_num(char **p, uint32_t n) { if (n >= 100u) *(*p)++ = (char)('0' + n / 100u); if (n >= 10u) *(*p)++ = (char)('0' + n / 10u % 10u); *(*p)++ = (char)('0' + n % 10u); }
static void put_signed(char **p, int32_t n) { if (n) *(*p)++ = n < 0 ? '-' : '+'; put_num(p, (uint32_t)(n < 0 ? -n : n)); }

static void draw_info(uint32_t key, int32_t oct, int lit)   /* "KEY C   DRONE   MOSS" (+ OCT, CH when set) */
{
    char s[40], *p = s;
    put(&p, "KEY "); put(&p, KEY_NAME[key]); put(&p, "   "); put(&p, ENG_NAME[set.eng]); put(&p, "   "); put(&p, TIM_NAME[set.tim]);
    if (oct) { put(&p, "  OCT"); *p++ = oct < 0 ? '-' : '+'; *p++ = (char)('0' + (oct < 0 ? -oct : oct)); }
    if (set.ch) { put(&p, "  CH"); put_num(&p, set.ch); }
    *p = 0;
    draw_text_box(0, 52, 240, &FONT_S, s, lit ? C_ACC : C_OFF, 1);   /* (green a moment when the season moves the key) */
}
static void fmt_season(char **p, uint32_t sea)   /* OFF, 45s, 1m, 1m15 */
{
    uint32_t t = SEASON_S(sea);
    if (!t) { put(p, "OFF"); return; }
    if (t < 60u) { put_num(p, t); *(*p)++ = 's'; return; }
    put_num(p, t / 60u); *(*p)++ = 'm';
    if (t % 60u) put_num(p, t % 60u);
}
static void fmt_field(char **q, uint32_t f, uint32_t cpu)   /* a setting's value as the panel shows it */
{
    uint32_t v = f < F_N ? CFG[f] : 0u;
    switch (f) {
    case F_PIT: if (v == 12u) put(q, "0"); else put_signed(q, (int32_t)v - 12); break;
    case F_CHG: if (CHG_N[v]) put_num(q, CHG_N[v]); else put(q, "HOLD"); break;
    case F_ENG: put(q, ENG_NAME[v]); break;
    case F_TIM: put(q, TIM_NAME[v]); break;
    case F_ATT: case F_ATT1: put(q, ATT_LABEL[v]); break;
    case F_REL: case F_REL1: put(q, REL_LABEL[v]); break;
    case F_PUL: if (PULSE_S[v]) { put_num(q, PULSE_S[v]); *(*q)++ = 's'; } else put(q, "STILL"); break;
    case F_SHP: put(q, SHAPE_NAME[v]); break;
    case F_SEA: fmt_season(q, v); break;
    case F_CUT: put(q, CUT_LABEL[v]); break;
    case F_SPC: put(q, SPACE_LABEL[v]); break;
    case F_MIX: put_num(q, v * 5u); *(*q)++ = '%'; break;
    case F_TON: if (v == 5u) put(q, "WHITE"); else put_signed(q, (int32_t)v - 5); break;
    case F_HIS: case F_CRK: case F_WOB: if (v) put_num(q, v); else put(q, "OFF"); break;
    case F_CPU: if (cpu) { put_num(q, cpu); *(*q)++ = '%'; } else put(q, "--"); break;
    default: put_num(q, v); break;
    }
}
static uint32_t cfg_min(uint32_t f) { return f < 4u ? CFG_MIN[f] : 0u; }
/* the four knobs of the page: a name, its value, a bar */
static void draw_knobs(uint32_t page, uint32_t cpu)
{
    uint32_t k;
    cv_begin(240, 50, C_BLACK);
    for (k = 0; k < 4u; k++) {
        uint32_t f = page_field(page, k), x0 = k * 60u;
        char v[12], *q = v;
        fmt_field(&q, f, cpu);
        *q = 0;
        cv_text((int32_t)x0 + (60 - text_w(&FONT_S, F_NAME[f])) / 2, 0, &FONT_S, F_NAME[f], C_OFF);
        cv_text((int32_t)x0 + (60 - text_w(&FONT_S, v)) / 2, 18, &FONT_S, v, f == F_CPU ? C_OFF : C_ACC);
        if (f < F_N) {
            uint32_t w = (CFG[f] - cfg_min(f)) * 46u / (CFG_MAX[f] - cfg_min(f));
            cv_rect((int32_t)x0 + 7, 40, 46, 3, RGB(45, 45, 45));
            if (w) cv_rect((int32_t)x0 + 7, 40, (int32_t)w, 3, C_ACC);
        }
    }
    cv_blit(0, 148);
}
static void draw_tabs(uint32_t page)            /* the bottom line: the pages, the one shown lit */
{
    static const char *const TAB[NPAGE] = {"HOME", "SEL", "LFO", "FX1", "FX2"};
    uint32_t i;
    int32_t x, w = 0;
    for (i = 0; i < NPAGE; i++) w += text_w(&FONT_S, TAB[i]) + (i ? 14 : 0);
    x = (240 - w) / 2;
    cv_begin(240, 16, C_BLACK);
    for (i = 0; i < NPAGE; i++) x = cv_text(x, 0, &FONT_S, TAB[i], i == page ? C_ACC : RGB(65, 65, 65)) + 14;
    cv_blit(0, 214);
}

static void draw_network(uint32_t den, uint32_t on, int flash, uint32_t sounding)   /* nodes and threads: grown ones lit, the rest seeds; bit i: node i sounds */
{
    static const int8_t DY[DV] = {2, -5, 4, -3, 5, -4};   /* a little unevenness, like something that grew */
    uint32_t i;
    uint16_t lit = flash ? RGB(190, 235, 150) : on ? C_ACC : RGB(70, 90, 60), seed = RGB(55, 55, 55);   /* a pulse: brighter a moment */
    cv_begin(240, 26, C_BLACK);
    for (i = 0; i + 1u < DV; i++) {             /* the threads between grown nodes */
        int32_t x0 = 45 + (int32_t)i * 30, x1 = x0 + 30, y0 = 13 + DY[i], y1 = 13 + DY[i + 1u];
        uint16_t c = i + 1u < den ? lit : seed;
        cv_line(x0, y0, x1, y1, c);
        if (i + 1u < den) cv_line(x0, y0 + 1, x1, y1 + 1, c);
    }
    for (i = 0; i < DV; i++) {                  /* the nodes */
        int32_t x = 45 + (int32_t)i * 30, y = 13 + DY[i];
        if (i < den) cv_rect(x - 4, y - 4, 9, 9, sounding >> i & 1u ? RGB(190, 235, 150) : lit);
        else cv_rect(x - 2, y - 2, 5, 5, seed);
    }
    cv_blit(0, 113);
}

static void draw_midi(uint32_t src, uint32_t bpm, uint32_t running)   /* where notes came from last, the clock */
{
    char s[32] = "", *p = s;
    const char *a = src == 2u ? "TRS IN" : src == 1u ? "USB IN" : "";
    while (*a) *p++ = *a++;
    if (bpm) {
        if (p != s) { *p++ = ' '; *p++ = ' '; }
        a = running ? "CLOCK " : "clock ";
        while (*a) *p++ = *a++;
        if (bpm >= 100u) *p++ = (char)('0' + bpm / 100u);
        *p++ = (char)('0' + bpm / 10u % 10u);
        *p++ = (char)('0' + bpm % 10u);
        a = " BPM";
        while (*a) *p++ = *a++;
    }
    *p = 0;
    draw_text_box(0, 120, 240, &FONT_S, s, C_OFF, 1);
}

/* ---- MIDI in: from the main loop, and from app_idle while the screen draws (idle_ok) */
static uint32_t idle_ok, midi_src, clk_n, clk_t0, clk_bpm, clk_run, clk_beat_ms;

static void midi_in(void)
{
    uint8_t st, d1, d2;
    int src;
    while ((src = midi_get(&st, &d1, &d2)) != 0) {   /* 1 USB, 2 the TRS jack */
        uint32_t ty = st & 0xF0u;
        if (st == 0xF8u) {                      /* clock: 24 a quarter note */
            if (clk_n % 24u == 0u) {
                if (clk_n) {
                    uint32_t ms = fm1_ms - clk_t0;   /* a quarter's length */
                    clk_bpm = ms ? (60000u + ms / 2u) / ms : 0;
                }
                clk_t0 = fm1_ms;
                clk_beat_ms = fm1_ms;
            }
            clk_n++;
            continue;
        }
        if (st == 0xFAu) { clk_run = 1; clk_n = 0; continue; }   /* START: the next clock is beat 1 */
        if (st == 0xFBu) { clk_run = 1; continue; }              /* CONTINUE */
        if (st == 0xFCu) { clk_run = 0; continue; }              /* STOP */
        if (set.ch && (st & 15u) != set.ch - 1u) continue;      /* not our channel */
        if (ty == 0x90u && d2) { play(d1, d2); midi_src = (uint32_t)src; }
        else if (ty == 0x80u || ty == 0x90u) stop(d1);
    }
}
static void app_idle(void)
{
    if (idle_ok) midi_in();
}

static void app_main(void)
{
    uint32_t prev = 0, k, t_change = 0, dirty = 0, note_drawn = ~0u, status_drawn = ~0u;
    uint32_t midi_drawn = ~0u, lfo_drawn = ~0u, hint_drawn = ~0u, key_ms = 0, kl, fl, snd, page = P_HOME, cpu_pct = 0;
#ifndef DINGHY_HOST
    uint32_t cpu_t = 0;
#endif
    uint8_t held[27] = {0};                     /* the note each key plays */
#if BOARD_SMK37
    static const uint8_t WHITE[16] = {0, 2, 4, 6, 7, 9, 11, 12, 14, 16, 18, 19, 21, 23, 24, 26};   /* white key -> key */
    uint8_t pad_note[16] = {0}, ext_note[FM1_NKEY] = {0};
    smk_pw_t wheel = {0};
    smk_pad_events = 1;                         /* the pads as events (smk_pad_take): they play here, lit while they sound */
#endif
    {                                           /* the settings: loaded, every one in its range, else all the defaults */
        int ok = !settings_load(&set, sizeof set) && (int8_t)set.oct >= -2 && (int8_t)set.oct <= 2;
        for (k = 1; ok && k < F_N; k++) ok = CFG[k] >= cfg_min(k) && CFG[k] <= CFG_MAX[k];
        if (!ok) for (k = 0; k < F_N; k++) CFG[k] = CFG_DEF[k];
    }
    apply();
    tim_prev = set.tim;
    density = set.den;
    key_now = set.key;
    lcd_fill(0, 0, 240, 240, C_BLACK);
    draw_text_box(0, 18, 240, &FONT_L, APP_NAME, C_ACC, 1);

    for (;;) {
        uint32_t keys, down, up, rel, b, lit = 0, out_ch = set.ch ? set.ch - 1u : 0u, st;
        int32_t oct = (int8_t)set.oct, e;
        uint32_t base = (uint32_t)((int32_t)KEY0 + 12 * oct);
        platform_poll();

        keys = fm1_in.notes;                    /* bit k: key k (0 = F3 .. 26 = G5) */
        down = keys & ~prev;
        up = prev & ~keys;
        prev = keys;
        for (k = 0; k < 27u; k++) {                 /* (a key stops the note it started: the octave may change meanwhile) */
#if BOARD_SMK37
            uint32_t vel = fm1_in.key_vel[SMK_KEY_F3 + k];   /* the SMK's keys feel the velocity */
#else
            uint32_t vel = 100;
#endif
            if (down >> k & 1u) { held[k] = (uint8_t)(base + k); play(held[k], vel); midi_send(0x90u | out_ch, held[k], vel); }
            if (up >> k & 1u) { stop(held[k]); midi_send(0x80u | out_ch, held[k], 0); }
        }
#if BOARD_SMK37
        while ((e = smk_extkey_take()) >= 0) {  /* the keys left and right of the FM-1's 27: key << 8 | velocity */
            uint32_t key = (uint32_t)e >> 8, vel = (uint32_t)e & 0x7Fu;
            if (key >= FM1_NKEY) continue;
            if (vel) { ext_note[key] = (uint8_t)(base + key - SMK_KEY_F3); play(ext_note[key], vel); midi_send(0x90u | out_ch, ext_note[key], vel); }
            else if (ext_note[key]) { stop(ext_note[key]); midi_send(0x80u | out_ch, ext_note[key], 0); ext_note[key] = 0; }
        }
        while ((e = smk_pad_take()) >= 0) {     /* a pad: hardware pad << 8 | velocity (0 = let go) */
            uint32_t pad = ((uint32_t)e >> 8) & 15u, vel = (uint32_t)e & 0x7Fu;
            uint32_t w = pad < 8u ? pad + 8u : pad - 8u;   /* bottom row = white keys 1-8, top row 9-16 */
            if (vel) { pad_note[pad] = (uint8_t)(base + WHITE[w]); play(pad_note[pad], vel); midi_send(0x90u | out_ch, pad_note[pad], vel); }
            else if (pad_note[pad]) { stop(pad_note[pad]); midi_send(0x80u | out_ch, pad_note[pad], 0); pad_note[pad] = 0; }
        }
        {
            int32_t bd = smk_pw_bend(&wheel, fm1_in.wheel[0], fm1_ms);   /* 1/4096 semitone */
            bend_q16 = (uint32_t)(65536 + (bd * 3785 >> 12) + (((bd * bd) >> 12) * 109 >> 12));   /* ~2^(s/12) */
        }
#endif

        b = fm1_input_edges(&rel);              /* buttons pressed since the last call (bit = button id) */
        if (b & (1u << BTN_OCT_DOWN) && oct > -2) { set.oct = (uint8_t)(oct - 1); dirty = 1; t_change = fm1_ms; }
        if (b & (1u << BTN_OCT_UP) && oct < 2) { set.oct = (uint8_t)(oct + 1); dirty = 1; t_change = fm1_ms; }
        if ((e = fm1_enc_take(ENC_SELECT)) != 0) {     /* SELECT: the key, round the circle of fifths (pivoting, as the season) */
            int32_t i = e > 11 ? 11 : e < -11 ? -11 : e;
            for (; i > 0; i--) key_step(1);
            for (; i < 0; i++) key_step(-1);
            set.key = (uint8_t)key_now;
            dirty = 1; t_change = fm1_ms;
        }
        if (b & (1u << BTN_HOME)) page = P_HOME;                    /* HOME: back home */
        if (b & (1u << BTN_SEL)) page = page == P_SEL ? P_HOME : P_SEL;   /* SEL: the engine page (again: home) */
        if (b & (1u << BTN_LFO)) page = page == P_LFO ? P_HOME : P_LFO;   /* LFO: the LFO page (again: home) */
        if (b & (1u << BTN_FX)) page = page == P_FX1 ? P_FX2 : P_FX1;     /* FX: round the effect pages */
        for (k = 0; k < 4u; k++)                /* KNOB 1..4: the page's four settings */
            if ((e = fm1_enc_take(ENC_KNOB1 + k)) != 0) {
                uint32_t f = page_field(page, k);
                int32_t d;
                if (f >= F_N) continue;         /* (a display, not a setting) */
                d = (int32_t)CFG[f] + e;
                CFG[f] = (uint8_t)(d < (int32_t)cfg_min(f) ? (int32_t)cfg_min(f) : d > (int32_t)CFG_MAX[f] ? (int32_t)CFG_MAX[f] : d);
                apply();
                if (f == F_DEN) set_density(set.den);
                if (f == F_SPR || f == F_PIT) revoice();
                dirty = 1; t_change = fm1_ms;
            }
        if ((e = fm1_enc_take(ENC_ALGORITHM)) != 0) {   /* ALGORITHM: the timbre, round the ring (as SEL's KNOB 2) */
            set.tim = (uint8_t)(((int32_t)set.tim + e % (int32_t)NT + (int32_t)NT) % (int32_t)NT);
            apply();
            dirty = 1;
            t_change = fm1_ms;
        }
        if ((e = fm1_enc_take(ENC_PRESETS)) != 0) {   /* detents turned since the last call */
            int32_t c = (int32_t)set.ch + e;
            set.ch = (uint8_t)(c < 0 ? 0 : c > 16 ? 16 : c);
            dirty = 1;
            t_change = fm1_ms;
        }

        if (b & (1u << BTN_PLAY)) {             /* PLAY: grow, or let it fade back into the ground */
            if (drone_on) grow_stop();
            else grow_start();
        }
        if (lfo_tick(set.pul, set.shp, set.sea)) {   /* the season moved the key */
            set.key = (uint8_t)key_now;
            dirty = 1; t_change = fm1_ms; key_ms = fm1_ms;
        }

        midi_in();
        if (clk_bpm && fm1_ms - clk_t0 > 2000u) clk_bpm = 0, clk_run = 0;   /* the clock has stopped coming */

        oct = (int8_t)set.oct;
        base = (uint32_t)((int32_t)KEY0 + 12 * oct);
        for (k = 0; k < NV; k++)                /* what sounds, on the keys of this octave */
            if (voice[k].gate && voice[k].note >= base && voice[k].note < base + 27u)
                lit |= 1u << (voice[k].note - base);
        for (k = 0; k < DV; k++)                /* the network's sounding notes light up too */
            if (drone[k].vamp > 2000 && drone[k].note >= base && drone[k].note < base + 27u)
                lit |= 1u << (drone[k].note - base);
        for (k = 0; k < 27u; k++)
            fm1_led_key(KEY_LED(k), lit >> k & 1u);
        fm1_led_key(BTN_HOME, page == P_HOME);  /* the page's button lit */
        fm1_led_key(BTN_SEL, page == P_SEL);
        fm1_led_key(BTN_LFO, page == P_LFO);
        fm1_led_key(BTN_FX, page == P_FX1 || page == P_FX2);
        fm1_led_key(BTN_OCT_DOWN, oct < 0);
        fm1_led_key(BTN_OCT_UP, oct > 0);
        fm1_led_key(BTN_PLAY, drone_on || clk_run || (clk_bpm && fm1_ms - clk_beat_ms < 80u));   /* lit: growing */
#if BOARD_SMK37
        for (k = 0; k < 16u; k++) {             /* a pad glows lilac while its key sounds */
            uint32_t w = k < 8u ? k + 8u : k - 8u, on = lit >> WHITE[w] & 1u;
            padled_set(k, on ? 90 : 4, on ? 60 : 2, on ? 128 : 8);
        }
        padled_flush();                         /* (sends only what changed) */
#endif

        idle_ok = 1;                            /* the drawing below may read MIDI meanwhile (LCD_IDLE) */
        if ((st = drone_on ? 1000u + chord_now + key_now * 8u : last_note) != note_drawn) { note_drawn = st; draw_note(); }
        kl = key_ms && fm1_ms - key_ms < 3000u;
        if ((st = (uint32_t)(oct + 2) | (uint32_t)set.ch << 4 | (uint32_t)set.tim << 9 | key_now << 12 | kl << 16 | (uint32_t)set.eng << 17) != status_drawn) { status_drawn = st; draw_info(key_now, oct, kl); }
        st = page | cpu_pct << 3;               /* the panel: redrawn when the page, a value on it or the CPU changes */
        for (k = 0; k < 4u; k++) { uint32_t f = page_field(page, k); st = st * 31u + (f < F_N ? CFG[f] : 0u) + f * 7u; }
        if (st != lfo_drawn) { lfo_drawn = st; draw_knobs(page, cpu_pct); }
#ifndef DINGHY_HOST
        if (fm1_ms - cpu_t >= 1000u) {          /* the audio ISR's worst run of the last second, of its 5.8 ms */
            cpu_t = fm1_ms;
            cpu_pct = cpu_max_us * 100u / 5805u;
            cpu_max_us = 0;
        }
#endif
        if (page != hint_drawn) { hint_drawn = page; draw_tabs(page); }
        fl = drone_on && pulse_ms && fm1_ms - pulse_ms < 300u;
        snd = 0;                                /* BLOOM: the nodes of the voices sounding now */
        if (set.eng) for (k = 0; k < DV; k++) if (drone[k].vamp > 6000) snd |= 1u << RANK[k];
        if ((st = set.den | (uint32_t)drone_on << 4 | fl << 5 | snd << 6) != midi_drawn) { midi_drawn = st; draw_network(set.den, (uint32_t)drone_on, (int)fl, snd); }
        idle_ok = 0;

        if (dirty && quiet() && fm1_ms - t_change > 2000u) {   /* 2 s after the change, when silent (an erase stalls the sound) */
            settings_save(&set, sizeof set);
            dirty = 0;
        }
    }
}
