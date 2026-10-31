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

/* ---- Groundkeeper's drone: DV voices that hold a chord and drift (the audio part; the decisions are in the main loop)
 * Each voice is two sines a hair apart (slow beating) and a soft octave above, with its own slow "breath".
 * The main loop sets a voice's target pitch; here the pitch glides there over a second or two. */
#define DV 6u                                   /* the most voices the network can grow (density 6) */
static struct {
    volatile uint32_t tinc;                     /* target phase step (main loop writes) */
    uint32_t inc;                               /* the step now: glides to tinc */
    uint32_t pa, pb, lfo;                       /* the two sines' phases, the breath's phase */
    volatile uint32_t lfo_inc;                  /* breath speed, per block */
    volatile int32_t vgate;                     /* 1: this voice is part of the network (main loop writes) */
    int32_t vamp;                               /* its level now, Q15: grows in / withers out over ~3 s */
    volatile uint8_t note;
} drone[DV];
#define NT 4u                                   /* timbres: MOSS, LOAM, MIST, ROOT */
static volatile uint32_t tim_now;               /* the timbre (main loop writes) */
static uint32_t tim_prev, tim_xf = 32767;       /* the crossfade from the last one, Q15 (audio) */
static volatile int32_t drone_on;               /* 1: growing / sounding, 0: fading to silence */
static int32_t drone_lvl;                       /* the drone's level now, Q15: fades in and out */
#define DRONE_GAIN 4600                         /* per voice, Q15: six voices with partials stay under full scale */

static int32_t shape(uint32_t t, uint32_t pa, uint32_t pb, int32_t breath)   /* one sample of timbre t, ~+-32767 */
{
    switch (t) {
    case 1u:                                    /* LOAM: odd harmonics, hollow and dark, like a soft reed */
        return (sine(pa) + (sine(pb) >> 1) + ((sine(pa * 3u) * 10923) >> 15) + ((sine(pa * 5u) * 4000) >> 15)) >> 1;
    case 2u:                                    /* MIST: the pair an octave up, the fundamental underneath, airy */
        return ((sine(pa << 1) + sine(pb << 1) + (sine(pa) >> 1) + (sine(pa * 3u) >> 3)) * 24576) >> 16;
    case 3u: {                                  /* ROOT: FM, the modulator a hair off: brighter as the voice breathes in */
        int32_t m = sine(pb);
        return (sine(pa + (uint32_t)(m * (breath >> 1))) * 24576) >> 15;   /* up to ~0.8 rad of modulation; 3/4 level */
    }
    default:                                    /* MOSS: two sines a hair apart and a quiet octave */
        return (sine(pa) + sine(pb) + (sine(pa << 1) >> 2)) >> 1;
    }
}

static void drone_audio(int32_t *bus, uint32_t n)   /* adds the drone, mono, to bus */
{
    uint32_t v, i;
    int32_t lvl = drone_lvl;
    if (drone_on) { lvl += 48; if (lvl > 32767) lvl = 32767; }   /* fade in ~4 s */
    else { lvl -= 32; if (lvl < 0) lvl = 0; }                     /* fade out ~6 s */
    drone_lvl = lvl;
    if (tim_now != tim_prev && tim_xf == 32767) tim_xf = 0;   /* (prev: the old one, now: the new one) */
    if (tim_xf < 32767) {                       /* a new timbre: crossfade over ~0.2 s */
        tim_xf += 1024;
        if (tim_xf >= 32767) { tim_xf = 32767; tim_prev = tim_now; }
    }
    if (!lvl) return;                           /* silent: costs nothing */
    for (v = 0; v < DV; v++) {
        uint32_t pa = drone[v].pa, pb = drone[v].pb, inc = drone[v].inc, tinc = drone[v].tinc, incb;
        int32_t breath, g, va = drone[v].vamp;
        if (drone[v].vgate) { va += 64; if (va > 32767) va = 32767; }   /* grows in over ~3 s */
        else { va -= 48; if (va < 0) va = 0; }                          /* withers over ~4 s */
        drone[v].vamp = va;
        if (!va) { drone[v].inc = tinc; continue; }   /* not part of the network: silent, and no glide when it grows */
        inc += (uint32_t)((int32_t)(tinc - inc) >> 7);   /* glide: ~1.5 s to settle */
        if ((int32_t)(tinc - inc) > -128 && (int32_t)(tinc - inc) < 128) inc = tinc;
        drone[v].inc = inc;
        incb = inc + (inc >> 10);               /* +1.7 cents: a slow beat between the two */
        drone[v].lfo += drone[v].lfo_inc;
        breath = 24576 + ((sine(drone[v].lfo) * 8192) >> 15);   /* 0.5 .. 1.0 */
        g = (((((DRONE_GAIN * breath) >> 15) * lvl) >> 15) * va) >> 15;
        for (i = 0; i < n; i++) {
            int32_t s;
            pa += inc;
            pb += incb;
            s = shape(tim_now, pa, pb, breath);
            if (tim_xf < 32767)
                s = (s * (int32_t)(tim_xf >> 1) + shape(tim_prev, pa, pb, breath) * (int32_t)((32767u - tim_xf) >> 1)) >> 14;   /* (signed; no overflow) */
            bus[i] += (s * g) >> 15;
        }
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
static const uint8_t RANGE_LO[DV] = {40, 52, 57, 62, 67, 72}, RANGE_HI[DV] = {52, 64, 69, 74, 79, 84};   /* E2..E3 up to C5..C6 */
static const uint8_t RANK[DV] = {0, 2, 1, 3, 4, 5};   /* the order voices grow in: the root, the middle, the low, the high, the colours */
static uint32_t grow_rng = 0x2545F491u, chord_now, key_now, moves_left, grow_ver, density = 4u, pulse_ms;

static uint32_t rnd(uint32_t n)                 /* 0 .. n-1 (xorshift32) */
{
    grow_rng ^= grow_rng << 13; grow_rng ^= grow_rng >> 17; grow_rng ^= grow_rng << 5;
    return n ? grow_rng % n : 0u;
}
static uint32_t note_inc(uint32_t note) { return INC_C9[note % 12u] >> (10u - note / 12u); }
static uint32_t cpc(uint32_t c, uint32_t k) { return (CHORD[c][k] + FIFTHS[key_now]) % 12u; }   /* chord c's k-th pitch class, in the key now */
static int alive(uint32_t v) { return RANK[v] < density; }
static int in_chord(uint32_t note, uint32_t c, uint32_t v)   /* a note voice v may hold: voices 4, 5 may hold the colour */
{
    uint32_t k, nk = v >= 4u ? 5u : 4u;
    for (k = 0; k < nk; k++) if (note % 12u == cpc(c, k)) return 1;
    return 0;
}
static int fits(uint32_t v, uint32_t c)         /* voice v's note belongs to chord c (the bass: only the root) */
{
    return v == 0u ? drone[0].note % 12u == cpc(c, 0) : in_chord(drone[v].note, c, v);
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
        for (n = RANGE_LO[v]; n <= RANGE_HI[v]; n++) {
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
    moves_left = 4u + rnd(3u);                  /* rest on the pivot a while before the new key's chords move on */
    grow_ver++;
}
static void pulse_move(void)                    /* a pulse through the network: one voice moves; now and then the chord */
{
    uint32_t v, w, n;
    pulse_ms = fm1_ms;
    if (!moves_left) {                          /* the network shifts: a neighbour on the ring */
        chord_now = (chord_now + (rnd(3u) ? 1u : 5u)) % 6u;
        moves_left = 4u + rnd(3u);
    }
    moves_left--;
    for (w = 0, v = DV; w < DV && v == DV; w++) /* a living voice outside the chord moves first, the bass before the rest */
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
    if (SEASON_S(sea)) {
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
        drone[v].note = (uint8_t)((RANGE_LO[v] + RANGE_HI[v]) / 2u);
        drone[v].vgate = 0;
        drone[v].lfo_inc = 4294967295u / (172u * (7u + rnd(13u)));   /* a breath every 7..19 s */
    }
    moves_left = 4u + rnd(3u);
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

static const char *const TIM_NAME[NT] = {"MOSS", "LOAM", "MIST", "ROOT"};
static void put(char **p, const char *a) { while (*a) *(*p)++ = *a++; }
static void put_num(char **p, uint32_t n) { if (n >= 100u) *(*p)++ = (char)('0' + n / 100u); if (n >= 10u) *(*p)++ = (char)('0' + n / 10u % 10u); *(*p)++ = (char)('0' + n % 10u); }

static void draw_info(uint32_t key, uint32_t tim, int32_t oct, uint32_t ch, int lit)   /* "KEY C   MOSS   OCT +0" */
{
    char s[32], *p = s;
    put(&p, "KEY "); put(&p, KEY_NAME[key]); put(&p, "   "); put(&p, TIM_NAME[tim]);
    put(&p, "   OCT "); *p++ = oct < 0 ? '-' : '+'; *p++ = (char)('0' + (oct < 0 ? -oct : oct));
    if (ch) { put(&p, "  CH"); put_num(&p, ch); }
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
/* the four knobs of the page: a name, its value, a bar; page 0 GROWTH, page 1 SOUND (FX switches) */
static void draw_knobs(uint32_t page, const uint8_t *sv)   /* sv: den pul shp sea cut res spc mix */
{
    static const char *const NAME[2][4] = {{"DENSITY", "PULSE", "SHAPE", "SEASON"}, {"FILTER", "RES", "SPACE", "MIX"}};
    static const uint8_t TOP[8] = {DV, NPULSE - 1u, NSHAPE - 1u, NSEASON - 1u, NCUT - 1u, NRES - 1u, NSPACE - 1u, NMIX - 1u};
    static const uint8_t BOT[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    uint32_t k;
    cv_begin(240, 50, C_BLACK);
    for (k = 0; k < 4u; k++) {
        uint32_t i = page * 4u + k, x0 = k * 60u, w;
        char v[12], *q = v;
        switch (i) {
        case 0: put_num(&q, sv[0]); break;
        case 1: if (PULSE_S[sv[1]]) { put_num(&q, PULSE_S[sv[1]]); *q++ = 's'; } else put(&q, "STILL"); break;
        case 2: put(&q, SHAPE_NAME[sv[2]]); break;
        case 3: fmt_season(&q, sv[3]); break;
        case 4: put(&q, CUT_LABEL[sv[4]]); break;
        case 5: put_num(&q, sv[5]); break;
        case 6: put(&q, SPACE_LABEL[sv[6]]); break;
        default: put_num(&q, (uint32_t)sv[7] * 5u); *q++ = '%'; break;
        }
        *q = 0;
        cv_text((int32_t)x0 + (60 - text_w(&FONT_S, NAME[page][k])) / 2, 0, &FONT_S, NAME[page][k], C_OFF);
        cv_text((int32_t)x0 + (60 - text_w(&FONT_S, v)) / 2, 18, &FONT_S, v, C_ACC);
        w = (uint32_t)(sv[i] - BOT[i]) * 46u / (uint32_t)(TOP[i] - BOT[i]);
        cv_rect((int32_t)x0 + 7, 40, 46, 3, RGB(45, 45, 45));
        if (w) cv_rect((int32_t)x0 + 7, 40, (int32_t)w, 3, C_ACC);
    }
    cv_blit(0, 148);
}
static void draw_hint(uint32_t page, uint32_t cpu)   /* the bottom line: what FX does; on the SOUND page, the CPU */
{
    char s[40], *p = s;
    if (!page) put(&p, "FX: filter + reverb");
    else {
        put(&p, "FX: back to growth");
        if (cpu) { put(&p, "   CPU "); put_num(&p, cpu); *p++ = '%'; }
    }
    *p = 0;
    draw_text_box(0, 214, 240, &FONT_S, s, C_OFF, 1);
}

static void draw_network(uint32_t den, uint32_t on, int flash)   /* density as nodes and threads: grown ones lit, the rest seeds */
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
        if (i < den) cv_rect(x - 4, y - 4, 9, 9, lit);
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
static struct { uint8_t oct, ch, tim, den, key, pul, shp, sea, cut, res, spc, mix; } set = {0, 0, 0, 4, 0, 4, 1, 0, NCUT - 1u, 0, 8, 5};   /* (key: on the circle of fifths) */   /* what survives a power-off (add yours, up to SECTOR_MAX bytes) */
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
    uint32_t midi_drawn = ~0u, lfo_drawn = ~0u, hint_drawn = ~0u, key_ms = 0, kl, fl, page = 0, cpu_pct = 0;
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
    if (settings_load(&set, sizeof set) || (int8_t)set.oct < -2 || (int8_t)set.oct > 2 || set.ch > 16u || set.tim >= NT || set.den < 1u || set.den > DV
        || set.key > 11u || set.pul >= NPULSE || set.shp >= NSHAPE || set.sea >= NSEASON
        || set.cut >= NCUT || set.res >= NRES || set.spc >= NSPACE || set.mix >= NMIX)
        set.oct = 0, set.ch = 0, set.tim = 0, set.den = 4, set.key = 0, set.pul = 4, set.shp = 1, set.sea = 0,
        set.cut = NCUT - 1u, set.res = 0, set.spc = 8, set.mix = 5;

    tim_now = tim_prev = set.tim;
    density = set.den;
    key_now = set.key;
    fx_cut = set.cut; fx_res = set.res; fx_space = set.spc; fx_mix = set.mix;
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
        if (b & (1u << BTN_FX)) page ^= 1u;    /* FX: the knobs' other page */
        for (k = 0; k < 4u; k++)                /* KNOB 1..4: GROWTH (density, pulse, shape, season) or SOUND (filter, res, space, mix) */
            if ((e = fm1_enc_take(ENC_KNOB1 + k)) != 0) {
                static const uint8_t TOPS[8] = {DV, NPULSE - 1u, NSHAPE - 1u, NSEASON - 1u, NCUT - 1u, NRES - 1u, NSPACE - 1u, NMIX - 1u};
                uint8_t *sv[8] = {&set.den, &set.pul, &set.shp, &set.sea, &set.cut, &set.res, &set.spc, &set.mix};
                uint32_t i = page * 4u + k;
                int32_t d = (int32_t)*sv[i] + e, lo = i == 0u ? 1 : 0;
                *sv[i] = (uint8_t)(d < lo ? lo : d > (int32_t)TOPS[i] ? (int32_t)TOPS[i] : d);
                if (i == 0u) set_density(set.den);
                fx_cut = set.cut; fx_res = set.res; fx_space = set.spc; fx_mix = set.mix;
                dirty = 1; t_change = fm1_ms;
            }
        if ((e = fm1_enc_take(ENC_ALGORITHM)) != 0) {   /* ALGORITHM: the timbre, round the ring */
            set.tim = (uint8_t)(((int32_t)set.tim + e % (int32_t)NT + (int32_t)NT) % (int32_t)NT);
            tim_now = set.tim;
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
        if (drone_on)                           /* the network's notes light up too */
            for (k = 0; k < DV; k++)
                if (drone[k].vgate && drone[k].note >= base && drone[k].note < base + 27u)
                    lit |= 1u << (drone[k].note - base);
        for (k = 0; k < 27u; k++)
            fm1_led_key(KEY_LED(k), lit >> k & 1u);
        fm1_led_key(BTN_FX, page);              /* lit: the knobs are on the SOUND page */
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
        if ((st = (uint32_t)(oct + 2) | (uint32_t)set.ch << 4 | (uint32_t)set.tim << 9 | key_now << 11 | kl << 15) != status_drawn) { status_drawn = st; draw_info(key_now, set.tim, oct, set.ch, kl); }
        {
            const uint8_t sv[8] = {set.den, set.pul, set.shp, set.sea, set.cut, set.res, set.spc, set.mix};
            st = page ? 1u << 31 | set.cut | (uint32_t)set.res << 5 | (uint32_t)set.spc << 9 | (uint32_t)set.mix << 13
                      : set.den | (uint32_t)set.pul << 3 | (uint32_t)set.shp << 7 | (uint32_t)set.sea << 9;
            if (st != lfo_drawn) { lfo_drawn = st; draw_knobs(page, sv); }
        }
#ifndef DINGHY_HOST
        if (fm1_ms - cpu_t >= 1000u) {          /* the audio ISR's worst run of the last second, of its 5.8 ms */
            cpu_t = fm1_ms;
            cpu_pct = cpu_max_us * 100u / 5805u;
            cpu_max_us = 0;
        }
#endif
        if ((st = page | (page ? cpu_pct << 1 : 0u)) != hint_drawn) { hint_drawn = st; draw_hint(page, cpu_pct); }
        fl = drone_on && pulse_ms && fm1_ms - pulse_ms < 300u;
        if ((st = set.den | (uint32_t)drone_on << 4 | fl << 5) != midi_drawn) { midi_drawn = st; draw_network(set.den, (uint32_t)drone_on, (int)fl); }   /* (the MIDI line's place) */
        idle_ok = 0;

        if (dirty && quiet() && fm1_ms - t_change > 2000u) {   /* 2 s after the change, when silent (an erase stalls the sound) */
            settings_save(&set, sizeof set);
            dirty = 0;
        }
    }
}
