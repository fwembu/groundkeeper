/* SPDX-License-Identifier: GPL-3.0-only */
/* Dinghy: the smallest FM-1 firmware to build your own on. A sine wave on the keys, and that is all.
 * This is the file to change. Everything below it (platform.c) is done: updates, the USB rescue,
 * MIDI (USB, the TRS jack), USB audio, keys, knobs, LEDs, the screen, flash storage; the FM-1 and the SMK-37.
 *
 *   keys       play sines, 4 voices; the notes go out over USB-MIDI too, on the MIDI channel
 *   OCT- OCT+  the octave (kept in flash)
 *   PRESETS    the spell's scale (F#5 held: CHORDS); the MIDI channel is kept in flash, tucked away for now
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
#define NV 4u                                   /* the keys' voices: they bloom, in BLOOM's timbre and envelope */
static struct {
    volatile uint32_t inc;                      /* phase step per frame: f * 2^32 / 44100 (unbent) */
    volatile int32_t gate;                      /* 1: the key is held (main loop writes) */
    volatile int32_t peak;                      /* how loud this bloom gets, Q15 (main loop writes, before trig) */
    volatile uint32_t trig;                     /* +1 = bloom now (main loop writes) */
    uint32_t tseen;                             /* (the last trig the audio saw) */
    int32_t amp, nlp;                           /* the level now, Q15; MIST's breath */
    uint32_t ph, pb, age;                       /* the two oscillators' phases */
    uint8_t stage;                              /* 1 rising, else falling */
    volatile uint8_t note, band;
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
 * Two ways to sound. Held (UMWELT DRONE): a voice that is part of the network grows in (ATTACK) and holds; one that is
 * not withers away (RELEASE). Triggered (UMWELT EVENTS, BLOOM): a voice sounds only when the main loop triggers it: it
 * rises to its peak (ATTACK), then fades (RELEASE), and leaves silence until the next one. */
#include "gk_tables.h"
#define DV 6u                                   /* the most voices the network can grow (density 6) */
#define DEN_GROW (DV + 1u)                      /* DENSITY past 6: GROW, the network lives a slow life by itself */
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
static volatile uint32_t engine;                /* 0: voices hold (DRONE); 1: voices are triggered (EVENTS, BLOOM) */
static volatile uint32_t trig_boost;            /* BLOOM: each voice twice as loud (main loop writes) */
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
    if (drone_on) { lvl += engine ? 4096 : 48; if (lvl > 32767) lvl = 32767; }   /* fade in ~4 s (a groove: at once) */
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
        if (trig_boost) g <<= 1;                /* BLOOM: a voice or two at a time, so each twice as loud */
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

/* the keys (audio ISR): each a bloom in BLOOM's timbre, attack and release, as loud as one; it rises to its peak while
 * the key is held, then falls (a key let go early falls at once). Into the bus: through FILTER and SPACE as the rest. */
#define KEY_GAIN 6900                           /* a bloom's level (DRONE_GAIN, its breath on average, BLOOM's boost) */
static volatile uint32_t key_tim = 4, key_att = 63, key_rel = 218;   /* (main loop writes) */
static int keys_audio(int32_t *bus, uint32_t n)   /* adds the keys to bus; 1 if any sounds */
{
    uint32_t v, i, tn = key_tim, att = key_att, rel = key_rel;
    int any = 0;
    for (v = 0; v < NV; v++) {
        uint32_t pa = voice[v].ph, pb = voice[v].pb, band = voice[v].band, inc, incb;
        int32_t a0 = voice[v].amp, a = a0, g0, g1, step;
        if (voice[v].trig != voice[v].tseen) { voice[v].tseen = voice[v].trig; voice[v].stage = 1; }
        if (voice[v].stage && !voice[v].gate) voice[v].stage = 0;   /* let go while rising: fall from here */
        if (voice[v].stage) {
            a += (int32_t)att;
            if (a >= voice[v].peak) { a = voice[v].peak; voice[v].stage = 0; }
        } else a -= ((a * (int32_t)rel) >> 15) + 1;
        if (a < 0) a = 0;
        voice[v].amp = a;
        if (!a && !a0) continue;                /* silent: costs nothing */
        any = 1;
        inc = (uint32_t)(((uint64_t)voice[v].inc * bend_q16) >> 16);
        incb = inc + (inc >> DETUNE[tn]);
        g0 = (((KEY_GAIN * (int32_t)TGAIN[tn]) >> 15) * a0) >> 15;
        g1 = (((KEY_GAIN * (int32_t)TGAIN[tn]) >> 15) * a) >> 15;
        step = (g1 - g0) / (int32_t)n;          /* (the gain ramps across the block: no clicks) */
        for (i = 0; i < n; i++) {
            pa += inc;
            pb += incb;
            g0 += step;
            bus[i] += (shape(tn, pa, pb, band, &voice[v].nlp, noise[i]) * g0) >> 15;
        }
        voice[v].ph = pa;
        voice[v].pb = pb;
    }
    return any;
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
    uint32_t i;
    int k;
#ifndef DINGHY_HOST
    uint32_t t0 = fm1_ticks(), us;
#endif
    if (n > BUS_N) n = BUS_N;
    for (i = 0; i < n; i++) bus[i] = 0;
    if (tim_now == 1u || tim_prev == 1u || key_tim == 1u)   /* MIST breathes: a block of noise */
        for (i = 0; i < n; i++) noise[i] = (int16_t)(xs32(&aud_rng) >> 16);
    drone_audio(bus, n);
    k = keys_audio(bus, n);
    fx_audio(bus, out, n, drone_lvl == 0 && !k);
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
    voice[pick].band = (uint8_t)(note < WT_BAND0 ? 0u : (note - WT_BAND0) / WT_BAND_STEP < WT_NBAND ? (note - WT_BAND0) / WT_BAND_STEP : WT_NBAND - 1u);
    voice[pick].age = ++ages;
    voice[pick].inc = INC_C9[note % 12u] >> (10u - note / 12u);
    voice[pick].peak = (int32_t)(18000u + vel * 115u);   /* 127 -> ~32600: as loud as a bloom */
    voice[pick].gate = 1;
    voice[pick].trig++;                         /* (last: the audio sees the note and the peak with it) */
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

/* ---- Groundkeeper's spells (main loop): the harmony the network holds, and how it moves.
 * A SCALE is a root and a mode. A SPELL is a group of scales that belong together, with a name; the groundkeeper casts
 * one from the BOOK. The season moves the harmony between a spell's scales: WHEEL walks round the circle of fifths a
 * step at a time; the other spells leap, to any of their scales. A move pivots: the new scale's chord that shares the
 * most with the one sounding takes over, so a change slips in rather than jumps; at the next pulse the harmony goes home,
 * to the new scale's first chord: the turn is felt (in STILLWATER the notes stay, only the centre moves).
 * Chords stack every other note of the scale (thirds in a seven-note scale; open, quartal sounds in a pentatonic):
 * root, three tones, and a colour (the 9th) for the two highest voices. They come in a ring that falls by a third, so
 * neighbours share three notes; the tense half-diminished and diminished chords are left out of the ring. */
#define NMODE 10u
enum { M_MAJ, M_DOR, M_PHR, M_LYD, M_MIX, M_MIN, M_HMIN, M_MMIN, M_PENT, M_MPENT };
static const uint8_t MODE_N[NMODE] = {7, 7, 7, 7, 7, 7, 7, 7, 5, 5};
static const uint8_t MODE_PC[NMODE][7] = {{0, 2, 4, 5, 7, 9, 11}, {0, 2, 3, 5, 7, 9, 10}, {0, 1, 3, 5, 7, 8, 10},
    {0, 2, 4, 6, 7, 9, 11}, {0, 2, 4, 5, 7, 9, 10}, {0, 2, 3, 5, 7, 8, 10}, {0, 2, 3, 5, 7, 8, 11}, {0, 2, 3, 5, 7, 9, 11},
    {0, 2, 4, 7, 9}, {0, 3, 5, 7, 10}};
static const char *const MODE_NAME[NMODE] = {"MAJOR", "DORIAN", "PHRYGIAN", "LYDIAN", "MIXOLYD", "MINOR", "HARM MIN",
    "MEL MIN", "PENTA", "MIN PENT"};
typedef struct { const char *name; uint8_t walk, n, off[12], mode[12]; } spell_t;   /* off: from the tonic, semitones */
#define NSPELL 6u
static const spell_t SPELL[NSPELL] = {
    {"WHEEL", 1, 12, {0, 7, 2, 9, 4, 11, 6, 1, 8, 3, 10, 5}, {0}},                       /* the twelve major keys, by fifths */
    {"STILLWATER", 0, 6, {0, 2, 4, 5, 7, 9}, {M_MAJ, M_DOR, M_PHR, M_LYD, M_MIX, M_MIN}},   /* one set of notes, six centres */
    {"HOLLOW", 0, 5, {0, 0, 0, 0, 0}, {M_MIN, M_DOR, M_HMIN, M_MMIN, M_PHR}},              /* the minors on one root */
    {"MEADOW", 0, 4, {0, 0, 0, 7}, {M_LYD, M_MAJ, M_MIX, M_LYD}},                           /* bright: lydian light */
    {"FAE RING", 0, 6, {0, 4, 8, 9, 1, 5}, {M_MAJ, M_MAJ, M_MAJ, M_MIN, M_MIN, M_MIN}},     /* keys a third apart */
    {"OLD SONG", 0, 4, {0, 7, 5, 0}, {M_PENT, M_PENT, M_PENT, M_MPENT}}};                   /* pentatonic, open */
static const char *const SPELL_SHORT[NSPELL] = {"WHEEL", "STILL", "HOLLOW", "MEADOW", "FAE", "OLDSONG"};   /* (the knob panel) */
static const uint8_t FIFTHS[12] = {0, 7, 2, 9, 4, 11, 6, 1, 8, 3, 10, 5};   /* the tonics round the circle of fifths */
static const char *const PC_NAME[12] = {"C", "Db", "D", "Eb", "E", "F", "F#", "G", "Ab", "A", "Bb", "B"};
static const char *const TONIC_NAME[12] = {"C", "G", "D", "A", "E", "B", "F#", "Db", "Ab", "Eb", "Bb", "F"};

static const uint8_t WIN_C[DV] = {46, 58, 63, 68, 73, 78};   /* each voice's window centre at full SPREAD: E2..E3 up to C5..C6 */
static const uint8_t RANK[DV] = {0, 2, 1, 3, 4, 5};   /* the order voices grow in: the root, the middle, the low, the high, the colours */
static uint32_t grow_rng = 0x2545F491u, chord_now, moves_left, grow_ver, density = 4u, pulse_ms;
static uint32_t spell_now, tonic, scale_now;    /* the spell cast, its tonic (on the circle of fifths), its scale now */
static uint8_t scale_pc[7], ring[7];            /* the scale now: its notes; its chords' degrees, in ring order */
static uint32_t scale_n, ring_n;
static int32_t spread = 8, pitch;               /* SPREAD 0..8 (8: the windows above), PITCH -12..+12 semitones */
static uint32_t chord_every = 5u;               /* CHORDS: pulses a chord (0: HOLD) */
static uint32_t bloom_t[DV];                    /* BLOOM: when each voice last bloomed (fm1_ms) */
static uint32_t eng_sel;                        /* 0 UMWELT, 1 BLOOM (SEL; UMWELT at every power-on) */
static uint32_t um_mode;                        /* UMWELT: 0 DRONE, 1 EVENTS */
static int homing;                              /* a new scale: at the next pulse, its home chord */

static uint32_t rnd(uint32_t n)                 /* 0 .. n-1 (xorshift32) */
{
    grow_rng ^= grow_rng << 13; grow_rng ^= grow_rng >> 17; grow_rng ^= grow_rng << 5;
    return n ? grow_rng % n : 0u;
}
static uint32_t note_inc(uint32_t note) { return INC_C9[note % 12u] >> (10u - note / 12u); }
static void scale_load(void)                    /* the scale now: its notes, and its ring of chords */
{
    const spell_t *sp = &SPELL[spell_now];
    uint32_t m = sp->mode[scale_now], root = (FIFTHS[tonic] + sp->off[scale_now]) % 12u, i, j;
    scale_n = MODE_N[m];
    for (i = 0; i < scale_n; i++) scale_pc[i] = (uint8_t)((root + MODE_PC[m][i]) % 12u);
    ring_n = 0;
    for (j = 0; j < scale_n; j++) {             /* falling by thirds: 0, n-2, n-4, ... */
        uint32_t d = (scale_n * 2u - 2u * j) % scale_n, k, seen = 0, t, f;
        for (k = 0; k < ring_n; k++) seen |= ring[k] == d;
        if (seen) break;
        t = (scale_pc[(d + 2u) % scale_n] + 12u - scale_pc[d]) % 12u;
        f = (scale_pc[(d + 4u) % scale_n] + 12u - scale_pc[d]) % 12u;
        if ((t == 3u || t == 4u) && f == 6u) continue;   /* (half-diminished, diminished: left out) */
        ring[ring_n++] = (uint8_t)d;
    }
}
static uint32_t cpc(uint32_t c, uint32_t k)     /* chord c's k-th pitch class: 0 root, 1..3 its tones, 4 its colour */
{
    uint32_t d = ring[c % ring_n];
    return scale_pc[(d + (k < 4u ? 2u * k : 8u)) % scale_n];
}
static const char *chord_qual(uint32_t c)       /* maj7, m7, 7, ... from the chord's intervals ("" if it has no name) */
{
    static const struct { uint8_t t, f, s; const char *q; } Q[] = {{4, 7, 11, "maj7"}, {3, 7, 10, "m7"}, {4, 7, 10, "7"},
        {3, 7, 11, "mMaj7"}, {4, 8, 11, "maj7#5"}, {4, 9, 2, "6/9"}, {5, 10, 2, "9sus"}, {5, 10, 3, "7sus"}, {5, 9, 2, "6sus"},
        {3, 10, 5, "m11"}, {4, 7, 9, "6"}, {3, 7, 9, "m6"}};
    uint32_t r = cpc(c, 0), t = (cpc(c, 1) + 12u - r) % 12u, f = (cpc(c, 2) + 12u - r) % 12u, s = (cpc(c, 3) + 12u - r) % 12u, i;
    for (i = 0; i < sizeof Q / sizeof Q[0]; i++)
        if (Q[i].t == t && Q[i].f == f && Q[i].s == s) return Q[i].q;
    return "";
}
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
static uint32_t pick_note(uint32_t v, uint32_t c, int fresh)   /* a chord note in v's window, near where v is */
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
static uint32_t grow_next;                      /* GROW: when the network next grows or withers (fm1_ms) */
static int32_t grow_dir = 1;                    /* GROW: +1 maturing, -1 withering */
static uint32_t grow_pace;                      /* GROW: the SEASON, in s (0: OFF) (main loop writes) */
static void grow_wait(void)                     /* about a step a season (60..140 %, at least 4 s); SEASON OFF: 15..35 s */
{
    uint32_t w = grow_pace ? grow_pace * (600u + rnd(801u)) : 15000u + rnd(21u) * 1000u;
    grow_next = fm1_ms + (w < 4000u ? 4000u : w);
}
static void grow_tick(int wild)                 /* GROW: now and then a voice grows in or withers, a life of minutes */
{
    if (!wild || !drone_on || (int32_t)(fm1_ms - grow_next) < 0) return;
    grow_wait();
    if (!rnd(8u)) grow_dir = -grow_dir;   /* (now and then it changes its mind) */
    if (density >= DV) grow_dir = -1;           /* full grown: it begins to wither */
    if (density <= 2u) grow_dir = 1;            /* down to the root and its third: it grows again (the root alone: a seedling) */
    set_density((uint32_t)((int32_t)density + grow_dir));
#ifdef DINGHY_HOST
    printf("%7u ms  grow: %s to %u\n", (unsigned)fm1_ms, grow_dir > 0 ? "matures" : "withers", (unsigned)density);
#endif
}
static void revoice(void)                       /* SPREAD or PITCH moved: every living voice to a chord note in its new window */
{
    uint32_t v;
    for (v = 0; v < DV; v++)
        if (alive(v)) voice_to(v, pick_note(v, chord_now, 0));
}
static void cast(uint32_t sp, uint32_t tn, uint32_t sc)   /* to this spell, tonic and scale, pivoting on the chord sounding */
{
    uint32_t old[4], c, k, j, best = 0, bests = 0;
    for (k = 0; k < 4u; k++) old[k] = ring_n ? cpc(chord_now, k) : 0u;
    spell_now = sp; tonic = tn; scale_now = sc % SPELL[sp].n;
    scale_load();
    for (c = 0; c < ring_n; c++) {              /* the new scale's chord sharing the most with the one sounding: often the same */
        uint32_t s = 0;
        for (k = 0; k < 4u; k++) for (j = 0; j < 4u; j++) s += cpc(c, k) == old[j];
        s = s * 2u + (cpc(c, 0) == old[0]);     /* (the same root breaks a tie) */
        if (s > bests) { bests = s; best = c; }
    }
    chord_now = best;
#ifdef DINGHY_HOST
    printf("%7u ms  cast: %s, %s %s, pivot %s%s\n", (unsigned)fm1_ms, SPELL[sp].name, PC_NAME[scale_pc[0]],
           MODE_NAME[SPELL[sp].mode[scale_now]], PC_NAME[cpc(best, 0)], chord_qual(best));
#endif
    homing = 1;                                 /* (on the pivot until the next pulse; then home) */
    grow_ver++;
}
static int32_t season_dir = 1;                  /* WHEEL: +1 travelling sharpwards round the circle, -1 flatwards */
static void season_turn(void)                   /* the season moves the harmony: WHEEL a step (mostly onward), the rest a leap */
{
    const spell_t *sp = &SPELL[spell_now];
    uint32_t to;
    if (sp->n < 2u) return;
    if (sp->walk) {
        if (!rnd(3u)) season_dir = -season_dir;
        to = (scale_now + (season_dir > 0 ? 1u : sp->n - 1u)) % sp->n;
    } else
        to = (scale_now + 1u + rnd(sp->n - 1u)) % sp->n;   /* any other of the spell's scales */
    cast(spell_now, tonic, to);
}

static int32_t roots_dir = 1;                   /* ROOTS: +1 sharpwards round the circle of fifths, -1 flatwards */
static void roots_turn(void)                    /* ROOTS: the tonic a fifth along (mostly onward), the scale kept, pivoting */
{
    if (!rnd(3u)) roots_dir = -roots_dir;
    cast(spell_now, (tonic + (roots_dir > 0 ? 1u : 11u)) % 12u, scale_now);
}

/* the sound events: a STAB (UMWELT, every living voice at once) or a BLOOM (one voice), on a hit of the cycle */
static void stab(int accent)
{
    uint32_t v;
    for (v = 0; v < DV; v++)
        if (alive(v)) {
            drone[v].peak = accent ? 32767 : 23000 + (int32_t)rnd(5000u);
            drone[v].trig++;
        }
}
static void bloom(int accent)                   /* one living voice blooms on a note of the chord: a free one, else the oldest */
{
    uint32_t v, n = 0, pick = DV;
    for (v = 0; v < DV; v++)                    /* a voice that has nearly faded is free */
        if (alive(v) && drone[v].vamp < 1500) n++;
    if (n) {
        n = rnd(n);
        for (v = 0; v < DV; v++)
            if (alive(v) && drone[v].vamp < 1500 && !n--) { pick = v; break; }
    } else
        for (v = 0; v < DV; v++)
            if (alive(v) && (pick == DV || (int32_t)(bloom_t[v] - bloom_t[pick]) < 0)) pick = v;
    if (pick == DV) return;
    voice_to(pick, pick_note(pick, chord_now, 1));
    drone[pick].peak = accent ? 32767 : 18000 + (int32_t)rnd(12000u);   /* each bloom a little different */
    bloom_t[pick] = fm1_ms;
    drone[pick].trig++;                         /* (last: the audio starts it with the note and peak above) */
}
static void pulse_move(void)                    /* a pulse through the network: now and then the chord; a voice moves */
{
    uint32_t v, w, n;
    pulse_ms = fm1_ms;
    if (homing) {                               /* a new scale: home (also with CHORDS on HOLD) */
        homing = 0;
        chord_now = 0;
        moves_left = new_moves();
    } else if (chord_every && !moves_left) {    /* the network shifts: a neighbour on the ring (CHORDS: HOLD never) */
        chord_now = (chord_now + (rnd(3u) ? 1u : ring_n - 1u)) % ring_n;
        moves_left = new_moves();
    }
    if (moves_left) moves_left--;
#ifdef DINGHY_HOST
    printf("%7u ms  chord %s%s\n", (unsigned)fm1_ms, PC_NAME[cpc(chord_now, 0)], chord_qual(chord_now));
#endif
    if (eng_sel) return;                        /* (BLOOM picks its notes as it blooms) */
    for (w = 0, v = DV; w < DV && v == DV; w++) /* a living voice outside the chord (or its window) moves first, the bass first */
        if (alive(w) && !fits(w, chord_now)) v = w;
    if (v < DV) { voice_to(v, pick_note(v, chord_now, 0)); return; }
    for (w = 1, n = 0; w < DV; w++) n += (uint32_t)alive(w);
    if (!n) return;                             /* only the root: nothing to drift */
    n = rnd(n);                                 /* all in the chord: one living upper voice drifts to a new voicing */
    for (w = 1; w < DV; w++)
        if (alive(w) && !n--) { voice_to(w, pick_note(w, chord_now, 1)); return; }
}

/* ---- time (main loop): TEMPO in BPM; the PULSE and SEASON waves; the CYCLE.
 * PULSE moves the voicings, a move each time its wave rises through the middle; its length is in beats (1/4 .. 64).
 * SINE: regular, on the beat; RANDOM: a smooth random wave (a new target each quarter cycle), so the moves come
 * irregularly; MIX: the two summed. SEASON (OFF, 15 s .. 10 min in 15 s steps) is a slow random wave: each time it
 * peaks or dips, the season turns (season_turn). Its cycle is 0.7 of the setting: a turn about once per setting.
 * The CYCLE is the groove: STEPS steps of RATE (1/4 .. 1/32, triplets too), HITS of them spread as evenly as they go
 * (a Euclidean rhythm), turned by ROTATE; the first step is the accent. Each hit is a STAB or a BLOOM. */
typedef struct { uint32_t ph; int32_t a, b; uint8_t seg, armed, zone; } lfo_t;   /* seg: log2 of random targets a cycle */
static lfo_t pulse = {0, 0, 0, 2, 0, 0}, season = {0, 0, 0, 1, 0, 0}, roots = {0, 0, 0, 1, 0, 0};
static uint32_t lfo_t0;
#define NPULSE 15u
static const uint16_t PULSE_Q[NPULSE] = {0, 1, 2, 4, 8, 12, 16, 24, 32, 48, 64, 96, 128, 192, 256};   /* quarter beats; 0: STILL */
static const char *const PULSE_LABEL[NPULSE] = {"STILL", "1/4", "1/2", "1", "2", "3", "4", "6", "8", "12", "16", "24", "32", "48", "64"};
#define NSHAPE 3u
static const char *const SHAPE_NAME[NSHAPE] = {"SINE", "MIX", "RANDOM"};
#define NSEASON 57u                             /* OFF, then 5 s .. 2 min in 5 s steps, then 15 s steps to 10 min */
#define SEASON_S(i) ((uint32_t)(i) <= 24u ? (uint32_t)(i) * 5u : 120u + ((uint32_t)(i) - 24u) * 15u)
#define NRATE 6u
static const uint8_t RATE_N[NRATE] = {1, 2, 3, 4, 6, 8};   /* cycle steps a beat */
static const char *const RATE_LABEL[NRATE] = {"1/4", "1/8", "1/8T", "1/16", "16T", "1/32"};

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
    pulse.ph = season.ph = roots.ph = 0;
    pulse.a = pulse.b = season.a = season.b = roots.a = roots.b = 0;
    pulse.armed = season.zone = roots.zone = 0;
    roots_dir = rnd(2u) ? 1 : -1;
    lfo_target(&pulse);
    season_dir = rnd(2u) ? 1 : -1;
    lfo_t0 = fm1_ms;
}
static int lfo_turn(lfo_t *l, uint32_t dt, uint32_t sea)   /* a season's slow random wave: 1 when it turns */
{
    int32_t v;
    if (lfo_advance(l, dt, SEASON_S(sea) * 700u)) lfo_target(l);
    if (!sea) return 0;
    v = lfo_random(l);                          /* a turn: the wave swings well up or well down (back near the middle */
    if (l->zone != 1u && v > 16384) { l->zone = 1; return 1; }   /*  before the same side turns it again) */
    if (l->zone != 2u && v < -16384) { l->zone = 2; return 1; }
    if ((l->zone == 1u && v < 8192) || (l->zone == 2u && v > -8192)) l->zone = 0;
    return 0;
}
static int lfo_tick(uint32_t bpm, uint32_t pul, uint32_t shp, uint32_t sea, uint32_t roo)   /* 1: the season turned, 2: ROOTS */
{
    uint32_t dt = fm1_ms - lfo_t0;
    int32_t v;
    int moved = 0;
    lfo_t0 = fm1_ms;
    if (!drone_on || !dt) return 0;
    if (dt > 500u) dt = 500u;                   /* (after a stall: no leap) */
    if (lfo_advance(&pulse, dt, PULSE_Q[pul] * 15000u / bpm)) lfo_target(&pulse);
    if (PULSE_Q[pul]) {
        v = shp == 0u ? sine(pulse.ph) : shp == 2u ? lfo_random(&pulse) : (sine(pulse.ph) + lfo_random(&pulse)) >> 1;
        if (v < -4000) pulse.armed = 1;         /* (below the middle first: one move per rise) */
        else if (v > 0 && pulse.armed) { pulse.armed = 0; pulse_move(); }
    }
    if (lfo_turn(&season, dt, sea)) { season_turn(); moved |= 1; }
    if (lfo_turn(&roots, dt, roo)) { roots_turn(); moved |= 2; }
    return moved;
}

static uint32_t cyc_next, cyc_step, hit_ms;     /* the cycle: the next step's time (ms x16), its index; the last hit */
static int euclid(uint32_t i, uint32_t steps, uint32_t hits, uint32_t rot)   /* step i is a hit */
{
    if (!hits) return 0;
    if (hits >= steps) return 1;
    return ((i + steps - rot % steps) % steps) * hits % steps < hits;
}
static void cycle_tick(uint32_t bpm, uint32_t rate, uint32_t steps, uint32_t hits, uint32_t rot, uint32_t chance)
{
    uint32_t now = fm1_ms << 4, len = 960000u / (bpm * RATE_N[rate]);   /* a step, in ms x16 */
    if (!drone_on || (!eng_sel && !um_mode)) { cyc_next = now; cyc_step = 0; return; }   /* (UMWELT DRONE: no cycle) */
    if ((int32_t)(now - cyc_next) > 16000) cyc_next = now;   /* (after a stall: back on time, no burst) */
    while ((int32_t)(now - cyc_next) >= 0) {
        if (cyc_step >= steps) cyc_step = 0;
        if (euclid(cyc_step, steps, hits, rot) && (!eng_sel || rnd(20u) < chance)) {
            hit_ms = fm1_ms;
            if (eng_sel) bloom(cyc_step == 0u);
            else stab(cyc_step == 0u);
        }
        cyc_step = (cyc_step + 1u) % steps;
        cyc_next += len;
    }
}

static void grow_start(void)                    /* PLAY: plant the network on the scale's home chord */
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
    homing = 0;
    lfo_reset();
    cyc_next = fm1_ms << 4; cyc_step = 0;
    drone_on = 1;
    set_density(density);
}
static void grow_stop(void)                     /* PLAY again: the whole network fades back into the ground */
{
    drone_on = 0;                               /* (the voices keep their gates: the master fade does it) */
    grow_ver++;
}

/* ---- the settings: one byte each, kept in flash; the F_ order is the struct's.
 * Each engine has its own timbre and cycle; DRONE, EVENTS and BLOOM each their own attack and release. The engine is
 * not kept: every power-on starts in UMWELT. */
#define NCHG 11u
static const uint8_t CHG_N[NCHG] = {0, 1, 2, 3, 4, 5, 6, 8, 10, 12, 16};   /* CHORDS: pulses a chord; 0: HOLD */
#define NATT 14u
#define NREL 14u
static struct {
    uint8_t oct, ch, den, pul, shp, sea, cut, res, spc, mix, spr, pit, chg, his, ton, crk, wob, bpm, spl, tnc, scl, umd, chc;
    uint8_t tim[2], att[3], rel[3], stp[2], hit[2], rot[2], rat[2], roo;
} set;
enum { F_OCT, F_CH, F_DEN, F_PUL, F_SHP, F_SEA, F_CUT, F_RES, F_SPC, F_MIX, F_SPR, F_PIT, F_CHG, F_HIS, F_TON, F_CRK, F_WOB,
       F_BPM, F_SPL, F_TNC, F_SCL, F_UMD, F_CHC, F_TIM, F_TIM1, F_ATT, F_ATT1, F_ATT2, F_REL, F_REL1, F_REL2,
       F_STP, F_STP1, F_HIT, F_HIT1, F_ROT, F_ROT1, F_RAT, F_RAT1, F_ROO, F_N, F_CPU = F_N };
#define CFG ((uint8_t *)&set)
static const uint8_t CFG_MIN[F_N] = {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t CFG_MAX[F_N] = {255, 16, DEN_GROW, NPULSE - 1u, NSHAPE - 1u, NSEASON - 1u, NCUT - 1u, NRES - 1u, NSPACE - 1u,
    NMIX - 1u, 8, 24, NCHG - 1u, 20, 10, 20, 10, 200, NSPELL - 1u, 11, 11, 1, 20, NT - 1u, NT - 1u, NATT - 1u, NATT - 1u,
    NATT - 1u, NREL - 1u, NREL - 1u, NREL - 1u, 32, 32, 32, 32, 31, 31, NRATE - 1u, NRATE - 1u, NSEASON - 1u};
static const uint8_t CFG_DEF[F_N] = {0, 0, 4, 8, 1, 0, NCUT - 1u, 0, 8, 5, 8, 12, 5, 0, 5, 0, 0, 90, 0, 0, 0, 0, 20,
    0, 4,                                       /* timbre: UMWELT MOSS, BLOOM REED */
    10, 1, 4,  10, 4, 8,                        /* attack, release: DRONE 3 s, 6 s; EVENTS 50 ms, 750 ms; BLOOM 350 ms, 3 s */
    7, 9, 3, 4, 0, 0, 1, 1,                     /* cycles: UMWELT 3 in 7, BLOOM 4 in 9, in eighths */
    0};                                         /* ROOTS: OFF */
static const char *const F_NAME[F_N + 1] = {"OCT", "CH", "DENSITY", "PULSE", "SHAPE", "SEASON", "FILTER", "RES", "SPACE", "MIX",
    "SPREAD", "PITCH", "CHORDS", "HISS", "TONE", "CRACKLE", "WOBBLE", "TEMPO", "SPELL", "TONIC", "SCALE", "MODE", "CHANCE",
    "TIMBRE", "TIMBRE", "ATTACK", "ATTACK", "ATTACK", "RELEASE", "RELEASE", "RELEASE", "STEPS", "STEPS", "HITS", "HITS",
    "ROTATE", "ROTATE", "RATE", "RATE", "ROOTS", "CPU"};
/* the keys: 0..23 play (F3..E5, in BLOOM's sound); the top three (F5, F#5, G5) are performance keys */
#define KEY_PLAY 24u
#define PK_ALT 25u                              /* F#5: while held, SELECT, PRESETS and ALGORITHM turn something else */
#define PK_UMWELT 26u                           /* G5, the top key: UMWELT while held (BLOOM's cycles wait) */
static uint32_t um_hold, um_back;               /* the UMWELT key is held; the engine to go back to */
/* the pages: HOME; EDIT 1, 2 (the engine's own: SEL picks the engine); LFO; FX 1, 2; the BOOK (GLO) */
enum { P_HOME, P_ED1, P_ED2, P_LFO, P_FX1, P_FX2, P_BOOK, NPAGE };
static uint32_t env_ix(void) { return eng_sel ? 2u : set.umd; }   /* the envelope in use: DRONE, EVENTS or BLOOM */
static uint32_t page_field(uint32_t page, uint32_t k)
{
    static const uint8_t PF[NPAGE][4] = {{F_SPR, F_PIT, F_PUL, F_UMD},   /* (HOME's KNOB 4 is the engine: engine_turn) */ {F_UMD, F_TIM, F_ATT, F_REL}, {F_STP, F_HIT, F_ROT, F_RAT},
        {F_BPM, F_PUL, F_SHP, F_SEA}, {F_CUT, F_RES, F_SPC, F_MIX}, {F_HIS, F_TON, F_CRK, F_WOB}, {F_SPL, F_TNC, F_SCL, F_ROO}};
    static const uint8_t BLOOM_ED1[4] = {F_TIM, F_ATT, F_REL, F_CHC};
    uint32_t f = page == P_ED1 && eng_sel ? BLOOM_ED1[k] : PF[page][k];
    if (f == F_ATT || f == F_REL) return f + env_ix();
    if (f == F_TIM || f == F_STP || f == F_HIT || f == F_ROT || f == F_RAT) return f + eng_sel;
    return f;
}
static uint32_t cfg_max(uint32_t f)             /* the top of a setting, some of them by another */
{
    if (f == F_SCL) return SPELL[set.spl].n - 1u;
    if (f == F_HIT || f == F_HIT1) return CFG[f - F_HIT + F_STP];
    if (f == F_ROT || f == F_ROT1) return CFG[f - F_ROT + F_STP] - 1u;
    return CFG_MAX[f];
}
static void apply(void)                         /* the settings to the sound */
{
    fx_cut = set.cut; fx_res = set.res; fx_space = set.spc; fx_mix = set.mix;
    tim_now = set.tim[eng_sel];
    grow_pace = SEASON_S(set.sea);
    key_tim = set.tim[1];                       /* the keys: BLOOM's timbre and envelope, whichever engine plays */
    key_att = ATT_INC[set.att[2]];
    key_rel = REL_K[set.rel[2]];
    um_mode = set.umd;
    engine = eng_sel || set.umd;                /* the audio: 0 hold (DRONE), 1 triggered (EVENTS, BLOOM) */
    trig_boost = eng_sel;
    env_att = ATT_INC[set.att[env_ix()]];
    env_rel = REL_K[set.rel[env_ix()]];
    spread = set.spr;
    pitch = (int32_t)set.pit - 12;
    chord_every = CHG_N[set.chg];
    tex_hiss = HISS_G[set.his]; tex_tone = set.ton; tex_crk = CRK_T[set.crk];
    wob_depth = WOB_D[set.wob];
}

/* ---- the screen: the title; the spell's sigil on the left; the scale, the chord and the network on the right; the
 * spell, engine and timbre; the page's four knobs; the pages as tabs */
#define C_ACC RGB(120, 170, 90)                 /* moss green */
#define C_KEY RGB(210, 210, 210)
#define C_OFF RGB(90, 90, 90)
#define C_FAINT RGB(50, 62, 46)
#define C_GLOW RGB(190, 235, 150)
#define KEY0 53u                                /* key 0 = F3 */
static const char *const TIM_NAME[NT] = {"MOSS", "MIST", "CHOIR", "ORGAN", "REED"};
static void put(char **p, const char *a) { while (*a) *(*p)++ = *a++; }
static void put_num(char **p, uint32_t n) { if (n >= 100u) *(*p)++ = (char)('0' + n / 100u); if (n >= 10u) *(*p)++ = (char)('0' + n / 10u % 10u); *(*p)++ = (char)('0' + n % 10u); }
static void put_signed(char **p, int32_t n) { if (n) *(*p)++ = n < 0 ? '-' : '+'; put_num(p, (uint32_t)(n < 0 ? -n : n)); }

/* sigils: each spell's scales are points on its own figure; the scale now is lit (and glows a moment after a turn) */
static int32_t sx(uint32_t a, int32_t r) { return (sine(a + 0x40000000u) * r) >> 15; }   /* r cos a (a: a full turn is 2^32) */
static int32_t sy(uint32_t a, int32_t r) { return (sine(a) * r) >> 15; }                 /* r sin a */
static void cv_ring(int32_t cx, int32_t cy, int32_t r, uint16_t c, uint32_t dots)   /* a circle; dots: every other segment */
{
    uint32_t i;
    for (i = 0; i < 48u; i++) {
        uint32_t a = i * (4294967295u / 48u), b = a + 4294967295u / 48u;
        if (!dots || !(i & 1u)) cv_line(cx + sx(a, r), cy + sy(a, r), cx + sx(b, r), cy + sy(b, r), c);
    }
}
static void draw_sigil(uint32_t sp, uint32_t cur, int glow)
{
    const uint32_t n = SPELL[sp].n, Q = 0x40000000u, W = 100, H = 96;
    int32_t px[12], py[12], cx = 50, cy = 48;
    uint32_t i;
    uint16_t hi = glow ? C_GLOW : C_ACC;
    cv_begin(W, H, C_BLACK);
    switch (sp) {
    case 0:                                     /* WHEEL: a wheel of twelve, the hand on the key now */
        cv_ring(cx, cy, 40, C_FAINT, 0);
        cv_ring(cx, cy, 4, C_FAINT, 0);
        for (i = 0; i < n; i++) {
            uint32_t a = i * (4294967295u / 12u) - Q;
            px[i] = cx + sx(a, 40); py[i] = cy + sy(a, 40);
            cv_line(cx + sx(a, 34), cy + sy(a, 34), px[i], py[i], C_FAINT);
        }
        cv_line(cx, cy, cx + sx(cur * (4294967295u / 12u) - Q, 33), cy + sy(cur * (4294967295u / 12u) - Q, 33), hi);
        break;
    case 1:                                     /* STILLWATER: still lines of water, the scales along the middle one */
        for (i = 0; i < 3u; i++) {
            uint32_t x;
            for (x = 6; x < 94u; x++)
                cv_pset((int32_t)x, 28 + (int32_t)i * 20 + (sine(x * 0x05000000u + i * Q) * 3 >> 15), i == 1u ? C_ACC : C_FAINT);
        }
        for (i = 0; i < n; i++) { px[i] = 14 + (int32_t)i * 14; py[i] = 48 + (sine((uint32_t)px[i] * 0x05000000u + Q) * 3 >> 15); }
        break;
    case 2:                                     /* HOLLOW: a bowl under a dark moon */
        cv_ring(cx, 30, 12, C_FAINT, 0);
        cv_ring(cx + 5, 27, 10, C_FAINT, 1);
        for (i = 0; i < n; i++) {
            uint32_t a = 0x0E38E38Eu + i * (0x638E38E3u / (n - 1u));   /* 20 .. 160 degrees: the lower half, a bowl */
            px[i] = cx + sx(a, 40); py[i] = 44 + sy(a, 40);
            if (i) cv_line(px[i - 1u], py[i - 1u], px[i], py[i], C_FAINT);
        }
        break;
    case 3:                                     /* MEADOW: a sun, its rays the scales */
        cv_ring(cx, cy, 10, C_ACC, 0);
        for (i = 0; i < 8u; i++) {
            uint32_t a = i * (Q / 2u) + Q / 4u;
            cv_line(cx + sx(a, 15), cy + sy(a, 15), cx + sx(a, i & 1u ? 24 : 32), cy + sy(a, i & 1u ? 24 : 32), C_FAINT);
        }
        for (i = 0; i < n; i++) { uint32_t a = i * Q + Q / 2u; px[i] = cx + sx(a, 38); py[i] = cy + sy(a, 38); }
        break;
    case 4:                                     /* FAE RING: a ring of mushrooms */
        cv_ring(cx, cy + 4, 34, C_FAINT, 1);
        for (i = 0; i < n; i++) {
            uint32_t a = i * (4294967295u / 6u) - Q;
            px[i] = cx + sx(a, 34); py[i] = cy + 4 + sy(a, 34);
            cv_rect(px[i], py[i] + 3, 1, 5, C_FAINT);   /* the stem; the cap is the point */
        }
        break;
    default:                                    /* OLD SONG: a cairn of stones */
        for (i = 0; i < n; i++) {
            int32_t w = 64 - (int32_t)i * 12, y = 80 - (int32_t)i * 16;
            cv_rect(cx - w / 2, y - 5, w, 11, i == cur ? hi : C_FAINT);
            px[i] = cx; py[i] = y;
        }
        cv_blit(4, 24);
        return;
    }
    for (i = 0; i < n; i++) {                   /* the scales: dim seeds, the one now lit */
        if (i == cur) cv_rect(px[i] - 4, py[i] - 4, 9, 9, hi);
        else cv_rect(px[i] - 2, py[i] - 2, 5, 5, sp == 4u ? C_ACC : C_OFF);
    }
    cv_blit(4, 24);
}
static void draw_scale(int lit)                 /* "D DORIAN" over the chord (green a moment after the season turns) */
{
    char s[24], *p = s;
    put(&p, PC_NAME[scale_pc[0]]); *p++ = ' '; put(&p, MODE_NAME[SPELL[spell_now].mode[scale_now]]);
    *p = 0;
    cv_begin(132, 16, C_BLACK);
    cv_text((132 - text_w(&FONT_S, s)) / 2, 0, &FONT_S, s, lit ? C_ACC : C_OFF);
    cv_blit(108, 28);
}
static void draw_note(void)                     /* the chord: its root big, the accidental and the quality small */
{
    cv_begin(132, 32, C_BLACK);
    if (drone_on) {
        const char *nm = PC_NAME[cpc(chord_now, 0)], *q = chord_qual(chord_now);
        char root[2] = {nm[0], 0};
        int32_t x = (132 - (text_w(&FONT_L, root) + text_w(&FONT_S, nm + 1) + 2 + text_w(&FONT_S, q))) / 2;
        x = cv_text(x, 0, &FONT_L, root, C_ACC);
        x = cv_text(x, 1, &FONT_S, nm + 1, C_ACC) + 2;   /* b or #, raised */
        cv_text(x, 15, &FONT_S, q, C_ACC);
    } else
        cv_text((132 - text_w(&FONT_S, "press PLAY")) / 2, 9, &FONT_S, "press PLAY", C_OFF);
    cv_blit(108, 48);
}
static void draw_network(uint32_t den, uint32_t on, int flash, uint32_t sounding, int wild)   /* nodes and threads: grown ones lit; bit i: node i sounds; wild: GROW */
{
    static const int8_t DY[DV] = {2, -5, 4, -3, 5, -4};   /* a little unevenness, like something that grew */
    uint32_t i;
    uint16_t lit = flash ? C_GLOW : on ? C_ACC : RGB(70, 90, 60), seed = wild ? RGB(60, 82, 50) : RGB(55, 55, 55);   /* a pulse: brighter a moment; GROW: the seeds green */
    cv_begin(132, 26, C_BLACK);
    for (i = 0; i + 1u < DV; i++) {
        int32_t x0 = 16 + (int32_t)i * 20, x1 = x0 + 20, y0 = 13 + DY[i], y1 = 13 + DY[i + 1u];
        uint16_t c = i + 1u < den ? lit : seed;
        cv_line(x0, y0, x1, y1, c);
        if (i + 1u < den) cv_line(x0, y0 + 1, x1, y1 + 1, c);
    }
    for (i = 0; i < DV; i++) {
        int32_t x = 16 + (int32_t)i * 20, y = 13 + DY[i];
        if (i < den) cv_rect(x - 4, y - 4, 9, 9, sounding >> i & 1u ? C_GLOW : lit);
        else cv_rect(x - 2, y - 2, 5, 5, seed);
    }
    cv_blit(108, 88);
}
static void draw_info(void)                     /* "WHEEL   UMWELT DRONE   MOSS" */
{
    char s[44], *p = s;
    put(&p, SPELL[spell_now].name); put(&p, "  ");
    put(&p, eng_sel ? "BLOOM" : set.umd ? "UM EVENTS" : "UM DRONE"); put(&p, "  "); put(&p, TIM_NAME[set.tim[eng_sel]]);
    *p = 0;
    draw_text_box(0, 128, 240, &FONT_S, s, C_OFF, 1);
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
    case F_DEN: if (v == DEN_GROW) put(q, "GROW"); else put_num(q, v); break;
    case F_CHG: if (CHG_N[v]) put_num(q, CHG_N[v]); else put(q, "HOLD"); break;
    case F_UMD: put(q, v ? "EVENTS" : "DRONE"); break;
    case F_TIM: case F_TIM1: put(q, TIM_NAME[v]); break;
    case F_ATT: case F_ATT1: case F_ATT2: put(q, ATT_LABEL[v]); break;
    case F_REL: case F_REL1: case F_REL2: put(q, REL_LABEL[v]); break;
    case F_PUL: put(q, PULSE_LABEL[v]); break;
    case F_SHP: put(q, SHAPE_NAME[v]); break;
    case F_SEA: case F_ROO: fmt_season(q, v); break;
    case F_CUT: put(q, CUT_LABEL[v]); break;
    case F_SPC: put(q, SPACE_LABEL[v]); break;
    case F_MIX: case F_CHC: put_num(q, v * 5u); *(*q)++ = '%'; break;
    case F_TON: if (v == 5u) put(q, "WHITE"); else put_signed(q, (int32_t)v - 5); break;
    case F_HIS: case F_CRK: case F_WOB: if (v) put_num(q, v); else put(q, "OFF"); break;
    case F_SPL: put(q, SPELL_SHORT[v]); break;
    case F_TNC: put(q, TONIC_NAME[v]); break;
    case F_SCL: put_num(q, v + 1u); *(*q)++ = '/'; put_num(q, SPELL[set.spl].n); break;
    case F_HIT: case F_HIT1: put_num(q, v < CFG[f - F_HIT + F_STP] ? v : CFG[f - F_HIT + F_STP]); break;
    case F_RAT: case F_RAT1: put(q, RATE_LABEL[v]); break;
    case F_CPU: if (cpu) { put_num(q, cpu); *(*q)++ = '%'; } else put(q, "--"); break;
    default: put_num(q, v); break;
    }
}
static void draw_knobs(uint32_t page, uint32_t cpu)   /* the page's four knobs: a name, its value, a bar */
{
    uint32_t k;
    cv_begin(240, 50, C_BLACK);
    for (k = 0; k < 4u; k++) {
        uint32_t f = page_field(page, k), x0 = k * 60u;
        char v[16], *q = v;
        fmt_field(&q, f, cpu);
        *q = 0;
        cv_text((int32_t)x0 + (60 - text_w(&FONT_S, F_NAME[f])) / 2, 0, &FONT_S, F_NAME[f], C_OFF);
        cv_text((int32_t)x0 + (60 - text_w(&FONT_S, v)) / 2, 18, &FONT_S, v, f == F_CPU ? C_OFF : C_ACC);
        if (f < F_N && cfg_max(f) > CFG_MIN[f]) {
            uint32_t val = CFG[f] > cfg_max(f) ? cfg_max(f) : CFG[f];
            uint32_t w = (val - CFG_MIN[f]) * 46u / (cfg_max(f) - CFG_MIN[f]);
            cv_rect((int32_t)x0 + 7, 40, 46, 3, RGB(45, 45, 45));
            if (w) cv_rect((int32_t)x0 + 7, 40, (int32_t)w, 3, C_ACC);
        }
    }
    cv_blit(0, 150);
}
static void draw_tabs(uint32_t page)            /* the bottom line: the pages, the one shown lit */
{
    static const char *const TAB[5] = {"HOME", "EDIT", "LFO", "FX", "BOOK"};
    static const uint8_t TAB_OF[NPAGE] = {0, 1, 1, 2, 3, 3, 4};
    uint32_t i;
    int32_t x, w = 0;
    for (i = 0; i < 5u; i++) w += text_w(&FONT_S, TAB[i]) + (i ? 14 : 0) + ((i == 1u || i == 3u) ? text_w(&FONT_S, "2") : 0);
    x = (240 - w) / 2;
    cv_begin(240, 16, C_BLACK);
    for (i = 0; i < 5u; i++) {
        uint16_t c = i == TAB_OF[page] ? C_ACC : RGB(65, 65, 65);
        x = cv_text(x, 0, &FONT_S, TAB[i], c);
        if (i == 1u || i == 3u) x = cv_text(x, 0, &FONT_S, (page == P_ED2 || page == P_FX2) && i == TAB_OF[page] ? "2" : " ", c);
        x += 14;
    }
    cv_blit(0, 214);
}

/* ---- HOME: a living page. The spell's sigil, large and gently alive; a new spell crumbles it to seeds that fall to
 * the soil and grow the next one. Below it the network, as mycelium in the ground: DENSITY the nodes grown, SPREAD
 * how far its threads reach, PITCH how deep it lies (above the soil: high). A node shines as its voice sounds; a pulse
 * runs out along the threads. Words only for a moment: what was just turned, or the scale after the season turns. */
#define SG_X 40u                                /* the sigil: 160 x 128 at (40, 20) */
#define SG_Y 20u
#define SG_W 160
#define SG_H 128
#define SG_CX 80
#define SG_CY 62
#define MY_Y 150u                               /* the ground: 240 x 66 at (0, 150) */
#define MY_H 64
#define MY_SOIL 22                              /* the soil line in it */
#define T_OUT 700u                              /* a new spell: the old one crumbles, its seeds rest, the new one grows */
#define T_REST 250u
#define T_IN 1100u
#define T_MOVE 450u                             /* a new scale: the light moves along the figure */
#define F_ENG F_N                               /* (HOME, KNOB 4: DRONE, EVENTS, BLOOM; not a setting of its own) */

static uint16_t lite(int32_t l)                 /* a light: 0 dark .. 160 moss .. 255 pale green */
{
    int32_t r, g, b;
    if (l <= 0) return C_BLACK;
    if (l > 255) l = 255;
    if (l < 160) { r = 120 * l / 160; g = 170 * l / 160; b = 90 * l / 160; }
    else { l -= 160; r = 120 + 70 * l / 95; g = 170 + 65 * l / 95; b = 90 + 60 * l / 95; }
    return RGB(r, g, b);
}
static uint32_t hsh(uint32_t x) { x *= 0x9E3779B1u; x ^= x >> 15; x *= 0x85EBCA6Bu; x ^= x >> 13; return x; }
static int32_t ease(int32_t k) { return k <= 0 ? 0 : k >= 256 ? 256 : (k * k * (768 - 2 * k)) >> 16; }   /* 0..256, smooth */
static uint32_t per(uint32_t t, uint32_t ms) { return t * (4294967295u / ms); }   /* a phase that turns once every ms */

/* the sigil's pen: everything may be crumbling (dithered away) or growing up from the soil */
static uint32_t sg_mode, sg_amt;                /* 0 whole; 1 crumbling, sg_amt 0..256 gone; 2 growing, 0..320 grown */
static int sg_on(int32_t x, int32_t y)
{
    uint32_t h, d;
    if (!sg_mode) return 1;
    h = hsh((uint32_t)(x * 977 + y * 131)) & 255u;
    if (sg_mode == 1u) return h >= sg_amt;
    d = (uint32_t)((x - SG_CX) * (x - SG_CX) + (y - SG_H) * (y - SG_H)) * 256u / 23000u;   /* from the soil, below the middle */
    return d + (h >> 2) < sg_amt;
}
static void sg_pset(int32_t x, int32_t y, uint16_t c) { if (sg_on(x, y)) cv_pset(x, y, c); }
static void sg_line(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint16_t c)
{
    int32_t dx = x1 > x0 ? x1 - x0 : x0 - x1, sx_ = x0 < x1 ? 1 : -1, dy = y1 > y0 ? y0 - y1 : y1 - y0, sy_ = y0 < y1 ? 1 : -1;
    int32_t err = dx + dy, guard = 600;
    while (guard--) {
        int32_t e2 = 2 * err;
        sg_pset(x0, y0, c);
        if (x0 == x1 && y0 == y1) break;
        if (e2 >= dy) { err += dy; x0 += sx_; }
        if (e2 <= dx) { err += dx; y0 += sy_; }
    }
}
static void sg_rect(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t c, int round)   /* filled; round: corners off */
{
    int32_t i, j;
    for (j = 0; j < h; j++)
        for (i = 0; i < w; i++)
            if (!round || !((i == 0 || i == w - 1) && (j == 0 || j == h - 1))) sg_pset(x + i, y + j, c);
}
static void sg_ell(int32_t cx, int32_t cy, int32_t rx, int32_t ry, uint32_t segs, uint32_t rot, int dots, uint16_t c)
{
    uint32_t i, st = 4294967295u / segs;
    for (i = 0; i < segs; i++) {
        uint32_t a = rot + i * st, b = a + st;
        if (!dots || !(i & 1u)) sg_line(cx + sx(a, rx), cy + sy(a, ry), cx + sx(b, rx), cy + sy(b, ry), c);
    }
}
static int32_t water(int32_t x, uint32_t line, uint32_t t)   /* STILLWATER: a line's slow swell at x */
{
    return sine((uint32_t)x * 0x03800000u + line * 0x40000000u + per(t, 7000u) * (line == 1u ? 1u : 2u)) * 4 >> 15;
}

static void sg_points(uint32_t sp, uint32_t t, int32_t *px, int32_t *py)   /* where each of the spell's scales sits now */
{
    const uint32_t n = SPELL[sp].n, Q = 0x40000000u;
    uint32_t i, a;
    for (i = 0; i < n; i++)
        switch (sp) {
        case 0: a = i * (4294967295u / 12u) - Q; px[i] = SG_CX + sx(a, 54); py[i] = SG_CY + sy(a, 54); break;
        case 1: px[i] = 20 + (int32_t)i * 24; py[i] = SG_CY + water(px[i], 1u, t); break;
        case 2: a = 0x0E38E38Eu + i * (0x638E38E3u / (n - 1u)); px[i] = SG_CX + sx(a, 56); py[i] = 48 + sy(a, 56); break;
        case 3: a = i * Q + Q / 2u; px[i] = SG_CX + sx(a, 52); py[i] = SG_CY + sy(a, 52); break;
        case 4:                                 /* the caps bob, one after another */
            a = i * (4294967295u / 6u) - Q;
            px[i] = SG_CX + sx(a, 48); py[i] = SG_CY + 2 + sy(a, 48) + (sine(per(t, 2600u) + i * 0x2AAAAAAAu) * 3 >> 16);
            break;
        default: px[i] = SG_CX; py[i] = 112 - (int32_t)i * 26; break;
        }
}
static void slot(uint32_t i, uint32_t n, int32_t *x, int32_t *y)   /* a seed resting in the soil */
{
    *x = SG_CX + ((int32_t)(2u * i) - (int32_t)(n - 1u)) * 6;
    *y = SG_H - 5;
}
/* the figure, and the light on the scale now: lit 0..256 of the way from scale `from` to `cur`; hi its brightness */
static void sg_figure(uint32_t sp, uint32_t t, uint32_t cur, uint32_t from, int32_t lit, int32_t hi, int marks)
{
    const uint32_t n = SPELL[sp].n, Q = 0x40000000u;
    int32_t px[12], py[12], i, mx, my, fx, fy;
    uint16_t faint = lite(64), dim = lite(96);
    sg_points(sp, t, px, py);
    mx = px[from] + (px[cur] - px[from]) * lit / 256;   /* the light, on its way */
    my = py[from] + (py[cur] - py[from]) * lit / 256;
    switch (sp) {
    case 0: {                                   /* WHEEL: a glint runs round the rim; the hand turns to the key */
        uint32_t g = per(t, 26000u), seg = 4294967295u / 72u, a0, a1, ah;
        int32_t d;
        for (i = 0; i < 72; i++) {
            uint32_t a = (uint32_t)i * seg, off = a - g;
            int32_t near = (int32_t)(off < 0x80000000u ? off : 0u - off) >> 24;   /* how far from the glint, 0..128 */
            sg_line(SG_CX + sx(a, 54), SG_CY + sy(a, 54), SG_CX + sx(a + seg, 54), SG_CY + sy(a + seg, 54),
                    lite(near < 12 ? 64 + (12 - near) * 8 : 64));
        }
        sg_ell(SG_CX, SG_CY, 5, 5, 16u, 0u, 0, faint);
        for (i = 0; i < 12; i++) {
            uint32_t a = (uint32_t)i * (4294967295u / 12u) - Q;
            sg_line(SG_CX + sx(a, 45), SG_CY + sy(a, 45), SG_CX + sx(a, 51), SG_CY + sy(a, 51), faint);
        }
        a0 = from * (4294967295u / 12u); a1 = cur * (4294967295u / 12u);   /* the hand: the short way round */
        d = (int32_t)(a1 - a0);
        ah = a0 + (uint32_t)(d >> 8) * (uint32_t)lit - Q + (uint32_t)(sine(per(t, 5300u)) * 512);   /* (and it sways a breath) */
        mx = SG_CX + sx(ah, 54); my = SG_CY + sy(ah, 54);
        fx = SG_CX + sx(ah, 7); fy = SG_CY + sy(ah, 7);
        sg_line(fx, fy, SG_CX + sx(ah, 44), SG_CY + sy(ah, 44), lite(hi));
        sg_line(fx + 1, fy, SG_CX + sx(ah, 44) + 1, SG_CY + sy(ah, 44), lite(hi));
        break;
    }
    case 1: {                                   /* STILLWATER: three lines swell; rings spread from the scale now */
        uint32_t ln, r = t % 3600u;
        for (ln = 0; ln < 3u; ln++) {
            int32_t x, y0 = SG_CY - 26 + (int32_t)ln * 26, prev = y0 + water(6, ln, t);
            for (x = 7; x < 154; x++) {
                int32_t y = y0 + water(x, ln, t);
                sg_line(x - 1, prev, x, y, ln == 1u ? lite(130) : faint);
                prev = y;
            }
        }
        if (r > 200u && marks) {
            int32_t rr = (int32_t)(r * 30u / 3600u);
            sg_ell(mx, my, rr, rr / 3 + 1, 24u, 0u, 0, lite(150 - rr * 4));
        }
        break;
    }
    case 2: {                                   /* HOLLOW: a bowl under a dark moon; its shadow drifts, the bowl's water rocks */
        int32_t dr = sine(per(t, 15000u)) * 6 >> 15, tl = sine(per(t, 6100u)) * 3 >> 15;
        sg_ell(SG_CX, 24, 16, 16, 32u, 0u, 0, lite(62));
        sg_ell(SG_CX + 6 + dr, 21, 14, 14, 32u, 0u, 1, lite(38));
        for (i = 0; i < 28; i++) {
            uint32_t a = 0x0E38E38Eu + (uint32_t)i * (0x638E38E3u / 28u), b = a + 0x638E38E3u / 28u;
            sg_line(SG_CX + sx(a, 56), 48 + sy(a, 56), SG_CX + sx(b, 56), 48 + sy(b, 56), faint);
        }
        sg_line(SG_CX - 38, 86 + tl, SG_CX + 38, 86 - tl, lite(40));
        break;
    }
    case 3: {                                   /* MEADOW: a sun; its rays turn slowly and breathe */
        uint32_t rot = per(t, 90000u);
        sg_ell(SG_CX, SG_CY, 14, 14, 32u, 0u, 0, lite(150));
        sg_ell(SG_CX, SG_CY, 13, 13, 32u, 0u, 0, lite(150));
        for (i = 0; i < 8; i++) {
            uint32_t a = rot + (uint32_t)i * (Q / 2u);
            int32_t len = (i & 1 ? 30 : 40) + (sine(per(t, 4700u) + (uint32_t)i * 0x20000000u) * 3 >> 15);
            sg_line(SG_CX + sx(a, 20), SG_CY + sy(a, 20), SG_CX + sx(a, len), SG_CY + sy(a, len), faint);
        }
        break;
    }
    case 4: {                                   /* FAE RING: the ring's dots wander round; spores rise and fade */
        sg_ell(SG_CX, SG_CY + 2, 48, 48, 64u, per(t, 60000u), 1, faint);
        for (i = 0; i < (int32_t)n; i++) {
            uint32_t ph = (t + (uint32_t)i * 1913u) % 5200u;
            int32_t amt = i == (int32_t)cur ? lit : i == (int32_t)from ? 256 - lit : 0;
            uint16_t c = lite(110 + (hi - 110) * amt / 256);
            sg_line(px[i], py[i] + 2, px[i], py[i] + 9, faint);
            sg_rect(px[i] - 5, py[i] - 1, 11, 3, c, 1);
            sg_rect(px[i] - 3, py[i] - 3, 7, 2, c, 0);
            if (ph < 3000u)
                sg_pset(px[i] + (sine(ph * 1431655u) * 3 >> 15), py[i] - 6 - (int32_t)(ph * 22u / 3000u), lite(130 - (int32_t)(ph * 120u / 3000u)));
        }
        return;
    }
    default: {                                  /* OLD SONG: a cairn; a slow light climbs it */
        int32_t climb = (int32_t)(t % 9000u) * ((int32_t)n + 2) * 256 / 9000 - 256;
        for (i = 0; i < (int32_t)n; i++) {
            int32_t w = 92 - i * 18, amt = i == (int32_t)cur ? lit : i == (int32_t)from ? 256 - lit : 0;
            int32_t d = climb - i * 256, bump = d < 0 ? -d : d, base = 46 + (bump < 256 ? (256 - bump) * 24 / 256 : 0);
            sg_rect(SG_CX - w / 2, py[i] - 9, w, 18, lite(base + (hi * 3 / 4 - base) * amt / 256), 1);   /* (lit: softer, it is big)*/
        }
        return;
    }
    }
    if (!marks) return;
    for (i = 0; i < (int32_t)n; i++) sg_rect(px[i] - 1, py[i] - 1, 3, 3, dim, 0);   /* the scales: seeds; the one now lit */
    sg_rect(mx - 3, my - 3, 7, 7, lite(hi), 1);
}

static struct { uint32_t sp, cur, osp, ocur, tsp, tcur; } sg = {99u, 0, 0, 0, 0, 0};
static void draw_home_sigil(uint32_t now, int glow)
{
    int32_t hi = glow ? 250 : 178 + (sine(per(now, 4300u)) * 16 >> 15), lit = 256, px[12], py[12], x, y, half;
    uint32_t ds, i, n;
    if (sg.sp == 99u) { sg.sp = spell_now; sg.cur = sg.ocur = scale_now; sg.tsp = now - 10000u; sg.tcur = now - 10000u; }
    if (spell_now != sg.sp) { sg.osp = sg.sp; sg.ocur = sg.cur; sg.sp = spell_now; sg.cur = scale_now; sg.tsp = now; sg.tcur = now - 10000u; }
    else if (scale_now != sg.cur) { sg.ocur = sg.cur; sg.cur = scale_now; sg.tcur = now; }
    for (half = 0; half < 2; half++) {          /* (the canvas holds 64 rows: the top half, then the bottom) */
    cv_begin(SG_W, SG_H / 2, C_BLACK);
    cv_oy = -half * (SG_H / 2);
    ds = now - sg.tsp;
    if (ds < T_OUT) {                           /* the old figure crumbles; its seeds fall to the soil */
        n = SPELL[sg.osp].n;
        sg_mode = 1u; sg_amt = ds * 256u / T_OUT;
        sg_figure(sg.osp, now, sg.ocur, sg.ocur, 256, hi, 0);
        sg_mode = 0u;
        sg_points(sg.osp, now, px, py);
        for (i = 0; i < n; i++) {
            int32_t k = ease((int32_t)(ds * 256u / T_OUT)), kk = k * k / 256;   /* (falling: slow, then faster) */
            slot(i, n, &x, &y);
            x = px[i] + (x - px[i]) * k / 256; y = py[i] + (y - py[i]) * kk / 256;
            sg_rect(x - 1, y - 1, 3, 3, lite(i == sg.ocur ? hi : 120), 0);
        }
    } else if (ds < T_OUT + T_REST) {           /* seeds at rest */
        n = SPELL[sg.osp].n;
        for (i = 0; i < n; i++) { slot(i, n, &x, &y); sg_rect(x - 1, y - 1, 3, 3, lite(120), 0); }
    } else if (ds < T_OUT + T_REST + T_IN) {    /* the new one grows up from the soil; its seeds rise into place */
        uint32_t u = ds - T_OUT - T_REST;
        n = SPELL[sg.sp].n;
        sg_mode = 2u; sg_amt = u * 320u / T_IN;
        sg_figure(sg.sp, now, sg.cur, sg.cur, 256, hi, 0);
        sg_mode = 0u;
        sg_points(sg.sp, now, px, py);
        for (i = 0; i < n; i++) {
            int32_t k = ease((int32_t)(u * 256u / (T_IN * 3u / 4u)) - (int32_t)(i * 64u / n));
            slot(i, n, &x, &y);
            x += (px[i] - x) * k / 256; y += (py[i] - y) * k / 256;
            sg_rect(x - 1, y - 1, 3, 3, lite(i == sg.cur ? 120 + (hi - 120) * k / 256 : 120), 0);
        }
    } else {
        if (now - sg.tcur < T_MOVE) lit = ease((int32_t)((now - sg.tcur) * 256u / T_MOVE));
        sg_figure(sg.sp, now, sg.cur, lit < 256 ? sg.ocur : sg.cur, lit, hi, 1);
    }
    cv_blit(SG_X, SG_Y + (uint32_t)half * (SG_H / 2));
    }
    cv_oy = 0;
}

/* the mycelium: nodes in RANK order (the root, then out to either side), each on a thread from its parent */
static const int8_t MY_DX[DV] = {0, 40, -44, 78, -84, 108}, MY_DY[DV] = {0, -6, 5, 8, -7, -3};
static const uint8_t MY_PAR[DV] = {0, 0, 0, 1, 2, 3}, MY_DEPTH[DV] = {0, 1, 1, 2, 2, 3};
#define MY_SEG 10
static int32_t my_spr = 256, my_pit = 12 << 8, my_g[DV];   /* eased: the reach (/256), the pitch (x256), each node's growth */
static uint32_t my_t;
static void my_node(uint32_t r, uint32_t t, int32_t *x, int32_t *y)
{
    int32_t yc = 34 - ((my_pit - (12 << 8)) >> 7);   /* PITCH 0: just under the soil; +12 above it, -12 deep */
    *x = 120 + MY_DX[r] * my_spr / 256;
    *y = yc + MY_DY[r] * (my_spr + 256) / 512 + (r ? sine(per(t, 6000u + r * 700u)) * 1 >> 15 : 0);
}
static void my_thread_pt(uint32_t r, uint32_t j, uint32_t t, int32_t *x, int32_t *y)   /* point j of node r's thread */
{
    int32_t x0, y0, x1, y1;
    my_node(MY_PAR[r], t, &x0, &y0);
    my_node(r, t, &x1, &y1);
    *x = x0 + (x1 - x0) * (int32_t)j / MY_SEG;
    *y = y0 + (y1 - y0) * (int32_t)j / MY_SEG;
    if (j && j < MY_SEG) {                      /* a gentle bow, a slow wave along it, breathing */
        uint32_t h = hsh(r * 31u + 7u);
        *y += (sine(j * (0x80000000u / MY_SEG)) * ((int32_t)(h % 11u) - 5) >> 15)
            + (sine(j * 0x2C000000u + h + per(t, 4100u + r * 300u)) * 2 >> 15);
    }
}
static void draw_mycelium(uint32_t now, int grow_mode)
{
    uint32_t r, j, dt = now - my_t;
    int32_t x, y, px_, py_, spr_t = 40 + (int32_t)set.spr * 27, pit_t = (int32_t)set.pit << 8;
    uint16_t thr = lite(90), hair = lite(44), seed = grow_mode ? lite(70) : RGB(55, 55, 55);
    int fl = drone_on && pulse_ms && now - pulse_ms < 900u;
    my_t = now;
    if (dt > 200u) dt = 200u;
    my_spr += (spr_t - my_spr) * (int32_t)dt / 300 + (spr_t > my_spr) - (spr_t < my_spr);   /* SPREAD and PITCH glide */
    my_pit += (pit_t - my_pit) * (int32_t)dt / 300 + (pit_t > my_pit) - (pit_t < my_pit);
    for (r = 0; r < DV; r++) {                  /* each node grows in over ~2.5 s, withers over ~3.5 s */
        int32_t want = drone_on && r < density ? 256 : 0;
        if (my_g[r] < want) { my_g[r] += (int32_t)dt * 256 / 2500 + 1; if (my_g[r] > want) my_g[r] = want; }
        else if (my_g[r] > want) { my_g[r] -= (int32_t)dt * 256 / 3500 + 1; if (my_g[r] < want) my_g[r] = want; }
    }
    cv_begin(240, MY_H, C_BLACK);
    for (j = 0; j < 46u; j++) {                 /* the earth: a few still specks; the soil line */
        uint32_t h = hsh(j + 77u);
        cv_pset((int32_t)(h % 240u), MY_SOIL + 4 + (int32_t)((h >> 8) % (uint32_t)(MY_H - MY_SOIL - 5)), lite(26));
    }
    for (x = 1; x < 240; x += 3) cv_pset(x, MY_SOIL, lite(40));
    if (my_g[0]) {                              /* the root's rootlets */
        my_node(0, now, &x, &y);
        for (j = 0; j < 3u; j++) {
            int32_t len = 5 + (int32_t)((hsh(j + 5u) % 6u) * (uint32_t)my_g[0] / 256u);
            cv_line(x, y, x - 4 + (int32_t)j * 4, y + len, hair);
        }
    }
    for (r = 1; r < DV; r++) {                  /* the threads, as far as they have grown, and their hairs */
        uint32_t reach = (uint32_t)my_g[r] * MY_SEG;
        if (!my_g[r]) continue;
        my_thread_pt(r, 0, now, &px_, &py_);
        for (j = 1; j <= MY_SEG && (j - 1u) * 256u < reach; j++) {
            my_thread_pt(r, j, now, &x, &y);
            if (j * 256u > reach) { int32_t f = (int32_t)(reach - (j - 1u) * 256u); x = px_ + (x - px_) * f / 256; y = py_ + (y - py_) * f / 256; }
            cv_line(px_, py_, x, y, fl ? lite(120) : thr);
            if (j < MY_SEG && j * 256u <= reach && (hsh(r * 97u + j) & 3u) < 2u) {   /* hairs, some forked */
                uint32_t h = hsh(r * 53u + j);
                int32_t hx = x + (int32_t)(h % 9u) - 4, hy = y + ((h >> 4) & 1u ? -4 : 5) + (int32_t)((h >> 5) % 3u);
                cv_line(x, y, hx, hy, hair);
                if ((h >> 8) & 1u) cv_line(hx, hy, hx + ((h >> 9) & 1u ? 3 : -3), hy + (hy < y ? -2 : 3), hair);
            }
            px_ = x; py_ = y;
        }
    }
    if (fl) {                                   /* a pulse: a bead runs out from the root, thread by thread */
        int32_t u = (int32_t)(now - pulse_ms) * 256 / 300;
        for (r = 1; r < DV; r++) {
            int32_t k = u - (int32_t)(MY_DEPTH[r] - 1u) * 256;
            if (k < 0 || k >= 256 || my_g[r] < 256) continue;
            my_thread_pt(r, (uint32_t)k * MY_SEG / 256u, now, &x, &y);
            cv_rect(x - 1, y - 1, 3, 3, lite(235));
        }
    }
    for (r = 0; r < DV; r++) {                  /* the nodes: a seed where one may grow; a grown one shines as its voice sounds */
        my_node(r, now, &x, &y);
        if (my_g[r] < 96) { cv_rect(x - 1, y - 1, 2, 2, seed); continue; }
        {
            int32_t a = drone[RANK[r]].vamp, l = 100 + (a > 0 ? a : 0) * 155 / 32767, s = my_g[r] < 256 ? 1 : 2;   /* (RANK is its own inverse) */
            cv_rect(x - s, y - s - 1, 2 * s + 1, 2 * s + 3, lite(l));
            cv_rect(x - s - 1, y - s, 2 * s + 3, 2 * s + 1, lite(l));
        }
    }
    cv_blit(0, MY_Y);
}

/* words for a moment */
static char say_s[34];
static uint32_t say_t, say_v;
static void say(const char *s)
{
    uint32_t i = 0;
    while (s[i] && i < sizeof say_s - 1u) { say_s[i] = s[i]; i++; }
    say_s[i] = 0;
    say_t = fm1_ms; say_v++;
}
static void say_field(uint32_t f)
{
    char s[34], *q = s;
    if (f == F_ENG) put(&q, eng_sel ? "BLOOM" : set.umd ? "EVENTS" : "DRONE");
    else { put(&q, F_NAME[f]); *q++ = ' '; fmt_field(&q, f, 0); }
    *q = 0;
    say(s);
}
static void say_scale(void)
{
    char s[24], *p = s;
    put(&p, PC_NAME[scale_pc[0]]); *p++ = ' '; put(&p, MODE_NAME[SPELL[spell_now].mode[scale_now]]);
    *p = 0;
    say(s);
}

/* turning things */
static int field_turn(uint32_t f, int32_t e)    /* a setting turned e detents; 1: the spell, tonic or scale changed */
{
    int32_t d = (int32_t)CFG[f] + e;
    CFG[f] = (uint8_t)(d < (int32_t)CFG_MIN[f] ? (int32_t)CFG_MIN[f] : d > (int32_t)cfg_max(f) ? (int32_t)cfg_max(f) : d);
    if (f == F_STP || f == F_STP1) {            /* fewer steps: hits and rotation stay inside */
        uint32_t i = f - F_STP;
        if (set.hit[i] > set.stp[i]) set.hit[i] = set.stp[i];
        if (set.rot[i] >= set.stp[i]) set.rot[i] = (uint8_t)(set.stp[i] - 1u);
    }
    if (f == F_SPL) set.scl = 0;                /* a new spell: from its first scale */
    if (f == F_SPL || f == F_TNC || f == F_SCL) cast(set.spl, set.tnc, set.scl);
    apply();
    if (f == F_DEN && set.den == DEN_GROW) { grow_dir = density >= DV ? -1 : 1; grow_wait(); }   /* GROW: on from here */
    else if (f == F_DEN) set_density(set.den);
    if (f == F_SPR || f == F_PIT) revoice();
    return f == F_SPL || f == F_TNC || f == F_SCL;
}
static void engine_turn(int32_t e, uint32_t um_held, uint32_t *um_ret)   /* HOME KNOB 4: DRONE, EVENTS, BLOOM */
{
    int32_t ei = (int32_t)((um_held ? *um_ret : eng_sel) ? 2u : set.umd) + e;
    ei = ei < 0 ? 0 : ei > 2 ? 2 : ei;
    if (ei < 2) set.umd = (uint8_t)ei;
    if (um_held) *um_ret = ei == 2;             /* (the UMWELT key held: where it goes back to) */
    else eng_sel = ei == 2;
    apply();
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
static void app_idle(void)                      /* while the screen draws: MIDI in, and the cycle keeps its time */
{
    if (!idle_ok) return;
    midi_in();
    cycle_tick(set.bpm, set.rat[eng_sel], set.stp[eng_sel], set.hit[eng_sel], set.rot[eng_sel], set.chc);
}

static void app_main(void)
{
    uint32_t prev = 0, k, t_change = 0, dirty = 0, note_drawn = ~0u, scale_drawn = ~0u, info_drawn = ~0u, sigil_drawn = ~0u;
    uint32_t net_drawn = ~0u, knobs_drawn = ~0u, tabs_drawn = ~0u, chord_drawn = ~0u, label_drawn = ~0u, shown = NPAGE, sg_drawn_t = 0, my_drawn_t = 0, home_turn = 0, turn_ms = 0, kl, fl, snd, page = P_HOME, cpu_pct = 0, cpu_drawn = 0;
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
        for (k = 1; ok && k < F_N; k++) ok = CFG[k] >= CFG_MIN[k] && CFG[k] <= CFG_MAX[k];
        if (ok) ok = set.scl < SPELL[set.spl].n;
        if (!ok) for (k = 0; k < F_N; k++) CFG[k] = CFG_DEF[k];
    }
    eng_sel = 0;                                /* every power-on: UMWELT */
    spell_now = set.spl; tonic = set.tnc; scale_now = set.scl;
    scale_load();
    apply();
    tim_prev = tim_now;
    density = set.den == DEN_GROW ? 1u : set.den;   /* GROW: from a seedling */
    lcd_fill(0, 0, 240, 240, C_BLACK);
    draw_text_box(0, 4, 240, &FONT_S, APP_NAME, C_ACC, 1);

    for (;;) {
        uint32_t keys, down, up, rel, b, lit = 0, out_ch = set.ch ? set.ch - 1u : 0u, st, e8 = eng_sel, alt;
        int32_t oct = (int8_t)set.oct, e;
        uint32_t base = (uint32_t)((int32_t)KEY0 + 12 * oct);
        platform_poll();

        keys = fm1_in.notes;                    /* bit k: key k (0 = F3 .. 26 = G5) */
        down = keys & ~prev;
        up = prev & ~keys;
        prev = keys;
        if (down >> PK_UMWELT & 1u) { um_back = eng_sel; um_hold = 1; eng_sel = 0; apply(); }   /* held: UMWELT */
        if (up >> PK_UMWELT & 1u && um_hold) { um_hold = 0; eng_sel = um_back; apply(); }        /* let go: back */
        for (k = 0; k < KEY_PLAY; k++) {            /* (a key stops the note it started: the octave may change meanwhile) */
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
        if (b & (1u << BTN_SEL)) { if (um_hold) um_back ^= 1u; else { eng_sel ^= 1u; apply(); } }   /* SEL: UMWELT <-> BLOOM (the UMWELT key held: where it goes back to) */
        if (b & (1u << BTN_HOME)) page = P_HOME;                          /* HOME: back home */
        if (b & (1u << BTN_EDIT)) page = page == P_ED1 ? P_ED2 : P_ED1;   /* EDIT: the engine's pages */
        if (b & (1u << BTN_LFO)) page = page == P_LFO ? P_HOME : P_LFO;   /* LFO: tempo, pulse, season (again: home) */
        if (b & (1u << BTN_FX)) page = page == P_FX1 ? P_FX2 : P_FX1;     /* FX: round the effect pages */
        if (b & (1u << BTN_GLO)) page = page == P_BOOK ? P_HOME : P_BOOK; /* GLO: the BOOK of spells (again: home) */
        alt = fm1_in.notes >> PK_ALT & 1u;      /* F#5 held: the second layer of SELECT, PRESETS and ALGORITHM */
        if ((e = fm1_enc_take(ENC_SELECT)) != 0) {     /* SELECT: the spell, round the BOOK (F#5: the tempo) */
            if (alt) { field_turn(F_BPM, e); say_field(F_BPM); }
            else {
                set.spl = (uint8_t)(((int32_t)set.spl + e % (int32_t)NSPELL + (int32_t)NSPELL) % (int32_t)NSPELL);
                set.scl = 0;
                cast(set.spl, set.tnc, 0);
                apply();
                say(SPELL[spell_now].name);
                turn_ms = fm1_ms;
            }
            dirty = 1; t_change = fm1_ms;
        }
        if ((e = fm1_enc_take(ENC_PRESETS)) != 0) {    /* PRESETS: the spell's next or last scale, pivoting (F#5: CHORDS) */
            if (alt) { field_turn(F_CHG, e); say_field(F_CHG); }
            else {
                uint32_t n = SPELL[spell_now].n;
                cast(spell_now, tonic, (scale_now + (uint32_t)((e % (int32_t)n) + (int32_t)n)) % n);
                set.scl = (uint8_t)scale_now;
                say_scale();
            }
            dirty = 1; t_change = fm1_ms;
        }
        for (k = 0; k < 4u; k++)                /* KNOB 1..4: the page's four settings (HOME: KNOB 4 the engine) */
            if ((e = fm1_enc_take(ENC_KNOB1 + k)) != 0) {
                uint32_t f = page_field(page, k);
                if (page == P_HOME && k == 3u) { engine_turn(e, um_hold, &um_back); say_field(F_ENG); }
                else if (f >= F_N) continue;    /* (a display, not a setting) */
                else {
                    if (field_turn(f, e)) turn_ms = fm1_ms;
                    if (page == P_HOME) say_field(f);
                }
                dirty = 1; t_change = fm1_ms;
            }
        if ((e = fm1_enc_take(ENC_ALGORITHM)) != 0) {   /* ALGORITHM: the engine's timbre, round the ring (F#5: DENSITY) */
            if (alt) { field_turn(F_DEN, e); say_field(F_DEN); }
            else {
                set.tim[eng_sel] = (uint8_t)(((int32_t)set.tim[eng_sel] + e % (int32_t)NT + (int32_t)NT) % (int32_t)NT);
                apply();
                say(TIM_NAME[set.tim[eng_sel]]);
            }
            dirty = 1; t_change = fm1_ms;
        }

        if (b & (1u << BTN_PLAY)) {             /* PLAY: grow, or let it fade back into the ground */
            if (drone_on) grow_stop();
            else {
                if (set.den == DEN_GROW) { density = 1u; grow_dir = 1; }   /* GROW: a seedling, the root alone */
                grow_start();
                grow_next = fm1_ms + 8000u;     /* (the seedling's first leaf comes soon) */
            }
        }
        if (lfo_tick(set.bpm, set.pul, set.shp, set.sea, set.roo)) {   /* the season or ROOTS turned */
            set.scl = (uint8_t)scale_now;
            set.tnc = (uint8_t)tonic;
            dirty = 1; t_change = fm1_ms; turn_ms = fm1_ms;
            say_scale();                        /* (HOME: its name, a moment) */
        }
        grow_tick(set.den == DEN_GROW);
        cycle_tick(set.bpm, set.rat[eng_sel], set.stp[eng_sel], set.hit[eng_sel], set.rot[eng_sel], set.chc);

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
        lit &= (1u << KEY_PLAY) - 1u;           /* the performance keys: lit while they act */
        lit |= (uint32_t)um_hold << PK_UMWELT | (fm1_in.notes >> PK_ALT & 1u) << PK_ALT;
        for (k = 0; k < 27u; k++)
            fm1_led_key(KEY_LED(k), lit >> k & 1u);
        fm1_led_key(BTN_SEL, !eng_sel);         /* lit: UMWELT */
        fm1_led_key(BTN_HOME, page == P_HOME);  /* the page's button lit */
        fm1_led_key(BTN_EDIT, page == P_ED1 || page == P_ED2);
        fm1_led_key(BTN_LFO, page == P_LFO);
        fm1_led_key(BTN_FX, page == P_FX1 || page == P_FX2);
        fm1_led_key(BTN_GLO, page == P_BOOK);
        fm1_led_key(BTN_OCT_DOWN, oct < 0);
        fm1_led_key(BTN_OCT_UP, oct > 0);
        fm1_led_key(BTN_PLAY, drone_on ? !hit_ms || fm1_ms - hit_ms > 60u || (!eng_sel && !set.umd) : clk_run);   /* lit: growing; a blink a hit */
#if BOARD_SMK37
        for (k = 0; k < 16u; k++) {             /* a pad glows while its key sounds */
            uint32_t w = k < 8u ? k + 8u : k - 8u, on = lit >> WHITE[w] & 1u;
            padled_set(k, on ? 60 : 4, on ? 110 : 6, on ? 40 : 2);
        }
        padled_flush();                         /* (sends only what changed) */
#endif

        idle_ok = 1;                            /* the drawing below may read MIDI meanwhile (LCD_IDLE) */
#ifndef DINGHY_HOST
        if (fm1_ms - cpu_t >= 1000u) {          /* the audio ISR's worst run of the last second, of its 5.8 ms */
            cpu_t = fm1_ms;
            cpu_pct = cpu_max_us * 100u / 5805u;
            cpu_max_us = 0;
        }
#endif
        kl = turn_ms && fm1_ms - turn_ms < 3000u;   /* the season just turned: the sigil and the scale glow */
        if ((page == P_HOME) != (shown == P_HOME)) {   /* HOME and the other pages: each its own screen */
            lcd_fill(0, 0, 240, 240, C_BLACK);
            sigil_drawn = scale_drawn = note_drawn = net_drawn = info_drawn = knobs_drawn = tabs_drawn = cpu_drawn = ~0u;
            chord_drawn = label_drawn = ~0u;
            if (page != P_HOME) draw_text_box(0, 4, 240, &FONT_S, APP_NAME, C_ACC, 1);
            sg.sp = 99u;                        /* (back home: the sigil as it is, no growing) */
        }
        shown = page;
        if (page == P_HOME) {                   /* HOME: the sigil and the ground take turns, each ~14 frames a second at most */
            uint32_t now = fm1_ms, lv = 0;
            const char *ls = "";
            char hint[34];
            if (now - sg_drawn_t >= 70u && (!home_turn || now - my_drawn_t < 70u)) { draw_home_sigil(now, (int)kl); sg_drawn_t = now; home_turn = 1; }
            else if (now - my_drawn_t >= 70u) { draw_mycelium(now, set.den == DEN_GROW); my_drawn_t = now; home_turn = 0; }
            if ((st = drone_on ? 1u + chord_now + scale_now * 8u + spell_now * 128u + tonic * 1024u + kl * 16384u : 0u) != chord_drawn) {
                char c[16], *q = c;             /* the chord, small, at the top */
                chord_drawn = st;
                if (st) { put(&q, PC_NAME[cpc(chord_now, 0)]); put(&q, chord_qual(chord_now)); }
                *q = 0;
                draw_text_box(0, 2, 240, &FONT_S, c, lite(kl ? 235 : 150), 1);
            }
            if (say_v && now - say_t < 1800u) { ls = say_s; lv = say_v; }   /* what was just turned, a moment */
            else if (alt) {                     /* F#5 held: what the second layer holds */
                char *q = hint;
                put(&q, "BPM "); put_num(&q, set.bpm); put(&q, "  CHORDS "); fmt_field(&q, F_CHG, 0);
                put(&q, "  DENS "); fmt_field(&q, F_DEN, 0);
                *q = 0;
                ls = hint; lv = 0x80000000u | set.bpm | (uint32_t)set.chg << 8 | (uint32_t)set.den << 12;
            }
            if (lv != label_drawn) { label_drawn = lv; draw_text_box(0, 222, 240, &FONT_S, ls, lite(150), 1); }
        } else {
            if ((st = spell_now | scale_now << 4 | kl << 8) != sigil_drawn) { sigil_drawn = st; draw_sigil(spell_now, scale_now, (int)kl); }
            if ((st = spell_now | scale_now << 4 | tonic << 8 | kl << 12) != scale_drawn) { scale_drawn = st; draw_scale((int)kl); }
            if ((st = drone_on ? 1000u + chord_now + scale_now * 8u + spell_now * 128u + tonic * 1024u : 0u) != note_drawn) { note_drawn = st; draw_note(); }
            fl = drone_on && pulse_ms && fm1_ms - pulse_ms < 300u;
            snd = 0;                                /* EVENTS, BLOOM: the nodes of the voices sounding now */
            if (engine) for (k = 0; k < DV; k++) if (drone[k].vamp > 6000) snd |= 1u << RANK[k];
            if ((st = density | (uint32_t)drone_on << 4 | fl << 5 | snd << 6 | (uint32_t)(set.den == DEN_GROW) << 12) != net_drawn) { net_drawn = st; draw_network(density, (uint32_t)drone_on, (int)fl, snd, set.den == DEN_GROW); }
            if ((st = spell_now | e8 << 4 | (uint32_t)set.umd << 5 | (uint32_t)set.tim[e8] << 6) != info_drawn) { info_drawn = st; draw_info(); }
            st = page | cpu_pct << 3 | e8 << 12;    /* the panel: redrawn when the page, a value on it or the CPU changes */
            for (k = 0; k < 4u; k++) { uint32_t f = page_field(page, k); st = st * 31u + (f < F_N ? CFG[f] : 0u) + f * 7u; }
            if (st != knobs_drawn) { knobs_drawn = st; draw_knobs(page, cpu_pct); }
            if (page != tabs_drawn) { tabs_drawn = page; draw_tabs(page); }
            if ((st = page == P_BOOK ? 1000u + cpu_pct : 0u) != cpu_drawn) {   /* the BOOK: the CPU load, top right */
                char c[12], *q = c;
                cpu_drawn = st;
                if (st) { put(&q, "CPU "); fmt_field(&q, F_CPU, cpu_pct); }
                *q = 0;
                draw_text_box(184, 4, 56, &FONT_S, c, C_OFF, 2);
            }
        }
        idle_ok = 0;

        if (dirty && quiet() && fm1_ms - t_change > 2000u) {   /* 2 s after the change, when silent (an erase stalls the sound) */
            settings_save(&set, sizeof set);
            dirty = 0;
        }
    }
}
