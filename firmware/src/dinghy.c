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
#define DV 4u
static struct {
    volatile uint32_t tinc;                     /* target phase step (main loop writes) */
    uint32_t inc;                               /* the step now: glides to tinc */
    uint32_t pa, pb, lfo;                       /* the two sines' phases, the breath's phase */
    volatile uint32_t lfo_inc;                  /* breath speed, per block */
    volatile uint8_t note;
} drone[DV];
#define NT 4u                                   /* timbres: MOSS, LOAM, MIST, ROOT */
static volatile uint32_t tim_now;               /* the timbre (main loop writes) */
static uint32_t tim_prev, tim_xf = 32767;       /* the crossfade from the last one, Q15 (audio) */
static volatile int32_t drone_on;               /* 1: growing / sounding, 0: fading to silence */
static int32_t drone_lvl;                       /* the drone's level now, Q15: fades in and out */
#define DRONE_GAIN 5200                         /* per voice, Q15: four voices with partials stay under full scale */

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

static void drone_audio(int32_t *out, uint32_t n)
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
        int32_t breath, g;
        inc += (uint32_t)((int32_t)(tinc - inc) >> 7);   /* glide: ~1.5 s to settle */
        if ((int32_t)(tinc - inc) > -128 && (int32_t)(tinc - inc) < 128) inc = tinc;
        drone[v].inc = inc;
        incb = inc + (inc >> 10);               /* +1.7 cents: a slow beat between the two */
        drone[v].lfo += drone[v].lfo_inc;
        breath = 24576 + ((sine(drone[v].lfo) * 8192) >> 15);   /* 0.5 .. 1.0 */
        g = (((DRONE_GAIN * breath) >> 15) * lvl) >> 15;
        for (i = 0; i < n; i++) {
            int32_t s;
            pa += inc;
            pb += incb;
            s = shape(tim_now, pa, pb, breath);
            if (tim_xf < 32767)
                s = (s * (int32_t)(tim_xf >> 1) + shape(tim_prev, pa, pb, breath) * (int32_t)((32767u - tim_xf) >> 1)) >> 14;   /* (signed; no overflow) */
            s = (s * g) >> 15;
            out[2u * i] += s;
            out[2u * i + 1u] += s;
        }
        drone[v].pa = pa;
        drone[v].pb = pb;
    }
}

static void app_audio(int32_t *out, uint32_t n)
{
    uint32_t v, i;
    drone_audio(out, n);
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
static const uint8_t CHORD[6][4] = {            /* pitch classes, root first */
    {0, 4, 7, 11}, {9, 0, 4, 7}, {5, 9, 0, 4}, {2, 5, 9, 0}, {7, 11, 2, 5}, {4, 7, 11, 2}};
static const char *const CHORD_NAME[6] = {"Cmaj7", "Am7", "Fmaj7", "Dm7", "G7", "Em7"};
static const uint8_t RANGE_LO[DV] = {40, 52, 57, 62}, RANGE_HI[DV] = {52, 64, 69, 74};   /* E2..E3, E3..E4, ... */
static uint32_t grow_rng = 0x2545F491u, chord_now, moves_left, t_next_move, grow_ver;

static uint32_t rnd(uint32_t n)                 /* 0 .. n-1 (xorshift32) */
{
    grow_rng ^= grow_rng << 13; grow_rng ^= grow_rng >> 17; grow_rng ^= grow_rng << 5;
    return n ? grow_rng % n : 0u;
}
static uint32_t note_inc(uint32_t note) { return INC_C9[note % 12u] >> (10u - note / 12u); }
static int in_chord(uint32_t note, uint32_t c)
{
    uint32_t k;
    for (k = 0; k < 4u; k++) if (note % 12u == CHORD[c][k]) return 1;
    return 0;
}
static int held_by_other(uint32_t pc, uint32_t v)   /* another upper voice already has this pitch class */
{
    uint32_t w;
    for (w = 1; w < DV; w++) if (w != v && drone[w].note % 12u == pc) return 1;
    return 0;
}
static uint32_t pick_note(uint32_t v, uint32_t c, int fresh)   /* a chord note in v's range, near where v is */
{
    uint32_t n, best = drone[v].note, bestd = 0xFFFFu;
    for (n = RANGE_LO[v]; n <= RANGE_HI[v]; n++) {
        uint32_t d = n > drone[v].note ? n - drone[v].note : drone[v].note - n;
        if (v == 0u ? n % 12u != CHORD[c][0] : !in_chord(n, c) || held_by_other(n % 12u, v)) continue;
        if (fresh && n == drone[v].note) continue;   /* a drift: somewhere new */
        d = d * 4u + rnd(6u);                   /* nearest, with a little chance */
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
static void grow_start(void)                    /* PLAY: plant the network (silent, so no glide) */
{
    uint32_t v;
    grow_rng ^= fm1_ms * 2654435761u;
    if (!grow_rng) grow_rng = 1;
    chord_now = 0;
    for (v = 0; v < DV; v++) drone[v].note = (uint8_t)((RANGE_LO[v] + RANGE_HI[v]) / 2u);
    for (v = 0; v < DV; v++) {
        uint32_t n = pick_note(v, chord_now, 0);
        drone[v].note = (uint8_t)n;
        drone[v].tinc = drone[v].inc = note_inc(n);
        drone[v].lfo_inc = 4294967295u / (172u * (7u + rnd(13u)));   /* a breath every 7..19 s */
    }
    moves_left = 4u + rnd(3u);
    t_next_move = fm1_ms + 6000u;
    grow_ver++;
    drone_on = 1;
}
static void grow_tick(void)                     /* call often; acts every few seconds */
{
    uint32_t v, w;
    if (!drone_on || (int32_t)(fm1_ms - t_next_move) < 0) return;
    t_next_move = fm1_ms + 3500u + rnd(4500u);  /* the next move in 3.5..8 s */
    if (!moves_left) {                          /* the network shifts: a neighbour on the ring */
        chord_now = (chord_now + (rnd(3u) ? 1u : 5u)) % 6u;
        moves_left = 4u + rnd(3u);
    }
    moves_left--;
    for (w = 0, v = DV; w < DV; w++)            /* a voice outside the chord moves first, the bass before the rest */
        if (v == DV && (w == 0u ? drone[0].note % 12u != CHORD[chord_now][0] : !in_chord(drone[w].note, chord_now)))
            v = w;
    if (v == DV) v = 1u + rnd(DV - 1u);         /* all in the chord: one upper voice drifts to a new voicing */
    voice_to(v, pick_note(v, chord_now, v == DV ? 0 : 1));
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
    if (drone_on) { draw_text_box(0, 82, 240, &FONT_L, CHORD_NAME[chord_now], C_ACC, 1); return; }
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
static void draw_status(int32_t oct, uint32_t ch, uint32_t tim)   /* "OCT +0   MOSS   OMNI" */
{
    char s[28] = "OCT +0   ", *p = s + 9;
    const char *a = TIM_NAME[tim];
    s[4] = oct < 0 ? '-' : '+';
    s[5] = (char)('0' + (oct < 0 ? -oct : oct));
    while (*a) *p++ = *a++;
    *p++ = ' '; *p++ = ' '; *p++ = ' ';
    if (ch) { *p++ = 'C'; *p++ = 'H'; if (ch >= 10u) *p++ = '1'; *p++ = (char)('0' + ch % 10u); }
    else { a = "OMNI"; while (*a) *p++ = *a++; }
    *p = 0;
    lcd_fill(0, 208, 240, 24, C_BLACK);
    draw_text_box(0, 212, 240, &FONT_S, s, C_OFF, 1);
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
static struct { uint8_t oct, ch, tim; } set = {0, 0, 0};   /* what survives a power-off (add yours, up to SECTOR_MAX bytes) */
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
    uint32_t prev = 0, k, t_change = 0, dirty = 0, lit_drawn = ~0u, note_drawn = ~0u, status_drawn = ~0u;
    uint32_t midi_drawn = ~0u;
    uint8_t held[27] = {0};                     /* the note each key plays */
#if BOARD_SMK37
    static const uint8_t WHITE[16] = {0, 2, 4, 6, 7, 9, 11, 12, 14, 16, 18, 19, 21, 23, 24, 26};   /* white key -> key */
    uint8_t pad_note[16] = {0}, ext_note[FM1_NKEY] = {0};
    smk_pw_t wheel = {0};
    smk_pad_events = 1;                         /* the pads as events (smk_pad_take): they play here, lit while they sound */
#endif
    if (settings_load(&set, sizeof set) || (int8_t)set.oct < -2 || (int8_t)set.oct > 2 || set.ch > 16u || set.tim >= NT)
        set.oct = 0, set.ch = 0, set.tim = 0;

    tim_now = tim_prev = set.tim;
    lcd_fill(0, 0, 240, 240, C_BLACK);
    draw_text_box(0, 18, 240, &FONT_L, APP_NAME, C_ACC, 1);
    draw_text_box(0, 52, 240, &FONT_S, "the ground layer  v" APP_VER, C_OFF, 1);

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
            if (drone_on) drone_on = 0;
            else grow_start();
        }
        grow_tick();

        midi_in();
        if (clk_bpm && fm1_ms - clk_t0 > 2000u) clk_bpm = 0, clk_run = 0;   /* the clock has stopped coming */

        oct = (int8_t)set.oct;
        base = (uint32_t)((int32_t)KEY0 + 12 * oct);
        for (k = 0; k < NV; k++)                /* what sounds, on the keys of this octave */
            if (voice[k].gate && voice[k].note >= base && voice[k].note < base + 27u)
                lit |= 1u << (voice[k].note - base);
        if (drone_on)                           /* the network's notes light up too */
            for (k = 0; k < DV; k++)
                if (drone[k].note >= base && drone[k].note < base + 27u)
                    lit |= 1u << (drone[k].note - base);
        for (k = 0; k < 27u; k++)
            fm1_led_key(KEY_LED(k), lit >> k & 1u);
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
        if (lit != lit_drawn) { lit_drawn = lit; draw_keys(lit); }   /* redraw only what changed */
        if ((st = drone_on ? 1000u + chord_now : last_note) != note_drawn) { note_drawn = st; draw_note(); }
        if ((st = (uint32_t)(oct + 2) | (uint32_t)set.ch << 4 | (uint32_t)set.tim << 9) != status_drawn) { status_drawn = st; draw_status(oct, set.ch, set.tim); }
        if ((st = midi_src | clk_bpm << 2 | clk_run << 12) != midi_drawn) { midi_drawn = st; draw_midi(midi_src, clk_bpm, clk_run); }
        idle_ok = 0;

        if (dirty && quiet() && fm1_ms - t_change > 2000u) {   /* 2 s after the change, when silent (an erase stalls the sound) */
            settings_save(&set, sizeof set);
            dirty = 0;
        }
    }
}
