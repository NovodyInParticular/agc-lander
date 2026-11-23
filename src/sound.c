// Cabin sound synthesis. See sound.h.
//
// One stereo output; the audio thread generates every sample from a handful
// of targets written by the game thread (plain floats and event counters:
// a torn read costs nothing audible).
//
// The device is opened directly through miniaudio (the copy compiled into
// raylib) rather than through raylib's audio module: raylib gives no control
// over the device buffer (its default is a few milliseconds, with little
// margin when the machine hiccups), and it would resample a fixed-rate
// stream, where here the synthesis simply runs at whatever rate the device uses.

#include <math.h>
#include <stdio.h>

// Same switches raylib's raudio.c builds miniaudio with, so the types match.
#define MA_NO_JACK
#define MA_NO_WAV
#define MA_NO_FLAC
#define MA_NO_MP3
#define MA_NO_RESOURCE_MANAGER
#define MA_NO_NODE_GRAPH
#define MA_NO_ENGINE
#define MA_NO_GENERATION
#include "external/miniaudio.h"

#include "sound.h"

#define TAU 6.28318530718f
#define PERIOD_FRAMES 1024   // requested device period, about 21 ms

static ma_device device;
static int ready, muted;
static float sr = 48000.0f;   // device sample rate, set when the device opens

// Targets written by the game thread.
static volatile float t_gain, t_thrust, t_aps, t_rcs, t_alarm, t_fans = 1.0f;
static volatile unsigned ev_switch, ev_key, ev_rcs, ev_thud, ev_crash, ev_stage, ev_step;
static volatile float ev_thud_gain = 1.0f, ev_step_gain = 1.0f;

// --- Audio-thread state ----------------------------------------------------------

static unsigned rng_state = 0x1234567u;
static inline float white(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return (float)(rng_state & 0xffffff) / 8388608.0f - 1.0f;
}

// One-pole low-pass coefficient for a cutoff in Hz.
static inline float lp_coef(float hz) { return 1.0f - expf(-TAU * hz / sr); }

typedef struct { float env, decay, phase, freq, fdrop, noise_lp, lp_k, tone, noise; } Hit;

// Start a percussive hit: decaying sine (freq falling by fdrop per second) plus low-passed noise.
static void hit_start(Hit *h, float gain, float seconds, float freq, float fdrop, float noise_hz, float tone, float noise)
{
    h->env = gain;
    h->decay = expf(-1.0f / (seconds * sr));
    h->phase = 0;
    h->freq = freq;
    h->fdrop = fdrop / sr;
    h->lp_k = lp_coef(noise_hz);
    h->tone = tone;
    h->noise = noise;
}

static inline float hit_run(Hit *h)
{
    if (h->env < 1e-4f) return 0;
    h->phase += TAU * h->freq / sr;
    if (h->phase > TAU) h->phase -= TAU;
    if (h->freq > 20) h->freq -= h->fdrop;
    h->noise_lp += (white() - h->noise_lp) * h->lp_k;
    float v = (sinf(h->phase) * h->tone + h->noise_lp * h->noise) * h->env;
    h->env *= h->decay;
    return v;
}

#define NHIT 6
static Hit rcs_hit[NHIT], click_hit[4], thud_hit[2], crash_hit[2], pyro_hit[4], ign_hit[2], step_hit[2];
static int rcs_i, click_i, pyro_i, step_i;
static unsigned seen_switch, seen_key, seen_rcs, seen_thud, seen_crash, seen_stage, seen_step;
static int pyro_left, pyro_wait;   // staging: the remaining charges and samples to the next one

static float gain, thrust, aps, rcs, alarm, fans = 1.0f;
static float aps_bp1, aps_bp2, aps_hiss1, aps_hiss2, aps_flut, aps_ph, aps_jit, aps_was;
static float k_aps_hi, k_aps_lo, k_hiss_hi, k_hiss_lo, k_flut, g_aps;
static float eng_lp1, eng_lp2, eng_wob, eng_ph;
static float fan_bp1[2], fan_bp2[2], hum_ph[3], rcs_lp, alarm_ph, alarm_gate_ph;
static float k_eng1, k_eng2, k_wob, k_fan_lo, k_fan_hi, k_rcs;
static float g_gain, g_thrust, g_rcs, g_alarm;

// Glide v toward target t; lands exactly so a faded-out value never lingers as a denormal.
static inline float glide(float v, float t, float k)
{
    v += (t - v) * k;
    return fabsf(v - t) < 1e-6f ? t : v;
}

static void audio_cb(ma_device *dev, void *buffer, const void *input, ma_uint32 frames)
{
    (void)dev;
    (void)input;
    short *out = buffer;
    // Events since the last block.
    unsigned n;
    n = ev_switch;
    if (n != seen_switch) {
        seen_switch = n;
        Hit *h = &click_hit[click_i++ & 3];
        hit_start(h, 0.22f, 0.006f, 2600, 0, 7000, 0.6f, 1.6f);   // snap of a toggle
    }
    n = ev_key;
    if (n != seen_key) {
        seen_key = n;
        Hit *h = &click_hit[click_i++ & 3];
        hit_start(h, 0.16f, 0.012f, 900, 4000, 2500, 0.8f, 1.2f);  // soft key thock
    }
    n = ev_rcs;
    if (n != seen_rcs) {
        unsigned count = n - seen_rcs;
        seen_rcs = n;
        if (count > 3) count = 3;
        for (unsigned i = 0; i < count; i++) {
            Hit *h = &rcs_hit[rcs_i++ % NHIT];
            // A 100 lb jet lighting off: a dull knock on the hull.
            hit_start(h, 0.30f + 0.08f * white(), 0.050f, 105 + 25 * white(), 300, 900, 1.0f, 1.3f);
        }
    }
    n = ev_thud;
    if (n != seen_thud) {
        seen_thud = n;
        hit_start(&thud_hit[0], 0.75f * ev_thud_gain, 0.22f, 62, 60, 300, 1.0f, 1.5f);
        hit_start(&thud_hit[1], 0.25f * ev_thud_gain, 0.60f, 38, 10, 120, 1.0f, 0.6f);   // gear and structure settling
    }
    n = ev_crash;
    if (n != seen_crash) {
        seen_crash = n;
        hit_start(&crash_hit[0], 1.3f, 0.9f, 48, 25, 1400, 0.8f, 2.6f);
        hit_start(&crash_hit[1], 0.6f, 2.2f, 31, 4, 200, 1.0f, 1.2f);
    }
    n = ev_stage;
    if (n != seen_stage) {
        seen_stage = n;
        pyro_left = 4;     // four explosive nuts and the umbilical guillotine, a few ms apart
        pyro_wait = 0;
    }
    n = ev_step;
    if (n != seen_step) {
        seen_step = n;
        // A boot meeting the regolith, heard only through the suit: soft and low.
        hit_start(&step_hit[step_i++ & 1], 0.16f * ev_step_gain, 0.07f, 75 + 10 * white(), 200, 500, 0.9f, 1.0f);
    }

    for (unsigned i = 0; i < frames; i++) {
        // Glide to the targets (about 60 ms, the engine a little slower).
        gain = glide(gain, t_gain, g_gain);
        thrust = glide(thrust, t_thrust, g_thrust);
        rcs = glide(rcs, t_rcs, g_rcs);
        alarm = glide(alarm, t_alarm, g_alarm);
        fans = glide(fans, t_fans, g_thrust * 0.3f);   // fans take a second or two to spin up or down
        aps = glide(aps, t_aps, g_aps);

        float w0 = white(), w1 = white();

        // Staging charges: sharp cracks a few milliseconds apart, each ringing the structure.
        if (pyro_left > 0 && --pyro_wait <= 0) {
            Hit *h = &pyro_hit[pyro_i++ & 3];
            hit_start(h, 0.55f + 0.1f * white(), 0.030f, 1500 + 400 * white(), 9000, 6000, 0.5f, 2.2f);
            if (pyro_left == 4) hit_start(&crash_hit[1], 0.45f, 0.45f, 140, 60, 600, 1.0f, 0.8f);   // the whole cabin rings once
            pyro_left--;
            pyro_wait = (int)(sr * (0.012f + 0.025f * (white() * 0.5f + 0.5f)));
        }

        // Environmental control: two suit fans as broad bands of air noise, slightly
        // different left and right, over the 400 Hz inverter whine and its harmonics.
        float fan[2];
        for (int c = 0; c < 2; c++) {
            float wn = c ? w1 : w0;
            fan_bp1[c] += (wn - fan_bp1[c]) * k_fan_hi;
            fan_bp2[c] += (fan_bp1[c] - fan_bp2[c]) * k_fan_lo;
            fan[c] = (fan_bp1[c] - fan_bp2[c]) * 0.10f * fans;
        }
        static const float hum_f[3] = { 400.0f, 800.0f, 1200.0f }, hum_a[3] = { 0.0110f, 0.0042f, 0.0018f };
        float hum = 0;
        for (int k = 0; k < 3; k++) {
            hum_ph[k] += TAU * hum_f[k] / sr;
            if (hum_ph[k] > TAU) hum_ph[k] -= TAU;
            hum += sinf(hum_ph[k]) * hum_a[k];
        }

        // Descent engine: no air to carry a roar, only what the structure passes
        // on: a deep, slightly unsteady rumble that follows the throttle.
        eng_lp1 += (w0 - eng_lp1) * k_eng1;
        eng_lp2 += (w1 - eng_lp2) * k_eng2;
        eng_wob += (w0 - eng_wob) * k_wob;
        eng_ph += TAU * (41.0f + 9.0f * thrust) / sr;
        if (eng_ph > TAU) eng_ph -= TAU;
        float ta = powf(thrust, 0.7f);
        float eng = (eng_lp1 * 5.5f + eng_lp2 * 1.3f + sinf(eng_ph) * 0.10f) * ta * (0.34f + eng_wob * 6.0f * 0.34f);

        // Ascent engine: bolted to the structure right behind the cabin (its cover
        // sits between the crew's feet). Harder and higher than the descent engine:
        // a rough mid-band roar, a combustion buzz that never quite settles, and the
        // hiss of propellant through the feed lines. A cough when it lights.
        if (aps > 0.5f && aps_was <= 0.5f) {
            hit_start(&ign_hit[0], 0.70f, 0.16f, 70, 120, 900, 0.8f, 1.8f);
            hit_start(&ign_hit[1], 0.30f, 0.50f, 44, 10, 250, 1.0f, 0.9f);
        }
        aps_was = aps;
        float apsv = 0;
        if (aps > 1e-4f) {
            aps_bp1 += (w1 - aps_bp1) * k_aps_hi;
            aps_bp2 += (aps_bp1 - aps_bp2) * k_aps_lo;
            aps_hiss1 += (w0 - aps_hiss1) * k_hiss_hi;
            aps_hiss2 += (aps_hiss1 - aps_hiss2) * k_hiss_lo;
            aps_flut += (white() - aps_flut) * k_flut;
            aps_jit += (white() - aps_jit) * k_flut * 0.3f;
            aps_ph += TAU * (88.0f + 40.0f * aps_jit) / sr;
            if (aps_ph > TAU) aps_ph -= TAU;
            float saw = aps_ph / TAU * 2.0f - 1.0f;   // ragged buzz: a band-limited saw would be politer than the real thing
            float rough = 0.55f + aps_flut * 7.0f;
            apsv = ((aps_bp1 - aps_bp2) * 2.6f + eng_lp1 * 2.4f + saw * 0.035f + (aps_hiss1 - aps_hiss2) * 0.12f) * rough * aps * 0.55f;
        }

        // RCS: the knock of each jet lighting, then a faint hiss while jets stay on.
        rcs_lp += (w1 - rcs_lp) * k_rcs;
        float r = rcs_lp * 0.5f * fminf(rcs, 2.0f) * 0.22f;
        for (int k = 0; k < NHIT; k++) r += hit_run(&rcs_hit[k]);

        // Master alarm: 3 kHz tone in the headsets, pulsed so it nags without drilling.
        float al = 0;
        if (alarm > 0.001f) {
            alarm_ph += TAU * 3000.0f / sr;
            if (alarm_ph > TAU) alarm_ph -= TAU;
            alarm_gate_ph += 3.0f / sr;
            if (alarm_gate_ph > 1.0f) alarm_gate_ph -= 1.0f;
            float gate = alarm_gate_ph < 0.5f ? 1.0f : 0.0f;
            al = sinf(alarm_ph) * 0.045f * alarm * gate;
        }

        float one = 0;
        for (int k = 0; k < 4; k++) one += hit_run(&click_hit[k]);
        for (int k = 0; k < 2; k++) one += hit_run(&thud_hit[k]) + hit_run(&crash_hit[k]) + hit_run(&ign_hit[k]) + hit_run(&step_hit[k]);
        for (int k = 0; k < 4; k++) one += hit_run(&pyro_hit[k]);

        float mono = hum + eng + apsv + r + al + one;
        for (int c = 0; c < 2; c++) {
            float v = (mono + fan[c]) * gain;
            v = tanhf(v * 1.4f) / 1.4f;   // soft limit: a crash must not clip
            out[2 * i + c] = (short)(v * 30000.0f);
        }
    }
}

// --- Game thread --------------------------------------------------------------------

void sound_init(void)
{
    ma_device_config cfg = ma_device_config_init(ma_device_type_playback);
    cfg.playback.format = ma_format_s16;
    cfg.playback.channels = 2;
    cfg.sampleRate = 0;                      // the device's own rate: nothing to resample
    cfg.periodSizeInFrames = PERIOD_FRAMES;
    cfg.periods = 3;
    cfg.dataCallback = audio_cb;
    if (ma_device_init(NULL, &cfg, &device) != MA_SUCCESS) {
        fprintf(stderr, "SOUND: no audio device, running silent\n");
        return;
    }
    sr = (float)device.sampleRate;
    k_eng1 = lp_coef(70);
    k_eng2 = lp_coef(260);
    k_wob = lp_coef(6);
    k_fan_hi = lp_coef(2200);
    k_fan_lo = lp_coef(500);
    k_rcs = lp_coef(1500);
    k_aps_hi = lp_coef(700);
    k_aps_lo = lp_coef(140);
    k_hiss_hi = lp_coef(5000);
    k_hiss_lo = lp_coef(1800);
    k_flut = lp_coef(11);
    g_aps = 1.0f - expf(-1.0f / (0.040f * sr));
    g_gain = 1.0f - expf(-1.0f / (0.057f * sr));
    g_thrust = 1.0f - expf(-1.0f / (0.150f * sr));
    g_rcs = 1.0f - expf(-1.0f / (0.011f * sr));
    g_alarm = 1.0f - expf(-1.0f / (0.008f * sr));
    if (ma_device_start(&device) != MA_SUCCESS) {
        fprintf(stderr, "SOUND: audio device would not start, running silent\n");
        ma_device_uninit(&device);
        return;
    }
    fprintf(stderr, "SOUND: %s, %u Hz, %u-frame periods\n", ma_get_backend_name(device.pContext->backend),
            (unsigned)device.sampleRate, (unsigned)device.playback.internalPeriodSizeInFrames);
    ready = 1;
}

void sound_shutdown(void)
{
    if (ready) ma_device_uninit(&device);
    ready = 0;
}

void sound_update(const SoundState *s, float dt)
{
    static float alarm_age;
    static int prev_alarm;
    if (!ready) return;
    int on = s->inside && s->running && !muted;
    t_gain = on ? 1.0f : 0.0f;
    t_thrust = s->thrust < 0 ? 0 : s->thrust > 1 ? 1 : s->thrust;
    t_aps = s->aps < 0 ? 0 : s->aps > 1 ? 1 : s->aps;
    t_rcs = s->rcs;
    t_fans = s->fans < 0 ? 0 : s->fans > 2 ? 2 : s->fans;
    if (on && s->rcs_hits > 0) ev_rcs += (unsigned)s->rcs_hits;
    // The tone stops by itself after a while if nobody pushes MASTER ALARM
    // (in the window-camera view there is no button to push).
    if (s->alarm && !prev_alarm) alarm_age = 0;
    prev_alarm = s->alarm;
    if (s->alarm) alarm_age += dt;
    t_alarm = s->alarm && alarm_age < 12.0f ? 1.0f : 0.0f;
}

void sound_switch(void) { ev_switch++; }
void sound_key(void) { ev_key++; }

void sound_touchdown(float speed)
{
    float g = 0.35f + speed / 4.0f;
    ev_thud_gain = g > 1.6f ? 1.6f : g;
    ev_thud++;
}

void sound_crash(void) { ev_crash++; }
void sound_stage(void) { ev_stage++; }

void sound_step(float speed)
{
    float g = 0.4f + speed;
    ev_step_gain = g > 1.5f ? 1.5f : g;
    ev_step++;
}

void sound_set_muted(int m) { muted = m; }
int sound_muted(void) { return muted; }
