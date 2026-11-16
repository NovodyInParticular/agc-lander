#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "agc_host.h"
#include "agc_mem.h"
#include "crew.h"
#include "lm.h"
#include "milestones.h"
#include "snapshot.h"

// data/snapshots.c (tools/embed_bin.py --multi, `make snapshots`).
extern const unsigned char ms_tig30[], ms_radar[], ms_highgate[], ms_150m[], ms_track[];
extern const unsigned int ms_tig30_len, ms_radar_len, ms_highgate_len, ms_150m_len, ms_track_len;

static const struct {
    const char *name, *title;
    const unsigned char *data;
    const unsigned int *len;
    const char *callout;   // Mission Control, as in the Apollo 11 air-to-ground transcript
} table[MS_COUNT] = {
    [MS_TIG30] = { "TIG-30", "30 seconds to ignition", ms_tig30, &ms_tig30_len,
                   "Houston: \"Eagle, Houston. If you read, you're GO for powered descent.\"" },
    [MS_RADAR] = { "RADAR", "Landing radar data good", ms_radar, &ms_radar_len,
                   "Houston: \"Eagle, Houston. You are GO to continue powered descent.\"" },
    [MS_HIGHGATE] = { "HIGH GATE", "High gate, 8,500 ft", ms_highgate, &ms_highgate_len,
                      "Houston: \"Eagle, Houston. You're GO for landing.\"" },
    [MS_150M] = { "150 M", "500 ft", ms_150m, &ms_150m_len, "" },
};

const char *milestone_title(int ms) { return ms >= 0 && ms < MS_COUNT ? table[ms].title : ""; }

const char *milestone_name(int ms) { return ms >= 0 && ms < MS_COUNT ? table[ms].name : "?"; }
const char *milestone_callout(int ms) { return ms >= 0 && ms < MS_COUNT ? table[ms].callout : ""; }

const uint8_t *milestone_blob(int ms, size_t *len)
{
    *len = 0;
    if (ms < 0 || ms >= MS_COUNT || !*table[ms].len) return NULL;
    *len = *table[ms].len;
    return table[ms].data;
}

double milestone_time(int ms)
{
    size_t n;
    const uint8_t *b = milestone_blob(ms, &n);
    SnapInfo in;
    if (!b || snapshot_peek(b, n, &in)) return 0;
    return in.agc_time;
}

int milestone_load(int ms, char *err, size_t errn)
{
    size_t n;
    const uint8_t *b = milestone_blob(ms, &n);
    if (!b) { snprintf(err, errn, "this build has no %s save state (make snapshots)", milestone_name(ms)); return -1; }
    return snapshot_load(b, n, NULL, err, errn);
}

// --- Track ------------------------------------------------------------------------

static const char *flag_names[TRACK_NFLAGS] = {
    "FLAGWRD0", "FLAGWRD1", "FLAGWRD2", "FLAGWRD3", "FLAGWRD4", "FLAGWRD5", "FLAGWRD6", "FLAGWRD7",
    "FLAGWRD8", "FLAGWRD9", "FLGWRD10", "FLGWRD11", "FLGWRD12", "FLGWRD13", "DAPBOOLS", "RADMODES",
};

static uint8_t two_digits(const uint8_t d[2])
{
    char a = agc_digit_char(d[0]), b = agc_digit_char(d[1]);
    if (a < '0' || a > '9' || b < '0' || b > '9') return 255;
    return (uint8_t)((a - '0') * 10 + (b - '0'));
}

void track_sample_now(TrackSample *s, int crew_acted)
{
    memset(s, 0, sizeof *s);
    s->t = lm.t;
    s->r = lm.r;
    s->v = lm.v;
    s->q = lm.q;
    for (int i = 0; i < TRACK_NFLAGS; i++) s->flags[i] = (uint16_t)agc_get_word(flag_names[i], 0);
    const DskyState *d = agc_host_dsky();
    s->prog = two_digits(d->prog);
    s->verb = two_digits(d->verb);
    s->noun = two_digits(d->noun);
    s->flashing = (agc_host_read_channel(011) & 040) != 0;
    s->arm = (uint8_t)lm.engine_armed;
    s->mode = (uint8_t)lm_get_attitude_mode();
    s->dps = (uint8_t)lm.dps_on;
    s->crew = (uint8_t)(crew_acted != 0);
}

#define TRACK_MAGIC 0x4B525441u   // "ATRK"
#define TRACK_VERSION 1

static void track_io(Snap *s, TrackSample *t)
{
    snap_f64(s, &t->t);
    snap_v3(s, &t->r);   // doubles: a float would lose ~0.1 m at lunar radius
    float f[7] = { (float)t->v.x, (float)t->v.y, (float)t->v.z, (float)t->q.w, (float)t->q.x, (float)t->q.y, (float)t->q.z };
    snap_f32v(s, f, 7);
    if (s->mode == SNAP_LOAD) {
        t->v = v3(f[0], f[1], f[2]);
        t->q = (Q4){ f[3], f[4], f[5], f[6] };
    }
    snap_u16v(s, t->flags, TRACK_NFLAGS);
    snap_u8(s, &t->prog); snap_u8(s, &t->verb); snap_u8(s, &t->noun);
    snap_u8(s, &t->flashing); snap_u8(s, &t->arm); snap_u8(s, &t->mode); snap_u8(s, &t->dps); snap_u8(s, &t->crew);
}

size_t track_encode(const TrackSample *samples, int n, uint8_t **out)
{
    Snap s = { 0 };
    s.mode = SNAP_SAVE;
    uint32_t magic = TRACK_MAGIC, version = TRACK_VERSION, rope = agc_host_rope_crc();
    int32_t count = n;
    snap_u32(&s, &magic);
    snap_u32(&s, &version);
    snap_u32(&s, &rope);
    snap_i32(&s, &count);
    for (int i = 0; i < n; i++) {
        TrackSample t = samples[i];
        track_io(&s, &t);
    }
    *out = s.buf;
    return s.error ? 0 : s.len;
}

static TrackSample *track;
static int ntrack = -1;

static void track_decode(void)
{
    if (ntrack >= 0) return;
    ntrack = 0;
    if (!ms_track_len) return;
    Snap s = { 0 };
    s.mode = SNAP_LOAD;
    s.buf = (uint8_t *)ms_track;
    s.len = ms_track_len;
    uint32_t magic = 0, version = 0, rope = 0;
    int32_t count = 0;
    snap_u32(&s, &magic);
    snap_u32(&s, &version);
    snap_u32(&s, &rope);
    snap_i32(&s, &count);
    if (s.error || magic != TRACK_MAGIC || version != TRACK_VERSION || rope != agc_host_rope_crc() || count <= 0 || count > 100000) return;
    track = calloc((size_t)count, sizeof *track);
    if (!track) return;
    for (int i = 0; i < count; i++) track_io(&s, &track[i]);
    if (s.error) { free(track); track = NULL; return; }
    ntrack = count;
}

int track_count(void) { track_decode(); return ntrack; }
const TrackSample *track_at_index(int i) { track_decode(); return i >= 0 && i < ntrack ? &track[i] : NULL; }

// The sample at or just before t (samples are 1 s apart), or -1.
static int track_index(double t)
{
    track_decode();
    if (ntrack < 2 || t < track[0].t || t > track[ntrack - 1].t) return -1;
    double dt = (track[ntrack - 1].t - track[0].t) / (ntrack - 1);
    int i = (int)((t - track[0].t) / dt);
    if (i < 0) i = 0;
    if (i > ntrack - 2) i = ntrack - 2;
    while (i > 0 && track[i].t > t) i--;
    while (i < ntrack - 2 && track[i + 1].t <= t) i++;
    return i;
}

// --- Is this the canonical flight? ------------------------------------------------

// Tolerances: the player's flight may differ from the canonical one by this much
// and still continue from the canonical milestone.
#define TOL_POS 30.0     // m
#define TOL_VEL 0.3      // m/s
#define TOL_ATT 1.5      // deg

// AGC flag bits that only reflect the display or a moment-to-moment state, not
// a decision of the crew: ignored in the comparison (filled in from the track).
static const uint16_t flag_ignore[TRACK_NFLAGS] = { 0 };

int milestone_on_track(char *why, size_t whyn)
{
    char dummy[8];
    if (!why) { why = dummy; whyn = sizeof dummy; }
    int i = track_index(lm.t);
    if (i < 0) { snprintf(why, whyn, "outside the canonical flight's time"); return 0; }
    const TrackSample *a = &track[i], *b = &track[i + 1];
    double f = (lm.t - a->t) / (b->t - a->t);
    V3 r = v3_add(a->r, v3_scale(v3_sub(b->r, a->r), f));
    V3 v = v3_add(a->v, v3_scale(v3_sub(b->v, a->v), f));
    const TrackSample *n = f < 0.5 ? a : b;
    double dr = v3_len(v3_sub(lm.r, r)), dv = v3_len(v3_sub(lm.v, v));
    if (dr > TOL_POS) { snprintf(why, whyn, "%.0f m from the canonical position", dr); return 0; }
    if (dv > TOL_VEL) { snprintf(why, whyn, "%.2f m/s from the canonical velocity", dv); return 0; }
    // Attitude: normalised linear blend of the two samples (they are 1 s apart).
    Q4 qa = a->q, qb = b->q;
    double sgn = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z < 0 ? -1 : 1;
    Q4 q = q4_normalize((Q4){ qa.w + (sgn * qb.w - qa.w) * f, qa.x + (sgn * qb.x - qa.x) * f,
                              qa.y + (sgn * qb.y - qa.y) * f, qa.z + (sgn * qb.z - qa.z) * f });
    double dot = fabs(q.w * lm.q.w + q.x * lm.q.x + q.y * lm.q.y + q.z * lm.q.z);
    double datt = 2 * acos(dot > 1 ? 1 : dot) * 180 / PI_D;
    if (datt > TOL_ATT) { snprintf(why, whyn, "attitude %.1f deg from the canonical", datt); return 0; }
    TrackSample now;
    track_sample_now(&now, 0);
    if (now.prog != n->prog) { snprintf(why, whyn, "program P%02d, canonical P%02d", now.prog, n->prog); return 0; }
    if (now.flashing != n->flashing || (n->flashing && (now.verb != n->verb || now.noun != n->noun))) {
        snprintf(why, whyn, "DSKY V%02dN%02d%s, canonical V%02dN%02d%s", now.verb, now.noun, now.flashing ? " flashing" : "",
                 n->verb, n->noun, n->flashing ? " flashing" : "");
        return 0;
    }
    if (now.arm != n->arm) { snprintf(why, whyn, "engine arm differs"); return 0; }
    if (now.mode != n->mode) { snprintf(why, whyn, "PGNS mode control differs"); return 0; }
    if (now.dps != n->dps) { snprintf(why, whyn, "descent engine %s", now.dps ? "on" : "off"); return 0; }
    for (int k = 0; k < TRACK_NFLAGS; k++) {
        uint16_t m = (uint16_t)~flag_ignore[k];
        if ((now.flags[k] & m) != (a->flags[k] & m) && (now.flags[k] & m) != (b->flags[k] & m)) {
            snprintf(why, whyn, "%s %05o, canonical %05o", flag_names[k], now.flags[k], n->flags[k]);
            return 0;
        }
    }
    const DskyState *d = agc_host_dsky();
    if (d->prog_lamp || d->opr_err || d->restart) { snprintf(why, whyn, "an alarm lamp is lit"); return 0; }
    if (crew_typing() || agc_host_keys_queued()) { snprintf(why, whyn, "keys still going in"); return 0; }
    snprintf(why, whyn, "on track (%.1f m, %.3f m/s, %.2f deg)", dr, dv, datt);
    return 1;
}

// Jumps worth making: at least this far ahead.
#define MIN_JUMP 20.0

int milestone_advance(void)
{
    int ms = milestone_next_event(NULL, 0);
    if (ms < 0) return -1;
    int ac = crew.auto_crew, al = crew.auto_land;
    char err[200];
    if (milestone_load(ms, err, sizeof err)) return -1;
    crew.auto_crew = ac;
    crew.auto_land = al;
    return ms;
}

int milestone_next_event(char *why, size_t whyn)
{
    static const int targets[] = { MS_TIG30, MS_RADAR, MS_HIGHGATE };
    char dummy[8];
    if (!why) { why = dummy; whyn = sizeof dummy; }
    int i = track_index(lm.t);
    if (i < 0) { snprintf(why, whyn, "no events ahead"); return -1; }
    for (size_t k = 0; k < sizeof targets / sizeof targets[0]; k++) {
        int ms = targets[k];
        double tm = milestone_time(ms);
        if (tm <= lm.t) continue;
        if (tm < lm.t + MIN_JUMP) { snprintf(why, whyn, "%s is only seconds away", milestone_title(ms)); return -1; }
        // Nothing for the crew to do between now and then.
        for (int j = i; j < ntrack && track[j].t <= tm; j++)
            if (track[j].crew) { snprintf(why, whyn, "something needs doing first"); return -1; }
        if (!milestone_on_track(why, whyn)) return -1;
        return ms;
    }
    snprintf(why, whyn, "no events ahead");
    return -1;
}
