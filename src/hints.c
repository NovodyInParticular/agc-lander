// Plain-English hints for new players. Everything here is derived from state
// that co-op clients also have (LM state, DSKY, crew.hint), so the LMP sees
// the same advice as the Commander.
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"

#include "agc_host.h"
#include "ascent.h"
#include "cockpit.h"
#include "crew.h"
#include "eva.h"
#include "hints.h"
#include "hud.h"
#include "lm.h"

int hints_on = 1;

static const char *next_event = "";
void hints_set_next_event(const char *title) { next_event = title ? title : ""; }
static const char *note = "";
static int must_take_over;
void hints_set_note(const char *line, int take_over) { note = line ? line : ""; must_take_over = take_over; }

static int in_cockpit;   // the view is the Commander's station (controls can be ringed)
static int in_cockpit_view(void) { return in_cockpit; }

#define FT 0.3048
#define MAXL 4
#define MAXS 6

// ---------------------------------------------------------------------------
// Settings file: one "hints=0|1" line.

static void cfg_path(char *buf, size_t n)
{
#ifdef _WIN32
    const char *base = getenv("APPDATA");
    snprintf(buf, n, "%s\\agc-lander.conf", base ? base : ".");
#elif defined(__ANDROID__)
    extern const char *agc_android_data_dir(void);
    const char *base = agc_android_data_dir();
    snprintf(buf, n, "%s/agc-lander.conf", base ? base : ".");
#else
    const char *xdg = getenv("XDG_CONFIG_HOME"), *home = getenv("HOME");
    if (xdg && xdg[0]) snprintf(buf, n, "%s/agc-lander.conf", xdg);
    else snprintf(buf, n, "%s/.config/agc-lander.conf", home ? home : ".");
#endif
}

void hints_load(void)
{
    char p[512];
    cfg_path(p, sizeof p);
    FILE *f = fopen(p, "r");
    if (!f) return;
    char line[128];
    while (fgets(line, sizeof line, f))
        if (!strncmp(line, "hints=", 6)) hints_on = atoi(line + 6) != 0;
    fclose(f);
}

void hints_save(void)
{
    char p[512];
    cfg_path(p, sizeof p);
    FILE *f = fopen(p, "w");
    if (!f) return;
    fprintf(f, "hints=%d\n", hints_on);
    fclose(f);
}

// ---------------------------------------------------------------------------

static int prog_is(int p)
{
    const DskyState *d = agc_host_dsky();
    return agc_digit_char(d->prog[0]) == '0' + p / 10 && agc_digit_char(d->prog[1]) == '0' + p % 10;
}

// A walkthrough step: what to do, and the keys (PC) or controller buttons (headset)
// that do it, drawn as key caps - '|' between caps.
enum { ST_TODO, ST_NOW, ST_DONE };
typedef struct { char label[120]; char caps[48]; int state, vb; } Step;
typedef struct {
    char line[MAXL][160]; int n; int urgent; int vrbtn;
    char title[120];          // the checklist's heading ("" = no checklist)
    Step step[MAXS]; int nstep;
    char caps[48];            // what to press for the advice in the lines (no checklist step is current)
} Hint;

// The same hint in the headset's own terms (controller buttons, pointing at
// the cabin's switches) instead of keyboard keys.
#ifdef AGC_VR
#define VRT(pc, vr) (vr)
#else
#define VRT(pc, vr) (pc)
#endif

static void add(Hint *h, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void add(Hint *h, const char *fmt, ...)
{
    if (h->n >= MAXL) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(h->line[h->n++], sizeof h->line[0], fmt, ap);
    va_end(ap);
}

static void caps(Hint *h, const char *keys) { snprintf(h->caps, sizeof h->caps, "%s", keys); }

static void step(Hint *h, const char *label, const char *keys, int vb)
{
    if (h->nstep >= MAXS) return;
    Step *st = &h->step[h->nstep++];
    snprintf(st->label, sizeof st->label, "%s", label);
    snprintf(st->caps, sizeof st->caps, "%s", keys);
    st->state = ST_TODO;
    st->vb = vb;
}

// Tick off the checklist from `done` (bit i = step i is done): the first step not
// done is the one to do now; its controller button glows in the headset.
// wait: the next step is not due yet (nothing is marked as "now").
static void mark_steps(Hint *h, unsigned done, int wait)
{
    int now = 0;
    for (int i = 0; i < h->nstep; i++) {
        if (done & (1u << i)) h->step[i].state = ST_DONE;
        else if (!now && !wait) { h->step[i].state = ST_NOW; now = 1; h->vrbtn = h->step[i].vb; }
        else { h->step[i].state = ST_TODO; now = 1; }
    }
}

static int vn_is(int v, int n)
{
    const DskyState *d = agc_host_dsky();
    return agc_digit_char(d->verb[0]) == '0' + v / 10 && agc_digit_char(d->verb[1]) == '0' + v % 10 &&
           agc_digit_char(d->noun[0]) == '0' + n / 10 && agc_digit_char(d->noun[1]) == '0' + n % 10;
}

// Seconds of descent-engine propellant at the current thrust (Isp ~305 s).
static double fuel_seconds(void)
{
    double flow = lm.thrust / (305.0 * 9.80665);
    return flow > 0.5 ? lm.m_dps / flow : 1e9;
}

// Hand-flying advice for P66 (and any time below 150 m with the engine on).
static void flying_hints(Hint *h)
{
    double alt = lm.altitude;
    double fs = fuel_seconds();
    if (fs < 60) {
        add(h, "Fuel: about %.0f seconds left. Get it on the ground now!", fs);
        h->urgent = 1;
    }

    // Vertical: aim for a descent rate that shrinks as the ground gets closer.
    double target = fmax(0.6, fmin(4.5, alt / 12.0));      // m/s down
    double down = -lm.hdot;
    double diff_ft = (down - target) / FT;                  // + = too fast
    if (alt < 4)
        add(h, "Almost down: hold steady and wait for the blue CONTACT light");
    else if (down > target + 0.8)
        add(h, VRT("Coming down too fast (%.0f ft/s): tap Up about %d times to slow down",
                   "Coming down too fast (%.0f ft/s): flick the LEFT stick up about %d times to slow down"),
            down / FT, (int)lround(fmin(15, diff_ft))), h->vrbtn = VB_LSTICK_UP, caps(h, VRT("UP", "L STICK UP"));
    else if (down < target * 0.4 - 0.15)
        add(h, lm.hdot > 0.2 ? VRT("You're climbing: tap Down about %d times to descend", "You're climbing: flick the LEFT stick down about %d times")
                             : VRT("Barely descending: tap Down about %d times", "Barely descending: flick the LEFT stick down about %d times"),
            (int)lround(fmin(15, fmax(1, -diff_ft)))), h->vrbtn = VB_LSTICK_DOWN, caps(h, VRT("DOWN", "L STICK DOWN"));

    // Horizontal drift, in the LM's own forward/right directions.
    V3 up = v3_norm(lm.r);
    V3 vs = lm_surface_velocity();
    V3 vh = v3_sub(vs, v3_scale(up, v3_dot(vs, up)));
    V3 fwd = lm_body_to_inertial(v3(0, 0, 1)), right = lm_body_to_inertial(v3(0, 1, 0));
    fwd = v3_norm(v3_sub(fwd, v3_scale(up, v3_dot(fwd, up))));
    right = v3_norm(v3_sub(right, v3_scale(up, v3_dot(right, up))));
    double vf = v3_dot(vh, fwd), vr = v3_dot(vh, right);
    double lim = alt < 20 ? 0.35 : 0.8;
    double tilt = lm_tilt();
    V3 up_b = lm_inertial_to_body(up);   // +z: leaning back, -y: leaning right

    const char *lean = "";
    if (fabs(vf) > lim && fabs(vf) >= fabs(vr)) lean = vf > 0 ? "S" : "W";
    else if (fabs(vr) > lim) lean = vr > 0 ? "A" : "D";
    else if (tilt > 10 * DEG2RAD) lean = fabs(up_b.z) >= fabs(up_b.y) ? (up_b.z > 0 ? "W" : "S") : (up_b.y < 0 ? "A" : "D");
    if (lean[0] && !h->caps[0]) caps(h, VRT(lean, "R STICK"));
    if (fabs(vf) > lim && fabs(vf) >= fabs(vr))
        add(h, vf > 0 ? VRT("Drifting forward (%.0f ft/s): hold S briefly to lean back, then let go",
                            "Drifting forward (%.0f ft/s): pull the RIGHT stick back briefly to lean back")
                      : VRT("Drifting backward (%.0f ft/s): hold W briefly to lean forward, then let go",
                            "Drifting backward (%.0f ft/s): push the RIGHT stick forward briefly to lean forward"),
            fabs(vf) / FT), h->vrbtn = VB_RSTICK;
    else if (fabs(vr) > lim)
        add(h, vr > 0 ? VRT("Drifting right (%.0f ft/s): hold A briefly to lean left, then let go",
                            "Drifting right (%.0f ft/s): push the RIGHT stick left briefly to lean left")
                      : VRT("Drifting left (%.0f ft/s): hold D briefly to lean right, then let go",
                            "Drifting left (%.0f ft/s): push the RIGHT stick right briefly to lean right"),
            fabs(vr) / FT), h->vrbtn = VB_RSTICK;
    else if (tilt > 10 * DEG2RAD) {
        if (fabs(up_b.z) >= fabs(up_b.y))
            add(h, up_b.z > 0 ? VRT("Leaning back %.0f deg: tap W to level out", "Leaning back %.0f deg: nudge the RIGHT stick forward to level out")
                               : VRT("Leaning forward %.0f deg: tap S to level out", "Leaning forward %.0f deg: nudge the RIGHT stick back to level out"),
                tilt * RAD2DEG), h->vrbtn = VB_RSTICK;
        else
            add(h, up_b.y < 0 ? VRT("Leaning right %.0f deg: tap A to level out", "Leaning right %.0f deg: nudge the RIGHT stick left to level out")
                               : VRT("Leaning left %.0f deg: tap D to level out", "Leaning left %.0f deg: nudge the RIGHT stick right to level out"),
                tilt * RAD2DEG), h->vrbtn = VB_RSTICK;
    }
    if (tilt > LAND_CRASH_TILT * 0.8 && alt < 30) h->urgent = 1;
    if (down > LAND_CRASH_V * 0.8 && alt < 20) h->urgent = 1;
}

int hud_camera_is_docking;

// Docking: one thing at a time, the most urgent first. Keys as seen through
// the docking window (Tab): arrows slide, T/G close/open, Q/E turn about the
// docking axis, W/S and A/D tilt.
static void docking_hints(Hint *h)
{
    DockGeo g;
    if (!ascent_dock_geo(&g)) return;
    double drift_r = -g.vy, drift_u = g.vz;           // screen right / up
    double off_r = -g.lat_y, off_u = g.lat_z;
    double closing = -g.rate;
    if (fabs(drift_r) > 0.05 || fabs(drift_u) > 0.05) {
        if (fabs(drift_r) >= fabs(drift_u))
            add(h, VRT("Columbia is drifting %s (%.2f m/s): tap the %s arrow to stop it", "Columbia is drifting %s (%.2f m/s): nudge the LEFT stick %s to stop it"),
                drift_r > 0 ? "right" : "left", fabs(drift_r), VRT(drift_r > 0 ? "Right" : "Left", drift_r > 0 ? "right" : "left")),
                caps(h, VRT(drift_r > 0 ? "RIGHT" : "LEFT", "L STICK"));
        else
            add(h, VRT("Columbia is drifting %s (%.2f m/s): tap the %s arrow to stop it", "Columbia is drifting %s (%.2f m/s): nudge the LEFT stick %s to stop it"),
                drift_u > 0 ? "up" : "down", fabs(drift_u), VRT(drift_u > 0 ? "Up" : "Down", drift_u > 0 ? "up" : "down")),
                caps(h, VRT(drift_u > 0 ? "UP" : "DOWN", "L STICK"));
        h->vrbtn = VB_LSTICK;
        return;
    }
    if (fabs(g.roll) > 4 * DEG2RAD) {
        add(h, VRT("Roll %.0f deg off: hold %s to turn until the target's cross is square in the sight",
                   "Roll %.0f deg off: hold the RIGHT grip and push the RIGHT stick %s until the cross is square"),
            fabs(g.roll) * RAD2DEG, VRT(g.roll > 0 ? "Q" : "E", g.roll > 0 ? "left" : "right"));
        caps(h, VRT(g.roll > 0 ? "Q" : "E", "R GRIP|R STICK"));
        h->vrbtn = VB_RGRIP;
        return;
    }
    if (g.angle > 3 * DEG2RAD) {
        add(h, VRT("Axes %.1f deg apart: tilt with W/S and A/D until the standoff cross sits in the middle of its disc",
                   "Axes %.1f deg apart: tilt with the RIGHT stick until the standoff cross sits in the middle of its disc"),
            g.angle * RAD2DEG);
        caps(h, VRT("W|A|S|D", "R STICK"));
        h->vrbtn = VB_RSTICK;
        return;
    }
    if (fabs(off_r) > 0.12 || fabs(off_u) > 0.12) {
        if (fabs(off_r) >= fabs(off_u))
            add(h, VRT("The probe is %.1f m %s of the drogue: tap the %s arrow, then stop the drift", "The probe is %.1f m %s of the drogue: nudge the LEFT stick %s, then stop the drift"),
                fabs(off_r), off_r > 0 ? "right" : "left", VRT(off_r > 0 ? "Right" : "Left", off_r > 0 ? "right" : "left")),
                caps(h, VRT(off_r > 0 ? "RIGHT" : "LEFT", "L STICK"));
        else
            add(h, VRT("The probe is %.1f m %s of the drogue: tap the %s arrow, then stop the drift", "The probe is %.1f m %s of the drogue: nudge the LEFT stick %s, then stop the drift"),
                fabs(off_u), off_u > 0 ? "above" : "below", VRT(off_u > 0 ? "Up" : "Down", off_u > 0 ? "up" : "down")),
                caps(h, VRT(off_u > 0 ? "UP" : "DOWN", "L STICK"));
        h->vrbtn = VB_LSTICK;
        return;
    }
    if (g.axial > 2 && closing > 0.35) {
        add(h, VRT("Closing at %.2f m/s: too fast. Tap G to slow to 0.1-0.3", "Closing at %.2f m/s: too fast. Tap the LEFT trigger to slow to 0.1-0.3"), closing);
        h->urgent = g.axial < 8;
        caps(h, VRT("G", "L TRIGGER"));
        h->vrbtn = VB_LTRIG;
        return;
    }
    if (g.axial > 1 && closing < 0.04) {
        add(h, VRT("Lined up. Tap T to close in at about 0.1-0.3 m/s (RATE about -0.2)", "Lined up. Tap the LEFT grip to close in at about 0.1-0.3 m/s (RATE about -0.2)"));
        caps(h, VRT("T", "L GRIP"));
        h->vrbtn = VB_LGRIP;
        return;
    }
    add(h, "On the way in at %.2f m/s, %.0f m to go: keep the target centred", closing, fmax(0, g.axial));
}

// On the surface: walk, plant the flag, back in at the ladder.
static void moonwalk_hints(Hint *h, int countdown)
{
    snprintf(h->title, sizeof h->title, "%s", countdown ? "Moonwalk: the countdown waits for you" : "Moonwalk");
    step(h, VRT("Walk: W A S D, Shift to lope, Space to jump", "Walk: LEFT stick, left grip to lope, A to jump"), VRT("W|A|S|D", "L STICK"), VB_LSTICK);
    step(h, VRT("Plant the flag", "Take the flag (RIGHT grip), let go to plant it"), VRT("F", "R GRIP"), VB_RGRIP);
    step(h, VRT("Back in: walk to the foot of the ladder and press X", "Back in: grab the ladder, or press X at its foot"), "X", VB_X);
    mark_steps(h, 1u | (eva_flag_planted() ? 2u : 0), 0);
}

static void ascent_hints(Hint *h)
{
    int ph = ascent_phase();
    if (ph == MP_COUNTDOWN) {
        if (eva_phase() == EVA_SURFACE) { moonwalk_hints(h, 1); return; }
        // At V99: ABORT STAGE, ENG ARM to ASC, PRO - "Abort Stage, Engine Arm Ascent, Proceed".
        int v99 = crew.hint == HINT_ASC_STAGE || crew.hint == HINT_ASC_ARM || crew.hint == HINT_ASC_PRO;
        if (crew.auto_crew)
            snprintf(h->title, sizeof h->title, "%s", VRT("Lift-off: the auto crew does this at V99 (F2 to do it yourself)", "Lift-off: the auto crew does this at V99"));
        else
            snprintf(h->title, sizeof h->title, "%s", v99 ? "Lift-off NOW: three steps" : "Lift-off checklist: when V99 flashes on the DSKY");
        step(h, VRT("ABORT STAGE: fires the bolts holding the two stages together", "ABORT STAGE (ringed green): point at it, pull the trigger"), VRT("B", "TRIGGER"), VB_RTRIG);
        step(h, VRT("ENG ARM to ASC: arms the ascent engine", "ENG ARM to ASC (ringed green): point, pull the trigger"), VRT("F7", "TRIGGER"), VB_RTRIG);
        step(h, "PRO: the computer lights the ascent engine", VRT("P", "B"), VB_B);
        unsigned done = (lm.abort_stage ? 1u : 0) | (lm.engine_armed == ARM_ASC ? 2u : 0) | (lm.aps_on ? 4u : 0);
        mark_steps(h, done, crew.auto_crew || !v99);
        if (v99 && !crew.auto_crew) h->urgent = 1;
        else if (!crew.auto_crew) {
            add(h, "The DSKY's top number counts down minutes and seconds to lift-off.");
            if (eva_available()) add(h, VRT("Or press X for a moonwalk first", "Or press X for a moonwalk first")), caps(h, "X"), h->vrbtn = VB_X;
        }
        return;
    }
    if (ph == MP_ASCENT) {
        if (lm.aps_on && !lm.staged) {
            add(h, "The ascent engine is firing into the descent stage - you're still bolted to it!");
            add(h, VRT("Press B (ABORT STAGE) now", "Point at ABORT STAGE (ringed green) and pull the trigger now"));
            h->urgent = 1;
            h->vrbtn = VB_RTRIG;
            return;
        }
        add(h, "P12 is flying you to orbit: straight up, then the pitch-over. Nothing to do.");
        add(h, "It shuts the engine off at 5,500 ft/s (about seven minutes)");
        return;
    }
    if (ph == MP_ORBIT) {
        add(h, VRT("In orbit, trailing Columbia. Press Enter to skip the 3 1/2-hour rendezvous", "In orbit, trailing Columbia. Press B to skip the 3 1/2-hour rendezvous"));
        caps(h, VRT("ENTER", "B"));
        h->vrbtn = VB_B;
        return;
    }
    if (ph == MP_PROX) {
        docking_hints(h);
        if (h->n && hud_camera_is_docking == 0) {
            add(h, VRT("Tab to the DOCKING WINDOW view: the target sits in the COAS sight", "Press Y for the DOCKING WINDOW view: the target sits in the COAS sight"));
            caps(h, VRT("TAB", "Y"));
            h->vrbtn = VB_Y;
        }
    }
}

// The pre-burn checklist, ticked off from the state itself (so it is right after a
// save state or J too) and remembered while the computer works between questions.
static unsigned preburn_done(void)
{
    static unsigned best;
    static double last_t = -1;
    if (last_t < 0 || lm.t < last_t - 0.5 || lm.t > last_t + 20) best = 0;   // a new flight, scenario or save state
    last_t = lm.t;
    int hn = crew.hint;
    int tig = vn_is(6, 62) || vn_is(99, 62);
    int burnt = lm.dps_on || prog_is(64) || prog_is(65) || prog_is(66) || lm.touchdown;
    unsigned m = 0;
    if (hn == HINT_MANEUVER || hn == HINT_MANEUVER_DONE || hn == HINT_ARM || hn == HINT_ENABLE_IGN || tig || burnt) m |= 1;
    if (hn == HINT_MANEUVER_DONE || hn == HINT_ARM || hn == HINT_ENABLE_IGN || tig || burnt) m |= 2;
    if (hn == HINT_ARM || hn == HINT_ENABLE_IGN || tig || burnt) m |= 4;
    if (lm.engine_armed || burnt) m |= 8;
    if (burnt) m |= 16;
    best |= m;
    // V57: done once the radar is in use (not remembered: the hint itself says so).
    unsigned v57 = burnt && ((lm.lr_range_good && hn != HINT_V57) || !prog_is(63)) ? 32u : 0;
    return best | v57;
}

static void preburn_hints(Hint *h)
{
    int hn = crew.hint;
    unsigned done = preburn_done();
    int auto_mode = lm_get_attitude_mode() == 2;
    step(h, "Skip the gyro fine-align: ENTR", VRT("ENTER", "L TRIGGER"), VB_LTRIG);
    step(h, VRT("Let it turn to the burn attitude: PGNS mode AUTO, then PRO", "Burn attitude: PGNS mode AUTO (ringed), then PRO"),
         auto_mode ? VRT("P", "B") : VRT("F8|P", "TRIGGER|B"), auto_mode ? VB_B : VB_RTRIG);
    step(h, "Turn finished: ENTR to carry on", VRT("ENTER", "L TRIGGER"), VB_LTRIG);
    step(h, VRT("Arm the descent engine: ENG ARM to DES", "Arm the descent engine: ENG ARM to DES (ringed)"), VRT("F7", "TRIGGER"), VB_RTRIG);
    step(h, "At V99, five seconds to go: PRO lets the engine light", VRT("P", "B"), VB_B);
    step(h, VRT("In the burn, when the radar locks on: V 5 7 ENTR", "In the burn, when the radar locks on: V57 on the DSKY"), VRT("V|5|7|ENTER", "TRIGGER"), VB_RTRIG);
    // Due now: what the computer is asking for (or the engine arm, any time before V99).
    int due = hn == HINT_FINE_ALIGN || hn == HINT_MANEUVER || hn == HINT_MANEUVER_DONE || hn == HINT_ARM || hn == HINT_ENABLE_IGN;
    if (!(done & 8) && (done & 4)) due = 1;
    mark_steps(h, done, crew.auto_crew || !due);
    int n = 0;
    for (int i = 0; i < h->nstep; i++) n += h->step[i].state == ST_DONE;
    if (crew.auto_crew)
        snprintf(h->title, sizeof h->title, "%s", VRT("Before the burn: the auto crew is doing this (F2 to do it yourself)", "Before the burn: the auto crew is doing this checklist"));
    else
        snprintf(h->title, sizeof h->title, "Before the burn: step %d of 6", n + 1 > 6 ? 6 : n + 1);
    if (crew.auto_crew) return;
    switch (hn) {
    case HINT_FINE_ALIGN: add(h, "The computer asks if it should fine-tune its gyros. Not needed: skip it."); break;
    case HINT_MANEUVER: add(h, "The computer wants to turn the LM to the burn attitude."); break;
    case HINT_MANEUVER_DONE: add(h, "The turn is finished. Tell the computer to carry on."); break;
    case HINT_ARM: add(h, "Arm the descent engine now, before ignition."); break;
    case HINT_ENABLE_IGN:
        if (!lm.engine_armed) add(h, "The engine is ready to light, but it is not armed yet!"), h->urgent = 1;
        else add(h, "V99 is flashing: the engine is ready to light.");
        break;
    default:
        if (done & 4) add(h, "Counting down to ignition: the DSKY shows the time to go.");
        else add(h, "The computer is working: wait for its next question on the DSKY.");
        break;
    }
}

static void compute(Hint *h)
{
    memset(h, 0, sizeof *h);
    if (ascent_active()) { ascent_hints(h); return; }
    if (lm.land_result == LAND_CRASH) return;   // the results panel takes over
    if (note[0] && !lm.touchdown) add(h, "%s", note);
    if (lm.touchdown) {
        if (lm.dps_on) {
            add(h, VRT("You're on the ground: press SPACE to shut the engine off", "You're on the ground: press A (ENGINE STOP)"));
            caps(h, VRT("SPACE", "A"));
            h->urgent = 1; h->vrbtn = VB_A;
        }
        else if (eva_phase() == EVA_INSIDE && eva_available()) {
            add(h, VRT("You are on the Moon. Press X to open the hatch and go for a walk,", "You are on the Moon. Grab the hatch handle by your knees, or press X, to go outside"));
#ifndef AGC_VR
            add(h, "or L to skip ahead to lift-off (P12) and the trip back to Columbia");
#endif
            caps(h, VRT("X", "X"));
            h->vrbtn = VB_X;
        }
        else if (eva_phase() == EVA_SURFACE) moonwalk_hints(h, 0);
        return;
    }
    if (lm.probe_contact && lm.dps_on) {
        add(h, VRT("Blue CONTACT light! Press SPACE now to cut the engine", "Blue CONTACT light! Press A now to cut the engine"));
        caps(h, VRT("SPACE", "A"));
        h->urgent = 1;
        h->vrbtn = VB_A;
        return;
    }

    // Program alarm: silence the master alarm, clear the PROG light.
    if (!crew.auto_crew && crew.hint == HINT_ALARM) {
        snprintf(h->title, sizeof h->title, "Program alarm: the computer is overloaded (1202, 1201) but still flying");
        if (cp_master_alarm() || in_cockpit_view())
            step(h, VRT("Silence the MASTER ALARM (the red light, top of the panels)", "Silence the MASTER ALARM: point at it, pull the trigger"),
                 VRT("CLICK IT", "TRIGGER"), VB_RTRIG);
        step(h, VRT("RSET on the DSKY clears the PROG light", "RSET on the cabin DSKY (ringed): point, pull the trigger"), VRT("R", "TRIGGER"), VB_RTRIG);
        mark_steps(h, h->nstep == 2 && !cp_master_alarm() ? 1u : 0, 0);
        add(h, VRT("V05N09E shows the alarm code. Houston will call GO.", "Houston will call GO."));
        return;
    }

    // Landing radar: VERB, 5, 7, ENTR - followed key by key on the DSKY.
    if (!crew.auto_crew && crew.hint == HINT_V57) {
        const DskyState *d = agc_host_dsky();
        char a = agc_digit_char(d->verb[0]), b = agc_digit_char(d->verb[1]);
        unsigned done = a == '5' && b == '7' ? 7u : a == '5' && b == ' ' ? 3u : a == ' ' && b == ' ' ? 1u : 0u;
        snprintf(h->title, sizeof h->title, "Landing radar: tell the computer to use it (V57)");
        add(h, "The radar can see the ground. Four keys on the DSKY, one after another:");
        step(h, VRT("VERB", "VERB on the cabin DSKY (ringed): point, pull the trigger"), VRT("V", "TRIGGER"), VB_RTRIG);
        step(h, "5", VRT("5", "TRIGGER"), VB_RTRIG);
        step(h, "7", VRT("7", "TRIGGER"), VB_RTRIG);
        step(h, VRT("ENTR: the computer starts using the radar", "ENTR: the computer starts using the radar"), VRT("ENTER", "TRIGGER"), VB_RTRIG);
        mark_steps(h, done, 0);
        return;
    }

    int p63 = prog_is(63), p64 = prog_is(64), p65 = prog_is(65), p66 = prog_is(66);
    if (p66 || (lm.dps_on && lm.altitude < 150 && lm_get_attitude_mode() != 2)) {
        flying_hints(h);
        if (h->n == 0) add(h, "Looking good. Keep it slow and level");
        return;
    }
    if (p65) {
        add(h, "The computer is landing for you. Watch for the blue CONTACT light%s",
            crew.auto_crew ? "" : VRT(", then press SPACE", ", then press A"));
        if (!crew.auto_crew) caps(h, VRT("SPACE", "A"));
        return;
    }
    if (p64) {
        if (must_take_over) {
            snprintf(h->title, sizeof h->title, "Take over and land it yourself");
            step(h, VRT("PGNS mode switch to ATT HOLD: you fly, the computer steadies", "PGNS mode to ATT HOLD (ringed): point, pull the trigger"), VRT("F8", "TRIGGER"), VB_RTRIG);
            step(h, VRT("Up / Down: the ROD switch slows or speeds the descent", "LEFT stick up / down: slower / faster descent"), VRT("UP|DOWN", "L STICK"), VB_LSTICK);
            step(h, VRT("Lean with W A S D to move across and find a clear spot", "RIGHT stick: lean to move across, find a clear spot"), VRT("W|A|S|D", "R STICK"), VB_RSTICK);
            mark_steps(h, lm_get_attitude_mode() != 2 ? 1u : 0, 0);
            h->urgent = lm_get_attitude_mode() == 2;
        } else {
            add(h, "The computer is steering to the landing site. Let it fly,");
            add(h, VRT("or take over: press F8 (ATT HOLD), then tap Up/Down",
                       "or take over: set the PGNS mode switch to ATT HOLD, then flick the LEFT stick up/down"));
            caps(h, VRT("F8", "TRIGGER"));
        }
        return;
    }
    if (p63 && lm.dps_on) {
        add(h, "Braking burn: the computer is slowing you down from orbit.");
        if (next_event[0]) add(h, VRT("Nothing to do yet. J skips ahead: %s", "Nothing to do yet. NEXT EVENT skips ahead: %s"), next_event), caps(h, VRT("J", ""));
        else add(h, "Nothing to do yet: watch the Moon come up in the window");
        return;
    }
    if (p63) {
        preburn_hints(h);
        if (next_event[0] && h->n < MAXL) add(h, VRT("J skips ahead: %s", "NEXT EVENT skips ahead: %s"), next_event);
    }
}

// ---------------------------------------------------------------------------
// The control to operate next (same decisions as compute(), reduced to one
// switch, button or DSKY key).

static int key_target(HintTarget *t, int key, const char *text)
{
    t->kind = HT_DSKY_KEY;
    t->arg = key;
    t->text = text;
    return 1;
}

static int target(HintTarget *t, int kind, const char *text)
{
    t->kind = kind;
    t->arg = 0;
    t->text = text;
    return 1;
}

int hints_target(HintTarget *t)
{
    t->kind = HT_NONE;
    t->arg = 0;
    t->text = "";
    // Debugging: AGC_HINT=arm|auto|stop|alarm|rodup|roddown, or a DSKY key letter, forces a target.
    static const char *force;
    static int force_read;
    if (!force_read) { force = getenv("AGC_HINT"); force_read = 1; }
    if (force) {
        if (!strcmp(force, "arm")) return target(t, HT_ENG_ARM, "FLIP DOWN: DES");
        if (!strcmp(force, "auto")) return target(t, HT_PGNS_AUTO, "FLIP UP: AUTO");
        if (!strcmp(force, "stop")) return target(t, HT_ENGINE_STOP, "PUSH: ENGINE STOP");
        if (!strcmp(force, "alarm")) return target(t, HT_MASTER_ALARM, "PUSH: SILENCE ALARM");
        if (!strcmp(force, "rodup")) return target(t, HT_ROD_UP, "CLICK UP: SLOWER");
        if (!strcmp(force, "roddown")) return target(t, HT_ROD_DOWN, "CLICK DOWN: FASTER");
        return key_target(t, force[0], "PRESS THIS KEY");
    }
    if (!hints_on) return 0;
    if (ascent_active()) {
        if (crew.auto_crew) return 0;
        if (lm.aps_on && !lm.staged) return target(t, HT_ABORT_STAGE, "PUSH NOW: ABORT STAGE");
        switch (crew.hint) {
        case HINT_ASC_STAGE: return target(t, HT_ABORT_STAGE, "PUSH: ABORT STAGE");
        case HINT_ASC_ARM: return target(t, HT_ENG_ARM_ASC, "FLIP UP: ASC");
        case HINT_ASC_PRO: return key_target(t, 'P', "PRESS PRO");
        }
        return 0;
    }
    if (lm.land_result == LAND_CRASH) return 0;
    if (lm.touchdown) return lm.dps_on ? target(t, HT_ENGINE_STOP, "PUSH: ENGINE STOP") : 0;
    if (lm.probe_contact && lm.dps_on) return target(t, HT_ENGINE_STOP, "PUSH NOW: ENGINE STOP");

    if (!crew.auto_crew) {
        const DskyState *d = agc_host_dsky();
        switch (crew.hint) {
        case HINT_FINE_ALIGN: return key_target(t, 'E', "PRESS ENTR");
        case HINT_MANEUVER:
            if (lm_get_attitude_mode() != 2) return target(t, HT_PGNS_AUTO, "FLIP UP: AUTO");
            return key_target(t, 'P', "PRESS PRO");
        case HINT_MANEUVER_DONE: return key_target(t, 'E', "PRESS ENTR");
        case HINT_ARM: return target(t, HT_ENG_ARM, "FLIP DOWN: DES");
        case HINT_ENABLE_IGN:
            if (!lm.engine_armed) return target(t, HT_ENG_ARM, "FLIP DOWN: DES");
            return key_target(t, 'P', "PRESS PRO");
        case HINT_V57: {
            // Follow the verb being keyed in: VERB blanks the digits, then 5, 7, ENTR.
            char a = agc_digit_char(d->verb[0]), b = agc_digit_char(d->verb[1]);
            if (a == '5' && b == '7') return key_target(t, 'E', "THEN ENTR");
            if (a == '5' && b == ' ') return key_target(t, '7', "THEN 7");
            if (a == ' ' && b == ' ') return key_target(t, '5', "THEN 5");
            return key_target(t, 'V', "PRESS VERB");
        }
        case HINT_ALARM:
            if (cp_master_alarm()) return target(t, HT_MASTER_ALARM, "PUSH: SILENCE ALARM");
            return key_target(t, 'R', "PRESS RSET");
        }
    }
    if (cp_master_alarm()) return target(t, HT_MASTER_ALARM, "PUSH: SILENCE ALARM");
    if (must_take_over && prog_is(64) && lm_get_attitude_mode() == 2) return target(t, HT_PGNS_ATTHOLD, "FLIP DOWN: ATT HOLD");

    // Hand-flying: the rate-of-descent switch, when the sink rate is off.
    if ((prog_is(66) || (lm.dps_on && lm.altitude < 150 && lm_get_attitude_mode() != 2)) && lm.altitude >= 4) {
        double tgt = fmax(0.6, fmin(4.5, lm.altitude / 12.0)), down = -lm.hdot;
        if (down > tgt + 0.8) return target(t, HT_ROD_UP, "CLICK UP: SLOWER");
        if (down < tgt * 0.4 - 0.15) return target(t, HT_ROD_DOWN, "CLICK DOWN: FASTER");
    }
    return 0;
}

void hints_set_cockpit(int c) { in_cockpit = c; }

int hints_vr_button(void)
{
    if (!hints_on) return VB_NONE;
    Hint h;
    compute(&h);
    return h.n || h.nstep ? h.vrbtn : VB_NONE;
}

// Key caps from a '|'-separated list at (x, y); hot ones pulse green. Returns the width.
static float draw_caps(const char *list, float x, float y, float sc, int hot, int draw)
{
    float cx = x, size = 16 * sc, kh = 26 * sc, pulse = 0.5f + 0.5f * sinf((float)GetTime() * 6.0f);
    const char *p = list;
    while (*p) {
        char lab[24];
        int n = 0;
        while (p[n] && p[n] != '|' && n < 23) { lab[n] = p[n]; n++; }
        lab[n] = 0;
        p += n;
        if (*p == '|') p++;
        Vector2 m = MeasureTextEx(font_bold, lab, size, size * 0.02f);
        float kw = fmaxf(m.x + 14 * sc, 28 * sc);
        if (draw) {
            Rectangle r = { cx, y, kw, kh };
            if (hot) DrawRectangleRounded((Rectangle){ cx - 3 * sc * pulse, y - 3 * sc * pulse, kw + 6 * sc * pulse, kh + 6 * sc * pulse }, 0.3f, 6,
                                          (Color){ 60, 230, 120, (unsigned char)(40 + 60 * pulse) });
            DrawRectangleRounded((Rectangle){ cx, y + 2 * sc, kw, kh }, 0.3f, 6, (Color){ 0, 0, 0, 140 });
            DrawRectangleRounded(r, 0.3f, 6, hot ? (Color){ 18, 66, 40, 255 } : (Color){ 34, 38, 44, 230 });
            DrawRectangleRoundedLinesEx(r, 0.3f, 6, hot ? 2.0f * sc : 1.0f * sc,
                                        hot ? (Color){ 110, 255, 160, (unsigned char)(170 + 85 * pulse) } : (Color){ 100, 106, 116, 255 });
            hud_text(font_bold, lab, cx + (kw - m.x) / 2, y + (kh - m.y) / 2, size, hot ? (Color){ 235, 255, 240, 255 } : (Color){ 150, 156, 166, 255 });
        }
        cx += kw + 6 * sc;
    }
    return cx > x ? cx - x - 6 * sc : 0;
}

void hints_draw(int w, int h, int dsky_w)
{
    if (!hints_on) return;
    Hint hint;
    compute(&hint);
    if (hint.n == 0 && hint.nstep == 0) return;
    HintTarget ht;
    if (in_cockpit && hints_target(&ht) && hint.n < MAXL)
        add(&hint, VRT("In the cabin: the control to use is ringed in green", "The control to use is ringed in green; the button glows on your controller"));
    int now_step = -1;
    for (int i = 0; i < hint.nstep; i++) if (hint.step[i].state == ST_NOW) now_step = i;

    float scale = h / 1080.0f;
    if (scale < 0.7f) scale = 0.7f;
    float size = 22 * scale, lh = 30 * scale, rh = 34 * scale, ssz = 20 * scale;
    float maxw = 0;
    if (hint.title[0]) maxw = MeasureTextEx(font_bold, hint.title, size, size * 0.02f).x;
    for (int i = 0; i < hint.n; i++) {
        Vector2 m = MeasureTextEx(font_ui, hint.line[i], size, size * 0.02f);
        if (m.x > maxw) maxw = m.x;
    }
    for (int i = 0; i < hint.nstep; i++) {
        float sw = 40 * scale + MeasureTextEx(font_ui, hint.step[i].label, ssz, ssz * 0.02f).x + 24 * scale +
                   draw_caps(hint.step[i].caps, 0, 0, scale, 0, 0);
        if (sw > maxw) maxw = sw;
    }
    int caps_row = hint.caps[0] && now_step < 0;
    if (caps_row) {
        float cw = MeasureTextEx(font_bold, "PRESS", ssz, 0).x + 16 * scale + draw_caps(hint.caps, 0, 0, scale, 1, 0);
        if (cw > maxw) maxw = cw;
    }
    float cx = (w - dsky_w) / 2.0f;
    float bw = maxw + 40 * scale;
    float bh = 40 * scale + (hint.title[0] ? lh : 0) + hint.n * lh + hint.nstep * rh + (caps_row ? rh + 6 * scale : 0);
    float x = cx - bw / 2, y = 150 * scale;
    int flash = hint.urgent && ((int)(GetTime() * 3) & 1);
    Color bg = hint.urgent ? (Color){ 60, 12, 10, 215 } : (Color){ 8, 30, 22, 205 };
    Color edge = hint.urgent ? (Color){ 255, 90, 70, 255 } : (Color){ 90, 220, 150, 255 };
    DrawRectangleRounded((Rectangle){ x, y, bw, bh }, 0.12f, 8, bg);
    DrawRectangleRoundedLinesEx((Rectangle){ x, y, bw, bh }, 0.12f, 8, flash ? 3.0f : 1.5f, edge);
    hud_text(font_bold, "HINT", x + 14 * scale, y + 6 * scale, 15 * scale, edge);
    {   // the headset has no keyboard: hints are switched off from the menu there
        const char *off = VRT("H hides", "off in menu");
        float ow = MeasureTextEx(font_ui, off, 14 * scale, 14 * scale * 0.02f).x;
        hud_text(font_ui, off, x + bw - 14 * scale - ow, y + 6 * scale, 14 * scale, (Color){ 150, 160, 170, 200 });
    }
    float ty = y + 26 * scale;
    if (hint.title[0]) { hud_text(font_bold, hint.title, x + 20 * scale, ty, size, (Color){ 140, 255, 185, 255 }); ty += lh; }
    for (int i = 0; i < hint.n; i++, ty += lh)
        hud_text(font_ui, hint.line[i], x + 20 * scale, ty, size, (Color){ 235, 240, 236, 255 });
    float pulse = 0.5f + 0.5f * sinf((float)GetTime() * 6.0f);
    for (int i = 0; i < hint.nstep; i++, ty += rh) {
        const Step *st = &hint.step[i];
        float mx = x + 26 * scale, my = ty + rh * 0.42f;
        Color lc;
        if (st->state == ST_DONE) {   // a tick
            Color g = { 90, 220, 140, 255 };
            DrawLineEx((Vector2){ mx - 7 * scale, my }, (Vector2){ mx - 2 * scale, my + 6 * scale }, 3 * scale, g);
            DrawLineEx((Vector2){ mx - 2 * scale, my + 6 * scale }, (Vector2){ mx + 8 * scale, my - 7 * scale }, 3 * scale, g);
            lc = (Color){ 120, 170, 140, 255 };
        } else if (st->state == ST_NOW) {   // the one to do now: a pulsing arrow
            float d = 3 * scale * pulse;
            DrawTriangle((Vector2){ mx - 7 * scale + d, my - 8 * scale }, (Vector2){ mx - 7 * scale + d, my + 8 * scale },
                         (Vector2){ mx + 8 * scale + d, my }, (Color){ 110, 255, 160, 255 });
            lc = (Color){ 255, 255, 255, 255 };
        } else {
            DrawRingLines((Vector2){ mx, my }, 5 * scale, 6 * scale, 0, 360, 16, (Color){ 120, 126, 136, 255 });
            lc = (Color){ 150, 156, 166, 255 };
        }
        hud_text(st->state == ST_NOW ? font_bold : font_ui, st->label, x + 46 * scale, ty + 4 * scale, ssz, lc);
        float cw = draw_caps(st->caps, 0, 0, scale, 0, 0);
        if (st->state != ST_DONE) draw_caps(st->caps, x + bw - 20 * scale - cw, ty + 2 * scale, scale, st->state == ST_NOW, 1);
    }
    if (caps_row) {
        ty += 6 * scale;
        hud_text(font_bold, "PRESS", x + 20 * scale, ty + 4 * scale, ssz, (Color){ 140, 255, 185, 255 });
        draw_caps(hint.caps, x + 36 * scale + MeasureTextEx(font_bold, "PRESS", ssz, 0).x, ty, scale, 1, 1);
    }
}
