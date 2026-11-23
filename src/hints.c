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

#define FT 0.3048
#define MAXL 3

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

typedef struct { char line[MAXL][160]; int n; int urgent; int vrbtn; } Hint;

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
            down / FT, (int)lround(fmin(15, diff_ft))), h->vrbtn = VB_LSTICK_UP;
    else if (down < target * 0.4 - 0.15)
        add(h, lm.hdot > 0.2 ? VRT("You're climbing: tap Down about %d times to descend", "You're climbing: flick the LEFT stick down about %d times")
                             : VRT("Barely descending: tap Down about %d times", "Barely descending: flick the LEFT stick down about %d times"),
            (int)lround(fmin(15, fmax(1, -diff_ft)))), h->vrbtn = VB_LSTICK_DOWN;

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
                drift_r > 0 ? "right" : "left", fabs(drift_r), VRT(drift_r > 0 ? "Right" : "Left", drift_r > 0 ? "right" : "left"));
        else
            add(h, VRT("Columbia is drifting %s (%.2f m/s): tap the %s arrow to stop it", "Columbia is drifting %s (%.2f m/s): nudge the LEFT stick %s to stop it"),
                drift_u > 0 ? "up" : "down", fabs(drift_u), VRT(drift_u > 0 ? "Up" : "Down", drift_u > 0 ? "up" : "down"));
        h->vrbtn = VB_LSTICK;
        return;
    }
    if (fabs(g.roll) > 4 * DEG2RAD) {
        add(h, VRT("Roll %.0f deg off: hold %s to turn until the target's cross is square in the sight",
                   "Roll %.0f deg off: hold the RIGHT grip and push the RIGHT stick %s until the cross is square"),
            fabs(g.roll) * RAD2DEG, VRT(g.roll > 0 ? "Q" : "E", g.roll > 0 ? "left" : "right"));
        h->vrbtn = VB_RGRIP;
        return;
    }
    if (g.angle > 3 * DEG2RAD) {
        add(h, VRT("Axes %.1f deg apart: tilt with W/S and A/D until the standoff cross sits in the middle of its disc",
                   "Axes %.1f deg apart: tilt with the RIGHT stick until the standoff cross sits in the middle of its disc"),
            g.angle * RAD2DEG);
        h->vrbtn = VB_RSTICK;
        return;
    }
    if (fabs(off_r) > 0.12 || fabs(off_u) > 0.12) {
        if (fabs(off_r) >= fabs(off_u))
            add(h, VRT("The probe is %.1f m %s of the drogue: tap the %s arrow, then stop the drift", "The probe is %.1f m %s of the drogue: nudge the LEFT stick %s, then stop the drift"),
                fabs(off_r), off_r > 0 ? "right" : "left", VRT(off_r > 0 ? "Right" : "Left", off_r > 0 ? "right" : "left"));
        else
            add(h, VRT("The probe is %.1f m %s of the drogue: tap the %s arrow, then stop the drift", "The probe is %.1f m %s of the drogue: nudge the LEFT stick %s, then stop the drift"),
                fabs(off_u), off_u > 0 ? "above" : "below", VRT(off_u > 0 ? "Up" : "Down", off_u > 0 ? "up" : "down"));
        h->vrbtn = VB_LSTICK;
        return;
    }
    if (g.axial > 2 && closing > 0.35) {
        add(h, VRT("Closing at %.2f m/s: too fast. Tap G to slow to 0.1-0.3", "Closing at %.2f m/s: too fast. Tap the LEFT trigger to slow to 0.1-0.3"), closing);
        h->urgent = g.axial < 8;
        h->vrbtn = VB_LTRIG;
        return;
    }
    if (g.axial > 1 && closing < 0.04) {
        add(h, VRT("Lined up. Tap T to close in at about 0.1-0.3 m/s (RATE about -0.2)", "Lined up. Tap the LEFT grip to close in at about 0.1-0.3 m/s (RATE about -0.2)"));
        h->vrbtn = VB_LGRIP;
        return;
    }
    add(h, "On the way in at %.2f m/s, %.0f m to go: keep the target centred", closing, fmax(0, g.axial));
}

static void ascent_hints(Hint *h)
{
    int ph = ascent_phase();
    if (ph == MP_COUNTDOWN) {
        if (eva_phase() == EVA_SURFACE) {
            add(h, VRT("Walk with W A S D (Shift to lope), Space to jump, F to plant the flag.",
                       "Walk with the LEFT stick (left grip lopes), A jumps. Squeeze the RIGHT grip to take the flag, let go to plant it."));
            add(h, "The countdown waits for you: walk to the ladder and press X to climb back in");
            return;
        }
        if (crew.auto_crew) { add(h, VRT("The auto crew will light the ascent engine at V99 (F2 to do it yourself)", "The auto crew will light the ascent engine at V99")); return; }
        switch (crew.hint) {
        case HINT_ASC_STAGE:
            add(h, VRT("Lift-off! First press B (ABORT STAGE): it fires the bolts holding",
                       "Lift-off! First point at ABORT STAGE (ringed green) and pull the trigger: it fires"));
            add(h, VRT("the ascent stage to the descent stage, which stays on the Moon", "the bolts holding the stages together; the descent stage stays behind"));
            h->urgent = 1;
            h->vrbtn = VB_RTRIG;
            return;
        case HINT_ASC_ARM:
            add(h, VRT("Now press F7 to arm the ascent engine (ENG ARM: ASC)", "Now point at ENG ARM (ringed green) and pull the trigger: ASC"));
            h->urgent = 1;
            h->vrbtn = VB_RTRIG;
            return;
        case HINT_ASC_PRO:
            add(h, VRT("Press P (PRO): the computer lights the ascent engine", "Press B (PRO): the computer lights the ascent engine"));
            h->urgent = 1;
            h->vrbtn = VB_B;
            return;
        default:
            add(h, "Countdown to lift-off: the DSKY's top number is minutes and seconds to go.");
            add(h, VRT("When it flashes V99: B (ABORT STAGE), F7 (arm), then P (PRO). X: a moonwalk first",
                       "At V99: ABORT STAGE, ENG ARM to ASC, then B (PRO). X for a moonwalk first"));
            if (eva_available()) h->vrbtn = VB_X;
            return;
        }
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
        h->vrbtn = VB_B;
        return;
    }
    if (ph == MP_PROX) {
        docking_hints(h);
        if (h->n && hud_camera_is_docking == 0)
            add(h, VRT("Tab to the DOCKING WINDOW view: the target sits in the COAS sight", "Press Y for the DOCKING WINDOW view: the target sits in the COAS sight"));
    }
}

static void compute(Hint *h)
{
    memset(h, 0, sizeof *h);
    if (ascent_active()) { ascent_hints(h); return; }
    if (lm.land_result == LAND_CRASH) return;   // the results panel takes over
    if (note[0] && !lm.touchdown) add(h, "%s", note);
    if (lm.touchdown) {
        if (lm.dps_on) { add(h, VRT("You're on the ground: press SPACE to shut the engine off", "You're on the ground: press A (ENGINE STOP)")); h->urgent = 1; h->vrbtn = VB_A; }
        else if (eva_phase() == EVA_INSIDE && eva_available()) {
            add(h, VRT("You are on the Moon. Press X to open the hatch and go for a walk,", "You are on the Moon. Grab the hatch handle by your knees, or press X, to go outside"));
#ifndef AGC_VR
            add(h, "or L to skip ahead to lift-off (P12) and the trip back to Columbia");
#endif
            h->vrbtn = VB_X;
        }
        else if (eva_phase() == EVA_SURFACE) {
            add(h, VRT("Walk with W A S D (Shift to lope), Space to jump, F to plant the flag.",
                       "Walk with the LEFT stick (left grip lopes), A jumps. Squeeze the RIGHT grip to take the flag, let go to plant it."));
            add(h, VRT("To go back in, walk to the foot of the ladder and press X", "To go back in, grab the ladder (grip) or press X at its foot"));
        }
        return;
    }
    if (lm.probe_contact && lm.dps_on) {
        add(h, VRT("Blue CONTACT light! Press SPACE now to cut the engine", "Blue CONTACT light! Press A now to cut the engine"));
        h->urgent = 1;
        h->vrbtn = VB_A;
        return;
    }

    // The computer is asking for something.
    if (!crew.auto_crew) {
        switch (crew.hint) {
        case HINT_FINE_ALIGN:
            add(h, "The computer asks if it should fine-tune its gyros. Not needed:");
            add(h, VRT("press Enter to skip", "pull the LEFT trigger (ENTR) to skip"));
            h->vrbtn = VB_LTRIG;
            return;
        case HINT_MANEUVER:
            add(h, "The computer wants to turn the LM to the burn attitude.");
            if (lm_get_attitude_mode() != 2) {
                add(h, VRT("Make sure F8 says AUTO, then press P (PRO) to let it turn",
                           "Point at the PGNS mode switch (ringed green), pull the trigger for AUTO, then press B (PRO)"));
                h->vrbtn = VB_RTRIG;
            } else {
                add(h, VRT("Make sure F8 says AUTO, then press P (PRO) to let it turn", "Press B (PRO) to let it turn"));
                h->vrbtn = VB_B;
            }
            return;
        case HINT_MANEUVER_DONE:
            add(h, VRT("Turn finished. Press Enter to carry on", "Turn finished. Pull the LEFT trigger (ENTR) to carry on"));
            h->vrbtn = VB_LTRIG;
            return;
        case HINT_ARM:
            add(h, VRT("Press F7 to arm the descent engine before the burn starts",
                       "Arm the descent engine: point at ENG ARM (ringed green) and pull the trigger"));
            h->vrbtn = VB_RTRIG;
            return;
        case HINT_ENABLE_IGN:
            if (!lm.engine_armed) {
                add(h, VRT("The engine is ready to light. Press P (PRO) to allow ignition", "The engine is ready to light, but not armed:"));
                add(h, VRT("...but arm it first with F7", "point at ENG ARM (ringed green) and pull the trigger first"));
                h->vrbtn = VB_RTRIG;
            } else {
                add(h, VRT("The engine is ready to light. Press P (PRO) to allow ignition", "The engine is ready to light. Press B (PRO) to allow ignition"));
                h->vrbtn = VB_B;
            }
            return;
        case HINT_V57:
            add(h, "The landing radar can see the ground. Tell the computer to use it:");
            add(h, VRT("type V 5 7 then Enter", "on the cabin DSKY, point and pull the trigger on VERB, 5, 7, then ENTR (ringed green)"));
            h->vrbtn = VB_RTRIG;
            return;
        case HINT_ALARM:
            add(h, "Program alarm: the computer is overloaded (1202, 1201) but still flying the descent.");
            add(h, VRT("V05N09E shows the code. R (RSET) clears the light; Houston will call GO",
                       "Point at RSET on the cabin DSKY (ringed green) and pull the trigger; Houston will call GO"));
            h->vrbtn = VB_RTRIG;
            return;
        }
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
        return;
    }
    if (p64) {
        if (must_take_over) {
            add(h, VRT("Take over now: press F8 (ATT HOLD), then tap Up/Down",
                       "Take over now: set the PGNS mode switch to ATT HOLD, then flick the LEFT stick up/down"));
        } else {
            add(h, "The computer is steering to the landing site. Let it fly,");
            add(h, VRT("or take over: press F8 (ATT HOLD), then tap Up/Down",
                       "or take over: set the PGNS mode switch to ATT HOLD, then flick the LEFT stick up/down"));
        }
        return;
    }
    if (p63 && lm.dps_on) {
        add(h, "Braking burn: the computer is slowing you down from orbit.");
        if (next_event[0]) add(h, VRT("Nothing to do yet. J skips ahead: %s", "Nothing to do yet. NEXT EVENT skips ahead: %s"), next_event);
        else add(h, "Nothing to do yet: watch the Moon come up in the window");
        return;
    }
    if (p63) {
        add(h, crew.auto_crew ? VRT("Waiting for the burn. The auto crew runs the checklist (F2 to do it yourself).",
                                    "Waiting for the burn. The auto crew runs the checklist.")
                              : VRT("Waiting for the computer. Press F2 if you want the auto crew to do the checklist.",
                                    "Waiting for the computer: it will ask for each step on the DSKY."));
        if (next_event[0]) add(h, VRT("J skips ahead: %s", "NEXT EVENT skips ahead: %s"), next_event);
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

    // Hand-flying: the rate-of-descent switch, when the sink rate is off.
    if ((prog_is(66) || (lm.dps_on && lm.altitude < 150 && lm_get_attitude_mode() != 2)) && lm.altitude >= 4) {
        double tgt = fmax(0.6, fmin(4.5, lm.altitude / 12.0)), down = -lm.hdot;
        if (down > tgt + 0.8) return target(t, HT_ROD_UP, "CLICK UP: SLOWER");
        if (down < tgt * 0.4 - 0.15) return target(t, HT_ROD_DOWN, "CLICK DOWN: FASTER");
    }
    return 0;
}

static int in_cockpit;
void hints_set_cockpit(int c) { in_cockpit = c; }

int hints_vr_button(void)
{
    if (!hints_on) return VB_NONE;
    Hint h;
    compute(&h);
    return h.n ? h.vrbtn : VB_NONE;
}

void hints_draw(int w, int h, int dsky_w)
{
    if (!hints_on) return;
    Hint hint;
    compute(&hint);
    if (hint.n == 0) return;
    HintTarget ht;
    if (in_cockpit && hints_target(&ht) && hint.n < MAXL)
        add(&hint, VRT("In the cabin: the control to use is ringed in green", "The control to use is ringed in green; the button glows on your controller"));

    float scale = h / 1080.0f;
    if (scale < 0.7f) scale = 0.7f;
    float size = 22 * scale, lh = 30 * scale;
    float maxw = 0;
    for (int i = 0; i < hint.n; i++) {
        Vector2 m = MeasureTextEx(font_ui, hint.line[i], size, size * 0.02f);
        if (m.x > maxw) maxw = m.x;
    }
    float cx = (w - dsky_w) / 2.0f;
    float bw = maxw + 40 * scale, bh = hint.n * lh + 40 * scale;
    float x = cx - bw / 2, y = 150 * scale;
    int flash = hint.urgent && ((int)(GetTime() * 3) & 1);
    Color bg = hint.urgent ? (Color){ 60, 12, 10, 215 } : (Color){ 8, 30, 22, 205 };
    Color edge = hint.urgent ? (Color){ 255, 90, 70, 255 } : (Color){ 90, 220, 150, 255 };
    DrawRectangleRounded((Rectangle){ x, y, bw, bh }, 0.12f, 8, bg);
    DrawRectangleRoundedLinesEx((Rectangle){ x, y, bw, bh }, 0.12f, 8, flash ? 3.0f : 1.5f, edge);
    hud_text(font_bold, "HINT", x + 14 * scale, y + 6 * scale, 15 * scale, edge);
    hud_text(font_ui, "H hides", x + bw - 62 * scale, y + 6 * scale, 14 * scale, (Color){ 150, 160, 170, 200 });
    for (int i = 0; i < hint.n; i++)
        hud_text(font_ui, hint.line[i], x + 20 * scale, y + 26 * scale + i * lh, size, (Color){ 235, 240, 236, 255 });
}
