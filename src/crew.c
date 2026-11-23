#include <math.h>
#include <stdio.h>
#include <string.h>

#include "agc_host.h"
#include "agc_mem.h"
#include "crew.h"
#include "lm.h"
#include "moon.h"
#include "scenario.h"

Crew crew;

// Keys are queued and released over time so the AGC sees real keystrokes.
static char keybuf[64];
static int keypos, keylen;
static double key_wait;
static int pro_held;
static double last_step_t;
static int v50n18_count, v57_done, stopped, armed_said, ign_said, lr_said, p64_said, p65_said, p66_said,
           flat_said, thr_said, low_said, prev_dps, prog_alarm_said, contact_said;
static int asc_count_said, asc_lift_said, asc_pitch_said, asc_quiet_said, asc_cut_said, prev_aps;
static double next_action;   // the automatic crew's next allowed action (sim time)
// Program alarms: FAILREG entries seen, Mission Control's GO pending (code, due
// time), 1202s called so far, the automatic crew's V05N09 / RSET sequence.
static int alarm_n, go_code, go_1202_n, alarm_step;
static double go_t;
static int fuel60_said, fuel30_said;   // Mission Control's propellant calls

int crew_typing(void) { return keypos < keylen || pro_held; }

static int actions;   // statistics only (not part of the simulation)
int crew_action_count(void) { return actions; }

void crew_reset(void)
{
    int a = crew.auto_crew, l = crew.auto_land;
    memset(&crew, 0, sizeof crew);
    crew.auto_crew = a;
    crew.auto_land = l;
    keypos = keylen = 0;
    key_wait = 0;
    pro_held = 0;
    last_step_t = 0;
    v50n18_count = v57_done = stopped = armed_said = ign_said = lr_said = p64_said = p65_said = p66_said = 0;
    flat_said = thr_said = low_said = prev_dps = prog_alarm_said = contact_said = 0;
    asc_count_said = asc_lift_said = asc_pitch_said = asc_quiet_said = asc_cut_said = prev_aps = 0;
    alarm_n = go_code = go_1202_n = alarm_step = 0;
    go_t = 0;
    fuel60_said = fuel30_said = 0;
}

// Orbit from the vehicle state: perilune and apolune heights above the
// landing site's radius, nautical miles (as the crew called them).
static void orbit_nmi(double *peri, double *apo)
{
    double rr = v3_len(lm.r), vv = v3_len(lm.v);
    double en = vv * vv / 2 - MOON_MU / rr;
    double a = -MOON_MU / (2 * en);
    V3 h = v3_cross(lm.r, lm.v);
    double e = sqrt(fmax(0, 1 + 2 * en * v3_dot(h, h) / (MOON_MU * MOON_MU)));
    double r0 = scenario_info.site_radius > 1.7e6 ? scenario_info.site_radius : 1738000.0;
    *peri = (a * (1 - e) - r0) / 1852.0;
    *apo = (a * (1 + e) - r0) / 1852.0;
}

void crew_say(const char *text)
{
    snprintf(crew.callout, sizeof crew.callout, "%s", text);
    crew.callout_time = lm.t;
}

void crew_type(const char *keys)
{
    size_t n = strlen(keys);
    if (keylen - keypos + (int)n >= (int)sizeof keybuf) return;
    if (keypos > 0 && keypos < keylen) {   // make room: drop what has been typed
        memmove(keybuf, keybuf + keypos, keylen - keypos);
        keylen -= keypos;
        keypos = 0;
    }
    if (keypos == keylen) keypos = keylen = 0;
    memcpy(keybuf + keylen, keys, n);
    keylen += (int)n;
}

static void key_pump(double dt)
{
    if (key_wait > 0) { key_wait -= dt; return; }
    if (keypos >= keylen) return;
    char c = keybuf[keypos++];
    key_wait = 0.15;
    switch (c) {
    case 'V': agc_host_key(KEY_AGC_VERB); break;
    case 'N': agc_host_key(KEY_AGC_NOUN); break;
    case 'E': agc_host_key(KEY_AGC_ENTR); break;
    case 'R': agc_host_key(KEY_AGC_RSET); break;
    case 'C': agc_host_key(KEY_AGC_CLR); break;
    case 'K': agc_host_key(KEY_AGC_KEYREL); break;
    case '+': agc_host_key(KEY_AGC_PLUS); break;
    case '-': agc_host_key(KEY_AGC_MINUS); break;
    case 'P': agc_host_pro(1); pro_held = 1; key_wait = 0.4; break;
    case 'p': agc_host_pro(0); pro_held = 0; break;
    case ' ': key_wait = 1.0; break;   // give the display time to come up
    case '0': agc_host_key(KEY_AGC_0); break;
    default: if (c >= '1' && c <= '9') agc_host_key(c - '0');
    }
}

// V57 has been keyed (by anyone): Luminary's LRINH flag, FLGWRD11 bit 8, set by
// its LRON routine - "landing radar updates permitted by astronaut".
static int lr_permitted(void) { return agc_get_bit("FLGWRD11", 8); }

static int vn(int v, int n)
{
    const DskyState *d = agc_host_dsky();
    return agc_digit_char(d->verb[0]) == '0' + v / 10 && agc_digit_char(d->verb[1]) == '0' + v % 10 &&
           agc_digit_char(d->noun[0]) == '0' + n / 10 && agc_digit_char(d->noun[1]) == '0' + n % 10;
}

static int prog(int p)
{
    const DskyState *d = agc_host_dsky();
    return agc_digit_char(d->prog[0]) == '0' + p / 10 && agc_digit_char(d->prog[1]) == '0' + p % 10;
}


void crew_step(void)
{
    double dt = last_step_t > 0 ? lm.t - last_step_t : 0.1;
    if (dt < 0 || dt > 1) dt = 0.1;
    last_step_t = lm.t;
    key_pump(dt);

    const DskyState *d = agc_host_dsky();
    int flashing = agc_host_read_channel(011) & 040;

    // ---- What does the computer want? ----------------------------------------
    crew.prompt[0] = 0;
    crew.hint = HINT_NONE;
    if (flashing && prog(63) && vn(50, 25))
        crew.hint = HINT_FINE_ALIGN, snprintf(crew.prompt, sizeof crew.prompt, "V50 N25 R1 00014: fine align option. ENTER to skip, PRO to fine-align");
    else if (flashing && vn(50, 18))
        crew.hint = v50n18_count == 0 ? HINT_MANEUVER : HINT_MANEUVER_DONE, snprintf(crew.prompt, sizeof crew.prompt, v50n18_count == 0 ?
                 "V50 N18: auto maneuver to ignition attitude. Set PGNS AUTO, press PRO" :
                 "V50 N18: maneuver complete. Press ENTER to continue");
    else if (flashing && vn(99, 62))
        crew.hint = HINT_ENABLE_IGN, snprintf(crew.prompt, sizeof crew.prompt, "V99 N62: ENGINE ON ENABLE. Press PRO to permit ignition");
    else if (vn(6, 62) && !lm.engine_armed)
        crew.hint = HINT_ARM, snprintf(crew.prompt, sizeof crew.prompt, "Arm the descent engine (F7) before TIG");
    else if (prog(63) && lm.lr_range_good && !v57_done && !lr_permitted() && lm.dps_on)
        crew.hint = HINT_V57, snprintf(crew.prompt, sizeof crew.prompt, "Landing radar has lock: key V57E to permit LR updates");
    else if (flashing && prog(12) && vn(99, 74) && !lm.abort_stage)
        crew.hint = HINT_ASC_STAGE, snprintf(crew.prompt, sizeof crew.prompt, "V99 N74: lift-off. ABORT STAGE first (separates the stages)");
    else if (flashing && prog(12) && vn(99, 74) && lm.engine_armed != ARM_ASC)
        crew.hint = HINT_ASC_ARM, snprintf(crew.prompt, sizeof crew.prompt, "V99 N74: ENG ARM to ASC (the ascent engine)");
    else if (flashing && prog(12) && vn(99, 74))
        crew.hint = HINT_ASC_PRO, snprintf(crew.prompt, sizeof crew.prompt, "V99 N74: ENGINE ON ENABLE. Press PRO to light the ascent engine");
    else if (prog(12) && vn(6, 74) && !lm.aps_on)
        crew.hint = HINT_ASC_COUNTDOWN, snprintf(crew.prompt, sizeof crew.prompt, "P12 V06 N74: lift-off countdown (R1 min:sec to go). At V99: ABORT STAGE, ENG ARM ASC, PRO");
    else if (d->prog_lamp)
        crew.hint = HINT_ALARM, snprintf(crew.prompt, sizeof crew.prompt, "PROGRAM ALARM: key V05N09E to see codes, RSET to clear");

    // ---- Callouts (always) -------------------------------------------------------
    if (lm.dps_on && !prev_dps && !ign_said) { crew_say("\"Ignition.\"  10% thrust, ullage settled"); ign_said = 1; }
    prev_dps = lm.dps_on;
    if (lm.dps_on && lm.thrust > 40000 && !flat_said) { crew_say("\"Throttle up.\"  Full thrust"); flat_said = 1; }
    if (flat_said && lm.thrust < 30000 && lm.dps_on && !thr_said) { crew_say("\"Throttle down... better than the simulator.\""); thr_said = 1; }
    if (lm.lr_range_good && lm.dps_on && !lr_said) { crew_say("\"Radar data good.\"  Landing radar lock"); lr_said = 1; }
    if (!crew.landed && prog(64) && !p64_said) { crew_say("\"P64.\"  High gate: pitching over, the site is in view"); p64_said = 1; }
    if (!crew.landed && prog(65) && !p65_said) { crew_say("\"P65.\"  Automatic velocity nulling"); p65_said = 1; }
    if (!crew.landed && prog(66) && !p66_said) { crew_say("\"P66.\"  Rate of descent mode"); p66_said = 1; }
    if (lm.dps_on && lm.altitude < 30 && !low_said) { crew_say("\"Picking up some dust...\""); low_said = 1; }
    if (d->prog_lamp && !prog_alarm_said) { crew_say("\"Program alarm.\""); prog_alarm_said = 1; }
    if (!d->prog_lamp) prog_alarm_said = 0;

    // Mission Control on program alarms during the descent: a new entry in the
    // FAILREGs (first, second, latest; RSET clears them), and the guidance
    // officers' GO about 20 s later, as on Apollo 11.
    {
        int f0 = agc_get_word("FAILREG", 0) & 037777, f1 = agc_get_word("FAILREG", 1) & 037777;
        int f2raw = agc_get_word("FAILREG", 2), f2 = f2raw & 037777;
        int n = (f0 != 0) + (f1 != 0) + (f2 != 0) + ((f2raw & 040000) != 0);
        int code = f2 ? f2 : f1 ? f1 : f0;
        if (n > alarm_n && code && lm.dps_on && !go_code) { go_code = code; go_t = lm.t + 20; }
        alarm_n = n;
        if (go_code && lm.t >= go_t) {
            if (go_code == 01202)
                crew_say(go_1202_n++ == 0 ? "Houston: \"Roger. We got... We're GO on that alarm.\"" : "Houston: \"We're GO on that alarm.\"");
            else if (go_code == 01201)
                crew_say("Houston: \"Roger. 1201 alarm. We're GO. Same type. We're GO.\"");
            go_code = 0;
        }
    }
    // Mission Control's propellant calls: seconds to bingo (20 s of hovering
    // left: land within them or abort).
    if (lm.dps_on && !lm.staged && !lm.touchdown) {
        double hover = lm_mass() * 1.62 / (305.0 * 9.80665);   // kg/s at hover thrust
        double to_bingo = lm.m_dps / hover - 20;
        if (to_bingo < 60 && !fuel60_said) { crew_say("Houston: \"60 seconds.\""); fuel60_said = 1; }
        if (to_bingo < 30 && !fuel30_said) { crew_say("Houston: \"30 seconds.\""); fuel30_said = 1; }
    }
    if (lm.probe_contact && !stopped && lm.dps_on && !crew.auto_crew && !contact_said) {
        crew_say("CONTACT LIGHT: press ENGINE STOP (Space)");
        contact_said = 1;
    }

    // ---- Ascent callouts ----------------------------------------------------------------
    if (prog(12) && vn(99, 74) && !asc_count_said) { crew_say("\"...5, 4, 3, 2, 1. Abort Stage, Engine Arm Ascent, Proceed.\""); asc_count_said = 1; }
    if (lm.liftoff && crew.landed && !asc_lift_said && lm.t - lm.liftoff_t > 1.5) { crew_say("\"Beautiful. 26, 36 feet per second up.\""); asc_lift_said = 1; }
    if (asc_lift_said && !asc_pitch_said && lm_tilt() > 0.35) { crew_say("\"Be advised of the pitchover.\""); asc_pitch_said = 1; }
    if (asc_pitch_said && !asc_quiet_said && lm.t - lm.liftoff_t > 60) {
        crew_say("\"Very smooth... very quiet ride. Just a little slow wallowing back and forth.\"");
        asc_quiet_said = 1;
    }
    if (lm.staged && !lm.touchdown && !asc_cut_said && !asc_lift_said && lm.aps_on && !prev_aps) {
        crew_say("\"Abort stage.\"  P71: the ascent engine takes Eagle back to orbit");
        asc_lift_said = asc_pitch_said = asc_quiet_said = 1;
    }
    if (prev_aps && !lm.aps_on && lm.liftoff && !asc_cut_said && lm.altitude > 5000) {
        double pe, ap;
        orbit_nmi(&pe, &ap);
        char m[160];
        snprintf(m, sizeof m, crew.landed ? "\"Shutdown.\"  Orbit %.1f by %.1f nautical miles (Eagle had 9.4 by 46.7)"
                                          : "\"Shutdown.\"  Abort complete: orbit %.1f by %.1f nautical miles", pe, ap);
        crew_say(m);
        asc_cut_said = 1;
        crew.in_orbit = 1;
    }
    prev_aps = lm.aps_on;

    if (lm.land_result == LAND_CRASH && !crew.landed) {
        crew.landed = 1;
        static const char *why[] = { "", "came down too fast, the legs folded",
                                     "hit the ground moving sideways", "landed too tilted", "tipped over",
                                     "came down on a boulder: the descent stage is holed" };
        char m[160];
        snprintf(m, sizeof m, "CRASH: the LM %s", why[lm.crash_cause]);
        crew_say(m);
        return;
    }
    if (!crew.auto_crew) {
        if (lm.touchdown && !crew.landed && !lm.dps_on && !lm.staged) {
            crew.landed = 1;
            crew_say("\"Houston, Tranquility Base here. The Eagle has landed.\"");
        }
        return;
    }

    // ---- Automatic crew ------------------------------------------------------------
    // After each action, give Luminary time to respond before acting again.
    if (lm.t < next_action - 100) next_action = 0;   // new run
    if (keypos < keylen || pro_held || lm.t < next_action) return;
    int acted = 1;
    if (flashing && prog(63) && vn(50, 25)) {
        crew_type("E");
    } else if (flashing && vn(50, 18)) {
        if (v50n18_count++ == 0) {
            lm_set_attitude_mode(2);
            crew_type("Pp");
        } else {
            crew_type("E");
        }
    } else if (flashing && vn(99, 62)) {
        crew_type("Pp");
    } else if (flashing && prog(12) && vn(99, 74) && !crew.outside) {
        // Aldrin's call: "Abort Stage, Engine Arm Ascent, Proceed."
        lm_abort_stage();
        lm_set_engine_arm(ARM_ASC);
        lm_set_attitude_mode(2);
        crew_type("Pp");
    } else if (d->prog_lamp && alarm_step == 0) {
        // Aldrin on a program alarm: the codes on the DSKY ...
        crew_type("V05N09E");
        alarm_step = 1;
    } else if (alarm_step == 1 && !go_code) {
        // ... and once Houston has called it, reset the light and give the
        // display back to the program.
        crew_type(d->key_rel ? "RK" : "R");
        alarm_step = 0;
    } else {
        acted = 0;
    }
    if (acted) next_action = lm.t + 3.0, actions++;
    if (!lm.engine_armed && (vn(6, 62) || vn(99, 62))) {
        lm_set_engine_arm(1);
        actions++;
        if (!armed_said) { crew_say("Descent engine armed"); armed_said = 1; }
    }
    if (!v57_done && prog(63) && lm.lr_range_good && lm.altitude < 12000 && lm.dps_on) {
        crew_type("V57E");
        next_action = lm.t + 3.0;
        actions++;
        crew_say("V57: \"...permit landing radar updates\"");
        v57_done = 1;
    }
    if (!stopped && lm.probe_contact && lm.dps_on) {
        lm_engine_stop();
        actions++;
        stopped = 1;
        crew_say("\"Contact light.\"  \"OK, engine stop.\"");
    }
    if (prog(12) && vn(6, 74) && lm_get_attitude_mode() != 2) lm_set_attitude_mode(2);   // PGNS mode control AUTO for the ascent
    if (lm.touchdown && !crew.landed && !lm.dps_on && !lm.staged && lm.t - crew.callout_time > 3) {
        crew.landed = 1;
        lm_set_attitude_mode(0);   // PGNS mode control OFF: stop the DAP fighting the ground
        crew_say("\"Houston, Tranquility Base here. The Eagle has landed.\"");
    }
}

// --- Save states ---------------------------------------------------------------------

#include "snapshot.h"

void crew_snap(Snap *s)
{
    snap_int(s, &crew.auto_crew); snap_int(s, &crew.auto_land);
    snap_chars(s, crew.prompt, sizeof crew.prompt); snap_chars(s, crew.callout, sizeof crew.callout);
    snap_f64(s, &crew.callout_time); snap_int(s, &crew.landed); snap_int(s, &crew.hint);
    snap_int(s, &crew.in_orbit); snap_int(s, &crew.outside);
    snap_chars(s, keybuf, sizeof keybuf); snap_int(s, &keypos); snap_int(s, &keylen);
    snap_f64(s, &key_wait); snap_int(s, &pro_held); snap_f64(s, &last_step_t); snap_f64(s, &next_action);
    snap_int(s, &alarm_n); snap_int(s, &go_code); snap_int(s, &go_1202_n); snap_int(s, &alarm_step); snap_f64(s, &go_t);
    snap_int(s, &fuel60_said); snap_int(s, &fuel30_said);
    int *flags[] = { &v50n18_count, &v57_done, &stopped, &armed_said, &ign_said, &lr_said, &p64_said, &p65_said, &p66_said,
                     &flat_said, &thr_said, &low_said, &prev_dps, &prog_alarm_said, &contact_said,
                     &asc_count_said, &asc_lift_said, &asc_pitch_said, &asc_quiet_said, &asc_cut_said, &prev_aps };
    for (size_t i = 0; i < sizeof flags / sizeof flags[0]; i++) snap_int(s, flags[i]);
}
