#include <math.h>
#include <stdio.h>
#include <string.h>

#include "agc_host.h"
#include "crew.h"
#include "lm.h"

Crew crew;

// Keys are queued and released over time so the AGC sees real keystrokes.
static char keybuf[64];
static int keypos, keylen;
static double key_wait;
static int pro_held;
static double last_step_t;
static int v50n18_count, v57_done, stopped, armed_said, ign_said, lr_said, p64_said, p65_said, p66_said,
           flat_said, thr_said, low_said, prev_dps, prog_alarm_said, contact_said;

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
    case '0': agc_host_key(KEY_AGC_0); break;
    default: if (c >= '1' && c <= '9') agc_host_key(c - '0');
    }
}

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
    else if (prog(63) && lm.lr_range_good && !v57_done && lm.dps_on)
        crew.hint = HINT_V57, snprintf(crew.prompt, sizeof crew.prompt, "Landing radar has lock: key V57E to permit LR updates");
    else if (d->prog_lamp)
        crew.hint = HINT_ALARM, snprintf(crew.prompt, sizeof crew.prompt, "PROGRAM ALARM: key V05N09E to see codes, RSET to clear");

    // ---- Callouts (always) -------------------------------------------------------
    if (lm.dps_on && !prev_dps && !ign_said) { crew_say("\"Ignition.\"  10% thrust, ullage settled"); ign_said = 1; }
    prev_dps = lm.dps_on;
    if (lm.dps_on && lm.thrust > 40000 && !flat_said) { crew_say("\"Throttle up.\"  Full thrust"); flat_said = 1; }
    if (flat_said && lm.thrust < 30000 && lm.dps_on && !thr_said) { crew_say("\"Throttle down... better than the simulator.\""); thr_said = 1; }
    if (lm.lr_range_good && lm.dps_on && !lr_said) { crew_say("\"Radar data good.\"  Landing radar lock"); lr_said = 1; }
    if (prog(64) && !p64_said) { crew_say("\"P64.\"  High gate: pitching over, the site is in view"); p64_said = 1; }
    if (prog(65) && !p65_said) { crew_say("\"P65.\"  Automatic velocity nulling"); p65_said = 1; }
    if (prog(66) && !p66_said) { crew_say("\"P66.\"  Rate of descent mode"); p66_said = 1; }
    if (lm.dps_on && lm.altitude < 30 && !low_said) { crew_say("\"Picking up some dust...\""); low_said = 1; }
    if (d->prog_lamp && !prog_alarm_said) { crew_say("\"Program alarm.\""); prog_alarm_said = 1; }
    if (!d->prog_lamp) prog_alarm_said = 0;
    if (lm.probe_contact && !stopped && lm.dps_on && !crew.auto_crew && !contact_said) {
        crew_say("CONTACT LIGHT: press ENGINE STOP (Space)");
        contact_said = 1;
    }

    if (lm.land_result == LAND_CRASH && !crew.landed) {
        crew.landed = 1;
        static const char *why[] = { "", "came down too fast, the legs folded",
                                     "hit the ground moving sideways", "landed too tilted", "tipped over" };
        char m[160];
        snprintf(m, sizeof m, "CRASH: the LM %s", why[lm.crash_cause]);
        crew_say(m);
        return;
    }
    if (!crew.auto_crew) {
        if (lm.touchdown && !crew.landed && !lm.dps_on) {
            crew.landed = 1;
            crew_say("\"Houston, Tranquility Base here. The Eagle has landed.\"");
        }
        return;
    }

    // ---- Automatic crew ------------------------------------------------------------
    // After each action, give Luminary time to respond before acting again.
    static double next_action;
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
    } else {
        acted = 0;
    }
    if (acted) next_action = lm.t + 3.0;
    if (!lm.engine_armed && (vn(6, 62) || vn(99, 62))) {
        lm_set_engine_arm(1);
        if (!armed_said) { crew_say("Descent engine armed"); armed_said = 1; }
    }
    if (!v57_done && prog(63) && lm.lr_range_good && lm.altitude < 12000 && lm.dps_on) {
        crew_type("V57E");
        next_action = lm.t + 3.0;
        crew_say("V57: \"...permit landing radar updates\"");
        v57_done = 1;
    }
    if (!stopped && lm.probe_contact && lm.dps_on) {
        lm_engine_stop();
        stopped = 1;
        crew_say("\"Contact light.\"  \"OK, engine stop.\"");
    }
    if (lm.touchdown && !crew.landed && !lm.dps_on && lm.t - crew.callout_time > 3) {
        crew.landed = 1;
        lm_set_attitude_mode(0);   // PGNS mode control OFF: stop the DAP fighting the ground
        crew_say("\"Houston, Tranquility Base here. The Eagle has landed.\"");
    }
}
