// Lift-off, orbit, rendezvous shortcut and docking with Columbia.
//
// Docking is probe-and-drogue: the CSM's probe (on the CM apex) must enter
// the drogue cone in the LM's top tunnel. Capture needs the probe tip within
// the drogue mouth (0.33 m of the axis) as it crosses it, the two docking
// axes within 10 deg, and a closing speed under 0.6 m/s (0.03-0.3 is the
// book). Then the capture latches hold, the probe retracts and pulls the two
// tunnel rings together, and the twelve ring latches fire: hard dock.

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "agc_host.h"
#include "agc_mem.h"
#include "ascent.h"
#include "crew.h"
#include "csm.h"
#include "lm.h"
#include "moon.h"
#include "scenario.h"

#define DEG (PI_D / 180.0)

// LM body axes (model origin): the drogue mouth at the top of the docking tunnel.
static const V3 MOUTH = { 3.86, 0.0, -0.30 };
#define DROGUE_R 0.33       // the cone funnels the probe in from this far off axis
#define TUNNEL_R 0.55       // probe hits the tunnel rim beyond the drogue
#define CAPTURE_ANGLE (10.0 * DEG)
#define CAPTURE_MAX_V 0.60
#define DAMAGE_V 0.45       // body contact this fast breaks something
#define RETRACT_TIME 8.0

static int active, phase_override, hidden, mirrored = -1;
static int dap_step;      // docking DAP set-up: 0 idle, 1 P00 asked, 2 V48 asked, 3 N46 proceeded, 4 done
static double dap_wait;
static struct {
    int state;          // 0 approaching, 1 captured (retracting), 2 hard docked, 3 failed
    double t_capture;   // lm.t
    double t_start;     // lm.t at the start of the approach
    double prev_axial;
    double cooldown;    // after a bounce
    int bounces;
    char msg[160];
    double msg_t;
    // At capture: LM pose relative to the CSM, and the pose it is pulled to.
    V3 p_rel0, p_rel1;
    Q4 q_rel;
    double c_speed, c_radial, c_angle, c_roll, c_rcs;
    double rcs_start;
    char fail[160];
} dk;
static struct { double t_lift, t_cut, peri, apo, aps_left, rcs_left; int have; } ins;

void ascent_reset(void)
{
    dap_step = 0;
    active = 0;
    mirrored = -1;
    phase_override = 0;
    hidden = 0;
    memset(&dk, 0, sizeof dk);
    memset(&ins, 0, sizeof ins);
}

void ascent_begin(void)
{
    ascent_reset();
    active = 1;
}

int ascent_active(void) { return active; }

void ascent_mirror(int act, int phase)
{
    active = act;
    mirrored = act ? phase : -1;
}

int ascent_phase(void)
{
    if (mirrored >= 0 && active) return mirrored;
    if (!active) return lm.touchdown && lm.land_result != LAND_CRASH ? MP_SURFACE : MP_DESCENT;
    if (dk.state == 3) return MP_DOCK_FAILED;
    if (dk.state == 2) return MP_DOCKED;
    if (phase_override) return MP_PROX;
    if (crew.in_orbit) return MP_ORBIT;
    if (lm.liftoff || lm.aps_on) return MP_ASCENT;
    return MP_COUNTDOWN;
}

static void say(const char *m)
{
    snprintf(dk.msg, sizeof dk.msg, "%s", m);
    dk.msg_t = lm.t;
    crew_say(m);
}

// Orbit heights above the landing site's radius, km.
static void orbit_km(double *peri, double *apo)
{
    double rr = v3_len(lm.r), vv = v3_len(lm.v);
    double en = vv * vv / 2 - MOON_MU / rr;
    double a = -MOON_MU / (2 * en);
    V3 h = v3_cross(lm.r, lm.v);
    double e = sqrt(fmax(0, 1 + 2 * en * v3_dot(h, h) / (MOON_MU * MOON_MU)));
    double r0 = scenario_info.site_radius > 1.7e6 ? scenario_info.site_radius : 1738000.0;
    *peri = (a * (1 - e) - r0) / 1000;
    *apo = (a * (1 + e) - r0) / 1000;
}

// Columns of a rotation from three body axes given in inertial coordinates.
static Q4 q_from_axes(V3 x, V3 y, V3 z)
{
    M3 m;
    m.m[0][0] = x.x; m.m[0][1] = y.x; m.m[0][2] = z.x;
    m.m[1][0] = x.y; m.m[1][1] = y.y; m.m[1][2] = z.y;
    m.m[2][0] = x.z; m.m[2][1] = y.z; m.m[2][2] = z.z;
    return q4_normalize(m3_to_q4(m));
}

// Put the LM at `dist` metres (probe tip to drogue mouth) from Columbia, facing it.
static void place_near_csm(double dist, double off_y, double off_z, double roll, double tilt)
{
    // Eagle sits beside Columbia's orbit plane with the same velocity: the
    // line between them then stays fixed in inertial space (only the slow
    // out-of-plane pull, d n^2 t, closes it), so an inertially held LM sees
    // Columbia stay put instead of wandering at orbital rate.
    V3 rc = csm.r, vc = csm.v;
    double back = dist + MOUTH.x + CSM_PROBE_TIP;
    V3 nrm = v3_norm(v3_cross(rc, vc));
    V3 rl = v3_sub(rc, v3_scale(nrm, back)), vl = vc;

    // LM +X toward Columbia, +Z (the front windows) toward the Moon.
    V3 X = v3_norm(v3_sub(rc, rl));
    V3 down = v3_scale(v3_norm(rl), -1);
    V3 Z = v3_norm(v3_sub(down, v3_scale(X, v3_dot(down, X))));
    V3 Y = v3_cross(Z, X);
    Q4 q_new = q_from_axes(X, Y, Z);
    // Turn the stable member with the vehicle: the gimbal angles (and so the
    // CDUs the LGC reads) stay exactly as they were.
    Q4 dq = q4_mul(q_new, q4_conj(lm.q));
    lm.q_sm = q4_normalize(q4_mul(dq, lm.q_sm));
    lm.q = q_new;
    lm.w = v3(0, 0, 0);
    // The drogue axis passes through the mouth (z -0.30): shift the LM so
    // Columbia's axis sits off it by (off_y, off_z).
    lm.r = v3_sub(rl, v3_add(v3_add(v3_scale(Y, off_y), v3_scale(Z, off_z + MOUTH.z)), v3(0, 0, 0)));
    lm.v = vl;

    // Columbia: probe toward Eagle, its +Z on the LM's +Z side (so the docking
    // target faces the LM's overhead window), then the misalignment.
    V3 cx = v3_scale(X, -1), cy = v3_scale(Y, -1), cz = Z;
    Q4 qc = q_from_axes(cx, cy, cz);
    qc = q4_normalize(q4_mul(qc, q4_axis_angle(v3(1, 0, 0), roll)));
    qc = q4_normalize(q4_mul(qc, q4_axis_angle(v3(0, 1, 0), tilt)));
    csm.q = qc;
    csm.w = v3(0, 0, 0);
    csm.hold = 1;
    csm.docked = 0;
}

// The crew's docking set-up of the LM autopilot: P00, then V48 with the DAP
// data changed to 0.3 deg deadband (DBSELECT), 2-jet X translation (ACC4OR2X)
// and fine attitude-controller scaling (RHCSCALE), proceeded through N46 and
// N47 so Luminary itself applies them (RESTORDB recomputes the switch curves).
static void dap_for_docking(void)
{
    agc_set_bit("DAPBOOLS", 4, 0);
    agc_set_bit("DAPBOOLS", 11, 0);
    agc_set_bit("DAPBOOLS", 7, 0);
    crew_type("V37E 00E");
    dap_step = 1;
    dap_wait = lm.t;
}

static int dsky_is(char v0, char v1, char n0, char n1)
{
    const DskyState *d = agc_host_dsky();
    return agc_digit_char(d->verb[0]) == v0 && agc_digit_char(d->verb[1]) == v1 &&
           agc_digit_char(d->noun[0]) == n0 && agc_digit_char(d->noun[1]) == n1;
}

// Follow the DSKY through the V48 sequence (each step waits for Luminary).
static void dap_sequence(void)
{
    if (dap_step == 0 || dap_step == 4 || lm.t < dap_wait) return;
    const DskyState *d = agc_host_dsky();
    int p00 = agc_digit_char(d->prog[0]) == '0' && agc_digit_char(d->prog[1]) == '0';
    if (dap_step == 1 && p00) { crew_type("V48E"); dap_step = 2; dap_wait = lm.t + 1.0; }
    else if (dap_step == 2 && dsky_is('0', '1', '4', '6')) { crew_type("Pp"); dap_step = 3; dap_wait = lm.t + 1.0; }
    else if (dap_step == 3 && dsky_is('0', '6', '4', '7')) { crew_type("Pp"); dap_step = 4; }
    else if (lm.t - dap_wait > 40) dap_step = 4;   // give up quietly: the DAP still works, just wider
}

void ascent_rendezvous(void)
{
    if (!csm.valid) return;
    place_near_csm(40.0, 1.1, -0.8, 10.0 * DEG, 2.0 * DEG);
    // The rendezvous burns are done; engine safed, 2-jet X translation for
    // a gentle docking, attitude hold for hand control, the LGC in P00.
    lm_set_engine_arm(ARM_OFF);
    lm.aps_on = 0;
    lm.thrust = 0;
    dap_for_docking();
    lm_set_attitude_mode(1);
    phase_override = 1;
    hidden = 0;
    memset(&dk, 0, sizeof dk);
    dk.t_start = lm.t;
    dk.prev_axial = 1e9;
    dk.rcs_start = lm.m_rcs;
    say("Rendezvous complete: \"Columbia, Eagle. Station keeping, 130 feet.\"  Now dock: close in along +X");
}

void ascent_debug_dock(double dist)
{
    if (!active) ascent_begin();
    if (!csm.valid) return;
    if (!lm.staged) { lm_abort_stage(); lm.ds_landed = 1; }
    lm.liftoff = 1;
    crew.in_orbit = 1;
    place_near_csm(dist, 0.0, 0.0, 0.0, 0.0);
    lm_set_engine_arm(ARM_OFF);
    lm.aps_on = 0;
    lm.thrust = 0;
    dap_for_docking();
    lm_set_attitude_mode(1);
    phase_override = 1;
    memset(&dk, 0, sizeof dk);
    dk.t_start = lm.t;
    dk.prev_axial = 1e9;
    dk.rcs_start = lm.m_rcs;
}

static Q4 slerp(Q4 a, Q4 b, double t)
{
    double d = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    if (d < 0) { b.w = -b.w; b.x = -b.x; b.y = -b.y; b.z = -b.z; d = -d; }
    if (d > 0.9995) {
        Q4 r = { a.w + (b.w - a.w) * t, a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t };
        return q4_normalize(r);
    }
    double th = acos(d), sa = sin((1 - t) * th) / sin(th), sb = sin(t * th) / sin(th);
    Q4 r = { a.w * sa + b.w * sb, a.x * sa + b.x * sb, a.y * sa + b.y * sb, a.z * sa + b.z * sb };
    return r;
}

static M3 lm_rot(void) { return q4_to_m3(lm.q); }
static M3 csm_rot(void) { return q4_to_m3(csm.q); }

// A CSM-frame point in LM body axes, and back.
static V3 csm_to_lm(V3 pc)
{
    V3 pi = v3_add(csm.r, m3_apply(csm_rot(), pc));
    return m3_apply(m3_transpose(lm_rot()), v3_sub(pi, lm.r));
}

static V3 lm_to_csm(V3 pl)
{
    V3 pi = v3_add(lm.r, m3_apply(lm_rot(), pl));
    return m3_apply(m3_transpose(csm_rot()), v3_sub(pi, csm.r));
}

int ascent_dock_geo(DockGeo *g)
{
    memset(g, 0, sizeof *g);
    if (!csm.valid) return 0;
    V3 tip = csm_to_lm(v3(CSM_PROBE_TIP, 0, 0));
    V3 d = v3_sub(tip, MOUTH);
    g->range = v3_len(d);
    g->axial = d.x;
    g->lat_y = d.y;
    g->lat_z = d.z;
    // Velocity of the probe tip relative to the LM, in LM axes (both vehicles
    // hold attitude while docking, so the rotation terms are left out).
    V3 vrel = m3_apply(m3_transpose(lm_rot()), v3_sub(csm.v, lm.v));
    g->rate = vrel.x;
    g->vy = vrel.y;
    g->vz = vrel.z;
    V3 lx = m3_apply(lm_rot(), v3(1, 0, 0)), cx = m3_apply(csm_rot(), v3(1, 0, 0));
    g->angle = acos(fmax(-1, fmin(1, -v3_dot(lx, cx))));
    // Roll: Columbia's -Y should lie along the LM's +Y.
    V3 cy_l = m3_apply(m3_transpose(lm_rot()), m3_apply(csm_rot(), v3(0, -1, 0)));
    g->roll = atan2(cy_l.z, cy_l.y);
    return g->range < 3000;
}

// Is an LM-frame point inside the ascent stage's rough shape?
static int in_lm(V3 p)
{
    if (p.x > 0.40 && p.x < 3.40 && p.y > -1.6 && p.y < 1.6 && p.z > -2.45 && p.z < 1.15) return 1;
    double ry = p.y, rz = p.z + 0.30;
    if (p.x >= 3.40 && p.x < MOUTH.x && ry * ry + rz * rz < 0.50 * 0.50) return 1;
    return 0;
}

// Is a CSM-frame point inside Columbia (CM cone, SM, nozzle)?
static int in_csm(V3 p)
{
    double r = sqrt(p.y * p.y + p.z * p.z);
    if (p.x <= 0.0 && p.x > -0.30) return r < 0.50;
    if (p.x <= -0.30 && p.x > CSM_CM_BASE) return r < 0.50 + (CSM_R - 0.50) * (-0.30 - p.x) / (-0.30 - CSM_CM_BASE);
    if (p.x <= CSM_CM_BASE && p.x > CSM_SM_END) return r < CSM_R;
    if (p.x <= CSM_SM_END && p.x > CSM_NOZZLE_END) return r < 0.40 + 0.85 * (CSM_SM_END - p.x) / (CSM_SM_END - CSM_NOZZLE_END);
    return 0;
}

// Any part of one vehicle inside the other? Sample points on each.
static int bodies_touch(void)
{
    static const double lx[] = { 0.45, 1.9, 3.35 }, ly[] = { -1.55, 0, 1.55 }, lz[] = { -2.4, -0.6, 1.1 };
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                if (in_csm(lm_to_csm(v3(lx[i], ly[j], lz[k])))) return 1;
    for (int a = 0; a < 12; a++) {
        double t = a * PI_D / 6;
        if (in_csm(lm_to_csm(v3(MOUTH.x, 0.5 * cos(t), -0.30 + 0.5 * sin(t))))) return 1;
        static const double cxs[] = { 0.0, -0.30, -1.2, -2.2, CSM_CM_BASE, -5.0, CSM_SM_END };
        for (int k = 0; k < 7; k++) {
            double x = cxs[k];
            double r = x > -0.30 ? 0.42 : x > CSM_CM_BASE ? 0.50 + (CSM_R - 0.50) * (-0.30 - x) / (-0.30 - CSM_CM_BASE) : CSM_R;
            if (in_lm(csm_to_lm(v3(x, r * cos(t), r * sin(t))))) return 1;
        }
    }
    // The probe itself, outside the drogue.
    for (int k = 0; k <= 4; k++) {
        V3 p = csm_to_lm(v3(CSM_PROBE_TIP * k / 4.0, 0, 0));
        double rd = sqrt(p.y * p.y + (p.z + 0.30) * (p.z + 0.30));
        if (in_lm(p) && rd > TUNNEL_R) return 1;
    }
    return 0;
}

static void bounce(double along_x_rate, V3 normal_l)
{
    // Push Eagle back (Columbia is six times heavier), keeping 30% of the speed.
    double vn = along_x_rate;
    V3 n = v3_norm(normal_l);
    V3 dv_l = v3_scale(n, 1.3 * vn);
    lm.v = v3_add(lm.v, m3_apply(lm_rot(), dv_l));
    dk.cooldown = 1.0;
    dk.bounces++;
}

static void fail(const char *why)
{
    dk.state = 3;
    snprintf(dk.fail, sizeof dk.fail, "%s", why);
    say(why);
    hidden = 0;
}

static void note_insertion(void);

void ascent_step(double dt)
{
    note_insertion();
    dap_sequence();
    if (!active || !phase_override || !csm.valid) return;
    if (dk.cooldown > 0) dk.cooldown -= dt;

    if (dk.state == 1 || dk.state == 2) {
        // Held by the capture latches: follow Columbia, pulled in by the probe.
        // The rings meeting square the two vehicles up: the relative attitude
        // goes to "LM +X along the CSM's -X, tunnels coaxial".
        double k = fmin(1.0, (lm.t - dk.t_capture) / RETRACT_TIME);
        double s = k * k * (3 - 2 * k);
        static const Q4 aligned = { 0, 0, 0, 1 };   // 180 deg about Z
        Q4 qr = slerp(dk.q_rel, aligned, s);
        V3 p1 = v3_scale(m3_apply(q4_to_m3(qr), MOUTH), -1);
        V3 p = v3_add(v3_scale(dk.p_rel0, 1 - s), v3_scale(p1, s));
        lm.r = v3_add(csm.r, m3_apply(csm_rot(), p));
        lm.v = csm.v;
        lm.q = q4_normalize(q4_mul(csm.q, qr));
        lm.w = v3(0, 0, 0);
        if (dk.state == 1 && k >= 1.0) {
            dk.state = 2;
            hidden = 0;
            say("\"...hard dock.\"  Twelve latches: Eagle and Columbia are one spacecraft again");
        }
        return;
    }
    if (dk.state == 3) return;

    DockGeo g;
    if (!ascent_dock_geo(&g)) return;
    double radial = sqrt(g.lat_y * g.lat_y + g.lat_z * g.lat_z);
    double closing = -g.rate;
    if (dk.prev_axial > 0 && g.axial <= 0 && dk.cooldown <= 0) {
        if (radial < DROGUE_R && g.angle < CAPTURE_ANGLE && closing > 0.005 && closing < CAPTURE_MAX_V) {
            dk.state = 1;
            dk.t_capture = lm.t;
            dk.c_speed = closing;
            dk.c_radial = radial;
            dk.c_angle = g.angle;
            dk.c_roll = g.roll;
            dk.c_rcs = dk.rcs_start - lm.m_rcs;
            // Keep the captured attitude; slide along the drogue axis until the rings meet.
            M3 Rc_t = m3_transpose(csm_rot());
            dk.q_rel = q4_normalize(q4_mul(q4_conj(csm.q), lm.q));
            dk.p_rel0 = m3_apply(Rc_t, v3_sub(lm.r, csm.r));
            dk.p_rel1 = v3_scale(m3_apply(q4_to_m3(dk.q_rel), MOUTH), -1);
            lm_set_thc(0, 0); lm_set_thc(1, 0); lm_set_thc(2, 0);
            lm_set_attitude_mode(0);   // the LM's DAP stands down: Columbia flies the stack
            say(closing > 0.3 ? "\"Capture.\"  A firm one: the latches took it. Probe retracting"
                              : "\"Capture.\"  Probe retracting");
            return;
        }
        if (closing >= CAPTURE_MAX_V && radial < TUNNEL_R) {
            char m[160];
            snprintf(m, sizeof m, "Hit the drogue at %.2f m/s (%.1f ft/s): too fast, the probe broke", closing, closing / 0.3048);
            fail(m);
            return;
        }
        if (radial < TUNNEL_R) {
            bounce(g.rate, v3(-1, 0, 0));
            say(g.angle >= CAPTURE_ANGLE ? "The probe hit the drogue at an angle and bounced off: line the axes up"
                                         : "The probe hit the rim of the tunnel and bounced off: centre the target first");
        }
    }
    dk.prev_axial = g.axial;
    if (dk.cooldown <= 0 && bodies_touch()) {
        V3 vrel = v3(-g.rate, -g.vy, -g.vz);   // LM relative to the CSM, LM axes
        double speed = v3_len(vrel);
        if (speed > DAMAGE_V) {
            char m[160];
            snprintf(m, sizeof m, "Eagle struck Columbia at %.2f m/s: both spacecraft damaged", speed);
            fail(m);
            return;
        }
        // Push apart along the line between them.
        V3 away = csm_to_lm(v3(-3.0, 0, 0));
        bounce(-speed, v3_scale(away, 1.0));
        say("Bumped Columbia - backed off. Keep the probe on the drogue axis");
    }
}

int ascent_results_due(void)
{
    if (!active || hidden) return 0;
    int ph = ascent_phase();
    if (ph == MP_ORBIT) return lm.t - ins.t_cut > 4 && ins.have;
    if (ph == MP_DOCKED) return lm.t - dk.t_capture > RETRACT_TIME + 2;
    if (ph == MP_DOCK_FAILED) return 1;
    return 0;
}

void ascent_hide_results(int hide) { hidden = hide; }
int ascent_results_hidden(void) { return hidden; }

// Record the insertion numbers once, when the engine cuts off.
static int ins_prev_aps;   // at file scope for save states
static void note_insertion(void)
{
#define prev_aps ins_prev_aps
    if (!active) { prev_aps = 0; return; }
    if (lm.aps_on && !ins.t_lift) ins.t_lift = lm.t;
    if (prev_aps && !lm.aps_on && lm.liftoff && !ins.have) {
        ins.t_cut = lm.t;
        orbit_km(&ins.peri, &ins.apo);
        ins.aps_left = lm.m_aps;
        ins.rcs_left = lm.m_rcs;
        ins.have = 1;
    }
    prev_aps = lm.aps_on;
#undef prev_aps
}

const AscentStats *ascent_stats(void)
{
    static AscentStats st;
    st.ins_have = ins.have;
    st.t_lift = ins.t_lift;
    st.t_cut = ins.t_cut;
    st.peri = ins.peri;
    st.apo = ins.apo;
    st.aps_left = ins.aps_left;
    st.rcs_left = ins.rcs_left;
    st.dock_state = dk.state;
    st.t_start = dk.t_start;
    st.t_capture = dk.t_capture;
    st.c_speed = dk.c_speed;
    st.c_radial = dk.c_radial;
    st.c_angle = dk.c_angle;
    st.c_roll = dk.c_roll;
    st.c_rcs = dk.c_rcs;
    st.bounces = dk.bounces;
    snprintf(st.msg, sizeof st.msg, "%s", dk.msg);
    st.msg_t = dk.msg_t;
    snprintf(st.fail, sizeof st.fail, "%s", dk.fail);
    return &st;
}

// --- Save states ---------------------------------------------------------------------

#include "snapshot.h"

void ascent_snap(Snap *s)
{
    snap_int(s, &active); snap_int(s, &phase_override); snap_int(s, &hidden); snap_int(s, &mirrored);
    snap_int(s, &dap_step); snap_f64(s, &dap_wait);
    snap_int(s, &dk.state); snap_f64(s, &dk.t_capture); snap_f64(s, &dk.t_start); snap_f64(s, &dk.prev_axial);
    snap_f64(s, &dk.cooldown); snap_int(s, &dk.bounces); snap_chars(s, dk.msg, sizeof dk.msg); snap_f64(s, &dk.msg_t);
    snap_v3(s, &dk.p_rel0); snap_v3(s, &dk.p_rel1); snap_q4(s, &dk.q_rel);
    snap_f64(s, &dk.c_speed); snap_f64(s, &dk.c_radial); snap_f64(s, &dk.c_angle); snap_f64(s, &dk.c_roll); snap_f64(s, &dk.c_rcs);
    snap_f64(s, &dk.rcs_start); snap_chars(s, dk.fail, sizeof dk.fail);
    snap_f64(s, &ins.t_lift); snap_f64(s, &ins.t_cut); snap_f64(s, &ins.peri); snap_f64(s, &ins.apo);
    snap_f64(s, &ins.aps_left); snap_f64(s, &ins.rcs_left); snap_int(s, &ins.have);
    snap_int(s, &ins_prev_aps);
}
