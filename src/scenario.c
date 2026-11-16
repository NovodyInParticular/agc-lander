// Apollo 11 pre-PDI scenario.
//
// The LGC erasable memory and discretes come from NASSP's "Apollo 11 - PDI
// T-15m" scenario (Luminary099, P63 at V50 N18). The vehicle's true state is
// then constructed from what the LGC itself believes: its LM state vector,
// REFSMMAT and IMU gimbal angles, so the simulated world and the computer
// start out consistent, as they were after the real P52 alignment and
// ground state-vector uplink.

#include <stdio.h>
#include <string.h>

#include "yaAGC.h"
#include "agc_engine.h"
#include "agc_host.h"
#include "agc_mem.h"
#include "csm.h"
#include "lm.h"
#include "moon.h"
#include "scenario.h"
#include "scenario_data.h"

agc_t *agc_host_state(void);

#define CDU_TO_RAD(c) ((double)((c) & 077777) * (2.0 * PI_D / 32768.0))

ScenarioInfo scenario_info;

// Default terrain until a real surface is attached: a sphere through the site.
static double site_sphere(void *ctx, double lat, double lon)
{
    (void)ctx; (void)lat; (void)lon;
    return scenario_info.site_radius;
}

// Two-body propagation (RK4 on point-mass lunar gravity).
static void propagate(V3 *r, V3 *v, double dt)
{
    const double h = dt > 0 ? 1.0 : -1.0;
    double left = dt;
    while ((dt > 0 && left > 1e-9) || (dt < 0 && left < -1e-9)) {
        double s = (dt > 0) ? (left < h ? left : h) : (left > h ? left : h);
        V3 r0 = *r, v0 = *v;
#define ACC(p) v3_scale((p), -MOON_MU / pow(v3_len(p), 3))
        V3 k1v = ACC(r0), k1r = v0;
        V3 r2 = v3_add(r0, v3_scale(k1r, s / 2)), v2 = v3_add(v0, v3_scale(k1v, s / 2));
        V3 k2v = ACC(r2), k2r = v2;
        V3 r3 = v3_add(r0, v3_scale(k2r, s / 2)), v3_ = v3_add(v0, v3_scale(k2v, s / 2));
        V3 k3v = ACC(r3), k3r = v3_;
        V3 r4 = v3_add(r0, v3_scale(k3r, s)), v4 = v3_add(v0, v3_scale(k3v, s));
        V3 k4v = ACC(r4), k4r = v4;
#undef ACC
        *r = v3_add(r0, v3_scale(v3_add(v3_add(k1r, v3_scale(k2r, 2)), v3_add(v3_scale(k3r, 2), k4r)), s / 6));
        *v = v3_add(v0, v3_scale(v3_add(v3_add(k1v, v3_scale(k2v, 2)), v3_add(v3_scale(k3v, 2), k4v)), s / 6));
        left -= s;
    }
}

static double read_time_cs(void)
{
    agc_t *s = agc_host_state();
    long t2 = s->Erasable[0][024] & 037777, t1 = s->Erasable[0][025] & 037777;
    return (double)(t2 << 14 | t1);
}

// Erasable memory and channels, then a hardware restart (GOJAM): Luminary
// resumes the running program from its phase tables.
static void load_lgc(const ScenarioWord *em, int nem, const ScenarioWord *ch, int nch, int superbank)
{
    agc_t *s = agc_host_state();
    memset(s->Erasable, 0, sizeof s->Erasable);
    for (int i = 0; i < nem; i++) {
        int a = em[i].address;
        s->Erasable[a / 0400][a % 0400] = (int16_t)em[i].value;
    }
    for (int i = 0; i < nch; i++)
        s->InputChannel[ch[i].address] = (int16_t)ch[i].value;
    s->OutputChannel7 = s->InputChannel[7] = (int16_t)superbank;

    s->Erasable[0][RegZ] = 04000;
    for (int j = 0; j < 1 + NUM_INTERRUPT_TYPES; j++)
        s->InterruptRequests[j] = 0;
    s->InIsr = 0;
    s->AllowInterrupt = 1;
    s->ExtraCode = 0;
    s->PendFlag = 0;
    s->PendDelay = 0;
    s->ExtraDelay = 0;
    s->SubstituteInstruction = 0;
    s->RestartLight = 0;
    if (s->CycleCounter == 0)
        s->CycleCounter = 1;   // the core lights RESTART on its very first cycle
}

static double read_tephem(void)
{
    // TEPHEM is triple precision, csec scaled 2^42.
    double w0 = agc_from_ones(agc_get_word("TEPHEM", 0));
    double w1 = agc_from_ones(agc_get_word("TEPHEM", 1));
    double w2 = agc_from_ones(agc_get_word("TEPHEM", 2));
    return ((w0 * 16384.0 + w1) * 16384.0 + w2) / 100.0;
}

static M3 read_refsmmat(void)
{
    M3 m;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            m.m[i][j] = agc_get_dp("REFSMMAT", 6 * i + 2 * j, 1);
    return m;
}

// The CSM's state vector as the LGC holds it, propagated to now.
static void load_csm(void)
{
    V3 r = v3_add(agc_get_vec("RCVCSM", 27), agc_get_vec("DELTACSM", 18));
    V3 v = v3_scale(v3_add(agc_get_vec("VCVCSM", 5), agc_get_vec("NUVCSM", -1)), 100.0);
    double tet = agc_get_dp("TETCSM", 0, 28) / 100.0;
    if (v3_len(r) < 1.5e6 || v3_len(r) > 2.5e6) return;   // none loaded
    propagate(&r, &v, lm.t - tet);
    csm_set_state(r, v);
}

void scenario_pre_pdi(void)
{
    agc_t *s = agc_host_state();

    load_lgc(a11_pdi_erasable, a11_pdi_erasable_count, a11_pdi_channels, a11_pdi_channel_count, a11_pdi_superbank);

    // 3. Vehicle state from the LGC's own knowledge.
    lm_reset();
    double t_cs = read_time_cs();
    lm.t = t_cs / 100.0;
    lm.tephem = read_tephem();

    V3 rcv = agc_get_vec("RCVLEM", 27);
    V3 delta = agc_get_vec("DELTALEM", 18);
    V3 vcv = agc_get_vec("VCVLEM", 5);
    V3 nuv = agc_get_vec("NUVLEM", -1);
    double tet = agc_get_dp("TETLEM", 0, 28) / 100.0;
    V3 r = v3_add(rcv, delta);
    V3 v = v3_scale(v3_add(vcv, nuv), 100.0);   // m/cs -> m/s
    scenario_info.sv_age = lm.t - tet;
    propagate(&r, &v, lm.t - tet);
    lm.r = r;
    lm.v = v;

    // Attitude: NB = Rx(OGA) Rz(MGA) Ry(IGA) SM, SM rows from REFSMMAT.
    M3 refsmmat = read_refsmmat();
    double ogaa = CDU_TO_RAD(s->Erasable[0][RegCDUX]);
    double igaa = CDU_TO_RAD(s->Erasable[0][RegCDUY]);
    double mgaa = CDU_TO_RAD(s->Erasable[0][RegCDUZ]);
    M3 nb_from_sm = m3_mul(m3_rx(ogaa), m3_mul(m3_rz(mgaa), m3_ry(igaa)));
    M3 i_from_sm = m3_transpose(refsmmat);
    M3 i_from_nb = m3_mul(i_from_sm, m3_transpose(nb_from_sm));
    lm.q = m3_to_q4(i_from_nb);
    lm.w = v3(0, 0, 0);
    lm.q_sm = m3_to_q4(i_from_sm);
    lm.cdu_sent[0] = s->Erasable[0][RegCDUX] & 077777;
    lm.cdu_sent[1] = s->Erasable[0][RegCDUY] & 077777;
    lm.cdu_sent[2] = s->Erasable[0][RegCDUZ] & 077777;

    // Masses from the scenario (RCS propellant nearly full).
    lm.m_desc_dry = a11_pdi_dscemptymass;
    lm.m_dps = a11_pdi_dscfuel;
    lm.m_ascent = a11_pdi_ascemptymass;
    lm.m_aps = a11_pdi_ascfuel;
    lm.m_rcs = 270.0;

    // Landing site.
    V3 rls = agc_get_vec("RLS", 27);
    double lat, lon, rad;
    moon_fixed_to_latlon(rls, &lat, &lon, &rad);
    scenario_info.rls = rls;
    scenario_info.site_lat = lat;
    scenario_info.site_lon = lon;
    scenario_info.site_radius = rad;
    scenario_info.tland = agc_get_dp("TLAND", 0, 28) / 100.0;
    scenario_info.refsmmat = refsmmat;
    if (!lm.surface)
        lm.surface = site_sphere;

    csm_reset();
    load_csm();

    lm_attach_agc();
    // Discretes from the scenario, then our hardware models on top.
    lm_set_attitude_mode(2);
    lm_set_engine_arm(0);
}

void scenario_liftoff(int keep_pose)
{
    agc_t *s = agc_host_state();
    V3 keep_fixed = moon_inertial_to_fixed(lm.r, lm.t + lm.tephem);
    M3 keep_rot = m3_mul(moon_matrix(lm.t + lm.tephem), q4_to_m3(lm.q));
    double keep_dps = lm.m_dps, keep_rcs = lm.m_rcs;
    double collapse[4];
    memcpy(collapse, lm.collapse, sizeof collapse);

    load_lgc(a11_liftoff_erasable, a11_liftoff_erasable_count, a11_liftoff_channels, a11_liftoff_channel_count,
             a11_liftoff_superbank);

    SurfaceFn sf = lm.surface;
    lm_reset();
    lm.t = read_time_cs() / 100.0;
    lm.tephem = read_tephem();
    double t = lm.t + lm.tephem;
    M3 refsmmat = read_refsmmat();
    M3 i_from_sm = m3_transpose(refsmmat);
    lm.q_sm = m3_to_q4(i_from_sm);

    if (keep_pose) {
        // Where the player landed: same spot and attitude in the moon-fixed frame.
        lm.r = moon_fixed_to_inertial(keep_fixed, t);
        lm.q = m3_to_q4(m3_mul(m3_transpose(moon_matrix(t)), keep_rot));
        lm.m_dps = keep_dps;
        lm.m_rcs = keep_rcs;
        memcpy(lm.collapse, collapse, sizeof collapse);
    } else {
        // At the landing target of our terrain (the LGC's own site, where NASSP's
        // crew put Eagle, is ~750 m off and sits on a crater wall here), upright,
        // heading from the CDUs; the legs then settle onto the slope.
        V3 rls = scenario_info.rls;
        if (v3_len(rls) < 1.5e6) rls = agc_get_vec("RLS", 27);
        double lat, lon, rad;
        moon_fixed_to_latlon(rls, &lat, &lon, &rad);
        double ground = sf ? sf(lm.surface_ctx, lat, lon) : rad;
        lm.r = moon_fixed_to_inertial(v3_scale(v3_norm(rls), ground + 2.75 + 0.02), t);
        double ogaa = CDU_TO_RAD(s->Erasable[0][RegCDUX]);
        double igaa = CDU_TO_RAD(s->Erasable[0][RegCDUY]);
        double mgaa = CDU_TO_RAD(s->Erasable[0][RegCDUZ]);
        M3 nb_from_sm = m3_mul(m3_rx(ogaa), m3_mul(m3_rz(mgaa), m3_ry(igaa)));
        lm.q = m3_to_q4(m3_mul(i_from_sm, m3_transpose(nb_from_sm)));
        lm.m_dps = 360.0;
        lm.m_rcs = 250.0;
    }
    lm.v = v3_cross(moon_omega(t), lm.r);
    lm.w = v3(0, 0, 0);
    lm.m_desc_dry = a11_liftoff_dscemptymass;
    lm.m_ascent = a11_liftoff_ascemptymass;
    lm.m_aps = a11_liftoff_ascfuel;
    lm.surface = sf;
    lm_set_landed();
    if (!keep_pose) {
        lm_place_on_ground();
        lm_settle(4.0);   // onto our terrain's slope
    }

    // The CDUs read the attitude as it really is, and the DAP's commanded
    // attitude is that one: it holds the LM as it stands instead of firing
    // jets against the ground until lift-off.
    V3 g = lm_gimbal_angles();
    lm.cdu_sent[0] = (int)(lround(g.x / (2.0 * PI_D / 32768.0)) & 077777);
    lm.cdu_sent[1] = (int)(lround(g.y / (2.0 * PI_D / 32768.0)) & 077777);
    lm.cdu_sent[2] = (int)(lround(g.z / (2.0 * PI_D / 32768.0)) & 077777);
    s->Erasable[0][RegCDUX] = (int16_t)lm.cdu_sent[0];
    s->Erasable[0][RegCDUY] = (int16_t)lm.cdu_sent[1];
    s->Erasable[0][RegCDUZ] = (int16_t)lm.cdu_sent[2];
    agc_set_word("CDUXD", 0, lm.cdu_sent[0]);
    agc_set_word("CDUYD", 0, lm.cdu_sent[1]);
    agc_set_word("CDUZD", 0, lm.cdu_sent[2]);

    csm_reset();
    load_csm();

    lm_attach_agc();
    lm_set_attitude_mode(2);
    lm_set_engine_arm(0);
}
