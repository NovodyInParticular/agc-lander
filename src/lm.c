// Lunar Module simulation: rigid-body dynamics plus the hardware the AGC
// talks to. Sources for each interface are noted in PROGRESS.md.

#include <math.h>
#include <string.h>

#include "yaAGC.h"
#include "agc_engine.h"
#include "agc_host.h"
#include "lm.h"
#include "coop.h"
#include "moon.h"

agc_t *agc_host_state(void);

Lm lm;

// ---------------------------------------------------------------------------
// Constants.

#define TWO_PI (2.0 * PI_D)
#define CDU_RAD   (TWO_PI / 32768.0)            // 2^-15 rev per CDU count
#define GYRO_RAD  (TWO_PI / 2097152.0)          // 2^-21 rev per gyro pulse
#define COARSE_RAD (TWO_PI / 8192.0)            // 2^-13 rev per coarse pulse
#define PIPA_MPS  0.01                          // 1 cm/s per PIPA pulse

#define G0 9.80665
#define DPS_ISP 311.0
#define DPS_N_PER_BIT (1.0 / 0.0797959872)      // SCALEFAC (BITPERF)
#define DPS_FMAX 43455.0                        // FTP hardware stop, N
#define DPS_SAT_BITS 3841.0                     // FMAXODD
#define DPS_MIN_THRUST 4698.0                   // ~10% manual throttle minimum
#define DPS_LAG 0.2                             // THROTLAG, s
#define GIMBAL_RATE (0.2 * PI_D / 180.0)        // trim drive rate, rad/s
#define GIMBAL_MAX (6.0 * PI_D / 180.0)

#define RCS_THRUST 445.0
#define RCS_ISP 290.0

// Geometry (body NB axes, metres, relative to the centre of mass).
#define QUAD_R 1.19        // quad offset along each of Y and Z
#define QUAD_X 1.60        // quads above the CG
#define ENGINE_X (-1.40)   // DPS gimbal point below the CG
#define PAD_X (-2.75)      // footpad plane below the CG
#define PAD_R 4.40         // footpad distance from the X axis
#define PROBE_LEN 1.73     // 67.2 inch contact probes

// LR beam geometry (Luminary CONTROLLED_CONSTANTS, LUM69 padload).
#define LR_ALPHA (6.0 * PI_D / 180.0)
#define LR_BETA1 (24.0 * PI_D / 180.0)
#define LR_BETA2 0.0
#define LR_MAX_RANGE (50000.0 * 0.3048)
#define LR_VEL_MAX_ALT (25000.0 * 0.3048)
#define FT 0.3048
#define REG_THRUST 055   // LM use of counter 055
#define LR_PERIOD 0.05   // radar measurement update interval, s

// Quad positions (Y, Z) and each jet's thrust direction in body axes.
typedef struct { int quad; V3 dir; } Jet;
static const V3 quad_pos[5] = {
    { 0, 0, 0 },
    { QUAD_X, -QUAD_R, QUAD_R },    // Q1 front-left
    { QUAD_X, -QUAD_R, -QUAD_R },   // Q2 aft-left
    { QUAD_X, QUAD_R, -QUAD_R },    // Q3 aft-right
    { QUAD_X, QUAD_R, QUAD_R },     // Q4 front-right
};
// U jets fire upward (thrust -X), D downward (+X), F forward (-Z), A aft (+Z),
// L left (+Y), R right (-Y).
static const Jet jets[LM_NJETS] = {
    { 4, { -1, 0, 0 } }, { 4, { 1, 0, 0 } },  // Q4U Q4D
    { 3, { -1, 0, 0 } }, { 3, { 1, 0, 0 } },  // Q3U Q3D
    { 2, { -1, 0, 0 } }, { 2, { 1, 0, 0 } },  // Q2U Q2D
    { 1, { -1, 0, 0 } }, { 1, { 1, 0, 0 } },  // Q1U Q1D
    { 3, { 0, 0, 1 } },  { 4, { 0, 0, -1 } }, // Q3A Q4F
    { 1, { 0, 0, -1 } }, { 2, { 0, 0, 1 } },  // Q1F Q2A
    { 2, { 0, 1, 0 } },  { 3, { 0, -1, 0 } }, // Q2L Q3R
    { 4, { 0, -1, 0 } }, { 1, { 0, 1, 0 } },  // Q4R Q1L
};

V3 lm_jet_position(int j) { return quad_pos[jets[j].quad]; }
V3 lm_jet_direction(int j) { return jets[j].dir; }

static int rod_pending;      // ROD clicks to deliver
static double rod_hold;      // seconds the current click bit stays set
static int engine_stop_latched;
static int pad_hit;           // footpads that have touched (bit per pad)
static int att_mode_cur = 2;   // PGNS mode control switch: 0 off, 1 att hold, 2 auto

// ---------------------------------------------------------------------------
// Helpers.

double lm_mass(void) { return lm.m_ascent + lm.m_desc_dry + lm.m_dps + lm.m_rcs; }

static V3 inertia(void)
{
    // Principal moments scaled from LM data-book values at PDI mass.
    double s = lm_mass() / 15100.0;
    return v3(25000.0 * s, 34000.0 * s, 33000.0 * s);
}

V3 lm_body_to_inertial(V3 b) { return q4_rotate(lm.q, b); }
V3 lm_inertial_to_body(V3 i) { return q4_rotate(q4_conj(lm.q), i); }

static M3 c_body_from_inertial(void) { return m3_transpose(q4_to_m3(lm.q)); }
static M3 c_inertial_from_sm(void) { return q4_to_m3(lm.q_sm); }

// NB = Rx(OGA) Rz(MGA) Ry(IGA) SM, as in the AGC's AX*SR*T.
V3 lm_gimbal_angles(void)
{
    M3 t = m3_mul(c_body_from_inertial(), c_inertial_from_sm());
    double m = asin(fmax(-1.0, fmin(1.0, t.m[0][1])));
    double i = atan2(-t.m[0][2], t.m[0][0]);
    double o = atan2(-t.m[2][1], t.m[1][1]);
    return v3(o, i, m);
}

static M3 gimbal_matrix(double o, double i, double m)
{
    return m3_mul(m3_rx(o), m3_mul(m3_rz(m), m3_ry(i)));
}

// Recompute the stable member orientation for given gimbal angles.
static void set_gimbals(double o, double i, double m)
{
    M3 c_i_b = q4_to_m3(lm.q);
    M3 c_i_sm = m3_mul(c_i_b, gimbal_matrix(o, i, m));
    lm.q_sm = m3_to_q4(c_i_sm);
}

static int angle_to_cdu(double a)
{
    long c = lround(a / CDU_RAD);
    return (int)(c & 077777);
}

static int wrap15(int d)
{
    d &= 077777;
    return d >= 040000 ? d - 0100000 : d;
}

double lm_surface_radius_below(void)
{
    double lat, lon, rad;
    V3 rp = moon_inertial_to_fixed(lm.r, lm.t + lm.tephem);
    moon_fixed_to_latlon(rp, &lat, &lon, &rad);
    return lm.surface ? lm.surface(lm.surface_ctx, lat, lon) : MOON_RADIUS;
}

static double surface_radius_at(V3 r_inertial)
{
    double lat, lon, rad;
    V3 rp = moon_inertial_to_fixed(r_inertial, lm.t + lm.tephem);
    moon_fixed_to_latlon(rp, &lat, &lon, &rad);
    return lm.surface ? lm.surface(lm.surface_ctx, lat, lon) : MOON_RADIUS;
}

V3 lm_surface_velocity(void)
{
    V3 w = moon_omega(lm.t + lm.tephem);
    return v3_sub(lm.v, v3_cross(w, lm.r));
}

// Distance along an inertial ray from the LM to the terrain, or -1.
static double raycast(V3 dir, double max_range)
{
    double step = 50.0, prev = 0;
    double h0 = v3_len(lm.r) - surface_radius_at(lm.r);
    if (h0 < step * 4) step = fmax(0.5, h0 / 4);
    for (double s = step; s <= max_range; s += step) {
        V3 p = v3_add(lm.r, v3_scale(dir, s));
        double h = v3_len(p) - surface_radius_at(p);
        if (h <= 0) {
            // Bisect between prev and s.
            double a = prev, b = s;
            for (int k = 0; k < 20; k++) {
                double c = (a + b) / 2;
                V3 pc = v3_add(lm.r, v3_scale(dir, c));
                if (v3_len(pc) - surface_radius_at(pc) > 0) a = c; else b = c;
            }
            return (a + b) / 2;
        }
        prev = s;
        if (h > 4 * step) step *= 1.5;
    }
    return -1;
}

// ---------------------------------------------------------------------------
// AGC output handling.

static void on_output(int ch, int value)
{
    agc_t *s = agc_host_state();
    switch (ch) {
    case 005: lm.ch5 = (unsigned)value & 0377; break;
    case 006: lm.ch6 = (unsigned)value & 0377; break;
    case 011:
        if (value & 020000) lm.dps_on = 0;                          // ENGINE OFF
        else if ((value & 010000) && lm.engine_armed && !engine_stop_latched)
            lm.dps_on = 1;                                           // ENGINE ON
        break;
    case 0174: case 0175: case 0176: {
        int n = value & 037777;
        if (value & 040000) n = -n;
        int axis = ch - 0174;
        if (s->InputChannel[012] & 010) {
            // Coarse align: drive the gimbal directly.
            V3 g = lm_gimbal_angles();
            double a[3] = { g.x, g.y, g.z };
            a[axis] += n * COARSE_RAD;
            set_gimbals(a[0], a[1], a[2]);
        } else {
            lm.needle[axis] += n;  // FDAI attitude-error needle drive
        }
        break;
    }
    case 0177: {
        // Gyro fine-align torquing: bit15 sign, bits 14-13 select, 11 bits count.
        int count = value & 03777;
        int sel = (value >> 12) & 3;
        double ang = count * GYRO_RAD * ((value & 040000) ? -1 : 1);
        V3 axis = sel == 1 ? v3(1, 0, 0) : sel == 2 ? v3(0, 1, 0) : sel == 3 ? v3(0, 0, 1) : v3(0, 0, 0);
        if (sel)
            lm.q_sm = q4_normalize(q4_mul(lm.q_sm, q4_axis_angle(axis, ang)));
        break;
    }
    case 0x80 | 055:  // THRUST counter output: POUT / MOUT
        if (value == 015) lm.thr_bits += 1;
        else if (value == 016) lm.thr_bits -= 1;
        if (lm.thr_bits > DPS_SAT_BITS) lm.thr_bits = DPS_SAT_BITS;
        if (lm.thr_bits < 0) lm.thr_bits = 0;
        break;
    default:
        break;
    }
}

// Landing radar measurement for the selected data type (ch 013 bits 1-3).
void RequestRadarData(agc_t *State)
{
    int sel = State->InputChannel[013] & 7;
    int value = 0;
    if (sel >= 4 && sel <= 6) {
        // LRVELX (4), LRVELY (5), LRVELZ (6): counts = 12288 + v / scale.
        static const double ft_per_bit[3] = { -0.644, 1.212, 0.8668 };
        int k = sel - 4;
        double vc = (k == 0 ? lm.lr_vel_ant.x : k == 1 ? lm.lr_vel_ant.y : lm.lr_vel_ant.z) / FT;
        double c = 12288.0 + vc / ft_per_bit[k];
        if (c < 0) c = 0;
        if (c > 077777) c = 077777;
        value = (int)lround(c);
    } else if (sel == 7) {
        // High scale 5.395 ft/count, low scale 1.079 ft/count (HISCALIM: 460
        // counts = 2481.7 ft). Ch33 bit 9 tells the AGC which one is in use.
        double ft_per_count = lm.lr_low_scale ? 1.079 : 5.395;
        double c = lm.lr_range > 0 ? lm.lr_range / (ft_per_count * FT) : 0;
        if (c > 077777) c = 077777;
        value = (int)lround(c);
    }
    State->Erasable[0][RegRNRAD] = (int16_t)(value & 077777);
}

// ---------------------------------------------------------------------------
// Setup.

void lm_reset(void)
{
    SurfaceFn sf = lm.surface;
    void *ctx = lm.surface_ctx;
    memset(&lm, 0, sizeof lm);
    lm.surface = sf;
    lm.surface_ctx = ctx;
    lm.q = q4_identity();
    lm.q_sm = q4_identity();
    // Default: circular orbit 15 km up (overridden by scenarios).
    double r0 = MOON_RADIUS + 15000.0;
    lm.r = v3(r0, 0, 0);
    lm.v = v3(0, sqrt(MOON_MU / r0), 0);
    lm.lr_pos = 1;
    lm.lr_range = -1;
    engine_stop_latched = 0;
    pad_hit = 0;
    rod_pending = 0;
    rod_hold = 0;
    // Apollo 11 LM near PDI (kg).
    lm.m_ascent = 4850.0;
    lm.m_desc_dry = 2180.0;
    lm.m_dps = 8073.0;
    lm.m_rcs = 287.0;
}

void lm_attach_agc(void)
{
    agc_host_set_output_hook(on_output);
    // Channel 030 (active low): IMU operate (9), AGC has control (10),
    // auto throttle (5); temperature in limits (15 = 0).
    agc_host_set_input_bits(030, 040000 | 01000 | 0400 | 020, 0);
    // Channel 031: attitude mode AUTO (bit 14 low), hand controllers in detent.
    agc_host_set_input_bits(031, 077777, 077777 & ~020000);
    att_mode_cur = 2;
    // Channel 033: LR range on high scale (bit 9 = 1), LR in position 1
    // (bit 6 low), data-good bits high until the radar has lock.
    agc_host_set_input_bits(033, 0777, 0777 & ~040);
    lm_set_engine_arm(lm.engine_armed);
}

void lm_align_imu(M3 refsmmat)
{
    // REFSMMAT rows are the SM axes in inertial coordinates.
    lm.q_sm = m3_to_q4(m3_transpose(refsmmat));
    V3 g = lm_gimbal_angles();
    lm.cdu_sent[0] = angle_to_cdu(g.x);
    lm.cdu_sent[1] = angle_to_cdu(g.y);
    lm.cdu_sent[2] = angle_to_cdu(g.z);
    agc_t *s = agc_host_state();
    s->Erasable[0][RegCDUX] = (int16_t)lm.cdu_sent[0];
    s->Erasable[0][RegCDUY] = (int16_t)lm.cdu_sent[1];
    s->Erasable[0][RegCDUZ] = (int16_t)lm.cdu_sent[2];
    lm.pipa_residual = v3(0, 0, 0);
}

// ---------------------------------------------------------------------------
// Crew controls.

int lm_get_attitude_mode(void) { return att_mode_cur; }
int lm_engine_stop_is_latched(void) { return engine_stop_latched; }

void lm_set_attitude_mode(int mode)
{
    if (coop_role == COOP_CLIENT) { coop_forward(CMD_ATTMODE, mode, 0); att_mode_cur = mode; return; }

    att_mode_cur = mode;
    int v = 077777;
    if (mode == 1) v &= ~010000;   // ATT HOLD: bit 13 low
    if (mode == 2) v &= ~020000;   // AUTO: bit 14 low
    agc_host_set_input_bits(031, 030000, v);
}

void lm_set_engine_arm(int armed)
{
    if (coop_role == COOP_CLIENT) { coop_forward(CMD_ENGARM, armed, 0); return; }

    lm.engine_armed = armed;
    agc_host_set_input_bits(030, 04, armed ? 0 : 04);  // bit 3, active low
    if (!armed) lm.dps_on = 0;
}

double lm_tilt(void)
{
    V3 up_b = lm_inertial_to_body(v3_norm(lm.r));
    return acos(fmax(-1.0, fmin(1.0, up_b.x)));
}

void lm_engine_stop(void)
{
    if (coop_role == COOP_CLIENT) { coop_forward(CMD_STOP, 0, 0); return; }

    engine_stop_latched = 1;
    lm.dps_on = 0;
}

void lm_rod_click(int direction)
{
    if (coop_role == COOP_CLIENT) { coop_forward(CMD_ROD, direction, 0); return; }
    rod_pending += direction > 0 ? 1 : -1;
}

// Co-op LMP: mirror the host's switch states without commanding anything.
void lm_mirror_controls(int att_mode, int stop_latched)
{
    att_mode_cur = att_mode;
    engine_stop_latched = stop_latched;
}

void lm_set_thc(int axis, int dir)
{
    if (coop_role == COOP_CLIENT) { coop_forward(CMD_THC, axis, dir); return; }

    // Channel 031 bits 7-12: +X -X +Y -Y +Z -Z, active low.
    int plus = 0100 << (axis * 2), minus = 0200 << (axis * 2);
    int v = plus | minus;
    if (dir > 0) v &= ~plus;
    if (dir < 0) v &= ~minus;
    agc_host_set_input_bits(031, plus | minus, v);
}

// ---------------------------------------------------------------------------
// Physics step.

static void update_sensors_to_agc(double dt)
{
    agc_t *s = agc_host_state();

    // CDUs follow the gimbals.
    V3 g = lm_gimbal_angles();
    double ga[3] = { g.x, g.y, g.z };
    for (int k = 0; k < 3; k++) {
        int want = angle_to_cdu(ga[k]);
        int d = wrap15(want - lm.cdu_sent[k]);
        if (d) {
            agc_host_pulse(RegCDUX + k, d, AGC_PULSE_CDU_FAST);
            lm.cdu_sent[k] = (lm.cdu_sent[k] + d) & 077777;
        }
    }

    // PIPAs integrate non-gravitational acceleration in SM axes.
    V3 dv_sm = m3_apply(m3_transpose(c_inertial_from_sm()), v3_scale(lm.accel_ng, dt));
    lm.pipa_residual = v3_add(lm.pipa_residual, dv_sm);
    double *res[3] = { &lm.pipa_residual.x, &lm.pipa_residual.y, &lm.pipa_residual.z };
    for (int k = 0; k < 3; k++) {
        int n = (int)(*res[k] / PIPA_MPS);
        if (n) {
            agc_host_pulse(RegPIPAX + k, n, AGC_PULSE_PINC);
            *res[k] -= n * PIPA_MPS;
        }
    }

    // THRUST counter is drained by DINC at 3200 pulses/s while ch14 bit 4 is set.
    if ((s->InputChannel[014] & 010) && s->Erasable[0][REG_THRUST] != 0) {
        lm.thrust_dinc_acc += 3200.0 * dt;
        int n = (int)lm.thrust_dinc_acc;
        int pending = agc_host_pending(REG_THRUST);
        if (n > 0 && pending < 4) {
            agc_host_pulse(REG_THRUST, n, AGC_PULSE_DINC);
            lm.thrust_dinc_acc -= n;
        }
    } else {
        lm.thrust_dinc_acc = 0;
    }

    // Landing radar discretes (channel 033, active low).
    int ch33 = 0;
    if (!lm.lr_range_good) ch33 |= 020;
    if (!lm.lr_vel_good) ch33 |= 0200;
    if (!(lm.lr_pos == 1 && lm.lr_move <= 0)) ch33 |= 040;
    if (!(lm.lr_pos == 2 && lm.lr_move <= 0)) ch33 |= 0100;
    if (!lm.lr_low_scale) ch33 |= 0400;   // bit 9 low = LR range low scale
    agc_host_set_input_bits(033, 0760, ch33);

    // ROD switch: hold bit 6 (+) or 7 (-) of channel 016 for 50 ms per click,
    // with a MARKRUPT at the leading edge.
    if (rod_hold > 0) {
        rod_hold -= dt;
        if (rod_hold <= 0)
            agc_host_set_input_bits(016, 0140, 0);
    } else if (rod_pending) {
        int bit = rod_pending > 0 ? 040 : 0100;
        rod_pending += rod_pending > 0 ? -1 : 1;
        agc_host_set_input_bits(016, 0140, bit);
        s->InterruptRequests[6] = 1;
        rod_hold = 0.05;
    }
}

static double radar_timer;

static void update_radar(double dt)
{
    agc_t *s = agc_host_state();
    // Ch12 bit 13 drives the antenna to position 2; the software removes the
    // command once ch33 shows it has arrived, and the antenna stays there.
    if ((s->InputChannel[012] & 010000) && lm.lr_pos != 2 && lm.lr_move <= 0) {
        lm.lr_move = 6.0;
        lm.lr_pos = 2;
    }
    if (lm.lr_move > 0) lm.lr_move -= dt;
    radar_timer -= dt;
    if (radar_timer > 0)
        return;
    radar_timer = LR_PERIOD;

    // Antenna axes in NB: v_NB = Rx(alpha) Ry(beta) v_ant.
    double beta = lm.lr_pos == 1 ? LR_BETA1 : LR_BETA2;
    M3 nb_from_ant = gimbal_matrix(LR_ALPHA, beta, 0);
    M3 ant_from_nb = m3_transpose(nb_from_ant);
    V3 hbeam_ant = v3_norm(v3(-0.4687018041, 0, -0.1741224271));
    V3 hbeam_i = lm_body_to_inertial(m3_apply(m3_transpose(ant_from_nb), hbeam_ant));

    double range = lm.lr_move > 0 ? -1 : raycast(hbeam_i, LR_MAX_RANGE);
    lm.lr_range = range;
    // Range scale switch at ~2500 ft, just above Luminary's HISCALIM check
    // (460 high-scale counts = 2481.7 ft): if the radar were still on high
    // scale below that, R12 would flag SCALBAD and light ALT.
    if (range > 0) {
        if (lm.lr_low_scale && range > 2750 * FT) lm.lr_low_scale = 0;
        else if (!lm.lr_low_scale && range < 2520 * FT) lm.lr_low_scale = 1;
    }
    lm.lr_range_good = range > 0;
    if (range > 0)
        lm.lr_alt = -v3_dot(v3_scale(hbeam_i, range), v3_norm(lm.r));

    V3 vrel_b = lm_inertial_to_body(lm_surface_velocity());
    lm.lr_vel_ant = m3_apply(ant_from_nb, vrel_b);
    lm.lr_vel_good = range > 0 && lm.altitude < LR_VEL_MAX_ALT && lm.lr_move <= 0;
}

static void rigid_body(double dt)
{
    double m = lm_mass();
    V3 F_b = v3(0, 0, 0), T_b = v3(0, 0, 0);

    // DPS.
    agc_t *s = agc_host_state();
    if (!lm.dps_on || lm.m_dps <= 0) {
        lm.thrust += (0 - lm.thrust) * fmin(1.0, dt / 0.08);
        if (!lm.dps_on) lm.flame_time = 0;
    } else {
        double cmd = fmin(DPS_FMAX, fmax(DPS_MIN_THRUST, lm.thr_bits * DPS_N_PER_BIT));
        lm.thrust += (cmd - lm.thrust) * fmin(1.0, dt / DPS_LAG);
        lm.flame_time += dt;
    }
    if (lm.thrust > 1.0) {
        // Trim gimbal driven by ch12 bits 9-12 (pitch +/-, roll +/-).
        int c12 = s->InputChannel[012];
        double pitch_rate = (c12 & 0400) ? GIMBAL_RATE : (c12 & 01000) ? -GIMBAL_RATE : 0;
        double roll_rate = (c12 & 02000) ? GIMBAL_RATE : (c12 & 04000) ? -GIMBAL_RATE : 0;
        lm.gimbal[0] = fmax(-GIMBAL_MAX, fmin(GIMBAL_MAX, lm.gimbal[0] + pitch_rate * dt));
        lm.gimbal[1] = fmax(-GIMBAL_MAX, fmin(GIMBAL_MAX, lm.gimbal[1] + roll_rate * dt));
        // Positive gimbal[0] (ch12 bit 9) gives +Y (pitch, Q) torque and positive
        // gimbal[1] (bit 11) +Z (roll, R) torque, as the DAP's WRCHN12 expects.
        // Engine is below the CG (ENGINE_X < 0): T = r x F.
        V3 dir = v3_norm(v3(1, -lm.gimbal[1], lm.gimbal[0]));
        V3 f = v3_scale(dir, lm.thrust);
        F_b = v3_add(F_b, f);
        T_b = v3_add(T_b, v3_cross(v3(ENGINE_X, 0, 0), f));
        lm.m_dps = fmax(0, lm.m_dps - lm.thrust / (DPS_ISP * G0) * dt);
    }

    // RCS.
    unsigned bits = lm.ch5 | (lm.ch6 << 8);
    for (int j = 0; j < LM_NJETS; j++) {
        int on = (bits >> j) & 1 && lm.m_rcs > 0;
        float target = on ? 1.0f : 0.0f;
        lm.jet_level[j] += (target - lm.jet_level[j]) * (float)fmin(1.0, dt / 0.02);
        if (!on) continue;
        V3 f = v3_scale(jets[j].dir, RCS_THRUST);
        F_b = v3_add(F_b, f);
        T_b = v3_add(T_b, v3_cross(quad_pos[jets[j].quad], f));
        lm.m_rcs = fmax(0, lm.m_rcs - RCS_THRUST / (RCS_ISP * G0) * dt);
    }

    // Ground contact: spring-damper at each footpad, in the moon-fixed frame.
    V3 F_i_contact = v3(0, 0, 0);
    V3 w_moon = moon_omega(lm.t + lm.tephem);
    lm.probe_contact = 0;
    int pads_down = 0;
    for (int k = 0; k < 4; k++) {
        double ang = k * PI_D / 2;
        V3 pad_b = v3(PAD_X + lm.collapse[k], PAD_R * sin(ang), PAD_R * cos(ang));
        V3 pad_i = v3_add(lm.r, lm_body_to_inertial(pad_b));
        double h = v3_len(pad_i) - surface_radius_at(pad_i);
        if (k != 0 && h < PROBE_LEN) lm.probe_contact = 1;   // no probe on the front leg
        if (h >= 0) continue;
        pads_down++;
        V3 up = v3_norm(pad_i);
        V3 vpad = v3_add(lm.v, lm_body_to_inertial(v3_cross(lm.w, pad_b)));
        vpad = v3_sub(vpad, v3_cross(w_moon, pad_i));
        double vn = v3_dot(vpad, up);
        // Strut crush: the honeycomb cartridges absorb a hard hit; beyond the
        // gear limit the leg folds (up to 1.4 m), dropping that corner.
        if (!(pad_hit & (1 << k))) {
            pad_hit |= 1 << k;
            static const double uneven[4] = { 1.0, 0.55, 0.85, 0.4 };   // legs never fail evenly
            if (-vn > LAND_CRASH_V) lm.collapse[k] = fmin(1.4, ((-vn - LAND_CRASH_V) * 0.3 + 0.35) * uneven[k]);
        }
        double fn = -h * 6.0e5 - vn * 6.0e4;
        if (fn < 0) fn = 0;
        V3 vt = v3_sub(vpad, v3_scale(up, vn));
        V3 f = v3_add(v3_scale(up, fn), v3_scale(vt, -fmin(fn * 0.8, 8.0e4) / fmax(0.05, v3_len(vt)) * fmin(1.0, v3_len(vt) / 0.05)));
        F_i_contact = v3_add(F_i_contact, f);
        T_b = v3_add(T_b, v3_cross(pad_b, lm_inertial_to_body(f)));
    }
    if (pads_down && !lm.touchdown) {
        lm.touchdown = 1;
        lm.touchdown_speed = lm.hdot;
        lm.touchdown_hspeed = lm.hspeed;
        lm.touchdown_tilt = lm_tilt();
        lm.touchdown_fuel = lm.m_dps;
        lm.touchdown_t = lm.t;
        double v = -lm.hdot, hs = lm.hspeed, tl = lm.touchdown_tilt;
        int cause = v > LAND_CRASH_V ? CRASH_VERTICAL : hs > LAND_CRASH_H ? CRASH_SIDEWAYS
                  : tl > LAND_CRASH_TILT ? CRASH_TILT : CRASH_NONE;
        if (cause) {
            lm.land_result = LAND_CRASH;
            lm.crash_cause = cause;
            lm.crash_t = lm.t;
            lm.crash_speed = sqrt(v * v + hs * hs);
            engine_stop_latched = 1;
            lm.dps_on = 0;
            lm_set_attitude_mode(0);
        } else {
            lm.land_result = (v > LAND_GOOD_V || hs > LAND_GOOD_H || tl > LAND_GOOD_TILT) ? LAND_HARD : LAND_GOOD;
        }
    }
    // Tipping over after touchdown wrecks it too.
    if (lm.touchdown && lm.land_result != LAND_CRASH && lm_tilt() > LAND_TOPPLE_TILT) {
        lm.land_result = LAND_CRASH;
        lm.crash_cause = CRASH_TOPPLED;
        lm.crash_t = lm.t;
        lm.crash_speed = v3_len(lm_surface_velocity());
        engine_stop_latched = 1;
        lm.dps_on = 0;
        lm_set_attitude_mode(0);
    }

    // Translational: gravity + forces, semi-implicit Euler.
    V3 F_i = v3_add(lm_body_to_inertial(F_b), F_i_contact);
    lm.accel_ng = v3_scale(F_i, 1.0 / m);
    double rr = v3_len(lm.r);
    V3 g = v3_scale(lm.r, -MOON_MU / (rr * rr * rr));
    lm.v = v3_add(lm.v, v3_scale(v3_add(g, lm.accel_ng), dt));
    lm.r = v3_add(lm.r, v3_scale(lm.v, dt));

    // Rotational: Euler's equations, principal axes.
    V3 I = inertia();
    V3 Iw = v3(I.x * lm.w.x, I.y * lm.w.y, I.z * lm.w.z);
    V3 wd = v3_sub(T_b, v3_cross(lm.w, Iw));
    wd = v3(wd.x / I.x, wd.y / I.y, wd.z / I.z);
    if (pads_down >= 3) wd = v3_sub(wd, v3_scale(lm.w, 8.0));  // landing-gear damping
    lm.w = v3_add(lm.w, v3_scale(wd, dt));
    lm.q = q4_normalize(q4_mul(lm.q, q4_from_rotvec(v3_scale(lm.w, dt))));
}

void lm_step(double dt)
{
    rigid_body(dt);
    lm.t += dt;

    V3 up = v3_norm(lm.r);
    V3 vs = lm_surface_velocity();
    lm.altitude = v3_len(lm.r) - lm_surface_radius_below() - (-PAD_X);
    lm.hdot = v3_dot(vs, up);
    lm.hspeed = v3_len(v3_sub(vs, v3_scale(up, lm.hdot)));

    update_radar(dt);
    update_sensors_to_agc(dt);
}

void lm_run(double seconds)
{
    const double h = 0.001;
    while (seconds > 1e-9) {
        double dt = seconds < h ? seconds : h;
        lm_step(dt);
        agc_host_run(dt);
        seconds -= dt;
    }
}

// Debug: LR range beam in NB axes and its angle from local vertical, deg.
double lab_lr_debug(V3 *hb_nb)
{
    double beta = lm.lr_pos == 1 ? LR_BETA1 : LR_BETA2;
    M3 nb_from_ant = gimbal_matrix(LR_ALPHA, beta, 0);
    V3 hbeam_ant = v3_norm(v3(-0.4687018041, 0, -0.1741224271));
    *hb_nb = m3_apply(nb_from_ant, hbeam_ant);
    V3 hi = lm_body_to_inertial(*hb_nb);
    return acos(-v3_dot(hi, v3_norm(lm.r))) * 180.0 / PI_D;
}
