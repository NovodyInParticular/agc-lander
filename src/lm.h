// Lunar Module vehicle simulation and its interface to the AGC
// (IMU, PIPAs, DPS, RCS, landing radar, crew discretes).
#ifndef LM_H
#define LM_H

#include "vmath.h"

#define LM_NJETS 16

// Jet indices in channel order: ch05 bits 1-8 then ch06 bits 1-8.
enum {
    JET_Q4U, JET_Q4D, JET_Q3U, JET_Q3D, JET_Q2U, JET_Q2D, JET_Q1U, JET_Q1D,
    JET_Q3A, JET_Q4F, JET_Q1F, JET_Q2A, JET_Q2L, JET_Q3R, JET_Q4R, JET_Q1L,
};

typedef double (*SurfaceFn)(void *ctx, double lat, double lon);  // returns radius, m

typedef struct {
    // --- Time ---------------------------------------------------------
    double t;          // seconds of AGC time (TIME2:TIME1 / 100)
    double tephem;     // TEPHEM in seconds (moon orientation epoch)

    // --- Rigid body (basic reference = moon-centred inertial) ----------
    V3 r, v;           // m, m/s
    Q4 q;              // body (NB) -> inertial
    V3 w;              // body rates, rad/s (NB axes: X up, Y right, Z fwd)

    // --- Mass -----------------------------------------------------------
    double m_ascent;   // ascent stage dry, with the crew
    double m_aps;      // APS (ascent engine) propellant remaining
    double m_desc_dry;
    double m_dps;      // DPS propellant remaining
    double m_rcs;      // RCS propellant remaining

    // --- Engines -----------------------------------------------------------
    int engine_armed;       // ENG ARM switch: 0 off, ARM_DES, ARM_ASC
    int dps_on;             // descent engine running
    int aps_on;             // ascent engine running
    double thr_bits;        // throttle command integrator (12.53 N/bit)
    double thrust;          // actual thrust of the running engine, N (lagged)
    int thrust_aps;         // 1: lm.thrust is the ascent engine's
    double gimbal[2];       // trim gimbal pitch (about Y), roll (about Z), rad
    double flame_time;      // time since ignition, s (for visuals)

    // --- RCS --------------------------------------------------------------
    unsigned ch5, ch6;
    float jet_level[LM_NJETS];   // 0..1, smoothed for rendering

    // --- IMU -------------------------------------------------------------
    Q4 q_sm;                // stable member -> inertial
    int cdu_sent[3];        // CDU counts already delivered (X, Y, Z)
    V3 pipa_residual;       // SM-frame delta-v not yet delivered, m/s
    double needle[3];       // FDAI error needle drive counts (X, Y, Z)
    double thrust_dinc_acc; // DINC requests owed for the THRUST counter

    // --- Landing radar ---------------------------------------------------
    int lr_pos;             // 1 or 2 (actual)
    double lr_move;         // seconds left in a reposition
    double lr_range;        // last slant range, m (<0: no return)
    double lr_alt;          // altitude from range beam, m
    V3 lr_vel_ant;          // velocity along antenna axes, m/s
    int lr_range_good, lr_vel_good;
    int lr_low_scale;       // range on low scale (below ~2500 ft)

    // --- Surface --------------------------------------------------------
    SurfaceFn surface;
    void *surface_ctx;
    int probe_contact;      // LUNAR CONTACT light
    int touchdown;          // footpads on the ground
    double touchdown_speed; // vertical speed at first footpad contact, m/s
    double touchdown_hspeed;// horizontal speed at first footpad contact, m/s
    double touchdown_tilt;  // tilt from local vertical at first contact, rad
    double touchdown_fuel;  // DPS propellant left at first contact, kg
    double touchdown_t;     // lm.t of first contact
    int land_result;        // LAND_NONE / LAND_GOOD / LAND_HARD / LAND_CRASH
    int crash_cause;        // CRASH_* when land_result == LAND_CRASH
    double crash_t;         // lm.t of the crash
    double crash_speed;     // impact speed, m/s
    double collapse[4];     // footpad strut crush, m (crash damage)

    // --- Staging -----------------------------------------------------------
    int abort_stage;        // ABORT STAGE pushed: separation pyros fired
    int staged;             // the ascent stage flies alone
    V3 cg;                  // centre of mass in model axes (0 until staging)
    int ds_landed;          // the descent stage left behind rests on the ground
    V3 ds_fixed;            // ... its model origin, moon-fixed
    M3 ds_fixed_rot;        // ... moon-fixed from its body axes
    V3 ds_r, ds_v;          // ... falling (abort in flight): inertial state
    Q4 ds_q;
    int liftoff;            // the ascent stage has left the descent stage
    double liftoff_t;

    // --- Derived, for displays ---------------------------------------------
    V3 accel_ng;            // non-gravitational acceleration, inertial, m/s^2
    double altitude;        // above local terrain, m
    double hdot;            // vertical rate relative to surface, m/s
    double hspeed;          // horizontal speed relative to surface, m/s
} Lm;

extern Lm lm;

enum { ARM_OFF, ARM_DES, ARM_ASC };
enum { LAND_NONE, LAND_GOOD, LAND_HARD, LAND_CRASH };
enum { CRASH_NONE, CRASH_VERTICAL, CRASH_SIDEWAYS, CRASH_TILT, CRASH_TOPPLED };

// Landing limits (game tuning). ENGINE STOP at the contact light drops the LM
// the last 1.7 m in free fall, so a careful 1 m/s descent touches at ~2.6 m/s.
#define LAND_GOOD_V   2.7    // m/s vertical at first footpad contact
#define LAND_CRASH_V  4.0
#define LAND_GOOD_H   0.6
#define LAND_CRASH_H  2.0    // m/s horizontal
#define LAND_GOOD_TILT  (6.0 * PI_D / 180.0)
#define LAND_CRASH_TILT (20.0 * PI_D / 180.0)
#define LAND_TOPPLE_TILT (40.0 * PI_D / 180.0)

// Tilt of the LM X axis from local vertical, rad.
double lm_tilt(void);

// Reset all vehicle state; call before setting up a scenario.
void lm_reset(void);

// Connect to the AGC (installs the output hook, sets discretes).
void lm_attach_agc(void);

// Advance physics and sensors by dt seconds (call with small steps, ~1 ms).
void lm_step(double dt);

// Run AGC and LM together for `seconds` of mission time.
void lm_run(double seconds);

double lm_mass(void);

// Place the stable member and initialise CDU counts to match the current
// attitude (IMU already aligned, as after P52/P57).
void lm_align_imu(M3 refsmmat);

// Gimbal angles (OGA, IGA, MGA) in radians, from current attitude and SM.
V3 lm_gimbal_angles(void);

// Body-frame helpers.
V3 lm_body_to_inertial(V3 b);
V3 lm_inertial_to_body(V3 i);

// Surface-relative velocity (inertial frame) and local up.
V3 lm_surface_velocity(void);
double lm_surface_radius_below(void);

// Crew controls.
void lm_set_attitude_mode(int mode);   // 0 = off, 1 = attitude hold, 2 = auto
void lm_set_engine_arm(int armed);     // ARM_OFF / ARM_DES / ARM_ASC
void lm_abort_stage(void);             // ABORT STAGE: fire the stage separation pyros
void lm_engine_stop_reset(void);       // pull the ENGINE STOP button back out
// The LM stands on the surface (scenario start): no touchdown grading.
void lm_set_landed(void);
// Descent stage left behind: model origin and rotation in the moon-fixed frame.
void lm_descent_stage_pose(V3 *fixed, M3 *fixed_from_body);
void lm_engine_stop(void);             // DPS stop button (latched)
void lm_rod_click(int direction);      // P66 rate-of-descent switch: +1 up / -1 down
void lm_set_thc(int axis, int dir);    // translation: axis 0..2 (X,Y,Z), dir -1/0/+1
int lm_get_attitude_mode(void);        // last mode set with lm_set_attitude_mode
int lm_engine_stop_is_latched(void);
void lm_mirror_controls(int att_mode, int stop_latched);

#endif
