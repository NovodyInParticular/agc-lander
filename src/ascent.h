// After the landing: lift-off with Luminary's P12, orbit insertion, a
// shortcut over the three-hour rendezvous, and a hand-flown docking with the
// CSM Columbia (probe into the LM's drogue).
#ifndef ASCENT_H
#define ASCENT_H

#include "vmath.h"

enum {
    MP_DESCENT,     // flying down (or crashed)
    MP_SURFACE,     // landed; the moonwalk is possible
    MP_COUNTDOWN,   // P12 loaded, counting down to lift-off
    MP_ASCENT,      // the ascent engine is lifting the ascent stage
    MP_ORBIT,       // engine off in orbit, the CSM somewhere ahead
    MP_PROX,        // close to Columbia: the docking approach
    MP_DOCKED,      // hard docked
    MP_DOCK_FAILED, // hit Columbia too hard
};

void ascent_reset(void);
// P12 has been loaded (scenario_liftoff): the ascent part of the mission is on.
void ascent_begin(void);
int ascent_active(void);
int ascent_phase(void);

// The shortcut: the coelliptic rendezvous (CSI, CDH, TPI, braking) is skipped
// and Eagle is put 40 m from Columbia, facing it, nearly lined up.
void ascent_rendezvous(void);

// Docking contact, capture and the probe retract. Call after each slice of
// simulated time (dt seconds).
void ascent_step(double dt);

// Relative geometry for the displays: everything in LM body axes.
typedef struct {
    double range;       // probe tip to drogue mouth, m
    double axial;       // along the LM +X axis (positive: still apart), m
    double lat_y, lat_z;// probe tip off the drogue axis, m
    double rate;        // d(axial)/dt, m/s (negative: closing)
    double vy, vz;      // lateral drift, m/s
    double angle;       // between the two docking axes, rad
    double roll;        // roll misalignment about the docking axis, rad
} DockGeo;
int ascent_dock_geo(DockGeo *g);   // 0 when Columbia is not near

// Docking readout (range, rate, offsets) and messages.
void ascent_overlay(int w, int h);

// Results panels: orbit insertion, docked, failed. Returns 1 if one is due.
int ascent_results_due(void);
void ascent_draw_results(int w, int h, int client);
void ascent_hide_results(int hide);
int ascent_results_hidden(void);

// Numbers for the results panels.
typedef struct {
    int ins_have;
    double t_lift, t_cut, peri, apo, aps_left, rcs_left;   // insertion (km above the site radius)
    int dock_state;                                         // 0 approach, 1 captured, 2 hard docked, 3 failed
    double t_start, t_capture, c_speed, c_radial, c_angle, c_roll, c_rcs;
    int bounces;
    char msg[160], fail[160];
    double msg_t;
} AscentStats;
const AscentStats *ascent_stats(void);

// Co-op LM Pilot and the outside view: take the phase from the host.
void ascent_mirror(int active, int phase);

// Debug / audit: put the LM in a docked or approach pose without flying.
void ascent_debug_dock(double dist);

#endif
