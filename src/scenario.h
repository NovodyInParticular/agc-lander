// Mission scenarios: initial AGC erasable state and vehicle state.
#ifndef SCENARIO_H
#define SCENARIO_H

#include "vmath.h"

typedef struct {
    V3 rls;                 // landing site, moon-fixed, m
    double site_lat, site_lon, site_radius;
    double tland;           // nominal landing time, AGC seconds
    double sv_age;          // age of the LGC state vector at scenario start, s
    M3 refsmmat;
} ScenarioInfo;

extern ScenarioInfo scenario_info;

// Apollo 11 LM shortly before powered descent initiation: padload, state
// vector, REFSMMAT, landing site, IMU aligned, P00 running.
void scenario_pre_pdi(void);

// Apollo 11 on the surface one minute before lift-off: P12 counting down to
// ignition, the CSM state vector in the LGC (which also places our CSM).
// keep_pose: leave the LM where it stands (after the player's own landing);
// otherwise it is put down at the LGC's landing site. lm.surface must be set.
void scenario_liftoff(int keep_pose);

#endif
