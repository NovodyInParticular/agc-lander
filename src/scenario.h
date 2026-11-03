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

#endif
