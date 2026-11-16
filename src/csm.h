// The Command and Service Module "Columbia" in lunar orbit: a point-mass orbit
// (the same integrator as the LM, so their relative motion is consistent) and
// a simple attitude: it keeps its docking probe toward the LM.
#ifndef CSM_H
#define CSM_H

#include "vmath.h"

typedef struct {
    int valid;
    V3 r, v;       // inertial (moon-centred), m, m/s
    Q4 q;          // body -> inertial. Body +X: out of the docking probe (CM apex)
    V3 w;          // body rates, rad/s
    int hold;      // 1: hold the current inertial attitude (station keeping, docking)
    int docked;
} Csm;

extern Csm csm;

void csm_reset(void);
void csm_set_state(V3 r, V3 v);

// Advance dt seconds. lm_r: the LM's position, for tracking.
void csm_step(double dt, V3 lm_r);

// Point the probe at `target` (inertial), keeping body +Z toward `upish` as far as possible.
void csm_point_at(V3 target, V3 upish);

// Two-body orbit propagation (RK4, 1 s steps), either direction in time.
void csm_propagate(V3 *r, V3 *v, double dt);

// Docking geometry, CSM body axes: origin at the docking ring face on the CM
// apex, +X out along the probe, +Z toward the side with the LM docking target.
#define CSM_PROBE_TIP 0.80     // probe tip, m ahead of the ring
#define CSM_R 1.956            // CM base / SM radius
#define CSM_CM_BASE (-2.95)    // CM heat shield edge
#define CSM_SM_END (-7.30)     // SM aft bulkhead
#define CSM_NOZZLE_END (-10.15)

#endif
