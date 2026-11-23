// Boot prints in the regolith, left by the moonwalkers. They stay for the
// rest of the flight (a new flight clears them) and are drawn as decals that
// darken and shade the ground already drawn (render.c decal shader).
#ifndef FOOTPRINTS_H
#define FOOTPRINTS_H

#include "vmath.h"

void footprints_reset(void);

// One print. x, z: terrain plane (m east, m south of the site) of the boot's
// centre; yaw: its heading (radians from north through east); depth 0..1
// (1 = a firm step, about 3.5 cm).
void footprints_add(double x, double z, double yaw, float depth);

int footprints_count(void);

// After the terrain, before anything that stands on it: cam_pos in the site frame.
void footprints_draw(V3 cam_pos);

#endif
