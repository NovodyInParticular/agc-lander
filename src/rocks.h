// Boulder field around the landing site.
#ifndef ROCKS_H
#define ROCKS_H

#include "vmath.h"

void rocks_init(void);            // after terrain_init (needs heights and the Sun)
void rocks_draw(V3 camera_site);  // inside the 3D scene, LM shader uniforms set

#endif
