// Headset: the controllers drawn in the player's hands.
#ifndef VRHANDS_H
#define VRHANDS_H

#include "render.h"
#include "vr.h"

void vrhands_init(void);
// One eye: base = the seat's camera (tracking space mapped onto it), eye_pos = this eye (world).
void vrhands_draw(const ViewCam *base, V3 eye_pos, const VrInput *in, double t);

#endif
