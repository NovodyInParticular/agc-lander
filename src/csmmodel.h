// Procedural model of the CSM Columbia (CSM body axes, see csm.h).
#ifndef CSMMODEL_H
#define CSMMODEL_H

#include "vmath.h"

void csmmodel_init(void);
// site_from_body: CSM body -> site frame; pos_cam_rel: its origin relative to the camera.
void csmmodel_draw(M3 site_from_body, V3 pos_cam_rel);

#endif
