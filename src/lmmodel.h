// Procedural Lunar Module model and exhaust / dust effects.
#ifndef LMMODEL_H
#define LMMODEL_H

#include "lm.h"
#include "vmath.h"

typedef struct {
    float thrust;              // N
    float jet[LM_NJETS];       // 0..1
    V3 jet_pos[LM_NJETS];      // body axes
    V3 jet_dir[LM_NJETS];      // thrust direction, body axes
    float dust;                // 0..1 blown-dust intensity
    V3 ground_point;           // camera-relative point under the engine
    V3 ground_up;              // local up (site frame)
    float crash_age;           // seconds since a crash, < 0 = none
    float crash_speed;         // impact speed, m/s
    int crash_fire;            // propellant left: hypergolic fireball
} LmFx;

void lmmodel_init(void);
int lmmodel_triangles(void);

// site_from_body: rotation body -> site frame; pos_cam_rel: CG relative to camera.
void lmmodel_draw(M3 site_from_body, V3 pos_cam_rel);
void lmmodel_draw_depth(M3 site_from_body, V3 pos_cam_rel);
void lmmodel_update_dust(float dt, float intensity);
void lmmodel_draw_effects(M3 site_from_body, V3 pos_cam_rel, const LmFx *fx);

#endif
