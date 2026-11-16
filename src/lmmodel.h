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
    int aps;                   // the thrust is the ascent engine's (from the stage interface)
    float burst_age;           // seconds since lift-off: foil blown off the descent stage (< 0 none)
    V3 burst_point;            // camera-relative centre of the descent stage's top deck
    V3 burst_up;               // its +X, site frame
} LmFx;

void lmmodel_init(void);
int lmmodel_triangles(void);

// site_from_body: rotation body -> site frame; pos_cam_rel: CG relative to camera.
void lmmodel_draw(M3 site_from_body, V3 pos_cam_rel);
void lmmodel_draw_depth(M3 site_from_body, V3 pos_cam_rel);
// One stage only (after staging): 0 descent, 1 ascent.
void lmmodel_draw_stage(int ascent, M3 site_from_body, V3 pos_cam_rel);
void lmmodel_draw_stage_depth(int ascent, M3 site_from_body, V3 pos_cam_rel);
// The planted flag: site_from_flag takes its axes (+X up the pole, +Z along
// the cloth) to the site frame; base_cam_rel is the foot of the pole.
void lmmodel_draw_flag(M3 site_from_flag, V3 base_cam_rel);
void lmmodel_update_dust(float dt, float intensity);
void lmmodel_draw_effects(M3 site_from_body, V3 pos_cam_rel, const LmFx *fx);

#endif
