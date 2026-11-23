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
void lmmodel_draw_flag_depth(M3 site_from_flag, V3 base_cam_rel);

// Astronaut parts (posed by astro.c): own axes +X up, +Y right, +Z forward,
// the joint at the origin and limbs hanging along -X. The bands are the
// Commander's red stripes, drawn over the arm and thigh. depth: into the
// current shadow pass instead of the scene.
// AP_TORSO_LOW: the trunk below the chest, for the wearer's own view.
enum { AP_TORSO, AP_PLSS, AP_HELMET, AP_UARM, AP_FARM, AP_THIGH, AP_SHIN, AP_BOOT, AP_BAND_ARM, AP_BAND_LEG, AP_TORSO_LOW, AP_COUNT };
void lmmodel_draw_part(int part, M3 site_from_part, V3 pos_cam_rel, int depth);
void lmmodel_update_dust(float dt, float intensity);
void lmmodel_draw_effects(M3 site_from_body, V3 pos_cam_rel, const LmFx *fx);
// The descent stage hitting the ground after an abort: age (s) since impact
// (< 0 resets), speed (m/s), fire: propellant still aboard.
void lmmodel_draw_impact(V3 point_cam_rel, V3 up, float age, float speed, int fire);

#endif
