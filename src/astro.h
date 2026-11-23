// A suited moonwalker, posed from a few numbers and drawn from the rigid
// parts lmmodel.c builds (torso, life-support pack, helmet, arms, legs,
// overshoes). The legs reach the drawn ground by two-bone IK.
#ifndef ASTRO_H
#define ASTRO_H

#include "vmath.h"

typedef struct {
    V3 pos;                 // site frame: the ground point between the boots
    double yaw;             // facing, radians from north through east
    float lean;             // the whole body tilted forward, rad
    float crouch;           // hips lowered, m
    // Boots: where each one is relative to the hip, along the facing (m) and
    // how high off the ground (m). The IK bends the knees to put them there.
    float foot_fwd[2], foot_lift[2], foot_out[2];
    float sh_fwd[2], sh_out[2], elbow[2];   // arms (0 left, 1 right): forward swing, out to the side, elbow bend
    float kneel;            // 0..1: down on the knees (crawling out onto the porch)
    int cdr;                // the Commander's red stripes
    int no_arms;            // first person on the headset: the controllers are the hands
} AstroPose;

// first_person: the wearer's own view (no helmet or pack; the parts nearest
// the eye are cut away by the caller's near cut).
void astro_draw(const AstroPose *p, V3 cam_pos, int first_person);
// Into the current shadow pass; centre: the pass's origin (site frame).
void astro_draw_depth(const AstroPose *p, V3 centre);

// Where a glove is (site frame) and which way the hand points.
V3 astro_hand(const AstroPose *p, int side, V3 *dir);

#endif
