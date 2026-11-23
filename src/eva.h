// Moonwalk. After a safe landing, with the engine off, the crew can open the
// forward hatch, back out down the ladder, walk and jump about in one-sixth
// gravity, plant the flag, and climb back in.
//
// Two crewmen: the Commander (0) and the LM Pilot (1). The local player walks
// one of them (the LM Pilot when joined to a co-op game); the other is the
// co-op partner, or, playing alone, a computer-run LM Pilot who follows the
// Commander out, keeps him company and climbs back in first. Both leave boot
// prints, cast shadows and bump into boulders.
#ifndef EVA_H
#define EVA_H

#include <stdint.h>

#include "render.h"
#include "vmath.h"

enum { EVA_INSIDE, EVA_HATCH_OPENING, EVA_EGRESS, EVA_SURFACE, EVA_INGRESS, EVA_HATCH_CLOSING };

void eva_reset(void);
// Which crewman this game walks: 0 Commander, 1 LM Pilot (co-op client), -1 none (spectator).
void eva_set_role(int who);
// The local crewman's phase (EVA_INSIDE for a spectator).
int eva_phase(void);
// Nobody outside or on the way out or in.
int eva_all_inside(void);

// Landed in one piece and the engine is off: the hatch may be opened.
int eva_available(void);
// The flag stands on the surface.
int eva_flag_planted(void);

// The hatch key: opens the hatch from inside; outside, climbs back in at the ladder.
void eva_key(void);

// Once per frame: timing, input and walking. lm_pos / R: the LM in the site frame.
void eva_update(float dt, V3 lm_pos, M3 site_from_body);

// 1 while the view is the astronaut's own, outside the cabin.
int eva_outside(void);
// 1 while the cabin view is taken over (kneeling at the hatch).
int eva_at_hatch(void);
ViewCam eva_camera(void);

// Hatch door, 0 closed .. 1 open, and how far the crewman has moved to the
// hatch (0 standing at his station .. 1 kneeling at it .. 2 in the opening).
float eva_hatch(void);
float eva_kneel(void);

// Headset: the view stays level and the head does the looking; walking
// "forward" follows the head. Call every frame with the head's yaw relative
// to the body (radians, positive to the right). eva_turn: turn the body.
void eva_set_vr(double head_yaw_rad);
void eva_turn(double rad);

// Headset hands, every frame: where the controllers are (site frame) and
// whether each grip is squeezed. Squeeze the right grip to take the flag,
// let go to plant it where the hand is; grab the ladder to climb in.
// Returns EVA_VR_* events (for the controllers' rumble).
enum { EVA_VR_TOOK_FLAG = 1, EVA_VR_PLANTED = 2, EVA_VR_CLIMB = 4, EVA_VR_HATCH = 8 };
int eva_vr_hands(const V3 hand_site[2], const int grip[2]);
// In the cabin: hands in LM body axes; a grip on the hatch handle opens it.
int eva_vr_cabin_hands(const V3 hand_body[2], const int grip[2]);

// Lift-off: the ascent engine's blast from the descent stage's top deck
// (site frame), `age` seconds after the stages parted on the pad. The flag
// goes over if it was planted close by, as Apollo 11's did.
void eva_liftoff_blast(V3 deck_site, double age);

// The crew, the flag and the boot prints (drawn for every outside camera).
void eva_draw(const ViewCam *cam);
// The crew's own shadow: 1 if anyone (or the flag) needs one; centre (site
// frame) and radius of what the map must cover, and how long the shadows are.
int eva_shadow_focus(V3 *centre, float *radius);
void eva_draw_depth(V3 centre);

// Debugging: jump to a point in the sequence (first letter of hatch, crawl, ladder, walk, flag).
void eva_debug(const char *what, V3 lm_pos, M3 site_from_body);

// Prompts, and the fade through the hatch.
void eva_overlay(int w, int h);

// --- Network (co-op and the second-monitor view) ----------------------------------

typedef struct {
    int32_t phase, act, grounded;
    float t_phase, act_t, yaw, pitch, stride, speed, land_k, vy;
    double px, pz, alt;
} EvaNetWalker;

typedef struct EvaNet {
    EvaNetWalker w[2];
    int32_t flag_on, carry[2];
    double flag_x, flag_z;
    float flag_yaw;
} EvaNet;

// The whole moonwalk as this game sees it (host -> joiner and spectator).
void eva_net_pack(EvaNet *n);
// Joiner / spectator: take the other crewman (both, for a spectator) and the flag.
void eva_net_apply(const EvaNet *n);
// Joiner -> host: its own crewman and any flag it planted; the host takes them in.
void eva_net_pack_mine(EvaNet *n);
void eva_net_apply_joiner(const EvaNet *n);

#endif
