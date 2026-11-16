// Moonwalk. After a safe landing, with the engine off, the crew can open the
// forward hatch, back out down the ladder, walk and jump about in one-sixth
// gravity, plant the flag, and climb back in.
#ifndef EVA_H
#define EVA_H

#include "render.h"
#include "vmath.h"

enum { EVA_INSIDE, EVA_HATCH_OPENING, EVA_EGRESS, EVA_SURFACE, EVA_INGRESS, EVA_HATCH_CLOSING };

void eva_reset(void);
int eva_phase(void);

// Landed in one piece and the engine is off: the hatch may be opened.
int eva_available(void);

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

// The planted flag (drawn for every outside camera).
void eva_draw(const ViewCam *cam);

// Debugging: jump to a point in the sequence (first letter of hatch, crawl, ladder, walk, flag).
void eva_debug(const char *what, V3 lm_pos, M3 site_from_body);

// Prompts, and the fade through the hatch.
void eva_overlay(int w, int h);

#endif
