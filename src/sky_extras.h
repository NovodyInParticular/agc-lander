// Lunar orbit (globe) camera, and two things that are not in the flight plan.
#ifndef SKY_EXTRAS_H
#define SKY_EXTRAS_H

#include "render.h"
#include "vmath.h"

void extras_init(void);

// Per frame. flying: simulation running (not menu/paused).
void extras_update(float dt, int flying, const ViewCam *cam);
// Force the rare visitor to appear in a few seconds (testing).
void extras_force_ufo(double after_s);

// Inside the 3D scene (camera-relative): visitors and, for the globe view,
// the Moon itself with whatever lives on it.
void extras_draw_ufo(const ViewCam *cam);
void globe_draw(const ViewCam *cam, V3 sun_site, V3 earth_site);

// Globe camera.
void globe_input(int w, int h);
ViewCam globe_camera(V3 lm_site);
void globe_reset(V3 lm_site);
void globe_goto(double lat_deg, double lon_deg, double alt_m);
void globe_overlay(const ViewCam *cam, int w, int h, V3 lm_site);

#endif
