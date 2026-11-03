// Lunar Module cabin interior: 3D cockpit with live, clickable panels.
#ifndef COCKPIT_H
#define COCKPIT_H

#include "render.h"
#include "vmath.h"

void cockpit_init(void);
void cockpit_shutdown(void);

// Panel logic (timers, alarms, switch sync); call every frame in flight.
void cockpit_update(float dt);

// Per frame, before BeginDrawing: panel textures and the cabin sun shadow map.
// site_from_body: LM attitude; sun/up in the site frame.
void cockpit_prepare(M3 site_from_body, V3 sun_site, V3 up_site, float dt);

// Head position and look direction (mouse look, seat, zoom). Call once per
// frame while the cockpit view is active, before cockpit_camera().
void cockpit_input(int scene_w, int scene_h);

ViewCam cockpit_camera(V3 lm_pos_site, M3 site_from_body);

// Inside the 3D scene: draws the cabin (call after the outside world).
// lm_rel: LM centre of mass relative to the camera (site frame).
void cockpit_draw(M3 site_from_body, V3 lm_rel, int w, int h);

// 2D overlay: tooltip for the control under the mouse, view hints.
void cockpit_overlay(int scene_w, int scene_h);

// Set head yaw/pitch in degrees (0,0 = looking straight ahead).
void cockpit_set_view(double yaw_deg, double pitch_deg);
void cockpit_set_seat(int lmp);   // 0 Commander, 1 LM Pilot

// Hand controller deflections for the 3D ACA (-1..1).
void cockpit_set_hand_controls(double pitch, double yaw, double roll);

// Distance (m) inside which the exterior LM model must be hidden.
#define COCKPIT_NEAR_CUT 1.15f

#endif
