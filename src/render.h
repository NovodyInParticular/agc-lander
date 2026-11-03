// Scene rendering: camera-relative 3D with logarithmic depth, embedded shaders.
#ifndef RENDER_H
#define RENDER_H

#include "raylib.h"
#include "vmath.h"

typedef struct {
    V3 pos;          // site frame (double)
    V3 fwd, upv;     // unit vectors, site frame
    double fov_y;    // radians
} ViewCam;

void render_init(void);
void render_shutdown(void);

// Common uniforms for all scene shaders; call once per frame inside 3D mode.
void render_begin_scene(const ViewCam *cam, int screen_w, int screen_h);
void render_end_scene(void);

Shader *render_terrain_shader(void);
Shader *render_lm_shader(void);
Shader *render_fx_shader(void);

// Sky: stars, Sun glare, Earth. Directions in the site frame.
void render_sky(const ViewCam *cam, V3 sun_dir, V3 earth_dir, V3 earth_sun_dir);

// Exposure control (1 = nominal sunlit surface), film look 0..1.
void render_set_exposure(float e);
void render_set_film(float f);
void render_set_output(int w, int h);
void render_set_output_fbo(unsigned int fbo);   // 0 = window

// 3D projection with log depth inside the HDR target.
void render_scene_3d_begin(int w, int h);
void render_scene_3d_end(void);
void render_set_scene_uniforms(V3 sun, V3 cam_mod, V3 up_dir, float ground_albedo, V3 dust_center, float dust);

// Same, with a custom near plane (cockpit interior).
void render_scene_3d_begin_near(int w, int h, double near_plane);

// LM-shader extras: discard fragments nearer than near_cut (m, 0 = off),
// scale sunlight, up to 4 point lamps (camera-relative positions, rgb power).
void render_set_lm_extras(float near_cut, float sun_gain, const V3 *lamp_pos, const V3 *lamp_col, int n);
float render_exposure(void);
float render_log_far(void);   // logFar uniform value for log-depth shaders

// Sun shadow map around a camera-relative centre.
void render_shadow_begin(V3 center_cam_rel, V3 sun, float radius);
void render_shadow_end(void);
Shader *render_depth_shader(void);

#endif
