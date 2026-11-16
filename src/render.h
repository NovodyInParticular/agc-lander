// Scene rendering: camera-relative 3D with logarithmic depth, embedded shaders.
#ifndef RENDER_H
#define RENDER_H

#include "raylib.h"
#include "vmath.h"

typedef struct {
    V3 pos;          // site frame (double)
    V3 fwd, upv;     // unit vectors, site frame
    double fov_y;    // radians
    // Off-centre frustum (one eye of a headset): tangents of the angles to the
    // left, right, bottom and top edges (left and bottom negative). asym = 0: symmetric, from fov_y.
    int asym;
    double tan_l, tan_r, tan_d, tan_u;
} ViewCam;

void render_init(void);
void render_shutdown(void);

// Common uniforms for all scene shaders; call once per frame inside 3D mode.
void render_begin_scene(const ViewCam *cam, int screen_w, int screen_h);
void render_end_scene(void);

// Scene output mode; set before render_init(). 0 = HDR target with bloom,
// tonemap and edge smoothing (PC). 1 = straight into a linear target that the
// display encodes (Quest swapchain). 2 = straight into a plain 8-bit target.
void render_set_direct(int mode);
int render_direct(void);
// Direct modes: the framebuffer (with depth) and viewport origin the scene goes to.
void render_set_direct_target(unsigned int fbo, int x, int y);
// Compile a shader pair: adds the GLSL version line and sceneOut().
Shader render_load_shader(const char *vs, const char *fs);

Shader *render_terrain_shader(void);
Shader *render_lm_shader(void);
// Material setup for meshes drawn with the LM shader: the shader plus its
// surface maps (foil, panel, blanket, rock).
void render_lm_material(Material *m);
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
// Same shadow map seen from another eye point: move its centre (camera-relative).
void render_set_shadow_center(V3 center_cam_rel);
Shader *render_depth_shader(void);

#endif
