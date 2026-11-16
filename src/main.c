// AGC Lander: Apollo 11's Luminary099 flies the Lunar Module to the surface
// of the Moon, on an emulated AGC inside this program.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
__declspec(dllimport) unsigned long __stdcall GetModuleFileNameA(void *module, char *name, unsigned long size);
#else
#include <unistd.h>
#endif

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "input.h"
#include "vr.h"

// AGC_PERF=1: print per-section frame timings once a second (GPU-synced).
static int perf_on;
static double perf_t[8], perf_n;
#ifdef AGC_VR
static double perf_mark(void) { return GetTime(); }
#else
extern void *glfwGetProcAddress(const char *);
static void (*perf_finish)(void);
static double perf_mark(void) { if (perf_on) { rlDrawRenderBatchActive(); if (!perf_finish) perf_finish = (void (*)(void))glfwGetProcAddress("glFinish"); perf_finish(); } return GetTime(); }
#endif

#include "agc_host.h"
#include "agc_mem.h"
#include "ascent.h"
#include "assets.h"
#include "cockpit.h"
#include "sky_extras.h"
#include "controls.h"
#include "coop.h"
#include "crew.h"
#include "csm.h"
#include "csmmodel.h"
#include "eva.h"
#include "dsky.h"
#include "hints.h"
#include "hud.h"
#include "lm.h"
#include "lmmodel.h"
#include "net.h"
#include "moon.h"
#include "sound.h"
#include "render.h"
#include "rocks.h"
#include "scenario.h"
#include "terrain.h"

V3 lm_jet_position(int j);
V3 lm_jet_direction(int j);
double lab_lr_debug(V3 *hb);

#define DEG (PI_D / 180.0)

static double orbit_yaw_opt = 1e9, orbit_pitch_opt = 1e9, orbit_dist_opt = 1e9;

// ---------------------------------------------------------------------------
// Options (also used for automated screenshots).

typedef struct {
    const char *shot;      // screenshot path
    double after;          // seconds of wall time before the shot
    int start;             // scenario index, -1 = menu
    int camera;
    double warp;
    double fast_to_alt;    // fast-forward until altitude below this (m), 0 = off
    int film;
    double ufo;            // force the visitor after N seconds (0 = off)
    int base;              // start the orbit camera over the far side
    double look_yaw, look_pitch;   // cockpit head angles (deg), for screenshots
    int look_set;
    int spectator;         // outside-view instance fed by another instance
    int monitor;           // monitor to open on (-1 = current)
    int windowed;          // don't go fullscreen
    int dual;              // start the second-monitor outside view
    int demo;              // hands-off demo: auto crew lands from high gate, cameras cycle
    int host;              // two players: host as Commander
    const char *join;      // two players: join this host as LM Pilot
    int crash;             // test: cut the engine when the flight starts (screenshots)
    const char *audit;     // directory: render the fixed set of exterior + cockpit views there, then quit
} Options;

static void parse_args(int argc, char **argv, Options *o)
{
    memset(o, 0, sizeof *o);
    o->after = 8;
    o->start = -1;
    o->warp = 1;
    o->camera = 0;
    o->monitor = -1;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--spectator")) o->spectator = 1;
        else if (!strcmp(argv[i], "--windowed")) o->windowed = 1;
        else if (!strcmp(argv[i], "--dual")) o->dual = 1;
        else if (!strcmp(argv[i], "--demo")) { o->demo = 1; o->start = 2; }
        else if (!strcmp(argv[i], "--host")) o->host = 1;
        else if (!strcmp(argv[i], "--crash")) o->crash = 1;
    }
    for (int i = 1; i + 1 < argc; i++) {
        if (!strcmp(argv[i], "--shot")) o->shot = argv[++i];
        else if (!strcmp(argv[i], "--after")) o->after = atof(argv[++i]);
        else if (!strcmp(argv[i], "--audit")) o->audit = argv[++i];
        else if (!strcmp(argv[i], "--start")) o->start = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--camera")) o->camera = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--warp")) o->warp = atof(argv[++i]);
        else if (!strcmp(argv[i], "--alt")) o->fast_to_alt = atof(argv[++i]);
        else if (!strcmp(argv[i], "--film")) o->film = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--yaw")) orbit_yaw_opt = atof(argv[++i]);
        else if (!strcmp(argv[i], "--pitch")) orbit_pitch_opt = atof(argv[++i]);
        else if (!strcmp(argv[i], "--dist")) orbit_dist_opt = atof(argv[++i]);
        else if (!strcmp(argv[i], "--ufo")) o->ufo = atof(argv[++i]);
        else if (!strcmp(argv[i], "--monitor")) o->monitor = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--join")) o->join = argv[++i];
        else if (!strcmp(argv[i], "--base")) o->base = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--look")) { o->look_set = 1; sscanf(argv[++i], "%lf,%lf", &o->look_yaw, &o->look_pitch); }
    }
}

// ---------------------------------------------------------------------------
// World.

static V3 sun_site, earth_site;
static int terrain_ready;

static double surface_cb(void *ctx, double lat, double lon)
{
    (void)ctx;
    return terrain_radius_at(lat, lon);
}

// Surface audit (--audit): the LM is shown parked upright on the landing site,
// facing west as Eagle did, whatever the simulation is doing.
static int pose_override;
static V3 pose_pos;
static M3 pose_rot;

static void pose_park(void)
{
    pose_pos = terrain_plane_to_site(0, 0, terrain_height(0, 0, 0) + 2.75);
    // Body X (up) -> site +Y, body Y (right) -> north (-Z), body Z (forward) -> west (-X).
    pose_rot = m3_rows(v3(0, 0, -1), v3(1, 0, 0), v3(0, -1, 0));
    pose_override = 1;
}

static V3 lm_site_pos(void)
{
    if (pose_override) return pose_pos;
    V3 fixed = moon_inertial_to_fixed(lm.r, lm.t + lm.tephem);
    return terrain_fixed_to_site(fixed);
}

static M3 lm_site_rot(void)
{
    if (pose_override) return pose_rot;
    M3 s = terrain_site_from_fixed();
    M3 m = moon_matrix(lm.t + lm.tephem);
    return m3_mul(s, m3_mul(m, q4_to_m3(lm.q)));
}

static V3 local_up_at(V3 site)
{
    double r0 = terrain_ref_radius();
    return v3_norm(v3_add(site, v3(0, r0, 0)));
}

static void sim_advance(double seconds)
{
    // Run in 0.1 s slices so the crew can react between them.
    while (seconds > 1e-9) {
        double s = seconds > 0.1 ? 0.1 : seconds;
        lm_run(s);
        crew_step();
        ascent_step(s);
        seconds -= s;
    }
}

static void start_world(void)
{
    agc_host_init();
    lm.surface = NULL;
    lm_reset();
    lm_attach_agc();
    crew_reset();
    eva_reset();
    ascent_reset();
    scenario_pre_pdi();
    if (!terrain_ready) {
        terrain_set_sun(sun_site);
        terrain_init(scenario_info.rls);
        terrain_ready = 1;
        rocks_init();
    }
    lm.surface = surface_cb;
}

// The ascent from where the LM stands now (after a landing) or from the
// LGC's landing site (the LUNAR LIFT-OFF scenario): P12 one minute to go.
static void start_ascent(int keep_pose)
{
    scenario_liftoff(keep_pose);
    int ac = crew.auto_crew;
    crew_reset();
    crew.auto_crew = ac;
    crew.landed = 1;
    ascent_begin();
}

// Where the CSM and the descent stage left behind are, in the site frame.
static V3 csm_site_pos(void)
{
    return terrain_fixed_to_site(moon_inertial_to_fixed(csm.r, lm.t + lm.tephem));
}

static M3 csm_site_rot(void)
{
    return m3_mul(terrain_site_from_fixed(), m3_mul(moon_matrix(lm.t + lm.tephem), q4_to_m3(csm.q)));
}

static void ds_site_pose(V3 *pos, M3 *rot)
{
    V3 f;
    M3 r;
    lm_descent_stage_pose(&f, &r);
    *pos = terrain_fixed_to_site(f);
    *rot = m3_mul(terrain_site_from_fixed(), r);
}

// ---------------------------------------------------------------------------
// Cameras.

enum { CAM_CHASE, CAM_CDR, CAM_DAC, CAM_TOWER, CAM_CINEMA, CAM_GLOBE, CAM_DOCK, CAM_COUNT };
enum { CAM_EVA = CAM_COUNT };   // the astronaut's own eyes on the surface: not in the camera cycle
static const char *cam_names[CAM_COUNT] = { "CHASE", "COCKPIT", "16MM DAC (LMP WINDOW)", "TOWER", "CINEMATIC", "LUNAR ORBIT",
                                            "DOCKING WINDOW (COAS)" };
static double orbit_yaw = -0.9, orbit_pitch = 0.16, orbit_dist = 24;
static int focus_override;   // audit: orbit the chase camera about focus_site instead of the LM
static V3 focus_site;

// Audit after lift-off: the descent stage stays parked, the ascent stage hangs
// 9 m above it (or, for Columbia, the CSM sits 30 m away for a walk-round).
static V3 site_to_fixed(V3 p)
{
    M3 r = terrain_site_from_fixed();
    return v3_add(m3_apply(m3_transpose(r), p), v3_scale(m3_row(r, 1), terrain_ref_radius()));
}

static void audit_after_liftoff(int show_csm, int look_at_ds)
{
    static int staged_done;
    double t = lm.t + lm.tephem;
    M3 fixed_from_site = m3_transpose(terrain_site_from_fixed());
    if (!staged_done) {
        pose_park();
        lm.staged = 1;
        lm.ds_landed = 1;
        lm.liftoff = 1;
        lm.liftoff_t = lm.t - 100;
        lm.ds_fixed = site_to_fixed(pose_pos);
        lm.ds_fixed_rot = m3_mul(fixed_from_site, pose_rot);
        pose_pos = v3_add(pose_pos, v3_scale(local_up_at(pose_pos), 9.0));
        staged_done = 1;
    }
    focus_override = 0;
    csm.valid = 0;
    if (look_at_ds) {
        focus_override = 1;
        focus_site = terrain_fixed_to_site(lm.ds_fixed);
    }
    if (show_csm) {
        // Columbia: long axis east-west, probe toward the LM (west), +Z up.
        V3 up = local_up_at(pose_pos);
        V3 centre = v3_add(pose_pos, v3_add(v3(30, 0, 0), v3_scale(up, 4)));
        V3 x = v3(-1, 0, 0), z = up, y = v3_cross(z, x);
        M3 site_rot;   // columns: body axes in site coordinates
        site_rot.m[0][0] = x.x; site_rot.m[0][1] = y.x; site_rot.m[0][2] = z.x;
        site_rot.m[1][0] = x.y; site_rot.m[1][1] = y.y; site_rot.m[1][2] = z.y;
        site_rot.m[2][0] = x.z; site_rot.m[2][1] = y.z; site_rot.m[2][2] = z.z;
        V3 origin = v3_sub(centre, m3_apply(site_rot, v3(-4.5, 0, 0)));   // docking ring end
        csm.valid = 1;
        csm.r = moon_fixed_to_inertial(site_to_fixed(origin), t);
        csm.q = m3_to_q4(m3_mul(m3_transpose(moon_matrix(t)), m3_mul(fixed_from_site, site_rot)));
        focus_override = 1;
        focus_site = centre;
    }
}


static ViewCam make_camera(int mode, V3 lm_pos, M3 R, double t_wall)
{
    ViewCam c = { 0 };
    c.fov_y = 50 * DEG;
    V3 up = local_up_at(lm_pos);
    V3 east = v3_norm(v3_cross(v3(0, 0, 1), up));   // site +Z is south: z x up = east-ish
    if (v3_len(east) < 0.5) east = v3(1, 0, 0);
    V3 north = v3_cross(up, east);
    switch (mode) {
    case CAM_CDR:
        return cockpit_camera(lm_pos, R);
    case CAM_EVA:
        return eva_camera();
    case CAM_GLOBE:
        return globe_camera(lm_pos);
    case CAM_DOCK:
        // The Commander's overhead docking window, looking out along +X through
        // the COAS; the top of the screen is the LM's front (+Z).
        c.pos = v3_add(lm_pos, m3_apply(R, v3(3.30, -0.66, -0.20)));
        c.fwd = v3_norm(m3_apply(R, v3(1, 0, 0)));
        c.upv = v3_norm(m3_apply(R, v3(0, 0, 1)));
        c.fov_y = 46 * DEG;
        break;
    case CAM_DAC: {
        int dac = mode == CAM_DAC;
        V3 eye_b = dac ? v3(1.75, 0.55, 1.05) : v3(1.80, -0.55, 1.10);
        double down = dac ? 38 * DEG : 22 * DEG, side = dac ? 25 * DEG : -6 * DEG;
        V3 dir_b = v3_norm(v3(-sin(down), sin(side) * cos(down), cos(side) * cos(down)));
        c.pos = v3_add(lm_pos, m3_apply(R, eye_b));
        c.fwd = v3_norm(m3_apply(R, dir_b));
        c.upv = v3_norm(m3_apply(R, v3(1, 0, 0)));
        c.fov_y = dac ? 42 * DEG : 55 * DEG;
        break;
    }
    case CAM_TOWER: {
        V3 base = terrain_plane_to_site(-140, 190, terrain_height(-140, 190, 0) + 1.8);
        c.pos = base;
        V3 d = v3_sub(lm_pos, base);
        double dist = v3_len(d);
        c.fwd = v3_norm(d);
        c.upv = local_up_at(base);
        // Telephoto: keep the LM a constant size on screen.
        double fov = 2 * atan(18.0 / fmax(dist, 1));
        c.fov_y = fmax(1.0 * DEG, fmin(60 * DEG, fov));
        break;
    }
    case CAM_CINEMA:
        orbit_yaw = t_wall * 0.05 - 2.0;
        orbit_pitch = 0.12 + 0.08 * sin(t_wall * 0.07);
        /* fall through */
    default: {
        double d = orbit_dist;
        V3 focus = focus_override ? focus_site : v3_add(lm_pos, v3_scale(up, 0.5));
        if (csm.valid && ascent_active() && ascent_phase() >= MP_PROX && !focus_override && !getenv("AGC_LMFOCUS")) {
            // Docking: frame Eagle and Columbia together.
            V3 cp = csm_site_pos();
            double sep = v3_len(v3_sub(cp, lm_pos));
            if (sep < 300) {
                focus = v3_scale(v3_add(lm_pos, v3_add(cp, m3_apply(csm_site_rot(), v3(-3.5, 0, 0)))), 0.5);
                d = fmax(d, sep * 1.1 + 14);
            }
        }
        V3 off = v3_add(v3_scale(up, sin(orbit_pitch) * d),
                        v3_scale(v3_add(v3_scale(east, cos(orbit_yaw)), v3_scale(north, sin(orbit_yaw))), cos(orbit_pitch) * d));
        c.pos = v3_add(focus, off);
        c.fwd = v3_norm(v3_scale(off, -1));
        c.upv = up;
        break;
    }
    }
    // Orthonormalise up.
    V3 right = v3_norm(v3_cross(c.fwd, c.upv));
    c.upv = v3_cross(right, c.fwd);
    return c;
}

// ---------------------------------------------------------------------------
// Rendering of the 3D scene.

static V3 earth_site_g;

// Once per frame, before any eye is drawn: terrain level of detail for the
// viewpoint, the LM's sun shadow map, the dust simulation.
static float world_dust;
static V3 shadow_centre;   // site frame: where the shadow map is centred this frame
static void world_prepare(const ViewCam *cam, int h, int camera)
{
    if (camera == CAM_GLOBE) return;
    V3 lm_pos = lm_site_pos();
    M3 R = lm_site_rot();
    terrain_update(cam->pos, cam->fov_y, h);

    // LM shadow map (LM at the origin of the light space). After staging it
    // stays on the descent stage while the ascent stage is close above it.
    if (lm.staged) {
        V3 dp;
        M3 dr;
        ds_site_pose(&dp, &dr);
        V3 centre = v3_len(v3_sub(lm_pos, dp)) < 14 ? dp : lm_pos;
        render_shadow_begin(v3_sub(centre, cam->pos), sun_site, 8.0f);
        lmmodel_draw_stage_depth(0, dr, v3_sub(dp, centre));
        lmmodel_draw_stage_depth(1, R, v3_sub(lm_pos, centre));
        render_shadow_end();
        render_set_shadow_center(v3_sub(centre, cam->pos));
    } else {
        render_shadow_begin(v3_sub(lm_pos, cam->pos), sun_site, 8.0f);
        lmmodel_draw_depth(R, v3(0, 0, 0));
        render_shadow_end();
    }
    shadow_centre = lm_pos;
    if (lm.staged) {
        V3 dp;
        M3 dr;
        ds_site_pose(&dp, &dr);
        if (v3_len(v3_sub(lm_pos, dp)) < 14) shadow_centre = dp;
    }

    // Dust kicked up by the descent engine.
    float dust = 0;
    if (!pose_override && !lm.thrust_aps && lm.thrust > 500 && lm.altitude < 70)
        dust = (float)fmin(1.0, pow(1.0 - lm.altitude / 70.0, 1.3) * lm.thrust / 11000.0);
    float crash_age = lm.land_result == LAND_CRASH ? (float)(lm.t - lm.crash_t) : -1.0f;
    if (crash_age >= 0 && crash_age < 8) {
        // Impact blows a ring of dust out that settles over a few seconds.
        float burst = (float)fmin(1.0, 0.35 + lm.crash_speed / 8.0) * expf(-crash_age / 1.8f);
        if (burst > dust) dust = burst;
    }
    lmmodel_update_dust(GetFrameTime(), dust);
    world_dust = dust;
}

// One view of the world (the screen, or one eye).
static void world_draw(const ViewCam *cam, int w, int h, float film, int camera)
{
    V3 lm_pos = lm_site_pos();
    M3 R = lm_site_rot();
    V3 rel = v3_sub(lm_pos, cam->pos);

    if (camera == CAM_GLOBE) {
        render_set_film(film);
        render_begin_scene(cam, w, h);
        render_sky(cam, sun_site, earth_site, sun_site);
        render_scene_3d_begin(w, h);
        render_set_scene_uniforms(sun_site, v3(0, 0, 0), local_up_at(cam->pos), 0.11f, v3(0, 0, 0), 0);
        globe_draw(cam, sun_site, earth_site);
        extras_draw_ufo(cam);
        render_scene_3d_end();
        render_end_scene();
        return;
    }
    render_set_shadow_center(v3_sub(shadow_centre, cam->pos));   // the shadow map was made once; this view sees it from its own eye point

    render_set_film(film);
    render_begin_scene(cam, w, h);
    render_sky(cam, sun_site, earth_site, sun_site);
    render_scene_3d_begin(w, h);

    V3 up = local_up_at(lm_pos);
    double ground_dist = lm.altitude + 2.75;
    V3 ground = v3_sub(v3_add(lm_pos, v3_scale(up, -ground_dist)), cam->pos);
    float dust = world_dust;
    float crash_age = lm.land_result == LAND_CRASH ? (float)(lm.t - lm.crash_t) : -1.0f;
    V3 cam_mod = v3(fmod(cam->pos.x, 4096.0), fmod(cam->pos.y, 4096.0), fmod(cam->pos.z, 4096.0));
    render_set_scene_uniforms(sun_site, cam_mod, local_up_at(cam->pos), 0.11f, ground, dust);

    globe_draw_backdrop(cam, sun_site, earth_site);
    render_set_scene_uniforms(sun_site, cam_mod, local_up_at(cam->pos), 0.11f, ground, dust);
    if (getenv("AGC_NODEPTH")) rlDisableDepthTest();
    terrain_draw(cam->pos, render_terrain_shader());
    rlEnableDepthTest();
    rocks_draw(cam->pos);
    if (camera == CAM_CDR || camera == CAM_DOCK) render_set_lm_extras(COCKPIT_NEAR_CUT, 1.0f, NULL, NULL, 0);
    if (lm.staged) {
        lmmodel_draw_stage(1, R, rel);
        render_set_lm_extras(0.0f, 1.0f, NULL, NULL, 0);
        V3 dp;
        M3 dr;
        ds_site_pose(&dp, &dr);
        lmmodel_draw_stage(0, dr, v3_sub(dp, cam->pos));
    } else {
        lmmodel_draw(R, rel);
    }
    render_set_lm_extras(0.0f, 1.0f, NULL, NULL, 0);
    if (csm.valid) {
        V3 cp = v3_sub(csm_site_pos(), cam->pos);
        if (v3_len(cp) < 400000) csmmodel_draw(csm_site_rot(), cp);
    }
    eva_draw(cam);

    LmFx fx = { 0 };
    fx.thrust = pose_override ? 0 : (float)lm.thrust;
    fx.aps = lm.thrust_aps;
    if (lm.staged && lm.ds_landed && lm.liftoff) {
        // Fire in the hole: the ascent engine blasts the descent stage's top deck.
        V3 dp;
        M3 dr;
        ds_site_pose(&dp, &dr);
        fx.burst_age = (float)(lm.t - lm.liftoff_t);
        fx.burst_point = v3_sub(v3_add(dp, m3_apply(dr, v3(0.40, 0, 0))), cam->pos);
        fx.burst_up = m3_apply(dr, v3(1, 0, 0));
    } else {
        fx.burst_age = -1;
    }
    for (int j = 0; j < LM_NJETS; j++) {
        fx.jet[j] = pose_override ? 0 : lm.jet_level[j];
        fx.jet_pos[j] = lm_jet_position(j);
        fx.jet_dir[j] = lm_jet_direction(j);
    }
    fx.dust = dust;
    fx.ground_point = ground;
    fx.ground_up = up;
    fx.crash_age = crash_age;
    fx.crash_speed = (float)lm.crash_speed;
    fx.crash_fire = lm.touchdown_fuel > 30 && lm.crash_speed > 5.0;
    lmmodel_draw_effects(R, rel, &fx);
    extras_draw_ufo(cam);

    render_scene_3d_end();
    if (camera == CAM_CDR) cockpit_draw(R, rel, w, h);
    render_end_scene();
}

// The docking window: its frame, and the COAS reticle projected at infinity
// along +X (screen centre).
static void draw_coas(int w, int h)
{
    float cx = w / 2.0f, cy = h / 2.0f, s = h / 1080.0f;
    // Window frame: dark cabin all round a rounded opening.
    float ww = h * 0.86f, wh = h * 0.78f;
    Rectangle win = { cx - ww / 2, cy - wh / 2, ww, wh };
    Color cab = { 12, 13, 15, 255 };
    DrawRectangle(0, 0, w, (int)win.y, cab);
    DrawRectangle(0, (int)(win.y + win.height), w, h - (int)(win.y + win.height), cab);
    DrawRectangle(0, (int)win.y, (int)win.x, (int)win.height + 1, cab);
    DrawRectangle((int)(win.x + win.width), (int)win.y, w - (int)(win.x + win.width), (int)win.height + 1, cab);
    DrawRectangleRoundedLinesEx(win, 0.08f, 8, 10 * s, (Color){ 40, 42, 46, 255 });
    DrawRectangleRoundedLinesEx(win, 0.08f, 8, 2 * s, (Color){ 90, 92, 98, 255 });
    // COAS: a collimated cross with range marks, in the sight's own amber.
    Color rc = { 255, 170, 60, 200 };
    float L = h * 0.30f;
    DrawLineEx((Vector2){ cx - L, cy }, (Vector2){ cx - 14 * s, cy }, 2.0f * s, rc);
    DrawLineEx((Vector2){ cx + 14 * s, cy }, (Vector2){ cx + L, cy }, 2.0f * s, rc);
    DrawLineEx((Vector2){ cx, cy - L }, (Vector2){ cx, cy - 14 * s }, 2.0f * s, rc);
    DrawLineEx((Vector2){ cx, cy + 14 * s }, (Vector2){ cx, cy + L }, 2.0f * s, rc);
    for (int k = 1; k <= 5; k++) {
        float d = L * k / 5.5f, t = (k == 5 ? 12 : 6) * s;
        DrawLineEx((Vector2){ cx + d, cy - t }, (Vector2){ cx + d, cy + t }, 1.5f * s, rc);
        DrawLineEx((Vector2){ cx - d, cy - t }, (Vector2){ cx - d, cy + t }, 1.5f * s, rc);
        DrawLineEx((Vector2){ cx - t, cy + d }, (Vector2){ cx + t, cy + d }, 1.5f * s, rc);
        DrawLineEx((Vector2){ cx - t, cy - d }, (Vector2){ cx + t, cy - d }, 1.5f * s, rc);
    }
    DrawCircleLinesV((Vector2){ cx, cy }, 9 * s, rc);
    hud_text(font_bold, "COAS", win.x + 18 * s, win.y + win.height - 34 * s, 18 * s, (Color){ 255, 170, 60, 160 });
}

#ifndef AGC_VR
static void draw_world(const ViewCam *cam, int w, int h, float film, int camera)
{
    world_prepare(cam, h, camera);
    world_draw(cam, w, h, film, camera);
}
#endif

// Commander's window: Landing Point Designator reticle (angles from N64).
static void draw_lpd(int w, int h, double fov_y)
{
    float cx = w / 2.0f, cy = h / 2.0f;
    float ppd = (float)(h / (fov_y / DEG));
    // The camera looks 22 deg below the window's reference line; LPD angle is
    // measured down from the reference, so 22 deg sits at screen centre.
    Color c = { 255, 120, 40, 200 };
    for (int a = 25; a <= 75; a += 5) {
        float y = cy + (a - 22) * ppd;
        if (y > h) break;
        float len = (a % 10 == 0) ? 40 : 20;
        DrawLineEx((Vector2){ cx - len, y }, (Vector2){ cx + len, y }, 2, c);
        if (a % 10 == 0) {
            char s[8];
            snprintf(s, sizeof s, "%d", a);
            hud_text(font_mono, s, cx + len + 6, y - 9, 18, c);
        }
    }
    DrawLineEx((Vector2){ cx, cy }, (Vector2){ cx, (float)h }, 1.5f, c);
    // LPD angle from the DSKY (N64 R1 low digits) when P64 is running.
    const DskyState *d = agc_host_dsky();
    if (agc_digit_char(d->prog[0]) == '6' && agc_digit_char(d->prog[1]) == '4') {
        int lpd = (agc_digit_char(d->reg[0][3]) - '0') * 10 + (agc_digit_char(d->reg[0][4]) - '0');
        if (lpd > 0 && lpd < 90) {
            float y = cy + (lpd - 22) * ppd;
            DrawCircleLinesV((Vector2){ cx, y }, 14, (Color){ 120, 255, 150, 230 });
            hud_text(font_bold, "LPD", cx - 60, y - 10, 18, (Color){ 120, 255, 150, 230 });
        }
    }
}

// ---------------------------------------------------------------------------

typedef enum { GS_MENU, GS_FASTFWD, GS_FLY } GameState;

static const char *scen_names[] = {
    "FULL MISSION  -  P63 at PDI -15 min",
    "POWERED DESCENT  -  start just before ignition",
    "HIGH GATE  -  P64 approach phase",
    "FINAL 500 FT  -  you fly P66",
    "LUNAR LIFT-OFF  -  P12 ascent, then dock with Columbia",
};
static const char *scen_desc[] = {
    "Luminary099 is in P63 waiting for the crew. The real procedure: fine-align option, auto maneuver, arm the descent engine, enable ignition at V99, then permit landing radar updates with V57.",
    "Fast-forwards the real AGC and crew through the pre-burn checklist and stops 30 seconds before ignition.",
    "Fast-forwards through the braking phase. Pitch-over is just ahead and Tranquility Base comes into view.",
    "The computer brings you to 150 m. Then it's yours: ROD switch (Up/Down arrows) in P66, ENGINE STOP at contact.",
    "Tranquility Base, one minute to lift-off. At V99: ABORT STAGE (B), ENG ARM ASC (F7), PRO. P12 flies you to orbit, then you dock.",
};
#define N_SCEN 5
enum { ROW_HOST = N_SCEN, ROW_JOIN, ROW_HINTS, N_ROWS };

// ---------------------------------------------------------------------------
// Outside view on a second monitor: a second instance of the game, fed the
// vehicle state by this one over local UDP (one window can't span monitors
// under tiling window managers).

#define PKT_MAGIC 0x41474331u
typedef struct {
    unsigned magic, seq;
    int bye;
    double t, tephem;
    V3 r, v, w, accel_ng;
    Q4 q, q_sm;
    double thrust, thr_bits, gimbal[2], flame_time;
    float jet[LM_NJETS];
    int dps_on, engine_armed, probe_contact, touchdown, lr_pos, lr_range_good, lr_vel_good;
    double altitude, hdot, hspeed, m_dps, m_rcs, m_ascent, m_desc_dry, lr_alt, lr_move, needle[3];
    DskyState dsky;
    char prompt[160], callout[160];
    double callout_time;
    int auto_crew;
    unsigned ack;          // co-op: last LMP command the host applied
    int att_mode, stop_latched;
    int hint, land_result, crash_cause;
    double crash_t, crash_speed, touchdown_t, touchdown_speed, touchdown_hspeed, touchdown_tilt, touchdown_fuel, collapse[4];
    // Ascent and docking.
    int aps_on, thrust_aps, abort_stage, staged, ds_landed, liftoff, csm_valid, asc_active, asc_phase;
    double m_aps, liftoff_t;
    V3 cg, ds_fixed, ds_r, csm_r, csm_v;
    M3 ds_fixed_rot;
    Q4 ds_q, csm_q;
} StatePkt;

static void pack_state(StatePkt *p, int bye)
{
    static unsigned seq;
    memset(p, 0, sizeof *p);
    p->magic = PKT_MAGIC;
    p->seq = ++seq;
    p->bye = bye;
    p->t = lm.t; p->tephem = lm.tephem; p->r = lm.r; p->v = lm.v; p->w = lm.w; p->accel_ng = lm.accel_ng;
    p->q = lm.q; p->q_sm = lm.q_sm; p->thrust = lm.thrust; p->thr_bits = lm.thr_bits;
    p->gimbal[0] = lm.gimbal[0]; p->gimbal[1] = lm.gimbal[1]; p->flame_time = lm.flame_time;
    memcpy(p->jet, lm.jet_level, sizeof p->jet);
    p->dps_on = lm.dps_on; p->engine_armed = lm.engine_armed; p->probe_contact = lm.probe_contact;
    p->touchdown = lm.touchdown; p->lr_pos = lm.lr_pos; p->lr_range_good = lm.lr_range_good; p->lr_vel_good = lm.lr_vel_good;
    p->altitude = lm.altitude; p->hdot = lm.hdot; p->hspeed = lm.hspeed; p->m_dps = lm.m_dps; p->m_rcs = lm.m_rcs;
    p->m_ascent = lm.m_ascent; p->m_desc_dry = lm.m_desc_dry; p->lr_alt = lm.lr_alt; p->lr_move = lm.lr_move;
    memcpy(p->needle, lm.needle, sizeof p->needle);
    p->dsky = *agc_host_dsky();
    memcpy(p->prompt, crew.prompt, sizeof p->prompt);
    memcpy(p->callout, crew.callout, sizeof p->callout);
    p->callout_time = crew.callout_time;
    p->auto_crew = crew.auto_crew;
    p->ack = coop_host_ack();
    p->att_mode = lm_get_attitude_mode();
    p->stop_latched = lm_engine_stop_is_latched();
    p->hint = crew.hint; p->land_result = lm.land_result; p->crash_cause = lm.crash_cause;
    p->crash_t = lm.crash_t; p->crash_speed = lm.crash_speed; p->touchdown_t = lm.touchdown_t;
    p->touchdown_speed = lm.touchdown_speed; p->touchdown_hspeed = lm.touchdown_hspeed;
    p->touchdown_tilt = lm.touchdown_tilt; p->touchdown_fuel = lm.touchdown_fuel;
    memcpy(p->collapse, lm.collapse, sizeof p->collapse);
    p->aps_on = lm.aps_on; p->thrust_aps = lm.thrust_aps; p->abort_stage = lm.abort_stage; p->staged = lm.staged;
    p->ds_landed = lm.ds_landed; p->liftoff = lm.liftoff; p->m_aps = lm.m_aps; p->liftoff_t = lm.liftoff_t;
    p->cg = lm.cg; p->ds_fixed = lm.ds_fixed; p->ds_r = lm.ds_r; p->ds_fixed_rot = lm.ds_fixed_rot; p->ds_q = lm.ds_q;
    p->csm_valid = csm.valid; p->csm_r = csm.r; p->csm_v = csm.v; p->csm_q = csm.q;
    p->asc_active = ascent_active(); p->asc_phase = ascent_phase();
}

static void apply_state(const StatePkt *p)
{
    lm.t = p->t; lm.tephem = p->tephem; lm.r = p->r; lm.v = p->v; lm.w = p->w; lm.accel_ng = p->accel_ng;
    lm.q = p->q; lm.q_sm = p->q_sm; lm.thrust = p->thrust; lm.thr_bits = p->thr_bits;
    lm.gimbal[0] = p->gimbal[0]; lm.gimbal[1] = p->gimbal[1]; lm.flame_time = p->flame_time;
    memcpy(lm.jet_level, p->jet, sizeof p->jet);
    lm.dps_on = p->dps_on; lm.engine_armed = p->engine_armed; lm.probe_contact = p->probe_contact;
    lm.touchdown = p->touchdown; lm.lr_pos = p->lr_pos; lm.lr_range_good = p->lr_range_good; lm.lr_vel_good = p->lr_vel_good;
    lm.altitude = p->altitude; lm.hdot = p->hdot; lm.hspeed = p->hspeed; lm.m_dps = p->m_dps; lm.m_rcs = p->m_rcs;
    lm.m_ascent = p->m_ascent; lm.m_desc_dry = p->m_desc_dry; lm.lr_alt = p->lr_alt; lm.lr_move = p->lr_move;
    memcpy(lm.needle, p->needle, sizeof p->needle);
    agc_host_set_dsky(&p->dsky);
    memcpy(crew.prompt, p->prompt, sizeof crew.prompt);
    memcpy(crew.callout, p->callout, sizeof crew.callout);
    crew.callout_time = p->callout_time;
    crew.auto_crew = p->auto_crew;
    lm_mirror_controls(p->att_mode, p->stop_latched);
    crew.hint = p->hint; lm.land_result = p->land_result; lm.crash_cause = p->crash_cause;
    lm.crash_t = p->crash_t; lm.crash_speed = p->crash_speed; lm.touchdown_t = p->touchdown_t;
    lm.touchdown_speed = p->touchdown_speed; lm.touchdown_hspeed = p->touchdown_hspeed;
    lm.touchdown_tilt = p->touchdown_tilt; lm.touchdown_fuel = p->touchdown_fuel;
    memcpy(lm.collapse, p->collapse, sizeof lm.collapse);
    lm.aps_on = p->aps_on; lm.thrust_aps = p->thrust_aps; lm.abort_stage = p->abort_stage; lm.staged = p->staged;
    lm.ds_landed = p->ds_landed; lm.liftoff = p->liftoff; lm.m_aps = p->m_aps; lm.liftoff_t = p->liftoff_t;
    lm.cg = p->cg; lm.ds_fixed = p->ds_fixed; lm.ds_r = p->ds_r; lm.ds_fixed_rot = p->ds_fixed_rot; lm.ds_q = p->ds_q;
    csm.valid = p->csm_valid; csm.r = p->csm_r; csm.v = p->csm_v; csm.q = p->csm_q;
    ascent_mirror(p->asc_active, p->asc_phase);
}

// Landing results panel: how it went, with numbers.
static double miss_distance(void)
{
    V3 rp = moon_inertial_to_fixed(lm.r, lm.t + lm.tephem);
    double lat, lon, rad;
    moon_fixed_to_latlon(rp, &lat, &lon, &rad);
    double dn = (lat - scenario_info.site_lat) * rad, de = (lon - scenario_info.site_lon) * rad * cos(lat);
    return sqrt(dn * dn + de * de);
}

static void draw_results(int w, int h, int client)
{
    float sc = h / 1080.0f;
    if (sc < 0.7f) sc = 0.7f;
    int crash = lm.land_result == LAND_CRASH;
    const char *title = crash ? "CRASH" : lm.land_result == LAND_HARD ? "HARD LANDING" : "THE EAGLE HAS LANDED";
    static const char *why[] = { "", "Came down too fast: the landing legs folded",
                                 "Hit the ground moving sideways", "Touched down too tilted", "Tipped over after touchdown" };
    const char *sub = crash ? why[lm.crash_cause]
                    : lm.land_result == LAND_HARD ? "Bent a few struts, but the crew is fine" : "A soft, steady landing";
    Color tc = crash ? (Color){ 255, 80, 64, 255 } : lm.land_result == LAND_HARD ? (Color){ 255, 190, 60, 255 }
                                                   : (Color){ 120, 255, 150, 255 };
    float bw = 760 * sc, bh = 420 * sc, x = (w - bw) / 2, y = (h - bh) / 2;
    DrawRectangleRounded((Rectangle){ x, y, bw, bh }, 0.06f, 8, (Color){ 8, 10, 14, 225 });
    DrawRectangleRoundedLinesEx((Rectangle){ x, y, bw, bh }, 0.06f, 8, 2.0f, tc);
    hud_text_centered(font_bold, title, w / 2.0f, y + 28 * sc, 52 * sc, tc);
    hud_text_centered(font_ui, sub, w / 2.0f, y + 92 * sc, 24 * sc, (Color){ 220, 224, 230, 255 });

    char line[160];
    double v = -lm.touchdown_speed, hs = lm.touchdown_hspeed, tl = lm.touchdown_tilt / DEG;
    struct { const char *label; double val; double good, bad; const char *fmt; } rows[] = {
        { "Vertical speed", v, LAND_GOOD_V, LAND_CRASH_V, "%.1f ft/s  (%.1f m/s)" },
        { "Sideways speed", hs, LAND_GOOD_H, LAND_CRASH_H, "%.1f ft/s  (%.1f m/s)" },
        { "Tilt", tl, LAND_GOOD_TILT / DEG, LAND_CRASH_TILT / DEG, "%.0f deg" },
    };
    float ry = y + 150 * sc;
    for (int i = 0; i < 3; i++) {
        if (i < 2) snprintf(line, sizeof line, rows[i].fmt, rows[i].val / 0.3048, rows[i].val);
        else snprintf(line, sizeof line, rows[i].fmt, rows[i].val);
        Color c = rows[i].val > rows[i].bad ? (Color){ 255, 80, 64, 255 }
                : rows[i].val > rows[i].good ? (Color){ 255, 190, 60, 255 } : (Color){ 120, 255, 150, 255 };
        hud_text(font_ui, rows[i].label, x + 70 * sc, ry, 26 * sc, (Color){ 170, 176, 186, 255 });
        hud_text(font_mono, line, x + 360 * sc, ry + 2 * sc, 24 * sc, c);
        ry += 40 * sc;
    }
    snprintf(line, sizeof line, "%.0f m  (%.2f mi)", miss_distance(), miss_distance() / 1609.34);
    hud_text(font_ui, "From the target", x + 70 * sc, ry, 26 * sc, (Color){ 170, 176, 186, 255 });
    hud_text(font_mono, line, x + 360 * sc, ry + 2 * sc, 24 * sc, (Color){ 220, 224, 230, 255 });
    ry += 40 * sc;
    double fs = lm.touchdown_fuel / (13000.0 / (305.0 * 9.80665));
    snprintf(line, sizeof line, "%.0f kg  (~%.0f s of hover)", lm.touchdown_fuel, fs);
    hud_text(font_ui, "Fuel left", x + 70 * sc, ry, 26 * sc, (Color){ 170, 176, 186, 255 });
    hud_text(font_mono, line, x + 360 * sc, ry + 2 * sc, 24 * sc, (Color){ 220, 224, 230, 255 });

#ifdef AGC_VR
    hud_text_centered(font_ui, "B  fly it again      A  hide this      X  open the hatch      Menu button  menu",
#else
    hud_text_centered(font_ui, client ? "Waiting for the Commander.   Space hides this   Esc  menu"
                      : crash ? "Enter  fly it again      Space  hide this      Esc  menu"
                              : "Enter  fly it again    L  lift off (21 hours later)    Space  hide this    Esc  menu",
#endif
                      w / 2.0f, y + bh - 50 * sc, 22 * sc, (Color){ 150, 158, 170, 255 });
}

static int is_fullscreen;
static void set_fullscreen(int on, int monitor)
{
    if (on == is_fullscreen) return;
    if (on) {
        if (monitor >= 0 && monitor < GetMonitorCount()) SetWindowMonitor(monitor);
        int m = GetCurrentMonitor();
#ifdef _WIN32
        (void)m;
        ToggleBorderlessWindowed();
#else
        SetWindowSize(GetMonitorWidth(m), GetMonitorHeight(m));
        ToggleFullscreen();
#endif
    } else {
#ifdef _WIN32
        ToggleBorderlessWindowed();
#else
        ToggleFullscreen();
        SetWindowSize(1600, 900);
#endif
    }
    is_fullscreen = on;
}

static char self_exe[1024];

static void spawn_outside_view(void)
{
    int mon = GetMonitorCount() > 1 ? (GetCurrentMonitor() == 0 ? 1 : 0) : 0;
    char cmd[1400];
#ifdef _WIN32
    snprintf(cmd, sizeof cmd, "start \"\" \"%s\" --spectator --monitor %d", self_exe, mon);
#else
    snprintf(cmd, sizeof cmd, "'%s' --spectator --monitor %d >/dev/null 2>&1 &", self_exe, mon);
#endif
    if (system(cmd) != 0) TraceLog(LOG_WARNING, "could not start the outside view");
}

// --audit views: the exterior all round (chase camera: yaw rad, pitch rad, distance m),
// the cabin from the Commander's seat (head yaw deg, pitch deg, fov deg), and
// every panel and the main instruments framed from the same seat
// (panel index, u, v across/down the panel, fov deg or 0 = whole panel).
typedef struct { const char *name; int kind; double a, b, c, d; } AuditView;
enum { AV_EXT, AV_HEAD, AV_PANEL, AV_STAGED, AV_CSM };
static const AuditView audit_views[] = {
    { "ext_01_front", AV_EXT, 3.14159, 0.10, 13, 0 },      { "ext_02_front_right", AV_EXT, 2.35619, 0.22, 13, 0 },
    { "ext_03_right", AV_EXT, 1.57080, 0.10, 13, 0 },      { "ext_04_aft_right", AV_EXT, 0.78540, 0.22, 13, 0 },
    { "ext_05_aft", AV_EXT, 0.0, 0.10, 13, 0 },            { "ext_06_aft_left", AV_EXT, -0.78540, 0.22, 13, 0 },
    { "ext_07_left", AV_EXT, -1.57080, 0.10, 13, 0 },      { "ext_08_front_left", AV_EXT, -2.35619, 0.22, 13, 0 },
    { "ext_09_top", AV_EXT, 2.6, 1.15, 12, 0 },            { "ext_10_low", AV_EXT, -2.0, -0.12, 11, 0 },
    { "ext_11_close_front", AV_EXT, 2.9, 0.30, 6.5, 0 },   { "ext_12_close_aft", AV_EXT, -0.4, 0.32, 6.5, 0 },
    { "ext_13_close_left", AV_EXT, -1.9, 0.25, 6.5, 0 },   { "ext_14_close_right", AV_EXT, 1.2, 0.25, 6.5, 0 },
    { "ext_15_rocks", AV_EXT, 2.2, 0.04, 30, 0 },
    { "ck_01_window", AV_HEAD, 0, -36, 80, 0 },            { "ck_02_panels_1_2", AV_HEAD, 22, -38, 62, 0 },
    { "ck_03_panel_3_dsky", AV_HEAD, 30, -64, 62, 0 },     { "ck_04_left_wall", AV_HEAD, -80, -25, 80, 0 },
    { "ck_05_right_side", AV_HEAD, 75, -25, 80, 0 },       { "ck_06_overhead", AV_HEAD, 0, 55, 80, 0 },
    { "ck_07_aft", AV_HEAD, 150, -15, 80, 0 },             { "ck_08_aft_left", AV_HEAD, -150, -20, 80, 0 },
    { "ck_09_floor", AV_HEAD, 60, -75, 80, 0 },            { "ck_10_floor_aft", AV_HEAD, 170, -60, 80, 0 },
    { "pn_01_panel_1", AV_PANEL, 0, 0.5, 0.5, 0 },         { "pn_02_panel_2", AV_PANEL, 1, 0.5, 0.5, 0 },
    { "pn_03_panel_3", AV_PANEL, 2, 0.5, 0.5, 0 },         { "pn_04_panel_4", AV_PANEL, 3, 0.5, 0.5, 0 },
    { "pn_05_panel_5", AV_PANEL, 4, 0.5, 0.5, 0 },         { "pn_06_panel_6", AV_PANEL, 5, 0.5, 0.5, 0 },
    { "pn_07_panel_8", AV_PANEL, 6, 0.5, 0.5, 0 },         { "pn_08_panel_11", AV_PANEL, 7, 0.5, 0.5, 0 },
    { "pn_09_panel_12", AV_PANEL, 8, 0.5, 0.5, 0 },        { "pn_10_panel_14", AV_PANEL, 9, 0.5, 0.5, 0 },
    { "pn_11_panel_16", AV_PANEL, 10, 0.5, 0.5, 0 },
    { "in_01_tapes", AV_PANEL, 0, 0.887, 0.325, 20 },      { "in_02_fdai", AV_PANEL, 0, 0.54, 0.31, 22 },
    { "in_03_warning_lights", AV_PANEL, 0, 0.50, 0.05, 22 }, { "in_04_timers", AV_PANEL, 0, 0.33, 0.60, 16 },
    { "in_05_dsky", AV_PANEL, 3, 0.50, 0.50, 24 },         { "in_06_panel_1_switches", AV_PANEL, 0, 0.45, 0.86, 26 },
    // After lift-off: the ascent stage 9 m up, the descent stage left on the ground.
    { "asc_01_front", AV_STAGED, 3.14159, 0.05, 15, 0 },    { "asc_02_aft", AV_STAGED, 0.0, 0.10, 15, 0 },
    { "asc_03_left", AV_STAGED, -1.57080, 0.0, 9, 0 },      { "asc_04_right", AV_STAGED, 1.57080, 0.0, 9, 0 },
    { "asc_05_below", AV_STAGED, 2.4, -0.55, 10, 0 },       { "asc_06_descent_stage", AV_STAGED, -2.3, 0.45, 22, 1 },
    // Columbia, parked beside the LM (yaw, pitch, distance about its middle).
    // (the chase camera's yaw 0 looks from the site's west: Columbia's nose end)
    { "csm_01_side", AV_CSM, 1.57080, 0.05, 16, 0 },        { "csm_02_nose", AV_CSM, 0.0, 0.10, 14, 0 },
    { "csm_03_aft", AV_CSM, 3.14159, 0.15, 16, 0 },         { "csm_04_quarter", AV_CSM, 2.3, 0.35, 15, 0 },
    { "csm_05_below", AV_CSM, -1.2, -0.45, 15, 0 },         { "csm_06_nose_close", AV_CSM, 0.12, 0.04, 6.5, 0 },
    { "csm_07_aft_close", AV_CSM, 3.0, 0.05, 9, 0 },
};
#define N_AUDIT ((int)(sizeof audit_views / sizeof audit_views[0]))

#ifdef AGC_VR
// ---------------------------------------------------------------------------
// Headset. The world is drawn once per eye; everything two-dimensional (menu,
// hints, prompts, results) is drawn into a picture that floats in front of
// the seat. The right-hand controller points at the cabin's switches.

#define UI_W 1600
#define UI_H 900
#define UI_DIST 0.75      // m in front of the seat's eye point
#define UI_WIDTH 0.95     // m
#define UI_DROP 0.03      // m below the eye line

static RenderTexture2D ui_rt;
static Shader ui_sh;
static Texture2D ui_white;

static const char *ui_vs =
    "in vec3 vertexPosition; in vec2 vertexTexCoord; in vec4 vertexColor;\n"
    "uniform mat4 mvp; out vec2 uv; out vec4 vcol;\n"
    "void main(){ uv = vertexTexCoord; vcol = vertexColor; gl_Position = mvp * vec4(vertexPosition, 1.0); }\n";
// The picture holds ordinary display colours with the alpha already multiplied in.
static const char *ui_fs =
    "in vec2 uv; in vec4 vcol; uniform sampler2D texture0; out vec4 finalColor;\n"
    "void main(){\n"
    "  vec4 c = texture(texture0, uv) * vcol;\n"
    "#if DIRECT_OUT == 1\n"
    "  vec3 s = c.a > 0.001 ? c.rgb / c.a : vec3(0.0);\n"
    "  c.rgb = pow(s, vec3(2.2)) * c.a;\n"   // the eye target expects linear light
    "#endif\n"
    "  finalColor = c;\n"
    "}\n";

static void vr_game_init(void)
{
    ui_rt = LoadRenderTexture(UI_W, UI_H);
    SetTextureFilter(ui_rt.texture, TEXTURE_FILTER_BILINEAR);
    ui_sh = render_load_shader(ui_vs, ui_fs);
    static const unsigned char px[4] = { 255, 255, 255, 255 };
    Image img = { (void *)px, 1, 1, 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 };
    ui_white = LoadTextureFromImage(img);
}

// Tracking-space vector -> site frame, for a seat whose view is `b`.
static V3 vr_map(const ViewCam *b, V3 v)
{
    V3 r = v3_norm(v3_cross(b->fwd, b->upv));
    return v3_add(v3_add(v3_scale(r, v.x), v3_scale(b->upv, v.y)), v3_scale(b->fwd, -v.z));
}

static ViewCam vr_eye_cam(const ViewCam *b, const VrEye *e)
{
    ViewCam c = *b;
    c.pos = v3_add(b->pos, vr_map(b, e->pos));
    c.fwd = v3_norm(vr_map(b, e->fwd));
    c.upv = v3_norm(vr_map(b, e->up));
    c.asym = 1;
    c.tan_l = e->tan_l; c.tan_r = e->tan_r; c.tan_d = e->tan_d; c.tan_u = e->tan_u;
    c.fov_y = atan(e->tan_u) - atan(e->tan_d);
    return c;
}

// Controller buttons as the keys the game already understands.
// Pointing at the floating display (a hand or a controller): where the ray
// meets it, in display pixels (top-left origin); -1 when it misses.
static float ui_ptr_x = -1, ui_ptr_y = -1;
static void vr_ui_pointer(void)
{
    const VrInput *in = vr_input();
    ui_ptr_x = ui_ptr_y = -1;
    for (int h = 1; h >= 0; h--) {
        if (!in->aim_ok[h] || in->aim_dir[h].z > -0.05) continue;
        V3 o = in->aim_pos[h], d = in->aim_dir[h];
        double t = (-UI_DIST - o.z) / d.z;
        if (t <= 0) continue;
        V3 p = v3_add(o, v3_scale(d, t));
        double hw = UI_WIDTH / 2, hh = UI_WIDTH * UI_H / UI_W / 2;
        double u = (p.x + hw) / (2 * hw), v = (-UI_DROP + hh - p.y) / (2 * hh);
        if (u < 0 || u > 1 || v < 0 || v > 1) continue;
        ui_ptr_x = (float)(u * UI_W);
        ui_ptr_y = (float)(v * UI_H);
        return;
    }
}

// The menu line under the pointer (same layout as the menu drawing), or -1.
static int vr_menu_row(void)
{
    if (ui_ptr_y < 0 || ui_ptr_x < 60 || ui_ptr_x > 1300) return -1;
    for (int r = 0; r < N_ROWS; r++) {
        float y = 250 + r * 56 + (r >= ROW_HOST ? (r == ROW_HINTS ? 32 : 16) : 0);
        if (ui_ptr_y >= y - 10 && ui_ptr_y < y + 40) return r;
    }
    return -1;
}

static void vr_map_keys(int menu, int results_showing)
{
    const VrInput *in = vr_input();
    static int hold_a = 1, hold_b = 1;   // a button used in the menu must be let go before it acts in flight
    in_begin_frame();
    if (menu) {
        in_set_key(KEY_UP, in->ly > 0.6f || in->ry > 0.6f);
        in_set_key(KEY_DOWN, in->ly < -0.6f || in->ry < -0.6f);
        in_set_key(KEY_LEFT, in->lx < -0.6f || in->rx < -0.6f);
        in_set_key(KEY_RIGHT, in->lx > 0.6f || in->rx > 0.6f);
        in_set_key(KEY_ENTER, in->a || in->rtrig > 0.6f || in->ltrig > 0.6f);   // a pinch, with bare hands
        hold_a = hold_b = 1;
        return;
    }
    if (!in->a) hold_a = 0;
    if (!in->b) hold_b = 0;
    in_set_key(KEY_ESCAPE, in->menu);
    in_set_key(KEY_X, in->x);
    in_set_key(KEY_TAB, in->y);
    in_set_key(KEY_F6, in->lclick);
    in_set_key(KEY_H, in->rclick);
    in_set_key(KEY_SPACE, in->a && !hold_a);
    if (eva_phase() == EVA_SURFACE) {
        in_set_key(KEY_W, in->ly > 0.4f);
        in_set_key(KEY_S, in->ly < -0.4f);
        in_set_key(KEY_D, in->lx > 0.4f);
        in_set_key(KEY_A, in->lx < -0.4f);
        in_set_key(KEY_F, in->ltrig > 0.6f);
        in_set_key(KEY_LEFT_SHIFT, in->lgrip > 0.5f);
        static int turn_was;
        int turn = in->rx > 0.6f ? 1 : in->rx < -0.6f ? -1 : 0;
        if (turn && turn != turn_was) eva_turn(turn * 30 * DEG);   // snap turns: easier on the stomach than a smooth one
        turn_was = turn;
    } else if (results_showing) {
        // B, or a pinch while pointing at the panel (bare hands have no B).
        in_set_key(KEY_ENTER, (in->b && !hold_b) || (ui_ptr_x >= 0 && (in->rtrig > 0.6f || in->ltrig > 0.6f)));
    } else {
        in_set_key(VKEY_PRO, in->b && !hold_b);
    }
    // Docking: the left hand works the translation controller.
    extern int controls_vr_docking;
    controls_vr_docking = ascent_active() && ascent_phase() >= MP_ORBIT && ascent_phase() <= MP_PROX;
    if (controls_vr_docking && eva_phase() != EVA_SURFACE) {
        in_set_key(KEY_UP, in->ly > 0.5f);
        in_set_key(KEY_DOWN, in->ly < -0.5f);
        in_set_key(KEY_LEFT, in->lx < -0.5f);
        in_set_key(KEY_RIGHT, in->lx > 0.5f);
        in_set_key(KEY_T, in->lgrip > 0.5f);
        in_set_key(KEY_G, in->ltrig > 0.6f);
    }
}

// The floating display and the pointing ray, drawn over the finished view of one eye.
static void vr_draw_overlays(const ViewCam *base, const ViewCam *eye, int w, int h, int show_ui, int laser, V3 laser_from, V3 laser_to, int laser_hit)
{
    render_scene_3d_begin_near(w, h, 0.05);
    rlDisableDepthTest();
    rlDisableBackfaceCulling();
    rlSetBlendFactors(RL_ONE, RL_ONE_MINUS_SRC_ALPHA, RL_FUNC_ADD);
    rlSetBlendMode(RL_BLEND_CUSTOM);
    BeginShaderMode(ui_sh);
    V3 r = v3_norm(v3_cross(base->fwd, base->upv)), u = base->upv;
    if (show_ui) {
        V3 c = v3_sub(v3_add(base->pos, vr_map(base, v3(0, -UI_DROP, -UI_DIST))), eye->pos);
        V3 hx = v3_scale(r, UI_WIDTH / 2), hy = v3_scale(u, UI_WIDTH * UI_H / UI_W / 2);
        V3 p0 = v3_sub(v3_sub(c, hx), hy), p1 = v3_sub(v3_add(c, hx), hy), p2 = v3_add(v3_add(c, hx), hy), p3 = v3_add(v3_sub(c, hx), hy);
        rlSetTexture(ui_rt.texture.id);
        rlBegin(RL_QUADS);
        rlColor4ub(255, 255, 255, 255);
        rlTexCoord2f(0, 0); rlVertex3f((float)p0.x, (float)p0.y, (float)p0.z);
        rlTexCoord2f(1, 0); rlVertex3f((float)p1.x, (float)p1.y, (float)p1.z);
        rlTexCoord2f(1, 1); rlVertex3f((float)p2.x, (float)p2.y, (float)p2.z);
        rlTexCoord2f(0, 1); rlVertex3f((float)p3.x, (float)p3.y, (float)p3.z);
        rlEnd();
        rlDrawRenderBatchActive();
    }
    if (laser) {
        // A thin beam (a strip turned toward the eye) and a dot where it lands.
        V3 a = v3_sub(laser_from, eye->pos), b = v3_sub(laser_to, eye->pos);
        V3 side = v3_norm(v3_cross(v3_sub(b, a), a));
        V3 s0 = v3_scale(side, 0.0012), s1 = v3_scale(side, 0.0008);
        V3 q0 = v3_sub(a, s0), q1 = v3_add(a, s0), q2 = v3_add(b, s1), q3 = v3_sub(b, s1);
        rlSetTexture(ui_white.id);
        rlBegin(RL_QUADS);
        rlColor4ub(60, 190, 110, 150);
        rlTexCoord2f(0, 0); rlVertex3f((float)q0.x, (float)q0.y, (float)q0.z);
        rlTexCoord2f(1, 0); rlVertex3f((float)q1.x, (float)q1.y, (float)q1.z);
        rlTexCoord2f(1, 1); rlVertex3f((float)q2.x, (float)q2.y, (float)q2.z);
        rlTexCoord2f(0, 1); rlVertex3f((float)q3.x, (float)q3.y, (float)q3.z);
        if (laser_hit) {
            V3 er = v3_norm(v3_cross(eye->fwd, eye->upv));
            V3 dx = v3_scale(er, 0.005), dy = v3_scale(eye->upv, 0.005);
            V3 d0 = v3_sub(v3_sub(b, dx), dy), d1 = v3_sub(v3_add(b, dx), dy), d2 = v3_add(v3_add(b, dx), dy), d3 = v3_add(v3_sub(b, dx), dy);
            rlColor4ub(120, 255, 170, 230);
            rlTexCoord2f(0, 0); rlVertex3f((float)d0.x, (float)d0.y, (float)d0.z);
            rlTexCoord2f(1, 0); rlVertex3f((float)d1.x, (float)d1.y, (float)d1.z);
            rlTexCoord2f(1, 1); rlVertex3f((float)d2.x, (float)d2.y, (float)d2.z);
            rlTexCoord2f(0, 1); rlVertex3f((float)d3.x, (float)d3.y, (float)d3.z);
        }
        rlEnd();
        rlDrawRenderBatchActive();
    }
    rlSetTexture(0);
    EndShaderMode();
    rlSetBlendMode(RL_BLEND_ALPHA);
    rlEnableBackfaceCulling();
    render_scene_3d_end();
}
#endif

#ifdef __ANDROID__
// The headset has no command line. A text file in the app's own storage,
// /sdcard/Android/data/org.agclander.quest/files/agc-args.txt (pushed with
// adb), supplies one: "--start 3" style options and NAME=VALUE settings, one
// or more per line (benchmarks and tests without the controllers).
const char *agc_android_external_dir(void);
static char *android_argv[64];
static int android_args(char **argv0)
{
    int n = 0;
    android_argv[n++] = argv0[0];
    const char *dir = agc_android_external_dir();
    if (!dir) return n;
    char path[600];
    snprintf(path, sizeof path, "%s/agc-args.txt", dir);
    FILE *f = fopen(path, "r");
    if (!f) return n;
    char tok[256];
    while (n < 63 && fscanf(f, "%255s", tok) == 1) {
        char *eq = strchr(tok, '=');
        if (eq && tok[0] != '-') {
            *eq = 0;
            setenv(tok, eq + 1, 1);
        } else {
            android_argv[n++] = strdup(tok);
        }
    }
    fclose(f);
    TraceLog(LOG_INFO, "options from %s: %d arguments", path, n - 1);
    return n;
}
#endif

int main(int argc, char **argv)
{
#ifdef __ANDROID__
    argc = android_args(argv);
    argv = android_argv;
#endif
    Options opt;
    parse_args(argc, argv, &opt);
    if (opt.audit && opt.start < 0) opt.start = 3;
    int capture = opt.shot || opt.audit;
    int audit_i = -1, audit_frames = 0, audit_fail = 0;

#ifdef AGC_VR
#ifdef __ANDROID__
    SetConfigFlags(FLAG_WINDOW_ALWAYS_RUN);
    InitWindow(0, 0, "AGC Lander");
#else
    SetConfigFlags(FLAG_VSYNC_HINT);
    InitWindow(1600, 844, "AGC Lander - headset stand-in");
#endif
    if (vr_init() != 0) {
        TraceLog(LOG_ERROR, "VR: the headset could not be started");
        CloseWindow();
        return 1;
    }
    render_set_direct(vr_srgb_target() ? 1 : 2);
    capture = 0;
    opt.windowed = 1;
    if (opt.camera == 0) opt.camera = CAM_CDR;   // in the headset you start at the Commander's station
#else
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
    InitWindow(1600, 900, opt.spectator ? "AGC Lander - Outside View" : "AGC Lander - Apollo 11 / Luminary099");
#endif
#ifdef _WIN32
    GetModuleFileNameA(NULL, self_exe, sizeof self_exe);
#else
    {
        ssize_t n = readlink("/proc/self/exe", self_exe, sizeof self_exe - 1);
        if (n > 0) self_exe[n] = 0;
        else snprintf(self_exe, sizeof self_exe, "%s", argv[0]);
    }
#endif
    if (!capture && !opt.windowed) set_fullscreen(1, opt.monitor);
    else if (opt.monitor >= 0 && opt.monitor < GetMonitorCount()) SetWindowMonitor(opt.monitor);
    int outside_on = 0;
    double last_pkt = 0;
    if (opt.spectator) {
        if (net_open_listener(NET_PORT) != 0) TraceLog(LOG_WARNING, "outside view: port %d busy", NET_PORT);
        opt.start = 0;
    }
    SetExitKey(KEY_NULL);
#ifndef AGC_VR
    SetTargetFPS(60);
#endif

    // Sun 10.8 deg up in the east, Earth 66 deg up toward WSW (site frame: x E, y up, z S).
    sun_site = v3_norm(v3(0.978, 0.187, -0.0856));
    earth_site = v3_norm(v3(-0.40, 0.9135, 0.0706));

    if (orbit_yaw_opt < 1e8) orbit_yaw = orbit_yaw_opt;
    if (orbit_pitch_opt < 1e8) orbit_pitch = orbit_pitch_opt;
    if (orbit_dist_opt < 1e8) orbit_dist = orbit_dist_opt;
    render_init();
    hud_init();
    hints_load();
    lmmodel_init();
    csmmodel_init();
    cockpit_init();
    extras_init();
#ifdef AGC_VR
    vr_game_init();
#endif
    start_world();
    (void)earth_site_g;
    if (opt.ufo > 0) extras_force_ufo(opt.ufo);
    if (opt.base) globe_goto(-84.6 + (opt.base > 1 ? opt.base : 9000) * 2.2 / 30300.0, 168.2, opt.base > 1 ? opt.base : 9000);

    GameState gs = opt.start >= 0 ? GS_FASTFWD : GS_MENU;
    if (opt.spectator) gs = GS_FLY;
    int scen = opt.start >= 0 ? opt.start : 0, menu_sel = 0;
    int camera = opt.camera;
    double warp = opt.warp;
    int paused = 0, show_dsky = camera != CAM_CDR && opt.start <= 0, show_help = 0, show_hud = 1;
    int prev_camera = camera;
    float film = (float)opt.film;
    double wall = 0, sim_acc = 0;
    double ff_target_alt = 0;
    int manual_takeover = 0;
    int pgns_mode = 2;
    int retry_req = 0, results_hidden = 0;
    if (gs == GS_FASTFWD) {
        crew.auto_crew = 1;
        ff_target_alt = opt.fast_to_alt > 0 ? opt.fast_to_alt : scen == 2 ? 2600 : scen == 3 ? 150 : 0;
        if (scen == 0) gs = GS_FLY;
        if (scen == 4) { start_ascent(0); crew.auto_crew = getenv("AGC_AUTOCREW") != NULL; gs = GS_FLY; show_dsky = camera != CAM_CDR; }
    }
    // Fades to black between parts of the mission ("21 hours later").
    enum { FADE_NONE, FADE_LIFTOFF, FADE_RNDZ };
    int fade_kind = FADE_NONE, fade_done = 0;
    double fade_t = 0;
    const char *fade_text = "";
    if (opt.spectator) { gs = GS_FLY; crew.auto_crew = 0; }
    static char join_ip[64];
    int coop_host_on = opt.host;
    double coop_lost = 0;
    if (opt.host && gs != GS_MENU && coop_start_host() != 0) crew_say("Co-op: UDP port 47012 is busy");
    if (opt.join) {
        snprintf(join_ip, sizeof join_ip, "%s", opt.join);
        if (coop_start_client(join_ip) == 0) {
            gs = GS_FLY;
            camera = CAM_CDR;
            show_dsky = 0;
            cockpit_set_seat(1);
        }
    }
    if (getenv("AGC_SEND_ONLY") && !opt.spectator) {
        net_open_sender("127.0.0.1", NET_PORT);
        outside_on = 1;
    }
    if (opt.dual && !opt.spectator) {
        spawn_outside_view();
        net_open_sender("127.0.0.1", NET_PORT);
        outside_on = 1;
    }

    RenderTexture2D shot_rt = { 0 };
    if (capture) shot_rt = LoadRenderTexture(1920, 1080);
#if defined(AGC_VR) && !defined(__ANDROID__)
    if (opt.shot) capture = 1;   // stand-in screenshots: stay silent
#endif
    if (!capture && !opt.spectator) sound_init();
#ifdef AGC_VR
    capture = 0;
#endif

    perf_on = getenv("AGC_PERF") != NULL;
    while (!WindowShouldClose()) {
        double pf0 = perf_on ? perf_mark() : 0, pf1 = 0, pf2 = 0, pf3 = 0, pf4 = 0;
#ifdef AGC_VR
        float dt = 0;
        {
            // Headset off, or the system menu is up: nothing is drawn and the flight waits.
            int vr_quit = 0;
            if (!vr_begin_frame(&dt, &vr_quit)) {
                if (vr_quit) break;
                continue;
            }
        }
        if (dt > 0.1f) dt = 0.1f;
        wall += dt;
        int w = UI_W, h = UI_H;
        vr_ui_pointer();
        vr_map_keys(gs == GS_MENU, (gs == GS_FLY && lm.land_result != LAND_NONE && !lm.dps_on && !results_hidden &&
                                    eva_phase() == EVA_INSIDE && !ascent_active()) || ascent_results_due());
#else
        float dt = GetFrameTime();
        if (dt > 0.1f) dt = 0.1f;
        wall += dt;
        int w = GetScreenWidth(), h = GetScreenHeight();
        if (capture) { w = 1920; h = 1080; }
        render_set_output(w, h);
        render_set_output_fbo(capture ? shot_rt.id : 0);
#endif
        // The DSKY floats over the bottom-right corner; the 3D view always fills the screen.
        int dsky_w = (show_dsky && gs != GS_MENU) ? (int)(h * 0.30f) : 0;

        // ---------------- Input ----------------
        if (opt.spectator) {
            if (IsKeyPressed(KEY_TAB)) camera = (camera + 1) % CAM_COUNT;
            if (IsKeyPressed(KEY_F1)) show_help = !show_help;
            if (IsKeyPressed(KEY_F6)) show_hud = !show_hud;
            if (IsKeyPressed(KEY_ESCAPE)) break;
            show_dsky = 0;
            if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                Vector2 d = GetMouseDelta();
                orbit_yaw += d.x * 0.005;
                orbit_pitch = fmax(-1.4, fmin(1.5, orbit_pitch + d.y * 0.005));
            }
            float wh = GetMouseWheelMove();
            if (wh != 0) orbit_dist = fmax(8, fmin(4000, orbit_dist * pow(0.88, wh)));
        } else if (gs == GS_MENU) {
#ifdef AGC_VR
            {   // Pointing at a line chooses it (controller or bare hand).
                int r = vr_menu_row();
                if (r >= 0) menu_sel = r;
            }
#endif
            if (IsKeyPressed(KEY_DOWN)) menu_sel = (menu_sel + 1) % N_ROWS;
            if (IsKeyPressed(KEY_UP)) menu_sel = (menu_sel + N_ROWS - 1) % N_ROWS;
            int enter = IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER) || retry_req;
            retry_req = 0;
            if (menu_sel == ROW_HINTS && (enter || IsKeyPressed(KEY_LEFT) || IsKeyPressed(KEY_RIGHT))) {
                hints_on = !hints_on;
                hints_save();
                enter = 0;
            }
            if (menu_sel == ROW_HOST && (enter || IsKeyPressed(KEY_LEFT) || IsKeyPressed(KEY_RIGHT))) {
                coop_host_on = !coop_host_on;
                enter = 0;
            }
            if (menu_sel == ROW_JOIN) {
                for (int ch = GetCharPressed(); ch; ch = GetCharPressed())
                    if (((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || ch == '.' || ch == '-') &&
                        strlen(join_ip) < sizeof join_ip - 1) {
                        size_t n = strlen(join_ip);
                        join_ip[n] = (char)ch;
                        join_ip[n + 1] = 0;
                    }
                if (IsKeyPressed(KEY_BACKSPACE) && join_ip[0]) join_ip[strlen(join_ip) - 1] = 0;
                if (enter && join_ip[0]) {
                    start_world();
                    if (coop_start_client(join_ip) == 0) {
                        gs = GS_FLY;
                        camera = prev_camera = CAM_CDR;
                        show_dsky = 0;
                        cockpit_set_seat(1);
                        coop_lost = 0;
                    } else {
                        crew_say("Co-op: could not resolve that address");
                    }
                }
                enter = 0;
            } else {
                while (GetCharPressed()) {}
            }
            if (enter) {
                scen = menu_sel;
                coop_stop();
                if (coop_host_on && coop_start_host() != 0) crew_say("Co-op: UDP port 47012 is busy");
                start_world();
                crew.auto_crew = 1;
                manual_takeover = 0;
                results_hidden = 0;
                show_dsky = (scen == 0 || scen == 4) && camera != CAM_CDR;   // F4 shows it any time
                ff_target_alt = scen == 2 ? 2600 : scen == 3 ? 150 : 0;
                gs = scen == 0 || scen == 4 ? GS_FLY : GS_FASTFWD;
                if (scen == 0) crew.auto_crew = 0;
                if (scen == 4) { start_ascent(0); crew.auto_crew = 0; }
                fade_kind = FADE_NONE;
            }
        } else {
            if (IsKeyPressed(KEY_TAB)) camera = (camera + 1) % CAM_COUNT;
            if (camera != prev_camera) {
                // The cockpit has its own DSKY: hide the side one there.
                if (camera == CAM_CDR) show_dsky = 0;
                else if (prev_camera == CAM_CDR) show_dsky = scen == 0 || ascent_active();   // only the full mission and the ascent need it
                prev_camera = camera;
            }
            if (IsKeyPressed(KEY_F1)) show_help = !show_help;
            if (IsKeyPressed(KEY_F2)) crew.auto_crew = !crew.auto_crew;
            if (IsKeyPressed(KEY_F3)) film = film > 0 ? 0 : 1;
            if (IsKeyPressed(KEY_F4)) show_dsky = !show_dsky;
            if (IsKeyPressed(KEY_F5) || IsKeyPressed(KEY_PAUSE)) paused = !paused;
            if (IsKeyPressed(KEY_F6)) show_hud = !show_hud;
            if (IsKeyPressed(KEY_PAGE_UP)) warp = warp >= 60 ? 60 : warp < 1 ? 1 : warp * (warp < 2 ? 2 : warp < 5 ? 2.5 : 2);
            if (IsKeyPressed(KEY_PAGE_DOWN)) warp = warp <= 1 ? 1 : warp / (warp <= 2 ? 2 : warp <= 5 ? 2.5 : 2);
            if (IsKeyPressed(KEY_F7)) {
                int arm = lm.engine_armed ? ARM_OFF : (ascent_active() || lm.staged) ? ARM_ASC : ARM_DES;
                lm_set_engine_arm(arm);
                crew_say(arm == ARM_ASC ? "ENG ARM: ASC (ascent engine)" : arm == ARM_DES ? "ENG ARM: DES (descent engine)" : "ENG ARM: OFF");
            }
            if (IsKeyPressed(KEY_B) && !lm.abort_stage) {
                int flying = !lm.touchdown;
                lm_abort_stage();
                if (flying && !lm.aps_on) lm_set_engine_arm(ARM_ASC);
                if (flying && !ascent_active() && lm.staged) ascent_begin();   // abort to orbit: then rendezvous and docking
                crew_say(flying ? "ABORT STAGE: stages separated, ascent engine armed" : "ABORT STAGE: the stage separation bolts have fired");
            }
            if (IsKeyPressed(KEY_M)) {
                sound_set_muted(!sound_muted());
                crew_say(sound_muted() ? "Cabin sound off (M turns it on)" : "Cabin sound on (heard in the cockpit and window views)");
            }
            if (IsKeyPressed(KEY_H)) {
                hints_on = !hints_on;
                hints_save();
                crew_say(hints_on ? "Hints on (H hides them)" : "Hints off (H shows them)");
            }
            if (IsKeyPressed(KEY_X) && !opt.spectator && !ascent_active()) {
                if (eva_phase() == EVA_INSIDE && eva_available()) { results_hidden = 1; camera = CAM_CDR; }
                eva_key();
            }
            if (ascent_active() && fade_kind == FADE_NONE) {
                int ph = ascent_phase();
                if (ph == MP_ORBIT || ph == MP_DOCKED || ph == MP_DOCK_FAILED) {
                    if (IsKeyPressed(KEY_SPACE)) ascent_hide_results(!ascent_results_hidden());
                    if (coop_role != COOP_CLIENT && (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER)) && ascent_results_due()) {
                        if (ph == MP_DOCKED) {
                            gs = GS_MENU;
                            menu_sel = scen;
                        } else {
                            fade_kind = FADE_RNDZ;
                            fade_t = 0;
                            fade_done = 0;
                            fade_text = ph == MP_ORBIT ? "THREE AND A HALF HOURS LATER  -  CSI, CDH, TPI and braking flown.  Columbia is 130 feet away."
                                                       : "BACKING OFF  -  another approach to Columbia";
                        }
                    }
                }
            }
            if (lm.land_result != LAND_NONE && !lm.dps_on && eva_phase() == EVA_INSIDE && !ascent_active()) {
                if (IsKeyPressed(KEY_SPACE)) results_hidden = !results_hidden;
                if (IsKeyPressed(KEY_L) && lm.land_result != LAND_CRASH && coop_role != COOP_CLIENT && fade_kind == FADE_NONE &&
                    lm.t - lm.touchdown_t > 3) {
                    fade_kind = FADE_LIFTOFF;
                    fade_t = 0;
                    fade_done = 0;
                    fade_text = "TWENTY-ONE HOURS LATER  -  124:21 GET.  The ascent stage is ready.  P12 counting down.";
                }
                if (coop_role != COOP_CLIENT && (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER)) &&
                    lm.t - lm.touchdown_t > 3) {
                    gs = GS_MENU;
                    menu_sel = scen;
                    retry_req = 1;
                }
            }
            if (IsKeyPressed(KEY_F8)) {
                pgns_mode = pgns_mode == 2 ? 1 : 2;
                lm_set_attitude_mode(pgns_mode);
                crew_say(pgns_mode == 2 ? "PGNS mode control: AUTO" : "PGNS mode control: ATT HOLD");
            }
            // Hand controllers: keyboard, gamepad, flight stick (rate command in ATT HOLD).
            {
                Controls ct;
                controls_poll(&ct);
                if (eva_phase() != EVA_INSIDE) {
                    // Outside (or at the hatch) the same keys walk the astronaut: nothing reaches the LM.
                    char dev[sizeof ct.device];
                    memcpy(dev, ct.device, sizeof dev);
                    memset(&ct, 0, sizeof ct);
                    memcpy(ct.device, dev, sizeof dev);
                }
                if (ascent_active() && ascent_phase() >= MP_ORBIT) {
                    // In orbit the keyboard has a translation controller: T / G along
                    // +X (toward / away from Columbia), the arrows across (as seen
                    // through the docking window: screen right is the LM's -Y).
                    int kx = (IsKeyDown(KEY_T) ? 1 : 0) - (IsKeyDown(KEY_G) ? 1 : 0);
                    int ky = (IsKeyDown(KEY_LEFT) ? 1 : 0) - (IsKeyDown(KEY_RIGHT) ? 1 : 0);
                    int kz = (IsKeyDown(KEY_UP) ? 1 : 0) - (IsKeyDown(KEY_DOWN) ? 1 : 0);
                    if (kx) ct.tx = kx;
                    if (ky) ct.ty = ky;
                    if (kz) ct.tz = kz;
                    ct.rod = 0;
                }
                {   // Debugging: AGC_FAKE_RHC=pitch,yaw,roll[,tx,ty,tz] holds the hand controllers there.
                    const char *fr = getenv("AGC_FAKE_RHC");
                    if (fr) {
                        double fp = 0, fyw = 0, frl = 0;
                        int fx = 0, fy = 0, fz = 0;
                        sscanf(fr, "%lf,%lf,%lf,%d,%d,%d", &fp, &fyw, &frl, &fx, &fy, &fz);
                        ct.pitch = fp; ct.yaw = fyw; ct.roll = frl;
                        if (fx || fy || fz) { ct.tx = fx; ct.ty = fy; ct.tz = fz; }
                    }
                }
                if (coop_role == COOP_HOST) coop_host_rhc(&ct.pitch, &ct.yaw, &ct.roll);
                agc_host_set_rhc(ct.pitch, ct.yaw, ct.roll);
                cockpit_set_hand_controls(ct.pitch, ct.yaw, ct.roll);
                static int last_t[3], pro_held;
                int t[3] = { ct.tx, ct.ty, ct.tz };
                for (int k = 0; k < 3; k++)
                    if (t[k] != last_t[k]) { lm_set_thc(k, t[k]); last_t[k] = t[k]; }
                while (ct.rod > 0) { lm_rod_click(+1); ct.rod--; }
                while (ct.rod < 0) { lm_rod_click(-1); ct.rod++; }
                if (ct.stop) lm_engine_stop();
                if (ct.pro != pro_held) { agc_host_pro(ct.pro); pro_held = ct.pro; }
                if (ct.enter) agc_host_key(KEY_AGC_ENTR);
                if (ct.camera) camera = (camera + 1) % CAM_COUNT;
                if (ct.pause) paused = !paused;
                static char last_dev[400];
                if (strcmp(last_dev, ct.device)) {
                    if (ct.device[0]) {
                        char m[460];
                        snprintf(m, sizeof m, "Controller connected - %s", ct.device);
                        crew_say(m);
                    }
                    snprintf(last_dev, sizeof last_dev, "%s", ct.device);
                }
            }
            if (IsKeyPressed(KEY_ESCAPE)) { gs = GS_MENU; if (coop_role == COOP_CLIENT) coop_stop(); }
            if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                Vector2 d = GetMouseDelta();
                orbit_yaw += d.x * 0.005;
                orbit_pitch = fmax(-1.4, fmin(1.5, orbit_pitch + d.y * 0.005));
            }
            float wh = GetMouseWheelMove();
            if (wh != 0) orbit_dist = fmax(8, fmin(4000, orbit_dist * pow(0.88, wh)));
        }

        if (IsKeyPressed(KEY_F11)) set_fullscreen(!is_fullscreen, -1);
        if (!opt.spectator && IsKeyPressed(KEY_F10)) {
            if (!outside_on) {
                spawn_outside_view();
                net_open_sender("127.0.0.1", NET_PORT);
                outside_on = 1;
                crew_say("Outside view opened on the other monitor (F10 closes it)");
            } else {
                StatePkt p;
                pack_state(&p, 1);
                net_send(&p, sizeof p);
                outside_on = 0;
            }
        }

        // Demo director: cut between cameras, finish on the touchdown.
        if (opt.demo && gs == GS_FLY) {
            static double cut_t;
            static int shot_i, landed_cut;
            static const int seq[] = { CAM_CHASE, CAM_CDR, CAM_TOWER, CAM_CINEMA, CAM_CDR, CAM_DAC };
            static const double look[][2] = { { 0, 0 }, { 0, -30 }, { 0, 0 }, { 0, 0 }, { 25, -48 }, { 0, 0 } };
            cut_t += dt;
            int want_cut = cut_t > 14;
            if (lm.altitude < 45 && !landed_cut) { landed_cut = 1; camera = CAM_CHASE; orbit_yaw = -0.6; orbit_pitch = 0.12; orbit_dist = 30; cut_t = 0; want_cut = 0; }
            if (lm.touchdown && cut_t > 12) { landed_cut = 2; camera = CAM_CINEMA; cut_t = -1e9; }
            if (want_cut && !landed_cut) {
                cut_t = 0;
                shot_i = (shot_i + 1) % 6;
                camera = seq[shot_i];
                if (camera == CAM_CDR) cockpit_set_view(look[shot_i][0], look[shot_i][1]);
                if (camera == CAM_CHASE) { orbit_yaw = -0.9 + shot_i; orbit_dist = 26; }
            }
            show_dsky = camera != CAM_CDR;
        }

        // ---------------- Simulation ----------------
        coop_tick(GetTime());
        if (coop_role == COOP_HOST) coop_host_poll();
        if (coop_role == COOP_CLIENT && gs == GS_FLY) {
            // LM Pilot: the host runs the AGC and the vehicle; mirror it.
            StatePkt p;
            if (coop_client_poll_state(&p, sizeof p) == (int)sizeof p && p.magic == PKT_MAGIC && !p.bye) {
                apply_state(&p);
                coop_client_ack(p.ack);
                coop_lost = 0;
            }
            {
                // Test hook: the LMP types a key sequence once connected.
                static int typed;
                const char *tk = getenv("AGC_COOP_TEST_KEYS");
                if (tk && !typed && coop_peer_connected()) {
                    static const int dig[10] = { KEY_AGC_0, KEY_AGC_1, KEY_AGC_2, KEY_AGC_3, KEY_AGC_4,
                                                 KEY_AGC_5, KEY_AGC_6, KEY_AGC_7, KEY_AGC_8, KEY_AGC_9 };
                    for (const char *c = tk; *c; c++)
                        agc_host_key(*c == 'V' ? KEY_AGC_VERB : *c == 'N' ? KEY_AGC_NOUN : *c == 'E' ? KEY_AGC_ENTR
                                     : *c == 'R' ? KEY_AGC_RSET : dig[(*c - '0') % 10]);
                    typed = 1;
                }
            }
            coop_client_send();
            if (!coop_peer_connected()) {
                coop_lost += dt;
                if (coop_lost > 30) { coop_stop(); gs = GS_MENU; crew_say("Co-op: no answer from the host"); }
            }
        } else if (opt.spectator) {
            StatePkt p;
            int got = 0, bye = 0;
            while (net_recv(&p, sizeof p) == (int)sizeof p)
                if (p.magic == PKT_MAGIC) {
                    if (p.bye) bye = 1;
                    else { apply_state(&p); got = 1; }
                }
            if (got) last_pkt = GetTime();
            if (bye || (last_pkt > 0 && GetTime() - last_pkt > 20) || (last_pkt == 0 && GetTime() > 60)) break;
            if (camera == CAM_CDR) camera = CAM_CHASE;
        } else if (gs == GS_FASTFWD) {
            // Run the real AGC flat out while the screen shows progress.
            double t0 = GetTime();
            int done = 0;
            while (GetTime() - t0 < 0.025 && !done) {
                sim_advance(1.0);
                if (scen == 1) done = lm.t > 369615;                        // TIG - 30 s
                else if (ff_target_alt > 0) done = lm.dps_on && lm.altitude < ff_target_alt;
                if (lm.touchdown) done = 1;
            }
            if (done) {
                gs = GS_FLY;
                if (scen == 3) {
                    crew.auto_crew = 0;
                    manual_takeover = 1;
                    pgns_mode = 1;
                    lm_set_attitude_mode(1);   // PGNS ATT HOLD ...
                    lm_rod_click(-1);          // ... and a ROD click: Luminary switches to P66
                    crew_say("\"I'm going to take it.\"  P66: you have the ROD switch");
                }
                if (scen == 1) crew.auto_crew = 0;
                if (opt.crash) lm_engine_stop();
            }
        } else if (gs == GS_FLY && !paused && coop_role != COOP_CLIENT) {
            sim_acc += dt * warp;
            double t0 = GetTime();
            while (sim_acc > 0.02 && GetTime() - t0 < 0.03) {
                sim_advance(0.02);
                sim_acc -= 0.02;
            }
            if (sim_acc > 0.5) sim_acc = 0.5;   // can't keep up: drop time
        }
        (void)manual_takeover;

        // Surface audit: park the LM, hide the overlays, step through the views.
        if (opt.audit && gs == GS_FLY) {
            if (audit_i < 0) { audit_i = 0; audit_frames = 0; pose_park(); paused = 1; show_hud = 0; show_dsky = 0; }
            const AuditView *av = &audit_views[audit_i];
            if (av->kind == AV_HEAD) {
                camera = CAM_CDR;
                cockpit_set_view(av->a, av->b);
                cockpit_set_fov(av->c);
            } else if (av->kind == AV_PANEL) {
                camera = CAM_CDR;
                cockpit_look_at_panel((int)av->a, av->b, av->c, av->d, 16.0 / 9.0);
            } else if (av->kind == AV_STAGED || av->kind == AV_CSM) {
                audit_after_liftoff(av->kind == AV_CSM, av->d > 0);
                camera = CAM_CHASE;
                orbit_yaw = av->a; orbit_pitch = av->b; orbit_dist = av->c;
            } else {
                camera = CAM_CHASE;
                orbit_yaw = av->a; orbit_pitch = av->b; orbit_dist = av->c;
            }
            prev_camera = camera;
        }

        // Fades between parts of the mission: the change happens while the screen is black.
        if (fade_kind != FADE_NONE) {
            fade_t += dt;
            if (!fade_done && fade_t >= 1.3) {
                fade_done = 1;
                if (fade_kind == FADE_LIFTOFF) {
                    start_ascent(1);
                    results_hidden = 0;
                    show_dsky = camera != CAM_CDR;
                } else if (fade_kind == FADE_RNDZ) {
                    ascent_rendezvous();
                    camera = prev_camera = CAM_DOCK;
                    show_dsky = 0;
                }
            }
            if (fade_t >= 3.8) fade_kind = FADE_NONE;
        }
        {   // Debugging / screenshots: AGC_DOCK=metres puts Eagle that far from Columbia, lined up.
            static int dock_dbg;
            const char *dd = getenv("AGC_DOCK");
            if (dd && gs == GS_FLY && !dock_dbg) {
                dock_dbg = 1;
                if (!ascent_active()) start_ascent(0);
                ascent_debug_dock(atof(dd));
                if (getenv("AGC_DOCKCAM")) camera = prev_camera = atoi(getenv("AGC_DOCKCAM"));
            }
        }

        // ---------------- Rendering ----------------
        if (perf_on) pf1 = perf_mark();
        if (!opt.spectator && gs != GS_MENU) dsky_frame();
        int scene_w0 = w;
        if (opt.look_set) { cockpit_set_view(opt.look_yaw, opt.look_pitch); opt.look_set = 0; }
        {   // Debugging: AGC_EVA=hatch|crawl|ladder|walk|flag parks the LM and jumps into the moonwalk.
            static int eva_dbg;
            const char *ed = getenv("AGC_EVA");
            if (ed && ed[0] == 'a') {
                // "auto": the LM stands parked as after a good landing, and the hatch opens by itself.
                if (gs == GS_FLY) {
                    if (!eva_dbg) { pose_park(); paused = 1; }
                    lm.touchdown = 1; lm.dps_on = 0; lm.land_result = LAND_GOOD;
                    if (!eva_dbg) { eva_dbg = 2; results_hidden = 1; eva_key(); }
                    eva_update(dt, lm_site_pos(), lm_site_rot());   // the sim is paused; the walk is not
                }
            } else if (ed && gs == GS_FLY && !eva_dbg) {
                eva_dbg = 1;
                pose_park();
                paused = 1;
                eva_debug(ed, lm_site_pos(), lm_site_rot());
            }
        }
        // The moonwalk takes the view over: kneeling at the hatch in the cabin, then the astronaut's own eyes.
        if (gs == GS_FLY && !paused && !opt.spectator) eva_update(dt, lm_site_pos(), lm_site_rot());
        cockpit_set_egress(eva_hatch(), eva_at_hatch() ? eva_kneel() : 0);
        int vcam = eva_outside() ? CAM_EVA : eva_at_hatch() ? CAM_CDR : camera;
        if (gs != GS_MENU && vcam == CAM_CDR) cockpit_input(scene_w0, h);
        if (gs != GS_MENU && vcam == CAM_GLOBE) globe_input(scene_w0, h);
#ifdef AGC_VR
        // The seat's view is level and straight ahead; the head does the looking.
        cockpit_vr_level();
        {
            const VrInput *in = vr_input();
            eva_set_vr(atan2(in->head_fwd.x, -in->head_fwd.z));
        }
        if (gs == GS_MENU) vcam = CAM_CHASE;   // the menu floats outside, with the LM in view
#endif
        ViewCam cam = make_camera(vcam, lm_site_pos(), lm_site_rot(), wall);
        if (lm.land_result == LAND_CRASH && vcam != CAM_GLOBE) {
            // Impact shake, strongest in the cockpit.
            double age = lm.t - lm.crash_t;
            double amp = (vcam == CAM_CDR ? 0.06 : 0.25) * fmin(1.0, lm.crash_speed / 6.0) * exp(-age * 1.6);
            if (age >= 0 && amp > 0.002)
                cam.pos = v3_add(cam.pos, v3(amp * sin(wall * 61.0), amp * sin(wall * 47.0 + 1.0), amp * sin(wall * 53.0 + 2.0)));
        }
        extras_update(dt, gs == GS_FLY && !paused && !opt.spectator, &cam);
        float film_now = film > 0 || vcam == CAM_DAC ? 1.0f : 0.0f;
        if ((show_hud && gs != GS_MENU) || vcam == CAM_CDR) hud_prepare();
        if (gs != GS_MENU && !opt.spectator) cockpit_update(dt);
        {   // Cabin sound follows the flight state; only heard from inside.
            static float prev_jet[LM_NJETS];
            static int prev_result = LAND_NONE, prev_touch;
            SoundState ss = { 0 };
            ss.inside = vcam == CAM_CDR || vcam == CAM_DAC || vcam == CAM_EVA || vcam == CAM_DOCK;   // on the surface: the suit's own fan
            ss.running = gs == GS_FLY && !paused;
            // The ascent engine sits right behind the cabin's back wall: loud for its size.
            ss.thrust = (float)(lm.thrust_aps ? lm.thrust / 24000.0 : lm.thrust / 45000.0);
            for (int j = 0; j < LM_NJETS; j++) {
                ss.rcs += lm.jet_level[j];
                if (lm.jet_level[j] > 0.3f && prev_jet[j] <= 0.3f) ss.rcs_hits++;
                prev_jet[j] = lm.jet_level[j];
            }
            ss.alarm = cp_master_alarm();
            ss.fans = cp_fan_level();
            if (ss.running) {
                if (lm.land_result == LAND_CRASH && prev_result != LAND_CRASH) sound_crash();
                else if (lm.touchdown && !prev_touch) sound_touchdown((float)lm.touchdown_speed);
            }
            prev_result = lm.land_result;
            prev_touch = lm.touchdown;
            sound_update(&ss, dt);
        }
        if (perf_on) pf2 = perf_mark();
        if (vcam == CAM_CDR) cockpit_prepare(lm_site_rot(), sun_site, local_up_at(lm_site_pos()), dt);
        if (perf_on) pf3 = perf_mark();
        int scene_w = w;
#ifdef AGC_VR
        // The right-hand controller points at the cabin's switches; the trigger works them.
        int laser = 0, laser_hit = 0;
        V3 laser_from = v3(0, 0, 0), laser_to = v3(0, 0, 0);
        {
            const VrInput *in = vr_input();
            static int trig_was = 1;
            int trig = in->rtrig > 0.6f, pressed = trig && !trig_was;
            trig_was = trig || gs == GS_MENU;   // the trigger that chose a menu line must be let go first
            if (gs == GS_FLY && vcam == CAM_CDR && in->aim_ok[1] && !eva_at_hatch()) {
                M3 Rt = m3_transpose(lm_site_rot());
                V3 o = v3_add(cam.pos, vr_map(&cam, in->aim_pos[1])), d = v3_norm(vr_map(&cam, in->aim_dir[1]));
                double dist = cockpit_pointer(m3_apply(Rt, v3_sub(o, lm_site_pos())), m3_apply(Rt, d), pressed);
                laser = 1;
                laser_hit = dist > 0;
                laser_from = o;
                laser_to = v3_add(o, v3_scale(d, dist > 0 ? dist : 1.2));
                if (pressed && laser_hit) vr_haptic(1, 0.5f, 0.03f);
            }
        }
        // Everything two-dimensional goes into the floating display.
        BeginTextureMode(ui_rt);
        ClearBackground(BLANK);
        rlSetBlendFactorsSeparate(RL_SRC_ALPHA, RL_ONE_MINUS_SRC_ALPHA, RL_ONE, RL_ONE_MINUS_SRC_ALPHA, RL_FUNC_ADD, RL_FUNC_ADD);
        rlSetBlendMode(RL_BLEND_CUSTOM_SEPARATE);
#else
        BeginDrawing();
        if (capture) BeginTextureMode(shot_rt);
        ClearBackground(BLACK);
        draw_world(&cam, scene_w > 64 ? scene_w : w, h, film_now, gs == GS_MENU ? CAM_CHASE : vcam);
#endif
        (void)draw_lpd;
        if (perf_on) pf4 = perf_mark();

        if (gs == GS_MENU) {
            DrawRectangleGradientV(0, 0, w, h, (Color){ 0, 0, 0, 120 }, (Color){ 0, 0, 0, 230 });
            hud_text(font_bold, "AGC LANDER", 80, 90, 84, (Color){ 235, 235, 235, 255 });
            hud_text(font_ui, "Apollo 11 Lunar Module  -  flown by the original Luminary099 flight software on an emulated AGC",
                     84, 182, 24, (Color){ 170, 176, 186, 255 });
            for (int i = 0; i < N_SCEN; i++) {
                Color c = i == menu_sel ? (Color){ 255, 190, 60, 255 } : (Color){ 200, 205, 212, 255 };
                hud_text(font_bold, scen_names[i], 100, 250 + i * 56, 30, c);
            }
            {
                Color c4 = menu_sel == ROW_HOST ? (Color){ 255, 190, 60, 255 } : (Color){ 200, 205, 212, 255 };
                Color c5 = menu_sel == ROW_JOIN ? (Color){ 255, 190, 60, 255 } : (Color){ 200, 205, 212, 255 };
                hud_text(font_bold, coop_host_on ? "TWO PLAYERS:  HOST  -  you are the Commander" : "TWO PLAYERS:  OFF", 100, 250 + ROW_HOST * 56 + 16, 30, c4);
                char jl[128];
                snprintf(jl, sizeof jl, "JOIN A GAME AS LM PILOT:  %s%s", join_ip[0] ? join_ip : "type the host's IP",
                         menu_sel == ROW_JOIN && ((int)(GetTime() * 2) & 1) ? "_" : "");
                hud_text(font_bold, jl, 100, 250 + ROW_JOIN * 56 + 16, 30, c5);
                Color c6 = menu_sel == ROW_HINTS ? (Color){ 255, 190, 60, 255 } : (Color){ 200, 205, 212, 255 };
                hud_text(font_bold, hints_on ? "PLAIN-ENGLISH HINTS:  ON" : "PLAIN-ENGLISH HINTS:  OFF", 100, 250 + ROW_HINTS * 56 + 32, 30, c6);
            }
            const char *desc = menu_sel < N_SCEN ? scen_desc[menu_sel]
                             : menu_sel == ROW_HINTS ? "Enter toggles. A box at the top of the screen tells you what to do next in plain words: which key, which way to lean, when to cut the engine. H toggles it in flight."
                             : menu_sel == ROW_HOST ? "Enter toggles. With HOST on, start any scenario above: a second player can join you as LM Pilot (UDP port 47012). You fly; they run the DSKY and the panels."
                                                    : "Type the Commander's IP address (or name) and press Enter. You get the LMP station: DSKY, panels, switches. Same version of the game on both sides.";
#ifdef AGC_VR
            hud_text(font_ui, desc, 100, 250 + N_ROWS * 56 + 34, 25, (Color){ 175, 182, 192, 255 });   // a headset needs bigger type
#else
            hud_text(font_ui, desc, 100, 250 + N_ROWS * 56 + 40, 21, (Color){ 160, 168, 178, 255 });
#endif
#ifdef AGC_VR
            hud_text(font_ui, "Point at a line (or thumbstick up/down) and pull the trigger - or pinch, with bare hands - to start.", 100, h - 150, 25, (Color){ 165, 172, 182, 255 });
            hud_text(font_ui, "In flight:  right stick = hand controller   left stick = yaw, up/down = ROD switch   A = ENGINE STOP   B = PRO",
                     100, h - 112, 23, (Color){ 150, 156, 166, 255 });
            hud_text(font_ui, "Point at a switch and pull the trigger (or pinch)   X = hatch   Y = camera   Menu button = back here",
                     100, h - 80, 23, (Color){ 150, 156, 166, 255 });
#else
            hud_text(font_ui, "Up/Down to choose, Enter to start.  In flight: F1 for help.", 100, h - 70, 22,
                     (Color){ 130, 136, 146, 255 });
#endif
        } else {
            hud_set_minimal(vcam == CAM_CDR || vcam == CAM_GLOBE || vcam == CAM_EVA || vcam == CAM_DOCK);
            if (vcam == CAM_DOCK) draw_coas(scene_w, h);
            if (show_hud) hud_draw(w, h, 0, (float)warp, vcam == CAM_EVA ? "MOONWALK" : cam_names[vcam], paused);
            hints_set_cockpit(vcam == CAM_CDR);
            hud_camera_is_docking = vcam == CAM_DOCK;
            if (show_hud && gs == GS_FLY) hints_draw(w, h, 0);
            if (gs == GS_FLY && !opt.audit) eva_overlay(scene_w, h);
            if (gs == GS_FLY && lm.land_result != LAND_NONE && !lm.dps_on && !results_hidden && eva_phase() == EVA_INSIDE &&
                !ascent_active() && fade_kind == FADE_NONE && lm.t - (lm.land_result == LAND_CRASH ? lm.crash_t : lm.touchdown_t) > 3)
                draw_results(scene_w, h, coop_role == COOP_CLIENT);
            if (gs == GS_FLY && ascent_active()) {
                ascent_overlay(scene_w, h);
                if (fade_kind == FADE_NONE) ascent_draw_results(scene_w, h, coop_role == COOP_CLIENT);
            }
            if (fade_kind != FADE_NONE) {
                float a = fade_t < 1.2 ? (float)(fade_t / 1.2) : fade_t < 2.6 ? 1.0f : (float)fmax(0, (3.8 - fade_t) / 1.2);
                DrawRectangle(0, 0, w, h, (Color){ 0, 0, 0, (unsigned char)(255 * a) });
                float sc = h / 1080.0f;
                Color tc = { 230, 230, 225, (unsigned char)(255 * a) };
                hud_text_centered(font_bold, fade_text, w / 2.0f, h / 2.0f - 20 * sc, 30 * sc, tc);
            }
            if (vcam == CAM_CDR && gs == GS_FLY && !opt.audit) cockpit_overlay(scene_w, h);
            if (coop_role != COOP_OFF) {
                float sc = h / 1080.0f;
                if (sc < 0.7f) sc = 0.7f;
                hud_text(font_mono, coop_status(), 18 * sc, 166 * sc, 17 * sc,
                         coop_peer_connected() ? (Color){ 120, 220, 255, 255 } : (Color){ 255, 190, 60, 255 });
            }
            if (vcam == CAM_GLOBE) globe_overlay(&cam, scene_w, h, lm_site_pos());
            if (gs == GS_FASTFWD) {
                DrawRectangle(0, h / 2 - 50, scene_w, 100, (Color){ 0, 0, 0, 170 });
                char b[96];
                snprintf(b, sizeof b, "FAST-FORWARDING THE AGC   GET %03d:%02d:%02d   ALT %.0f ft",
                         (int)(lm.t / 3600), (int)(lm.t / 60) % 60, (int)fmod(lm.t, 60), lm.altitude / 0.3048);
                hud_text_centered(font_bold, b, scene_w / 2.0f, h / 2 - 18, 30, (Color){ 255, 190, 60, 255 });
            }
            if (show_dsky && vcam != CAM_EVA) {   // out on the surface the computer stays in the cabin
                float dh = dsky_w * DSKY_H / DSKY_W, m = h * 0.015f;
                dsky_update_draw((Rectangle){ w - dsky_w - m, h - dh - m, (float)dsky_w, dh });
            }
            if (show_help) {
                const char *lines[] = {
                    "DSKY  0-9  V verb  N noun  Enter  C clr  R rset  P pro  K key rel  = +  - -",
                    "Tab  camera (chase, cockpit, 16mm, tower, cinematic, lunar orbit)   Right-drag look/orbit",
                    "PgUp/PgDn  time warp   F5  pause   Esc  menu   F10  outside view on 2nd monitor   F11  fullscreen",
                    "F2  auto crew on/off  F3  16mm film  F4  DSKY  F6  HUD  H  plain-English hints  M  cabin sound",
                    "F7  engine arm (DES, or ASC after landing)  F8  PGNS mode AUTO/ATT HOLD  B  ABORT STAGE  Space  ENGINE STOP",
                    "W/S A/D Q/E or stick/gamepad: hand controller   Up/Down, d-pad or hat: ROD switch (P66)",
                    "",
                    "The crew procedure: R51 V50N25 -> ENTER,  V50N18 -> PRO (maneuver) then ENTER,",
                    "arm the engine, V99N62 -> PRO at TIG-5 s, after radar lock key V57E,",
                    "engine stop at the contact light.  After landing: L lifts off (P12: at V99 B, F7, PRO).",
                    "Docking (in orbit): T/G translate toward/away, arrows sideways, Q/E W/S A/D attitude, Tab to the COAS view",
                    "",
                    "TWO PLAYERS (co-op, UDP port 47012 on the host, both on the same version)",
                    "Host:  menu TWO PLAYERS: HOST, then start a scenario   (or agc-lander --host)",
                    "Join:  menu JOIN A GAME AS LM PILOT, type the host's IP, Enter   (or --join IP)",
                    "Commander (host): flies the LM, ROD switch, Space ENGINE STOP, F5 pause, PgUp/PgDn warp",
                    "LM Pilot (join): DSKY keys, panels, F7 engine arm, F8 PGNS mode, Space ENGINE STOP",
                    "The LM Pilot's stick/keys add to the Commander's.  Esc on the LM Pilot's side leaves.",
                };
                const int n = (int)(sizeof lines / sizeof lines[0]), coop_first = 12;
                float hy = 140;
                DrawRectangleRounded((Rectangle){ 60, hy - 20, 1000, n * 30 + 40 }, 0.05f, 8, (Color){ 0, 0, 0, 210 });
                for (int i = 0; i < n; i++)
                    hud_text(font_mono, lines[i], 80, hy + i * 30, 19,
                             i == coop_first ? (Color){ 120, 220, 255, 255 } : (Color){ 220, 224, 230, 255 });
            }
        }
#ifdef AGC_VR
        if (show_hud || gs == GS_MENU) {
            // How the headset is coping: frame rate, load (above 100% = too slow), resolution in use.
            float load, scale, fps;
            char b[96];
            vr_stats(&load, &scale, &fps);
            snprintf(b, sizeof b, "%.0f fps   load %.0f%%   resolution %.0f%%", fps, load * 100, scale * 100);
            hud_text(font_mono, b, 18, h - 26, 14, (Color){ 120, 128, 138, 200 });
        }
        if (ui_ptr_x >= 0) {   // where the hand / controller points on the display
            DrawCircleV((Vector2){ ui_ptr_x, ui_ptr_y }, 9, (Color){ 120, 255, 170, 230 });
            DrawCircleLinesV((Vector2){ ui_ptr_x, ui_ptr_y }, 13, (Color){ 20, 40, 30, 200 });
        }
        rlDrawRenderBatchActive();
        rlSetBlendMode(RL_BLEND_ALPHA);
        EndTextureMode();
        {
            int wcam = gs == GS_MENU ? CAM_CHASE : vcam;
            int show_ui = gs != GS_FLY || show_hud || show_help;
            if (show_ui) {
                // The display as its own compositor layer, when the headset offers one:
                // sharp text, independent of the eye resolution.
                unsigned int fb = vr_ui_begin(UI_W, UI_H);
                if (fb) {
                    rlDrawRenderBatchActive();
                    rlEnableFramebuffer(fb);
                    rlViewport(0, 0, UI_W, UI_H);
                    rlClearColor(0, 0, 0, 0);
                    rlClearScreenBuffers();
                    rlMatrixMode(RL_PROJECTION);
                    rlPushMatrix();
                    rlLoadIdentity();
                    rlOrtho(0, 1, 0, 1, -1, 1);
                    rlMatrixMode(RL_MODELVIEW);
                    rlPushMatrix();
                    rlLoadIdentity();
                    rlDisableColorBlend();
                    rlDisableDepthTest();
                    rlDisableBackfaceCulling();
                    BeginShaderMode(ui_sh);
                    rlSetTexture(ui_rt.texture.id);
                    rlBegin(RL_QUADS);
                    rlColor4ub(255, 255, 255, 255);
                    rlTexCoord2f(0, 0); rlVertex2f(0, 0);
                    rlTexCoord2f(1, 0); rlVertex2f(1, 0);
                    rlTexCoord2f(1, 1); rlVertex2f(1, 1);
                    rlTexCoord2f(0, 1); rlVertex2f(0, 1);
                    rlEnd();
                    rlDrawRenderBatchActive();
                    rlSetTexture(0);
                    EndShaderMode();
                    rlEnableColorBlend();
                    rlEnableBackfaceCulling();
                    rlMatrixMode(RL_PROJECTION);
                    rlPopMatrix();
                    rlMatrixMode(RL_MODELVIEW);
                    rlPopMatrix();
                    rlDisableFramebuffer();
                    vr_ui_end(v3(0, -UI_DROP, -UI_DIST), UI_WIDTH, UI_WIDTH * UI_H / UI_W);
                    show_ui = 0;   // not drawn into the eyes as well
                }
            }
            VrEye eyes[2];
            ViewCam ecam[2];
            for (int i = 0; i < 2; i++) { vr_eye_begin(i, &eyes[i]); ecam[i] = vr_eye_cam(&cam, &eyes[i]); }
            // Level of detail and shadows are worked out once, for the point between the eyes.
            ViewCam mid = ecam[0];
            mid.pos = v3_scale(v3_add(ecam[0].pos, ecam[1].pos), 0.5);
            world_prepare(&mid, eyes[0].h, wcam);
            M3 Rt = m3_transpose(lm_site_rot());
            for (int i = 0; i < 2; i++) {
                terrain_hold_fade(i == 0);
                cockpit_set_eye_offset(m3_apply(Rt, v3_sub(ecam[i].pos, cam.pos)));
                render_set_direct_target(eyes[i].fbo, 0, 0);
                world_draw(&ecam[i], eyes[i].w, eyes[i].h, film_now, wcam);
                vr_draw_overlays(&cam, &ecam[i], eyes[i].w, eyes[i].h, show_ui, laser, laser_from, laser_to, laser_hit);
                vr_eye_end(i);
            }
            terrain_hold_fade(0);
        }
        vr_end_frame();
#else
        if (capture) {
            EndTextureMode();
            DrawTexturePro(shot_rt.texture, (Rectangle){ 0, 0, 1920, -1080 },
                           (Rectangle){ 0, 0, (float)GetScreenWidth(), (float)GetScreenHeight() }, (Vector2){ 0, 0 }, 0, WHITE);
        }
        if (perf_on) {
            double pf5 = perf_mark();
            EndDrawing();
            double pf6 = GetTime();
            double d[6] = { pf1 - pf0, pf2 - pf1, pf3 - pf2, pf4 - pf3, pf5 - pf4, pf6 - pf5 };
            for (int i = 0; i < 6; i++) perf_t[i] += d[i];
            perf_n++;
            if (perf_n >= 60) {
                fprintf(stderr, "PERF %dx%d cam=%d fps=%d  sim %.1f  pre %.1f  ckatlas %.1f  world %.1f  hud %.1f  swap %.1f ms\n",
                        w, h, camera, GetFPS(), perf_t[0] / perf_n * 1e3, perf_t[1] / perf_n * 1e3, perf_t[2] / perf_n * 1e3,
                        perf_t[3] / perf_n * 1e3, perf_t[4] / perf_n * 1e3, perf_t[5] / perf_n * 1e3);
                memset(perf_t, 0, sizeof perf_t);
                perf_n = 0;
            }
        } else {
            EndDrawing();
        }
#endif

        if (getenv("AGC_DBGJETS") && ((int)(wall * 2) != (int)((wall - dt) * 2))) {
            V3 g = lm_gimbal_angles();
            printf("JETS wall %.1f t %.1f ch5 %03o ch6 %03o rcs %.1f w %+.3f %+.3f %+.3f deg/s  gimb %.2f %.2f %.2f  CDUD %05o %05o %05o  CDU %05o %05o %05o mode %d\n",
                   wall, lm.t, lm.ch5, lm.ch6, lm.m_rcs, lm.w.x * 57.3, lm.w.y * 57.3, lm.w.z * 57.3, g.x * 57.3, g.y * 57.3, g.z * 57.3,
                   agc_get_word("CDUXD", 0), agc_get_word("CDUYD", 0), agc_get_word("CDUZD", 0), agc_get_word("CDUX", 0),
                   agc_get_word("CDUY", 0), agc_get_word("CDUZ", 0), lm_get_attitude_mode());
            fflush(stdout);
        }
        if (getenv("AGC_DBGDOCK") && ((int)(wall * 2) != (int)((wall - dt) * 2))) {
            DockGeo g;
            if (ascent_dock_geo(&g))
                printf("DOCK wall %.1f axial %.2f lat %+.2f %+.2f rate %+.3f drift %+.3f %+.3f angle %.2f roll %+.2f phase %d\n", wall,
                       g.axial, g.lat_y, g.lat_z, g.rate, g.vy, g.vz, g.angle * 57.29578, g.roll * 57.29578, ascent_phase());
            fflush(stdout);
        }
        if (getenv("AGC_DBG") && ((int)(wall * 2) != (int)((wall - dt) * 2))) {
            int dn, ld, pd;
            terrain_stats(&dn, &ld, &pd);
            V3 lp = lm_site_pos();
            printf("DBG wall %.1f: chunks drawn %d loaded %d pending %d | cam (%.0f %.0f %.0f) lm (%.0f %.0f %.0f) alt %.0f\n",
                   wall, dn, ld, pd, cam.pos.x, cam.pos.y, cam.pos.z, lp.x, lp.y, lp.z, lm.altitude);
            fflush(stdout);
        }
        if (outside_on || coop_role == COOP_HOST) {
            StatePkt p;
            pack_state(&p, 0);
            if (outside_on) net_send(&p, sizeof p);
            if (coop_role == COOP_HOST && gs != GS_MENU) coop_host_send_state(&p, sizeof p);
        }
        if (opt.audit && audit_i >= 0 && ++audit_frames >= (audit_i == 0 ? 120 : 10)) {
            char path[1200];
            snprintf(path, sizeof path, "%s/%s.png", opt.audit, audit_views[audit_i].name);
            Image img = LoadImageFromTexture(shot_rt.texture);
            ImageFlipVertical(&img);
            ExportImage(img, path);
            UnloadImage(img);
            audit_frames = 0;
            if (++audit_i >= N_AUDIT) {
                TraceLog(assets_missing() ? LOG_ERROR : LOG_INFO, "AUDIT: %d views saved to %s; textures loaded %d, missing %d",
                         N_AUDIT, opt.audit, assets_loaded(), assets_missing());
                audit_fail = assets_missing() != 0;
                break;
            }
        }
#if defined(AGC_VR) && !defined(__ANDROID__)
        if (opt.shot && wall >= opt.after) { vr_sim_save(opt.shot); break; }
#endif
        if (!render_direct() && opt.shot && wall >= opt.after) {
            // AGC_SEQ=N: save N consecutive frames (SHOT-000.bmp ...) instead of one
            // picture, to check motion for popping.
            static int seq_i;
            const char *seq_env = getenv("AGC_SEQ");
            int seq_n = seq_env ? atoi(seq_env) : 0;
            char seq_path[600];
            snprintf(seq_path, sizeof seq_path, "%s-%03d.bmp", opt.shot, seq_i);
            Image img = LoadImageFromTexture(shot_rt.texture);
            ImageFlipVertical(&img);
            ExportImage(img, seq_n > 0 ? seq_path : opt.shot);
            UnloadImage(img);
            if (++seq_i >= seq_n) break;
        }
    }

    if (outside_on) {
        StatePkt p;
        pack_state(&p, 1);
        net_send(&p, sizeof p);
    }
    net_close();
    coop_stop();
    sound_shutdown();
    cockpit_shutdown();
    terrain_shutdown();
    hud_shutdown();
    render_shutdown();
#ifdef AGC_VR
    vr_shutdown();
#endif
    CloseWindow();
    return audit_fail;
}
