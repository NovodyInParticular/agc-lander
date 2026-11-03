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

#include "agc_host.h"
#include "cockpit.h"
#include "sky_extras.h"
#include "controls.h"
#include "coop.h"
#include "crew.h"
#include "dsky.h"
#include "hints.h"
#include "hud.h"
#include "lm.h"
#include "lmmodel.h"
#include "net.h"
#include "moon.h"
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

static V3 lm_site_pos(void)
{
    V3 fixed = moon_inertial_to_fixed(lm.r, lm.t + lm.tephem);
    return terrain_fixed_to_site(fixed);
}

static M3 lm_site_rot(void)
{
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
    scenario_pre_pdi();
    if (!terrain_ready) {
        terrain_set_sun(sun_site);
        terrain_init(scenario_info.rls);
        terrain_ready = 1;
        rocks_init();
    }
    lm.surface = surface_cb;
}

// ---------------------------------------------------------------------------
// Cameras.

enum { CAM_CHASE, CAM_CDR, CAM_DAC, CAM_TOWER, CAM_CINEMA, CAM_GLOBE, CAM_COUNT };
static const char *cam_names[CAM_COUNT] = { "CHASE", "COCKPIT", "16MM DAC (LMP WINDOW)", "TOWER", "CINEMATIC", "LUNAR ORBIT" };
static double orbit_yaw = -0.9, orbit_pitch = 0.16, orbit_dist = 24;

static ViewCam make_camera(int mode, V3 lm_pos, M3 R, double t_wall)
{
    ViewCam c;
    c.fov_y = 50 * DEG;
    V3 up = local_up_at(lm_pos);
    V3 east = v3_norm(v3_cross(v3(0, 0, 1), up));   // site +Z is south: z x up = east-ish
    if (v3_len(east) < 0.5) east = v3(1, 0, 0);
    V3 north = v3_cross(up, east);
    switch (mode) {
    case CAM_CDR:
        return cockpit_camera(lm_pos, R);
    case CAM_GLOBE:
        return globe_camera(lm_pos);
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
        V3 off = v3_add(v3_scale(up, sin(orbit_pitch) * d),
                        v3_scale(v3_add(v3_scale(east, cos(orbit_yaw)), v3_scale(north, sin(orbit_yaw))), cos(orbit_pitch) * d));
        V3 focus = v3_add(lm_pos, v3_scale(up, 0.5));
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

static void draw_world(const ViewCam *cam, int w, int h, float film, int camera)
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
    terrain_update(cam->pos, cam->fov_y, h);

    // LM shadow map (LM at the origin of the light space).
    render_shadow_begin(rel, sun_site, 8.0f);
    lmmodel_draw_depth(R, v3(0, 0, 0));
    render_shadow_end();

    render_set_film(film);
    render_begin_scene(cam, w, h);
    render_sky(cam, sun_site, earth_site, sun_site);
    render_scene_3d_begin(w, h);

    // Dust kicked up by the descent engine.
    V3 up = local_up_at(lm_pos);
    double ground_dist = lm.altitude + 2.75;
    V3 ground = v3_sub(v3_add(lm_pos, v3_scale(up, -ground_dist)), cam->pos);
    float dust = 0;
    if (lm.thrust > 500 && lm.altitude < 70)
        dust = (float)fmin(1.0, pow(1.0 - lm.altitude / 70.0, 1.3) * lm.thrust / 11000.0);
    float crash_age = lm.land_result == LAND_CRASH ? (float)(lm.t - lm.crash_t) : -1.0f;
    if (crash_age >= 0 && crash_age < 8) {
        // Impact blows a ring of dust out that settles over a few seconds.
        float burst = (float)fmin(1.0, 0.35 + lm.crash_speed / 8.0) * expf(-crash_age / 1.8f);
        if (burst > dust) dust = burst;
    }
    lmmodel_update_dust(GetFrameTime(), dust);
    V3 cam_mod = v3(fmod(cam->pos.x, 4096.0), fmod(cam->pos.y, 4096.0), fmod(cam->pos.z, 4096.0));
    render_set_scene_uniforms(sun_site, cam_mod, local_up_at(cam->pos), 0.11f, ground, dust);

    if (getenv("AGC_NODEPTH")) rlDisableDepthTest();
    terrain_draw(cam->pos, render_terrain_shader());
    rlEnableDepthTest();
    rocks_draw(cam->pos);
    if (camera == CAM_CDR) render_set_lm_extras(COCKPIT_NEAR_CUT, 1.0f, NULL, NULL, 0);
    lmmodel_draw(R, rel);
    render_set_lm_extras(0.0f, 1.0f, NULL, NULL, 0);

    LmFx fx = { 0 };
    fx.thrust = (float)lm.thrust;
    for (int j = 0; j < LM_NJETS; j++) {
        fx.jet[j] = lm.jet_level[j];
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
};
static const char *scen_desc[] = {
    "Luminary099 is in P63 waiting for the crew. The real procedure: fine-align option, auto maneuver, arm the descent engine, enable ignition at V99, then permit landing radar updates with V57.",
    "Fast-forwards the real AGC and crew through the pre-burn checklist and stops 30 seconds before ignition.",
    "Fast-forwards through the braking phase. Pitch-over is just ahead and Tranquility Base comes into view.",
    "The computer brings you to 150 m. Then it's yours: ROD switch (Up/Down arrows) in P66, ENGINE STOP at contact.",
};

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

    hud_text_centered(font_ui, client ? "Waiting for the Commander.   Space hides this   Esc  menu"
                                      : "Enter  fly it again      Space  hide this      Esc  menu",
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

int main(int argc, char **argv)
{
    Options opt;
    parse_args(argc, argv, &opt);

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
    InitWindow(1600, 900, opt.spectator ? "AGC Lander - Outside View" : "AGC Lander - Apollo 11 / Luminary099");
#ifdef _WIN32
    GetModuleFileNameA(NULL, self_exe, sizeof self_exe);
#else
    {
        ssize_t n = readlink("/proc/self/exe", self_exe, sizeof self_exe - 1);
        if (n > 0) self_exe[n] = 0;
        else snprintf(self_exe, sizeof self_exe, "%s", argv[0]);
    }
#endif
    if (!opt.shot && !opt.windowed) set_fullscreen(1, opt.monitor);
    else if (opt.monitor >= 0 && opt.monitor < GetMonitorCount()) SetWindowMonitor(opt.monitor);
    int outside_on = 0;
    double last_pkt = 0;
    if (opt.spectator) {
        if (net_open_listener(NET_PORT) != 0) TraceLog(LOG_WARNING, "outside view: port %d busy", NET_PORT);
        opt.start = 0;
    }
    SetExitKey(KEY_NULL);
    SetTargetFPS(60);

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
    cockpit_init();
    extras_init();
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
    }
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
    if (opt.shot) shot_rt = LoadRenderTexture(1920, 1080);

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt > 0.1f) dt = 0.1f;
        wall += dt;
        int w = GetScreenWidth(), h = GetScreenHeight();
        if (opt.shot) { w = 1920; h = 1080; }
        render_set_output(w, h);
        render_set_output_fbo(opt.shot ? shot_rt.id : 0);
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
            if (IsKeyPressed(KEY_DOWN)) menu_sel = (menu_sel + 1) % 7;
            if (IsKeyPressed(KEY_UP)) menu_sel = (menu_sel + 6) % 7;
            int enter = IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER) || retry_req;
            retry_req = 0;
            if (menu_sel == 6 && (enter || IsKeyPressed(KEY_LEFT) || IsKeyPressed(KEY_RIGHT))) {
                hints_on = !hints_on;
                hints_save();
                enter = 0;
            }
            if (menu_sel == 4 && (enter || IsKeyPressed(KEY_LEFT) || IsKeyPressed(KEY_RIGHT))) {
                coop_host_on = !coop_host_on;
                enter = 0;
            }
            if (menu_sel == 5) {
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
                show_dsky = scen == 0 && camera != CAM_CDR;   // F4 shows it any time
                ff_target_alt = scen == 2 ? 2600 : scen == 3 ? 150 : 0;
                gs = scen == 0 ? GS_FLY : GS_FASTFWD;
                if (scen == 0) crew.auto_crew = 0;
            }
        } else {
            if (IsKeyPressed(KEY_TAB)) camera = (camera + 1) % CAM_COUNT;
            if (camera != prev_camera) {
                // The cockpit has its own DSKY: hide the side one there.
                if (camera == CAM_CDR) show_dsky = 0;
                else if (prev_camera == CAM_CDR) show_dsky = scen == 0;   // only the full mission needs it
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
            if (IsKeyPressed(KEY_F7)) lm_set_engine_arm(!lm.engine_armed);
            if (IsKeyPressed(KEY_H)) {
                hints_on = !hints_on;
                hints_save();
                crew_say(hints_on ? "Hints on (H hides them)" : "Hints off (H shows them)");
            }
            if (lm.land_result != LAND_NONE && !lm.dps_on) {
                if (IsKeyPressed(KEY_SPACE)) results_hidden = !results_hidden;
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

        // ---------------- Rendering ----------------
        if (!opt.spectator && gs != GS_MENU) dsky_frame();
        int scene_w0 = w;
        if (opt.look_set) { cockpit_set_view(opt.look_yaw, opt.look_pitch); opt.look_set = 0; }
        if (gs != GS_MENU && camera == CAM_CDR) cockpit_input(scene_w0, h);
        if (gs != GS_MENU && camera == CAM_GLOBE) globe_input(scene_w0, h);
        ViewCam cam = make_camera(camera, lm_site_pos(), lm_site_rot(), wall);
        if (lm.land_result == LAND_CRASH && camera != CAM_GLOBE) {
            // Impact shake, strongest in the cockpit.
            double age = lm.t - lm.crash_t;
            double amp = (camera == CAM_CDR ? 0.06 : 0.25) * fmin(1.0, lm.crash_speed / 6.0) * exp(-age * 1.6);
            if (age >= 0 && amp > 0.002)
                cam.pos = v3_add(cam.pos, v3(amp * sin(wall * 61.0), amp * sin(wall * 47.0 + 1.0), amp * sin(wall * 53.0 + 2.0)));
        }
        extras_update(dt, gs == GS_FLY && !paused && !opt.spectator, &cam);
        float film_now = film > 0 || camera == CAM_DAC ? 1.0f : 0.0f;
        if ((show_hud && gs != GS_MENU) || camera == CAM_CDR) hud_prepare();
        if (gs != GS_MENU && !opt.spectator) cockpit_update(dt);
        if (camera == CAM_CDR) cockpit_prepare(lm_site_rot(), sun_site, local_up_at(lm_site_pos()), dt);
        BeginDrawing();
        if (opt.shot) BeginTextureMode(shot_rt);
        ClearBackground(BLACK);
        int scene_w = w;
        draw_world(&cam, scene_w > 64 ? scene_w : w, h, film_now, gs == GS_MENU ? CAM_CHASE : camera);
        (void)draw_lpd;

        if (gs == GS_MENU) {
            DrawRectangleGradientV(0, 0, w, h, (Color){ 0, 0, 0, 120 }, (Color){ 0, 0, 0, 230 });
            hud_text(font_bold, "AGC LANDER", 80, 90, 84, (Color){ 235, 235, 235, 255 });
            hud_text(font_ui, "Apollo 11 Lunar Module  -  flown by the original Luminary099 flight software on an emulated AGC",
                     84, 182, 24, (Color){ 170, 176, 186, 255 });
            for (int i = 0; i < 4; i++) {
                Color c = i == menu_sel ? (Color){ 255, 190, 60, 255 } : (Color){ 200, 205, 212, 255 };
                hud_text(font_bold, scen_names[i], 100, 250 + i * 56, 30, c);
            }
            {
                Color c4 = menu_sel == 4 ? (Color){ 255, 190, 60, 255 } : (Color){ 200, 205, 212, 255 };
                Color c5 = menu_sel == 5 ? (Color){ 255, 190, 60, 255 } : (Color){ 200, 205, 212, 255 };
                hud_text(font_bold, coop_host_on ? "TWO PLAYERS:  HOST  -  you are the Commander" : "TWO PLAYERS:  OFF", 100, 250 + 4 * 56 + 16, 30, c4);
                char jl[128];
                snprintf(jl, sizeof jl, "JOIN A GAME AS LM PILOT:  %s%s", join_ip[0] ? join_ip : "type the host's IP",
                         menu_sel == 5 && ((int)(GetTime() * 2) & 1) ? "_" : "");
                hud_text(font_bold, jl, 100, 250 + 5 * 56 + 16, 30, c5);
                Color c6 = menu_sel == 6 ? (Color){ 255, 190, 60, 255 } : (Color){ 200, 205, 212, 255 };
                hud_text(font_bold, hints_on ? "PLAIN-ENGLISH HINTS:  ON" : "PLAIN-ENGLISH HINTS:  OFF", 100, 250 + 6 * 56 + 32, 30, c6);
            }
            const char *desc = menu_sel < 4 ? scen_desc[menu_sel]
                             : menu_sel == 6 ? "Enter toggles. A box at the top of the screen tells you what to do next in plain words: which key, which way to lean, when to cut the engine. H toggles it in flight."
                             : menu_sel == 4 ? "Enter toggles. With HOST on, start any scenario above: a second player can join you as LM Pilot (UDP port 47012). You fly; they run the DSKY and the panels."
                                             : "Type the Commander's IP address (or name) and press Enter. You get the LMP station: DSKY, panels, switches. Same version of the game on both sides.";
            hud_text(font_ui, desc, 100, 250 + 7 * 56 + 40, 21, (Color){ 160, 168, 178, 255 });
            hud_text(font_ui, "Up/Down to choose, Enter to start.  In flight: F1 for help.", 100, h - 70, 22,
                     (Color){ 130, 136, 146, 255 });
        } else {
            hud_set_minimal(camera == CAM_CDR || camera == CAM_GLOBE);
            if (show_hud) hud_draw(w, h, 0, (float)warp, cam_names[camera], paused);
            if (show_hud && gs == GS_FLY) hints_draw(w, h, 0);
            if (gs == GS_FLY && lm.land_result != LAND_NONE && !lm.dps_on && !results_hidden &&
                lm.t - (lm.land_result == LAND_CRASH ? lm.crash_t : lm.touchdown_t) > 3)
                draw_results(scene_w, h, coop_role == COOP_CLIENT);
            if (camera == CAM_CDR && gs == GS_FLY) cockpit_overlay(scene_w, h);
            if (coop_role != COOP_OFF) {
                float sc = h / 1080.0f;
                if (sc < 0.7f) sc = 0.7f;
                hud_text(font_mono, coop_status(), 18 * sc, 166 * sc, 17 * sc,
                         coop_peer_connected() ? (Color){ 120, 220, 255, 255 } : (Color){ 255, 190, 60, 255 });
            }
            if (camera == CAM_GLOBE) globe_overlay(&cam, scene_w, h, lm_site_pos());
            if (gs == GS_FASTFWD) {
                DrawRectangle(0, h / 2 - 50, scene_w, 100, (Color){ 0, 0, 0, 170 });
                char b[96];
                snprintf(b, sizeof b, "FAST-FORWARDING THE AGC   GET %03d:%02d:%02d   ALT %.0f ft",
                         (int)(lm.t / 3600), (int)(lm.t / 60) % 60, (int)fmod(lm.t, 60), lm.altitude / 0.3048);
                hud_text_centered(font_bold, b, scene_w / 2.0f, h / 2 - 18, 30, (Color){ 255, 190, 60, 255 });
            }
            if (show_dsky) {
                float dh = dsky_w * DSKY_H / DSKY_W, m = h * 0.015f;
                dsky_update_draw((Rectangle){ w - dsky_w - m, h - dh - m, (float)dsky_w, dh });
            }
            if (show_help) {
                const char *lines[] = {
                    "DSKY  0-9  V verb  N noun  Enter  C clr  R rset  P pro  K key rel  = +  - -",
                    "Tab  camera (chase, cockpit, 16mm, tower, cinematic, lunar orbit)   Right-drag look/orbit",
                    "PgUp/PgDn  time warp   F5  pause   Esc  menu   F10  outside view on 2nd monitor   F11  fullscreen",
                    "F2  auto crew on/off  F3  16mm film  F4  DSKY  F6  HUD  H  plain-English hints",
                    "F7  descent engine arm    F8  PGNS mode AUTO/ATT HOLD    Space  ENGINE STOP",
                    "W/S A/D Q/E or stick/gamepad: hand controller   Up/Down, d-pad or hat: ROD switch (P66)",
                    "",
                    "The crew procedure: R51 V50N25 -> ENTER,  V50N18 -> PRO (maneuver) then ENTER,",
                    "arm the engine, V99N62 -> PRO at TIG-5 s, after radar lock key V57E,",
                    "engine stop at the contact light.",
                };
                float hy = 140;
                DrawRectangleRounded((Rectangle){ 60, hy - 20, 980, 10 * 30 + 40 }, 0.05f, 8, (Color){ 0, 0, 0, 210 });
                for (int i = 0; i < 10; i++) hud_text(font_mono, lines[i], 80, hy + i * 30, 19, (Color){ 220, 224, 230, 255 });
            }
        }
        if (opt.shot) {
            EndTextureMode();
            DrawTexturePro(shot_rt.texture, (Rectangle){ 0, 0, 1920, -1080 },
                           (Rectangle){ 0, 0, (float)GetScreenWidth(), (float)GetScreenHeight() }, (Vector2){ 0, 0 }, 0, WHITE);
        }
        EndDrawing();

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
        if (opt.shot && wall >= opt.after) {
            Image img = LoadImageFromTexture(shot_rt.texture);
            ImageFlipVertical(&img);
            ExportImage(img, opt.shot);
            UnloadImage(img);
            break;
        }
    }

    if (outside_on) {
        StatePkt p;
        pack_state(&p, 1);
        net_send(&p, sizeof p);
    }
    net_close();
    coop_stop();
    cockpit_shutdown();
    terrain_shutdown();
    hud_shutdown();
    render_shutdown();
    CloseWindow();
    return 0;
}
