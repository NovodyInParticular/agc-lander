// Moonwalk: see eva.h.
//
// The astronaut is a point on the terrain's plane coordinates with a height
// above the ground; everything else (eye, flag) is placed from that through
// the terrain's own conversions, so he walks the same craters the lander saw.
#include <math.h>
#include <stdio.h>

#include "raylib.h"

#include "input.h"
#include "ascent.h"
#include "crew.h"
#include "eva.h"
#include "hud.h"
#include "lm.h"
#include "lmmodel.h"
#include "sound.h"
#include "terrain.h"

#define MOON_G 1.62
#define EYE_H 1.62          // eye above the boot soles, m
#define WALK_V 1.5          // m/s
#define LOPE_V 2.6
#define JUMP_V 1.9          // m/s: about 1.1 m up, 2.3 s off the ground
#define HOP_V 0.55          // the loping gait leaves the ground at every stride
#define LM_KEEPOUT 2.4      // m from the LM's centreline
#define LADDER_REACH 3.5    // m from the foot of the ladder

#define T_OPEN 4.0f         // kneel down and swing the hatch in
#define T_CRAWL 1.6f        // through the opening
#define T_LADDER 6.0f       // porch and ladder

static int phase;
static float t_phase;       // seconds in the current phase
static double px, pz;       // plane coordinates
static double alt, vy;      // height of the boot soles (terrain datum), vertical speed
static double vx, vz;       // horizontal velocity along the plane axes
static double yaw, pitch;   // view: yaw from north through east, pitch up
static int grounded;
static double foot_x, foot_z;   // foot of the ladder
static int flag_on;
static double flag_x, flag_z, flag_yaw;
static V3 lm_pos;
static M3 lm_R;
static int vr_mode;         // headset: the head does the looking
static double head_yaw;     // headset: where the head points, relative to the body's yaw

void eva_set_vr(double head_yaw_rad) { vr_mode = 1; head_yaw = head_yaw_rad; }
void eva_turn(double rad) { yaw += rad; }

void eva_reset(void)
{
    phase = EVA_INSIDE;
    t_phase = 0;
    flag_on = 0;
    vx = vz = vy = 0;
}

int eva_phase(void) { return phase; }

int eva_available(void)
{
    return lm.touchdown && !lm.dps_on && lm.land_result != LAND_CRASH && lm.land_result != LAND_NONE && !lm.staged &&
           !ascent_active();
}

int eva_outside(void) { return phase == EVA_EGRESS || phase == EVA_SURFACE || phase == EVA_INGRESS; }
int eva_at_hatch(void) { return phase == EVA_HATCH_OPENING || phase == EVA_HATCH_CLOSING; }

static float smooth(float x) { x = x < 0 ? 0 : x > 1 ? 1 : x; return x * x * (3 - 2 * x); }

// Time into the hatch sequence, running forward when leaving and backward when returning.
static float hatch_clock(void)
{
    if (phase == EVA_HATCH_OPENING) return t_phase;
    if (phase == EVA_HATCH_CLOSING) return T_OPEN + T_CRAWL - t_phase;
    return phase == EVA_INSIDE ? 0 : T_OPEN + T_CRAWL;
}

float eva_hatch(void) { return smooth((hatch_clock() - 1.2f) / (T_OPEN - 1.4f)); }

float eva_kneel(void)
{
    float c = hatch_clock();
    return c < T_OPEN ? smooth(c / 1.6f) : 1.0f + smooth((c - T_OPEN) / T_CRAWL);
}

// Local frame at a plane point: site position at height h, and unit east / north / up.
static V3 frame_at(double x, double z, double h, V3 *east, V3 *north, V3 *up)
{
    V3 p = terrain_plane_to_site(x, z, h);
    V3 e = v3_norm(v3_sub(terrain_plane_to_site(x + 1, z, h), p));
    V3 s = v3_norm(v3_sub(terrain_plane_to_site(x, z + 1, h), p));
    if (east) *east = e;
    if (north) *north = v3_scale(s, -1);
    if (up) *up = v3_norm(v3_cross(s, e));
    return p;
}

static V3 body_point(double x, double y, double z) { return v3_add(lm_pos, m3_apply(lm_R, v3(x, y, z))); }

static void start_surface(void)
{
    // Stand a pace in front of the forward footpad, facing the LM.
    terrain_site_to_plane(body_point(-2.75, 0, 5.3), &foot_x, &foot_z);
    px = foot_x;
    pz = foot_z;
    alt = terrain_height(px, pz, 0);
    vx = vz = vy = 0;
    grounded = 1;
    V3 east, north;
    frame_at(px, pz, alt, &east, &north, NULL);
    V3 f = m3_apply(lm_R, v3(0, 0, -1));
    yaw = atan2(v3_dot(f, east), v3_dot(f, north));
    pitch = 0.05;
}

void eva_key(void)
{
    switch (phase) {
    case EVA_INSIDE:
        if (!eva_available()) {
            crew_say("The hatch stays shut until we are down in one piece with the engine off");
            return;
        }
        phase = EVA_HATCH_OPENING;
        t_phase = 0;
        crew_say("Cabin depressurised. Hatch coming open");
        return;
    case EVA_SURFACE: {
        double d = hypot(px - foot_x, pz - foot_z);
        if (d > LADDER_REACH) {
            char b[96];
            snprintf(b, sizeof b, "Walk back to the foot of the ladder to climb in (%.0f m away)", d);
            crew_say(b);
            return;
        }
        phase = EVA_INGRESS;
        t_phase = 0;
        crew_say("Up the ladder");
        return;
    }
    default:
        return;   // already on the way in or out
    }
}

static void walk(float dt)
{
    // Looking: right-drag with the mouse, or the arrow keys.
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        Vector2 d = GetMouseDelta();
        yaw += d.x * 0.0032;
        pitch -= d.y * 0.0032;
    }
    yaw += ((IsKeyDown(KEY_RIGHT) ? 1 : 0) - (IsKeyDown(KEY_LEFT) ? 1 : 0)) * 1.6 * dt;
    pitch += ((IsKeyDown(KEY_UP) ? 1 : 0) - (IsKeyDown(KEY_DOWN) ? 1 : 0)) * 1.0 * dt;
    if (pitch > 1.35) pitch = 1.35;
    if (pitch < -1.35) pitch = -1.35;
    if (vr_mode) pitch = 0;   // the base view stays level; the head looks up and down

    // Plane axes: +x east, +z south. On the headset "forward" is where the head points.
    double f = (IsKeyDown(KEY_W) ? 1 : 0) - (IsKeyDown(KEY_S) ? 1 : 0);
    double r = (IsKeyDown(KEY_D) ? 1 : 0) - (IsKeyDown(KEY_A) ? 1 : 0);
    double wy = yaw + (vr_mode ? head_yaw : 0);
    double wx = sin(wy) * f + cos(wy) * r, wz = -cos(wy) * f + sin(wy) * r;
    double wl = hypot(wx, wz);
    if (wl > 1) { wx /= wl; wz /= wl; }
    double speed = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT) ? LOPE_V : WALK_V;
    double grip = grounded ? 5.0 : 0.4;   // little to push against while off the ground
    double k = fmin(1.0, grip * dt);
    vx += (wx * speed - vx) * k;
    vz += (wz * speed - vz) * k;

    if (grounded) {
        if (IsKeyPressed(KEY_SPACE)) { vy = JUMP_V; grounded = 0; }
        else if (wl > 0.1) { vy = HOP_V * (speed / WALK_V); grounded = 0; }
    }

    px += vx * dt;
    pz += vz * dt;
    // Keep clear of the descent stage.
    double cx, cz;
    terrain_site_to_plane(lm_pos, &cx, &cz);
    double dx = px - cx, dz = pz - cz, d = hypot(dx, dz);
    if (d < LM_KEEPOUT && d > 1e-6) {
        px = cx + dx / d * LM_KEEPOUT;
        pz = cz + dz / d * LM_KEEPOUT;
    }

    vy -= MOON_G * dt;
    alt += vy * dt;
    double ground = terrain_height(px, pz, 0);
    if (alt <= ground) {
        if (!grounded && vy < -1.0) sound_touchdown((float)(-vy * 0.25));   // boots, felt through the suit
        alt = ground;
        vy = 0;
        grounded = 1;
    } else if (alt - ground > 0.03) {
        grounded = 0;
    }

    if (IsKeyPressed(KEY_F)) {
        if (flag_on) {
            crew_say("The flag is already up");
        } else if (!grounded) {
            crew_say("Both feet on the ground first");
        } else {
            double fy = yaw + (vr_mode ? head_yaw : 0);
            flag_x = px + sin(fy) * 1.3;
            flag_z = pz - cos(fy) * 1.3;
            flag_yaw = fy;
            flag_on = 1;
            crew_say("The flag is planted");
            sound_touchdown(0.4f);
        }
    }
}

void eva_update(float dt, V3 pos, M3 R)
{
    lm_pos = pos;
    lm_R = R;
    if (phase != EVA_INSIDE && lm.land_result == LAND_NONE) eva_reset();   // a new flight began
    t_phase += dt;
    switch (phase) {
    case EVA_HATCH_OPENING:
        if (t_phase >= T_OPEN + T_CRAWL) { phase = EVA_EGRESS; t_phase = 0; start_surface(); }
        break;
    case EVA_EGRESS:
        if (t_phase >= T_LADDER) {
            phase = EVA_SURFACE;
            t_phase = 0;
            crew_say("On the surface. That's one small step");
        }
        break;
    case EVA_SURFACE:
        walk(dt);
        break;
    case EVA_INGRESS:
        if (t_phase >= T_LADDER) { phase = EVA_HATCH_CLOSING; t_phase = 0; }
        break;
    case EVA_HATCH_CLOSING:
        if (t_phase >= T_OPEN + T_CRAWL) { phase = EVA_INSIDE; t_phase = 0; crew_say("Hatch closed and latched"); }
        break;
    }
}

// Debugging (AGC_EVA=hatch|crawl|ladder|walk|flag): jump straight to a point in the sequence.
void eva_debug(const char *what, V3 pos, M3 R)
{
    lm_pos = pos;
    lm_R = R;
    start_surface();
    switch (what[0]) {
    case 'h': phase = EVA_HATCH_OPENING; t_phase = T_OPEN - 0.05f; break;
    case 'c': phase = EVA_HATCH_OPENING; t_phase = T_OPEN + T_CRAWL * 0.5f; break;
    case 'l': phase = EVA_EGRESS; t_phase = T_LADDER * 0.55f; break;
    case 'f':
        phase = EVA_SURFACE;
        px += 4.0; pz += 6.0;
        alt = terrain_height(px, pz, 0);
        yaw += 0.35;
        flag_x = px + sin(yaw) * 2.2 - cos(yaw) * 0.9;
        flag_z = pz - cos(yaw) * 2.2 - sin(yaw) * 0.9;
        flag_yaw = yaw;
        flag_on = 1;
        break;
    default: phase = EVA_SURFACE; break;
    }
}

ViewCam eva_camera(void)
{
    ViewCam c = { 0 };
    c.fov_y = 70 * PI_D / 180;
    V3 east, north, up;
    V3 stand = frame_at(px, pz, alt + EYE_H, &east, &north, &up);
    V3 look = v3_add(v3_scale(v3_add(v3_scale(east, sin(yaw)), v3_scale(north, cos(yaw))), cos(pitch)), v3_scale(up, sin(pitch)));
    if (phase == EVA_SURFACE) {
        c.pos = stand;
        c.fwd = v3_norm(look);
        c.upv = up;
    } else {
        // Backing out: on the porch, down the ladder facing the LM, then onto the surface.
        float u = t_phase / T_LADDER;
        if (phase == EVA_INGRESS) u = 1 - u;
        V3 a = body_point(0.92, 0, 2.30), b = body_point(0.80, 0, 3.25), d = body_point(-1.25, 0, 4.60);
        float s = smooth(u);
        V3 p = s < 0.22f ? v3_add(a, v3_scale(v3_sub(b, a), s / 0.22f)) :
               s < 0.80f ? v3_add(b, v3_scale(v3_sub(d, b), (s - 0.22f) / 0.58f)) :
                           v3_add(d, v3_scale(v3_sub(stand, d), (s - 0.80f) / 0.20f));
        V3 face = v3_norm(m3_apply(lm_R, v3(-0.12, 0, -1)));
        float turn = smooth((u - 0.8f) / 0.2f);
        c.pos = p;
        c.fwd = v3_norm(v3_add(v3_scale(face, 1 - turn), v3_scale(look, turn)));
        c.upv = v3_norm(v3_add(v3_scale(m3_apply(lm_R, v3(1, 0, 0)), 1 - turn), v3_scale(up, turn)));
    }
    V3 right = v3_norm(v3_cross(c.fwd, c.upv));
    c.upv = v3_cross(right, c.fwd);
    return c;
}

void eva_draw(const ViewCam *cam)
{
    if (!flag_on) return;
    V3 east, north, up;
    V3 base = frame_at(flag_x, flag_z, terrain_height(flag_x, flag_z, 0), &east, &north, &up);
    // Flag axes: X up the pole, Z along the cloth (to the planter's right), Y = Z x X.
    V3 zf = v3_add(v3_scale(east, cos(flag_yaw)), v3_scale(north, -sin(flag_yaw)));
    V3 yf = v3_cross(zf, up);
    M3 R = { { { up.x, yf.x, zf.x }, { up.y, yf.y, zf.y }, { up.z, yf.z, zf.z } } };
    lmmodel_draw_flag(R, v3_sub(base, cam->pos));
}

void eva_overlay(int w, int h)
{
    float s = h / 1080.0f;
    if (s < 0.7f) s = 0.7f;
    // Through the hatch: the cabin view fades out, the outside view fades in.
    float fade = 0;
    if (eva_at_hatch()) fade = smooth((hatch_clock() - (T_OPEN + T_CRAWL - 0.7f)) / 0.7f);
    if (phase == EVA_EGRESS) fade = 1 - smooth(t_phase / 0.8f);
    if (phase == EVA_INGRESS) fade = smooth((t_phase - (T_LADDER - 0.8f)) / 0.8f);
    if (fade > 0.003f) DrawRectangle(0, 0, w, h, (Color){ 0, 0, 0, (unsigned char)(255 * fade) });

    const char *msg = NULL;
    char b[200];
    if (phase == EVA_INSIDE && eva_available()) {
        msg = "X   open the hatch and go outside";
    } else if (phase == EVA_SURFACE) {
        double d = hypot(px - foot_x, pz - foot_z);
#ifdef AGC_VR
        snprintf(b, sizeof b, "MOONWALK    left stick walk    left grip lope    A jump    right stick turn    %s    %s",
                 flag_on ? "flag planted" : "left trigger plant the flag",
                 d <= LADDER_REACH ? "X climb back in" : "X at the ladder: climb back in");
#else
        snprintf(b, sizeof b, "MOONWALK    W A S D walk    Shift lope    Space jump    right-drag or arrows look    %s    %s",
                 flag_on ? "flag planted" : "F plant the flag",
                 d <= LADDER_REACH ? "X climb back in" : "X at the ladder: climb back in");
#endif
        msg = b;
    }
    if (!msg) return;
    Vector2 m = MeasureTextEx(font_ui, msg, 20 * s, 0.4f);
    float x = (w - m.x) / 2, y = h - 74 * s;
    DrawRectangleRounded((Rectangle){ x - 12, y - 6, m.x + 24, m.y + 12 }, 0.3f, 6, (Color){ 8, 12, 16, 200 });
    DrawTextEx(font_ui, msg, (Vector2){ x, y }, 20 * s, 0.4f, (Color){ 235, 238, 240, 255 });
}
