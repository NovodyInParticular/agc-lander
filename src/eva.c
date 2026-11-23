// Moonwalk: see eva.h.
//
// Each crewman is a point on the terrain's plane coordinates with a height
// above the reference sphere (his boot soles); everything else (eye, figure,
// flag) is placed from that through the terrain's own conversions, so the
// crew walk the same craters the lander saw.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"

#include "input.h"   // headset: the controllers arrive as keys (IsKeyDown is remapped there)
#include "astro.h"
#include "coop.h"
#include "crew.h"
#include "eva.h"
#include "footprints.h"
#include "hud.h"
#include "lm.h"
#include "lmmodel.h"
#include "sound.h"
#include "terrain.h"
#include "terrain_shape.h"

#define MOON_G 1.62
#define EYE_H 1.62          // eye above the boot soles, m
#define WALK_V 1.5          // m/s
#define LOPE_V 2.6
#define JUMP_V 1.9          // m/s: about 1.1 m up, 2.3 s off the ground
#define HOP_V 0.55          // the loping gait leaves the ground at every stride
#define LM_KEEPOUT 2.4      // m from the LM's centreline
#define LADDER_REACH 3.5    // m from the foot of the ladder
#define STEP_UP 0.35        // a block lower than this (above the soles) is stepped onto; a taller one stops you
#define BODY_R 0.32         // the suit and pack, for bumping into blocks
#define CREW_GAP 0.9        // the two crewmen keep at least this far apart
#define ROCK_MIN 0.12       // stones smaller than this are walked over

#define T_OPEN 4.0f         // kneel down and swing the hatch in
#define T_CRAWL 1.6f        // through the opening
#define T_LADDER 6.0f       // porch and ladder
#define LMP_DELAY 9.0f      // playing alone: the LM Pilot follows this long after the Commander is down

enum { ACT_NONE, ACT_FLAG, ACT_SALUTE };
enum { AI_FOLLOW, AI_FLAG, AI_GO_IN };
#define FLAG_ACT_T 1.4f     // planting the flag: arms forward, pole pushed down
#define SALUTE_T 3.0f

typedef struct {
    int phase;
    float t_phase;          // seconds in the current phase
    double px, pz;          // plane coordinates
    double alt, vy;         // height of the boot soles (terrain datum), vertical speed
    double vx, vz;          // horizontal velocity along the plane axes
    double yaw;             // body facing, radians from north through east
    double pitch;           // head pitch (the local crewman's view)
    int grounded;
    float stride, speed, land_k, air;   // gait phase; horizontal speed; landing squat 0..1; time off the ground
    int act;
    float act_t;
    int carry;              // holding the flag (headset hand, or the planting action)
    // The computer-run LM Pilot.
    int ai, ai_mode;
    float ai_t, stuck_t, avoid_t;
    double ai_tx, ai_tz, anchor_x, anchor_z, avoid;
    int want_in;            // asked to climb in; waiting for the other crewman to go first
    int remote_seen;
} Walker;

static Walker cw[2];
static int me;                  // the local crewman, -1 = none
static double view_yaw;         // the local view's yaw (the body turns toward it)
static double foot_x, foot_z;   // foot of the ladder
static double face_yaw;         // standing there facing the LM
static int flag_on;
static double flag_x, flag_z, flag_yaw;
static double blast_age = -1;   // lift-off blast on the flag
static V3 blast_from;
static int lmp_followed;        // the computer LM Pilot has gone out after this egress
static V3 lm_pos;
static M3 lm_R;
static int vr_mode;             // headset: the head does the looking
static double head_yaw;         // headset: where the head points, relative to the body's yaw
static V3 vr_hand[2];
static int vr_hands_on;

void eva_set_vr(double head_yaw_rad) { vr_mode = 1; head_yaw = head_yaw_rad; }
void eva_turn(double rad) { view_yaw += rad; }

static Walker *mine(void) { return me >= 0 ? &cw[me] : NULL; }
static int solo(void) { return coop_role == COOP_OFF; }

void eva_reset(void)
{
    for (int i = 0; i < 2; i++) {
        memset(&cw[i], 0, sizeof cw[i]);
        cw[i].phase = EVA_INSIDE;
    }
    flag_on = 0;
    blast_age = -1;
    lmp_followed = 0;
    footprints_reset();
}

void eva_set_role(int who) { me = who; }

int eva_phase(void) { return me >= 0 ? cw[me].phase : EVA_INSIDE; }
int eva_all_inside(void) { return cw[0].phase == EVA_INSIDE && cw[1].phase == EVA_INSIDE; }

int eva_flag_planted(void) { return flag_on; }

int eva_available(void)
{
    // On the pad before lift-off too: P12 just waits at V99 until the crew is back.
    return lm.touchdown && !lm.dps_on && lm.land_result != LAND_CRASH && lm.land_result != LAND_NONE && !lm.staged &&
           !lm.abort_stage && !lm.aps_on && !lm.liftoff;
}

static int outside(const Walker *w) { return w->phase == EVA_EGRESS || w->phase == EVA_SURFACE || w->phase == EVA_INGRESS; }
static int at_hatch(const Walker *w) { return w->phase == EVA_HATCH_OPENING || w->phase == EVA_HATCH_CLOSING; }
int eva_outside(void) { return me >= 0 && outside(&cw[me]); }
int eva_at_hatch(void) { return me >= 0 && at_hatch(&cw[me]); }

static float smooth(float x) { x = x < 0 ? 0 : x > 1 ? 1 : x; return x * x * (3 - 2 * x); }

// Time into the hatch sequence, running forward when leaving and backward when returning.
static float hatch_clock(const Walker *w)
{
    if (w->phase == EVA_HATCH_OPENING) return w->t_phase;
    if (w->phase == EVA_HATCH_CLOSING) return T_OPEN + T_CRAWL - w->t_phase;
    return w->phase == EVA_INSIDE ? 0 : T_OPEN + T_CRAWL;
}

float eva_hatch(void)
{
    // Open while anyone is outside; otherwise it follows whoever is at it (the
    // one furthest out, if both are: it shuts only behind the last man in).
    const Walker *w = NULL;
    for (int i = 0; i < 2; i++) {
        if (outside(&cw[i])) return 1;
        if (at_hatch(&cw[i]) && (!w || hatch_clock(&cw[i]) > hatch_clock(w))) w = &cw[i];
    }
    return w ? smooth((hatch_clock(w) - 1.2f) / (T_OPEN - 1.4f)) : 0;
}

float eva_kneel(void)
{
    if (me < 0) return 0;
    float c = hatch_clock(&cw[me]);
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

// --- Ground and boulders -------------------------------------------------------------

// The blocks near the crew, refreshed as they walk.
#define NNEAR 512
static struct { double x, z, rh, base, sy; } near_rock[NNEAR];
static int nnear;
static double near_cx = 1e30, near_cz = 1e30;

static void refresh_rocks(double x, double z)
{
    if (hypot(x - near_cx, z - near_cz) < 10) return;
    near_cx = x;
    near_cz = z;
    nnear = 0;
    const TerrainRock *r;
    int n = terrain_rocks(&r);
    for (int i = 0; i < n && nnear < NNEAR; i++) {
        if (r[i].r < ROCK_MIN) continue;
        double dx = r[i].x - x, dz = r[i].z - z;
        if (dx * dx + dz * dz > 30.0 * 30.0) continue;
        near_rock[nnear].x = r[i].x;
        near_rock[nnear].z = r[i].z;
        near_rock[nnear].rh = 0.5 * (r[i].sx + r[i].sz);
        near_rock[nnear].base = 0.15 * r[i].r;
        near_rock[nnear].sy = r[i].sy;
        nnear++;
    }
}

// Height of any block under a point, above the ground there (as terrain_rock_height, smaller stones too).
static double rock_at(double x, double z)
{
    double best = 0;
    for (int i = 0; i < nnear; i++) {
        double dx = x - near_rock[i].x, dz = z - near_rock[i].z, rh = near_rock[i].rh;
        double q = (dx * dx + dz * dz) / (rh * rh);
        if (q >= 1) continue;
        double h = near_rock[i].base + near_rock[i].sy * sqrt(1 - q);
        if (h > best) best = h;
    }
    return best;
}

static double ground_at(double x, double z) { return terrain_ground_height(x, z) + rock_at(x, z); }

// Blocks taller than a step stop the crewman: push him back out to their edge.
static int bump_rocks(Walker *w)
{
    int hit = 0;
    for (int i = 0; i < nnear; i++) {
        double dx = w->px - near_rock[i].x, dz = w->pz - near_rock[i].z, d = hypot(dx, dz);
        double reach = near_rock[i].rh + BODY_R;
        if (d >= reach || d < 1e-6) continue;
        double top = terrain_ground_height(near_rock[i].x, near_rock[i].z) + near_rock[i].base + near_rock[i].sy;
        if (w->alt + STEP_UP >= top) continue;   // low enough to step up, or jumped over it
        double nx = dx / d, nz = dz / d;
        w->px = near_rock[i].x + nx * reach;
        w->pz = near_rock[i].z + nz * reach;
        double vn = w->vx * nx + w->vz * nz;
        if (vn < 0) { w->vx -= vn * nx; w->vz -= vn * nz; }
        hit = 1;
    }
    return hit;
}

static void keep_clear(Walker *w)
{
    // The descent stage.
    double cx, cz;
    terrain_site_to_plane(lm_pos, &cx, &cz);
    double dx = w->px - cx, dz = w->pz - cz, d = hypot(dx, dz);
    if (d < LM_KEEPOUT && d > 1e-6) {
        w->px = cx + dx / d * LM_KEEPOUT;
        w->pz = cz + dz / d * LM_KEEPOUT;
    }
    // The other crewman.
    Walker *o = &cw[w == &cw[0] ? 1 : 0];
    if (o->phase == EVA_SURFACE) {
        dx = w->px - o->px;
        dz = w->pz - o->pz;
        d = hypot(dx, dz);
        if (d < CREW_GAP && d > 1e-6) {
            w->px = o->px + dx / d * CREW_GAP;
            w->pz = o->pz + dz / d * CREW_GAP;
        }
    }
}

// --- Boot prints ----------------------------------------------------------------------

static void print_step(Walker *w, double land_speed)
{
    if (rock_at(w->px, w->pz) > 0.03) return;   // on a block: no print in the dust
    double y = w->yaw, rx = cos(y), rz = sin(y), fx = sin(y), fz = -cos(y);
    float depth = (float)fmin(1.0, 0.55 + land_speed * 0.4);
    // A hop lands on one foot then the other; a jump on both.
    int both = land_speed > 1.3 || w->speed < 0.2f;
    static int alt_side[2];
    int k = (int)(w - cw);
    for (int s = 0; s < 2; s++) {
        if (!both && s != alt_side[k]) continue;
        double side = s ? 0.12 : -0.12, ahead = both ? 0.0 : 0.15;
        footprints_add(w->px + rx * side + fx * ahead, w->pz + rz * side + fz * ahead, y, depth);
    }
    alt_side[k] ^= 1;
}

// --- Walking ---------------------------------------------------------------------------

// Move one crewman along his velocity, with gravity, boulders and the gait clock.
// want_x, want_z: direction he is trying to go (unit or zero), speed: how fast.
static void move(Walker *w, float dt, double want_x, double want_z, double speed, int jump)
{
    double grip = w->grounded ? 5.0 : 0.4;   // little to push against while off the ground
    double k = fmin(1.0, grip * dt);
    w->vx += (want_x * speed - w->vx) * k;
    w->vz += (want_z * speed - w->vz) * k;
    double wl = hypot(want_x, want_z);
    if (w->grounded) {
        if (jump) { w->vy = JUMP_V; w->grounded = 0; }
        else if (wl > 0.1) { w->vy = HOP_V * (speed / WALK_V); w->grounded = 0; }
    }
    double x0 = w->px, z0 = w->pz;
    w->px += w->vx * dt;
    w->pz += w->vz * dt;
    refresh_rocks(w->px, w->pz);
    bump_rocks(w);
    keep_clear(w);
    double moved = hypot(w->px - x0, w->pz - z0);
    w->speed = (float)(moved / fmax(dt, 1e-4));

    w->vy -= MOON_G * dt;
    w->alt += w->vy * dt;
    double ground = ground_at(w->px, w->pz);
    if (w->alt <= ground) {
        if (!w->grounded) {
            double v = -w->vy;
            if (w == mine()) {
                if (v > 1.0) sound_touchdown((float)(v * 0.25));   // boots, felt through the suit
                else sound_step((float)v);
            }
            w->land_k = (float)fmin(1.0, v / 1.6);
            print_step(w, v);
        }
        w->alt = ground;
        w->vy = 0;
        w->grounded = 1;
        w->air = 0;
    } else if (w->alt - ground > 0.03) {
        w->grounded = 0;
        w->air += dt;
    }
    w->land_k = fmaxf(0, w->land_k - dt * 3.0f);
    // Gait: one stride (both feet) per ~1.6 m at a walk, longer when loping.
    double stride_len = 1.2 + 0.8 * fmin(1.0, w->speed / LOPE_V);
    w->stride = fmodf(w->stride + (float)(moved / stride_len * 2 * M_PI), (float)(2 * M_PI));
    // The body turns to the way he goes (backing up keeps the face forward).
    double target = w->yaw;
    if (w->speed > 0.35) target = atan2(w->vx, -w->vz);
    double dy = remainder(target - w->yaw, 2 * M_PI);
    w->yaw += dy * fmin(1.0, 6.0 * dt);
}

static void plant_flag(double x, double z, double yaw, Walker *by)
{
    flag_x = x;
    flag_z = z;
    flag_yaw = yaw;
    flag_on = 1;
    if (by == mine()) {
        crew_say("The flag is planted");
        sound_touchdown(0.4f);
    }
}

static void start_surface(Walker *w, int second)
{
    // Stand a pace in front of the forward footpad, facing the LM (the second man a step to the side).
    terrain_site_to_plane(body_point(-2.75, 0, 5.3), &foot_x, &foot_z);
    w->px = foot_x;
    w->pz = foot_z;
    V3 east, north;
    frame_at(w->px, w->pz, 0, &east, &north, NULL);
    V3 f = m3_apply(lm_R, v3(0, 0, -1));
    w->yaw = face_yaw = atan2(v3_dot(f, east), v3_dot(f, north));
    if (second) {
        // A step to his right, out of the way of the ladder.
        w->px += cos(w->yaw) * 1.1;
        w->pz += sin(w->yaw) * 1.1;
    }
    refresh_rocks(w->px, w->pz);
    w->alt = ground_at(w->px, w->pz);
    w->vx = w->vz = w->vy = 0;
    w->grounded = 1;
    w->stride = 0;
    if (w == mine()) {
        view_yaw = w->yaw;
        w->pitch = 0.05;
    }
}

static void local_walk(Walker *w, float dt)
{
    // Looking: right-drag with the mouse, or the arrow keys.
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        Vector2 d = GetMouseDelta();
        view_yaw += d.x * 0.0032;
        w->pitch -= d.y * 0.0032;
    }
    view_yaw += ((IsKeyDown(KEY_RIGHT) ? 1 : 0) - (IsKeyDown(KEY_LEFT) ? 1 : 0)) * 1.6 * dt;
    w->pitch += ((IsKeyDown(KEY_UP) ? 1 : 0) - (IsKeyDown(KEY_DOWN) ? 1 : 0)) * 1.0 * dt;
    if (w->pitch > 1.35) w->pitch = 1.35;
    if (w->pitch < -1.35) w->pitch = -1.35;
    if (vr_mode) w->pitch = 0;   // the base view stays level; the head looks up and down

    // Plane axes: +x east, +z south. On the headset "forward" is where the head points.
    double f = (IsKeyDown(KEY_W) ? 1 : 0) - (IsKeyDown(KEY_S) ? 1 : 0);
    {   // Debugging: AGC_EVA_WALK=SECONDS walks at the nearest big block (r >= 0.8 m) for that long.
        static double rx = 1e30, rz, t_walk, rr;
        static int reported;
        const char *ew = getenv("AGC_EVA_WALK");
        if (ew && t_walk < atof(ew)) {
            if (rx > 1e29) {
                const TerrainRock *r;
                int n = terrain_rocks(&r);
                double best = 1e30;
                for (int i = 0; i < n; i++) {
                    if (r[i].r < 0.8) continue;
                    double d = hypot(r[i].x - w->px, r[i].z - w->pz);
                    if (d < best) { best = d; rx = r[i].x; rz = r[i].z; rr = 0.5 * (r[i].sx + r[i].sz); }
                }
                fprintf(stderr, "EVAWALK block at %.1f m, radius %.2f, top %.2f m\n", best, rr, rock_at(rx, rz));
            }
            view_yaw = atan2(rx - w->px, -(rz - w->pz));
            f = 1;
            t_walk += dt;
            if ((int)(t_walk * 2) != (int)((t_walk - dt) * 2))
                fprintf(stderr, "EVAWALK t=%.1f  centre %.2f m (edge %.2f)  soles %.2f above ground  speed %.2f\n", t_walk,
                        hypot(rx - w->px, rz - w->pz), rr, w->alt - terrain_ground_height(w->px, w->pz), w->speed);
        } else if (ew && !reported) {
            reported = 1;
            fprintf(stderr, "EVAWALK done: %.2f m from the block's centre (edge %.2f + body %.2f)\n", hypot(rx - w->px, rz - w->pz), rr, BODY_R);
        }
    }
    double r = (IsKeyDown(KEY_D) ? 1 : 0) - (IsKeyDown(KEY_A) ? 1 : 0);
    if (w->act == ACT_FLAG) f = r = 0;   // busy with the flag
    double wy = view_yaw + (vr_mode ? head_yaw : 0);
    double wx = sin(wy) * f + cos(wy) * r, wz = -cos(wy) * f + sin(wy) * r;
    double wl = hypot(wx, wz);
    if (wl > 1) { wx /= wl; wz /= wl; }
    double speed = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT) ? LOPE_V : WALK_V;
    move(w, dt, wx, wz, speed, IsKeyPressed(KEY_SPACE) && w->grounded && w->act != ACT_FLAG);
    // Standing still, the body faces where the view does (walking, the way he goes: see move()).
    if (w->speed < 0.35) {
        double dy = remainder(wy - w->yaw, 2 * M_PI);
        w->yaw += dy * fmin(1.0, 5.0 * dt);
    } else if (fabs(remainder(atan2(w->vx, -w->vz) - wy, 2 * M_PI)) > 1.9) {
        w->yaw = wy;   // walking backward: the body keeps facing the view
    }

    if (IsKeyPressed(KEY_F) && w->act == ACT_NONE && !w->carry) {
        if (flag_on) crew_say("The flag is already up");
        else if (!w->grounded) crew_say("Both feet on the ground first");
        else { w->act = ACT_FLAG; w->act_t = 0; w->yaw = wy; }
    }
}

// --- The computer-run LM Pilot (playing alone) ------------------------------------------

static void ai_walk(Walker *w, float dt)
{
    Walker *cdr = &cw[0];
    w->ai_t += dt;
    double tx = w->px, tz = w->pz, near = 1.4;
    switch (w->ai_mode) {
    case AI_FOLLOW:
        // Keep a few metres ahead and to the right of the Commander, so he is in view;
        // the spot moves only once the Commander has.
        if (hypot(cdr->px - w->anchor_x, cdr->pz - w->anchor_z) > 5.0 || w->ai_tx == 0) {
            double y = cdr->yaw;
            w->anchor_x = cdr->px;
            w->anchor_z = cdr->pz;
            w->ai_tx = cdr->px + sin(y) * 4.0 + cos(y) * 3.0;
            w->ai_tz = cdr->pz - cos(y) * 4.0 + sin(y) * 3.0;
            // Not under the LM: out to clear ground round it.
            double cx, cz;
            terrain_site_to_plane(lm_pos, &cx, &cz);
            double dx = w->ai_tx - cx, dz = w->ai_tz - cz, d = hypot(dx, dz);
            if (d < 6.0) {
                if (d < 1e-3) { dx = cos(y); dz = sin(y); d = 1; }
                w->ai_tx = cx + dx / d * 6.0;
                w->ai_tz = cz + dz / d * 6.0;
            }
        }
        tx = w->ai_tx;
        tz = w->ai_tz;
        if (flag_on && w->ai_t > 2 && w->act == ACT_NONE && w->ai_mode == AI_FOLLOW && !w->want_in) {
            static double saluted_x = 1e30, saluted_z = 1e30;
            if (hypot(flag_x - saluted_x, flag_z - saluted_z) > 0.5) {
                saluted_x = flag_x;
                saluted_z = flag_z;
                w->ai_mode = AI_FLAG;
                w->ai_t = 0;
            }
        }
        break;
    case AI_FLAG:
        // Beside the flag, a step back, then salute it.
        tx = flag_x - cos(flag_yaw) * 1.4 - sin(flag_yaw) * 0.6;
        tz = flag_z - sin(flag_yaw) * 1.4 + cos(flag_yaw) * 0.6;
        near = 0.5;
        if (w->ai_t > 25) { w->ai_mode = AI_FOLLOW; w->ai_t = 0; }   // could not get there
        break;
    case AI_GO_IN:
        tx = foot_x + cos(face_yaw) * 1.1;   // where he stepped off the ladder
        tz = foot_z + sin(face_yaw) * 1.1;
        near = 0.8;
        break;
    }
    double dx = tx - w->px, dz = tz - w->pz, d = hypot(dx, dz);
    double wx = 0, wz = 0, speed = WALK_V;
    if (w->act == ACT_NONE && d > near) {
        wx = dx / d;
        wz = dz / d;
        if (d > 7 || w->ai_mode == AI_GO_IN) speed = LOPE_V;
        // Round a block: veer off to one side for a moment when he stops getting anywhere.
        if (w->avoid_t > 0) {
            w->avoid_t -= dt;
            double c = cos(w->avoid), s = sin(w->avoid);
            double nx = wx * c - wz * s, nz = wx * s + wz * c;
            wx = nx;
            wz = nz;
        }
    }
    double x0 = w->px, z0 = w->pz;
    move(w, dt, wx, wz, speed, 0);
    double prog = hypot(w->px - x0, w->pz - z0);
    if (wx != 0 || wz != 0) {
        if (prog < 0.15 * speed * dt && w->grounded) w->stuck_t += dt;
        else w->stuck_t = fmaxf(0, w->stuck_t - dt);
        if (w->stuck_t > 0.8f) {
            w->stuck_t = 0;
            w->avoid = (w->avoid > 0 ? -1 : 1) * 1.2;
            w->avoid_t = 1.4f;
        }
    } else if (w->speed < 0.2f && w->act == ACT_NONE) {
        // Waiting: face the Commander (or the flag).
        double fy = w->ai_mode == AI_FLAG ? atan2(flag_x - w->px, -(flag_z - w->pz)) : atan2(cdr->px - w->px, -(cdr->pz - w->pz));
        w->yaw += remainder(fy - w->yaw, 2 * M_PI) * fmin(1.0, 3.0 * dt);
        if (w->ai_mode == AI_FLAG && fabs(remainder(fy - w->yaw, 2 * M_PI)) < 0.1) {
            w->act = ACT_SALUTE;
            w->act_t = 0;
            w->ai_mode = AI_FOLLOW;
            w->ai_t = 0;
        }
    }
    if (w->ai_mode == AI_GO_IN && d <= near) {
        w->phase = EVA_INGRESS;
        w->t_phase = 0;
        w->ai_mode = AI_FOLLOW;
    }
}

// --- Per frame -----------------------------------------------------------------------------

static void step_actions(Walker *w, float dt)
{
    if (w->act == ACT_NONE) return;
    w->act_t += dt;
    if (w->act == ACT_FLAG) {
        w->carry = 1;
        if (w->act_t >= FLAG_ACT_T) {
            w->carry = 0;
            w->act = ACT_NONE;
            if (!flag_on) plant_flag(w->px + sin(w->yaw) * 0.75, w->pz - cos(w->yaw) * 0.75, w->yaw, w);
        }
    } else if (w->act == ACT_SALUTE && w->act_t >= SALUTE_T) {
        w->act = ACT_NONE;
    }
}

static void step_walker(int i, float dt)
{
    Walker *w = &cw[i];
    int local = i == me;
    if (!local && !w->ai) return;   // the network moves him
    w->t_phase += dt;
    {   // Debugging: AGC_EVA_IN=SECONDS presses X that long after reaching the surface; AGC_EVA_LOG prints the phases.
        static int last[2] = { -1, -1 }, pressed;
        const char *ein = getenv("AGC_EVA_IN");
        if (local && ein && !pressed && w->phase == EVA_SURFACE && w->t_phase > atof(ein)) { pressed = 1; eva_key(); }
        if (getenv("AGC_EVA_LOG") && w->phase != last[i]) {
            static const char *nm[] = { "INSIDE", "HATCH_OPENING", "EGRESS", "SURFACE", "INGRESS", "HATCH_CLOSING" };
            fprintf(stderr, "EVA %s %s  t=%.1f  hatch %.2f\n", i ? "LMP" : "CDR", nm[w->phase], GetTime(), eva_hatch());
            last[i] = w->phase;
        }
    }
    switch (w->phase) {
    case EVA_HATCH_OPENING:
        if (w->t_phase >= T_OPEN + T_CRAWL) { w->phase = EVA_EGRESS; w->t_phase = 0; start_surface(w, i == 1); }
        break;
    case EVA_EGRESS:
        if (w->t_phase >= T_LADDER) {
            w->phase = EVA_SURFACE;
            w->t_phase = 0;
            if (local) crew_say(i == 0 ? "On the surface. That's one small step" : "On the surface. Magnificent desolation");
        }
        break;
    case EVA_SURFACE:
        if (local) local_walk(w, dt);
        else ai_walk(w, dt);
        step_actions(w, dt);
        // Waiting at the ladder for the other crewman to go in first.
        if (w->want_in && w->phase == EVA_SURFACE) {
            Walker *o = &cw[i ? 0 : 1];
            if (o->phase == EVA_INSIDE || o->phase == EVA_HATCH_CLOSING) {
                w->want_in = 0;
                if (hypot(w->px - foot_x, w->pz - foot_z) <= LADDER_REACH + 1.0) {
                    w->phase = EVA_INGRESS;
                    w->t_phase = 0;
                    if (local) crew_say("Up the ladder");
                }
            }
        }
        break;
    case EVA_INGRESS:
        if (w->t_phase >= T_LADDER) { w->phase = EVA_HATCH_CLOSING; w->t_phase = 0; }
        break;
    case EVA_HATCH_CLOSING:
        if (w->t_phase >= T_OPEN + T_CRAWL) {
            w->phase = EVA_INSIDE;
            w->t_phase = 0;
            if (local && eva_all_inside()) crew_say("Hatch closed and latched");
        }
        break;
    }
}

void eva_update(float dt, V3 pos, M3 R)
{
    lm_pos = pos;
    lm_R = R;
    if (lm.land_result == LAND_NONE && (!eva_all_inside() || flag_on || footprints_count())) eva_reset();   // a new flight began
    // Playing alone, the LM Pilot is the computer's.
    cw[1].ai = solo() && me == 0;
    cw[0].ai = 0;
    if (cw[1].ai) {
        Walker *l = &cw[1], *c = &cw[0];
        if (c->phase == EVA_SURFACE && l->phase == EVA_INSIDE && !lmp_followed && c->t_phase > LMP_DELAY && !c->want_in) {
            l->phase = EVA_HATCH_OPENING;
            l->t_phase = T_OPEN - 1.6f;   // the hatch is open already: only the crawl out
            l->ai_mode = AI_FOLLOW;
            l->ai_t = 0;
            l->ai_tx = 0;
            lmp_followed = 1;
            crew_say("LM Pilot: \"Coming out\"");
        }
        if (c->phase == EVA_INSIDE) lmp_followed = 0;
    }
    for (int i = 0; i < 2; i++) step_walker(i, dt);
    {   // Debugging: AGC_EVA_FLAG=EAST,SOUTH plants the flag that far from the LM (m) once it is down.
        const char *ef = getenv("AGC_EVA_FLAG");
        double fx, fz;
        if (ef && !flag_on && lm.touchdown && sscanf(ef, "%lf,%lf", &fx, &fz) == 2) {
            double cx, cz;
            terrain_site_to_plane(lm.staged ? pos : lm_pos, &cx, &cz);
            plant_flag(cx + fx, cz + fz, atan2(fx, -fz) + M_PI / 2, NULL);
        }
    }
}

void eva_key(void)
{
    Walker *w = mine();
    if (!w) return;
    Walker *o = &cw[me ? 0 : 1];
    switch (w->phase) {
    case EVA_INSIDE:
        if (!eva_available()) {
            crew_say("The hatch stays shut until we are down in one piece with the engine off");
            return;
        }
        if (me == 1 && o->phase != EVA_EGRESS && o->phase != EVA_SURFACE) {
            crew_say("The Commander goes out first: the hatch opens across the LM Pilot's side");
            return;
        }
        w->phase = EVA_HATCH_OPENING;
        w->t_phase = eva_hatch() > 0.99f ? T_OPEN - 1.6f : 0;   // already open: just kneel and crawl
        crew_say(w->t_phase > 0 ? "Out through the hatch" : "Cabin depressurised. Hatch coming open");
        return;
    case EVA_SURFACE: {
        double d = hypot(w->px - foot_x, w->pz - foot_z);
        if (d > LADDER_REACH) {
            char b[96];
            snprintf(b, sizeof b, "Walk back to the foot of the ladder to climb in (%.0f m away)", d);
            crew_say(b);
            return;
        }
        if (w->want_in) return;
        if (me == 0 && o->phase != EVA_INSIDE && o->phase != EVA_HATCH_CLOSING) {
            // The LM Pilot goes in first (his side of the hatch).
            w->want_in = 1;
            if (o->ai) {
                if (o->phase == EVA_SURFACE) { o->ai_mode = AI_GO_IN; o->ai_t = 0; }
                else if (o->phase == EVA_EGRESS) { o->phase = EVA_INGRESS; o->t_phase = T_LADDER - o->t_phase; }
                else if (o->phase == EVA_HATCH_OPENING) { o->phase = EVA_INSIDE; o->t_phase = 0; }
                crew_say("The LM Pilot climbs in first");
            } else {
                crew_say("The LM Pilot climbs in first: waiting for him at the ladder");
            }
            return;
        }
        w->phase = EVA_INGRESS;
        w->t_phase = 0;
        crew_say("Up the ladder");
        return;
    }
    default:
        return;   // already on the way in or out
    }
}

void eva_liftoff_blast(V3 deck_site, double age)
{
    blast_age = age;
    blast_from = deck_site;
}

// Debugging (AGC_EVA=hatch|crawl|ladder|walk|flag): jump straight to a point in the sequence.
void eva_debug(const char *what, V3 pos, M3 R)
{
    lm_pos = pos;
    lm_R = R;
    if (me < 0) me = 0;
    Walker *w = &cw[me];
    start_surface(w, me == 1);
    switch (what[0]) {
    case 'h': w->phase = EVA_HATCH_OPENING; w->t_phase = T_OPEN - 0.05f; break;
    case 'c': w->phase = EVA_HATCH_OPENING; w->t_phase = T_OPEN + T_CRAWL * 0.5f; break;
    case 'l': w->phase = EVA_EGRESS; w->t_phase = T_LADDER * 0.55f; break;
    case 'f': {
        w->phase = EVA_SURFACE;
        w->px += 4.0;
        w->pz += 6.0;
        refresh_rocks(w->px, w->pz);
        w->alt = ground_at(w->px, w->pz);
        view_yaw += 0.35;
        w->yaw = view_yaw;
        plant_flag(w->px + sin(view_yaw) * 2.2 - cos(view_yaw) * 0.9, w->pz - cos(view_yaw) * 2.2 - sin(view_yaw) * 0.9, view_yaw, NULL);
        // The LM Pilot out too, and a trail of prints between the ladder and here.
        Walker *l = &cw[1];
        if (me == 0) {
            start_surface(l, 1);
            l->phase = EVA_SURFACE;
            l->px = w->px + sin(view_yaw) * 4.5 + cos(view_yaw) * 2.0;
            l->pz = w->pz - cos(view_yaw) * 4.5 + sin(view_yaw) * 2.0;
            refresh_rocks(l->px, l->pz);
            l->alt = ground_at(l->px, l->pz);
            l->yaw = view_yaw + M_PI * 0.8;
            lmp_followed = 1;
        }
        for (int k = 0; k < 16; k++) {
            double t = k / 15.0;
            double x = foot_x + (w->px - foot_x) * t, z = foot_z + (w->pz - foot_z) * t;
            double y = atan2(w->px - foot_x, -(w->pz - foot_z));
            footprints_add(x + cos(y) * (k & 1 ? 0.12 : -0.12), z + sin(y) * (k & 1 ? 0.12 : -0.12), y, 0.9f);
        }
        break;
    }
    default: w->phase = EVA_SURFACE; break;
    }
    // AGC_EVA_LOOK=yaw,pitch (degrees): turn the view (yaw added to the facing).
    const char *lk = getenv("AGC_EVA_LOOK");
    double ly = 0, lp = 0;
    if (lk && sscanf(lk, "%lf,%lf", &ly, &lp) == 2) {
        view_yaw += ly * M_PI / 180;
        w->yaw = view_yaw;
        w->pitch = lp * M_PI / 180;
    }
}

ViewCam eva_camera(void)
{
    ViewCam c = { 0 };
    c.fov_y = 70 * PI_D / 180;
    Walker *w = mine();
    if (!w) { c.pos = lm_pos; c.fwd = v3(1, 0, 0); c.upv = v3(0, 1, 0); return c; }
    V3 east, north, up;
    V3 stand = frame_at(w->px, w->pz, w->alt + EYE_H, &east, &north, &up);
    double vy = view_yaw;
    V3 look = v3_add(v3_scale(v3_add(v3_scale(east, sin(vy)), v3_scale(north, cos(vy))), cos(w->pitch)), v3_scale(up, sin(w->pitch)));
    if (w->phase == EVA_SURFACE) {
        // The eye sits a little forward of the body's axis, inside the helmet.
        V3 bf = v3_add(v3_scale(east, sin(w->yaw)), v3_scale(north, cos(w->yaw)));
        c.pos = v3_add(stand, v3_scale(bf, 0.05));
        c.fwd = v3_norm(look);
        c.upv = up;
    } else {
        // Backing out: on the porch, down the ladder facing the LM, then onto the surface.
        float u = w->t_phase / T_LADDER;
        if (w->phase == EVA_INGRESS) u = 1 - u;
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

// --- Headset hands -----------------------------------------------------------------------

static V3 ladder0(void) { return body_point(-1.95, 0, 4.90); }
static V3 ladder1(void) { return body_point(0.45, 0, 3.50); }

static double seg_dist(V3 p, V3 a, V3 b)
{
    V3 ab = v3_sub(b, a);
    double t = v3_dot(v3_sub(p, a), ab) / fmax(1e-9, v3_dot(ab, ab));
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    return v3_len(v3_sub(p, v3_add(a, v3_scale(ab, t))));
}

int eva_vr_hands(const V3 hand_site[2], const int grip[2])
{
    static int was[2];
    int ev = 0;
    Walker *w = mine();
    vr_hands_on = w && w->phase == EVA_SURFACE;
    for (int h = 0; h < 2; h++) vr_hand[h] = hand_site[h];
    if (vr_hands_on) {
        // The flag: squeeze the right grip to take it, let go to plant it below the hand.
        if (grip[1] && !was[1] && !flag_on && w->act == ACT_NONE) { w->carry = 1; ev |= EVA_VR_TOOK_FLAG; }
        if (!grip[1] && was[1] && w->carry && w->act == ACT_NONE) {
            w->carry = 0;
            double x, z;
            terrain_site_to_plane(hand_site[1], &x, &z);
            if (hypot(x - w->px, z - w->pz) > 1.8) {   // too far out to reach the ground there
                x = w->px + (x - w->px) * 1.8 / hypot(x - w->px, z - w->pz);
                z = w->pz + (z - w->pz) * 1.8 / hypot(x - w->px, z - w->pz);
            }
            plant_flag(x, z, w->yaw + head_yaw, w);
            ev |= EVA_VR_PLANTED;
        }
        // The ladder: a hand on the rails at the foot of it climbs back in.
        for (int h = 0; h < 2; h++)
            if (grip[h] && !was[h] && !(h == 1 && w->carry) && seg_dist(hand_site[h], ladder0(), ladder1()) < 0.45 &&
                hypot(w->px - foot_x, w->pz - foot_z) <= LADDER_REACH) {
                eva_key();
                ev |= EVA_VR_CLIMB;
            }
    } else if (w && w->carry && w->act == ACT_NONE) {
        w->carry = 0;
    }
    was[0] = grip[0];
    was[1] = grip[1];
    return ev;
}

int eva_vr_cabin_hands(const V3 hand_body[2], const int grip[2])
{
    static int was[2];
    int ev = 0;
    V3 handle = v3(1.56, -0.28, 0.94);   // the hatch's latch handle (cockpit.c build_dynamic)
    for (int h = 0; h < 2; h++) {
        if (grip[h] && !was[h] && eva_phase() == EVA_INSIDE && eva_available() && v3_len(v3_sub(hand_body[h], handle)) < 0.28) {
            eva_key();
            ev |= EVA_VR_HATCH;
        }
        was[h] = grip[h];
    }
    return ev;
}

// --- Drawing -----------------------------------------------------------------------------

// The flag's axes (+X up the pole, +Z along the cloth) and the foot of its pole.
static M3 flag_axes(double x, double z, double yaw, V3 *base)
{
    V3 east, north, up;
    *base = frame_at(x, z, terrain_ground_height(x, z), &east, &north, &up);
    V3 zf = v3_add(v3_scale(east, cos(yaw)), v3_scale(north, -sin(yaw)));
    V3 yf = v3_cross(zf, up);
    M3 R = { { { up.x, yf.x, zf.x }, { up.y, yf.y, zf.y }, { up.z, yf.z, zf.z } } };
    // Lift-off: near the LM the exhaust knocks it flat (Apollo 11's went over); further out it only shakes.
    if (blast_age >= 0) {
        V3 away = v3_sub(*base, blast_from);
        away = v3_sub(away, v3_scale(up, v3_dot(away, up)));
        double d = v3_len(away);
        double t = blast_age - 0.25 - d / 60.0;   // the gas front reaches it
        if (d > 1e-3 && t > 0) {
            away = v3_scale(away, 1 / d);
            V3 axis = v3_norm(v3_cross(up, away));
            double ang;
            if (d < 16) {
                double k = 6.0 * (1.0 - d / 20.0);   // how hard it is shoved over, rad/s^2 (then gravity)
                ang = fmin(1.38, 0.5 * (k + 1.2 * fmin(t, 1.5)) * t * t);
            } else {
                ang = 0.10 * exp(-t * 0.8) * sin(t * 13.0) * fmax(0.0, 1.0 - (d - 16) / 30.0);
            }
            M3 Rb = q4_to_m3(q4_axis_angle(axis, ang));
            R = m3_mul(Rb, R);
        }
    }
    return R;
}

static void flag_in_hands(const Walker *w, const AstroPose *p, M3 *R, V3 *base)
{
    // Held upright, the gloves on the pole; while planting it goes down into the ground.
    V3 dir;
    V3 hand = astro_hand(p, 1, &dir);
    double x, z;
    terrain_site_to_plane(hand, &x, &z);
    V3 east, north, up;
    frame_at(x, z, 0, &east, &north, &up);
    double yaw = w->yaw;
    V3 zf = v3_add(v3_scale(east, cos(yaw)), v3_scale(north, -sin(yaw)));
    V3 yf = v3_cross(zf, up);
    M3 r = { { { up.x, yf.x, zf.x }, { up.y, yf.y, zf.y }, { up.z, yf.z, zf.z } } };
    *R = r;
    double down = w->act == ACT_FLAG ? 0.5 * smooth(w->act_t / FLAG_ACT_T) : 0;
    *base = v3_sub(hand, v3_scale(up, 1.05 + down));
}

static void flag_in_vr_hand(const Walker *w, M3 *R, V3 *base)
{
    double x, z;
    terrain_site_to_plane(vr_hand[1], &x, &z);
    V3 east, north, up;
    frame_at(x, z, 0, &east, &north, &up);
    double yaw = w->yaw + head_yaw;
    V3 zf = v3_add(v3_scale(east, cos(yaw)), v3_scale(north, -sin(yaw)));
    V3 yf = v3_cross(zf, up);
    M3 r = { { { up.x, yf.x, zf.x }, { up.y, yf.y, zf.y }, { up.z, yf.z, zf.z } } };
    *R = r;
    *base = v3_sub(vr_hand[1], v3_scale(up, 1.15));
}

// The figure for a crewman (0 if nothing to draw).
static int pose_of(int i, AstroPose *p, int *first_person)
{
    const Walker *w = &cw[i];
    memset(p, 0, sizeof *p);
    p->cdr = i == 0;
    *first_person = 0;
    if (w->phase == EVA_SURFACE) {
        if (i == me) *first_person = 1;
        p->pos = terrain_plane_to_site(w->px, w->pz, w->alt);
        p->yaw = w->yaw;
        float s = fminf(1.0f, w->speed / (float)LOPE_V);
        float ph = w->stride;
        p->lean = 0.04f + 0.20f * s;
        float amp = 0.06f + 0.26f * s;
        p->foot_fwd[0] = amp * sinf(ph);
        p->foot_fwd[1] = -amp * sinf(ph);
        if (!w->grounded) {
            // Off the ground: the trailing boot comes up behind, more so on a jump.
            float tuck = fminf(0.16f, 0.05f + w->air * 0.15f) * (w->speed < 0.3f ? 1.4f : 1.0f);
            p->foot_lift[0] = tuck * fmaxf(0.2f, -sinf(ph));
            p->foot_lift[1] = tuck * fmaxf(0.2f, sinf(ph));
        }
        p->crouch = 0.10f * w->land_k;
        for (int k = 0; k < 2; k++) {
            float sw = sinf(ph + (k ? 0.0f : (float)M_PI));
            p->sh_fwd[k] = 0.12f + 0.22f * s * sw;
            p->sh_out[k] = 0.20f;
            p->elbow[k] = 0.55f + 0.25f * s;
        }
        if (w->act == ACT_FLAG || (w->carry && !(vr_mode && i == me))) {
            // Both hands on the pole in front, pushing it down.
            float t = w->act == ACT_FLAG ? smooth(w->act_t / FLAG_ACT_T) : 0;
            for (int k = 0; k < 2; k++) {
                p->sh_fwd[k] = 0.85f - 0.25f * t;
                p->sh_out[k] = k ? 0.05f : 0.30f;
                p->elbow[k] = 0.65f - 0.30f * t;
            }
            p->lean += 0.12f * t;
        } else if (w->act == ACT_SALUTE) {
            float t = smooth(w->act_t / 0.5f) * smooth((SALUTE_T - w->act_t) / 0.5f);
            p->sh_fwd[1] = 0.12f + 1.55f * t;
            p->sh_out[1] = 0.20f + 0.45f * t;
            p->elbow[1] = 0.55f + 1.55f * t;
        }
        return 1;
    }
    if (w->phase == EVA_EGRESS || w->phase == EVA_INGRESS) {
        if (i == me) return 0;   // the camera rides this path itself
        float u = w->t_phase / T_LADDER;
        if (w->phase == EVA_INGRESS) u = 1 - u;
        float s = smooth(u);
        V3 a = body_point(0.92, 0, 2.30), b = body_point(0.80, 0, 3.25), d = body_point(-1.25, 0, 4.60);
        V3 east, north, up;
        double fx, fz;
        terrain_site_to_plane(lm_pos, &fx, &fz);
        frame_at(fx, fz, 0, &east, &north, &up);
        V3 stand_eye = terrain_plane_to_site(w->px, w->pz, terrain_ground_height(w->px, w->pz) + EYE_H);
        V3 eye = s < 0.22f ? v3_add(a, v3_scale(v3_sub(b, a), s / 0.22f)) :
                 s < 0.80f ? v3_add(b, v3_scale(v3_sub(d, b), (s - 0.22f) / 0.58f)) :
                             v3_add(d, v3_scale(v3_sub(stand_eye, d), (s - 0.80f) / 0.20f));
        p->pos = v3_sub(eye, v3_scale(up, EYE_H - 0.05));
        V3 face = m3_apply(lm_R, v3(0, 0, -1));   // facing the LM: his back to the open ground
        p->yaw = atan2(v3_dot(face, east), v3_dot(face, north));
        if (s < 0.22f) {
            p->kneel = 1.0f - s / 0.22f;   // backing out on hands and knees
            p->pos = v3_add(p->pos, v3_scale(up, 0.45f * p->kneel));
        }
        float climb = s * 22.0f;
        int on_ladder = s >= 0.22f && s < 0.86f;
        p->lean = on_ladder ? 0.30f : 0.10f;
        for (int k = 0; k < 2; k++) {
            float c = sinf(climb + (k ? (float)M_PI : 0));
            p->foot_fwd[k] = on_ladder ? 0.18f : 0;
            p->foot_lift[k] = on_ladder ? 0.12f * fmaxf(0, c) : 0;
            p->sh_fwd[k] = on_ladder ? 1.55f + 0.25f * c : 0.9f * p->kneel + 0.15f;
            p->sh_out[k] = 0.25f;
            p->elbow[k] = on_ladder ? 0.9f : 0.4f;
        }
        return 1;
    }
    return 0;
}

void eva_draw(const ViewCam *cam)
{
    footprints_draw(cam->pos);
    for (int i = 0; i < 2; i++) {
        AstroPose p;
        int fp;
        if (!pose_of(i, &p, &fp)) continue;
        // The wearer's own body: the parts round the head are cut away. On the
        // headset the hands are the controllers: his arms hang back out of view.
        if (fp) render_set_lm_extras(0.34f, 1.0f, NULL, NULL, 0);
        if (fp && vr_mode) p.no_arms = 1;
        astro_draw(&p, cam->pos, fp);
        if (fp) render_set_lm_extras(0.0f, 1.0f, NULL, NULL, 0);
        // A flag carried in the hands.
        if (cw[i].carry && !(vr_mode && i == me)) {
            M3 R;
            V3 base;
            flag_in_hands(&cw[i], &p, &R, &base);
            lmmodel_draw_flag(R, v3_sub(base, cam->pos));
        }
    }
    Walker *w = mine();
    if (w && vr_mode && w->carry && vr_hands_on) {
        M3 R;
        V3 base;
        flag_in_vr_hand(w, &R, &base);
        lmmodel_draw_flag(R, v3_sub(base, cam->pos));
    }
    if (flag_on) {
        V3 base;
        M3 R = flag_axes(flag_x, flag_z, flag_yaw, &base);
        lmmodel_draw_flag(R, v3_sub(base, cam->pos));
    }
}

int eva_shadow_focus(V3 *centre, float *radius)
{
    V3 pts[4];
    int n = 0;
    for (int i = 0; i < 2; i++) {
        AstroPose p;
        int fp;
        if (pose_of(i, &p, &fp)) pts[n++] = p.pos;
    }
    if (flag_on) pts[n++] = terrain_plane_to_site(flag_x, flag_z, terrain_ground_height(flag_x, flag_z));
    if (!n) return 0;
    // Take in everyone if they are close together; otherwise the local crewman and whoever is near him.
    V3 c = pts[0];
    if (me >= 0 && cw[me].phase == EVA_SURFACE) c = terrain_plane_to_site(cw[me].px, cw[me].pz, cw[me].alt);
    V3 lo = c, hi = c;
    for (int k = 0; k < n; k++) {
        if (v3_len(v3_sub(pts[k], c)) > 24) continue;
        lo = v3(fmin(lo.x, pts[k].x), fmin(lo.y, pts[k].y), fmin(lo.z, pts[k].z));
        hi = v3(fmax(hi.x, pts[k].x), fmax(hi.y, pts[k].y), fmax(hi.z, pts[k].z));
    }
    *centre = v3_scale(v3_add(lo, hi), 0.5);
    *radius = (float)fmax(2.6, v3_len(v3_sub(hi, lo)) * 0.5 + 2.4);
    return 1;
}

void eva_draw_depth(V3 centre)
{
    for (int i = 0; i < 2; i++) {
        AstroPose p;
        int fp;
        if (!pose_of(i, &p, &fp)) continue;
        if (fp && vr_mode) p.sh_fwd[0] = p.sh_fwd[1] = -0.2f;
        astro_draw_depth(&p, centre);
        if (cw[i].carry && !(vr_mode && i == me)) {
            M3 R;
            V3 base;
            flag_in_hands(&cw[i], &p, &R, &base);
            lmmodel_draw_flag_depth(R, v3_sub(base, centre));
        }
    }
    Walker *w = mine();
    if (w && vr_mode && w->carry && vr_hands_on) {
        M3 R;
        V3 base;
        flag_in_vr_hand(w, &R, &base);
        lmmodel_draw_flag_depth(R, v3_sub(base, centre));
    }
    if (flag_on) {
        V3 base;
        M3 R = flag_axes(flag_x, flag_z, flag_yaw, &base);
        lmmodel_draw_flag_depth(R, v3_sub(base, centre));
    }
}

void eva_overlay(int w, int h)
{
    float s = h / 1080.0f;
    if (s < 0.7f) s = 0.7f;
    Walker *m = mine();
    if (!m) return;
    // Through the hatch: the cabin view fades out, the outside view fades in.
    float fade = 0;
    if (at_hatch(m)) fade = smooth((hatch_clock(m) - (T_OPEN + T_CRAWL - 0.7f)) / 0.7f);
    if (m->phase == EVA_EGRESS) fade = 1 - smooth(m->t_phase / 0.8f);
    if (m->phase == EVA_INGRESS) fade = smooth((m->t_phase - (T_LADDER - 0.8f)) / 0.8f);
    if (fade > 0.003f) DrawRectangle(0, 0, w, h, (Color){ 0, 0, 0, (unsigned char)(255 * fade) });

    const char *msg = NULL;
    char b[240];
    if (m->phase == EVA_INSIDE && eva_available()) {
#ifdef AGC_VR
        msg = "X, or grab the hatch handle   open the hatch and go outside";
#else
        msg = "X   open the hatch and go outside";
#endif
    } else if (m->phase == EVA_SURFACE) {
        double d = hypot(m->px - foot_x, m->pz - foot_z);
        const char *in = m->want_in ? "waiting for the LM Pilot to climb in" : d <= LADDER_REACH ? "X climb back in" : "X at the ladder: climb back in";
#ifdef AGC_VR
        snprintf(b, sizeof b, "MOONWALK    left stick walk    left grip lope    A jump    right stick turn    %s    %s",
                 flag_on ? "flag planted" : m->carry ? "let go of the grip to plant the flag" : "right grip: take the flag", in);
#else
        snprintf(b, sizeof b, "MOONWALK    W A S D walk    Shift lope    Space jump    right-drag or arrows look    %s    %s",
                 flag_on ? "flag planted" : "F plant the flag", in);
#endif
        msg = b;
    }
    if (!msg) return;
    Vector2 mm = MeasureTextEx(font_ui, msg, 20 * s, 0.4f);
    float x = (w - mm.x) / 2, y = h - 74 * s;
    DrawRectangleRounded((Rectangle){ x - 12, y - 6, mm.x + 24, mm.y + 12 }, 0.3f, 6, (Color){ 8, 12, 16, 200 });
    DrawTextEx(font_ui, msg, (Vector2){ x, y }, 20 * s, 0.4f, (Color){ 235, 238, 240, 255 });
}

// --- Network -----------------------------------------------------------------------------

static void to_net(const Walker *w, EvaNetWalker *n)
{
    n->phase = w->phase;
    n->act = w->act;
    n->grounded = w->grounded;
    n->t_phase = w->t_phase;
    n->act_t = w->act_t;
    n->yaw = (float)w->yaw;
    n->pitch = (float)w->pitch;
    n->stride = w->stride;
    n->speed = w->speed;
    n->land_k = w->land_k;
    n->vy = (float)w->vy;
    n->px = w->px;
    n->pz = w->pz;
    n->alt = w->alt;
}

static void from_net(Walker *w, const EvaNetWalker *n)
{
    // Prints for a crewman moved over the network: when his boots come down.
    int was_air = !w->grounded && w->remote_seen && w->phase == EVA_SURFACE;
    double land_v = -w->vy;
    w->phase = n->phase;
    w->act = n->act;
    w->grounded = n->grounded;
    w->t_phase = n->t_phase;
    w->act_t = n->act_t;
    w->yaw = n->yaw;
    w->pitch = n->pitch;
    w->stride = n->stride;
    w->speed = n->speed;
    w->land_k = n->land_k;
    w->vy = n->vy;
    w->px = n->px;
    w->pz = n->pz;
    w->alt = n->alt;
    w->air = w->grounded ? 0 : w->air + 0.016f;
    if (was_air && w->grounded && w->phase == EVA_SURFACE) {
        refresh_rocks(w->px, w->pz);
        print_step(w, land_v > 0 ? land_v : 0.6);
    }
    w->remote_seen = 1;
}

void eva_net_pack(EvaNet *n)
{
    memset(n, 0, sizeof *n);
    for (int i = 0; i < 2; i++) {
        to_net(&cw[i], &n->w[i]);
        n->carry[i] = cw[i].carry;
    }
    n->flag_on = flag_on;
    n->flag_x = flag_x;
    n->flag_z = flag_z;
    n->flag_yaw = (float)flag_yaw;
}

void eva_net_apply(const EvaNet *n)
{
    for (int i = 0; i < 2; i++) {
        if (i == me) continue;
        from_net(&cw[i], &n->w[i]);
        cw[i].carry = n->carry[i];
        cw[i].ai = 0;
    }
    if (n->flag_on) {
        flag_on = 1;
        flag_x = n->flag_x;
        flag_z = n->flag_z;
        flag_yaw = n->flag_yaw;
    } else if (me != 1) {
        flag_on = 0;
    }
}

void eva_net_pack_mine(EvaNet *n)
{
    memset(n, 0, sizeof *n);
    if (me >= 0) {
        to_net(&cw[me], &n->w[me]);
        n->carry[me] = cw[me].carry;
    }
    n->flag_on = flag_on;
    n->flag_x = flag_x;
    n->flag_z = flag_z;
    n->flag_yaw = (float)flag_yaw;
}

void eva_net_apply_joiner(const EvaNet *n)
{
    from_net(&cw[1], &n->w[1]);
    cw[1].carry = n->carry[1];
    cw[1].ai = 0;
    if (n->flag_on && !flag_on) {
        flag_on = 1;
        flag_x = n->flag_x;
        flag_z = n->flag_z;
        flag_yaw = n->flag_yaw;
    }
}
