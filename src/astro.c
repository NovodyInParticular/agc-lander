// The moonwalker's figure: see astro.h.
//
// Body axes as the LM's: X up, Y right, Z forward. Torso, pack and helmet
// share the pelvis frame (tilted by the lean); each leg is a thigh and a shin
// solved so the boot lands where the pose asks; arms hang from the shoulders.

#include <math.h>

#include "astro.h"
#include "lmmodel.h"
#include "terrain.h"

#define HIP_X 0.93          // pelvis centre above the soles, standing
#define THIGH 0.41
#define SHIN 0.40
#define ANKLE 0.10          // ankle joint above the sole
#define UARM 0.30
#define FARM 0.34           // elbow to the middle of the glove

static M3 rot(V3 axis, double a) { return q4_to_m3(q4_axis_angle(v3_norm(axis), a)); }
static M3 cols(V3 x, V3 y, V3 z)
{
    M3 r = { { { x.x, y.x, z.x }, { x.y, y.y, z.y }, { x.z, y.z, z.z } } };
    return r;
}

typedef struct { int part; M3 R; V3 p; } Piece;

typedef struct {
    Piece pc[16];
    int n;
    V3 hand[2], hand_dir[2];
} Rig;

static void add(Rig *r, int part, M3 R, V3 p) { r->pc[r->n++] = (Piece){ part, R, p }; }

static void solve(const AstroPose *ap, Rig *rig, int first_person)
{
    rig->n = 0;
    double x, z;
    terrain_site_to_plane(ap->pos, &x, &z);
    double h0 = terrain_ground_height(x, z);
    V3 o = terrain_plane_to_site(x, z, h0);
    V3 east = v3_norm(v3_sub(terrain_plane_to_site(x + 1, z, h0), o));
    V3 south = v3_norm(v3_sub(terrain_plane_to_site(x, z + 1, h0), o));
    V3 up = v3_norm(v3_cross(south, east)), north = v3_scale(south, -1);
    V3 fwd = v3_add(v3_scale(east, sin(ap->yaw)), v3_scale(north, cos(ap->yaw)));
    V3 right = v3_cross(fwd, up);
    M3 B = cols(up, right, fwd);

    float kneel = ap->kneel;
    double lean = ap->lean + 0.55 * kneel, crouch = ap->crouch + 0.40 * kneel;
    M3 Rt = m3_mul(rot(right, -lean), B);
    V3 tx = m3_col(Rt, 0), ty = m3_col(Rt, 1), tz = m3_col(Rt, 2);
    V3 pelvis = v3_add(ap->pos, v3_scale(up, HIP_X - crouch));

    // The wearer's own view: boots, lower legs and gloves only (the rest would
    // crowd the eye, and on the headset fight the real body's sense of itself).
    if (!first_person) {
        add(rig, AP_TORSO, Rt, pelvis);
        add(rig, AP_PLSS, Rt, pelvis);
        add(rig, AP_HELMET, Rt, v3_add(pelvis, v3_add(v3_scale(tx, 0.76), v3_scale(tz, 0.03))));
    }

    for (int s = 0; s < 2; s++) {
        double side = s ? 1 : -1;
        // Leg: hip joint, the boot's target (on the drawn ground, following its slope), two-bone IK.
        V3 hip = v3_add(pelvis, v3_add(v3_scale(tx, -0.02), v3_scale(ty, 0.10 * side)));
        double ff = ap->foot_fwd[s] - 0.45 * kneel;
        V3 foot = v3_add(ap->pos, v3_add(v3_scale(fwd, ff), v3_scale(right, side * (0.12 + ap->foot_out[s]))));
        double fx, fz;
        terrain_site_to_plane(foot, &fx, &fz);
        double lift = ap->foot_lift[s] + (terrain_ground_height(fx, fz) - h0);
        if (kneel > 0) lift += 0.06 * kneel;
        foot = v3_add(foot, v3_scale(up, ANKLE + lift));
        V3 d = v3_sub(foot, hip);
        double df = v3_dot(d, fwd), du = v3_dot(d, up), dr = v3_dot(d, right);
        double a = atan2(-dr, -du);   // spread: positive swings the leg toward -Y
        double down = sqrt(du * du + dr * dr);
        double L = sqrt(df * df + down * down);
        if (L > THIGH + SHIN - 1e-3) L = THIGH + SHIN - 1e-3;
        if (L < 0.25) L = 0.25;
        double ck = (THIGH * THIGH + SHIN * SHIN - L * L) / (2 * THIGH * SHIN);
        double knee = M_PI - acos(ck < -1 ? -1 : ck > 1 ? 1 : ck);
        double cb = (THIGH * THIGH + L * L - SHIN * SHIN) / (2 * THIGH * L);
        double swing = atan2(df, down) + acos(cb < -1 ? -1 : cb > 1 ? 1 : cb);
        M3 Rth = m3_mul(rot(right, swing), m3_mul(rot(fwd, a), B));
        V3 kn = v3_add(hip, m3_apply(Rth, v3(-THIGH, 0, 0)));
        M3 Rsh = m3_mul(rot(m3_col(Rth, 1), -knee), Rth);
        V3 an = v3_add(kn, m3_apply(Rsh, v3(-SHIN, 0, 0)));
        double toe = ap->foot_lift[s] > 0.02 ? -0.25 * fmin(1.0, ap->foot_lift[s] / 0.1) : 0;
        if (kneel > 0) toe = -1.2 * kneel;   // on the knees: the boots point back
        M3 Rb = m3_mul(rot(right, toe), m3_mul(rot(fwd, a * 0.4), B));
        if (!first_person) add(rig, AP_THIGH, Rth, hip);
        add(rig, AP_SHIN, Rsh, kn);
        add(rig, AP_BOOT, Rb, an);
        if (ap->cdr && !first_person) add(rig, AP_BAND_LEG, Rth, hip);

        // Arm.
        V3 sh = v3_add(pelvis, v3_add(v3_scale(tx, 0.48), v3_add(v3_scale(ty, 0.25 * side), v3_scale(tz, -0.02))));
        M3 Rua = m3_mul(rot(ty, ap->sh_fwd[s]), m3_mul(rot(tz, -side * ap->sh_out[s]), Rt));
        V3 el = v3_add(sh, m3_apply(Rua, v3(-UARM, 0, 0)));
        M3 Rfa = m3_mul(rot(m3_col(Rua, 1), ap->elbow[s]), Rua);
        rig->hand[s] = v3_add(el, m3_apply(Rfa, v3(-FARM, 0, 0)));
        rig->hand_dir[s] = m3_apply(Rfa, v3(-1, 0, 0));
        if (!first_person) add(rig, AP_UARM, Rua, sh);
        if (!first_person || !ap->no_arms) add(rig, AP_FARM, Rfa, el);
        if (ap->cdr && !first_person) add(rig, AP_BAND_ARM, Rua, sh);
    }
}

void astro_draw(const AstroPose *p, V3 cam_pos, int first_person)
{
    Rig r;
    solve(p, &r, first_person);
    for (int i = 0; i < r.n; i++) lmmodel_draw_part(r.pc[i].part, r.pc[i].R, v3_sub(r.pc[i].p, cam_pos), 0);
}

void astro_draw_depth(const AstroPose *p, V3 centre)
{
    Rig r;
    solve(p, &r, 0);
    for (int i = 0; i < r.n; i++) lmmodel_draw_part(r.pc[i].part, r.pc[i].R, v3_sub(r.pc[i].p, centre), 1);
}

V3 astro_hand(const AstroPose *p, int side, V3 *dir)
{
    Rig r;
    solve(p, &r, 1);
    if (dir) *dir = r.hand_dir[side ? 1 : 0];
    return r.hand[side ? 1 : 0];
}
