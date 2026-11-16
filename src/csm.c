#include <math.h>
#include <string.h>

#include "csm.h"
#include "moon.h"

Csm csm;

void csm_reset(void)
{
    memset(&csm, 0, sizeof csm);
    csm.q = q4_identity();
}

void csm_set_state(V3 r, V3 v)
{
    csm.r = r;
    csm.v = v;
    csm.valid = 1;
}

void csm_point_at(V3 target, V3 upish)
{
    V3 x = v3_norm(v3_sub(target, csm.r));
    V3 z = v3_sub(upish, v3_scale(x, v3_dot(upish, x)));
    if (v3_len(z) < 1e-6) z = v3_cross(x, fabs(x.x) < 0.9 ? v3(1, 0, 0) : v3(0, 1, 0));
    z = v3_norm(z);
    V3 y = v3_cross(z, x);
    M3 m;
    // Columns of inertial_from_body are the body axes in inertial coordinates.
    m.m[0][0] = x.x; m.m[0][1] = y.x; m.m[0][2] = z.x;
    m.m[1][0] = x.y; m.m[1][1] = y.y; m.m[1][2] = z.y;
    m.m[2][0] = x.z; m.m[2][1] = y.z; m.m[2][2] = z.z;
    csm.q = m3_to_q4(m);
    csm.w = v3(0, 0, 0);
}

void csm_step(double dt, V3 lm_r)
{
    if (!csm.valid) return;
    double rr = v3_len(csm.r);
    csm.v = v3_add(csm.v, v3_scale(csm.r, -MOON_MU / (rr * rr * rr) * dt));
    csm.r = v3_add(csm.r, v3_scale(csm.v, dt));
    if (csm.hold || csm.docked) {
        csm.q = q4_normalize(q4_mul(csm.q, q4_from_rotvec(v3_scale(csm.w, dt))));
    } else {
        // Tracking: the probe follows the LM, the crew window (+Z) away from the Moon.
        static double acc;
        acc += dt;
        if (acc >= 0.05) {
            acc = 0;
            csm_point_at(lm_r, v3_norm(csm.r));
        }
    }
}

void csm_propagate(V3 *r, V3 *v, double dt)
{
    double left = dt;
    while (fabs(left) > 1e-9) {
        double s = fabs(left) < 1.0 ? left : (left > 0 ? 1.0 : -1.0);
        V3 r0 = *r, v0 = *v;
#define ACC(p) v3_scale((p), -MOON_MU / pow(v3_len(p), 3))
        V3 k1v = ACC(r0), k1r = v0;
        V3 r2 = v3_add(r0, v3_scale(k1r, s / 2)), v2 = v3_add(v0, v3_scale(k1v, s / 2));
        V3 k2v = ACC(r2), k2r = v2;
        V3 r3 = v3_add(r0, v3_scale(k2r, s / 2)), v3b = v3_add(v0, v3_scale(k2v, s / 2));
        V3 k3v = ACC(r3), k3r = v3b;
        V3 r4 = v3_add(r0, v3_scale(k3r, s)), v4 = v3_add(v0, v3_scale(k3v, s));
        V3 k4v = ACC(r4), k4r = v4;
#undef ACC
        *r = v3_add(r0, v3_scale(v3_add(v3_add(k1r, v3_scale(k2r, 2)), v3_add(v3_scale(k3r, 2), k4r)), s / 6));
        *v = v3_add(v0, v3_scale(v3_add(v3_add(k1v, v3_scale(k2v, 2)), v3_add(v3_scale(k3v, 2), k4v)), s / 6));
        left -= s;
    }
}
