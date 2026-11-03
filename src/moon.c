#include "moon.h"

// Constants from Luminary099 CONTROLLED_CONSTANTS.agc (rad, rad/s).
#define B0     0.40916190299
#define BDOT  -7.19757301e-14
#define F0     5.20932947829
#define FDOT   2.67240410e-6
#define N0     6.19653663041
#define NDOT  -1.07047011e-8
#define COSI   0.99964173
#define SINI   0.02676579

M3 moon_matrix(double t)
{
    double b = B0 + BDOT * t, f = F0 + FDOT * t, n = N0 + NDOT * t;
    double cob = cos(b), sob = sin(b), cn = cos(n), sn = sin(n), cf = cos(f), sf = sin(f);
    V3 a = v3(cn, cob * sn, sob * sn);
    V3 bv = v3(-sn, cob * cn, sob * cn);
    V3 c = v3(0, -sob, cob);
    V3 m2 = v3_add(v3_scale(bv, SINI), v3_scale(c, COSI));
    V3 d = v3_sub(v3_scale(bv, COSI), v3_scale(c, SINI));
    V3 m1 = v3_sub(v3_scale(a, sf), v3_scale(d, cf));
    V3 m0 = v3_scale(v3_add(v3_scale(a, cf), v3_scale(d, sf)), -1.0);
    return m3_rows(m0, m1, m2);
}

V3 moon_fixed_to_inertial(V3 rp, double t) { return m3_apply(m3_transpose(moon_matrix(t)), rp); }
V3 moon_inertial_to_fixed(V3 r, double t) { return m3_apply(moon_matrix(t), r); }

V3 moon_omega(double t)
{
    // Numerical derivative: dM^T/dt M = [w]x  (skew-symmetric).
    const double h = 1.0;
    M3 a = m3_transpose(moon_matrix(t - h)), b = m3_transpose(moon_matrix(t + h));
    M3 m = moon_matrix(t);
    M3 d;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            d.m[i][j] = (b.m[i][j] - a.m[i][j]) / (2 * h);
    M3 w = m3_mul(d, m);
    return v3((w.m[2][1] - w.m[1][2]) / 2, (w.m[0][2] - w.m[2][0]) / 2, (w.m[1][0] - w.m[0][1]) / 2);
}

V3 moon_latlon_to_fixed(double lat, double lon, double radius)
{
    return v3(radius * cos(lat) * cos(lon), radius * cos(lat) * sin(lon), radius * sin(lat));
}

void moon_fixed_to_latlon(V3 rp, double *lat, double *lon, double *radius)
{
    *radius = v3_len(rp);
    *lat = asin(rp.z / *radius);
    *lon = atan2(rp.y, rp.x);
}
