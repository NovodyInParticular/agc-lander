// Double-precision vector / matrix / quaternion helpers for the simulation.
#ifndef VMATH_H
#define VMATH_H

#include <math.h>

typedef struct { double x, y, z; } V3;
typedef struct { double m[3][3]; } M3;      // row-major, v' = M v
typedef struct { double w, x, y, z; } Q4;   // unit quaternion

#ifndef PI_D
#define PI_D 3.14159265358979323846
#endif

static inline V3 v3(double x, double y, double z) { V3 r = { x, y, z }; return r; }
static inline V3 v3_add(V3 a, V3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline V3 v3_sub(V3 a, V3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline V3 v3_scale(V3 a, double s) { return v3(a.x * s, a.y * s, a.z * s); }
static inline double v3_dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline V3 v3_cross(V3 a, V3 b)
{
    return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
static inline double v3_len(V3 a) { return sqrt(v3_dot(a, a)); }
static inline V3 v3_norm(V3 a)
{
    double l = v3_len(a);
    return l > 0 ? v3_scale(a, 1.0 / l) : a;
}

static inline M3 m3_identity(void)
{
    M3 r = { { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } } };
    return r;
}
static inline M3 m3_rows(V3 a, V3 b, V3 c)
{
    M3 r = { { { a.x, a.y, a.z }, { b.x, b.y, b.z }, { c.x, c.y, c.z } } };
    return r;
}
static inline V3 m3_row(M3 a, int i) { return v3(a.m[i][0], a.m[i][1], a.m[i][2]); }
static inline V3 m3_col(M3 a, int j) { return v3(a.m[0][j], a.m[1][j], a.m[2][j]); }
static inline M3 m3_mul(M3 a, M3 b)
{
    M3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
    return r;
}
static inline M3 m3_transpose(M3 a)
{
    M3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            r.m[i][j] = a.m[j][i];
    return r;
}
static inline V3 m3_apply(M3 a, V3 v)
{
    return v3(a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
              a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
              a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z);
}

// Passive (coordinate-frame) rotations, as used by the AGC's AX*SR*T:
// v_new = R v_old where the new frame is rotated by +a about the axis.
static inline M3 m3_rx(double a)
{
    double c = cos(a), s = sin(a);
    M3 r = { { { 1, 0, 0 }, { 0, c, s }, { 0, -s, c } } };
    return r;
}
static inline M3 m3_ry(double a)
{
    double c = cos(a), s = sin(a);
    M3 r = { { { c, 0, -s }, { 0, 1, 0 }, { s, 0, c } } };
    return r;
}
static inline M3 m3_rz(double a)
{
    double c = cos(a), s = sin(a);
    M3 r = { { { c, s, 0 }, { -s, c, 0 }, { 0, 0, 1 } } };
    return r;
}

static inline Q4 q4_identity(void) { Q4 q = { 1, 0, 0, 0 }; return q; }
static inline Q4 q4_mul(Q4 a, Q4 b)
{
    Q4 r = { a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
             a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
             a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
             a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w };
    return r;
}
static inline Q4 q4_normalize(Q4 q)
{
    double l = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    Q4 r = { q.w / l, q.x / l, q.y / l, q.z / l };
    return r;
}
// Rotation by angle about unit axis (active).
static inline Q4 q4_axis_angle(V3 axis, double angle)
{
    double s = sin(angle / 2);
    Q4 q = { cos(angle / 2), axis.x * s, axis.y * s, axis.z * s };
    return q;
}
// Rotation vector (axis * angle), small or large.
static inline Q4 q4_from_rotvec(V3 r)
{
    double a = v3_len(r);
    if (a < 1e-12) {
        Q4 q = { 1, r.x / 2, r.y / 2, r.z / 2 };
        return q4_normalize(q);
    }
    return q4_axis_angle(v3_scale(r, 1.0 / a), a);
}
// Active rotation matrix: columns are the rotated frame's axes.
static inline M3 q4_to_m3(Q4 q)
{
    double w = q.w, x = q.x, y = q.y, z = q.z;
    M3 r = { { { 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y) },
               { 2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x) },
               { 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y) } } };
    return r;
}
static inline Q4 m3_to_q4(M3 m)
{
    Q4 q;
    double t = m.m[0][0] + m.m[1][1] + m.m[2][2];
    if (t > 0) {
        double s = sqrt(t + 1.0) * 2;
        q.w = 0.25 * s;
        q.x = (m.m[2][1] - m.m[1][2]) / s;
        q.y = (m.m[0][2] - m.m[2][0]) / s;
        q.z = (m.m[1][0] - m.m[0][1]) / s;
    } else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2]) {
        double s = sqrt(1.0 + m.m[0][0] - m.m[1][1] - m.m[2][2]) * 2;
        q.w = (m.m[2][1] - m.m[1][2]) / s;
        q.x = 0.25 * s;
        q.y = (m.m[0][1] + m.m[1][0]) / s;
        q.z = (m.m[0][2] + m.m[2][0]) / s;
    } else if (m.m[1][1] > m.m[2][2]) {
        double s = sqrt(1.0 + m.m[1][1] - m.m[0][0] - m.m[2][2]) * 2;
        q.w = (m.m[0][2] - m.m[2][0]) / s;
        q.x = (m.m[0][1] + m.m[1][0]) / s;
        q.y = 0.25 * s;
        q.z = (m.m[1][2] + m.m[2][1]) / s;
    } else {
        double s = sqrt(1.0 + m.m[2][2] - m.m[0][0] - m.m[1][1]) * 2;
        q.w = (m.m[1][0] - m.m[0][1]) / s;
        q.x = (m.m[0][2] + m.m[2][0]) / s;
        q.y = (m.m[1][2] + m.m[2][1]) / s;
        q.z = 0.25 * s;
    }
    return q4_normalize(q);
}
static inline V3 q4_rotate(Q4 q, V3 v) { return m3_apply(q4_to_m3(q), v); }
static inline Q4 q4_conj(Q4 q) { Q4 r = { q.w, -q.x, -q.y, -q.z }; return r; }

#endif
