// Procedural Command and Service Module "Columbia".
//
// CSM body axes (csm.h): origin at the docking ring on the CM apex, +X out
// along the probe, -X down the stack to the SPS nozzle. The LM-active docking
// target stands off the CM at (y +0.66, z +0.10), where the LM Commander's
// overhead window and COAS see it when the tunnels are lined up.

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "csm.h"
#include "csmmodel.h"
#include "render.h"

enum { M_GOLD = 1, M_METAL, M_BLACK, M_WHITE, M_MYLAR, M_GLASS, M_EMIT, M_CLOTH = 9, M_ANOD = 10 };
typedef struct { unsigned char r, g, b, m; } Mat;

// The CM's skin is aluminised Mylar tape over the heat shield: near-mirror.
static const Mat CM_SKIN = { 205, 205, 210, M_MYLAR };
static const Mat CM_DARK = { 60, 58, 56, M_METAL };
static const Mat SM_SKIN = { 182, 182, 186, M_METAL };
static const Mat SM_RAD = { 200, 200, 196, M_WHITE };
static const Mat K_BLACK = { 8, 8, 9, M_BLACK };
static const Mat GLASS = { 20, 25, 35, M_GLASS };
static const Mat SILVER = { 210, 210, 214, M_METAL };
static const Mat NOZ_IN = { 30, 28, 27, M_METAL };
static const Mat AFT_SHIELD = { 34, 32, 30, M_BLACK };   // black-coated blanket over the aft bulkhead
static const Mat NOZ_OUT = { 78, 72, 66, M_METAL };
static const Mat K_WHITE = { 215, 215, 210, M_WHITE };
static const Mat K_RED = { 178, 34, 52, M_WHITE }, K_BLUE = { 40, 50, 120, M_WHITE };

typedef struct { float *v, *n; unsigned char *c; int count, cap; } Builder;
static Builder B;

static void push_vert(Vector3 p, Vector3 n, Mat m)
{
    if (B.count == B.cap) {
        B.cap = B.cap ? B.cap * 2 : 4096;
        B.v = realloc(B.v, sizeof(float) * 3 * B.cap);
        B.n = realloc(B.n, sizeof(float) * 3 * B.cap);
        B.c = realloc(B.c, 4 * B.cap);
    }
    int k = B.count++;
    B.v[3 * k] = p.x; B.v[3 * k + 1] = p.y; B.v[3 * k + 2] = p.z;
    B.n[3 * k] = n.x; B.n[3 * k + 1] = n.y; B.n[3 * k + 2] = n.z;
    B.c[4 * k] = m.r; B.c[4 * k + 1] = m.g; B.c[4 * k + 2] = m.b; B.c[4 * k + 3] = m.m;
}

static Vector3 V(float x, float y, float z) { return (Vector3){ x, y, z }; }

static void tri(Vector3 a, Vector3 b, Vector3 c, Mat m)
{
    Vector3 n = Vector3Normalize(Vector3CrossProduct(Vector3Subtract(b, a), Vector3Subtract(c, a)));
    push_vert(a, n, m); push_vert(b, n, m); push_vert(c, n, m);
}

static void quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Mat m) { tri(a, b, c, m); tri(a, c, d, m); }

// Surface of revolution about X between two stations: (x0, r0) -> (x1, r1),
// from angle t0 to t1 (radians), smooth normals. Faces outward.
static void band_dir(float x0, float r0, float x1, float r1, float t0, float t1, int seg, Mat m, int inward);
static void band(float x0, float r0, float x1, float r1, float t0, float t1, int seg, Mat m)
{
    band_dir(x0, r0, x1, r1, t0, t1, seg, m, 0);
}

// The same surface facing the axis (the inside of a nozzle).
static void band_in(float x0, float r0, float x1, float r1, float t0, float t1, int seg, Mat m)
{
    band_dir(x0, r0, x1, r1, t0, t1, seg, m, 1);
}

static void band_dir(float x0, float r0, float x1, float r1, float t0, float t1, int seg, Mat m, int inward)
{
    float dx = x1 - x0, dr = r1 - r0;
    float len = sqrtf(dx * dx + dr * dr);
    // Outward normal of the profile line, in the (x, r) plane.
    float nx = len > 0 ? -dr / len : 0, nr = len > 0 ? dx / len : 1;
    if (x1 < x0) { nx = -nx; nr = -nr; }
    if (inward) { nx = -nx; nr = -nr; }
    for (int i = 0; i < seg; i++) {
        float a0 = t0 + (t1 - t0) * i / seg, a1 = t0 + (t1 - t0) * (i + 1) / seg;
        float c0 = cosf(a0), s0 = sinf(a0), c1 = cosf(a1), s1 = sinf(a1);
        Vector3 p00 = V(x0, r0 * c0, r0 * s0), p01 = V(x0, r0 * c1, r0 * s1);
        Vector3 p10 = V(x1, r1 * c0, r1 * s0), p11 = V(x1, r1 * c1, r1 * s1);
        Vector3 n0 = V(nx, nr * c0, nr * s0), n1 = V(nx, nr * c1, nr * s1);
        // Wind so the face points along the normal.
        Vector3 fn = Vector3CrossProduct(Vector3Subtract(p10, p00), Vector3Subtract(p01, p00));
        if (Vector3DotProduct(fn, n0) >= 0) {
            push_vert(p00, n0, m); push_vert(p10, n0, m); push_vert(p11, n1, m);
            push_vert(p00, n0, m); push_vert(p11, n1, m); push_vert(p01, n1, m);
        } else {
            push_vert(p00, n0, m); push_vert(p11, n1, m); push_vert(p10, n0, m);
            push_vert(p00, n0, m); push_vert(p01, n1, m); push_vert(p11, n1, m);
        }
    }
}

static void disc(float x, float r, int facing, int seg, Mat m)
{
    for (int i = 0; i < seg; i++) {
        float a0 = 2 * PI * i / seg, a1 = 2 * PI * (i + 1) / seg;
        Vector3 c = V(x, 0, 0), p0 = V(x, r * cosf(a0), r * sinf(a0)), p1 = V(x, r * cosf(a1), r * sinf(a1));
        // Counter-clockwise seen from +X faces +X.
        if (facing > 0) tri(c, p0, p1, m); else tri(c, p1, p0, m);
    }
}

// Tube between two points (radius ra -> rb), smooth sides, optional caps.
static void tube(Vector3 a, Vector3 b, float ra, float rb, int seg, Mat m, int caps)
{
    Vector3 axis = Vector3Normalize(Vector3Subtract(b, a));
    Vector3 t = fabsf(axis.x) < 0.9f ? V(1, 0, 0) : V(0, 1, 0);
    Vector3 u = Vector3Normalize(Vector3CrossProduct(axis, t));
    Vector3 w = Vector3CrossProduct(axis, u);
    float slope = (ra - rb) / Vector3Distance(a, b);
    for (int i = 0; i < seg; i++) {
        float t0 = 2 * PI * i / seg, t1 = 2 * PI * (i + 1) / seg;
        Vector3 d0 = Vector3Add(Vector3Scale(u, cosf(t0)), Vector3Scale(w, sinf(t0)));
        Vector3 d1 = Vector3Add(Vector3Scale(u, cosf(t1)), Vector3Scale(w, sinf(t1)));
        Vector3 n0 = Vector3Normalize(Vector3Add(d0, Vector3Scale(axis, slope)));
        Vector3 n1 = Vector3Normalize(Vector3Add(d1, Vector3Scale(axis, slope)));
        Vector3 a0 = Vector3Add(a, Vector3Scale(d0, ra)), a1 = Vector3Add(a, Vector3Scale(d1, ra));
        Vector3 b0 = Vector3Add(b, Vector3Scale(d0, rb)), b1 = Vector3Add(b, Vector3Scale(d1, rb));
        push_vert(a0, n0, m); push_vert(a1, n1, m); push_vert(b1, n1, m);
        push_vert(a0, n0, m); push_vert(b1, n1, m); push_vert(b0, n0, m);
        if (caps) { tri(a, a1, a0, m); tri(b, b0, b1, m); }
    }
}

static void box(Vector3 lo, Vector3 hi, Mat m)
{
    Vector3 p[8];
    for (int i = 0; i < 8; i++) p[i] = V(i & 1 ? hi.x : lo.x, i & 2 ? hi.y : lo.y, i & 4 ? hi.z : lo.z);
    quad(p[0], p[2], p[3], p[1], m); quad(p[4], p[5], p[7], p[6], m);   // -z, +z
    quad(p[0], p[1], p[5], p[4], m); quad(p[2], p[6], p[7], p[3], m);   // -y, +y
    quad(p[0], p[4], p[6], p[2], m); quad(p[1], p[3], p[7], p[5], m);   // -x, +x
}

// A flat panel lying on the cylinder (radius r) between angles a0..a1, x0..x1, raised by `off`.
static void skin_panel(float r, float off, float x0, float x1, float a0, float a1, Mat m)
{
    band(x1, r + off, x0, r + off, a0, a1, 6, m);
}

// Point on the CM cone at station x (x between the apex ring and the base), angle a.
static float cm_radius(float x) { return 0.50f + (CSM_R - 0.50f) * (-0.30f - x) / (-0.30f - (float)CSM_CM_BASE); }

static void build(void)
{
    const float R = (float)CSM_R, XB = (float)CSM_CM_BASE, XS = (float)CSM_SM_END;

    // --- Command module ------------------------------------------------------------
    band(0.0f, 0.42f, -0.30f, 0.50f, 0, 2 * PI, 32, SILVER);          // docking ring / tunnel
    band(-0.30f, 0.50f, XB, R, 0, 2 * PI, 48, CM_SKIN);                // conical crew compartment
    band(XB, R, XB - 0.10f, R - 0.02f, 0, 2 * PI, 48, CM_DARK);        // heat shield shoulder
    // The docking tunnel: its inner wall and, 0.28 m in, the forward hatch the probe mounts on.
    band_in(0.0f, 0.415f, -0.28f, 0.415f, 0, 2 * PI, 32, CM_DARK);
    disc(-0.28f, 0.42f, 1, 32, SILVER);
    for (int k = 0; k < 8; k++) {   // hatch latch blocks round the rim
        float a = k * PI / 4;
        Vector3 c = V(-0.26f, 0.36f * cosf(a), 0.36f * sinf(a));
        box(Vector3Subtract(c, V(0.02f, 0.03f, 0.03f)), Vector3Add(c, V(0.02f, 0.03f, 0.03f)), CM_DARK);
    }
    // Five windows: two forward-looking rendezvous windows, two side windows, the hatch window.
    {
        const float wa[5] = { 0.45f, -0.45f, 1.45f, -1.45f, 0.0f };     // angle from +Z
        const float wx[5] = { -0.95f, -0.95f, -1.75f, -1.75f, -1.55f };
        const float ws[5] = { 0.18f, 0.18f, 0.24f, 0.24f, 0.22f };
        for (int i = 0; i < 5; i++) {
            float a = PI / 2 - wa[i];   // +Z is angle pi/2 in the (y, z) circle
            float x = wx[i], r = cm_radius(x) + 0.012f;
            float da = ws[i] / r;
            band(x + ws[i] * 0.6f, cm_radius(x + ws[i] * 0.6f) + 0.012f, x - ws[i] * 0.6f, cm_radius(x - ws[i] * 0.6f) + 0.012f,
                 a - da, a + da, 3, GLASS);
            band(x + ws[i] * 0.75f, cm_radius(x + ws[i] * 0.75f) + 0.008f, x + ws[i] * 0.6f, cm_radius(x + ws[i] * 0.6f) + 0.008f,
                 a - da * 1.2f, a + da * 1.2f, 3, K_BLACK);
            (void)r;
        }
    }
    // Hatch outline and the CM RCS thruster ports (pitch, yaw, roll pairs).
    {
        float a = PI / 2, x0 = -1.15f, x1 = -2.15f;
        band(x0, cm_radius(x0) + 0.006f, x1, cm_radius(x1) + 0.006f, a - 0.38f, a - 0.35f, 2, CM_DARK);
        band(x0, cm_radius(x0) + 0.006f, x1, cm_radius(x1) + 0.006f, a + 0.35f, a + 0.38f, 2, CM_DARK);
        for (int k = 0; k < 6; k++) {
            float aa = k * PI / 3 + 0.25f, x = -0.75f - (k & 1) * 0.9f;
            Vector3 c = V(x, cm_radius(x) * cosf(aa), cm_radius(x) * sinf(aa));
            tube(c, Vector3Add(c, V(0.02f, 0.05f * cosf(aa), 0.05f * sinf(aa))), 0.05f, 0.05f, 8, K_BLACK, 1);
        }
    }
    // Docking probe: a slim boom from a three-armed spider on the ring, capture latches at the tip.
    tube(V(-0.05f, 0, 0), V((float)CSM_PROBE_TIP - 0.06f, 0, 0), 0.045f, 0.035f, 12, SILVER, 0);
    tube(V((float)CSM_PROBE_TIP - 0.06f, 0, 0), V((float)CSM_PROBE_TIP, 0, 0), 0.035f, 0.012f, 12, SILVER, 1);
    for (int k = 0; k < 3; k++) {
        float a = k * 2 * PI / 3;
        tube(V(0.02f, 0.40f * cosf(a), 0.40f * sinf(a)), V(0.28f, 0.03f * cosf(a), 0.03f * sinf(a)), 0.022f, 0.022f, 6, SILVER, 1);
        Vector3 t = V((float)CSM_PROBE_TIP - 0.10f, 0.04f * cosf(a), 0.04f * sinf(a));
        tube(t, Vector3Add(t, V(0.06f, 0.02f * cosf(a), 0.02f * sinf(a))), 0.010f, 0.006f, 5, K_BLACK, 1);
    }
    // LM-active docking target: base disc on a short stand, a standoff cross 0.3 m out.
    {
        float ty = 0.66f, tz = 0.10f, x0 = -0.61f;
        tube(V(x0 - 0.04f, ty, tz), V(x0 + 0.06f, ty, tz), 0.03f, 0.03f, 8, SILVER, 0);
        Vector3 c = V(x0 + 0.07f, ty, tz);
        tube(c, Vector3Add(c, V(0.012f, 0, 0)), 0.16f, 0.16f, 24, K_WHITE, 1);
        box(Vector3Add(c, V(0.012f, -0.15f, -0.012f)), Vector3Add(c, V(0.016f, 0.15f, 0.012f)), K_BLACK);
        box(Vector3Add(c, V(0.012f, -0.012f, -0.15f)), Vector3Add(c, V(0.016f, 0.012f, 0.15f)), K_BLACK);
        tube(Vector3Add(c, V(0.016f, 0, 0)), Vector3Add(c, V(0.30f, 0, 0)), 0.010f, 0.010f, 6, K_BLACK, 0);
        Vector3 s = Vector3Add(c, V(0.30f, 0, 0));
        box(Vector3Add(s, V(-0.008f, -0.07f, -0.009f)), Vector3Add(s, V(0.008f, 0.07f, 0.009f)), K_WHITE);
        box(Vector3Add(s, V(-0.008f, -0.009f, -0.07f)), Vector3Add(s, V(0.008f, 0.009f, 0.07f)), K_WHITE);
    }

    // --- CM / SM fairing and the service module ------------------------------------------
    band(XB - 0.10f, R - 0.02f, XB - 0.35f, R, 0, 2 * PI, 48, SM_RAD);      // fairing with radiators
    for (int k = 0; k < 8; k++)
        skin_panel(R, 0.004f, XB - 0.34f, XB - 0.12f, k * PI / 4 + 0.05f, k * PI / 4 + 0.30f, SILVER);
    band(XB - 0.35f, R, XS, R, 0, 2 * PI, 48, SM_SKIN);                     // SM body
    disc(XS, R, -1, 48, AFT_SHIELD);                                        // aft bulkhead heat shield
    // EPS radiator panels (white) and the ECS radiators lower down; sector seams.
    for (int k = 0; k < 6; k++) {
        float a = k * PI / 3 + 0.18f;
        skin_panel(R, 0.006f, XB - 0.55f, XB - 1.55f, a, a + 0.62f, SM_RAD);
        skin_panel(R, 0.004f, XS + 0.25f, XS + 1.35f, a + 0.1f, a + 0.45f, SM_RAD);
        skin_panel(R, 0.008f, XB - 0.35f, XS, a - 0.20f, a - 0.18f, CM_DARK);
    }
    // "UNITED STATES" band stand-in and the flag on the side facing the LM's window.
    skin_panel(R, 0.009f, -4.45f, -4.65f, PI / 2 - 0.55f, PI / 2 + 0.55f, K_BLACK);
    for (int i = 0; i < 7; i++)
        skin_panel(R, 0.010f, -5.00f - i * 0.06f, -5.06f - i * 0.06f, PI / 2 + 0.05f, PI / 2 + 0.55f, (i & 1) ? K_WHITE : K_RED);
    skin_panel(R, 0.011f, -5.00f, -5.24f, PI / 2 + 0.33f, PI / 2 + 0.55f, K_BLUE);

    // RCS quads: four "doghouse" housings, each with four thrusters (two along X, two round the hull).
    for (int q = 0; q < 4; q++) {
        float a = q * PI / 2 + 7.25f * DEG2RAD;
        Vector3 rad = V(0, cosf(a), sinf(a)), tng = V(0, -sinf(a), cosf(a));
        Vector3 c = Vector3Add(V(-4.30f, 0, 0), Vector3Scale(rad, R + 0.10f));
        Vector3 lo = Vector3Add(c, V(-0.25f, 0, 0));
        for (int s = 0; s < 2; s++) {
            Vector3 p = Vector3Add(lo, Vector3Scale(tng, s ? 0.18f : -0.18f));
            tube(Vector3Add(p, Vector3Scale(rad, -0.10f)), Vector3Add(p, Vector3Scale(rad, 0.08f)), 0.10f, 0.10f, 8, SILVER, 1);
        }
        tube(Vector3Add(c, Vector3Scale(rad, -0.12f)), Vector3Add(c, Vector3Scale(rad, 0.10f)), 0.26f, 0.22f, 10, K_WHITE, 1);
        tube(Vector3Add(c, V(0.18f, 0, 0)), Vector3Add(c, V(0.42f, 0, 0)), 0.035f, 0.075f, 10, NOZ_IN, 0);
        tube(Vector3Add(c, V(-0.18f, 0, 0)), Vector3Add(c, V(-0.42f, 0, 0)), 0.035f, 0.075f, 10, NOZ_IN, 0);
        tube(Vector3Add(c, Vector3Scale(tng, 0.18f)), Vector3Add(c, Vector3Scale(tng, 0.40f)), 0.035f, 0.07f, 10, NOZ_IN, 0);
        tube(Vector3Add(c, Vector3Scale(tng, -0.18f)), Vector3Add(c, Vector3Scale(tng, -0.40f)), 0.035f, 0.07f, 10, NOZ_IN, 0);
    }
    // Scimitar VHF antennas.
    for (int s = -1; s <= 1; s += 2) {
        Vector3 base = V(-5.6f, 0, s * (R + 0.02f));
        Vector3 p0 = base;
        for (int k = 1; k <= 6; k++) {
            float t = k / 6.0f;
            Vector3 p = V(-5.6f + 0.9f * t, 0, s * (R + 0.02f + 0.55f * sinf(t * PI * 0.85f)));
            tube(p0, p, 0.03f, 0.03f, 6, K_WHITE, 0);
            p0 = p;
        }
    }

    // --- SPS engine: dark columbium nozzle extension below the aft heat shield ----------------
    band(XS, 0.62f, XS - 0.30f, 0.45f, 0, 2 * PI, 32, CM_DARK);
    band(XS - 0.30f, 0.45f, (float)CSM_NOZZLE_END, 1.25f, 0, 2 * PI, 40, NOZ_OUT);
    band_in((float)CSM_NOZZLE_END, 1.235f, XS - 0.30f, 0.43f, 0, 2 * PI, 40, NOZ_IN);   // inner wall
    disc(XS - 0.29f, 0.44f, -1, 24, K_BLACK);                                            // the throat
    for (int k = 1; k < 4; k++) {
        float t = k / 4.0f, x = XS - 0.30f + ((float)CSM_NOZZLE_END - (XS - 0.30f)) * t, r = 0.45f + 0.80f * t;
        band(x + 0.03f, r + 0.008f, x - 0.03f, r + 0.010f, 0, 2 * PI, 40, CM_DARK);
    }

    // --- High-gain antenna on its boom off the aft bulkhead -------------------------------------
    {
        float a = -PI / 2 + 0.2f;
        Vector3 root = V(XS + 0.05f, (R - 0.25f) * cosf(a), (R - 0.25f) * sinf(a));
        Vector3 tip = Vector3Add(root, V(-1.30f, 0.65f * cosf(a), 0.65f * sinf(a)));
        tube(root, tip, 0.05f, 0.05f, 8, K_WHITE, 1);
        Vector3 hub = Vector3Add(tip, V(-0.25f, 0, 0));
        box(Vector3Subtract(hub, V(0.15f, 0.15f, 0.15f)), Vector3Add(hub, V(0.15f, 0.15f, 0.15f)), SILVER);
        for (int k = 0; k < 4; k++) {
            Vector3 c = Vector3Add(hub, V(-0.05f, 0.42f * ((k & 1) ? 1 : -1), 0.42f * ((k & 2) ? 1 : -1)));
            // Each dish faces -X (aft), a shallow cone.
            tube(c, Vector3Add(c, V(-0.10f, 0, 0)), 0.05f, 0.40f, 20, K_WHITE, 0);
            tube(Vector3Add(c, V(-0.10f, 0, 0)), c, 0.40f, 0.05f, 20, SILVER, 0);
        }
    }

    // Docking lights and running lights.
    box(V(-1.0f, -0.05f, cm_radius(-1.0f) - 0.02f), V(-0.92f, 0.05f, cm_radius(-1.0f) + 0.03f), (Mat){ 70, 60, 40, M_EMIT });
    box(V(-4.0f, -R - 0.03f, -0.06f), V(-3.9f, -R + 0.01f, 0.06f), (Mat){ 70, 6, 4, M_EMIT });
    box(V(-4.0f, R - 0.01f, -0.06f), V(-3.9f, R + 0.03f, 0.06f), (Mat){ 4, 60, 10, M_EMIT });
}

static Mesh csm_mesh;
static Material csm_mat;
static int ready;

void csmmodel_init(void)
{
    memset(&B, 0, sizeof B);
    build();
    Mesh m = { 0 };
    m.vertexCount = B.count;
    m.triangleCount = B.count / 3;
    m.vertices = B.v;
    m.normals = B.n;
    m.colors = B.c;
    UploadMesh(&m, false);
    csm_mesh = m;
    csm_mat = LoadMaterialDefault();
    render_lm_material(&csm_mat);
    ready = 1;
    TraceLog(LOG_INFO, "CSM model: %d triangles", m.triangleCount);
}

static Matrix to_matrix(M3 r, V3 t)
{
    Matrix m = {
        (float)r.m[0][0], (float)r.m[0][1], (float)r.m[0][2], (float)t.x,
        (float)r.m[1][0], (float)r.m[1][1], (float)r.m[1][2], (float)t.y,
        (float)r.m[2][0], (float)r.m[2][1], (float)r.m[2][2], (float)t.z,
        0, 0, 0, 1,
    };
    return m;
}

void csmmodel_draw(M3 site_from_body, V3 pos_cam_rel)
{
    if (!ready) return;
    DrawMesh(csm_mesh, csm_mat, to_matrix(site_from_body, pos_cam_rel));
}
