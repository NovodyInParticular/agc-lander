// Procedural Lunar Module model (descent + ascent stage) and its effects:
// DPS plume, RCS puffs, blown-dust sheet.
//
// Authored in LM body axes, metres, origin at the centre of mass:
// +X up (thrust axis), +Y right, +Z forward. Footpad plane at X = -2.75.

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "lm.h"
#include "lmmodel.h"
#include "render.h"

#define H0 (-2.75f)   // footpad plane in body X
#define DPS_BELL_LEN 1.18f   // nozzle exit below the bottom of the descent stage

// Materials (vertex alpha): 1 gold foil, 2 metal, 3 black paint, 4 white paint,
// 5 mylar blanket, 6 glass, 7 emissive.
enum { M_GOLD = 1, M_METAL, M_BLACK, M_WHITE, M_MYLAR, M_GLASS, M_EMIT };

typedef struct { unsigned char r, g, b, m; } Mat;
static const Mat MT_GOLD = { 220, 160, 60, M_GOLD };
static const Mat MT_AMBER = { 190, 110, 40, M_GOLD };
static const Mat MT_SILVER = { 200, 200, 205, M_METAL };
static const Mat MT_GREY = { 150, 150, 152, M_METAL };
static const Mat MT_DARKMETAL = { 70, 66, 62, M_METAL };
static const Mat MT_BLACK = { 18, 18, 20, M_BLACK };
static const Mat MT_WHITE = { 225, 225, 220, M_WHITE };
static const Mat MT_MYLAR = { 175, 175, 180, M_MYLAR };
static const Mat MT_GLASS = { 20, 25, 35, M_GLASS };
static const Mat MT_DPSIN = { 40, 34, 30, M_METAL };

// --- Mesh builder -------------------------------------------------------------

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

static void tri(Vector3 a, Vector3 b, Vector3 c, Mat m)
{
    Vector3 n = Vector3Normalize(Vector3CrossProduct(Vector3Subtract(b, a), Vector3Subtract(c, a)));
    push_vert(a, n, m); push_vert(b, n, m); push_vert(c, n, m);
}

static void tri_n(Vector3 a, Vector3 b, Vector3 c, Vector3 na, Vector3 nb, Vector3 nc, Mat m)
{
    push_vert(a, na, m); push_vert(b, nb, m); push_vert(c, nc, m);
}

static void quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Mat m)
{
    tri(a, b, c, m);
    tri(a, c, d, m);
}

static Vector3 V(float x, float y, float z) { return (Vector3){ x, y, z }; }

// Convex polygon prism along body X from x0 to x1; pts in (y, z).
static void prism(const float (*pts)[2], int n, float x0, float x1, Mat side, Mat cap_bottom, Mat cap_top)
{
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        quad(V(x0, pts[i][0], pts[i][1]), V(x0, pts[j][0], pts[j][1]),
             V(x1, pts[j][0], pts[j][1]), V(x1, pts[i][0], pts[i][1]), side);
    }
    for (int i = 1; i + 1 < n; i++) {
        tri(V(x1, pts[0][0], pts[0][1]), V(x1, pts[i + 1][0], pts[i + 1][1]), V(x1, pts[i][0], pts[i][1]), cap_top);
        tri(V(x0, pts[0][0], pts[0][1]), V(x0, pts[i][0], pts[i][1]), V(x0, pts[i + 1][0], pts[i + 1][1]), cap_bottom);
    }
}

// Tube / frustum between two points, smooth normals.
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
        tri_n(a0, a1, b1, n0, n1, n1, m);
        tri_n(a0, b1, b0, n0, n1, n0, m);
        if (caps) {
            tri(a, a1, a0, m);
            tri(b, b0, b1, m);
        }
    }
}

// Shallow dish (paraboloid) facing `dir`.
static void dish(Vector3 c, Vector3 dir, float r, float depth, Mat front, Mat back)
{
    Vector3 axis = Vector3Normalize(dir);
    Vector3 t = fabsf(axis.x) < 0.9f ? V(1, 0, 0) : V(0, 1, 0);
    Vector3 u = Vector3Normalize(Vector3CrossProduct(axis, t));
    Vector3 w = Vector3CrossProduct(axis, u);
    const int seg = 20, rings = 5;
    for (int k = 0; k < rings; k++)
        for (int i = 0; i < seg; i++) {
            float r0 = r * k / rings, r1 = r * (k + 1) / rings;
            float t0 = 2 * PI * i / seg, t1 = 2 * PI * (i + 1) / seg;
#define DP(rr, tt) Vector3Add(c, Vector3Add(Vector3Add(Vector3Scale(u, (rr) * cosf(tt)), Vector3Scale(w, (rr) * sinf(tt))), Vector3Scale(axis, depth * (rr) * (rr) / (r * r))))
            Vector3 p00 = DP(r0, t0), p01 = DP(r0, t1), p10 = DP(r1, t0), p11 = DP(r1, t1);
#undef DP
            quad(p00, p10, p11, p01, front);
            quad(p00, p01, p11, p10, back);
        }
}

static void box(Vector3 lo, Vector3 hi, Mat m)
{
    float pts[4][2] = { { lo.y, lo.z }, { hi.y, lo.z }, { hi.y, hi.z }, { lo.y, hi.z } };
    prism(pts, 4, lo.x, hi.x, m, m, m);
}

// --- The vehicle -----------------------------------------------------------------

static void build_descent_stage(void)
{
    // Cruciform body: square with chamfered corners (the four quadrant bays).
    const float s = 2.11f, c = 0.62f;
    float oct[8][2] = { { -s + c, -s }, { s - c, -s }, { s, -s + c }, { s, s - c },
                        { s - c, s }, { -s + c, s }, { -s, s - c }, { -s, -s + c } };
    float x0 = H0 + 1.55f, x1 = H0 + 3.15f;
    prism(oct, 8, x0, x1, MT_GOLD, MT_AMBER, MT_BLACK);
    // Black thermal band near the top and a darker lower skirt.
    float oct2[8][2];
    for (int i = 0; i < 8; i++) { oct2[i][0] = oct[i][0] * 1.006f; oct2[i][1] = oct[i][1] * 1.006f; }
    prism(oct2, 8, x1 - 0.22f, x1 - 0.05f, MT_BLACK, MT_BLACK, MT_BLACK);
    prism(oct2, 8, x0, x0 + 0.12f, MT_MYLAR, MT_MYLAR, MT_MYLAR);
    // Quadrant bay panels with crinkled foil insets.
    for (int q = 0; q < 4; q++) {
        float ang = q * PI / 2;
        float cy = cosf(ang), cz = sinf(ang);
        Vector3 n = V(0, cy, cz), tng = V(0, -cz, cy);
        Vector3 ctr = Vector3Add(V(x0 + 0.85f, 0, 0), Vector3Scale(n, s + 0.012f));
        Vector3 a = Vector3Add(ctr, Vector3Add(Vector3Scale(tng, -0.9f), V(-0.55f, 0, 0)));
        Vector3 b = Vector3Add(ctr, Vector3Add(Vector3Scale(tng, 0.9f), V(-0.55f, 0, 0)));
        Vector3 cc = Vector3Add(ctr, Vector3Add(Vector3Scale(tng, 0.9f), V(0.45f, 0, 0)));
        Vector3 d = Vector3Add(ctr, Vector3Add(Vector3Scale(tng, -0.9f), V(0.45f, 0, 0)));
        quad(a, b, cc, d, (q & 1) ? MT_AMBER : MT_GOLD);
    }
    // DPS: throat, bell, nozzle extension; heat shield under the stage.
    // Curved bell (radius grows fast below the throat, then flattens toward the
    // exit). Upper part wrapped in a thermal blanket, radiatively cooled dark
    // extension below, bright stiffening ring at the lip, inner wall all the way
    // up to a closed throat so the bell never looks hollow or cut off.
    {
        const float len = DPS_BELL_LEN, rt = 0.30f, re = 0.78f, d0 = 0.12f;
        const int nseg = 10;
        tube(V(x0 + 0.05f, 0, 0), V(x0 - d0, 0, 0), 0.40f, rt + 0.04f, 28, MT_BLACK, 0);   // chamber / throat
        for (int i = 0; i < nseg; i++) {
            float t0 = (float)i / nseg, t1 = (float)(i + 1) / nseg;
            float da = d0 + (len - d0) * t0, db = d0 + (len - d0) * t1;
            float ra = rt + (re - rt) * sqrtf(t0) + 0.04f, rb = rt + (re - rt) * sqrtf(t1) + 0.04f;
            Mat m = t1 <= 0.45f ? MT_MYLAR : MT_DARKMETAL;
            tube(V(x0 - da, 0, 0), V(x0 - db, 0, 0), ra, rb, 36, m, 0);
            tube(V(x0 - db, 0, 0), V(x0 - da, 0, 0), rb - 0.02f, ra - 0.02f, 36, MT_DPSIN, 0);   // inner wall
        }
        // Blanket seam band and exit lip.
        float tb = 0.45f, rbnd = rt + (re - rt) * sqrtf(tb) + 0.05f, dbnd = d0 + (len - d0) * tb;
        tube(V(x0 - dbnd + 0.03f, 0, 0), V(x0 - dbnd - 0.03f, 0, 0), rbnd, rbnd + 0.01f, 36, MT_GREY, 0);
        tube(V(x0 - len + 0.05f, 0, 0), V(x0 - len, 0, 0), re + 0.05f, re + 0.055f, 36, MT_SILVER, 1);
        tube(V(x0 - d0 - 0.02f, 0, 0), V(x0 - d0 + 0.02f, 0, 0), rt + 0.03f, rt + 0.03f, 24, MT_BLACK, 1);  // throat plug
    }
    float under[8][2];
    for (int i = 0; i < 8; i++) { under[i][0] = oct[i][0] * 0.97f; under[i][1] = oct[i][1] * 0.97f; }
    prism(under, 8, x0 - 0.04f, x0, MT_MYLAR, MT_MYLAR, MT_MYLAR);

    // Landing gear: primary strut, two secondaries, footpad, probe.
    for (int k = 0; k < 4; k++) {
        float ang = k * PI / 2;   // 0 = +Z (forward), then +Y, -Z, -Y
        float sy = sinf(ang), sz = cosf(ang);
        Vector3 r = V(0, sy, sz), t = V(0, sz, -sy);
        Vector3 top = Vector3Add(V(x1 - 0.35f, 0, 0), Vector3Scale(r, s + 0.15f));
        Vector3 knee = Vector3Add(V(H0 + 0.42f, 0, 0), Vector3Scale(r, 4.30f));
        tube(top, knee, 0.085f, 0.075f, 12, MT_GOLD, 1);
        tube(Vector3Add(top, Vector3Scale(r, 0.02f)), Vector3Lerp(top, knee, 0.55f), 0.11f, 0.10f, 12, MT_AMBER, 0);
        // Outrigger box where the strut meets the stage.
        box(Vector3Add(V(x1 - 0.55f, 0, 0), Vector3Add(Vector3Scale(r, s - 0.05f), V(0, -0.15f * fabsf(sz) - 0.15f * fabsf(sy), -0.15f * fabsf(sz) - 0.15f * fabsf(sy)))),
            Vector3Add(V(x1 - 0.15f, 0, 0), Vector3Add(Vector3Scale(r, s + 0.25f), V(0, 0.15f * fabsf(sz) + 0.15f * fabsf(sy), 0.15f * fabsf(sz) + 0.15f * fabsf(sy)))), MT_BLACK);
        // Secondary struts from the knee to the lower stage corners.
        for (int side = -1; side <= 1; side += 2) {
            Vector3 base = Vector3Add(V(x0 + 0.15f, 0, 0), Vector3Add(Vector3Scale(r, s - 0.1f), Vector3Scale(t, side * 1.05f)));
            Vector3 mid = Vector3Add(V(H0 + 0.95f, 0, 0), Vector3Scale(r, 3.75f));
            tube(base, mid, 0.04f, 0.04f, 8, MT_SILVER, 1);
        }
        // Deployment truss.
        tube(Vector3Add(V(x0 + 0.05f, 0, 0), Vector3Scale(r, s)), Vector3Add(V(H0 + 0.95f, 0, 0), Vector3Scale(r, 3.75f)), 0.035f, 0.035f, 8, MT_SILVER, 1);
        // Footpad: shallow dish, 0.94 m diameter.
        Vector3 pad = Vector3Add(V(H0 + 0.02f, 0, 0), Vector3Scale(r, 4.40f));
        tube(Vector3Add(pad, V(0.30f, 0, 0)), knee, 0.07f, 0.07f, 10, MT_SILVER, 0);
        tube(pad, Vector3Add(pad, V(0.25f, 0, 0)), 0.47f, 0.43f, 24, MT_GOLD, 1);
        // Contact probe (not on the forward leg, which has the ladder).
        if (k != 0) {
            Vector3 pb = Vector3Add(pad, V(0, 0, 0));
            tube(pb, Vector3Add(pb, V(-1.73f, 0, 0)), 0.018f, 0.018f, 6, MT_SILVER, 1);
            tube(Vector3Add(pb, V(-1.73f, 0, 0)), Vector3Add(pb, V(-1.68f, 0, 0)), 0.04f, 0.04f, 6, MT_BLACK, 1);
        } else {
            // Ladder on the forward strut, plus the porch.
            for (int rung = 0; rung < 9; rung++) {
                float f = 0.12f + rung * 0.095f;
                Vector3 p = Vector3Lerp(top, knee, f);
                p = Vector3Add(p, Vector3Scale(r, 0.16f));
                tube(Vector3Add(p, Vector3Scale(t, -0.24f)), Vector3Add(p, Vector3Scale(t, 0.24f)), 0.018f, 0.018f, 6, MT_SILVER, 1);
            }
            Vector3 l0 = Vector3Add(Vector3Lerp(top, knee, 0.08f), Vector3Scale(r, 0.16f));
            Vector3 l1 = Vector3Add(Vector3Lerp(top, knee, 0.95f), Vector3Scale(r, 0.16f));
            tube(Vector3Add(l0, Vector3Scale(t, -0.25f)), Vector3Add(l1, Vector3Scale(t, -0.25f)), 0.025f, 0.025f, 6, MT_SILVER, 1);
            tube(Vector3Add(l0, Vector3Scale(t, 0.25f)), Vector3Add(l1, Vector3Scale(t, 0.25f)), 0.025f, 0.025f, 6, MT_SILVER, 1);
            box(V(x1 - 0.06f, -0.55f, s - 0.1f), V(x1 + 0.0f, 0.55f, s + 0.75f), MT_SILVER);   // porch
        }
    }
}

static const Mat MT_ANOD = { 120, 118, 112, M_METAL };     // anodised panel
static const Mat MT_DARKGREY = { 60, 60, 64, M_BLACK };

// Quad helper with the vertices given as (x, y, z) floats.
#define Q(a,b,c,d,m) quad(V a, V b, V c, V d, m)

static void build_ascent_stage(void)
{
    const float xb = H0 + 3.15f;   // ascent / descent interface plane

    // --- Mid section: engine compartment, irregular octagon -----------------
    float mid[8][2] = { { -1.55f, -1.30f }, { -0.95f, -1.55f }, { 0.95f, -1.55f }, { 1.55f, -1.30f },
                        { 1.55f, 0.85f }, { 1.05f, 1.02f }, { -1.05f, 1.02f }, { -1.55f, 0.85f } };
    prism(mid, 8, xb, xb + 1.0f, MT_ANOD, MT_BLACK, MT_ANOD);
    // Black skirt band around the bottom.
    float midb[8][2];
    for (int i = 0; i < 8; i++) { midb[i][0] = mid[i][0] * 1.01f; midb[i][1] = mid[i][1] * 1.01f; }
    prism(midb, 8, xb, xb + 0.18f, MT_BLACK, MT_BLACK, MT_BLACK);

    // Propellant tank bulges on the left (oxidizer) and right (fuel).
    for (int side = -1; side <= 1; side += 2) {
        tube(V(xb + 0.50f, side * 1.40f, -0.85f), V(xb + 0.50f, side * 1.40f, 0.15f), 0.40f, 0.40f, 20, MT_BLACK, 0);
        tube(V(xb + 0.50f, side * 1.40f, 0.15f), V(xb + 0.50f, side * 1.40f, 0.32f), 0.40f, 0.18f, 20, MT_BLACK, 0);
        tube(V(xb + 0.50f, side * 1.40f, -0.85f), V(xb + 0.50f, side * 1.40f, -1.02f), 0.40f, 0.18f, 20, MT_BLACK, 0);
        tube(V(xb + 0.50f, side * 1.40f, -0.40f), V(xb + 0.50f, side * 1.40f, -0.34f), 0.41f, 0.41f, 20, MT_SILVER, 0);
    }

    // --- Crew compartment: X-Z profile extruded along Y, front narrowing upward.
    // Profile points (x, z), counter-clockwise seen from +Y.
    const float px[8] = { xb + 0.95f, xb + 1.60f, xb + 2.70f, xb + 2.98f, xb + 2.98f, xb + 2.70f, xb + 1.95f, xb + 0.95f };
    const float pz[8] = { 1.05f, 1.05f, 0.62f, 0.05f, -0.75f, -1.15f, -1.32f, -1.32f };
    // Half-width at each profile point (narrower at the top front: the "face").
    const float hw[8] = { 1.12f, 1.12f, 1.00f, 1.00f, 1.00f, 1.05f, 1.12f, 1.12f };
    const Mat face[8] = { MT_ANOD, MT_ANOD, MT_DARKGREY, MT_ANOD, MT_BLACK, MT_ANOD, MT_BLACK, MT_BLACK };
    for (int i = 0; i < 8; i++) {
        int j = (i + 1) % 8;
        if (i == 7) continue;   // bottom closed by the mid section
        // Main face between profile points i and j.
        Q((px[i], -hw[i], pz[i]), (px[i], hw[i], pz[i]), (px[j], hw[j], pz[j]), (px[j], -hw[j], pz[j]), face[i]);
    }
    // Side walls: fan from a centre point at each side.
    for (int side = -1; side <= 1; side += 2) {
        float cxp = xb + 1.9f, czp = -0.15f;
        for (int i = 0; i < 8; i++) {
            int j = (i + 1) % 8;
            Vector3 a = V(px[i], side * hw[i], pz[i]), b = V(px[j], side * hw[j], pz[j]), c = V(cxp, side * 1.12f, czp);
            Mat m = (i == 0 || i == 1) ? MT_BLACK : (i >= 5 ? MT_MYLAR : MT_ANOD);
            if (side > 0) tri(a, c, b, m); else tri(a, b, c, m);
        }
    }

    // Windows: two triangles on the slanted upper front face (profile 1 -> 2).
    for (int side = -1; side <= 1; side += 2) {
        // Must match the cockpit's window apertures (cockpit.c).
        float t0 = 0.0254f, t1 = 0.9314f;
        Vector3 lo_in = Vector3Lerp(V(px[1], side * 0.46f, pz[1]), V(px[2], side * 0.46f, pz[2]), t0);
        Vector3 lo_out = Vector3Lerp(V(px[1], side * 1.00f, pz[1]), V(px[2], side * 1.00f, pz[2]), t0);
        Vector3 hi_in = Vector3Lerp(V(px[1], side * 0.46f, pz[1]), V(px[2], side * 0.46f, pz[2]), t1);
        // Push slightly out of the face so they don't z-fight.
        Vector3 nrm = Vector3Normalize(V(pz[2] - pz[1], 0, -(px[2] - px[1])));
        if (nrm.z < 0) nrm = Vector3Negate(nrm);
        lo_in = Vector3Add(lo_in, Vector3Scale(nrm, 0.012f));
        lo_out = Vector3Add(lo_out, Vector3Scale(nrm, 0.012f));
        hi_in = Vector3Add(hi_in, Vector3Scale(nrm, 0.012f));
        tri(lo_in, lo_out, hi_in, MT_GLASS);   // double-sided glass
        tri(lo_in, hi_in, lo_out, MT_GLASS);
        tube(lo_in, lo_out, 0.03f, 0.03f, 6, MT_BLACK, 0);
        tube(lo_out, hi_in, 0.03f, 0.03f, 6, MT_BLACK, 0);
        tube(hi_in, lo_in, 0.03f, 0.03f, 6, MT_BLACK, 0);
    }
    // Forward hatch with frame, on the vertical lower front (profile 0 -> 1).
    box(V(xb + 1.00f, -0.42f, 1.05f), V(xb + 1.52f, 0.42f, 1.09f), MT_MYLAR);
    box(V(xb + 1.52f, -0.48f, 1.04f), V(xb + 1.58f, 0.48f, 1.11f), MT_BLACK);
    box(V(xb + 0.96f, -0.48f, 1.04f), V(xb + 1.00f, 0.48f, 1.11f), MT_BLACK);
    // Docking window (small, above the commander's side, on the top face).
    Q((xb + 2.99f, -0.80f, -0.05f), (xb + 2.99f, -0.52f, -0.05f), (xb + 2.99f, -0.52f, -0.35f), (xb + 2.99f, -0.80f, -0.35f), MT_GLASS);

    // --- Aft equipment bay ----------------------------------------------------------
    box(V(xb + 0.55f, -1.15f, -2.35f), V(xb + 2.35f, 1.15f, -1.32f), MT_MYLAR);
    box(V(xb + 0.60f, -1.05f, -2.40f), V(xb + 2.30f, 1.05f, -2.35f), MT_BLACK);
    // Radiator fins on the aft bay.
    for (int k = 0; k < 5; k++) {
        float y = -0.8f + k * 0.4f;
        box(V(xb + 0.7f, y - 0.02f, -2.48f), V(xb + 2.2f, y + 0.02f, -2.38f), MT_WHITE);
    }

    // --- Top: docking tunnel, drogue, ascent engine cover ---------------------------
    tube(V(xb + 2.96f, 0, -0.30f), V(xb + 3.40f, 0, -0.30f), 0.47f, 0.42f, 28, MT_ANOD, 1);
    tube(V(xb + 3.40f, 0, -0.30f), V(xb + 3.46f, 0, -0.30f), 0.50f, 0.50f, 28, MT_BLACK, 1);
    // Docking target on the top face.
    tube(V(xb + 2.98f, 0.60f, 0.10f), V(xb + 3.25f, 0.60f, 0.10f), 0.02f, 0.02f, 6, MT_WHITE, 1);
    box(V(xb + 3.25f, 0.48f, 0.08f), V(xb + 3.28f, 0.72f, 0.12f), MT_WHITE);
    // Overhead equipment: tracking light (above the face), EVA antenna.
    tube(V(xb + 2.85f, 0, 0.30f), V(xb + 2.95f, 0, 0.36f), 0.09f, 0.09f, 12, MT_WHITE, 1);

    // --- Rendezvous radar: dish on a short mast above the face ------------------------
    tube(V(xb + 2.95f, 0, 0.0f), V(xb + 3.30f, 0, 0.35f), 0.06f, 0.06f, 8, MT_BLACK, 1);
    box(V(xb + 3.22f, -0.12f, 0.28f), V(xb + 3.42f, 0.12f, 0.48f), MT_BLACK);
    dish(V(xb + 3.36f, 0, 0.55f), V(0.35f, 0, 1.0f), 0.31f, 0.09f, MT_GOLD, MT_BLACK);

    // --- S-band steerable antenna on its boom (upper right, aft) -----------------------
    tube(V(xb + 2.6f, 1.0f, -1.0f), V(xb + 3.25f, 1.55f, -1.35f), 0.035f, 0.035f, 6, MT_SILVER, 1);
    tube(V(xb + 3.25f, 1.55f, -1.35f), V(xb + 3.30f, 1.60f, -1.38f), 0.08f, 0.08f, 8, MT_BLACK, 1);
    dish(V(xb + 3.32f, 1.62f, -1.40f), V(0.65f, 0.35f, 0.35f), 0.43f, 0.12f, MT_WHITE, MT_GREY);
    // VHF antennas.
    tube(V(xb + 2.8f, -0.8f, -0.9f), V(xb + 3.6f, -1.15f, -1.25f), 0.012f, 0.007f, 5, MT_SILVER, 1);
    tube(V(xb + 1.9f, 0.9f, -1.9f), V(xb + 1.5f, 1.35f, -2.55f), 0.012f, 0.007f, 5, MT_SILVER, 1);

    // --- RCS quads: thruster clusters on booms, matching lm.c jet positions -------------
    for (int q = 1; q <= 4; q++) {
        float py = (q == 1 || q == 2) ? -1.19f : 1.19f;
        float pz = (q == 1 || q == 4) ? 1.19f : -1.19f;
        Vector3 P = V(H0 + 4.35f, py * 1.45f, pz * 1.45f);   // CG + 1.6 m
        tube(V(P.x, py * 0.85f, pz * 0.85f), P, 0.07f, 0.07f, 8, MT_BLACK, 0);
        box(Vector3Subtract(P, V(0.17f, 0.17f, 0.17f)), Vector3Add(P, V(0.17f, 0.17f, 0.17f)), MT_BLACK);
        Vector3 dirs[4] = { V(1, 0, 0), V(-1, 0, 0), V(0, 0, pz > 0 ? 1.0f : -1.0f), V(0, py > 0 ? 1.0f : -1.0f, 0) };
        for (int k = 0; k < 4; k++) {
            Vector3 a = Vector3Add(P, Vector3Scale(dirs[k], 0.17f));
            Vector3 b = Vector3Add(P, Vector3Scale(dirs[k], 0.42f));
            tube(a, b, 0.045f, 0.095f, 12, MT_DARKMETAL, 0);
        }
    }
}


// --- Extra detail: seams, bays, decals, lights, handrails ---------------------------

static const Mat MT_RED = { 178, 34, 52, M_WHITE }, MT_BLUE = { 40, 50, 120, M_WHITE }, MT_PAINTW = { 236, 236, 236, M_WHITE };

// Flat rectangle on a plane: centre c, unit axes u (width) and v (height), outward normal n.
static void plate(Vector3 c, Vector3 u, Vector3 v, Vector3 n, float w, float h, float off, Mat m)
{
    Vector3 o = Vector3Add(c, Vector3Scale(n, off));
    Vector3 a = Vector3Add(o, Vector3Add(Vector3Scale(u, -w / 2), Vector3Scale(v, -h / 2)));
    Vector3 b = Vector3Add(o, Vector3Add(Vector3Scale(u, w / 2), Vector3Scale(v, -h / 2)));
    Vector3 cc = Vector3Add(o, Vector3Add(Vector3Scale(u, w / 2), Vector3Scale(v, h / 2)));
    Vector3 d = Vector3Add(o, Vector3Add(Vector3Scale(u, -w / 2), Vector3Scale(v, h / 2)));
    // Wind so the face points along n.
    Vector3 fn = Vector3CrossProduct(Vector3Subtract(b, a), Vector3Subtract(cc, a));
    if (Vector3DotProduct(fn, n) > 0) quad(a, b, cc, d, m);
    else quad(a, d, cc, b, m);
}

static void build_details(void)
{
    const float s = 2.11f, c = 0.62f;
    const float x0 = H0 + 1.55f, x1 = H0 + 3.15f, xm = (x0 + x1) / 2, hx = (x1 - x0);
    float oct[8][2] = { { -s + c, -s }, { s - c, -s }, { s, -s + c }, { s, s - c },
                        { s - c, s }, { -s + c, s }, { -s, s - c }, { -s, -s + c } };
    Vector3 X = V(1, 0, 0);

    // Foil blanket seams (tape lines) on every descent-stage face.
    for (int i = 0; i < 8; i++) {
        int j = (i + 1) % 8;
        Vector3 pa = V(0, oct[i][0], oct[i][1]), pb = V(0, oct[j][0], oct[j][1]);
        Vector3 along = Vector3Normalize(Vector3Subtract(pb, pa));
        Vector3 n = Vector3Normalize(V(0, along.z, -along.y));
        if (Vector3DotProduct(n, Vector3Add(pa, pb)) < 0) n = Vector3Negate(n);
        float len = Vector3Distance(pa, pb);
        Vector3 mid = Vector3Add(Vector3Scale(Vector3Add(pa, pb), 0.5f), V(xm, 0, 0));
        for (int k = 1; k < 3; k++)
            plate(Vector3Add(mid, V(-hx / 2 + hx * k / 3, 0, 0)), along, X, n, len * 0.98f, 0.025f, 0.03f, MT_BLACK);
        int nv = len > 2 ? 3 : 1;
        for (int k = 1; k <= nv; k++)
            plate(Vector3Add(mid, Vector3Scale(along, -len / 2 + len * k / (nv + 1))), along, X, n, 0.025f, hx * 0.8f, 0.03f, MT_BLACK);
    }

    // MESA (equipment stowage) door left of the ladder, on the forward face.
    {
        Vector3 n = V(0, 0, 1), u = V(0, 1, 0);
        Vector3 cc = V(xm - 0.05f, -1.15f, s);
        plate(cc, u, X, n, 1.05f, 1.15f, 0.035f, MT_MYLAR);
        for (int e = 0; e < 4; e++) {
            float ox = (e < 2 ? (e ? 0.575f : -0.575f) : 0), oy = (e >= 2 ? (e == 3 ? 0.525f : -0.525f) : 0);
            plate(Vector3Add(cc, V(ox, oy, 0)), u, X, n, e < 2 ? 1.08f : 0.03f, e < 2 ? 0.03f : 1.18f, 0.045f, MT_BLACK);
        }
        tube(Vector3Add(cc, V(0.45f, -0.3f, 0.06f)), Vector3Add(cc, V(0.45f, 0.3f, 0.06f)), 0.015f, 0.015f, 6, MT_SILVER, 1);   // pull handle
        tube(Vector3Add(cc, V(-0.57f, -0.5f, 0.05f)), Vector3Add(cc, V(-0.57f, 0.5f, 0.05f)), 0.02f, 0.02f, 6, MT_SILVER, 1);  // hinge
    }
    // Scientific equipment bay (seismometer / laser reflector) on the aft face.
    {
        Vector3 n = V(0, 0, -1), u = V(0, 1, 0);
        Vector3 cc = V(xm - 0.05f, 0.0f, -s);
        plate(cc, u, X, n, 1.5f, 1.1f, 0.035f, MT_MYLAR);
        plate(Vector3Add(cc, V(0, 0, 0)), u, X, n, 0.03f, 1.1f, 0.045f, MT_BLACK);
        for (int k = -1; k <= 1; k += 2) {
            plate(Vector3Add(cc, V(0, k * 0.4f, 0)), u, X, n, 0.6f, 0.9f, 0.04f, (k > 0) ? MT_SILVER : MT_GREY);
            tube(Vector3Add(cc, V(0.2f, k * 0.4f - 0.15f, -0.06f)), Vector3Add(cc, V(0.2f, k * 0.4f + 0.15f, -0.06f)), 0.012f, 0.012f, 6, MT_SILVER, 1);
        }
    }
    // Landing radar antenna under the descent stage (aft of the engine).
    box(V(x0 - 0.12f, -0.55f, -1.45f), V(x0 - 0.01f, 0.25f, -0.75f), MT_GOLD);
    box(V(x0 - 0.14f, -0.48f, -1.38f), V(x0 - 0.11f, 0.18f, -0.82f), MT_DARKMETAL);
    // Ascent/descent interstage fittings.
    for (int k = 0; k < 4; k++) {
        float a = k * PI / 2 + PI / 4;
        Vector3 p = V(x1 + 0.02f, 1.25f * cosf(a), 1.25f * sinf(a));
        box(Vector3Subtract(p, V(0.02f, 0.09f, 0.09f)), Vector3Add(p, V(0.14f, 0.09f, 0.09f)), MT_SILVER);
    }

    // Flag decal and "the plaque" on the forward strut.
    {
        Vector3 n = V(0, 1, 0), u = V(0, 0, -1);
        Vector3 cc = V(x1 - 0.55f, s, 0.25f);
        float fw = 0.62f, fh = 0.40f;
        for (int k = 0; k < 13; k++)
            plate(Vector3Add(cc, V(fh / 2 - fh * (k + 0.5f) / 13, 0, 0)), u, X, n, fw, fh / 13 + 0.001f, 0.03f, (k & 1) ? MT_PAINTW : MT_RED);
        plate(Vector3Add(cc, V(fh / 2 - fh * 7 / 26, 0, fw / 2 - fw * 0.2f)), u, X, n, fw * 0.4f, fh * 7 / 13, 0.033f, MT_BLUE);
        for (int k = 0; k < 6; k++)
            for (int l = 0; l < 5; l++)
                plate(Vector3Add(cc, V(fh / 2 - 0.02f - l * 0.04f, 0, fw / 2 - 0.02f - k * 0.04f)), u, X, n, 0.012f, 0.012f, 0.036f, MT_PAINTW);
    }
    {
        Vector3 top = Vector3Add(V(x1 - 0.35f, 0, 0), V(0, 0, s + 0.15f));
        Vector3 knee = Vector3Add(V(H0 + 0.42f, 0, 0), V(0, 0, 4.30f));
        Vector3 p = Vector3Lerp(top, knee, 0.50f);
        Vector3 axis = Vector3Normalize(Vector3Subtract(knee, top));
        Vector3 n = Vector3Normalize(Vector3CrossProduct(V(0, 1, 0), axis));
        if (n.z < 0) n = Vector3Negate(n);
        plate(Vector3Add(p, Vector3Scale(n, 0.09f)), V(0, 1, 0), axis, n, 0.23f, 0.19f, 0.0f, MT_SILVER);
    }

    // Porch handrails up to the hatch.
    for (int side = -1; side <= 1; side += 2) {
        Vector3 a = V(x1 + 0.0f, side * 0.5f, s + 0.7f), b = V(x1 + 0.75f, side * 0.5f, s + 0.55f), cc = V(H0 + 3.15f + 1.45f, side * 0.5f, 1.15f);
        tube(a, b, 0.016f, 0.016f, 6, MT_SILVER, 1);
        tube(b, cc, 0.016f, 0.016f, 6, MT_SILVER, 1);
    }

    // Ascent stage: EVA handrails beside the hatch and on the top deck.
    const float xb = H0 + 3.15f;
    for (int side = -1; side <= 1; side += 2) {
        Vector3 a = V(xb + 0.98f, side * 0.62f, 1.11f), b = V(xb + 1.62f, side * 0.62f, 1.11f);
        tube(a, b, 0.014f, 0.014f, 6, MT_SILVER, 1);
        tube(a, V(a.x, a.y, 1.06f), 0.012f, 0.012f, 6, MT_SILVER, 1);
        tube(b, V(b.x, b.y, 1.06f), 0.012f, 0.012f, 6, MT_SILVER, 1);
        tube(V(xb + 3.02f, side * 0.75f, -1.0f), V(xb + 3.02f, side * 0.75f, -0.35f), 0.014f, 0.014f, 6, MT_SILVER, 1);
    }
    // Blanket quilting on the aft equipment bay.
    for (int k = 0; k < 4; k++)
        plate(V(xb + 0.75f + k * 0.42f, 0, -2.36f), V(0, 1, 0), X, V(0, 0, -1), 2.25f, 0.02f, 0.045f, MT_BLACK);

    // Docking/tracking lights: red port (-Y), green starboard (+Y), amber forward, white aft.
    box(V(xb + 2.35f, -1.14f, 0.20f), V(xb + 2.45f, -1.10f, 0.30f), (Mat){ 70, 6, 4, M_EMIT });
    box(V(xb + 2.35f, 1.10f, 0.20f), V(xb + 2.45f, 1.14f, 0.30f), (Mat){ 4, 60, 10, M_EMIT });
    box(V(xb + 2.92f, -0.05f, 0.40f), V(xb + 2.98f, 0.05f, 0.44f), (Mat){ 70, 45, 6, M_EMIT });
    box(V(xb + 2.10f, -0.05f, -2.42f), V(xb + 2.20f, 0.05f, -2.38f), (Mat){ 55, 55, 50, M_EMIT });

    // RCS quad thermal "doghouse" shields around each cluster.
    for (int q = 1; q <= 4; q++) {
        float py = (q == 1 || q == 2) ? -1.19f : 1.19f;
        float pz = (q == 1 || q == 4) ? 1.19f : -1.19f;
        Vector3 P = V(H0 + 4.35f, py * 1.45f, pz * 1.45f);
        Vector3 inward = Vector3Normalize(V(0, -py, -pz));
        box(Vector3Subtract(Vector3Add(P, Vector3Scale(inward, 0.30f)), V(0.24f, 0.12f, 0.12f)),
            Vector3Add(Vector3Add(P, Vector3Scale(inward, 0.30f)), V(0.24f, 0.12f, 0.12f)), MT_MYLAR);
    }
}

// --- Public ---------------------------------------------------------------------

static Mesh lm_mesh;
static Material lm_mat, depth_mat, fx_mat;

void lmmodel_init(void)
{
    B.count = 0;
    build_descent_stage();
    build_ascent_stage();
    build_details();
    Mesh m = { 0 };
    m.vertexCount = B.count;
    m.triangleCount = B.count / 3;
    m.vertices = B.v;
    m.normals = B.n;
    m.colors = B.c;
    UploadMesh(&m, false);
    lm_mesh = m;
    lm_mat = LoadMaterialDefault();
    lm_mat.shader = *render_lm_shader();
    depth_mat = LoadMaterialDefault();
    depth_mat.shader = *render_depth_shader();
    fx_mat = LoadMaterialDefault();
    fx_mat.shader = *render_fx_shader();
    TraceLog(LOG_INFO, "LM model: %d triangles", m.triangleCount);
}

int lmmodel_triangles(void) { return lm_mesh.triangleCount; }

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

void lmmodel_draw(M3 site_from_body, V3 pos_cam_rel)
{
    DrawMesh(lm_mesh, lm_mat, to_matrix(site_from_body, pos_cam_rel));
}

void lmmodel_draw_depth(M3 site_from_body, V3 pos_cam_rel)
{
    DrawMesh(lm_mesh, depth_mat, to_matrix(site_from_body, pos_cam_rel));
}

// --- Effects -------------------------------------------------------------------------

static Vector3 W(M3 r, V3 t, V3 b)
{
    V3 p = v3_add(m3_apply(r, b), t);
    return (Vector3){ (float)p.x, (float)p.y, (float)p.z };
}

// Camera-facing quad strip from a to b (width w), uv.y 0 at a.
static void beam(Vector3 a, Vector3 b, float wa, float wb, Color c)
{
    Vector3 ax = Vector3Subtract(b, a);
    Vector3 mid = Vector3Scale(Vector3Add(a, b), 0.5f);
    Vector3 side = Vector3Normalize(Vector3CrossProduct(ax, mid));   // camera at origin
    Vector3 a0 = Vector3Add(a, Vector3Scale(side, -wa)), a1 = Vector3Add(a, Vector3Scale(side, wa));
    Vector3 b0 = Vector3Add(b, Vector3Scale(side, -wb)), b1 = Vector3Add(b, Vector3Scale(side, wb));
    rlColor4ub(c.r, c.g, c.b, c.a);
    rlTexCoord2f(0, 0); rlVertex3f(a0.x, a0.y, a0.z);
    rlTexCoord2f(1, 0); rlVertex3f(a1.x, a1.y, a1.z);
    rlTexCoord2f(1, 1); rlVertex3f(b1.x, b1.y, b1.z);
    rlTexCoord2f(0, 1); rlVertex3f(b0.x, b0.y, b0.z);
}

static void puff(Vector3 c, float size, Color col)
{
    // Billboard facing the camera at the origin.
    Vector3 f = Vector3Normalize(c);
    Vector3 u = fabsf(f.y) < 0.95f ? V(0, 1, 0) : V(1, 0, 0);
    Vector3 r = Vector3Normalize(Vector3CrossProduct(f, u));
    u = Vector3CrossProduct(r, f);
    Vector3 a = Vector3Add(c, Vector3Add(Vector3Scale(r, -size), Vector3Scale(u, -size)));
    Vector3 b = Vector3Add(c, Vector3Add(Vector3Scale(r, size), Vector3Scale(u, -size)));
    Vector3 d = Vector3Add(c, Vector3Add(Vector3Scale(r, size), Vector3Scale(u, size)));
    Vector3 e = Vector3Add(c, Vector3Add(Vector3Scale(r, -size), Vector3Scale(u, size)));
    rlColor4ub(col.r, col.g, col.b, col.a);
    rlTexCoord2f(0, 0); rlVertex3f(a.x, a.y, a.z);
    rlTexCoord2f(1, 0); rlVertex3f(b.x, b.y, b.z);
    rlTexCoord2f(1, 1); rlVertex3f(d.x, d.y, d.z);
    rlTexCoord2f(0, 1); rlVertex3f(e.x, e.y, e.z);
}

static void set_kind(int k)
{
    Shader s = *render_fx_shader();
    SetShaderValue(s, GetShaderLocation(s, "kind"), &k, SHADER_UNIFORM_INT);
}

// --- Blown dust particles ----------------------------------------------------
// Particles live in the local ground plane around the point under the engine:
// radius r (m), azimuth th, height y; they stream outward at high speed.
#define NDUST 1800
typedef struct { float r, th, y, v, len; } Dust;
static Dust dust[NDUST];
static int dust_init;

static float frand(void) { return (float)rand() / (float)RAND_MAX; }

static void dust_spawn(Dust *d, float intensity)
{
    d->r = 2.0f + frand() * 6.0f;
    d->th = frand() * 2 * PI;
    d->y = 0.05f + frand() * frand() * 1.6f;
    d->v = (35.0f + frand() * 90.0f) * (0.4f + 0.6f * intensity);
    d->len = 1.5f + frand() * 4.5f;
}

void lmmodel_update_dust(float dt, float intensity)
{
    if (!dust_init) {
        for (int i = 0; i < NDUST; i++) { dust_spawn(&dust[i], 1); dust[i].r = 2 + frand() * 120; }
        dust_init = 1;
    }
    float rmax = 40.0f + 110.0f * intensity;
    for (int i = 0; i < NDUST; i++) {
        dust[i].r += dust[i].v * dt;
        if (dust[i].r > rmax * (0.6f + 0.4f * frand())) dust_spawn(&dust[i], intensity);
    }
}

static void draw_dust(const LmFx *fx)
{
    Vector3 c = { (float)fx->ground_point.x, (float)fx->ground_point.y, (float)fx->ground_point.z };
    Vector3 n = { (float)fx->ground_up.x, (float)fx->ground_up.y, (float)fx->ground_up.z };
    Vector3 t1 = Vector3Normalize(Vector3CrossProduct(n, V(0, 0, 1)));
    Vector3 t2 = Vector3CrossProduct(n, t1);
    int count = (int)(NDUST * fminf(1.0f, fx->dust * 1.3f));
    float rmax = 40.0f + 110.0f * fx->dust;
    for (int i = 0; i < count; i++) {
        Dust *d = &dust[i];
        Vector3 dir = Vector3Add(Vector3Scale(t1, cosf(d->th)), Vector3Scale(t2, sinf(d->th)));
        Vector3 a = Vector3Add(c, Vector3Add(Vector3Scale(dir, d->r), Vector3Scale(n, d->y)));
        Vector3 b = Vector3Add(a, Vector3Scale(dir, d->len));
        float fade = 1.0f - d->r / rmax;
        if (fade <= 0) continue;
        unsigned char al = (unsigned char)(255 * fminf(1.0f, fx->dust * 0.55f) * fade * (d->r < 6 ? d->r / 6 : 1));
        beam(a, b, 0.10f + 0.006f * d->r, 0.12f + 0.008f * d->r, (Color){ 120, 116, 108, al });
    }
}


// --- Crash debris ---------------------------------------------------------------
// Ballistic fragments (foil, panels, strut pieces) in the local ground frame
// around the point under the LM; they fall at lunar g and rest on the ground.
#define NDEBRIS 260
typedef struct { float x, y, z, vx, vy, vz, size; unsigned char kind; } Debris;
static Debris debris[NDEBRIS];
static float debris_seed_age = -1;

static void debris_spawn(float speed)
{
    float k = fminf(2.2f, 0.5f + speed / 5.0f);
    for (int i = 0; i < NDEBRIS; i++) {
        Debris *d = &debris[i];
        float th = frand() * 2 * PI, el = 0.15f + frand() * 1.2f;
        float v = (2.0f + frand() * frand() * 14.0f) * k;
        d->x = (frand() - 0.5f) * 3.0f;
        d->z = (frand() - 0.5f) * 3.0f;
        d->y = 0.5f + frand() * 2.5f;
        d->vx = cosf(th) * cosf(el) * v;
        d->vz = sinf(th) * cosf(el) * v;
        d->vy = sinf(el) * v;
        d->size = 0.03f + frand() * frand() * frand() * 0.22f;
        d->kind = (unsigned char)(frand() < 0.45f ? 0 : frand() < 0.6f ? 1 : 2);   // gold foil, grey, black
    }
}

static void draw_crash(const LmFx *fx, Vector3 t1, Vector3 t2, Vector3 n, Vector3 c)
{
    float age = fx->crash_age;
    if (debris_seed_age < 0 || age < debris_seed_age) debris_spawn(fx->crash_speed);
    debris_seed_age = age;
    for (int i = 0; i < NDEBRIS; i++) {
        Debris *d = &debris[i];
        // Flight time until it lands: y + vy t - g/2 t^2 = 0.
        float g = 1.62f;
        float tl = (d->vy + sqrtf(d->vy * d->vy + 2 * g * d->y)) / g;
        float t = fminf(age, tl);
        float x = d->x + d->vx * t, z = d->z + d->vz * t;
        float y = fmaxf(0.03f, d->y + d->vy * t - 0.5f * g * t * t);
        Vector3 p = Vector3Add(c, Vector3Add(Vector3Add(Vector3Scale(t1, x), Vector3Scale(t2, z)), Vector3Scale(n, y)));
        Color col = d->kind == 0 ? (Color){ 214, 168, 70, 255 } : d->kind == 1 ? (Color){ 150, 150, 152, 255 } : (Color){ 40, 40, 42, 255 };
        // Foil flickers as it tumbles in the sun.
        if (d->kind == 0 && age < tl) {
            float f = 0.6f + 0.4f * sinf(age * (6.0f + i % 7) + i);
            col.r = (unsigned char)(col.r * f); col.g = (unsigned char)(col.g * f); col.b = (unsigned char)(col.b * f);
        }
        puff(p, d->size, col);
    }
}

static void draw_fireball(const LmFx *fx, Vector3 c, Vector3 n)
{
    float age = fx->crash_age;
    if (!fx->crash_fire || age > 2.6f) return;
    float k = fminf(1.0f, fx->crash_speed / 8.0f + 0.4f);
    for (int i = 0; i < 14; i++) {
        float a = i * 2.39996f;
        float r = (1.2f + 6.5f * k) * sqrtf(age) * (0.3f + 0.7f * ((i * 37) % 11) / 10.0f);
        Vector3 o = { cosf(a) * r, 0, sinf(a) * r };
        Vector3 p = Vector3Add(Vector3Add(c, o), Vector3Scale(n, 1.5f + age * 2.2f * (0.4f + (i % 5) * 0.2f)));
        float fade = fmaxf(0.0f, 1.0f - age / 2.6f);
        unsigned char al = (unsigned char)(220 * fade * fade);
        puff(p, (2.0f + 6.0f * k) * (0.5f + age), (Color){ 255, (unsigned char)(150 + 60 * fade), (unsigned char)(70 * fade), al });
    }
    // White flash in the first instant.
    if (age < 0.35f)
        puff(Vector3Add(c, Vector3Scale(n, 2.0f)), 14.0f * k, (Color){ 255, 245, 220, (unsigned char)(255 * (1 - age / 0.35f)) });
}

void lmmodel_draw_effects(M3 R, V3 T, const LmFx *fx)
{
    Shader s = *render_fx_shader();
    rlDrawRenderBatchActive();
    BeginShaderMode(s);
    BeginBlendMode(BLEND_ADD_COLORS);   // pure additive: GL_ONE, GL_ONE
    rlDisableDepthMask();
    rlDisableBackfaceCulling();

    // DPS plume: hot throat glow plus a faint, flickering exhaust cone.
    if (fx->thrust > 100) {
        float k = fx->thrust / 43455.0f;
        set_kind(0);
        rlBegin(RL_QUADS);
        Vector3 throat = W(R, T, v3(H0 + 1.55 - 0.5, 0, 0));
        puff(throat, 0.9f, (Color){ 255, 190, 120, (unsigned char)(70 + 120 * k) });
        Vector3 exitp = W(R, T, v3(H0 + 1.55 - DPS_BELL_LEN - 0.03, 0, 0));
        puff(exitp, 1.3f, (Color){ 255, 160, 90, (unsigned char)(40 + 80 * k) });
        rlEnd();
        rlDrawRenderBatchActive();
        set_kind(1);
        rlBegin(RL_QUADS);
        float len = 6.0f + 18.0f * k;
        Vector3 a = W(R, T, v3(H0 + 1.55 - DPS_BELL_LEN, 0, 0));
        Vector3 b = W(R, T, v3(H0 + 1.55 - DPS_BELL_LEN - len, 0, 0));
        beam(a, b, 0.78f, 2.6f + 4.0f * k, (Color){ 255, 175, 115, (unsigned char)(20 + 45 * k) });
        rlEnd();
        rlDrawRenderBatchActive();
    }

    // RCS: short bright puffs from firing jets.
    set_kind(0);
    rlBegin(RL_QUADS);
    for (int j = 0; j < LM_NJETS; j++) {
        if (fx->jet[j] < 0.02f) continue;
        V3 p = fx->jet_pos[j], d = fx->jet_dir[j];
        // Exhaust leaves opposite to the thrust direction.
        for (int i = 0; i < 4; i++) {
            V3 q = v3_add(p, v3_scale(d, -(0.45 + 0.55 * i)));
            Vector3 w = W(R, T, q);
            // RCS exhaust is nearly invisible in vacuum: faint, brief wisps.
            puff(w, 0.18f + 0.30f * i, (Color){ 210, 220, 240, (unsigned char)(fx->jet[j] * (34 - 7 * i)) });
        }
    }
    rlEnd();
    rlDrawRenderBatchActive();

    // Streaming dust particles (alpha blended so they veil the surface).
    if (fx->dust > 0.01f) {
        EndBlendMode();
        BeginBlendMode(BLEND_ALPHA);
        set_kind(3);
        rlBegin(RL_QUADS);
        draw_dust(fx);
        rlEnd();
        rlDrawRenderBatchActive();
        EndBlendMode();
        BeginBlendMode(BLEND_ADD_COLORS);
    }

    // Blown dust: radial sheet hugging the ground under the engine.
    if (fx->dust > 0.01f) {
        set_kind(2);
        rlBegin(RL_QUADS);
        Vector3 c = { (float)fx->ground_point.x, (float)fx->ground_point.y + 0.25f, (float)fx->ground_point.z };
        Vector3 n = { (float)fx->ground_up.x, (float)fx->ground_up.y, (float)fx->ground_up.z };
        Vector3 t1 = Vector3Normalize(Vector3CrossProduct(n, V(0, 0, 1)));
        Vector3 t2 = Vector3CrossProduct(n, t1);
        for (int layer = 0; layer < 3; layer++) {
            float rad = 25.0f + 45.0f * fx->dust + layer * 18.0f;
            Vector3 cc = Vector3Add(c, Vector3Scale(n, 0.4f + layer * 0.5f));
            unsigned char a = (unsigned char)(fx->dust * (120 - layer * 30));
            rlColor4ub(170, 165, 155, a);
            Vector3 p0 = Vector3Add(cc, Vector3Add(Vector3Scale(t1, -rad), Vector3Scale(t2, -rad)));
            Vector3 p1 = Vector3Add(cc, Vector3Add(Vector3Scale(t1, rad), Vector3Scale(t2, -rad)));
            Vector3 p2 = Vector3Add(cc, Vector3Add(Vector3Scale(t1, rad), Vector3Scale(t2, rad)));
            Vector3 p3 = Vector3Add(cc, Vector3Add(Vector3Scale(t1, -rad), Vector3Scale(t2, rad)));
            rlTexCoord2f(0, 0); rlVertex3f(p0.x, p0.y, p0.z);
            rlTexCoord2f(1, 0); rlVertex3f(p1.x, p1.y, p1.z);
            rlTexCoord2f(1, 1); rlVertex3f(p2.x, p2.y, p2.z);
            rlTexCoord2f(0, 1); rlVertex3f(p3.x, p3.y, p3.z);
        }
        rlEnd();
        rlDrawRenderBatchActive();
    }

    // Crash: fireball (additive) then debris (alpha).
    if (fx->crash_age >= 0) {
        Vector3 c = { (float)fx->ground_point.x, (float)fx->ground_point.y, (float)fx->ground_point.z };
        Vector3 n = { (float)fx->ground_up.x, (float)fx->ground_up.y, (float)fx->ground_up.z };
        Vector3 t1 = Vector3Normalize(Vector3CrossProduct(n, V(0, 0, 1)));
        Vector3 t2 = Vector3CrossProduct(n, t1);
        set_kind(0);
        rlBegin(RL_QUADS);
        draw_fireball(fx, c, n);
        rlEnd();
        rlDrawRenderBatchActive();
        EndBlendMode();
        BeginBlendMode(BLEND_ALPHA);
        rlEnableDepthMask();
        set_kind(4);
        rlBegin(RL_QUADS);
        draw_crash(fx, t1, t2, n, c);
        rlEnd();
        rlDrawRenderBatchActive();
    } else {
        debris_seed_age = -1;
    }

    rlEnableBackfaceCulling();
    rlEnableDepthMask();
    EndBlendMode();
    EndShaderMode();
}
