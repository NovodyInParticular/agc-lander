// Lunar orbit (globe) view of the whole Moon, plus two things nobody put in
// the flight plan: a visitor that very rarely crosses the sky, and something
// on the far side, deep in a permanently shadowed polar crater.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "hud.h"
#include "moon.h"
#include "render.h"
#include "sky_extras.h"
#include "terrain.h"

#define DEG (PI_D / 180.0)

// The far-side site: lat/lon in degrees (moon-fixed, as the AGC's frame).
#define BASE_LAT (-84.6)
#define BASE_LON 168.2
#define BASE_CRATER_R 7000.0
#define G_NLON 255
#define G_NLAT 128

// --- Mesh builder for the LM shader (vertex alpha = material) -------------------------
// Materials: 1 gold foil, 2 metal, 3 black paint, 4 white paint, 5 mylar, 6 glass, 7 emissive.

typedef struct { float *p, *n; unsigned char *c; int count, cap; } Mb;
static Mb mb;
static M3 xf_r;     // current transform for added geometry
static V3 xf_t;

static void mb_push(V3 p, V3 n, Color c, int m)
{
    if (mb.count == mb.cap) {
        mb.cap = mb.cap ? mb.cap * 2 : 4096;
        mb.p = realloc(mb.p, sizeof(float) * 3 * mb.cap);
        mb.n = realloc(mb.n, sizeof(float) * 3 * mb.cap);
        mb.c = realloc(mb.c, 4 * mb.cap);
    }
    p = v3_add(m3_apply(xf_r, p), xf_t);
    n = m3_apply(xf_r, n);
    int k = mb.count++;
    mb.p[3 * k] = (float)p.x; mb.p[3 * k + 1] = (float)p.y; mb.p[3 * k + 2] = (float)p.z;
    mb.n[3 * k] = (float)n.x; mb.n[3 * k + 1] = (float)n.y; mb.n[3 * k + 2] = (float)n.z;
    mb.c[4 * k] = c.r; mb.c[4 * k + 1] = c.g; mb.c[4 * k + 2] = c.b; mb.c[4 * k + 3] = (unsigned char)m;
}

static void mb_tri(V3 a, V3 b, V3 c, Color col, int m)
{
    V3 n = v3_norm(v3_cross(v3_sub(b, a), v3_sub(c, a)));
    mb_push(a, n, col, m); mb_push(b, n, col, m); mb_push(c, n, col, m);
}

static void mb_quad(V3 a, V3 b, V3 c, V3 d, Color col, int m) { mb_tri(a, b, c, col, m); mb_tri(a, c, d, col, m); }

static void mb_box(V3 lo, V3 hi, Color col, int m)
{
    V3 p[8];
    for (int i = 0; i < 8; i++) p[i] = v3(i & 1 ? hi.x : lo.x, i & 2 ? hi.y : lo.y, i & 4 ? hi.z : lo.z);
    mb_quad(p[0], p[2], p[3], p[1], col, m); mb_quad(p[4], p[5], p[7], p[6], col, m);
    mb_quad(p[0], p[1], p[5], p[4], col, m); mb_quad(p[2], p[6], p[7], p[3], col, m);
    mb_quad(p[0], p[4], p[6], p[2], col, m); mb_quad(p[1], p[3], p[7], p[5], col, m);
}

// Surface of revolution about local +Y: profile (r, y) points, centre c.
static void mb_lathe(V3 c, const double (*pr)[2], int n, int seg, Color col, int m)
{
    for (int k = 0; k + 1 < n; k++)
        for (int i = 0; i < seg; i++) {
            double a0 = 2 * PI_D * i / seg, a1 = 2 * PI_D * (i + 1) / seg;
            double r0 = pr[k][0], y0 = pr[k][1], r1 = pr[k + 1][0], y1 = pr[k + 1][1];
            V3 p00 = v3_add(c, v3(r0 * cos(a0), y0, r0 * sin(a0))), p01 = v3_add(c, v3(r0 * cos(a1), y0, r0 * sin(a1)));
            V3 p10 = v3_add(c, v3(r1 * cos(a0), y1, r1 * sin(a0))), p11 = v3_add(c, v3(r1 * cos(a1), y1, r1 * sin(a1)));
            // Smooth normal from the profile slope.
            double dr = r1 - r0, dy = y1 - y0, l = sqrt(dr * dr + dy * dy) + 1e-9;
            double nr = dy / l, ny = -dr / l;
            V3 n0 = v3(nr * cos(a0), ny, nr * sin(a0)), n1 = v3(nr * cos(a1), ny, nr * sin(a1));
            mb_push(p00, n0, col, m); mb_push(p11, n1, col, m); mb_push(p10, n0, col, m);
            mb_push(p00, n0, col, m); mb_push(p01, n1, col, m); mb_push(p11, n1, col, m);
        }
}

static void mb_tube(V3 a, V3 b, double r, int seg, Color col, int m)
{
    V3 ax = v3_norm(v3_sub(b, a));
    V3 t = fabs(ax.x) < 0.9 ? v3(1, 0, 0) : v3(0, 1, 0);
    V3 u = v3_norm(v3_cross(ax, t)), w = v3_cross(ax, u);
    for (int i = 0; i < seg; i++) {
        double t0 = 2 * PI_D * i / seg, t1 = 2 * PI_D * (i + 1) / seg;
        V3 d0 = v3_add(v3_scale(u, cos(t0)), v3_scale(w, sin(t0))), d1 = v3_add(v3_scale(u, cos(t1)), v3_scale(w, sin(t1)));
        V3 a0 = v3_add(a, v3_scale(d0, r)), a1 = v3_add(a, v3_scale(d1, r)), b0 = v3_add(b, v3_scale(d0, r)), b1 = v3_add(b, v3_scale(d1, r));
        mb_push(a0, d0, col, m); mb_push(b1, d1, col, m); mb_push(a1, d1, col, m);
        mb_push(a0, d0, col, m); mb_push(b0, d0, col, m); mb_push(b1, d1, col, m);
    }
}

static Mesh mb_upload(void)
{
    Mesh m = { 0 };
    m.vertexCount = mb.count;
    m.triangleCount = mb.count / 3;
    m.vertices = mb.p;
    m.normals = mb.n;
    m.colors = mb.c;
    UploadMesh(&m, false);
    memset(&mb, 0, sizeof mb);
    return m;
}


// A 1945-style secret installation: concrete flak-tower keep, riveted
// steel hangar, a launch ramp with a disc craft, fuel spheres, a lattice
// radio mast and searchlights. (Original design; no insignia.)
static const Color K_BLACK_TRACK = { 30, 30, 30, 255 };
static const Color K_CONC = { 150, 146, 138, 255 }, K_CONC_D = { 112, 108, 102, 255 }, K_STEEL = { 92, 96, 90, 255 },
                   K_RUST = { 120, 82, 58, 255 }, K_LAMP = { 120, 78, 30, 255 };

static void slit_row(V3 a, V3 dir, V3 out, int n, double step, double w, double h)
{
    // Emissive window slits along a wall (a: first slit centre, dir: along wall, out: wall normal).
    V3 up = v3(0, 1, 0);
    for (int i = 0; i < n; i++) {
        V3 c = v3_add(v3_add(a, v3_scale(dir, step * i)), v3_scale(out, 0.06));
        V3 p0 = v3_sub(v3_sub(c, v3_scale(dir, w / 2)), v3_scale(up, h / 2));
        V3 p1 = v3_add(v3_sub(c, v3_scale(up, h / 2)), v3_scale(dir, w / 2));
        V3 p2 = v3_add(v3_add(c, v3_scale(up, h / 2)), v3_scale(dir, w / 2));
        V3 p3 = v3_sub(v3_add(c, v3_scale(up, h / 2)), v3_scale(dir, w / 2));
        mb_quad(p0, p1, p2, p3, K_LAMP, 7);
        mb_quad(p0, p3, p2, p1, K_LAMP, 7);
    }
}

static void build_disc(Color hull_col)
{
    // Riveted disc craft: flat saucer, stepped cupola, three landing spheres.
    const double hull[][2] = { { 0.0, -0.8 }, { 3.0, -0.75 }, { 6.0, -0.45 }, { 8.0, -0.08 }, { 8.15, 0.0 },
                               { 7.9, 0.15 }, { 5.5, 0.55 }, { 3.2, 0.9 }, { 0.0, 0.95 } };
    mb_lathe(v3(0, 0, 0), hull, 9, 48, hull_col, 2);
    const double cup[][2] = { { 3.1, 0.9 }, { 3.0, 2.0 }, { 2.4, 2.1 }, { 2.3, 3.0 }, { 1.4, 3.6 }, { 0.0, 3.8 } };
    mb_lathe(v3(0, 0, 0), cup, 6, 32, hull_col, 2);
    const double win[][2] = { { 3.02, 1.3 }, { 3.0, 1.7 } };
    mb_lathe(v3(0, 0, 0), win, 2, 32, K_LAMP, 7);
    for (int k = 0; k < 24; k++) {   // rivet bands
        double a0 = k * 2 * PI_D / 24;
        V3 c = v3(7.0 * cos(a0), 0.36, 7.0 * sin(a0));
        mb_box(v3_sub(c, v3(0.12, 0.05, 0.12)), v3_add(c, v3(0.12, 0.12, 0.12)), K_STEEL, 2);
    }
    const double ball[][2] = { { 0.0, -1.0 }, { 0.7, -0.7 }, { 1.0, 0.0 }, { 0.7, 0.7 }, { 0.0, 1.0 } };
    for (int k = 0; k < 3; k++) {
        double a0 = k * 2 * PI_D / 3 + 0.5;
        mb_lathe(v3(3.6 * cos(a0), -1.6, 3.6 * sin(a0)), ball, 5, 16, hull_col, 2);
    }
    const double belly[][2] = { { 1.2, -0.82 }, { 0.0, -0.86 } };
    mb_lathe(v3(0, 0, 0), belly, 2, 20, (Color){ 140, 70, 20, 255 }, 7);
}

static void build_saucer(void) { build_disc((Color){ 150, 154, 150, 255 }); }

static void build_base(void)
{
    // --- Keep: two-tier concrete flak tower with corner towers -----------------
    mb_box(v3(-18, 0, -18), v3(18, 20, 18), K_CONC, 4);
    mb_box(v3(-12, 20, -12), v3(12, 28, 12), K_CONC_D, 4);
    mb_box(v3(-19, 19.5, -19), v3(19, 20.6, 19), K_CONC_D, 4);   // gallery ledge
    const double tw[][2] = { { 5.2, 0 }, { 5.2, 25 }, { 6.0, 25 }, { 6.0, 26.2 }, { 0.0, 26.2 } };
    const double cup[][2] = { { 3.0, 26.2 }, { 2.8, 27.5 }, { 2.0, 28.4 }, { 0.0, 28.8 } };
    for (int i = 0; i < 4; i++) {
        V3 c = v3(i & 1 ? 18 : -18, 0, i & 2 ? 18 : -18);
        mb_lathe(c, tw, 5, 24, K_CONC, 4);
        mb_lathe(c, cup, 4, 16, K_STEEL, 2);
        const double ring[][2] = { { 5.25, 15 }, { 5.25, 16.2 } };
        mb_lathe(c, ring, 2, 24, K_LAMP, 7);
    }
    for (int f = 0; f < 4; f++) {   // window slits on every face, two floors
        V3 dir = f == 0 ? v3(1, 0, 0) : f == 1 ? v3(0, 0, 1) : f == 2 ? v3(-1, 0, 0) : v3(0, 0, -1);
        V3 out = v3_cross(dir, v3(0, 1, 0));
        V3 start = v3_add(v3_scale(out, 18), v3_scale(dir, -10));
        slit_row(v3_add(start, v3(0, 7, 0)), dir, out, 6, 4.0, 1.6, 0.5);
        slit_row(v3_add(start, v3(0, 13, 0)), dir, out, 6, 4.0, 1.6, 0.5);
    }
    mb_box(v3(17.5, 0, -3), v3(24, 6, 3), K_CONC_D, 4);            // blast-door entrance
    mb_box(v3(23.9, 0.4, -2.2), v3(24.05, 5.2, 2.2), K_LAMP, 7);

    // --- Riveted steel hangar (Quonset half-cylinder) with lit open door -----------
    V3 h0 = v3(56, 0, -40);
    const double R = 14, L = 60;
    for (int i = 0; i < 24; i++) {
        double a0 = PI_D * i / 24, a1 = PI_D * (i + 1) / 24;
        V3 p0 = v3_add(h0, v3(R * cos(a0), R * sin(a0), 0)), p1 = v3_add(h0, v3(R * cos(a1), R * sin(a1), 0));
        V3 q0 = v3_add(p0, v3(0, 0, L)), q1 = v3_add(p1, v3(0, 0, L));
        mb_quad(p0, q0, q1, p1, i % 2 ? K_STEEL : (Color){ 84, 88, 82, 255 }, 2);
        // Back wall.
        mb_tri(h0, p1, p0, K_STEEL, 2);
    }
    for (int k = 0; k <= 10; k++) {   // rib bands
        double z = -40 + k * 6;
        const double rib[][2] = { { R + 0.25, 0 }, { R + 0.25, 0.6 } };
        (void)rib;
        for (int i = 0; i < 24; i++) {
            double a0 = PI_D * i / 24, a1 = PI_D * (i + 1) / 24;
            V3 p0 = v3(56 + (R + 0.2) * cos(a0), (R + 0.2) * sin(a0), z), p1 = v3(56 + (R + 0.2) * cos(a1), (R + 0.2) * sin(a1), z);
            mb_quad(p0, p1, v3_add(p1, v3(0, 0, 0.5)), v3_add(p0, v3(0, 0, 0.5)), K_RUST, 2);
        }
    }
    // Front wall around the door opening (door 18 m wide, 11 m tall, lit inside).
    V3 fz = v3(0, 0, 20);
    for (int i = 0; i < 24; i++) {
        double a0 = PI_D * i / 24, a1 = PI_D * (i + 1) / 24;
        V3 p0 = v3_add(v3(56 + R * cos(a0), R * sin(a0), 0), fz), p1 = v3_add(v3(56 + R * cos(a1), R * sin(a1), 0), fz);
        V3 c = v3(56, 11, 20);
        mb_tri(c, p0, p1, K_STEEL, 2);
    }
    mb_box(v3(47, 0, 19.9), v3(65, 11, 20.2), (Color){ 150, 100, 45, 255 }, 7);
    mb_box(v3(30, 0, 18), v3(47, 12, 21), K_STEEL, 2);   // slid-open door leaves
    mb_box(v3(65, 0, 18), v3(82, 12, 21), K_STEEL, 2);

    // --- Launch ramp out of the hangar, with the disc craft ready on it ------------
    V3 r0 = v3(56, 0, 21), r1 = v3(56, 24, 96);
    V3 rd = v3_norm(v3_sub(r1, r0)), side = v3(1, 0, 0);
    V3 rn = v3_cross(rd, side);
    for (int s2 = -1; s2 <= 1; s2 += 2) {
        V3 a = v3_add(r0, v3_scale(side, s2 * 4.0)), b = v3_add(r1, v3_scale(side, s2 * 4.0));
        mb_tube(a, b, 0.35, 8, K_RUST, 2);
    }
    mb_quad(v3_add(r0, v3(-5, -0.3, 0)), v3_add(r0, v3(5, -0.3, 0)), v3_add(r1, v3(5, -0.3, 0)), v3_add(r1, v3(-5, -0.3, 0)), K_CONC_D, 4);
    for (int k = 1; k < 8; k++) {
        V3 p = v3_add(r0, v3_scale(v3_sub(r1, r0), k / 8.0));
        mb_box(v3(p.x - 4.5, 0, p.z - 0.6), v3(p.x + 4.5, p.y - 0.2, p.z + 0.6), K_STEEL, 2);   // pylons
    }
    {
        M3 save = xf_r;
        V3 savet = xf_t;
        M3 tilt = m3_rows(side, rn, v3_scale(rd, 1));
        xf_r = m3_mul(save, m3_transpose(tilt));
        xf_r = m3_mul(save, m3_rows(v3(side.x, rn.x, rd.x), v3(side.y, rn.y, rd.y), v3(side.z, rn.z, rd.z)));
        V3 pos = v3_add(v3_add(r0, v3_scale(v3_sub(r1, r0), 0.82)), v3_scale(rn, 2.6));
        xf_t = v3_add(m3_apply(save, pos), savet);
        build_disc((Color){ 70, 74, 70, 255 });
        xf_r = save;
        xf_t = savet;
    }

    // --- Fuel spheres on legs ------------------------------------------------------
    const double sph[][2] = { { 0.0, -6 }, { 3.0, -5.2 }, { 5.2, -3 }, { 6.0, 0 }, { 5.2, 3 }, { 3.0, 5.2 }, { 0.0, 6 } };
    for (int k = 0; k < 2; k++) {
        V3 c = v3(-44, 10, -12 - k * 16);
        mb_lathe(c, sph, 7, 24, k ? K_RUST : K_CONC, 2);
        for (int l = 0; l < 4; l++) {
            double a0 = l * PI_D / 2 + PI_D / 4;
            mb_tube(v3_add(c, v3(4.5 * cos(a0), -2, 4.5 * sin(a0))), v3_add(c, v3(6.5 * cos(a0), -10, 6.5 * sin(a0))), 0.35, 6, K_STEEL, 2);
        }
    }
    mb_tube(v3(-38, 3, -12), v3(-18, 3, -8), 0.6, 8, K_RUST, 2);   // pipeline to the keep

    // --- Lattice radio mast ------------------------------------------------------
    V3 mb0 = v3(-48, 0, 42);
    for (int l = 0; l < 3; l++) {
        double a0 = l * 2 * PI_D / 3;
        mb_tube(v3_add(mb0, v3(4 * cos(a0), 0, 4 * sin(a0))), v3_add(mb0, v3(0.4 * cos(a0), 48, 0.4 * sin(a0))), 0.3, 6, K_RUST, 2);
    }
    for (int k = 1; k < 12; k++) {
        double y = k * 4.0, r = 4 - 3.6 * y / 48;
        for (int l = 0; l < 3; l++) {
            double a0 = l * 2 * PI_D / 3, a1 = (l + 1) * 2 * PI_D / 3;
            mb_tube(v3_add(mb0, v3(r * cos(a0), y, r * sin(a0))), v3_add(mb0, v3(r * cos(a1), y + 2, r * sin(a1))), 0.12, 4, K_RUST, 2);
        }
    }
    for (int k = 0; k < 4; k++)   // dipole yards
        mb_tube(v3_add(mb0, v3(-7 + k * 0.2, 30 + k * 5, 0)), v3_add(mb0, v3(7 - k * 0.2, 30 + k * 5, 0)), 0.15, 4, K_STEEL, 2);

    // --- Searchlight emplacements -------------------------------------------------
    V3 sl[3] = { v3(-34, 0, -48), v3(30, 0, -60), v3(-62, 0, 4) };
    for (int k = 0; k < 3; k++) {
        const double ring[][2] = { { 5, 0 }, { 5, 1.4 }, { 4.4, 1.4 }, { 4.4, 0.2 } };
        mb_lathe(sl[k], ring, 4, 20, K_CONC_D, 4);
        mb_tube(v3_add(sl[k], v3(0, 0.2, 0)), v3_add(sl[k], v3(0, 2.6, 0)), 0.5, 8, K_STEEL, 2);
        const double drum[][2] = { { 0.0, 2.4 }, { 1.4, 2.4 }, { 1.4, 4.4 }, { 0.0, 4.4 } };
        mb_lathe(sl[k], drum, 4, 16, K_STEEL, 2);
        const double lens[][2] = { { 1.3, 4.45 }, { 0.0, 4.5 } };
        mb_lathe(sl[k], lens, 2, 16, (Color){ 200, 180, 140, 255 }, 7);
    }

    // --- Perimeter lamp posts and a tracked crawler ----------------------------------
    for (int k = 0; k < 8; k++) {
        double a0 = k * 2 * PI_D / 8 + 0.2;
        V3 p = v3(100 * cos(a0), 0, 100 * sin(a0));
        mb_tube(p, v3_add(p, v3(0, 7, 0)), 0.2, 6, K_STEEL, 2);
        mb_box(v3_add(p, v3(-0.5, 7, -0.5)), v3_add(p, v3(0.5, 7.6, 0.5)), K_LAMP, 7);
    }
    mb_box(v3(26, 1.2, 30), v3(36, 4.5, 36), (Color){ 78, 82, 74, 255 }, 2);
    mb_box(v3(25.5, 0, 29.4), v3(36.5, 1.8, 30.4), K_BLACK_TRACK, 3);
    mb_box(v3(25.5, 0, 35.6), v3(36.5, 1.8, 36.6), K_BLACK_TRACK, 3);
    mb_tube(v3(31, 4.5, 33), v3(31, 6.5, 33), 1.0, 10, (Color){ 78, 82, 74, 255 }, 2);
}

// --- Globe -------------------------------------------------------------------------------

static const char *globe_vs =
    "#version 330\n"
    "in vec3 vertexPosition;\n"
    "uniform mat4 mvp; uniform mat4 matModel; uniform float logFar;\n"
    "out vec3 fixedDir; out vec3 camRel; out float fragLogZ;\n"
    "void main(){ fixedDir = vertexPosition; camRel = (matModel * vec4(vertexPosition, 1.0)).xyz;\n"
    "  gl_Position = mvp * vec4(vertexPosition, 1.0); fragLogZ = 1.0 + gl_Position.w; }\n";

static const char *globe_fs =
    "#version 330\n"
    "in vec3 fixedDir; in vec3 camRel; in float fragLogZ;\n"
    "uniform float logFar; uniform mat4 fixedFromSite; uniform vec3 sunF; uniform vec3 earthF; uniform vec3 baseF;\n"
    "uniform float exposure; uniform float baseR; uniform vec3 lampP[4]; uniform vec3 lampC[4];\n"
    "uniform vec4 maria[24]; uniform int nMaria; uniform vec3 tint;\n"
    "out vec4 finalColor;\n"
    "vec3 hash33(vec3 p){ p = fract(p * vec3(0.1031, 0.1030, 0.0973)); p += dot(p, p.yxz + 33.33); return fract((p.xxy + p.yxx) * p.zyx); }\n"
    "float hash13(vec3 p){ p = fract(p * 0.1031); p += dot(p, p.zyx + 31.32); return fract((p.x + p.y) * p.z); }\n"
    "float vnoise3(vec3 p){ vec3 i = floor(p), f = fract(p); f = f*f*(3.0-2.0*f);\n"
    "  return mix(mix(mix(hash13(i),hash13(i+vec3(1,0,0)),f.x),mix(hash13(i+vec3(0,1,0)),hash13(i+vec3(1,1,0)),f.x),f.y),\n"
    "             mix(mix(hash13(i+vec3(0,0,1)),hash13(i+vec3(1,0,1)),f.x),mix(hash13(i+vec3(0,1,1)),hash13(i+vec3(1,1,1)),f.x),f.y),f.z); }\n"
    "float fbm3(vec3 p){ float s=0.0,a=0.5; for(int i=0;i<5;i++){ s+=a*vnoise3(p); p*=2.03; a*=0.5; } return s; }\n"
    "// Craters from a 3D cell pattern: returns albedo factor, adds to slope.\n"
    "float craters(vec3 n, float scale, inout vec3 grad, float strength){\n"
    "  vec3 p = n * scale; vec3 c = floor(p); float alb = 1.0;\n"
    "  for (int x=-1;x<=1;x++) for(int y=-1;y<=1;y++) for(int z=-1;z<=1;z++){\n"
    "    vec3 cell = c + vec3(x,y,z); vec3 h = hash33(cell);\n"
    "    if (h.z < 0.78) continue;\n"
    "    vec3 ctr = cell + h; float r = 0.10 + 0.30 * pow(fract(h.x * 7.3 + h.y), 3.0);\n"
    "    vec3 d = p - ctr; d -= n * dot(d, n); float q = length(d) / r;\n"
    "    if (q > 1.6) continue;\n"
    "    vec3 dir = d / max(length(d), 1e-5);\n"
    "    float slope = q < 1.0 ? 2.0 * q * 0.5 : -3.0 * (q - 1.0) * exp(-(q - 1.0) * 4.0);\n"
    "    grad += dir * slope * strength;\n"
    "    if (q < 0.9) alb *= 0.97; else if (q < 1.15) alb *= 1.06; else alb *= 1.0 + 0.03 * (1.6 - q);\n"
    "  }\n"
    "  return alb;\n"
    "}\n"
    "float angdist(vec3 a, float lat, float lon){ vec3 b = vec3(cos(lat)*cos(lon), cos(lat)*sin(lon), sin(lat)); return acos(clamp(dot(a,b), -1.0, 1.0)); }\n"
    "void main(){\n"
    "  gl_FragDepth = log2(fragLogZ) * logFar * 0.5;\n"
    "  vec3 n = normalize(fixedDir);\n"
    "  float dist = length(camRel);\n"
    "  vec3 V = normalize(-(mat3(fixedFromSite) * camRel));\n"
    "  // Albedo: highlands, maria, basins, ray systems.\n"
    "  float alb = 0.155 + 0.05 * (fbm3(n * 9.0) - 0.5) + 0.03 * (fbm3(n * 40.0) - 0.5);\n"
    "  for (int i = 0; i < nMaria; i++) {\n"
    "    vec4 m = maria[i]; float d = angdist(n, m.x, m.y);\n"
    "    float edge = m.z * (1.0 + 0.22 * (fbm3(n * 14.0 + float(i)) - 0.5) * 2.0);\n"
    "    alb = mix(alb, alb * m.w, smoothstep(edge, edge * 0.82, d));\n"
    "  }\n"
    "  float rays = 0.0;\n"
    "  { float d = angdist(n, radians(-43.3), radians(-11.2)); vec3 t = n * 30.0;\n"
    "    rays += smoothstep(0.45, 0.0, d) * smoothstep(0.62, 0.9, fbm3(t + 3.0)) * 0.6 + smoothstep(0.012, 0.0, d) * 0.5; }\n"
    "  { float d = angdist(n, radians(9.6), radians(-20.1)); rays += smoothstep(0.22, 0.0, d) * smoothstep(0.6, 0.9, fbm3(n * 34.0 + 9.0)) * 0.5; }\n"
    "  { float d = angdist(n, radians(8.1), radians(-38.0)); rays += smoothstep(0.12, 0.0, d) * smoothstep(0.6, 0.9, fbm3(n * 40.0 + 2.0)) * 0.4; }\n"
    "  alb *= 1.0 + rays;\n"
    "  vec3 grad = vec3(0.0);\n"
    "  float far1 = smoothstep(9.0e6, 2.0e6, dist), far2 = smoothstep(2.5e6, 4.0e5, dist), far3 = smoothstep(4.0e5, 6.0e4, dist);\n"
    "  if (far1 > 0.0) alb *= mix(1.0, craters(n, 60.0, grad, 0.05 * far1), far1);\n"
    "  if (far2 > 0.0) alb *= mix(1.0, craters(n, 260.0, grad, 0.05 * far2), far2);\n"
    "  if (far3 > 0.0) alb *= mix(1.0, craters(n, 1400.0, grad, 0.06 * far3), far3);\n"
    "  // The far-side polar crater (permanently shadowed).\n"
    "  float db = acos(clamp(dot(n, baseF), -1.0, 1.0)) / baseR;\n"
    "  float inside = 0.0;\n"
    "  if (db < 1.8) {\n"
    "    vec3 d = n - baseF * dot(n, baseF); vec3 dir = d / max(length(d), 1e-6);\n"
    "    float slope = db < 1.0 ? 1.4 * db : -2.5 * (db - 1.0) * exp(-(db - 1.0) * 3.0);\n"
    "    grad += dir * slope * 0.5;\n"
    "    inside = smoothstep(1.0, 0.85, db);\n"
    "    alb *= db < 1.0 ? 0.9 : db < 1.12 ? 1.15 : 1.0;\n"
    "  }\n"
    "  vec3 N = normalize(n - grad);\n"
    "  float mu0 = max(dot(N, sunF), 0.0) * smoothstep(-0.02, 0.03, dot(n, sunF));\n"
    "  mu0 *= 1.0 - inside;\n"
    "  float mu = max(dot(N, V), 0.03);\n"
    "  float g = acos(clamp(dot(sunF, V), -1.0, 1.0));\n"
    "  float surge = 1.0 + 0.6 * exp(-g / 0.08);\n"
    "  vec3 col = vec3(1.0, 0.95, 0.88) * alb * (2.0 * mu0 / (mu0 + mu)) * surge * 2.6;\n"
    "  col += vec3(0.55, 0.62, 0.8) * alb * 0.012 * max(dot(n, earthF), 0.0) * (1.0 - inside);\n"
    "  for (int i = 0; i < 4; i++) {\n"
    "    vec3 l = lampP[i] - camRel; float d2 = dot(l, l);\n"
    "    vec3 ln = mat3(fixedFromSite) * (l * inversesqrt(d2));\n"
    "    col += alb * lampC[i] * max(dot(N, ln), 0.0) / (d2 + 1.0);\n"
    "  }\n"
    "  finalColor = vec4(col * tint * exposure, 1.0);\n"
    "}\n";

static Shader globe_sh;
static Material globe_mat, lm_mat;
static Mesh globe_mesh, base_mesh, saucer_mesh;

static double g_lat, g_lon, g_alt = 400e3, g_heading = 0;
static int globe_inited;

static V3 fixed_dir(double lat_deg, double lon_deg)
{
    return moon_latlon_to_fixed(lat_deg * DEG, lon_deg * DEG, 1.0);
}

static double globe_radius(void) { return terrain_ref_radius(); }

static Matrix to_matrix(M3 r, V3 t)
{
    Matrix m = { (float)r.m[0][0], (float)r.m[0][1], (float)r.m[0][2], (float)t.x,
                 (float)r.m[1][0], (float)r.m[1][1], (float)r.m[1][2], (float)t.y,
                 (float)r.m[2][0], (float)r.m[2][1], (float)r.m[2][2], (float)t.z, 0, 0, 0, 1 };
    return m;
}

static Mesh make_sphere(int nlon, int nlat)
{
    Mesh m = { 0 };
    m.vertexCount = (nlon + 1) * (nlat + 1);
    m.triangleCount = nlon * nlat * 2;
    m.vertices = malloc(sizeof(float) * 3 * m.vertexCount);
    m.indices = malloc(sizeof(unsigned short) * 3 * m.triangleCount);
    int k = 0;
    for (int j = 0; j <= nlat; j++)
        for (int i = 0; i <= nlon; i++) {
            double lat = -PI_D / 2 + PI_D * j / nlat, lon = 2 * PI_D * i / nlon;
            m.vertices[k++] = (float)(cos(lat) * cos(lon));
            m.vertices[k++] = (float)(cos(lat) * sin(lon));
            m.vertices[k++] = (float)sin(lat);
        }
    k = 0;
    for (int j = 0; j < nlat; j++)
        for (int i = 0; i < nlon; i++) {
            unsigned short a = (unsigned short)(j * (nlon + 1) + i), b = (unsigned short)(a + 1);
            unsigned short c = (unsigned short)(a + nlon + 1), d = (unsigned short)(c + 1);
            m.indices[k++] = a; m.indices[k++] = b; m.indices[k++] = c;
            m.indices[k++] = b; m.indices[k++] = d; m.indices[k++] = c;
        }
    UploadMesh(&m, false);
    return m;
}

static double next_ufo = -1, ufo_t = -1, flying_time;
static V3 ufo_a, ufo_b, ufo_up;
static int ufo_side;

static double frand(void) { return (double)rand() / RAND_MAX; }

void extras_init(void)
{
    xf_r = m3_identity();
    xf_t = v3(0, 0, 0);
    build_saucer();
    saucer_mesh = mb_upload();
    build_base();
    base_mesh = mb_upload();
    globe_mesh = make_sphere(G_NLON, G_NLAT);   // 16-bit indices: (256 * 129) vertices fit
    globe_sh = LoadShaderFromMemory(globe_vs, globe_fs);
    float lf = render_log_far();
    SetShaderValue(globe_sh, GetShaderLocation(globe_sh, "logFar"), &lf, SHADER_UNIFORM_FLOAT);
    float one[3] = { 1, 1, 1 };
    SetShaderValue(globe_sh, GetShaderLocation(globe_sh, "tint"), one, SHADER_UNIFORM_VEC3);
    globe_mat = LoadMaterialDefault();
    globe_mat.shader = globe_sh;
    lm_mat = LoadMaterialDefault();
    lm_mat.shader = *render_lm_shader();
    // Maria and basins: lat, lon (deg), angular radius (deg), albedo factor.
    static const float maria[][4] = {
        { 32.8f, -15.6f, 17, 0.50f }, { 28.0f, 17.5f, 9.5f, 0.52f }, { 8.5f, 31.4f, 10.5f, 0.50f }, { 17.0f, 59.1f, 7, 0.52f },
        { -7.8f, 51.3f, 8.5f, 0.55f }, { -15.2f, 35.5f, 5, 0.56f }, { -21.3f, -16.6f, 8.5f, 0.56f }, { -24.4f, -38.6f, 5, 0.55f },
        { 18.4f, -57.4f, 19, 0.52f }, { -2.0f, -45.0f, 13, 0.55f }, { 56.0f, -30.0f, 6, 0.58f }, { 56.0f, 0.0f, 6, 0.58f },
        { 57.0f, 30.0f, 6, 0.58f }, { 13.3f, 3.6f, 4, 0.58f }, { -10.0f, -23.0f, 5, 0.58f }, { 7.5f, -30.9f, 7, 0.58f },
        { 27.3f, 147.9f, 3.5f, 0.55f }, { -19.4f, -92.8f, 4.5f, 0.6f }, { -33.7f, 163.5f, 2.5f, 0.62f }, { 1.3f, 87.5f, 4, 0.6f },
        { 13.3f, 86.1f, 3.5f, 0.6f }, { -38.9f, 93.0f, 6, 0.65f }, { -53.0f, -169.0f, 30, 0.86f }, { 20.0f, 31.0f, 3, 0.55f },
    };
    float mv[24 * 4];
    for (int i = 0; i < 24; i++) {
        mv[4 * i] = (float)(maria[i][0] * DEG);
        mv[4 * i + 1] = (float)(maria[i][1] * DEG);
        mv[4 * i + 2] = (float)(maria[i][2] * DEG);
        mv[4 * i + 3] = maria[i][3];
    }
    SetShaderValueV(globe_sh, GetShaderLocation(globe_sh, "maria"), mv, SHADER_UNIFORM_VEC4, 24);
    int nm = 24;
    SetShaderValue(globe_sh, GetShaderLocation(globe_sh, "nMaria"), &nm, SHADER_UNIFORM_INT);
    next_ufo = 360 + frand() * 840;   // first chance: 6..20 minutes of flying
}

// --- The rare visitor ----------------------------------------------------------------------

void extras_force_ufo(double after_s) { next_ufo = flying_time + after_s; }

#define UFO_T 3.4

void extras_update(float dt, int flying, const ViewCam *cam)
{
    if (!flying) return;
    flying_time += dt;
    if (ufo_t >= 0) {
        ufo_t += dt;
        if (ufo_t > UFO_T) {
            ufo_t = -1;
            next_ufo = flying_time + 900 + frand() * 1200;
        }
        return;
    }
    if (next_ufo < 0 || flying_time < next_ufo) return;
    // Cross the sky just above the horizon in front of whoever is looking.
    V3 site_c = terrain_fixed_to_site(v3(0, 0, 0));
    V3 up = v3_norm(v3_sub(cam->pos, site_c));
    V3 f = v3_sub(cam->fwd, v3_scale(up, v3_dot(cam->fwd, up)));
    if (v3_len(f) < 0.2) f = v3_cross(up, v3(0, 0, 1));
    f = v3_norm(f);
    V3 r = v3_cross(f, up);
    ufo_side = frand() < 0.5 ? -1 : 1;
    double dist = 1800 + frand() * 3200, el = (2.5 + frand() * 6) * DEG, half = (28 + frand() * 10) * DEG;
    V3 da = v3_add(v3_scale(v3_add(v3_scale(f, cos(half)), v3_scale(r, -ufo_side * sin(half))), cos(el)), v3_scale(up, sin(el)));
    V3 db = v3_add(v3_scale(v3_add(v3_scale(f, cos(half)), v3_scale(r, ufo_side * sin(half))), cos(el)), v3_scale(up, sin(el) * 1.3));
    ufo_a = v3_add(cam->pos, v3_scale(da, dist));
    ufo_b = v3_add(cam->pos, v3_scale(db, dist));
    ufo_up = up;
    ufo_t = 0;
}

static V3 ufo_pos(double t, double *wobble)
{
    // Glide in, stop and hover, then leave far faster than anything should.
    double u, climb = 0;
    *wobble = 0;
    if (t < 1.5) u = 0.5 * (1 - pow(1 - t / 1.5, 2.2));
    else if (t < 2.3) { u = 0.5; *wobble = sin((t - 1.5) * 18) * 0.5; }
    else { double k = (t - 2.3) / (UFO_T - 2.3); u = 0.5 + 3.0 * k * k; climb = 2500 * k * k; }
    return v3_add(v3_add(ufo_a, v3_scale(v3_sub(ufo_b, ufo_a), u)), v3_scale(ufo_up, climb + *wobble * 12));
}

static void glow(Vector3 c, float size, Color col)
{
    Vector3 f = Vector3Normalize(c);
    Vector3 u = fabsf(f.y) < 0.95f ? (Vector3){ 0, 1, 0 } : (Vector3){ 1, 0, 0 };
    Vector3 r = Vector3Normalize(Vector3CrossProduct(f, u));
    u = Vector3CrossProduct(r, f);
    Vector3 p[4] = { Vector3Add(c, Vector3Add(Vector3Scale(r, -size), Vector3Scale(u, -size))),
                     Vector3Add(c, Vector3Add(Vector3Scale(r, size), Vector3Scale(u, -size))),
                     Vector3Add(c, Vector3Add(Vector3Scale(r, size), Vector3Scale(u, size))),
                     Vector3Add(c, Vector3Add(Vector3Scale(r, -size), Vector3Scale(u, size))) };
    rlColor4ub(col.r, col.g, col.b, col.a);
    rlTexCoord2f(0, 0); rlVertex3f(p[0].x, p[0].y, p[0].z);
    rlTexCoord2f(1, 0); rlVertex3f(p[1].x, p[1].y, p[1].z);
    rlTexCoord2f(1, 1); rlVertex3f(p[2].x, p[2].y, p[2].z);
    rlTexCoord2f(0, 1); rlVertex3f(p[3].x, p[3].y, p[3].z);
}

static void fx_begin(float gain)
{
    Shader s = *render_fx_shader();
    rlDrawRenderBatchActive();
    BeginShaderMode(s);
    int k = 0;
    SetShaderValue(s, GetShaderLocation(s, "kind"), &k, SHADER_UNIFORM_INT);
    SetShaderValue(s, GetShaderLocation(s, "gain"), &gain, SHADER_UNIFORM_FLOAT);
    BeginBlendMode(BLEND_ADD_COLORS);
    rlDisableDepthMask();
    rlDisableBackfaceCulling();
    rlBegin(RL_QUADS);
}

static void fx_end(void)
{
    rlEnd();
    rlDrawRenderBatchActive();
    rlEnableBackfaceCulling();
    rlEnableDepthMask();
    EndBlendMode();
    float one = 1.0f;
    Shader s = *render_fx_shader();
    SetShaderValue(s, GetShaderLocation(s, "gain"), &one, SHADER_UNIFORM_FLOAT);
    EndShaderMode();
}

// Local frame on the surface: columns east, up, south (site frame).
static M3 local_frame(V3 up)
{
    V3 e = v3_cross(v3(0, 0, -1), up);
    if (v3_len(e) < 1e-3) e = v3(1, 0, 0);
    e = v3_norm(e);
    V3 s = v3_cross(e, up);
    M3 m = { { { e.x, up.x, s.x }, { e.y, up.y, s.y }, { e.z, up.z, s.z } } };
    return m;
}

void extras_draw_ufo(const ViewCam *cam)
{
    if (ufo_t < 0) return;
    double wob;
    V3 p = ufo_pos(ufo_t, &wob);
    V3 rel = v3_sub(p, cam->pos);
    M3 fr = local_frame(ufo_up);
    M3 tilt = m3_mul(m3_rz(0.12 * ufo_side + wob * 0.15), m3_rx(0.05));
    render_set_lm_extras(0.0f, 1.0f, NULL, NULL, 0);
    DrawMesh(saucer_mesh, lm_mat, to_matrix(m3_mul(fr, tilt), rel));
    fx_begin(3.0f);
    float t = (float)GetTime();
    for (int k = 0; k < 8; k++) {
        double a = k * 2 * PI_D / 8 + t * 4.0;
        V3 q = v3_add(rel, m3_apply(fr, v3(8.2 * cos(a), 0, 8.2 * sin(a))));
        int on = ((int)(t * 10) + k) % 4 == 0;
        Color c = k % 2 ? (Color){ 255, 190, 90, 255 } : (Color){ 255, 120, 60, 255 };
        glow((Vector3){ (float)q.x, (float)q.y, (float)q.z }, on ? 1.4f : 0.8f, c);
    }
    V3 under = v3_add(rel, m3_apply(fr, v3(0, -1.2, 0)));
    glow((Vector3){ (float)under.x, (float)under.y, (float)under.z }, 3.5f, (Color){ 255, 150, 70, 200 });
    fx_end();
}

// --- Globe view -----------------------------------------------------------------------------

// The globe mesh is faceted (flat triangles between grid vertices); put the
// base on the facet under it, not on the ideal sphere above it.


static V3 grid_vertex(int j, int i)
{
    double lat = -PI_D / 2 + PI_D * j / G_NLAT, lon = 2 * PI_D * i / G_NLON;
    return v3(cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat));
}

static double facet_factor(V3 dir)
{
    double lat, lon, r;
    moon_fixed_to_latlon(dir, &lat, &lon, &r);
    if (lon < 0) lon += 2 * PI_D;
    int j = (int)((lat + PI_D / 2) / PI_D * G_NLAT), i = (int)(lon / (2 * PI_D) * G_NLON);
    V3 a = grid_vertex(j, i), b = grid_vertex(j, i + 1), c = grid_vertex(j + 1, i), d = grid_vertex(j + 1, i + 1);
    double best = 1.0;
    V3 tris[2][3] = { { a, b, c }, { b, d, c } };
    for (int k = 0; k < 2; k++) {
        V3 n = v3_norm(v3_cross(v3_sub(tris[k][1], tris[k][0]), v3_sub(tris[k][2], tris[k][0])));
        double t = v3_dot(n, tris[k][0]) / v3_dot(n, dir);
        if (t > 0.99 && t < best) best = t;
    }
    return best;
}

static V3 base_site(V3 *up_site)
{
    V3 f = fixed_dir(BASE_LAT, BASE_LON);
    if (up_site) *up_site = terrain_fixed_dir_to_site(f);
    static double ff = 0;
    if (ff == 0) ff = facet_factor(f);
    return terrain_fixed_to_site(v3_scale(f, globe_radius() * ff - 0.5));
}

static void globe_sphere(const ViewCam *cam, V3 sun_site, V3 earth_site, V3 brel, double R)
{
    (void)brel;
    M3 S = terrain_site_from_fixed();
    M3 St = m3_transpose(S);
    V3 centre = terrain_fixed_to_site(v3(0, 0, 0));
    // The Moon.
    M3 rs = m3_mul(S, (M3){ { { R, 0, 0 }, { 0, R, 0 }, { 0, 0, R } } });
    Matrix ffs = { (float)St.m[0][0], (float)St.m[0][1], (float)St.m[0][2], 0, (float)St.m[1][0], (float)St.m[1][1],
                   (float)St.m[1][2], 0, (float)St.m[2][0], (float)St.m[2][1], (float)St.m[2][2], 0, 0, 0, 0, 1 };
    SetShaderValueMatrix(globe_sh, GetShaderLocation(globe_sh, "fixedFromSite"), ffs);
    V3 sf = m3_apply(St, sun_site), ef = m3_apply(St, earth_site), bf = fixed_dir(BASE_LAT, BASE_LON);
    float v[3];
#define SET3(name, vec) do { v[0] = (float)(vec).x; v[1] = (float)(vec).y; v[2] = (float)(vec).z; \
        SetShaderValue(globe_sh, GetShaderLocation(globe_sh, name), v, SHADER_UNIFORM_VEC3); } while (0)
    SET3("sunF", sf);
    SET3("earthF", ef);
    SET3("baseF", bf);
#undef SET3
    if (getenv("AGC_GLOBEDBG")) {
        V3 cf = m3_apply(St, v3_sub(cam->pos, centre));
        printf("GLOBEDBG sunF %.3f %.3f %.3f  camF %.0f %.0f %.0f  dot %.3f  fwdF %.3f %.3f %.3f\n", sf.x, sf.y, sf.z, cf.x, cf.y, cf.z,
               v3_dot(v3_norm(cf), sf), m3_apply(St, cam->fwd).x, m3_apply(St, cam->fwd).y, m3_apply(St, cam->fwd).z);
    }
    float br = (float)(BASE_CRATER_R / R), ex = render_exposure();
    SetShaderValue(globe_sh, GetShaderLocation(globe_sh, "baseR"), &br, SHADER_UNIFORM_FLOAT);
    SetShaderValue(globe_sh, GetShaderLocation(globe_sh, "exposure"), &ex, SHADER_UNIFORM_FLOAT);
    DrawMesh(globe_mesh, globe_mat, to_matrix(rs, v3_sub(centre, cam->pos)));
}


// The whole Moon behind the detailed terrain in the normal views, so the ground
// never ends at the edge of the terrain patch (or before its chunks load) and
// the limb is a real curve. Drawn first without writing depth: terrain, the LM
// and everything else simply paint over it.
void globe_draw_backdrop(const ViewCam *cam, V3 sun_site, V3 earth_site)
{
    V3 brel = v3_sub(base_site(NULL), cam->pos);
    float zero[12] = { 0 };
    SetShaderValueV(globe_sh, GetShaderLocation(globe_sh, "lampC"), zero, SHADER_UNIFORM_VEC3, 4);
    // Match the detailed terrain's regolith colour where the two meet.
    float tint[3] = { 0.80f, 0.74f, 0.66f };
    SetShaderValue(globe_sh, GetShaderLocation(globe_sh, "tint"), tint, SHADER_UNIFORM_VEC3);
    rlDrawRenderBatchActive();
    rlDisableDepthMask();
    globe_sphere(cam, sun_site, earth_site, brel, globe_radius() - 300.0);
    rlDrawRenderBatchActive();
    rlEnableDepthMask();
    float one[3] = { 1, 1, 1 };
    SetShaderValue(globe_sh, GetShaderLocation(globe_sh, "tint"), one, SHADER_UNIFORM_VEC3);
}

void globe_draw(const ViewCam *cam, V3 sun_site, V3 earth_site)
{
    double R = globe_radius();
    V3 bup;
    V3 bpos = base_site(&bup);
    V3 brel = v3_sub(bpos, cam->pos);
    double bdist = v3_len(brel);

    // Base lamps (camera-relative), for the ground and the buildings.
    M3 bfr = local_frame(bup);
    V3 lp[4] = { v3(0, 40, 0), v3(56, 8, 34), v3(-44, 24, -20), v3(-48, 30, 42) };
    V3 lc[4] = { v3(2600, 1800, 1000), v3(2200, 1400, 700), v3(700, 520, 300), v3(500, 360, 220) };
    float lpf[12], lcf[12];
    V3 lrel[4];
    for (int i = 0; i < 4; i++) {
        lrel[i] = v3_add(brel, m3_apply(bfr, lp[i]));
        lpf[3 * i] = (float)lrel[i].x; lpf[3 * i + 1] = (float)lrel[i].y; lpf[3 * i + 2] = (float)lrel[i].z;
        lcf[3 * i] = (float)lc[i].x; lcf[3 * i + 1] = (float)lc[i].y; lcf[3 * i + 2] = (float)lc[i].z;
    }

    SetShaderValueV(globe_sh, GetShaderLocation(globe_sh, "lampP"), lpf, SHADER_UNIFORM_VEC3, 4);
    SetShaderValueV(globe_sh, GetShaderLocation(globe_sh, "lampC"), lcf, SHADER_UNIFORM_VEC3, 4);
    globe_sphere(cam, sun_site, earth_site, brel, R);

    // The base: only worth drawing as geometry when close; its lights show
    // from much further away - if the camera is above its horizon.
    V3 cam_from_base = v3_scale(brel, -1);
    int above = v3_dot(cam_from_base, bup) > -30;
    if (!above) return;
    if (bdist < 300e3) {
        render_set_lm_extras(0.0f, 0.0f, lrel, lc, 4);   // sunGain 0: permanent shadow
        float ga = 0.02f;
        Shader s = *render_lm_shader();
        SetShaderValue(s, GetShaderLocation(s, "groundAlbedo"), &ga, SHADER_UNIFORM_FLOAT);
        DrawMesh(base_mesh, lm_mat, to_matrix(bfr, brel));
        ga = 0.11f;
        SetShaderValue(s, GetShaderLocation(s, "groundAlbedo"), &ga, SHADER_UNIFORM_FLOAT);
        render_set_lm_extras(0.0f, 1.0f, NULL, NULL, 0);
    }
    float t = (float)GetTime();
    fx_begin(6.0f);
    float minsz = (float)(bdist * 0.0016);
    int beacon = fmodf(t, 1.6f) < 0.12f;
    V3 bc = v3_add(brel, m3_apply(bfr, v3(-48, 49, 42)));
    glow((Vector3){ (float)bc.x, (float)bc.y, (float)bc.z }, fmaxf(beacon ? 6.0f : 1.0f, beacon ? minsz * 1.4f : minsz * 0.4f),
         (Color){ 255, 50, 40, (unsigned char)(beacon ? 255 : 90) });
    V3 dc = v3_add(brel, m3_apply(bfr, v3(0, 10, 0)));
    if (bdist > 20e3) glow((Vector3){ (float)dc.x, (float)dc.y, (float)dc.z }, fmaxf(30.0f, minsz), (Color){ 255, 180, 110, 110 });
    if (bdist < 300e3) {
        for (int k = 0; k < 8; k++) {
            double a = k * 2 * PI_D / 8 + 0.2;
            V3 q = v3_add(brel, m3_apply(bfr, v3(100 * cos(a), 7.4, 100 * sin(a))));
            glow((Vector3){ (float)q.x, (float)q.y, (float)q.z }, 2.5f, (Color){ 255, 190, 120, 200 });
        }
        // Hangar door spill.
        V3 hd = v3_add(brel, m3_apply(bfr, v3(56, 6, 22)));
        glow((Vector3){ (float)hd.x, (float)hd.y, (float)hd.z }, 12.0f, (Color){ 255, 170, 90, 90 });
        // Searchlights: drum glare plus slowly sweeping shafts (lit by
        // the dust the base itself keeps kicking up).
        V3 sl[3] = { v3(-34, 4.5, -48), v3(30, 4.5, -60), v3(-62, 4.5, 4) };
        for (int k = 0; k < 3; k++) {
            V3 p = v3_add(brel, m3_apply(bfr, sl[k]));
            glow((Vector3){ (float)p.x, (float)p.y, (float)p.z }, 3.0f, (Color){ 255, 240, 210, 255 });
        }
        fx_end();
        Shader fs = *render_fx_shader();
        rlDrawRenderBatchActive();
        BeginShaderMode(fs);
        int kind = 1;
        float gain = 0.9f;
        SetShaderValue(fs, GetShaderLocation(fs, "kind"), &kind, SHADER_UNIFORM_INT);
        SetShaderValue(fs, GetShaderLocation(fs, "gain"), &gain, SHADER_UNIFORM_FLOAT);
        BeginBlendMode(BLEND_ADD_COLORS);
        rlDisableDepthMask();
        rlDisableBackfaceCulling();
        rlBegin(RL_QUADS);
        for (int k = 0; k < 3; k++) {
            double az = t * (0.11 + 0.04 * k) + k * 2.1, el = (55 + 15 * sin(t * 0.07 + k)) * DEG;
            V3 a = v3_add(brel, m3_apply(bfr, sl[k]));
            V3 dir = m3_apply(bfr, v3(cos(el) * cos(az), sin(el), cos(el) * sin(az)));
            V3 b = v3_add(a, v3_scale(dir, 1400));
            Vector3 A = { (float)a.x, (float)a.y, (float)a.z }, Bv = { (float)b.x, (float)b.y, (float)b.z };
            Vector3 ax = Vector3Subtract(Bv, A);
            Vector3 mid = Vector3Scale(Vector3Add(A, Bv), 0.5f);
            Vector3 sd = Vector3Normalize(Vector3CrossProduct(ax, mid));
            float wa = 1.4f, wb = 60.0f;
            rlColor4ub(255, 235, 200, 60);
            rlTexCoord2f(0, 0); rlVertex3f(A.x - sd.x * wa, A.y - sd.y * wa, A.z - sd.z * wa);
            rlTexCoord2f(1, 0); rlVertex3f(A.x + sd.x * wa, A.y + sd.y * wa, A.z + sd.z * wa);
            rlTexCoord2f(1, 1); rlVertex3f(Bv.x + sd.x * wb, Bv.y + sd.y * wb, Bv.z + sd.z * wb);
            rlTexCoord2f(0, 1); rlVertex3f(Bv.x - sd.x * wb, Bv.y - sd.y * wb, Bv.z - sd.z * wb);
        }
        rlEnd();
        rlDrawRenderBatchActive();
        rlEnableBackfaceCulling();
        rlEnableDepthMask();
        EndBlendMode();
        float one = 1.0f;
        SetShaderValue(fs, GetShaderLocation(fs, "gain"), &one, SHADER_UNIFORM_FLOAT);
        EndShaderMode();
        return;
    }
    fx_end();
}

void globe_reset(V3 lm_site)
{
    V3 fixed = m3_apply(m3_transpose(terrain_site_from_fixed()), v3_sub(lm_site, terrain_fixed_to_site(v3(0, 0, 0))));
    double lat, lon, r;
    moon_fixed_to_latlon(fixed, &lat, &lon, &r);
    g_lat = lat / DEG;
    g_lon = lon / DEG;
    g_alt = 400e3;
    g_heading = 270;   // looking west, the way the LM flies
    globe_inited = 1;
}

void globe_goto(double lat, double lon, double alt)
{
    g_lat = lat;
    g_lon = lon;
    g_alt = alt;
    g_heading = lat < -80 ? 180 : 0;
    globe_inited = 1;
}

static double min_alt(void)
{
    // Down to a couple of km only over that one crater...
    double d = acos(fmax(-1, fmin(1, v3_dot(fixed_dir(g_lat, g_lon), fixed_dir(BASE_LAT, BASE_LON))))) * globe_radius();
    return d < 120e3 ? 250 : 25e3;
}

void globe_input(int w, int h)
{
    (void)w;
    (void)h;
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        Vector2 d = GetMouseDelta();
        if (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)) {
            g_heading = fmod(g_heading + d.x * 0.3 + 360, 360);
        } else {
            double k = g_alt / globe_radius() * 0.0022 / DEG;
            double hd = g_heading * DEG;
            // Drag the surface: screen up = heading direction.
            double dn = d.y * cos(hd) + d.x * sin(hd), de = d.y * sin(hd) - d.x * cos(hd);
            g_lat = fmax(-89.5, fmin(89.5, g_lat + dn * k));
            g_lon = fmod(g_lon + de * k / fmax(0.05, cos(g_lat * DEG)) + 540, 360) - 180;
        }
    }
    float wh = GetMouseWheelMove();
    if (wh != 0) g_alt = fmax(min_alt(), fmin(40000e3, g_alt * pow(0.82, wh)));
    if (g_alt < min_alt()) g_alt = fmin(g_alt * 1.05 + 200, min_alt());
}

ViewCam globe_camera(V3 lm_site)
{
    if (!globe_inited) globe_reset(lm_site);
    M3 S = terrain_site_from_fixed();
    double R = globe_radius();
    V3 up_f = fixed_dir(g_lat, g_lon);
    V3 east_f = v3_norm(v3_cross(v3(0, 0, 1), up_f));
    if (v3_len(v3_cross(v3(0, 0, 1), up_f)) < 1e-6) east_f = v3(0, 1, 0);
    V3 north_f = v3_cross(up_f, east_f);
    double hd = g_heading * DEG;
    V3 fwd_h = v3_add(v3_scale(north_f, cos(hd)), v3_scale(east_f, sin(hd)));
    // Look straight down from far away, toward the horizon when low.
    double tilt = fmin(1.0, fmax(0.0, (log10(g_alt) - 3.3) / (5.8 - 3.3)));
    double look_down = (12 + 78 * tilt) * DEG;
    V3 dir_f = v3_norm(v3_sub(v3_scale(fwd_h, cos(look_down)), v3_scale(up_f, sin(look_down))));
    V3 pos_f = v3_scale(up_f, R + g_alt);
    // Step back along the view so the target stays under the screen centre.
    ViewCam c;
    V3 centre = terrain_fixed_to_site(v3(0, 0, 0));
    c.pos = v3_add(centre, m3_apply(S, pos_f));
    c.fwd = v3_norm(m3_apply(S, dir_f));
    V3 upv = m3_apply(S, v3_add(v3_scale(up_f, cos(look_down)), v3_scale(fwd_h, sin(look_down))));
    V3 r = v3_norm(v3_cross(c.fwd, upv));
    c.upv = v3_cross(r, c.fwd);
    c.fov_y = 50 * DEG;
    return c;
}

static int project(const ViewCam *cam, V3 p, int w, int h, Vector2 *out)
{
    V3 d = v3_sub(p, cam->pos);
    V3 r = v3_norm(v3_cross(cam->fwd, cam->upv));
    double z = v3_dot(d, cam->fwd);
    if (z <= 0) return 0;
    double th = tan(cam->fov_y / 2);
    double x = v3_dot(d, r) / (z * th * w / h), y = v3_dot(d, cam->upv) / (z * th);
    out->x = (float)((x + 1) * 0.5 * w);
    out->y = (float)((1 - y) * 0.5 * h);
    return fabs(x) < 1.2 && fabs(y) < 1.2;
}

void globe_overlay(const ViewCam *cam, int w, int h, V3 lm_site)
{
    float s = h / 1080.0f;
    if (s < 0.7f) s = 0.7f;
    // LM marker (if on the visible hemisphere).
    V3 centre = terrain_fixed_to_site(v3(0, 0, 0));
    V3 lm_up = v3_norm(v3_sub(lm_site, centre));
    Vector2 sp;
    if (v3_dot(v3_sub(cam->pos, lm_site), lm_up) > 0 && project(cam, lm_site, w, h, &sp)) {
        DrawCircleLinesV(sp, 7 * s, (Color){ 120, 255, 150, 230 });
        DrawTextEx(font_bold, "EAGLE", (Vector2){ sp.x + 10 * s, sp.y - 9 * s }, 18 * s, 0.4f, (Color){ 120, 255, 150, 230 });
    }
    char b[160];
    snprintf(b, sizeof b, "LUNAR ORBIT VIEW   LAT %+06.2f  LON %+07.2f  ALT %.0f km   HDG %03.0f", g_lat, g_lon, g_alt / 1000, g_heading);
    DrawTextEx(font_mono, b, (Vector2){ 18 * s, h - 58 * s }, 17 * s, 0.3f, (Color){ 200, 206, 214, 230 });
    DrawTextEx(font_ui, "right-drag pan   ctrl+right-drag heading   wheel altitude   Home: back over Eagle", (Vector2){ 18 * s, h - 32 * s },
               16 * s, 0.3f, (Color){ 140, 146, 156, 200 });
    if (IsKeyPressed(KEY_HOME)) globe_reset(lm_site);
}
