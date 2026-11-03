// Boulders and cobbles around Tranquility Base, with their long low-sun
// shadows. Drawn with the LM shader (diffuse "black paint" material, grey).

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"
#include "raymath.h"

#include "render.h"
#include "rocks.h"
#include "terrain.h"

typedef struct { float *p, *n; unsigned char *c; int count, cap; } Rb;
static Rb rb;
static Mesh rocks_mesh;
static Material rock_mat;
static int ready;

static void push(V3 p, V3 n, Color c)
{
    if (rb.count == rb.cap) {
        rb.cap = rb.cap ? rb.cap * 2 : 65536;
        rb.p = realloc(rb.p, sizeof(float) * 3 * rb.cap);
        rb.n = realloc(rb.n, sizeof(float) * 3 * rb.cap);
        rb.c = realloc(rb.c, 4 * rb.cap);
    }
    int k = rb.count++;
    rb.p[3 * k] = (float)p.x; rb.p[3 * k + 1] = (float)p.y; rb.p[3 * k + 2] = (float)p.z;
    rb.n[3 * k] = (float)n.x; rb.n[3 * k + 1] = (float)n.y; rb.n[3 * k + 2] = (float)n.z;
    rb.c[4 * k] = c.r; rb.c[4 * k + 1] = c.g; rb.c[4 * k + 2] = c.b; rb.c[4 * k + 3] = 3;   // material 3: matte
}

static unsigned rng = 12345u;
static double frand(void) { rng = rng * 1664525u + 1013904223u; return (rng >> 8) / 16777216.0; }

// Lumpy rock: subdivided octahedron with per-vertex noise, scaled and rotated.
static void rock(V3 centre, V3 up, double r, Color col)
{
    static V3 base[6] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
    static const int faces[8][3] = { { 0, 2, 4 }, { 4, 2, 1 }, { 1, 2, 5 }, { 5, 2, 0 }, { 0, 4, 3 }, { 4, 1, 3 }, { 1, 5, 3 }, { 5, 0, 3 } };
    double sx = r * (0.8 + 0.5 * frand()), sy = r * (0.45 + 0.4 * frand()), sz = r * (0.8 + 0.5 * frand());
    double rot = frand() * 2 * PI_D;
    V3 east = v3_norm(v3_cross(v3(0, 0, -1), up));
    V3 south = v3_cross(east, up);
    V3 ax = v3_add(v3_scale(east, cos(rot)), v3_scale(south, sin(rot)));
    V3 az = v3_cross(ax, up);
    double seed = frand() * 100;
    for (int f = 0; f < 8; f++) {
        // Subdivide each face into 4 x 4 small triangles.
        const int N = 3;
        V3 A = base[faces[f][0]], B = base[faces[f][1]], C = base[faces[f][2]];
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N - i; j++) {
                V3 q[3][2];
                for (int t = 0; t < 2; t++) {
                    int ii[3], jj[3];
                    if (t == 0) { ii[0] = i; jj[0] = j; ii[1] = i + 1; jj[1] = j; ii[2] = i; jj[2] = j + 1; }
                    else { if (i + j + 1 >= N) continue; ii[0] = i + 1; jj[0] = j; ii[1] = i + 1; jj[1] = j + 1; ii[2] = i; jj[2] = j + 1; }
                    V3 P[3];
                    for (int k = 0; k < 3; k++) {
                        double u = (double)ii[k] / N, v = (double)jj[k] / N;
                        V3 d = v3_norm(v3_add(v3_add(v3_scale(A, 1 - u - v), v3_scale(B, u)), v3_scale(C, v)));
                        double bump = 1 + 0.18 * sin(d.x * 5.1 + seed) * sin(d.y * 4.3 + seed * 1.3) * sin(d.z * 3.7 + seed * 0.7);
                        V3 l = v3(d.x * sx * bump, d.y * sy * bump, d.z * sz * bump);
                        P[k] = v3_add(centre, v3_add(v3_add(v3_scale(ax, l.x), v3_scale(up, l.y)), v3_scale(az, l.z)));
                    }
                    V3 n = v3_norm(v3_cross(v3_sub(P[1], P[0]), v3_sub(P[2], P[0])));
                    if (v3_dot(n, v3_sub(P[0], centre)) < 0) { V3 t2 = P[1]; P[1] = P[2]; P[2] = t2; n = v3_scale(n, -1); }
                    for (int k = 0; k < 3; k++) push(P[k], n, col);
                    (void)q;
                }
            }
    }
}

// Shadow cast on the ground, following the terrain.
static void shadow(double x, double z, double w, double len, double sdx, double sdz)
{
    const int SEG = 5;
    double px = -sdz, pz = sdx;   // perpendicular in the plane
    Color k = { 10, 10, 10, 255 };
    for (int i = 0; i < SEG; i++) {
        double t0 = len * i / SEG, t1 = len * (i + 1) / SEG;
        double w0 = w * (1 - 0.55 * i / SEG), w1 = w * (1 - 0.55 * (i + 1) / SEG);
        double xs[4] = { x + sdx * t0 - px * w0, x + sdx * t0 + px * w0, x + sdx * t1 + px * w1, x + sdx * t1 - px * w1 };
        double zs[4] = { z + sdz * t0 - pz * w0, z + sdz * t0 + pz * w0, z + sdz * t1 + pz * w1, z + sdz * t1 - pz * w1 };
        V3 P[4];
        for (int k2 = 0; k2 < 4; k2++) P[k2] = terrain_plane_to_site(xs[k2], zs[k2], terrain_height(xs[k2], zs[k2], 0) + 0.04);
        V3 n = v3_norm(v3_cross(v3_sub(P[1], P[0]), v3_sub(P[2], P[0])));
        if (n.y < 0) n = v3_scale(n, -1);
        push(P[0], n, k); push(P[1], n, k); push(P[2], n, k);
        push(P[0], n, k); push(P[2], n, k); push(P[3], n, k);
    }
}

void rocks_init(void)
{
    if (ready) return;
    rng = 1969u;
    V3 sun = terrain_sun();
    double sh = sqrt(sun.x * sun.x + sun.z * sun.z);
    double sdx = -sun.x / sh, sdz = -sun.z / sh;          // shadows point away from the Sun
    double slope = sun.y / sh;                            // tan(elevation)
    const int N = 4500;
    for (int i = 0; i < N; i++) {
        // Denser near the landing area, sparser out to ~1.2 km; a few clusters.
        double rr = 1200 * pow(frand(), 0.75), th = frand() * 2 * PI_D;
        double x = rr * cos(th), z = rr * sin(th);
        if (i % 9 == 0) {   // boulder cluster around a few random centres
            int cl = (int)(frand() * 6);
            double cx = -300 + 140 * cl, cz = 120 - 90 * (cl % 3);
            x = cx + (frand() - 0.5) * 60;
            z = cz + (frand() - 0.5) * 60;
        }
        double r = 0.14 * pow(frand() + 1e-3, -0.6);
        if (r > 2.8) r = 2.8;
        double h = terrain_height(x, z, 0);
        V3 up = v3_norm(v3_sub(terrain_plane_to_site(x, z, h + 1), terrain_plane_to_site(x, z, h)));
        V3 c = terrain_plane_to_site(x, z, h + r * 0.15);
        unsigned char g = (unsigned char)(28 + frand() * 16);
        rock(c, up, r, (Color){ g, (unsigned char)(g * 0.97), (unsigned char)(g * 0.93), 255 });
        if (r > 0.12) shadow(x, z, r * 0.8, r * 0.75 / slope, sdx, sdz);
    }
    Mesh m = { 0 };
    m.vertexCount = rb.count;
    m.triangleCount = rb.count / 3;
    m.vertices = rb.p;
    m.normals = rb.n;
    m.colors = rb.c;
    UploadMesh(&m, false);
    rocks_mesh = m;
    memset(&rb, 0, sizeof rb);
    rock_mat = LoadMaterialDefault();
    rock_mat.shader = *render_lm_shader();
    ready = 1;
    TraceLog(LOG_INFO, "rocks: %d triangles", m.triangleCount);
}

void rocks_draw(V3 camera_site)
{
    if (!ready) return;
    DrawMesh(rocks_mesh, rock_mat, MatrixTranslate((float)-camera_site.x, (float)-camera_site.y, (float)-camera_site.z));
}
