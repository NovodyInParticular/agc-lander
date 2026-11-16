// Boulders and cobbles around Tranquility Base, with their long low-sun
// shadows. Drawn with the LM shader (material 8: rock, with the baked rock map).

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"
#include "raymath.h"

#include "render.h"
#include "rocks.h"
#include "terrain.h"
#include "terrain_shape.h"

typedef struct { float *p, *n; unsigned char *c; int count, cap; } Rb;
static Rb rb;
static Material rock_mat;
static int ready;

// The rocks are cut into tiles (325 m squares), small stones apart from big
// ones, so each eye draws only the tiles in view and small stones only where
// they are bigger than a pixel or so.
#define TILE_SIZE 325.0
#define MAX_TILES 512
#define SMALL_R 0.45
typedef struct { Rb b; Mesh mesh; V3 ctr; double rad; int small, ix, iz; } Tile;
static Tile tiles[MAX_TILES];
static int ntiles, tile_tris;

static void push(V3 p, V3 n, Color c, int mat)
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
    rb.c[4 * k] = c.r; rb.c[4 * k + 1] = c.g; rb.c[4 * k + 2] = c.b; rb.c[4 * k + 3] = (unsigned char)mat;   // LM-shader material: 8 rock, 9 plain matte
}

// Lumpy rock: subdivided octahedron with per-vertex noise, scaled and rotated
// (its shape numbers come with it from terrain_shape.c).
static void rock(V3 centre, V3 up, const TerrainRock *k, Color col)
{
    static V3 base[6] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
    static const int faces[8][3] = { { 0, 2, 4 }, { 4, 2, 1 }, { 1, 2, 5 }, { 5, 2, 0 }, { 0, 4, 3 }, { 4, 1, 3 }, { 1, 5, 3 }, { 5, 0, 3 } };
    double sx = k->sx, sy = k->sy, sz = k->sz;
    double rot = k->rot;
    V3 east = v3_norm(v3_cross(v3(0, 0, -1), up));
    V3 south = v3_cross(east, up);
    V3 ax = v3_add(v3_scale(east, cos(rot)), v3_scale(south, sin(rot)));
    V3 az = v3_cross(ax, up);
    double seed = k->seed;
    for (int f = 0; f < 8; f++) {
        // Subdivide each face into N x N small triangles (stones: fewer, they are a few pixels).
        const int N = k->r < SMALL_R ? 2 : 3;
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
                    // Mostly radial normals: a rounded boulder, not a faceted gem.
                    for (int k = 0; k < 3; k++)
                        push(P[k], v3_norm(v3_add(v3_scale(v3_norm(v3_sub(P[k], centre)), 0.65), v3_scale(n, 0.35))), col, 8);
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
        push(P[0], n, k, 9); push(P[1], n, k, 9); push(P[2], n, k, 9);
        push(P[0], n, k, 9); push(P[2], n, k, 9); push(P[3], n, k, 9);
    }
}

static Tile *tile_for(double x, double z, int small)
{
    int ix = (int)floor(x / TILE_SIZE), iz = (int)floor(z / TILE_SIZE);
    for (int i = 0; i < ntiles; i++)
        if (tiles[i].ix == ix && tiles[i].iz == iz && tiles[i].small == small) return &tiles[i];
    if (ntiles == MAX_TILES) return &tiles[MAX_TILES - 1];
    Tile *t = &tiles[ntiles++];
    t->ix = ix;
    t->iz = iz;
    t->small = small;
    return t;
}

void rocks_init(void)
{
    if (ready) return;
    V3 sun = terrain_sun();
    double sh = sqrt(sun.x * sun.x + sun.z * sun.z);
    double sdx = -sun.x / sh, sdz = -sun.z / sh;          // shadows point away from the Sun
    double slope = sun.y / sh;                            // tan(elevation)
    const TerrainRock *list;
    int N = terrain_rocks(&list);
    for (int i = 0; i < N; i++) {
        const TerrainRock *k = &list[i];
        double x = k->x, z = k->z, r = k->r;
        Tile *tl = tile_for(x, z, r < SMALL_R);
        rb = tl->b;
        double h = terrain_height(x, z, 0);
        V3 up = v3_norm(v3_sub(terrain_plane_to_site(x, z, h + 1), terrain_plane_to_site(x, z, h)));
        V3 c = terrain_plane_to_site(x, z, h + r * 0.15);
        unsigned char g = k->grey;
        rock(c, up, k, (Color){ g, (unsigned char)(g * 0.97), (unsigned char)(g * 0.93), 255 });
        if (r > 0.12) shadow(x, z, r * 0.8, r * 0.75 / slope, sdx, sdz);
        tl->b = rb;
    }
    memset(&rb, 0, sizeof rb);
    int nt = 0;
    for (int i = 0; i < ntiles; i++) {
        Tile *t = &tiles[i];
        if (t->b.count == 0) continue;
        // Bounding sphere of the tile's vertices.
        V3 lo = v3(1e30, 1e30, 1e30), hi = v3(-1e30, -1e30, -1e30);
        for (int k = 0; k < t->b.count; k++) {
            V3 p = v3(t->b.p[3 * k], t->b.p[3 * k + 1], t->b.p[3 * k + 2]);
            lo = v3(fmin(lo.x, p.x), fmin(lo.y, p.y), fmin(lo.z, p.z));
            hi = v3(fmax(hi.x, p.x), fmax(hi.y, p.y), fmax(hi.z, p.z));
        }
        t->ctr = v3_scale(v3_add(lo, hi), 0.5);
        t->rad = v3_len(v3_sub(hi, lo)) * 0.5;
        Mesh m = { 0 };
        m.vertexCount = t->b.count;
        m.triangleCount = t->b.count / 3;
        m.vertices = t->b.p;
        m.normals = t->b.n;
        m.colors = t->b.c;
        UploadMesh(&m, false);
        t->mesh = m;
        tile_tris += m.triangleCount;
        memset(&t->b, 0, sizeof t->b);
        nt++;
    }
    rock_mat = LoadMaterialDefault();
    render_lm_material(&rock_mat);
    ready = 1;
    TraceLog(LOG_INFO, "rocks: %d triangles in %d tiles", tile_tris, nt);
}

void rocks_draw(V3 camera_site)
{
    if (!ready) return;
    // Small stones (under 0.45 m) are a pixel or less beyond this distance.
    double small_far = render_quality()->low ? 140.0 : 260.0;
    Matrix m = MatrixTranslate((float)-camera_site.x, (float)-camera_site.y, (float)-camera_site.z);
    for (int i = 0; i < ntiles; i++) {
        Tile *t = &tiles[i];
        if (!t->mesh.vertexCount) continue;
        V3 c = v3_sub(t->ctr, camera_site);
        if (t->small && v3_len(c) - t->rad > small_far) continue;
        if (!render_sphere_visible(c, t->rad)) continue;
        DrawMesh(t->mesh, rock_mat, m);
    }
}
