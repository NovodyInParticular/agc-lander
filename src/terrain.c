// Procedural lunar terrain: crater fields + regolith undulation, quadtree LOD,
// baked low-sun shadows, background generation threads.

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"

#include "moon.h"
#include "terrain.h"

// ---------------------------------------------------------------------------
// Frames.

static V3 site_dir, east, north, up;   // moon-fixed unit vectors
static double R0;                        // reference radius (landing site)
static V3 sun_site = { 0.98, 0.19, 0.0 };

void terrain_set_sun(V3 s) { sun_site = v3_norm(s); }
V3 terrain_sun(void) { return sun_site; }
double terrain_ref_radius(void) { return R0; }

M3 terrain_site_from_fixed(void) { return m3_rows(east, up, v3_scale(north, -1)); }

V3 terrain_fixed_dir_to_site(V3 d)
{
    return v3(v3_dot(d, east), v3_dot(d, up), -v3_dot(d, north));
}

V3 terrain_fixed_to_site(V3 p)
{
    return terrain_fixed_dir_to_site(v3_sub(p, v3_scale(site_dir, R0)));
}

static V3 plane_to_dir(double x, double z)
{
    return v3_norm(v3_add(up, v3_sub(v3_scale(east, x / R0), v3_scale(north, z / R0))));
}

V3 terrain_plane_to_site(double x, double z, double h)
{
    V3 d = plane_to_dir(x, z);
    return terrain_fixed_to_site(v3_scale(d, R0 + h));
}

void terrain_site_to_plane(V3 s, double *x, double *z)
{
    // site -> fixed direction, then gnomonic projection.
    V3 p = v3_add(v3_scale(site_dir, R0), v3_add(v3_scale(east, s.x), v3_add(v3_scale(up, s.y), v3_scale(north, -s.z))));
    double du = v3_dot(p, up);
    if (du < 1.0) du = 1.0;
    *x = R0 * v3_dot(p, east) / du;
    *z = -R0 * v3_dot(p, north) / du;
}

// ---------------------------------------------------------------------------
// Hashing and noise.

static inline uint32_t hash2(int32_t x, int32_t y, uint32_t seed)
{
    uint32_t h = (uint32_t)x * 0x8da6b343u ^ (uint32_t)y * 0xd8163841u ^ seed * 0xcb1ab31fu;
    h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15;
    return h;
}
static inline double u01(uint32_t h) { return (h & 0xffffff) / 16777216.0; }

static double vnoise(double x, double y, uint32_t seed)
{
    double fx = floor(x), fy = floor(y);
    int32_t ix = (int32_t)fx, iy = (int32_t)fy;
    double tx = x - fx, ty = y - fy;
    tx = tx * tx * (3 - 2 * tx);
    ty = ty * ty * (3 - 2 * ty);
    double a = u01(hash2(ix, iy, seed)), b = u01(hash2(ix + 1, iy, seed));
    double c = u01(hash2(ix, iy + 1, seed)), d = u01(hash2(ix + 1, iy + 1, seed));
    return (a + (b - a) * tx) + ((c + (d - c) * tx) - (a + (b - a) * tx)) * ty;
}

// ---------------------------------------------------------------------------
// Craters.

typedef struct { double cell; double prob; uint32_t seed; } Octave;
static const Octave octaves[] = {
    { 24000, 0.30, 11 }, { 6000, 0.45, 12 }, { 1500, 0.55, 13 }, { 400, 0.60, 14 },
    { 100, 0.65, 15 }, { 25, 0.65, 16 }, { 6, 0.55, 17 }, { 1.6, 0.45, 18 },
};
#define NOCT (int)(sizeof octaves / sizeof octaves[0])

// Hand-placed features near the landing site, in plane coordinates.
// "West crater"-like fresh blocky crater just past the target, plus a few.
typedef struct { double x, z, r, depth, rim, fresh; } Feature;
static const Feature features[] = {
    { -380, -120, 95, 22, 5, 1.0 },     // fresh, boulder strewn
    { 240, 310, 160, 20, 4, 0.4 },
    { -900, 450, 60, 9, 2, 0.8 },
    { 650, -520, 300, 30, 6, 0.2 },
    { -60, 520, 35, 6, 1.4, 0.7 },
};
#define NFEAT (int)(sizeof features / sizeof features[0])

static double crater_profile(double r, double depth, double rim)
{
    if (r < 1.0) {
        double r2 = r * r;
        double floor_ = -depth * 0.82;
        double bowl = -depth * (1.0 - r2) + rim * r2 * r2 * r2;
        return bowl < floor_ ? floor_ + (bowl - floor_) * 0.15 : bowl;
    }
    double t = r - 1.0;
    double e = 1.0 / (1.0 + 5.0 * t);
    return rim * e * e * e;
}

typedef struct { double h, fresh; } Sample;

static void crater_octave(double x, double z, const Octave *o, Sample *s)
{
    double fx = floor(x / o->cell), fz = floor(z / o->cell);
    int32_t cx = (int32_t)fx, cz = (int32_t)fz;
    for (int dz = -1; dz <= 1; dz++) {
        for (int dx = -1; dx <= 1; dx++) {
            uint32_t h = hash2(cx + dx, cz + dz, o->seed);
            if (u01(h) > o->prob) continue;
            uint32_t h2 = hash2(cx + dx, cz + dz, o->seed + 101);
            uint32_t h3 = hash2(cx + dx, cz + dz, o->seed + 202);
            double px = (cx + dx + 0.15 + 0.7 * u01(h2)) * o->cell;
            double pz = (cz + dz + 0.15 + 0.7 * u01(h2 >> 7 | h2 << 25)) * o->cell;
            double rr = o->cell * (0.08 + 0.30 * u01(h3) * u01(h3 >> 9 | h3 << 23));
            double ddx = x - px, ddz = z - pz;
            double d2 = ddx * ddx + ddz * ddz;
            if (d2 > rr * rr * 6.25) continue;
            double r = sqrt(d2) / rr;
            double fresh = u01(hash2(cx + dx, cz + dz, o->seed + 303));
            fresh = fresh * fresh * fresh;
            double depth = rr * (0.10 + 0.30 * fresh);
            double rim = depth * (0.18 + 0.12 * fresh);
            s->h += crater_profile(r, depth, rim);
            if (r < 2.5) {
                double w = r < 1.0 ? 1.0 : (2.5 - r) / 1.5;
                double f = fresh * w * (rr < 400 ? 1.0 : 0.4);
                if (f > s->fresh) s->fresh = f;
            }
        }
    }
}

static void eval(double x, double z, double min_feature, Sample *s)
{
    s->h = 0;
    s->fresh = 0;
    // Gentle mare undulation.
    double a = 0;
    a += 140.0 * (vnoise(x / 30000.0, z / 30000.0, 1) - 0.5);
    a += 45.0 * (vnoise(x / 7000.0, z / 7000.0, 2) - 0.5);
    if (min_feature < 400) a += 9.0 * (vnoise(x / 1200.0, z / 1200.0, 3) - 0.5);
    if (min_feature < 60) a += 1.6 * (vnoise(x / 180.0, z / 180.0, 4) - 0.5);
    if (min_feature < 8) a += 0.25 * (vnoise(x / 22.0, z / 22.0, 5) - 0.5);
    // Flatten the immediate landing area a little (real site: gentle mare).
    double dsite = sqrt(x * x + z * z);
    double flat = dsite < 3000 ? 0.35 + 0.65 * dsite / 3000 : 1.0;
    s->h = a * flat;

    for (int i = 0; i < NOCT; i++) {
        if (octaves[i].cell * 0.38 < min_feature) break;
        Sample c = { 0, 0 };
        crater_octave(x, z, &octaves[i], &c);
        double damp = (i < 2) ? flat : 1.0;
        s->h += c.h * damp;
        if (c.fresh > s->fresh) s->fresh = c.fresh;
    }
    for (int i = 0; i < NFEAT; i++) {
        const Feature *f = &features[i];
        if (f->r * 0.5 < min_feature) continue;
        double dx = x - f->x, dz = z - f->z;
        double r = sqrt(dx * dx + dz * dz) / f->r;
        if (r > 3.0) continue;
        s->h += crater_profile(r, f->depth, f->rim);
        if (r < 2.5) {
            double fr = f->fresh * (r < 1 ? 1.0 : (2.5 - r) / 1.5);
            if (fr > s->fresh) s->fresh = fr;
        }
    }
}

double terrain_height(double x, double z, double min_feature)
{
    Sample s;
    eval(x, z, min_feature, &s);
    return s.h;
}

void terrain_surface_props(double x, double z, double min_feature, float *albedo, float *fresh)
{
    Sample s;
    eval(x, z, min_feature, &s);
    double base = 0.085 + 0.02 * (vnoise(x / 9000.0, z / 9000.0, 21) - 0.5) + 0.012 * (vnoise(x / 700.0, z / 700.0, 22) - 0.5);
    *albedo = (float)(base + 0.07 * s.fresh);
    *fresh = (float)s.fresh;
}

double terrain_radius_at(double lat, double lon)
{
    V3 d = moon_latlon_to_fixed(lat, lon, 1.0);
    double du = v3_dot(d, up);
    if (du < 0.2) return R0;   // far outside the modelled region
    double x = R0 * v3_dot(d, east) / du, z = -R0 * v3_dot(d, north) / du;
    return R0 + terrain_height(x, z, 0);
}

// ---------------------------------------------------------------------------
// Chunks.

#define GRID 32                       // quads per chunk side
#define NV ((GRID + 1) * (GRID + 1))
#define ROOT_SIZE 4194304.0           // plane metres (covers ~ +-50 deg)
#define MAX_LEVEL 17
#define MAX_NODES 6000

typedef struct Node {
    int level;
    double x0, z0, size;              // plane bounds
    struct Node *child[4];
    struct Node *parent;
    int state;                        // 0 none, 1 queued, 2 cpu-ready, 3 gpu-ready
    double hmin, hmax;
    // CPU-side data (filled by worker), freed after upload.
    float *pos, *nrm;
    unsigned char *col;
    unsigned short *idx;
    int nverts, nidx;
    V3 origin;                        // site-frame chunk origin
    Mesh mesh;
    unsigned last_used;
} Node;

static Node *root;
static int node_count;
static unsigned frame_no;
static int drawn_count;

// Job queue.
#define QCAP 4096
static Node *queue[QCAP];
static int q_head, q_tail;
static pthread_mutex_t q_mx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t q_cv = PTHREAD_COND_INITIALIZER;
static pthread_t workers[16];
static int nworkers;
static volatile int quitting;
static int busy;

static Node *new_node(int level, double x0, double z0, double size, Node *parent)
{
    Node *n = calloc(1, sizeof *n);
    n->level = level;
    n->x0 = x0;
    n->z0 = z0;
    n->size = size;
    n->parent = parent;
    node_count++;
    return n;
}

static double sun_slope;   // tan(sun elevation)
static double sun_ax, sun_az;  // horizontal unit direction toward the sun (plane)

static float shadow_at(double x, double z, double h0, double spacing)
{
    double t = fmax(spacing * 0.6, 0.4);
    double vis = 1.0;
    const double soft = 22.0;   // penumbra sharpness
    while (t < 9000.0) {
        double mf = fmax(spacing * 1.5, t * 0.04);   // same detail floor as the vertex itself
        double hs = terrain_height(x + sun_ax * t, z + sun_az * t, mf) + t * t / (2 * R0);
        double hl = h0 + t * sun_slope;
        double v = soft * (hl - hs) / t + 0.5;
        if (v < vis) vis = v;
        if (vis <= 0) return 0;
        t *= 1.13;
        t += spacing * 0.15;
    }
    if (vis > 1) vis = 1;
    return (float)vis;
}

static void build_chunk(Node *n)
{
    double spacing = n->size / GRID;
    double mf = spacing * 1.5;
    int skirt = 4 * (GRID + 1);
    int nv = NV + skirt;
    n->pos = malloc(sizeof(float) * 3 * nv);
    n->nrm = malloc(sizeof(float) * 3 * nv);
    n->col = malloc(4 * nv);
    double *hh = malloc(sizeof(double) * NV);
    float *alb = malloc(sizeof(float) * NV), *fr = malloc(sizeof(float) * NV);
    V3 *p3 = malloc(sizeof(V3) * NV);

    double cx = n->x0 + n->size / 2, cz = n->z0 + n->size / 2;
    n->origin = terrain_plane_to_site(cx, cz, 0);
    n->hmin = 1e9;
    n->hmax = -1e9;
    for (int j = 0; j <= GRID; j++)
        for (int i = 0; i <= GRID; i++) {
            int k = j * (GRID + 1) + i;
            double x = n->x0 + i * spacing, z = n->z0 + j * spacing;
            Sample s;
            eval(x, z, mf, &s);
            hh[k] = s.h;
            terrain_surface_props(x, z, mf, &alb[k], &fr[k]);
            p3[k] = terrain_plane_to_site(x, z, s.h);
            if (s.h < n->hmin) n->hmin = s.h;
            if (s.h > n->hmax) n->hmax = s.h;
        }

    // Shadows at half resolution for big chunks, full for small ones.
    float *sh = malloc(sizeof(float) * NV);
    int step = spacing > 50 ? 2 : 1;
    for (int j = 0; j <= GRID; j += step)
        for (int i = 0; i <= GRID; i += step) {
            int k = j * (GRID + 1) + i;
            sh[k] = shadow_at(n->x0 + i * spacing, n->z0 + j * spacing, hh[k], spacing);
        }
    if (step == 2)
        for (int j = 0; j <= GRID; j++)
            for (int i = 0; i <= GRID; i++) {
                if (!(i & 1) && !(j & 1)) continue;
                int i0 = i & ~1, j0 = j & ~1, i1 = i0 + 2 > GRID ? i0 : i0 + 2, j1 = j0 + 2 > GRID ? j0 : j0 + 2;
                double tx = (i & 1) ? 0.5 : 0, tz = (j & 1) ? 0.5 : 0;
                float a = sh[j0 * (GRID + 1) + i0], b = sh[j0 * (GRID + 1) + i1];
                float c = sh[j1 * (GRID + 1) + i0], d = sh[j1 * (GRID + 1) + i1];
                sh[j * (GRID + 1) + i] = (float)((a + (b - a) * tx) * (1 - tz) + (c + (d - c) * tx) * tz);
            }

    for (int j = 0; j <= GRID; j++)
        for (int i = 0; i <= GRID; i++) {
            int k = j * (GRID + 1) + i;
            V3 p = v3_sub(p3[k], n->origin);
            n->pos[3 * k] = (float)p.x;
            n->pos[3 * k + 1] = (float)p.y;
            n->pos[3 * k + 2] = (float)p.z;
            // Normal from neighbours (with curvature).
            int il = i > 0 ? i - 1 : i, ir = i < GRID ? i + 1 : i;
            int jd = j > 0 ? j - 1 : j, ju = j < GRID ? j + 1 : j;
            V3 dx = v3_sub(p3[j * (GRID + 1) + ir], p3[j * (GRID + 1) + il]);
            V3 dz = v3_sub(p3[ju * (GRID + 1) + i], p3[jd * (GRID + 1) + i]);
            V3 nn = v3_norm(v3_cross(dz, dx));
            n->nrm[3 * k] = (float)nn.x;
            n->nrm[3 * k + 1] = (float)nn.y;
            n->nrm[3 * k + 2] = (float)nn.z;
            n->col[4 * k] = (unsigned char)fminf(255.0f, alb[k] * 1000.0f);
            n->col[4 * k + 1] = (unsigned char)(sh[k] * 255.0f);
            n->col[4 * k + 2] = (unsigned char)(fr[k] * 255.0f);
            n->col[4 * k + 3] = 255;
        }

    // Skirts: copies of the border vertices, pushed down, to hide LOD cracks.
    double drop = spacing * 1.5 + 2.0;
    int sk = NV;
    for (int e = 0; e < 4; e++)
        for (int t = 0; t <= GRID; t++) {
            int i = e == 0 ? t : e == 1 ? GRID : e == 2 ? GRID - t : 0;
            int j = e == 0 ? 0 : e == 1 ? t : e == 2 ? GRID : GRID - t;
            int k = j * (GRID + 1) + i;
            V3 dn = terrain_fixed_dir_to_site(plane_to_dir(n->x0 + i * spacing, n->z0 + j * spacing));
            n->pos[3 * sk] = n->pos[3 * k] - (float)(dn.x * drop);
            n->pos[3 * sk + 1] = n->pos[3 * k + 1] - (float)(dn.y * drop);
            n->pos[3 * sk + 2] = n->pos[3 * k + 2] - (float)(dn.z * drop);
            memcpy(&n->nrm[3 * sk], &n->nrm[3 * k], sizeof(float) * 3);
            memcpy(&n->col[4 * sk], &n->col[4 * k], 4);
            sk++;
        }

    int ntri = GRID * GRID * 2 + 4 * GRID * 4;   // grid + double-sided skirts
    n->idx = malloc(sizeof(unsigned short) * 3 * ntri);
    int q = 0;
    for (int j = 0; j < GRID; j++)
        for (int i = 0; i < GRID; i++) {
            unsigned short a = (unsigned short)(j * (GRID + 1) + i), b = a + 1;
            unsigned short c = (unsigned short)(a + GRID + 1), d = c + 1;
            // Alternate diagonal for nicer shading.
            if ((i ^ j) & 1) {
                n->idx[q++] = a; n->idx[q++] = c; n->idx[q++] = b;
                n->idx[q++] = b; n->idx[q++] = c; n->idx[q++] = d;
            } else {
                n->idx[q++] = a; n->idx[q++] = c; n->idx[q++] = d;
                n->idx[q++] = a; n->idx[q++] = d; n->idx[q++] = b;
            }
        }
    for (int e = 0; e < 4; e++)
        for (int t = 0; t < GRID; t++) {
            int i = e == 0 ? t : e == 1 ? GRID : e == 2 ? GRID - t : 0;
            int j = e == 0 ? 0 : e == 1 ? t : e == 2 ? GRID : GRID - t;
            int i2 = e == 0 ? t + 1 : e == 1 ? GRID : e == 2 ? GRID - t - 1 : 0;
            int j2 = e == 0 ? 0 : e == 1 ? t + 1 : e == 2 ? GRID : GRID - t - 1;
            unsigned short a = (unsigned short)(j * (GRID + 1) + i), b = (unsigned short)(j2 * (GRID + 1) + i2);
            unsigned short sa = (unsigned short)(NV + e * (GRID + 1) + t), sb = sa + 1;
            n->idx[q++] = a; n->idx[q++] = sa; n->idx[q++] = b;
            n->idx[q++] = b; n->idx[q++] = sa; n->idx[q++] = sb;
            n->idx[q++] = a; n->idx[q++] = b; n->idx[q++] = sa;   // double-sided skirt
            n->idx[q++] = b; n->idx[q++] = sb; n->idx[q++] = sa;
        }
    n->nverts = nv;
    n->nidx = q;
    free(hh);
    free(alb);
    free(fr);
    free(p3);
    free(sh);
}

static void *worker(void *arg)
{
    (void)arg;
    for (;;) {
        pthread_mutex_lock(&q_mx);
        while (q_head == q_tail && !quitting)
            pthread_cond_wait(&q_cv, &q_mx);
        if (quitting) {
            pthread_mutex_unlock(&q_mx);
            return NULL;
        }
        Node *n = queue[q_head];
        q_head = (q_head + 1) % QCAP;
        busy++;
        pthread_mutex_unlock(&q_mx);
        build_chunk(n);
        pthread_mutex_lock(&q_mx);
        n->state = 2;
        busy--;
        pthread_mutex_unlock(&q_mx);
    }
}

static void enqueue(Node *n)
{
    pthread_mutex_lock(&q_mx);
    int next = (q_tail + 1) % QCAP;
    if (next != q_head) {
        queue[q_tail] = n;
        q_tail = next;
        n->state = 1;
        pthread_cond_signal(&q_cv);
    }
    pthread_mutex_unlock(&q_mx);
}

static void upload(Node *n)
{
    // Every chunk has the same index pattern; keep one shared CPU copy because
    // raylib's DrawMesh only draws indexed when mesh.indices is non-NULL.
    static unsigned short *shared_idx;
    if (!shared_idx) {
        shared_idx = malloc(sizeof(unsigned short) * n->nidx);
        memcpy(shared_idx, n->idx, sizeof(unsigned short) * n->nidx);
    }
    Mesh m = { 0 };
    m.vertexCount = n->nverts;
    m.triangleCount = n->nidx / 3;
    m.vertices = n->pos;
    m.normals = n->nrm;
    m.colors = n->col;
    m.indices = shared_idx;
    UploadMesh(&m, false);
    // Keep only GPU copies of the vertex data.
    free(n->pos); free(n->nrm); free(n->col); free(n->idx);
    m.vertices = NULL; m.normals = NULL; m.colors = NULL;
    n->pos = NULL; n->nrm = NULL; n->col = NULL; n->idx = NULL;
    n->mesh = m;
    n->state = 3;
}

void terrain_init(V3 site_fixed)
{
    R0 = v3_len(site_fixed);
    site_dir = v3_norm(site_fixed);
    up = site_dir;
    V3 z = v3(0, 0, 1);
    east = v3_norm(v3_cross(z, up));
    north = v3_cross(up, east);
    root = new_node(0, -ROOT_SIZE / 2, -ROOT_SIZE / 2, ROOT_SIZE, NULL);
    double el = asin(sun_site.y);
    sun_slope = tan(el);
    double hl = sqrt(sun_site.x * sun_site.x + sun_site.z * sun_site.z);
    sun_ax = sun_site.x / hl;
    sun_az = sun_site.z / hl;
    quitting = 0;
    long cores = 8;
#ifdef _SC_NPROCESSORS_ONLN
    cores = sysconf(_SC_NPROCESSORS_ONLN);
#endif
    nworkers = (int)(cores > 2 ? cores - 1 : 1);
    if (nworkers > 15) nworkers = 15;
    for (int i = 0; i < nworkers; i++)
        pthread_create(&workers[i], NULL, worker, NULL);
}

static void free_node(Node *n)
{
    if (!n) return;
    for (int i = 0; i < 4; i++) free_node(n->child[i]);
    if (n->state == 3) {
        n->mesh.indices = NULL;   // shared, not owned by this mesh
        UnloadMesh(n->mesh);
    }
    free(n->pos); free(n->nrm); free(n->col); free(n->idx);
    free(n);
    node_count--;
}

void terrain_shutdown(void)
{
    pthread_mutex_lock(&q_mx);
    quitting = 1;
    pthread_cond_broadcast(&q_cv);
    pthread_mutex_unlock(&q_mx);
    for (int i = 0; i < nworkers; i++) pthread_join(workers[i], NULL);
    free_node(root);
    root = NULL;
}

// ---------------------------------------------------------------------------
// LOD selection.

#define MAX_DRAW 4096
static Node *draw_list[MAX_DRAW];
static int draw_n;
static double lod_k;
static double cam_x, cam_z, cam_h;

static double node_distance(Node *n)
{
    double dx = fmax(fmax(n->x0 - cam_x, 0), cam_x - (n->x0 + n->size));
    double dz = fmax(fmax(n->z0 - cam_z, 0), cam_z - (n->z0 + n->size));
    double dy = 0;
    if (n->state >= 2) {
        if (cam_h > n->hmax) dy = cam_h - n->hmax;
        else if (cam_h < n->hmin) dy = n->hmin - cam_h;
    } else {
        dy = fabs(cam_h);
    }
    return sqrt(dx * dx + dz * dz + dy * dy);
}

// Children ready to draw (state 3)?
static int children_ready(Node *n)
{
    for (int i = 0; i < 4; i++)
        if (!n->child[i] || n->child[i]->state != 3) return 0;
    return 1;
}

// Horizon culling: skip nodes well beyond the camera's horizon.
static int beyond_horizon(Node *n)
{
    double d = node_distance(n);
    double h = fmax(cam_h, 10.0) + 2000.0;
    double horizon = sqrt(2 * R0 * h) + sqrt(2 * R0 * 6000.0);
    return d > horizon * 1.05;
}

static void select_lod(Node *n)
{
    n->last_used = frame_no;
    if (beyond_horizon(n)) return;
    if (n->state == 0) enqueue(n);
    double d = node_distance(n);
    int want_split = n->level < MAX_LEVEL && d < n->size * lod_k;
    if (want_split) {
        for (int i = 0; i < 4; i++)
            if (!n->child[i] && node_count < MAX_NODES) {
                double hs = n->size / 2;
                n->child[i] = new_node(n->level + 1, n->x0 + (i & 1) * hs, n->z0 + (i >> 1) * hs, hs, n);
            }
        int all = 1;
        for (int i = 0; i < 4; i++) if (!n->child[i]) all = 0;
        if (all && children_ready(n)) {
            for (int i = 0; i < 4; i++) select_lod(n->child[i]);
            return;
        }
        // Make sure the children get built; draw ourselves meanwhile.
        for (int i = 0; i < 4; i++)
            if (n->child[i]) {
                n->child[i]->last_used = frame_no;
                if (n->child[i]->state == 0) enqueue(n->child[i]);
            }
    }
    if (n->state == 3 && draw_n < MAX_DRAW) draw_list[draw_n++] = n;
}

// Free subtrees not used for a while.
static void prune(Node *n)
{
    for (int i = 0; i < 4; i++) {
        Node *c = n->child[i];
        if (!c) continue;
        if (frame_no - c->last_used > 240 && c->state != 1) {
            pthread_mutex_lock(&q_mx);
            int queued = c->state == 1;
            pthread_mutex_unlock(&q_mx);
            if (!queued) {
                // Only free if no descendant is in the queue.
                int safe = 1;
                Node *stack[256];
                int sp = 0;
                stack[sp++] = c;
                while (sp && safe) {
                    Node *x = stack[--sp];
                    if (x->state == 1) safe = 0;
                    for (int k = 0; k < 4; k++) if (x->child[k] && sp < 255) stack[sp++] = x->child[k];
                }
                if (safe) {
                    free_node(c);
                    n->child[i] = NULL;
                    continue;
                }
            }
        }
        prune(c);
    }
}

void terrain_update(V3 camera_site, double fov_y, int screen_h)
{
    frame_no++;
    terrain_site_to_plane(camera_site, &cam_x, &cam_z);
    // Camera height above the reference sphere.
    V3 p = v3_add(v3_scale(site_dir, R0), v3_add(v3_scale(east, camera_site.x),
                  v3_add(v3_scale(up, camera_site.y), v3_scale(north, -camera_site.z))));
    cam_h = v3_len(p) - R0;
    // Split when the chunk is closer than k * size; k from screen resolution.
    double pix_per_rad = screen_h / fov_y;
    lod_k = 2.2 * pix_per_rad / 900.0 + 0.8;

    // Upload finished chunks (bounded per frame).
    int uploads = 0;
    Node *stack[512];
    int sp = 0;
    stack[sp++] = root;
    while (sp) {
        Node *n = stack[--sp];
        if (n->state == 2 && uploads < 48) {
            upload(n);
            uploads++;
        }
        for (int i = 0; i < 4; i++) if (n->child[i] && sp < 511) stack[sp++] = n->child[i];
    }

    draw_n = 0;
    select_lod(root);
    if ((frame_no & 63) == 0) prune(root);
}

void terrain_draw(V3 cam, void *shader_ptr)
{
    Shader *shader = shader_ptr;
    Material mat = LoadMaterialDefault();
    if (!getenv("AGC_DEFSH")) mat.shader = *shader;
    drawn_count = 0;
    for (int i = 0; i < draw_n; i++) {
        Node *n = draw_list[i];
        V3 o = v3_sub(n->origin, cam);
        Matrix t = MatrixTranslate((float)o.x, (float)o.y, (float)o.z);
        DrawMesh(n->mesh, mat, t);
        drawn_count++;
    }
    RL_FREE(mat.maps);
}

void terrain_stats(int *drawn, int *loaded, int *pending)
{
    *drawn = drawn_count;
    *loaded = node_count;
    pthread_mutex_lock(&q_mx);
    *pending = (q_tail - q_head + QCAP) % QCAP + busy;
    pthread_mutex_unlock(&q_mx);
}

// Debug probe.
float terrain_debug_shadow(double x, double z, double spacing)
{
    double h0 = terrain_height(x, z, spacing * 1.5);
    return shadow_at(x, z, h0, spacing);
}

// Debug: build one chunk on the CPU and report geometry statistics.
void terrain_debug_chunk(double x0, double z0, double size)
{
    Node n = { 0 };
    n.x0 = x0; n.z0 = z0; n.size = size;
    build_chunk(&n);
    int degenerate = 0;
    double amin = 1e30, amax = 0;
    for (int t = 0; t < n.nidx / 3; t++) {
        float *a = &n.pos[3 * n.idx[3 * t]], *b = &n.pos[3 * n.idx[3 * t + 1]], *c = &n.pos[3 * n.idx[3 * t + 2]];
        V3 ab = v3(b[0] - a[0], b[1] - a[1], b[2] - a[2]), ac = v3(c[0] - a[0], c[1] - a[1], c[2] - a[2]);
        V3 cr = v3_cross(ab, ac);
        double area = v3_len(cr) / 2;
        if (area < 1e-6) degenerate++;
        if (t < GRID * GRID * 2) { if (area < amin) amin = area; if (area > amax) amax = area; }
        if (t < 2) printf("tri %d: (%.2f %.2f %.2f) (%.2f %.2f %.2f) (%.2f %.2f %.2f) normal y %.3f\n", t,
                          a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], cr.y / (2 * area));
    }
    printf("chunk %.0f m: %d verts, %d tris, %d degenerate, grid tri area %.3g..%.3g, origin (%.1f %.1f %.1f)\n",
           size, n.nverts, n.nidx / 3, degenerate, amin, amax, n.origin.x, n.origin.y, n.origin.z);
}
