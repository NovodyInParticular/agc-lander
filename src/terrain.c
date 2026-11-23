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

#include "input.h"

#include "moon.h"
#include "terrain.h"
#include "terrain_shape.h"
#include "render.h"

// Copies of the shape's frame (terrain_shape.c), taken at terrain_init.
static V3 site_dir, east, north, up;
static double R0;
static V3 sun_site = { 0.98, 0.19, 0.0 };

void terrain_set_sun(V3 s) { sun_site = v3_norm(s); }
V3 terrain_sun(void) { return sun_site; }

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
    float *tan, *tc, *tc2;            // morph target: the parent chunk's surface at each vertex
    unsigned char *col;
    unsigned short *idx;
    int nverts, nidx;
    V3 origin;                        // site-frame chunk origin
    V3 bctr;                          // bounding sphere (site frame), for culling
    double brad;
    Mesh mesh;
    unsigned last_used;
    float fade;                       // 0 = just appeared (shown as the parent's surface), 1 = fully itself
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

// Surface point of grid vertex (i, j); indices outside the chunk are evaluated on demand.
static V3 grid_point(const Node *n, const V3 *p3, int i, int j, double spacing, double mf)
{
    if (i >= 0 && i <= GRID && j >= 0 && j <= GRID) return p3[j * (GRID + 1) + i];
    double x = n->x0 + i * spacing, z = n->z0 + j * spacing;
    TerrainSample s;
    terrain_eval(x, z, mf, &s);
    return terrain_plane_to_site(x, z, s.h);
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
            TerrainSample s;
            terrain_eval(x, z, mf, &s);
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
            // Central differences everywhere: border vertices sample one step
            // outside the chunk, so neighbouring chunks agree on the edge normal
            // and no lighting seam shows between them.
            V3 dx = v3_sub(grid_point(n, p3, i + 1, j, spacing, mf), grid_point(n, p3, i - 1, j, spacing, mf));
            V3 dz = v3_sub(grid_point(n, p3, i, j + 1, spacing, mf), grid_point(n, p3, i, j - 1, spacing, mf));
            V3 nn = v3_norm(v3_cross(dz, dx));
            n->nrm[3 * k] = (float)nn.x;
            n->nrm[3 * k + 1] = (float)nn.y;
            n->nrm[3 * k + 2] = (float)nn.z;
            n->col[4 * k] = (unsigned char)fminf(255.0f, alb[k] * 1000.0f);
            n->col[4 * k + 1] = (unsigned char)(sh[k] * 255.0f);
            n->col[4 * k + 2] = (unsigned char)(fr[k] * 255.0f);
            n->col[4 * k + 3] = 255;
        }

    // Morph target. The even vertices of this grid are the parent chunk's
    // vertices; evaluate the surface there exactly as the parent does (its
    // coarser detail floor, its shadow step), and interpolate the odd vertices
    // along the parent's triangles. The vertex shader blends every vertex
    // toward this target with distance, so a chunk swap changes nothing on
    // screen: detail grows in and melts away instead of popping.
    {
        const int HG = GRID / 2, HW = HG + 3;   // coarse grid with a one-step apron for the normals
        double cs = spacing * 2, cmf = cs * 1.5;
        V3 *cp = malloc(sizeof(V3) * HW * HW), *cn = malloc(sizeof(V3) * (HG + 1) * (HG + 1));
        float *ca = malloc(sizeof(float) * 3 * (HG + 1) * (HG + 1));   // albedo, shadow, fresh
        for (int J = -1; J <= HG + 1; J++)
            for (int I = -1; I <= HG + 1; I++) {
                double x = n->x0 + I * cs, z = n->z0 + J * cs;
                TerrainSample sc;
                terrain_eval(x, z, cmf, &sc);
                cp[(J + 1) * HW + I + 1] = terrain_plane_to_site(x, z, sc.h);
                if (I < 0 || J < 0 || I > HG || J > HG) continue;
                float *a = &ca[3 * (J * (HG + 1) + I)];
                terrain_surface_props(x, z, cmf, &a[0], &a[2]);
                a[1] = cs > 50 ? sh[(2 * J) * (GRID + 1) + 2 * I] : shadow_at(x, z, sc.h, cs);
            }
        for (int J = 0; J <= HG; J++)
            for (int I = 0; I <= HG; I++) {
                V3 dx = v3_sub(cp[(J + 1) * HW + I + 2], cp[(J + 1) * HW + I]);
                V3 dz = v3_sub(cp[(J + 2) * HW + I + 1], cp[J * HW + I + 1]);
                cn[J * (HG + 1) + I] = v3_norm(v3_cross(dz, dx));
            }
        n->tan = malloc(sizeof(float) * 4 * nv);
        n->tc = malloc(sizeof(float) * 2 * nv);
        n->tc2 = malloc(sizeof(float) * 2 * nv);
        for (int j = 0; j <= GRID; j++)
            for (int i = 0; i <= GRID; i++) {
                int k = j * (GRID + 1) + i;
                // The one or two coarse vertices this vertex sits on / between.
                int I0 = i / 2, J0 = j / 2, I1 = I0, J1 = J0;
                if ((i & 1) && (j & 1)) {
                    if ((I0 ^ J0) & 1) { I0 += 1; J1 += 1; }   // parent diagonal (I+1,J)-(I,J+1)
                    else { I1 += 1; J1 += 1; }                 // parent diagonal (I,J)-(I+1,J+1)
                } else if (i & 1) I1 += 1;
                else if (j & 1) J1 += 1;
                int ka = J0 * (HG + 1) + I0, kb = J1 * (HG + 1) + I1;
                V3 pc = v3_scale(v3_add(cp[(J0 + 1) * HW + I0 + 1], cp[(J1 + 1) * HW + I1 + 1]), 0.5);
                V3 nc = v3_norm(v3_add(cn[ka], cn[kb]));
                V3 d = v3_sub(pc, p3[k]);
                n->tan[4 * k] = (float)d.x;
                n->tan[4 * k + 1] = (float)d.y;
                n->tan[4 * k + 2] = (float)d.z;
                n->tan[4 * k + 3] = 0.5f * (ca[3 * ka + 1] + ca[3 * kb + 1]);
                n->tc[2 * k] = (float)nc.x;
                n->tc[2 * k + 1] = (float)nc.z;
                n->tc2[2 * k] = fminf(1.0f, 0.5f * (ca[3 * ka] + ca[3 * kb]) * 1000.0f / 255.0f);
                n->tc2[2 * k + 1] = 0.5f * (ca[3 * ka + 2] + ca[3 * kb + 2]);
            }
        free(cp);
        free(cn);
        free(ca);
    }

    // Skirts: copies of the border vertices, pushed down, to hide LOD cracks.
    double drop = spacing * 1.5 + 2.0;
    int sk = NV;
    for (int e = 0; e < 4; e++)
        for (int t = 0; t <= GRID; t++) {
            int i = e == 0 ? t : e == 1 ? GRID : e == 2 ? GRID - t : 0;
            int j = e == 0 ? 0 : e == 1 ? t : e == 2 ? GRID : GRID - t;
            int k = j * (GRID + 1) + i;
            V3 dn = terrain_fixed_dir_to_site(terrain_plane_to_dir(n->x0 + i * spacing, n->z0 + j * spacing));
            n->pos[3 * sk] = n->pos[3 * k] - (float)(dn.x * drop);
            n->pos[3 * sk + 1] = n->pos[3 * k + 1] - (float)(dn.y * drop);
            n->pos[3 * sk + 2] = n->pos[3 * k + 2] - (float)(dn.z * drop);
            memcpy(&n->nrm[3 * sk], &n->nrm[3 * k], sizeof(float) * 3);
            memcpy(&n->col[4 * sk], &n->col[4 * k], 4);
            memcpy(&n->tan[4 * sk], &n->tan[4 * k], sizeof(float) * 4);
            memcpy(&n->tc[2 * sk], &n->tc[2 * k], sizeof(float) * 2);
            memcpy(&n->tc2[2 * sk], &n->tc2[2 * k], sizeof(float) * 2);
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
    m.tangents = n->tan;
    m.texcoords = n->tc;
    m.texcoords2 = n->tc2;
    m.indices = shared_idx;
    UploadMesh(&m, false);
    // Keep only GPU copies of the vertex data.
    free(n->pos); free(n->nrm); free(n->col); free(n->idx); free(n->tan); free(n->tc); free(n->tc2);
    m.vertices = NULL; m.normals = NULL; m.colors = NULL; m.tangents = NULL; m.texcoords = NULL; m.texcoords2 = NULL;
    n->pos = NULL; n->nrm = NULL; n->col = NULL; n->idx = NULL; n->tan = NULL; n->tc = NULL; n->tc2 = NULL;
    n->mesh = m;
    n->state = 3;
}

void terrain_init(V3 site_fixed)
{
    terrain_shape_init(site_fixed);
    terrain_frame(&site_dir, &east, &north, &up);
    R0 = terrain_ref_radius();
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
#ifdef __ANDROID__
    nworkers = 2;   // headset: leave the fast cores to the simulation and the renderer
#endif
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
    free(n->pos); free(n->nrm); free(n->col); free(n->idx); free(n->tan); free(n->tc); free(n->tc2);
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
// After a camera jump (start, camera cut, teleport) chunks appear at once;
// otherwise each new chunk grows out of its parent's surface over FADE_TIME.
#define FADE_TIME 0.6f
static int settling = 1, settle_frames;
static V3 last_cam;

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
        // Hand over to the children only once this chunk has finished growing
        // in itself, so each level blends out of the one before it.
        if (all && children_ready(n) && (settling || n->state != 3 || n->fade >= 1.0f)) {
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
    lod_k = (2.2 * pix_per_rad / 900.0 + 0.8) * render_quality()->lod;
    {
        double moved = v3_len(v3_sub(camera_site, last_cam));
        last_cam = camera_site;
        if (moved > fmax(400.0, 0.5 * fabs(cam_h))) { settling = 1; settle_frames = 0; }
        if (settling) {
            pthread_mutex_lock(&q_mx);
            int pending = (q_tail - q_head + QCAP) % QCAP + busy;
            pthread_mutex_unlock(&q_mx);
            if (++settle_frames > 20 && (pending == 0 || settle_frames > 900)) settling = 0;
        }
    }

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

// Headset: the terrain is drawn once per eye; the fade-in of new chunks must
// advance only once per frame (hold it for the first eye).
static int fade_hold;
void terrain_hold_fade(int hold) { fade_hold = hold; }

void terrain_draw(V3 cam, void *shader_ptr)
{
    Shader *shader = shader_ptr;
    Material mat = LoadMaterialDefault();
    if (!getenv("AGC_DEFSH")) mat.shader = *shader;
    drawn_count = 0;
    int morph_loc = GetShaderLocation(*shader, "morph");
    int chunk_loc = GetShaderLocation(*shader, "chunk");
    int base_loc = GetShaderLocation(*shader, "cellBase"), frac_loc = GetShaderLocation(*shader, "cellFrac");
    static int no_morph = -1;   // AGC_NOMORPH=1: old hard LOD switching, for comparison
    if (no_morph < 0) no_morph = getenv("AGC_NOMORPH") != NULL;
    float dt = GetFrameTime();
    if (dt > 0.1f) dt = 0.1f;
    if (fade_hold) dt = 0;
    static int no_cull = -1;   // AGC_NOCULL=1: draw every chunk (comparison)
    if (no_cull < 0) no_cull = getenv("AGC_NOCULL") != NULL;
    for (int i = 0; i < draw_n; i++) {
        Node *n = draw_list[i];
        // A chunk grows in over FADE_TIME whether or not this eye can see it.
        if (settling || no_morph) n->fade = 1.0f;
        n->fade += dt / FADE_TIME;
        if (n->fade > 1.0f) n->fade = 1.0f;
        if (n->brad <= 0) {
            // Bounding sphere: half-diagonal, height span, the sphere's bulge, skirts, morph slack.
            n->bctr = terrain_plane_to_site(n->x0 + n->size / 2, n->z0 + n->size / 2, (n->hmin + n->hmax) / 2);
            n->brad = n->size * 0.72 + (n->hmax - n->hmin) / 2 + n->size * n->size / (8 * R0) + n->size * 0.05 + 20.0;
        }
        if (!no_cull && !render_sphere_visible(v3_sub(n->bctr, cam), n->brad)) {
            if (getenv("AGC_CULLDBG")) {
                for (int k = 0; k < 50; k++) {
                    V3 q = terrain_plane_to_site(n->x0 + (k % 5) * 0.25 * n->size, n->z0 + ((k / 5) % 5) * 0.25 * n->size, (k >= 25) ? n->hmax : n->hmin);
                    if (render_sphere_visible(v3_sub(q, cam), 0.0)) {
                        printf("CULLBUG level %d size %.1f x0 %.1f z0 %.1f h %.2f..%.2f bctr (%.1f %.1f %.1f) rad %.1f corner (%.1f %.1f %.1f) cam (%.1f %.1f %.1f)\n",
                               n->level, n->size, n->x0, n->z0, n->hmin, n->hmax, n->bctr.x, n->bctr.y, n->bctr.z, n->brad, q.x, q.y, q.z, cam.x, cam.y, cam.z);
                        break;
                    }
                }
            }
            continue;
        }
        V3 o = v3_sub(n->origin, cam);
        Matrix t = MatrixTranslate((float)o.x, (float)o.y, (float)o.z);
        if (settling || no_morph) n->fade = 1.0f;
        // Blend toward the parent's surface across the far part of this
        // level's range (it hands over to the parent at 2 * size * k).
        float range = (float)(2.0 * n->size * lod_k);
        float morph[3] = { range * 0.72f, range * 0.95f, n->level == 0 ? 1.0f : n->fade };
        if (n->level == 0 || no_morph) morph[0] = morph[1] = 1e30f;
        SetShaderValue(*shader, morph_loc, morph, SHADER_UNIFORM_VEC3);
        // Where this chunk sits in each crater octave's cell grid (integer cell
        // + fraction, so the shader keeps full precision far from the site).
        float chunk[4] = { (float)(n->size / GRID), 0, 0, 0 };
        int base[2 * SHADER_NOCT];
        float frac[2 * SHADER_NOCT];
        for (int k = 0; k < SHADER_NOCT; k++) {
            double c = terrain_octave_cell(SHADER_OCT0 + k);
            double fx = floor(n->x0 / c), fz = floor(n->z0 / c);
            base[2 * k] = (int)fx;
            base[2 * k + 1] = (int)fz;
            frac[2 * k] = (float)(n->x0 / c - fx);
            frac[2 * k + 1] = (float)(n->z0 / c - fz);
        }
        SetShaderValue(*shader, chunk_loc, chunk, SHADER_UNIFORM_VEC4);
        SetShaderValueV(*shader, base_loc, base, SHADER_UNIFORM_IVEC2, SHADER_NOCT);
        SetShaderValueV(*shader, frac_loc, frac, SHADER_UNIFORM_VEC2, SHADER_NOCT);
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

// The finest drawn mesh's surface (1 m grid, features under 1.5 m left out,
// alternating diagonals as in build_chunk): what the eye sees underfoot.
double terrain_ground_height(double x, double z)
{
    double gx = floor(x), gz = floor(z), fx = x - gx, fz = z - gz;
    const double mf = 1.5;
    double ha = terrain_height(gx, gz, mf), hb = terrain_height(gx + 1, gz, mf);
    double hc = terrain_height(gx, gz + 1, mf), hd = terrain_height(gx + 1, gz + 1, mf);
    long ix = (long)gx, iz = (long)gz;
    if ((ix ^ iz) & 1) {   // diagonal b-c
        if (fx + fz <= 1) return ha + (hb - ha) * fx + (hc - ha) * fz;
        return hd + (hc - hd) * (1 - fx) + (hb - hd) * (1 - fz);
    }
    if (fx >= fz) return ha + (hb - ha) * fx + (hd - hb) * fz;   // diagonal a-d
    return ha + (hd - hc) * fx + (hc - ha) * fz;
}
