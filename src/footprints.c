// Boot prints: see footprints.h.
//
// Each print is a small grid laid on the drawn terrain surface
// (terrain_ground_height) and lifted a little off it; the shape of the hollow
// is worked out per pixel by the decal shader from the quad's coordinates.

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"

#include "footprints.h"
#include "render.h"
#include "terrain.h"

#define MAXP 900            // oldest prints are reused after this many
#define NU 3                // grid vertices across
#define NV 5                // ... and along
#define VPP ((NU - 1) * (NV - 1) * 6)   // vertices per print (plain triangles)
#define HALF_W 0.11         // quad half-size, m (the shader's uv scale)
#define HALF_L 0.21
#define LIFT 0.022          // above the drawn surface, m

typedef struct { double x, z, yaw; float depth; } Print;
static Print prints[MAXP];
static int nprints, next_slot, dirty;
static Mesh mesh;
static int have_mesh;
static Material mat;
static int have_mat;

void footprints_reset(void)
{
    nprints = next_slot = 0;
    dirty = 1;
}

int footprints_count(void) { return nprints; }

void footprints_add(double x, double z, double yaw, float depth)
{
    prints[next_slot] = (Print){ x, z, yaw, depth < 0 ? 0 : depth > 1 ? 1 : depth };
    next_slot = (next_slot + 1) % MAXP;
    if (nprints < MAXP) nprints++;
    dirty = 1;
}

static void rebuild(void)
{
    if (have_mesh) {
        // The arrays belong to this module: detach them before raylib frees the GPU side.
        mesh.vertices = NULL; mesh.normals = NULL; mesh.tangents = NULL; mesh.texcoords = NULL; mesh.colors = NULL;
        UnloadMesh(mesh);
        have_mesh = 0;
    }
    if (!nprints) return;
    static float *pos, *nrm, *tan, *uv;
    static unsigned char *col;
    static int cap;
    int nv = nprints * VPP;
    if (nv > cap) {
        cap = MAXP * VPP;
        pos = realloc(pos, sizeof(float) * 3 * cap);
        nrm = realloc(nrm, sizeof(float) * 3 * cap);
        tan = realloc(tan, sizeof(float) * 4 * cap);
        uv = realloc(uv, sizeof(float) * 2 * cap);
        col = realloc(col, 4 * cap);
    }
    int k = 0;
    for (int p = 0; p < nprints; p++) {
        const Print *pr = &prints[p];
        // Boot axes on the plane: forward along the heading, right across it.
        double fx = sin(pr->yaw), fz = -cos(pr->yaw), rx = cos(pr->yaw), rz = sin(pr->yaw);
        V3 g[NU][NV];
        float gu[NU][NV], gv[NU][NV];
        for (int i = 0; i < NU; i++)
            for (int j = 0; j < NV; j++) {
                double u = -1 + 2.0 * i / (NU - 1), v = -1 + 2.0 * j / (NV - 1);
                double x = pr->x + rx * u * HALF_W + fx * v * HALF_L, z = pr->z + rz * u * HALF_W + fz * v * HALF_L;
                g[i][j] = terrain_plane_to_site(x, z, terrain_ground_height(x, z) + LIFT);
                gu[i][j] = (float)u;
                gv[i][j] = (float)v;
            }
        // The ground's normal and the boot's forward, site frame.
        V3 c = terrain_plane_to_site(pr->x, pr->z, terrain_ground_height(pr->x, pr->z));
        V3 e = v3_sub(terrain_plane_to_site(pr->x + 0.3, pr->z, terrain_ground_height(pr->x + 0.3, pr->z)), c);
        V3 s = v3_sub(terrain_plane_to_site(pr->x, pr->z + 0.3, terrain_ground_height(pr->x, pr->z + 0.3)), c);
        V3 n = v3_norm(v3_cross(s, e));
        V3 f = v3_norm(v3_sub(terrain_plane_to_site(pr->x + fx, pr->z + fz, 0), terrain_plane_to_site(pr->x, pr->z, 0)));
        for (int i = 0; i + 1 < NU; i++)
            for (int j = 0; j + 1 < NV; j++) {
                static const int tri[6][2] = { { 0, 0 }, { 1, 1 }, { 0, 1 }, { 0, 0 }, { 1, 0 }, { 1, 1 } };   // facing up
                for (int t = 0; t < 6; t++) {
                    int a = i + tri[t][0], b = j + tri[t][1];
                    pos[3 * k] = (float)g[a][b].x; pos[3 * k + 1] = (float)g[a][b].y; pos[3 * k + 2] = (float)g[a][b].z;
                    nrm[3 * k] = (float)n.x; nrm[3 * k + 1] = (float)n.y; nrm[3 * k + 2] = (float)n.z;
                    tan[4 * k] = (float)f.x; tan[4 * k + 1] = (float)f.y; tan[4 * k + 2] = (float)f.z; tan[4 * k + 3] = 1;
                    uv[2 * k] = gu[a][b]; uv[2 * k + 1] = gv[a][b];
                    col[4 * k] = (unsigned char)(255 * pr->depth); col[4 * k + 1] = 0; col[4 * k + 2] = 0; col[4 * k + 3] = 255;
                    k++;
                }
            }
    }
    memset(&mesh, 0, sizeof mesh);
    mesh.vertexCount = k;
    mesh.triangleCount = k / 3;
    mesh.vertices = pos;
    mesh.normals = nrm;
    mesh.tangents = tan;
    mesh.texcoords = uv;
    mesh.colors = col;
    UploadMesh(&mesh, false);
    have_mesh = 1;
}

void footprints_draw(V3 cam_pos)
{
    if (dirty) { rebuild(); dirty = 0; }
    if (!have_mesh) return;
    if (!have_mat) {
        mat = LoadMaterialDefault();
        mat.shader = *render_decal_shader();
        have_mat = 1;
    }
    rlDrawRenderBatchActive();
    // The decal multiplies the colour already there; alpha is left alone.
    rlSetBlendFactorsSeparate(RL_DST_COLOR, RL_ZERO, RL_ZERO, RL_ONE, RL_FUNC_ADD, RL_FUNC_ADD);
    BeginBlendMode(BLEND_CUSTOM_SEPARATE);
    rlDisableDepthMask();
    Matrix m = MatrixTranslate((float)-cam_pos.x, (float)-cam_pos.y, (float)-cam_pos.z);
    DrawMesh(mesh, mat, m);
    rlDrawRenderBatchActive();
    rlEnableDepthMask();
    EndBlendMode();
}
