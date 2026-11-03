// Lunar Module cabin interior.
//
// Geometry is authored in LM body axes (metres, origin at the CG, +X up,
// +Y right, +Z forward), matching lmmodel.c: the two triangular forward
// windows here are the apertures behind the exterior model's windows.
// Panels are textured from a live atlas (cockpit_panels.c). Lighting:
// sunlight through the windows (with a cabin shadow map, so sun patches
// move across the panels as the LM turns), light bounced in from the lunar
// surface through each window, the cabin floodlights, and the panels'
// own integral/annunciator lighting.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "cockpit.h"
#include "cockpit_int.h"
#include "hud.h"
#include "lm.h"
#include "render.h"

#define DEG (PI_D / 180.0)

// --- Panel placement (body frame, metres) -----------------------------------------

PanelDef cp_panels[NPANELS] = {
    [P1] = { "PANEL 1", { 2.75, -0.47, 0.70 }, { 2.75, -0.01, 0.62 }, { 2.15, -0.47, 0.48 }, 2.2f },
    [P2] = { "PANEL 2", { 2.75, 0.01, 0.62 }, { 2.75, 0.47, 0.70 }, { 2.15, 0.01, 0.40 }, 2.2f },
    [P3] = { "PANEL 3", { 2.02, -0.64, 0.62 }, { 2.02, 0.64, 0.62 }, { 1.82, -0.64, 0.42 }, 1.9f },
    [P4] = { "PANEL 4", { 1.82, -0.22, 0.42 }, { 1.82, 0.22, 0.42 }, { 1.74, -0.22, 0.16 }, 2.4f },
    [P5] = { "PANEL 5", { 2.02, -1.02, 0.62 }, { 2.02, -0.66, 0.62 }, { 1.82, -1.02, 0.42 }, 2.0f },
    [P6] = { "PANEL 6", { 2.02, 0.66, 0.62 }, { 2.02, 1.02, 0.62 }, { 1.82, 0.66, 0.42 }, 2.0f },
    [P8] = { "PANEL 8", { 2.00, -1.035, -0.40 }, { 2.00, -1.035, 0.40 }, { 1.90, -0.84, -0.40 }, 1.6f },
    [P11] = { "PANEL 11", { 3.25, -0.96, -0.80 }, { 3.25, -0.96, 0.35 }, { 2.70, -1.035, -0.80 }, 1.3f },
    [P12] = { "PANEL 12", { 2.00, 1.035, 0.40 }, { 2.00, 1.035, -0.40 }, { 1.90, 0.84, 0.40 }, 1.6f },
    [P14] = { "PANEL 14", { 2.62, 1.035, 0.40 }, { 2.62, 1.035, -0.20 }, { 2.08, 1.035, 0.40 }, 1.6f },
    [P16] = { "PANEL 16", { 3.25, 0.96, 0.35 }, { 3.25, 0.96, -0.80 }, { 2.70, 1.035, 0.35 }, 1.3f },
};

// Forward wall: exterior plane E(y, s) = (2.00 + s*SX, y, 1.05 - s*SZ) with s
// measured up the slant; the cabin wall is the same plane moved inward.
#define SX 0.93144
#define SZ 0.36410
#define WALL_T 0.0466
static const V3 N_IN = { -SZ, 0, -SX };      // into the cabin
#define WIN_S0 0.0300
#define WIN_S1 1.1000
#define WIN_YIN 0.46
#define WIN_YOUT 1.00

static V3 ext_pt(double y, double s) { return v3(2.00 + s * SX, y, 1.05 - s * SZ); }
static V3 int_pt(double y, double s) { return v3_add(ext_pt(y, s), v3_scale(N_IN, WALL_T)); }
static V3 pane_pt(double y, double s, double depth) { return v3_add(ext_pt(y, s), v3_scale(N_IN, depth)); }

// Design eye positions (the LPD is calibrated for the CDR's).
static const V3 EYE_CDR = { 2.84, -0.66, 0.30 };
static const V3 EYE_LMP = { 2.84, 0.66, 0.30 };
#define CABIN_FLOOR 1.38
#define CABIN_CEIL 3.32
#define CABIN_HALF 1.04
#define CABIN_AFT (-1.25)
// Docking (overhead rendezvous) window above the CDR.
#define DW_Y0 (-0.80)
#define DW_Y1 (-0.52)
#define DW_Z0 (-0.35)
#define DW_Z1 (-0.05)

// --- Mesh builder --------------------------------------------------------------------

enum { M_PANEL = 1, M_PAINT, M_METAL, M_BLACK, M_EMIT, M_ETCH, M_GLASS };

typedef struct {
    float *p, *n, *t;
    unsigned char *c;
    int count, cap;
} Builder;

static Builder SB, DB, TB;   // static opaque, dynamic (levers etc.), transparent
static Builder *B = &SB;
static float white_u, white_v;   // atlas UV of a white texel

static void bpush(V3 p, V3 n, float u, float v, Color c, int mat)
{
    if (B->count == B->cap) {
        B->cap = B->cap ? B->cap * 2 : 8192;
        B->p = realloc(B->p, sizeof(float) * 3 * B->cap);
        B->n = realloc(B->n, sizeof(float) * 3 * B->cap);
        B->t = realloc(B->t, sizeof(float) * 2 * B->cap);
        B->c = realloc(B->c, 4 * B->cap);
    }
    int k = B->count++;
    B->p[3 * k] = (float)p.x; B->p[3 * k + 1] = (float)p.y; B->p[3 * k + 2] = (float)p.z;
    B->n[3 * k] = (float)n.x; B->n[3 * k + 1] = (float)n.y; B->n[3 * k + 2] = (float)n.z;
    B->t[2 * k] = u; B->t[2 * k + 1] = v;
    B->c[4 * k] = c.r; B->c[4 * k + 1] = c.g; B->c[4 * k + 2] = c.b; B->c[4 * k + 3] = (unsigned char)mat;
}

static void tri(V3 a, V3 b, V3 c, Color col, int mat)
{
    V3 n = v3_norm(v3_cross(v3_sub(b, a), v3_sub(c, a)));
    bpush(a, n, white_u, white_v, col, mat);
    bpush(b, n, white_u, white_v, col, mat);
    bpush(c, n, white_u, white_v, col, mat);
}

static void quad(V3 a, V3 b, V3 c, V3 d, Color col, int mat)
{
    tri(a, b, c, col, mat);
    tri(a, c, d, col, mat);
}

// Textured quad: corners a (uv0) b (uv1) c (uv2) d (uv3).
static void quad_uv(V3 a, V3 b, V3 c, V3 d, Vector2 ta, Vector2 tb, Vector2 tc, Vector2 td, Color col, int mat)
{
    V3 n = v3_norm(v3_cross(v3_sub(b, a), v3_sub(c, a)));
    bpush(a, n, ta.x, ta.y, col, mat); bpush(b, n, tb.x, tb.y, col, mat); bpush(c, n, tc.x, tc.y, col, mat);
    bpush(a, n, ta.x, ta.y, col, mat); bpush(c, n, tc.x, tc.y, col, mat); bpush(d, n, td.x, td.y, col, mat);
}

static void box(V3 lo, V3 hi, Color col, int mat)
{
    V3 p[8];
    for (int i = 0; i < 8; i++) p[i] = v3(i & 1 ? hi.x : lo.x, i & 2 ? hi.y : lo.y, i & 4 ? hi.z : lo.z);
    quad(p[0], p[2], p[3], p[1], col, mat); quad(p[4], p[5], p[7], p[6], col, mat);
    quad(p[0], p[1], p[5], p[4], col, mat); quad(p[2], p[6], p[7], p[3], col, mat);
    quad(p[0], p[4], p[6], p[2], col, mat); quad(p[1], p[3], p[7], p[5], col, mat);
}

// Oriented box: centre, half extents along unit axes a, b, c.
static void obox(V3 ctr, V3 a, V3 b, V3 c, double ha, double hb, double hc, Color col, int mat)
{
    V3 p[8];
    for (int i = 0; i < 8; i++)
        p[i] = v3_add(ctr, v3_add(v3_scale(a, i & 1 ? ha : -ha), v3_add(v3_scale(b, i & 2 ? hb : -hb), v3_scale(c, i & 4 ? hc : -hc))));
    quad(p[0], p[2], p[3], p[1], col, mat); quad(p[4], p[5], p[7], p[6], col, mat);
    quad(p[0], p[1], p[5], p[4], col, mat); quad(p[2], p[6], p[7], p[3], col, mat);
    quad(p[0], p[4], p[6], p[2], col, mat); quad(p[1], p[3], p[7], p[5], col, mat);
}

static void tube(V3 a, V3 b, double ra, double rb, int seg, Color col, int mat, int caps)
{
    V3 ax = v3_norm(v3_sub(b, a));
    V3 t = fabs(ax.x) < 0.9 ? v3(1, 0, 0) : v3(0, 1, 0);
    V3 u = v3_norm(v3_cross(ax, t)), w = v3_cross(ax, u);
    for (int i = 0; i < seg; i++) {
        double t0 = 2 * PI_D * i / seg, t1 = 2 * PI_D * (i + 1) / seg;
        V3 d0 = v3_add(v3_scale(u, cos(t0)), v3_scale(w, sin(t0))), d1 = v3_add(v3_scale(u, cos(t1)), v3_scale(w, sin(t1)));
        V3 a0 = v3_add(a, v3_scale(d0, ra)), a1 = v3_add(a, v3_scale(d1, ra));
        V3 b0 = v3_add(b, v3_scale(d0, rb)), b1 = v3_add(b, v3_scale(d1, rb));
        bpush(a0, d0, white_u, white_v, col, mat); bpush(a1, d1, white_u, white_v, col, mat); bpush(b1, d1, white_u, white_v, col, mat);
        bpush(a0, d0, white_u, white_v, col, mat); bpush(b1, d1, white_u, white_v, col, mat); bpush(b0, d0, white_u, white_v, col, mat);
        if (caps) {
            tri(a, a1, a0, col, mat);
            tri(b, b0, b1, col, mat);
        }
    }
}

// --- Colours ---------------------------------------------------------------------------

static const Color K_WALL = { 168, 166, 158, 255 };     // cabin structure paint
static const Color K_PANEL = { 255, 255, 255, 255 };
static const Color K_CONSOLE = { 80, 84, 88, 255 };
static const Color K_FRAME = { 30, 31, 33, 255 };
static const Color K_FLOOR = { 104, 104, 100, 255 };
static const Color K_CLOTH = { 222, 220, 210, 255 };    // Beta cloth
static const Color K_BLACK = { 22, 22, 24, 255 };
static const Color K_METAL = { 170, 172, 176, 255 };
static const Color K_BLUE = { 40, 80, 170, 255 };
static const Color K_RED = { 170, 40, 36, 255 };

// --- Atlas ------------------------------------------------------------------------------

#define ATLAS 4096
static RenderTexture2D atlas_rt, emit_rt;
static int lpd_ax, lpd_ay, white_ax, white_ay;
#define LPD_CELL_W 96
#define LPD_CELL_H 48

static Vector2 atlas_uv(int panel, float u_mm, float v_mm)
{
    const PanelDef *p = &cp_panels[panel];
    float px = p->ax + u_mm / p->w_mm * p->aw, py = p->ay + v_mm / p->h_mm * p->ah;
    return (Vector2){ px / ATLAS, 1.0f - py / ATLAS };   // render textures are stored bottom-up
}

static void pack_atlas(void)
{
    for (float scale = 1.0f; scale > 0.3f; scale *= 0.92f) {
        int order[NPANELS + 2];
        int w[NPANELS + 2], h[NPANELS + 2];
        int n = NPANELS + 2;
        for (int i = 0; i < NPANELS; i++) {
            PanelDef *p = &cp_panels[i];
            w[i] = (int)ceilf(p->w_mm * p->dens * scale) + 2;
            h[i] = (int)ceilf(p->h_mm * p->dens * scale) + 2;
        }
        w[NPANELS] = LPD_CELL_W * 9; h[NPANELS] = LPD_CELL_H;   // LPD numerals
        w[NPANELS + 1] = 16; h[NPANELS + 1] = 16;               // white
        for (int i = 0; i < n; i++) order[i] = i;
        for (int i = 0; i < n; i++)
            for (int j = i + 1; j < n; j++)
                if (h[order[j]] > h[order[i]]) { int t = order[i]; order[i] = order[j]; order[j] = t; }
        int x = 0, y = 0, rowh = 0, ok = 1;
        int ax[NPANELS + 2], ay[NPANELS + 2];
        for (int k = 0; k < n; k++) {
            int i = order[k];
            if (x + w[i] > ATLAS) { x = 0; y += rowh + 2; rowh = 0; }
            if (y + h[i] > ATLAS) { ok = 0; break; }
            ax[i] = x; ay[i] = y;
            x += w[i] + 2;
            if (h[i] > rowh) rowh = h[i];
        }
        if (!ok) continue;
        for (int i = 0; i < NPANELS; i++) {
            cp_panels[i].ax = ax[i] + 1; cp_panels[i].ay = ay[i] + 1;
            cp_panels[i].aw = w[i] - 2; cp_panels[i].ah = h[i] - 2;
        }
        lpd_ax = ax[NPANELS]; lpd_ay = ay[NPANELS];
        white_ax = ax[NPANELS + 1]; white_ay = ay[NPANELS + 1];
        TraceLog(LOG_INFO, "cockpit: atlas packed at %.0f%% density", scale * 100);
        return;
    }
    TraceLog(LOG_WARNING, "cockpit: atlas packing failed");
}

static void draw_atlas(int pass)
{
    float k = pass ? 0.5f : 1.0f;
    for (int i = 0; i < NPANELS; i++) {
        PanelDef *p = &cp_panels[i];
        Camera2D cam = { 0 };
        cam.offset = (Vector2){ p->ax * k, p->ay * k };
        cam.zoom = p->aw / p->w_mm * k;
        BeginScissorMode((int)(p->ax * k), (int)(p->ay * k), (int)(p->aw * k + 1), (int)(p->ah * k + 1));
        BeginMode2D(cam);
        cp_draw_panel(i, pass, cam);
        EndMode2D();
        EndScissorMode();
    }
    if (pass == 0) {
        DrawRectangle(white_ax, white_ay, 16, 16, WHITE);
        DrawRectangle(lpd_ax, lpd_ay, LPD_CELL_W * 9, LPD_CELL_H, BLACK);
        for (int i = 0; i < 9; i++) {
            char s[4];
            snprintf(s, sizeof s, "%d", i * 10);
            Vector2 m = MeasureTextEx(font_bold, s, 40, 1);
            DrawTextEx(font_bold, s, (Vector2){ lpd_ax + i * LPD_CELL_W + (LPD_CELL_W - m.x) / 2, (float)lpd_ay + 4 }, 40, 1, WHITE);
        }
    }
}

// --- Static cabin geometry ------------------------------------------------------------

static int in_tri2(double y, double s, double ay, double as, double by, double bs, double cy, double cs)
{
    double d1 = (y - by) * (as - bs) - (ay - by) * (s - bs);
    double d2 = (y - cy) * (bs - cs) - (by - cy) * (s - cs);
    double d3 = (y - ay) * (cs - as) - (cy - ay) * (s - as);
    int neg = d1 < 0 || d2 < 0 || d3 < 0, pos = d1 > 0 || d2 > 0 || d3 > 0;
    return !(neg && pos);
}

static int in_window(double y, double s)
{
    for (int side = -1; side <= 1; side += 2)
        if (in_tri2(y, s, side * WIN_YIN, WIN_S0, side * WIN_YOUT, WIN_S0, side * WIN_YIN, WIN_S1)) return 1;
    return 0;
}

static void build_front_wall(void)
{
    const double s_a = 0.01825, s_b = 1.199;
    const int NS = 90, NY = 140;
    for (int j = 0; j < NS; j++)
        for (int i = 0; i < NY; i++) {
            double s0 = s_a + (s_b - s_a) * j / NS, s1 = s_a + (s_b - s_a) * (j + 1) / NS;
            double y0 = -CABIN_HALF + 2 * CABIN_HALF * i / NY, y1 = -CABIN_HALF + 2 * CABIN_HALF * (i + 1) / NY;
            if (in_window(y0, s0) || in_window(y1, s0) || in_window(y0, s1) || in_window(y1, s1) ||
                in_window((y0 + y1) / 2, (s0 + s1) / 2))
                continue;
            quad(int_pt(y0, s0), int_pt(y1, s0), int_pt(y1, s1), int_pt(y0, s1), K_WALL, M_PAINT);
        }
    // Window wells and frames.
    for (int side = -1; side <= 1; side += 2) {
        double vy[3] = { side * WIN_YIN, side * WIN_YOUT, side * WIN_YIN };
        double vs[3] = { WIN_S0, WIN_S0, WIN_S1 };
        double cy = (vy[0] + vy[1] + vy[2]) / 3, cs = (vs[0] + vs[1] + vs[2]) / 3;
        for (int e = 0; e < 3; e++) {
            int f = (e + 1) % 3;
            quad(ext_pt(vy[e], vs[e]), ext_pt(vy[f], vs[f]), int_pt(vy[f], vs[f]), int_pt(vy[e], vs[e]), K_CONSOLE, M_PAINT);
            // Frame strip on the cabin side, outward from the edge.
            double dy = vy[f] - vy[e], ds = vs[f] - vs[e], len = sqrt(dy * dy + ds * ds);
            double ny = ds / len, ns = -dy / len;
            double my = (vy[e] + vy[f]) / 2 - cy, ms = (vs[e] + vs[f]) / 2 - cs;
            if (ny * my + ns * ms < 0) { ny = -ny; ns = -ns; }
            const double W = 0.03, X = 0.03;
            double ey = dy / len * X, es = ds / len * X;
            double ay = vy[e] - ey, as = vs[e] - es, by = vy[f] + ey, bs = vs[f] + es;
            V3 off = v3_scale(N_IN, 0.004);
            quad(v3_add(int_pt(ay, as), off), v3_add(int_pt(by, bs), off), v3_add(int_pt(by + ny * W, bs + ns * W), off),
                 v3_add(int_pt(ay + ny * W, as + ns * W), off), K_FRAME, M_BLACK);
            // Bolt heads around the frame.
            for (int k = 1; k < 8; k++) {
                double t = k / 8.0;
                V3 c = v3_add(int_pt(vy[e] + dy * t + ny * W * 0.5, vs[e] + ds * t + ns * W * 0.5), v3_scale(N_IN, 0.004));
                box(v3_sub(c, v3(0.003, 0.003, 0.003)), v3_add(c, v3(0.003, 0.003, 0.003)), K_METAL, M_METAL);
            }
        }
    }
}

static void build_shell(void)
{
    const double F = CABIN_FLOOR, C = CABIN_CEIL, H = CABIN_HALF, A = CABIN_AFT;
    // Floor (with a darker walkway) and aft wall.
    quad(v3(F, -H, A), v3(F, -H, 1.00), v3(F, H, 1.00), v3(F, H, A), K_FLOOR, M_PAINT);
    for (int i = 0; i < 6; i++) {
        double z = -0.9 + i * 0.3;
        box(v3(F, -0.95, z), v3(F + 0.004, 0.95, z + 0.02), K_BLACK, M_BLACK);   // floor rails
    }
    quad(v3(F, -H, A), v3(F, H, A), v3(C, H, A), v3(C, -H, A), K_WALL, M_PAINT);
    // Lower forward wall (hatch) and upper bevel.
    quad(v3(F, -H, 1.00), v3(2.00, -H, 1.00), v3(2.00, H, 1.00), v3(F, H, 1.00), K_WALL, M_PAINT);
    quad(int_pt(-H, 1.199), int_pt(H, 1.199), v3(C, H, 0.10), v3(C, -H, 0.10), K_WALL, M_PAINT);
    // Ceiling with the docking-window opening.
    quad(v3(C, -H, A), v3(C, H, A), v3(C, H, DW_Z0), v3(C, -H, DW_Z0), K_WALL, M_PAINT);
    quad(v3(C, -H, DW_Z1), v3(C, H, DW_Z1), v3(C, H, 0.10), v3(C, -H, 0.10), K_WALL, M_PAINT);
    quad(v3(C, -H, DW_Z0), v3(C, DW_Y0, DW_Z0), v3(C, DW_Y0, DW_Z1), v3(C, -H, DW_Z1), K_WALL, M_PAINT);
    quad(v3(C, DW_Y1, DW_Z0), v3(C, H, DW_Z0), v3(C, H, DW_Z1), v3(C, DW_Y1, DW_Z1), K_WALL, M_PAINT);
    // Docking window well.
    double xo = 3.39;
    quad(v3(C, DW_Y0, DW_Z0), v3(xo, DW_Y0, DW_Z0), v3(xo, DW_Y1, DW_Z0), v3(C, DW_Y1, DW_Z0), K_CONSOLE, M_PAINT);
    quad(v3(C, DW_Y0, DW_Z1), v3(xo, DW_Y0, DW_Z1), v3(xo, DW_Y1, DW_Z1), v3(C, DW_Y1, DW_Z1), K_CONSOLE, M_PAINT);
    quad(v3(C, DW_Y0, DW_Z0), v3(xo, DW_Y0, DW_Z0), v3(xo, DW_Y0, DW_Z1), v3(C, DW_Y0, DW_Z1), K_CONSOLE, M_PAINT);
    quad(v3(C, DW_Y1, DW_Z0), v3(xo, DW_Y1, DW_Z0), v3(xo, DW_Y1, DW_Z1), v3(C, DW_Y1, DW_Z1), K_CONSOLE, M_PAINT);
    // Side walls: fan over the cabin profile.
    double px[6] = { F, F, 2.00, int_pt(0, 1.199).x, C, C };
    double pz[6] = { A, 1.00, 1.00, int_pt(0, 1.199).z, 0.10, A };
    for (int side = -1; side <= 1; side += 2)
        for (int i = 0; i < 6; i++) {
            int j = (i + 1) % 6;
            V3 c = v3(2.35, side * H, -0.10);
            tri(c, v3(px[i], side * H, pz[i]), v3(px[j], side * H, pz[j]), K_WALL, M_PAINT);
        }
    // Slanted front wall edge closure between the lower wall and the slant.
    quad(v3(2.00, -H, 1.00), v3(2.00, H, 1.00), int_pt(H, 0.01825), int_pt(-H, 0.01825), K_WALL, M_PAINT);
    build_front_wall();
}

static V3 panel_pt(int i, double u, double v)
{
    const PanelDef *p = &cp_panels[i];
    V3 eu = v3_sub(p->tr, p->tl), ev = v3_sub(p->bl, p->tl);
    return v3_add(p->tl, v3_add(v3_scale(eu, u / p->w_mm), v3_scale(ev, v / p->h_mm)));
}

static V3 panel_normal(int i)
{
    const PanelDef *p = &cp_panels[i];
    return v3_norm(v3_cross(v3_sub(p->bl, p->tl), v3_sub(p->tr, p->tl)));
}

static void build_panels(void)
{
    for (int i = 0; i < NPANELS; i++) {
        PanelDef *p = &cp_panels[i];
        V3 a = p->tl, b = p->tr, c = v3_add(p->tr, v3_sub(p->bl, p->tl)), d = p->bl;
        quad_uv(a, b, c, d, atlas_uv(i, 0, 0), atlas_uv(i, p->w_mm, 0), atlas_uv(i, p->w_mm, p->h_mm), atlas_uv(i, 0, p->h_mm),
                K_PANEL, M_PANEL);
        // Panel plate thickness and a dark gap around it.
        V3 back = v3_scale(panel_normal(i), -0.012);
        V3 q[4] = { a, b, c, d };
        for (int e = 0; e < 4; e++) {
            int f = (e + 1) % 4;
            quad(q[e], v3_add(q[e], back), v3_add(q[f], back), q[f], K_FRAME, M_BLACK);
        }
    }
    // Console structure around panels 1/2 (side cheeks, top, bottom fillet).
    for (int side = -1; side <= 1; side += 2) {
        int pn = side < 0 ? P1 : P2;
        PanelDef *p = &cp_panels[pn];
        V3 top_out = side < 0 ? p->tl : p->tr;
        V3 bot_out = side < 0 ? p->bl : v3_add(p->tr, v3_sub(p->bl, p->tl));
        V3 wall_top = v3(top_out.x, top_out.y, 1.00 - SZ / SX * (top_out.x - 2.00) - 0.003);
        V3 wall_bot = v3(bot_out.x, bot_out.y, 1.00 - SZ / SX * (bot_out.x - 2.00) - 0.003);
        quad(top_out, bot_out, wall_bot, wall_top, K_CONSOLE, M_PAINT);
        V3 top_in = side < 0 ? p->tr : p->tl;
        quad(top_out, top_in, v3(top_in.x, top_in.y, wall_top.z), wall_top, K_CONSOLE, M_PAINT);
        V3 bot_in = side < 0 ? v3_add(p->tr, v3_sub(p->bl, p->tl)) : p->bl;
        quad(bot_out, bot_in, v3(2.02, bot_in.y, 0.62), v3(2.02, bot_out.y, 0.62), K_CONSOLE, M_PAINT);
    }
    // Shelf top under the windows, and the shelf underside (knee space).
    for (int side = -1; side <= 1; side += 2) {
        double y0 = side * 0.47, y1 = side * 1.035;
        quad(v3(2.02, y0, 0.62), v3(2.02, y1, 0.62), v3(2.02, y1, 0.99), v3(2.02, y0, 0.99), K_CONSOLE, M_PAINT);
        double u0 = side * 0.22, u1 = side * 1.02;
        quad(v3(1.82, u0, 0.42), v3(1.82, u1, 0.42), v3(1.82, u1, 1.00), v3(1.82, u0, 1.00), K_CONSOLE, M_PAINT);
        quad(v3(1.82, u1, 0.42), v3(2.02, u1, 0.62), v3(2.02, u1, 1.00), v3(1.82, u1, 1.00), K_CONSOLE, M_PAINT);
        // DSKY pedestal sides and underside.
        V3 a = v3(1.82, u0, 0.42), b = v3(1.74, u0, 0.16), c = v3(1.74, u0, 1.00), d = v3(1.82, u0, 1.00);
        quad(a, b, c, d, K_CONSOLE, M_PAINT);
    }
    quad(v3(1.74, -0.22, 0.16), v3(1.74, 0.22, 0.16), v3(1.74, 0.22, 1.00), v3(1.74, -0.22, 1.00), K_CONSOLE, M_PAINT);
    // Strip between panel 3 and panels 5/6.
    for (int side = -1; side <= 1; side += 2)
        quad(v3(2.02, side * 0.64, 0.62), v3(2.02, side * 0.66, 0.62), v3(1.82, side * 0.66, 0.42), v3(1.82, side * 0.64, 0.42),
             K_FRAME, M_BLACK);
    // Side consoles under panels 8 and 12.
    for (int side = -1; side <= 1; side += 2) {
        double yi = side * 0.84;
        quad(v3(1.90, yi, -0.40), v3(1.90, yi, 0.40), v3(1.45, yi, 0.40), v3(1.45, yi, -0.40), K_CONSOLE, M_PAINT);
        quad(v3(1.90, yi, 0.40), v3(1.90, side * 1.035, 0.40), v3(1.45, side * 1.035, 0.40), v3(1.45, yi, 0.40), K_CONSOLE, M_PAINT);
        quad(v3(1.90, yi, -0.40), v3(1.90, side * 1.035, -0.40), v3(1.45, side * 1.035, -0.40), v3(1.45, yi, -0.40), K_CONSOLE, M_PAINT);
    }
}

static void build_details(void)
{
    // Forward hatch (inner face) under the DSKY pedestal, with handle.
    box(v3(1.40, -0.40, 0.975), v3(1.72, 0.40, 0.995), K_CLOTH, M_PAINT);
    box(v3(1.40, -0.43, 0.97), v3(1.42, 0.43, 0.995), K_FRAME, M_BLACK);
    box(v3(1.70, -0.43, 0.97), v3(1.72, 0.43, 0.995), K_FRAME, M_BLACK);
    tube(v3(1.56, -0.18, 0.96), v3(1.56, 0.18, 0.96), 0.012, 0.012, 8, K_METAL, M_METAL, 1);
    box(v3(1.55, -0.20, 0.955), v3(1.57, -0.16, 0.975), K_METAL, M_METAL);
    box(v3(1.55, 0.16, 0.955), v3(1.57, 0.20, 0.975), K_METAL, M_METAL);

    // Overhead (docking) hatch with its handle.
    tube(v3(CABIN_CEIL - 0.002, 0, -0.30), v3(CABIN_CEIL - 0.03, 0, -0.30), 0.42, 0.40, 40, K_WALL, M_PAINT, 1);
    tube(v3(CABIN_CEIL - 0.03, -0.12, -0.30), v3(CABIN_CEIL - 0.03, 0.12, -0.30), 0.015, 0.015, 8, K_METAL, M_METAL, 1);
    for (int k = 0; k < 12; k++) {
        double a = k * 2 * PI_D / 12;
        V3 c = v3(CABIN_CEIL - 0.034, 0.36 * cos(a), -0.30 + 0.36 * sin(a));
        box(v3_sub(c, v3(0.006, 0.012, 0.012)), v3_add(c, v3(0.006, 0.012, 0.012)), K_METAL, M_METAL);
    }

    // Ascent engine cover between the crew stations (aft).
    tube(v3(CABIN_FLOOR, 0, -0.82), v3(1.86, 0, -0.82), 0.44, 0.44, 40, K_WALL, M_PAINT, 0);
    tube(v3(1.86, 0, -0.82), v3(1.94, 0, -0.82), 0.44, 0.36, 40, K_WALL, M_PAINT, 1);
    box(v3(1.94, -0.12, -0.95), v3(1.96, 0.12, -0.70), K_CLOTH, M_PAINT);   // stowed checklist pad

    // Aft equipment: ECS package, LiOH canisters, hoses, PLSS stowage.
    box(v3(1.95, -0.60, -1.24), v3(2.90, 0.60, -1.05), K_WALL, M_PAINT);
    tube(v3(2.20, -0.45, -1.04), v3(2.60, -0.45, -1.04), 0.07, 0.07, 18, K_METAL, M_METAL, 1);
    tube(v3(2.20, -0.25, -1.04), v3(2.60, -0.25, -1.04), 0.07, 0.07, 18, K_METAL, M_METAL, 1);
    box(v3(2.15, 0.10, -1.06), v3(2.70, 0.50, -0.98), K_BLACK, M_BLACK);
    for (int side = -1; side <= 1; side += 2) {
        box(v3(CABIN_FLOOR, side * 0.92, -1.20), v3(2.05, side * 0.55, -0.92), K_CLOTH, M_PAINT);   // PLSS
        box(v3(2.05, side * 0.90, -1.18), v3(2.12, side * 0.57, -0.95), K_BLUE, M_PAINT);
    }
    // Suit hoses (blue supply, red return) from the ECS to each station.
    for (int side = -1; side <= 1; side += 2) {
        V3 p0 = v3(2.40, side * 0.40, -1.04), p1 = v3(2.10, side * 0.55, -0.60), p2 = v3(1.90, side * 0.70, -0.20);
        tube(p0, p1, 0.025, 0.025, 10, K_BLUE, M_PAINT, 0);
        tube(p1, p2, 0.025, 0.025, 10, K_BLUE, M_PAINT, 0);
        V3 q0 = v3(2.35, side * 0.32, -1.04), q1 = v3(2.02, side * 0.48, -0.62), q2 = v3(1.84, side * 0.64, -0.24);
        tube(q0, q1, 0.025, 0.025, 10, K_RED, M_PAINT, 0);
        tube(q1, q2, 0.025, 0.025, 10, K_RED, M_PAINT, 0);
    }
    // Wiring bundles along the upper wall corners.
    for (int side = -1; side <= 1; side += 2) {
        tube(v3(3.28, side * 1.00, -1.20), v3(3.28, side * 1.00, 0.30), 0.02, 0.02, 8, K_BLACK, M_BLACK, 0);
        tube(v3(3.24, side * 1.00, -1.20), v3(3.24, side * 1.00, 0.30), 0.012, 0.012, 8, (Color){ 200, 196, 180, 255 }, M_PAINT, 0);
    }
    // Floodlights and the utility light (lenses glow).
    for (int side = -1; side <= 1; side += 2) {
        box(v3(3.28, side * 0.55 - 0.06, -0.05), v3(3.32, side * 0.55 + 0.06, 0.05), K_FRAME, M_BLACK);
        box(v3(3.275, side * 0.55 - 0.05, -0.04), v3(3.28, side * 0.55 + 0.05, 0.04), (Color){ 255, 236, 200, 255 }, M_EMIT);
    }
    // COAS (crewman optical alignment sight) above the CDR window.
    box(v3(3.02, -0.70, 0.36), v3(3.12, -0.60, 0.52), K_BLACK, M_BLACK);
    tube(v3(3.07, -0.65, 0.52), v3(3.07, -0.65, 0.56), 0.03, 0.03, 16, K_FRAME, M_BLACK, 1);
    // Checklist book on the left wall, velcro-mounted.
    box(v3(2.25, -1.035, -0.25), v3(2.52, -1.010, -0.06), (Color){ 220, 214, 190, 255 }, M_PAINT);
    box(v3(2.25, -1.010, -0.25), v3(2.52, -1.004, -0.06), (Color){ 60, 70, 120, 255 }, M_PAINT);
    // Armrests at each station.
    for (int side = -1; side <= 1; side += 2)
        for (int k = 0; k < 2; k++) {
            double y = side * (k ? 0.95 : 0.40);
            tube(v3(1.86, y, 0.40), v3(1.86, y, 0.05), 0.018, 0.018, 10, K_BLACK, M_BLACK, 1);
        }
}

// --- LPD markings and window panes (transparent mesh) ----------------------------------

static void etch_line(V3 a, V3 b, double width)
{
    V3 d = v3_norm(v3_sub(b, a));
    V3 side = v3_scale(v3_norm(v3_cross(d, N_IN)), width / 2);
    quad(v3_sub(a, side), v3_add(a, side), v3_add(b, side), v3_sub(b, side), (Color){ 40, 38, 36, 255 }, M_ETCH);
}

// Where the CDR's design-eye line of sight at LPD angle `a` (deg below the
// Y-Z plane, as Luminary's LOOKANGL) crosses a pane at `depth`.
static int lpd_point(double a_deg, double y_off, double depth, V3 *out, double *py, double *ps)
{
    double a = a_deg * DEG;
    V3 d = v3(-sin(a), 0, cos(a));
    V3 eye = v3_add(EYE_CDR, v3(0, y_off, 0));
    V3 p0 = pane_pt(0, 0, depth);
    double den = v3_dot(d, N_IN);
    if (fabs(den) < 1e-6) return 0;
    double t = v3_dot(v3_sub(p0, eye), N_IN) / den;
    if (t <= 0) return 0;
    V3 p = v3_add(eye, v3_scale(d, t));
    // Plane coordinates.
    double s = v3_dot(v3_sub(p, p0), v3(SX, 0, -SZ));
    *py = p.y;
    *ps = s;
    *out = p;
    return in_tri2(p.y, s, -WIN_YIN, WIN_S0, -WIN_YOUT, WIN_S0, -WIN_YIN, WIN_S1);
}

static void build_glass(void)
{
    // Panes: inner and outer of each forward window, and the docking window.
    for (int side = -1; side <= 1; side += 2)
        for (int k = 0; k < 2; k++) {
            double dep = k ? 0.006 : 0.030;
            tri(pane_pt(side * WIN_YIN, WIN_S0, dep), pane_pt(side * WIN_YOUT, WIN_S0, dep), pane_pt(side * WIN_YIN, WIN_S1, dep),
                (Color){ 120, 130, 140, 255 }, M_GLASS);
        }
    quad(v3(3.37, DW_Y0, DW_Z0), v3(3.37, DW_Y1, DW_Z0), v3(3.37, DW_Y1, DW_Z1), v3(3.37, DW_Y0, DW_Z1), (Color){ 120, 130, 140, 255 },
         M_GLASS);

    // LPD reticle. Outer pane: the vertical line plus 5-degree marks; inner
    // pane: the angle scale with numerals. They only line up from the
    // design eye, as on the real windows.
    const double outer = 0.008, inner = 0.028;
    V3 prev;
    int have = 0;
    for (double a = 0; a <= 85; a += 0.5) {
        V3 p;
        double y, s;
        if (lpd_point(a, 0, outer, &p, &y, &s)) {
            if (have) etch_line(prev, p, 0.0010);
            prev = p;
            have = 1;
        } else {
            have = 0;
        }
    }
    for (int a = 0; a <= 85; a++) {
        V3 p, q;
        double y, s;
        if (!lpd_point(a, 0, inner, &p, &y, &s)) continue;
        double half = a % 10 == 0 ? 0.016 : a % 5 == 0 ? 0.010 : 0.005;
        V3 dy = v3(0, half, 0);
        etch_line(v3_sub(p, dy), v3_add(p, dy), a % 5 ? 0.0007 : 0.0010);
        if (lpd_point(a, 0, outer, &q, &y, &s) && a % 5 == 0)
            etch_line(v3_sub(q, v3(0, 0.004, 0)), v3_add(q, v3(0, 0.004, 0)), 0.0008);
        if (a % 10 == 0 && a > 0 && a < 90) {
            // Numeral quad to the right of the mark (toward the console).
            int cell = a / 10;
            double w = 0.014, h = 0.008;
            V3 up = v3(SX, 0, -SZ);
            V3 base = v3_add(p, v3(0, half + 0.004, 0));
            V3 c0 = v3_add(base, v3_scale(up, -h / 2)), c1 = v3_add(c0, v3(0, w, 0));
            V3 c2 = v3_add(c1, v3_scale(up, h)), c3 = v3_add(c0, v3_scale(up, h));
            float u0 = (float)(lpd_ax + cell * LPD_CELL_W) / ATLAS, u1 = (float)(lpd_ax + (cell + 1) * LPD_CELL_W) / ATLAS;
            float vt = 1.0f - (float)lpd_ay / ATLAS, vb = 1.0f - (float)(lpd_ay + LPD_CELL_H) / ATLAS;
            quad_uv(c0, c1, c2, c3, (Vector2){ u0, vb }, (Vector2){ u1, vb }, (Vector2){ u1, vt }, (Vector2){ u0, vt },
                    (Color){ 255, 255, 255, 255 }, 254);
        }
    }
}

// --- Dynamic geometry: switch levers, knobs, hand controllers -----------------------

static double hc_pitch, hc_yaw, hc_roll;
void cockpit_set_hand_controls(double pitch, double yaw, double roll) { hc_pitch = pitch; hc_yaw = yaw; hc_roll = roll; }

static void build_dynamic(void)
{
    B = &DB;
    DB.count = 0;
    for (int i = 0; i < cp_nlev; i++) {
        const Lever *l = &cp_lev[i];
        const PanelDef *p = &cp_panels[l->panel];
        V3 n = panel_normal(l->panel);
        V3 upv = v3_norm(v3_sub(p->tl, p->bl));
        V3 uv = v3_norm(v3_sub(p->tr, p->tl));
        V3 base = panel_pt(l->panel, l->x, l->y);
        Color c = { l->r, l->g, l->b, 255 };
        if (l->kind == LEV_TOGGLE) {
            V3 d = v3_norm(v3_add(n, v3_scale(upv, tan(30 * DEG) * l->val)));
            V3 tip = v3_add(base, v3_scale(d, 0.016));
            tube(v3_add(base, v3_scale(n, 0.001)), v3_add(base, v3_scale(n, 0.004)), 0.0042, 0.0040, 6, (Color){ 140, 142, 146, 255 }, M_METAL, 1);
            tube(base, v3_add(base, v3_scale(d, 0.011)), 0.0019, 0.0016, 6, c, M_METAL, 0);
            tube(v3_add(base, v3_scale(d, 0.011)), tip, 0.0024, 0.0024, 6, c, M_METAL, 1);
            if (l->guard)
                for (int s = -1; s <= 1; s += 2) {
                    V3 g = v3_add(base, v3_scale(uv, s * 0.0058));
                    obox(v3_add(g, v3_scale(n, 0.009)), uv, upv, n, 0.0007, 0.007, 0.009, (Color){ 175, 176, 180, 255 }, M_METAL);
                }
        } else if (l->kind == LEV_KNOB) {
            double r = l->size * 0.001;
            V3 top = v3_add(base, v3_scale(n, 0.010));
            tube(base, top, r, r * 0.92, 16, c, M_BLACK, 1);
            tube(top, v3_add(top, v3_scale(n, 0.004)), r * 0.55, r * 0.5, 12, c, M_BLACK, 1);
            V3 dir = v3_add(v3_scale(uv, sin(l->val)), v3_scale(upv, cos(l->val)));
            V3 side = v3_cross(n, dir);
            obox(v3_add(top, v3_add(v3_scale(dir, r * 0.55), v3_scale(n, 0.0006))), dir, side, n, r * 0.45, 0.0006, 0.0006,
                 (Color){ 240, 240, 236, 255 }, M_PAINT);
            obox(v3_add(base, v3_add(v3_scale(dir, r * 0.95), v3_scale(n, 0.007))), dir, side, n, r * 0.15, 0.0012, 0.003,
                 (Color){ 30, 30, 32, 255 }, M_BLACK);
        }
    }
    // CDR attitude controller assembly (pistol grip), deflected by the RHC.
    for (int seat = 0; seat < 2; seat++) {
        double ys = seat ? 0.86 : -0.44;
        V3 pivot = v3(1.80, ys, 0.36);
        double pp = seat ? 0 : hc_pitch, rr = seat ? 0 : hc_roll, yy = seat ? 0 : hc_yaw;
        M3 rot = m3_mul(m3_ry(-pp * 0.30), m3_mul(m3_rz(-rr * 0.30), m3_rx(yy * 0.25)));
        V3 ax = m3_apply(rot, v3(1, 0, 0)), ay = m3_apply(rot, v3(0, 1, 0)), az = m3_apply(rot, v3(0, 0, 1));
        tube(v3(1.76, ys, 0.36), v3(1.80, ys, 0.36), 0.035, 0.028, 16, K_BLACK, M_BLACK, 1);
        box(v3(1.70, ys - 0.03, 0.36), v3(1.76, ys + 0.03, 0.46), K_CONSOLE, M_PAINT);
        V3 g0 = v3_add(pivot, v3_scale(ax, 0.02));
        tube(pivot, g0, 0.012, 0.012, 10, K_METAL, M_METAL, 0);
        V3 gc = v3_add(g0, v3_add(v3_scale(ax, 0.055), v3_scale(az, -0.008)));
        obox(gc, v3_norm(v3_add(ax, v3_scale(az, -0.25))), ay, az, 0.055, 0.016, 0.020, K_BLACK, M_BLACK);
        obox(v3_add(gc, v3_add(v3_scale(az, 0.022), v3_scale(ax, 0.025))), ax, ay, az, 0.010, 0.006, 0.005, (Color){ 60, 60, 64, 255 },
             M_METAL);
        obox(v3_add(gc, v3_scale(ax, 0.058)), ax, ay, az, 0.004, 0.010, 0.010, (Color){ 150, 40, 30, 255 }, M_PAINT);
    }
    // Thrust/translation controllers (T-handles), left hand of each crewman.
    for (int seat = 0; seat < 2; seat++) {
        double ys = seat ? 0.46 : -0.90;
        V3 m = v3(1.80, ys, 0.42);
        tube(v3(1.80, ys, 0.44), m, 0.02, 0.02, 12, K_CONSOLE, M_PAINT, 1);
        V3 e = v3(1.80, ys, 0.30);
        tube(m, e, 0.008, 0.008, 8, K_METAL, M_METAL, 0);
        tube(v3(1.80, ys - 0.055, 0.30), v3(1.80, ys + 0.055, 0.30), 0.012, 0.012, 10, K_BLACK, M_BLACK, 1);
        tube(v3(1.80, ys + 0.055, 0.30), v3(1.86, ys + 0.055, 0.30), 0.006, 0.006, 8, K_METAL, M_METAL, 1);   // throttle/jets lever
    }
    B = &SB;
}

// --- Shader -------------------------------------------------------------------------------

static const char *ck_vs =
    "#version 330\n"
    "in vec3 vertexPosition; in vec3 vertexNormal; in vec2 vertexTexCoord; in vec4 vertexColor;\n"
    "uniform mat4 mvp; uniform mat4 matModel;\n"
    "out vec3 bodyPos; out vec3 bodyN; out vec2 uv; out vec4 vcol; out vec3 camPos;\n"
    "void main(){ bodyPos = vertexPosition; bodyN = vertexNormal; uv = vertexTexCoord; vcol = vertexColor;\n"
    "  camPos = (matModel * vec4(vertexPosition, 1.0)).xyz; gl_Position = mvp * vec4(vertexPosition, 1.0); }\n";

static const char *ck_fs =
    "#version 330\n"
    "in vec3 bodyPos; in vec3 bodyN; in vec2 uv; in vec4 vcol; in vec3 camPos;\n"
    "uniform sampler2D texture0; uniform sampler2D texture1; uniform sampler2D texture2;\n"
    "uniform mat4 shadowVP2; uniform vec3 shadowC2; uniform vec3 sunB; uniform float sunOn; uniform vec3 eyeB;\n"
    "uniform vec3 winC[3]; uniform vec3 winN[3]; uniform float winA[3]; uniform vec3 winL[3];\n"
    "uniform vec3 lampP[3]; uniform vec3 lampC[3];\n"
    "uniform float exposure; uniform float emitGain; uniform float glare;\n"
    "out vec4 finalColor;\n"
    "float hash12(vec2 p){ vec3 p3 = fract(vec3(p.xyx) * 0.1031); p3 += dot(p3, p3.yzx + 33.33); return fract((p3.x + p3.y) * p3.z); }\n"
    "float vnoise(vec2 p){ vec2 i = floor(p), f = fract(p); f = f*f*(3.0-2.0*f);\n"
    "  return mix(mix(hash12(i), hash12(i+vec2(1,0)), f.x), mix(hash12(i+vec2(0,1)), hash12(i+vec2(1,1)), f.x), f.y); }\n"
    "float sunShadow(vec3 p, vec3 n){\n"
    "  vec4 sp = shadowVP2 * vec4(p - shadowC2 + n * 0.008, 1.0);\n"
    "  vec3 c = sp.xyz / sp.w * 0.5 + 0.5;\n"
    "  if (c.x < 0.0 || c.x > 1.0 || c.y < 0.0 || c.y > 1.0) return 0.0;\n"
    "  float s = 0.0; vec2 t = 1.0 / vec2(textureSize(texture2, 0));\n"
    "  for (int x=-1;x<=1;x++) for(int y=-1;y<=1;y++){ float d = texture(texture2, c.xy + vec2(x,y)*t*1.2).r; s += (c.z - 0.0015 > d) ? 0.0 : 1.0; }\n"
    "  return s / 9.0;\n"
    "}\n"
    "void main(){\n"
    "  gl_FragDepth = clamp(length(camPos) / 6.0, 0.0, 1.0) * 0.03;\n"
    "  int mat = int(vcol.a * 255.0 + 0.5);\n"
    "  vec3 N = normalize(bodyN); vec3 V = normalize(eyeB - bodyPos);\n"
    "  if (dot(N, V) < 0.0) N = -N;\n"
    "  vec3 alb = vcol.rgb; vec3 emis = vec3(0.0);\n"
    "  if (mat == 1) { alb *= texture(texture0, uv).rgb; emis = texture(texture1, uv).rgb * emitGain; }\n"
    "  if (mat == 5) { finalColor = vec4(alb * (0.3 + 3.0 * glare) * exposure, 1.0); return; }\n"
    "  if (mat == 255) {}\n"
    "  if (mat == 254) { float a = texture(texture0, uv).r; finalColor = vec4(vec3(0.03) * exposure, a * 0.85); return; }\n"
    "  if (mat == 6) { finalColor = vec4(vec3(0.03) * exposure, 0.85); return; }\n"
    "  float specK = mat == 3 ? 0.55 : mat == 1 ? 0.06 : mat == 4 ? 0.05 : 0.03;\n"
    "  float shin = mat == 3 ? 80.0 : 14.0;\n"
    "  vec3 col = vec3(0.0);\n"
    "  float NL = max(dot(N, sunB), 0.0) * sunOn;\n"
    "  if (NL > 0.0) {\n"
    "    float sh = sunShadow(bodyPos, N);\n"
    "    vec3 H = normalize(sunB + V);\n"
    "    col += (alb * NL + specK * pow(max(dot(N, H), 0.0), shin) * NL * 4.0) * sh * vec3(1.0, 0.985, 0.96) * 2.8;\n"
    "  }\n"
    "  for (int i = 0; i < 3; i++) {\n"
    "    vec3 w = winC[i] - bodyPos; float d2 = dot(w, w); vec3 wn = w * inversesqrt(d2);\n"
    "    float f = winA[i] * max(dot(N, wn), 0.0) * max(dot(-wn, winN[i]), 0.0) / (3.14159 * d2 + winA[i]);\n"
    "    col += alb * winL[i] * f * 6.0;\n"
    "  }\n"
    "  for (int i = 0; i < 3; i++) {\n"
    "    vec3 l = lampP[i] - bodyPos; float d2 = dot(l, l); l *= inversesqrt(d2);\n"
    "    vec3 H = normalize(l + V);\n"
    "    float nl = max(dot(N, l), 0.0);\n"
    "    col += lampC[i] * (alb * nl + specK * pow(max(dot(N, H), 0.0), shin) * nl) / (d2 + 0.06);\n"
    "  }\n"
    "  col += alb * vec3(0.022, 0.023, 0.025);\n"
    "  if (mat == 7) {\n"
    "    float smudge = vnoise(bodyPos.yz * 60.0) * 0.6 + vnoise(bodyPos.yz * 230.0) * 0.4;\n"
    "    float a = 0.012 + glare * (0.03 + 0.18 * smudge);\n"
    "    finalColor = vec4(vec3(0.9, 0.92, 0.95) * (0.02 + glare * 1.5) * exposure, a);\n"
    "    return;\n"
    "  }\n"
    "  col += emis;\n"
    "  finalColor = vec4(col * exposure, 1.0);\n"
    "}\n";

static Shader ck_sh;
static Material ck_mat, ck_depth_mat;
static Mesh mesh_static, mesh_dyn, mesh_glass;
static int dyn_cap;

#define SHADOW2 2048
static unsigned int sh2_fbo, sh2_tex;
static Matrix sh2_vp;
static const V3 SH2_C = { 2.35, 0, -0.10 };

static Mesh upload(Builder *b, int dynamic, int cap)
{
    Mesh m = { 0 };
    int n = dynamic ? cap : b->count;
    if (dynamic && b->cap < cap) {
        b->cap = cap;
        b->p = realloc(b->p, sizeof(float) * 3 * cap);
        b->n = realloc(b->n, sizeof(float) * 3 * cap);
        b->t = realloc(b->t, sizeof(float) * 2 * cap);
        b->c = realloc(b->c, 4 * cap);
        memset(b->p, 0, sizeof(float) * 3 * cap);
    }
    m.vertexCount = n;
    m.triangleCount = n / 3;
    m.vertices = malloc(sizeof(float) * 3 * n);
    m.normals = malloc(sizeof(float) * 3 * n);
    m.texcoords = malloc(sizeof(float) * 2 * n);
    m.colors = malloc(4 * n);
    memcpy(m.vertices, b->p, sizeof(float) * 3 * (dynamic ? 0 : n));
    memcpy(m.normals, b->n, sizeof(float) * 3 * (dynamic ? 0 : n));
    memcpy(m.texcoords, b->t, sizeof(float) * 2 * (dynamic ? 0 : n));
    memcpy(m.colors, b->c, 4 * (dynamic ? 0 : n));
    UploadMesh(&m, dynamic);
    return m;
}

void cockpit_init(void)
{
    for (int i = 0; i < NPANELS; i++) {
        PanelDef *p = &cp_panels[i];
        p->w_mm = (float)(v3_len(v3_sub(p->tr, p->tl)) * 1000);
        p->h_mm = (float)(v3_len(v3_sub(p->bl, p->tl)) * 1000);
    }
    pack_atlas();
    white_u = (white_ax + 8) / (float)ATLAS;
    white_v = 1.0f - (white_ay + 8) / (float)ATLAS;

    atlas_rt = LoadRenderTexture(ATLAS, ATLAS);
    emit_rt = LoadRenderTexture(ATLAS / 2, ATLAS / 2);

    B = &SB;
    build_shell();
    build_panels();
    build_details();
    mesh_static = upload(&SB, 0, 0);
    B = &TB;
    build_glass();
    mesh_glass = upload(&TB, 0, 0);
    B = &SB;
    dyn_cap = 60000;
    mesh_dyn = upload(&DB, 1, dyn_cap);

    ck_sh = LoadShaderFromMemory(ck_vs, ck_fs);
    ck_mat = LoadMaterialDefault();
    ck_mat.shader = ck_sh;
    ck_mat.maps[MATERIAL_MAP_DIFFUSE].texture = atlas_rt.texture;
    ck_mat.maps[MATERIAL_MAP_SPECULAR].texture = emit_rt.texture;

    sh2_fbo = rlLoadFramebuffer();
    rlEnableFramebuffer(sh2_fbo);
    sh2_tex = rlLoadTextureDepth(SHADOW2, SHADOW2, false);
    rlFramebufferAttach(sh2_fbo, sh2_tex, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);
    if (!rlFramebufferComplete(sh2_fbo)) TraceLog(LOG_WARNING, "cockpit: shadow FBO incomplete");
    rlDisableFramebuffer();
    ck_mat.maps[MATERIAL_MAP_NORMAL].texture = (Texture2D){ sh2_tex, SHADOW2, SHADOW2, 1, PIXELFORMAT_UNCOMPRESSED_GRAYSCALE };
    ck_depth_mat = LoadMaterialDefault();
    ck_depth_mat.shader = *render_depth_shader();
    TraceLog(LOG_INFO, "cockpit: %d static + %d glass triangles", mesh_static.triangleCount, mesh_glass.triangleCount);
}

void cockpit_shutdown(void)
{
    UnloadRenderTexture(atlas_rt);
    UnloadRenderTexture(emit_rt);
    UnloadShader(ck_sh);
}

// --- Per-frame -------------------------------------------------------------------------------

static V3 sun_b, up_b;
static M3 cur_R;
static float glare_amt;

void cockpit_update(float dt) { cp_panels_update(dt); }

void cockpit_prepare(M3 R, V3 sun_site, V3 up_site, float dt)
{
    cur_R = R;
    M3 Rt = m3_transpose(R);
    sun_b = v3_norm(m3_apply(Rt, sun_site));
    up_b = v3_norm(m3_apply(Rt, up_site));

    (void)dt;
    cp_nlev = 0;
    cp_nhot = 0;
    BeginTextureMode(atlas_rt);
    ClearBackground(BLACK);
    draw_atlas(0);
    EndTextureMode();
    BeginTextureMode(emit_rt);
    ClearBackground(BLACK);
    draw_atlas(1);
    EndTextureMode();
    static int mips_done;
    GenTextureMipmaps(&atlas_rt.texture);
    GenTextureMipmaps(&emit_rt.texture);
    if (!mips_done) {
        SetTextureFilter(atlas_rt.texture, TEXTURE_FILTER_TRILINEAR);
        SetTextureFilter(atlas_rt.texture, TEXTURE_FILTER_ANISOTROPIC_8X);
        SetTextureFilter(emit_rt.texture, TEXTURE_FILTER_TRILINEAR);
        ck_mat.maps[MATERIAL_MAP_DIFFUSE].texture = atlas_rt.texture;
        ck_mat.maps[MATERIAL_MAP_SPECULAR].texture = emit_rt.texture;
        mips_done = 1;
    }

    // Levers etc. for this frame.
    build_dynamic();
    int n = DB.count < dyn_cap ? DB.count : dyn_cap;
    UpdateMeshBuffer(mesh_dyn, 0, DB.p, n * 3 * sizeof(float), 0);
    UpdateMeshBuffer(mesh_dyn, 1, DB.t, n * 2 * sizeof(float), 0);
    UpdateMeshBuffer(mesh_dyn, 2, DB.n, n * 3 * sizeof(float), 0);
    UpdateMeshBuffer(mesh_dyn, 3, DB.c, n * 4, 0);
    mesh_dyn.vertexCount = n;
    mesh_dyn.triangleCount = n / 3;

    // Cabin sun shadow map (body frame, around the cabin centre).
    const float rad = 2.0f;
    Vector3 eye = { (float)(sun_b.x * 4), (float)(sun_b.y * 4), (float)(sun_b.z * 4) };
    Vector3 upv = fabs(sun_b.y) > 0.95 ? (Vector3){ 1, 0, 0 } : (Vector3){ 0, 1, 0 };
    Matrix view = MatrixLookAt(eye, (Vector3){ 0, 0, 0 }, upv);
    Matrix proj = MatrixOrtho(-rad, rad, -rad, rad, 0.01, 8.0);
    sh2_vp = MatrixMultiply(view, proj);
    rlDrawRenderBatchActive();
    rlEnableFramebuffer(sh2_fbo);
    rlViewport(0, 0, SHADOW2, SHADOW2);
    rlClearScreenBuffers();
    rlEnableDepthTest();
    rlDisableBackfaceCulling();
    rlSetMatrixProjection(proj);
    rlSetMatrixModelview(view);
    rlDisableColorBlend();
    DrawMesh(mesh_static, ck_depth_mat, MatrixTranslate((float)-SH2_C.x, (float)-SH2_C.y, (float)-SH2_C.z));
    rlDrawRenderBatchActive();
    rlEnableColorBlend();
    rlEnableBackfaceCulling();
    rlDisableFramebuffer();
    rlDisableDepthTest();
}

// --- Head / view ----------------------------------------------------------------------------

static int seat;            // 0 CDR, 1 LMP
static double head_yaw = 0.0, head_pitch = -36 * DEG, fov = 80 * DEG, lean = 0;
static const Hot *hover;
static float hover_u, hover_v;
static Vector2 mouse_pos;
static int view_w = 1, view_h = 1;

static V3 eye_body(void)
{
    V3 e = seat ? EYE_LMP : EYE_CDR;
    // Leaning toward the window (and down a little).
    return v3_add(e, v3_add(v3_scale(N_IN, -lean * 0.14), v3(-lean * 0.05, 0, 0)));
}

static V3 look_dir_body(void)
{
    double y = head_yaw + (seat ? 6 * DEG : -6 * DEG);   // the window centre is slightly outboard
    return v3_norm(v3(sin(head_pitch), cos(head_pitch) * sin(y), cos(head_pitch) * cos(y)));
}

void cockpit_set_seat(int lmp) { seat = lmp ? 1 : 0; }
void cockpit_set_view(double yaw_deg, double pitch_deg) { head_yaw = yaw_deg * DEG; head_pitch = pitch_deg * DEG; }

void cockpit_input(int w, int h)
{
    view_w = w;
    view_h = h;
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        Vector2 d = GetMouseDelta();
        head_yaw = fmax(-160 * DEG, fmin(160 * DEG, head_yaw + d.x * 0.0035 * fov / (60 * DEG)));
        head_pitch = fmax(-85 * DEG, fmin(70 * DEG, head_pitch - d.y * 0.0035 * fov / (60 * DEG)));
    }
    float wh = GetMouseWheelMove();
    if (wh != 0) {
        if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) lean = fmax(0, fmin(1, lean + wh * 0.15));
        else fov = fmax(18 * DEG, fmin(95 * DEG, fov * pow(0.9, wh)));
    }
    if (IsKeyPressed(KEY_HOME)) { head_yaw = 0; head_pitch = -36 * DEG; fov = 80 * DEG; lean = 0; }
    if (IsKeyPressed(KEY_END)) { head_yaw = seat ? -38 * DEG : 38 * DEG; head_pitch = -62 * DEG; fov = 60 * DEG; }
    if (IsKeyPressed(KEY_F9)) seat = !seat;
    if (IsKeyPressed(KEY_L)) lean = lean > 0.5 ? 0 : 1;

    // Mouse picking against the panels.
    mouse_pos = GetMousePosition();
    hover = NULL;
    if (mouse_pos.x < 0 || mouse_pos.x >= w || mouse_pos.y < 0 || mouse_pos.y >= h) return;
    V3 e = eye_body(), f = look_dir_body();
    V3 r = v3_norm(v3_cross(f, v3(1, 0, 0)));
    V3 u = v3_cross(r, f);
    double th = tan(fov / 2), aspect = (double)w / h;
    double sx = (2.0 * mouse_pos.x / w - 1) * th * aspect, sy = (1 - 2.0 * mouse_pos.y / h) * th;
    V3 d = v3_norm(v3_add(f, v3_add(v3_scale(r, sx), v3_scale(u, sy))));
    double best = 1e9;
    int bp = -1;
    float bu = 0, bv = 0;
    for (int i = 0; i < NPANELS; i++) {
        const PanelDef *p = &cp_panels[i];
        V3 n = panel_normal(i);
        double den = v3_dot(d, n);
        if (fabs(den) < 1e-6) continue;
        double t = v3_dot(v3_sub(p->tl, e), n) / den;
        if (t <= 0 || t > best) continue;
        V3 q = v3_sub(v3_add(e, v3_scale(d, t)), p->tl);
        V3 eu = v3_sub(p->tr, p->tl), ev = v3_sub(p->bl, p->tl);
        // Solve q = a*eu + b*ev (eu, ev not necessarily orthogonal).
        double uu = v3_dot(eu, eu), vv = v3_dot(ev, ev), uvd = v3_dot(eu, ev);
        double qu = v3_dot(q, eu), qv = v3_dot(q, ev), det = uu * vv - uvd * uvd;
        double a = (qu * vv - qv * uvd) / det, b = (qv * uu - qu * uvd) / det;
        if (a < 0 || a > 1 || b < 0 || b > 1) continue;
        best = t;
        bp = i;
        bu = (float)(a * p->w_mm);
        bv = (float)(b * p->h_mm);
    }
    if (bp < 0) return;
    for (int i = 0; i < cp_nhot; i++) {
        const Hot *hh = &cp_hot[i];
        if (hh->panel == bp && CheckCollisionPointRec((Vector2){ bu, bv }, hh->r)) {
            hover = hh;
            hover_u = bu;
            hover_v = bv;
            break;
        }
    }
    if (hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) cp_click(hover, hover_u, hover_v);
}

ViewCam cockpit_camera(V3 lm_pos, M3 R)
{
    ViewCam c;
    V3 f = look_dir_body();
    V3 r = v3_norm(v3_cross(f, v3(1, 0, 0)));
    V3 u = v3_cross(r, f);
    c.pos = v3_add(lm_pos, m3_apply(R, eye_body()));
    c.fwd = v3_norm(m3_apply(R, f));
    c.upv = v3_norm(m3_apply(R, u));
    c.fov_y = fov;
    return c;
}

// --- Draw --------------------------------------------------------------------------------------

static void set3(const char *name, V3 v)
{
    float f[3] = { (float)v.x, (float)v.y, (float)v.z };
    SetShaderValue(ck_sh, GetShaderLocation(ck_sh, name), f, SHADER_UNIFORM_VEC3);
}
static void set1(const char *name, float v) { SetShaderValue(ck_sh, GetShaderLocation(ck_sh, name), &v, SHADER_UNIFORM_FLOAT); }

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

void cockpit_draw(M3 R, V3 lm_rel, int w, int h)
{
    render_scene_3d_begin_near(w, h, 0.02);
    // Windows: centre, inward normal, area, and incoming radiance (what the
    // window sees: sunlit regolith below the horizon, black sky above).
    V3 wc[3], wn[3];
    float wa[3], wl[9];
    for (int side = -1, k = 0; side <= 1; side += 2, k++) {
        double cy = side * (WIN_YIN * 2 + WIN_YOUT) / 3, cs = (WIN_S0 * 2 + WIN_S1) / 3;
        wc[k] = int_pt(cy, cs);
        wn[k] = N_IN;
        wa[k] = (float)((WIN_YOUT - WIN_YIN) * (WIN_S1 - WIN_S0) / 2);
    }
    wc[2] = v3(CABIN_CEIL, (DW_Y0 + DW_Y1) / 2, (DW_Z0 + DW_Z1) / 2);
    wn[2] = v3(-1, 0, 0);
    wa[2] = (float)((DW_Y1 - DW_Y0) * (DW_Z1 - DW_Z0));
    double sun_up = fmax(0.0, v3_dot(m3_apply(R, sun_b), m3_apply(R, up_b)));
    double ground = 0.11 * 2.6 * fmin(1.0, sun_up * 3.0 + 0.15);
    float glare = 0;
    for (int k = 0; k < 3; k++) {
        V3 out_b = v3_scale(wn[k], -1);
        double g = v3_dot(out_b, up_b);
        double f = fmax(0.0, fmin(1.0, 0.5 - 0.75 * g));
        double L = ground * f;
        wl[3 * k] = (float)(L * 1.0); wl[3 * k + 1] = (float)(L * 0.96); wl[3 * k + 2] = (float)(L * 0.90);
        // Sun shining in through a window: glare on the glass.
        double sd = v3_dot(sun_b, out_b);
        if (sd > 0) glare = fmaxf(glare, (float)sd);
    }
    glare_amt = glare;
    float c[9], a3[3], p9[9], lc[9];
    for (int k = 0; k < 3; k++) {
        c[3 * k] = (float)wc[k].x; c[3 * k + 1] = (float)wc[k].y; c[3 * k + 2] = (float)wc[k].z;
        a3[k] = wa[k];
    }
    SetShaderValueV(ck_sh, GetShaderLocation(ck_sh, "winC"), c, SHADER_UNIFORM_VEC3, 3);
    for (int k = 0; k < 3; k++) { c[3 * k] = (float)wn[k].x; c[3 * k + 1] = (float)wn[k].y; c[3 * k + 2] = (float)wn[k].z; }
    SetShaderValueV(ck_sh, GetShaderLocation(ck_sh, "winN"), c, SHADER_UNIFORM_VEC3, 3);
    SetShaderValueV(ck_sh, GetShaderLocation(ck_sh, "winA"), a3, SHADER_UNIFORM_FLOAT, 3);
    SetShaderValueV(ck_sh, GetShaderLocation(ck_sh, "winL"), wl, SHADER_UNIFORM_VEC3, 3);
    // Floodlights (panel 5 FLOOD knob) and the utility light.
    float fl = cp_flood_level();
    V3 lp[3] = { v3(3.26, -0.55, 0.0), v3(3.26, 0.55, 0.0), v3(3.05, -0.98, -0.40) };
    for (int k = 0; k < 3; k++) {
        p9[3 * k] = (float)lp[k].x; p9[3 * k + 1] = (float)lp[k].y; p9[3 * k + 2] = (float)lp[k].z;
        float g = k < 2 ? fl * 0.38f : fl * 0.10f;
        lc[3 * k] = g; lc[3 * k + 1] = g * 0.93f; lc[3 * k + 2] = g * 0.80f;
    }
    SetShaderValueV(ck_sh, GetShaderLocation(ck_sh, "lampP"), p9, SHADER_UNIFORM_VEC3, 3);
    SetShaderValueV(ck_sh, GetShaderLocation(ck_sh, "lampC"), lc, SHADER_UNIFORM_VEC3, 3);
    set3("sunB", sun_b);
    set1("sunOn", 1.0f);
    set3("eyeB", eye_body());
    set3("shadowC2", SH2_C);
    SetShaderValueMatrix(ck_sh, GetShaderLocation(ck_sh, "shadowVP2"), sh2_vp);
    set1("exposure", render_exposure());
    set1("emitGain", 0.9f);
    set1("glare", glare);

    Matrix M = to_matrix(R, lm_rel);
    rlDisableBackfaceCulling();
    DrawMesh(mesh_static, ck_mat, M);
    if (mesh_dyn.vertexCount > 0) DrawMesh(mesh_dyn, ck_mat, M);
    rlDrawRenderBatchActive();
    rlDisableDepthMask();
    DrawMesh(mesh_glass, ck_mat, M);
    rlDrawRenderBatchActive();
    rlEnableDepthMask();
    rlEnableBackfaceCulling();
    render_scene_3d_end();
}

void cockpit_overlay(int w, int h)
{
    (void)h;
    float s = GetScreenHeight() / 1080.0f;
    if (s < 0.7f) s = 0.7f;
    if (hover) {
        const char *t = cp_tooltip(hover, hover_u, hover_v);
        Vector2 m = MeasureTextEx(font_ui, t, 19 * s, 0.4f);
        float x = mouse_pos.x + 18, y = mouse_pos.y + 14;
        if (x + m.x + 16 > w) x = w - m.x - 16;
        DrawRectangleRounded((Rectangle){ x - 8, y - 5, m.x + 16, m.y + 10 }, 0.3f, 6, (Color){ 10, 12, 14, 220 });
        DrawTextEx(font_ui, t, (Vector2){ x, y }, 19 * s, 0.4f, (Color){ 235, 238, 240, 255 });
    }
    char b[160];
    snprintf(b, sizeof b, "%s STATION   right-drag look   wheel zoom   shift+wheel / L lean   Home window   End panels   F9 swap seat   click switches",
             seat ? "LMP" : "CDR");
    DrawTextEx(font_ui, b, (Vector2){ 18 * s, (float)GetScreenHeight() - 30 * s }, 16 * s, 0.3f, (Color){ 150, 156, 164, 200 });
}
