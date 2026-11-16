// Lunar Module cabin interior.
//
// Geometry is authored in LM body axes (metres, origin at the CG, +X up,
// +Y right, +Z forward), matching lmmodel.c: the two triangular forward
// windows here are the apertures behind the exterior model's windows.
// Panel artwork is vector geometry on each panel's plane (pcanvas.c, drawn
// by cockpit_panels.c). Lighting:
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

#include "input.h"

#include "assets.h"
#include "cockpit.h"
#include "cockpit_int.h"
#include "hud.h"
#include "lm.h"
#include "pcanvas.h"
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
// Forward hatch opening in the lower front wall (z = 1.00): body X range and half width.
#define HATCH_X0 1.40
#define HATCH_X1 1.72
#define HATCH_HW 0.40

#define DW_Y0 (-0.80)
#define DW_Y1 (-0.52)
#define DW_Z0 (-0.35)
#define DW_Z1 (-0.05)

// --- Mesh builder --------------------------------------------------------------------

enum { M_PANEL = 1, M_PAINT, M_METAL, M_BLACK, M_EMIT, M_ETCH, M_GLASS, M_WALL };   // M_WALL: pressure-shell structure

typedef struct {
    float *p, *n, *t;
    unsigned char *c;
    int count, cap;
} Builder;

static Builder SB, DB, TB, CB;   // static opaque, dynamic (hand controllers), transparent, modelled controls
static Builder *B = &SB;
static const float white_u = 0, white_v = 0;   // texture coordinates are unused

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

// Cylinder or cone frustum from a to b. caps: bit 0 closes the a end, bit 1 the b end.
static void cyl(V3 a, V3 b, double ra, double rb, int seg, Color col, int mat, int caps)
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
        if (caps & 1) tri(a, a1, a0, col, mat);
        if (caps & 2) tri(b, b0, b1, col, mat);
    }
}

static void tube(V3 a, V3 b, double ra, double rb, int seg, Color col, int mat, int caps)
{
    cyl(a, b, ra, rb, seg, col, mat, caps ? 3 : 0);
}

// --- Colours ---------------------------------------------------------------------------

static const Color K_WALL = { 150, 158, 170, 255 };     // cabin structure: blue-grey
static const Color K_CEIL = { 214, 208, 182, 255 };     // overhead and upper bulkhead: cream
static const Color K_YELLOW = { 222, 176, 28, 255 };    // handholds
static const Color K_ORANGE = { 204, 122, 40, 255 };    // AOT guard, wire lacing
static const Color K_PANEL = { 104, 112, 126, 255 };    // panel blue-grey
static const Color K_CONSOLE = { 112, 120, 134, 255 };
static const Color K_FRAME = { 30, 31, 33, 255 };
static const Color K_FLOOR = { 118, 124, 134, 255 };
static const Color K_CLOTH = { 222, 220, 210, 255 };    // Beta cloth
static const Color K_BLACK = { 22, 22, 24, 255 };
static const Color K_METAL = { 170, 172, 176, 255 };
static const Color K_BLUE = { 40, 80, 170, 255 };
static const Color K_RED = { 170, 40, 36, 255 };

// --- Panel canvas layers -----------------------------------------------------------------
// One layer per panel (index = panel), plus the LPD numerals etched on the
// Commander's window.
#define LAYER_LPD NPANELS

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
            quad(int_pt(y0, s0), int_pt(y1, s0), int_pt(y1, s1), int_pt(y0, s1), K_WALL, M_WALL);
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
    quad(v3(F, -H, A), v3(F, H, A), v3(C, H, A), v3(C, -H, A), K_WALL, M_WALL);
    // Lower forward wall, with the opening of the forward hatch, and upper bevel.
    quad(v3(F, -H, 1.00), v3(2.00, -H, 1.00), v3(2.00, -HATCH_HW, 1.00), v3(F, -HATCH_HW, 1.00), K_WALL, M_WALL);
    quad(v3(F, HATCH_HW, 1.00), v3(2.00, HATCH_HW, 1.00), v3(2.00, H, 1.00), v3(F, H, 1.00), K_WALL, M_WALL);
    quad(v3(F, -HATCH_HW, 1.00), v3(HATCH_X0, -HATCH_HW, 1.00), v3(HATCH_X0, HATCH_HW, 1.00), v3(F, HATCH_HW, 1.00), K_WALL, M_WALL);
    quad(v3(HATCH_X1, -HATCH_HW, 1.00), v3(2.00, -HATCH_HW, 1.00), v3(2.00, HATCH_HW, 1.00), v3(HATCH_X1, HATCH_HW, 1.00), K_WALL, M_WALL);
    quad(int_pt(-H, 1.199), int_pt(H, 1.199), v3(C, H, 0.10), v3(C, -H, 0.10), K_CEIL, M_WALL);
    // Ceiling with the docking-window opening.
    quad(v3(C, -H, A), v3(C, H, A), v3(C, H, DW_Z0), v3(C, -H, DW_Z0), K_CEIL, M_WALL);
    quad(v3(C, -H, DW_Z1), v3(C, H, DW_Z1), v3(C, H, 0.10), v3(C, -H, 0.10), K_CEIL, M_WALL);
    quad(v3(C, -H, DW_Z0), v3(C, DW_Y0, DW_Z0), v3(C, DW_Y0, DW_Z1), v3(C, -H, DW_Z1), K_CEIL, M_WALL);
    quad(v3(C, DW_Y1, DW_Z0), v3(C, H, DW_Z0), v3(C, H, DW_Z1), v3(C, DW_Y1, DW_Z1), K_CEIL, M_WALL);
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
            tri(c, v3(px[i], side * H, pz[i]), v3(px[j], side * H, pz[j]), K_WALL, M_WALL);
        }
    // Slanted front wall edge closure between the lower wall and the slant.
    quad(v3(2.00, -H, 1.00), v3(2.00, H, 1.00), int_pt(H, 0.01825), int_pt(-H, 0.01825), K_WALL, M_WALL);
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
        // The plate itself; its artwork is the panel canvas, a hair above it.
        quad(a, b, c, d, K_PANEL, M_PANEL);
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
    // Forward hatch frame under the DSKY pedestal (the door itself is dynamic: it opens for the EVA).
    box(v3(1.385, -0.43, 0.97), v3(1.40, 0.43, 0.995), K_FRAME, M_BLACK);
    box(v3(1.72, -0.43, 0.97), v3(1.735, 0.43, 0.995), K_FRAME, M_BLACK);
    box(v3(1.385, -0.43, 0.97), v3(1.735, -0.415, 0.995), K_FRAME, M_BLACK);
    box(v3(1.385, 0.415, 0.97), v3(1.735, 0.43, 0.995), K_FRAME, M_BLACK);

    // Yellow handholds: a grip either side of each station's controllers, a rail on each side wall.
    for (int side = -1; side <= 1; side += 2) {
        for (int k = 0; k < 2; k++) {
            double y = side * (k ? 1.025 : 0.652);
            tube(v3(1.79, y, 0.395), v3(1.93, y, 0.375), 0.012, 0.012, 10, K_YELLOW, M_PAINT, 1);
        }
        double yw = side * 0.985;
        tube(v3(2.30, yw, -0.70), v3(2.30, yw, -0.15), 0.013, 0.013, 10, K_YELLOW, M_PAINT, 1);
        tube(v3(2.30, side * 1.035, -0.70), v3(2.30, yw, -0.70), 0.010, 0.010, 8, K_YELLOW, M_PAINT, 0);
        tube(v3(2.30, side * 1.035, -0.15), v3(2.30, yw, -0.15), 0.010, 0.010, 8, K_YELLOW, M_PAINT, 0);
    }

    // Alignment optical telescope between the windows: eyepiece housing hung
    // from the overhead, behind its orange guard hoop.
    {
        double z = 0.50;
        tube(v3(3.30, 0, z + 0.06), v3(3.02, 0, z + 0.06), 0.060, 0.050, 18, K_FRAME, M_BLACK, 1);     // housing
        tube(v3(3.04, 0, z + 0.04), v3(3.04, 0, z - 0.03), 0.030, 0.026, 14, K_BLACK, M_BLACK, 1);     // eyepiece
        for (int side = -1; side <= 1; side += 2) {
            tube(v3(3.30, side * 0.085, z), v3(3.08, side * 0.085, z - 0.04), 0.011, 0.011, 8, K_BLACK, M_BLACK, 0);   // struts
            tube(v3(3.08, side * 0.085, z - 0.04), v3(2.90, side * 0.085, z - 0.05), 0.013, 0.013, 10, K_ORANGE, M_PAINT, 1);
            tube(v3(2.90, side * 0.085, z - 0.05), v3(2.84, side * 0.045, z - 0.05), 0.013, 0.013, 10, K_ORANGE, M_PAINT, 1);
        }
        tube(v3(2.84, -0.045, z - 0.05), v3(2.84, 0.045, z - 0.05), 0.013, 0.013, 10, K_ORANGE, M_PAINT, 1);
    }

    // Overhead (docking) hatch with its handle.
    tube(v3(CABIN_CEIL - 0.002, 0, -0.30), v3(CABIN_CEIL - 0.03, 0, -0.30), 0.42, 0.40, 40, K_WALL, M_WALL, 1);
    tube(v3(CABIN_CEIL - 0.03, -0.12, -0.30), v3(CABIN_CEIL - 0.03, 0.12, -0.30), 0.015, 0.015, 8, K_METAL, M_METAL, 1);
    for (int k = 0; k < 12; k++) {
        double a = k * 2 * PI_D / 12;
        V3 c = v3(CABIN_CEIL - 0.034, 0.36 * cos(a), -0.30 + 0.36 * sin(a));
        box(v3_sub(c, v3(0.006, 0.012, 0.012)), v3_add(c, v3(0.006, 0.012, 0.012)), K_METAL, M_METAL);
    }

    // Ascent engine cover between the crew stations (aft).
    tube(v3(CABIN_FLOOR, 0, -0.82), v3(1.86, 0, -0.82), 0.44, 0.44, 40, K_WALL, M_WALL, 0);
    tube(v3(1.86, 0, -0.82), v3(1.94, 0, -0.82), 0.44, 0.36, 40, K_WALL, M_WALL, 1);
    box(v3(1.94, -0.12, -0.95), v3(1.96, 0.12, -0.70), K_CLOTH, M_PAINT);   // stowed checklist pad

    // Aft equipment: ECS package, LiOH canisters, hoses, PLSS stowage.
    box(v3(1.95, -0.60, -1.24), v3(2.90, 0.60, -1.05), K_WALL, M_WALL);
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
    }
}

// LPD numerals: lettering on the inner pane beside each 10-degree mark.
static void build_lpd_numerals(void)
{
    const double inner = 0.028;
    V3 up = v3(SX, 0, -SZ);
    pc_begin(LAYER_LPD, pane_pt(0, 0, inner), v3(0, 0.001, 0), v3_scale(up, -0.001), N_IN, 0.0003);
    for (int a = 10; a <= 80; a += 10) {
        V3 p;
        double y, s;
        if (!lpd_point(a, 0, inner, &p, &y, &s)) continue;
        char t[4];
        snprintf(t, sizeof t, "%d", a);
        pc_text(t, (float)((y + 0.016 + 0.004) * 1000), (float)(-s * 1000 - 4.5), 9.0f, 0.3f, (Color){ 8, 8, 8, 215 });
    }
    pc_end();
}

static double hc_pitch, hc_yaw, hc_roll;
void cockpit_set_hand_controls(double pitch, double yaw, double roll) { hc_pitch = pitch; hc_yaw = yaw; hc_roll = roll; }

// --- Modelled controls -------------------------------------------------------------------
// Every switch, knob, button, key and breaker the panel code registers gets
// real geometry standing off its panel. The mesh is only rebuilt when a
// control moves (or is still gliding to its new position).

static float ctl_anim[NPANELS][CP_MAXLEV];
static unsigned char ctl_anim_set[NPANELS][CP_MAXLEV];
static unsigned long long ctl_sig;

static unsigned long long mix(unsigned long long h, const void *data, size_t n)
{
    const unsigned char *p = data;
    for (size_t i = 0; i < n; i++) h = (h ^ p[i]) * 1099511628211ull;
    return h;
}

static void model_toggle(const Lever *l, V3 base, V3 n, V3 upv, V3 uv, double pos)
{
    Color c = { l->r, l->g, l->b, 255 };
    // Mounting nut and threaded bushing.
    V3 nut_top = v3_add(base, v3_scale(n, 0.0024));
    cyl(base, nut_top, 0.0046, 0.0046, 6, (Color){ 150, 152, 156, 255 }, M_METAL, 2);
    cyl(nut_top, v3_add(base, v3_scale(n, 0.0050)), 0.0030, 0.0027, 10, (Color){ 118, 120, 124, 255 }, M_METAL, 2);
    // Bat handle: slim at the pivot, swelling toward a rounded tip.
    double ang = pos * 22 * DEG;
    V3 d = v3_add(v3_scale(n, cos(ang)), v3_scale(upv, sin(ang)));
    V3 piv = v3_add(base, v3_scale(n, 0.0030));
    static const double prof[6][2] = { { 0.0020, 0.00125 }, { 0.0065, 0.00125 }, { 0.0125, 0.00190 },
                                       { 0.0165, 0.00225 }, { 0.0183, 0.00190 }, { 0.0192, 0.00110 } };
    for (int k = 0; k < 5; k++)
        cyl(v3_add(piv, v3_scale(d, prof[k][0])), v3_add(piv, v3_scale(d, prof[k + 1][0])), prof[k][1], prof[k + 1][1], 10, c,
            M_METAL, k == 4 ? 2 : 0);
    if (l->guard) {
        // Wicket guards: a hoop either side, taller than the handle's swing.
        Color g = { 176, 178, 182, 255 };
        for (int s = -1; s <= 1; s += 2) {
            V3 m = v3_add(base, v3_scale(uv, s * 0.0066));
            V3 f0 = v3_add(m, v3_scale(upv, 0.0064)), f1 = v3_add(m, v3_scale(upv, -0.0064));
            V3 t0 = v3_add(f0, v3_scale(n, 0.0130)), t1 = v3_add(f1, v3_scale(n, 0.0130));
            cyl(f0, t0, 0.0008, 0.0008, 6, g, M_METAL, 0);
            cyl(f1, t1, 0.0008, 0.0008, 6, g, M_METAL, 0);
            cyl(v3_add(t0, v3_scale(upv, 0.0008)), v3_add(t1, v3_scale(upv, -0.0008)), 0.0008, 0.0008, 6, g, M_METAL, 3);
        }
    }
}

static void model_knob(const Lever *l, V3 base, V3 n, V3 upv, V3 uv, double ang)
{
    Color c = { l->r, l->g, l->b, 255 };
    double r = l->size * 0.001;
    V3 dir = v3_add(v3_scale(uv, sin(ang)), v3_scale(upv, cos(ang)));
    V3 side = v3_cross(n, dir);
    V3 skirt = v3_add(base, v3_scale(n, 0.0028));
    cyl(base, skirt, r * 1.08, r, 20, c, M_BLACK, 2);
    cyl(skirt, v3_add(base, v3_scale(n, 0.0105)), r * 0.70, r * 0.62, 16, c, M_BLACK, 2);
    // Bar grip across the knob, with the white index line on its pointing half.
    obox(v3_add(base, v3_add(v3_scale(n, 0.0085), v3_scale(dir, r * 0.10))), dir, side, n, r * 1.02, r * 0.24, 0.0045, c, M_BLACK);
    obox(v3_add(base, v3_add(v3_scale(n, 0.01305), v3_scale(dir, r * 0.62))), dir, side, n, r * 0.44, 0.00045, 0.00012,
         (Color){ 240, 240, 236, 255 }, M_PAINT);
}

// Thumbwheel: the top of a ridged wheel standing through its slot.
static void model_thumb(const Lever *l, V3 base, V3 n, V3 upv, V3 uv)
{
    const double R = 0.0090, sink = 0.0062, hw = 0.0029;
    V3 ctr = v3_sub(base, v3_scale(n, sink));
    double tm = acos(sink / R);
    int steps = 12, phase = (int)l->val;
    for (int k = 0; k < steps; k++) {
        double t0 = -tm + 2 * tm * k / steps, t1 = -tm + 2 * tm * (k + 1) / steps;
        int ridge = (k + phase) & 1;
        double rr = R + (ridge ? 0.00035 : 0);
        Color c = ridge ? (Color){ 78, 78, 84, 255 } : (Color){ l->r, l->g, l->b, 255 };
        V3 p0 = v3_add(ctr, v3_add(v3_scale(n, rr * cos(t0)), v3_scale(upv, rr * sin(t0))));
        V3 p1 = v3_add(ctr, v3_add(v3_scale(n, rr * cos(t1)), v3_scale(upv, rr * sin(t1))));
        V3 a0 = v3_sub(p0, v3_scale(uv, hw)), b0 = v3_add(p0, v3_scale(uv, hw));
        V3 a1 = v3_sub(p1, v3_scale(uv, hw)), b1 = v3_add(p1, v3_scale(uv, hw));
        quad(a0, b0, b1, a1, c, M_BLACK);
        // Wheel flanks.
        tri(v3_sub(base, v3_scale(uv, hw)), a0, a1, (Color){ 30, 30, 33, 255 }, M_BLACK);
        tri(v3_add(base, v3_scale(uv, hw)), b1, b0, (Color){ 30, 30, 33, 255 }, M_BLACK);
    }
}

// Push-button or key cap: a block standing `height` proud of the panel. Its
// face and legend are panel-canvas artwork lifted to the same height.
static void model_cap(const Lever *l, V3 base, V3 n, V3 upv, V3 uv, int bezel)
{
    Color c = { l->r, l->g, l->b, 255 };
    double hw = l->w * 0.0005, hh = l->h * 0.0005, ht = l->val * 0.001;
    double in = bezel ? 0 : 0.0003;   // keys: a hair of daylight between neighbours
    obox(v3_add(base, v3_scale(n, ht / 2)), uv, upv, n, hw - in, hh - in, ht / 2, c, M_BLACK);
    if (!bezel) return;
    // Raised bezel round the button; a guarded button gets a tall frame instead.
    for (int g = 0; g <= (l->guard ? 1 : 0); g++) {
        double off = g ? 0.0034 : 0.0012, th = g ? 0.0005 : 0.0006, up = g ? 0.0034 : 0.0009;
        Color fc = g ? (Color){ 178, 180, 184, 255 } : (Color){ 34, 35, 38, 255 };
        int fm = g ? M_METAL : M_BLACK;
        for (int s = -1; s <= 1; s += 2) {
            obox(v3_add(base, v3_add(v3_scale(uv, s * (hw + off - th)), v3_scale(n, up))), uv, upv, n, th, hh + off, up, fc, fm);
            obox(v3_add(base, v3_add(v3_scale(upv, s * (hh + off - th)), v3_scale(n, up))), uv, upv, n, hw + off, th, up, fc, fm);
        }
    }
}

// Circuit breaker: collar and plunger; pulled, the plunger stands out and shows its white band.
static void model_breaker(const Lever *l, V3 base, V3 n, double out)
{
    Color c = { l->r, l->g, l->b, 255 };
    cyl(base, v3_add(base, v3_scale(n, 0.0014)), 0.0045, 0.0040, 10, (Color){ 40, 40, 43, 255 }, M_BLACK, 2);
    cyl(base, v3_add(base, v3_scale(n, 0.0034 + out * 0.0056)), 0.0027, 0.0027, 8, c, M_BLACK, 2);
    if (out > 0.05)
        cyl(v3_add(base, v3_scale(n, 0.0015)), v3_add(base, v3_scale(n, 0.0015 + out * 0.0046)), 0.00285, 0.00285, 8,
            (Color){ 232, 232, 226, 255 }, M_PAINT, 0);
}

// Returns 1 if the mesh was rebuilt.
static int build_controls(float dt)
{
    unsigned long long sig = 1469598103934665603ull;
    int moving = 0;
    for (int pn = 0; pn < NPANELS; pn++) {
        sig = mix(sig, &cp_items[pn].nlev, sizeof(int));
        for (int i = 0; i < cp_items[pn].nlev; i++) {
            const Lever *l = &cp_items[pn].lev[i];
            float f[7] = { (float)l->kind, l->x, l->y, l->val, l->size, l->w, l->h };
            sig = mix(sig, f, sizeof f);
            sig = mix(sig, &l->guard, sizeof l->guard);
            float *a = &ctl_anim[pn][i];
            int glides = l->kind == LEV_TOGGLE || l->kind == LEV_KNOB || l->kind == LEV_CB;
            if (!ctl_anim_set[pn][i] || !glides) { *a = l->val; ctl_anim_set[pn][i] = 1; }
            float d = l->val - *a;
            if (fabsf(d) > 0.004f) { *a += d * fminf(1.0f, dt * 24.0f); moving = 1; }
            else *a = l->val;
        }
    }
    if (!moving && sig == ctl_sig && CB.count > 0) return 0;
    ctl_sig = sig;

    B = &CB;
    CB.count = 0;
    for (int pn = 0; pn < NPANELS; pn++)
    for (int i = 0; i < cp_items[pn].nlev; i++) {
        const Lever *l = &cp_items[pn].lev[i];
        const PanelDef *p = &cp_panels[l->panel];
        V3 n = panel_normal(l->panel);
        V3 upv = v3_norm(v3_sub(p->tl, p->bl));
        V3 uv = v3_norm(v3_sub(p->tr, p->tl));
        V3 base = panel_pt(l->panel, l->x, l->y);
        double a = ctl_anim[pn][i];
        switch (l->kind) {
        case LEV_TOGGLE: model_toggle(l, base, n, upv, uv, a); break;
        case LEV_KNOB: model_knob(l, base, n, upv, uv, a); break;
        case LEV_THUMB: model_thumb(l, base, n, upv, uv); break;
        case LEV_BUTTON: model_cap(l, base, n, upv, uv, 1); break;
        case LEV_KEY: model_cap(l, base, n, upv, uv, 0); break;
        case LEV_CB: model_breaker(l, base, n, a); break;
        }
    }
    B = &SB;
    return 1;
}

// --- Dynamic geometry: hand controllers ------------------------------------------------

static double hatch_open, kneel;   // forward hatch 0..1; crewman's move to it 0..2 (see eva.h)
void cockpit_set_egress(double hatch, double k) { hatch_open = hatch; kneel = k; }

static void build_dynamic(void)
{
    B = &DB;
    DB.count = 0;
    {
        // Forward hatch door: hinged on the LM Pilot's side, swings in and across.
        double a = hatch_open * 100 * DEG;
        V3 hinge = v3((HATCH_X0 + HATCH_X1) / 2, HATCH_HW, 0.985);
        V3 xh = v3(1, 0, 0), wd = v3(0, -cos(a), -sin(a)), in = v3_cross(xh, wd);
        V3 ctr = v3_add(hinge, v3_scale(wd, HATCH_HW));
        Color door = { 150, 158, 170, 255 };
        obox(ctr, xh, wd, in, (HATCH_X1 - HATCH_X0) / 2, HATCH_HW, 0.012, door, M_PAINT);
        for (int k = -1; k <= 1; k++)   // stiffening ribs on the inner face
            obox(v3_add(ctr, v3_add(v3_scale(wd, k * 0.24), v3_scale(in, 0.018))), xh, wd, in, 0.14, 0.012, 0.006, door, M_PAINT);
        obox(v3_add(ctr, v3_add(v3_scale(xh, 0.115), v3_scale(in, 0.018))), xh, wd, in, 0.012, 0.34, 0.006, door, M_PAINT);
        obox(v3_add(ctr, v3_add(v3_scale(xh, -0.115), v3_scale(in, 0.018))), xh, wd, in, 0.012, 0.34, 0.006, door, M_PAINT);
        // Latch handle and its placard.
        V3 h0 = v3_add(ctr, v3_add(v3_scale(wd, 0.20), v3_scale(in, 0.045))), h1 = v3_add(h0, v3_scale(wd, 0.16));
        tube(h0, h1, 0.011, 0.011, 8, K_METAL, M_METAL, 1);
        tube(v3_add(h0, v3_scale(in, -0.03)), h0, 0.008, 0.008, 6, K_METAL, M_METAL, 0);
        tube(v3_add(h1, v3_scale(in, -0.03)), h1, 0.008, 0.008, 6, K_METAL, M_METAL, 0);
        obox(v3_add(ctr, v3_add(v3_scale(wd, -0.16), v3_add(v3_scale(xh, 0.04), v3_scale(in, 0.0135)))), xh, wd, in, 0.045, 0.085, 0.001,
             K_BLACK, M_BLACK);
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

// --- Shaders -------------------------------------------------------------------------------
// The cabin mesh and the panel canvas share one lighting model: sunlight
// through the windows (shadow-mapped), light bounced in through each window,
// the floodlights, and a little ambient.

#define CK_LIGHT_GLSL \
    "uniform sampler2D texture2;\n" \
    "uniform mat4 shadowVP2; uniform vec3 shadowC2; uniform vec3 sunB; uniform float sunOn; uniform vec3 eyeB;\n" \
    "uniform vec3 winC[3]; uniform vec3 winN[3]; uniform float winA[3]; uniform vec3 winL[3];\n" \
    "uniform vec3 lampP[3]; uniform vec3 lampC[3];\n" \
    "uniform float exposure; uniform float emitGain; uniform float glare;\n" \
    "float sunShadow(vec3 p, vec3 n){\n" \
    "  vec4 sp = shadowVP2 * vec4(p - shadowC2 + n * 0.008, 1.0);\n" \
    "  vec3 c = sp.xyz / sp.w * 0.5 + 0.5;\n" \
    "  if (c.x < 0.0 || c.x > 1.0 || c.y < 0.0 || c.y > 1.0) return 0.0;\n" \
    "  float s = 0.0; vec2 t = 1.0 / vec2(textureSize(texture2, 0));\n" \
    "  for (int x=-1;x<=1;x++) for(int y=-1;y<=1;y++){ float d = texture(texture2, c.xy + vec2(x,y)*t*1.2).r; s += (c.z - 0.0015 > d) ? 0.0 : 1.0; }\n" \
    "  return s / 9.0;\n" \
    "}\n" \
    "vec3 cabinLight(vec3 p, vec3 N, vec3 V, vec3 alb, float specK, float shin){\n" \
    "  vec3 col = vec3(0.0);\n" \
    "  float NL = max(dot(N, sunB), 0.0) * sunOn;\n" \
    "  if (NL > 0.0) {\n" \
    "    float sh = sunShadow(p, N);\n" \
    "    vec3 H = normalize(sunB + V);\n" \
    "    col += (alb * NL + specK * pow(max(dot(N, H), 0.0), shin) * NL * 4.0) * sh * vec3(1.0, 0.985, 0.96) * 2.8;\n" \
    "  }\n" \
    "  for (int i = 0; i < 3; i++) {\n" \
    "    vec3 w = winC[i] - p; float d2 = dot(w, w); vec3 wn = w * inversesqrt(d2);\n" \
    "    float f = winA[i] * max(dot(N, wn), 0.0) * max(dot(-wn, winN[i]), 0.0) / (3.14159 * d2 + winA[i]);\n" \
    "    col += alb * winL[i] * f * 6.0;\n" \
    "  }\n" \
    "  for (int i = 0; i < 3; i++) {\n" \
    "    vec3 l = lampP[i] - p; float d2 = dot(l, l); l *= inversesqrt(d2);\n" \
    "    vec3 H = normalize(l + V);\n" \
    "    float nl = max(dot(N, l), 0.0);\n" \
    "    col += lampC[i] * (alb * nl + specK * pow(max(dot(N, H), 0.0), shin) * nl) / (d2 + 0.06);\n" \
    "  }\n" \
    "  return col + alb * vec3(0.022, 0.023, 0.025);\n" \
    "}\n"

static const char *ck_vs =
    "in vec3 vertexPosition; in vec3 vertexNormal; in vec4 vertexColor;\n"
    "uniform mat4 mvp; uniform mat4 matModel;\n"
    "out vec3 bodyPos; out vec3 bodyN; out vec4 vcol; out vec3 camPos;\n"
    "void main(){ bodyPos = vertexPosition; bodyN = vertexNormal; vcol = vertexColor;\n"
    "  camPos = (matModel * vec4(vertexPosition, 1.0)).xyz; gl_Position = mvp * vec4(vertexPosition, 1.0); }\n";

static const char *ck_fs =
    "in vec3 bodyPos; in vec3 bodyN; in vec4 vcol; in vec3 camPos;\n"
    CK_LIGHT_GLSL
    "uniform sampler2D cabinMap; uniform sampler2D wallMap;\n"
    "out vec4 finalColor;\n"
    "float hash12(vec2 p){ vec3 p3 = fract(vec3(p.xyx) * 0.1031); p3 += dot(p3, p3.yzx + 33.33); return fract((p3.x + p3.y) * p3.z); }\n"
    "float vnoise(vec2 p){ vec2 i = floor(p), f = fract(p); f = f*f*(3.0-2.0*f);\n"
    "  return mix(mix(hash12(i), hash12(i+vec2(1,0)), f.x), mix(hash12(i+vec2(0,1)), hash12(i+vec2(1,1)), f.x), f.y); }\n"
    "vec4 triMap(sampler2D t, vec3 p, vec3 n, out vec3 bump){\n"
    "  vec3 w = abs(n); w = w * w; w = w * w * w; w /= (w.x + w.y + w.z);\n"
    "  vec3 dx = dFdx(p), dy = dFdy(p); vec4 s = vec4(0.0); bump = vec3(0.0);\n"
    "  if (w.x > 0.004) { vec4 c = textureGrad(t, p.yz, dx.yz, dy.yz); s += c * w.x; bump += vec3(0.0, c.rg * 2.0 - 1.0) * w.x; }\n"
    "  if (w.y > 0.004) { vec4 c = textureGrad(t, p.xz, dx.xz, dy.xz); s += c * w.y; bump += vec3(c.r * 2.0 - 1.0, 0.0, c.g * 2.0 - 1.0) * w.y; }\n"
    "  if (w.z > 0.004) { vec4 c = textureGrad(t, p.xy, dx.xy, dy.xy); s += c * w.z; bump += vec3(c.rg * 2.0 - 1.0, 0.0) * w.z; }\n"
    "  return s;\n"
    "}\n"
    "void main(){\n"
    "  gl_FragDepth = clamp(length(camPos) / 6.0, 0.0, 1.0) * 0.03;\n"
    "  int mat = int(vcol.a * 255.0 + 0.5);\n"
    "  vec3 N = normalize(bodyN); vec3 V = normalize(eyeB - bodyPos);\n"
    "  if (dot(N, V) < 0.0) N = -N;\n"
    "  vec3 alb = vcol.rgb;\n"
    "  if (mat == 5) { finalColor = sceneOut(alb * (0.3 + 3.0 * glare) * exposure, 1.0); return; }\n"
    "  if (mat == 6) { finalColor = sceneOut(vec3(0.03) * exposure, 0.85); return; }\n"
    "  float specK = mat == 3 ? 0.55 : mat == 1 ? 0.06 : mat == 4 ? 0.05 : 0.03;\n"
    "  float shin = mat == 3 ? 80.0 : 14.0;\n"
    "  // Painted surfaces and the pressure shell take their relief, wear and sheen from the baked maps.\n"
    "  if (mat == 2 || mat == 4 || mat == 8) {\n"
    "    vec3 b; vec4 sm;\n"
    "    if (mat == 8) sm = triMap(wallMap, bodyPos / 0.90, N, b); else sm = triMap(cabinMap, bodyPos / 0.60, N, b);\n"
    "    alb *= sm.a * 2.0; N = normalize(N + b); specK *= clamp(1.0 - 3.0 * (sm.b - 0.5), 0.4, 2.2);\n"
    "  }\n"
    "  vec3 col = cabinLight(bodyPos, N, V, alb, specK, shin);\n"
    "  if (mat == 7) {\n"
    "    float smudge = vnoise(bodyPos.yz * 60.0) * 0.6 + vnoise(bodyPos.yz * 230.0) * 0.4;\n"
    "    float a = 0.012 + glare * (0.03 + 0.18 * smudge);\n"
    "    finalColor = sceneOut(vec3(0.9, 0.92, 0.95) * (0.02 + glare * 1.5) * exposure, a);\n"
    "    return;\n"
    "  }\n"
    "  finalColor = sceneOut(col * exposure, 1.0);\n"
    "}\n";

// Panel canvas (pcanvas.c): every quad evaluates its own edge, so shapes and
// lettering are antialiased analytically at any distance.
static const char *pn_vs =
    "in vec3 vertexPosition; in vec2 vertexTexCoord; in vec3 vertexNormal; in vec4 vertexColor; in vec4 vertexTangent; in vec2 vertexTexCoord2;\n"
    "uniform mat4 mvp; uniform mat4 matModel;\n"
    "out vec3 bodyPos; out vec2 uv; out vec3 glow; out vec4 vcol; out vec4 par; out vec2 uv2; out vec3 camPos;\n"
    "void main(){ bodyPos = vertexPosition; uv = vertexTexCoord; glow = vertexNormal; vcol = vertexColor; par = vertexTangent; uv2 = vertexTexCoord2;\n"
    "  camPos = (matModel * vec4(vertexPosition, 1.0)).xyz; gl_Position = mvp * vec4(vertexPosition, 1.0); }\n";

static const char *pn_fs =
    "in vec3 bodyPos; in vec2 uv; in vec3 glow; in vec4 vcol; in vec4 par; in vec2 uv2; in vec3 camPos;\n"
    CK_LIGHT_GLSL
    "uniform sampler2D texture0; uniform sampler2D texture1;\n"   // SDF font, picture (FDAI ball)
    "uniform vec3 planeN; uniform float fontTexels; uniform float fontSlope;\n"
    "out vec4 finalColor;\n"
    "void main(){\n"
    "  gl_FragDepth = clamp(length(camPos) / 6.0, 0.0, 1.0) * 0.03;\n"
    "  int kind = int(par.w + 0.5);\n"
    "  vec3 alb = vcol.rgb, emis = glow; float cov = 1.0;\n"
    "  if (kind == 1) {\n"   // rounded rectangle (also circles, lines), optionally an outline
    "    vec2 q = abs(uv) - (par.xy - par.z);\n"
    "    float d = length(max(q, 0.0)) + min(max(q.x, q.y), 0.0) - par.z;\n"
    "    if (uv2.x > 0.0) d = max(d, -(d + uv2.x));\n"
    "    float aa = max(length(fwidth(uv)) * 0.7071, 1e-5);\n"
    "    cov = clamp(0.5 - d / aa, 0.0, 1.0);\n"
    "  } else if (kind == 3) {\n"   // glyph: distance field, one screen pixel of edge
    "    float sd = texture(texture0, uv).r;\n"
    "    float tpp = length(fwidth(uv)) * 0.7071 * fontTexels;\n"
    "    cov = clamp((sd - 0.5) / clamp(fontSlope * tpp, 1e-4, 0.5) + 0.5 + 0.10 * clamp(tpp / 6.0 - 0.3, 0.0, 1.0), 0.0, 1.0);\n"
    "  } else if (kind == 4) {\n"
    "    vec4 t = texture(texture1, uv); alb = t.rgb; cov = t.a; emis = t.rgb * glow.x;\n"
    "  }\n"
    "  cov *= vcol.a;\n"
    "  if (cov < 0.004) discard;\n"
    "  vec3 N = planeN; vec3 V = normalize(eyeB - bodyPos);\n"
    "  if (dot(N, V) < 0.0) N = -N;\n"
    "  vec3 col = cabinLight(bodyPos, N, V, alb, 0.06, 14.0) + emis * emitGain;\n"
    "  finalColor = sceneOut(col * exposure, cov);\n"
    "}\n";

static Shader ck_sh, pn_sh;
static Material ck_mat, ck_depth_mat, pn_mat;
static Mesh mesh_static, mesh_dyn, mesh_glass, mesh_ctl;
static int dyn_cap, ctl_cap;

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
    pc_init();

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
    ctl_cap = 320000;
    mesh_ctl = upload(&CB, 1, ctl_cap);
    mesh_ctl.vertexCount = 0;
    build_lpd_numerals();

    sh2_fbo = rlLoadFramebuffer();
    rlEnableFramebuffer(sh2_fbo);
    sh2_tex = rlLoadTextureDepth(SHADOW2, SHADOW2, false);
    rlFramebufferAttach(sh2_fbo, sh2_tex, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);
    if (!rlFramebufferComplete(sh2_fbo)) TraceLog(LOG_WARNING, "cockpit: shadow FBO incomplete");
    rlDisableFramebuffer();
    Texture2D shadow = { sh2_tex, SHADOW2, SHADOW2, 1, PIXELFORMAT_UNCOMPRESSED_GRAYSCALE };

    ck_sh = render_load_shader(ck_vs, ck_fs);
    ck_sh.locs[SHADER_LOC_MAP_ROUGHNESS] = GetShaderLocation(ck_sh, "cabinMap");
    ck_sh.locs[SHADER_LOC_MAP_OCCLUSION] = GetShaderLocation(ck_sh, "wallMap");
    ck_mat = LoadMaterialDefault();
    ck_mat.shader = ck_sh;
    ck_mat.maps[MATERIAL_MAP_NORMAL].texture = shadow;
    ck_mat.maps[MATERIAL_MAP_ROUGHNESS].texture = assets_texture("cabin");
    ck_mat.maps[MATERIAL_MAP_OCCLUSION].texture = assets_texture("cabin_wall");

    pn_sh = render_load_shader(pn_vs, pn_fs);
    pn_mat = LoadMaterialDefault();
    pn_mat.shader = pn_sh;
    pn_mat.maps[MATERIAL_MAP_ALBEDO].texture = pc_font_texture();
    pn_mat.maps[MATERIAL_MAP_METALNESS].texture = hud_fdai_texture();
    pn_mat.maps[MATERIAL_MAP_NORMAL].texture = shadow;
    float texels = (float)pc_font_texture().width, slope = pc_font_slope();
    SetShaderValue(pn_sh, GetShaderLocation(pn_sh, "fontTexels"), &texels, SHADER_UNIFORM_FLOAT);
    SetShaderValue(pn_sh, GetShaderLocation(pn_sh, "fontSlope"), &slope, SHADER_UNIFORM_FLOAT);

    ck_depth_mat = LoadMaterialDefault();
    ck_depth_mat.shader = *render_depth_shader();
    TraceLog(LOG_INFO, "cockpit: %d static + %d glass triangles", mesh_static.triangleCount, mesh_glass.triangleCount);
}

void cockpit_shutdown(void)
{
    pc_shutdown();
    UnloadShader(ck_sh);
    UnloadShader(pn_sh);
}

// --- Per-frame -------------------------------------------------------------------------------

static V3 sun_b, up_b;
static M3 cur_R;
static float glare_amt;

void cockpit_update(float dt) { cp_panels_update(dt); }

// Re-record the panel artwork. The flight instruments and the DSKY are
// redrawn every frame; the other panels take turns (one every few frames),
// and all of them at once after a switch was operated.
static void record_panels(void)
{
    static unsigned frame_no;
    int quads = 0;
    for (int i = 0; i < NPANELS; i++) {
        int fast = i == P1 || i == P2 || i == P4 || (cp_live_panels >> i & 1);
        if (!fast && !cp_dirty && pc_layer_ready(i) && (frame_no + (unsigned)i) % 6 != 0) continue;
        const PanelDef *p = &cp_panels[i];
        pc_begin(i, p->tl, v3_scale(v3_sub(p->tr, p->tl), 1.0 / p->w_mm), v3_scale(v3_sub(p->bl, p->tl), 1.0 / p->h_mm),
                 panel_normal(i), 0.0004);
        cp_draw_panel(i);
        pc_end();
        quads += pc_layer_quads(i);
    }
    if (frame_no == 0) TraceLog(LOG_INFO, "cockpit: panel canvas %d quads", quads);
    cp_dirty = 0;
    frame_no++;
}

void cockpit_prepare(M3 R, V3 sun_site, V3 up_site, float dt)
{
    cur_R = R;
    M3 Rt = m3_transpose(R);
    sun_b = v3_norm(m3_apply(Rt, sun_site));
    up_b = v3_norm(m3_apply(Rt, up_site));

    record_panels();

    // Modelled controls (rebuilt only when something moved), then the hand controllers.
    if (build_controls(dt)) {
        int nc = CB.count < ctl_cap ? CB.count : ctl_cap;
        UpdateMeshBuffer(mesh_ctl, 0, CB.p, nc * 3 * sizeof(float), 0);
        UpdateMeshBuffer(mesh_ctl, 1, CB.t, nc * 2 * sizeof(float), 0);
        UpdateMeshBuffer(mesh_ctl, 2, CB.n, nc * 3 * sizeof(float), 0);
        UpdateMeshBuffer(mesh_ctl, 3, CB.c, nc * 4, 0);
        mesh_ctl.vertexCount = nc;
        mesh_ctl.triangleCount = nc / 3;
        static int said;
        if (!said) { said = 1; TraceLog(LOG_INFO, "cockpit: %d triangles of modelled controls", nc / 3); }
    }
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
static double duck, duck_to;   // stepping back and down, to see panel 3 under the overhang of panels 1/2
static const Hot *hover;
static float hover_u, hover_v;
static Vector2 mouse_pos;
static int view_w = 1, view_h = 1;
static V3 eye_off;          // headset: the eye being drawn, relative to the seat's eye point (body frame)
static double pick_ray(V3 e, V3 d);

static V3 eye_body(void)
{
    V3 e = seat ? EYE_LMP : EYE_CDR;
    // Leaning toward the window (and down a little); or a step back with the head lowered.
    e = v3_add(e, v3(-0.24 * duck, 0, -0.25 * duck));
    if (kneel > 0) {
        // Going out: down on the knees facing the hatch, then head-first into the opening.
        V3 k1 = v3(1.86, 0.0, -0.25), k2 = v3(1.58, 0.0, 0.82);
        double a = fmin(kneel, 1.0), b = fmax(kneel - 1.0, 0.0);
        e = v3_add(v3_scale(e, 1 - a), v3_scale(k1, a));
        return v3_add(v3_scale(e, 1 - b), v3_scale(k2, b));
    }
    return v3_add(e, v3_add(v3_scale(N_IN, -lean * 0.14), v3(-lean * 0.05, 0, 0)));
}

static V3 look_dir_body(void)
{
    double y = head_yaw + (seat ? 6 * DEG : -6 * DEG);   // the window centre is slightly outboard
    V3 d = v3_norm(v3(sin(head_pitch), cos(head_pitch) * sin(y), cos(head_pitch) * cos(y)));
    if (kneel > 0) {
        V3 to_hatch = v3_norm(v3_sub(v3((HATCH_X0 + HATCH_X1) / 2, 0, 1.6), eye_body()));
        double a = fmin(kneel, 1.0);
        d = v3_norm(v3_add(v3_scale(d, 1 - a), v3_scale(to_hatch, a)));
    }
    return d;
}

void cockpit_set_seat(int lmp) { seat = lmp ? 1 : 0; }
// Headset: the seat's view looks straight ahead along the cabin, level; the head does the rest.
void cockpit_vr_level(void) { head_yaw = seat ? -6 * DEG : 6 * DEG; head_pitch = 0; lean = 0; }
void cockpit_set_view(double yaw_deg, double pitch_deg) { head_yaw = yaw_deg * DEG; head_pitch = pitch_deg * DEG; }
void cockpit_set_fov(double fov_deg) { fov = fov_deg * DEG; }

int cockpit_panel_count(void) { return NPANELS; }

// Aim the head at a point on a panel (u, v = 0..1 across and down it).
// fov_deg <= 0 frames the whole panel for the given aspect ratio.
void cockpit_look_at_panel(int panel, double u, double v, double fov_deg, double aspect)
{
    if (panel < 0 || panel >= NPANELS) return;
    const PanelDef *p = &cp_panels[panel];
    V3 eu = v3_sub(p->tr, p->tl), ev = v3_sub(p->bl, p->tl);
    V3 e = eye_body();
    V3 d = v3_norm(v3_sub(v3_add(p->tl, v3_add(v3_scale(eu, u), v3_scale(ev, v))), e));
    head_pitch = asin(fmax(-1.0, fmin(1.0, d.x)));
    head_yaw = atan2(d.y, d.z) - (seat ? 6 * DEG : -6 * DEG);
    if (fov_deg > 0) { fov = fov_deg * DEG; return; }
    V3 f = look_dir_body();
    V3 r = v3_norm(v3_cross(f, v3(1, 0, 0)));
    V3 up = v3_cross(r, f);
    double need = 0;
    for (int k = 0; k < 4; k++) {
        V3 c = v3_sub(v3_add(p->tl, v3_add(v3_scale(eu, k & 1), v3_scale(ev, k >> 1))), e);
        double z = fmax(0.05, v3_dot(c, f));
        need = fmax(need, fmax(fabs(v3_dot(c, up)) / z, fabs(v3_dot(c, r)) / z / aspect));
    }
    fov = fmin(110 * DEG, 2 * atan(need * 1.08));
}

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
    if (IsKeyPressed(KEY_HOME)) { head_yaw = 0; head_pitch = -36 * DEG; fov = 80 * DEG; lean = 0; duck_to = 0; }
    if (IsKeyPressed(KEY_END)) {
        // Lower panels: step back so that panel 3 shows from under panels 1 and 2. Again: back to standing.
        duck_to = duck_to > 0.5 && head_pitch < -50 * DEG ? 0 : 1;
        head_yaw = seat ? -30 * DEG : 30 * DEG; head_pitch = -58 * DEG; fov = 62 * DEG; lean = 0;
    }
    static int duck_env = -1;
    if (duck_env < 0) { duck_env = getenv("AGC_DUCK") != NULL; if (duck_env) duck = duck_to = 1; }   // debugging
    duck += (duck_to - duck) * fmin(1.0, GetFrameTime() * 9.0);
    if (IsKeyPressed(KEY_F9)) seat = !seat;
    if (IsKeyPressed(KEY_L)) lean = lean > 0.5 ? 0 : 1;

#ifndef AGC_VR
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
    pick_ray(e, d);
    if (hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) cp_click(hover, hover_u, hover_v);
#endif
}

// Headset: the hand controller points at the panels instead of the mouse.
// origin and dir in the body frame; returns the distance to the panel hit (0 = none).
double cockpit_pointer(V3 origin_b, V3 dir_b, int pressed)
{
    hover = NULL;
    double dist = pick_ray(origin_b, v3_norm(dir_b));
    if (hover && pressed) cp_click(hover, hover_u, hover_v);
    return dist;
}

void cockpit_set_eye_offset(V3 off_b) { eye_off = off_b; }

// The control under a ray (body frame): sets hover; returns the distance to the panel, 0 = no panel.
static double pick_ray(V3 e, V3 d)
{
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
    if (bp < 0) return 0;
    for (int i = 0; i < cp_items[bp].nhot; i++) {
        const Hot *hh = &cp_items[bp].hot[i];
        if (CheckCollisionPointRec((Vector2){ bu, bv }, hh->r)) {
            hover = hh;
            hover_u = bu;
            hover_v = bv;
            break;
        }
    }
    return best;
}

ViewCam cockpit_camera(V3 lm_pos, M3 R)
{
    ViewCam c = { 0 };
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

static void light_uniforms(Shader sh, const float *wc, const float *wn, const float *wa, const float *wl, const float *lp,
                           const float *lc, float glare)
{
    SetShaderValueV(sh, GetShaderLocation(sh, "winC"), wc, SHADER_UNIFORM_VEC3, 3);
    SetShaderValueV(sh, GetShaderLocation(sh, "winN"), wn, SHADER_UNIFORM_VEC3, 3);
    SetShaderValueV(sh, GetShaderLocation(sh, "winA"), wa, SHADER_UNIFORM_FLOAT, 3);
    SetShaderValueV(sh, GetShaderLocation(sh, "winL"), wl, SHADER_UNIFORM_VEC3, 3);
    SetShaderValueV(sh, GetShaderLocation(sh, "lampP"), lp, SHADER_UNIFORM_VEC3, 3);
    SetShaderValueV(sh, GetShaderLocation(sh, "lampC"), lc, SHADER_UNIFORM_VEC3, 3);
    V3 e = v3_add(eye_body(), eye_off);
    float sun[3] = { (float)sun_b.x, (float)sun_b.y, (float)sun_b.z }, eye[3] = { (float)e.x, (float)e.y, (float)e.z };
    float sc[3] = { (float)SH2_C.x, (float)SH2_C.y, (float)SH2_C.z };
    float one = 1.0f, expo = render_exposure(), gain = 0.9f;
    SetShaderValue(sh, GetShaderLocation(sh, "sunB"), sun, SHADER_UNIFORM_VEC3);
    SetShaderValue(sh, GetShaderLocation(sh, "sunOn"), &one, SHADER_UNIFORM_FLOAT);
    SetShaderValue(sh, GetShaderLocation(sh, "eyeB"), eye, SHADER_UNIFORM_VEC3);
    SetShaderValue(sh, GetShaderLocation(sh, "shadowC2"), sc, SHADER_UNIFORM_VEC3);
    SetShaderValueMatrix(sh, GetShaderLocation(sh, "shadowVP2"), sh2_vp);
    SetShaderValue(sh, GetShaderLocation(sh, "exposure"), &expo, SHADER_UNIFORM_FLOAT);
    SetShaderValue(sh, GetShaderLocation(sh, "emitGain"), &gain, SHADER_UNIFORM_FLOAT);
    SetShaderValue(sh, GetShaderLocation(sh, "glare"), &glare, SHADER_UNIFORM_FLOAT);
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

static void draw_layer(int layer, V3 n, Matrix M)
{
    float f[3] = { (float)n.x, (float)n.y, (float)n.z };
    SetShaderValue(pn_sh, GetShaderLocation(pn_sh, "planeN"), f, SHADER_UNIFORM_VEC3);
    pc_draw_layer(layer, pn_mat, M);
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
    float c9[9], n9[9], p9[9], lc[9];
    for (int k = 0; k < 3; k++) {
        c9[3 * k] = (float)wc[k].x; c9[3 * k + 1] = (float)wc[k].y; c9[3 * k + 2] = (float)wc[k].z;
        n9[3 * k] = (float)wn[k].x; n9[3 * k + 1] = (float)wn[k].y; n9[3 * k + 2] = (float)wn[k].z;
    }
    // Floodlights (panel 5 FLOOD knob) and the utility light.
    float fl = cp_flood_level();
    V3 lp[3] = { v3(3.26, -0.55, 0.0), v3(3.26, 0.55, 0.0), v3(3.05, -0.98, -0.40) };
    for (int k = 0; k < 3; k++) {
        p9[3 * k] = (float)lp[k].x; p9[3 * k + 1] = (float)lp[k].y; p9[3 * k + 2] = (float)lp[k].z;
        float g = k < 2 ? fl * 0.38f : fl * 0.10f;
        lc[3 * k] = g; lc[3 * k + 1] = g * 0.93f; lc[3 * k + 2] = g * 0.80f;
    }
    light_uniforms(ck_sh, c9, n9, wa, wl, p9, lc, glare);
    light_uniforms(pn_sh, c9, n9, wa, wl, p9, lc, glare);

    Matrix M = to_matrix(R, lm_rel);
    rlDisableBackfaceCulling();
    DrawMesh(mesh_static, ck_mat, M);
    if (mesh_dyn.vertexCount > 0) DrawMesh(mesh_dyn, ck_mat, M);
    if (mesh_ctl.vertexCount > 0) DrawMesh(mesh_ctl, ck_mat, M);
    rlDrawRenderBatchActive();
    // Panel artwork: coplanar quads in painter's order, tested against the
    // cabin's depth but not written to it.
    rlDisableDepthMask();
    for (int i = 0; i < NPANELS; i++) draw_layer(i, panel_normal(i), M);
    rlDrawRenderBatchActive();
    DrawMesh(mesh_glass, ck_mat, M);
    draw_layer(LAYER_LPD, N_IN, M);
    rlDrawRenderBatchActive();
    rlEnableDepthMask();
    rlEnableBackfaceCulling();
    render_scene_3d_end();
}

void cockpit_overlay(int w, int h)
{
    float s = h / 1080.0f;
    if (s < 0.7f) s = 0.7f;
#ifdef AGC_VR
    mouse_pos = (Vector2){ w * 0.5f - 160 * s, h * 0.70f };   // the tooltip sits low in the floating display
#endif
    if (hover) {
        const char *t = cp_tooltip(hover, hover_u, hover_v);
        Vector2 m = MeasureTextEx(font_ui, t, 19 * s, 0.4f);
        float x = mouse_pos.x + 18, y = mouse_pos.y + 14;
        if (x + m.x + 16 > w) x = w - m.x - 16;
        DrawRectangleRounded((Rectangle){ x - 8, y - 5, m.x + 16, m.y + 10 }, 0.3f, 6, (Color){ 10, 12, 14, 220 });
        DrawTextEx(font_ui, t, (Vector2){ x, y }, 19 * s, 0.4f, (Color){ 235, 238, 240, 255 });
    }
    // Hints: when the marked control is out of view, an arrow at the screen
    // edge says which way to look.
    int hp;
    float hx, hy;
    const char *ht;
    if (cp_hint_point(&hp, &hx, &hy, &ht)) {
        V3 e = eye_body(), f = look_dir_body();
        V3 r = v3_norm(v3_cross(f, v3(1, 0, 0)));
        V3 u = v3_cross(r, f);
        V3 d = v3_sub(panel_pt(hp, hx, hy), e);
        double z = v3_dot(d, f), th = tan(fov / 2), aspect = (double)w / h;
        double px = v3_dot(d, r), py = v3_dot(d, u);
        double sx = z > 1e-6 ? px / (z * th * aspect) : 9, sy = z > 1e-6 ? py / (z * th) : 9;
        // In view but behind another panel (panel 3's top rows hide under panels 1 and 2).
        int hidden = 0;
        V3 tgt = v3_add(e, d);
        for (int i = 0; i < NPANELS && !hidden; i++) {
            if (i == hp) continue;
            const PanelDef *q = &cp_panels[i];
            V3 n = panel_normal(i);
            double den = v3_dot(d, n);
            if (fabs(den) < 1e-9) continue;
            double t = v3_dot(v3_sub(q->tl, e), n) / den;
            if (t <= 0.01 || t >= 0.99) continue;
            V3 o = v3_sub(v3_add(e, v3_scale(d, t)), q->tl), eu = v3_sub(q->tr, q->tl), ev = v3_sub(q->bl, q->tl);
            double uu = v3_dot(eu, eu), vv = v3_dot(ev, ev), uvd = v3_dot(eu, ev), det = uu * vv - uvd * uvd;
            double a = (v3_dot(o, eu) * vv - v3_dot(o, ev) * uvd) / det, b2 = (v3_dot(o, ev) * uu - v3_dot(o, eu) * uvd) / det;
            hidden = a >= 0 && a <= 1 && b2 >= 0 && b2 <= 1;
        }
        (void)tgt;
        if (hidden && fabs(sx) <= 0.9 && fabs(sy) <= 0.9) {
            static char hb[160];
            snprintf(hb, sizeof hb, "%s   (hidden under the main panel: press End to step back)", ht);
            Vector2 m = MeasureTextEx(font_ui, hb, 18 * s, 0.4f);
            float cx = (float)((sx * 0.5 + 0.5) * w), cy = (float)((0.5 - sy * 0.5) * h);
            float tx = cx - m.x / 2, ty = cy - 40 * s;
            if (tx < 8) tx = 8;
            if (tx + m.x + 8 > w) tx = w - m.x - 8;
            float p = 0.5f + 0.5f * sinf((float)GetTime() * 6.0f);
            DrawRing((Vector2){ cx, cy }, 12 * s + 4 * p, 15 * s + 4 * p, 0, 360, 32, (Color){ 90, 255, 150, (unsigned char)(150 + 100 * p) });
            DrawRectangleRounded((Rectangle){ tx - 8, ty - 5, m.x + 16, m.y + 10 }, 0.3f, 6, (Color){ 5, 24, 14, 225 });
            DrawTextEx(font_ui, hb, (Vector2){ tx, ty }, 18 * s, 0.4f, (Color){ 150, 255, 190, 255 });
        }
        if (fabs(sx) > 0.9 || fabs(sy) > 0.9) {
            double len = sqrt(px * px + py * py);
            double dx = len > 1e-9 ? px / len : 0, dy = len > 1e-9 ? -py / len : 1;   // screen y runs down
            double mx = w / 2.0 - 90 * s, my = h / 2.0 - 90 * s;
            double k = fmin(fabs(dx) > 1e-9 ? mx / fabs(dx) : 1e9, fabs(dy) > 1e-9 ? my / fabs(dy) : 1e9);
            Vector2 c = { (float)(w / 2.0 + dx * k), (float)(h / 2.0 + dy * k) };
            float p = 0.5f + 0.5f * sinf((float)GetTime() * 6.0f), a = 34 * s;
            Color g = { 90, 255, 150, (unsigned char)(170 + 85 * p) };
            Vector2 tip = { c.x + (float)dx * a * (1 + 0.3f * p), c.y + (float)dy * a * (1 + 0.3f * p) };
            Vector2 l1 = { c.x - (float)dy * a * 0.7f, c.y + (float)dx * a * 0.7f };
            Vector2 l2 = { c.x + (float)dy * a * 0.7f, c.y - (float)dx * a * 0.7f };
            DrawTriangle(tip, l2, l1, g);
            DrawTriangle(tip, l1, l2, g);
            Vector2 m = MeasureTextEx(font_ui, ht, 18 * s, 0.4f);
            float tx = c.x - (float)dx * (a + 10 * s) - m.x / 2, ty = c.y - (float)dy * (a + 16 * s) - m.y / 2;
            if (tx < 8) tx = 8;
            if (tx + m.x + 8 > w) tx = w - m.x - 8;
            DrawRectangleRounded((Rectangle){ tx - 8, ty - 5, m.x + 16, m.y + 10 }, 0.3f, 6, (Color){ 5, 24, 14, 225 });
            DrawTextEx(font_ui, ht, (Vector2){ tx, ty }, 18 * s, 0.4f, (Color){ 150, 255, 190, 255 });
        }
    }
#ifndef AGC_VR
    char b[160];
    snprintf(b, sizeof b, "%s STATION   right-drag look   wheel zoom   shift+wheel / L lean   Home window   End lower panels (steps back)   F9 swap seat   click switches",
             seat ? "LMP" : "CDR");
    DrawTextEx(font_ui, b, (Vector2){ 18 * s, (float)h - 30 * s }, 16 * s, 0.3f, (Color){ 150, 156, 164, 200 });
#endif
}
