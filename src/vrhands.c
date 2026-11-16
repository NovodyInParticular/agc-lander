// The Quest 3 controllers (Touch Plus) drawn where the player holds them:
// the thumbsticks tilt, the buttons go down, the trigger and grip follow the
// fingers, and the button the current hint asks for glows green.
//
// Built in the OpenXR grip frame (origin in the palm, -z along the pointing
// finger, +y up the back of the hand, +x to the right), right-hand layout;
// the left controller is the mirror image (x flipped) with X / Y and Menu.

#if defined(AGC_VR)

#include <math.h>
#include <string.h>

#include "raylib.h"
#include "rlgl.h"

#include "hints.h"
#include "render.h"
#include "vr.h"
#include "vrhands.h"

static Shader sh;
static int ready;

static const char *vs =
    "in vec3 vertexPosition; in vec3 vertexNormal; in vec4 vertexColor;\n"
    "uniform mat4 mvp; out vec3 pos; out vec3 nrm; out vec4 col;\n"
    "void main(){ pos = vertexPosition; nrm = vertexNormal; col = vertexColor; gl_Position = mvp * vec4(vertexPosition, 1.0); }\n";
// Colours arrive as display (sRGB) values; alpha < 0.5 marks a glowing part.
static const char *fs =
    "in vec3 pos; in vec3 nrm; in vec4 col; uniform vec3 lightDir; uniform float pulse; out vec4 finalColor;\n"
    "void main(){\n"
    "  gl_FragDepth = clamp(length(pos) / 6.0, 0.0, 1.0) * 0.03;\n"   // the cockpit's depth: panels hide the hands correctly
    "  vec3 n = normalize(nrm); vec3 v = normalize(-pos); if (dot(n, v) < 0.0) n = -n;\n"
    "  vec3 c = pow(col.rgb, vec3(2.2));\n"
    "  float l = 0.30 + 0.70 * max(dot(n, lightDir), 0.0);\n"
    "  float spec = pow(max(dot(reflect(-lightDir, n), v), 0.0), 28.0) * 0.30;\n"
    "  vec3 o = c * l + spec;\n"
    "  if (col.a < 0.5) o = vec3(0.25, 1.0, 0.45) * (0.9 + 1.4 * pulse) + c * 0.2;\n"
    "  finalColor = sceneOut(o, 1.0);\n"
    "}\n";

void vrhands_init(void)
{
    sh = render_load_shader(vs, fs);
    ready = sh.id != 0;
}

// --- Emitting geometry -------------------------------------------------------------
// Local controller coordinates -> eye-relative world coordinates.

typedef struct {
    V3 o, x, y, z;      // grip origin and axes, world (eye-relative), metres
    double mirror;      // -1 for the left controller
} Frame;

static Frame F;
static Color cur;
static int cur_glow;

static V3 W(V3 l) { return v3_add(F.o, v3_add(v3_add(v3_scale(F.x, l.x * F.mirror), v3_scale(F.y, l.y)), v3_scale(F.z, l.z))); }
static V3 Wn(V3 n) { return v3_norm(v3_add(v3_add(v3_scale(F.x, n.x * F.mirror), v3_scale(F.y, n.y)), v3_scale(F.z, n.z))); }

static void vtx(V3 p, V3 n)
{
    V3 wp = W(p), wn = Wn(n);
    rlColor4ub(cur.r, cur.g, cur.b, cur_glow ? 0 : 255);
    rlNormal3f((float)wn.x, (float)wn.y, (float)wn.z);
    rlVertex3f((float)wp.x, (float)wp.y, (float)wp.z);
}

static void tri(V3 a, V3 b, V3 c, V3 na, V3 nb, V3 nc) { vtx(a, na); vtx(b, nb); vtx(c, nc); }

// Elliptical cylinder along `ax` (unit), centre c, length len, radii ru (along u) and rv (along v = ax x u), capped.
static void ecyl(V3 c, V3 ax, V3 u, double ru, double rv, double len, int seg)
{
    V3 v = v3_cross(ax, u);
    V3 a = v3_sub(c, v3_scale(ax, len / 2)), b = v3_add(c, v3_scale(ax, len / 2));
    for (int i = 0; i < seg; i++) {
        double t0 = 2 * PI_D * i / seg, t1 = 2 * PI_D * (i + 1) / seg;
        V3 d0 = v3_add(v3_scale(u, ru * cos(t0)), v3_scale(v, rv * sin(t0)));
        V3 d1 = v3_add(v3_scale(u, ru * cos(t1)), v3_scale(v, rv * sin(t1)));
        V3 n0 = v3_norm(v3_add(v3_scale(u, cos(t0) / ru), v3_scale(v, sin(t0) / rv)));
        V3 n1 = v3_norm(v3_add(v3_scale(u, cos(t1) / ru), v3_scale(v, sin(t1) / rv)));
        tri(v3_add(a, d0), v3_add(a, d1), v3_add(b, d1), n0, n1, n1);
        tri(v3_add(a, d0), v3_add(b, d1), v3_add(b, d0), n0, n1, n0);
        V3 nb = v3_scale(ax, -1);
        tri(a, v3_add(a, d1), v3_add(a, d0), nb, nb, nb);
        tri(b, v3_add(b, d0), v3_add(b, d1), ax, ax, ax);
    }
}

// Ellipsoid centre c, semi-axes along the unit vectors u, v, w.
static void ellipsoid(V3 c, V3 u, V3 v, V3 w, double ru, double rv, double rw, int seg, int rings)
{
    for (int j = 0; j < rings; j++) {
        double p0 = PI_D * j / rings - PI_D / 2, p1 = PI_D * (j + 1) / rings - PI_D / 2;
        for (int i = 0; i < seg; i++) {
            double t0 = 2 * PI_D * i / seg, t1 = 2 * PI_D * (i + 1) / seg;
            V3 q[4], nq[4];
            double pp[4] = { p0, p0, p1, p1 }, tt[4] = { t0, t1, t1, t0 };
            for (int k = 0; k < 4; k++) {
                double cx = cos(pp[k]) * cos(tt[k]), cy = cos(pp[k]) * sin(tt[k]), cz = sin(pp[k]);
                q[k] = v3_add(c, v3_add(v3_add(v3_scale(u, ru * cx), v3_scale(v, rv * cy)), v3_scale(w, rw * cz)));
                nq[k] = v3_norm(v3_add(v3_add(v3_scale(u, cx / ru), v3_scale(v, cy / rv)), v3_scale(w, cz / rw)));
            }
            tri(q[0], q[1], q[2], nq[0], nq[1], nq[2]);
            tri(q[0], q[2], q[3], nq[0], nq[2], nq[3]);
        }
    }
}

static void obox(V3 c, V3 ax, V3 ay, V3 az, double hx, double hy, double hz)
{
    V3 a[3] = { ax, ay, az };
    double h[3] = { hx, hy, hz };
    for (int f = 0; f < 3; f++)
        for (int s = -1; s <= 1; s += 2) {
            V3 n = v3_scale(a[f], s), u = a[(f + 1) % 3], v = a[(f + 2) % 3];
            double hu = h[(f + 1) % 3], hv = h[(f + 2) % 3];
            V3 fc = v3_add(c, v3_scale(n, h[f]));
            V3 p0 = v3_add(fc, v3_add(v3_scale(u, -hu), v3_scale(v, -hv))), p1 = v3_add(fc, v3_add(v3_scale(u, hu), v3_scale(v, -hv)));
            V3 p2 = v3_add(fc, v3_add(v3_scale(u, hu), v3_scale(v, hv))), p3 = v3_add(fc, v3_add(v3_scale(u, -hu), v3_scale(v, hv)));
            tri(p0, p1, p2, n, n, n);
            tri(p0, p2, p3, n, n, n);
        }
}

// --- The controller ------------------------------------------------------------------

static const Color BODY = { 214, 214, 210, 255 };   // Touch Plus: light grey shell
static const Color FACE = { 28, 29, 31, 255 };      // black face plate
static const Color BTN = { 60, 62, 66, 255 };
static const Color BTN_DOWN = { 120, 124, 130, 255 };
static const Color STICK = { 40, 41, 44, 255 };

static void set(Color c, int glow) { cur = c; cur_glow = glow; }

// side: 0 left, 1 right. glow: VB_* of the button to light up.
static void controller(int side, const VrInput *in, int glow)
{
    // Handle: a slightly flattened capsule, its top leaning forward.
    V3 hb = v3(0, -0.058, 0.026), ht = v3(0, 0.020, -0.006);
    V3 hax = v3_norm(v3_sub(ht, hb));
    V3 hc = v3_scale(v3_add(hb, ht), 0.5);
    double hl = v3_len(v3_sub(ht, hb));
    set(BODY, 0);
    ecyl(hc, hax, v3(1, 0, 0), 0.0165, 0.0195, hl, 18);
    V3 hy = v3_norm(v3_cross(hax, v3(1, 0, 0)));
    ellipsoid(hb, v3(1, 0, 0), hy, hax, 0.0165, 0.0195, 0.014, 14, 6);

    // Head: a wide rounded body under the face plate, tilted back toward the user.
    V3 fn = v3_norm(v3(0, 0.86, 0.50));                // face normal
    V3 fu = v3(1, 0, 0), fv = v3_cross(fn, fu);        // face plane: fu right, fv toward the user (down the face)
    V3 fc = v3(0, 0.030, -0.024);                      // face centre
    set(BODY, 0);
    ellipsoid(v3_sub(fc, v3_scale(fn, 0.010)), fu, fv, fn, 0.031, 0.038, 0.014, 22, 8);
    set(FACE, 0);
    ecyl(v3_add(fc, v3_scale(fn, 0.0035)), fn, fu, 0.027, 0.033, 0.002, 28);

    // Thumbstick (tilts with the stick), toward the inside of the face.
    double sx = side ? in->rx : in->lx, sy = side ? in->ry : in->ly;
    V3 sp = v3_add(fc, v3_add(v3_scale(fu, -0.006), v3_scale(fv, 0.004)));
    V3 sax = v3_norm(v3_add(fn, v3_add(v3_scale(fu, 0.35 * sx * (side ? 1 : -1)), v3_scale(fv, -0.35 * sy))));
    int sglow = (side && glow == VB_RSTICK) || (!side && (glow == VB_LSTICK || glow == VB_LSTICK_UP || glow == VB_LSTICK_DOWN));
    set(STICK, 0);
    ecyl(v3_add(sp, v3_scale(sax, 0.008)), sax, fu, 0.0035, 0.0035, 0.010, 10);
    set(STICK, sglow);
    ecyl(v3_add(sp, v3_scale(sax, 0.0145)), sax, fu, 0.0095, 0.0095, 0.0035, 18);

    // Face buttons: A (B above it) on the right, X (Y above) on the left.
    int lower = side ? in->a : in->x, upper = side ? in->b : in->y;
    int glow_lower = side ? glow == VB_A : glow == VB_X, glow_upper = side ? glow == VB_B : glow == VB_Y;
    V3 pl = v3_add(fc, v3_add(v3_scale(fu, 0.012), v3_scale(fv, 0.008)));
    V3 pu = v3_add(fc, v3_add(v3_scale(fu, 0.010), v3_scale(fv, -0.009)));
    set(lower ? BTN_DOWN : BTN, glow_lower);
    ecyl(v3_add(pl, v3_scale(fn, lower ? 0.0045 : 0.006)), fn, fu, 0.0058, 0.0058, 0.0035, 16);
    set(upper ? BTN_DOWN : BTN, glow_upper);
    ecyl(v3_add(pu, v3_scale(fn, upper ? 0.0045 : 0.006)), fn, fu, 0.0058, 0.0058, 0.0035, 16);
    // Menu (left) / Meta (right): a small flush button near the user's end.
    set(BTN, 0);
    ecyl(v3_add(v3_add(fc, v3_add(v3_scale(fu, 0.004), v3_scale(fv, 0.022))), v3_scale(fn, 0.0045)), fn, fu, 0.0032, 0.0032, 0.002, 12);

    // Trigger: a paddle under the front of the head, swinging back as it is pulled.
    double tv = side ? in->rtrig : in->ltrig;
    int tglow = side ? glow == VB_RTRIG : glow == VB_LTRIG;
    V3 tp = v3(0, 0.010, -0.050);                       // pivot
    double ta = -0.45 * tv;                             // radians, about +x
    V3 tdir = v3_norm(v3(0, -cos(0.5 + ta), -sin(0.5 + ta) * 0.4 + 0.15));
    set(tglow ? BTN : (Color){ 40, 41, 44, 255 }, tglow);
    obox(v3_add(tp, v3_scale(tdir, 0.012)), v3(1, 0, 0), tdir, v3_norm(v3_cross(v3(1, 0, 0), tdir)), 0.0085, 0.013, 0.0035);

    // Grip button: on the front of the handle, under the middle finger.
    double gv = side ? in->rgrip : in->lgrip;
    int gglow = side ? glow == VB_RGRIP : glow == VB_LGRIP;
    V3 gn = v3_norm(v3_cross(v3(1, 0, 0), hax));       // the handle's front
    if (v3_dot(gn, v3(0, 0, -1)) < 0) gn = v3_scale(gn, -1);
    set((Color){ 46, 47, 50, 255 }, gglow);
    obox(v3_add(v3_add(hc, v3_scale(hax, 0.010)), v3_scale(gn, 0.0185 - 0.003 * gv)), v3(1, 0, 0), hax, gn, 0.010, 0.020, 0.0035);
}

void vrhands_draw(const ViewCam *base, V3 eye_pos, const VrInput *in, double t)
{
    if (!ready) return;
    int glow = hints_vr_button();
    V3 r = v3_norm(v3_cross(base->fwd, base->upv)), u = base->upv, f = base->fwd;
    // tracking -> world (as main.c's vr_map), then relative to this eye.
#define MAP(v) v3_add(v3_add(v3_scale(r, (v).x), v3_scale(u, (v).y)), v3_scale(f, -(v).z))
    V3 light = v3_norm(MAP(v3(0.3, 0.8, 0.5)));
    float lf[3] = { (float)light.x, (float)light.y, (float)light.z };
    float pulse = (float)(0.5 + 0.5 * sin(t * 6.0));
    rlDrawRenderBatchActive();
    rlEnableDepthTest();
    rlEnableDepthMask();
    rlDisableBackfaceCulling();
    BeginShaderMode(sh);
    SetShaderValue(sh, GetShaderLocation(sh, "lightDir"), lf, SHADER_UNIFORM_VEC3);
    SetShaderValue(sh, GetShaderLocation(sh, "pulse"), &pulse, SHADER_UNIFORM_FLOAT);
    rlBegin(RL_TRIANGLES);
    for (int h = 0; h < 2; h++) {
        if (!in->grip_ok[h] || !in->controller[h]) continue;
        F.o = v3_sub(v3_add(base->pos, MAP(in->grip_pos[h])), eye_pos);
        F.x = MAP(in->grip_x[h]);
        F.y = MAP(in->grip_y[h]);
        F.z = MAP(in->grip_z[h]);
        F.mirror = h ? 1 : -1;
        controller(h, in, glow);
    }
    rlEnd();
    rlDrawRenderBatchActive();
    EndShaderMode();
    rlEnableBackfaceCulling();
#undef MAP
}

#endif
