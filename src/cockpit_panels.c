// LM cockpit panel artwork and switch logic.
//
// Every panel is drawn in millimetres onto the panel canvas (pcanvas.h):
// each shape and glyph carries its paint colour and what it emits (lamps, EL
// numerics, the integral panel lighting, back-lit instrument faces).
// Switches register themselves for the 3D levers and for picking while they
// are drawn.
//
// The layout and nomenclature follow the Apollo 11 LM (LM-5) as closely as
// a hand-drawn panel set can; switches that the simulation does not model
// still move and say so in their tooltip.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sound.h"
#include "raylib.h"
#include "rlgl.h"

#include "agc_host.h"
#include "cockpit_int.h"
#include "crew.h"
#include "dsky.h"
#include "hints.h"
#include "hud.h"
#include "lm.h"
#include "pcanvas.h"

PanelItems cp_items[NPANELS];
int cp_dirty = 1;
unsigned cp_live_panels;

static int cur_panel;

#define FT 0.3048

// --- Colours -------------------------------------------------------------------

static const Color C_PANEL = { 104, 112, 126, 255 };    // panel blue-grey (as the flight panels photograph)
static const Color C_WHITE = { 236, 236, 230, 255 };    // nomenclature
static const Color C_FACE = { 16, 17, 19, 255 };        // instrument faces
static const Color C_BEZEL = { 34, 35, 38, 255 };
static const Color C_NUT = { 150, 152, 156, 255 };
static const Color C_LAMP_OFF = { 52, 50, 46, 255 };

// --- Switch state store, keyed by string-literal identity ------------------------

typedef struct { const char *id; int v; } Slot;
static Slot slots[512];

static int *state_of(const char *id, int def)
{
    unsigned h = (unsigned)(((size_t)id >> 3) * 2654435761u) & 511;
    for (int i = 0; i < 512; i++) {
        Slot *s = &slots[(h + i) & 511];
        if (s->id == id) return &s->v;
        if (!s->id) { s->id = id; s->v = def; return &s->v; }
    }
    static int dummy;
    return &dummy;
}
#define ST(id, def) (*state_of(id, def))

// Functional switch ids.
static const char ID_ENGARM[] = "ENG ARM";
static const char ID_PGNS[] = "MODE CONTROL PGNS";
static const char ID_ROD[] = "RATE OF DESCENT";
static const char ID_STOP5[] = "ENGINE STOP (CDR)";
static const char ID_STOP6[] = "ENGINE STOP (LMP)";
static const char ID_START5[] = "ENGINE START (CDR)";
static const char ID_PLUSX5[] = "PLUS X TRANSL (CDR)";
static const char ID_PLUSX6[] = "PLUS X TRANSL (LMP)";
static const char ID_MA1[] = "MASTER ALARM (CDR)";
static const char ID_MA2[] = "MASTER ALARM (LMP)";
static const char ID_FLOOD[] = "FLOOD";
static const char ID_INTEG[] = "INTEGRAL";
static const char ID_NUM[] = "ANUN/NUM";
static const char ID_LTGTEST[] = "LTG TEST";
static const char ID_XSCALE[] = "X-POINTER SCALE";
static const char ID_RATESCALE[] = "RATE SCALE";
static const char ID_MODESEL[] = "MODE SEL";
static const char ID_ET[] = "EVENT TIMER";
static const char ID_ABORT[] = "ABORT";
static const char ID_ABSTAGE[] = "ABORT STAGE";
static const char ID_DEDA[] = "AGS DEDA";
static const char ID_FLOODSEL[] = "FLOOD SEL";
static const char ID_SIDEPNL[] = "SIDE PNLS";
static const char ID_ANUNBRT[] = "ANUN/NUM BRT";
static const char ID_DSKYLTG[] = "DSKY LTG";
static const char ID_MT[] = "MISSION TIMER";
static const char ID_MTH[] = "MT HOURS", ID_MTM[] = "MT MIN", ID_MTS[] = "MT SEC";
static const char ID_ETM[] = "ET MIN", ID_ETS[] = "ET SEC";
static const char ID_ETCOUNT[] = "ET COUNT";
static const char ID_ETRESET[] = "EVENT TIMER RESET";
static const char ID_PRPMON[] = "PRPLNT MON";
static const char ID_TEMPMON[] = "TEMP MON";
static const char ID_HTRMON[] = "HEATER MON";
static const char ID_PWRMON[] = "PWR MON";
static const char ID_SIGMON[] = "SIG STR";
static const char ID_SUITFAN[] = "SUIT FAN";
static const char ID_CABFAN[] = "CABIN FAN";

static float rod_anim, plusx_timer;
static double event_timer = 0;
static int event_run = 0;
static double mission_timer = -1;   // seconds shown on the mission timer (-1: not yet synchronised)
static int master_alarm, prev_alarm;
int cp_master_alarm(void) { return master_alarm; }
static char deda_addr[4] = "   ", deda_data[7] = "      ";
static int deda_n, deda_mode, deda_err;   // mode 0: address entry, 1: data entry

// Push buttons and DEDA keys stay down for a moment after a click.
static const char *press_id;
static int press_idx;
static float press_t;
static int pressed(const char *id, int idx) { return press_t > 0 && press_id == id && press_idx == idx; }

// Circuit breakers of panels 11 and 16, in drawing order: 1 = pulled open.
#define CB_PER_PANEL 240
static unsigned char cb_open[2][CB_PER_PANEL];
static const char *cb_label[2][CB_PER_PANEL];
static int cb_count[2], cb_side, cb_next;

// A named breaker is open on either panel (the same circuits repeat on both buses).
static int cb_out(const char *name)
{
    for (int k = 0; k < 2; k++)
        for (int i = 0; i < cb_count[k]; i++)
            if (cb_open[k][i] && cb_label[k][i] && !strcmp(cb_label[k][i], name)) return 1;
    return 0;
}

static int side_panel(int p) { return p == P8 || p == P11 || p == P12 || p == P14 || p == P16; }

static int lamp_test(void) { return ST(ID_LTGTEST, -1) > 0; }
float cp_flood_level(void)
{
    if (cb_out("FLOOD")) return 0;
    return ST(ID_FLOOD, 5) / 10.0f * (ST(ID_FLOODSEL, 1) > 0 ? 1.0f : 0.55f);
}
static int drawing_panel;   // inside cp_draw_panel: lighting follows the panel being drawn
float cp_integral_level(void)
{
    if (cb_out("INTGL")) return 0;
    if (drawing_panel && side_panel(cur_panel) && (ST(ID_SIDEPNL, 1) < 0 || cb_out("SIDE PNLS"))) return 0;
    return ST(ID_INTEG, 6) / 10.0f;
}
float cp_numerics_level(void) { return (0.35f + ST(ID_NUM, 8) / 10.0f * 0.65f) * (ST(ID_ANUNBRT, 1) > 0 ? 1.0f : 0.45f); }
float cp_fan_level(void)
{
    int suit = !cb_out(ST(ID_SUITFAN, 1) > 0 ? "SUIT FAN 1" : "SUIT FAN 2");
    int cabin = ST(ID_CABFAN, -1) > 0 && !cb_out("CABIN FAN CONT");
    return suit * 1.0f + cabin * 0.6f;
}

// --- Hint marker: a pulsing green ring and a short instruction by the control
// the player should operate next. Drawn last on its panel, over the artwork.

static HintTarget hint_tgt;
static struct { int on, dir; float x, y, r, lift; const char *text; } mark;
static struct { int on; float x, y; const char *text; } hint_rec[NPANELS];

static void hint_at(float x, float y, float r, float lift, int dir)
{
    mark.on = 1; mark.x = x; mark.y = y; mark.r = r; mark.lift = lift; mark.dir = dir; mark.text = hint_tgt.text;
}

static void draw_mark(void)
{
    if (!mark.on) return;
    mark.on = 0;
    float p = 0.5f + 0.5f * sinf((float)GetTime() * 6.0f);
    Color g = { 90, 255, 150, (unsigned char)(150 + 105 * p) };
    float x = mark.x, y = mark.y, r = mark.r + 1.5f * p;
    pc_lift(mark.lift + 0.4f);
    pc_glow((Color){ 60, 230, 120, 255 });
    pc_ring((Vector2){ x, y }, r, 1.8f, g);
    if (mark.dir) {
        // Which way to move it: a chevron bobbing in that direction, left of the ring.
        float d = (float)-mark.dir, ax = x - r - 5.5f, ay = y + d * (1.5f + 2.5f * p);
        pc_tri((Vector2){ ax - 3.2f, ay - d * 2.5f }, (Vector2){ ax + 3.2f, ay - d * 2.5f }, (Vector2){ ax, ay + d * 3.0f }, g);
    }
    pc_glow_off();
    // The instruction: beside a switch (its own legends sit above and below
    // it), above a button; kept on the panel.
    float sz = 8.0f, pw = cp_panels[cur_panel].w_mm;
    Vector2 m = pc_measure(mark.text, sz, 0.2f);
    float tx, ty;
    if (mark.dir) {
        tx = x + r + 5.0f;
        ty = y - sz / 2;
        if (tx + m.x + 4 > pw) tx = x - r - 15.0f - m.x;
    } else {
        tx = x - m.x / 2;
        ty = y - r - sz - 4.0f;
        if (ty < 3) ty = y + r + 4.0f;
        if (tx + m.x + 4 > pw) tx = pw - m.x - 4;
        if (tx < 4) tx = 4;
    }
    pc_lift(mark.lift + 7.0f);
    pc_rrect((Rectangle){ tx - 2.2f, ty - 1.4f, m.x + 4.4f, sz + 2.8f }, 0.5f, (Color){ 5, 24, 14, 240 });
    pc_glow((Color){ 70, 220, 130, 255 });
    pc_rrect_lines((Rectangle){ tx - 2.2f, ty - 1.4f, m.x + 4.4f, sz + 2.8f }, 0.5f, 0.35f, g);
    pc_text(mark.text, tx, ty, sz, 0.2f, (Color){ 150, 255, 190, 255 });
    pc_glow_off();
    pc_lift(0);
    hint_rec[cur_panel].on = 1;
    hint_rec[cur_panel].x = x;
    hint_rec[cur_panel].y = y;
    hint_rec[cur_panel].text = mark.text;
    cp_live_panels |= 1u << cur_panel;
}

int cp_hint_point(int *panel, float *x_mm, float *y_mm, const char **text)
{
    for (int i = 0; i < NPANELS; i++)
        if (hint_rec[i].on) {
            *panel = i; *x_mm = hint_rec[i].x; *y_mm = hint_rec[i].y; *text = hint_rec[i].text;
            return 1;
        }
    return 0;
}

// --- Drawing helpers (millimetres, on the panel canvas) ------------------------------
//
// The panel code below is written against a handful of raylib-style 2D calls;
// here they record canvas primitives instead of painting a texture.

static void pcx_rect(Rectangle r, Color c) { pc_rect(r.x, r.y, r.width, r.height, c); }
static void pcx_text(Font f, const char *s, Vector2 p, float size, float sp, Color c) { (void)f; pc_text(s, p.x, p.y, size, sp, c); }
static void pcx_rrect(Rectangle r, float roundness, int seg, Color c) { (void)seg; pc_rrect(r, roundness, c); }
static void pcx_rrect_lines(Rectangle r, float roundness, int seg, float t, Color c) { (void)seg; pc_rrect_lines(r, roundness, t, c); }
static void pcx_poly_lines(Vector2 c, int sides, float r, float rot, Color col) { pc_poly_lines(c, sides, r, rot, 0.3f, col); }
#define DrawRectangleRec pcx_rect
#define DrawLineEx pc_line
#define DrawTextEx pcx_text
#define DrawCircleV pc_circle
#define DrawRectangleRounded pcx_rrect
#define DrawRectangleRoundedLinesEx pcx_rrect_lines
#define DrawRectangleLinesEx pc_rect_lines
#define DrawTriangle pc_tri
#define DrawPoly pc_poly
#define DrawPolyLines pcx_poly_lines

// Lettering is drawn larger than the real 3 mm engraving so it reads from the
// seat in a headset; LEG scales nomenclature, the layout leaves room for it.
#define LEG 1.28f

static Color scale_col(Color c, float k)
{
    return (Color){ (unsigned char)fminf(255, c.r * k), (unsigned char)fminf(255, c.g * k), (unsigned char)fminf(255, c.b * k), 255 };
}

// Integral (edge) lighting of the engraved nomenclature: blue-green EL.
static Color el_col(float k)
{
    float g = cp_integral_level() * k;
    return (Color){ (unsigned char)(140 * g), (unsigned char)(235 * g), (unsigned char)(185 * g), 255 };
}

// Back-lighting of instrument faces (scales, pointers, tapes): fraction of the paint colour.
static float backlight(void) { return 0.10f + 0.62f * cp_integral_level(); }

// Nomenclature. (x, y) anchors the top of the text at its nominal size; the
// enlarged text grows about its own centre line so it stays where it was.
static void txt(const char *s, float x, float y, float size, int align)
{
    float sz = size * LEG;
    Vector2 m = pc_measure(s, sz, sz * 0.05f);
    float px = align == 0 ? x : align == 1 ? x - m.x / 2 : x - m.x;
    pc_glow(el_col(0.60f));
    pc_text(s, px, y - (sz - size) / 2, sz, sz * 0.05f, C_WHITE);
    pc_glow_off();
}
#define TXT_C(s, x, y, sz) txt(s, x, y, sz, 1)

static void rect(float x, float y, float w, float h, Color c) { pc_rect(x, y, w, h, c); }

// White group outline with the title breaking the top line.
static void group(float x, float y, float w, float h, const char *title)
{
    float t = 0.6f;
    float tw = title ? pc_measure(title, 3.4f * LEG, 0.2f).x + 4 : 0;
    float cx = x + w / 2;
    pc_glow(el_col(0.35f));
    pc_rect(x, y, cx - tw / 2 - x, t, C_WHITE);
    pc_rect(cx + tw / 2, y, x + w - cx - tw / 2, t, C_WHITE);
    pc_rect(x, y + h - t, w, t, C_WHITE);
    pc_rect(x, y, t, h, C_WHITE);
    pc_rect(x + w - t, y, t, h, C_WHITE);
    pc_glow_off();
    if (title) TXT_C(title, cx, y - 1.9f, 3.4f);
}

static void add_hot(int kind, Rectangle r, const char *id, int idx)
{
    PanelItems *it = &cp_items[cur_panel];
    if (it->nhot >= CP_MAXHOT) return;
    it->hot[it->nhot++] = (Hot){ cur_panel, kind, r, id, idx };
}

static void add_part(int kind, float x, float y, float val, float size, float w, float h, int guard, Color c)
{
    PanelItems *it = &cp_items[cur_panel];
    if (it->nlev >= CP_MAXLEV) return;
    Lever *l = &it->lev[it->nlev++];
    memset(l, 0, sizeof *l);
    l->panel = cur_panel; l->kind = kind; l->x = x; l->y = y; l->val = val; l->size = size;
    l->w = w; l->h = h; l->guard = guard; l->r = c.r; l->g = c.g; l->b = c.b;
}

static void add_lever(int kind, float x, float y, float val, float size, int guard, Color c)
{
    add_part(kind, x, y, val, size, 0, 0, guard, c);
}

// Height of the modelled caps above the panel, mm.
#define BTN_UP 4.5f
#define BTN_DOWN 1.6f
#define KEY_UP 3.2f
#define KEY_DOWN 0.9f

// Yellow-and-black caution border, as round the ABORT pair and the engine
// START / STOP buttons.
static void hazard_frame(float x, float y, float w, float h, float t)
{
    Color yel = { 216, 178, 30, 255 }, blk = { 16, 16, 16, 255 };
    pc_rect_lines((Rectangle){ x, y, w, h }, t, yel);
    float step = t * 1.5f;
    for (float s = step * 0.5f; s < w - step * 0.5f; s += 2 * step) {
        float len = fminf(step, w - s);
        pc_rect(x + s, y, len, t, blk);
        pc_rect(x + s, y + h - t, len, t, blk);
    }
    for (float s = step * 0.5f; s < h - step * 0.5f; s += 2 * step) {
        float len = fminf(step, h - s);
        pc_rect(x, y + s, t, len, blk);
        pc_rect(x + w - t, y + s, t, len, blk);
    }
}

// Toggle switch. up/mid/dn labels (mid NULL = two positions). State: +1 up,
// 0 centre, -1 down.
static int toggle(float x, float y, const char *id, const char *title, const char *up, const char *mid, const char *dn,
                  int def, int guard)
{
    int s = ST(id, def);
    if (title) TXT_C(title, x, y - 15.5f, 3.2f);
    if (up) TXT_C(up, x, y - 10.5f, 2.8f);
    if (dn) TXT_C(dn, x, y + 7.8f, 2.8f);
    if (mid) txt(mid, x + 5.5f, y - 1.4f, 2.6f, 0);
    DrawPoly((Vector2){ x, y }, 6, 4.2f, 0, C_NUT);
    DrawPolyLines((Vector2){ x, y }, 6, 4.2f, 0, (Color){ 90, 92, 96, 255 });
    DrawCircleV((Vector2){ x, y }, 2.4f, (Color){ 60, 62, 66, 255 });
    if (guard) {
        DrawRectangleRec((Rectangle){ x - 6.5f, y - 7, 1.4f, 14 }, (Color){ 170, 170, 172, 255 });
        DrawRectangleRec((Rectangle){ x + 5.1f, y - 7, 1.4f, 14 }, (Color){ 170, 170, 172, 255 });
    }
    add_lever(LEV_TOGGLE, x, y, (float)s, 16, guard, (Color){ 196, 198, 202, 255 });
    add_hot(HOT_TOGGLE, (Rectangle){ x - 7, y - 11, 14, 22 }, id, mid ? 3 : 2);
    if (id == ID_ENGARM && hint_tgt.kind == HT_ENG_ARM) hint_at(x, y, 13.0f, 0, -1);
    else if (id == ID_PGNS && hint_tgt.kind == HT_PGNS_AUTO) hint_at(x, y, 13.0f, 0, +1);
    else if (id == ID_ROD && hint_tgt.kind == HT_ROD_UP) hint_at(x, y, 13.0f, 0, +1);
    else if (id == ID_ROD && hint_tgt.kind == HT_ROD_DOWN) hint_at(x, y, 13.0f, 0, -1);
    return s;
}

// Rotary selector knob: n labelled positions on an arc.
static int rotary(float x, float y, const char *id, const char *title, const char *const *labels, int n, int def,
                  float arc_deg)
{
    int s = ST(id, def);
    if (s >= n) s = n - 1;
    if (s < 0) s = 0;
    if (title) TXT_C(title, x, y - 19.0f, 3.2f);
    float a0 = -arc_deg / 2;
    for (int i = 0; i < n; i++) {
        float a = (a0 + (n > 1 ? arc_deg * i / (n - 1) : 0)) * DEG2RAD;
        float sx = sinf(a), sy = -cosf(a);
        pc_glow(el_col(0.45f));
        DrawLineEx((Vector2){ x + sx * 8.5f, y + sy * 8.5f }, (Vector2){ x + sx * 10.5f, y + sy * 10.5f }, 0.6f, C_WHITE);
        pc_glow_off();
        if (labels && labels[i]) {
            int al = sx > 0.3f ? 0 : sx < -0.3f ? 2 : 1;
            txt(labels[i], x + sx * 12.0f, y + sy * 12.0f - 1.4f, 2.6f, al);
        }
    }
    float a = (a0 + (n > 1 ? arc_deg * s / (n - 1) : 0)) * DEG2RAD;
    DrawCircleV((Vector2){ x, y }, 7.5f, (Color){ 30, 30, 32, 255 });
    add_lever(LEV_KNOB, x, y, a, 6.5f, 0, (Color){ 22, 22, 24, 255 });
    add_hot(HOT_KNOB, (Rectangle){ x - 9, y - 9, 18, 18 }, id, n);
    return s;
}

// Thumbwheel (audio volumes etc.), 0..1.
static void thumbwheel(float x, float y, const char *id, const char *title)
{
    int s = ST(id, 6);
    if (title) TXT_C(title, x, y - 12, 2.8f);
    rect(x - 3.5f, y - 7, 7, 14, (Color){ 20, 20, 22, 255 });
    for (int i = 0; i < 6; i++)
        DrawRectangleRec((Rectangle){ x - 2.8f, y - 6 + i * 2.3f + (s % 2) * 1.1f, 5.6f, 0.6f }, (Color){ 70, 70, 74, 255 });
    TXT_C("MAX", x + 8, y - 7, 2.2f);
    add_lever(LEV_THUMB, x, y, (float)s, 9, 0, (Color){ 46, 46, 50, 255 });
    add_hot(HOT_KNOB, (Rectangle){ x - 4, y - 8, 8, 16 }, id, 10);
}

// Annunciator lamp (part of a caution/warning matrix or stand-alone). The
// legend is always legible; lit, the whole lens glows behind a dark legend.
static void lamp(float x, float y, float w, float h, const char *label, Color on_col, int on)
{
    on = on || lamp_test();
    float sz = fminf(2.9f * LEG, h * 0.5f);
    Vector2 m = pc_measure(label, sz, 0.15f);
    if (m.x > w - 2) { sz *= (w - 2) / m.x; m = pc_measure(label, sz, 0.15f); }
    if (on) {
        pc_glow(scale_col(on_col, cp_numerics_level()));
        DrawRectangleRec((Rectangle){ x, y, w, h }, scale_col(on_col, 0.8f));
        pc_glow((Color){ 40, 25, 10, 255 });
        pc_text(label, x + (w - m.x) / 2, y + (h - sz) / 2, sz, 0.15f, (Color){ 30, 20, 10, 255 });
        pc_glow_off();
    } else {
        DrawRectangleRec((Rectangle){ x, y, w, h }, C_LAMP_OFF);
        pc_text(label, x + (w - m.x) / 2, y + (h - sz) / 2, sz, 0.15f, (Color){ 150, 146, 134, 255 });
    }
    DrawRectangleLinesEx((Rectangle){ x, y, w, h }, 0.5f, (Color){ 20, 20, 20, 255 });
}

// Push button (optionally lighted).
static void pushbtn(float x, float y, float w, float h, const char *id, const char *label, Color lit_col, int lit, int guard)
{
    lit = lit || lamp_test();
    DrawRectangleRounded((Rectangle){ x - 1.2f, y - 1.2f, w + 2.4f, h + 2.4f }, 0.2f, 4, C_BEZEL);
    Color face = lit ? lit_col : (Color){ 58, 56, 54, 255 };
    if (!lit && lit_col.r > 200 && lit_col.g < 120) face = (Color){ 96, 40, 36, 255 };
    // The cap is modelled in the cabin; its face and legend ride on top of it.
    float up = pressed(id, 0) ? BTN_DOWN : BTN_UP;
    add_part(LEV_BUTTON, x + w / 2, y + h / 2, up, 0, w, h, guard, scale_col(face, 0.45f));
    pc_lift(up + 0.12f);
    if (lit) pc_glow(scale_col(lit_col, cp_numerics_level()));
    DrawRectangleRounded((Rectangle){ x, y, w, h }, 0.15f, 4, face);
    pc_glow_off();
    float sz = 3.4f * LEG;
    Vector2 m = pc_measure(label, sz, 0.15f);
    if (m.x > w - 3) sz *= (w - 3) / m.x;
    if (lit) pc_glow((Color){ 255, 250, 240, 255 });
    else pc_glow(el_col(0.35f));
    pc_text_box(label, (Rectangle){ x, y, w, h }, sz, 0.15f, lit ? (Color){ 255, 250, 240, 255 } : (Color){ 235, 232, 225, 255 });
    pc_glow_off();
    pc_lift(0);
    add_hot(HOT_BUTTON, (Rectangle){ x, y, w, h }, id, 0);
    if (((id == ID_STOP5 || id == ID_STOP6) && hint_tgt.kind == HT_ENGINE_STOP) ||
        ((id == ID_MA1 || id == ID_MA2) && hint_tgt.kind == HT_MASTER_ALARM))
        hint_at(x + w / 2, y + h / 2, fmaxf(w, h) * 0.5f + 5.0f, up, 0);
}

// Circuit breaker with its label above.
static void breaker(float x, float y, const char *label)
{
    int i = cb_next < CB_PER_PANEL ? cb_next++ : CB_PER_PANEL - 1;
    cb_label[cb_side][i] = label;
    if (cb_count[cb_side] <= i) cb_count[cb_side] = i + 1;
    int popped = cb_open[cb_side][i];
    if (label) TXT_C(label, x, y - 8.4f, 2.7f);
    DrawCircleV((Vector2){ x, y }, 4.6f, (Color){ 14, 14, 15, 255 });
    add_part(LEV_CB, x, y, (float)popped, 2.7f, 0, 0, 0, (Color){ 24, 24, 26, 255 });
    add_hot(HOT_CB, (Rectangle){ x - 4.6f, y - 4.6f, 9.2f, 9.2f }, label, i);
}

// Talkback flag indicator: 0 = barber pole, 1 = grey.
static void talkback(float x, float y, int grey)
{
    DrawCircleV((Vector2){ x, y }, 5.2f, C_BEZEL);
    DrawCircleV((Vector2){ x, y }, 4.2f, (Color){ 10, 10, 10, 255 });
    pc_glow_paint(backlight() * 0.5f);
    if (grey) {
        DrawRectangleRec((Rectangle){ x - 3.4f, y - 2.4f, 6.8f, 4.8f }, (Color){ 150, 150, 148, 255 });
        pc_glow_off();
    } else {
        for (int i = -2; i <= 2; i++) {
            Color c = (i & 1) ? (Color){ 20, 20, 20, 255 } : (Color){ 225, 225, 220, 255 };
            DrawLineEx((Vector2){ x - 2.4f + i * 1.2f, y + 2.4f }, (Vector2){ x + 2.4f + i * 1.2f, y - 2.4f }, 1.2f, c);
        }
        pc_glow_off();
        DrawRectangleLinesEx((Rectangle){ x - 5.0f, y - 3.6f, 10.0f, 7.2f }, 1.4f, (Color){ 10, 10, 10, 255 });
    }
}

// Seven-segment EL numerics (DSKY style). str: digits, ' ' blank, '+', '-', ':'.
static int el_power = 1;   // 0: the display's breaker is open
static void el_segment(Rectangle r, int on)
{
    if (on && el_power) {
        pc_glow(scale_col((Color){ 110, 255, 140, 255 }, cp_numerics_level()));
        pc_rrect(r, 0.9f, (Color){ 150, 230, 160, 255 });
        pc_glow_off();
    } else {
        pc_rrect(r, 0.9f, (Color){ 30, 40, 32, 255 });
    }
}

static void el_digits(float x, float y, float h, const char *str)
{
    float w = h * 0.58f, t = w * 0.19f, gap = h * 0.22f;
    static const unsigned char seg[10] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };
    rect(x - 1.5f, y - 1.5f, (float)strlen(str) * (w + gap) - gap + 3, h + 3, (Color){ 12, 16, 13, 255 });
    for (const char *p = str; *p; p++, x += w + gap) {
        if (*p == ':') {
            el_segment((Rectangle){ x + w / 2 - t / 2, y + h * 0.25f, t, t }, 1);
            el_segment((Rectangle){ x + w / 2 - t / 2, y + h * 0.7f, t, t }, 1);
            continue;
        }
        unsigned char s = 0;
        if (*p >= '0' && *p <= '9') s = seg[*p - '0'];
        if (lamp_test() && *p != ' ') s = 0x7F;
        float hh = h / 2, g = t * 0.12f;
        Rectangle r[7] = {
            { x + t * 0.5f + g, y, w - t - 2 * g, t }, { x + w - t, y + t * 0.5f + g, t, hh - t * 0.5f - 2 * g },
            { x + w - t, y + hh + g, t, hh - t * 0.5f - 2 * g }, { x + t * 0.5f + g, y + h - t, w - t - 2 * g, t },
            { x, y + hh + g, t, hh - t * 0.5f - 2 * g }, { x, y + t * 0.5f + g, t, hh - t * 0.5f - 2 * g },
            { x + t * 0.5f + g, y + hh - t / 2, w - t - 2 * g, t },
        };
        if (*p == '+' || *p == '-') {
            el_segment((Rectangle){ x, y + hh - t / 2, w, t }, 1);
            if (*p == '+') el_segment((Rectangle){ x + w / 2 - t / 2, y + h * 0.2f, t, h * 0.6f }, 1);
            continue;
        }
        for (int i = 0; i < 7; i++) el_segment(r[i], (s >> i) & 1);
    }
}

// Vertical edgewise meter: value 0..1 against a scale.
static void meter(float x, float y, float w, float h, const char *title, float v, const char *top, const char *bot)
{
    if (title) TXT_C(title, x + w / 2, y - 5.5f, 2.8f);
    rect(x - 1, y - 1, w + 2, h + 2, C_BEZEL);
    rect(x, y, w, h, C_FACE);
    pc_glow_paint(backlight());
    for (int i = 0; i <= 10; i++) {
        float yy = y + 2 + (h - 4) * i / 10.0f;
        DrawLineEx((Vector2){ x + 1, yy }, (Vector2){ x + (i % 5 ? 3.5f : 5.5f), yy }, 0.45f, C_WHITE);
    }
    if (top) DrawTextEx(font_bold, top, (Vector2){ x + 6.0f, y + 0.4f }, 2.9f, 0.1f, C_WHITE);
    if (bot) DrawTextEx(font_bold, bot, (Vector2){ x + 6.0f, y + h - 3.4f }, 2.9f, 0.1f, C_WHITE);
    v = v < 0 ? 0 : v > 1 ? 1 : v;
    float yy = y + 2 + (h - 4) * (1 - v);
    DrawTriangle((Vector2){ x + w - 0.5f, yy - 2 }, (Vector2){ x + 2, yy }, (Vector2){ x + w - 0.5f, yy + 2 },
                 (Color){ 240, 240, 236, 255 });
    pc_glow_off();
}

// Round dial with a needle: v 0..1 over 270 degrees.
static void dial(float x, float y, float r, const char *title, float v, const char *lo, const char *hi)
{
    if (title) TXT_C(title, x, y - r - 6.5f, 2.8f);
    DrawCircleV((Vector2){ x, y }, r + 1.5f, C_BEZEL);
    DrawCircleV((Vector2){ x, y }, r, C_FACE);
    pc_glow_paint(backlight());
    for (int i = 0; i <= 10; i++) {
        float a = (-135 + 27 * i) * DEG2RAD;
        float l = i % 5 ? 0.82f : 0.72f;
        DrawLineEx((Vector2){ x + sinf(a) * r * l, y - cosf(a) * r * l }, (Vector2){ x + sinf(a) * r * 0.95f, y - cosf(a) * r * 0.95f },
                   0.45f, C_WHITE);
    }
    if (lo) DrawTextEx(font_bold, lo, (Vector2){ x - r * 0.78f, y + r * 0.42f }, 2.9f, 0.1f, C_WHITE);
    if (hi) DrawTextEx(font_bold, hi, (Vector2){ x + r * 0.22f, y + r * 0.42f }, 2.9f, 0.1f, C_WHITE);
    v = v < 0 ? 0 : v > 1 ? 1 : v;
    float a = (-135 + 270 * v) * DEG2RAD;
    DrawLineEx((Vector2){ x, y }, (Vector2){ x + sinf(a) * r * 0.86f, y - cosf(a) * r * 0.86f }, 0.8f, (Color){ 245, 245, 240, 255 });
    pc_glow_off();
    DrawCircleV((Vector2){ x, y }, 1.4f, (Color){ 60, 60, 60, 255 });
}

static float clampf(float v, float a, float b) { return v < a ? a : v > b ? b : v; }

// --- Instruments ----------------------------------------------------------------

static void fdai(float cx, float cy, float r)
{
    DrawRectangleRounded((Rectangle){ cx - r - 18, cy - r - 18, 2 * r + 36, 2 * r + 36 }, 0.12f, 6, C_BEZEL);
    DrawCircleV((Vector2){ cx, cy }, r + 3, (Color){ 6, 6, 7, 255 });
    float s = r * 2.0f * 1.22f;
    pc_glow_paint(backlight() * 0.75f);
    pc_image((Rectangle){ cx - s / 2, cy - s / 2, s, s });
    pc_glow_paint(backlight());
    // Roll scale around the ball.
    for (int i = -6; i <= 6; i++) {
        float a = i * 15 * DEG2RAD;
        float l = i % 2 ? 4 : 7;
        DrawLineEx((Vector2){ cx + sinf(a) * (r + 4), cy - cosf(a) * (r + 4) },
                   (Vector2){ cx + sinf(a) * (r + 4 + l), cy - cosf(a) * (r + 4 + l) }, 0.7f, C_WHITE);
    }
    Color amb = { 255, 170, 40, 255 };
    // Fixed LM symbol.
    DrawLineEx((Vector2){ cx - r * 0.55f, cy }, (Vector2){ cx - r * 0.16f, cy }, 1.6f, amb);
    DrawLineEx((Vector2){ cx + r * 0.16f, cy }, (Vector2){ cx + r * 0.55f, cy }, 1.6f, amb);
    DrawCircleV((Vector2){ cx, cy }, 1.4f, amb);
    // Error needles (from the AGC through the CDU error counters).
    float ey = clampf((float)(lm.needle[1] / 384.0), -1, 1), ez = clampf((float)(lm.needle[2] / 384.0), -1, 1);
    float ex = clampf((float)(lm.needle[0] / 384.0), -1, 1);
    DrawLineEx((Vector2){ cx + ey * r * 0.8f, cy - r * 0.78f }, (Vector2){ cx + ey * r * 0.8f, cy - r * 0.25f }, 1.3f, amb);
    DrawLineEx((Vector2){ cx + r * 0.78f, cy + ez * r * 0.8f }, (Vector2){ cx + r * 0.25f, cy + ez * r * 0.8f }, 1.3f, amb);
    DrawLineEx((Vector2){ cx + ex * r * 0.8f - 5, cy + r * 0.92f }, (Vector2){ cx + ex * r * 0.8f + 5, cy + r * 0.92f }, 1.3f, amb);
    pc_glow_off();
    // Rate scales: top roll, right pitch, bottom yaw.
    float full = ST(ID_RATESCALE, -1) > 0 ? 25.0f : 5.0f;
    float rr = clampf((float)(lm.w.z * 57.2958 / full), -1, 1), rp = clampf((float)(lm.w.y * 57.2958 / full), -1, 1),
          ry = clampf((float)(lm.w.x * 57.2958 / full), -1, 1);
    Rectangle top = { cx - r * 0.8f, cy - r - 15, r * 1.6f, 6 }, right = { cx + r + 9, cy - r * 0.8f, 6, r * 1.6f },
              bot = { cx - r * 0.8f, cy + r + 9, r * 1.6f, 6 };
    DrawRectangleRec(top, C_FACE); DrawRectangleRec(right, C_FACE); DrawRectangleRec(bot, C_FACE);
    pc_glow_paint(backlight());
    for (int i = 0; i <= 4; i++) {
        DrawLineEx((Vector2){ top.x + top.width * i / 4, top.y }, (Vector2){ top.x + top.width * i / 4, top.y + 2 }, 0.45f, C_WHITE);
        DrawLineEx((Vector2){ bot.x + bot.width * i / 4, bot.y + 6 }, (Vector2){ bot.x + bot.width * i / 4, bot.y + 4 }, 0.45f, C_WHITE);
        DrawLineEx((Vector2){ right.x + 6, right.y + right.height * i / 4 }, (Vector2){ right.x + 4, right.y + right.height * i / 4 }, 0.45f, C_WHITE);
    }
    Color nd = { 240, 240, 236, 255 };
    DrawTriangle((Vector2){ cx + rr * r * 0.8f, top.y + 5.5f }, (Vector2){ cx + rr * r * 0.8f + 2, top.y + 1 }, (Vector2){ cx + rr * r * 0.8f - 2, top.y + 1 }, nd);
    DrawTriangle((Vector2){ right.x + 0.5f, cy - rp * r * 0.8f }, (Vector2){ right.x + 5, cy - rp * r * 0.8f + 2 }, (Vector2){ right.x + 5, cy - rp * r * 0.8f - 2 }, nd);
    DrawTriangle((Vector2){ cx + ry * r * 0.8f, bot.y + 0.5f }, (Vector2){ cx + ry * r * 0.8f - 2, bot.y + 5 }, (Vector2){ cx + ry * r * 0.8f + 2, bot.y + 5 }, nd);
    // Flags: OFF flag when the attitude reference is lost (NO ATT).
    if (agc_host_dsky()->no_att) {
        DrawRectangleRec((Rectangle){ cx - r * 0.95f, cy - r * 0.35f, 13, 9 }, (Color){ 220, 60, 40, 255 });
        DrawTextEx(font_bold, "OFF", (Vector2){ cx - r * 0.95f + 1.5f, cy - r * 0.35f + 2.2f }, 4.6f, 0.1f, WHITE);
    }
    pc_glow_off();
    TXT_C("ROLL", cx, cy - r - 22, 2.6f);
    txt("PITCH", cx + r + 17, cy - 1.2f, 2.6f, 0);
    TXT_C("YAW", cx, cy + r + 17, 2.6f);
}

static void x_pointer(float x, float y, float s, int lmp)
{
    TXT_C("X-POINTER", x + s / 2, y - 6, 3.0f);
    rect(x - 2, y - 2, s + 4, s + 4, C_BEZEL);
    rect(x, y, s, s, C_FACE);
    float cx = x + s / 2, cy = y + s / 2;
    pc_glow_paint(backlight());
    for (int i = -4; i <= 4; i++) {
        float l = i % 2 ? 1.5f : 3;
        DrawLineEx((Vector2){ cx + i * s / 10, cy - l }, (Vector2){ cx + i * s / 10, cy + l }, 0.45f, C_WHITE);
        DrawLineEx((Vector2){ cx - l, cy + i * s / 10 }, (Vector2){ cx + l, cy + i * s / 10 }, 0.45f, C_WHITE);
    }
    DrawTextEx(font_bold, "FWD", (Vector2){ cx + 2, y + 2 }, 3.4f, 0.1f, C_WHITE);
    DrawTextEx(font_bold, "LAT", (Vector2){ x + s - 10, cy + 2 }, 3.4f, 0.1f, C_WHITE);
    V3 vb = lm_inertial_to_body(lm_surface_velocity());
    double full = ST(ID_XSCALE, 1) > 0 ? 20.0 : 200.0;   // ft/s full scale, LO MULT / HI MULT
    float fx = clampf((float)(vb.y / FT / full), -1, 1), fz = clampf((float)(vb.z / FT / full), -1, 1);
    Color nc = { 245, 245, 240, 255 };
    DrawLineEx((Vector2){ cx + fx * s * 0.42f, y + 3 }, (Vector2){ cx + fx * s * 0.42f, y + s - 3 }, 1.1f, nc);
    DrawLineEx((Vector2){ x + 3, cy - fz * s * 0.42f }, (Vector2){ x + s - 3, cy - fz * s * 0.42f }, 1.1f, nc);
    pc_glow_off();
    (void)lmp;
}

// Altitude / altitude-rate tape meter. The tapes are back-lit; numerals are
// 6.5 mm so both read at a glance from the seat.
static void alt_tapes(float x, float y, float w, float h)
{
    TXT_C("ALT", x + w * 0.26f, y - 10, 3.0f);
    TXT_C("ALT RT", x + w * 0.76f, y - 10, 3.0f);
    TXT_C("FT", x + w * 0.26f, y - 5.5f, 2.4f);
    TXT_C("FT/SEC", x + w * 0.76f, y - 5.5f, 2.4f);
    rect(x - 2, y - 2, w + 4, h + 4, C_BEZEL);
    rect(x, y, w, h, C_FACE);
    // Source: landing radar or PGNS (MODE SEL), as on the real panel.
    int src = ST(ID_MODESEL, 1);   // +1 LDG RDR, 0 PGNS, -1 AGS
    double alt = lm.altitude / FT, rate = lm.hdot / FT;
    int nodata = 0;
    if (src > 0) {
        if (lm.lr_range_good) alt = lm.lr_alt / FT;
        else nodata = 1;
    }
    const float NUM = 6.5f;
    float tw = w * 0.5f - 3;
    Rectangle ta = { x + 2, y + 2, tw, h - 4 }, tr = { x + w * 0.5f + 1, y + 2, tw, h - 4 };
    // Altitude tape: white tape, black numerals, scale changes like the real
    // one (20 ft per mark below 200 ft ... 1000 ft per mark above 10000 ft).
    pc_glow_paint(backlight() * 0.8f);
    DrawRectangleRec(ta, (Color){ 215, 212, 200, 255 });
    double step = alt < 200 ? 20 : alt < 2000 ? 100 : alt < 10000 ? 500 : 1000;
    double ppu = ta.height / (step * 8);
    for (double v = floor((alt - step * 5) / step) * step; v < alt + step * 5; v += step) {
        float yy = (float)(ta.y + ta.height / 2 - (v - alt) * ppu);
        if (yy < ta.y + 0.5f || yy > ta.y + ta.height - 0.5f) continue;
        DrawLineEx((Vector2){ ta.x, yy }, (Vector2){ ta.x + 5, yy }, 0.7f, (Color){ 10, 10, 10, 255 });
        float half = (float)(ta.y + ta.height / 2 - (v + step / 2 - alt) * ppu);
        if (half > ta.y + 0.5f && half < ta.y + ta.height - 0.5f)
            DrawLineEx((Vector2){ ta.x, half }, (Vector2){ ta.x + 2.5f, half }, 0.5f, (Color){ 10, 10, 10, 255 });
        char s[16];
        snprintf(s, sizeof s, "%.0f", v);
        if (v >= 0 && yy > ta.y + NUM * 0.55f && yy < ta.y + ta.height - NUM * 0.55f)
            DrawTextEx(font_bold, s, (Vector2){ ta.x + 6.5f, yy - NUM * 0.5f }, NUM, 0.1f, (Color){ 10, 10, 10, 255 });
    }
    pc_glow_off();
    DrawRectangleRec(tr, (Color){ 20, 20, 22, 255 });
    pc_glow_paint(backlight());
    double rstep = fabs(rate) < 50 ? 5 : 50;
    double rppu = tr.height / (rstep * 8);
    for (double v = floor((rate - rstep * 5) / rstep) * rstep; v < rate + rstep * 5; v += rstep) {
        float yy = (float)(tr.y + tr.height / 2 - (v - rate) * rppu);
        if (yy < tr.y + 0.5f || yy > tr.y + tr.height - 0.5f) continue;
        DrawLineEx((Vector2){ tr.x, yy }, (Vector2){ tr.x + 5, yy }, 0.7f, C_WHITE);
        char s[16];
        snprintf(s, sizeof s, "%+.0f", v);
        if (yy > tr.y + NUM * 0.55f && yy < tr.y + tr.height - NUM * 0.55f)
            DrawTextEx(font_bold, s, (Vector2){ tr.x + 6.5f, yy - NUM * 0.5f }, NUM, 0.1f, C_WHITE);
    }
    pc_glow_off();
    // Index line across both tapes.
    pc_glow_paint(0.9f);
    DrawRectangleRec((Rectangle){ x, y + h / 2 - 0.6f, w, 1.2f }, (Color){ 255, 150, 30, 255 });
    pc_glow_off();
    if (nodata) {
        pc_glow_paint(0.7f);
        DrawRectangleRec((Rectangle){ ta.x + 1, ta.y + 6, tw - 2, 10 }, (Color){ 200, 50, 40, 255 });
        Vector2 m = pc_measure("NO TRACK", 4.6f, 0.1f);
        DrawTextEx(font_bold, "NO TRACK", (Vector2){ ta.x + (tw - m.x) / 2, ta.y + 8.7f }, 4.6f, 0.1f, WHITE);
        pc_glow_off();
    }
}

static void caution_matrix(float x, float y, const char *const *labels, int cols, int rows, const int *on, Color col)
{
    float w = 68, h = 15, g = 2;
    rect(x - 3, y - 3, cols * (w + g) - g + 6, rows * (h + g) - g + 6, (Color){ 22, 22, 22, 255 });
    for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++) {
            int i = r * cols + c;
            lamp(x + c * (w + g), y + r * (h + g), w, h, labels[i], col, on ? on[i] : 0);
        }
}

// --- Panels -----------------------------------------------------------------------

static const Color RED_L = { 255, 60, 45, 255 }, AMB_L = { 255, 175, 40, 255 }, BLU_L = { 90, 160, 255, 255 },
                   WHT_L = { 250, 245, 225, 255 }, GRN_L = { 90, 255, 120, 255 };

static int prop_low(void) { return lm.m_dps / 8248.0 < 0.056; }

static void panel1(void)
{
    const DskyState *d = agc_host_dsky();
    pushbtn(12, 12, 44, 30, ID_MA1, "MASTER\nALARM", RED_L, master_alarm, 0);
    static const char *const warn[15] = { "CES AC", "CES DC", "AGS", "LGC", "ISS", "RCS TCA", "RCS A REG", "RCS B REG",
                                          "DES REG", "ASC PRESS", "ECS", "CO2 HI", "SUIT/FAN", "GLYCOL", "H2O SEP" };
    int on[15] = { 0 };
    on[3] = d->restart;           // LGC warning
    on[4] = d->no_att;            // ISS
    caution_matrix(78, 12, warn, 5, 3, on, RED_L);
    lamp(12, 50, 44, 12, "LUNAR CONTACT", BLU_L, lm.probe_contact);
    TXT_C("WARNING", 78 + 175, 66, 2.8f);

    x_pointer(16, 108, 112, 0);
    fdai(250, 196, 72);
    alt_tapes(362, 98, 92, 220);

    // T/W indicator and THRUST.
    float tw = (float)(lm.thrust / (lm_mass() * 1.623));
    meter(16, 345, 12, 90, "T/W", tw / 5.0f, "5", "0");
    meter(52, 345, 12, 90, "THRUST", (float)(lm.thrust / 46978.0), "100", "0");
    TXT_C("%", 58, 438, 2.4f);
    meter(70, 345, 12, 90, NULL, (float)(lm.thr_bits * 12.53 / 46978.0), NULL, NULL);
    TXT_C("ACT  CMD", 63, 441, 2.2f);

    // Propellant quantity and event timer (EL numerics).
    group(94, 344, 116, 36, "PRPLNT QTY %");
    char b[16];
    int q = (int)(lm.m_dps / 8248.0 * 100 + 0.5);
    if (q > 99) q = 99;
    snprintf(b, sizeof b, "%02d", q);
    TXT_C("FUEL", 120, 349, 2.6f);
    el_power = !cb_out("PRPLNT QTY");
    el_digits(110, 355, 12, b);
    snprintf(b, sizeof b, "%02d", q > 0 ? q - (q > 1) : 0);
    TXT_C("OXID", 182, 349, 2.6f);
    el_digits(172, 355, 12, b);
    group(94, 396, 116, 36, "EVENT TIMER");
    int et = (int)event_timer;
    snprintf(b, sizeof b, "%02d:%02d", (et / 60) % 100, et % 60);
    el_power = !cb_out("EVENT TIMER");
    el_digits(112, 407, 13, b);
    el_power = 1;

    toggle(240, 372, "RATE/ERR MON", "RATE/ERR MON", "LDG RDR/CMPTR", NULL, "RNDZ RDR", -1, 0);
    toggle(300, 372, "ATT MON", "ATT MON", "PGNS", NULL, "AGS", 1, 0);
    toggle(360, 372, ID_XSCALE, "X-PTR SCALE", "LO MULT", NULL, "HI MULT", 1, 0);
    toggle(420, 372, ID_RATESCALE, "RATE SCALE", "25 DEG/S", NULL, "5 DEG/S", -1, 0);
    toggle(240, 420, ID_MODESEL, "MODE SEL", "LDG RDR", "PGNS", "AGS", 1, 0);
    toggle(300, 420, "ALT/RNG", "RNG/ALT MON", "ALT/ALT RT", NULL, "RNG/RNG RT", 1, 0);
    toggle(360, 420, "SHIFT/TRUNNION", "SHFT/TRUN", "+50", "SLEW", "-50", 0, 0);
    toggle(420, 420, "SLEW RATE", "SLEW RATE", "HI", NULL, "LO", -1, 0);

    group(10, 462, 250, 92, "ENGINE THRUST CONT");
    toggle(38, 500, "THR CONT", "THR CONT", "AUTO", NULL, "MAN", 1, 0);
    toggle(96, 500, "MAN THROT", "MAN THROT", "CDR", NULL, "LMP", 1, 0);
    toggle(154, 500, "DES ENG CMD OVRD", "CMD OVRD", "ON", NULL, "OFF", -1, 1);
    toggle(212, 500, "ATT/TRANSL", "ATT/TRANSL", "4 JETS", NULL, "2 JETS", 1, 0);
    toggle(38, 538, "THROTTLE/JETS", "THROTTLE/JETS", "JETS", NULL, "THROTTLE", -1, 0);
    toggle(96, 538, "BAL CPL", "BAL CPL", "ON", NULL, "OFF", 1, 0);
    toggle(154, 538, "GUID CONT", "GUID CONT", "PGNS", NULL, "AGS", 1, 0);
    toggle(212, 538, "DEADBAND", "DEADBAND", "MIN", NULL, "MAX", 1, 0);

    group(268, 462, 190, 92, "MAIN PROPULSION");
    // The four meters show whichever tank set TEMP/PRESS MON selects:
    // supercritical helium, descent tank set 1 or 2, or the ascent tanks.
    static const float pm[4][4] = { { 0.55f, 0.20f, 0.20f, 0.30f }, { 0.55f, 0.52f, 0.50f, 0.55f },
                                    { 0.55f, 0.53f, 0.51f, 0.56f }, { 0.74f, 0.50f, 0.49f, 0.58f } };
    int ps = ST(ID_PRPMON, 1);
    if (ps < 0 || ps > 3) ps = 1;
    float wob = 0.02f * sinf((float)GetTime() * 0.1f), burn = lm.dps_on && (ps == 1 || ps == 2) ? 0.07f : 0;
    meter(282, 482, 11, 58, "HE", pm[ps][0] + wob, "4", "0");
    meter(312, 482, 11, 58, "FUEL T", pm[ps][1], "100", "20");
    meter(342, 482, 11, 58, "OXID T", pm[ps][2], "100", "20");
    meter(372, 482, 11, 58, "PRESS", pm[ps][3] + burn, "300", "0");
    static const char *const mon[4] = { "HE", "DES 1", "DES 2", "ASC" };
    rotary(425, 516, ID_PRPMON, "TEMP/PRESS", mon, 4, 1, 120);

    hazard_frame(274, 570, 180, 60, 3.2f);
    pushbtn(282, 578, 70, 44, ID_ABORT, "ABORT", RED_L, 0, 0);
    pushbtn(372, 578, 74, 44, ID_ABSTAGE, "ABORT\nSTAGE", RED_L, 0, 1);
    toggle(38, 604, "SBD AUD", "AUDIO", "T/R", "RCV", "OFF", 1, 0);
    toggle(96, 604, "ICS", "ICS", "PTT", NULL, "VOX", 1, 0);
    toggle(154, 604, "ALT RT/OFF", "ALT RT", "ON", NULL, "OFF", 1, 0);
    toggle(212, 604, "CDR XPTR", "X-PTR", "PGNS", NULL, "AGS", 1, 0);
}

static void panel2(void)
{
    pushbtn(411, 12, 44, 30, ID_MA2, "MASTER\nALARM", RED_L, master_alarm, 0);
    static const char *const caut[15] = { "ASC HI REG", "DES QTY", "ED RELAYS", "S-BD RCVR", "RNDZ RDR",
                                          "INVERTER", "BATTERY", "C/W PWR", "HEATER", "LDG RDR",
                                          "PRE AMPS", "ENG GMBL", "RCS", "O2 QTY", "WATER QTY" };
    int on[15] = { 0 };
    on[1] = prop_low();
    caution_matrix(12, 12, caut, 5, 3, on, AMB_L);
    TXT_C("CAUTION", 12 + 175, 66, 2.8f);
    lamp(411, 50, 44, 12, "LUNAR CONTACT", BLU_L, lm.probe_contact);

    // Mission timer (GET).
    group(12, 82, 150, 30, "MISSION TIMER");
    char b[24];
    int t = (int)(mission_timer < 0 ? lm.t : mission_timer);
    snprintf(b, sizeof b, "%03d:%02d:%02d", (t / 3600) % 1000, (t / 60) % 60, t % 60);
    el_power = !cb_out("MISSION TIMER");
    el_digits(20, 90, 14, b);
    el_power = 1;

    fdai(215, 210, 72);
    x_pointer(338, 108, 112, 1);

    group(10, 330, 448, 110, "REACTION CONTROL");
    float rq = (float)(lm.m_rcs / 287.0);
    meter(24, 352, 11, 60, "TEMP", 0.48f, "100", "20");
    meter(46, 352, 11, 60, NULL, 0.47f, NULL, NULL);
    meter(78, 352, 11, 60, "PRESS", 0.60f, "400", "0");
    meter(100, 352, 11, 60, NULL, 0.61f, NULL, NULL);
    meter(132, 352, 11, 60, "QTY", rq, "100", "0");
    meter(154, 352, 11, 60, NULL, rq * 0.98f, NULL, NULL);
    TXT_C("A   B", 35, 416, 2.4f);
    TXT_C("A   B", 89, 416, 2.4f);
    TXT_C("A   B", 143, 416, 2.4f);
    TXT_C("THRUSTER PAIR CMD", 320, 340, 2.8f);
    static const char *const q[8] = { "QUAD 4", "QUAD 3", "QUAD 2", "QUAD 1", "4 DN", "3 DN", "2 DN", "1 DN" };
    static const char *const qid[8] = { "Q4ISOL", "Q3ISOL", "Q2ISOL", "Q1ISOL", "Q4DN", "Q3DN", "Q2DN", "Q1DN" };
    for (int i = 0; i < 4; i++) {
        TXT_C(q[i], 210 + i * 64, 352, 2.6f);
        talkback(210 + i * 64 - 12, 366, 1);
        talkback(210 + i * 64 + 12, 366, 1);
        toggle(210 + i * 64, 396, qid[i], NULL, "OPEN", NULL, "CLOSE", 1, 0);
        (void)qid[i + 4];
    }

    group(10, 456, 448, 170, "ENVIRONMENTAL CONTROL");
    meter(26, 482, 11, 64, "SUIT", 0.48f, "10", "0");
    meter(60, 482, 11, 64, "CABIN", 0.48f, "10", "0");
    meter(94, 482, 11, 64, "CO2", 0.05f, "30", "0");
    meter(128, 482, 11, 64, "GLY T", 0.55f, "80", "20");
    meter(162, 482, 11, 64, "SUIT T", 0.45f, "80", "20");
    meter(196, 482, 11, 64, "H2O QTY", 0.70f, "100", "0");
    TXT_C("PSIA  PSIA  MMHG  DEG F  DEG F   %", 112, 552, 2.4f);
    toggle(254, 500, ID_SUITFAN, "SUIT FAN", "1", NULL, "2", 1, 0);
    toggle(304, 500, "GLYCOL", "GLYCOL", "PUMP 1", "INST", "PUMP 2", 1, 0);
    toggle(354, 500, "QTY MON", "QTY MON", "DEADFACE", NULL, "RESET", 1, 0);
    toggle(404, 500, "TEMP/PRESS MON", "ECS MON", "SUIT", "CABIN", "GLYCOL", 0, 0);
    toggle(254, 580, "CABIN REPRESS", "CAB REPRESS", "AUTO", "MAN", "CLOSE", 1, 1);
    toggle(304, 580, "O2 DEMAND", "O2 DEMAND", "1", NULL, "2", 1, 0);
    toggle(354, 580, "H2O SEP", "H2O SEP", "1", NULL, "2", 1, 0);
    toggle(404, 580, "LMP XPTR", "X-PTR", "PGNS", NULL, "AGS", -1, 0);
    talkback(40, 592, 1);
    talkback(80, 592, 0);
    talkback(120, 592, 1);
    TXT_C("SUIT DIV  CO2 CAN  REPRESS", 82, 604, 2.4f);
}

static void panel3(void)
{
    group(8, 18, 236, 252, "RADAR");
    static const char *const tm[6] = { "RNDZ", "LDG", "S-BD", "AFT", "QUAD 1", "QUAD 4" };
    static const float tmv[6] = { 0.30f, 0.45f, 0.52f, 0.40f, 0.58f, 0.57f };
    int tsel = rotary(48, 66, ID_TEMPMON, "TEMP MONITOR", tm, 6, 1, 200);
    dial(150, 66, 20, "TEMP", tmv[tsel], "-100", "+200");
    // Signal strength of the selected transmitter: the landing radar's two
    // (altitude, velocity) follow the radar's lock; the rendezvous radar is off.
    static const char *const ss[6] = { "ALT XMTR", "VEL XMTR", "RNDZ", "S-BD", "VHF", "CDR" };
    int ssel = ST(ID_SIGMON, 0);
    float lock = 0.72f + 0.04f * sinf((float)GetTime() * 3);
    float sig = ssel == 0 ? (lm.lr_range_good ? lock : 0.05f) : ssel == 1 ? (lm.lr_vel_good ? lock - 0.06f : 0.05f) :
                ssel == 2 ? 0.0f : ssel == 3 ? 0.78f : ssel == 4 ? 0.66f : 0.5f;
    dial(48, 160, 20, "SIGNAL STRENGTH", sig, "0", "5");
    rotary(150, 160, ID_SIGMON, "SIG STR MON", ss, 6, 0, 200);
    toggle(40, 236, "RDR TEST", "RDR TEST", "LDG", "OFF", "RNDZ", 0, 0);
    toggle(100, 236, "RNDZ RDR", "RNDZ RDR", "LGC", "AUTO TRK", "SLEW", 1, 0);
    toggle(160, 236, "LDG ANT", "LDG ANT", "AUTO", "DES", "HOVER", 1, 0);
    toggle(212, 236, "LR XMTR", "LR XMTR", "ON", NULL, "OFF", 1, 0);

    group(252, 18, 210, 252, "ATTITUDE / TRANSLATION");
    toggle(282, 64, "ROLL ATT", "ROLL", "PULSE", "MODE CONT", "DIR", 0, 0);
    toggle(340, 64, "PITCH ATT", "PITCH", "PULSE", "MODE CONT", "DIR", 0, 0);
    toggle(398, 64, "YAW ATT", "YAW", "PULSE", "MODE CONT", "DIR", 0, 0);
    toggle(282, 140, "GYRO TEST", "GYRO TEST", "POS RT", "OFF", "NEG RT", 0, 0);
    toggle(340, 140, "ACA PROP", "ACA PROP", "ENABLE", NULL, "DISABLE", 1, 0);
    toggle(398, 140, "SCS MODE", "SCS MODE", "AUTO", NULL, "ATT HOLD", 1, 0);
    toggle(282, 220, "LMP X-TRANSL", "+X TRANSL", "4 JETS", NULL, "2 JETS", 1, 0);
    toggle(340, 220, "RCS SYS A/B", "RCS SYS A/B", "OPEN", "", "CLOSE", 0, 0);
    toggle(398, 220, "IMU CAGE", "IMU CAGE", "ON", NULL, "OFF", -1, 1);

    group(470, 18, 340, 252, "STABILIZATION / CONTROL");
    TXT_C("MODE CONTROL", 560, 36, 3.4f);
    toggle(520, 84, ID_PGNS, "PGNS", "AUTO", "ATT HOLD", "OFF", 1, 0);
    toggle(600, 84, "MODE CONTROL AGS", "AGS", "AUTO", "ATT HOLD", "OFF", 0, 0);
    TXT_C("ENG ARM", 720, 36, 3.4f);
    toggle(720, 84, ID_ENGARM, NULL, "ASC", "OFF", "DES", 0, 1);
    pushbtn(498, 130, 52, 24, "ABORT STAGE RESET", "STAGE\nRESET", WHT_L, 0, 0);
    toggle(600, 150, "STAGE", "STAGE", "FIRE", NULL, "SAFE", -1, 1);
    toggle(660, 150, "ENG GMBL", "ENG GMBL", "ENABLE", NULL, "OFF", 1, 0);
    toggle(720, 150, "DES ENG", "DES ENG", "ON", NULL, "OFF", 1, 0);
    toggle(780, 150, "ATCA", "ATCA", "PGNS", NULL, "AGS", 1, 0);
    toggle(520, 226, "TTCA/THROT", "TTCA THROT", "CDR", NULL, "LMP", 1, 0);
    toggle(580, 226, "TTCA JETS", "TTCA JETS", "4", NULL, "2", 1, 0);
    toggle(640, 226, "ATT CONT DEADBAND", "DEADBAND", "MIN", NULL, "MAX", 1, 0);
    toggle(700, 226, "PROP PRESS", "PROP PRESS", "ON", NULL, "OFF", -1, 1);
    toggle(760, 226, ID_ETRESET, "EVT TMR", "RESET", "", "COUNT", 0, 0);

    group(818, 18, 214, 252, "HEATERS / INST");
    static const char *const hm[7] = { "RNDZ", "LDG", "S-BD", "QUAD 1", "QUAD 2", "QUAD 3", "QUAD 4" };
    static const float hmv[7] = { 0.44f, 0.50f, 0.55f, 0.62f, 0.61f, 0.63f, 0.62f };
    int hsel = rotary(870, 70, ID_HTRMON, "HEATER MON", hm, 7, 1, 220);
    dial(970, 70, 20, "HTR TEMP", hmv[hsel], "-200", "+200");
    toggle(850, 160, "SBD ANT HTR", "S-BD ANT", "ON", NULL, "OFF", -1, 0);
    toggle(910, 160, "LDG RDR HTR", "LDG RDR", "ON", NULL, "OFF", 1, 0);
    toggle(970, 160, "RNDZ RDR HTR", "RNDZ RDR", "ON", NULL, "OFF", 1, 0);
    toggle(850, 236, "INST SIG SENSOR", "SIG SENSOR", "ON", NULL, "OFF", 1, 0);
    toggle(910, 236, "PCM", "PCM", "HI", NULL, "LO", 1, 0);
    toggle(970, 236, "TAPE RCDR", "TAPE RCDR", "ON", NULL, "OFF", -1, 0);

    group(1040, 18, 232, 252, "COMM / LTG");
    toggle(1070, 64, "SBD MODE", "S-BAND", "PM", "OFF", "FM", 1, 0);
    toggle(1130, 64, "SBD XMTR", "XMTR", "PRIM", NULL, "SEC", 1, 0);
    toggle(1190, 64, "SBD PWR AMPL", "PWR AMPL", "PRIM", "OFF", "SEC", 1, 0);
    toggle(1246, 64, "VOICE", "VOICE", "VOICE", "OFF", "DN VOICE BU", 1, 0);
    toggle(1070, 140, "VHF A", "VHF A", "VOICE", "OFF", "DATA", 1, 0);
    toggle(1130, 140, "VHF B", "VHF B", "VOICE", "OFF", "DATA", -1, 0);
    toggle(1190, 140, "SQUELCH", "SQUELCH", "ON", NULL, "OFF", 1, 0);
    toggle(1246, 140, "TRACK LT", "TRACK LT", "ON", NULL, "OFF", -1, 0);
    toggle(1070, 226, "DOCK LTS", "DOCK LTS", "ON", NULL, "OFF", -1, 0);
    toggle(1130, 226, ID_LTGTEST, "LAMP TEST", "TEST", NULL, "OFF", -1, 0);
    static const char *const fl[11] = { "OFF", "", "", "", "", "", "", "", "", "", "BRT" };
    rotary(1220, 222, ID_INTEG, "INTEGRAL", fl, 11, 6, 270);
}

// Panel 4: the LGC DSKY.
static Rectangle dsky_mm = { 120, 12, 200, 252 };
static Rectangle dsky_native_rect(void);

static void panel4(void)
{
    TXT_C("LGC DISPLAY AND KEYBOARD", 220, 6, 3.0f);
    rect(dsky_mm.x - 3, dsky_mm.y - 3, dsky_mm.width + 6, dsky_mm.height + 6, (Color){ 30, 30, 32, 255 });
    // The DSKY draws itself in its own units, scaled into its cut-out.
    Rectangle n = dsky_native_rect();
    // Key caps are modelled in the cabin; the DSKY draws their tops at that height.
    dsky_set_key_lift((KEY_UP + 0.12f) / n.width, (KEY_DOWN + 0.12f) / n.width);
    for (int i = 0; ; i++) {
        Rectangle k;
        int down;
        if (!dsky_key_layout(i, &k, &down)) break;
        float kx = n.x + (k.x + k.width / 2) * n.width, ky = n.y + (k.y + k.height / 2) * n.width;
        add_part(LEV_KEY, kx, ky, down ? KEY_DOWN : KEY_UP, 0, k.width * n.width, k.height * n.width, 0, (Color){ 20, 20, 22, 255 });
        if (hint_tgt.kind == HT_DSKY_KEY && dsky_key_index(hint_tgt.arg) == i) {
            // The DSKY rings the key itself; note where it is for the off-screen pointer.
            hint_rec[cur_panel].on = 1;
            hint_rec[cur_panel].x = kx;
            hint_rec[cur_panel].y = ky;
            hint_rec[cur_panel].text = hint_tgt.text;
        }
    }
    pc_push(n.x, n.y, n.width);
    dsky_draw_canvas(cp_numerics_level() * (ST(ID_DSKYLTG, 1) > 0 ? 1.0f : 0.45f));
    pc_pop();
    add_hot(HOT_DSKY, dsky_mm, "DSKY", 0);
    // Side switches: X-pointer scale, attitude monitor... and the PGNS lighting.
    toggle(48, 80, ID_DSKYLTG, "DSKY", "BRT", NULL, "DIM", 1, 0);
    toggle(48, 150, "LGC PWR", "LGC", "OPR", NULL, "STBY", 1, 1);
    toggle(48, 220, "IMU PWR", "IMU", "OPR", NULL, "STBY", 1, 1);
    static const char *const nm[11] = { "OFF", "", "", "", "", "", "", "", "", "", "BRT" };
    rotary(392, 80, ID_NUM, "ANUN/NUM", nm, 11, 8, 270);
    toggle(392, 170, "ALT/VEL LTS", "ALT/VEL LTS", "ON", NULL, "OFF", 1, 0);
    toggle(392, 236, "COMP ACTY", "COMP ACTY", "ON", NULL, "OFF", 1, 0);
}

// Panel 5: CDR lighting, timers and the engine start/stop buttons.
static void panel5(void)
{
    group(8, 18, 344, 92, "TIMERS");
    toggle(40, 64, ID_MT, "MSN TIMER", "START", "STOP", "RESET", 1, 0);
    thumbwheel(90, 64, ID_MTH, "HRS");
    thumbwheel(118, 64, ID_MTM, "MIN");
    thumbwheel(146, 64, ID_MTS, "SEC");
    toggle(196, 64, ID_ET, "EVENT TIMER", "START", "STOP", "RESET", 1, 0);
    thumbwheel(246, 64, ID_ETM, "MIN");
    thumbwheel(274, 64, ID_ETS, "SEC");
    toggle(322, 64, ID_ETCOUNT, "COUNT", "UP", NULL, "DOWN", 1, 0);

    group(8, 124, 166, 150, "LIGHTING");
    static const char *const fl[11] = { "OFF", "", "", "", "", "", "", "", "", "", "BRT" };
    rotary(48, 170, ID_FLOOD, "FLOOD", fl, 11, 5, 270);
    toggle(120, 170, ID_FLOODSEL, "FLOOD", "ALL", NULL, "POST", 1, 0);
    toggle(48, 244, ID_ANUNBRT, "ANUN/NUM", "BRT", NULL, "DIM", 1, 0);
    toggle(120, 244, ID_SIDEPNL, "SIDE PNLS", "ON", NULL, "OFF", 1, 0);

    group(184, 124, 168, 150, "ENGINE");
    hazard_frame(190, 132, 58, 42, 2.6f);
    hazard_frame(254, 126, 92, 56, 2.8f);
    pushbtn(196, 138, 46, 30, ID_START5, "ENGINE\nSTART", GRN_L, 0, 0);
    pushbtn(260, 132, 80, 44, ID_STOP5, "ENGINE\nSTOP", RED_L, lm_engine_stop_is_latched(), 0);
    pushbtn(196, 184, 46, 30, ID_PLUSX5, "PLUS X", WHT_L, plusx_timer > 0, 0);
    int rod = rod_anim > 0 ? 1 : rod_anim < 0 ? -1 : 0;
    *state_of(ID_ROD, 0) = rod;
    toggle(300, 230, ID_ROD, "RATE OF DESCENT", "-", NULL, "+", 0, 0);
    TXT_C("(SPRING)", 300, 252, 2.4f);
    toggle(222, 246, "MAN THROT START", "START OVRD", "ON", NULL, "OFF", -1, 1);
}

// Panel 6: LMP side - the Abort Guidance System DEDA.
static void panel6(void)
{
    group(8, 18, 230, 256, "ABORT GUIDANCE - DEDA");
    rect(22, 32, 202, 50, (Color){ 22, 22, 24, 255 });
    TXT_C("ADDRESS", 60, 35, 2.6f);
    TXT_C("DATA", 160, 35, 2.6f);
    el_digits(36, 44, 16, deda_addr);
    el_digits(100, 44, 16, deda_data);
    lamp(176, 66, 40, 12, "OPR ERR", WHT_L, deda_err);
    static const char *const keys[16] = { "+", "7", "8", "9", "-", "4", "5", "6", "0", "1", "2", "3", "CLR", "READ\nOUT", "ENTR", "HOLD" };
    for (int i = 0; i < 16; i++) {
        float x = 32 + (i % 4) * 48, y = 96 + (i / 4) * 42;
        int dn = pressed(ID_DEDA, i);
        float up = dn ? KEY_DOWN : KEY_UP;
        add_part(LEV_KEY, x + 19, y + 16, up, 0, 38, 32, 0, (Color){ 20, 20, 22, 255 });
        pc_lift(up + 0.12f);
        DrawRectangleRounded((Rectangle){ x, y, 38, 32 }, 0.15f, 4, dn ? (Color){ 66, 68, 72, 255 } : (Color){ 32, 33, 35, 255 });
        DrawRectangleRoundedLinesEx((Rectangle){ x, y, 38, 32 }, 0.15f, 4, 0.6f, (Color){ 12, 12, 12, 255 });
        float sz = strlen(keys[i]) > 2 ? 6.0f : 11.0f;
        pc_glow(el_col(0.5f));
        pc_text_box(keys[i], (Rectangle){ x, y, 38, 32 }, sz, 0.1f, C_WHITE);
        pc_glow_off();
        pc_lift(0);
        add_hot(HOT_DEDA, (Rectangle){ x, y, 38, 32 }, ID_DEDA, i);
    }
    group(248, 18, 104, 256, "ENGINE");
    hazard_frame(258, 34, 86, 56, 2.8f);
    pushbtn(264, 40, 74, 44, ID_STOP6, "ENGINE\nSTOP", RED_L, lm_engine_stop_is_latched(), 0);
    pushbtn(272, 104, 58, 30, ID_PLUSX6, "PLUS X", WHT_L, plusx_timer > 0, 0);
    toggle(300, 176, "AGS STATUS", "AGS STATUS", "OPERATE", "STBY", "OFF", 1, 1);
    toggle(300, 240, "AGS SOURCE", "DEDA", "ON", NULL, "OFF", 1, 0);
}

static void panel8(void)
{
    group(8, 16, 360, 196, "AUDIO - CDR");
    const char *ch[7] = { "S-BAND", "ICS", "RELAY", "VHF A", "VHF B", "VOX", "MASTER" };
    const char *id[7] = { "A8 SB", "A8 ICS", "A8 RLY", "A8 VHFA", "A8 VHFB", "A8 VOX", "A8 MAS" };
    const char *idt[5] = { "A8 SBT", "A8 ICST", "A8 RLYT", "A8 VHFAT", "A8 VHFBT" };
    for (int i = 0; i < 7; i++) {
        thumbwheel(36 + i * 48, 64, id[i], ch[i]);
        if (i < 5) toggle(36 + i * 48, 150, idt[i], NULL, "T/R", "RCV", "OFF", 1, 0);
    }
    toggle(276, 150, "A8 MODE", "MODE", "ICS/PTT", "ICS/VOX", "VOX", 0, 0);
    toggle(330, 150, "A8 AUDIO CONT", "AUDIO CONT", "NORM", NULL, "UP DATA", 1, 0);

    group(380, 16, 410, 196, "EXPLOSIVE DEVICES");
    toggle(418, 70, "ED SYS A", "SYS A", "ARM", NULL, "OFF", -1, 1);
    toggle(476, 70, "ED SYS B", "SYS B", "ARM", NULL, "OFF", -1, 1);
    toggle(534, 70, "ED MASTER", "MASTER ARM", "ON", NULL, "OFF", -1, 1);
    talkback(592, 54, 1);
    TXT_C("LDG GEAR", 592, 44, 2.6f);
    toggle(592, 86, "ED LDG GEAR", "DEPLOY", "FIRE", NULL, "SAFE", -1, 1);
    talkback(652, 54, 1);
    TXT_C("DES He", 652, 44, 2.6f);
    toggle(652, 86, "ED DES HE", "He PRESS", "FIRE", NULL, "SAFE", -1, 1);
    toggle(712, 86, "ED DES VENT", "DES VENT", "FIRE", NULL, "SAFE", -1, 1);
    toggle(418, 158, "ED ASC HE", "ASC He", "FIRE", NULL, "SAFE", -1, 1);
    toggle(476, 158, "ED ASC PRPLNT", "ASC PRPLNT", "FIRE", NULL, "SAFE", -1, 1);
    toggle(534, 158, "ED DES ISOL", "DES PRPLNT ISOL VLV", "FIRE", NULL, "SAFE", -1, 1);
    toggle(652, 158, "ED STAGE", "STAGE", "FIRE", NULL, "SAFE", -1, 1);
    talkback(712, 140, 0);
    TXT_C("STAGE", 712, 152, 2.4f);
}

static void panel12(void)
{
    group(8, 16, 360, 196, "AUDIO - LMP");
    const char *ch[7] = { "S-BAND", "ICS", "RELAY", "VHF A", "VHF B", "VOX", "MASTER" };
    const char *id[7] = { "A12 SB", "A12 ICS", "A12 RLY", "A12 VHFA", "A12 VHFB", "A12 VOX", "A12 MAS" };
    const char *idt[5] = { "A12 SBT", "A12 ICST", "A12 RLYT", "A12 VHFAT", "A12 VHFBT" };
    for (int i = 0; i < 7; i++) {
        thumbwheel(36 + i * 48, 64, id[i], ch[i]);
        if (i < 5) toggle(36 + i * 48, 150, idt[i], NULL, "T/R", "RCV", "OFF", 1, 0);
    }
    toggle(276, 150, "A12 MODE", "MODE", "ICS/PTT", "ICS/VOX", "VOX", 0, 0);
    toggle(330, 150, "A12 AUDIO CONT", "AUDIO CONT", "NORM", NULL, "UP DATA", 1, 0);

    group(380, 16, 410, 196, "COMMUNICATIONS");
    toggle(418, 70, "C12 SBD ANT", "S-BD ANT", "SLEW", "AUTO", "OFF", 0, 0);
    static const char *const ant[4] = { "FWD", "AFT", "STEER", "SLEW" };
    rotary(500, 74, "S-BD ANT SEL", "ANT SEL", ant, 4, 2, 150);
    dial(600, 74, 22, "S-BD SIG", 0.62f, "0", "5");
    static const char *const pitch[5] = { "-75", "", "0", "", "+255" };
    rotary(700, 74, "SBD PITCH", "PITCH", pitch, 5, 2, 200);
    toggle(418, 158, "C12 VHF ANT", "VHF ANT", "FWD", NULL, "AFT", 1, 0);
    toggle(476, 158, "C12 TLM", "TLM", "BIOMED", NULL, "OFF", 1, 0);
    toggle(534, 158, "C12 PCM", "PCM", "HI", NULL, "LO", 1, 0);
    toggle(592, 158, "C12 RNG", "RANGE", "TV/CWEA ENABLE", NULL, "OFF", -1, 0);
    toggle(700, 158, "C12 TV", "TV", "ON", NULL, "OFF", -1, 0);
}

static void panel14(void)
{
    group(8, 16, 584, 220, "ELECTRICAL POWER");
    float jitter = 0.01f * sinf((float)GetTime() * 0.7f);
    // Volts and amps of whatever POWER/TEMP MON selects: the two buses, the
    // four descent batteries (carrying the load), the ascent batteries (idle).
    static const char *const mon[9] = { "CDR BUS", "LMP BUS", "BAT 1", "BAT 2", "BAT 3", "BAT 4", "BAT 5", "BAT 6", "XLUNAR" };
    static const float pv[9] = { 0.42f, 0.42f, 0.46f, 0.46f, 0.45f, 0.46f, 0.60f, 0.60f, 0.0f };
    static const float pa[9] = { 0.45f, 0.38f, 0.22f, 0.21f, 0.20f, 0.21f, 0.0f, 0.0f, 0.0f };
    int psel = ST(ID_PWRMON, 0);
    if (psel < 0 || psel > 8) psel = 0;
    float load = pa[psel] > 0 ? pa[psel] + (float)(lm.thrust / 46978.0) * 0.08f * pa[psel] / 0.45f + jitter : 0;
    dial(70, 78, 30, "DC VOLTS", pv[psel] > 0 ? pv[psel] + jitter : 0, "20", "40");
    dial(160, 78, 30, "DC AMPS", load, "0", "120");
    rotary(270, 78, ID_PWRMON, "POWER/TEMP MON", mon, 9, 0, 260);
    for (int i = 0; i < 4; i++) {
        float x = 340 + i * 62;
        char *lab = (char *)(i == 0 ? "DES BAT 1" : i == 1 ? "DES BAT 2" : i == 2 ? "DES BAT 3" : "DES BAT 4");
        TXT_C(lab, x, 40, 2.6f);
        talkback(x, 58, 1);
        static const char *const bid[4] = { "BAT1", "BAT2", "BAT3", "BAT4" };
        toggle(x, 102, bid[i], NULL, "HV", "OFF", "LV", 1, 0);
    }
    toggle(60, 190, "BAT5", "ASC BAT 5", "NORM", "OFF", "BACKUP", 1, 0);
    toggle(130, 190, "BAT6", "ASC BAT 6", "NORM", "OFF", "BACKUP", 1, 0);
    talkback(200, 178, 0);
    talkback(240, 178, 0);
    TXT_C("ASC FEED", 220, 192, 2.4f);
    toggle(300, 190, "INV", "INVERTER", "1", "OFF", "2", 1, 0);
    toggle(370, 190, "XLUNAR BUS", "XLUNAR BUS", "TIE", NULL, "OFF", 1, 0);
    toggle(440, 190, "DES BAT DEADFACE", "DES DEADFACE", "RESET", NULL, "DEADFACE", 1, 0);
    toggle(510, 190, "ASC BAT DEADFACE", "ASC DEADFACE", "RESET", NULL, "DEADFACE", -1, 0);

    group(8, 254, 584, 270, "ECS - LMP");
    meter(40, 290, 12, 80, "O2 MAN", 0.80f, "1000", "0");
    meter(80, 290, 12, 80, "GLY PRESS", 0.42f, "60", "0");
    meter(120, 290, 12, 80, "SUIT P", 0.48f, "10", "0");
    meter(160, 290, 12, 80, "H2O DES", 0.69f, "100", "0");
    meter(200, 290, 12, 80, "H2O ASC", 0.98f, "100", "0");
    toggle(280, 320, "DIVERT VLV", "SUIT GAS DIV", "EGRESS", NULL, "CABIN", 1, 1);
    toggle(350, 320, "CO2 CAN", "CO2 CANISTER", "PRIM", NULL, "SEC", 1, 0);
    toggle(420, 320, "WATER SEP", "WATER SEP", "1", NULL, "2", 1, 0);
    toggle(490, 320, ID_CABFAN, "CABIN FAN", "ON", NULL, "OFF", -1, 0);
    toggle(280, 420, "O2 DES", "DES O2", "OPEN", NULL, "CLOSE", 1, 0);
    toggle(350, 420, "O2 ASC1", "ASC O2 #1", "OPEN", NULL, "CLOSE", -1, 0);
    toggle(420, 420, "O2 ASC2", "ASC O2 #2", "OPEN", NULL, "CLOSE", -1, 0);
    toggle(490, 420, "PLSS FILL", "PLSS FILL", "OPEN", NULL, "CLOSE", -1, 1);
    TXT_C("CABIN PRESSURE 4.8 PSIA NOMINAL", 130, 404, 2.8f);
}

static const char *const cb11[5][24] = {
    { "PHASE MOD", "S-BD XCVR", "S-BD PWR AMPL", "VHF A XMTR", "VHF B RCVR", "PMP", "SE AUDIO", "CDR AUDIO", "TLM PCMTEA",
      "INST SIG CONDR", "INST SIG SENSOR", "PCM/TE", "C/W PWR", "TAPE RCDR", "CDR FDAI", "CDR X-PTR", "ORDEAL", "ATTITUDE DIR CONT",
      "RNDZ RDR", "LDG RDR", "ABORT STAGE", "ATCA", "AELD", "ATCA PGNS" },
    { "DECA PWR", "DES ENG OVRD", "IMU STBY", "IMU OPR", "LGC/DSKY", "SIG SENSOR", "PGNS", "ENG START OVRD", "ENG ARM",
      "GIMBAL PWR", "DES ENG CONT", "QUAD 1", "QUAD 2", "QUAD 3", "QUAD 4", "RCS A ISOL", "RCS B ISOL", "TCA 1",
      "TCA 2", "TCA 3", "TCA 4", "ASC FEED", "DES FEED", "XFEED" },
    { "FLOOD", "ANUN/DOCK/COMPNT", "MISSION TIMER", "EVENT TIMER", "INTGL", "SIDE PNLS", "PORT", "UTIL", "TRACK",
      "DOCK", "HTR DISP", "S-BD ANT", "RNDZ RDR HTR", "LDG RDR HTR", "QUAD 1 HTR", "QUAD 2 HTR", "QUAD 3 HTR", "QUAD 4 HTR",
      "AGS HTR", "RCS A HTR", "RCS B HTR", "CAMR", "SUIT FAN 1", "SUIT FAN 2" },
    { "CABIN REPRESS", "CO2 SENSOR", "GLYCOL PUMP 1", "GLYCOL PUMP 2", "GLYCOL AUTO XFER", "DIVERT VLV", "H2O SEP 1",
      "H2O SEP 2", "CABIN FAN CONT", "SUIT/CABIN P", "O2 DEMAND", "EPS DISP", "INVERTER 1", "INVERTER 2", "BAT FEED TIE",
      "CROSS TIE BUS", "CROSS TIE BAL LOADS", "ASC ECA CONT", "DES ECA", "ED LOGIC PWR A", "ED LOGIC PWR B", "AC BUS A",
      "AC BUS B", "DC BUS" },
    { "PROP DISP", "PRPLNT QTY", "ASC HE REG 1", "ASC HE REG 2", "DES HE REG", "FUEL VENT", "OXID VENT", "ASC PRESS",
      "DES PRPLNT ISOL", "HE PRESS", "RCS SYS A", "RCS SYS B", "THRSTR A", "THRSTR B", "SOV A", "SOV B", "PCM A",
      "PCM B", "STAB/CONT AC", "STAB/CONT DC", "RATE GYRO", "ACA", "TTCA", "ENG GMBL" },
};
static const char *const cbg11[5] = { "COMMUNICATIONS / FLIGHT DISPLAYS", "PROPULSION / GUIDANCE", "LIGHTING / HEATERS",
                                      "ENVIRONMENTAL / ELECTRICAL POWER", "REACTION CONTROL / STAB-CONT" };

static void cb_panel(int lmp)
{
    cb_side = lmp;
    cb_next = 0;
    // Debugging: AGC_CBOPEN=n starts with every n-th breaker pulled.
    static int preset;
    if (!preset) {
        preset = 1;
        const char *e = getenv("AGC_CBOPEN");
        int n = e ? atoi(e) : 0;
        for (int i = 0; n > 0 && i < CB_PER_PANEL; i += n) cb_open[0][i] = cb_open[1][i] = 1;
    }
    TXT_C(lmp ? "CB PANEL 16 - LMP BUS" : "CB PANEL 11 - CDR BUS", 575, 6, 4.0f);
    for (int r = 0; r < 5; r++) {
        float y = 44 + r * 104;
        group(8, y - 22, 1134, 92, cbg11[(r + lmp * 2) % 5]);
        for (int i = 0; i < 24; i++) {
            const char *lab = cb11[(r + lmp * 2) % 5][(i + lmp * 7) % 24];
            float x = 34 + i * 46.5f;
            breaker(x, y + 18, lab);
            breaker(x, y + 46, NULL);
        }
    }
}

void cp_draw_panel(int panel)
{
    cur_panel = panel;
    drawing_panel = 1;
    cp_items[panel].nlev = 0;
    cp_items[panel].nhot = 0;
    hints_target(&hint_tgt);
    mark.on = 0;
    hint_rec[panel].on = 0;
    cp_live_panels &= ~(1u << panel);
    const PanelDef *pd = &cp_panels[panel];
    DrawRectangleRec((Rectangle){ 0, 0, pd->w_mm, pd->h_mm }, C_PANEL);
    // Mounting screws in the corners, as on the real panels.
    float m = 4.5f;
    Vector2 sc[4] = { { m, m }, { pd->w_mm - m, m }, { m, pd->h_mm - m }, { pd->w_mm - m, pd->h_mm - m } };
    for (int i = 0; i < 4; i++) {
        DrawCircleV(sc[i], 2.0f, (Color){ 120, 122, 126, 255 });
        DrawLineEx((Vector2){ sc[i].x - 1.4f, sc[i].y }, (Vector2){ sc[i].x + 1.4f, sc[i].y }, 0.5f, (Color){ 50, 50, 52, 255 });
    }
    switch (panel) {
    case P1: panel1(); break;
    case P2: panel2(); break;
    case P3: panel3(); break;
    case P4: panel4(); break;
    case P5: panel5(); break;
    case P6: panel6(); break;
    case P8: panel8(); break;
    case P11: cb_panel(0); break;
    case P12: panel12(); break;
    case P14: panel14(); break;
    case P16: cb_panel(1); break;
    }
    draw_mark();
    drawing_panel = 0;
}

// --- Logic --------------------------------------------------------------------------

void cp_panels_update(float dt)
{
    // Keep the functional switches in step with the simulation (keyboard
    // shortcuts and the automatic crew change the same things).
    ST(ID_ENGARM, 0) = lm.engine_armed ? -1 : 0;
    int m = lm_get_attitude_mode();
    ST(ID_PGNS, 1) = m == 2 ? 1 : m == 1 ? 0 : -1;

    if (rod_anim > 0) rod_anim = fmaxf(0, rod_anim - dt);
    if (rod_anim < 0) rod_anim = fminf(0, rod_anim + dt);
    if (plusx_timer > 0) {
        plusx_timer -= dt;
        if (plusx_timer <= 0) lm_set_thc(0, 0);
    }
    if (press_t > 0) {
        press_t -= dt;
        if (press_t <= 0) cp_dirty = 1;   // the cap comes back up
    }
    // Master alarm: any program alarm (PROG lamp) or warning.
    const DskyState *d = agc_host_dsky();
    int alarm = d->prog_lamp || d->restart || prop_low();
    if (alarm && !prev_alarm) master_alarm = 1;
    prev_alarm = alarm;

    // Event timer: started by the crew; also auto-started at DPS ignition so
    // it shows time since PDI, as the crew set it up.
    // Counts mission time (not wall time), so it stays right under time warp.
    static int was_on;
    static double last_t = -1;
    double dts = last_t < 0 ? 0 : lm.t - last_t;
    if (dts < 0) dts = 0;
    last_t = lm.t;
    if (lm.dps_on && !was_on && event_timer == 0) event_run = 1;
    was_on = lm.dps_on;
    int ets = ST(ID_ET, 1);
    if (ets == -1) { event_timer = 0; event_run = 0; ST(ID_ET, 1) = 0; }
    // Panel 3 duplicate: RESET (spring-loaded) zeroes the timer, COUNT starts it.
    int er = ST(ID_ETRESET, 0);
    if (er == 1) { event_timer = 0; event_run = 0; ST(ID_ETRESET, 0) = 0; cp_dirty = 1; }
    if (er == -1) { event_run = 1; if (ets == 0) ST(ID_ET, 1) = 1; ST(ID_ETRESET, 0) = 0; cp_dirty = 1; }
    if (event_run && ST(ID_ET, 1) != 0) {
        if (ST(ID_ETCOUNT, 1) > 0) event_timer += dts;
        else if ((event_timer -= dts) <= 0) { event_timer = 0; event_run = 0; }   // counted down to zero
    }

    // Mission timer: follows mission time until the crew stops, resets or slews it.
    if (mission_timer < 0 || dts > 600 || lm.t < 1) mission_timer = lm.t;   // first frame, or a new scenario
    int mts = ST(ID_MT, 1);
    if (mts == -1) { mission_timer = 0; ST(ID_MT, 1) = 0; cp_dirty = 1; }
    else if (mts == 1) mission_timer += dts;
}

static void deda_key(int i)
{
    static const char kc[16] = { '+', '7', '8', '9', '-', '4', '5', '6', '0', '1', '2', '3', 'C', 'R', 'E', 'H' };
    char k = kc[i];
    deda_err = 0;
    if (k == 'C') { strcpy(deda_addr, "   "); strcpy(deda_data, "      "); deda_n = 0; deda_mode = 0; return; }
    if (deda_mode == 0) {
        if (k >= '0' && k <= '7' && deda_n < 3) { deda_addr[deda_n++] = k; return; }
        if (k == 'R' && deda_n == 3) {
            // The AGS is not simulated: it reads out zero.
            strcpy(deda_data, "+00000");
            return;
        }
        if (k == '+' || k == '-') { if (deda_n == 3) { deda_mode = 1; deda_n = 0; strcpy(deda_data, "      "); deda_data[0] = k; } else deda_err = 1; return; }
        deda_err = 1;
    } else {
        if (k >= '0' && k <= '9' && deda_n < 5) { deda_data[1 + deda_n++] = k; return; }
        if (k == 'E' && deda_n == 5) { deda_mode = 0; deda_n = 3; return; }
        deda_err = 1;
    }
}

static Rectangle dsky_native_rect(void)
{
    float scale = fminf(dsky_mm.width / DSKY_W, dsky_mm.height / DSKY_H);
    return (Rectangle){ dsky_mm.x + (dsky_mm.width - DSKY_W * scale) / 2, dsky_mm.y + (dsky_mm.height - DSKY_H * scale) / 2,
                        scale, scale };
}

static int dsky_key_from_mm(float u, float v)
{
    Rectangle n = dsky_native_rect();
    return dsky_key_at((Vector2){ (u - n.x) / n.width, (v - n.y) / n.height });
}

void cp_click(const Hot *h, float u, float v)
{
    cp_dirty = 1;
    if (h->kind == HOT_DEDA) sound_key();
    else if (h->kind != HOT_DSKY) sound_switch();   // DSKY keys click in dsky.c
    switch (h->kind) {
    case HOT_DSKY:
        dsky_mouse_press(dsky_key_from_mm(u, v));
        return;
    case HOT_DEDA:
        deda_key(h->idx);
        press_id = ID_DEDA; press_idx = h->idx; press_t = 0.14f;
        return;
    case HOT_CB: {
        int k = h->panel == P16;
        unsigned char *o = &cb_open[k][h->idx < CB_PER_PANEL ? h->idx : 0];
        *o = !*o;
        static char b[120];
        snprintf(b, sizeof b, "Circuit breaker %s: %s", h->id ? h->id : "(spare)", *o ? "OPEN" : "closed");
        crew_say(b);
        return;
    }
    case HOT_TOGGLE: {
        int *s = state_of(h->id, 0);
        int up = v < h->r.y + h->r.height / 2;
        if (h->id == ID_ROD) {
            // Spring-loaded: one click per flick, back to centre.
            lm_rod_click(up ? +1 : -1);   // +1 = slower descent (measured in agc-lab)
            rod_anim = up ? 0.25f : -0.25f;
            return;
        }
        int n = h->idx == 2 ? (up ? 1 : -1) : *s + (up ? 1 : -1);
        if (n > 1) n = 1;
        if (n < -1) n = -1;
        *s = n;
        if (h->id == ID_ENGARM) {
            lm_set_engine_arm(n == -1);
            crew_say(n == -1 ? "ENG ARM: DES" : n == 1 ? "ENG ARM: ASC (ascent engine not simulated)" : "ENG ARM: OFF");
        } else if (h->id == ID_PGNS) {
            lm_set_attitude_mode(n == 1 ? 2 : n == 0 ? 1 : 0);
            crew_say(n == 1 ? "PGNS mode control: AUTO" : n == 0 ? "PGNS mode control: ATT HOLD" : "PGNS mode control: OFF");
        } else if (h->id == ID_ET) {
            if (n == 1) event_run = 1;
            if (n == 0) event_run = 0;
        }
        return;
    }
    case HOT_KNOB: {
        int *s = state_of(h->id, 0);
        float cx = h->r.x + h->r.width / 2;
        int step = u >= cx ? 1 : -1;
        // Timer slew thumbwheels turn freely: each click moves the reading one unit.
        double *tm = h->id == ID_MTH || h->id == ID_MTM || h->id == ID_MTS ? &mission_timer :
                     h->id == ID_ETM || h->id == ID_ETS ? &event_timer : NULL;
        if (tm) {
            step = v < h->r.y + h->r.height / 2 ? 1 : -1;   // roll the wheel up or down
            double unit = h->id == ID_MTH ? 3600 : h->id == ID_MTM || h->id == ID_ETM ? 60 : 1;
            *tm = fmax(0, *tm + step * unit);
            *s = ((*s + step) % 10 + 10) % 10;
            return;
        }
        int n = *s + step;
        if (n < 0) n = 0;
        if (n > h->idx - 1) n = h->idx - 1;
        *s = n;
        return;
    }
    case HOT_BUTTON:
        press_id = h->id; press_idx = 0; press_t = 0.16f;
        if (h->id == ID_STOP5 || h->id == ID_STOP6) {
            lm_engine_stop();
            crew_say("ENGINE STOP");
        } else if (h->id == ID_MA1 || h->id == ID_MA2) {
            master_alarm = 0;
        } else if (h->id == ID_PLUSX5 || h->id == ID_PLUSX6) {
            lm_set_thc(0, +1);
            plusx_timer = 2.0f;
            crew_say("+X translation: 4 jets for ullage");
        } else if (h->id == ID_ABORT || h->id == ID_ABSTAGE) {
            crew_say("Abort is not simulated (it would hand the LGC to P70/P71)");
        } else if (h->id == ID_START5) {
            crew_say("Manual DPS start is not modelled: the LGC commands ignition");
        }
        return;
    }
}

static const char *pos_name(const Hot *h)
{
    (void)h;
    return "";
}

const char *cp_tooltip(const Hot *h, float u, float v)
{
    static char b[160];
    (void)u;
    (void)v;
    (void)pos_name;
    static const char *const works[] = {
        ID_ENGARM, ID_PGNS, ID_ROD, ID_STOP5, ID_STOP6, ID_MA1, ID_MA2, ID_FLOOD, ID_INTEG, ID_NUM, ID_LTGTEST, ID_XSCALE,
        ID_RATESCALE, ID_MODESEL, ID_ET, ID_PLUSX5, ID_PLUSX6, ID_FLOODSEL, ID_SIDEPNL, ID_ANUNBRT, ID_DSKYLTG, ID_MT, ID_MTH,
        ID_MTM, ID_MTS, ID_ETM, ID_ETS, ID_ETCOUNT, ID_ETRESET, ID_PRPMON, ID_TEMPMON, ID_HTRMON, ID_PWRMON, ID_SIGMON,
        ID_SUITFAN, ID_CABFAN,
    };
    int functional = 0;
    for (size_t i = 0; i < sizeof works / sizeof works[0]; i++) functional |= h->id == works[i];
    switch (h->kind) {
    case HOT_DSKY: {
        int k = dsky_key_from_mm(u, v);
        snprintf(b, sizeof b, k >= 0 ? "DSKY  %s" : "DSKY - LGC display and keyboard", dsky_key_name(k));
        return b;
    }
    case HOT_DEDA: return "AGS DEDA (abort guidance not simulated)";
    case HOT_CB: {
        static const char *const wired[] = { "FLOOD", "INTGL", "SIDE PNLS", "MISSION TIMER", "EVENT TIMER", "PRPLNT QTY",
                                             "SUIT FAN 1", "SUIT FAN 2", "CABIN FAN CONT" };
        int w = 0;
        for (size_t i = 0; h->id && i < sizeof wired / sizeof wired[0]; i++) w |= !strcmp(h->id, wired[i]);
        int open = cb_open[h->panel == P16][h->idx < CB_PER_PANEL ? h->idx : 0];
        snprintf(b, sizeof b, "Circuit breaker  %s  (%s)   click to %s%s", h->id ? h->id : "spare", open ? "OPEN" : "closed",
                 open ? "push in" : "pull", w ? "" : "   (its circuit is not modelled)");
        return b;
    }
    case HOT_TOGGLE:
        if (h->id == ID_ROD) return "RATE OF DESCENT  click up: -1 ft/s  down: +1 ft/s  (P66)";
        snprintf(b, sizeof b, "%s  %s%s", h->id, *state_of(h->id, 0) > 0 ? "UP" : *state_of(h->id, 0) < 0 ? "DOWN" : "CENTRE",
                 functional ? "" : "   (not modelled)");
        return b;
    case HOT_KNOB:
        if (h->id == ID_MTH || h->id == ID_MTM || h->id == ID_MTS || h->id == ID_ETM || h->id == ID_ETS) {
            snprintf(b, sizeof b, "%s  timer slew: click the upper half to add one, the lower half to take one off", h->id);
            return b;
        }
        snprintf(b, sizeof b, "%s  position %d   click left/right half to turn%s", h->id, *state_of(h->id, 0),
                 functional ? "" : "   (not modelled)");
        return b;
    case HOT_BUTTON:
        snprintf(b, sizeof b, "%s%s", h->id, functional ? "" : "   (not modelled)");
        return b;
    }
    return "";
}
