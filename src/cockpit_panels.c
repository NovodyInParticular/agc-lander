// LM cockpit panel artwork and switch logic.
//
// Every panel is drawn in millimetres into a texture atlas: pass 0 is the
// painted surface (albedo), pass 1 is what glows (lamps, EL numerics and the
// integral panel lighting). Switches register themselves for the 3D levers
// and for mouse picking while they are drawn.
//
// The layout and nomenclature follow the Apollo 11 LM (LM-5) as closely as
// a hand-drawn panel set can; switches that the simulation does not model
// still move and say so in their tooltip.

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "raylib.h"
#include "rlgl.h"

#include "agc_host.h"
#include "cockpit_int.h"
#include "crew.h"
#include "dsky.h"
#include "hud.h"
#include "lm.h"

Lever cp_lev[CP_MAXLEV];
int cp_nlev;
Hot cp_hot[CP_MAXHOT];
int cp_nhot;

Texture2D hud_fdai_texture(void);

static int pass;          // 0 albedo, 1 emission
static int cur_panel;
static Camera2D cur_cam;

#define FT 0.3048

// --- Colours -------------------------------------------------------------------

static const Color C_PANEL = { 74, 78, 82, 255 };       // panel grey
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

static float rod_anim, plusx_timer;
static double event_timer = 0;
static int event_run = 0;
static int master_alarm, prev_alarm;
static char deda_addr[4] = "   ", deda_data[7] = "      ";
static int deda_n, deda_mode, deda_err;   // mode 0: address entry, 1: data entry

static int lamp_test(void) { return ST(ID_LTGTEST, -1) > 0; }
float cp_flood_level(void) { return ST(ID_FLOOD, 5) / 10.0f; }
float cp_integral_level(void) { return ST(ID_INTEG, 6) / 10.0f; }
float cp_numerics_level(void) { return 0.35f + ST(ID_NUM, 8) / 10.0f * 0.65f; }

// --- Drawing helpers (millimetres) ------------------------------------------------

static Color el_col(float k)
{
    float g = cp_integral_level() * k;
    return (Color){ (unsigned char)(140 * g), (unsigned char)(235 * g), (unsigned char)(185 * g), 255 };
}

static void txt(const char *s, float x, float y, float size, int align)
{
    Vector2 m = MeasureTextEx(font_bold, s, size, size * 0.06f);
    float px = align == 0 ? x : align == 1 ? x - m.x / 2 : x - m.x;
    Color c = pass ? el_col(0.55f) : C_WHITE;
    DrawTextEx(font_bold, s, (Vector2){ px, y }, size, size * 0.06f, c);
}
#define TXT_C(s, x, y, sz) txt(s, x, y, sz, 1)


static void rect(float x, float y, float w, float h, Color c)
{
    if (pass == 0) DrawRectangleRec((Rectangle){ x, y, w, h }, c);
}

// White group outline with the title breaking the top line.
static void group(float x, float y, float w, float h, const char *title)
{
    Color c = pass ? el_col(0.35f) : C_WHITE;
    float t = 0.6f;
    float tw = title ? MeasureTextEx(font_bold, title, 3.4f, 0.2f).x + 4 : 0;
    float cx = x + w / 2;
    DrawRectangleRec((Rectangle){ x, y, cx - tw / 2 - x, t }, c);
    DrawRectangleRec((Rectangle){ cx + tw / 2, y, x + w - cx - tw / 2, t }, c);
    DrawRectangleRec((Rectangle){ x, y + h - t, w, t }, c);
    DrawRectangleRec((Rectangle){ x, y, t, h }, c);
    DrawRectangleRec((Rectangle){ x + w - t, y, t, h }, c);
    if (title) TXT_C(title, cx, y - 1.9f, 3.4f);
}

static void add_hot(int kind, Rectangle r, const char *id, int idx)
{
    if (pass != 0 || cp_nhot >= CP_MAXHOT) return;
    cp_hot[cp_nhot++] = (Hot){ cur_panel, kind, r, id, idx };
}

static void add_lever(int kind, float x, float y, float val, float size, int guard, Color c)
{
    if (pass != 0 || cp_nlev >= CP_MAXLEV) return;
    cp_lev[cp_nlev++] = (Lever){ cur_panel, kind, x, y, val, size, guard, c.r, c.g, c.b };
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
    if (pass == 0) {
        DrawPoly((Vector2){ x, y }, 6, 4.2f, 0, C_NUT);
        DrawPolyLines((Vector2){ x, y }, 6, 4.2f, 0, (Color){ 90, 92, 96, 255 });
        DrawCircleV((Vector2){ x, y }, 2.4f, (Color){ 60, 62, 66, 255 });
        if (guard) {
            DrawRectangleRec((Rectangle){ x - 6.5f, y - 7, 1.4f, 14 }, (Color){ 170, 170, 172, 255 });
            DrawRectangleRec((Rectangle){ x + 5.1f, y - 7, 1.4f, 14 }, (Color){ 170, 170, 172, 255 });
        }
    }
    add_lever(LEV_TOGGLE, x, y, (float)s, 16, guard, (Color){ 196, 198, 202, 255 });
    add_hot(HOT_TOGGLE, (Rectangle){ x - 7, y - 11, 14, 22 }, id, mid ? 3 : 2);
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
        if (pass == 0)
            DrawLineEx((Vector2){ x + sx * 8.5f, y + sy * 8.5f }, (Vector2){ x + sx * 10.5f, y + sy * 10.5f }, 0.6f, C_WHITE);
        if (labels && labels[i]) {
            int al = sx > 0.3f ? 0 : sx < -0.3f ? 2 : 1;
            txt(labels[i], x + sx * 12.0f, y + sy * 12.0f - 1.4f, 2.6f, al);
        }
    }
    float a = (a0 + (n > 1 ? arc_deg * s / (n - 1) : 0)) * DEG2RAD;
    if (pass == 0) DrawCircleV((Vector2){ x, y }, 7.5f, (Color){ 30, 30, 32, 255 });
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
    if (pass == 0)
        for (int i = 0; i < 6; i++)
            DrawRectangleRec((Rectangle){ x - 2.8f, y - 6 + i * 2.3f + (s % 2) * 1.1f, 5.6f, 0.6f }, (Color){ 70, 70, 74, 255 });
    TXT_C("MAX", x + 8, y - 7, 2.2f);
    add_hot(HOT_KNOB, (Rectangle){ x - 4, y - 8, 8, 16 }, id, 10);
}

// Annunciator lamp (part of a caution/warning matrix or stand-alone).
static void lamp(float x, float y, float w, float h, const char *label, Color on_col, int on)
{
    on = on || lamp_test();
    if (pass == 0) {
        DrawRectangleRec((Rectangle){ x, y, w, h }, on ? (Color){ (unsigned char)(on_col.r * 0.8f), (unsigned char)(on_col.g * 0.8f), (unsigned char)(on_col.b * 0.8f), 255 } : C_LAMP_OFF);
        DrawRectangleLinesEx((Rectangle){ x, y, w, h }, 0.5f, (Color){ 20, 20, 20, 255 });
        Vector2 m = MeasureTextEx(font_bold, label, 2.9f, 0.15f);
        DrawTextEx(font_bold, label, (Vector2){ x + (w - m.x) / 2, y + (h - 2.9f) / 2 }, 2.9f, 0.15f,
                   on ? (Color){ 30, 20, 10, 255 } : (Color){ 30, 30, 28, 255 });
    } else if (on) {
        float k = cp_numerics_level();
        DrawRectangleRec((Rectangle){ x, y, w, h },
                         (Color){ (unsigned char)(on_col.r * k), (unsigned char)(on_col.g * k), (unsigned char)(on_col.b * k), 255 });
        Vector2 m = MeasureTextEx(font_bold, label, 2.9f, 0.15f);
        DrawTextEx(font_bold, label, (Vector2){ x + (w - m.x) / 2, y + (h - 2.9f) / 2 }, 2.9f, 0.15f, (Color){ 40, 25, 10, 255 });
    }
}

// Push button (optionally lighted).
static void pushbtn(float x, float y, float w, float h, const char *id, const char *label, Color lit_col, int lit, int guard)
{
    if (pass == 0) {
        DrawRectangleRounded((Rectangle){ x - 1.2f, y - 1.2f, w + 2.4f, h + 2.4f }, 0.2f, 4, C_BEZEL);
        Color face = lit || lamp_test() ? lit_col : (Color){ 58, 56, 54, 255 };
        if (!lit && !lamp_test() && lit_col.r > 200 && lit_col.g < 120) face = (Color){ 96, 40, 36, 255 };
        DrawRectangleRounded((Rectangle){ x, y, w, h }, 0.15f, 4, face);
        if (guard) {
            DrawRectangleLinesEx((Rectangle){ x - 3, y - 3, w + 6, h + 6 }, 1.2f, (Color){ 180, 180, 184, 255 });
            for (int i = 0; i < 6; i++)
                DrawLineEx((Vector2){ x - 3 + i * (w + 6) / 6, y - 3 }, (Vector2){ x - 3 + i * (w + 6) / 6 + 4, y - 6 }, 0.6f,
                           (Color){ 200, 200, 204, 255 });
        }
    }
    Vector2 m = MeasureTextEx(font_bold, label, 3.4f, 0.15f);
    float ty = y + (h - 3.4f * (strchr(label, '\n') ? 2.0f : 1.0f)) / 2;
    if (pass == 0)
        DrawTextEx(font_bold, label, (Vector2){ x + (w - m.x) / 2, ty }, 3.4f, 0.15f, (Color){ 235, 232, 225, 255 });
    else if (lit || lamp_test()) {
        float k = cp_numerics_level();
        DrawRectangleRounded((Rectangle){ x, y, w, h }, 0.15f, 4,
                             (Color){ (unsigned char)(lit_col.r * k), (unsigned char)(lit_col.g * k), (unsigned char)(lit_col.b * k), 255 });
        DrawTextEx(font_bold, label, (Vector2){ x + (w - m.x) / 2, ty }, 3.4f, 0.15f, (Color){ 255, 250, 240, 255 });
    }
    add_hot(HOT_BUTTON, (Rectangle){ x, y, w, h }, id, 0);
}

// Circuit breaker with its label above.
static void breaker(float x, float y, const char *label, int popped)
{
    if (label) TXT_C(label, x, y - 8.2f, 2.3f);
    if (pass == 0) {
        DrawCircleV((Vector2){ x, y }, 4.6f, (Color){ 14, 14, 15, 255 });
        if (popped) DrawCircleV((Vector2){ x, y }, 3.2f, (Color){ 230, 230, 230, 255 });
        DrawCircleV((Vector2){ x, y }, 2.6f, (Color){ 26, 26, 28, 255 });
        DrawCircleV((Vector2){ x - 0.8f, y - 0.8f }, 0.8f, (Color){ 70, 70, 74, 255 });
    }
    add_hot(HOT_CB, (Rectangle){ x - 4.6f, y - 4.6f, 9.2f, 9.2f }, label, 0);
}

// Talkback flag indicator: 0 = barber pole, 1 = grey.
static void talkback(float x, float y, int grey)
{
    if (pass != 0) return;
    DrawCircleV((Vector2){ x, y }, 5.2f, C_BEZEL);
    DrawCircleV((Vector2){ x, y }, 4.2f, (Color){ 10, 10, 10, 255 });
    if (grey) {
        DrawRectangleRec((Rectangle){ x - 3.4f, y - 2.4f, 6.8f, 4.8f }, (Color){ 150, 150, 148, 255 });
    } else {
        for (int i = -3; i <= 3; i++) {
            Color c = (i & 1) ? (Color){ 20, 20, 20, 255 } : (Color){ 225, 225, 220, 255 };
            DrawLineEx((Vector2){ x - 3.4f + i * 1.2f, y + 2.4f }, (Vector2){ x - 3.4f + i * 1.2f + 4.8f, y - 2.4f }, 1.2f, c);
        }
        DrawRectangleLinesEx((Rectangle){ x - 4.6f, y - 3.2f, 9.2f, 6.4f }, 1.4f, (Color){ 10, 10, 10, 255 });
    }
}

// Seven-segment EL numerics (DSKY style). str: digits, ' ' blank, '+', '-', ':'.
static void el_digits(float x, float y, float h, const char *str)
{
    float w = h * 0.58f, t = w * 0.17f, gap = h * 0.22f;
    static const unsigned char seg[10] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };
    float k = cp_numerics_level();
    Color on = pass ? (Color){ (unsigned char)(110 * k), (unsigned char)(255 * k), (unsigned char)(140 * k), 255 }
                    : (Color){ 150, 230, 160, 255 };
    Color off = pass ? (Color){ 0, 0, 0, 255 } : (Color){ 30, 40, 32, 255 };
    rect(x - 1.5f, y - 1.5f, (float)strlen(str) * (w + gap) - gap + 3, h + 3, (Color){ 12, 16, 13, 255 });
    for (const char *p = str; *p; p++, x += w + gap) {
        if (*p == ':') {
            if (pass == 0 || 1) {
                DrawRectangleRec((Rectangle){ x + w / 2 - t / 2, y + h * 0.25f, t, t }, on);
                DrawRectangleRec((Rectangle){ x + w / 2 - t / 2, y + h * 0.7f, t, t }, on);
            }
            continue;
        }
        unsigned char s = 0;
        if (*p >= '0' && *p <= '9') s = seg[*p - '0'];
        if (lamp_test() && *p != ' ') s = 0x7F;
        float hh = h / 2;
        Rectangle r[7] = {
            { x + t, y, w - 2 * t, t }, { x + w - t, y + t * 0.5f, t, hh - t }, { x + w - t, y + hh + t * 0.5f, t, hh - t },
            { x + t, y + h - t, w - 2 * t, t }, { x, y + hh + t * 0.5f, t, hh - t }, { x, y + t * 0.5f, t, hh - t },
            { x + t, y + hh - t / 2, w - 2 * t, t },
        };
        if (*p == '+' || *p == '-') {
            DrawRectangleRec((Rectangle){ x, y + hh - t / 2, w, t }, on);
            if (*p == '+') DrawRectangleRec((Rectangle){ x + w / 2 - t / 2, y + h * 0.2f, t, h * 0.6f }, on);
            continue;
        }
        for (int i = 0; i < 7; i++) DrawRectangleRec(r[i], (s >> i) & 1 ? on : off);
    }
}

// Vertical edgewise meter: value 0..1 against a scale.
static void meter(float x, float y, float w, float h, const char *title, float v, const char *top, const char *bot)
{
    if (title) TXT_C(title, x + w / 2, y - 5.5f, 2.8f);
    rect(x - 1, y - 1, w + 2, h + 2, C_BEZEL);
    rect(x, y, w, h, C_FACE);
    if (pass == 0) {
        for (int i = 0; i <= 10; i++) {
            float yy = y + 2 + (h - 4) * i / 10.0f;
            DrawLineEx((Vector2){ x + 1, yy }, (Vector2){ x + (i % 5 ? 3.5f : 5.5f), yy }, 0.4f, C_WHITE);
        }
        if (top) DrawTextEx(font_bold, top, (Vector2){ x + 6.5f, y + 0.5f }, 2.4f, 0.1f, C_WHITE);
        if (bot) DrawTextEx(font_bold, bot, (Vector2){ x + 6.5f, y + h - 3.2f }, 2.4f, 0.1f, C_WHITE);
        v = v < 0 ? 0 : v > 1 ? 1 : v;
        float yy = y + 2 + (h - 4) * (1 - v);
        DrawTriangle((Vector2){ x + w - 0.5f, yy - 2 }, (Vector2){ x + 2, yy }, (Vector2){ x + w - 0.5f, yy + 2 },
                     (Color){ 240, 240, 236, 255 });
    }
}

// Round dial with a needle: v 0..1 over 270 degrees.
static void dial(float x, float y, float r, const char *title, float v, const char *lo, const char *hi)
{
    if (title) TXT_C(title, x, y - r - 6.5f, 2.8f);
    if (pass != 0) return;
    DrawCircleV((Vector2){ x, y }, r + 1.5f, C_BEZEL);
    DrawCircleV((Vector2){ x, y }, r, C_FACE);
    for (int i = 0; i <= 10; i++) {
        float a = (-135 + 27 * i) * DEG2RAD;
        float l = i % 5 ? 0.82f : 0.72f;
        DrawLineEx((Vector2){ x + sinf(a) * r * l, y - cosf(a) * r * l }, (Vector2){ x + sinf(a) * r * 0.95f, y - cosf(a) * r * 0.95f },
                   0.4f, C_WHITE);
    }
    if (lo) DrawTextEx(font_bold, lo, (Vector2){ x - r * 0.75f, y + r * 0.45f }, 2.3f, 0.1f, C_WHITE);
    if (hi) DrawTextEx(font_bold, hi, (Vector2){ x + r * 0.30f, y + r * 0.45f }, 2.3f, 0.1f, C_WHITE);
    v = v < 0 ? 0 : v > 1 ? 1 : v;
    float a = (-135 + 270 * v) * DEG2RAD;
    DrawLineEx((Vector2){ x, y }, (Vector2){ x + sinf(a) * r * 0.86f, y - cosf(a) * r * 0.86f }, 0.8f, (Color){ 245, 245, 240, 255 });
    DrawCircleV((Vector2){ x, y }, 1.4f, (Color){ 60, 60, 60, 255 });
}

static float clampf(float v, float a, float b) { return v < a ? a : v > b ? b : v; }

// --- Instruments ----------------------------------------------------------------

static void fdai(float cx, float cy, float r)
{
    if (pass == 0) {
        DrawCircleV((Vector2){ cx, cy }, r + 16, C_BEZEL);
        DrawRectangleRounded((Rectangle){ cx - r - 18, cy - r - 18, 2 * r + 36, 2 * r + 36 }, 0.12f, 6, C_BEZEL);
        DrawCircleV((Vector2){ cx, cy }, r + 3, (Color){ 6, 6, 7, 255 });
        Texture2D t = hud_fdai_texture();
        float s = r * 2.0f * 1.22f;
        DrawTexturePro(t, (Rectangle){ 0, 0, (float)t.width, -(float)t.height }, (Rectangle){ cx - s / 2, cy - s / 2, s, s },
                       (Vector2){ 0, 0 }, 0, WHITE);
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
        // Rate scales: top roll, right pitch, bottom yaw.
        float full = ST(ID_RATESCALE, -1) > 0 ? 25.0f : 5.0f;
        float rr = clampf((float)(lm.w.z * 57.2958 / full), -1, 1), rp = clampf((float)(lm.w.y * 57.2958 / full), -1, 1),
              ry = clampf((float)(lm.w.x * 57.2958 / full), -1, 1);
        Rectangle top = { cx - r * 0.8f, cy - r - 15, r * 1.6f, 6 }, right = { cx + r + 9, cy - r * 0.8f, 6, r * 1.6f },
                  bot = { cx - r * 0.8f, cy + r + 9, r * 1.6f, 6 };
        DrawRectangleRec(top, C_FACE); DrawRectangleRec(right, C_FACE); DrawRectangleRec(bot, C_FACE);
        for (int i = 0; i <= 4; i++) {
            DrawLineEx((Vector2){ top.x + top.width * i / 4, top.y }, (Vector2){ top.x + top.width * i / 4, top.y + 2 }, 0.4f, C_WHITE);
            DrawLineEx((Vector2){ bot.x + bot.width * i / 4, bot.y + 6 }, (Vector2){ bot.x + bot.width * i / 4, bot.y + 4 }, 0.4f, C_WHITE);
            DrawLineEx((Vector2){ right.x + 6, right.y + right.height * i / 4 }, (Vector2){ right.x + 4, right.y + right.height * i / 4 }, 0.4f, C_WHITE);
        }
        Color nd = { 240, 240, 236, 255 };
        DrawTriangle((Vector2){ cx + rr * r * 0.8f, top.y + 5.5f }, (Vector2){ cx + rr * r * 0.8f + 2, top.y + 1 }, (Vector2){ cx + rr * r * 0.8f - 2, top.y + 1 }, nd);
        DrawTriangle((Vector2){ right.x + 0.5f, cy - rp * r * 0.8f }, (Vector2){ right.x + 5, cy - rp * r * 0.8f + 2 }, (Vector2){ right.x + 5, cy - rp * r * 0.8f - 2 }, nd);
        DrawTriangle((Vector2){ cx + ry * r * 0.8f, bot.y + 0.5f }, (Vector2){ cx + ry * r * 0.8f - 2, bot.y + 5 }, (Vector2){ cx + ry * r * 0.8f + 2, bot.y + 5 }, nd);
        // Flags: OFF flag when the attitude reference is lost (NO ATT).
        if (agc_host_dsky()->no_att) {
            DrawRectangleRec((Rectangle){ cx - r * 0.95f, cy - r * 0.35f, 12, 9 }, (Color){ 220, 60, 40, 255 });
            DrawTextEx(font_bold, "OFF", (Vector2){ cx - r * 0.95f + 1.5f, cy - r * 0.35f + 2.8f }, 3.2f, 0.1f, WHITE);
        }
    }
    TXT_C("ROLL", cx, cy - r - 22, 2.6f);
    txt("PITCH", cx + r + 17, cy - 1.2f, 2.6f, 0);
    TXT_C("YAW", cx, cy + r + 17, 2.6f);
}

static void x_pointer(float x, float y, float s, int lmp)
{
    TXT_C("X-POINTER", x + s / 2, y - 6, 3.0f);
    rect(x - 2, y - 2, s + 4, s + 4, C_BEZEL);
    rect(x, y, s, s, C_FACE);
    if (pass != 0) return;
    float cx = x + s / 2, cy = y + s / 2;
    for (int i = -4; i <= 4; i++) {
        float l = i % 2 ? 1.5f : 3;
        DrawLineEx((Vector2){ cx + i * s / 10, cy - l }, (Vector2){ cx + i * s / 10, cy + l }, 0.4f, C_WHITE);
        DrawLineEx((Vector2){ cx - l, cy + i * s / 10 }, (Vector2){ cx + l, cy + i * s / 10 }, 0.4f, C_WHITE);
    }
    DrawTextEx(font_bold, "FWD", (Vector2){ cx + 2, y + 2 }, 2.4f, 0.1f, C_WHITE);
    DrawTextEx(font_bold, "LAT", (Vector2){ x + s - 9, cy + 2 }, 2.4f, 0.1f, C_WHITE);
    V3 vb = lm_inertial_to_body(lm_surface_velocity());
    double full = ST(ID_XSCALE, 1) > 0 ? 20.0 : 200.0;   // ft/s full scale, LO MULT / HI MULT
    float fx = clampf((float)(vb.y / FT / full), -1, 1), fz = clampf((float)(vb.z / FT / full), -1, 1);
    Color nc = { 245, 245, 240, 255 };
    DrawLineEx((Vector2){ cx + fx * s * 0.42f, y + 3 }, (Vector2){ cx + fx * s * 0.42f, y + s - 3 }, 1.1f, nc);
    DrawLineEx((Vector2){ x + 3, cy - fz * s * 0.42f }, (Vector2){ x + s - 3, cy - fz * s * 0.42f }, 1.1f, nc);
    (void)lmp;
}

// Altitude / altitude-rate tape meter.
static void alt_tapes(float x, float y, float w, float h)
{
    TXT_C("ALT", x + w * 0.28f, y - 10, 3.0f);
    TXT_C("ALT RT", x + w * 0.76f, y - 10, 3.0f);
    TXT_C("FT", x + w * 0.28f, y - 5.5f, 2.4f);
    TXT_C("FT/SEC", x + w * 0.76f, y - 5.5f, 2.4f);
    rect(x - 2, y - 2, w + 4, h + 4, C_BEZEL);
    rect(x, y, w, h, C_FACE);
    if (pass != 0) return;
    // Source: landing radar or PGNS (MODE SEL), as on the real panel.
    int src = ST(ID_MODESEL, 1);   // +1 LDG RDR, 0 PGNS, -1 AGS
    double alt = lm.altitude / FT, rate = lm.hdot / FT;
    int nodata = 0;
    if (src > 0) {
        if (lm.lr_range_good) alt = lm.lr_alt / FT;
        else nodata = 1;
    }
    float tw = w * 0.5f - 3;
    Rectangle ta = { x + 2, y + 2, tw, h - 4 }, tr = { x + w * 0.5f + 1, y + 2, tw, h - 4 };
    // Altitude tape: white tape, black numerals, scale changes like the real
    // one (20 ft per mark below 200 ft ... 1000 ft per mark above 10000 ft).
    DrawRectangleRec(ta, (Color){ 215, 212, 200, 255 });
    double step = alt < 200 ? 20 : alt < 2000 ? 100 : alt < 10000 ? 500 : 1000;
    double ppu = ta.height / (step * 8);
    for (double v = floor((alt - step * 5) / step) * step; v < alt + step * 5; v += step) {
        float yy = (float)(ta.y + ta.height / 2 - (v - alt) * ppu);
        if (yy < ta.y + 1 || yy > ta.y + ta.height - 3) continue;
        DrawLineEx((Vector2){ ta.x, yy }, (Vector2){ ta.x + 4, yy }, 0.5f, (Color){ 10, 10, 10, 255 });
        char s[16];
        snprintf(s, sizeof s, "%.0f", v);
        if (v >= 0 && yy > ta.y + 3) DrawTextEx(font_bold, s, (Vector2){ ta.x + 5, yy - 1.4f }, 2.8f, 0.1f, (Color){ 10, 10, 10, 255 });
    }
    DrawRectangleRec(tr, (Color){ 20, 20, 22, 255 });
    double rstep = fabs(rate) < 50 ? 5 : 50;
    double rppu = tr.height / (rstep * 8);
    for (double v = floor((rate - rstep * 5) / rstep) * rstep; v < rate + rstep * 5; v += rstep) {
        float yy = (float)(tr.y + tr.height / 2 - (v - rate) * rppu);
        if (yy < tr.y + 1 || yy > tr.y + tr.height - 3) continue;
        DrawLineEx((Vector2){ tr.x, yy }, (Vector2){ tr.x + 4, yy }, 0.5f, C_WHITE);
        char s[16];
        snprintf(s, sizeof s, "%+.0f", v);
        if (yy > tr.y + 3) DrawTextEx(font_bold, s, (Vector2){ tr.x + 5, yy - 1.4f }, 2.8f, 0.1f, C_WHITE);
    }
    // Index lines.
    DrawRectangleRec((Rectangle){ x, y + h / 2 - 0.5f, w, 1.0f }, (Color){ 255, 150, 30, 255 });
    if (nodata) {
        DrawRectangleRec((Rectangle){ ta.x + 2, ta.y + 6, tw - 4, 9 }, (Color){ 200, 50, 40, 255 });
        DrawTextEx(font_bold, "NO TRACK", (Vector2){ ta.x + 3.5f, ta.y + 9 }, 2.8f, 0.1f, WHITE);
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
    el_digits(110, 355, 12, b);
    snprintf(b, sizeof b, "%02d", q > 0 ? q - (q > 1) : 0);
    TXT_C("OXID", 182, 349, 2.6f);
    el_digits(172, 355, 12, b);
    group(94, 396, 116, 36, "EVENT TIMER");
    int et = (int)event_timer;
    snprintf(b, sizeof b, "%02d:%02d", (et / 60) % 100, et % 60);
    el_digits(112, 407, 13, b);

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
    float he = 0.55f + 0.02f * sinf((float)GetTime() * 0.1f);
    meter(282, 482, 11, 58, "HE", he, "4", "0");
    meter(312, 482, 11, 58, "FUEL T", 0.52f, "100", "20");
    meter(342, 482, 11, 58, "OXID T", 0.50f, "100", "20");
    meter(372, 482, 11, 58, "PRESS", lm.dps_on ? 0.62f : 0.55f, "300", "0");
    static const char *const mon[4] = { "HE", "DES 1", "DES 2", "ASC" };
    rotary(425, 516, "PRPLNT MON", "TEMP/PRESS", mon, 4, 1, 120);

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
    int t = (int)lm.t;
    snprintf(b, sizeof b, "%03d:%02d:%02d", (t / 3600) % 1000, (t / 60) % 60, t % 60);
    el_digits(20, 90, 14, b);

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
    toggle(254, 500, "SUIT FAN", "SUIT FAN", "1", NULL, "2", 1, 0);
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
    rotary(48, 66, "TEMP MON", "TEMP MONITOR", tm, 6, 1, 200);
    dial(150, 66, 20, "TEMP", 0.45f, "-100", "+200");
    float sig = lm.lr_range_good ? 0.72f + 0.04f * sinf((float)GetTime() * 3) : 0.05f;
    dial(48, 160, 20, "SIGNAL STRENGTH", sig, "0", "5");
    static const char *const ss[6] = { "ALT XMTR", "VEL XMTR", "RNDZ", "S-BD", "VHF", "CDR" };
    rotary(150, 160, "SIG STR", "SIG STR MON", ss, 6, 0, 200);
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
    toggle(760, 226, "EVENT TIMER RESET", "EVT TMR", "RESET", "", "COUNT", 0, 0);

    group(818, 18, 214, 252, "HEATERS / INST");
    static const char *const hm[7] = { "RNDZ", "LDG", "S-BD", "QUAD 1", "QUAD 2", "QUAD 3", "QUAD 4" };
    rotary(870, 70, "HEATER MON", "HEATER MON", hm, 7, 1, 220);
    dial(970, 70, 20, "HTR TEMP", 0.5f, "-200", "+200");
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

static void panel4(void)
{
    TXT_C("LGC DISPLAY AND KEYBOARD", 220, 6, 3.0f);
    rect(dsky_mm.x - 3, dsky_mm.y - 3, dsky_mm.width + 6, dsky_mm.height + 6, (Color){ 30, 30, 32, 255 });
    // DSKY drawn natively into atlas pixels.
    EndMode2D();
    Rectangle px = { cur_cam.offset.x + dsky_mm.x * cur_cam.zoom, cur_cam.offset.y + dsky_mm.y * cur_cam.zoom,
                     dsky_mm.width * cur_cam.zoom, dsky_mm.height * cur_cam.zoom };
    dsky_set_emission(pass, cp_numerics_level());
    dsky_draw(px);
    dsky_set_emission(0, 1);
    BeginMode2D(cur_cam);
    add_hot(HOT_DSKY, dsky_mm, "DSKY", 0);
    // Side switches: X-pointer scale, attitude monitor... and the PGNS lighting.
    toggle(48, 80, "DSKY LTG", "DSKY", "BRT", NULL, "DIM", 1, 0);
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
    toggle(40, 64, "MISSION TIMER", "MSN TIMER", "START", "STOP", "RESET", 1, 0);
    thumbwheel(90, 64, "MT HOURS", "HRS");
    thumbwheel(118, 64, "MT MIN", "MIN");
    thumbwheel(146, 64, "MT SEC", "SEC");
    toggle(196, 64, ID_ET, "EVENT TIMER", "START", "STOP", "RESET", 1, 0);
    thumbwheel(246, 64, "ET MIN", "MIN");
    thumbwheel(274, 64, "ET SEC", "SEC");
    toggle(322, 64, "ET COUNT", "COUNT", "UP", NULL, "DOWN", 1, 0);

    group(8, 124, 166, 150, "LIGHTING");
    static const char *const fl[11] = { "OFF", "", "", "", "", "", "", "", "", "", "BRT" };
    rotary(48, 170, ID_FLOOD, "FLOOD", fl, 11, 5, 270);
    toggle(120, 170, "FLOOD SEL", "FLOOD", "ALL", NULL, "POST", 1, 0);
    toggle(48, 244, "ANUN/NUM BRT", "ANUN/NUM", "BRT", NULL, "DIM", 1, 0);
    toggle(120, 244, "SIDE PNLS", "SIDE PNLS", "ON", NULL, "OFF", 1, 0);

    group(184, 124, 168, 150, "ENGINE");
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
        if (pass == 0) {
            DrawRectangleRounded((Rectangle){ x, y, 38, 32 }, 0.15f, 4, (Color){ 32, 33, 35, 255 });
            DrawRectangleRoundedLinesEx((Rectangle){ x, y, 38, 32 }, 0.15f, 4, 0.6f, (Color){ 12, 12, 12, 255 });
        }
        int two = strchr(keys[i], '\n') != NULL;
        float sz = strlen(keys[i]) > 2 ? 3.6f : 6.0f;
        Vector2 m = MeasureTextEx(font_bold, keys[i], sz, 0.1f);
        if (pass == 0)
            DrawTextEx(font_bold, keys[i], (Vector2){ x + (38 - m.x) / 2, y + (32 - sz * (two ? 2 : 1)) / 2 }, sz, 0.1f, C_WHITE);
        add_hot(HOT_DEDA, (Rectangle){ x, y, 38, 32 }, ID_DEDA, i);
    }
    group(248, 18, 104, 256, "ENGINE");
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
    dial(70, 78, 30, "DC VOLTS", 0.66f + jitter, "20", "40");
    float amps = 0.45f + (float)(lm.thrust / 46978.0) * 0.08f + jitter;
    dial(160, 78, 30, "DC AMPS", amps, "0", "120");
    static const char *const mon[9] = { "CDR BUS", "LMP BUS", "BAT 1", "BAT 2", "BAT 3", "BAT 4", "BAT 5", "BAT 6", "XLUNAR" };
    rotary(270, 78, "PWR MON", "POWER/TEMP MON", mon, 9, 0, 260);
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
    toggle(490, 320, "CABIN FAN", "CABIN FAN", "ON", NULL, "OFF", -1, 0);
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
    TXT_C(lmp ? "CB PANEL 16 - LMP BUS" : "CB PANEL 11 - CDR BUS", 575, 6, 4.0f);
    for (int r = 0; r < 5; r++) {
        float y = 44 + r * 104;
        group(8, y - 22, 1134, 92, cbg11[(r + lmp * 2) % 5]);
        for (int i = 0; i < 24; i++) {
            const char *lab = cb11[(r + lmp * 2) % 5][(i + lmp * 7) % 24];
            float x = 34 + i * 46.5f;
            breaker(x, y + 18, lab, 0);
            breaker(x, y + 46, NULL, 0);
        }
    }
}

void cp_draw_panel(int panel, int p, Camera2D cam)
{
    pass = p;
    cur_panel = panel;
    cur_cam = cam;
    const PanelDef *pd = &cp_panels[panel];
    if (pass == 0) {
        DrawRectangleRec((Rectangle){ 0, 0, pd->w_mm, pd->h_mm }, C_PANEL);
        // Mounting screws in the corners, as on the real panels.
        float m = 4.5f;
        Vector2 sc[4] = { { m, m }, { pd->w_mm - m, m }, { m, pd->h_mm - m }, { pd->w_mm - m, pd->h_mm - m } };
        for (int i = 0; i < 4; i++) {
            DrawCircleV(sc[i], 2.0f, (Color){ 120, 122, 126, 255 });
            DrawLineEx((Vector2){ sc[i].x - 1.4f, sc[i].y }, (Vector2){ sc[i].x + 1.4f, sc[i].y }, 0.5f, (Color){ 50, 50, 52, 255 });
        }
    } else {
        DrawRectangleRec((Rectangle){ 0, 0, pd->w_mm, pd->h_mm }, BLACK);
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
    if (event_run && ets != 0) event_timer += dts;
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
    switch (h->kind) {
    case HOT_DSKY:
        dsky_mouse_press(dsky_key_from_mm(u, v));
        return;
    case HOT_DEDA:
        deda_key(h->idx);
        return;
    case HOT_CB:
        crew_say("Circuit breakers stay in: pulling them is not modelled");
        return;
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
        int n = *s + (u >= cx ? 1 : -1);
        if (n < 0) n = 0;
        if (n > h->idx - 1) n = h->idx - 1;
        *s = n;
        return;
    }
    case HOT_BUTTON:
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
    int functional = h->id == ID_ENGARM || h->id == ID_PGNS || h->id == ID_ROD || h->id == ID_STOP5 || h->id == ID_STOP6 ||
                     h->id == ID_MA1 || h->id == ID_MA2 || h->id == ID_FLOOD || h->id == ID_INTEG || h->id == ID_NUM ||
                     h->id == ID_LTGTEST || h->id == ID_XSCALE || h->id == ID_RATESCALE || h->id == ID_MODESEL ||
                     h->id == ID_ET || h->id == ID_PLUSX5 || h->id == ID_PLUSX6;
    switch (h->kind) {
    case HOT_DSKY: {
        int k = dsky_key_from_mm(u, v);
        snprintf(b, sizeof b, k >= 0 ? "DSKY  %s" : "DSKY - LGC display and keyboard", dsky_key_name(k));
        return b;
    }
    case HOT_DEDA: return "AGS DEDA (abort guidance not simulated)";
    case HOT_CB: snprintf(b, sizeof b, "Circuit breaker  %s  (closed)", h->id ? h->id : ""); return b;
    case HOT_TOGGLE:
        if (h->id == ID_ROD) return "RATE OF DESCENT  click up: -1 ft/s  down: +1 ft/s  (P66)";
        snprintf(b, sizeof b, "%s  %s%s", h->id, *state_of(h->id, 0) > 0 ? "UP" : *state_of(h->id, 0) < 0 ? "DOWN" : "CENTRE",
                 functional ? "" : "   (not modelled)");
        return b;
    case HOT_KNOB:
        snprintf(b, sizeof b, "%s  position %d   click left/right half to turn%s", h->id, *state_of(h->id, 0),
                 functional ? "" : "   (not modelled)");
        return b;
    case HOT_BUTTON:
        snprintf(b, sizeof b, "%s%s", h->id, functional ? "" : "   (not modelled)");
        return b;
    }
    return "";
}
