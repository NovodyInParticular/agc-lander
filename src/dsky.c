#include <stddef.h>
#include <string.h>

#include "agc_host.h"
#include "dsky.h"
#include "hud.h"
#include "pcanvas.h"

// ---------------------------------------------------------------------------
// Colours.

static const Color C_HOUSING   = { 74, 76, 78, 255 };
static const Color C_PANEL     = { 40, 42, 44, 255 };
static const Color C_EL_BG     = { 14, 18, 16, 255 };
static const Color C_EL_ON     = { 120, 255, 140, 255 };
static const Color C_EL_OFF    = { 26, 40, 30, 255 };
static const Color C_EL_LABEL  = { 120, 255, 140, 255 };
static const Color C_LAMP_OFF  = { 58, 60, 60, 255 };
static const Color C_LAMP_WHITE= { 235, 235, 225, 255 };
static const Color C_LAMP_AMBER= { 255, 176, 40, 255 };
static const Color C_KEY       = { 34, 35, 37, 255 };
static const Color C_KEY_DOWN  = { 70, 72, 76, 255 };
static const Color C_KEY_TEXT  = { 230, 230, 230, 255 };

// Drawing target: the 2D screen, or the cockpit panel canvas, where lit
// elements also glow (scaled by `gain`, the ANUN/NUM brightness).
static int canvas;
static float gain = 1;

static Color dim(Color c, float k)
{
    return (Color){ (unsigned char)(c.r * k), (unsigned char)(c.g * k), (unsigned char)(c.b * k), 255 };
}

static void box(Rectangle r, Color c, int lit)
{
    if (!canvas) { DrawRectangleRec(r, c); return; }
    if (lit) pc_glow(dim(c, gain));
    pc_rect(r.x, r.y, r.width, r.height, lit ? dim(c, 0.85f) : c);
    pc_glow_off();
}

static void rbox(Rectangle r, float roundness, Color c)
{
    if (canvas) pc_rrect(r, roundness, c);
    else DrawRectangleRounded(r, roundness, 8, c);
}

static void rbox_lines(Rectangle r, float roundness, float t, Color c)
{
    if (canvas) pc_rrect_lines(r, roundness, t, c);
    else DrawRectangleRoundedLinesEx(r, roundness, 6, t, c);
}

// Text centred in a rectangle, lines split at '\n'. glow: 0 none, else the
// fraction of the text colour it emits on the canvas.
static void label(const char *text, Rectangle r, float fs, Color c, float glow)
{
    fs *= 1.25f;
    if (canvas) {
        if (glow > 0) pc_glow(dim(c, glow * gain));
        pc_text_box(text, r, fs, 0.4f, c);
        pc_glow_off();
        return;
    }
    int lines = 1;
    for (const char *p = text; *p; p++) lines += *p == '\n';
    float y = r.y + (r.height - fs * (1 + (lines - 1) * 1.12f)) / 2;
    while (1) {
        char buf[32];
        const char *nl = strchr(text, '\n');
        size_t n = nl ? (size_t)(nl - text) : strlen(text);
        if (n >= sizeof buf) n = sizeof buf - 1;
        memcpy(buf, text, n);
        buf[n] = 0;
        Vector2 m = MeasureTextEx(font_bold, buf, fs, 0.4f);
        DrawTextEx(font_bold, buf, (Vector2){ r.x + (r.width - m.x) / 2, y }, fs, 0.4f, c);
        if (!nl) break;
        text = nl + 1;
        y += fs * 1.12f;
    }
}

// ---------------------------------------------------------------------------
// Seven-segment EL digits.

// Segment bits: a=top b=top-right c=bottom-right d=bottom e=bottom-left
// f=top-left g=middle.
static unsigned char segments_for(uint8_t code)
{
    switch (agc_digit_char(code)) {
    case '0': return 0x3F; case '1': return 0x06; case '2': return 0x5B;
    case '3': return 0x4F; case '4': return 0x66; case '5': return 0x6D;
    case '6': return 0x7D; case '7': return 0x07; case '8': return 0x7F;
    case '9': return 0x6F;
    default:  return 0;
    }
}

static void segment(Rectangle r, int on)
{
    Color c = on ? C_EL_ON : C_EL_OFF;
    if (!canvas) { DrawRectangleRounded(r, 0.9f, 4, c); return; }
    if (on) pc_glow(dim(c, gain));
    pc_rrect(r, 0.9f, on ? dim(c, 0.9f) : c);
    pc_glow_off();
}

static void draw_digit(float x, float y, float w, float h, unsigned char seg)
{
    float t = w * 0.19f, gap = t * 0.14f, hh = h / 2;
    Rectangle r[7] = {
        { x + t * 0.5f + gap, y, w - t - 2 * gap, t },                 // a
        { x + w - t, y + t * 0.5f + gap, t, hh - t * 0.5f - 2 * gap }, // b
        { x + w - t, y + hh + gap, t, hh - t * 0.5f - 2 * gap },       // c
        { x + t * 0.5f + gap, y + h - t, w - t - 2 * gap, t },         // d
        { x, y + hh + gap, t, hh - t * 0.5f - 2 * gap },               // e
        { x, y + t * 0.5f + gap, t, hh - t * 0.5f - 2 * gap },         // f
        { x + t * 0.5f + gap, y + hh - t / 2, w - t - 2 * gap, t },    // g
    };
    for (int i = 0; i < 7; i++) segment(r[i], (seg >> i) & 1);
}

static void draw_sign(float x, float y, float w, float h, uint8_t sign)
{
    float t = w * 0.19f;
    int plus = (sign & 2) != 0, minus = !plus && (sign & 1);
    segment((Rectangle){ x, y + h / 2 - t / 2, w, t }, plus || minus);
    if (plus) segment((Rectangle){ x + w / 2 - t / 2, y + h * 0.18f, t, h * 0.64f }, 1);
}

// Lit legend bar (PROG / VERB / NOUN): dark lettering on the glowing bar.
static void draw_label(const char *text, float x, float y, float w, float h)
{
    box((Rectangle){ x, y, w, h }, C_EL_LABEL, 1);
    label(text, (Rectangle){ x, y, w, h }, h * 0.66f, C_EL_BG, 0);
}

// ---------------------------------------------------------------------------
// Lamps.

typedef struct { const char *label; int amber; size_t offset; } Lamp;

#define LAMP(name, field, amber) { name, amber, offsetof(DskyState, field) }

static const Lamp lamps[14] = {
    LAMP("UPLINK\nACTY", uplink_acty, 0), LAMP("TEMP", temp, 1),
    LAMP("NO ATT", no_att, 0),            LAMP("GIMBAL\nLOCK", gimbal_lock, 1),
    LAMP("STBY", stby, 0),                LAMP("PROG", prog_lamp, 1),
    LAMP("KEY REL", key_rel, 0),          LAMP("RESTART", restart, 1),
    LAMP("OPR ERR", opr_err, 0),          LAMP("TRACKER", tracker, 1),
    LAMP("PRIO DISP", prio_disp, 0),      LAMP("ALT", alt, 1),
    LAMP("NO DAP", no_dap, 0),            LAMP("VEL", vel, 1),
};

// ---------------------------------------------------------------------------
// Keypad.

typedef struct {
    const char *label;
    int code;          // KEY_AGC_* or -1 for PRO
    float col, row;    // grid position (row may be fractional)
    int kb1, kb2, kb3; // raylib keyboard shortcuts
} Key;

#define PRO_CODE (-1)

static const Key keys[19] = {
    { "VERB", KEY_AGC_VERB, 0, 0.5f, KEY_V, 0, 0 },
    { "NOUN", KEY_AGC_NOUN, 0, 1.5f, KEY_N, 0, 0 },
    { "+", KEY_AGC_PLUS, 1, 0, KEY_EQUAL, KEY_KP_ADD, 0 },
    { "-", KEY_AGC_MINUS, 1, 1, KEY_MINUS, KEY_KP_SUBTRACT, 0 },
    { "0", KEY_AGC_0, 1, 2, KEY_ZERO, KEY_KP_0, 0 },
    { "7", KEY_AGC_7, 2, 0, KEY_SEVEN, KEY_KP_7, 0 },
    { "4", KEY_AGC_4, 2, 1, KEY_FOUR, KEY_KP_4, 0 },
    { "1", KEY_AGC_1, 2, 2, KEY_ONE, KEY_KP_1, 0 },
    { "8", KEY_AGC_8, 3, 0, KEY_EIGHT, KEY_KP_8, 0 },
    { "5", KEY_AGC_5, 3, 1, KEY_FIVE, KEY_KP_5, 0 },
    { "2", KEY_AGC_2, 3, 2, KEY_TWO, KEY_KP_2, 0 },
    { "9", KEY_AGC_9, 4, 0, KEY_NINE, KEY_KP_9, 0 },
    { "6", KEY_AGC_6, 4, 1, KEY_SIX, KEY_KP_6, 0 },
    { "3", KEY_AGC_3, 4, 2, KEY_THREE, KEY_KP_3, 0 },
    { "CLR", KEY_AGC_CLR, 5, 0, KEY_C, KEY_BACKSPACE, KEY_DELETE },
    { "PRO", PRO_CODE, 5, 1, KEY_P, 0, 0 },
    { "KEY\nREL", KEY_AGC_KEYREL, 5, 2, KEY_K, 0, 0 },
    { "ENTR", KEY_AGC_ENTR, 6, 0.5f, KEY_ENTER, KEY_KP_ENTER, 0 },
    { "RSET", KEY_AGC_RSET, 6, 1.5f, KEY_R, 0, 0 },
};

#define KEY_SIZE 62.0f
#define KEY_GAP 8.0f
#define KEYPAD_X ((DSKY_W - (7 * KEY_SIZE + 6 * KEY_GAP)) / 2)
#define KEYPAD_Y 500.0f

static Rectangle key_rect(const Key *k)
{
    return (Rectangle){ KEYPAD_X + k->col * (KEY_SIZE + KEY_GAP),
                        KEYPAD_Y + k->row * (KEY_SIZE + KEY_GAP), KEY_SIZE, KEY_SIZE };
}

static int mouse_held_key = -1;
static float key_flash[19];
static int pro_down;

static void press(int i)
{
    if (keys[i].code == PRO_CODE) {
        if (!pro_down) agc_host_pro(1);
        pro_down = 1;
    } else {
        agc_host_key(keys[i].code);
    }
    key_flash[i] = 0.12f;
}

static void release_pro(void)
{
    if (pro_down) agc_host_pro(0);
    pro_down = 0;
}

static int shortcut_matches(const Key *k, int kb)
{
    return kb && (kb == k->kb1 || kb == k->kb2 || kb == k->kb3);
}

// Keyboard shortcuts, key-flash timers and PRO release: once per frame.
void dsky_frame(void)
{
    // Keyboard: digit keys are matched by key code, so "+" is shift-free (=).
    for (int kb = GetKeyPressed(); kb; kb = GetKeyPressed())
        for (int i = 0; i < 19; i++)
            if (shortcut_matches(&keys[i], kb))
                press(i);
    int pro_kb_held = IsKeyDown(KEY_P);
    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        mouse_held_key = -1;
    int pro_mouse_held = mouse_held_key >= 0 && keys[mouse_held_key].code == PRO_CODE;
    if (pro_down && !pro_kb_held && !pro_mouse_held)
        release_pro();
    float dt = GetFrameTime();
    for (int i = 0; i < 19; i++)
        if (key_flash[i] > 0) key_flash[i] -= dt;
}

int dsky_key_at(Vector2 p)
{
    for (int i = 0; i < 19; i++)
        if (CheckCollisionPointRec(p, key_rect(&keys[i])))
            return i;
    return -1;
}

void dsky_mouse_press(int i)
{
    if (i < 0 || i >= 19) return;
    mouse_held_key = i;
    press(i);
}

const char *dsky_key_name(int i)
{
    static char b[16];
    if (i < 0 || i >= 19) return "";
    const char *s = keys[i].label;
    int n = 0;
    for (; *s && n < 15; s++) b[n++] = *s == '\n' ? ' ' : *s;
    b[n] = 0;
    return b;
}

static void handle_mouse(Camera2D cam)
{
    Vector2 m = GetScreenToWorld2D(GetMousePosition(), cam);
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        dsky_mouse_press(dsky_key_at(m));
}

static void draw_native(void);

// Onto the cockpit panel canvas, in native DSKY units (the caller has set the
// canvas transform); lit elements glow, scaled by `brightness`.
void dsky_draw_canvas(float brightness)
{
    canvas = 1;
    gain = brightness;
    draw_native();
    canvas = 0;
    gain = 1;
}

// ---------------------------------------------------------------------------

void dsky_update_draw(Rectangle area)
{
    float scale = area.width / DSKY_W;
    if (area.height / DSKY_H < scale)
        scale = area.height / DSKY_H;
    Camera2D cam = { 0 };
    cam.zoom = scale;
    cam.offset = (Vector2){ area.x + (area.width - DSKY_W * scale) / 2,
                            area.y + (area.height - DSKY_H * scale) / 2 };
    handle_mouse(cam);
    BeginMode2D(cam);
    draw_native();
    EndMode2D();
}

// Draw at native DSKY coordinates (0..DSKY_W, 0..DSKY_H) to the current target.
static void draw_native(void)
{
    const DskyState *d = agc_host_dsky();
    rbox((Rectangle){ 0, 0, DSKY_W, DSKY_H }, 0.03f, C_HOUSING);

    // Indicator lamp panel.
    Rectangle lp = { 16, 16, 230, 468 };
    box(lp, C_PANEL, 0);
    for (int i = 0; i < 14; i++) {
        int col = i % 2, row = i / 2;
        Rectangle r = { lp.x + 10 + col * 108, lp.y + 10 + row * 64, 102, 56 };
        int on = *(const int *)((const char *)d + lamps[i].offset);
        box(r, on ? (lamps[i].amber ? C_LAMP_AMBER : C_LAMP_WHITE) : C_LAMP_OFF, on);
        label(lamps[i].label, r, 14, on ? (Color){ 30, 30, 30, 255 } : (Color){ 140, 142, 138, 255 }, 0);
    }

    // EL display panel.
    Rectangle ep = { 262, 16, 242, 468 };
    box(ep, C_PANEL, 0);
    box((Rectangle){ ep.x + 8, ep.y + 8, ep.width - 16, ep.height - 16 }, C_EL_BG, 0);

    float dw = 28, dh = 46;  // digit size
    float rx = ep.x + 18;

    // COMP ACTY and PROG.
    Rectangle ca = { rx, ep.y + 22, 82, 70 };
    box(ca, d->comp_acty ? C_EL_ON : C_EL_OFF, d->comp_acty);
    label("COMP\nACTY", ca, 14, d->comp_acty ? C_EL_BG : (Color){ 70, 100, 78, 255 }, 0);
    float px = ep.x + ep.width - 18 - 2 * dw - 10;
    draw_label("PROG", px, ep.y + 22, 2 * dw + 10, 18);
    draw_digit(px, ep.y + 46, dw, dh, segments_for(d->prog[0]));
    draw_digit(px + dw + 10, ep.y + 46, dw, dh, segments_for(d->prog[1]));

    // VERB and NOUN (blank while V/N flash is in its off phase).
    float vny = ep.y + 110;
    int vn_on = !d->vn_flash_off;
    draw_label("VERB", rx, vny, 2 * dw + 10, 18);
    draw_label("NOUN", px, vny, 2 * dw + 10, 18);
    draw_digit(rx, vny + 24, dw, dh, vn_on ? segments_for(d->verb[0]) : 0);
    draw_digit(rx + dw + 10, vny + 24, dw, dh, vn_on ? segments_for(d->verb[1]) : 0);
    draw_digit(px, vny + 24, dw, dh, vn_on ? segments_for(d->noun[0]) : 0);
    draw_digit(px + dw + 10, vny + 24, dw, dh, vn_on ? segments_for(d->noun[1]) : 0);

    // R1-R3.
    float ry = vny + 86;
    for (int r = 0; r < 3; r++) {
        float y = ry + r * 84;
        box((Rectangle){ rx, y, ep.width - 36, 4 }, C_EL_LABEL, 1);
        y += 16;
        draw_sign(rx, y, 26, dh, d->sign[r]);
        for (int j = 0; j < 5; j++)
            draw_digit(rx + 34 + j * (dw + 8), y, dw, dh, segments_for(d->reg[r][j]));
    }

    // Keypad (the legends are back-lit in the cabin).
    for (int i = 0; i < 19; i++) {
        int down = key_flash[i] > 0 || i == mouse_held_key || (keys[i].code == PRO_CODE && pro_down);
        Rectangle r = key_rect(&keys[i]);
        rbox(r, 0.12f, down ? C_KEY_DOWN : C_KEY);
        rbox_lines(r, 0.12f, 2, (Color){ 20, 20, 20, 255 });
        label(keys[i].label, r, strlen(keys[i].label) > 2 ? 17 : 30, C_KEY_TEXT, 0.45f);
    }
}
