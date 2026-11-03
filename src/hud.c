#include <math.h>
#include <stdio.h>
#include <string.h>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "agc_host.h"
#include "crew.h"
#include "hud.h"
#include "lm.h"
#include "moon.h"

extern const unsigned char font_jbm[], font_rc[], font_rcb[];
extern const unsigned int font_jbm_len, font_rc_len, font_rcb_len;

Font font_mono, font_ui, font_bold;

static RenderTexture2D fdai_rt;
static Model fdai_ball;
static Texture2D fdai_tex;

#define FT 0.3048

static const Color PANEL = { 14, 16, 20, 205 };
static const Color EDGE = { 70, 78, 90, 255 };
static const Color TXT = { 214, 220, 228, 255 };
static const Color DIM = { 120, 128, 140, 255 };
static const Color ACC = { 120, 255, 150, 255 };
static const Color AMB = { 255, 176, 40, 255 };
static const Color RED_ = { 255, 70, 60, 255 };

void hud_text(Font f, const char *s, float x, float y, float size, Color c)
{
    DrawTextEx(f, s, (Vector2){ roundf(x), roundf(y) }, size, size * 0.02f, c);
}

void hud_text_centered(Font f, const char *s, float cx, float y, float size, Color c)
{
    Vector2 m = MeasureTextEx(f, s, size, size * 0.02f);
    hud_text(f, s, cx - m.x / 2, y, size, c);
}

static Font load_font(const unsigned char *data, unsigned int len, int size)
{
    Font f = LoadFontFromMemory(".ttf", data, (int)len, size, NULL, 0);
    GenTextureMipmaps(&f.texture);
    SetTextureFilter(f.texture, TEXTURE_FILTER_TRILINEAR);
    return f;
}

// FDAI ball texture: equirectangular, longitude = yaw about the SM pole.
static Texture2D make_fdai_texture(void)
{
    const int W = 1024, H = 512;
    Image img = GenImageColor(W, H, (Color){ 0, 0, 0, 255 });
    for (int y = 0; y < H; y++) {
        double lat = (0.5 - (y + 0.5) / H) * 180.0;
        for (int x = 0; x < W; x++) {
            double lon = (x + 0.5) / W * 360.0;
            int upper = lat > 0;
            Color c = upper ? (Color){ 228, 226, 218, 255 } : (Color){ 34, 34, 38, 255 };
            Color line = upper ? (Color){ 30, 30, 30, 255 } : (Color){ 220, 220, 220, 255 };
            double ml = fmod(lon, 30.0), pl = fmod(fabs(lat), 10.0);
            if (ml < 0.45 || ml > 29.55) c = line;
            if ((pl < 0.35 || pl > 9.65) && fabs(lat) < 85) {
                double tick = fmod(lon, 10.0);
                if (tick < 1.4 || tick > 8.6 || fabs(lat) < 1) c = line;
            }
            if (fabs(lat) < 0.6) c = (Color){ 255, 255, 255, 255 };
            ImageDrawPixel(&img, x, y, c);
        }
    }
    // Yaw labels along the equator band.
    for (int k = 0; k < 12; k++) {
        char s[4];
        snprintf(s, sizeof s, "%02d", k * 3);
        int x = (int)(k * 30.0 / 360.0 * W) + 8;
        ImageDrawText(&img, s, x, H / 2 - 30, 20, (Color){ 30, 30, 30, 255 });
        ImageDrawText(&img, s, x, H / 2 + 12, 20, (Color){ 220, 220, 220, 255 });
    }
    Texture2D t = LoadTextureFromImage(img);
    UnloadImage(img);
    GenTextureMipmaps(&t);
    SetTextureFilter(t, TEXTURE_FILTER_TRILINEAR);
    return t;
}

void hud_init(void)
{
    font_mono = load_font(font_jbm, font_jbm_len, 48);
    font_ui = load_font(font_rc, font_rc_len, 48);
    font_bold = load_font(font_rcb, font_rcb_len, 64);
    fdai_tex = make_fdai_texture();
    fdai_ball = LoadModelFromMesh(GenMeshSphere(1.0f, 48, 64));
    fdai_ball.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = fdai_tex;
    fdai_rt = LoadRenderTexture(512, 512);
    SetTextureFilter(fdai_rt.texture, TEXTURE_FILTER_BILINEAR);
}

Texture2D hud_fdai_texture(void) { return fdai_rt.texture; }

void hud_shutdown(void)
{
    UnloadRenderTexture(fdai_rt);
    UnloadModel(fdai_ball);
    UnloadTexture(fdai_tex);
    UnloadFont(font_mono);
    UnloadFont(font_ui);
    UnloadFont(font_bold);
}

static void panel(Rectangle r, const char *title)
{
    DrawRectangleRounded(r, 0.06f, 8, PANEL);
    DrawRectangleRoundedLinesEx(r, 0.06f, 8, 1.5f, EDGE);
    if (title) hud_text(font_bold, title, r.x + 10, r.y + 6, 15, DIM);
}

// --- FDAI ----------------------------------------------------------------------------

// Render the 8-ball into its texture. Must run outside any other texture mode
// (raylib's EndTextureMode rebinds the window framebuffer).
void hud_prepare(void)
{
    V3 g = lm_gimbal_angles();
    M3 t = m3_mul(m3_rx(g.x), m3_mul(m3_rz(g.z), m3_ry(g.y)));   // NB from SM
    // raylib's sphere mesh has its poles on its Z axis: mesh z = SM X (pole),
    // mesh x = SM Y, mesh y = SM Z.  Screen: x = body Y, y = body X, z = -body Z.
    M3 sm_from_mesh = { { { 0, 0, 1 }, { 1, 0, 0 }, { 0, 1, 0 } } };
    M3 screen_from_body = { { { 0, 1, 0 }, { 1, 0, 0 }, { 0, 0, -1 } } };
    M3 m = m3_mul(screen_from_body, m3_mul(t, sm_from_mesh));
    Matrix mm = { (float)m.m[0][0], (float)m.m[0][1], (float)m.m[0][2], 0,
                  (float)m.m[1][0], (float)m.m[1][1], (float)m.m[1][2], 0,
                  (float)m.m[2][0], (float)m.m[2][1], (float)m.m[2][2], 0, 0, 0, 0, 1 };
    fdai_ball.transform = mm;

    BeginTextureMode(fdai_rt);
    ClearBackground((Color){ 0, 0, 0, 0 });
    Camera3D cam = { { 0, 0, 3.2f }, { 0, 0, 0 }, { 0, 1, 0 }, 40.0f, CAMERA_PERSPECTIVE };
    BeginMode3D(cam);
    DrawModel(fdai_ball, (Vector3){ 0, 0, 0 }, 1.0f, WHITE);
    EndMode3D();
    EndTextureMode();
}

static void draw_fdai(Rectangle r)
{
    V3 g = lm_gimbal_angles();
    panel(r, "FDAI");
    float cx = r.x + r.width / 2, cy = r.y + r.height / 2 + 6, rad = fminf(r.width, r.height) * 0.36f;
    DrawCircleV((Vector2){ cx, cy }, rad + 10, (Color){ 40, 42, 46, 255 });
    DrawCircleV((Vector2){ cx, cy }, rad + 4, (Color){ 10, 10, 12, 255 });
    Rectangle src = { 0, 0, (float)fdai_rt.texture.width, -(float)fdai_rt.texture.height };
    float s = rad * 2.0f * 1.22f;
    DrawTexturePro(fdai_rt.texture, src, (Rectangle){ cx - s / 2, cy - s / 2, s, s }, (Vector2){ 0, 0 }, 0, WHITE);
    // Fixed aircraft symbol.
    DrawLineEx((Vector2){ cx - rad * 0.55f, cy }, (Vector2){ cx - rad * 0.15f, cy }, 3, AMB);
    DrawLineEx((Vector2){ cx + rad * 0.15f, cy }, (Vector2){ cx + rad * 0.55f, cy }, 3, AMB);
    DrawCircleV((Vector2){ cx, cy }, 3, AMB);
    // Attitude error needles (driven by the AGC through the CDU error counters).
    float ex = (float)fmax(-1, fmin(1, lm.needle[1] / 384.0)), ey = (float)fmax(-1, fmin(1, lm.needle[2] / 384.0));
    float er = (float)fmax(-1, fmin(1, lm.needle[0] / 384.0));
    DrawLineEx((Vector2){ cx + ex * rad * 0.8f, cy - rad * 0.75f }, (Vector2){ cx + ex * rad * 0.8f, cy - rad * 0.2f }, 3, AMB);
    DrawLineEx((Vector2){ cx - rad * 0.75f, cy + ey * rad * 0.8f }, (Vector2){ cx - rad * 0.2f, cy + ey * rad * 0.8f }, 3, AMB);
    DrawLineEx((Vector2){ cx + er * rad * 0.8f - 10, cy - rad - 2 }, (Vector2){ cx + er * rad * 0.8f + 10, cy - rad - 2 }, 3, AMB);
    // Rate needles: body rates, 5 deg/s full scale.
    float ry = (float)fmax(-1, fmin(1, lm.w.x * 57.3 / 5)), rp = (float)fmax(-1, fmin(1, lm.w.y * 57.3 / 5));
    float rr = (float)fmax(-1, fmin(1, lm.w.z * 57.3 / 5));
    DrawRectangle((int)(cx + rr * rad - 3), (int)(cy - rad - 22), 6, 12, ACC);
    DrawRectangle((int)(cx + rad + 14), (int)(cy - rp * rad - 3), 12, 6, ACC);
    DrawRectangle((int)(cx + ry * rad - 3), (int)(cy + rad + 12), 6, 12, ACC);
    char b[64];
    snprintf(b, sizeof b, "R %05.1f  P %05.1f  Y %05.1f", fmod(g.x * 57.2958 + 360, 360), fmod(g.y * 57.2958 + 360, 360),
             fmod(g.z * 57.2958 + 360, 360));
    hud_text_centered(font_mono, b, cx, r.y + r.height - 22, 14, DIM);
}

// --- Tape meters ---------------------------------------------------------------------

static void tape(Rectangle r, const char *title, double value, double span, double major, const char *fmt, Color c)
{
    panel(r, title);
    Rectangle in = { r.x + 8, r.y + 26, r.width - 16, r.height - 34 };
    DrawRectangleRec(in, (Color){ 6, 7, 9, 255 });
    BeginScissorMode((int)in.x, (int)in.y, (int)in.width, (int)in.height);
    double ppu = in.height / span;
    double first = floor((value - span / 2) / major) * major;
    for (double v = first; v < value + span / 2 + major; v += major) {
        float y = (float)(in.y + in.height / 2 - (v - value) * ppu);
        DrawLineEx((Vector2){ in.x, y }, (Vector2){ in.x + 14, y }, 2, TXT);
        DrawLineEx((Vector2){ in.x, (float)(y + major / 2 * ppu) }, (Vector2){ in.x + 7, (float)(y + major / 2 * ppu) }, 1, DIM);
        char s[32];
        snprintf(s, sizeof s, fmt, v);
        hud_text(font_mono, s, in.x + 18, y - 8, 15, TXT);
    }
    EndScissorMode();
    float my = in.y + in.height / 2;
    DrawTriangle((Vector2){ in.x + in.width, my - 8 }, (Vector2){ in.x + in.width - 12, my }, (Vector2){ in.x + in.width, my + 8 }, c);
    DrawLineEx((Vector2){ in.x, my }, (Vector2){ in.x + in.width, my }, 1.5f, c);
}

// --- Cross pointer -------------------------------------------------------------------

static void cross_pointer(Rectangle r, double fwd, double lat)
{
    panel(r, "X-POINTER  ft/s");
    float cx = r.x + r.width / 2, cy = r.y + r.height / 2 + 8, s = fminf(r.width, r.height) * 0.36f;
    DrawRectangleLinesEx((Rectangle){ cx - s, cy - s, 2 * s, 2 * s }, 1, EDGE);
    for (int i = -4; i <= 4; i++) {
        DrawLine((int)(cx + i * s / 4), (int)(cy - 4), (int)(cx + i * s / 4), (int)(cy + 4), DIM);
        DrawLine((int)(cx - 4), (int)(cy + i * s / 4), (int)(cx + 4), (int)(cy + i * s / 4), DIM);
    }
    double full = 20.0;   // ft/s full scale (low scale)
    if (fabs(fwd) > 20 || fabs(lat) > 20) full = 200.0;
    float px = (float)fmax(-1, fmin(1, lat / FT / full)), py = (float)fmax(-1, fmin(1, fwd / FT / full));
    DrawLineEx((Vector2){ cx + px * s, cy - s }, (Vector2){ cx + px * s, cy + s }, 2.5f, ACC);
    DrawLineEx((Vector2){ cx - s, cy - py * s }, (Vector2){ cx + s, cy - py * s }, 2.5f, ACC);
    char b[48];
    snprintf(b, sizeof b, "FWD %+6.1f  LAT %+6.1f  (x%.0f)", fwd / FT, lat / FT, full / 20);
    hud_text_centered(font_mono, b, cx, r.y + r.height - 22, 13, DIM);
}

static void bar(float x, float y, float w, float h, float f, Color c, const char *label, const char *val)
{
    DrawRectangle((int)x, (int)y, (int)w, (int)h, (Color){ 6, 7, 9, 255 });
    DrawRectangle((int)x, (int)(y + h * (1 - f)), (int)w, (int)(h * f), c);
    DrawRectangleLines((int)x, (int)y, (int)w, (int)h, EDGE);
    hud_text_centered(font_bold, label, x + w / 2, y + h + 4, 14, DIM);
    hud_text_centered(font_mono, val, x + w / 2, y - 18, 13, TXT);
}

static const char *phase_name(void)
{
    const DskyState *d = agc_host_dsky();
    char p0 = agc_digit_char(d->prog[0]), p1 = agc_digit_char(d->prog[1]);
    if (p0 == '6' && p1 == '3') return lm.dps_on ? "P63  BRAKING PHASE" : "P63  BRAKING PHASE  (pre-ignition)";
    if (p0 == '6' && p1 == '4') return "P64  APPROACH PHASE";
    if (p0 == '6' && p1 == '5') return "P65  AUTO LANDING";
    if (p0 == '6' && p1 == '6') return "P66  RATE OF DESCENT";
    if (p0 == '6' && p1 == '8') return "P68  LANDING CONFIRMATION";
    if (p0 == '0' && p1 == '0') return "P00  LGC IDLE";
    return "";
}

static int minimal;
void hud_set_minimal(int on) { minimal = on; }

void hud_draw(int w, int h, int dsky_w, float warp, const char *camera_name, int paused)
{
    float scale = h / 1080.0f;
    if (scale < 0.7f) scale = 0.7f;
    float ph = 270 * scale;
    float y0 = h - ph - 14 * scale;
    float x = 14 * scale;

    // Altitude and rate: landing-radar style display from the vehicle.
    double alt = lm.altitude, hdot = lm.hdot;
    V3 vs_b = lm_inertial_to_body(lm_surface_velocity());
    // Forward / lateral relative to the LM's +Z / +Y body axes, projected horizontal.
    double fwd = vs_b.z, lat = vs_b.y;

    if (minimal) goto status;
    draw_fdai((Rectangle){ x, y0, ph, ph });
    x += ph + 10 * scale;
    double alt_ft = alt / FT;
    double span = alt_ft > 20000 ? 20000 : alt_ft > 2000 ? 2000 : 200;
    tape((Rectangle){ x, y0, 120 * scale, ph }, "ALT ft", alt_ft, span, span / 8, span >= 2000 ? "%6.0f" : "%5.0f", ACC);
    x += 128 * scale;
    double hd_ft = hdot / FT;
    double hspan = fabs(hd_ft) > 100 ? 400 : 40;
    tape((Rectangle){ x, y0, 110 * scale, ph }, "ALT RATE", hd_ft, hspan, hspan / 8, "%+5.0f", AMB);
    x += 118 * scale;
    cross_pointer((Rectangle){ x, y0, ph, ph }, fwd, lat);
    x += ph + 10 * scale;

    // Engine / propellant panel.
    Rectangle ep = { x, y0, 190 * scale, ph };
    panel(ep, "DPS / RCS");
    float bw = 30 * scale, bh = ph - 90 * scale, by = y0 + 50 * scale;
    char v[32];
    snprintf(v, sizeof v, "%3.0f%%", lm.thrust / 46978.0 * 100);
    bar(ep.x + 18 * scale, by, bw, bh, (float)(lm.thrust / 46978.0), AMB, "THR", v);
    double prop = lm.m_dps / 8248.0;
    snprintf(v, sizeof v, "%3.0f%%", prop * 100);
    bar(ep.x + 18 * scale + bw + 22 * scale, by, bw, bh, (float)prop, prop < 0.06 ? RED_ : ACC, "PROP", v);
    double rcs = lm.m_rcs / 287.0;
    snprintf(v, sizeof v, "%3.0f%%", rcs * 100);
    bar(ep.x + 18 * scale + 2 * (bw + 22 * scale), by, bw, bh, (float)rcs, (Color){ 120, 180, 255, 255 }, "RCS", v);
    if (prop < 0.06 && lm.dps_on && ((int)(GetTime() * 2) & 1))
        hud_text_centered(font_bold, "LOW LEVEL", ep.x + ep.width / 2, y0 + 24 * scale, 16 * scale, RED_);

status:
    if (minimal) DrawRectangleGradientH(0, 0, (int)(620 * scale), (int)(160 * scale), (Color){ 0, 0, 0, 170 }, (Color){ 0, 0, 0, 0 });
    // Top-left: mission status.
    double get = lm.t;
    int hh = (int)(get / 3600), mm = (int)(get / 60) % 60;
    double ss = get - hh * 3600 - mm * 60;
    char line[160];
    float tx = 18 * scale, ty = 14 * scale;
    snprintf(line, sizeof line, "GET %03d:%02d:%05.2f", hh, mm, ss);
    hud_text(font_mono, line, tx, ty, 26 * scale, TXT);
    hud_text(font_bold, phase_name(), tx, ty + 32 * scale, 22 * scale, ACC);
    snprintf(line, sizeof line, "ALT %8.0f ft   H-DOT %+7.1f ft/s   H-VEL %7.1f ft/s   MASS %6.0f kg",
             alt / FT, hdot / FT, lm.hspeed / FT, lm_mass());
    hud_text(font_mono, line, tx, ty + 62 * scale, 15 * scale, DIM);
    snprintf(line, sizeof line, "LR %s%s  ANT POS %d %s   CREW %s%s", lm.lr_range_good ? "ALT " : "--- ",
             lm.lr_vel_good ? "VEL" : "---", lm.lr_pos, lm.lr_move > 0 ? "(MOVING)" : "",
             crew.auto_crew ? "AUTO" : "MANUAL", lm.engine_armed ? "   DES ENG ARMED" : "");
    hud_text(font_mono, line, tx, ty + 82 * scale, 15 * scale, DIM);
    if (lm.probe_contact)
        hud_text(font_bold, "LUNAR CONTACT", tx, ty + 104 * scale, 26 * scale, (Color){ 120, 190, 255, 255 });

    // Prompt (top centre) and callout (lower centre).
    float cx = (w - dsky_w) / 2.0f;
    if (crew.prompt[0]) {
        Vector2 m = MeasureTextEx(font_ui, crew.prompt, 22 * scale, 0.4f);
        DrawRectangleRounded((Rectangle){ cx - m.x / 2 - 16, 14 * scale, m.x + 32, 36 * scale }, 0.4f, 8, (Color){ 40, 30, 8, 200 });
        hud_text_centered(font_ui, crew.prompt, cx, 20 * scale, 22 * scale, AMB);
    }
    if (crew.callout[0] && lm.t - crew.callout_time < 9) {
        float a = (float)fmin(1.0, (9 - (lm.t - crew.callout_time)) / 2.0);
        Color c = TXT;
        c.a = (unsigned char)(255 * a);
        hud_text_centered(font_ui, crew.callout, cx, y0 - 46 * scale, 26 * scale, c);
    }

    // Bottom-right of the scene: warp / camera / help.
    snprintf(line, sizeof line, "%s   TIME x%.0f%s", camera_name, warp, paused ? "   PAUSED" : "");
    Vector2 m = MeasureTextEx(font_mono, line, 16 * scale, 0.3f);
    hud_text(font_mono, line, w - dsky_w - m.x - 18 * scale, 18 * scale, 16 * scale, TXT);
    hud_text(font_ui, "F1 help", w - dsky_w - 70 * scale, 40 * scale, 16 * scale, DIM);
}
