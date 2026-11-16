// Displays for the ascent and docking: the docking readout and the results
// panels (orbit insertion, docked, failed).

#include <math.h>
#include <stdio.h>

#include "raylib.h"

#include "ascent.h"
#include "hud.h"
#include "lm.h"

#define DEG (PI_D / 180.0)
#define DROGUE_R 0.33
#define CAPTURE_MAX_V 0.60
#define RETRACT_TIME 8.0

// ---------------------------------------------------------------------------
// Displays.

static Color colr(double v, double good, double bad)
{
    v = fabs(v);
    return v <= good ? (Color){ 120, 255, 150, 255 } : v <= bad ? (Color){ 255, 200, 70, 255 } : (Color){ 255, 90, 70, 255 };
}

void ascent_overlay(int w, int h)
{
    if (!ascent_active()) return;
    float sc = h / 1080.0f;
    if (sc < 0.7f) sc = 0.7f;
    int ph = ascent_phase();
    if (ph == MP_PROX) {
        DockGeo g;
        if (!ascent_dock_geo(&g)) return;
        float x = w - 360 * sc, y = 150 * sc;
        DrawRectangleRounded((Rectangle){ x, y, 340 * sc, 286 * sc }, 0.06f, 6, (Color){ 6, 10, 14, 200 });
        hud_text(font_bold, "DOCKING  -  COLUMBIA", x + 16 * sc, y + 10 * sc, 22 * sc, (Color){ 140, 200, 255, 255 });
        char b[96];
        double ry = 46;
        snprintf(b, sizeof b, "RANGE   %7.1f m", fmax(0, g.axial));
        hud_text(font_mono, b, x + 16 * sc, y + ry * sc, 21 * sc, (Color){ 230, 234, 240, 255 }); ry += 30;
        snprintf(b, sizeof b, "RATE    %+7.2f m/s", g.rate);
        double cl = -g.rate;
        Color rc = g.axial > 8 ? colr(cl > 0 && cl < 1.0 ? 0 : 1, 0.5, 0.5)
                              : cl > CAPTURE_MAX_V ? (Color){ 255, 90, 70, 255 } : cl > 0.3 ? (Color){ 255, 200, 70, 255 }
                              : cl > 0.02 ? (Color){ 120, 255, 150, 255 } : (Color){ 200, 205, 212, 255 };
        hud_text(font_mono, b, x + 16 * sc, y + ry * sc, 21 * sc, rc); ry += 30;
        snprintf(b, sizeof b, "LEFT/RT %+7.2f m", -g.lat_y);
        hud_text(font_mono, b, x + 16 * sc, y + ry * sc, 21 * sc, colr(g.lat_y, 0.15, DROGUE_R)); ry += 30;
        snprintf(b, sizeof b, "UP/DOWN %+7.2f m", g.lat_z);
        hud_text(font_mono, b, x + 16 * sc, y + ry * sc, 21 * sc, colr(g.lat_z, 0.15, DROGUE_R)); ry += 30;
        snprintf(b, sizeof b, "DRIFT   %+5.2f %+5.2f m/s", -g.vy, g.vz);
        hud_text(font_mono, b, x + 16 * sc, y + ry * sc, 21 * sc, colr(fmax(fabs(g.vy), fabs(g.vz)), 0.03, 0.1)); ry += 30;
        snprintf(b, sizeof b, "ANGLE   %6.1f deg", g.angle / DEG);
        hud_text(font_mono, b, x + 16 * sc, y + ry * sc, 21 * sc, colr(g.angle / DEG, 3, 10)); ry += 30;
        snprintf(b, sizeof b, "ROLL    %+6.1f deg", g.roll / DEG);
        hud_text(font_mono, b, x + 16 * sc, y + ry * sc, 21 * sc, colr(g.roll / DEG, 3, 15)); ry += 30;
        snprintf(b, sizeof b, "RCS     %5.0f kg", lm.m_rcs);
        hud_text(font_mono, b, x + 16 * sc, y + ry * sc, 21 * sc, lm.m_rcs < 40 ? (Color){ 255, 90, 70, 255 } : (Color){ 200, 205, 212, 255 });
    }
    const AscentStats *st = ascent_stats();
    if (st->msg[0] && lm.t - st->msg_t < 6 && (ph == MP_PROX || ph == MP_DOCKED)) {
        hud_text_centered(font_bold, st->msg, w / 2.0f, h - 150 * sc, 24 * sc, (Color){ 255, 220, 140, 255 });
    }
}

static void row(float x, float y, float sc, const char *label, const char *val, Color c)
{
    hud_text(font_ui, label, x + 70 * sc, y, 26 * sc, (Color){ 170, 176, 186, 255 });
    hud_text(font_mono, val, x + 380 * sc, y + 2 * sc, 24 * sc, c);
}

void ascent_draw_results(int w, int h, int client)
{
    if (!ascent_results_due()) return;
    const AscentStats *st = ascent_stats();
    float sc = h / 1080.0f;
    if (sc < 0.7f) sc = 0.7f;
    int ph = ascent_phase();
    const char *title, *sub, *keys;
    Color tc;
    if (ph == MP_ORBIT) {
        title = "IN LUNAR ORBIT";
        sub = st->peri > 9 ? "Luminary's P12 put Eagle into a safe orbit, trailing Columbia" : "In orbit, but the low point is uncomfortably low";
        tc = st->peri > 9 ? (Color){ 120, 255, 150, 255 } : (Color){ 255, 200, 70, 255 };
        keys = client ? "Waiting for the Commander.   Space hides this"
                      : "Enter  rendezvous with Columbia (skips 3 1/2 hours)      Space  hide this      Esc  menu";
    } else if (ph == MP_DOCKED) {
        title = "DOCKED WITH COLUMBIA";
        sub = "Eagle's job is done. Next stop: Earth";
        tc = (Color){ 120, 255, 150, 255 };
        keys = client ? "Space hides this   Esc  menu" : "Enter  back to the menu      Space  hide this";
    } else {
        title = "DOCKING FAILED";
        sub = st->fail;
        tc = (Color){ 255, 90, 70, 255 };
        keys = client ? "Waiting for the Commander.   Space hides this" : "Enter  try the approach again      Space  hide this      Esc  menu";
    }
    float bw = 820 * sc, bh = 420 * sc, x = (w - bw) / 2, y = (h - bh) / 2;
    DrawRectangleRounded((Rectangle){ x, y, bw, bh }, 0.06f, 8, (Color){ 8, 10, 14, 225 });
    DrawRectangleRoundedLinesEx((Rectangle){ x, y, bw, bh }, 0.06f, 8, 2.0f, tc);
    hud_text_centered(font_bold, title, w / 2.0f, y + 28 * sc, 52 * sc, tc);
    hud_text_centered(font_ui, sub, w / 2.0f, y + 92 * sc, 24 * sc, (Color){ 220, 224, 230, 255 });
    char b[160];
    float ry = y + 150 * sc;
    Color wht = { 220, 224, 230, 255 }, grn = { 120, 255, 150, 255 }, amb = { 255, 200, 70, 255 };
    if (ph == MP_ORBIT) {
        snprintf(b, sizeof b, "%.1f x %.1f km  (%.1f x %.1f nmi)", st->peri, st->apo, st->peri / 1.852, st->apo / 1.852);
        row(x, ry, sc, "Orbit", b, st->peri > 9 ? grn : amb); ry += 40 * sc;
        row(x, ry, sc, "Apollo 11 got", "17.4 x 86.5 km  (9.4 x 46.7 nmi)", wht); ry += 40 * sc;
        snprintf(b, sizeof b, "%.0f s  (%d min %02d s)", st->t_cut - st->t_lift, (int)(st->t_cut - st->t_lift) / 60, (int)(st->t_cut - st->t_lift) % 60);
        row(x, ry, sc, "Ascent burn", b, wht); ry += 40 * sc;
        snprintf(b, sizeof b, "%.0f kg ascent   %.0f kg RCS", st->aps_left, st->rcs_left);
        row(x, ry, sc, "Propellant left", b, wht);
    } else {
        double t = st->t_capture - st->t_start;
        snprintf(b, sizeof b, "%.2f m/s  (%.2f ft/s)", st->c_speed, st->c_speed / 0.3048);
        row(x, ry, sc, "Contact speed", b, ph == MP_DOCKED ? (st->c_speed <= 0.3 ? grn : amb) : wht); ry += 40 * sc;
        snprintf(b, sizeof b, "%.0f cm off axis, %.1f deg tilt", st->c_radial * 100, st->c_angle / DEG);
        row(x, ry, sc, "Alignment", b, ph == MP_DOCKED ? (st->c_radial < 0.15 && st->c_angle < 3 * DEG ? grn : amb) : wht); ry += 40 * sc;
        snprintf(b, sizeof b, "%+.1f deg", st->c_roll / DEG);
        row(x, ry, sc, "Roll", b, ph == MP_DOCKED ? (fabs(st->c_roll) < 5 * DEG ? grn : amb) : wht); ry += 40 * sc;
        if (ph == MP_DOCKED) {
            snprintf(b, sizeof b, "%d min %02d s, %.0f kg of RCS, %d bump%s", (int)t / 60, (int)t % 60, st->c_rcs, st->bounces,
                     st->bounces == 1 ? "" : "s");
            row(x, ry, sc, "Approach", b, wht);
        }
    }
    hud_text_centered(font_ui, keys, w / 2.0f, y + bh - 50 * sc, 22 * sc, (Color){ 150, 158, 170, 255 });
}
