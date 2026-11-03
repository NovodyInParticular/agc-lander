// Plain-English hints for new players. Everything here is derived from state
// that co-op clients also have (LM state, DSKY, crew.hint), so the LMP sees
// the same advice as the Commander.
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"

#include "agc_host.h"
#include "crew.h"
#include "hints.h"
#include "hud.h"
#include "lm.h"

int hints_on = 1;

#define FT 0.3048
#define MAXL 3

// ---------------------------------------------------------------------------
// Settings file: one "hints=0|1" line.

static void cfg_path(char *buf, size_t n)
{
#ifdef _WIN32
    const char *base = getenv("APPDATA");
    snprintf(buf, n, "%s\\agc-lander.conf", base ? base : ".");
#else
    const char *xdg = getenv("XDG_CONFIG_HOME"), *home = getenv("HOME");
    if (xdg && xdg[0]) snprintf(buf, n, "%s/agc-lander.conf", xdg);
    else snprintf(buf, n, "%s/.config/agc-lander.conf", home ? home : ".");
#endif
}

void hints_load(void)
{
    char p[512];
    cfg_path(p, sizeof p);
    FILE *f = fopen(p, "r");
    if (!f) return;
    char line[128];
    while (fgets(line, sizeof line, f))
        if (!strncmp(line, "hints=", 6)) hints_on = atoi(line + 6) != 0;
    fclose(f);
}

void hints_save(void)
{
    char p[512];
    cfg_path(p, sizeof p);
    FILE *f = fopen(p, "w");
    if (!f) return;
    fprintf(f, "hints=%d\n", hints_on);
    fclose(f);
}

// ---------------------------------------------------------------------------

static int prog_is(int p)
{
    const DskyState *d = agc_host_dsky();
    return agc_digit_char(d->prog[0]) == '0' + p / 10 && agc_digit_char(d->prog[1]) == '0' + p % 10;
}

typedef struct { char line[MAXL][160]; int n; int urgent; } Hint;

static void add(Hint *h, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void add(Hint *h, const char *fmt, ...)
{
    if (h->n >= MAXL) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(h->line[h->n++], sizeof h->line[0], fmt, ap);
    va_end(ap);
}

// Seconds of descent-engine propellant at the current thrust (Isp ~305 s).
static double fuel_seconds(void)
{
    double flow = lm.thrust / (305.0 * 9.80665);
    return flow > 0.5 ? lm.m_dps / flow : 1e9;
}

// Hand-flying advice for P66 (and any time below 150 m with the engine on).
static void flying_hints(Hint *h)
{
    double alt = lm.altitude;
    double fs = fuel_seconds();
    if (fs < 60) {
        add(h, "Fuel: about %.0f seconds left. Get it on the ground now!", fs);
        h->urgent = 1;
    }

    // Vertical: aim for a descent rate that shrinks as the ground gets closer.
    double target = fmax(0.6, fmin(4.5, alt / 12.0));      // m/s down
    double down = -lm.hdot;
    double diff_ft = (down - target) / FT;                  // + = too fast
    if (alt < 4)
        add(h, "Almost down: hold steady and wait for the blue CONTACT light");
    else if (down > target + 0.8)
        add(h, "Coming down too fast (%.0f ft/s): tap Up about %d times to slow down",
            down / FT, (int)lround(fmin(15, diff_ft)));
    else if (down < target * 0.4 - 0.15)
        add(h, lm.hdot > 0.2 ? "You're climbing: tap Down about %d times to descend"
                             : "Barely descending: tap Down about %d times",
            (int)lround(fmin(15, fmax(1, -diff_ft))));

    // Horizontal drift, in the LM's own forward/right directions.
    V3 up = v3_norm(lm.r);
    V3 vs = lm_surface_velocity();
    V3 vh = v3_sub(vs, v3_scale(up, v3_dot(vs, up)));
    V3 fwd = lm_body_to_inertial(v3(0, 0, 1)), right = lm_body_to_inertial(v3(0, 1, 0));
    fwd = v3_norm(v3_sub(fwd, v3_scale(up, v3_dot(fwd, up))));
    right = v3_norm(v3_sub(right, v3_scale(up, v3_dot(right, up))));
    double vf = v3_dot(vh, fwd), vr = v3_dot(vh, right);
    double lim = alt < 20 ? 0.35 : 0.8;
    double tilt = lm_tilt();
    V3 up_b = lm_inertial_to_body(up);   // +z: leaning back, -y: leaning right

    if (fabs(vf) > lim && fabs(vf) >= fabs(vr))
        add(h, vf > 0 ? "Drifting forward (%.0f ft/s): hold W briefly to lean back, then let go"
                      : "Drifting backward (%.0f ft/s): hold S briefly to lean forward, then let go",
            fabs(vf) / FT);
    else if (fabs(vr) > lim)
        add(h, vr > 0 ? "Drifting right (%.0f ft/s): hold A briefly to lean left, then let go"
                      : "Drifting left (%.0f ft/s): hold D briefly to lean right, then let go",
            fabs(vr) / FT);
    else if (tilt > 10 * DEG2RAD) {
        if (fabs(up_b.z) >= fabs(up_b.y))
            add(h, up_b.z > 0 ? "Leaning back %.0f deg: tap S to level out" : "Leaning forward %.0f deg: tap W to level out",
                tilt * RAD2DEG);
        else
            add(h, up_b.y < 0 ? "Leaning right %.0f deg: tap A to level out" : "Leaning left %.0f deg: tap D to level out",
                tilt * RAD2DEG);
    }
    if (tilt > LAND_CRASH_TILT * 0.8 && alt < 30) h->urgent = 1;
    if (down > LAND_CRASH_V * 0.8 && alt < 20) h->urgent = 1;
}

static void compute(Hint *h)
{
    memset(h, 0, sizeof *h);
    if (lm.land_result == LAND_CRASH) return;   // the results panel takes over
    if (lm.touchdown) {
        if (lm.dps_on) { add(h, "You're on the ground: press SPACE to shut the engine off"); h->urgent = 1; }
        return;
    }
    if (lm.probe_contact && lm.dps_on) {
        add(h, "Blue CONTACT light! Press SPACE now to cut the engine");
        h->urgent = 1;
        return;
    }

    // The computer is asking for something.
    if (!crew.auto_crew) {
        switch (crew.hint) {
        case HINT_FINE_ALIGN:
            add(h, "The computer asks if it should fine-tune its gyros. Not needed:");
            add(h, "press Enter to skip");
            return;
        case HINT_MANEUVER:
            add(h, "The computer wants to turn the LM to the burn attitude.");
            add(h, "Make sure F8 says AUTO, then press P (PRO) to let it turn");
            return;
        case HINT_MANEUVER_DONE:
            add(h, "Turn finished. Press Enter to carry on");
            return;
        case HINT_ARM:
            add(h, "Press F7 to arm the descent engine before the burn starts");
            return;
        case HINT_ENABLE_IGN:
            add(h, "The engine is ready to light. Press P (PRO) to allow ignition");
            if (!lm.engine_armed) add(h, "...but arm it first with F7");
            return;
        case HINT_V57:
            add(h, "The landing radar can see the ground. Tell the computer to use it:");
            add(h, "type V 5 7 then Enter");
            return;
        case HINT_ALARM:
            add(h, "Program alarm: the computer is busy but still flying.");
            add(h, "Press R (RSET) to clear the light");
            return;
        }
    }

    int p63 = prog_is(63), p64 = prog_is(64), p65 = prog_is(65), p66 = prog_is(66);
    if (p66 || (lm.dps_on && lm.altitude < 150 && lm_get_attitude_mode() != 2)) {
        flying_hints(h);
        if (h->n == 0) add(h, "Looking good. Keep it slow and level");
        return;
    }
    if (p65) {
        add(h, "The computer is landing for you. Watch for the blue CONTACT light%s",
            crew.auto_crew ? "" : ", then press SPACE");
        return;
    }
    if (p64) {
        add(h, "The computer is steering to the landing site. Let it fly,");
        add(h, "or take over: press F8 (ATT HOLD), then tap Up/Down");
        return;
    }
    if (p63 && lm.dps_on) {
        add(h, "Braking burn: the computer is slowing you down from orbit.");
        add(h, "Nothing to do yet. PgUp speeds up time");
        return;
    }
    if (p63) {
        add(h, crew.auto_crew ? "Waiting for the burn. The auto crew runs the checklist (F2 to do it yourself)."
                              : "Waiting for the computer. Press F2 if you want the auto crew to do the checklist.");
        add(h, "PgUp speeds up time");
    }
}

void hints_draw(int w, int h, int dsky_w)
{
    if (!hints_on) return;
    Hint hint;
    compute(&hint);
    if (hint.n == 0) return;

    float scale = h / 1080.0f;
    if (scale < 0.7f) scale = 0.7f;
    float size = 22 * scale, lh = 30 * scale;
    float maxw = 0;
    for (int i = 0; i < hint.n; i++) {
        Vector2 m = MeasureTextEx(font_ui, hint.line[i], size, size * 0.02f);
        if (m.x > maxw) maxw = m.x;
    }
    float cx = (w - dsky_w) / 2.0f;
    float bw = maxw + 40 * scale, bh = hint.n * lh + 40 * scale;
    float x = cx - bw / 2, y = 150 * scale;
    int flash = hint.urgent && ((int)(GetTime() * 3) & 1);
    Color bg = hint.urgent ? (Color){ 60, 12, 10, 215 } : (Color){ 8, 30, 22, 205 };
    Color edge = hint.urgent ? (Color){ 255, 90, 70, 255 } : (Color){ 90, 220, 150, 255 };
    DrawRectangleRounded((Rectangle){ x, y, bw, bh }, 0.12f, 8, bg);
    DrawRectangleRoundedLinesEx((Rectangle){ x, y, bw, bh }, 0.12f, 8, flash ? 3.0f : 1.5f, edge);
    hud_text(font_bold, "HINT", x + 14 * scale, y + 6 * scale, 15 * scale, edge);
    hud_text(font_ui, "H hides", x + bw - 62 * scale, y + 6 * scale, 14 * scale, (Color){ 150, 160, 170, 200 });
    for (int i = 0; i < hint.n; i++)
        hud_text(font_ui, hint.line[i], x + 20 * scale, y + 26 * scale + i * lh, size, (Color){ 235, 240, 236, 255 });
}
