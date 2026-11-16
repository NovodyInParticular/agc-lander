// agc-lab: headless experiment runner for the AGC + LM simulation.
//
// Usage: agc-lab [scenario] cmd cmd ...
//   keys:V16N20E   type DSKY keys (V N E R C + - P=pro press p=pro release 0-9)
//   run:SECONDS    advance simulation
//   show           print DSKY and vehicle state
//   rate:p,q,r     set body rates, deg/s (X yaw, Y pitch, Z roll)
//   att:yaw,pitch,roll  set attitude relative to the stable member, deg
//   mode:N         attitude mode 0 off / 1 hold / 2 auto
//   arm:0|1        DPS engine arm
//   watch:SECS,EVERY  run and print a compact line every EVERY seconds
//   pdi            set up the Apollo 11 pre-PDI scenario (scenario.c)
//   liftoff        Apollo 11 on the surface, P12 one minute before lift-off
//   stage          push ABORT STAGE
//   orbit          print the orbit (perilune / apolune) and the CSM's range
//   ascent:SECS    run, printing ascent lines every 10 s
//   rndz           the rendezvous shortcut: Eagle 40 m from Columbia
//   dock:METRES    lined-up start at that range (no offsets)
//   dockpilot:SECS fly the docking with the hand controllers (THC pulses, RHC)
//   rhc:p,y,r      hold the attitude controller (as before); thc:x,y,z hold the translation controller
//   terrain        fly over the game's terrain instead of a smooth sphere (after pdi / liftoff)
//   snapgen:DIR    the canonical descent: milestone save states + track into DIR (make snapshots)
//   snapsave:FILE  save state now; snapload:FILE restore one (also a milestone name: snapload:HIGH GATE)
//   nextevent      is the flight on the canonical track, and which event can it advance to?
//   advance        advance to the next event if offered (prints ADVANCE <name> or ADVANCE none)
//   plane          the LM's position on the terrain's plane (m east, m south of the site), heading of travel
//   rrfault:PPS    rendezvous radar CDU dither (Apollo 11's 1202 alarms: 6400)
//   alarms:SECS    run with the crew, printing every program alarm (PROG lamp, FAILREG)

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "agc_host.h"
#include "agc_mem.h"
#include "lm.h"
#include "moon.h"
#include "scenario.h"
#include "crew.h"
#include "csm.h"
#include "ascent.h"
#include "milestones.h"
#include "sim.h"
#include "snapshot.h"
#include "terrain.h"
#include "terrain_shape.h"

#define DEG (PI_D / 180.0)

static void type_keys(const char *s)
{
    for (; *s; s++) {
        switch (*s) {
        case 'V': agc_host_key(KEY_AGC_VERB); break;
        case 'N': agc_host_key(KEY_AGC_NOUN); break;
        case 'E': agc_host_key(KEY_AGC_ENTR); break;
        case 'R': agc_host_key(KEY_AGC_RSET); break;
        case 'C': agc_host_key(KEY_AGC_CLR); break;
        case 'K': agc_host_key(KEY_AGC_KEYREL); break;
        case '+': agc_host_key(KEY_AGC_PLUS); break;
        case '-': agc_host_key(KEY_AGC_MINUS); break;
        case 'P': agc_host_pro(1); break;
        case 'p': agc_host_pro(0); break;
        case '0': agc_host_key(KEY_AGC_0); break;
        default:
            if (*s >= '1' && *s <= '9') agc_host_key(*s - '0');
        }
        lm_run(*s == 'P' ? 0.4 : 0.12);
    }
}

static void dsky_str(char *buf, size_t n)
{
    const DskyState *d = agc_host_dsky();
    char r[3][7];
    for (int i = 0; i < 3; i++) {
        r[i][0] = (d->sign[i] & 2) ? '+' : (d->sign[i] & 1) ? '-' : ' ';
        for (int j = 0; j < 5; j++) r[i][j + 1] = agc_digit_char(d->reg[i][j]);
        r[i][6] = 0;
    }
    snprintf(buf, n, "P%c%c V%c%c N%c%c %s %s %s%s%s%s%s%s%s",
             agc_digit_char(d->prog[0]), agc_digit_char(d->prog[1]),
             agc_digit_char(d->verb[0]), agc_digit_char(d->verb[1]),
             agc_digit_char(d->noun[0]), agc_digit_char(d->noun[1]), r[0], r[1], r[2],
             d->prog_lamp ? " PROG" : "", d->opr_err ? " OPRERR" : "", d->restart ? " RESTART" : "",
             d->no_att ? " NOATT" : "", d->alt ? " ALT" : "", d->vel ? " VEL" : "");
}

static void show(void)
{
    char d[200];
    dsky_str(d, sizeof d);
    V3 g = lm_gimbal_angles();
    printf("t=%9.2f  DSKY %s\n", lm.t, d);
    printf("   gimbal OGA %8.3f IGA %8.3f MGA %8.3f deg   rates %.3f %.3f %.3f deg/s\n",
           g.x / DEG, g.y / DEG, g.z / DEG, lm.w.x / DEG, lm.w.y / DEG, lm.w.z / DEG);
    printf("   CDU regs X %8.3f Y %8.3f Z %8.3f deg  (pending %d %d %d)\n",
           agc_get_word("CDUX", 0) * 360.0 / 32768, agc_get_word("CDUY", 0) * 360.0 / 32768,
           agc_get_word("CDUZ", 0) * 360.0 / 32768, agc_host_pending(032), agc_host_pending(033), agc_host_pending(034));
    printf("   alt %.1f m  hdot %.2f  hspd %.2f  thrust %.0f N  mass %.0f kg  dps %d  jets %03o %03o  LR %s%s pos%d\n",
           lm.altitude, lm.hdot, lm.hspeed, lm.thrust, lm_mass(), lm.dps_on, lm.ch5, lm.ch6,
           lm.lr_range_good ? "R" : "-", lm.lr_vel_good ? "V" : "-", lm.lr_pos);
}

// The game's automatic crew (crew.c); callouts are echoed to stdout.
static int crew_on;
static double dp_sign = -1, dr_sign = 1;   // docking pilot: pitch / roll response signs (dpsign:, drsign:)
static double last_callout;

static void crew_tick(void)
{
    ascent_step(0.1);
    if (!crew_on) return;
    crew_step();
    if (crew.callout_time != last_callout) {
        last_callout = crew.callout_time;
        printf("   [crew] %s (alt %.1f m)\n", crew.callout, lm.altitude);
    }
}

static void orbit_report(void)
{
    V3 r = lm.r, v = lm.v;
    double rr = v3_len(r), vv = v3_len(v);
    double en = vv * vv / 2 - MOON_MU / rr;
    double a = -MOON_MU / (2 * en);
    V3 h = v3_cross(r, v);
    double e = sqrt(fmax(0, 1 + 2 * en * v3_dot(h, h) / (MOON_MU * MOON_MU)));
    double R0 = 1738000.0;
    printf("ORBIT LM perilune %.1f km apolune %.1f km (above 1738 km)  |v| %.1f m/s  staged %d liftoff %d  APS left %.0f kg  RCS %.0f kg\n",
           (a * (1 - e) - R0) / 1000, (a * (1 + e) - R0) / 1000, vv, lm.staged, lm.liftoff, lm.m_aps, lm.m_rcs);
    if (csm.valid) {
        double rc = v3_len(csm.r);
        double phase = atan2(v3_dot(v3_cross(r, csm.r), v3_norm(h)), v3_dot(r, csm.r)) / DEG;
        V3 n_c = v3_norm(v3_cross(csm.r, csm.v));
        printf("ORBIT CSM alt %.1f km  range %.1f km  phase (CSM ahead) %.2f deg  LM out of CSM plane %.2f km  wedge %.3f deg\n",
               (rc - R0) / 1000, v3_len(v3_sub(csm.r, r)) / 1000, phase, v3_dot(r, n_c) / 1000,
               acos(fmin(1, v3_dot(n_c, v3_norm(h)))) / DEG);
    }
}

static void dock_report(void)
{
    DockGeo g;
    if (!ascent_dock_geo(&g)) { printf("DOCK no CSM\n"); return; }
    const AscentStats *st = ascent_stats();
    printf("DOCK t=%.1f phase %d state %d axial %.2f lat %+.3f %+.3f rate %+.3f drift %+.3f %+.3f angle %.2f roll %+.2f rcs %.0f\n",
           lm.t, ascent_phase(), st->dock_state, g.axial, g.lat_y, g.lat_z, g.rate, g.vy, g.vz, g.angle / DEG, g.roll / DEG, lm.m_rcs);
    if (st->dock_state >= 1)
        printf("DOCKED capture speed %.3f m/s radial %.3f m angle %.2f deg roll %+.2f deg bounces %d result %s\n", st->c_speed, st->c_radial,
               st->c_angle / DEG, st->c_roll / DEG, st->bounces, st->dock_state == 2 ? "hard" : st->dock_state == 3 ? "FAILED" : "capture");
    if (st->dock_state == 3) printf("DOCKFAIL %s\n", st->fail);
}

// A simple docking pilot using only what a player has: translation pulses
// (THC) and the attitude controller in ATT HOLD.
static void dock_pilot(double secs)
{
    int tick = 0;
    for (double t = 0; t < secs; t += 0.1, tick++) {
        lm_run(0.1);
        crew_tick();
        DockGeo g;
        if (!ascent_dock_geo(&g) || ascent_stats()->dock_state) { lm_set_thc(0, 0); lm_set_thc(1, 0); lm_set_thc(2, 0); agc_host_set_rhc(0, 0, 0); continue; }
        // Attitude: roll about the docking axis with yaw (Q = +yaw lowers the roll error);
        // the tilt with pitch / roll, from where Columbia's axis points in LM axes.
        V3 d = m3_apply(m3_transpose(q4_to_m3(lm.q)), m3_apply(q4_to_m3(csm.q), v3(-1, 0, 0)));
        double yaw = fmax(-0.5, fmin(0.5, g.roll / DEG * 0.08));
        double pitch = fmax(-0.5, fmin(0.5, dp_sign * d.z / DEG * 0.08));
        double roll = fmax(-0.5, fmin(0.5, dr_sign * d.y / DEG * 0.08));
        if (fabs(g.roll) < 0.4 * DEG) yaw = 0;
        if (fabs(d.z) < 0.4 * DEG) pitch = 0;
        if (fabs(d.y) < 0.4 * DEG) roll = 0;
        agc_host_set_rhc(pitch, yaw, roll);
        // Translation: pulse on alternate ticks toward the wanted velocities.
        int on = tick & 1;
        double vy_des = fmax(-0.08, fmin(0.08, -0.15 * g.lat_y)), vz_des = fmax(-0.08, fmin(0.08, -0.15 * g.lat_z));
        int ty = !on ? 0 : g.vy > vy_des + 0.012 ? 1 : g.vy < vy_des - 0.012 ? -1 : 0;
        int tz = !on ? 0 : g.vz > vz_des + 0.012 ? 1 : g.vz < vz_des - 0.012 ? -1 : 0;
        double lat = sqrt(g.lat_y * g.lat_y + g.lat_z * g.lat_z);
        double r_des = lat < 0.15 && g.angle < 2.5 * DEG && fabs(g.roll) < 4 * DEG ? (g.axial > 8 ? -0.25 : -0.12) : (g.axial > 6 ? -0.05 : 0.0);
        int tx = !on ? 0 : g.rate > r_des + 0.02 ? 1 : g.rate < r_des - 0.03 ? -1 : 0;
        lm_set_thc(0, tx); lm_set_thc(1, ty); lm_set_thc(2, tz);
        if (tick % 50 == 0) dock_report();
    }
    lm_set_thc(0, 0); lm_set_thc(1, 0); lm_set_thc(2, 0);
    agc_host_set_rhc(0, 0, 0);
    dock_report();
}

static void ascent_watch(double secs)
{
    for (double t = 0; t < secs; t += 10) {
        for (double k = 0; k < 10 - 1e-9; k += 0.1) {
            lm_run(0.1);
            crew_tick();
        }
        char d[200];
        dsky_str(d, sizeof d);
        V3 up = v3_norm(lm.r);
        double vh = v3_len(v3_sub(lm.v, v3_scale(up, v3_dot(lm.v, up))));
        printf("t=%8.1f alt %8.0f m hdot %7.1f vh %7.1f F %6.0f aps %d m %5.0f tilt %5.1f | %s\n",
               lm.t, v3_len(lm.r) - 1738000.0 + 0 * vh, v3_dot(lm.v, up), vh, lm.thrust, lm.aps_on, lm_mass(),
               lm_tilt() / DEG, d);
        fflush(stdout);
    }
}

static void watch(double secs, double every)
{
    for (double t = 0; t < secs; t += every) {
        for (double k = 0; k < every - 1e-9; k += 0.1) {
            lm_run(every - k < 0.1 ? every - k : 0.1);
            crew_tick();
        }
        char d[200];
        dsky_str(d, sizeof d);
        printf("t=%8.1f alt %9.1f hdot %7.2f hspd %7.1f F %6.0f m %6.0f | %s\n",
               lm.t, lm.altitude, lm.hdot, lm.hspeed, lm.thrust, lm_mass(), d);
        fflush(stdout);
    }
}

static int write_file(const char *path, const uint8_t *b, size_t n)
{
    FILE *f = fopen(path, "wb");
    if (!f || fwrite(b, 1, n, f) != n) { fprintf(stderr, "cannot write %s\n", path); if (f) fclose(f); return -1; }
    return fclose(f);
}

// The canonical descent for the save states (make snapshots). The game's
// start (start_world, the automatic crew, the terrain under the radar), with
// milestones saved at the moment their condition first holds and a sample of
// the flight every second.
static const char *snap_file[MS_COUNT] = { "tig30", "radar", "highgate", "150m", "boulders", "site2" };

// Move Luminary's landing site to a spot on the terrain plane, as Apollo 12's
// Noun 69 landing site update did: P63's guidance target LAND (stable-member
// frame, set from RLS when P63 started) shifts by the same amount, and RLS
// follows. The results measure from the new site; the terrain stays anchored
// where it was (scenario_info.rls).
static void retarget(double x, double z, double site_x, double site_z)
{
    V3 old = agc_get_vec("RLS", 27);
    V3 rls = v3_scale(terrain_plane_to_dir(x, z), v3_len(old));
    double t = scenario_info.tland + lm.tephem;
    M3 sm = scenario_info.refsmmat;
    V3 land = agc_get_vec("LAND", 24);
    V3 check = m3_apply(sm, moon_fixed_to_inertial(old, t));
    V3 d = m3_apply(sm, v3_sub(moon_fixed_to_inertial(rls, t), moon_fixed_to_inertial(old, t)));
    agc_set_vec("LAND", v3_add(land, d), 24);
    agc_set_vec("RLS", rls, 27);
    double lat, lon, rad;
    moon_fixed_to_latlon(terrain_plane_to_dir(site_x, site_z), &lat, &lon, &rad);
    scenario_info.site_lat = lat;
    scenario_info.site_lon = lon;
    printf("SNAPGEN   retarget %.0f m; LAND from RLS reproduced to %.1f m\n", v3_len(d), v3_len(v3_sub(check, land)));
}

static void start_flight(void)
{
    agc_host_init();
    lm.surface = NULL;
    lm_reset();
    lm_attach_agc();
    crew_reset();
    ascent_reset();
    scenario_pre_pdi();
    terrain_shape_init(scenario_info.rls);
    lm.surface = terrain_surface_cb;
    crew.auto_crew = 1;
}

// A scenario from its own flight: retargeted at PDI-15, saved when the
// descent passes alt_m; fuel_s > 0 leaves that many seconds of hovering
// propellant (the AGC's mass estimate follows).
// Where the canonical flight touched down relative to its target (its
// navigation error, much as Apollo 11 landed long); the scenario flights aim
// that much short so they come down where they are meant to.
static double miss_x, miss_z;

// How surely an upright LM set down at x, z comes down on a boulder: the
// least, over its spots within 3 m, of how far the worst block reaches into
// the engine bell or the stage base (m, positive = a strike). As in lm.c.
static double boulder_hazard(double x, double z)
{
    double worst = 1e9;
    for (double ox = -3; ox <= 3; ox += 1.5)
        for (double oz = -3; oz <= 3; oz += 1.5) {
            double hit = -1e9, cx = x + ox, cz = z + oz;
            for (int k = 0; k < 9; k++) {
                static const double p[9][3] = { { 0.55, 0, 0 }, { 0.55, 0.75, 0 }, { 0.55, -0.75, 0 }, { 0.55, 0, 0.75 }, { 0.55, 0, -0.75 },
                                                { 1.15, 1.9, 1.9 }, { 1.15, 1.9, -1.9 }, { 1.15, -1.9, 1.9 }, { 1.15, -1.9, -1.9 } };
                double h = terrain_rock_height(cx + p[k][1], cz + p[k][2]) - p[k][0];
                if (h > hit) hit = h;
            }
            if (hit < worst) worst = hit;
        }
    return worst;
}

static int snapgen_flight(const char *dir, int ms, double x, double z, double alt_m, double fuel_s, int onto_boulders)
{
    double aim_x = x - miss_x, aim_z = z - miss_z;
    if (onto_boulders) {
        // The block-strewn spot nearest the target where the computer would set
        // Eagle down on a boulder; one test flight trims the aim onto it.
        double best = -1e9, bx = x, bz = z;
        for (double dx = -40; dx <= 40; dx += 1)
            for (double dz = -40; dz <= 40; dz += 1) {
                double hz = boulder_hazard(x + dx, z + dz) - 0.002 * sqrt(dx * dx + dz * dz);
                if (hz > best) { best = hz; bx = x + dx; bz = z + dz; }
            }
        x = bx;
        z = bz;
        aim_x = x - miss_x;
        aim_z = z - miss_z;
        start_flight();
        retarget(aim_x, aim_z, x, z);
        while (!lm.touchdown && lm.t < 371000) sim_advance(0.5);
        double lx, lz;
        terrain_site_to_plane(terrain_fixed_to_site(moon_inertial_to_fixed(lm.r, lm.t + lm.tephem)), &lx, &lz);
        aim_x += x - lx;
        aim_z += z - lz;
        printf("SNAPGEN   boulder spot %.0f, %.0f (blocks %.2f m into the stage); test flight landed %.1f m off it\n", x, z, best,
               sqrt((x - lx) * (x - lx) + (z - lz) * (z - lz)));
    }
    start_flight();
    retarget(aim_x, aim_z, x, z);
    double t0 = lm.t, t_down = 0;
    int saved = 0;
    char path[512];
    while (lm.t - t0 < 2400) {
        sim_advance(0.1);
        if (!saved && lm.dps_on && lm.altitude < alt_m) {
            if (fuel_s > 0) {
                double hover = lm_mass() * 1.62 / (305.0 * 9.80665);
                double cut = lm.m_dps - fuel_s * hover;
                lm.m_dps -= cut;
                agc_set_dp("MASS", 0, agc_get_dp("MASS", 0, 16) - cut, 16);
            }
            uint8_t *b;
            size_t n = snapshot_save(&b, milestone_name(ms), ms);
            snprintf(path, sizeof path, "%s/%s.snap", dir, snap_file[ms]);
            if (!n || write_file(path, b, n)) return 1;
            free(b);
            saved = 1;
            double px, pz;
            terrain_site_to_plane(terrain_fixed_to_site(moon_inertial_to_fixed(lm.r, lm.t + lm.tephem)), &px, &pz);
            printf("SNAPGEN %-9s t=%.1f (TIG%+.1f s)  alt %7.0f m  at %.0f, %.0f (target %.0f, %.0f)  DPS %.0f kg  %zu bytes\n",
                   milestone_name(ms), lm.t, lm.t - 369645, lm.altitude, px, pz, x, z, lm.m_dps, n);
        }
        if (lm.touchdown && !t_down) t_down = lm.t;
        if (t_down && lm.t - t_down > 5) break;
    }
    if (!saved) { fprintf(stderr, "SNAPGEN: %s never reached\n", milestone_name(ms)); return 1; }
    double px, pz;
    terrain_site_to_plane(terrain_fixed_to_site(moon_inertial_to_fixed(lm.r, lm.t + lm.tephem)), &px, &pz);
    static const char *res[] = { "none", "good", "hard", "crash" };
    printf("SNAPGEN   the automatic crew: %s%s at %.0f, %.0f, %.2f m/s, tilt %.1f deg, DPS %.0f kg\n", res[lm.land_result],
           lm.land_result == LAND_CRASH && lm.crash_cause == CRASH_BOULDER ? " (on a boulder)" : "", px, pz,
           -lm.touchdown_speed, lm.touchdown_tilt * 180 / PI_D, lm.m_dps);
    return 0;
}

static int snapgen(const char *dir)
{
    agc_host_init();
    lm.surface = NULL;
    lm_reset();
    lm_attach_agc();
    crew_reset();
    ascent_reset();
    scenario_pre_pdi();
    terrain_shape_init(scenario_info.rls);
    lm.surface = terrain_surface_cb;
    crew.auto_crew = 1;

    int have[MS_COUNT] = { 0 }, fails = 0;
    double t_ms[MS_COUNT] = { 0 }, last_action = -1e9, action_before[MS_COUNT];
    int ntrk = 0, cap = 4096;
    TrackSample *trk = malloc(cap * sizeof *trk);
    track_sample_now(&trk[ntrk++], 0);
    double t0 = lm.t, t_down = 0;
    int acts = crew_action_count(), acted = 0;
    char path[512];
    for (long step = 1; lm.t - t0 < 2400; step++) {
        int fire[MS_COUNT] = { 0 };
        lm_run(0.1);
        // Before the crew reacts: moments when the player has something to do.
        TrackSample now;
        track_sample_now(&now, 0);
        fire[MS_RADAR] = now.prog == 63 && lm.lr_range_good && lm.dps_on;
        for (int pass = 0; pass < 2; pass++) {
            if (pass == 1) {
                crew_step();
                ascent_step(0.1);
                int a = crew_action_count();
                if (a != acts) {
                    acts = a;
                    acted = 1;
                    last_action = lm.t;
                    printf("SNAPGEN   crew acts t=%.1f (TIG%+.1f s)  %s\n", lm.t, lm.t - 369645, crew.callout_time == lm.t ? crew.callout : "");
                }
                // After the slice, as the old fast-forward checked.
                fire[MS_TIG30] = lm.t > 369615;
                fire[MS_HIGHGATE] = lm.dps_on && lm.altitude < 2600;
                fire[MS_150M] = lm.dps_on && lm.altitude < 150;
            }
            for (int m = 0; m < MS_BOULDERS; m++) {
                if (have[m] || !fire[m]) continue;
                uint8_t *b;
                size_t n = snapshot_save(&b, milestone_name(m), m);
                snprintf(path, sizeof path, "%s/%s.snap", dir, snap_file[m]);
                if (!n || write_file(path, b, n)) return 1;
                free(b);
                have[m] = 1;
                t_ms[m] = lm.t;
                action_before[m] = last_action;
                printf("SNAPGEN %-9s t=%.1f (TIG%+.1f s)  alt %7.0f m  P%02d V%02dN%02d  %zu bytes\n", milestone_name(m), lm.t,
                       lm.t - 369645, lm.altitude, now.prog, now.verb, now.noun, n);
            }
        }
        if (step % 10 == 0) {
            if (ntrk == cap) trk = realloc(trk, (cap *= 2) * sizeof *trk);
            track_sample_now(&trk[ntrk++], acted);
            acted = 0;
        }
        if (lm.touchdown && !t_down) t_down = lm.t;
        if (t_down && lm.t - t_down > 5) break;
    }
    for (int m = 0; m < MS_BOULDERS; m++)   // the canonical flight's own milestones
        if (!have[m]) { fprintf(stderr, "SNAPGEN: milestone %s never reached\n", milestone_name(m)); fails++; }
    // Advance targets must have a quiet lead-in (the track marks actions only to the second).
    int targets[] = { MS_TIG30, MS_RADAR, MS_HIGHGATE };
    for (int k = 0; k < 3; k++) {
        int m = targets[k];
        if (have[m] && t_ms[m] - action_before[m] < 2.0) {
            fprintf(stderr, "SNAPGEN: the crew acted %.1f s before %s\n", t_ms[m] - action_before[m], milestone_name(m));
            fails++;
        }
    }
    uint8_t *b;
    size_t n = track_encode(trk, ntrk, &b);
    snprintf(path, sizeof path, "%s/track.bin", dir);
    if (!n || write_file(path, b, n)) return 1;
    printf("SNAPGEN track %d samples (%.0f s), %zu bytes; touchdown t=%.1f (TIG%+.1f s), %s\n", ntrk, trk[ntrk - 1].t - trk[0].t, n,
           t_down, t_down - 369645, lm.land_result == 1 ? "good" : lm.land_result == 2 ? "hard" : "crash");
    free(b);
    free(trk);
    if (fails) return fails;
    terrain_site_to_plane(terrain_fixed_to_site(moon_inertial_to_fixed(lm.r, lm.t + lm.tephem)), &miss_x, &miss_z);
    printf("SNAPGEN   canonical touchdown %.0f m east, %.0f m south of the target: the scenario flights aim that much short\n", miss_x, miss_z);
    if (snapgen_flight(dir, MS_BOULDERS, BOULDERS_X, BOULDERS_Z, 150, 100, 1)) return 1;
    if (snapgen_flight(dir, MS_SITE2, SITE2_X, SITE2_Z, 2600, 0, 0)) return 1;
    return 0;
}

static int load_state(const char *arg)
{
    char err[200];
    for (int m = 0; m < MS_COUNT; m++)
        if (!strcmp(arg, milestone_name(m))) {
            if (milestone_load(m, err, sizeof err)) { fprintf(stderr, "snapload: %s\n", err); return 1; }
            // Milestones were flown over the terrain: their altitudes are above it.
            terrain_shape_init(scenario_info.rls);
            lm.surface = terrain_surface_cb;
            return 0;
        }
    FILE *f = fopen(arg, "rb");
    if (!f) { fprintf(stderr, "snapload: cannot open %s\n", arg); return 1; }
    static uint8_t buf[1 << 20];
    size_t n = fread(buf, 1, sizeof buf, f);
    fclose(f);
    if (snapshot_load(buf, n, NULL, err, sizeof err)) { fprintf(stderr, "snapload: %s\n", err); return 1; }
    return 0;
}

int main(int argc, char **argv)
{
    if (agc_host_init()) return 1;
    lm_reset();
    lm_attach_agc();

    for (int i = 1; i < argc; i++) {
        char *a = argv[i];
        if (!strncmp(a, "keys:", 5)) type_keys(a + 5);
        else if (!strncmp(a, "run:", 4)) lm_run(atof(a + 4));
        else if (!strcmp(a, "show")) show();
        else if (!strncmp(a, "rate:", 5)) {
            double x, y, z;
            sscanf(a + 5, "%lf,%lf,%lf", &x, &y, &z);
            lm.w = v3(x * DEG, y * DEG, z * DEG);
        } else if (!strncmp(a, "att:", 4)) {
            double x, y, z;
            sscanf(a + 4, "%lf,%lf,%lf", &x, &y, &z);
            Q4 q = q4_mul(q4_axis_angle(v3(1, 0, 0), x * DEG),
                          q4_mul(q4_axis_angle(v3(0, 1, 0), y * DEG), q4_axis_angle(v3(0, 0, 1), z * DEG)));
            lm.q = q4_normalize(q4_mul(lm.q_sm, q));
        } else if (!strncmp(a, "mode:", 5)) lm_set_attitude_mode(atoi(a + 5));
        else if (!strncmp(a, "arm:", 4)) lm_set_engine_arm(atoi(a + 4));
        else if (!strncmp(a, "watch:", 6)) {
            double s, e;
            sscanf(a + 6, "%lf,%lf", &s, &e);
            watch(s, e);
        } else if (!strcmp(a, "pdi")) {
            scenario_pre_pdi();
            double lat, lon, rad;
            moon_fixed_to_latlon(moon_inertial_to_fixed(lm.r, lm.t + lm.tephem), &lat, &lon, &rad);
            printf("PDI scenario: t=%.2f s (GET %02d:%02d:%05.2f) tephem=%.0f s  SV age %.0f s\n", lm.t,
                   (int)(lm.t / 3600), (int)(lm.t / 60) % 60, lm.t - 60 * (int)(lm.t / 60), lm.tephem, scenario_info.sv_age);
            printf("  LM |r| %.1f m (lat %.3f lon %.3f)  |v| %.2f m/s  mass %.1f kg (AGC MASS %.1f)\n",
                   v3_len(lm.r), lat / DEG, lon / DEG, v3_len(lm.v), lm_mass(), agc_get_dp("MASS", 0, 16));
            printf("  site lat %.4f lon %.4f radius %.1f m   TLAND %.2f s\n", scenario_info.site_lat / DEG,
                   scenario_info.site_lon / DEG, scenario_info.site_radius, scenario_info.tland);
        }
        else if (!strncmp(a, "peek:", 5)) printf("   %s = %05o\n", a + 5, agc_get_word(a + 5, 0));
        else if (!strncmp(a, "ch:", 3)) {
            int c = (int)strtol(a + 3, NULL, 8);
            printf("   ch%03o = %05o\n", c, agc_host_read_channel(c));
        }
        else if (!strncmp(a, "rod:", 4)) lm_rod_click(atoi(a + 4));
        else if (!strncmp(a, "rhc:", 4)) {
            double x, y, z;
            sscanf(a + 4, "%lf,%lf,%lf", &x, &y, &z);
            agc_host_set_rhc(x, y, z);
        }
        else if (!strcmp(a, "vb")) {
            V3 vb = lm_inertial_to_body(lm_surface_velocity());
            V3 ub = lm_inertial_to_body(v3_norm(lm.r));
            printf("   body vel X(up) %.2f Y(right) %.2f Z(fwd) %.2f m/s   local up in body %.3f %.3f %.3f\n",
                   vb.x, vb.y, vb.z, ub.x, ub.y, ub.z);
        }
        else if (!strcmp(a, "manual")) crew.auto_crew = 0;
        else if (!strcmp(a, "crew")) { crew_on = 1; crew.auto_crew = 1; }
        else if (!strcmp(a, "landed")) {
            V3 rp = moon_inertial_to_fixed(lm.r, lm.t + lm.tephem);
            double lat, lon, rad;
            moon_fixed_to_latlon(rp, &lat, &lon, &rad);
            double dn = (lat - scenario_info.site_lat) * rad, de = (lon - scenario_info.site_lon) * rad * cos(lat);
            printf("LANDING touchdown=%d speed %.2f m/s at t=%.1f; lat %.4f lon %.4f; %.0f m %s, %.0f m %s of target; DPS left %.0f kg\n",
                   lm.touchdown, -lm.touchdown_speed, lm.t, lat / DEG, lon / DEG, fabs(dn), dn >= 0 ? "N" : "S",
                   fabs(de), de >= 0 ? "E" : "W", lm.m_dps);
            static const char *res[] = { "none", "good", "hard", "crash" };
            printf("RESULT %s cause %d  v %.2f m/s  h %.2f m/s  tilt %.1f deg  tilt now %.1f deg\n", res[lm.land_result],
                   lm.crash_cause, -lm.touchdown_speed, lm.touchdown_hspeed, lm.touchdown_tilt / DEG, lm_tilt() / DEG);
        }
        else if (!strcmp(a, "nav")) {
            V3 rn = agc_get_vec("RN", 29), vn = v3_scale(agc_get_vec("VN", 7), 100.0);
            double pt = agc_get_dp("PIPTIME", 0, 28) / 100.0;
            V3 thrust_i = lm_body_to_inertial(v3(1, 0, 0));
            V3 vrel = lm_surface_velocity();
            double ang = acos(fmax(-1, fmin(1, v3_dot(thrust_i, v3_scale(v3_norm(vrel), -1))))) / DEG;
            double up = asin(v3_dot(thrust_i, v3_norm(lm.r))) / DEG;
            printf("   NAV  AGC r %.1f v %.2f (PIPTIME %.2f)  truth r %.1f v %.2f  |dr| %.1f m  |dv| %.3f m/s\n",
                   v3_len(rn), v3_len(vn), pt, v3_len(lm.r), v3_len(lm.v),
                   v3_len(v3_sub(rn, lm.r)), v3_len(v3_sub(vn, lm.v)));
            printf("   thrust axis vs retrograde %.1f deg, elevation above horizon %.1f deg\n", ang, up);
            {
                extern double lab_lr_debug(V3 *hb_nb_out);
                V3 hb;
                double off = lab_lr_debug(&hb);
                V3 agc_hb = v3_scale(agc_get_vec("HBEAMNB", 1), 2.0);
                printf("   LR beam off-vertical %.1f deg, slant %.0f m, beam-alt %.0f m, true alt %.0f m\n",
                       off, lm.lr_range, lm.lr_alt, lm.altitude);
                printf("   beam NB sim (%.3f %.3f %.3f)  AGC HBEAMNB (%.3f %.3f %.3f)  DELTAH %.1f m\n",
                       hb.x, hb.y, hb.z, agc_hb.x, agc_hb.y, agc_hb.z, agc_get_dp("DELTAH", 0, 24));
            }
            printf("   FAILREG %05o %05o %05o\n", agc_get_word("FAILREG", 0), agc_get_word("FAILREG", 1), agc_get_word("FAILREG", 2));
        }
        else if (!strcmp(a, "align")) lm_align_imu(m3_identity());
        else if (!strcmp(a, "liftoff")) {
            scenario_pre_pdi();       // the landing site, as in the game
            scenario_liftoff(0);
            crew.landed = 1;
            double lat, lon, rad;
            moon_fixed_to_latlon(moon_inertial_to_fixed(lm.r, lm.t + lm.tephem), &lat, &lon, &rad);
            printf("LIFTOFF scenario: t=%.2f s (GET %02d:%02d:%05.2f)  LM lat %.4f lon %.4f r %.1f  mass %.1f kg (AGC MASS %.1f)\n",
                   lm.t, (int)(lm.t / 3600), (int)(lm.t / 60) % 60, lm.t - 60 * (int)(lm.t / 60), lat / DEG, lon / DEG, rad,
                   lm_mass(), agc_get_dp("MASS", 0, 16));
            printf("  TIG %.2f s (in %.1f s)  FLGWRD10 %05o  DAPBOOLS %05o  ch30 %05o\n", agc_get_dp("TIG", 0, 28) / 100.0,
                   agc_get_dp("TIG", 0, 28) / 100.0 - lm.t, agc_get_word("FLGWRD10", 0), agc_get_word("DAPBOOLS", 0),
                   agc_host_read_channel(030));
            orbit_report();
        }
        else if (!strcmp(a, "liftoffkeep")) {
            // After a landing in this run: P12 from where the LM stands (the game's L key).
            scenario_liftoff(1);
            crew_reset();
            crew.auto_crew = crew_on;
            crew.landed = 1;
            ascent_begin();
            V3 g = lm_gimbal_angles();
            printf("LIFTOFF from the landing: gimbals %.2f %.2f %.2f deg, tilt %.1f deg\n", g.x / DEG, g.y / DEG, g.z / DEG, lm_tilt() / DEG);
        }
        else if (!strcmp(a, "stage")) lm_abort_stage();
        else if (!strcmp(a, "rndz")) { if (!ascent_active()) ascent_begin(); ascent_rendezvous(); dock_report(); }
        else if (!strncmp(a, "dock:", 5)) { ascent_debug_dock(atof(a + 5)); dock_report(); }
        else if (!strncmp(a, "dockpilot:", 10)) dock_pilot(atof(a + 10));
        else if (!strcmp(a, "dockreport")) dock_report();
        else if (!strncmp(a, "dpsign:", 7)) dp_sign = atof(a + 7);
        else if (!strncmp(a, "drsign:", 7)) dr_sign = atof(a + 7);
        else if (!strncmp(a, "thc:", 4)) {
            int x, y, z;
            sscanf(a + 4, "%d,%d,%d", &x, &y, &z);
            lm_set_thc(0, x); lm_set_thc(1, y); lm_set_thc(2, z);
        }
        else if (!strcmp(a, "orbit")) orbit_report();
        else if (!strncmp(a, "ascent:", 7)) ascent_watch(atof(a + 7));
        else if (!strcmp(a, "terrain")) { terrain_shape_init(scenario_info.rls); lm.surface = terrain_surface_cb; }
        else if (!strncmp(a, "snapgen:", 8)) { if (snapgen(a + 8)) return 1; }
        else if (!strncmp(a, "snapsave:", 9)) {
            uint8_t *b;
            size_t n = snapshot_save(&b, "lab", -1);
            if (!n || write_file(a + 9, b, n)) return 1;
            free(b);
        }
        else if (!strncmp(a, "snapload:", 9)) { if (load_state(a + 9)) return 1; }
        else if (!strncmp(a, "rrfault:", 8)) agc_host_set_rr_dither(atof(a + 8));
        else if (!strncmp(a, "vec:", 4)) {
            for (int sc = 22; sc <= 30; sc++) {
                V3 v = agc_get_vec(a + 4, sc);
                printf("   %s at B%d: |v| %.1f  (%.1f %.1f %.1f)\n", a + 4, sc, v3_len(v), v.x, v.y, v.z);
            }
        }
        else if (!strncmp(a, "rockmap:", 8)) {
            // Boulder cover around a point: fraction of a 1 m grid with a rock over 0.2 m high, the tallest.
            double cx, cz, rad = 25;
            sscanf(a + 8, "%lf,%lf,%lf", &cx, &cz, &rad);
            int n = 0, hit = 0;
            double top = 0, tx = 0, tz = 0;
            for (double x = cx - rad; x <= cx + rad; x += 1)
                for (double z = cz - rad; z <= cz + rad; z += 1) {
                    double h = terrain_rock_height(x, z);
                    n++;
                    if (h > 0.2) hit++;
                    if (h > top) { top = h; tx = x; tz = z; }
                }
            printf("ROCKMAP around %.0f, %.0f (+-%.0f m): %.1f%% covered by rocks over 0.2 m; tallest %.2f m at %.0f, %.0f\n", cx, cz, rad,
                   100.0 * hit / n, top, tx, tz);
        }
        else if (!strncmp(a, "slope:", 6)) {
            double x, z;
            sscanf(a + 6, "%lf,%lf", &x, &z);
            double d = 3.0, hx = (terrain_height(x + d, z, 0) - terrain_height(x - d, z, 0)) / (2 * d);
            double hz = (terrain_height(x, z + d, 0) - terrain_height(x, z - d, 0)) / (2 * d);
            printf("SLOPE at %.0f, %.0f: %.1f deg (over 6 m)\n", x, z, atan(sqrt(hx * hx + hz * hz)) * 180 / PI_D);
        }
        else if (!strcmp(a, "plane")) {
            double t = lm.t + lm.tephem, x, z, x2, z2;
            V3 f = moon_inertial_to_fixed(lm.r, t);
            terrain_site_to_plane(terrain_fixed_to_site(f), &x, &z);
            V3 f2 = moon_inertial_to_fixed(v3_add(lm.r, lm.v), t + 1);
            terrain_site_to_plane(terrain_fixed_to_site(f2), &x2, &z2);
            printf("PLANE t=%.1f x %.1f z %.1f (m east, m south of the site)  ground velocity %.1f %.1f m/s  alt %.1f m  rock here %.2f m\n",
                   lm.t, x, z, x2 - x, z2 - z, lm.altitude, terrain_rock_height(x, z));
        }
        else if (!strncmp(a, "alarms:", 7)) {
            int prev = 0, f0 = -1;
            for (double t = 0; t < atof(a + 7); t += 0.1) {
                lm_run(0.1);
                crew_tick();
                const DskyState *d = agc_host_dsky();
                int f = agc_get_word("FAILREG", 0);
                if ((d->prog_lamp && !prev) || f != f0) {
                    char ds[200];
                    dsky_str(ds, sizeof ds);
                    printf("ALARM t=%.1f (TIG%+.1f s) alt %.0f m  FAILREG %05o %05o %05o  %s\n", lm.t, lm.t - 369645, lm.altitude, f,
                           agc_get_word("FAILREG", 1), agc_get_word("FAILREG", 2), ds);
                    f0 = f;
                }
                prev = d->prog_lamp;
                if (lm.touchdown) break;
            }
        }
        else if (!strcmp(a, "nextevent")) {
            char why[160];
            char why2[160];
            milestone_on_track(why, sizeof why);
            int ms = milestone_next_event(why2, sizeof why2);
            printf("NEXTEVENT t=%.1f (TIG%+.1f s) %s; next event: %s%s%s\n", lm.t, lm.t - 369645, why, ms < 0 ? "none" : milestone_name(ms),
                   ms < 0 ? " - " : "", ms < 0 ? why2 : "");
        }
        else if (!strcmp(a, "advance")) {
            double t = lm.t;
            int ms = milestone_advance();
            printf("ADVANCE %s (t %.1f -> %.1f)\n", ms < 0 ? "none" : milestone_name(ms), t, lm.t);
        }
        else fprintf(stderr, "unknown command: %s\n", a);
    }
    return 0;
}
