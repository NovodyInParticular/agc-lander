// Headless check that Luminary099 boots and drives the DSKY.
// Exit status 0 = all checks passed.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "agc_engine.h"
#include "agc_host.h"
#include "crew.h"
#include "lm.h"
#include "milestones.h"
#include "scenario.h"
#include "sim.h"
#include "snapshot.h"
#include "terrain.h"

static void type(const char *s)
{
    for (; *s; s++) {
        switch (*s) {
        case 'V': agc_host_key(KEY_AGC_VERB); break;
        case 'N': agc_host_key(KEY_AGC_NOUN); break;
        case 'E': agc_host_key(KEY_AGC_ENTR); break;
        case 'R': agc_host_key(KEY_AGC_RSET); break;
        case 'C': agc_host_key(KEY_AGC_CLR); break;
        case '+': agc_host_key(KEY_AGC_PLUS); break;
        case '-': agc_host_key(KEY_AGC_MINUS); break;
        case '0': agc_host_key(KEY_AGC_0); break;
        default:
            if (*s >= '1' && *s <= '9')
                agc_host_key(*s - '0');
        }
    }
}

static void dsky_line(char *buf, size_t n)
{
    const DskyState *d = agc_host_dsky();
    char r[3][7];
    for (int i = 0; i < 3; i++) {
        r[i][0] = (d->sign[i] & 2) ? '+' : (d->sign[i] & 1) ? '-' : ' ';
        for (int j = 0; j < 5; j++)
            r[i][j + 1] = agc_digit_char(d->reg[i][j]);
        r[i][6] = 0;
    }
    snprintf(buf, n, "PROG %c%c  VERB %c%c  NOUN %c%c  R1 %s  R2 %s  R3 %s",
             agc_digit_char(d->prog[0]), agc_digit_char(d->prog[1]),
             agc_digit_char(d->verb[0]), agc_digit_char(d->verb[1]),
             agc_digit_char(d->noun[0]), agc_digit_char(d->noun[1]),
             r[0], r[1], r[2]);
}

static void show(const char *label)
{
    char line[160];
    dsky_line(line, sizeof line);
    const DskyState *d = agc_host_dsky();
    printf("%-22s %s  [%s%s%s%s]\n", label, line,
           d->comp_acty ? "COMP " : "", d->opr_err ? "OPR_ERR " : "",
           d->restart ? "RESTART " : "", d->prog_lamp ? "PROG" : "");
}

static int fails;
static void check(int cond, const char *what)
{
    printf("  %s: %s\n", cond ? "PASS" : "FAIL", what);
    if (!cond)
        fails++;
}

static int verb_noun_is(const char *vn)
{
    const DskyState *d = agc_host_dsky();
    return agc_digit_char(d->verb[0]) == vn[0] && agc_digit_char(d->verb[1]) == vn[1] &&
           agc_digit_char(d->noun[0]) == vn[2] && agc_digit_char(d->noun[1]) == vn[3];
}

// Save states must resume exactly: snapshot mid-P63 (auto crew, engine
// burning), run 60 s, snapshot again; then wipe the world, restore the first
// snapshot, run the same 60 s and compare the two results byte for byte. Any
// piece of state the snapshot misses makes them differ.
static void first_difference(const uint8_t *a, const uint8_t *b, size_t n)
{
    for (size_t i = 0; i < n; i++)
        if (a[i] != b[i]) {
            // Walk the sections to name the one that differs.
            size_t p = 4 * 4 + 40 + 4 + 8 + 4;
            while (p + 12 <= n) {
                uint32_t len = a[p + 8] | a[p + 9] << 8 | a[p + 10] << 16 | (uint32_t)a[p + 11] << 24;
                if (i < p + 12 + len) {
                    printf("    first difference at byte %zu: section %.4s, offset %zu\n", i, (const char *)a + p, i - p - 12);
                    return;
                }
                p += 12 + len;
            }
            printf("    first difference at byte %zu (header)\n", i);
            return;
        }
}

// Wipe as much as possible: another LGC (the lift-off one), another crew
// state, an odd amount of time run, so anything the snapshot leaves out keeps
// a value unlike the original's.
agc_t *agc_host_state(void);
void agc_engine_hidden_io(void (*io)(void *ctx, void *var, int bytes), void *ctx);

static void poison_io(void *ctx, void *var, int bytes)
{
    (void)ctx;
    if (bytes == 8) *(uint64_t *)var = 3;
    else *(uint32_t *)var = 3;
}

static void wipe(void)
{
    crew_reset();
    crew.auto_crew = 0;
    scenario_liftoff(0);
    crew_type("V16N36E");
    sim_advance(7.3137);
    // Junk where an incomplete restore would leave it: every erasable word,
    // the engine's hidden counters, counter pulses and a keystroke in flight.
    agc_t *a = agc_host_state();
    for (int bank = 0; bank < 8; bank++)
        for (int i = 0; i < 0400; i++) a->Erasable[bank][i] ^= 012345;
    agc_engine_hidden_io(poison_io, NULL);
    for (int k = 0; k < 3; k++) agc_host_pulse(RegCDUX + k, 40 + k, AGC_PULSE_CDU_FAST);
    agc_host_pulse(RegPIPAX, 7, AGC_PULSE_PINC);
    agc_host_key(KEY_AGC_CLR);
    agc_host_set_rhc(0.4, -0.3, 0.2);
}

// Snapshot now, run 60 s; wipe, restore, run the same 60 s; compare.
static void round_trip(const char *where)
{
    uint8_t *s0, *x1, *x2;
    size_t n0 = snapshot_save(&s0, where, -1);
    sim_advance(60.0);
    size_t n1 = snapshot_save(&x1, "after 60 s", -1);
    wipe();
    char err[200];
    int rc = snapshot_load(s0, n0, NULL, err, sizeof err);
    if (rc) printf("    load failed: %s\n", err);
    sim_advance(60.0);
    size_t n2 = snapshot_save(&x2, "after 60 s", -1);
    int same = rc == 0 && n1 == n2 && memcmp(x1, x2, n1) == 0;
    if (rc == 0 && !same && n1 == n2) first_difference(x1, x2, n1);
    printf("    %s: snapshot %zu bytes, after 60 s: %s\n", where, n0, same ? "identical" : "DIFFERENT");
    char what[160];
    snprintf(what, sizeof what, "%s: restore + 60 s matches the original run bit for bit", where);
    check(same, what);
    // A snapshot from another rope is refused.
    s0[12] ^= 1;
    if (!strcmp(where, "mid-P63")) check(snapshot_load(s0, n0, NULL, err, sizeof err) != 0, "a snapshot from another rope is refused");
    free(s0);
    free(x1);
    free(x2);
}

static void save_state_test(void)
{
    printf("save states (automatic crew, from 15 min before PDI):\n");
    // 1. While the crew is keying something in: keys queued, mid-entry.
    crew_reset();
    crew.auto_crew = 1;
    scenario_pre_pdi();
    double t0 = lm.t;
    while (lm.t - t0 < 900 && !(crew_typing() && agc_host_keys_queued() > 0)) sim_advance(0.0173);
    check(crew_typing(), "caught the crew part-way through a DSKY entry");
    round_trip("crew typing");

    // 2. Mid-P63, the descent engine burning, at an odd time.
    crew_reset();
    crew.auto_crew = 1;
    scenario_pre_pdi();
    t0 = lm.t;
    double tig = 0;
    while (lm.t - t0 < 2400) {
        sim_advance(1.0);
        if (lm.dps_on && tig == 0) tig = lm.t;
        if (tig > 0 && lm.t - tig > 200) break;
    }
    check(tig > 0, "the descent engine lit (P63 under way)");
    sim_advance(0.4567);
    round_trip("mid-P63");
}

// The embedded milestones (make snapshots) must load in this build - a
// snapshot made before the state layout changed is refused - and a world
// restored from one must fly on: from 150 m the automatic crew lands.
static void milestone_test(void)
{
    printf("embedded milestones (data/snapshots.c):\n");
    char err[200], what[160];
    for (int m = 0; m < MS_COUNT; m++) {
        crew_reset();
        int rc = milestone_load(m, err, sizeof err);
        if (rc) printf("    %s: %s\n", milestone_name(m), err);
        else printf("    %-9s t=%.1f  alt %.0f m  DPS %s  ENG ARM %d\n", milestone_name(m), lm.t, lm.altitude, lm.dps_on ? "on" : "off", lm.engine_armed);
        snprintf(what, sizeof what, "milestone %s loads in this build", milestone_name(m));
        check(rc == 0, what);
    }
    check(track_count() > 1000, "the canonical track is embedded");
    if (milestone_load(MS_150M, err, sizeof err)) return;
    terrain_shape_init(scenario_info.rls);
    lm.surface = terrain_surface_cb;
    crew.auto_crew = 1;
    double t0 = lm.t;
    while (!lm.touchdown && lm.t - t0 < 300) sim_advance(1.0);
    sim_advance(5.0);
    printf("    from 150 m: touchdown %d after %.0f s, result %d, %.2f m/s\n", lm.touchdown, lm.touchdown_t - t0, lm.land_result, -lm.touchdown_speed);
    check(lm.touchdown && lm.land_result == 1, "from the 150 m milestone the automatic crew lands");
    lm.surface = NULL;
}

int main(void)
{
    if (agc_host_init() != 0) {
        printf("FAIL: could not initialise AGC\n");
        return 1;
    }
    agc_host_run(5.0);
    show("after 5 s boot:");
    check(agc_host_cycles() > 0, "CPU is executing");

    // V35E: DSKY light test. All segments show 8, lamps lit.
    type("V35E");
    agc_host_run(2.0);
    show("V35E (lamp test):");
    check(verb_noun_is("8888"), "lamp test drives VERB/NOUN to 88/88");
    agc_host_run(6.0);  // test lasts ~5 s
    show("lamp test finished:");

    // V16N36E: monitor AGC clock (hours / minutes / seconds*100).
    type("V16N36E");
    agc_host_run(2.0);
    show("V16N36E:");
    check(verb_noun_is("1636"), "VERB 16 NOUN 36 displayed");
    char a[160], b[160];
    dsky_line(a, sizeof a);
    agc_host_run(3.0);
    dsky_line(b, sizeof b);
    show("+3 s:");
    check(strcmp(a, b) != 0, "clock display advances in R3");

    // RSET: acknowledges the cold-boot alarm; PROG and RESTART lamps go out.
    type("R");
    agc_host_run(2.0);
    show("RSET:");
    const DskyState *d = agc_host_dsky();
    check(!d->restart && !d->prog_lamp, "RSET clears RESTART and PROG lamps");

    save_state_test();
    milestone_test();

    printf("%s (%d failure%s)\n", fails ? "FAILED" : "ALL PASSED", fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
