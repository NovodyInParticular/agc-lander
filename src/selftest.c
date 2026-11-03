// Headless check that Luminary099 boots and drives the DSKY.
// Exit status 0 = all checks passed.

#include <stdio.h>
#include <string.h>

#include "agc_host.h"

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

    printf("%s (%d failure%s)\n", fails ? "FAILED" : "ALL PASSED", fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
