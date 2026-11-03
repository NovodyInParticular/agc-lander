#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "yaAGC.h"
#include "agc_engine.h"
#include "agc_mem.h"

agc_t *agc_host_state(void);

static int cmp_sym(const void *key, const void *elem)
{
    return strcmp((const char *)key, ((const AgcSymbol *)elem)->name);
}

int agc_sym(const char *name, int *bank, int *offset)
{
    const AgcSymbol *s = bsearch(name, agc_symbols, (size_t)agc_symbol_count,
                                 sizeof agc_symbols[0], cmp_sym);
    if (!s)
        return 1;
    *bank = s->bank;
    *offset = s->offset;
    return 0;
}

static int16_t *word_ptr(const char *name, int index)
{
    int bank, off;
    if (agc_sym(name, &bank, &off)) {
        fprintf(stderr, "agc_mem: unknown erasable symbol '%s'\n", name);
        abort();
    }
    // Unswitched banks 0-2 are contiguous; switched banks wrap within the bank.
    int abs_addr = bank * 0400 + off + index;
    if (bank >= 3 && (off + index) >= 0400) {
        fprintf(stderr, "agc_mem: %s+%d crosses an erasable bank\n", name, index);
        abort();
    }
    return &agc_host_state()->Erasable[abs_addr / 0400][abs_addr % 0400];
}

int agc_to_ones(int v)
{
    if (v >= 0)
        return v & 037777;
    return (~(-v)) & 077777;
}

int agc_from_ones(int w)
{
    w &= 077777;
    if (w & 040000)
        return -((~w) & 037777);
    return w;
}

int agc_get_word(const char *name, int index) { return *word_ptr(name, index) & 077777; }
void agc_set_word(const char *name, int index, int value) { *word_ptr(name, index) = (int16_t)(value & 077777); }

double agc_get_sp(const char *name, int index, int scale)
{
    return ldexp(agc_from_ones(agc_get_word(name, index)), scale - 14);
}

void agc_set_sp(const char *name, int index, double value, int scale)
{
    double frac = ldexp(value, -scale);
    long n = lround(fabs(frac) * 16384.0);
    if (n > 037777) n = 037777;
    agc_set_word(name, index, agc_to_ones(frac < 0 ? -(int)n : (int)n));
}

double agc_get_dp(const char *name, int index, int scale)
{
    double hi = agc_from_ones(agc_get_word(name, index));
    double lo = agc_from_ones(agc_get_word(name, index + 1));
    return ldexp(hi * 16384.0 + lo, scale - 28);
}

static void set_multi(const char *name, int index, double value, int scale, int words)
{
    double frac = ldexp(value, -scale);
    int bits = 14 * words;
    double mag = ldexp(fabs(frac), bits);
    double maxv = ldexp(1.0, bits) - 1;
    if (mag > maxv) {
        fprintf(stderr, "agc_mem: %s+%d value %g overflows scale 2^%d\n", name, index, value, scale);
        mag = maxv;
    }
    unsigned long long n = (unsigned long long)llround(mag);
    for (int i = words - 1; i >= 0; i--) {
        int part = (int)(n & 037777);
        n >>= 14;
        agc_set_word(name, index + i, agc_to_ones(frac < 0 ? -part : part));
    }
}

void agc_set_dp(const char *name, int index, double value, int scale) { set_multi(name, index, value, scale, 2); }
void agc_set_tp(const char *name, int index, double value, int scale) { set_multi(name, index, value, scale, 3); }

V3 agc_get_vec(const char *name, int scale)
{
    return v3(agc_get_dp(name, 0, scale), agc_get_dp(name, 2, scale), agc_get_dp(name, 4, scale));
}

void agc_set_vec(const char *name, V3 v, int scale)
{
    agc_set_dp(name, 0, v.x, scale);
    agc_set_dp(name, 2, v.y, scale);
    agc_set_dp(name, 4, v.z, scale);
}

void agc_set_bit(const char *word, int bit, int on)
{
    int w = agc_get_word(word, 0), m = 1 << (bit - 1);
    agc_set_word(word, 0, on ? (w | m) : (w & ~m));
}

int agc_get_bit(const char *word, int bit)
{
    return (agc_get_word(word, 0) >> (bit - 1)) & 1;
}
