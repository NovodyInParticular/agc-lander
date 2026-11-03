// Named access to AGC erasable memory with fixed-point scaling.
//
// Scale convention: `scale` is the power of two the stored fraction is
// multiplied by to get the physical value. A variable documented as
// "scaled at 2(24) M" or padloaded with "B-24" has scale = 24.
#ifndef AGC_MEM_H
#define AGC_MEM_H

#include <stdint.h>

#include "vmath.h"

typedef struct { const char *name; int bank, offset; } AgcSymbol;
extern const AgcSymbol agc_symbols[];
extern const int agc_symbol_count;

// Look up an erasable symbol. Returns 0 if found.
int agc_sym(const char *name, int *bank, int *offset);

// Raw 15-bit word access (index = word offset from the symbol).
int agc_get_word(const char *name, int index);
void agc_set_word(const char *name, int index, int value);

// Single precision: 14 bits + sign.
double agc_get_sp(const char *name, int index, int scale);
void agc_set_sp(const char *name, int index, double value, int scale);

// Double precision: 28 bits + sign (two words).
double agc_get_dp(const char *name, int index, int scale);
void agc_set_dp(const char *name, int index, double value, int scale);

// Triple precision (e.g. TEPHEM): 42 bits + sign.
void agc_set_tp(const char *name, int index, double value, int scale);

// Vectors of three DP components.
V3 agc_get_vec(const char *name, int scale);
void agc_set_vec(const char *name, V3 v, int scale);

// Flags: name of the flagword (e.g. "FLAGWRD3") and bit number 1..15.
void agc_set_bit(const char *word, int bit, int on);
int agc_get_bit(const char *word, int bit);

// Ones'-complement helpers.
int agc_to_ones(int value);          // signed int (|v| < 2^14) -> 15-bit word
int agc_from_ones(int word);         // 15-bit word -> signed int

#endif
