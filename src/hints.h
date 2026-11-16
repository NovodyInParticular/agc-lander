// Plain-English hints: what to do next, in words a first-time player knows.
// Toggled with H (or from the menu); the setting is saved between runs.
#ifndef HINTS_H
#define HINTS_H

extern int hints_on;

void hints_load(void);
void hints_save(void);

// Draw the hint box at the top of the scene (below the DSKY prompt).
void hints_draw(int w, int h, int dsky_w);

// The one control the player should operate next, so that the cockpit and
// the DSKY can mark it. `arg` for HT_DSKY_KEY is the key: 'V' VERB, 'N' NOUN,
// 'E' ENTR, 'R' RSET, 'P' PRO, or a digit. `text` says what to do with it.
enum { HT_NONE, HT_DSKY_KEY, HT_ENG_ARM, HT_PGNS_AUTO, HT_ENGINE_STOP, HT_MASTER_ALARM, HT_ROD_UP, HT_ROD_DOWN };
typedef struct { int kind, arg; const char *text; } HintTarget;
// Returns 0 (kind HT_NONE) when hints are off or nothing needs doing.
int hints_target(HintTarget *t);
// Whether the view is the cockpit (the hint box then mentions the marker).
void hints_set_cockpit(int in_cockpit);

#endif
