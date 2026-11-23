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
enum { HT_NONE, HT_DSKY_KEY, HT_ENG_ARM, HT_PGNS_AUTO, HT_ENGINE_STOP, HT_MASTER_ALARM, HT_ROD_UP, HT_ROD_DOWN,
       HT_ENG_ARM_ASC, HT_ABORT_STAGE, HT_PGNS_ATTHOLD };
typedef struct { int kind, arg; const char *text; } HintTarget;
// Returns 0 (kind HT_NONE) when hints are off or nothing needs doing.
int hints_target(HintTarget *t);
// Whether the view is the cockpit (the hint box then mentions the marker).
void hints_set_cockpit(int in_cockpit);

// The event "advance to next event" can skip to now ("" if none).
void hints_set_next_event(const char *title);

// A line about the scenario, shown first until the landing ("" for none);
// take_over: the computer must not be left to land (no "let it fly").
void hints_set_note(const char *line, int take_over);
// The controller button the current hint wants pressed (headset): it glows on
// the controller model. VB_RTRIG = point at the ringed switch and pull.
enum { VB_NONE, VB_A, VB_B, VB_X, VB_Y, VB_LTRIG, VB_RTRIG, VB_LGRIP, VB_RGRIP, VB_LSTICK, VB_LSTICK_UP, VB_LSTICK_DOWN, VB_RSTICK };
int hints_vr_button(void);

// Set by the game: 1 while the docking-window view is up.
extern int hud_camera_is_docking;

#endif
