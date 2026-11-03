// Plain-English hints: what to do next, in words a first-time player knows.
// Toggled with H (or from the menu); the setting is saved between runs.
#ifndef HINTS_H
#define HINTS_H

extern int hints_on;

void hints_load(void);
void hints_save(void);

// Draw the hint box at the top of the scene (below the DSKY prompt).
void hints_draw(int w, int h, int dsky_w);

#endif
