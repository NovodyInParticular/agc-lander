// DSKY (display and keyboard) rendering and input, drawn with raylib.
#ifndef AGC_LANDER_DSKY_H
#define AGC_LANDER_DSKY_H

#include "raylib.h"

// Native DSKY size in local units; it is scaled to fit the given rectangle.
#define DSKY_W 520.0f
#define DSKY_H 740.0f

// Draw the DSKY into `area` (keeping aspect ratio) and handle mouse clicks
// and keyboard shortcuts, forwarding key presses to the AGC.
void dsky_update_draw(Rectangle area);

// Draw onto the cockpit panel canvas (pcanvas.h) in native DSKY units; lit
// elements glow, scaled by `brightness`.
void dsky_draw_canvas(float brightness);

// Keyboard shortcuts, key timers, PRO release. Call once per frame.
void dsky_frame(void);

// Key under a point in native DSKY coordinates (-1 = none); press it with the mouse.
// Key geometry and state for the cabin's modelled key caps: rectangle in
// native units, and whether it is pressed right now. Returns 0 past the last key.
int dsky_key_layout(int i, Rectangle *native, int *down);
// Height of the key tops above the canvas plane (native units), up and pressed.
void dsky_set_key_lift(float up, float down);
// Index of a key from its letter: V N E R P C K, + - or a digit (-1 = none).
int dsky_key_index(int c);

int dsky_key_at(Vector2 native);
void dsky_mouse_press(int key_index);
const char *dsky_key_name(int key_index);

#endif
