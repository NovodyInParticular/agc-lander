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

// Draw only (no input), e.g. into the cockpit panel texture.
void dsky_draw(Rectangle area);

// Keyboard shortcuts, key timers, PRO release. Call once per frame.
void dsky_frame(void);

// Key under a point in native DSKY coordinates (-1 = none); press it with the mouse.
int dsky_key_at(Vector2 native);
void dsky_mouse_press(int key_index);
const char *dsky_key_name(int key_index);

// Emission mode for the cockpit texture: draw only what glows, scaled by gain.
void dsky_set_emission(int on, float gain);

#endif
