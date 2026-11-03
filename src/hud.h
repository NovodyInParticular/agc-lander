// Cockpit instruments and overlays.
#ifndef HUD_H
#define HUD_H

#include "raylib.h"

extern Font font_mono, font_ui, font_bold;

void hud_init(void);
void hud_shutdown(void);
void hud_prepare(void);
void hud_set_minimal(int on);   // only status text (cockpit / orbit views)
Texture2D hud_fdai_texture(void);   // call before BeginDrawing / any texture mode

// Draw all instruments. dsky_w: width reserved on the right for the DSKY.
void hud_draw(int w, int h, int dsky_w, float warp, const char *camera_name, int paused);

// Small helpers shared with other overlays.
void hud_text(Font f, const char *s, float x, float y, float size, Color c);
void hud_text_centered(Font f, const char *s, float cx, float y, float size, Color c);

#endif
