// Panel canvas: instrument panel artwork as real geometry in the cabin.
//
// Panels used to be painted into a texture atlas every frame; lettering came
// out a few texels tall. Here every rectangle, circle, line and glyph is a
// quad on the panel's plane whose edge is evaluated in the fragment shader
// (rounded-rectangle distance, or a signed-distance-field font), so it stays
// sharp at any distance and resolution. Each primitive carries its paint
// colour and its own glow (panel integral lighting, EL numerics, lamps).
//
// Coordinates are the panel's own (millimetres, y down). A canvas is split
// into layers, one per panel; a layer is only re-uploaded when it changes.
#ifndef PCANVAS_H
#define PCANVAS_H

#include "raylib.h"
#include "vmath.h"

#define PC_MAX_LAYERS 16

void pc_init(void);
void pc_shutdown(void);

// Start recording a layer: `origin` is the plane point of (0, 0), `du` / `dv`
// the body-frame step (metres) for one unit across / down, `normal` the unit
// normal toward the viewer. Everything is lifted `lift` metres off the plane.
void pc_begin(int layer, V3 origin, V3 du, V3 dv, V3 normal, double lift);
// Finish the layer; returns 1 if its geometry changed since the last time.
int pc_end(void);

// Nested 2D transform: local (x, y) -> (ox + x * zoom, oy + y * zoom).
void pc_push(float ox, float oy, float zoom);
void pc_pop(void);

// Glow of the following primitives: an absolute colour, or `k` times their
// own paint colour (back-lit markings). pc_glow_off() ends either.
void pc_glow(Color c);
void pc_glow_paint(float k);
void pc_glow_off(void);

void pc_rect(float x, float y, float w, float h, Color c);
void pc_rect_lines(Rectangle r, float thick, Color c);
// roundness as raylib: corner radius = roundness * min(w, h) / 2.
void pc_rrect(Rectangle r, float roundness, Color c);
void pc_rrect_lines(Rectangle r, float roundness, float thick, Color c);
void pc_line(Vector2 a, Vector2 b, float thick, Color c);
void pc_circle(Vector2 centre, float radius, Color c);
void pc_ring(Vector2 centre, float radius, float thick, Color c);
void pc_tri(Vector2 a, Vector2 b, Vector2 c, Color col);
void pc_poly(Vector2 centre, int sides, float radius, float rot_deg, Color c);
void pc_poly_lines(Vector2 centre, int sides, float radius, float rot_deg, float thick, Color c);

// Text: (x, y) is the top-left of the first line, `size` the line height.
Vector2 pc_measure(const char *s, float size, float spacing);
void pc_text(const char *s, float x, float y, float size, float spacing, Color c);
// Lines (split at '\n') centred in a rectangle.
void pc_text_box(const char *s, Rectangle r, float size, float spacing, Color c);

// A picture (the FDAI ball render): `dst` rectangle, texture v flipped as for
// raylib render textures. Its glow is pc_glow_paint's factor.
void pc_image(Rectangle dst);

// Drawing. The caller owns the shader (see pc_fragment_glsl) and its uniforms.
Texture2D pc_font_texture(void);
float pc_font_slope(void);   // distance-field value change per atlas texel
int pc_layer_ready(int layer);
void pc_draw_layer(int layer, Material mat, Matrix transform);
int pc_layer_quads(int layer);

#endif
