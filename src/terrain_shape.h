// Terrain internals shared by terrain_shape.c (the ground's shape) and
// terrain.c (its rendering). The public interface is terrain.h.
#ifndef TERRAIN_SHAPE_H
#define TERRAIN_SHAPE_H

#include "vmath.h"

// Shader-drawn crater octaves: octaves[SHADER_OCT0 .. SHADER_OCT0 + SHADER_NOCT - 1]
// (render.c OCT_* tables: keep the two in step).
#define SHADER_OCT0 1
#define SHADER_NOCT 6

typedef struct { double h, fresh; } TerrainSample;

void terrain_eval(double x, double z, double min_feature, TerrainSample *s);
V3 terrain_plane_to_dir(double x, double z);
void terrain_frame(V3 *site_dir, V3 *east, V3 *north, V3 *up);
double terrain_octave_cell(int i);

#endif
