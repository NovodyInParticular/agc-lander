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

// Boulders and stones around the landing sites (plane coordinates). The shape
// parameters are drawn here so the renderer (rocks.c) and the footpad contact
// see the same rock: an ellipsoid of semi-axes sx, sy (up), sz turned by rot,
// its centre 0.15 r above the ground; seed shapes its lumps.
typedef struct {
    float x, z, r;
    float sx, sy, sz, rot, seed;
    unsigned char grey;
} TerrainRock;
int terrain_rocks(const TerrainRock **list);
// Height of the boulders above the ground at a point (0 where there is none);
// stones under ROCK_PHYS_MIN are drawn only.
#define ROCK_PHYS_MIN 0.3
double terrain_rock_height(double x, double z);
V3 terrain_plane_to_dir(double x, double z);
void terrain_frame(V3 *site_dir, V3 *east, V3 *north, V3 *up);
double terrain_octave_cell(int i);

#endif
