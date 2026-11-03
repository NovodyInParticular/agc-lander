// Lunar terrain around the landing site.
//
// Geometry lives on a gnomonic ("plane") projection centred on the landing
// site: plane coordinates (x east, z south, metres) map to a direction on the
// sphere, and the height function adds metres along that direction. The same
// height function drives rendering, the landing radar and footpad contact.
//
// Rendering uses the "site frame": moon-fixed Cartesian metres with origin at
// the landing site on the reference sphere, +X east, +Y up, +Z south.
#ifndef TERRAIN_H
#define TERRAIN_H

#include "vmath.h"

// site_fixed: landing site, moon-fixed (from RLS). radius = its radius.
void terrain_init(V3 site_fixed);
void terrain_shutdown(void);

// Height above the reference sphere at plane coordinates. min_feature (m)
// drops crater octaves smaller than that (for LOD); 0 = full detail.
double terrain_height(double x, double z, double min_feature);

// Albedo and "freshness" (0..1) at plane coordinates, for shading.
void terrain_surface_props(double x, double z, double min_feature, float *albedo, float *fresh);

// Conversions.
V3 terrain_plane_to_site(double x, double z, double h);
void terrain_site_to_plane(V3 site, double *x, double *z);
V3 terrain_fixed_to_site(V3 fixed);           // moon-fixed point -> site frame
V3 terrain_fixed_dir_to_site(V3 dir);         // moon-fixed direction -> site frame
M3 terrain_site_from_fixed(void);              // rotation, rows = east, up, south
double terrain_ref_radius(void);

// Moon radius below a selenographic lat/lon (rad), for physics.
double terrain_radius_at(double lat, double lon);

// Sun direction in the site frame (unit). Set before the first frame.
void terrain_set_sun(V3 sun_site);
V3 terrain_sun(void);

// Per-frame: choose LOD for the camera (site frame), schedule work, upload.
void terrain_update(V3 camera_site, double fov_y, int screen_h);

// Draw visible chunks relative to the camera (camera-relative rendering).
void terrain_draw(V3 camera_site, void *shader);

// Stats for the debug overlay.
void terrain_stats(int *chunks_drawn, int *chunks_loaded, int *jobs_pending);

#endif
