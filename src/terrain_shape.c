// The shape of the ground around the landing site, without any rendering:
// frames, the height field (craters, undulation, hand-placed features) and the
// plane <-> site conversions. Used by the renderer (terrain.c) and, headless,
// by the lab and the save-state generator, so all of them fly over the same
// Moon.

#include <math.h>
#include <stdint.h>

#include "moon.h"
#include "terrain.h"
#include "terrain_shape.h"

// ---------------------------------------------------------------------------
// Frames.

static V3 site_dir, east, north, up;   // moon-fixed unit vectors
static double R0;                        // reference radius (landing site)
double terrain_ref_radius(void) { return R0; }

M3 terrain_site_from_fixed(void) { return m3_rows(east, up, v3_scale(north, -1)); }

V3 terrain_fixed_dir_to_site(V3 d)
{
    return v3(v3_dot(d, east), v3_dot(d, up), -v3_dot(d, north));
}

V3 terrain_fixed_to_site(V3 p)
{
    return terrain_fixed_dir_to_site(v3_sub(p, v3_scale(site_dir, R0)));
}

V3 terrain_plane_to_dir(double x, double z)
{
    return v3_norm(v3_add(up, v3_sub(v3_scale(east, x / R0), v3_scale(north, z / R0))));
}

V3 terrain_plane_to_site(double x, double z, double h)
{
    V3 d = terrain_plane_to_dir(x, z);
    return terrain_fixed_to_site(v3_scale(d, R0 + h));
}

void terrain_site_to_plane(V3 s, double *x, double *z)
{
    // site -> fixed direction, then gnomonic projection.
    V3 p = v3_add(v3_scale(site_dir, R0), v3_add(v3_scale(east, s.x), v3_add(v3_scale(up, s.y), v3_scale(north, -s.z))));
    double du = v3_dot(p, up);
    if (du < 1.0) du = 1.0;
    *x = R0 * v3_dot(p, east) / du;
    *z = -R0 * v3_dot(p, north) / du;
}

// ---------------------------------------------------------------------------
// Hashing and noise.

static inline uint32_t hash2(int32_t x, int32_t y, uint32_t seed)
{
    uint32_t h = (uint32_t)x * 0x8da6b343u ^ (uint32_t)y * 0xd8163841u ^ seed * 0xcb1ab31fu;
    h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15;
    return h;
}
static inline double u01(uint32_t h) { return (h & 0xffffff) / 16777216.0; }

static double vnoise(double x, double y, uint32_t seed)
{
    double fx = floor(x), fy = floor(y);
    int32_t ix = (int32_t)fx, iy = (int32_t)fy;
    double tx = x - fx, ty = y - fy;
    tx = tx * tx * (3 - 2 * tx);
    ty = ty * ty * (3 - 2 * ty);
    double a = u01(hash2(ix, iy, seed)), b = u01(hash2(ix + 1, iy, seed));
    double c = u01(hash2(ix, iy + 1, seed)), d = u01(hash2(ix + 1, iy + 1, seed));
    return (a + (b - a) * tx) + ((c + (d - c) * tx) - (a + (b - a) * tx)) * ty;
}

// ---------------------------------------------------------------------------
// Craters.

// gk: an octave is modelled as geometry once cell * gk >= the chunk's detail
// floor (1.5 x vertex spacing). Below that the terrain shader draws the very
// same craters per pixel (render.c, OCT_* tables: keep the two in step), so
// they are visible from any height and the mesh takes them over unnoticed.
typedef struct { double cell; double prob; uint32_t seed; double gk; } Octave;
static const Octave octaves[] = {
    { 24000, 0.30, 11, 0.38 }, { 6000, 0.45, 12, 0.15 }, { 1500, 0.55, 13, 0.15 }, { 400, 0.60, 14, 0.15 },
    { 100, 0.65, 15, 0.15 }, { 25, 0.65, 16, 0.15 }, { 6, 0.55, 17, 0.38 }, { 1.6, 0.45, 18, 0.38 },
};
#define NOCT (int)(sizeof octaves / sizeof octaves[0])

// Hand-placed features near the landing site, in plane coordinates.
// "West crater"-like fresh blocky crater just past the target, plus a few.
typedef struct { double x, z, r, depth, rim, fresh; } Feature;
static const Feature features[] = {
    { -380, -120, 95, 22, 5, 1.0 },     // fresh, boulder strewn
    { 240, 310, 160, 20, 4, 0.4 },
    { -900, 450, 60, 9, 2, 0.8 },
    { 650, -520, 300, 30, 6, 0.2 },
    { -60, 520, 35, 6, 1.4, 0.7 },
};
#define NFEAT (int)(sizeof features / sizeof features[0])

static double crater_profile(double r, double depth, double rim)
{
    if (r < 1.0) {
        double r2 = r * r;
        double floor_ = -depth * 0.82;
        double bowl = -depth * (1.0 - r2) + rim * r2 * r2 * r2;
        return bowl < floor_ ? floor_ + (bowl - floor_) * 0.15 : bowl;
    }
    double t = r - 1.0;
    double e = 1.0 / (1.0 + 5.0 * t);
    return rim * e * e * e;
}


static void crater_octave(double x, double z, const Octave *o, TerrainSample *s)
{
    double fx = floor(x / o->cell), fz = floor(z / o->cell);
    int32_t cx = (int32_t)fx, cz = (int32_t)fz;
    for (int dz = -1; dz <= 1; dz++) {
        for (int dx = -1; dx <= 1; dx++) {
            uint32_t h = hash2(cx + dx, cz + dz, o->seed);
            if (u01(h) > o->prob) continue;
            uint32_t h2 = hash2(cx + dx, cz + dz, o->seed + 101);
            uint32_t h3 = hash2(cx + dx, cz + dz, o->seed + 202);
            double px = (cx + dx + 0.15 + 0.7 * u01(h2)) * o->cell;
            double pz = (cz + dz + 0.15 + 0.7 * u01(h2 >> 7 | h2 << 25)) * o->cell;
            double rr = o->cell * (0.08 + 0.30 * u01(h3) * u01(h3 >> 9 | h3 << 23));
            double ddx = x - px, ddz = z - pz;
            double d2 = ddx * ddx + ddz * ddz;
            if (d2 > rr * rr * 6.25) continue;
            double r = sqrt(d2) / rr;
            double fresh = u01(hash2(cx + dx, cz + dz, o->seed + 303));
            fresh = fresh * fresh * fresh;
            double depth = rr * (0.10 + 0.30 * fresh);
            double rim = depth * (0.18 + 0.12 * fresh);
            s->h += crater_profile(r, depth, rim);
            if (r < 2.5) {
                double w = r < 1.0 ? 1.0 : (2.5 - r) / 1.5;
                double f = fresh * w * (rr < 400 ? 1.0 : 0.4);
                if (f > s->fresh) s->fresh = f;
            }
        }
    }
}

void terrain_eval(double x, double z, double min_feature, TerrainSample *s)
{
    s->h = 0;
    s->fresh = 0;
    // Gentle mare undulation.
    double a = 0;
    a += 140.0 * (vnoise(x / 30000.0, z / 30000.0, 1) - 0.5);
    a += 45.0 * (vnoise(x / 7000.0, z / 7000.0, 2) - 0.5);
    if (min_feature < 400) a += 9.0 * (vnoise(x / 1200.0, z / 1200.0, 3) - 0.5);
    if (min_feature < 60) a += 1.6 * (vnoise(x / 180.0, z / 180.0, 4) - 0.5);
    if (min_feature < 8) a += 0.25 * (vnoise(x / 22.0, z / 22.0, 5) - 0.5);
    // Flatten the immediate landing area a little (real site: gentle mare).
    double dsite = sqrt(x * x + z * z);
    double flat = dsite < 3000 ? 0.35 + 0.65 * dsite / 3000 : 1.0;
    s->h = a * flat;

    for (int i = 0; i < NOCT; i++) {
        if (octaves[i].cell * octaves[i].gk < min_feature) break;
        TerrainSample c = { 0, 0 };
        crater_octave(x, z, &octaves[i], &c);
        double damp = (i < 2) ? flat : 1.0;
        s->h += c.h * damp;
        if (c.fresh > s->fresh) s->fresh = c.fresh;
    }
    for (int i = 0; i < NFEAT; i++) {
        const Feature *f = &features[i];
        if (f->r * 0.5 < min_feature) continue;
        double dx = x - f->x, dz = z - f->z;
        double r = sqrt(dx * dx + dz * dz) / f->r;
        if (r > 3.0) continue;
        s->h += crater_profile(r, f->depth, f->rim);
        if (r < 2.5) {
            double fr = f->fresh * (r < 1 ? 1.0 : (2.5 - r) / 1.5);
            if (fr > s->fresh) s->fresh = fr;
        }
    }
}

double terrain_height(double x, double z, double min_feature)
{
    TerrainSample s;
    terrain_eval(x, z, min_feature, &s);
    return s.h;
}

void terrain_surface_props(double x, double z, double min_feature, float *albedo, float *fresh)
{
    TerrainSample s;
    terrain_eval(x, z, min_feature, &s);
    double base = 0.085 + 0.02 * (vnoise(x / 9000.0, z / 9000.0, 21) - 0.5) + 0.012 * (vnoise(x / 700.0, z / 700.0, 22) - 0.5);
    *albedo = (float)(base + 0.07 * s.fresh);
    *fresh = (float)s.fresh;
}

double terrain_radius_at(double lat, double lon)
{
    V3 d = moon_latlon_to_fixed(lat, lon, 1.0);
    double du = v3_dot(d, up);
    if (du < 0.2) return R0;   // far outside the modelled region
    double x = R0 * v3_dot(d, east) / du, z = -R0 * v3_dot(d, north) / du;
    return R0 + terrain_height(x, z, 0);
}

void terrain_shape_init(V3 site_fixed)
{
    R0 = v3_len(site_fixed);
    site_dir = v3_norm(site_fixed);
    up = site_dir;
    V3 z = v3(0, 0, 1);
    east = v3_norm(v3_cross(z, up));
    north = v3_cross(up, east);
}

void terrain_frame(V3 *site_dir_out, V3 *east_out, V3 *north_out, V3 *up_out)
{
    *site_dir_out = site_dir;
    *east_out = east;
    *north_out = north;
    *up_out = up;
}

double terrain_octave_cell(int i) { return octaves[i].cell; }

// Physics hook (Lm.surface): the radius under a selenographic lat/lon.
double terrain_surface_cb(void *ctx, double lat, double lon)
{
    (void)ctx;
    return terrain_radius_at(lat, lon);
}
