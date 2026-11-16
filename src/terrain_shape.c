// The shape of the ground around the landing site, without any rendering:
// frames, the height field (craters, undulation, hand-placed features) and the
// plane <-> site conversions. Used by the renderer (terrain.c) and, headless,
// by the lab and the save-state generator, so all of them fly over the same
// Moon.

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

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
// boulders: how many blocks its ejecta strews around it.
typedef struct { double x, z, r, depth, rim, fresh; int boulders; } Feature;
static const Feature features[] = {
    { -380, -120, 95, 22, 5, 1.0, 1500 },  // fresh, boulder strewn
    { 240, 310, 160, 20, 4, 0.4, 0 },
    { -900, 450, 60, 9, 2, 0.8, 0 },
    { 650, -520, 300, 30, 6, 0.2, 0 },
    { -60, 520, 35, 6, 1.4, 0.7, 0 },
    // Site 2, 7 km downrange (the ANOTHER SITE scenario, target -7000, -60): an
    // old crater, a fresh blocky one, and a chain of small ones.
    { -7330, 560, 450, 48, 8, 0.25, 0 },
    { -6830, 130, 70, 16, 4, 1.0, 450 },
    { -6640, -240, 42, 8, 2, 0.6, 0 },
    { -6730, -205, 34, 7, 1.6, 0.6, 0 },
    { -6810, -175, 38, 7, 1.8, 0.6, 0 },
    { -6900, -150, 30, 6, 1.4, 0.6, 0 },
};
// Stones strewn around Site 2 (the main site has its own scatter, rocks below).
static const struct { double x, z, radius; int n; } scatters[] = { { -7000, -60, 900, 1400 } };
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
    return R0 + terrain_height(x, z, 0) + terrain_rock_height(x, z);
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

// --- Boulders ----------------------------------------------------------------------

static TerrainRock *rocks;
static int nrocks, caprocks;
static unsigned rrng;
static double rfrand(void) { rrng = rrng * 1664525u + 1013904223u; return (rrng >> 8) / 16777216.0; }

static void add_rock(double x, double z, double r)
{
    if (nrocks == caprocks) {
        caprocks = caprocks ? caprocks * 2 : 8192;
        rocks = realloc(rocks, (size_t)caprocks * sizeof *rocks);
    }
    TerrainRock *k = &rocks[nrocks++];
    k->x = (float)x;
    k->z = (float)z;
    k->r = (float)r;
    k->grey = (unsigned char)(28 + rfrand() * 16);
    k->sx = (float)(r * (0.8 + 0.5 * rfrand()));
    k->sy = (float)(r * (0.45 + 0.4 * rfrand()));
    k->sz = (float)(r * (0.8 + 0.5 * rfrand()));
    k->rot = (float)(rfrand() * 2 * PI_D);
    k->seed = (float)(rfrand() * 100);
}

// Contact grid: 4 m cells, each physical rock listed in every cell it covers.
#define RCELL 4.0
typedef struct { int64_t key; int idx; } RockCell;
static RockCell *cells;
static int ncells;

static int64_t cell_key(int ix, int iz) { return ((int64_t)ix << 32) ^ (uint32_t)iz; }
static int cmp_cell(const void *a, const void *b)
{
    int64_t x = ((const RockCell *)a)->key, y = ((const RockCell *)b)->key;
    return x < y ? -1 : x > y;
}

static void build_rocks(void)
{
    // The scatter around the site: denser near the landing area, sparser out
    // to ~1.2 km, a few clusters (the numbers drawn in the order rocks.c always used).
    rrng = 1969u;
    for (int i = 0; i < 4500; i++) {
        double rr = 1200 * pow(rfrand(), 0.75), th = rfrand() * 2 * PI_D;
        double x = rr * cos(th), z = rr * sin(th);
        if (i % 9 == 0) {
            int cl = (int)(rfrand() * 6);
            double cx = -300 + 140 * cl, cz = 120 - 90 * (cl % 3);
            x = cx + (rfrand() - 0.5) * 60;
            z = cz + (rfrand() - 0.5) * 60;
        }
        double r = 0.14 * pow(rfrand() + 1e-3, -0.6);
        if (r > 2.8) r = 2.8;
        add_rock(x, z, r);
    }
    for (size_t s = 0; s < sizeof scatters / sizeof scatters[0]; s++) {
        rrng = 4242u + (unsigned)s;
        for (int i = 0; i < scatters[s].n; i++) {
            double rr = scatters[s].radius * pow(rfrand(), 0.75), th = rfrand() * 2 * PI_D;
            double r = 0.14 * pow(rfrand() + 1e-3, -0.6);
            if (r > 2.8) r = 2.8;
            add_rock(scatters[s].x + rr * cos(th), scatters[s].z + rr * sin(th), r);
        }
    }
    // Boulder fields: blocks thrown out of fresh craters, a few on the floor
    // and walls, most on and just beyond the rim.
    for (int f = 0; f < NFEAT; f++) {
        rrng = 7001u + (unsigned)f * 7919u;
        for (int i = 0; i < features[f].boulders; i++) {
            double R = features[f].r, d;
            if (rfrand() < 0.15) d = R * (0.25 + 0.7 * rfrand());
            else d = R * (0.95 + 0.5 * pow(rfrand(), 1.3));
            double th = rfrand() * 2 * PI_D;
            double r = 0.5 * pow(rfrand() + 1e-3, -0.38);
            if (r > 3.2) r = 3.2;
            add_rock(features[f].x + d * cos(th), features[f].z + d * sin(th), r);
        }
    }
    // Contact grid.
    int cap = 0;
    for (int pass = 0; pass < 2; pass++) {
        ncells = 0;
        for (int i = 0; i < nrocks; i++) {
            const TerrainRock *k = &rocks[i];
            if (k->r < ROCK_PHYS_MIN) continue;
            double rh = 0.5 * (k->sx + k->sz);
            int x0 = (int)floor((k->x - rh) / RCELL), x1 = (int)floor((k->x + rh) / RCELL);
            int z0 = (int)floor((k->z - rh) / RCELL), z1 = (int)floor((k->z + rh) / RCELL);
            for (int ix = x0; ix <= x1; ix++)
                for (int iz = z0; iz <= z1; iz++) {
                    if (pass) cells[ncells] = (RockCell){ cell_key(ix, iz), i };
                    ncells++;
                }
        }
        if (!pass) { cap = ncells; cells = malloc((size_t)(cap ? cap : 1) * sizeof *cells); }
    }
    qsort(cells, (size_t)ncells, sizeof *cells, cmp_cell);
}

int terrain_rocks(const TerrainRock **list)
{
    if (!rocks) build_rocks();
    *list = rocks;
    return nrocks;
}

double terrain_rock_height(double x, double z)
{
    if (!rocks) build_rocks();
    int64_t key = cell_key((int)floor(x / RCELL), (int)floor(z / RCELL));
    int lo = 0, hi = ncells;
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (cells[mid].key < key) lo = mid + 1; else hi = mid;
    }
    double best = 0;
    for (int i = lo; i < ncells && cells[i].key == key; i++) {
        const TerrainRock *k = &rocks[cells[i].idx];
        double dx = x - k->x, dz = z - k->z, rh = 0.5 * (k->sx + k->sz);
        double q = (dx * dx + dz * dz) / (rh * rh);
        if (q >= 1) continue;
        double h = 0.15 * k->r + k->sy * sqrt(1 - q);
        if (h > best) best = h;
    }
    return best;
}
