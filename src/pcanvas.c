// Panel canvas: see pcanvas.h.
//
// Vertex layout (a raylib Mesh, 4 vertices per primitive, 16-bit indices):
//   position   body-frame point
//   texcoord   shape-local coordinates in canvas units (kind 1), font atlas
//              uv (kind 3) or picture uv (kind 4)
//   normal     glow rgb (linear); for pictures x = glow factor
//   color      paint rgba
//   tangent    half width, half height, corner radius, kind
//              (0 plain triangle, 1 rounded rectangle, 3 glyph, 4 picture)
//   texcoord2  x = outline width (0 = filled)

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"
#include "rlgl.h"

// Private copy of the TrueType rasteriser (raylib keeps its own static).
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#define STB_TRUETYPE_IMPLEMENTATION
#define STBTT_STATIC
#include "external/stb_truetype.h"
#pragma GCC diagnostic pop

#include "pcanvas.h"

extern const unsigned char font_rcb[];
extern const unsigned int font_rcb_len;

// --- Signed-distance-field font -------------------------------------------------------

#define SDF_PX 64            // glyph line height in the atlas (pixels)
#define SDF_PAD 12           // distance range on each side of the outline (pixels)
#define SDF_ATLAS 1024
#define FIRST_CH 32
#define LAST_CH 126

typedef struct { float u0, v0, u1, v1, x0, y0, x1, y1, adv; } Glyph;   // quad and advance in atlas pixels
static Glyph glyphs[LAST_CH - FIRST_CH + 1];
static Texture2D font_tex;

static void build_font(void)
{
    stbtt_fontinfo info;
    unsigned char *atlas = calloc(SDF_ATLAS, SDF_ATLAS);
    if (!stbtt_InitFont(&info, font_rcb, 0)) {
        TraceLog(LOG_ERROR, "PCANVAS: panel font could not be parsed");
    } else {
        float scale = stbtt_ScaleForPixelHeight(&info, SDF_PX);
        int ascent, descent, gap;
        stbtt_GetFontVMetrics(&info, &ascent, &descent, &gap);
        float base = ascent * scale;
        int px = 1, py = 1, rowh = 0;
        for (int ch = FIRST_CH; ch <= LAST_CH; ch++) {
            Glyph *g = &glyphs[ch - FIRST_CH];
            int adv, lsb, w = 0, h = 0, xo = 0, yo = 0;
            stbtt_GetCodepointHMetrics(&info, ch, &adv, &lsb);
            g->adv = adv * scale;
            unsigned char *sdf = stbtt_GetCodepointSDF(&info, scale, ch, SDF_PAD, 128, 127.0f / SDF_PAD, &w, &h, &xo, &yo);
            if (!sdf) continue;   // blank (space)
            if (px + w + 1 > SDF_ATLAS) { px = 1; py += rowh + 1; rowh = 0; }
            if (py + h + 1 > SDF_ATLAS) { stbtt_FreeSDF(sdf, NULL); TraceLog(LOG_ERROR, "PCANVAS: font atlas full"); break; }
            for (int y = 0; y < h; y++) memcpy(atlas + (py + y) * SDF_ATLAS + px, sdf + y * w, w);
            stbtt_FreeSDF(sdf, NULL);
            g->u0 = (float)px / SDF_ATLAS; g->v0 = (float)py / SDF_ATLAS;
            g->u1 = (float)(px + w) / SDF_ATLAS; g->v1 = (float)(py + h) / SDF_ATLAS;
            g->x0 = (float)xo; g->y0 = base + yo; g->x1 = (float)(xo + w); g->y1 = base + yo + h;
            px += w + 1;
            if (h > rowh) rowh = h;
        }
    }
    Image img = { atlas, SDF_ATLAS, SDF_ATLAS, 1, PIXELFORMAT_UNCOMPRESSED_GRAYSCALE };
    font_tex = LoadTextureFromImage(img);
    free(atlas);
    GenTextureMipmaps(&font_tex);
    SetTextureWrap(font_tex, TEXTURE_WRAP_CLAMP);
    SetTextureFilter(font_tex, TEXTURE_FILTER_TRILINEAR);
    SetTextureFilter(font_tex, TEXTURE_FILTER_ANISOTROPIC_8X);
}

Texture2D pc_font_texture(void) { return font_tex; }
// Change of the stored distance value per atlas texel (0..1 scale).
float pc_font_slope(void) { return 127.0f / SDF_PAD / 255.0f; }

// --- Layers ---------------------------------------------------------------------------

typedef struct {
    float *pos, *uv, *glow, *par, *uv2;
    unsigned char *col;
    int nv, cap;
    unsigned long long hash;
    int have_hash;
    Mesh mesh;
    int mesh_cap;
} Layer;

static Layer layers[PC_MAX_LAYERS];
static Layer *L;
static V3 pl_o, pl_du, pl_dv, pl_n;
static double lift_m;   // extra lift of the following primitives, metres
static struct { float ox, oy, zoom; } xf[8];
static int nxf;
static float glow_rgb[3], glow_k;
static int glow_mode;   // 0 none, 1 absolute, 2 times the paint colour

#define PAD 0.45f   // quad margin (canvas units) for the antialiased edge

void pc_init(void) { build_font(); }

static void free_mesh(Layer *l)
{
    if (!l->mesh.vaoId && !l->mesh.vboId) return;
    rlUnloadVertexArray(l->mesh.vaoId);
    if (l->mesh.vboId) {
        for (int i = 0; i < 7; i++)
            if (l->mesh.vboId[i]) rlUnloadVertexBuffer(l->mesh.vboId[i]);
        RL_FREE(l->mesh.vboId);
    }
    free(l->mesh.indices);
    memset(&l->mesh, 0, sizeof l->mesh);
    l->mesh_cap = 0;
}

void pc_shutdown(void)
{
    for (int i = 0; i < PC_MAX_LAYERS; i++) {
        Layer *l = &layers[i];
        free_mesh(l);
        free(l->pos); free(l->uv); free(l->glow); free(l->par); free(l->uv2); free(l->col);
        memset(l, 0, sizeof *l);
    }
    UnloadTexture(font_tex);
}

void pc_begin(int layer, V3 origin, V3 du, V3 dv, V3 normal, double lift)
{
    L = &layers[layer];
    L->nv = 0;
    pl_o = v3_add(origin, v3_scale(normal, lift));
    pl_du = du;
    pl_dv = dv;
    pl_n = normal;
    lift_m = 0;
    nxf = 0;
    xf[0].ox = 0; xf[0].oy = 0; xf[0].zoom = 1;
    glow_mode = 0;
}

void pc_push(float ox, float oy, float zoom)
{
    if (nxf >= 7) return;
    float pz = xf[nxf].zoom;
    xf[nxf + 1].ox = xf[nxf].ox + ox * pz;
    xf[nxf + 1].oy = xf[nxf].oy + oy * pz;
    xf[nxf + 1].zoom = pz * zoom;
    nxf++;
}

void pc_pop(void) { if (nxf > 0) nxf--; }

void pc_glow(Color c)
{
    glow_mode = 1;
    glow_rgb[0] = c.r / 255.0f; glow_rgb[1] = c.g / 255.0f; glow_rgb[2] = c.b / 255.0f;
}
void pc_glow_paint(float k) { glow_mode = 2; glow_k = k; }
void pc_glow_off(void) { glow_mode = 0; }

void pc_lift(float units)
{
    lift_m = units * xf[nxf].zoom * v3_len(pl_du);
}

static void grow(void)
{
    L->cap = L->cap ? L->cap * 2 : 4096;
    L->pos = realloc(L->pos, sizeof(float) * 3 * L->cap);
    L->uv = realloc(L->uv, sizeof(float) * 2 * L->cap);
    L->glow = realloc(L->glow, sizeof(float) * 3 * L->cap);
    L->par = realloc(L->par, sizeof(float) * 4 * L->cap);
    L->uv2 = realloc(L->uv2, sizeof(float) * 2 * L->cap);
    L->col = realloc(L->col, 4 * L->cap);
}

// One vertex: canvas point (already transformed), attribute values.
static void vert(float cx, float cy, float u, float v, const float par[4], float stroke, Color c)
{
    int k = L->nv++;
    L->pos[3 * k] = (float)(pl_o.x + pl_du.x * cx + pl_dv.x * cy + pl_n.x * lift_m);
    L->pos[3 * k + 1] = (float)(pl_o.y + pl_du.y * cx + pl_dv.y * cy + pl_n.y * lift_m);
    L->pos[3 * k + 2] = (float)(pl_o.z + pl_du.z * cx + pl_dv.z * cy + pl_n.z * lift_m);
    L->uv[2 * k] = u; L->uv[2 * k + 1] = v;
    float *g = &L->glow[3 * k];
    if (glow_mode == 1) { g[0] = glow_rgb[0]; g[1] = glow_rgb[1]; g[2] = glow_rgb[2]; }
    else if (glow_mode == 2) { g[0] = c.r / 255.0f * glow_k; g[1] = c.g / 255.0f * glow_k; g[2] = c.b / 255.0f * glow_k; }
    else { g[0] = g[1] = g[2] = 0; }
    memcpy(&L->par[4 * k], par, sizeof(float) * 4);
    L->uv2[2 * k] = stroke; L->uv2[2 * k + 1] = 0;
    L->col[4 * k] = c.r; L->col[4 * k + 1] = c.g; L->col[4 * k + 2] = c.b; L->col[4 * k + 3] = c.a;
}

static int room(void)
{
    if (!L) return 0;
    if (L->nv + 4 > 65532) return 0;   // 16-bit indices
    if (L->nv + 4 > L->cap) grow();
    return 1;
}

// Rounded rectangle about `c` (local units) with its x axis along unit vector `ax`.
static void shape(Vector2 c, Vector2 ax, float hx, float hy, float radius, float stroke, Color col)
{
    if (!room() || col.a == 0) return;
    float z = xf[nxf].zoom;
    float ccx = xf[nxf].ox + c.x * z, ccy = xf[nxf].oy + c.y * z;
    hx *= z; hy *= z; radius *= z; stroke *= z;
    float par[4] = { hx, hy, radius, 1 };
    float ex = hx + PAD, ey = hy + PAD;
    static const float sx[4] = { -1, 1, 1, -1 }, sy[4] = { -1, -1, 1, 1 };
    for (int i = 0; i < 4; i++) {
        float lx = sx[i] * ex, ly = sy[i] * ey;
        vert(ccx + ax.x * lx - ax.y * ly, ccy + ax.y * lx + ax.x * ly, lx, ly, par, stroke, col);
    }
}

void pc_rect(float x, float y, float w, float h, Color c)
{
    if (w < 0) { x += w; w = -w; }
    if (h < 0) { y += h; h = -h; }
    shape((Vector2){ x + w / 2, y + h / 2 }, (Vector2){ 1, 0 }, w / 2, h / 2, 0, 0, c);
}

void pc_rect_lines(Rectangle r, float t, Color c)
{
    pc_rect(r.x, r.y, r.width, t, c);
    pc_rect(r.x, r.y + r.height - t, r.width, t, c);
    pc_rect(r.x, r.y + t, t, r.height - 2 * t, c);
    pc_rect(r.x + r.width - t, r.y + t, t, r.height - 2 * t, c);
}

void pc_rrect(Rectangle r, float roundness, Color c)
{
    float rad = roundness * fminf(r.width, r.height) / 2;
    shape((Vector2){ r.x + r.width / 2, r.y + r.height / 2 }, (Vector2){ 1, 0 }, r.width / 2, r.height / 2, rad, 0, c);
}

void pc_rrect_lines(Rectangle r, float roundness, float t, Color c)
{
    float rad = roundness * fminf(r.width, r.height) / 2;
    shape((Vector2){ r.x + r.width / 2, r.y + r.height / 2 }, (Vector2){ 1, 0 }, r.width / 2 + t, r.height / 2 + t, rad + t, t, c);
}

void pc_line(Vector2 a, Vector2 b, float t, Color c)
{
    float dx = b.x - a.x, dy = b.y - a.y, len = sqrtf(dx * dx + dy * dy);
    if (len < 1e-6f) return;
    shape((Vector2){ (a.x + b.x) / 2, (a.y + b.y) / 2 }, (Vector2){ dx / len, dy / len }, len / 2, t / 2, 0, 0, c);
}

void pc_circle(Vector2 centre, float radius, Color c)
{
    shape(centre, (Vector2){ 1, 0 }, radius, radius, radius, 0, c);
}

void pc_ring(Vector2 centre, float radius, float t, Color c)
{
    shape(centre, (Vector2){ 1, 0 }, radius, radius, radius, t, c);
}

void pc_tri(Vector2 a, Vector2 b, Vector2 c, Color col)
{
    if (!room()) return;
    float z = xf[nxf].zoom, ox = xf[nxf].ox, oy = xf[nxf].oy;
    static const float par[4] = { 0, 0, 0, 0 };
    vert(ox + a.x * z, oy + a.y * z, 0, 0, par, 0, col);
    vert(ox + b.x * z, oy + b.y * z, 0, 0, par, 0, col);
    vert(ox + c.x * z, oy + c.y * z, 0, 0, par, 0, col);
    vert(ox + c.x * z, oy + c.y * z, 0, 0, par, 0, col);
}

static Vector2 poly_pt(Vector2 c, int sides, float r, float rot_deg, int i)
{
    float a = rot_deg * DEG2RAD + 2 * PI * i / sides;
    return (Vector2){ c.x + cosf(a) * r, c.y + sinf(a) * r };
}

void pc_poly(Vector2 centre, int sides, float radius, float rot_deg, Color c)
{
    for (int i = 0; i < sides; i++)
        pc_tri(centre, poly_pt(centre, sides, radius, rot_deg, i + 1), poly_pt(centre, sides, radius, rot_deg, i), c);
}

void pc_poly_lines(Vector2 centre, int sides, float radius, float rot_deg, float t, Color c)
{
    for (int i = 0; i < sides; i++)
        pc_line(poly_pt(centre, sides, radius, rot_deg, i), poly_pt(centre, sides, radius, rot_deg, i + 1), t, c);
}

// --- Text ---------------------------------------------------------------------------------

static const Glyph *glyph_of(unsigned char ch)
{
    if (ch < FIRST_CH || ch > LAST_CH) ch = '?';
    return &glyphs[ch - FIRST_CH];
}

static float line_width(const char *s, int n, float size, float spacing)
{
    float k = size / SDF_PX, w = 0;
    for (int i = 0; i < n; i++) w += glyph_of((unsigned char)s[i])->adv * k + (i + 1 < n ? spacing : 0);
    return w;
}

Vector2 pc_measure(const char *s, float size, float spacing)
{
    float wmax = 0;
    int lines = 0;
    while (1) {
        const char *nl = strchr(s, '\n');
        int n = nl ? (int)(nl - s) : (int)strlen(s);
        float w = line_width(s, n, size, spacing);
        if (w > wmax) wmax = w;
        lines++;
        if (!nl) break;
        s = nl + 1;
    }
    return (Vector2){ wmax, size * (1 + (lines - 1) * 1.12f) };
}

static void text_line(const char *s, int n, float x, float y, float size, float spacing, Color c)
{
    float k = size / SDF_PX, z = xf[nxf].zoom, ox = xf[nxf].ox, oy = xf[nxf].oy;
    static const float par[4] = { 0, 0, 0, 3 };
    for (int i = 0; i < n; i++) {
        const Glyph *g = glyph_of((unsigned char)s[i]);
        if (g->u1 > g->u0 && room()) {
            float x0 = ox + (x + g->x0 * k) * z, y0 = oy + (y + g->y0 * k) * z;
            float x1 = ox + (x + g->x1 * k) * z, y1 = oy + (y + g->y1 * k) * z;
            vert(x0, y0, g->u0, g->v0, par, 0, c);
            vert(x1, y0, g->u1, g->v0, par, 0, c);
            vert(x1, y1, g->u1, g->v1, par, 0, c);
            vert(x0, y1, g->u0, g->v1, par, 0, c);
        }
        x += g->adv * k + spacing;
    }
}

void pc_text(const char *s, float x, float y, float size, float spacing, Color c)
{
    while (1) {
        const char *nl = strchr(s, '\n');
        int n = nl ? (int)(nl - s) : (int)strlen(s);
        text_line(s, n, x, y, size, spacing, c);
        if (!nl) break;
        s = nl + 1;
        y += size * 1.12f;
    }
}

void pc_text_box(const char *s, Rectangle r, float size, float spacing, Color c)
{
    int lines = 1;
    for (const char *p = s; *p; p++) lines += *p == '\n';
    float y = r.y + (r.height - size * (1 + (lines - 1) * 1.12f)) / 2;
    while (1) {
        const char *nl = strchr(s, '\n');
        int n = nl ? (int)(nl - s) : (int)strlen(s);
        text_line(s, n, r.x + (r.width - line_width(s, n, size, spacing)) / 2, y, size, spacing, c);
        if (!nl) break;
        s = nl + 1;
        y += size * 1.12f;
    }
}

void pc_image(Rectangle d)
{
    if (!room()) return;
    float z = xf[nxf].zoom, ox = xf[nxf].ox, oy = xf[nxf].oy;
    static const float par[4] = { 0, 0, 0, 4 };
    int mode = glow_mode;
    float k = mode == 2 ? glow_k : 0;
    glow_mode = 1;
    glow_rgb[0] = k; glow_rgb[1] = 0; glow_rgb[2] = 0;
    float x0 = ox + d.x * z, y0 = oy + d.y * z, x1 = ox + (d.x + d.width) * z, y1 = oy + (d.y + d.height) * z;
    vert(x0, y0, 0, 1, par, 0, WHITE);
    vert(x1, y0, 1, 1, par, 0, WHITE);
    vert(x1, y1, 1, 0, par, 0, WHITE);
    vert(x0, y1, 0, 0, par, 0, WHITE);
    glow_mode = mode;
}

// --- Upload and draw ---------------------------------------------------------------------

static unsigned long long hash_bytes(unsigned long long h, const void *data, size_t n)
{
    const unsigned char *p = data;
    size_t i = 0;
    for (; i + 8 <= n; i += 8) {
        unsigned long long w;
        memcpy(&w, p + i, 8);
        h = (h ^ w) * 0x9E3779B97F4A7C15ull;
        h ^= h >> 29;
    }
    for (; i < n; i++) h = (h ^ p[i]) * 0x100000001B3ull;
    return h;
}

static void make_mesh(Layer *l)
{
    free_mesh(l);
    int cap = l->cap;
    Mesh m = { 0 };
    m.vertexCount = cap;
    m.triangleCount = cap / 2;
    m.vertices = l->pos; m.texcoords = l->uv; m.normals = l->glow; m.colors = l->col; m.tangents = l->par; m.texcoords2 = l->uv2;
    m.indices = malloc(sizeof(unsigned short) * (size_t)(cap / 4) * 6);
    for (int q = 0; q < cap / 4; q++) {
        unsigned short *ix = &m.indices[q * 6], b = (unsigned short)(q * 4);
        ix[0] = b; ix[1] = (unsigned short)(b + 1); ix[2] = (unsigned short)(b + 2);
        ix[3] = b; ix[4] = (unsigned short)(b + 2); ix[5] = (unsigned short)(b + 3);
    }
    UploadMesh(&m, true);
    l->mesh = m;
    l->mesh_cap = cap;
}

int pc_end(void)
{
    Layer *l = L;
    L = NULL;
    if (!l) return 0;
    size_t n = (size_t)l->nv;
    unsigned long long h = 1469598103934665603ull ^ n;
    h = hash_bytes(h, l->pos, n * 12);
    h = hash_bytes(h, l->uv, n * 8);
    h = hash_bytes(h, l->glow, n * 12);
    h = hash_bytes(h, l->par, n * 16);
    h = hash_bytes(h, l->uv2, n * 8);
    h = hash_bytes(h, l->col, n * 4);
    if (l->have_hash && h == l->hash && l->mesh_cap == l->cap) return 0;
    l->hash = h;
    l->have_hash = 1;
    if (!l->cap) return 0;
    if (l->mesh_cap != l->cap) {
        make_mesh(l);   // uploads the arrays as they are
    } else {
        // The arrays may have moved if they grew; the mesh only borrows them.
        l->mesh.vertices = l->pos; l->mesh.texcoords = l->uv; l->mesh.normals = l->glow;
        l->mesh.colors = l->col; l->mesh.tangents = l->par; l->mesh.texcoords2 = l->uv2;
        UpdateMeshBuffer(l->mesh, 0, l->pos, (int)(n * 12), 0);
        UpdateMeshBuffer(l->mesh, 1, l->uv, (int)(n * 8), 0);
        UpdateMeshBuffer(l->mesh, 2, l->glow, (int)(n * 12), 0);
        UpdateMeshBuffer(l->mesh, 3, l->col, (int)(n * 4), 0);
        UpdateMeshBuffer(l->mesh, 4, l->par, (int)(n * 16), 0);
        UpdateMeshBuffer(l->mesh, 5, l->uv2, (int)(n * 8), 0);
    }
    return 1;
}

int pc_layer_ready(int layer) { return layers[layer].mesh.vaoId != 0 && layers[layer].nv > 0; }
int pc_layer_quads(int layer) { return layers[layer].nv / 4; }

void pc_draw_layer(int layer, Material mat, Matrix transform)
{
    Layer *l = &layers[layer];
    if (!l->mesh.vaoId || l->nv <= 0) return;
    Mesh m = l->mesh;
    m.vertexCount = l->nv;
    m.triangleCount = l->nv / 2;
    DrawMesh(m, mat, transform);
}
