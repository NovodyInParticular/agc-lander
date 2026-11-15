#include <string.h>

#include "raylib.h"
#include "rlgl.h"

#include "assets.h"

static int missing, loaded;
int assets_missing(void) { return missing; }
int assets_loaded(void) { return loaded; }

// 1x1 "no detail" map: flat normal, no roughness offset, albedo x1.
static Texture2D neutral_map(void)
{
    static const unsigned char px[4] = { 128, 128, 128, 128 };
    Image img = { (void *)px, 1, 1, 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 };
    return LoadTextureFromImage(img);
}

static void finish(Texture2D *t)
{
    if (t->mipmaps <= 1) GenTextureMipmaps(t);
    SetTextureWrap(*t, TEXTURE_WRAP_REPEAT);
    SetTextureFilter(*t, TEXTURE_FILTER_TRILINEAR);
    SetTextureFilter(*t, TEXTURE_FILTER_ANISOTROPIC_8X);
}

#ifndef AGC_ASSETS_FROM_APK

#define EMBED(n) extern const unsigned char tex_##n[]; extern const unsigned int tex_##n##_len;
EMBED(lm_panel) EMBED(lm_foil) EMBED(lm_blanket) EMBED(rock) EMBED(cabin) EMBED(cabin_wall)
#undef EMBED

Texture2D assets_texture(const char *name)
{
#define ROW(n) { #n, tex_##n, &tex_##n##_len }
    static const struct { const char *name; const unsigned char *data; const unsigned int *len; } table[] = {
        ROW(lm_panel), ROW(lm_foil), ROW(lm_blanket), ROW(rock), ROW(cabin), ROW(cabin_wall),
    };
#undef ROW
    for (unsigned i = 0; i < sizeof table / sizeof table[0]; i++) {
        if (strcmp(table[i].name, name)) continue;
        Image img = LoadImageFromMemory(".png", table[i].data, (int)*table[i].len);
        if (!img.data) break;
        Texture2D t = LoadTextureFromImage(img);
        UnloadImage(img);
        if (!t.id) break;
        finish(&t);
        loaded++;
        TraceLog(LOG_INFO, "ASSETS: texture %s loaded (%dx%d, %d mips, embedded)", name, t.width, t.height, t.mipmaps);
        return t;
    }
    missing++;
    TraceLog(LOG_ERROR, "ASSETS: MISSING TEXTURE %s (not embedded or failed to decode)", name);
    return neutral_map();
}

#endif
