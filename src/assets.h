// Game assets: surface maps baked by tools/bake_textures.py.
//
// On the PC the files are embedded in the executable (it stays a single
// file); on Android they are read from the APK through the asset manager,
// compressed as ASTC.
#ifndef ASSETS_H
#define ASSETS_H

#include "raylib.h"

// Tileable surface map by name ("lm_panel", "lm_foil", "lm_blanket", "rock",
// "cabin", "cabin_wall", "regolith"): repeat wrap, full mip chain, anisotropic filtering. Logs and
// returns a 1x1 neutral map if the asset cannot be loaded, so a missing
// texture is always reported and never shows as garbage.
Texture2D assets_texture(const char *name);

// Number of asset loads that failed so far (0 = everything is present).
int assets_missing(void);
// Number of asset loads that succeeded.
int assets_loaded(void);

#endif
