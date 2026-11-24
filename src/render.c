// Scene rendering: HDR target, logarithmic depth, sky, bloom, tonemapping,
// sun shadow map. All shaders are embedded so the game stays a single exe.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "assets.h"
#include "render.h"

#ifndef AGC_GLES
// Multisampled scene target (PC): raw GL calls through the loader raylib already set up.
#include "external/glad.h"
#define HAVE_MSAA 1
#endif

#define FAR_PLANE 2.0e8
#define NEAR_PLANE 0.25

// ---------------------------------------------------------------------------
// Shared GLSL.

// Every shader gets its version line, precision and the output function at load
// time (render_load_shader), so one source serves desktop GL 3.3 and GLES 3.0.
#define GLSL_HEADER ""

#define LOGZ_VS \
    "uniform float logFar;\n" \
    "out float fragLogZ;\n"
#define LOGZ_FS \
    "uniform float logFar;\n" \
    "in float fragLogZ;\n" \
    "#if Q_VZ\n" \
    "void writeDepth(){}\n" \
    "#else\n" \
    "void writeDepth(){ gl_FragDepth = log2(fragLogZ) * logFar * 0.5; }\n" \
    "#endif\n"

// The crew's own shadow map (render_shadow2_*): a small, sharp map that follows
// the astronauts, so their long low-Sun shadows land wherever they walk.
#define CREW_SHADOW_GLSL \
    "uniform mat4 shadow2VP; uniform sampler2D shadowMap2; uniform vec3 shadow2Center; uniform float shadow2On;\n" \
    "float crewShadow(vec3 p){\n" \
    "  if (shadow2On < 0.5) return 1.0;\n" \
    "  vec4 sp = shadow2VP * vec4(p - shadow2Center, 1.0);\n" \
    "  vec3 c = sp.xyz / sp.w * 0.5 + 0.5;\n" \
    "  if (c.x < 0.0 || c.x > 1.0 || c.y < 0.0 || c.y > 1.0 || c.z > 1.0) return 1.0;\n" \
    "  float s = 0.0; vec2 texel = 1.0 / vec2(textureSize(shadowMap2, 0));\n" \
    "  for (int k=0;k<4;k++){ vec2 o = vec2(k & 1, k >> 1) * 2.0 - 1.0; float d = texture(shadowMap2, c.xy + o*texel*0.9).r; s += (c.z - 0.0012 > d) ? 0.0 : 1.0; }\n" \
    "  return s / 4.0;\n" \
    "}\n"

#define NOISE_GLSL \
    "float hash12(vec2 p){ vec3 p3 = fract(vec3(p.xyx) * 0.1031); p3 += dot(p3, p3.yzx + 33.33); return fract((p3.x + p3.y) * p3.z); }\n" \
    "float hash13(vec3 p3){ p3 = fract(p3 * 0.1031); p3 += dot(p3, p3.zyx + 31.32); return fract((p3.x + p3.y) * p3.z); }\n" \
    "float vnoise(vec2 p){ vec2 i = floor(p), f = fract(p); f = f*f*(3.0-2.0*f);\n" \
    "  float a = hash12(i), b = hash12(i+vec2(1,0)), c = hash12(i+vec2(0,1)), d = hash12(i+vec2(1,1));\n" \
    "  return mix(mix(a,b,f.x), mix(c,d,f.x), f.y); }\n" \
    "float vnoise3(vec3 p){ vec3 i = floor(p), f = fract(p); f = f*f*(3.0-2.0*f);\n" \
    "  float n000=hash13(i), n100=hash13(i+vec3(1,0,0)), n010=hash13(i+vec3(0,1,0)), n110=hash13(i+vec3(1,1,0));\n" \
    "  float n001=hash13(i+vec3(0,0,1)), n101=hash13(i+vec3(1,0,1)), n011=hash13(i+vec3(0,1,1)), n111=hash13(i+vec3(1,1,1));\n" \
    "  return mix(mix(mix(n000,n100,f.x),mix(n010,n110,f.x),f.y), mix(mix(n001,n101,f.x),mix(n011,n111,f.x),f.y), f.z); }\n" \
    "float fbm(vec2 p){ float s=0.0, a=0.5; for(int i=0;i<Q_FBM;i++){ s+=a*vnoise(p); p*=2.03; a*=0.5;} return s + a; }\n" \
    "float fbm3(vec3 p){ float s=0.0, a=0.5; for(int i=0;i<5;i++){ s+=a*vnoise3(p); p*=2.03; a*=0.5;} return s; }\n"

// ---------------------------------------------------------------------------
// Terrain.

static const char *terrain_vs = GLSL_HEADER LOGZ_VS
    "in vec3 vertexPosition; in vec3 vertexNormal; in vec4 vertexColor;\n"
    "in vec4 vertexTangent; in vec2 vertexTexCoord; in vec2 vertexTexCoord2;\n"   // morph target (parent chunk's surface)
    "uniform mat4 mvp; uniform mat4 matModel; uniform vec3 morph;\n"             // morph: start, end distance; fade-in 0..1
    "uniform vec4 chunk;\n"                                                      // x: vertex spacing (m)
    "out vec3 fragPos; out vec3 fragNormal; out vec4 fragColor; out vec2 planeRel; out float morphK;\n"
    "void main(){\n"
    "  // Position in the terrain plane relative to the chunk corner, from the\n"
    "  // vertex number (33 x 33 grid, then four skirt edges).\n"
    "  int id = gl_VertexID; ivec2 ij;\n"
    "  if (id < 1089) ij = ivec2(id % 33, id / 33);\n"
    "  else { int e = (id - 1089) / 33, t = (id - 1089) % 33;\n"
    "    ij = e == 0 ? ivec2(t, 0) : e == 1 ? ivec2(32, t) : e == 2 ? ivec2(32 - t, 32) : ivec2(0, 32 - t); }\n"
    "  planeRel = vec2(ij) * chunk.x;\n"
    "  // Level-of-detail morph: far vertices, and those of a chunk that has just\n"
    "  // appeared, sit on the coarser parent surface, so nothing ever pops.\n"
    "  float dv = length((matModel * vec4(vertexPosition, 1.0)).xyz);\n"
    "  float m = max(smoothstep(morph.x, morph.y, dv), 1.0 - morph.z);\n"
    "  morphK = m;\n"
    "  vec3 pos = vertexPosition + vertexTangent.xyz * m;\n"
    "  vec3 cn = vec3(vertexTexCoord.x, sqrt(max(1.0 - dot(vertexTexCoord, vertexTexCoord), 0.0)), vertexTexCoord.y);\n"
    "  vec4 wp = matModel * vec4(pos, 1.0);\n"
    "  fragPos = wp.xyz; fragNormal = normalize(mix(vertexNormal, cn, m));\n"
    "  fragColor = mix(vertexColor, vec4(vertexTexCoord2.x, vertexTangent.w, vertexTexCoord2.y, 1.0), m);\n"
    "  gl_Position = mvp * vec4(pos, 1.0);\n"
    "  fragLogZ = 1.0 + gl_Position.w;\n"
    "}\n";

static const char *terrain_fs = GLSL_HEADER LOGZ_FS NOISE_GLSL
    "in vec3 fragPos; in vec3 fragNormal; in vec4 fragColor; in vec2 planeRel; in float morphK;\n"
    "uniform vec3 sunDir; uniform vec3 camMod; uniform float exposure;\n"
    "uniform vec4 chunk; uniform ivec2 cellBase[6]; uniform vec2 cellFrac[6]; uniform int dbg;\n"
    "// Crater octaves the mesh has not taken over yet: same cells, hash and\n"
    "// profile as terrain.c (octaves[1..6]), evaluated per pixel.\n"
    "const float OCT_CELL[6] = float[6](6000.0, 1500.0, 400.0, 100.0, 25.0, 6.0);\n"
    "const float OCT_PROB[6] = float[6](0.45, 0.55, 0.60, 0.65, 0.65, 0.55);\n"
    "const float OCT_GK[6] = float[6](0.15, 0.15, 0.15, 0.15, 0.15, 0.38);\n"
    "uint chash(ivec2 c, uint seed){ uint h = uint(c.x) * 0x8da6b343u ^ uint(c.y) * 0xd8163841u ^ seed * 0xcb1ab31fu;\n"
    "  h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15; return h; }\n"
    "float cu01(uint h){ return float(h & 0xffffffu) / 16777216.0; }\n"
    "void craters(float fw, vec2 su, float ss, out vec2 slope, out float shadow, out float fresh){\n"
    "  slope = vec2(0.0); shadow = 1.0; fresh = 0.0;\n"
    "  if (dbg == 6) return;\n"
    "  for (int i = Q_OCT0; i < Q_OCT1; i++) {\n"
    "    float cell = OCT_CELL[i], lim = cell * OCT_GK[i];\n"
    "    float inGeo = mix(step(1.5 * chunk.x, lim), step(3.0 * chunk.x, lim), morphK);\n"
    "    float pix = smoothstep(2.5, 7.0, cell / fw);\n"
    "    float w = (1.0 - inGeo) * pix;\n"
    "    // A crater the mesh has only just taken over is still too coarse for a\n"
    "    // clean shadow edge: keep drawing its shadow here until the mesh is fine.\n"
    "    float ws = max(w, pix * (1.0 - smoothstep(12.0, 48.0, cell / (chunk.x * (1.0 + morphK)))));\n"
    "    if (ws < 0.004) continue;\n"
    "    uint seed = uint(12 + i);\n"
    "    vec2 p = cellFrac[i] + planeRel / cell;\n"
    "    vec2 fl = floor(p), f = p - fl;\n"
    "    ivec2 c0 = cellBase[i] + ivec2(fl);\n"
    "    ivec2 o = ivec2(f.x < 0.5 ? -1 : 0, f.y < 0.5 ? -1 : 0);\n"   // the 2 x 2 cells that can reach this point
    "    for (int k = 0; k < 4; k++) {\n"
    "      ivec2 dc = o + ivec2(k & 1, k >> 1); ivec2 c = c0 + dc;\n"
    "      if (cu01(chash(c, seed)) > OCT_PROB[i]) continue;\n"
    "      uint h2 = chash(c, seed + 101u), h3 = chash(c, seed + 202u);\n"
    "      vec2 ctr = vec2(dc) + vec2(0.15 + 0.7 * cu01(h2), 0.15 + 0.7 * cu01(h2 >> 7 | h2 << 25));\n"
    "      float rr = 0.08 + 0.30 * cu01(h3) * cu01(h3 >> 9 | h3 << 23);\n"
    "      vec2 d = f - ctr; float dl = length(d), r = dl / rr;\n"
    "      if (r > 1.9) continue;\n"
    "      // A crater only a few pixels across can't be drawn steadily: it would\n"
    "      // sparkle as the view moves (worst in a headset). Fade it by its own size.\n"
    "      float pxd = 2.0 * rr * cell / fw;\n"
    "      float aa = smoothstep(Q_CRATER_PX0, Q_CRATER_PX1, pxd);\n"
    "      if (aa < 0.004) continue;\n"
    "      float fr = cu01(chash(c, seed + 303u)); fr = fr * fr * fr;\n"
    "      float depth = rr * (0.10 + 0.30 * fr), rim = depth * (0.18 + 0.12 * fr);\n"
    "      vec2 dir = d / max(dl, 1e-6); float g;\n"
    "      if (r < 1.0) {\n"
    "        float r2 = r * r;\n"
    "        g = 2.0 * depth * r + 6.0 * rim * r2 * r2 * r;\n"
    "        if (-depth * (1.0 - r2) + rim * r2 * r2 * r2 < -0.82 * depth) g *= 0.15;\n"   // flat floor
    "        // Shadow thrown across the bowl by its own sunward wall (parabolic bowl, exact).\n"
    "        float pu = dot(d, su);\n"
    "        float tstar = ss * rr * rr / depth - 2.0 * pu;\n"
    "        float texit = -pu + sqrt(max(pu * pu + rr * rr - dl * dl, 0.0));\n"
    "        float sw = max(1.5 * fw / cell, 0.07 * rr);\n"
    "        shadow *= mix(1.0, smoothstep(-sw, sw, tstar - texit), ws * aa);\n"
    "      } else { float e = 1.0 / (1.0 + 5.0 * (r - 1.0)); e *= e; g = -15.0 * rim * e * e; }\n"
    "      slope -= dir * (g / rr) * w * aa;\n"
    "      float halo = r < 1.0 ? 1.0 : (1.9 - r) / 0.9;\n"
    "      fresh = max(fresh, fr * halo * w * aa * (rr * cell < 400.0 ? 1.0 : 0.4));\n"
    "    }\n"
    "  }\n"
    "}\n"
    "uniform mat4 shadowVP; uniform sampler2D shadowMap; uniform vec3 shadowCenter; uniform float shadowOn;\n"
    "uniform vec3 dustCenter; uniform float dustAmount;\n"
    "uniform sampler2D soilMap;\n"
    CREW_SHADOW_GLSL
    "out vec4 finalColor;\n"
    "float lmShadow1(vec3 p);\n"
    "float lmShadow(vec3 p){ return min(lmShadow1(p), crewShadow(p)); }\n"
    "float lmShadow1(vec3 p){\n"
    "  if (shadowOn < 0.5) return 1.0;\n"
    "  vec4 sp = shadowVP * vec4(p - shadowCenter, 1.0);\n"
    "  vec3 c = sp.xyz / sp.w * 0.5 + 0.5;\n"
    "  if (c.x < 0.0 || c.x > 1.0 || c.y < 0.0 || c.y > 1.0 || c.z > 1.0) return 1.0;\n"
    "  float s = 0.0; vec2 texel = 1.0 / vec2(textureSize(shadowMap, 0));\n"
    "#if Q_PCF == 9\n"
    "  for (int x=-1;x<=1;x++) for(int y=-1;y<=1;y++){ float d = texture(shadowMap, c.xy + vec2(x,y)*texel*1.5).r; s += (c.z - 0.002 > d) ? 0.0 : 1.0; }\n"
    "  return s / 9.0;\n"
    "#else\n"
    "  for (int k=0;k<4;k++){ vec2 o = vec2(k & 1, k >> 1) * 2.0 - 1.0; float d = texture(shadowMap, c.xy + o*texel*1.1).r; s += (c.z - 0.002 > d) ? 0.0 : 1.0; }\n"
    "  return s / 4.0;\n"
    "#endif\n"
    "}\n"
    "void main(){\n"
    "  if (dbg != 5) writeDepth();\n"
    "  float dist = length(fragPos);\n"
    "  vec3 N = normalize(fragNormal);\n"
    "  vec3 V = -fragPos / max(dist, 1e-3);\n"
    "  vec2 dp = camMod.xz + fragPos.xz;\n"
    "  // Soil detail from the baked regolith map at three scales (grain and\n"
    "  // pebbles underfoot, metre-size pits, decametre swells), each turned so\n"
    "  // the repeats never line up.\n"
    "  float near = 1.0 - smoothstep(20.0, 160.0, dist);\n"
    "  float mid = 1.0 - smoothstep(250.0, 3000.0, dist);\n"
    "  mat2 r2 = mat2(0.80, 0.60, -0.60, 0.80), r3 = mat2(0.28, -0.96, 0.96, 0.28);\n"
    "  vec4 s1 = texture(soilMap, dp / 3.2);\n"
    "#if Q_SOIL >= 3\n"
    "  vec4 s2 = texture(soilMap, r2 * dp / 29.0);\n"
    "#else\n"
    "  vec4 s2 = vec4(0.5);\n"
    "#endif\n"
    "  vec4 s3 = texture(soilMap, r3 * dp / 310.0);\n"
    "  vec2 sg = (s1.rg * 2.0 - 1.0) * 0.30 * near + ((s2.rg * 2.0 - 1.0) * r2) * 0.26 * mid + ((s3.rg * 2.0 - 1.0) * r3) * 0.16;\n"
    "  vec3 Ng = N;\n"
    "  float fw = max(length(dFdx(planeRel)), length(dFdy(planeRel)));\n"
    "  vec2 sunH = sunDir.xz; float sunHl = max(length(sunH), 1e-3);\n"
    "  vec2 cslope; float cshadow, cfresh;\n"
    "  craters(fw, sunH / sunHl, sunDir.y / sunHl, cslope, cshadow, cfresh);\n"
    "  N = normalize(N + vec3(sg.x + cslope.x, 0.0, sg.y + cslope.y));\n"
    "  float albedo = fragColor.r * 1.0;\n"   // stored as albedo*1000/255
    "  albedo = albedo * 0.255 + 0.07 * cfresh;\n"
    "  float fresh = fragColor.b;\n"
    "  // Albedo mottling: broad dark and bright patches, then the soil's own tone.\n"
    "  albedo *= 0.84 + 0.32 * fbm(dp * 0.013);\n"
    "  albedo *= mix(1.0, s3.a * 2.0, 0.55) * mix(1.0, s2.a * 2.0, 0.55 * mid) * mix(1.0, s1.a * 2.0, 0.85 * near);\n"
    "  // Lommel-Seeliger with an opposition surge (the lunar 'flat' look).\n"
    "  // The grain is far smaller than a pixel almost everywhere, so a facet turned\n"
    "  // away from the low Sun must fade out, not snap to black: soft-clip the\n"
    "  // detail term and tie it to the true slope so the two never disagree.\n"
    "  float mug = max(dot(Ng, sunDir), 0.0);\n"
    "  float mud = dot(N, sunDir);\n"
    "  float kk = 0.045 + 0.045 * smoothstep(8.0, 120.0, dist);\n"
    "  float mu0 = kk * log(1.0 + exp(min(mud / kk, 30.0)));\n"

    "  float mu = max(dot(N, V), 0.02);\n"
    "  float g = acos(clamp(dot(sunDir, V), -1.0, 1.0));\n"
    "  float surge = 1.0 + 0.9 * exp(-g / 0.06) + 0.35 * exp(-g / 0.5);\n"
    "  float ls = mu0 / (mu0 + mu) * 2.0;\n"
    "  float sh = smoothstep(0.0, 1.0, fragColor.g) * lmShadow(fragPos) * cshadow;\n"
    "  vec3 sunCol = vec3(1.0, 0.985, 0.96);\n"
    "  vec3 regolith = vec3(1.0, 0.93, 0.84);\n"
    "  vec3 col = sunCol * regolith * albedo * ls * surge * sh * 2.6;\n"
    "  // Faint fill from the lit surroundings and earthshine.\n"
    "  col += vec3(0.62,0.66,0.75) * albedo * 0.035 * (0.6 + 0.4 * N.y);\n"
    "  // Blown dust brightens the ground under the descent engine.\n"
    "  if (dustAmount > 0.0) {\n"
    "    float dd = length(fragPos.xz - dustCenter.xz);\n"
    "    col *= 1.0 + dustAmount * 0.35 * exp(-dd / 25.0);\n"
    "  }\n"
    "  if (dbg == 1) col = fragColor.rgb;\n"
    "  if (dbg == 2) col = N * 0.5 + 0.5;\n"
    "  if (dbg == 3) col = vec3(ls, mu0, mu);\n"
    "  if (dbg == 4 || dbg == 5) col = vec3(1.0, 0.0, 0.0);\n"
    "  finalColor = sceneOut(col * exposure, 1.0);\n"
    "}\n";

// ---------------------------------------------------------------------------
// LM: metals, gold kapton foil, black paint, white thermal paint.

static const char *lm_vs = GLSL_HEADER LOGZ_VS
    "in vec3 vertexPosition; in vec3 vertexNormal; in vec4 vertexColor;\n"
    "uniform mat4 mvp; uniform mat4 matModel; uniform mat4 matNormal;\n"
    "out vec3 fragPos; out vec3 fragNormal; out vec4 fragColor; out vec3 objPos; out vec3 objNormal;\n"
    "void main(){\n"
    "  vec4 wp = matModel * vec4(vertexPosition, 1.0);\n"
    "  fragPos = wp.xyz; fragNormal = normalize((matNormal * vec4(vertexNormal, 0.0)).xyz);\n"
    "  fragColor = vertexColor; objPos = vertexPosition; objNormal = vertexNormal;\n"
    "  gl_Position = mvp * vec4(vertexPosition, 1.0);\n"
    "  fragLogZ = 1.0 + gl_Position.w;\n"
    "}\n";

// Surface map (see tools/bake_textures.py): rg = tangent normal, b = roughness
// offset, a = albedo / 2. Projected along the three object axes and blended by
// the normal, so the meshes need no texture coordinates. Explicit gradients
// keep the lookups valid inside branches.
#define TRIMAP_GLSL \
    "#if Q_TRI1\n" \
    "vec4 triMap(sampler2D t, vec3 p, vec3 n, out vec3 bump){\n" \
    "  vec3 a = abs(n); vec4 c;\n" \
    "  if (a.x >= a.y && a.x >= a.z) { c = texture(t, p.yz); bump = vec3(0.0, c.rg * 2.0 - 1.0); }\n" \
    "  else if (a.y >= a.z) { c = texture(t, p.xz); bump = vec3(c.r * 2.0 - 1.0, 0.0, c.g * 2.0 - 1.0); }\n" \
    "  else { c = texture(t, p.xy); bump = vec3(c.rg * 2.0 - 1.0, 0.0); }\n" \
    "  return c;\n" \
    "}\n" \
    "#else\n" \
    "vec4 triMap(sampler2D t, vec3 p, vec3 n, out vec3 bump){\n" \
    "  vec3 w = abs(n); w = w * w; w = w * w * w; w /= (w.x + w.y + w.z);\n" \
    "  vec3 dx = dFdx(p), dy = dFdy(p); vec4 s = vec4(0.0); bump = vec3(0.0);\n" \
    "  if (w.x > 0.004) { vec4 c = textureGrad(t, p.yz, dx.yz, dy.yz); s += c * w.x; bump += vec3(0.0, c.rg * 2.0 - 1.0) * w.x; }\n" \
    "  if (w.y > 0.004) { vec4 c = textureGrad(t, p.xz, dx.xz, dy.xz); s += c * w.y; bump += vec3(c.r * 2.0 - 1.0, 0.0, c.g * 2.0 - 1.0) * w.y; }\n" \
    "  if (w.z > 0.004) { vec4 c = textureGrad(t, p.xy, dx.xy, dy.xy); s += c * w.z; bump += vec3(c.rg * 2.0 - 1.0, 0.0) * w.z; }\n" \
    "  return s;\n" \
    "}\n" \
    "#endif\n"

static const char *lm_fs = GLSL_HEADER LOGZ_FS
    "in vec3 fragPos; in vec3 fragNormal; in vec4 fragColor; in vec3 objPos; in vec3 objNormal;\n"
    "uniform vec3 sunDir; uniform float exposure; uniform float groundAlbedo;\n"
    "uniform sampler2D texFoil; uniform sampler2D texPanel; uniform sampler2D texBlanket; uniform sampler2D texRock;\n"
    "uniform mat4 matModel;\n"
    TRIMAP_GLSL
    "uniform mat4 shadowVP; uniform sampler2D shadowMap; uniform vec3 shadowCenter; uniform float shadowOn;\n"
    "uniform vec3 upDir; uniform float nearCut; uniform float sunGain;\n"
    "uniform vec3 lampPos[4]; uniform vec3 lampCol[4];\n"
    CREW_SHADOW_GLSL
    "out vec4 finalColor;\n"
    "float selfShadow1(vec3 p, vec3 n);\n"
    "float selfShadow(vec3 p, vec3 n){ return min(selfShadow1(p, n), crewShadow(p + n * 0.02)); }\n"
    "float selfShadow1(vec3 p, vec3 n){\n"
    "  if (shadowOn < 0.5) return 1.0;\n"
    "  // Slope-scaled: a face the low Sun only grazes spans many map texels in depth,\n"
    "  // and without a larger offset it shadows itself (the black 'missing' roofs).\n"
    "  float gr = 1.0 - clamp(dot(n, sunDir), 0.0, 1.0);\n"
    "  vec4 sp = shadowVP * vec4(p - shadowCenter + n * (0.04 + 0.14 * gr * gr), 1.0);\n"
    "  vec3 c = sp.xyz / sp.w * 0.5 + 0.5;\n"
    "  if (c.x < 0.0 || c.x > 1.0 || c.y < 0.0 || c.y > 1.0) return 1.0;\n"
    "  float s = 0.0; vec2 texel = 1.0 / vec2(textureSize(shadowMap, 0));\n"
    "#if Q_PCF == 9\n"
    "  float bias = 0.0015 + 0.0035 * gr * gr;\n"
    "  for (int x=-1;x<=1;x++) for(int y=-1;y<=1;y++){ float d = texture(shadowMap, c.xy + vec2(x,y)*texel).r; s += (c.z - bias > d) ? 0.0 : 1.0; }\n"
    "  return s / 9.0;\n"
    "#else\n"
    "  float bias = 0.0015 + 0.0035 * gr * gr;\n"
    "  for (int k=0;k<4;k++){ vec2 o = vec2(k & 1, k >> 1) * 2.0 - 1.0; float d = texture(shadowMap, c.xy + o*texel*0.75).r; s += (c.z - bias > d) ? 0.0 : 1.0; }\n"
    "  return s / 4.0;\n"
    "#endif\n"
    "}\n"
    "vec3 envLight(vec3 dir, float rough){\n"
    "  // Black sky above, sunlit grey regolith below, bright toward the sun-lit ground.\n"
    "  float d = dot(dir, upDir);\n"
    "  vec3 ground = vec3(0.55,0.53,0.50) * groundAlbedo * 2.6;\n"
    "  float t = smoothstep(0.08 + rough * 0.3, -0.15 - rough * 0.3, d);\n"
    "  return ground * t + vec3(0.012);\n"
    "}\n"
    "void main(){\n"
    "  // Inside the cabin (cockpit and docking-window views) the LM's own outer\n"
    "  // skin around the crew compartment is hidden by its shape, not by distance\n"
    "  // from the eye: leaning back must not bring the hull or window glass into view.\n"
    "  if (nearCut > 0.0 && (length(fragPos) < nearCut ||\n"
    "      (all(greaterThan(objPos, vec3(1.30, -1.17, -1.42))) && all(lessThan(objPos, vec3(3.42, 1.17, 1.16)))))) discard;\n"
    "  writeDepth();\n"
    "  int mat = int(fragColor.a * 255.0 + 0.5);\n"
    "  vec3 base = fragColor.rgb;\n"
    "  vec3 N = normalize(fragNormal);\n"
    "  vec3 V = normalize(-fragPos);\n"
    "  if (!gl_FrontFacing) N = -N;\n"
    "  vec3 Ngeo = N;\n"   // the smooth surface: shadow lookups use it, never the bumped normal (that gave speckled shadow acne)
    "#ifdef DBG_MAT\n"   // debugging (AGC_DBG_MAT=1): flat false colour per material, striped every 25 cm along the stage axis
    "  { vec3 pal[14] = vec3[14](vec3(1,1,1), vec3(1,0.6,0), vec3(0.5,0.5,0.5), vec3(0,0,1), vec3(1,1,0.6), vec3(0,1,1), vec3(1,0,1), vec3(1,1,1), vec3(0.4,0.2,0), vec3(0.3,0.3,0), vec3(0,1,0), vec3(1,0,0), vec3(0.6,0,1), vec3(0,0.4,0.2));\n"
    "    vec3 c = pal[clamp(mat, 0, 13)] * (fract(objPos.x * 4.0) < 0.5 ? 1.0 : 0.7);\n"
    "#if DBG_MAT > 1\n"   // AGC_DBG_MAT=x|y|z: 2 cm bands of eight colours along that object axis (where each pixel is)
    "    vec3 band[8] = vec3[8](vec3(1,0,0), vec3(0,1,0), vec3(0,0,1), vec3(1,1,0), vec3(0,1,1), vec3(1,0,1), vec3(1,1,1), vec3(0.15));\n"
    "    float coord = DBG_MAT == 2 ? objPos.x : DBG_MAT == 3 ? objPos.y : objPos.z;\n"
    "    c = band[int(mod(floor(coord * 50.0 + 1000.0), 8.0))];\n"
    "#endif\n"
    "    finalColor = sceneOut(c * 0.6, 1.0); return; }\n"
    "#endif\n"
    "  if (mat == 7) { finalColor = sceneOut(base * 6.0 * exposure, 1.0); return; }\n"  // emissive
    "  float metal = 0.0, rough = 0.6, bumpK = 1.0;\n"
    "  vec3 on = normalize(objNormal), bumpO = vec3(0.0); vec4 sm = vec4(0.5);\n"
    "  if (mat == 1) { sm = triMap(texFoil, objPos / 2.20, on, bumpO); metal = 1.0; rough = 0.24; bumpK = 1.1; }\n"   // gold / amber kapton foil, crumpled facets
    "  else if (mat == 2) { sm = triMap(texPanel, objPos / 2.52, on, bumpO); metal = 1.0; rough = 0.35; }\n"     // aluminium / inconel
    "  else if (mat == 3) { sm = triMap(texPanel, objPos / 2.52, on, bumpO); rough = 0.42; bumpK = 1.4; base = max(base, vec3(0.075, 0.075, 0.08)); }\n"   // black-painted panels: satin, seams and rivets catch the light
    "  else if (mat == 4) { sm = triMap(texPanel, objPos / 2.52, on, bumpO); rough = 0.5; }\n"                   // white paint
    "  else if (mat == 5) { sm = triMap(texBlanket, objPos / 1.20, on, bumpO); metal = 1.0; rough = 0.40; bumpK = 1.5; }\n"   // silver mylar blanket
    "  else if (mat == 6) { rough = 0.15; }\n"                                                                   // window glass (dark)
    "  else if (mat == 8) { sm = triMap(texRock, objPos / 1.50, on, bumpO); rough = 0.92; }\n"                   // lunar rock
    "  else if (mat == 9) { rough = 0.95; }\n"                                                                   // plain matte (painted-on shadows)
    "  else if (mat == 10) { sm = triMap(texPanel, objPos / 2.52, on, bumpO); metal = 0.85; rough = 0.40; bumpK = 1.5; }\n"   // anodised micrometeoroid shield: metallic sheen, panels at slight angles
    "  else if (mat == 11) { metal = 1.0; rough = 0.10; }\n"                                                    // gold visor: a mirror
    "  else if (mat == 13) { sm = triMap(texBlanket, objPos / 1.20, on, bumpO); rough = 0.30; bumpK = 1.9; }\n"   // black H-film blanket: dark, glossy crinkles
    "  else if (mat == 12) { sm = triMap(texBlanket, objPos / 0.45, on, bumpO); sm.a = mix(0.5, sm.a, 0.35); rough = 0.85; bumpK = 0.35; }\n"   // beta-cloth suit
    "  base *= sm.a * 2.0;\n"
    "  rough = clamp(rough + (sm.b - 0.5), 0.08, 1.0);\n"
    "  N = normalize(N + mat3(matModel) * bumpO * bumpK);\n"
    "  // Specular anti-aliasing: where the bumped normal changes faster than the\n"
    "  // pixel grid can show, widen the highlight instead of letting it sparkle.\n"
    "  vec3 ndx = dFdx(N), ndy = dFdy(N);\n"
    "  float nvar = min(dot(ndx, ndx) + dot(ndy, ndy), 0.18);\n"
    "  // The same for the reflected sky/ground horizon: crinkles a few pixels wide\n"
    "  // would each snap to black sky or bright ground (salt-and-pepper speckle).\n"
    "  float envRough = min(1.0, sqrt(sqrt(min(rough * rough * rough * rough + nvar, 1.0))) + 2.5 * sqrt(dot(ndx, ndx) + dot(ndy, ndy)));\n"
    "  vec3 L = sunDir;\n"
    "  vec3 H = normalize(L + V);\n"
    "  float NL = max(dot(N, L), 0.0), NV = max(dot(N, V), 1e-3), NH = max(dot(N, H), 0.0), VH = max(dot(V, H), 0.0);\n"
    "  float a2 = min(rough * rough * rough * rough + nvar, 1.0);\n"
    "  rough = sqrt(sqrt(a2));\n"
    "  float dn = NH * NH * (a2 - 1.0) + 1.0; float D = a2 / (3.14159 * dn * dn);\n"
    "  float k = (rough + 1.0) * (rough + 1.0) / 8.0;\n"
    "  float G = NL / (NL * (1.0 - k) + k) * NV / (NV * (1.0 - k) + k);\n"
    "  vec3 F0 = mix(vec3(0.04), base, metal);\n"
    "  vec3 F = F0 + (1.0 - F0) * pow(1.0 - VH, 5.0);\n"
    "  vec3 spec = min(D * G * F / max(4.0 * NL * NV, 1e-3), vec3(24.0));\n"
    "  vec3 diff = (1.0 - F) * (1.0 - metal) * base / 3.14159;\n"
    "  float sh = selfShadow(fragPos, Ngeo) * sunGain;\n"
    "  // Sunlight on the vehicle is held a little under the true ratio to the\n"
    "  // regolith so bright skins keep their panel detail instead of clipping.\n"
    "  float sunK = (mat == 8 || mat == 9) ? 9.0 : 6.5;\n"
    "  vec3 col = (diff + spec) * NL * sh * sunK;\n"
    "  for (int i = 0; i < 4; i++) {\n"   // local lamps (point lights, camera-relative)
    "    vec3 lv = lampPos[i] - fragPos; float d2 = dot(lv, lv); vec3 l = lv * inversesqrt(d2);\n"
    "    float nl = max(dot(N, l), 0.0); vec3 h2 = normalize(l + V);\n"
    "    col += lampCol[i] / (d2 + 1.0) * nl * (diff + F0 * pow(max(dot(N, h2), 0.0), 40.0) * 0.5);\n"
    "  }\n"
    "  // Reflected / bounced light from the lunar surface.\n"
    "  vec3 R = reflect(-V, N);\n"
    "  vec3 Fe = F0 + (max(vec3(1.0 - rough), F0) - F0) * pow(1.0 - NV, 5.0);\n"
    "  col += envLight(R, envRough) * Fe * mix(0.6, 1.0, metal);\n"
    "  col += (1.0 - metal) * base * envLight(N, 1.0) * 0.5;\n"    "  // Fill: light bounced between the vehicle's own parts and up off the sunlit\n"
    "  // ground nearby. Faint, but in shadow the panels keep their seams and rivets\n"
    "  // instead of turning into flat black holes.\n"
    "  col += (base * (1.0 - metal * 0.6) * 0.075 + F0 * 0.04) * (0.6 + 0.4 * sm.a * 2.0);\n"
    "  finalColor = sceneOut(col * exposure, 1.0);\n"
    "}\n";

// ---------------------------------------------------------------------------
// Effects (additive sprites / plume / dust). Vertex colour carries intensity.

static const char *fx_vs = GLSL_HEADER LOGZ_VS
    "in vec3 vertexPosition; in vec2 vertexTexCoord; in vec4 vertexColor;\n"
    "uniform mat4 mvp;\n"
    "out vec2 uv; out vec4 col;\n"
    "void main(){ uv = vertexTexCoord; col = vertexColor; gl_Position = mvp * vec4(vertexPosition, 1.0); fragLogZ = 1.0 + gl_Position.w; }\n";

static const char *fx_fs = GLSL_HEADER LOGZ_FS NOISE_GLSL
    "in vec2 uv; in vec4 col; uniform float exposure; uniform float time; uniform int kind; uniform float gain;\n"
    "out vec4 finalColor;\n"
    "void main(){\n"
    "  writeDepth();\n"
    "  vec2 p = uv * 2.0 - 1.0; float r = length(p);\n"
    "  float a;\n"
    "  if (kind == 0) { a = exp(-r * r * 3.0) * smoothstep(1.0, 0.55, r); }\n"                                   // soft puff
    "  else if (kind == 1) { float n = fbm(vec2(uv.x * 6.0, uv.y * 2.0 - time * 9.0)); a = (1.0 - uv.y) * smoothstep(1.0, 0.2, abs(p.x)) * (0.5 + n); }\n"  // plume
    "  else if (kind == 2) { float n = fbm(vec2(atan(p.y, p.x) * 6.0, r * 3.0 - time * 2.5)); a = smoothstep(1.0, 0.55, r) * smoothstep(0.15, 0.5, r) * (0.3 + 1.2 * n); }\n"  // dust sheet
    "  else if (kind == 4) { float e = max(abs(p.x + p.y * 0.4), abs(p.y)) + 0.25 * fbm(uv * 5.0 + col.rg * 9.0);\n"   // crash debris shard (alpha blended)
    "         if (e > 0.85) discard; finalColor = sceneOut(col.rgb * exposure * gain * (0.75 + 0.25 * p.y), col.a); return; }\n"
    "  else { float ax = abs(p.x); a = smoothstep(1.0, 0.0, ax) * smoothstep(0.0, 0.25, uv.y) * smoothstep(1.0, 0.6, uv.y); \n"   // dust streak (alpha blended)
    "         finalColor = sceneOut(col.rgb * exposure * gain, a * col.a); return; }\n"
    "  finalColor = sceneOut(col.rgb * a * col.a * exposure * gain, 1.0);\n"
    "}\n";

// ---------------------------------------------------------------------------
// Footprints: multiplicative decals laid on the ground. Each quad carries the
// print's frame (normal = ground, tangent = the boot's forward); the fragment
// shader models the boot-sole hollow and its tread as a height field, shades
// it against the low Sun (with self-shadowing across the hollow) relative to
// the flat ground, and multiplies the terrain already drawn by that ratio.

static const char *decal_vs = GLSL_HEADER LOGZ_VS
    "in vec3 vertexPosition; in vec3 vertexNormal; in vec4 vertexTangent; in vec2 vertexTexCoord; in vec4 vertexColor;\n"
    "uniform mat4 mvp; uniform mat4 matModel;\n"
    "out vec3 fragPos; out vec3 gN; out vec3 gT; out vec2 uv; out vec4 info;\n"
    "void main(){\n"
    "  vec4 wp = matModel * vec4(vertexPosition, 1.0); fragPos = wp.xyz;\n"
    "  gN = vertexNormal; gT = vertexTangent.xyz; uv = vertexTexCoord; info = vertexColor;\n"
    "  gl_Position = mvp * vec4(vertexPosition, 1.0);\n"
    "  fragLogZ = 1.0 + gl_Position.w;\n"
    "}\n";

static const char *decal_fs = GLSL_HEADER LOGZ_FS
    "in vec3 fragPos; in vec3 gN; in vec3 gT; in vec2 uv; in vec4 info;\n"
    "uniform vec3 sunDir;\n"
    "uniform mat4 shadowVP; uniform sampler2D shadowMap; uniform vec3 shadowCenter; uniform float shadowOn;\n"
    CREW_SHADOW_GLSL
    "out vec4 finalColor;\n"
    "float lmShadow1(vec3 p){\n"
    "  if (shadowOn < 0.5) return 1.0;\n"
    "  vec4 sp = shadowVP * vec4(p - shadowCenter, 1.0);\n"
    "  vec3 c = sp.xyz / sp.w * 0.5 + 0.5;\n"
    "  if (c.x < 0.0 || c.x > 1.0 || c.y < 0.0 || c.y > 1.0 || c.z > 1.0) return 1.0;\n"
    "  return (c.z - 0.002 > texture(shadowMap, c.xy).r) ? 0.0 : 1.0;\n"
    "}\n"
    "// Signed distance to the sole outline (metres; q: across, along, from its centre).\n"
    "float sole(vec2 q){\n"
    "  q.x -= 0.012 * q.y * q.y / 0.03;\n"   // the overshoe's sole curves a little
    "  vec2 hs = vec2(q.y > 0.0 ? 0.068 : 0.058, 0.162); float r = 0.045;\n"
    "  vec2 d = abs(q) - hs + r; return length(max(d, 0.0)) + min(max(d.x, d.y), 0.0) - r;\n"
    "}\n"
    "// Height (1 = print depth): the hollow with its tread bars, soil pushed up round the edge.\n"
    "float hgt(vec2 q, float bars){\n"
    "  float sd = sole(q);\n"
    "  float inside = 1.0 - smoothstep(-0.010, 0.004, sd);\n"
    "  float ch = q.y * 34.0 + abs(q.x) * 14.0;\n"
    "  float tread = bars * smoothstep(0.30, 0.70, abs(fract(ch) - 0.5) * 2.0);\n"
    "  float rim = (1.0 - smoothstep(0.0, 0.035, abs(sd - 0.012))) * 0.22;\n"
    "  return -inside * (0.78 + 0.22 * tread) + rim;\n"
    "}\n"
    "void main(){\n"
    "  writeDepth();\n"
    "  float depth = info.r * 0.035, fade = info.a;\n"
    "  vec2 q = vec2(uv.x * 0.11, uv.y * 0.21);\n"
    "  float bars = 1.0 - smoothstep(0.0, 0.5, fwidth(q.y) * 34.0);\n"   // tread bars fade out before they shimmer
    "  float e = 0.003;\n"
    "  float h0 = hgt(q, bars), hx = hgt(q + vec2(e, 0.0), bars), hy = hgt(q + vec2(0.0, e), bars);\n"
    "  vec3 N = normalize(gN); vec3 T = normalize(gT - N * dot(gT, N)); vec3 R = cross(T, N);\n"
    "  vec3 n = normalize(N - R * ((hx - h0) / e * depth) - T * ((hy - h0) / e * depth));\n"
    "  float flat_ = max(dot(N, sunDir), 0.03);\n"
    "  float lit = max(dot(n, sunDir), 0.0);\n"
    "  // Across the hollow toward the Sun: does the far wall hide it?\n"
    "  vec2 sd = vec2(dot(sunDir, R), dot(sunDir, T)); float sl = length(sd); float sh = 1.0;\n"
    "  if (sl > 1e-3) {\n"
    "    vec2 st = sd / sl; float rise = dot(sunDir, N) / sl;\n"
    "    for (int i = 1; i <= 7; i++) { float dd = float(i) * 0.011; if (hgt(q + st * dd, bars) * depth > h0 * depth + rise * dd) { sh = 0.15; break; } }\n"
    "  }\n"
    "  float ratio = clamp(lit * sh / flat_, 0.0, 2.4);\n"
    "  float alb = mix(1.0, 0.86, clamp(-h0, 0.0, 1.0));\n"   // packed soil is a shade darker
    "  float sun = min(lmShadow1(fragPos), crewShadow(fragPos));\n"
    "  float k = mix(1.0, alb * mix(1.0, ratio, sun), fade);\n"
    "#if DIRECT_OUT == 2\n"
    "  finalColor = vec4(vec3(pow(k, 1.0 / 2.2)), 1.0);\n"
    "#else\n"
    "  finalColor = vec4(vec3(k), 1.0);\n"
    "#endif\n"
    "}\n";

// ---------------------------------------------------------------------------
// Sky: stars, Sun, Earth (fullscreen pass, reconstructs view rays).

static const char *sky_vs = GLSL_HEADER
    "in vec3 vertexPosition; in vec2 vertexTexCoord;\n"
    "out vec2 uv;\n"
    "void main(){ uv = vertexTexCoord; gl_Position = vec4(vertexPosition.xy, 0.0, 1.0); }\n";

static const char *sky_fs = GLSL_HEADER NOISE_GLSL
    "in vec2 uv; uniform vec3 camFwd; uniform vec3 camRight; uniform vec3 camUp; uniform vec2 tanHalf; uniform vec2 tanOff;\n"
    "uniform vec3 sunDir; uniform vec3 earthDir; uniform vec3 earthSun; uniform float exposure; uniform float starGain;\n"
    "uniform float earthAngRad;\n"
    "out vec4 finalColor;\n"
    "vec3 earthColor(vec3 n){\n"
    "  // n: Earth surface normal (Earth-fixed-ish frame). Procedural oceans, land, clouds.\n"
    "  float land = smoothstep(0.52, 0.56, fbm3(n * 2.3 + vec3(3.1)));\n"
    "  float lat = abs(n.y);\n"
    "  vec3 ocean = vec3(0.02, 0.06, 0.16);\n"
    "  vec3 ground = mix(vec3(0.20, 0.17, 0.10), vec3(0.10, 0.16, 0.06), fbm3(n * 6.0));\n"
    "  ground = mix(ground, vec3(0.9), smoothstep(0.78, 0.86, lat));\n"
    "  vec3 c = mix(ocean, ground, land);\n"
    "  float cl = smoothstep(0.45, 0.75, fbm3(n * 4.0 + vec3(0.0, 1.7, 5.2)) * 0.7 + fbm3(n * 13.0) * 0.4);\n"
    "  cl = max(cl, smoothstep(0.62, 0.70, fbm3(n * vec3(2.0, 8.0, 2.0))) * 0.8);\n"
    "  return mix(c, vec3(0.95), cl);\n"
    "}\n"
    "void main(){\n"
    "  vec2 s = uv * 2.0 - 1.0;\n"
    "  vec3 dir = normalize(camFwd + camRight * (s.x * tanHalf.x + tanOff.x) + camUp * (s.y * tanHalf.y + tanOff.y));\n"
    "  vec3 col = vec3(0.0);\n"
    "  // Stars: hashed points on a fine direction grid.\n"
    "  vec3 a = abs(dir); vec2 st; float face;\n"
    "  if (a.x > a.y && a.x > a.z) { st = dir.yz / a.x; face = dir.x > 0.0 ? 0.0 : 1.0; }\n"
    "  else if (a.y > a.z) { st = dir.xz / a.y; face = dir.y > 0.0 ? 2.0 : 3.0; }\n"
    "  else { st = dir.xy / a.z; face = dir.z > 0.0 ? 4.0 : 5.0; }\n"
    "  // Each star is drawn at least about a pixel wide (its light spread, not\n"
    "  // added): a point smaller than a pixel would blink on and off as the view\n"
    "  // moves - badly so in a headset, where the head never keeps still.\n"
    "  vec2 g = st * Q_STARGRID; vec2 cell = floor(g); vec2 f = fract(g);\n"
    "  float h = hash12(cell + face * 1000.0);\n"
    "  if (h > 0.985) {\n"
    "    vec2 c = vec2(hash12(cell + 7.0 + face), hash12(cell + 13.0 + face)) * 0.44 + 0.28;\n"
    "    float d = length(f - c);\n"
    "    float mag = pow(hash12(cell * 1.7 + face), 18.0) * 6.0 + 0.05;\n"
    "    float tint = hash12(cell + 99.0);\n"
    "    vec3 sc = mix(vec3(1.0,0.82,0.65), vec3(0.7,0.82,1.0), tint);\n"
    "    float px = min(max(fwidth(g.x), fwidth(g.y)), 0.25);\n"          // cells per pixel
    "    float sig0 = 0.0745 * Q_STARGRID / 420.0, sig = max(sig0, 0.6 * px);\n"
    "    col += sc * mag * sqrt(sig0 / sig) * Q_STARBOOST * exp(-d * d / (2.0 * sig * sig)) * starGain;\n"
    "  }\n"
    "  // Sun: disc + corona glare.\n"
    "  float cs = dot(dir, sunDir);\n"
    "  float ang = acos(clamp(cs, -1.0, 1.0));\n"
    "  col += vec3(1.0, 0.97, 0.92) * (smoothstep(0.0049, 0.0044, ang) * 4000.0);\n"
    "  col += vec3(1.0, 0.95, 0.85) * (0.05 / (ang * ang * 400.0 + 0.002)) * 0.02;\n"
    "  col += vec3(1.0, 0.95, 0.9) * (exp(-ang * 26.0) * 0.9 + exp(-ang * 7.0) * 0.07);\n"
    "  // Faint six-point diffraction star, as a lens would draw it.\n"
    "  vec3 sr = normalize(cross(sunDir, vec3(0.0, 1.0, 0.0) + vec3(1e-4))), su = cross(sr, sunDir);\n"
    "  float sa = atan(dot(dir, su), dot(dir, sr));\n"
    "  float spike = pow(abs(cos(sa * 3.0)), 60.0) + 0.5 * pow(abs(sin(sa * 3.0)), 90.0);\n"
    "  col += vec3(1.0, 0.96, 0.9) * spike * exp(-ang * 20.0) * 0.16 * step(0.0, cs);\n"
    "  // Earth: ray-sphere at its apparent angular radius.\n"
    "  float ce = dot(dir, earthDir);\n"
    "  float sinR = sin(earthAngRad);\n"
    "  float ang2 = acos(clamp(ce, -1.0, 1.0));\n"
    "  if (ang2 < earthAngRad * 1.08) {\n"
    "    vec3 right = normalize(cross(earthDir, vec3(0.0, 1.0, 0.0) + vec3(1e-4)));\n"
    "    vec3 upv = cross(right, earthDir);\n"
    "    vec2 q = vec2(dot(dir, right), dot(dir, upv)) / sinR;\n"
    "    float rq = length(q);\n"
    "    if (rq < 1.0) {\n"
    "      vec3 n = normalize(right * q.x + upv * q.y - earthDir * sqrt(1.0 - rq * rq));\n"
    "      float lit = dot(n, earthSun);\n"
    "      vec3 surf = earthColor(vec3(dot(n, right), dot(n, upv), dot(n, -earthDir)));\n"
    "      float day = smoothstep(-0.05, 0.1, lit);\n"
    "      vec3 c = surf * max(lit, 0.0) * 1.6 * day;\n"
    "      float rim = pow(rq, 6.0);\n"
    "      c += vec3(0.25, 0.45, 1.0) * rim * 0.9 * smoothstep(-0.2, 0.3, lit);\n"
    "      col = c * 4.0;\n"
    "    } else {\n"
    "      float glow = exp(-(rq - 1.0) * 40.0);\n"
    "      col += vec3(0.3, 0.5, 1.0) * glow * 0.8 * smoothstep(-0.3, 0.5, dot(normalize(right*q.x+upv*q.y), earthSun));\n"
    "    }\n"
    "  }\n"
    "  finalColor = sceneOut(col * exposure, 1.0);\n"
    "}\n";

// ---------------------------------------------------------------------------
// Post: bright-pass, blur, composite + tonemap.

static const char *post_vs = GLSL_HEADER
    "in vec3 vertexPosition; in vec2 vertexTexCoord; out vec2 uv;\n"
    "void main(){ uv = vertexTexCoord; gl_Position = vec4(vertexPosition.xy, 0.0, 1.0); }\n";

#ifdef HAVE_MSAA
// Multisample resolve into the HDR target. Each sample is weighted by
// 1 / (1 + brightness) so that, after tonemapping, an edge between a sunlit
// part (HDR values well above 1) and black space comes out half-way instead
// of nearly full bright: the edge stays smooth.
static const char *resolve_fs = GLSL_HEADER
    "uniform sampler2DMS msaaTex; uniform int samples;\n"
    "out vec4 o;\n"
    "void main(){\n"
    "  ivec2 p = ivec2(gl_FragCoord.xy); vec3 s = vec3(0.0); float ws = 0.0, a = 0.0;\n"
    "  for (int i = 0; i < samples; i++) {\n"
    "    vec4 c = texelFetch(msaaTex, p, i); float w = 1.0 / (1.0 + max(c.r, max(c.g, c.b)));\n"
    "    s += c.rgb * w; ws += w; a += c.a;\n"
    "  }\n"
    "  o = vec4(s / ws, a / float(samples));\n"
    "}\n";
#endif

static const char *down_fs = GLSL_HEADER
    "in vec2 uv; uniform sampler2D src; uniform vec2 texel; uniform float threshold;\n"
    "out vec4 o;\n"
    "void main(){\n"
    "  vec3 c = texture(src, uv + texel * vec2(-1,-1)).rgb + texture(src, uv + texel * vec2(1,-1)).rgb\n"
    "         + texture(src, uv + texel * vec2(-1,1)).rgb + texture(src, uv + texel * vec2(1,1)).rgb;\n"
    "  c = c * 0.25 + texture(src, uv).rgb * 0.0;\n"
    "  if (threshold > 0.0) { float l = max(max(c.r, c.g), c.b); c *= max(l - threshold, 0.0) / max(l, 1e-4); }\n"
    "  c = min(c, vec3(260.0));\n"
    "  o = vec4(c, 1.0);\n"
    "}\n";

static const char *up_fs = GLSL_HEADER
    "in vec2 uv; uniform sampler2D src; uniform vec2 texel;\n"
    "out vec4 o;\n"
    "void main(){\n"
    "  vec3 c = vec3(0.0);\n"
    "  c += texture(src, uv + texel * vec2(-1,-1)).rgb; c += texture(src, uv + texel * vec2(0,-1)).rgb * 2.0; c += texture(src, uv + texel * vec2(1,-1)).rgb;\n"
    "  c += texture(src, uv + texel * vec2(-1,0)).rgb * 2.0; c += texture(src, uv).rgb * 4.0; c += texture(src, uv + texel * vec2(1,0)).rgb * 2.0;\n"
    "  c += texture(src, uv + texel * vec2(-1,1)).rgb; c += texture(src, uv + texel * vec2(0,1)).rgb * 2.0; c += texture(src, uv + texel * vec2(1,1)).rgb;\n"
    "  o = vec4(c / 16.0, 1.0);\n"
    "}\n";

static const char *final_fs = GLSL_HEADER NOISE_GLSL
    "in vec2 uv; uniform sampler2D scene; uniform sampler2D bloom; uniform float bloomGain;\n"
    "uniform float film;\n"
    "out vec4 o;\n"
    "vec3 aces(vec3 x){ const float a=2.51, b=0.03, c=2.43, d=0.59, e=0.14; return clamp((x*(a*x+b))/(x*(c*x+d)+e), 0.0, 1.0); }\n"
    "void main(){\n"
    "  vec3 c = texture(scene, uv).rgb;\n"
    "  vec3 b = texture(bloom, uv).rgb;\n"
    "  c += b * bloomGain;\n"
    "  vec2 q = uv - 0.5;\n"
    "  float vig = 1.0 - dot(q, q) * (0.35 + film * 0.9);\n"
    "  c *= vig;\n"
    "  if (film > 0.0) {\n"
    "    // 16 mm Data Acquisition Camera look: halation, warm shift, grain, gate weave.\n"
    "    c += b * vec3(0.9, 0.35, 0.15) * 0.5 * film;\n"
    "    c = mix(c, c * vec3(1.05, 1.0, 0.88), film);\n"
    "  }\n"
    "  c = aces(c);\n"
    "  c = pow(c, vec3(1.0 / 2.2));\n"
    "  o = vec4(c, dot(c, vec3(0.299, 0.587, 0.114)));\n"
    "}\n";

// Edge smoothing (FXAA 3.11 quality, luma in alpha), then film grain and a
// half-bit dither so the dark sky never bands.
static const char *fxaa_fs = GLSL_HEADER NOISE_GLSL
    "in vec2 uv; uniform sampler2D src; uniform vec2 texel; uniform float film; uniform float time; uniform vec2 resolution;\n"
    "out vec4 o;\n"
    "float L(vec2 p){ return texture(src, p).a; }\n"
    "void main(){\n"
    "  vec4 cm = texture(src, uv);\n"
    "  float lM = cm.a, lN = L(uv + vec2(0.0, texel.y)), lS = L(uv - vec2(0.0, texel.y)), lE = L(uv + vec2(texel.x, 0.0)), lW = L(uv - vec2(texel.x, 0.0));\n"
    "  float lmin = min(lM, min(min(lN, lS), min(lE, lW))), lmax = max(lM, max(max(lN, lS), max(lE, lW)));\n"
    "  float range = lmax - lmin;\n"
    "  vec3 c = cm.rgb;\n"
    "  if (range > max(0.0312, lmax * 0.125)) {\n"
    "    float lNW = L(uv + texel * vec2(-1.0, 1.0)), lNE = L(uv + texel), lSW = L(uv - texel), lSE = L(uv + texel * vec2(1.0, -1.0));\n"
    "    float eH = abs(lNW + lSW - 2.0 * lW) + 2.0 * abs(lN + lS - 2.0 * lM) + abs(lNE + lSE - 2.0 * lE);\n"
    "    float eV = abs(lNW + lNE - 2.0 * lN) + 2.0 * abs(lW + lE - 2.0 * lM) + abs(lSW + lSE - 2.0 * lS);\n"
    "    bool horz = eV >= eH;\n"   // edge runs horizontally: step across it vertically
    "    float l1 = horz ? lS : lW, l2 = horz ? lN : lE;\n"
    "    float g1 = l1 - lM, g2 = l2 - lM;\n"
    "    bool steep1 = abs(g1) >= abs(g2);\n"
    "    float gscale = 0.25 * max(abs(g1), abs(g2));\n"
    "    float stepLen = horz ? texel.y : texel.x;\n"
    "    float lavg;\n"
    "    if (steep1) { stepLen = -stepLen; lavg = 0.5 * (l1 + lM); } else { lavg = 0.5 * (l2 + lM); }\n"
    "    vec2 p = uv;\n"
    "    if (horz) p.y += stepLen * 0.5; else p.x += stepLen * 0.5;\n"
    "    vec2 off = horz ? vec2(texel.x, 0.0) : vec2(0.0, texel.y);\n"
    "    vec2 p1 = p - off, p2 = p + off;\n"
    "    float e1 = L(p1) - lavg, e2 = L(p2) - lavg;\n"
    "    bool d1 = abs(e1) >= gscale, d2 = abs(e2) >= gscale;\n"
    "    for (int i = 0; i < 10 && !(d1 && d2); i++) {\n"
    "      float q = i < 4 ? 1.0 : (i < 7 ? 2.0 : 4.0);\n"
    "      if (!d1) { p1 -= off * q; e1 = L(p1) - lavg; d1 = abs(e1) >= gscale; }\n"
    "      if (!d2) { p2 += off * q; e2 = L(p2) - lavg; d2 = abs(e2) >= gscale; }\n"
    "    }\n"
    "    float dist1 = horz ? uv.x - p1.x : uv.y - p1.y, dist2 = horz ? p2.x - uv.x : p2.y - uv.y;\n"
    "    bool near1 = dist1 < dist2;\n"
    "    float dmin = min(dist1, dist2), span = dist1 + dist2;\n"
    "    bool good = ((near1 ? e1 : e2) < 0.0) != (lM < lavg);\n"
    "    float edgeOff = good ? 0.5 - dmin / span : 0.0;\n"
    "    float lall = (2.0 * (lN + lS + lE + lW) + lNW + lNE + lSW + lSE) / 12.0;\n"
    "    float sub = clamp(abs(lall - lM) / range, 0.0, 1.0); sub = sub * sub * (3.0 - 2.0 * sub); sub = sub * sub * 0.75;\n"
    "    float fo = max(edgeOff, sub);\n"
    "    vec2 fp = uv;\n"
    "    if (horz) fp.y += fo * stepLen; else fp.x += fo * stepLen;\n"
    "    c = texture(src, fp).rgb;\n"
    "  }\n"
    "  if (film > 0.0) {\n"
    "    float gr = hash12(uv * resolution + fract(time * 23.7) * 1000.0) - 0.5;\n"
    "    c += gr * 0.08 * film;\n"
    "    float lum = dot(c, vec3(0.3, 0.59, 0.11));\n"
    "    c = mix(c, vec3(lum) * vec3(1.02, 0.98, 0.9), 0.35 * film);\n"
    "  }\n"
    "  c += (hash12(uv * resolution + 17.0) - 0.5) / 255.0;\n"
    "  o = vec4(c, 1.0);\n"
    "}\n";

static const char *depth_fs = GLSL_HEADER
    "out vec4 o; void main(){ o = vec4(1.0); }\n";
static const char *depth_vs = GLSL_HEADER
    "in vec3 vertexPosition; uniform mat4 mvp; void main(){ gl_Position = mvp * vec4(vertexPosition, 1.0); }\n";

// ---------------------------------------------------------------------------

static Shader sh_terrain, sh_lm, sh_fx, sh_sky, sh_down, sh_up, sh_final, sh_fxaa, sh_depth, sh_decal;
static Texture2D tex_foil, tex_panel, tex_blanket, tex_rock, tex_soil;
static float exposure = 1.0f;

typedef struct { unsigned int fbo; Texture2D tex; unsigned int depth; int w, h; } Target;
static Target hdr;
static Target ldr;   // tonemapped image, luma in alpha (input of the edge-smoothing pass)
#define NBLOOM 6
static Target bloom[NBLOOM];
static float film_amount;
static int out_w = 0, out_h = 0;   // final output size (screen or screenshot target)
static unsigned int out_fbo = 0;
void render_set_output(int w, int h) { out_w = w; out_h = h; }
void render_set_output_fbo(unsigned int fbo) { out_fbo = fbo; }
static void bind_output(void)
{
    if (out_fbo) rlEnableFramebuffer(out_fbo);
    else rlDisableFramebuffer();
}
static float bloom_gain = 0.035f;

// Shadow map.
#define SHADOW_SIZE (render_quality()->shadow)
static unsigned int shadow_fbo, shadow_tex;
static Matrix shadow_vp;
static V3 shadow_center;
static int shadow_valid;
// The crew's shadow map.
#define SHADOW2_SIZE (render_quality()->shadow / 2)
static unsigned int shadow2_fbo, shadow2_tex;
static Matrix shadow2_vp;
static V3 shadow2_center;
static int shadow2_on;

// Output mode of the scene shaders: 0 = HDR target + bloom/tonemap/FXAA passes (PC);
// 1 = straight into a linear target that the display encodes (sRGB swapchain, Quest);
// 2 = straight into a plain 8-bit target (gamma applied here).
static int direct_out;
static unsigned int direct_fbo;
static int direct_x, direct_y;
static int cur_w, cur_h;
void render_set_direct(int mode) { direct_out = mode; }
int render_direct(void) { return direct_out; }
void render_set_direct_target(unsigned int fbo, int x, int y) { direct_fbo = fbo; direct_x = x; direct_y = y; }
void render_set_shadow_center(V3 c);

// Quality tier. "high" is the PC look; "low" is cut down for the Quest (fewer
// crater octaves, two soil lookups, one surface-map lookup per pixel, 4-tap
// shadows, shorter noise, depth from the vertex shader so the tiler can throw
// hidden pixels away early, less terrain detail, a smaller shadow map).
// AGC_QUALITY=low|high picks a tier; AGC_Q_* override single settings (tests).
static RenderQuality quality;
static int quality_ready;

static int env_int(const char *name, int def)
{
    const char *e = getenv(name);
    return e && *e ? atoi(e) : def;
}

static float env_float(const char *name, float def)
{
    const char *e = getenv(name);
    return e && *e ? (float)atof(e) : def;
}

const RenderQuality *render_quality(void)
{
    if (quality_ready) return &quality;
#ifdef AGC_VR
    int low = 1;
#else
    int low = 0;
#endif
    const char *e = getenv("AGC_QUALITY");
    if (e && *e) low = e[0] == 'l' || e[0] == 'L';
    RenderQuality q = { 0 };
    q.low = low;
    q.oct0 = 0;
    q.oct1 = 6;
    q.soil = low ? 2 : 3;
    q.tri1 = low;
    q.pcf = low ? 4 : 9;
    q.fbm = low ? 2 : 5;
    q.vz = 0;
    q.lod = low ? 0.75f : 1.0f;
    q.shadow = low ? 1024 : 2048;
    q.star_grid = low ? 160.0f : 420.0f;
    q.star_boost = low ? 2.2f : 1.0f;
    // Craters narrower than this many pixels fade out (wider in the headset,
    // where the picture is coarser and never still).
    q.crater_px0 = low ? 3.0f : 1.5f;
    q.crater_px1 = low ? 7.0f : 4.0f;
    q.msaa = low ? 0 : 4;
    q.oct0 = env_int("AGC_Q_OCT0", q.oct0);
    q.oct1 = env_int("AGC_Q_OCT1", q.oct1);
    q.soil = env_int("AGC_Q_SOIL", q.soil);
    q.tri1 = env_int("AGC_Q_TRI1", q.tri1);
    q.pcf = env_int("AGC_Q_PCF", q.pcf);
    q.fbm = env_int("AGC_Q_FBM", q.fbm);
    q.vz = env_int("AGC_Q_VZ", q.vz);
    q.lod = env_float("AGC_Q_LOD", q.lod);
    q.shadow = env_int("AGC_Q_SHADOW", q.shadow);
    q.star_grid = env_float("AGC_Q_STARGRID", q.star_grid);
    q.star_boost = env_float("AGC_Q_STARBOOST", q.star_boost);
    q.crater_px0 = env_float("AGC_Q_CRATERPX0", q.crater_px0);
    q.crater_px1 = env_float("AGC_Q_CRATERPX1", q.crater_px1);
    q.msaa = env_int("AGC_Q_MSAA", q.msaa);
    if (q.oct0 < 0) q.oct0 = 0;
    if (q.oct1 > 6) q.oct1 = 6;
    if (q.pcf != 9) q.pcf = 4;
    if (q.msaa < 2) q.msaa = 0;
    if (q.msaa > 8) q.msaa = 8;
    quality = q;
    quality_ready = 1;
    TraceLog(LOG_INFO, "render: quality %s (craters %d-%d, soil %d, trimap %s, pcf %d, fbm %d, vertex depth %d, lod %.2f, shadow %d, msaa %d)",
             low ? "low" : "high", q.oct0, q.oct1, q.soil, q.tri1 ? "1" : "3", q.pcf, q.fbm, q.vz, q.lod, q.shadow, q.msaa);
    return &quality;
}

Shader render_load_shader(const char *vs, const char *fs)
{
    const RenderQuality *q = render_quality();
    char defs[512];
    snprintf(defs, sizeof defs,
             "#define Q_LOW %d\n#define Q_OCT0 %d\n#define Q_OCT1 %d\n#define Q_SOIL %d\n#define Q_TRI1 %d\n#define Q_PCF %d\n"
             "#define Q_FBM %d\n#define Q_VZ %d\n#define Q_STARGRID %.1f\n#define Q_STARBOOST %.3f\n"
             "#define Q_CRATER_PX0 %.2f\n#define Q_CRATER_PX1 %.2f\n",
             q->low, q->oct0, q->oct1, q->soil, q->tri1, q->pcf, q->fbm, q->vz, q->star_grid, q->star_boost, q->crater_px0, q->crater_px1);
#ifdef AGC_GLES
    static const char *ver = "#version 300 es\nprecision highp float;\nprecision highp int;\nprecision highp sampler2D;\n";
#else
    static const char *ver = "#version 330\n";
#endif
    static const char *out_fn =
        "vec3 agcAces(vec3 x){ return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0); }\n"
        "vec4 sceneOut(vec3 c, float a){\n"
        "#if DIRECT_OUT == 1\n"
        "  return vec4(agcAces(c), a);\n"
        "#elif DIRECT_OUT == 2\n"
        "  return vec4(pow(agcAces(c), vec3(1.0 / 2.2)), a);\n"
        "#else\n"
        "  return vec4(c, a);\n"
        "#endif\n"
        "}\n";
    // Depth from the vertex shader (Q_VZ): the log-depth value the fragment
    // shaders would write, set on gl_Position instead.
    const char *logz = "fragLogZ = 1.0 + gl_Position.w;";
    const char *logz_vz = "fragLogZ = 1.0 + gl_Position.w; gl_Position.z = (log2(max(1e-6, fragLogZ)) * logFar - 1.0) * gl_Position.w;";
    char *vs2 = NULL;
    if (q->vz && strstr(vs, logz)) {
        size_t n = strlen(vs) + 4 * (strlen(logz_vz) - strlen(logz)) + 1;
        vs2 = malloc(n);
        vs2[0] = 0;
        const char *p = vs, *m;
        while ((m = strstr(p, logz))) {
            strncat(vs2, p, (size_t)(m - p));
            strcat(vs2, logz_vz);
            p = m + strlen(logz);
        }
        strcat(vs2, p);
        vs = vs2;
    }
    size_t nv = strlen(vs) + 1024, nf = strlen(fs) + strlen(out_fn) + 1024;
    char *v = malloc(nv), *f = malloc(nf);
    snprintf(v, nv, "%s#define DIRECT_OUT %d\n%s%s", ver, direct_out, defs, vs);
    snprintf(f, nf, "%s#define DIRECT_OUT %d\n%s%s%s%s", ver, direct_out, defs, getenv("AGC_DBG_MAT") ? (getenv("AGC_DBG_MAT")[0] == 'x' ? "#define DBG_MAT 2\n" : getenv("AGC_DBG_MAT")[0] == 'y' ? "#define DBG_MAT 3\n" : getenv("AGC_DBG_MAT")[0] == 'z' ? "#define DBG_MAT 4\n" : "#define DBG_MAT 1\n") : "", out_fn, fs);
    free(vs2);
    Shader sh = LoadShaderFromMemory(v, f);
    const char *dump = getenv("AGC_SHADER_DUMP");   // debugging: write every shader out for an offline validator
    if (dump) {
        static int n;
        char path[600];
        for (int k = 0; k < 2; k++) {
            snprintf(path, sizeof path, "%s/s%02d.%s", dump, n, k ? "frag" : "vert");
            FILE *fp = fopen(path, "w");
            if (fp) { fputs(k ? f : v, fp); fclose(fp); }
        }
        n++;
    }
    free(v);
    free(f);
    return sh;
}

static Target make_target(int w, int h, int with_depth)
{
    Target t = { 0 };
    t.w = w;
    t.h = h;
    t.fbo = rlLoadFramebuffer();
    rlEnableFramebuffer(t.fbo);
    t.tex.id = rlLoadTexture(NULL, w, h, RL_PIXELFORMAT_UNCOMPRESSED_R16G16B16A16, 1);
    t.tex.width = w;
    t.tex.height = h;
    t.tex.mipmaps = 1;
    t.tex.format = PIXELFORMAT_UNCOMPRESSED_R16G16B16A16;
    SetTextureFilter(t.tex, TEXTURE_FILTER_BILINEAR);
    SetTextureWrap(t.tex, TEXTURE_WRAP_CLAMP);
    rlFramebufferAttach(t.fbo, t.tex.id, RL_ATTACHMENT_COLOR_CHANNEL0, RL_ATTACHMENT_TEXTURE2D, 0);
    if (with_depth) {
        t.depth = rlLoadTextureDepth(w, h, true);
        rlFramebufferAttach(t.fbo, t.depth, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_RENDERBUFFER, 0);
    }
    if (!rlFramebufferComplete(t.fbo))
        TraceLog(LOG_WARNING, "render: framebuffer %dx%d incomplete", w, h);
    rlDisableFramebuffer();
    return t;
}

static void free_target(Target *t)
{
    if (t->fbo) rlUnloadFramebuffer(t->fbo);
    if (t->tex.id) rlUnloadTexture(t->tex.id);
    memset(t, 0, sizeof *t);
}

#ifdef HAVE_MSAA
// The multisampled scene target (PC high tier): colour as a multisample texture
// for the resolve shader, depth in the same format as the HDR target's.
static unsigned int ms_fbo, ms_color, ms_depth;
static int ms_n;   // samples in the current target; 0 = drawing straight into hdr
static Shader sh_resolve;
static void fullscreen(void);

static void free_msaa(void)
{
    if (ms_fbo) glDeleteFramebuffers(1, &ms_fbo);
    if (ms_color) glDeleteTextures(1, &ms_color);
    if (ms_depth) glDeleteRenderbuffers(1, &ms_depth);
    ms_fbo = ms_color = ms_depth = 0;
    ms_n = 0;
}

static void make_msaa(int w, int h, int n)
{
    free_msaa();
    if (n < 2 || !sh_resolve.id) return;
    GLint maxs = 0;
    glGetIntegerv(GL_MAX_SAMPLES, &maxs);
    if (n > maxs) n = maxs;
    if (n < 2) return;
    GLint depth_fmt = GL_DEPTH_COMPONENT24;
    glBindRenderbuffer(GL_RENDERBUFFER, hdr.depth);
    glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_INTERNAL_FORMAT, &depth_fmt);
    glGenFramebuffers(1, &ms_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, ms_fbo);
    glGenTextures(1, &ms_color);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, ms_color);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, n, GL_RGBA16F, w, h, GL_TRUE);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, ms_color, 0);
    glGenRenderbuffers(1, &ms_depth);
    glBindRenderbuffer(GL_RENDERBUFFER, ms_depth);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, n, (GLenum)depth_fmt, w, h);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, ms_depth);
    int ok = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (!ok) {
        TraceLog(LOG_WARNING, "render: %dx MSAA target %dx%d incomplete, drawing without it", n, w, h);
        free_msaa();
        return;
    }
    ms_n = n;
}

// End of the scene: average the samples into hdr.tex (hdr bound, blending off).
static void resolve_msaa(void)
{
    rlEnableFramebuffer(hdr.fbo);
    rlViewport(0, 0, hdr.w, hdr.h);
    rlDisableColorBlend();
    rlDisableDepthTest();
    BeginShaderMode(sh_resolve);
    int unit = 9;
    SetShaderValue(sh_resolve, GetShaderLocation(sh_resolve, "msaaTex"), &unit, SHADER_UNIFORM_INT);
    SetShaderValue(sh_resolve, GetShaderLocation(sh_resolve, "samples"), &ms_n, SHADER_UNIFORM_INT);
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, ms_color);
    glActiveTexture(GL_TEXTURE0);
    fullscreen();
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);
    glActiveTexture(GL_TEXTURE0);
    EndShaderMode();
    rlEnableColorBlend();
}
#endif

static void ensure_targets(int w, int h)
{
    if (w == cur_w && h == cur_h) return;
    free_target(&hdr);
    free_target(&ldr);
    ldr = make_target(w, h, 0);
    for (int i = 0; i < NBLOOM; i++) free_target(&bloom[i]);
    hdr = make_target(w, h, 1);
#ifdef HAVE_MSAA
    make_msaa(w, h, render_quality()->msaa);
#endif
    int bw = w / 2, bh = h / 2;
    for (int i = 0; i < NBLOOM; i++) {
        bloom[i] = make_target(bw > 1 ? bw : 1, bh > 1 ? bh : 1, 0);
        bw /= 2;
        bh /= 2;
    }
    cur_w = w;
    cur_h = h;
}

static void set_logfar(Shader s)
{
    float lf = (float)(2.0 / log2(FAR_PLANE + 1.0));
    SetShaderValue(s, GetShaderLocation(s, "logFar"), &lf, SHADER_UNIFORM_FLOAT);
}

void render_init(void)
{
    sh_terrain = render_load_shader(terrain_vs, terrain_fs);
    sh_lm = render_load_shader(lm_vs, lm_fs);
    sh_fx = render_load_shader(fx_vs, fx_fs);
    sh_sky = render_load_shader(sky_vs, sky_fs);
    sh_down = render_load_shader(post_vs, down_fs);
    sh_up = render_load_shader(post_vs, up_fs);
    sh_final = render_load_shader(post_vs, final_fs);
    sh_fxaa = render_load_shader(post_vs, fxaa_fs);
#ifdef HAVE_MSAA
    if (!direct_out && render_quality()->msaa) sh_resolve = render_load_shader(post_vs, resolve_fs);
#endif
    sh_depth = render_load_shader(depth_vs, depth_fs);
    sh_decal = render_load_shader(decal_vs, decal_fs);
    sh_lm.locs[SHADER_LOC_MATRIX_NORMAL] = GetShaderLocation(sh_lm, "matNormal");
    // Surface maps ride in the material map slots 0..3 (the shadow map uses unit 5).
    sh_lm.locs[SHADER_LOC_MAP_ALBEDO] = GetShaderLocation(sh_lm, "texFoil");
    sh_lm.locs[SHADER_LOC_MAP_METALNESS] = GetShaderLocation(sh_lm, "texPanel");
    sh_lm.locs[SHADER_LOC_MAP_NORMAL] = GetShaderLocation(sh_lm, "texBlanket");
    sh_lm.locs[SHADER_LOC_MAP_ROUGHNESS] = GetShaderLocation(sh_lm, "texRock");
    tex_foil = assets_texture("lm_foil");
    tex_panel = assets_texture("lm_panel");
    tex_blanket = assets_texture("lm_blanket");
    tex_rock = assets_texture("rock");
    tex_soil = assets_texture("regolith");
    SetTextureFilter(tex_soil, TEXTURE_FILTER_ANISOTROPIC_16X);   // seen at grazing angles
    set_logfar(sh_terrain);
    set_logfar(sh_lm);
    set_logfar(sh_fx);
    set_logfar(sh_decal);

    shadow_fbo = rlLoadFramebuffer();
    rlEnableFramebuffer(shadow_fbo);
    shadow_tex = rlLoadTextureDepth(SHADOW_SIZE, SHADOW_SIZE, false);
    rlFramebufferAttach(shadow_fbo, shadow_tex, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);
    if (!rlFramebufferComplete(shadow_fbo)) TraceLog(LOG_WARNING, "render: shadow FBO incomplete");
    shadow2_fbo = rlLoadFramebuffer();
    rlEnableFramebuffer(shadow2_fbo);
    shadow2_tex = rlLoadTextureDepth(SHADOW2_SIZE, SHADOW2_SIZE, false);
    rlFramebufferAttach(shadow2_fbo, shadow2_tex, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);
    if (!rlFramebufferComplete(shadow2_fbo)) TraceLog(LOG_WARNING, "render: crew shadow FBO incomplete");
    rlDisableFramebuffer();
}

void render_shutdown(void)
{
    free_target(&hdr);
    free_target(&ldr);
    for (int i = 0; i < NBLOOM; i++) free_target(&bloom[i]);
#ifdef HAVE_MSAA
    free_msaa();
    if (sh_resolve.id) UnloadShader(sh_resolve);
#endif
    UnloadShader(sh_fxaa);
    UnloadShader(sh_terrain); UnloadShader(sh_lm); UnloadShader(sh_fx); UnloadShader(sh_sky);
    UnloadShader(sh_down); UnloadShader(sh_up); UnloadShader(sh_final); UnloadShader(sh_depth); UnloadShader(sh_decal);
    rlUnloadFramebuffer(shadow_fbo);
    rlUnloadTexture(shadow_tex);
    rlUnloadFramebuffer(shadow2_fbo);
    rlUnloadTexture(shadow2_tex);
}

Shader *render_terrain_shader(void) { return &sh_terrain; }
Shader *render_lm_shader(void) { return &sh_lm; }
void render_lm_material(Material *m)
{
    m->shader = sh_lm;
    m->maps[MATERIAL_MAP_ALBEDO].texture = tex_foil;
    m->maps[MATERIAL_MAP_METALNESS].texture = tex_panel;
    m->maps[MATERIAL_MAP_NORMAL].texture = tex_blanket;
    m->maps[MATERIAL_MAP_ROUGHNESS].texture = tex_rock;
}
Shader *render_fx_shader(void) { return &sh_fx; }
Shader *render_depth_shader(void) { return &sh_depth; }
Shader *render_decal_shader(void) { return &sh_decal; }
void render_set_exposure(float e) { exposure = e; }
void render_set_film(float f) { film_amount = f; }

static void set3(Shader s, const char *name, V3 v)
{
    float f[3] = { (float)v.x, (float)v.y, (float)v.z };
    SetShaderValue(s, GetShaderLocation(s, name), f, SHADER_UNIFORM_VEC3);
}
static void set1(Shader s, const char *name, float v)
{
    SetShaderValue(s, GetShaderLocation(s, name), &v, SHADER_UNIFORM_FLOAT);
}

// Fullscreen quad in clip space.
static void fullscreen(void)
{
    rlBegin(RL_QUADS);
    rlTexCoord2f(0, 0); rlVertex3f(-1, -1, 0);
    rlTexCoord2f(1, 0); rlVertex3f(1, -1, 0);
    rlTexCoord2f(1, 1); rlVertex3f(1, 1, 0);
    rlTexCoord2f(0, 1); rlVertex3f(-1, 1, 0);
    rlEnd();
    rlDrawRenderBatchActive();
}

// --- Shadow pass ----------------------------------------------------------------

void render_shadow_begin(V3 center_site_rel, V3 sun, float radius)
{
    // Orthographic sun view around `center` (camera-relative coordinates).
    shadow_center = center_site_rel;
    Vector3 eye = { (float)(sun.x * radius * 2), (float)(sun.y * radius * 2), (float)(sun.z * radius * 2) };
    Vector3 upv = fabs(sun.y) > 0.95 ? (Vector3){ 1, 0, 0 } : (Vector3){ 0, 1, 0 };
    Matrix view = MatrixLookAt(eye, (Vector3){ 0, 0, 0 }, upv);
    Matrix proj = MatrixOrtho(-radius, radius, -radius, radius, 0.01, radius * 4);
    shadow_vp = MatrixMultiply(view, proj);
    rlDrawRenderBatchActive();
    rlEnableFramebuffer(shadow_fbo);
    rlViewport(0, 0, SHADOW_SIZE, SHADOW_SIZE);
    rlClearScreenBuffers();
    rlEnableDepthTest();
    rlSetMatrixProjection(proj);
    rlSetMatrixModelview(view);
    rlDisableColorBlend();
}

Matrix render_shadow_vp(void) { return shadow_vp; }

void render_shadow_end(void)
{
    rlDrawRenderBatchActive();
    bind_output();
    rlEnableColorBlend();
    shadow_valid = 1;
}

void render_shadow2_begin(V3 center, V3 sun, float radius, float length)
{
    // As render_shadow_begin, with the depth range stretched to take in a shadow
    // `length` metres long (a crewman's, with the Sun 11 degrees up: ~10 m).
    shadow2_center = center;
    float back = radius + 3.0f;
    Vector3 eye = { (float)(sun.x * back), (float)(sun.y * back), (float)(sun.z * back) };
    Vector3 upv = fabs(sun.y) > 0.95 ? (Vector3){ 1, 0, 0 } : (Vector3){ 0, 1, 0 };
    Matrix view = MatrixLookAt(eye, (Vector3){ 0, 0, 0 }, upv);
    Matrix proj = MatrixOrtho(-radius, radius, -radius, radius, 0.01, back + radius + length);
    shadow2_vp = MatrixMultiply(view, proj);
    rlDrawRenderBatchActive();
    rlEnableFramebuffer(shadow2_fbo);
    rlViewport(0, 0, SHADOW2_SIZE, SHADOW2_SIZE);
    rlClearScreenBuffers();
    rlEnableDepthTest();
    rlSetMatrixProjection(proj);
    rlSetMatrixModelview(view);
    rlDisableColorBlend();
}

void render_shadow2_end(void)
{
    rlDrawRenderBatchActive();
    bind_output();
    rlEnableColorBlend();
    shadow2_on = 1;
}

void render_shadow2_off(void) { shadow2_on = 0; }
void render_set_shadow2_center(V3 c) { shadow2_center = c; }

static void bind_shadow2(Shader s)
{
    SetShaderValueMatrix(s, GetShaderLocation(s, "shadow2VP"), shadow2_vp);
    set3(s, "shadow2Center", shadow2_center);
    set1(s, "shadow2On", shadow2_on ? 1.0f : 0.0f);
    int sloc = GetShaderLocation(s, "shadowMap2");
    rlEnableShader(s.id);
    rlActiveTextureSlot(7);
    rlEnableTexture(shadow2_tex);
    int slot = 7;
    rlSetUniform(sloc, &slot, SHADER_UNIFORM_INT, 1);
    rlActiveTextureSlot(0);
}

static void bind_shadow(Shader s)
{
    int loc = GetShaderLocation(s, "shadowVP");
    SetShaderValueMatrix(s, loc, shadow_vp);
    set3(s, "shadowCenter", shadow_center);
    set1(s, "shadowOn", shadow_valid ? 1.0f : 0.0f);
    int sloc = GetShaderLocation(s, "shadowMap");
    rlEnableShader(s.id);
    rlActiveTextureSlot(5);
    rlEnableTexture(shadow_tex);
    int slot = 5;
    rlSetUniform(sloc, &slot, SHADER_UNIFORM_INT, 1);
    rlActiveTextureSlot(0);
    bind_shadow2(s);
}

// --- Scene ---------------------------------------------------------------------

static ViewCam cur_cam;
static Camera3D rl_cam;

void render_set_shadow_center(V3 c) { shadow_center = c; }

// The view frustum of the scene being drawn (camera-relative, through the
// eye): left, right, bottom, top planes, unit inward normals.
static V3 frust[4];
static void set_frustum(const ViewCam *cam, int w, int h)
{
    V3 f = cam->fwd, r = v3_norm(v3_cross(cam->fwd, cam->upv)), u = v3_cross(r, f);
    double tl, tr, td, tu;
    if (cam->asym) { tl = cam->tan_l; tr = cam->tan_r; td = cam->tan_d; tu = cam->tan_u; }
    else {
        tu = tan(cam->fov_y / 2);
        td = -tu;
        tr = tu * (h > 0 ? (double)w / h : 1.0);
        tl = -tr;
    }
    frust[0] = v3_norm(v3_sub(r, v3_scale(f, tl)));
    frust[1] = v3_norm(v3_sub(v3_scale(f, tr), r));
    frust[2] = v3_norm(v3_sub(u, v3_scale(f, td)));
    frust[3] = v3_norm(v3_sub(v3_scale(f, tu), u));
}

int render_sphere_visible(V3 c, double radius)
{
    for (int k = 0; k < 4; k++)
        if (v3_dot(c, frust[k]) < -radius) return 0;
    return 1;
}

void render_begin_scene(const ViewCam *cam, int w, int h)
{
    cur_cam = *cam;
    set_frustum(cam, w, h);
    if (direct_out) {
        rlDrawRenderBatchActive();
        cur_w = w;
        cur_h = h;
        if (direct_fbo) rlEnableFramebuffer(direct_fbo);
        else rlDisableFramebuffer();
        rlViewport(direct_x, direct_y, w, h);
    } else {
        ensure_targets(w, h);
#ifdef HAVE_MSAA
        rlEnableFramebuffer(ms_n ? ms_fbo : hdr.fbo);
#else
        rlEnableFramebuffer(hdr.fbo);
#endif
        rlViewport(0, 0, w, h);
    }
    rlClearColor(0, 0, 0, 255);
    rlClearScreenBuffers();

    rl_cam.position = (Vector3){ 0, 0, 0 };
    rl_cam.target = (Vector3){ (float)cam->fwd.x, (float)cam->fwd.y, (float)cam->fwd.z };
    rl_cam.up = (Vector3){ (float)cam->upv.x, (float)cam->upv.y, (float)cam->upv.z };
    rl_cam.fovy = (float)(cam->fov_y * 180.0 / PI_D);
    rl_cam.projection = CAMERA_PERSPECTIVE;
}

static void scene_3d_begin(int w, int h, double near_plane);
void render_scene_3d_begin(int w, int h) { scene_3d_begin(w, h, NEAR_PLANE); }
void render_scene_3d_begin_near(int w, int h, double near_plane) { scene_3d_begin(w, h, near_plane); }

static void scene_3d_begin(int w, int h, double NEAR)
{
    rlSetClipPlanes(NEAR, FAR_PLANE);
    // BeginMode3D uses the current framebuffer size for the aspect ratio.
    rlDrawRenderBatchActive();
    rlMatrixMode(RL_PROJECTION);
    rlPushMatrix();
    rlLoadIdentity();
    double aspect = (double)w / h;
    double top = NEAR * tan(rl_cam.fovy * 0.5 * DEG2RAD);
    if (cur_cam.asym) rlFrustum(NEAR * cur_cam.tan_l, NEAR * cur_cam.tan_r, NEAR * cur_cam.tan_d, NEAR * cur_cam.tan_u, NEAR, FAR_PLANE);
    else rlFrustum(-top * aspect, top * aspect, -top, top, NEAR, FAR_PLANE);
    rlMatrixMode(RL_MODELVIEW);
    rlLoadIdentity();
    Matrix view = MatrixLookAt(rl_cam.position, rl_cam.target, rl_cam.up);
    rlMultMatrixf(MatrixToFloat(view));
    rlEnableDepthTest();
}

void render_scene_3d_end(void)
{
    rlDrawRenderBatchActive();
    rlMatrixMode(RL_PROJECTION);
    rlPopMatrix();
    rlMatrixMode(RL_MODELVIEW);
    rlLoadIdentity();
    rlDisableDepthTest();
}

void render_set_scene_uniforms(V3 sun, V3 cam_mod, V3 up_dir, float ground_albedo, V3 dust_center, float dust)
{
    set3(sh_terrain, "sunDir", sun);
    set3(sh_terrain, "camMod", cam_mod);
    set1(sh_terrain, "exposure", exposure);
    set3(sh_terrain, "dustCenter", dust_center);
    {
        const char *e = getenv("AGC_DBG");
        int dbg = e ? atoi(e) : 0;
        SetShaderValue(sh_terrain, GetShaderLocation(sh_terrain, "dbg"), &dbg, SHADER_UNIFORM_INT);
    }
    set1(sh_terrain, "dustAmount", dust);
    bind_shadow(sh_terrain);
    {   // The soil map rides on texture unit 6 (the shadow map is on 5).
        int slot = 6;
        rlActiveTextureSlot(6);
        rlEnableTexture(tex_soil.id);
        rlSetUniform(GetShaderLocation(sh_terrain, "soilMap"), &slot, SHADER_UNIFORM_INT, 1);
        rlActiveTextureSlot(0);
    }
    set3(sh_lm, "sunDir", sun);
    set1(sh_lm, "exposure", exposure);
    set1(sh_lm, "groundAlbedo", ground_albedo);
    set3(sh_lm, "upDir", up_dir);
    bind_shadow(sh_lm);
    set3(sh_decal, "sunDir", sun);
    bind_shadow(sh_decal);
    render_set_lm_extras(0.0f, 1.0f, NULL, NULL, 0);
    set1(sh_fx, "exposure", exposure);
    set1(sh_fx, "gain", 1.0f);
    set1(sh_fx, "time", (float)GetTime());
}

void render_set_lm_extras(float near_cut, float sun_gain, const V3 *lamp_pos, const V3 *lamp_col, int n)
{
    set1(sh_lm, "nearCut", near_cut);
    set1(sh_lm, "sunGain", sun_gain);
    float p[12] = { 0 }, c[12] = { 0 };
    for (int i = 0; i < n && i < 4; i++) {
        p[3 * i] = (float)lamp_pos[i].x; p[3 * i + 1] = (float)lamp_pos[i].y; p[3 * i + 2] = (float)lamp_pos[i].z;
        c[3 * i] = (float)lamp_col[i].x; c[3 * i + 1] = (float)lamp_col[i].y; c[3 * i + 2] = (float)lamp_col[i].z;
    }
    SetShaderValueV(sh_lm, GetShaderLocation(sh_lm, "lampPos"), p, SHADER_UNIFORM_VEC3, 4);
    SetShaderValueV(sh_lm, GetShaderLocation(sh_lm, "lampCol"), c, SHADER_UNIFORM_VEC3, 4);
}

float render_exposure(void) { return exposure; }
float render_log_far(void) { return (float)(2.0 / log2(FAR_PLANE + 1.0)); }

void render_sky(const ViewCam *cam, V3 sun, V3 earth, V3 earth_sun)
{
    V3 right = v3_norm(v3_cross(cam->fwd, cam->upv));
    V3 upv = v3_cross(right, cam->fwd);
    double th = tan(cam->fov_y / 2);
    float tanhalf[2] = { (float)(th * cur_w / cur_h), (float)th }, tanoff[2] = { 0, 0 };
    if (cam->asym) {
        tanhalf[0] = (float)((cam->tan_r - cam->tan_l) / 2); tanhalf[1] = (float)((cam->tan_u - cam->tan_d) / 2);
        tanoff[0] = (float)((cam->tan_r + cam->tan_l) / 2); tanoff[1] = (float)((cam->tan_u + cam->tan_d) / 2);
    }
    BeginShaderMode(sh_sky);
    set3(sh_sky, "camFwd", cam->fwd);
    set3(sh_sky, "camRight", right);
    set3(sh_sky, "camUp", upv);
    SetShaderValue(sh_sky, GetShaderLocation(sh_sky, "tanHalf"), tanhalf, SHADER_UNIFORM_VEC2);
    SetShaderValue(sh_sky, GetShaderLocation(sh_sky, "tanOff"), tanoff, SHADER_UNIFORM_VEC2);
    set3(sh_sky, "sunDir", sun);
    set3(sh_sky, "earthDir", earth);
    set3(sh_sky, "earthSun", earth_sun);
    set1(sh_sky, "exposure", exposure);
    // Stars are washed out when looking near the sunlit ground or the Sun.
    float sg = 0.35f * (float)fmax(0.0, 1.0 - fmax(0.0, v3_dot(cam->fwd, sun)) * 1.5);
    set1(sh_sky, "starGain", sg);
    set1(sh_sky, "earthAngRad", (float)(0.95 * PI_D / 180.0));
    rlDisableDepthTest();
    rlDisableDepthMask();
    fullscreen();
    rlEnableDepthMask();
    EndShaderMode();
}

void render_end_scene(void)
{
    rlDrawRenderBatchActive();
    if (direct_out) {
        // The scene went straight to its target, already tonemapped: nothing to composite.
        rlDisableDepthTest();
        return;
    }
#ifdef HAVE_MSAA
    if (ms_n) resolve_msaa();
#endif
    rlDisableFramebuffer();

    // Bloom: bright-pass downsample chain, then upsample-accumulate.
    rlDisableDepthTest();
    Texture2D src = hdr.tex;
    for (int i = 0; i < NBLOOM; i++) {
        rlEnableFramebuffer(bloom[i].fbo);
        rlViewport(0, 0, bloom[i].w, bloom[i].h);
        BeginShaderMode(sh_down);
        float texel[2] = { 1.0f / src.width, 1.0f / src.height };
        float thr = i == 0 ? 1.0f : 0.0f;
        SetShaderValue(sh_down, GetShaderLocation(sh_down, "texel"), texel, SHADER_UNIFORM_VEC2);
        SetShaderValue(sh_down, GetShaderLocation(sh_down, "threshold"), &thr, SHADER_UNIFORM_FLOAT);
        SetShaderValueTexture(sh_down, GetShaderLocation(sh_down, "src"), src);
        fullscreen();
        EndShaderMode();
        rlDisableFramebuffer();
        src = bloom[i].tex;
    }
    rlEnableColorBlend();
    for (int i = NBLOOM - 2; i >= 0; i--) {
        rlEnableFramebuffer(bloom[i].fbo);
        rlViewport(0, 0, bloom[i].w, bloom[i].h);
        rlSetBlendMode(BLEND_ADDITIVE);
        BeginShaderMode(sh_up);
        float texel[2] = { 1.0f / bloom[i + 1].w, 1.0f / bloom[i + 1].h };
        SetShaderValue(sh_up, GetShaderLocation(sh_up, "texel"), texel, SHADER_UNIFORM_VEC2);
        SetShaderValueTexture(sh_up, GetShaderLocation(sh_up, "src"), bloom[i + 1].tex);
        fullscreen();
        EndShaderMode();
        rlSetBlendMode(BLEND_ALPHA);
        rlDisableFramebuffer();
    }

    // Composite + tonemap into the LDR target.
    rlEnableFramebuffer(ldr.fbo);
    rlViewport(0, 0, cur_w, cur_h);
    rlDisableColorBlend();   // alpha carries luma here, it must not blend
    BeginShaderMode(sh_final);
    SetShaderValueTexture(sh_final, GetShaderLocation(sh_final, "scene"), hdr.tex);
    SetShaderValueTexture(sh_final, GetShaderLocation(sh_final, "bloom"), bloom[0].tex);
    set1(sh_final, "bloomGain", bloom_gain);
    set1(sh_final, "film", film_amount);
    fullscreen();
    EndShaderMode();
    rlDisableFramebuffer();
    rlEnableColorBlend();

    // Edge smoothing into the scene area of the output (bottom-left origin).
    bind_output();
    int ow = out_w ? out_w : GetRenderWidth(), oh = out_h ? out_h : GetRenderHeight();
    rlViewport(0, oh - cur_h, cur_w, cur_h);
    BeginShaderMode(sh_fxaa);
    SetShaderValueTexture(sh_fxaa, GetShaderLocation(sh_fxaa, "src"), ldr.tex);
    float texel_out[2] = { 1.0f / cur_w, 1.0f / cur_h };
    SetShaderValue(sh_fxaa, GetShaderLocation(sh_fxaa, "texel"), texel_out, SHADER_UNIFORM_VEC2);
    set1(sh_fxaa, "film", film_amount);
    set1(sh_fxaa, "time", (float)GetTime());
    float res[2] = { (float)cur_w, (float)cur_h };
    SetShaderValue(sh_fxaa, GetShaderLocation(sh_fxaa, "resolution"), res, SHADER_UNIFORM_VEC2);
    fullscreen();
    EndShaderMode();
    // Restore the 2D state raylib expects for the HUD.
    rlViewport(0, 0, ow, oh);
    rlMatrixMode(RL_PROJECTION);
    rlLoadIdentity();
    rlOrtho(0, ow, oh, 0, 0, 1);
    rlMatrixMode(RL_MODELVIEW);
    rlLoadIdentity();
    rlDisableDepthTest();
}
