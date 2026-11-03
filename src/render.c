// Scene rendering: HDR target, logarithmic depth, sky, bloom, tonemapping,
// sun shadow map. All shaders are embedded so the game stays a single exe.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "render.h"

#define FAR_PLANE 2.0e8
#define NEAR_PLANE 0.25

// ---------------------------------------------------------------------------
// Shared GLSL.

#define GLSL_HEADER "#version 330\n"

#define LOGZ_VS \
    "uniform float logFar;\n" \
    "out float fragLogZ;\n"
#define LOGZ_FS \
    "uniform float logFar;\n" \
    "in float fragLogZ;\n" \
    "void writeDepth(){ gl_FragDepth = log2(fragLogZ) * logFar * 0.5; }\n"

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
    "float fbm(vec2 p){ float s=0.0, a=0.5; for(int i=0;i<5;i++){ s+=a*vnoise(p); p*=2.03; a*=0.5;} return s; }\n" \
    "float fbm3(vec3 p){ float s=0.0, a=0.5; for(int i=0;i<5;i++){ s+=a*vnoise3(p); p*=2.03; a*=0.5;} return s; }\n"

// ---------------------------------------------------------------------------
// Terrain.

static const char *terrain_vs = GLSL_HEADER LOGZ_VS
    "in vec3 vertexPosition; in vec3 vertexNormal; in vec4 vertexColor;\n"
    "uniform mat4 mvp; uniform mat4 matModel;\n"
    "out vec3 fragPos; out vec3 fragNormal; out vec4 fragColor;\n"
    "void main(){\n"
    "  vec4 wp = matModel * vec4(vertexPosition, 1.0);\n"
    "  fragPos = wp.xyz; fragNormal = vertexNormal; fragColor = vertexColor;\n"
    "  gl_Position = mvp * vec4(vertexPosition, 1.0);\n"
    "  fragLogZ = 1.0 + gl_Position.w;\n"
    "}\n";

static const char *terrain_fs = GLSL_HEADER LOGZ_FS NOISE_GLSL
    "in vec3 fragPos; in vec3 fragNormal; in vec4 fragColor;\n"
    "uniform vec3 sunDir; uniform vec3 camMod; uniform float exposure;\n"
    "uniform mat4 shadowVP; uniform sampler2D shadowMap; uniform vec3 shadowCenter; uniform float shadowOn;\n"
    "uniform vec3 dustCenter; uniform float dustAmount; uniform int dbg;\n"
    "out vec4 finalColor;\n"
    "float lmShadow(vec3 p){\n"
    "  if (shadowOn < 0.5) return 1.0;\n"
    "  vec4 sp = shadowVP * vec4(p - shadowCenter, 1.0);\n"
    "  vec3 c = sp.xyz / sp.w * 0.5 + 0.5;\n"
    "  if (c.x < 0.0 || c.x > 1.0 || c.y < 0.0 || c.y > 1.0 || c.z > 1.0) return 1.0;\n"
    "  float s = 0.0; vec2 texel = 1.0 / vec2(textureSize(shadowMap, 0));\n"
    "  for (int x=-1;x<=1;x++) for(int y=-1;y<=1;y++){ float d = texture(shadowMap, c.xy + vec2(x,y)*texel*1.5).r; s += (c.z - 0.002 > d) ? 0.0 : 1.0; }\n"
    "  return s / 9.0;\n"
    "}\n"
    "void main(){\n"
    "  if (dbg != 5) writeDepth();\n"
    "  float dist = length(fragPos);\n"
    "  vec3 N = normalize(fragNormal);\n"
    "  vec3 V = -fragPos / max(dist, 1e-3);\n"
    "  vec2 dp = camMod.xz + fragPos.xz;\n"
    "  float near = 1.0 - smoothstep(30.0, 900.0, dist);\n"
    "  float mid = 1.0 - smoothstep(400.0, 9000.0, dist);\n"
    "  // Regolith micro relief: perturb the normal close up.\n"
    "  if (near > 0.0) {\n"
    "    float e = 0.08;\n"
    "    float h0 = fbm(dp * 1.7), hx = fbm((dp + vec2(e,0)) * 1.7), hz = fbm((dp + vec2(0,e)) * 1.7);\n"
    "    float k = 0.035 * near;\n"
    "    N = normalize(N + vec3(-(hx-h0)/e, 0.0, -(hz-h0)/e) * k);\n"
    "  }\n"
    "  if (mid > 0.0) {\n"
    "    float e = 1.5;\n"
    "    float h0 = fbm(dp * 0.07), hx = fbm((dp + vec2(e,0)) * 0.07), hz = fbm((dp + vec2(0,e)) * 0.07);\n"
    "    N = normalize(N + vec3(-(hx-h0)/e, 0.0, -(hz-h0)/e) * 0.45 * mid);\n"
    "  }\n"
    "  float albedo = fragColor.r * 1.0;\n"   // stored as albedo*1000/255
    "  albedo *= 0.255;\n"
    "  float fresh = fragColor.b;\n"
    "  // Fine albedo mottling, dark and bright regolith.\n"
    "  float mott = fbm(dp * 0.013) * 0.6 + fbm(dp * 0.31) * 0.4;\n"
    "  albedo *= mix(1.0, 0.75 + 0.5 * mott, 0.6);\n"
    "  albedo *= 1.0 + 0.6 * near * (fbm(dp * 6.0) - 0.5);\n"
    "  // Lommel-Seeliger with an opposition surge (the lunar 'flat' look).\n"
    "  float mu0 = max(dot(N, sunDir), 0.0);\n"
    "  float mu = max(dot(N, V), 0.02);\n"
    "  float g = acos(clamp(dot(sunDir, V), -1.0, 1.0));\n"
    "  float surge = 1.0 + 0.9 * exp(-g / 0.06) + 0.35 * exp(-g / 0.5);\n"
    "  float ls = mu0 / (mu0 + mu) * 2.0;\n"
    "  float sh = smoothstep(0.0, 1.0, fragColor.g) * lmShadow(fragPos);\n"
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
    "  finalColor = vec4(col * exposure, 1.0);\n"
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

static const char *lm_fs = GLSL_HEADER LOGZ_FS NOISE_GLSL
    "in vec3 fragPos; in vec3 fragNormal; in vec4 fragColor; in vec3 objPos; in vec3 objNormal;\n"
    "uniform vec3 sunDir; uniform float exposure; uniform float groundAlbedo;\n"
    "// Surface detail (no texture maps): planar coordinates on the face's main axis.\n"
    "vec2 faceUV(vec3 p, vec3 n){ vec3 a = abs(n); return a.x > a.y && a.x > a.z ? p.yz : (a.y > a.z ? p.xz : p.xy); }\n"
    "// Distance (m) to the nearest panel seam, and the panel's cell id.\n"
    "float seamDist(vec2 uv, float sp, out vec2 cell){ vec2 q = uv / sp; cell = floor(q); vec2 f = abs(fract(q) - 0.5); return (0.5 - max(f.x, f.y)) * sp; }\n"
    "// Bump a world normal by a scalar height field (metres) using screen derivatives.\n"
    "vec3 bump(vec3 N, float h){ vec3 dpx = dFdx(fragPos), dpy = dFdy(fragPos); float hx = dFdx(h), hy = dFdy(h);\n"
    "  vec3 r1 = cross(dpy, N), r2 = cross(N, dpx); float det = dot(dpx, r1);\n"
    "  if (abs(det) < 1e-12) return N; vec3 g = sign(det) * (hx * r1 + hy * r2); return normalize(abs(det) * N - g); }\n"
    "uniform mat4 shadowVP; uniform sampler2D shadowMap; uniform vec3 shadowCenter; uniform float shadowOn;\n"
    "uniform vec3 upDir; uniform float nearCut; uniform float sunGain;\n"
    "uniform vec3 lampPos[4]; uniform vec3 lampCol[4];\n"
    "out vec4 finalColor;\n"
    "float selfShadow(vec3 p, vec3 n){\n"
    "  if (shadowOn < 0.5) return 1.0;\n"
    "  vec4 sp = shadowVP * vec4(p - shadowCenter + n * 0.04, 1.0);\n"
    "  vec3 c = sp.xyz / sp.w * 0.5 + 0.5;\n"
    "  if (c.x < 0.0 || c.x > 1.0 || c.y < 0.0 || c.y > 1.0) return 1.0;\n"
    "  float s = 0.0; vec2 texel = 1.0 / vec2(textureSize(shadowMap, 0));\n"
    "  for (int x=-1;x<=1;x++) for(int y=-1;y<=1;y++){ float d = texture(shadowMap, c.xy + vec2(x,y)*texel).r; s += (c.z - 0.0015 > d) ? 0.0 : 1.0; }\n"
    "  return s / 9.0;\n"
    "}\n"
    "vec3 envLight(vec3 dir, float rough){\n"
    "  // Black sky above, sunlit grey regolith below, bright toward the sun-lit ground.\n"
    "  float d = dot(dir, upDir);\n"
    "  vec3 ground = vec3(0.55,0.53,0.50) * groundAlbedo * 2.6;\n"
    "  float t = smoothstep(0.08 + rough * 0.3, -0.15 - rough * 0.3, d);\n"
    "  return ground * t + vec3(0.012);\n"
    "}\n"
    "void main(){\n"
    "  if (nearCut > 0.0 && length(fragPos) < nearCut) discard;\n"
    "  writeDepth();\n"
    "  int mat = int(fragColor.a * 255.0 + 0.5);\n"
    "  vec3 base = fragColor.rgb;\n"
    "  vec3 N = normalize(fragNormal);\n"
    "  vec3 V = normalize(-fragPos);\n"
    "  if (!gl_FrontFacing) N = -N;\n"
    "  float metal = 0.0, rough = 0.6;\n"
    "  if (mat == 1) {\n"   // gold / amber kapton foil, crinkled
    "    float w = fbm3(objPos * 7.0);\n"
    "    vec3 g = vec3(fbm3(objPos*9.0+1.3), fbm3(objPos*9.0+7.1), fbm3(objPos*9.0+3.7)) - 0.5;\n"
    "    N = normalize(N + g * 0.32 + (w - 0.5) * 0.12);\n"
    "    metal = 1.0; rough = 0.30 + 0.18 * w;\n"
    "    base *= 0.85 + 0.3 * w;\n"
    "  } else if (mat == 2) { metal = 1.0; rough = 0.35; }\n"          // aluminium / inconel
    "  else if (mat == 3) { metal = 0.0; rough = 0.8; }\n"             // black paint
    "  else if (mat == 4) { metal = 0.0; rough = 0.5; }\n"             // white paint
    "  else if (mat == 5) {\n"                                        // silver/black mylar blanket
    "    float w = fbm3(objPos * 5.0);\n"
    "    vec3 g = vec3(fbm3(objPos*8.0+2.3), fbm3(objPos*8.0+5.1), fbm3(objPos*8.0+9.7)) - 0.5;\n"
    "    N = normalize(N + g * 0.22);\n"
    "    metal = 1.0; rough = 0.38 + 0.2 * w;\n"
    "  } else if (mat == 6) { metal = 0.0; rough = 0.15; }\n"          // window glass (dark)
    "  else if (mat == 7) { finalColor = vec4(base * 6.0 * exposure, 1.0); return; }\n"  // emissive
    "  // Panel seams, rivets, blanket quilting and panel-to-panel tone, faded out\n"
    "  // with distance so they never shimmer.\n"
    "  float px = length(fwidth(objPos));\n"
    "  float detail = 1.0 - smoothstep(0.03, 0.09, px);\n"
    "  if (detail > 0.0 && mat >= 1 && mat <= 5) {\n"
    "    vec3 on = normalize(objNormal);\n"
    "    vec2 uv = faceUV(objPos, on) + on.zx * 0.37;\n"
    "    vec2 cell; float h = 0.0;\n"
    "    if (mat == 2 || mat == 4) {\n"   // riveted sheet-metal panels
    "      float d = seamDist(uv, 0.42, cell);\n"
    "      h -= 0.003 * smoothstep(0.012, 0.0, d);\n"
    "      vec2 r = uv / 0.06; vec2 rf = fract(r) - 0.5;\n"
    "      float band = smoothstep(0.012, 0.004, abs(d - 0.028));\n"
    "      float rivet = smoothstep(0.30, 0.12, length(rf)) * (1.0 - smoothstep(0.004, 0.012, px));\n"
    "      h += 0.002 * band * rivet;\n"
    "      base *= mix(1.0, 0.8 + 0.4 * hash12(cell + 7.0), detail);\n"
    "      base *= 1.0 - 0.45 * smoothstep(max(0.008, px * 0.9), 0.0, d) * detail;\n"
    "    } else if (mat == 5 || mat == 3) {\n"   // quilted thermal blanket / black-coated blanket
    "      float d = seamDist(uv, 0.30, cell);\n"
    "      vec2 f = fract(uv / 0.30) - 0.5;\n"
    "      h += 0.018 * (1.0 - 4.0 * f.x * f.x) * (1.0 - 4.0 * f.y * f.y);\n"
    "      h -= 0.004 * smoothstep(0.01, 0.0, d);\n"
    "      base *= mix(1.0, 0.8 + 0.4 * hash12(cell + 3.0), detail);\n"
    "      base *= 1.0 - 0.4 * smoothstep(max(0.006, px * 0.9), 0.0, d) * detail;\n"
    "    } else if (mat == 1) {\n"   // kapton sheets joined with tape
    "      float d = seamDist(uv, 0.55, cell);\n"
    "      float tape = smoothstep(0.03, 0.022, d);\n"
    "      base *= 1.0 - 0.4 * smoothstep(max(0.006, px * 0.9), 0.0, d) * detail;\n"
    "      h += 0.002 * tape;\n"
    "      base *= mix(1.0, mix(0.82 + 0.3 * hash12(cell + 11.0), 1.1, tape), detail);\n"
    "      base = mix(base, base * vec3(1.05, 0.92, 0.7), 0.35 * hash12(cell + 5.0) * detail);\n"
    "    }\n"
    "    N = normalize(mix(N, bump(N, h), detail));\n"
    "  }\n"
    "  vec3 L = sunDir;\n"
    "  vec3 H = normalize(L + V);\n"
    "  float NL = max(dot(N, L), 0.0), NV = max(dot(N, V), 1e-3), NH = max(dot(N, H), 0.0), VH = max(dot(V, H), 0.0);\n"
    "  float a = rough * rough, a2 = a * a;\n"
    "  float dn = NH * NH * (a2 - 1.0) + 1.0; float D = a2 / (3.14159 * dn * dn);\n"
    "  float k = (rough + 1.0) * (rough + 1.0) / 8.0;\n"
    "  float G = NL / (NL * (1.0 - k) + k) * NV / (NV * (1.0 - k) + k);\n"
    "  vec3 F0 = mix(vec3(0.04), base, metal);\n"
    "  vec3 F = F0 + (1.0 - F0) * pow(1.0 - VH, 5.0);\n"
    "  vec3 spec = D * G * F / max(4.0 * NL * NV, 1e-3);\n"
    "  vec3 diff = (1.0 - F) * (1.0 - metal) * base / 3.14159;\n"
    "  float sh = selfShadow(fragPos, N) * sunGain;\n"
    "  vec3 col = (diff + spec) * NL * sh * 9.0;\n"
    "  for (int i = 0; i < 4; i++) {\n"   // local lamps (point lights, camera-relative)
    "    vec3 lv = lampPos[i] - fragPos; float d2 = dot(lv, lv); vec3 l = lv * inversesqrt(d2);\n"
    "    float nl = max(dot(N, l), 0.0); vec3 h2 = normalize(l + V);\n"
    "    col += lampCol[i] / (d2 + 1.0) * nl * (diff + F0 * pow(max(dot(N, h2), 0.0), 40.0) * 0.5);\n"
    "  }\n"
    "  // Reflected / bounced light from the lunar surface.\n"
    "  vec3 R = reflect(-V, N);\n"
    "  vec3 Fe = F0 + (max(vec3(1.0 - rough), F0) - F0) * pow(1.0 - NV, 5.0);\n"
    "  col += envLight(R, rough) * Fe * mix(0.6, 1.0, metal);\n"
    "  col += (1.0 - metal) * base * envLight(N, 1.0) * 0.5;\n"
    "  finalColor = vec4(col * exposure, 1.0);\n"
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
    "         if (e > 0.85) discard; finalColor = vec4(col.rgb * exposure * gain * (0.75 + 0.25 * p.y), col.a); return; }\n"
    "  else { float ax = abs(p.x); a = smoothstep(1.0, 0.0, ax) * smoothstep(0.0, 0.25, uv.y) * smoothstep(1.0, 0.6, uv.y); \n"   // dust streak (alpha blended)
    "         finalColor = vec4(col.rgb * exposure * gain, a * col.a); return; }\n"
    "  finalColor = vec4(col.rgb * a * col.a * exposure * gain, 1.0);\n"
    "}\n";

// ---------------------------------------------------------------------------
// Sky: stars, Sun, Earth (fullscreen pass, reconstructs view rays).

static const char *sky_vs = GLSL_HEADER
    "in vec3 vertexPosition; in vec2 vertexTexCoord;\n"
    "out vec2 uv;\n"
    "void main(){ uv = vertexTexCoord; gl_Position = vec4(vertexPosition.xy, 0.0, 1.0); }\n";

static const char *sky_fs = GLSL_HEADER NOISE_GLSL
    "in vec2 uv; uniform vec3 camFwd; uniform vec3 camRight; uniform vec3 camUp; uniform vec2 tanHalf;\n"
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
    "  vec3 dir = normalize(camFwd + camRight * s.x * tanHalf.x + camUp * s.y * tanHalf.y);\n"
    "  vec3 col = vec3(0.0);\n"
    "  // Stars: hashed points on a fine direction grid.\n"
    "  vec3 a = abs(dir); vec2 st; float face;\n"
    "  if (a.x > a.y && a.x > a.z) { st = dir.yz / a.x; face = dir.x > 0.0 ? 0.0 : 1.0; }\n"
    "  else if (a.y > a.z) { st = dir.xz / a.y; face = dir.y > 0.0 ? 2.0 : 3.0; }\n"
    "  else { st = dir.xy / a.z; face = dir.z > 0.0 ? 4.0 : 5.0; }\n"
    "  vec2 g = st * 420.0; vec2 cell = floor(g); vec2 f = fract(g);\n"
    "  float h = hash12(cell + face * 1000.0);\n"
    "  if (h > 0.985) {\n"
    "    vec2 c = vec2(hash12(cell + 7.0 + face), hash12(cell + 13.0 + face)) * 0.6 + 0.2;\n"
    "    float d = length(f - c);\n"
    "    float mag = pow(hash12(cell * 1.7 + face), 18.0) * 6.0 + 0.05;\n"
    "    float tint = hash12(cell + 99.0);\n"
    "    vec3 sc = mix(vec3(1.0,0.82,0.65), vec3(0.7,0.82,1.0), tint);\n"
    "    col += sc * mag * exp(-d * d * 90.0) * starGain;\n"
    "  }\n"
    "  // Sun: disc + corona glare.\n"
    "  float cs = dot(dir, sunDir);\n"
    "  float ang = acos(clamp(cs, -1.0, 1.0));\n"
    "  col += vec3(1.0, 0.97, 0.92) * (smoothstep(0.0049, 0.0044, ang) * 4000.0);\n"
    "  col += vec3(1.0, 0.95, 0.85) * (0.05 / (ang * ang * 400.0 + 0.002)) * 0.02;\n"
    "  col += vec3(1.0, 0.95, 0.9) * exp(-ang * 9.0) * 0.6;\n"
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
    "  finalColor = vec4(col * exposure, 1.0);\n"
    "}\n";

// ---------------------------------------------------------------------------
// Post: bright-pass, blur, composite + tonemap.

static const char *post_vs = GLSL_HEADER
    "in vec3 vertexPosition; in vec2 vertexTexCoord; out vec2 uv;\n"
    "void main(){ uv = vertexTexCoord; gl_Position = vec4(vertexPosition.xy, 0.0, 1.0); }\n";

static const char *down_fs = GLSL_HEADER
    "in vec2 uv; uniform sampler2D src; uniform vec2 texel; uniform float threshold;\n"
    "out vec4 o;\n"
    "void main(){\n"
    "  vec3 c = texture(src, uv + texel * vec2(-1,-1)).rgb + texture(src, uv + texel * vec2(1,-1)).rgb\n"
    "         + texture(src, uv + texel * vec2(-1,1)).rgb + texture(src, uv + texel * vec2(1,1)).rgb;\n"
    "  c = c * 0.25 + texture(src, uv).rgb * 0.0;\n"
    "  if (threshold > 0.0) { float l = max(max(c.r, c.g), c.b); c *= max(l - threshold, 0.0) / max(l, 1e-4); }\n"
    "  c = min(c, vec3(4000.0));\n"
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
    "uniform float film; uniform float time; uniform vec2 resolution;\n"
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
    "  if (film > 0.0) {\n"
    "    float gr = hash12(uv * resolution + fract(time * 23.7) * 1000.0) - 0.5;\n"
    "    c += gr * 0.08 * film;\n"
    "    float lum = dot(c, vec3(0.3, 0.59, 0.11));\n"
    "    c = mix(c, vec3(lum) * vec3(1.02, 0.98, 0.9), 0.35 * film);\n"
    "  }\n"
    "  o = vec4(c, 1.0);\n"
    "}\n";

static const char *depth_fs = GLSL_HEADER
    "out vec4 o; void main(){ o = vec4(1.0); }\n";
static const char *depth_vs = GLSL_HEADER
    "in vec3 vertexPosition; uniform mat4 mvp; void main(){ gl_Position = mvp * vec4(vertexPosition, 1.0); }\n";

// ---------------------------------------------------------------------------

static Shader sh_terrain, sh_lm, sh_fx, sh_sky, sh_down, sh_up, sh_final, sh_depth;
static float exposure = 1.0f;

typedef struct { unsigned int fbo; Texture2D tex; unsigned int depth; int w, h; } Target;
static Target hdr;
#define NBLOOM 6
static Target bloom[NBLOOM];
static int cur_w, cur_h;
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
#define SHADOW_SIZE 2048
static unsigned int shadow_fbo, shadow_tex;
static Matrix shadow_vp;
static V3 shadow_center;
static int shadow_valid;

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

static void ensure_targets(int w, int h)
{
    if (w == cur_w && h == cur_h) return;
    free_target(&hdr);
    for (int i = 0; i < NBLOOM; i++) free_target(&bloom[i]);
    hdr = make_target(w, h, 1);
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
    sh_terrain = LoadShaderFromMemory(terrain_vs, terrain_fs);
    sh_lm = LoadShaderFromMemory(lm_vs, lm_fs);
    sh_fx = LoadShaderFromMemory(fx_vs, fx_fs);
    sh_sky = LoadShaderFromMemory(sky_vs, sky_fs);
    sh_down = LoadShaderFromMemory(post_vs, down_fs);
    sh_up = LoadShaderFromMemory(post_vs, up_fs);
    sh_final = LoadShaderFromMemory(post_vs, final_fs);
    sh_depth = LoadShaderFromMemory(depth_vs, depth_fs);
    sh_lm.locs[SHADER_LOC_MATRIX_NORMAL] = GetShaderLocation(sh_lm, "matNormal");
    set_logfar(sh_terrain);
    set_logfar(sh_lm);
    set_logfar(sh_fx);

    shadow_fbo = rlLoadFramebuffer();
    rlEnableFramebuffer(shadow_fbo);
    shadow_tex = rlLoadTextureDepth(SHADOW_SIZE, SHADOW_SIZE, false);
    rlFramebufferAttach(shadow_fbo, shadow_tex, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);
    if (!rlFramebufferComplete(shadow_fbo)) TraceLog(LOG_WARNING, "render: shadow FBO incomplete");
    rlDisableFramebuffer();
}

void render_shutdown(void)
{
    free_target(&hdr);
    for (int i = 0; i < NBLOOM; i++) free_target(&bloom[i]);
    UnloadShader(sh_terrain); UnloadShader(sh_lm); UnloadShader(sh_fx); UnloadShader(sh_sky);
    UnloadShader(sh_down); UnloadShader(sh_up); UnloadShader(sh_final); UnloadShader(sh_depth);
    rlUnloadFramebuffer(shadow_fbo);
    rlUnloadTexture(shadow_tex);
}

Shader *render_terrain_shader(void) { return &sh_terrain; }
Shader *render_lm_shader(void) { return &sh_lm; }
Shader *render_fx_shader(void) { return &sh_fx; }
Shader *render_depth_shader(void) { return &sh_depth; }
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
}

// --- Scene ---------------------------------------------------------------------

static ViewCam cur_cam;
static Camera3D rl_cam;

void render_begin_scene(const ViewCam *cam, int w, int h)
{
    ensure_targets(w, h);
    cur_cam = *cam;
    rlEnableFramebuffer(hdr.fbo);
    rlViewport(0, 0, w, h);
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
    rlFrustum(-top * aspect, top * aspect, -top, top, NEAR, FAR_PLANE);
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
    set3(sh_lm, "sunDir", sun);
    set1(sh_lm, "exposure", exposure);
    set1(sh_lm, "groundAlbedo", ground_albedo);
    set3(sh_lm, "upDir", up_dir);
    bind_shadow(sh_lm);
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
    float tanhalf[2] = { (float)(th * cur_w / cur_h), (float)th };
    BeginShaderMode(sh_sky);
    set3(sh_sky, "camFwd", cam->fwd);
    set3(sh_sky, "camRight", right);
    set3(sh_sky, "camUp", upv);
    SetShaderValue(sh_sky, GetShaderLocation(sh_sky, "tanHalf"), tanhalf, SHADER_UNIFORM_VEC2);
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

    // Composite into the scene area of the output (bottom-left origin).
    bind_output();
    int ow = out_w ? out_w : GetRenderWidth(), oh = out_h ? out_h : GetRenderHeight();
    rlViewport(0, oh - cur_h, cur_w, cur_h);
    BeginShaderMode(sh_final);
    SetShaderValueTexture(sh_final, GetShaderLocation(sh_final, "scene"), hdr.tex);
    SetShaderValueTexture(sh_final, GetShaderLocation(sh_final, "bloom"), bloom[0].tex);
    set1(sh_final, "bloomGain", bloom_gain);
    set1(sh_final, "film", film_amount);
    set1(sh_final, "time", (float)GetTime());
    float res[2] = { (float)cur_w, (float)cur_h };
    SetShaderValue(sh_final, GetShaderLocation(sh_final, "resolution"), res, SHADER_UNIFORM_VEC2);
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
