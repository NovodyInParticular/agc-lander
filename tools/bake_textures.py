#!/usr/bin/env python3
"""Bake the tileable surface maps used by the LM exterior, the boulders and the cabin.

Every map is one RGBA image that tiles seamlessly:
  R, G  tangent-space normal x, y (0.5 = flat)
  B     roughness offset (0.5 = none, +-0.5 full range)
  A     albedo multiplier / 2 (0.5 = unchanged)

Usage: bake_textures.py OUT_DIR
The output is committed (data/textures), so the game build needs no Python imaging stack.
"""
import os
import sys

import numpy as np
from PIL import Image

N = 1024


def band_noise(n, lo, hi, beta, seed, aniso=(1.0, 1.0)):
    """Periodic noise by spectral synthesis: amplitude ~ f^-beta for lo <= f <= hi (cycles per tile)."""
    rng = np.random.default_rng(seed)
    fx = np.fft.fftfreq(n) * n
    fxx, fyy = np.meshgrid(fx, fx)
    f = np.sqrt((fxx * aniso[0]) ** 2 + (fyy * aniso[1]) ** 2)
    amp = np.zeros_like(f)
    m = (f >= lo) & (f <= hi)
    amp[m] = f[m] ** (-beta)
    spec = amp * np.exp(1j * rng.uniform(0, 2 * np.pi, (n, n)))
    x = np.fft.ifft2(spec).real
    x -= x.mean()
    return x / x.std()


def billow(n, lo, octaves, seed, gain=0.5):
    """Sum of |noise| octaves: sharp creases between smooth facets (crumpled foil, wrinkled cloth)."""
    out = np.zeros((n, n))
    a = 1.0
    f = lo
    for i in range(octaves):
        out += a * np.abs(band_noise(n, f, f * 2, 0.0, seed + i * 17))
        a *= gain
        f *= 2
    out -= out.min()
    return out / out.max()


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0, 1)
    return t * t * (3 - 2 * t)


def grid(n, tile_m):
    c = (np.arange(n) + 0.5) / n * tile_m
    return np.meshgrid(c, c)   # x (columns, u), y (rows, v) in metres


def cells(x, y, pitch, count, seed, lo, hi):
    """Per-cell random value for a count x count grid of square cells."""
    rng = np.random.default_rng(seed)
    table = rng.uniform(lo, hi, (count, count))
    ix = np.floor(x / pitch).astype(int) % count
    iy = np.floor(y / pitch).astype(int) % count
    return table[iy, ix]


def seam_distance(x, y, pitch):
    fx = np.abs((x / pitch) % 1.0 - 0.5)
    fy = np.abs((y / pitch) % 1.0 - 0.5)
    return (0.5 - np.maximum(fx, fy)) * pitch, (x / pitch) % 1.0 - 0.5, (y / pitch) % 1.0 - 0.5


def pack(height_m, tile_m, rough, albedo, path):
    n = height_m.shape[0]
    dx = tile_m / n
    gx = (np.roll(height_m, -1, 1) - np.roll(height_m, 1, 1)) / (2 * dx)
    gy = (np.roll(height_m, -1, 0) - np.roll(height_m, 1, 0)) / (2 * dx)
    inv = 1.0 / np.sqrt(gx * gx + gy * gy + 1.0)
    nx, ny = -gx * inv, -gy * inv
    img = np.stack([nx * 0.5 + 0.5, ny * 0.5 + 0.5, 0.5 + rough, albedo * 0.5], -1)
    img = (np.clip(img, 0, 1) * 255 + 0.5).astype(np.uint8)
    Image.fromarray(img, "RGBA").save(path, optimize=True)
    slope = np.sqrt(gx * gx + gy * gy)
    print(f"{os.path.basename(path)}: tile {tile_m} m, slope mean {slope.mean():.3f} max {slope.max():.2f}, "
          f"albedo {albedo.min():.2f}..{albedo.max():.2f}")


def bake_panel(path):
    """Riveted aluminium skin: 3 x 3 panels of 0.42 m, oil-canned, seams, rivet rows."""
    tile, pitch = 1.26, 0.42
    x, y = grid(N, tile)
    d, fx, fy = seam_distance(x, y, pitch)
    pillow = (1 - 4 * fx * fx) * (1 - 4 * fy * fy)
    h = cells(x, y, pitch, 3, 11, -0.0030, 0.0030) * pillow
    # Thin skins buckle: one or two shallow diagonal dents per panel.
    ang = cells(x, y, pitch, 3, 12, 0, np.pi)
    off = cells(x, y, pitch, 3, 13, -0.08, 0.08)
    across = (fx * np.cos(ang) + fy * np.sin(ang)) * pitch - off
    h += cells(x, y, pitch, 3, 14, -0.0016, 0.0016) * np.exp(-(across / 0.035) ** 2) * pillow
    h += 0.0007 * band_noise(N, 2, 24, 1.6, 15) * pillow
    h -= 0.0016 * smoothstep(0.007, 0.0, d)
    # Rivets: rows 28 mm inside each seam, 60 mm apart.
    rx = (x / 0.06) % 1.0 - 0.5
    ry = (y / 0.06) % 1.0 - 0.5
    rdist = np.sqrt(rx * rx + ry * ry) * 0.06
    band = smoothstep(0.012, 0.005, np.abs(d - 0.030))
    rivet = smoothstep(0.0065, 0.0025, rdist) * band
    h += 0.0009 * rivet
    h += 0.00004 * band_noise(N, 60, 500, 0.3, 16, aniso=(0.12, 1.0))   # brushed grain
    albedo = cells(x, y, pitch, 3, 17, 0.90, 1.07)
    albedo = albedo * (1 - 0.45 * smoothstep(0.009, 0.0, d))
    albedo *= 1 + 0.035 * band_noise(N, 1, 12, 1.5, 18)
    albedo *= 1 - 0.18 * smoothstep(0.011, 0.006, rdist) * band * (1 - smoothstep(0.0065, 0.0025, rdist))
    rough = cells(x, y, pitch, 3, 19, -0.07, 0.07) + 0.04 * band_noise(N, 4, 60, 1.2, 20)
    rough += 0.25 * smoothstep(0.008, 0.0, d)
    pack(h, tile, rough, albedo, path)


def bake_foil(path):
    """Crinkled Kapton foil: 2 x 2 sheets of 0.55 m joined with tape."""
    tile, pitch = 1.10, 0.55
    x, y = grid(N, tile)
    d, _, _ = seam_distance(x, y, pitch)
    crumple = billow(N, 5, 6, 31, gain=0.55)
    broad = band_noise(N, 1, 6, 1.5, 32)
    tape = smoothstep(0.030, 0.022, d)
    h = (0.0085 * crumple + 0.004 * broad) * (1 - 0.8 * tape) + 0.0006 * tape
    albedo = cells(x, y, pitch, 2, 33, 0.86, 1.08) * (0.86 + 0.26 * crumple)
    albedo = albedo * (1 - tape) + 1.10 * tape
    albedo *= 1 - 0.40 * smoothstep(0.004, 0.0, d)
    rough = 0.16 * (crumple - 0.5) + 0.03 * band_noise(N, 8, 80, 1.0, 34) - 0.10 * tape
    pack(h, tile, rough, albedo, path)


def bake_blanket(path):
    """Thermal blanket over the ascent stage: aluminised Kapton sheets 0.30 m wide, barely
    quilted, crinkled all over (a low Sun must glint off it, not cut it into pyramids)."""
    tile, pitch = 1.20, 0.30
    x, y = grid(N, tile)
    d, fx, fy = seam_distance(x, y, pitch)
    pillow = (1 - 4 * fx * fx) * (1 - 4 * fy * fy)
    crinkle = billow(N, 14, 6, 41, gain=0.62)
    fine = billow(N, 40, 4, 45, gain=0.55)
    h = cells(x, y, pitch, 4, 42, 0.0012, 0.0026) * pillow
    h += 0.0014 * crinkle + 0.00045 * fine
    h -= 0.0012 * smoothstep(0.008, 0.0, d)
    albedo = cells(x, y, pitch, 4, 43, 0.90, 1.06) * (0.90 + 0.18 * crinkle)
    albedo *= 1 - 0.30 * smoothstep(0.006, 0.0, d)
    rough = cells(x, y, pitch, 4, 44, -0.05, 0.05) + 0.14 * (crinkle - 0.5)
    pack(h, tile, rough, albedo, path)


def bake_rock(path):
    """Lunar basalt: craggy relief, pits, mottled dust-coated surface."""
    tile = 1.5
    x, y = grid(N, tile)
    relief = band_noise(N, 1, 400, 1.7, 51)
    crags = billow(N, 3, 6, 52, gain=0.6)
    pits = np.clip(band_noise(N, 12, 60, 0.5, 53) - 1.2, 0, None)
    h = 0.016 * relief + 0.02 * crags - 0.006 * pits
    mott = band_noise(N, 1, 30, 1.3, 54)
    speck = np.clip(band_noise(N, 80, 400, 0.0, 55) - 1.4, 0, None)
    albedo = 1.0 + 0.22 * mott + 0.25 * (crags - 0.5) + 0.5 * speck - 0.3 * np.clip(pits, 0, 1)
    rough = 0.10 * mott
    pack(h, tile, rough, np.clip(albedo, 0.4, 1.9), path)


def bake_cabin(path):
    """Cabin surfaces: painted sheet metal with a fine orange-peel texture, scuffs and wear."""
    tile = 0.60
    x, y = grid(N, tile)
    peel = band_noise(N, 90, 500, 0.6, 61)
    broad = band_noise(N, 1, 10, 1.6, 62)
    scuff = np.clip(band_noise(N, 3, 200, 1.1, 63, aniso=(0.08, 1.0)) - 1.5, 0, None)
    scuff += np.clip(band_noise(N, 3, 200, 1.1, 64, aniso=(1.0, 0.1)) - 1.7, 0, None)
    h = 0.00003 * peel + 0.0003 * broad
    albedo = 1.0 + 0.045 * broad + 0.02 * peel - 0.10 * np.clip(scuff, 0, 1.5)
    rough = 0.05 * broad - 0.12 * np.clip(scuff, 0, 1)
    pack(h, tile, rough, albedo, path)


def bake_cabin_wall(path):
    """Cabin pressure shell, as in photographs of LM-2 and the LM cabin: flat painted aluminium
    sheets with straight seams (0.45 x 0.30 m), a row of screw heads along every edge, one raised
    structural frame and one stringer per 0.9 m tile with rivet rows either side. No padding."""
    tile = 0.90
    x, y = grid(N, tile)
    peel = band_noise(N, 90, 500, 0.6, 71)
    broad = band_noise(N, 1, 10, 1.6, 72)
    scuff = np.clip(band_noise(N, 3, 200, 1.1, 73, aniso=(0.08, 1.0)) - 1.6, 0, None)
    scuff += np.clip(band_noise(N, 3, 200, 1.1, 74, aniso=(1.0, 0.1)) - 1.8, 0, None)
    # Sheets: distance to the nearest seam along each axis.
    px_, py_ = 0.45, 0.30
    sx = np.abs((x / px_) % 1.0 - 0.5) * px_      # 0 at the middle of a sheet, px_/2 at its seam
    sy = np.abs((y / py_) % 1.0 - 0.5) * py_
    dsx, dsy = px_ / 2 - sx, py_ / 2 - sy
    dseam = np.minimum(dsx, dsy)
    sheet = cells(x, y, 0.15, 6, 75, -1.0, 1.0)   # per-sheet tone (cells of 0.15 m grouped below)
    tone = cells(np.floor(x / px_) * 0.15 + 0.01, np.floor(y / py_) * 0.15 + 0.01, 0.15, 6, 76, 0.94, 1.05)
    oil = (1 - (2 * sx / px_) ** 2) * (1 - (2 * sy / py_) ** 2)   # thin sheets bow a hair
    h = 0.00025 * oil * sheet + 0.00003 * peel + 0.00015 * broad
    h -= 0.0007 * smoothstep(0.0018, 0.0, dseam)                 # the joint
    # Screw heads 12 mm in from each seam, 38 mm apart along it.
    def fastener_rows(dist_to_line, along, inset, pitch, r_head):
        a = ((along / pitch) % 1.0 - 0.5) * pitch
        d = np.sqrt(a * a + (dist_to_line - inset) ** 2)
        head = smoothstep(r_head, r_head * 0.45, d)
        ring = smoothstep(r_head * 1.9, r_head * 1.25, d) * (1 - head)
        return head, ring
    hx, rx_ = fastener_rows(dsx, y, 0.012, 0.038, 0.0032)
    hy, ry_ = fastener_rows(dsy, x, 0.012, 0.038, 0.0032)
    head = np.maximum(hx, hy)
    ring = np.maximum(rx_, ry_) * (1 - head)
    h += 0.0006 * head - 0.0002 * ring
    # Structural frame (vertical, at the tile edge) and stringer (horizontal, mid tile): raised flat strips.
    fx = np.abs(((x + tile / 2) % tile) - tile / 2)               # distance from x = 0 (mod tile)
    fy = np.abs(((y - tile / 2 + tile / 2) % tile) - tile / 2)     # distance from y = 0 (mod tile)
    frame = smoothstep(0.019, 0.016, fx)
    stringer = smoothstep(0.013, 0.010, fy) * (1 - frame)
    h += 0.0035 * frame + 0.0025 * stringer
    # Rivet rows on the strips, 9 mm in from their edges, 25 mm apart.
    fh, _ = fastener_rows(fx, y, 0.0085, 0.025, 0.0026)
    gh, _ = fastener_rows(fy, x, 0.0060, 0.025, 0.0024)
    riv = np.maximum(fh * frame, gh * stringer)
    h += 0.0005 * riv
    albedo = (1.0 + 0.04 * broad + 0.02 * peel - 0.10 * np.clip(scuff, 0, 1.5)) * tone
    albedo *= 1 - 0.40 * smoothstep(0.0015, 0.0, dseam) * (1 - frame) * (1 - stringer)
    albedo *= 1 - 0.38 * ring - 0.12 * head                       # dark recesses round the screws
    albedo *= 1 - 0.12 * smoothstep(0.022, 0.017, fx) * (1 - frame)   # the frame's shadow line
    rough = 0.05 * broad - 0.12 * np.clip(scuff, 0, 1) - 0.10 * head - 0.04 * frame
    pack(h, tile, rough, albedo, path)


def bake_regolith(path):
    """Lunar soil: granular fines, clods, pebbles and small craters with raised rims."""
    tile = 3.2
    x, y = grid(N, tile)
    rng = np.random.default_rng(83)
    h = 0.010 * band_noise(N, 1, 10, 1.3, 81)           # gentle hummocks
    h += 0.0035 * band_noise(N, 10, 500, 0.9, 82)       # granular soil
    clods = np.clip(band_noise(N, 30, 160, 0.3, 84) - 1.3, 0, None)
    h += 0.004 * clods
    albedo = 1.0 + 0.07 * band_noise(N, 1, 20, 1.2, 85) + 0.04 * band_noise(N, 40, 400, 0.4, 86) + 0.10 * np.clip(clods, 0, 1.5)

    def stamp(cx, cy, r):
        dx = (x - cx + tile / 2) % tile - tile / 2
        dy = (y - cy + tile / 2) % tile - tile / 2
        return np.sqrt(dx * dx + dy * dy) / r

    for i in range(90):                                  # craters, 4 cm .. 60 cm across
        r = min(0.30, 0.02 * (rng.uniform() + 0.03) ** -0.75)
        d = stamp(rng.uniform(0, tile), rng.uniform(0, tile), r)
        bowl = np.where(d < 1, (d * d - 1) * 0.20 * r, 0.0)
        rim = 0.045 * r * np.exp(-((d - 1.05) / 0.28) ** 2)
        h += bowl + rim
        albedo *= 1 - 0.10 * np.clip(1 - d, 0, 1) + 0.06 * np.exp(-((d - 1.1) / 0.3) ** 2)
    for i in range(420):                                 # pebbles, 1 .. 5 cm
        r = min(0.028, 0.006 * (rng.uniform() + 0.05) ** -0.6)
        d = stamp(rng.uniform(0, tile), rng.uniform(0, tile), r)
        inside = np.clip(1 - d * d, 0, 1)
        h += 0.75 * r * np.sqrt(inside)
        albedo *= 1 + 0.22 * (inside > 0)
    rough = 0.0 * h
    pack(h, tile, rough, np.clip(albedo, 0.5, 1.6), path)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "data/textures"
    os.makedirs(out, exist_ok=True)
    bake_panel(os.path.join(out, "lm_panel.png"))
    bake_foil(os.path.join(out, "lm_foil.png"))
    bake_blanket(os.path.join(out, "lm_blanket.png"))
    bake_rock(os.path.join(out, "rock.png"))
    bake_cabin(os.path.join(out, "cabin.png"))
    bake_cabin_wall(os.path.join(out, "cabin_wall.png"))
    bake_regolith(os.path.join(out, "regolith.png"))


if __name__ == "__main__":
    main()
