# Third-party code

| Directory | Upstream | Version | License |
|---|---|---|---|
| `raylib/` (src only) | https://github.com/raysan5/raylib | 5.5 | zlib (`raylib/LICENSE`) |
| `virtualagc/` (yaAGC core, yaYUL, Luminary099 source) | https://github.com/virtualagc/virtualagc | commit 3351d115 | GPLv2 (`virtualagc/COPYING`); the AGC source code is public domain |

`data/a11_pdi_lgc.c` is extracted from the Project Apollo - NASSP "Apollo 11 - 10 - PDI T-15m"
scenario (GPLv2, https://github.com/orbiternassp/NASSP) by `tools/extract_nassp.py`. To regenerate
it, clone NASSP into `third_party/ref/nassp`.
