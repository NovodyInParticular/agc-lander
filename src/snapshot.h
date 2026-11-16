// Save states: everything sim_advance needs to resume a flight exactly - the
// AGC (core, the engine's hidden counters, the host's queued keystrokes and
// counter pulses), the LM, the CSM, the crew procedure, the ascent and docking,
// and the scenario. Game-side modules (moonwalk, cockpit) are not saved; a
// program that needs them can add sections with snapshot_register.
//
// A blob is a header followed by tagged sections. Each module has one
// serialiser that both saves and loads (so the two cannot drift apart); a
// third "layout" pass fingerprints its field list, and a blob whose
// fingerprint no longer matches the code is refused, not misread. Fields are
// written one by one, little-endian, bitfields widened to bytes: the same blob
// loads in the Linux and Windows builds.
#ifndef SNAPSHOT_H
#define SNAPSHOT_H

#include <stddef.h>
#include <stdint.h>

#include "vmath.h"

#define SNAP_MAGIC 0x504E5341u   // "ASNP"
#define SNAP_VERSION 1

enum { SNAP_SAVE, SNAP_LOAD, SNAP_LAYOUT };

typedef struct Snap {
    int mode;
    uint8_t *buf;        // save: growing buffer; load: the section being read
    size_t len, cap, pos;
    uint32_t crc;        // layout fingerprint of the current section
    int error;
} Snap;

// Field serialisers: save writes *v, load reads into *v, layout only fingerprints.
void snap_u8(Snap *s, uint8_t *v);
void snap_i8(Snap *s, int8_t *v);
void snap_u16(Snap *s, uint16_t *v);
void snap_i16(Snap *s, int16_t *v);
void snap_u32(Snap *s, uint32_t *v);
void snap_i32(Snap *s, int32_t *v);
void snap_int(Snap *s, int *v);              // stored as 32 bits
void snap_uint(Snap *s, unsigned *v);
void snap_u64(Snap *s, uint64_t *v);
void snap_f32(Snap *s, float *v);
void snap_f64(Snap *s, double *v);
void snap_v3(Snap *s, V3 *v);
void snap_q4(Snap *s, Q4 *v);
void snap_m3(Snap *s, M3 *v);
void snap_chars(Snap *s, char *v, int n);    // fixed-size text / byte array
void snap_i16v(Snap *s, int16_t *v, int n);
void snap_u16v(Snap *s, uint16_t *v, int n);
void snap_intv(Snap *s, int *v, int n);
void snap_f32v(Snap *s, float *v, int n);
void snap_f64v(Snap *s, double *v, int n);
// A count that varies with the data (variable-length lists): the layout pass
// fingerprints only the list's presence.
void snap_count(Snap *s, int *n);

// Extra sections (game-only modules: moonwalk, cockpit, game flow). A blob
// without one leaves that module as it is; a loader without one skips it.
typedef void (*SnapFn)(Snap *s);
void snapshot_register(const char tag[5], SnapFn fn);

typedef struct {
    char label[40];      // "TIG-30 s", "HIGH GATE", ...
    int milestone;       // index in the canonical descent, -1 = not a milestone
    double agc_time;     // lm.t at capture, s
} SnapInfo;

// Capture now: returns a malloc'd blob and its size.
size_t snapshot_save(uint8_t **out, const char *label, int milestone);
// Restore. Returns 0, or -1 with a reason in err (nothing is changed then).
int snapshot_load(const uint8_t *buf, size_t len, SnapInfo *info, char *err, size_t errn);
// Header only (no state touched). Returns 0 if it looks like a snapshot for this build.
int snapshot_peek(const uint8_t *buf, size_t len, SnapInfo *info);

uint32_t snap_crc32(const void *data, size_t n, uint32_t crc);

// Per-module serialisers (core sections, always present).
void agc_host_snap(Snap *s);
void lm_snap(Snap *s);
void csm_snap(Snap *s);
void crew_snap(Snap *s);
void ascent_snap(Snap *s);
void scenario_snap(Snap *s);

#endif
