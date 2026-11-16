#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "agc_host.h"
#include "lm.h"
#include "snapshot.h"

// --- CRC-32 (IEEE), bitwise: blobs are small ---------------------------------

uint32_t snap_crc32(const void *data, size_t n, uint32_t crc)
{
    const uint8_t *p = data;
    crc = ~crc;
    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

// --- Raw byte I/O --------------------------------------------------------------

static void put(Snap *s, const void *p, size_t n)
{
    if (s->len + n > s->cap) {
        size_t c = s->cap ? s->cap : 4096;
        while (c < s->len + n) c *= 2;
        uint8_t *b = realloc(s->buf, c);
        if (!b) { s->error = 1; return; }
        s->buf = b;
        s->cap = c;
    }
    memcpy(s->buf + s->len, p, n);
    s->len += n;
}

static void get(Snap *s, void *p, size_t n)
{
    if (s->pos + n > s->len) { s->error = 1; memset(p, 0, n); return; }
    memcpy(p, s->buf + s->pos, n);
    s->pos += n;
}

// Little-endian scalar of `bytes` bytes (the build machines are little-endian;
// the byte order is still fixed explicitly so a blob is well defined).
static void scalar(Snap *s, void *v, int bytes, uint8_t type)
{
    if (s->mode == SNAP_LAYOUT) { s->crc = snap_crc32(&type, 1, s->crc); return; }
    uint8_t b[8];
    if (s->mode == SNAP_SAVE) {
        uint64_t x = 0;
        memcpy(&x, v, (size_t)bytes);
        for (int i = 0; i < bytes; i++) b[i] = (uint8_t)(x >> (8 * i));
        put(s, b, (size_t)bytes);
    } else {
        get(s, b, (size_t)bytes);
        uint64_t x = 0;
        for (int i = 0; i < bytes; i++) x |= (uint64_t)b[i] << (8 * i);
        memcpy(v, &x, (size_t)bytes);
    }
}

void snap_u8(Snap *s, uint8_t *v) { scalar(s, v, 1, 'b'); }
void snap_i8(Snap *s, int8_t *v) { scalar(s, v, 1, 'c'); }
void snap_u16(Snap *s, uint16_t *v) { scalar(s, v, 2, 'w'); }
void snap_i16(Snap *s, int16_t *v) { scalar(s, v, 2, 'h'); }
void snap_u32(Snap *s, uint32_t *v) { scalar(s, v, 4, 'u'); }
void snap_i32(Snap *s, int32_t *v) { scalar(s, v, 4, 'i'); }
void snap_u64(Snap *s, uint64_t *v) { scalar(s, v, 8, 'q'); }
void snap_f32(Snap *s, float *v) { scalar(s, v, 4, 'f'); }
void snap_f64(Snap *s, double *v) { scalar(s, v, 8, 'd'); }

void snap_int(Snap *s, int *v)
{
    int32_t x = (int32_t)*v;
    snap_i32(s, &x);
    if (s->mode == SNAP_LOAD) *v = (int)x;
}

void snap_uint(Snap *s, unsigned *v)
{
    uint32_t x = (uint32_t)*v;
    snap_u32(s, &x);
    if (s->mode == SNAP_LOAD) *v = (unsigned)x;
}

void snap_v3(Snap *s, V3 *v) { snap_f64(s, &v->x); snap_f64(s, &v->y); snap_f64(s, &v->z); }
void snap_q4(Snap *s, Q4 *v) { snap_f64(s, &v->w); snap_f64(s, &v->x); snap_f64(s, &v->y); snap_f64(s, &v->z); }
void snap_m3(Snap *s, M3 *v) { for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) snap_f64(s, &v->m[i][j]); }

void snap_chars(Snap *s, char *v, int n)
{
    if (s->mode == SNAP_LAYOUT) {
        uint8_t t[5] = { 's', (uint8_t)n, (uint8_t)(n >> 8), (uint8_t)(n >> 16), (uint8_t)(n >> 24) };
        s->crc = snap_crc32(t, 5, s->crc);
        return;
    }
    if (s->mode == SNAP_SAVE) put(s, v, (size_t)n);
    else get(s, v, (size_t)n);
}

void snap_i16v(Snap *s, int16_t *v, int n) { for (int i = 0; i < n; i++) snap_i16(s, &v[i]); }
void snap_u16v(Snap *s, uint16_t *v, int n) { for (int i = 0; i < n; i++) snap_u16(s, &v[i]); }
void snap_intv(Snap *s, int *v, int n) { for (int i = 0; i < n; i++) snap_int(s, &v[i]); }
void snap_f32v(Snap *s, float *v, int n) { for (int i = 0; i < n; i++) snap_f32(s, &v[i]); }
void snap_f64v(Snap *s, double *v, int n) { for (int i = 0; i < n; i++) snap_f64(s, &v[i]); }

void snap_count(Snap *s, int *n)
{
    if (s->mode == SNAP_LAYOUT) { uint8_t t = 'n'; s->crc = snap_crc32(&t, 1, s->crc); *n = 0; return; }
    snap_int(s, n);
}

// --- Sections ------------------------------------------------------------------

typedef struct { char tag[5]; SnapFn fn; int core; } Section;
#define MAX_SECTIONS 16
static Section sections[MAX_SECTIONS];
static int nsections;

static void add_section(const char tag[5], SnapFn fn, int core)
{
    for (int i = 0; i < nsections; i++)
        if (!memcmp(sections[i].tag, tag, 4)) { sections[i].fn = fn; return; }
    if (nsections >= MAX_SECTIONS) return;
    memcpy(sections[nsections].tag, tag, 4);
    sections[nsections].tag[4] = 0;
    sections[nsections].fn = fn;
    sections[nsections].core = core;
    nsections++;
}

void snapshot_register(const char tag[5], SnapFn fn) { add_section(tag, fn, 0); }

static void core_sections(void)
{
    static int done;
    if (done) return;
    done = 1;
    add_section("AGC ", agc_host_snap, 1);
    add_section("LM  ", lm_snap, 1);
    add_section("CSM ", csm_snap, 1);
    add_section("CREW", crew_snap, 1);
    add_section("MISN", ascent_snap, 1);
    add_section("SCEN", scenario_snap, 1);
}

static uint32_t layout_of(SnapFn fn)
{
    Snap s = { 0 };
    s.mode = SNAP_LAYOUT;
    fn(&s);
    return s.crc;
}

// --- Header ----------------------------------------------------------------------

#define HDR_BYTES (4 * 4 + 40 + 4 + 8 + 4)

static void put_u32(Snap *s, uint32_t v) { snap_u32(s, &v); }

size_t snapshot_save(uint8_t **out, const char *label, int milestone)
{
    core_sections();
    Snap s = { 0 };
    s.mode = SNAP_SAVE;
    put_u32(&s, SNAP_MAGIC);
    put_u32(&s, SNAP_VERSION);
    put_u32(&s, 0);   // total bytes, patched below
    put_u32(&s, agc_host_rope_crc());
    char lab[40] = { 0 };
    snprintf(lab, sizeof lab, "%s", label ? label : "");
    snap_chars(&s, lab, 40);
    int32_t ms = milestone;
    snap_i32(&s, &ms);
    double t = lm.t;
    snap_f64(&s, &t);
    int32_t n = nsections;
    snap_i32(&s, &n);
    for (int i = 0; i < nsections; i++) {
        uint32_t lay = layout_of(sections[i].fn);
        snap_chars(&s, sections[i].tag, 4);
        put_u32(&s, lay);
        size_t len_at = s.len;
        put_u32(&s, 0);
        size_t start = s.len;
        sections[i].fn(&s);
        uint32_t plen = (uint32_t)(s.len - start);
        for (int k = 0; k < 4; k++) s.buf[len_at + k] = (uint8_t)(plen >> (8 * k));
    }
    uint32_t total = (uint32_t)s.len;
    for (int k = 0; k < 4; k++) s.buf[8 + k] = (uint8_t)(total >> (8 * k));
    if (s.error) { free(s.buf); *out = NULL; return 0; }
    *out = s.buf;
    return s.len;
}

static int read_header(Snap *s, SnapInfo *info, uint32_t *nsec, char *err, size_t errn)
{
    uint32_t magic = 0, version = 0, total = 0, rope = 0;
    snap_u32(s, &magic);
    snap_u32(s, &version);
    snap_u32(s, &total);
    snap_u32(s, &rope);
    SnapInfo in = { 0 };
    snap_chars(s, in.label, 40);
    in.label[39] = 0;
    int32_t ms = 0;
    snap_i32(s, &ms);
    in.milestone = ms;
    snap_f64(s, &in.agc_time);
    int32_t n = 0;
    snap_i32(s, &n);
    if (s->error || magic != SNAP_MAGIC) { snprintf(err, errn, "not a snapshot"); return -1; }
    if (version != SNAP_VERSION) { snprintf(err, errn, "snapshot version %u, this build reads %u", version, SNAP_VERSION); return -1; }
    if (total != s->len) { snprintf(err, errn, "snapshot is %zu bytes, header says %u", s->len, total); return -1; }
    if (rope != agc_host_rope_crc()) { snprintf(err, errn, "snapshot was made with a different AGC rope"); return -1; }
    if (info) *info = in;
    *nsec = (uint32_t)n;
    return 0;
}

int snapshot_peek(const uint8_t *buf, size_t len, SnapInfo *info)
{
    Snap s = { 0 };
    s.mode = SNAP_LOAD;
    s.buf = (uint8_t *)buf;
    s.len = len;
    uint32_t n;
    char err[8];
    return read_header(&s, info, &n, err, sizeof err);
}

int snapshot_load(const uint8_t *buf, size_t len, SnapInfo *info, char *err, size_t errn)
{
    core_sections();
    Snap s = { 0 };
    s.mode = SNAP_LOAD;
    s.buf = (uint8_t *)buf;
    s.len = len;
    uint32_t n;
    if (read_header(&s, info, &n, err, errn)) return -1;
    size_t first = s.pos;

    // Pass 1: every section we know must match this build's field list, and
    // every core section must be there - before anything is changed.
    int have[MAX_SECTIONS] = { 0 };
    for (uint32_t i = 0; i < n; i++) {
        char tag[5] = { 0 };
        uint32_t lay = 0, plen = 0;
        snap_chars(&s, tag, 4);
        snap_u32(&s, &lay);
        snap_u32(&s, &plen);
        if (s.error || s.pos + plen > s.len) { snprintf(err, errn, "snapshot is cut short"); return -1; }
        for (int k = 0; k < nsections; k++)
            if (!memcmp(sections[k].tag, tag, 4)) {
                if (lay != layout_of(sections[k].fn)) {
                    snprintf(err, errn, "snapshot section %s is from another build (run make snapshots)", tag);
                    return -1;
                }
                have[k] = 1;
            }
        s.pos += plen;
    }
    for (int k = 0; k < nsections; k++)
        if (sections[k].core && !have[k]) { snprintf(err, errn, "snapshot has no %s section", sections[k].tag); return -1; }

    // Pass 2: load.
    s.pos = first;
    for (uint32_t i = 0; i < n; i++) {
        char tag[5] = { 0 };
        uint32_t lay = 0, plen = 0;
        snap_chars(&s, tag, 4);
        snap_u32(&s, &lay);
        snap_u32(&s, &plen);
        size_t end = s.pos + plen;
        for (int k = 0; k < nsections; k++)
            if (!memcmp(sections[k].tag, tag, 4)) {
                Snap sub = { 0 };
                sub.mode = SNAP_LOAD;
                sub.buf = s.buf + s.pos;
                sub.len = plen;
                sections[k].fn(&sub);
                if (sub.error || sub.pos != sub.len) {
                    snprintf(err, errn, "snapshot section %s did not read back cleanly", tag);
                    return -1;
                }
            }
        s.pos = end;
    }
    return 0;
}
