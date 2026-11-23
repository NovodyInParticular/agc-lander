// Milestones of the canonical Apollo 11 descent: save states taken from one
// flight of the automatic crew from the NASSP PDI-15 state over the game's
// terrain (`make snapshots`, agc-lab snapgen), embedded in the program, plus
// a once-a-second track of that flight.
//
// The menu scenarios start from them, and "advance to next event" jumps to
// the next one - but only when the player's flight is the canonical one at
// this moment (same orbit and attitude, same program, switches and AGC flags)
// and the canonical crew does nothing between now and that milestone.
#ifndef MILESTONES_H
#define MILESTONES_H

#include <stddef.h>
#include <stdint.h>

#include "vmath.h"

enum {
    MS_TIG30,      // 30 s before ignition, engine armed (the POWERED DESCENT scenario; V99 comes at TIG-5)
    MS_RADAR,      // landing radar data good, V57 not yet keyed
    MS_HIGHGATE,   // 2600 m, about to pitch over into P64 (the HIGH GATE scenario)
    MS_150M,       // 150 m in P64 (FINAL 500 FT: the player takes over in P66)
    // Scenarios from other flights (not on the canonical track):
    MS_BOULDERS,   // Luminary aimed onto West crater's boulder-strewn rim; 150 m, ~100 s of hover fuel
    MS_SITE2,      // retargeted 7 km downrange at PDI-15; high gate
    // The canonical ascent (the LUNAR LIFT-OFF scenario's flight, automatic crew):
    MS_INSERTION,  // P12 has shut the ascent engine down in orbit
    MS_COUNT
};

// Name used for labels ("TIG-30", "HIGH GATE", ...).
const char *milestone_name(int ms);
// The embedded save state (NULL, 0 if this build has none).
const uint8_t *milestone_blob(int ms, size_t *len);
// Restore it. 0 = done; -1 with a reason (nothing changed).
int milestone_load(int ms, char *err, size_t errn);
// AGC time (lm.t) of the milestone, 0 if unknown.
double milestone_time(int ms);

// --- Canonical track ------------------------------------------------------------

// Where the scenario flights aim (terrain plane, m east, m south of the site).
#define BOULDERS_X -270.0   // West crater's east rim (the crater: -380, -120, radius 95 m)
#define BOULDERS_Z -120.0
#define SITE2_X -7000.0
#define SITE2_Z -60.0

#define TRACK_NFLAGS 16   // FLAGWRD0..9, FLGWRD10..13, DAPBOOLS, RADMODES

typedef struct {
    double t;
    V3 r, v;                     // inertial, m and m/s
    Q4 q;                        // attitude
    uint16_t flags[TRACK_NFLAGS];
    uint8_t prog, verb, noun;    // two-digit values, 255 = blank
    uint8_t flashing, arm, mode, dps;
    uint8_t crew;                // the crew acted during the second before this sample
} TrackSample;

// Capture a sample of the current state (the generator; also the live check).
void track_sample_now(TrackSample *s, int crew_acted);
// The embedded track (count 0 if none).
int track_count(void);
const TrackSample *track_at_index(int i);

// Serialise a track (generator) / parse the embedded one.
size_t track_encode(const TrackSample *s, int n, uint8_t **out);

// Is the flight now the canonical one? 1 = yes. Otherwise why (for debugging).
int milestone_on_track(char *why, size_t whyn);

// The milestone "advance to next event" may jump to now, or -1 (why, if
// given, says what stands in the way).
int milestone_next_event(char *why, size_t whyn);

// What the player sees after the jump: "Landing radar data good", ...
const char *milestone_title(int ms);

// What Mission Control says after the jump to a milestone ("" if nothing).
const char *milestone_callout(int ms);

// Jump to the next event if one is offered: returns the milestone, or -1.
// Who flies (crew.auto_crew, auto_land) and the rendezvous radar fault stay as they were.
int milestone_advance(void);

#endif
