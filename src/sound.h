// Cabin sound: everything is synthesised (no audio files), mixed in one stream.
//
// Only what the crew would hear inside the pressurised cabin is modelled:
// the environmental-control fans and 400 Hz inverter whine, the descent
// engine felt through the structure, RCS jets thumping the hull, panel
// switches and DSKY keys, the master alarm tone, the touchdown.
// Outside views are silent: there is no air out there.
#ifndef SOUND_H
#define SOUND_H

void sound_init(void);
void sound_shutdown(void);

typedef struct {
    int inside;        // camera is in the cabin
    int running;       // flying and not paused
    float thrust;      // descent engine, 0..1 of full thrust
    float aps;         // ascent engine, 0..1 (on or off: it is not throttled)
    float rcs;         // summed RCS jet levels (0 = quiet)
    int rcs_hits;      // jets that started firing since the last update
    int alarm;         // master alarm lit
    float fans;        // cabin fans: 1 = suit fan alone, more with the cabin fan, 0 = none
} SoundState;

// Once per frame.
void sound_update(const SoundState *s, float dt);

// One-shot events.
void sound_switch(void);              // panel switch, button, knob
void sound_key(void);                 // DSKY key
void sound_touchdown(float speed);    // footpads meet the surface, m/s
void sound_crash(void);
void sound_stage(void);               // ABORT STAGE / lift-off: explosive bolts, nuts and guillotines firing
void sound_step(float speed);         // a boot landing on the surface (felt through the suit), m/s

void sound_set_muted(int muted);
int sound_muted(void);

#endif
