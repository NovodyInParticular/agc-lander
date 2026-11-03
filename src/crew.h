// Crew procedures: an automatic crew that answers Luminary's DSKY requests
// the way Armstrong and Aldrin did, plus prompts/callouts for a human player.
#ifndef CREW_H
#define CREW_H

typedef struct {
    int auto_crew;            // 1: crew performs procedures automatically
    int auto_land;            // 1: let P65 land; 0: crew takes over in P66
    char prompt[160];         // what the computer is asking for right now
    char callout[160];        // most recent callout
    double callout_time;      // sim time of the callout
    int landed;
    int hint;                 // HINT_*: plain-English version of the prompt
} Crew;

enum { HINT_NONE, HINT_FINE_ALIGN, HINT_MANEUVER, HINT_MANEUVER_DONE, HINT_ENABLE_IGN,
       HINT_ARM, HINT_V57, HINT_ALARM };

extern Crew crew;

void crew_reset(void);

// Call every ~0.1 s of simulated time (after lm_run).
void crew_step(void);

// Type keys into the DSKY: V N E R C K + - P(press) p(release) 0-9.
void crew_type(const char *keys);

void crew_say(const char *text);

#endif
