// One step of the whole simulation, the same everywhere (game, lab, tests,
// snapshot generator): the LM and AGC in 1 ms steps, the crew and the mission
// logic every 0.1 s.
#ifndef SIM_H
#define SIM_H

void sim_advance(double seconds);

#endif
