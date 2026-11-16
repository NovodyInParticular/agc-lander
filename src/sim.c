#include "ascent.h"
#include "crew.h"
#include "lm.h"
#include "sim.h"

void sim_advance(double seconds)
{
    // 0.1 s slices so the crew can react between them.
    while (seconds > 1e-9) {
        double s = seconds > 0.1 ? 0.1 : seconds;
        lm_run(s);
        crew_step();
        ascent_step(s);
        seconds -= s;
    }
}
