// Hand controllers on the headset (Quest build; replaces controls.c).
//
// Right thumbstick = the ACA (attitude: forward/back pitch, left/right roll),
// left thumbstick = yaw (left/right) and the ROD switch (a flick up or down).
// A = ENGINE STOP, B = PRO (hold), left trigger = ENTR.
// Docking (controls_vr_docking set): the left stick and left grip/trigger are
// the translation controller instead (the game maps them to the arrow keys and
// T/G), and yaw is the right stick left/right with the right grip held.
// A keyboard still works (PC stand-in, or one paired with the headset).

#include <math.h>
#include <string.h>

#include "raylib.h"

#include "controls.h"
#include "input.h"
#include "vr.h"

int controls_vr_docking;

static double dz(double v)
{
    const double d = 0.15;
    if (fabs(v) < d) return 0;
    return (v - copysign(d, v)) / (1 - d);
}

void controls_poll(Controls *c)
{
    memset(c, 0, sizeof *c);
    c->pitch = (IsKeyDown(KEY_W) ? 1 : 0) - (IsKeyDown(KEY_S) ? 1 : 0);
    c->roll = (IsKeyDown(KEY_D) ? 1 : 0) - (IsKeyDown(KEY_A) ? 1 : 0);
    c->yaw = (IsKeyDown(KEY_Q) ? 1 : 0) - (IsKeyDown(KEY_E) ? 1 : 0);
    c->pitch *= 0.6; c->roll *= 0.6; c->yaw *= 0.6;
    c->rod = (IsKeyPressed(KEY_UP) ? 1 : 0) - (IsKeyPressed(KEY_DOWN) ? 1 : 0);
    c->stop = IsKeyPressed(KEY_SPACE);

    const VrInput *in = vr_input();
    if (controls_vr_docking) {
        c->pitch += dz(in->ry);
        if (in->rgrip > 0.5f) c->yaw -= dz(in->rx);
        else c->roll += dz(in->rx);
        if (IsKeyDown(VKEY_PRO)) c->pro = 1;
        if (c->pitch > 1) c->pitch = 1;
        if (c->pitch < -1) c->pitch = -1;
        if (c->roll > 1) c->roll = 1;
        if (c->roll < -1) c->roll = -1;
        if (c->yaw > 1) c->yaw = 1;
        if (c->yaw < -1) c->yaw = -1;
        return;
    }
    c->pitch += dz(in->ry);
    c->roll += dz(in->rx);
    // The left stick does two jobs, so each needs a clear push in its own direction.
    if (fabs(in->lx) > fabs(in->ly)) c->yaw -= dz(in->lx);
    static int rod_was;
    int rod_now = in->ly > 0.7f && fabs(in->lx) < 0.5f ? 1 : in->ly < -0.7f && fabs(in->lx) < 0.5f ? -1 : 0;
    if (rod_now && rod_now != rod_was) c->rod += rod_now;
    rod_was = rod_now;
    if (IsKeyDown(VKEY_PRO)) c->pro = 1;
    static int trig_was;
    int trig = in->ltrig > 0.6f;
    if (trig && !trig_was) c->enter = 1;
    trig_was = trig;

    if (c->pitch > 1) c->pitch = 1;
    if (c->pitch < -1) c->pitch = -1;
    if (c->roll > 1) c->roll = 1;
    if (c->roll < -1) c->roll = -1;
    if (c->yaw > 1) c->yaw = 1;
    if (c->yaw < -1) c->yaw = -1;
}
