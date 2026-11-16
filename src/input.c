// Virtual keys for the Quest build: see input.h.
#ifdef AGC_VR

#include "raylib.h"

#include "vr.h"

#define NKEYS 512
static unsigned char now[NKEYS], before[NKEYS];

void in_begin_frame(void)
{
    for (int i = 0; i < NKEYS; i++) { before[i] = now[i]; now[i] = 0; }
}

void in_set_key(int key, int down)
{
    if (key > 0 && key < NKEYS && down) now[key] = 1;
}

int in_key_down(int key)
{
    if (key > 0 && key < NKEYS && now[key]) return 1;
    return IsKeyDown(key);
}

int in_key_pressed(int key)
{
    if (key > 0 && key < NKEYS && now[key] && !before[key]) return 1;
    return IsKeyPressed(key);
}

float in_frame_time(void) { return vr_frame_dt(); }

#endif
