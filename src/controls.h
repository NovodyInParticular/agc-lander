// Keyboard, gamepad and flight-stick input for the LM hand controllers.
#ifndef CONTROLS_H
#define CONTROLS_H

typedef struct {
    double pitch, yaw, roll;   // ACA (rotational hand controller), -1..1
    int tx, ty, tz;            // TTCA translation, -1/0/+1 per axis (X up, Y right, Z forward)
    int rod;                   // ROD switch clicks this frame (+1 up / -1 down)
    int stop;                  // ENGINE STOP pressed
    int pro;                   // DSKY PRO held
    int enter;                 // DSKY ENTR pressed
    int camera, pause;         // next camera / pause pressed
    char device[400];          // connected controllers, for display
} Controls;

void controls_poll(Controls *c);

#endif
