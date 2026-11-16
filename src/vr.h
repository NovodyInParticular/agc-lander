// Headset interface (Quest build). Two implementations:
//   vr_openxr.c  OpenXR on the headset (Android)
//   vr_sim.c     a stand-in on the PC: two fixed eyes drawn side by side, so the
//                stereo path and the Quest shaders can be tested without a headset
//
// Tracking space: x right, y up, z toward the user (so "forward" is -z),
// origin at the head position when the view was last recentred, metres.
#ifndef VR_H
#define VR_H

#include "vmath.h"

typedef struct {
    V3 pos;                               // eye position, tracking space
    V3 fwd, up;                           // unit vectors, tracking space
    double tan_l, tan_r, tan_d, tan_u;    // frustum edge tangents (left and down negative)
    int w, h;                             // size to render at, pixels
    unsigned int fbo;                     // framebuffer to render into (colour + depth)
} VrEye;

typedef struct {
    float lx, ly, rx, ry;                 // thumbsticks, -1..1, y positive away from the user
    float ltrig, rtrig, lgrip, rgrip;     // 0..1
    int a, b, x, y, menu, lclick, rclick; // buttons held
    int aim_ok[2];                        // 0 = left hand, 1 = right hand
    V3 aim_pos[2], aim_dir[2];            // pointing ray, tracking space
    V3 head_pos, head_fwd, head_up;       // centre of the two eyes
} VrInput;

// After the window (GL context) exists. Returns 0 on success.
int vr_init(void);
void vr_shutdown(void);

// 1 = the eye targets convert linear colour to sRGB themselves (headset
// swapchains); 0 = plain 8-bit targets. Valid after vr_init().
int vr_srgb_target(void);

// Start a frame: system events, controller state, wait for the display.
// Returns 1 when a frame must be drawn (then call vr_eye_begin/end for both
// eyes and vr_end_frame). Returns 0 when the headset is not showing the game
// (taken off, system menu): draw nothing and do not call vr_end_frame.
// *dt: seconds since the last drawn frame. *quit: the system wants the game closed.
int vr_begin_frame(float *dt, int *quit);
void vr_eye_begin(int eye, VrEye *e);
void vr_eye_end(int eye);
void vr_end_frame(void);

const VrInput *vr_input(void);
float vr_frame_dt(void);                  // the dt of the current frame
void vr_haptic(int hand, float strength, float seconds);

// How hard the frame is to draw (1 = fits the display rate exactly; above 1 =
// too slow) and the resolution scale in use; for the on-screen statistics.
void vr_stats(float *load, float *scale, float *fps);

// The floating 2D display as a compositor layer (sharp text). vr_ui_begin:
// between vr_begin_frame and vr_end_frame; returns the framebuffer (w x h,
// same colour encoding as the eyes) to draw it into, or 0 when there is no
// such layer (draw it into the scene instead). vr_ui_end: its centre and size
// in tracking space (metres), facing the user.
unsigned int vr_ui_begin(int w, int h);
void vr_ui_end(V3 centre, double width, double height);

// PC stand-in only: save both eyes side by side as a picture.
void vr_sim_save(const char *path);

#endif
