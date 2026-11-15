// Internal interface between the cockpit geometry/renderer (cockpit.c) and
// the panel artwork and switch logic (cockpit_panels.c).
#ifndef COCKPIT_INT_H
#define COCKPIT_INT_H

#include "raylib.h"
#include "vmath.h"

// LM panel numbering follows the real vehicle where it can.
enum { P1, P2, P3, P4, P5, P6, P8, P11, P12, P14, P16, NPANELS };

typedef struct {
    const char *name;
    V3 tl, tr, bl;      // body-frame corners (m): top-left, top-right, bottom-left
    float dens;         // atlas pixels per mm
    float w_mm, h_mm;   // derived from the corners
    int ax, ay, aw, ah; // atlas rectangle (pixels)
} PanelDef;

extern PanelDef cp_panels[NPANELS];

// Things the panel code registers while drawing, consumed by
// cockpit.c for 3D levers/knobs and mouse picking.
enum { LEV_TOGGLE, LEV_KNOB, LEV_THUMB };
typedef struct {
    int panel, kind;
    float x, y;         // mm on the panel
    float val;          // toggle: -1..1 (down..up); knob: angle (rad); thumb: 0..1
    float size;         // mm (knob radius / lever length)
    int guard;          // toggle has a guard
    unsigned char r, g, b;
} Lever;

enum { HOT_TOGGLE, HOT_KNOB, HOT_BUTTON, HOT_DSKY, HOT_DEDA, HOT_CB, HOT_INFO };
typedef struct {
    int panel, kind;
    Rectangle r;        // mm
    const char *id;     // state key (string literal; pointer identity)
    int idx;            // button / key index
} Hot;

// Per panel, so that panels can be redrawn at different rates.
#define CP_MAXLEV 96
#define CP_MAXHOT 280
typedef struct {
    Lever lev[CP_MAXLEV];
    int nlev;
    Hot hot[CP_MAXHOT];
    int nhot;
} PanelItems;
extern PanelItems cp_items[NPANELS];

// Set when a switch was operated: every panel is redrawn on the next frame.
extern int cp_dirty;

// Record one panel onto the current canvas layer (pc_begin .. pc_end), in
// millimetres from its top-left corner, and refresh its levers and hotspots.
void cp_draw_panel(int panel);

// Per-frame logic (timers, spring switches, syncing switches to the sim).
void cp_panels_update(float dt);

// Mouse: click on a hotspot at (u, v) mm; button 0 = left.
void cp_click(const Hot *h, float u, float v);
// Tooltip text for a hotspot (static buffer).
const char *cp_tooltip(const Hot *h, float u, float v);

// Lighting levels set from the panel knobs (0..1).
float cp_flood_level(void);
float cp_integral_level(void);
float cp_numerics_level(void);

#endif
