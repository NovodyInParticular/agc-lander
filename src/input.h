// Quest build: the game reads the keyboard through raylib; on the headset the
// controller buttons are presented to it as "virtual" keys, so the game logic
// stays the same. Include after raylib.h. On the PC this header does nothing.
#ifndef INPUT_H
#define INPUT_H

#ifdef AGC_VR

// Virtual keys (raylib KEY_* codes), set once per frame by the VR input mapping.
void in_begin_frame(void);            // forget last frame's virtual keys (keeps them for edge detection)
void in_set_key(int key, int down);
int in_key_down(int key);             // real keyboard or virtual
int in_key_pressed(int key);          // went down this frame
float in_frame_time(void);

// Controller-only functions with no keyboard key of their own.
#define VKEY_PRO 400      // DSKY PRO held (B button)

#define IsKeyDown(k) in_key_down(k)
#define IsKeyPressed(k) in_key_pressed(k)
#define GetFrameTime() in_frame_time()

#endif
#endif
