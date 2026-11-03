// Hand controllers: keyboard, gamepads (Xbox/PlayStation layout via raylib)
// and flight sticks / HOTAS read raw through GLFW (sticks usually aren't in
// the gamepad mapping database, so raylib alone doesn't see them).
//
// In the LM the Commander's right hand held the ACA (attitude controller,
// a pistol grip) and his left hand the TTCA (thrust/translation T-handle).

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "raylib.h"
#include "external/glfw/include/GLFW/glfw3.h"

#include "controls.h"

static double dz(double v)
{
    const double d = 0.12;
    if (fabs(v) < d) return 0;
    return (v - copysign(d, v)) / (1 - d);
}

// A real flight stick: not a mapped gamepad, has axes and buttons, and is not
// one of the side devices a DualSense/DualShock exposes on Linux (touchpad,
// motion sensors) - those would otherwise grab the stick axes.
static int find_stick(void)
{
    for (int j = GLFW_JOYSTICK_1; j <= GLFW_JOYSTICK_LAST; j++) {
        if (!glfwJoystickPresent(j) || glfwJoystickIsGamepad(j)) continue;
        const char *nm = glfwGetJoystickName(j);
        if (nm && (strstr(nm, "Touchpad") || strstr(nm, "Motion") || strstr(nm, "Sensors"))) continue;
        int na = 0, nb = 0;
        glfwGetJoystickAxes(j, &na);
        glfwGetJoystickButtons(j, &nb);
        if (na >= 2 && nb >= 2) return j;
    }
    return -1;
}

// Sony pads (DualSense / DualShock 4) over Bluetooth or USB: the stock mapping
// table only matches exact GUIDs (bus + driver version), which vary between
// kernels and connections. Give any unmapped Sony controller a mapping built
// for its own GUID, with the Linux hid-playstation layout.
static void map_sony_pads(void)
{
    static double next_scan;
    if (GetTime() < next_scan) return;
    next_scan = GetTime() + 1.0;
    for (int j = GLFW_JOYSTICK_1; j <= GLFW_JOYSTICK_LAST; j++) {
        if (!glfwJoystickPresent(j) || glfwJoystickIsGamepad(j)) continue;
        const char *guid = glfwGetJoystickGUID(j), *nm = glfwGetJoystickName(j);
        if (!guid || strlen(guid) != 32) continue;
        if (strncmp(guid + 8, "4c05", 4) != 0) continue;   // vendor 054c = Sony
        if (nm && (strstr(nm, "Touchpad") || strstr(nm, "Motion") || strstr(nm, "Sensors"))) continue;
        int na = 0, nb = 0;
        glfwGetJoystickAxes(j, &na);
        glfwGetJoystickButtons(j, &nb);
        if (na < 6 || nb < 10) continue;
        char m[512];
#ifdef _WIN32
        snprintf(m, sizeof m, "%s,Sony controller,a:b1,b:b2,back:b8,dpdown:h0.4,dpleft:h0.8,dpright:h0.2,dpup:h0.1,guide:b12,"
                 "leftshoulder:b4,leftstick:b10,lefttrigger:a3,leftx:a0,lefty:a1,rightshoulder:b5,rightstick:b11,righttrigger:a4,"
                 "rightx:a2,righty:a5,start:b9,x:b0,y:b3,platform:Windows,", guid);
#else
        snprintf(m, sizeof m, "%s,Sony controller,a:b0,b:b1,back:b8,dpdown:h0.4,dpleft:h0.8,dpright:h0.2,dpup:h0.1,guide:b10,"
                 "leftshoulder:b4,leftstick:b11,lefttrigger:a2,leftx:a0,lefty:a1,rightshoulder:b5,rightstick:b12,righttrigger:a5,"
                 "rightx:a3,righty:a4,start:b9,x:b3,y:b2,platform:Linux,", guid);
#endif
        if (glfwUpdateGamepadMappings(m) == GLFW_TRUE)
            TraceLog(LOG_INFO, "controls: mapped %s (%s) as a gamepad", nm ? nm : "Sony controller", guid);
    }
}

static int find_pad(void)
{
    map_sony_pads();
    // raylib reports every joystick as "available"; only mapped gamepads count.
    for (int i = 0; i < 4; i++)
        if (IsGamepadAvailable(i) && glfwJoystickIsGamepad(GLFW_JOYSTICK_1 + i)) return i;
    return -1;
}

static int prev_buttons[64], prev_hat_up, prev_hat_dn;

void controls_poll(Controls *c)
{
    memset(c, 0, sizeof *c);
    // Keyboard (always).
    c->pitch = (IsKeyDown(KEY_W) ? 1 : 0) - (IsKeyDown(KEY_S) ? 1 : 0);
    c->roll = (IsKeyDown(KEY_D) ? 1 : 0) - (IsKeyDown(KEY_A) ? 1 : 0);
    c->yaw = (IsKeyDown(KEY_Q) ? 1 : 0) - (IsKeyDown(KEY_E) ? 1 : 0);
    c->pitch *= 0.6; c->roll *= 0.6; c->yaw *= 0.6;   // keyboard = gentle fixed deflection
    c->rod = (IsKeyPressed(KEY_UP) ? 1 : 0) - (IsKeyPressed(KEY_DOWN) ? 1 : 0);
    c->stop = IsKeyPressed(KEY_SPACE);

    // Gamepad.
    int g = find_pad();
    if (g >= 0) {
        c->pitch -= dz(GetGamepadAxisMovement(g, GAMEPAD_AXIS_LEFT_Y));
        c->roll += dz(GetGamepadAxisMovement(g, GAMEPAD_AXIS_LEFT_X));
        // Triggers rest at -1 and go to +1.
        double lt = (GetGamepadAxisMovement(g, GAMEPAD_AXIS_LEFT_TRIGGER) + 1) * 0.5;
        double rt = (GetGamepadAxisMovement(g, GAMEPAD_AXIS_RIGHT_TRIGGER) + 1) * 0.5;
        c->yaw += dz(lt) - dz(rt);
        // Right stick + bumpers = TTCA translation.
        double rx = dz(GetGamepadAxisMovement(g, GAMEPAD_AXIS_RIGHT_X)), ry = dz(GetGamepadAxisMovement(g, GAMEPAD_AXIS_RIGHT_Y));
        c->ty = rx > 0.5 ? 1 : rx < -0.5 ? -1 : 0;
        c->tz = ry < -0.5 ? 1 : ry > 0.5 ? -1 : 0;
        c->tx = IsGamepadButtonDown(g, GAMEPAD_BUTTON_RIGHT_TRIGGER_1) ? 1 : IsGamepadButtonDown(g, GAMEPAD_BUTTON_LEFT_TRIGGER_1) ? -1 : 0;
        if (IsGamepadButtonPressed(g, GAMEPAD_BUTTON_LEFT_FACE_UP)) c->rod += 1;
        if (IsGamepadButtonPressed(g, GAMEPAD_BUTTON_LEFT_FACE_DOWN)) c->rod -= 1;
        if (IsGamepadButtonPressed(g, GAMEPAD_BUTTON_RIGHT_FACE_DOWN)) c->stop = 1;          // A / Cross
        if (IsGamepadButtonDown(g, GAMEPAD_BUTTON_RIGHT_FACE_RIGHT)) c->pro = 1;             // B / Circle (hold)
        if (IsGamepadButtonPressed(g, GAMEPAD_BUTTON_RIGHT_FACE_LEFT)) c->enter = 1;         // X / Square
        if (IsGamepadButtonPressed(g, GAMEPAD_BUTTON_RIGHT_FACE_UP)) c->camera = 1;          // Y / Triangle
        if (IsGamepadButtonPressed(g, GAMEPAD_BUTTON_MIDDLE_RIGHT)) c->pause = 1;            // Start
        snprintf(c->device, sizeof c->device, "Gamepad: %s", GetGamepadName(g));
    }

    // Flight stick (raw): X roll, Y pitch, twist yaw, hat = ROD switch.
    int j = find_stick();
    if (j >= 0) {
        int na = 0, nb = 0, nh = 0;
        const float *ax = glfwGetJoystickAxes(j, &na);
        const unsigned char *bt = glfwGetJoystickButtons(j, &nb);
        const unsigned char *hat = glfwGetJoystickHats(j, &nh);
        if (na >= 2) {
            c->roll += dz(ax[0]);
            c->pitch -= dz(ax[1]);
        }
        if (na >= 3) c->yaw -= dz(ax[2]) * 0.8;
        if (nh > 0) {
            int up = (hat[0] & GLFW_HAT_UP) != 0, dn = (hat[0] & GLFW_HAT_DOWN) != 0;
            if (up && !prev_hat_up) c->rod += 1;
            if (dn && !prev_hat_dn) c->rod -= 1;
            prev_hat_up = up;
            prev_hat_dn = dn;
            if (hat[0] & GLFW_HAT_RIGHT) c->ty = 1;
            if (hat[0] & GLFW_HAT_LEFT) c->ty = -1;
        }
        if (nb > 64) nb = 64;
        for (int b = 0; b < nb; b++) {
            int down = bt[b] == GLFW_PRESS, pressed = down && !prev_buttons[b];
            if (b == 0 && down) c->pro = 1;            // trigger: PRO (hold), e.g. "enable ignition" at V99
            if (b == 1 && pressed) c->stop = 1;        // thumb: ENGINE STOP
            if (b == 2 && pressed) c->rod -= 1;
            if (b == 3 && pressed) c->rod += 1;
            if (b == 4 && pressed) c->enter = 1;
            if (b == 5 && pressed) c->camera = 1;
            prev_buttons[b] = down;
        }
        const char *nm = glfwGetJoystickName(j);
        char prev[400];
        snprintf(prev, sizeof prev, "%s", c->device);
        snprintf(c->device, sizeof c->device, "%s%sStick: %s", prev, prev[0] ? "  +  " : "", nm ? nm : "joystick");
    }
    if (c->pitch > 1) c->pitch = 1;
    if (c->pitch < -1) c->pitch = -1;
    if (c->roll > 1) c->roll = 1;
    if (c->roll < -1) c->roll = -1;
    if (c->yaw > 1) c->yaw = 1;
    if (c->yaw < -1) c->yaw = -1;
}
