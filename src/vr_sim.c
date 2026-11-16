// PC stand-in for the headset: see vr.h. Two eyes with a Quest-like field of
// view are rendered into their own framebuffers and shown side by side.
//
//   AGC_VRSIM_SIZE=WxH        eye size in pixels (default 1280x1344)
//   AGC_VRSIM_LOOK=yaw,pitch  head direction, degrees (right-drag also turns it)
//   AGC_VRSIM_HEAD=x,y,z      head position in the tracking space, metres
//   AGC_VRSIM_AIM=yaw,pitch   right-hand pointing direction, degrees (default: follows the mouse)
//   AGC_VRSIM_SRGB=1          sRGB eye targets, as the headset has
#if defined(AGC_VR) && !defined(__ANDROID__)

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <GLES3/gl3.h>

#include "raylib.h"
#include "rlgl.h"

#include "vr.h"

#define DEG (PI_D / 180.0)

static int eye_w = 1280, eye_h = 1344, srgb;
static GLuint fbo[2], tex[2], depth[2];
static VrInput input;
static float frame_dt = 1.0f / 72;
static double yaw, pitch;      // head, radians
static V3 head;

static V3 dir_of(double y, double p) { return v3(sin(y) * cos(p), sin(p), -cos(y) * cos(p)); }

int vr_init(void)
{
    const char *e = getenv("AGC_VRSIM_SIZE");
    if (e) sscanf(e, "%dx%d", &eye_w, &eye_h);
    srgb = getenv("AGC_VRSIM_SRGB") != NULL;
    double a = 0, b = 0;
    if ((e = getenv("AGC_VRSIM_LOOK")) && sscanf(e, "%lf,%lf", &a, &b) == 2) { yaw = a * DEG; pitch = b * DEG; }
    if ((e = getenv("AGC_VRSIM_HEAD"))) sscanf(e, "%lf,%lf,%lf", &head.x, &head.y, &head.z);
    for (int i = 0; i < 2; i++) {
        glGenTextures(1, &tex[i]);
        glBindTexture(GL_TEXTURE_2D, tex[i]);
        glTexImage2D(GL_TEXTURE_2D, 0, srgb ? GL_SRGB8_ALPHA8 : GL_RGBA8, eye_w, eye_h, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glGenRenderbuffers(1, &depth[i]);
        glBindRenderbuffer(GL_RENDERBUFFER, depth[i]);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, eye_w, eye_h);
        glGenFramebuffers(1, &fbo[i]);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo[i]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex[i], 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth[i]);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            TraceLog(LOG_ERROR, "VR stand-in: eye framebuffer %d is incomplete", i);
            return -1;
        }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    TraceLog(LOG_INFO, "VR stand-in: two eyes of %dx%d%s", eye_w, eye_h, srgb ? " (sRGB targets)" : "");
    return 0;
}

void vr_shutdown(void) {}
int vr_srgb_target(void) { return srgb; }
float vr_frame_dt(void) { return frame_dt; }
const VrInput *vr_input(void) { return &input; }
void vr_haptic(int hand, float strength, float seconds) { (void)hand; (void)strength; (void)seconds; }
void vr_stats(float *load, float *scale, float *fps) { *load = 0; *scale = 1; *fps = (float)GetFPS(); }

int vr_begin_frame(float *dt, int *quit)
{
    *quit = 0;
    frame_dt = GetFrameTime();
    if (frame_dt <= 0 || frame_dt > 0.1f) frame_dt = 1.0f / 72;
    *dt = frame_dt;
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        Vector2 d = GetMouseDelta();
        yaw += d.x * 0.004;
        pitch = fmax(-1.5, fmin(1.5, pitch - d.y * 0.004));
    }
    memset(&input, 0, sizeof input);
    input.head_pos = head;
    input.head_fwd = dir_of(yaw, pitch);
    V3 right = v3(cos(yaw), 0, sin(yaw));
    input.head_up = v3_cross(right, input.head_fwd);
    // Right hand: held a little down and to the right, pointing where the mouse is.
    Vector2 m = GetMousePosition();
    double ay = (m.x / (double)GetScreenWidth() - 0.5) * 100 * DEG, ap = (0.5 - m.y / (double)GetScreenHeight()) * 90 * DEG;
    const char *e = getenv("AGC_VRSIM_AIM");
    double a = 0, b = 0;
    if (e && sscanf(e, "%lf,%lf", &a, &b) == 2) { ay = a * DEG; ap = b * DEG; }
    input.aim_ok[1] = 1;
    input.aim_pos[1] = v3_add(head, v3(0.18, -0.22, -0.15));
    input.aim_dir[1] = dir_of(ay, ap);
    input.rtrig = IsMouseButtonDown(MOUSE_BUTTON_LEFT) ? 1.0f : 0.0f;
    // Two controllers held in front (AGC_VRSIM_HANDS=0: bare hands). The left
    // one points straight ahead, the right one where the mouse aims.
    // AGC_VRSIM_PRESS=a,b,x,y,lt,rt,lg,rg,ls,rs: buttons held (for screenshots).
    const char *hands = getenv("AGC_VRSIM_HANDS");
    int have = !hands || atoi(hands) != 0;
    for (int h = 0; h < 2; h++) {
        V3 dir = h ? input.aim_dir[1] : dir_of(-8 * DEG, -20 * DEG);
        V3 z = v3_scale(dir, -1), x = v3_norm(v3_cross(v3(0, 1, 0), z)), y = v3_cross(z, x);
        input.grip_ok[h] = 1;
        input.controller[h] = have;
        input.grip_pos[h] = v3_add(head, v3(h ? 0.17 : -0.17, -0.27, -0.30));
        input.grip_x[h] = x; input.grip_y[h] = y; input.grip_z[h] = z;
    }
    const char *pr = getenv("AGC_VRSIM_PRESS");
    if (pr) {
        if (strstr(pr, "a")) input.a = 1;
        if (strstr(pr, "b")) input.b = 1;
        if (strstr(pr, "x")) input.x = 1;
        if (strstr(pr, "y")) input.y = 1;
        if (strstr(pr, "lt")) input.ltrig = 1;
        if (strstr(pr, "rt")) input.rtrig = 1;
        if (strstr(pr, "lg")) input.lgrip = 1;
        if (strstr(pr, "rg")) input.rgrip = 1;
        if (strstr(pr, "ls")) input.ly = 0.8f;
        if (strstr(pr, "rs")) input.rx = 0.8f;
    }
    return 1;
}

void vr_eye_begin(int eye, VrEye *e)
{
    V3 f = dir_of(yaw, pitch), right = v3(cos(yaw), 0, sin(yaw));
    e->fwd = f;
    e->up = v3_cross(right, f);
    e->pos = v3_add(head, v3_scale(right, eye ? 0.032 : -0.032));
    // Quest 3: about 54 deg outward, 42 deg inward, 44 deg up, 54 deg down.
    double out = tan(54 * DEG), in = tan(42 * DEG);
    e->tan_l = eye ? -in : -out;
    e->tan_r = eye ? out : in;
    e->tan_u = tan(44 * DEG);
    e->tan_d = -tan(54 * DEG);
    e->w = eye_w;
    e->h = eye_h;
    e->fbo = fbo[eye];
}

void vr_eye_end(int eye)
{
    (void)eye;
    rlDrawRenderBatchActive();
    rlDisableFramebuffer();   // back to the window
}

void vr_end_frame(void)
{
    BeginDrawing();
    ClearBackground(BLACK);
    int sw = GetScreenWidth(), sh = GetScreenHeight();
    for (int i = 0; i < 2; i++) {
        Texture2D t = { tex[i], eye_w, eye_h, 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 };
        float k = fminf(sw / 2.0f / eye_w, (float)sh / eye_h);
        DrawTexturePro(t, (Rectangle){ 0, 0, (float)eye_w, -(float)eye_h },
                       (Rectangle){ sw / 2.0f * i + (sw / 2.0f - eye_w * k) / 2, (sh - eye_h * k) / 2, eye_w * k, eye_h * k },
                       (Vector2){ 0, 0 }, 0, WHITE);
    }
    EndDrawing();
}

void vr_sim_save(const char *path)
{
    Image out = GenImageColor(eye_w * 2, eye_h, BLACK);
    unsigned char *px = malloc((size_t)eye_w * eye_h * 4);
    for (int i = 0; i < 2; i++) {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo[i]);
        glReadPixels(0, 0, eye_w, eye_h, GL_RGBA, GL_UNSIGNED_BYTE, px);
        for (int y = 0; y < eye_h; y++) {
            unsigned char *dst = (unsigned char *)out.data + ((size_t)y * eye_w * 2 + (size_t)i * eye_w) * 4;
            memcpy(dst, px + (size_t)(eye_h - 1 - y) * eye_w * 4, (size_t)eye_w * 4);
            for (int x = 0; x < eye_w; x++) dst[x * 4 + 3] = 255;
        }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    free(px);
    ExportImage(out, path);
    UnloadImage(out);
}

// No compositor on the PC: the display is drawn into the scene.
unsigned int vr_ui_begin(int w, int h) { (void)w; (void)h; return 0; }
void vr_ui_end(V3 centre, double width, double height) { (void)centre; (void)width; (void)height; }

#endif
