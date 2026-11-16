// The headset through OpenXR (Meta Quest, Android): see vr.h.
//
// raylib's Android layer owns the activity, the window and the GL context;
// this file adds the OpenXR session on that context. Each eye has its own
// swapchain (sRGB), rendered with 4x multisampling where the driver can
// resolve straight into the swapchain image, and with fixed foveation.
// The eye resolution adapts: if frames are missed it drops a step, and it
// climbs back when the display rate has been held for a while.
#if defined(AGC_VR) && defined(__ANDROID__)

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <EGL/egl.h>
#include <GLES3/gl3.h>
#include <android/log.h>
#include <android_native_app_glue.h>
#include <jni.h>

#define XR_USE_PLATFORM_ANDROID
#define XR_USE_GRAPHICS_API_OPENGL_ES
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include "raylib.h"
#include "rlgl.h"

#include "vr.h"

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, "agclander", __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, "agclander", __VA_ARGS__)

struct android_app *GetAndroidApp(void);   // raylib

#define GL_SRGB8_ALPHA8_FMT 0x8C43
#define GL_RGBA8_FMT 0x8058
#define MAX_IMAGES 8

// --- Instance, session ---------------------------------------------------------------------

static XrInstance instance;
static XrSystemId system_id;
static XrSession session;
static XrSpace local_space, view_space, aim_space[2];
static int session_running, session_focused, want_quit;
static int have_foveation, have_perf, have_thread, have_layer_settings;

// The floating 2D display as its own compositor layer: the runtime samples it
// straight onto the lenses, so text stays sharp whatever the eye resolution.
typedef struct {
    XrSwapchain sc;
    int w, h, n;
    GLuint fbo[MAX_IMAGES];
    uint32_t index;
    int acquired, submit;
    XrPosef pose;
    XrExtent2Df size;
} UiLayer;
static UiLayer ui;

// GPU time per frame (GL_EXT_disjoint_timer_query), for the log.
#define NQUERY 6
static GLuint gpu_q[NQUERY];
static int gpu_q_used[NQUERY], gpu_q_next, have_timer;
static double gpu_ms_sum;
static int gpu_ms_n;
static float gpu_ms;
static int fix_head;   // AGC_VR_FIXHEAD: look straight ahead whatever the headset does (benchmarks)

static EGLDisplay egl_display;
static EGLContext egl_context;
static EGLConfig egl_config;
static EGLSurface egl_pbuffer = EGL_NO_SURFACE;

// Extension functions (not exported by the loader: fetched by name).
static XrResult (*pGetGLESRequirements)(XrInstance, XrSystemId, XrGraphicsRequirementsOpenGLESKHR *);
static XrResult (*pCreateFoveationProfile)(XrSession, const XrFoveationProfileCreateInfoFB *, XrFoveationProfileFB *);
static XrResult (*pDestroyFoveationProfile)(XrFoveationProfileFB);
static XrResult (*pUpdateSwapchain)(XrSwapchain, const XrSwapchainStateBaseHeaderFB *);
static XrResult (*pPerfSetLevel)(XrSession, XrPerfSettingsDomainEXT, XrPerfSettingsLevelEXT);
static XrResult (*pSetThread)(XrSession, XrAndroidThreadTypeKHR, uint32_t);

// GL extension: multisampled rendering that resolves into an ordinary texture.
static void (*pRenderbufferStorageMS)(GLenum, GLsizei, GLenum, GLsizei, GLsizei);
static void (*pFramebufferTexture2DMS)(GLenum, GLenum, GLenum, GLuint, GLint, GLsizei);

// --- Eyes -----------------------------------------------------------------------------------

typedef struct {
    XrSwapchain sc;
    int w, h, n;
    GLuint fbo[MAX_IMAGES], depth[MAX_IMAGES];
    uint32_t index;
    int acquired;
} EyeTarget;

static EyeTarget eye[2];
static int rec_w, rec_h;              // the runtime's recommended eye size
static int64_t color_format;
static int srgb;
static float res_scale = 0.8f;        // fraction of the recommended size in use
static XrView views[2];
static XrFrameState frame_state;
static int frame_begun;

static VrInput input;
static float frame_dt = 1.0f / 72;

// --- Actions --------------------------------------------------------------------------------

static XrActionSet action_set;
static XrAction act_stick, act_trigger, act_grip, act_a, act_b, act_x, act_y, act_menu, act_click, act_aim, act_haptic;
static XrPath hand_path[2];

static int ok(XrResult r, const char *what)
{
    if (XR_SUCCEEDED(r)) return 1;
    LOGE("OpenXR: %s failed (%d)", what, (int)r);
    return 0;
}

static void *xr_fn(const char *name)
{
    PFN_xrVoidFunction f = NULL;
    if (XR_FAILED(xrGetInstanceProcAddr(instance, name, &f))) return NULL;
    return (void *)f;
}

static int has_gl_ext(const char *name)
{
    GLint n = 0;
    glGetIntegerv(GL_NUM_EXTENSIONS, &n);
    for (GLint i = 0; i < n; i++) {
        const char *e = (const char *)glGetStringi(GL_EXTENSIONS, (GLuint)i);
        if (e && !strcmp(e, name)) return 1;
    }
    return 0;
}

static void destroy_eye(EyeTarget *t)
{
    for (int i = 0; i < t->n; i++) {
        if (t->fbo[i]) glDeleteFramebuffers(1, &t->fbo[i]);
        if (t->depth[i]) glDeleteRenderbuffers(1, &t->depth[i]);
    }
    if (t->sc) xrDestroySwapchain(t->sc);
    memset(t, 0, sizeof *t);
}

static int create_eye(EyeTarget *t, int w, int h)
{
    memset(t, 0, sizeof *t);
    XrSwapchainCreateInfoFoveationFB fov_ci = { XR_TYPE_SWAPCHAIN_CREATE_INFO_FOVEATION_FB };
    fov_ci.flags = XR_SWAPCHAIN_CREATE_FOVEATION_SCALED_BIN_BIT_FB;
    XrSwapchainCreateInfo ci = { XR_TYPE_SWAPCHAIN_CREATE_INFO };
    if (have_foveation) ci.next = &fov_ci;
    ci.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
    ci.format = color_format;
    ci.sampleCount = 1;
    ci.width = (uint32_t)w;
    ci.height = (uint32_t)h;
    ci.faceCount = 1;
    ci.arraySize = 1;
    ci.mipCount = 1;
    if (!ok(xrCreateSwapchain(session, &ci, &t->sc), "xrCreateSwapchain")) return 0;
    t->w = w;
    t->h = h;
    uint32_t n = 0;
    xrEnumerateSwapchainImages(t->sc, 0, &n, NULL);
    if (n > MAX_IMAGES) n = MAX_IMAGES;
    XrSwapchainImageOpenGLESKHR img[MAX_IMAGES];
    for (uint32_t i = 0; i < n; i++) { img[i].type = XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_ES_KHR; img[i].next = NULL; }
    if (!ok(xrEnumerateSwapchainImages(t->sc, n, &n, (XrSwapchainImageBaseHeader *)img), "xrEnumerateSwapchainImages")) return 0;
    t->n = (int)n;
    int ms = pRenderbufferStorageMS && pFramebufferTexture2DMS;
    for (uint32_t i = 0; i < n; i++) {
        glGenRenderbuffers(1, &t->depth[i]);
        glBindRenderbuffer(GL_RENDERBUFFER, t->depth[i]);
        if (ms) pRenderbufferStorageMS(GL_RENDERBUFFER, 4, GL_DEPTH_COMPONENT24, w, h);
        else glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, w, h);
        glGenFramebuffers(1, &t->fbo[i]);
        glBindFramebuffer(GL_FRAMEBUFFER, t->fbo[i]);
        if (ms) pFramebufferTexture2DMS(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, img[i].image, 0, 4);
        else glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, img[i].image, 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, t->depth[i]);
        GLenum st = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (st != GL_FRAMEBUFFER_COMPLETE) {
            LOGE("VR: eye framebuffer incomplete (0x%x)%s", st, ms ? ", retrying without multisampling" : "");
            if (!ms) return 0;
            // Fall back to plain rendering for every eye target.
            pRenderbufferStorageMS = NULL;
            pFramebufferTexture2DMS = NULL;
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            EyeTarget keep = *t;
            destroy_eye(&keep);
            return create_eye(t, w, h);
        }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    if (have_foveation && pCreateFoveationProfile && pUpdateSwapchain) {
        // Fixed foveation: full detail in the middle of the lens, less toward the edge.
        XrFoveationLevelProfileCreateInfoFB level = { XR_TYPE_FOVEATION_LEVEL_PROFILE_CREATE_INFO_FB };
        level.level = XR_FOVEATION_LEVEL_HIGH_FB;
        level.verticalOffset = 0;
        level.dynamic = XR_FOVEATION_DYNAMIC_DISABLED_FB;
        XrFoveationProfileCreateInfoFB pci = { XR_TYPE_FOVEATION_PROFILE_CREATE_INFO_FB };
        pci.next = &level;
        XrFoveationProfileFB profile = XR_NULL_HANDLE;
        if (XR_SUCCEEDED(pCreateFoveationProfile(session, &pci, &profile))) {
            XrSwapchainStateFoveationFB st = { XR_TYPE_SWAPCHAIN_STATE_FOVEATION_FB };
            st.profile = profile;
            if (XR_FAILED(pUpdateSwapchain(t->sc, (const XrSwapchainStateBaseHeaderFB *)&st))) LOGE("VR: foveation could not be applied");
            if (pDestroyFoveationProfile) pDestroyFoveationProfile(profile);
        }
    }
    return 1;
}

static int create_eyes(void)
{
    int w = ((int)(rec_w * res_scale) + 7) & ~7, h = ((int)(rec_h * res_scale) + 7) & ~7;
    for (int i = 0; i < 2; i++)
        if (!create_eye(&eye[i], w, h)) return 0;
    LOGI("VR: eye targets %dx%d (%.0f%% of %dx%d), %d images, %s, %s", w, h, res_scale * 100, rec_w, rec_h, eye[0].n,
         pFramebufferTexture2DMS ? "4x multisampled" : "no multisampling", have_foveation ? "foveated" : "no foveation");
    return 1;
}

// --- Actions --------------------------------------------------------------------------------

static XrAction make_action(const char *name, const char *label, XrActionType type, int both_hands)
{
    XrActionCreateInfo ci = { XR_TYPE_ACTION_CREATE_INFO };
    ci.actionType = type;
    snprintf(ci.actionName, sizeof ci.actionName, "%s", name);
    snprintf(ci.localizedActionName, sizeof ci.localizedActionName, "%s", label);
    if (both_hands) { ci.countSubactionPaths = 2; ci.subactionPaths = hand_path; }
    XrAction a = XR_NULL_HANDLE;
    ok(xrCreateAction(action_set, &ci, &a), name);
    return a;
}

static XrPath path(const char *s)
{
    XrPath p = XR_NULL_PATH;
    xrStringToPath(instance, s, &p);
    return p;
}

static int init_actions(void)
{
    hand_path[0] = path("/user/hand/left");
    hand_path[1] = path("/user/hand/right");
    XrActionSetCreateInfo si = { XR_TYPE_ACTION_SET_CREATE_INFO };
    snprintf(si.actionSetName, sizeof si.actionSetName, "flight");
    snprintf(si.localizedActionSetName, sizeof si.localizedActionSetName, "Flight");
    if (!ok(xrCreateActionSet(instance, &si, &action_set), "xrCreateActionSet")) return 0;
    act_stick = make_action("stick", "Thumbstick", XR_ACTION_TYPE_VECTOR2F_INPUT, 1);
    act_trigger = make_action("trigger", "Trigger", XR_ACTION_TYPE_FLOAT_INPUT, 1);
    act_grip = make_action("grip", "Grip", XR_ACTION_TYPE_FLOAT_INPUT, 1);
    act_click = make_action("stick_click", "Thumbstick click", XR_ACTION_TYPE_BOOLEAN_INPUT, 1);
    act_aim = make_action("aim", "Pointing", XR_ACTION_TYPE_POSE_INPUT, 1);
    act_haptic = make_action("haptic", "Vibration", XR_ACTION_TYPE_VIBRATION_OUTPUT, 1);
    act_a = make_action("a", "A", XR_ACTION_TYPE_BOOLEAN_INPUT, 0);
    act_b = make_action("b", "B", XR_ACTION_TYPE_BOOLEAN_INPUT, 0);
    act_x = make_action("x", "X", XR_ACTION_TYPE_BOOLEAN_INPUT, 0);
    act_y = make_action("y", "Y", XR_ACTION_TYPE_BOOLEAN_INPUT, 0);
    act_menu = make_action("menu", "Menu", XR_ACTION_TYPE_BOOLEAN_INPUT, 0);

    XrActionSuggestedBinding b[] = {
        { act_stick, path("/user/hand/left/input/thumbstick") },
        { act_stick, path("/user/hand/right/input/thumbstick") },
        { act_trigger, path("/user/hand/left/input/trigger/value") },
        { act_trigger, path("/user/hand/right/input/trigger/value") },
        { act_grip, path("/user/hand/left/input/squeeze/value") },
        { act_grip, path("/user/hand/right/input/squeeze/value") },
        { act_click, path("/user/hand/left/input/thumbstick/click") },
        { act_click, path("/user/hand/right/input/thumbstick/click") },
        { act_aim, path("/user/hand/left/input/aim/pose") },
        { act_aim, path("/user/hand/right/input/aim/pose") },
        { act_haptic, path("/user/hand/left/output/haptic") },
        { act_haptic, path("/user/hand/right/output/haptic") },
        { act_a, path("/user/hand/right/input/a/click") },
        { act_b, path("/user/hand/right/input/b/click") },
        { act_x, path("/user/hand/left/input/x/click") },
        { act_y, path("/user/hand/left/input/y/click") },
        { act_menu, path("/user/hand/left/input/menu/click") },
    };
    XrInteractionProfileSuggestedBinding sb = { XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
    sb.interactionProfile = path("/interaction_profiles/oculus/touch_controller");
    sb.suggestedBindings = b;
    sb.countSuggestedBindings = sizeof b / sizeof b[0];
    if (!ok(xrSuggestInteractionProfileBindings(instance, &sb), "xrSuggestInteractionProfileBindings")) return 0;

    // Bare hands (no controllers): the runtime offers them as a simple
    // controller - a pinch is "select", the pointing ray is "aim", and the
    // left hand's palm-up pinch is the menu gesture.
    XrActionSuggestedBinding hb[] = {
        { act_trigger, path("/user/hand/left/input/select/click") },
        { act_trigger, path("/user/hand/right/input/select/click") },
        { act_aim, path("/user/hand/left/input/aim/pose") },
        { act_aim, path("/user/hand/right/input/aim/pose") },
        { act_menu, path("/user/hand/left/input/menu/click") },
        { act_haptic, path("/user/hand/left/output/haptic") },
        { act_haptic, path("/user/hand/right/output/haptic") },
    };
    XrInteractionProfileSuggestedBinding hsb = { XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
    hsb.interactionProfile = path("/interaction_profiles/khr/simple_controller");
    hsb.suggestedBindings = hb;
    hsb.countSuggestedBindings = sizeof hb / sizeof hb[0];
    if (XR_FAILED(xrSuggestInteractionProfileBindings(instance, &hsb))) LOGI("VR: no simple-controller (hands) bindings");

    XrSessionActionSetsAttachInfo ai = { XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO };
    ai.countActionSets = 1;
    ai.actionSets = &action_set;
    if (!ok(xrAttachSessionActionSets(session, &ai), "xrAttachSessionActionSets")) return 0;
    for (int h = 0; h < 2; h++) {
        XrActionSpaceCreateInfo ci = { XR_TYPE_ACTION_SPACE_CREATE_INFO };
        ci.action = act_aim;
        ci.subactionPath = hand_path[h];
        ci.poseInActionSpace.orientation.w = 1;
        ok(xrCreateActionSpace(session, &ci, &aim_space[h]), "xrCreateActionSpace");
    }
    return 1;
}

static float get_float(XrAction a, int hand)
{
    XrActionStateGetInfo gi = { XR_TYPE_ACTION_STATE_GET_INFO };
    gi.action = a;
    gi.subactionPath = hand_path[hand];
    XrActionStateFloat st = { XR_TYPE_ACTION_STATE_FLOAT };
    if (XR_FAILED(xrGetActionStateFloat(session, &gi, &st)) || !st.isActive) return 0;
    return st.currentState;
}

static int get_bool(XrAction a, int hand)
{
    XrActionStateGetInfo gi = { XR_TYPE_ACTION_STATE_GET_INFO };
    gi.action = a;
    if (hand >= 0) gi.subactionPath = hand_path[hand];
    XrActionStateBoolean st = { XR_TYPE_ACTION_STATE_BOOLEAN };
    if (XR_FAILED(xrGetActionStateBoolean(session, &gi, &st)) || !st.isActive) return 0;
    return st.currentState ? 1 : 0;
}

static void get_stick(int hand, float *x, float *y)
{
    XrActionStateGetInfo gi = { XR_TYPE_ACTION_STATE_GET_INFO };
    gi.action = act_stick;
    gi.subactionPath = hand_path[hand];
    XrActionStateVector2f st = { XR_TYPE_ACTION_STATE_VECTOR2F };
    *x = *y = 0;
    if (XR_FAILED(xrGetActionStateVector2f(session, &gi, &st)) || !st.isActive) return;
    *x = st.currentState.x;
    *y = st.currentState.y;
}

// Rotate a vector by a quaternion.
static V3 qrot(XrQuaternionf q, V3 v)
{
    V3 u = v3(q.x, q.y, q.z);
    V3 t = v3_scale(v3_cross(u, v), 2.0);
    return v3_add(v3_add(v, v3_scale(t, q.w)), v3_cross(u, t));
}

// --- Start-up -------------------------------------------------------------------------------

static int has_ext(const XrExtensionProperties *list, uint32_t n, const char *name)
{
    for (uint32_t i = 0; i < n; i++)
        if (!strcmp(list[i].extensionName, name)) return 1;
    return 0;
}

int vr_init(void)
{
    struct android_app *app = GetAndroidApp();

    // The loader must be told about the Java VM before anything else.
    PFN_xrVoidFunction init_loader = NULL;
    if (XR_FAILED(xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrInitializeLoaderKHR", &init_loader)) || !init_loader) {
        LOGE("OpenXR: no loader initialisation function");
        return -1;
    }
    XrLoaderInitInfoAndroidKHR li = { XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR };
    li.applicationVM = app->activity->vm;
    li.applicationContext = app->activity->clazz;
    if (!ok(((XrResult (*)(const XrLoaderInitInfoBaseHeaderKHR *))init_loader)((const XrLoaderInitInfoBaseHeaderKHR *)&li), "xrInitializeLoaderKHR")) return -1;

    uint32_t next = 0;
    xrEnumerateInstanceExtensionProperties(NULL, 0, &next, NULL);
    XrExtensionProperties *exts = calloc(next ? next : 1, sizeof *exts);
    for (uint32_t i = 0; i < next; i++) exts[i].type = XR_TYPE_EXTENSION_PROPERTIES;
    xrEnumerateInstanceExtensionProperties(NULL, next, &next, exts);
    const char *want[12];
    uint32_t nwant = 0;
    if (!has_ext(exts, next, XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME) || !has_ext(exts, next, XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME)) {
        LOGE("OpenXR: the runtime lacks Android / OpenGL ES support");
        free(exts);
        return -1;
    }
    want[nwant++] = XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME;
    want[nwant++] = XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME;
    have_foveation = has_ext(exts, next, XR_FB_FOVEATION_EXTENSION_NAME) && has_ext(exts, next, XR_FB_FOVEATION_CONFIGURATION_EXTENSION_NAME) &&
                     has_ext(exts, next, XR_FB_SWAPCHAIN_UPDATE_STATE_EXTENSION_NAME);
    if (have_foveation) {
        want[nwant++] = XR_FB_FOVEATION_EXTENSION_NAME;
        want[nwant++] = XR_FB_FOVEATION_CONFIGURATION_EXTENSION_NAME;
        want[nwant++] = XR_FB_SWAPCHAIN_UPDATE_STATE_EXTENSION_NAME;
    }
    have_perf = has_ext(exts, next, XR_EXT_PERFORMANCE_SETTINGS_EXTENSION_NAME);
    if (have_perf) want[nwant++] = XR_EXT_PERFORMANCE_SETTINGS_EXTENSION_NAME;
    have_thread = has_ext(exts, next, XR_KHR_ANDROID_THREAD_SETTINGS_EXTENSION_NAME);
    if (have_thread) want[nwant++] = XR_KHR_ANDROID_THREAD_SETTINGS_EXTENSION_NAME;
    have_layer_settings = has_ext(exts, next, XR_FB_COMPOSITION_LAYER_SETTINGS_EXTENSION_NAME);
    if (have_layer_settings) want[nwant++] = XR_FB_COMPOSITION_LAYER_SETTINGS_EXTENSION_NAME;
    free(exts);

    XrInstanceCreateInfoAndroidKHR ai = { XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR };
    ai.applicationVM = app->activity->vm;
    ai.applicationActivity = app->activity->clazz;
    XrInstanceCreateInfo ici = { XR_TYPE_INSTANCE_CREATE_INFO };
    ici.next = &ai;
    snprintf(ici.applicationInfo.applicationName, sizeof ici.applicationInfo.applicationName, "AGC Lander");
    snprintf(ici.applicationInfo.engineName, sizeof ici.applicationInfo.engineName, "agc-lander");
    ici.applicationInfo.applicationVersion = 1;
    ici.applicationInfo.apiVersion = XR_API_VERSION_1_0;
    ici.enabledExtensionCount = nwant;
    ici.enabledExtensionNames = want;
    if (!ok(xrCreateInstance(&ici, &instance), "xrCreateInstance")) return -1;

    XrSystemGetInfo sgi = { XR_TYPE_SYSTEM_GET_INFO };
    sgi.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    if (!ok(xrGetSystem(instance, &sgi, &system_id), "xrGetSystem")) return -1;

    pGetGLESRequirements = xr_fn("xrGetOpenGLESGraphicsRequirementsKHR");
    if (pGetGLESRequirements) {
        XrGraphicsRequirementsOpenGLESKHR req = { XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_ES_KHR };
        pGetGLESRequirements(instance, system_id, &req);
    }
    if (have_foveation) {
        pCreateFoveationProfile = xr_fn("xrCreateFoveationProfileFB");
        pDestroyFoveationProfile = xr_fn("xrDestroyFoveationProfileFB");
        pUpdateSwapchain = xr_fn("xrUpdateSwapchainFB");
    }
    if (have_perf) pPerfSetLevel = xr_fn("xrPerfSettingsSetPerformanceLevelEXT");
    if (have_thread) pSetThread = xr_fn("xrSetAndroidApplicationThreadKHR");

    // The session shares raylib's GL context.
    egl_display = eglGetCurrentDisplay();
    egl_context = eglGetCurrentContext();
    if (egl_context == EGL_NO_CONTEXT) { LOGE("VR: no GL context"); return -1; }
    EGLint cfg_id = 0, ncfg = 0;
    eglQueryContext(egl_display, egl_context, EGL_CONFIG_ID, &cfg_id);
    EGLint attr[] = { EGL_CONFIG_ID, cfg_id, EGL_NONE };
    if (!eglChooseConfig(egl_display, attr, &egl_config, 1, &ncfg) || ncfg < 1) { LOGE("VR: GL config not found"); return -1; }
    // A tiny off-screen surface keeps the context usable when Android takes the window away.
    EGLint pb[] = { EGL_WIDTH, 16, EGL_HEIGHT, 16, EGL_NONE };
    egl_pbuffer = eglCreatePbufferSurface(egl_display, egl_config, pb);
    if (egl_pbuffer == EGL_NO_SURFACE) { eglGetError(); LOGI("VR: no off-screen surface for this GL config (drawing pauses with the window)"); }

    XrGraphicsBindingOpenGLESAndroidKHR gb = { XR_TYPE_GRAPHICS_BINDING_OPENGL_ES_ANDROID_KHR };
    gb.display = egl_display;
    gb.config = egl_config;
    gb.context = egl_context;
    XrSessionCreateInfo sci = { XR_TYPE_SESSION_CREATE_INFO };
    sci.next = &gb;
    sci.systemId = system_id;
    if (!ok(xrCreateSession(instance, &sci, &session), "xrCreateSession")) return -1;

    XrReferenceSpaceCreateInfo rci = { XR_TYPE_REFERENCE_SPACE_CREATE_INFO };
    rci.poseInReferenceSpace.orientation.w = 1;
    rci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    if (!ok(xrCreateReferenceSpace(session, &rci, &local_space), "local space")) return -1;
    rci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    if (!ok(xrCreateReferenceSpace(session, &rci, &view_space), "view space")) return -1;

    uint32_t nv = 0;
    XrViewConfigurationView vcv[2] = { { XR_TYPE_VIEW_CONFIGURATION_VIEW }, { XR_TYPE_VIEW_CONFIGURATION_VIEW } };
    if (!ok(xrEnumerateViewConfigurationViews(instance, system_id, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 2, &nv, vcv), "view configuration") || nv != 2)
        return -1;
    rec_w = (int)vcv[0].recommendedImageRectWidth;
    rec_h = (int)vcv[0].recommendedImageRectHeight;

    uint32_t nf = 0;
    xrEnumerateSwapchainFormats(session, 0, &nf, NULL);
    int64_t *fmts = calloc(nf ? nf : 1, sizeof *fmts);
    xrEnumerateSwapchainFormats(session, nf, &nf, fmts);
    color_format = 0;
    for (uint32_t i = 0; i < nf && !color_format; i++)
        if (fmts[i] == GL_SRGB8_ALPHA8_FMT) color_format = fmts[i];
    for (uint32_t i = 0; i < nf && !color_format; i++)
        if (fmts[i] == GL_RGBA8_FMT) color_format = fmts[i];
    free(fmts);
    if (!color_format) { LOGE("VR: no usable swapchain format"); return -1; }
    srgb = color_format == GL_SRGB8_ALPHA8_FMT;

    if (has_gl_ext("GL_EXT_multisampled_render_to_texture")) {
        pRenderbufferStorageMS = (void (*)(GLenum, GLsizei, GLenum, GLsizei, GLsizei))eglGetProcAddress("glRenderbufferStorageMultisampleEXT");
        pFramebufferTexture2DMS = (void (*)(GLenum, GLenum, GLenum, GLuint, GLint, GLsizei))eglGetProcAddress("glFramebufferTexture2DMultisampleEXT");
    }
    const char *e = getenv("AGC_VR_SCALE");
    if (e) res_scale = (float)atof(e);
    if (res_scale < 0.5f) res_scale = 0.5f;
    if (res_scale > 1.2f) res_scale = 1.2f;
    if (!create_eyes()) return -1;
    if (!init_actions()) return -1;
    fix_head = getenv("AGC_VR_FIXHEAD") != NULL;
    if (has_gl_ext("GL_EXT_disjoint_timer_query")) {
        glGenQueries(NQUERY, gpu_q);
        have_timer = 1;
    }
    for (int i = 0; i < 2; i++) { views[i].type = XR_TYPE_VIEW; views[i].pose.orientation.w = 1; }
    LOGI("VR: OpenXR ready (%s colour, GL %s)", srgb ? "sRGB" : "linear 8-bit", (const char *)glGetString(GL_VERSION));
    return 0;
}

void vr_shutdown(void)
{
    for (int i = 0; i < 2; i++) destroy_eye(&eye[i]);
    if (session) xrDestroySession(session);
    if (instance) xrDestroyInstance(instance);
    session = XR_NULL_HANDLE;
    instance = XR_NULL_HANDLE;
}

int vr_srgb_target(void) { return srgb; }
float vr_frame_dt(void) { return frame_dt; }
const VrInput *vr_input(void) { return &input; }

void vr_haptic(int hand, float strength, float seconds)
{
    if (!session_running || hand < 0 || hand > 1) return;
    XrHapticVibration v = { XR_TYPE_HAPTIC_VIBRATION };
    v.amplitude = strength;
    v.duration = (XrDuration)(seconds * 1e9);
    v.frequency = XR_FREQUENCY_UNSPECIFIED;
    XrHapticActionInfo hi = { XR_TYPE_HAPTIC_ACTION_INFO };
    hi.action = act_haptic;
    hi.subactionPath = hand_path[hand];
    xrApplyHapticFeedback(session, &hi, (const XrHapticBaseHeader *)&v);
}

// --- Frame ----------------------------------------------------------------------------------

static void poll_events(void)
{
    XrEventDataBuffer ev = { XR_TYPE_EVENT_DATA_BUFFER };
    while (xrPollEvent(instance, &ev) == XR_SUCCESS) {
        if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
            const XrEventDataSessionStateChanged *sc = (const XrEventDataSessionStateChanged *)&ev;
            LOGI("VR: session state %d", (int)sc->state);
            switch (sc->state) {
            case XR_SESSION_STATE_READY: {
                XrSessionBeginInfo bi = { XR_TYPE_SESSION_BEGIN_INFO };
                bi.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
                if (ok(xrBeginSession(session, &bi), "xrBeginSession")) {
                    session_running = 1;
                    // Ask for the fast cores and steady high clock speeds.
                    if (pPerfSetLevel) {
                        pPerfSetLevel(session, XR_PERF_SETTINGS_DOMAIN_CPU_EXT, XR_PERF_SETTINGS_LEVEL_SUSTAINED_HIGH_EXT);
                        pPerfSetLevel(session, XR_PERF_SETTINGS_DOMAIN_GPU_EXT, XR_PERF_SETTINGS_LEVEL_SUSTAINED_HIGH_EXT);
                    }
                    if (pSetThread) pSetThread(session, XR_ANDROID_THREAD_TYPE_RENDERER_MAIN_KHR, (uint32_t)gettid());
                }
                break;
            }
            case XR_SESSION_STATE_FOCUSED: session_focused = 1; break;
            case XR_SESSION_STATE_VISIBLE: session_focused = 0; break;
            case XR_SESSION_STATE_STOPPING:
                session_focused = 0;
                session_running = 0;
                xrEndSession(session);
                break;
            case XR_SESSION_STATE_EXITING:
            case XR_SESSION_STATE_LOSS_PENDING:
                want_quit = 1;
                break;
            default: break;
            }
        } else if (ev.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) {
            want_quit = 1;
        }
        ev.type = XR_TYPE_EVENT_DATA_BUFFER;
        ev.next = NULL;
    }
}

// Frame statistics and the resolution controller.
static float stat_fps, stat_load = 1;
static double stat_t0;
static int stat_frames, stat_missed;
static double hold_until;      // no resolution change before this time
static float scale_ceiling = 1.0f;
static double ceiling_until;
static int rebuild_scale;      // new eye size wanted at the next frame start

static void frame_stats(double period)
{
    static XrTime last_display;
    double interval = last_display ? (double)(frame_state.predictedDisplayTime - last_display) * 1e-9 : period;
    last_display = frame_state.predictedDisplayTime;
    if (interval <= 0 || interval > 0.25) interval = period;
    frame_dt = (float)interval;
    stat_frames++;
    if (interval > period * 1.5) stat_missed++;
    double now = GetTime();
    if (stat_t0 == 0) stat_t0 = now;
    if (now - stat_t0 < 1.0) return;
    stat_fps = (float)(stat_frames / (now - stat_t0));
    float missed = (float)stat_missed / (float)stat_frames;
    stat_load = (float)((1.0 / period) / (stat_fps > 1 ? stat_fps : 1));
    static int log_n;
    if ((log_n++ % 3) == 0)
        LOGI("VR: %.1f fps (display %.0f Hz), %.0f%% of frames late, eyes %dx%d (%.0f%%), GPU %.1f ms", stat_fps, 1.0 / period, missed * 100,
             eye[0].w, eye[0].h, res_scale * 100, gpu_ms);
    if (!getenv("AGC_VR_SCALE") && now > hold_until) {
        if (missed > 0.10f && res_scale > 0.55f) {
            // Too slow: one step down, and do not try this size again for a while.
            scale_ceiling = res_scale - 0.1f;
            ceiling_until = now + 45;
            res_scale -= 0.1f;
            rebuild_scale = 1;
            hold_until = now + 3;
        } else if (missed < 0.01f && res_scale < 0.999f) {
            float limit = now < ceiling_until ? scale_ceiling : 1.0f;
            if (res_scale + 0.1f <= limit + 0.001f) {
                res_scale += 0.1f;
                rebuild_scale = 1;
                hold_until = now + 6;
            }
        }
    }
    stat_t0 = now;
    stat_frames = stat_missed = 0;
    if (gpu_ms_n) { gpu_ms = (float)(gpu_ms_sum / gpu_ms_n); gpu_ms_sum = 0; gpu_ms_n = 0; }
}

#define GL_TIME_ELAPSED_EXT_ 0x88BF
#define GL_GPU_DISJOINT_EXT_ 0x8FBB
static void gpu_timer_collect(void)
{
    if (!have_timer) return;
    for (int i = 0; i < NQUERY; i++) {
        if (!gpu_q_used[i]) continue;
        GLuint avail = 0;
        glGetQueryObjectuiv(gpu_q[i], GL_QUERY_RESULT_AVAILABLE, &avail);
        if (!avail) continue;
        GLuint ns = 0;
        glGetQueryObjectuiv(gpu_q[i], GL_QUERY_RESULT, &ns);
        GLint disjoint = 0;
        glGetIntegerv(GL_GPU_DISJOINT_EXT_, &disjoint);
        if (!disjoint) { gpu_ms_sum += ns * 1e-6; gpu_ms_n++; }
        gpu_q_used[i] = 0;
    }
}

void vr_stats(float *load, float *scale, float *fps) { *load = stat_load; *scale = res_scale; *fps = stat_fps; }

int vr_begin_frame(float *dt, int *quit)
{
    *quit = 0;
    *dt = 0;
    PollInputEvents();   // raylib: Android activity events
    poll_events();
    struct android_app *app = GetAndroidApp();
    if (want_quit || app->destroyRequested || WindowShouldClose()) { *quit = 1; return 0; }
    if (!session_running) { usleep(20000); return 0; }

    // Android may have taken the window (and with it the current context) away.
    if (eglGetCurrentContext() == EGL_NO_CONTEXT && egl_pbuffer != EGL_NO_SURFACE)
        eglMakeCurrent(egl_display, egl_pbuffer, egl_pbuffer, egl_context);
    int can_draw = eglGetCurrentContext() != EGL_NO_CONTEXT;

    if (rebuild_scale && can_draw) {
        rebuild_scale = 0;
        for (int i = 0; i < 2; i++) destroy_eye(&eye[i]);
        if (!create_eyes()) { want_quit = 1; *quit = 1; return 0; }
    }

    frame_state = (XrFrameState){ XR_TYPE_FRAME_STATE };
    if (XR_FAILED(xrWaitFrame(session, NULL, &frame_state))) return 0;
    if (XR_FAILED(xrBeginFrame(session, NULL))) return 0;
    frame_begun = 1;
    double period = frame_state.predictedDisplayPeriod > 0 ? (double)frame_state.predictedDisplayPeriod * 1e-9 : 1.0 / 72;

    XrViewLocateInfo vli = { XR_TYPE_VIEW_LOCATE_INFO };
    vli.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    vli.displayTime = frame_state.predictedDisplayTime;
    vli.space = local_space;
    XrViewState vs = { XR_TYPE_VIEW_STATE };
    uint32_t nv = 0;
    XrView located[2] = { { XR_TYPE_VIEW }, { XR_TYPE_VIEW } };
    int tracked = XR_SUCCEEDED(xrLocateViews(session, &vli, &vs, 2, &nv, located)) && nv == 2 &&
                  (vs.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) && (vs.viewStateFlags & XR_VIEW_STATE_POSITION_VALID_BIT);
    if (tracked) { views[0] = located[0]; views[1] = located[1]; }
    if (fix_head) {
        for (int i = 0; i < 2; i++) {
            views[i].pose.orientation = (XrQuaternionf){ 0, 0, 0, 1 };
            views[i].pose.position = (XrVector3f){ i ? 0.032f : -0.032f, 0, 0 };
            if (views[i].fov.angleRight - views[i].fov.angleLeft < 0.1f)
                views[i].fov = (XrFovf){ -0.9f, 0.9f, 0.9f, -0.9f };
        }
    }

    if (!frame_state.shouldRender || !can_draw) {
        // Nothing to show this frame (headset off the face, or no GL context).
        XrFrameEndInfo ei = { XR_TYPE_FRAME_END_INFO };
        ei.displayTime = frame_state.predictedDisplayTime;
        ei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        xrEndFrame(session, &ei);
        frame_begun = 0;
        return 0;
    }
    frame_stats(period);
    *dt = frame_dt;

    // Controllers.
    memset(&input, 0, sizeof input);
    XrActiveActionSet as = { action_set, XR_NULL_PATH };
    XrActionsSyncInfo si = { XR_TYPE_ACTIONS_SYNC_INFO };
    si.countActiveActionSets = 1;
    si.activeActionSets = &as;
    if (session_focused && xrSyncActions(session, &si) == XR_SUCCESS) {
        get_stick(0, &input.lx, &input.ly);
        get_stick(1, &input.rx, &input.ry);
        input.ltrig = get_float(act_trigger, 0);
        input.rtrig = get_float(act_trigger, 1);
        input.lgrip = get_float(act_grip, 0);
        input.rgrip = get_float(act_grip, 1);
        input.lclick = get_bool(act_click, 0);
        input.rclick = get_bool(act_click, 1);
        input.a = get_bool(act_a, -1);
        input.b = get_bool(act_b, -1);
        input.x = get_bool(act_x, -1);
        input.y = get_bool(act_y, -1);
        input.menu = get_bool(act_menu, -1);
        for (int h = 0; h < 2; h++) {
            XrSpaceLocation loc = { XR_TYPE_SPACE_LOCATION };
            if (XR_SUCCEEDED(xrLocateSpace(aim_space[h], local_space, frame_state.predictedDisplayTime, &loc)) &&
                (loc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) && (loc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)) {
                input.aim_ok[h] = 1;
                input.aim_pos[h] = v3(loc.pose.position.x, loc.pose.position.y, loc.pose.position.z);
                input.aim_dir[h] = qrot(loc.pose.orientation, v3(0, 0, -1));
            }
        }
    }
    XrSpaceLocation head = { XR_TYPE_SPACE_LOCATION };
    input.head_fwd = v3(0, 0, -1);
    input.head_up = v3(0, 1, 0);
    if (!fix_head && XR_SUCCEEDED(xrLocateSpace(view_space, local_space, frame_state.predictedDisplayTime, &head)) &&
        (head.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)) {
        input.head_pos = v3(head.pose.position.x, head.pose.position.y, head.pose.position.z);
        input.head_fwd = qrot(head.pose.orientation, v3(0, 0, -1));
        input.head_up = qrot(head.pose.orientation, v3(0, 1, 0));
    }
    return 1;
}

void vr_eye_begin(int i, VrEye *e)
{
    EyeTarget *t = &eye[i];
    XrSwapchainImageAcquireInfo ai = { XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO };
    XrSwapchainImageWaitInfo wi = { XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO };
    wi.timeout = XR_INFINITE_DURATION;
    t->index = 0;
    t->acquired = XR_SUCCEEDED(xrAcquireSwapchainImage(t->sc, &ai, &t->index)) && XR_SUCCEEDED(xrWaitSwapchainImage(t->sc, &wi));
    if (t->index >= (uint32_t)t->n) t->index = 0;
    const XrView *v = &views[i];
    e->pos = v3(v->pose.position.x, v->pose.position.y, v->pose.position.z);
    e->fwd = qrot(v->pose.orientation, v3(0, 0, -1));
    e->up = qrot(v->pose.orientation, v3(0, 1, 0));
    e->tan_l = tan(v->fov.angleLeft);
    e->tan_r = tan(v->fov.angleRight);
    e->tan_u = tan(v->fov.angleUp);
    e->tan_d = tan(v->fov.angleDown);
    if (e->tan_r - e->tan_l < 0.1 || e->tan_u - e->tan_d < 0.1) {
        // No field of view from the runtime yet: something sane.
        e->tan_l = -1; e->tan_r = 1; e->tan_u = 1; e->tan_d = -1;
    }
    e->w = t->w;
    e->h = t->h;
    e->fbo = t->fbo[t->index];
    if (i == 0 && have_timer) {
        gpu_timer_collect();
        if (!gpu_q_used[gpu_q_next]) {
            glBeginQuery(GL_TIME_ELAPSED_EXT_, gpu_q[gpu_q_next]);
            gpu_q_used[gpu_q_next] = 2;   // running
        }
    }
}

void vr_eye_end(int i)
{
    EyeTarget *t = &eye[i];
    rlDrawRenderBatchActive();
    if (i == 1 && have_timer && gpu_q_used[gpu_q_next] == 2) {
        glEndQuery(GL_TIME_ELAPSED_EXT_);
        gpu_q_used[gpu_q_next] = 1;
        gpu_q_next = (gpu_q_next + 1) % NQUERY;
    }
    // The depth buffer is not needed once the eye is drawn: tell the tiler not to store it.
    glBindFramebuffer(GL_FRAMEBUFFER, t->fbo[t->index]);
    const GLenum drop[] = { GL_DEPTH_ATTACHMENT };
    glInvalidateFramebuffer(GL_FRAMEBUFFER, 1, drop);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (t->acquired) {
        XrSwapchainImageReleaseInfo ri = { XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO };
        xrReleaseSwapchainImage(t->sc, &ri);
        t->acquired = 0;
    }
}

static int create_ui(int w, int h)
{
    XrSwapchainCreateInfo ci = { XR_TYPE_SWAPCHAIN_CREATE_INFO };
    ci.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
    ci.format = color_format;
    ci.sampleCount = 1;
    ci.width = (uint32_t)w;
    ci.height = (uint32_t)h;
    ci.faceCount = 1;
    ci.arraySize = 1;
    ci.mipCount = 1;
    if (!ok(xrCreateSwapchain(session, &ci, &ui.sc), "xrCreateSwapchain (display)")) return 0;
    uint32_t n = 0;
    xrEnumerateSwapchainImages(ui.sc, 0, &n, NULL);
    if (n > MAX_IMAGES) n = MAX_IMAGES;
    XrSwapchainImageOpenGLESKHR img[MAX_IMAGES];
    for (uint32_t i = 0; i < n; i++) { img[i].type = XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_ES_KHR; img[i].next = NULL; }
    if (!ok(xrEnumerateSwapchainImages(ui.sc, n, &n, (XrSwapchainImageBaseHeader *)img), "display images")) return 0;
    ui.n = (int)n;
    for (uint32_t i = 0; i < n; i++) {
        glGenFramebuffers(1, &ui.fbo[i]);
        glBindFramebuffer(GL_FRAMEBUFFER, ui.fbo[i]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, img[i].image, 0);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) { glBindFramebuffer(GL_FRAMEBUFFER, 0); return 0; }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    ui.w = w;
    ui.h = h;
    LOGI("VR: display layer %dx%d%s", w, h, have_layer_settings ? ", super-sampled" : "");
    return 1;
}

unsigned int vr_ui_begin(int w, int h)
{
    static int failed;
    if (failed || !frame_begun) return 0;
    if (!ui.sc && !create_ui(w, h)) { failed = 1; LOGE("VR: no display layer, drawing it in the scene"); return 0; }
    XrSwapchainImageAcquireInfo ai = { XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO };
    XrSwapchainImageWaitInfo wi = { XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO };
    wi.timeout = XR_INFINITE_DURATION;
    ui.index = 0;
    ui.acquired = XR_SUCCEEDED(xrAcquireSwapchainImage(ui.sc, &ai, &ui.index)) && XR_SUCCEEDED(xrWaitSwapchainImage(ui.sc, &wi));
    if (!ui.acquired || ui.index >= (uint32_t)ui.n) return 0;
    return ui.fbo[ui.index];
}

void vr_ui_end(V3 centre, double width, double height)
{
    if (!ui.acquired) return;
    rlDrawRenderBatchActive();
    XrSwapchainImageReleaseInfo ri = { XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO };
    xrReleaseSwapchainImage(ui.sc, &ri);
    ui.acquired = 0;
    ui.submit = 1;
    ui.pose.orientation = (XrQuaternionf){ 0, 0, 0, 1 };
    ui.pose.position = (XrVector3f){ (float)centre.x, (float)centre.y, (float)centre.z };
    ui.size = (XrExtent2Df){ (float)width, (float)height };
}

void vr_end_frame(void)
{
    if (!frame_begun) return;
    frame_begun = 0;
    XrCompositionLayerProjectionView pv[2];
    memset(pv, 0, sizeof pv);
    for (int i = 0; i < 2; i++) {
        pv[i].type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW;
        pv[i].pose = views[i].pose;
        pv[i].fov = views[i].fov;
        pv[i].subImage.swapchain = eye[i].sc;
        pv[i].subImage.imageRect.extent.width = eye[i].w;
        pv[i].subImage.imageRect.extent.height = eye[i].h;
    }
    XrCompositionLayerProjection layer = { XR_TYPE_COMPOSITION_LAYER_PROJECTION };
    layer.space = local_space;
    layer.viewCount = 2;
    layer.views = pv;
    XrCompositionLayerQuad quad = { XR_TYPE_COMPOSITION_LAYER_QUAD };
    XrCompositionLayerSettingsFB qs = { XR_TYPE_COMPOSITION_LAYER_SETTINGS_FB };
    const XrCompositionLayerBaseHeader *layers[2] = { (const XrCompositionLayerBaseHeader *)&layer, NULL };
    uint32_t nlayers = 1;
    if (ui.submit) {
        ui.submit = 0;
        quad.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
        quad.space = local_space;
        quad.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
        quad.subImage.swapchain = ui.sc;
        quad.subImage.imageRect.extent.width = ui.w;
        quad.subImage.imageRect.extent.height = ui.h;
        quad.pose = ui.pose;
        quad.size = ui.size;
        if (have_layer_settings) {
            // Text: let the compositor super-sample and sharpen the panel.
            qs.layerFlags = XR_COMPOSITION_LAYER_SETTINGS_QUALITY_SUPER_SAMPLING_BIT_FB | XR_COMPOSITION_LAYER_SETTINGS_NORMAL_SHARPENING_BIT_FB;
            quad.next = &qs;
        }
        layers[nlayers++] = (const XrCompositionLayerBaseHeader *)&quad;
    }
    XrFrameEndInfo ei = { XR_TYPE_FRAME_END_INFO };
    ei.displayTime = frame_state.predictedDisplayTime;
    ei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    ei.layerCount = nlayers;
    ei.layers = layers;
    if (XR_FAILED(xrEndFrame(session, &ei))) LOGE("VR: xrEndFrame failed");
}

void vr_sim_save(const char *path) { (void)path; }

// The app's own folder on the shared storage (reachable with adb push).
const char *agc_android_external_dir(void)
{
    struct android_app *app = GetAndroidApp();
    return app && app->activity ? app->activity->externalDataPath : NULL;
}

// Where settings can be written on the headset.
const char *agc_android_data_dir(void)
{
    struct android_app *app = GetAndroidApp();
    return app && app->activity ? app->activity->internalDataPath : NULL;
}

#endif
