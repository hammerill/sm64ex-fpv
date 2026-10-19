// FPV drone mode for sm64ex, built on libfpv (Betaflight in the loop).
//
// Mario deploys the drone with the "Drone Toggle" bind and stands still as its
// pilot: the line-of-sight camera and the video link are measured from him.
// The drone collides with the level (static surfaces exported once per area,
// object collision models as moving libfpv surface objects, water and lava
// kill the props), collects coins, stars and 1-ups, and knocks out enemies it
// hits at speed. World units: 100 per metre, so Mario is about 1.6 m tall.
// With a kamikaze build of libfpv (LIBFPV_KAMIKAZE), the drone can carry a
// warhead (F8 > Fly > "Bomb installed"): its blast slays the enemies around it
// and hits the bosses.

#ifdef FPV_DRONE

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <SDL2/SDL.h>
#include <libfpv.h>

#include "sm64.h"
#include "types.h"
#include "audio/external.h"
#include "engine/graph_node.h"
#include "engine/math_util.h"
#include "engine/surface_collision.h"
#include "engine/surface_load.h"
#include "game/area.h"
#include "game/behavior_actions.h"
#include "game/camera.h"
#include "game/game_init.h"
#include "game/interaction.h"
#include "game/level_update.h"
#include "game/main.h"
#include "game/mario.h"
#include "game/memory.h"
#include "game/object_helpers.h"
#include "game/object_list_processor.h"
#include "game/spawn_sound.h"
#include "behavior_data.h"
#include "model_ids.h"
#include "surface_terrains.h"
#include "object_constants.h"
#include "audio_defines.h"
#include "pc/configfile.h"
#include "pc/platform.h"
#include "pc/controller/controller_api.h"
#include "pc/gfx/gfx_pc.h"
#ifdef RAPI_GL
#include "pc/gfx/gfx_fpv_gl.h"
#else
#define gfx_fpv_gl_postprocess(...)
#define gfx_fpv_gl_draw(...)
#endif
#include "fpv_sm64.h"

// from interaction.c
u32 interact_coin(struct MarioState *m, u32 interactType, struct Object *o);
u32 interact_star_or_key(struct MarioState *m, u32 interactType, struct Object *o);
u32 attack_object(struct Object *o, s32 interaction);
#define FPV_INT_GROUND_POUND (1 << 0) // INT_GROUND_POUND_OR_TWIRL in interaction.c
#define FPV_INT_KICK (1 << 2)         // INT_KICK

#define UNITS_PER_METER 100.0f
#define DRONE_RADIUS 14.0f        // collection/attack reach around the frame centre
#define ATTACK_MIN_SPEED 200.0f   // units/s, 2 m/s
#define BONK_MIN_SPEED 300.0f
#define MAT_DEFAULT 0
#define MAT_LAVA 1
#define MAT_NONE 2
#define MAT_WATER 3

enum CamMode { CAM_FPV, CAM_CHASE, CAM_LOS, CAM_COUNT };
// buttons in the overlay's Fly tab (F8), the same as the drone binds
enum OverlayAction { ACTION_TOGGLE = 1, ACTION_RESET, ACTION_CAMERA, ACTION_ENDING, ACTION_HERE };

// Objects spawn, show, wake up and aim by their distance and angle to Mario's object. As
// gtaiv-fpv streams the world with Niko as an anchor, Mario's object stands in at the drone
// while the other objects update once the drone is far from him, so the level comes alive
// around it. Mario stays where he is for everything else: his own update, object collisions
// (enemies can't hurt him at the drone), the camera and the video link.
#define ANCHOR_START 2000.0f // 20 m from Mario: the stand-in follows the drone
#define ANCHOR_STOP 1000.0f  // back within 10 m: Mario himself again

static s32 sInited;
static s32 sActive;
static fpv_drone *sDrone;
static fpv_frame sFrame;
static s32 sHaveFrame;
static s32 sCamMode;
static u16 sPadPrev, sPadPressed;
static u64 sLastTick;
static u64 sMouseTick;          // last mouse motion, click or wheel (0: none yet)
static float sTime;
static u32 sUpdateTimer;        // gGlobalTimer of the last game frame that updated the drone
static float sImpactSpeed;      // hardest impact since the last game frame

static s16 sLevel = -1, sArea = -1;
static float sSpawnPos[3], sSpawnRot[4];
static double sSpawnSimTime;    // sim_time of the first frame after fpv_drone_create
static float sWarmup;           // host seconds since then, while the frames may be stale
static s32 sActionsShown = -1;  // sActive when the overlay buttons were last set
static Vec3f sPrevPos;

static u32 sTerrainGen = 1, sLoadedTerrainGen;
static s16 sLoadedTerrainLevel = -1, sLoadedTerrainArea = -1;
static u32 sWaterId;
static float sWaterLevel = -20000.0f;

static struct Object *sDroneObj;
static Mat4 sDroneMtx;

static u8 sKeysDown[512];

static s32 sAnchor;              // the stand-in follows the drone this game frame
static s32 sAnchored;            // Mario's object is at the drone now (inside cur_obj_update)
static Vec3f sAnchorPos, sAnchorGfxPos; // Mario's object's own position meanwhile

#ifdef LIBFPV_KAMIKAZE
// After the blast (Options > Controls > After the Explosion): the static of the lost
// video link with "SIGNAL LOST", then Mario takes over; or arcade, a cutscene of the
// blast, then a new drone at the launch point. Its length is the player's (F8 > Fly, "Ending").
enum BoomEnding { BOOM_NONE, BOOM_SIGNAL_LOST, BOOM_ARCADE };
static s32 sBoom;
static u32 sBoomFrames;     // game frames since the blast
static u32 sBoomEndFrames;  // game frames the ending lasts
static float sBoomTime;     // sTime of the blast
static Vec3f sBoomPos;
static f32 sBoomCamYaw, sBoomCamReach;
static fpv_kamikaze_state sWarhead;
static u32 sDetonations;
#endif

// --- self test (SM64_FPV_TEST=<level>[,chase|los]) --------------------------------------------
// Boots through the debug level select into <level>, deploys the drone, flies a
// scripted autopilot and logs telemetry to <user path>/fpv_test.log, so a build
// can be checked without touching the keyboard.
static s32 sTestLevel;
static s32 sTestCam;
static u32 sTestFrame;
static s32 sTestLevelFrames = -1;
static float sTestTime;
static FILE *sTestLog;
static struct Object *sTestGoomba;
static s32 sTestGoombaHit;
static s32 sTestDeploys;
static s32 sTestOverlay;
static s32 sTestDive;
static s32 sTestClean; // flip the OSD disable switch (AUX6) on halfway through the flight
static s32 sTestPilot; // look at Mario from the front instead of a drone camera
static f32 sTestPilotYaw; // pilot=<degrees>: around him, 0 = straight in front
static u32 sTestDrawnFrames; // displayed frames, for the test's framerate log
static s32 sTestKamikaze; // kamikaze builds: arm the warhead and strike the Goomba
static s32 sTestHere; // end the first flight with "Move Mario Here" instead of a recall
static s32 sTestFar;  // move the first drone 30 m out at 2 s, past where the stand-in follows it

static void test_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
struct fpv_camera;
static f32 camera_fov(const fpv_camera *c);
#include <stdarg.h>
static void test_log(const char *fmt, ...) {
    if (!sTestLog) return;
    if (!sTestLevel && fmt[0] == '[') return; // libfpv chatter only in test runs
    va_list ap;
    va_start(ap, fmt);
    vfprintf(sTestLog, fmt, ap);
    va_end(ap);
    fputc('\n', sTestLog);
    fflush(sTestLog);
}

// --- small math ---------------------------------------------------------------------

static void quat_rotate(const float q[4], const float v[3], float out[3]) {
    // v + 2w (q x v) + 2 q x (q x v)
    float tx = 2.0f * (q[1] * v[2] - q[2] * v[1]);
    float ty = 2.0f * (q[2] * v[0] - q[0] * v[2]);
    float tz = 2.0f * (q[0] * v[1] - q[1] * v[0]);
    out[0] = v[0] + q[3] * tx + (q[1] * tz - q[2] * ty);
    out[1] = v[1] + q[3] * ty + (q[2] * tx - q[0] * tz);
    out[2] = v[2] + q[3] * tz + (q[0] * ty - q[1] * tx);
}

// rotation matrix (column vectors r[i][j]) to quaternion {x, y, z, w}
static void quat_from_matrix(float r[3][3], float q[4]) {
    float tr = r[0][0] + r[1][1] + r[2][2];
    if (tr > 0.0f) {
        float s = sqrtf(tr + 1.0f) * 2.0f;
        q[3] = 0.25f * s;
        q[0] = (r[2][1] - r[1][2]) / s;
        q[1] = (r[0][2] - r[2][0]) / s;
        q[2] = (r[1][0] - r[0][1]) / s;
    } else if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
        float s = sqrtf(1.0f + r[0][0] - r[1][1] - r[2][2]) * 2.0f;
        q[3] = (r[2][1] - r[1][2]) / s;
        q[0] = 0.25f * s;
        q[1] = (r[0][1] + r[1][0]) / s;
        q[2] = (r[0][2] + r[2][0]) / s;
    } else if (r[1][1] > r[2][2]) {
        float s = sqrtf(1.0f + r[1][1] - r[0][0] - r[2][2]) * 2.0f;
        q[3] = (r[0][2] - r[2][0]) / s;
        q[0] = (r[0][1] + r[1][0]) / s;
        q[1] = 0.25f * s;
        q[2] = (r[1][2] + r[2][1]) / s;
    } else {
        float s = sqrtf(1.0f + r[2][2] - r[0][0] - r[1][1]) * 2.0f;
        q[3] = (r[1][0] - r[0][1]) / s;
        q[0] = (r[0][2] + r[2][0]) / s;
        q[1] = (r[1][2] + r[2][1]) / s;
        q[2] = 0.25f * s;
    }
}

// SM64 row-vector matrix (rows = transformed local axes, row 3 = translation,
// rows may carry scale) to a libfpv transform
static void transform_from_mat4(Mat4 m, fpv_transform *t) {
    float r[3][3];
    for (int j = 0; j < 3; j++) {
        float len = sqrtf(m[j][0] * m[j][0] + m[j][1] * m[j][1] + m[j][2] * m[j][2]);
        t->scale[j] = len > 0.0f ? len : 1.0f;
        for (int i = 0; i < 3; i++) r[i][j] = m[j][i] / t->scale[j];
        t->position[j] = m[3][j];
    }
    quat_from_matrix(r, t->rotation);
}

static u64 now_ticks(void) {
    return SDL_GetPerformanceCounter();
}

// --- logging --------------------------------------------------------------------------

static void log_cb(UNUSED void *user, fpv_log_level level, const char *message) {
    test_log("[libfpv] %s", message);
    if (level >= FPV_LOG_INFO) {
        fprintf(stdout, "[libfpv] %s\n", message);
        fflush(stdout);
    }
}

// --- drone model -------------------------------------------------------------------------
// Vertices are in millimetres (the object is drawn at scale 0.1), forward is -Z.

#define MAX_BOXES 12
static Vtx sBodyVtx[MAX_BOXES * 24];
static Gfx sBodyGfx[MAX_BOXES * 8 + 8];
static struct GraphNodeGenerated sBodyNode, sPropNode;
static float sPropAngle[4];
static float sMotorMm[4][3];
static float sPropRadiusMm = 64.0f;

static void set_vtx(Vtx *v, float x, float y, float z, const u8 *col, float shade) {
    v->v.ob[0] = (s16) x;
    v->v.ob[1] = (s16) y;
    v->v.ob[2] = (s16) z;
    v->v.flag = 0;
    v->v.tc[0] = 0;
    v->v.tc[1] = 0;
    v->v.cn[0] = (u8) (col[0] * shade);
    v->v.cn[1] = (u8) (col[1] * shade);
    v->v.cn[2] = (u8) (col[2] * shade);
    v->v.cn[3] = 255;
}

// 24 vertices (4 per face, baked shading) of a box with centre c, half extents h, yaw around Y
static void build_box(Vtx *v, float cx, float cy, float cz, float hx, float hy, float hz, float yaw,
                      const u8 *col) {
    static const s8 faces[6][4][3] = {
        { { -1, 1, -1 }, { -1, 1, 1 }, { 1, 1, 1 }, { 1, 1, -1 } },     // +y
        { { -1, -1, -1 }, { 1, -1, -1 }, { 1, -1, 1 }, { -1, -1, 1 } }, // -y
        { { -1, -1, 1 }, { 1, -1, 1 }, { 1, 1, 1 }, { -1, 1, 1 } },     // +z
        { { 1, -1, -1 }, { -1, -1, -1 }, { -1, 1, -1 }, { 1, 1, -1 } }, // -z
        { { 1, -1, 1 }, { 1, -1, -1 }, { 1, 1, -1 }, { 1, 1, 1 } },     // +x
        { { -1, -1, -1 }, { -1, -1, 1 }, { -1, 1, 1 }, { -1, 1, -1 } }, // -x
    };
    static const float shades[6] = { 1.0f, 0.4f, 0.75f, 0.85f, 0.6f, 0.65f };
    float cs = cosf(yaw), sn = sinf(yaw);
    for (int f = 0; f < 6; f++) {
        for (int k = 0; k < 4; k++) {
            float x = faces[f][k][0] * hx, y = faces[f][k][1] * hy, z = faces[f][k][2] * hz;
            float rx = x * cs + z * sn, rz = -x * sn + z * cs;
            set_vtx(&v[f * 4 + k], cx + rx, cy + y, cz + rz, col, shades[f]);
        }
    }
}

static void build_drone_model(const fpv_frame *f) {
    static const u8 carbon[3] = { 40, 40, 46 };
    static const u8 red[3] = { 220, 30, 30 };
    static const u8 white[3] = { 235, 235, 235 };
    static const u8 orange[3] = { 255, 140, 20 };
    static const u8 black[3] = { 10, 10, 10 };
    static const u8 gold[3] = { 250, 200, 40 };
    int n = 0;
    float maxR = 0.0f;

    for (int i = 0; i < 4; i++) {
        for (int k = 0; k < 3; k++) sMotorMm[i][k] = f->motors[i].position[k] * 10.0f; // host units -> mm
        float r = sqrtf(sMotorMm[i][0] * sMotorMm[i][0] + sMotorMm[i][2] * sMotorMm[i][2]);
        if (r > maxR) maxR = r;
    }
    if (maxR < 30.0f) { // presets always have motors, but stay drawable
        for (int i = 0; i < 4; i++) {
            sMotorMm[i][0] = (i & 1) ? 80.0f : -80.0f;
            sMotorMm[i][1] = 0.0f;
            sMotorMm[i][2] = (i & 2) ? 80.0f : -80.0f;
        }
        maxR = 113.0f;
    }
    // props a bit shorter than half the motor spacing
    sPropRadiusMm = maxR * 0.62f;

    // arms
    for (int i = 0; i < 4; i++) {
        float mx = sMotorMm[i][0], mz = sMotorMm[i][2];
        float len = sqrtf(mx * mx + mz * mz);
        build_box(&sBodyVtx[24 * n++], mx * 0.5f, 0.0f, mz * 0.5f, 7.0f, 2.5f, len * 0.5f, atan2f(mx, mz), carbon);
    }
    // centre plates and stack
    build_box(&sBodyVtx[24 * n++], 0.0f, 12.0f, 0.0f, 22.0f, 12.0f, 45.0f, 0.0f, carbon);
    // battery strapped on top, Mario red with a white band
    build_box(&sBodyVtx[24 * n++], 0.0f, 37.0f, 8.0f, 18.0f, 13.0f, 36.0f, 0.0f, red);
    build_box(&sBodyVtx[24 * n++], 0.0f, 37.0f, 8.0f, 18.5f, 13.5f, 5.0f, 0.0f, white);
    // FPV camera (orange) with lens at the front (-Z), and a gold VTX antenna at the back
    build_box(&sBodyVtx[24 * n++], 0.0f, 16.0f, -44.0f, 10.0f, 10.0f, 9.0f, 0.0f, orange);
    build_box(&sBodyVtx[24 * n++], 0.0f, 16.0f, -55.0f, 6.0f, 6.0f, 3.0f, 0.0f, black);
    build_box(&sBodyVtx[24 * n++], 0.0f, 30.0f, 52.0f, 3.0f, 22.0f, 3.0f, 0.0f, gold);

    Gfx *g = sBodyGfx;
    gDPPipeSync(g++);
    gSPClearGeometryMode(g++, G_LIGHTING | G_CULL_BACK);
    gSPTexture(g++, 0xFFFF, 0xFFFF, 0, G_TX_RENDERTILE, G_OFF);
    gDPSetCombineMode(g++, G_CC_SHADE, G_CC_SHADE);
    for (int b = 0; b < n; b++) {
        gSPVertex(g++, &sBodyVtx[24 * b], 24, 0);
        for (int face = 0; face < 6; face++) {
            int o = face * 4;
            gSP2Triangles(g++, o, o + 1, o + 2, 0, o, o + 2, o + 3, 0);
        }
    }
    gSPSetGeometryMode(g++, G_LIGHTING | G_CULL_BACK);
    gSPEndDisplayList(g++);
}

static Vtx sMotorVtx[4 * 24];
static Gfx sMotorGfx[4 * 8 + 8];

static void build_motor_model(void) {
    static const u8 metal[3] = { 175, 175, 190 };
    Gfx *g = sMotorGfx;
    for (int i = 0; i < 4; i++) {
        build_box(&sMotorVtx[24 * i], sMotorMm[i][0], sMotorMm[i][1] + 9.0f, sMotorMm[i][2], 12.0f, 9.0f, 12.0f,
                  0.785f, metal);
    }
    gDPPipeSync(g++);
    gSPClearGeometryMode(g++, G_LIGHTING | G_CULL_BACK);
    gSPTexture(g++, 0xFFFF, 0xFFFF, 0, G_TX_RENDERTILE, G_OFF);
    gDPSetCombineMode(g++, G_CC_SHADE, G_CC_SHADE);
    for (int i = 0; i < 4; i++) {
        gSPVertex(g++, &sMotorVtx[24 * i], 24, 0);
        for (int face = 0; face < 6; face++) {
            int o = face * 4;
            gSP2Triangles(g++, o, o + 1, o + 2, 0, o, o + 2, o + 3, 0);
        }
    }
    gSPSetGeometryMode(g++, G_LIGHTING | G_CULL_BACK);
    gSPEndDisplayList(g++);
}

static s32 sModelBuilt;

static Gfx *geo_fpv_drone_body(s32 callContext, UNUSED struct GraphNode *node, UNUSED void *context) {
    if (callContext != GEO_CONTEXT_RENDER || !sModelBuilt) return NULL;
    Gfx *dl = alloc_display_list(3 * sizeof(Gfx));
    if (!dl) return NULL;
    gSPDisplayList(&dl[0], sBodyGfx);
    gSPDisplayList(&dl[1], sMotorGfx);
    gSPEndDisplayList(&dl[2]);
    return dl;
}

// propellers: three blades when slow, a translucent disc when spinning
#define PROP_SEGMENTS 12
static Gfx *geo_fpv_drone_props(s32 callContext, UNUSED struct GraphNode *node, UNUSED void *context) {
    if (callContext != GEO_CONTEXT_RENDER || !sHaveFrame || !sModelBuilt) return NULL;
    Vtx *v = alloc_display_list(4 * (PROP_SEGMENTS + 1) * sizeof(Vtx));
    Gfx *dl = alloc_display_list((8 + 4 * (2 + PROP_SEGMENTS)) * sizeof(Gfx));
    if (!v || !dl) return NULL;
    Gfx *g = dl;
    gDPPipeSync(g++);
    gSPClearGeometryMode(g++, G_LIGHTING | G_CULL_BACK);
    gSPTexture(g++, 0xFFFF, 0xFFFF, 0, G_TX_RENDERTILE, G_OFF);
    gDPSetCombineMode(g++, G_CC_SHADE, G_CC_SHADE);
    for (int i = 0; i < 4; i++) {
        const fpv_motor_state *m = &sFrame.motors[i];
        if (m->flags & FPV_MOTOR_PROP_BROKEN) continue;
        Vtx *pv = &v[i * (PROP_SEGMENTS + 1)];
        float cx = sMotorMm[i][0], cy = sMotorMm[i][1] + 20.0f, cz = sMotorMm[i][2];
        float spin = fabsf(m->rpm);
        static const u8 blur[3] = { 200, 210, 230 };
        static const u8 blade[3] = { 60, 150, 255 };
        if (spin > 1500.0f) {
            // disc: centre + ring
            u8 a = (u8) (spin > 8000.0f ? 110 : 60 + (spin - 1500.0f) / 6500.0f * 50.0f);
            set_vtx(&pv[0], cx, cy, cz, blur, 1.0f);
            pv[0].v.cn[3] = a;
            for (int s = 0; s < PROP_SEGMENTS; s++) {
                float ang = s * 6.2831853f / PROP_SEGMENTS;
                set_vtx(&pv[1 + s], cx + cosf(ang) * sPropRadiusMm, cy, cz + sinf(ang) * sPropRadiusMm, blur, 0.9f);
                pv[1 + s].v.cn[3] = a / 2;
            }
            gSPVertex(g++, pv, PROP_SEGMENTS + 1, 0);
            for (int s = 0; s < PROP_SEGMENTS; s++) gSP1Triangle(g++, 0, 1 + s, 1 + (s + 1) % PROP_SEGMENTS, 0);
        } else {
            float a0 = sPropAngle[i];
            set_vtx(&pv[0], cx, cy, cz, blade, 1.0f);
            for (int b = 0; b < 3; b++) {
                float ang = a0 + b * 2.0943951f;
                float tx = cosf(ang) * sPropRadiusMm, tz = sinf(ang) * sPropRadiusMm;
                float sx = -sinf(ang) * 9.0f, sz = cosf(ang) * 9.0f;
                set_vtx(&pv[1 + b * 2], cx + tx + sx, cy, cz + tz + sz, blade, 1.0f);
                set_vtx(&pv[2 + b * 2], cx + tx - sx, cy, cz + tz - sz, blade, 0.8f);
            }
            gSPVertex(g++, pv, 7, 0);
            for (int b = 0; b < 3; b++) gSP1Triangle(g++, 0, 1 + b * 2, 2 + b * 2, 0);
        }
    }
    gSPSetGeometryMode(g++, G_LIGHTING | G_CULL_BACK);
    gSPEndDisplayList(g++);
    return dl;
}

static void init_drone_nodes(void) {
    init_graph_node_generated(NULL, &sBodyNode, geo_fpv_drone_body, 0);
    init_graph_node_generated(NULL, &sPropNode, geo_fpv_drone_props, 0);
    sBodyNode.fnNode.node.flags = (LAYER_OPAQUE << 8) | GRAPH_RENDER_ACTIVE;
    sPropNode.fnNode.node.flags = (LAYER_TRANSPARENT << 8) | GRAPH_RENDER_ACTIVE;
    // siblings: processed together as the object's shared child
    sBodyNode.fnNode.node.next = &sPropNode.fnNode.node;
    sBodyNode.fnNode.node.prev = &sPropNode.fnNode.node;
    sPropNode.fnNode.node.next = &sBodyNode.fnNode.node;
    sPropNode.fnNode.node.prev = &sBodyNode.fnNode.node;
}

// --- the pilot's radio -----------------------------------------------------------------------
// Drawn in Mario's hands (mario_misc.c) while he pilots, in world units. The
// radio's face points up and back at him and its sticks follow the channels
// Betaflight sees (mode 2: throttle and yaw on the left stick).

#define RADIO_BOXES 14
#define RADIO_TILT 0.75f // radians the face is tipped back towards Mario
static struct Object sRadioObj;
static struct GraphNodeGenerated sRadioNode;

// a box in the radio's frame (x left, y out of the face, z away from Mario), tipped back
static void radio_box(Vtx *v, float cx, float cy, float cz, float hx, float hy, float hz, const u8 *col) {
    build_box(v, cx, cy, cz, hx, hy, hz, 0.0f, col);
    float c = cosf(RADIO_TILT), sn = sinf(RADIO_TILT);
    for (int i = 0; i < 24; i++) {
        float y = v[i].v.ob[1], z = v[i].v.ob[2];
        v[i].v.ob[1] = (s16) (y * c + z * sn);
        v[i].v.ob[2] = (s16) (-y * sn + z * c);
    }
}

static Gfx *geo_fpv_radio(s32 callContext, UNUSED struct GraphNode *node, UNUSED void *context) {
    if (callContext != GEO_CONTEXT_RENDER || !sActive) return NULL;
    static const u8 body[3] = { 52, 54, 62 };
    static const u8 grip[3] = { 28, 28, 32 };
    static const u8 screen[3] = { 70, 180, 230 };
    static const u8 black[3] = { 12, 12, 14 };
    static const u8 metal[3] = { 190, 190, 200 };
    static const u8 red[3] = { 220, 30, 30 };
    Vtx *v = alloc_display_list(RADIO_BOXES * 24 * sizeof(Vtx));
    Gfx *g = alloc_display_list((RADIO_BOXES * 7 + 8) * sizeof(Gfx));
    if (!v || !g) return NULL;
    Gfx *dl = g;
    float ch[4] = { 0.0f, 0.0f, -1.0f, 0.0f };
    if (sHaveFrame) memcpy(ch, sFrame.channels, sizeof(ch));

    // in fifths of a unit (the radio is drawn at scale 0.2)
    int n = 0;
    radio_box(&v[24 * n++], 0, 0, 0, 120, 25, 70, body);
    radio_box(&v[24 * n++], 105, -15, -55, 30, 35, 40, grip);
    radio_box(&v[24 * n++], -105, -15, -55, 30, 35, 40, grip);
    radio_box(&v[24 * n++], 0, 27, -35, 38, 2, 24, screen);
    radio_box(&v[24 * n++], 0, 15, 85, 30, 15, 15, red);         // top handle
    radio_box(&v[24 * n++], 80, 20, 90, 8, 8, 60, black);        // antenna
    radio_box(&v[24 * n++], 100, 45, 55, 5, 20, 5, metal);       // switches
    radio_box(&v[24 * n++], -100, 45, 55, 5, 20, 5, metal);
    // gimbals, then the sticks leaning with their channel (+x is Mario's left)
    const float stickX[2] = { 65, -65 };
    const float stickDx[2] = { -ch[3], -ch[0] }, stickDz[2] = { ch[2], ch[1] };
    for (int i = 0; i < 2; i++) {
        radio_box(&v[24 * n++], stickX[i], 27, 25, 28, 3, 28, black);
        radio_box(&v[24 * n++], stickX[i] + stickDx[i] * 10, 48, 25 + stickDz[i] * 10, 6, 20, 6, metal);
        radio_box(&v[24 * n++], stickX[i] + stickDx[i] * 20, 70, 25 + stickDz[i] * 20, 11, 6, 11, black);
    }

    gDPPipeSync(g++);
    gSPClearGeometryMode(g++, G_LIGHTING | G_CULL_BACK);
    gSPTexture(g++, 0xFFFF, 0xFFFF, 0, G_TX_RENDERTILE, G_OFF);
    gDPSetCombineMode(g++, G_CC_SHADE, G_CC_SHADE);
    for (int b = 0; b < n; b++) {
        gSPVertex(g++, &v[24 * b], 24, 0);
        for (int face = 0; face < 6; face++) {
            int o = face * 4;
            gSP2Triangles(g++, o, o + 1, o + 2, 0, o, o + 2, o + 3, 0);
        }
    }
    gSPSetGeometryMode(g++, G_LIGHTING | G_CULL_BACK);
    gSPEndDisplayList(g++);
    return dl;
}

static void init_radio(void) {
    init_graph_node_generated(NULL, &sRadioNode, geo_fpv_radio, 0);
    sRadioNode.fnNode.node.flags = (LAYER_OPAQUE << 8) | GRAPH_RENDER_ACTIVE;
    sRadioNode.fnNode.node.next = sRadioNode.fnNode.node.prev = &sRadioNode.fnNode.node;
    sRadioObj.header.gfx.sharedChild = &sRadioNode.fnNode.node;
    vec3f_set(sRadioObj.header.gfx.scale, 0.2f, 0.2f, 0.2f);
}

struct Object *fpv_sm64_radio_object(void) {
    return sActive ? &sRadioObj : NULL;
}

// The head follows the drone, and the whole body turns when it gets behind him.
// Like real pilots, he leans into the roll and the yaw.
void fpv_sm64_pilot_pose(struct MarioState *m) {
    struct MarioBodyState *body = m->marioBodyState;
    body->handState = MARIO_HAND_FISTS;
    s16 yaw = m->faceAngle[1], pitch = 0;
    if (sHaveFrame) {
        f32 dx = sFrame.position[0] - m->pos[0], dz = sFrame.position[2] - m->pos[2];
        f32 dy = sFrame.position[1] - (m->pos[1] + 120.0f);
        f32 dist = sqrtf(dx * dx + dz * dz);
        if (dist > 1.0f) yaw = atan2s(dz, dx);
        pitch = atan2s(dist, dy);
    }
    s16 dYaw = yaw - m->faceAngle[1];
    if (dYaw > 0x2800) m->faceAngle[1] = approach_s16_symmetric(m->faceAngle[1], (s16) (yaw - 0x2800), 0x400);
    if (dYaw < -0x2800) m->faceAngle[1] = approach_s16_symmetric(m->faceAngle[1], (s16) (yaw + 0x2800), 0x400);
    dYaw = yaw - m->faceAngle[1];
    if (dYaw > 0x2800) dYaw = 0x2800;
    if (dYaw < -0x2800) dYaw = -0x2800;
    // negative head pitch looks up
    s32 headPitch = -pitch;
    if (headPitch < -0x2400) headPitch = -0x2400;
    if (headPitch > 0x1400) headPitch = 0x1400;
    body->headAngle[0] = approach_s32(body->headAngle[0], headPitch, 0x600, 0x600);
    body->headAngle[1] = approach_s32(body->headAngle[1], dYaw, 0x600, 0x600);
    body->headAngle[2] = 0;

    float roll = 0.0f, fwd = 0.0f;
    if (sHaveFrame) {
        roll = sFrame.channels[0] + 0.5f * sFrame.channels[3];
        fwd = sFrame.channels[1];
    }
    body->torsoAngle[0] = approach_s32(body->torsoAngle[0], (s32) (fwd * 0x0A00), 0x200, 0x200);
    body->torsoAngle[1] = 0;
    body->torsoAngle[2] = approach_s32(body->torsoAngle[2], (s32) (roll * 0x0C00), 0x200, 0x200);
}

// --- collision -----------------------------------------------------------------------------

static u8 material_for_surface(s16 type) {
    switch (type) {
        case SURFACE_BURNING:
            return MAT_LAVA;
        case SURFACE_INTANGIBLE:
        case SURFACE_CAMERA_BOUNDARY:
            return MAT_NONE;
        default:
            return MAT_DEFAULT;
    }
}

static void load_static_terrain(void) {
    s32 count = gNumStaticSurfaces;
    if (count <= 0 || sSurfacePool == NULL) {
        fpv_static_surfaces_clear();
        return;
    }
    float *pos = malloc(sizeof(float) * 9 * count);
    u8 *mats = malloc(count);
    if (!pos || !mats) {
        free(pos);
        free(mats);
        return;
    }
    for (s32 i = 0; i < count; i++) {
        struct Surface *s = &sSurfacePool[i];
        for (int k = 0; k < 3; k++) {
            pos[i * 9 + 0 + k] = s->vertex1[k];
            pos[i * 9 + 3 + k] = s->vertex2[k];
            pos[i * 9 + 6 + k] = s->vertex3[k];
        }
        mats[i] = material_for_surface(s->type);
    }
    fpv_mesh mesh;
    memset(&mesh, 0, sizeof(mesh));
    mesh.positions = pos;
    mesh.vertex_count = (u32) count * 3;
    mesh.triangle_count = (u32) count;
    mesh.materials = mats;
    fpv_result r = fpv_static_surfaces_load(&mesh, 1);
    if (r != FPV_OK) fprintf(stderr, "fpv: static surfaces: %s\n", fpv_last_error());
    free(pos);
    free(mats);
    sLoadedTerrainGen = sTerrainGen;
    sLoadedTerrainLevel = gCurrLevelNum;
    sLoadedTerrainArea = gCurrAreaIndex;
}

void fpv_sm64_terrain_loaded(void) {
    sTerrainGen++;
}

// Object collision models (platforms, doors, Whomps...) become libfpv surface
// objects while SM64 keeps them tangible.
#define MAX_SOLIDS 192
struct Solid {
    struct Object *obj;
    const s16 *data;
    u32 id;
    u32 frame;
    Mat4 mtx;
};
static struct Solid sSolids[MAX_SOLIDS];

static s32 has_force(s16 type) {
    switch (type) {
        case SURFACE_0004:
        case SURFACE_FLOWING_WATER:
        case SURFACE_DEEP_MOVING_QUICKSAND:
        case SURFACE_SHALLOW_MOVING_QUICKSAND:
        case SURFACE_MOVING_QUICKSAND:
        case SURFACE_HORIZONTAL_WIND:
        case SURFACE_INSTANT_MOVING_QUICKSAND:
            return TRUE;
        default:
            return FALSE;
    }
}

static u32 create_solid_mesh(const s16 *data) {
    // TERRAIN_LOAD_OBJECT, vertex count, vertices, then (type, count, triangles)
    // groups until TERRAIN_LOAD_CONTINUE
    const s16 *p = data + 1;
    s32 numVerts = *p++;
    if (numVerts <= 0 || numVerts > 2000) return 0;
    float *pos = malloc(sizeof(float) * 3 * numVerts);
    if (!pos) return 0;
    for (s32 i = 0; i < numVerts * 3; i++) pos[i] = *p++;
    s32 cap = 64, numTris = 0;
    u32 *idx = malloc(sizeof(u32) * 3 * cap);
    u8 *mats = malloc(cap);
    while (*p != TERRAIN_LOAD_CONTINUE && idx && mats) {
        s16 type = *p++;
        s32 count = *p++;
        s32 stride = has_force(type) ? 4 : 3;
        for (s32 t = 0; t < count; t++, p += stride) {
            if (p[0] < 0 || p[0] >= numVerts || p[1] < 0 || p[1] >= numVerts || p[2] < 0 || p[2] >= numVerts) continue;
            if (numTris == cap) {
                cap *= 2;
                idx = realloc(idx, sizeof(u32) * 3 * cap);
                mats = realloc(mats, cap);
                if (!idx || !mats) break;
            }
            idx[numTris * 3 + 0] = (u32) p[0];
            idx[numTris * 3 + 1] = (u32) p[1];
            idx[numTris * 3 + 2] = (u32) p[2];
            mats[numTris] = material_for_surface(type);
            numTris++;
        }
    }
    u32 id = 0;
    if (idx && mats && numTris > 0) {
        fpv_mesh mesh;
        memset(&mesh, 0, sizeof(mesh));
        mesh.positions = pos;
        mesh.vertex_count = (u32) numVerts;
        mesh.indices = idx;
        mesh.triangle_count = (u32) numTris;
        mesh.materials = mats;
        fpv_transform t;
        memset(&t, 0, sizeof(t));
        t.rotation[3] = 1.0f;
        t.position[1] = -100000.0f; // placed by the first move
        if (fpv_surface_object_create(&mesh, &t, &id) != FPV_OK) id = 0;
    }
    free(pos);
    free(idx);
    free(mats);
    return id;
}

void fpv_sm64_object_collision_loaded(struct Object *obj, Mat4 transform) {
    if (!sActive || obj->collisionData == NULL) return;
    struct Solid *slot = NULL, *freeSlot = NULL;
    for (int i = 0; i < MAX_SOLIDS; i++) {
        if (sSolids[i].obj == obj) {
            slot = &sSolids[i];
            break;
        }
        if (!sSolids[i].obj && !freeSlot) freeSlot = &sSolids[i];
    }
    if (slot && slot->data != obj->collisionData) {
        if (slot->id) fpv_surface_object_delete(slot->id);
        slot->id = 0;
        slot->data = NULL;
    }
    if (!slot) slot = freeSlot;
    if (!slot) return;
    slot->obj = obj;
    slot->frame = gGlobalTimer;
    obj_apply_scale_to_matrix(obj, slot->mtx, transform);
    if (!slot->data) {
        slot->data = obj->collisionData;
        slot->id = create_solid_mesh(obj->collisionData);
    }
}

static void sync_solids(void) {
    for (int i = 0; i < MAX_SOLIDS; i++) {
        struct Solid *s = &sSolids[i];
        if (!s->obj) continue;
        if (s->frame != gGlobalTimer || !(s->obj->activeFlags & ACTIVE_FLAG_ACTIVE)) {
            if (s->id) fpv_surface_object_delete(s->id);
            memset(s, 0, sizeof(*s));
            continue;
        }
        if (s->id) {
            fpv_transform t;
            transform_from_mat4(s->mtx, &t);
            fpv_surface_object_move(s->id, &t);
        }
    }
}

static void clear_solids(void) {
    for (int i = 0; i < MAX_SOLIDS; i++) {
        if (sSolids[i].id) fpv_surface_object_delete(sSolids[i].id);
    }
    memset(sSolids, 0, sizeof(sSolids));
}

// One huge plane that follows the water level under the drone: entering water
// (from above or through the side of a water box) kills the props.
static void create_water(void) {
    static const float pos[12] = { -20000, 0, -20000, 20000, 0, -20000, 20000, 0, 20000, -20000, 0, 20000 };
    static const u32 idx[6] = { 0, 1, 2, 0, 2, 3 };
    fpv_mesh mesh;
    memset(&mesh, 0, sizeof(mesh));
    mesh.positions = pos;
    mesh.vertex_count = 4;
    mesh.indices = idx;
    mesh.triangle_count = 2;
    mesh.material = MAT_WATER;
    fpv_transform t;
    memset(&t, 0, sizeof(t));
    t.rotation[3] = 1.0f;
    t.position[1] = sWaterLevel = -20000.0f;
    if (fpv_surface_object_create(&mesh, &t, &sWaterId) != FPV_OK) sWaterId = 0;
}

static void update_water(void) {
    if (!sWaterId || !sHaveFrame) return;
    f32 level = find_water_level(sFrame.position[0], sFrame.position[2]);
    if (level < -10000.0f) level = -20000.0f;
    if (level != sWaterLevel) {
        fpv_transform t;
        memset(&t, 0, sizeof(t));
        t.rotation[3] = 1.0f;
        t.position[1] = sWaterLevel = level;
        fpv_surface_object_move(sWaterId, &t);
    }
}

// --- interactions --------------------------------------------------------------------------

static s32 is_1up(struct Object *o) {
    const BehaviorScript *b = o->behavior;
    return b == segmented_to_virtual(bhv1Up) || b == segmented_to_virtual(bhv1upWalking)
           || b == segmented_to_virtual(bhv1upRunningAway) || b == segmented_to_virtual(bhv1upSliding)
           || b == segmented_to_virtual(bhv1upJumpOnApproach) || b == segmented_to_virtual(bhvHidden1up)
           || b == segmented_to_virtual(bhvHidden1upInPole);
}

static s32 touches(struct Object *o, Vec3f a, Vec3f b) {
    f32 r = o->hitboxRadius + DRONE_RADIUS;
    f32 bottom = o->oPosY - o->hitboxDownOffset - DRONE_RADIUS;
    f32 top = o->oPosY - o->hitboxDownOffset + o->hitboxHeight + DRONE_RADIUS;
    for (int s = 0; s <= 4; s++) {
        f32 k = s / 4.0f;
        f32 x = a[0] + (b[0] - a[0]) * k, y = a[1] + (b[1] - a[1]) * k, z = a[2] + (b[2] - a[2]) * k;
        f32 dx = x - o->oPosX, dz = z - o->oPosZ;
        if (dx * dx + dz * dz < r * r && y >= bottom && y <= top) return TRUE;
    }
    return FALSE;
}

#ifdef LIBFPV_KAMIKAZE
// --- kamikaze: the blast -------------------------------------------------------------------
// A live warhead also goes off when the drone touches an enemy. The blast
// slays every enemy within 5 m, through the game's own defeat where an attack
// has one (so its coins drop and the levels count it), and hits each boss
// once, the way the boss takes damage from Mario.

#define BLAST_RADIUS 500.0f    // from the blast to an enemy's hitbox
#define BLAST_FINISH_FRAMES 45 // enemies still fighting 1.5 s after the blast are finished off
#define MAX_BLAST_VICTIMS 64

static struct Object *sVictims[MAX_BLAST_VICTIMS];
static const BehaviorScript *sVictimBhv[MAX_BLAST_VICTIMS];
static u32 sVictimCount, sVictimTimer;
static s16 sVictimLevel, sVictimArea;

static s32 bhv_is(struct Object *o, const BehaviorScript *bhv) {
    return o->behavior == segmented_to_virtual(bhv);
}

static s32 is_boss(struct Object *o) {
    return bhv_is(o, bhvKingBobomb) || bhv_is(o, bhvWhompKingBoss) || bhv_is(o, bhvBigBully)
           || bhv_is(o, bhvBigBullyWithMinions) || bhv_is(o, bhvBigChillBully) || bhv_is(o, bhvGhostHuntBigBoo)
           || bhv_is(o, bhvMerryGoRoundBigBoo) || bhv_is(o, bhvBalconyBigBoo) || bhv_is(o, bhvWigglerHead)
           || bhv_is(o, bhvEyerokHand) || bhv_is(o, bhvBowser);
}

// enemies an attack doesn't defeat
static s32 dies_at_once(struct Object *o) {
    return bhv_is(o, bhvKoopa) || bhv_is(o, bhvSmallBully) || bhv_is(o, bhvSmallChillBully) || bhv_is(o, bhvChuckya)
           || bhv_is(o, bhvHeaveHo) || bhv_is(o, bhvCirclingAmp) || bhv_is(o, bhvHomingAmp) || bhv_is(o, bhvBubba);
}

#define MOB_INTERACT (INTERACT_BOUNCE_TOP | INTERACT_BOUNCE_TOP2 | INTERACT_HIT_FROM_BELOW | INTERACT_KOOPA \
                      | INTERACT_BULLY | INTERACT_MR_BLIZZARD | INTERACT_BREAKABLE | INTERACT_SHOCK)

static s32 is_mob(struct Object *o) {
    if (is_boss(o) || bhv_is(o, bhvWigglerBody) || bhv_is(o, bhvChainChomp) || bhv_is(o, bhvTreasureChestBottom)) {
        return FALSE;
    }
    // Koopa the Quick races Mario, he isn't fought
    if (bhv_is(o, bhvKoopa) && (o->oBehParams2ndByte == KOOPA_BP_KOOPA_THE_QUICK_BOB
                                || o->oBehParams2ndByte == KOOPA_BP_KOOPA_THE_QUICK_THI)) {
        return FALSE;
    }
    return (o->oInteractType & MOB_INTERACT) || dies_at_once(o) || bhv_is(o, bhvBobomb) || bhv_is(o, bhvSmallWhomp);
}

// from p to the object's hitbox cylinder (Whomps have none: a cylinder of their size)
static f32 blast_distance(struct Object *o, Vec3f p) {
    f32 radius = o->hitboxRadius, height = o->hitboxHeight, down = o->hitboxDownOffset;
    if (radius <= 0.0f) {
        radius = 150.0f * o->header.gfx.scale[0];
        height = 300.0f * o->header.gfx.scale[1];
        down = 0.0f;
    }
    f32 dx = p[0] - o->oPosX, dz = p[2] - o->oPosZ;
    f32 h = sqrtf(dx * dx + dz * dz) - radius;
    f32 bottom = o->oPosY - down, top = bottom + height;
    f32 v = p[1] < bottom ? bottom - p[1] : p[1] > top ? p[1] - top : 0.0f;
    if (h < 0.0f) h = 0.0f;
    return sqrtf(h * h + v * v);
}

// an enemy's death puff, with its coins
static void blast_slay(struct Object *obj) {
    struct Object *saved = gCurrentObject;
    gCurrentObject = obj;
    if (bhv_is(obj, bhvSmallBully) || bhv_is(obj, bhvSmallChillBully)) {
        // the big bully comes out once its minions are gone
        if (obj->oBullySubtype == BULLY_STYPE_MINION && obj->parentObj != NULL) {
            obj->parentObj->oBullyKBTimerAndMinionKOCounter++;
        }
        if (obj->oNumLootCoins <= 0) obj->oNumLootCoins = 1; // as from the lava
    }
    spawn_mist_particles_with_sound(SOUND_OBJ_DEFAULT_DEATH);
    obj_spawn_loot_yellow_coins(obj, obj->oNumLootCoins, 20.0f);
    obj_mark_for_deletion(obj);
    gCurrentObject = saved;
}

static void blast_mob(struct Object *obj) {
    if (bhv_is(obj, bhvBobomb)) {
        obj->oInteractStatus |= INT_STATUS_INTERACTED | INT_STATUS_TOUCHED_BOB_OMB; // it goes off too
    } else if (bhv_is(obj, bhvSmallWhomp)) {
        if (obj->oAction != 8) { // its death, with the coins of a ground pound
            obj->oNumLootCoins = 5;
            obj_spawn_loot_yellow_coins(obj, 5, 20.0f);
            obj->oAction = 8;
        }
    } else if (dies_at_once(obj)) {
        blast_slay(obj);
    } else {
        attack_object(obj, FPV_INT_KICK);
        if (sVictimCount < MAX_BLAST_VICTIMS) {
            sVictims[sVictimCount] = obj;
            sVictimBhv[sVictimCount++] = obj->behavior;
        }
    }
}

static void blast_boss(struct Object *obj, Vec3f p, f32 dist) {
    struct Object *saved = gCurrentObject;
    gCurrentObject = obj;
    s32 a = obj->oAction;
    if (bhv_is(obj, bhvKingBobomb)) {
        // as when he lands after a throw
        if ((a == 1 || a == 2) && obj->oHealth > 0) {
            obj->oHealth--;
            obj->oForwardVel = 0.0f;
            obj->oVelY = 0.0f;
            cur_obj_play_sound_2(SOUND_OBJ_KING_BOBOMB);
            obj->oAction = obj->oHealth ? 6 : 7;
        }
    } else if (bhv_is(obj, bhvWhompKingBoss)) {
        // as a ground pound on his back
        if (a >= 1 && a <= 7 && obj->oHealth > 0) {
            obj->oHealth--;
            cur_obj_play_sound_2(SOUND_OBJ2_WHOMP_SOUND_SHORT);
            cur_obj_play_sound_2(SOUND_OBJ_KING_WHOMP_DEATH);
            if (obj->oHealth == 0) {
                obj->oAction = 8;
            } else {
                spawn_mist_particles_variable(0, 0, 100.0f);
                spawn_triangle_break_particles(20, 138, 3.0f, 4);
            }
        }
    } else if (bhv_is(obj, bhvWigglerHead)) {
        // as a stomp, while he can be stomped
        if (a == WIGGLER_ACT_WALK && obj->oTimer >= 60 && obj->oHealth > 1) {
            attack_object(obj, FPV_INT_GROUND_POUND);
        }
    } else if (bhv_is(obj, bhvEyerokHand)) {
        // as a punch in the eye (eyerok_hand_check_attacked), from any side
        struct Object *boss = obj->parentObj;
        if (boss != NULL && boss->oAction == EYEROK_BOSS_ACT_FIGHT && obj->oHealth > 1 && a != EYEROK_HAND_ACT_SLEEP
            && a != EYEROK_HAND_ACT_ATTACKED && a != EYEROK_HAND_ACT_RECOVER && a != EYEROK_HAND_ACT_BECOME_ACTIVE
            && a != EYEROK_HAND_ACT_DIE) {
            cur_obj_play_sound_2(SOUND_OBJ2_EYEROK_SOUND_SHORT);
            if (--obj->oHealth >= 2) {
                obj->oAction = EYEROK_HAND_ACT_ATTACKED;
                obj->oVelY = 30.0f;
            } else {
                boss->oEyerokBossNumHands -= 1;
                obj->oAction = EYEROK_HAND_ACT_DIE;
                obj->oVelY = 50.0f;
            }
            obj->oForwardVel *= 0.2f;
            obj->oMoveAngleYaw = obj->oFaceAngleYaw + 0x8000;
            obj->oMoveFlags = 0;
            obj->oGravity = -4.0f;
            obj->oAnimState = 3;
        }
    } else if (bhv_is(obj, bhvBowser)) {
        // as one of his mines; not while he jumps onto the stage, talks, teleports or is down
        if (obj->oHealth > 0 && obj->oHeldState == HELD_FREE && a != 2 && a != 4 && a != 5 && a != 6 && a != 12
            && a != 16) {
            obj->oHealth--;
            obj->oAction = obj->oHealth <= 0 ? 4 : 12;
        }
    } else if (bhv_is(obj, bhvGhostHuntBigBoo) || bhv_is(obj, bhvMerryGoRoundBigBoo)
               || bhv_is(obj, bhvBalconyBigBoo)) {
        if (a == 1) attack_object(obj, FPV_INT_KICK); // a punch: one of his three hits
    } else if (a != BULLY_ACT_LAVA_DEATH && a != BULLY_ACT_DEATH_PLANE_DEATH) {
        // big bullies: blown away, into the lava if it's close
        obj->oMoveAngleYaw = atan2s(obj->oPosZ - p[2], obj->oPosX - p[0]);
        obj->oForwardVel = 30.0f + 70.0f * (1.0f - dist / BLAST_RADIUS);
        obj->oVelY = 30.0f;
        obj->oInteractStatus |= INT_STATUS_INTERACTED;
    }
    gCurrentObject = saved;
}

static void blast(Vec3f p) {
    sVictimTimer = 0;
    sVictimLevel = gCurrLevelNum;
    sVictimArea = gCurrAreaIndex;
    for (int i = 0; i < OBJECT_POOL_CAPACITY; i++) {
        struct Object *obj = &gObjectPool[i];
        if (!(obj->activeFlags & ACTIVE_FLAG_ACTIVE) || obj == sDroneObj || obj == gMarioObject) continue;
        s32 boss = is_boss(obj);
        if (!boss && (obj->oIntangibleTimer != 0 || !is_mob(obj))) continue;
        f32 d = blast_distance(obj, p);
        if (d > BLAST_RADIUS) continue;
        test_log("blast: %s bhv %p action %d health %d at %.0f", boss ? "boss" : "enemy", (void *) obj->behavior,
                 obj->oAction, obj->oHealth, d);
        if (boss) {
            blast_boss(obj, p, d);
        } else {
            blast_mob(obj);
        }
    }
}

// enemies the attack didn't defeat (still tangible: dying enemies aren't) are slain
static void finish_victims(void) {
    if (sVictimCount == 0) return;
    if (gCurrLevelNum != sVictimLevel || gCurrAreaIndex != sVictimArea) {
        sVictimCount = 0;
        return;
    }
    if (++sVictimTimer < BLAST_FINISH_FRAMES) return;
    for (u32 i = 0; i < sVictimCount; i++) {
        struct Object *obj = sVictims[i];
        if ((obj->activeFlags & ACTIVE_FLAG_ACTIVE) && obj->behavior == sVictimBhv[i] && obj->oIntangibleTimer == 0) {
            test_log("blast: finishing off bhv %p", (void *) obj->behavior);
            blast_slay(obj);
        }
    }
    sVictimCount = 0;
}

// a burst of the game's explosions: one at the blast, then a ring around it
static void blast_effects(u32 frame) {
    static const s8 ring[2][2][2] = { { { 1, 0 }, { -1, 0 } }, { { 0, 1 }, { 0, -1 } } };
    if (gMarioObject == NULL) return;
    if (frame == 0) {
        spawn_object_abs_with_rot(gMarioObject, 0, MODEL_EXPLOSION, bhvExplosion, sBoomPos[0], sBoomPos[1],
                                  sBoomPos[2], 0, 0, 0);
    } else if (frame == 3 || frame == 6) {
        for (int i = 0; i < 2; i++) {
            const s8 *d = ring[frame / 6][i];
            spawn_object_abs_with_rot(gMarioObject, 0, MODEL_EXPLOSION, bhvExplosion, sBoomPos[0] + d[0] * 130.0f,
                                      sBoomPos[1] + 60.0f, sBoomPos[2] + d[1] * 130.0f, 0, 0, 0);
        }
    }
}
#endif

static void drone_interactions(Vec3f a, Vec3f b, f32 speed) {
    struct MarioState *m = gMarioState;
    for (int i = 0; i < OBJECT_POOL_CAPACITY; i++) {
        struct Object *o = &gObjectPool[i];
        if (!(o->activeFlags & ACTIVE_FLAG_ACTIVE) || o == sDroneObj || o == gMarioObject) continue;
        if (o->oIntangibleTimer != 0 || (o->oInteractStatus & INT_STATUS_INTERACTED)) continue;
        u32 type = o->oInteractType;
        s32 oneUp = is_1up(o);
        s32 bobomb = o->behavior == segmented_to_virtual(bhvBobomb);
        u32 enemyTypes = INTERACT_BOUNCE_TOP | INTERACT_BOUNCE_TOP2 | INTERACT_HIT_FROM_BELOW | INTERACT_KOOPA
                         | INTERACT_BULLY | INTERACT_BREAKABLE | INTERACT_MR_BLIZZARD;
#ifdef LIBFPV_KAMIKAZE
        // contact fuse: a live warhead goes off on any enemy
        s32 target = sWarhead.warhead == FPV_WARHEAD_ARMED && (is_mob(o) || is_boss(o));
#else
        s32 target = FALSE;
#endif
        if (!(type & (INTERACT_COIN | INTERACT_STAR_OR_KEY | enemyTypes)) && !oneUp && !bobomb && !target) continue;
        if (!touches(o, a, b)) continue;
#ifdef LIBFPV_KAMIKAZE
        if (target) {
            test_log("drone hit an enemy (bhv %p) with a live warhead", (void *) o->behavior);
            fpv_kamikaze_detonate(sDrone, 0);
            return;
        }
#endif

        if (sTestLevel) test_log("drone touched object bhv %p type 0x%x at %.0f %.0f %.0f speed %.0f", (void *) o->behavior, type,
                 o->oPosX, o->oPosY, o->oPosZ, speed);
        if (type & INTERACT_COIN) {
            interact_coin(m, INTERACT_COIN, o);
        } else if (type & INTERACT_STAR_OR_KEY) {
            interact_star_or_key(m, INTERACT_STAR_OR_KEY, o);
        } else if (oneUp) {
            play_sound(SOUND_GENERAL_COLLECT_1UP, gDefaultSoundArgs);
            m->numLives++;
            o->activeFlags = ACTIVE_FLAG_DEACTIVATED;
        } else if (speed >= ATTACK_MIN_SPEED) {
            if (bobomb) {
                o->oInteractStatus = INT_STATUS_INTERACTED | INT_STATUS_TOUCHED_BOB_OMB;
            } else {
                attack_object(o, FPV_INT_KICK);
            }
            play_sound(SOUND_ACTION_HIT, sDroneObj ? sDroneObj->header.gfx.cameraToObject : gDefaultSoundArgs);
        }
    }
}

// --- lifecycle ------------------------------------------------------------------------------

void fpv_sm64_init(void) {
    if (sInited) return;
    char dir[SYS_MAX_PATH];
    snprintf(dir, sizeof(dir), "%s/fpv.log", sys_user_path());
    sTestLog = fopen(dir, "w");
    snprintf(dir, sizeof(dir), "%s/libfpv-data", sys_user_path());

    fpv_global_config cfg;
    fpv_global_config_defaults(&cfg);
    cfg.data_dir = dir;
    cfg.coords = fpv_coords_preset(FPV_COORDS_Y_UP_RIGHT_HANDED, UNITS_PER_METER);
    cfg.log = log_cb;
#ifndef RAPI_GL
    cfg.flags |= FPV_INIT_NO_LENS_DISTORTION; // only the OpenGL renderer draws the fisheye
#endif
    fpv_result res = fpv_global_init(&cfg);
    if (res != FPV_OK) {
        fprintf(stderr, "fpv: libfpv init failed: %s\n", fpv_last_error());
        test_log("libfpv init failed: %s (%s), data dir %s", fpv_result_string(res), fpv_last_error(), dir);
        return;
    }
    test_log("libfpv %s ready, data dir %s", fpv_version_string(), dir);
    fpv_material lava = { 0.1f, 0.6f, FPV_MATERIAL_KILLS_PROPS };
    fpv_material none = { 0.2f, 0.6f, FPV_MATERIAL_NO_COLLISION | FPV_MATERIAL_TRANSPARENT_TO_VIDEO };
    fpv_material water = { 0.05f, 0.9f, FPV_MATERIAL_KILLS_PROPS | FPV_MATERIAL_TRANSPARENT_TO_VIDEO };
    fpv_material_set(MAT_LAVA, &lava);
    fpv_material_set(MAT_NONE, &none);
    fpv_material_set(MAT_WATER, &water);
    fpv_audio_configure(32000, 2);
    init_drone_nodes();
    init_radio();
    sInited = TRUE;

    const char *test = getenv("SM64_FPV_TEST");
    if (test && atoi(test) > 0) {
        sTestLevel = atoi(test);
        sTestCam = strstr(test, "chase") ? CAM_CHASE : strstr(test, "los") ? CAM_LOS : CAM_FPV;
        gDebugLevelSelect = 1;
        sTestOverlay = strstr(test, "overlay") != NULL;
        sTestDive = strstr(test, "dive") != NULL;
        sTestClean = strstr(test, "clean") != NULL;
        sTestPilot = strstr(test, "pilot") != NULL;
        sTestHere = strstr(test, "here") != NULL;
        sTestFar = strstr(test, "far") != NULL;
        if (sTestPilot) sTestCam = CAM_CHASE; // no lens pass or OSD
        if (sTestPilot && strstr(test, "pilot=")) sTestPilotYaw = atof(strstr(test, "pilot=") + 6) * (3.14159265f / 180.0f);
#ifdef LIBFPV_KAMIKAZE
        sTestKamikaze = strstr(test, "kamikaze") != NULL;
        if (sTestKamikaze) {
            configFpvBoomEnding = strstr(test, "arcade") != NULL;
            fpv_kamikaze_set_installed(1); // the player would tick it in F8 > Fly
        }
#endif
        test_log("fpv test: level %d cam %d", sTestLevel, sTestCam);
    }
    printf("fpv: libfpv %s ready, data in %s\n", fpv_version_string(), dir);
}

void fpv_sm64_shutdown(void) {
    if (!sInited) return;
    fpv_global_terminate();
    sInited = FALSE;
}

static void release_keys(void) {
    for (int k = 0; k < 512; k++) {
        if (sKeysDown[k]) {
            fpv_input_key(k, 0);
            sKeysDown[k] = 0;
        }
    }
}

static void update_drone_object(void) {
    if (!sDroneObj) return;
    // rows = local axes in world space, row 3 = position (host units == SM64 units)
    static const float ax[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
    for (int j = 0; j < 3; j++) {
        float out[3];
        quat_rotate(sFrame.rotation, ax[j], out);
        sDroneMtx[j][0] = out[0];
        sDroneMtx[j][1] = out[1];
        sDroneMtx[j][2] = out[2];
        sDroneMtx[j][3] = 0.0f;
    }
    sDroneMtx[3][0] = sFrame.position[0];
    sDroneMtx[3][1] = sFrame.position[1];
    sDroneMtx[3][2] = sFrame.position[2];
    sDroneMtx[3][3] = 1.0f;
    sDroneObj->oPosX = sFrame.position[0];
    sDroneObj->oPosY = sFrame.position[1];
    sDroneObj->oPosZ = sFrame.position[2];
    // geo_process_object consumes and clears throwMatrix every frame
    sDroneObj->header.gfx.throwMatrix = &sDroneMtx;
    vec3f_set(sDroneObj->header.gfx.scale, 0.1f, 0.1f, 0.1f);
    s32 hidden = sCamMode == CAM_FPV;
#ifdef LIBFPV_KAMIKAZE
    hidden = hidden || sBoom != BOOM_NONE || (sFrame.flags & FPV_DRONE_DETONATED); // blown up
#endif
    if (hidden) {
        sDroneObj->header.gfx.node.flags |= GRAPH_RENDER_INVISIBLE;
    } else {
        sDroneObj->header.gfx.node.flags &= ~GRAPH_RENDER_INVISIBLE;
    }
    sDroneObj->header.gfx.node.flags |= GRAPH_RENDER_ACTIVE;
}

static s32 can_deploy(struct MarioState *m) {
    if (!sInited || gMarioObject == NULL || gCurrentArea == NULL || gCurrDemoInput != NULL) return FALSE;
    if (gWarpTransition.isActive || m->health < 0x100) return FALSE;
    if (m->action & (ACT_FLAG_AIR | ACT_FLAG_SWIMMING | ACT_FLAG_INTANGIBLE | ACT_FLAG_RIDING_SHELL
                     | ACT_FLAG_HANGING | ACT_FLAG_ON_POLE | ACT_FLAG_METAL_WATER)) return FALSE;
    return TRUE;
}

static void deploy(struct MarioState *m) {
    if (sLoadedTerrainGen != sTerrainGen || sLoadedTerrainLevel != gCurrLevelNum
        || sLoadedTerrainArea != gCurrAreaIndex) {
        load_static_terrain();
    }

    // on the ground a little in front of Mario, facing where he faces
    f32 yaw = m->faceAngle[1] * (3.14159265f / 32768.0f);
    f32 x = m->pos[0] + sinf(yaw) * 70.0f, z = m->pos[2] + cosf(yaw) * 70.0f;
    struct Surface *floor;
    f32 y = find_floor(x, m->pos[1] + 80.0f, z, &floor);
    if (!floor || fabsf(y - m->pos[1]) > 60.0f) {
        x = m->pos[0];
        z = m->pos[2];
        y = m->pos[1] + 170.0f; // over Mario's head
    }

    fpv_drone_desc desc;
    fpv_drone_desc_defaults(&desc);
    desc.position[0] = x;
    desc.position[1] = y + 1.0f;
    desc.position[2] = z;
    // identity faces -Z; turn it to face (sin yaw, 0, cos yaw)
    f32 a = (yaw + 3.14159265f) * 0.5f;
    desc.rotation[0] = 0.0f;
    desc.rotation[1] = sinf(a);
    desc.rotation[2] = 0.0f;
    desc.rotation[3] = cosf(a);
    desc.pilot_position[0] = m->pos[0];
    desc.pilot_position[1] = m->pos[1] + 150.0f;
    desc.pilot_position[2] = m->pos[2];

    if (fpv_drone_create(&desc, &sDrone) != FPV_OK) {
        fprintf(stderr, "fpv: drone: %s\n", fpv_last_error());
        sDrone = NULL;
        play_sound(SOUND_MENU_CAMERA_BUZZ, gDefaultSoundArgs);
        return;
    }
    memcpy(sSpawnPos, desc.position, sizeof(sSpawnPos));
    memcpy(sSpawnRot, desc.rotation, sizeof(sSpawnRot));
    // Mario is the pilot: the OSD's home arrow and distance lead back to him
    fpv_gps_set_home(m->pos);

    memset(&sFrame, 0, sizeof(sFrame));
    sFrame.struct_size = sizeof(sFrame);
    // the first frames after creation can still show the origin or the previous
    // drone (the simulation clock is global): the spawn pose and the game camera
    // until 50 ms of simulation have passed, and the model from the first real motor layout
    fpv_drone_tick(sDrone, 0.0f, &sFrame);
    sSpawnSimTime = sFrame.sim_time;
    sWarmup = 0.0f;
    memcpy(sFrame.position, desc.position, sizeof(desc.position));
    memcpy(sFrame.rotation, desc.rotation, sizeof(desc.rotation));
    sHaveFrame = FALSE;
    sModelBuilt = FALSE;
    vec3f_set(sPrevPos, desc.position[0], desc.position[1], desc.position[2]);

    sDroneObj = spawn_object(gMarioObject, MODEL_NONE, bhvStaticObject);
    if (sDroneObj) {
        sDroneObj->header.gfx.sharedChild = &sBodyNode.fnNode.node;
        sDroneObj->oFlags &= ~OBJ_FLAG_UPDATE_GFX_POS_AND_ANGLE;
    }
    create_water();
#ifdef LIBFPV_KAMIKAZE
    fpv_detonation d;
    d.struct_size = sizeof(d);
    while (fpv_kamikaze_poll_detonation(&d)) d.struct_size = sizeof(d); // the previous drone's
    memset(&sWarhead, 0, sizeof(sWarhead));
    sBoom = BOOM_NONE;
#endif

    sActive = TRUE;
    sCamMode = CAM_FPV;
    sLevel = gCurrLevelNum;
    sArea = gCurrAreaIndex;
    sLastTick = now_ticks();
    update_drone_object();
    play_sound(SOUND_MENU_CLICK_FILE_SELECT, gDefaultSoundArgs);
}

static void recall(void) {
    if (!sActive) return;
    release_keys();
    clear_solids();
    if (sWaterId) fpv_surface_object_delete(sWaterId);
    sWaterId = 0;
    if (sDrone) fpv_drone_delete(sDrone);
    sDrone = NULL;
    if (sDroneObj) {
        sDroneObj->header.gfx.throwMatrix = NULL;
        sDroneObj->header.gfx.sharedChild = NULL;
        obj_mark_for_deletion(sDroneObj);
    }
    sDroneObj = NULL;
    sActive = FALSE;
    sHaveFrame = FALSE;
    sAnchor = FALSE;
#ifdef LIBFPV_KAMIKAZE
    sBoom = BOOM_NONE;
#endif
}

s32 fpv_sm64_is_active(void) {
    return sActive;
}

s32 fpv_sm64_camera_active(void) {
    return sActive && sHaveFrame;
}

// the pilot's goggles are on screen: the FPV camera, or the static after the blast
static s32 goggles_view(void) {
    if (!sActive || !sHaveFrame) return FALSE;
#ifdef LIBFPV_KAMIKAZE
    if (sBoom != BOOM_NONE) return sBoom == BOOM_SIGNAL_LOST;
#endif
    return sCamMode == CAM_FPV;
}

s32 fpv_sm64_osd_visible(void) {
    return goggles_view();
}

s32 fpv_sm64_ui_hidden(void) {
#ifdef LIBFPV_KAMIKAZE
    // a lost feed: only the static and "SIGNAL LOST"
    if (sActive && sBoom == BOOM_SIGNAL_LOST) return TRUE;
#endif
    return fpv_sm64_osd_visible() && (sFrame.flags & FPV_DRONE_OSD_DISABLED);
}

void fpv_sm64_filter_pad(OSContPad *pad) {
    if (sTestLevel) {
        // title -> level select -> level: press Start every 40 frames until in the level
        sTestFrame++;
        // a connected but unmapped radio opens the setup wizard, which would swallow the script's input
        if (!sTestOverlay && fpv_overlay_is_open()) fpv_overlay_set_open(0);
        if (sTestLevelFrames < 0) {
            if (gCurrLevelNum != sTestLevel && sTestFrame > 60 && gCurrDemoInput == NULL) gCurrLevelNum = sTestLevel;
            if (sTestFrame > 60 && sTestFrame % 40 == 0) pad->button |= START_BUTTON;
        } else if (!sActive && sTestLevelFrames >= 45 && sTestLevelFrames % 45 == 0 && sTestDeploys < 2) {
            pad->ext_button |= FPV_BTN_TOGGLE >> FPV_BTN_SHIFT;
            sTestDeploys++;
        } else if (sActive && sTestDeploys == 1 && sTestTime > 10.0f && !sTestKamikaze) {
            // recall (or move Mario under the drone), then deploy again
            pad->ext_button |= (sTestHere ? FPV_BTN_HERE : FPV_BTN_TOGGLE) >> FPV_BTN_SHIFT;
            test_log(sTestHere ? "test: moving Mario under the drone" : "test: recalling the drone");
        }
    }
    u16 ext = pad->ext_button;
    sPadPressed |= ext & ~sPadPrev;
    sPadPrev = ext;
    if (!sInited) return;
    if (fpv_overlay_wants_input()) {
        pad->button = 0;
    } else if (sActive) {
        pad->button &= START_BUTTON; // the pause menu still works
    } else {
        return;
    }
    pad->stick_x = pad->stick_y = 0;
    pad->ext_stick_x = pad->ext_stick_y = 0;
}

static void test_update(void) {
    if (!sTestLevel) return;
    if (sTestLevelFrames < 0) {
        sTestLevelFrames = 0;
        test_log("in level %d area %d, mario at %.0f %.0f %.0f, static surfaces %d", gCurrLevelNum, gCurrAreaIndex,
                 gMarioState->pos[0], gMarioState->pos[1], gMarioState->pos[2], gNumStaticSurfaces);
    }
    sTestLevelFrames++;
    if (!sActive) return;
    if (sTestDeploys == 2 && sTestTime > 10.0f) {
        test_log("test: redeployed, drone at %.0f %.0f %.0f", sFrame.position[0], sFrame.position[1], sFrame.position[2]);
        sTestTime = 0.0f;
        sTestDeploys = 3;
    }
    sTestTime += 1.0f / 30.0f;
    sCamMode = sTestCam;
    if (sTestOverlay && sTestDeploys == 1) fpv_overlay_set_open(sTestTime > 3.0f && sTestTime < 6.0f);
    // arm, then hold ~2.5 m above the ground and cruise forward (angle mode)
    float t = sTestTime, ch[FPV_CHANNEL_COUNT] = { 0, 0, -1, 0, -1, 1, -1, -1, -1, -1 };
    for (int i = 10; i < FPV_CHANNEL_COUNT; i++) ch[i] = -1; // kamikaze builds: bomb arm and explode off
    if (t > 0.5f) ch[4] = 1;
#ifdef LIBFPV_KAMIKAZE
    // bomb arm switch on high in the air: live at once (no arming delay by default), before the Goomba
    if (sTestKamikaze && sTestDeploys == 1 && t > 4.0f) ch[10] = 1;
#endif
    if (sTestClean && t > 6.0f) ch[9] = 1;
    if (t > 2.5f) ch[1] = 0.35f;
    if (t > 1.0f) {
        // after 5 s fly low (~0.5 m) through a coin line and into a Goomba spawned ahead
        // (a live warhead goes off on the ground: 1 m with one)
        float target = (sTestDive && t > 9.0f) ? -300.0f : t > 5.0f ? (sTestKamikaze ? 100.0f : 20.0f) : 250.0f;
        float alt = sFrame.altitude < 0.0f ? target : sFrame.altitude;
        float thr = -0.45f + 0.006f * (target - alt) - 0.003f * sFrame.velocity[1];
        ch[2] = thr < -1.0f ? -1.0f : thr > 0.6f ? 0.6f : thr;
    }
    if (sTestFar && sTestDeploys == 1 && t > 2.0f && t < 2.05f) {
        float far[3] = { sSpawnPos[0] + 3000.0f, sSpawnPos[1] + 300.0f, sSpawnPos[2] };
        fpv_drone_reset(sDrone, far, sSpawnRot);
        test_log("test: drone moved 30 m out");
    }
    if (sTestFar && sTestLevelFrames % 15 == 0) {
        int near = 0;
        for (struct Object *o = (struct Object *) gObjectLists[OBJ_LIST_GENACTOR].next;
             o != (struct Object *) &gObjectLists[OBJ_LIST_GENACTOR]; o = (struct Object *) o->header.next) {
            near += o->oDistanceToMario < 2000.0f;
        }
        test_log("anchor %d, mario at %.0f %.0f %.0f, enemies within 20 m of mario's object %d", sAnchor,
                 gMarioState->pos[0], gMarioState->pos[1], gMarioState->pos[2], near);
    }
    static s32 spawned;
    if (t > 5.0f && !spawned) {
        spawned = TRUE;
        float vx = sFrame.velocity[0], vz = sFrame.velocity[2], len = sqrtf(vx * vx + vz * vz);
        if (len < 1.0f) len = 1.0f, vx = 0.0f, vz = 1.0f;
        vx /= len, vz /= len;
        for (int i = 0; i < 6; i++) {
            float d = 500.0f + i * 120.0f, x = sFrame.position[0] + vx * d, z = sFrame.position[2] + vz * d;
            struct Surface *fl;
            float y = find_floor(x, sFrame.position[1] + 200.0f, z, &fl);
            spawn_object_abs_with_rot(gMarioObject, 0, MODEL_YELLOW_COIN, bhvYellowCoin, x, y + 60.0f, z, 0, 0, 0);
        }
        float x = sFrame.position[0] + vx * 1500.0f, z = sFrame.position[2] + vz * 1500.0f;
        struct Surface *fl;
        float y = find_floor(x, sFrame.position[1] + 200.0f, z, &fl);
        struct Object *g = spawn_object_abs_with_rot(gMarioObject, 0, MODEL_GOOMBA, bhvGoomba, x, y, z, 0, 0, 0);
        g->parentObj = g; // level-placed Goombas are their own parent (no triplet spawner)
        sTestGoomba = g;
        test_log("spawned coins and goomba %p ahead at %.0f %.0f %.0f", (void *) g, x, y, z);
    }
    // keep the Goomba at flight height in the path until the drone hits it
    if (sTestGoomba) {
        struct Object *g = sTestGoomba;
        if (!(g->activeFlags & ACTIVE_FLAG_ACTIVE)) {
            test_log("goomba gone (defeated)");
            sTestGoomba = NULL;
        } else {
            if (!(g->oInteractStatus & INT_STATUS_WAS_ATTACKED) && !sTestGoombaHit) {
                g->oPosY = sFrame.position[1] - 25.0f;
                g->oVelY = 0.0f;
            } else if (!sTestGoombaHit) {
                sTestGoombaHit = TRUE;
                test_log("goomba attacked: interact status 0x%x action %d", g->oInteractStatus, g->oAction);
            }
            if (sTestGoombaHit && sTestLevelFrames % 5 == 0) test_log("goomba action %d", g->oAction);
        }
    }
    fpv_input_set_channels(ch, FPV_CHANNEL_COUNT);
    if (sTestLevelFrames % 15 == 0) {
        static u64 lastLog;
        static u32 lastDrawn;
        u64 now = now_ticks();
        if (lastLog) {
            test_log("displayed %.0f fps", (sTestDrawnFrames - lastDrawn) * (double) SDL_GetPerformanceFrequency() / (double) (now - lastLog));
        }
        lastLog = now;
        lastDrawn = sTestDrawnFrames;
        int solids = 0;
        for (int i = 0; i < MAX_SOLIDS; i++) solids += sSolids[i].id != 0;
        test_log("t=%.1f props %.0f%% %.0f%% %.0f%% %.0f%% pos %.0f %.0f %.0f speed %.0f km/h alt %.0f flags 0x%x arm_block '%s' impacts %u coins %d noise %.2f solids %d",
                 t, sFrame.motors[0].prop_damage * 100.0f, sFrame.motors[1].prop_damage * 100.0f,
                 sFrame.motors[2].prop_damage * 100.0f, sFrame.motors[3].prop_damage * 100.0f, sFrame.position[0], sFrame.position[1], sFrame.position[2], sFrame.speed_kmh, sFrame.altitude,
                 sFrame.flags, sFrame.arming_disable_text, sFrame.impact_count, gMarioState->numCoins,
                 sFrame.video.noise, solids);
#ifdef LIBFPV_KAMIKAZE
        test_log("warhead %d (%.1f s) flags 0x%x payload %.0f g of %.0f g, detonations %u, ending %d frame %u",
                 sWarhead.warhead, sWarhead.arming_remaining, sWarhead.flags, sWarhead.payload_mass * 1000.0f,
                 sWarhead.total_mass * 1000.0f, sDetonations, sBoom, sBoomFrames);
#endif
        if (sTestClean) test_log("ui hidden %d, fov %.1f (aspect %.2f) -> %.1f at %ux%u", fpv_sm64_ui_hidden(),
                                 sFrame.camera.fov_y_degrees, sFrame.camera.aspect_hint, camera_fov(&sFrame.camera),
                                 gfx_current_dimensions.width, gfx_current_dimensions.height);
    }
}

static u16 overlay_actions(void) {
    if (sActionsShown != sActive) {
        sActionsShown = sActive;
        fpv_overlay_set_action(ACTION_TOGGLE, sActive ? "Recall the drone" : "Deploy the drone");
        fpv_overlay_set_action(ACTION_RESET, sActive ? "Repair and respawn at the launch point" : NULL);
        fpv_overlay_set_action(ACTION_CAMERA, sActive ? "Camera: FPV / chase / line of sight" : NULL);
        fpv_overlay_set_action(ACTION_HERE, sActive ? "Recall the drone and move Mario under it" : NULL);
    }
#ifdef LIBFPV_KAMIKAZE
    static s32 endingShown = -1;
    if (endingShown != (s32) configFpvBoomEnding) {
        endingShown = configFpvBoomEnding;
        fpv_overlay_set_action(ACTION_ENDING, configFpvBoomEnding ? "After the explosion: arcade (click for signal lost)"
                                                                  : "After the explosion: signal lost (click for arcade)");
    }
#endif
    u16 pressed = 0;
    for (uint32_t id; (id = fpv_overlay_poll_action()) != 0;) {
        if (id == ACTION_TOGGLE) pressed |= FPV_BTN_TOGGLE >> FPV_BTN_SHIFT;
        if (id == ACTION_RESET) pressed |= FPV_BTN_RESET >> FPV_BTN_SHIFT;
        if (id == ACTION_CAMERA) pressed |= FPV_BTN_CAMERA >> FPV_BTN_SHIFT;
        if (id == ACTION_HERE) pressed |= FPV_BTN_HERE >> FPV_BTN_SHIFT;
        if (id == ACTION_ENDING) configFpvBoomEnding = !configFpvBoomEnding;
    }
    return pressed;
}

#ifdef LIBFPV_KAMIKAZE
static void boom_camera_start(void);

// detonations, and the ending after one; TRUE once Mario has taken over
static s32 kamikaze_update(void) {
    memset(&sWarhead, 0, sizeof(sWarhead));
    sWarhead.struct_size = sizeof(sWarhead);
    fpv_kamikaze_get_state(sDrone, &sWarhead);
    fpv_detonation d;
    memset(&d, 0, sizeof(d));
    d.struct_size = sizeof(d);
    while (fpv_kamikaze_poll_detonation(&d)) {
        sDetonations++;
        vec3f_set(sBoomPos, d.position[0], d.position[1], d.position[2]);
        test_log("BOOM (cause %d) at %.0f %.0f %.0f, %.0f km/h, object %u", d.cause, d.position[0], d.position[1],
                 d.position[2], sqrtf(d.velocity[0] * d.velocity[0] + d.velocity[1] * d.velocity[1]
                                      + d.velocity[2] * d.velocity[2]) * 0.036f, d.object_id);
        blast(sBoomPos);
        sBoom = configFpvBoomEnding ? BOOM_ARCADE : BOOM_SIGNAL_LOST;
        sBoomFrames = 0;
        sBoomTime = sTime;
        fpv_kamikaze_config kc;
        memset(&kc, 0, sizeof(kc));
        kc.struct_size = sizeof(kc);
        kc.ending_duration = 3.0f;
        fpv_kamikaze_get_config(&kc);
        sBoomEndFrames = (u32)(kc.ending_duration * 30.0f + 0.5f);
        if (sBoom == BOOM_ARCADE) boom_camera_start();
        d.struct_size = sizeof(d);
    }
    if (sBoom == BOOM_NONE) return FALSE;
    blast_effects(sBoomFrames);
    if (++sBoomFrames >= sBoomEndFrames) {
        if (sBoom == BOOM_ARCADE) {
            test_log("kamikaze: ending %d over, a new drone", sBoom);
            fpv_drone_repair(sDrone); // a new, safe warhead
            fpv_drone_reset(sDrone, sSpawnPos, sSpawnRot);
            sBoom = BOOM_NONE;
            return FALSE;
        }
        test_log("kamikaze: ending %d over, Mario takes over", sBoom);
        recall();
        return TRUE;
    }
    return FALSE;
}
#endif

static s32 deadly_floor(s16 type) {
    return type == SURFACE_BURNING || type == SURFACE_DEATH_PLANE || type == SURFACE_DEEP_QUICKSAND
           || type == SURFACE_INSTANT_QUICKSAND || type == SURFACE_DEEP_MOVING_QUICKSAND
           || type == SURFACE_INSTANT_MOVING_QUICKSAND;
}

// "Move Mario Here": recall the drone and put Mario on the ground under it, facing where it
// faced (gtaiv-fpv's "Move Niko here"). FALSE, and the flight goes on, without a safe floor.
static s32 move_mario_here(struct MarioState *m) {
    s32 wrecked = !sHaveFrame;
#ifdef LIBFPV_KAMIKAZE
    wrecked = wrecked || sBoom != BOOM_NONE || (sFrame.flags & FPV_DRONE_DETONATED);
#endif
    struct Surface *floor = NULL;
    f32 x = sFrame.position[0], z = sFrame.position[2];
    f32 y = wrecked ? 0.0f : find_floor(x, sFrame.position[1], z, &floor);
    if (floor == NULL || deadly_floor(floor->type)) {
        test_log("move here: no safe floor under %.0f %.0f %.0f", x, sFrame.position[1], z);
        play_sound(SOUND_MENU_CAMERA_BUZZ, gDefaultSoundArgs);
        return FALSE;
    }
    static const float ahead[3] = { 0.0f, 0.0f, -1.0f }; // the drone's nose at identity
    float fwd[3];
    quat_rotate(sFrame.rotation, ahead, fwd);
    s16 yaw = (fwd[0] * fwd[0] + fwd[2] * fwd[2] > 1e-4f) ? atan2s(fwd[2], fwd[0]) : m->faceAngle[1];
    Vec3f shift = { x - m->pos[0], y - m->pos[1], z - m->pos[2] };

    recall();
    vec3f_set(m->pos, x, y, z);
    vec3f_set(m->vel, 0.0f, 0.0f, 0.0f);
    m->forwardVel = 0.0f;
    vec3s_set(m->faceAngle, 0, yaw, 0);
    m->floor = floor;
    m->floorHeight = y;
    set_mario_action(m, ACT_IDLE, 0);
    struct Object *o = m->marioObj;
    o->oPosX = x, o->oPosY = y, o->oPosZ = z;
    vec3f_copy(o->header.gfx.pos, m->pos);
    vec3s_set(o->header.gfx.angle, 0, yaw, 0);
    o->header.gfx.prevTimestamp = 0; // no frame blended across the level
    o->header.gfx.prevShadowPosTimestamp = 0;
    warp_camera(shift[0], shift[1], shift[2]); // the camera keeps its place relative to him
    test_log("move here: Mario at %.0f %.0f %.0f yaw %d", x, y, z, yaw);
    play_sound(SOUND_MENU_CLICK_FILE_SELECT, gDefaultSoundArgs);
    return TRUE;
}

// once per game frame: the stand-in follows the drone past ANCHOR_START, until it is back
// within ANCHOR_STOP
static void update_anchor(struct MarioState *m) {
    s32 was = sAnchor;
    if (!sActive || !sHaveFrame || m->action != ACT_FPV_PILOTING) {
        sAnchor = FALSE;
    } else {
        f32 dx = sFrame.position[0] - m->pos[0], dy = sFrame.position[1] - m->pos[1];
        f32 dz = sFrame.position[2] - m->pos[2];
        f32 d = sqrtf(dx * dx + dy * dy + dz * dz);
        if (d > ANCHOR_START) sAnchor = TRUE;
        if (d < ANCHOR_STOP) sAnchor = FALSE;
    }
    if (sAnchor != was) {
        test_log("anchor: %s at %.0f %.0f %.0f", sAnchor ? "the stand-in follows the drone" : "back to Mario",
                 sFrame.position[0], sFrame.position[1], sFrame.position[2]);
    }
}

void fpv_sm64_anchor_begin(struct Object *obj) {
    struct Object *mario = gMarioObject;
    // his own effects (dust, splashes, the celebration star) stay with him
    if (!sAnchor || mario == NULL || obj == mario || obj->parentObj == mario
        || gMarioState->action != ACT_FPV_PILOTING) {
        return;
    }
    sAnchorPos[0] = mario->oPosX, sAnchorPos[1] = mario->oPosY, sAnchorPos[2] = mario->oPosZ;
    vec3f_copy(sAnchorGfxPos, mario->header.gfx.pos);
    mario->oPosX = sFrame.position[0];
    mario->oPosY = sFrame.position[1];
    mario->oPosZ = sFrame.position[2];
    vec3f_set(mario->header.gfx.pos, sFrame.position[0], sFrame.position[1], sFrame.position[2]);
    sAnchored = TRUE;
}

void fpv_sm64_anchor_end(void) {
    if (!sAnchored) return;
    struct Object *mario = gMarioObject;
    mario->oPosX = sAnchorPos[0], mario->oPosY = sAnchorPos[1], mario->oPosZ = sAnchorPos[2];
    vec3f_copy(mario->header.gfx.pos, sAnchorGfxPos);
    sAnchored = FALSE;
}

s32 fpv_sm64_anchored(void) {
    return sAnchored;
}

void fpv_sm64_update(void) {
    if (!sInited) return;
#ifdef LIBFPV_KAMIKAZE
    finish_victims();
#endif
    test_update();
    struct MarioState *m = gMarioState;
    u16 pressed = sPadPressed | overlay_actions();
    sPadPressed = 0;

    if (sActive) {
        s32 lost = gCurrLevelNum != sLevel || gCurrAreaIndex != sArea || gMarioObject == NULL
                   || sDroneObj == NULL || !(sDroneObj->activeFlags & ACTIVE_FLAG_ACTIVE);
        s32 marioBusy = m->health < 0x100 || (m->action & ACT_FLAG_INTANGIBLE) || gWarpTransition.isActive;
        if (lost) sDroneObj = NULL; // unloaded with the area
        if (lost || marioBusy || (pressed & (FPV_BTN_TOGGLE >> FPV_BTN_SHIFT))) {
            recall();
            return;
        }
    } else {
        if (pressed & (FPV_BTN_TOGGLE >> FPV_BTN_SHIFT)) {
            if (can_deploy(m)) {
                deploy(m);
            } else {
                play_sound(SOUND_MENU_CAMERA_BUZZ, gDefaultSoundArgs);
            }
        }
        if (!sActive) return;
    }

    if ((pressed & (FPV_BTN_HERE >> FPV_BTN_SHIFT)) && move_mario_here(m)) return;
    if (pressed & (FPV_BTN_RESET >> FPV_BTN_SHIFT)) {
        fpv_drone_repair(sDrone); // kamikaze builds: a new, safe warhead
        fpv_drone_reset(sDrone, sSpawnPos, sSpawnRot);
#ifdef LIBFPV_KAMIKAZE
        sBoom = BOOM_NONE; // the next drone, straight away
#endif
    }
    if (pressed & (FPV_BTN_CAMERA >> FPV_BTN_SHIFT)) {
        sCamMode = (sCamMode + 1) % CAM_COUNT;
        play_sound(SOUND_MENU_CHANGE_SELECT, gDefaultSoundArgs);
    }
#ifdef LIBFPV_KAMIKAZE
    if (kamikaze_update()) return;
#endif

    fpv_gps_set_home(m->pos); // he can still be carried by a platform
    update_anchor(m);
    sync_solids();
    update_water();
    // the flight itself is simulated once per displayed frame, in fpv_sm64_frame_tick
    sUpdateTimer = gGlobalTimer;
    update_drone_object();

    Vec3f cur = { sFrame.position[0], sFrame.position[1], sFrame.position[2] };
    f32 speed = sqrtf(sFrame.velocity[0] * sFrame.velocity[0] + sFrame.velocity[1] * sFrame.velocity[1]
                      + sFrame.velocity[2] * sFrame.velocity[2]);
    f32 dx = cur[0] - sPrevPos[0], dy = cur[1] - sPrevPos[1], dz = cur[2] - sPrevPos[2];
    s32 wrecked = FALSE;
#ifdef LIBFPV_KAMIKAZE
    wrecked = sBoom != BOOM_NONE || (sFrame.flags & FPV_DRONE_DETONATED);
#endif
    if (dx * dx + dy * dy + dz * dz < 1000.0f * 1000.0f && !wrecked) { // skip respawn teleports
        drone_interactions(sPrevPos, cur, speed);
    }
    vec3f_copy(sPrevPos, cur);

    if (sImpactSpeed > BONK_MIN_SPEED && sDroneObj) {
        play_sound(SOUND_ACTION_BONK, sDroneObj->header.gfx.cameraToObject);
    }
    sImpactSpeed = 0.0f;
}

void fpv_sm64_frame_tick(void) {
    u64 now = now_ticks();
    float dt = (float) ((double) (now - sLastTick) / (double) SDL_GetPerformanceFrequency());
    sLastTick = now;
    // only fly while the game runs (not paused, no level transition)
    if (!sActive || sDrone == NULL || sUpdateTimer + 1 != gGlobalTimer) return;
    if (dt > 0.1f) dt = 0.1f;
    sTime += dt;
    if (fpv_drone_tick(sDrone, dt, &sFrame) != FPV_OK) return;
    if (!sHaveFrame) {
        sWarmup += dt;
        if ((sFrame.sim_time - sSpawnSimTime < 0.05 && sWarmup < 0.5f)
            || (sFrame.position[0] == 0.0f && sFrame.position[1] == 0.0f && sFrame.position[2] == 0.0f)) {
            // no fresh pose yet: hold the spawn pose and keep the game camera
            memcpy(sFrame.position, sSpawnPos, sizeof(sSpawnPos));
            memcpy(sFrame.rotation, sSpawnRot, sizeof(sSpawnRot));
            sFrame.impact_count = 0;
            update_drone_object();
            return;
        }
    }
    sHaveFrame = TRUE;
    if (!sModelBuilt && (sFrame.motors[0].position[0] != 0.0f || sFrame.motors[0].position[2] != 0.0f)) {
        build_drone_model(&sFrame);
        build_motor_model();
        sModelBuilt = TRUE;
    }

    for (int i = 0; i < 4; i++) {
        sPropAngle[i] += sFrame.motors[i].rpm / 60.0f * 6.2831853f * dt * sFrame.motors[i].spin_direction;
        sPropAngle[i] = fmodf(sPropAngle[i], 6.2831853f);
    }
    for (u32 i = 0; i < sFrame.impact_count; i++) {
        if (sFrame.impacts[i].speed > sImpactSpeed) sImpactSpeed = sFrame.impacts[i].speed;
    }
    update_drone_object();
}

s32 fpv_sm64_is_drone_object(struct Object *obj) {
    return sActive && obj != NULL && obj == sDroneObj;
}

s32 fpv_sm64_drone_pose(Mat4 pose) {
    if (!sActive || !sHaveFrame || sDroneObj == NULL) return FALSE;
    mtxf_copy(pose, sDroneMtx);
    return TRUE;
}

// --- culling and rooms ---------------------------------------------------------------------

static f32 dist_to(struct Object *obj, const float *p) {
    f32 dx = obj->oPosX - p[0], dy = obj->oPosY - p[1], dz = obj->oPosZ - p[2];
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

f32 fpv_sm64_cull_distance(struct Object *obj, f32 marioDist) {
    if (!sActive || !sHaveFrame) return marioDist;
    f32 d = dist_to(obj, sFrame.position);
    if (d < marioDist) marioDist = d;
    // marioDist is to the stand-in at the drone: what is around Mario himself stays too
    if (sAnchored && (d = dist_to(obj, sAnchorPos)) < marioDist) marioDist = d;
    return marioDist;
}

s32 fpv_sm64_room_position(Vec3f pos) {
    if (!sActive || !sHaveFrame) return FALSE;
    vec3f_set(pos, sFrame.position[0], sFrame.position[1], sFrame.position[2]);
    return TRUE;
}

// --- camera ------------------------------------------------------------------------------------

// a level camera at its position, looking at target
static void look_at(fpv_camera *c, const Vec3f target) {
    f32 len = 0.0f;
    for (int i = 0; i < 3; i++) {
        c->forward[i] = target[i] - c->position[i];
        len += c->forward[i] * c->forward[i];
    }
    len = sqrtf(len);
    for (int i = 0; i < 3; i++) c->forward[i] /= len;
    // up = (right x forward) with right = forward x world up
    f32 r[3] = { -c->forward[2], 0.0f, c->forward[0] };
    c->up[0] = r[1] * c->forward[2] - r[2] * c->forward[1];
    c->up[1] = r[2] * c->forward[0] - r[0] * c->forward[2];
    c->up[2] = r[0] * c->forward[1] - r[1] * c->forward[0];
    len = sqrtf(c->up[0] * c->up[0] + c->up[1] * c->up[1] + c->up[2] * c->up[2]);
    for (int i = 0; i < 3; i++) c->up[i] /= len;
}

// self test: in front of Mario, looking at his hands
static const fpv_camera *pilot_test_camera(void) {
    static fpv_camera c;
    struct MarioState *m = gMarioState;
    f32 yaw = m->faceAngle[1] * (3.14159265f / 32768.0f) + sTestPilotYaw;
    Vec3f target = { m->pos[0], m->pos[1] + 90.0f, m->pos[2] };
    c.position[0] = target[0] + sinf(yaw) * 160.0f;
    c.position[1] = target[1] + 40.0f;
    c.position[2] = target[2] + cosf(yaw) * 160.0f;
    look_at(&c, target);
    c.fov_y_degrees = 45.0f;
    c.near_plane = 10.0f;
    return &c;
}

#ifdef LIBFPV_KAMIKAZE
// the cutscene: on the pilot's side of the blast, up to 9 m back and 3.5 m up
// (short of walls), turning slowly around it
#define BOOM_CAM_BACK 900.0f
#define BOOM_CAM_UP 350.0f

static void boom_camera_start(void) {
    struct MarioState *m = gMarioState;
    f32 dx = m->pos[0] - sBoomPos[0], dz = m->pos[2] - sBoomPos[2];
    sBoomCamYaw = dx * dx + dz * dz > 1.0f ? atan2f(dx, dz) : 0.0f;
    Vec3f orig = { sBoomPos[0], sBoomPos[1] + 50.0f, sBoomPos[2] };
    Vec3f dir = { sinf(sBoomCamYaw) * BOOM_CAM_BACK, BOOM_CAM_UP, cosf(sBoomCamYaw) * BOOM_CAM_BACK };
    struct Surface *surf = NULL;
    Vec3f hit;
    find_surface_on_ray(orig, dir, &surf, hit);
    sBoomCamReach = 1.0f;
    if (surf != NULL) {
        f32 len = sqrtf(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
        f32 hx = hit[0] - orig[0], hy = hit[1] - orig[1], hz = hit[2] - orig[2];
        sBoomCamReach = (sqrtf(hx * hx + hy * hy + hz * hz) - 40.0f) / len;
        if (sBoomCamReach < 0.25f) sBoomCamReach = 0.25f;
    }
}

static const fpv_camera *boom_camera(void) {
    static fpv_camera c;
    f32 t = sTime - sBoomTime;
    f32 yaw = sBoomCamYaw + 0.06f * t;
    c.position[0] = sBoomPos[0] + sinf(yaw) * BOOM_CAM_BACK * sBoomCamReach;
    c.position[1] = sBoomPos[1] + 50.0f + BOOM_CAM_UP * sBoomCamReach;
    c.position[2] = sBoomPos[2] + cosf(yaw) * BOOM_CAM_BACK * sBoomCamReach;
    // the blast shakes it for a moment
    f32 shake = t < 0.8f ? (0.8f - t) * 20.0f : 0.0f;
    c.position[0] += cosf(t * 47.0f) * shake;
    c.position[1] += sinf(t * 61.0f) * shake;
    Vec3f target = { sBoomPos[0], sBoomPos[1] + 60.0f, sBoomPos[2] };
    look_at(&c, target);
    c.fov_y_degrees = 50.0f;
    c.near_plane = 10.0f;
    return &c;
}
#endif

static const fpv_camera *active_camera(void) {
    if (!sActive || !sHaveFrame) return NULL;
    if (sTestPilot) return pilot_test_camera();
#ifdef LIBFPV_KAMIKAZE
    if (sBoom == BOOM_SIGNAL_LOST) return &sFrame.camera;
    if (sBoom == BOOM_ARCADE) return boom_camera();
#endif
    switch (sCamMode) {
        case CAM_CHASE: return &sFrame.chase_camera;
        case CAM_LOS: return &sFrame.los_camera;
        default: return &sFrame.camera;
    }
}

static void camera_basis(const fpv_camera *c, float r[3], float u[3], float f[3]) {
    for (int i = 0; i < 3; i++) {
        f[i] = c->forward[i];
        u[i] = c->up[i];
    }
    // right = forward x up (right-handed)
    r[0] = f[1] * u[2] - f[2] * u[1];
    r[1] = f[2] * u[0] - f[0] * u[2];
    r[2] = f[0] * u[1] - f[1] * u[0];
}

s32 fpv_sm64_view_matrix(Mat4 view, Vec3f pos, Vec3f focus) {
    const fpv_camera *c = active_camera();
    if (!c) return FALSE;
    float r[3], u[3], f[3];
    camera_basis(c, r, u, f);
    const float *p = c->position;
    for (int i = 0; i < 3; i++) {
        view[i][0] = r[i];
        view[i][1] = u[i];
        view[i][2] = -f[i];
        view[i][3] = 0.0f;
    }
    view[3][0] = -(p[0] * r[0] + p[1] * r[1] + p[2] * r[2]);
    view[3][1] = -(p[0] * u[0] + p[1] * u[1] + p[2] * u[2]);
    view[3][2] = (p[0] * f[0] + p[1] * f[1] + p[2] * f[2]);
    view[3][3] = 1.0f;
    vec3f_set(pos, p[0], p[1], p[2]);
    vec3f_set(focus, p[0] + f[0] * 100.0f, p[1] + f[1] * 100.0f, p[2] + f[2] * 100.0f);
    return TRUE;
}

// libfpv widens the FPV camera's FOV for the fisheye at its aspect_hint (4:3 analog, 16:9 HD);
// redo it for the window's aspect, so the distorted picture's top and bottom
// edges show the FOV the player chose (docs/INTEGRATION.md in libfpv)
static f32 camera_fov(const fpv_camera *c) {
    f32 fov = c->fov_y_degrees;
    float aspect = gfx_current_dimensions.height ? (float) gfx_current_dimensions.width / (float) gfx_current_dimensions.height : 0.0f;
    if (c != &sFrame.camera || (c->distortion_k1 == 0.0f && c->distortion_k2 == 0.0f) || c->aspect_hint <= 0.0f
        || aspect <= 0.0f || c->distortion_zoom <= 0.0f) {
        return fov;
    }
    float r2h = 1.0f / (1.0f + c->aspect_hint * c->aspect_hint), r2a = 1.0f / (1.0f + aspect * aspect);
    float edgeHint = 1.0f + c->distortion_k1 * r2h + c->distortion_k2 * r2h * r2h;
    float edgeReal = 1.0f + c->distortion_k1 * r2a + c->distortion_k2 * r2a * r2a;
    if (edgeReal <= 0.0f) return fov;
    // the zoom divides both edges alike
    fov = atanf(tanf(fov * (3.14159265f / 360.0f)) * edgeHint / edgeReal) * (360.0f / 3.14159265f);
    return fov > 170.0f ? 170.0f : fov;
}

s32 fpv_sm64_view_frustum(f32 *fov, s16 *near) {
    const fpv_camera *c = active_camera();
    if (!c) return FALSE;
    *fov = camera_fov(c);
    f32 n = c->near_plane;
    if (n < 4.0f) n = 4.0f;
    if (n > 50.0f) n = 50.0f;
    *near = (s16) n;
    return TRUE;
}

s32 fpv_sm64_view_rotation(Mat4 rot, f32 *fov) {
    Vec3f pos, focus;
    if (!fpv_sm64_view_matrix(rot, pos, focus)) return FALSE;
    rot[3][0] = rot[3][1] = rot[3][2] = 0.0f;
    *fov = camera_fov(active_camera());
    return TRUE;
}

// --- rendering ------------------------------------------------------------------------------

void fpv_sm64_gfx_scene_done(int width, int height) {
#ifndef RAPI_GL
    return;
#endif
    if (!goggles_view()) return;
    const fpv_camera *c = &sFrame.camera;
    float lost = 0.0f;
#ifdef LIBFPV_KAMIKAZE
    if (sBoom == BOOM_SIGNAL_LOST) lost = 1.0f; // the video transmitter went up with the drone
#endif
    gfx_fpv_gl_postprocess(width, height, c->distortion_k1, c->distortion_k2, c->distortion_zoom,
                           sFrame.video.noise, sFrame.video.breakup, sFrame.video.is_digital ? 0.0f : 1.0f, lost, sTime);
}

#ifdef RAPI_GL
void gfx_fpv_gl_save_bmp(const char *path, int width, int height);
#endif

void fpv_sm64_gfx_frame_done(int width, int height) {
#ifndef RAPI_GL
    return;
#endif
    if (!sInited) return;
    static u64 last;
    u64 now = now_ticks();
    float dt = last ? (float) ((double) (now - last) / (double) SDL_GetPerformanceFrequency()) : 1.0f / 30.0f;
    last = now;
    // (after the blast, libfpv's OSD shows "SIGNAL LOST" over the static)
    if (goggles_view()) {
        gfx_fpv_gl_draw(fpv_osd_draw_data(&sFrame, (float) width, (float) height, FPV_OSD_STYLE_ANALOG), width,
                        height);
    }
    // the game hides the cursor in fullscreen; the overlay needs it. Either way it hides after 3 s still
    s32 cursor = (fpv_overlay_is_open() || !configWindow.fullscreen) && sMouseTick &&
                 now - sMouseTick < 3 * SDL_GetPerformanceFrequency();
    if (cursor != (SDL_ShowCursor(SDL_QUERY) == SDL_ENABLE)) SDL_ShowCursor(cursor ? SDL_ENABLE : SDL_DISABLE);
    fpv_overlay_set_display((float) width, (float) height, height > 900 ? (float) height / 900.0f : 1.0f);
    gfx_fpv_gl_draw(fpv_overlay_render(dt), width, height);
#ifdef RAPI_GL
    // test runs: screenshots of the game camera, then at fixed points of the scripted flight
    static const float shots[] = { 2.0f, 5.5f, 7.0f, 9.0f };
    static int shotIdx;
    sTestDrawnFrames++;
    static int marioShot;
    if (sTestLevel && !sActive && sTestDeploys == 0 && sTestLevelFrames >= 30 && !marioShot) {
        char path[SYS_MAX_PATH];
        snprintf(path, sizeof(path), "%s/fpv_shot_mario.bmp", sys_user_path());
        gfx_fpv_gl_save_bmp(path, width, height);
        test_log("screenshot %s", path);
        marioShot = 1;
    }
#ifdef LIBFPV_KAMIKAZE
    // kamikaze test: the ending, then Mario back in control (or the new drone, arcade)
    static int boomShot, afterFrames;
    if (sTestLevel && sBoom != BOOM_NONE && boomShot < 2 && sBoomFrames >= (boomShot ? 25u : 8u)) {
        char path[SYS_MAX_PATH];
        snprintf(path, sizeof(path), "%s/fpv_shot_boom%d.bmp", sys_user_path(), boomShot++);
        gfx_fpv_gl_save_bmp(path, width, height);
        test_log("screenshot %s", path);
    }
    if (sTestLevel && sDetonations > 0 && sBoom == BOOM_NONE && afterFrames >= 0 && ++afterFrames > 5) {
        char path[SYS_MAX_PATH];
        snprintf(path, sizeof(path), "%s/fpv_shot_after.bmp", sys_user_path());
        gfx_fpv_gl_save_bmp(path, width, height);
        test_log("screenshot %s, mario action 0x%x", path, gMarioState->action);
        afterFrames = -1;
    }
#endif
    if (sTestLevel && sActive && shotIdx < 4 && sTestTime >= shots[shotIdx]) {
        char path[SYS_MAX_PATH];
        snprintf(path, sizeof(path), "%s/fpv_shot%d.bmp", sys_user_path(), shotIdx);
        gfx_fpv_gl_save_bmp(path, width, height);
        test_log("screenshot %s", path);
        shotIdx++;
    }
#endif
}

// --- audio ---------------------------------------------------------------------------------

void fpv_sm64_mix_audio(s16 *samples, u32 frames) {
    if (!sActive || frames == 0) return;
    static float buf[2048 * 2];
    if (frames > 2048) frames = 2048;
    fpv_audio_read(buf, frames);
    float vol = 32767.0f * ((float) configMasterVolume / 127.0f) * ((float) configSfxVolume / 127.0f);
    for (u32 i = 0; i < frames * 2; i++) {
        float s = samples[i] + buf[i] * vol;
        if (s > 32767.0f) s = 32767.0f;
        if (s < -32768.0f) s = -32768.0f;
        samples[i] = (s16) s;
    }
}

// --- window input --------------------------------------------------------------------------

s32 fpv_sm64_wants_keys(void) {
    return sInited && (sActive || fpv_overlay_is_open());
}

void fpv_sm64_key(int fpvKey, int down) {
    if (!sInited || fpvKey <= 0 || fpvKey >= 512) return;
    int fkey = fpvKey >= FPV_KEY_F1 && fpvKey <= FPV_KEY_F12;
    if (down && !fkey && !fpv_sm64_wants_keys()) return;
    if (!down && !sKeysDown[fpvKey] && !fkey) return;
    sKeysDown[fpvKey] = (u8) down;
    fpv_input_key(fpvKey, down);
}

void fpv_sm64_mouse_move(float x, float y) {
    sMouseTick = now_ticks();
    if (sInited) fpv_overlay_mouse_move(x, y);
}
void fpv_sm64_mouse_button(int button, int down) {
    sMouseTick = now_ticks();
    if (sInited) fpv_overlay_mouse_button(button, down);
}
void fpv_sm64_mouse_wheel(float dx, float dy) {
    sMouseTick = now_ticks();
    if (sInited) fpv_overlay_mouse_wheel(dx, dy);
}
void fpv_sm64_text(const char *utf8) {
    if (sInited && fpv_overlay_is_open()) fpv_overlay_text(utf8);
}

#endif // FPV_DRONE
