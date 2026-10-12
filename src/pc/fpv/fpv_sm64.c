// FPV drone mode for sm64ex, built on libfpv (Betaflight in the loop).
//
// Mario deploys the drone with the "Drone Toggle" bind and stands still as its
// pilot: the line-of-sight camera and the video link are measured from him.
// The drone collides with the level (static surfaces exported once per area,
// object collision models as moving libfpv surface objects, water and lava
// kill the props), collects coins, stars and 1-ups, and knocks out enemies it
// hits at speed. World units: 100 per metre, so Mario is about 1.6 m tall.

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
#include "game/camera.h"
#include "game/game_init.h"
#include "game/interaction.h"
#include "game/level_update.h"
#include "game/main.h"
#include "game/mario.h"
#include "game/memory.h"
#include "game/object_helpers.h"
#include "game/object_list_processor.h"
#include "behavior_data.h"
#include "model_ids.h"
#include "surface_terrains.h"
#include "object_constants.h"
#include "audio_defines.h"
#include "pc/configfile.h"
#include "pc/platform.h"
#include "pc/controller/controller_api.h"
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
#define FPV_INT_KICK (1 << 2) // INT_KICK in interaction.c

#define UNITS_PER_METER 100.0f
#define DRONE_RADIUS 14.0f        // collection/attack reach around the frame centre
#define ATTACK_MIN_SPEED 200.0f   // units/s, 2 m/s
#define BONK_MIN_SPEED 300.0f
#define MAT_DEFAULT 0
#define MAT_LAVA 1
#define MAT_NONE 2
#define MAT_WATER 3

enum CamMode { CAM_FPV, CAM_CHASE, CAM_LOS, CAM_COUNT };

static s32 sInited;
static s32 sActive;
static fpv_drone *sDrone;
static fpv_frame sFrame;
static s32 sHaveFrame;
static s32 sCamMode;
static u16 sPadPrev, sPadPressed;
static u64 sLastTick;
static float sTime;

static s16 sLevel = -1, sArea = -1;
static float sSpawnPos[3], sSpawnRot[4];
static Vec3f sPrevPos;

static u32 sTerrainGen = 1, sLoadedTerrainGen;
static s16 sLoadedTerrainLevel = -1, sLoadedTerrainArea = -1;
static u32 sWaterId;
static float sWaterLevel = -20000.0f;

static struct Object *sDroneObj;
static Mat4 sDroneMtx;

static u8 sKeysDown[512];

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

static void test_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
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
        if (!(type & (INTERACT_COIN | INTERACT_STAR_OR_KEY | enemyTypes)) && !oneUp && !bobomb) continue;
        if (!touches(o, a, b)) continue;

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
    sInited = TRUE;

    const char *test = getenv("SM64_FPV_TEST");
    if (test && atoi(test) > 0) {
        sTestLevel = atoi(test);
        sTestCam = strstr(test, "chase") ? CAM_CHASE : strstr(test, "los") ? CAM_LOS : CAM_FPV;
        gDebugLevelSelect = 1;
        sTestOverlay = strstr(test, "overlay") != NULL;
        sTestDive = strstr(test, "dive") != NULL;
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
    if (sCamMode == CAM_FPV) {
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

    memset(&sFrame, 0, sizeof(sFrame));
    sFrame.struct_size = sizeof(sFrame);
    // the first frames after creation can still be empty: pose from the spawn
    // until the simulation reports, and the model from the first real motor layout
    fpv_drone_tick(sDrone, 0.0f, &sFrame);
    memcpy(sFrame.position, desc.position, sizeof(desc.position));
    memcpy(sFrame.rotation, desc.rotation, sizeof(desc.rotation));
    sHaveFrame = TRUE;
    sModelBuilt = FALSE;
    vec3f_set(sPrevPos, desc.position[0], desc.position[1], desc.position[2]);

    sDroneObj = spawn_object(gMarioObject, MODEL_NONE, bhvStaticObject);
    if (sDroneObj) {
        sDroneObj->header.gfx.sharedChild = &sBodyNode.fnNode.node;
        sDroneObj->oFlags &= ~OBJ_FLAG_UPDATE_GFX_POS_AND_ANGLE;
    }
    create_water();

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
}

s32 fpv_sm64_is_active(void) {
    return sActive;
}

s32 fpv_sm64_camera_active(void) {
    return sActive && sHaveFrame;
}

s32 fpv_sm64_osd_visible(void) {
    return sActive && sHaveFrame && sCamMode == CAM_FPV;
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
        } else if (sActive && sTestDeploys == 1 && sTestTime > 10.0f) {
            pad->ext_button |= FPV_BTN_TOGGLE >> FPV_BTN_SHIFT; // recall, then deploy again
            test_log("test: recalling the drone");
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
    float t = sTestTime, ch[8] = { 0, 0, -1, 0, -1, 1, -1, -1 };
    if (t > 0.5f) ch[4] = 1;
    if (t > 2.5f) ch[1] = 0.35f;
    if (t > 1.0f) {
        // after 5 s fly low (~0.5 m) through a coin line and into a Goomba spawned ahead
        float target = (sTestDive && t > 9.0f) ? -300.0f : t > 5.0f ? 20.0f : 250.0f;
        float alt = sFrame.altitude < 0.0f ? target : sFrame.altitude;
        float thr = -0.45f + 0.006f * (target - alt) - 0.003f * sFrame.velocity[1];
        ch[2] = thr < -1.0f ? -1.0f : thr > 0.6f ? 0.6f : thr;
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
    fpv_input_set_channels(ch, 8);
    if (sTestLevelFrames % 15 == 0) {
        int solids = 0;
        for (int i = 0; i < MAX_SOLIDS; i++) solids += sSolids[i].id != 0;
        test_log("t=%.1f props %.0f%% %.0f%% %.0f%% %.0f%% pos %.0f %.0f %.0f speed %.0f km/h alt %.0f flags 0x%x arm_block '%s' impacts %u coins %d noise %.2f solids %d",
                 t, sFrame.motors[0].prop_damage * 100.0f, sFrame.motors[1].prop_damage * 100.0f,
                 sFrame.motors[2].prop_damage * 100.0f, sFrame.motors[3].prop_damage * 100.0f, sFrame.position[0], sFrame.position[1], sFrame.position[2], sFrame.speed_kmh, sFrame.altitude,
                 sFrame.flags, sFrame.arming_disable_text, sFrame.impact_count, gMarioState->numCoins,
                 sFrame.video.noise, solids);
    }
}

void fpv_sm64_update(void) {
    if (!sInited) return;
    test_update();
    struct MarioState *m = gMarioState;
    u16 pressed = sPadPressed;
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

    if (pressed & (FPV_BTN_RESET >> FPV_BTN_SHIFT)) {
        fpv_drone_repair(sDrone);
        fpv_drone_reset(sDrone, sSpawnPos, sSpawnRot);
    }
    if (pressed & (FPV_BTN_CAMERA >> FPV_BTN_SHIFT)) {
        sCamMode = (sCamMode + 1) % CAM_COUNT;
        play_sound(SOUND_MENU_CHANGE_SELECT, gDefaultSoundArgs);
    }

    sync_solids();
    update_water();

    u64 now = now_ticks();
    float dt = (float) ((double) (now - sLastTick) / (double) SDL_GetPerformanceFrequency());
    sLastTick = now;
    if (dt > 0.1f) dt = 0.1f;
    sTime += dt;
    if (fpv_drone_tick(sDrone, dt, &sFrame) != FPV_OK) return;
    if (sFrame.position[0] == 0.0f && sFrame.position[1] == 0.0f && sFrame.position[2] == 0.0f) {
        // no pose reported yet: hold the spawn pose
        memcpy(sFrame.position, sSpawnPos, sizeof(sSpawnPos));
        memcpy(sFrame.rotation, sSpawnRot, sizeof(sSpawnRot));
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
    update_drone_object();

    Vec3f cur = { sFrame.position[0], sFrame.position[1], sFrame.position[2] };
    f32 speed = sqrtf(sFrame.velocity[0] * sFrame.velocity[0] + sFrame.velocity[1] * sFrame.velocity[1]
                      + sFrame.velocity[2] * sFrame.velocity[2]);
    f32 dx = cur[0] - sPrevPos[0], dy = cur[1] - sPrevPos[1], dz = cur[2] - sPrevPos[2];
    if (dx * dx + dy * dy + dz * dz < 1000.0f * 1000.0f) { // skip respawn teleports
        drone_interactions(sPrevPos, cur, speed);
    }
    vec3f_copy(sPrevPos, cur);

    for (u32 i = 0; i < sFrame.impact_count; i++) {
        if (sFrame.impacts[i].speed > BONK_MIN_SPEED && sDroneObj) {
            play_sound(SOUND_ACTION_BONK, sDroneObj->header.gfx.cameraToObject);
            break;
        }
    }
}

// --- culling and rooms ---------------------------------------------------------------------

f32 fpv_sm64_cull_distance(struct Object *obj, f32 marioDist) {
    if (!sActive || !sHaveFrame) return marioDist;
    f32 dx = obj->oPosX - sFrame.position[0];
    f32 dy = obj->oPosY - sFrame.position[1];
    f32 dz = obj->oPosZ - sFrame.position[2];
    f32 d = sqrtf(dx * dx + dy * dy + dz * dz);
    return d < marioDist ? d : marioDist;
}

s32 fpv_sm64_room_position(Vec3f pos) {
    if (!sActive || !sHaveFrame) return FALSE;
    vec3f_set(pos, sFrame.position[0], sFrame.position[1], sFrame.position[2]);
    return TRUE;
}

// --- camera ------------------------------------------------------------------------------------

static const fpv_camera *active_camera(void) {
    if (!sActive || !sHaveFrame) return NULL;
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

s32 fpv_sm64_view_frustum(f32 *fov, s16 *near) {
    const fpv_camera *c = active_camera();
    if (!c) return FALSE;
    *fov = c->fov_y_degrees;
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
    *fov = active_camera()->fov_y_degrees;
    return TRUE;
}

// --- rendering ------------------------------------------------------------------------------

void fpv_sm64_gfx_scene_done(int width, int height) {
#ifndef RAPI_GL
    return;
#endif
    if (!sActive || !sHaveFrame || sCamMode != CAM_FPV) return;
    const fpv_camera *c = &sFrame.camera;
    gfx_fpv_gl_postprocess(width, height, c->distortion_k1, c->distortion_k2, c->distortion_zoom,
                           sFrame.video.noise, sFrame.video.breakup, sFrame.video.is_digital ? 0.0f : 1.0f, sTime);
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
    if (sActive && sHaveFrame && sCamMode == CAM_FPV) {
        gfx_fpv_gl_draw(fpv_osd_draw_data(&sFrame, (float) width, (float) height, FPV_OSD_STYLE_ANALOG), width,
                        height);
    }
    // the game hides the cursor in fullscreen; the overlay needs it
    static s32 cursorForOverlay;
    s32 open = fpv_overlay_is_open();
    if (open != cursorForOverlay) {
        SDL_ShowCursor(open || !configWindow.fullscreen ? SDL_ENABLE : SDL_DISABLE);
        cursorForOverlay = open;
    }
    fpv_overlay_set_display((float) width, (float) height, height > 900 ? (float) height / 900.0f : 1.0f);
    gfx_fpv_gl_draw(fpv_overlay_render(dt), width, height);
#ifdef RAPI_GL
    // test runs: screenshots at fixed points of the scripted flight
    static const float shots[] = { 2.0f, 5.5f, 7.0f, 9.0f };
    static int shotIdx;
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
    if (sInited) fpv_overlay_mouse_move(x, y);
}
void fpv_sm64_mouse_button(int button, int down) {
    if (sInited) fpv_overlay_mouse_button(button, down);
}
void fpv_sm64_mouse_wheel(float dx, float dy) {
    if (sInited) fpv_overlay_mouse_wheel(dx, dy);
}
void fpv_sm64_text(const char *utf8) {
    if (sInited && fpv_overlay_is_open()) fpv_overlay_text(utf8);
}

#endif // FPV_DRONE
