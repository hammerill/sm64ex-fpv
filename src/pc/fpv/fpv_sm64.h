#ifndef FPV_SM64_H
#define FPV_SM64_H

// FPV drone mode: Mario deploys a Betaflight-flown quadcopter (libfpv) and
// stands still as its pilot while the player flies it.

#include <PR/ultratypes.h>
#include <PR/os_cont.h>
#include "types.h"

#ifdef FPV_DRONE

void fpv_sm64_init(void);
void fpv_sm64_shutdown(void);

// controller_entry_point: drone buttons, and blocks Mario's input while flying
void fpv_sm64_filter_pad(OSContPad *pad);
// level_update (PLAY_MODE_NORMAL), once per game frame
void fpv_sm64_update(void);
// pc_main, before drawing each displayed frame: simulates the flight up to now
void fpv_sm64_frame_tick(void);
// surface_load
void fpv_sm64_terrain_loaded(void);
void fpv_sm64_object_collision_loaded(struct Object *obj, Mat4 transform);

s32 fpv_sm64_is_active(void);
// the FPV camera (with its Betaflight OSD) is on screen
s32 fpv_sm64_osd_visible(void);
// any drone camera (FPV, chase, line of sight) is on screen
s32 fpv_sm64_camera_active(void);
// distance used for object drawing/collision culling: the closer of Mario and the drone
f32 fpv_sm64_cull_distance(struct Object *obj, f32 marioDist);
// room selection follows the drone; returns TRUE and the position when flying
s32 fpv_sm64_room_position(Vec3f pos);

// rendering_graph_node: replaces the game camera while a drone camera is active
s32 fpv_sm64_view_matrix(Mat4 view, Vec3f pos, Vec3f focus);
s32 fpv_sm64_view_frustum(f32 *fov, s16 *near);
// skybox: a sphere around the camera instead of the scrolling backdrop (handles roll and flips)
s32 fpv_sm64_view_rotation(Mat4 rot, f32 *fov);
// frame interpolation: the drone object, and its live world transform
s32 fpv_sm64_is_drone_object(struct Object *obj);
s32 fpv_sm64_drone_pose(Mat4 pose);

// gfx_pc (renderer thread = main thread): after the 3D scene, and at the end of the frame
void fpv_sm64_gfx_scene_done(int width, int height);
void fpv_sm64_gfx_frame_done(int width, int height);

// pc_main: adds motor sound to the game's 32 kHz stereo output
void fpv_sm64_mix_audio(s16 *samples, u32 frames);

// window events (keys forwarded as libfpv key codes, see libfpv.h fpv_key)
void fpv_sm64_key(int fpvKey, int down);
s32 fpv_sm64_wants_keys(void);
void fpv_sm64_mouse_move(float x, float y);
void fpv_sm64_mouse_button(int button, int down);
void fpv_sm64_mouse_wheel(float dx, float dy);
void fpv_sm64_text(const char *utf8);

// display list marker emitted between the 3D scene and the HUD
#define FPV_GFX_SCENE_DONE_TAG 0x46505631 /* 'FPV1' */

#endif // FPV_DRONE
#endif // FPV_SM64_H
