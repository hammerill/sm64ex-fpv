#ifndef GFX_FPV_GL_H
#define GFX_FPV_GL_H

// OpenGL 2.1 drawing for the FPV drone: the lens/analog-video post-process on
// the 3D scene and libfpv's OSD and overlay draw lists. Every function saves
// and restores the GL state the game's renderer relies on.

#include <stdbool.h>
#include <libfpv.h>

// distorts the current back buffer contents (width x height pixels) in place;
// lost (0..1) fades the picture to the static of a receiver with no signal
void gfx_fpv_gl_postprocess(int width, int height, float k1, float k2, float zoom, float noise,
                            float breakup, float analog, float lost, float time);
// the goggles from frame->video_feed (libfpv's fpv_video_gl2.h): the back buffer through the
// lens, the analog noise or digital breakup and the camera's resolution and aspect, and the OSD;
// false (nothing drawn) when the renderer can't run
bool gfx_fpv_gl_video(const fpv_frame *frame, int width, int height);
// draws libfpv draw data (positions in display_size units) over a width x height pixel framebuffer
void gfx_fpv_gl_draw(const fpv_draw_data *data, int width, int height);

#endif
