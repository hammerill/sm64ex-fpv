#ifndef GFX_FPV_GL_H
#define GFX_FPV_GL_H

// OpenGL 2.1 drawing for the FPV drone: the lens/analog-video post-process on
// the 3D scene and libfpv's OSD and overlay draw lists. Every function saves
// and restores the GL state the game's renderer relies on.

#include <libfpv.h>

// distorts the current back buffer contents (width x height pixels) in place
void gfx_fpv_gl_postprocess(int width, int height, float k1, float k2, float zoom, float noise,
                            float breakup, float analog, float time);
// draws libfpv draw data (positions in display_size units) over a width x height pixel framebuffer
void gfx_fpv_gl_draw(const fpv_draw_data *data, int width, int height);

#endif
