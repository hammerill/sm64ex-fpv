#if defined(FPV_DRONE) && defined(RAPI_GL)

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef __MINGW32__
# define FOR_WINDOWS 1
#else
# define FOR_WINDOWS 0
#endif

#if FOR_WINDOWS
# define GLEW_STATIC
# include <GL/glew.h>
#endif

#define GL_GLEXT_PROTOTYPES 1
#include <SDL2/SDL.h>
#ifdef USE_GLES
# include <SDL2/SDL_opengles2.h>
#else
# include <SDL2/SDL_opengl.h>
#endif

#include "gfx_fpv_gl.h"

#ifdef USE_GLES
# define GLSL_HEADER "#version 100\nprecision mediump float;\n"
#else
# define GLSL_HEADER "#version 120\n"
#endif

// --- state save/restore ---------------------------------------------------------

struct SavedState {
    GLint program, arrayBuffer, elementBuffer, activeTexture, texture0;
    GLint viewport[4], scissor[4];
    GLboolean depthTest, blend, scissorTest, cullFace, polyOffset, depthMask;
    GLint blendSrc, blendDst;
};

static void save_state(struct SavedState *s) {
    glGetIntegerv(GL_CURRENT_PROGRAM, &s->program);
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &s->arrayBuffer);
    glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, &s->elementBuffer);
    glGetIntegerv(GL_ACTIVE_TEXTURE, &s->activeTexture);
    glActiveTexture(GL_TEXTURE0);
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &s->texture0);
    glGetIntegerv(GL_VIEWPORT, s->viewport);
    glGetIntegerv(GL_SCISSOR_BOX, s->scissor);
    s->depthTest = glIsEnabled(GL_DEPTH_TEST);
    s->blend = glIsEnabled(GL_BLEND);
    s->scissorTest = glIsEnabled(GL_SCISSOR_TEST);
    s->cullFace = glIsEnabled(GL_CULL_FACE);
    s->polyOffset = glIsEnabled(GL_POLYGON_OFFSET_FILL);
    glGetBooleanv(GL_DEPTH_WRITEMASK, &s->depthMask);
    glGetIntegerv(GL_BLEND_SRC_RGB, &s->blendSrc);
    glGetIntegerv(GL_BLEND_DST_RGB, &s->blendDst);
}

static void set_enabled(GLenum cap, GLboolean on) {
    if (on) glEnable(cap); else glDisable(cap);
}

static void restore_state(const struct SavedState *s) {
    glUseProgram(s->program);
    glBindBuffer(GL_ARRAY_BUFFER, s->arrayBuffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, s->elementBuffer);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, s->texture0);
    glActiveTexture(s->activeTexture);
    glViewport(s->viewport[0], s->viewport[1], s->viewport[2], s->viewport[3]);
    glScissor(s->scissor[0], s->scissor[1], s->scissor[2], s->scissor[3]);
    set_enabled(GL_DEPTH_TEST, s->depthTest);
    set_enabled(GL_BLEND, s->blend);
    set_enabled(GL_SCISSOR_TEST, s->scissorTest);
    set_enabled(GL_CULL_FACE, s->cullFace);
    set_enabled(GL_POLYGON_OFFSET_FILL, s->polyOffset);
    glDepthMask(s->depthMask);
    glBlendFunc(s->blendSrc, s->blendDst);
}

// --- shaders --------------------------------------------------------------------

static GLuint compile(GLenum type, const char *src) {
    GLuint sh = glCreateShader(type);
    glShaderSource(sh, 1, &src, NULL);
    glCompileShader(sh);
    GLint ok = 0;
    glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(sh, sizeof(log), NULL, log);
        fprintf(stderr, "fpv: shader compilation failed: %s\n", log);
    }
    return sh;
}

static GLuint link(const char *vs, const char *fs, const char *attr0, const char *attr1, const char *attr2) {
    GLuint p = glCreateProgram();
    glAttachShader(p, compile(GL_VERTEX_SHADER, vs));
    glAttachShader(p, compile(GL_FRAGMENT_SHADER, fs));
    glBindAttribLocation(p, 0, attr0);
    if (attr1) glBindAttribLocation(p, 1, attr1);
    if (attr2) glBindAttribLocation(p, 2, attr2);
    glLinkProgram(p);
    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog(p, sizeof(log), NULL, log);
        fprintf(stderr, "fpv: shader link failed: %s\n", log);
    }
    return p;
}

static const char *sUiVs = GLSL_HEADER
    "attribute vec2 aPos; attribute vec2 aUv; attribute vec4 aCol;\n"
    "uniform vec2 uScreen; varying vec2 vUv; varying vec4 vCol;\n"
    "void main() { vUv = aUv; vCol = aCol;\n"
    "  gl_Position = vec4(aPos.x / uScreen.x * 2.0 - 1.0, 1.0 - aPos.y / uScreen.y * 2.0, 0.0, 1.0); }\n";
static const char *sUiFs = GLSL_HEADER
    "uniform sampler2D uTex; varying vec2 vUv; varying vec4 vCol;\n"
    "void main() { gl_FragColor = vCol * texture2D(uTex, vUv); }\n";

// Brown-Conrady lens distortion and analog video noise (libfpv docs/INTEGRATION.md)
static const char *sPostVs = GLSL_HEADER
    "attribute vec2 aPos; varying vec2 uv;\n"
    "void main() { uv = aPos * 0.5 + 0.5; gl_Position = vec4(aPos, 0.0, 1.0); }\n";
static const char *sPostFs = GLSL_HEADER
    "varying vec2 uv; uniform sampler2D scene; uniform vec2 k; uniform float zoom; uniform float aspect;\n"
    "uniform float noise; uniform float breakup; uniform float time; uniform float analog;\n"
    "float hash(vec2 p) { return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453); }\n"
    "void main() {\n"
    "  vec2 scale = vec2(aspect, 1.0) / (0.5 * sqrt(aspect * aspect + 1.0));\n"
    "  vec2 c = (uv - 0.5) * scale;\n"
    "  float r2 = dot(c, c);\n"
    "  vec2 d = c * (1.0 + k.x * r2 + k.y * r2 * r2) / zoom;\n"
    "  vec2 suv = d / scale + 0.5;\n"
    "  if (breakup > 0.0) {\n"
    "    float line = step(1.0 - breakup * 0.25, hash(vec2(floor(uv.y * 120.0), floor(time * 20.0))));\n"
    "    suv.x += line * (hash(vec2(time, uv.y)) - 0.5) * 0.15;\n"
    "  }\n"
    "  vec3 col = (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) ? vec3(0.0) : texture2D(scene, suv).rgb;\n"
    "  float n = hash(uv * 513.0 + time) - 0.5;\n"
    "  col += n * noise * 0.9 * analog;\n"
    "  col = mix(col, vec3(dot(col, vec3(0.3, 0.59, 0.11))), noise * 0.6 * analog);\n"
    "  gl_FragColor = vec4(col, 1.0);\n"
    "}\n";

static bool sInited;
static GLuint sUiProg, sPostProg, sUiVbo, sUiIbo, sPostVbo, sSceneTex;
static GLint sUiScreen, sUiTex;
static GLint sPostScene, sPostK, sPostZoom, sPostAspect, sPostNoise, sPostBreakup, sPostTime, sPostAnalog;
static int sSceneW, sSceneH;

struct FontTex {
    uint64_t id;
    GLuint tex;
    uint32_t revision;
    bool valid;
};
static struct FontTex sFonts[2] = { { FPV_TEXTURE_OVERLAY_FONT, 0, 0, false }, { FPV_TEXTURE_OSD_FONT, 0, 0, false } };

static void init(void) {
    if (sInited) return;
    sInited = true;
    sUiProg = link(sUiVs, sUiFs, "aPos", "aUv", "aCol");
    sUiScreen = glGetUniformLocation(sUiProg, "uScreen");
    sUiTex = glGetUniformLocation(sUiProg, "uTex");
    sPostProg = link(sPostVs, sPostFs, "aPos", NULL, NULL);
    sPostScene = glGetUniformLocation(sPostProg, "scene");
    sPostK = glGetUniformLocation(sPostProg, "k");
    sPostZoom = glGetUniformLocation(sPostProg, "zoom");
    sPostAspect = glGetUniformLocation(sPostProg, "aspect");
    sPostNoise = glGetUniformLocation(sPostProg, "noise");
    sPostBreakup = glGetUniformLocation(sPostProg, "breakup");
    sPostTime = glGetUniformLocation(sPostProg, "time");
    sPostAnalog = glGetUniformLocation(sPostProg, "analog");
    glGenBuffers(1, &sUiVbo);
    glGenBuffers(1, &sUiIbo);
    glGenBuffers(1, &sPostVbo);
    static const float tri[6] = { -1.0f, -1.0f, 3.0f, -1.0f, -1.0f, 3.0f };
    glBindBuffer(GL_ARRAY_BUFFER, sPostVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(tri), tri, GL_STATIC_DRAW);
    glGenTextures(1, &sSceneTex);
    for (int i = 0; i < 2; i++) glGenTextures(1, &sFonts[i].tex);
}

// the game's shader programs may have any vertex attribute enabled; ours use 0..2
static GLboolean sAttribWasEnabled[3];
static void begin_attribs(int count) {
    for (int i = 0; i < 3; i++) {
        GLint on = 0;
        glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &on);
        sAttribWasEnabled[i] = on != 0;
        if (i < count) glEnableVertexAttribArray(i); else glDisableVertexAttribArray(i);
    }
}
static void end_attribs(void) {
    // the game re-specifies its attribute pointers when it next loads a shader
    for (int i = 0; i < 3; i++) {
        if (sAttribWasEnabled[i]) glEnableVertexAttribArray(i); else glDisableVertexAttribArray(i);
    }
}

void gfx_fpv_gl_postprocess(int width, int height, float k1, float k2, float zoom, float noise,
                            float breakup, float analog, float time) {
    if (width <= 0 || height <= 0) return;
    struct SavedState st;
    save_state(&st);
    init();

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, sSceneTex);
    if (width != sSceneW || height != sSceneH) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        sSceneW = width;
        sSceneH = height;
    }
    glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, width, height);

    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glDisable(GL_POLYGON_OFFSET_FILL);
    glDepthMask(GL_FALSE);
    glViewport(0, 0, width, height);

    glUseProgram(sPostProg);
    glUniform1i(sPostScene, 0);
    glUniform2f(sPostK, k1, k2);
    glUniform1f(sPostZoom, zoom > 0.0f ? zoom : 1.0f);
    glUniform1f(sPostAspect, (float) width / (float) height);
    glUniform1f(sPostNoise, noise);
    glUniform1f(sPostBreakup, breakup);
    glUniform1f(sPostTime, time);
    glUniform1f(sPostAnalog, analog);

    glBindBuffer(GL_ARRAY_BUFFER, sPostVbo);
    begin_attribs(1);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void *) 0);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    end_attribs();

    restore_state(&st);
}

static GLuint font_texture(uint64_t id) {
    for (int i = 0; i < 2; i++) {
        struct FontTex *f = &sFonts[i];
        if (f->id != id) continue;
        fpv_texture t;
        if (fpv_texture_get(id, &t) == FPV_OK && t.rgba && (!f->valid || t.revision != f->revision)) {
            glBindTexture(GL_TEXTURE_2D, f->tex);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, t.width, t.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, t.rgba);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            f->revision = t.revision;
            f->valid = true;
        }
        return f->tex;
    }
    return 0;
}

// test helper: writes the back buffer as a 24-bit BMP
void gfx_fpv_gl_save_bmp(const char *path, int width, int height) {
    int stride = (width * 3 + 3) & ~3;
    unsigned char *px = malloc((size_t) stride * height);
    if (!px) return;
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    glReadPixels(0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, px);
    FILE *f = fopen(path, "wb");
    if (f) {
        unsigned int size = 54 + stride * height;
        unsigned char h[54] = { 'B', 'M' };
        h[2] = size; h[3] = size >> 8; h[4] = size >> 16; h[5] = size >> 24;
        h[10] = 54; h[14] = 40;
        h[18] = width; h[19] = width >> 8; h[22] = height; h[23] = height >> 8;
        h[26] = 1; h[28] = 24;
        fwrite(h, 1, 54, f);
        fwrite(px, 1, (size_t) stride * height, f);
        fclose(f);
    }
    free(px);
}

void gfx_fpv_gl_draw(const fpv_draw_data *data, int width, int height) {
    if (!data || data->list_count == 0 || width <= 0 || height <= 0) return;
    if (data->display_size[0] <= 0.0f || data->display_size[1] <= 0.0f) return;
    struct SavedState st;
    save_state(&st);
    init();

    const float sx = (float) width / data->display_size[0];
    const float sy = (float) height / data->display_size[1];

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_POLYGON_OFFSET_FILL);
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_SCISSOR_TEST);
    glViewport(0, 0, width, height);

    glUseProgram(sUiProg);
    glUniform2f(sUiScreen, data->display_size[0], data->display_size[1]);
    glUniform1i(sUiTex, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindBuffer(GL_ARRAY_BUFFER, sUiVbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sUiIbo);
    begin_attribs(3);

    const GLsizei stride = sizeof(fpv_draw_vertex);
    for (uint32_t l = 0; l < data->list_count; l++) {
        const fpv_draw_list *list = &data->lists[l];
        if (list->vertex_count == 0 || list->index_count == 0) continue;
        glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr) list->vertex_count * stride, list->vertices, GL_STREAM_DRAW);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr) list->index_count * sizeof(uint32_t), list->indices,
                     GL_STREAM_DRAW);
        for (uint32_t c = 0; c < list->command_count; c++) {
            const fpv_draw_cmd *cmd = &list->commands[c];
            if (cmd->index_count == 0) continue;
            const float *r = cmd->clip_rect;
            int x0 = (int) (r[0] * sx), y0 = (int) (r[1] * sy), x1 = (int) (r[2] * sx), y1 = (int) (r[3] * sy);
            if (x1 <= x0 || y1 <= y0) continue;
            glScissor(x0, height - y1, x1 - x0, y1 - y0);
            glBindTexture(GL_TEXTURE_2D, font_texture(cmd->texture_id));
            // GL 2.1 has no base vertex: offset the attribute pointers instead
            const size_t base = (size_t) cmd->vertex_offset * stride;
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, stride, (void *) (base + 0));
            glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (void *) (base + 8));
            glVertexAttribPointer(2, 4, GL_UNSIGNED_BYTE, GL_TRUE, stride, (void *) (base + 16));
            glDrawElements(GL_TRIANGLES, cmd->index_count, GL_UNSIGNED_INT,
                           (void *) ((size_t) cmd->index_offset * sizeof(uint32_t)));
        }
    }

    end_attribs();
    restore_state(&st);
}

#endif
