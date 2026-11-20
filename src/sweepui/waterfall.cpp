#include "waterfall.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "decimate.h"

namespace sweepui {
namespace {

const char *kVertexSrc = R"(#version 330 core
layout(location = 0) in vec2 aPos;
out vec2 vUv;
void main() {
    // aPos is the unit square. v runs 0 at the top so row 0 of the render is
    // the newest sweep, which is the direction every spectrum analyzer
    // waterfall scrolls.
    vUv = vec2(aPos.x, 1.0 - aPos.y);
    gl_Position = vec4(aPos * 2.0 - 1.0, 0.0, 1.0);
}
)";

const char *kFragmentSrc = R"(#version 330 core
in vec2 vUv;
out vec4 fragColor;

uniform sampler2D uData;    // cols x rows, R32F, dBFS, NEAREST
uniform sampler2D uLut;     // 256 x 1, RGBA8, LINEAR
uniform float uRows;        // history depth in rows
uniform float uWrite;       // ring head: the row the next sweep will occupy
uniform float uFilled;      // rows actually written since the last clear
uniform float uLines;       // history lines covering the widget's height
uniform vec2  uU;           // visible fraction of the span, from the trace axis
uniform float uRef;         // dBFS at the top of the colour range
uniform float uRange;       // dB covered by the colour range

const vec4 kNoData = vec4(0.10, 0.10, 0.13, 1.0);

void main() {
    float line = floor(vUv.y * uLines);      // 0 = newest
    if (line >= uFilled) {                   // history that does not exist yet
        fragColor = kNoData;
        return;
    }

    // The ring, resolved in the shader so nothing is ever scrolled in memory.
    // +2*uRows keeps the argument of mod positive for every reachable line.
    float row = mod(uWrite - 1.0 - line + 2.0 * uRows, uRows);
    float v = (row + 0.5) / uRows;
    float u = mix(uU.x, uU.y, vUv.x);

    float db = texture(uData, vec2(u, v)).r;
    if (isnan(db)) {                         // unmeasured, and it stays visible
        fragColor = kNoData;
        return;
    }

    float t = clamp((db - (uRef - uRange)) / uRange, 0.0, 1.0);
    fragColor = vec4(texture(uLut, vec2(t, 0.5)).rgb, 1.0);
}
)";

// Turbo, from Anton Mikhailov's published polynomial approximation of the
// Google colormap. Evaluated on the CPU into a 256-entry table rather than in
// the shader, because every colormap here has to arrive the same way and the
// others are tables to begin with.
void turbo(float t, float *rgb) {
    const float x = std::min(std::max(t, 0.0f), 1.0f);
    const float x2 = x * x, x3 = x2 * x, x4 = x3 * x, x5 = x4 * x;
    rgb[0] = 0.13572138f + 4.61539260f * x - 42.66032258f * x2 + 132.13108234f * x3 -
             152.94239396f * x4 + 59.28637943f * x5;
    rgb[1] = 0.09140261f + 2.19418839f * x + 4.84296658f * x2 - 14.18503333f * x3 +
             4.27729857f * x4 + 2.82956604f * x5;
    rgb[2] = 0.10667330f + 12.64194608f * x - 60.58204836f * x2 + 110.36276771f * x3 -
             89.90310912f * x4 + 27.34824973f * x5;
    for (int i = 0; i < 3; ++i) rgb[i] = std::min(std::max(rgb[i], 0.0f), 1.0f);
}

// Viridis, as nine control points from the reference table with linear
// interpolation between them. Nine points is enough that the difference from
// the 256-entry original is below what an 8-bit channel can show, and it does
// not put a 768-value table in the source.
void viridis(float t, float *rgb) {
    static const float k[9][3] = {
        {0.267004f, 0.004874f, 0.329415f}, {0.282623f, 0.140926f, 0.457517f},
        {0.253935f, 0.265254f, 0.529983f}, {0.206756f, 0.371758f, 0.553117f},
        {0.163625f, 0.471133f, 0.558148f}, {0.127568f, 0.566949f, 0.550556f},
        {0.134692f, 0.658636f, 0.517649f}, {0.266941f, 0.748751f, 0.440573f},
        {0.477504f, 0.821444f, 0.318195f},
    };
    const float x = std::min(std::max(t, 0.0f), 1.0f) * 8.0f;
    const int i = std::min(int(x), 7);
    const float f = x - float(i);
    for (int c = 0; c < 3; ++c) rgb[c] = k[i][c] + (k[i + 1][c] - k[i][c]) * f;
    // The reference map's last stop is yellow; fold it in at the very top so
    // strong signals stay distinguishable from merely loud ones.
    if (t > 0.94f) {
        const float g = (t - 0.94f) / 0.06f;
        const float top[3] = {0.993248f, 0.906157f, 0.143936f};
        for (int c = 0; c < 3; ++c) rgb[c] += (top[c] - rgb[c]) * g;
    }
}

void grayscale(float t, float *rgb) {
    const float x = std::min(std::max(t, 0.0f), 1.0f);
    rgb[0] = rgb[1] = rgb[2] = x;
}

// The CRT-phosphor look of an analogue analyzer: black to green, whitening only
// at the very top so the brightest returns still read as brighter.
void classic_green(float t, float *rgb) {
    const float x = std::min(std::max(t, 0.0f), 1.0f);
    rgb[0] = std::max(0.0f, (x - 0.75f) / 0.25f);
    rgb[1] = std::pow(x, 0.75f);
    rgb[2] = std::max(0.0f, (x - 0.85f) / 0.15f) * 0.8f;
}

gl::UInt compile(gl::Enum type, const char *src, std::string *err) {
    const gl::UInt s = gl::CreateShader(type);
    gl::ShaderSource(s, 1, &src, nullptr);
    gl::CompileShader(s);
    gl::Int ok = 0;
    gl::GetShaderiv(s, gl::kCompileStatus, &ok);
    if (!ok) {
        char log[1024] = {0};
        gl::GetShaderInfoLog(s, sizeof log - 1, nullptr, log);
        *err = log;
        gl::DeleteShader(s);
        return 0;
    }
    return s;
}

}  // namespace

void colormap_rgb(Colormap c, float t, float *rgb) {
    switch (c) {
        case Colormap::Turbo: turbo(t, rgb); break;
        case Colormap::Viridis: viridis(t, rgb); break;
        case Colormap::Grayscale: grayscale(t, rgb); break;
        case Colormap::ClassicGreen: classic_green(t, rgb); break;
    }
}

const char *colormap_name(Colormap c) {
    switch (c) {
        case Colormap::Turbo: return "turbo";
        case Colormap::Viridis: return "viridis";
        case Colormap::Grayscale: return "grayscale";
        case Colormap::ClassicGreen: return "classic green";
    }
    return "?";
}

Waterfall::~Waterfall() { shutdown(); }

bool Waterfall::init(size_t cols, size_t rows, std::string *err) {
    shutdown();

    // The history is a texture, so it is bounded by what the driver will
    // allocate. §6 asks for 1024 lines at 32k points; 32k exceeds the guaranteed
    // minimum texture size and would fail on some drivers, so the width is
    // clamped and rows are peak-decimated onto it. The trace plot above is not
    // affected -- it decimates from the full-resolution frame every time.
    gl::Int max_tex = 2048;
    gl::GetIntegerv(gl::kMaxTextureSize, &max_tex);
    const size_t cap = size_t(std::max(gl::Int(256), max_tex));
    cols_ = std::min(std::max<size_t>(cols, 64), cap);
    rows_ = std::min(std::max<size_t>(rows, 16), cap);

    if (!build_program(err)) {
        shutdown();
        return false;
    }

    gl::GenTextures(1, &data_tex_);
    gl::BindTexture(gl::kTexture2D, data_tex_);
    // NEAREST, deliberately: linear filtering would average dBFS values, and
    // averaging dB is the operation §5 spends a paragraph forbidding. It would
    // also smear NaN into its neighbours and destroy the holes.
    gl::TexParameteri(gl::kTexture2D, gl::kTextureMinFilter, gl::kNearest);
    gl::TexParameteri(gl::kTexture2D, gl::kTextureMagFilter, gl::kNearest);
    gl::TexParameteri(gl::kTexture2D, gl::kTextureWrapS, gl::kClampToEdge);
    gl::TexParameteri(gl::kTexture2D, gl::kTextureWrapT, gl::kClampToEdge);

    const std::vector<float> blank(cols_, std::nanf(""));
    std::vector<float> init_rows;
    init_rows.reserve(cols_ * rows_);
    for (size_t r = 0; r < rows_; ++r)
        init_rows.insert(init_rows.end(), blank.begin(), blank.end());
    gl::PixelStorei(gl::kUnpackAlignment, 1);
    gl::TexImage2D(gl::kTexture2D, 0, gl::Int(gl::kR32F), gl::Sizei(cols_), gl::Sizei(rows_),
                   0, gl::kRed, gl::kFloat, init_rows.data());

    gl::GenTextures(1, &lut_tex_);
    gl::BindTexture(gl::kTexture2D, lut_tex_);
    gl::TexParameteri(gl::kTexture2D, gl::kTextureMinFilter, gl::kLinear);
    gl::TexParameteri(gl::kTexture2D, gl::kTextureMagFilter, gl::kLinear);
    gl::TexParameteri(gl::kTexture2D, gl::kTextureWrapS, gl::kClampToEdge);
    gl::TexParameteri(gl::kTexture2D, gl::kTextureWrapT, gl::kClampToEdge);
    upload_lut();

    // A unit-square triangle strip. The whole render is one quad; there is
    // nothing else to draw.
    const float verts[8] = {0, 0, 1, 0, 0, 1, 1, 1};
    gl::GenVertexArrays(1, &vao_);
    gl::BindVertexArray(vao_);
    gl::GenBuffers(1, &vbo_);
    gl::BindBuffer(gl::kArrayBuffer, vbo_);
    gl::BufferData(gl::kArrayBuffer, sizeof verts, verts, gl::kStaticDraw);
    gl::EnableVertexAttribArray(0);
    gl::VertexAttribPointer(0, 2, gl::kFloat, 0, 2 * sizeof(float), nullptr);
    gl::BindVertexArray(0);

    write_ = 0;
    filled_ = 0;
    row_.assign(cols_, std::nanf(""));
    return true;
}

bool Waterfall::build_program(std::string *err) {
    std::string verr, ferr;
    const gl::UInt vs = compile(gl::kVertexShader, kVertexSrc, &verr);
    if (!vs) {
        *err = "waterfall vertex shader: " + verr;
        return false;
    }
    const gl::UInt fs = compile(gl::kFragmentShader, kFragmentSrc, &ferr);
    if (!fs) {
        gl::DeleteShader(vs);
        *err = "waterfall fragment shader: " + ferr;
        return false;
    }

    prog_ = gl::CreateProgram();
    gl::AttachShader(prog_, vs);
    gl::AttachShader(prog_, fs);
    gl::LinkProgram(prog_);
    gl::DeleteShader(vs);
    gl::DeleteShader(fs);

    gl::Int ok = 0;
    gl::GetProgramiv(prog_, gl::kLinkStatus, &ok);
    if (!ok) {
        char log[1024] = {0};
        gl::GetProgramInfoLog(prog_, sizeof log - 1, nullptr, log);
        *err = std::string("waterfall link: ") + log;
        gl::DeleteProgram(prog_);
        prog_ = 0;
        return false;
    }
    return true;
}

void Waterfall::shutdown() {
    if (fbo_) gl::DeleteFramebuffers(1, &fbo_);
    if (fbo_tex_) gl::DeleteTextures(1, &fbo_tex_);
    if (data_tex_) gl::DeleteTextures(1, &data_tex_);
    if (lut_tex_) gl::DeleteTextures(1, &lut_tex_);
    if (vbo_) gl::DeleteBuffers(1, &vbo_);
    if (vao_) gl::DeleteVertexArrays(1, &vao_);
    if (prog_) gl::DeleteProgram(prog_);
    fbo_ = fbo_tex_ = data_tex_ = lut_tex_ = vbo_ = vao_ = prog_ = 0;
    fbo_w_ = fbo_h_ = 0;
    write_ = filled_ = 0;
}

void Waterfall::set_colormap(Colormap c) {
    if (c == cmap_) return;
    cmap_ = c;
    if (lut_tex_) upload_lut();
}

void Waterfall::upload_lut() {
    unsigned char px[256 * 4];
    for (int i = 0; i < 256; ++i) {
        const float t = float(i) / 255.0f;
        float rgb[3] = {0, 0, 0};
        colormap_rgb(cmap_, t, rgb);
        for (int c = 0; c < 3; ++c)
            px[i * 4 + c] = (unsigned char)std::lround(std::min(1.0f, std::max(0.0f, rgb[c])) * 255.0f);
        px[i * 4 + 3] = 255;
    }
    gl::BindTexture(gl::kTexture2D, lut_tex_);
    gl::PixelStorei(gl::kUnpackAlignment, 1);
    gl::TexImage2D(gl::kTexture2D, 0, gl::Int(gl::kRgba8), 256, 1, 0, gl::kRgba,
                   gl::kUnsignedByte, px);
}

void Waterfall::push(const float *db, size_t n) {
    if (!data_tex_ || cols_ == 0) return;

    row_.assign(cols_, std::nanf(""));
    decimate_peak(db, n, row_.data(), cols_);

    gl::BindTexture(gl::kTexture2D, data_tex_);
    gl::PixelStorei(gl::kUnpackAlignment, 1);
    gl::TexSubImage2D(gl::kTexture2D, 0, 0, gl::Int(write_), gl::Sizei(cols_), 1, gl::kRed,
                      gl::kFloat, row_.data());

    write_ = (write_ + 1) % rows_;
    if (filled_ < rows_) ++filled_;
}

void Waterfall::clear() {
    write_ = 0;
    filled_ = 0;
}

bool Waterfall::ensure_target(int w, int h) {
    if (w <= 0 || h <= 0) return false;
    if (fbo_ && w == fbo_w_ && h == fbo_h_) return true;

    if (fbo_tex_) gl::DeleteTextures(1, &fbo_tex_);
    if (fbo_) gl::DeleteFramebuffers(1, &fbo_);

    gl::GenTextures(1, &fbo_tex_);
    gl::BindTexture(gl::kTexture2D, fbo_tex_);
    gl::TexParameteri(gl::kTexture2D, gl::kTextureMinFilter, gl::kLinear);
    gl::TexParameteri(gl::kTexture2D, gl::kTextureMagFilter, gl::kLinear);
    gl::TexParameteri(gl::kTexture2D, gl::kTextureWrapS, gl::kClampToEdge);
    gl::TexParameteri(gl::kTexture2D, gl::kTextureWrapT, gl::kClampToEdge);
    gl::TexImage2D(gl::kTexture2D, 0, gl::Int(gl::kRgba8), w, h, 0, gl::kRgba,
                   gl::kUnsignedByte, nullptr);

    gl::GenFramebuffers(1, &fbo_);
    gl::BindFramebuffer(gl::kFramebuffer, fbo_);
    gl::FramebufferTexture2D(gl::kFramebuffer, gl::kColorAttachment0, gl::kTexture2D,
                             fbo_tex_, 0);
    const bool ok = gl::CheckFramebufferStatus(gl::kFramebuffer) == gl::kFramebufferComplete;
    gl::BindFramebuffer(gl::kFramebuffer, 0);
    if (!ok) {
        gl::DeleteFramebuffers(1, &fbo_);
        gl::DeleteTextures(1, &fbo_tex_);
        fbo_ = fbo_tex_ = 0;
        return false;
    }
    fbo_w_ = w;
    fbo_h_ = h;
    return true;
}

unsigned Waterfall::render(int w, int h, double u0, double u1, float ref_db,
                           float range_db) {
    if (!prog_ || !ensure_target(w, h)) return 0;
    if (!(range_db > 0)) range_db = 1.0f;

    // The caller is mid-frame and ImGui owns the GL state, so both the bound
    // framebuffer and the viewport are restored before returning. Leaving
    // either changed shows up as the rest of the UI drawing into the wrong
    // target, which looks like a UI bug rather than a waterfall bug.
    gl::Int prev_fbo = 0, prev_vp[4] = {0, 0, 0, 0};
    gl::GetIntegerv(gl::kFramebufferBinding, &prev_fbo);
    gl::GetIntegerv(gl::kViewportState, prev_vp);

    gl::BindFramebuffer(gl::kFramebuffer, fbo_);
    gl::Viewport(0, 0, w, h);
    gl::Disable(gl::kBlend);
    gl::Disable(gl::kScissorTest);
    gl::Disable(gl::kDepthTest);
    gl::Disable(gl::kCullFace);

    gl::UseProgram(prog_);
    gl::ActiveTexture(gl::kTexture0);
    gl::BindTexture(gl::kTexture2D, data_tex_);
    gl::ActiveTexture(gl::kTexture1);
    gl::BindTexture(gl::kTexture2D, lut_tex_);
    gl::ActiveTexture(gl::kTexture0);

    gl::Uniform1i(gl::GetUniformLocation(prog_, "uData"), 0);
    gl::Uniform1i(gl::GetUniformLocation(prog_, "uLut"), 1);
    gl::Uniform1f(gl::GetUniformLocation(prog_, "uRows"), float(rows_));
    gl::Uniform1f(gl::GetUniformLocation(prog_, "uWrite"), float(write_));
    gl::Uniform1f(gl::GetUniformLocation(prog_, "uFilled"), float(filled_));
    // One history line per pixel row: fewer would throw sweeps away, more would
    // repeat them. §6's "1024 waterfall lines" is a history depth, and it is
    // visible in full only when the panel is that tall.
    gl::Uniform1f(gl::GetUniformLocation(prog_, "uLines"), float(h));
    gl::Uniform2f(gl::GetUniformLocation(prog_, "uU"), float(u0), float(u1));
    gl::Uniform1f(gl::GetUniformLocation(prog_, "uRef"), ref_db);
    gl::Uniform1f(gl::GetUniformLocation(prog_, "uRange"), range_db);

    gl::BindVertexArray(vao_);
    gl::DrawArrays(gl::kTriangleStrip, 0, 4);
    gl::BindVertexArray(0);
    gl::UseProgram(0);

    gl::BindFramebuffer(gl::kFramebuffer, gl::UInt(prev_fbo));
    gl::Viewport(prev_vp[0], prev_vp[1], prev_vp[2], prev_vp[3]);
    return fbo_tex_;
}

}  // namespace sweepui
