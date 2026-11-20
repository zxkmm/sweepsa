// The §6 waterfall.
//
//   "Render into an OpenGL texture ring buffer with a colormap fragment shader.
//    Do not use ImPlot's heatmap -- it will not sustain 60 fps at useful history
//    depth."
//
// So: one R32F texture of dBFS, `cols` wide and `rows` tall, written one row per
// sweep at a rotating index. Nothing is ever scrolled or copied. A new sweep
// costs one glTexSubImage2D of a single row -- 8 kB at the default width -- and
// a frame costs one textured quad, regardless of how deep the history is.
//
// Colour mapping happens in the fragment shader from a 256-entry lookup texture,
// not on the CPU, for the same reason: changing the reference level or the
// colormap must not mean re-colouring 1024 stored rows. It is a uniform change
// and the whole history recolours in one frame.
//
// ------------------------------------------------------------------ on NaN
//
// The texture stores dBFS including NaN, and the shader gives NaN its own
// colour rather than clamping it into the map. §7: bins no segment covered are
// unmeasured, and a waterfall that paints them the same as the noise floor is
// asserting a measurement that was never made. Rows that have not been written
// yet get the same treatment, so a fresh window shows an honestly empty history
// instead of whatever was in the texture memory.

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "gl.h"

namespace sweepui {

enum class Colormap { Turbo, Viridis, Grayscale, ClassicGreen };

const char *colormap_name(Colormap c);

// The same maps the shader's lookup table is built from, for anything else that
// has to colour a level -- the persistence display -- so the two cannot drift.
void colormap_rgb(Colormap c, float t, float *rgb);
constexpr int kColormapCount = 4;

class Waterfall {
public:
    ~Waterfall();

    // `cols` is the fixed width of the stored history. It is not the widget
    // width and not the trace's point count: rows already written cannot be
    // re-decimated when the window is resized or the trace length changes, so
    // the history needs a width of its own. Rows are peak-decimated onto it.
    bool init(size_t cols, size_t rows, std::string *err);
    void shutdown();
    bool ready() const { return prog_ != 0; }

    // Appends one sweep. `db` is n dBFS values spanning the whole trace.
    void push(const float *db, size_t n);

    // Drops the history without reallocating. Call when the span, the point
    // count or the source changes -- the stored rows are no longer comparable
    // with the new ones, and letting them scroll away instead would leave a
    // block of the display silently mislabelled by the frequency axis.
    void clear();

    void set_colormap(Colormap c);
    Colormap colormap() const { return cmap_; }

    // Renders the visible fraction [u0,u1] of the span into an offscreen target
    // of `w` x `h` pixels and returns its texture, for ImGui::Image.
    //
    // u0/u1 come from the trace plot's x limits, which is what keeps the two
    // panels aligned when the user zooms -- §6 stacks them under a splitter, and
    // a waterfall whose axis disagrees with the trace above it is worse than no
    // waterfall.
    //
    // `ref_db` is the top of the colour range and `range_db` its extent, so the
    // mapping is the same reference-level-and-scale idea as the trace's y axis.
    unsigned render(int w, int h, double u0, double u1, float ref_db, float range_db);

    size_t rows_filled() const { return filled_; }
    size_t cols() const { return cols_; }
    size_t rows() const { return rows_; }

private:
    bool build_program(std::string *err);
    void upload_lut();
    bool ensure_target(int w, int h);

    size_t cols_ = 0;
    size_t rows_ = 0;
    size_t write_ = 0;     // next row to write; the ring's head
    size_t filled_ = 0;

    gl::UInt data_tex_ = 0;
    gl::UInt lut_tex_ = 0;
    gl::UInt prog_ = 0;
    gl::UInt vao_ = 0;
    gl::UInt vbo_ = 0;

    gl::UInt fbo_ = 0;
    gl::UInt fbo_tex_ = 0;
    int fbo_w_ = 0;
    int fbo_h_ = 0;

    Colormap cmap_ = Colormap::Turbo;
    std::vector<float> row_;   // scratch for the decimated row
};

}  // namespace sweepui
