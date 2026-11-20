// Display decimation. CLAUDE.md §6, "Performance":
//
//   "Decimate for display (min/max per pixel column) -- never hand ImPlot 32k
//    raw points."
//
// This is the one piece of the UI that can be wrong in a way that looks right:
// a decimator that samples instead of taking extremes will drop a narrow tone
// entirely at some zoom levels and show it at others, and the user will read
// that as the signal coming and going. Min/max per column is what keeps a
// one-bin spur visible when 40 bins share a pixel.
//
// No ImGui, no GL, no source dependency, so tests/decimate_test.cpp exercises it
// natively -- which is the only practical way to test it, because the failure on
// screen is a plausible-looking trace.
//
// ------------------------------------------------------------------ on NaN
//
// proto.h: the trace carries NaN for bins no segment covered, and §7 wants those
// visibly missing rather than interpolated into something plausible. So NaN is
// not "no data to compare" here -- it is data, and it propagates:
//
//   * a column with at least one real value takes the min/max of the real ones
//     and ignores the NaNs (the hole is narrower than the pixel; showing the
//     signal that shares the pixel is honest)
//   * a column with no real values at all emits NaN in both min and max, which
//     ImPlot renders as a gap in the line rather than as a dip to zero
//
// The second case is the one that matters: dc_policy=blank leaves a 200 kHz hole
// per segment, and at full span those holes are narrower than a pixel. They only
// become visible when zoomed in, which is exactly when they should be.

#pragma once

#include <cstddef>
#include <vector>

namespace sweepui {

// A decimated trace: one x, one min and one max per output column.
//
// x is the bin-centre frequency of the column's first source bin, in Hz. The
// column spans several bins, so any single x is a choice; the left edge is the
// one that keeps the plotted envelope from drifting right as the decimation
// factor grows.
// Doubles, including the dB, even though the frame carries float: ImPlot's
// plotting calls take one type for both axes, and the x axis has to be double.
// A 6 GHz frequency in float32 resolves to about 512 Hz, which would put a
// marker most of an RBW away from the bin it is reading.
struct DecimatedTrace {
    std::vector<double> x;
    std::vector<double> lo;
    std::vector<double> hi;

    size_t size() const { return x.size(); }
    void clear() {
        x.clear();
        lo.clear();
        hi.clear();
    }
};

// Reduces `db[0..n)` -- spanning [f_start_hz, f_start_hz + n*bin_hz) -- onto at
// most `max_cols` columns covering the visible x range [x0_hz, x1_hz].
//
// Only bins inside the visible range are considered, so zooming in genuinely
// increases resolution instead of just stretching the same decimation. When the
// visible range holds fewer bins than `max_cols`, the output is one column per
// bin and lo == hi: no data is invented to fill the extra pixels.
//
// `max_cols` should be the plot's pixel width. Passing more only costs time --
// ImPlot cannot draw two line segments in one pixel column distinguishably.
void decimate_minmax(const float *db, size_t n, double f_start_hz, double bin_hz,
                     double x0_hz, double x1_hz, size_t max_cols,
                     DecimatedTrace *out);

// Peak-decimates `db[0..n)` onto exactly `n_out` columns covering the whole
// trace, writing NaN where a column had no real value.
//
// This is the waterfall's row reduction, not the trace's: the waterfall texture
// has a fixed width, every row must be the same width, and history cannot be
// re-decimated when the user zooms. Peak rather than min/max because a row is
// one pixel tall and can only show one value, and a spectrum analyzer that
// hides a spur to show the noise floor next to it has the priority backwards.
void decimate_peak(const float *db, size_t n, float *out, size_t n_out);

}  // namespace sweepui
