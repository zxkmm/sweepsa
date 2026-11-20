#include "decimate.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sweepui {

void decimate_minmax(const float *db, size_t n, double f_start_hz, double bin_hz,
                     double x0_hz, double x1_hz, size_t max_cols,
                     DecimatedTrace *out) {
    out->clear();
    if (!db || n == 0 || max_cols == 0 || !(bin_hz > 0)) return;
    if (!(x1_hz > x0_hz)) return;

    // Visible bin range, clamped to the trace: every bin whose *span* --
    // [f_start + i*bin, f_start + (i+1)*bin) -- intersects the view, which
    // includes the two partly-visible bins at the edges. Selecting by bin
    // centre instead would drop them, and the trace would appear to shrink away
    // from the axis as you pan.
    //
    // floor and ceil are symmetric here. An extra +1 on one side would widen
    // the view by one bin at one edge only, which at 1:1 zoom shows up as an
    // off-by-one between the axis and the data.
    long long i0 = (long long)std::floor((x0_hz - f_start_hz) / bin_hz);
    long long i1 = (long long)std::ceil((x1_hz - f_start_hz) / bin_hz);
    i0 = std::max<long long>(i0, 0);
    i1 = std::min<long long>(i1, (long long)n);
    if (i1 <= i0) return;

    const size_t vis = size_t(i1 - i0);
    const size_t cols = std::min(max_cols, vis);

    out->x.reserve(cols);
    out->lo.reserve(cols);
    out->hi.reserve(cols);

    for (size_t c = 0; c < cols; ++c) {
        // Integer-proportional split, so every visible bin lands in exactly one
        // column and none is counted twice. Rounding the boundaries in floating
        // point would occasionally drop or duplicate a bin, which shows up as a
        // one-pixel notch that moves when you pan.
        const size_t b0 = size_t(i0) + (vis * c) / cols;
        const size_t b1 = size_t(i0) + (vis * (c + 1)) / cols;
        if (b1 <= b0) continue;

        double lo = std::numeric_limits<double>::infinity();
        double hi = -std::numeric_limits<double>::infinity();
        for (size_t b = b0; b < b1; ++b) {
            const float v = db[b];
            if (std::isnan(v)) continue;   // see the NaN note in decimate.h
            if (v < lo) lo = v;
            if (v > hi) hi = v;
        }

        out->x.push_back(f_start_hz + (double(b0) + 0.5) * bin_hz);
        if (hi < lo) {
            // Every bin in this column was NaN: unmeasured, and it stays that
            // way. ImPlot breaks the line here instead of drawing through it.
            const double nan = std::numeric_limits<double>::quiet_NaN();
            out->lo.push_back(nan);
            out->hi.push_back(nan);
        } else {
            out->lo.push_back(lo);
            out->hi.push_back(hi);
        }
    }
}

void decimate_peak(const float *db, size_t n, float *out, size_t n_out) {
    if (!out || n_out == 0) return;
    const float nan = std::numeric_limits<float>::quiet_NaN();

    if (!db || n == 0) {
        std::fill(out, out + n_out, nan);
        return;
    }

    for (size_t c = 0; c < n_out; ++c) {
        const size_t b0 = (n * c) / n_out;
        size_t b1 = (n * (c + 1)) / n_out;

        // Upsampling: fewer source bins than columns, so some columns would be
        // empty. Give them the bin they fall inside rather than a hole -- a
        // stretched row is honest at this zoom, an interleaved comb of NaN is
        // not.
        if (b1 <= b0) b1 = std::min(b0 + 1, n);

        float hi = -std::numeric_limits<float>::infinity();
        for (size_t b = b0; b < b1; ++b) {
            const float v = db[b];
            if (std::isnan(v)) continue;
            if (v > hi) hi = v;
        }
        out[c] = std::isinf(hi) ? nan : hi;
    }
}

}  // namespace sweepui
