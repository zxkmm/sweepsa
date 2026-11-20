#include "persist.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sweepui {

void Persistence::layout(size_t cols, size_t rows, double x0, double x1, double y_lo,
                         double y_hi) {
    cols_ = std::max<size_t>(cols, 1);
    rows_ = std::max<size_t>(rows, 2);
    x0_ = x0;
    x1_ = x1;
    y_lo_ = y_lo;
    y_hi_ = y_hi;
    h_.assign(cols_ * rows_, 0.0);
    lo_.assign(cols_, 0.0f);
    hi_.assign(cols_, 0.0f);
    sweeps_ = 0;
}

bool Persistence::matches(size_t cols, size_t rows, double x0, double x1, double y_lo,
                          double y_hi) const {
    return cols == cols_ && rows == rows_ && x0 == x0_ && x1 == x1_ && y_lo == y_lo_ &&
           y_hi == y_hi_;
}

void Persistence::clear() {
    std::fill(h_.begin(), h_.end(), 0.0);
    sweeps_ = 0;
}

size_t Persistence::row_of(double db) const {
    const double t = (db - y_lo_) / (y_hi_ - y_lo_);
    const long r = long(std::floor(t * double(rows_)));
    return size_t(std::min<long>(std::max<long>(r, 0), long(rows_) - 1));
}

void Persistence::add(const float *db, size_t n, double x0_pts, double dx, double dt_s) {
    if (h_.empty() || n == 0 || !(x1_ > x0_) || !(dx > 0)) return;

    if (tau_s_ > 0 && dt_s > 0) {
        const double k = std::exp(-dt_s / tau_s_);
        for (double &v : h_) v *= k;
    }

    // Per-column extent of the points that land in it.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    std::fill(lo_.begin(), lo_.end(), nan);
    std::fill(hi_.begin(), hi_.end(), nan);
    const double cw = (x1_ - x0_) / double(cols_);
    for (size_t i = 0; i < n; ++i) {
        const float v = db[i];
        if (std::isnan(v)) continue;
        const double x = x0_pts + (double(i) + 0.5) * dx;
        if (x < x0_ || x >= x1_) continue;
        const size_t c = std::min(cols_ - 1, size_t((x - x0_) / cw));
        if (std::isnan(lo_[c]) || v < lo_[c]) lo_[c] = v;
        if (std::isnan(hi_[c]) || v > hi_[c]) hi_[c] = v;
    }

    // Columns with no point of their own -- narrow spans, more columns than
    // points -- are bridged from their neighbours, so the stroke is joined.
    long prev = -1;
    for (size_t c = 0; c < cols_; ++c) {
        if (std::isnan(lo_[c])) continue;
        if (prev >= 0 && size_t(prev) + 1 < c) {
            const float a = hi_[size_t(prev)], b = lo_[c];
            for (size_t g = size_t(prev) + 1; g < c; ++g) {
                const float t = float(g - size_t(prev)) / float(c - size_t(prev));
                lo_[g] = hi_[g] = a + (b - a) * t;
            }
        }
        prev = long(c);
    }

    for (size_t c = 0; c < cols_; ++c) {
        if (std::isnan(lo_[c])) continue;
        // Join to the previous column too, so a step between columns is drawn
        // as the vertical stroke it is on a joined trace.
        float lo = lo_[c], hi = hi_[c];
        if (c > 0 && !std::isnan(hi_[c - 1])) {
            lo = std::min(lo, std::min(hi, hi_[c - 1]));
            hi = std::max(hi, std::max(lo_[c], lo_[c - 1]));
        }
        const size_t r0 = row_of(lo), r1 = row_of(hi);
        for (size_t r = r0; r <= r1; ++r) h_[r * cols_ + c] += 1.0;
    }
    ++sweeps_;
}

void Persistence::intensity(std::vector<float> *out) const {
    out->assign(cols_ * rows_, 0.0f);
    double mx = 0;
    for (double v : h_) mx = std::max(mx, v);
    if (mx <= 0) return;
    // Log, so a cell hit 1% of the time is visible next to the floor that is
    // hit every time -- the whole point of the display.
    const double den = std::log1p(mx * 100.0);
    for (size_t r = 0; r < rows_; ++r) {
        const size_t top = rows_ - 1 - r;
        for (size_t c = 0; c < cols_; ++c) {
            const double v = h_[r * cols_ + c];
            if (v > 1e-6) (*out)[top * cols_ + c] = float(std::log1p(v * 100.0) / den);
        }
    }
}

}  // namespace sweepui
