// The persistence / density display. CLAUDE.md §6 lists it as the waterfall's
// alternative; SATSAGEN calls its version DPD ("digital phosphor").
//
// A histogram over (frequency, level): every sweep lights the cells its trace
// passes through, and older hits fade. Where the waterfall answers "when", this
// answers "how often at what level" -- a burst that is there 5% of the time
// shows as a dim layer over the floor instead of vanishing behind an average or
// dominating a max hold.
//
// ---------------------------------------------------------------- the strokes
//
// A sweep lights each column from the lowest to the highest level among the
// points that fall in it, and joins each column to the next. That is the trace
// drawn with every point joined, as a phosphor screen would draw it; lighting
// only the points would leave a dotted picture at narrow spans, where there are
// fewer points than columns.
//
// ------------------------------------------------------------------- decay
//
// Exponential with a time constant in seconds of wall clock rather than in
// sweeps, so the picture means the same thing whether the source runs at 0.5 or
// 50 sweeps/s. tau <= 0 is infinite persistence: pure accumulation.
//
// The grid is fixed when it is laid out -- the trace's span and the plot's
// level range -- because a hit cannot be moved to a new cell after the fact.
// Changing either clears it, and the caller says so rather than showing a
// histogram whose axes no longer match the ones drawn around it.
//
// NaN points (unmeasured) light nothing.

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace sweepui {

class Persistence {
public:
    // cols x rows cells over x [x0, x1) and level [y_lo, y_hi). Clears.
    void layout(size_t cols, size_t rows, double x0, double x1, double y_lo, double y_hi);
    bool matches(size_t cols, size_t rows, double x0, double x1, double y_lo,
                 double y_hi) const;
    void clear();

    // One sweep of n points, point i at x0_pts + (i + 0.5) * dx. `dt_s` is the
    // time since the previous add, for the decay.
    void add(const float *db, size_t n, double x0_pts, double dx, double dt_s);

    void set_tau(double tau_s) { tau_s_ = tau_s; }
    double tau() const { return tau_s_; }

    // Colour-ready intensity per cell, row 0 at the TOP (highest level), in
    // [0, 1]: log-scaled hits relative to the busiest cell, 0 where none.
    void intensity(std::vector<float> *out) const;

    size_t cols() const { return cols_; }
    double x0() const { return x0_; }
    double x1() const { return x1_; }
    double y_lo() const { return y_lo_; }
    double y_hi() const { return y_hi_; }
    size_t rows() const { return rows_; }
    double hits(size_t col, size_t row_from_bottom) const {
        return h_[row_from_bottom * cols_ + col];
    }
    uint64_t sweeps() const { return sweeps_; }

private:
    size_t row_of(double db) const;

    size_t cols_ = 0, rows_ = 0;
    double x0_ = 0, x1_ = 0, y_lo_ = 0, y_hi_ = 0;
    double tau_s_ = 2.0;
    std::vector<double> h_;        // row-major, row 0 = y_lo
    std::vector<float> lo_, hi_;   // per-column scratch
    uint64_t sweeps_ = 0;
};

}  // namespace sweepui
