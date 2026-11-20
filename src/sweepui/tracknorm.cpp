#include "tracknorm.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace sweepui {

bool TrackNorm::same_layout(const TrackGrid &a, const TrackGrid &b) {
    // 1 Hz: bin-centre arithmetic, not a different grid.
    return a.n == b.n && a.tones == b.tones && std::fabs(a.f_start_hz - b.f_start_hz) <= 1.0 &&
           std::fabs(a.bin_hz - b.bin_hz) <= 1e-3;
}

void TrackNorm::start_capture(size_t sweeps) {
    ref_.clear();
    want_ = std::max<size_t>(1, sweeps);
    got_ = 0;
    acc_.clear();
}

void TrackNorm::cancel_capture() {
    want_ = got_ = 0;
    acc_.clear();
}

void TrackNorm::clear() {
    cancel_capture();
    ref_.clear();
}

bool TrackNorm::feed(const TrackGrid &g, const float *db, size_t n, std::string *why) {
    if (!want_ || n != g.n || n == 0) return false;
    if (got_ > 0 && (!same_layout(g, acc_grid_) || g.atten_db != acc_grid_.atten_db ||
                     g.gain_db != acc_grid_.gain_db)) {
        if (why) *why = "the sweep changed while the thru was being stored; started again";
        got_ = 0;
    }
    if (got_ == 0) {
        acc_.assign(n, 0.0);
        acc_grid_ = g;
    }
    for (size_t i = 0; i < n; ++i)
        acc_[i] += std::isnan(db[i]) ? std::numeric_limits<double>::quiet_NaN()
                                     : std::pow(10.0, double(db[i]) / 10.0);
    if (++got_ < want_) return false;

    ref_.resize(n);
    for (size_t i = 0; i < n; ++i)
        ref_[i] = std::isnan(acc_[i]) ? std::numeric_limits<float>::quiet_NaN()
                                      : float(10.0 * std::log10(acc_[i] / double(got_)));
    ref_grid_ = acc_grid_;
    want_ = 0;
    acc_.clear();
    return true;
}

std::string TrackNorm::apply(const TrackGrid &g, float *db, size_t n, double *atten_comp_db,
                             std::string *caveat) const {
    if (atten_comp_db) *atten_comp_db = 0;
    if (caveat) caveat->clear();
    if (ref_.empty()) return "no thru stored";
    if (n != ref_.size() || !same_layout(g, ref_grid_)) {
        char m[200];
        std::snprintf(m, sizeof m,
                      "the thru was stored over a different grid (%zu points from %.6g MHz, "
                      "%zu tones per step); store it again on this one",
                      ref_grid_.n, ref_grid_.f_start_hz / 1e6, ref_grid_.tones);
        return m;
    }
    if (std::fabs(g.gain_db - ref_grid_.gain_db) > 0.05) {
        char m[160];
        std::snprintf(m, sizeof m,
                      "the thru was stored at %.1f dB RX gain, the sweep is at %.1f dB; the "
                      "gain table is not linear enough to correct for",
                      ref_grid_.gain_db, g.gain_db);
        return m;
    }
    if (caveat && (g.harmonic != ref_grid_.harmonic || g.rx_offset_hz != ref_grid_.rx_offset_hz ||
                   g.rx_invert != ref_grid_.rx_invert))
        *caveat = "the reference was stored with a different RX mapping; the receiver's "
                  "response where the output lands is not divided out";
    const double comp = g.atten_db - ref_grid_.atten_db;
    for (size_t i = 0; i < n; ++i) db[i] = float(double(db[i]) - double(ref_[i]) + comp);
    if (atten_comp_db) *atten_comp_db = comp;
    return {};
}

float TrackNorm::ref_min_db() const {
    float m = std::numeric_limits<float>::infinity();
    for (float v : ref_)
        if (!std::isnan(v)) m = std::min(m, v);
    return m;
}

}  // namespace sweepui
