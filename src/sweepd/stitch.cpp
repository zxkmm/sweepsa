#include "stitch.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace sweepd {

void Stitcher::reset(double out_lo_hz, double out_hi_hz, size_t n_points, Detector det) {
    out_lo_ = out_lo_hz;
    out_hi_ = out_hi_hz;
    n_points_ = n_points;
    det_ = det;
    bin_hz_ = n_points ? (out_hi_hz - out_lo_hz) / double(n_points) : 0.0;

    grid_val_.assign(n_points, 0.0f);
    grid_cnt_.assign(n_points, 0u);
    grid_dist_.assign(n_points, 0);
    seg_val_.assign(n_points, 0.0f);
    seg_aux_.assign(n_points, 0.0);
    seg_cnt_.assign(n_points, 0u);
}

void Stitcher::clear() {
    std::fill(grid_cnt_.begin(), grid_cnt_.end(), 0u);
}

size_t Stitcher::uncovered() const {
    size_t n = 0;
    for (uint32_t c : grid_cnt_)
        if (c == 0) ++n;
    return n;
}

void Stitcher::add(const float *power, size_t n_fft, double fs_hz, const StitchBand &b) {
    if (n_points_ == 0 || n_fft == 0 || bin_hz_ <= 0) return;

    const size_t n = n_fft;
    const double bin_fft = fs_hz / double(n);
    const double lo = b.lo_hz;
    const double inv_bin = 1.0 / bin_hz_;

    // Frequency span this block may contribute to: its kept band, clipped to
    // the output grid.
    const double f_lo = std::max(b.keep_lo_hz, out_lo_);
    const double f_hi = std::min(b.keep_hi_hz, out_hi_);
    if (f_hi <= f_lo) return;

    // Bin j sits at lo + (j - n/2) * bin_fft (power_spectrum returns fftshifted
    // output, DC at n/2).
    long long j_lo = (long long)std::ceil((f_lo - lo) / bin_fft) + (long long)(n / 2);
    long long j_hi = (long long)std::floor((f_hi - lo) / bin_fft) + (long long)(n / 2);
    j_lo = std::max<long long>(j_lo, 0);
    j_hi = std::min<long long>(j_hi, (long long)n - 1);
    if (j_hi < j_lo) return;

    size_t o_first = size_t(std::max(0.0, (f_lo - out_lo_) * inv_bin));
    size_t o_last = size_t(std::max(0.0, (f_hi - out_lo_) * inv_bin));
    if (o_first >= n_points_) return;
    if (o_last >= n_points_) o_last = n_points_ - 1;
    for (size_t o = o_first; o <= o_last; ++o) seg_cnt_[o] = 0;

    const bool blanked = b.blank_hi_hz > b.blank_lo_hz;
    const double blank_lo = b.blank_lo_hz, blank_hi = b.blank_hi_hz;

    // Both the frequency and the output bin advance by a constant step, so they
    // are carried incrementally. The obvious formulation recomputes
    // (long long)((f - out_lo) / bin_hz) per FFT bin, and on the E310's
    // Cortex-A9 that double-to-int64 conversion plus the surrounding double
    // arithmetic cost ~110 cycles a bin -- measured at 777 us per segment at
    // N=4096, which is most of a segment's control budget spent on address
    // arithmetic. Stepping an integer alongside a monotonically increasing
    // double removes it: the while loop advances at most n_points times over
    // the whole block, so it is O(1) amortised per bin.
    const double step_o = bin_fft * inv_bin;
    double f = lo + double(j_lo - (long long)(n / 2)) * bin_fft;
    double opos = (f - out_lo_) * inv_bin;
    if (opos < 0) opos = 0;                 // j_lo is chosen so f >= out_lo_
    long long o = (long long)opos;
    double o_hi = double(o) + 1.0;

    for (long long j = j_lo; j <= j_hi; ++j, f += bin_fft, opos += step_o) {
        // Kept ahead of the blank test so `o` stays in step across skipped bins.
        while (opos >= o_hi) { ++o; o_hi += 1.0; }

        if (blanked && f >= blank_lo && f <= blank_hi) continue;
        if (o >= (long long)n_points_) break;
        const size_t o_idx = size_t(o);
        const float p = power[j];

        if (seg_cnt_[o_idx] == 0) {
            seg_val_[o_idx] = p;
            if (det_ == Detector::Sample)
                seg_aux_[o_idx] = std::fabs(f - (out_lo_ + (double(o) + 0.5) * bin_hz_));
        } else {
            switch (det_) {
                case Detector::Peak:
                    if (p > seg_val_[o_idx]) seg_val_[o_idx] = p;
                    break;
                case Detector::NegativePeak:
                    if (p < seg_val_[o_idx]) seg_val_[o_idx] = p;
                    break;
                case Detector::Sample: {
                    // Nearest the output bin's centre, so the sample represents
                    // the bin rather than one of its edges.
                    const double dd =
                        std::fabs(f - (out_lo_ + (double(o) + 0.5) * bin_hz_));
                    if (dd < seg_aux_[o_idx]) {
                        seg_aux_[o_idx] = dd;
                        seg_val_[o_idx] = p;
                    }
                    break;
                }
                case Detector::RMS:
                    // Sum of LINEAR power. §5: averaging dB is a different and
                    // wrong operation; the single conversion happens in to_db()
                    // once the whole sweep is assembled.
                    seg_val_[o_idx] += p;
                    break;
            }
        }
        ++seg_cnt_[o_idx];
    }

    const long long dist_base = (long long)lo;
    for (size_t o = o_first; o <= o_last; ++o) {
        const uint32_t cnt = seg_cnt_[o];
        if (cnt == 0) continue;

        const double centre = out_lo_ + (double(o) + 0.5) * bin_hz_;
        const long long dist = std::llabs((long long)centre - dist_base);
        const bool better = grid_cnt_[o] == 0 || cnt > grid_cnt_[o] ||
                            (cnt == grid_cnt_[o] && dist < grid_dist_[o]);
        if (!better) continue;

        grid_val_[o] = (det_ == Detector::RMS) ? seg_val_[o] / float(cnt) : seg_val_[o];
        grid_cnt_[o] = cnt;
        grid_dist_[o] = dist;
    }
}

void Stitcher::to_db(float *out) const {
    std::copy(grid_val_.begin(), grid_val_.end(), out);
    power_to_db(out, n_points_);
    for (size_t o = 0; o < n_points_; ++o)
        if (grid_cnt_[o] == 0) out[o] = std::numeric_limits<float>::quiet_NaN();
}

}  // namespace sweepd
