#include "marker.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sweepui {
namespace {

inline double db_to_power(double db) { return std::pow(10.0, db / 10.0); }

// The search range, resolved against the trace length.
bool range_of(size_t n, const PeakOptions &opt, size_t *lo, size_t *hi) {
    *lo = std::min(opt.lo, n);
    *hi = (opt.hi == 0 || opt.hi > n) ? n : opt.hi;
    return *hi > *lo;
}

}  // namespace

// ---------------------------------------------------------------- searching

void find_peaks(const float *db, size_t n, const PeakOptions &opt,
                std::vector<size_t> *out) {
    out->clear();
    size_t lo = 0, hi = 0;
    if (!db || !range_of(n, opt, &lo, &hi)) return;

    const double exc = opt.excursion_db > 0 ? opt.excursion_db : 0.0;

    // The single-pass excursion state machine described in marker.h. `Rising`
    // is tracking a candidate maximum and waiting for the fall that confirms
    // it; `Falling` is tracking the valley and waiting for the rise that starts
    // the next candidate.
    enum State { Rising, Falling };
    State state = Rising;
    bool seeded = false;
    double best = 0, valley = 0, rise_from = 0;
    size_t best_i = 0;

    // One rule, used both when the confirming fall arrives and when the data
    // runs out before it does: the candidate must have *risen* `exc` above the
    // valley it climbed from, and clear the threshold. Requiring the rise on
    // the fall path too is what keeps the first sample in the range from being
    // reported as a peak merely because the trace dropped away to its right --
    // nothing is known about the ground to its left, and `rise_from` is seeded
    // to the candidate itself so that case cannot qualify.
    auto confirm = [&]() {
        if (!seeded || state != Rising) return;
        if (best - rise_from < exc) return;
        if (best < opt.threshold_db) return;
        out->push_back(best_i);
    };

    for (size_t i = lo; i < hi; ++i) {
        const double v = double(db[i]);
        if (std::isnan(v)) {
            // An unmeasured bin is not a valley -- nothing measured whether the
            // trace fell there -- so it breaks the run rather than continuing
            // it. A signal either side of a blanked hole is two features, not
            // one that appears to dip.
            confirm();
            seeded = false;
            state = Rising;
            continue;
        }
        if (!seeded) {
            seeded = true;
            state = Rising;
            best = valley = rise_from = v;
            best_i = i;
            continue;
        }

        if (state == Rising) {
            if (v > best) {
                best = v;
                best_i = i;
            } else if (v <= best - exc) {
                confirm();
                state = Falling;
                valley = v;
            }
        } else {
            if (v < valley) {
                valley = v;
            } else if (v >= valley + exc) {
                state = Rising;
                best = v;
                best_i = i;
                rise_from = valley;
            }
        }
    }
    confirm();
}

bool peak_search(const float *db, size_t n, const PeakOptions &opt, size_t *out_bin) {
    std::vector<size_t> p;
    find_peaks(db, n, opt, &p);
    if (p.empty()) return false;
    size_t best = p[0];
    for (size_t i : p)
        if (double(db[i]) > double(db[best])) best = i;
    *out_bin = best;
    return true;
}

bool next_peak_lower(const float *db, size_t n, const PeakOptions &opt,
                     size_t from_bin, size_t *out_bin) {
    std::vector<size_t> p;
    find_peaks(db, n, opt, &p);
    if (p.empty()) return false;

    // No usable amplitude to step down from -- the marker is sitting on a hole
    // or has never been placed -- so "next peak down" degenerates to "the
    // highest peak", which is where a search would have started anyway.
    if (from_bin >= n || std::isnan(db[from_bin])) return peak_search(db, n, opt, out_bin);

    const double a = double(db[from_bin]);
    bool found = false;
    size_t best = 0;
    for (size_t i : p) {
        const double v = double(db[i]);
        // Strictly lower, or exactly equal but further right. The tie rule is
        // what lets repeated presses walk through a set of equal-amplitude
        // peaks instead of oscillating between the first two.
        if (!(v < a || (v == a && i > from_bin))) continue;
        if (!found || v > double(db[best])) {
            found = true;
            best = i;
        }
    }
    if (found) *out_bin = best;
    return found;
}

bool next_peak_left(const float *db, size_t n, const PeakOptions &opt, size_t from_bin,
                    size_t *out_bin) {
    std::vector<size_t> p;
    find_peaks(db, n, opt, &p);
    for (auto it = p.rbegin(); it != p.rend(); ++it) {
        if (*it < from_bin) {
            *out_bin = *it;
            return true;
        }
    }
    return false;
}

bool next_peak_right(const float *db, size_t n, const PeakOptions &opt, size_t from_bin,
                     size_t *out_bin) {
    std::vector<size_t> p;
    find_peaks(db, n, opt, &p);
    for (size_t i : p) {
        if (i > from_bin) {
            *out_bin = i;
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------- the axis

bool TraceAxis::bin_of(double x, size_t *out) const {
    if (!valid()) return false;
    const double f = (x - x0) / dx;
    if (!(f >= 0.0)) return false;              // also rejects NaN
    const double i = std::floor(f);
    if (i >= double(n)) return false;
    *out = size_t(i);
    return true;
}

// ---------------------------------------------------------------- readout

bool noise_detector_ok(sweepd::Detector d) {
    // The noise density is the mean noise power per hertz, so the trace has to
    // be an estimate of the mean. RMS is; Sample is (noisier, but unbiased in
    // power). Peak and Negative Peak are the extremes of the FFT bins under
    // each output bin and sit well away from the mean, by an amount that grows
    // with the decimation factor -- which is to say, with the span.
    return d == sweepd::Detector::RMS || d == sweepd::Detector::Sample;
}

MarkerReadout read_marker(const Marker &m, const Marker *ref, const float *db, size_t n,
                          const TraceAxis &ax, size_t noise_avg_bins) {
    MarkerReadout r;
    r.on = m.on;
    r.is_delta = m.delta;
    r.is_noise = m.noise;
    if (!m.on) return r;

    if (!db || n == 0 || !ax.valid()) {
        r.reason = "no trace";
        return r;
    }
    r.time = ax.time;
    if (!ax.bin_of(m.x, &r.bin) || r.bin >= n) {
        r.reason = ax.outside_reason();
        return r;
    }
    r.abs_x = ax.x_at(r.bin);
    r.x = r.abs_x;

    if (std::isnan(db[r.bin])) {
        r.unmeasured = true;
        // In zero span a NaN point is a block whose capture was lost, not a
        // dc_policy=blank hole -- different cause, same rule: it is unmeasured
        // rather than quiet, and §7 wants that said, not interpolated away.
        r.reason = ax.time ? "unmeasured -- this block was not captured"
                           : "unmeasured -- no segment covered this bin";
        return r;
    }

    if (m.noise) {
        if (!(ax.rbw_hz > 0)) {
            r.reason = "no RBW reported, so a density cannot be computed";
            return r;
        }
        // Average linear power over a few neighbouring bins to cut the
        // variance, then normalise to 1 Hz by the *resolution* bandwidth. See
        // the header: the divisor is the RBW, never the output bin width.
        const size_t half = noise_avg_bins / 2;
        const size_t b0 = r.bin > half ? r.bin - half : 0;
        const size_t b1 = std::min(n, r.bin + half + 1);
        double sum = 0;
        size_t cnt = 0;
        for (size_t i = b0; i < b1; ++i) {
            if (std::isnan(db[i])) continue;
            sum += db_to_power(double(db[i]));
            ++cnt;
        }
        const double mean = cnt ? sum / double(cnt) : 0.0;
        r.abs_db = (mean > 0 ? 10.0 * std::log10(mean) : -200.0) -
                   10.0 * std::log10(ax.rbw_hz);
    } else {
        r.abs_db = double(db[r.bin]);
    }

    if (!m.delta) {
        r.db = r.abs_db;
        r.valid = true;
        return r;
    }

    // From here `db` and `x` stay zero until the delta is known to be
    // meaningful. abs_db and abs_x keep the marker's own reading, so a
    // refused delta can still show where the marker is and what it sees -- but
    // a caller that forgets to check `valid` gets an obvious zero rather than
    // the absolute level wearing a delta's label.
    r.x = 0;

    if (!ref || ref == &m) {
        r.reason = "delta marker has no reference";
        return r;
    }
    const MarkerReadout rr = read_marker(*ref, nullptr, db, n, ax, noise_avg_bins);
    if (!rr.valid) {
        r.reason = "the reference marker has no reading";
        return r;
    }
    if (rr.is_noise != m.noise) {
        // One is a level in dBFS and the other a density in dBFS/Hz. Their
        // difference is not a ratio of anything; it is an RBW in disguise.
        r.reason = "cannot subtract a noise density from a level";
        return r;
    }
    r.x = r.abs_x - rr.abs_x;
    r.db = r.abs_db - rr.abs_db;
    r.valid = true;
    return r;
}

// ---------------------------------------------------------------- MarkerSet

bool MarkerSet::any_on() const {
    for (size_t i = 0; i < kCount; ++i)
        if (m_[i].on) return true;
    return false;
}

void MarkerSet::all_off() {
    for (size_t i = 0; i < kCount; ++i) m_[i].on = false;
}

}  // namespace sweepui
