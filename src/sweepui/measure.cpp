#include "measure.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace sweepui {

namespace {

// Fraction of point i's cell [x_i - dx/2, x_i + dx/2) inside [lo, hi).
double overlap(const TraceAxis &ax, size_t i, double lo, double hi) {
    const double a = ax.x0 + double(i) * ax.dx;
    const double b = a + ax.dx;
    const double w = std::min(b, hi) - std::max(a, lo);
    return w > 0 ? w / ax.dx : 0.0;
}

bool check(const float *db, size_t n, const TraceAxis &ax, double *f_lo, double *f_hi,
           std::string *reason) {
    if (!db || n == 0 || !ax.valid() || ax.time) {
        *reason = ax.time ? "a band measurement needs a frequency axis" : "no trace";
        return false;
    }
    if (*f_hi < *f_lo) std::swap(*f_lo, *f_hi);
    const double t0 = ax.x0, t1 = ax.x0 + ax.dx * double(n);
    *f_lo = std::max(*f_lo, t0);
    *f_hi = std::min(*f_hi, t1);
    if (!(*f_hi > *f_lo)) {
        *reason = "the band is outside the trace";
        return false;
    }
    return true;
}

}  // namespace

BandResult channel_power(const float *db, size_t n, const TraceAxis &ax, double f_lo,
                         double f_hi, sweepd::Detector det) {
    BandResult r;
    if (!check(db, n, ax, &f_lo, &f_hi, &r.reason)) return r;
    if (!(ax.rbw_hz > 0)) {
        r.reason = "the source reports no RBW, so there is no bandwidth to integrate against";
        return r;
    }
    double lin = 0;
    for (size_t i = 0; i < n; ++i) {
        const double w = overlap(ax, i, f_lo, f_hi);
        if (w <= 0) continue;
        if (std::isnan(db[i])) {
            ++r.n_nan;
            continue;
        }
        ++r.n_points;
        lin += std::pow(10.0, double(db[i]) / 10.0) * w * ax.dx / ax.rbw_hz;
    }
    if (r.n_points == 0) {
        r.reason = "no measured point inside the band";
        return r;
    }
    r.width_hz = f_hi - f_lo;
    r.power_db = 10.0 * std::log10(std::max(lin, 1e-30));
    r.psd_db_hz = r.power_db - 10.0 * std::log10(r.width_hz);
    if (det == sweepd::Detector::Peak)
        r.warning = "Peak detector: noise reads high; use Sample or RMS for power";
    else if (det == sweepd::Detector::NegativePeak)
        r.warning = "Negative Peak detector: noise reads low; use Sample or RMS for power";
    if (r.n_nan)
        r.warning += std::string(r.warning.empty() ? "" : "; ") + std::to_string(r.n_nan) +
                     " unmeasured points skipped";
    r.valid = true;
    return r;
}

ObwResult occupied_bw(const float *db, size_t n, const TraceAxis &ax, double f_lo,
                      double f_hi, double frac) {
    ObwResult r;
    if (!check(db, n, ax, &f_lo, &f_hi, &r.reason)) return r;
    if (!(frac > 0 && frac < 1)) {
        r.reason = "the fraction must be between 0 and 1";
        return r;
    }
    // Cumulative linear power at each cell's right edge. Unmeasured points add
    // nothing -- they are not power, and treating them as such would move the
    // edges.
    std::vector<double> edge, cum;
    double total = 0;
    edge.push_back(f_lo);
    cum.push_back(0);
    for (size_t i = 0; i < n; ++i) {
        const double w = overlap(ax, i, f_lo, f_hi);
        if (w <= 0) continue;
        if (!std::isnan(db[i])) total += std::pow(10.0, double(db[i]) / 10.0) * w;
        edge.push_back(std::min(ax.x0 + double(i + 1) * ax.dx, f_hi));
        cum.push_back(total);
    }
    if (!(total > 0)) {
        r.reason = "no power in the band";
        return r;
    }
    // Linear inside a cell: the crossing lands where it falls, not on a cell edge.
    auto cross = [&](double target) {
        for (size_t k = 1; k < cum.size(); ++k) {
            if (cum[k] >= target) {
                const double span = cum[k] - cum[k - 1];
                const double t = span > 0 ? (target - cum[k - 1]) / span : 0.0;
                return edge[k - 1] + t * (edge[k] - edge[k - 1]);
            }
        }
        return edge.back();
    };
    r.f_lo = cross(total * (1.0 - frac) / 2.0);
    r.f_hi = cross(total * (1.0 + frac) / 2.0);
    r.width_hz = r.f_hi - r.f_lo;
    r.total_db = 10.0 * std::log10(total * ax.dx / std::max(ax.rbw_hz, 1e-30));
    r.valid = true;
    return r;
}

}  // namespace sweepui
