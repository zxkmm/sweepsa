#include "imagecheck.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace sweepui {
namespace {

// Half-width of the floor estimator, as a fraction of one segment's width in
// output bins. A quarter of a segment is wide enough that a signal cannot drag
// its own floor up appreciably, and narrow enough that the window never spans a
// segment boundary -- which is the property that makes the per-segment
// amplitude offset cancel. Not a measured constant: it is a shape parameter of
// a median filter, and ImageCheckConfig::floor_halfwidth overrides it.
constexpr double kFloorFractionOfSegment = 0.25;
constexpr size_t kFloorMinBins = 4;
constexpr size_t kFloorMaxBins = 96;

bool finite(float v) { return std::isfinite(v); }

// Does `x` sit in a blanked span of `bands`, allowing one output bin of slack
// either side? The slack matters: an output bin straddling the edge of the hole
// is part fill and part real, and it reads hot for the same reason the centre
// does.
bool blanked_near(const std::vector<SegmentBand> &bands, double x, double dx) {
    if (bands.empty()) return false;
    return in_blanked_span(bands, x) || in_blanked_span(bands, x - dx) ||
           in_blanked_span(bands, x + dx);
}

}  // namespace

const char *image_verdict_name(ImageVerdict v) {
    switch (v) {
        case ImageVerdict::Image: return "image";
        case ImageVerdict::LoSpur: return "LO spur";
        case ImageVerdict::Moved: return "moved";
    }
    return "?";
}

std::vector<float> local_excursion(const std::vector<float> &db, size_t halfwidth) {
    return local_excursion(db, std::vector<int>(), halfwidth);
}

std::vector<float> local_excursion(const std::vector<float> &db,
                                   const std::vector<int> &owner, size_t halfwidth) {
    std::vector<float> out(db.size(), std::numeric_limits<float>::quiet_NaN());
    if (db.empty() || halfwidth == 0) return out;
    const bool clip = owner.size() == db.size();

    std::vector<float> win;
    win.reserve(2 * halfwidth + 1);
    for (size_t i = 0; i < db.size(); ++i) {
        if (!finite(db[i])) continue;
        size_t lo = i > halfwidth ? i - halfwidth : 0;
        size_t hi = std::min(db.size(), i + halfwidth + 1);
        // Clip to the run of bins sharing this bin's owning segment. The runs
        // are contiguous because segment ownership is an ascending partition of
        // the axis (segments.h), so walking out from i finds the whole run.
        if (clip && owner[i] >= 0) {
            while (lo < i && owner[lo] != owner[i]) ++lo;
            while (hi > i + 1 && owner[hi - 1] != owner[i]) --hi;
        }
        win.clear();
        for (size_t j = lo; j < hi; ++j)
            if (finite(db[j])) win.push_back(db[j]);
        if (win.empty()) continue;
        // Median, not mean: a strong signal inside the window pulls a mean up by
        // its own excursion divided by the window length, so a 40 dB tone on a
        // 33-bin window would understate itself by more than a decibel and the
        // error would depend on how many signals happened to be nearby.
        const size_t mid = win.size() / 2;
        std::nth_element(win.begin(), win.begin() + mid, win.end());
        out[i] = db[i] - win[mid];
    }
    return out;
}

namespace {

// The strongest excursion the shifted pass shows anywhere within one bin of
// `x`, or NaN if it measured nothing there.
//
// A *maximum* over the overlap, deliberately. The two passes are on grids
// offset by the shift, so their bin centres do not line up and a real signal
// can land on a bin edge in one pass and a bin centre in the other. Taking the
// max biases the check toward *not* flagging, which is the safe direction: a
// missed image leaves an artifact on screen unlabelled, while a false one tells
// the user a real signal is not real.
float b_excursion_at(const std::vector<float> &exc_b, const TraceAxis &ax_b, double x,
                     double reach) {
    if (!ax_b.valid() || exc_b.empty()) return std::numeric_limits<float>::quiet_NaN();
    const double lo = x - reach, hi = x + reach;
    long long j0 = (long long)std::floor((lo - ax_b.x0) / ax_b.dx - 0.5);
    long long j1 = (long long)std::ceil((hi - ax_b.x0) / ax_b.dx - 0.5);
    if (j0 < 0) j0 = 0;
    if (j1 >= (long long)exc_b.size()) j1 = (long long)exc_b.size() - 1;

    float best = std::numeric_limits<float>::quiet_NaN();
    for (long long j = j0; j <= j1; ++j) {
        const float v = exc_b[size_t(j)];
        if (!finite(v)) continue;
        if (!finite(best) || v > best) best = v;
    }
    return best;
}

// Which segment owns each output bin, for clipping the floor window. Empty when
// the grid was not reported, which leaves the window unclipped.
std::vector<int> owners(const ImagePass &p) {
    std::vector<int> v;
    if (p.segments.empty()) return v;
    v.resize(p.axis.n);
    for (size_t i = 0; i < p.axis.n; ++i) v[i] = segment_at(p.segments, p.axis.x_at(i));
    return v;
}

}  // namespace

// ---------------------------------------------------------------- one pass

void PassAccumulator::begin(const TraceAxis &ax, double lo_shift_hz) {
    ax_ = ax;
    shift_ = lo_shift_hz;
    sum_.assign(ax.n, 0.0);
    cnt_.assign(ax.n, 0);
    sweeps_ = 0;
}

bool PassAccumulator::matches(const TraceAxis &ax) const {
    return ax_.valid() && ax.n == ax_.n && ax.dx == ax_.dx && ax.x0 == ax_.x0 &&
           ax.time == ax_.time;
}

bool PassAccumulator::add(const TraceAxis &ax, const float *db, size_t n) {
    if (!matches(ax) || n != ax_.n) return false;
    for (size_t i = 0; i < n; ++i) {
        if (!finite(db[i])) continue;
        sum_[i] += std::pow(10.0, double(db[i]) / 10.0);
        ++cnt_[i];
    }
    ++sweeps_;
    return true;
}

ImagePass PassAccumulator::finish(const std::vector<SegmentBand> &segments) const {
    ImagePass p;
    p.axis = ax_;
    p.lo_shift_hz = shift_;
    p.sweeps = sweeps_;
    p.segments = segments;
    p.db.assign(ax_.n, std::numeric_limits<float>::quiet_NaN());
    for (size_t i = 0; i < ax_.n; ++i) {
        if (!cnt_[i]) continue;
        p.db[i] = float(10.0 * std::log10(sum_[i] / double(cnt_[i])));
    }
    return p;
}

ImageCheckResult run_image_check(const ImagePass &a, const ImagePass &b,
                                 const ImageCheckConfig &cfg) {
    ImageCheckResult r;
    auto fail = [&](const std::string &m) {
        r.err = m;
        return r;
    };

    if (!a.axis.valid() || !b.axis.valid()) return fail("one of the passes has no axis yet");
    if (a.axis.time || b.axis.time)
        return fail("the image check is a sweep measurement; zero span has one LO and "
                    "no grid to shift");
    if (a.db.size() != a.axis.n || b.db.size() != b.axis.n)
        return fail("a pass does not match its own axis");

    const double shift = b.lo_shift_hz - a.lo_shift_hz;
    if (shift == 0)
        return fail("both passes were taken on the same LO grid, so nothing could have "
                    "moved");
    if (a.sweeps < cfg.min_sweeps || b.sweeps < cfg.min_sweeps) {
        char m[220];
        std::snprintf(m, sizeof m,
                      "each pass needs at least %zu averaged sweeps (got %zu and %zu); "
                      "below that the comparison is between two moments, not two grids",
                      cfg.min_sweeps, a.sweeps, b.sweeps);
        return fail(m);
    }

    r.shift_hz = shift;
    r.segments_known = !a.segments.empty() && !b.segments.empty();

    // The floor window, in bins. Derived from the segment width so it stays
    // inside one segment at any span and point count -- see the header.
    size_t hw = cfg.floor_halfwidth;
    if (hw == 0) {
        // The LO step is twice the shift under the check's own convention, and
        // that is the width one segment owns after the stitch.
        const double step_hz = 2.0 * std::fabs(shift);
        const double bins = step_hz / a.axis.dx;
        double want = bins * kFloorFractionOfSegment * 0.5;
        if (!(want >= double(kFloorMinBins))) want = double(kFloorMinBins);
        if (want > double(kFloorMaxBins)) want = double(kFloorMaxBins);
        hw = size_t(std::llround(want));
    }
    r.floor_halfwidth = double(hw);

    const std::vector<float> exc_a = local_excursion(a.db, owners(a), hw);
    const std::vector<float> exc_b = local_excursion(b.db, owners(b), hw);

    // How far either side of an A bin centre a B bin is allowed to be and still
    // be the same frequency: one bin of each grid. Their centres are offset by
    // the shift, so they never coincide.
    const double reach = a.axis.dx + b.axis.dx;

    // Pass 1: classify every bin, without yet grouping them.
    std::vector<char> flagged(a.axis.n, 0);
    std::vector<float> drop(a.axis.n, 0.0f);
    std::vector<float> bexc(a.axis.n, 0.0f);

    for (size_t i = 0; i < a.axis.n; ++i) {
        const double x = a.axis.x_at(i);
        if (!finite(exc_a[i])) continue;

        // The splice comb, excluded by position in *both* grids: at an A centre
        // the A pass is hot, and at a B centre the B pass is, and either one
        // produces a difference that has nothing to do with an image.
        if (blanked_near(a.segments, x, a.axis.dx) ||
            blanked_near(b.segments, x, a.axis.dx)) {
            ++r.n_blanked;
            continue;
        }

        const float eb = b_excursion_at(exc_b, b.axis, x, reach);
        if (!finite(eb)) {
            ++r.n_uncovered;
            continue;
        }
        ++r.n_compared;

        if (exc_a[i] < cfg.min_excursion_db) continue;
        const float d = exc_a[i] - eb;
        if (d < cfg.move_db) continue;

        flagged[i] = 1;
        drop[i] = d;
        bexc[i] = eb;
    }

    // Pass 2: a run of flagged bins is one response. Report it at its peak.
    for (size_t i = 0; i < a.axis.n;) {
        if (!flagged[i]) { ++i; continue; }
        size_t j = i, peak = i;
        while (j < a.axis.n && flagged[j]) {
            if (exc_a[j] > exc_a[peak]) peak = j;
            ++j;
        }

        if (r.findings.size() >= cfg.max_findings) {
            ++r.dropped;
            i = j;
            continue;
        }

        ImageFinding f;
        f.bin = peak;
        f.x_hz = a.axis.x_at(peak);
        f.excursion_a_db = exc_a[peak];
        f.excursion_b_db = bexc[peak];
        f.drop_db = drop[peak];
        f.width_bins = j - i;

        // Where did it go? An image is mirrored about the LO, so moving the
        // grid by S moves it by 2S. A spur at a fixed offset from the LO moves
        // by S, with the LO. Either sign: which way depends on which side of
        // its own LO the responsible signal sits.
        const double cands[4] = {f.x_hz + 2 * shift, f.x_hz - 2 * shift, f.x_hz + shift,
                                 f.x_hz - shift};
        const ImageVerdict kinds[4] = {ImageVerdict::Image, ImageVerdict::Image,
                                       ImageVerdict::LoSpur, ImageVerdict::LoSpur};
        bool matched = false;
        for (int k = 0; k < 4; ++k) {
            const float v = b_excursion_at(exc_b, b.axis, cands[k], reach);
            if (!finite(v)) continue;
            if (v > f.best_candidate_db || f.best_candidate_hz == 0) {
                f.best_candidate_db = v;
                f.best_candidate_hz = cands[k];
            }
            if (matched) continue;
            if (v < cfg.match_excursion_db) continue;
            f.verdict = kinds[k];
            f.found_at_hz = cands[k];
            matched = true;
        }

        r.findings.push_back(f);
        i = j;
    }

    std::sort(r.findings.begin(), r.findings.end(),
              [](const ImageFinding &x, const ImageFinding &y) {
                  return x.excursion_a_db > y.excursion_a_db;
              });

    // Never empty, and it names the failure the measurement genuinely has
    // rather than the ones it does not.
    std::string c =
        "Both passes are averages taken minutes apart. A transmitter that was on "
        "for one and off for the other reads exactly like an image.";
    if (!r.segments_known)
        c += " The segment grid was not reported, so the +4.5 dB splice comb at each "
             "segment centre could not be excluded and those bins are unclassifiable.";
    r.caveat = c;

    r.valid = true;
    return r;
}

}  // namespace sweepui
