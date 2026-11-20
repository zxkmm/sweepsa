#include "segments.h"

#include <algorithm>
#include <cmath>

namespace sweepui {

namespace {

// plan_sweep's sanity limit. Reproduced rather than shared because profiles.h
// pulls in sysfs.h, which is POSIX-only and sweepui builds on Windows (§6).
constexpr size_t kMaxSegments = 4096;

// The daemon works in integer hertz and its halves truncate
// (PlanConfig::keep_hz / 2, profiles.cpp). Rather than model that truncation --
// which would have to be un-modelled again for the synthetic source, whose
// segment width is span/N and rarely an integer -- the half-width is *derived*
// from covered_lo_hz below, where the daemon has already applied it.
//
// What remains is a tolerance for comparing the far edge, which accumulates
// nothing: every LO is first_lo + i*step evaluated once, so the error is double
// rounding on values below 2^53, not a running sum.
constexpr double kEdgeTolHz = 2.0;

}  // namespace

bool SegmentPlanInfo::complete() const {
    return n_segments > 0 && keep_hz > 0 && step_hz > 0 && blank_hz >= 0 &&
           tune_max_hz > tune_min_hz && covered_hi_hz > covered_lo_hz;
}

bool reconstruct_segments(const SegmentPlanInfo &info, std::vector<SegmentBand> *out,
                          std::string *err) {
    out->clear();
    auto fail = [&](const std::string &m) {
        if (err) *err = m;
        out->clear();
        return false;
    };

    if (!info.complete()) return fail("the source does not report a segment plan");
    if (info.n_segments > kMaxSegments) return fail("implausible segment count");

    const double step = info.step_hz;
    const double lo_min = info.tune_min_hz, lo_max = info.tune_max_hz;
    const auto clamp_lo = [&](double lo) { return std::min(std::max(lo, lo_min), lo_max); };

    // covered_lo_hz is the *first* segment's kept edge, so the half-width the
    // daemon used is the difference between it and that segment's LO. Taking it
    // from the source removes any need to reproduce an integer division here,
    // and works unchanged for a source whose segments are not an integer number
    // of hertz wide.
    const double half = clamp_lo(info.first_lo_hz) - info.covered_lo_hz;
    if (!(half > 0) || half > info.keep_hz)
        return fail("the source's covered range disagrees with its segment width");

    // plan_sweep's loop, and deliberately the same shape: place, clamp to the
    // tuner, and drop a segment whose LO repeats its predecessor's -- clamping
    // collapses the outriggers at the ends of the range onto the limit, and a
    // repeated LO costs a hop and adds nothing.
    //
    // The bound is the reported count plus slack for exactly those collapsed
    // outriggers. Running past it means the reconstruction has diverged, and the
    // check below is what turns that into a refusal.
    std::vector<SegmentBand> bands;
    bands.reserve(info.n_segments);
    const size_t max_iter = size_t(info.n_segments) + 8;
    for (size_t i = 0; i < max_iter && bands.size() < info.n_segments; ++i) {
        const double lo = clamp_lo(info.first_lo_hz + double(i) * step);
        if (!bands.empty() && bands.back().lo_hz == lo) continue;

        SegmentBand b;
        b.lo_hz = lo;
        b.keep_lo_hz = lo - half;
        b.keep_hi_hz = lo + half;
        b.blank_lo_hz = lo - info.blank_hz;
        b.blank_hi_hz = lo + info.blank_hz;
        bands.push_back(b);
    }

    // The reconstruction has to agree with the daemon's own summary of the plan
    // before a single line goes on screen. Two of these are genuinely
    // independent of the arithmetic above -- the segment count and the far
    // covered edge both come from the daemon, and both move if the grid is
    // wrong. The near edge is not: `half` was derived from it, so it can only
    // confirm that the first LO was clamped the way this code assumed.
    if (bands.size() != info.n_segments)
        return fail("reconstructed " + std::to_string(bands.size()) +
                    " segments, the source reports " + std::to_string(info.n_segments));
    if (std::fabs(bands.back().keep_hi_hz - info.covered_hi_hz) > kEdgeTolHz)
        return fail("reconstructed segment grid does not reach the source's covered range");

    // Ownership. sweep.cpp's stitch breaks the ordinary overlap by proximity to
    // the LO, so adjacent segments hand over at the midpoint of their two LOs.
    // The ends own out to their kept edge because no neighbour is there to take
    // over -- clipping them to a midpoint would draw the sweep as narrower than
    // covered_lo_hz/covered_hi_hz say it is.
    for (size_t i = 0; i < bands.size(); ++i) {
        SegmentBand &b = bands[i];
        b.own_lo_hz = i == 0 ? b.keep_lo_hz : 0.5 * (bands[i - 1].lo_hz + b.lo_hz);
        b.own_hi_hz = i + 1 == bands.size() ? b.keep_hi_hz : 0.5 * (b.lo_hz + bands[i + 1].lo_hz);
        // A midpoint cannot reach outside the kept band while step <= keep, which
        // plan_sweep guarantees for both policies. Clamped anyway, because the
        // alternative is an overlay that draws a segment owning frequencies it
        // never measured.
        b.own_lo_hz = std::max(b.own_lo_hz, b.keep_lo_hz);
        b.own_hi_hz = std::min(b.own_hi_hz, b.keep_hi_hz);
    }

    *out = std::move(bands);
    return true;
}

int segment_at(const std::vector<SegmentBand> &bands, double f_hz) {
    if (bands.empty()) return -1;
    if (f_hz < bands.front().own_lo_hz || f_hz > bands.back().own_hi_hz) return -1;

    // Ownership is a partition of [covered_lo, covered_hi], so the first band
    // whose own_hi reaches f_hz is the owner.
    const auto it = std::lower_bound(bands.begin(), bands.end(), f_hz,
                                     [](const SegmentBand &b, double f) { return b.own_hi_hz < f; });
    if (it == bands.end()) return -1;
    return int(it - bands.begin());
}

bool in_blanked_span(const std::vector<SegmentBand> &bands, double f_hz) {
    // A blanked span belongs to the segment centred on it, but that segment does
    // not own those bins after the stitch -- its neighbour does. So the owner is
    // found first and its neighbours are what get tested, not the owner itself.
    const int i = segment_at(bands, f_hz);
    if (i < 0) return false;
    for (int j = std::max(0, i - 1); j <= std::min(int(bands.size()) - 1, i + 1); ++j) {
        if (f_hz >= bands[size_t(j)].blank_lo_hz && f_hz <= bands[size_t(j)].blank_hi_hz)
            return true;
    }
    return false;
}

SegmentOverlay build_overlay(const std::vector<SegmentBand> &bands, double x_lo, double x_hi,
                             double plot_px, double min_px) {
    SegmentOverlay ov;
    if (bands.empty() || !(x_hi > x_lo) || !(plot_px > 0)) return ov;

    const double px_per_hz = plot_px / (x_hi - x_lo);

    // Count what the view touches before deciding whether to draw it, so the
    // caller can say "14 segments here, too close to draw" rather than showing
    // nothing and looking broken.
    for (const SegmentBand &b : bands)
        if (b.own_hi_hz >= x_lo && b.own_lo_hz <= x_hi) ++ov.n_in_view;

    // Spacing is uniform except where the tuner clamp collapsed an outrigger, so
    // the step between the first two bands is representative. A single segment
    // has no spacing to be too dense about.
    if (bands.size() > 1) {
        const double spacing = bands[1].lo_hz - bands[0].lo_hz;
        if (spacing * px_per_hz < min_px) {
            ov.too_dense = true;
            return ov;
        }
    }

    for (size_t i = 0; i < bands.size(); ++i) {
        const SegmentBand &b = bands[i];
        if (b.lo_hz >= x_lo && b.lo_hz <= x_hi) {
            ov.centres.push_back(b.lo_hz);
            ov.centre_seg.push_back(int(i));
            ov.blank_lo.push_back(b.blank_lo_hz);
            ov.blank_hi.push_back(b.blank_hi_hz);
        }
        // The seam at own_hi is shared with the next segment; emitting it from
        // the lower one only is what keeps it from being drawn twice at double
        // opacity. The last segment's own_hi is the edge of the sweep, not a
        // seam, so it is skipped.
        if (i + 1 < bands.size() && b.own_hi_hz >= x_lo && b.own_hi_hz <= x_hi)
            ov.seams.push_back(b.own_hi_hz);
    }
    return ov;
}

}  // namespace sweepui
