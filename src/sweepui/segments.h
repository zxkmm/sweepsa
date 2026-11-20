// §7's segment boundary overlay: where the stitched spectrum's seams are.
//
// CLAUDE.md §7 asks for a "segment boundary overlay (debug view) so stitching
// artifacts are identifiable rather than mysterious". The tree has a concrete
// one waiting for it: the **+4.5 dB splice comb** at every segment centre
// (PROJECT-STATE.md, "The splice fill is hot"). That comb is invisible in the
// trace and obvious in the waterfall, and until something on screen says where
// the segment centres are, it reads as 298 narrow signals rather than as one
// artifact of how the sweep is assembled.
//
// ------------------------------------------------------- three kinds of line
//
// They are not the same thing and the overlay must not merge them:
//
//   1. **Centres.** Where the LO sits. Direct conversion puts LO leakage at
//      baseband DC, so this is the frequency the daemon blanks -- and under
//      dc_policy=splice, the frequency whose value is filled in from a
//      neighbour. The comb is here.
//   2. **Seams.** Where ownership crosses from one segment to the next.
//      sweep.cpp's stitch prefers the segment that saw a frequency closest to
//      its own LO, so the crossover is the midpoint between adjacent LOs. This
//      is where fastlock's predecessor-dependent amplitude (up to 6.3 dB,
//      docs/characterization.md) shows up as a step, and the stitch deliberately
//      does not blend it away.
//   3. **Blanked spans.** [lo - blank_hz, lo + blank_hz] around each centre.
//      Under `blank` those bins are NaN; under `splice` they carry a
//      neighbour's data, taken at keep/2 from *that* segment's LO -- the only
//      data in the whole sweep drawn from that far out, which is the leading
//      explanation for why it reads hot.
//
// ------------------------------------------------------------ reconstruction
//
// The daemon owns the plan; this file rebuilds it from what `config` reports.
// That is a second implementation of sweepd::plan_sweep's geometry and it can
// drift, so two things guard it: the reconstruction is checked against the
// config's own n_segments and covered range before anything is drawn, and
// tests/segments_test.cpp runs the *real* plan_sweep over a spread of configs
// and compares every edge. A drifting overlay is worse than no overlay -- it
// would attribute an artifact to the wrong frequency, which is the §7 failure
// one level up.
//
// Nothing here touches ImGui, ImPlot or OpenGL, so all of it is host-tested.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sweepui {

// The subset of the daemon's `config` that describes the segment plan. Every
// field here is already on the wire.
struct SegmentPlanInfo {
    // The LO grid, before the tuner clamp. sweepd sends these; see SweepPlan.
    double first_lo_hz = 0;
    double step_hz = 0;

    double keep_hz = 0;

    // HALF-width of the blanked hole, matching PlanConfig::blank_hz: the hole is
    // [lo - blank_hz, lo + blank_hz]. Reading this as a full width draws a hole
    // half the size of the real one and puts the comb inside the "good" region.
    double blank_hz = 0;

    // The tuner's reachable LO range, which is what the plan clamps to.
    double tune_min_hz = 0;
    double tune_max_hz = 0;

    // The union of the kept bands, and the count -- both used to check the
    // reconstruction rather than to build it.
    double covered_lo_hz = 0;
    double covered_hi_hz = 0;
    uint32_t n_segments = 0;

    // dc_policy: splice (centres keep/2 apart, holes covered) or blank.
    bool splice = true;

    bool complete() const;
};

struct SegmentBand {
    double lo_hz = 0;                      // the LO / segment centre
    double keep_lo_hz = 0, keep_hi_hz = 0; // the usable band, lo +/- keep/2
    double blank_lo_hz = 0, blank_hi_hz = 0;

    // The part of the output grid this segment actually owns after the stitch.
    // Interior segments hand over at the midpoint to each neighbour; the two end
    // segments own out to their kept edge, because there is nothing beyond them.
    // Under dc_policy=blank step == keep, so ownership and the kept band
    // coincide and the two pairs are equal -- which is the correct picture of
    // that policy, not a degenerate one.
    double own_lo_hz = 0, own_hi_hz = 0;
};

// Rebuilds the daemon's LO grid. False, with a reason, when the result does not
// reproduce the config's own n_segments and covered range -- the overlay is then
// not drawn rather than drawn somewhere plausible.
bool reconstruct_segments(const SegmentPlanInfo &info, std::vector<SegmentBand> *out,
                          std::string *err);

// Index of the segment owning `f_hz` after the stitch, or -1 outside the
// covered range. Bands are ascending and non-overlapping in ownership, so this
// is a binary search.
int segment_at(const std::vector<SegmentBand> &bands, double f_hz);

// True when `f_hz` falls inside some segment's blanked hole. Under
// dc_policy=splice that means the value shown there was measured by a
// *neighbour*, at keep/2 from its LO; under `blank` it means the bin is NaN.
// Either way it is not what the owning segment measured, and a marker sitting
// on one is reading something different from its neighbours.
bool in_blanked_span(const std::vector<SegmentBand> &bands, double f_hz);

// ------------------------------------------------------------------ overlay
//
// What to draw for a given view. Culled to the visible range, because a full
// span is 298 segments and all but a handful are off screen at any useful zoom.

struct SegmentOverlay {
    // Parallel arrays: position and the segment it belongs to, so the caller can
    // colour a centre by that segment's overload bit without a second search.
    std::vector<double> centres;
    std::vector<int> centre_seg;

    // Ownership crossovers. One fewer than the centres in a full view.
    std::vector<double> seams;

    // Blanked spans as [lo, hi] pairs, flattened. Parallel to `centres`.
    std::vector<double> blank_lo, blank_hi;

    // Set when the centres are closer together than the pixel threshold. The
    // overlay is then empty rather than a solid band of lines, and the caller
    // says so: a picture of 298 adjacent vertical lines carries no information
    // and hides the trace it is supposed to annotate.
    bool too_dense = false;

    // Segments intersecting the view, whether or not their lines were emitted.
    size_t n_in_view = 0;

    bool empty() const { return centres.empty() && seams.empty(); }
};

// `plot_px` is the plot's width in pixels and `min_px` the closest two centres
// may be before the overlay declines to draw. min_px is a threshold on
// *readability*, not on correctness -- see too_dense.
//
// The default was 6 px and that was measured wrong by looking at it: 149
// segments across a 1340 px plot is 9 px apart, which passed, and the result is
// a picket fence that hides the trace it annotates. A line only reads as an
// annotation with clear space either side of it, and 20 px is where that starts.
SegmentOverlay build_overlay(const std::vector<SegmentBand> &bands, double x_lo,
                             double x_hi, double plot_px, double min_px = 20.0);

}  // namespace sweepui
