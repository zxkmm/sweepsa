// §7's image check: which responses on screen are not signals.
//
// CLAUDE.md §7 asks for an "image check -- re-sweep with the LO offset inverted
// and flag bins that move. Those are images, not signals. Partially substitutes
// for the missing preselector."
//
// -------------------------------------------------------------- the physics
//
// The receiver is direct conversion. A real signal at F is captured at baseband
// F - lo, and uncorrected quadrature leaks a copy of it to -(F - lo), which the
// stitch then maps back to the apparent frequency 2*lo - F. So every real
// signal can paint a second, false response mirrored about the LO of whichever
// segment owns it -- and nothing in the trace distinguishes the two.
//
// Move the whole LO grid by S (sweepd's PlanConfig::lo_shift_hz) and the owner
// LO moves with it. The real signal is still at F. The image is now at
// 2*(lo + S) - F, which is 2S away from where it was. **Real signals stay,
// artifacts move**, and that is the entire measurement.
//
// sweepd offers the second grid; this file reads the difference. The comparison
// is here rather than in the daemon for the reason §7's amplitude correction is
// (ampcorr.h): it is a property of two *displayed* traces, it needs no device,
// and a daemon that sometimes shipped a "this bin is an image" bit would make
// the same 0x02 frame type mean two things.
//
// -------------------------------------------- why levels are never compared
//
// Not `A[i] - B[i]`. Recalled amplitude on this hardware depends on which LO
// position preceded the recall -- up to 6.3 dB, repeatable, mechanism unknown
// (docs/characterization.md, "Not preserved: amplitude"). The shifted pass hops
// through different LO positions, so every segment's level offset is different,
// and a 6.3 dB step at a segment boundary would outvote any threshold worth
// setting.
//
// So each pass is reduced to an **excursion above its own local floor**, by a
// running median narrow enough to sit inside one segment. A per-segment
// constant offset shifts a bin and its local floor together and cancels
// exactly; so does the front-end tilt, and so does a gain change between the
// passes. What survives is "how far this bin stands out", which is the only
// quantity the two passes can be expected to agree on.
//
// ---------------------------------------------------- the two false friends
//
//  1. **The splice comb.** Every segment centre reads ~4.5 dB hot, because the
//     blanked DC hole is filled from a neighbour at keep/2 from *its* LO
//     (PROJECT-STATE.md, "The splice fill is hot"). Those bins move with the
//     grid, exactly like an image. They are excluded by position: the blanked
//     spans of *both* grids are known, and the comb is precisely those bins.
//     A check that did not exclude them would flag one false image per segment
//     -- 298 of them at full span -- which is how a user learns to ignore a
//     feature.
//
//  2. **Synthesizer spurs.** The 20+ dB spikes at LO +/- f_ref/8 also move with
//     the grid. They are genuinely artifacts and deserve flagging, but they are
//     not images and calling them images would be its own small lie. They are
//     told apart by *how far* they moved: an image is displaced by 2S, an
//     LO-relative spur by S. Where the displaced copy is found, the verdict
//     says which; where it is not, the verdict says only that the bin moved.
//
// ------------------------------------------------- what this cannot do, ever
//
// The two passes are taken at different times. A burst transmitter that was on
// during pass A and off during pass B is indistinguishable from an image, and
// no amount of care in this file changes that -- it is why both passes should
// average many sweeps and why `min_sweeps` refuses a comparison built from too
// few. ImageCheckResult::caveat carries the sentence for the UI to display.
//
// No ImGui, no GL, no source dependency, so all of it is host-tested.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "marker.h"    // TraceAxis
#include "segments.h"  // SegmentBand

namespace sweepui {

// One averaged sweep, plus what is needed to know where its segments were.
struct ImagePass {
    std::vector<float> db;   // dBFS, NaN for unmeasured bins
    TraceAxis axis;

    // The LO grid this pass was taken on, as sweepd reported it. `segments` may
    // be empty, in which case the blanked spans cannot be excluded and the
    // result says so rather than pretending the comb is signal.
    std::vector<SegmentBand> segments;
    double lo_shift_hz = 0;

    // How many sweeps were averaged into `db`. Carried because it is the only
    // defence against the time-varying-signal failure above, and a result built
    // from one sweep per pass should not be presented like one built from 200.
    size_t sweeps = 0;
};

struct ImageCheckConfig {
    // How far above its own local floor a bin must stand before it is a
    // response worth classifying. Below this the comparison is comparing noise
    // to noise and will flag bins at whatever rate the thresholds allow.
    double min_excursion_db = 8.0;

    // How much of that excursion must vanish in the shifted pass before the bin
    // is called an artifact. Deliberately not small: the passes are minutes
    // apart on a live band.
    double move_db = 6.0;

    // Half-width of the running median that estimates the local floor, in
    // output bins. 0 derives it from the segment width, which is what keeps the
    // window inside one segment and therefore keeps the per-segment amplitude
    // offset out of the excursion.
    size_t floor_halfwidth = 0;

    // How far above its own local floor the displaced copy has to stand to count
    // as the same response reappearing. Only affects the *verdict*, never
    // whether a bin is flagged.
    //
    // Deliberately a presence test and **not** a level match. An image's
    // amplitude comes from the quadrature error at its own baseband offset, and
    // the shift is precisely a change of that offset -- so the copy routinely
    // reads several dB different from the original, and measurably does
    // (10.9 dB on the synthetic source at 15 dB image rejection). Requiring the
    // levels to agree would leave the commonest real image labelled "moved".
    // Position is the evidence here; level is not.
    double match_excursion_db = 4.0;

    // A pass averaged from fewer sweeps than this is refused. One sweep against
    // one sweep on a live band is a comparison of two different moments, not of
    // two grids.
    size_t min_sweeps = 4;

    // Findings past this are dropped and counted. A result with thousands of
    // entries is a threshold that is set wrong, not a receiver with thousands
    // of images, and rendering them all would hide that.
    size_t max_findings = 256;
};

enum class ImageVerdict {
    // The response reappeared 2S away: a mirror about the segment LO, which is
    // what §7 means by an image.
    Image,
    // It reappeared S away: an LO-relative spur -- the synthesizer's own, at a
    // fixed offset from wherever the LO is. An artifact, but not an image.
    LoSpur,
    // It went away and was not found at either displacement. Still not a signal
    // -- a signal does not care where the LO grid is -- but what it is instead
    // is not established by this measurement.
    Moved,
};

const char *image_verdict_name(ImageVerdict v);

// One response, reported at its strongest bin. A strong image spans several
// bins and they are one finding, not twenty.
struct ImageFinding {
    double x_hz = 0;
    size_t bin = 0;

    float excursion_a_db = 0;   // above the local floor, unshifted pass
    float excursion_b_db = 0;   // above the local floor, shifted pass, same frequency
    float drop_db = 0;          // a - b: how much of the response went away

    ImageVerdict verdict = ImageVerdict::Moved;
    // Where the displaced copy was found, for Image and LoSpur. 0 otherwise.
    double found_at_hz = 0;

    // The strongest excursion the shifted pass showed at *any* of the four
    // candidate displacements, and which one it was, whether or not it was
    // close enough to count as a match. Reported because "Moved" is otherwise
    // an unexplained verdict: this is the number that says whether the copy was
    // missed by a decibel or was genuinely not there.
    float best_candidate_db = 0;
    double best_candidate_hz = 0;

    // Bins either side of `bin` that were flagged with it.
    size_t width_bins = 1;
};

struct ImageCheckResult {
    bool valid = false;
    std::string err;

    std::vector<ImageFinding> findings;
    size_t dropped = 0;         // findings past max_findings

    size_t n_compared = 0;      // bins both passes measured
    size_t n_blanked = 0;       // bins skipped: inside a blanked span of either grid
    size_t n_uncovered = 0;     // bins the shifted pass did not measure

    double shift_hz = 0;        // S
    double floor_halfwidth = 0; // bins, as actually used

    // True when both grids were known and the splice comb could be excluded by
    // position. False means the result still stands for everything away from a
    // segment centre, and the centres are unclassifiable rather than clean.
    bool segments_known = false;

    // The sentence the UI has to show next to the verdicts. Never empty.
    std::string caveat;
};

// Bins in `pass` reduced to how far each stands above its own local floor.
// Exposed because it is the quantity the whole check is built on and a plot of
// it is the fastest way to see that a threshold is set wrong. NaN in, NaN out.
std::vector<float> local_excursion(const std::vector<float> &db, size_t halfwidth);

// The same, with the median window clipped to the segment that owns each bin.
//
// A median already survives a single seam: whichever segment the bin belongs to
// holds the majority of a window centred on it, so one step either side does not
// move the answer. What it does not survive is a window *wider than a segment* --
// at a wide span and a modest point count a segment is a few tens of bins, the
// window then spans several of them, and the median is taken across a mixture of
// fastlock's per-segment offsets. That is the error this clipping removes, and
// it is the one that scales with span. `owner` is the segment index per bin, -1
// where unknown, which falls back to the unclipped window.
std::vector<float> local_excursion(const std::vector<float> &db,
                                   const std::vector<int> &owner, size_t halfwidth);

// ---------------------------------------------------------------- one pass
//
// Accumulates sweeps into one pass. It is here rather than in app.cpp because
// what it does is arithmetic with a rule attached, and the rule is §5's:
//
//   **it averages linear power, not dB.** That is the fourth independent
//   implementation of that rule in this project (sweepd::detect, the sweep
//   loop's stitch, the display average, and now this), and getting it wrong
//   here would read the noise floor about 2.5 dB low in *both* passes -- which
//   cancels in the excursion and would therefore look entirely fine, right up
//   until it did not.
//
// NaN takes no part: a bin that was unmeasured in some sweeps is averaged over
// the sweeps that did measure it, and one never measured stays NaN.
class PassAccumulator {
public:
    // Starts a pass on a given geometry. Discards anything already held.
    void begin(const TraceAxis &ax, double lo_shift_hz);

    // False when `ax` is not the geometry this pass began on -- which is not a
    // detail to smooth over: bin i is then a different frequency than it was,
    // and an average across the change is a blend of two axes.
    bool matches(const TraceAxis &ax) const;

    // Adds one sweep. Returns false, and changes nothing, on a geometry
    // mismatch.
    bool add(const TraceAxis &ax, const float *db, size_t n);

    size_t sweeps() const { return sweeps_; }
    bool active() const { return ax_.valid(); }
    const TraceAxis &axis() const { return ax_; }

    ImagePass finish(const std::vector<SegmentBand> &segments) const;

private:
    TraceAxis ax_;
    double shift_ = 0;
    std::vector<double> sum_;     // linear power
    std::vector<uint32_t> cnt_;
    size_t sweeps_ = 0;
};

// `a` is the reference pass and the one the findings are reported against --
// the trace the user is looking at. `b` is the shifted pass.
ImageCheckResult run_image_check(const ImagePass &a, const ImagePass &b,
                                 const ImageCheckConfig &cfg);

}  // namespace sweepui
