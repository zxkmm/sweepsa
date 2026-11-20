// Marking the receiver's own responses: LO leakage, the DC splice, LO-relative
// spurs and whatever the box radiates into itself.
//
// §7 says an uncalibrated analyzer that looks calibrated is worse than useless.
// The same goes for one whose own artifacts look like signals, and on this
// hardware they are most of what a quiet band shows: the full-span spike
// budget (PROJECT-STATE.md, memory "full-span-spike-budget") found 371 bins
// more than 6 dB over their local floor with nothing connected, 53% of them on
// segment centres and the loudest ten on LO +/- f_ref/8. The image check
// (imagecheck.h) can classify those, but it is a two-pass measurement that
// phases the LO grid and costs a profile rebuild; nobody runs it continuously.
// This file is the continuous half: it runs on the averaged live trace, on the
// grid that is actually in use, and marks bins as *suspect* -- never removes
// them.
//
// ---------------------------------------------------------- four tests
//
//   1. **DC hole (geometry).** Bins inside a segment's blanked span. That is
//      where LO leakage lands, and under dc_policy=splice the value there is a
//      neighbour's data from keep/2 off its own LO, measured +4.5 dB hot.
//      Needs no data at all: the plan says where they are.
//
//   2. **LO comb (fold).** Every bin is re-expressed as an offset from the LO
//      of the segment that owns it, and the trace is folded on that offset.
//      The fold's per-offset *median* excursion across segments is the
//      receiver's own shape around its LO -- the +3 dB hump within a few MHz of
//      DC, a fixed spur present in every segment -- because the air does not
//      know where the LO grid is and cannot put energy at the same offset in
//      most segments at once. Median, so a real signal that happens to land at
//      that offset in a minority of segments does not move it.
//
//   3. **LO spur (recurrence).** The f_ref/8 spurs are not in *most*
//      segments -- about ten of 298 at full span -- so the median cannot see
//      them. What it can see is that a narrow peak recurs at the same |offset|
//      in segments scattered across the span. Two guards keep real signals out:
//      segments closer than `cluster_gap_hz` count once (a band of real
//      carriers on a raster commensurate with the LO step is contiguous, a
//      synthesizer spur is not), and the count must be improbable against the
//      number of peaks on screen (a crowded span produces coincidences, and the
//      threshold rises with the crowd rather than flagging all of it).
//
//   4. **Baseline (terminated input).** The one test that catches spurs at a
//      fixed *RF* frequency -- reference and clock harmonics, Ethernet,
//      whatever the Zynq radiates -- which neither fold can, because they do
//      not move with the LO. The user terminates the RX input and records a
//      baseline; a live bin standing out where the baseline also stood out,
//      and not appreciably above the baseline's own level, is marked internal.
//      A strong real signal on top of an internal spur exceeds the baseline's
//      level and is *not* marked, which is the direction to err in.
//
// ------------------------------------------------ what "marked" promises
//
// A flag is "this response is likely the receiver", with the test that said so.
// It is not a verdict that nothing real is there: a genuine carrier sitting on
// an LO offset where a spur also sits is indistinguishable from the spur by
// these tests. That is why nothing is hidden or subtracted -- the trace is
// untouched, the flags ride beside it -- and why every result carries a caveat
// sentence the UI has to show.
//
// Levels are never compared between segments, for the reason imagecheck.h
// gives: fastlock's recall amplitude depends on the predecessor LO, up to
// 6.3 dB. Every test works on excursion above a local floor clipped to the
// owning segment, so a per-segment offset cancels.
//
// No ImGui, no GL, so all of it is host-tested.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "imagecheck.h"  // ImagePass, local_excursion
#include "marker.h"      // TraceAxis
#include "segments.h"    // SegmentBand

namespace sweepui {

// Per-bin flags. A bin can carry more than one.
enum ArtifactFlag : uint8_t {
    kArtDcHole = 1u << 0,    // inside a blanked span: LO leakage / splice fill
    kArtLoComb = 1u << 1,    // an offset most segments stand out at
    kArtLoSpur = 1u << 2,    // a peak recurring at one LO offset across the span
    kArtInternal = 1u << 3,  // present with the input terminated
};

// Short tag for the most specific flag set, for a marker label. Empty for 0.
const char *artifact_tag(uint8_t flags);
// One sentence saying why, for a tooltip.
const char *artifact_explain(uint8_t flags);

// Every threshold is a judgement about an installation's floor and signal
// population, so all of them are config (memory "no-measure-once-hardcode").
// The defaults are justified in the comments, not measured into constants.
struct ArtifactConfig {
    // Fold median, in dB over the local floor, above which an offset is the
    // receiver's own. The measured DC hump is +3.1 dB at full span and +1.7 dB
    // at 19 segments; 2 dB catches the first without flagging averaged noise,
    // whose median excursion is ~0 by construction.
    double comb_db = 2.0;

    // Fewer segments than this contributing to an offset and the fold does not
    // judge it. A median of five is a median of whatever five things happened.
    size_t min_fold_segments = 8;

    // A peak must stand this far over its local floor to be a spur candidate.
    // Same default as the image check's min_excursion, for the same reason.
    double spur_db = 8.0;

    // Distinct clusters a recurrence must reach, and the gap that separates
    // clusters. 100 MHz is wider than any contiguous allocation of real
    // carriers that sits on an LO-commensurate raster within one band.
    size_t min_recur = 3;
    double cluster_gap_hz = 100e6;

    // How improbable the recurrence has to be against chance coincidence of
    // the peaks on screen (Poisson tail), for the whole span: it is divided
    // across the offset windows tested. 1e-3 flags nothing on 300 random
    // carriers at full span; see the tests.
    double recur_alpha = 1e-3;

    // Baseline: how far over its own floor a terminated-input bin must stand to
    // count as internal, and how far above the baseline's own level a live bin
    // may sit and still be called that same internal response.
    double base_db = 6.0;
    double base_margin_db = 6.0;

    // LO leakage is not confined to the blanked hole. Fastlock carries no DC
    // offset calibration, so the residual varies per segment and its skirt
    // reaches past blank_hz: on the E310 at full span, 2026-09-25, peaks up to
    // +10 dB stood 0.4 and 0.86 MHz off an LO with the hole at +/-100 kHz.
    // Within this distance of an LO, a bin standing spur_db over its floor is
    // marked LO/DC. 0 confines the tag to the hole itself.
    double lo_guard_hz = 1e6;

    // Median half-width in output bins; 0 derives it from the segment width,
    // exactly as the image check does.
    size_t floor_halfwidth = 0;

    size_t max_findings = 256;
};

// A recorded terminated-input sweep. Carries what it was taken at, because a
// baseline from another gain or another LO grid is a weaker statement and the
// result has to say so.
struct ArtifactBaseline {
    ImagePass pass;          // averaged dB, axis, segments, sweeps
    double gain_db = 0;
    double seg_first_lo_hz = 0;
    double seg_step_hz = 0;

    bool valid() const { return pass.axis.valid() && pass.db.size() == pass.axis.n; }
};

// CSV round trip, so a baseline is measured once per installation rather than
// once per session (terminating the input is the annoying part). Directives on
// `#` lines, like the amplitude correction file.
bool save_baseline(const std::string &path, const ArtifactBaseline &b, std::string *err);
bool load_baseline(const std::string &path, ArtifactBaseline *b, std::string *err);

struct ArtifactFinding {
    double x_hz = 0;
    size_t bin = 0;
    uint8_t flags = 0;
    float excursion_db = 0;     // above the local floor, live trace
    double lo_offset_hz = 0;    // from the owning segment's LO; 0 if unknown
    size_t recur_segments = 0;  // LO spur: segments showing it
    size_t recur_clusters = 0;  // ... and how many separate places that is
    size_t width_bins = 1;
};

// One run of elevated offsets in the fold, reported at its strongest offset.
struct CombOffset {
    double offset_hz = 0;       // signed: RF minus the owning LO
    double width_hz = 0;        // extent of the run
    float median_db = 0;        // fold median at offset_hz
    size_t segments = 0;        // how many contributed there
};

struct ArtifactResult {
    bool valid = false;
    std::string err;

    // Parallel to the live trace. Empty unless valid.
    std::vector<uint8_t> flags;
    TraceAxis axis;

    // LO spurs and internal responses, strongest first. The DC holes and the
    // comb are one statement per *offset*, not per segment, so they are counted
    // and listed in `comb` rather than repeated 298 times here.
    std::vector<ArtifactFinding> findings;
    size_t dropped = 0;
    std::vector<CombOffset> comb;

    size_t n_dc = 0, n_comb = 0, n_spur = 0, n_internal = 0;  // bins

    // Peaks at least spur_db over their floor that no test explained,
    // strongest first, capped at max_findings. Not a verdict that they are
    // signals -- only that nothing here says they are the receiver. Listed so
    // "8 self-spurs" is never read as "and everything else is real".
    std::vector<ArtifactFinding> unmarked;

    bool segments_known = false;
    // False when there were too few segments or too few bins per segment for
    // the folds to mean anything. Tests 2 and 3 were then not run.
    bool fold_ran = false;
    std::string fold_note;
    bool baseline_used = false;
    std::string baseline_note;  // mismatch against the live conditions

    std::string caveat;  // never empty when valid
};

// `live` is an averaged pass over the grid in use (PassAccumulator gives one).
// `base` may be null. `live_gain_db` is compared against the baseline's.
ArtifactResult mark_artifacts(const ImagePass &live, const ArtifactBaseline *base,
                              double live_gain_db, const ArtifactConfig &cfg);

// Upper tail of a Poisson distribution, P(X >= k | lambda). Exposed for the
// tests: the recurrence threshold is only as honest as this.
double poisson_tail(size_t k, double lambda);

}  // namespace sweepui
