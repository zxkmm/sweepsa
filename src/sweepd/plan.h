// The segment plan: where the LO goes, what is kept, how the DC hole is
// covered. CLAUDE.md §5.
//
// Split out of profiles.h, which holds the plan *and* the fastlock profile
// table built at each planned LO. The split is not cosmetic. profiles.cpp opens
// files and reads sysfs, so the library it compiles into is POSIX-only; the
// arithmetic here touches nothing, and it is now needed by a host-side source
// that must also build on Windows (§6). netsource.cpp carries its own socket
// shim for the same reason, rather than linking the POSIX-only sweepd_server.
//
// ---------------------------------------------------------------- who uses it
//
//   - sweepd's sweep loop, where each planned LO becomes a fastlock profile
//     (sweep.cpp, profiles.cpp);
//   - sweepui's RTL-SDR source, where each planned LO becomes a host-side
//     retune over USB (rtlsdrsource.cpp);
//   - sweepui's §7 segment-boundary overlay, which reconstructs the grid from
//     what `config` reports and is checked against *this* code in
//     tests/segments_test.cpp.
//
// Three readings of "where are the segment centres" would be three chances to
// disagree, and a disagreement here is not an error -- it is a plausible trace
// with its artifacts attributed to the wrong frequencies, which is the §7
// failure the overlay exists to prevent.
//
// -------------------------------------------------------------------- DC
//
// §5 asks for "keep 40 MHz of a 56 MHz segment, blank +/-100 kHz around DC,
// splice the gap from the neighbouring segment". Taken literally with segment
// centers spaced one kept-width apart, that does not work: the blanked hole sits
// at the *center* of the segment's own kept band, and the neighbours' kept bands
// end exactly where this one's begins, so no neighbour ever overlaps the hole.
// There is nothing to splice from.
//
// Splicing requires the centers to be spaced `keep/2`, so that segment k's DC
// hole falls at the shared edge of segments k-1 and k+1 and is covered from both
// sides. That is DcPolicy::SpliceOverlap, the default. It costs segments: net
// progress per hop is keep/2 rather than keep, so a full 70 MHz - 6 GHz sweep at
// keep=40 MHz needs ~298 segments rather than the 149 assumed in
// docs/characterization.md "Derived sweep budget". Since the sweep is
// fastlock_load-bound at ~780 us/segment, that is ~232 ms/sweep, not ~124 ms.
//
// DcPolicy::BlankGap is the alternative: space centers a full keep apart, accept
// a 2*blank_hz hole at every segment center, and mark those bins invalid so they
// are visibly missing rather than quietly wrong (§7). Twice as fast, with holes.

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace sweepd {

// A plan with millions of segments is a mistyped config, not a sweep. Refuse it
// rather than spending an hour in the cold build finding out. Shared with the
// profile table's cache loader, so a corrupt header cannot ask for a billion
// segments either.
constexpr size_t kMaxSegments = 20000;

// ---------------------------------------------------------------- plan

enum class DcPolicy {
    // Centers spaced keep/2. Every DC hole is covered by both neighbours.
    SpliceOverlap,
    // Centers spaced keep. Holes remain; the sweep marks them invalid.
    BlankGap,
};

const char *dc_policy_name(DcPolicy p);
bool dc_policy_from_string(const std::string &s, DcPolicy *out);

// Everything that determines the LO positions. All of it is config, per the
// owner decision that nothing needing external gear or bounded by the system
// gets measured once and hardcoded.
struct PlanConfig {
    long long f_start_hz = 70000000LL;
    long long f_stop_hz = 6000000000LL;

    // Fixed for the whole sweep (§5 hard rule): changing either reconfigures the
    // BBPLL and invalidates every profile in the table.
    long long fs_hz = 61440000LL;
    long long bw_hz = 56000000LL;

    // Usable fraction of each segment, centred on the LO. Kept well inside
    // bw_hz so the kept band's edges are not on the analog filter's skirt.
    long long keep_hz = 40000000LL;

    // Half-width blanked around DC, covering the LO leakage spike.
    long long blank_hz = 100000LL;

    DcPolicy dc = DcPolicy::SpliceOverlap;

    // §7's image check: a phase offset applied to the whole LO grid, so that
    // every RF frequency is received at a *different* baseband offset than it
    // was at shift 0.
    //
    // Direct conversion mirrors a tone about the LO: a real signal at F sits at
    // baseband F - lo, and uncorrected quadrature leaks a copy to -(F - lo),
    // which the stitch then maps to the apparent frequency 2*lo - F. Move the
    // grid by S and the owning LO moves with it, so the *image* moves by 2S
    // while the real signal does not move at all. That difference is the whole
    // measurement (§7: "re-sweep with the LO offset inverted and flag bins that
    // move"), and sweepui/imagecheck.h is what reads it.
    //
    // S = step/2 is the useful value: it inverts the sign of every bin's
    // baseband offset, which is the strongest version of the test, and it moves
    // an image by exactly one step_hz. Restricted to |S| < step_hz, because
    // beyond that the grid is simply a different grid and the comparison has
    // nothing to do with images.
    //
    // Not part of a sweep anyone measures with: it is a second pass, and the
    // profile table has to be rebuilt for it because the LOs are different
    // positions. Hence it is in the cache key.
    long long lo_shift_hz = 0;

    // Puts the grid's origin on a multiple of this many Hz (0 = off), before
    // lo_shift_hz is applied. Where an LO sits decides the RX synthesizer's
    // fractional-N word; on the E310's 80 MHz PFD an LO on a multiple of
    // 40 MHz is integer-N in every VCO band, where every LO of the default
    // grid (odd multiples of 10 MHz) has a word of 1/4, 1/2 or 3/4. Built to
    // test that as the cause of a spur at 3280.5 MHz; it made no measurable
    // difference (docs/characterization.md, "The 3280.5 MHz spur, and an
    // integer-N LO grid that did not help"), so it stays off. Only the origin
    // is snapped, so every LO lands on the grid only when step_hz is itself a
    // multiple of it.
    long long lo_grid_hz = 0;

    // Hard tuner limits. The plan places LOs up to keep/2 beyond the requested
    // range so the endpoints are covered by a segment on each side, and those
    // outriggers can fall off the end of what the synthesizer accepts -- at
    // f_stop = 6 GHz the last center wants 6.01 GHz and the driver rejects it
    // with EINVAL mid-build. Clamped here instead.
    //
    // Defaults are this part's published range; read them from the driver's
    // out_altvoltage0_RX_LO_frequency_available rather than trusting these.
    long long lo_min_hz = 70000000LL;
    long long lo_max_hz = 6000000000LL;
};

struct Segment {
    // What the plan asked for, kept so the cold build's read-back can be
    // audited against it rather than quietly replacing it.
    long long lo_request_hz = 0;

    // LO position actually in use: the plan's request until the cold build
    // overwrites it with the driver's read-back. Everything that maps a bin to
    // an RF frequency uses this one.
    long long lo_hz = 0;

    // RF edges of the kept band: [lo - keep/2, lo + keep/2].
    long long keep_lo_hz = 0;
    long long keep_hi_hz = 0;

    // RF edges of the blanked hole: [lo - blank, lo + blank].
    long long blank_lo_hz = 0;
    long long blank_hi_hz = 0;
};

struct SweepPlan {
    PlanConfig cfg;
    long long step_hz = 0;          // LO spacing; net coverage per hop
    std::vector<Segment> segments;

    // Where the LO grid starts, *before* the tuner clamp. Kept because it is
    // what regenerates the grid, and it cannot be recovered from the segments:
    // segments.front().lo_hz is the clamped value, and stepping from there
    // reproduces the plan only when (lo_min - f_start) happens to be a multiple
    // of step_hz. §7's segment-boundary overlay draws the grid in another
    // process, so the un-clamped origin goes on the wire rather than being
    // inferred there from a formula that is right most of the time.
    long long first_lo_hz = 0;

    // Union of the kept bands. Wider than [f_start, f_stop] by up to keep/2 at
    // each end, because the endpoints have to be reachable without relying on a
    // segment that does not exist -- unless the tuner limits got in the way, in
    // which case this is narrower than requested and says so.
    long long covered_lo_hz = 0;
    long long covered_hi_hz = 0;

    // Set when the tuner limits stopped the plan reaching the requested range.
    // The sweep still runs; it just does not cover what was asked for, and that
    // has to reach the user rather than being inferred from the axis.
    bool truncated = false;
};

// Kept and blanked edges follow the LO, so they are recomputed whenever the LO
// changes -- which it does once, when the cold build reads back what the driver
// actually tuned to. Exposed rather than file-local because the cold build and
// the cache loader both re-derive them after overwriting lo_hz.
void set_segment_edges(Segment *s, long long keep_hz, long long blank_hz);

// Rejects a config it cannot honour rather than silently producing a plan with
// gaps: keep_hz must fit inside both fs_hz and bw_hz, and the blanked hole must
// be small enough for a neighbour to cover.
bool plan_sweep(const PlanConfig &cfg, SweepPlan *out, std::string *err);

// LO spacing the policy implies, without building a plan. plan_sweep() uses it
// too, so the two cannot disagree -- which matters because the image check's
// shift is defined in terms of it and a second copy of the rule here would
// quietly phase the grid by the wrong amount under dc_policy=blank.
long long plan_step_hz(const PlanConfig &cfg);

// The shift §7's image check wants: half a step, which inverts the sign of every
// bin's baseband offset and therefore displaces every mirror image by exactly
// one full step. See PlanConfig::lo_shift_hz.
long long image_check_shift_hz(const PlanConfig &cfg);

// True if the plan leaves no unmeasured frequency inside [f_start, f_stop].
// False under BlankGap, which is the point of that policy.
bool plan_is_gapless(const SweepPlan &plan);

}  // namespace sweepd
