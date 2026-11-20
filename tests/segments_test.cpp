// Host-side tests for §7's segment boundary overlay.
//
// The overlay reconstructs the daemon's LO grid from what `config` reports,
// which makes it a second implementation of sweepd::plan_sweep's geometry living
// in a different process. That is the whole risk here, and it is a quiet one:
// a drifted grid draws seams and centres at frequencies that look entirely
// reasonable, and would then attribute the +4.5 dB splice comb to the wrong
// bins -- §7's failure mode exactly, one level up from an uncalibrated axis.
//
// So the centrepiece is the **drift guard**: this test links the real
// plan_sweep, runs it over a spread of configs including the ones where the
// tuner clamp bites and an outrigger collapses, and compares every edge of every
// segment against the reconstruction. The rest pins the things the guard cannot
// see, because they do not exist on the daemon side at all: ownership, the
// blanked-span lookup, and the density cutoff.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "../src/sweepd/plan.h"
#include "../src/sweepui/segments.h"

using namespace sweepui;

namespace {

int g_failures = 0;

void ok(const char *what, bool pass) {
    std::printf("  %-66s %s\n", what, pass ? "ok" : "FAIL");
    if (!pass) ++g_failures;
}

void near(const char *what, double got, double want, double tol = 1e-6) {
    const bool pass = std::fabs(got - want) <= tol;
    std::printf("  %-46s %14.1f want %14.1f %s\n", what, got, want, pass ? "ok" : "FAIL");
    if (!pass) ++g_failures;
}

// The daemon's side of the wire: everything §7's overlay is given, built from a
// real plan the same way src/sweepd/main.cpp builds the config message. Keeping
// this in one place is what makes the drift guard a test of the *reconstruction*
// rather than of a hand-written info struct.
SegmentPlanInfo info_from(const sweepd::SweepPlan &plan) {
    SegmentPlanInfo i;
    i.first_lo_hz = double(plan.first_lo_hz);
    i.step_hz = double(plan.step_hz);
    i.keep_hz = double(plan.cfg.keep_hz);
    i.blank_hz = double(plan.cfg.blank_hz);
    i.tune_min_hz = double(plan.cfg.lo_min_hz);
    i.tune_max_hz = double(plan.cfg.lo_max_hz);
    i.covered_lo_hz = double(plan.covered_lo_hz);
    i.covered_hi_hz = double(plan.covered_hi_hz);
    i.n_segments = uint32_t(plan.segments.size());
    i.splice = plan.cfg.dc == sweepd::DcPolicy::SpliceOverlap;
    return i;
}

// ------------------------------------------------------------- drift guard

void compare_against_plan(const char *label, const sweepd::PlanConfig &cfg) {
    sweepd::SweepPlan plan;
    std::string err;
    if (!sweepd::plan_sweep(cfg, &plan, &err)) {
        std::printf("  %-66s %s\n", label, "FAIL (plan_sweep refused)");
        std::printf("    %s\n", err.c_str());
        ++g_failures;
        return;
    }

    std::vector<SegmentBand> bands;
    if (!reconstruct_segments(info_from(plan), &bands, &err)) {
        std::printf("  %-66s %s\n", label, "FAIL (reconstruction refused)");
        std::printf("    %s\n", err.c_str());
        ++g_failures;
        return;
    }

    size_t bad = 0;
    long long worst = 0;
    if (bands.size() != plan.segments.size()) {
        bad = 1;
    } else {
        for (size_t i = 0; i < bands.size(); ++i) {
            const sweepd::Segment &s = plan.segments[i];
            const SegmentBand &b = bands[i];
            const long long d[4] = {
                (long long)std::llround(b.lo_hz) - s.lo_hz,
                (long long)std::llround(b.keep_lo_hz) - s.keep_lo_hz,
                (long long)std::llround(b.keep_hi_hz) - s.keep_hi_hz,
                (long long)std::llround(b.blank_lo_hz) - s.blank_lo_hz,
            };
            for (long long e : d) {
                if (e != 0) ++bad;
                if (std::llabs(e) > std::llabs(worst)) worst = e;
            }
        }
    }
    std::printf("  %-52s %3zu seg %s\n", label, plan.segments.size(),
                bad == 0 ? "ok" : "FAIL");
    if (bad) {
        std::printf("    %zu edges differ, worst %lld Hz (reconstructed %zu segments)\n", bad,
                    worst, bands.size());
        ++g_failures;
    }
}

void test_drift_guard() {
    std::printf("drift guard: the reconstruction against the real plan_sweep\n");

    sweepd::PlanConfig base;   // the defaults: 70 MHz - 6 GHz, keep 40M, blank 100k, splice

    compare_against_plan("full span, splice (the operating configuration)", base);

    sweepd::PlanConfig blank = base;
    blank.dc = sweepd::DcPolicy::BlankGap;
    compare_against_plan("full span, blank", blank);

    // The narrow span the whole project is validated at.
    sweepd::PlanConfig narrow = base;
    narrow.f_start_hz = 900000000LL;
    narrow.f_stop_hz = 1100000000LL;
    compare_against_plan("900-1100 MHz, splice", narrow);

    // The clamp at the top: the plan wants a centre at f_stop + keep/2 = 6.02 GHz
    // and the driver will not take it, so the outrigger collapses onto lo_max and
    // is deduplicated away. This is the case that a reconstruction stepping from
    // covered_lo_hz gets right by luck and this one gets right by construction.
    sweepd::PlanConfig top = base;
    top.f_start_hz = 5900000000LL;
    top.f_stop_hz = 6000000000LL;
    compare_against_plan("5.9-6.0 GHz, splice (top outrigger clamped)", top);

    // The clamp at the bottom, and deliberately *not* on a step boundary:
    // lo_min - f_start is 13 MHz against a 20 MHz step, which is exactly the
    // offset that makes "step from the first kept edge" diverge from the plan.
    sweepd::PlanConfig bot = base;
    bot.f_start_hz = 57000000LL;
    bot.f_stop_hz = 400000000LL;
    compare_against_plan("57-400 MHz, splice (bottom clamp, off-step)", bot);

    sweepd::PlanConfig bot_blank = bot;
    bot_blank.dc = sweepd::DcPolicy::BlankGap;
    compare_against_plan("57-400 MHz, blank (bottom clamp, off-step)", bot_blank);

    // An odd keep_hz, where the daemon's `keep_hz / 2` truncates. The
    // reconstruction does not model that truncation -- it derives the half-width
    // from covered_lo_hz, where the daemon has already applied it -- and this is
    // what proves that is equivalent rather than merely close.
    sweepd::PlanConfig odd = narrow;
    odd.keep_hz = 33333333LL;
    compare_against_plan("900-1100 MHz, keep 33333333 Hz (odd, halves truncate)", odd);

    // §7's image check runs the same overlay over a *phased* grid, and the
    // reconstruction has no special case for it -- it steps from first_lo_hz,
    // which the daemon already shifted. That is exactly the situation where a
    // reconstruction inferring the origin from covered_lo_hz would drift, so
    // both policies get a shifted pass here.
    sweepd::PlanConfig shifted = narrow;
    shifted.lo_shift_hz = sweepd::image_check_shift_hz(shifted);
    compare_against_plan("900-1100 MHz, splice, LO grid phased by half a step", shifted);

    sweepd::PlanConfig shifted_neg = narrow;
    shifted_neg.lo_shift_hz = -sweepd::image_check_shift_hz(shifted_neg);
    compare_against_plan("900-1100 MHz, splice, phased the other way", shifted_neg);

    sweepd::PlanConfig shifted_bot = bot;
    shifted_bot.lo_shift_hz = sweepd::image_check_shift_hz(shifted_bot);
    compare_against_plan("57-400 MHz, splice, phased (bottom clamp and a shift)",
                         shifted_bot);

    sweepd::PlanConfig shifted_blank = narrow;
    shifted_blank.dc = sweepd::DcPolicy::BlankGap;
    shifted_blank.lo_shift_hz = sweepd::image_check_shift_hz(shifted_blank);
    compare_against_plan("900-1100 MHz, blank, phased", shifted_blank);

    // A single segment: no neighbours, no seams, and the loop's dedup and the
    // covered checks all have to survive n == 1.
    sweepd::PlanConfig one = base;
    one.f_start_hz = 1000000000LL;
    one.f_stop_hz = 1000000001LL;
    one.dc = sweepd::DcPolicy::BlankGap;
    compare_against_plan("a one-segment sweep", one);
}

// --------------------------------------------------------------- refusals

void test_refusals() {
    std::printf("refusals: an overlay that might be wrong is not drawn\n");

    std::vector<SegmentBand> bands;
    std::string err;

    SegmentPlanInfo empty;
    ok("a source reporting no plan is refused",
       !reconstruct_segments(empty, &bands, &err) && bands.empty());
    std::printf("    \"%s\"\n", err.c_str());

    sweepd::SweepPlan plan;
    sweepd::PlanConfig cfg;
    cfg.f_start_hz = 900000000LL;
    cfg.f_stop_hz = 1100000000LL;
    std::string perr;
    plan_sweep(cfg, &plan, &perr);

    // Each of these is a plausible single-field disagreement between the daemon
    // and the receiver -- the shape a wire-format drift actually takes. None may
    // produce a grid.
    // One too many: the loop happily builds the extra band, so it is the far
    // covered edge that catches this and not the count. Worth naming, because
    // the two guards cover different failures and only one of them fires here.
    SegmentPlanInfo i = info_from(plan);
    i.n_segments += 1;
    ok("one segment too many is refused (by the far edge, not the count)",
       !reconstruct_segments(i, &bands, &err));
    std::printf("    \"%s\"\n", err.c_str());

    // Far too many, so the grid runs into the tuner limit and dedups: the loop
    // cannot produce that many bands and the count guard is what says so.
    i = info_from(plan);
    i.n_segments *= 40;
    ok("a count the grid cannot reach is refused by the count guard",
       !reconstruct_segments(i, &bands, &err) && err.find("segments, the source reports") != std::string::npos);
    std::printf("    \"%s\"\n", err.c_str());

    i = info_from(plan);
    i.step_hz *= 2;
    ok("a step that does not reach the covered range is refused",
       !reconstruct_segments(i, &bands, &err));
    std::printf("    \"%s\"\n", err.c_str());

    i = info_from(plan);
    i.covered_lo_hz += 5e6;
    ok("a covered range inconsistent with the segment width is refused",
       !reconstruct_segments(i, &bands, &err));
    std::printf("    \"%s\"\n", err.c_str());

    // Not a refusal, but the reason the loop is bounded at all: a zero step
    // would otherwise place every segment on top of the first, dedup them all
    // away and spin.
    i = info_from(plan);
    i.step_hz = 0;
    ok("a zero step is refused rather than looping", !reconstruct_segments(i, &bands, &err));
}

// -------------------------------------------------------------- ownership

void test_ownership() {
    std::printf("ownership: which segment a frequency's value came from\n");

    sweepd::PlanConfig cfg;
    cfg.f_start_hz = 900000000LL;
    cfg.f_stop_hz = 1100000000LL;
    sweepd::SweepPlan plan;
    std::string err;
    plan_sweep(cfg, &plan, &err);

    std::vector<SegmentBand> b;
    ok("the validation sweep reconstructs", reconstruct_segments(info_from(plan), &b, &err));
    if (b.size() < 3) return;

    // Under splice the step is keep/2, so ownership crosses at keep/4 from each
    // LO -- which is the 10 MHz figure the seam flatness was measured at
    // (PROJECT-STATE.md: "the seams at 10 MHz offset are flat to +/-1 dB").
    near("crossover is a quarter of keep from the LO", b[1].own_hi_hz - b[1].lo_hz,
         double(cfg.keep_hz) / 4.0, 1.0);

    ok("ownership partitions the covered range with no gap",
       std::fabs(b[1].own_hi_hz - b[2].own_lo_hz) < 1e-6);
    ok("the first segment owns out to its kept edge",
       b.front().own_lo_hz == b.front().keep_lo_hz);
    ok("the last segment owns out to its kept edge",
       b.back().own_hi_hz == b.back().keep_hi_hz);
    ok("no segment owns outside what it measured",
       b[1].own_lo_hz >= b[1].keep_lo_hz && b[1].own_hi_hz <= b[1].keep_hi_hz);

    // A frequency just inside segment 2's half is segment 2's.
    const int s = segment_at(b, b[2].lo_hz + 1e6);
    ok("a frequency near an LO belongs to that segment", s == 2);
    ok("a frequency below the covered range has no segment",
       segment_at(b, b.front().keep_lo_hz - 1e6) < 0);
    ok("a frequency above the covered range has no segment",
       segment_at(b, b.back().keep_hi_hz + 1e6) < 0);

    // The point of the whole exercise: a bin at a segment centre is *not* what
    // that segment measured. Its owner blanked it, so the value came from a
    // neighbour keep/2 away -- the +4.5 dB fill.
    ok("a segment centre reads as blanked/spliced fill", in_blanked_span(b, b[2].lo_hz));
    ok("a frequency a megahertz off a centre does not",
       !in_blanked_span(b, b[2].lo_hz + 1e6));

    // The hole is blank_hz either side, not blank_hz wide. Reading the wire
    // field as a full width would put this one outside.
    ok("the hole extends blank_hz on each side of the centre",
       in_blanked_span(b, b[2].lo_hz + double(cfg.blank_hz) * 0.9) &&
           !in_blanked_span(b, b[2].lo_hz + double(cfg.blank_hz) * 1.1));

    // A centre is on the boundary between two owners, so whichever one claims it
    // must still see the hole. This is what the neighbour scan in
    // in_blanked_span() is for; testing only the owner's own span would pass on
    // one side of the tie and fail on the other.
    ok("the hole is found from either side of the ownership boundary",
       in_blanked_span(b, b[2].lo_hz - 1.0) && in_blanked_span(b, b[2].lo_hz + 1.0));
}

// ----------------------------------------------------------------- overlay

void test_overlay() {
    std::printf("overlay: culling, density and what gets drawn\n");

    sweepd::PlanConfig cfg;   // full span: 298 segments at the defaults
    sweepd::SweepPlan plan;
    std::string err;
    plan_sweep(cfg, &plan, &err);
    std::vector<SegmentBand> b;
    reconstruct_segments(info_from(plan), &b, &err);
    ok("full span reconstructs", b.size() > 200);
    if (b.empty()) return;

    // 298 centres across 1600 px is ~5 px apart. Drawing that is a picket fence
    // over the trace carrying no information, so it is declined and said.
    const SegmentOverlay full = build_overlay(b, 70e6, 6e9, 1600.0);
    ok("a full span at 1600 px is too dense to draw", full.too_dense);
    // The threshold as it is actually set, pinned at the boundary rather than
    // trusted: 149 blank-policy segments at 9 px apart passed the original 6 px
    // default and produced exactly the fence the check is for.
    ok("  9 px between centres is still too dense",
       build_overlay(b, b[0].lo_hz, b[0].lo_hz + 9.0 * (b[1].lo_hz - b[0].lo_hz), 81.0)
           .too_dense);
    ok("  30 px between centres draws",
       !build_overlay(b, b[0].lo_hz, b[0].lo_hz + 9.0 * (b[1].lo_hz - b[0].lo_hz), 270.0)
            .too_dense);
    ok("  and still reports how many segments are in view", full.n_in_view == b.size());
    ok("  and emits nothing rather than a wall of lines", full.empty());

    // Zoomed to a few segments, everything is drawn and only what is visible.
    const double x0 = b[10].keep_lo_hz, x1 = b[14].keep_hi_hz;
    const SegmentOverlay z = build_overlay(b, x0, x1, 1600.0);
    ok("zoomed in, the overlay draws", !z.too_dense && !z.empty());
    ok("  only the visible centres are emitted", z.centres.size() >= 5 && z.centres.size() <= 9);
    ok("  centres, their segment indices and their holes stay parallel",
       z.centres.size() == z.centre_seg.size() && z.centres.size() == z.blank_lo.size() &&
           z.centres.size() == z.blank_hi.size());
    bool in_view = true;
    for (double c : z.centres) in_view = in_view && c >= x0 && c <= x1;
    ok("  every emitted centre is inside the view", in_view);
    ok("  the segment indices point back at the right bands",
       !z.centres.empty() && b[size_t(z.centre_seg[0])].lo_hz == z.centres[0]);

    // One seam per adjacent pair, never two at the same frequency: the seam
    // between k and k+1 is emitted by k alone. Drawn twice it is twice as dark
    // as the centres it is supposed to be subordinate to.
    bool distinct = true;
    for (size_t i = 1; i < z.seams.size(); ++i)
        distinct = distinct && z.seams[i] > z.seams[i - 1];
    ok("  seams are distinct and ascending", distinct);
    ok("  there is about one seam per visible centre",
       z.seams.size() + 1 >= z.centres.size() && z.seams.size() <= z.centres.size() + 1);

    // A view entirely off the end of the sweep.
    const SegmentOverlay none = build_overlay(b, 6.2e9, 6.4e9, 1600.0);
    ok("a view outside the sweep draws nothing", none.empty() && none.n_in_view == 0);

    // Degenerate inputs a resizing window really does produce.
    ok("a zero-width plot draws nothing", build_overlay(b, 1e9, 2e9, 0.0).empty());
    ok("an inverted view draws nothing", build_overlay(b, 2e9, 1e9, 1600.0).empty());
    ok("no bands draws nothing", build_overlay({}, 1e9, 2e9, 1600.0).empty());
}

}  // namespace

int main() {
    test_drift_guard();
    test_refusals();
    test_ownership();
    test_overlay();
    std::printf("%s\n", g_failures ? "FAILURES" : "all ok");
    return g_failures ? 1 : 0;
}
