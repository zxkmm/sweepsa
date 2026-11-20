#include "plan.h"

namespace sweepd {
namespace {

long long divide_round_up(long long a, long long b) {
    return (a + b - 1) / b;
}

}  // namespace

void set_segment_edges(Segment *s, long long keep_hz, long long blank_hz) {
    s->keep_lo_hz = s->lo_hz - keep_hz / 2;
    s->keep_hi_hz = s->lo_hz + keep_hz / 2;
    s->blank_lo_hz = s->lo_hz - blank_hz;
    s->blank_hi_hz = s->lo_hz + blank_hz;
}

// ---------------------------------------------------------------- plan

const char *dc_policy_name(DcPolicy p) {
    switch (p) {
        case DcPolicy::SpliceOverlap: return "splice";
        case DcPolicy::BlankGap: return "blank";
    }
    return "?";
}

bool dc_policy_from_string(const std::string &s, DcPolicy *out) {
    if (s == "splice") { *out = DcPolicy::SpliceOverlap; return true; }
    if (s == "blank") { *out = DcPolicy::BlankGap; return true; }
    return false;
}

long long plan_step_hz(const PlanConfig &cfg) {
    return cfg.dc == DcPolicy::SpliceOverlap ? cfg.keep_hz / 2 : cfg.keep_hz;
}

long long image_check_shift_hz(const PlanConfig &cfg) {
    return plan_step_hz(cfg) / 2;
}

bool plan_sweep(const PlanConfig &cfg, SweepPlan *out, std::string *err) {
    auto fail = [&](const std::string &m) {
        if (err) *err = m;
        return false;
    };

    if (cfg.f_stop_hz <= cfg.f_start_hz) return fail("f_stop must exceed f_start");
    if (cfg.fs_hz <= 0 || cfg.bw_hz <= 0) return fail("sample rate and bandwidth must be positive");
    if (cfg.keep_hz <= 0) return fail("keep_hz must be positive");
    if (cfg.blank_hz < 0) return fail("blank_hz cannot be negative");

    // The kept band spans +/- keep/2 around DC, so it has to fit in the digital
    // Nyquist span and inside the analog filter. Exceeding either does not
    // produce an error from the hardware -- it produces attenuated or aliased
    // data that looks like a real measurement.
    if (cfg.keep_hz > cfg.fs_hz)
        return fail("keep_hz " + std::to_string(cfg.keep_hz) + " exceeds the sample rate " +
                    std::to_string(cfg.fs_hz) + "; the kept band would alias");
    if (cfg.keep_hz > cfg.bw_hz)
        return fail("keep_hz " + std::to_string(cfg.keep_hz) + " exceeds rf_bandwidth " +
                    std::to_string(cfg.bw_hz) + "; the kept band's edges are past the filter");
    if (2 * cfg.blank_hz >= cfg.keep_hz)
        return fail("blank_hz " + std::to_string(cfg.blank_hz) +
                    " blanks the whole kept band");
    if (cfg.lo_max_hz <= cfg.lo_min_hz) return fail("lo_max must exceed lo_min");

    // Nothing in the requested range is reachable. Refusing is the honest
    // answer; clamping would produce a sweep of somewhere else entirely.
    if (cfg.f_start_hz > cfg.lo_max_hz + cfg.keep_hz / 2 ||
        cfg.f_stop_hz < cfg.lo_min_hz - cfg.keep_hz / 2)
        return fail("requested range is outside the tuner's " +
                    std::to_string(cfg.lo_min_hz) + "-" + std::to_string(cfg.lo_max_hz) +
                    " Hz LO range");

    SweepPlan plan;
    plan.cfg = cfg;

    // The only real difference between the policies: how far the LO advances per
    // hop, and therefore whether a neighbour's kept band reaches the DC hole.
    long long first_lo = 0;
    long long n = 0;
    plan.step_hz = plan_step_hz(cfg);
    if (cfg.dc == DcPolicy::SpliceOverlap) {
        // Centers land on f_start and every keep/2 above it. The hole at each
        // center is then the shared edge of its two neighbours' kept bands, and
        // the holes at the two ends of the sweep sit on the requested limits
        // where only the inward half needs covering.
        first_lo = cfg.f_start_hz;
        n = divide_round_up(cfg.f_stop_hz - cfg.f_start_hz, plan.step_hz) + 1;
    } else {
        // Centers half a kept-width in, so the kept bands tile [f_start, f_stop]
        // exactly. Each carries its own unfilled hole.
        first_lo = cfg.f_start_hz + cfg.keep_hz / 2;
        n = divide_round_up(cfg.f_stop_hz - cfg.f_start_hz, plan.step_hz);
    }
    if (n < 1) n = 1;

    // The synthesizer's grid (PlanConfig::lo_grid_hz): the origin moves down to
    // the multiple at or below it, so it never moves past the coverage the
    // construction above was built to give, and the extension below adds a
    // segment at the top if the move left the end short.
    const long long built_first = first_lo;
    if (cfg.lo_grid_hz < 0) return fail("lo_grid_hz cannot be negative");
    if (cfg.lo_grid_hz > 0) {
        const long long g = cfg.lo_grid_hz;
        first_lo -= ((first_lo % g) + g) % g;
    }

    // §7's image check. Phasing the grid is the whole measurement: the same RF
    // frequency lands at a different baseband offset, so its mirror image moves
    // and it does not. See PlanConfig::lo_shift_hz.
    //
    // Bounded by one step because past that the grid is not a phased version of
    // itself, it is a different grid, and comparing two sweeps taken on
    // unrelated grids says nothing about images.
    if (cfg.lo_shift_hz <= -plan.step_hz || cfg.lo_shift_hz >= plan.step_hz)
        return fail("lo_shift_hz " + std::to_string(cfg.lo_shift_hz) +
                    " must be smaller in magnitude than the LO step " +
                    std::to_string(plan.step_hz) +
                    "; a larger shift is a different grid, not a phased one");

    // Deliberately a no-op at shift 0 on an unaligned grid, down to the segment
    // count: the ordinary sweep must not change because this feature exists.
    first_lo += cfg.lo_shift_hz;
    if (first_lo != built_first) {

        // The grid is then extended at both ends until the requested range is
        // covered to the same standard the unshifted plan meets. Under splice
        // that standard is stronger than "the kept band reaches f_start": a
        // segment's DC hole is filled by its two *neighbours*, so the first
        // segment needs one below it or its own hole lands inside the requested
        // range with nothing to splice from. Requiring first_lo <= f_start and
        // last_lo >= f_stop is exactly what the unshifted construction gives,
        // and it is what plan_is_gapless then goes on to verify.
        //
        // Under blank there is no splice and plain coverage is the whole
        // requirement, so the bound is the kept edge instead.
        const long long half = cfg.keep_hz / 2;
        const bool splice = cfg.dc == DcPolicy::SpliceOverlap;
        const long long need_first = splice ? cfg.f_start_hz : cfg.f_start_hz + half;
        const long long need_last = splice ? cfg.f_stop_hz : cfg.f_stop_hz - half;
        while (first_lo > need_first) {
            first_lo -= plan.step_hz;
            ++n;
        }
        while (first_lo + (n - 1) * plan.step_hz < need_last && size_t(n) <= kMaxSegments)
            ++n;
    }

    plan.first_lo_hz = first_lo;
    if (size_t(n) > kMaxSegments)
        return fail(std::to_string(n) + " segments exceeds the " +
                    std::to_string(kMaxSegments) + "-segment sanity limit; check keep_hz");

    plan.segments.reserve(size_t(n));
    for (long long i = 0; i < n; ++i) {
        long long lo = first_lo + i * plan.step_hz;
        if (lo < cfg.lo_min_hz) lo = cfg.lo_min_hz;
        if (lo > cfg.lo_max_hz) lo = cfg.lo_max_hz;
        // Clamping collapses the outriggers onto the limit, and a segment that
        // repeats its predecessor's LO costs a hop and adds nothing.
        if (!plan.segments.empty() && plan.segments.back().lo_request_hz == lo) continue;

        Segment s;
        s.lo_request_hz = lo;
        s.lo_hz = lo;
        set_segment_edges(&s, cfg.keep_hz, cfg.blank_hz);
        plan.segments.push_back(s);
    }
    plan.covered_lo_hz = plan.segments.front().keep_lo_hz;
    plan.covered_hi_hz = plan.segments.back().keep_hi_hz;
    plan.truncated = plan.covered_lo_hz > cfg.f_start_hz || plan.covered_hi_hz < cfg.f_stop_hz;

    *out = std::move(plan);
    return true;
}

bool plan_is_gapless(const SweepPlan &plan) {
    if (plan.segments.empty()) return false;
    if (plan.covered_lo_hz > plan.cfg.f_start_hz) return false;
    if (plan.covered_hi_hz < plan.cfg.f_stop_hz) return false;
    // With nothing blanked there is nothing to splice, so either policy covers.
    if (plan.cfg.blank_hz == 0) return true;
    if (plan.cfg.dc != DcPolicy::SpliceOverlap) return false;

    // Verify the splice rather than trusting the construction. Each segment
    // usefully supplies two intervals -- its kept band either side of its own
    // blanked hole -- and every other segment's hole has to be covered by the
    // union of those. Checked by greedy interval cover, because checking only
    // the hole's endpoints would pass a plan with a gap in the middle of one.
    //
    // This is the property the entire DcPolicy distinction exists for, and the
    // margin it depends on (keep_hz > 4 * blank_hz) is not obvious from the
    // config, so it is measured here rather than asserted in a comment.
    for (size_t i = 0; i < plan.segments.size(); ++i) {
        long long lo = plan.segments[i].blank_lo_hz;
        const long long hi = plan.segments[i].blank_hi_hz < plan.cfg.f_stop_hz
                                 ? plan.segments[i].blank_hi_hz
                                 : plan.cfg.f_stop_hz;
        if (lo < plan.cfg.f_start_hz) lo = plan.cfg.f_start_hz;
        if (lo >= hi) continue;  // the hole falls outside the requested range

        long long cursor = lo;
        while (cursor < hi) {
            long long reach = cursor;
            for (size_t j = 0; j < plan.segments.size(); ++j) {
                if (j == i) continue;
                const Segment &o = plan.segments[j];
                const long long ivals[2][2] = {{o.keep_lo_hz, o.blank_lo_hz},
                                               {o.blank_hi_hz, o.keep_hi_hz}};
                for (const auto &iv : ivals) {
                    if (iv[0] <= cursor && iv[1] > reach) reach = iv[1];
                }
            }
            if (reach <= cursor) return false;  // nothing covers this frequency
            cursor = reach;
        }
    }
    return true;
}

}  // namespace sweepd
