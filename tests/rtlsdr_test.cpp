// Host-side tests for the RTL-SDR source's hardware-free half.
//
// No dongle and no librtlsdr are needed to run this, which is the point: the
// things being checked are the things a dongle could not tell you were wrong.
// A misderived keep width or a gain snapped to the wrong entry does not produce
// an error -- it produces a trace that looks entirely reasonable and is scaled
// or seamed incorrectly, which is the same class of failure tests/stitch_test
// and tests/segments_test exist for.
//
// Four groups:
//
//   1. **The plan.** rtl_plan_config decides the LO grid this source sweeps,
//      and then plan_sweep is *the daemon's own planner*, so the test worth
//      having is that the derived config feeds it something it accepts and that
//      the resulting geometry is the one the header claims -- step = keep/2,
//      kept band = LO +/- fs/4, the splice fill drawn from fs/4 out.
//   2. **§7's overlay agrees.** The reason this source exists at all is that
//      its grid is reconstructible by sweepui/segments.h where the HackRF's is
//      not. That claim is checked here rather than asserted in a comment.
//   3. **Gain snapping**, against the R820T's real 29-entry list.
//   4. **Refusals**, because the value of rtl_validate is entirely in what it
//      says no to before the device is touched.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "../src/sweepd/plan.h"
#include "../src/sweepui/rtlsdrsource.h"
#include "../src/sweepui/segments.h"

using namespace sweepui;

namespace {

int g_failures = 0;

void ok(const char *what, bool pass) {
    std::printf("  %-70s %s\n", what, pass ? "ok" : "FAIL");
    if (!pass) ++g_failures;
}

void eq(const char *what, long long got, long long want) {
    const bool pass = got == want;
    std::printf("  %-44s %16lld want %16lld %s\n", what, got, want, pass ? "ok" : "FAIL");
    if (!pass) ++g_failures;
}

// rtlsdr_get_tuner_gains' output for an R820T, in tenths of a dB. Unevenly
// spaced, which is the whole reason rtl_snap_gain exists.
const std::vector<int> kR820TGains = {0,   9,   14,  27,  37,  77,  87,  125, 144, 157,
                                      166, 197, 207, 229, 254, 280, 297, 328, 338, 364,
                                      372, 386, 402, 421, 434, 439, 445, 480, 496};

// ---------------------------------------------------------------- 1. the plan

void test_plan() {
    std::printf("plan geometry\n");

    RtlSdrOptions o;   // defaults: 88-108 MHz, 2.4 MSPS, keep 0 (derive)
    long long lo = 0, hi = 0;
    ok("an R820T reports a range", rtl_tuner_range(RtlTuner::R820T, 0, &lo, &hi));

    const sweepd::PlanConfig c = rtl_plan_config(o, 2400000.0, lo, hi);
    eq("keep derives as fs/2", c.keep_hz, 1200000);
    eq("the filter check falls back to fs", c.bw_hz, 2400000);
    eq("the tuner floor is the R820T's", c.lo_min_hz, 24000000);
    eq("the tuner ceiling is the R820T's", c.lo_max_hz, 1766000000);
    ok("splice is the default policy", c.dc == sweepd::DcPolicy::SpliceOverlap);

    // The daemon's planner, not a copy of it. If this refuses, the source is
    // handing plan_sweep a config it cannot honour and no sweep happens.
    sweepd::SweepPlan p;
    std::string err;
    ok("the daemon's planner accepts it", sweepd::plan_sweep(c, &p, &err));
    if (!err.empty()) std::printf("    planner said: %s\n", err.c_str());

    eq("the step is keep/2", p.step_hz, 600000);
    ok("the plan is gapless", sweepd::plan_is_gapless(p));

    // 88-108 MHz is 20 MHz; at a 600 kHz step that is 33 hops plus the closing
    // centre on f_stop. The point is not the exact count, it is that the sweep
    // is tens of segments rather than the thousands a full span would be.
    eq("88-108 MHz is 35 segments", (long long)p.segments.size(), 35);

    // The kept band is the LO +/- fs/4, which is what keeps the splice fill off
    // the IF filter corner. See the header's "what the geometry is".
    const sweepd::Segment &s = p.segments[10];
    eq("the kept band reaches fs/4 below the LO", s.lo_hz - s.keep_lo_hz, 600000);
    eq("the kept band reaches fs/4 above the LO", s.keep_hi_hz - s.lo_hz, 600000);
    eq("the blank is the configured half-width", s.lo_hz - s.blank_lo_hz, 25000);

    // The claim that makes keep = fs/2 worth its extra hops: the neighbour that
    // fills segment k's DC hole measures it at exactly one step from its own
    // LO, and one step is fs/4 rather than the 3fs/8 a keep of 3fs/4 would give.
    eq("the splice fill is drawn from fs/4 out", p.step_hz, 600000);

    // A wider keep is the documented speed/seam trade, in the direction
    // claimed: fewer segments, fill drawn from further out.
    RtlSdrOptions wide = o;
    wide.keep_hz = 1800000;
    sweepd::SweepPlan pw;
    ok("a keep of 3fs/4 still plans",
       sweepd::plan_sweep(rtl_plan_config(wide, 2400000.0, lo, hi), &pw, &err));
    ok("a wider keep is fewer segments", pw.segments.size() < p.segments.size());
    eq("and the fill comes from further out", pw.step_hz, 900000);

    // dc_policy=blank: the step becomes the whole keep, so the sweep is twice
    // as fast and every centre keeps its own hole. Both halves checked, because
    // the policy's entire purpose is that second half.
    RtlSdrOptions blanked = o;
    blanked.dc = sweepd::DcPolicy::BlankGap;
    sweepd::SweepPlan pb;
    ok("blank plans", sweepd::plan_sweep(rtl_plan_config(blanked, 2400000.0, lo, hi), &pb,
                                         &err));
    eq("blank steps a whole keep", pb.step_hz, 1200000);
    ok("blank is not gapless, which is the point", !sweepd::plan_is_gapless(pb));

    // §7's image check, which the HackRF source has to refuse. Half a step,
    // expressed in Hz, which this radio can actually tune to.
    eq("the image-check shift is half a step", sweepd::image_check_shift_hz(c), 300000);
    RtlSdrOptions shifted = o;
    shifted.lo_shift_hz = sweepd::image_check_shift_hz(c);
    sweepd::SweepPlan ps;
    ok("the shifted pass plans",
       sweepd::plan_sweep(rtl_plan_config(shifted, 2400000.0, lo, hi), &ps, &err));
    ok("and it moved the grid", ps.first_lo_hz != p.first_lo_hz);

    // A full span, to pin the number the header's table quotes. This is the
    // honest cost of the radio and it should not drift silently.
    RtlSdrOptions full = o;
    full.f_start_hz = 24000000LL;
    full.f_stop_hz = 1766000000LL;
    sweepd::SweepPlan pf;
    ok("a full span plans",
       sweepd::plan_sweep(rtl_plan_config(full, 2400000.0, lo, hi), &pf, &err));
    ok("a full span is ~2900 segments",
       pf.segments.size() > 2800 && pf.segments.size() < 3000);
    std::printf("    full span: %zu segments, ~%.1f s at a 5 ms settle\n",
                pf.segments.size(), double(pf.segments.size()) * 0.005);

    // Direct sampling replaces the range wholesale, and the tuner stops
    // mattering -- an R820T and an unidentified part give the same answer.
    long long dlo = 0, dhi = 0, ulo = 0, uhi = 0;
    ok("direct sampling has a range", rtl_tuner_range(RtlTuner::R820T, 2, &dlo, &dhi));
    ok("even for an unknown tuner", rtl_tuner_range(RtlTuner::Unknown, 2, &ulo, &uhi));
    ok("and it is the same range", dlo == ulo && dhi == uhi);
    eq("direct sampling tops out at fs_rtl/2", dhi, 14400000);

    // The one range this source refuses to invent.
    ok("an unidentified tuner has no swept range",
       !rtl_tuner_range(RtlTuner::Unknown, 0, &ulo, &uhi));

    // Every tuner it does claim to know must give a usable interval, or
    // plan_sweep will refuse every span on that dongle.
    const RtlTuner known[] = {RtlTuner::R820T,  RtlTuner::R828D,  RtlTuner::E4000,
                              RtlTuner::FC0012, RtlTuner::FC0013, RtlTuner::FC2580};
    bool all_sane = true;
    for (RtlTuner t : known) {
        long long a = 0, b = 0;
        if (!rtl_tuner_range(t, 0, &a, &b) || a <= 0 || b <= a) all_sane = false;
        // uint32 Hz is what rtlsdr_set_center_freq takes.
        if (b > 4294967295LL) all_sane = false;
    }
    ok("every known tuner has a sane, tunable range", all_sane);

    // nextrtl: the fork upconverts HF, so the range must reach below the FC0013's
    // 22 MHz floor, and it must stay a single interval a uint32 can tune.
    long long nlo = 0, nhi = 0;
    nextrtl_range(&nlo, &nhi);
    ok("nextrtl reaches HF and L-band in one tunable interval",
       nlo > 0 && nlo < 1000000LL && nhi >= 1575420000LL && nhi <= 4294967295LL);
}

// ------------------------------------------------- 2. §7's overlay agrees

void test_overlay_agrees() {
    std::printf("the segment overlay reconstructs this source's grid\n");

    // The reason this source is in the tree (rtlsdrsource.h, reason 2). The
    // HackRF source leaves these fields zero because its LOs are not a uniform
    // grid; this one fills them, so the claim to check is that segments.h
    // rebuilds *exactly* what was swept. A drifted overlay would put the splice
    // comb at the wrong frequencies, which is the §7 failure it exists to stop.
    RtlSdrOptions o;
    long long lo = 0, hi = 0;
    rtl_tuner_range(RtlTuner::R820T, 0, &lo, &hi);

    struct Case {
        const char *what;
        long long f_start, f_stop;
        long long keep;
        sweepd::DcPolicy dc;
        long long shift;
    };
    const Case cases[] = {
        {"88-108 MHz, defaults", 88000000LL, 108000000LL, 0, sweepd::DcPolicy::SpliceOverlap, 0},
        {"a wide keep", 400000000LL, 440000000LL, 1800000LL, sweepd::DcPolicy::SpliceOverlap, 0},
        {"dc_policy=blank", 88000000LL, 108000000LL, 0, sweepd::DcPolicy::BlankGap, 0},
        {"the shifted image-check pass", 88000000LL, 108000000LL, 0,
         sweepd::DcPolicy::SpliceOverlap, 300000LL},
        // The tuner clamp biting at the bottom end, which is where the
        // reconstruction is hardest: an outrigger collapses onto lo_min and the
        // plan drops the duplicate.
        {"clamped at the tuner floor", 24000000LL, 40000000LL, 0,
         sweepd::DcPolicy::SpliceOverlap, 0},
    };

    for (const Case &cs : cases) {
        RtlSdrOptions c = o;
        c.f_start_hz = cs.f_start;
        c.f_stop_hz = cs.f_stop;
        c.keep_hz = cs.keep;
        c.dc = cs.dc;
        c.lo_shift_hz = cs.shift;

        const sweepd::PlanConfig pc = rtl_plan_config(c, 2400000.0, lo, hi);
        sweepd::SweepPlan p;
        std::string err;
        if (!sweepd::plan_sweep(pc, &p, &err)) {
            std::printf("  %-70s %s\n", cs.what, "FAIL (planner refused)");
            ++g_failures;
            continue;
        }

        // Exactly the fields session() publishes into SourceConfig, which is
        // exactly what app.cpp hands to reconstruct_segments.
        SegmentPlanInfo info;
        info.first_lo_hz = double(p.first_lo_hz);
        info.step_hz = double(p.step_hz);
        info.keep_hz = double(p.cfg.keep_hz);
        info.blank_hz = double(p.cfg.blank_hz);
        info.tune_min_hz = double(pc.lo_min_hz);
        info.tune_max_hz = double(pc.lo_max_hz);
        info.covered_lo_hz = double(p.covered_lo_hz);
        info.covered_hi_hz = double(p.covered_hi_hz);
        info.n_segments = uint32_t(p.segments.size());
        info.splice = p.cfg.dc == sweepd::DcPolicy::SpliceOverlap;

        std::vector<SegmentBand> bands;
        std::string rerr;
        if (!reconstruct_segments(info, &bands, &rerr)) {
            std::printf("  %-70s FAIL (%s)\n", cs.what, rerr.c_str());
            ++g_failures;
            continue;
        }

        bool same = bands.size() == p.segments.size();
        for (size_t i = 0; same && i < bands.size(); ++i) {
            const sweepd::Segment &s = p.segments[i];
            same = bands[i].lo_hz == double(s.lo_hz) &&
                   bands[i].keep_lo_hz == double(s.keep_lo_hz) &&
                   bands[i].keep_hi_hz == double(s.keep_hi_hz) &&
                   bands[i].blank_lo_hz == double(s.blank_lo_hz) &&
                   bands[i].blank_hi_hz == double(s.blank_hi_hz);
        }
        ok(cs.what, same);
    }
}

// ---------------------------------------------------------- 3. gain snapping

void test_gain() {
    std::printf("gain snapping\n");

    double got = 0;
    ok("an empty list refuses", !rtl_snap_gain({}, 30.0, &got));

    // An exact entry stays put.
    ok("an exact entry is kept", rtl_snap_gain(kR820TGains, 32.8, &got) &&
                                     std::fabs(got - 32.8) < 1e-9);

    // 30 dB is not on the R820T's list; 29.7 and 32.8 straddle it and 29.7 is
    // nearer. This is the default gain, so the default readout is 29.7 and the
    // status bar must say so rather than 30.
    ok("the 30 dB default snaps down to 29.7",
       rtl_snap_gain(kR820TGains, 30.0, &got) && std::fabs(got - 29.7) < 1e-9);

    // Above the top of the list means maximum, which is how --rtl-gain max
    // works: there is no separate flag for it.
    ok("a request above the list means maximum",
       rtl_snap_gain(kR820TGains, 999.0, &got) && std::fabs(got - 49.6) < 1e-9);
    ok("and below it means minimum",
       rtl_snap_gain(kR820TGains, -50.0, &got) && std::fabs(got - 0.0) < 1e-9);

    // Whatever comes back must be settable, because librtlsdr rejects a gain
    // that is not on its list -- and a rejected gain is a whole sweep at
    // whatever the gain happened to already be.
    bool always_on_list = true;
    for (double want = -5.0; want <= 55.0; want += 0.05) {
        if (!rtl_snap_gain(kR820TGains, want, &got)) {
            always_on_list = false;
            break;
        }
        const int tenths = int(std::lround(got * 10.0));
        bool found = false;
        for (int g : kR820TGains)
            if (g == tenths) found = true;
        if (!found) always_on_list = false;
    }
    ok("every snapped gain is an entry librtlsdr will accept", always_on_list);
}

// --------------------------------------------------------------- 4. refusals

void test_refusals() {
    std::printf("what rtl_validate refuses\n");

    auto refused = [](RtlSdrOptions o) {
        std::string err;
        return !rtl_validate(&o, &err);
    };

    RtlSdrOptions good;
    std::string err;
    RtlSdrOptions g = good;
    ok("the defaults validate", rtl_validate(&g, &err));
    if (!err.empty()) std::printf("    said: %s\n", err.c_str());

    RtlSdrOptions o = good;
    o.f_stop_hz = o.f_start_hz;
    ok("an empty span is refused", refused(o));

    // librtlsdr's two windows, and the hole between them that catches people
    // who reach for 1 MSPS or 0.5 MSPS as a round number.
    o = good;
    o.fs_hz = 500000.0;
    ok("500 kSPS is refused: it is between librtlsdr's windows", refused(o));
    // The window edges exactly as librtlsdr draws them (librtlsdr.c: it rejects
    // rate <= 225000, rate > 3200000, and 300000 < rate <= 900000), because an
    // off-by-one here is a driver error from inside the worker.
    o.fs_hz = 900000.0;
    ok("900 kSPS is refused, being the top of the hole", refused(o));
    o.fs_hz = 900001.0;
    ok("900001 Hz is accepted, being the bottom of the window", !refused(o));
    o.fs_hz = 1000000.0;
    ok("1 MSPS is accepted, well inside the window", !refused(o));
    o.fs_hz = 225000.0;
    ok("225 kSPS is refused, being 1 Hz below the low window", refused(o));
    o.fs_hz = 225001.0;
    ok("225001 Hz is accepted", !refused(o));
    o.fs_hz = 300000.0;
    ok("300 kSPS is accepted, being the top of the low window", !refused(o));
    o.fs_hz = 300001.0;
    ok("300001 Hz is refused, being inside the hole", refused(o));
    o.fs_hz = 3200000.0;
    ok("3.2 MSPS is accepted, being the ceiling", !refused(o));
    o.fs_hz = 3200001.0;
    ok("above 3.2 MSPS is refused", refused(o));
    o = good;
    o.fs_hz = 2800000.0;
    ok("2.8 MSPS is accepted, not refused -- the drop risk is a banner, not a veto",
       !refused(o));

    o = good;
    o.n_fft = 1000;
    ok("a non-power-of-two FFT is refused", refused(o));
    o.n_fft = 32;
    ok("a tiny FFT is refused", refused(o));

    o = good;
    o.keep_hz = 4000000;
    ok("a keep wider than the sample rate is refused: it would alias", refused(o));

    o = good;
    o.direct_sampling = 3;
    ok("a direct-sampling branch that does not exist is refused", refused(o));

    o = good;
    o.settle_us = 2000000;
    ok("a settle of seconds per hop is refused", refused(o));

    o = good;
    o.n_points = 100000;   // max_points defaults to 65536
    ok("more points than max_points is refused", refused(o));

    // Rounded, not refused: all that matters about the dump is that librtlsdr
    // can ask for it.
    o = good;
    o.dump_bytes = 4000;
    ok("an odd dump size is rounded to the USB granularity",
       rtl_validate(&o, &err) && o.dump_bytes == 4096);

    // The read length for a segment, likewise.
    eq("N=1024 reads 2048 bytes", (long long)rtl_read_bytes(1024), 2048);
    eq("N=100 rounds 200 bytes up to 512", (long long)rtl_read_bytes(100), 512);
    ok("a read is always a multiple of 512 and never short", [] {
        for (size_t n = 64; n <= 65536; n *= 2)
            if (rtl_read_bytes(n) % 512 || rtl_read_bytes(n) < 2 * n) return false;
        return true;
    }());

    // A geometry rtl_validate lets through but the planner must not: the DC
    // blank eating the whole kept band. Checked here because the source relies
    // on plan_sweep for it rather than duplicating the rule.
    o = good;
    o.blank_hz = 900000;   // keep derives as 1.2 MHz, so 2*blank > keep
    ok("a blank wider than the keep passes validation", !refused(o));
    long long lo = 0, hi = 0;
    rtl_tuner_range(RtlTuner::R820T, 0, &lo, &hi);
    sweepd::SweepPlan p;
    std::string perr;
    ok("...and is then refused by the planner",
       !sweepd::plan_sweep(rtl_plan_config(o, 2400000.0, lo, hi), &p, &perr));

    // A span the tuner cannot reach at all. The source does not clamp it into
    // somewhere else, which would be a sweep of a band nobody asked for.
    o = good;
    o.f_start_hz = 2500000000LL;
    o.f_stop_hz = 2600000000LL;
    ok("a span above an R820T's ceiling passes validation", !refused(o));
    ok("...and is then refused by the planner",
       !sweepd::plan_sweep(rtl_plan_config(o, 2400000.0, lo, hi), &p, &perr));
    std::printf("    planner said: %s\n", perr.c_str());
}

}  // namespace

int main() {
    // Stated up front so a run of this test is not mistaken for evidence about
    // a dongle. It is evidence about arithmetic.
    std::printf("rtlsdr_test: the hardware-free half. librtlsdr %s.\n\n",
                rtlsdr_available() ? "is compiled in but is not opened here"
                                   : "is not compiled in, which changes nothing here");
    test_plan();
    test_overlay_agrees();
    test_gain();
    test_refusals();
    std::printf("\n%s\n", g_failures ? "FAILURES" : "all ok");
    return g_failures ? 1 : 0;
}
