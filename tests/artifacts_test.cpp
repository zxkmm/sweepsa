// Host-side tests for the receiver-artifact marker (artifacts.h).
//
// Like imagecheck_test, the centrepiece is a simulated receiver built on the
// *real* sweepd::plan_sweep, painting on purpose everything the marker has to
// tell apart:
//
//   * the DC hole / splice fill at every LO           -> must be marked LO/DC
//   * a +3 dB hump within 2 MHz of every LO           -> must be marked LO comb
//   * a narrow spur at LO +/- 6 MHz in a scattered
//     minority of segments (the f_ref/8 shape)        -> must be marked LO spur
//   * a spur at a fixed RF frequency                  -> must be marked internal,
//                                                        and only with a baseline
//   * real carriers, random and on a raster           -> must NOT be marked
//
// plus fastlock's per-segment amplitude offset and a front-end tilt, which a
// marker comparing levels across segments would mistake for all of the above.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "../src/sweepd/plan.h"
#include "../src/sweepui/artifacts.h"
#include "../src/sweepui/segments.h"

using namespace sweepui;

namespace {

int g_failures = 0;

void ok(const char *what, bool pass) {
    std::printf("  %-68s %s\n", what, pass ? "ok" : "FAIL");
    if (!pass) ++g_failures;
}

constexpr double kFloorDb = -100.0;
constexpr double kFillDb = 4.5;
constexpr double kHumpDb = 3.0;
constexpr double kHumpHz = 2e6;
constexpr double kSpurOffsetHz = 6e6;
constexpr double kSpurDb = 22.0;
constexpr double kInternalHz = 2400.0e6 + 1234567.0;
constexpr double kInternalDb = 15.0;

uint64_t mix(uint64_t x) {
    x ^= x >> 33;
    x *= 0xff51afd7ed558ccdULL;
    x ^= x >> 33;
    x *= 0xc4ceb9fe1a85ec53ULL;
    x ^= x >> 33;
    return x;
}

double seg_offset_db(double lo) {
    return double(mix(uint64_t(lo / 1000.0)) % 1261) / 100.0 - 6.3;
}

double jitter_db(size_t i, uint64_t seed) {
    return double(mix(i * 7919 + seed) % 101) / 100.0 - 0.5;
}

std::vector<SegmentBand> bands_of(const sweepd::SweepPlan &plan) {
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
    std::vector<SegmentBand> out;
    std::string err;
    if (!reconstruct_segments(i, &out, &err)) out.clear();
    return out;
}

struct Scene {
    std::vector<double> tones;      // real carriers, dB above floor = tone_db
    double tone_db = 20.0;
    bool hump = true;
    size_t spur_every = 0;          // LO spur in every Nth segment; 0 = none
    bool internal = false;          // the fixed-RF spur
    double internal_extra_db = 0;   // a real signal on top of it
    uint64_t seed = 1;
};

struct World {
    sweepd::SweepPlan plan;
    std::vector<SegmentBand> bands;
    TraceAxis ax;
    bool ok = false;
};

World make_world(long long f0, long long f1, size_t n) {
    World w;
    sweepd::PlanConfig c;
    c.f_start_hz = f0;
    c.f_stop_hz = f1;
    std::string err;
    if (!sweepd::plan_sweep(c, &w.plan, &err)) return w;
    w.bands = bands_of(w.plan);
    if (w.bands.empty()) return w;
    w.ax.x0 = double(f0);
    w.ax.dx = double(f1 - f0) / double(n);
    w.ax.n = n;
    w.ax.rbw_hz = w.ax.dx;
    w.ok = true;
    return w;
}

ImagePass paint(const World &w, const Scene &sc) {
    ImagePass p;
    p.axis = w.ax;
    p.segments = w.bands;
    p.sweeps = 100;
    const size_t n = w.ax.n;
    std::vector<double> excess(n, 0.0);
    auto add_at = [&](double f, double db) {
        if (f < w.ax.x0) return;
        const size_t i = size_t((f - w.ax.x0) / w.ax.dx);
        if (i < n && db > excess[i]) excess[i] = db;
    };
    for (double t : sc.tones) add_at(t, sc.tone_db);
    if (sc.spur_every)
        for (size_t s = 0; s < w.bands.size(); s += sc.spur_every) {
            add_at(w.bands[s].lo_hz + kSpurOffsetHz, kSpurDb);
            add_at(w.bands[s].lo_hz - kSpurOffsetHz, kSpurDb);
        }
    if (sc.internal) add_at(kInternalHz, kInternalDb + sc.internal_extra_db);

    p.db.resize(n);
    for (size_t i = 0; i < n; ++i) {
        const double f = w.ax.x_at(i);
        const int s = segment_at(w.bands, f);
        const double tilt = -4.0 * (f - w.ax.x0) / (double(n) * w.ax.dx);
        const double off = s >= 0 ? seg_offset_db(w.bands[size_t(s)].lo_hz) : 0.0;
        double noise = kFloorDb + tilt + off + jitter_db(i, sc.seed);
        if (s >= 0 && sc.hump && std::fabs(f - w.bands[size_t(s)].lo_hz) < kHumpHz)
            noise += kHumpDb;
        if (in_blanked_span(w.bands, f)) noise += kFillDb;
        // Power sum, not max: a spur sitting on the hump is both.
        double v = noise;
        if (excess[i] > 0) {
            const double sig = kFloorDb + tilt + off + excess[i];
            v = 10.0 * std::log10(std::pow(10.0, noise / 10.0) + std::pow(10.0, sig / 10.0));
        }
        p.db[i] = float(v);
    }
    return p;
}

uint8_t flags_at(const ArtifactResult &r, double f) {
    size_t i;
    if (!r.axis.bin_of(f, &i)) return 0;
    uint8_t v = r.flags[i];
    // The painted bin plus its neighbours, since a tone landing near a bin edge
    // is found by the run it belongs to.
    if (i > 0) v |= r.flags[i - 1];
    if (i + 1 < r.flags.size()) v |= r.flags[i + 1];
    return v;
}

bool near_lo(const World &w, double f, double tol) {
    for (const auto &b : w.bands)
        if (std::fabs(f - b.lo_hz) < tol) return true;
    return false;
}

// ------------------------------------------------------------------- tests

void test_refusals() {
    std::printf("refusals\n");
    ArtifactConfig cfg;
    ImagePass p;
    ok("no axis is refused", !mark_artifacts(p, nullptr, 0, cfg).valid);
    p.axis.x0 = 0;
    p.axis.dx = 1e-3;
    p.axis.n = 4;
    p.axis.time = true;
    p.db.assign(4, -80.0f);
    ArtifactResult r = mark_artifacts(p, nullptr, 0, cfg);
    ok("zero span is refused, with a reason", !r.valid && !r.err.empty());
}

void test_narrow_span() {
    std::printf("900-1100 MHz, 4096 points\n");
    World w = make_world(900000000LL, 1100000000LL, 4096);
    ok("world built", w.ok);
    if (!w.ok) return;
    ArtifactConfig cfg;
    Scene sc;
    sc.tones = {917.3e6, 955.55e6, 1043.21e6};
    const ArtifactResult r = mark_artifacts(paint(w, sc), nullptr, 30, cfg);
    ok("valid", r.valid);
    ok("fold ran", r.fold_ran);

    size_t lo_marked = 0, lo_total = 0;
    for (const auto &b : w.bands) {
        if (b.lo_hz < w.ax.x0 || b.lo_hz >= w.ax.x0 + w.ax.dx * double(w.ax.n)) continue;
        ++lo_total;
        if (flags_at(r, b.lo_hz) & kArtDcHole) ++lo_marked;
    }
    ok("every in-span LO carries an LO/DC mark", lo_total > 0 && lo_marked == lo_total);

    bool comb_near_dc = false;
    for (const auto &c : r.comb)
        if (std::fabs(c.offset_hz) < kHumpHz) comb_near_dc = true;
    ok("the fold finds the hump around DC", comb_near_dc);

    size_t humped = 0, hump_bins = 0;
    for (size_t i = 0; i < w.ax.n; ++i) {
        const double f = w.ax.x_at(i);
        if (!near_lo(w, f, kHumpHz - 2 * w.ax.dx) || near_lo(w, f, 3 * w.ax.dx)) continue;
        ++hump_bins;
        if (r.flags[i] & kArtLoComb) ++humped;
    }
    char m[120];
    std::snprintf(m, sizeof m, "most hump bins are marked LO comb (%zu/%zu)", humped,
                  hump_bins);
    ok(m, hump_bins > 0 && humped * 10 >= hump_bins * 7);

    bool tone_clean = true;
    for (double t : sc.tones)
        if (flags_at(r, t)) tone_clean = false;
    ok("real carriers carry no mark", tone_clean);
    ok("no LO spurs invented on a spur-free scene", r.n_spur == 0);
    ok("caveat is never empty", !r.caveat.empty());
}

// `carriers` random real signals across the span, and, if `spurs`, the
// f_ref/8-shaped LO spur in every 29th segment -- about ten of ~298, scattered.
//
// Detection is asserted only at the lighter load. At 60 carriers a +/-1 bucket
// window of ~27 catches ~6.5 of them by chance, so ten spur segments alone are
// not improbable; it is found here only because this spur comes in +/- pairs,
// which a real one need not. What is asserted at every load is that no carrier
// away from the spur's offset is ever marked.
void test_full_span_spurs(size_t carriers, bool expect_detection) {
    std::printf("70 MHz - 6 GHz, 16384 points, sparse LO spurs + %zu random carriers\n",
                carriers);
    World w = make_world(70000000LL, 6000000000LL, 16384);
    ok("world built", w.ok);
    if (!w.ok) return;
    ArtifactConfig cfg;
    Scene sc;
    sc.spur_every = 29;
    for (size_t k = 0; k < carriers; ++k)
        sc.tones.push_back(80e6 + double(mix(k + 99) % 5900000) * 1000.0);
    const ArtifactResult r = mark_artifacts(paint(w, sc), nullptr, 30, cfg);
    ok("valid and folded", r.valid && r.fold_ran);

    size_t found = 0, painted = 0;
    for (size_t s = 0; s < w.bands.size(); s += sc.spur_every)
        for (double sign : {-1.0, 1.0}) {
            const double f = w.bands[s].lo_hz + sign * kSpurOffsetHz;
            if (f < w.ax.x0 || f >= w.ax.x0 + w.ax.dx * double(w.ax.n)) continue;
            ++painted;
            if (flags_at(r, f) & kArtLoSpur) ++found;
        }
    char m[120];
    std::snprintf(m, sizeof m, "scattered LO spurs are marked (%zu/%zu)", found, painted);
    if (expect_detection) ok(m, painted > 0 && found * 10 >= painted * 9);
    else std::printf("  (%s -- not asserted at this load)\n", m);

    // A carrier at the spur's |offset| from *its own* LO is marked with the
    // spur whichever segment it is in -- offset is the only evidence this test
    // has, and the header says so. What must not happen is a mark anywhere
    // else, so those carriers are counted separately and not asserted.
    size_t false_pos = 0, checked = 0, shared = 0;
    for (double t : sc.tones) {
        if (near_lo(w, t, kHumpHz + 2 * w.ax.dx)) continue;
        const int s = segment_at(w.bands, t);
        if (s < 0) continue;
        const double off = std::fabs(t - w.bands[size_t(s)].lo_hz);
        if (std::fabs(off - kSpurOffsetHz) < 2 * w.ax.dx) {
            ++shared;
            continue;
        }
        ++checked;
        if (flags_at(r, t) & (kArtLoSpur | kArtLoComb)) ++false_pos;
    }
    std::printf("  (%zu carriers share the spur's LO offset and are marked with it)\n",
                shared);
    std::snprintf(m, sizeof m, "no carrier off that offset is called an LO artifact (%zu/%zu)",
                  false_pos, checked);
    ok(m, checked * 2 >= carriers && false_pos == 0);
}

void test_crowded_no_spurs() {
    std::printf("full span, 300 random carriers and no spurs at all\n");
    World w = make_world(70000000LL, 6000000000LL, 16384);
    if (!w.ok) { ok("world built", false); return; }
    Scene sc;
    for (size_t k = 0; k < 300; ++k)
        sc.tones.push_back(80e6 + double(mix(k + 5000) % 5900000) * 1000.0);
    const ArtifactResult r = mark_artifacts(paint(w, sc), nullptr, 30, ArtifactConfig());
    char m[120];
    std::snprintf(m, sizeof m, "no LO spur is invented from coincidences (%zu bins)", r.n_spur);
    ok(m, r.valid && r.fold_ran && r.n_spur == 0);
}

void test_raster_band() {
    std::printf("a contiguous band of carriers on a raster commensurate with the step\n");
    World w = make_world(1700000000LL, 2000000000LL, 8192);
    ok("world built", w.ok);
    if (!w.ok) return;
    ArtifactConfig cfg;
    Scene sc;
    // Every 5 MHz from 1805 to 1880: the same LO offsets recur in every segment
    // the band touches, which is exactly what a naive recurrence count flags.
    for (double f = 1805.3e6; f < 1880e6; f += 5e6) sc.tones.push_back(f);
    const ArtifactResult r = mark_artifacts(paint(w, sc), nullptr, 30, cfg);
    size_t marked = 0, checked = 0;
    for (double t : sc.tones) {
        if (near_lo(w, t, kHumpHz + 2 * w.ax.dx)) continue;
        ++checked;
        if (flags_at(r, t) & kArtLoSpur) ++marked;
    }
    char m[120];
    std::snprintf(m, sizeof m, "none of them is an LO spur (%zu/%zu marked)", marked,
                  checked);
    ok(m, checked > 5 && marked == 0);
}

void test_baseline() {
    std::printf("terminated-input baseline\n");
    World w = make_world(2300000000LL, 2500000000LL, 4096);
    ok("world built", w.ok);
    if (!w.ok) return;
    ArtifactConfig cfg;

    Scene dark;
    dark.internal = true;
    dark.seed = 7;
    ArtifactBaseline base;
    base.pass = paint(w, dark);
    base.gain_db = 30;
    base.seg_first_lo_hz = w.bands.front().lo_hz;
    base.seg_step_hz = double(w.plan.step_hz);

    Scene live;
    live.internal = true;
    live.tones = {2351.7e6};
    live.seed = 8;
    ImagePass lp = paint(w, live);

    ArtifactResult without = mark_artifacts(lp, nullptr, 30, cfg);
    ok("without a baseline the fixed-RF spur is unmarked",
       !(flags_at(without, kInternalHz) & kArtInternal));
    ok("... and the caveat says so",
       without.caveat.find("baseline") != std::string::npos);

    ArtifactResult r = mark_artifacts(lp, &base, 30, cfg);
    ok("baseline used", r.baseline_used && r.baseline_note.empty());
    ok("with it, the fixed-RF spur is marked internal",
       (flags_at(r, kInternalHz) & kArtInternal) != 0);
    ok("the live-only carrier is not", !(flags_at(r, 2351.7e6) & kArtInternal));
    bool listed = false;
    for (const auto &f : r.findings)
        if (std::fabs(f.x_hz - kInternalHz) < 2 * w.ax.dx && (f.flags & kArtInternal))
            listed = true;
    ok("and it is listed as a finding", listed);

    Scene loud = live;
    loud.internal_extra_db = 20;
    r = mark_artifacts(paint(w, loud), &base, 30, cfg);
    ok("a real signal 20 dB over the internal spur is not marked",
       !(flags_at(r, kInternalHz) & kArtInternal));

    r = mark_artifacts(paint(w, loud), &base, 50, cfg);
    ok("a gain mismatch is reported", !r.baseline_note.empty());

    // Round trip.
    const std::string path = "artifacts_test_baseline.csv";
    std::string err;
    ok("baseline saves", save_baseline(path, base, &err));
    ArtifactBaseline back;
    ok("and loads", load_baseline(path, &back, &err));
    bool same = back.valid() && back.pass.axis.n == base.pass.axis.n &&
                std::fabs(back.pass.axis.dx - base.pass.axis.dx) < 1e-6 &&
                back.gain_db == base.gain_db;
    for (size_t i = 0; same && i < base.pass.db.size(); ++i)
        if (std::fabs(back.pass.db[i] - base.pass.db[i]) > 0.001f) same = false;
    ok("with the same axis, gain and levels", same);
    std::remove(path.c_str());

    ok("a file without the magic line is refused",
       !load_baseline("/dev/null", &back, &err) && !err.empty());
}

void test_coarse() {
    std::printf("full span at 1024 points is too coarse to fold\n");
    World w = make_world(70000000LL, 6000000000LL, 1024);
    if (!w.ok) { ok("world built", false); return; }
    Scene sc;
    sc.spur_every = 29;
    const ArtifactResult r = mark_artifacts(paint(w, sc), nullptr, 30, ArtifactConfig());
    ok("valid, but the folds did not run, and it says why",
       r.valid && !r.fold_ran && !r.fold_note.empty() && r.n_spur == 0 && r.n_comb == 0);
    ok("the DC holes are still marked from geometry", r.n_dc > 0);
}

void test_poisson() {
    std::printf("poisson tail\n");
    ok("P(X>=0) = 1", poisson_tail(0, 3.0) == 1.0);
    ok("P(X>=1 | 0) = 0", poisson_tail(1, 0.0) == 0.0);
    ok("P(X>=1 | 1) = 1 - e^-1", std::fabs(poisson_tail(1, 1.0) - (1 - std::exp(-1.0))) < 1e-12);
    ok("P(X>=10 | 2) is small", poisson_tail(10, 2.0) < 1e-4);
}

}  // namespace

int main() {
    test_refusals();
    test_narrow_span();
    test_full_span_spurs(20, true);
    test_full_span_spurs(60, false);
    test_crowded_no_spurs();
    test_raster_band();
    test_baseline();
    test_coarse();
    test_poisson();
    if (g_failures) {
        std::printf("%d FAILED\n", g_failures);
        return 1;
    }
    std::printf("all passed\n");
    return 0;
}
