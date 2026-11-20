// Host-side tests for §7's image check.
//
// The check answers "is this response a signal, or is it the receiver?", and
// every way it can be wrong produces a confident wrong answer rather than an
// error: a false positive tells the user a real transmission is not real, and a
// false negative leaves an artifact on screen wearing a signal's clothes.
//
// So the centrepiece here is a **simulated receiver** rather than a hand-made
// pair of arrays. It links the real sweepd::plan_sweep, builds both LO grids the
// daemon would build, and paints a spectrum that contains, on purpose, all four
// things the check has to tell apart:
//
//   * real tones, which must survive
//   * their mirror images, which must be caught and called images
//   * an LO-relative spur, which must be caught and *not* called an image
//   * the +4.5 dB splice comb at every segment centre, which must be excluded
//
// plus the two things that would break a naive comparison: a per-segment
// amplitude offset that differs between the passes (fastlock's 6.3 dB
// predecessor dependence) and a front-end tilt across the span.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "../src/sweepd/plan.h"
#include "../src/sweepui/imagecheck.h"
#include "../src/sweepui/segments.h"

using namespace sweepui;

namespace {

int g_failures = 0;

void ok(const char *what, bool pass) {
    std::printf("  %-68s %s\n", what, pass ? "ok" : "FAIL");
    if (!pass) ++g_failures;
}

// --------------------------------------------------------- the simulated radio

constexpr double kFloorDb = -100.0;
constexpr double kCombDb = 4.5;      // PROJECT-STATE.md, "The splice fill is hot"
constexpr double kSpurOffsetHz = 6e6;
constexpr double kSpurDb = 25.0;

// The real f_ref/8 spur sits at LO +/- 10 MHz, which under the default plan is
// exactly on the ownership seam (step/2) and therefore ambiguous about which
// segment produced it. 6 MHz keeps it unambiguously inside one segment, which
// is what makes the *classification* testable; the flagging does not care.

struct Tone {
    double hz;
    double db_above_floor;
};

// Repeatable per-LO amplitude offset, standing in for fastlock's
// predecessor-dependent recall level. Different LOs give different offsets, so
// the two passes disagree segment by segment -- which is exactly the thing a
// level-to-level comparison would mistake for signals appearing and vanishing.
double seg_offset_db(long long lo_hz) {
    const uint64_t h = uint64_t(lo_hz / 1000) * 6364136223846793005ULL + 1442695040888963407ULL;
    return double((h >> 33) % 1261) / 100.0 - 6.3;  // -6.3 .. +6.3 dB
}

double tilt_db(double f_hz) {
    return -4.0 * (f_hz - 9e8) / 2e8;   // a few dB of front-end slope across the span
}

// Deterministic sub-dB jitter, so the median has something to do and a test that
// only passes on a perfectly flat floor fails here.
double jitter_db(size_t i) {
    const uint64_t h = uint64_t(i) * 2862933555777941757ULL + 3037000493ULL;
    return double((h >> 40) % 101) / 100.0 - 0.5;
}

double lo_of(const std::vector<SegmentBand> &bands, double f) {
    const int s = segment_at(bands, f);
    return s < 0 ? 0.0 : bands[size_t(s)].lo_hz;
}

// Paints one pass. `image_rej_db` is how far below its parent each mirror image
// lands; a large number means a receiver with good quadrature and nothing for
// the check to find.
ImagePass paint(const sweepd::SweepPlan &plan, const std::vector<SegmentBand> &bands,
                const std::vector<Tone> &tones, double image_rej_db, bool with_spur,
                double x0, double dx, size_t n) {
    ImagePass p;
    p.axis.x0 = x0;
    p.axis.dx = dx;
    p.axis.n = n;
    p.axis.rbw_hz = dx;
    p.segments = bands;
    p.lo_shift_hz = double(plan.cfg.lo_shift_hz);
    p.sweeps = 200;
    p.db.assign(n, 0.0f);

    // Everything is assembled in dB above the local floor first, then the floor
    // and the per-segment offset are added -- which is the order the real
    // receiver does it in, and the order that makes the check's cancellation a
    // real claim rather than one baked into the fixture.
    std::vector<double> excess(n, 0.0);

    auto add_at = [&](double f, double db) {
        if (f < x0 || f >= x0 + double(n) * dx) return;
        const size_t i = size_t((f - x0) / dx);
        if (i < n && db > excess[i]) excess[i] = db;
    };

    for (const Tone &t : tones) {
        add_at(t.hz, t.db_above_floor);
        // Its image, mirrored about the LO of whichever segment owns the tone.
        const double lo = lo_of(bands, t.hz);
        if (lo > 0) add_at(2 * lo - t.hz, t.db_above_floor - image_rej_db);
    }

    if (with_spur) {
        for (const auto &b : bands) {
            add_at(b.lo_hz + kSpurOffsetHz, kSpurDb);
            add_at(b.lo_hz - kSpurOffsetHz, kSpurDb);
        }
    }

    for (size_t i = 0; i < n; ++i) {
        const double f = x0 + (double(i) + 0.5) * dx;
        const int s = segment_at(bands, f);
        const double lo = s < 0 ? 0.0 : bands[size_t(s)].lo_hz;
        double v = kFloorDb + tilt_db(f) + jitter_db(i);
        if (s >= 0) v += seg_offset_db((long long)lo);
        // The splice comb: the blanked hole is filled from a neighbour taken at
        // keep/2 from *its* LO, and reads hot.
        if (in_blanked_span(bands, f)) v += kCombDb;
        v += excess[i];
        p.db[i] = float(v);
    }
    return p;
}

std::vector<SegmentBand> bands_of(const sweepd::SweepPlan &plan, std::string *err) {
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
    if (!reconstruct_segments(i, &out, err)) out.clear();
    return out;
}

// True if some finding sits within `tol` of `f`.
const ImageFinding *found_near(const ImageCheckResult &r, double f, double tol) {
    for (const auto &x : r.findings)
        if (std::fabs(x.x_hz - f) <= tol) return &x;
    return nullptr;
}

// ------------------------------------------------------------------- the tests

struct World {
    sweepd::SweepPlan pa, pb;
    std::vector<SegmentBand> ba, bb;
    double x0 = 0, dx = 0;
    size_t n = 0;
    bool ok = false;
};

World build_world() {
    World w;
    sweepd::PlanConfig ca;
    ca.f_start_hz = 900000000LL;
    ca.f_stop_hz = 1100000000LL;
    std::string err;
    if (!sweepd::plan_sweep(ca, &w.pa, &err)) return w;

    sweepd::PlanConfig cb = ca;
    cb.lo_shift_hz = sweepd::image_check_shift_hz(ca);
    if (!sweepd::plan_sweep(cb, &w.pb, &err)) return w;

    w.ba = bands_of(w.pa, &err);
    w.bb = bands_of(w.pb, &err);
    if (w.ba.empty() || w.bb.empty()) return w;

    // One output grid for both passes, which is not what the daemon produces --
    // the covered range moves with the grid. Handled deliberately in the
    // separate offset-axis test below; here the question is the classification.
    w.x0 = 9.0e8;
    w.dx = 200e6 / 4096.0;
    w.n = 4096;
    w.ok = true;
    return w;
}

void test_finds_images() {
    std::printf("run_image_check -- tells four things apart\n");
    World w = build_world();
    ok("builds both grids", w.ok);
    if (!w.ok) return;

    // Two tones, deliberately placed so their images land well away from any
    // segment centre and from each other.
    const std::vector<Tone> tones = {{9.372e8, 45.0}, {1.0538e9, 38.0}};
    ImagePass a = paint(w.pa, w.ba, tones, 20.0, true, w.x0, w.dx, w.n);
    ImagePass b = paint(w.pb, w.bb, tones, 20.0, true, w.x0, w.dx, w.n);

    ImageCheckConfig cfg;
    const ImageCheckResult r = run_image_check(a, b, cfg);
    ok("runs", r.valid);
    if (!r.valid) {
        std::printf("    %s\n", r.err.c_str());
        ++g_failures;
        return;
    }
    ok("knows both segment grids", r.segments_known);
    ok("compared most of the span", r.n_compared > w.n / 2);
    ok("excluded the blanked spans of both grids", r.n_blanked > 0);
    ok("the caveat is never empty", !r.caveat.empty());

    const double tol = 3 * w.dx;

    // The real tones must survive. This is the failure that matters most: a
    // check that flags real signals is worse than no check.
    ok("the first real tone is not flagged", found_near(r, tones[0].hz, tol) == nullptr);
    ok("the second real tone is not flagged", found_near(r, tones[1].hz, tol) == nullptr);

    // Their images must be caught, and called images.
    for (size_t t = 0; t < tones.size(); ++t) {
        const double img = 2 * lo_of(w.ba, tones[t].hz) - tones[t].hz;
        const ImageFinding *f = found_near(r, img, tol);
        char label[96];
        std::snprintf(label, sizeof label, "image of tone %zu is flagged", t + 1);
        ok(label, f != nullptr);
        if (!f) continue;
        std::snprintf(label, sizeof label, "image of tone %zu is called an image", t + 1);
        ok(label, f->verdict == ImageVerdict::Image);
        std::snprintf(label, sizeof label,
                      "image of tone %zu says where it went (one whole step)", t + 1);
        ok(label, std::fabs(std::fabs(f->found_at_hz - f->x_hz) - w.pa.step_hz) <= tol);
    }

    // The LO spur must be caught but must not be called an image: it moved by
    // the shift, not by twice it.
    const double spur = w.ba[3].lo_hz + kSpurOffsetHz;
    const ImageFinding *sf = found_near(r, spur, tol);
    ok("the LO spur is flagged", sf != nullptr);
    if (sf) {
        ok("the LO spur is not called an image", sf->verdict == ImageVerdict::LoSpur);
        ok("the LO spur moved by the shift, not twice it",
           std::fabs(std::fabs(sf->found_at_hz - sf->x_hz) - std::fabs(r.shift_hz)) <= tol);
    }

    // The splice comb must produce no findings at all. Without the exclusion
    // this is one false image per segment.
    size_t on_a_centre = 0;
    for (const auto &f : r.findings)
        if (in_blanked_span(w.ba, f.x_hz) || in_blanked_span(w.bb, f.x_hz)) ++on_a_centre;
    ok("no finding sits on a segment centre of either grid", on_a_centre == 0);

    std::printf("    %zu findings, %zu compared, %zu blanked, floor window +/-%.0f bins\n",
                r.findings.size(), r.n_compared, r.n_blanked, r.floor_halfwidth);
}

void test_clean_receiver_finds_nothing() {
    std::printf("run_image_check -- a receiver with no images yields no findings\n");
    World w = build_world();
    if (!w.ok) { ok("builds both grids", false); return; }

    // 80 dB of image rejection puts every mirror below the floor. The measured
    // figure on this hardware is 53-59 dB and at the noise floor
    // (docs/characterization.md), so "finds nothing" is the expected result
    // against the real radio and has to be reachable.
    const std::vector<Tone> tones = {{9.372e8, 45.0}, {1.0538e9, 38.0}};
    ImagePass a = paint(w.pa, w.ba, tones, 80.0, false, w.x0, w.dx, w.n);
    ImagePass b = paint(w.pb, w.bb, tones, 80.0, false, w.x0, w.dx, w.n);

    const ImageCheckResult r = run_image_check(a, b, ImageCheckConfig());
    ok("runs", r.valid);
    ok("finds nothing", r.findings.empty());
    if (!r.findings.empty())
        for (const auto &f : r.findings)
            std::printf("    stray finding at %.3f MHz, %.1f dB over floor, drop %.1f\n",
                        f.x_hz / 1e6, f.excursion_a_db, f.drop_db);
}

void test_per_segment_offset_cancels() {
    std::printf("run_image_check -- fastlock's per-segment level offset cancels\n");
    World w = build_world();
    if (!w.ok) { ok("builds both grids", false); return; }

    // The fixture already gives every segment a different offset in each pass,
    // up to +/-6.3 dB, so a level-to-level comparison would flag whole segments.
    // Demonstrate that directly: the raw difference is large, the excursion
    // difference is not.
    ImagePass a = paint(w.pa, w.ba, {}, 80.0, false, w.x0, w.dx, w.n);
    ImagePass b = paint(w.pb, w.bb, {}, 80.0, false, w.x0, w.dx, w.n);

    double worst_raw = 0;
    for (size_t i = 0; i < w.n; ++i)
        worst_raw = std::max(worst_raw, std::fabs(double(a.db[i]) - double(b.db[i])));
    ok("the raw levels do disagree by several dB", worst_raw > 5.0);

    const ImageCheckResult r = run_image_check(a, b, ImageCheckConfig());
    ok("runs", r.valid);
    ok("and nothing is flagged anyway", r.findings.empty());
    std::printf("    worst raw level disagreement %.1f dB, findings %zu\n",
                worst_raw, r.findings.size());
}

void test_offset_output_grids() {
    std::printf("run_image_check -- the two passes sit on different output grids\n");
    World w = build_world();
    if (!w.ok) { ok("builds both grids", false); return; }

    // Which is what the daemon actually produces: covered_lo moves with the LO
    // grid, so the shifted pass's bin centres are offset from the reference
    // pass's and never coincide. A comparison that indexed by bin number would
    // be reading a different frequency in each pass.
    const std::vector<Tone> tones = {{9.372e8, 45.0}};
    ImagePass a = paint(w.pa, w.ba, tones, 20.0, false, w.x0, w.dx, w.n);
    const double dxb = w.dx * 1.013;           // a different point count, too
    ImagePass b = paint(w.pb, w.bb, tones, 20.0, false,
                        w.x0 + double(w.pb.cfg.lo_shift_hz), dxb, w.n);

    const ImageCheckResult r = run_image_check(a, b, ImageCheckConfig());
    ok("runs", r.valid);
    const double tol = 4 * w.dx;
    ok("the real tone still survives", found_near(r, tones[0].hz, tol) == nullptr);
    const double img = 2 * lo_of(w.ba, tones[0].hz) - tones[0].hz;
    const ImageFinding *f = found_near(r, img, tol);
    ok("the image is still caught", f != nullptr);
    ok("and still called an image", f && f->verdict == ImageVerdict::Image);
}

void test_refusals() {
    std::printf("run_image_check -- refuses what it cannot answer\n");
    World w = build_world();
    if (!w.ok) { ok("builds both grids", false); return; }
    ImagePass a = paint(w.pa, w.ba, {}, 80.0, false, w.x0, w.dx, w.n);
    ImagePass b = paint(w.pb, w.bb, {}, 80.0, false, w.x0, w.dx, w.n);

    ImagePass same = a;
    ok("refuses two passes on the same grid", !run_image_check(a, same, {}).valid);

    ImagePass few = b;
    few.sweeps = 1;
    const ImageCheckResult r = run_image_check(a, few, {});
    ok("refuses a pass averaged from too few sweeps", !r.valid);
    ok("and says why", r.err.find("sweeps") != std::string::npos);

    ImagePass zs = b;
    zs.axis.time = true;
    ok("refuses a zero-span axis", !run_image_check(a, zs, {}).valid);

    ImagePass ragged = b;
    ragged.db.pop_back();
    ok("refuses a pass that does not match its own axis",
       !run_image_check(a, ragged, {}).valid);
}

void test_excursion_clips_to_segments() {
    std::printf("local_excursion -- the floor window stops at the segment boundary\n");
    // A median already survives one seam: the bin's own segment holds the
    // majority of a window centred on it. What it does not survive is a window
    // wider than a segment, which is what a wide span at a modest point count
    // produces -- the median is then taken across a mixture of fastlock's
    // per-segment offsets and the floor under a bin is not the floor that bin
    // was measured against.
    //
    // Segments of 10 bins, offsets alternating by 15 dB, window +/-20 bins.
    const size_t n = 200, seg = 10;
    std::vector<float> db(n);
    std::vector<int> owner(n);
    for (size_t i = 0; i < n; ++i) {
        owner[i] = int(i / seg);
        db[i] = (owner[i] % 2) ? -85.0f : -100.0f;
    }

    const std::vector<float> plain = local_excursion(db, 20);
    const std::vector<float> clipped = local_excursion(db, owner, 20);

    double worst_plain = 0, worst_clipped = 0;
    for (size_t i = 0; i < n; ++i) {
        worst_plain = std::max(worst_plain, std::fabs(double(plain[i])));
        worst_clipped = std::max(worst_clipped, std::fabs(double(clipped[i])));
    }
    ok("a window wider than a segment mixes their offsets", worst_plain > 10.0);
    ok("clipping the window to the owning segment removes it", worst_clipped < 1e-6);
    std::printf("    worst excursion from the offsets alone: unclipped %.1f dB, "
                "clipped %.1f dB\n", worst_plain, worst_clipped);
}

}  // namespace

int main() {
    test_finds_images();
    test_clean_receiver_finds_nothing();
    test_per_segment_offset_cancels();
    test_offset_output_grids();
    test_refusals();
    test_excursion_clips_to_segments();

    std::printf("\n%s\n", g_failures == 0 ? "all passed" : "FAILURES");
    return g_failures == 0 ? 0 : 1;
}
