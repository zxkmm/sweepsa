// Host-side tests for the markers. CLAUDE.md §6, "Markers".
//
// The readout is the part of this UI that gets *quoted*. A trace that is a few
// dB off is a picture somebody might squint at; a marker reading is a number
// somebody writes down. Three of the things pinned here would produce a
// confident wrong number rather than a visible fault:
//
//   * marker noise mode dividing by the output bin width instead of the RBW.
//     On a full span those differ by tens of dB, and the answer still looks
//     like a plausible power density.
//   * a marker being clamped to the edge of the trace when it falls outside the
//     swept span, so it reads the last bin's level under a frequency label from
//     somewhere else entirely.
//   * peak search walking the noise floor because nothing enforces an
//     excursion, which makes "next peak" useless without ever failing.

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "../src/sweepui/marker.h"

using namespace sweepui;

namespace {

int g_failures = 0;
const float kNan = std::numeric_limits<float>::quiet_NaN();

void check(const char *what, bool ok) {
    std::printf("  %-68s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) ++g_failures;
}

void check_near(const char *what, double got, double want, double tol) {
    const bool ok = std::fabs(got - want) <= tol;
    std::printf("  %-68s got=%-13.4f want=%-13.4f %s\n", what, got, want,
                ok ? "ok" : "FAIL");
    if (!ok) ++g_failures;
}

void check_eq(const char *what, long long got, long long want) {
    const bool ok = got == want;
    std::printf("  %-68s got=%-13lld want=%-13lld %s\n", what, got, want,
                ok ? "ok" : "FAIL");
    if (!ok) ++g_failures;
}

// ------------------------------------------------------------------ search

void test_peak_search_finds_the_tone() {
    std::printf("peak search finds the tone, not a noise sample\n");
    std::vector<float> db(200, -100.0f);
    db[137] = -25.0f;
    db[40] = -60.0f;   // a second, smaller signal

    PeakOptions opt;
    size_t bin = 0;
    check("a peak is found", peak_search(db.data(), db.size(), opt, &bin));
    check_eq("and it is the biggest one", (long long)bin, 137);

    check("next peak down is found",
          next_peak_lower(db.data(), db.size(), opt, 137, &bin));
    check_eq("and it is the smaller signal, not a floor sample", (long long)bin, 40);

    check("there is no third peak on a flat floor",
          !next_peak_lower(db.data(), db.size(), opt, 40, &bin));
}

void test_a_flat_floor_has_no_peaks() {
    std::printf("nothing rose, so nothing is a peak\n");
    std::vector<float> db(500, -98.5f);
    PeakOptions opt;
    std::vector<size_t> p;
    find_peaks(db.data(), db.size(), opt, &p);
    check_eq("a perfectly flat trace yields no peaks", (long long)p.size(), 0);
}

void test_excursion_decides_one_peak_or_two() {
    std::printf("the excursion is what separates two peaks from one lumpy one\n");
    std::vector<float> db(40, -100.0f);
    db[10] = -20.0f;
    db[11] = -23.0f;   // a 3 dB notch between two equal tips
    db[12] = -20.0f;

    std::vector<size_t> p;
    PeakOptions six;                       // the default
    find_peaks(db.data(), db.size(), six, &p);
    check_eq("a 3 dB notch does not split a peak at 6 dB excursion",
             (long long)p.size(), 1);
    check_eq("and the peak reported is the first tip", (long long)p[0], 10);

    PeakOptions two;
    two.excursion_db = 2.0;
    find_peaks(db.data(), db.size(), two, &p);
    check_eq("at 2 dB excursion the same notch splits it in two",
             (long long)p.size(), 2);
}

void test_threshold_excludes_the_grass() {
    std::printf("the threshold excludes real-but-uninteresting peaks\n");
    std::vector<float> db(60, -100.0f);
    db[20] = -80.0f;
    db[40] = -30.0f;

    PeakOptions opt;
    std::vector<size_t> p;
    find_peaks(db.data(), db.size(), opt, &p);
    check_eq("both qualify on excursion alone", (long long)p.size(), 2);

    opt.threshold_db = -50.0;
    find_peaks(db.data(), db.size(), opt, &p);
    check_eq("only one clears a -50 dBFS threshold", (long long)p.size(), 1);
    check_eq("and it is the strong one", (long long)p[0], 40);
}

void test_a_peak_at_the_edge_is_still_found() {
    std::printf("a tone at the edge of the view is found on its rise alone\n");
    std::vector<float> db(50, -100.0f);
    db[49] = -30.0f;   // no room for a confirming fall

    PeakOptions opt;
    size_t bin = 0;
    check("the last-bin tone is found", peak_search(db.data(), db.size(), opt, &bin));
    check_eq("at the last bin", (long long)bin, 49);

    // And the boundary case that must NOT qualify: a range whose first sample
    // simply happens to be the highest, with nothing known to its left.
    std::vector<float> ramp(50, -100.0f);
    ramp[0] = -40.0f;
    std::vector<size_t> p;
    find_peaks(ramp.data(), ramp.size(), opt, &p);
    check_eq("but the first sample of the range is not a peak by falling away",
             (long long)p.size(), 0);
}

void test_a_hole_breaks_the_run_but_not_the_peak() {
    std::printf("an unmeasured bin ends a run without discarding what rose\n");
    std::vector<float> db(60, -100.0f);
    db[20] = -30.0f;
    db[21] = kNan;     // the tone's downhill side is inside a blanked hole
    db[22] = kNan;
    db[45] = -50.0f;

    PeakOptions opt;
    std::vector<size_t> p;
    find_peaks(db.data(), db.size(), opt, &p);
    check_eq("both tones are found", (long long)p.size(), 2);
    check_eq("including the one whose fall is unmeasured", (long long)p[0], 20);
    check_eq("and the one after the hole", (long long)p[1], 45);
}

void test_next_left_and_right_are_positional() {
    std::printf("next left / next right walk by frequency, not by amplitude\n");
    std::vector<float> db(120, -100.0f);
    db[20] = -70.0f;
    db[50] = -20.0f;
    db[80] = -60.0f;

    PeakOptions opt;
    size_t bin = 0;
    check("right of 20 is 50", next_peak_right(db.data(), db.size(), opt, 20, &bin) &&
                                   bin == 50);
    check("right of 50 is 80 -- lower, but to the right",
          next_peak_right(db.data(), db.size(), opt, 50, &bin) && bin == 80);
    check("nothing is right of 80", !next_peak_right(db.data(), db.size(), opt, 80, &bin));
    check("left of 80 is 50", next_peak_left(db.data(), db.size(), opt, 80, &bin) &&
                                  bin == 50);
    check("nothing is left of 20", !next_peak_left(db.data(), db.size(), opt, 20, &bin));
}

void test_search_range_is_the_visible_range() {
    std::printf("peak search searches what is on screen\n");
    std::vector<float> db(200, -100.0f);
    db[30] = -20.0f;
    db[150] = -40.0f;

    PeakOptions opt;
    opt.lo = 100;
    opt.hi = 200;
    size_t bin = 0;
    check("a peak outside the range is not returned",
          peak_search(db.data(), db.size(), opt, &bin) && bin == 150);
}

// -------------------------------------------------------------------- axis

void test_axis_matches_the_decimator() {
    std::printf("bin centres agree with decimate.cpp, to within nothing at all\n");
    TraceAxis ax;
    ax.x0 = 100e6;
    ax.dx = 1e3;
    ax.n = 1000;

    check_near("bin 0 is centred half a bin in", ax.x_at(0), 100e6 + 500.0, 1e-9);
    size_t b = 0;
    check("a frequency inside bin 0 lands in bin 0", ax.bin_of(100e6 + 1.0, &b) && b == 0);
    check("its own centre lands in bin 0 too", ax.bin_of(ax.x_at(0), &b) && b == 0);
    check("the last bin's centre lands in the last bin",
          ax.bin_of(ax.x_at(999), &b) && b == 999);
    check("below the start is off the trace", !ax.bin_of(99e6, &b));
    check("above the stop is off the trace", !ax.bin_of(101e6 + 1.0, &b));
}

// The same axis in zero span. Everything above is arithmetic on (x0, dx, n) and
// none of it knows what the units are, which is the whole reason zero span rides
// the same frame and the same marker code -- so this checks that the arithmetic
// really is unit-blind, and that the one thing which is *not* -- the wording of
// a refusal, and what the noise density divides by -- follows the flag.
void test_the_axis_works_in_seconds() {
    std::printf("the same axis, in seconds\n");

    TraceAxis ax;
    ax.x0 = 0.0;                    // zero span measures t from the frame start
    ax.dx = 16.667e-6;              // N=1024 at 61.44 MSPS
    ax.n = 512;
    ax.rbw_hz = 90087.98;           // still a bandwidth: ENBW x fs / n_fft
    ax.time = true;

    check_near("point 0 is centred half a dt in", ax.x_at(0), 0.5 * 16.667e-6, 1e-15);
    size_t b = 0;
    check("a time inside point 0 lands in point 0", ax.bin_of(1e-6, &b) && b == 0);
    check("its own centre lands there too", ax.bin_of(ax.x_at(0), &b) && b == 0);
    check("the last point's centre lands in the last point",
          ax.bin_of(ax.x_at(511), &b) && b == 511);
    // Before the frame started and after it ended are both off the trace, and
    // neither is clamped to an edge: a marker past the end of the captured
    // window has no reading, not the last point's.
    check("before t=0 is off the trace", !ax.bin_of(-1e-9, &b));
    check("past the window is off the trace", !ax.bin_of(512 * 16.667e-6 + 1e-9, &b));

    // The two refusals send the user somewhere different, so they read
    // differently. "outside the swept span" on a time axis would be nonsense.
    TraceAxis freq = ax;
    freq.time = false;
    check("the time axis says window, not span",
          std::string(ax.outside_reason()).find("time window") != std::string::npos);
    check("the frequency axis still says span",
          std::string(freq.outside_reason()).find("swept span") != std::string::npos);

    // A marker on a time axis reads a level the same way, and reports the time.
    std::vector<float> db(512, -100.0f);
    db[100] = -40.0f;
    Marker m;
    m.on = true;
    m.x = ax.x_at(100);
    const MarkerReadout r = read_marker(m, nullptr, db.data(), db.size(), ax);
    check("a marker on a time axis reads its level", r.valid && r.db == -40.0);
    check_near("and reports its time, not a frequency", r.abs_x, ax.x_at(100), 1e-15);
    check("and says the reading is a time", r.time);

    // The noise density still divides by the RBW. The output points are 16.7 us
    // apart and dividing by anything derived from *that* would be dividing a
    // power by a time -- which is not a density, and would still print.
    Marker nm = m;
    nm.noise = true;
    nm.x = ax.x_at(300);
    const MarkerReadout nr = read_marker(nm, nullptr, db.data(), db.size(), ax);
    check("noise mode works on a time axis", nr.valid);
    check_near("and still divides by the RBW", nr.db,
               -100.0 - 10.0 * std::log10(ax.rbw_hz), 0.01);

    // An unmeasured point is a lost block here rather than a blanked DC hole.
    // Different cause, same rule, and the reason says which.
    std::vector<float> holed(512, -100.0f);
    holed[200] = std::numeric_limits<float>::quiet_NaN();
    Marker hm = m;
    hm.x = ax.x_at(200);
    const MarkerReadout hr = read_marker(hm, nullptr, holed.data(), holed.size(), ax);
    check("an unmeasured point is not a reading", !hr.valid && hr.unmeasured);
    check("and the reason names the block, not a segment",
          hr.reason.find("block") != std::string::npos);
}

// ----------------------------------------------------------------- readout

TraceAxis axis(size_t n, double f0 = 100e6, double bin = 1e6, double rbw = 10e3) {
    TraceAxis ax;
    ax.x0 = f0;
    ax.dx = bin;
    ax.n = n;
    ax.rbw_hz = rbw;
    return ax;
}

void test_a_marker_off_span_reads_nothing_rather_than_the_edge() {
    std::printf("a marker outside the swept span is not clamped to it\n");
    std::vector<float> db(100, -70.0f);
    db[99] = -10.0f;   // something loud at the very edge, to be misread

    const TraceAxis ax = axis(100);
    Marker m;
    m.on = true;
    m.x = 500e6;   // well past f_start + 100 * 1 MHz = 200 MHz

    const MarkerReadout r = read_marker(m, nullptr, db.data(), db.size(), ax);
    check("it has no reading", !r.valid);
    check("and it does not report the last bin's -10 dBFS", r.db != -10.0);
    check("the reason names the span", r.reason == "outside the swept span");
    check("and it is not confused with an unmeasured bin", !r.unmeasured);
}

void test_a_marker_in_a_hole_says_so() {
    std::printf("a marker in a dc_policy=blank hole says unmeasured, not quiet\n");
    std::vector<float> db(100, -70.0f);
    db[50] = kNan;

    const TraceAxis ax = axis(100);
    Marker m;
    m.on = true;
    m.x = ax.x_at(50);

    const MarkerReadout r = read_marker(m, nullptr, db.data(), db.size(), ax);
    check("it has no reading", !r.valid);
    check("and it is flagged as unmeasured", r.unmeasured);
    check("distinguishably from being off span", r.reason != "outside the swept span");
}

void test_noise_mode_divides_by_the_rbw_not_the_bin_width() {
    std::printf("marker noise uses the actual RBW -- §6's word, and it matters\n");
    // A flat -100 dBFS floor. RBW 10 kHz, output bins 1 MHz wide: the two
    // differ by 20 dB, which is the whole point.
    std::vector<float> db(100, -100.0f);
    const TraceAxis ax = axis(100, 100e6, 1e6, 10e3);

    Marker m;
    m.on = true;
    m.noise = true;
    m.x = ax.x_at(50);

    const MarkerReadout r = read_marker(m, nullptr, db.data(), db.size(), ax);
    check("it reads", r.valid && r.is_noise);
    check_near("-100 dBFS in a 10 kHz RBW is -140 dBFS/Hz", r.db, -140.0, 1e-6);
    check("and not -160, which is what the 1 MHz bin width would give",
          std::fabs(r.db + 160.0) > 1.0);
}

void test_noise_mode_averages_power_not_db() {
    std::printf("the noise average is in linear power (§5, yet again)\n");
    // Five bins: four at -100 and one at -70, with the marker in the middle.
    // Power mean = (4e-10 + 1e-7)/5 = 2.008e-8 -> -76.977 dBFS.
    // A dB mean would give (4*-100 + -70)/5 = -94 dBFS, 17 dB lower.
    std::vector<float> db = {-100.0f, -100.0f, -70.0f, -100.0f, -100.0f};
    TraceAxis ax = axis(5, 100e6, 1e6, 1.0);   // RBW 1 Hz, so the divisor is 0 dB

    Marker m;
    m.on = true;
    m.noise = true;
    m.x = ax.x_at(2);

    const MarkerReadout r = read_marker(m, nullptr, db.data(), db.size(), ax, 5);
    check_near("five bins averaged in power read -76.97 dBFS/Hz", r.db, -76.9723, 1e-3);
    check("and not -94, which is the dB average", std::fabs(r.db + 94.0) > 5.0);
}

void test_noise_detector_honesty() {
    std::printf("the noise readout is only correct under a mean-preserving detector\n");
    check("RMS is fine", noise_detector_ok(sweepd::Detector::RMS));
    check("Sample is fine", noise_detector_ok(sweepd::Detector::Sample));
    check("Peak is not", !noise_detector_ok(sweepd::Detector::Peak));
    check("Negative Peak is not", !noise_detector_ok(sweepd::Detector::NegativePeak));
}

void test_delta_markers() {
    std::printf("delta markers subtract both axes\n");
    std::vector<float> db(100, -90.0f);
    db[20] = -50.0f;
    db[60] = -20.0f;

    const TraceAxis ax = axis(100);
    Marker ref;
    ref.on = true;
    ref.x = ax.x_at(20);
    Marker d;
    d.on = true;
    d.delta = true;
    d.ref = 0;
    d.x = ax.x_at(60);

    const MarkerReadout r = read_marker(d, &ref, db.data(), db.size(), ax);
    check("the delta reads", r.valid && r.is_delta);
    check_near("30 dB up", r.db, 30.0, 1e-6);
    check_near("and 40 MHz to the right", r.x, 40e6, 1.0);
    check_near("while still reporting its own absolute level", r.abs_db, -20.0, 1e-6);
}

void test_a_delta_with_no_usable_reference_is_not_a_number() {
    std::printf("delta from nothing is refused, not reported as zero\n");
    std::vector<float> db(100, -90.0f);
    const TraceAxis ax = axis(100);

    Marker d;
    d.on = true;
    d.delta = true;
    d.x = ax.x_at(10);

    MarkerReadout r = read_marker(d, nullptr, db.data(), db.size(), ax);
    check("no reference at all is refused", !r.valid);
    check("and the delta reads zero rather than the absolute level in disguise",
          r.db == 0.0 && r.x == 0.0);
    check("while the marker's own level is still available", r.abs_db == -90.0);

    // A reference that is itself off the span.
    Marker ref;
    ref.on = true;
    ref.x = 900e6;
    r = read_marker(d, &ref, db.data(), db.size(), ax);
    check("a reference with no reading is refused too", !r.valid);
    check("and says which end failed",
          r.reason == "the reference marker has no reading");

    // Referring to itself.
    r = read_marker(d, &d, db.data(), db.size(), ax);
    check("a marker cannot be its own reference", !r.valid);
}

void test_delta_refuses_to_mix_a_density_with_a_level() {
    std::printf("a level minus a density is an RBW, not a ratio\n");
    std::vector<float> db(100, -90.0f);
    const TraceAxis ax = axis(100);

    Marker ref;
    ref.on = true;
    ref.noise = true;                 // a density
    ref.x = ax.x_at(10);
    Marker d;
    d.on = true;
    d.delta = true;                   // a level
    d.x = ax.x_at(50);

    const MarkerReadout r = read_marker(d, &ref, db.data(), db.size(), ax);
    check("the mixed subtraction is refused", !r.valid);
    check("with a reason that names the mismatch",
          r.reason == "cannot subtract a noise density from a level");
}

void test_marker_set_basics() {
    std::printf("eight markers, and the set knows when none is on\n");
    MarkerSet s;
    check_eq("there are eight", (long long)s.size(), 8);
    check("none is on to start with", !s.any_on());
    s[3].on = true;
    s.set_active(3);
    check("one is now on", s.any_on());
    check("and it is the active one", &s.current() == &s[3]);
    s.all_off();
    check("all off clears them", !s.any_on());
}

}  // namespace

int main() {
    test_peak_search_finds_the_tone();
    test_a_flat_floor_has_no_peaks();
    test_excursion_decides_one_peak_or_two();
    test_threshold_excludes_the_grass();
    test_a_peak_at_the_edge_is_still_found();
    test_a_hole_breaks_the_run_but_not_the_peak();
    test_next_left_and_right_are_positional();
    test_search_range_is_the_visible_range();
    test_axis_matches_the_decimator();
    test_the_axis_works_in_seconds();
    test_a_marker_off_span_reads_nothing_rather_than_the_edge();
    test_a_marker_in_a_hole_says_so();
    test_noise_mode_divides_by_the_rbw_not_the_bin_width();
    test_noise_mode_averages_power_not_db();
    test_noise_detector_honesty();
    test_delta_markers();
    test_a_delta_with_no_usable_reference_is_not_a_number();
    test_delta_refuses_to_mix_a_density_with_a_level();
    test_marker_set_basics();

    std::printf("\n%s (%d failure%s)\n", g_failures ? "FAILED" : "all passed",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures ? 1 : 0;
}
