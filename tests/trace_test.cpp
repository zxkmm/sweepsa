// Host-side tests for the four traces. CLAUDE.md §6, "Traces".
//
// Like the decimator's tests, everything guarded here fails as a picture rather
// than as an error. Three of them are worth naming:
//
//   * an Average that averages dB reads a few dB low on noise and correctly on
//     a strong tone, so it looks like a working average with a slightly better
//     noise floor -- which is precisely the flattering-and-wrong result §7
//     exists to prevent, and which the marker noise readout would then divide
//     by the RBW and present as a power density
//   * a Min Hold that treats NaN as data draws a solid line across every
//     dc_policy=blank hole: a picture of a measurement that never happened
//   * a Max Hold that inherits the buffer on a mode change reports a peak from
//     before the hold started, with no way to tell from the screen

#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "../src/sweepui/trace.h"

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
    std::printf("  %-68s got=%-11.4f want=%-11.4f %s\n", what, got, want,
                ok ? "ok" : "FAIL");
    if (!ok) ++g_failures;
}

std::vector<float> flat(size_t n, float v) { return std::vector<float>(n, v); }

void push(Trace *t, const std::vector<float> &v) { t->update(v.data(), v.size()); }

sweepd::proto::TraceMsg frame(long long f0, long long f1, size_t n, float v,
                              sweepd::Detector d = sweepd::Detector::Peak,
                              sweepd::Window w = sweepd::Window::Hann) {
    sweepd::proto::TraceMsg m;
    m.f_start_hz = f0;
    m.f_stop_hz = f1;
    m.detector = d;
    m.window = w;
    m.db.assign(n, v);
    return m;
}

// ------------------------------------------------------------------ average

// The rule from §5, on the display side. Two sweeps, 0 dBFS and -20 dBFS.
// Linear power: (1.0 + 0.01)/2 = 0.505 -> -2.967 dBFS.
// Averaging dB:  (0 + -20)/2 = -10 dBFS, which is 7 dB lower and wrong.
void test_average_is_in_power_not_db() {
    std::printf("Average(N) averages linear power, not dB (§5, third copy)\n");
    Trace t;
    t.set_mode(TraceMode::Average);
    t.set_average_n(4);

    push(&t, flat(8, 0.0f));
    push(&t, flat(8, -20.0f));

    check_near("two sweeps of 0 and -20 dBFS average to -2.967", double(t.db()[0]),
               -2.9671, 1e-3);
    check("and emphatically not to -10 (which is what averaging dB gives)",
          std::fabs(double(t.db()[0]) + 10.0) > 5.0);
}

void test_average_converges_then_tracks() {
    std::printf("Average(N) converges over N sweeps, then keeps tracking\n");
    Trace t;
    t.set_mode(TraceMode::Average);
    t.set_average_n(10);

    for (int i = 0; i < 9; ++i) push(&t, flat(4, -100.0f));
    check("nine sweeps into a 10-average is not converged", !t.converged());
    check_near("but it already reads the right value, not a fraction of it",
               double(t.db()[0]), -100.0, 1e-4);

    push(&t, flat(4, -100.0f));
    check("the tenth sweep converges it", t.converged());

    // One full-scale sweep after convergence. The weight is 1/N, so the mean
    // power becomes ~0.1 and the trace reads about -10 dBFS. A frozen average
    // would still read -100 and a Clear/Write would read 0.
    push(&t, flat(4, 0.0f));
    check_near("a full-scale sweep moves a converged average by 1/N of the power",
               double(t.db()[0]), -10.0, 0.01);
}

void test_average_weights_each_bin_by_its_own_sample_count() {
    std::printf("a bin that was NaN for some sweeps is weighted by what it saw\n");
    Trace t;
    t.set_mode(TraceMode::Average);
    t.set_average_n(8);

    // Bin 0 sees -20, -20. Bin 1 sees NaN, -20.
    std::vector<float> a = {-20.0f, kNan};
    std::vector<float> b = {-20.0f, -20.0f};
    push(&t, a);
    push(&t, b);

    check_near("bin 0, two samples of -20, reads -20", double(t.db()[0]), -20.0, 1e-4);
    check_near("bin 1, one sample of -20, also reads -20 -- not half of it",
               double(t.db()[1]), -20.0, 1e-4);
}

// -------------------------------------------------------------------- holds

void test_max_hold_ignores_unmeasured_bins() {
    std::printf("NaN is unmeasured, so it neither sets nor clears a hold\n");
    Trace t;
    t.set_mode(TraceMode::MaxHold);

    push(&t, flat(4, -80.0f));
    std::vector<float> spur = {-80.0f, -10.0f, -80.0f, -80.0f};
    push(&t, spur);
    push(&t, flat(4, kNan));   // a whole sweep of holes

    check_near("the held peak survives a sweep of NaN", double(t.db()[1]), -10.0, 1e-6);
    check_near("so does the floor", double(t.db()[0]), -80.0, 1e-6);
}

void test_min_hold_leaves_permanent_holes_as_holes() {
    std::printf("Min Hold does not draw a line across a dc_policy=blank hole\n");
    Trace t;
    t.set_mode(TraceMode::MinHold);

    for (int i = 0; i < 5; ++i) {
        std::vector<float> v = {-60.0f, kNan, -60.0f};
        push(&t, v);
    }
    check("a bin that was never measured is still NaN after five sweeps",
          std::isnan(t.db()[1]));
    check_near("the measured bins hold their minimum", double(t.db()[0]), -60.0, 1e-6);

    // And a real value arriving late is taken, rather than being rejected for
    // failing to beat the NaN it is replacing.
    std::vector<float> late = {-60.0f, -30.0f, -60.0f};
    push(&t, late);
    check_near("the first real sample of a previously-empty bin is taken",
               double(t.db()[1]), -30.0, 1e-6);
}

void test_clear_write_shows_the_holes() {
    std::printf("Clear/Write passes NaN through -- it is showing this sweep\n");
    Trace t;
    t.set_mode(TraceMode::ClearWrite);
    push(&t, flat(4, -50.0f));
    std::vector<float> v = {-50.0f, kNan, -50.0f, -50.0f};
    push(&t, v);
    check("a hole in this sweep is a hole on screen", std::isnan(t.db()[1]));
}

// ------------------------------------------------------------------- modes

void test_entering_a_hold_clears() {
    std::printf("entering a hold clears, so it cannot report a peak it never held\n");
    Trace t;
    t.set_mode(TraceMode::ClearWrite);
    std::vector<float> big = {-10.0f, -10.0f};
    push(&t, big);
    check_near("clear/write shows the big sweep", double(t.db()[0]), -10.0, 1e-6);

    t.set_mode(TraceMode::MaxHold);
    check("the buffer is dropped on entering Max Hold", t.empty());
    check("and so is the sweep count it would be reported against", t.sweeps() == 0);

    std::vector<float> small = {-90.0f, -90.0f};
    push(&t, small);
    check_near("so the held maximum is -90, not the inherited -10",
               double(t.db()[0]), -90.0, 1e-6);
}

void test_view_and_blank_freeze_but_differ_in_visibility() {
    std::printf("View freezes and is drawn; Blank freezes and is not\n");
    Trace t;
    t.set_mode(TraceMode::MaxHold);
    push(&t, flat(4, -40.0f));

    t.set_mode(TraceMode::View);
    push(&t, flat(4, 0.0f));
    check_near("View ignores incoming sweeps", double(t.db()[0]), -40.0, 1e-6);
    check("View is drawn", t.visible());

    t.set_mode(TraceMode::Blank);
    push(&t, flat(4, 0.0f));
    check_near("Blank ignores them too", double(t.db()[0]), -40.0, 1e-6);
    check("Blank is not drawn", !t.visible());

    // Un-blanking restores what was frozen rather than starting empty, which is
    // what makes Blank a display control and not a delete key.
    t.set_mode(TraceMode::View);
    check_near("un-blanking restores the frozen trace", double(t.db()[0]), -40.0, 1e-6);
}

void test_changing_n_clears_the_average() {
    std::printf("changing N clears, so the trace is never a blend of two weights\n");
    Trace t;
    t.set_mode(TraceMode::Average);
    t.set_average_n(4);
    push(&t, flat(4, -30.0f));
    check("the average has a sweep in it", t.sweeps() == 1);
    t.set_average_n(16);
    check("changing N drops it", t.sweeps() == 0 && t.empty());
}

// ------------------------------------------------------------------- store

void test_store_clears_when_the_data_stops_being_comparable() {
    std::printf("TraceStore clears a hold that would span incomparable sweeps\n");
    TraceStore s;
    s[0].set_mode(TraceMode::MaxHold);

    check("the first frame is not a change", !s.push(frame(100e6, 200e6, 16, -70.0f)));
    check_near("and it is held", double(s[0].db()[0]), -70.0, 1e-6);

    check("a span change clears", s.push(frame(100e6, 300e6, 16, -90.0f)));
    check_near("so the hold is the new sweep, not the old maximum",
               double(s[0].db()[0]), -90.0, 1e-6);

    s.push(frame(100e6, 300e6, 16, -90.0f));
    check("a point-count change clears", s.push(frame(100e6, 300e6, 32, -95.0f)));

    s.push(frame(100e6, 300e6, 32, -95.0f));
    check("a detector change clears -- peak and RMS bins are not one quantity",
          s.push(frame(100e6, 300e6, 32, -95.0f, sweepd::Detector::RMS)));

    s.push(frame(100e6, 300e6, 32, -95.0f, sweepd::Detector::RMS));
    check("a window change clears -- the RBW moved under the trace",
          s.push(frame(100e6, 300e6, 32, -95.0f, sweepd::Detector::RMS,
                       sweepd::Window::FlatTop)));

    check("an unchanged frame does not clear",
          !s.push(frame(100e6, 300e6, 32, -95.0f, sweepd::Detector::RMS,
                        sweepd::Window::FlatTop)));
}

// The largest version of "the data stopped being comparable": the axis changed
// units. A max hold carried from a sweep into a zero span would draw peaks found
// at frequencies on top of a trace measured over time -- a picture of a
// measurement that never happened, and one that would look entirely plausible.
void test_store_clears_when_the_axis_changes_kind() {
    std::printf("entering and leaving zero span clears the holds\n");

    TraceStore s;
    s[0].set_mode(TraceMode::MaxHold);

    sweepd::proto::TraceMsg swept = frame(900000000LL, 1100000000LL, 64, -50.0f);
    swept.bin_hz = 200e6 / 64.0;
    check("the first frame is not a change", !s.push(swept));
    check_near("the hold took it", s[0].db()[0], -50.0, 1e-6);

    // Same point count, same detector, same window -- only x_kind differs. If
    // that were not in the key this would silently keep the -50 dBFS hold.
    sweepd::proto::TraceMsg zs;
    zs.f_start_hz = zs.f_stop_hz = 1000000000LL;
    zs.bin_hz = 0.0;
    zs.x_kind = sweepd::proto::XKind::Time;
    zs.dt_ns = 16667;
    zs.detector = sweepd::Detector::Peak;
    zs.window = sweepd::Window::Hann;
    zs.db.assign(64, -90.0f);
    check("switching to a time axis is a change", s.push(zs));
    check_near("and the hold restarted from the zero-span data", s[0].db()[0], -90.0, 1e-6);

    // dt is in the key too: retuning zero span to a different FFT size changes
    // how long each point integrates over, so the points are not comparable
    // either, even though both traces are in seconds.
    sweepd::proto::TraceMsg faster = zs;
    faster.dt_ns = 4167;
    faster.db.assign(64, -70.0f);
    check("a different dt is a change", s.push(faster));
    check_near("and restarts again", s[0].db()[0], -70.0, 1e-6);

    // Same frame twice is not a change, or every sweep would clear the hold.
    sweepd::proto::TraceMsg again = faster;
    again.db.assign(64, -80.0f);
    check("the same axis twice is not a change", !s.push(again));
    check_near("so the hold holds", s[0].db()[0], -70.0, 1e-6);

    check("going back to a sweep is a change", s.push(swept));
}

void test_store_first_visible_skips_blanked_traces() {
    std::printf("the markers' default trace is one that is actually on screen\n");
    TraceStore s;
    check("nothing visible before the first sweep", s.first_visible() == TraceStore::kCount);

    s[0].set_mode(TraceMode::Blank);
    s[2].set_mode(TraceMode::ClearWrite);
    s.push(frame(100e6, 200e6, 8, -70.0f));
    check("trace 3 is the first visible one", s.first_visible() == 2);
}

}  // namespace

int main() {
    test_average_is_in_power_not_db();
    test_average_converges_then_tracks();
    test_average_weights_each_bin_by_its_own_sample_count();
    test_max_hold_ignores_unmeasured_bins();
    test_min_hold_leaves_permanent_holes_as_holes();
    test_clear_write_shows_the_holes();
    test_entering_a_hold_clears();
    test_view_and_blank_freeze_but_differ_in_visibility();
    test_changing_n_clears_the_average();
    test_store_clears_when_the_data_stops_being_comparable();
    test_store_clears_when_the_axis_changes_kind();
    test_store_first_visible_skips_blanked_traces();

    std::printf("\n%s (%d failure%s)\n", g_failures ? "FAILED" : "all passed",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures ? 1 : 0;
}
