// Host tests for the tracking generator's thru normalization. The two rules
// that fail silently: a reference applied over a different step layout, which
// moves the band-switch step rather than removing it, and averaging in dB.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "tracknorm.h"

using namespace sweepui;

namespace {

int g_failures = 0;

void check_true(const char *what, bool ok) {
    std::printf("  %-62s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) ++g_failures;
}

void check_near(const char *what, double got, double want, double tol) {
    const bool ok = std::fabs(got - want) <= tol;
    std::printf("  %-62s %s (got %.4f, want %.4f +/-%.4f)\n", what, ok ? "ok" : "FAIL",
                got, want, tol);
    if (!ok) ++g_failures;
}

TrackGrid grid() {
    TrackGrid g;
    g.f_start_hz = 70e6;
    g.bin_hz = 1.02e6;
    g.n = 100;
    g.tones = 39;
    g.atten_db = 30;
    g.gain_db = 20;
    return g;
}

void test_thru_divides_out() {
    std::printf("normalization\n");
    const TrackGrid g = grid();
    std::vector<float> path(g.n), dut(g.n);
    for (size_t i = 0; i < g.n; ++i) {
        path[i] = float(-30.0 - 0.3 * double(i) + (i >= 60 ? 14.0 : 0.0));
        dut[i] = float(i > 40 && i < 50 ? 0.0 : -55.0);
    }
    TrackNorm n;
    n.start_capture(1);
    check_true("one sweep completes a one-sweep capture", n.feed(g, path.data(), g.n));
    check_true("a reference is held", n.have());

    std::vector<float> meas(g.n);
    for (size_t i = 0; i < g.n; ++i) meas[i] = path[i] + dut[i];
    double comp = 0;
    const std::string r = n.apply(g, meas.data(), g.n, &comp);
    check_true("applied", r.empty());
    double worst = 0;
    for (size_t i = 0; i < g.n; ++i) worst = std::max(worst, std::fabs(double(meas[i] - dut[i])));
    check_near("the path and its band step are gone", worst, 0.0, 1e-4);

    // More drive for the DUT: 10 dB less attenuation reads 10 dB hotter and is
    // compensated back.
    TrackGrid hot = g;
    hot.atten_db = 20;
    for (size_t i = 0; i < g.n; ++i) meas[i] = path[i] + dut[i] + 10.0f;
    check_true("an attenuation change still applies", n.apply(hot, meas.data(), g.n, &comp).empty());
    check_near("and is compensated by the difference", comp, -10.0, 1e-9);
    check_near("leaving S21", meas[45], dut[45], 1e-4);
}

void test_refusals() {
    std::printf("refusals\n");
    const TrackGrid g = grid();
    std::vector<float> v(g.n, -40.0f);
    TrackNorm n;
    n.start_capture(1);
    n.feed(g, v.data(), g.n);

    TrackGrid other = g;
    other.tones = 8;
    std::vector<float> m = v;
    check_true("a different tones-per-step layout is refused",
               !n.apply(other, m.data(), g.n, nullptr).empty() && m[0] == -40.0f);
    other = g;
    other.f_start_hz += 1.02e6;
    check_true("a shifted start is refused", !n.apply(other, m.data(), g.n, nullptr).empty());
    other = g;
    other.gain_db = 30;
    check_true("an RX gain change is refused", !n.apply(other, m.data(), g.n, nullptr).empty());
}

void test_average_is_linear() {
    std::printf("averaging\n");
    TrackGrid g = grid();
    g.n = 1;
    TrackNorm n;
    n.start_capture(2);
    const float a = -10.0f, b = -30.0f;
    check_true("first of two does not complete", !n.feed(g, &a, 1));
    check_true("second completes", n.feed(g, &b, 1));
    // mean of 0.1 and 0.001 = 0.0505 -> -12.967 dB; the dB mean would be -20.
    check_near("average of -10 and -30 dB is in power", n.ref_db()[0], -12.967, 0.001);

    std::string why;
    n.start_capture(3);
    n.feed(g, &a, 1);
    TrackGrid g2 = g;
    g2.atten_db = 40;
    n.feed(g2, &a, 1, &why);
    check_true("a grid change mid-capture restarts it and says so",
               !why.empty() && n.captured() == 1);
}

}  // namespace

int main() {
    std::printf("tracking normalization tests\n\n");
    test_thru_divides_out();
    std::printf("\n");
    test_refusals();
    std::printf("\n");
    test_average_is_linear();
    std::printf("\n%s\n", g_failures ? "FAILURES" : "all passed");
    return g_failures ? 1 : 0;
}
