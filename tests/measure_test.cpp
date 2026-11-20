// Host tests for the band measurements. The traps: integrating dB instead of
// power, and integrating against the point spacing alone, which makes the
// answer depend on how many points the trace happens to have.

#include <cmath>
#include <cstdio>
#include <vector>

#include "measure.h"

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

TraceAxis axis(double x0, double dx, size_t n, double rbw) {
    TraceAxis a;
    a.x0 = x0;
    a.dx = dx;
    a.n = n;
    a.rbw_hz = rbw;
    return a;
}

void test_flat_noise() {
    std::printf("channel power\n");
    // -100 dB per 10 kHz RBW is -140 dB/Hz. 1 MHz of it is -80 dB, however the
    // points are spaced.
    for (double dx : {1e3, 10e3, 50e3}) {
        const size_t n = size_t(10e6 / dx);
        std::vector<float> v(n, -100.0f);
        const BandResult r =
            channel_power(v.data(), n, axis(0, dx, n, 10e3), 4.5e6, 5.5e6, sweepd::Detector::RMS);
        char what[96];
        std::snprintf(what, sizeof what, "1 MHz of -140 dB/Hz is -80 dB at %.0f kHz/point",
                      dx / 1e3);
        check_near(what, r.power_db, -80.0, 1e-6);
        if (dx == 10e3) check_near("PSD reads back -140 dB/Hz", r.psd_db_hz, -140.0, 1e-6);
    }
    std::vector<float> v(1000, -100.0f);
    const BandResult p =
        channel_power(v.data(), 1000, axis(0, 10e3, 1000, 10e3), 0, 1e6, sweepd::Detector::Peak);
    check_true("the Peak detector is warned about", !p.warning.empty());
    // An edge half way through a point counts half of it.
    const BandResult h =
        channel_power(v.data(), 1000, axis(0, 10e3, 1000, 10e3), 0, 15e3, sweepd::Detector::RMS);
    check_near("a half-covered point counts half", h.power_db, -100.0 + 10 * std::log10(1.5),
               1e-6);
}

void test_tone_power() {
    // A CW is one RBW wide: whatever the point spacing, a band around it holds
    // the tone's own level (plus a sliver of floor).
    std::vector<float> v(1000, -150.0f);
    for (size_t i = 495; i < 505; ++i) v[i] = -30.0f;   // 10 points = one 10 kHz RBW at 1 kHz
    const BandResult r =
        channel_power(v.data(), 1000, axis(0, 1e3, 1000, 10e3), 400e3, 600e3, sweepd::Detector::Sample);
    check_near("a tone's channel power is its level", r.power_db, -30.0, 0.01);
}

void test_obw() {
    std::printf("occupied bandwidth\n");
    // Flat 200 kHz of signal in a 1 MHz band: 99% OBW is 198 kHz, centred.
    std::vector<float> v(1000, -200.0f);
    for (size_t i = 400; i < 600; ++i) v[i] = -50.0f;
    const ObwResult r = occupied_bw(v.data(), 1000, axis(0, 1e3, 1000, 1e3), 0, 1e6);
    check_true("valid", r.valid);
    check_near("99% of a flat 200 kHz block is 198 kHz", r.width_hz, 198e3, 1.0);
    check_near("and centred on it", (r.f_lo + r.f_hi) / 2, 500e3, 1.0);
    // In dB the same block averaged would put the edges elsewhere; the floor
    // must not pull them outwards.
    check_true("the -200 floor does not widen it", r.f_lo > 399e3 && r.f_hi < 601e3);
}

}  // namespace

int main() {
    std::printf("band measurement tests\n\n");
    test_flat_noise();
    test_tone_power();
    std::printf("\n");
    test_obw();
    std::printf("\n%s\n", g_failures ? "FAILURES" : "all passed");
    return g_failures ? 1 : 0;
}
