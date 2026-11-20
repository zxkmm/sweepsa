// Host tests for the generator's waveforms. What must hold: the buffer is
// exactly periodic (no phase step at the wrap, which would be a spur comb at
// fs/L), the achieved rate is reported, and the spectral content is what the
// modulation says -- checked by direct DFT at the expected lines.

#include <cmath>
#include <complex>
#include <cstdio>
#include <string>
#include <vector>

#include "../src/sweepd/gen.h"

using namespace sweepd;

namespace {

int g_failures = 0;
constexpr double kFs = 61440000.0;

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

// Power of the buffer's DFT at `f` Hz, relative to a full-scale carrier, dB.
// f must be a multiple of fs/n for the buffer to hold it exactly.
double line_db(const std::vector<int16_t> &iq, double f) {
    const size_t n = iq.size() / 2;
    std::complex<double> acc = 0;
    for (size_t i = 0; i < n; ++i) {
        const double a = -2.0 * M_PI * f * double(i) / kFs;
        acc += std::complex<double>(iq[2 * i], iq[2 * i + 1]) *
               std::complex<double>(std::cos(a), std::sin(a));
    }
    return 20.0 * std::log10(std::abs(acc) / double(n) / 32767.0 + 1e-15);
}

// The jump across the wrap against the largest step inside the buffer.
double wrap_ratio(const std::vector<int16_t> &iq) {
    const size_t n = iq.size() / 2;
    auto step = [&](size_t a, size_t b) {
        return std::hypot(double(iq[2 * a] - iq[2 * b]), double(iq[2 * a + 1] - iq[2 * b + 1]));
    };
    double mx = 1;
    for (size_t i = 1; i < n; ++i) mx = std::max(mx, step(i, i - 1));
    return step(0, n - 1) / mx;
}

void test_cw() {
    std::printf("cw\n");
    GenConfig c;
    GenDerived d;
    std::string err;
    check_true("derives", derive_gen(c, kFs, 70000000, 6000000000LL, &d, &err));
    std::vector<int16_t> iq;
    gen_waveform(c, d, kFs, &iq);
    check_near("all the power at DC, at the scale asked for", line_db(iq, 0), 20 * std::log10(0.7),
               0.01);
    c.freq_hz = 10000000;
    check_true("a carrier below the tuner is refused",
               !derive_gen(c, kFs, 70000000, 6000000000LL, &d, &err));
}

void test_am_fm() {
    std::printf("modulation\n");
    GenConfig c;
    c.mod = GenMod::AM;
    c.rate_hz = 1000;
    c.am_depth = 0.5;
    GenDerived d;
    std::string err;
    check_true("AM derives", derive_gen(c, kFs, 70000000, 6000000000LL, &d, &err));
    check_near("1 kHz is exactly achievable at 61.44 MSPS", d.rate_hz, 1000.0, 1e-9);
    std::vector<int16_t> iq;
    gen_waveform(c, d, kFs, &iq);
    // m = 0.5: each sideband is m/2 of the carrier, -12.04 dB.
    check_near("AM sidebands at -12 dBc", line_db(iq, 1000) - line_db(iq, 0), -12.04, 0.05);
    check_true("AM wraps without a step", wrap_ratio(iq) < 1.01);

    c.mod = GenMod::FM;
    c.fm_dev_hz = 2405;   // beta 2.405: the first carrier null
    derive_gen(c, kFs, 70000000, 6000000000LL, &d, &err);
    gen_waveform(c, d, kFs, &iq);
    check_true("FM at beta 2.405 nulls the carrier (> 30 dB down)",
               line_db(iq, 0) - line_db(iq, 1000) < -30);
    check_true("FM wraps without a step", wrap_ratio(iq) < 1.01);

    c.rate_hz = 7.0;
    check_true("a rate slower than the buffer holds is refused, with the floor",
               !derive_gen(c, kFs, 70000000, 6000000000LL, &d, &err) &&
                   err.find("slowest") != std::string::npos);
    c.rate_hz = 1234.5;
    derive_gen(c, kFs, 70000000, 6000000000LL, &d, &err);
    check_true("an inexact rate is snapped and reported",
               d.rate_hz != 1234.5 && std::fabs(d.rate_hz - 1234.5) < 0.1);
}

void test_sweep() {
    std::printf("sweeper\n");
    GenConfig c;
    c.sweep_start_hz = 100000000;
    c.sweep_stop_hz = 200000000;
    c.sweep_step_hz = 30000000;
    GenDerived d;
    std::string err;
    check_true("derives", derive_gen(c, kFs, 70000000, 6000000000LL, &d, &err));
    check_true("100..200 by 30 is 100, 130, 160, 190, 200", d.sweep_points == 5);
    check_true("the last step is clamped to stop",
               gen_sweep_freq(c, 3) == 190000000 && gen_sweep_freq(c, 4) == 200000000);
    c.sweep_step_hz = 50000000;
    derive_gen(c, kFs, 70000000, 6000000000LL, &d, &err);
    check_true("on-grid: 100, 150, 200", d.sweep_points == 3);
}

}  // namespace

int main() {
    std::printf("generator tests\n\n");
    test_cw();
    std::printf("\n");
    test_am_fm();
    std::printf("\n");
    test_sweep();
    std::printf("\n%s\n", g_failures ? "FAILURES" : "all passed");
    return g_failures ? 1 : 0;
}
