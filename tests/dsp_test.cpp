// Host-side tests for the sweepd DSP core. No device required.
//
// These exist because §5 contains two traps that are easy to get wrong and
// invisible once wrong: RBW must be *derived* from the window, and the RMS
// detector must average linear power, never dB.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "../src/sweepd/dsp.h"

using namespace sweepd;

namespace {

int g_failures = 0;

void check_near(const char *what, double got, double want, double tol) {
    const bool ok = std::fabs(got - want) <= tol;
    std::printf("  %-52s got=%-12.6f want=%-12.6f %s\n", what, got, want,
                ok ? "ok" : "FAIL");
    if (!ok) ++g_failures;
}

void check_true(const char *what, bool ok) {
    std::printf("  %-52s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) ++g_failures;
}

// ENBW reference values are the textbook figures for these windows (Harris
// 1978). Computing them numerically and comparing against the literature
// validates make_window() rather than restating a constant.
void test_window_enbw() {
    std::printf("window ENBW (computed from coefficients, vs Harris 1978)\n");
    const size_t n = 4096;
    check_near("rectangular", make_window(Window::Rectangular, n).enbw, 1.0, 1e-6);
    check_near("hann", make_window(Window::Hann, n).enbw, 1.5, 1e-3);
    check_near("blackman-harris", make_window(Window::BlackmanHarris, n).enbw, 2.0044, 1e-3);
    check_near("flat-top", make_window(Window::FlatTop, n).enbw, 3.77, 2e-2);
}

// A full-scale complex sinusoid must read 0 dBFS regardless of window. This is
// what makes the reported numbers comparable across window choices.
void test_full_scale_is_0_dbfs() {
    std::printf("full-scale complex sinusoid reads 0 dBFS\n");
    const size_t n = 1024;
    for (Window wt : {Window::Rectangular, Window::Hann, Window::BlackmanHarris,
                      Window::FlatTop}) {
        const WindowFunction win = make_window(wt, n);
        Fft fft(n);

        // Exactly on bin +64 so there is no scalloping loss to confound this.
        const int bin = 64;
        std::vector<float> iq(2 * n);
        for (size_t i = 0; i < n; ++i) {
            const double ph = 2.0 * M_PI * double(bin) * double(i) / double(n);
            iq[2 * i + 0] = float(std::cos(ph));
            iq[2 * i + 1] = float(std::sin(ph));
        }

        std::vector<float> pw(n);
        fft.power_spectrum(iq.data(), win, pw.data());
        power_to_db(pw.data(), n);

        double peak = -1e9;
        size_t peak_idx = 0;
        for (size_t k = 0; k < n; ++k)
            if (pw[k] > peak) { peak = pw[k]; peak_idx = k; }

        check_near(window_name(wt), peak, 0.0, 0.05);
        // fftshift puts DC at n/2, so bin +64 must land at n/2 + 64.
        check_true("  peak lands at fftshifted position", peak_idx == n / 2 + bin);
    }
}

// The trap §5 calls out by name. Averaging dB and averaging power give
// different answers; this pins down which one we do.
void test_rms_averages_power_not_db() {
    std::printf("RMS detector averages linear power, not dB (§5)\n");
    // Two bins: 1.0 and 100.0 linear power == 0 dB and 20 dB.
    //   correct   : mean power 50.5 -> 10*log10(50.5) = 17.033 dB
    //   wrong (dB): (0 + 20)/2      =                    10.000 dB
    std::vector<float> power = {1.0f, 100.0f};
    float out = 0.0f;
    detect(power.data(), power.size(), Detector::RMS, &out, 1);
    power_to_db(&out, 1);
    check_near("mean of power, then dB", out, 17.0332, 1e-3);
    check_true("is NOT the mean of dB (10.0)", std::fabs(out - 10.0) > 1.0);
}

void test_other_detectors() {
    std::printf("peak / negative-peak / sample detectors\n");
    std::vector<float> power = {1.0f, 100.0f, 5.0f, 2.0f};
    float out = 0.0f;
    detect(power.data(), power.size(), Detector::Peak, &out, 1);
    check_near("peak", out, 100.0, 1e-6);
    detect(power.data(), power.size(), Detector::NegativePeak, &out, 1);
    check_near("negative-peak", out, 1.0, 1e-6);
}

// §5: "Output point count independent of FFT size."
void test_output_count_independent_of_fft_size() {
    std::printf("output point count is independent of FFT size (§5)\n");
    std::vector<float> power(4096);
    for (size_t i = 0; i < power.size(); ++i) power[i] = float(i);

    for (size_t n_out : {1u, 7u, 100u, 1024u, 8192u}) {
        std::vector<float> out(n_out, -1.0f);
        detect(power.data(), power.size(), Detector::Peak, out.data(), n_out);
        bool all_written = true;
        for (float v : out) if (v < 0.0f) all_written = false;
        char buf[64];
        std::snprintf(buf, sizeof buf, "n_out=%zu fully written", n_out);
        check_true(buf, all_written);
    }
    // Monotonic input with a peak detector must give monotonic output.
    std::vector<float> out(64);
    detect(power.data(), power.size(), Detector::Peak, out.data(), out.size());
    bool mono = true;
    for (size_t i = 1; i < out.size(); ++i) if (out[i] < out[i - 1]) mono = false;
    check_true("monotonic in -> monotonic out", mono);
}

// §5: "RBW is derived, not asserted ... If the user asks for one the FFT can't
// deliver, snap and say so."
void test_rbw_derivation() {
    std::printf("RBW derivation and snapping (§5)\n");
    const double fs = 61.44e6;
    const WindowFunction hann = make_window(Window::Hann, 1024);

    check_near("hann, fs=61.44M, N=1024", rbw_hz(hann.enbw, fs, 1024), 90000.0, 200.0);
    check_near("hann, fs=61.44M, N=4096", rbw_hz(hann.enbw, fs, 4096), 22500.0, 50.0);

    double achieved = 0;
    const size_t n = fft_size_for_rbw(30000.0, hann.enbw, fs, 32, 65536, &achieved);
    check_true("asking 30 kHz snaps to N=4096", n == 4096);
    check_true("achieved RBW is at least as fine as requested", achieved <= 30000.0);

    // An impossible request must clamp at max_n rather than loop forever.
    const size_t n2 = fft_size_for_rbw(0.001, hann.enbw, fs, 32, 8192, &achieved);
    check_true("impossible request clamps to max_n", n2 == 8192);
}

void test_overload_counter() {
    std::printf("overload counter (§5)\n");
    std::vector<int16_t> iq = {0, 0, 32000, 5, -32000, 0, 100, 100};
    check_true("counts both polarities",
               count_overload(iq.data(), iq.size() / 2, 30000) == 2);
    check_true("quiet signal reports zero",
               count_overload(iq.data(), iq.size() / 2, 32760) == 0);
}

// §6's video trigger. Every failure in here is a trace that looks fine and is
// aligned to the wrong instant, so the cases pinned are the ones where a wrong
// answer is still a plausible picture.
void test_video_trigger() {
    std::printf("video trigger (§6)\n");

    // A step at index 5, in linear power: floor 0.01 (-20 dB), burst 1.0 (0 dB).
    std::vector<float> step(20, 0.01f);
    for (size_t i = 5; i < 12; ++i) step[i] = 1.0f;

    size_t at = 0;
    check_true("rising finds the edge, not the peak",
               find_video_trigger(step.data(), step.size(), true, 0.1, 0.1, 0, 19, &at) &&
                   at == 5);
    check_true("falling finds the trailing edge",
               find_video_trigger(step.data(), step.size(), false, 0.1, 0.1, 0, 19, &at) &&
                   at == 12);

    // The arming rule. A series that is already above the level must wait for a
    // real edge; without arming it would report sample 0 and every frame would
    // be "triggered" at its own start, which draws as a perfectly steady trace.
    std::vector<float> high(20, 1.0f);
    for (size_t i = 8; i < 12; ++i) high[i] = 0.01f;
    check_true("a series starting above the level does not fire on its first sample",
               find_video_trigger(high.data(), high.size(), true, 0.1, 0.1, 0, 19, &at) &&
                   at == 12);

    // A floor that never reaches the level yields nothing, rather than the
    // largest sample it happened to see.
    std::vector<float> quiet(20, 0.001f);
    check_true("a quiet series does not trigger",
               !find_video_trigger(quiet.data(), quiet.size(), true, 0.1, 0.1, 0, 19, &at));

    // `first` is the pre-trigger reservation: the edge at 5 cannot be shown
    // with 8 points of history in front of it, so the search skips it and takes
    // the next one at 15.
    std::vector<float> two = step;
    for (size_t i = 15; i < 18; ++i) two[i] = 1.0f;
    check_true("an edge earlier than `first` is skipped for the next one",
               find_video_trigger(two.data(), two.size(), true, 0.1, 0.1, 8, 19, &at) &&
                   at == 15);
    check_true("a search window that ends before the edge finds nothing",
               !find_video_trigger(two.data(), two.size(), true, 0.1, 0.1, 8, 14, &at));

    // A burst that begins before the search window does not get a trigger in
    // the middle of itself. Without the disarm-on-early-crossing rule the frame
    // above would fire at index 8 -- inside the pulse -- and every point of it
    // would be honestly measured and wrongly placed against t=0.
    check_true("no trigger inside a burst that started too early",
               !find_video_trigger(step.data(), step.size(), true, 0.1, 0.1, 8, 11, &at));

    // Hysteresis widens what counts as "low" before an edge, and the trap it
    // brings is that a floor which never gets that low never arms -- so the
    // trigger stops firing entirely and produces no trace at all. That is why
    // no_trigger is counted and shown rather than left to look like a dead
    // radio.
    std::vector<float> floor_high(20, 0.08f);
    for (size_t i = 12; i < 20; ++i) floor_high[i] = 1.0f;
    check_true("with no hysteresis an 0.08 floor arms and the edge fires",
               find_video_trigger(floor_high.data(), floor_high.size(), true, 0.1, 0.1, 0,
                                  19, &at) && at == 12);
    check_true("hysteresis deeper than the floor's excursion never arms",
               !find_video_trigger(floor_high.data(), floor_high.size(), true, 0.1, 0.05,
                                   0, 19, &at));

    // The incremental and batch forms are one implementation; this is what says
    // so, because the engine feeds block by block and the synthetic source
    // feeds a whole array.
    VideoTrigger t(true, 0.1, 0.1, 0, 19);
    size_t inc = 0;
    bool got = false;
    for (size_t i = 0; i < step.size() && !got; ++i)
        if (t.feed(step[i])) { got = true; inc = t.index(); }
    check_true("feed() agrees with find_video_trigger()", got && inc == 5);
    check_true("spent() reports the search is over once it fires", t.spent());
}

}  // namespace

int main() {
    test_window_enbw();
    test_full_scale_is_0_dbfs();
    test_rms_averages_power_not_db();
    test_other_detectors();
    test_output_count_independent_of_fft_size();
    test_rbw_derivation();
    test_overload_counter();
    test_video_trigger();

    std::printf("\n%s (%d failure%s)\n", g_failures ? "FAILED" : "PASSED",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures ? 1 : 0;
}
