// Tests for raw IQ: the 0x03 wire layout, the decimator, the edge trigger, the
// burst geometry and the blind statistics (src/sweepd/iq.h, proto.h "0x03 iq").
//
// What is pinned is what would otherwise fail plausibly: a header offset that
// moved, a decimator whose gain is not 1 (every FS reading off by the same
// amount), a stopband that leaks, a trigger that jitters by a sample or fires
// on noise, and an imbalance estimate that reports the LO leakage as a mismatch.

#include <cmath>
#include <complex>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "iq.h"
#include "proto.h"

using namespace sweepd;
using namespace sweepd::proto;

static int g_fail = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);      \
            ++g_fail;                                                          \
        }                                                                      \
    } while (0)

#define NEAR(a, b, tol) CHECK(std::fabs(double(a) - double(b)) <= (tol))

static std::vector<float> tone(size_t n, double f_norm, double amp, double phase = 0) {
    std::vector<float> v(2 * n);
    for (size_t i = 0; i < n; ++i) {
        const double p = 2 * M_PI * f_norm * double(i) + phase;
        v[2 * i] = float(amp * std::cos(p));
        v[2 * i + 1] = float(amp * std::sin(p));
    }
    return v;
}

static double rms_of(const std::vector<float> &v, size_t skip = 0) {
    double s = 0;
    const size_t n = v.size() / 2;
    for (size_t i = skip; i < n; ++i) s += double(v[2 * i]) * v[2 * i] + double(v[2 * i + 1]) * v[2 * i + 1];
    return std::sqrt(s / double(n - skip));
}

static void test_wire_layout() {
    std::printf("wire layout\n");
    IqMsg m;
    m.seq = 0x0102030405060708ULL;
    m.t_mono_s = 12.5;
    m.center_hz = 433895000LL;
    m.adc_fs_hz = 61.44e6;
    m.decim = 4;
    m.fs_hz = 61.44e6 / 4;
    m.rf_bw_hz = 56e6;
    m.t0_ns = -1234;
    m.capture_s = 0.002;
    m.gain_db = 30;
    m.n_clipped = 7;
    m.n_acquisitions = 3;
    m.format = IqFormat::Cf32;
    m.trig_mode = 1;
    m.trig_state = IqTrigState::Triggered;
    m.trig_source = 2;
    m.trig_rising = false;
    m.trig_level = 0.25f;
    m.trig_hyst = 0.01f;
    m.trig_index = 5;
    m.iq = {0.5f, -0.25f, 1.0f, 0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0, 0, 0, 0};

    std::vector<uint8_t> b;
    encode_iq(m, &b);
    CHECK(b.size() == kHeaderBytes + kIqHeadBytes + 6 * 8);
    CHECK(b[4] == 0x03);
    const uint8_t *p = b.data() + kHeaderBytes;
    uint32_t ns;
    std::memcpy(&ns, p + 72, 4);
    CHECK(ns == 6);
    CHECK(p[88] == 1 && p[89] == 1 && p[90] == 1 && p[91] == 2 && p[92] == 0);
    int64_t t0;
    std::memcpy(&t0, p + 48, 8);
    CHECK(t0 == -1234);

    IqMsg d;
    std::string err;
    CHECK(decode_iq(p, b.size() - kHeaderBytes, &d, &err));
    CHECK(d.seq == m.seq && d.center_hz == m.center_hz && d.decim == 4);
    CHECK(d.trig_state == IqTrigState::Triggered && d.trig_index == 5 && !d.trig_rising);
    CHECK(d.iq == m.iq);
    NEAR(d.t_at(0), -1234e-9, 1e-15);

    // Short by one byte: refused, not truncated.
    CHECK(!decode_iq(p, b.size() - kHeaderBytes - 1, &d, &err));
    // A trigger index outside the burst is refused.
    std::vector<uint8_t> bad = b;
    bad[kHeaderBytes + 104] = 9;
    CHECK(!decode_iq(bad.data() + kHeaderBytes, bad.size() - kHeaderBytes, &d, &err));
    // fs inconsistent with adc_fs / decim is refused.
    IqMsg m2 = m;
    m2.fs_hz = 1e6;
    b.clear();
    encode_iq(m2, &b);
    CHECK(!decode_iq(b.data() + kHeaderBytes, b.size() - kHeaderBytes, &d, &err));

    // sc16: exact for values on the 1/2048 grid, clamped (not wrapped) past it.
    IqMsg s = m;
    s.decim = 1;
    s.fs_hz = s.adc_fs_hz;
    s.format = IqFormat::Sc16;
    s.full_scale = 2048;
    s.trig_index = 0;
    s.iq = {1024 / 2048.0f, -2048 / 2048.0f, 3 / 2048.0f, 20.0f};
    b.clear();
    encode_iq(s, &b);
    CHECK(b.size() == kHeaderBytes + kIqHeadBytes + 2 * 4);
    CHECK(decode_iq(b.data() + kHeaderBytes, b.size() - kHeaderBytes, &d, &err));
    CHECK(d.iq[0] == 0.5f && d.iq[1] == -1.0f && d.iq[2] == 3 / 2048.0f);
    CHECK(d.iq[3] > 15.0f);   // 20 FS clamps at int16 max, stays positive
}

static void test_decimator() {
    std::printf("decimator\n");
    for (unsigned D : {1u, 2u, 4u, 16u, 64u}) {
        IqDecimator dec(D);
        const size_t n_out = 512;
        const size_t n_in = dec.input_for(n_out);
        CHECK(dec.output_for(n_in) == n_out);
        CHECK(dec.output_for(n_in - 1) == n_out - 1 || D == 1);

        // Passband: a tone at 0.3 x fs_out passes at unity (within 0.1 dB).
        std::vector<float> out;
        const auto in = tone(n_in, 0.3 / D, 0.5);
        dec.run(in.data(), n_in, &out);
        CHECK(out.size() == 2 * n_out);
        NEAR(20 * std::log10(rms_of(out) / 0.5), 0.0, 0.1);

        if (D > 1) {
            // Stopband: a tone that would alias onto 0.2 x fs_out sits at
            // 0.8 x fs_out on the input side; it must come out 65+ dB down.
            const auto alias = tone(n_in, 0.8 / D, 0.5);
            dec.run(alias.data(), n_in, &out);
            CHECK(20 * std::log10(rms_of(out) / 0.5) < -65.0);
            // DC gain is exactly 1.
            std::vector<float> dc(2 * n_in, 0.25f);
            dec.run(dc.data(), n_in, &out);
            NEAR(out[0], 0.25, 1e-5);
            NEAR(out[2 * (n_out - 1) + 1], 0.25, 1e-5);
        }
    }
    CHECK(!iq_decim_valid(3) && !iq_decim_valid(128) && iq_decim_valid(32));
}

static void test_trigger() {
    std::printf("trigger\n");
    // A slow sine on I: rising crossings of 0 at multiples of the period.
    const size_t n = 1000;
    const double per = 100.0;
    auto v = tone(n, 1.0 / per, 1.0, -M_PI / 2 + 0.013);   // I = sin(.)-ish, offset start
    IqTrigger t;
    t.mode = IqTrigMode::Normal;
    t.source = IqTrigSource::Q;   // Q = sin(2 pi i/per - pi/2 + phi) = -cos(...)
    t.level = 0.0;
    t.hyst = 0.05;
    size_t at = 0;
    double frac = 0;
    CHECK(find_iq_trigger(v.data(), n, t, 10, n, &at, &frac));
    // Interpolated edge (at - frac) is where Q crosses 0 upward.
    const double edge = double(at) - frac;
    const double q = std::sin(2 * M_PI * edge / per - M_PI / 2 + 0.013);
    NEAR(q, 0.0, 2e-3);
    CHECK(frac >= 0 && frac <= 1);

    // Falling, on |x| of a burst: no edge without ever being above the level.
    std::vector<float> b(2 * n, 0.0f);
    for (size_t i = 300; i < 600; ++i) b[2 * i] = 0.8f;
    IqTrigger m;
    m.mode = IqTrigMode::Normal;
    m.source = IqTrigSource::Mag;
    m.rising = false;
    m.level = 0.4;
    m.hyst = 0.1;
    CHECK(find_iq_trigger(b.data(), n, m, 1, n, &at, &frac));
    CHECK(at == 600);
    // The edge before the window is refused rather than reported late.
    m.rising = true;
    CHECK(!find_iq_trigger(b.data(), n, m, 400, n, &at, &frac));
    CHECK(find_iq_trigger(b.data(), n, m, 300, n, &at, &frac) && at == 300);

    // Noise inside the hysteresis band does not fire twice.
    std::vector<float> c(2 * n, 0.0f);
    for (size_t i = 0; i < n; ++i) c[2 * i] = (i < 500 ? -0.5f : 0.5f) + ((i & 1) ? 0.02f : -0.02f);
    IqTrigger h;
    h.mode = IqTrigMode::Normal;
    h.level = 0.5;
    h.hyst = 0.1;
    CHECK(find_iq_trigger(c.data(), n, h, 1, n, &at, &frac) && at >= 500);
    CHECK(!find_iq_trigger(c.data(), n, h, at + 1, n, &at, &frac));
}

static void test_geometry_and_cut() {
    std::printf("geometry\n");
    IqConfig c;
    c.n_samples = 1000;
    IqGeometry g;
    std::string err;
    CHECK(derive_iq(c, 1 << 20, &g, &err));
    CHECK(g.search == 0 && g.pre == 0 && g.acq_out == 1000 && g.acq_in == 1000);

    c.trig.mode = IqTrigMode::Normal;
    c.trig.pretrig = 0.25;
    CHECK(derive_iq(c, 1 << 20, &g, &err));
    CHECK(g.pre == 250 && g.search == 1000 && g.acq_out == 2000);

    // Buffer too small for a search: refused, and the message names the fit.
    CHECK(!derive_iq(c, 1000, &g, &err));
    CHECK(err.find("500") != std::string::npos);
    c.decim = 3;
    CHECK(!derive_iq(c, 1 << 20, &g, &err));
    c.decim = 8;
    CHECK(derive_iq(c, 1 << 20, &g, &err));
    CHECK(g.acq_in == IqDecimator(8).input_for(2000));
    // The DMA buffer is the acquisition rounded up to whole 8-sample units.
    CHECK(g.buf_in % kIqBufAlign == 0 && g.buf_in >= g.acq_in && g.buf_in - g.acq_in < 8);
    c.n_samples = kIqMaxSamples + 1;
    CHECK(!derive_iq(c, size_t(1) << 30, &g, &err));

    // Cutting: pre = 20, search = 100, so an edge may land in [20, 120). One
    // at 100 puts the burst at [80, 180) with the edge at its sample 20; one
    // at 150 is outside the search and is not taken.
    IqConfig k;
    k.n_samples = 100;
    k.trig.mode = IqTrigMode::Normal;
    k.trig.level = 0.5;
    k.trig.hyst = 0.1;
    k.trig.pretrig = 0.2;
    IqGeometry kg;
    CHECK(derive_iq(k, 1 << 20, &kg, &err));
    std::vector<float> acq(2 * kg.acq_out, 0.0f);
    std::vector<float> late(2 * kg.acq_out, 0.0f);
    for (size_t i = 150; i < kg.acq_out; ++i) late[2 * i] = 1.0f;
    for (size_t i = 100; i < kg.acq_out; ++i) acq[2 * i] = 1.0f;
    std::vector<float> out;
    int64_t t0 = 0;
    uint32_t ti = 0;
    CHECK(iq_cut(acq.data(), kg.acq_out, k, kg, 1e6, false, &out, &t0, &ti));
    CHECK(out.size() == 200 && ti == 20);
    if (out.size() == 200) CHECK(out[2 * 19] == 0.0f && out[2 * 20] == 1.0f);
    // frac: crossing of 0.5 between 0 and 1 is half a sample before 100, so
    // t0 = (0.5 - 20) us.
    CHECK(t0 == -19500);
    CHECK(!iq_cut(late.data(), kg.acq_out, k, kg, 1e6, false, &out, &t0, &ti));
    // No edge: nothing, unless forced.
    std::vector<float> flat(2 * kg.acq_out, 0.0f);
    CHECK(!iq_cut(flat.data(), kg.acq_out, k, kg, 1e6, false, &out, &t0, &ti));
    CHECK(iq_cut(flat.data(), kg.acq_out, k, kg, 1e6, true, &out, &t0, &ti) && t0 == 0);
}

static void test_stats() {
    std::printf("statistics\n");
    const size_t n = 1 << 15;
    const double fs = 1e6, f = 12345.0;
    // A tone with a known mismatch and DC: I = cos, Q = g*sin(p + phi), plus DC.
    const double g = std::pow(10.0, -0.5 / 20.0), phi = 2.0 * M_PI / 180.0;
    std::vector<float> v(2 * n);
    for (size_t i = 0; i < n; ++i) {
        const double p = 2 * M_PI * f / fs * double(i);
        v[2 * i] = float(0.3 * std::cos(p) + 0.01);
        v[2 * i + 1] = float(0.3 * g * std::sin(p + phi) - 0.02);
    }
    const IqStats s = iq_stats(v.data(), 0, n, fs);
    // 404.5 cycles: the half cycle left over leaks ~0.3/(2 pi 404) into the mean.
    NEAR(s.dc_i, 0.01, 5e-4);
    NEAR(s.dc_q, -0.02, 5e-4);
    CHECK(s.circular_ok);
    NEAR(s.gain_imb_db, 0.5, 0.02);           // I is the bigger arm
    NEAR(std::fabs(s.phase_imb_deg), 2.0, 0.05);
    // The -29 dB image pulls the phase-increment estimate a little (iq.h).
    NEAR(s.tone_hz, f, f * 0.005);
    CHECK(s.tone_coherence > 0.99);
    // IRR from g and phi directly.
    const double c = 2 * (1 / g) * std::cos(phi);
    const double irr = 10 * std::log10((1 + c + 1 / (g * g)) / (1 - c + 1 / (g * g)));
    NEAR(s.irr_db, irr, 0.3);

    // A perfect receiver: no mismatch, IRR huge.
    const auto t = tone(n, f / fs, 0.3);
    const IqStats p = iq_stats(t.data(), 0, n, fs);
    NEAR(p.gain_imb_db, 0.0, 1e-3);
    NEAR(p.phase_imb_deg, 0.0, 1e-3);
    CHECK(p.irr_db > 80);
    NEAR(p.crest_db, 0.0, 0.01);   // a complex tone has a constant envelope
    NEAR(p.tone_hz, f, 0.01);
    NEAR(p.power_dbfs, 20 * std::log10(0.3), 0.01);
    // Negative frequency reads negative.
    const auto tn = tone(n, -f / fs, 0.3);
    NEAR(iq_stats(tn.data(), 0, n, fs).tone_hz, -f, 1.0);

    // A sub-range.
    const IqStats r = iq_stats(v.data(), 100, 200, fs);
    CHECK(r.n == 100);
    // A window holding under ten turns of the tone refuses the mismatch: over
    // part of a cycle the arms differ because of where it was cut.
    const IqStats w = iq_stats(v.data(), 0, 400, fs);   // ~4.9 turns
    CHECK(w.too_few_cycles && !w.circular_ok);
    CHECK(iq_stats(v.data(), 0, 2000, fs).circular_ok);  // ~24.7 turns
}

int main() {
    test_wire_layout();
    test_decimator();
    test_trigger();
    test_geometry_and_cut();
    test_stats();
    if (g_fail) {
        std::printf("%d check(s) failed\n", g_fail);
        return 1;
    }
    std::printf("all iq checks passed\n");
    return 0;
}
