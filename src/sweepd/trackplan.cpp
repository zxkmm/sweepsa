#include "trackplan.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace sweepd {

namespace {
// Not M_PI: it is not standard C++, and the Windows build of sweepui links this.
constexpr double kPi = 3.14159265358979323846;
}  // namespace

// ---------------------------------------------------------------- derive

bool derive_track(const TrackConfig &cfg, double fs_hz, long long bw_hz, long long lo_min_hz,
                  long long lo_max_hz, TrackDerived *out, std::string *err) {
    auto fail = [&](const std::string &m) {
        if (err) *err = m;
        return false;
    };
    if (fs_hz <= 0) return fail("sample rate must be positive");
    if (cfg.n_fft < 64 || (cfg.n_fft & (cfg.n_fft - 1)) != 0)
        return fail("n_fft must be a power of two and at least 64");
    if (cfg.coherent < 1 || cfg.coherent > 256)
        return fail("coherent must be 1..256 blocks");
    if (cfg.f_stop_hz <= cfg.f_start_hz) return fail("f_stop must be above f_start");
    if (cfg.spacing_hz <= 0) return fail("spacing must be positive");
    if (!(cfg.scale > 0.0 && cfg.scale <= 1.0)) return fail("scale must be in (0, 1]");
    if (cfg.overload_frac <= 0.0 || cfg.overload_frac > 1.0)
        return fail("overload_frac must be in (0, 1]");

    TrackDerived d;
    d.fs_hz = fs_hz;
    d.n_fft = cfg.n_fft;
    d.coherent = cfg.coherent;
    d.bin_hz = fs_hz / double(cfg.n_fft);
    d.q = std::max(1, int(std::lround(double(cfg.spacing_hz) / (4.0 * d.bin_hz))));
    d.spacing_hz = 4.0 * d.q * d.bin_hz;

    // The usable width. Bounded by the sampled band as well as the analog one,
    // since a radio run below its bandwidth aliases the rest.
    const long long usable = std::min<long long>(bw_hz > 0 ? bw_hz : (long long)fs_hz,
                                                 (long long)fs_hz);
    d.keep_hz = cfg.keep_hz > 0 ? cfg.keep_hz : (long long)std::llround(0.72 * double(usable));
    if (d.keep_hz > usable)
        return fail("keep " + std::to_string(d.keep_hz / 1000) + " kHz is wider than the " +
                    std::to_string(usable / 1000) + " kHz the radio samples");

    if (cfg.harmonic < 1 || cfg.harmonic > 9) return fail("harmonic must be 1..9");
    d.harmonic = cfg.harmonic;
    d.rx_offset_hz = cfg.rx_offset_hz;
    d.rx_invert = cfg.rx_invert;
    // At the nth harmonic the tones arrive n times further apart, so the kept
    // width holds n times fewer.
    const size_t fit =
        size_t(std::floor(double(d.keep_hz) / (d.spacing_hz * double(cfg.harmonic))));
    if (fit < 1) {
        char m[200];
        std::snprintf(m, sizeof m,
                      "a spacing of %.1f kHz does not fit even one tone in the %.1f MHz "
                      "kept per step",
                      d.spacing_hz / 1e3, d.keep_hz / 1e6);
        return fail(m);
    }
    if (cfg.tones > fit) {
        char m[200];
        std::snprintf(m, sizeof m,
                      "%zu tones at %.1f kHz spacing need %.1f MHz per step; only %.1f MHz "
                      "is kept, which holds %zu",
                      cfg.tones, d.spacing_hz / 1e3, cfg.tones * d.spacing_hz / 1e6,
                      d.keep_hz / 1e6, fit);
        return fail(m);
    }
    const size_t M = cfg.tones ? cfg.tones : fit;
    d.tones_per_step = M;
    const int half = int(M / 2);
    for (size_t m = 0; m < M; ++m) d.tone_bins.push_back((int(m) - half) * 4 * d.q + d.q);
    // Every tone must be a real bin of the transform, inside Nyquist, where
    // the receiver sees it.
    if (std::abs(d.harmonic * d.tone_bins.front()) >= int(cfg.n_fft / 2) ||
        std::abs(d.harmonic * d.tone_bins.back()) >= int(cfg.n_fft / 2))
        return fail("the tones reach past Nyquist; lower the spacing or the tone count");

    const double span = double(cfg.f_stop_hz - cfg.f_start_hz);
    const double np = std::floor(span / d.spacing_hz);
    if (np < 1) return fail("the span is narrower than one point spacing");
    if (np > double(cfg.max_points)) {
        char m[200];
        std::snprintf(m, sizeof m,
                      "%.0f points at %.1f kHz spacing; the limit is %zu. Widen the "
                      "spacing or narrow the span.",
                      np, d.spacing_hz / 1e3, cfg.max_points);
        return fail(m);
    }
    d.n_points = size_t(np);
    d.out_lo_hz = double(cfg.f_start_hz);
    d.out_hi_hz = d.out_lo_hz + double(d.n_points) * d.spacing_hz;

    // Point i is at F0 + i*spacing. With tone m at ((m - half) + 1/4)*spacing
    // from its LO, a step that measures points p0.. with tone t0 for p0 sits at
    // F0 + (p0 - t0 + half - 1/4) * spacing.
    const double f0 = d.out_lo_hz + 0.5 * d.spacing_hz;
    auto lo_for = [&](size_t p0, int t0) {
        return (long long)std::llround(f0 + (double(p0) - t0 + half - 0.25) * d.spacing_hz);
    };
    for (size_t p0 = 0; p0 < d.n_points; p0 += M) {
        TrackStep s;
        s.p0 = p0;
        s.cnt = std::min(M, d.n_points - p0);
        s.t0 = 0;
        s.lo_hz = lo_for(p0, 0);
        // A short last step may slide its points up the tone set, which moves
        // its LO down, to stay under the synthesizer's ceiling.
        while (s.lo_hz > lo_max_hz && s.t0 + int(s.cnt) < int(M)) {
            ++s.t0;
            s.lo_hz = lo_for(p0, s.t0);
        }
        if (s.lo_hz > lo_max_hz || s.lo_hz < lo_min_hz) {
            char m[300];
            std::snprintf(m, sizeof m,
                          "the step measuring %.3f MHz needs the LO at %.3f MHz, outside "
                          "the tuner's %.3f - %.3f MHz. Tracking reaches about %.3f MHz "
                          "below the lowest LO and %.3f MHz above the highest.",
                          (f0 + double(p0) * d.spacing_hz) / 1e6, s.lo_hz / 1e6,
                          lo_min_hz / 1e6, lo_max_hz / 1e6,
                          (half - 0.25) * d.spacing_hz / 1e6,
                          (double(M) - 1 - half + 0.25) * d.spacing_hz / 1e6);
            return fail(m);
        }
        const long long nlo = (long long)d.harmonic * s.lo_hz;
        s.rx_lo_hz = d.rx_invert ? d.rx_offset_hz - nlo : nlo + d.rx_offset_hz;
        if (s.rx_lo_hz > lo_max_hz || s.rx_lo_hz < lo_min_hz) {
            char m[300];
            std::snprintf(m, sizeof m,
                          "the stimulus at %.3f MHz puts the receiver at %.3f MHz (harmonic %d, "
                          "offset %+.3f MHz%s), outside the tuner's %.3f - %.3f MHz",
                          s.lo_hz / 1e6, s.rx_lo_hz / 1e6, d.harmonic, d.rx_offset_hz / 1e6,
                          d.rx_invert ? ", inverted" : "", lo_min_hz / 1e6, lo_max_hz / 1e6);
            return fail(m);
        }
        d.steps.push_back(s);
    }

    const WindowFunction w = make_window(cfg.window, cfg.n_fft);
    d.enbw = w.enbw;
    d.rbw_hz = w.enbw * d.bin_hz / double(d.coherent);
    d.settle_samples = size_t(std::llround(cfg.settle_us * 1e-6 * fs_hz));
    const size_t need = d.settle_samples + d.coherent * d.n_fft;
    const size_t floor_by_time = size_t(std::llround(cfg.min_buffer_us * 1e-6 * fs_hz));
    d.buf_samples = ((std::max(need, floor_by_time) + 1023) / 1024) * 1024;
    d.buffer_us = 1e6 * double(d.buf_samples) / fs_hz;

    *out = std::move(d);
    return true;
}

void track_stimulus(const TrackDerived &d, double scale, std::vector<int16_t> *iq,
                    double *crest_db, double *tone_dbfs) {
    const size_t L = d.n_fft;
    const size_t M = d.tone_bins.size();
    std::vector<double> re(L, 0.0), im(L, 0.0);
    for (size_t m = 0; m < M; ++m) {
        const double ph = kPi * double(m) * double(m) / double(M);
        const double w = 2.0 * kPi * double(d.tone_bins[m]) / double(L);
        for (size_t n = 0; n < L; ++n) {
            const double a = w * double(n) + ph;
            re[n] += std::cos(a);
            im[n] += std::sin(a);
        }
    }
    double peak = 0, pow = 0;
    for (size_t n = 0; n < L; ++n) {
        const double p = re[n] * re[n] + im[n] * im[n];
        peak = std::max(peak, std::sqrt(p));
        pow += p;
    }
    const double rms = std::sqrt(pow / double(L));
    const double k = scale * 32767.0 / peak;
    iq->resize(2 * L);
    for (size_t n = 0; n < L; ++n) {
        (*iq)[2 * n] = int16_t(std::lround(re[n] * k));
        (*iq)[2 * n + 1] = int16_t(std::lround(im[n] * k));
    }
    if (crest_db) *crest_db = 20.0 * std::log10(peak / rms);
    // Each tone has unit amplitude before scaling, so after it each is
    // scale/peak of full scale.
    if (tone_dbfs) *tone_dbfs = 20.0 * std::log10(scale / peak);
}

void track_extract(const TrackDerived &d, size_t s, const float *power, float *points) {
    const TrackStep &st = d.steps[s];
    const int c = int(d.n_fft / 2);
    for (size_t i = 0; i < st.cnt; ++i)
        points[st.p0 + i] = power[c + d.rx_bin(size_t(st.t0) + i)];
}

}  // namespace sweepd
