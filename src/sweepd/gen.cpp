#include "gen.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace sweepd {

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

const char *gen_mod_name(GenMod m) {
    switch (m) {
        case GenMod::CW: return "cw";
        case GenMod::AM: return "am";
        case GenMod::FM: return "fm";
        case GenMod::ASK: return "ask";
        case GenMod::FSK: return "fsk";
    }
    return "?";
}

bool gen_mod_from_string(const std::string &s, GenMod *out) {
    for (GenMod m : {GenMod::CW, GenMod::AM, GenMod::FM, GenMod::ASK, GenMod::FSK})
        if (s == gen_mod_name(m)) {
            *out = m;
            return true;
        }
    return false;
}

bool derive_gen(const GenConfig &c, double fs_hz, long long lo_min, long long lo_max,
                GenDerived *out, std::string *err) {
    auto fail = [&](const std::string &m) {
        if (err) *err = m;
        return false;
    };
    if (fs_hz <= 0) return fail("sample rate must be positive");
    if (!(c.scale > 0 && c.scale <= 1)) return fail("scale must be in (0, 1]");
    GenDerived d;
    const bool sweeping = c.sweep_step_hz != 0;
    auto in_range = [&](long long f) { return f >= lo_min && f <= lo_max; };
    if (sweeping) {
        if (c.sweep_stop_hz <= c.sweep_start_hz || c.sweep_step_hz < 0)
            return fail("the sweep needs start < stop and a positive step");
        if (!in_range(c.sweep_start_hz) || !in_range(c.sweep_stop_hz))
            return fail("the sweep reaches outside the TX tuner's range");
        if (!(c.sweep_dwell_s >= 0.001)) return fail("dwell must be at least 1 ms");
        // The stop is always a step, even off the step grid: a sweeper that
        // ends short of where it was told to is a sweeper nobody trusts.
        const long long span = c.sweep_stop_hz - c.sweep_start_hz;
        d.sweep_points = size_t((span + c.sweep_step_hz - 1) / c.sweep_step_hz) + 1;
        if (d.sweep_points > 1000000) return fail("more than a million sweep steps");
    } else if (!in_range(c.freq_hz)) {
        char m[160];
        std::snprintf(m, sizeof m, "%.6f MHz is outside the TX tuner's %.3f - %.3f MHz",
                      double(c.freq_hz) / 1e6, double(lo_min) / 1e6, double(lo_max) / 1e6);
        return fail(m);
    }

    if (c.mod == GenMod::CW) {
        // Any length is periodic for a constant; small, but past the DMA's
        // minimum transfer.
        d.samples = 4096;
    } else {
        if (!(c.rate_hz > 0)) return fail("the modulation rate must be positive");
        const double per = fs_hz / c.rate_hz;
        if (per < 8) return fail("the modulation rate is above fs/8");
        // Whole periods, at least 4096 samples: few periods for slow rates,
        // many for fast ones so the buffer is not tiny.
        size_t p = size_t(std::llround(per));
        size_t k = std::max<size_t>(1, (4096 + p - 1) / p);
        if (p > kMaxGenSamples) {
            char m[160];
            std::snprintf(m, sizeof m,
                          "%.1f Hz needs %zu samples per period; the slowest this buffer "
                          "holds is %.1f Hz",
                          c.rate_hz, p, fs_hz / double(kMaxGenSamples));
            return fail(m);
        }
        d.samples = std::min(p * k, kMaxGenSamples / p * p);
        d.rate_hz = fs_hz / double(p);
        if (c.mod == GenMod::AM && !(c.am_depth >= 0 && c.am_depth <= 1))
            return fail("AM depth must be 0..1");
        if ((c.mod == GenMod::FM || c.mod == GenMod::FSK) &&
            !(c.fm_dev_hz > 0 && c.fm_dev_hz < fs_hz / 2.5))
            return fail("deviation must be positive and well inside the sample rate");
    }
    *out = d;
    return true;
}

void gen_waveform(const GenConfig &c, const GenDerived &d, double fs_hz,
                  std::vector<int16_t> *iq) {
    const size_t n = d.samples;
    iq->assign(2 * n, 0);
    const double peak = c.scale * 32767.0;
    const double w_mod = 2.0 * kPi * d.rate_hz / fs_hz;
    double ph = 0;   // FM / FSK carrier phase
    for (size_t i = 0; i < n; ++i) {
        const double t = w_mod * double(i);
        double a = 1.0, re = 0, im = 0;
        switch (c.mod) {
            case GenMod::CW: break;
            case GenMod::AM:
                // Peak (1 + depth) at full scale: the envelope is what fills the DAC.
                a = (1.0 + c.am_depth * std::sin(t)) / (1.0 + c.am_depth);
                break;
            case GenMod::ASK:
                a = std::sin(t) >= 0 ? 1.0 : 0.0;
                break;
            case GenMod::FM:
                ph += 2.0 * kPi * c.fm_dev_hz * std::sin(t) / fs_hz;
                break;
            case GenMod::FSK:
                ph += 2.0 * kPi * c.fm_dev_hz * (std::sin(t) >= 0 ? 1.0 : -1.0) / fs_hz;
                break;
        }
        re = a * std::cos(ph);
        im = a * std::sin(ph);
        (*iq)[2 * i] = int16_t(std::lround(re * peak));
        (*iq)[2 * i + 1] = int16_t(std::lround(im * peak));
    }
}

long long gen_sweep_freq(const GenConfig &c, size_t k) {
    return std::min(c.sweep_stop_hz, c.sweep_start_hz + (long long)k * c.sweep_step_hz);
}

}  // namespace sweepd
