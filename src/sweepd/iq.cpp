#include "iq.h"

#include <algorithm>
#include <cmath>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define SWEEPSA_IQ_NEON 1
#endif

namespace sweepd {

// ---------------------------------------------------------------- names

const char *iq_trig_mode_name(IqTrigMode m) {
    switch (m) {
        case IqTrigMode::None: return "none";
        case IqTrigMode::Normal: return "normal";
        case IqTrigMode::Auto: return "auto";
    }
    return "none";
}

bool iq_trig_mode_from_string(const std::string &s, IqTrigMode *out) {
    if (s == "none" || s == "free" || s == "free_run") *out = IqTrigMode::None;
    else if (s == "normal") *out = IqTrigMode::Normal;
    else if (s == "auto") *out = IqTrigMode::Auto;
    else return false;
    return true;
}

const char *iq_trig_source_name(IqTrigSource s) {
    switch (s) {
        case IqTrigSource::I: return "i";
        case IqTrigSource::Q: return "q";
        case IqTrigSource::Mag: return "mag";
    }
    return "i";
}

bool iq_trig_source_from_string(const std::string &s, IqTrigSource *out) {
    if (s == "i" || s == "I") *out = IqTrigSource::I;
    else if (s == "q" || s == "Q") *out = IqTrigSource::Q;
    else if (s == "mag" || s == "abs" || s == "|x|") *out = IqTrigSource::Mag;
    else return false;
    return true;
}

bool iq_decim_valid(unsigned d) { return d >= 1 && d <= 64 && (d & (d - 1)) == 0; }

// ---------------------------------------------------------------- trigger

bool find_iq_trigger(const float *iq, size_t n, const IqTrigger &t, size_t from, size_t to,
                     size_t *at, double *frac) {
    if (to > n) to = n;
    if (from < 1) from = 1;   // a crossing needs the sample before it
    if (from >= to) return false;
    const double level = t.level;
    const double hyst = std::max(0.0, t.hyst);
    bool armed = false;
    double prev = 0;
    for (size_t i = 0; i < to; ++i) {
        const double v = iq_trig_value(iq, i, t.source);
        if (t.rising) {
            if (v < level - hyst) armed = true;
            else if (armed && v >= level) {
                if (i >= from) {
                    *at = i;
                    const double d = v - prev;
                    *frac = d > 0 ? std::min(1.0, std::max(0.0, (v - level) / d)) : 0.0;
                    return true;
                }
                // Crossed before the window opened: that edge cannot be placed
                // with its pre-trigger history inside the burst. Disarm and
                // wait for the next one.
                armed = false;
            }
        } else {
            if (v > level + hyst) armed = true;
            else if (armed && v <= level) {
                if (i >= from) {
                    *at = i;
                    const double d = prev - v;
                    *frac = d > 0 ? std::min(1.0, std::max(0.0, (level - v) / d)) : 0.0;
                    return true;
                }
                armed = false;
            }
        }
        prev = v;
    }
    return false;
}

// ---------------------------------------------------------------- decimator

namespace {

// Zeroth-order modified Bessel function of the first kind, by its power
// series. Converges fast for the beta a Kaiser window uses (< 10).
double bessel_i0(double x) {
    double sum = 1.0, term = 1.0;
    const double q = x * x * 0.25;
    for (int k = 1; k < 64; ++k) {
        term *= q / (double(k) * double(k));
        sum += term;
        if (term < sum * 1e-17) break;
    }
    return sum;
}

// Stopband the decimator is designed to: anything that would alias into the
// kept band is at least this far down. 70 dB clears the 12-bit ADC's ~74 dB of
// quantisation range by enough that an alias is not the largest thing on screen
// next to a real full-scale signal, without the filter growing past a few
// thousand taps at D = 64.
constexpr double kStopDb = 70.0;

}  // namespace

IqDecimator::IqDecimator(unsigned decim) : decim_(iq_decim_valid(decim) ? decim : 1) {
    if (decim_ == 1) {
        h_.assign(1, 1.0f);
        return;
    }
    // Kaiser's estimate for the length: passband to kPassFraction x fs_out,
    // stopband from (1 - kPassFraction) x fs_out, both in cycles per input
    // sample once divided by D.
    const double D = double(decim_);
    const double tw = (1.0 - 2.0 * kPassFraction) / D;   // transition width, cyc/sample
    size_t n = size_t(std::ceil((kStopDb - 7.95) / (2.285 * 2.0 * M_PI * tw))) + 1;
    if ((n & 1) == 0) ++n;   // odd: a centre tap, so the delay is a whole sample
    const double beta = 0.1102 * (kStopDb - 8.7);
    const double fc = 0.5 / D;                           // cutoff, mid-transition
    const double m = double(n - 1) * 0.5;
    const double i0b = bessel_i0(beta);

    h_.resize(n);
    double sum = 0;
    for (size_t k = 0; k < n; ++k) {
        const double x = double(k) - m;
        const double sinc = x == 0 ? 2.0 * fc : std::sin(2.0 * M_PI * fc * x) / (M_PI * x);
        const double r = x / m;
        const double w = bessel_i0(beta * std::sqrt(std::max(0.0, 1.0 - r * r))) / i0b;
        const double v = sinc * w;
        h_[k] = float(v);
        sum += v;
    }
    // Unity gain at DC, exactly: an FS reading through the decimator must mean
    // the same as one without it.
    for (float &v : h_) v = float(double(v) / sum);
}

void IqDecimator::run(const float *in, size_t n_in, std::vector<float> *out) const {
    const size_t n_out = output_for(n_in);
    out->resize(2 * n_out);
    if (decim_ == 1) {
        std::copy(in, in + 2 * n_out, out->begin());
        return;
    }
    const size_t nt = h_.size();
    const float *h = h_.data();
    float *o = out->data();
    for (size_t k = 0; k < n_out; ++k) {
        const float *x = in + 2 * (k * decim_);
        float ai = 0, aq = 0;
        size_t j = 0;
#ifdef SWEEPSA_IQ_NEON
        // Four taps at a time, I and Q de-interleaved by the load. Written out
        // because the compiler will not vectorise a float sum onto NEON without
        // fast-math, and scalar this loop was 217 ms per decimation-16 burst
        // on a Cortex-A9 against 8.6 ms of capture (sweepd on a PlutoSDR,
        // 2026-11-06). The summation order differs from the scalar loop by
        // rounding only; the decimator tests run this path on an arm64 host.
        float32x4_t vi = vdupq_n_f32(0.0f), vq = vdupq_n_f32(0.0f);
        for (; j + 4 <= nt; j += 4) {
            const float32x4x2_t v = vld2q_f32(x + 2 * j);
            const float32x4_t hv = vld1q_f32(h + j);
            vi = vmlaq_f32(vi, hv, v.val[0]);
            vq = vmlaq_f32(vq, hv, v.val[1]);
        }
        ai = vgetq_lane_f32(vi, 0) + vgetq_lane_f32(vi, 1) + vgetq_lane_f32(vi, 2) +
             vgetq_lane_f32(vi, 3);
        aq = vgetq_lane_f32(vq, 0) + vgetq_lane_f32(vq, 1) + vgetq_lane_f32(vq, 2) +
             vgetq_lane_f32(vq, 3);
#endif
        for (; j < nt; ++j) {
            ai += h[j] * x[2 * j];
            aq += h[j] * x[2 * j + 1];
        }
        o[2 * k] = ai;
        o[2 * k + 1] = aq;
    }
}

// ---------------------------------------------------------------- geometry

bool derive_iq(const IqConfig &c, size_t max_input, IqGeometry *g, std::string *err) {
    auto fail = [&](const std::string &m) {
        if (err) *err = m;
        return false;
    };
    if (!iq_decim_valid(c.decim))
        return fail("decimation must be 1, 2, 4, 8, 16, 32 or 64");
    if (c.n_samples < 16) return fail("a burst needs at least 16 samples");
    if (c.n_samples > kIqMaxSamples)
        return fail("at most " + std::to_string(kIqMaxSamples) +
                    " samples per burst: a larger one does not fit in one 0x03 message");
    if (!(c.trig.level == c.trig.level) || std::fabs(c.trig.level) > 2.0)
        return fail("trigger level is in full-scale units and must be within +/-2");
    if (!(c.trig.hyst >= 0.0) || c.trig.hyst > 1.0)
        return fail("trigger hysteresis is a depth in full-scale units, 0..1");
    if (!(c.trig.pretrig >= 0.0) || c.trig.pretrig > 0.95)
        return fail("pre-trigger is the fraction of the burst before the edge, 0..0.95");
    if (!(c.auto_s >= 0.001) || c.auto_s > 10.0)
        return fail("auto timeout must be between 1 ms and 10 s");

    const IqDecimator dec(c.decim);
    const size_t max_out = dec.output_for(max_input);
    if (c.n_samples > max_out)
        return fail("at decimation " + std::to_string(c.decim) + " the device buffer holds " +
                    std::to_string(max_out) + " samples; asked for " +
                    std::to_string(c.n_samples));

    IqGeometry r;
    if (c.trig.mode != IqTrigMode::None) {
        r.pre = std::min(c.n_samples - 1,
                         size_t(std::llround(c.trig.pretrig * double(c.n_samples))));
        // As much search as there is buffer for, up to one burst's worth: more
        // finds a rare edge sooner, but every acquisition costs the whole
        // buffer's capture time whether or not the edge is early.
        r.search = std::min(c.n_samples, max_out - c.n_samples);
        if (r.search < 1)
            return fail("a triggered burst needs buffer left over to search for the edge; "
                        "at decimation " + std::to_string(c.decim) + " use at most " +
                        std::to_string(max_out / 2) + " samples");
    }
    r.acq_out = c.n_samples + r.search;
    r.acq_in = dec.input_for(r.acq_out);
    r.buf_in = (r.acq_in + kIqBufAlign - 1) / kIqBufAlign * kIqBufAlign;
    *g = r;
    return true;
}

bool iq_cut(const float *acq, size_t acq_n, const IqConfig &c, const IqGeometry &g,
            double fs_hz, bool force, std::vector<float> *out, int64_t *t0_ns,
            uint32_t *trig_index) {
    const size_t n = c.n_samples;
    if (acq_n < n) return false;
    if (c.trig.mode == IqTrigMode::None || force) {
        out->assign(acq, acq + 2 * n);
        *t0_ns = 0;
        *trig_index = 0;
        return true;
    }
    size_t at = 0;
    double frac = 0;
    // The window is limited so the whole burst after the edge is inside the
    // acquisition: an edge at `at` needs n - pre samples from `at - pre` on.
    const size_t to = std::min(g.pre + g.search, acq_n - n + g.pre + 1);
    if (!find_iq_trigger(acq, acq_n, c.trig, g.pre, to, &at, &frac)) return false;
    const size_t start = at - g.pre;
    out->assign(acq + 2 * start, acq + 2 * (start + n));
    // Sample 0 is `pre` samples before `at`, and the edge itself is `frac` of a
    // sample before `at`.
    *t0_ns = (int64_t)std::llround((frac - double(g.pre)) / fs_hz * 1e9);
    *trig_index = uint32_t(g.pre);
    return true;
}

// ---------------------------------------------------------------- sc16

size_t iq_sc16_to_float(const int16_t *in, size_t n, double full_scale, double clip_frac,
                        std::vector<float> *out) {
    out->resize(2 * n);
    const float inv = float(1.0 / full_scale);
    const int lim = int(std::lround(full_scale * clip_frac));
    size_t clipped = 0;
    for (size_t i = 0; i < n; ++i) {
        const int a = in[2 * i], b = in[2 * i + 1];
        (*out)[2 * i] = float(a) * inv;
        (*out)[2 * i + 1] = float(b) * inv;
        if (std::abs(a) >= lim || std::abs(b) >= lim) ++clipped;
    }
    return clipped;
}

// ---------------------------------------------------------------- statistics

IqStats iq_stats(const float *iq, size_t i0, size_t i1, double fs_hz) {
    IqStats s;
    if (i1 <= i0) return s;
    const size_t n = i1 - i0;
    s.n = n;

    double si = 0, sq = 0;
    s.min_i = s.max_i = iq[2 * i0];
    s.min_q = s.max_q = iq[2 * i0 + 1];
    for (size_t k = i0; k < i1; ++k) {
        const double a = iq[2 * k], b = iq[2 * k + 1];
        si += a;
        sq += b;
        s.min_i = std::min(s.min_i, a);
        s.max_i = std::max(s.max_i, a);
        s.min_q = std::min(s.min_q, b);
        s.max_q = std::max(s.max_q, b);
    }
    s.dc_i = si / double(n);
    s.dc_q = sq / double(n);

    // Second pass on the DC-removed signal: the mismatch and the tone estimate
    // both have to be blind to the LO leakage, which sits exactly at DC and
    // would otherwise read as a huge imbalance and pull the tone toward 0 Hz.
    double p_all = 0, pi = 0, pq = 0, piq = 0, peak_ac2 = 0;
    double pr = 0, pim = 0, pnorm = 0;
    double prev_i = 0, prev_q = 0;
    for (size_t k = i0; k < i1; ++k) {
        const double a = iq[2 * k], b = iq[2 * k + 1];
        const double m2 = a * a + b * b;
        p_all += m2;
        s.peak_mag = std::max(s.peak_mag, std::sqrt(m2));
        const double x = a - s.dc_i, y = b - s.dc_q;
        pi += x * x;
        pq += y * y;
        piq += x * y;
        peak_ac2 = std::max(peak_ac2, x * x + y * y);
        if (k > i0) {
            // x[n] * conj(x[n-1])
            pr += x * prev_i + y * prev_q;
            pim += y * prev_i - x * prev_q;
            pnorm += std::sqrt((x * x + y * y) * (prev_i * prev_i + prev_q * prev_q));
        }
        prev_i = x;
        prev_q = y;
    }
    const double nd = double(n);
    s.rms = std::sqrt(p_all / nd);
    s.rms_ac = std::sqrt((pi + pq) / nd);
    s.power_dbfs = s.rms > 0 ? 20.0 * std::log10(s.rms) : -300.0;
    s.crest_db = s.rms_ac > 0 ? 20.0 * std::log10(std::sqrt(peak_ac2) / s.rms_ac) : 0.0;

    if (pnorm > 0) {
        s.tone_hz = std::atan2(pim, pr) * fs_hz / (2.0 * M_PI);
        s.tone_coherence = std::sqrt(pr * pr + pim * pim) / pnorm;
    }
    s.tone_cycles = std::fabs(s.tone_hz) * nd / fs_hz;

    // 1e-4 FS is ~-80 dBFS, the floor of a 12-bit converter: below it the
    // ratios are quantisation noise divided by quantisation noise. And a tone
    // has to turn over many times in the window: over a fraction of a cycle
    // E[I^2] and E[Q^2] differ because of where the cycle was cut, not because
    // of the receiver, and the "mismatch" is the window's.
    s.too_few_cycles = s.tone_coherence > 0.2 && s.tone_cycles < 10.0;
    s.circular_ok = n >= 256 && s.rms_ac > 1e-4 && pi > 0 && pq > 0 && !s.too_few_cycles;
    if (s.circular_ok) {
        const double g = std::sqrt(pi / pq);
        const double sphi = std::max(-1.0, std::min(1.0, piq / std::sqrt(pi * pq)));
        const double phi = std::asin(sphi);
        s.gain_imb_db = 20.0 * std::log10(g);
        s.phase_imb_deg = phi * 180.0 / M_PI;
        const double c = 2.0 * g * std::cos(phi);
        const double num = 1.0 + c + g * g, den = 1.0 - c + g * g;
        s.irr_db = den > 0 ? 10.0 * std::log10(num / den) : 300.0;
    }
    return s;
}

}  // namespace sweepd
