#include "dsp.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>

#include "pffft.h"

namespace sweepd {
namespace {

constexpr double kPi = 3.14159265358979323846;

// Cosine-sum window coefficients. A window of order K is
//   w[n] = sum_{k<K} (-1)^k a[k] cos(2*pi*k*n/(N-1))
// which covers every window §5 asks for except Rectangular.
struct CosineSum {
    int order;
    double a[5];
};

// Hann: standard.
// Blackman-Harris: 4-term minimum, -92 dB sidelobes.
// Flat-top: 5-term, the SRS/HFT-style window §5 wants for amplitude accuracy
//   (~0.01 dB scalloping loss at the cost of a very wide main lobe).
constexpr CosineSum kHann = {2, {0.5, 0.5, 0, 0, 0}};
constexpr CosineSum kBlackmanHarris = {4, {0.35875, 0.48829, 0.14128, 0.01168, 0}};
constexpr CosineSum kFlatTop = {5, {0.21557895, 0.41663158, 0.277263158,
                                    0.083578947, 0.006947368}};

}  // namespace

const char *window_name(Window w) {
    switch (w) {
        case Window::Rectangular: return "rectangular";
        case Window::Hann: return "hann";
        case Window::BlackmanHarris: return "blackman-harris";
        case Window::FlatTop: return "flat-top";
    }
    return "?";
}

bool window_from_string(const std::string &s, Window *out) {
    if (s == "rectangular" || s == "rect") { *out = Window::Rectangular; return true; }
    if (s == "hann" || s == "hanning")     { *out = Window::Hann; return true; }
    if (s == "blackman-harris" || s == "bh") { *out = Window::BlackmanHarris; return true; }
    if (s == "flat-top" || s == "flattop") { *out = Window::FlatTop; return true; }
    return false;
}

WindowFunction make_window(Window type, size_t n) {
    WindowFunction wf;
    wf.w.resize(n);

    if (type == Window::Rectangular) {
        std::fill(wf.w.begin(), wf.w.end(), 1.0f);
    } else {
        const CosineSum &cs = (type == Window::Hann)            ? kHann
                            : (type == Window::BlackmanHarris)  ? kBlackmanHarris
                                                                : kFlatTop;
        for (size_t i = 0; i < n; ++i) {
            double v = 0.0;
            for (int k = 0; k < cs.order; ++k) {
                const double sign = (k % 2 == 0) ? 1.0 : -1.0;
                v += sign * cs.a[k] * std::cos(2.0 * kPi * double(k) * double(i) /
                                               double(n - 1));
            }
            wf.w[i] = float(v);
        }
    }

    // ENBW = N * sum(w^2) / (sum(w))^2, computed from the coefficients so it
    // can never disagree with them.
    double s1 = 0.0, s2 = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double v = wf.w[i];
        s1 += v;
        s2 += v * v;
    }
    wf.s1 = s1;
    wf.enbw = double(n) * s2 / (s1 * s1);
    wf.coherent_gain = s1 / double(n);
    return wf;
}

size_t fft_size_for_rbw(double requested_rbw_hz, double enbw, double sample_rate_hz,
                        size_t min_n, size_t max_n, double *achieved_rbw_hz) {
    size_t n = min_n;
    while (n < max_n && rbw_hz(enbw, sample_rate_hz, n) > requested_rbw_hz) n *= 2;
    if (achieved_rbw_hz) *achieved_rbw_hz = rbw_hz(enbw, sample_rate_hz, n);
    return n;
}

const char *detector_name(Detector d) {
    switch (d) {
        case Detector::Peak: return "peak";
        case Detector::NegativePeak: return "negative-peak";
        case Detector::Sample: return "sample";
        case Detector::RMS: return "rms";
    }
    return "?";
}

bool detector_from_string(const std::string &s, Detector *out) {
    if (s == "peak" || s == "max")      { *out = Detector::Peak; return true; }
    if (s == "negative-peak" || s == "min" || s == "negpeak") {
        *out = Detector::NegativePeak; return true;
    }
    if (s == "sample")                  { *out = Detector::Sample; return true; }
    if (s == "rms" || s == "average" || s == "avg") { *out = Detector::RMS; return true; }
    return false;
}

void detect(const float *power, size_t n_in, Detector det, float *out, size_t n_out) {
    if (n_out == 0 || n_in == 0) return;

    for (size_t o = 0; o < n_out; ++o) {
        // Half-open bin range [lo, hi); guaranteed non-empty even when
        // n_out > n_in, in which case adjacent outputs repeat input bins.
        size_t lo = size_t(double(o) * double(n_in) / double(n_out));
        size_t hi = size_t(double(o + 1) * double(n_in) / double(n_out));
        if (hi <= lo) hi = lo + 1;
        if (hi > n_in) hi = n_in;
        if (lo >= n_in) lo = n_in - 1;

        switch (det) {
            case Detector::Peak: {
                float m = power[lo];
                for (size_t i = lo + 1; i < hi; ++i) m = std::max(m, power[i]);
                out[o] = m;
                break;
            }
            case Detector::NegativePeak: {
                float m = power[lo];
                for (size_t i = lo + 1; i < hi; ++i) m = std::min(m, power[i]);
                out[o] = m;
                break;
            }
            case Detector::Sample:
                // Middle of the range, so the sample is representative of the
                // bin rather than biased to one edge.
                out[o] = power[lo + (hi - lo) / 2];
                break;
            case Detector::RMS: {
                // Mean of LINEAR power. §5 is explicit that averaging dB is a
                // different and wrong operation; conversion happens once, later,
                // in power_to_db().
                double sum = 0.0;
                for (size_t i = lo; i < hi; ++i) sum += double(power[i]);
                out[o] = float(sum / double(hi - lo));
                break;
            }
        }
    }
}

Fft::Fft(size_t n) : n_(n) {
    setup_ = pffft_new_setup(int(n), PFFFT_COMPLEX);
    if (!setup_) throw std::runtime_error("pffft_new_setup failed (n must be 2^a*3^b*5^c, >=32)");
    in_ = (float *)pffft_aligned_malloc(n * 2 * sizeof(float));
    out_ = (float *)pffft_aligned_malloc(n * 2 * sizeof(float));
    work_ = (float *)pffft_aligned_malloc(n * 2 * sizeof(float));
    if (!in_ || !out_ || !work_) throw std::runtime_error("pffft_aligned_malloc failed");
}

Fft::~Fft() {
    if (work_) pffft_aligned_free(work_);
    if (out_) pffft_aligned_free(out_);
    if (in_) pffft_aligned_free(in_);
    if (setup_) pffft_destroy_setup(setup_);
}

void Fft::power_spectrum(const float *iq, const WindowFunction &win, float *power_out) {
    const float *w = win.w.data();
    for (size_t i = 0; i < n_; ++i) {
        in_[2 * i + 0] = iq[2 * i + 0] * w[i];
        in_[2 * i + 1] = iq[2 * i + 1] * w[i];
    }
    pffft_transform_ordered(setup_, in_, out_, work_, PFFFT_FORWARD);

    // Normalise so a full-scale complex sinusoid peaks at 1.0 (0 dBFS): its
    // peak bin has magnitude S1 = sum(w).
    const float inv = float(1.0 / (win.s1 * win.s1));

    // fftshift while writing: negative frequencies first, DC at n/2.
    const size_t half = n_ / 2;
    for (size_t k = 0; k < n_; ++k) {
        const size_t src = (k < half) ? (k + half) : (k - half);
        const float re = out_[2 * src + 0];
        const float im = out_[2 * src + 1];
        power_out[k] = (re * re + im * im) * inv;
    }
}

void power_to_db(float *v, size_t n, float floor_db) {
    const float lin_floor = std::pow(10.0f, floor_db / 10.0f);
    for (size_t i = 0; i < n; ++i)
        v[i] = 10.0f * std::log10(std::max(v[i], lin_floor));
}

// ---------------------------------------------------------------- trigger

VideoTrigger::VideoTrigger(bool rising, double level, double arm, size_t first,
                           size_t last)
    : rising_(rising), level_(level), arm_(arm), first_(first), last_(last) {}

void VideoTrigger::reset() {
    pos_ = 0;
    armed_ = false;
    fired_ = false;
    index_ = 0;
}

bool VideoTrigger::feed(float v) {
    if (fired_) return false;
    const size_t i = pos_++;
    if (i > last_) return false;

    if (!armed_) {
        // A sample on the low side arms but cannot also fire: one sample is a
        // level, not an edge.
        if (rising_ ? (double(v) < arm_) : (double(v) > arm_)) armed_ = true;
        return false;
    }
    if (rising_ ? (double(v) < level_) : (double(v) > level_)) return false;

    // A crossing. If it is too early to be shown with the requested
    // pre-trigger history it is still a crossing -- so the state machine
    // disarms and looks for the *next* edge, rather than firing on the first
    // in-window sample of a burst that started before the window. That would
    // put t=0 in the middle of a pulse and label it the edge.
    armed_ = false;
    if (i < first_) return false;
    fired_ = true;
    index_ = i;
    return true;
}

bool find_video_trigger(const float *video, size_t n, bool rising, double level,
                        double arm, size_t first, size_t last, size_t *out) {
    VideoTrigger t(rising, level, arm, first, last);
    for (size_t i = 0; i < n; ++i) {
        if (t.feed(video[i])) {
            if (out) *out = t.index();
            return true;
        }
        if (t.spent()) break;
    }
    return false;
}

size_t count_overload(const int16_t *iq, size_t n_pairs, int16_t threshold) {
    size_t n = 0;
    for (size_t i = 0; i < n_pairs * 2; ++i)
        if (iq[i] >= threshold || iq[i] <= int16_t(-threshold)) ++n;
    return n;
}

}  // namespace sweepd
