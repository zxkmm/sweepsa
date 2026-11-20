// DSP core for sweepd -- windows, FFT, detectors. CLAUDE.md §5.
//
// Deliberately free of any device dependency so it can be unit-tested on the
// host. Everything here is exercised by tests/dsp_test.cpp.

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

struct PFFFT_Setup;

namespace sweepd {

// ---------------------------------------------------------------- windows

enum class Window { Rectangular, Hann, BlackmanHarris, FlatTop };

const char *window_name(Window w);
bool window_from_string(const std::string &s, Window *out);

// A window plus the two scalars every downstream calculation needs.
//
// Both figures are computed numerically from the coefficients rather than
// looked up from a table. CLAUDE.md §8 forbids unjustified magic constants, and
// a table would silently disagree with the coefficients if either changed.
struct WindowFunction {
    std::vector<float> w;
    double enbw = 1.0;            // equivalent noise bandwidth, in bins
    double coherent_gain = 1.0;   // S1/N; peak amplitude scaling

    // S1 = sum(w). A full-scale complex sinusoid produces |X[k]| = S1 at its
    // peak bin, so dividing by S1 normalises full scale to 0 dBFS.
    double s1 = 0.0;
};

WindowFunction make_window(Window type, size_t n);

// ---------------------------------------------------------------- RBW

// CLAUDE.md §5: "RBW is derived, not asserted."
inline double rbw_hz(double enbw, double sample_rate_hz, size_t n_fft) {
    return enbw * sample_rate_hz / double(n_fft);
}

// Smallest FFT size (power of two, >= min_n) whose achieved RBW is <= the
// request. Returns the achieved RBW via *achieved so the caller can tell the
// user what it actually got -- §5 requires snapping to be reported, not silent.
size_t fft_size_for_rbw(double requested_rbw_hz, double enbw, double sample_rate_hz,
                        size_t min_n, size_t max_n, double *achieved_rbw_hz);

// ---------------------------------------------------------------- detectors

enum class Detector { Peak, NegativePeak, Sample, RMS };

const char *detector_name(Detector d);
bool detector_from_string(const std::string &s, Detector *out);

// Reduce `power` (linear power, not dB) onto `n_out` output bins.
//
// CLAUDE.md §5: "RMS averages power, then converts to dB. Averaging dB values is
// a different and wrong operation." All four detectors therefore operate in
// linear power and only the caller converts to dB, once, at the end.
//
// Output point count is independent of FFT size, also per §5.
void detect(const float *power, size_t n_in, Detector det,
            float *out, size_t n_out);

// ------------------------------------------------------- video trigger
//
// CLAUDE.md §6: zero span wants "free-run and video triggering". A video
// trigger fires on the *detected* trace -- the same numbers zero span draws,
// one per FFT block -- and not on the raw IQ envelope.
//
// That choice is the whole design and it is not an implementation detail. The
// level the user dials in is read off the y-axis of the trace in front of them,
// so the quantity compared against it has to be the quantity that axis shows:
// same window, same ENBW, same measurement band, same detector. Triggering on
// wideband time-domain magnitude would be far cheaper -- no FFT to find the
// edge -- but it measures the whole sampled bandwidth including the LO leakage
// at DC that zspan.h's offset exists to dodge, so a level set from the screen
// would fire somewhere else entirely. Cheap and answering a different question.
//
// This lives in dsp.h rather than in zspan.cpp so the daemon and sweepui's
// synthetic source share one implementation. The synthetic source has to
// trigger too (§6 requires the UI be developable with no hardware) and a second
// reading of "what counts as an edge" is exactly the kind of duplicate rule
// this codebase keeps finding it has three copies of.
//
// Everything here is in *linear power*, matching detect(): the caller converts
// its dB level once instead of this converting n samples.

// The arming rule, stated once because it is the part that surprises people:
// a crossing is an edge, not a level. A rising trigger needs a sample on the
// low side of `arm` *before* it will accept one at or above `level`, so an
// acquisition that begins already above the level waits for the next real edge
// instead of reporting its own first sample as a trigger. With arm == level
// that is a plain crossing; separating them is hysteresis, which keeps a floor
// fluctuating either side of the level from firing on noise.
class VideoTrigger {
public:
    // `first`/`last` bound where the trigger may *fire*, inclusive. Arming is
    // not bounded: samples before `first` are the pre-trigger history and are
    // allowed to be the low side of the edge, which is the only way an edge at
    // the very start of the search window can be caught at all.
    VideoTrigger(bool rising, double level, double arm, size_t first, size_t last);

    // Feeds the next sample. Returns true exactly once, on the sample that
    // triggers; index() is then that sample's position. Feeding past a fire is
    // harmless and changes nothing.
    bool feed(float v);

    bool fired() const { return fired_; }
    size_t index() const { return index_; }

    // True once no later sample could fire -- the search window is spent. Lets
    // a caller stop transforming blocks it can no longer use.
    bool spent() const { return fired_ || pos_ > last_; }

    void reset();

private:
    bool rising_;
    double level_, arm_;
    size_t first_, last_;
    size_t pos_ = 0;
    bool armed_ = false;
    bool fired_ = false;
    size_t index_ = 0;
};

// Batch form, for a video series that is already in hand. Same rule, same
// object: this is a loop over feed(), not a second reading of it.
bool find_video_trigger(const float *video, size_t n, bool rising, double level,
                        double arm, size_t first, size_t last, size_t *out);

// ---------------------------------------------------------------- FFT

// Thin owner around pffft, which won the §4b.5 benchmark (1.25-1.43x kissfft).
// Kept behind this interface because §5 requires the library be swappable.
class Fft {
public:
    explicit Fft(size_t n);
    ~Fft();
    Fft(const Fft &) = delete;
    Fft &operator=(const Fft &) = delete;

    size_t size() const { return n_; }

    // Interleaved complex input (I,Q,I,Q,...) of n() pairs. Applies `win`,
    // transforms, and writes n() linear power values to `power_out`.
    //
    // Power is normalised so that a full-scale complex sinusoid reads 1.0
    // (0 dBFS): |X|^2 / S1^2.
    //
    // Output is in natural order with DC at index n/2 (fftshifted), because the
    // sweep splices segments by frequency and wants monotonic bins.
    void power_spectrum(const float *iq_interleaved, const WindowFunction &win,
                        float *power_out);

private:
    size_t n_;
    PFFFT_Setup *setup_;
    float *in_;
    float *out_;
    float *work_;
};

// ---------------------------------------------------------------- helpers

// Convert linear power to dB in place. Floored to avoid -inf on exact zeros.
void power_to_db(float *v, size_t n, float floor_db = -200.0f);

// Count samples at or beyond `threshold` of full scale. CLAUDE.md §5 requires a
// per-segment overload flag: "An overloaded front end produces convincing fake
// spurs, and it must never be silent."
size_t count_overload(const int16_t *iq_interleaved, size_t n_pairs,
                      int16_t threshold);

}  // namespace sweepd
