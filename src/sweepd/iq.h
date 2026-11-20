// Raw IQ capture -- the time-domain DSP behind the 0x03 message. Pure: no
// device, no socket, no threads, so sweepd, the synthetic source and the tests
// all run the same code, and what the UI is developed against is the rule the
// daemon applies rather than a second reading of it.
//
// What an IQ burst is, and why it is not a stream, is in proto.h ("0x03 iq").
// The short version: one contiguous acquisition into one buffer, triggered or
// not, decimated or not, transferred after it completes. Re-arming is the
// receiver's request, one burst per request, so the link sets the pace and a
// gap between bursts is a gap the UI can count and report -- never one hidden
// inside a burst.
//
// ---------------------------------------------------------------- decimation
//
// The AD936x's sample rate is fixed for the whole session: changing it
// reprograms the BBPLL and invalidates every fastlock profile (CLAUDE.md §5).
// A longer time window or a narrower view is therefore bought with an integer
// decimator on the ARM, never with a rate change. It is a windowed-sinc
// low-pass followed by keep-one-in-D, computed only at the kept outputs.
//
// Each burst is filtered on its own, with no state carried between bursts:
// bursts are not contiguous with one another, and a filter state carried
// across the gap would smear the previous burst's last samples into this one's
// first -- a transient made of a moment that is not on screen. Instead only the
// outputs whose whole impulse response lies inside the burst are produced, so
// every sample sent is a fully settled one. The cost is `taps - 1` input samples
// per burst, which the capture length accounts for.
//
// ---------------------------------------------------------------- trigger
//
// A level crossing on I, Q or |x| with hysteresis, like a scope's edge trigger.
// Hysteresis is a depth on the far side of the level the signal must first
// reach before a crossing counts, so noise riding on a slow edge does not fire
// it twice. The crossing is located to a fraction of a sample by linear
// interpolation; the frame's t0 carries that fraction, so a repetitive
// waveform sits still on screen instead of jittering by up to one sample.
//
// ---------------------------------------------------------------- statistics
//
// Blind estimates over a stretch of samples: DC, RMS, peaks, the I/Q gain and
// phase mismatch and the image rejection they imply, and the frequency of a
// dominant tone from the mean phase increment. None of them is an FFT (§8:
// no FFT on the host), and each one says in its comment what it assumes,
// because every one of them is a number that can be confidently wrong.

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sweepd {

// ---------------------------------------------------------------- trigger

enum class IqTrigMode : uint8_t {
    None = 0,     // free run: the burst starts wherever the capture did
    Normal = 1,   // only a burst that contains an edge is sent
    Auto = 2,     // an edge if one turns up within the auto window, else the
                  // last acquisition, flagged as untriggered
};
enum class IqTrigSource : uint8_t { I = 0, Q = 1, Mag = 2 };

const char *iq_trig_mode_name(IqTrigMode m);
bool iq_trig_mode_from_string(const std::string &s, IqTrigMode *out);
const char *iq_trig_source_name(IqTrigSource s);
bool iq_trig_source_from_string(const std::string &s, IqTrigSource *out);

struct IqTrigger {
    IqTrigMode mode = IqTrigMode::None;
    IqTrigSource source = IqTrigSource::I;
    bool rising = true;
    double level = 0.0;     // full-scale units: I and Q are -1..1, |x| is 0..1.414
    double hyst = 0.01;     // FS, >= 0
    double pretrig = 0.25;  // fraction of the burst before the edge, 0..0.95
};

// The quantity the trigger watches, for sample `i` of interleaved I/Q.
inline float iq_trig_value(const float *iq, size_t i, IqTrigSource s) {
    const float re = iq[2 * i], im = iq[2 * i + 1];
    switch (s) {
        case IqTrigSource::I: return re;
        case IqTrigSource::Q: return im;
        case IqTrigSource::Mag: break;
    }
    return std::sqrt(re * re + im * im);
}

// Searches interleaved I/Q `iq` (n samples) for the first crossing of
// `t.level` in the trigger's direction whose crossing sample index lies in
// [from, to). Armed by first seeing the signal beyond the hysteresis depth; the
// search may arm on samples before `from`. On success `*at` is the first sample
// at or past the level and `*frac` (0..1) is how far before `*at` the
// interpolated crossing lies, so the edge is at (*at - *frac) samples.
bool find_iq_trigger(const float *iq, size_t n, const IqTrigger &t, size_t from,
                     size_t to, size_t *at, double *frac);

// ---------------------------------------------------------------- decimator

class IqDecimator {
public:
    // decim 1 is a pass-through with no filter at all: the raw ADC samples are
    // what was asked for, and a filter would change them.
    explicit IqDecimator(unsigned decim = 1);

    unsigned decim() const { return decim_; }
    size_t taps() const { return h_.size(); }

    // Input samples needed for `n_out` fully settled outputs.
    size_t input_for(size_t n_out) const {
        return n_out == 0 ? 0 : (n_out - 1) * decim_ + h_.size();
    }
    // Output n_out from `n_in` input samples, the inverse of input_for.
    size_t output_for(size_t n_in) const {
        return n_in < h_.size() ? 0 : (n_in - h_.size()) / decim_ + 1;
    }
    // Input index the centre of output k's impulse response sits on. The
    // filter is linear-phase, so this is where output k "is" in time.
    double input_index_of(size_t k) const {
        return double(k) * decim_ + double(h_.size() - 1) * 0.5;
    }

    // Interleaved float I/Q in, interleaved float I/Q out, both in the same
    // units (the filter has unity DC gain). Writes output_for(n_in) samples.
    void run(const float *in, size_t n_in, std::vector<float> *out) const;

    // The passband edge the filter is designed to keep flat, as a fraction of
    // the OUTPUT rate: content within +/- this x fs_out is passed, the rest of
    // the output Nyquist zone is the transition band. Reported to the UI so the
    // edge of the usable band is a number on screen, not a surprise.
    static constexpr double kPassFraction = 0.4;

private:
    unsigned decim_ = 1;
    std::vector<float> h_;
};

bool iq_decim_valid(unsigned d);   // 1, 2, 4, ..., 64

// ---------------------------------------------------------------- geometry

// What a burst is asked to be. The sample rate is not in here: it is the
// session's, fixed (header comment), and arrives as adc_fs_hz.
struct IqConfig {
    long long center_hz = 0;
    unsigned decim = 1;
    size_t n_samples = 16384;      // OUTPUT samples per burst
    IqTrigger trig;
    double auto_s = 0.1;           // how long Auto looks for an edge before giving up
};

// The cap on n_samples. A cf32 burst is 8 bytes a sample and has to fit under
// proto::kMaxPayload (64 MB) with its header; 4 Mi samples is 32 MB, half of it,
// and at the link's ~50 MB/s is already most of a second per burst.
constexpr size_t kIqMaxSamples = size_t(1) << 22;

// How one acquisition is laid out, in OUTPUT samples unless named otherwise.
// Free run captures exactly the burst. A trigger captures the burst plus a
// search region and the edge may fall anywhere in [pre, pre + search): the
// burst is then the window [edge - pre, edge - pre + n_samples) -- pre-trigger
// history from inside the same refill, the rule zero span follows (zspan.h),
// because two refills are not contiguous and a burst spliced from both would
// hide the gap inside itself.
struct IqGeometry {
    size_t pre = 0;
    size_t search = 0;
    size_t acq_out = 0;     // decimated samples one acquisition yields
    size_t acq_in = 0;      // ADC samples one acquisition needs
    size_t buf_in = 0;      // the DMA buffer: acq_in rounded up to kIqBufAlign
};

// The AXI DMA hands back an all-zero buffer, with no error, when the buffer is
// not a whole number of 8-sample units: measured on a PlutoSDR (sweepd raw IQ,
// 2026-11-06) -- 4101 samples read as silence, 4104 and 4096 as signal. The
// sweep never met it because it rounds its buffers to 1024 for another reason
// (sweep.cpp, kBufGranularity), and zero span's sizes happen to be multiples of
// its hop. Raw IQ lengths are arbitrary, so the buffer is rounded up and the
// burst still takes exactly the samples it needs from the front of it.
constexpr size_t kIqBufAlign = 8;

// Validates `c` against the device buffer (`max_input` ADC samples) and lays
// out the acquisition. On refusal the message names the knob, and the number
// that would fit.
bool derive_iq(const IqConfig &c, size_t max_input, IqGeometry *g, std::string *err);

// Cuts one burst out of a decimated acquisition. With a trigger, searches
// [pre, pre + search) and on an edge copies the window around it, sets
// *t0_ns so t = 0 lands on the interpolated crossing, and *trig_index to the
// sample at or after it; returns false with nothing written if there is no
// edge. Without a trigger (or with `force`, for Auto's timeout) copies the
// first n_samples and sets t0 = 0.
bool iq_cut(const float *acq, size_t acq_n, const IqConfig &c, const IqGeometry &g,
            double fs_hz, bool force, std::vector<float> *out, int64_t *t0_ns,
            uint32_t *trig_index);

// ---------------------------------------------------------------- sc16

// int16 interleaved -> float interleaved, divided by `full_scale` (2048 for the
// AD936x's 12 bits). Counts samples with |I| or |Q| >= clip_frac x full scale,
// the same rule the sweep's overload flag uses.
size_t iq_sc16_to_float(const int16_t *in, size_t n, double full_scale, double clip_frac,
                        std::vector<float> *out);

// ---------------------------------------------------------------- statistics

struct IqStats {
    size_t n = 0;
    double dc_i = 0, dc_q = 0;        // mean, FS
    double rms = 0;                   // sqrt(mean |x|^2), FS, DC included
    double rms_ac = 0;                // the same with DC removed
    double peak_mag = 0;              // max |x|
    double min_i = 0, max_i = 0, min_q = 0, max_q = 0;
    double crest_db = 0;              // peak/rms_ac, dB (PAPR of the AC part)
    double power_dbfs = -300;         // 20 log10(rms): mean power re full scale

    // Mismatch between the two arms, estimated blind from second-order
    // statistics after the DC is removed. Valid when the signal is "circular"
    // -- a tone away from DC, noise, most digital modulations -- because then
    // E[I^2] = E[Q^2] and E[IQ] = 0 in a perfect receiver, and any departure is
    // the receiver. A real-valued signal (pure AM at DC, a tone exactly at 0 Hz)
    // is not circular and these numbers then describe the signal instead.
    double gain_imb_db = 0;           // 20 log10(rms_I / rms_Q)
    double phase_imb_deg = 0;         // departure from 90 degrees
    double irr_db = 0;                // image rejection the two imply
    bool circular_ok = false;         // the estimate below is worth showing
    bool too_few_cycles = false;      // ... and if not, because the window is too short

    // Dominant tone offset from the centre, in Hz, from the mean phase
    // increment arg(sum x[n] conj(x[n-1])). Exact for one tone, a power-weighted
    // average for several -- `tone_coherence` says which: |mean phasor| over
    // mean |x|^2, near 1 for a single clean tone and near 0 for noise. The
    // receiver's own image of the tone counts as a second tone: at 30 dB image
    // rejection it pulls the estimate by about 0.2 %.
    double tone_hz = 0;
    double tone_coherence = 0;
    double tone_cycles = 0;           // turns of that tone inside the window
};

// Over samples [i0, i1) of interleaved `iq`, at sample rate fs_hz.
IqStats iq_stats(const float *iq, size_t i0, size_t i1, double fs_hz);

}  // namespace sweepd
