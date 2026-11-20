// The tracking generator's geometry: where the tones go, which LO measures
// which point, and the stimulus itself. Split from track.h for the reason
// plan.h is split from profiles.h -- this is arithmetic, and sweepui's
// synthetic source needs it on Windows, where the engine's POSIX timing does
// not build. See track.h for what the numbers mean.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "dsp.h"

namespace sweepd {

// Below this the TX can drive a direct TX->RX cable past the receiver's
// absolute maximum (tools/toneg, "safety"). Refused unless the daemon was
// started with --tsa-pad.
constexpr double kTrackLoudAttenDb = 30.0;
constexpr double kTrackMaxAttenDb = 89.75;

struct TrackConfig {
    long long f_start_hz = 70000000LL;
    long long f_stop_hz = 6000000000LL;

    // Point spacing, snapped to 4q FFT bins (q >= 1). 1 MHz at N=4096 and
    // 61.44 MSPS becomes 1.02 MHz.
    long long spacing_hz = 1000000LL;

    // Tones per LO step. 0 = as many as keep_hz holds; 1 = classic stepped CW.
    size_t tones = 0;

    // Usable width around each LO. 0 = 72% of the RF bandwidth, which on a
    // 56 MHz radio is the sweep's own 40 MHz and on a 20 MHz AD9363-class
    // Pluto is 14.4 MHz.
    long long keep_hz = 0;

    size_t n_fft = 4096;
    size_t coherent = 8;
    Window window = Window::Hann;

    // ---- what the receiver listens to, relative to the stimulus.
    //
    // RX = harmonic x TX + rx_offset (or rx_offset - harmonic x TX when
    // inverted). The trace's x axis stays the STIMULUS frequency, as on
    // SATSAGEN and most SNAs: a harmonic or converter measurement is read
    // against what went in.
    //
    //   harmonic > 1   a DUT's nth harmonic output (amplifiers, multipliers).
    //                  The RX sees the tone set n times wider, so fewer fit.
    //   rx_offset_hz   frequency converters and mixers: the DUT's output sits
    //                  this far from its input (the external LO, for a mixer).
    //   rx_invert      a converter whose output band is mirrored (LO above).
    int harmonic = 1;
    long long rx_offset_hz = 0;
    bool rx_invert = false;

    // The stimulus silenced but everything else running: what the receiver
    // reads with nothing sent is the floor below which a trace means nothing
    // (SATSAGEN's "TX Off").
    bool stim_off = false;

    double atten_db = 50.0;   // TX attenuation, the level control
    double scale = 0.5;       // DAC peak / full scale of the multitone

    // Same meaning and justification as SweepConfig's: the driver blocks the LO
    // write until calibration finishes, so this only has to cover what survives
    // the drain (docs/characterization.md, "§4b.2").
    double settle_us = 267.0;
    double min_buffer_us = 267.0;
    double overload_frac = 0.98;

    // The point count past which a spacing is a typo, not a measurement.
    size_t max_points = 262144;
};

// One LO step: which output points it measures and with which tones.
struct TrackStep {
    long long lo_hz = 0;        // TX LO
    long long rx_lo_hz = 0;     // RX LO: equal to lo_hz unless harmonic/offset
    size_t p0 = 0;      // first output point
    size_t cnt = 0;     // points measured here
    int t0 = 0;         // tone index of point p0
};

struct TrackDerived {
    double fs_hz = 0;
    size_t n_fft = 0;
    size_t coherent = 1;
    double bin_hz = 0;          // fs / n_fft
    int q = 1;
    double spacing_hz = 0;      // 4q bins
    long long keep_hz = 0;
    size_t tones_per_step = 0;
    std::vector<int> tone_bins; // offsets from the TX LO bin, ascending
    int harmonic = 1;
    long long rx_offset_hz = 0;
    bool rx_invert = false;
    // Where tone m lands in the RX transform: rx_sign x harmonic x tone_bins[m].
    int rx_bin(size_t m) const { return (rx_invert ? -1 : 1) * harmonic * tone_bins[m]; }

    // Output grid: point i is at out_lo_hz + (i + 0.5) * spacing_hz.
    size_t n_points = 0;
    double out_lo_hz = 0;
    double out_hi_hz = 0;

    std::vector<TrackStep> steps;

    double enbw = 0;
    double rbw_hz = 0;          // ENBW x fs / (n_fft x coherent)
    size_t settle_samples = 0;
    size_t buf_samples = 0;
    double buffer_us = 0;
};

bool derive_track(const TrackConfig &cfg, double fs_hz, long long bw_hz, long long lo_min_hz,
                  long long lo_max_hz, TrackDerived *out, std::string *err);

// The cyclic TX buffer: n_fft interleaved int16 I/Q pairs, peak at
// scale x 32767 (the AD936x DAC takes 12 bits MSB-aligned in 16). Newman
// phases keep the crest factor near 2.6 dB for 39 tones rather than 16 dB.
// *tone_dbfs is each tone's level relative to a full-scale CW.
void track_stimulus(const TrackDerived &d, double scale, std::vector<int16_t> *iq,
                    double *crest_db, double *tone_dbfs);

// Copies step `s`'s tones out of an fftshifted linear power spectrum into
// points[p0 .. p0+cnt).
void track_extract(const TrackDerived &d, size_t s, const float *power, float *points);

}  // namespace sweepd
