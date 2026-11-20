// The signal generator: the TX as a bench source. SATSAGEN's Generator and
// Gen/Sweeper panels.
//
// It runs alongside the analyzer, not instead of it -- the sweep and zero span
// never touch the TX -- so the radio can generate and measure at once, which is
// the whole value of a source on the bench. Tracking owns the TX, so entering
// tracking stops the generator and says so.
//
// ---------------------------------------------------------------- the carrier
//
// At baseband DC: the TX LO *is* the output frequency, and the DMA plays a
// constant (I, Q). An offset tone would put the carrier beside the LO and leave
// the LO leakage and the quadrature image as two spurs next to it, 40-50 dB
// down; at DC the leakage adds to the carrier itself and there is no image.
// Modulation is built around DC the same way.
//
// ------------------------------------------------------------- periodicity
//
// The DMA buffer is cyclic, so the waveform must be exactly periodic in it or
// the wrap is a phase step every buffer -- a spur comb at fs/L. So the buffer
// holds whole modulation periods, and the modulation rate is snapped to
// fs / samples_per_period, reported as achieved rather than as asked (§5's rule
// for RBW, applied to a generator). FM and FSK are phase-continuous across the
// wrap because a symmetric modulating waveform integrates to zero phase over a
// period.
//
// The buffer is bounded (kMaxGenSamples), which bounds the slowest modulation:
// at 61.44 MSPS, about 59 Hz.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sweepd {

constexpr size_t kMaxGenSamples = 1u << 20;   // 4 MB of sc16

enum class GenMod { CW, AM, FM, ASK, FSK };

const char *gen_mod_name(GenMod m);
bool gen_mod_from_string(const std::string &s, GenMod *out);

struct GenConfig {
    bool on = false;
    long long freq_hz = 1000000000LL;
    double atten_db = 50.0;      // TX attenuation; level control
    double scale = 0.7;          // DAC peak / full scale
    GenMod mod = GenMod::CW;
    double rate_hz = 1000.0;     // AM/FM tone, ASK/FSK symbol rate (square wave)
    double am_depth = 0.5;       // 0..1
    double fm_dev_hz = 5000.0;   // FM peak deviation, FSK shift (+/-)

    // Gen/Sweeper: step the carrier across [start, stop] every dwell. Off when
    // step_hz is 0.
    long long sweep_start_hz = 0, sweep_stop_hz = 0, sweep_step_hz = 0;
    double sweep_dwell_s = 0.1;
};

struct GenDerived {
    size_t samples = 0;          // buffer length
    double rate_hz = 0;          // achieved modulation rate; 0 for CW
    size_t sweep_points = 0;     // 0 when not sweeping
};

bool derive_gen(const GenConfig &c, double fs_hz, long long lo_min, long long lo_max,
                GenDerived *out, std::string *err);

// Interleaved int16 I/Q, d.samples pairs, peak scale x 32767.
void gen_waveform(const GenConfig &c, const GenDerived &d, double fs_hz,
                  std::vector<int16_t> *iq);

// The carrier for sweep step k.
long long gen_sweep_freq(const GenConfig &c, size_t k);

}  // namespace sweepd
