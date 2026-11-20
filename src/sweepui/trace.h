// The four traces. CLAUDE.md §6, "Traces":
//
//   "4 independent, each Clear/Write, Max Hold, Min Hold, Average(N), View,
//    Blank."
//
// No ImGui, no GL, no source dependency, so tests/trace_test.cpp exercises all
// of it natively. That matters more here than it looks: every mistake this file
// can make produces a *plausible spectrum*, not an error, and the two that
// matter are described below.
//
// ------------------------------------------------------- averaging in power
//
// CLAUDE.md §5, on the RMS detector:
//
//   "RMS averages power, then converts to dB. Averaging dB values is a
//    different and wrong operation -- do not do it by accident."
//
// That rule is about the detector, but it is the same rule here and this is its
// third independent implementation in the project (sweepd::detect, the sweep
// loop's stitch, and now the display average). Average(N) converts each sweep's
// dB back to linear power, averages that, and converts once at the end. A dB
// average of a noisy floor reads low -- about 2.5 dB low for exponentially
// distributed noise power -- and it reads low by an amount that depends on how
// noisy the bin is, so it flatters the noise floor and not the signals. That is
// exactly the kind of quietly-wrong number §7 exists to prevent, and it would
// then be inherited by the marker noise readout, which divides by the RBW and
// presents the result as a power spectral density.
//
// ---------------------------------------------------------------- on NaN
//
// proto.h: the trace carries NaN for bins no segment covered. Those bins are
// *unmeasured*, not zero and not quiet, so they take no part in a hold or an
// average: a NaN sample leaves a held maximum where it was, and a bin that has
// never carried a real value stays NaN however many sweeps go by. A max hold
// that treated NaN as -inf would be unaffected, but a min hold that did would
// pin every blanked bin at the floor forever and draw a solid line across the
// dc_policy=blank holes -- which is a picture of a receiver that measured
// something, in a band where it measured nothing.
//
// ------------------------------------------------------- what clears a trace
//
// A held or averaged trace is an accumulation of sweeps that are assumed
// comparable. Two things break that assumption and both silently:
//
//   * the geometry changes -- span, point count -- so bin i is a different
//     frequency than it was, and the accumulator is a blend of two axes
//   * the detector or the window changes, so the samples being combined are
//     different quantities (a peak-detected bin and an RMS-detected bin are not
//     two measurements of one thing)
//
// TraceStore::push() checks all of that against the frame and clears every
// trace when it moves. Losing a max hold on a retune is annoying; keeping one
// that spans a retune is wrong.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "proto.h"

namespace sweepui {

enum class TraceMode {
    ClearWrite,   // show the newest sweep
    MaxHold,      // per-bin maximum since the last clear
    MinHold,      // per-bin minimum since the last clear
    Average,      // per-bin mean of the last N sweeps, in linear power
    View,         // frozen: keep what is there, stop updating, keep drawing
    Blank,        // frozen and not drawn
};

const char *trace_mode_name(TraceMode m);
bool trace_mode_from_string(const std::string &s, TraceMode *out);

// True for the modes that consume incoming sweeps.
bool trace_mode_accumulates(TraceMode m);

class Trace {
public:
    // Mode changes go through here rather than through a public field, because
    // entering an accumulating mode has to clear -- see set_mode().
    void set_mode(TraceMode m);
    TraceMode mode() const { return mode_; }

    // Number of sweeps to average over. Changing it clears, because the
    // accumulator's weight is 1/N and a mid-flight change would leave a trace
    // that is neither an N-average nor an M-average.
    void set_average_n(int n);
    int average_n() const { return average_n_; }

    // Sweeps folded in since the last clear. The UI shows this against
    // average_n() so an average that has not converged is visibly unconverged.
    uint64_t sweeps() const { return sweeps_; }
    bool converged() const {
        return mode_ != TraceMode::Average || sweeps_ >= uint64_t(average_n_);
    }

    // dBFS, one per bin. Empty until the first sweep arrives.
    const std::vector<float> &db() const { return db_; }
    size_t size() const { return db_.size(); }
    bool empty() const { return db_.empty(); }

    // Drawn when it has data and is not blanked. View is drawn: that is the
    // difference between View and Blank and the only reason both exist.
    bool visible() const { return !db_.empty() && mode_ != TraceMode::Blank; }

    void clear();

    // Folds one sweep in. A no-op for View and Blank.
    void update(const float *db, size_t n);

private:
    void reset_to(size_t n);

    TraceMode mode_ = TraceMode::Blank;
    int average_n_ = 10;
    uint64_t sweeps_ = 0;

    std::vector<float> db_;
    // Average state, per bin. Linear power, and a per-bin count because a bin
    // that was NaN for the first six sweeps has seen six fewer samples than its
    // neighbours and must be weighted as such.
    std::vector<double> mean_power_;
    std::vector<uint32_t> count_;
};

// The four traces plus the geometry guard. §6 asks for four; the number is
// fixed rather than configurable because the soft-key column, the marker's
// trace selector and the state file all name them individually.
class TraceStore {
public:
    static constexpr size_t kCount = 4;

    Trace &operator[](size_t i) { return t_[i < kCount ? i : 0]; }
    const Trace &operator[](size_t i) const { return t_[i < kCount ? i : 0]; }
    size_t size() const { return kCount; }

    // Folds a frame into every accumulating trace, clearing all four first if
    // the frame is not comparable with the ones already accumulated.
    // Returns true if it cleared, so the caller can say so.
    bool push(const sweepd::proto::TraceMsg &f);

    void clear_all();

    // Index of the lowest-numbered visible trace, or kCount if none is. The
    // markers read this one by default: a marker reading a blanked trace would
    // report a number that is not on screen.
    size_t first_visible() const;

private:
    Trace t_[kCount];

    // The geometry the accumulators were built against. Compared, not trusted.
    bool have_key_ = false;
    long long f_start_hz_ = 0;
    long long f_stop_hz_ = 0;
    sweepd::proto::XKind x_kind_ = sweepd::proto::XKind::Frequency;
    uint64_t dt_ns_ = 0;
    size_t n_points_ = 0;
    sweepd::Detector detector_ = sweepd::Detector::Peak;
    sweepd::Window window_ = sweepd::Window::Hann;
};

}  // namespace sweepui
