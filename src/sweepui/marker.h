// Markers. CLAUDE.md §6, "Markers":
//
//   "8, normal and delta. Peak search, next peak, next left/right,
//    marker->center, marker->ref-level, marker table, marker noise mode
//    (dBm/Hz using the *actual* RBW and window ENBW)."
//
// No ImGui, no GL, so tests/marker_test.cpp exercises the search and the
// readout natively. What is here is the arithmetic and the searching; the
// actions that move the *display* -- marker->center, marker->ref-level -- are
// in app.cpp, because they write the view and the view is the app's.
//
// ------------------------------------------------- a marker holds a frequency
//
// Not a bin index. A bin index is only meaningful against one (f_start, span,
// n_points) triple, and all three move at runtime now that the frequency keys
// retune the daemon (PROJECT-STATE.md, "Runtime reconfiguration"). A marker
// that stored a bin would slide to a different frequency on every span change
// while continuing to display a confident number. Storing the frequency means a
// retune leaves the marker where the user put it, and a marker that ends up
// outside the swept span reads as having no value rather than as having the
// value of whatever bin its old index now points at.
//
// ------------------------------------------------------- peak excursion
//
// "Next peak" on a noise floor without an excursion criterion walks from one
// noise sample to the next and never leaves the grass. Every analyzer solves
// this the same way: a candidate counts as a peak only if the trace rose at
// least `excursion_db` on the way up to it and fell at least that far on the
// way down. find_peaks() is the usual single-pass state machine for that.
//
// The textbook version needs *both* sides, and that costs it two cases that
// matter here: a tone at the edge of the visible range, and a tone whose
// downhill side is inside a dc_policy=blank hole. In both, the fall that would
// confirm the peak is simply not in the data -- and reporting "no peak found"
// with a tone plainly on screen is the kind of answer that makes the user stop
// believing the instrument. So a candidate is also confirmed at a boundary (the
// end of the range, or a NaN) if its *rise* alone met the excursion. It rose
// 6 dB above the floor it came from; that is a feature, whatever happens to its
// right. A flat floor still yields nothing, because nothing rose.
//
// ------------------------------------------------------- marker noise mode
//
// §6 asks for a density "using the *actual* RBW and window ENBW". The divisor
// is rbw_hz from the frame, which sweepd already computed as ENBW x fs / N_fft
// (dsp.h) -- that is a noise bandwidth in Hz, so
//
//     density = 10*log10( mean_power_over_bins ) - 10*log10(rbw_hz)
//
// Three things about that are easy to get wrong:
//
//   * The divisor is rbw_hz, not bin_hz. The output bin grid is decimated from
//     the FFT and its width has nothing to do with the noise bandwidth of the
//     measurement. On a full span the output bins are far *wider* than the RBW,
//     and dividing by the bin width would understate the density by tens of dB.
//   * The averaging over neighbouring bins is in linear power (§5 again). A dB
//     average here would be the same quietly-low answer the trace average
//     avoids, and it would be presented as a calibrated-looking density.
//   * The +2.5 dB correction quoted in application notes for noise markers is
//     a correction for *log* averaging and log detection. This path averages
//     power, so applying it would be a 2.5 dB error, not a fix. It is
//     deliberately absent.
//
// And one that cannot be fixed here, only reported: with a peak detector the
// trace is the maximum of the FFT bins under each output bin, so its noise
// floor sits well above the mean and the density reads high. That is a property
// of the sweep, not of this arithmetic, so noise_detector_ok() exists and the
// UI says so rather than silently correcting a number it cannot know.

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "dsp.h"

namespace sweepui {

// ---------------------------------------------------------------- searching

struct PeakOptions {
    // A candidate must rise this far above the preceding valley and fall this
    // far into the following one. 6 dB is the usual instrument default.
    double excursion_db = 6.0;

    // And be at least this high. -200 dBFS is sweepd's own floor (dsp.h's
    // power_to_db), so the default excludes nothing.
    double threshold_db = -200.0;

    // Bin range to search, `hi` exclusive. hi == 0 means "to the end". The UI
    // passes the visible range, so peak search finds what is on screen.
    size_t lo = 0;
    size_t hi = 0;
};

// Every qualifying peak in the range, in increasing bin order. NaN bins are
// unmeasured and break the run: a signal either side of a dc_policy=blank hole
// is two features, not one that happens to dip.
void find_peaks(const float *db, size_t n, const PeakOptions &opt,
                std::vector<size_t> *out);

// Highest qualifying peak. False if none qualifies.
bool peak_search(const float *db, size_t n, const PeakOptions &opt, size_t *out_bin);

// The next peak *down* in amplitude from the one at `from_bin`. Ties are broken
// by bin so repeated presses walk the whole list instead of oscillating between
// two equal peaks.
bool next_peak_lower(const float *db, size_t n, const PeakOptions &opt,
                     size_t from_bin, size_t *out_bin);

// The nearest qualifying peak to the left / right of `from_bin`, whatever its
// amplitude.
bool next_peak_left(const float *db, size_t n, const PeakOptions &opt,
                    size_t from_bin, size_t *out_bin);
bool next_peak_right(const float *db, size_t n, const PeakOptions &opt,
                     size_t from_bin, size_t *out_bin);

// ---------------------------------------------------------------- the axis

// The mapping between an x value and a bin of one trace. Bin centres are
// x0 + (i + 0.5) * dx, matching decimate.cpp and sweepd::proto::TraceMsg::x_at
// -- if any of the three disagreed, a marker would sit half a bin away from the
// peak it is reading.
//
// x is a frequency in Hz when `time` is false and a time in seconds when it is
// true (zero span, proto.h "one trace format"). The arithmetic is identical in
// both, which is the whole reason zero span rides the same frame: peak search,
// the delta, the noise density and the hold logic are one implementation
// serving both modes, and a marker reads the same way in either.
//
// `rbw_hz` stays a frequency in both. It is the resolution bandwidth the FFT
// achieved, which is what the noise density divides by, and that is a bandwidth
// whether the x-axis happens to be time or frequency.
struct TraceAxis {
    double x0 = 0;
    double dx = 0;
    size_t n = 0;
    double rbw_hz = 0;

    // Formatting and messages only; no arithmetic depends on it.
    bool time = false;

    bool valid() const { return n > 0 && dx > 0; }
    double x_at(size_t i) const { return x0 + (double(i) + 0.5) * dx; }

    // The bin containing `x`, or false if it is off the end. Not clamped: a
    // marker outside the trace has no reading, and clamping it to the edge
    // would give it somebody else's.
    bool bin_of(double x, size_t *out) const;

    // "outside the swept span" / "outside the captured window" -- the two send
    // the user somewhere different, so the readout says which.
    const char *outside_reason() const {
        return time ? "outside the captured time window" : "outside the swept span";
    }
};

// ---------------------------------------------------------------- markers

struct Marker {
    bool on = false;

    // Where the marker sits on the x-axis: Hz while sweeping, seconds in zero
    // span. Stored as a *position*, never as a bin index, because all of
    // (x0, dx, n_points) move at runtime -- and in zero span the whole axis
    // changes units -- so a stored bin would slide to a different place on
    // every retune while continuing to display a confident number.
    double x = 0;

    // §6: "normal and delta". A delta marker reads the difference from its
    // reference, in dB and in Hz. The reference is another marker, by index.
    bool delta = false;
    size_t ref = 0;

    // Which of the four traces it reads.
    size_t trace = 0;

    // §6's noise mode: report a density in dB/Hz instead of a level.
    bool noise = false;
};

struct MarkerReadout {
    bool on = false;
    bool valid = false;          // false when the marker is off the swept span
    bool is_delta = false;
    bool is_noise = false;

    // Hz while sweeping, seconds in zero span -- whichever the axis carries.
    double x = 0;                // absolute, or the difference for a delta
    double db = 0;               // level, delta, or density -- see is_*
    double abs_x = 0;            // always the marker's own position
    double abs_db = 0;           // always its own level, even in delta mode
    size_t bin = 0;
    bool time = false;           // what x is in, copied from the axis

    // Both of these leave `valid` false; this says which. The two send the user
    // to different places -- "the marker is outside what the radio is sweeping,
    // retune" versus "that frequency is inside a dc_policy=blank hole, nothing
    // measured it" -- and a single "no reading" would hide the difference.
    bool unmeasured = false;

    std::string reason;          // why there is no number; empty when valid
};

// Reads marker `i` against `db[0..n)`. `ref_readout` may be null for a normal
// marker; a delta marker with an invalid or absent reference is itself invalid,
// because "delta from nothing" is a number with no meaning.
MarkerReadout read_marker(const Marker &m, const Marker *ref, const float *db, size_t n,
                          const TraceAxis &ax, size_t noise_avg_bins = 5);

// True for the detectors whose trace mean is the noise mean. §6's noise mode is
// arithmetically fine under any detector; it is only *correct* under these.
bool noise_detector_ok(sweepd::Detector d);

class MarkerSet {
public:
    static constexpr size_t kCount = 8;

    Marker &operator[](size_t i) { return m_[i < kCount ? i : 0]; }
    const Marker &operator[](size_t i) const { return m_[i < kCount ? i : 0]; }
    size_t size() const { return kCount; }

    size_t active() const { return active_; }
    void set_active(size_t i) { if (i < kCount) active_ = i; }
    Marker &current() { return m_[active_]; }
    const Marker &current() const { return m_[active_]; }

    bool any_on() const;
    void all_off();

    PeakOptions peak = PeakOptions();

private:
    Marker m_[kCount];
    size_t active_ = 0;
};

}  // namespace sweepui
