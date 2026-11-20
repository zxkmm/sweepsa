// §7's amplitude correction: load `(freq_hz, offset_db)` CSV, interpolate,
// apply.
//
// This is the feature that unlocks dBm. Until a correction is loaded the whole
// instrument reads dBFS and the UNCAL badge is lit (source.h, `uncal`), which
// is the §7 rule: "an uncalibrated analyzer that looks calibrated is worse than
// useless."
//
// ---------------------------------------------------------------- where it runs
//
// In `sweepui`, not in `sweepd`, and applied at the single point where a frame
// enters the display pipeline (`App::pump`). Three reasons, in order of weight:
//
//   1. **The device cannot hold the file.** §2: the E310's rootfs is a RAM
//      initramfs and is wiped on every boot. A per-installation calibration
//      that has to be re-uploaded every power cycle is not a calibration.
//   2. **The wire stays honest.** proto.h says the 0x02 payload is dBFS. If the
//      daemon applied a correction, the same frame type would carry dBFS
//      sometimes and dBm other times with nothing in the header to say which,
//      and `tools/sweep_client`'s CSV would silently change meaning.
//   3. One ingest point covers everything. The waterfall, all four traces, the
//      holds, the averages, the markers and the CSV export all read the
//      corrected array, so there is no second path to drift out of sync -- the
//      same argument zero span made for riding the 0x02 frame.
//
// ------------------------------------------------------------- adding dB is right
//
// This codebase is careful that averaging dB is wrong (§5's RMS rule, in three
// independent implementations). *Adding* dB is a different operation and it is
// correct: an offset in dB is a constant linear scale factor on power, so
// `db + offset` is exactly `10*log10(P * k)`. No linearise/re-log round trip is
// needed and none is done.
//
// -------------------------------------------------------- no extrapolation, ever
//
// The correction is defined only on `[f_min_hz(), f_max_hz()]` -- the range the
// file actually measured. Outside it `offset_at()` returns false and `apply()`
// leaves the bin untouched and counts it.
//
// Clamping to the nearest endpoint would have been easy and is the trap: a
// front end's response rolls off hardest exactly where the file stops, so a
// clamp is largest where it is least justified, and it produces a confident dBm
// number for a frequency nobody measured. §7 would rather show dBFS than a
// plausible fiction, so a span reaching outside the file's coverage keeps its
// UNCAL badge -- see `covers()`.
//
// ------------------------------------------------------------------- zero span
//
// A zero-span trace's x-axis is *time* (proto.h, "one trace format"), so
// indexing this table by the x value would look up a correction at "0.004 Hz"
// and find nothing, or worse, find something. Zero span has exactly one
// frequency -- the parked centre -- and `apply_flat()` is the entry point that
// takes it. `apply()` is for the frequency axis and says so.
//
// ------------------------------------------- not every correction is a dBm one
//
// §7 says "load `(freq_hz, offset_db)` CSV" and says nothing about where the
// numbers came from. `tools/ampcal` can produce two materially different files
// from the same two columns:
//
//   - **absolute** -- each offset was referenced to a *known* level at the RX
//     input, so `dBFS + offset` really is dBm. This is the file §7 means, and
//     it needs an external reference (a VNA, a calibrated generator, a source
//     whose output level is known at each frequency).
//   - **relative** -- the reference level was not known. A TX->RX loopback on
//     this board measures the *round trip*: the transmitter's frequency
//     response, the cable, and the receiver's, multiplied together. Dividing
//     that out flattens the trace, which is genuinely useful, but the result is
//     a shape with an arbitrary constant in it, not a power.
//
// Applying a relative file and then labelling the axis "dBm" would be the §7
// failure in its purest form: a confident absolute number nobody measured. So
// the kind travels with the file, in a comment the loader reads:
//
//     # sweepsa-ampcorr: relative
//     # sweepsa-ampcorr-gain-db: 30
//     # sweepsa-ampcorr-span-hz: 70000000 6000000000
//     # sweepsa-ampcorr-adc-hz: 640000000
//     # sweepsa-ampcorr-detector: peak
//     # sweepsa-ampcorr-note: free text, shown in the UI
//
// A `#` comment because every other reader of this format -- a spreadsheet, a
// plotting script, an older build of this program -- still sees exactly two
// columns and is unaffected. **A file with no directive is read as absolute**,
// which is what a hand-written §7 file means and keeps every existing file
// working; `ampcal` always writes the line explicitly so its own output is
// never read by that fallback.
//
// The three other directives are conditions, not units, and they exist because
// a correction is only valid under the conditions it was measured under:
//
//   - **RX gain.** The whole curve shifts with it. Nothing in the frame header
//     made this checkable before; `gain_db` is on the 0x02 frame, so it is now.
//   - **Span, and therefore hop order.** docs/characterization.md: a recalled
//     LO's amplitude depends on the *previous* LO, up to 6.3 dB, and the fix on
//     record is "the offset becomes a fixed per-segment constant that the
//     per-installation amplitude correction absorbs". That only holds while
//     every segment keeps the same predecessor -- i.e. while the span and the
//     segment count are the ones the correction was measured on.
//   - **Detector.** A peak-detected bin and an RMS-detected bin differ by a
//     decimation-dependent amount (marker.cpp says the same thing about noise
//     markers), so an offset measured under one does not transfer to the other.
//
// None of the three is enforced -- a refusal would be worse than the error it
// prevents, because the operator may have a good reason. They are reported.

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace sweepui {

class AmpCorrection {
public:
    struct Point {
        double freq_hz = 0;
        double offset_db = 0;
    };

    // What the offsets mean. See the header comment; `Absolute` is the fallback
    // for a file that does not say, because that is what §7's own wording
    // describes and what a hand-written file intends.
    enum class Kind { Absolute, Relative };

    // The conditions the file records about its own measurement. Every field is
    // optional and absent is represented rather than defaulted, because "no
    // gain was recorded" and "it was measured at 0 dB" are different facts and
    // only one of them should produce a warning.
    struct Conditions {
        bool has_gain_db = false;
        double gain_db = 0;

        bool has_span = false;
        double span_lo_hz = 0;
        double span_hi_hz = 0;

        std::string detector;   // empty if not recorded

        // The ADC clock it was measured at. The rate governor picks it, and
        // the signal path's gain differs by ~0.5 dB between clocks
        // (docs/characterization.md, "The pedestal follows the ADC clock").
        bool has_adc_hz = false;
        double adc_hz = 0;
        std::string note;       // free text, shown verbatim
    };

    // ------------------------------------------------------------- loading
    //
    // The file format is §7's: two columns, `freq_hz,offset_db`. Blank lines and
    // `#` comments are skipped, a UTF-8 BOM and CRLF line endings are tolerated,
    // and either a comma or whitespace separates the columns. A header row whose
    // first field is not a number is skipped once, so a file exported from a
    // spreadsheet loads.
    //
    // It is a *file*, so it is parsed defensively and refused loudly rather than
    // half-understood -- the same rule the instrument state file follows. On
    // failure the previous correction is left untouched: a bad file must not
    // quietly de-calibrate a working session.
    bool load_csv(const std::string &path, std::string *err);

    // The same parser over an in-memory string. This is what the tests drive,
    // so what they cover is the real one.
    bool load_csv_text(const std::string &text, const std::string &name,
                       std::string *err);

    void clear();

    bool loaded() const { return pts_.size() >= 2; }
    size_t size() const { return pts_.size(); }
    const std::string &name() const { return name_; }

    // The measured range. Meaningless unless `loaded()`.
    double f_min_hz() const { return pts_.empty() ? 0 : pts_.front().freq_hz; }
    double f_max_hz() const { return pts_.empty() ? 0 : pts_.back().freq_hz; }

    const std::vector<Point> &points() const { return pts_; }

    // What the numbers mean, and under what conditions they were taken.
    Kind kind() const { return kind_; }
    bool absolute() const { return kind_ == Kind::Absolute; }
    const Conditions &conditions() const { return cond_; }

    // One line naming the kind, for a status bar or a log. Never empty.
    const char *kind_name() const;

    // ---------------------------------------------------------- interpolation

    // Linear interpolation in dB against frequency. False outside the measured
    // range -- see the header comment; this never extrapolates.
    bool offset_at(double freq_hz, double *out_db) const;

    // Is every frequency in [f_lo, f_hi] covered? This is what decides whether
    // the units may read dBm: a span that runs off the end of the correction is
    // partly corrected and partly not, and there is no honest single label for
    // that trace, so it stays dBFS.
    bool covers(double f_lo, double f_hi) const;

    // ------------------------------------------------------------------ apply

    // Frequency axis. Bin `i` is centred at `f_start_hz + (i + 0.5) * bin_hz`,
    // which is the convention export.cpp already writes and marker.cpp already
    // reads. NaN bins stay NaN (they were never measured, and an offset would
    // give them a value). Returns the number of bins left uncorrected because
    // they fall outside the file's range.
    size_t apply(float *db, size_t n, double f_start_hz, double bin_hz) const;

    // Zero span: one frequency for the whole trace. Returns false if the parked
    // centre is outside the correction, in which case nothing is modified --
    // a partially corrected zero-span trace is not a thing that can happen, so
    // it is all or nothing.
    bool apply_flat(float *db, size_t n, double center_hz) const;

private:
    // Sorted by frequency, strictly increasing, at least 2 entries when loaded.
    std::vector<Point> pts_;
    std::string name_;
    Kind kind_ = Kind::Absolute;
    Conditions cond_;
};

}  // namespace sweepui
