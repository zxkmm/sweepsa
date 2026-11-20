// Thru normalization for the tracking generator: what turns a received tone
// level into |S21|.
//
// The tracking trace on its own is "how loud did each tone arrive", which is
// the stimulus level times the board's own TX and RX response times the DUT.
// The first two are large and anything but flat -- 50 dB of roll-off across
// 70 MHz - 6 GHz on the E310, plus a 14 dB step where an LO step crosses the
// 3 GHz band switch (sweepd/track.h). Storing a sweep with a thru in place of
// the DUT and subtracting it leaves the DUT alone. This is SATSAGEN's
// "Calib. Req. 0dB".
//
// ------------------------------------------------------------- the grid rule
//
// A reference is only applied to a frame on exactly the grid it was stored
// on: same start, same spacing, same point count, same tones per LO step. The
// last one is not pedantry. The band-switch step belongs to the LO step that
// straddles the crossover, so a different step layout puts it at a different
// frequency, and a reference from the old layout would *add* a 14 dB artifact
// rather than remove one. Refused, with the reason, rather than applied.
//
// ---------------------------------------------------- what may differ, and how
//
//   TX attenuation  compensated: S21 = meas - ref + (atten_now - atten_ref).
//                   This is what lets a thru be stored at a safe level and a
//                   lossy DUT be measured with more drive -- SATSAGEN's
//                   "Calib. Req. -40dB". Said on screen, because the
//                   attenuator's own step accuracy is now in the reading.
//   RX gain         refused. The gain table is not linear to the 0.1 dB a
//                   thru is good for, and nothing on screen would show it.
//
// Averaging is linear power, never dB (CLAUDE.md §5), over the sweeps asked
// for. Below 500 MHz each LO step lands within ~0.5 dB of its last level as a
// block (docs/characterization.md, "tracking"), so a one-sweep thru carries
// that error into every later measurement; the default is several.

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace sweepui {

struct TrackGrid {
    double f_start_hz = 0;
    double bin_hz = 0;
    size_t n = 0;
    size_t tones = 0;      // per LO step
    double atten_db = 0;
    double gain_db = 0;
    // The RX mapping (sweepd/trackplan.h). Not part of the layout: a mixer or a
    // multiplier is normally normalized against a thru taken WITHOUT it, at
    // harmonic 1 and no offset -- there is no other thru to take. Applied, with
    // the caveat said, because the receiver's response at the output frequency
    // is then not divided out.
    int harmonic = 1;
    double rx_offset_hz = 0;
    bool rx_invert = false;
};

class TrackNorm {
public:
    // Average the next `sweeps` frames into a new reference. Drops any
    // reference already held: a half-replaced one would be the worst kind.
    void start_capture(size_t sweeps);
    void cancel_capture();
    void clear();

    // A raw frame, before normalization. True when this one completed a
    // capture. A frame on a different grid restarts the capture, and *why*
    // explains it.
    bool feed(const TrackGrid &g, const float *db, size_t n, std::string *why = nullptr);

    // Subtracts the reference in place. Returns "" when applied; otherwise the
    // reason it was not, and `db` is untouched. *atten_comp_db gets the
    // compensation that was added (0 if the attenuation matched).
    std::string apply(const TrackGrid &g, float *db, size_t n, double *atten_comp_db,
                      std::string *caveat = nullptr) const;

    bool have() const { return !ref_.empty(); }
    bool capturing() const { return want_ > 0; }
    size_t captured() const { return got_; }
    size_t target() const { return want_; }
    const TrackGrid &grid() const { return ref_grid_; }
    const std::vector<float> &ref_db() const { return ref_; }

    // The weakest point of the reference, which bounds the dynamic range a
    // normalized trace can show there.
    float ref_min_db() const;

private:
    static bool same_layout(const TrackGrid &a, const TrackGrid &b);

    std::vector<float> ref_;
    TrackGrid ref_grid_;

    size_t want_ = 0, got_ = 0;
    std::vector<double> acc_;
    TrackGrid acc_grid_;
};

}  // namespace sweepui
