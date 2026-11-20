// Folding blocks of spectrum onto one output grid. CLAUDE.md §5, "Sweep loop"
// step 5 and "DC, LO leakage, images".
//
// This is the rule for turning N separate transforms, each taken at its own LO
// and each covering only the usable fraction of its sampled bandwidth, into a
// single monotonic trace. It lives on its own because two different radios in
// this tree now need it and they must not answer the question differently:
//
//   - sweepd's fastlock sweep, where the blocks are segments of a profile table
//     and the LO grid comes from §5's cold build (sweep.cpp);
//   - sweepui's HackRF source, where the blocks are the two 5 MHz sub-bands the
//     firmware's interleaved sweep hands back either side of each tuning
//     (hackrfsource.cpp).
//
// The geometry differs -- one hops by fastlock recall, the other by a firmware
// sweep over USB -- but "which block owns this output bin" is the same question
// with the same right answer, and a second reading of it is exactly the kind of
// duplicated rule this codebase keeps finding it has three copies of. The
// stitch is also where a mistake is invisible: a wrong answer here is not an
// error, it is a plausible-looking trace with a step in it.
//
// ------------------------------------------------------------------ the rule
//
// Blocks overlap almost everywhere under a splicing plan, so ownership is
// decided per output bin by:
//
//   1. prefer the contribution built from more FFT bins;
//   2. break ties by proximity to the contributing block's own LO.
//
// The count test is what covers the DC hole. Where a block blanks its own
// centre it contributes nothing there, so its neighbour -- farther from its LO,
// but with data -- wins by default, which is exactly the splice §5 asks for. It
// also handles the bin that straddles the edge of the blank, where the nearer
// block has only a fraction of the FFT bins and should not win with it.
//
// Proximity then decides the ordinary case, picking the block that sees this
// frequency closest to its own LO -- furthest from the analog filter's skirt.
// The crossover is a step in amplitude rather than a blend, because neither
// radio preserves amplitude between blocks (docs/characterization.md, "Not
// preserved: amplitude") and averaging two differently-wrong levels would only
// hide the seam that §7's segment-boundary overlay exists to show.
//
// ---------------------------------------------------------------- linear power
//
// Everything in and out of add() is LINEAR power, never dB. §5: "RMS averages
// power, then converts to dB. Averaging dB values is a different and wrong
// operation." The single conversion happens in to_db(), once, when the whole
// sweep is assembled.

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "dsp.h"

namespace sweepd {

// Where one block's spectrum may land, and where it must not. All absolute Hz.
struct StitchBand {
    // The block's centre: the frequency of FFT bin n_fft/2, since
    // Fft::power_spectrum returns fftshifted output with DC in the middle. Also
    // the reference for the proximity tie-break.
    double lo_hz = 0;

    // The usable fraction of the sampled bandwidth -- §5's "keep only the
    // usable fraction of each segment and overlap". Bins outside it are
    // discarded rather than merged, because they sit on the analog filter's
    // skirt.
    double keep_lo_hz = 0;
    double keep_hi_hz = 0;

    // The hole punched around the centre for LO leakage and the DC offset that
    // fastlock does not carry. `blank_hi_hz <= blank_lo_hz` means no hole,
    // which is what dc_policy=blank's opposite looks like and what a radio whose
    // block never contains its own LO wants.
    double blank_lo_hz = 0;
    double blank_hi_hz = 0;
};

class Stitcher {
public:
    // Sets the output grid and clears everything. `det` is the rule used to
    // combine several FFT bins landing in one output bin, which is a different
    // question from ownership above and is decided by §5's four detectors.
    void reset(double out_lo_hz, double out_hi_hz, size_t n_points, Detector det);

    // Folds one power spectrum in: `n_fft` fftshifted linear-power bins spaced
    // `fs_hz / n_fft`, with bin n_fft/2 at `b.lo_hz`. Bins outside the keep
    // band, inside the blank, or off the output grid are dropped.
    void add(const float *power, size_t n_fft, double fs_hz, const StitchBand &b);

    // Writes n_points dB values. Bins no block covered are NaN, not a floor
    // value: they are genuinely unmeasured, and §7 wants that visible rather
    // than rendered as a very quiet but real-looking reading.
    void to_db(float *out) const;

    // Drops the accumulated sweep and keeps the grid, for the next one.
    void clear();

    size_t n_points() const { return n_points_; }
    double bin_hz() const { return bin_hz_; }
    double out_lo_hz() const { return out_lo_; }

    // FFT bins that landed in output bin `i` from its winning block; 0 means
    // unmeasured. Exposed so a caller can tell "nothing was there" from
    // "nothing looked", which is the distinction §7 is built on.
    uint32_t count(size_t i) const { return grid_cnt_[i]; }

    // Output bins no block has covered since the last clear().
    size_t uncovered() const;

private:
    double out_lo_ = 0, out_hi_ = 0, bin_hz_ = 0;
    size_t n_points_ = 0;
    Detector det_ = Detector::Peak;

    // The winning contribution per output bin, and what won it.
    std::vector<float> grid_val_;
    std::vector<uint32_t> grid_cnt_;
    std::vector<long long> grid_dist_;

    // Scratch for the block currently being folded, kept as members so a sweep
    // of several hundred blocks does not allocate per block.
    std::vector<float> seg_val_;
    std::vector<double> seg_aux_;
    std::vector<uint32_t> seg_cnt_;
};

}  // namespace sweepd
