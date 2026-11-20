// Band measurements over a trace: channel power, power spectral density and
// occupied bandwidth. SATSAGEN's "Cursors CP/PSD", and the things an analyzer's
// MEAS key does.
//
// ------------------------------------------------------------- integration
//
// Each trace point is the power the analyzer saw in one resolution bandwidth
// centred on it. Points sit `dx` apart and each RBW is `nbw` wide, so a point
// stands for dx / nbw of an RBW's worth of the band, and channel power is
//
//     P = sum over points in the band of 10^(p_i/10) * dx / nbw
//
// in LINEAR power, converted to dB once (CLAUDE.md §5, the same rule as RMS).
// nbw is the noise bandwidth -- ENBW x fs / N, which is what sweepd reports as
// rbw_hz -- not the -3 dB width: a flat noise floor integrates to its true
// power only against the noise bandwidth.
//
// Correct under the Sample and RMS detectors, whose point is the mean power in
// the cell. Peak reads high on noise by an amount that depends on how many
// bins each point took the maximum of, so the result carries a warning rather
// than being silently wrong; Negative Peak reads low the same way.
//
// ------------------------------------------------------------- partial points
//
// A band edge falls inside a point. That point contributes the fraction of its
// width inside the band, so moving an edge by a fraction of a point moves the
// answer smoothly instead of in steps of one point.

#pragma once

#include <cstddef>
#include <string>

#include "dsp.h"
#include "marker.h"

namespace sweepui {

struct BandResult {
    bool valid = false;
    std::string reason;          // why not, when !valid
    double power_db = 0;         // channel power, in the trace's units
    double psd_db_hz = 0;        // power / band width, per Hz
    double width_hz = 0;         // band actually integrated (clipped to the trace)
    size_t n_points = 0;
    size_t n_nan = 0;            // unmeasured points inside the band, skipped
    std::string warning;         // detector caveat; empty when none
};

BandResult channel_power(const float *db, size_t n, const TraceAxis &ax, double f_lo,
                         double f_hi, sweepd::Detector det);

// Occupied bandwidth: the narrowest-by-construction band holding `frac` of the
// power in [f_lo, f_hi] -- the edges are where the cumulative power crosses
// (1-frac)/2 and (1+frac)/2, as the ITU and FCC define it. 0.99 is the usual.
struct ObwResult {
    bool valid = false;
    std::string reason;
    double f_lo = 0, f_hi = 0;   // the occupied band's edges
    double width_hz = 0;
    double total_db = 0;         // power of the band it was taken within
};

ObwResult occupied_bw(const float *db, size_t n, const TraceAxis &ax, double f_lo,
                      double f_hi, double frac = 0.99);

}  // namespace sweepui
