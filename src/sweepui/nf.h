// Noise figure and gain by the Y-factor method. SATSAGEN's "NF/G Analyzer".
//
// A calibrated noise source (HP346A/B/C or similar) switches between two known
// noise temperatures: off (ambient, taken as T0 = 290 K) and on, which adds
// its Excess Noise Ratio, ENR, printed per frequency on its calibration label.
// The receiver measures noise power both ways, and the ratio Y = N_on / N_off
// is all the method needs -- an *uncalibrated* receiver is fine, because it is
// a ratio of two readings taken through the same gain.
//
//     F = ENR / (Y - 1)                    (ENR and Y linear, T_cold = T0)
//
// --------------------------------------------------------------- two stages
//
// Measured directly, that is the receiver's own noise factor, F2. With a DUT
// in front, the same measurement gives the cascade F12, and the DUT's gain is
// how much it raised the hot-minus-cold difference:
//
//     G1 = (N_on,dut - N_off,dut) / (N_on,cal - N_off,cal)
//     F1 = F12 - (F2 - 1) / G1             (Friis, solved for the first stage)
//
// So a measurement is four noise readings per frequency: cal on/off, DUT
// on/off. The second-stage correction is what makes the answer the DUT's and
// not the DUT's-plus-this-receiver's, and it is only as good as G1 is large:
// with a low-gain DUT, F12 and the correction are close and their difference
// is noisy. The result says so (`low_gain`).
//
// --------------------------------------------------------------- what is not
//
// ENR is the source's *calibrated* value at each frequency -- a typical-model
// curve is not a calibration and the readout must not pretend otherwise.
// T_cold is assumed to be T0; at 300 K instead of 290 K the error is ~0.1 dB
// for a low-ENR source and is ignored, as most instruments' defaults do.

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace sweepui {

class EnrTable {
public:
    // "freq_hz,enr_db" per line; '#' comments; frequency suffixes accepted.
    bool load_csv(const std::string &path, std::string *err);
    bool load_text(const std::string &text, std::string *err);
    void clear() { f_.clear(); enr_.clear(); }
    bool loaded() const { return !f_.empty(); }
    size_t size() const { return f_.size(); }
    double f_min() const { return f_.empty() ? 0 : f_.front(); }
    double f_max() const { return f_.empty() ? 0 : f_.back(); }
    // Linear in frequency between table points; refuses outside the table
    // rather than extrapolating a calibration that was never made.
    bool enr_db(double f_hz, double *out) const;

private:
    std::vector<double> f_, enr_;
};

// The four readings at one frequency, linear power (any common unit).
struct NfReadings {
    double cal_on = 0, cal_off = 0;   // source straight into the receiver
    double dut_on = 0, dut_off = 0;   // source -> DUT -> receiver
};

struct NfResult {
    bool valid = false;
    std::string reason;
    double nf2_db = 0;      // the receiver's own noise figure
    double nf12_db = 0;     // the cascade, DUT + receiver
    double gain_db = 0;     // DUT gain
    double nf1_db = 0;      // the DUT, second stage removed
    bool low_gain = false;  // G1 < 10 dB: the correction dominates the answer
};

// Receiver only: from the cal pair.
NfResult nf_receiver(double enr_db, double on, double off);

// The whole measurement.
NfResult nf_dut(double enr_db, const NfReadings &r);

}  // namespace sweepui
