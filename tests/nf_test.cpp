// Host tests for the Y-factor noise figure. The readings are synthesised from
// known temperatures, so the method must recover the NFs and the gain they
// were built from exactly.

#include <cmath>
#include <cstdio>
#include <string>

#include "nf.h"

using namespace sweepui;

namespace {

int g_failures = 0;
constexpr double kT0 = 290.0;

void check_true(const char *what, bool ok) {
    std::printf("  %-62s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) ++g_failures;
}

void check_near(const char *what, double got, double want, double tol) {
    const bool ok = std::fabs(got - want) <= tol;
    std::printf("  %-62s %s (got %.4f, want %.4f +/-%.4f)\n", what, ok ? "ok" : "FAIL",
                got, want, tol);
    if (!ok) ++g_failures;
}

double lin(double db) { return std::pow(10.0, db / 10.0); }

// Noise power (in kTB units) at the receiver's detector: input temperature
// through a stage of gain g and noise factor f, then the receiver (f2, its own
// gain folded into the arbitrary unit).
double reading(double t_in, double g, double f, double f2) {
    const double te1 = (f - 1.0) * kT0, te2 = (f2 - 1.0) * kT0;
    return (t_in + te1) * g + te2;
}

void test_recovers_known_values() {
    std::printf("y-factor\n");
    const double enr = 15.0;                           // an HP346B-ish source
    const double t_hot = kT0 + kT0 * lin(enr), t_cold = kT0;
    const double f2 = lin(6.0);                         // the receiver: 6 dB NF
    const double g1 = lin(20.0), f1 = lin(1.5);         // the DUT: 20 dB, 1.5 dB NF

    const NfResult rx = nf_receiver(enr, reading(t_hot, 1, 1, f2), reading(t_cold, 1, 1, f2));
    check_true("receiver alone is valid", rx.valid);
    check_near("receiver NF recovered", rx.nf2_db, 6.0, 1e-9);

    NfReadings x;
    x.cal_on = reading(t_hot, 1, 1, f2);
    x.cal_off = reading(t_cold, 1, 1, f2);
    x.dut_on = reading(t_hot, g1, f1, f2);
    x.dut_off = reading(t_cold, g1, f1, f2);
    const NfResult r = nf_dut(enr, x);
    check_true("DUT measurement valid", r.valid);
    check_near("DUT gain recovered", r.gain_db, 20.0, 1e-9);
    check_near("DUT NF recovered, receiver removed", r.nf1_db, 1.5, 1e-9);
    check_true("the cascade is worse than the DUT alone", r.nf12_db > r.nf1_db);
    check_true("20 dB of gain is not low", !r.low_gain);

    // A 3 dB attenuator: NF = 3 dB, gain -3 dB, and it is flagged.
    x.dut_on = reading(t_hot, lin(-3), lin(3), f2);
    x.dut_off = reading(t_cold, lin(-3), lin(3), f2);
    const NfResult a = nf_dut(enr, x);
    check_near("a 3 dB pad reads NF 3 dB", a.nf1_db, 3.0, 1e-9);
    check_true("and is flagged low-gain", a.low_gain);
}

void test_refusals_and_enr() {
    std::printf("refusals and ENR table\n");
    check_true("Y = 1 (source not switching) is refused",
               !nf_receiver(15, 1000, 1000).valid);
    check_true("Y = 0.1 dB (scatter, not a source) is refused, not read as NF ~32 dB",
               !nf_receiver(15, 1000 * std::pow(10.0, 0.01), 1000).valid);
    EnrTable t;
    std::string err;
    check_true("loads a label-style table with a header",
               t.load_text("freq_hz,enr_db\n10m,15.2\n1g,15.0\n# comment\n2g ; 14.6\n", &err));
    double e = 0;
    check_true("interpolates", t.enr_db(1.5e9, &e) && std::fabs(e - 14.8) < 1e-9);
    check_true("refuses outside the table (no extrapolated calibration)", !t.enr_db(3e9, &e));
}

}  // namespace

int main() {
    std::printf("noise figure tests\n\n");
    test_recovers_known_values();
    std::printf("\n");
    test_refusals_and_enr();
    std::printf("\n%s\n", g_failures ? "FAILURES" : "all passed");
    return g_failures ? 1 : 0;
}
