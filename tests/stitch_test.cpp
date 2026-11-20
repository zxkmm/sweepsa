// stitch.h: the rule that turns N transforms taken at N LOs into one trace.
//
// This is tested on its own because a mistake here is invisible. A wrong
// ownership decision is not an error, it is a plausible trace with a step in
// it, and a wrong band mapping is a plausible trace with everything half a
// segment out. Both would pass any "does it draw something" check.
//
// Two geometries are exercised, because two radios in this tree now use it:
//
//   - sweepd's, where each segment keeps a band centred on its own LO and
//     punches a hole at the centre for LO leakage (dc_policy=splice);
//   - the HackRF's, where each tuning keeps two quarter-bands either side of
//     its LO and the interleaved partner tuning fills the gap between them.
//
// The HackRF case is the one that has to tile exactly: if the two kept slices
// and the partner's do not add up, a full sweep grows NaN holes or double-counts
// a band, and hackrf_probe's "uncovered" count is the only thing that would
// ever say so.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "stitch.h"

using namespace sweepd;

static int failures = 0;

static void check(bool ok, const char *what) {
    if (!ok) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

static void check_near(double got, double want, double tol, const char *what) {
    if (!(std::fabs(got - want) <= tol)) {
        std::fprintf(stderr, "FAIL: %s: got %g, want %g (tol %g)\n", what, got, want, tol);
        ++failures;
    }
}

// A flat power spectrum whose value identifies which block it came from, so the
// assembled trace says who owns each bin.
static std::vector<float> flat(size_t n, float v) { return std::vector<float>(n, v); }

// ------------------------------------------------------- the HackRF geometry
//
// A block whose header reads F was tuned to F + 3fs/8 and keeps
// [F, F+fs/4) and [F+fs/2, F+3fs/4). Successive blocks are fs/4 apart within a
// tuning pair and fs apart between pairs -- which is what the firmware was
// measured to produce (docs/hackrf.md).
static StitchBand hackrf_band(double f, double fs) {
    StitchBand b;
    b.lo_hz = f + fs * 3.0 / 8.0;
    b.keep_lo_hz = f;
    b.keep_hi_hz = f + fs * 3.0 / 4.0;
    b.blank_lo_hz = f + fs / 4.0;
    b.blank_hi_hz = f + fs / 2.0;
    return b;
}

static void test_hackrf_tiles_without_holes() {
    const double fs = 20e6;
    const size_t n = 1024;

    // One tuning step: blocks at a and a + fs/4, as the firmware interleaves
    // them. Together they must cover [a, a + fs) with nothing missing.
    const double a = 100e6;
    const double out_lo = a, out_hi = a + fs;

    Stitcher s;
    s.reset(out_lo, out_hi, 400, Detector::Peak);
    s.add(flat(n, 1.0f).data(), n, fs, hackrf_band(a, fs));
    s.add(flat(n, 1.0f).data(), n, fs, hackrf_band(a + fs / 4.0, fs));

    // The last output bin ends exactly at a + fs, which the kept bands reach
    // only at their closed edge; allow that one bin.
    check(s.uncovered() <= 1, "one interleaved tuning pair covers its whole step");

    std::vector<float> db(s.n_points());
    s.to_db(db.data());
    size_t nan = 0;
    for (size_t i = 0; i + 1 < db.size(); ++i)
        if (std::isnan(db[i])) ++nan;
    check(nan == 0, "no NaN bins inside a covered step");
}

static void test_hackrf_places_bands_where_the_header_says() {
    const double fs = 20e6;
    const size_t n = 1024;
    const double a = 100e6;

    // Two blocks carrying distinguishable levels. Block A keeps [100,105) and
    // [110,115); block B, the interleaved partner at 105, keeps [105,110) and
    // [115,120).
    Stitcher s;
    s.reset(a, a + fs, 400, Detector::Peak);
    s.add(flat(n, 100.0f).data(), n, fs, hackrf_band(a, fs));
    s.add(flat(n, 1.0f).data(), n, fs, hackrf_band(a + fs / 4.0, fs));

    std::vector<float> db(s.n_points());
    s.to_db(db.data());

    const double bin = s.bin_hz();
    auto at = [&](double f_hz) { return db[size_t((f_hz - a) / bin)]; };

    // 20*log10(100) = 40 dB, 20*log10(1) = 0 dB -- power_to_db is 10*log10 of
    // linear power, so 100 -> 20 dB and 1 -> 0 dB.
    check_near(at(102e6), 20.0, 0.01, "[100,105) MHz comes from the block at 100");
    check_near(at(107e6), 0.0, 0.01, "[105,110) MHz comes from the partner at 105");
    check_near(at(112e6), 20.0, 0.01, "[110,115) MHz comes from the block at 100");
    check_near(at(117e6), 0.0, 0.01, "[115,120) MHz comes from the partner at 105");
}

static void test_blank_is_filled_by_the_neighbour() {
    const double fs = 20e6;
    const size_t n = 1024;
    const double a = 100e6;

    // The LO of the block at 100 sits at 107.5 MHz, inside its own blanked
    // hole. Nothing it measured may appear there -- that is where its leakage
    // is -- so the value must come from the partner.
    Stitcher s;
    s.reset(a, a + fs, 400, Detector::Peak);
    s.add(flat(n, 100.0f).data(), n, fs, hackrf_band(a, fs));
    s.add(flat(n, 1.0f).data(), n, fs, hackrf_band(a + fs / 4.0, fs));

    std::vector<float> db(s.n_points());
    s.to_db(db.data());
    const double bin = s.bin_hz();
    check_near(db[size_t((107.5e6 - a) / bin)], 0.0, 0.01,
               "the LO's own frequency is filled from the partner tuning");
}

// -------------------------------------------------------- the sweepd geometry

static StitchBand sweepd_band(double lo, double keep, double blank) {
    StitchBand b;
    b.lo_hz = lo;
    b.keep_lo_hz = lo - keep / 2;
    b.keep_hi_hz = lo + keep / 2;
    b.blank_lo_hz = lo - blank;
    b.blank_hi_hz = lo + blank;
    return b;
}

static void test_proximity_breaks_the_overlap() {
    const double fs = 56e6;
    const size_t n = 1024;
    const double keep = 40e6, blank = 100e3;

    // Two segments 20 MHz apart, overlapping between them. The stitch hands
    // over at the midpoint of the two LOs.
    const double lo1 = 1000e6, lo2 = 1020e6;
    Stitcher s;
    s.reset(lo1 - keep / 2, lo2 + keep / 2, 600, Detector::Peak);
    s.add(flat(n, 100.0f).data(), n, fs, sweepd_band(lo1, keep, blank));
    s.add(flat(n, 1.0f).data(), n, fs, sweepd_band(lo2, keep, blank));

    std::vector<float> db(s.n_points());
    s.to_db(db.data());
    const double base = lo1 - keep / 2, bin = s.bin_hz();
    auto at = [&](double f) { return db[size_t((f - base) / bin)]; };

    check_near(at(1005e6), 20.0, 0.01, "below the midpoint the nearer LO owns the bin");
    check_near(at(1015e6), 0.0, 0.01, "above the midpoint the other LO owns it");
}

static void test_rms_averages_power_not_db() {
    // §5's trap. Two FFT bins landing in one output bin, one at power 100 and
    // one at power 0: the RMS detector must report 50 (17 dB), not the mean of
    // 20 dB and -inf.
    const double fs = 1e6;
    const size_t n = 8;
    std::vector<float> p(n, 0.0f);
    for (size_t i = 0; i < n; ++i) p[i] = (i % 2) ? 0.0f : 100.0f;

    Stitcher s;
    // One output bin spanning the whole transform.
    StitchBand b;
    b.lo_hz = 1e9;
    b.keep_lo_hz = 1e9 - fs;
    b.keep_hi_hz = 1e9 + fs;
    s.reset(1e9 - fs / 2, 1e9 + fs / 2, 1, Detector::RMS);
    s.add(p.data(), n, fs, b);

    std::vector<float> db(1);
    s.to_db(db.data());
    check_near(db[0], 10.0 * std::log10(50.0), 0.5,
               "RMS averages linear power, then converts once");
}

static void test_uncovered_bins_are_nan() {
    const double fs = 20e6;
    const size_t n = 256;
    // A grid twice as wide as the single block that covers it.
    Stitcher s;
    s.reset(100e6, 140e6, 400, Detector::Peak);
    s.add(flat(n, 1.0f).data(), n, fs, hackrf_band(100e6, fs));

    std::vector<float> db(400);
    s.to_db(db.data());
    size_t nan = 0;
    for (float v : db)
        if (std::isnan(v)) ++nan;
    check(nan > 0, "bins no block covered come back NaN");
    check(s.uncovered() == nan, "uncovered() agrees with the NaNs it produced");
}

int main() {
    test_hackrf_tiles_without_holes();
    test_hackrf_places_bands_where_the_header_says();
    test_blank_is_filled_by_the_neighbour();
    test_proximity_breaks_the_overlap();
    test_rms_averages_power_not_db();
    test_uncovered_bins_are_nan();

    if (failures) {
        std::fprintf(stderr, "%d failure(s)\n", failures);
        return 1;
    }
    std::printf("stitch: all checks passed\n");
    return 0;
}
