// Host-side tests for the display decimator. CLAUDE.md §6, "Performance".
//
// Every failure this file guards against produces a picture rather than an
// error: a spur that disappears at one window width and returns at another, a
// blanked bin drawn as a signal at -200 dB, an envelope that slides sideways as
// you zoom. None of them look like bugs on screen, so they are pinned here.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "../src/sweepui/decimate.h"

using namespace sweepui;

namespace {

int g_failures = 0;
const float kNan = std::numeric_limits<float>::quiet_NaN();

void check(const char *what, bool ok) {
    std::printf("  %-64s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) ++g_failures;
}

void check_near(const char *what, double got, double want, double tol) {
    const bool ok = std::fabs(got - want) <= tol;
    std::printf("  %-64s got=%-10.4f want=%-10.4f %s\n", what, got, want,
                ok ? "ok" : "FAIL");
    if (!ok) ++g_failures;
}

// A flat floor with one single-bin spur. This is the case the whole design
// exists for: 40 bins per pixel column, and the spur must survive.
void test_single_bin_spur_survives() {
    std::printf("a one-bin spur survives 40:1 decimation (this is the point)\n");
    const size_t n = 32768;
    std::vector<float> db(n, -100.0f);
    const size_t spur = 12345;
    db[spur] = -20.0f;

    const double f0 = 100e6, bin = 1e3;
    DecimatedTrace d;
    decimate_minmax(db.data(), n, f0, bin, f0, f0 + double(n) * bin, 800, &d);

    check("output is no wider than the pixel budget", d.size() <= 800);
    double hi = -1e9, lo = 1e9;
    for (size_t i = 0; i < d.size(); ++i) {
        hi = std::max(hi, d.hi[i]);
        lo = std::min(lo, d.lo[i]);
    }
    check_near("the spur is still the maximum", hi, -20.0, 1e-6);
    check_near("the floor is still the minimum", lo, -100.0, 1e-6);

    // And it must be in the right column, not merely present somewhere.
    size_t peak = 0;
    for (size_t i = 1; i < d.size(); ++i)
        if (d.hi[i] > d.hi[peak]) peak = i;
    const double f_spur = f0 + (double(spur) + 0.5) * bin;
    const double col_w = double(n) * bin / double(d.size());
    check_near("and in the right column", d.x[peak], f_spur, col_w);
}

// §7: bins no segment covered are unmeasured. A column that is entirely
// unmeasured must stay unmeasured; a column that merely contains a hole must
// show the signal that shares it.
void test_nan_policy() {
    std::printf("NaN holes: propagated when total, ignored when partial (§7)\n");
    const size_t n = 1000;
    std::vector<float> db(n, -90.0f);
    for (size_t i = 100; i < 200; ++i) db[i] = kNan;   // a wide hole

    DecimatedTrace d;
    decimate_minmax(db.data(), n, 0.0, 1.0, 0.0, double(n), 10, &d);
    // 100 bins per column: column 1 is entirely inside the hole.
    check("a fully unmeasured column is NaN in both min and max",
          std::isnan(d.lo[1]) && std::isnan(d.hi[1]));
    check("a measured column is not", !std::isnan(d.lo[0]) && !std::isnan(d.hi[0]));

    // Now a hole narrower than a column, sharing it with a signal.
    std::vector<float> db2(n, -90.0f);
    db2[550] = kNan;
    db2[551] = -30.0f;
    decimate_minmax(db2.data(), n, 0.0, 1.0, 0.0, double(n), 10, &d);
    check_near("a partial hole does not hide the signal beside it", d.hi[5], -30.0, 1e-6);
    check("and does not turn the column into a hole", !std::isnan(d.lo[5]));
}

// Zooming in must actually increase resolution, not stretch the same columns.
void test_zoom_increases_resolution() {
    std::printf("zooming reduces the decimation factor rather than stretching\n");
    const size_t n = 4096;
    std::vector<float> db(n);
    for (size_t i = 0; i < n; ++i) db[i] = float(-100 + (i % 7));

    DecimatedTrace full, zoom;
    decimate_minmax(db.data(), n, 0.0, 1.0, 0.0, double(n), 256, &full);
    decimate_minmax(db.data(), n, 0.0, 1.0, 1000.0, 1100.0, 256, &zoom);

    check("full view is capped by the pixel budget", full.size() == 256);
    check("a 100-bin view yields one column per bin, not 256 stretched ones",
          zoom.size() == 100);
    bool exact = true;
    for (size_t i = 0; i < zoom.size(); ++i)
        if (zoom.lo[i] != zoom.hi[i]) exact = false;
    check("and at 1:1 the min and max are the same sample", exact);
}

// The x of a column must be inside the column. A half-bin or half-column offset
// here reads as a calibration error later, when a marker is placed by x.
void test_x_alignment() {
    std::printf("column x stays inside the column it summarises\n");
    const size_t n = 1000;
    std::vector<float> db(n, -80.0f);
    DecimatedTrace d;
    const double f0 = 1e9, bin = 25e3;
    decimate_minmax(db.data(), n, f0, bin, f0, f0 + double(n) * bin, 40, &d);

    bool ok = d.size() == 40;
    for (size_t c = 0; c < d.size() && ok; ++c) {
        const double left = f0 + double(25 * c) * bin;
        const double right = left + 25.0 * bin;
        if (!(d.x[c] >= left && d.x[c] < right)) ok = false;
    }
    check("every x is within its own column's frequency range", ok);
    check("columns are monotonic in frequency",
          std::is_sorted(d.x.begin(), d.x.end()));
}

void test_degenerate_inputs() {
    std::printf("degenerate input produces nothing rather than something wrong\n");
    DecimatedTrace d;
    std::vector<float> db(10, -50.0f);
    decimate_minmax(nullptr, 0, 0, 1, 0, 10, 100, &d);
    check("null input", d.size() == 0);
    decimate_minmax(db.data(), db.size(), 0, 1, 5, 5, 100, &d);
    check("zero-width view", d.size() == 0);
    decimate_minmax(db.data(), db.size(), 0, 1, 100, 200, 100, &d);
    check("view entirely outside the trace", d.size() == 0);
    decimate_minmax(db.data(), db.size(), 0, 0, 0, 10, 100, &d);
    check("zero bin width", d.size() == 0);
}

// The waterfall's row reduction. Peak, not min/max: one pixel row can only show
// one value, and hiding a spur to show the floor beside it is the wrong choice
// for a spectrum analyzer.
void test_peak_rows() {
    std::printf("waterfall rows: peak-decimated to a fixed width\n");
    const size_t n = 4000;
    std::vector<float> db(n, -95.0f);
    db[1999] = -10.0f;
    for (size_t i = 3000; i < 3100; ++i) db[i] = kNan;

    std::vector<float> row(500, 0.0f);
    decimate_peak(db.data(), n, row.data(), row.size());

    float hi = -1e9;
    size_t peak = 0;
    for (size_t i = 0; i < row.size(); ++i)
        if (!std::isnan(row[i]) && row[i] > hi) {
            hi = row[i];
            peak = i;
        }
    check_near("the spur survives", double(hi), -10.0, 1e-6);
    check("in the right column", peak == 1999 / 8);
    check("a fully blanked column stays NaN", std::isnan(row[3000 / 8 + 1]));

    // Upsampling: more columns than bins. Every column must carry a value.
    std::vector<float> small(10, -70.0f);
    std::vector<float> wide(100, 0.0f);
    decimate_peak(small.data(), small.size(), wide.data(), wide.size());
    bool all_set = true;
    for (float v : wide)
        if (std::isnan(v)) all_set = false;
    check("upsampling leaves no empty columns", all_set);
}

}  // namespace

int main() {
    test_single_bin_spur_survives();
    test_nan_policy();
    test_zoom_increases_resolution();
    test_x_alignment();
    test_degenerate_inputs();
    test_peak_rows();
    std::printf("%s\n", g_failures ? "FAILURES" : "all ok");
    return g_failures ? 1 : 0;
}
