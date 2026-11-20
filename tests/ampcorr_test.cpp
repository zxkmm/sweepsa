// Host-side tests for §7's amplitude correction.
//
// Three traps are pinned here, and all three produce a *plausible number*
// rather than an error when got wrong, which is the whole reason §7 exists:
//
//   1. **Extrapolation.** Clamping to the nearest endpoint outside the measured
//      range is one line and it is wrong: a front end rolls off hardest exactly
//      where the file stops, so the clamp is largest where it is least
//      justified. `offset_at` must refuse, and `covers()` must be what decides
//      whether the units may say dBm at all.
//   2. **Zero span indexing by the x axis.** A zero-span trace's x is *time*,
//      so `apply()` on it would look up a correction at "0.004 Hz". There is a
//      separate `apply_flat()` taking the parked centre, and it is all-or-
//      nothing.
//   3. **NaN.** An unmeasured bin plus an offset must not become a measurement.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "../src/sweepui/ampcorr.h"

using namespace sweepui;

namespace {

int g_failures = 0;

void ok(const char *what, bool pass) {
    std::printf("  %-66s %s\n", what, pass ? "ok" : "FAIL");
    if (!pass) ++g_failures;
}

void near(const char *what, double got, double want, double tol = 1e-9) {
    const bool pass = std::fabs(got - want) <= tol;
    std::printf("  %-46s %12.6f want %12.6f %s\n", what, got, want, pass ? "ok" : "FAIL");
    if (!pass) ++g_failures;
}

AmpCorrection load(const std::string &text) {
    AmpCorrection c;
    std::string err;
    if (!c.load_csv_text(text, "test", &err))
        std::printf("    (load refused: %s)\n", err.c_str());
    return c;
}

// ------------------------------------------------------------------ parsing

void test_parse_accepts() {
    std::printf("parsing: what a real file looks like\n");

    AmpCorrection c = load("# generated against a reference, 2026-09-11\n"
                           "freq_hz,offset_db\n"
                           "\n"
                           "1000000000,-3.5\r\n"
                           "2000000000,-5.0\n");
    ok("comments, a header row, a blank line and CRLF all load", c.loaded());
    ok("and yield 2 points", c.size() == 2);
    near("f_min", c.f_min_hz(), 1e9);
    near("f_max", c.f_max_hz(), 2e9);

    ok("whitespace-separated columns load",
       load("1e9 -3.5\n2e9 -5.0\n").loaded());
    ok("extra columns beyond the first two are ignored",
       load("1e9,-3.5,measured\n2e9,-5.0,measured\n").loaded());

    // Order is the operator's business, not the format's.
    AmpCorrection u = load("2e9,-5.0\n1e9,-3.5\n1.5e9,-4.0\n");
    ok("an unsorted file is sorted on load", u.loaded() && u.size() == 3);
    near("  sorted f_min", u.f_min_hz(), 1e9);
    near("  sorted f_max", u.f_max_hz(), 2e9);

    ok("an exactly duplicated row collapses",
       load("1e9,-3.5\n1e9,-3.5\n2e9,-5.0\n").size() == 2);
}

void test_parse_refuses() {
    std::printf("parsing: what must be refused rather than half-understood\n");

    AmpCorrection c;
    std::string err;

    ok("one point is refused (it is an offset, not a curve)",
       !c.load_csv_text("1e9,-3.5\n", "t", &err));
    ok("empty is refused", !c.load_csv_text("", "t", &err));
    ok("comments only is refused", !c.load_csv_text("# nothing here\n", "t", &err));

    // The column-swap trap: (-3.5, 1e9) parses as two perfectly good numbers.
    // Only the sign of the frequency catches it.
    ok("swapped columns are refused, not loaded as negative frequencies",
       !c.load_csv_text("-3.5,1e9\n-5.0,2e9\n", "t", &err));
    ok("  and the message says so",
       err.find("swapped") != std::string::npos);

    ok("contradictory duplicates are refused",
       !c.load_csv_text("1e9,-3.5\n1e9,-9.9\n2e9,-5.0\n", "t", &err));
    ok("a trailing junk line is refused, not silently truncated to a number",
       !c.load_csv_text("1e9,-3.5\n2e9,-5.0\n3e9abc,-6.0\n", "t", &err));
    ok("a second header-looking row is refused",
       !c.load_csv_text("f,o\n1e9,-3.5\nf,o\n2e9,-5.0\n", "t", &err));

    // A bad file must not de-calibrate a working session.
    AmpCorrection good = load("1e9,-3.5\n2e9,-5.0\n");
    ok("a good correction is loaded", good.loaded());
    ok("loading a bad file over it is refused",
       !good.load_csv_text("garbage\n", "bad", &err));
    ok("  and the previous correction survives intact",
       good.loaded() && good.size() == 2 && good.f_max_hz() == 2e9);
}

// ---------------------------------------------------------- interpolation

void test_interpolation() {
    std::printf("interpolation, and the refusal to extrapolate\n");

    AmpCorrection c = load("1e9,-3.0\n2e9,-5.0\n4e9,-13.0\n");
    double v = 0;

    ok("the lower endpoint is defined", c.offset_at(1e9, &v));
    near("  at f_min", v, -3.0);
    ok("the upper endpoint is defined", c.offset_at(4e9, &v));
    near("  at f_max", v, -13.0);

    ok("a midpoint interpolates", c.offset_at(1.5e9, &v));
    near("  halfway across the first span", v, -4.0);
    ok("a knot is exact", c.offset_at(2e9, &v));
    near("  at the interior knot", v, -5.0);
    ok("the second span has its own slope", c.offset_at(3e9, &v));
    near("  halfway across the second span", v, -9.0);

    v = 1234.0;
    ok("below the file is REFUSED, not clamped", !c.offset_at(0.5e9, &v));
    ok("  and the output is untouched", v == 1234.0);
    ok("above the file is REFUSED, not clamped", !c.offset_at(6e9, &v));

    ok("an unloaded correction defines nothing", !AmpCorrection().offset_at(1e9, &v));

    std::printf("coverage decides whether the units may read dBm\n");
    ok("a span inside the file is covered", c.covers(1.2e9, 3.0e9));
    ok("the exact file range is covered", c.covers(1e9, 4e9));
    ok("a span running off the top is NOT covered", !c.covers(1.2e9, 5e9));
    ok("a span running off the bottom is NOT covered", !c.covers(0.1e9, 3e9));
    ok("an unloaded correction covers nothing", !AmpCorrection().covers(1e9, 2e9));
}

// ------------------------------------------------------------------- apply

void test_apply_frequency() {
    std::printf("apply() on a frequency axis\n");

    // 4 bins of 1 GHz each, centres at 0.5, 1.5, 2.5, 3.5 GHz.
    AmpCorrection c = load("1e9,-10.0\n3e9,-30.0\n");

    std::vector<float> db = {-50, -50, -50, -50};
    const size_t un = c.apply(db.data(), db.size(), 0.0, 1e9);

    ok("the bin below the file is left alone", db[0] == -50.0f);
    near("bin at 1.5 GHz gets the interpolated -15 dB", db[1], -65.0, 1e-4);
    near("bin at 2.5 GHz gets the interpolated -25 dB", db[2], -75.0, 1e-4);
    ok("the bin above the file is left alone", db[3] == -50.0f);
    ok("and both uncorrected bins are counted", un == 2);

    // The dB offset is a linear power scale factor: adding it is exact, and
    // that is why no linearise / re-log round trip appears in apply().
    const double lin_in = std::pow(10.0, -50.0 / 10.0);
    const double lin_want = lin_in * std::pow(10.0, -15.0 / 10.0);
    near("adding dB == scaling linear power", std::pow(10.0, db[1] / 10.0), lin_want,
         lin_want * 1e-6);

    std::printf("NaN is not a measurement and must not become one\n");
    std::vector<float> n = {NAN, -50, NAN};
    c.apply(n.data(), n.size(), 1e9, 1e9);   // centres 1.5, 2.5, 3.5 GHz
    ok("a NaN bin stays NaN", std::isnan(n[0]));
    ok("a measured bin between two NaNs is still corrected", !std::isnan(n[1]));
    ok("the trailing NaN stays NaN", std::isnan(n[2]));

    std::printf("degenerate inputs do not corrupt the trace\n");
    std::vector<float> z = {-10, -20};
    ok("an unloaded correction reports every bin uncorrected",
       AmpCorrection().apply(z.data(), z.size(), 1e9, 1e9) == 2);
    ok("  and changes nothing", z[0] == -10.0f && z[1] == -20.0f);
    ok("bin_hz <= 0 corrects nothing", c.apply(z.data(), z.size(), 1e9, 0.0) == 2);
    ok("a null pointer is not dereferenced", c.apply(nullptr, 4, 1e9, 1e9) == 0);
}

void test_apply_zero_span() {
    std::printf("apply_flat(): zero span has one frequency, not an axis\n");

    AmpCorrection c = load("1e9,-10.0\n3e9,-30.0\n");

    std::vector<float> db = {-40, NAN, -60};
    ok("a centre inside the file applies", c.apply_flat(db.data(), db.size(), 2e9));
    near("  every point takes the same -20 dB offset", db[0], -60.0, 1e-4);
    ok("  NaN still stays NaN", std::isnan(db[1]));
    near("  including the last point", db[2], -80.0, 1e-4);

    std::vector<float> out = {-40, -60};
    ok("a centre outside the file is refused", !c.apply_flat(out.data(), out.size(), 9e9));
    ok("  and the trace is left completely untouched (all or nothing)",
       out[0] == -40.0f && out[1] == -60.0f);

    // The trap this whole function exists for: a zero-span x value is seconds.
    // Feeding it to apply() would ask for a correction at ~0 Hz and find
    // nothing -- which is the *lucky* outcome. Assert the honest path instead.
    std::vector<float> t = {-40, -40, -40};
    const size_t un = c.apply(t.data(), t.size(), 0.0, 4e-6);   // a time axis, in error
    ok("a time axis fed to apply() corrects nothing rather than inventing a level",
       un == 3 && t[0] == -40.0f);
}

// --------------------------------------------------- the kind, and the conditions

// The fourth trap, and the one that can put a fabricated dBm number on screen:
// a TX->RX loopback measures the round trip, so its offsets flatten a response
// and carry no reference level. `tools/ampcal` marks such a file `relative`,
// and everything downstream of `absolute()` has to keep saying dBFS for it.
void test_kind() {
    std::printf("kind: a relative correction is not a dBm one\n");

    AmpCorrection plain = load("1e9,-3\n2e9,-4\n");
    ok("a file with no directive reads as absolute (what a hand-written §7 file means)",
       plain.absolute() && plain.kind() == AmpCorrection::Kind::Absolute);

    AmpCorrection rel = load("# sweepsa-ampcorr: relative\n1e9,-3\n2e9,-4\n");
    ok("an explicit relative directive is honoured", !rel.absolute());
    ok("  and it still loads its points like any other file", rel.size() == 2);
    ok("  kind_name() says so out loud", std::string(rel.kind_name()) == "relative");

    // The offsets themselves are applied identically. Relative is a statement
    // about what the numbers *mean*, not about whether to use them: the shape
    // really is removed either way, and refusing to apply it would throw away
    // the half of the measurement that is valid.
    std::vector<float> db = {-50, -50};
    rel.apply(db.data(), db.size(), 1e9, 1e9);   // bin 0 is centred at 1.5 GHz
    near("  a relative correction is still applied to the data", db[0], -53.5, 1e-4);

    AmpCorrection abs_ = load("#sweepsa-ampcorr:ABSOLUTE\n1e9,-3\n2e9,-4\n");
    ok("case and spacing in the directive do not matter", abs_.absolute());

    AmpCorrection bad;
    std::string err;
    ok("an unknown kind is refused rather than guessed",
       !bad.load_csv_text("# sweepsa-ampcorr: probably\n1e9,-3\n2e9,-4\n", "t", &err));

    // Forward compatibility: a future ampcal writing a directive this build has
    // never heard of must not make its files unloadable.
    AmpCorrection fwd = load("# sweepsa-ampcorr: relative\n"
                             "# sweepsa-ampcorr-temperature-c: 41.5\n"
                             "1e9,-3\n2e9,-4\n");
    ok("an unknown directive key is ignored, not refused", fwd.loaded() && !fwd.absolute());

    // An ordinary comment must not be mistaken for a directive.
    AmpCorrection cmt = load("# sweepsa is great\n# ampcorr: relative\n1e9,-3\n2e9,-4\n");
    ok("a comment that is not the directive leaves the kind alone", cmt.absolute());
}

void test_conditions() {
    std::printf("conditions: a correction is only valid where it was measured\n");

    AmpCorrection c = load("# sweepsa-ampcorr: absolute\n"
                           "# sweepsa-ampcorr-gain-db: 30.5\n"
                           "# sweepsa-ampcorr-span-hz: 70000000 6000000000\n"
                           "# sweepsa-ampcorr-detector: rms\n"
                           "# sweepsa-ampcorr-adc-hz: 640000000\n"
                           "# sweepsa-ampcorr-note: via sweep, 238 points\n"
                           "1e9,-3\n2e9,-4\n");
    const AmpCorrection::Conditions &k = c.conditions();
    ok("the RX gain is recorded", k.has_gain_db);
    near("  and read back", k.gain_db, 30.5, 1e-9);
    ok("the span is recorded", k.has_span);
    near("  low edge", k.span_lo_hz, 70e6, 1e-3);
    near("  high edge", k.span_hi_hz, 6e9, 1e-3);
    ok("the detector is recorded", k.detector == "rms");
    ok("the ADC clock is recorded", k.has_adc_hz);
    near("  and read back", k.adc_hz, 640e6, 1e-3);
    ok("the note survives verbatim", k.note == "via sweep, 238 points");

    AmpCorrection none = load("1e9,-3\n2e9,-4\n");
    ok("absent is represented, not defaulted to zero",
       !none.conditions().has_gain_db && !none.conditions().has_span);

    // An unknown key is a future writer's business; a *known* key with an
    // unreadable value is a broken file, and letting it degrade to "not
    // recorded" would turn a typo into a missing warning.
    std::string err;
    AmpCorrection bad;
    ok("a malformed gain is refused, not silently dropped",
       !bad.load_csv_text("# sweepsa-ampcorr-gain-db: thirty\n1e9,-3\n2e9,-4\n", "t", &err));
    ok("a one-sided span is refused",
       !bad.load_csv_text("# sweepsa-ampcorr-span-hz: 70000000\n1e9,-3\n2e9,-4\n", "t", &err));
    ok("a malformed ADC clock is refused",
       !bad.load_csv_text("# sweepsa-ampcorr-adc-hz: fast\n1e9,-3\n2e9,-4\n", "t", &err));
    ok("a backwards span is refused",
       !bad.load_csv_text("# sweepsa-ampcorr-span-hz: 6e9 70e6\n1e9,-3\n2e9,-4\n", "t", &err));

    // clear() has to take the kind and the conditions with it, or the next file
    // inherits the previous one's claims.
    c.clear();
    ok("clear() resets the kind to the default", c.absolute());
    ok("clear() forgets the conditions", !c.conditions().has_gain_db &&
                                         !c.conditions().has_adc_hz &&
                                         c.conditions().detector.empty());
}

}  // namespace

int main() {
    test_parse_accepts();
    test_parse_refuses();
    test_interpolation();
    test_apply_frequency();
    test_apply_zero_span();
    test_kind();
    test_conditions();
    std::printf("%s\n", g_failures ? "FAILURES" : "all ok");
    return g_failures ? 1 : 0;
}
