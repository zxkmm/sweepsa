// Host-side tests for the §6 frequency entry convention.
//
// The trap this pins down is `m`. On an instrument keypad it means mega; in SI
// it means milli. A parser that got that backwards would tune 100 Hz when asked
// for 100 MHz, and the display would show a perfectly plausible empty span.

#include <cmath>
#include <cstdio>
#include <string>

#include "../src/sweepui/freq.h"

using namespace sweepui;

namespace {

int g_failures = 0;

void check_parse(const char *in, double want) {
    double got = 0;
    const bool ok = parse_frequency(in, &got);
    const bool pass = ok && std::fabs(got - want) <= std::fabs(want) * 1e-12 + 1e-6;
    std::printf("  %-20s -> %-18.6f want %-18.6f %s\n", in, ok ? got : 0.0, want,
                pass ? "ok" : "FAIL");
    if (!pass) ++g_failures;
}

void check_reject(const char *in) {
    double got = 12345.0;
    const bool ok = parse_frequency(in, &got);
    const bool pass = !ok && got == 12345.0;   // *out untouched on failure
    std::printf("  %-20s -> rejected                              %s\n", in,
                pass ? "ok" : "FAIL");
    if (!pass) ++g_failures;
}

void check_format(double hz, const char *want) {
    const std::string got = format_frequency(hz);
    const bool pass = got == want;
    std::printf("  %-18.0f -> %-18s want %-18s %s\n", hz, got.c_str(), want,
                pass ? "ok" : "FAIL");
    if (!pass) ++g_failures;
}

void test_suffixes() {
    std::printf("instrument suffixes (§6: \"2.4g, 100m, 455k\")\n");
    check_parse("2.4g", 2.4e9);
    check_parse("100m", 100e6);       // mega, not milli
    check_parse("455k", 455e3);
    check_parse("1000000", 1e6);      // bare number is Hz
    check_parse("70M", 70e6);
    check_parse("2.4 GHz", 2.4e9);
    check_parse("455 khz", 455e3);
    check_parse("  915MHz  ", 915e6);
    check_parse("1.5meg", 1.5e6);
    check_parse("6gig", 6e9);
    check_parse("1e6", 1e6);
    check_parse("1e3k", 1e6);         // exponent and suffix compose
    check_parse("-1.5m", -1.5e6);     // deltas and offsets can be negative
    check_parse("0", 0.0);
}

void test_rejections() {
    std::printf("input that must be refused rather than half-understood\n");
    check_reject("");
    check_reject("   ");
    check_reject("ghz");
    check_reject("2.4gg");
    check_reject("2.4 gigahertz");
    check_reject("100mm");
    check_reject("100 m hz");
    check_reject("nan");
    check_reject("inf");
    check_reject("1.2.3");
}

void test_format() {
    std::printf("display formatting\n");
    check_format(0, "0 Hz");
    check_format(455e3, "455 kHz");
    check_format(100e6, "100 MHz");
    check_format(1.5e9, "1.5 GHz");
    check_format(999.0, "999 Hz");
    check_format(1000.0, "1 kHz");
    check_format(2.412e9, "2.412 GHz");
}

// The round trip is what the UI actually does every frame: it formats the
// current view into the entry boxes, and the user may press Enter without
// editing. If that changed the frequency, the display would drift on its own.
void test_round_trip() {
    std::printf("format -> parse round trip is exact\n");
    const double vals[] = {70e6, 88.1e6, 433.92e6, 915e6, 2.412e9, 5.745e9, 5.93e9,
                           1234567.0, 1.0};
    for (double v : vals) {
        double back = 0;
        const std::string s = format_frequency(v);
        const bool ok = parse_frequency(s, &back) && std::fabs(back - v) < 1e-3;
        std::printf("  %-18.3f -> %-14s -> %-18.3f %s\n", v, s.c_str(), back,
                    ok ? "ok" : "FAIL");
        if (!ok) ++g_failures;
    }
}

}  // namespace

int main() {
    test_suffixes();
    test_rejections();
    test_format();
    test_round_trip();
    std::printf("%s\n", g_failures ? "FAILURES" : "all ok");
    return g_failures ? 1 : 0;
}
