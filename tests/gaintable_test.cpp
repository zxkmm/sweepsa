// Host tests for the gain-table text (src/sweepd/gaintable.h). What must hold:
// the driver's own format parses, both cell spellings it mixes (30 and 0x1E)
// read the same, the compaction that fits a table into the attribute's 4096
// bytes changes no value, and a malformed file is refused with a line number
// before the driver ever sees it.

#include <cstdio>
#include <string>

#include "../src/sweepd/gaintable.h"

using namespace sweepd;

namespace {

int g_failures = 0;

void check_true(const char *what, bool ok) {
    std::printf("  %-62s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) ++g_failures;
}

// A band in the layout the driver prints: "gain, 0xLL, 0xMM, 0xDD", a space
// after each comma, two hex digits each. Rows of the same width as the stock
// tables, so three of them exceed 4096 bytes exactly as the real list does.
std::string band(long long start, long long end, int g0, int rows) {
    std::string s = "<gaintable AD9361 type=FULL dest=3 start=" + std::to_string(start) +
                    " end=" + std::to_string(end) + ">\n";
    char line[64];
    for (int i = 0; i < rows; ++i) {
        std::snprintf(line, sizeof line, "%d, 0x%02X, 0x%02X, 0x%02X\n", g0 + i, i / 16,
                      i % 64, (i % 8) ? 0 : 0x20);
        s += line;
    }
    return s + "</gaintable>\n";
}

std::string stock_like() {
    return "<list>\n" + band(0, 1300000000, -1, 77) + band(1300000000, 4000000000LL, -3, 77) +
           band(4000000000LL, 6000000000LL, -10, 73) + "</list>\n";
}

}  // namespace

int main() {
    std::printf("parse\n");
    {
        GainTableInfo gi;
        std::string err;
        const bool ok = parse_gain_table(stock_like(), &gi, &err);
        check_true("three-band list parses", ok);
        check_true("three bands", gi.bands.size() == 3);
        check_true("covers 0 - 6 GHz", gi.covers_lo_hz == 0 && gi.covers_hi_hz == 6000000000LL);
        // -1..75, -3..73, -10..62: what every band can deliver is -1..62.
        check_true("gain range is the intersection of the bands",
                   gi.gain_lo_db == -1 && gi.gain_hi_db == 62);
    }
    {
        // SATSAGEN's spelling: bare 0 next to 0x01 on one line, no spaces.
        const std::string t = "<list>\n<gaintable AD9361 type=FULL dest=3 start=0 end=6000000000>\n"
                              "0,0,0x01,0\n1,0,0x02,0\n5,0x01,0x01,0x20\n</gaintable>\n</list>";
        GainTableInfo gi;
        std::string err;
        check_true("mixed decimal and hex cells parse", parse_gain_table(t, &gi, &err));
        check_true("0x20 reads as 32", gi.bands.size() == 1 && gi.bands[0].cells[11] == 32);
        check_true("single band covers the whole range",
                   gi.covers_lo_hz == 0 && gi.covers_hi_hz == 6000000000LL &&
                   gi.gain_lo_db == 0 && gi.gain_hi_db == 5);
    }

    std::printf("refusals\n");
    struct Bad { const char *what, *text, *needle; };
    const Bad bad[] = {
        {"no table at all", "hello\n", "line 1"},
        {"unterminated", "<gaintable AD9361 type=FULL dest=3 start=0 end=10>\n1,0,0,0\n",
         "unterminated"},
        {"three cells", "<gaintable AD9361 type=FULL dest=3 start=0 end=10>\n1,0,0\n</gaintable>",
         "line 2"},
        {"gain goes down",
         "<gaintable AD9361 type=FULL dest=3 start=0 end=10>\n5,0,0,0\n4,0,0,0\n</gaintable>",
         "decreases"},
        {"register out of range",
         "<gaintable AD9361 type=FULL dest=3 start=0 end=10>\n5,0x100,0,0\n</gaintable>",
         "0xFF"},
        {"split table", "<gaintable AD9361 type=SPLIT dest=3 start=0 end=10>\n5,0,0,0\n</gaintable>",
         "FULL"},
        {"overlapping bands",
         "<gaintable AD9361 type=FULL dest=3 start=0 end=10>\n1,0,0,0\n</gaintable>\n"
         "<gaintable AD9361 type=FULL dest=3 start=5 end=20>\n1,0,0,0\n</gaintable>",
         "overlap"},
    };
    for (const Bad &b : bad) {
        GainTableInfo gi;
        std::string err;
        const bool refused = !parse_gain_table(b.text, &gi, &err);
        check_true(b.what, refused && err.find(b.needle) != std::string::npos);
        if (refused && err.find(b.needle) == std::string::npos)
            std::printf("    (said: %s)\n", err.c_str());
    }

    std::printf("compaction\n");
    {
        const std::string t = stock_like();
        check_true("the stock-like list is over the limit", t.size() > kGainTableMaxBytes);
        std::string wire, err;
        GainTableInfo gi;
        const bool ok = prepare_gain_table(t, &wire, &gi, &err);
        check_true("it is prepared", ok);
        check_true("and fits", wire.size() <= kGainTableMaxBytes);
        GainTableInfo a, b;
        parse_gain_table(t, &a, &err);
        parse_gain_table(wire, &b, &err);
        check_true("with every value unchanged", same_cells(a, b));
        check_true("0x00 is spelled 0", wire.find("0x00") == std::string::npos);
        check_true("headers are untouched",
                   wire.find("start=1300000000 end=4000000000>") != std::string::npos);
        std::printf("    %zu -> %zu bytes\n", t.size(), wire.size());
    }
    {
        const std::string small = band(0, 6000000000LL, 0, 80);
        check_true("a table that fits is written as given", compact_gain_table(small) == small);
    }
    {
        std::string huge = "<list>\n";
        for (int k = 0; k < 8; ++k) huge += band(k * 100, k * 100 + 100, 0, 77);
        huge += "</list>\n";
        std::string wire, err;
        GainTableInfo gi;
        const bool ok = prepare_gain_table(huge, &wire, &gi, &err);
        check_true("too big even compacted is refused, naming the limit",
                   !ok && err.find("4096") != std::string::npos);
    }

    std::printf("built-in\n");
    {
        // sweepd's default must load as is: one band over the whole tuner,
        // ADI's 1.3-4 GHz ladder (77 rows, -3..71 dB), and the default --gain
        // 30 inside it.
        std::string wire, err;
        GainTableInfo gi;
        const bool ok = prepare_gain_table(builtin_gain_table(), &wire, &gi, &err);
        check_true("the built-in table is prepared", ok);
        check_true("one band, 0 - 6 GHz",
                   gi.bands.size() == 1 && gi.covers_lo_hz == 0 &&
                   gi.covers_hi_hz == 6000000000LL);
        check_true("77 rows, -3 .. 71 dB",
                   gi.bands.size() == 1 && gi.bands[0].n_rows == 77 &&
                   gi.gain_lo_db == -3 && gi.gain_hi_db == 71);
        check_true("30 dB is the row ADI's table puts it on (35: 0x24, 0x21)",
                   gi.bands.size() == 1 && gi.bands[0].cells[35 * 4] == 30 &&
                   gi.bands[0].cells[35 * 4 + 1] == 0x24 && gi.bands[0].cells[35 * 4 + 2] == 0x21);
        check_true("it is named", std::string(builtin_gain_table_name()) == "adi_1300_4000_wide");
    }

    std::printf("hash\n");
    check_true("no table hashes to 0", gain_table_hash("") == 0);
    check_true("a table does not", gain_table_hash(stock_like()) != 0);
    check_true("different tables differ",
               gain_table_hash(band(0, 10, 0, 3)) != gain_table_hash(band(0, 10, 1, 3)));

    std::printf("%s\n", g_failures ? "FAILED" : "all passed");
    return g_failures ? 1 : 0;
}
