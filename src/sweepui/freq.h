// Frequency entry and display. CLAUDE.md §6, "Frequency".
//
// §6 asks for "keyboard entry with unit suffixes (2.4g, 100m, 455k)". That is a
// deliberately terse instrument convention and it collides with SI: on a
// spectrum analyzer `m` means mega, never milli, and `100m` in a frequency field
// is 100 MHz. This file is where that convention lives, alone and tested, rather
// than being open-coded in a widget callback where a wrong guess would silently
// tune 9 orders of magnitude away.
//
// Nothing here depends on ImGui or GL, so tests/freq_test.cpp exercises it
// natively.

#pragma once

#include <string>

namespace sweepui {

// Parses a frequency the way an instrument keypad does.
//
// Accepted: an optional sign, a decimal or exponential mantissa, optional
// whitespace, and an optional unit. Units are case-insensitive and may be the
// bare multiplier letter or the full spelling:
//
//   (none)          Hz          "1000000"   -> 1e6
//   h, hz           Hz          "455000 hz" -> 455e3
//   k, khz          kHz         "455k"      -> 455e3
//   m, mhz, meg     MHz         "100m"      -> 100e6
//   g, ghz, gig     GHz         "2.4g"      -> 2.4e9
//
// Returns false on trailing garbage, an empty string, a non-finite result, or a
// unit that is not one of the above -- never a partial parse. `*out` is written
// only on success.
//
// Note the asymmetry with §6's other numeric fields: this is exclusively for
// frequency. A dB field that accepted `m` as mega would be a menace.
bool parse_frequency(const std::string &s, double *out_hz);

// Formats Hz for display, picking the unit from the magnitude and trimming
// trailing zeros: 1.5e9 -> "1.5 GHz", 455e3 -> "455 kHz", 0 -> "0 Hz".
//
// `sig` is the number of digits kept after the decimal point before trimming.
// The default of 6 resolves 1 Hz at GHz scale, which is finer than the hardware
// can tune and therefore never the reason two frequencies look equal.
std::string format_frequency(double hz, int sig = 6);

// Same, but padded to a fixed unit so a column of values lines up. Used by the
// marker table, where changing units mid-column is unreadable.
std::string format_frequency_mhz(double hz, int decimals = 6);

// ---------------------------------------------------------------- time

// Zero span's x-axis is time, so the same display problem appears again in
// seconds: 0.0000166667 is unreadable and "16.667 us" is not.
//
// Deliberately *not* the mirror of parse_frequency. There is no time-entry
// field, and if one is ever added it must not inherit the `m` = mega
// convention, which would make "20m" twenty megaseconds in a field where the
// user unambiguously meant twenty milliseconds. Formatting is safe to share;
// parsing is not, and that asymmetry is the point.
//
//   1.5e-3 -> "1.5 ms", 16.6667e-6 -> "16.667 us", 0 -> "0 s"
std::string format_seconds(double s, int sig = 4);

}  // namespace sweepui
