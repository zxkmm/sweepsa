#include "freq.h"

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace sweepui {
namespace {

std::string lower(const std::string &s) {
    std::string r = s;
    for (char &c : r) c = char(std::tolower((unsigned char)c));
    return r;
}

// Trims trailing zeros from a fixed-point rendering, and the point with them.
// "1.500000" -> "1.5", "2.000000" -> "2".
void trim_zeros(std::string *s) {
    if (s->find('.') == std::string::npos) return;
    size_t end = s->size();
    while (end > 0 && (*s)[end - 1] == '0') --end;
    if (end > 0 && (*s)[end - 1] == '.') --end;
    s->resize(end);
}

}  // namespace

bool parse_frequency(const std::string &s, double *out_hz) {
    // Strip surrounding whitespace; everything else is handled positionally.
    size_t b = 0, e = s.size();
    while (b < e && std::isspace((unsigned char)s[b])) ++b;
    while (e > b && std::isspace((unsigned char)s[e - 1])) --e;
    if (b == e) return false;

    const std::string t = s.substr(b, e - b);

    // strtod consumes the mantissa, including an exponent. That means "1e6" is
    // 1e6 Hz and "1e6k" is 1e9 Hz, both of which are what someone typing them
    // means.
    const char *cs = t.c_str();
    char *end = nullptr;
    errno = 0;
    const double mant = std::strtod(cs, &end);
    if (end == cs) return false;               // no number at all
    if (!std::isfinite(mant)) return false;    // "inf", "nan", or an overflow

    std::string unit = end;
    // Whitespace between number and unit is allowed: "455 kHz".
    size_t ub = 0;
    while (ub < unit.size() && std::isspace((unsigned char)unit[ub])) ++ub;
    unit = lower(unit.substr(ub));

    double mult;
    if (unit.empty() || unit == "h" || unit == "hz") {
        mult = 1.0;
    } else if (unit == "k" || unit == "khz") {
        mult = 1e3;
    } else if (unit == "m" || unit == "mhz" || unit == "meg") {
        // Instrument convention, not SI. See the header.
        mult = 1e6;
    } else if (unit == "g" || unit == "ghz" || unit == "gig") {
        mult = 1e9;
    } else {
        return false;                          // trailing garbage
    }

    const double hz = mant * mult;
    if (!std::isfinite(hz)) return false;
    *out_hz = hz;
    return true;
}

std::string format_frequency(double hz, int sig) {
    if (!std::isfinite(hz)) return "-";
    if (sig < 0) sig = 0;
    if (sig > 12) sig = 12;

    const double a = std::fabs(hz);
    double scale = 1.0;
    const char *unit = "Hz";
    if (a >= 1e9) {
        scale = 1e9;
        unit = "GHz";
    } else if (a >= 1e6) {
        scale = 1e6;
        unit = "MHz";
    } else if (a >= 1e3) {
        scale = 1e3;
        unit = "kHz";
    }

    char buf[64];
    std::snprintf(buf, sizeof buf, "%.*f", sig, hz / scale);
    std::string num = buf;
    trim_zeros(&num);
    return num + " " + unit;
}

std::string format_frequency_mhz(double hz, int decimals) {
    if (!std::isfinite(hz)) return "-";
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.*f", decimals, hz / 1e6);
    return buf;
}

std::string format_seconds(double s, int sig) {
    if (!std::isfinite(s)) return "-";
    if (sig < 0) sig = 0;
    if (sig > 12) sig = 12;

    const double a = std::fabs(s);
    double scale = 1.0;
    const char *unit = "s";
    // Exact zero keeps the base unit rather than being scaled into nanoseconds,
    // where "0 ns" would suggest a resolution that was never measured.
    if (a == 0.0) {
        // fall through with scale 1
    } else if (a < 1e-6) {
        scale = 1e-9;
        unit = "ns";
    } else if (a < 1e-3) {
        scale = 1e-6;
        unit = "us";
    } else if (a < 1.0) {
        scale = 1e-3;
        unit = "ms";
    }

    char buf[64];
    std::snprintf(buf, sizeof buf, "%.*f", sig, s / scale);
    std::string num = buf;
    trim_zeros(&num);
    return num + " " + unit;
}

}  // namespace sweepui
