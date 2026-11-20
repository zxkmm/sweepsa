#include "ampcorr.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace sweepui {
namespace {

// Coverage tolerance, in Hz. A span edge and a correction endpoint that differ
// by a fraction of a Hz differ because of bin-centre arithmetic, not because
// the file stops short; 1 Hz is far below any RBW this instrument can produce
// and far above the rounding. Not a measured constant, so §8's "no hardcoded
// timing constants" does not apply -- this is a floating-point guard.
constexpr double kCoverEpsHz = 1.0;

// Trim ASCII whitespace, and a UTF-8 BOM if this is the first line.
std::string trim(const std::string &s) {
    size_t b = 0, e = s.size();
    while (b < e && (s[b] == ' ' || s[b] == '\t' || s[b] == '\r' || s[b] == '\n')) ++b;
    while (e > b && (s[e - 1] == ' ' || s[e - 1] == '\t' || s[e - 1] == '\r' || s[e - 1] == '\n')) --e;
    return s.substr(b, e - b);
}

// Parse one field as a finite double. Returns false on anything that is not a
// complete number, so "1e9abc" and "" are both refused rather than becoming 1e9
// and 0 -- strtod alone would accept the first and silently truncate it.
bool parse_num(const std::string &f, double *out) {
    if (f.empty()) return false;
    const char *p = f.c_str();
    char *end = nullptr;
    const double v = std::strtod(p, &end);
    if (end == p) return false;
    while (*end == ' ' || *end == '\t') ++end;
    if (*end != '\0') return false;
    if (!std::isfinite(v)) return false;
    *out = v;
    return true;
}

// Split on a comma if there is one, otherwise on whitespace. Either is a CSV
// people actually produce, and guessing wrong turns a two-column file into a
// one-column one.
bool split2(const std::string &line, std::string *a, std::string *b) {
    const size_t c = line.find(',');
    if (c != std::string::npos) {
        *a = trim(line.substr(0, c));
        // Anything after a second comma is ignored: a file with extra columns
        // (a measurement date, a note) still loads on its first two.
        std::string rest = trim(line.substr(c + 1));
        const size_t c2 = rest.find(',');
        *b = (c2 == std::string::npos) ? rest : trim(rest.substr(0, c2));
        return !a->empty() && !b->empty();
    }
    std::istringstream is(line);
    if (!(is >> *a)) return false;
    if (!(is >> *b)) return false;
    return true;
}

// If `line` is one of the `# sweepsa-ampcorr...` directives, strip the prefix
// and hand back the key and the value. Anything else is an ordinary comment.
//
// Deliberately tolerant of case and of the space after the `#`, and
// deliberately *silent* about a directive it does not know: this is a forward
// compatibility point, and a future `ampcal` writing a fifth line must not make
// its files unloadable by this build.
bool directive(const std::string &line, std::string *key, std::string *value) {
    if (line.empty() || line[0] != '#') return false;
    std::string s = trim(line.substr(1));
    const std::string kPrefix = "sweepsa-ampcorr";
    if (s.size() < kPrefix.size()) return false;
    for (size_t i = 0; i < kPrefix.size(); ++i)
        if (std::tolower((unsigned char)s[i]) != kPrefix[i]) return false;
    s = s.substr(kPrefix.size());

    const size_t colon = s.find(':');
    if (colon == std::string::npos) return false;
    *key = trim(s.substr(0, colon));
    if (!key->empty() && (*key)[0] == '-') *key = key->substr(1);
    for (char &c : *key) c = char(std::tolower((unsigned char)c));
    *value = trim(s.substr(colon + 1));
    return true;
}

}  // namespace

const char *AmpCorrection::kind_name() const {
    return kind_ == Kind::Relative ? "relative" : "absolute";
}

void AmpCorrection::clear() {
    pts_.clear();
    name_.clear();
    kind_ = Kind::Absolute;
    cond_ = Conditions{};
}

bool AmpCorrection::load_csv(const std::string &path, std::string *err) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        if (err) *err = "cannot open '" + path + "'";
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return load_csv_text(ss.str(), path, err);
}

bool AmpCorrection::load_csv_text(const std::string &text, const std::string &name,
                                  std::string *err) {
    auto fail = [&](const std::string &m) {
        if (err) *err = m;
        return false;   // note: pts_ is untouched, so a bad file does not
                        // de-calibrate a working session (see ampcorr.h)
    };

    std::vector<Point> pts;
    Kind kind = Kind::Absolute;
    Conditions cond;
    std::istringstream in(text);
    std::string raw;
    size_t lineno = 0;
    bool header_allowed = true;

    while (std::getline(in, raw)) {
        ++lineno;
        std::string line = trim(raw);
        if (lineno == 1 && line.size() >= 3 &&
            (unsigned char)line[0] == 0xEF && (unsigned char)line[1] == 0xBB &&
            (unsigned char)line[2] == 0xBF)
            line = trim(line.substr(3));            // UTF-8 BOM

        if (line.empty()) continue;
        if (line[0] == '#') {
            std::string k, v;
            if (!directive(line, &k, &v)) continue;

            if (k.empty()) {
                // `# sweepsa-ampcorr: KIND` -- the units question, and the only
                // directive whose absence has a defined meaning.
                std::string kv = v;
                for (char &c : kv) c = char(std::tolower((unsigned char)c));
                if (kv == "relative") kind = Kind::Relative;
                else if (kv == "absolute") kind = Kind::Absolute;
                else
                    return fail("line " + std::to_string(lineno) +
                                ": unknown correction kind '" + v +
                                "'; expected 'absolute' or 'relative'");
            } else if (k == "gain-db") {
                // An unknown *key* is ignored (forward compatibility), but a
                // known key with an unreadable value is a broken file. Letting
                // it degrade to "not recorded" would turn a typo into a missing
                // warning, which is the one outcome this directive exists to
                // prevent.
                double g = 0;
                if (!parse_num(v, &g))
                    return fail("line " + std::to_string(lineno) + ": gain-db '" + v +
                                "' is not a number");
                cond.has_gain_db = true;
                cond.gain_db = g;
            } else if (k == "span-hz") {
                std::string a, b;
                double lo = 0, hi = 0;
                if (!split2(v, &a, &b) || !parse_num(a, &lo) || !parse_num(b, &hi))
                    return fail("line " + std::to_string(lineno) +
                                ": span-hz expects two frequencies, got '" + v + "'");
                if (!(hi > lo))
                    return fail("line " + std::to_string(lineno) +
                                ": span-hz is not increasing");
                cond.has_span = true;
                cond.span_lo_hz = lo;
                cond.span_hi_hz = hi;
            } else if (k == "adc-hz") {
                double a = 0;
                if (!parse_num(v, &a) || !(a > 0))
                    return fail("line " + std::to_string(lineno) + ": adc-hz '" + v +
                                "' is not a positive number");
                cond.has_adc_hz = true;
                cond.adc_hz = a;
            } else if (k == "detector") {
                cond.detector = v;
            } else if (k == "note") {
                cond.note = v;
            }
            // Any other key: a newer writer's business, not ours.
            continue;
        }

        std::string fa, fb;
        if (!split2(line, &fa, &fb))
            return fail("line " + std::to_string(lineno) +
                        ": expected two columns 'freq_hz,offset_db'");

        double f = 0, o = 0;
        if (!parse_num(fa, &f) || !parse_num(fb, &o)) {
            // Exactly one header row is tolerated, and only before any data.
            if (header_allowed) {
                header_allowed = false;
                continue;
            }
            return fail("line " + std::to_string(lineno) + ": '" + line +
                        "' is not a number pair");
        }
        header_allowed = false;

        // A non-positive frequency is very nearly always a swapped column pair
        // (offset first), which would otherwise load as a correction covering
        // negative frequencies and silently correct nothing.
        if (!(f > 0))
            return fail("line " + std::to_string(lineno) + ": frequency " +
                        std::to_string(f) +
                        " Hz is not positive -- are the columns swapped? "
                        "the format is 'freq_hz,offset_db'");

        pts.push_back({f, o});
    }

    if (pts.size() < 2)
        return fail("a correction needs at least 2 points; found " +
                    std::to_string(pts.size()) +
                    ". A single number is a reference-level offset, not a "
                    "correction curve, and applying it at every frequency is "
                    "the extrapolation §7 exists to prevent.");

    std::stable_sort(pts.begin(), pts.end(),
                     [](const Point &a, const Point &b) { return a.freq_hz < b.freq_hz; });

    // Collapse exact duplicate rows; refuse contradictory ones. A file that
    // says two different things about one frequency has no defensible reading,
    // and picking either is the kind of quiet choice §7 forbids.
    std::vector<Point> uniq;
    uniq.reserve(pts.size());
    for (const Point &p : pts) {
        if (!uniq.empty() && uniq.back().freq_hz == p.freq_hz) {
            if (uniq.back().offset_db == p.offset_db) continue;
            char buf[160];
            std::snprintf(buf, sizeof buf,
                          "two different offsets (%.3f and %.3f dB) given for %.0f Hz",
                          uniq.back().offset_db, p.offset_db, p.freq_hz);
            return fail(buf);
        }
        uniq.push_back(p);
    }
    if (uniq.size() < 2)
        return fail("a correction needs at least 2 distinct frequencies");

    pts_ = std::move(uniq);
    name_ = name;
    kind_ = kind;
    cond_ = std::move(cond);
    return true;
}

bool AmpCorrection::offset_at(double freq_hz, double *out_db) const {
    if (!loaded()) return false;
    if (freq_hz < pts_.front().freq_hz || freq_hz > pts_.back().freq_hz)
        return false;                              // never extrapolate

    // Upper bound by frequency, then interpolate against its predecessor.
    const auto it = std::lower_bound(
        pts_.begin(), pts_.end(), freq_hz,
        [](const Point &p, double f) { return p.freq_hz < f; });

    if (it == pts_.begin()) {                      // exactly the first point
        *out_db = pts_.front().offset_db;
        return true;
    }
    const Point &hi = *it;
    const Point &lo = *(it - 1);
    if (hi.freq_hz == lo.freq_hz) {                // cannot happen: deduped
        *out_db = hi.offset_db;
        return true;
    }
    const double t = (freq_hz - lo.freq_hz) / (hi.freq_hz - lo.freq_hz);
    *out_db = lo.offset_db + t * (hi.offset_db - lo.offset_db);
    return true;
}

bool AmpCorrection::covers(double f_lo, double f_hi) const {
    if (!loaded()) return false;
    return f_lo >= pts_.front().freq_hz - kCoverEpsHz &&
           f_hi <= pts_.back().freq_hz + kCoverEpsHz;
}

size_t AmpCorrection::apply(float *db, size_t n, double f_start_hz, double bin_hz) const {
    if (!db || n == 0) return 0;
    if (!loaded() || !(bin_hz > 0)) return n;      // nothing corrected

    size_t uncorrected = 0;

    // Bin frequencies increase monotonically, so walk a cursor through the
    // table rather than binary-searching 65536 times per frame.
    size_t j = 0;
    for (size_t i = 0; i < n; ++i) {
        const double f = f_start_hz + (double(i) + 0.5) * bin_hz;

        if (f < pts_.front().freq_hz || f > pts_.back().freq_hz) {
            ++uncorrected;
            continue;
        }
        while (j + 2 < pts_.size() && pts_[j + 1].freq_hz < f) ++j;

        const Point &lo = pts_[j];
        const Point &hi = pts_[j + 1];
        double off;
        if (f <= lo.freq_hz) {
            off = lo.offset_db;
        } else if (hi.freq_hz > lo.freq_hz) {
            const double t = (f - lo.freq_hz) / (hi.freq_hz - lo.freq_hz);
            off = lo.offset_db + t * (hi.offset_db - lo.offset_db);
        } else {
            off = hi.offset_db;
        }

        // NaN marks a bin no segment covered (proto.h). It stays NaN: an offset
        // would give an unmeasured bin a number. This is implicit in IEEE
        // arithmetic but stated explicitly because it is a §7 rule, not an
        // accident of the float format.
        if (!std::isnan(db[i])) db[i] = float(double(db[i]) + off);
    }
    return uncorrected;
}

bool AmpCorrection::apply_flat(float *db, size_t n, double center_hz) const {
    double off = 0;
    if (!offset_at(center_hz, &off)) return false;
    if (!db) return false;
    for (size_t i = 0; i < n; ++i)
        if (!std::isnan(db[i])) db[i] = float(double(db[i]) + off);
    return true;
}

}  // namespace sweepui
