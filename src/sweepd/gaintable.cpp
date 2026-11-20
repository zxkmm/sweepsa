#include "gaintable.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace sweepd {
namespace {

std::string trim(const std::string &s) {
    size_t a = 0, b = s.size();
    while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r')) ++a;
    while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r')) --b;
    return s.substr(a, b - a);
}

// strtol with base 0 reads "30" and "0x1E" alike, which is what the driver's
// own output and SATSAGEN's files mix on one line.
bool read_cell(const char **p, long *out) {
    while (**p == ' ' || **p == '\t') ++*p;
    char *end = nullptr;
    errno = 0;
    const long v = std::strtol(*p, &end, 0);
    if (end == *p || errno) return false;
    *p = end;
    while (**p == ' ' || **p == '\t') ++*p;
    *out = v;
    return true;
}

std::string at_line(int n) { return "line " + std::to_string(n) + ": "; }

}  // namespace

bool parse_gain_table(const std::string &text, GainTableInfo *out, std::string *err) {
    GainTableInfo info;
    GainTableBand cur;
    bool in_table = false;
    int line_no = 0;
    size_t pos = 0;
    while (pos <= text.size()) {
        size_t nl = text.find('\n', pos);
        if (nl == std::string::npos) nl = text.size();
        const std::string line = trim(text.substr(pos, nl - pos));
        pos = nl + 1;
        ++line_no;
        if (line.empty() || line == "<list>" || line == "</list>") continue;

        if (line.compare(0, 10, "<gaintable") == 0) {
            if (in_table) {
                *err = at_line(line_no) + "<gaintable> inside another <gaintable>";
                return false;
            }
            int chip = 0;
            char type[16] = {};
            cur = GainTableBand{};
            if (std::sscanf(line.c_str(), "<gaintable AD%d type=%15s dest=%d start=%lld end=%lld>",
                            &chip, type, &cur.dest, &cur.start_hz, &cur.end_hz) != 5) {
                *err = at_line(line_no) + "expected <gaintable AD9361 type=FULL dest=N "
                                          "start=HZ end=HZ>, got \"" + line + "\"";
                return false;
            }
            if (std::strcmp(type, "FULL") != 0) {
                *err = at_line(line_no) + "only type=FULL tables are supported (got " +
                       type + "): a split table separates LNA and mixer gain, and "
                              "sweepd sets one manual gain";
                return false;
            }
            if (cur.end_hz <= cur.start_hz) {
                *err = at_line(line_no) + "end is not above start";
                return false;
            }
            in_table = true;
            continue;
        }
        if (line == "</gaintable>") {
            if (!in_table || cur.n_rows == 0) {
                *err = at_line(line_no) + (in_table ? "empty <gaintable>"
                                                    : "</gaintable> without <gaintable>");
                return false;
            }
            info.bands.push_back(cur);
            in_table = false;
            continue;
        }
        if (!in_table) {
            *err = at_line(line_no) + "row outside a <gaintable>: \"" + line + "\"";
            return false;
        }

        long v[4];
        const char *p = line.c_str();
        for (int k = 0; k < 4; ++k) {
            if (!read_cell(&p, &v[k]) || (k < 3 ? *p != ',' : *p != '\0')) {
                *err = at_line(line_no) + "expected four comma-separated numbers "
                                          "(gain, lna, mixer, dc_cal), got \"" + line + "\"";
                return false;
            }
            if (k < 3) ++p;
        }
        for (int k = 1; k < 4; ++k) {
            if (v[k] < 0 || v[k] > 0xFF) {
                *err = at_line(line_no) + "register value out of 0..0xFF";
                return false;
            }
        }
        // The driver indexes by row and reports the first column as dB. A
        // column that goes down makes "more gain" mean less on some index,
        // which no manual-gain request can be validated against.
        if (cur.n_rows > 0 && v[0] < cur.gain_max_db) {
            *err = at_line(line_no) + "gain column decreases (" + std::to_string(v[0]) +
                   " after " + std::to_string(cur.gain_max_db) + ")";
            return false;
        }
        if (cur.n_rows == 0) cur.gain_min_db = int(v[0]);
        cur.gain_max_db = int(v[0]);
        cur.cells.insert(cur.cells.end(), v, v + 4);
        ++cur.n_rows;
    }
    if (in_table) {
        *err = "unterminated <gaintable> at end of file";
        return false;
    }
    if (info.bands.empty()) {
        *err = "no <gaintable> found";
        return false;
    }

    // The driver picks a band by LO, so bands must not overlap.
    std::vector<GainTableBand> sorted = info.bands;
    std::sort(sorted.begin(), sorted.end(),
              [](const GainTableBand &a, const GainTableBand &b) { return a.start_hz < b.start_hz; });
    for (size_t i = 1; i < sorted.size(); ++i) {
        if (sorted[i].start_hz < sorted[i - 1].end_hz) {
            *err = "bands overlap at " + std::to_string(sorted[i].start_hz) + " Hz";
            return false;
        }
    }
    info.covers_lo_hz = sorted.front().start_hz;
    info.covers_hi_hz = sorted.back().end_hz;
    info.gain_lo_db = sorted.front().gain_min_db;
    info.gain_hi_db = sorted.front().gain_max_db;
    for (const auto &b : sorted) {
        info.gain_lo_db = std::max(info.gain_lo_db, b.gain_min_db);
        info.gain_hi_db = std::min(info.gain_hi_db, b.gain_max_db);
    }
    *out = std::move(info);
    return true;
}

std::string compact_gain_table(const std::string &text) {
    if (text.size() <= kGainTableMaxBytes) return text;
    std::string out;
    out.reserve(text.size());
    for (size_t i = 0; i < text.size();) {
        const char c = text[i];
        if (c == '\r') { ++i; continue; }
        if (c == ',') {
            out.push_back(',');
            ++i;
            while (i < text.size() && (text[i] == ' ' || text[i] == '\t')) ++i;
            continue;
        }
        // A 0x token at the start of a cell: 0x00 -> 0, 0x0N -> 0xN. Only
        // where a cell starts, so "start=0x..." in a header is never touched
        // (headers carry decimal anyway).
        const bool cell_start = out.empty() || out.back() == ',' || out.back() == '\n';
        if (cell_start && c == '0' && i + 1 < text.size() && (text[i + 1] == 'x' || text[i + 1] == 'X')) {
            size_t j = i + 2;
            while (j < text.size() && std::isxdigit((unsigned char)text[j])) ++j;
            std::string digits = text.substr(i + 2, j - i - 2);
            size_t z = 0;
            while (z + 1 < digits.size() && digits[z] == '0') ++z;
            digits.erase(0, z);
            if (digits == "0") out += "0";
            else out += "0x" + digits;
            i = j;
            continue;
        }
        out.push_back(c);
        ++i;
    }
    return out;
}

bool same_cells(const GainTableInfo &a, const GainTableInfo &b) {
    if (a.bands.size() != b.bands.size()) return false;
    for (size_t i = 0; i < a.bands.size(); ++i) {
        const auto &x = a.bands[i], &y = b.bands[i];
        if (x.start_hz != y.start_hz || x.end_hz != y.end_hz || x.dest != y.dest ||
            x.cells != y.cells)
            return false;
    }
    return true;
}

bool prepare_gain_table(const std::string &text, std::string *wire, GainTableInfo *info,
                        std::string *err) {
    GainTableInfo a;
    if (!parse_gain_table(text, &a, err)) return false;
    std::string w = compact_gain_table(text);
    if (w != text) {
        GainTableInfo b;
        std::string e2;
        if (!parse_gain_table(w, &b, &e2) || !same_cells(a, b)) {
            *err = "compacting the table changed it (" + e2 + "); refusing to write it";
            return false;
        }
    }
    if (w.size() > kGainTableMaxBytes) {
        *err = "the table is " + std::to_string(w.size()) + " bytes even compacted; the "
               "driver takes at most " + std::to_string(kGainTableMaxBytes) + " in one write";
        return false;
    }
    *wire = std::move(w);
    *info = std::move(a);
    return true;
}

uint64_t gain_table_hash(const std::string &wire) {
    if (wire.empty()) return 0;
    uint64_t h = 0xcbf29ce484222325ull;
    for (unsigned char c : wire) {
        h ^= c;
        h *= 0x100000001b3ull;
    }
    return h ? h : 1;
}

}  // namespace sweepd
