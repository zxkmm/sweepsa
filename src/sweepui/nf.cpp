#include "nf.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

#include "freq.h"

namespace sweepui {

namespace {

double lin(double db) { return std::pow(10.0, db / 10.0); }
double db(double x) { return 10.0 * std::log10(x); }

// F from a Y-factor; false when Y <= 1 -- the source made no measurable
// difference, which is a cabling or a level problem, not a noise figure.
bool factor(double enr_db, double on, double off, double *f, std::string *why) {
    if (!(on > 0 && off > 0)) {
        *why = "a noise reading is zero or missing";
        return false;
    }
    const double y = on / off;
    // 0.3 dB: below it, Y is inside the scatter of two averaged noise
    // readings, and F = ENR/(Y-1) turns that scatter into a confident 30 or
    // 40 dB "noise figure" of nothing. Measured on the synthetic floor with no
    // source at all, before this check existed.
    if (!(10.0 * std::log10(std::max(y, 1e-12)) >= 0.3)) {
        char m[200];
        std::snprintf(m, sizeof m,
                      "Y is %.2f dB: the source barely moves the noise (is it connected "
                      "and switching? is the DUT's NF beyond this source's ENR?)",
                      10.0 * std::log10(std::max(y, 1e-12)));
        *why = m;
        return false;
    }
    *f = lin(enr_db) / (y - 1.0);
    return true;
}

}  // namespace

bool EnrTable::load_text(const std::string &text, std::string *err) {
    std::vector<std::pair<double, double>> rows;
    std::istringstream in(text);
    std::string line;
    size_t no = 0;
    while (std::getline(in, line)) {
        ++no;
        const size_t h = line.find('#');
        if (h != std::string::npos) line.resize(h);
        for (char &c : line)
            if (c == ';' || c == '\t' || c == '=') c = ',';
        if (line.find_first_not_of(" ,\r") == std::string::npos) continue;
        const size_t comma = line.find(',');
        if (comma == std::string::npos) {
            *err = "line " + std::to_string(no) + ": expected freq,enr_db";
            return false;
        }
        std::string fs = line.substr(0, comma), es = line.substr(comma + 1);
        auto trim = [](std::string &s) {
            s.erase(0, s.find_first_not_of(" \r"));
            s.erase(s.find_last_not_of(" \r") + 1);
        };
        trim(fs);
        trim(es);
        double f = 0;
        char *end = nullptr;
        const double e = std::strtod(es.c_str(), &end);
        if (!parse_frequency(fs, &f) || !end || end == es.c_str()) {
            // A header row ("freq_hz,enr_db") is allowed once, at the top.
            if (rows.empty()) continue;
            *err = "line " + std::to_string(no) + ": cannot read \"" + fs + "," + es + "\"";
            return false;
        }
        rows.emplace_back(f, e);
    }
    if (rows.empty()) {
        *err = "no ENR points in the file";
        return false;
    }
    std::sort(rows.begin(), rows.end());
    f_.clear();
    enr_.clear();
    for (const auto &r : rows) {
        f_.push_back(r.first);
        enr_.push_back(r.second);
    }
    return true;
}

bool EnrTable::load_csv(const std::string &path, std::string *err) {
    std::ifstream f(path);
    if (!f) {
        *err = "cannot open " + path;
        return false;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return load_text(ss.str(), err);
}

bool EnrTable::enr_db(double f_hz, double *out) const {
    if (f_.empty() || f_hz < f_.front() - 1.0 || f_hz > f_.back() + 1.0) return false;
    if (f_.size() == 1) {
        *out = enr_[0];
        return true;
    }
    size_t k = size_t(std::upper_bound(f_.begin(), f_.end(), f_hz) - f_.begin());
    k = std::min(std::max<size_t>(k, 1), f_.size() - 1);
    const double t = (f_hz - f_[k - 1]) / (f_[k] - f_[k - 1]);
    *out = enr_[k - 1] + std::min(1.0, std::max(0.0, t)) * (enr_[k] - enr_[k - 1]);
    return true;
}

NfResult nf_receiver(double enr_db, double on, double off) {
    NfResult r;
    double f2 = 0;
    if (!factor(enr_db, on, off, &f2, &r.reason)) return r;
    r.nf2_db = db(f2);
    r.valid = true;
    return r;
}

NfResult nf_dut(double enr_db, const NfReadings &x) {
    NfResult r;
    double f2 = 0, f12 = 0;
    std::string why;
    if (!factor(enr_db, x.cal_on, x.cal_off, &f2, &why)) {
        r.reason = "calibration: " + why;
        return r;
    }
    if (!factor(enr_db, x.dut_on, x.dut_off, &f12, &why)) {
        r.reason = "measurement: " + why;
        return r;
    }
    const double g1 = (x.dut_on - x.dut_off) / (x.cal_on - x.cal_off);
    if (!(g1 > 0)) {
        r.reason = "the DUT's gain came out non-positive";
        return r;
    }
    const double f1 = f12 - (f2 - 1.0) / g1;
    r.nf2_db = db(f2);
    r.nf12_db = db(f12);
    r.gain_db = db(g1);
    r.low_gain = r.gain_db < 10.0;
    if (!(f1 > 0)) {
        r.reason = "the second-stage correction exceeds the cascade: the DUT gain is too "
                   "low for this receiver to separate its noise from the DUT's";
        return r;
    }
    // Below 1 (0 dB) is not physical, but it is what scatter in a low-gain
    // measurement produces; reported, not clamped, with low_gain saying why.
    r.nf1_db = db(f1);
    r.valid = true;
    return r;
}

}  // namespace sweepui
