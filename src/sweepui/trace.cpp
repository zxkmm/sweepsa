#include "trace.h"

#include <cmath>
#include <cstring>
#include <limits>

namespace sweepui {
namespace {

const float kNan = std::numeric_limits<float>::quiet_NaN();

// dB -> linear power and back. The floor sweepd applies (power_to_db's
// -200 dBFS) round-trips through these without becoming zero, so a floored bin
// stays a number rather than turning into a hole.
inline double db_to_power(float db) { return std::pow(10.0, double(db) / 10.0); }
inline float power_to_db(double p) {
    return p > 0 ? float(10.0 * std::log10(p)) : -200.0f;
}

}  // namespace

const char *trace_mode_name(TraceMode m) {
    switch (m) {
        case TraceMode::ClearWrite: return "clear/write";
        case TraceMode::MaxHold:    return "max hold";
        case TraceMode::MinHold:    return "min hold";
        case TraceMode::Average:    return "average";
        case TraceMode::View:       return "view";
        case TraceMode::Blank:      return "blank";
    }
    return "?";
}

bool trace_mode_from_string(const std::string &s, TraceMode *out) {
    if (s == "clear/write" || s == "clearwrite" || s == "write") {
        *out = TraceMode::ClearWrite;
    } else if (s == "max hold" || s == "maxhold" || s == "max") {
        *out = TraceMode::MaxHold;
    } else if (s == "min hold" || s == "minhold" || s == "min") {
        *out = TraceMode::MinHold;
    } else if (s == "average" || s == "avg") {
        *out = TraceMode::Average;
    } else if (s == "view") {
        *out = TraceMode::View;
    } else if (s == "blank") {
        *out = TraceMode::Blank;
    } else {
        return false;
    }
    return true;
}

bool trace_mode_accumulates(TraceMode m) {
    return m == TraceMode::ClearWrite || m == TraceMode::MaxHold ||
           m == TraceMode::MinHold || m == TraceMode::Average;
}

// ---------------------------------------------------------------- Trace

void Trace::set_mode(TraceMode m) {
    if (m == mode_) return;
    const TraceMode was = mode_;
    mode_ = m;

    // Entering an accumulating mode clears. A max hold that inherited whatever
    // the previous mode left in the buffer would report a maximum it never
    // held: the obvious case is View -> Max Hold, where the frozen trace is a
    // sweep from an unknown time ago and would sit in the hold forever. Leaving
    // an accumulating mode keeps the data, so View and Blank freeze what is on
    // screen rather than erasing it.
    if (trace_mode_accumulates(m) && was != m) clear();
}

void Trace::set_average_n(int n) {
    if (n < 1) n = 1;
    if (n == average_n_) return;
    average_n_ = n;
    if (mode_ == TraceMode::Average) clear();
}

void Trace::clear() {
    db_.clear();
    mean_power_.clear();
    count_.clear();
    sweeps_ = 0;
}

void Trace::reset_to(size_t n) {
    db_.assign(n, kNan);
    if (mode_ == TraceMode::Average) {
        mean_power_.assign(n, 0.0);
        count_.assign(n, 0u);
    } else {
        mean_power_.clear();
        count_.clear();
    }
    sweeps_ = 0;
}

void Trace::update(const float *db, size_t n) {
    if (!db || n == 0) return;
    if (!trace_mode_accumulates(mode_)) return;
    if (db_.size() != n) reset_to(n);

    switch (mode_) {
        case TraceMode::ClearWrite:
            // The newest sweep verbatim, NaN included: a hole in this sweep is
            // a hole in what Clear/Write is showing.
            std::memcpy(db_.data(), db, n * sizeof(float));
            break;

        case TraceMode::MaxHold:
            for (size_t i = 0; i < n; ++i) {
                const float v = db[i];
                if (std::isnan(v)) continue;          // unmeasured: no evidence
                if (std::isnan(db_[i]) || v > db_[i]) db_[i] = v;
            }
            break;

        case TraceMode::MinHold:
            for (size_t i = 0; i < n; ++i) {
                const float v = db[i];
                if (std::isnan(v)) continue;
                if (std::isnan(db_[i]) || v < db_[i]) db_[i] = v;
            }
            break;

        case TraceMode::Average:
            // Linear power, per the header note. Two regimes, which is what
            // instruments do and what makes Average(N) usable: until N sweeps
            // have arrived the trace is the true mean of the sweeps so far, so
            // it converges instead of crawling up from zero; after that the
            // weight stays at 1/N so it keeps tracking rather than freezing.
            for (size_t i = 0; i < n; ++i) {
                const float v = db[i];
                if (std::isnan(v)) continue;
                const double p = db_to_power(v);
                const uint32_t c = count_[i];
                const double w = (c < uint32_t(average_n_)) ? 1.0 / double(c + 1)
                                                            : 1.0 / double(average_n_);
                mean_power_[i] += (p - mean_power_[i]) * w;
                if (c < uint32_t(average_n_)) count_[i] = c + 1;
                db_[i] = power_to_db(mean_power_[i]);
            }
            break;

        case TraceMode::View:
        case TraceMode::Blank:
            return;   // unreachable; trace_mode_accumulates() screened these
    }

    ++sweeps_;
}

// ---------------------------------------------------------------- TraceStore

bool TraceStore::push(const sweepd::proto::TraceMsg &f) {
    if (f.db.empty()) return false;

    // x_kind and dt_ns join the key because a hold has to clear when the data
    // stops being comparable, and switching between a sweep and a zero span is
    // the largest possible version of that: the axis changes units. A max hold
    // carried across the switch would draw peaks found at frequencies on top of
    // a trace measured over time -- a picture of a measurement that never
    // happened, which is the same failure the detector and window are in this
    // key to prevent.
    const bool moved = have_key_ && (f.f_start_hz != f_start_hz_ ||
                                     f.f_stop_hz != f_stop_hz_ ||
                                     f.db.size() != n_points_ ||
                                     f.detector != detector_ || f.window != window_ ||
                                     f.x_kind != x_kind_ || f.dt_ns != dt_ns_);
    if (moved) clear_all();

    have_key_ = true;
    f_start_hz_ = f.f_start_hz;
    f_stop_hz_ = f.f_stop_hz;
    n_points_ = f.db.size();
    detector_ = f.detector;
    window_ = f.window;
    x_kind_ = f.x_kind;
    dt_ns_ = f.dt_ns;

    for (size_t i = 0; i < kCount; ++i) t_[i].update(f.db.data(), f.db.size());
    return moved;
}

void TraceStore::clear_all() {
    for (size_t i = 0; i < kCount; ++i) t_[i].clear();
}

size_t TraceStore::first_visible() const {
    for (size_t i = 0; i < kCount; ++i)
        if (t_[i].visible()) return i;
    return kCount;
}

}  // namespace sweepui
