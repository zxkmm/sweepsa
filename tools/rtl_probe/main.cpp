// Drives RtlSdrSource with no GUI and prints what it produced.
//
// Same role tools/hackrf_probe plays for the HackRF source and tools/sweep_client
// plays for the TCP source: a trace has to be checkable as numbers before it is
// judged as a picture, because a spectrum that is half a band out or scaled
// wrongly looks entirely plausible on a graticule and is obvious in a column of
// dBFS next to a known emitter.
//
//   rtl_probe --list
//   rtl_probe --start 88m --stop 108m --peaks 12
//   rtl_probe --start 88m --stop 108m --sweeps 20 --csv fm.csv
//
// The line worth reading first is `uncovered N of M bins`. Under the default
// splice policy it should be 0; anything else means the segment geometry and the
// output grid disagree.
//
// ---------------------------------------------------------------- --settle-scan
//
// This tool carries one job its HackRF counterpart does not. Per
// src/sweepui/rtlsdrsource.h, ~90% of an RTL-SDR sweep is a per-hop `usleep`
// whose length is **inherited from rtl_power and never measured**, so it is both
// the dominant cost and the least justified number in the source.
//
//   rtl_probe --start 98m --stop 100m --settle-scan 0,250,500,1000,2000,5000
//
// opens the source once per settle value and prints the sweep rate alongside
// what the trace actually looked like. Point it at a band with a strong steady
// carrier in it -- an FM broadcast station is ideal -- and read the table
// downward: the settle is long enough at the point where `max` stops climbing
// and `median` stops falling. Everything longer than that is sweep time being
// spent on nothing.
//
// That is CLAUDE.md §4b.2 for this radio, and until someone runs it the source
// keeps a conservative inherited default and docs/rtlsdr.md says so.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

#include "freq.h"
#include "rtlsdrsource.h"

using namespace sweepui;

namespace {

void usage() {
    std::fprintf(stderr,
"usage: rtl_probe [options]\n"
"\n"
"  --list             print the attached dongles and exit\n"
"\n"
"  --start FREQ       sweep start                      (default 88m)\n"
"  --stop FREQ        sweep stop                       (default 108m)\n"
"  --rate HZ          sample rate. librtlsdr takes 225001-300000 and\n"
"                     900001-3200000 and nothing between; above ~2.4m the\n"
"                     dongle drops samples and cannot say so.  (default 2.4m)\n"
"  --fft N            FFT size, power of two, 64-65536            (default 1024)\n"
"  --points N         output points, 0 = follow the achieved RBW     (default 0)\n"
"  --keep HZ          usable width per segment, 0 = fs/2             (default 0)\n"
"  --blank HZ         half-width blanked around each LO           (default 25k)\n"
"  --dc POLICY        splice|blank                              (default splice)\n"
"  --gain DB          snapped to the tuner's own list; a big number means\n"
"                     maximum                                       (default 30)\n"
"  --ppm N            crystal correction                             (default 0)\n"
"  --bw HZ            tuner IF filter, 0 = automatic                 (default 0)\n"
"  --direct N         0 off, 1 = I branch, 2 = Q branch. Bypasses the tuner\n"
"                     and samples HF directly.                       (default 0)\n"
"  --offset-tuning    E4000 only; librtlsdr refuses it on an R820T\n"
"  --bias-tee         put ~4.5 V on the antenna port to feed an inline LNA\n"
"  --settle US        per-hop settle. Most of the sweep time; see --settle-scan\n"
"                     and docs/rtlsdr.md.                         (default 5000)\n"
"  --dump BYTES       discarded after each retune to flush the pipeline\n"
"                     (default 4096, which is rtl_power's)\n"
"  --detector NAME    peak|negpeak|sample|rms                    (default peak)\n"
"  --window NAME      rect|hann|blackmanharris|flattop            (default hann)\n"
"  --serial S         which dongle, by serial\n"
"  --index N          which dongle, by index\n"
"\n"
"  --sweeps N         how many to collect                            (default 2)\n"
"  --csv PATH         write the last sweep as freq_hz,dbfs\n"
"  --peaks N          print the N strongest bins                     (default 0)\n"
"  --timeout S        give up if no sweep arrives                   (default 60)\n"
"  --settle-scan LIST comma-separated settle values in us; runs each and prints\n"
"                     the rate and the levels, so the settle can be measured\n"
"                     rather than inherited. Use a band with a steady carrier.\n"
"\n"
"  FREQ accepts instrument suffixes: 2.4g, 100m, 455k, or plain Hz.\n");
}

bool freq_arg(const char *s, double *out) {
    if (parse_frequency(s, out)) return true;
    std::fprintf(stderr, "cannot read \"%s\" as a frequency\n", s);
    return false;
}

void sleep_20ms() {
    struct timespec ts = {0, 20 * 1000 * 1000};
    nanosleep(&ts, nullptr);
}

// What one configuration produced. Deliberately a handful of order statistics
// rather than a whole trace: the settle scan is read as a table.
struct Summary {
    bool ok = false;
    std::string err;
    sweepd::proto::TraceMsg last;
    SourceConfig cfg;
    SourceStatus st;
    size_t uncovered = 0;
    double median_db = 0, max_db = 0;
};

Summary collect(const RtlSdrOptions &o, int want_sweeps, double timeout_s) {
    Summary s;
    RtlSdrSource src(o);
    if (!src.open(&s.err)) return s;

    int got = 0;
    for (double waited = 0; waited < timeout_s && got < want_sweeps; waited += 0.02) {
        sweepd::proto::TraceMsg t;
        while (src.poll(&t)) {
            s.last = std::move(t);
            ++got;
        }
        if (got >= want_sweeps) break;
        const SourceStatus st = src.status();
        if (!st.connected) {
            s.err = st.detail.empty() ? "the source stopped" : st.detail;
            return s;
        }
        sleep_20ms();
    }
    if (got == 0) {
        const SourceStatus st = src.status();
        char buf[320];
        std::snprintf(buf, sizeof buf, "no sweep arrived in %.0f s%s%s", timeout_s,
                      st.detail.empty() ? "" : ": ", st.detail.c_str());
        s.err = buf;
        return s;
    }

    s.cfg = src.config();
    s.st = src.status();

    std::vector<float> good;
    good.reserve(s.last.db.size());
    for (float v : s.last.db) {
        if (std::isnan(v))
            ++s.uncovered;
        else
            good.push_back(v);
    }
    if (!good.empty()) {
        std::sort(good.begin(), good.end());
        s.median_db = good[good.size() / 2];
        s.max_db = good.back();
    }
    s.ok = true;
    return s;
}

}  // namespace

int main(int argc, char **argv) {
    RtlSdrOptions o;
    int want_sweeps = 2, want_peaks = 0;
    double timeout_s = 60.0;
    std::string csv_path;
    std::vector<unsigned> settle_scan;
    bool list_only = false;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const char *v = (i + 1 < argc) ? argv[i + 1] : nullptr;
        auto need = [&]() -> const char * {
            if (!v) {
                std::fprintf(stderr, "%s needs a value\n", a.c_str());
                std::exit(2);
            }
            ++i;
            return v;
        };

        double f = 0;
        if (a == "--list") {
            list_only = true;
        } else if (a == "--start") {
            if (!freq_arg(need(), &f)) return 2;
            o.f_start_hz = (long long)llround(f);
        } else if (a == "--stop") {
            if (!freq_arg(need(), &f)) return 2;
            o.f_stop_hz = (long long)llround(f);
        } else if (a == "--rate") {
            if (!freq_arg(need(), &f)) return 2;
            o.fs_hz = f;
        } else if (a == "--fft") {
            o.n_fft = size_t(std::strtoul(need(), nullptr, 10));
        } else if (a == "--points") {
            o.n_points = size_t(std::strtoul(need(), nullptr, 10));
        } else if (a == "--keep") {
            if (!freq_arg(need(), &f)) return 2;
            o.keep_hz = (long long)llround(f);
        } else if (a == "--blank") {
            if (!freq_arg(need(), &f)) return 2;
            o.blank_hz = (long long)llround(f);
        } else if (a == "--dc") {
            if (!sweepd::dc_policy_from_string(need(), &o.dc)) {
                std::fprintf(stderr, "--dc wants splice or blank\n");
                return 2;
            }
        } else if (a == "--gain") {
            o.gain_db = std::strtod(need(), nullptr);
        } else if (a == "--ppm") {
            o.ppm = std::atoi(need());
        } else if (a == "--bw") {
            if (!freq_arg(need(), &f)) return 2;
            o.bw_hz = (long long)llround(f);
        } else if (a == "--direct") {
            o.direct_sampling = std::atoi(need());
        } else if (a == "--offset-tuning") {
            o.offset_tuning = true;
        } else if (a == "--bias-tee") {
            o.bias_tee = true;
        } else if (a == "--settle") {
            o.settle_us = unsigned(std::strtoul(need(), nullptr, 10));
        } else if (a == "--dump") {
            o.dump_bytes = size_t(std::strtoul(need(), nullptr, 10));
        } else if (a == "--detector") {
            if (!sweepd::detector_from_string(need(), &o.detector)) {
                std::fprintf(stderr, "unknown detector\n");
                return 2;
            }
        } else if (a == "--window") {
            if (!sweepd::window_from_string(need(), &o.window)) {
                std::fprintf(stderr, "unknown window\n");
                return 2;
            }
        } else if (a == "--serial") {
            o.serial = need();
        } else if (a == "--index") {
            o.index = std::atoi(need());
        } else if (a == "--sweeps") {
            want_sweeps = std::atoi(need());
        } else if (a == "--csv") {
            csv_path = need();
        } else if (a == "--peaks") {
            want_peaks = std::atoi(need());
        } else if (a == "--timeout") {
            timeout_s = std::strtod(need(), nullptr);
        } else if (a == "--settle-scan") {
            const std::string list = need();
            size_t pos = 0;
            while (pos <= list.size()) {
                const size_t comma = list.find(',', pos);
                const std::string tok = list.substr(pos, comma - pos);
                if (!tok.empty()) settle_scan.push_back(unsigned(std::strtoul(tok.c_str(),
                                                                              nullptr, 10)));
                if (comma == std::string::npos) break;
                pos = comma + 1;
            }
            if (settle_scan.empty()) {
                std::fprintf(stderr, "--settle-scan wants a comma-separated list of us\n");
                return 2;
            }
        } else if (a == "-h" || a == "--help") {
            usage();
            return 0;
        } else {
            std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
            usage();
            return 2;
        }
    }

    if (!rtlsdr_available()) {
        std::fprintf(stderr,
                     "rtl_probe: built without librtlsdr. Install librtlsdr-dev (or "
                     "rtl-sdr) and reconfigure CMake.\n");
        return 1;
    }

    if (list_only) {
        const std::vector<std::string> devs = rtlsdr_devices();
        if (devs.empty()) {
            std::fprintf(stderr,
                         "no RTL-SDR found. If one is attached, the kernel DVB driver has "
                         "probably claimed it: blacklist dvb_usb_rtl28xxu.\n");
            return 1;
        }
        for (const std::string &d : devs) std::printf("%s\n", d.c_str());
        return 0;
    }

    // ------------------------------------------------------------ settle scan
    if (!settle_scan.empty()) {
        std::printf("settle scan over %.3f-%.3f MHz. Read downward: the settle is long\n"
                    "enough where `max` stops climbing and `median` stops falling.\n\n",
                    double(o.f_start_hz) / 1e6, double(o.f_stop_hz) / 1e6);
        std::printf("  %9s %8s %9s %10s %10s %10s\n", "settle_us", "sweeps/s", "hops/s",
                    "median_db", "max_db", "uncovered");
        for (unsigned us : settle_scan) {
            RtlSdrOptions t = o;
            t.settle_us = us;
            const Summary s = collect(t, want_sweeps, timeout_s);
            if (!s.ok) {
                std::printf("  %9u  %s\n", us, s.err.c_str());
                continue;
            }
            std::printf("  %9u %8.2f %9.0f %10.2f %10.2f %10zu\n", us, s.st.sweeps_per_s,
                        s.st.hops_per_s, s.median_db, s.max_db, s.uncovered);
        }
        std::printf("\nEvery row is the same radio and the same gain, so the levels are\n"
                    "comparable to each other. None of them is calibrated (§7).\n");
        return 0;
    }

    // ------------------------------------------------------------ one sweep
    const Summary s = collect(o, want_sweeps, timeout_s);
    if (!s.ok) {
        std::fprintf(stderr, "rtl_probe: %s\n", s.err.c_str());
        return 1;
    }
    const sweepd::proto::TraceMsg &m = s.last;
    const SourceConfig &c = s.cfg;

    std::printf("%s\n", c.description.c_str());
    std::printf("span        %.6f - %.6f MHz   (tuner reaches %.3f - %.3f MHz)\n",
                double(c.f_start_hz) / 1e6, double(c.f_stop_hz) / 1e6,
                double(c.tune_lo()) / 1e6, double(c.tune_hi()) / 1e6);
    std::printf("points      %zu   bin %.1f Hz   rbw %.1f Hz (%s window)\n", c.n_points,
                c.bin_hz, c.rbw_hz, sweepd::window_name(c.window));
    std::printf("segments    %u   step %.3f MHz   keep %.3f MHz   blank +/-%.1f kHz (%s)\n",
                c.n_segments, c.seg_step_hz / 1e6, c.keep_hz / 1e6, c.blank_hz / 1e3,
                c.splice ? "splice" : "blank");
    std::printf("detector    %s   gain %.1f dB\n", sweepd::detector_name(c.detector),
                double(c.gain_db));
    std::printf("rate        %.2f sweeps/s, %.0f hops/s, last %.1f ms\n", s.st.sweeps_per_s,
                s.st.hops_per_s, s.st.last_sweep_ms);
    if (!s.st.detail.empty()) std::printf("note        %s\n", s.st.detail.c_str());

    // How much of the grid nothing covered. Under splice it should be 0; a
    // non-zero count is the first sign the plan and the output grid disagree.
    std::printf("uncovered   %zu of %zu bins%s\n", s.uncovered, m.db.size(),
                (s.uncovered && c.splice) ? "   <-- should be 0 under splice" : "");
    std::printf("overloaded  %u of %u segments\n", m.n_overloaded_segments, m.n_segments);
    std::printf("levels      median %.2f dBFS, max %.2f dBFS   (UNCAL: dBFS, not dBm)\n",
                s.median_db, s.max_db);

    if (want_peaks > 0) {
        std::vector<size_t> idx;
        idx.reserve(m.db.size());
        for (size_t i = 0; i < m.db.size(); ++i)
            if (!std::isnan(m.db[i])) idx.push_back(i);
        std::partial_sort(idx.begin(),
                          idx.begin() + std::min<size_t>(size_t(want_peaks), idx.size()),
                          idx.end(),
                          [&](size_t a, size_t b) { return m.db[a] > m.db[b]; });
        std::printf("\n  %-16s %10s\n", "freq_hz", "dbfs");
        for (int k = 0; k < want_peaks && size_t(k) < idx.size(); ++k)
            std::printf("  %-16.0f %10.2f\n", m.x_at(idx[k]), double(m.db[idx[k]]));
    }

    if (!csv_path.empty()) {
        FILE *f = std::fopen(csv_path.c_str(), "w");
        if (!f) {
            std::fprintf(stderr, "cannot write %s\n", csv_path.c_str());
            return 1;
        }
        std::fprintf(f, "freq_hz,dbfs\n");
        for (size_t i = 0; i < m.db.size(); ++i) {
            if (std::isnan(m.db[i]))
                std::fprintf(f, "%.0f,\n", m.x_at(i));
            else
                std::fprintf(f, "%.0f,%.3f\n", m.x_at(i), double(m.db[i]));
        }
        std::fclose(f);
        std::fprintf(stderr, "wrote %s (%zu points)\n", csv_path.c_str(), m.db.size());
    }
    return 0;
}
