// Drives HackRfSource with no GUI and prints what it produced.
//
// The same role tools/sweep_client plays for the §10.4 TCP source, and it
// exists for the same reason §5 requires sweepd to have a standalone CSV mode:
// a source has to be checkable as numbers before it is judged as a picture. A
// trace that is half a band out, or scaled wrongly, looks entirely plausible on
// a graticule and is obvious in a column of dBFS next to a known emitter.
//
//   hackrf_probe --start 88m --stop 108m --sweeps 3 --csv fm.csv
//   hackrf_probe --start 88m --stop 108m --peaks 10
//
// --peaks is the quick check: it prints the strongest bins, which for an FM
// band with any antenna attached should land on actual broadcast carriers.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "freq.h"
#include "hackrfsource.h"

using namespace sweepui;

namespace {

void usage() {
    std::fprintf(stderr,
"usage: hackrf_probe [options]\n"
"\n"
"  --start FREQ       sweep start                      (default 70m)\n"
"  --stop FREQ        sweep stop                       (default 6g)\n"
"  --rate HZ          sample rate, 2-20 MHz            (default 20m)\n"
"  --fft N            FFT size, power of two, 64-4096  (default 4096)\n"
"  --points N         output points, 0 = follow the achieved RBW (default 0)\n"
"  --detector NAME    peak|negpeak|sample|rms          (default peak)\n"
"  --window NAME      rect|hann|blackmanharris|flattop (default hann)\n"
"  --lna DB           IF gain, 0-40 in 8 dB steps      (default 16)\n"
"  --vga DB           baseband gain, 0-62 in 2 dB steps (default 20)\n"
"  --amp              enable the ~11 dB RF amplifier\n"
"  --bw HZ            baseband filter, 0 = nearest to the sample rate\n"
"  --sweeps N         how many to collect                        (default 2)\n"
"  --csv PATH         write the last sweep as freq_hz,dbfs\n"
"  --peaks N          print the N strongest bins                 (default 0)\n"
"  --timeout S        give up if no sweep arrives                (default 30)\n");
}

bool freq_arg(const char *s, double *out) {
    if (parse_frequency(s, out)) return true;
    std::fprintf(stderr, "cannot read \"%s\" as a frequency\n", s);
    return false;
}

}  // namespace

int main(int argc, char **argv) {
    HackRfOptions o;
    int want_sweeps = 2, want_peaks = 0;
    double timeout_s = 30.0;
    std::string csv_path;

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
        if (a == "--start") {
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
        } else if (a == "--lna") {
            o.lna_db = std::atoi(need());
        } else if (a == "--vga") {
            o.vga_db = std::atoi(need());
        } else if (a == "--amp") {
            o.amp = true;
        } else if (a == "--bw") {
            if (!freq_arg(need(), &f)) return 2;
            o.bb_filter_hz = f;
        } else if (a == "--sweeps") {
            want_sweeps = std::atoi(need());
        } else if (a == "--csv") {
            csv_path = need();
        } else if (a == "--peaks") {
            want_peaks = std::atoi(need());
        } else if (a == "--timeout") {
            timeout_s = std::strtod(need(), nullptr);
        } else if (a == "-h" || a == "--help") {
            usage();
            return 0;
        } else {
            std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
            usage();
            return 2;
        }
    }

    if (!hackrf_available()) {
        std::fprintf(stderr, "built without libhackrf\n");
        return 1;
    }

    HackRfSource src(o);
    std::string err;
    if (!src.open(&err)) {
        std::fprintf(stderr, "hackrf_probe: %s\n", err.c_str());
        return 1;
    }

    sweepd::proto::TraceMsg m;
    int got = 0;
    const double t0 = double(clock()) / CLOCKS_PER_SEC;
    (void)t0;

    // Poll rather than block: the source hands frames over a queue and the
    // discovery sweep has to finish first.
    for (double waited = 0; waited < timeout_s && got < want_sweeps; waited += 0.02) {
        sweepd::proto::TraceMsg t;
        while (src.poll(&t)) {
            m = std::move(t);
            ++got;
        }
        if (got >= want_sweeps) break;
        const SourceStatus st = src.status();
        if (!st.connected) {
            std::fprintf(stderr, "hackrf_probe: %s\n", st.detail.c_str());
            return 1;
        }
        struct timespec ts = {0, 20 * 1000 * 1000};
        nanosleep(&ts, nullptr);
    }

    if (got == 0) {
        const SourceStatus st = src.status();
        std::fprintf(stderr, "hackrf_probe: no sweep arrived in %.0f s%s%s\n", timeout_s,
                     st.detail.empty() ? "" : ": ", st.detail.c_str());
        return 1;
    }

    const SourceConfig c = src.config();
    const SourceStatus st = src.status();

    std::printf("%s\n", c.description.c_str());
    std::printf("span        %.6f - %.6f MHz\n", double(c.f_start_hz) / 1e6,
                double(c.f_stop_hz) / 1e6);
    std::printf("points      %zu   bin %.1f Hz   rbw %.1f Hz (%s window)\n", c.n_points,
                c.bin_hz, c.rbw_hz, sweepd::window_name(c.window));
    std::printf("tunings     %u   detector %s   gain %.0f dB\n", c.n_segments,
                sweepd::detector_name(c.detector), double(c.gain_db));
    std::printf("rate        %.2f sweeps/s, %.0f tunings/s, last %.1f ms\n",
                st.sweeps_per_s, st.hops_per_s, st.last_sweep_ms);
    if (!st.detail.empty()) std::printf("note        %s\n", st.detail.c_str());

    // How much of the grid nothing covered. Should be zero: the interleaved
    // partner tuning is supposed to fill every hole, and a non-zero count here
    // is the first sign the block geometry is wrong.
    size_t nan_bins = 0;
    for (float v : m.db)
        if (std::isnan(v)) ++nan_bins;
    std::printf("uncovered   %zu of %zu bins\n", nan_bins, m.db.size());
    std::printf("overloaded  %u of %u tunings\n", m.n_overloaded_segments, m.n_segments);

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
