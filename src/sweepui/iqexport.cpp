#include "iqexport.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <vector>

#include "iq.h"

namespace sweepui {
namespace {

bool ends_with(const std::string &s, const char *suf) {
    const size_t n = std::strlen(suf);
    if (s.size() < n) return false;
    for (size_t i = 0; i < n; ++i)
        if (std::tolower((unsigned char)s[s.size() - n + i]) != suf[i]) return false;
    return true;
}

struct File {
    FILE *f = nullptr;
    explicit File(const std::string &p) : f(std::fopen(p.c_str(), "wb")) {}
    ~File() {
        if (f) std::fclose(f);
    }
    bool close() {
        const bool ok = f && std::fclose(f) == 0;
        f = nullptr;
        return ok;
    }
};

void put_le32(unsigned char *q, uint32_t v) {
    for (int k = 0; k < 4; ++k) q[k] = (unsigned char)(v >> (8 * k));
}

// Floats out little-endian whatever the host is, in chunks.
bool write_cf32(FILE *f, const float *iq, size_t n) {
    unsigned char buf[8 * 1024];
    size_t i = 0;
    while (i < 2 * n) {
        const size_t m = std::min<size_t>(sizeof buf / 4, 2 * n - i);
        for (size_t k = 0; k < m; ++k) {
            uint32_t b;
            std::memcpy(&b, &iq[i + k], 4);
            put_le32(buf + 4 * k, b);
        }
        if (std::fwrite(buf, 4, m, f) != m) return false;
        i += m;
    }
    return true;
}

std::string json_escape(const std::string &s) {
    std::string o;
    for (char c : s) {
        if (c == '"' || c == '\\') {
            o += '\\';
            o += c;
        } else if ((unsigned char)c < 0x20) {
            char b[8];
            std::snprintf(b, sizeof b, "\\u%04x", c);
            o += b;
        } else {
            o += c;
        }
    }
    return o;
}

std::string describe(const sweepd::proto::IqMsg &m, const std::string &note) {
    char b[512];
    std::snprintf(b, sizeof b,
                  "%s%sraw IQ burst, centre %lld Hz, fs %.6f Hz (adc %.6f Hz / %u), gain %.1f dB, "
                  "trigger %s, %u clipped input samples",
                  note.c_str(), note.empty() ? "" : " -- ", m.center_hz, m.fs_hz, m.adc_fs_hz,
                  m.decim, double(m.gain_db), sweepd::proto::iq_trig_state_name(m.trig_state),
                  m.n_clipped);
    return b;
}

}  // namespace

bool iq_kind_from_path(const std::string &path, IqFileKind *out) {
    if (ends_with(path, ".cfile") || ends_with(path, ".cf32") || ends_with(path, ".fc32"))
        *out = IqFileKind::Cfile;
    else if (ends_with(path, ".sc16") || ends_with(path, ".iq16") || ends_with(path, ".cs16"))
        *out = IqFileKind::Sc16;
    else if (ends_with(path, ".csv"))
        *out = IqFileKind::Csv;
    else if (ends_with(path, ".sigmf-meta") || ends_with(path, ".sigmf-data") ||
             ends_with(path, ".sigmf"))
        *out = IqFileKind::SigMF;
    else
        return false;
    return true;
}

bool write_iq_file(const std::string &path, const sweepd::proto::IqMsg &m, size_t i0,
                   size_t i1, const std::string &note, std::string *err,
                   std::string *written) {
    auto fail = [&](const std::string &e) {
        if (err) *err = e;
        return false;
    };
    IqFileKind kind;
    if (!iq_kind_from_path(path, &kind))
        return fail("unknown IQ file type; use .cfile, .sc16, .csv or .sigmf-meta");
    const size_t n_all = m.n_samples();
    i1 = std::min(i1, n_all);
    if (i0 >= i1) return fail("no samples in the range to export");
    const size_t n = i1 - i0;
    const float *iq = m.iq.data() + 2 * i0;
    if (written) *written = path;

    if (kind == IqFileKind::Cfile) {
        File f(path);
        if (!f.f) return fail("cannot open " + path);
        if (!write_cf32(f.f, iq, n) || !f.close()) return fail("write failed: " + path);
        return true;
    }

    if (kind == IqFileKind::Sc16) {
        File f(path);
        if (!f.f) return fail("cannot open " + path);
        const double fs = m.full_scale > 0 ? double(m.full_scale) : 2048.0;
        std::vector<unsigned char> buf(4 * 4096);
        for (size_t i = 0; i < n;) {
            const size_t c = std::min<size_t>(4096, n - i);
            for (size_t k = 0; k < 2 * c; ++k) {
                const long v = std::lround(double(iq[2 * i + k]) * fs);
                const uint16_t u = uint16_t(int16_t(std::max(-32768L, std::min(32767L, v))));
                buf[2 * k] = (unsigned char)u;
                buf[2 * k + 1] = (unsigned char)(u >> 8);
            }
            if (std::fwrite(buf.data(), 4, c, f.f) != c) return fail("write failed: " + path);
            i += c;
        }
        if (!f.close()) return fail("write failed: " + path);
        return true;
    }

    if (kind == IqFileKind::Csv) {
        File f(path);
        if (!f.f) return fail("cannot open " + path);
        std::fprintf(f.f, "# sweepsa raw IQ export\n# %s\n", describe(m, note).c_str());
        std::fprintf(f.f, "# center_hz=%lld fs_hz=%.6f decim=%u gain_db=%.1f t0_ns=%lld "
                          "first_sample=%zu units=full_scale\n",
                     m.center_hz, m.fs_hz, m.decim, double(m.gain_db), (long long)m.t0_ns, i0);
        std::fprintf(f.f, "t_s,i,q,mag,phase_deg\n");
        for (size_t k = 0; k < n; ++k) {
            const double a = iq[2 * k], b = iq[2 * k + 1];
            std::fprintf(f.f, "%.12g,%.9g,%.9g,%.9g,%.6g\n", m.t_at(i0 + k), a, b,
                         std::sqrt(a * a + b * b), std::atan2(b, a) * 180.0 / M_PI);
        }
        if (!f.close()) return fail("write failed: " + path);
        return true;
    }

    // SigMF: <base>.sigmf-data and <base>.sigmf-meta.
    std::string base = path;
    for (const char *suf : {".sigmf-meta", ".sigmf-data", ".sigmf"}) {
        if (ends_with(base, suf)) {
            base.resize(base.size() - std::strlen(suf));
            break;
        }
    }
    const std::string data = base + ".sigmf-data", meta = base + ".sigmf-meta";
    {
        File f(data);
        if (!f.f) return fail("cannot open " + data);
        if (!write_cf32(f.f, iq, n) || !f.close()) return fail("write failed: " + data);
    }
    // The host's clock, which is a real one (§2: the device's is not).
    char when[40] = "";
    const std::time_t now = std::time(nullptr);
    std::tm tmv{};
#if defined(_WIN32)
    gmtime_s(&tmv, &now);
#else
    gmtime_r(&now, &tmv);
#endif
    std::strftime(when, sizeof when, "%Y-%m-%dT%H:%M:%SZ", &tmv);

    File f(meta);
    if (!f.f) return fail("cannot open " + meta);
    std::fprintf(f.f,
                 "{\n  \"global\": {\n"
                 "    \"core:datatype\": \"cf32_le\",\n"
                 "    \"core:sample_rate\": %.9g,\n"
                 "    \"core:version\": \"1.0.0\",\n"
                 "    \"core:recorder\": \"sweepsa sweepui\",\n"
                 "    \"core:hw\": \"AD936x direct-conversion receiver, gain %.1f dB, "
                 "RF bandwidth %.0f Hz\",\n"
                 "    \"core:description\": \"%s\"\n  },\n"
                 "  \"captures\": [\n    {\"core:sample_start\": 0, \"core:frequency\": %lld, "
                 "\"core:datetime\": \"%s\"}\n  ],\n  \"annotations\": [",
                 m.fs_hz, double(m.gain_db), m.rf_bw_hz, json_escape(describe(m, note)).c_str(),
                 m.center_hz, when);
    if (m.trig_state == sweepd::proto::IqTrigState::Triggered && m.trig_index >= i0 &&
        m.trig_index < i1) {
        std::fprintf(f.f,
                     "\n    {\"core:sample_start\": %zu, \"core:sample_count\": 1, "
                     "\"core:label\": \"trigger\", \"core:comment\": \"%s edge on %s at %.6g FS\"}\n  ",
                     size_t(m.trig_index) - i0, m.trig_rising ? "rising" : "falling",
                     sweepd::iq_trig_source_name(sweepd::IqTrigSource(m.trig_source)),
                     double(m.trig_level));
    }
    std::fprintf(f.f, "]\n}\n");
    if (!f.close()) return fail("write failed: " + meta);
    if (written) *written = data;
    return true;
}

}  // namespace sweepui
