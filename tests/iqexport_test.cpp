// Every IQ export format written and read back (src/sweepui/iqexport.h). A file
// that opens in GNU Radio but is off by a factor, byte-swapped, or one sample
// short looks exactly like a valid capture of a different signal.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "iqexport.h"

using namespace sweepui;
using sweepd::proto::IqMsg;

static int g_fail = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);      \
            ++g_fail;                                                          \
        }                                                                      \
    } while (0)

static std::string tmp(const char *name) {
    const char *dir = std::getenv("TMPDIR");
    std::string d = dir && *dir ? dir : "/tmp";
    if (d.back() != '/') d += '/';
    return d + name;
}

static std::vector<unsigned char> slurp(const std::string &p) {
    std::ifstream f(p, std::ios::binary);
    return std::vector<unsigned char>((std::istreambuf_iterator<char>(f)),
                                      std::istreambuf_iterator<char>());
}

int main() {
    IqMsg m;
    m.center_hz = 433895000LL;
    m.adc_fs_hz = 61.44e6;
    m.decim = 1;
    m.fs_hz = 61.44e6;
    m.full_scale = 2048;
    m.t0_ns = -100;
    m.trig_state = sweepd::proto::IqTrigState::Triggered;
    m.trig_index = 2;
    for (int i = 0; i < 8; ++i) {
        m.iq.push_back(float(i) / 2048.0f);
        m.iq.push_back(-float(i) / 1024.0f);
    }

    std::string err;
    IqFileKind k;
    CHECK(iq_kind_from_path("a.CFILE", &k) && k == IqFileKind::Cfile);
    CHECK(iq_kind_from_path("a.sigmf", &k) && k == IqFileKind::SigMF);
    CHECK(!iq_kind_from_path("a.bin", &k));

    // cfile: samples [1, 5) as little-endian float32 pairs.
    const std::string cf = tmp("sweepsa_iqx.cfile");
    CHECK(write_iq_file(cf, m, 1, 5, "", &err));
    auto b = slurp(cf);
    CHECK(b.size() == 4 * 8);
    if (b.size() == 32) {
        float v[8];
        std::memcpy(v, b.data(), 32);   // the test host is little-endian
        CHECK(v[0] == 1 / 2048.0f && v[1] == -1 / 1024.0f && v[6] == 4 / 2048.0f);
    }

    // sc16: the ADC's own counts back.
    const std::string sc = tmp("sweepsa_iqx.sc16");
    CHECK(write_iq_file(sc, m, 0, 8, "", &err));
    b = slurp(sc);
    CHECK(b.size() == 8 * 4);
    if (b.size() == 32) {
        int16_t v[16];
        std::memcpy(v, b.data(), 32);
        CHECK(v[2] == 1 && v[3] == -2 && v[14] == 7 && v[15] == -14);
    }

    // csv: comment header, column header, one row per sample, with time.
    const std::string cs = tmp("sweepsa_iqx.csv");
    CHECK(write_iq_file(cs, m, 2, 4, "SYNTHETIC", &err));
    std::ifstream f(cs);
    std::string line, all;
    int rows = 0;
    while (std::getline(f, line)) {
        all += line + "\n";
        if (!line.empty() && line[0] != '#' && line[0] != 't') ++rows;
    }
    CHECK(rows == 2);
    CHECK(all.find("SYNTHETIC") != std::string::npos);
    CHECK(all.find("fs_hz=61440000") != std::string::npos);
    CHECK(all.find("t_s,i,q,mag,phase_deg") != std::string::npos);

    // SigMF: both halves, the rate and frequency in the metadata, the trigger
    // annotated at its index relative to the exported range.
    const std::string sm = tmp("sweepsa_iqx.sigmf-meta");
    std::string data;
    CHECK(write_iq_file(sm, m, 1, 8, "", &err, &data));
    CHECK(data == tmp("sweepsa_iqx.sigmf-data"));
    CHECK(slurp(data).size() == 7 * 8);
    const auto mb = slurp(sm);
    const std::string meta(mb.begin(), mb.end());
    CHECK(meta.find("\"core:datatype\": \"cf32_le\"") != std::string::npos);
    CHECK(meta.find("\"core:sample_rate\": 61440000") != std::string::npos);
    CHECK(meta.find("\"core:frequency\": 433895000") != std::string::npos);
    CHECK(meta.find("\"core:sample_start\": 1, \"core:sample_count\": 1") != std::string::npos);

    // Empty range is refused.
    CHECK(!write_iq_file(cf, m, 5, 5, "", &err));

    for (const std::string &p : {cf, sc, cs, sm, data}) std::remove(p.c_str());
    if (g_fail) {
        std::printf("%d check(s) failed\n", g_fail);
        return 1;
    }
    std::printf("all iqexport checks passed\n");
    return 0;
}
