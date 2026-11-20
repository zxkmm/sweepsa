// Host-side tests for export. CLAUDE.md §6: "trace CSV, screenshot PNG,
// instrument state to JSON."
//
// The PNG half of this file contains a small inflate. That is deliberate: the
// encoder in export.cpp writes a fixed-Huffman deflate stream by hand, and the
// only test that means anything about a compressed format is one that decodes
// it back and compares. Checking the encoder against itself -- recomputing its
// own CRCs with its own function -- would pass on a stream no other program
// could read. The decoder here is written from RFC 1951 independently of the
// encoder, so the two agreeing is evidence.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "../src/sweepui/export.h"

using namespace sweepui;

namespace {

int g_failures = 0;
const float kNan = std::numeric_limits<float>::quiet_NaN();

void check(const char *what, bool ok) {
    std::printf("  %-68s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) ++g_failures;
}

bool contains(const std::string &hay, const std::string &needle) {
    return hay.find(needle) != std::string::npos;
}

// ------------------------------------------------------------------- CSV

void test_csv_carries_what_makes_it_readable_later() {
    std::printf("the CSV header carries the RBW, the window and the detector\n");
    std::vector<float> a = {-10.0f, -20.0f, kNan, -40.0f};
    std::vector<float> b = {-11.0f, -21.0f, -31.0f, -41.0f};

    CsvMeta m;
    m.source = "synthetic";
    m.f_start_hz = 100000000;
    m.f_stop_hz = 104000000;
    m.bin_hz = 1e6;
    m.rbw_hz = 12345.5;
    m.detector = sweepd::Detector::RMS;
    m.window = sweepd::Window::FlatTop;
    m.gain_db = 30.0f;
    m.uncal = true;
    m.synthetic = true;

    std::vector<CsvColumn> cols = {{"trace1", "max hold", a.data(), a.size()},
                                   {"trace2", "average(16)", b.data(), b.size()}};
    std::string csv, err;
    check("it writes", format_trace_csv(m, cols, &csv, &err));

    check("the RBW is in the header", contains(csv, "rbw_hz=12345.500"));
    // The same spellings sweepd's own CSV uses, so one reader handles both
    // files and a script does not have to know which end wrote it.
    check("so is the window", contains(csv, "window=flat-top"));
    check("and the detector", contains(csv, "detector=rms"));
    check("and the units, spelled out", contains(csv, "units=dBFS"));
    check("§7's uncal state survives into the file", contains(csv, "uncal=true"));
    check("so does the fact that nothing measured this", contains(csv, "SYNTHETIC"));
    check("each column says what mode produced it", contains(csv, "trace1: max hold"));
    check("the column header row is there", contains(csv, "\nfreq_hz,trace1,trace2\n"));

    // Bin centres, matching decimate.cpp and marker.cpp: f_start + (i+0.5)*bin.
    check("the first row is the first bin's centre, not its edge",
          contains(csv, "\n100500000,-10.00,-11.00\n"));
    check("an unmeasured bin is written as nan, not as a number",
          contains(csv, "\n102500000,nan,-31.00\n"));
}

void test_csv_refuses_to_misalign_its_own_columns() {
    std::printf("mismatched columns are refused, not truncated\n");
    std::vector<float> a = {-10.0f, -20.0f};
    std::vector<float> b = {-11.0f};
    std::vector<CsvColumn> cols = {{"trace1", "write", a.data(), a.size()},
                                   {"trace2", "write", b.data(), b.size()}};
    std::string csv, err;
    check("it refuses", !format_trace_csv(CsvMeta(), cols, &csv, &err));
    check("and says why", contains(err, "misaligned"));

    check("an empty export is refused too",
          !format_trace_csv(CsvMeta(), {}, &csv, &err));
}

// ------------------------------------------------------------------- PNG
//
// A fixed-Huffman / stored-block inflate, from RFC 1951. Enough to decode what
// export.cpp writes and nothing more -- there are no dynamic-Huffman blocks in
// this project's output, and a decoder that handled them would be a second
// implementation to get wrong.

struct Inflater {
    const uint8_t *p = nullptr;
    size_t n = 0, pos = 0;
    int bit = 0;
    bool err = false;

    uint32_t getbit() {
        if (pos >= n) {
            err = true;
            return 0;
        }
        const uint32_t b = (uint32_t(p[pos]) >> bit) & 1u;
        if (++bit == 8) {
            bit = 0;
            ++pos;
        }
        return b;
    }
    // Everything except a Huffman code is least-significant bit first.
    uint32_t getbits(int k) {
        uint32_t v = 0;
        for (int i = 0; i < k; ++i) v |= getbit() << i;
        return v;
    }
    // A Huffman code is most-significant bit first, which is the trap the
    // encoder's comment names.
    int decode_fixed_literal() {
        uint32_t code = 0;
        for (int len = 1; len <= 9; ++len) {
            code = (code << 1) | getbit();
            if (err) return -1;
            if (len == 7 && code <= 0x17) return 256 + int(code);
            if (len == 8) {
                if (code >= 0x30 && code <= 0xBF) return int(code) - 0x30;
                if (code >= 0xC0 && code <= 0xC7) return 280 + int(code) - 0xC0;
            }
            if (len == 9 && code >= 0x190 && code <= 0x1FF) return 144 + int(code) - 0x190;
        }
        err = true;
        return -1;
    }
};

const uint16_t kLenBase[29] = {3,  4,  5,  6,  7,  8,  9,  10,  11,  13,
                               15, 17, 19, 23, 27, 31, 35, 43,  51,  59,
                               67, 83, 99, 115, 131, 163, 195, 227, 258};
const uint8_t kLenExtra[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2,
                               2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
const uint16_t kDistBase[30] = {1,    2,    3,    4,    5,    7,     9,     13,
                                17,   25,   33,   49,   65,   97,    129,   193,
                                257,  385,  513,  769,  1025, 1537,  2049,  3073,
                                4097, 6145, 8193, 12289, 16385, 24577};
const uint8_t kDistExtra[30] = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4,  4,  5,  5,  6,
                                6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};

bool inflate_raw(const uint8_t *p, size_t n, std::vector<uint8_t> *out) {
    Inflater z;
    z.p = p;
    z.n = n;
    for (;;) {
        const uint32_t final_block = z.getbits(1);
        const uint32_t type = z.getbits(2);
        if (z.err) return false;
        if (type == 0) {
            if (z.bit) {
                z.bit = 0;
                ++z.pos;
            }
            if (z.pos + 4 > z.n) return false;
            const uint32_t len = uint32_t(z.p[z.pos]) | (uint32_t(z.p[z.pos + 1]) << 8);
            z.pos += 4;
            if (z.pos + len > z.n) return false;
            out->insert(out->end(), z.p + z.pos, z.p + z.pos + len);
            z.pos += len;
        } else if (type == 1) {
            for (;;) {
                const int sym = z.decode_fixed_literal();
                if (z.err) return false;
                if (sym < 256) {
                    out->push_back(uint8_t(sym));
                    continue;
                }
                if (sym == 256) break;
                const int li = sym - 257;
                if (li < 0 || li > 28) return false;
                const size_t len = kLenBase[li] + z.getbits(kLenExtra[li]);
                uint32_t dsym = 0;
                for (int i = 0; i < 5; ++i) dsym = (dsym << 1) | z.getbit();
                if (z.err || dsym > 29) return false;
                const size_t dist = kDistBase[dsym] + z.getbits(kDistExtra[dsym]);
                if (dist == 0 || dist > out->size()) return false;
                for (size_t i = 0; i < len; ++i)
                    out->push_back((*out)[out->size() - dist]);
            }
        } else {
            return false;
        }
        if (final_block) break;
    }
    return !z.err;
}

uint32_t be32(const uint8_t *p) {
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) |
           uint32_t(p[3]);
}

// Walks the chunk list, checking every CRC, and returns the IDAT payload.
bool parse_png(const std::vector<uint8_t> &png, int *w, int *h,
               std::vector<uint8_t> *idat, std::string *why) {
    const uint8_t sig[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    if (png.size() < 8 || std::memcmp(png.data(), sig, 8) != 0) {
        *why = "bad signature";
        return false;
    }
    size_t i = 8;
    bool saw_ihdr = false, saw_iend = false;
    while (i + 12 <= png.size()) {
        const uint32_t len = be32(&png[i]);
        if (i + 12 + len > png.size()) {
            *why = "chunk runs past the end of the file";
            return false;
        }
        const char *type = (const char *)&png[i + 4];
        const uint8_t *data = &png[i + 8];
        const uint32_t want = be32(&png[i + 8 + len]);
        if (crc32_of(&png[i + 4], size_t(len) + 4) != want) {
            *why = std::string("bad CRC on ") + std::string(type, 4);
            return false;
        }
        if (std::memcmp(type, "IHDR", 4) == 0) {
            if (len != 13) {
                *why = "IHDR is not 13 bytes";
                return false;
            }
            *w = int(be32(data));
            *h = int(be32(data + 4));
            if (data[8] != 8 || data[9] != 2 || data[10] != 0 || data[11] != 0 ||
                data[12] != 0) {
                *why = "IHDR is not 8-bit non-interlaced RGB";
                return false;
            }
            saw_ihdr = true;
        } else if (std::memcmp(type, "IDAT", 4) == 0) {
            idat->insert(idat->end(), data, data + len);
        } else if (std::memcmp(type, "IEND", 4) == 0) {
            saw_iend = true;
        }
        i += 12 + len;
    }
    if (i != png.size()) {
        *why = "trailing bytes after the last chunk";
        return false;
    }
    if (!saw_ihdr || !saw_iend) {
        *why = "missing IHDR or IEND";
        return false;
    }
    return true;
}

// The image the encoder should have produced, as PNG scanlines.
std::vector<uint8_t> expected_scanlines(const std::vector<uint8_t> &rgb, int w, int h,
                                        bool bottom_up) {
    const size_t stride = size_t(w) * 3;
    std::vector<uint8_t> raw;
    for (int y = 0; y < h; ++y) {
        const int sy = bottom_up ? (h - 1 - y) : y;
        raw.push_back(0);
        raw.insert(raw.end(), rgb.begin() + long(size_t(sy) * stride),
                   rgb.begin() + long(size_t(sy) * stride + stride));
    }
    return raw;
}

bool round_trip_png(const std::vector<uint8_t> &rgb, int w, int h, bool bottom_up,
                    std::string *why, size_t *png_bytes) {
    std::vector<uint8_t> png;
    std::string err;
    if (!encode_png_rgb(rgb.data(), w, h, bottom_up, &png, &err)) {
        *why = "encode failed: " + err;
        return false;
    }
    *png_bytes = png.size();

    int gw = 0, gh = 0;
    std::vector<uint8_t> idat;
    if (!parse_png(png, &gw, &gh, &idat, why)) return false;
    if (gw != w || gh != h) {
        *why = "IHDR dimensions do not match";
        return false;
    }

    // zlib wrapper: 2-byte header whose 16-bit big-endian value is a multiple
    // of 31, and a 4-byte big-endian Adler-32 of the *uncompressed* data.
    if (idat.size() < 6) {
        *why = "IDAT too short to be a zlib stream";
        return false;
    }
    if ((uint32_t(idat[0]) * 256u + uint32_t(idat[1])) % 31u != 0) {
        *why = "zlib header fails its own check value";
        return false;
    }
    if ((idat[0] & 0x0F) != 8) {
        *why = "zlib compression method is not deflate";
        return false;
    }

    std::vector<uint8_t> raw;
    if (!inflate_raw(idat.data() + 2, idat.size() - 6, &raw)) {
        *why = "the deflate stream does not decode";
        return false;
    }
    const uint32_t want_adler = be32(&idat[idat.size() - 4]);
    if (adler32_of(raw.data(), raw.size()) != want_adler) {
        *why = "the Adler-32 does not match the decoded data";
        return false;
    }

    const std::vector<uint8_t> expect = expected_scanlines(rgb, w, h, bottom_up);
    if (raw != expect) {
        *why = "the decoded scanlines are not the image that went in";
        return false;
    }
    return true;
}

void test_png_round_trips() {
    std::printf("the PNG decodes back to exactly the pixels that went in\n");

    // A gradient, so consecutive bytes differ and the encoder is forced through
    // its literal path as well as its match path.
    const int w = 61, h = 37;   // deliberately not multiples of anything
    std::vector<uint8_t> rgb(size_t(w) * size_t(h) * 3);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const size_t o = (size_t(y) * size_t(w) + size_t(x)) * 3;
            rgb[o + 0] = uint8_t(x * 4);
            rgb[o + 1] = uint8_t(y * 7);
            rgb[o + 2] = uint8_t((x * y) & 0xFF);
        }

    std::string why = "-";
    size_t bytes = 0;
    check("a gradient survives the round trip", round_trip_png(rgb, w, h, false, &why, &bytes));
    if (g_failures) std::printf("      %s\n", why.c_str());

    check("and so does it upside down (the GL framebuffer case)",
          round_trip_png(rgb, w, h, true, &why, &bytes));

    // Every byte value at least once, which exercises both halves of the fixed
    // literal table -- the 8-bit codes below 144 and the 9-bit codes above.
    std::vector<uint8_t> all(768);
    for (size_t i = 0; i < all.size(); ++i) all[i] = uint8_t(i & 0xFF);
    check("so does an image containing every byte value",
          round_trip_png(all, 256, 1, false, &why, &bytes));

    // One pixel: the smallest legal image, and the case where the encoder never
    // enters its match loop at all.
    std::vector<uint8_t> one = {200, 100, 50};
    check("so does a single pixel", round_trip_png(one, 1, 1, false, &why, &bytes));
}

void test_png_actually_compresses() {
    std::printf("the match path works, or a screenshot would be megabytes\n");
    const int w = 400, h = 300;
    std::vector<uint8_t> rgb(size_t(w) * size_t(h) * 3, 0x1F);   // flat, like a UI
    std::string why = "-";
    size_t bytes = 0;
    check("a flat image round-trips", round_trip_png(rgb, w, h, false, &why, &bytes));
    const size_t raw = rgb.size() + size_t(h);
    std::printf("      %zu raw bytes -> %zu in the file (%.0fx)\n", raw, bytes,
                double(raw) / double(bytes));
    check("and it is at least 50x smaller than the raw scanlines", bytes * 50 < raw);
}

void test_png_rejects_nothing() {
    std::printf("a zero-sized image is refused rather than emitted\n");
    std::vector<uint8_t> out;
    std::string err;
    check("width 0 is refused", !encode_png_rgb((const uint8_t *)"", 0, 1, false, &out, &err));
}

// ------------------------------------------------------------------ state

void test_state_round_trips() {
    std::printf("instrument state survives a save and a load\n");
    InstrumentState s;
    s.view_start_hz = 2.4e9;
    s.view_stop_hz = 2.5e9;
    s.ref_db = -12.5;
    s.db_per_div = 5;
    s.wf_ref_db = -33.5;
    s.wf_range_db = 75;
    s.colormap = 2;
    s.auto_tune = false;
    s.show_band = false;
    s.src_start_hz = 70e6;
    s.src_stop_hz = 6e9;
    s.n_points = 16384;
    s.detector = "rms";
    s.window = "flat-top";
    s.keep_hz = 54e6;
    s.dc_policy = "blank";
    s.peak_excursion_db = 9.5;
    s.peak_threshold_db = -85.0;
    s.active_marker = 5;
    s.trace[0].mode = "max hold";
    s.trace[0].average_n = 32;
    s.trace[3].mode = "average";
    s.marker[5].on = true;
    s.marker[5].x = 2.45e9;
    s.marker[5].delta = true;
    s.marker[5].ref = 2;
    s.marker[5].trace = 3;
    s.marker[5].noise = true;

    const std::string j = state_to_json(s);
    InstrumentState r;
    std::string err;
    check("it parses back", state_from_json(j, &r, &err));

    check("the view survives", r.view_start_hz == s.view_start_hz &&
                                   r.view_stop_hz == s.view_stop_hz);
    check("the amplitude scale survives", r.ref_db == s.ref_db &&
                                              r.db_per_div == s.db_per_div);
    check("the waterfall scale and colormap survive",
          r.wf_ref_db == s.wf_ref_db && r.wf_range_db == s.wf_range_db &&
              r.colormap == s.colormap);
    check("auto-tune survives as false, not as a default true", r.auto_tune == false);
    check("and so does the min/max band, which also defaults true",
          r.show_band == false);
    check("the source request survives",
          r.src_start_hz == s.src_start_hz && r.n_points == s.n_points &&
              r.detector == "rms" && r.window == "flat-top");
    // The sweep rate is the segment count, so a state file that restored the
    // span but not the segmentation would come back at a different rate with
    // nothing on screen to say why.
    check("the segment width and DC policy survive",
          r.keep_hz == 54e6 && r.dc_policy == "blank");
    check("the peak search settings survive",
          r.peak_excursion_db == s.peak_excursion_db &&
              r.peak_threshold_db == s.peak_threshold_db);
    check("trace modes and N survive",
          r.trace[0].mode == "max hold" && r.trace[0].average_n == 32 &&
              r.trace[3].mode == "average");
    check("every marker field survives",
          r.marker[5].on && r.marker[5].x == 2.45e9 && r.marker[5].delta &&
              r.marker[5].ref == 2 && r.marker[5].trace == 3 && r.marker[5].noise);
    check("and so does which one is active", r.active_marker == 5);
    check("a marker that was off stays off", !r.marker[0].on);
}

void test_state_refuses_junk_and_clamps_indices() {
    std::printf("a state file is a file, so it is not trusted\n");
    InstrumentState r;
    std::string err;

    check("malformed JSON is refused", !state_from_json("{oh dear", &r, &err));
    check("so is valid JSON that is not a state file",
          !state_from_json("{\"kind\":\"something-else\"}", &r, &err));
    check("and it says so", contains(err, "not a sweepsa state file"));

    // Indices out of range would be an out-of-bounds array write on the very
    // next frame.
    const std::string bad =
        "{\"kind\":\"sweepsa-state\",\"active_marker\":99,\"marker.1.ref\":-4,"
        "\"marker.1.trace\":17,\"trace.1.average_n\":0}";
    check("an out-of-range file still loads", state_from_json(bad, &r, &err));
    check("with the active marker clamped", r.active_marker == 0);
    check("the delta reference clamped", r.marker[0].ref == 0);
    check("the trace index clamped", r.marker[0].trace == 0);
    check("and an average of zero sweeps raised to one", r.trace[0].average_n == 1);

    // A file from an older build, missing most keys.
    check("a sparse file loads on defaults",
          state_from_json("{\"kind\":\"sweepsa-state\",\"ref_db\":-7}", &r, &err));
    check("taking the value it does carry", r.ref_db == -7.0);
    check("and the default for the rest", r.db_per_div == 10.0 && r.trace[0].mode == "blank");
    // Segmentation specifically: absent must mean "ask for nothing", not "ask
    // for 0 Hz", or loading a pre-segmentation state file would refuse its way
    // through a retune it never intended.
    check("a state file predating segmentation asks for none",
          r.keep_hz == 0 && r.dc_policy.empty());
}

}  // namespace

int main() {
    test_csv_carries_what_makes_it_readable_later();
    test_csv_refuses_to_misalign_its_own_columns();
    test_png_round_trips();
    test_png_actually_compresses();
    test_png_rejects_nothing();
    test_state_round_trips();
    test_state_refuses_junk_and_clamps_indices();

    std::printf("\n%s (%d failure%s)\n", g_failures ? "FAILED" : "all passed",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures ? 1 : 0;
}
