#include "export.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "proto.h"

namespace sweepui {
namespace {

// A free function rather than a lambda: a lambda cannot be C-variadic, and the
// format attribute is worth having on something that formats every row of a
// file the user will read into another program.
#if defined(__GNUC__)
__attribute__((format(printf, 2, 3)))
#endif
void appendf(std::string *out, const char *fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    out->append(buf);
}

bool write_file(const std::string &path, const void *p, size_t n, std::string *err) {
    FILE *f = std::fopen(path.c_str(), "wb");
    if (!f) {
        *err = "cannot open " + path + " for writing";
        return false;
    }
    const bool ok = n == 0 || std::fwrite(p, 1, n, f) == n;
    // The close matters: a short write on a full disk surfaces here, and an
    // export that silently truncated is worse than one that failed.
    const bool closed = std::fclose(f) == 0;
    if (!ok || !closed) {
        *err = "could not write all of " + path;
        return false;
    }
    return true;
}

}  // namespace

// ---------------------------------------------------------------- trace CSV

bool format_trace_csv(const CsvMeta &meta, const std::vector<CsvColumn> &columns,
                      std::string *out, std::string *err) {
    if (columns.empty()) {
        *err = "nothing to export -- no trace is visible";
        return false;
    }
    const size_t n = columns[0].n;
    if (n == 0) {
        *err = "nothing to export -- the trace is empty";
        return false;
    }
    for (const CsvColumn &c : columns) {
        if (c.n != n || !c.db) {
            *err = "the traces are not the same length; refusing to write a file "
                   "whose columns would be misaligned";
            return false;
        }
    }

    out->clear();
    out->reserve(n * 16 * (columns.size() + 1) + 1024);
    char buf[512];

    // The same `# key=value` comment style sweepd's CSV uses, so one reader
    // handles both.
    appendf(out, "# sweepsa trace export\n");
    appendf(out, "# source=%s\n", meta.source.c_str());
    // The x column's identity, first, because everything after it is read in
    // terms of it. A zero-span export is amplitude over time and a reader that
    // took it for a spectrum would map every point to a frequency that was
    // never measured -- the same failure the wire format's x_kind prevents.
    if (meta.time_axis) {
        appendf(out, "# x_kind=time\n");
        appendf(out, "# center_hz=%lld n_points=%zu dt_s=%.12f\n", meta.f_start_hz, n,
                meta.dt_s);
        appendf(out, "# time_resolution_s=%.12f -- each point integrates over this; "
                     "the overlap does not improve it\n", meta.time_res_s);
        if (!meta.trigger.empty() && meta.trigger != "free_run")
            appendf(out, "# trigger=%s t0_s=%.12f -- t=0 is the trigger edge; earlier "
                         "points are negative\n", meta.trigger.c_str(), meta.t0_s);
        else
            appendf(out, "# trigger=free_run t0_s=0 -- frames start at unrelated "
                         "moments, so t is measured from the start of this one\n");
    } else {
        appendf(out, "# x_kind=frequency\n");
        appendf(out, "# f_start_hz=%lld f_stop_hz=%lld n_points=%zu bin_hz=%.6f\n",
                meta.f_start_hz, meta.f_stop_hz, n, meta.bin_hz);
    }
    appendf(out, "# rbw_hz=%.3f window=%s detector=%s gain_db=%.1f\n", meta.rbw_hz,
            sweepd::window_name(meta.window), sweepd::detector_name(meta.detector),
            double(meta.gain_db));
    appendf(out, "# segments=%u overloaded=%u seq=%llu sweeps_per_s=%.2f\n",
            unsigned(meta.n_segments), unsigned(meta.n_overloaded_segments),
            (unsigned long long)meta.seq, meta.sweeps_per_s);
    // §7, in the file rather than only on the screen. A CSV that leaves the
    // reader to remember that the units were dBFS and the source was invented
    // is a CSV that will be read as dBm from a radio.
    appendf(out, "# units=%s uncal=%s synthetic=%s\n",
            !meta.units.empty() ? meta.units.c_str() : meta.uncal ? "dBFS" : "dBm",
            meta.uncal ? "true" : "false", meta.synthetic ? "true" : "false");
    if (!meta.units.empty())
        appendf(out, "# %s: normalized against a stored thru; each value is |S21| in dB.\n",
                meta.units.c_str());
    if (!meta.units_note.empty()) appendf(out, "# %s\n", meta.units_note.c_str());
    if (meta.uncal)
        appendf(out, "# %s: these are dBFS, not dBm.\n",
                meta.uncal_reason.empty() ? "no amplitude correction is loaded"
                                          : meta.uncal_reason.c_str());
    if (meta.synthetic)
        appendf(out, "# SYNTHETIC: no radio produced these numbers.\n");
    appendf(out, "# nan: %s -- unmeasured, not quiet.\n",
            meta.time_axis ? "this block was not captured"
                           : "no segment covered that bin");
    for (const CsvColumn &c : columns)
        appendf(out, "# %s: %s\n", c.name.c_str(), c.mode.c_str());

    out->append(meta.time_axis ? "time_s" : "freq_hz");
    for (const CsvColumn &c : columns) {
        out->append(",");
        out->append(c.name);
    }
    out->append("\n");

    for (size_t i = 0; i < n; ++i) {
        if (meta.time_axis) {
            // From t0, and with enough digits for a microsecond dt. In free
            // run t0 is 0 and t is measured from the start of this frame -- the
            // frames are not contiguous with each other, so an absolute time
            // would mean nothing. Under a trigger t0 is negative and 0 is the
            // edge.
            std::snprintf(buf, sizeof buf, "%.12f",
                          meta.t0_s + (double(i) + 0.5) * meta.dt_s);
        } else {
            const double hz = double(meta.f_start_hz) + (double(i) + 0.5) * meta.bin_hz;
            std::snprintf(buf, sizeof buf, "%.0f", hz);
        }
        out->append(buf);
        for (const CsvColumn &c : columns) {
            const float v = c.db[i];
            // "nan" spelled out: %f's sign for NaN is platform-dependent and a
            // leading "-nan" trips up readers. sweepd's CSV does the same.
            if (std::isnan(v))
                std::snprintf(buf, sizeof buf, ",nan");
            else
                std::snprintf(buf, sizeof buf, ",%.2f", double(v));
            out->append(buf);
        }
        out->append("\n");
    }
    return true;
}

bool write_trace_csv(const std::string &path, const CsvMeta &meta,
                     const std::vector<CsvColumn> &columns, std::string *err) {
    std::string s;
    if (!format_trace_csv(meta, columns, &s, err)) return false;
    return write_file(path, s.data(), s.size(), err);
}

// ---------------------------------------------------------------- checksums

uint32_t crc32_of(const uint8_t *p, size_t n) {
    static uint32_t table[256];
    static bool built = false;
    if (!built) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            table[i] = c;
        }
        built = true;
    }
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; ++i) c = table[(c ^ p[i]) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

uint32_t adler32_of(const uint8_t *p, size_t n) {
    uint32_t a = 1, b = 0;
    for (size_t i = 0; i < n; ++i) {
        a = (a + p[i]) % 65521u;
        b = (b + a) % 65521u;
    }
    return (b << 16) | a;
}

// ---------------------------------------------------------------- PNG

namespace {

// RFC 1951's fixed-Huffman block. Literals and match lengths share one code
// table with a fixed assignment; distances are a flat 5 bits.
//
// The bit-order trap, which is the one thing that makes hand-written deflate go
// wrong: Huffman *codes* are written most-significant bit first, while every
// other field in the format -- the block header, the extra bits -- is written
// least-significant bit first. Two orders in one stream, and getting either
// backwards produces a file that decodes into garbage rather than failing.
class BitWriter {
public:
    explicit BitWriter(std::vector<uint8_t> *out) : out_(out) {}

    void bits(uint32_t v, int n) {
        buf_ |= (v & ((1u << n) - 1u)) << cnt_;
        cnt_ += n;
        while (cnt_ >= 8) {
            out_->push_back(uint8_t(buf_ & 0xFF));
            buf_ >>= 8;
            cnt_ -= 8;
        }
    }

    void code(uint32_t c, int n) {
        for (int i = n - 1; i >= 0; --i) bits((c >> i) & 1u, 1);
    }

    void flush() {
        if (cnt_ > 0) {
            out_->push_back(uint8_t(buf_ & 0xFF));
            buf_ = 0;
            cnt_ = 0;
        }
    }

private:
    std::vector<uint8_t> *out_;
    uint32_t buf_ = 0;
    int cnt_ = 0;
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

void emit_literal(BitWriter &w, unsigned v) {
    if (v <= 143)        w.code(0x030 + v, 8);            // 00110000..10111111
    else if (v <= 255)   w.code(0x190 + (v - 144), 9);    // 110010000..111111111
    else if (v <= 279)   w.code(v - 256, 7);              // 0000000..0010111
    else                 w.code(0x0C0 + (v - 280), 8);    // 11000000..11000111
}

void emit_match(BitWriter &w, size_t len, size_t dist) {
    size_t li = 28;
    while (li > 0 && kLenBase[li] > len) --li;
    emit_literal(w, unsigned(257 + li));
    if (kLenExtra[li]) w.bits(uint32_t(len - kLenBase[li]), kLenExtra[li]);

    size_t di = 29;
    while (di > 0 && kDistBase[di] > dist) --di;
    w.code(uint32_t(di), 5);
    if (kDistExtra[di]) w.bits(uint32_t(dist - kDistBase[di]), kDistExtra[di]);
}

constexpr size_t kHashBits = 15;
constexpr size_t kHashSize = size_t(1) << kHashBits;
constexpr size_t kMaxDist = 32768;

inline uint32_t hash3(const uint8_t *p) {
    return ((uint32_t(p[0]) << 10) ^ (uint32_t(p[1]) << 5) ^ uint32_t(p[2])) &
           uint32_t(kHashSize - 1);
}

// A single fixed-Huffman block with greedy LZ77 over a one-entry-per-slot hash
// table. Deliberately not a full match-chain search: a screenshot is mostly
// flat runs, and a 3-byte hash finds the position one byte back for a run of
// identical pixels, so the run collapses into one long distance-1 match. That
// is where nearly all of the compression on this kind of image is, and a chain
// search would add code to chase the rest.
void deflate_fixed(const uint8_t *data, size_t n, std::vector<uint8_t> *out) {
    BitWriter w(out);
    w.bits(1, 1);   // BFINAL
    w.bits(1, 2);   // BTYPE = 01, fixed Huffman

    std::vector<int32_t> head(kHashSize, -1);
    size_t i = 0;
    while (i + 3 <= n) {
        const uint32_t h = hash3(data + i);
        const int32_t cand = head[h];
        head[h] = int32_t(i);

        size_t best_len = 0, best_dist = 0;
        if (cand >= 0) {
            const size_t dist = i - size_t(cand);
            if (dist >= 1 && dist <= kMaxDist) {
                const size_t maxlen = std::min<size_t>(258, n - i);
                size_t l = 0;
                // Overlapping matches are legal and are what turns a run of
                // identical bytes into one match: the source of the copy is
                // still being written as it is read.
                while (l < maxlen && data[size_t(cand) + l] == data[i + l]) ++l;
                if (l >= 3) {
                    best_len = l;
                    best_dist = dist;
                }
            }
        }

        if (best_len >= 3) {
            emit_match(w, best_len, best_dist);
            for (size_t k = 1; k < best_len && i + k + 3 <= n; ++k)
                head[hash3(data + i + k)] = int32_t(i + k);
            i += best_len;
        } else {
            emit_literal(w, data[i]);
            ++i;
        }
    }
    while (i < n) emit_literal(w, data[i++]);

    emit_literal(w, 256);   // end of block
    w.flush();
}

void put_be32(std::vector<uint8_t> *out, uint32_t v) {
    out->push_back(uint8_t(v >> 24));
    out->push_back(uint8_t(v >> 16));
    out->push_back(uint8_t(v >> 8));
    out->push_back(uint8_t(v));
}

void put_chunk(std::vector<uint8_t> *out, const char type[4], const uint8_t *data,
               size_t n) {
    put_be32(out, uint32_t(n));
    std::vector<uint8_t> body;
    body.reserve(4 + n);
    body.insert(body.end(), type, type + 4);
    body.insert(body.end(), data, data + n);
    out->insert(out->end(), body.begin(), body.end());
    put_be32(out, crc32_of(body.data(), body.size()));
}

}  // namespace

bool encode_png_rgb(const uint8_t *rgb, int w, int h, bool bottom_up,
                    std::vector<uint8_t> *out, std::string *err) {
    if (!rgb || w <= 0 || h <= 0) {
        *err = "nothing to encode";
        return false;
    }

    // The raw PNG scanlines: one filter byte per row, then the row. Filter 0
    // (None) throughout -- filtering would help a photograph and does very
    // little for flat UI colour, and every filter type is another place to be
    // subtly wrong in a format nothing here can validate.
    const size_t stride = size_t(w) * 3;
    std::vector<uint8_t> raw;
    raw.reserve((stride + 1) * size_t(h));
    for (int y = 0; y < h; ++y) {
        const int sy = bottom_up ? (h - 1 - y) : y;
        raw.push_back(0);
        raw.insert(raw.end(), rgb + size_t(sy) * stride, rgb + size_t(sy) * stride + stride);
    }

    std::vector<uint8_t> z;
    z.push_back(0x78);   // CMF: deflate, 32 kB window
    z.push_back(0x01);   // FLG: no preset dict; (0x78<<8|0x01) % 31 == 0
    deflate_fixed(raw.data(), raw.size(), &z);
    put_be32(&z, adler32_of(raw.data(), raw.size()));

    out->clear();
    const uint8_t sig[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    out->insert(out->end(), sig, sig + 8);

    uint8_t ihdr[13];
    ihdr[0] = uint8_t(uint32_t(w) >> 24);
    ihdr[1] = uint8_t(uint32_t(w) >> 16);
    ihdr[2] = uint8_t(uint32_t(w) >> 8);
    ihdr[3] = uint8_t(uint32_t(w));
    ihdr[4] = uint8_t(uint32_t(h) >> 24);
    ihdr[5] = uint8_t(uint32_t(h) >> 16);
    ihdr[6] = uint8_t(uint32_t(h) >> 8);
    ihdr[7] = uint8_t(uint32_t(h));
    ihdr[8] = 8;    // bit depth
    ihdr[9] = 2;    // colour type: truecolour RGB
    ihdr[10] = 0;   // compression: deflate
    ihdr[11] = 0;   // filter method
    ihdr[12] = 0;   // no interlace
    put_chunk(out, "IHDR", ihdr, sizeof ihdr);
    put_chunk(out, "IDAT", z.data(), z.size());
    put_chunk(out, "IEND", nullptr, 0);
    return true;
}

bool write_png_rgb(const std::string &path, const uint8_t *rgb, int w, int h,
                   bool bottom_up, std::string *err) {
    std::vector<uint8_t> png;
    if (!encode_png_rgb(rgb, w, h, bottom_up, &png, err)) return false;
    return write_file(path, png.data(), png.size(), err);
}

// ---------------------------------------------------------------- state

namespace {

std::string trace_key(int i, const char *field) {
    return "trace." + std::to_string(i + 1) + "." + field;
}
std::string marker_key(int i, const char *field) {
    return "marker." + std::to_string(i + 1) + "." + field;
}

}  // namespace

std::string state_to_json(const InstrumentState &s) {
    sweepd::proto::JsonWriter w;
    w.kv("kind", "sweepsa-state");
    w.kv("version", (long long)1);

    w.kv("view_start_hz", s.view_start_hz);
    w.kv("view_stop_hz", s.view_stop_hz);
    w.kv("ref_db", s.ref_db);
    w.kv("db_per_div", s.db_per_div);
    w.kv("lin_scale", s.lin_scale);
    w.kv("wf_ref_db", s.wf_ref_db);
    w.kv("wf_range_db", s.wf_range_db);
    w.kv("colormap", (long long)s.colormap);
    w.kv("auto_tune", s.auto_tune);
    w.kv("show_band", s.show_band).kv("fill_band", s.fill_band)
        .kv("show_segments", s.show_segments);

    w.kv("src_start_hz", s.src_start_hz);
    w.kv("src_stop_hz", s.src_stop_hz);
    w.kv("n_points", (long long)s.n_points);
    w.kv("detector", s.detector);
    w.kv("window", s.window);
    w.kv("keep_hz", s.keep_hz);
    w.kv("dc_policy", s.dc_policy);

    w.kv("peak_excursion_db", s.peak_excursion_db);
    w.kv("peak_threshold_db", s.peak_threshold_db);
    w.kv("active_marker", (long long)s.active_marker);

    for (int i = 0; i < 4; ++i) {
        w.kv(trace_key(i, "mode"), s.trace[i].mode);
        w.kv(trace_key(i, "average_n"), (long long)s.trace[i].average_n);
    }
    for (int i = 0; i < 8; ++i) {
        w.kv(marker_key(i, "on"), s.marker[i].on);
        w.kv(marker_key(i, "x"), s.marker[i].x);
        w.kv(marker_key(i, "delta"), s.marker[i].delta);
        w.kv(marker_key(i, "ref"), (long long)s.marker[i].ref);
        w.kv(marker_key(i, "trace"), (long long)s.marker[i].trace);
        w.kv(marker_key(i, "noise"), s.marker[i].noise);
    }
    return w.done();
}

bool state_from_json(const std::string &json, InstrumentState *out, std::string *err) {
    sweepd::proto::JsonObject o;
    if (!sweepd::proto::parse_flat_json(json, &o, err)) return false;
    if (sweepd::proto::json_str(o, "kind") != "sweepsa-state") {
        *err = "not a sweepsa state file";
        return false;
    }

    InstrumentState s;   // start from the defaults, so a missing key keeps one
    using sweepd::proto::json_bool;
    using sweepd::proto::json_num;
    using sweepd::proto::json_str;

    s.view_start_hz = json_num(o, "view_start_hz", s.view_start_hz);
    s.view_stop_hz = json_num(o, "view_stop_hz", s.view_stop_hz);
    s.ref_db = json_num(o, "ref_db", s.ref_db);
    s.db_per_div = json_num(o, "db_per_div", s.db_per_div);
    s.wf_ref_db = json_num(o, "wf_ref_db", s.wf_ref_db);
    s.wf_range_db = json_num(o, "wf_range_db", s.wf_range_db);
    s.colormap = int(json_num(o, "colormap", s.colormap));
    s.auto_tune = json_bool(o, "auto_tune", s.auto_tune);
    s.show_band = json_bool(o, "show_band", s.show_band);
    s.lin_scale = json_bool(o, "lin_scale", s.lin_scale);
    s.fill_band = json_bool(o, "fill_band", s.fill_band);
    s.show_segments = json_bool(o, "show_segments", s.show_segments);

    s.src_start_hz = json_num(o, "src_start_hz", s.src_start_hz);
    s.src_stop_hz = json_num(o, "src_stop_hz", s.src_stop_hz);
    s.n_points = int(json_num(o, "n_points", s.n_points));
    s.detector = json_str(o, "detector", s.detector);
    s.window = json_str(o, "window", s.window);
    s.keep_hz = json_num(o, "keep_hz", s.keep_hz);
    s.dc_policy = json_str(o, "dc_policy", s.dc_policy);

    s.peak_excursion_db = json_num(o, "peak_excursion_db", s.peak_excursion_db);
    s.peak_threshold_db = json_num(o, "peak_threshold_db", s.peak_threshold_db);
    s.active_marker = int(json_num(o, "active_marker", s.active_marker));

    for (int i = 0; i < 4; ++i) {
        s.trace[i].mode = json_str(o, trace_key(i, "mode"), s.trace[i].mode);
        s.trace[i].average_n =
            int(json_num(o, trace_key(i, "average_n"), s.trace[i].average_n));
    }
    for (int i = 0; i < 8; ++i) {
        s.marker[i].on = json_bool(o, marker_key(i, "on"), s.marker[i].on);
        // `freq_hz` was the key before zero span made the axis able to be time.
        // Read as the fallback so a state file saved by an older build still
        // loads its markers rather than silently placing them all at 0.
        s.marker[i].x = json_num(o, marker_key(i, "freq_hz"), s.marker[i].x);
        s.marker[i].x = json_num(o, marker_key(i, "x"), s.marker[i].x);
        s.marker[i].delta = json_bool(o, marker_key(i, "delta"), s.marker[i].delta);
        s.marker[i].ref = int(json_num(o, marker_key(i, "ref"), s.marker[i].ref));
        s.marker[i].trace = int(json_num(o, marker_key(i, "trace"), s.marker[i].trace));
        s.marker[i].noise = json_bool(o, marker_key(i, "noise"), s.marker[i].noise);
    }

    // Clamped on the way in rather than trusted. A state file is a file, and an
    // out-of-range index would be an out-of-bounds write into a fixed array on
    // the very next frame.
    if (s.active_marker < 0 || s.active_marker > 7) s.active_marker = 0;
    for (int i = 0; i < 8; ++i) {
        if (s.marker[i].ref < 0 || s.marker[i].ref > 7) s.marker[i].ref = 0;
        if (s.marker[i].trace < 0 || s.marker[i].trace > 3) s.marker[i].trace = 0;
    }
    for (int i = 0; i < 4; ++i)
        if (s.trace[i].average_n < 1) s.trace[i].average_n = 1;

    *out = s;
    return true;
}

bool write_state_json(const std::string &path, const InstrumentState &s,
                      std::string *err) {
    const std::string j = state_to_json(s);
    return write_file(path, j.data(), j.size(), err);
}

bool read_state_json(const std::string &path, InstrumentState *out, std::string *err) {
    FILE *f = std::fopen(path.c_str(), "rb");
    if (!f) {
        *err = "cannot open " + path;
        return false;
    }
    std::string s;
    char buf[4096];
    size_t got;
    while ((got = std::fread(buf, 1, sizeof buf, f)) > 0) s.append(buf, got);
    std::fclose(f);
    return state_from_json(s, out, err);
}

}  // namespace sweepui
