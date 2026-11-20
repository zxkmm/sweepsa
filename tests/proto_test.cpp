// Tests for the sweepd <-> sweepui wire protocol. CLAUDE.md §5.
//
// The protocol is a contract between two binaries built in different phases and
// on different operating systems, so what is worth pinning here is not "does
// encode/decode round-trip" -- that would pass even if both sides agreed on the
// wrong thing -- but the byte layout itself, and the failure modes that would
// otherwise produce a plausible-looking wrong spectrum:
//
//   - the exact offsets and widths in the trace header;
//   - that the wire codes for Detector and Window match dsp.h, so reordering an
//     enumerator is a test failure rather than a silently relabelled trace;
//   - that NaN survives, because §7 needs unmeasured bins visibly missing;
//   - that a truncated or over-declared payload is rejected rather than
//     decoded into a spectrum with a wrong end;
//   - that the stream reader reassembles messages correctly across adversarial
//     chunk boundaries, which is the part that works in testing and fails under
//     load.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "proto.h"

using namespace sweepd;
using namespace sweepd::proto;

static int g_fail = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);      \
            ++g_fail;                                                          \
        }                                                                      \
    } while (0)

#define CHECK_EQ(a, b)                                                         \
    do {                                                                       \
        const auto va_ = (a);                                                  \
        const auto vb_ = (b);                                                  \
        if (!(va_ == vb_)) {                                                   \
            std::printf("  FAIL %s:%d  %s == %s  (%lld vs %lld)\n", __FILE__,  \
                        __LINE__, #a, #b, (long long)va_, (long long)vb_);     \
            ++g_fail;                                                          \
        }                                                                      \
    } while (0)

namespace {

uint32_t rd_u32(const std::vector<uint8_t> &b, size_t at) {
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i) v |= uint32_t(b[at + size_t(i)]) << (8 * i);
    return v;
}

uint64_t rd_u64(const std::vector<uint8_t> &b, size_t at) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v |= uint64_t(b[at + size_t(i)]) << (8 * i);
    return v;
}

double rd_f64(const std::vector<uint8_t> &b, size_t at) {
    const uint64_t u = rd_u64(b, at);
    double d;
    std::memcpy(&d, &u, 8);
    return d;
}

TraceMsg sample_frame(size_t n_points, uint32_t n_segments) {
    TraceMsg m;
    m.seq = 0x0102030405060708ull;
    m.t_mono_s = 1234.5;
    m.f_start_hz = 70000000LL;
    m.f_stop_hz = 6000000000LL;
    m.n_segments = n_segments;
    m.rbw_hz = 22050.5;
    m.bin_hz = 1000.25;
    m.sweep_seconds = 0.2315;
    m.gain_db = 30.5f;
    m.detector = Detector::RMS;
    m.window = Window::FlatTop;
    m.db.resize(n_points);
    for (size_t i = 0; i < n_points; ++i) m.db[i] = float(-100.0 + double(i));
    return m;
}

// ------------------------------------------------------------------ layout

void test_header_layout() {
    std::printf("header layout\n");

    std::vector<uint8_t> b;
    // Under kMaxPayload, with four distinct bytes so a byte-order mistake
    // cannot survive.
    encode_header(MsgType::Trace, 0x01223344u, &b);
    CHECK_EQ(b.size(), kHeaderBytes);
    // Little-endian length prefix, exactly as §5 specifies.
    CHECK_EQ(b[0], 0x44);
    CHECK_EQ(b[1], 0x33);
    CHECK_EQ(b[2], 0x22);
    CHECK_EQ(b[3], 0x01);
    CHECK_EQ(b[4], 0x02);
    CHECK_EQ(b[5], kVersion);
    CHECK_EQ(b[6], 0x00);
    CHECK_EQ(b[7], 0x00);

    Header h;
    std::string err;
    CHECK(decode_header(b.data(), b.size(), &h, &err));
    CHECK_EQ(h.payload_len, 0x01223344u);
    CHECK(h.type == MsgType::Trace);

    // A short read is not an error, it is "come back with more bytes"; but the
    // caller must not be able to mistake it for a decoded header.
    CHECK(!decode_header(b.data(), kHeaderBytes - 1, &h, &err));

    // A version the far end does not understand must be refused, not guessed
    // at: the payload layout is version-specific and misreading it produces a
    // spectrum rather than an error.
    b[5] = 99;
    CHECK(!decode_header(b.data(), b.size(), &h, &err));
    CHECK(err.find("version") != std::string::npos);

    // An absurd length prefix must not become an allocation.
    b[5] = kVersion;
    b[0] = 0xff; b[1] = 0xff; b[2] = 0xff; b[3] = 0xff;
    CHECK(!decode_header(b.data(), b.size(), &h, &err));
}

void test_trace_layout() {
    std::printf("trace layout\n");

    const TraceMsg m = sample_frame(7, 20);
    std::vector<uint8_t> b;
    encode_trace(m, &b);

    const uint32_t overload_bytes = (20 + 7) / 8;   // 3
    CHECK_EQ(overload_bytes, 3u);
    CHECK_EQ(b.size(), kHeaderBytes + kTraceHeadBytes + 7 * 4 + overload_bytes);
    CHECK_EQ(rd_u32(b, 0), uint32_t(kTraceHeadBytes + 7 * 4 + overload_bytes));

    // Field offsets, relative to the start of the payload. These are the
    // published layout; sweepui will hardcode them.
    const size_t P = kHeaderBytes;
    CHECK_EQ(rd_u64(b, P + 0), m.seq);
    CHECK(rd_f64(b, P + 8) == m.t_mono_s);
    CHECK_EQ((long long)rd_u64(b, P + 16), m.f_start_hz);
    CHECK_EQ((long long)rd_u64(b, P + 24), m.f_stop_hz);
    CHECK_EQ(rd_u32(b, P + 32), 7u);            // n_points
    CHECK_EQ(rd_u32(b, P + 36), 20u);           // n_segments
    CHECK(rd_f64(b, P + 40) == m.rbw_hz);
    CHECK(rd_f64(b, P + 48) == m.bin_hz);
    CHECK(rd_f64(b, P + 56) == m.sweep_seconds);
    CHECK_EQ(b[P + 68], detector_code(Detector::RMS));
    CHECK_EQ(b[P + 69], window_code(Window::FlatTop));
    CHECK_EQ(b[P + 70], 0);                     // x_kind: frequency
    CHECK_EQ(b[P + 71], 0);                     // reserved
    CHECK_EQ(rd_u32(b, P + 76), overload_bytes);
    CHECK_EQ(rd_u64(b, P + 80), 0u);            // dt_ns: unused in a sweep
    CHECK_EQ(rd_u64(b, P + 88), 0u);            // t0_ns: likewise
}

// A zero-span frame: same format, x_kind = time, and the two frequency fields
// collapsed onto the parked centre.
TraceMsg zspan_frame(size_t n_points, uint64_t dt_ns) {
    TraceMsg m = sample_frame(n_points, 1);
    m.f_start_hz = 1000000000LL;
    m.f_stop_hz = 1000000000LL;
    m.bin_hz = 0.0;
    m.x_kind = XKind::Time;
    m.dt_ns = dt_ns;
    return m;
}

void test_zspan_layout() {
    std::printf("zero-span trace layout\n");

    const TraceMsg m = zspan_frame(512, 16667);
    std::vector<uint8_t> b;
    encode_trace(m, &b);

    const size_t P = kHeaderBytes;
    CHECK_EQ(b[P + 70], 1);                      // x_kind: time
    CHECK_EQ(rd_u64(b, P + 80), 16667u);         // dt_ns
    // The two frequency fields carry the parked centre, not a span. A receiver
    // that ignored x_kind would compute a zero-wide axis rather than a
    // plausible-looking wrong one.
    CHECK_EQ((long long)rd_u64(b, P + 16), 1000000000LL);
    CHECK_EQ((long long)rd_u64(b, P + 24), 1000000000LL);
    CHECK(rd_f64(b, P + 48) == 0.0);             // bin_hz

    Header h;
    std::string err;
    CHECK(decode_header(b.data(), b.size(), &h, &err));
    TraceMsg r;
    CHECK(decode_trace(b.data() + kHeaderBytes, h.payload_len, &r, &err));
    CHECK(r.x_kind == XKind::Time);
    CHECK_EQ(r.dt_ns, 16667u);
    CHECK_EQ(r.n_segments, 1u);

    // The axis accessor is the whole point of carrying x_kind: one call site
    // serves both modes, so a marker cannot read a frequency off a time frame.
    CHECK(std::fabs(r.x_at(0) - 0.5 * 16667e-9) < 1e-15);
    CHECK(std::fabs(r.x_at(1) - 1.5 * 16667e-9) < 1e-15);

    const TraceMsg f = sample_frame(8, 4);
    CHECK(f.x_at(0) == double(f.f_start_hz) + 0.5 * f.bin_hz);
}

// §6's video trigger: a triggered frame's axis starts *before* its own origin,
// because the pre-trigger history was captured before the edge that defines
// t = 0. This is the one signed field on the frame, and the case that matters
// is that it survives the wire as a negative number rather than as a very large
// positive one -- which would draw a plausible trace shifted by ~584 years.
void test_trigger_origin() {
    std::printf("triggered zero span carries a negative origin\n");

    TraceMsg m = zspan_frame(512, 16667);
    m.t0_ns = -51 * 16667;    // 51 points of pre-trigger history
    std::vector<uint8_t> b;
    encode_trace(m, &b);

    TraceMsg r;
    std::string err;
    CHECK(decode_trace(b.data() + kHeaderBytes, b.size() - kHeaderBytes, &r, &err));
    CHECK_EQ(r.t0_ns, -51 * 16667);
    CHECK(r.x_at(0) < 0.0);
    CHECK(std::fabs(r.x_at(51) - 0.5 * 16667e-9) < 1e-12);   // the edge is at t=0

    // The spacing is unchanged: t0 moves the axis, it does not scale it.
    CHECK(std::fabs((r.x_at(1) - r.x_at(0)) - 16667e-9) < 1e-15);

    // And a swept frame carrying an origin is the producer confusing its modes,
    // caught here rather than drawn.
    TraceMsg s = sample_frame(16, 4);
    s.t0_ns = -5;
    b.clear();
    encode_trace(s, &b);
    CHECK(!decode_trace(b.data() + kHeaderBytes, b.size() - kHeaderBytes, &r, &err));
    CHECK(err.find("t0_ns") != std::string::npos);
}

void test_zspan_rejects_undefined_axis() {
    std::printf("zero-span rejects an undefined time axis\n");

    // dt_ns = 0 in time mode would put every point at t = 0: the trace draws as
    // one vertical line, which reads as a signal rather than as a missing field.
    TraceMsg m = zspan_frame(16, 0);
    std::vector<uint8_t> b;
    encode_trace(m, &b);
    TraceMsg r;
    std::string err;
    CHECK(!decode_trace(b.data() + kHeaderBytes, b.size() - kHeaderBytes, &r, &err));
    CHECK(err.find("dt_ns") != std::string::npos);

    // And the converse: a swept frame carrying an interval means the producer
    // has confused its two modes, which is worth catching at the wire rather
    // than downstream.
    TraceMsg s = sample_frame(16, 4);
    s.dt_ns = 1000;
    b.clear();
    encode_trace(s, &b);
    CHECK(!decode_trace(b.data() + kHeaderBytes, b.size() - kHeaderBytes, &r, &err));
    CHECK(err.find("zero span") != std::string::npos);

    // An x_kind the receiver does not know is refused rather than defaulted to
    // frequency, which would silently relabel a time axis in Hz.
    b.clear();
    encode_trace(sample_frame(16, 4), &b);
    b[kHeaderBytes + 70] = 7;
    CHECK(!decode_trace(b.data() + kHeaderBytes, b.size() - kHeaderBytes, &r, &err));
    CHECK(err.find("x_kind") != std::string::npos);
}

void test_enum_codes() {
    std::printf("enum codes vs dsp.h\n");

    // If someone reorders the enumerators in dsp.h, this fails here rather than
    // relabelling every trace at the far end -- which would look like a UI bug,
    // not a protocol one.
    const Detector dets[] = {Detector::Peak, Detector::NegativePeak, Detector::Sample,
                             Detector::RMS};
    for (Detector d : dets) {
        Detector back;
        CHECK(detector_from_code(detector_code(d), &back));
        CHECK(back == d);
    }
    CHECK_EQ(detector_code(Detector::Peak), 0);
    CHECK_EQ(detector_code(Detector::NegativePeak), 1);
    CHECK_EQ(detector_code(Detector::Sample), 2);
    CHECK_EQ(detector_code(Detector::RMS), 3);

    const Window wins[] = {Window::Rectangular, Window::Hann, Window::BlackmanHarris,
                           Window::FlatTop};
    for (Window w : wins) {
        Window back;
        CHECK(window_from_code(window_code(w), &back));
        CHECK(back == w);
    }
    CHECK_EQ(window_code(Window::Rectangular), 0);
    CHECK_EQ(window_code(Window::Hann), 1);
    CHECK_EQ(window_code(Window::BlackmanHarris), 2);
    CHECK_EQ(window_code(Window::FlatTop), 3);

    Detector d;
    Window w;
    CHECK(!detector_from_code(4, &d));
    CHECK(!window_from_code(4, &w));
}

// ------------------------------------------------------------------ trace

void test_trace_roundtrip() {
    std::printf("trace round trip\n");

    TraceMsg m = sample_frame(1024, 298);
    // §7: an unmeasured bin must arrive unmeasured, not as a plausible number.
    m.db[0] = std::numeric_limits<float>::quiet_NaN();
    m.db[511] = std::numeric_limits<float>::quiet_NaN();
    set_overload_bit(&m.overload, m.n_segments, 0);
    set_overload_bit(&m.overload, m.n_segments, 63);
    set_overload_bit(&m.overload, m.n_segments, 297);
    m.n_overloaded_segments = 3;

    std::vector<uint8_t> b;
    encode_trace(m, &b);

    Header h;
    std::string err;
    CHECK(decode_header(b.data(), b.size(), &h, &err));
    CHECK(h.type == MsgType::Trace);

    TraceMsg r;
    CHECK(decode_trace(b.data() + kHeaderBytes, h.payload_len, &r, &err));
    if (!err.empty()) std::printf("    err: %s\n", err.c_str());

    CHECK_EQ(r.seq, m.seq);
    CHECK(r.t_mono_s == m.t_mono_s);
    CHECK_EQ(r.f_start_hz, m.f_start_hz);
    CHECK_EQ(r.f_stop_hz, m.f_stop_hz);
    CHECK_EQ(r.n_segments, m.n_segments);
    CHECK(r.rbw_hz == m.rbw_hz);
    CHECK(r.bin_hz == m.bin_hz);
    CHECK(r.sweep_seconds == m.sweep_seconds);
    CHECK(r.gain_db == m.gain_db);
    CHECK(r.detector == m.detector);
    CHECK(r.window == m.window);
    CHECK_EQ(r.n_overloaded_segments, 3u);
    CHECK_EQ(r.n_points(), 1024u);

    CHECK(std::isnan(r.db[0]));
    CHECK(std::isnan(r.db[511]));
    for (size_t i = 1; i < 1024; ++i) {
        if (i == 511) continue;
        if (r.db[i] != m.db[i]) {
            std::printf("  FAIL db[%zu] %f != %f\n", i, double(r.db[i]), double(m.db[i]));
            ++g_fail;
            break;
        }
    }

    // The 298-segment case is exactly why the mask is a bitmap rather than the
    // u64 §5's wording suggests: segment 297 has to survive.
    CHECK(r.segment_overloaded(0));
    CHECK(r.segment_overloaded(63));
    CHECK(r.segment_overloaded(297));
    CHECK(!r.segment_overloaded(1));
    CHECK(!r.segment_overloaded(296));
}

void test_trace_rejects_bad_length() {
    std::printf("trace rejects inconsistent length\n");

    const TraceMsg m = sample_frame(64, 10);
    std::vector<uint8_t> b;
    encode_trace(m, &b);
    const uint8_t *pay = b.data() + kHeaderBytes;
    const size_t len = b.size() - kHeaderBytes;

    TraceMsg r;
    std::string err;

    // One byte short: a lenient decoder would hand back 63 good points and one
    // garbage one, which on screen is a spectrum with a wrong end.
    CHECK(!decode_trace(pay, len - 1, &r, &err));
    CHECK(err.find("bytes") != std::string::npos);

    // One byte long: extra data means the two ends disagree about the format.
    std::vector<uint8_t> longer(pay, pay + len);
    longer.push_back(0);
    CHECK(!decode_trace(longer.data(), longer.size(), &r, &err));

    // A payload shorter than the fixed header at all.
    CHECK(!decode_trace(pay, 10, &r, &err));

    // n_points inflated so that head + 4*n would wrap 32 bits back into range.
    // Checked in 64-bit precisely so this cannot look self-consistent.
    std::vector<uint8_t> evil(len);
    std::memcpy(evil.data(), pay, len);
    const uint32_t huge = 0x40000000u;   // *4 == 2^32
    for (size_t i = 0; i < 4; ++i) evil.data()[32 + i] = uint8_t(huge >> (8 * i));
    CHECK(!decode_trace(evil.data(), evil.size(), &r, &err));

    // An overload bitmap whose size disagrees with n_segments.
    std::vector<uint8_t> bad(pay, pay + len);
    bad[76] = 99;
    CHECK(!decode_trace(bad.data(), bad.size(), &r, &err));
}

void test_trace_empty_and_no_segments() {
    std::printf("trace degenerate cases\n");

    TraceMsg m;
    m.n_segments = 0;
    std::vector<uint8_t> b;
    encode_trace(m, &b);

    Header h;
    std::string err;
    CHECK(decode_header(b.data(), b.size(), &h, &err));
    TraceMsg r;
    CHECK(decode_trace(b.data() + kHeaderBytes, h.payload_len, &r, &err));
    CHECK_EQ(r.n_points(), 0u);
    CHECK_EQ(r.n_segments, 0u);

    // A caller that declares segments but never sizes the mask still produces a
    // well-formed all-clear frame rather than a length mismatch on the wire.
    TraceMsg m2 = sample_frame(4, 12);
    m2.overload.clear();
    std::vector<uint8_t> b2;
    encode_trace(m2, &b2);
    CHECK(decode_header(b2.data(), b2.size(), &h, &err));
    CHECK(decode_trace(b2.data() + kHeaderBytes, h.payload_len, &r, &err));
    CHECK_EQ(r.overload.size(), 2u);
    CHECK(!r.segment_overloaded(0));
}

// ------------------------------------------------------------------ json

void test_json_writer() {
    std::printf("json writer\n");

    const std::string s = JsonWriter()
                              .kv("type", "status")
                              .kv("sweeps", uint64_t(42))
                              .kv("rate", 4.3125)
                              .kv("uncal", true)
                              .key("nothing").null()
                              .done();
    JsonObject o;
    std::string err;
    CHECK(parse_flat_json(s, &o, &err));
    if (!err.empty()) std::printf("    err: %s (%s)\n", err.c_str(), s.c_str());
    CHECK(json_str(o, "type") == "status");
    CHECK(json_num(o, "sweeps") == 42.0);
    CHECK(json_num(o, "rate") == 4.3125);
    CHECK(json_bool(o, "uncal") == true);
    CHECK(o.count("nothing") == 1);
    CHECK(o.at("nothing").kind == JsonValue::Kind::Null);

    // JSON has no NaN or Infinity. Emitting the bare token would produce a
    // document that any real parser at the far end rejects -- and the far end
    // is a GUI in another language, so it will be a real parser.
    const std::string n = JsonWriter()
                              .kv("bad", std::nan(""))
                              .kv("big", std::numeric_limits<double>::infinity())
                              .done();
    CHECK(n.find("nan") == std::string::npos);
    CHECK(n.find("inf") == std::string::npos);
    CHECK(parse_flat_json(n, &o, &err));
    CHECK(o.at("bad").kind == JsonValue::Kind::Null);

    // Quotes and control characters in a value must not break out of the
    // string; error messages go through here and they contain both.
    const std::string q =
        JsonWriter().kv("msg", std::string("he said \"no\"\nand \\left")).done();
    CHECK(parse_flat_json(q, &o, &err));
    CHECK(json_str(o, "msg") == "he said \"no\"\nand \\left");

    CHECK(parse_flat_json(JsonWriter().done(), &o, &err));
    CHECK(o.empty());
}

void test_json_parser_rejects() {
    std::printf("json parser rejects what it cannot handle\n");

    JsonObject o;
    std::string err;

    // Nesting is refused, not half-understood. proto.h explains why.
    CHECK(!parse_flat_json("{\"a\":{\"b\":1}}", &o, &err));
    CHECK(err.find("nested") != std::string::npos);
    CHECK(!parse_flat_json("{\"a\":[1,2]}", &o, &err));

    CHECK(!parse_flat_json("", &o, &err));
    CHECK(!parse_flat_json("[]", &o, &err));
    CHECK(!parse_flat_json("{\"a\":1", &o, &err));
    CHECK(!parse_flat_json("{\"a\" 1}", &o, &err));
    CHECK(!parse_flat_json("{\"a\":1}garbage", &o, &err));
    CHECK(!parse_flat_json("{a:1}", &o, &err));
    CHECK(!parse_flat_json("{\"a\":1,\"a\":2}", &o, &err));
    CHECK(!parse_flat_json("{\"a\":tru}", &o, &err));
    CHECK(!parse_flat_json("{\"a\":\"unterminated}", &o, &err));

    // Whitespace between every token is legal and must survive.
    CHECK(parse_flat_json("  {  \"a\" : 1 , \"b\" : \"x\" }  ", &o, &err));
    CHECK(json_num(o, "a") == 1.0);
    CHECK(json_str(o, "b") == "x");

    // Wrong-kind reads fall back to the default rather than throwing, so a
    // handler is not a wall of type checks.
    CHECK(json_num(o, "b", -1.0) == -1.0);
    CHECK(json_str(o, "a", "dflt") == "dflt");
    CHECK(json_num(o, "absent", 7.0) == 7.0);
}

// ------------------------------------------------------------------ reader

void test_reader_reassembly() {
    std::printf("stream reader reassembly\n");

    // Three messages of different types and sizes, concatenated -- which is
    // what a busy socket delivers.
    std::vector<uint8_t> stream;
    encode_control(JsonWriter().kv("cmd", "hello").done(), &stream);
    encode_trace(sample_frame(300, 12), &stream);
    encode_control(JsonWriter().kv("type", "status").done(), &stream);
    const size_t total = stream.size();

    // Every chunk size from 1 byte up, including sizes that split the length
    // prefix itself. One byte at a time is the case that catches a reader which
    // peeks at the header before it has all of it.
    for (size_t chunk : {size_t(1), size_t(2), size_t(3), size_t(7), size_t(8),
                         size_t(9), size_t(64), size_t(1000), total}) {
        Reader r;
        int n_control = 0, n_trace = 0;
        for (size_t at = 0; at < total; at += chunk) {
            const size_t n = std::min(chunk, total - at);
            r.feed(stream.data() + at, n);
            Header h;
            const uint8_t *p = nullptr;
            while (r.next(&h, &p)) {
                if (h.type == MsgType::Control) {
                    ++n_control;
                    JsonObject o;
                    std::string err;
                    CHECK(parse_flat_json(
                        std::string(reinterpret_cast<const char *>(p), h.payload_len), &o,
                        &err));
                } else if (h.type == MsgType::Trace) {
                    ++n_trace;
                    TraceMsg t;
                    std::string err;
                    CHECK(decode_trace(p, h.payload_len, &t, &err));
                    CHECK_EQ(t.n_points(), 300u);
                    CHECK_EQ(t.n_segments, 12u);
                }
            }
            CHECK(!r.failed());
        }
        if (n_control != 2 || n_trace != 1) {
            std::printf("  FAIL chunk=%zu control=%d trace=%d\n", chunk, n_control, n_trace);
            ++g_fail;
        }
        CHECK_EQ(r.buffered(), 0u);
    }
}

void test_reader_fails_hard() {
    std::printf("stream reader fails hard on a corrupt stream\n");

    // There are no frame markers to resynchronise to, by design: a corrupt
    // stream must look unrecoverable rather than recoverable.
    std::vector<uint8_t> stream;
    encode_control("{}", &stream);
    stream[5] = 42;   // bad version

    Reader r;
    r.feed(stream.data(), stream.size());
    Header h;
    const uint8_t *p = nullptr;
    CHECK(!r.next(&h, &p));
    CHECK(r.failed());
    CHECK(r.error().find("version") != std::string::npos);

    // Once failed it stays failed, even if valid bytes follow.
    std::vector<uint8_t> good;
    encode_control("{}", &good);
    r.feed(good.data(), good.size());
    CHECK(!r.next(&h, &p));
    CHECK(r.failed());

    r.reset();
    CHECK(!r.failed());
    r.feed(good.data(), good.size());
    CHECK(r.next(&h, &p));
}

void test_reader_compaction() {
    std::printf("stream reader stays bounded\n");

    // A long-lived connection must not grow its buffer without bound just
    // because it never goes idle. sweepd's control channel is low rate, but the
    // same Reader decodes the trace stream inside sweepui, which is not.
    std::vector<uint8_t> one;
    encode_control(JsonWriter().kv("cmd", "ping").done(), &one);

    Reader r;
    for (int i = 0; i < 20000; ++i) {
        r.feed(one.data(), one.size());
        Header h;
        const uint8_t *p = nullptr;
        CHECK(r.next(&h, &p));
    }
    CHECK(!r.failed());
    CHECK_EQ(r.buffered(), 0u);
}

}  // namespace

int main() {
    std::printf("proto tests\n\n");
    test_header_layout();
    test_trace_layout();
    test_zspan_layout();
    test_zspan_rejects_undefined_axis();
    test_trigger_origin();
    test_enum_codes();
    test_trace_roundtrip();
    test_trace_rejects_bad_length();
    test_trace_empty_and_no_segments();
    test_json_writer();
    test_json_parser_rejects();
    test_reader_reassembly();
    test_reader_fails_hard();
    test_reader_compaction();

    std::printf("\n%s\n", g_fail == 0 ? "all passed" : "FAILURES");
    return g_fail == 0 ? 0 : 1;
}
