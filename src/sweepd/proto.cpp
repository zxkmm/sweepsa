#include "proto.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace sweepd {
namespace proto {
namespace {

// The wire is little-endian regardless of the host (see proto.h). On the
// machines this actually runs on the two agree, and the bulk float32 array is
// memcpy'd on that basis; the scalar accessors stay explicit so the wire order
// remains a property of the format rather than of the compiler.
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
constexpr bool kHostIsLittleEndian = true;
#else
constexpr bool kHostIsLittleEndian = false;
#endif

void put_u8(std::vector<uint8_t> *o, uint8_t v) { o->push_back(v); }

void put_u16(std::vector<uint8_t> *o, uint16_t v) {
    o->push_back(uint8_t(v));
    o->push_back(uint8_t(v >> 8));
}

void put_u32(std::vector<uint8_t> *o, uint32_t v) {
    for (int i = 0; i < 4; ++i) o->push_back(uint8_t(v >> (8 * i)));
}

void put_u64(std::vector<uint8_t> *o, uint64_t v) {
    for (int i = 0; i < 8; ++i) o->push_back(uint8_t(v >> (8 * i)));
}

void put_i64(std::vector<uint8_t> *o, long long v) { put_u64(o, uint64_t(v)); }

// Bit patterns, not values: NaN must survive the wire intact (proto.h, dBFS).
void put_f32(std::vector<uint8_t> *o, float v) {
    uint32_t b;
    std::memcpy(&b, &v, 4);
    put_u32(o, b);
}

void put_f64(std::vector<uint8_t> *o, double v) {
    uint64_t b;
    std::memcpy(&b, &v, 8);
    put_u64(o, b);
}

uint16_t get_u16(const uint8_t *p) { return uint16_t(p[0]) | uint16_t(uint16_t(p[1]) << 8); }

uint32_t get_u32(const uint8_t *p) {
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i) v |= uint32_t(p[i]) << (8 * i);
    return v;
}

uint64_t get_u64(const uint8_t *p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v |= uint64_t(p[i]) << (8 * i);
    return v;
}

long long get_i64(const uint8_t *p) { return (long long)get_u64(p); }

float get_f32(const uint8_t *p) {
    const uint32_t b = get_u32(p);
    float v;
    std::memcpy(&v, &b, 4);
    return v;
}

double get_f64(const uint8_t *p) {
    const uint64_t b = get_u64(p);
    double v;
    std::memcpy(&v, &b, 8);
    return v;
}

}  // namespace

const char *msg_type_name(MsgType t) {
    switch (t) {
        case MsgType::Control: return "control";
        case MsgType::Trace: return "trace";
        case MsgType::Iq: return "iq";
    }
    return "unknown";
}

// ---------------------------------------------------------------- codes

uint8_t detector_code(Detector d) {
    switch (d) {
        case Detector::Peak: return 0;
        case Detector::NegativePeak: return 1;
        case Detector::Sample: return 2;
        case Detector::RMS: return 3;
    }
    return 0;
}

bool detector_from_code(uint8_t c, Detector *out) {
    switch (c) {
        case 0: *out = Detector::Peak; return true;
        case 1: *out = Detector::NegativePeak; return true;
        case 2: *out = Detector::Sample; return true;
        case 3: *out = Detector::RMS; return true;
    }
    return false;
}

uint8_t window_code(Window w) {
    switch (w) {
        case Window::Rectangular: return 0;
        case Window::Hann: return 1;
        case Window::BlackmanHarris: return 2;
        case Window::FlatTop: return 3;
    }
    return 1;
}

bool window_from_code(uint8_t c, Window *out) {
    switch (c) {
        case 0: *out = Window::Rectangular; return true;
        case 1: *out = Window::Hann; return true;
        case 2: *out = Window::BlackmanHarris; return true;
        case 3: *out = Window::FlatTop; return true;
    }
    return false;
}

const char *x_kind_name(XKind k) {
    switch (k) {
        case XKind::Frequency: return "frequency";
        case XKind::Time: return "time";
    }
    return "unknown";
}

bool x_kind_from_code(uint8_t c, XKind *out) {
    switch (c) {
        case 0: *out = XKind::Frequency; return true;
        case 1: *out = XKind::Time; return true;
    }
    return false;
}

// ---------------------------------------------------------------- header

void encode_header(MsgType type, uint32_t payload_len, std::vector<uint8_t> *out) {
    put_u32(out, payload_len);
    put_u8(out, uint8_t(type));
    put_u8(out, kVersion);
    put_u16(out, 0);
}

bool decode_header(const uint8_t *p, size_t n, Header *out, std::string *err) {
    auto fail = [&](const std::string &m) {
        if (err) *err = m;
        return false;
    };
    if (n < kHeaderBytes) return fail("short header");

    Header h;
    h.payload_len = get_u32(p);
    h.type = MsgType(p[4]);
    h.version = p[5];
    h.flags = get_u16(p + 6);

    if (h.version != kVersion)
        return fail("protocol version " + std::to_string(h.version) + ", expected " +
                    std::to_string(kVersion));
    if (h.payload_len > kMaxPayload)
        return fail("payload of " + std::to_string(h.payload_len) +
                    " bytes exceeds the " + std::to_string(kMaxPayload) + " byte cap");
    *out = h;
    return true;
}

// ---------------------------------------------------------------- trace

void set_overload_bit(std::vector<uint8_t> *mask, size_t n_segments, size_t i) {
    const size_t bytes = (n_segments + 7) / 8;
    if (mask->size() < bytes) mask->resize(bytes, 0);
    if (i >= n_segments) return;
    (*mask)[i >> 3] |= uint8_t(1u << (i & 7));
}

void encode_trace(const TraceMsg &m, std::vector<uint8_t> *out) {
    const uint32_t n_points = uint32_t(m.db.size());
    const uint32_t overload_bytes = (m.n_segments + 7) / 8;

    const uint32_t payload_len =
        uint32_t(kTraceHeadBytes) + n_points * 4u + overload_bytes;
    encode_header(MsgType::Trace, payload_len, out);

    const size_t head_at = out->size();
    put_u64(out, m.seq);
    put_f64(out, m.t_mono_s);
    put_i64(out, m.f_start_hz);
    put_i64(out, m.f_stop_hz);
    put_u32(out, n_points);
    put_u32(out, m.n_segments);
    put_f64(out, m.rbw_hz);
    put_f64(out, m.bin_hz);
    put_f64(out, m.sweep_seconds);
    put_f32(out, m.gain_db);
    put_u8(out, detector_code(m.detector));
    put_u8(out, window_code(m.window));
    put_u8(out, uint8_t(m.x_kind));
    put_u8(out, 0);                       // reserved
    put_u32(out, m.n_overloaded_segments);
    put_u32(out, overload_bytes);
    put_u64(out, m.dt_ns);
    put_i64(out, m.t0_ns);
    (void)head_at;

    // The bulk path. At the auto point count a full-span frame is tens of
    // thousands of floats and this runs once per sweep on a Cortex-A9, so the
    // memcpy is worth having; the per-float fallback keeps the format correct
    // anywhere else.
    const size_t at = out->size();
    out->resize(at + size_t(n_points) * 4);
    if (kHostIsLittleEndian) {
        if (n_points) std::memcpy(out->data() + at, m.db.data(), size_t(n_points) * 4);
    } else {
        for (uint32_t i = 0; i < n_points; ++i) {
            uint32_t b;
            std::memcpy(&b, &m.db[i], 4);
            uint8_t *q = out->data() + at + size_t(i) * 4;
            for (int k = 0; k < 4; ++k) q[k] = uint8_t(b >> (8 * k));
        }
    }

    // A caller that reported n_segments but never sized the mask still gets a
    // well-formed, all-clear bitmap rather than a length mismatch.
    for (uint32_t i = 0; i < overload_bytes; ++i)
        put_u8(out, i < m.overload.size() ? m.overload[i] : 0);
}

bool decode_trace(const uint8_t *p, size_t n, TraceMsg *out, std::string *err) {
    auto fail = [&](const std::string &m) {
        if (err) *err = m;
        return false;
    };
    if (n < kTraceHeadBytes) return fail("trace payload shorter than its header");

    TraceMsg m;
    m.seq = get_u64(p + 0);
    m.t_mono_s = get_f64(p + 8);
    m.f_start_hz = get_i64(p + 16);
    m.f_stop_hz = get_i64(p + 24);
    const uint32_t n_points = get_u32(p + 32);
    m.n_segments = get_u32(p + 36);
    m.rbw_hz = get_f64(p + 40);
    m.bin_hz = get_f64(p + 48);
    m.sweep_seconds = get_f64(p + 56);
    m.gain_db = get_f32(p + 64);

    if (!detector_from_code(p[68], &m.detector))
        return fail("unknown detector code " + std::to_string(p[68]));
    if (!window_from_code(p[69], &m.window))
        return fail("unknown window code " + std::to_string(p[69]));
    if (!x_kind_from_code(p[70], &m.x_kind))
        return fail("unknown x_kind code " + std::to_string(p[70]));

    m.n_overloaded_segments = get_u32(p + 72);
    const uint32_t overload_bytes = get_u32(p + 76);
    m.dt_ns = get_u64(p + 80);
    m.t0_ns = get_i64(p + 88);

    if (overload_bytes != (m.n_segments + 7) / 8)
        return fail("overload bitmap is " + std::to_string(overload_bytes) +
                    " bytes for " + std::to_string(m.n_segments) + " segments");

    // A time-domain frame with no interval has no x-axis at all: every point
    // would land at t=0 and the trace would draw as a single vertical line that
    // looks like a signal rather than like a missing field.
    if (m.x_kind == XKind::Time && m.dt_ns == 0)
        return fail("zero-span frame carries dt_ns = 0, so its time axis is "
                    "undefined");
    if (m.x_kind == XKind::Frequency && m.dt_ns != 0)
        return fail("frequency-domain frame carries dt_ns = " +
                    std::to_string(m.dt_ns) + "; dt_ns is meaningful only in zero span");
    // Same rule for the origin. A frequency frame with a non-zero t0 would draw
    // correctly and mean that the sender and the receiver disagree about which
    // axis this is, which is worth a dropped connection rather than a plot.
    if (m.x_kind == XKind::Frequency && m.t0_ns != 0)
        return fail("frequency-domain frame carries t0_ns = " +
                    std::to_string(m.t0_ns) + "; t0_ns is meaningful only in zero span");

    // Checked in 64-bit so a crafted n_points cannot wrap the sum back into
    // range and make a short payload look self-consistent.
    const uint64_t want = uint64_t(kTraceHeadBytes) + uint64_t(n_points) * 4u + overload_bytes;
    if (want != uint64_t(n))
        return fail("trace payload is " + std::to_string(n) + " bytes but declares " +
                    std::to_string(want) + " (" + std::to_string(n_points) +
                    " points); a short frame decoded leniently is a spectrum with a "
                    "plausible wrong end");

    m.db.resize(n_points);
    const uint8_t *fp = p + kTraceHeadBytes;
    if (kHostIsLittleEndian) {
        if (n_points) std::memcpy(m.db.data(), fp, size_t(n_points) * 4);
    } else {
        for (uint32_t i = 0; i < n_points; ++i) m.db[i] = get_f32(fp + size_t(i) * 4);
    }

    m.overload.assign(fp + size_t(n_points) * 4,
                      fp + size_t(n_points) * 4 + overload_bytes);
    *out = std::move(m);
    return true;
}

// ---------------------------------------------------------------- iq

const char *iq_trig_state_name(IqTrigState s) {
    switch (s) {
        case IqTrigState::Untriggered: return "untriggered";
        case IqTrigState::Triggered: return "triggered";
        case IqTrigState::AutoTimeout: return "auto";
    }
    return "unknown";
}

void encode_iq(const IqMsg &m, std::vector<uint8_t> *out) {
    const uint32_t n = uint32_t(m.iq.size() / 2);
    const size_t bps = m.format == IqFormat::Sc16 ? 4 : 8;
    encode_header(MsgType::Iq, uint32_t(kIqHeadBytes + size_t(n) * bps), out);

    put_u64(out, m.seq);
    put_f64(out, m.t_mono_s);
    put_i64(out, m.center_hz);
    put_f64(out, m.fs_hz);
    put_f64(out, m.adc_fs_hz);
    put_f64(out, m.rf_bw_hz);
    put_i64(out, m.t0_ns);
    put_f64(out, m.capture_s);
    put_f32(out, m.gain_db);
    put_f32(out, m.format == IqFormat::Sc16 ? m.full_scale : 1.0f);
    put_u32(out, n);
    put_u32(out, m.decim);
    put_u32(out, m.n_clipped);
    put_u32(out, m.n_acquisitions);
    put_u8(out, uint8_t(m.format));
    put_u8(out, m.trig_mode);
    put_u8(out, uint8_t(m.trig_state));
    put_u8(out, m.trig_source);
    put_u8(out, m.trig_rising ? 1 : 0);
    put_u8(out, 0);
    put_u8(out, 0);
    put_u8(out, 0);
    put_f32(out, m.trig_level);
    put_f32(out, m.trig_hyst);
    put_u32(out, m.trig_index);
    put_u32(out, 0);

    const size_t at = out->size();
    out->resize(at + size_t(n) * bps);
    uint8_t *q = out->data() + at;
    if (m.format == IqFormat::Sc16) {
        const float fs = m.full_scale;
        for (size_t i = 0; i < size_t(n) * 2; ++i) {
            // Clamped, not wrapped: a sample past full scale is a clip, and
            // int16 wraparound would turn it into a full-scale sample of the
            // opposite sign -- a glitch that looks like signal.
            // Round half away from zero by hand: lround is a libm call per
            // value, and on a single-core Pluto this loop shares the core with
            // the capture (half a million calls a burst at 256 k samples).
            float v = m.iq[i] * fs;
            v = std::max(-32768.0f, std::min(32767.0f, v));
            const int16_t c = int16_t(v >= 0 ? v + 0.5f : v - 0.5f);
            q[2 * i] = uint8_t(uint16_t(c));
            q[2 * i + 1] = uint8_t(uint16_t(c) >> 8);
        }
    } else if (kHostIsLittleEndian) {
        std::memcpy(q, m.iq.data(), size_t(n) * 8);
    } else {
        for (size_t i = 0; i < size_t(n) * 2; ++i) {
            uint32_t b;
            std::memcpy(&b, &m.iq[i], 4);
            for (int k = 0; k < 4; ++k) q[4 * i + size_t(k)] = uint8_t(b >> (8 * k));
        }
    }
}

bool decode_iq(const uint8_t *p, size_t n, IqMsg *out, std::string *err) {
    auto fail = [&](const std::string &msg) {
        if (err) *err = msg;
        return false;
    };
    if (n < kIqHeadBytes) return fail("iq payload shorter than its header");

    IqMsg m;
    m.seq = get_u64(p + 0);
    m.t_mono_s = get_f64(p + 8);
    m.center_hz = get_i64(p + 16);
    m.fs_hz = get_f64(p + 24);
    m.adc_fs_hz = get_f64(p + 32);
    m.rf_bw_hz = get_f64(p + 40);
    m.t0_ns = (int64_t)get_i64(p + 48);
    m.capture_s = get_f64(p + 56);
    m.gain_db = get_f32(p + 64);
    m.full_scale = get_f32(p + 68);
    const uint32_t ns = get_u32(p + 72);
    m.decim = get_u32(p + 76);
    m.n_clipped = get_u32(p + 80);
    m.n_acquisitions = get_u32(p + 84);
    const uint8_t fmt = p[88];
    m.trig_mode = p[89];
    const uint8_t ts = p[90];
    m.trig_source = p[91];
    m.trig_rising = p[92] != 0;
    m.trig_level = get_f32(p + 96);
    m.trig_hyst = get_f32(p + 100);
    m.trig_index = get_u32(p + 104);

    if (fmt > 1) return fail("unknown iq sample format " + std::to_string(fmt));
    m.format = IqFormat(fmt);
    if (ts > 2) return fail("unknown iq trigger state " + std::to_string(ts));
    m.trig_state = IqTrigState(ts);
    if (m.trig_mode > 2) return fail("unknown iq trigger mode " + std::to_string(m.trig_mode));
    if (m.trig_source > 2)
        return fail("unknown iq trigger source " + std::to_string(m.trig_source));
    // No rate means no time axis: every sample would land at t = 0.
    if (!(m.fs_hz > 0) || !(m.adc_fs_hz > 0))
        return fail("iq burst carries a non-positive sample rate, so its time axis is "
                    "undefined");
    if (m.decim == 0 || std::fabs(m.adc_fs_hz / double(m.decim) - m.fs_hz) > 1e-6 * m.fs_hz)
        return fail("iq burst says fs = " + std::to_string(m.fs_hz) + " but adc_fs / decim = " +
                    std::to_string(m.adc_fs_hz) + " / " + std::to_string(m.decim));
    if (m.format == IqFormat::Sc16 && !(m.full_scale > 0))
        return fail("sc16 iq burst carries full_scale <= 0");

    const uint64_t bps = m.format == IqFormat::Sc16 ? 4 : 8;
    const uint64_t want = uint64_t(kIqHeadBytes) + uint64_t(ns) * bps;
    if (want != uint64_t(n))
        return fail("iq payload is " + std::to_string(n) + " bytes but declares " +
                    std::to_string(want) + " (" + std::to_string(ns) + " samples)");
    if (m.trig_state == IqTrigState::Triggered && m.trig_index >= ns)
        return fail("iq burst places its trigger at sample " + std::to_string(m.trig_index) +
                    " of " + std::to_string(ns));

    const uint8_t *q = p + kIqHeadBytes;
    m.iq.resize(size_t(ns) * 2);
    if (m.format == IqFormat::Sc16) {
        const float inv = 1.0f / m.full_scale;
        for (size_t i = 0; i < size_t(ns) * 2; ++i)
            m.iq[i] = float(int16_t(get_u16(q + 2 * i))) * inv;
    } else if (kHostIsLittleEndian) {
        if (ns) std::memcpy(m.iq.data(), q, size_t(ns) * 8);
    } else {
        for (size_t i = 0; i < size_t(ns) * 2; ++i) m.iq[i] = get_f32(q + 4 * i);
    }
    *out = std::move(m);
    return true;
}

// ---------------------------------------------------------------- json

namespace {

void skip_ws(const std::string &s, size_t *i) {
    while (*i < s.size() && (s[*i] == ' ' || s[*i] == '\t' || s[*i] == '\n' || s[*i] == '\r'))
        ++*i;
}

bool parse_string(const std::string &s, size_t *i, std::string *out, std::string *err) {
    auto fail = [&](const char *m) {
        if (err) *err = m;
        return false;
    };
    if (*i >= s.size() || s[*i] != '"') return fail("expected a string");
    ++*i;
    out->clear();
    while (*i < s.size()) {
        const char c = s[*i];
        if (c == '"') {
            ++*i;
            return true;
        }
        if (c == '\\') {
            ++*i;
            if (*i >= s.size()) return fail("truncated escape");
            switch (s[*i]) {
                case '"': out->push_back('"'); break;
                case '\\': out->push_back('\\'); break;
                case '/': out->push_back('/'); break;
                case 'b': out->push_back('\b'); break;
                case 'f': out->push_back('\f'); break;
                case 'n': out->push_back('\n'); break;
                case 'r': out->push_back('\r'); break;
                case 't': out->push_back('\t'); break;
                case 'u': {
                    // Only the BMP subset that maps to one byte is accepted.
                    // The control schema is ASCII; anything else is rejected
                    // rather than mangled.
                    if (*i + 4 >= s.size()) return fail("truncated \\u escape");
                    unsigned code = 0;
                    for (int k = 1; k <= 4; ++k) {
                        const char h = s[*i + size_t(k)];
                        code <<= 4;
                        if (h >= '0' && h <= '9') code |= unsigned(h - '0');
                        else if (h >= 'a' && h <= 'f') code |= unsigned(h - 'a' + 10);
                        else if (h >= 'A' && h <= 'F') code |= unsigned(h - 'A' + 10);
                        else return fail("bad hex in \\u escape");
                    }
                    if (code > 0x7f) return fail("non-ASCII \\u escape is not supported");
                    out->push_back(char(code));
                    *i += 4;
                    break;
                }
                default: return fail("unknown escape");
            }
            ++*i;
            continue;
        }
        out->push_back(c);
        ++*i;
    }
    return fail("unterminated string");
}

}  // namespace

bool parse_flat_json(const std::string &s, JsonObject *out, std::string *err) {
    auto fail = [&](const std::string &m) {
        if (err) *err = m;
        return false;
    };

    JsonObject o;
    size_t i = 0;
    skip_ws(s, &i);
    if (i >= s.size() || s[i] != '{') return fail("expected an object");
    ++i;
    skip_ws(s, &i);

    if (i < s.size() && s[i] == '}') {
        ++i;
        skip_ws(s, &i);
        if (i != s.size()) return fail("trailing data after the object");
        *out = std::move(o);
        return true;
    }

    for (;;) {
        skip_ws(s, &i);
        std::string k;
        if (!parse_string(s, &i, &k, err)) return false;
        skip_ws(s, &i);
        if (i >= s.size() || s[i] != ':') return fail("expected ':' after key \"" + k + "\"");
        ++i;
        skip_ws(s, &i);
        if (i >= s.size()) return fail("truncated after key \"" + k + "\"");

        JsonValue v;
        const char c = s[i];
        if (c == '"') {
            if (!parse_string(s, &i, &v.str, err)) return false;
            v.kind = JsonValue::Kind::String;
        } else if (c == '{' || c == '[') {
            // Deliberate: see proto.h. A parser that half-understands nesting
            // at the edge of a socket is worse than one that refuses it.
            return fail("nested objects and arrays are not supported by this "
                        "control schema (key \"" + k + "\")");
        } else if (s.compare(i, 4, "true") == 0) {
            v.kind = JsonValue::Kind::Bool;
            v.b = true;
            i += 4;
        } else if (s.compare(i, 5, "false") == 0) {
            v.kind = JsonValue::Kind::Bool;
            v.b = false;
            i += 5;
        } else if (s.compare(i, 4, "null") == 0) {
            v.kind = JsonValue::Kind::Null;
            i += 4;
        } else {
            const char *begin = s.c_str() + i;
            char *end = nullptr;
            const double d = std::strtod(begin, &end);
            if (end == begin) return fail("bad value for key \"" + k + "\"");
            v.kind = JsonValue::Kind::Number;
            v.num = d;
            i += size_t(end - begin);
        }

        if (!o.emplace(k, std::move(v)).second)
            return fail("duplicate key \"" + k + "\"");

        skip_ws(s, &i);
        if (i < s.size() && s[i] == ',') {
            ++i;
            continue;
        }
        if (i < s.size() && s[i] == '}') {
            ++i;
            break;
        }
        return fail("expected ',' or '}'");
    }

    skip_ws(s, &i);
    if (i != s.size()) return fail("trailing data after the object");
    *out = std::move(o);
    return true;
}

std::string json_str(const JsonObject &o, const std::string &k, const std::string &dflt) {
    const auto it = o.find(k);
    if (it == o.end() || !it->second.is_str()) return dflt;
    return it->second.str;
}

double json_num(const JsonObject &o, const std::string &k, double dflt) {
    const auto it = o.find(k);
    if (it == o.end() || !it->second.is_num()) return dflt;
    return it->second.num;
}

bool json_bool(const JsonObject &o, const std::string &k, bool dflt) {
    const auto it = o.find(k);
    if (it == o.end() || it->second.kind != JsonValue::Kind::Bool) return dflt;
    return it->second.b;
}

JsonWriter &JsonWriter::key(const std::string &k) {
    if (!first_) s_ += ',';
    first_ = false;
    s_ += '"';
    for (char c : k) {
        if (c == '"' || c == '\\') s_ += '\\';
        s_ += c;
    }
    s_ += "\":";
    want_value_ = true;
    return *this;
}

JsonWriter &JsonWriter::str(const std::string &v) {
    s_ += '"';
    for (char c : v) {
        switch (c) {
            case '"': s_ += "\\\""; break;
            case '\\': s_ += "\\\\"; break;
            case '\n': s_ += "\\n"; break;
            case '\r': s_ += "\\r"; break;
            case '\t': s_ += "\\t"; break;
            default:
                if (uint8_t(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof buf, "\\u%04x", unsigned(uint8_t(c)));
                    s_ += buf;
                } else {
                    s_ += c;
                }
        }
    }
    s_ += '"';
    want_value_ = false;
    return *this;
}

JsonWriter &JsonWriter::num(double v) {
    if (!std::isfinite(v)) return null();   // JSON has no NaN or Infinity
    char buf[40];
    std::snprintf(buf, sizeof buf, "%.17g", v);
    s_ += buf;
    want_value_ = false;
    return *this;
}

JsonWriter &JsonWriter::num(long long v) {
    s_ += std::to_string(v);
    want_value_ = false;
    return *this;
}

JsonWriter &JsonWriter::num(uint64_t v) {
    s_ += std::to_string(v);
    want_value_ = false;
    return *this;
}

JsonWriter &JsonWriter::boolean(bool v) {
    s_ += v ? "true" : "false";
    want_value_ = false;
    return *this;
}

JsonWriter &JsonWriter::null() {
    s_ += "null";
    want_value_ = false;
    return *this;
}

std::string JsonWriter::done() {
    if (want_value_) s_ += "null";   // a key with no value would not parse
    s_ += '}';
    want_value_ = false;
    return s_;
}

void encode_control(const std::string &json, std::vector<uint8_t> *out) {
    encode_header(MsgType::Control, uint32_t(json.size()), out);
    out->insert(out->end(), json.begin(), json.end());
}

// ---------------------------------------------------------------- reader

void Reader::feed(const uint8_t *p, size_t n) {
    if (!err_.empty()) return;
    // Compacting only when the consumed prefix has grown past half the buffer
    // keeps this amortised O(1) rather than memmoving on every message.
    if (head_ > 0 && head_ * 2 >= buf_.size()) {
        buf_.erase(buf_.begin(), buf_.begin() + long(head_));
        head_ = 0;
    }
    buf_.insert(buf_.end(), p, p + n);
}

bool Reader::next(Header *hdr, const uint8_t **payload) {
    if (!err_.empty()) return false;
    if (buf_.size() - head_ < kHeaderBytes) return false;

    Header h;
    if (!decode_header(buf_.data() + head_, buf_.size() - head_, &h, &err_)) return false;
    if (buf_.size() - head_ < kHeaderBytes + h.payload_len) return false;

    *hdr = h;
    *payload = buf_.data() + head_ + kHeaderBytes;
    head_ += kHeaderBytes + h.payload_len;
    return true;
}

void Reader::reset() {
    buf_.clear();
    head_ = 0;
    err_.clear();
}

}  // namespace proto
}  // namespace sweepd
