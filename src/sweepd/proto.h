// The sweepd <-> sweepui wire protocol. CLAUDE.md §5, "Wire protocol".
//
// This file is the contract between two binaries that are built in different
// phases and, per §6, on different operating systems. Everything about the
// format is pinned here and in tests/proto_test.cpp, and nothing in it depends
// on libiio, on the sweep engine, or on a device -- so the decoder compiles into
// sweepui unchanged.
//
// ------------------------------------------------------------------ framing
//
// §5: "TCP, length-prefixed, little-endian." Every message is
//
//   offset size field
//        0    4 u32  payload_len   bytes following this 8-byte header
//        4    1 u8   type          MsgType
//        5    1 u8   version       kVersion
//        6    2 u16  flags         reserved, 0
//        8    n      payload
//
// The header is 8 bytes rather than the minimal 5 so that the payload starts
// 8-byte aligned, which lets the f64 fields and the f32 array be memcpy'd on
// the little-endian machines this actually runs on instead of assembled byte at
// a time. At 65536 points a trace frame is 256 kB; on a Cortex-A9 the
// difference is milliseconds per frame, which is a meaningful fraction of a
// sweep.
//
// Little-endian is the wire order, not the host's. Both ends here are LE, but
// the conversion is written explicitly so that stays a fact about the wire and
// not an assumption about the reader.
//
// ------------------------------------------------------------------- types
//
//   0x01 control   flat JSON, bidirectional. Also carries the §5 status
//                  message and the configuration dump.
//   0x02 trace     the §5 frame: fixed header + float32[n_points] in dBFS.
//                  Carries both the swept spectrum and the zero-span trace --
//                  see "one trace format" below.
//   0x03 iq        one raw IQ burst: fixed header + n_samples complex samples,
//                  sc16 or cf32. See "0x03 iq" below.
//
// -------------------------------------------------------- one trace format
//
// A zero-span trace is a trace whose x-axis is time. It comes off the same
// window, the same ENBW, the same detector and the same overload counter as a
// swept one -- the LO is parked and the pipeline emits one power value per FFT
// instead of decimating across a bin grid. So it travels as an 0x02 frame with
// two extra header fields rather than as a second trace format:
//
//   x_kind   0 = frequency, 1 = time
//   dt_ns    the per-point interval, meaningful only when x_kind is time
//
// In zero span f_start_hz == f_stop_hz == the parked centre, and rbw_hz,
// detector, window and the overload bitmap keep their exact meanings.
//
// The payoff is on the receiving end: markers, all four traces, max/min hold,
// averaging, amplitude correction, the UNCAL badge, CSV export and the
// waterfall all work in zero span with no new code and no second DSP path to
// drift out of sync, and a marker reads the same way in both modes because both
// numbers came out of the same pipeline.
//
// The alternative -- reinterpreting f_start_hz as a start time under a flag --
// was rejected: a decoder that missed the flag would draw a spectrum from 0 Hz
// to 1 ms and look entirely plausible doing it, which is the class of failure §7
// exists to prevent. Time fields are named for time.
//
// ------------------------------------------------------------------- 0x03 iq
//
// The time sink's data: the receiver's own complex baseband, I and Q against
// time, as GNU Radio's QT GUI Time Sink draws it. It is *not* a stream.
//
//   * One burst is one contiguous acquisition -- one DMA refill -- so there is
//     never a gap inside it. sc16 at 61.44 MSPS is ~245 MB/s against a
//     40-70 MB/s link (§2): a "throttled continuous" stream could only be
//     gapped or decimated without saying where, which is useless for every job
//     raw IQ is wanted for.
//   * A burst is sent only on request: `iq_arm` asks for exactly one. A UI in
//     run mode re-arms when it has the previous one, so the link sets the pace,
//     nothing queues on the device, and the dead time between bursts is a
//     number the UI measures and shows (as a duty figure, like zero span's) --
//     the gap is between bursts, where the user can see it, never inside one.
//     Single is one arm and no re-arm.
//   * The sample rate is the session's: the AD936x rate is never changed for
//     this, because that reprograms the BBPLL and invalidates every fastlock
//     profile (§5). Longer windows come from an integer decimator on the ARM
//     (iq.h), and `decim`, `fs_hz` and `adc_fs_hz` say which happened.
//
// Header (kIqHeadBytes, little-endian, offsets from the payload start):
//
//     0  u64 seq             per-connection burst counter
//     8  f64 t_mono_s        device CLOCK_MONOTONIC at capture start (§2: not a date)
//    16  i64 center_hz       the RF frequency at baseband 0 Hz -- the parked LO
//    24  f64 fs_hz           sample rate of THIS payload, after decimation
//    32  f64 adc_fs_hz       the converter's rate; fs_hz = adc_fs_hz / decim
//    40  f64 rf_bw_hz        analog filter bandwidth
//    48  i64 t0_ns           time of sample 0 against the trigger edge: negative by
//                            the pre-trigger history, 0 when untriggered (as 0x02)
//    56  f64 capture_s       span of input the trigger searched, so an Auto
//                            timeout can say how long it looked
//    64  f32 gain_db         RX gain in force
//    68  f32 full_scale      sc16 counts per full scale (2048 for 12 bits); 1 for cf32
//    72  u32 n_samples
//    76  u32 decim           1 = raw ADC samples, no filter
//    80  u32 n_clipped       input samples at/over the overload threshold, counted
//                            BEFORE decimation -- a filter would hide the clip
//    84  u32 n_acquisitions  captures it took (Normal/Auto keep capturing until
//                            an edge turns up or the auto window ends)
//    88  u8  format          0 = sc16, 1 = cf32
//    89  u8  trig_mode       IqTrigMode
//    90  u8  trig_state      0 = untriggered (free run), 1 = triggered,
//                            2 = auto: no edge, sent anyway
//    91  u8  trig_source     IqTrigSource
//    92  u8  trig_rising
//    93  u8[3] reserved
//    96  f32 trig_level      FS
//   100  f32 trig_hyst       FS
//   104  u32 trig_index      sample of the edge (valid when trig_state == 1)
//   108  u32 reserved
//   112  samples            n_samples x {I, Q}: int16 pairs or float32 pairs
//
// sc16 is what the radio produces and is sent when decim == 1: half the bytes,
// and the samples are exactly the ADC's. Once decimated the samples carry more
// resolution than 12 bits and go as cf32 in full-scale units.
//
// The 0x03 type was added without a version bump: an older peer never sends
// it, and the 0x01 and 0x02 layouts are unchanged.

// ------------------------------------------------------------------- dBFS
//
// The float32 array is dBFS and carries NaN for bins no segment covered, which
// is how dc_policy=blank reports its holes. NaN survives the wire as its IEEE
// 754 bit pattern; §7 wants those bins visibly missing rather than interpolated
// into something plausible, so the decoder must not "clean" them.

#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "dsp.h"

namespace sweepd {
namespace proto {

// 2 was zero span: the 0x02 header gained x_kind and dt_ns, which moved the
// float array and grew the trace header from 80 bytes to 88. 3 is the video
// trigger: t0_ns, so a trace can start before its own origin. Both are breaking
// changes to a cross-binary contract, taken deliberately -- both binaries are
// built and deployed together and there is no released version to stay
// compatible with. An older peer is refused by decode_header() with the version
// in the message rather than being fed a frame whose payload it would misread.
constexpr uint8_t kVersion = 3;
constexpr size_t kHeaderBytes = 8;
constexpr size_t kTraceHeadBytes = 96;
constexpr size_t kIqHeadBytes = 112;

// A cap so a corrupt or hostile length prefix cannot make either end allocate
// unboundedly. 64 MB is far above the largest legitimate message: the biggest
// trace frame the daemon can produce is max_points=65536 floats, 256 kB.
constexpr uint32_t kMaxPayload = 64u * 1024 * 1024;

enum class MsgType : uint8_t {
    Control = 0x01,
    Trace = 0x02,
    Iq = 0x03,   // one raw IQ burst; see "0x03 iq" above
};

const char *msg_type_name(MsgType t);

// ---------------------------------------------------------------- codes
//
// Wire codes for the two enums that travel in the trace header. They are
// defined here, not derived from dsp.h, because dsp.h's enumerator order is an
// implementation detail that may be reordered, whereas these are a published
// format. tests/proto_test.cpp pins the two to each other, so a reorder is a
// test failure rather than a silent misinterpretation at the far end.

uint8_t detector_code(Detector d);
bool detector_from_code(uint8_t c, Detector *out);
uint8_t window_code(Window w);
bool window_from_code(uint8_t c, Window *out);

// What the trace's x-axis measures. Sent explicitly rather than inferred from,
// say, f_start_hz == f_stop_hz, because an inference has no way to distinguish
// "zero span" from "a degenerate sweep" and would guess in a case where being
// wrong is invisible.
enum class XKind : uint8_t {
    Frequency = 0,
    Time = 1,
};

const char *x_kind_name(XKind k);
bool x_kind_from_code(uint8_t c, XKind *out);

// ---------------------------------------------------------------- header

struct Header {
    uint32_t payload_len = 0;
    MsgType type = MsgType::Control;
    uint8_t version = kVersion;
    uint16_t flags = 0;
};

// Appends the 8-byte header for `payload_len` bytes of `type` to `out`.
void encode_header(MsgType type, uint32_t payload_len, std::vector<uint8_t> *out);

// Reads a header from `n` bytes at `p`. False if fewer than kHeaderBytes are
// available, if the version is not understood, or if payload_len exceeds
// kMaxPayload.
bool decode_header(const uint8_t *p, size_t n, Header *out, std::string *err);

// ---------------------------------------------------------------- trace

// The §5 0x02 frame, decoupled from the engine's TraceFrame so that a decoder
// needs neither the sweep engine nor pthread. sweepd converts one to the other;
// sweepui only ever sees this.
//
// §5 asks the header for `seq`, a host-side timestamp, `f_start_hz`,
// `f_stop_hz`, `n_points`, `rbw_hz`, `detector`, `gain_db` and an
// `overload_mask`. Two of those need explaining:
//
//   t_mono_s is the *device's* CLOCK_MONOTONIC, not a wall clock. §2: the E310
//   has no RTC and its clock reads 1970, so the device cannot produce a
//   meaningful absolute time. It sends an interval-valid one and the receiver
//   stamps arrival itself -- which is what §2 means by stamping frames
//   host-side. The field is named for what it is so nobody mistakes it for a
//   date.
//
//   overload_mask is one bit per *segment*, not a fixed-width integer. A full
//   span is 298 segments under dc=splice, so a u64 would silently report only
//   the first 64. §5 requires that an overloaded front end never be silent,
//   and a mask that quietly drops four fifths of the sweep would be exactly
//   that. Zero span reports n_segments == 1: the parked LO is one acquisition,
//   and the front end can be overdriven there exactly as it can while sweeping.
struct TraceMsg {
    uint64_t seq = 0;
    double t_mono_s = 0;
    long long f_start_hz = 0;
    long long f_stop_hz = 0;
    uint32_t n_segments = 0;
    double rbw_hz = 0;
    double bin_hz = 0;          // (f_stop - f_start) / n_points, sent so the
                                // receiver never has to re-derive the axis.
                                // 0 in zero span: there is no frequency grid,
                                // and a plausible-looking wrong one is worse
                                // than an obviously absent one.
    double sweep_seconds = 0;
    float gain_db = 0;
    Detector detector = Detector::Peak;
    Window window = Window::Hann;
    uint32_t n_overloaded_segments = 0;

    // The x-axis. In Frequency mode the axis runs f_start_hz..f_stop_hz, and
    // dt_ns and t0_ns are 0; in Time mode it runs t0_ns..t0_ns+n_points*dt_ns
    // and f_start_hz == f_stop_hz == the parked centre.
    XKind x_kind = XKind::Frequency;
    uint64_t dt_ns = 0;

    // Where the trace's own clock starts, and the only field on this frame that
    // is allowed to be negative.
    //
    // Free run has nothing to measure from and sends 0: successive frames begin
    // at unrelated moments, so an absolute time would be a number with nothing
    // behind it. A *triggered* frame does have an origin -- the edge -- and t0
    // is negative by exactly the pre-trigger history, so the points captured
    // before the edge read as the negative times they are.
    //
    // It is on the frame rather than left to the receiver to work out from
    // `config`, because config and frames are briefly out of step across a
    // reconfiguration and an axis assembled from two sources would be confident
    // and wrong for those few frames. A frame carries its own axis.
    int64_t t0_ns = 0;

    std::vector<float> db;             // n_points, dBFS, NaN where unmeasured
    std::vector<uint8_t> overload;     // (n_segments+7)/8 bytes, bit i LSB-first

    size_t n_points() const { return db.size(); }
    bool segment_overloaded(size_t i) const {
        return i < n_segments && (overload[i >> 3] >> (i & 7)) & 1u;
    }

    // Centre of output point `i` on whichever axis this frame carries, in Hz or
    // in seconds. One place, so the two modes cannot disagree about where a
    // marker sits -- and so a caller cannot accidentally use bin_hz on a frame
    // that has none.
    double x_at(size_t i) const {
        if (x_kind == XKind::Time)
            return double(t0_ns) * 1e-9 + (double(i) + 0.5) * double(dt_ns) * 1e-9;
        return double(f_start_hz) + (double(i) + 0.5) * bin_hz;
    }
};

// Sets bit `i` in a bitmap sized for `n_segments`. Resizes if needed.
void set_overload_bit(std::vector<uint8_t> *mask, size_t n_segments, size_t i);

// Appends a complete framed message (header + payload) to `out`.
void encode_trace(const TraceMsg &m, std::vector<uint8_t> *out);

// Decodes a trace payload -- `p`/`n` are the payload only, header already
// consumed. False with an explanation on any inconsistency; in particular a
// payload whose declared n_points and overload_bytes do not add up to its
// actual length is rejected rather than truncated, because a short frame
// decoded leniently is a spectrum with a plausible-looking wrong end.
bool decode_trace(const uint8_t *p, size_t n, TraceMsg *out, std::string *err);

// ---------------------------------------------------------------- iq

enum class IqFormat : uint8_t { Sc16 = 0, Cf32 = 1 };
enum class IqTrigState : uint8_t { Untriggered = 0, Triggered = 1, AutoTimeout = 2 };

const char *iq_trig_state_name(IqTrigState s);

// One 0x03 burst. `iq` is always float in full-scale units, interleaved, on
// both sides of the wire: the encoder quantises to sc16 when `format` says so,
// and the decoder divides by full_scale, so no consumer handles two layouts.
struct IqMsg {
    uint64_t seq = 0;
    double t_mono_s = 0;
    long long center_hz = 0;
    double fs_hz = 0;
    double adc_fs_hz = 0;
    double rf_bw_hz = 0;
    int64_t t0_ns = 0;
    double capture_s = 0;
    float gain_db = 0;
    float full_scale = 2048.0f;
    uint32_t decim = 1;
    uint32_t n_clipped = 0;
    uint32_t n_acquisitions = 1;
    IqFormat format = IqFormat::Sc16;
    uint8_t trig_mode = 0;      // sweepd::IqTrigMode
    IqTrigState trig_state = IqTrigState::Untriggered;
    uint8_t trig_source = 0;    // sweepd::IqTrigSource
    bool trig_rising = true;
    float trig_level = 0;
    float trig_hyst = 0;
    uint32_t trig_index = 0;

    std::vector<float> iq;      // 2 x n_samples, FS

    size_t n_samples() const { return iq.size() / 2; }
    // Time of sample i, seconds, on the burst's own axis (t = 0 at the edge).
    double t_at(size_t i) const { return double(t0_ns) * 1e-9 + double(i) / fs_hz; }
};

void encode_iq(const IqMsg &m, std::vector<uint8_t> *out);
// Payload only, header consumed. Rejects any inconsistency -- a length that
// does not match n_samples, an unknown format or trigger code, a non-positive
// rate -- rather than drawing a waveform on a guessed axis.
bool decode_iq(const uint8_t *p, size_t n, IqMsg *out, std::string *err);

// ---------------------------------------------------------------- control

// Flat JSON only: an object whose values are strings, numbers, booleans or
// null. No nesting, no arrays.
//
// That restriction is the design, not a shortcut deferred until later. The
// control channel carries commands and a status dump, both of which are flat,
// and a parser that rejects what it does not handle is safer at the edge of a
// network socket than one that half-understands it. If the schema ever needs
// nesting, vendor a real JSON library rather than growing this one.

struct JsonValue {
    enum class Kind { Null, Bool, Number, String } kind = Kind::Null;
    bool b = false;
    double num = 0;
    std::string str;

    bool is_num() const { return kind == Kind::Number; }
    bool is_str() const { return kind == Kind::String; }
};

using JsonObject = std::map<std::string, JsonValue>;

// False on nesting, trailing garbage, or malformed input -- never a partial
// object.
bool parse_flat_json(const std::string &s, JsonObject *out, std::string *err);

// Convenience readers. Return the default when absent or of the wrong kind,
// which keeps command handlers from being a wall of type checks.
std::string json_str(const JsonObject &o, const std::string &k,
                     const std::string &dflt = "");
double json_num(const JsonObject &o, const std::string &k, double dflt = 0);
bool json_bool(const JsonObject &o, const std::string &k, bool dflt = false);

// Builder for a flat object. Emits numbers with enough precision to round-trip
// a double, and writes non-finite values as null: JSON has no NaN or Infinity,
// and emitting the bare token would produce a document that a real parser at
// the other end rejects.
class JsonWriter {
public:
    JsonWriter &key(const std::string &k);
    JsonWriter &str(const std::string &v);
    JsonWriter &num(double v);
    JsonWriter &num(long long v);
    JsonWriter &num(uint64_t v);
    JsonWriter &boolean(bool v);
    JsonWriter &null();

    // Shorthands: writer.kv("cmd", "hello").
    JsonWriter &kv(const std::string &k, const std::string &v) { return key(k).str(v); }
    JsonWriter &kv(const std::string &k, const char *v) { return key(k).str(v); }
    JsonWriter &kv(const std::string &k, double v) { return key(k).num(v); }
    JsonWriter &kv(const std::string &k, long long v) { return key(k).num(v); }
    JsonWriter &kv(const std::string &k, uint64_t v) { return key(k).num(v); }
    JsonWriter &kv(const std::string &k, bool v) { return key(k).boolean(v); }

    std::string done();   // closes the object; the writer is spent afterwards

private:
    std::string s_ = "{";
    bool first_ = true;
    bool want_value_ = false;
};

// Appends a framed 0x01 control message carrying `json`.
void encode_control(const std::string &json, std::vector<uint8_t> *out);

// ---------------------------------------------------------------- reader
//
// TCP is a byte stream: a read() can return half a header, three frames, or a
// header plus part of the next payload. Feed everything that arrives to feed()
// and take whole messages out with next(). This is the piece it is easy to get
// almost right and then see fail only under load, so it is tested directly
// against adversarial chunk boundaries.

class Reader {
public:
    void feed(const uint8_t *p, size_t n);

    // Pops one complete message. `payload` points into the reader's buffer and
    // is valid until the next feed() or next() call.
    bool next(Header *hdr, const uint8_t **payload);

    // Set once the stream is unusable -- a bad version or an oversized length
    // means the byte stream is no longer trustworthy and the connection must be
    // dropped, not resynchronised. There are no frame markers to resynchronise
    // to, by design: adding them would let a corrupt stream look recoverable.
    bool failed() const { return !err_.empty(); }
    const std::string &error() const { return err_; }

    size_t buffered() const { return buf_.size() - head_; }
    void reset();

private:
    std::vector<uint8_t> buf_;
    size_t head_ = 0;
    std::string err_;
};

}  // namespace proto
}  // namespace sweepd
