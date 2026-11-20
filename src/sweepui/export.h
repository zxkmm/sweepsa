// Export. CLAUDE.md §6: "trace CSV, screenshot PNG, instrument state to JSON."
//
// No ImGui and no GL, so tests/export_test.cpp exercises all three natively --
// including the PNG encoder, which is the one piece of this project that has to
// produce a byte-exact format another program will read.
//
// -------------------------------------------------------------- on the CSV
//
// An exported trace outlives the session it came from, and the thing that makes
// it useless a month later is not a missing column, it is a missing RBW. So the
// header carries everything needed to interpret the numbers -- the span, the
// achieved RBW, the window, the detector, the point count, the gain, and
// whether the run was calibrated or synthetic -- in the same `# key=value`
// comment style sweepd's own CSV uses (src/sweepd/main.cpp), so the two can be
// read by the same eye and the same script.
//
// NaN is written as the literal `nan`, matching sweepd. §7: an unmeasured bin
// stays unmeasured through the export. Writing it as an empty field, a zero or
// a floor value would put a measurement into the file that was never made.
//
// -------------------------------------------------------------- on the PNG
//
// Written here rather than pulled in as a dependency. The alternative was a
// fourth vendored library for one function, and PNG's encode side is small
// enough to own: a fixed-Huffman deflate stream, a CRC-32 and an Adler-32.
// What is *not* small is getting it subtly wrong -- a stream that one decoder
// accepts and another rejects -- so the test round-trips through an
// independently written inflate, and the encoder is additionally checked
// against a real zlib (see docs/PROJECT-STATE.md).
//
// ------------------------------------------------------------ on the state
//
// The state file is flat, with dotted keys (`trace.1.mode`, `marker.3.freq_hz`)
// rather than nested objects, so it can be read back by proto.h's flat JSON
// parser. That parser exists, is tested, and refuses what it cannot represent;
// vendoring a real JSON library to nest four traces and eight markers would add
// a dependency to gain punctuation. The flattening is visible in the key names
// rather than hidden, which is the trade being made.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "dsp.h"

namespace sweepui {

// ---------------------------------------------------------------- trace CSV

// The metadata that makes an exported trace readable later. Every field is
// something the receiver cannot recover from the numbers themselves.
struct CsvMeta {
    std::string source;          // the FrameSource's name
    long long f_start_hz = 0;    // of the *trace data*, not of the view
    long long f_stop_hz = 0;
    double bin_hz = 0;
    double rbw_hz = 0;
    double sweeps_per_s = 0;
    uint64_t seq = 0;
    uint32_t n_segments = 0;
    uint32_t n_overloaded_segments = 0;
    float gain_db = 0;
    sweepd::Detector detector = sweepd::Detector::Peak;
    sweepd::Window window = sweepd::Window::Hann;
    bool uncal = true;
    bool synthetic = true;

    // Why `uncal` is set, in the exporter's own words. "no correction loaded"
    // and "the correction is relative, so there is no reference level" are
    // different facts about a file, and a reader who sees only `uncal=true`
    // cannot tell whether the trace had its frequency response removed. Empty
    // when `uncal` is false; then the header says nothing extra.
    std::string uncal_reason;
    // Overrides the dBFS/dBm unit when set: a normalized tracking trace is
    // dB(S21), a ratio, and neither of those.
    std::string units;
    std::string units_note;

    // Zero span: the x column is seconds from the start of the frame rather
    // than hertz, and f_start_hz carries the parked centre. The header names
    // the axis before anything else, so a reader cannot take an amplitude-over-
    // time export for a spectrum.
    bool time_axis = false;
    double dt_s = 0;
    double time_res_s = 0;       // n_fft/fs; the overlap does not improve it

    // Where the trace's clock starts. 0 in free run, which has no origin to
    // measure from; negative under a video trigger, by exactly the pre-trigger
    // history, so t=0 is the edge. Exporting from 0 regardless would shift
    // every timestamp in the file by that amount and look perfectly ordinary.
    double t0_s = 0;
    std::string trigger;         // empty unless a trigger produced this trace
};

// One column. `db` must have the same length as every other column.
struct CsvColumn {
    std::string name;            // "trace1", and the mode in the header comment
    std::string mode;
    const float *db = nullptr;
    size_t n = 0;
};

// Writes `# key=value` metadata, a header row, then one row per bin:
// frequency in Hz followed by one column per trace.
//
// False with a reason if the file cannot be opened or the columns disagree
// about their length -- a CSV whose columns are different traces of different
// lengths would silently misalign every row after the shortest.
bool write_trace_csv(const std::string &path, const CsvMeta &meta,
                     const std::vector<CsvColumn> &columns, std::string *err);

// The same content to a string, which is what the test reads.
bool format_trace_csv(const CsvMeta &meta, const std::vector<CsvColumn> &columns,
                      std::string *out, std::string *err);

// ---------------------------------------------------------------- PNG

// Encodes 8-bit RGB into a complete PNG file.
//
// `rgb` is w*h*3 bytes. `bottom_up` is for OpenGL's framebuffer, whose origin
// is bottom-left where PNG's is top-left; passing it wrong produces an
// upside-down screenshot, which is at least obvious.
bool encode_png_rgb(const uint8_t *rgb, int w, int h, bool bottom_up,
                    std::vector<uint8_t> *out, std::string *err);

bool write_png_rgb(const std::string &path, const uint8_t *rgb, int w, int h,
                   bool bottom_up, std::string *err);

// Exposed for the test, which checks the encoder's own integrity fields against
// independently computed ones rather than against itself.
uint32_t crc32_of(const uint8_t *p, size_t n);
uint32_t adler32_of(const uint8_t *p, size_t n);

// ---------------------------------------------------------------- state

// §6's "instrument state to JSON". Deliberately a plain struct of what the user
// set, with no pointers into the app: saving state must not save a moment of
// the *data*, only the configuration that would reproduce the view.
struct StateTrace {
    std::string mode = "blank";
    int average_n = 10;
};

struct StateMarker {
    bool on = false;
    // Hz while sweeping, seconds in zero span -- whichever axis the marker was
    // placed on. Written as `marker.N.x`; `marker.N.freq_hz` is still read, so
    // state files saved before zero span existed still load.
    double x = 0;
    bool delta = false;
    int ref = 0;
    int trace = 0;
    bool noise = false;
};

struct InstrumentState {
    // Display
    double view_start_hz = 0;
    double view_stop_hz = 0;
    double ref_db = 0;
    double db_per_div = 10;
    bool lin_scale = false;
    double wf_ref_db = -40;
    double wf_range_db = 60;
    int colormap = 0;
    bool auto_tune = true;
    bool show_band = true;
    bool fill_band = false;
    // §7's segment boundary overlay. In the state file because it is a view
    // setting like the others here; the *plan* it draws is not, because that
    // belongs to whatever source is running when the file is loaded.
    bool show_segments = false;

    // What the source was asked for. Restoring this is a retune, so the loader
    // hands it back and lets the app decide whether to apply it.
    double src_start_hz = 0;
    double src_stop_hz = 0;
    int n_points = 0;
    std::string detector = "peak";
    std::string window = "hann";
    // The segment width and DC policy. Here because they set the sweep rate,
    // and a state file that restored the span and the detector but not these
    // would come back at a different rate with no visible reason -- 0 and ""
    // mean the file predates them, and then nothing is asked for.
    double keep_hz = 0;
    std::string dc_policy;

    // Search
    double peak_excursion_db = 6.0;
    double peak_threshold_db = -200.0;

    StateTrace trace[4];
    StateMarker marker[8];
    int active_marker = 0;
};

std::string state_to_json(const InstrumentState &s);

// False with a reason on malformed JSON. Missing keys keep their defaults, so
// a state file written by an older build loads rather than being rejected --
// but a *malformed* one is refused outright, because a half-applied instrument
// state is a screen that does not match any of its own controls.
bool state_from_json(const std::string &json, InstrumentState *out, std::string *err);

bool write_state_json(const std::string &path, const InstrumentState &s,
                      std::string *err);
bool read_state_json(const std::string &path, InstrumentState *out, std::string *err);

}  // namespace sweepui
