// Where sweepui gets its traces. CLAUDE.md §6, "Synthetic source":
//
//   "a built-in fake generator so the whole UI develops with no hardware
//    attached. Build this early."
//
// §10 puts the whole UI (step 3) before wiring it to the daemon (step 4), so
// this interface exists to make that ordering real: the UI is written against
// FrameSource, the synthetic generator implements it now, and the TCP client
// implements it in step 4 without the UI noticing.
//
// The shape of the interface is taken from what the daemon actually sends -- a
// stream of 0x02 trace frames plus two 0x01 control messages, `config` and
// `status` (see src/sweepd/proto.h and tools/sweep_client/main.cpp). Nothing is
// added here that the wire cannot carry, because a UI built against a richer
// interface than the protocol provides is a UI that will have to be rewritten.
//
// ---------------------------------------------------------------- refusals
//
// The setters return false with a reason rather than being absent, because
// sweepd today *refuses* runtime reconfiguration: it takes its configuration at
// start() and answers anything else with an explicit error naming the
// limitation (PROJECT-STATE.md, "Runtime reconfiguration is not implemented").
// The UI must therefore be able to show a refusal. A control that silently does
// nothing is the failure mode §7 exists to prevent, one level up.

#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "dsp.h"
#include "gen.h"
#include "iq.h"
#include "proto.h"

namespace sweepui {

// One tracking-generator request. NaN in a double and a negative int mean
// "leave that one alone", so one soft key sends one field and the refusal
// names the knob that was wrong.
struct TrackRequest {
    double atten_db = std::numeric_limits<double>::quiet_NaN();
    double spacing_hz = std::numeric_limits<double>::quiet_NaN();
    int tones = -1;
    int coherent = -1;
    int harmonic = -1;
    double rx_offset_hz = std::numeric_limits<double>::quiet_NaN();
    int rx_invert = -1;   // 0 / 1
    int stim_off = -1;    // 0 / 1
};

// One IQ (time sink) request. Every field has a "leave it alone" value -- NaN,
// a negative count, an empty string -- so one soft key sends one knob and a
// refusal names the knob that was wrong, the same rule TrackRequest follows.
struct IqRequest {
    double center_hz = std::numeric_limits<double>::quiet_NaN();
    long long n_samples = -1;          // output samples per burst
    int decim = -1;                    // 1, 2, 4 ... 64
    std::string trig_mode;             // none | normal | auto
    std::string trig_source;           // i | q | mag
    int trig_rising = -1;              // 0 / 1
    double trig_level = std::numeric_limits<double>::quiet_NaN();   // FS
    double trig_hyst = std::numeric_limits<double>::quiet_NaN();    // FS
    double pretrig = std::numeric_limits<double>::quiet_NaN();      // 0..0.95
    double auto_s = std::numeric_limits<double>::quiet_NaN();       // Auto's patience
};

// The 0x01 `config` message, plus the one thing the wire does not carry: whether
// this is real. §7's UNCAL badge is about amplitude; this is about the whole
// trace, and it earns a badge of its own for the same reason.
struct SourceConfig {
    long long f_start_hz = 0;
    long long f_stop_hz = 0;

    // What the *tuner* can reach, as distinct from what is being swept right
    // now. Without this "full span" can only mean "the span the daemon happened
    // to be started with", which is not what full span means on an instrument
    // -- and it is §7's capability banner: the range the driver reports, not the
    // one on the datasheet.
    long long tune_min_hz = 0;
    long long tune_max_hz = 0;

    size_t n_points = 0;
    double rbw_hz = 0;
    double bin_hz = 0;

    // The FFT behind rbw_hz, and whether the daemon is choosing it from the
    // span. n_fft 0 means the source did not say, which hides the RBW keys.
    size_t n_fft = 0;
    bool rbw_auto = false;
    // Non-zero when the point count asked for was more than the FFT has bins
    // for in this span, and n_points is what it was capped to.
    size_t n_points_capped_from = 0;
    uint32_t n_segments = 0;
    float gain_db = 0;
    // What set_gain() accepts, from the daemon's config. Both 0 means the
    // source has no settable RX gain, which hides the control.
    double gain_min_db = 0, gain_max_db = 0;
    // The ADC clock behind the sample rate, as the daemon read it back; 0 when
    // the source did not say. Only compared against a correction file's.
    double adc_hz = 0;
    // The RX gain table under that gain: "stock" for the driver's own per-band
    // tables, else the name a custom one was loaded under. Empty from a source
    // that has no AD936x gain table, which hides the control.
    std::string gain_table;

    // ------------------------------------------------------- the segment plan
    //
    // §7's segment boundary overlay reconstructs the LO grid from these. They
    // describe how the sweep was *assembled*, which is a different question from
    // what it covers, and they are the only thing that turns the +4.5 dB splice
    // comb from 298 narrow signals into one named artifact. A source that does
    // not sweep in segments leaves them zero and the overlay stays unavailable
    // rather than drawing an invented grid. See segments.h.
    double seg_first_lo_hz = 0;   // grid origin, before the tuner clamp
    double seg_step_hz = 0;       // LO spacing
    double keep_hz = 0;           // usable width per segment
    double blank_hz = 0;          // HALF-width of the blanked hole, as PlanConfig has it
    double covered_lo_hz = 0;     // union of the kept bands, for checking the above
    double covered_hi_hz = 0;

    // §7's image check. Non-zero means this is the *shifted* pass: the LO grid
    // is deliberately phased, so every response that came from a mirror image
    // has moved and every real signal has not. Reported so the UI can label the
    // trace it is looking at rather than inferring the state from what it last
    // asked for -- which would be wrong for the frames still in flight when the
    // request was made. See imagecheck.h.
    double lo_shift_hz = 0;
    bool splice = true;           // dc_policy: splice covers the holes, blank leaves them
    sweepd::Window window = sweepd::Window::Hann;
    sweepd::Detector detector = sweepd::Detector::Peak;

    // ------------------------------------------------------------ zero span
    //
    // §6's last feature. A zero-span trace arrives as an ordinary 0x02 frame
    // whose x-axis is time (proto.h, "one trace format"), so nothing above this
    // interface needs a second frame path -- but the *labels* have to change,
    // and these are what the daemon reports so the UI never has to guess.

    // The axis the source is currently producing. Frequency while sweeping.
    sweepd::proto::XKind x_kind = sweepd::proto::XKind::Frequency;

    // Per-point interval, and the honest resolution behind it. These are
    // different numbers whenever the overlap is non-zero: dt is how often a
    // point is produced, time_res_s is how long each one integrates over, and
    // the overlap moves only the first. A status bar showing dt alone would
    // imply a resolution the trace does not have.
    uint64_t dt_ns = 0;
    double time_res_s = 0;
    double overlap = 0;

    // The parked centre, and how far below it the LO actually sits. The offset
    // is reported rather than assumed because the LO is *not* at the frequency
    // the user typed -- it is held clear of it so its own leakage stays out of
    // the measurement band (zspan.h, "the LO offset").
    long long center_hz = 0;
    long long lo_hz = 0;
    long long lo_offset_hz = 0;
    long long meas_bw_hz = 0;

    // Two conditions the daemon flags rather than refuses, and which the UI has
    // to surface for the same reason §7 exists: the reading is still a number,
    // it just is not the number the user thinks it is.
    bool band_includes_dc = false;   // LO leakage is inside the measurement band
    bool band_exceeds_bw = false;    // the band reaches onto the filter's skirt

    // §6's video trigger. `trigger` is the daemon's own name for the mode --
    // free_run, video_rising, video_falling -- and the rest describe where the
    // trace sits relative to the edge.
    //
    // The frame carries t0 as well, and the *axis* is built from the frame, not
    // from here: the two are briefly out of step across a reconfiguration and an
    // axis assembled from config would be confident and wrong for those frames.
    // These are what the soft keys show, which is a different job.
    std::string trigger = "free_run";
    double trig_level_db = -40.0;
    double trig_hyst_db = 0.0;
    size_t trig_pretrig_points = 0;
    size_t trig_search_points = 0;

    // How long each acquisition is armed for, as against the frame it shows.
    // Under a trigger the first is larger, and the gap between them is why a
    // rare event can be missed even when the level is right.
    double capture_span_s = 0;

    // ------------------------------------------------- tracking generator
    //
    // The SNA (sweepd/track.h). The trace is a frequency trace, so nothing
    // above needs to change shape; what changes is what a level *means* -- the
    // received level of a stimulus tone, which turns into |S21| once a thru
    // has been stored and subtracted -- and these are what the soft keys and
    // the normalization need to say so.
    bool tracking = false;
    double tsa_atten_db = 0;       // TX attenuation, as the driver read it back
    double tsa_spacing_hz = 0;     // point spacing after snapping to FFT bins
    size_t tsa_tones = 0;          // tones per LO step in use
    size_t tsa_tones_req = 0;      // as asked for; 0 = fill the step
    size_t tsa_coherent = 0;       // blocks averaged coherently per step
    double tsa_keep_hz = 0;        // width measured per LO step
    double tsa_tone_dbfs = 0;      // each tone at the DAC, re full-scale CW
    double tsa_min_atten_db = 30;  // the lowest attenuation the source will accept
    int tsa_harmonic = 1;          // RX at harmonic x stimulus (+ offset)
    double tsa_rx_offset_hz = 0;
    bool tsa_rx_invert = false;
    bool tsa_stim_off = false;     // stimulus silenced: the isolation floor

    // ------------------------------------------------- signal generator
    //
    // The TX as a bench source, running alongside whatever the receiver is
    // doing (sweepd/gen.h). `gen` is the source's own report of what it is
    // transmitting -- the soft keys show this, not what was last asked for.
    sweepd::GenConfig gen;
    double gen_rate_achieved_hz = 0;
    size_t gen_sweep_points = 0;

    // ------------------------------------------------------------ raw IQ
    //
    // The time sink (proto.h, "0x03 iq"). While iq_mode is set the source
    // sends 0x03 bursts on request and no 0x02 traces at all; x_kind stays
    // Frequency, so nothing that reads traces mistakes a burst for one.
    // Everything here is the source's own report of what it is doing -- the
    // soft keys show this, not what was last asked for.
    bool iq_mode = false;
    long long iq_center_hz = 0;
    double iq_adc_fs_hz = 0;
    double iq_fs_hz = 0;               // after decimation
    unsigned iq_decim = 1;
    size_t iq_taps = 1;                // decimator length: its group delay and cost
    size_t iq_n_samples = 0;
    size_t iq_max_input = 0;           // the device buffer, in input samples
    double iq_rf_bw_hz = 0;
    sweepd::IqTrigger iq_trig;
    double iq_auto_s = 0.1;            // Auto gives up on an edge after this long

    // The largest burst the current decimation allows, in OUTPUT samples.
    size_t iq_max_samples() const {
        const sweepd::IqDecimator d(iq_decim);
        return d.output_for(iq_max_input);
    }

    bool zero_span() const { return x_kind == sweepd::proto::XKind::Time; }
    bool triggered() const { return zero_span() && trigger != "free_run"; }

    // §7: units stay dBFS until an amplitude correction file is loaded. The
    // daemon sends this and it is true until §7 lands.
    bool uncal = true;

    // False only for a source fed by a radio. Drives the SYNTHETIC badge.
    bool synthetic = true;

    // §7's capability banner: the measured range, from the source, not from a
    // datasheet.
    std::string description;

    // True while the source is rebuilding a profile table and producing no
    // frames. Distinguishable from a dead link, which is the point: a full-span
    // retune is ~1.3 s of no data and a UI that cannot tell that from a crash
    // shows a frozen trace and no reason for it.
    bool reconfiguring = false;

    // The tuner's reachable range, defaulted to the current span when the source
    // does not report one, so callers never have to special-case it.
    long long tune_lo() const { return tune_min_hz > 0 ? tune_min_hz : f_start_hz; }
    long long tune_hi() const { return tune_max_hz > tune_lo() ? tune_max_hz : f_stop_hz; }
};

// The 0x01 `status` message, plus connection state.
struct SourceStatus {
    bool connected = false;
    double sweeps_per_s = 0;
    double hops_per_s = 0;
    double last_sweep_ms = 0;
    uint64_t frames = 0;      // frames this client has decoded
    uint64_t missed = 0;      // sweeps the daemon dropped for us (seq gaps)
    std::string detail;       // one line for the status bar; may be empty

    // Zero span's probability-of-intercept number: the fraction of wall-clock
    // time the source is actually looking at the signal. The transform is
    // slower than the radio, so frames are gap-free inside themselves and there
    // is a gap between them; §7 requires that trade be visible rather than
    // implied. 0 while sweeping, where revisit_ms carries the same idea.
    double duty = 0;

    // The video trigger's own accounting. `acquisitions` exceeds `frames` by
    // the ones that found no edge; `armed_duty` is the fraction of wall clock
    // the trigger was looking at all. Together they distinguish "the signal is
    // not there" from "we are only watching 12% of the time", which the frame
    // rate alone cannot -- both read as an empty screen.
    uint64_t acquisitions = 0;
    uint64_t no_trigger = 0;
    double armed_duty = 0;

    // True while a trigger is armed and has not fired since the last frame.
    // Drives the WAIT TRIG badge: without it a trigger that never fires looks
    // exactly like a dead link.
    bool waiting_for_trigger = false;
};

class FrameSource {
public:
    virtual ~FrameSource() = default;

    virtual const char *name() const = 0;

    // Non-blocking. Moves the oldest queued frame into *out and returns true,
    // or returns false if nothing is queued. Callers drain in a loop.
    //
    // Oldest rather than newest, and a short queue rather than a single slot,
    // because the waterfall wants every sweep: a source running faster than the
    // display would otherwise lose rows, and a waterfall with sweeps missing
    // silently misrepresents how often a signal was there. The queue is bounded
    // and the *producer* drops when it overflows, which is the same contract
    // the daemon's server has with a slow client (server.h): lossy by design,
    // never silently lossy. SourceStatus::missed counts it.
    virtual bool poll(sweepd::proto::TraceMsg *out) = 0;

    virtual SourceConfig config() const = 0;
    virtual SourceStatus status() const = 0;

    // Failures nobody asked about: a retune the source accepted and then could
    // not carry out. They arrive after the setter has already returned true, so
    // without this they have nowhere to go but a status line the next config
    // message overwrites -- and the user is left with a view the radio is not
    // sweeping and no reason why. The App drains these into its notes.
    virtual bool take_error(std::string *msg) {
        (void)msg;
        return false;
    }

    // Requests. False plus a reason in *err means the request was refused --
    // see the header note on refusals.
    virtual bool set_span(double f_start_hz, double f_stop_hz, std::string *err) {
        (void)f_start_hz;
        (void)f_stop_hz;
        *err = "this source cannot change the span while running";
        return false;
    }
    virtual bool set_points(size_t n_points, std::string *err) {
        (void)n_points;
        *err = "this source cannot change the point count while running";
        return false;
    }
    // 0 couples the RBW to the span; anything else is snapped to an
    // achievable one by the source, which reports what it got in the config.
    virtual bool set_rbw(double rbw_hz, std::string *err) {
        (void)rbw_hz;
        *err = "this source cannot change the RBW while running";
        return false;
    }
    virtual bool set_detector(sweepd::Detector d, std::string *err) {
        (void)d;
        *err = "this source cannot change the detector while running";
        return false;
    }
    virtual bool set_window(sweepd::Window w, std::string *err) {
        (void)w;
        *err = "this source cannot change the window while running";
        return false;
    }

    // The segment width, and with it the sweep rate. `keep_hz` <= 0 means "leave
    // the width alone" and an empty `dc_policy` means "leave the policy alone",
    // so one soft key sends one field and a refusal names the knob that was
    // wrong. `dc_policy` is the daemon's own vocabulary -- "splice" or "blank" --
    // for the reason set_trigger passes a string: the UI must not be able to
    // offer a policy the daemon has never heard of.
    //
    // Costs a profile rebuild, because every segment's LO moved.
    virtual bool set_segmentation(double keep_hz, const std::string &dc_policy,
                                  std::string *err) {
        (void)keep_hz;
        (void)dc_policy;
        *err = "this source has no segments, so there is no width to set";
        return false;
    }

    // A custom RX gain table (src/sweepd/gaintable.h), or the driver's own back
    // when `text` is empty. `text` is the file's contents -- the daemon does not
    // read the host's files -- and `name` is what the config will call it.
    // Costs a profile rebuild: the table is in the cache key.
    // The manual RX gain, whole dB. Applied between sweeps, never AGC (§5).
    // Costs a sweep restart and no profile rebuild.
    virtual bool set_gain(double gain_db, std::string *err) {
        (void)gain_db;
        *err = "this source has no RX gain to set";
        return false;
    }

    virtual bool set_gain_table(const std::string &text, const std::string &name,
                                std::string *err) {
        (void)text;
        (void)name;
        *err = "this source has no AD936x gain table to replace";
        return false;
    }

    // §6's zero span. `center_hz` <= 0 means "keep the current centre", which is
    // what the soft key sends when it is only changing the FFT size or the
    // overlap. n_fft / n_points / overlap of 0 mean the same: leave them alone.
    // One request per key press, so the daemon's refusal names the one knob
    // that was wrong rather than a whole configuration.
    virtual bool set_zero_span(double center_hz, size_t n_fft, size_t n_points,
                               double overlap, std::string *err) {
        (void)center_hz;
        (void)n_fft;
        (void)n_points;
        (void)overlap;
        *err = "this source has no zero span";
        return false;
    }

    // §6's video trigger. `mode` is the daemon's vocabulary -- free_run,
    // video_rising, video_falling -- passed through as a string rather than as
    // an enum here, so the UI cannot offer a mode the daemon has never heard
    // of and then have to guess what it did with it. A NaN in level, hysteresis
    // or pretrig means "leave that one alone", so one soft key sends one field
    // and the refusal names the knob that was wrong.
    virtual bool set_trigger(const std::string &mode, double level_db, double hyst_db,
                             double pretrig, std::string *err) {
        (void)mode;
        (void)level_db;
        (void)hyst_db;
        (void)pretrig;
        *err = "this source has no trigger";
        return false;
    }

    // §7's image check. Phases the LO grid by `hz`, which is 0 to go back to the
    // ordinary grid. The comparison itself is the UI's (imagecheck.h); this only
    // asks for the second pass. Costs a profile rebuild on a real daemon,
    // because every segment's LO moved.
    virtual bool set_lo_shift(double hz, std::string *err) {
        (void)hz;
        *err = "this source has no LO grid to shift, so it cannot run an image check";
        return false;
    }

    // The tracking generator: enters the mode and sets its knobs. NaN in a
    // double and a negative count both mean "leave that one alone", so one
    // soft key sends one field and the refusal names the knob that was wrong.
    // Leaving is set_sweep_mode, as for zero span.
    virtual bool set_tracking(const TrackRequest &r, std::string *err) {
        (void)r;
        *err = "this source has no transmitter, so it cannot be a tracking generator";
        return false;
    }
    bool set_tracking(double atten_db, double spacing_hz, int tones, int coherent,
                      std::string *err) {
        TrackRequest r;
        r.atten_db = atten_db;
        r.spacing_hz = spacing_hz;
        r.tones = tones;
        r.coherent = coherent;
        return set_tracking(r, err);
    }
    // The signal generator. The whole setting at once: a source applies it as
    // one act, so a sweep cannot run briefly with a new frequency and the old
    // attenuation. `on` false turns the transmitter off.
    virtual bool set_generator(const sweepd::GenConfig &g, std::string *err) {
        (void)g;
        *err = "this source has no transmitter";
        return false;
    }
    virtual bool can_generate() const { return false; }

    // Whether to offer the tracking soft keys at all. A source that answers
    // false would only ever refuse them.
    virtual bool can_track() const { return false; }

    // A fabricated source's stand-in for "what is connected between TX and
    // RX". Only the synthetic source has one; the UI shows the selector when
    // the list is non-empty, so a real radio never offers a DUT it cannot
    // change. Index 0 is always the thru.
    virtual std::vector<std::string> sim_duts() const { return {}; }
    virtual int sim_dut() const { return 0; }
    virtual void set_sim_dut(int i) { (void)i; }

    // ------------------------------------------------------------ raw IQ
    //
    // set_iq enters the mode (or adjusts it). iq_arm asks for exactly one
    // burst; the source answers it with one 0x03 message, or with nothing if a
    // Normal trigger never fires -- in which case the request stays pending
    // until it does, or until set_iq or set_sweep_mode cancels it. A second arm
    // while one is pending is not queued: there is only ever one outstanding
    // burst, which is what keeps the device from building a backlog the link
    // cannot drain (proto.h). Leaving the mode is set_sweep_mode.
    virtual bool set_iq(const IqRequest &r, std::string *err) {
        (void)r;
        *err = "this source has no raw IQ capture";
        return false;
    }
    virtual bool iq_arm(std::string *err) {
        *err = "this source has no raw IQ capture";
        return false;
    }
    // Drops a pending arm. Not an error when none is pending.
    virtual void iq_disarm() {}
    virtual bool poll_iq(sweepd::proto::IqMsg *out) {
        (void)out;
        return false;
    }
    virtual bool can_iq() const { return false; }

    // Back to sweeping. Separate from set_zero_span because leaving is not a
    // configuration change -- there is nothing to configure about it -- and
    // because the two are different keys on the front panel.
    virtual bool set_sweep_mode(std::string *err) {
        *err = "this source has no zero span to leave";
        return false;
    }
};

}  // namespace sweepui
