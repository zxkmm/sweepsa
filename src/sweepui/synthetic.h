// The §6 synthetic source: a fake radio environment, so the whole UI can be
// built and judged with no hardware attached.
//
// The point is not to look plausible in a screenshot. It is to exercise every
// path the real source will: NaN holes from dc_policy=blank, per-segment
// overload flags, sequence gaps, a detector that visibly changes the trace, and
// signals that are only visible in the waterfall because they are not there
// every sweep. A generator that emits a smooth curve would let the UI pass its
// own tests and then fall over on the first real frame.
//
// It runs on its own thread and is polled, exactly like the TCP source in §10.4,
// so the UI's frame handling is written against real asynchrony from day one.
//
// ------------------------------------------------------------- fabrication
//
// Every number this produces is invented. The sweep rate is a knob, not a
// measurement, and it is *not* derived from the segment count and the measured
// fastlock_load cost -- deriving it would produce a number that looks like the
// documented 4.3 sweeps/s and is not. SourceConfig::synthetic stays true and the
// UI wears a badge for it.

#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "source.h"
#include "trackplan.h"

namespace sweepui {

struct SyntheticOptions {
    long long f_start_hz = 70000000LL;
    long long f_stop_hz = 6000000000LL;
    size_t n_points = 8192;
    double sweeps_per_s = 20.0;
    uint32_t seed = 1;

    // Mirrors the daemon's dc_policy. `blank` leaves a marked hole per segment;
    // NaN survives the wire and §7 wants it visible, so the UI must handle it.
    bool blank_holes = true;
    uint32_t n_segments = 149;
    double blank_hz = 200e3;

    // §7's image check, and the reason the fake has to know about it: the check
    // is a two-pass measurement driven from the UI, so without a generator whose
    // images move when the grid moves, the state machine, the verdicts and the
    // overlay could only ever be developed against the radio -- which is exactly
    // what this file exists to avoid.
    //
    // `lo_shift_hz` phases the segment centres, the way sweepd's
    // PlanConfig::lo_shift_hz phases the LO grid. `image_rej_db` is how far
    // below its parent each mirror image lands; set it high enough and there is
    // nothing for the check to find, which is the result the real hardware is
    // expected to give (image rejection measured at 53-59 dB, at the noise
    // floor).
    long long lo_shift_hz = 0;
    double image_rej_db = 42.0;

    double noise_floor_dbfs = -98.0;
    sweepd::Detector detector = sweepd::Detector::Peak;
    sweepd::Window window = sweepd::Window::Hann;

    // Bins per output point in the internal spectrum. The generator builds a
    // finer grid and reduces it with sweepd::detect(), which is what makes
    // Peak / Sample / RMS / Negative Peak differ the way they will on hardware
    // instead of all producing the same curve.
    size_t oversample = 8;

    // ------------------------------------------------------------ zero span
    //
    // The same fake environment seen through a parked LO. It exists for the
    // same reason the swept generator does: §6 requires the UI to be buildable
    // and judgeable with no hardware, and zero span is the mode whose display
    // is *least* like the others, so developing it against the daemon only
    // would mean developing it against a device that has to be plugged in.
    //
    // What it has to exercise, and a smooth curve would not: bursts with edges
    // that land between points, a floor that fluctuates the way noise power
    // actually does, and the fact that the trace is blind between frames.
    bool zero_span = false;
    long long center_hz = 1000000000LL;
    long long lo_offset_hz = 15360000LL;
    long long meas_bw_hz = 7680000LL;
    size_t zs_n_fft = 1024;
    size_t zs_points = 512;
    double zs_overlap = 0.0;
    double fs_hz = 61440000.0;

    // §6's video trigger, reproduced here for the same reason the rest of this
    // file exists: the UI has to be judgeable with no hardware, and the trigger
    // is the feature whose *whole* visible effect is that a burst stops moving
    // between frames. That is not something a still screenshot of free run can
    // show, so the fake has to do it too.
    //
    // It runs the same dsp.h VideoTrigger the daemon does, over the same
    // quantity (the reduced band power, before the dB conversion), so what is
    // being exercised is the real rule and not a second reading of it.
    std::string zs_trigger = "free_run";
    double zs_trig_level_db = -40.0;
    double zs_trig_hyst_db = 0.0;
    double zs_trig_pretrig = 0.1;
    size_t zs_trig_search = 0;      // 0 = one frame's worth

    // ------------------------------------------------- tracking generator
    //
    // The SNA, fabricated on the daemon's own grid (sweepd/trackplan.h) so the
    // point placement, the step count and the sweep time come out as the radio
    // will give them. The path between TX and RX is modelled on what the E310
    // measured through a thru cable (tools/tsa_probe, 2026-10-21): about
    // -31 dBFS at 100 MHz at 30 dB attenuation, falling towards 6 GHz, with a
    // step where the LO crosses the front end's 3 GHz band switch -- the one
    // thing a stored thru has to remove and the UI therefore has to show. A
    // simulated DUT sits on top; index 0 is the thru.
    bool tracking = false;
    sweepd::TrackConfig track;
    int dut = 0;

    // The generator, faked as emitters through the same fabricated path as
    // tracking, so its level responds to the attenuation the way the radio's
    // does.
    sweepd::GenConfig gen;

    // ------------------------------------------------------------ raw IQ
    //
    // The same environment as complex baseband around a parked LO, at the
    // fabricated ADC rate (fs_hz). Rendered as waveforms, not powers: tones
    // as tones, channels as band-limited noise, bursty emitters gated on and
    // off -- and then put through the impairments a direct-conversion receiver
    // adds, because the time sink's readouts exist to show them: LO leakage at
    // DC, an I/Q gain and phase mismatch, 12-bit quantisation and a hard clip
    // at full scale. Decimated and triggered by sweepd/iq.h, the code the
    // daemon runs, so the display is developed against the real rule.
    //
    // The default centre puts the 433.92 MHz carrier 25 kHz above the LO, so
    // entering the mode with nothing else set shows a sine on I and Q.
    bool iq_mode = false;
    sweepd::IqConfig iq{433895000LL, 1, 16384, sweepd::IqTrigger{}, 0.1};
    size_t iq_max_input = size_t(1) << 22;   // 16 MB of sc16: the E310 buffer cap
    double rf_bw_hz = 56e6;
    double iq_noise_dbfs = -66.0;            // total over the ADC bandwidth
    double iq_dc_i = 0.0045, iq_dc_q = -0.0030;
    double iq_gain_imb_db = 0.35, iq_phase_imb_deg = 1.8;
    // Transfer time is simulated, at the link's sustained rate (§2), so the
    // burst rate and duty on screen behave as they will over Ethernet.
    double iq_link_mb_s = 50.0;
};

class SyntheticSource : public FrameSource {
public:
    explicit SyntheticSource(const SyntheticOptions &opt);
    ~SyntheticSource() override;

    const char *name() const override { return "synthetic"; }
    bool poll(sweepd::proto::TraceMsg *out) override;
    SourceConfig config() const override;
    SourceStatus status() const override;

    // The synthetic source *can* reconfigure -- it has no BBPLL to reprogram
    // and no profile table to rebuild. It says yes where sweepd says no, which
    // is a difference the UI must not be built to depend on: §10.4 will hand it
    // a source that refuses, and the controls have to survive that.
    bool set_span(double f_start_hz, double f_stop_hz, std::string *err) override;
    bool set_points(size_t n_points, std::string *err) override;
    bool set_segmentation(double keep_hz, const std::string &dc_policy,
                          std::string *err) override;
    bool set_detector(sweepd::Detector d, std::string *err) override;
    bool set_window(sweepd::Window w, std::string *err) override;
    bool set_zero_span(double center_hz, size_t n_fft, size_t n_points, double overlap,
                       std::string *err) override;
    bool set_trigger(const std::string &mode, double level_db, double hyst_db,
                     double pretrig, std::string *err) override;
    bool set_lo_shift(double hz, std::string *err) override;
    bool set_sweep_mode(std::string *err) override;
    bool set_iq(const IqRequest &r, std::string *err) override;
    bool iq_arm(std::string *err) override;
    void iq_disarm() override;
    bool poll_iq(sweepd::proto::IqMsg *out) override;
    bool can_iq() const override { return true; }
    using FrameSource::set_tracking;
    bool set_tracking(const TrackRequest &r, std::string *err) override;
    bool can_track() const override { return true; }
    bool set_generator(const sweepd::GenConfig &g, std::string *err) override;
    bool can_generate() const override { return true; }
    std::vector<std::string> sim_duts() const override;
    int sim_dut() const override;
    void set_sim_dut(int i) override;

private:
    void build_track_frame(sweepd::proto::TraceMsg *out);
    // The daemon's derivation over the fake's span, with the radio limits the
    // fake pretends to have. False only if the knobs do not resolve, which the
    // setter has already refused.
    bool track_derive(const SyntheticOptions &o, sweepd::TrackDerived *d,
                      std::string *err) const;

    struct Emitter;

    void run();                                  // generator thread
    void build_frame(sweepd::proto::TraceMsg *out);
    // False when a trigger was armed and found no edge in this acquisition --
    // which produces no frame at all, exactly as the daemon does. Falling back
    // to free run would draw an untriggered acquisition under a triggered label.
    bool build_zspan_frame(sweepd::proto::TraceMsg *out);
    void plan_emitters();

    // One armed request, worked to completion: acquisitions until the trigger
    // is satisfied (or Auto gives up), then the burst. False if it was
    // cancelled -- a new configuration, a disarm, the mode left -- in which
    // case nothing is sent: a burst taken under settings that are no longer
    // the ones on screen would be labelled with the wrong ones.
    bool build_iq_burst(sweepd::proto::IqMsg *out);
    // One acquisition at the ADC rate, into iq_in_ (interleaved FS floats).
    // Returns the clipped-sample count.
    size_t synth_iq(const SyntheticOptions &o, const std::vector<Emitter> &ems,
                    double t_start, size_t n_in);

    // Derived from the zero-span options the same way derive_zspan() derives
    // them in the daemon, so the two agree about what dt is.
    double zs_dt_s() const;
    double zs_time_res_s() const;

    SyntheticOptions opt_;                       // guarded by mu_
    std::vector<Emitter> emitters_;              // guarded by mu_
    mutable std::mutex mu_;

    std::atomic<bool> stop_{false};
    std::thread thread_;

    // A short bounded queue, oldest out. See FrameSource::poll for why it is
    // not a single slot. Sixteen frames is about a second of history at the
    // default rate -- enough to ride out a stalled UI frame, short enough that
    // what the display shows is never far behind the source.
    static constexpr size_t kQueueDepth = 16;
    mutable std::mutex box_mu_;
    std::deque<sweepd::proto::TraceMsg> queue_;
    uint64_t seq_ = 0;
    uint64_t dropped_ = 0;
    uint64_t delivered_ = 0;

    std::vector<float> power_;                   // generator scratch, oversampled
    std::vector<float> reduced_;
    // Trigger accounting, mirroring ZeroSpanStats. Guarded by box_mu_.
    uint64_t acquisitions_ = 0;
    uint64_t no_trigger_ = 0;
    uint64_t no_trigger_at_frame_ = 0;
    std::mt19937 rng_;
    double t0_ = 0;

    // Raw IQ. iq_gen_ moves on every set_iq / mode change; a burst in progress
    // that sees it move is abandoned. One outstanding arm, never a queue.
    std::atomic<bool> iq_pending_{false};
    std::atomic<uint64_t> iq_gen_{0};
    std::deque<sweepd::proto::IqMsg> iq_queue_;   // guarded by box_mu_
    uint64_t iq_seq_ = 0;
    uint64_t iq_bursts_ = 0;                      // guarded by box_mu_
    uint64_t iq_acqs_ = 0;                        // guarded by box_mu_
    std::vector<float> iq_in_, iq_dec_;
};

}  // namespace sweepui
