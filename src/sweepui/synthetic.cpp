#include "synthetic.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

#include <time.h>

namespace sweepui {
namespace {

// A tracking step costs what it measured on the E310: two real LO writes, one
// refill and one transform, 14.4 ms in all (sweepd status line, 2026-10-21).
constexpr double kTrackStepS = 0.0144;

double mono_now() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return double(ts.tv_sec) + double(ts.tv_nsec) * 1e-9;
}

double db_to_power(double db) { return std::pow(10.0, db / 10.0); }

}  // namespace

// A single signal in the fake environment.
//
// Everything is expressed in power, never in dB, for the same reason §5 makes
// the RMS detector average power: two emitters sharing a bin add in power, and
// adding their dB values would be a different and wrong operation. The generator
// would look right and the arithmetic would be a lie the UI is supposed to be
// good at not telling.
struct SyntheticSource::Emitter {
    enum class Shape {
        Cw,          // a tone: one bin wide before the window spreads it
        Channel,     // a flat-topped occupied channel with skirts
        Noise,       // a broad rise in the floor, e.g. an interferer
    };

    double f_hz = 0;
    double bw_hz = 0;
    double power_db = -60;      // dBFS at the peak
    Shape shape = Shape::Cw;

    // Time behaviour. All three exist to give the waterfall something to show
    // that the trace alone cannot.
    double duty = 1.0;          // fraction of sweeps it is present
    double drift_hz_per_s = 0;  // slow frequency drift
    double hop_hz = 0;          // random re-tune within +/- hop_hz each sweep
    double hop_period_s = 0;

    // Runtime state.
    double f_now_hz = 0;
    bool on = true;
};

SyntheticSource::SyntheticSource(const SyntheticOptions &opt)
    : opt_(opt), rng_(opt.seed) {
    if (opt_.n_points < 16) opt_.n_points = 16;
    if (opt_.oversample < 1) opt_.oversample = 1;
    if (opt_.f_stop_hz <= opt_.f_start_hz) opt_.f_stop_hz = opt_.f_start_hz + 1000000LL;
    if (opt_.n_segments < 1) opt_.n_segments = 1;
    t0_ = mono_now();
    plan_emitters();
    thread_ = std::thread([this] { run(); });
}

SyntheticSource::~SyntheticSource() {
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
}

// A band plan chosen so that every part of the UI has something to display:
// crowded regions and empty ones, signals narrower than a display pixel at full
// span, one wide enough to survive any decimation, intermittent ones that only
// the waterfall reveals, and one strong enough to overload.
void SyntheticSource::plan_emitters() {
    emitters_.clear();
    auto add = [&](double f, double bw, double db, Emitter::Shape sh) -> Emitter & {
        Emitter e;
        e.f_hz = f;
        e.f_now_hz = f;
        e.bw_hz = bw;
        e.power_db = db;
        e.shape = sh;
        emitters_.push_back(e);
        return emitters_.back();
    };

    // FM broadcast: a dense comb of 180 kHz channels. At full span these are
    // far narrower than a pixel, which is precisely what min/max decimation is
    // for -- sampling would make them flicker as the window is resized.
    for (int i = 0; i < 12; ++i) {
        const double f = 88.1e6 + i * 1.7e6;
        add(f, 180e3, -42 - (i % 5) * 4.0, Emitter::Shape::Channel);
    }

    // A pair of unmodulated carriers, one of them drifting: the classic thing a
    // marker is pointed at, and a reason for the waterfall to show a slope.
    add(433.92e6, 0, -55, Emitter::Shape::Cw);
    add(915.0e6, 0, -63, Emitter::Shape::Cw).drift_hz_per_s = 12e3;

    // A frequency-hopping emitter. Present every sweep but somewhere new each
    // time, so the trace shows one spur and the waterfall shows the pattern --
    // which is the §7 probability-of-intercept story made visible.
    {
        Emitter &e = add(2.44e9, 400e3, -58, Emitter::Shape::Channel);
        e.hop_hz = 35e6;
        e.hop_period_s = 0.0;   // every sweep
    }

    // Cellular-ish occupied channels, wide enough to survive full-span
    // decimation, so there is always something with real shape on screen.
    add(942e6, 10e6, -58, Emitter::Shape::Channel);
    add(1.845e9, 18e6, -61, Emitter::Shape::Channel);
    add(2.655e9, 20e6, -66, Emitter::Shape::Channel);
    add(3.55e9, 40e6, -72, Emitter::Shape::Channel);

    // Wi-Fi: bursty. Off most sweeps, so it appears as dashes down the
    // waterfall and is absent from most single traces. Max-hold exists for
    // this, and it cannot be demonstrated against a static scene.
    for (double f : {2.412e9, 2.437e9, 2.462e9}) {
        Emitter &e = add(f, 20e6, -64, Emitter::Shape::Channel);
        e.duty = 0.35;
    }
    for (double f : {5.18e9, 5.5e9, 5.745e9}) {
        Emitter &e = add(f, 40e6, -70, Emitter::Shape::Channel);
        e.duty = 0.2;
    }

    // A broadband interferer: raises the floor over a whole region rather than
    // adding a line. Distinguishing this from the real floor is a thing the
    // display has to make possible.
    add(1.2e9, 120e6, -86, Emitter::Shape::Noise);

    // One emitter strong enough to drive the front end into overload. §5 makes
    // the daemon flag the affected segments and §6 puts an indicator in the
    // status bar; neither can be checked against a scene that never overloads.
    {
        Emitter &e = add(1.5752e9, 0, -3, Emitter::Shape::Cw);
        e.duty = 0.25;
    }
}

SourceConfig SyntheticSource::config() const {
    std::lock_guard<std::mutex> lk(mu_);
    SourceConfig c;
    c.f_start_hz = opt_.f_start_hz;
    c.f_stop_hz = opt_.f_stop_hz;
    // The AD936x's published range, so "full span" against the fake behaves the
    // same way it does against the radio. It is a made-up number like everything
    // else here, and the SYNTHETIC badge is what says so.
    c.tune_min_hz = 70000000LL;
    c.tune_max_hz = 6000000000LL;
    c.n_points = opt_.n_points;
    c.bin_hz = double(opt_.f_stop_hz - opt_.f_start_hz) / double(opt_.n_points);
    // RBW here is the generator's own bin spacing rather than a derived
    // ENBW * fs / N: there is no fs and no FFT. It is labelled as such so
    // nobody reads the status bar and believes a resolution was achieved.
    c.rbw_hz = c.bin_hz * sweepd::make_window(opt_.window, 64).enbw;
    c.n_segments = opt_.n_segments;
    c.gain_db = 30.0f;

    // The segment plan, so §7's overlay develops against the fake like
    // everything else in §6. This generator tiles the span uniformly and cuts
    // its own hole at each centre, which is exactly dc_policy=blank geometry:
    // centres a full keep apart, first one half a keep in. Reported in the
    // daemon's terms rather than the generator's so the overlay reads one
    // vocabulary -- note blank_hz is a HALF-width there and a full width here.
    c.keep_hz = double(opt_.f_stop_hz - opt_.f_start_hz) / double(opt_.n_segments);
    c.seg_step_hz = c.keep_hz;
    c.seg_first_lo_hz =
        double(opt_.f_start_hz) + c.keep_hz * 0.5 + double(opt_.lo_shift_hz);
    c.blank_hz = opt_.blank_holes ? opt_.blank_hz * 0.5 : 0.0;
    // The covered range follows the grid, not the requested span -- which is
    // what the daemon reports too, and what §7's overlay checks its
    // reconstruction against. A shifted grid that still claimed to cover
    // [f_start, f_stop] would make the overlay refuse to draw.
    c.covered_lo_hz = c.seg_first_lo_hz - c.keep_hz * 0.5;
    c.covered_hi_hz =
        c.seg_first_lo_hz + (double(opt_.n_segments) - 0.5) * c.seg_step_hz;
    c.lo_shift_hz = double(opt_.lo_shift_hz);
    c.splice = false;

    c.window = opt_.window;
    c.detector = opt_.detector;
    c.gen = opt_.gen;
    {
        sweepd::GenDerived gd;
        std::string e;
        if (opt_.gen.on && sweepd::derive_gen(opt_.gen, opt_.fs_hz, 70000000LL, 6000000000LL,
                                              &gd, &e)) {
            c.gen_rate_achieved_hz = gd.rate_hz;
            c.gen_sweep_points = gd.sweep_points;
        }
    }
    c.uncal = true;
    c.synthetic = true;
    c.description = "synthetic source -- no radio attached; every value is fabricated";

    if (opt_.iq_mode) {
        c.iq_mode = true;
        c.iq_center_hz = opt_.iq.center_hz;
        c.iq_adc_fs_hz = opt_.fs_hz;
        c.iq_decim = opt_.iq.decim;
        c.iq_fs_hz = opt_.fs_hz / double(opt_.iq.decim);
        c.iq_taps = sweepd::IqDecimator(opt_.iq.decim).taps();
        c.iq_n_samples = opt_.iq.n_samples;
        c.iq_max_input = opt_.iq_max_input;
        c.iq_rf_bw_hz = opt_.rf_bw_hz;
        c.iq_trig = opt_.iq.trig;
        c.iq_auto_s = opt_.iq.auto_s;
        c.f_start_hz = c.f_stop_hz = opt_.iq.center_hz;
        c.description = "synthetic source, RAW IQ -- no radio attached; the waveforms and "
                        "the receiver impairments are fabricated";
        return c;
    }

    if (opt_.tracking) {
        sweepd::TrackDerived d;
        std::string err;
        if (track_derive(opt_, &d, &err)) {
            c.tracking = true;
            c.f_start_hz = (long long)llround(d.out_lo_hz);
            c.f_stop_hz = (long long)llround(d.out_hi_hz);
            c.n_points = d.n_points;
            c.bin_hz = d.spacing_hz;
            c.rbw_hz = d.rbw_hz;
            c.n_fft = d.n_fft;
            c.rbw_auto = false;
            c.n_segments = uint32_t(d.steps.size());
            // No fastlock grid to draw: §7's overlay stays off, as the daemon
            // arranges by sending a zero step.
            c.seg_step_hz = 0;
            c.seg_first_lo_hz = 0;
            c.keep_hz = double(d.keep_hz);
            c.blank_hz = 0;
            c.covered_lo_hz = d.out_lo_hz;
            c.covered_hi_hz = d.out_hi_hz;
            c.detector = sweepd::Detector::Sample;
            c.tsa_atten_db = opt_.track.atten_db;
            c.tsa_spacing_hz = d.spacing_hz;
            c.tsa_tones = d.tones_per_step;
            c.tsa_tones_req = opt_.track.tones;
            c.tsa_coherent = d.coherent;
            c.tsa_keep_hz = double(d.keep_hz);
            std::vector<int16_t> stim;
            sweepd::track_stimulus(d, opt_.track.scale, &stim, nullptr, &c.tsa_tone_dbfs);
            c.tsa_min_atten_db = sweepd::kTrackLoudAttenDb;
            c.tsa_harmonic = d.harmonic;
            c.tsa_rx_offset_hz = double(d.rx_offset_hz);
            c.tsa_rx_invert = d.rx_invert;
            c.tsa_stim_off = opt_.track.stim_off;
            c.description = "synthetic source, TRACKING -- no radio attached; the path and "
                            "the DUT are fabricated";
        }
        return c;
    }
    if (opt_.zero_span) {
        c.x_kind = sweepd::proto::XKind::Time;
        c.n_points = opt_.zs_points;
        c.n_segments = 1;
        c.bin_hz = 0.0;
        c.dt_ns = (uint64_t)llround(zs_dt_s() * 1e9);
        c.time_res_s = zs_time_res_s();
        c.overlap = opt_.zs_overlap;
        c.center_hz = opt_.center_hz;
        c.lo_offset_hz = opt_.lo_offset_hz;
        c.lo_hz = opt_.center_hz - opt_.lo_offset_hz;
        c.meas_bw_hz = opt_.meas_bw_hz;
        c.band_includes_dc = opt_.lo_offset_hz == 0;
        c.band_exceeds_bw = false;
        c.trigger = opt_.zs_trigger;
        c.trig_level_db = opt_.zs_trig_level_db;
        c.trig_hyst_db = opt_.zs_trig_hyst_db;
        if (c.triggered()) {
            c.trig_pretrig_points =
                std::min(opt_.zs_points - 1,
                         size_t(llround(opt_.zs_trig_pretrig * double(opt_.zs_points))));
            c.trig_search_points =
                opt_.zs_trig_search ? opt_.zs_trig_search : opt_.zs_points;
            c.capture_span_s =
                double(opt_.zs_points + c.trig_search_points) * zs_dt_s();
        } else {
            c.capture_span_s = double(opt_.zs_points) * zs_dt_s();
        }
        // Here it is a real derivation, unlike the swept branch above: the fake
        // has an FFT size and a sample rate in zero span, so RBW is ENBW x fs /
        // n_fft exactly as §5 requires. Still fabricated -- the sample rate is
        // invented -- but arrived at the same way.
        c.rbw_hz = sweepd::rbw_hz(sweepd::make_window(opt_.window, opt_.zs_n_fft).enbw,
                                  opt_.fs_hz, opt_.zs_n_fft);
        c.f_start_hz = opt_.center_hz;
        c.f_stop_hz = opt_.center_hz;
        c.description = "synthetic source, ZERO SPAN -- no radio attached; every value "
                        "is fabricated";
    }
    return c;
}

SourceStatus SyntheticSource::status() const {
    SourceStatus s;
    s.connected = true;
    bool triggered = false;
    double armed = 0;
    {
        std::lock_guard<std::mutex> lk(mu_);
        s.sweeps_per_s = opt_.sweeps_per_s;
        s.hops_per_s = opt_.zero_span ? 0.0
                                      : opt_.sweeps_per_s * double(opt_.n_segments);
        s.last_sweep_ms = 1000.0 / std::max(1e-6, opt_.sweeps_per_s);
        if (opt_.tracking) {
            sweepd::TrackDerived d;
            std::string e;
            if (track_derive(opt_, &d, &e)) {
                const double period = std::max(0.05, double(d.steps.size()) * kTrackStepS);
                s.sweeps_per_s = 1.0 / period;
                s.hops_per_s = double(d.steps.size()) / period;
                s.last_sweep_ms = period * 1e3;
            }
        }
        if (opt_.zero_span) {
            // frames/s x the window each one covers. Fabricated like everything
            // else here, but computed the way the daemon computes it, so the
            // status bar's duty readout is exercised rather than hardcoded.
            s.duty = std::min(1.0, opt_.sweeps_per_s * double(opt_.zs_points) *
                                       zs_dt_s());
            triggered = opt_.zs_trigger != "free_run";
            if (triggered) {
                const size_t search =
                    opt_.zs_trig_search ? opt_.zs_trig_search : opt_.zs_points;
                armed = opt_.sweeps_per_s * double(opt_.zs_points + search) * zs_dt_s();
            }
        }
    }
    {
        std::lock_guard<std::mutex> lk(box_mu_);
        s.frames = delivered_;
        s.missed = dropped_;
        s.acquisitions = acquisitions_;
        s.no_trigger = no_trigger_;
        s.armed_duty = triggered ? std::min(1.0, armed) : 0.0;
        // The same rule NetSource uses: if the miss count has moved on since
        // the last frame we handed out, the trigger has looked and found
        // nothing since. Without it a trigger set too high is indistinguishable
        // from a dead source.
        s.waiting_for_trigger = triggered && no_trigger_ > no_trigger_at_frame_;
    }
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (opt_.iq_mode) {
            s.hops_per_s = 0;
            s.duty = 0;
            std::lock_guard<std::mutex> bl(box_mu_);
            s.frames = iq_bursts_;
            s.acquisitions = iq_acqs_;
            s.waiting_for_trigger =
                iq_pending_.load() && opt_.iq.trig.mode == sweepd::IqTrigMode::Normal &&
                iq_acqs_ > iq_bursts_;
        }
    }
    s.detail = "fabricated";
    return s;
}

bool SyntheticSource::poll(sweepd::proto::TraceMsg *out) {
    std::lock_guard<std::mutex> lk(box_mu_);
    if (queue_.empty()) return false;
    *out = std::move(queue_.front());
    queue_.pop_front();
    ++delivered_;
    return true;
}

bool SyntheticSource::set_span(double f_start_hz, double f_stop_hz, std::string *err) {
    if (!(f_stop_hz > f_start_hz)) {
        *err = "stop must be above start";
        return false;
    }
    std::lock_guard<std::mutex> lk(mu_);
    opt_.f_start_hz = (long long)llround(f_start_hz);
    opt_.f_stop_hz = (long long)llround(f_stop_hz);
    return true;
}

bool SyntheticSource::set_points(size_t n_points, std::string *err) {
    if (n_points < 16 || n_points > 262144) {
        *err = "point count must be between 16 and 262144";
        return false;
    }
    std::lock_guard<std::mutex> lk(mu_);
    opt_.n_points = n_points;
    return true;
}

bool SyntheticSource::set_segmentation(double keep_hz, const std::string &dc_policy,
                                       std::string *err) {
    std::lock_guard<std::mutex> lk(mu_);
    if (opt_.zero_span) {
        *err = "the source is in zero span, which has one parked LO and no segments";
        return false;
    }
    // This generator tiles the span uniformly and cuts its own hole at each
    // centre, which is dc_policy=blank geometry and nothing else (see
    // plan_emitters). Refusing splice is the honest answer: pretending to
    // produce it would develop the UI against overlap this source never has,
    // and the difference would first appear with a radio attached.
    if (!dc_policy.empty() && dc_policy != "blank") {
        *err = "the synthetic source only models dc_policy=blank geometry";
        return false;
    }
    if (keep_hz > 0) {
        const double span = double(opt_.f_stop_hz - opt_.f_start_hz);
        const long long n = (long long)llround(span / keep_hz);
        if (n < 1 || n > 4096) {
            *err = "that segment width gives an unusable segment count for this span";
            return false;
        }
        // n_segments is what this source actually has; keep_hz is reported back
        // as span/n_segments, so a request lands on the nearest width the
        // generator can express rather than on a number it then contradicts.
        opt_.n_segments = size_t(n);
    }
    return true;
}

bool SyntheticSource::set_detector(sweepd::Detector d, std::string *err) {
    (void)err;
    std::lock_guard<std::mutex> lk(mu_);
    opt_.detector = d;
    return true;
}

bool SyntheticSource::set_window(sweepd::Window w, std::string *err) {
    (void)err;
    std::lock_guard<std::mutex> lk(mu_);
    opt_.window = w;
    return true;
}

bool SyntheticSource::set_lo_shift(double hz, std::string *err) {
    std::lock_guard<std::mutex> lk(mu_);
    if (opt_.zero_span) {
        *err = "the source is in zero span, which has one parked LO and no grid to "
               "shift";
        return false;
    }
    // The daemon's bound, reproduced: past one step the grid is a different
    // grid, not a phased one. Reproducing the refusal is the point -- a UI built
    // against a fake that accepted anything would meet this for the first time
    // with a radio plugged in.
    const double step =
        double(opt_.f_stop_hz - opt_.f_start_hz) / double(opt_.n_segments);
    if (std::fabs(hz) >= step) {
        *err = "the LO shift must be smaller than the segment step";
        return false;
    }
    opt_.lo_shift_hz = (long long)std::llround(hz);
    return true;
}

// The refusals here are the daemon's, reproduced deliberately. A synthetic
// source that accepted an overlap of 0.6 would let the UI be built against a
// contract sweepd does not honour, and the refusal would first appear when
// somebody plugged in a radio.
bool SyntheticSource::set_zero_span(double center_hz, size_t n_fft, size_t n_points,
                                    double overlap, std::string *err) {
    std::lock_guard<std::mutex> lk(mu_);
    SyntheticOptions o = opt_;
    o.zero_span = true;
    o.tracking = false;
    o.iq_mode = false;
    if (center_hz > 0) o.center_hz = (long long)llround(center_hz);
    if (n_fft) o.zs_n_fft = n_fft;
    if (n_points) o.zs_points = n_points;
    if (overlap >= 0) o.zs_overlap = overlap;

    if (o.zs_n_fft < 32 || (o.zs_n_fft & (o.zs_n_fft - 1)) != 0) {
        *err = "n_fft must be a power of two and at least 32";
        return false;
    }
    if (o.zs_points < 2) {
        *err = "zero span needs at least 2 points to be a trace over time";
        return false;
    }
    if (o.zs_overlap != 0.0 && o.zs_overlap != 0.5 && o.zs_overlap != 0.75) {
        *err = "overlap must be 0, 0.5 or 0.75; any other fraction makes the hop a "
               "non-integer number of samples and the reported dt would drift from the "
               "data across a frame";
        return false;
    }
    if (o.center_hz - o.lo_offset_hz <= 0) {
        *err = "that centre puts the LO at or below 0 Hz";
        return false;
    }
    opt_ = o;
    iq_pending_.store(false);
    ++iq_gen_;
    return true;
}

// The daemon's own vocabulary and the daemon's own refusals, for the reason
// stated above set_zero_span: a fake that says yes to something sweepd refuses
// lets the UI be built against a contract that only fails when a radio is
// plugged in.
bool SyntheticSource::set_trigger(const std::string &mode, double level_db,
                                  double hyst_db, double pretrig, std::string *err) {
    std::lock_guard<std::mutex> lk(mu_);
    SyntheticOptions o = opt_;
    if (!mode.empty()) {
        if (mode != "free_run" && mode != "video_rising" && mode != "video_falling") {
            *err = "unknown trigger '" + mode +
                   "'; expected free_run, video_rising or video_falling";
            return false;
        }
        o.zs_trigger = mode;
    }
    if (level_db == level_db) o.zs_trig_level_db = level_db;
    if (hyst_db == hyst_db) {
        if (hyst_db < 0) {
            *err = "trig_hyst_db is a depth below the trigger level and cannot be "
                   "negative; 0 is a plain crossing";
            return false;
        }
        o.zs_trig_hyst_db = hyst_db;
    }
    if (pretrig == pretrig) {
        if (pretrig < 0.0 || pretrig > 0.9) {
            *err = "trig_pretrig is the fraction of the frame before the edge and must "
                   "be between 0 and 0.9";
            return false;
        }
        o.zs_trig_pretrig = pretrig;
    }
    opt_ = o;
    return true;
}

bool SyntheticSource::set_sweep_mode(std::string *err) {
    (void)err;
    std::lock_guard<std::mutex> lk(mu_);
    opt_.zero_span = false;
    opt_.tracking = false;
    if (opt_.iq_mode) {
        opt_.iq_mode = false;
        iq_pending_.store(false);
        ++iq_gen_;
    }
    return true;
}

// ---------------------------------------------------------------- raw IQ

// The daemon's validation (sweepd::derive_iq), so a request the fake accepts
// is one the radio accepts. A pending arm survives a reconfiguration: the
// burst being built is abandoned and the next one is taken under the new
// settings, which is what a running time sink should do when a knob turns.
bool SyntheticSource::set_iq(const IqRequest &r, std::string *err) {
    std::lock_guard<std::mutex> lk(mu_);
    SyntheticOptions o = opt_;
    o.iq_mode = true;
    o.zero_span = false;
    o.tracking = false;
    if (r.center_hz == r.center_hz) {
        if (r.center_hz < 70e6 || r.center_hz > 6e9) {
            *err = "centre must be within the tuner's 70 MHz - 6 GHz";
            return false;
        }
        o.iq.center_hz = llround(r.center_hz);
    }
    if (r.n_samples >= 0) o.iq.n_samples = size_t(r.n_samples);
    if (r.decim >= 0) o.iq.decim = unsigned(r.decim);
    if (!r.trig_mode.empty() && !sweepd::iq_trig_mode_from_string(r.trig_mode, &o.iq.trig.mode)) {
        *err = "unknown trigger mode '" + r.trig_mode + "'; expected none, normal or auto";
        return false;
    }
    if (!r.trig_source.empty() &&
        !sweepd::iq_trig_source_from_string(r.trig_source, &o.iq.trig.source)) {
        *err = "unknown trigger source '" + r.trig_source + "'; expected i, q or mag";
        return false;
    }
    if (r.trig_rising >= 0) o.iq.trig.rising = r.trig_rising != 0;
    if (r.trig_level == r.trig_level) o.iq.trig.level = r.trig_level;
    if (r.trig_hyst == r.trig_hyst) o.iq.trig.hyst = r.trig_hyst;
    if (r.pretrig == r.pretrig) o.iq.trig.pretrig = r.pretrig;
    if (r.auto_s == r.auto_s) o.iq.auto_s = r.auto_s;
    sweepd::IqGeometry g;
    if (!sweepd::derive_iq(o.iq, o.iq_max_input, &g, err)) return false;
    opt_ = o;
    ++iq_gen_;
    return true;
}

bool SyntheticSource::iq_arm(std::string *err) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!opt_.iq_mode) {
        *err = "not in IQ mode";
        return false;
    }
    iq_pending_.store(true);
    return true;
}

void SyntheticSource::iq_disarm() {
    iq_pending_.store(false);
    ++iq_gen_;
}

bool SyntheticSource::poll_iq(sweepd::proto::IqMsg *out) {
    std::lock_guard<std::mutex> lk(box_mu_);
    if (iq_queue_.empty()) return false;
    *out = std::move(iq_queue_.front());
    iq_queue_.pop_front();
    return true;
}

namespace {

// A complex phasor stepped by multiplication rather than by sin/cos per
// sample, renormalised now and then so rounding cannot grow or shrink it.
struct Phasor {
    double re = 1, im = 0, sr = 1, si = 0;
    Phasor(double phase, double step)
        : re(std::cos(phase)), im(std::sin(phase)), sr(std::cos(step)), si(std::sin(step)) {}
    void next(size_t k) {
        const double r = re * sr - im * si;
        im = re * si + im * sr;
        re = r;
        if ((k & 1023) == 1023) {
            const double m = 1.0 / std::sqrt(re * re + im * im);
            re *= m;
            im *= m;
        }
    }
};

}  // namespace

size_t SyntheticSource::synth_iq(const SyntheticOptions &o, const std::vector<Emitter> &ems,
                                 double t_start, size_t n_in) {
    const double fs = o.fs_hz;
    iq_in_.assign(2 * n_in, 0.0f);
    std::normal_distribution<float> nd(0.0f, 1.0f);

    for (const Emitter &e : ems) {
        const double f = e.f_now_hz > 0 ? e.f_now_hz : e.f_hz;
        const double off = f - double(o.iq.center_hz);
        if (std::fabs(off) - e.bw_hz * 0.5 > fs * 0.5) continue;
        const double amp = std::sqrt(db_to_power(e.power_db));

        // Bursty emitters are gated in time the way build_zspan_frame gates
        // them, so the same emitter has the same rhythm in both views.
        const bool gated = e.duty < 1.0;
        const double k = std::fmod(f * 1e-6, 7.0);
        const double period = 200e-6 + k * 400e-6;
        const double width = period * std::max(0.05, e.duty);
        const double phase_s = std::fmod(f * 1e-9, 1.0) * period;

        const double step = 2.0 * M_PI * off / fs;
        Phasor ph(std::fmod(2.0 * M_PI * off * t_start, 2.0 * M_PI), step);

        const bool tone = e.shape == Emitter::Shape::Cw;
        // A one-pole low-pass on white noise, scaled so the output carries the
        // emitter's power: its variance is sigma^2 * a / (2 - a).
        const double a = std::min(1.0, 1.0 - std::exp(-2.0 * M_PI * std::max(e.bw_hz, 1.0) *
                                                       0.5 / fs));
        const double sig = amp / std::sqrt(2.0) * std::sqrt((2.0 - a) / a);
        double lr = 0, li = 0;
        for (size_t i = 0; i < n_in; ++i) {
            double br, bi;
            if (tone) {
                br = amp * ph.re;
                bi = amp * ph.im;
            } else {
                lr += a * (sig * nd(rng_) - lr);
                li += a * (sig * nd(rng_) - li);
                br = lr * ph.re - li * ph.im;
                bi = lr * ph.im + li * ph.re;
            }
            ph.next(i);
            if (gated) {
                const double t = t_start + double(i) / fs;
                if (std::fmod(t + phase_s, period) >= width) continue;
            }
            iq_in_[2 * i] += float(br);
            iq_in_[2 * i + 1] += float(bi);
        }
    }

    // The receiver: noise, the I/Q mismatch, LO leakage, then the converter.
    const float nsig = float(std::pow(10.0, o.iq_noise_dbfs / 20.0) / std::sqrt(2.0));
    const double g = std::pow(10.0, -o.iq_gain_imb_db / 20.0);
    const double phi = o.iq_phase_imb_deg * M_PI / 180.0;
    const double cphi = std::cos(phi), sphi = std::sin(phi);
    const double lsb = 1.0 / 2048.0;
    size_t clipped = 0;
    for (size_t i = 0; i < n_in; ++i) {
        const double x = double(iq_in_[2 * i]) + double(nsig * nd(rng_));
        const double y = double(iq_in_[2 * i + 1]) + double(nsig * nd(rng_));
        double vi = x + o.iq_dc_i;
        double vq = g * (y * cphi + x * sphi) + o.iq_dc_q;
        bool clip = false;
        auto adc = [&](double v) {
            double q = std::round(v / lsb) * lsb;
            if (q >= 2047.0 * lsb) q = 2047.0 * lsb;
            if (q <= -1.0) q = -1.0;
            if (std::fabs(q) >= 0.98) clip = true;
            return float(q);
        };
        iq_in_[2 * i] = adc(vi);
        iq_in_[2 * i + 1] = adc(vq);
        if (clip) ++clipped;
    }
    return clipped;
}

bool SyntheticSource::build_iq_burst(sweepd::proto::IqMsg *out) {
    SyntheticOptions o;
    std::vector<Emitter> ems;
    const uint64_t gen = iq_gen_.load();
    {
        std::lock_guard<std::mutex> lk(mu_);
        o = opt_;
        ems = emitters_;
    }
    sweepd::IqGeometry g;
    std::string e;
    if (!o.iq_mode || !sweepd::derive_iq(o.iq, o.iq_max_input, &g, &e)) return false;
    const sweepd::IqDecimator dec(o.iq.decim);
    const double fs_out = o.fs_hz / double(o.iq.decim);

    auto cancelled = [&] {
        return stop_.load() || iq_gen_.load() != gen || !iq_pending_.load();
    };
    // Real time passes, so the burst rate and the Auto timeout behave as they
    // will on the radio rather than as fast as this machine can synthesise.
    auto nap = [&](double secs) {
        const double until = mono_now() + secs;
        while (!cancelled()) {
            const double left = until - mono_now();
            if (left <= 0) break;
            timespec ts{};
            ts.tv_nsec = long(std::min(left, 0.01) * 1e9);
            nanosleep(&ts, nullptr);
        }
    };

    const double t_begin = mono_now();
    uint32_t acqs = 0;
    size_t clipped = 0;
    double t_cap = 0;
    int64_t t0 = 0;
    uint32_t ti = 0;
    auto state = sweepd::proto::IqTrigState::Untriggered;
    for (;;) {
        if (cancelled()) return false;
        t_cap = mono_now();
        // Outside mu_: a 4 Mi-sample acquisition takes a while to fabricate,
        // and the UI reads config() every frame.
        clipped = synth_iq(o, ems, t_cap - t0_, g.acq_in);
        ++acqs;
        {
            std::lock_guard<std::mutex> bl(box_mu_);
            ++iq_acqs_;
        }
        dec.run(iq_in_.data(), g.acq_in, &iq_dec_);
        // The capture takes its own length in time, and at least the 1 ms a
        // refill round trip costs on the device.
        nap(std::max(0.001, double(g.acq_in) / o.fs_hz - (mono_now() - t_cap)));
        if (cancelled()) return false;

        const size_t acq_n = iq_dec_.size() / 2;
        if (sweepd::iq_cut(iq_dec_.data(), acq_n, o.iq, g, fs_out, false, &out->iq, &t0, &ti)) {
            state = o.iq.trig.mode == sweepd::IqTrigMode::None
                        ? sweepd::proto::IqTrigState::Untriggered
                        : sweepd::proto::IqTrigState::Triggered;
            break;
        }
        if (o.iq.trig.mode == sweepd::IqTrigMode::Auto && mono_now() - t_begin >= o.iq.auto_s) {
            sweepd::iq_cut(iq_dec_.data(), acq_n, o.iq, g, fs_out, true, &out->iq, &t0, &ti);
            state = sweepd::proto::IqTrigState::AutoTimeout;
            break;
        }
    }

    out->seq = ++iq_seq_;
    out->t_mono_s = t_cap;
    out->center_hz = o.iq.center_hz;
    out->fs_hz = fs_out;
    out->adc_fs_hz = o.fs_hz;
    out->rf_bw_hz = o.rf_bw_hz;
    out->t0_ns = t0;
    out->capture_s = double(g.acq_in) / o.fs_hz;
    out->gain_db = 30.0f;
    out->full_scale = 2048.0f;
    out->decim = o.iq.decim;
    out->n_clipped = uint32_t(clipped);
    out->n_acquisitions = acqs;
    out->format = o.iq.decim == 1 ? sweepd::proto::IqFormat::Sc16 : sweepd::proto::IqFormat::Cf32;
    out->trig_mode = uint8_t(o.iq.trig.mode);
    out->trig_state = state;
    out->trig_source = uint8_t(o.iq.trig.source);
    out->trig_rising = o.iq.trig.rising;
    out->trig_level = float(o.iq.trig.level);
    out->trig_hyst = float(o.iq.trig.hyst);
    out->trig_index = ti;

    // The transfer, at the link's rate: the bytes the 0x03 message would be.
    const double bytes = double(sweepd::proto::kIqHeadBytes) +
                         double(out->n_samples()) * (o.iq.decim == 1 ? 4.0 : 8.0);
    nap(bytes / (o.iq_link_mb_s * 1e6));
    return !cancelled();
}

// ---------------------------------------------------------------- tracking

namespace {

const char *const kDuts[] = {
    "thru",
    "bandpass 2.40-2.50 GHz, 5-pole",
    "lowpass 1 GHz, 7-pole",
    "20 dB attenuator",
    "nothing connected",
};

double dut_db(int dut, double f) {
    switch (dut) {
        case 1: {
            const double f0 = std::sqrt(2.40e9 * 2.50e9), bw = 0.10e9;
            const double x = (f * f - f0 * f0) / (f * bw);
            return -10.0 * std::log10(1.0 + std::pow(x * x, 5)) - 1.2;
        }
        case 2: return -10.0 * std::log10(1.0 + std::pow(f / 1e9, 14)) - 0.3;
        case 3: return -20.0;
        case 4: return -150.0;
        default: return 0.0;
    }
}

// The cable and both front ends, as the E310 measured them through a thru at
// 30 dB, minus the DAC level and the attenuation. Keyed on the step's LO for
// the band switch, because it is the LO that selects the path, not the tone --
// which is why the measured jump sits at a step edge (3013 MHz), not at 3 GHz.
double path_db(double f, double lo) {
    const double ghz = f * 1e-9;
    double g = 23.5 - 9.0 * ghz - 1.2 * ghz * ghz + 1.5 * std::sin(ghz * 7.3);
    if (lo >= 3.0e9) g += 10.0;
    return g;
}

}  // namespace

bool SyntheticSource::track_derive(const SyntheticOptions &o, sweepd::TrackDerived *d,
                                   std::string *err) const {
    sweepd::TrackConfig c = o.track;
    c.f_start_hz = o.f_start_hz;
    c.f_stop_hz = o.f_stop_hz;
    c.window = o.window;
    return sweepd::derive_track(c, o.fs_hz, 56000000LL, 70000000LL, 6000000000LL, d, err);
}

bool SyntheticSource::set_tracking(const TrackRequest &r, std::string *err) {
    std::lock_guard<std::mutex> lk(mu_);
    SyntheticOptions o = opt_;
    o.tracking = true;
    o.zero_span = false;
    o.iq_mode = false;
    o.gen.on = false;   // tracking takes the transmitter, as on the radio
    if (r.atten_db == r.atten_db) o.track.atten_db = r.atten_db;
    if (r.spacing_hz == r.spacing_hz && r.spacing_hz > 0) o.track.spacing_hz = llround(r.spacing_hz);
    if (r.tones >= 0) o.track.tones = size_t(r.tones);
    if (r.coherent > 0) o.track.coherent = size_t(r.coherent);
    if (r.harmonic > 0) o.track.harmonic = r.harmonic;
    if (r.rx_offset_hz == r.rx_offset_hz) o.track.rx_offset_hz = llround(r.rx_offset_hz);
    if (r.rx_invert >= 0) o.track.rx_invert = r.rx_invert != 0;
    if (r.stim_off >= 0) o.track.stim_off = r.stim_off != 0;
    // The daemon's guard, reproduced so the soft keys are judged against the
    // same refusal they will meet on the radio.
    if (o.track.atten_db < sweepd::kTrackLoudAttenDb || o.track.atten_db > sweepd::kTrackMaxAttenDb) {
        char m[200];
        std::snprintf(m, sizeof m,
                      "refusing %.1f dB TX attenuation: below %.0f dB a direct TX->RX "
                      "cable can exceed the receiver's absolute maximum",
                      o.track.atten_db, sweepd::kTrackLoudAttenDb);
        *err = m;
        return false;
    }
    sweepd::TrackDerived d;
    if (!track_derive(o, &d, err)) return false;
    opt_ = o;
    iq_pending_.store(false);
    ++iq_gen_;
    return true;
}

bool SyntheticSource::set_generator(const sweepd::GenConfig &g, std::string *err) {
    std::lock_guard<std::mutex> lk(mu_);
    if (g.on && opt_.tracking) {
        *err = "the tracking generator is using the transmitter; leave tracking to use it "
               "as a generator";
        return false;
    }
    if (g.on && g.atten_db < sweepd::kTrackLoudAttenDb) {
        *err = "refusing TX attenuation below 30 dB without a pad (sweepd --tsa-pad)";
        return false;
    }
    sweepd::GenDerived gd;
    if (g.on && !sweepd::derive_gen(g, opt_.fs_hz, 70000000LL, 6000000000LL, &gd, err))
        return false;
    opt_.gen = g;
    return true;
}

std::vector<std::string> SyntheticSource::sim_duts() const {
    return std::vector<std::string>(std::begin(kDuts), std::end(kDuts));
}

int SyntheticSource::sim_dut() const {
    std::lock_guard<std::mutex> lk(mu_);
    return opt_.dut;
}

void SyntheticSource::set_sim_dut(int i) {
    std::lock_guard<std::mutex> lk(mu_);
    if (i >= 0 && i < int(sizeof kDuts / sizeof kDuts[0])) opt_.dut = i;
}

void SyntheticSource::build_track_frame(sweepd::proto::TraceMsg *out) {
    SyntheticOptions opt;
    {
        std::lock_guard<std::mutex> lk(mu_);
        opt = opt_;
    }
    sweepd::TrackDerived d;
    std::string err;
    if (!track_derive(opt, &d, &err)) return;
    std::vector<int16_t> stim;
    double tone_dbfs = 0;
    sweepd::track_stimulus(d, opt.track.scale, &stim, nullptr, &tone_dbfs);

    // The floor in the measurement's own noise bandwidth: -100 dBFS at the
    // default 2.8 kHz, as the E310 read it mid-band at gain 20.
    const double floor_db = -100.0 + 10.0 * std::log10(d.rbw_hz / 2813.0);
    std::normal_distribution<double> g(0.0, 1.0);

    out->seq = seq_++;
    out->t_mono_s = mono_now();
    out->f_start_hz = (long long)llround(d.out_lo_hz);
    out->f_stop_hz = (long long)llround(d.out_hi_hz);
    out->bin_hz = d.spacing_hz;
    out->rbw_hz = d.rbw_hz;
    out->x_kind = sweepd::proto::XKind::Frequency;
    out->detector = sweepd::Detector::Sample;
    out->window = opt.window;
    out->gain_db = 20.0f;
    out->sweep_seconds = double(d.steps.size()) * kTrackStepS;
    out->db.assign(d.n_points, 0.0f);
    out->n_segments = uint32_t(d.steps.size());
    out->overload.assign((d.steps.size() + 7) / 8, 0);
    out->n_overloaded_segments = 0;

    for (const sweepd::TrackStep &st : d.steps) {
        // Below 500 MHz each LO write lands its step within a few tenths of a
        // dB of the last time, as a block (measured: 0.3-0.9 dB sd per step,
        // 0.02 dB within one). Reproduced so the UI's averaging is judged
        // against it rather than against a perfectly repeatable fake.
        const double lo = double(st.lo_hz);
        const double step_jitter = lo < 0.5e9 ? 0.5 * g(rng_) : 0.05 * g(rng_);
        for (size_t i = 0; i < st.cnt; ++i) {
            const size_t p = st.p0 + i;
            const double f = d.out_lo_hz + (double(p) + 0.5) * d.spacing_hz;
            // A harmonic or a converter: the fake path is read at the receive
            // frequency, and the "DUT" converts at a made-up -20 dB. Nothing
            // arrives when the stimulus is silenced.
            const bool mapped = d.harmonic != 1 || d.rx_offset_hz != 0;
            const double f_rx = d.rx_invert ? double(d.rx_offset_hz) - d.harmonic * f
                                            : d.harmonic * f + double(d.rx_offset_hz);
            const double sig = opt.track.stim_off
                                   ? -400.0
                                   : tone_dbfs - opt.track.atten_db +
                                         path_db(f_rx, double(st.rx_lo_hz)) +
                                         (mapped ? -20.0 : dut_db(opt.dut, f)) + step_jitter;
            // Signal plus complex-Gaussian noise, in power.
            const double a = std::pow(10.0, sig / 20.0);
            const double n = std::pow(10.0, floor_db / 20.0) / std::sqrt(2.0);
            const double re = a + n * g(rng_), im = n * g(rng_);
            out->db[p] = float(10.0 * std::log10(re * re + im * im + 1e-30));
        }
    }
}

double SyntheticSource::zs_dt_s() const {
    const double denom = opt_.zs_overlap >= 0.75 ? 4.0 : (opt_.zs_overlap >= 0.5 ? 2.0 : 1.0);
    return (double(opt_.zs_n_fft) / denom) / std::max(1.0, opt_.fs_hz);
}

double SyntheticSource::zs_time_res_s() const {
    // n_fft / fs. Not dt: the overlap divides dt and leaves this alone, and the
    // generator has to reproduce that or the UI's claim about it is untested.
    return double(opt_.zs_n_fft) / std::max(1.0, opt_.fs_hz);
}

void SyntheticSource::run() {
    double next = mono_now();
    while (!stop_.load()) {
        bool iq;
        {
            std::lock_guard<std::mutex> lk(mu_);
            iq = opt_.iq_mode;
        }
        if (iq) {
            // No traces in IQ mode: bursts only, one per arm.
            sweepd::proto::IqMsg m;
            if (iq_pending_.load() && build_iq_burst(&m)) {
                std::lock_guard<std::mutex> bl(box_mu_);
                iq_pending_.store(false);
                while (iq_queue_.size() >= 2) iq_queue_.pop_front();
                iq_queue_.push_back(std::move(m));
                ++iq_bursts_;
            } else {
                timespec ts{};
                ts.tv_nsec = 2000000;
                nanosleep(&ts, nullptr);
            }
            next = mono_now();
            continue;
        }
        double period;
        {
            std::lock_guard<std::mutex> lk(mu_);
            period = 1.0 / std::max(0.1, std::min(200.0, opt_.sweeps_per_s));
            // Tracking runs at the radio's pace, not the fake's: a full span is
            // seconds per sweep, and a UI judged at 20 sweeps/s would hide how
            // that feels.
            if (opt_.tracking) {
                sweepd::TrackDerived d;
                std::string e;
                if (track_derive(opt_, &d, &e))
                    period = std::max(0.05, double(d.steps.size()) * kTrackStepS);
            }
        }

        sweepd::proto::TraceMsg f;
        bool have = true;
        {
            bool zs;
            {
                std::lock_guard<std::mutex> lk(mu_);
                zs = opt_.zero_span;
            }
            bool tr;
            {
                std::lock_guard<std::mutex> lk(mu_);
                tr = opt_.tracking;
            }
            if (tr) {
                build_track_frame(&f);
            } else if (zs) {
                {
                    std::lock_guard<std::mutex> bl(box_mu_);
                    ++acquisitions_;
                }
                // False means the trigger was armed and found no edge, which
                // produces no frame at all -- exactly as the daemon does. The
                // screen keeps the last trace that fired and the WAIT TRIG
                // badge says why, rather than a free-run acquisition being
                // drawn under a triggered label.
                have = build_zspan_frame(&f);
            } else {
                build_frame(&f);
            }
        }

        {
            std::lock_guard<std::mutex> lk(box_mu_);
            if (!have) {
                ++no_trigger_;
            } else {
                // Overflow drops the *oldest*: if the display has fallen a
                // second behind, the second-old sweep is the one nobody wants.
                // The loss is counted either way -- see FrameSource::poll.
                while (queue_.size() >= kQueueDepth) {
                    queue_.pop_front();
                    ++dropped_;
                }
                queue_.push_back(std::move(f));
                no_trigger_at_frame_ = no_trigger_;
            }
        }

        next += period;
        const double now = mono_now();
        if (next < now) next = now;   // fell behind; do not sprint to catch up
        // Sleep in slices so the destructor is not held up by a slow sweep rate.
        while (!stop_.load()) {
            const double left = next - mono_now();
            if (left <= 0) break;
            timespec ts{};
            const double nap = std::min(left, 0.02);
            ts.tv_nsec = long(nap * 1e9);
            nanosleep(&ts, nullptr);
        }
    }
}

void SyntheticSource::build_frame(sweepd::proto::TraceMsg *out) {
    SyntheticOptions opt;
    std::vector<Emitter> ems;
    {
        std::lock_guard<std::mutex> lk(mu_);
        opt = opt_;
        ems = emitters_;
    }

    const double t = mono_now() - t0_;
    const size_t n_pts = opt.n_points;
    const size_t over = opt.oversample;
    const size_t n_fine = n_pts * over;
    const double span = double(opt.f_stop_hz - opt.f_start_hz);
    const double fine_hz = span / double(n_fine);

    power_.assign(n_fine, 0.0f);

    std::uniform_real_distribution<double> uni(std::numeric_limits<double>::min(), 1.0);

    // ---- floor -------------------------------------------------------------
    //
    // Exponentially distributed power, which is what the magnitude-squared of
    // complex Gaussian noise actually is. A floor drawn as "mean plus a small
    // uniform wobble" would be visibly too tidy, and worse, it would make the
    // RMS and Peak detectors look almost identical -- hiding the one difference
    // §5 insists on getting right.
    //
    // Tilted with frequency, because a real front end is not flat and §7's
    // whole argument is that the display must not imply it is.
    for (size_t i = 0; i < n_fine; ++i) {
        const double f = double(opt.f_start_hz) + (double(i) + 0.5) * fine_hz;
        const double ghz = f * 1e-9;
        const double tilt = -1.5 * ghz - 6.0 * std::max(0.0, ghz - 3.8);
        const double mean = db_to_power(opt.noise_floor_dbfs + tilt);
        power_[i] = float(-mean * std::log(uni(rng_)));
    }

    // ---- emitters ----------------------------------------------------------
    //
    // Each emitter is deposited twice: once where it is, and once where the
    // receiver's uncorrected quadrature puts its mirror, which is 2*lo - f for
    // whichever segment LO owns it. That second deposit is what §7's image
    // check exists to find, and it is here rather than in the check's fixture
    // because a UI feature that can only be exercised against hardware is a UI
    // feature nobody develops.
    const double seg_span_e = span / double(opt.n_segments);
    const double first_lo_e =
        double(opt.f_start_hz) + seg_span_e * 0.5 + double(opt.lo_shift_hz);
    const auto owner_lo = [&](double f) {
        double k = std::floor((f - first_lo_e) / seg_span_e + 0.5);
        if (k < 0) k = 0;
        if (k > double(opt.n_segments) - 1) k = double(opt.n_segments) - 1;
        return first_lo_e + k * seg_span_e;
    };

    // The generator, if on: through the same made-up path as tracking.
    if (opt.gen.on) {
        sweepd::GenDerived gd;
        std::string err;
        if (sweepd::derive_gen(opt.gen, opt.fs_hz, 70000000LL, 6000000000LL, &gd, &err)) {
            const sweepd::GenConfig &g = opt.gen;
            double fc = double(g.freq_hz);
            if (g.sweep_step_hz && gd.sweep_points)
                fc = double(sweepd::gen_sweep_freq(
                    g, size_t(t / std::max(0.001, g.sweep_dwell_s)) % gd.sweep_points));
            const double lvl = 20.0 * std::log10(g.scale) - g.atten_db + path_db(fc, fc);
            auto cw = [&](double f, double db) {
                Emitter e;
                e.f_hz = e.f_now_hz = f;
                e.power_db = db;
                e.shape = Emitter::Shape::Cw;
                ems.push_back(e);
            };
            switch (g.mod) {
                case sweepd::GenMod::CW: cw(fc, lvl); break;
                case sweepd::GenMod::AM: {
                    const double c = lvl - 20.0 * std::log10(1.0 + g.am_depth);
                    cw(fc, c);
                    cw(fc - gd.rate_hz, c + 20.0 * std::log10(g.am_depth / 2.0));
                    cw(fc + gd.rate_hz, c + 20.0 * std::log10(g.am_depth / 2.0));
                    break;
                }
                case sweepd::GenMod::FSK:
                    cw(fc - g.fm_dev_hz, lvl - 3.0);
                    cw(fc + g.fm_dev_hz, lvl - 3.0);
                    break;
                default: {
                    // FM and ASK as an occupied channel of Carson's width. Not
                    // their line spectra -- fabricated, and labelled so.
                    Emitter e;
                    e.f_hz = e.f_now_hz = fc;
                    e.bw_hz = 2.0 * ((g.mod == sweepd::GenMod::FM ? g.fm_dev_hz : 0) + gd.rate_hz);
                    e.power_db = lvl - 3.0;
                    e.shape = Emitter::Shape::Channel;
                    ems.push_back(e);
                }
            }
        }
    }

    std::uniform_real_distribution<double> u01(0.0, 1.0);
    for (Emitter &e : ems) {
        e.on = (e.duty >= 1.0) || (u01(rng_) < e.duty);
        if (!e.on) continue;

        double f = e.f_hz + e.drift_hz_per_s * t;
        if (e.hop_hz > 0) {
            std::uniform_real_distribution<double> h(-e.hop_hz, e.hop_hz);
            f += h(rng_);
        }
        e.f_now_hz = f;

        // Effective width. A CW tone is not a delta on screen: the window
        // spreads it over a few bins, and drawing it as a single sample would
        // make it vanish under decimation in a way the real one does not.
        const double bw = (e.shape == Emitter::Shape::Cw) ? 2.5 * fine_hz : e.bw_hz;
        const double half = std::max(bw * 0.5, fine_hz);
        const double skirt = (e.shape == Emitter::Shape::Noise) ? half : half * 0.35;

        const auto deposit = [&](double at, double power_db) {
            const double lo_f = at - half - 4 * skirt;
            const double hi_f = at + half + 4 * skirt;
            long long i0 = (long long)std::floor((lo_f - double(opt.f_start_hz)) / fine_hz);
            long long i1 = (long long)std::ceil((hi_f - double(opt.f_start_hz)) / fine_hz);
            i0 = std::max<long long>(i0, 0);
            i1 = std::min<long long>(i1, (long long)n_fine);

            const double peak = db_to_power(power_db);
            for (long long i = i0; i < i1; ++i) {
                const double fi = double(opt.f_start_hz) + (double(i) + 0.5) * fine_hz;
                const double d = std::fabs(fi - at);
                double shape;
                if (d <= half) {
                    shape = 1.0;
                } else {
                    // Gaussian skirts. Not a real spectral mask, but it falls off
                    // at a rate that looks like an occupied channel rather than a
                    // rectangle, and it keeps adjacent emitters from stacking into
                    // an obvious step.
                    const double x = (d - half) / skirt;
                    shape = std::exp(-0.5 * x * x);
                }
                // Modulation: each bin of an occupied channel fluctuates the same
                // way the floor does. Only a CW carrier is steady.
                double fluct = 1.0;
                if (e.shape != Emitter::Shape::Cw) fluct = -std::log(uni(rng_));
                power_[size_t(i)] += float(peak * shape * fluct);
            }
        };

        deposit(f, e.power_db);
        // The mirror. Its level is independent of the grid, so it lands
        // somewhere different in every pass and the check has something real to
        // measure.
        if (opt.image_rej_db < 200.0)
            deposit(2 * owner_lo(f) - f, e.power_db - opt.image_rej_db);
    }

    // ---- detector ----------------------------------------------------------
    //
    // The daemon's own detect(), not a reimplementation. It is the one piece of
    // sweepd the UI links against, and it means switching the detector in the
    // UI changes the synthetic trace the same way it will change a real one.
    reduced_.assign(n_pts, 0.0f);
    sweepd::detect(power_.data(), n_fine, opt.detector, reduced_.data(), n_pts);
    sweepd::power_to_db(reduced_.data(), n_pts);

    // ---- frame -------------------------------------------------------------
    out->seq = ++seq_;
    out->t_mono_s = mono_now();
    out->f_start_hz = opt.f_start_hz;
    out->f_stop_hz = opt.f_stop_hz;
    out->n_segments = opt.n_segments;
    out->bin_hz = span / double(n_pts);
    out->rbw_hz = out->bin_hz * sweepd::make_window(opt.window, 64).enbw;
    out->sweep_seconds = 1.0 / std::max(1e-6, opt.sweeps_per_s);
    out->gain_db = 30.0f;
    out->detector = opt.detector;
    out->window = opt.window;
    out->db = reduced_;

    // ---- segment holes and overload ---------------------------------------
    const double seg_span = span / double(opt.n_segments);
    out->overload.assign((opt.n_segments + 7) / 8, 0);
    out->n_overloaded_segments = 0;

    for (uint32_t s = 0; s < opt.n_segments; ++s) {
        const double centre =
            double(opt.f_start_hz) + (double(s) + 0.5) * seg_span + double(opt.lo_shift_hz);

        if (opt.blank_holes) {
            // The daemon's dc_policy=blank hole: LO leakage at the segment
            // centre, blanked and left as NaN. Narrower than a display pixel at
            // full span, which is the case the decimator has to get right.
            const double h0 = centre - opt.blank_hz * 0.5;
            const double h1 = centre + opt.blank_hz * 0.5;
            long long b0 = (long long)std::floor((h0 - double(opt.f_start_hz)) / out->bin_hz);
            long long b1 = (long long)std::ceil((h1 - double(opt.f_start_hz)) / out->bin_hz);
            b0 = std::max<long long>(b0, 0);
            b1 = std::min<long long>(b1, (long long)n_pts);
            for (long long b = b0; b < b1; ++b)
                out->db[size_t(b)] = std::numeric_limits<float>::quiet_NaN();
        }

        // A segment overloads when something very strong is inside it. -6 dBFS
        // is the generator's stand-in for the daemon's near-full-scale sample
        // count; the flag means the same thing either way.
        for (const Emitter &e : ems) {
            if (!e.on || e.power_db < -6.0) continue;
            if (std::fabs(e.f_now_hz - centre) <= seg_span * 0.5) {
                sweepd::proto::set_overload_bit(&out->overload, opt.n_segments, s);
                ++out->n_overloaded_segments;
                break;
            }
        }
    }
}

// ---------------------------------------------------------------- zero span

// The same environment through a parked LO: total power in the measurement band
// versus time.
//
// The physics that matters for the *display* is the burst structure, so that is
// what this models. Each emitter inside the band gets a deterministic pulse
// period and width derived from its own frequency, which gives a repeatable
// scene with edges that land between points -- which is the case the trace has
// to render honestly, because a pulse edge inside a block is genuinely smeared
// across that block's whole integration time.
//
// Time runs continuously across frames, not from 0 each time, so a burst does
// not appear frozen at the same x on every frame. But each frame's *axis*
// starts at 0, matching the daemon: the frames are not contiguous with each
// other -- there is a gap while the transform runs -- so an absolute time would
// be a number with nothing behind it.
bool SyntheticSource::build_zspan_frame(sweepd::proto::TraceMsg *out) {
    SyntheticOptions opt;
    std::vector<Emitter> ems;
    double dt = 0, res = 0;
    {
        std::lock_guard<std::mutex> lk(mu_);
        opt = opt_;
        ems = emitters_;
        dt = zs_dt_s();
        res = zs_time_res_s();
    }

    // The acquisition, which under a trigger is longer than the frame it will
    // emit -- the same geometry the daemon uses, so a UI checked against this
    // is checked against the real one. See src/sweepd/zspan.h.
    const bool rising = opt.zs_trigger == "video_rising";
    const bool triggered = rising || opt.zs_trigger == "video_falling";
    const size_t pre =
        triggered ? std::min(opt.zs_points - 1,
                             size_t(llround(opt.zs_trig_pretrig * double(opt.zs_points))))
                  : 0;
    const size_t search =
        triggered ? (opt.zs_trig_search ? opt.zs_trig_search : opt.zs_points) : 0;

    const size_t n = opt.zs_points + search;
    const double t_base = mono_now() - t0_;
    const double band_lo = double(opt.center_hz) - double(opt.meas_bw_hz) * 0.5;
    const double band_hi = double(opt.center_hz) + double(opt.meas_bw_hz) * 0.5;

    // Which emitters this LO can actually see. Everything outside the
    // measurement band contributes nothing, which is the whole point of the
    // band and the thing a UI reviewer should be able to check by retuning.
    struct Burst {
        double power = 0;
        double period_s = 0;
        double width_s = 0;
        double phase_s = 0;
    };
    std::vector<Burst> bursts;
    for (const Emitter &e : ems) {
        const double f = e.f_now_hz > 0 ? e.f_now_hz : e.f_hz;
        const double half_bw = std::max(e.bw_hz, 1.0) * 0.5;
        if (f + half_bw < band_lo || f - half_bw > band_hi) continue;

        Burst b;
        b.power = db_to_power(e.power_db);
        // Derived from the emitter's own frequency so the scene is stable
        // across frames and across runs at a fixed seed. Periods land in the
        // hundreds of microseconds to a few milliseconds -- the regime this
        // mode exists for.
        const double k = std::fmod(f * 1e-6, 7.0);
        b.period_s = 200e-6 + k * 400e-6;
        b.width_s = b.period_s * (e.duty >= 1.0 ? 0.5 : std::max(0.05, e.duty));
        b.phase_s = std::fmod(f * 1e-9, 1.0) * b.period_s;
        bursts.push_back(b);
    }

    const double floor_mean = db_to_power(opt.noise_floor_dbfs +
                                          -1.5 * double(opt.center_hz) * 1e-9);
    std::uniform_real_distribution<double> uni(std::numeric_limits<double>::min(), 1.0);

    reduced_.assign(n, 0.0f);
    for (size_t i = 0; i < n; ++i) {
        const double t = t_base + double(i) * dt;

        // Exponentially distributed, as in the swept generator and for the same
        // reason: it is what the magnitude-squared of complex Gaussian noise is,
        // and it is what makes Peak and RMS visibly differ.
        double p = -floor_mean * std::log(uni(rng_));

        for (const Burst &b : bursts) {
            const double ph = std::fmod(t + b.phase_s, b.period_s);
            // A block straddling an edge sees part of the burst, so the point
            // reads part of the power. Modelling that rather than snapping to
            // on/off is what makes the rendered rise time equal the resolution
            // -- which is the honest answer, and the one the UI claims.
            double frac = 0.0;
            if (ph < b.width_s) {
                frac = std::min(1.0, (b.width_s - ph) / std::max(res, 1e-12));
                frac = std::min(1.0, std::max(frac, std::min(1.0, ph / std::max(res, 1e-12))));
            } else if (ph < b.width_s + res) {
                frac = std::max(0.0, 1.0 - (ph - b.width_s) / std::max(res, 1e-12));
            }
            if (frac <= 0) continue;
            p += b.power * frac * -std::log(uni(rng_));
        }
        reduced_[i] = float(p);
    }

    // The trigger runs on linear power, before the dB conversion -- the same
    // quantity, through the same object, as sweepd::ZeroSpan. A separate
    // reading of "what counts as an edge" here would let the UI be developed
    // against a rule the daemon does not have.
    size_t start = 0;
    if (triggered) {
        const double level = std::pow(10.0, opt.zs_trig_level_db / 10.0);
        const double arm = std::pow(
            10.0, (opt.zs_trig_level_db + (rising ? -opt.zs_trig_hyst_db
                                                  : opt.zs_trig_hyst_db)) / 10.0);
        size_t at = 0;
        if (!sweepd::find_video_trigger(reduced_.data(), n, rising, level, arm, pre,
                                        pre + search, &at))
            return false;   // no edge: no frame, exactly as the daemon does
        start = at - pre;
    }

    const size_t np = opt.zs_points;
    if (start > 0)
        std::copy(reduced_.begin() + start, reduced_.begin() + start + np,
                  reduced_.begin());
    reduced_.resize(np);
    sweepd::power_to_db(reduced_.data(), np);

    out->seq = ++seq_;
    out->t_mono_s = mono_now();
    // Both ends carry the parked centre and bin_hz is 0: there is no frequency
    // grid here, and a plausible-looking wrong one is worse than an obviously
    // absent one (proto.h).
    out->f_start_hz = opt.center_hz;
    out->f_stop_hz = opt.center_hz;
    out->bin_hz = 0.0;
    out->x_kind = sweepd::proto::XKind::Time;
    out->dt_ns = (uint64_t)llround(dt * 1e9);
    // Negative by exactly the pre-trigger history, so t=0 is the edge. 0 in
    // free run, which has no origin to measure from.
    out->t0_ns = -(int64_t)pre * (int64_t)out->dt_ns;
    out->n_segments = 1;
    out->rbw_hz = sweepd::rbw_hz(sweepd::make_window(opt.window, opt.zs_n_fft).enbw,
                                 opt.fs_hz, opt.zs_n_fft);
    out->sweep_seconds = 1.0 / std::max(1e-6, opt.sweeps_per_s);
    out->gain_db = 30.0f;
    out->detector = opt.detector;
    out->window = opt.window;
    out->db = reduced_;

    out->overload.assign(1, 0);
    out->n_overloaded_segments = 0;
    for (const Burst &b : bursts) {
        if (b.power < db_to_power(-6.0)) continue;
        sweepd::proto::set_overload_bit(&out->overload, 1, 0);
        out->n_overloaded_segments = 1;
        break;
    }
    return true;
}

}  // namespace sweepui
