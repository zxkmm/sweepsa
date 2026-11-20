#include "rtlsdrsource.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>

#ifdef SWEEPSA_HAVE_RTLSDR
#include <rtl-sdr.h>

#include "stitch.h"

// Exported by the nextrtl librtlsdr fork only. Weak, so it resolves to null
// when the stock library is the one loaded and the source falls back to the
// plain FC0013 range.
extern "C" uint32_t rtlsdr_get_nextrtl_hf_lo(rtlsdr_dev_t *dev) __attribute__((weak));
#endif

namespace sweepui {

// =====================================================================
// The hardware-free half.
//
// Everything above the device sits outside the #ifdef on purpose: the geometry
// is where a mistake is invisible -- a wrong answer is a plausible trace, not an
// error -- so it is unit-tested (tests/rtlsdr_test.cpp), and a test that needs
// librtlsdr installed in order to check arithmetic would be tested less often.
// =====================================================================

const char *rtl_tuner_name(RtlTuner t) {
    switch (t) {
        case RtlTuner::E4000: return "E4000";
        case RtlTuner::FC0012: return "FC0012";
        case RtlTuner::FC0013: return "FC0013";
        case RtlTuner::FC2580: return "FC2580";
        case RtlTuner::R820T: return "R820T";
        case RtlTuner::R828D: return "R828D";
        case RtlTuner::Unknown: break;
    }
    return "unknown";
}

bool rtl_tuner_range(RtlTuner t, int direct_sampling, long long *lo_hz, long long *hi_hz) {
    // Direct sampling bypasses the tuner, so which tuner it is stops mattering:
    // the input is the RTL2832's own first Nyquist zone off a nominal 28.8 MHz
    // crystal. The lower bound is not 0 because the DC blank would swallow it
    // and because nothing below ~100 kHz reaches the ADC through the dongle's
    // input network anyway.
    if (direct_sampling != 0) {
        *lo_hz = 100000LL;
        *hi_hz = 14400000LL;
        return true;
    }
    switch (t) {
        // librtlsdr's own tuner drivers, and every one of these is published
        // rather than measured here. See the header.
        case RtlTuner::R820T:
        case RtlTuner::R828D:
            *lo_hz = 24000000LL;
            *hi_hz = 1766000000LL;
            return true;
        case RtlTuner::E4000:
            *lo_hz = 52000000LL;
            *hi_hz = 2200000000LL;
            return true;
        case RtlTuner::FC0012:
            *lo_hz = 22000000LL;
            *hi_hz = 948600000LL;
            return true;
        case RtlTuner::FC0013:
            *lo_hz = 22000000LL;
            *hi_hz = 1100000000LL;
            return true;
        case RtlTuner::FC2580:
            // Two bands -- 146-308 and 438-924 MHz -- with a hole between them.
            // The plan takes one interval, so these are the outer bounds and a
            // sweep across the hole will show the gap as noise rather than as a
            // gap. Recorded in docs/rtlsdr.md; an FC2580 is rare enough that
            // teaching PlanConfig about disjoint tuner bands for it would be
            // work spent on nobody's dongle.
            *lo_hz = 146000000LL;
            *hi_hz = 924000000LL;
            return true;
        case RtlTuner::Unknown:
            break;
    }
    // Guessing a range for a part we could not identify is the one option with
    // no defence: every other branch here is at least somebody's datasheet.
    *lo_hz = 0;
    *hi_hz = 0;
    return false;
}

void nextrtl_range(long long *lo_hz, long long *hi_hz) {
    // Low end: the fork's upconverter path, with the same 100 kHz floor and
    // reasoning as direct sampling above. High end: the FC0013 PLL locks to
    // 1.7 GHz and a 1575 MHz tone was received on the main input (measured on
    // the rev 1 board), past the 1.1 GHz the FC0013 row claims.
    *lo_hz = 100000LL;
    *hi_hz = 1700000000LL;
}

bool rtl_snap_gain(const std::vector<int> &tenths, double want_db, double *got_db) {
    if (tenths.empty()) return false;
    const double want_tenths = want_db * 10.0;
    size_t best = 0;
    double best_d = std::fabs(double(tenths[0]) - want_tenths);
    for (size_t i = 1; i < tenths.size(); ++i) {
        const double d = std::fabs(double(tenths[i]) - want_tenths);
        if (d < best_d) {
            best_d = d;
            best = i;
        }
    }
    *got_db = double(tenths[best]) / 10.0;
    return true;
}

std::string RtlSdrInfo::gain_list() const {
    std::string s;
    for (size_t i = 0; i < gains_tenth_db.size(); ++i) {
        char buf[16];
        std::snprintf(buf, sizeof buf, "%.1f", double(gains_tenth_db[i]) / 10.0);
        if (i) s += ' ';
        s += buf;
    }
    return s;
}

size_t rtl_read_bytes(size_t n_fft) {
    // Two bytes per complex sample: unsigned 8-bit I and Q, interleaved.
    const size_t want = 2 * n_fft;
    // librtlsdr hands the length straight to libusb_bulk_transfer and every
    // tool in the rtl-sdr tree asks in multiples of 512. Rounding up rather
    // than down, and the transform then reads the END of the buffer, so the
    // extra samples are spent on settling rather than discarded for nothing.
    const size_t granularity = 512;
    return ((want + granularity - 1) / granularity) * granularity;
}

bool rtl_validate(RtlSdrOptions *o, std::string *err) {
    auto fail = [&](const std::string &m) {
        *err = m;
        return false;
    };

    if (o->f_stop_hz <= o->f_start_hz) return fail("stop must be above start");

    // librtlsdr's two accepted windows, transcribed from its own test in
    // rtlsdr_set_sample_rate: it rejects rate <= 225000, rate > 3200000, and
    // 300000 < rate <= 900000. Restated here so the refusal is a sentence
    // rather than an error code from inside the worker, and the edges are
    // pinned in tests/rtlsdr_test.cpp because an off-by-one would only show up
    // against a dongle.
    const long long fs = (long long)std::llround(o->fs_hz);
    const bool low_window = fs >= 225001 && fs <= 300000;
    const bool high_window = fs >= 900001 && fs <= 3200000;
    if (!low_window && !high_window)
        return fail("librtlsdr accepts 225001-300000 Hz and 900001-3200000 Hz and "
                    "nothing between; " + std::to_string(fs) + " Hz is outside both");

    if (o->n_fft < 64 || o->n_fft > 65536)
        return fail("FFT size must be between 64 and 65536");
    if (o->n_fft & (o->n_fft - 1)) return fail("FFT size must be a power of two");

    if (o->keep_hz < 0) return fail("keep must not be negative");
    if (o->keep_hz > fs)
        return fail("keep " + std::to_string(o->keep_hz) + " Hz exceeds the sample rate " +
                    std::to_string(fs) + " Hz; the kept band would alias");
    if (o->blank_hz < 0) return fail("the DC blank must not be negative");

    if (o->direct_sampling < 0 || o->direct_sampling > 2)
        return fail("direct sampling is 0 (off), 1 (I branch) or 2 (Q branch)");

    if (o->ppm < -1000 || o->ppm > 1000)
        return fail("a frequency correction beyond +/-1000 ppm is a broken crystal, "
                    "not a correction");

    // A guard, not a physics limit: the settle is per hop and a full span is
    // thousands of hops, so a mistyped millisecond figure is a sweep nobody
    // waits out.
    if (o->settle_us > 1000000u)
        return fail("a settle longer than a second per hop would make even a narrow "
                    "span take minutes");

    if (o->max_points == 0) return fail("max_points must be positive");
    if (o->n_points > o->max_points)
        return fail("more points than max_points (" + std::to_string(o->max_points) + ")");

    // Rounded rather than refused: the only thing that matters about the dump is
    // that librtlsdr can ask for it.
    if (o->dump_bytes) o->dump_bytes = ((o->dump_bytes + 511) / 512) * 512;
    return true;
}

sweepd::PlanConfig rtl_plan_config(const RtlSdrOptions &o, double fs_achieved_hz,
                                   long long tune_lo_hz, long long tune_hi_hz) {
    sweepd::PlanConfig c;
    c.f_start_hz = o.f_start_hz;
    c.f_stop_hz = o.f_stop_hz;
    c.fs_hz = (long long)std::llround(fs_achieved_hz);

    // This librtlsdr cannot read back the IF filter it applied, so with
    // automatic bandwidth there is no measured number to check the kept band
    // against. Using the sample rate is the honest stand-in -- it is what
    // librtlsdr tracks the automatic filter to -- and inventing a filter width
    // would make plan_sweep's "the kept band's edges are past the filter" check
    // pass or fail on a fiction.
    c.bw_hz = o.bw_hz > 0 ? o.bw_hz : c.fs_hz;

    // fs/2, so the kept band is the LO +/- fs/4 and the splice fill is drawn
    // from fs/4 out rather than from the filter corner. See the header.
    c.keep_hz = o.keep_hz > 0 ? o.keep_hz : c.fs_hz / 2;

    c.blank_hz = o.blank_hz;
    c.dc = o.dc;
    c.lo_shift_hz = o.lo_shift_hz;
    c.lo_min_hz = tune_lo_hz;
    c.lo_max_hz = tune_hi_hz;
    return c;
}

bool rtlsdr_available() {
#ifdef SWEEPSA_HAVE_RTLSDR
    return true;
#else
    return false;
#endif
}

#ifndef SWEEPSA_HAVE_RTLSDR

// Built without librtlsdr. The option still exists and still fails with a
// sentence that says why, because "unknown argument: --rtl" would send someone
// looking for a typo rather than for a missing dependency.
struct RtlSdrSource::Impl {};

std::vector<std::string> rtlsdr_devices() { return {}; }

RtlSdrSource::RtlSdrSource(const RtlSdrOptions &) {}
RtlSdrSource::~RtlSdrSource() = default;

bool RtlSdrSource::open(std::string *err) {
    *err =
        "this sweepui was built without librtlsdr. Install the development "
        "package (librtlsdr-dev / rtl-sdr) and reconfigure CMake.";
    return false;
}

bool RtlSdrSource::poll(sweepd::proto::TraceMsg *) { return false; }
SourceConfig RtlSdrSource::config() const { return {}; }
SourceStatus RtlSdrSource::status() const { return {}; }

#define REFUSE(msg) \
    { *err = msg; return false; }

bool RtlSdrSource::set_span(double, double, std::string *err) REFUSE("no librtlsdr")
bool RtlSdrSource::set_points(size_t, std::string *err) REFUSE("no librtlsdr")
bool RtlSdrSource::set_detector(sweepd::Detector, std::string *err) REFUSE("no librtlsdr")
bool RtlSdrSource::set_window(sweepd::Window, std::string *err) REFUSE("no librtlsdr")
bool RtlSdrSource::set_segmentation(double, const std::string &, std::string *err)
    REFUSE("no librtlsdr")
bool RtlSdrSource::set_lo_shift(double, std::string *err) REFUSE("no librtlsdr")
bool RtlSdrSource::set_zero_span(double, size_t, size_t, double, std::string *err)
    REFUSE("no librtlsdr")
bool RtlSdrSource::set_trigger(const std::string &, double, double, double, std::string *err)
    REFUSE("no librtlsdr")
bool RtlSdrSource::set_sweep_mode(std::string *err) REFUSE("no librtlsdr")

#undef REFUSE

#else  // SWEEPSA_HAVE_RTLSDR

namespace {

using sweepd::Detector;
using sweepd::Window;

// Unsigned 8-bit samples centred on 127.5, which is the arithmetic midpoint of
// 0..255. Normalising by the same number puts a signal that swings the full
// 0..255 at exactly 0 dBFS, matching what SweepDevice::full_scale() does with
// the AD936x's 2048 and hackrfsource.cpp with its int8 128.
//
// librtlsdr's own tools use 127.4, an empirical correction for the RTL2832's
// residual DC offset. The difference lands entirely in the DC bin, which this
// source blanks in every segment, so the arithmetic midpoint is the defensible
// choice and the fudge would buy nothing.
constexpr double kOffset = 127.5;
constexpr double kFullScale = 127.5;

double now_s() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

RtlTuner map_tuner(enum rtlsdr_tuner t) {
    switch (t) {
        case RTLSDR_TUNER_E4000: return RtlTuner::E4000;
        case RTLSDR_TUNER_FC0012: return RtlTuner::FC0012;
        case RTLSDR_TUNER_FC0013: return RtlTuner::FC0013;
        case RTLSDR_TUNER_FC2580: return RtlTuner::FC2580;
        case RTLSDR_TUNER_R820T: return RtlTuner::R820T;
        case RTLSDR_TUNER_R828D: return RtlTuner::R828D;
        case RTLSDR_TUNER_UNKNOWN: break;
    }
    return RtlTuner::Unknown;
}

}  // namespace

std::vector<std::string> rtlsdr_devices() {
    std::vector<std::string> out;
    const uint32_t n = rtlsdr_get_device_count();
    for (uint32_t i = 0; i < n; ++i) {
        char manuf[256] = {0}, prod[256] = {0}, serial[256] = {0};
        const char *name = rtlsdr_get_device_name(i);
        char buf[900];
        if (rtlsdr_get_device_usb_strings(i, manuf, prod, serial) == 0)
            std::snprintf(buf, sizeof buf, "%u: %s (%s %s, serial %s)", unsigned(i),
                          name ? name : "?", manuf, prod, serial);
        else
            std::snprintf(buf, sizeof buf, "%u: %s", unsigned(i), name ? name : "?");
        out.push_back(buf);
    }
    return out;
}

// ---------------------------------------------------------------- impl

struct RtlSdrSource::Impl {
    explicit Impl(const RtlSdrOptions &o) : opt(o) {}

    RtlSdrOptions opt;                 // the running configuration
    RtlSdrInfo info;

    rtlsdr_dev_t *dev = nullptr;

    std::atomic<bool> stop_{false};
    std::atomic<bool> restart_{false};   // a setter changed something
    std::thread worker;

    // What configure() learned, published as atomics so request() can plan
    // against the real sample rate and tuner range from the UI thread.
    std::atomic<double> fs_achieved{0};
    std::atomic<long long> tune_lo{0};
    std::atomic<long long> tune_hi{0};

    // --------------------------------------------------- frames out
    mutable std::mutex f_mu;
    std::deque<sweepd::proto::TraceMsg> frames;
    uint64_t delivered = 0;
    uint64_t local_drops = 0;

    // --------------------------------------------------- shared state
    mutable std::mutex cfg_mu;
    SourceConfig cfg;
    std::string detail;
    std::string fatal;

    std::atomic<double> sweeps_per_s{0};
    std::atomic<double> hops_per_s{0};
    std::atomic<double> last_sweep_ms{0};
    std::atomic<uint64_t> sweeps{0};

    // Two counts that must never be silent. A tune the driver refused is a
    // segment missing from the trace; a short read is a buffer librtlsdr did
    // not fill, which means the transform ran over samples from before the
    // retune.
    std::atomic<uint64_t> tune_failures{0};
    std::atomic<uint64_t> short_reads{0};

    // Requests from the UI thread, applied at the top of the worker loop.
    // `pending` always holds the most recently *desired* configuration, so a
    // setter reads it, changes one field and hands it back.
    std::mutex req_mu;
    RtlSdrOptions pending;
    bool has_pending = false;

    // ------------------------------------------------------------ lifetime

    bool open_device(std::string *err);
    void close_device();
    void start();
    void stop();

    void run();
    bool configure(std::string *err);

    // One configuration's worth of sweeping: plan the grid, publish it, then
    // sweep until something asks it to stop.
    void session();

    bool read_exact(uint8_t *buf, size_t bytes);

    void set_detail(const std::string &s);
    void set_fatal(const std::string &s);
    void publish(const SourceConfig &c);

    bool request(const RtlSdrOptions &o, std::string *err);
    bool plan_for(const RtlSdrOptions &o, sweepd::SweepPlan *out, std::string *err) const;
};

bool RtlSdrSource::Impl::plan_for(const RtlSdrOptions &o, sweepd::SweepPlan *out,
                                  std::string *err) const {
    const double fs = fs_achieved.load();
    if (fs <= 0) {
        *err = "the sample rate has not been read back from the device yet";
        return false;
    }
    return sweepd::plan_sweep(rtl_plan_config(o, fs, tune_lo.load(), tune_hi.load()), out,
                              err);
}

// ---------------------------------------------------------------- session

void RtlSdrSource::Impl::session() {
    // The grid is the daemon's, not a second reading of it: plan_sweep is the
    // same function sweep.cpp uses, which is what makes §7's overlay and §7's
    // image check meaningful on this radio.
    sweepd::SweepPlan plan;
    std::string err;
    if (!plan_for(opt, &plan, &err)) {
        set_fatal("the segment plan was refused: " + err);
        return;
    }

    const size_t n = opt.n_fft;
    const double fs = fs_achieved.load();
    const size_t read_bytes = rtl_read_bytes(n);

    const double out_lo = std::max(double(plan.covered_lo_hz), double(opt.f_start_hz));
    const double out_hi = std::min(double(plan.covered_hi_hz), double(opt.f_stop_hz));
    if (out_hi <= out_lo) {
        set_fatal("the plan covers nothing inside the requested range");
        return;
    }

    sweepd::WindowFunction win = sweepd::make_window(opt.window, n);
    sweepd::Fft fft(n);

    const double rbw = sweepd::rbw_hz(win.enbw, fs, n);
    const double span = out_hi - out_lo;
    size_t n_points = opt.n_points;
    if (n_points == 0) {
        // Match the resolution actually achieved rather than inventing one --
        // the same rule derive_sweep() uses in the daemon.
        const double want = std::ceil(span / rbw);
        const double cap = double(opt.max_points ? opt.max_points : 1);
        n_points = size_t(std::max(1.0, std::min(want, cap)));
    }
    const double bin_hz = span / double(n_points);

    const size_t n_seg = plan.segments.size();

    // The whole point of the arithmetic in the header, computed for the plan
    // actually in hand so the banner states this sweep's cost rather than a
    // table's.
    const double settle_s = double(opt.settle_us) * 1e-6;
    const double samples_s = double(read_bytes / 2 + opt.dump_bytes / 2) / fs;
    const double predicted_s = double(n_seg) * (settle_s + samples_s);

    // ------------------------------------------------------------- publish
    {
        SourceConfig c;
        c.f_start_hz = (long long)std::llround(out_lo);
        c.f_stop_hz = (long long)std::llround(out_hi);
        c.tune_min_hz = tune_lo.load();
        c.tune_max_hz = tune_hi.load();
        c.n_points = n_points;
        c.rbw_hz = rbw;
        c.bin_hz = bin_hz;
        c.n_segments = uint32_t(n_seg);
        c.gain_db = float(info.gain_achieved_db);

        // §7's segment overlay reconstructs the grid from exactly these, and
        // tests/segments_test.cpp checks that reconstruction against
        // plan_sweep -- which is the function that produced `plan` above. So
        // unlike the HackRF source, these are filled in and the overlay draws.
        c.seg_first_lo_hz = double(plan.first_lo_hz);
        c.seg_step_hz = double(plan.step_hz);
        c.keep_hz = double(plan.cfg.keep_hz);
        c.blank_hz = double(plan.cfg.blank_hz);
        c.covered_lo_hz = double(plan.covered_lo_hz);
        c.covered_hi_hz = double(plan.covered_hi_hz);
        c.lo_shift_hz = double(plan.cfg.lo_shift_hz);
        c.splice = plan.cfg.dc == sweepd::DcPolicy::SpliceOverlap;

        c.window = opt.window;
        c.detector = opt.detector;
        c.uncal = true;
        c.synthetic = false;
        c.reconfiguring = false;

        char buf[768];
        std::snprintf(buf, sizeof buf,
                      "%s (%s tuner%s%s), serial %s -- %.3f-%.3f MHz as %zu segments of "
                      "%.4f MSPS, keep %.3f MHz step %.3f MHz, gain %.1f dB. "
                      "Host-side retune at %u us settle: ~%.2f s/sweep predicted%s%s",
                      info.name.empty() ? "RTL-SDR" : info.name.c_str(),
                      rtl_tuner_name(info.tuner),
                      info.direct_sampling
                          ? (info.direct_sampling == 2 ? ", direct Q" : ", direct I")
                          : "",
                      info.nextrtl_hf_lo_hz ? ", nextrtl upconverter below 30 MHz" : "",
                      info.serial.empty() ? "?" : info.serial.c_str(), out_lo / 1e6,
                      out_hi / 1e6, n_seg, fs / 1e6, double(plan.cfg.keep_hz) / 1e6,
                      double(plan.step_hz) / 1e6, info.gain_achieved_db, opt.settle_us,
                      predicted_s,
                      plan.truncated ? " -- TRUNCATED by the tuner's range" : "",
                      fs > 2400000.0 ? " -- above 2.4 MSPS this dongle may drop samples"
                                       " and cannot report it"
                                     : "");
        c.description = buf;
        publish(c);
    }
    set_detail("");

    // ------------------------------------------------------------- sweep
    sweepd::Stitcher stitch;
    stitch.reset(out_lo, out_hi, n_points, opt.detector);

    std::vector<uint8_t> raw(read_bytes);
    std::vector<uint8_t> dump(opt.dump_bytes);
    std::vector<float> iqf(2 * n);
    std::vector<int16_t> iq16(2 * n);
    std::vector<float> power(n);
    std::vector<uint32_t> overload(n_seg, 0);

    // The int16 path carries `raw - 128`, so it is half an LSB off the true
    // midpoint. At a 98% threshold that is 125 either way and the asymmetry is
    // far below the threshold's own resolution; count_overload is shared with
    // the daemon rather than re-read here for the sake of it.
    const int16_t threshold = (int16_t)std::lround(kFullScale * opt.overload_frac);
    const float scale = float(1.0 / kFullScale);

    uint64_t seq = 0;
    double t_rate_base = 0;
    uint64_t sweeps_at_base = 0;

    while (!stop_.load() && !restart_.load()) {
        stitch.clear();
        std::fill(overload.begin(), overload.end(), 0u);
        const double t_start = now_s();
        size_t hops = 0;
        bool aborted = false;

        for (size_t k = 0; k < n_seg; ++k) {
            // Checked per segment, not per sweep: a full span is thousands of
            // hops and ~19 seconds, and a UI whose span key took that long to
            // answer would read as frozen.
            if (stop_.load() || restart_.load()) {
                aborted = true;
                break;
            }

            const sweepd::Segment &s = plan.segments[k];

            if (rtlsdr_set_center_freq(dev, uint32_t(s.lo_hz)) < 0) {
                // Counted and skipped rather than substituted. A segment whose
                // LO never moved would contribute its predecessor's spectrum at
                // this segment's frequencies, which is indistinguishable from
                // signal; leaving it out shows as an uncovered bin instead,
                // which the stitch renders NaN and §7 wants visible.
                tune_failures.fetch_add(1, std::memory_order_relaxed);
                continue;
            }

            // rtl_power's recipe, and its numbers. Both are knobs; see the
            // header for why they are most of the sweep time.
            if (opt.settle_us)
                std::this_thread::sleep_for(std::chrono::microseconds(opt.settle_us));
            if (!dump.empty() && !read_exact(dump.data(), dump.size())) {
                aborted = true;
                break;
            }
            if (!read_exact(raw.data(), raw.size())) {
                aborted = true;
                break;
            }

            // The END of the buffer: read_bytes is rounded up to the USB
            // granularity, so the surplus at the front buys a little more
            // settling instead of being thrown away. Same reason
            // hackrfsource.cpp transforms the tail of its block.
            const uint8_t *src = raw.data() + raw.size() - 2 * n;
            for (size_t i = 0; i < 2 * n; ++i) {
                iq16[i] = int16_t(int(src[i]) - 128);
                iqf[i] = (float(src[i]) - float(kOffset)) * scale;
            }
            overload[k] = uint32_t(sweepd::count_overload(iq16.data(), n, threshold));

            fft.power_spectrum(iqf.data(), win, power.data());

            // The kept and blanked edges are the plan's, so the fold here and
            // the overlay the UI draws describe the same geometry by
            // construction rather than by agreement.
            sweepd::StitchBand band;
            band.lo_hz = double(s.lo_hz);
            band.keep_lo_hz = double(s.keep_lo_hz);
            band.keep_hi_hz = double(s.keep_hi_hz);
            band.blank_lo_hz = double(s.blank_lo_hz);
            band.blank_hi_hz = double(s.blank_hi_hz);
            stitch.add(power.data(), n, fs, band);
            ++hops;
        }

        // A partial sweep is not emitted. Half a trace under a full-span axis
        // is a picture of a band that was never looked at, and the consumer has
        // no way to tell which half.
        if (aborted) continue;

        sweepd::proto::TraceMsg m;
        m.seq = seq++;
        m.t_mono_s = now_s();
        m.f_start_hz = (long long)std::llround(out_lo);
        m.f_stop_hz = (long long)std::llround(out_hi);
        m.n_segments = uint32_t(n_seg);
        m.rbw_hz = rbw;
        m.bin_hz = bin_hz;
        m.sweep_seconds = m.t_mono_s - t_start;
        m.gain_db = float(info.gain_achieved_db);
        m.detector = opt.detector;
        m.window = opt.window;
        m.x_kind = sweepd::proto::XKind::Frequency;

        m.db.resize(n_points);
        stitch.to_db(m.db.data());

        m.overload.assign((n_seg + 7) / 8, 0);
        for (size_t i = 0; i < overload.size(); ++i) {
            if (!overload[i]) continue;
            sweepd::proto::set_overload_bit(&m.overload, n_seg, i);
            ++m.n_overloaded_segments;
        }

        last_sweep_ms.store(m.sweep_seconds * 1e3, std::memory_order_relaxed);
        const uint64_t done = sweeps.fetch_add(1, std::memory_order_relaxed) + 1;
        if (done == 1) {
            t_rate_base = m.t_mono_s;
            sweeps_at_base = done;
        } else if (m.t_mono_s > t_rate_base) {
            const double dt = m.t_mono_s - t_rate_base;
            sweeps_per_s.store(double(done - sweeps_at_base) / dt, std::memory_order_relaxed);
        }
        // Per sweep rather than cumulative: this is the number that says where
        // the time went, and it is the one to watch while cutting the settle.
        if (m.sweep_seconds > 0)
            hops_per_s.store(double(hops) / m.sweep_seconds, std::memory_order_relaxed);

        {
            std::lock_guard<std::mutex> g(f_mu);
            if (frames.size() >= opt.frame_queue_depth) {
                frames.pop_front();
                ++local_drops;
            }
            frames.push_back(std::move(m));
        }

        const uint64_t tf = tune_failures.load(std::memory_order_relaxed);
        const uint64_t sr = short_reads.load(std::memory_order_relaxed);
        const size_t uncovered = stitch.uncovered();
        if (tf || sr || uncovered) {
            char buf[224];
            std::snprintf(buf, sizeof buf,
                          "%llu tunes refused, %llu short reads, %zu of %zu bins uncovered",
                          (unsigned long long)tf, (unsigned long long)sr, uncovered,
                          n_points);
            set_detail(buf);
        }
    }
}

// ---------------------------------------------------------------- worker

void RtlSdrSource::Impl::run() {
    std::string err;
    while (!stop_.load()) {
        {
            std::lock_guard<std::mutex> g(req_mu);
            if (has_pending) {
                opt = pending;
                has_pending = false;
            }
        }
        restart_.store(false);

        {
            std::lock_guard<std::mutex> g(cfg_mu);
            cfg.reconfiguring = true;
        }

        tune_failures.store(0);
        short_reads.store(0);

        if (!configure(&err)) {
            set_fatal(err);
            return;
        }
        session();
    }
}

void RtlSdrSource::Impl::start() {
    worker = std::thread([this] { run(); });
}

void RtlSdrSource::Impl::stop() {
    stop_.store(true);
    // rtlsdr_read_sync blocks in libusb with its own timeout, so the worker is
    // not interrupted here -- it notices stop_ on the next loop. Joining is
    // what makes closing the device safe, which is why it happens before
    // close_device() and not instead of it.
    if (worker.joinable()) worker.join();
    close_device();
}

bool RtlSdrSource::Impl::read_exact(uint8_t *buf, size_t bytes) {
    size_t done = 0;
    while (done < bytes) {
        if (stop_.load() || restart_.load()) return false;
        int got = 0;
        const int want = int(std::min<size_t>(bytes - done, 1u << 20));
        if (rtlsdr_read_sync(dev, buf + done, want, &got) < 0) return false;
        if (got <= 0) return false;
        if (size_t(got) < size_t(want)) short_reads.fetch_add(1, std::memory_order_relaxed);
        done += size_t(got);
    }
    return true;
}

// ---------------------------------------------------------------- device

bool RtlSdrSource::Impl::open_device(std::string *err) {
    const uint32_t count = rtlsdr_get_device_count();
    if (count == 0) {
        *err =
            "no RTL-SDR device found. If one is attached, the kernel's DVB driver has "
            "probably claimed it: blacklist dvb_usb_rtl28xxu (or `modprobe -r` it) and "
            "check the udev rule from the rtl-sdr package grants you access.";
        return false;
    }

    int idx = 0;
    if (!opt.serial.empty()) {
        idx = rtlsdr_get_index_by_serial(opt.serial.c_str());
        if (idx < 0) {
            *err = "no attached RTL-SDR has serial \"" + opt.serial + "\"";
            return false;
        }
    } else if (opt.index >= 0) {
        if (uint32_t(opt.index) >= count) {
            *err = "device index " + std::to_string(opt.index) + " but only " +
                   std::to_string(count) + " attached";
            return false;
        }
        idx = opt.index;
    }

    if (rtlsdr_open(&dev, uint32_t(idx)) < 0) {
        dev = nullptr;
        *err = "rtlsdr_open(" + std::to_string(idx) +
               ") failed: the device is attached but could not be claimed. Another "
               "process may have it, or the kernel DVB driver does.";
        return false;
    }

    const char *name = rtlsdr_get_device_name(uint32_t(idx));
    info.name = name ? name : "";
    char manuf[256] = {0}, prod[256] = {0}, serial[256] = {0};
    if (rtlsdr_get_usb_strings(dev, manuf, prod, serial) == 0) {
        info.manufacturer = manuf;
        info.product = prod;
        info.serial = serial;
    }
    info.nextrtl_strings = info.manufacturer == "zxkmm" && info.product == "nextrtl";
    if (rtlsdr_get_nextrtl_hf_lo) info.nextrtl_hf_lo_hz = rtlsdr_get_nextrtl_hf_lo(dev);
    info.tuner = map_tuner(rtlsdr_get_tuner_type(dev));
    uint32_t rtl_x = 0, tun_x = 0;
    if (rtlsdr_get_xtal_freq(dev, &rtl_x, &tun_x) == 0) {
        info.rtl_xtal_hz = rtl_x;
        info.tuner_xtal_hz = tun_x;
    }

    // The gain list, read rather than assumed. On an R820T it is 29 unevenly
    // spaced entries and it is the only honest description of what this radio's
    // gain control is.
    const int n_gains = rtlsdr_get_tuner_gains(dev, nullptr);
    if (n_gains > 0) {
        info.gains_tenth_db.assign(size_t(n_gains), 0);
        if (rtlsdr_get_tuner_gains(dev, info.gains_tenth_db.data()) != n_gains)
            info.gains_tenth_db.clear();
    }

    long long lo = 0, hi = 0;
    if (opt.tune_min_hz > 0 && opt.tune_max_hz > opt.tune_min_hz) {
        lo = opt.tune_min_hz;
        hi = opt.tune_max_hz;
    } else if (info.nextrtl_hf_lo_hz && opt.direct_sampling == 0) {
        nextrtl_range(&lo, &hi);
    } else if (!rtl_tuner_range(info.tuner, opt.direct_sampling, &lo, &hi)) {
        *err =
            "librtlsdr could not identify the tuner, so this source has no range to "
            "sweep and will not invent one. Pass --rtl-tune-min and --rtl-tune-max to "
            "say what it can reach.";
        rtlsdr_close(dev);
        dev = nullptr;
        return false;
    }
    info.tune_min_hz = lo;
    info.tune_max_hz = hi;
    tune_lo.store(lo);
    tune_hi.store(hi);
    return true;
}

void RtlSdrSource::Impl::close_device() {
    if (!dev) return;
    // Leaving the bias tee energised after exit would put 4.5 V on the antenna
    // port of a dongle nothing is driving. Turned off on the way out regardless
    // of whether we turned it on.
    rtlsdr_set_bias_tee(dev, 0);
    rtlsdr_close(dev);
    dev = nullptr;
}

bool RtlSdrSource::Impl::configure(std::string *err) {
    auto chk = [&](int r, const char *what) {
        if (r >= 0) return true;
        *err = std::string(what) + " failed (librtlsdr returned " + std::to_string(r) + ")";
        return false;
    };

    if (!chk(rtlsdr_set_sample_rate(dev, uint32_t(std::llround(opt.fs_hz))),
             "rtlsdr_set_sample_rate"))
        return false;
    const uint32_t got_fs = rtlsdr_get_sample_rate(dev);
    if (got_fs == 0) {
        *err = "the device reported a sample rate of 0 after being set";
        return false;
    }
    // The achieved rate, not the requested one: the RTL2832 divides a 28.8 MHz
    // clock, so 2.4 MSPS is exact but most other requests are not, and every
    // frequency on the trace is derived from this number.
    info.fs_achieved_hz = double(got_fs);
    fs_achieved.store(info.fs_achieved_hz);

    // -2 means "already that value", which is not a failure.
    const int r_ppm = rtlsdr_set_freq_correction(dev, opt.ppm);
    if (r_ppm < 0 && r_ppm != -2) {
        *err = "rtlsdr_set_freq_correction failed (librtlsdr returned " +
               std::to_string(r_ppm) + ")";
        return false;
    }

    if (!chk(rtlsdr_set_direct_sampling(dev, opt.direct_sampling),
             "rtlsdr_set_direct_sampling"))
        return false;
    info.direct_sampling = rtlsdr_get_direct_sampling(dev);

    if (opt.offset_tuning && !chk(rtlsdr_set_offset_tuning(dev, 1),
                                  "rtlsdr_set_offset_tuning (an E4000-only feature; "
                                  "librtlsdr refuses it on an R820T)"))
        return false;

    // 0 is automatic, which tracks the sample rate. Not readable back in this
    // librtlsdr, which is why rtl_plan_config() checks the kept band against
    // the sample rate instead of against a filter width we would be inventing.
    if (!chk(rtlsdr_set_tuner_bandwidth(dev, uint32_t(opt.bw_hz)),
             "rtlsdr_set_tuner_bandwidth"))
        return false;

    // §5's "AGC off. Ever." -- and this radio has two, so both are named.
    // rtlsdr_set_agc_mode is the RTL2832's digital AGC and is a separate switch
    // from the tuner's gain mode; turning off only the obvious one leaves the
    // other running and the amplitude wandering between segments.
    if (!chk(rtlsdr_set_agc_mode(dev, opt.rtl_agc ? 1 : 0), "rtlsdr_set_agc_mode"))
        return false;

    if (info.direct_sampling != 0) {
        // The tuner is bypassed, so it has no gain to set and librtlsdr may
        // refuse the call outright. Said in the status bar rather than silently
        // leaving the gain readout showing a number that controls nothing.
        info.gain_achieved_db = 0;
        set_detail("direct sampling: the tuner is bypassed, so the gain control does "
                   "nothing and the RTL2832's own scaling is all there is");
    } else {
        if (!chk(rtlsdr_set_tuner_gain_mode(dev, 1),
                 "rtlsdr_set_tuner_gain_mode(manual)"))
            return false;
        if (info.gains_tenth_db.empty()) {
            *err =
                "the tuner reported no gain list, so there is no manual gain to set and "
                "§5's \"AGC off, manual gain\" cannot be honoured on this device";
            return false;
        }
        double got_gain = 0;
        rtl_snap_gain(info.gains_tenth_db, opt.gain_db, &got_gain);
        if (!chk(rtlsdr_set_tuner_gain(dev, int(std::lround(got_gain * 10.0))),
                 "rtlsdr_set_tuner_gain"))
            return false;
        // What was set, not what was asked for.
        info.gain_achieved_db = got_gain;
    }

    if (!chk(rtlsdr_set_bias_tee(dev, opt.bias_tee ? 1 : 0), "rtlsdr_set_bias_tee"))
        return false;

    // Once, here -- not per hop. rtl_power resets the buffer at startup and
    // then flushes each retune with a discarded read instead, because a FIFO
    // reset per segment costs a USB control transfer on a path that is already
    // the bottleneck.
    if (!chk(rtlsdr_reset_buffer(dev), "rtlsdr_reset_buffer")) return false;
    return true;
}

// ---------------------------------------------------------------- helpers

void RtlSdrSource::Impl::set_detail(const std::string &s) {
    std::lock_guard<std::mutex> g(cfg_mu);
    detail = s;
}

void RtlSdrSource::Impl::set_fatal(const std::string &s) {
    std::lock_guard<std::mutex> g(cfg_mu);
    if (fatal.empty()) fatal = s;
    detail = s;
}

void RtlSdrSource::Impl::publish(const SourceConfig &c) {
    std::lock_guard<std::mutex> g(cfg_mu);
    cfg = c;
}

bool RtlSdrSource::Impl::request(const RtlSdrOptions &o, std::string *err) {
    {
        std::lock_guard<std::mutex> g(cfg_mu);
        if (!fatal.empty()) {
            *err = fatal;
            return false;
        }
    }

    RtlSdrOptions c = o;
    if (!rtl_validate(&c, err)) return false;

    // Planned here, on the calling thread, so an impossible request is refused
    // with the planner's own sentence while the sweep in progress carries on --
    // the same contract control.cpp has with the daemon. Letting the worker
    // discover it would mean a refused soft key took the whole source down.
    sweepd::SweepPlan probe;
    if (!plan_for(c, &probe, err)) return false;

    {
        std::lock_guard<std::mutex> g(req_mu);
        pending = c;
        has_pending = true;
    }
    restart_.store(true);
    return true;
}

// ---------------------------------------------------------------- public

RtlSdrSource::RtlSdrSource(const RtlSdrOptions &opt) : p_(new Impl(opt)) {
    p_->pending = opt;
}

RtlSdrSource::~RtlSdrSource() {
    if (p_) p_->stop();
}

bool RtlSdrSource::open(std::string *err) {
    RtlSdrOptions o = p_->opt;
    if (!rtl_validate(&o, err)) return false;
    p_->opt = o;
    p_->pending = o;

    if (!p_->open_device(err)) {
        p_->close_device();
        return false;
    }

    // The plan is checked before the worker starts, so a span the tuner cannot
    // reach is a message on stderr and a non-zero exit rather than a window
    // that opens and then shows nothing.
    if (!p_->configure(err)) {
        p_->close_device();
        return false;
    }
    sweepd::SweepPlan probe;
    if (!p_->plan_for(o, &probe, err)) {
        p_->close_device();
        return false;
    }

    info_ = p_->info;
    p_->start();
    return true;
}

bool RtlSdrSource::poll(sweepd::proto::TraceMsg *out) {
    std::lock_guard<std::mutex> g(p_->f_mu);
    if (p_->frames.empty()) return false;
    *out = std::move(p_->frames.front());
    p_->frames.pop_front();
    ++p_->delivered;
    return true;
}

SourceConfig RtlSdrSource::config() const {
    std::lock_guard<std::mutex> g(p_->cfg_mu);
    return p_->cfg;
}

SourceStatus RtlSdrSource::status() const {
    SourceStatus s;
    {
        std::lock_guard<std::mutex> g(p_->cfg_mu);
        s.connected = p_->fatal.empty();
        s.detail = p_->detail;
    }
    s.sweeps_per_s = p_->sweeps_per_s.load();
    s.hops_per_s = p_->hops_per_s.load();
    s.last_sweep_ms = p_->last_sweep_ms.load();
    {
        std::lock_guard<std::mutex> g(p_->f_mu);
        s.frames = p_->delivered;
        s.missed = p_->local_drops;
    }
    return s;
}

// ------------------------------------------------------------- setters

// Each of these reads the most recently *desired* configuration, changes one
// field and hands it back, so two soft keys pressed in quick succession do not
// undo one another.
#define WITH_PENDING(o)                              \
    RtlSdrOptions o;                                 \
    {                                                \
        std::lock_guard<std::mutex> g(p_->req_mu);   \
        o = p_->pending;                             \
    }

bool RtlSdrSource::set_span(double f_start_hz, double f_stop_hz, std::string *err) {
    WITH_PENDING(o)
    o.f_start_hz = (long long)std::llround(f_start_hz);
    o.f_stop_hz = (long long)std::llround(f_stop_hz);
    return p_->request(o, err);
}

bool RtlSdrSource::set_points(size_t n_points, std::string *err) {
    WITH_PENDING(o)
    o.n_points = n_points;
    return p_->request(o, err);
}

bool RtlSdrSource::set_detector(sweepd::Detector d, std::string *err) {
    WITH_PENDING(o)
    o.detector = d;
    return p_->request(o, err);
}

bool RtlSdrSource::set_window(sweepd::Window w, std::string *err) {
    WITH_PENDING(o)
    o.window = w;
    return p_->request(o, err);
}

bool RtlSdrSource::set_segmentation(double keep_hz, const std::string &dc_policy,
                                    std::string *err) {
    WITH_PENDING(o)
    if (keep_hz > 0) o.keep_hz = (long long)std::llround(keep_hz);
    if (!dc_policy.empty() && !sweepd::dc_policy_from_string(dc_policy, &o.dc)) {
        *err = "dc policy must be \"splice\" or \"blank\"";
        return false;
    }
    // A wider keep is a faster and dirtier sweep here just as it is on the
    // daemon, and for the same reason -- the splice fill moves further out from
    // the neighbour's LO. The request is not second-guessed; the plan checks it.
    return p_->request(o, err);
}

bool RtlSdrSource::set_lo_shift(double hz, std::string *err) {
    WITH_PENDING(o)
    o.lo_shift_hz = (long long)std::llround(hz);
    return p_->request(o, err);
}

#undef WITH_PENDING

bool RtlSdrSource::set_zero_span(double, size_t, size_t, double, std::string *err) {
    // An honest refusal rather than a second DSP path bolted on in a hurry.
    // This is the source where zero span would be easiest -- the read path is
    // already a parked-LO read loop, and sweepd::ZeroSpan is written against
    // the ZeroSpanDevice interface rather than against the AD936x, so the
    // engine, the trigger and the duty accounting would all be reused. What it
    // still needs is a second mode in this worker and its own status fields,
    // and that is a change of its own rather than a corner of this one.
    *err =
        "zero span is not implemented for the RTL-SDR source yet. It is the next "
        "thing this source should grow -- parking the LO is strictly less than "
        "sweeping it, and sweepd's zero-span engine is already behind an interface "
        "-- but it is a second mode and not a flag. Use sweepd, or the synthetic "
        "source, for zero span.";
    return false;
}

bool RtlSdrSource::set_trigger(const std::string &, double, double, double,
                               std::string *err) {
    *err = "the video trigger only exists in zero span, which this source does not have yet";
    return false;
}

bool RtlSdrSource::set_sweep_mode(std::string *err) {
    *err = "this source is always sweeping";
    return false;
}

#endif  // SWEEPSA_HAVE_RTLSDR

}  // namespace sweepui
