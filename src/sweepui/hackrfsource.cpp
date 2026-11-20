#include "hackrfsource.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>

#ifdef SWEEPSA_HAVE_HACKRF
#include "hackrf_dl.h"

#include <chrono>

#include "stitch.h"
#endif

namespace sweepui {

bool hackrf_available() {
#ifdef SWEEPSA_HAVE_HACKRF
    return true;
#else
    return false;
#endif
}

#ifndef SWEEPSA_HAVE_HACKRF

// Built without libhackrf. The option still exists and still fails with a
// sentence that says why, because "unknown argument: --hackrf" would send
// someone looking for a typo rather than for a missing dependency.
struct HackRfSource::Impl {};

HackRfSource::HackRfSource(const HackRfOptions &) {}
HackRfSource::~HackRfSource() = default;

bool HackRfSource::open(std::string *err) {
    *err =
        "this sweepui was built without libhackrf's header. Install the development "
        "package (libhackrf-dev / hackrf) and reconfigure CMake.";
    return false;
}

bool HackRfSource::poll(sweepd::proto::TraceMsg *) { return false; }
SourceConfig HackRfSource::config() const { return {}; }
SourceStatus HackRfSource::status() const { return {}; }
bool HackRfSource::take_error(std::string *) { return false; }

#define REFUSE(msg) \
    { *err = msg; return false; }

bool HackRfSource::set_span(double, double, std::string *err) REFUSE("no libhackrf")
bool HackRfSource::set_points(size_t, std::string *err) REFUSE("no libhackrf")
bool HackRfSource::set_detector(sweepd::Detector, std::string *err) REFUSE("no libhackrf")
bool HackRfSource::set_window(sweepd::Window, std::string *err) REFUSE("no libhackrf")
bool HackRfSource::set_segmentation(double, const std::string &, std::string *err)
    REFUSE("no libhackrf")
bool HackRfSource::set_zero_span(double, size_t, size_t, double, std::string *err)
    REFUSE("no libhackrf")
bool HackRfSource::set_trigger(const std::string &, double, double, double, std::string *err)
    REFUSE("no libhackrf")
bool HackRfSource::set_lo_shift(double, std::string *err) REFUSE("no libhackrf")
bool HackRfSource::set_sweep_mode(std::string *err) REFUSE("no libhackrf")

#undef REFUSE

#else  // SWEEPSA_HAVE_HACKRF

namespace {

using sweepd::Detector;
using sweepd::Window;

// The three numbers the firmware's block format fixes. Named rather than
// inlined so the arithmetic below reads as the format it is.
constexpr size_t kBlockBytes = BYTES_PER_BLOCK;   // 16384
constexpr size_t kHeaderBytes = 10;               // 0x7F 0x7F + uint64 LE Hz
constexpr size_t kMaxFftPairs = (kBlockBytes - kHeaderBytes) / 2;   // 8187

// int8 I/Q. Full scale is 128, which is what normalises a full-scale tone to
// 0 dBFS the same way SweepDevice::full_scale() does for the AD936x's 2048.
constexpr double kFullScale = 128.0;

// The HackRF One's documented tuning range, which is also what config()
// publishes as tune_min/tune_max. set_span clamps to it; without that a
// "center" entry at a wide span asks the firmware for a negative start.
constexpr long long kTuneMinHz = 1000000LL;
constexpr long long kTuneMaxHz = 6000000000LL;

double now_s() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// The loaded libhackrf; see hackrf_dl.h. Set by open_device before any call.
const hackrf_dl::Api *H = nullptr;

const char *hrf_err(int r) { return H->error_name((enum hackrf_error)r); }

size_t floor_pow2(size_t n) {
    size_t p = 1;
    while (p * 2 <= n) p *= 2;
    return p;
}

}  // namespace

// ---------------------------------------------------------------- impl

struct HackRfSource::Impl {
    explicit Impl(const HackRfOptions &o) : opt(o) {}

    HackRfOptions opt;                 // the running configuration
    HackRfInfo info;

    hackrf_device *dev = nullptr;
    bool lib_open = false;
    bool sweeping = false;

    std::atomic<bool> stop_{false};
    std::atomic<bool> restart_{false};   // a setter changed something
    std::thread worker;

    // --------------------------------------------------- block queue
    //
    // Producer is the libusb callback and it may not block; consumer is the
    // worker. Fixed capacity, drop-oldest, counted.
    struct Block {
        uint64_t freq_hz = 0;
        std::vector<int8_t> iq;        // 2 * n_fft int8, interleaved
    };

    std::mutex q_mu;
    std::condition_variable q_cv;
    std::vector<Block> ring;
    size_t q_head = 0, q_count = 0;
    size_t block_pairs = 0;            // n_fft, what the callback slices out
    std::atomic<uint64_t> dropped_blocks{0};
    std::atomic<uint64_t> torn_blocks{0};   // no 0x7F 0x7F where one was due

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
    std::deque<std::string> errors;    // under cfg_mu; see take_error()

    std::atomic<double> sweeps_per_s{0};
    std::atomic<double> hops_per_s{0};
    std::atomic<double> last_sweep_ms{0};
    std::atomic<uint64_t> sweeps{0};

    double bb_hz = 0;                  // the filter the driver actually took

    // Requests from the UI thread, applied at the top of the worker loop.
    // `pending` always holds the most recently *desired* configuration, so a
    // setter reads it, changes one field and hands it back.
    std::mutex req_mu;
    HackRfOptions pending;
    bool has_pending = false;

    // ------------------------------------------------------------ lifetime

    bool open_device(std::string *err);
    void close_device();
    void start();
    void stop();

    void run();
    bool configure(std::string *err);       // rate, filter, gains
    bool begin_sweep(std::string *err);     // init_sweep + start_rx_sweep
    void end_sweep();

    int on_transfer(hackrf_transfer *t);
    bool pop_block(Block *out);

    void set_detail(const std::string &s);
    void set_fatal(const std::string &s);
    void post_error(const std::string &s);
    void publish(const SourceConfig &c);

    bool request(const HackRfOptions &o, std::string *err);

    // One configuration's worth of sweeping: discover the grid, then stream
    // until something asks it to stop.
    void session();
    bool discover(std::vector<uint64_t> *grid);
};

// Checks a configuration against what the firmware and the block format allow,
// and rounds what can be rounded. Called before the device is touched, so a bad
// request is refused with a sentence rather than as a driver error code.
static bool validate(HackRfOptions *o, std::string *err) {
    if (o->f_stop_hz <= o->f_start_hz) {
        *err = "stop must be above start";
        return false;
    }
    if (o->fs_hz < 2e6 || o->fs_hz > 20e6) {
        *err = "the MAX2837 accepts 2-20 MSPS; " + std::to_string(o->fs_hz / 1e6) +
               " MSPS is outside it";
        return false;
    }
    if (o->n_fft < 64 || o->n_fft > floor_pow2(kMaxFftPairs)) {
        *err = "FFT size must be a power of two between 64 and " +
               std::to_string(floor_pow2(kMaxFftPairs)) +
               "; one tuning delivers a single " + std::to_string(kBlockBytes) +
               "-byte block and 10 bytes of that are the frequency header, so "
               "there is no more signal to transform. Lower the sample rate for "
               "finer RBW instead.";
        return false;
    }
    if (o->n_fft & (o->n_fft - 1)) {
        *err = "FFT size must be a power of two";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------- discovery

// Collects one sweep's worth of block header frequencies. See the header note:
// the layout is measured here rather than predicted, so a firmware that steps
// differently produces a correct grid instead of a plausible wrong one.
//
// False means it gave up -- stopped, reconfigured, or the device stopped
// producing blocks.
bool HackRfSource::Impl::discover(std::vector<uint64_t> *grid) {
    grid->clear();

    const uint64_t first_expected = uint64_t(opt.f_start_hz / 1000000LL) * 1000000ULL;
    bool synced = false, have_prev = false;
    uint64_t prev = 0;
    Block b;

    // Bounded so a device that goes quiet -- unplugged, or wedged -- reports
    // that rather than hanging the UI on a source that will never produce.
    const double deadline = now_s() + 20.0;

    while (!stop_.load() && !restart_.load()) {
        if (now_s() > deadline) {
            set_fatal("the HackRF stopped producing blocks while the sweep grid was "
                      "being measured");
            return false;
        }
        if (!pop_block(&b)) continue;

        // Within a sweep the header frequency strictly increases, so anything
        // that does not is the wrap. `<=` rather than `<` so a range small
        // enough to be one block per sweep is still detected.
        const bool wrap = have_prev && b.freq_hz <= prev;
        prev = b.freq_hz;
        have_prev = true;

        if (!synced) {
            // Normally the firmware begins at the range start and the first
            // block is already the sweep's first; the wrap test is the fallback
            // for anything that does not.
            if (b.freq_hz == first_expected || wrap) {
                synced = true;
                grid->push_back(b.freq_hz);
            }
            continue;
        }
        if (wrap) return true;        // the grid is complete
        grid->push_back(b.freq_hz);

        if (grid->size() > 100000) {
            set_fatal("the HackRF's sweep never wrapped; the block header frequencies "
                      "are not the ascending grid this source is written against");
            return false;
        }
    }
    return false;
}

// ---------------------------------------------------------------- session

void HackRfSource::Impl::session() {
    std::vector<uint64_t> grid;
    set_detail("measuring the sweep grid");
    if (!discover(&grid)) return;
    if (grid.empty()) return;

    const size_t n = opt.n_fft;
    const double fs = opt.fs_hz;

    // The kept geometry, derived from the sample rate and nothing else. A block
    // whose header reads F was tuned to F + 3fs/8 and keeps the two quarter
    // bands either side of its own LO leakage -- see the header.
    const double lo_off = fs * 3.0 / 8.0;
    const double keep_w = fs * 3.0 / 4.0;
    const double blank_a = fs / 4.0;
    const double blank_b = fs / 2.0;

    const double covered_lo = double(grid.front());
    const double covered_hi = double(grid.back()) + keep_w;

    const double out_lo = std::max(covered_lo, double(opt.f_start_hz));
    const double out_hi = std::min(covered_hi, double(opt.f_stop_hz));
    if (out_hi <= out_lo) {
        set_fatal("the sweep covers nothing inside the requested range");
        return;
    }

    sweepd::WindowFunction win = sweepd::make_window(opt.window, n);
    sweepd::Fft fft(n);

    const double rbw = sweepd::rbw_hz(win.enbw, fs, n);
    const double span = out_hi - out_lo;
    size_t n_points = opt.n_points;
    if (n_points == 0) {
        // Match the resolution actually measured rather than inventing one --
        // the same rule derive_sweep() uses in the daemon.
        const double want = std::ceil(span / rbw);
        const double cap = double(opt.max_points ? opt.max_points : 1);
        n_points = size_t(std::max(1.0, std::min(want, cap)));
    }
    const double bin_hz = span / double(n_points);

    // ------------------------------------------------------------- publish
    {
        SourceConfig c;
        c.f_start_hz = (long long)std::llround(out_lo);
        c.f_stop_hz = (long long)std::llround(out_hi);
        c.tune_min_hz = kTuneMinHz;
        c.tune_max_hz = kTuneMaxHz;
        c.n_points = n_points;
        c.rbw_hz = rbw;
        c.bin_hz = bin_hz;
        c.n_segments = uint32_t(grid.size());
        c.gain_db = float(opt.lna_db + opt.vga_db + (opt.amp ? 11 : 0));
        c.covered_lo_hz = covered_lo;
        c.covered_hi_hz = covered_hi;
        // seg_first_lo_hz / seg_step_hz / keep_hz / blank_hz stay zero on
        // purpose: the interleaved sweep's LOs are not a uniform grid, so §7's
        // overlay reports itself unavailable rather than drawing an invented
        // one. See the header.
        c.splice = true;
        c.window = opt.window;
        c.detector = opt.detector;
        c.uncal = true;
        c.synthetic = false;
        c.reconfiguring = false;

        char buf[320];
        std::snprintf(buf, sizeof buf,
                      "%s %s, fw %s -- %.3f-%.3f MHz swept as %zu tunings of "
                      "%.1f MSPS (%.1f MHz baseband filter), LNA %d dB + VGA %d dB%s",
                      info.board.empty() ? "HackRF" : info.board.c_str(),
                      info.serial.c_str(), info.firmware.c_str(), covered_lo / 1e6,
                      covered_hi / 1e6, grid.size(), fs / 1e6, bb_hz / 1e6, opt.lna_db,
                      opt.vga_db, opt.amp ? " + 11 dB RF amp" : "");
        c.description = buf;
        publish(c);
    }
    set_detail("");

    // ------------------------------------------------------------- stream
    sweepd::Stitcher stitch;
    stitch.reset(out_lo, out_hi, n_points, opt.detector);

    std::vector<float> iqf(2 * n);
    std::vector<int16_t> iq16(2 * n);
    std::vector<float> power(n);
    std::vector<uint32_t> overload(grid.size(), 0);

    const int16_t threshold = (int16_t)std::lround(kFullScale * opt.overload_frac);
    const float inv_fs = float(1.0 / kFullScale);

    uint64_t seq = 0;
    size_t blocks_this_sweep = 0;
    uint64_t off_grid = 0;
    bool have_prev = false;
    uint64_t prev = 0;
    double t_sweep_start = now_s();

    // Rate is measured from the end of the first completed sweep, so the
    // discovery sweep's start-up cost does not drag the average down forever.
    double t_rate_base = 0;
    // Counted per session, not from the lifetime `sweeps` total: keyed on that,
    // the base was only ever set by the first session, and after a retune the
    // readout averaged every earlier span (and the gaps between) into this one.
    uint64_t sweeps_at_base = 0, blocks_at_base = 0, blocks_total = 0;
    uint64_t session_sweeps = 0;

    Block b;
    while (!stop_.load() && !restart_.load()) {
        if (!pop_block(&b)) continue;

        const bool wrap = have_prev && b.freq_hz <= prev;
        prev = b.freq_hz;
        have_prev = true;

        if (wrap && blocks_this_sweep) {
            sweepd::proto::TraceMsg m;
            m.seq = seq++;
            m.t_mono_s = now_s();
            m.f_start_hz = (long long)std::llround(out_lo);
            m.f_stop_hz = (long long)std::llround(out_hi);
            m.n_segments = uint32_t(grid.size());
            m.rbw_hz = rbw;
            m.bin_hz = bin_hz;
            m.sweep_seconds = m.t_mono_s - t_sweep_start;
            m.gain_db = float(opt.lna_db + opt.vga_db + (opt.amp ? 11 : 0));
            m.detector = opt.detector;
            m.window = opt.window;
            m.x_kind = sweepd::proto::XKind::Frequency;

            m.db.resize(n_points);
            stitch.to_db(m.db.data());

            m.overload.assign((grid.size() + 7) / 8, 0);
            for (size_t i = 0; i < overload.size(); ++i) {
                if (!overload[i]) continue;
                sweepd::proto::set_overload_bit(&m.overload, grid.size(), i);
                ++m.n_overloaded_segments;
            }

            const double t_end = m.t_mono_s;
            last_sweep_ms.store(m.sweep_seconds * 1e3, std::memory_order_relaxed);
            sweeps.fetch_add(1, std::memory_order_relaxed);
            const uint64_t done = ++session_sweeps;
            if (done == 1) {
                t_rate_base = t_end;
                sweeps_at_base = done;
                blocks_at_base = blocks_total;
            } else if (t_end > t_rate_base) {
                const double dt = t_end - t_rate_base;
                sweeps_per_s.store(double(done - sweeps_at_base) / dt,
                                   std::memory_order_relaxed);
                hops_per_s.store(double(blocks_total - blocks_at_base) / dt,
                                 std::memory_order_relaxed);
            }

            {
                std::lock_guard<std::mutex> g(f_mu);
                if (frames.size() >= opt.frame_queue_depth) {
                    frames.pop_front();
                    ++local_drops;
                }
                frames.push_back(std::move(m));
            }

            stitch.clear();
            std::fill(overload.begin(), overload.end(), 0u);
            blocks_this_sweep = 0;
            t_sweep_start = t_end;

            const uint64_t dropped = dropped_blocks.load(std::memory_order_relaxed);
            const uint64_t torn = torn_blocks.load(std::memory_order_relaxed);
            if (dropped || torn || off_grid) {
                char buf[192];
                std::snprintf(buf, sizeof buf,
                              "%llu blocks dropped, %llu headerless, %llu off-grid",
                              (unsigned long long)dropped, (unsigned long long)torn,
                              (unsigned long long)off_grid);
                set_detail(buf);
            }
        }

        // Which tuning this is. A header naming a frequency the discovery sweep
        // never saw is not placed by arithmetic: the grid is what the hardware
        // said it was, and a block outside it means the two have diverged.
        const auto it = std::lower_bound(grid.begin(), grid.end(), b.freq_hz);
        if (it == grid.end() || *it != b.freq_hz) {
            ++off_grid;
            continue;
        }
        const size_t idx = size_t(it - grid.begin());

        for (size_t k = 0; k < 2 * n; ++k) {
            iq16[k] = int16_t(b.iq[k]);
            iqf[k] = float(b.iq[k]) * inv_fs;
        }
        overload[idx] = uint32_t(sweepd::count_overload(iq16.data(), n, threshold));

        fft.power_spectrum(iqf.data(), win, power.data());

        sweepd::StitchBand band;
        const double f = double(b.freq_hz);
        band.lo_hz = f + lo_off;
        band.keep_lo_hz = f;
        band.keep_hi_hz = f + keep_w;
        band.blank_lo_hz = f + blank_a;
        band.blank_hi_hz = f + blank_b;
        stitch.add(power.data(), n, fs, band);

        ++blocks_this_sweep;
        ++blocks_total;
    }
}

// ---------------------------------------------------------------- worker

void HackRfSource::Impl::run() {
    std::string err;
    // The configuration that last started streaming. A request the device
    // refuses falls back to it, and the source goes fatal only if that fails
    // too. Before this, any refusal (a negative start, for one) ended the
    // worker: the radio stopped, the plot froze on its last trace, and every
    // later request was refused with the latched error.
    HackRfOptions last_good = opt;
    bool have_good = false;
    bool retrying = false;

    while (!stop_.load()) {
        if (!retrying) {
            std::lock_guard<std::mutex> g(req_mu);
            if (has_pending) {
                opt = pending;
                has_pending = false;
            }
        }
        retrying = false;
        restart_.store(false);

        {
            std::lock_guard<std::mutex> g(cfg_mu);
            cfg.reconfiguring = true;
        }

        block_pairs = opt.n_fft;
        {
            std::lock_guard<std::mutex> g(q_mu);
            ring.assign(std::max<size_t>(8, opt.queue_blocks), Block{});
            q_head = q_count = 0;
        }
        dropped_blocks.store(0);
        torn_blocks.store(0);

        if (!configure(&err) || !begin_sweep(&err)) {
            end_sweep();
            if (!have_good) {
                set_fatal(err);
                return;
            }
            post_error("HackRF refused the new configuration (" + err +
                       "); back on the last one that ran");
            opt = last_good;
            have_good = false;          // if this fails too, the device is the problem
            retrying = true;
            {
                // A setter builds on `pending`, so it has to describe what is
                // running now -- unless a newer request already replaced it.
                std::lock_guard<std::mutex> g(req_mu);
                if (!has_pending) pending = last_good;
            }
            continue;
        }
        last_good = opt;
        have_good = true;

        session();
        end_sweep();
    }
}

void HackRfSource::Impl::start() {
    worker = std::thread([this] { run(); });
}

void HackRfSource::Impl::stop() {
    stop_.store(true);
    q_cv.notify_all();
    if (worker.joinable()) worker.join();
    close_device();
}

// ---------------------------------------------------------------- callback

int HackRfSource::Impl::on_transfer(hackrf_transfer *t) {
    if (stop_.load(std::memory_order_relaxed)) return 1;

    const size_t pairs = block_pairs;
    const size_t want = 2 * pairs;
    const size_t n_blocks = size_t(t->valid_length) / kBlockBytes;
    const uint8_t *base = (const uint8_t *)t->buffer;

    std::lock_guard<std::mutex> g(q_mu);
    for (size_t b = 0; b < n_blocks; ++b) {
        const uint8_t *blk = base + b * kBlockBytes;

        // A block without the marker is a tuning whose header was lost. It is
        // counted rather than guessed at: placing its samples at the previous
        // block's frequency would put real signal at the wrong place on the
        // trace, which is indistinguishable from a spur.
        if (blk[0] != 0x7f || blk[1] != 0x7f) {
            torn_blocks.fetch_add(1, std::memory_order_relaxed);
            continue;
        }

        uint64_t f = 0;
        for (int k = 0; k < 8; ++k) f |= uint64_t(blk[2 + k]) << (8 * k);

        if (q_count == ring.size()) {
            // Drop the oldest: the newest block is the one whose partner is
            // still in flight, and keeping the sweep's tail intact loses less
            // than keeping its head.
            q_head = (q_head + 1) % ring.size();
            --q_count;
            dropped_blocks.fetch_add(1, std::memory_order_relaxed);
        }
        Block &slot = ring[(q_head + q_count) % ring.size()];
        slot.freq_hz = f;
        slot.iq.resize(want);
        // The transform reads the END of the block: the header displaces the
        // first 10 bytes, and hackrf_sweep takes the tail for the same reason.
        std::memcpy(slot.iq.data(), blk + kBlockBytes - want, want);
        ++q_count;
    }
    q_cv.notify_one();
    return 0;
}

bool HackRfSource::Impl::pop_block(Block *out) {
    std::unique_lock<std::mutex> lk(q_mu);
    q_cv.wait_for(lk, std::chrono::milliseconds(200), [&] {
        return q_count > 0 || stop_.load() || restart_.load();
    });
    if (q_count == 0) return false;
    *out = std::move(ring[q_head]);
    q_head = (q_head + 1) % ring.size();
    --q_count;
    return true;
}

// ---------------------------------------------------------------- device

bool HackRfSource::Impl::open_device(std::string *err) {
    if (!(H = hackrf_dl::load(err))) return false;
    int r = H->init();
    if (r != HACKRF_SUCCESS) {
        *err = std::string("hackrf_init: ") + hrf_err(r);
        return false;
    }
    lib_open = true;

    r = opt.serial.empty() ? H->open(&dev)
                           : H->open_by_serial(opt.serial.c_str(), &dev);
    if (r != HACKRF_SUCCESS) {
        dev = nullptr;
        *err = std::string("hackrf_open: ") + hrf_err(r) +
#ifdef __linux__
               " (is the device attached, and do you have permission? the udev "
               "rule from the hackrf package is what grants it)";
#else
               " (is the device attached?)";
#endif
        return false;
    }

    read_partid_serialno_t sn;
    if (H->board_partid_serialno_read(dev, &sn) == HACKRF_SUCCESS) {
        char buf[64];
        std::snprintf(buf, sizeof buf, "%08x%08x%08x%08x", sn.serial_no[0],
                      sn.serial_no[1], sn.serial_no[2], sn.serial_no[3]);
        info.serial = buf;
    }
    uint8_t board = 0;
    if (H->board_id_read(dev, &board) == HACKRF_SUCCESS)
        info.board = H->board_id_name((enum hackrf_board_id)board);
    char ver[256] = {0};
    if (H->version_string_read(dev, ver, sizeof ver - 1) == HACKRF_SUCCESS)
        info.firmware = ver;
    uint16_t api = 0;
    if (H->usb_api_version_read(dev, &api) == HACKRF_SUCCESS) info.usb_api = api;

    // hackrf_start_rx_sweep needs USB API 0x0104. Checked rather than
    // discovered as a confusing error from the first sweep.
    if (info.usb_api && info.usb_api < 0x0104) {
        char buf[160];
        std::snprintf(buf, sizeof buf,
                      "firmware USB API 0x%04x is too old for sweep mode, which "
                      "needs 0x0104 or above; update the HackRF firmware",
                      info.usb_api);
        *err = buf;
        H->close(dev);
        dev = nullptr;
        return false;
    }
    return true;
}

void HackRfSource::Impl::close_device() {
    if (dev) {
        end_sweep();
        H->close(dev);
        dev = nullptr;
    }
    if (lib_open) {
        H->exit();
        lib_open = false;
    }
}

bool HackRfSource::Impl::configure(std::string *err) {
    auto chk = [&](int r, const char *what) {
        if (r == HACKRF_SUCCESS) return true;
        *err = std::string(what) + ": " + hrf_err(r);
        return false;
    };

    if (!chk(H->set_sample_rate(dev, opt.fs_hz), "set_sample_rate")) return false;

    const uint32_t bw = opt.bb_filter_hz > 0
                            ? H->compute_baseband_filter_bw(uint32_t(opt.bb_filter_hz))
                            : H->compute_baseband_filter_bw(uint32_t(opt.fs_hz));
    if (!chk(H->set_baseband_filter_bandwidth(dev, bw), "set_baseband_filter_bandwidth"))
        return false;

    // The driver rejects a gain that is not on its step grid, so they are
    // rounded here and the rounded value is what gets reported -- a gain
    // readout that shows what was asked for rather than what was set is the
    // §7 failure in miniature.
    opt.lna_db = std::min(40, std::max(0, (opt.lna_db / 8) * 8));
    opt.vga_db = std::min(62, std::max(0, (opt.vga_db / 2) * 2));
    if (!chk(H->set_lna_gain(dev, uint32_t(opt.lna_db)), "set_lna_gain")) return false;
    if (!chk(H->set_vga_gain(dev, uint32_t(opt.vga_db)), "set_vga_gain")) return false;
    if (!chk(H->set_amp_enable(dev, opt.amp ? 1 : 0), "set_amp_enable")) return false;

    bb_hz = double(bw);
    return true;
}

bool HackRfSource::Impl::begin_sweep(std::string *err) {
    // The firmware's range list is uint16 MHz, so the swept range is whole
    // megahertz. Rounding outward rather than to nearest: a sweep that does not
    // reach the frequency that was asked for is worse than one that overshoots
    // it, and the output grid is clipped back to the request afterwards.
    const long long a_mhz = opt.f_start_hz / 1000000LL;
    long long b_mhz = (opt.f_stop_hz + 999999LL) / 1000000LL;
    if (b_mhz <= a_mhz) b_mhz = a_mhz + 1;
    if (a_mhz < 0 || b_mhz > 65535) {
        *err = "the firmware's sweep range list is uint16 MHz, so the span must lie "
               "inside 0-65535 MHz";
        return false;
    }

    const uint16_t ranges[2] = {uint16_t(a_mhz), uint16_t(b_mhz)};
    const uint32_t step = uint32_t(std::llround(opt.fs_hz));
    const uint32_t offset = uint32_t(std::llround(opt.fs_hz * 3.0 / 8.0));

    int r = H->init_sweep(dev, ranges, 1, uint32_t(kBlockBytes), step, offset,
                              INTERLEAVED);
    if (r != HACKRF_SUCCESS) {
        *err = std::string("hackrf_init_sweep: ") + hrf_err(r);
        return false;
    }
    r = H->start_rx_sweep(
        dev, [](hackrf_transfer *t) { return ((Impl *)t->rx_ctx)->on_transfer(t); }, this);
    if (r != HACKRF_SUCCESS) {
        *err = std::string("hackrf_start_rx_sweep: ") + hrf_err(r);
        return false;
    }
    sweeping = true;
    return true;
}

void HackRfSource::Impl::end_sweep() {
    if (!sweeping) return;
    H->stop_rx(dev);
    sweeping = false;
    std::lock_guard<std::mutex> g(q_mu);
    q_head = q_count = 0;
}

// ---------------------------------------------------------------- helpers

void HackRfSource::Impl::set_detail(const std::string &s) {
    std::lock_guard<std::mutex> g(cfg_mu);
    detail = s;
}

void HackRfSource::Impl::set_fatal(const std::string &s) {
    std::lock_guard<std::mutex> g(cfg_mu);
    if (fatal.empty()) fatal = s;
    detail = s;
}

void HackRfSource::Impl::post_error(const std::string &s) {
    std::lock_guard<std::mutex> g(cfg_mu);
    errors.push_back(s);
    while (errors.size() > 16) errors.pop_front();
}

void HackRfSource::Impl::publish(const SourceConfig &c) {
    std::lock_guard<std::mutex> g(cfg_mu);
    cfg = c;
}

bool HackRfSource::Impl::request(const HackRfOptions &o, std::string *err) {
    {
        std::lock_guard<std::mutex> g(cfg_mu);
        if (!fatal.empty()) {
            *err = fatal;
            return false;
        }
    }
    {
        std::lock_guard<std::mutex> g(req_mu);
        pending = o;
        has_pending = true;
    }
    restart_.store(true);
    q_cv.notify_all();
    return true;
}

// ---------------------------------------------------------------- public

HackRfSource::HackRfSource(const HackRfOptions &opt) : p_(new Impl(opt)) {
    p_->pending = opt;
}

HackRfSource::~HackRfSource() {
    if (p_) p_->stop();
}

bool HackRfSource::open(std::string *err) {
    HackRfOptions o = p_->opt;
    if (!validate(&o, err)) return false;
    p_->opt = o;
    p_->pending = o;

    if (!p_->open_device(err)) {
        p_->close_device();
        return false;
    }
    info_ = p_->info;
    p_->start();
    return true;
}

bool HackRfSource::poll(sweepd::proto::TraceMsg *out) {
    std::lock_guard<std::mutex> g(p_->f_mu);
    if (p_->frames.empty()) return false;
    *out = std::move(p_->frames.front());
    p_->frames.pop_front();
    ++p_->delivered;
    return true;
}

SourceConfig HackRfSource::config() const {
    std::lock_guard<std::mutex> g(p_->cfg_mu);
    return p_->cfg;
}

bool HackRfSource::take_error(std::string *msg) {
    std::lock_guard<std::mutex> g(p_->cfg_mu);
    if (p_->errors.empty()) return false;
    *msg = std::move(p_->errors.front());
    p_->errors.pop_front();
    return true;
}

SourceStatus HackRfSource::status() const {
    SourceStatus s;
    // Connected means the worker is alive and the device has not failed. A
    // HackRF cannot "disconnect" the way a TCP daemon can, but it can be
    // unplugged, and that must not read as a live display of a stale trace.
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

bool HackRfSource::set_span(double f_start_hz, double f_stop_hz, std::string *err) {
    HackRfOptions o;
    {
        std::lock_guard<std::mutex> g(p_->req_mu);
        o = p_->pending;
    }
    if (!(f_stop_hz > f_start_hz)) {
        *err = "stop must be above start";
        return false;
    }
    // Clamped to what the radio tunes, the way sweepd's plan clamps. The view
    // routinely asks for more: "center 473m" at full span, or "zoom out" near
    // a band edge, both want a start below zero.
    if (f_stop_hz <= double(kTuneMinHz) || f_start_hz >= double(kTuneMaxHz)) {
        *err = "the span lies wholly outside the HackRF's " +
               std::to_string(kTuneMinHz / 1000000LL) + "-" +
               std::to_string(kTuneMaxHz / 1000000LL) + " MHz tuning range";
        return false;
    }
    o.f_start_hz = std::max(kTuneMinHz, (long long)std::llround(f_start_hz));
    o.f_stop_hz = std::min(kTuneMaxHz, (long long)std::llround(f_stop_hz));

    // The firmware's range list is whole megahertz, so a span narrower than
    // that cannot be expressed. Said plainly rather than silently widened.
    if (o.f_stop_hz / 1000000LL == o.f_start_hz / 1000000LL &&
        o.f_stop_hz - o.f_start_hz < 1000000LL) {
        *err = "the firmware sweeps in whole megahertz, so the narrowest span this "
               "radio can be asked for is 1 MHz; for anything finer it would have to "
               "park, which is zero span";
        return false;
    }
    if (!validate(&o, err)) return false;
    return p_->request(o, err);
}

bool HackRfSource::set_points(size_t n_points, std::string *err) {
    HackRfOptions o;
    {
        std::lock_guard<std::mutex> g(p_->req_mu);
        o = p_->pending;
    }
    o.n_points = n_points;
    if (n_points > o.max_points) {
        *err = "more points than max_points (" + std::to_string(o.max_points) + ")";
        return false;
    }
    return p_->request(o, err);
}

bool HackRfSource::set_detector(sweepd::Detector d, std::string *err) {
    HackRfOptions o;
    {
        std::lock_guard<std::mutex> g(p_->req_mu);
        o = p_->pending;
    }
    o.detector = d;
    return p_->request(o, err);
}

bool HackRfSource::set_window(sweepd::Window w, std::string *err) {
    HackRfOptions o;
    {
        std::lock_guard<std::mutex> g(p_->req_mu);
        o = p_->pending;
    }
    o.window = w;
    return p_->request(o, err);
}

bool HackRfSource::set_segmentation(double keep_hz, const std::string &dc_policy,
                                    std::string *err) {
    (void)keep_hz;
    (void)dc_policy;
    *err =
        "on a HackRF the segment width is not a free parameter: each tuning keeps "
        "fs/2 either side of a DC hole fs/4 wide, so the sample rate sets both. "
        "There is no splice/blank choice either -- the interleaved partner tuning "
        "always fills the hole. Change the sample rate instead.";
    return false;
}

bool HackRfSource::set_zero_span(double, size_t, size_t, double, std::string *err) {
    // Honest refusal rather than a second DSP path bolted on in a hurry. Zero
    // span needs hackrf_start_rx with the LO parked -- a different streaming
    // mode from hackrf_start_rx_sweep -- plus zspan.h's offset handling and the
    // video trigger. It is a clean addition and it is not this change.
    *err =
        "zero span is not implemented for the HackRF source yet: it needs the "
        "non-sweep RX path, which is a separate streaming mode. Use sweepd, or "
        "the synthetic source, for zero span.";
    return false;
}

bool HackRfSource::set_trigger(const std::string &, double, double, double,
                               std::string *err) {
    *err = "the video trigger only exists in zero span, which this source does not have yet";
    return false;
}

bool HackRfSource::set_lo_shift(double, std::string *err) {
    *err =
        "§7's image check is not implemented for the HackRF source yet. The LO grid "
        "can only be phased in whole megahertz here -- the firmware's range list is "
        "uint16 MHz -- which moves the covered range as well as the images, and the "
        "check has not been rewritten to compare two passes on different grids.";
    return false;
}

bool HackRfSource::set_sweep_mode(std::string *err) {
    *err = "this source is always sweeping";
    return false;
}

#endif  // SWEEPSA_HAVE_HACKRF

}  // namespace sweepui
