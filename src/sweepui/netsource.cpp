#include "netsource.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "freq.h"

// ----------------------------------------------------------------- sockets
//
// §6 requires sweepui to build on Windows and src/sweepd/server.cpp is POSIX
// only, so the client half carries its own shim. It is deliberately the
// smallest one that works: connect, recv, send, close, and a bounded connect.

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
using socklen_t_compat = int;
using sw_usec_t = long;          // Windows' timeval has a plain long tv_usec
#define SW_INVALID INVALID_SOCKET
#define sw_close closesocket
#define sw_errno WSAGetLastError()
#define SW_EWOULDBLOCK WSAEWOULDBLOCK
#define SW_EINPROGRESS WSAEWOULDBLOCK
#define SW_EINTR WSAEINTR
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
using socklen_t_compat = socklen_t;
using sw_usec_t = suseconds_t;
#define SW_INVALID (-1)
#define sw_close ::close
#define sw_errno errno
#define SW_EWOULDBLOCK EWOULDBLOCK
#define SW_EINPROGRESS EINPROGRESS
#define SW_EINTR EINTR
#endif

using namespace sweepd;
using namespace sweepd::proto;

namespace sweepui {
namespace {

double mono_s() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

std::string sock_error() {
#ifdef _WIN32
    char buf[64];
    std::snprintf(buf, sizeof buf, "winsock error %d", WSAGetLastError());
    return buf;
#else
    return std::strerror(errno);
#endif
}

void winsock_once() {
#ifdef _WIN32
    static bool done = [] {
        WSADATA wsa;
        return WSAStartup(MAKEWORD(2, 2), &wsa) == 0;
    }();
    (void)done;
#endif
}

void set_blocking(intptr_t fd, bool blocking) {
#ifdef _WIN32
    u_long mode = blocking ? 0 : 1;
    ioctlsocket((SOCKET)fd, FIONBIO, &mode);
#else
    const int f = ::fcntl(int(fd), F_GETFL, 0);
    ::fcntl(int(fd), F_SETFL, blocking ? (f & ~O_NONBLOCK) : (f | O_NONBLOCK));
#endif
}

void set_timeouts(intptr_t fd, double recv_s, double send_s) {
#ifdef _WIN32
    DWORD r = DWORD(recv_s * 1000.0), s = DWORD(send_s * 1000.0);
    ::setsockopt((SOCKET)fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&r, sizeof r);
    ::setsockopt((SOCKET)fd, SOL_SOCKET, SO_SNDTIMEO, (const char *)&s, sizeof s);
#else
    timeval r{time_t(recv_s), suseconds_t((recv_s - std::floor(recv_s)) * 1e6)};
    timeval s{time_t(send_s), suseconds_t((send_s - std::floor(send_s)) * 1e6)};
    ::setsockopt(int(fd), SOL_SOCKET, SO_RCVTIMEO, &r, sizeof r);
    ::setsockopt(int(fd), SOL_SOCKET, SO_SNDTIMEO, &s, sizeof s);
#endif
}

// Blocking send of the whole buffer. Control messages are tens of bytes and the
// daemon reads its control channel continuously, but the send timeout above
// means even a wedged peer costs a bounded stall rather than a hung UI.
bool send_all(intptr_t fd, const uint8_t *p, size_t n) {
    while (n) {
#ifdef _WIN32
        const int k = ::send((SOCKET)fd, (const char *)p, int(n), 0);
#else
        const ssize_t k = ::send(int(fd), p, n, MSG_NOSIGNAL);
#endif
        if (k > 0) {
            p += k;
            n -= size_t(k);
            continue;
        }
        if (k < 0 && sw_errno == SW_EINTR) continue;
        return false;
    }
    return true;
}

}  // namespace

// ------------------------------------------------------------------ lifetime

NetSource::NetSource(const NetOptions &opt) : opt_(opt) {
    winsock_once();
    cfg_.synthetic = false;      // there is a radio at the other end of this
    cfg_.uncal = true;           // until the daemon says otherwise (§7)
    detail_ = "connecting to " + opt_.host + ":" + std::to_string(opt_.port);
    cfg_.description = detail_;
    thread_ = std::thread([this] { run(); });
}

NetSource::~NetSource() {
    stop_.store(true);
    // Wake anything waiting on a reply so the setter's thread is not left in
    // wait_for() while this object is being destroyed.
    resolve(false, "sweepui is shutting down");
    if (thread_.joinable()) thread_.join();
}

// ------------------------------------------------------------------ thread

void NetSource::run() {
    while (!stop_.load()) {
        std::string err;
        const intptr_t fd = dial(&err);
        if (fd < 0) {
            connected_.store(false);
            set_detail("cannot reach " + opt_.host + ":" + std::to_string(opt_.port) +
                       " -- " + err);
            // Sleep in slices so the destructor is not made to wait a whole
            // backoff for a thread that already knows it should stop.
            const double until = mono_s() + opt_.reconnect_backoff_s;
            while (!stop_.load() && mono_s() < until)
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        connected_.store(true);
        set_detail("connected to " + opt_.host + ":" + std::to_string(opt_.port));
        serve(fd);
        sw_close((decltype(SW_INVALID))fd);

        connected_.store(false);
        // A disconnect can strand a setter that is waiting for an answer.
        resolve(false, "sweepd closed the connection before answering");
        if (!stop_.load()) set_detail("sweepd went away -- reconnecting");
    }
    connected_.store(false);
}

intptr_t NetSource::dial(std::string *err) {
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    addrinfo *res = nullptr;
    const std::string svc = std::to_string(opt_.port);
    const int rc = ::getaddrinfo(opt_.host.c_str(), svc.c_str(), &hints, &res);
    if (rc != 0) {
        *err = std::string("resolve: ") + ::gai_strerror(rc);
        return -1;
    }

    intptr_t fd = -1;
    for (addrinfo *a = res; a && !stop_.load(); a = a->ai_next) {
        const intptr_t s = (intptr_t)::socket(a->ai_family, a->ai_socktype, a->ai_protocol);
        if (s == (intptr_t)SW_INVALID) continue;

        // Non-blocking connect with a bounded wait: a blocking connect() to an
        // unreachable host sits for over a minute, and the destructor joins
        // this thread, so closing the window would appear to hang.
        set_blocking(s, false);
        int cr;
#ifdef _WIN32
        cr = ::connect((SOCKET)s, a->ai_addr, (int)a->ai_addrlen);
#else
        cr = ::connect(int(s), a->ai_addr, a->ai_addrlen);
#endif
        bool ok = (cr == 0);
        if (!ok && (sw_errno == SW_EINPROGRESS || sw_errno == SW_EWOULDBLOCK)) {
            fd_set wr;
            FD_ZERO(&wr);
            FD_SET((decltype(SW_INVALID))s, &wr);
            timeval tv{time_t(opt_.connect_timeout_s),
                       sw_usec_t((opt_.connect_timeout_s -
                                  std::floor(opt_.connect_timeout_s)) * 1e6)};
            const int n = ::select(int(s) + 1, nullptr, &wr, nullptr, &tv);
            if (n > 0) {
                int soerr = 0;
                socklen_t_compat len = sizeof soerr;
                ::getsockopt((decltype(SW_INVALID))s, SOL_SOCKET, SO_ERROR,
                             (char *)&soerr, &len);
                ok = (soerr == 0);
                if (!ok) {
#ifdef _WIN32
                    char b[48];
                    std::snprintf(b, sizeof b, "winsock error %d", soerr);
                    *err = b;
#else
                    *err = std::strerror(soerr);
#endif
                }
            } else if (n == 0) {
                *err = "timed out";
            } else {
                *err = sock_error();
            }
        } else if (!ok) {
            *err = sock_error();
        }

        if (!ok) {
            sw_close((decltype(SW_INVALID))s);
            continue;
        }
        set_blocking(s, true);
        fd = s;
        break;
    }
    ::freeaddrinfo(res);

    if (fd < 0 && err->empty()) *err = "no route";
    if (fd >= 0) {
        const int one = 1;
        // Control requests are tiny and latency matters more than packing.
        ::setsockopt((decltype(SW_INVALID))fd, IPPROTO_TCP, TCP_NODELAY,
                     (const char *)&one, sizeof one);
        // The recv side is paced by select() in serve(), not by a timeout
        // here: Windows reports an expired SO_RCVTIMEO as WSAETIMEDOUT and
        // documents the socket as indeterminate after it. This one is only a
        // backstop so a recv() can never hang the thread.
        set_timeouts(fd, 5.0, 0.5);
    }
    return fd;
}

void NetSource::serve(intptr_t fd) {
    Reader reader;
    std::vector<uint8_t> rx(1 << 16);
    double t_next_status = 0;      // ask immediately on connect

    // The greeting (a `config` control message) arrives unprompted, but ask as
    // well: a daemon started before this build, or one whose greeting was lost
    // to a half-open connection, should still be able to draw an axis.
    queue_out(JsonWriter().kv("cmd", "config").done());

    while (!stop_.load()) {
        if (mono_s() >= t_next_status) {
            queue_out(JsonWriter().kv("cmd", "status").done());
            t_next_status = mono_s() + opt_.status_interval_s;
        }

        // Drain the outbox. Anything the UI thread queued goes out here, on
        // this thread, so a setter never touches the socket itself.
        for (;;) {
            std::string msg;
            {
                std::lock_guard<std::mutex> lk(out_mu_);
                if (outbox_.empty()) break;
                msg = std::move(outbox_.front());
                outbox_.pop_front();
            }
            std::vector<uint8_t> frame;
            encode_control(msg, &frame);
            if (!send_all(fd, frame.data(), frame.size())) return;
        }

        // Wait at most 100 ms for data: that is how often the outbox and the
        // stop flag get looked at while the daemon is quiet, which at full
        // span is most of every ~230 ms sweep.
        fd_set rd;
        FD_ZERO(&rd);
        FD_SET((decltype(SW_INVALID))fd, &rd);
        timeval tv{0, 100000};
        const int ready = ::select(int(fd) + 1, &rd, nullptr, nullptr, &tv);
        if (ready == 0) continue;
        if (ready < 0) {
            if (sw_errno == SW_EINTR) continue;
            return;
        }

#ifdef _WIN32
        const int n = ::recv((SOCKET)fd, (char *)rx.data(), int(rx.size()), 0);
#else
        const ssize_t n = ::recv(int(fd), rx.data(), rx.size(), 0);
#endif
        if (n == 0) return;                       // peer closed
        if (n < 0) {
            const int e = sw_errno;
            if (e == SW_EWOULDBLOCK || e == SW_EINTR) continue;   // recv timeout
#ifndef _WIN32
            if (e == EAGAIN) continue;
#endif
            return;
        }
        reader.feed(rx.data(), size_t(n));

        Header h;
        const uint8_t *p = nullptr;
        while (reader.next(&h, &p)) {
            if (h.type == MsgType::Control)
                on_control(std::string((const char *)p, h.payload_len));
            else if (h.type == MsgType::Trace)
                on_trace(p, h.payload_len);
            else if (h.type == MsgType::Iq)
                on_iq(p, h.payload_len);
            // An unknown type is skipped rather than treated as an error: the
            // framing is intact, so the stream is still trustworthy.
        }

        if (reader.failed()) {
            // No frame markers exist to resynchronise to, by design. A framing
            // error means the byte stream can no longer be believed, so the
            // connection is dropped and rebuilt rather than patched up.
            set_detail("protocol error: " + reader.error());
            return;
        }
    }
}

// ------------------------------------------------------------------ inbound

void NetSource::on_control(const std::string &json) {
    JsonObject o;
    std::string perr;
    if (!parse_flat_json(json, &o, &perr)) {
        set_detail("unparseable control message from sweepd: " + perr);
        return;
    }

    const std::string type = json_str(o, "type");
    const std::string cmd = json_str(o, "cmd");

    // A reply that echoes `cmd` answers whatever request is in flight. The
    // daemon echoes it on refusals today; a future success reply must too.
    if (!cmd.empty()) {
        std::lock_guard<std::mutex> lk(req_mu_);
        if (!req_done_ && cmd == req_cmd_) {
            req_done_ = true;
            req_ok_ = (type != "error");
            req_msg_ = json_str(o, "msg");
            req_cv_.notify_all();
            return;
        }
    }

    if (type == "config") {
        std::lock_guard<std::mutex> lk(cfg_mu_);
        cfg_.f_start_hz = (long long)llround(json_num(o, "f_start_hz"));
        cfg_.f_stop_hz = (long long)llround(json_num(o, "f_stop_hz"));
        // The tuner's range rather than the swept span. Absent from an older
        // daemon, in which case tune_lo()/tune_hi() fall back to the span and
        // "full span" degrades to what it did before rather than to 0 Hz.
        cfg_.tune_min_hz = (long long)llround(json_num(o, "tune_min_hz"));
        cfg_.tune_max_hz = (long long)llround(json_num(o, "tune_max_hz"));
        cfg_.n_points = size_t(json_num(o, "n_points"));
        cfg_.rbw_hz = json_num(o, "rbw_hz");
        cfg_.bin_hz = json_num(o, "bin_hz");
        // All three absent from an older daemon, which leaves the RBW keys
        // hidden rather than offering a command it would refuse.
        cfg_.n_fft = size_t(json_num(o, "n_fft"));
        cfg_.rbw_auto = json_bool(o, "rbw_auto");
        cfg_.n_points_capped_from = size_t(json_num(o, "n_points_capped_from"));
        cfg_.n_segments = uint32_t(json_num(o, "n_segments"));
        cfg_.gain_db = float(json_num(o, "gain_db"));
        cfg_.adc_hz = json_num(o, "adc_hz", 0);
        // Absent from a daemon without set_gain, which hides the control.
        cfg_.gain_min_db = json_num(o, "gain_min_db", 0);
        cfg_.gain_max_db = json_num(o, "gain_max_db", 0);
        // Absent from a daemon without set_gain_table, which hides the control.
        cfg_.gain_table = json_str(o, "gain_table");

        // §7's segment boundary overlay. All zero from a daemon that predates
        // them, which SegmentPlanInfo::complete() reads as "no plan reported"
        // and the overlay reports as unavailable -- rather than reconstructing a
        // grid from defaults and drawing seams where there are none.
        cfg_.seg_first_lo_hz = json_num(o, "seg_first_lo_hz");
        cfg_.seg_step_hz = json_num(o, "seg_step_hz");
        cfg_.keep_hz = json_num(o, "keep_hz");
        cfg_.blank_hz = json_num(o, "blank_hz");
        cfg_.lo_shift_hz = json_num(o, "lo_shift_hz");
        cfg_.covered_lo_hz = json_num(o, "covered_lo_hz");
        cfg_.covered_hi_hz = json_num(o, "covered_hi_hz");
        cfg_.splice = json_str(o, "dc_policy", "splice") != "blank";
        window_from_string(json_str(o, "window"), &cfg_.window);
        detector_from_string(json_str(o, "detector"), &cfg_.detector);
        // Defaulting to true, not false: a daemon that omits the field, or an
        // older one that never sends it, must not be read as calibrated. §7's
        // badge has to fail lit.
        cfg_.uncal = json_bool(o, "uncal", true);
        cfg_.synthetic = false;
        // A config message is what ends a retune: the daemon sends one when the
        // new sweep starts (src/sweepd/main.cpp).
        cfg_.reconfiguring = false;

        // The axis. Absent from a daemon with no zero span, which reads as
        // "frequency" -- the behaviour before this feature existed, rather than
        // a time axis with no interval.
        const std::string xk = json_str(o, "x_kind", "frequency");
        cfg_.x_kind = xk == "time" ? XKind::Time : XKind::Frequency;
        cfg_.dt_ns = (uint64_t)llround(json_num(o, "dt_ns"));
        cfg_.time_res_s = json_num(o, "time_res_s");
        cfg_.overlap = json_num(o, "overlap");
        cfg_.center_hz = (long long)llround(json_num(o, "center_hz"));
        cfg_.lo_hz = (long long)llround(json_num(o, "lo_hz"));
        cfg_.lo_offset_hz = (long long)llround(json_num(o, "lo_offset_hz"));
        cfg_.meas_bw_hz = (long long)llround(json_num(o, "meas_bw_hz"));
        cfg_.band_includes_dc = json_bool(o, "band_includes_dc");
        cfg_.band_exceeds_bw = json_bool(o, "band_exceeds_bw");
        cfg_.trigger = json_str(o, "trigger", "free_run");
        cfg_.trig_level_db = json_num(o, "trig_level_db", -40.0);
        cfg_.trig_hyst_db = json_num(o, "trig_hyst_db");
        cfg_.trig_pretrig_points = size_t(json_num(o, "trig_pretrig_points"));
        cfg_.trig_search_points = size_t(json_num(o, "trig_search_points"));
        cfg_.capture_span_s = json_num(o, "capture_span_s");

        cfg_.tracking = json_str(o, "mode") == "tracking";
        // Raw IQ. iq_supported is sent by every daemon that has it, in any mode,
        // so the way in can be offered before the mode is entered.
        iq_supported_.store(json_bool(o, "iq_supported", false));
        cfg_.iq_max_input = size_t(json_num(o, "iq_max_input", 0));
        cfg_.iq_mode = json_str(o, "mode") == "iq";
        if (cfg_.iq_mode) {
            cfg_.iq_center_hz = (long long)llround(json_num(o, "iq_center_hz"));
            cfg_.iq_adc_fs_hz = json_num(o, "iq_adc_fs_hz");
            cfg_.iq_fs_hz = json_num(o, "iq_fs_hz");
            cfg_.iq_decim = unsigned(json_num(o, "iq_decim", 1));
            cfg_.iq_taps = size_t(json_num(o, "iq_taps", 1));
            cfg_.iq_n_samples = size_t(json_num(o, "iq_n_samples"));
            cfg_.iq_rf_bw_hz = json_num(o, "iq_rf_bw_hz");
            sweepd::iq_trig_mode_from_string(json_str(o, "iq_trig_mode", "none"),
                                             &cfg_.iq_trig.mode);
            sweepd::iq_trig_source_from_string(json_str(o, "iq_trig_source", "i"),
                                               &cfg_.iq_trig.source);
            cfg_.iq_trig.rising = json_bool(o, "iq_trig_rising", true);
            cfg_.iq_trig.level = json_num(o, "iq_trig_level");
            cfg_.iq_trig.hyst = json_num(o, "iq_trig_hyst");
            cfg_.iq_trig.pretrig = json_num(o, "iq_pretrig");
            cfg_.iq_auto_s = json_num(o, "iq_auto_s", 0.1);
        }
        cfg_.tsa_atten_db = json_num(o, "tsa_atten_db");
        cfg_.tsa_spacing_hz = json_num(o, "tsa_spacing_hz");
        cfg_.tsa_tones = size_t(json_num(o, "tsa_tones"));
        cfg_.tsa_tones_req = size_t(json_num(o, "tsa_tones_req"));
        cfg_.tsa_coherent = size_t(json_num(o, "tsa_coherent"));
        cfg_.tsa_keep_hz = json_num(o, "tsa_keep_hz");
        cfg_.tsa_tone_dbfs = json_num(o, "tsa_tone_dbfs");
        cfg_.tsa_min_atten_db = json_num(o, "tsa_min_atten_db", 30.0);
        cfg_.tsa_harmonic = int(json_num(o, "tsa_harmonic", 1.0));
        cfg_.tsa_rx_offset_hz = json_num(o, "tsa_rx_offset_hz");
        cfg_.tsa_rx_invert = json_bool(o, "tsa_rx_invert");
        cfg_.tsa_stim_off = json_bool(o, "tsa_stim_off");

        cfg_.gen.on = json_bool(o, "gen_on");
        cfg_.gen.freq_hz = (long long)llround(json_num(o, "gen_freq_hz", 1e9));
        cfg_.gen.atten_db = json_num(o, "gen_atten_db", 50.0);
        sweepd::gen_mod_from_string(json_str(o, "gen_mod", "cw"), &cfg_.gen.mod);
        cfg_.gen.rate_hz = json_num(o, "gen_rate_hz", 1000.0);
        cfg_.gen_rate_achieved_hz = cfg_.gen.rate_hz;
        cfg_.gen.am_depth = json_num(o, "gen_am_depth", 0.5);
        cfg_.gen.fm_dev_hz = json_num(o, "gen_fm_dev_hz", 5000.0);
        cfg_.gen.sweep_start_hz = (long long)llround(json_num(o, "gen_sweep_start_hz"));
        cfg_.gen.sweep_stop_hz = (long long)llround(json_num(o, "gen_sweep_stop_hz"));
        cfg_.gen.sweep_step_hz = (long long)llround(json_num(o, "gen_sweep_step_hz"));
        cfg_.gen.sweep_dwell_s = json_num(o, "gen_dwell_s", 0.1);
        cfg_.gen_sweep_points = size_t(json_num(o, "gen_sweep_points"));

        // §7's capability banner: what this installation actually is, in the
        // daemon's own measured numbers rather than a datasheet's.
        std::string d = "sweepd at " + opt_.host + ":" + std::to_string(opt_.port) + "\n";
        if (cfg_.tracking) {
            char tb[512];
            std::snprintf(tb, sizeof tb,
                          "TRACKING GENERATOR -- each point is a stimulus tone's received "
                          "level, not a spectrum\n"
                          "%s - %s, %zu points %s apart, %zu tones per LO step\n"
                          "both LOs written for real at every step (no fastlock)\n"
                          "TX attenuation %.2f dB, %.1f dBFS per tone at the DAC\n"
                          "noise bandwidth %.2f kHz (N=%zu x %zu coherent)\n",
                          format_frequency(double(cfg_.f_start_hz), 4).c_str(),
                          format_frequency(double(cfg_.f_stop_hz), 4).c_str(),
                          cfg_.n_points, format_frequency(cfg_.tsa_spacing_hz, 4).c_str(),
                          cfg_.tsa_tones, cfg_.tsa_atten_db, cfg_.tsa_tone_dbfs,
                          cfg_.rbw_hz / 1e3, cfg_.n_fft, cfg_.tsa_coherent);
            d += tb;
        } else if (cfg_.iq_mode) {
            char qb[512];
            std::snprintf(qb, sizeof qb,
                          "RAW IQ at %s -- I and Q against TIME, not a spectrum\n"
                          "LO on the centre (its leakage is the DC); %zu samples at %s "
                          "(ADC %s / %u)\n"
                          "trigger %s; one burst per arm, blind between bursts\n",
                          format_frequency(double(cfg_.iq_center_hz), 9).c_str(),
                          cfg_.iq_n_samples, format_frequency(cfg_.iq_fs_hz, 6).c_str(),
                          format_frequency(cfg_.iq_adc_fs_hz, 6).c_str(), cfg_.iq_decim,
                          sweepd::iq_trig_mode_name(cfg_.iq_trig.mode));
            d += qb;
        } else if (cfg_.zero_span()) {
            char zb[512];
            std::snprintf(zb, sizeof zb,
                          "ZERO SPAN at %s -- amplitude vs. TIME, not a spectrum\n"
                          "LO parked at %s, %s below the centre, measuring %s around it\n"
                          "%zu points at %.3f us; each integrates %.3f us "
                          "(overlap %.0f%% does not improve that)\n"
                          "window %.3f ms per frame\n",
                          format_frequency(double(cfg_.center_hz), 6).c_str(),
                          format_frequency(double(cfg_.lo_hz), 6).c_str(),
                          format_frequency(double(cfg_.lo_offset_hz), 3).c_str(),
                          format_frequency(double(cfg_.meas_bw_hz), 3).c_str(),
                          cfg_.n_points, double(cfg_.dt_ns) * 1e-3,
                          cfg_.time_res_s * 1e6, cfg_.overlap * 100.0,
                          json_num(o, "frame_span_s") * 1e3);
            d += zb;
            // Flagged by the daemon rather than refused, so the UI is what has
            // to say it: the reading is still a number, it is just not the
            // number the user thinks it is.
            if (cfg_.band_includes_dc)
                d += "WARNING: the LO offset is 0, so this reading includes the LO's own "
                     "leakage\n";
            if (cfg_.band_exceeds_bw)
                d += "WARNING: the measurement band reaches onto the analog filter's "
                     "skirt\n";
        } else {
            d += format_frequency(double(cfg_.f_start_hz), 4) + " - " +
                 format_frequency(double(cfg_.f_stop_hz), 4) + " in " +
                 std::to_string(cfg_.n_segments) + " segments\n";
            d += "dc_policy=" + json_str(o, "dc_policy", "?") + ", keep " +
                 format_frequency(json_num(o, "keep_hz"), 3) + ", blank " +
                 format_frequency(json_num(o, "blank_hz"), 3) +
                 (json_bool(o, "gapless") ? ", gapless" : ", HAS GAPS") + "\n";
            if (json_bool(o, "truncated"))
                d += "plan truncated by the tuner limits: covered " +
                     format_frequency(json_num(o, "covered_lo_hz"), 4) + " - " +
                     format_frequency(json_num(o, "covered_hi_hz"), 4) + "\n";
        }
        d += "fs " + format_frequency(json_num(o, "fs_hz"), 4) +
             (json_num(o, "adc_hz", 0) > 0
                  ? " (ADC " + format_frequency(json_num(o, "adc_hz"), 4) + ")" : "") +
             ", RF BW " +
             format_frequency(json_num(o, "bw_hz"), 3) + ", N_fft " +
             std::to_string((long long)llround(json_num(o, "n_fft"))) + "\n";
        char buf[192];
        std::snprintf(buf, sizeof buf,
                      "RBW %s derived from %s (ENBW %.4f)\nprofiles %lld (%s)\n"
                      "revisit %.0f ms -- %s\nunits %s\n",
                      format_frequency(cfg_.rbw_hz, 3).c_str(),
                      json_str(o, "window").c_str(), json_num(o, "enbw"),
                      (long long)llround(json_num(o, "profiles")),
                      json_str(o, "profile_cache", "?").c_str(),
                      json_num(o, "revisit_ms"),
                      json_str(o, "poi_note",
                               "signals shorter than the revisit interval may be missed")
                          .c_str(),
                      json_str(o, "units", "dBFS").c_str());
        d += buf;
        cfg_.description = d;
        return;
    }

    if (type == "status") {
        sweeps_per_s_.store(json_num(o, "sweeps_per_s"));
        hops_per_s_.store(json_num(o, "hops_per_s"));
        last_sweep_ms_.store(json_num(o, "last_sweep_ms"));
        server_dropped_.store((uint64_t)llround(json_num(o, "dropped_frames")));
        // Zero span's §7 number. Absent while sweeping, where it reads 0 and
        // the status bar shows the revisit interval instead.
        duty_.store(json_num(o, "duty"));
        acquisitions_.store((uint64_t)llround(json_num(o, "acquisitions")));
        no_trigger_.store((uint64_t)llround(json_num(o, "no_trigger")));
        armed_duty_.store(json_num(o, "armed_duty"));
        iq_waiting_.store(json_bool(o, "waiting_for_trigger", false));

        // "reconfiguring" means the daemon is rebuilding a profile table and
        // will produce no frames until it finishes -- up to ~1.3 s at full span.
        // Left to itself the UI would show a frozen trace with no explanation,
        // which is indistinguishable from a hung radio.
        const std::string state = json_str(o, "state", "sweeping");
        const bool busy = (state == "reconfiguring");
        {
            std::lock_guard<std::mutex> lk(cfg_mu_);
            cfg_.reconfiguring = busy;
        }
        if (busy) set_detail("sweepd is reconfiguring");
        return;
    }

    if (type == "error") {
        // Not a reply to anything we asked for, so it is news. It goes where
        // the user can see it rather than to a terminal they may not have.
        const std::string m = "sweepd: " + json_str(o, "msg");
        set_detail(m);
        std::lock_guard<std::mutex> lk(cfg_mu_);
        errors_.push_back(m);
        while (errors_.size() > 16) errors_.pop_front();
        return;
    }
}

void NetSource::on_iq(const uint8_t *p, size_t n) {
    IqMsg m;
    std::string err;
    if (!decode_iq(p, n, &m, &err)) {
        set_detail("undecodable IQ burst: " + err);
        return;
    }
    // The burst answers the arm; whatever "waiting" the last status said is over.
    iq_waiting_.store(false);
    std::lock_guard<std::mutex> lk(box_mu_);
    // One arm, one burst, so more than one waiting means the UI is not taking
    // them: keep the newest. Bursts are megabytes; a queue of them is memory.
    while (iq_queue_.size() >= 2) iq_queue_.pop_front();
    iq_queue_.push_back(std::move(m));
}

void NetSource::on_trace(const uint8_t *p, size_t n) {
    TraceMsg t;
    std::string err;
    if (!decode_trace(p, n, &t, &err)) {
        set_detail("undecodable trace frame: " + err);
        return;
    }

    // A frame arriving is the trigger having fired. Snapshotting the miss count
    // here is what lets status() answer "is it waiting?" without the daemon
    // having to carry a flag whose meaning would then be a second thing to keep
    // in sync: if the count has moved on since the last frame we drew, the
    // trigger has looked and found nothing since.
    no_trigger_at_frame_.store(no_trigger_.load());

    std::lock_guard<std::mutex> lk(box_mu_);
    // A gap in the daemon's sequence numbers means the *server* dropped frames
    // for this client under queue pressure (server.h). Frames are lossy by
    // design and never silently lossy, so the count is kept and shown.
    if (have_seq_ && t.seq > last_seq_ + 1) seq_gaps_ += t.seq - last_seq_ - 1;
    last_seq_ = t.seq;
    have_seq_ = true;

    // And here the UI is the slow client. Same contract, our end of it: oldest
    // out, counted.
    while (queue_.size() >= opt_.queue_depth) {
        queue_.pop_front();
        ++local_drops_;
    }
    queue_.push_back(std::move(t));
}

// ------------------------------------------------------------------ outbound

void NetSource::queue_out(const std::string &json) {
    std::lock_guard<std::mutex> lk(out_mu_);
    // Bounded, so a disconnected source cannot accumulate a year of status
    // requests to fire the instant it reconnects.
    while (outbox_.size() >= 32) outbox_.pop_front();
    outbox_.push_back(json);
}

void NetSource::resolve(bool ok, const std::string &msg) {
    std::lock_guard<std::mutex> lk(req_mu_);
    if (req_cmd_.empty() || req_done_) return;
    req_done_ = true;
    req_ok_ = ok;
    req_msg_ = msg;
    req_cv_.notify_all();
}

bool NetSource::request(const std::string &cmd, const std::string &json, std::string *err) {
    if (!connected_.load()) {
        *err = "not connected to " + opt_.host + ":" + std::to_string(opt_.port);
        return false;
    }

    std::unique_lock<std::mutex> lk(req_mu_);
    if (!req_cmd_.empty()) {
        *err = "a '" + req_cmd_ + "' request is still in flight";
        return false;
    }
    req_cmd_ = cmd;
    req_done_ = false;
    req_ok_ = false;
    req_msg_.clear();
    queue_out(json);

    const auto dur = std::chrono::duration<double>(opt_.request_timeout_s);
    const bool answered = req_cv_.wait_for(
        lk, std::chrono::duration_cast<std::chrono::milliseconds>(dur),
        [this] { return req_done_; });

    const bool ok = req_ok_;
    const std::string msg = req_msg_;
    req_cmd_.clear();
    req_done_ = false;
    lk.unlock();

    if (!answered) {
        char buf[160];
        std::snprintf(buf, sizeof buf, "sweepd did not answer '%s' within %.0f ms",
                      cmd.c_str(), opt_.request_timeout_s * 1e3);
        *err = buf;
        return false;
    }
    if (!ok) {
        *err = msg.empty() ? ("sweepd refused '" + cmd + "'") : msg;
        return false;
    }

    // It said yes, so whatever it now is, it is not what we last recorded.
    queue_out(JsonWriter().kv("cmd", "config").done());
    return true;
}

// ------------------------------------------------------------------ interface

bool NetSource::poll(TraceMsg *out) {
    std::lock_guard<std::mutex> lk(box_mu_);
    if (queue_.empty()) return false;
    *out = std::move(queue_.front());
    queue_.pop_front();
    ++delivered_;
    return true;
}

SourceConfig NetSource::config() const {
    std::lock_guard<std::mutex> lk(cfg_mu_);
    return cfg_;
}

SourceStatus NetSource::status() const {
    SourceStatus s;
    s.connected = connected_.load();
    s.sweeps_per_s = sweeps_per_s_.load();
    s.hops_per_s = hops_per_s_.load();
    s.last_sweep_ms = last_sweep_ms_.load();
    s.duty = duty_.load();
    s.acquisitions = acquisitions_.load();
    s.no_trigger = no_trigger_.load();
    s.armed_duty = armed_duty_.load();
    {
        std::lock_guard<std::mutex> lk(box_mu_);
        s.frames = delivered_;
        // Both are sweeps that happened and are not on screen. They have
        // different causes -- the daemon's queue versus ours -- so the split is
        // spelled out below rather than left to be guessed from one number.
        s.missed = seq_gaps_ + local_drops_;
    }
    {
        std::lock_guard<std::mutex> lk(cfg_mu_);
        s.detail = detail_;
        s.waiting_for_trigger =
            cfg_.iq_mode ? iq_waiting_.load()
                         : cfg_.triggered() && s.no_trigger > no_trigger_at_frame_.load();
    }
    if (s.missed) {
        std::lock_guard<std::mutex> lk(box_mu_);
        char buf[160];
        std::snprintf(buf, sizeof buf,
                      " | dropped: %llu by sweepd, %llu by the display",
                      (unsigned long long)seq_gaps_, (unsigned long long)local_drops_);
        s.detail += buf;
    }
    return s;
}

bool NetSource::take_error(std::string *msg) {
    std::lock_guard<std::mutex> lk(cfg_mu_);
    if (errors_.empty()) return false;
    *msg = std::move(errors_.front());
    errors_.pop_front();
    return true;
}

void NetSource::set_detail(const std::string &s) {
    std::lock_guard<std::mutex> lk(cfg_mu_);
    detail_ = s;
}

bool NetSource::set_span(double f_start_hz, double f_stop_hz, std::string *err) {
    return request("set_span",
                   JsonWriter()
                       .kv("cmd", "set_span")
                       .kv("f_start_hz", f_start_hz)
                       .kv("f_stop_hz", f_stop_hz)
                       .done(),
                   err);
}

bool NetSource::set_lo_shift(double hz, std::string *err) {
    return request("set_lo_shift",
                   JsonWriter().kv("cmd", "set_lo_shift").kv("lo_shift_hz", hz).done(),
                   err);
}

bool NetSource::set_segmentation(double keep_hz, const std::string &dc_policy,
                                 std::string *err) {
    // Only the fields the caller actually meant are sent. set_segmentation
    // leaves an omitted field alone, so sending both every time would make a
    // key that changes the policy also re-assert a width the user never typed.
    JsonWriter w;
    w.kv("cmd", "set_segmentation");
    if (keep_hz > 0) w.kv("keep_hz", keep_hz);
    if (!dc_policy.empty()) w.kv("dc_policy", dc_policy);
    return request("set_segmentation", w.done(), err);
}

bool NetSource::set_gain(double gain_db, std::string *err) {
    return request("set_gain",
                   JsonWriter().kv("cmd", "set_gain").kv("gain_db", gain_db).done(), err);
}

bool NetSource::set_gain_table(const std::string &text, const std::string &name,
                               std::string *err) {
    JsonWriter w;
    w.kv("cmd", "set_gain_table");
    if (text.empty()) {
        w.kv("stock", true);
    } else {
        w.kv("text", text);
        w.kv("name", name);
    }
    return request("set_gain_table", w.done(), err);
}

bool NetSource::set_points(size_t n_points, std::string *err) {
    return request("set_points",
                   JsonWriter()
                       .kv("cmd", "set_points")
                       .kv("n_points", double(n_points))
                       .done(),
                   err);
}

bool NetSource::set_rbw(double rbw_hz, std::string *err) {
    return request("set_rbw",
                   JsonWriter().kv("cmd", "set_rbw").kv("rbw_hz", rbw_hz).done(), err);
}

bool NetSource::set_detector(Detector d, std::string *err) {
    return request("set_detector",
                   JsonWriter()
                       .kv("cmd", "set_detector")
                       .kv("detector", detector_name(d))
                       .done(),
                   err);
}

bool NetSource::set_window(Window w, std::string *err) {
    return request("set_window",
                   JsonWriter().kv("cmd", "set_window").kv("window", window_name(w)).done(),
                   err);
}

bool NetSource::set_zero_span(double center_hz, size_t n_fft, size_t n_points,
                              double overlap, std::string *err) {
    // Only the fields the caller actually set are sent. sweepd treats an absent
    // field as "leave it alone", so one soft key changes one knob and a refusal
    // names that knob rather than a whole configuration the user did not touch.
    JsonWriter w;
    w.kv("cmd", "set_zero_span");
    if (center_hz > 0) w.kv("center_hz", center_hz);
    if (n_fft) w.kv("n_fft", double(n_fft));
    if (n_points) w.kv("n_points", double(n_points));
    if (overlap >= 0) w.kv("overlap", overlap);
    return request("set_zero_span", w.done(), err);
}

// One request per key press, and NaN means "leave that one alone", so the
// daemon's refusal names the knob the user just turned rather than a whole
// configuration they did not touch.
bool NetSource::set_trigger(const std::string &mode, double level_db, double hyst_db,
                            double pretrig, std::string *err) {
    JsonWriter w;
    w.kv("cmd", "set_zero_span");
    if (!mode.empty()) w.kv("trigger", mode);
    if (level_db == level_db) w.kv("trig_level_db", level_db);
    if (hyst_db == hyst_db) w.kv("trig_hyst_db", hyst_db);
    if (pretrig == pretrig) w.kv("trig_pretrig", pretrig);
    return request("set_zero_span", w.done(), err);
}

bool NetSource::set_tracking(const TrackRequest &r, std::string *err) {
    JsonWriter w;
    w.kv("cmd", "set_tracking");
    if (r.atten_db == r.atten_db) w.kv("atten_db", r.atten_db);
    if (r.spacing_hz == r.spacing_hz && r.spacing_hz > 0) w.kv("spacing_hz", r.spacing_hz);
    if (r.tones >= 0) w.kv("tones", double(r.tones));
    if (r.coherent > 0) w.kv("coherent", double(r.coherent));
    if (r.harmonic > 0) w.kv("harmonic", double(r.harmonic));
    if (r.rx_offset_hz == r.rx_offset_hz) w.kv("rx_offset_hz", r.rx_offset_hz);
    if (r.rx_invert >= 0) w.kv("rx_invert", r.rx_invert != 0);
    if (r.stim_off >= 0) w.kv("stim_off", r.stim_off != 0);
    return request("set_tracking", w.done(), err);
}

bool NetSource::set_generator(const sweepd::GenConfig &g, std::string *err) {
    JsonWriter w;
    w.kv("cmd", "set_generator")
        .kv("on", g.on)
        .kv("freq_hz", double(g.freq_hz))
        .kv("atten_db", g.atten_db)
        .kv("mod", sweepd::gen_mod_name(g.mod))
        .kv("rate_hz", g.rate_hz)
        .kv("am_depth", g.am_depth)
        .kv("fm_dev_hz", g.fm_dev_hz)
        .kv("sweep_start_hz", double(g.sweep_start_hz))
        .kv("sweep_stop_hz", double(g.sweep_stop_hz))
        .kv("sweep_step_hz", double(g.sweep_step_hz))
        .kv("dwell_s", g.sweep_dwell_s);
    return request("set_generator", w.done(), err);
}

// ------------------------------------------------------------------ raw IQ

bool NetSource::set_iq(const IqRequest &r, std::string *err) {
    JsonWriter w;
    w.kv("cmd", "set_iq");
    if (r.center_hz == r.center_hz) w.kv("center_hz", r.center_hz);
    if (r.n_samples >= 0) w.kv("n_samples", double(r.n_samples));
    if (r.decim >= 0) w.kv("decim", double(r.decim));
    if (!r.trig_mode.empty()) w.kv("trig_mode", r.trig_mode);
    if (!r.trig_source.empty()) w.kv("trig_source", r.trig_source);
    if (r.trig_rising >= 0) w.kv("trig_rising", double(r.trig_rising));
    if (r.trig_level == r.trig_level) w.kv("trig_level", r.trig_level);
    if (r.trig_hyst == r.trig_hyst) w.kv("trig_hyst", r.trig_hyst);
    if (r.pretrig == r.pretrig) w.kv("pretrig", r.pretrig);
    if (r.auto_s == r.auto_s) w.kv("auto_s", r.auto_s);
    return request("set_iq", w.done(), err);
}

// Not request(): an arm goes out once per burst from the render thread, and
// waiting for its "ok" would put a network round trip into every frame that
// re-arms. Sent and not waited on. A refusal still arrives -- as an error with
// cmd=iq_arm, which on_control reports like any other unsolicited error.
bool NetSource::iq_arm(std::string *err) {
    if (!connected_.load()) {
        *err = "not connected to " + opt_.host + ":" + std::to_string(opt_.port);
        return false;
    }
    queue_out(JsonWriter().kv("cmd", "iq_arm").done());
    return true;
}

void NetSource::iq_disarm() {
    if (connected_.load()) queue_out(JsonWriter().kv("cmd", "iq_disarm").done());
}

bool NetSource::poll_iq(IqMsg *out) {
    std::lock_guard<std::mutex> lk(box_mu_);
    if (iq_queue_.empty()) return false;
    *out = std::move(iq_queue_.front());
    iq_queue_.pop_front();
    return true;
}

bool NetSource::set_sweep_mode(std::string *err) {
    return request("set_mode",
                   JsonWriter().kv("cmd", "set_mode").kv("mode", "sweep").done(), err);
}

}  // namespace sweepui
