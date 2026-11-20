// The host end of the sweepd wire protocol. CLAUDE.md §5, delivery step §10.4.
//
// Connects to a running sweepd, decodes 0x02 trace frames, and writes the same
// CSV sweepd writes when run locally. Two reasons it exists before sweepui:
//
//   1. It proves the protocol end to end over a real network with no GUI in the
//      picture, so when §10.4 wires the two together a disagreement about the
//      wire cannot hide behind a rendering bug.
//   2. `sweep_client e310 > trace.csv` and `ssh e310 sweepd > trace.csv` should
//      produce the same file. That is a much sharper check than any assertion
//      about the format, and it is one command.
//
// This is also the reference decoder. sweepui does exactly what this does.
//
// Amplitude is dBFS and stays dBFS: no correction file is loaded, so nothing
// licenses calling it dBm (§7).

#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include "proto.h"

using namespace sweepd;
using namespace sweepd::proto;

namespace {

volatile sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

void usage() {
    std::fprintf(stderr,
"usage: sweep_client [options] HOST[:PORT]   (CSV to stdout, rest to stderr)\n"
"\n"
"  --port N          default 5556 if HOST carries no :port\n"
"  --format wide|long                                (default wide)\n"
"  --sweeps N        stop after N frames, 0 = forever (default 0)\n"
"  --status          ask for a status message every second and print it\n"
"  --raw-control     print every inbound control message verbatim\n"
"  --send JSON       send a 0x01 control message after connecting, and print\n"
"                    every reply. Repeatable, sent in order. This is the shell\n"
"                    end of the retune path, so it can be checked before the GUI\n"
"                    is in the picture:\n"
"                      --send '{\"cmd\":\"set_span\",\"f_start_hz\":7e7,\"f_stop_hz\":6e9}'\n"
"                      --send '{\"cmd\":\"set_detector\",\"detector\":\"rms\"}'\n"
"  --send-after S    delay before sending them, seconds     (default 1)\n"
"  -h, --help\n");
}

int dial(const std::string &host, uint16_t port, std::string *err) {
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    addrinfo *res = nullptr;
    const std::string svc = std::to_string(port);
    const int rc = ::getaddrinfo(host.c_str(), svc.c_str(), &hints, &res);
    if (rc != 0) {
        *err = "resolve " + host + ": " + ::gai_strerror(rc);
        return -1;
    }

    int fd = -1;
    for (addrinfo *a = res; a; a = a->ai_next) {
        fd = ::socket(a->ai_family, a->ai_socktype, a->ai_protocol);
        if (fd < 0) continue;
        if (::connect(fd, a->ai_addr, a->ai_addrlen) == 0) break;
        ::close(fd);
        fd = -1;
    }
    ::freeaddrinfo(res);
    if (fd < 0) {
        *err = "connect " + host + ":" + svc + ": " + std::strerror(errno);
        return -1;
    }
    const int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    return fd;
}

int fmt_db(char *buf, size_t cap, float v) {
    // Matches sweepd's own CSV: "nan" spelled explicitly, because %f's choice
    // of sign for NaN is platform-dependent and "-nan" trips up readers.
    if (std::isnan(v)) return std::snprintf(buf, cap, "nan");
    return std::snprintf(buf, cap, "%.2f", v);
}

}  // namespace

int main(int argc, char **argv) {
    std::string host;
    uint16_t port = 5556;
    bool port_given = false;
    bool wide = true;
    uint64_t max_frames = 0;
    bool want_status = false;
    bool raw_control = false;
    std::vector<std::string> to_send;
    double send_after_s = 1.0;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const char *v = (i + 1 < argc) ? argv[i + 1] : nullptr;
        auto need = [&]() -> const char * {
            if (!v) {
                std::fprintf(stderr, "%s needs a value\n", a.c_str());
                std::exit(2);
            }
            ++i;
            return v;
        };
        if (a == "--port") {
            port = uint16_t(std::atoi(need()));
            port_given = true;
        } else if (a == "--format") {
            const std::string f = need();
            if (f == "wide") wide = true;
            else if (f == "long") wide = false;
            else {
                std::fprintf(stderr, "--format takes wide or long\n");
                return 2;
            }
        } else if (a == "--sweeps") {
            max_frames = std::strtoull(need(), nullptr, 10);
        } else if (a == "--status") {
            want_status = true;
        } else if (a == "--raw-control") {
            raw_control = true;
        } else if (a == "--send") {
            to_send.push_back(need());
        } else if (a == "--send-after") {
            send_after_s = std::atof(need());
        } else if (a == "-h" || a == "--help") {
            usage();
            return 0;
        } else if (!a.empty() && a[0] == '-') {
            std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
            usage();
            return 2;
        } else if (host.empty()) {
            host = a;
        } else {
            std::fprintf(stderr, "unexpected argument: %s\n", a.c_str());
            return 2;
        }
    }

    if (host.empty()) {
        usage();
        return 2;
    }
    // "[fe80::1%usb0]:5556" is IPv6; only a lone colon separates a port.
    if (!host.empty() && host[0] == '[') {
        const size_t close = host.find(']');
        if (close == std::string::npos) {
            std::fprintf(stderr, "unclosed [ in \"%s\"\n", host.c_str());
            return 2;
        }
        if (close + 1 < host.size() && host[close + 1] == ':' && !port_given)
            port = uint16_t(std::atoi(host.c_str() + close + 2));
        host = host.substr(1, close - 1);
    } else if (host.find(':') != std::string::npos && host.find(':') == host.rfind(':')) {
        const size_t colon = host.rfind(':');
        if (!port_given) port = uint16_t(std::atoi(host.c_str() + colon + 1));
        host = host.substr(0, colon);
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGPIPE, SIG_IGN);

    std::string err;
    const int fd = dial(host, port, &err);
    if (fd < 0) {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 1;
    }
    std::fprintf(stderr, "connected to %s:%u\n", host.c_str(), unsigned(port));

    Reader reader;
    std::vector<uint8_t> rx(1 << 16);
    std::vector<char> line;
    uint64_t frames = 0;
    uint64_t last_seq = 0;
    uint64_t gaps = 0;      // sweeps the server dropped for us; see server.h
    bool header_written = false;
    double t_next_status = 0;

    // A 200 ms receive timeout, so Ctrl-C is noticed promptly and the status
    // request has somewhere to be issued from without a second thread.
    timeval tv{0, 200000};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);

    // Sent once, a moment after connecting, so the greeting has been read and
    // the "before" config is on screen above the reply.
    double t_send = -1;
    {
        timespec ts{};
        clock_gettime(CLOCK_MONOTONIC, &ts);
        t_send = double(ts.tv_sec) + double(ts.tv_nsec) * 1e-9 + send_after_s;
    }

    while (!g_stop) {
        if (!to_send.empty()) {
            timespec ts{};
            clock_gettime(CLOCK_MONOTONIC, &ts);
            const double now = double(ts.tv_sec) + double(ts.tv_nsec) * 1e-9;
            if (now >= t_send) {
                for (const std::string &js : to_send) {
                    std::vector<uint8_t> req;
                    encode_control(js, &req);
                    if (::send(fd, req.data(), req.size(), 0) < 0 && errno != EAGAIN) {
                        std::fprintf(stderr, "send: %s\n", std::strerror(errno));
                        break;
                    }
                    std::fprintf(stderr, "sent     %s\n", js.c_str());
                }
                to_send.clear();
            }
        }

        if (want_status) {
            timespec ts{};
            clock_gettime(CLOCK_MONOTONIC, &ts);
            const double now = double(ts.tv_sec) + double(ts.tv_nsec) * 1e-9;
            if (now >= t_next_status) {
                std::vector<uint8_t> req;
                encode_control(JsonWriter().kv("cmd", "status").done(), &req);
                if (::send(fd, req.data(), req.size(), 0) < 0 && errno != EAGAIN) break;
                t_next_status = now + 1.0;
            }
        }

        const ssize_t n = ::recv(fd, rx.data(), rx.size(), 0);
        if (n == 0) {
            std::fprintf(stderr, "server closed the connection\n");
            break;
        }
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            std::fprintf(stderr, "recv: %s\n", std::strerror(errno));
            break;
        }
        reader.feed(rx.data(), size_t(n));

        Header h;
        const uint8_t *p = nullptr;
        while (reader.next(&h, &p)) {
            if (h.type == MsgType::Control) {
                const std::string js((const char *)p, h.payload_len);
                JsonObject o;
                std::string perr;
                if (!parse_flat_json(js, &o, &perr)) {
                    std::fprintf(stderr, "control: unparseable (%s): %s\n", perr.c_str(),
                                 js.c_str());
                    continue;
                }
                if (raw_control) std::fprintf(stderr, "control: %s\n", js.c_str());
                const std::string type = json_str(o, "type");
                if (type == "config" && !raw_control) {
                    // The capability banner and the axis, from the daemon
                    // rather than from a datasheet (§7).
                    std::fprintf(stderr,
                                 "config   %.3f - %.3f MHz, %.0f points, RBW %.1f kHz, "
                                 "%s/%s, %.0f segments, %s\n",
                                 json_num(o, "f_start_hz") / 1e6,
                                 json_num(o, "f_stop_hz") / 1e6, json_num(o, "n_points"),
                                 json_num(o, "rbw_hz") / 1e3,
                                 json_str(o, "window").c_str(),
                                 json_str(o, "detector").c_str(),
                                 json_num(o, "n_segments"),
                                 json_bool(o, "uncal", true) ? "UNCAL (dBFS)" : "corrected");
                } else if (type == "status" && !raw_control) {
                    std::fprintf(stderr,
                                 "status   %.2f sweeps/s, %.0f hops/s, last %.1f ms, "
                                 "%.0f dropped to this client\n",
                                 json_num(o, "sweeps_per_s"), json_num(o, "hops_per_s"),
                                 json_num(o, "last_sweep_ms"),
                                 json_num(o, "dropped_frames"));
                } else if (type == "error") {
                    std::fprintf(stderr, "server error: %s\n", json_str(o, "msg").c_str());
                } else if (type == "ok" && !raw_control) {
                    std::fprintf(stderr, "accepted %s: %s\n", json_str(o, "cmd").c_str(),
                                 json_str(o, "msg").c_str());
                }
                continue;
            }

            if (h.type != MsgType::Trace) continue;

            TraceMsg t;
            std::string derr;
            if (!decode_trace(p, h.payload_len, &t, &derr)) {
                std::fprintf(stderr, "trace decode: %s\n", derr.c_str());
                g_stop = 1;
                break;
            }

            if (!header_written) {
                std::printf("# sweep_client -- CLAUDE.md §5 frame 0x02 decoded from TCP\n");
                std::printf("# UNCAL: values are dBFS, not dBm (§7).\n");
                // The axis this frame carries, before anything that is read in
                // terms of it. In zero span the x column is seconds and
                // f_start == f_stop == the parked centre; labelling that in Hz
                // would be the exact mistake the frame's x_kind exists to stop
                // (proto.h, "one trace format").
                const bool time_axis = t.x_kind == XKind::Time;
                std::printf("# nan:   %s -- unmeasured, not quiet.\n",
                            time_axis ? "this block was not captured"
                                      : "no segment covered that bin");
                if (time_axis) {
                    std::printf("# x_kind=time -- amplitude vs. TIME at a parked LO, "
                                "not a spectrum.\n");
                    std::printf("# center_hz=%lld n_points=%zu dt_s=%.12f\n",
                                t.f_start_hz, t.n_points(), double(t.dt_ns) * 1e-9);
                } else {
                    std::printf("# x_kind=frequency\n");
                    std::printf("# f_start_hz=%lld f_stop_hz=%lld n_points=%zu bin_hz=%.3f\n",
                                t.f_start_hz, t.f_stop_hz, t.n_points(), t.bin_hz);
                }
                std::printf("# rbw_hz=%.3f window=%s detector=%s gain_db=%.1f segments=%u\n",
                            t.rbw_hz, window_name(t.window), detector_name(t.detector),
                            double(t.gain_db), unsigned(t.n_segments));
                if (wide)
                    std::printf("# columns: seq,t_mono_s,%s,n_overload_seg,dbfs[0..%zu]\n",
                                time_axis ? "frame_s" : "sweep_s", t.n_points() - 1);
                else
                    std::printf("seq,t_mono_s,%s,dbfs\n",
                                time_axis ? "time_s" : "freq_hz");
                header_written = true;
            }

            // Sequence numbers are the daemon's, so a gap means the server
            // dropped frames for this client under queue pressure (server.h).
            // Frames are lossy by design; silently lossy is not acceptable.
            if (frames && t.seq > last_seq + 1) gaps += t.seq - last_seq - 1;
            last_seq = t.seq;

            if (t.n_overloaded_segments)
                std::fprintf(stderr, "OVERLOAD: %u of %u segments near full scale -- the "
                                     "spurs in this sweep may be the front end, not the air\n",
                             unsigned(t.n_overloaded_segments), unsigned(t.n_segments));

            // x_at() knows which axis the frame carries, so the long format
            // cannot print a time under a frequency heading.
            const bool time_axis = t.x_kind == XKind::Time;

            char buf[32];
            if (wide) {
                line.clear();
                char head[128];
                int k = std::snprintf(head, sizeof head, "%" PRIu64 ",%.6f,%.6f,%u", t.seq,
                                      t.t_mono_s, t.sweep_seconds,
                                      unsigned(t.n_overloaded_segments));
                line.insert(line.end(), head, head + k);
                for (float v : t.db) {
                    line.push_back(',');
                    k = fmt_db(buf, sizeof buf, v);
                    line.insert(line.end(), buf, buf + k);
                }
                line.push_back('\n');
                std::fwrite(line.data(), 1, line.size(), stdout);
            } else {
                for (size_t i = 0; i < t.db.size(); ++i) {
                    fmt_db(buf, sizeof buf, t.db[i]);
                    if (time_axis)
                        std::printf("%" PRIu64 ",%.6f,%.9f,%s\n", t.seq, t.t_mono_s,
                                    t.x_at(i), buf);
                    else
                        std::printf("%" PRIu64 ",%.6f,%.0f,%s\n", t.seq, t.t_mono_s,
                                    t.x_at(i), buf);
                }
            }

            ++frames;
            if (max_frames && frames >= max_frames) g_stop = 1;
        }

        if (reader.failed()) {
            std::fprintf(stderr, "protocol: %s\n", reader.error().c_str());
            break;
        }
    }

    std::fflush(stdout);
    ::close(fd);
    std::fprintf(stderr, "%" PRIu64 " frames", frames);
    if (gaps)
        std::fprintf(stderr, ", %" PRIu64 " sweeps missed (the server dropped them to "
                             "avoid slowing the sweep)", gaps);
    std::fprintf(stderr, "\n");
    return 0;
}
