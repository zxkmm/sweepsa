// The §10.4 source: sweepui talking to a running sweepd over the §5 wire.
//
// This is one new implementation of FrameSource and nothing above that
// interface changes, which is what building §10.3 against the synthetic source
// first was for. tools/sweep_client is the reference decoder and this does
// exactly what it does -- Reader::feed, decode_trace, flat JSON for control --
// with three differences that come from being inside a GUI rather than a pipe:
//
//   1. It owns a thread. The UI thread must never block on a socket; a frame
//      arrives every ~230 ms at full span (docs/characterization.md) and a
//      recv() on the render thread would tie the display's frame rate to the
//      network.
//   2. It reconnects. The E310's rootfs is RAM-backed and the daemon is
//      restarted constantly during development; a UI that silently freezes on
//      the last frame it received is the §7 failure -- a display that looks
//      live and is not. Disconnection is a badge, and the last known
//      configuration is kept so the axis does not collapse on a blip.
//   3. It is portable. §6 requires sweepui to build on Windows, and
//      src/sweepd/server.cpp is POSIX-only by design (see the CMake comment on
//      sweepd_proto), so the client half carries its own small socket shim
//      rather than linking the daemon's.
//
// ------------------------------------------------------------------ refusals
//
// sweepd refuses runtime reconfiguration today: it takes its span, FFT size,
// window and detector from argv and answers anything else with an error naming
// the limitation (src/sweepd/main.cpp). The setters here therefore send the
// request and *wait for the daemon's answer* rather than guessing it. Two
// reasons that matters:
//
//   - the refusal shown in the UI is the daemon's own words, so when runtime
//     reconfiguration lands the UI starts working with no change here;
//   - a client that hardcoded "sweepd always says no" would keep saying no
//     after the daemon learned to say yes, which is a lie in the other
//     direction.
//
// The wait is bounded (`request_timeout_s`) and a timeout is itself reported as
// a refusal, because a control that hangs the UI is worse than one that fails.
//
// The command vocabulary -- set_span, set_points, set_rbw, set_detector, set_window,
// set_zero_span, set_mode, set_lo_shift, set_segmentation --
// is proposed by this file and today only ever produces an error. That error
// echoes `cmd`, which is how the reply is matched to the request; a future
// success reply must echo it too.

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "source.h"

namespace sweepui {

struct NetOptions {
    std::string host = "192.168.1.10";
    uint16_t port = 5556;

    // Bounded so that closing the window never waits on a dead route. A
    // blocking connect() to an unreachable host can sit for over a minute, and
    // the destructor joins this thread.
    double connect_timeout_s = 3.0;

    // How often to ask for the §5 status message. The daemon only builds one
    // when asked (src/sweepd/main.cpp), so this is the rate the sweep-rate and
    // dropped-frame readouts update at.
    double status_interval_s = 1.0;

    double reconnect_backoff_s = 1.0;
    double request_timeout_s = 1.0;

    // Same depth and the same contract as the synthetic source: oldest out, and
    // the producer drops on overflow rather than applying backpressure. Here
    // the producer is a socket, so backpressure would propagate all the way to
    // the daemon's per-client queue and then to its sweep -- which server.h
    // exists to prevent.
    size_t queue_depth = 16;
};

class NetSource : public FrameSource {
public:
    explicit NetSource(const NetOptions &opt);
    ~NetSource() override;

    const char *name() const override { return "sweepd"; }
    bool poll(sweepd::proto::TraceMsg *out) override;
    SourceConfig config() const override;
    SourceStatus status() const override;

    bool take_error(std::string *msg) override;
    bool set_span(double f_start_hz, double f_stop_hz, std::string *err) override;
    bool set_points(size_t n_points, std::string *err) override;
    bool set_rbw(double rbw_hz, std::string *err) override;
    bool set_segmentation(double keep_hz, const std::string &dc_policy,
                          std::string *err) override;
    bool set_gain(double gain_db, std::string *err) override;
    bool set_gain_table(const std::string &text, const std::string &name,
                        std::string *err) override;
    bool set_detector(sweepd::Detector d, std::string *err) override;
    bool set_window(sweepd::Window w, std::string *err) override;
    bool set_zero_span(double center_hz, size_t n_fft, size_t n_points, double overlap,
                       std::string *err) override;
    bool set_trigger(const std::string &mode, double level_db, double hyst_db,
                     double pretrig, std::string *err) override;
    bool set_lo_shift(double hz, std::string *err) override;
    bool set_sweep_mode(std::string *err) override;
    using FrameSource::set_tracking;
    bool set_tracking(const TrackRequest &r, std::string *err) override;
    // The daemon is the judge: one built before tracking existed refuses
    // set_tracking by name, and the refusal reaches the user.
    bool can_track() const override { return true; }
    bool set_generator(const sweepd::GenConfig &g, std::string *err) override;
    bool can_generate() const override { return true; }
    bool set_iq(const IqRequest &r, std::string *err) override;
    bool iq_arm(std::string *err) override;
    void iq_disarm() override;
    bool poll_iq(sweepd::proto::IqMsg *out) override;
    // Only a daemon that says it has raw IQ: an older one would refuse set_iq.
    bool can_iq() const override { return iq_supported_.load(); }

private:
    void run();                                  // network thread
    // intptr_t rather than int because a Windows SOCKET is a UINT_PTR and does
    // not fit one. -1 is the invalid handle on both.
    intptr_t dial(std::string *err);
    void serve(intptr_t fd);                     // read/dispatch until the peer goes
    void on_control(const std::string &json);
    void on_trace(const uint8_t *p, size_t n);
    void on_iq(const uint8_t *p, size_t n);

    // Sends `json` and blocks until the daemon answers it or the timeout
    // expires. False means refused or unanswered, with the reason in *err.
    bool request(const std::string &cmd, const std::string &json, std::string *err);
    void resolve(bool ok, const std::string &msg);
    void queue_out(const std::string &json);

    void set_detail(const std::string &s);

    NetOptions opt_;

    std::atomic<bool> stop_{false};
    std::atomic<bool> connected_{false};
    std::thread thread_;

    mutable std::mutex cfg_mu_;
    // Kept across a disconnect: dropping back to a zero span would make the app
    // clear the waterfall and reset the view every time the link blinked.
    SourceConfig cfg_;
    std::string detail_;
    std::deque<std::string> errors_;   // under cfg_mu_; see take_error()

    mutable std::mutex box_mu_;
    std::deque<sweepd::proto::TraceMsg> queue_;
    std::deque<sweepd::proto::IqMsg> iq_queue_;   // at most two; see on_iq
    uint64_t delivered_ = 0;      // frames handed to the UI
    uint64_t seq_gaps_ = 0;       // sweeps the *daemon* dropped for us
    uint64_t local_drops_ = 0;    // sweeps *we* dropped: the UI fell behind
    uint64_t last_seq_ = 0;
    bool have_seq_ = false;

    // Numbers from the daemon's 0x01 status message.
    std::atomic<double> sweeps_per_s_{0};
    std::atomic<double> hops_per_s_{0};
    std::atomic<double> last_sweep_ms_{0};
    std::atomic<uint64_t> server_dropped_{0};
    // Zero span's duty cycle, from the daemon's status message.
    std::atomic<double> duty_{0};
    // The video trigger's accounting, likewise.
    std::atomic<uint64_t> acquisitions_{0};
    std::atomic<uint64_t> no_trigger_{0};
    std::atomic<double> armed_duty_{0};
    std::atomic<bool> iq_supported_{false};
    std::atomic<bool> iq_waiting_{false};
    // no_trigger as it stood when the last frame arrived. See on_trace.
    std::atomic<uint64_t> no_trigger_at_frame_{0};

    std::mutex out_mu_;
    std::deque<std::string> outbox_;

    std::mutex req_mu_;
    std::condition_variable req_cv_;
    std::string req_cmd_;         // empty when no request is in flight
    bool req_done_ = false;
    bool req_ok_ = false;
    std::string req_msg_;
};

}  // namespace sweepui
