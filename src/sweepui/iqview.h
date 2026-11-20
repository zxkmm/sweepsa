// The raw IQ time sink: what GNU Radio's QT GUI Time Sink shows for a complex
// stream -- I and Q against time -- with the scope-shaped tools around it that
// make the picture a measurement: a draggable trigger level, two time cursors
// and two amplitude cursors, an I/Q (constellation) plane, persistence, and a
// readout of the receiver's own impairments.
//
// It is a display of 0x03 bursts (proto.h, "0x03 iq") and nothing else. The
// controls that change what the *radio* does -- centre, burst length,
// decimation, trigger, run/stop/single -- are App's, next to the source, the
// same split as zero span. Everything here changes only what is drawn and
// measured, and none of it is an FFT (§8: the host does no FFT; the frequency
// domain is the swept view's job).
//
// What is on screen is labelled with what it is:
//   * the time axis is the burst's own, t = 0 at the trigger edge, negative
//     before it -- the same convention as zero span's t0;
//   * amplitudes are full-scale units of the ADC, never volts: nothing here is
//     calibrated, and the UNCAL rule (§7) applies to a waveform too;
//   * "remove DC" is a display subtraction, and the DC readout keeps the raw
//     number, because the LO leakage it hides is a property of the receiver
//     that the user has to be able to see.

#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "imgui.h"

#include "decimate.h"
#include "iq.h"
#include "proto.h"
#include "source.h"

namespace sweepui {

class IqView {
public:
    enum class Y2 { None = 0, Phase = 1, Freq = 2 };

    void on_burst(sweepd::proto::IqMsg &&m, double host_now);
    void clear();
    bool have() const { return have_; }
    const sweepd::proto::IqMsg &cur() const { return cur_; }

    // The time plot. Returns true once when the user lets go of the trigger
    // level line, with the new level (FS) in *new_level -- sent on release,
    // not per frame, so a drag is one reconfiguration and not sixty a second.
    // `why_empty` is drawn when there is no burst yet.
    bool draw_time(const ImVec2 &size, const SourceConfig &cfg, const char *why_empty,
                   double *new_level);
    // The lower pane: the I/Q plane and the measurement table.
    void draw_lower(const ImVec2 &size, const SourceConfig &cfg);
    // Soft-key group: channels, axes, cursors, persistence, export.
    void draw_display_group(bool synthetic);

    // Status bar readouts. Bursts per second over the last few arrivals, and
    // the fraction of wall-clock time the bursts cover -- the time sink's
    // probability-of-intercept number, as zero span's duty is (§7).
    double bursts_per_s() const { return rate_; }
    double duty() const;

    // Writes the burst, or the stretch between the time cursors when the
    // scope says so, to `path` in the format its extension names.
    bool export_file(const std::string &path, bool synthetic, std::string *err,
                     std::string *written);

    // --do commands. Each mirrors a control in draw_display_group.
    bool command(const std::vector<std::string> &a, bool synthetic, std::string *err);

private:
    void derive();                          // channel arrays from cur_
    void range(size_t *i0, size_t *i1) const;   // the stats/XY/export scope
    void restat();
    double x_of(const sweepd::proto::IqMsg &m, size_t i) const;
    double dx_of(const sweepd::proto::IqMsg &m) const;
    size_t index_at(double x) const;

    sweepd::proto::IqMsg cur_;
    bool have_ = false;

    // Derived per burst: I, Q, |x| and the secondary trace, display-ready
    // (DC removed if asked). Recomputed when the burst or a display option that
    // changes them moves.
    std::vector<float> ch_[4];
    sweepd::IqStats stats_;

    // Persistence: earlier bursts' I/Q/|x|, oldest first. Kept only while
    // the total stays under a sample budget -- sixteen 4 Mi-sample bursts
    // would be half a gigabyte of floats for a faded line.
    struct Ghost {
        double x0 = 0, dx = 0;
        std::vector<float> ch[3];
    };
    std::deque<Ghost> ghosts_;
    static constexpr size_t kGhostBudget = size_t(1) << 22;

    // Display decimation cache, one per channel, keyed by the view.
    DecimatedTrace dec_[4];
    std::vector<DecimatedTrace> ghost_dec_;
    uint64_t dec_key_seq_ = ~0ull;
    double dec_key_x0_ = 0, dec_key_x1_ = 0;
    size_t dec_key_cols_ = 0;
    int dec_key_opts_ = -1;

    // Options.
    bool show_[4] = {true, true, false, false};
    Y2 y2_ = Y2::None;
    bool remove_dc_ = false;
    bool x_samples_ = false;
    bool autoscale_ = true;
    int persist_n_ = 0;
    bool show_xy_ = true;
    bool tcur_ = false, vcur_ = false;
    bool scope_cursors_ = false;     // stats/XY/export over A..B, else the burst
    double ca_ = 0, cb_ = 0, va_ = 0.5, vb_ = -0.5;
    bool place_tcur_ = false, place_vcur_ = false;
    char export_path_[256] = "burst.sigmf-meta";

    // Axis bookkeeping.
    bool fit_x_ = true, fit_y_ = true, fit_y2_ = true, fit_xy_ = true;
    uint64_t axis_key_ = 0;
    double view_x0_ = 0, view_x1_ = 1;
    bool level_drag_ = false;
    double level_drag_v_ = 0;

    // Arrival times for the rate.
    std::deque<double> arrivals_;
    double rate_ = 0;
    uint64_t nburst_ = 0;            // local burst counter: the decimation key
    std::string msg_;                // last export result, shown in the group
    bool msg_err_ = false;
};

}  // namespace sweepui
