// The sweepui shell. CLAUDE.md §6.
//
// Layout, per §6: "large trace graticule on top, waterfall below with a
// splitter, soft-key column on the right", with a status bar carrying the
// numbers that say whether what is on screen can be believed.
//
// §10.3 built the parts that are hard to get right and expensive to retrofit:
// the frame path, the decimation, the waterfall's texture ring and the
// frequency/amplitude entry. §10.5 adds the instrument on top of it -- four
// traces with hold and average, eight markers with peak search, and export --
// none of which changed anything below the FrameSource interface.
//
// The arithmetic all lives outside this file: trace.h holds what a hold does
// with an unmeasured bin, marker.h holds what a peak search counts as a peak,
// export.h holds what a saved file says about itself. What is left here is
// layout, input and the decision of what to draw -- which is the part that
// cannot be unit-tested, so it is kept as thin as it can be made.
//
// Zero span (§6) needed no new frame path. It arrives as an ordinary 0x02
// frame whose x_kind says the axis is time (proto.h, "one trace format"), so
// the traces, the holds, the averaging, the markers, the waterfall and the CSV
// export all serve it unchanged. What this file gained is the part that could
// not be shared: an axis label, a formatter that knows seconds from hertz, and
// a soft-key group that replaces Frequency -- because "centre and span" is not
// what is being controlled when the LO is parked.

#pragma once

#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "imgui.h"

#include "ampcorr.h"
#include "artifacts.h"
#include "decimate.h"
#include "export.h"
#include "imagecheck.h"
#include "iqview.h"
#include "marker.h"
#include "measure.h"
#include "nf.h"
#include "persist.h"
#include "segments.h"
#include "source.h"
#include "trace.h"
#include "tracknorm.h"
#include "waterfall.h"

struct GLFWwindow;

namespace sweepui {

struct AppOptions {
    int width = 1600;
    int height = 950;
    size_t waterfall_cols = 2048;
    size_t waterfall_rows = 1024;
    bool vsync = true;

    // Off by default: §6's 60 fps target is only meaningful if something is
    // measuring it, and with vsync on the number is pinned to the display.
    // --no-vsync plus this is the configuration the target is checked in.
    bool show_perf = false;

    // Render `capture_after` frames, write the framebuffer to this path, and
    // exit. It exists so the display can be inspected from a machine whose
    // compositor will not let an external tool screenshot a window, and it is
    // the same code path as §6's screenshot export. PNG unless the path ends in
    // ".ppm", which is kept because a raw PPM needs no decoder at all when the
    // question is whether the encoder is what broke.
    std::string capture_path;
    int capture_after = 60;

    // Commands to run once, after `command_after` frames have been drawn with a
    // sweep in hand. This is the instrument's remote control, and it exists
    // because it is the only way the soft keys get exercised on this bench:
    // the development host runs native Wayland, where the compositor refuses
    // synthetic input, so no external tool can click a GLFW window. §10.4
    // shipped its controls untested for exactly that reason.
    //
    // It is not a debug hatch. Every command goes through the same method the
    // corresponding soft key calls, so a command that works is evidence about
    // the button, and driving an instrument from a script is a thing people
    // want from instruments.
    std::vector<std::string> commands;
    int command_after = 30;

    // §7's amplitude correction, loaded at startup. Empty means none, which is
    // the default and which leaves the instrument in dBFS with UNCAL lit.
    std::string ampcorr_path;
};

class App {
public:
    App(std::unique_ptr<FrameSource> source, const AppOptions &opt);
    ~App();

    bool init(std::string *err);
    void run();

private:
    void pump_source();
    void draw();
    void draw_trace(const ImVec2 &size);
    void draw_marker_overlay();
    void draw_waterfall(const ImVec2 &size);
    // How a soft key reports the source's answer: applied, or refused with the
    // source's own sentence. Passed into the groups that live in their own
    // function so all of them report the same way.
    using ApplyFn = std::function<void(bool ok, const std::string &err,
                                       const char *what)>;

    void draw_softkeys();
    void draw_system_group();
    // §6 lists gain under Amplitude; raw IQ needs it too.

    // §6's zero span. Replaces the Frequency group while the mode is live.
    void draw_zero_span_group(const ApplyFn &apply);
    void draw_rx_gain(const ApplyFn &apply);
    // Raw IQ (the time sink). Replaces Frequency and the trace groups while
    // the mode is live; the display half lives in iqview.h.
    void draw_iq_group(const ApplyFn &apply);
    void iq_set_run(int run);
    void draw_tracking_group(const ApplyFn &apply);
    void draw_trace_group();
    void draw_marker_group();
    void draw_marker_table();
    void draw_export_group();
    void draw_status_bar();
    void draw_about();
    void note(const std::string &msg, bool is_error);
    void reset_view_to_span();
    bool write_capture(int w, int h, const std::string &path);

    // The bin grid of the trace currently on screen. Invalid until a frame has
    // arrived, which is why every caller checks.
    TraceAxis axis() const;

    // Formats an x value in whichever unit the live axis carries: Hz while
    // sweeping, seconds in zero span. Everything that puts an x value on screen
    // goes through this, so no readout can be labelled in the wrong unit.
    std::string fmt_x(double x) const;
    // The data a marker should read: its chosen trace if that trace is visible,
    // otherwise nothing. A marker reading a blanked trace would report a number
    // that is not on the screen it is drawn over.
    const std::vector<float> *marker_data(const Marker &m) const;
    // Peak search over what is visible, not over the whole trace.
    PeakOptions search_opts() const;
    // 0 peak, 1 next, 2 left, 3 right. False with a reason: the reason is
    // the useful part, because "no peak found" on a screen with a visible
    // signal reads as a broken search rather than as a threshold setting.
    bool run_search(int which, std::string *err);
    // These report their own outcome as an on-screen note, because that is
    // what the soft key needs. They *also* return it, because --do needs a
    // script to be able to tell that a save failed -- an export that reports
    // success it did not have is the §7 failure applied to a file.
    bool export_csv(const std::string &path, std::string *err);
    bool save_state(const std::string &path, std::string *err);
    bool load_state(const std::string &path, std::string *err);

    // §7's amplitude correction. Both clear the held and averaged traces, for
    // the reason §10.5 clears them across a retune: a bin corrected by -12 dB
    // and the same bin uncorrected are not two measurements of one thing, and a
    // Max Hold spanning the load would be a picture of neither.
    bool load_ampcorr(const std::string &path, std::string *err);

    // Reads an RX gain table file, checks it with the daemon's own parser so a
    // bad file is refused here with a line number, and sends it.
    bool load_gain_table(const std::string &path, std::string *err);
    bool load_builtin_gain_table(std::string *err);
    void unload_ampcorr();

    // Whether the amplitude axis may call itself dBm. False -- meaning UNCAL
    // stays lit and the units stay dBFS -- unless a correction is loaded *and*
    // it covers the whole span being displayed. A span running off the end of
    // the correction is partly corrected and partly not, and there is no honest
    // single label for that trace (ampcorr.h, "no extrapolation, ever").
    bool uncal() const;
    // The swept range in Hz, for the coverage test above. Zero span reports its
    // parked centre as both ends, since that is the only frequency it measures.
    void measured_span(double *lo, double *hi) const;

    // How the live sweep differs from the conditions the correction file
    // records for itself -- RX gain, span (and so hop order), detector. Empty
    // when they agree or when the file did not say. Reported, never enforced;
    // the argument is in ampcorr.h.
    std::string corr_condition_mismatch() const;

    // One line of the vocabulary in AppOptions::commands. Returns false with a
    // reason; the caller reports it. See the table in app.cpp.
    bool run_command(const std::string &line, std::string *err);

    std::unique_ptr<FrameSource> src_;
    AppOptions opt_;
    GLFWwindow *win_ = nullptr;

    SourceConfig cfg_;
    SourceStatus status_;
    sweepd::proto::TraceMsg cur_;
    bool have_frame_ = false;
    DecimatedTrace dec_[TraceStore::kCount];
    Waterfall wf_;

    TraceStore traces_;
    MarkerSet markers_;

    // §7. Applied at the single point a frame enters the pipeline, so the
    // waterfall, all four traces, the markers and the CSV all read the same
    // corrected numbers (ampcorr.h, "where it runs").
    AmpCorrection corr_;
    // Bins in the last frame that fell outside the correction and were left
    // uncorrected. Reported rather than hidden: it is the number that explains
    // why the units still say dBFS with a correction loaded.
    size_t corr_uncovered_ = 0;

    // ------------------------------------------------- tracking generator
    //
    // The stored thru, applied at the same point as the amplitude correction
    // and for the same reason: every trace, marker, hold and export downstream
    // reads one array (tracknorm.h). When it applies, the units are dB of S21
    // and the amplitude correction is not applied on top -- a ratio of two
    // received levels has already cancelled the front end it corrects.
    TrackNorm tnorm_;
    bool tnorm_on_ = true;
    bool tnorm_applied_ = false;       // the frame on screen is normalized
    std::string tnorm_why_;            // why it is not, when a thru is held
    double tnorm_comp_db_ = 0;         // attenuation compensation in effect
    int tnorm_sweeps_ = 4;
    float tsa_atten_in_ = 50.0f;
    // Reflection: the stored reference is an open or a short through a
    // bridge/coupler, so the normalized trace is S11 (SATSAGEN's return-loss
    // setup). Return loss is its negative, the way it is usually quoted.
    bool tnorm_reflect_ = false;
    bool tnorm_rl_ = false;
    std::string tnorm_caveat_;
    char buf_tsa_off_[32] = {};

    // The isolation floor ("TX Off"): sweeps taken with the stimulus silenced,
    // drawn as a line so a point is visibly at or near "nothing was sent".
    // Sequenced across the daemon round trip, because the frames in flight
    // when stim_off is sent still carry the stimulus.
    enum class FloorPhase { Idle, Silencing, Capturing, Restoring };
    FloorPhase floor_phase_ = FloorPhase::Idle;
    TrackNorm floor_;
    int floor_sweeps_ = 3;
    int floor_skip_ = 0;
    bool floor_show_ = true;
    void floor_step(const sweepd::proto::TraceMsg &f, const TrackGrid &g);
    void draw_floor_overlay();
    std::vector<double> floor_x_, floor_y_;
    TrackGrid track_grid(const sweepd::proto::TraceMsg &f) const;
    // The y-axis unit for whatever is on screen: dB (S21) when normalized,
    // else dBFS or dBm as §7 decides.
    const char *units() const;

    // §6's min/max band, on by default. It is the part of the decimation that
    // carries the information a single line throws away, but four overlapping
    // bands are unreadable, so it is a control rather than a constant.
    bool show_band_ = true;
    // The SATSAGEN look: the min/max spread drawn solid and the points joined,
    // so full-span noise reads as a filled band rather than a line along its
    // top. Same data and decimation, a different picture of it.
    bool fill_band_ = false;

    // §6's persistence / density display, drawn under the traces in the trace
    // plot itself so it shares their axes exactly (persist.h). Laid out on the
    // frame's span and the plot's level range; a change to either clears it.
    Persistence persist_;
    bool persist_on_ = false;
    float persist_tau_ = 2.0f;       // seconds; 0 = infinite
    bool persist_dirty_ = false;
    double persist_t_last_ = 0;
    unsigned persist_tex_ = 0;
    size_t persist_tex_w_ = 0, persist_tex_h_ = 0;
    std::vector<float> persist_i_;
    std::vector<unsigned char> persist_rgba_;
    void persist_feed(const sweepd::proto::TraceMsg &f);

    // ------------------------------------------------ band measurements
    //
    // Channel power / PSD and occupied bandwidth over a band of one trace
    // (measure.h). Recomputed every displayed frame from the trace as drawn,
    // so holds and averages are measured as they are seen.
    enum class MeasMode { Off, ChannelPower, Obw };
    MeasMode meas_mode_ = MeasMode::Off;
    double meas_f0_ = 0, meas_bw_ = 0;      // band centre and width, Hz
    size_t meas_trace_ = 0;
    float meas_obw_pct_ = 99.0f;
    char buf_meas_c_[32] = {}, buf_meas_w_[32] = {};
    BandResult meas_cp_;
    ObwResult meas_obw_;
    void meas_update();
    void draw_measure_group();
    void draw_measure_overlay();
    bool meas_from_markers(std::string *err);

    // ------------------------------------------------ marker monitor
    //
    // SATSAGEN's "MKR Monitor": the active marker's level against time, one
    // reading per sweep, so a slow drift or an intermittent signal at one
    // frequency is visible as a line rather than remembered.
    bool mon_on_ = false;
    std::vector<double> mon_t_, mon_v_;
    double mon_t0_ = 0;
    void mon_record();
    void draw_marker_monitor();

    // ------------------------------------------------ signal generator
    //
    // The setting being edited. Seeded from the source's report on the first
    // config and whenever the source changes it underneath (tracking took the
    // TX); applied as a whole by the TX ON key.
    sweepd::GenConfig gen_edit_;
    bool gen_seeded_ = false;
    bool gen_sweep_ = false;
    char buf_gen_f_[32] = {}, buf_gen_s0_[32] = {}, buf_gen_s1_[32] = {}, buf_gen_st_[32] = {};
    void draw_generator_group();

    // ------------------------------------------------ noise figure (nf.h)
    //
    // Four readings per frequency -- cal on/off, DUT on/off -- each the
    // channel power over nf_bw around the point, averaged in power over
    // nf_sweeps sweeps. The source is switched by hand for now; the steps say
    // what to connect and when. The frequencies are fixed at the first
    // reading, so the four are of the same points.
    EnrTable nf_enr_;
    char buf_nf_enr_[256] = "enr.csv";
    float nf_enr_fixed_ = 15.0f;     // used when no table is loaded
    int nf_points_ = 11;
    float nf_bw_khz_ = 1000.0f;
    int nf_sweeps_ = 10;
    std::vector<double> nf_f_;
    std::vector<double> nf_acc_[4];
    size_t nf_got_[4] = {0, 0, 0, 0};
    int nf_capturing_ = -1;
    size_t nf_left_ = 0;
    std::vector<NfResult> nf_res_;
    std::vector<double> nf_enr_at_;
    void nf_feed(const sweepd::proto::TraceMsg &f);
    void nf_start(int which);
    void nf_compute();
    bool nf_save(const std::string &path, std::string *err);
    void draw_nf_group();
    bool gen_send(bool on, std::string *err);
    bool persist_draw();   // true when it drew something

    // ------------------------------------------------------ §7's image check
    //
    // A two-pass measurement, so it is a state machine rather than a button: it
    // averages sweeps on the ordinary LO grid, asks the source to phase the
    // grid, averages again, compares, and puts the grid back. It has to survive
    // the source refusing at any step, and it must always try to restore the
    // grid -- leaving the instrument on a deliberately-wrong grid with nothing
    // saying so is the §7 failure in its purest form.
    enum class ImgPhase { Idle, PassA, Shifting, PassB, Restoring, Done, Failed };
    ImgPhase img_phase_ = ImgPhase::Idle;
    PassAccumulator img_acc_;
    ImagePass img_a_, img_b_;
    std::vector<SegmentBand> img_segs_a_;
    ImageCheckConfig img_cfg_;
    ImageCheckResult img_res_;
    size_t img_want_sweeps_ = 60;
    double img_shift_hz_ = 0;
    std::string img_status_;
    bool img_show_ = true;        // draw the findings over the trace
    // Frames arriving between the request and the source's new `config` are
    // still from the old grid. One frame of settling after the config changes
    // is what keeps them out of the wrong pass.
    int img_settle_ = 0;

    bool start_image_check(std::string *err);
    void abort_image_check(const std::string &why);
    void step_image_check();
    void draw_image_check_group();
    void draw_image_overlay();

    // ------------------------------------------ receiver-artifact marking
    //
    // artifacts.h. On by default, unlike the segment overlay: that one draws the
    // machinery, this one says which responses on screen are likely the
    // receiver itself, which is the thing a user reading a quiet band gets
    // wrong. It never touches the trace -- the marks ride beside it.
    //
    // Fed from its own rolling average of every sweep rather than from a
    // display trace, so it does not change meaning when the user switches
    // trace 1 to Max Hold; refreshed every art_sweeps_ sweeps.
    //
    // Off by default since 2026-10-21. The built-in gain table lowered the
    // floor 4.6 dB, so at full span the LO skirt 1-2.5 MHz from every LO
    // (the same -79.5 dBFS under both tables) now stands out, and is correctly
    // marked "LO spur?" hundreds of times over (docs/characterization.md,
    // "A built-in single table"). Turn it on (Artifacts, `art 1`) for a
    // narrower span, where it is most useful.
    bool art_on_ = false;
    ArtifactConfig art_cfg_;
    PassAccumulator art_acc_;
    size_t art_sweeps_ = 16;
    ArtifactResult art_res_;
    // A terminated-input recording. Capturing is a mode: while it runs, every
    // sweep goes into art_base_acc_ and the user is told the input must be
    // terminated.
    ArtifactBaseline art_base_;
    PassAccumulator art_base_acc_;
    bool art_base_capturing_ = false;
    size_t art_base_sweeps_ = 60;
    char buf_base_[256] = "sweepsa-baseline.csv";

    void art_reset();
    void art_feed(const TraceAxis &fax, const float *db, size_t n);
    bool art_start_baseline(std::string *err);
    // The flags under `x` on the trace on screen, or 0 when there is no result
    // for this axis.
    uint8_t art_flags_at(double x) const;
    void draw_artifact_group();
    void draw_artifact_overlay();

    // §7's segment boundary overlay. Off by default: it is a debug view, and
    // annotating every sweep with the machinery that produced it would be its
    // own kind of dishonesty about what the trace is. Rebuilt from cfg_ whenever
    // the plan changes -- see rebuild_segments().
    bool show_segments_ = false;
    std::vector<SegmentBand> segs_;
    std::string segs_err_;
    void rebuild_segments();
    void draw_segment_overlay();

    // Deferred to the end of the frame: the framebuffer has to be read after
    // the draw and before the swap, which is nowhere near the button.
    std::string want_png_;

    int cmd_frames_ = 0;
    size_t cmd_next_ = 0;
    int cmd_wait_ = 0;
    bool commands_done_ = false;

    // View. Kept in axis units and dB rather than in plot units so the entry
    // fields and the axes cannot disagree about what is displayed.
    //
    // The axis units are Hz while sweeping and seconds in zero span, which is
    // why these are no longer named for Hz: a variable called view_start_hz_
    // holding 0.0085 seconds is exactly the quiet mislabelling that the wire
    // format refuses to do, one layer up.
    double view_x0_ = 0;
    double view_x1_ = 0;
    bool view_dirty_ = true;      // push view_* into the axis this frame
    float ref_db_ = 0.0f;
    float db_per_div_ = 10.0f;
    // Linear amplitude graticule. The data stays in dB everywhere; only the Y
    // axis maps it (see lin_axis() in app.cpp). Off is the log display.
    bool lin_scale_ = false;
    bool lin_active() const { return lin_scale_ && !(tnorm_applied_ && tnorm_rl_); }
    static constexpr int kDivisions = 10;
    bool amp_dirty_ = true;

    // Waterfall colour range. Separate from the trace's, because the useful
    // dynamic range for a colour ramp is much narrower than for a graticule --
    // 100 dB of turbo is mostly one colour.
    float wf_ref_db_ = -40.0f;
    float wf_range_db_ = 60.0f;

    // Whether the frequency keys retune the source or only move the display.
    // On by default because that is what the keys do on an instrument; the
    // switch exists because the two are genuinely different operations and one
    // of them costs a profile rebuild (see draw_softkeys).
    bool auto_tune_ = true;

    bool paused_ = false;

    // Raw IQ. Run re-arms on every burst, Single arms once and drops to Stop
    // when it arrives, Stop disarms. iq_armed_ is "a request is outstanding",
    // which is what keeps exactly one burst in flight (proto.h, "0x03 iq").
    enum IqRun { kIqRun = 0, kIqStop = 1, kIqSingle = 2 };
    IqView iqv_;
    int iq_run_ = kIqRun;
    bool iq_armed_ = false;
    double iq_armed_at_ = 0;
    char buf_iq_center_[48] = "";
    // Edit buffers for the trigger fields: refreshed from the source while no
    // field is being edited, sent when an edit is committed.
    float iq_lv_ = 0, iq_hy_ = 0, iq_pre_ = 0, iq_au_ = 0;
    bool show_about_ = false;
    float split_frac_ = 0.58f;    // trace's share of the vertical space
    float softkey_w_ = 320.0f;

    // Where the trace's plot area landed on screen, so the waterfall can line
    // up with it rather than with the panel it lives in.
    float plot_x0_ = 0;
    float plot_w_ = 0;

    // Entry field buffers. Text rather than parsed values, because §6 wants
    // "2.4g" to be typeable and half-typed input must not retune anything.
    bool any_frame_seen_ = false;   // latch for the --do runner; see run()
    char buf_trig_level_[32] = {0};
    char buf_center_[32] = {0};
    char buf_span_[32] = {0};
    char buf_start_[32] = {0};
    char buf_stop_[32] = {0};

    // Export paths. Editable, because "where did it go" is the first question
    // an export button provokes and a fixed path answers it badly.
    char buf_csv_[256] = "sweepsa-trace.csv";
    char buf_png_[256] = "sweepsa-screen.png";
    char buf_state_[256] = "sweepsa-state.json";
    char buf_corr_[256] = "sweepsa-ampcorr.csv";
    char buf_gt_[256] = "gaintables/satsagen_ad9361_cgt.txt";
    // The RX gain being edited. Follows the source's config until the user
    // touches the control, so a gain changed elsewhere (argv, --do, another
    // client) shows up instead of being overwritten.
    int gain_edit_db_ = 0;
    bool gain_editing_ = false;

    struct Note {
        std::string text;
        bool error;
        double t;
    };
    std::deque<Note> notes_;

    // Frame-time history for the §6 performance claim.
    std::vector<float> ft_ms_;
    size_t ft_head_ = 0;
    double last_report_t_ = 0;
    double worst_ms_ = 0;
    double dec_us_ = 0;           // time spent decimating, last frame
    int frames_ = 0;
};

}  // namespace sweepui
