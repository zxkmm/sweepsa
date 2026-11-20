#include "app.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

#include "imgui.h"
#include "implot.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

#include "freq.h"
#include "gaintable.h"
#include "gl.h"

namespace sweepui {
namespace {

using sweepd::Detector;
using sweepd::Window;

const char *kDetectorNames[] = {"Peak", "Neg Peak", "Sample", "RMS/Avg"};
const Detector kDetectors[] = {Detector::Peak, Detector::NegativePeak, Detector::Sample,
                               Detector::RMS};
// Order must match the combo string in draw_softkeys(). dsp.h's window_name()
// is the display name elsewhere; here the list is positional.
const Window kWindows[] = {Window::Rectangular, Window::Hann, Window::BlackmanHarris,
                           Window::FlatTop};

// Order must match the combo string in draw_trace_group().
const TraceMode kTraceModes[] = {TraceMode::ClearWrite, TraceMode::MaxHold,
                                 TraceMode::MinHold,    TraceMode::Average,
                                 TraceMode::View,       TraceMode::Blank};
constexpr int kTraceModeCount = 6;

// One colour per trace, and the same colour everywhere it appears: the line,
// the soft-key swatch, the marker that reads it and the CSV column order. Four
// hues that stay distinguishable on a dark graticule and do not collide with
// the amber and red the badges use for warnings.
const ImVec4 kTraceColors[TraceStore::kCount] = {
    ImVec4(0.45f, 1.00f, 0.55f, 1.0f),   // green
    ImVec4(0.45f, 0.78f, 1.00f, 1.0f),   // blue
    ImVec4(1.00f, 0.80f, 0.38f, 1.0f),   // amber
    ImVec4(1.00f, 0.52f, 0.88f, 1.0f),   // magenta
};

void glfw_error(int code, const char *desc) {
    std::fprintf(stderr, "glfw error %d: %s\n", code, desc);
}

// ImPlot hands the axis formatter raw Hz; the instrument convention is to show
// the unit that makes the number readable at the current zoom.
int format_hz_axis(double value, char *buf, int size, void *) {
    const std::string s = format_frequency(value, 6);
    return std::snprintf(buf, size_t(size), "%s", s.c_str());
}

// Zero span's axis. A separate formatter rather than a flag inside the one
// above, because ImPlot holds the pointer across the frame and the two must not
// be able to swap under it mid-draw.
int format_time_axis(double value, char *buf, int size, void *) {
    const std::string s = format_seconds(value, 3);
    return std::snprintf(buf, size_t(size), "%s", s.c_str());
}

// ---------------------------------------------------------- linear graticule
//
// "Lin" on a swept analyzer means the Y axis is linear in *amplitude* (volts),
// ref level at the top graticule line and zero at the bottom -- not linear in
// power. So height goes as 10^(dB/20): a 6 dB drop halves it, and 50 % AM reads
// as a trace swinging between half and one-and-a-half times the carrier.
//
// Every value stays in dB: traces, markers, holds, overlays, the waterfall and
// the CSV. Only the axis transform changes, through ImPlot's custom scale, so no
// overlay needs a second code path and none can drift out of step with the trace.
double db_to_amp(double db, void *) { return std::pow(10.0, db / 20.0); }
double amp_to_db(double a, void *) {
    // Zoom and pan work in amplitude and can push the bottom below zero; that
    // has no dB value, so it is clamped rather than handed back as NaN.
    return 20.0 * std::log10(std::max(a, 1e-30));
}

// How far below the ref level the bottom of a lin graticule sits, in dB. Zero
// amplitude is -inf dB; 300 dB down is 1e-15 of the top line, which is zero to
// any pixel.
constexpr double kLinFloorDb = 300.0;

// Ten amplitude divisions under `ref_db`, as tick positions in dB with labels in
// `unit_scale`-converted amplitude. The SI prefix is chosen once from the top
// line so every label shares it, and it goes into `prefix` for the axis title.
void lin_ticks(double ref_db, double unit_scale, double pos[11], char labels[11][16],
               const char **prefix) {
    const double top = db_to_amp(ref_db, nullptr) * unit_scale;
    static const char *pfx[] = {"p", "n", "\xc2\xb5", "m", "", "k"};
    int e = int(std::floor(std::log10(std::max(top, 1e-12)) / 3.0));
    e = std::max(-4, std::min(1, e));
    const double div = std::pow(1000.0, e);
    *prefix = pfx[e + 4];
    for (int k = 0; k <= 10; ++k) {
        const double frac = double(k) / 10.0;
        pos[k] = k ? ref_db + 20.0 * std::log10(frac) : ref_db - kLinFloorDb;
        std::snprintf(labels[k], 16, "%.3g", top * frac / div);
    }
}

// A badge: coloured text in a box, for the things §7 insists stay visible.
void badge(const char *text, ImU32 col, const char *tooltip) {
    ImGui::PushStyleColor(ImGuiCol_Button, col);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, col);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, col);
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(16, 16, 20, 255));
    ImGui::SmallButton(text);
    ImGui::PopStyleColor(4);
    if (tooltip && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tooltip);
}

double mono_now() {
    return glfwGetTime();
}

}  // namespace

App::App(std::unique_ptr<FrameSource> source, const AppOptions &opt)
    : src_(std::move(source)), opt_(opt) {
    ft_ms_.assign(240, 0.0f);

    // §7. Loaded before the first frame so nothing uncorrected ever reaches a
    // hold. A bad path is reported and the instrument starts in dBFS -- which
    // is the safe direction, and the only one that keeps UNCAL honest.
    if (!opt_.ampcorr_path.empty()) {
        std::snprintf(buf_corr_, sizeof buf_corr_, "%s", opt_.ampcorr_path.c_str());
        std::string err;
        if (!corr_.load_csv(opt_.ampcorr_path, &err))
            std::fprintf(stderr,
                         "sweepui: --ampcorr: %s\n"
                         "         continuing in dBFS with UNCAL lit.\n",
                         err.c_str());
        else
            std::fprintf(stderr, "sweepui: amplitude correction: %zu points, %.6g - %.6g MHz\n",
                         corr_.size(), corr_.f_min_hz() / 1e6, corr_.f_max_hz() / 1e6);
    }
}

App::~App() {
    wf_.shutdown();
    if (win_) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImPlot::DestroyContext();
        ImGui::DestroyContext();
        glfwDestroyWindow(win_);
        glfwTerminate();
    }
}

bool App::init(std::string *err) {
    glfwSetErrorCallback(glfw_error);
    if (!glfwInit()) {
        *err = "glfwInit failed (no display?)";
        return false;
    }

    // §6: OpenGL 3.3. Core profile, because the waterfall's shader is written
    // for it and a compatibility context would let a mistake there work here
    // and fail on a stricter driver.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

    win_ = glfwCreateWindow(opt_.width, opt_.height, "sweepsa", nullptr, nullptr);
    if (!win_) {
        glfwTerminate();
        *err = "could not create an OpenGL 3.3 window";
        return false;
    }
    glfwMakeContextCurrent(win_);
    glfwSwapInterval(opt_.vsync ? 1 : 0);

    const char *missing = nullptr;
    if (!gl::load([](const char *n) { return (void *)glfwGetProcAddress(n); }, &missing)) {
        *err = std::string("this OpenGL context is missing ") + missing +
               " -- 3.3 core is required";
        return false;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.IniFilename = nullptr;   // the layout is code, not a stray ini file

    ImGui::StyleColorsDark();
    ImGuiStyle &st = ImGui::GetStyle();
    st.WindowRounding = 0.0f;
    st.FrameRounding = 2.0f;
    st.WindowBorderSize = 0.0f;
    st.ItemSpacing = ImVec2(6, 5);

    ImPlot::GetStyle().UseLocalTime = false;

    if (!ImGui_ImplGlfw_InitForOpenGL(win_, true)) {
        *err = "ImGui GLFW backend failed";
        return false;
    }
    if (!ImGui_ImplOpenGL3_Init("#version 330")) {
        *err = "ImGui OpenGL3 backend failed";
        return false;
    }

    std::string werr;
    if (!wf_.init(opt_.waterfall_cols, opt_.waterfall_rows, &werr)) {
        *err = "waterfall: " + werr;
        return false;
    }

    cfg_ = src_->config();
    reset_view_to_span();
    // Here as well as in pump_source: pump compares against the previous config
    // to decide whether the plan moved, and this is the assignment that gives it
    // one. Without it the first --do "segments 1" is refused for want of a plan
    // that has in fact been available since startup.
    rebuild_segments();

    // Trace 1 lives; the other three are blanked until asked for, which is what
    // an instrument does on power-up. Marker 1 exists but is off.
    traces_[0].set_mode(TraceMode::ClearWrite);
    return true;
}

void App::reset_view_to_span() {
    if (cfg_.zero_span()) {
        // The whole captured window. Free run's axis starts at 0: t is measured
        // from the start of the frame, and the frames are not contiguous with
        // each other, so an absolute time would be a number with no meaning
        // attached to it. A *triggered* frame does have an origin -- the edge --
        // and the pre-trigger points sit before it, at negative time.
        const double dt = double(cfg_.dt_ns) * 1e-9;
        view_x0_ = -double(cfg_.trig_pretrig_points) * dt;
        view_x1_ = view_x0_ + double(cfg_.n_points) * dt;
        if (!(view_x1_ > view_x0_)) view_x1_ = view_x0_ + 1e-3;
    } else {
        view_x0_ = double(cfg_.f_start_hz);
        view_x1_ = double(cfg_.f_stop_hz);
    }
    view_dirty_ = true;
}

void App::note(const std::string &msg, bool is_error) {
    notes_.push_back({msg, is_error, mono_now()});
    while (notes_.size() > 8) notes_.pop_front();
    if (is_error) std::fprintf(stderr, "%s\n", msg.c_str());
}

void App::pump_source() {
    const SourceConfig prev = cfg_;
    cfg_ = src_->config();
    status_ = src_->status();
    for (std::string e; src_->take_error(&e);) note(e, true);

    // Raw IQ. Entering or leaving the mode drops what was on screen: a burst
    // from before is a picture of a different configuration.
    if (cfg_.iq_mode != prev.iq_mode) {
        iqv_.clear();
        iq_armed_ = false;
        if (cfg_.iq_mode) iq_run_ = kIqRun;
    }
    if (cfg_.iq_mode) {
        sweepd::proto::IqMsg m;
        while (src_->poll_iq(&m)) {
            iq_armed_ = false;
            if (!paused_) iqv_.on_burst(std::move(m), mono_now());
            if (iq_run_ == kIqSingle) iq_run_ = kIqStop;
        }
        // One outstanding request, re-issued when it has been answered. This
        // is the whole flow control: the link's pace is the burst rate.
        // Re-sent once a second while waiting: an arm is idempotent at the
        // source, and a repeat is what recovers one lost to a reconnect or to
        // a reconfiguration racing it -- otherwise the view would sit "armed"
        // forever on a burst nobody is making.
        const double now = mono_now();
        if (iq_run_ != kIqStop && (!iq_armed_ || now - iq_armed_at_ > 1.0)) {
            std::string e;
            if (src_->iq_arm(&e)) {
                iq_armed_ = true;
                iq_armed_at_ = now;
            } else {
                note("IQ arm refused: " + e, true);
                iq_run_ = kIqStop;
            }
        }
    }

    // A change of span or point count makes the stored waterfall rows
    // incomparable with the new ones. They are dropped rather than allowed to
    // scroll away under a frequency axis that no longer describes them.
    // x_kind is in this test for the same reason it is in the trace store's
    // key: entering or leaving zero span changes what the axis *means*, so rows
    // measured over frequency must not scroll away underneath one labelled in
    // seconds.
    // The trigger is in this test because it moves the axis origin: rows
    // measured against a free-running frame's 0..T would scroll away underneath
    // one labelled -t..+t and misrepresent when each of them happened.
    if (cfg_.f_start_hz != prev.f_start_hz || cfg_.f_stop_hz != prev.f_stop_hz ||
        cfg_.n_points != prev.n_points || cfg_.x_kind != prev.x_kind ||
        cfg_.dt_ns != prev.dt_ns || cfg_.trigger != prev.trigger ||
        cfg_.trig_pretrig_points != prev.trig_pretrig_points) {
        wf_.clear();
        reset_view_to_span();
        have_frame_ = false;
    }

    // §7's overlay. Rebuilt on any change to the plan itself, which is a
    // different set of fields from the one above: set_detector keeps the grid
    // and changes the axis-clearing test not at all, while a retune moves every
    // seam. Cheap either way -- 298 bands is a few microseconds -- but doing it
    // per frame would hide a reconstruction failure behind a message that
    // reappeared 60 times a second.
    if (cfg_.seg_first_lo_hz != prev.seg_first_lo_hz || cfg_.seg_step_hz != prev.seg_step_hz ||
        cfg_.keep_hz != prev.keep_hz || cfg_.blank_hz != prev.blank_hz ||
        cfg_.n_segments != prev.n_segments || cfg_.covered_lo_hz != prev.covered_lo_hz ||
        cfg_.covered_hi_hz != prev.covered_hi_hz || cfg_.x_kind != prev.x_kind) {
        rebuild_segments();
        // Every mark is a statement about *this* grid; one averaged across a
        // grid change attributes offsets to LOs that no longer exist.
        art_reset();
    }

    // Drain: every sweep goes into the waterfall *and* into the traces, and the
    // last one becomes the current frame. Draining only one per display frame
    // would silently thin the waterfall whenever the source outran the display
    // -- and, now, would make Max Hold miss sweeps and Average(N) average over
    // a different N than the one on the soft key.
    sweepd::proto::TraceMsg f;
    while (src_->poll(&f)) {
        if (paused_) continue;

        // A frame from the mode the source has just left. Entering or leaving
        // zero span flips cfg_.x_kind (and resets the view above) before the
        // frames already queued under the old mode have been drained, so the
        // first poll after "back to sweep" hands over time-axis frames. Taken,
        // one would become cur_ and the traces, and be drawn against a
        // frequency view where none of its points land -- an empty plot that
        // reported "every trace is blanked" until the first real sweep
        // arrived, which against sweepd is a whole profile rebuild later.
        // Dropped rather than kept: its axis is not the axis on screen, and
        // nothing downstream can place it.
        if (f.x_kind != cfg_.x_kind) continue;

        // §7's amplitude correction, applied here and nowhere else. Everything
        // downstream -- the waterfall, the four traces, their holds and
        // averages, the markers and the CSV -- reads the corrected array, so
        // there is no second path to drift out of sync (ampcorr.h).
        //
        // Zero span goes through apply_flat: its x axis is *time*, and the one
        // frequency it measures is the parked centre, which is not on the axis
        // at all.
        // The tracking generator's thru, which when it applies replaces the
        // amplitude correction rather than stacking on it (app.h). The capture
        // sees the raw frame, so a thru is never stored already-normalized.
        tnorm_applied_ = false;
        tnorm_comp_db_ = 0;
        if (cfg_.tracking && f.x_kind == sweepd::proto::XKind::Frequency) {
            const TrackGrid g = track_grid(f);
            if (tnorm_.capturing()) {
                std::string why;
                if (tnorm_.feed(g, f.db.data(), f.db.size(), &why))
                    note("thru stored: " + std::to_string(f.db.size()) + " points, " +
                             std::to_string(tnorm_sweeps_) + " sweeps averaged",
                         false);
                else if (!why.empty())
                    note(why, true);
            }
            floor_step(f, g);
            if (tnorm_on_ && tnorm_.have()) {
                tnorm_why_ = tnorm_.apply(g, f.db.data(), f.db.size(), &tnorm_comp_db_,
                                          &tnorm_caveat_);
                tnorm_applied_ = tnorm_why_.empty();
                if (tnorm_applied_ && tnorm_reflect_ && tnorm_rl_)
                    for (float &v : f.db) v = -v;
            } else {
                tnorm_why_.clear();
            }
        }

        if (corr_.loaded() && !tnorm_applied_) {
            if (f.x_kind == sweepd::proto::XKind::Time) {
                corr_uncovered_ =
                    corr_.apply_flat(f.db.data(), f.db.size(), double(f.f_start_hz))
                        ? 0
                        : f.db.size();
            } else {
                corr_uncovered_ =
                    corr_.apply(f.db.data(), f.db.size(), double(f.f_start_hz), f.bin_hz);
            }
        } else {
            corr_uncovered_ = 0;
        }

        wf_.push(f.db.data(), f.db.size());
        persist_feed(f);
        nf_feed(f);
        if (traces_.push(f))
            note("the sweep changed shape; held and averaged traces were cleared", false);

        // §7's image check takes *every* sweep of a pass, not the one that
        // happens to be current when the frame is drawn -- the whole defence
        // against a burst transmitter being mistaken for an image is that both
        // passes average a lot of sweeps.
        if (img_phase_ == ImgPhase::PassA || img_phase_ == ImgPhase::PassB) {
            TraceAxis fax;
            if (f.x_kind == sweepd::proto::XKind::Frequency && f.bin_hz > 0) {
                fax.x0 = double(f.f_start_hz);
                fax.dx = f.bin_hz;
                fax.n = f.db.size();
                fax.rbw_hz = f.rbw_hz;
            }
            if (img_settle_ > 0) {
                --img_settle_;
            } else if (!img_acc_.add(fax, f.db.data(), f.db.size())) {
                abort_image_check("the sweep changed shape part-way through a pass, so "
                                  "the two halves are not comparable");
            }
        }

        {
            TraceAxis fax;
            if (f.x_kind == sweepd::proto::XKind::Frequency && f.bin_hz > 0) {
                fax.x0 = double(f.f_start_hz);
                fax.dx = f.bin_hz;
                fax.n = f.db.size();
                fax.rbw_hz = f.rbw_hz;
            }
            // Not in tracking: the marks are statements about a fastlock LO grid
            // and a spectrum, and this is neither.
            if (!cfg_.tracking) art_feed(fax, f.db.data(), f.db.size());
        }

        cur_ = std::move(f);
        have_frame_ = true;
        mon_record();
    }
}

// ------------------------------------------------------- §7's image check
//
// The sequencing. The arithmetic is all in imagecheck.h, which is host-tested;
// what is here is the part that talks to a source and therefore cannot be.

bool App::start_image_check(std::string *err) {
    if (cfg_.zero_span()) {
        *err = "the image check is a sweep measurement; zero span parks the LO and has "
               "no grid to shift";
        return false;
    }
    if (img_phase_ == ImgPhase::PassA || img_phase_ == ImgPhase::Shifting ||
        img_phase_ == ImgPhase::PassB || img_phase_ == ImgPhase::Restoring) {
        *err = "an image check is already running";
        return false;
    }
    const TraceAxis ax = axis();
    if (!ax.valid()) {
        *err = "no sweep yet";
        return false;
    }
    if (!(cfg_.seg_step_hz > 0)) {
        *err = "the source does not report its LO grid, so there is nothing to phase";
        return false;
    }
    // Half a step: it inverts the sign of every bin's baseband offset and moves
    // every image by exactly one whole step (imagecheck.h). Derived from the
    // source's own reported step rather than assumed, because dc_policy=blank
    // has twice the step of splice.
    img_shift_hz_ = cfg_.seg_step_hz * 0.5;

    // Pass 1 has to be on the unshifted grid, or "shift by half a step" is a
    // shift by a whole one and the source refuses it -- and the two passes'
    // relationship would no longer be the one the verdicts are computed from.
    // Asking now also means a source that cannot do this at all refuses before
    // the accumulator has spent a minute filling up, rather than after.
    std::string serr;
    if (!src_->set_lo_shift(0.0, &serr)) {
        *err = "the source will not set its LO grid: " + serr;
        return false;
    }

    img_res_ = ImageCheckResult();
    img_segs_a_ = segs_;
    img_acc_.begin(ax, cfg_.lo_shift_hz);
    img_settle_ = 0;
    img_phase_ = ImgPhase::PassA;
    img_status_ = "pass 1 of 2, on the ordinary LO grid";
    return true;
}

void App::abort_image_check(const std::string &why) {
    // Always put the grid back. An instrument left on a deliberately-phased
    // grid draws every artifact in the wrong place and nothing on screen says
    // so, which is worse than the failure that got us here.
    std::string serr;
    if (cfg_.lo_shift_hz != 0 && !src_->set_lo_shift(0.0, &serr))
        note("image check: could not restore the LO grid -- " + serr, true);
    img_phase_ = ImgPhase::Failed;
    img_status_ = why;
    img_acc_ = PassAccumulator();
    note("image check: " + why, true);
}

void App::step_image_check() {
    std::string serr;
    switch (img_phase_) {
        case ImgPhase::Idle:
        case ImgPhase::Done:
        case ImgPhase::Failed:
            return;

        case ImgPhase::PassA:
            if (img_acc_.sweeps() < img_want_sweeps_) return;
            img_a_ = img_acc_.finish(img_segs_a_);
            if (!src_->set_lo_shift(img_shift_hz_, &serr)) {
                abort_image_check("the source refused to phase its LO grid: " + serr);
                return;
            }
            img_phase_ = ImgPhase::Shifting;
            img_status_ = "phasing the LO grid";
            return;

        case ImgPhase::Shifting: {
            // Wait for the source to confirm the new grid *and* produce a frame
            // on it. cfg_.lo_shift_hz is the source's own report, not what we
            // asked for, so a daemon that quietly did something else does not
            // get a pass B labelled as though it complied.
            if (cfg_.reconfiguring) return;
            if (std::fabs(cfg_.lo_shift_hz - img_shift_hz_) > 1.0) return;
            const TraceAxis ax = axis();
            if (!ax.valid()) return;
            img_acc_.begin(ax, cfg_.lo_shift_hz);
            img_settle_ = 1;
            img_phase_ = ImgPhase::PassB;
            img_status_ = "pass 2 of 2, on the phased LO grid";
            return;
        }

        case ImgPhase::PassB:
            if (img_acc_.sweeps() < img_want_sweeps_) return;
            img_b_ = img_acc_.finish(segs_);
            if (!src_->set_lo_shift(0.0, &serr))
                note("image check: could not restore the LO grid -- " + serr, true);
            img_phase_ = ImgPhase::Restoring;
            img_status_ = "restoring the LO grid";
            return;

        case ImgPhase::Restoring:
            if (cfg_.reconfiguring) return;
            if (std::fabs(cfg_.lo_shift_hz) > 1.0) return;
            img_res_ = run_image_check(img_a_, img_b_, img_cfg_);
            img_acc_ = PassAccumulator();
            if (!img_res_.valid) {
                img_phase_ = ImgPhase::Failed;
                img_status_ = img_res_.err;
                note("image check: " + img_res_.err, true);
                return;
            }
            img_phase_ = ImgPhase::Done;
            img_status_ = img_res_.findings.empty()
                              ? "no response moved with the LO grid"
                              : std::to_string(img_res_.findings.size()) +
                                    " responses moved with the LO grid";
            note("image check: " + img_status_, false);
            return;
    }
}

// ---------------------------------------------------- §7 segment boundaries

void App::rebuild_segments() {
    segs_.clear();
    segs_err_.clear();

    // Zero span has one acquisition and a time axis. A "segment boundary" there
    // is not a narrower version of this question, it is a different one, and
    // drawing frequency-plan geometry over seconds would be nonsense that looks
    // like information.
    if (cfg_.zero_span()) {
        segs_err_ = "zero span has no segments -- the LO is parked";
        return;
    }

    SegmentPlanInfo info;
    info.first_lo_hz = cfg_.seg_first_lo_hz;
    info.step_hz = cfg_.seg_step_hz;
    info.keep_hz = cfg_.keep_hz;
    info.blank_hz = cfg_.blank_hz;
    info.tune_min_hz = double(cfg_.tune_lo());
    info.tune_max_hz = double(cfg_.tune_hi());
    info.covered_lo_hz = cfg_.covered_lo_hz;
    info.covered_hi_hz = cfg_.covered_hi_hz;
    info.n_segments = cfg_.n_segments;
    info.splice = cfg_.splice;

    if (!reconstruct_segments(info, &segs_, &segs_err_) && show_segments_)
        note("segment overlay: " + segs_err_, true);
}

// Drawn inside BeginPlot/EndPlot, under the traces: this annotates the
// measurement, it does not compete with it.
void App::draw_segment_overlay() {
    if (!show_segments_ || segs_.empty()) return;

    const ImPlotRect lim = ImPlot::GetPlotLimits();
    const SegmentOverlay ov =
        build_overlay(segs_, lim.X.Min, lim.X.Max, double(ImPlot::GetPlotSize().x));
    if (ov.too_dense || ov.empty()) return;

    ImDrawList *dl = ImPlot::GetPlotDrawList();
    const float y0 = float(ImPlot::PlotToPixels(lim.X.Min, lim.Y.Min).y);
    const float y1 = float(ImPlot::PlotToPixels(lim.X.Min, lim.Y.Max).y);

    // Three visual weights for the three different things (segments.h). The
    // seam is the faintest because there are as many of them as centres and the
    // centre is the one the +4.5 dB comb sits on.
    const ImU32 c_seam = IM_COL32(120, 130, 150, 90);
    const ImU32 c_centre = IM_COL32(150, 190, 255, 130);
    const ImU32 c_ovld = IM_COL32(255, 110, 90, 190);
    const ImU32 c_blank = IM_COL32(150, 190, 255, 40);

    for (double s : ov.seams) {
        const float x = float(ImPlot::PlotToPixels(s, lim.Y.Min).x);
        dl->AddLine(ImVec2(x, y0), ImVec2(x, y1), c_seam, 1.0f);
    }

    for (size_t i = 0; i < ov.centres.size(); ++i) {
        // The blanked span first, so the centre line sits on top of it. It is
        // usually far narrower than a pixel -- 200 kHz of a 40 MHz segment --
        // so it is widened to a visible minimum. That is a deliberate lie about
        // *width* in service of the truth about *position*, and the Segments
        // panel says the real figure.
        float bx0 = float(ImPlot::PlotToPixels(ov.blank_lo[i], lim.Y.Min).x);
        float bx1 = float(ImPlot::PlotToPixels(ov.blank_hi[i], lim.Y.Min).x);
        if (bx1 - bx0 < 3.0f) {
            const float mid = 0.5f * (bx0 + bx1);
            bx0 = mid - 1.5f;
            bx1 = mid + 1.5f;
        }
        dl->AddRectFilled(ImVec2(bx0, y1), ImVec2(bx1, y0), c_blank);

        // §5: an overloaded front end must never be silent. The status bar
        // counts them; this says *which*, which is the only form of that number
        // a user can act on -- it names the frequencies to attenuate.
        const bool ovld = have_frame_ && cur_.segment_overloaded(size_t(ov.centre_seg[i]));
        const float x = float(ImPlot::PlotToPixels(ov.centres[i], lim.Y.Min).x);
        dl->AddLine(ImVec2(x, y0), ImVec2(x, y1), ovld ? c_ovld : c_centre,
                    ovld ? 1.6f : 1.0f);
    }
}

// §7's image check, drawn over the trace. Deliberately *above* the segment
// overlay and below the markers: it is a verdict about the data, not part of
// the machinery that produced it.
void App::draw_image_overlay() {
    if (!img_show_ || img_phase_ != ImgPhase::Done || img_res_.findings.empty()) return;
    if (cfg_.zero_span()) return;

    const ImPlotRect lim = ImPlot::GetPlotLimits();
    ImDrawList *dl = ImPlot::GetPlotDrawList();
    const float y0 = float(ImPlot::PlotToPixels(lim.X.Min, lim.Y.Min).y);
    const float y1 = float(ImPlot::PlotToPixels(lim.X.Min, lim.Y.Max).y);

    // Three verdicts, three colours, and none of them is the colour of a
    // signal. The verdict is not "this is noise" -- it is "this response is the
    // receiver, not the air" -- so it is marked rather than hidden.
    const ImU32 c_img = IM_COL32(255, 120, 200, 210);
    const ImU32 c_spur = IM_COL32(255, 180, 90, 190);
    const ImU32 c_moved = IM_COL32(190, 160, 220, 160);

    for (const ImageFinding &f : img_res_.findings) {
        if (f.x_hz < lim.X.Min || f.x_hz > lim.X.Max) continue;
        const ImU32 col = f.verdict == ImageVerdict::Image
                              ? c_img
                              : (f.verdict == ImageVerdict::LoSpur ? c_spur : c_moved);
        const float x = float(ImPlot::PlotToPixels(f.x_hz, lim.Y.Min).x);
        dl->AddLine(ImVec2(x, y0), ImVec2(x, y1), col, 1.4f);
        // A cross at the top, so a flagged response is identifiable at a glance
        // without reading the table. Small: it annotates, it does not cover.
        const float m = 5.0f;
        dl->AddLine(ImVec2(x - m, y1 + m), ImVec2(x + m, y1 + 3 * m), col, 2.0f);
        dl->AddLine(ImVec2(x + m, y1 + m), ImVec2(x - m, y1 + 3 * m), col, 2.0f);
    }
}

// The Image Check soft keys. Its own group and not a checkbox in System,
// because what it reports is a *verdict about the trace* -- the same weight as
// the UNCAL badge, and the thing §7 built it for.
void App::draw_image_check_group() {
    const bool running = img_phase_ == ImgPhase::PassA || img_phase_ == ImgPhase::Shifting ||
                         img_phase_ == ImgPhase::PassB ||
                         img_phase_ == ImgPhase::Restoring;

    int want = int(img_want_sweeps_);
    ImGui::PushItemWidth(-110);
    if (ImGui::InputInt("sweeps/pass", &want, 10, 50) && !running) {
        if (want < 4) want = 4;
        if (want > 5000) want = 5000;
        img_want_sweeps_ = size_t(want);
    }
    ImGui::PopItemWidth();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("both passes average this many sweeps.\nThe two are taken at "
                          "different times, so a burst that stops between them looks "
                          "exactly like an image -- averaging is the only defence.");

    // Both thresholds are judgement calls about a particular installation's
    // noise floor and signal population, so they are knobs rather than
    // constants -- the project's own rule about numbers that depend on where
    // the instrument is standing.
    float exc = float(img_cfg_.min_excursion_db);
    float mov = float(img_cfg_.move_db);
    ImGui::PushItemWidth(-110);
    if (ImGui::InputFloat("min excursion", &exc, 1.0f, 5.0f, "%.1f dB") && !running)
        img_cfg_.min_excursion_db = std::max(0.5, double(exc));
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("how far a bin must stand above its own local floor before "
                          "it counts as a response worth classifying");
    if (ImGui::InputFloat("must drop", &mov, 1.0f, 5.0f, "%.1f dB") && !running)
        img_cfg_.move_db = std::max(0.5, double(mov));
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("how much of that excursion must vanish on the phased grid "
                          "before the response is called an artifact");
    ImGui::PopItemWidth();

    if (running) {
        if (ImGui::Button("cancel", ImVec2(-1, 0)))
            abort_image_check("cancelled");
        const size_t done = img_acc_.sweeps();
        ImGui::ProgressBar(float(done) / float(std::max<size_t>(1, img_want_sweeps_)),
                           ImVec2(-1, 0));
        ImGui::TextWrapped("%s (%zu/%zu sweeps)", img_status_.c_str(), done,
                           img_want_sweeps_);
        if (img_phase_ == ImgPhase::Shifting || img_phase_ == ImgPhase::Restoring)
            ImGui::TextWrapped("the LO positions all changed, so the daemon is "
                               "cold-rebuilding its profile table");
    } else {
        if (ImGui::Button("run image check", ImVec2(-1, 0))) {
            std::string err;
            if (!start_image_check(&err)) note("image check: " + err, true);
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("sweeps twice, the second time with the LO grid phased by "
                              "half a step.\nReal signals stay where they are; mirror "
                              "images move by a whole step.");
    }

    if (img_phase_ == ImgPhase::Failed) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.6f, 0.5f, 1.0f));
        ImGui::TextWrapped("failed: %s", img_status_.c_str());
        ImGui::PopStyleColor();
    }

    if (img_phase_ != ImgPhase::Done) return;

    ImGui::Separator();
    ImGui::Checkbox("mark findings on the trace", &img_show_);
    ImGui::Text("%zu flagged of %zu bins compared", img_res_.findings.size(),
                img_res_.n_compared);
    ImGui::Text("shift %s, images move %s", format_frequency(img_res_.shift_hz, 3).c_str(),
                format_frequency(2.0 * img_res_.shift_hz, 3).c_str());
    if (img_res_.n_blanked)
        ImGui::TextWrapped("%zu bins skipped: they sit on a segment centre of one grid "
                           "or the other, where the splice fill reads hot either way",
                           img_res_.n_blanked);
    if (img_res_.dropped)
        ImGui::TextWrapped("%zu more were dropped -- that many findings means the "
                           "thresholds are wrong, not that the receiver has that many "
                           "images", img_res_.dropped);

    if (img_res_.findings.empty()) {
        ImGui::TextWrapped("Nothing moved. Every response on screen was at the same "
                           "frequency on both LO grids, which is what a receiver with "
                           "no visible images looks like.");
    } else if (ImGui::BeginTable("imgchk", 4,
                                 ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("freq");
        ImGui::TableSetupColumn("over");
        ImGui::TableSetupColumn("drop");
        ImGui::TableSetupColumn("verdict");
        ImGui::TableHeadersRow();
        size_t shown = 0;
        for (const ImageFinding &f : img_res_.findings) {
            if (shown++ >= 12) break;
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%s", format_frequency(f.x_hz, 5).c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%.1f", double(f.excursion_a_db));
            ImGui::TableNextColumn();
            ImGui::Text("%.1f", double(f.drop_db));
            ImGui::TableNextColumn();
            ImGui::Text("%s", image_verdict_name(f.verdict));
        }
        ImGui::EndTable();
        if (img_res_.findings.size() > 12)
            ImGui::Text("... and %zu more", img_res_.findings.size() - 12);
        ImGui::TextWrapped("image: mirrored about the segment LO. LO spur: a fixed "
                           "offset from the LO, so it moved by the shift rather than "
                           "twice it. moved: it went away and was not found again.");
    }

    // Never hidden behind a hover. The check's own limitation is the thing most
    // likely to produce a wrong verdict, and §7 exists because a measurement
    // that looks certain is worse than one that says what it cannot do.
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.85f, 0.8f, 0.6f, 1.0f));
    ImGui::TextWrapped("%s", img_res_.caveat.c_str());
    ImGui::PopStyleColor();
}

// ------------------------------------------------ receiver-artifact marking
//
// The sequencing and the drawing. What counts as an artifact is artifacts.h,
// which is host-tested; nothing here decides it.

void App::art_reset() {
    art_acc_ = PassAccumulator();
    art_res_ = ArtifactResult();
    if (art_base_capturing_) {
        art_base_capturing_ = false;
        art_base_acc_ = PassAccumulator();
        note("artifacts: baseline capture abandoned -- the sweep plan changed under it",
             true);
    }
}

void App::art_feed(const TraceAxis &fax, const float *db, size_t n) {
    // The image check's second pass is on a deliberately phased grid; every
    // offset in it is relative to LOs the plan does not normally use.
    if (!fax.valid() || std::fabs(cfg_.lo_shift_hz) > 1.0) return;

    if (art_base_capturing_) {
        if (!art_base_acc_.add(fax, db, n)) {
            art_base_capturing_ = false;
            note("artifacts: baseline capture abandoned -- the sweep changed shape", true);
        } else if (art_base_acc_.sweeps() >= art_base_sweeps_) {
            art_base_capturing_ = false;
            art_base_.pass = art_base_acc_.finish(segs_);
            art_base_.gain_db = double(cfg_.gain_db);
            art_base_.seg_first_lo_hz = segs_.empty() ? 0.0 : segs_.front().lo_hz;
            art_base_.seg_step_hz = cfg_.seg_step_hz;
            art_acc_ = PassAccumulator();   // the next result uses it from scratch
            note("artifacts: baseline recorded from " +
                     std::to_string(art_base_.pass.sweeps) +
                     " sweeps -- reconnect the antenna",
                 false);
        }
        // A terminated input is not the scene the user is looking at; marking
        // it would mark the baseline against itself.
        return;
    }

    if (!art_on_) return;
    if (!art_acc_.matches(fax)) {
        art_acc_.begin(fax, 0.0);
        if (art_res_.valid && (art_res_.axis.n != fax.n || art_res_.axis.x0 != fax.x0 ||
                               art_res_.axis.dx != fax.dx))
            art_res_ = ArtifactResult();
    }
    if (!art_acc_.add(fax, db, n)) return;
    if (art_acc_.sweeps() < art_sweeps_) return;

    const ImagePass live = art_acc_.finish(segs_);
    art_res_ = mark_artifacts(live, art_base_.valid() ? &art_base_ : nullptr,
                              double(cfg_.gain_db), art_cfg_);
    art_acc_.begin(fax, 0.0);
}

bool App::art_start_baseline(std::string *err) {
    if (cfg_.zero_span()) {
        *err = "a baseline is a sweep; leave zero span first";
        return false;
    }
    if (img_phase_ == ImgPhase::PassA || img_phase_ == ImgPhase::Shifting ||
        img_phase_ == ImgPhase::PassB || img_phase_ == ImgPhase::Restoring) {
        *err = "an image check is running";
        return false;
    }
    art_base_acc_ = PassAccumulator();
    const TraceAxis ax = axis();
    if (!ax.valid()) {
        *err = "no sweep yet";
        return false;
    }
    art_base_acc_.begin(ax, 0.0);
    art_base_capturing_ = true;
    return true;
}

uint8_t App::art_flags_at(double x) const {
    if (!art_on_ || !art_res_.valid || cfg_.zero_span()) return 0;
    const TraceAxis ax = axis();
    const TraceAxis &ra = art_res_.axis;
    if (!ax.valid() || ax.n != ra.n || ax.x0 != ra.x0 || ax.dx != ra.dx) return 0;
    size_t i;
    if (!ra.bin_of(x, &i) || i >= art_res_.flags.size()) return 0;
    return art_res_.flags[i];
}

namespace {
ImU32 art_colour(uint8_t f) {
    // One colour per tag, in artifact_tag's priority order. The LO/DC blue is
    // the segment overlay's centre blue, and the LO-spur orange is the image
    // check's: the same thing is the same colour wherever it is drawn.
    if (f & kArtInternal) return IM_COL32(200, 140, 255, 220);
    if (f & kArtLoSpur) return IM_COL32(255, 180, 90, 230);
    if (f & kArtDcHole) return IM_COL32(150, 190, 255, 200);
    return IM_COL32(110, 200, 210, 150);   // comb
}
ImVec4 art_colour_v(uint8_t f) { return ImGui::ColorConvertU32ToFloat4(art_colour(f)); }
}  // namespace

// A strip along the top of the graticule, coloured by tag, under the markers.
// A strip and not a shaded column: a column the height of the plot over every
// DC hole at full span would be the picket fence the segment overlay refuses
// to draw. The strip says where; the marker tag says what.
void App::draw_artifact_overlay() {
    if (!art_on_ || !art_res_.valid || cfg_.zero_span()) return;
    const TraceAxis ax = axis();
    const TraceAxis &ra = art_res_.axis;
    if (!ax.valid() || ax.n != ra.n || ax.x0 != ra.x0 || ax.dx != ra.dx) return;

    const ImPlotRect lim = ImPlot::GetPlotLimits();
    ImDrawList *dl = ImPlot::GetPlotDrawList();
    const float ytop = float(ImPlot::PlotToPixels(lim.X.Min, lim.Y.Max).y);
    const float ybot = float(ImPlot::PlotToPixels(lim.X.Min, lim.Y.Min).y);

    long long i0 = (long long)std::floor((lim.X.Min - ra.x0) / ra.dx);
    long long i1 = (long long)std::ceil((lim.X.Max - ra.x0) / ra.dx);
    i0 = std::max(0LL, i0);
    i1 = std::min((long long)ra.n, i1);

    auto tag_class = [](uint8_t f) -> uint8_t {
        if (f & kArtInternal) return kArtInternal;
        if (f & kArtLoSpur) return kArtLoSpur;
        if (f & kArtDcHole) return kArtDcHole;
        if (f & kArtLoComb) return kArtLoComb;
        return 0;
    };
    for (long long i = i0; i < i1;) {
        const uint8_t c = tag_class(art_res_.flags[size_t(i)]);
        if (!c) { ++i; continue; }
        long long j = i;
        while (j < i1 && tag_class(art_res_.flags[size_t(j)]) == c) ++j;
        float x0 = float(ImPlot::PlotToPixels(ra.x0 + double(i) * ra.dx, lim.Y.Max).x);
        float x1 = float(ImPlot::PlotToPixels(ra.x0 + double(j) * ra.dx, lim.Y.Max).x);
        if (x1 - x0 < 2.0f) {
            const float m = 0.5f * (x0 + x1);
            x0 = m - 1.0f;
            x1 = m + 1.0f;
        }
        // The two response-level verdicts get a taller strip than the two
        // positional ones: they are claims about a peak, not about a place.
        const float h = (c == kArtLoSpur || c == kArtInternal) ? 10.0f : 5.0f;
        dl->AddRectFilled(ImVec2(x0, ytop), ImVec2(x1, ytop + h), art_colour(c));
        i = j;
    }

    // And a faint drop line on each finding, so a flagged peak can be matched
    // to its strip at full span, where the strip is a pixel wide.
    for (const ArtifactFinding &f : art_res_.findings) {
        if (f.x_hz < lim.X.Min || f.x_hz > lim.X.Max) continue;
        const float x = float(ImPlot::PlotToPixels(f.x_hz, lim.Y.Min).x);
        ImU32 col = art_colour(f.flags);
        col = (col & 0x00FFFFFFu) | (IM_COL32_A_MASK & IM_COL32(0, 0, 0, 70));
        dl->AddLine(ImVec2(x, ytop + 10.0f), ImVec2(x, ybot), col, 1.0f);
    }
}

void App::draw_artifact_group() {
    ImGui::Checkbox("mark receiver artifacts", &art_on_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("marks responses that are likely the receiver itself -- LO "
                          "leakage,\nthe DC splice, LO-relative spurs, and (with a "
                          "baseline) internal\nspurs. The trace is never altered.");
    if (!art_on_) return;

    int sw = int(art_sweeps_);
    float comb = float(art_cfg_.comb_db), spur = float(art_cfg_.spur_db);
    int recur = int(art_cfg_.min_recur);
    ImGui::PushItemWidth(-110);
    if (ImGui::InputInt("sweeps/update", &sw, 4, 16)) {
        art_sweeps_ = size_t(std::max(1, std::min(sw, 2000)));
        art_acc_ = PassAccumulator();
    }
    if (ImGui::InputFloat("comb over", &comb, 0.5f, 1.0f, "%.1f dB"))
        art_cfg_.comb_db = std::max(0.5, double(comb));
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("how far most segments must stand above their own median at "
                          "one LO offset\nbefore that offset is called the receiver's "
                          "own shape");
    if (ImGui::InputFloat("spur over", &spur, 1.0f, 5.0f, "%.1f dB"))
        art_cfg_.spur_db = std::max(1.0, double(spur));
    if (ImGui::InputInt("spur places", &recur, 1, 1))
        art_cfg_.min_recur = size_t(std::max(2, std::min(recur, 50)));
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("separate places across the span (more than %.0f MHz apart) "
                          "a peak must recur\nat one LO offset before it is called an "
                          "LO spur",
                          art_cfg_.cluster_gap_hz / 1e6);
    ImGui::PopItemWidth();

    // Legend, in the strip's own colours.
    const uint8_t order[4] = {kArtDcHole, kArtLoComb, kArtLoSpur, kArtInternal};
    for (int k = 0; k < 4; ++k) {
        if (k) ImGui::SameLine();
        ImGui::TextColored(art_colour_v(order[k]), "%s", artifact_tag(order[k]));
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", artifact_explain(order[k]));
    }

    if (cfg_.zero_span()) {
        ImGui::TextDisabled("zero span: one parked LO, nothing to fold");
    } else if (!art_res_.valid) {
        ImGui::ProgressBar(float(art_acc_.sweeps()) / float(std::max<size_t>(1, art_sweeps_)),
                           ImVec2(-1, 0), art_res_.err.empty() ? "averaging" : art_res_.err.c_str());
    } else {
        ImGui::Text("marked bins: %zu LO/DC, %zu comb, %zu spur, %zu internal",
                    art_res_.n_dc, art_res_.n_comb, art_res_.n_spur, art_res_.n_internal);
        ImGui::Text("%zu peaks over %.0f dB not explained", art_res_.unmarked.size(),
                    art_cfg_.spur_db);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("peaks standing out that no test here attributes to the "
                              "receiver.\nNot proof they are signals -- only that nothing "
                              "here says otherwise.");
        if (!art_res_.fold_ran)
            ImGui::TextColored(ImVec4(0.85f, 0.8f, 0.6f, 1.0f), "LO folds not run: %s",
                               art_res_.fold_note.c_str());
        for (size_t k = 0; k < art_res_.comb.size() && k < 6; ++k) {
            const CombOffset &c = art_res_.comb[k];
            ImGui::TextColored(art_colour_v(kArtLoComb), "comb at LO %s%s  +%.1f dB  (%zu seg)",
                               c.offset_hz >= 0 ? "+" : "-",
                               format_frequency(std::fabs(c.offset_hz), 3).c_str(),
                               double(c.median_db), c.segments);
        }
        if (!art_res_.findings.empty() &&
            ImGui::BeginTable("##art", 4, ImGuiTableFlags_RowBg |
                                              ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("freq");
            ImGui::TableSetupColumn("over");
            ImGui::TableSetupColumn("LO off");
            ImGui::TableSetupColumn("tag");
            ImGui::TableHeadersRow();
            size_t shown = 0;
            for (const ArtifactFinding &f : art_res_.findings) {
                if (shown++ >= 12) break;
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("%s", format_frequency(f.x_hz, 5).c_str());
                ImGui::TableNextColumn();
                ImGui::Text("%.1f", double(f.excursion_db));
                ImGui::TableNextColumn();
                if (f.lo_offset_hz != 0)
                    ImGui::Text("%s%s", f.lo_offset_hz >= 0 ? "+" : "-",
                                format_frequency(std::fabs(f.lo_offset_hz), 3).c_str());
                else
                    ImGui::TextDisabled("--");
                ImGui::TableNextColumn();
                ImGui::TextColored(art_colour_v(f.flags), "%s", artifact_tag(f.flags));
                if (ImGui::IsItemHovered()) {
                    if (f.flags & kArtLoSpur)
                        ImGui::SetTooltip("%s\nseen in %zu segments, %zu separate places",
                                          artifact_explain(f.flags), f.recur_segments,
                                          f.recur_clusters);
                    else
                        ImGui::SetTooltip("%s", artifact_explain(f.flags));
                }
            }
            ImGui::EndTable();
            if (art_res_.findings.size() > 12)
                ImGui::Text("... and %zu more", art_res_.findings.size() - 12);
        }
    }

    ImGui::Separator();
    ImGui::TextUnformatted("terminated-input baseline");
    if (art_base_capturing_) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.75f, 0.35f, 1.0f));
        ImGui::TextWrapped("recording -- the RX input must be terminated (50 ohm load, "
                           "antenna off) until this finishes");
        ImGui::PopStyleColor();
        ImGui::ProgressBar(float(art_base_acc_.sweeps()) /
                               float(std::max<size_t>(1, art_base_sweeps_)),
                           ImVec2(-1, 0));
        if (ImGui::Button("cancel##base", ImVec2(-1, 0))) {
            art_base_capturing_ = false;
            art_base_acc_ = PassAccumulator();
        }
    } else {
        int bs = int(art_base_sweeps_);
        ImGui::PushItemWidth(-110);
        if (ImGui::InputInt("baseline sweeps", &bs, 10, 50))
            art_base_sweeps_ = size_t(std::max(4, std::min(bs, 5000)));
        ImGui::PopItemWidth();
        if (ImGui::Button("record baseline", ImVec2(-1, 0))) {
            std::string err;
            if (!art_start_baseline(&err)) note("artifacts: " + err, true);
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("terminate the RX input first. Whatever still stands out "
                              "is generated\ninside the box -- clock harmonics, reference "
                              "spurs -- and is marked\n'internal?' wherever it later "
                              "appears at about that level.");
    }
    ImGui::PushItemWidth(-110);
    ImGui::InputText("baseline file", buf_base_, sizeof buf_base_);
    ImGui::PopItemWidth();
    if (ImGui::Button("save##base")) {
        std::string err;
        if (save_baseline(buf_base_, art_base_, &err)) note(std::string("baseline saved to ") + buf_base_, false);
        else note("artifacts: " + err, true);
    }
    ImGui::SameLine();
    if (ImGui::Button("load##base")) {
        std::string err;
        ArtifactBaseline b;
        if (load_baseline(buf_base_, &b, &err)) {
            art_base_ = std::move(b);
            art_acc_ = PassAccumulator();
            note(std::string("baseline loaded from ") + buf_base_, false);
        } else {
            note("artifacts: " + err, true);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("clear##base")) {
        art_base_ = ArtifactBaseline();
        art_acc_ = PassAccumulator();
    }
    if (art_base_.valid())
        ImGui::Text("baseline: %zu sweeps at %.1f dB gain", art_base_.pass.sweeps,
                    art_base_.gain_db);
    else
        ImGui::TextDisabled("no baseline");
    if (art_res_.valid && !art_res_.baseline_note.empty())
        ImGui::TextColored(ImVec4(0.85f, 0.8f, 0.6f, 1.0f), "%s",
                           art_res_.baseline_note.c_str());

    if (art_res_.valid) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.85f, 0.8f, 0.6f, 1.0f));
        ImGui::TextWrapped("%s", art_res_.caveat.c_str());
        ImGui::PopStyleColor();
    }
}

// ------------------------------------------------------- §7 amplitude correction

// The span the *data* covers, which is what a correction has to span before the
// axis may call itself dBm. In zero span there is one frequency -- the parked
// centre -- and `f_start_hz == f_stop_hz` already carries it (proto.h).
void App::measured_span(double *lo, double *hi) const {
    if (have_frame_ && !cur_.db.empty()) {
        *lo = double(cur_.f_start_hz);
        *hi = double(cur_.f_stop_hz);
    } else {
        *lo = double(cfg_.f_start_hz);
        *hi = double(cfg_.f_stop_hz);
    }
    if (*hi < *lo) std::swap(*lo, *hi);
}

// §7's badge has to fail lit, so this answers "is it *still* uncalibrated".
//
// `cfg_.uncal` is the source's own starting state, and it is not a veto. It
// cannot be: `sweepd` hardcodes `uncal: true` and always will, because the
// daemon never applies a correction -- the file lives on the host, where a
// RAM-backed rootfs cannot lose it (ampcorr.h, "where it runs"). A rule that
// let the source's `true` win would therefore make this feature permanently
// dead against real hardware, not just against the synthetic generator. What
// the field is for is the other direction: a source that reports `false` has
// already corrected its own data, and then the UI must not demand a file.
//
// So the badge goes out when *someone* corrected the amplitude, and the host's
// correction counts -- provided it covers the whole measured span. That last
// condition is the one worth stating: a correction covering half the span
// produces a trace whose left half is dBm and whose right half is dBFS, and no
// single axis label is true of it. Rather than label it anyway, or silently
// clamp the file outwards over frequencies it never measured, the units stay
// dBFS and the Amplitude group says how much of the span is missing.
//
// The SYNTHETIC badge is a separate claim and is unaffected: fabricated data
// stays labelled fabricated whatever the units say.
//
// A **relative** correction never puts the badge out, however well it covers
// the span (ampcorr.h, "not every correction is a dBm one"). It is a shape with
// an unknown constant in it -- flattening the trace is real and useful, but the
// vertical axis is still an arbitrary reference, which is precisely what UNCAL
// says. Reading it as dBm would be worse than loading nothing at all, because
// nothing about the picture would look wrong.
const char *App::units() const {
    if (tnorm_applied_) return "dB";
    return uncal() ? "dBFS" : "dBm";
}

TrackGrid App::track_grid(const sweepd::proto::TraceMsg &f) const {
    TrackGrid g;
    g.f_start_hz = double(f.f_start_hz);
    g.bin_hz = f.bin_hz;
    g.n = f.db.size();
    g.tones = cfg_.tsa_tones;
    g.atten_db = cfg_.tsa_atten_db;
    g.gain_db = double(f.gain_db);
    g.harmonic = cfg_.tsa_harmonic;
    g.rx_offset_hz = cfg_.tsa_rx_offset_hz;
    g.rx_invert = cfg_.tsa_rx_invert;
    return g;
}

void App::floor_step(const sweepd::proto::TraceMsg &f, const TrackGrid &g) {
    std::string err;
    switch (floor_phase_) {
        case FloorPhase::Idle: return;
        case FloorPhase::Silencing:
            // The daemon says it is silent, and one more sweep has gone by so
            // nothing in flight from before still carries the stimulus.
            if (!cfg_.tsa_stim_off) return;
            if (floor_skip_-- > 0) return;
            floor_.start_capture(size_t(floor_sweeps_));
            floor_phase_ = FloorPhase::Capturing;
            return;
        case FloorPhase::Capturing:
            if (!floor_.feed(g, f.db.data(), f.db.size())) return;
            {
                TrackRequest r;
                r.stim_off = 0;
                if (!src_->set_tracking(r, &err)) note("restoring the stimulus: " + err, true);
            }
            floor_phase_ = FloorPhase::Restoring;
            note("isolation floor stored", false);
            return;
        case FloorPhase::Restoring:
            if (!cfg_.tsa_stim_off) floor_phase_ = FloorPhase::Idle;
            return;
    }
}

void App::draw_floor_overlay() {
    if (!cfg_.tracking || !floor_show_ || !floor_.have() || !have_frame_) return;
    const std::vector<float> &fl = floor_.ref_db();
    if (fl.size() != cur_.db.size() || !(cur_.bin_hz > 0)) return;
    // The floor goes through what the trace went through: the same reference
    // subtracted, the same compensation, the same sign for return loss.
    const TrackGrid g = track_grid(cur_);
    std::vector<float> v(fl);
    if (tnorm_applied_) {
        double comp = 0;
        if (!tnorm_.apply(g, v.data(), v.size(), &comp).empty()) return;
        if (tnorm_reflect_ && tnorm_rl_)
            for (float &x : v) x = -x;
    }
    floor_x_.resize(v.size());
    floor_y_.resize(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        floor_x_[i] = double(cur_.f_start_hz) + (double(i) + 0.5) * cur_.bin_hz;
        floor_y_[i] = double(v[i]);
    }
    ImPlotSpec spec;
    spec.LineColor = ImVec4(0.75f, 0.75f, 0.75f, 0.8f);
    spec.LineWeight = 1.0f;
    ImPlot::PlotLine("##floor", floor_x_.data(), floor_y_.data(), int(v.size()), spec);
}

bool App::uncal() const {
    if (!cfg_.uncal) return false;    // the source corrected it already
    if (!corr_.loaded()) return true;
    if (!corr_.absolute()) return true;
    double lo = 0, hi = 0;
    measured_span(&lo, &hi);
    return !corr_.covers(lo, hi);
}

// The conditions the file says it was measured under, against the conditions
// the live frame reports. Empty when they agree or when the file did not say.
//
// Not a refusal: the operator may have a reason, and ampcorr.h gives the
// argument. But an amplitude correction applied at a different RX gain, over a
// different span -- and therefore a different hop order, which
// docs/characterization.md measures at up to 6.3 dB -- or under a different
// detector is wrong by an amount nothing else on screen would reveal.
std::string App::corr_condition_mismatch() const {
    if (!corr_.loaded() || !have_frame_) return {};
    const AmpCorrection::Conditions &c = corr_.conditions();
    std::string out;
    char buf[200];

    if (c.has_gain_db && std::fabs(c.gain_db - double(cur_.gain_db)) > 0.5) {
        std::snprintf(buf, sizeof buf, "measured at %.1f dB RX gain, sweeping at %.1f dB",
                      c.gain_db, double(cur_.gain_db));
        out += buf;
    }
    if (c.has_span) {
        double lo = 0, hi = 0;
        measured_span(&lo, &hi);
        // 1 Hz, the same tolerance covers() uses: a difference this small is
        // bin-centre arithmetic, not a different span.
        if (std::fabs(c.span_lo_hz - lo) > 1.0 || std::fabs(c.span_hi_hz - hi) > 1.0) {
            std::snprintf(buf, sizeof buf,
                          "measured over %.6g - %.6g MHz, sweeping %.6g - %.6g MHz "
                          "(different hop order)",
                          c.span_lo_hz / 1e6, c.span_hi_hz / 1e6, lo / 1e6, hi / 1e6);
            if (!out.empty()) out += "; ";
            out += buf;
        }
    }
    // Only when both sides know: a source that reports no clock (0) is not a
    // mismatch, it is an absence, and the file's other conditions still speak.
    if (c.has_adc_hz && cfg_.adc_hz > 0 && std::fabs(c.adc_hz - cfg_.adc_hz) > 1.0) {
        std::snprintf(buf, sizeof buf,
                      "measured with the ADC at %.2f MHz, sweeping at %.2f MHz "
                      "(different rate or rate governor, ~0.5 dB)",
                      c.adc_hz / 1e6, cfg_.adc_hz / 1e6);
        if (!out.empty()) out += "; ";
        out += buf;
    }
    if (!c.detector.empty() && c.detector != sweepd::detector_name(cur_.detector)) {
        std::snprintf(buf, sizeof buf, "measured with the %s detector, sweeping with %s",
                      c.detector.c_str(), sweepd::detector_name(cur_.detector));
        if (!out.empty()) out += "; ";
        out += buf;
    }
    return out;
}

bool App::load_gain_table(const std::string &path, std::string *err) {
    std::string text;
    if (std::FILE *fp = std::fopen(path.c_str(), "rb")) {
        char buf[4096];
        size_t n;
        while ((n = std::fread(buf, 1, sizeof buf, fp)) > 0 && text.size() < (1u << 20))
            text.append(buf, n);
        std::fclose(fp);
    }
    if (text.empty()) {
        *err = "cannot read " + path;
        return false;
    }
    std::string wire;
    sweepd::GainTableInfo gi;
    if (!sweepd::prepare_gain_table(text, &wire, &gi, err)) {
        *err = path + ": " + *err;
        return false;
    }
    const size_t slash = path.find_last_of("/\\");
    return src_->set_gain_table(wire, slash == std::string::npos ? path : path.substr(slash + 1),
                                err);
}

bool App::load_builtin_gain_table(std::string *err) {
    std::string wire;
    sweepd::GainTableInfo gi;
    if (!sweepd::prepare_gain_table(sweepd::builtin_gain_table(), &wire, &gi, err)) return false;
    return src_->set_gain_table(wire, sweepd::builtin_gain_table_name(), err);
}

bool App::load_ampcorr(const std::string &path, std::string *err) {
    if (!corr_.load_csv(path, err)) return false;

    // Same rule as a retune: corrected and uncorrected bins are not comparable,
    // so anything accumulated across the change is discarded rather than
    // quietly mixed (§10.5, "a hold clears when the data stops being
    // comparable").
    traces_.clear_all();
    wf_.clear();

    double lo = 0, hi = 0;
    measured_span(&lo, &hi);
    char buf[320];
    if (!corr_.absolute()) {
        // Loaded and applied -- the response really is flattened -- but the
        // vertical axis gained no absolute reference, so it keeps saying so.
        std::snprintf(buf, sizeof buf,
                      "relative amplitude correction loaded (%zu points, %.6g - %.6g MHz): "
                      "the frequency response is flattened, but the file carries no "
                      "reference level, so units stay dBFS and UNCAL stays lit",
                      corr_.size(), corr_.f_min_hz() / 1e6, corr_.f_max_hz() / 1e6);
        note(buf, true);
    } else if (corr_.covers(lo, hi)) {
        std::snprintf(buf, sizeof buf,
                      "amplitude correction loaded: %zu points, %.6g - %.6g MHz. "
                      "units are now dBm",
                      corr_.size(), corr_.f_min_hz() / 1e6, corr_.f_max_hz() / 1e6);
        note(buf, false);
    } else {
        // Loaded and applied where it reaches, but not enough to relabel the
        // axis. Saying which is the whole point.
        std::snprintf(buf, sizeof buf,
                      "amplitude correction loaded (%zu points, %.6g - %.6g MHz) but it "
                      "does not cover the swept span %.6g - %.6g MHz; units stay dBFS "
                      "and UNCAL stays lit",
                      corr_.size(), corr_.f_min_hz() / 1e6, corr_.f_max_hz() / 1e6,
                      lo / 1e6, hi / 1e6);
        note(buf, true);
    }
    return true;
}

void App::unload_ampcorr() {
    if (!corr_.loaded()) return;
    corr_.clear();
    corr_uncovered_ = 0;
    traces_.clear_all();
    wf_.clear();
    note("amplitude correction unloaded; units are dBFS again", false);
}

// ---------------------------------------------------------------- helpers

// The axis of the frame currently on screen, taken from the frame rather than
// from the config, so a marker never reads against an axis the data does not
// have -- the two are briefly out of step across a retune.
// ---------------------------------------------------------- persistence

void App::persist_feed(const sweepd::proto::TraceMsg &f) {
    if (!persist_on_ || f.db.empty()) return;
    double x0 = 0, dx = 0;
    if (f.x_kind == sweepd::proto::XKind::Time) {
        if (f.dt_ns == 0) return;
        x0 = double(f.t0_ns) * 1e-9;
        dx = double(f.dt_ns) * 1e-9;
    } else {
        if (!(f.bin_hz > 0)) return;
        x0 = double(f.f_start_hz);
        dx = f.bin_hz;
    }
    const double x1 = x0 + dx * double(f.db.size());
    const double ylo = double(ref_db_ - db_per_div_ * kDivisions), yhi = double(ref_db_);
    // 1024 x 256 cells: finer than most plot widths, and a 1 MB histogram.
    if (!persist_.matches(1024, 256, x0, x1, ylo, yhi)) {
        persist_.layout(1024, 256, x0, x1, ylo, yhi);
        persist_t_last_ = 0;
    }
    const double now = mono_now();
    persist_.set_tau(double(persist_tau_));
    persist_.add(f.db.data(), f.db.size(), x0, dx,
                 persist_t_last_ > 0 ? now - persist_t_last_ : 0.0);
    persist_t_last_ = now;
    persist_dirty_ = true;
}

bool App::persist_draw() {
    if (!persist_on_ || persist_.sweeps() == 0) return false;
    // The histogram's rows are evenly spaced in dB, and PlotImage stretches an
    // image linearly between its corners: under the lin transform the rows would
    // be drawn at the wrong levels. Hidden rather than drawn wrong.
    if (lin_active()) return false;
    // The level range moved under it: its rows no longer mean the levels drawn
    // beside them. Cleared on the next sweep by persist_feed; hidden until then.
    const double ylo = double(ref_db_ - db_per_div_ * kDivisions), yhi = double(ref_db_);
    if (persist_.y_lo() != ylo || persist_.y_hi() != yhi) return false;

    const size_t w = persist_.cols(), h = persist_.rows();
    if (!persist_tex_ || persist_tex_w_ != w || persist_tex_h_ != h) {
        if (persist_tex_) gl::DeleteTextures(1, &persist_tex_);
        gl::GenTextures(1, &persist_tex_);
        gl::BindTexture(gl::kTexture2D, persist_tex_);
        gl::TexParameteri(gl::kTexture2D, gl::kTextureMinFilter, gl::kLinear);
        gl::TexParameteri(gl::kTexture2D, gl::kTextureMagFilter, gl::kLinear);
        gl::TexParameteri(gl::kTexture2D, gl::kTextureWrapS, gl::kClampToEdge);
        gl::TexParameteri(gl::kTexture2D, gl::kTextureWrapT, gl::kClampToEdge);
        gl::TexImage2D(gl::kTexture2D, 0, gl::Int(gl::kRgba8), gl::Sizei(w), gl::Sizei(h), 0,
                       gl::kRgba, gl::kUnsignedByte, nullptr);
        persist_tex_w_ = w;
        persist_tex_h_ = h;
        persist_dirty_ = true;
    }
    if (persist_dirty_) {
        persist_.intensity(&persist_i_);
        persist_rgba_.assign(w * h * 4, 0);
        for (size_t k = 0; k < w * h; ++k) {
            const float t = persist_i_[k];
            if (t <= 0) continue;    // transparent: the plot shows through
            float rgb[3];
            colormap_rgb(wf_.colormap(), 0.15f + 0.85f * t, rgb);
            for (int c = 0; c < 3; ++c)
                persist_rgba_[k * 4 + size_t(c)] = (unsigned char)std::lround(rgb[c] * 255.0f);
            persist_rgba_[k * 4 + 3] = 255;
        }
        gl::BindTexture(gl::kTexture2D, persist_tex_);
        gl::PixelStorei(gl::kUnpackAlignment, 1);
        gl::TexSubImage2D(gl::kTexture2D, 0, 0, 0, gl::Sizei(w), gl::Sizei(h), gl::kRgba,
                          gl::kUnsignedByte, persist_rgba_.data());
        persist_dirty_ = false;
    }
    // Row 0 of the texture is the top level, so v runs 0 at y_hi.
    ImPlot::PlotImage("##persist", (ImTextureID)(intptr_t)persist_tex_,
                      ImPlotPoint(persist_.x0(), persist_.y_lo()),
                      ImPlotPoint(persist_.x1(), persist_.y_hi()), ImVec2(0, 0), ImVec2(1, 1));
    return true;
}

// ------------------------------------------------------ band measurements

bool App::meas_from_markers(std::string *err) {
    if (!markers_[0].on || !markers_[1].on) {
        *err = "markers 1 and 2 must both be on; they become the band edges";
        return false;
    }
    const double a = markers_[0].x, b = markers_[1].x;
    meas_f0_ = (a + b) * 0.5;
    meas_bw_ = std::fabs(b - a);
    return true;
}

void App::meas_update() {
    meas_cp_ = BandResult();
    meas_obw_ = ObwResult();
    if (meas_mode_ == MeasMode::Off || !have_frame_ || cfg_.zero_span()) return;
    if (!(meas_bw_ > 0)) {
        meas_f0_ = (view_x0_ + view_x1_) * 0.5;
        meas_bw_ = (view_x1_ - view_x0_) * 0.2;
    }
    const Trace &tr = traces_[meas_trace_];
    TraceAxis ax = axis();
    if (tr.size() != ax.n) return;
    const double lo = meas_f0_ - meas_bw_ * 0.5, hi = meas_f0_ + meas_bw_ * 0.5;
    if (meas_mode_ == MeasMode::ChannelPower)
        meas_cp_ = channel_power(tr.db().data(), tr.size(), ax, lo, hi, cur_.detector);
    else
        meas_obw_ = occupied_bw(tr.db().data(), tr.size(), ax, lo, hi,
                                double(meas_obw_pct_) / 100.0);
}

void App::draw_measure_group() {
    int m = int(meas_mode_);
    ImGui::RadioButton("off##meas", &m, 0);
    ImGui::SameLine();
    ImGui::RadioButton("channel power", &m, 1);
    ImGui::SameLine();
    ImGui::RadioButton("OBW", &m, 2);
    meas_mode_ = MeasMode(m);
    if (meas_mode_ == MeasMode::Off) return;

    if (!ImGui::IsAnyItemActive()) {
        std::snprintf(buf_meas_c_, sizeof buf_meas_c_, "%s", format_frequency(meas_f0_, 6).c_str());
        std::snprintf(buf_meas_w_, sizeof buf_meas_w_, "%s", format_frequency(meas_bw_, 6).c_str());
    }
    ImGui::PushItemWidth(-90);
    double v = 0;
    if (ImGui::InputText("band centre", buf_meas_c_, sizeof buf_meas_c_,
                         ImGuiInputTextFlags_EnterReturnsTrue) &&
        parse_frequency(buf_meas_c_, &v))
        meas_f0_ = v;
    if (ImGui::InputText("band width", buf_meas_w_, sizeof buf_meas_w_,
                         ImGuiInputTextFlags_EnterReturnsTrue) &&
        parse_frequency(buf_meas_w_, &v) && v > 0)
        meas_bw_ = v;
    ImGui::PopItemWidth();
    if (ImGui::SmallButton("from markers 1-2")) {
        std::string err;
        if (!meas_from_markers(&err)) note(err, true);
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("= view")) {
        meas_f0_ = (view_x0_ + view_x1_) * 0.5;
        meas_bw_ = view_x1_ - view_x0_;
    }
    int t = int(meas_trace_) + 1;
    ImGui::PushItemWidth(60);
    if (ImGui::InputInt("trace##meas", &t, 0)) meas_trace_ = size_t(std::max(1, std::min(4, t)) - 1);
    if (meas_mode_ == MeasMode::Obw) {
        ImGui::SameLine();
        ImGui::InputFloat("%##obw", &meas_obw_pct_, 0, 0, "%.1f");
        meas_obw_pct_ = std::max(1.0f, std::min(99.99f, meas_obw_pct_));
    }
    ImGui::PopItemWidth();

    meas_update();
    if (meas_mode_ == MeasMode::ChannelPower) {
        if (!meas_cp_.valid) {
            ImGui::TextDisabled("%s", meas_cp_.reason.empty() ? "no reading" : meas_cp_.reason.c_str());
        } else {
            ImGui::Text("power  %.2f %s", meas_cp_.power_db, units());
            ImGui::Text("PSD    %.2f %s/Hz", meas_cp_.psd_db_hz, units());
            ImGui::TextDisabled("over %s, %zu points, NBW %s", format_frequency(meas_cp_.width_hz, 4).c_str(),
                                meas_cp_.n_points, format_frequency(cur_.rbw_hz, 3).c_str());
            if (!meas_cp_.warning.empty())
                ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f), "%s", meas_cp_.warning.c_str());
        }
    } else {
        if (!meas_obw_.valid) {
            ImGui::TextDisabled("%s", meas_obw_.reason.empty() ? "no reading" : meas_obw_.reason.c_str());
        } else {
            ImGui::Text("OBW    %s  (%.1f%%)", format_frequency(meas_obw_.width_hz, 5).c_str(),
                        double(meas_obw_pct_));
            ImGui::Text("from   %s", format_frequency(meas_obw_.f_lo, 7).c_str());
            ImGui::Text("to     %s", format_frequency(meas_obw_.f_hi, 7).c_str());
            ImGui::TextDisabled("of %.2f %s in the band", meas_obw_.total_db, units());
        }
    }
}

void App::draw_measure_overlay() {
    if (meas_mode_ == MeasMode::Off || cfg_.zero_span()) return;
    meas_update();
    const ImPlotRect lim = ImPlot::GetPlotLimits();
    const double lo = meas_f0_ - meas_bw_ * 0.5, hi = meas_f0_ + meas_bw_ * 0.5;
    const double xs[2] = {lo, hi}, ylo[2] = {lim.Y.Min, lim.Y.Min}, yhi[2] = {lim.Y.Max, lim.Y.Max};
    ImPlotSpec band;
    band.FillColor = ImVec4(0.4f, 0.7f, 1.0f, 1.0f);
    band.FillAlpha = 0.12f;
    ImPlot::PlotShaded("##measband", xs, ylo, yhi, 2, band);
    ImPlotSpec bedge;
    bedge.LineColor = ImVec4(0.4f, 0.7f, 1.0f, 0.7f);
    ImPlot::PlotInfLines("##measedge", xs, 2, bedge);
    char txt[160] = {};
    if (meas_mode_ == MeasMode::ChannelPower && meas_cp_.valid) {
        std::snprintf(txt, sizeof txt, "CP %.2f %s   PSD %.2f %s/Hz", meas_cp_.power_db, units(),
                      meas_cp_.psd_db_hz, units());
    } else if (meas_mode_ == MeasMode::Obw && meas_obw_.valid) {
        // The occupied band itself, as two edges inside the measurement band.
        ImPlotSpec edge;
        edge.LineColor = ImVec4(1.0f, 0.85f, 0.3f, 1.0f);
        edge.LineWeight = 1.5f;
        const double e[2] = {meas_obw_.f_lo, meas_obw_.f_hi};
        ImPlot::PlotInfLines("##obw", e, 2, edge);
        std::snprintf(txt, sizeof txt, "OBW %s (%.1f%%)", format_frequency(meas_obw_.width_hz, 5).c_str(),
                      double(meas_obw_pct_));
    }
    if (txt[0])
        ImPlot::Annotation(meas_f0_, lim.Y.Max, ImVec4(0.4f, 0.7f, 1.0f, 1.0f), ImVec2(0, 12), true,
                           "%s", txt);
}

// --------------------------------------------------------- signal generator

bool App::gen_send(bool on, std::string *err) {
    sweepd::GenConfig g = gen_edit_;
    g.on = on;
    if (!gen_sweep_) g.sweep_step_hz = 0;
    return src_->set_generator(g, err);
}

void App::draw_generator_group() {
    if (!gen_seeded_) {
        gen_edit_ = cfg_.gen;
        if (!(gen_edit_.freq_hz > 0)) gen_edit_.freq_hz = (long long)((view_x0_ + view_x1_) * 0.5);
        gen_sweep_ = cfg_.gen.sweep_step_hz != 0;
        gen_seeded_ = true;
    }
    const sweepd::GenConfig &now = cfg_.gen;
    if (now.on) {
        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.35f, 1.0f), "TX ON");
        ImGui::SameLine();
        if (now.sweep_step_hz)
            ImGui::Text("sweeping %s - %s, %zu steps",
                        format_frequency(double(now.sweep_start_hz), 6).c_str(),
                        format_frequency(double(now.sweep_stop_hz), 6).c_str(), cfg_.gen_sweep_points);
        else
            ImGui::Text("%s  %s", format_frequency(double(now.freq_hz), 9).c_str(),
                        sweepd::gen_mod_name(now.mod));
        ImGui::TextDisabled("attenuation %.2f dB%s", now.atten_db,
                            now.mod == sweepd::GenMod::CW
                                ? ""
                                : ("  rate " + format_frequency(cfg_.gen_rate_achieved_hz, 6)).c_str());
    } else {
        ImGui::TextDisabled("TX off");
    }
    if (cfg_.tracking) {
        ImGui::TextDisabled("the tracking generator is using the TX");
        return;
    }

    if (!ImGui::IsAnyItemActive()) {
        std::snprintf(buf_gen_f_, sizeof buf_gen_f_, "%s", format_frequency(double(gen_edit_.freq_hz), 9).c_str());
        std::snprintf(buf_gen_s0_, sizeof buf_gen_s0_, "%s", format_frequency(double(gen_edit_.sweep_start_hz), 6).c_str());
        std::snprintf(buf_gen_s1_, sizeof buf_gen_s1_, "%s", format_frequency(double(gen_edit_.sweep_stop_hz), 6).c_str());
        std::snprintf(buf_gen_st_, sizeof buf_gen_st_, "%s", format_frequency(double(gen_edit_.sweep_step_hz), 6).c_str());
    }
    ImGui::PushItemWidth(-90);
    double v = 0;
    auto freq_in = [&](const char *label, char *buf, size_t cap, long long *dst) {
        if (ImGui::InputText(label, buf, cap, ImGuiInputTextFlags_EnterReturnsTrue)) {
            if (parse_frequency(buf, &v) && v >= 0) *dst = (long long)std::llround(v);
            else note(std::string("cannot read \"") + buf + "\" as a frequency", true);
            return true;
        }
        return false;
    };
    bool changed = false;
    if (!gen_sweep_) changed |= freq_in("frequency", buf_gen_f_, sizeof buf_gen_f_, &gen_edit_.freq_hz);
    float att = float(gen_edit_.atten_db);
    if (ImGui::InputFloat("TX atten", &att, 1.0f, 10.0f, "%.2f dB",
                          ImGuiInputTextFlags_EnterReturnsTrue)) {
        gen_edit_.atten_db = double(att);
        changed = true;
    }
    int mod = int(gen_edit_.mod);
    if (ImGui::Combo("modulation", &mod, "CW\0AM\0FM\0ASK\0FSK\0")) {
        gen_edit_.mod = sweepd::GenMod(mod);
        changed = true;
    }
    if (gen_edit_.mod != sweepd::GenMod::CW) {
        float rate = float(gen_edit_.rate_hz);
        if (ImGui::InputFloat(gen_edit_.mod == sweepd::GenMod::AM || gen_edit_.mod == sweepd::GenMod::FM
                                  ? "tone Hz"
                                  : "symbol Hz",
                              &rate, 0, 0, "%.1f", ImGuiInputTextFlags_EnterReturnsTrue)) {
            gen_edit_.rate_hz = double(rate);
            changed = true;
        }
    }
    if (gen_edit_.mod == sweepd::GenMod::AM) {
        float d = float(gen_edit_.am_depth * 100.0);
        if (ImGui::SliderFloat("depth", &d, 0.0f, 100.0f, "%.0f %%")) gen_edit_.am_depth = double(d) / 100.0;
        changed |= ImGui::IsItemDeactivatedAfterEdit();
    }
    if (gen_edit_.mod == sweepd::GenMod::FM || gen_edit_.mod == sweepd::GenMod::FSK) {
        float dev = float(gen_edit_.fm_dev_hz);
        if (ImGui::InputFloat(gen_edit_.mod == sweepd::GenMod::FM ? "deviation Hz" : "shift +/- Hz",
                              &dev, 0, 0, "%.0f", ImGuiInputTextFlags_EnterReturnsTrue)) {
            gen_edit_.fm_dev_hz = double(dev);
            changed = true;
        }
    }
    changed |= ImGui::Checkbox("sweep (Gen/Sweeper)", &gen_sweep_);
    if (gen_sweep_) {
        changed |= freq_in("start##gs", buf_gen_s0_, sizeof buf_gen_s0_, &gen_edit_.sweep_start_hz);
        changed |= freq_in("stop##gs", buf_gen_s1_, sizeof buf_gen_s1_, &gen_edit_.sweep_stop_hz);
        changed |= freq_in("step##gs", buf_gen_st_, sizeof buf_gen_st_, &gen_edit_.sweep_step_hz);
        float dw = float(gen_edit_.sweep_dwell_s * 1e3);
        if (ImGui::InputFloat("dwell ms", &dw, 0, 0, "%.0f", ImGuiInputTextFlags_EnterReturnsTrue)) {
            gen_edit_.sweep_dwell_s = double(dw) * 1e-3;
            changed = true;
        }
    }
    ImGui::PopItemWidth();

    if (ImGui::SmallButton("from marker") && markers_.current().on) {
        gen_edit_.freq_hz = (long long)std::llround(markers_.current().x);
        changed = true;
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("put the carrier on the active marker");
    ImGui::SameLine();
    if (ImGui::SmallButton("= view")) {
        gen_edit_.freq_hz = (long long)std::llround((view_x0_ + view_x1_) * 0.5);
        gen_edit_.sweep_start_hz = (long long)std::llround(view_x0_);
        gen_edit_.sweep_stop_hz = (long long)std::llround(view_x1_);
        if (!(gen_edit_.sweep_step_hz > 0))
            gen_edit_.sweep_step_hz = (long long)std::llround((view_x1_ - view_x0_) / 20.0);
        changed = true;
    }

    // A running generator follows its keys; a stopped one waits for TX ON.
    std::string err;
    if (now.on && changed && !gen_send(true, &err)) note("generator refused: " + err, true);
    if (ImGui::Button(now.on ? "update" : "TX ON")) {
        if (gen_send(true, &err)) note("generator on", false);
        else note("generator refused: " + err, true);
    }
    ImGui::SameLine();
    if (ImGui::Button("TX OFF")) {
        if (!gen_send(false, &err)) note("generator: " + err, true);
    }
    ImGui::TextDisabled("CW sits at the LO: no image, no LO spur\nbeside it. Modulation rates snap to\nwhole periods in the TX buffer.");
}

// ----------------------------------------------------------- noise figure

namespace {
const char *const kNfStep[4] = {"cal: source ON", "cal: source OFF", "DUT: source ON",
                                "DUT: source OFF"};
const char *const kNfHint[4] = {
    "noise source straight into the receiver, +28 V ON",
    "noise source straight into the receiver, OFF",
    "noise source -> DUT -> receiver, ON",
    "noise source -> DUT -> receiver, OFF",
};
}  // namespace

void App::nf_start(int which) {
    // The first reading fixes the points; the other three must be of the same.
    if (which == 0 || nf_f_.empty()) {
        nf_f_.clear();
        const double lo = double(cfg_.f_start_hz), hi = double(cfg_.f_stop_hz);
        const double edge = double(nf_bw_khz_) * 1e3;
        int n = std::max(1, nf_points_);
        for (int k = 0; k < n; ++k) {
            const double f = n == 1 ? (lo + hi) / 2
                                    : lo + edge + (hi - lo - 2 * edge) * double(k) / double(n - 1);
            nf_f_.push_back(f);
        }
        for (auto &a : nf_acc_) a.assign(nf_f_.size(), 0.0);
        for (size_t &g : nf_got_) g = 0;
        nf_res_.clear();
    }
    nf_acc_[which].assign(nf_f_.size(), 0.0);
    nf_got_[which] = 0;
    nf_capturing_ = which;
    nf_left_ = size_t(std::max(1, nf_sweeps_));
    note(std::string("noise figure: measuring \"") + kNfStep[which] + "\" -- " + kNfHint[which],
         false);
}

void App::nf_feed(const sweepd::proto::TraceMsg &f) {
    if (nf_capturing_ < 0 || f.x_kind != sweepd::proto::XKind::Frequency || !(f.bin_hz > 0))
        return;
    TraceAxis ax;
    ax.x0 = double(f.f_start_hz);
    ax.dx = f.bin_hz;
    ax.n = f.db.size();
    ax.rbw_hz = f.rbw_hz;
    const double half = double(nf_bw_khz_) * 1e3 * 0.5;
    std::vector<double> &acc = nf_acc_[nf_capturing_];
    for (size_t k = 0; k < nf_f_.size(); ++k) {
        const BandResult r =
            channel_power(f.db.data(), f.db.size(), ax, nf_f_[k] - half, nf_f_[k] + half, f.detector);
        if (r.valid) acc[k] += std::pow(10.0, r.power_db / 10.0);
    }
    ++nf_got_[nf_capturing_];
    if (--nf_left_ == 0) {
        note(std::string("noise figure: \"") + kNfStep[nf_capturing_] + "\" done", false);
        nf_capturing_ = -1;
        nf_compute();
    }
}

void App::nf_compute() {
    nf_res_.assign(nf_f_.size(), NfResult());
    nf_enr_at_.assign(nf_f_.size(), double(nf_enr_fixed_));
    for (size_t k = 0; k < nf_f_.size(); ++k) {
        double enr = double(nf_enr_fixed_);
        if (nf_enr_.loaded() && !nf_enr_.enr_db(nf_f_[k], &enr)) {
            nf_res_[k].reason = "outside the ENR table";
            continue;
        }
        nf_enr_at_[k] = enr;
        auto avg = [&](int i) { return nf_got_[i] ? nf_acc_[i][k] / double(nf_got_[i]) : 0.0; };
        if (nf_got_[2] && nf_got_[3]) {
            NfReadings x{avg(0), avg(1), avg(2), avg(3)};
            nf_res_[k] = nf_dut(enr, x);
        } else if (nf_got_[0] && nf_got_[1]) {
            nf_res_[k] = nf_receiver(enr, avg(0), avg(1));
        } else {
            nf_res_[k].reason = "needs readings 1 and 2 (and 3 and 4 for a DUT)";
        }
    }
}

bool App::nf_save(const std::string &path, std::string *err) {
    FILE *fp = std::fopen(path.c_str(), "w");
    if (!fp) {
        *err = "cannot write " + path;
        return false;
    }
    std::fprintf(fp, "# sweepsa noise figure, Y-factor; ENR %s\n",
                 nf_enr_.loaded() ? "from a table" : "fixed");
    std::fprintf(fp, "# band %.0f Hz per point, %d sweeps per reading%s\n",
                 double(nf_bw_khz_) * 1e3, nf_sweeps_, cfg_.synthetic ? "; SYNTHETIC" : "");
    std::fprintf(fp, "freq_hz,enr_db,nf_receiver_db,gain_db,nf_cascade_db,nf_dut_db,note\n");
    for (size_t k = 0; k < nf_res_.size(); ++k) {
        const NfResult &r = nf_res_[k];
        std::fprintf(fp, "%.0f,%.2f,%.3f,%.3f,%.3f,%.3f,%s\n", nf_f_[k], nf_enr_at_[k], r.nf2_db,
                     r.gain_db, r.nf12_db, r.nf1_db,
                     !r.valid ? r.reason.c_str() : r.low_gain ? "low gain" : "");
    }
    std::fclose(fp);
    return true;
}

void App::draw_nf_group() {
    ImGui::TextDisabled("Y-factor with a calibrated noise source\n(HP346A/B/C or similar).");
    ImGui::PushItemWidth(-90);
    ImGui::InputText("ENR file", buf_nf_enr_, sizeof buf_nf_enr_);
    ImGui::PopItemWidth();
    if (ImGui::SmallButton("load ENR")) {
        std::string err;
        if (nf_enr_.load_csv(buf_nf_enr_, &err))
            note("ENR table: " + std::to_string(nf_enr_.size()) + " points", false);
        else
            note("ENR: " + err, true);
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("freq,enr_db per line -- the values on YOUR source's calibration\n"
                          "label. A model's typical curve is not a calibration.");
    ImGui::SameLine();
    if (nf_enr_.loaded()) {
        ImGui::TextDisabled("%zu pts %s-%s", nf_enr_.size(), format_frequency(nf_enr_.f_min(), 3).c_str(),
                            format_frequency(nf_enr_.f_max(), 3).c_str());
        ImGui::SameLine();
        if (ImGui::SmallButton("unload##enr")) nf_enr_.clear();
    } else {
        ImGui::PushItemWidth(70);
        ImGui::InputFloat("ENR dB (no table)", &nf_enr_fixed_, 0, 0, "%.2f");
        ImGui::PopItemWidth();
    }
    ImGui::PushItemWidth(70);
    ImGui::InputInt("points", &nf_points_, 0);
    ImGui::SameLine();
    ImGui::InputFloat("kHz band", &nf_bw_khz_, 0, 0, "%.0f");
    ImGui::InputInt("sweeps per reading", &nf_sweeps_, 0);
    ImGui::PopItemWidth();
    nf_points_ = std::max(1, std::min(201, nf_points_));
    nf_sweeps_ = std::max(1, std::min(1000, nf_sweeps_));
    nf_bw_khz_ = std::max(1.0f, nf_bw_khz_);
    if (cur_.detector != sweepd::Detector::RMS && cur_.detector != sweepd::Detector::Sample)
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f),
                           "set the RMS detector: noise power under\nPeak reads high and Y comes out wrong");

    for (int i = 0; i < 4; ++i) {
        const bool busy = nf_capturing_ == i;
        char lbl[64];
        std::snprintf(lbl, sizeof lbl, "%d %s%s", i + 1, kNfStep[i], busy ? " ..." : "");
        if (ImGui::Button(lbl) && nf_capturing_ < 0) nf_start(i);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", kNfHint[i]);
        ImGui::SameLine();
        if (busy)
            ImGui::Text("%zu/%d", nf_got_[i], nf_sweeps_);
        else if (nf_got_[i])
            ImGui::TextDisabled("done");
        else
            ImGui::TextDisabled("-");
    }
    ImGui::TextDisabled("1+2 alone give the receiver's own NF.");

    if (nf_res_.empty()) return;
    const bool dut = nf_got_[2] && nf_got_[3];
    if (ImGui::BeginTable("##nf", dut ? 4 : 2, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit)) {
        ImGui::TableSetupColumn("freq");
        ImGui::TableSetupColumn(dut ? "NF dut" : "NF rx");
        if (dut) {
            ImGui::TableSetupColumn("gain");
            ImGui::TableSetupColumn("NF rx");
        }
        ImGui::TableHeadersRow();
        for (size_t k = 0; k < nf_res_.size(); ++k) {
            const NfResult &r = nf_res_[k];
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%s", format_frequency(nf_f_[k], 5).c_str());
            ImGui::TableNextColumn();
            if (!r.valid) {
                ImGui::TextDisabled("%s", r.reason.empty() ? "-" : r.reason.c_str());
                continue;
            }
            if (dut) {
                if (r.low_gain) ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f), "%.2f*", r.nf1_db);
                else ImGui::Text("%.2f", r.nf1_db);
                ImGui::TableNextColumn();
                ImGui::Text("%.2f", r.gain_db);
                ImGui::TableNextColumn();
                ImGui::Text("%.2f", r.nf2_db);
            } else {
                ImGui::Text("%.2f", r.nf2_db);
            }
        }
        ImGui::EndTable();
    }
    if (dut) ImGui::TextDisabled("* DUT gain < 10 dB: the receiver correction\n  dominates and the NF is uncertain");
    if (ImGui::SmallButton("save NF csv")) {
        std::string err;
        if (nf_save("sweepsa-nf.csv", &err)) note("saved sweepsa-nf.csv", false);
        else note(err, true);
    }
}

// --------------------------------------------------------- marker monitor

void App::mon_record() {
    if (!mon_on_) return;
    const Marker &m = markers_.current();
    if (!m.on) return;
    const Trace &tr = traces_[m.trace];
    const TraceAxis ax = axis();
    if (!ax.valid() || tr.size() != ax.n) return;
    const MarkerReadout r = read_marker(m, nullptr, tr.db().data(), tr.size(), ax);
    if (!r.valid) return;
    const double now = mono_now();
    if (mon_t_.empty()) mon_t0_ = now;
    mon_t_.push_back(now - mon_t0_);
    mon_v_.push_back(r.abs_db);
    // An hour at 1 sweep/s, or a few minutes fast; then the oldest go.
    if (mon_t_.size() > 4000) {
        mon_t_.erase(mon_t_.begin(), mon_t_.begin() + 1000);
        mon_v_.erase(mon_v_.begin(), mon_v_.begin() + 1000);
    }
}

void App::draw_marker_monitor() {
    const Marker &m = markers_.current();
    if (!m.on) {
        ImGui::TextDisabled("the active marker is off");
        return;
    }
    if (ImPlot::BeginPlot("##mon", ImVec2(-1, 140), ImPlotFlags_NoTitle | ImPlotFlags_NoLegend |
                                                         ImPlotFlags_NoMouseText)) {
        ImPlot::SetupAxes("s", units(), ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
        if (!mon_t_.empty()) {
            ImPlotSpec line;
            line.LineColor = kTraceColors[m.trace];
            ImPlot::PlotLine("##monl", mon_t_.data(), mon_v_.data(), int(mon_t_.size()), line);
        }
        ImPlot::EndPlot();
    }
    if (!mon_v_.empty()) {
        double mn = mon_v_[0], mx = mon_v_[0];
        for (double v : mon_v_) {
            mn = std::min(mn, v);
            mx = std::max(mx, v);
        }
        ImGui::TextDisabled("%zu readings  last %.2f  min %.2f  max %.2f", mon_v_.size(),
                            mon_v_.back(), mn, mx);
    }
}

TraceAxis App::axis() const {
    TraceAxis ax;
    if (!have_frame_ || cur_.db.empty()) return ax;

    if (cur_.x_kind == sweepd::proto::XKind::Time) {
        if (cur_.dt_ns == 0) return ax;   // decode_trace refuses this; belt and braces
        // Negative under a video trigger, by exactly the pre-trigger history,
        // so t=0 is the edge. Taken from the frame and not from cfg_ for the
        // reason the whole function exists: the two are out of step across a
        // reconfiguration, and an axis assembled from config would be confident
        // and wrong for those frames.
        ax.x0 = double(cur_.t0_ns) * 1e-9;
        ax.dx = double(cur_.dt_ns) * 1e-9;
        ax.time = true;
    } else {
        if (!(cur_.bin_hz > 0)) return ax;
        ax.x0 = double(cur_.f_start_hz);
        ax.dx = cur_.bin_hz;
    }
    ax.n = cur_.db.size();
    // A bandwidth in both modes -- see TraceAxis. The noise-density divisor
    // does not change just because the x-axis did.
    ax.rbw_hz = cur_.rbw_hz;
    return ax;
}

// One formatter for whichever axis is live, so a number on the screen cannot be
// labelled in the wrong unit. Everything that prints an x value goes through
// this.
std::string App::fmt_x(double x) const {
    if (!cfg_.zero_span()) return format_frequency(x, 4);
    return format_seconds(x);
}

const std::vector<float> *App::marker_data(const Marker &m) const {
    const size_t i = m.trace < TraceStore::kCount ? m.trace : 0;
    const Trace &t = traces_[i];
    if (!t.visible()) return nullptr;
    return &t.db();
}

PeakOptions App::search_opts() const {
    PeakOptions o = markers_.peak;
    // §6's peak search is over what is on screen. The bins outside the view are
    // still swept and still in the trace, but a "peak search" that jumped the
    // marker somewhere the user cannot see reads as the marker having been lost.
    const TraceAxis ax = axis();
    if (!ax.valid()) return o;

    double a = std::floor((view_x0_ - ax.x0) / ax.dx);
    double b = std::ceil((view_x1_ - ax.x0) / ax.dx);
    a = std::max(0.0, std::min(a, double(ax.n)));
    b = std::max(0.0, std::min(b, double(ax.n)));
    if (b > a) {
        o.lo = size_t(a);
        o.hi = size_t(b);
    } else {
        // The view does not overlap the swept span at all. lo == hi == n is an
        // empty range; lo == hi == 0 would mean "the whole trace" to
        // PeakOptions, and searching all of it is the one answer that would be
        // wrong here.
        o.lo = o.hi = ax.n;
    }
    return o;
}

bool App::run_search(int which, std::string *err) {
    Marker &m = markers_.current();
    const std::vector<float> *d = marker_data(m);
    const TraceAxis ax = axis();
    if (!d || !ax.valid()) {
        *err = "no visible trace to search";
        return false;
    }

    const PeakOptions o = search_opts();
    size_t from = 0;
    const bool have_from = ax.bin_of(m.x, &from);
    size_t bin = 0;
    bool ok = false;
    switch (which) {
        case 0: ok = peak_search(d->data(), d->size(), o, &bin); break;
        case 1:
            ok = next_peak_lower(d->data(), d->size(), o, have_from ? from : d->size(),
                                 &bin);
            break;
        case 2:
            ok = have_from && next_peak_left(d->data(), d->size(), o, from, &bin);
            break;
        default:
            ok = have_from && next_peak_right(d->data(), d->size(), o, from, &bin);
            break;
    }
    if (!ok) {
        // Naming the criterion matters: "no peak found" on a screen with a
        // visible signal reads as a broken search, when what actually happened
        // is that the excursion is set higher than the signal stands above its
        // own surroundings.
        *err = "no further peak clears the " + std::to_string(int(o.excursion_db)) +
               " dB excursion in the visible span";
        return false;
    }
    m.on = true;
    m.x = ax.x_at(bin);
    return true;
}

void App::run() {
    double t_prev = mono_now();
    last_report_t_ = t_prev;

    while (!glfwWindowShouldClose(win_)) {
        glfwPollEvents();
        pump_source();
        // §7's image check advances once per display frame, after the sweeps it
        // was fed. On the render thread and not on the source's, because every
        // step of it is a request to the source and the refusals have to land
        // somewhere they can be shown.
        step_image_check();

        // One command per frame, not all at once. They are key presses, and key
        // presses have sweeps between them: entering Max Hold clears the trace,
        // so a batch that ran `trace 2 max hold` and `csv out.csv` back to back
        // would export an empty column and be right to. `wait N` buys more.
        //
        // It also waits for the first sweep before starting at all, because
        // almost every command needs the bin grid and a peak search against no
        // trace is a refusal rather than a test of anything. That is a latch,
        // not have_frame_: a reconfiguration clears have_frame_ until the next
        // sweep arrives, and under a video trigger the next sweep may never
        // arrive -- so gating on it would strand the rest of the sequence at
        // the exact command that armed the trigger.
        if (have_frame_) any_frame_seen_ = true;
        if (!commands_done_ && any_frame_seen_) {
            if (cmd_frames_ < opt_.command_after) {
                ++cmd_frames_;
            } else if (cmd_wait_ > 0) {
                --cmd_wait_;
            } else if (cmd_next_ < opt_.commands.size()) {
                const std::string c = opt_.commands[cmd_next_++];
                std::string cerr;
                if (run_command(c, &cerr)) {
                    std::fprintf(stderr, "do: %s\n", c.c_str());
                } else {
                    std::fprintf(stderr, "do: %s -- FAILED: %s\n", c.c_str(), cerr.c_str());
                    note("do: " + cerr, true);
                }
                if (cmd_next_ >= opt_.commands.size()) commands_done_ = true;
            } else {
                commands_done_ = true;
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        draw();
        ImGui::Render();

        int fb_w = 0, fb_h = 0;
        glfwGetFramebufferSize(win_, &fb_w, &fb_h);
        gl::Viewport(0, 0, fb_w, fb_h);
        gl::ClearColor(0.06f, 0.06f, 0.08f, 1.0f);
        gl::Clear(gl::kColorBufferBit);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Both screenshot paths read the framebuffer here: after the draw, and
        // before the swap makes the back buffer's contents undefined.
        if (!want_png_.empty()) {
            const std::string p = want_png_;
            want_png_.clear();
            const bool ok = write_capture(fb_w, fb_h, p);
            note(ok ? "wrote " + p : "could not write " + p, !ok);
        }
        const bool capture_now =
            !opt_.capture_path.empty() && ++frames_ >= opt_.capture_after;
        if (capture_now && !write_capture(fb_w, fb_h, opt_.capture_path))
            std::fprintf(stderr, "could not write %s\n", opt_.capture_path.c_str());

        glfwSwapBuffers(win_);
        if (capture_now) glfwSetWindowShouldClose(win_, 1);

        const double t = mono_now();
        const float ms = float((t - t_prev) * 1000.0);
        t_prev = t;
        ft_ms_[ft_head_] = ms;
        ft_head_ = (ft_head_ + 1) % ft_ms_.size();
        if (ms > worst_ms_) worst_ms_ = ms;
        if (t - last_report_t_ > 2.0) {   // a rolling worst case, not a lifetime one
            worst_ms_ = ms;
            last_report_t_ = t;
        }
    }

    // §6 sets a 60 fps target at 32k points and 1024 waterfall lines. A target
    // nobody measures is a wish, so the numbers go to stderr on the way out
    // where a run can be pasted into the docs.
    if (opt_.show_perf) {
        std::vector<float> v;
        for (float x : ft_ms_)
            if (x > 0) v.push_back(x);
        if (!v.empty()) {
            std::sort(v.begin(), v.end());
            double sum = 0;
            for (float x : v) sum += x;
            const double mean = sum / double(v.size());
            std::fprintf(stderr,
                         "frame time over the last %zu frames: mean %.2f ms (%.1f fps), "
                         "p50 %.2f, p99 %.2f, worst %.2f\n",
                         v.size(), mean, 1000.0 / mean, double(v[v.size() / 2]),
                         double(v[(v.size() * 99) / 100]), double(v.back()));
        }
    }
}

bool App::write_capture(int w, int h, const std::string &path) {
    if (w <= 0 || h <= 0 || path.empty()) return false;
    std::vector<unsigned char> px(size_t(w) * size_t(h) * 3);
    gl::PixelStorei(gl::kPackAlignment, 1);
    gl::ReadPixels(0, 0, w, h, gl::kRgb, gl::kUnsignedByte, px.data());

    // GL's framebuffer origin is bottom-left; both PPM and PNG are top-left, so
    // the rows go out in reverse. Getting this wrong produces an upside-down
    // screenshot, which is at least obvious -- unlike most of what this program
    // can get wrong.
    const bool ppm = path.size() >= 4 && path.compare(path.size() - 4, 4, ".ppm") == 0;
    if (!ppm) {
        std::string err;
        if (write_png_rgb(path, px.data(), w, h, true, &err)) return true;
        std::fprintf(stderr, "%s\n", err.c_str());
        return false;
    }

    // Kept for the case where the question is whether the PNG encoder is what
    // broke: a raw PPM needs no decoder at all.
    FILE *f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    std::fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (int y = h - 1; y >= 0; --y)
        std::fwrite(px.data() + size_t(y) * size_t(w) * 3, 1, size_t(w) * 3, f);
    return std::fclose(f) == 0;
}

// ---------------------------------------------------------------- layout

void App::draw() {
    const ImGuiViewport *vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(6, 6));
    ImGui::Begin("##root", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
                     ImGuiWindowFlags_NoSavedSettings);
    ImGui::PopStyleVar();

    const float status_h = ImGui::GetFrameHeightWithSpacing() + 4;
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float body_h = avail.y - status_h;
    const float splitter = 6.0f;
    const float left_w = std::max(200.0f, avail.x - softkey_w_ - splitter);

    ImGui::BeginChild("##left", ImVec2(left_w, body_h), false,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    {
        const float h = ImGui::GetContentRegionAvail().y;
        float trace_h = std::max(80.0f, (h - splitter) * split_frac_);
        const float wf_h = std::max(60.0f, h - splitter - trace_h);
        trace_h = h - splitter - wf_h;

        if (cfg_.iq_mode) {
            const char *why = iq_run_ == kIqStop ? "stopped -- Run or Single takes a burst"
                              : status_.waiting_for_trigger
                                  ? "armed, waiting for a trigger -- lower the level or use auto"
                                  : "waiting for the first burst";
            double lv = 0;
            if (iqv_.draw_time(ImVec2(-1, trace_h), cfg_, why, &lv)) {
                IqRequest r;
                r.trig_level = lv;
                std::string err;
                if (!src_->set_iq(r, &err)) note("trigger level refused: " + err, true);
            }
        } else {
            draw_trace(ImVec2(-1, trace_h));
        }

        // Horizontal splitter. An invisible button plus the mouse delta is the
        // whole mechanism; ImGui has no splitter widget.
        ImGui::InvisibleButton("##vsplit", ImVec2(-1, splitter));
        if (ImGui::IsItemHovered() || ImGui::IsItemActive())
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
        if (ImGui::IsItemActive() && h > 0)
            split_frac_ = std::min(0.9f, std::max(0.1f, split_frac_ +
                                                           ImGui::GetIO().MouseDelta.y / h));

        if (cfg_.iq_mode) iqv_.draw_lower(ImGui::GetContentRegionAvail(), cfg_);
        else draw_waterfall(ImVec2(-1, -1));
    }
    ImGui::EndChild();

    ImGui::SameLine(0, 0);
    ImGui::InvisibleButton("##hsplit", ImVec2(splitter, body_h));
    if (ImGui::IsItemHovered() || ImGui::IsItemActive())
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    if (ImGui::IsItemActive())
        softkey_w_ = std::min(600.0f, std::max(240.0f,
                                               softkey_w_ - ImGui::GetIO().MouseDelta.x));
    ImGui::SameLine(0, 0);

    ImGui::BeginChild("##softkeys", ImVec2(0, body_h), true);
    draw_softkeys();
    ImGui::EndChild();

    draw_status_bar();
    ImGui::End();
    draw_about();
}

// ---------------------------------------------------------------- trace

void App::draw_trace(const ImVec2 &size) {
    if (ImPlot::BeginPlot("##trace", size,
                          ImPlotFlags_NoTitle | ImPlotFlags_NoLegend |
                              ImPlotFlags_NoMouseText)) {
        // The x-axis label is the one thing on screen that says which
        // measurement this is. It is taken from the frame's own x_kind, not
        // from a UI mode flag, so it cannot disagree with the data under it.
        const bool zs = cfg_.zero_span();
        const bool lin = lin_active();
        // Lin: what one unit of amplitude is. dBFS -> fraction of full-scale
        // amplitude; dBm -> volts RMS into 50 ohm; a normalised S21/S11 -> |S|.
        // Return loss never gets here (lin_active): its linear form is 1/|S11|.
        double unit_scale = 1.0;
        const char *lin_unit = "FS";
        if (tnorm_applied_) lin_unit = tnorm_reflect_ ? "|S11|" : "|S21|";
        else if (!uncal()) { unit_scale = std::sqrt(50.0 * 1e-3); lin_unit = "V (50 ohm)"; }
        double tick_pos[11];
        char tick_lab[11][16];
        const char *tick_lab_p[11];
        const char *prefix = "";
        char ylabel[48];
        if (lin) {
            lin_ticks(double(ref_db_), unit_scale, tick_pos, tick_lab, &prefix);
            for (int k = 0; k <= 10; ++k) tick_lab_p[k] = tick_lab[k];
            std::snprintf(ylabel, sizeof ylabel, "linear, %s%s", prefix, lin_unit);
        }
        ImPlot::SetupAxes(zs ? "time" : "frequency",
                          lin               ? ylabel
                          : !tnorm_applied_ ? units()
                          : !tnorm_reflect_ ? "dB (S21)"
                          : tnorm_rl_       ? "return loss, dB"
                                            : "dB (S11)");
        ImPlot::SetupAxisFormat(ImAxis_X1, zs ? format_time_axis : format_hz_axis);
        ImPlot::SetupAxisLimits(ImAxis_X1, view_x0_, view_x1_,
                                view_dirty_ ? ImPlotCond_Always : ImPlotCond_Once);
        if (lin) {
            // Pinned every frame: the bottom line *is* zero amplitude, so a drag
            // or a wheel moves the ref level and never the floor.
            ImPlot::SetupAxisScale(ImAxis_Y1, db_to_amp, amp_to_db);
            ImPlot::SetupAxisLimits(ImAxis_Y1, double(ref_db_) - kLinFloorDb, double(ref_db_),
                                    ImPlotCond_Always);
            ImPlot::SetupAxisTicks(ImAxis_Y1, tick_pos, 11, tick_lab_p, false);
        } else {
            ImPlot::SetupAxisLimits(ImAxis_Y1, double(ref_db_ - db_per_div_ * kDivisions),
                                    double(ref_db_),
                                    amp_dirty_ ? ImPlotCond_Always : ImPlotCond_Once);
        }
        view_dirty_ = false;
        amp_dirty_ = false;

        const ImPlotRect lim = ImPlot::GetPlotLimits();
        const ImVec2 psize = ImPlot::GetPlotSize();

        // The decimator works in axis units, so it is driven from the same
        // TraceAxis the markers use rather than from cur_.bin_hz -- which is 0
        // in zero span and would silently draw nothing.
        const TraceAxis pax = axis();

        // Under the traces, so it annotates rather than obscures.
        const bool drew_persist = persist_draw();
        draw_segment_overlay();

        bool drew_anything = drew_persist;
        if (pax.valid()) {
            // §6: decimate to the pixel column count. Handing ImPlot the raw
            // 32k points would cost more than the whole rest of the frame and
            // draw nothing a pixel could resolve. Once per visible trace, so
            // four traces cost four decimations -- still far below one column
            // per pixel of drawing.
            const double t0 = mono_now();
            const size_t cols = size_t(std::max(64.0f, psize.x));
            for (size_t t = 0; t < TraceStore::kCount; ++t) {
                dec_[t].clear();
                const Trace &tr = traces_[t];
                if (!tr.visible() || tr.size() != cur_.db.size()) continue;
                decimate_minmax(tr.db().data(), tr.size(), pax.x0, pax.dx, lim.X.Min,
                                lim.X.Max, cols, &dec_[t]);
            }
            dec_us_ = (mono_now() - t0) * 1e6;

            for (size_t t = 0; t < TraceStore::kCount; ++t) {
                if (dec_[t].size() == 0) continue;
                drew_anything = true;
                const int n = int(dec_[t].size());
                char id[16];
                std::snprintf(id, sizeof id, "T%zu", t + 1);

                // The band between the per-pixel min and max, plus a line along
                // the max. The band is where the information decimation would
                // otherwise throw away lives: without it a narrow spur looks
                // like a full-height line rather than one sample of a noisy
                // column. It is a control because four of them overlapping is
                // unreadable -- but it stays on by default, because one trace
                // is the usual case and that is the case it is for.
                if (show_band_ || fill_band_) {
                    ImPlotSpec band;
                    band.FillColor = kTraceColors[t];
                    band.FillAlpha = fill_band_ ? 1.0f : 0.22f;
                    ImPlot::PlotShaded(id, dec_[t].x.data(), dec_[t].lo.data(),
                                       dec_[t].hi.data(), n, band);
                }
                ImPlotSpec line;
                line.LineColor = kTraceColors[t];
                line.LineWeight = fill_band_ ? 1.0f : 1.2f;
                ImPlot::PlotLine(id, dec_[t].x.data(), dec_[t].hi.data(), n, line);
                // Filled also joins the bottoms. Where a column holds one point
                // lo == hi and the fill has no height; the two lines are then the
                // trace itself, every point joined to the next, as SATSAGEN draws.
                if (fill_band_)
                    ImPlot::PlotLine(id, dec_[t].x.data(), dec_[t].lo.data(), n, line);
            }
        }

        if (!drew_anything) {
            const char *why = "waiting for the first sweep";
            bool any_visible = false;
            for (size_t t = 0; t < TraceStore::kCount; ++t)
                any_visible = any_visible || traces_[t].visible();
            if (paused_) why = "paused";
            else if (have_frame_ && !any_visible) why = "every trace is blanked";
            // A trace is on but drew nothing: the view does not overlap what
            // was measured. Saying "blanked" here sent the user to the trace
            // keys for a problem that is in the frequency keys.
            else if (have_frame_) why = "no measured points in this view";
            // An armed trigger that has not fired is not "no data yet": the
            // radio is running and the level is simply not being reached, which
            // is a thing the user can act on and the other message is not.
            else if (status_.waiting_for_trigger)
                why = "armed, waiting for a trigger -- lower the level or use free run";
            // Mid-height in whichever scale is drawn: half amplitude under lin.
            const double ymid = lin ? lim.Y.Max - 6.0206 : (lim.Y.Min + lim.Y.Max) * 0.5;
            ImPlot::Annotation((lim.X.Min + lim.X.Max) * 0.5, ymid,
                               ImVec4(0.7f, 0.7f, 0.7f, 1.0f), ImVec2(0, 0), false, "%s",
                               why);
        }

        // §7's image check verdicts, above the traces and below the markers.
        draw_image_overlay();
        draw_artifact_overlay();
        draw_floor_overlay();
        draw_measure_overlay();
        draw_marker_overlay();

        // Ctrl+click places the active marker. Plain click is ImPlot's pan, and
        // taking it would make the plot undraggable; a modifier keeps both.
        if (ImPlot::IsPlotHovered() && ImGui::GetIO().KeyCtrl &&
            ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            Marker &m = markers_.current();
            m.on = true;
            m.x = ImPlot::GetPlotMousePos().x;
        }

        // Remember where the plot area actually is: the waterfall below has to
        // line up with it, and the axis labels inset it by an amount that
        // depends on the font and the tick text.
        const ImVec2 ppos = ImPlot::GetPlotPos();
        ImPlot::EndPlot();

        view_x0_ = lim.X.Min;
        view_x1_ = lim.X.Max;
        ref_db_ = float(std::max(-200.0, std::min(30.0, lim.Y.Max)));
        // Under lin the Y range is a fixed 300 dB and says nothing about dB/div;
        // keep the log setting for when the user switches back.
        if (!lin) db_per_div_ = float((lim.Y.Max - lim.Y.Min) / kDivisions);

        plot_x0_ = ppos.x;
        plot_w_ = psize.x;
    }
}

// ---------------------------------------------------------------- markers
//
// Drawn inside BeginPlot/EndPlot, so it is in plot coordinates. Each live
// marker gets a drop line, a dot on the trace it reads, and a label carrying
// the same number the marker table shows -- the two must agree, so both read
// the same MarkerReadout rather than each computing one.

void App::draw_marker_overlay() {
    const TraceAxis ax = axis();
    if (!ax.valid()) return;

    for (size_t i = 0; i < MarkerSet::kCount; ++i) {
        const Marker &m = markers_[i];
        if (!m.on) continue;
        const std::vector<float> *d = marker_data(m);
        if (!d) continue;

        const Marker *ref = m.delta ? &markers_[m.ref] : nullptr;
        const MarkerReadout r = read_marker(m, ref, d->data(), d->size(), ax);

        const bool active = (i == markers_.active());
        ImVec4 col = kTraceColors[m.trace < TraceStore::kCount ? m.trace : 0];
        if (!active) {
            col.w = 0.62f;   // the active marker is the bright one
        }

        // Stagger the labels by marker number. Two markers on the same signal
        // -- which is the normal way to set up a delta -- would otherwise draw
        // their text on top of each other and produce one illegible line of
        // interleaved digits. Four rows is enough to separate the pairs people
        // actually make; beyond that they wrap and overlap again, which is
        // honest about there being eight markers and one screen.
        const ImVec2 label_off(8.0f, -9.0f - 15.0f * float(i % 4));

        // The drop line goes in at the marker's frequency whether or not there
        // is a reading there: a marker that has been placed somewhere with no
        // data still has a position, and hiding it would make it look like the
        // key press did nothing.
        double x = r.valid || r.unmeasured ? r.abs_x : m.x;
        ImPlotSpec drop;
        drop.LineColor = col;
        drop.LineWeight = active ? 1.4f : 1.0f;
        char id[24];
        std::snprintf(id, sizeof id, "##m%zu", i + 1);
        ImPlot::PlotInfLines(id, &x, 1, drop);

        char label[128];
        if (r.valid) {
            const char *units = r.is_noise ? (tnorm_applied_ ? "dB/Hz"
                                              : uncal()      ? "dBFS/Hz"
                                                             : "dBm/Hz")
                                           : this->units();
            if (r.is_delta) {
                std::snprintf(label, sizeof label, "M%zu  %s%s  %+.2f %s (d%zu)", i + 1,
                              r.x >= 0 ? "+" : "",
                              fmt_x(r.x).c_str(), r.db, units,
                              m.ref + 1);
            } else {
                std::snprintf(label, sizeof label, "M%zu  %s  %.2f %s", i + 1,
                              fmt_x(r.abs_x).c_str(), r.db, units);
            }
            // The receiver-artifact tag rides on the label, because the label is
            // where the number is read -- a level with "[LO spur?]" beside it
            // cannot be quoted as a signal by accident.
            if (const uint8_t af = art_flags_at(r.abs_x)) {
                const size_t len = std::strlen(label);
                std::snprintf(label + len, sizeof label - len, "  [%s]", artifact_tag(af));
            }
            ImPlotSpec dot;
            dot.Marker = ImPlotMarker_Circle;
            dot.MarkerSize = active ? 4.5f : 3.0f;
            dot.MarkerFillColor = col;
            dot.MarkerLineColor = col;
            const double y = r.abs_db;
            ImPlot::PlotScatter(id, &x, &y, 1, dot);
            ImPlot::Annotation(x, y, col, label_off, true, "%s", label);
        } else {
            // §7 one level up: a marker with no number says which kind of
            // nothing it found, on the plot, not only in a table nobody opened.
            std::snprintf(label, sizeof label, "M%zu  %s", i + 1, r.reason.c_str());
            ImPlot::Annotation(x, (ImPlot::GetPlotLimits().Y.Min +
                                   ImPlot::GetPlotLimits().Y.Max) * 0.5,
                               ImVec4(1.0f, 0.75f, 0.35f, 1.0f), label_off, true, "%s",
                               label);
        }
    }
}

// ---------------------------------------------------------------- waterfall

void App::draw_waterfall(const ImVec2 &size) {
    ImGui::BeginChild("##wf", size, false,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    const ImVec2 avail = ImGui::GetContentRegionAvail();
    // Match the trace's plot area horizontally, so a feature at 2.4 GHz is at
    // the same x in both panels. Without this the waterfall is offset by the
    // width of the y-axis labels and every alignment judgement made from the
    // screen is wrong.
    const float x0 = (plot_w_ > 0) ? plot_x0_ : ImGui::GetCursorScreenPos().x;
    const float w = (plot_w_ > 0) ? plot_w_ : avail.x;
    const float h = std::max(1.0f, avail.y);

    const double span = double(cfg_.f_stop_hz - cfg_.f_start_hz);
    double u0 = 0, u1 = 1;
    if (span > 0) {
        u0 = (view_x0_ - double(cfg_.f_start_hz)) / span;
        u1 = (view_x1_ - double(cfg_.f_start_hz)) / span;
    }

    const unsigned tex = wf_.render(int(w), int(h), u0, u1, wf_ref_db_, wf_range_db_);
    ImVec2 pos = ImGui::GetCursorScreenPos();
    pos.x = x0;
    ImGui::SetCursorScreenPos(pos);
    if (tex) {
        // Flipped vertically: GL's framebuffer origin is bottom-left and
        // ImGui's is top-left. Without this the history scrolls upwards and the
        // newest sweep -- the one the user is actually reading -- ends up at the
        // bottom of the panel.
        ImGui::Image((ImTextureID)(intptr_t)tex, ImVec2(w, h), ImVec2(0, 1), ImVec2(1, 0));
    } else {
        ImGui::Dummy(ImVec2(w, h));
    }

    ImGui::EndChild();
}

// ---------------------------------------------------------------- soft keys

// §6's zero span, as a soft-key group. Replaces Frequency while the mode is
// live, because "centre and span" is not what is being controlled: the LO is
// parked and the axis is time.
//
// Everything here that is not a control is a number the user needs in order to
// read the trace correctly, and each one is a thing that is otherwise invisible:
// the LO is not at the frequency they typed, each point integrates over longer
// than the spacing between points suggests, and the mode is only looking at the
// signal a few percent of the time.
// "Leave that knob alone" for the trigger setters, so one key press sends one
// field and the daemon's refusal names what the user actually turned.
static double nan_() { return std::numeric_limits<double>::quiet_NaN(); }

// ---------------------------------------------------- tracking generator
//
// The SNA's soft keys (sweepd/track.h). One group for entering, setting and
// normalizing, because on an instrument those are one task: connect a thru,
// store it, connect the DUT, read it.
void App::draw_tracking_group(const ApplyFn &apply) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (!cfg_.tracking) {
        ImGui::TextDisabled("TX follows RX: a scalar network\n"
                            "analyser for what sits between\n"
                            "them. Stops the sweep while on.");
        ImGui::Text("over %s - %s", format_frequency(view_x0_, 4).c_str(),
                    format_frequency(view_x1_, 4).c_str());
        if (ImGui::Button("start tracking")) {
            std::string err;
            apply(src_->set_span(view_x0_, view_x1_, &err) &&
                      src_->set_tracking(double(tsa_atten_in_), nan, -1, -1, &err),
                  err, "tracking");
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("transmits a multitone at %.1f dB attenuation and steps\n"
                              "both LOs together across the view", double(tsa_atten_in_));
        return;
    }

    // ---- level
    if (!ImGui::IsAnyItemActive()) tsa_atten_in_ = float(cfg_.tsa_atten_db);
    ImGui::PushItemWidth(-80);
    if (ImGui::InputFloat("TX atten", &tsa_atten_in_, 1.0f, 10.0f, "%.2f dB",
                          ImGuiInputTextFlags_EnterReturnsTrue)) {
        std::string err;
        apply(src_->set_tracking(double(tsa_atten_in_), nan, -1, -1, &err), err,
              "TX attenuation");
    }
    ImGui::PopItemWidth();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("the level control. Below %.0f dB the source refuses unless it\n"
                          "was told there is an attenuator between TX and RX.",
                          cfg_.tsa_min_atten_db);
    for (double a : {30.0, 40.0, 50.0, 60.0}) {
        if (a != 30.0) ImGui::SameLine();
        char lbl[16];
        std::snprintf(lbl, sizeof lbl, "%.0f##att", a);
        if (ImGui::SmallButton(lbl)) {
            std::string err;
            apply(src_->set_tracking(a, nan, -1, -1, &err), err, "TX attenuation");
        }
    }
    ImGui::TextDisabled("%.1f dBFS per tone at the DAC", cfg_.tsa_tone_dbfs);

    // ---- grid
    ImGui::Text("spacing:  %s  (%zu points)", format_frequency(cfg_.tsa_spacing_hz, 4).c_str(),
                cfg_.n_points);
    const double sp[] = {100e3, 250e3, 1e6, 5e6, 10e6};
    const char *spl[] = {"100k", "250k", "1M", "5M", "10M"};
    for (int k = 0; k < 5; ++k) {
        if (k) ImGui::SameLine();
        char lbl[24];
        std::snprintf(lbl, sizeof lbl, "%s##sp", spl[k]);
        if (ImGui::SmallButton(lbl)) {
            std::string err;
            apply(src_->set_tracking(nan, sp[k], -1, -1, &err), err, "spacing");
        }
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("snapped to 4 FFT bins, so the tones sit on exact bins and\n"
                          "no image or LO leakage can land on one");

    ImGui::Text("tones:    %zu per LO step%s", cfg_.tsa_tones,
                cfg_.tsa_tones_req == 0 ? " (fill)" : "");
    if (ImGui::SmallButton("fill##tn")) {
        std::string err;
        apply(src_->set_tracking(nan, nan, 0, -1, &err), err, "tones");
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("8##tn")) {
        std::string err;
        apply(src_->set_tracking(nan, nan, 8, -1, &err), err, "tones");
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("1 (CW)##tn")) {
        std::string err;
        apply(src_->set_tracking(nan, nan, 1, -1, &err), err, "tones");
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("one tone per LO step: the classic stepped-CW SNA. Slower by\n"
                          "the tone count, but no intermodulation of the stimulus can\n"
                          "land on a point -- use it for deep stopbands next to a\n"
                          "strong passband.");
    if (cfg_.tsa_tones > 1)
        ImGui::TextDisabled("multitone: odd-order IM of the tones\n"
                            "lands on the grid, so a stopband next\n"
                            "to a passband reads no deeper than the\n"
                            "receiver's own IM3. Use 1 (CW) there.");

    ImGui::Text("NBW:      %s  (N=%zu x %zu coherent)", format_frequency(cfg_.rbw_hz, 3).c_str(),
                cfg_.n_fft, cfg_.tsa_coherent);
    for (int c : {1, 8, 64}) {
        if (c != 1) ImGui::SameLine();
        char lbl[24];
        std::snprintf(lbl, sizeof lbl, "x%d##coh", c);
        if (ImGui::SmallButton(lbl)) {
            std::string err;
            apply(src_->set_tracking(nan, nan, -1, c, &err), err, "coherent averaging");
        }
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("blocks of IQ averaged before the transform. The stimulus\n"
                          "repeats every block, so each doubling buys 3 dB of dynamic\n"
                          "range for a fraction of a millisecond per step.");
    ImGui::Text("sweep:    %.2f s, %u LO steps", status_.last_sweep_ms * 1e-3,
                unsigned(cfg_.n_segments));

    // ---- what the receiver listens to
    ImGui::Text("RX at:    %dx stimulus%s%s", cfg_.tsa_harmonic,
                cfg_.tsa_rx_offset_hz != 0
                    ? (std::string(cfg_.tsa_rx_invert ? ", mirrored about " : " + ") +
                       format_frequency(cfg_.tsa_rx_offset_hz, 6))
                          .c_str()
                    : "",
                "");
    for (int n = 1; n <= 5; ++n) {
        if (n > 1) ImGui::SameLine();
        char lbl[16];
        std::snprintf(lbl, sizeof lbl, "%dx##harm", n);
        if (ImGui::SmallButton(lbl)) {
            TrackRequest r;
            r.harmonic = n;
            std::string err;
            apply(src_->set_tracking(r, &err), err, "harmonic");
        }
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("listen at N times the stimulus: a DUT's harmonic output.\n"
                          "The x axis stays the stimulus frequency.");
    if (!ImGui::IsAnyItemActive())
        std::snprintf(buf_tsa_off_, sizeof buf_tsa_off_, "%s",
                      format_frequency(cfg_.tsa_rx_offset_hz, 6).c_str());
    ImGui::PushItemWidth(-80);
    if (ImGui::InputText("RX offset", buf_tsa_off_, sizeof buf_tsa_off_,
                         ImGuiInputTextFlags_EnterReturnsTrue)) {
        double hz = 0;
        if (parse_frequency(buf_tsa_off_, &hz) || std::string(buf_tsa_off_) == "0") {
            TrackRequest r;
            r.rx_offset_hz = hz;
            std::string err;
            apply(src_->set_tracking(r, &err), err, "RX offset");
        } else {
            note(std::string("cannot read \"") + buf_tsa_off_ + "\" as a frequency", true);
        }
    }
    ImGui::PopItemWidth();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("mixers and frequency converters: the DUT's output sits this far\n"
                          "from its input -- for a mixer, its LO frequency");
    bool inv = cfg_.tsa_rx_invert;
    if (ImGui::Checkbox("mirrored (RX = offset - stimulus)", &inv)) {
        TrackRequest r;
        r.rx_invert = inv ? 1 : 0;
        std::string err;
        apply(src_->set_tracking(r, &err), err, "RX mirror");
    }

    // ---- what is connected, on a fabricated source only
    const std::vector<std::string> duts = src_->sim_duts();
    if (!duts.empty()) {
        int cur = src_->sim_dut();
        ImGui::PushItemWidth(-80);
        if (ImGui::BeginCombo("sim DUT", duts[size_t(cur)].c_str())) {
            for (size_t k = 0; k < duts.size(); ++k)
                if (ImGui::Selectable(duts[k].c_str(), int(k) == cur)) src_->set_sim_dut(int(k));
            ImGui::EndCombo();
        }
        ImGui::PopItemWidth();
    }

    // ---- normalization
    ImGui::Separator();
    int kind = tnorm_reflect_ ? 1 : 0;
    ImGui::RadioButton("transmission", &kind, 0);
    ImGui::SameLine();
    ImGui::RadioButton("reflection", &kind, 1);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("through a return-loss bridge or directional coupler: store an\n"
                          "OPEN (or a short) as the reference, then connect the DUT. The\n"
                          "trace is S11; scalar only, so no correction for the bridge's\n"
                          "own directivity.");
    if ((kind == 1) != tnorm_reflect_) {
        tnorm_reflect_ = kind == 1;
        // A reference means a different thing in the other mode.
        tnorm_.clear();
    }
    if (tnorm_reflect_) {
        ImGui::SameLine();
        ImGui::Checkbox("return loss", &tnorm_rl_);
    }
    ImGui::PushItemWidth(60);
    ImGui::InputInt("sweeps##thru", &tnorm_sweeps_, 0);
    ImGui::PopItemWidth();
    tnorm_sweeps_ = std::max(1, std::min(64, tnorm_sweeps_));
    ImGui::SameLine();
    if (tnorm_.capturing()) {
        if (ImGui::Button("cancel")) tnorm_.cancel_capture();
        ImGui::SameLine();
        ImGui::Text("storing %zu/%zu", tnorm_.captured(), tnorm_.target());
    } else if (ImGui::Button(tnorm_reflect_ ? "store open/short" : "store thru")) {
        tnorm_.start_capture(size_t(tnorm_sweeps_));
        note(std::string("storing the reference over the next ") + std::to_string(tnorm_sweeps_) +
                 (tnorm_reflect_ ? " sweeps -- leave the bridge's test port OPEN (or shorted)"
                                 : " sweeps -- connect TX straight to RX"),
             false);
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("connect a thru in place of the DUT first. Averaged in power\n"
                          "over this many sweeps; below 500 MHz each LO step lands\n"
                          "within ~0.5 dB of its last level, so more is steadier.");
    if (tnorm_.have()) {
        ImGui::Checkbox("normalize (|S21|)", &tnorm_on_);
        ImGui::SameLine();
        if (ImGui::SmallButton("clear##thru")) tnorm_.clear();
        const TrackGrid &g = tnorm_.grid();
        ImGui::TextDisabled("thru: %zu points at %.2f dB atten,\nweakest point %.1f dBFS",
                            g.n, g.atten_db, double(tnorm_.ref_min_db()));
        if (tnorm_on_ && !tnorm_applied_ && !tnorm_why_.empty())
            ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.3f, 1.0f), "not applied: %s",
                               tnorm_why_.c_str());
        if (tnorm_applied_ && !tnorm_caveat_.empty())
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.4f, 1.0f), "%s", tnorm_caveat_.c_str());
        if (tnorm_applied_ && tnorm_reflect_ && markers_.current().on) {
            // SWR from the active marker: |G| from S11, which the marker reads
            // directly or negated as return loss.
            const Marker &m = markers_.current();
            const TraceAxis ax = axis();
            const Trace &tr = traces_[m.trace];
            if (ax.valid() && tr.size() == ax.n) {
                const MarkerReadout r = read_marker(m, nullptr, tr.db().data(), tr.size(), ax);
                if (r.valid) {
                    const double s11 = tnorm_rl_ ? -r.abs_db : r.abs_db;
                    const double g = std::pow(10.0, std::min(0.0, s11) / 20.0);
                    if (g < 0.9999)
                        ImGui::Text("marker: RL %.2f dB, |G| %.3f, SWR %.2f", -s11, g,
                                    (1 + g) / (1 - g));
                    else
                        ImGui::Text("marker: RL %.2f dB, |G| 1, SWR infinite", -s11);
                }
            }
        }
        if (tnorm_applied_ && std::fabs(tnorm_comp_db_) > 0.01)
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.4f, 1.0f),
                               "compensating %+.2f dB of attenuation change;\n"
                               "the attenuator's step accuracy is now in it",
                               -tnorm_comp_db_);
    } else {
        ImGui::TextDisabled("no thru stored: levels are received\ndBFS, not |S21|");
    }

    // ---- isolation floor
    ImGui::Separator();
    if (floor_phase_ == FloorPhase::Idle) {
        if (ImGui::Button("store floor")) {
            TrackRequest r;
            r.stim_off = 1;
            std::string err;
            if (src_->set_tracking(r, &err)) {
                floor_phase_ = FloorPhase::Silencing;
                floor_skip_ = 1;
            } else {
                note("silencing the stimulus: " + err, true);
            }
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("sweep with the stimulus silenced and keep what the receiver\n"
                              "reads: points at or near this line measured nothing");
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.4f, 1.0f), "storing the floor: %s",
                           floor_phase_ == FloorPhase::Silencing   ? "silencing the stimulus"
                           : floor_phase_ == FloorPhase::Capturing ? "capturing"
                                                                   : "restoring the stimulus");
    }
    if (floor_.have()) {
        ImGui::SameLine();
        ImGui::Checkbox("show floor", &floor_show_);
        ImGui::SameLine();
        if (ImGui::SmallButton("clr##floor")) floor_.clear();
    }

    if (ImGui::Button("stop tracking")) {
        std::string err;
        apply(src_->set_sweep_mode(&err), err, "sweep mode");
    }
}

void App::draw_zero_span_group(const ApplyFn &apply) {
    if (!ImGui::CollapsingHeader("Zero span", ImGuiTreeNodeFlags_DefaultOpen)) return;

    if (!ImGui::IsAnyItemActive())
        std::snprintf(buf_center_, sizeof buf_center_, "%s",
                      format_frequency(double(cfg_.center_hz), 6).c_str());

    ImGui::PushItemWidth(-70);
    if (ImGui::InputText("centre", buf_center_, sizeof buf_center_,
                         ImGuiInputTextFlags_EnterReturnsTrue)) {
        double hz = 0;
        if (!parse_frequency(buf_center_, &hz)) {
            note(std::string("cannot read \"") + buf_center_ + "\" as a frequency", true);
        } else {
            std::string err;
            apply(src_->set_zero_span(hz, 0, 0, -1, &err), err, "zero-span centre");
        }
    }
    ImGui::PopItemWidth();
    ImGui::TextDisabled("suffixes: 2.4g  100m  455k");

    // The LO is not where the user typed. Saying so, with the offset, is the
    // difference between a reading they can interpret and one that quietly
    // disagrees with their signal generator.
    ImGui::Spacing();
    ImGui::Text("LO:       %s", format_frequency(double(cfg_.lo_hz), 6).c_str());
    ImGui::TextDisabled("          %s below the centre, to keep its own",
                        format_frequency(double(cfg_.lo_offset_hz), 3).c_str());
    ImGui::TextDisabled("          leakage out of the reading");
    ImGui::Text("meas BW:  %s", format_frequency(double(cfg_.meas_bw_hz), 3).c_str());

    if (cfg_.band_includes_dc) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.75f, 0.35f, 1.0f));
        ImGui::TextWrapped("the LO offset is 0, so this reading includes the LO's own "
                           "leakage");
        ImGui::PopStyleColor();
    }
    if (cfg_.band_exceeds_bw) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.75f, 0.35f, 1.0f));
        ImGui::TextWrapped("the measurement band reaches onto the analog filter's skirt");
        ImGui::PopStyleColor();
    }

    // dt and the resolution, together and labelled, because the overlap moves
    // only the first and a trace showing 4 us spacing with 16 us of integration
    // will render a 5 us pulse 16 us wide with nothing on screen to explain it.
    ImGui::Separator();
    ImGui::Text("dt:       %s per point", format_seconds(double(cfg_.dt_ns) * 1e-9).c_str());
    ImGui::Text("res:      %s per point", format_seconds(cfg_.time_res_s).c_str());
    if (cfg_.overlap > 0.0) {
        ImGui::TextDisabled("          overlap %.0f%% raises the point density;",
                            cfg_.overlap * 100.0);
        ImGui::TextDisabled("          it does not improve the resolution");
    }
    ImGui::Text("window:   %s",
                format_seconds(double(cfg_.n_points) * double(cfg_.dt_ns) * 1e-9).c_str());
    // ------------------------------------------------------------ trigger
    //
    // §6's "free-run and video triggering". The level is on the same dBFS axis
    // the trace is drawn on, which is what makes it settable by eye -- see
    // dsp.h's VideoTrigger for why that is the design and not a convenience.
    ImGui::Separator();
    ImGui::TextDisabled("trigger");

    static const char *kModes[] = {"free_run", "video_rising", "video_falling"};
    for (int i = 0; i < 3; ++i) {
        if (i) ImGui::SameLine();
        const bool on = cfg_.trigger == kModes[i];
        if (on) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.25f, 0.45f, 0.30f, 1.0f));
        const char *label = i == 0 ? "free" : (i == 1 ? "rise" : "fall");
        if (ImGui::Button(label)) {
            std::string err;
            apply(src_->set_trigger(kModes[i], nan_(), nan_(), nan_(), &err), err,
                  "trigger mode");
        }
        if (on) ImGui::PopStyleColor();
    }

    if (cfg_.triggered()) {
        if (!ImGui::IsAnyItemActive())
            std::snprintf(buf_trig_level_, sizeof buf_trig_level_, "%.1f",
                          cfg_.trig_level_db);
        ImGui::PushItemWidth(-70);
        if (ImGui::InputText("level dB", buf_trig_level_, sizeof buf_trig_level_,
                             ImGuiInputTextFlags_EnterReturnsTrue)) {
            std::string err;
            apply(src_->set_trigger("", std::atof(buf_trig_level_), nan_(), nan_(), &err),
                  err, "trigger level");
        }
        ImGui::PopItemWidth();

        ImGui::Text("hyst:     %.1f dB", cfg_.trig_hyst_db);
        ImGui::Text("pre-trig: %zu of %zu points", cfg_.trig_pretrig_points,
                    cfg_.n_points);
        ImGui::TextDisabled("          t=0 is the edge; points");
        ImGui::TextDisabled("          before it are negative");

        // The two windows, next to each other, because the gap between them is
        // why a rare event can be missed with the level set correctly. §7's
        // probability-of-intercept story in its triggered form.
        const double frame = double(cfg_.n_points) * double(cfg_.dt_ns) * 1e-9;
        ImGui::Text("armed:    %s per acquisition",
                    format_seconds(cfg_.capture_span_s).c_str());
        ImGui::TextDisabled("          showing %s of it",
                            format_seconds(frame).c_str());
        if (status_.acquisitions) {
            ImGui::Text("fired:    %llu of %llu",
                        (unsigned long long)(status_.acquisitions - status_.no_trigger),
                        (unsigned long long)status_.acquisitions);
            ImGui::TextDisabled("          armed %.1f%% of the time",
                                status_.armed_duty * 100.0);
        }
        if (status_.waiting_for_trigger) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.75f, 0.35f, 1.0f));
            ImGui::TextWrapped("waiting for a trigger -- the trace on screen is the last "
                               "one that fired, not the signal now");
            ImGui::PopStyleColor();
        }
    } else {
        // Free run's consequence, stated where the trigger keys are, because
        // this is the panel someone is looking at when they wonder why the
        // marker will not stay on the burst.
        ImGui::TextDisabled("free run: frames start at unrelated");
        ImGui::TextDisabled("moments, so a marker does not follow");
        ImGui::TextDisabled("a feature");
    }

    // Time-axis view controls. Zooming the time axis is a display operation and
    // never a retune -- there is nothing to retune -- so unlike the frequency
    // group there is no "tune source with view" question to answer.
    ImGui::Separator();
    const double full = double(cfg_.n_points) * double(cfg_.dt_ns) * 1e-9;
    const double t_lo = -double(cfg_.trig_pretrig_points) * double(cfg_.dt_ns) * 1e-9;
    const double t_hi = t_lo + (full > 0 ? full : 1e-3);
    if (ImGui::Button("full window")) {
        view_x0_ = t_lo;
        view_x1_ = t_hi;
        view_dirty_ = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("zoom out")) {
        const double c = (view_x0_ + view_x1_) * 0.5;
        const double s = (view_x1_ - view_x0_);
        view_x0_ = c - s;
        view_x1_ = c + s;
        view_dirty_ = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("zoom in")) {
        const double c = (view_x0_ + view_x1_) * 0.5;
        const double s = (view_x1_ - view_x0_) * 0.25;
        view_x0_ = c - s;
        view_x1_ = c + s;
        view_dirty_ = true;
    }

    if (view_x0_ < t_lo - 1e-12 || view_x1_ > t_hi + 1e-12) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.75f, 0.35f, 1.0f));
        ImGui::TextWrapped("the view reaches outside the captured window (%s - %s); "
                           "there is no data there, not quiet data",
                           format_seconds(t_lo).c_str(), format_seconds(t_hi).c_str());
        ImGui::PopStyleColor();
    }

    ImGui::Spacing();
    if (ImGui::Button("back to sweep")) {
        std::string err;
        apply(src_->set_sweep_mode(&err), err, "sweep mode");
    }
}

void App::draw_softkeys() {
    ImGui::TextDisabled("%s", src_->name());
    ImGui::Separator();

    auto apply = [&](bool ok, const std::string &err, const char *what) {
        if (ok) {
            note(std::string(what) + " applied", false);
        } else {
            // §7's argument, one level up: a control that silently does nothing
            // is worse than one that is not there.
            note(std::string(what) + " refused: " + err, true);
        }
    };

    // In zero span the frequency keys mean something different -- there is no
    // span, only a parked centre -- so the group is a different group rather
    // than the same one with dead controls. Leaving center/span/start/stop on
    // screen but inert is how a user concludes the instrument is broken.
    // Raw IQ replaces everything that is about a trace: there is no trace.
    // What is left is the burst's own controls, its display, the RX gain, and
    // System.
    if (cfg_.iq_mode) {
        draw_iq_group(apply);
        iqv_.draw_display_group(cfg_.synthetic);
        if (ImGui::CollapsingHeader("Amplitude", ImGuiTreeNodeFlags_DefaultOpen))
            draw_rx_gain(apply);
        draw_system_group();
        return;
    }

    if (cfg_.zero_span()) {
        draw_zero_span_group(apply);
    } else if (ImGui::CollapsingHeader("Frequency", ImGuiTreeNodeFlags_DefaultOpen)) {
        const double centre = (view_x0_ + view_x1_) * 0.5;
        const double span = view_x1_ - view_x0_;

        if (!ImGui::IsAnyItemActive()) {
            std::snprintf(buf_center_, sizeof buf_center_, "%s",
                          format_frequency(centre, 6).c_str());
            std::snprintf(buf_span_, sizeof buf_span_, "%s",
                          format_frequency(span, 6).c_str());
            std::snprintf(buf_start_, sizeof buf_start_, "%s",
                          format_frequency(view_x0_, 6).c_str());
            std::snprintf(buf_stop_, sizeof buf_stop_, "%s",
                          format_frequency(view_x1_, 6).c_str());
        }

        // §6: "center/span and start/stop, kept in sync". They are two views of
        // one pair of numbers, so editing either writes the same state and the
        // other redraws from it.
        ImGui::PushItemWidth(-70);
        auto entry = [&](const char *label, char *buf, size_t cap, double *out) -> bool {
            const bool enter = ImGui::InputText(label, buf, cap,
                                                ImGuiInputTextFlags_EnterReturnsTrue);
            if (!enter) return false;
            double hz = 0;
            if (!parse_frequency(buf, &hz)) {
                note(std::string("cannot read \"") + buf + "\" as a frequency", true);
                return false;
            }
            *out = hz;
            return true;
        };

        bool moved = false;
        double v = 0;
        if (entry("center", buf_center_, sizeof buf_center_, &v)) {
            view_x0_ = v - span * 0.5;
            view_x1_ = v + span * 0.5;
            view_dirty_ = moved = true;
        }
        if (entry("span", buf_span_, sizeof buf_span_, &v) && v > 0) {
            view_x0_ = centre - v * 0.5;
            view_x1_ = centre + v * 0.5;
            view_dirty_ = moved = true;
        }
        if (entry("start", buf_start_, sizeof buf_start_, &v) && v < view_x1_) {
            view_x0_ = v;
            view_dirty_ = moved = true;
        }
        if (entry("stop", buf_stop_, sizeof buf_stop_, &v) && v > view_x0_) {
            view_x1_ = v;
            view_dirty_ = moved = true;
        }
        ImGui::PopItemWidth();
        ImGui::TextDisabled("suffixes: 2.4g  100m  455k");

        // Full span is the *tuner's* range, not the span the source happens to
        // be sweeping. Taking it from the current span -- which is what it did
        // before the daemon could retune -- makes "full span" a no-op on a
        // narrow sweep, which is the opposite of what the key means.
        if (ImGui::Button("full span")) {
            view_x0_ = double(cfg_.tune_lo());
            view_x1_ = double(cfg_.tune_hi());
            view_dirty_ = moved = true;
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s - %s, the range this source reports it can tune",
                              format_frequency(double(cfg_.tune_lo()), 4).c_str(),
                              format_frequency(double(cfg_.tune_hi()), 4).c_str());
        ImGui::SameLine();
        if (ImGui::Button("zoom out")) {
            const double c = (view_x0_ + view_x1_) * 0.5;
            const double s = (view_x1_ - view_x0_);
            view_x0_ = c - s;
            view_x1_ = c + s;
            view_dirty_ = moved = true;
        }
        const double presets[] = {1e6, 10e6, 100e6, 1e9};
        const char *preset_labels[] = {"1M", "10M", "100M", "1G"};
        for (int i = 0; i < 4; ++i) {
            const double p = presets[i];
            if (i) ImGui::SameLine();
            if (ImGui::SmallButton(preset_labels[i])) {
                const double c = (view_x0_ + view_x1_) * 0.5;
                view_x0_ = c - p * 0.5;
                view_x1_ = c + p * 0.5;
                view_dirty_ = moved = true;
            }
        }

        // Moving the axis and retuning the radio are still two different acts,
        // and the switch says which one the frequency keys perform. It is not
        // hidden behind a preference, because the difference is visible in the
        // data: a view wider than the swept span shows empty axis rather than a
        // quiet band, and a retune costs a profile rebuild and a new sweep rate.
        // Only a retune narrows the RBW: under auto RBW the daemon picks N_fft
        // from the span it is sweeping, not from the span on screen.
        ImGui::Spacing();
        ImGui::Checkbox("tune source with view", &auto_tune_);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("on: the frequency keys retune the source.\n"
                              "off: they move the display only, and the sweep keeps "
                              "covering whatever it covers now.");
        if (!auto_tune_) {
            ImGui::SameLine();
            if (ImGui::Button("tune now")) {
                std::string err;
                apply(src_->set_span(view_x0_, view_x1_, &err), err, "span");
            }
        }
        if (moved && auto_tune_) {
            std::string err;
            apply(src_->set_span(view_x0_, view_x1_, &err), err, "span");
        }

        // The state that made the user's last action look like it failed: the
        // axis is showing frequencies the source is not sweeping, so those bins
        // are empty because nothing measured them, not because they are quiet.
        const double src_lo = double(cfg_.f_start_hz), src_hi = double(cfg_.f_stop_hz);
        if (view_x0_ < src_lo - 1.0 || view_x1_ > src_hi + 1.0) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.75f, 0.35f, 1.0f));
            ImGui::TextWrapped("the view reaches outside the swept span (%s - %s); "
                               "bins beyond it are unmeasured, not empty",
                               format_frequency(src_lo, 4).c_str(),
                               format_frequency(src_hi, 4).c_str());
            ImGui::PopStyleColor();
        }
    }

    if (ImGui::CollapsingHeader("Amplitude", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushItemWidth(-90);
        if (ImGui::DragFloat("ref level", &ref_db_, 0.5f, -200.0f, 30.0f, "%.1f dB"))
            amp_dirty_ = true;
        if (ImGui::RadioButton("log", !lin_scale_)) {
            lin_scale_ = false;
            amp_dirty_ = true;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("lin", lin_scale_)) lin_scale_ = true;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("linear in amplitude (volts), not power: ref level at the\n"
                              "top line, zero at the bottom, 6 dB down = half height.\n"
                              "Markers, export and the waterfall stay in dB.\n"
                              "Persistence is log-only and hides under lin.");
        if (lin_scale_ && tnorm_applied_ && tnorm_rl_) {
            ImGui::SameLine();
            ImGui::TextDisabled("(log: return loss)");
        }
        if (!lin_active()) {
            int div_idx = 3;
            const float divs[] = {1, 2, 5, 10, 20};
            for (int i = 0; i < 5; ++i)
                if (std::fabs(divs[i] - db_per_div_) < 0.51f) div_idx = i;
            if (ImGui::Combo("dB/div", &div_idx, "1\0" "2\0" "5\0" "10\0" "20\0")) {
                db_per_div_ = divs[div_idx];
                amp_dirty_ = true;
            }
        }
        ImGui::Separator();
        ImGui::DragFloat("wf top", &wf_ref_db_, 0.5f, -200.0f, 30.0f, "%.1f dB");
        ImGui::DragFloat("wf range", &wf_range_db_, 0.5f, 5.0f, 160.0f, "%.0f dB");
        ImGui::PopItemWidth();
        draw_rx_gain(apply);
        // The table under that gain (src/sweepd/gaintable.h). Only from a
        // source that reports one.
        if (!cfg_.gain_table.empty()) {
            ImGui::Text("gain table: %s", cfg_.gain_table == "stock"
                                              ? "driver's own (per band)"
                                              : cfg_.gain_table.c_str());
            ImGui::PushItemWidth(-70);
            ImGui::InputText("table file", buf_gt_, sizeof buf_gt_);
            ImGui::PopItemWidth();
            if (ImGui::Button("load table")) {
                std::string err;
                apply(load_gain_table(buf_gt_, &err), err, "gain table");
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("A different table changes what %.0f dB means. An amplitude "
                                  "correction taken under the other table no longer fits.",
                                  double(cfg_.gain_db));
            if (cfg_.gain_table != sweepd::builtin_gain_table_name()) {
                ImGui::SameLine();
                if (ImGui::Button("built-in")) {
                    std::string err;
                    apply(load_builtin_gain_table(&err), err, "gain table");
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("sweepd's default: ADI's 1.3-4 GHz table over the whole "
                                      "tuner (gaintables/adi_1300_4000_wide.txt)");
            }
            if (cfg_.gain_table != "stock") {
                ImGui::SameLine();
                if (ImGui::Button("driver's own")) {
                    std::string err;
                    apply(src_->set_gain_table("", "", &err), err, "gain table");
                }
            }
        }
        // §7: the units are dBFS and stay dBFS until a correction file is
        // loaded. Saying so here, next to the amplitude controls, is where it
        // is least avoidable.
        // Three states, not two. "dBFS (no correction loaded)" would be a lie
        // whenever a correction is loaded but does not reach across the span,
        // which is exactly the case the operator most needs explained.
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        ImGui::TextWrapped("units: %s%s", units(),
                           tnorm_applied_     ? " (|S21| against the stored thru)"
                           : !uncal()         ? ""
                           : !corr_.loaded()  ? " (no correction loaded)"
                           : !corr_.absolute()? " (correction is relative: no reference level)"
                                              : " (correction does not span the sweep)");
        ImGui::PopStyleColor();

        // ------------------------------------------- §7 amplitude correction
        ImGui::Separator();
        ImGui::PushItemWidth(-70);
        ImGui::InputText("corr file", buf_corr_, sizeof buf_corr_);
        ImGui::PopItemWidth();

        if (ImGui::Button("load correction")) {
            std::string err;
            if (!load_ampcorr(buf_corr_, &err))
                note("amplitude correction: " + err, true);
        }
        if (corr_.loaded()) {
            ImGui::SameLine();
            if (ImGui::Button("unload")) unload_ampcorr();

            ImGui::TextDisabled("%s, %zu points, %s - %s", corr_.kind_name(), corr_.size(),
                                format_frequency(corr_.f_min_hz(), 4).c_str(),
                                format_frequency(corr_.f_max_hz(), 4).c_str());
            if (!corr_.conditions().note.empty())
                ImGui::TextDisabled("%s", corr_.conditions().note.c_str());

            // A relative file is applied in full and still yields no dBm. That
            // is not a shortfall to be fixed by widening the file, so it is
            // said here rather than left to the uncovered-bins warning below.
            if (!corr_.absolute()) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.75f, 0.35f, 1.0f));
                ImGui::TextWrapped("RELATIVE correction: the shape is removed, the absolute "
                                   "level is not known. Differences between two points on "
                                   "the trace are meaningful; the number itself is not.");
                ImGui::PopStyleColor();
            }

            // The conditions the file was taken under, against the live sweep.
            // Applying a correction at a different gain, span or detector is
            // wrong by an amount nothing else on screen would reveal.
            const std::string mismatch = corr_condition_mismatch();
            if (!mismatch.empty()) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.75f, 0.35f, 1.0f));
                ImGui::TextWrapped("correction conditions differ from this sweep: %s",
                                   mismatch.c_str());
                ImGui::PopStyleColor();
            }

            // The way a loaded correction still does not yield dBm, named
            // rather than left to be inferred from a badge that did not go out.
            if (corr_uncovered_ > 0) {
                const double pct =
                    cur_.db.empty() ? 0.0
                                    : 100.0 * double(corr_uncovered_) / double(cur_.db.size());
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.75f, 0.35f, 1.0f));
                ImGui::TextWrapped("%zu of %zu bins (%.0f%%) are outside the correction and "
                                   "are UNCORRECTED. The file is not extrapolated, so the "
                                   "units stay dBFS until the span fits inside it.",
                                   corr_uncovered_, cur_.db.size(), pct);
                ImGui::PopStyleColor();
            }
        } else {
            ImGui::TextDisabled("none loaded - see docs/amplitude-correction.md");
        }
    }

    // In tracking the resolution, the point count and the step width all
    // belong to the Tracking group, and the daemon refuses these keys there.
    if (!cfg_.tracking && ImGui::CollapsingHeader("BW / Sweep", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (cfg_.n_points_capped_from)
            ImGui::Text("points:   %zu  (%zu capped)", cfg_.n_points,
                        cfg_.n_points_capped_from);
        else
            ImGui::Text("points:   %zu", cfg_.n_points);
        if (cfg_.n_points_capped_from && ImGui::IsItemHovered())
            ImGui::SetTooltip("%zu points were asked for, but the FFT has only %zu bins\n"
                              "in this span. More points would be empty gaps, not detail.\n"
                              "Narrow the RBW to get more.",
                              cfg_.n_points_capped_from, cfg_.n_points);
        if (cfg_.n_fft && !cfg_.zero_span())
            ImGui::Text("RBW:      %s  (%s, N=%zu)", format_frequency(cfg_.rbw_hz, 3).c_str(),
                        cfg_.rbw_auto ? "auto" : "fixed", cfg_.n_fft);
        else
            ImGui::Text("RBW:      %s", format_frequency(cfg_.rbw_hz, 3).c_str());
        if (!cfg_.zero_span()) ImGui::Text("segments: %u", unsigned(cfg_.n_segments));
        ImGui::Text("%s %.1f ms  (%.1f/s)", cfg_.zero_span() ? "frame:   " : "sweep:   ",
                    status_.last_sweep_ms, status_.sweeps_per_s);

        // RBW keys, labelled with what each FFT size actually achieves -- the
        // only RBWs this source can deliver, so there is nothing to snap.
        // "auto" hands the choice back to the daemon, which narrows the RBW as
        // the span narrows.
        if (cfg_.n_fft && !cfg_.zero_span() && cfg_.rbw_hz > 0) {
            if (ImGui::SmallButton("auto##rbw")) {
                std::string err;
                apply(src_->set_rbw(0, &err), err, "RBW");
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("couple RBW to span: finer at narrow spans, up to N=32768");
            for (size_t n = 1024; n <= 32768; n *= 2) {
                const double rbw = cfg_.rbw_hz * double(cfg_.n_fft) / double(n);
                char lbl[32];
                if (rbw >= 10e3) std::snprintf(lbl, sizeof lbl, "%.0fk", rbw / 1e3);
                else std::snprintf(lbl, sizeof lbl, "%.1fk", rbw / 1e3);
                ImGui::SameLine();
                if (ImGui::SmallButton(lbl)) {
                    std::string err;
                    apply(src_->set_rbw(rbw, &err), err, "RBW");
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("N=%zu; finer RBW costs FFT time per segment", n);
            }
        }

        // "auto" is 0, sweepd's "derive the count from the RBW". Zero span has
        // no such case -- its points are a time window -- so it is not offered.
        if (!cfg_.zero_span()) {
            if (ImGui::SmallButton("auto##pts")) {
                std::string err;
                apply(src_->set_points(0, &err), err, "point count");
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("one point per RBW cell");
        }
        static const size_t kPoints[] = {1024, 4096, 16384, 65536};
        for (int i = 0; i < 4; ++i) {
            if (i || !cfg_.zero_span()) ImGui::SameLine();
            char lbl[32];
            std::snprintf(lbl, sizeof lbl, "%zuk", kPoints[i] / 1024);
            if (ImGui::SmallButton(lbl)) {
                std::string err;
                apply(src_->set_points(kPoints[i], &err), err, "point count");
            }
        }

        // The segment width -- the knob with the largest effect on sweep rate,
        // because the rate is the segment count times a fixed per-segment cost
        // and the segment count is the span divided by the step.
        //
        // It is here rather than under Frequency because it is a bandwidth
        // choice, and it is spelled as keep -> step because the two differ by
        // the DC policy: splice halves the step to cover its own holes, so the
        // same width sweeps at half the rate. Showing only one of the two
        // numbers would make the segment count look wrong.
        if (!cfg_.zero_span() && cfg_.keep_hz > 0) {
            ImGui::Separator();
            ImGui::Text("segment:  %s keep -> %s step",
                        format_frequency(cfg_.keep_hz, 3).c_str(),
                        format_frequency(cfg_.seg_step_hz, 3).c_str());

            static const double kKeep[] = {20e6, 30e6, 40e6, 54e6};
            for (int i = 0; i < 4; ++i) {
                if (i) ImGui::SameLine();
                char lbl[32];
                std::snprintf(lbl, sizeof lbl, "%.0fM", kKeep[i] / 1e6);
                if (ImGui::SmallButton(lbl)) {
                    std::string err;
                    apply(src_->set_segmentation(kKeep[i], "", &err), err,
                          "segment width");
                }
            }
            int dc = cfg_.splice ? 0 : 1;
            if (ImGui::RadioButton("splice", dc == 0)) {
                std::string err;
                apply(src_->set_segmentation(0, "splice", &err), err, "DC policy");
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("blank", dc == 1)) {
                std::string err;
                apply(src_->set_segmentation(0, "blank", &err), err, "DC policy");
            }

            // §7 again: the trade has to be visible, not inferred. What a wider
            // segment costs is sensitivity at its edges -- the receiver's noise
            // floor rises with distance from the LO, steeply past about half of
            // Nyquist, and the edge of the kept band is the furthest point in
            // it. The measured curve and its conditions are in
            // docs/characterization.md, "The noise floor rises with LO offset";
            // no number is repeated here, because it depends on the sample rate,
            // the RBW and the RX gain, and a figure quoted without those is the
            // kind of confident-looking fiction §7 exists to prevent.
            //
            // Under splice it is worse than an edge effect: the fill that covers
            // each DC hole is the only data in the sweep drawn from keep/2, so
            // the penalty lands as a comb on the segment *centres* too.
            ImGui::TextWrapped(
                "segment edges sit %s from the LO, where the noise floor is "
                "higher%s. Wider sweeps faster and hears less.",
                format_frequency(cfg_.keep_hz * 0.5, 3).c_str(),
                cfg_.splice ? "; under splice the fill at each segment centre is "
                              "drawn from exactly there"
                            : "");
        }

        // §7's probability-of-intercept line, from the source's own numbers.
        // Fast sweeping trades intercept probability for rate and the user has
        // to see the trade rather than infer it.
        //
        // Zero span makes the same trade in a different shape, and a worse one:
        // it is gap-free inside a frame and blind between frames, because the
        // transform is far slower than the radio. So the number to show is the
        // duty cycle, and it needs saying in words -- a user who has just parked
        // the LO to catch a burst will reasonably assume it is always listening.
        if (cfg_.zero_span()) {
            if (status_.duty > 0)
                ImGui::TextWrapped("looking %.1f%% of the time - gap-free within each "
                                   "frame, blind between them",
                                   status_.duty * 100.0);
            if (status_.sweeps_per_s > 0)
                ImGui::TextWrapped("a burst outside a %s window, %.0f times a second, "
                                   "is not seen",
                                   format_seconds(double(cfg_.n_points) *
                                                  double(cfg_.dt_ns) * 1e-9).c_str(),
                                   status_.sweeps_per_s);
        } else if (status_.sweeps_per_s > 0) {
            ImGui::TextWrapped("revisit every %.0f ms - signals shorter than that may be "
                               "missed entirely",
                               1000.0 / status_.sweeps_per_s);
        }
    }

    if (ImGui::CollapsingHeader("Trace", ImGuiTreeNodeFlags_DefaultOpen))
        draw_trace_group();

    if (ImGui::CollapsingHeader("Marker", ImGuiTreeNodeFlags_DefaultOpen))
        draw_marker_group();

    if (ImGui::CollapsingHeader("Detector", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (cfg_.tracking) {
            // Nothing is reduced: each point is the one FFT bin its tone sits
            // on, which is what Sample means.
            ImGui::TextDisabled("sample -- one FFT bin per tone");
        } else {
            int det = 0;
            for (int i = 0; i < 4; ++i)
                if (kDetectors[i] == cfg_.detector) det = i;
            for (int i = 0; i < 4; ++i) {
                if (i) ImGui::SameLine();
                if (ImGui::RadioButton(kDetectorNames[i], det == i)) {
                    std::string err;
                    apply(src_->set_detector(kDetectors[i], &err), err, "detector");
                }
            }
        }
        int win = 1;
        for (int i = 0; i < 4; ++i)
            if (kWindows[i] == cfg_.window) win = i;
        ImGui::PushItemWidth(-70);
        if (ImGui::Combo("window", &win,
                         "Rectangular\0Hann\0Blackman-Harris\0Flat-top\0")) {
            std::string err;
            apply(src_->set_window(kWindows[win], &err), err, "window");
        }
        ImGui::PopItemWidth();
        ImGui::TextDisabled("RBW is derived from the window,\nnot asserted (§5)");
    }

    if (!cfg_.zero_span() && ImGui::CollapsingHeader("Measure")) draw_measure_group();

    if (!cfg_.zero_span() && !cfg_.tracking && ImGui::CollapsingHeader("Noise figure"))
        draw_nf_group();

    if (src_->can_generate() &&
        ImGui::CollapsingHeader("Generator", cfg_.gen.on ? ImGuiTreeNodeFlags_DefaultOpen : 0))
        draw_generator_group();

    if (src_->can_track() && !cfg_.zero_span() &&
        ImGui::CollapsingHeader("Tracking (SNA)",
                                cfg_.tracking ? ImGuiTreeNodeFlags_DefaultOpen : 0))
        draw_tracking_group(apply);

    // The way in. Once the mode is live this group is gone and the zero-span
    // group has taken Frequency's place, so there is exactly one place on
    // screen that controls it at any moment.
    if (!cfg_.zero_span() && ImGui::CollapsingHeader("Zero span")) {
        ImGui::TextDisabled("amplitude vs. TIME at a parked LO.\n"
                            "one RX chain and one LO, so this\n"
                            "stops the sweep while it runs.");

        // Where it parks: the middle of what is being swept, which is the
        // frequency already on screen. The daemon applies the same default, but
        // the button says the number rather than making the user find out by
        // pressing it.
        const double centre = (view_x0_ + view_x1_) * 0.5;
        ImGui::Text("centre: %s", format_frequency(centre, 6).c_str());
        if (ImGui::Button("enter zero span")) {
            std::string err;
            apply(src_->set_zero_span(centre, 0, 0, -1, &err), err, "zero span");
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("parks the LO and measures amplitude over time at "
                              "%s.\nThe sweep stops until you come back.",
                              format_frequency(centre, 6).c_str());
    }

    // Always listed, even from a source that cannot do it: a mode whose way in
    // is simply absent reads as "not built", which is what it looked like
    // against sweepd before the daemon side existed. Say why instead.
    if (!cfg_.zero_span() && ImGui::CollapsingHeader("Raw IQ (time sink)")) {
        ImGui::TextDisabled("I and Q against TIME at a parked LO,\n"
                            "like GNU Radio's time sink: the waveform\n"
                            "itself, not its power. Stops the sweep.");
        if (!src_->can_iq()) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.75f, 0.35f, 1.0f));
            ImGui::TextWrapped("not available from the %s source yet. It runs against the "
                               "synthetic source (start sweepui without --connect); "
                               "sweepd's side is not implemented.",
                               src_->name());
            ImGui::PopStyleColor();
        } else {
        const double centre = (view_x0_ + view_x1_) * 0.5;
        // From the active marker when there is one: the thing the user was
        // looking at is the thing they want to see the waveform of.
        double at = centre;
        const Marker &mk = markers_.current();
        if (mk.on && !cfg_.zero_span()) at = mk.x;
        ImGui::Text("centre: %s%s", format_frequency(at, 6).c_str(),
                    mk.on ? "  (active marker)" : "");
        if (ImGui::Button("enter raw IQ")) {
            IqRequest r;
            r.center_hz = at;
            std::string err;
            apply(src_->set_iq(r, &err), err, "raw IQ");
        }
        }
    }

    if (ImGui::CollapsingHeader("Export", ImGuiTreeNodeFlags_DefaultOpen))
        draw_export_group();

    // §7. A debug view, so it is its own group rather than a checkbox buried in
    // System: what it shows is how the sweep was assembled, which is a different
    // subject from anything the instrument groups above it are about.
    // Opened the first time the overlay is switched on: the group is where the
    // legend lives, and three colours of vertical line appearing over the trace
    // with nothing to read them by is not an improvement.
    if (show_segments_) ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    if (ImGui::CollapsingHeader("Segments")) {
        ImGui::Checkbox("segment boundary overlay", &show_segments_);
        if (!segs_err_.empty()) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.75f, 0.4f, 1.0f));
            ImGui::TextWrapped("unavailable: %s", segs_err_.c_str());
            ImGui::PopStyleColor();
        } else if (!segs_.empty()) {
            const ImPlotRect vlim{view_x0_, view_x1_, 0, 1};
            const SegmentOverlay ov =
                build_overlay(segs_, vlim.X.Min, vlim.X.Max, double(std::max(64.0f, plot_w_)));
            ImGui::Text("%zu segments, %zu in view", segs_.size(), ov.n_in_view);
            ImGui::Text("keep %s, step %s", format_frequency(cfg_.keep_hz, 3).c_str(),
                        format_frequency(cfg_.seg_step_hz, 3).c_str());
            // blank_hz is a half-width on the wire; the hole is twice it. Shown
            // as the full width because that is what is on screen.
            ImGui::Text("dc_policy=%s, hole %s wide", cfg_.splice ? "splice" : "blank",
                        format_frequency(cfg_.blank_hz * 2.0, 3).c_str());
            if (ov.too_dense)
                ImGui::TextWrapped(
                    "centres are closer than a few pixels here -- zoom in to draw them");
            // The finding this overlay was built for. Stated where the overlay
            // is turned on, because a comb at every centre otherwise reads as
            // hundreds of narrow signals (PROJECT-STATE.md, "The splice fill is
            // hot").
            if (cfg_.splice)
                ImGui::TextWrapped(
                    "under splice each centre's hole is filled from a neighbour, at "
                    "keep/2 from that segment's LO -- the only data in the sweep taken "
                    "that far out, and measured +4.5 dB hot on this hardware");
            else
                ImGui::TextWrapped(
                    "under blank each centre's hole is left unmeasured; those bins are "
                    "NaN, not quiet");
            ImGui::TextWrapped(
                "blue: LO centre and its hole. grey: where ownership crosses to the "
                "next segment. red: that segment overloaded on the last sweep.");
        }
    }

    // §7's image check. Below Segments because it uses the same grid the overlay
    // draws, and above System because it is a measurement rather than a setting.
    if (ImGui::CollapsingHeader("Artifacts")) draw_artifact_group();

    if (img_phase_ != ImgPhase::Idle) ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    if (ImGui::CollapsingHeader("Image check")) draw_image_check_group();

    draw_system_group();
}

// ---------------------------------------------------------------- traces

// ---------------------------------------------------------------- raw IQ

void App::iq_set_run(int run) {
    iq_run_ = run;
    if (run == kIqStop) {
        src_->iq_disarm();
        iq_armed_ = false;
    }
}

void App::draw_iq_group(const ApplyFn &apply) {
    if (!ImGui::CollapsingHeader("Raw IQ", ImGuiTreeNodeFlags_DefaultOpen)) return;
    auto send = [&](const IqRequest &r, const char *what) {
        std::string err;
        apply(src_->set_iq(r, &err), err, what);
    };

    // Run / Stop / Single, a scope's three keys. Run keeps one request in
    // flight; Single takes the next burst and stops on it.
    const ImVec4 on(0.25f, 0.55f, 0.30f, 1.0f);
    auto key = [&](const char *label, int run) {
        const bool lit = iq_run_ == run;
        if (lit) ImGui::PushStyleColor(ImGuiCol_Button, on);
        if (ImGui::Button(label)) iq_set_run(run);
        if (lit) ImGui::PopStyleColor();
    };
    key("run", kIqRun);
    ImGui::SameLine();
    key("stop", kIqStop);
    ImGui::SameLine();
    key("single", kIqSingle);
    ImGui::SameLine();
    ImGui::TextDisabled(iq_run_ == kIqStop ? "stopped" : iq_armed_ ? "armed" : "idle");

    if (!ImGui::IsAnyItemActive())
        std::snprintf(buf_iq_center_, sizeof buf_iq_center_, "%s",
                      format_frequency(double(cfg_.iq_center_hz), 9).c_str());
    ImGui::PushItemWidth(-70);
    if (ImGui::InputText("centre", buf_iq_center_, sizeof buf_iq_center_,
                         ImGuiInputTextFlags_EnterReturnsTrue)) {
        double hz = 0;
        if (!parse_frequency(buf_iq_center_, &hz)) {
            note(std::string("cannot read \"") + buf_iq_center_ + "\" as a frequency", true);
        } else {
            IqRequest r;
            r.center_hz = hz;
            send(r, "IQ centre");
        }
    }
    ImGui::PopItemWidth();
    ImGui::TextDisabled("the LO sits ON the centre: its leakage is the DC");

    // Burst length: the powers of four the buffer allows at this decimation.
    const size_t max_n = cfg_.iq_max_samples();
    {
        static const size_t lens[] = {1024, 4096, 16384, 65536, 262144, 1048576, 4194304};
        static const char *names[] = {"1 k", "4 k", "16 k", "64 k", "256 k", "1 M", "4 M"};
        ImGui::Text("samples");
        for (int i = 0; i < 7; ++i) {
            if (lens[i] > max_n) break;
            ImGui::SameLine();
            const bool lit = cfg_.iq_n_samples == lens[i];
            if (lit) ImGui::PushStyleColor(ImGuiCol_Button, on);
            if (ImGui::SmallButton(names[i])) {
                IqRequest r;
                r.n_samples = (long long)lens[i];
                send(r, "burst length");
            }
            if (lit) ImGui::PopStyleColor();
        }
    }
    {
        static const unsigned ds[] = {1, 2, 4, 8, 16, 32, 64};
        ImGui::Text("decim  ");
        for (int i = 0; i < 7; ++i) {
            ImGui::SameLine();
            const bool lit = cfg_.iq_decim == ds[i];
            if (lit) ImGui::PushStyleColor(ImGuiCol_Button, on);
            char l[8];
            std::snprintf(l, sizeof l, "%u", ds[i]);
            if (ImGui::SmallButton(l)) {
                IqRequest r;
                r.decim = int(ds[i]);
                // A longer burst than the new decimation allows would be
                // refused; shrink it in the same request instead.
                const size_t cap = sweepd::IqDecimator(ds[i]).output_for(cfg_.iq_max_input);
                if (cfg_.iq_n_samples > cap) r.n_samples = (long long)cap;
                send(r, "decimation");
            }
            if (lit) ImGui::PopStyleColor();
        }
    }
    const double win = cfg_.iq_fs_hz > 0 ? double(cfg_.iq_n_samples) / cfg_.iq_fs_hz : 0;
    ImGui::Text("fs %s   window %s", format_frequency(cfg_.iq_fs_hz, 5).c_str(),
                format_seconds(win, 4).c_str());
    if (cfg_.iq_decim > 1) {
        ImGui::TextDisabled("flat to +/-%s, %zu-tap filter on the ARM",
                            format_frequency(cfg_.iq_fs_hz * sweepd::IqDecimator::kPassFraction, 3)
                                .c_str(),
                            cfg_.iq_taps);
    } else {
        ImGui::TextDisabled("raw ADC samples; the analog filter is %s wide",
                            format_frequency(cfg_.iq_rf_bw_hz, 3).c_str());
    }
    // What a burst costs on the link (§2), which is what sets the burst rate.
    const double mb = double(cfg_.iq_n_samples) * (cfg_.iq_decim == 1 ? 4.0 : 8.0) / 1e6;
    ImGui::TextDisabled("%.2f MB per burst; at ~50 MB/s that is %s on the wire", mb,
                        format_seconds(mb / 50.0, 3).c_str());

    // The trigger.
    ImGui::Separator();
    const sweepd::IqTrigger &t = cfg_.iq_trig;
    ImGui::Text("trigger");
    const char *modes[] = {"none", "normal", "auto"};
    for (int i = 0; i < 3; ++i) {
        ImGui::SameLine();
        if (ImGui::RadioButton(modes[i], int(t.mode) == i)) {
            IqRequest r;
            r.trig_mode = modes[i];
            send(r, "trigger mode");
        }
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("none: free run, the burst starts wherever the capture did.\n"
                          "normal: only a burst with an edge in it is shown.\n"
                          "auto: an edge if one turns up within the auto time,\n"
                          "otherwise the last capture, marked as untriggered.");
    if (t.mode != sweepd::IqTrigMode::None) {
        const char *srcs[] = {"I", "Q", "|x|"};
        const char *src_keys[] = {"i", "q", "mag"};
        ImGui::Text("on     ");
        for (int i = 0; i < 3; ++i) {
            ImGui::SameLine();
            if (ImGui::RadioButton(srcs[i], int(t.source) == i)) {
                IqRequest r;
                r.trig_source = src_keys[i];
                send(r, "trigger source");
            }
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("rise", t.rising)) {
            IqRequest r;
            r.trig_rising = 1;
            send(r, "trigger slope");
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("fall", !t.rising)) {
            IqRequest r;
            r.trig_rising = 0;
            send(r, "trigger slope");
        }
        ImGui::PushItemWidth(-90);
        if (!ImGui::IsAnyItemActive()) {
            iq_lv_ = float(t.level);
            iq_hy_ = float(t.hyst);
            iq_pre_ = float(t.pretrig * 100.0);
            iq_au_ = float(cfg_.iq_auto_s * 1e3);
        }
        ImGui::DragFloat("level, FS", &iq_lv_, 0.0005f, -1.5f, 1.5f, "%.4f");
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            IqRequest r;
            r.trig_level = iq_lv_;
            send(r, "trigger level");
        }
        ImGui::DragFloat("hysteresis", &iq_hy_, 0.0002f, 0.0f, 0.5f, "%.4f");
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            IqRequest r;
            r.trig_hyst = iq_hy_;
            send(r, "trigger hysteresis");
        }
        ImGui::SliderFloat("pre-trigger", &iq_pre_, 0.0f, 95.0f, "%.0f %%");
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            IqRequest r;
            r.pretrig = iq_pre_ / 100.0;
            send(r, "pre-trigger");
        }
        if (t.mode == sweepd::IqTrigMode::Auto) {
            ImGui::DragFloat("auto after", &iq_au_, 1.0f, 1.0f, 10000.0f, "%.0f ms");
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                IqRequest r;
                r.auto_s = iq_au_ / 1e3;
                send(r, "auto timeout");
            }
        }
        ImGui::PopItemWidth();
        ImGui::TextDisabled("or drag the yellow line on the plot");
    }

    // The probability-of-intercept line, the same trade §7 makes visible for
    // the sweep: bursts are gap-free inside, blind between.
    if (iqv_.have()) {
        ImGui::Separator();
        ImGui::Text("%.1f bursts/s, looking %.1f%% of the time", iqv_.bursts_per_s(),
                    iqv_.duty() * 100.0);
        ImGui::TextDisabled("gap-free inside each burst, blind between them");
    }

    ImGui::Spacing();
    if (ImGui::Button("back to sweep")) {
        std::string err;
        apply(src_->set_sweep_mode(&err), err, "sweep mode");
    }
}

// The RX gain, shared by the Amplitude group and the raw IQ group: the one
// amplitude control that means the same thing in both.
void App::draw_rx_gain(const ApplyFn &apply) {
    // The RX gain. Every change restarts the sweep, so it is sent when the
    // value is committed -- Enter, the +/- buttons, or leaving the field --
    // not on every keystroke or drag tick.
    if (cfg_.gain_max_db > cfg_.gain_min_db) {
        if (!gain_editing_) gain_edit_db_ = int(std::lround(cfg_.gain_db));
        const int lo = int(std::ceil(cfg_.gain_min_db));
        const int hi = int(std::floor(cfg_.gain_max_db));
        ImGui::PushItemWidth(-90);
        const bool changed = ImGui::InputInt("RX gain", &gain_edit_db_, 1, 10);
        // InputInt is a group (field, -, +). Active means the text field is
        // being typed in; a +/- click changes the value with nothing left
        // active, and is a commit on its own.
        gain_editing_ = ImGui::IsItemActive();
        const bool commit = (changed && !gain_editing_) || ImGui::IsItemDeactivatedAfterEdit();
        ImGui::PopItemWidth();
        gain_edit_db_ = std::max(lo, std::min(hi, gain_edit_db_));
        if (commit && gain_edit_db_ != int(std::lround(cfg_.gain_db))) {
            std::string err;
            apply(src_->set_gain(double(gain_edit_db_), &err), err, "RX gain");
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("manual RX gain, %d - %d dB under this table. AGC is off\n"
                              "and stays off: it cannot settle inside a dwell (§5).\n"
                              "Applied between sweeps; the profile table is kept.\n"
                              "An amplitude correction taken at another gain no\n"
                              "longer fits.", lo, hi);
    } else {
        ImGui::Text("RX gain: %.1f dB", double(cfg_.gain_db));
    }
}

void App::draw_system_group() {
    if (ImGui::CollapsingHeader("System", ImGuiTreeNodeFlags_DefaultOpen)) {
        int cm = int(wf_.colormap());
        ImGui::PushItemWidth(-70);
        if (ImGui::Combo("colormap", &cm, "turbo\0viridis\0grayscale\0classic green\0"))
            wf_.set_colormap(Colormap(cm));
        ImGui::PopItemWidth();
        ImGui::Text("waterfall: %zu x %zu, %zu filled", wf_.cols(), wf_.rows(),
                    wf_.rows_filled());
        ImGui::Checkbox("performance overlay", &opt_.show_perf);
        if (opt_.show_perf) {
            // Only the slots that have been written. Averaging the zeros the
            // history starts as would report a frame rate the program has never
            // achieved, which is the one thing a performance readout must not
            // do.
            float sum = 0, worst = 0;
            size_t n = 0;
            for (float v : ft_ms_) {
                if (v <= 0) continue;
                sum += v;
                worst = std::max(worst, v);
                ++n;
            }
            const float mean = n ? sum / float(n) : 0.0f;
            ImGui::Text("frame %.2f ms mean, %.2f ms worst (%zu)", double(mean),
                        double(worst), n);
            ImGui::Text("decimate %.0f us of it", dec_us_);
            ImGui::PlotLines("##ft", ft_ms_.data(), int(ft_ms_.size()), int(ft_head_),
                             nullptr, 0.0f, 33.0f, ImVec2(-1, 40));
        }
        if (ImGui::Button("about sweepsa")) show_about_ = true;
        ImGui::Separator();
        ImGui::Text("source: %s", src_->name());
        if (!status_.detail.empty()) ImGui::TextWrapped("%s", status_.detail.c_str());
        ImGui::TextWrapped("%s", cfg_.description.c_str());
    }

    // The most recent notes, newest first. Refusals in particular have to be
    // visible somewhere that is not a terminal the user may not be looking at.
    if (!notes_.empty()) {
        ImGui::Separator();
        const double now = mono_now();
        for (auto it = notes_.rbegin(); it != notes_.rend(); ++it) {
            if (now - it->t > 12.0) continue;
            ImGui::PushStyleColor(ImGuiCol_Text,
                                  it->error ? ImVec4(1.0f, 0.55f, 0.45f, 1.0f)
                                            : ImVec4(0.6f, 0.8f, 0.6f, 1.0f));
            ImGui::TextWrapped("%s", it->text.c_str());
            ImGui::PopStyleColor();
        }
    }
}

void App::draw_trace_group() {
    if (ImGui::Button(paused_ ? "run" : "pause")) paused_ = !paused_;
    ImGui::SameLine();
    if (ImGui::Button("clear waterfall")) wf_.clear();
    ImGui::SameLine();
    if (ImGui::Button("clear traces")) traces_.clear_all();
    ImGui::Checkbox("min/max band", &show_band_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("the shaded band is the spread of the bins sharing each pixel "
                          "column.\nwithout it a one-bin spur is drawn as a full-height "
                          "line.");
    ImGui::SameLine();
    ImGui::Checkbox("filled", &fill_band_);
    if (ImGui::Checkbox("persistence", &persist_on_) && !persist_on_) persist_.clear();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("density of where the trace has been: brighter where it\n"
                          "passes more often, fading over the time constant. A\n"
                          "signal present 5%% of the time shows as a dim layer.");
    if (persist_on_) {
        ImGui::SameLine();
        static const float taus[] = {0.5f, 2.0f, 10.0f, 0.0f};
        static const char *tau_names[] = {"0.5 s", "2 s", "10 s", "infinite"};
        int ti = 1;
        for (int k = 0; k < 4; ++k)
            if (persist_tau_ == taus[k]) ti = k;
        ImGui::PushItemWidth(90);
        if (ImGui::Combo("##ptau", &ti, tau_names, 4)) persist_tau_ = taus[ti];
        ImGui::PopItemWidth();
        ImGui::SameLine();
        if (ImGui::SmallButton("clr##persist")) persist_.clear();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("draw the spread solid and join every point, the way SATSAGEN\n"
                          "and most analyzers do: full-span noise reads as a filled\n"
                          "band. Same data -- only the picture changes.");
    ImGui::Separator();

    for (size_t t = 0; t < TraceStore::kCount; ++t) {
        Trace &tr = traces_[t];
        ImGui::PushID(int(t));

        // A colour swatch, so the soft key and the line on screen are tied
        // together without a legend to read.
        ImGui::ColorButton("##sw", kTraceColors[t],
                           ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoPicker,
                           ImVec2(14, 14));
        ImGui::SameLine();
        ImGui::Text("%zu", t + 1);
        ImGui::SameLine();

        int mode = 0;
        for (int i = 0; i < kTraceModeCount; ++i)
            if (kTraceModes[i] == tr.mode()) mode = i;
        ImGui::PushItemWidth(-90);
        if (ImGui::Combo("##mode", &mode,
                         "clear/write\0max hold\0min hold\0average\0view\0blank\0"))
            tr.set_mode(kTraceModes[mode]);
        ImGui::PopItemWidth();

        ImGui::SameLine();
        if (ImGui::SmallButton("clr")) tr.clear();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("restart this trace's hold or average");

        if (tr.mode() == TraceMode::Average) {
            int n = tr.average_n();
            ImGui::PushItemWidth(-90);
            if (ImGui::InputInt("N", &n, 1, 10)) tr.set_average_n(n);
            ImGui::PopItemWidth();
            ImGui::SameLine();
            // An average is a different number before it has converged, and the
            // difference is largest exactly when the user is watching it settle.
            if (tr.converged())
                ImGui::TextDisabled("converged");
            else
                ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.35f, 1.0f), "%llu/%d",
                                   (unsigned long long)tr.sweeps(), tr.average_n());
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("averaging linear power, then converting once.\n"
                                  "averaging dB would read a few dB low on noise (§5).");
        }
        ImGui::PopID();
    }
}

// ---------------------------------------------------------------- markers

void App::draw_marker_group() {
    // Which marker the keys act on. Eight small buttons rather than a combo:
    // on an instrument this is a row of numbered keys and the lit one is the
    // active one, and that reads faster than a dropdown.
    for (size_t i = 0; i < MarkerSet::kCount; ++i) {
        if (i) ImGui::SameLine();
        char lbl[8];
        std::snprintf(lbl, sizeof lbl, "%zu", i + 1);
        const bool active = i == markers_.active();
        if (active) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.30f, 0.45f, 0.65f, 1));
        else if (markers_[i].on)
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.22f, 0.30f, 0.38f, 1));
        else
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyle().Colors[ImGuiCol_Button]);
        if (ImGui::Button(lbl, ImVec2(26, 0))) markers_.set_active(i);
        ImGui::PopStyleColor();
    }

    Marker &m = markers_.current();
    ImGui::Checkbox("on", &m.on);
    ImGui::SameLine();
    ImGui::Checkbox("delta", &m.delta);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("read the difference from another marker, in dB and in Hz");
    if (m.delta) {
        ImGui::SameLine();
        int ref = int(m.ref);
        ImGui::PushItemWidth(70);
        if (ImGui::Combo("ref", &ref, "1\0" "2\0" "3\0" "4\0" "5\0" "6\0" "7\0" "8\0"))
            m.ref = size_t(ref);
        ImGui::PopItemWidth();
    }

    int tr = int(m.trace);
    ImGui::PushItemWidth(-90);
    if (ImGui::Combo("reads", &tr, "trace 1\0trace 2\0trace 3\0trace 4\0"))
        m.trace = size_t(tr);
    ImGui::PopItemWidth();

    ImGui::Checkbox("noise (per Hz)", &m.noise);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("normalise to a 1 Hz bandwidth using the achieved RBW\n"
                          "(%s), not the display bin width.",
                          format_frequency(cfg_.rbw_hz, 3).c_str());
    // The correction this cannot make, said out loud. Under a peak detector the
    // trace's floor sits above the mean noise power by an amount that depends
    // on the decimation factor, so the density reads high -- and nothing in the
    // frame says by how much.
    // Free-run zero span is not phase-locked to anything, so successive frames
    // start at unrelated points in whatever the signal is doing. A marker holds
    // a *time*, so it keeps reading the same place on the axis while the signal
    // underneath it moves -- peak search finds a burst and the next frame has
    // the marker sitting in a gap, reading 40 dB lower.
    //
    // That is not a bug to fix here; it is what a trigger is for, and §6 asks
    // for one. Until it exists the marker readout has to say why it is jumping,
    // or the instrument reads as broken.
    if (cfg_.zero_span() && cfg_.trigger == "free_run" && m.on) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.75f, 0.35f, 1.0f));
        ImGui::TextWrapped("free run: each frame starts at an unrelated moment, so a "
                           "marker at a fixed time does not follow a feature. Max Hold "
                           "is the stable reading until triggering lands.");
        ImGui::PopStyleColor();
    }

    if (m.noise && !noise_detector_ok(cfg_.detector)) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.75f, 0.35f, 1.0f));
        ImGui::TextWrapped("the %s detector does not estimate the mean, so this density "
                           "reads high. use RMS/Avg or Sample.",
                           sweepd::detector_name(cfg_.detector));
        ImGui::PopStyleColor();
    }

    ImGui::Spacing();
    auto search_key = [&](const char *label, int which) {
        if (!ImGui::Button(label)) return;
        std::string serr;
        if (!run_search(which, &serr)) note(serr, true);
    };
    search_key("peak", 0);
    ImGui::SameLine();
    search_key("next peak", 1);
    ImGui::SameLine();
    search_key("< left", 2);
    ImGui::SameLine();
    search_key("right >", 3);

    ImGui::PushItemWidth(-110);
    float exc = float(markers_.peak.excursion_db);
    if (ImGui::DragFloat("excursion", &exc, 0.25f, 0.5f, 60.0f, "%.1f dB"))
        markers_.peak.excursion_db = exc;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("how far a candidate must rise above its surroundings to "
                          "count as a peak.\nlower it and peak search walks the noise "
                          "floor; raise it and small signals stop qualifying.");
    float thr = float(markers_.peak.threshold_db);
    if (ImGui::DragFloat("threshold", &thr, 1.0f, -200.0f, 30.0f, "%.0f dB"))
        markers_.peak.threshold_db = thr;
    ImGui::PopItemWidth();

    ImGui::Spacing();
    // The two actions that move the display rather than the marker. They live
    // here because that is where the marker is, but what they write is the view.
    const TraceAxis ax = axis();
    const std::vector<float> *d = marker_data(m);
    const bool readable = m.on && d && ax.valid();
    if (!readable) ImGui::BeginDisabled();
    if (ImGui::Button(cfg_.zero_span() ? "marker -> centre of view" : "marker -> center")) {
        const double span = view_x1_ - view_x0_;
        view_x0_ = m.x - span * 0.5;
        view_x1_ = m.x + span * 0.5;
        view_dirty_ = true;
        // In zero span this is a display move and nothing else. The marker sits
        // at a *time*, and there is no frequency in it to retune to -- asking
        // the source to tune to 8.5 milliseconds is not a request it could
        // honour or refuse meaningfully.
        if (auto_tune_ && !cfg_.zero_span()) {
            std::string err;
            if (!src_->set_span(view_x0_, view_x1_, &err))
                note("span refused: " + err, true);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("marker -> ref level")) {
        const MarkerReadout r =
            readable ? read_marker(m, nullptr, d->data(), d->size(), ax) : MarkerReadout();
        if (r.valid || r.abs_db != 0) {
            ref_db_ = float(r.abs_db) + 2.0f;   // a little headroom, as on an instrument
            amp_dirty_ = true;
        } else {
            note("that marker has no level to set the reference from", true);
        }
    }
    if (!readable) ImGui::EndDisabled();

    if (ImGui::Button("all markers off")) markers_.all_off();
    ImGui::SameLine();
    ImGui::TextDisabled("ctrl+click to place");

    draw_marker_table();

    ImGui::Separator();
    ImGui::Checkbox("monitor active marker", &mon_on_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("plot the active marker's level against time, one reading\n"
                          "per sweep -- drift or an intermittent signal at one\n"
                          "frequency, as a line");
    if (mon_on_) {
        ImGui::SameLine();
        if (ImGui::SmallButton("clr##mon")) {
            mon_t_.clear();
            mon_v_.clear();
        }
        draw_marker_monitor();
    }
}

void App::draw_marker_table() {
    if (!markers_.any_on()) return;
    ImGui::Spacing();

    const TraceAxis ax = axis();
    const char *units = this->units();

    // §7. The segment column appears with the overlay and not otherwise: it is
    // the same debug question, and it is worth more here than on the plot --
    // "M1 is on a spliced fill bin" is the sentence that explains a level that
    // disagrees with its neighbours, delivered where the level is read.
    const bool seg_col = show_segments_ && !segs_.empty() && !cfg_.zero_span();

    if (!ImGui::BeginTable("##markers", seg_col ? 5 : 4,
                           ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                               ImGuiTableFlags_SizingStretchProp))
        return;
    ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, 22);
    ImGui::TableSetupColumn(cfg_.zero_span() ? "time" : "frequency");
    ImGui::TableSetupColumn("level");
    ImGui::TableSetupColumn("tr", ImGuiTableColumnFlags_WidthFixed, 22);
    // Fixed, not stretched: the contents are "12 fill" and "12 OVLD", and a
    // proportional column truncates both to "12 f" -- which reads as a segment
    // number with a typo rather than as a flag.
    if (seg_col) ImGui::TableSetupColumn("seg", ImGuiTableColumnFlags_WidthFixed, 58);
    ImGui::TableHeadersRow();

    for (size_t i = 0; i < MarkerSet::kCount; ++i) {
        const Marker &m = markers_[i];
        if (!m.on) continue;
        const std::vector<float> *d = marker_data(m);
        const Marker *ref = m.delta ? &markers_[m.ref] : nullptr;
        const MarkerReadout r =
            (d && ax.valid()) ? read_marker(m, ref, d->data(), d->size(), ax)
                              : MarkerReadout();

        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        if (i == markers_.active())
            ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "%zu", i + 1);
        else
            ImGui::Text("%zu", i + 1);

        ImGui::TableNextColumn();
        if (r.valid && r.is_delta)
            ImGui::Text("%s%s", r.x >= 0 ? "+" : "",
                        fmt_x(r.x).c_str());
        else if (r.valid || r.unmeasured)
            ImGui::Text("%s", fmt_x(r.abs_x).c_str());
        else
            ImGui::Text("%s", fmt_x(m.x).c_str());

        ImGui::TableNextColumn();
        if (r.valid) {
            // The delta's sign is always shown: "3 dB" and "-3 dB" are opposite
            // findings and the minus is one pixel wide.
            ImGui::Text("%s%.2f %s%s", r.is_delta ? "+" : "", r.db, units,
                        r.is_noise ? "/Hz" : "");
            if (const uint8_t af = art_flags_at(r.abs_x)) {
                ImGui::SameLine();
                ImGui::TextColored(art_colour_v(af), "%s", artifact_tag(af));
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", artifact_explain(af));
            }
        } else {
            // A marker with no number says which nothing it found, in the table
            // as well as on the plot -- and the table is where it will be read
            // from, because that is where a level would have been.
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.75f, 0.35f, 1.0f));
            ImGui::TextUnformatted(r.reason.empty() ? "no trace" : r.reason.c_str());
            ImGui::PopStyleColor();
        }

        ImGui::TableNextColumn();
        ImGui::TextColored(kTraceColors[m.trace < TraceStore::kCount ? m.trace : 0], "%zu",
                           m.trace + 1);

        if (seg_col) {
            ImGui::TableNextColumn();
            // The marker's own absolute position, not the delta: a delta reads
            // a difference and two frequencies, and only one of them is here.
            const double fx = r.valid && r.is_delta ? m.x : (r.valid ? r.abs_x : m.x);
            const int s = segment_at(segs_, fx);
            if (s < 0) {
                ImGui::TextDisabled("--");
            } else if (in_blanked_span(segs_, fx)) {
                ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.35f, 1.0f), "%d fill", s + 1);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        cfg_.splice
                            ? "this bin is inside a blanked hole: the value shown was\n"
                              "measured by a neighbouring segment, at keep/2 from its\n"
                              "LO. Measured +4.5 dB hot on this hardware."
                            : "this bin is inside a blanked hole and was never\n"
                              "measured. Under dc_policy=blank it is NaN.");
            } else if (have_frame_ && cur_.segment_overloaded(size_t(s))) {
                ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.45f, 1.0f), "%d OVLD", s + 1);
            } else {
                ImGui::Text("%d", s + 1);
            }
        }
    }
    ImGui::EndTable();
}

// ---------------------------------------------------------------- export

void App::draw_export_group() {
    ImGui::PushItemWidth(-70);
    ImGui::InputText("csv", buf_csv_, sizeof buf_csv_);
    ImGui::PopItemWidth();
    ImGui::SameLine();
    if (ImGui::Button("save##csv")) {
        std::string e;
        export_csv(buf_csv_, &e);
    }

    ImGui::PushItemWidth(-70);
    ImGui::InputText("png", buf_png_, sizeof buf_png_);
    ImGui::PopItemWidth();
    ImGui::SameLine();
    if (ImGui::Button("save##png")) {
        // Deferred: the framebuffer has to be read after this frame is drawn
        // and before it is swapped away, which is not now.
        want_png_ = buf_png_;
    }

    ImGui::PushItemWidth(-70);
    ImGui::InputText("state", buf_state_, sizeof buf_state_);
    ImGui::PopItemWidth();
    ImGui::SameLine();
    if (ImGui::Button("save##st")) {
        std::string e;
        save_state(buf_state_, &e);
    }
    ImGui::SameLine();
    if (ImGui::Button("load")) {
        std::string e;
        load_state(buf_state_, &e);
    }

    ImGui::TextDisabled("the CSV carries the RBW, window, detector\n"
                        "and the UNCAL state, so it stays readable.");
}

bool App::export_csv(const std::string &path, std::string *err) {
    if (!have_frame_ || cur_.db.empty()) {
        *err = "nothing to export yet";
        note(*err, true);
        return false;
    }

    CsvMeta meta;
    meta.source = src_->name();
    meta.f_start_hz = cur_.f_start_hz;
    meta.f_stop_hz = cur_.f_stop_hz;
    meta.bin_hz = cur_.bin_hz;
    meta.rbw_hz = cur_.rbw_hz;
    meta.sweeps_per_s = status_.sweeps_per_s;
    meta.seq = cur_.seq;
    meta.n_segments = cur_.n_segments;
    meta.n_overloaded_segments = cur_.n_overloaded_segments;
    meta.gain_db = cur_.gain_db;
    meta.detector = cur_.detector;
    meta.window = cur_.window;
    meta.uncal = uncal() && !tnorm_applied_;
    if (tnorm_applied_)
        meta.units = "dB(S21)";
    else if (cfg_.tracking)
        meta.units_note = "tracking generator, not normalized: received tone level, "
                          "including the radio's own TX and RX response";
    if (meta.uncal && corr_.loaded())
        meta.uncal_reason =
            !corr_.absolute()
                ? "the loaded amplitude correction is RELATIVE, so the frequency "
                  "response is removed but there is no reference level"
                : "the loaded amplitude correction does not span this trace";
    meta.synthetic = cfg_.synthetic;
    // Taken from the frame, not from cfg_: the export must describe the data it
    // is writing, and the two are briefly out of step across a mode switch.
    meta.time_axis = cur_.x_kind == sweepd::proto::XKind::Time;
    meta.dt_s = double(cur_.dt_ns) * 1e-9;
    // From the frame, like the axis and for the same reason: config and frames
    // are out of step across a reconfiguration, and a file is forever.
    meta.t0_s = double(cur_.t0_ns) * 1e-9;
    meta.trigger = cfg_.trigger;
    meta.time_res_s = cfg_.time_res_s;

    // Only the traces that are on screen, and each one labelled with the mode
    // that produced it: a column of max-hold next to a column of averaged data
    // is two different measurements and the file has to say so.
    std::vector<std::string> names, modes;
    names.reserve(TraceStore::kCount);
    modes.reserve(TraceStore::kCount);
    for (size_t t = 0; t < TraceStore::kCount; ++t) {
        if (!traces_[t].visible() || traces_[t].size() != cur_.db.size()) continue;
        names.push_back("trace" + std::to_string(t + 1));
        std::string mode = trace_mode_name(traces_[t].mode());
        if (traces_[t].mode() == TraceMode::Average)
            mode += "(" + std::to_string(traces_[t].average_n()) + ")";
        modes.push_back(mode);
    }
    std::vector<CsvColumn> cols;
    size_t k = 0;
    for (size_t t = 0; t < TraceStore::kCount; ++t) {
        if (!traces_[t].visible() || traces_[t].size() != cur_.db.size()) continue;
        cols.push_back({names[k], modes[k], traces_[t].db().data(), traces_[t].size()});
        ++k;
    }

    if (!write_trace_csv(path, meta, cols, err)) {
        note("csv: " + *err, true);
        return false;
    }
    note("wrote " + path + " (" + std::to_string(cols.size()) + " trace" +
             (cols.size() == 1 ? "" : "s") + ")",
         false);
    return true;
}

bool App::save_state(const std::string &path, std::string *err) {
    InstrumentState s;
    s.view_start_hz = view_x0_;
    s.view_stop_hz = view_x1_;
    s.ref_db = ref_db_;
    s.db_per_div = db_per_div_;
    s.lin_scale = lin_scale_;
    s.wf_ref_db = wf_ref_db_;
    s.wf_range_db = wf_range_db_;
    s.colormap = int(wf_.colormap());
    s.auto_tune = auto_tune_;
    s.show_band = show_band_;
    s.fill_band = fill_band_;
    s.show_segments = show_segments_;
    s.src_start_hz = double(cfg_.f_start_hz);
    s.src_stop_hz = double(cfg_.f_stop_hz);
    s.n_points = int(cfg_.n_points);
    s.detector = sweepd::detector_name(cfg_.detector);
    s.window = sweepd::window_name(cfg_.window);
    s.keep_hz = cfg_.keep_hz;
    s.dc_policy = cfg_.splice ? "splice" : "blank";
    s.peak_excursion_db = markers_.peak.excursion_db;
    s.peak_threshold_db = markers_.peak.threshold_db;
    s.active_marker = int(markers_.active());
    for (size_t t = 0; t < TraceStore::kCount; ++t) {
        s.trace[t].mode = trace_mode_name(traces_[t].mode());
        s.trace[t].average_n = traces_[t].average_n();
    }
    for (size_t i = 0; i < MarkerSet::kCount; ++i) {
        s.marker[i].on = markers_[i].on;
        s.marker[i].x = markers_[i].x;
        s.marker[i].delta = markers_[i].delta;
        s.marker[i].ref = int(markers_[i].ref);
        s.marker[i].trace = int(markers_[i].trace);
        s.marker[i].noise = markers_[i].noise;
    }

    if (!write_state_json(path, s, err)) {
        note("state: " + *err, true);
        return false;
    }
    note("wrote " + path, false);
    return true;
}

bool App::load_state(const std::string &path, std::string *err) {
    InstrumentState s;
    if (!read_state_json(path, &s, err)) {
        note("state: " + *err, true);
        return false;
    }

    view_x0_ = s.view_start_hz;
    view_x1_ = s.view_stop_hz;
    view_dirty_ = true;
    ref_db_ = float(s.ref_db);
    db_per_div_ = float(s.db_per_div);
    lin_scale_ = s.lin_scale;
    amp_dirty_ = true;
    wf_ref_db_ = float(s.wf_ref_db);
    wf_range_db_ = float(s.wf_range_db);
    if (s.colormap >= 0 && s.colormap <= 3) wf_.set_colormap(Colormap(s.colormap));
    auto_tune_ = s.auto_tune;
    show_band_ = s.show_band;
    fill_band_ = s.fill_band;
    // The overlay only comes back on if this source has a plan to draw. A state
    // file saved against a sweep and loaded in zero span would otherwise restore
    // a checkbox that draws nothing and gives no reason.
    show_segments_ = s.show_segments && !segs_.empty();
    markers_.peak.excursion_db = s.peak_excursion_db;
    markers_.peak.threshold_db = s.peak_threshold_db;
    markers_.set_active(size_t(s.active_marker));

    for (size_t t = 0; t < TraceStore::kCount; ++t) {
        TraceMode mo = TraceMode::Blank;
        if (trace_mode_from_string(s.trace[t].mode, &mo)) traces_[t].set_mode(mo);
        traces_[t].set_average_n(s.trace[t].average_n);
    }
    for (size_t i = 0; i < MarkerSet::kCount; ++i) {
        markers_[i].on = s.marker[i].on;
        markers_[i].x = s.marker[i].x;
        markers_[i].delta = s.marker[i].delta;
        markers_[i].ref = size_t(s.marker[i].ref);
        markers_[i].trace = size_t(s.marker[i].trace);
        markers_[i].noise = s.marker[i].noise;
    }

    // The source's own configuration is *requested*, not assumed: the daemon
    // may refuse it, and a UI that silently redrew its axes as though the
    // retune had happened would be drawing the old sweep under a new label.
    // Everything above is the display and is ours to set; this is not.
    std::string serr;
    bool asked = false;
    if (s.src_stop_hz > s.src_start_hz &&
        (s.src_start_hz != double(cfg_.f_start_hz) ||
         s.src_stop_hz != double(cfg_.f_stop_hz))) {
        asked = true;
        if (!src_->set_span(s.src_start_hz, s.src_stop_hz, &serr))
            note("state: span refused: " + serr, true);
    }
    sweepd::Detector det;
    if (sweepd::detector_from_string(s.detector, &det) && det != cfg_.detector) {
        asked = true;
        if (!src_->set_detector(det, &serr)) note("state: detector refused: " + serr, true);
    }
    sweepd::Window win;
    if (sweepd::window_from_string(s.window, &win) && win != cfg_.window) {
        asked = true;
        if (!src_->set_window(win, &serr)) note("state: window refused: " + serr, true);
    }
    if (s.n_points > 0 && size_t(s.n_points) != cfg_.n_points) {
        asked = true;
        if (!src_->set_points(size_t(s.n_points), &serr))
            note("state: point count refused: " + serr, true);
    }
    // One request carrying both, unlike the soft keys: a state file is a whole
    // configuration, and asking twice would cost two cold rebuilds to arrive at
    // the same plan. Only sent when something actually differs, for the same
    // reason -- loading the state that is already running must be free.
    {
        const std::string want_dc = s.dc_policy;
        const bool dc_differs = !want_dc.empty() && (want_dc == "splice") != cfg_.splice;
        const bool keep_differs = s.keep_hz > 0 && s.keep_hz != cfg_.keep_hz;
        if (dc_differs || keep_differs) {
            asked = true;
            if (!src_->set_segmentation(keep_differs ? s.keep_hz : 0,
                                        dc_differs ? want_dc : std::string(), &serr))
                note("state: segmentation refused: " + serr, true);
        }
    }

    note(asked ? "loaded " + path + "; the source was asked to match it"
               : "loaded " + path,
         false);
    return true;
}

// ---------------------------------------------------------------- commands
//
// The soft keys, reachable from argv. Each command calls the same method its
// button does, so this is a way of pressing the buttons rather than a second
// implementation of what they do -- which is the only property that makes it
// evidence about the UI. See AppOptions::commands for why it exists.
//
//   trace N MODE          clear/write | max hold | min hold | average | view | blank
//   avg N COUNT           sweeps to average, for a trace in average mode
//   band 0|1              the min/max shading
//   fill 0|1              the shading drawn solid, points joined (SATSAGEN look)
//   persist 0|1 [TAU] | persist clear   the density display; TAU in s, 0 = infinite
//   meas off|cp|obw [PCT] | meas band CENTRE WIDTH | meas markers | meas ?
//                         channel power / PSD / occupied bandwidth; ? prints it
//   mon 0|1               the active marker's level against time
//   nf enr FILE | nf enr-db DB | nf points N | nf band KHZ | nf sweeps N
//   nf 1|2|3|4 | nf ? | nf save PATH     Y-factor noise figure, one reading per step
//   gen on|off [freq F] [atten DB] [mod cw|am|fm|ask|fsk] [rate HZ] [depth 0..1]
//       [dev HZ] [sweep START STOP STEP DWELL_S]    the TX as a signal generator
//   clear traces|waterfall
//   active N              which marker the marker keys act on
//   marker N on|off
//   marker N at FREQ      place it (2.4g, 100m, 455k, or plain Hz)
//   marker N trace T
//   marker N delta R      delta against marker R; `delta off` to cancel
//   marker N noise 0|1
//   peak | next | left | right          on the active marker
//   excursion DB | threshold DB
//   center FREQ | span FREQ | ref DB
//   zspan on|off|FREQ [fft N] [points N] [overlap F]
//                         enter/leave §6 zero span, or retune it. "on" parks at
//                         the middle of the view. The trailing knobs are
//                         optional and each one the daemon refuses is refused by
//                         name.
//   tsa on|off [atten DB] [spacing FREQ] [tones N] [coherent N]
//                         the tracking generator; "on" covers the view. Knobs
//                         alone retune it without leaving the mode.
//   tsa thru [N]          store a thru over N sweeps (default the soft key's)
//   tsa norm 0|1 | tsa clear
//   tsa dut N             the synthetic source's simulated DUT, 0 = thru
//   tsa reflect 0|1 | tsa rl 0|1   S11 against an open/short; return loss
//   tsa floor [N]         store the isolation floor with the stimulus silenced
//   tsa map N [OFFSET] [inv]   RX at N x stimulus + OFFSET (or OFFSET - N x stim)
//   csv PATH | png PATH | save PATH | load PATH
//   pause 0|1

namespace {

std::vector<std::string> split_ws(const std::string &s) {
    std::vector<std::string> out;
    size_t i = 0;
    while (i < s.size()) {
        while (i < s.size() && std::isspace((unsigned char)s[i])) ++i;
        const size_t b = i;
        while (i < s.size() && !std::isspace((unsigned char)s[i])) ++i;
        if (i > b) out.push_back(s.substr(b, i - b));
    }
    return out;
}

// 1-based on the outside, 0-based inside: the labels on the screen say 1..8.
bool index_arg(const std::string &s, size_t n, size_t *out) {
    char *end = nullptr;
    const long v = std::strtol(s.c_str(), &end, 10);
    if (!end || *end || v < 1 || size_t(v) > n) return false;
    *out = size_t(v - 1);
    return true;
}

}  // namespace

bool App::run_command(const std::string &line, std::string *err) {
    const std::vector<std::string> a = split_ws(line);
    if (a.empty()) return true;
    const std::string &cmd = a[0];
    auto need = [&](size_t n) -> bool {
        if (a.size() < n) {
            *err = cmd + ": needs " + std::to_string(n - 1) + " argument(s)";
            return false;
        }
        return true;
    };

    if (cmd == "trace") {
        if (!need(3)) return false;
        size_t t = 0;
        if (!index_arg(a[1], TraceStore::kCount, &t)) {
            *err = "trace: index must be 1-4";
            return false;
        }
        std::string mode = a[2];
        for (size_t i = 3; i < a.size(); ++i) mode += " " + a[i];
        TraceMode m;
        if (!trace_mode_from_string(mode, &m)) {
            *err = "trace: unknown mode \"" + mode + "\"";
            return false;
        }
        traces_[t].set_mode(m);
        return true;
    }
    if (cmd == "avg") {
        if (!need(3)) return false;
        size_t t = 0;
        if (!index_arg(a[1], TraceStore::kCount, &t)) {
            *err = "avg: index must be 1-4";
            return false;
        }
        traces_[t].set_average_n(std::atoi(a[2].c_str()));
        return true;
    }
    // Raw IQ: the source side here, the display side in IqView::command.
    if (cmd == "iq") {
        if (!need(2)) return false;
        const std::string &k = a[1];
        IqRequest r;
        if (k == "off") return src_->set_sweep_mode(err);
        if (k == "run") { iq_set_run(kIqRun); return true; }
        if (k == "stop") { iq_set_run(kIqStop); return true; }
        if (k == "single") { iq_set_run(kIqSingle); return true; }
        auto num = [&](size_t i) { return i < a.size() ? std::atof(a[i].c_str()) : 0.0; };
        if (k == "n") r.n_samples = (long long)num(2);
        else if (k == "decim") r.decim = int(num(2));
        else if (k == "trig") r.trig_mode = a.size() > 2 ? a[2] : "";
        else if (k == "src") r.trig_source = a.size() > 2 ? a[2] : "";
        else if (k == "slope") r.trig_rising = (a.size() > 2 && a[2] == "fall") ? 0 : 1;
        else if (k == "level") r.trig_level = num(2);
        else if (k == "hyst") r.trig_hyst = num(2);
        else if (k == "pretrig") r.pretrig = num(2);
        else if (k == "auto") r.auto_s = num(2);
        else if (k == "on") {
            // Enter at the current centre, or keep the source's.
        } else {
            double hz = 0;
            if (!parse_frequency(k, &hz)) {
                *err = "iq: on|off|FREQ|run|stop|single|n N|decim D|trig none|normal|auto|"
                       "src i|q|mag|slope rise|fall|level FS|hyst FS|pretrig F|auto S";
                return false;
            }
            r.center_hz = hz;
        }
        return src_->set_iq(r, err);
    }
    if (cmd.size() > 2 && cmd.compare(0, 2, "iq") == 0) return iqv_.command(a, cfg_.synthetic, err);
    if (cmd == "scale") {
        if (!need(2)) return false;
        if (a[1] != "log" && a[1] != "lin") {
            *err = "scale: log or lin";
            return false;
        }
        lin_scale_ = a[1] == "lin";
        amp_dirty_ = true;
        return true;
    }
    if (cmd == "band") {
        if (!need(2)) return false;
        show_band_ = a[1] != "0";
        return true;
    }
    if (cmd == "meas") {
        if (!need(2)) return false;
        if (a[1] == "off") meas_mode_ = MeasMode::Off;
        else if (a[1] == "cp") meas_mode_ = MeasMode::ChannelPower;
        else if (a[1] == "obw") {
            meas_mode_ = MeasMode::Obw;
            if (a.size() > 2) meas_obw_pct_ = float(std::atof(a[2].c_str()));
        } else if (a[1] == "markers") return meas_from_markers(err);
        else if (a[1] == "band") {
            double c = 0, w = 0;
            if (!need(4) || !parse_frequency(a[2], &c) || !parse_frequency(a[3], &w) || !(w > 0)) {
                *err = "meas band CENTRE WIDTH";
                return false;
            }
            meas_f0_ = c;
            meas_bw_ = w;
        } else if (a[1] == "?") {
            meas_update();
            if (meas_mode_ == MeasMode::ChannelPower)
                std::printf("meas: cp %s %.3f %s psd %.3f %s/Hz width %.0f Hz points %zu%s%s\n",
                            meas_cp_.valid ? "ok" : "invalid", meas_cp_.power_db, units(),
                            meas_cp_.psd_db_hz, units(), meas_cp_.width_hz, meas_cp_.n_points,
                            meas_cp_.warning.empty() ? "" : " -- ", meas_cp_.warning.c_str());
            else if (meas_mode_ == MeasMode::Obw)
                std::printf("meas: obw %s %.0f Hz from %.0f to %.0f\n",
                            meas_obw_.valid ? "ok" : "invalid", meas_obw_.width_hz, meas_obw_.f_lo,
                            meas_obw_.f_hi);
            std::fflush(stdout);
        } else {
            *err = "meas off|cp|obw [PCT]|band CENTRE WIDTH|markers|?";
            return false;
        }
        return true;
    }
    if (cmd == "gen") {
        if (!need(2)) return false;
        if (!gen_seeded_) {
            gen_edit_ = cfg_.gen;
            gen_sweep_ = cfg_.gen.sweep_step_hz != 0;
            gen_seeded_ = true;
        }
        if (a[1] == "off") return gen_send(false, err);
        bool send_on = a[1] == "on";
        for (size_t i = send_on ? 2 : 1; i < a.size(); ++i) {
            const std::string &k = a[i];
            auto val = [&]() -> const std::string & {
                static const std::string empty;
                return i + 1 < a.size() ? a[++i] : empty;
            };
            double v = 0;
            if (k == "freq" && parse_frequency(val(), &v)) gen_edit_.freq_hz = (long long)std::llround(v), gen_sweep_ = false;
            else if (k == "atten") gen_edit_.atten_db = std::atof(val().c_str());
            else if (k == "mod") {
                if (!sweepd::gen_mod_from_string(val(), &gen_edit_.mod)) {
                    *err = "gen mod cw|am|fm|ask|fsk";
                    return false;
                }
            } else if (k == "rate") gen_edit_.rate_hz = std::atof(val().c_str());
            else if (k == "depth") gen_edit_.am_depth = std::atof(val().c_str());
            else if (k == "dev") gen_edit_.fm_dev_hz = std::atof(val().c_str());
            else if (k == "sweep") {
                double s0 = 0, s1 = 0, st = 0;
                if (i + 4 >= a.size()) {
                    *err = "gen sweep START STOP STEP DWELL_S";
                    return false;
                }
                if (!parse_frequency(val(), &s0) || !parse_frequency(val(), &s1) ||
                    !parse_frequency(val(), &st)) {
                    *err = "gen sweep START STOP STEP DWELL_S";
                    return false;
                }
                gen_edit_.sweep_start_hz = (long long)s0;
                gen_edit_.sweep_stop_hz = (long long)s1;
                gen_edit_.sweep_step_hz = (long long)st;
                gen_edit_.sweep_dwell_s = std::atof(val().c_str());
                gen_sweep_ = true;
            } else {
                *err = "gen on|off [freq F] [atten DB] [mod M] [rate HZ] [depth 0..1] [dev HZ] "
                       "[sweep START STOP STEP DWELL_S]";
                return false;
            }
        }
        if (send_on || cfg_.gen.on) return gen_send(true, err);
        return true;
    }
    if (cmd == "nf") {
        // nf enr FILE | nf enr-db DB | nf points N | nf band KHZ | nf sweeps N
        // nf 1|2|3|4 (start that reading) | nf ? (print) | nf save PATH
        if (!need(2)) return false;
        if (a[1] == "enr") return need(3) && nf_enr_.load_csv(a[2], err);
        if (a[1] == "enr-db") return need(3) && ((nf_enr_fixed_ = float(std::atof(a[2].c_str()))), true);
        if (a[1] == "points") return need(3) && ((nf_points_ = std::atoi(a[2].c_str())), true);
        if (a[1] == "band") return need(3) && ((nf_bw_khz_ = float(std::atof(a[2].c_str()))), true);
        if (a[1] == "sweeps") return need(3) && ((nf_sweeps_ = std::atoi(a[2].c_str())), true);
        if (a[1] == "save") return need(3) && nf_save(a[2], err);
        if (a[1] == "?") {
            for (size_t k = 0; k < nf_res_.size(); ++k)
                std::printf("nf: %.0f Hz %s nf_rx %.2f gain %.2f nf_dut %.2f %s\n", nf_f_[k],
                            nf_res_[k].valid ? "ok" : "invalid", nf_res_[k].nf2_db,
                            nf_res_[k].gain_db, nf_res_[k].nf1_db, nf_res_[k].reason.c_str());
            std::fflush(stdout);
            return true;
        }
        const int w = std::atoi(a[1].c_str());
        if (w < 1 || w > 4) {
            *err = "nf enr FILE|enr-db DB|points N|band KHZ|sweeps N|1..4|?|save PATH";
            return false;
        }
        if (nf_capturing_ >= 0) {
            *err = std::string("nf: \"") + kNfStep[nf_capturing_] + "\" is still being measured";
            return false;
        }
        nf_start(w - 1);
        return true;
    }
    if (cmd == "mon") {
        if (!need(2)) return false;
        mon_on_ = a[1] != "0";
        return true;
    }
    if (cmd == "persist") {
        if (!need(2)) return false;
        if (a[1] == "clear") {
            persist_.clear();
            return true;
        }
        persist_on_ = a[1] != "0";
        if (!persist_on_) persist_.clear();
        if (a.size() > 2) persist_tau_ = float(std::atof(a[2].c_str()));
        return true;
    }
    if (cmd == "fill") {
        if (!need(2)) return false;
        fill_band_ = a[1] != "0";
        return true;
    }
    // §7's overlay, through the same flag the checkbox sets. Turning it on when
    // the source reports no plan is a refusal with the reason, not a silent
    // no-op: "on" that draws nothing is indistinguishable from a bug.
    if (cmd == "segments") {
        if (!need(2)) return false;
        const bool want = a[1] != "0" && a[1] != "off";
        if (want && segs_.empty()) {
            *err = "segments: " + (segs_err_.empty() ? std::string("no segment plan yet")
                                                     : segs_err_);
            return false;
        }
        show_segments_ = want;
        return true;
    }
    // Receiver-artifact marking. `art on|off`, `art-sweeps N`, `art-comb dB`,
    // `art-spur dB`; `art-baseline N` records N sweeps (input terminated),
    // `art-baseline save|load PATH`, `art-baseline off`; `art?` prints.
    if (cmd == "art") {
        if (!need(2)) return false;
        art_on_ = a[1] != "0" && a[1] != "off";
        art_acc_ = PassAccumulator();
        if (!art_on_) art_res_ = ArtifactResult();
        return true;
    }
    if (cmd == "art-sweeps" || cmd == "art-comb" || cmd == "art-spur") {
        if (!need(2)) return false;
        const double v = std::strtod(a[1].c_str(), nullptr);
        if (!(v > 0)) {
            *err = cmd + ": expected a positive number";
            return false;
        }
        if (cmd == "art-sweeps") {
            art_sweeps_ = size_t(std::max(1.0, std::min(v, 2000.0)));
            art_acc_ = PassAccumulator();
        } else if (cmd == "art-comb") {
            art_cfg_.comb_db = v;
        } else {
            art_cfg_.spur_db = v;
        }
        return true;
    }
    if (cmd == "art-baseline") {
        if (!need(2)) return false;
        if (a[1] == "off") {
            art_base_ = ArtifactBaseline();
            art_acc_ = PassAccumulator();
            return true;
        }
        if (a[1] == "save" || a[1] == "load") {
            if (!need(3)) return false;
            if (a[1] == "save") return save_baseline(a[2], art_base_, err);
            ArtifactBaseline b;
            if (!load_baseline(a[2], &b, err)) return false;
            art_base_ = std::move(b);
            art_acc_ = PassAccumulator();
            return true;
        }
        const long nsw = std::atol(a[1].c_str());
        if (nsw < 4 || nsw > 5000) {
            *err = "art-baseline: sweeps must be between 4 and 5000";
            return false;
        }
        art_base_sweeps_ = size_t(nsw);
        return art_start_baseline(err);
    }
    if (cmd == "art?") {
        if (!art_res_.valid) {
            *err = "art?: no result yet" + (art_res_.err.empty() ? std::string() : " -- " + art_res_.err);
            return false;
        }
        std::printf("art: %zu LO/DC, %zu comb, %zu spur, %zu internal bins; fold %s%s%s\n",
                    art_res_.n_dc, art_res_.n_comb, art_res_.n_spur, art_res_.n_internal,
                    art_res_.fold_ran ? "ran" : "not run",
                    art_res_.fold_note.empty() ? "" : " -- ", art_res_.fold_note.c_str());
        for (const CombOffset &c : art_res_.comb)
            std::printf("art:   comb at LO%+.3f MHz  width %.3f MHz  median %+.1f dB  (%zu seg)\n",
                        c.offset_hz / 1e6, c.width_hz / 1e6, double(c.median_db), c.segments);
        for (const ArtifactFinding &f : art_res_.findings)
            std::printf("art:   %.6f MHz  %+.1f dB over floor  LO%+.3f MHz  %s  (%zu seg, %zu places)\n",
                        f.x_hz / 1e6, double(f.excursion_db), f.lo_offset_hz / 1e6,
                        artifact_tag(f.flags), f.recur_segments, f.recur_clusters);
        for (const ArtifactFinding &f : art_res_.unmarked)
            std::printf("art:   unmarked %.6f MHz  %+.1f dB over floor  LO%+.3f MHz\n",
                        f.x_hz / 1e6, double(f.excursion_db), f.lo_offset_hz / 1e6);
        if (!art_res_.baseline_note.empty()) std::printf("art: %s\n", art_res_.baseline_note.c_str());
        return true;
    }
    // §7's image check, through the same methods the soft keys call. `imgchk N`
    // sets the per-pass sweep count and starts; `imgchk off` clears the result.
    // It does *not* block: the check takes two passes and a profile rebuild
    // between them, so a script drives it with a following `wait`.
    if (cmd == "imgchk") {
        if (a.size() >= 2 && (a[1] == "off" || a[1] == "0")) {
            if (img_phase_ == ImgPhase::PassA || img_phase_ == ImgPhase::Shifting ||
                img_phase_ == ImgPhase::PassB || img_phase_ == ImgPhase::Restoring)
                abort_image_check("cancelled");
            else
                img_phase_ = ImgPhase::Idle;
            img_res_ = ImageCheckResult();
            return true;
        }
        if (a.size() >= 2) {
            const long n = std::atol(a[1].c_str());
            if (n < 4 || n > 5000) {
                *err = "imgchk: sweeps per pass must be between 4 and 5000";
                return false;
            }
            img_want_sweeps_ = size_t(n);
        }
        return start_image_check(err);
    }
    if (cmd == "imgchk-excursion" || cmd == "imgchk-drop") {
        if (!need(2)) return false;
        const double v = std::strtod(a[1].c_str(), nullptr);
        if (!(v > 0)) {
            *err = cmd + ": expected a positive number of dB";
            return false;
        }
        if (cmd == "imgchk-excursion") img_cfg_.min_excursion_db = v;
        else img_cfg_.move_db = v;
        return true;
    }
    // Reports what the last check concluded, so a headless run can assert on it
    // rather than on a screenshot. Refuses when there is nothing to report --
    // "0 findings" and "never ran" are different answers.
    if (cmd == "imgchk?") {
        if (img_phase_ == ImgPhase::Done) {
            std::printf("imgchk: %zu findings, %zu compared, %zu blanked, shift %.0f Hz\n",
                        img_res_.findings.size(), img_res_.n_compared, img_res_.n_blanked,
                        img_res_.shift_hz);
            for (const ImageFinding &f : img_res_.findings)
                std::printf("imgchk:   %.6f MHz  %+.1f dB over floor  drop %.1f dB  %s"
                            "  (best candidate %+.1f dB at %.6f MHz)\n",
                            f.x_hz / 1e6, double(f.excursion_a_db), double(f.drop_db),
                            image_verdict_name(f.verdict), double(f.best_candidate_db),
                            f.best_candidate_hz / 1e6);
            return true;
        }
        *err = img_phase_ == ImgPhase::Failed
                   ? "imgchk: the last check failed -- " + img_status_
                   : std::string("imgchk: no check has finished");
        return false;
    }
    // §7. Goes through the same two methods the soft keys call, so a passing
    // command is evidence about the buttons (app.h, AppOptions::commands).
    if (cmd == "ampcorr") {
        if (!need(2)) return false;
        if (a[1] == "off") {
            unload_ampcorr();
            return true;
        }
        return load_ampcorr(a[1], err);
    }
    if (cmd == "clear") {
        if (!need(2)) return false;
        if (a[1] == "traces") traces_.clear_all();
        else if (a[1] == "waterfall") wf_.clear();
        else {
            *err = "clear: traces or waterfall";
            return false;
        }
        return true;
    }
    if (cmd == "active") {
        if (!need(2)) return false;
        size_t i = 0;
        if (!index_arg(a[1], MarkerSet::kCount, &i)) {
            *err = "active: index must be 1-8";
            return false;
        }
        markers_.set_active(i);
        return true;
    }
    if (cmd == "marker") {
        if (!need(3)) return false;
        size_t i = 0;
        if (!index_arg(a[1], MarkerSet::kCount, &i)) {
            *err = "marker: index must be 1-8";
            return false;
        }
        Marker &m = markers_[i];
        const std::string &verb = a[2];
        if (verb == "on") { m.on = true; return true; }
        if (verb == "off") { m.on = false; return true; }
        if (verb == "at") {
            if (!need(4)) return false;
            double hz = 0;
            if (!parse_frequency(a[3], &hz)) {
                *err = "marker at: cannot read \"" + a[3] + "\" as a frequency";
                return false;
            }
            m.x = hz;
            m.on = true;
            return true;
        }
        if (verb == "trace") {
            if (!need(4)) return false;
            size_t t = 0;
            if (!index_arg(a[3], TraceStore::kCount, &t)) {
                *err = "marker trace: index must be 1-4";
                return false;
            }
            m.trace = t;
            return true;
        }
        if (verb == "delta") {
            if (!need(4)) return false;
            if (a[3] == "off") { m.delta = false; return true; }
            size_t r = 0;
            if (!index_arg(a[3], MarkerSet::kCount, &r)) {
                *err = "marker delta: reference must be 1-8 or off";
                return false;
            }
            if (r == i) {
                *err = "marker delta: a marker cannot be its own reference";
                return false;
            }
            m.delta = true;
            m.ref = r;
            return true;
        }
        if (verb == "noise") {
            if (!need(4)) return false;
            m.noise = a[3] != "0";
            return true;
        }
        *err = "marker: unknown verb \"" + verb + "\"";
        return false;
    }
    if (cmd == "peak" || cmd == "next" || cmd == "left" || cmd == "right") {
        return run_search(cmd == "peak" ? 0 : cmd == "next" ? 1 : cmd == "left" ? 2 : 3,
                          err);
    }
    if (cmd == "excursion" || cmd == "threshold") {
        if (!need(2)) return false;
        const double v = std::strtod(a[1].c_str(), nullptr);
        if (cmd == "excursion") markers_.peak.excursion_db = v;
        else markers_.peak.threshold_db = v;
        return true;
    }
    if (cmd == "center" || cmd == "span") {
        if (!need(2)) return false;
        double hz = 0;
        if (!parse_frequency(a[1], &hz)) {
            *err = cmd + ": cannot read \"" + a[1] + "\" as a frequency";
            return false;
        }
        const double c = (view_x0_ + view_x1_) * 0.5;
        const double s = view_x1_ - view_x0_;
        if (cmd == "center") {
            view_x0_ = hz - s * 0.5;
            view_x1_ = hz + s * 0.5;
        } else {
            if (!(hz > 0)) {
                *err = "span: must be positive";
                return false;
            }
            view_x0_ = c - hz * 0.5;
            view_x1_ = c + hz * 0.5;
        }
        view_dirty_ = true;
        if (auto_tune_ && !src_->set_span(view_x0_, view_x1_, err)) return false;
        return true;
    }
    // The segment width, from argv. `keep 54m` or `keep splice` / `keep blank`:
    // one argument, because the daemon takes the two fields independently and
    // scripting them together would restate a width nobody typed.
    // The manual RX gain: `gain 40`.
    if (cmd == "gain") {
        if (!need(2)) return false;
        char *end = nullptr;
        const double g = std::strtod(a[1].c_str(), &end);
        if (end == a[1].c_str() || *end) {
            *err = "gain takes a number of dB";
            return false;
        }
        return src_->set_gain(g, err);
    }
    // The RX gain table: `gaintable FILE`, `gaintable builtin` or `gaintable stock`.
    if (cmd == "gaintable") {
        if (!need(2)) return false;
        if (a[1] == "stock") return src_->set_gain_table("", "", err);
        if (a[1] == "builtin") return load_builtin_gain_table(err);
        return load_gain_table(a[1], err);
    }
    if (cmd == "keep") {
        if (!need(2)) return false;
        if (a[1] == "splice" || a[1] == "blank") return src_->set_segmentation(0, a[1], err);
        double hz = 0;
        if (!parse_frequency(a[1], &hz)) {
            *err = "keep: cannot read \"" + a[1] +
                   "\" as a frequency (expected a width, splice, or blank)";
            return false;
        }
        return src_->set_segmentation(hz, "", err);
    }
    // §6's zero span, from argv. Goes through the same source methods the soft
    // keys call, which is the property that makes --do evidence about the
    // buttons rather than a second implementation of them.
    if (cmd == "tsa") {
        if (!need(2)) return false;
        const double nan = nan_();
        if (a[1] == "off") return src_->set_sweep_mode(err);
        if (a[1] == "thru") {
            if (a.size() > 2) tnorm_sweeps_ = std::max(1, std::atoi(a[2].c_str()));
            tnorm_.start_capture(size_t(tnorm_sweeps_));
            return true;
        }
        if (a[1] == "norm") {
            if (!need(3)) return false;
            tnorm_on_ = a[2] != "0";
            return true;
        }
        if (a[1] == "clear") {
            tnorm_.clear();
            return true;
        }
        if (a[1] == "dut") {
            if (!need(3)) return false;
            const int i = std::atoi(a[2].c_str());
            if (i < 0 || size_t(i) >= src_->sim_duts().size()) {
                *err = "tsa dut: this source has " + std::to_string(src_->sim_duts().size()) +
                       " simulated DUTs";
                return false;
            }
            src_->set_sim_dut(i);
            return true;
        }
        if (a[1] == "reflect") {
            if (!need(3)) return false;
            tnorm_reflect_ = a[2] != "0";
            tnorm_.clear();
            return true;
        }
        if (a[1] == "rl") {
            if (!need(3)) return false;
            tnorm_rl_ = a[2] != "0";
            return true;
        }
        if (a[1] == "floor") {
            if (a.size() > 2) floor_sweeps_ = std::max(1, std::atoi(a[2].c_str()));
            TrackRequest r;
            r.stim_off = 1;
            if (!src_->set_tracking(r, err)) return false;
            floor_phase_ = FloorPhase::Silencing;
            floor_skip_ = 1;
            return true;
        }
        if (a[1] == "map") {
            // tsa map HARMONIC [OFFSET] [inv]
            if (!need(3)) return false;
            TrackRequest r;
            r.harmonic = std::atoi(a[2].c_str());
            double off = 0;
            if (a.size() > 3 && !parse_frequency(a[3], &off)) {
                *err = "tsa map HARMONIC [OFFSET] [inv]";
                return false;
            }
            r.rx_offset_hz = off;
            r.rx_invert = (a.size() > 4 && a[4] == "inv") ? 1 : 0;
            return src_->set_tracking(r, err);
        }
        double att = nan, sp = nan;
        int tones = -1, coh = -1;
        size_t i = 1;
        if (a[1] == "on") {
            if (!cfg_.tracking && !src_->set_span(view_x0_, view_x1_, err)) return false;
            att = double(tsa_atten_in_);
            i = 2;
        }
        for (; i + 1 < a.size(); i += 2) {
            const double v = std::strtod(a[i + 1].c_str(), nullptr);
            if (a[i] == "atten") att = v;
            else if (a[i] == "spacing") {
                if (!parse_frequency(a[i + 1], &sp)) {
                    *err = "tsa: cannot read \"" + a[i + 1] + "\" as a frequency";
                    return false;
                }
            } else if (a[i] == "tones") tones = int(v);
            else if (a[i] == "coherent") coh = int(v);
            else {
                *err = "tsa: unknown option \"" + a[i] +
                       "\" (expected atten, spacing, tones or coherent)";
                return false;
            }
        }
        if (i < a.size()) {
            *err = "tsa: expected on, off, thru, norm, clear, dut, or knob/value pairs";
            return false;
        }
        return src_->set_tracking(att, sp, tones, coh, err);
    }
    if (cmd == "zspan") {
        if (!need(2)) return false;
        if (a[1] == "off") return src_->set_sweep_mode(err);

        double hz = 0;
        size_t n_fft = 0, n_pts = 0;
        double overlap = -1;
        if (a[1] != "on") {
            if (!parse_frequency(a[1], &hz)) {
                *err = "zspan: cannot read \"" + a[1] +
                       "\" as a frequency (expected on, off, or a centre)";
                return false;
            }
        } else {
            // "on" with no centre parks at the middle of the view, matching the
            // soft key and the daemon's own default.
            hz = (view_x0_ + view_x1_) * 0.5;
        }
        // Optional trailing knobs: zspan 1g fft 4096 points 1024 overlap 0.75
        for (size_t i = 2; i + 1 < a.size(); i += 2) {
            const double v = std::strtod(a[i + 1].c_str(), nullptr);
            if (a[i] == "fft") n_fft = size_t(v);
            else if (a[i] == "points") n_pts = size_t(v);
            else if (a[i] == "overlap") overlap = v;
            else {
                *err = "zspan: unknown option \"" + a[i] +
                       "\" (expected fft, points or overlap)";
                return false;
            }
        }
        return src_->set_zero_span(hz, n_fft, n_pts, overlap, err);
    }
    // §6's video trigger, through the same source method the soft keys call.
    //   trig free|rising|falling | trig level DB | trig hyst DB | trig pretrig F
    if (cmd == "trig") {
        if (!need(2)) return false;
        if (a[1] == "free" || a[1] == "free_run") return src_->set_trigger("free_run", nan_(), nan_(), nan_(), err);
        if (a[1] == "rising" || a[1] == "rise")
            return src_->set_trigger("video_rising", nan_(), nan_(), nan_(), err);
        if (a[1] == "falling" || a[1] == "fall")
            return src_->set_trigger("video_falling", nan_(), nan_(), nan_(), err);
        if (!need(3)) return false;
        const double v = std::strtod(a[2].c_str(), nullptr);
        if (a[1] == "level") return src_->set_trigger("", v, nan_(), nan_(), err);
        if (a[1] == "hyst") return src_->set_trigger("", nan_(), v, nan_(), err);
        if (a[1] == "pretrig") return src_->set_trigger("", nan_(), nan_(), v, err);
        *err = "trig: expected free, rising, falling, level DB, hyst DB or pretrig F";
        return false;
    }
    if (cmd == "ref") {
        if (!need(2)) return false;
        ref_db_ = float(std::strtod(a[1].c_str(), nullptr));
        amp_dirty_ = true;
        return true;
    }
    if (cmd == "pause") {
        if (!need(2)) return false;
        paused_ = a[1] != "0";
        return true;
    }
    if (cmd == "wait") {
        if (!need(2)) return false;
        cmd_wait_ = std::atoi(a[1].c_str());
        return true;
    }
    if (cmd == "about") {
        show_about_ = a.size() < 2 || a[1] != "0";
        return true;
    }
    if (cmd == "csv") {
        if (!need(2)) return false;
        return export_csv(a[1], err);
    }
    if (cmd == "png") {
        if (!need(2)) return false;
        want_png_ = a[1];
        return true;
    }
    if (cmd == "save") {
        if (!need(2)) return false;
        return save_state(a[1], err);
    }
    if (cmd == "load") {
        if (!need(2)) return false;
        return load_state(a[1], err);
    }

    *err = "unknown command \"" + cmd + "\"";
    return false;
}

// ---------------------------------------------------------------- about

#ifndef SWEEPSA_VERSION
#define SWEEPSA_VERSION "dev"
#endif

namespace {

const char *platform_name() {
#if defined(_WIN32)
    const char *os = "Windows";
#elif defined(__APPLE__)
    const char *os = "macOS";
#elif defined(__linux__)
    const char *os = "Linux";
#else
    const char *os = "unknown OS";
#endif
#if defined(__aarch64__) || defined(_M_ARM64)
    const char *arch = "arm64";
#elif defined(__x86_64__) || defined(_M_X64)
    const char *arch = "x86_64";
#else
    const char *arch = "";
#endif
    static char buf[48];
    std::snprintf(buf, sizeof buf, "%s %s", os, arch);
    return buf;
}

// What ships, and under which licence. Must agree with scripts/dist.py's
// THIRD-PARTY-NOTICES, which carries the full texts.
struct Component {
    const char *name, *where, *licence;
};
const Component kComponents[] = {
    {"Dear ImGui", "sweepui", "MIT"},
    {"ImPlot", "sweepui", "MIT"},
    {"GLFW", "sweepui", "zlib/libpng"},
    {"PFFFT", "sweepui, sweepd", "FFTPACK (BSD-like)"},
    {"libhackrf", "sweepui, loaded at run time", "BSD-3-Clause"},
    {"libusb", "libhackrf", "LGPL-2.1"},
    {"libiio v0.25", "sweepd", "LGPL-2.1"},
    {"glibc 2.27", "sweepd", "LGPL-2.1"},
    {"libstdc++ / libgcc", "sweepd", "GPL-3.0 + runtime exception"},
};

}  // namespace

void App::draw_about() {
    const char *title = "About sweepsa";
    if (show_about_ && !ImGui::IsPopupOpen(title)) ImGui::OpenPopup(title);
    const ImGuiViewport *vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(560, 0), ImGuiCond_Appearing);
    if (!ImGui::BeginPopupModal(title, &show_about_, ImGuiWindowFlags_NoSavedSettings))
        return;

    ImGui::SetWindowFontScale(1.6f);
    ImGui::TextUnformatted("sweepsa");
    ImGui::SetWindowFontScale(1.0f);
    ImGui::TextDisabled("%s  -  %s", SWEEPSA_VERSION, platform_name());
    ImGui::Spacing();
    ImGui::TextWrapped("A swept spectrum analyzer for software-defined radios. On AD936x "
                       "boards running Pluto-style firmware, sweepd hops the LO with "
                       "fastlock profiles on the board's own ARM; sweepui is the front "
                       "panel.");
    ImGui::Spacing();
    ImGui::TextLinkOpenURL("sweepsa.zxkmm.com", "https://sweepsa.zxkmm.com");
    ImGui::Spacing();
    ImGui::TextWrapped("Copyright (c) 2026 zxkmm. sweepui is free software under the "
                       "GNU AGPL-3.0, with no warranty; a commercial licence is also "
                       "available. sweepd is closed source. See LICENSING.md.");
    ImGui::TextLinkOpenURL("Source code", "https://github.com/zxkmm/sweepsa");
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.75f, 0.35f, 1.0f));
    ImGui::TextWrapped("Not a calibrated instrument: levels are dBFS unless an amplitude "
                       "correction is loaded.");
    ImGui::PopStyleColor();

    ImGui::Separator();
    ImGui::TextUnformatted("Third-party components");
    if (ImGui::BeginTable("##components", 3,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        for (const Component &c : kComponents) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(c.name);
            ImGui::TableNextColumn();
            ImGui::TextDisabled("%s", c.where);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(c.licence);
        }
        ImGui::EndTable();
    }
    ImGui::TextWrapped("Full licence texts are in THIRD-PARTY-NOTICES.txt. sweepd links "
                       "libiio and glibc statically; every release publishes a relink "
                       "kit so it can be rebuilt against modified versions of them.");

    ImGui::Spacing();
    if (ImGui::Button("close", ImVec2(120, 0)) || ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        show_about_ = false;
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
}

// ---------------------------------------------------------------- status bar

void App::draw_status_bar() {
    ImGui::Separator();

    if (cfg_.synthetic) {
        // Not decoration. Every number on this screen is invented, and a
        // screenshot of it must not be mistakable for a measurement.
        badge("SYNTHETIC", IM_COL32(230, 160, 60, 255),
              "no radio is attached; every value on screen is fabricated");
        ImGui::SameLine();
    } else if (!status_.connected) {
        // §10.4. A live source that has gone away leaves the last frame on
        // screen, and a spectrum that is not updating looks exactly like a
        // spectrum that is not changing. The badge is the difference.
        badge("NO LINK", IM_COL32(240, 90, 80, 255),
              "not connected to sweepd; the trace on screen is the last one that "
              "arrived and is not being updated");
        ImGui::SameLine();
    }
    // A normalized SNA trace is a ratio of two received levels; the front end
    // that UNCAL warns about has cancelled out of it, and SNA S21 says what it is.
    if (uncal() && !tnorm_applied_) {
        badge("UNCAL", IM_COL32(220, 200, 90, 255),
              corr_.loaded() && !corr_.absolute()
                  ? "the loaded correction is RELATIVE: the frequency response is "
                    "flattened but there is no reference level, so values are dBFS "
                    "with an arbitrary origin, not dBm (§7)"
                  : "no amplitude correction is loaded; values are dBFS, not dBm (§7)");
        ImGui::SameLine();
    }
    // §7's image check, while it is running. The second pass is taken on a
    // *deliberately wrong* LO grid, so for those seconds every artifact on
    // screen is in the wrong place -- and a screenshot taken then would be a
    // picture of that with nothing to say so.
    if (img_phase_ == ImgPhase::PassA || img_phase_ == ImgPhase::Shifting ||
        img_phase_ == ImgPhase::PassB || img_phase_ == ImgPhase::Restoring) {
        const bool phased = std::fabs(cfg_.lo_shift_hz) > 1.0;
        badge(phased ? "LO PHASED" : "IMG CHECK",
              phased ? IM_COL32(255, 120, 200, 255) : IM_COL32(180, 160, 220, 255),
              phased ? "the image check has phased the LO grid; images on screen are "
                       "displaced from where they normally sit"
                     : "an image check is running -- see the Image check group");
        ImGui::SameLine();
    }
    if (cfg_.gen.on && !cfg_.tracking) {
        // A transmitter is on. Its signal is on the trace if anything couples it
        // to the receiver, and that must never be mistaken for the air.
        badge("TX ON", IM_COL32(240, 110, 90, 255),
              "the signal generator is transmitting -- see the Generator group");
        ImGui::SameLine();
    }
    if (cfg_.tracking) {
        // What a level means has changed: a tone's received level, or a ratio
        // against a stored thru. Both look like a spectrum and neither is one.
        if (tnorm_applied_)
            badge("SNA S21", IM_COL32(120, 200, 255, 255),
                  "tracking generator, normalized: each point is |S21| in dB against "
                  "the stored thru");
        else
            badge("SNA RAW", IM_COL32(120, 160, 220, 255),
                  "tracking generator, NOT normalized: each point is the received level "
                  "of a stimulus tone, including the radio's own TX and RX response. "
                  "Store a thru to read |S21|.");
        ImGui::SameLine();
    }
    if (cfg_.iq_mode) {
        badge("RAW IQ", IM_COL32(120, 200, 230, 255),
              "the plot is the receiver's complex baseband against TIME -- I and Q in "
              "ADC full-scale units, not a spectrum and not calibrated");
        ImGui::SameLine();
        if (iqv_.have() && iqv_.cur().n_clipped) {
            badge("CLIPPED", IM_COL32(240, 90, 80, 255),
                  "input samples reached full scale in this burst: the waveform's shape "
                  "is the converter's, not the signal's");
            ImGui::SameLine();
        }
        if (iqv_.have() && iqv_.cur().trig_state == sweepd::proto::IqTrigState::AutoTimeout) {
            badge("AUTO", IM_COL32(230, 160, 60, 255),
                  "no edge within the auto time: this burst is untriggered");
            ImGui::SameLine();
        }
    }
    if (cfg_.zero_span()) {
        // The axis is not a frequency axis. That is the single most important
        // thing about this screen and the easiest to miss in a screenshot, so
        // it gets a badge rather than only an axis label.
        badge("ZERO SPAN", IM_COL32(150, 210, 150, 255),
              "the x-axis is TIME, not frequency; the LO is parked and the sweep "
              "is stopped");
        ImGui::SameLine();
    }
    // Receiver artifacts that look like signals: the two peak-level verdicts,
    // not the DC holes, which are on every sweep and are the strip's business.
    if (art_on_ && art_res_.valid && !cfg_.zero_span()) {
        size_t spur = 0, internal = 0;
        for (const ArtifactFinding &f : art_res_.findings) {
            if (f.flags & kArtInternal) ++internal;
            else if (f.flags & kArtLoSpur) ++spur;
        }
        if (spur + internal > 0) {
            char t[64];
            std::snprintf(t, sizeof t, "%zu SELF-SPUR%s", spur + internal,
                          spur + internal == 1 ? "" : "S");
            badge(t, IM_COL32(255, 180, 90, 255),
                  "peaks on screen that are likely the receiver itself (LO-relative or "
                  "present with the input terminated) -- see the Artifacts group");
            ImGui::SameLine();
        }
    }
    if (have_frame_ && cur_.n_overloaded_segments > 0) {
        badge("OVERLOAD", IM_COL32(240, 90, 80, 255),
              "the front end is near full scale; the spurs and levels here "
              "may be the receiver, not the air");
        ImGui::SameLine();
        if (!cfg_.zero_span()) {
            ImGui::Text("%u/%u seg", unsigned(cur_.n_overloaded_segments),
                        unsigned(cur_.n_segments));
            ImGui::SameLine();
        }
    }
    if (status_.waiting_for_trigger) {
        // A trigger that is not firing produces no frames, no errors and no
        // message -- which is exactly what a dead link looks like, and what a
        // quiet band looks like. This is the difference, and it is why the
        // daemon counts the misses rather than only the hits.
        badge("WAIT TRIG", IM_COL32(230, 160, 60, 255),
              "the trigger is armed and has not fired since the frame on screen; "
              "this trace is the last one that did, not the signal now");
        ImGui::SameLine();
    }
    if (paused_) {
        badge("PAUSED", IM_COL32(140, 170, 230, 255), "the display is frozen");
        ImGui::SameLine();
    }
    if (cfg_.reconfiguring) {
        // A full-span retune rebuilds 298 fastlock profiles and produces no
        // frames for ~1.3 s. Without this the trace simply stops, which is what
        // a hung radio also looks like.
        badge("RECONFIG", IM_COL32(140, 170, 230, 255),
              "the source is rebuilding its profile table; no sweeps are being "
              "produced until it finishes");
        ImGui::SameLine();
    }

    // The x range goes through fmt_x, so the bar cannot report seconds as
    // hertz. In zero span it also carries the resolution -- which is not dt --
    // and the duty cycle, the two numbers that decide whether the trace on
    // screen can answer the question the user is asking of it.
    if (cfg_.iq_mode) {
        ImGui::Text("| %s  |  %zu samples  |  fs %s (/%u)  |  window %s  |  %.1f bursts/s  |"
                    "  duty %.1f%%  |  trig %s",
                    format_frequency(double(cfg_.iq_center_hz), 9).c_str(), cfg_.iq_n_samples,
                    format_frequency(cfg_.iq_fs_hz, 5).c_str(), cfg_.iq_decim,
                    format_seconds(cfg_.iq_fs_hz > 0 ? double(cfg_.iq_n_samples) / cfg_.iq_fs_hz
                                                     : 0.0, 4).c_str(),
                    iqv_.bursts_per_s(), iqv_.duty() * 100.0,
                    sweepd::iq_trig_mode_name(cfg_.iq_trig.mode));
    } else if (cfg_.zero_span()) {
        ImGui::Text("| %s - %s  |  %zu pts  |  dt %s  |  res %s  |  RBW %s  |  %s / %s  |"
                    "  %.1f fr/s  |  duty %.1f%%",
                    fmt_x(view_x0_).c_str(), fmt_x(view_x1_).c_str(), cfg_.n_points,
                    format_seconds(double(cfg_.dt_ns) * 1e-9).c_str(),
                    format_seconds(cfg_.time_res_s).c_str(),
                    format_frequency(cfg_.rbw_hz, 2).c_str(),
                    sweepd::detector_name(cfg_.detector),
                    sweepd::window_name(cfg_.window), status_.sweeps_per_s,
                    status_.duty * 100.0);
    } else {
        ImGui::Text("| %s - %s  |  %zu pts  |  RBW %s  |  %s / %s  |  %.1f sw/s  |  %.1f ms",
                    fmt_x(view_x0_).c_str(), fmt_x(view_x1_).c_str(), cfg_.n_points,
                    format_frequency(cfg_.rbw_hz, 2).c_str(),
                    sweepd::detector_name(cfg_.detector),
                    sweepd::window_name(cfg_.window), status_.sweeps_per_s,
                    status_.last_sweep_ms);
    }

    // §6 lists gain in the status bar. Only from a source with a settable gain,
    // where the number means something; a synthetic 0 dB would not.
    if (cfg_.gain_max_db > cfg_.gain_min_db) {
        ImGui::SameLine();
        ImGui::Text("  |  gain %.0f dB", double(cfg_.gain_db));
    }

    ImGui::SameLine();
    const float fps = ImGui::GetIO().Framerate;
    ImGui::Text("  |  %.0f fps", double(fps));
    if (status_.missed) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.4f, 1.0f), "  |  %llu sweeps dropped",
                           (unsigned long long)status_.missed);
    }
    // Why there is no link, on the bar rather than behind a panel: "connection
    // refused" and "no route to host" send the user to different places.
    if (!cfg_.synthetic && !status_.connected && !status_.detail.empty()) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.45f, 1.0f), "  |  %s",
                           status_.detail.c_str());
    }
}

}  // namespace sweepui
