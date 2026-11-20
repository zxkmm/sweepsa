#include "iqview.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>

#include "implot.h"

#include "freq.h"
#include "iqexport.h"

namespace sweepui {
namespace {

const ImVec4 kColI(0.35f, 0.75f, 1.00f, 1.0f);
const ImVec4 kColQ(1.00f, 0.62f, 0.25f, 1.0f);
const ImVec4 kColMag(0.88f, 0.88f, 0.88f, 1.0f);
const ImVec4 kColY2(0.55f, 0.90f, 0.45f, 1.0f);
const ImVec4 kColTrig(1.00f, 0.85f, 0.20f, 1.0f);
const ImVec4 kColA(0.95f, 0.45f, 0.95f, 1.0f);
const ImVec4 kColB(0.40f, 0.95f, 0.85f, 1.0f);
const ImVec4 kColWarn(1.0f, 0.75f, 0.35f, 1.0f);
const ImVec4 kColBad(1.0f, 0.45f, 0.40f, 1.0f);

const char *const kChanNames[4] = {"I", "Q", "|x|", "y2"};

int time_axis_fmt(double v, char *buf, int size, void *) {
    return std::snprintf(buf, size_t(size), "%s", format_seconds(v, 3).c_str());
}
int hz_axis_fmt(double v, char *buf, int size, void *) {
    return std::snprintf(buf, size_t(size), "%s", format_frequency(v, 4).c_str());
}

double db20(double v) { return v > 0 ? 20.0 * std::log10(v) : -300.0; }

void stat_row(const char *k, const char *fmt, ...) IM_FMTARGS(2);
void stat_row(const char *k, const char *fmt, ...) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextDisabled("%s", k);
    ImGui::TableNextColumn();
    va_list ap;
    va_start(ap, fmt);
    ImGui::TextV(fmt, ap);
    va_end(ap);
}

}  // namespace

// ---------------------------------------------------------------- data

void IqView::clear() {
    have_ = false;
    cur_ = sweepd::proto::IqMsg();
    for (auto &c : ch_) c.clear();
    ghosts_.clear();
    arrivals_.clear();
    rate_ = 0;
    fit_x_ = fit_y_ = fit_y2_ = fit_xy_ = true;
    ++nburst_;
}

void IqView::on_burst(sweepd::proto::IqMsg &&m, double host_now) {
    // The rate from the last few arrivals: enough to be steady, few enough to
    // follow a change of burst length within a second or two.
    arrivals_.push_back(host_now);
    while (arrivals_.size() > 8 || (arrivals_.size() > 1 && host_now - arrivals_.front() > 5.0))
        arrivals_.pop_front();
    rate_ = arrivals_.size() > 1
                ? double(arrivals_.size() - 1) / std::max(1e-6, arrivals_.back() - arrivals_.front())
                : 0.0;

    // Persistence: the burst on screen becomes a ghost before it is replaced.
    if (have_ && persist_n_ > 0 && cur_.n_samples() * size_t(persist_n_) <= kGhostBudget) {
        Ghost g;
        g.x0 = x_of(cur_, 0);
        g.dx = dx_of(cur_);
        for (int c = 0; c < 3; ++c) g.ch[c] = ch_[c];
        ghosts_.push_back(std::move(g));
    }
    while (ghosts_.size() > size_t(persist_n_)) ghosts_.pop_front();

    // A new geometry -- length, rate, triggered or not -- refits the time
    // axis. The same geometry keeps whatever zoom the user chose, which is
    // what makes zooming into a repetitive triggered waveform possible at all.
    const uint64_t key = (uint64_t(m.n_samples()) << 20) ^ (uint64_t(m.decim) << 8) ^
                         (m.trig_state == sweepd::proto::IqTrigState::Untriggered ? 0u : 1u) ^
                         (uint64_t(llround(m.adc_fs_hz)) << 32);
    if (key != axis_key_) {
        axis_key_ = key;
        fit_x_ = true;
        ghosts_.clear();
    }
    if (autoscale_) fit_y_ = fit_xy_ = true;

    cur_ = std::move(m);
    have_ = true;
    ++nburst_;
    derive();
    restat();
}

double IqView::x_of(const sweepd::proto::IqMsg &m, size_t i) const {
    const double t = m.t_at(i);
    return x_samples_ ? t * m.fs_hz : t;
}

double IqView::dx_of(const sweepd::proto::IqMsg &m) const {
    return x_samples_ ? 1.0 : 1.0 / m.fs_hz;
}

size_t IqView::index_at(double x) const {
    const size_t n = cur_.n_samples();
    if (n == 0) return 0;
    const double t = x_samples_ ? x / cur_.fs_hz : x;
    const double i = std::round((t - double(cur_.t0_ns) * 1e-9) * cur_.fs_hz);
    if (i <= 0) return 0;
    if (i >= double(n - 1)) return n - 1;
    return size_t(i);
}

void IqView::range(size_t *i0, size_t *i1) const {
    const size_t n = cur_.n_samples();
    *i0 = 0;
    *i1 = n;
    if (scope_cursors_ && tcur_ && n) {
        const size_t a = index_at(std::min(ca_, cb_)), b = index_at(std::max(ca_, cb_));
        *i0 = a;
        *i1 = std::max(a + 1, b + 1);
    }
}

void IqView::restat() {
    size_t i0, i1;
    range(&i0, &i1);
    stats_ = sweepd::iq_stats(cur_.iq.data(), i0, i1, cur_.fs_hz);
}

void IqView::derive() {
    const size_t n = cur_.n_samples();
    const float *x = cur_.iq.data();
    double mi = 0, mq = 0;
    if (remove_dc_ && n) {
        for (size_t i = 0; i < n; ++i) {
            mi += x[2 * i];
            mq += x[2 * i + 1];
        }
        mi /= double(n);
        mq /= double(n);
    }
    for (int c = 0; c < 3; ++c) ch_[c].resize(n);
    for (size_t i = 0; i < n; ++i) {
        const float a = float(x[2 * i] - mi), b = float(x[2 * i + 1] - mq);
        ch_[0][i] = a;
        ch_[1][i] = b;
        ch_[2][i] = std::sqrt(a * a + b * b);
    }
    ch_[3].clear();
    if (y2_ == Y2::Phase) {
        ch_[3].resize(n);
        for (size_t i = 0; i < n; ++i)
            ch_[3][i] = float(std::atan2(double(ch_[1][i]), double(ch_[0][i])) * 180.0 / M_PI);
    } else if (y2_ == Y2::Freq && n > 1) {
        // Instantaneous frequency: the phase step between neighbouring
        // samples. Noisy by nature -- the min/max band shows how noisy -- and
        // exactly the offset of a tone from the centre when one dominates.
        ch_[3].resize(n);
        const double k = cur_.fs_hz / (2.0 * M_PI);
        for (size_t i = 1; i < n; ++i) {
            const double ar = ch_[0][i], ai = ch_[1][i], br = ch_[0][i - 1], bi = ch_[1][i - 1];
            ch_[3][i] = float(std::atan2(ai * br - ar * bi, ar * br + ai * bi) * k);
        }
        ch_[3][0] = ch_[3][1];
    }
    dec_key_seq_ = ~0ull;
}

double IqView::duty() const {
    if (!have_ || !(cur_.fs_hz > 0)) return 0;
    return std::min(1.0, double(cur_.n_samples()) / cur_.fs_hz * rate_);
}

// ---------------------------------------------------------------- time plot

bool IqView::draw_time(const ImVec2 &size, const SourceConfig &cfg, const char *why_empty,
                       double *new_level) {
    bool level_set = false;
    if (!ImPlot::BeginPlot("##iqtime", size, ImPlotFlags_NoTitle | ImPlotFlags_NoMouseText))
        return false;

    ImPlot::SetupAxes(x_samples_ ? "samples from the trigger edge" : "time", "amplitude, FS");
    if (!x_samples_) ImPlot::SetupAxisFormat(ImAxis_X1, time_axis_fmt);
    if (y2_ != Y2::None) {
        ImPlot::SetupAxis(ImAxis_Y2, y2_ == Y2::Phase ? "phase, deg" : "inst. frequency",
                          ImPlotAxisFlags_AuxDefault);
        if (y2_ == Y2::Freq) ImPlot::SetupAxisFormat(ImAxis_Y2, hz_axis_fmt);
    }
    ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);

    const size_t n = cur_.n_samples();
    if (have_ && n) {
        const double dx = dx_of(cur_);
        if (fit_x_) {
            ImPlot::SetupAxisLimits(ImAxis_X1, x_of(cur_, 0), x_of(cur_, n - 1) + dx,
                                    ImPlotCond_Always);
            fit_x_ = false;
        }
        if (fit_y_) {
            float m = 0;
            float lo = 0;
            for (int c = 0; c < 3; ++c) {
                if (!show_[c]) continue;
                for (float v : ch_[c]) {
                    m = std::max(m, std::fabs(v));
                    lo = std::min(lo, v);
                }
            }
            const double lim = std::max(1e-4, double(m) * 1.1);
            ImPlot::SetupAxisLimits(ImAxis_Y1, lo < 0 ? -lim : -lim * 0.05, lim,
                                    ImPlotCond_Always);
            fit_y_ = false;
        }
        if (fit_y2_ && y2_ != Y2::None) {
            const double l = y2_ == Y2::Phase ? 190.0 : cur_.fs_hz * 0.55;
            ImPlot::SetupAxisLimits(ImAxis_Y2, -l, l, ImPlotCond_Always);
            fit_y2_ = false;
        }
    }

    const ImPlotRect lim = ImPlot::GetPlotLimits();
    view_x0_ = lim.X.Min;
    view_x1_ = lim.X.Max;
    if (place_tcur_) {
        ca_ = view_x0_ + (view_x1_ - view_x0_) / 3.0;
        cb_ = view_x0_ + 2.0 * (view_x1_ - view_x0_) / 3.0;
        place_tcur_ = false;
        restat();
    }
    if (place_vcur_) {
        va_ = lim.Y.Min + (lim.Y.Max - lim.Y.Min) * 0.75;
        vb_ = lim.Y.Min + (lim.Y.Max - lim.Y.Min) * 0.25;
        place_vcur_ = false;
    }

    if (have_ && n) {
        const ImVec2 psz = ImPlot::GetPlotSize();
        const size_t cols = size_t(std::max(64.0f, psz.x));
        const double dx = dx_of(cur_);
        const int opts = (remove_dc_ ? 1 : 0) | (x_samples_ ? 2 : 0) | (int(y2_) << 2);
        if (dec_key_seq_ != nburst_ || dec_key_x0_ != lim.X.Min || dec_key_x1_ != lim.X.Max ||
            dec_key_cols_ != cols || dec_key_opts_ != opts) {
            const double x0 = x_of(cur_, 0) - 0.5 * dx;
            for (int c = 0; c < 4; ++c) {
                dec_[c].clear();
                if (ch_[c].size() == n)
                    decimate_minmax(ch_[c].data(), n, x0, dx, lim.X.Min, lim.X.Max, cols, &dec_[c]);
            }
            ghost_dec_.assign(ghosts_.size(), DecimatedTrace());
            dec_key_seq_ = nburst_;
            dec_key_x0_ = lim.X.Min;
            dec_key_x1_ = lim.X.Max;
            dec_key_cols_ = cols;
            dec_key_opts_ = opts;
        }

        // Ghosts first, so the live burst draws over them. One channel each --
        // the first shown -- because three faded families of lines overlapping
        // is noise, not persistence.
        int gc = -1;
        for (int c = 0; c < 3 && gc < 0; ++c)
            if (show_[c]) gc = c;
        if (gc >= 0) {
            const ImVec4 base = gc == 0 ? kColI : gc == 1 ? kColQ : kColMag;
            for (size_t k = 0; k < ghosts_.size(); ++k) {
                const Ghost &g = ghosts_[k];
                if (ghost_dec_[k].size() == 0)
                    decimate_minmax(g.ch[gc].data(), g.ch[gc].size(), g.x0 - 0.5 * g.dx, g.dx,
                                    lim.X.Min, lim.X.Max, cols, &ghost_dec_[k]);
                const DecimatedTrace &d = ghost_dec_[k];
                if (!d.size()) continue;
                ImPlotSpec sp;
                ImVec4 col = base;
                col.w = 0.10f + 0.35f * float(k + 1) / float(ghosts_.size() + 1);
                sp.LineColor = col;
                sp.FillColor = col;
                sp.FillAlpha = 0.5f;
                sp.Flags = ImPlotItemFlags_NoLegend;
                ImPlot::PlotShaded("##ghost", d.x.data(), d.lo.data(), d.hi.data(), int(d.size()), sp);
            }
        }

        // Visible samples against pixels: under about one sample per four
        // pixels the samples themselves are drawn, so a zoomed-in waveform
        // reads as the discrete thing it is rather than as a smooth curve the
        // receiver never produced.
        const double vis = (lim.X.Max - lim.X.Min) / dx;
        const bool dots = vis < double(cols) / 4.0;
        for (int c = 0; c < 4; ++c) {
            if (c < 3 && !show_[c]) continue;
            if (c == 3 && y2_ == Y2::None) continue;
            const DecimatedTrace &d = dec_[c];
            if (!d.size()) continue;
            const ImVec4 col = c == 0 ? kColI : c == 1 ? kColQ : c == 2 ? kColMag : kColY2;
            const char *name = c < 3 ? kChanNames[c] : (y2_ == Y2::Phase ? "phase" : "freq");
            if (c == 3) ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
            const int m = int(d.size());
            const bool banded = vis > double(m) * 1.01;
            if (banded) {
                // Several samples per column: the envelope between them, and
                // its edges as lines, so a waveform too fast to resolve reads
                // as a band of the right height rather than as a single trace.
                ImPlotSpec band;
                band.FillColor = col;
                band.FillAlpha = 0.30f;
                ImPlot::PlotShaded(name, d.x.data(), d.lo.data(), d.hi.data(), m, band);
            }
            ImPlotSpec line;
            line.LineColor = col;
            line.LineWeight = 1.2f;
            if (dots) {
                line.Marker = ImPlotMarker_Circle;
                line.MarkerSize = 2.5f;
                line.MarkerFillColor = col;
            }
            ImPlot::PlotLine(name, d.x.data(), d.hi.data(), m, line);
            if (banded) ImPlot::PlotLine(name, d.x.data(), d.lo.data(), m, line);
            if (c == 3) ImPlot::SetAxes(ImAxis_X1, ImAxis_Y1);
        }

        // The edge. t = 0 by construction, interpolated to a fraction of a
        // sample (iq.h), which is why a repetitive waveform stands still.
        if (cur_.trig_state == sweepd::proto::IqTrigState::Triggered) {
            const double x0 = 0.0;
            ImPlotSpec tl;
            tl.LineColor = ImVec4(kColTrig.x, kColTrig.y, kColTrig.z, 0.55f);
            tl.Flags = ImPlotItemFlags_NoLegend;
            ImPlot::PlotInfLines("##trigx", &x0, 1, tl);
            ImPlot::TagX(0.0, kColTrig, "T");
        }
    } else {
        ImPlot::Annotation((lim.X.Min + lim.X.Max) * 0.5, (lim.Y.Min + lim.Y.Max) * 0.5,
                           ImVec4(0.7f, 0.7f, 0.7f, 1.0f), ImVec2(0, 0), false, "%s", why_empty);
    }

    // The trigger level, draggable, on the axis it applies to. Under "remove
    // DC" the I/Q traces are drawn shifted, so the line is too -- the level
    // the radio compares against is the raw one, and a line that did not move
    // with the trace would sit at a level the trace does not cross.
    if (cfg.iq_trig.mode != sweepd::IqTrigMode::None) {
        double shift = 0;
        if (remove_dc_ && have_) {
            if (cfg.iq_trig.source == sweepd::IqTrigSource::I) shift = stats_.dc_i;
            if (cfg.iq_trig.source == sweepd::IqTrigSource::Q) shift = stats_.dc_q;
        }
        double lv = (level_drag_ ? level_drag_v_ : cfg.iq_trig.level) - shift;
        bool held = false;
        ImPlot::DragLineY(1, &lv, kColTrig, 1.5f, ImPlotDragToolFlags_None, nullptr, nullptr, &held);
        const double raw = lv + shift;
        if (held) {
            level_drag_ = true;
            level_drag_v_ = raw;
        } else if (level_drag_) {
            level_drag_ = false;
            *new_level = level_drag_v_;
            level_set = true;
        }
        ImPlot::TagY(lv, kColTrig, "T %s", sweepd::iq_trig_source_name(cfg.iq_trig.source));
        // Where it arms: the hysteresis depth on the far side of the level.
        const double arm = lv + (cfg.iq_trig.rising ? -cfg.iq_trig.hyst : cfg.iq_trig.hyst);
        if (cfg.iq_trig.hyst > 0) {
            ImPlotSpec al;
            al.LineColor = ImVec4(kColTrig.x, kColTrig.y, kColTrig.z, 0.25f);
            al.Flags = ImPlotItemFlags_NoLegend | ImPlotInfLinesFlags_Horizontal;
            ImPlot::PlotInfLines("##arm", &arm, 1, al);
        }
    }

    if (tcur_) {
        bool moved = ImPlot::DragLineX(10, &ca_, kColA, 1.2f);
        moved |= ImPlot::DragLineX(11, &cb_, kColB, 1.2f);
        ImPlot::TagX(ca_, kColA, "A");
        ImPlot::TagX(cb_, kColB, "B");
        if (moved && scope_cursors_) restat();
    }
    if (vcur_) {
        ImPlot::DragLineY(12, &va_, kColA, 1.0f);
        ImPlot::DragLineY(13, &vb_, kColB, 1.0f);
        ImPlot::TagY(va_, kColA, "a");
        ImPlot::TagY(vb_, kColB, "b");
    }

    ImPlot::EndPlot();
    return level_set;
}

// ---------------------------------------------------------------- lower pane

void IqView::draw_lower(const ImVec2 &size, const SourceConfig &cfg) {
    const float xyw = show_xy_ ? std::min(size.y, size.x * 0.42f) : 0.0f;
    size_t i0 = 0, i1 = 0;
    range(&i0, &i1);

    if (show_xy_) {
        ImGui::BeginChild("##iqxy", ImVec2(xyw, size.y), false, ImGuiWindowFlags_NoScrollbar);
        if (ImPlot::BeginPlot("##xy", ImVec2(-1, -1),
                              ImPlotFlags_NoTitle | ImPlotFlags_Equal | ImPlotFlags_NoLegend |
                                  ImPlotFlags_NoMouseText)) {
            ImPlot::SetupAxes("I, FS", "Q, FS");
            if (fit_xy_ && have_) {
                const double r = std::max(1e-4, double(stats_.peak_mag) * 1.15 +
                                                    (remove_dc_ ? 0.0 : 0.0));
                ImPlot::SetupAxesLimits(-r, r, -r, r, ImPlotCond_Always);
                fit_xy_ = false;
            }
            if (have_ && i1 > i0) {
                // Strided to a budget: a scatter of four million points costs a
                // whole frame and draws nothing a pixel can resolve.
                const size_t span = i1 - i0;
                const size_t stride = std::max<size_t>(1, span / 20000);
                std::vector<double> xs, ys;
                xs.reserve(span / stride + 1);
                ys.reserve(span / stride + 1);
                for (size_t i = i0; i < i1; i += stride) {
                    xs.push_back(ch_[0][i]);
                    ys.push_back(ch_[1][i]);
                }
                ImPlotSpec sp;
                sp.Marker = ImPlotMarker_Circle;
                sp.MarkerSize = 1.5f;
                sp.MarkerFillColor = ImVec4(kColI.x, kColI.y, kColI.z, 0.45f);
                sp.MarkerLineColor = ImVec4(kColI.x, kColI.y, kColI.z, 0.45f);
                ImPlot::PlotScatter("##iq", xs.data(), ys.data(), int(xs.size()), sp);

                // The DC point: where the LO leakage puts the origin. Shown
                // whether or not it was removed from the display.
                const double dcx = remove_dc_ ? 0.0 : stats_.dc_i;
                const double dcy = remove_dc_ ? 0.0 : stats_.dc_q;
                ImPlotSpec dp;
                dp.Marker = ImPlotMarker_Cross;
                dp.MarkerSize = 6.0f;
                dp.MarkerLineColor = kColWarn;
                dp.LineWeight = 2.0f;
                ImPlot::PlotScatter("##dc", &dcx, &dcy, 1, dp);
            }
            // Full scale: the ADC's edge. Anything that reaches the circle's
            // corners has clipped.
            static double cx[181], cy[181];
            static bool init = false;
            if (!init) {
                for (int k = 0; k < 181; ++k) {
                    cx[k] = std::cos(2.0 * M_PI * k / 180.0);
                    cy[k] = std::sin(2.0 * M_PI * k / 180.0);
                }
                init = true;
            }
            ImPlotSpec fs;
            fs.LineColor = ImVec4(0.6f, 0.3f, 0.3f, 0.7f);
            ImPlot::PlotLine("##fs", cx, cy, 181, fs);
            ImPlot::EndPlot();
        }
        ImGui::EndChild();
        ImGui::SameLine();
    }

    ImGui::BeginChild("##iqstats", ImVec2(0, size.y), true);
    if (!have_) {
        ImGui::TextDisabled("no burst yet");
        ImGui::EndChild();
        return;
    }
    const sweepd::proto::IqMsg &m = cur_;
    const sweepd::IqStats &s = stats_;
    const double span_s = double(i1 - i0) / m.fs_hz;
    ImGui::Text("%s  %zu samples, %s  (%s)", scope_cursors_ && tcur_ ? "A..B" : "burst",
                i1 - i0, format_seconds(span_s, 4).c_str(),
                scope_cursors_ && tcur_ ? "between the cursors" : "the whole burst");

    if (ImGui::BeginTable("##iqst", 2, ImGuiTableFlags_SizingStretchProp)) {
        stat_row("rate", "%s  (ADC %s / %u)", format_frequency(m.fs_hz, 6).c_str(),
            format_frequency(m.adc_fs_hz, 6).c_str(), m.decim);
        stat_row("centre", "%s", format_frequency(double(m.center_hz), 9).c_str());
        stat_row("DC  I / Q", "%+.5f / %+.5f FS   (%.1f dBFS)", s.dc_i, s.dc_q,
            db20(std::hypot(s.dc_i, s.dc_q)));
        stat_row("RMS", "%.5f FS   %.2f dBFS", s.rms, s.power_dbfs);
        stat_row("RMS, DC removed", "%.5f FS   %.2f dBFS", s.rms_ac, db20(s.rms_ac));
        stat_row("peak |x|", "%.5f FS   %.2f dBFS", s.peak_mag, db20(s.peak_mag));
        stat_row("crest (AC)", "%.2f dB", s.crest_db);
        stat_row("Vpp  I / Q", "%.5f / %.5f FS", s.max_i - s.min_i, s.max_q - s.min_q);

        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextDisabled("dominant tone");
        ImGui::TableNextColumn();
        if (s.tone_coherence > 0.5) {
            ImGui::Text("%+.1f Hz  -> %s   (coherence %.2f)", s.tone_hz,
                        format_frequency(double(m.center_hz) + s.tone_hz, 9).c_str(),
                        s.tone_coherence);
        } else {
            ImGui::TextDisabled("none dominant (coherence %.2f)", s.tone_coherence);
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("from the mean phase step between samples: exact for one tone,\n"
                              "a power-weighted average for several. Coherence near 1 means\n"
                              "one tone dominates; near 0 means noise. The receiver's own\n"
                              "image of the tone pulls it slightly (iq.h). Not an FFT.");

        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextDisabled("I/Q mismatch");
        ImGui::TableNextColumn();
        if (s.circular_ok) {
            ImGui::Text("gain %+.2f dB, phase %+.2f deg  ->  image %.1f dB down",
                        s.gain_imb_db, s.phase_imb_deg, s.irr_db);
        } else if (s.too_few_cycles) {
            ImGui::TextDisabled("only %.1f turns of the tone in the window; needs 10+ "
                                "(longer burst, or decimate)", s.tone_cycles);
        } else {
            ImGui::TextDisabled("too little signal above the DC to estimate");
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("blind estimate from E[I^2], E[Q^2] and E[IQ] with DC removed.\n"
                              "Valid for a 'circular' signal: a tone away from 0 Hz, noise,\n"
                              "most digital modulation. A real-valued signal (AM at the\n"
                              "centre, a tone exactly on the LO) breaks the assumption and\n"
                              "this then describes the signal, not the receiver.\n"
                              "Fastlock profiles carry no quadrature calibration (§5).");

        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextDisabled("clipped");
        ImGui::TableNextColumn();
        if (m.n_clipped) {
            ImGui::TextColored(kColBad, "%u input samples at full scale -- the shape is the "
                                        "receiver's, not the signal's", m.n_clipped);
        } else {
            ImGui::Text("none");
        }
        stat_row("trigger", "%s%s, %u acquisition%s", sweepd::proto::iq_trig_state_name(m.trig_state),
            m.trig_state == sweepd::proto::IqTrigState::AutoTimeout ? " (no edge, sent anyway)" : "",
            m.n_acquisitions, m.n_acquisitions == 1 ? "" : "s");
        ImGui::EndTable();
    }

    if (tcur_ || vcur_) {
        ImGui::Separator();
        if (tcur_) {
            const size_t ia = index_at(ca_), ib = index_at(cb_);
            auto at = [&](const char *nm, double x, size_t i, const ImVec4 &col) {
                const double a = cur_.iq[2 * i], b = cur_.iq[2 * i + 1];
                ImGui::TextColored(col, "%s  %s   I %+.5f  Q %+.5f  |x| %.5f  %+.1f deg", nm,
                                   x_samples_ ? (std::to_string(llround(x)) + " smp").c_str()
                                              : format_seconds(x, 5).c_str(),
                                   a, b, std::hypot(a, b), std::atan2(b, a) * 180.0 / M_PI);
            };
            at("A", ca_, ia, kColA);
            at("B", cb_, ib, kColB);
            const double dt = (cb_ - ca_) / (x_samples_ ? m.fs_hz : 1.0);
            ImGui::Text("dt  %s   1/dt  %s   (%lld samples)", format_seconds(dt, 5).c_str(),
                        std::fabs(dt) > 0 ? format_frequency(1.0 / std::fabs(dt), 5).c_str() : "--",
                        (long long)ib - (long long)ia);
        }
        if (vcur_) {
            ImGui::Text("a %+.5f  b %+.5f  dV %.5f FS  (%.2f dB ratio)", va_, vb_, va_ - vb_,
                        (va_ != 0 && vb_ != 0) ? db20(std::fabs(va_ / vb_)) : 0.0);
        }
    }
    (void)cfg;
    ImGui::EndChild();
}

// ---------------------------------------------------------------- controls

void IqView::draw_display_group(bool synthetic) {
    if (!ImGui::CollapsingHeader("IQ display", ImGuiTreeNodeFlags_DefaultOpen)) return;

    bool rederive = false;
    ImGui::Checkbox("I", &show_[0]);
    ImGui::SameLine();
    ImGui::Checkbox("Q", &show_[1]);
    ImGui::SameLine();
    ImGui::Checkbox("|x|", &show_[2]);
    int y2 = int(y2_);
    ImGui::PushItemWidth(-90);
    if (ImGui::Combo("2nd axis", &y2, "none\0phase\0inst. frequency\0")) {
        y2_ = Y2(y2);
        fit_y2_ = true;
        rederive = true;
    }
    ImGui::PopItemWidth();
    if (ImGui::Checkbox("remove DC (display)", &remove_dc_)) {
        rederive = true;
        fit_y_ = fit_xy_ = true;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("subtracts the burst's mean from what is drawn. The DC readout\n"
                          "keeps the raw value: the LO leakage it hides belongs to the\n"
                          "receiver, and it is still in the samples you export.");
    if (ImGui::RadioButton("time", !x_samples_) && x_samples_) {
        x_samples_ = false;
        fit_x_ = true;
        tcur_ = false;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("samples", x_samples_) && !x_samples_) {
        x_samples_ = true;
        fit_x_ = true;
        tcur_ = false;
    }
    ImGui::Checkbox("autoscale", &autoscale_);
    ImGui::SameLine();
    if (ImGui::SmallButton("fit")) fit_x_ = fit_y_ = fit_y2_ = fit_xy_ = true;

    int pi = persist_n_ == 0 ? 0 : persist_n_ <= 4 ? 1 : 2;
    ImGui::PushItemWidth(-90);
    if (ImGui::Combo("persistence", &pi, "off\0" "4 bursts\0" "16 bursts\0")) {
        persist_n_ = pi == 0 ? 0 : pi == 1 ? 4 : 16;
        ghosts_.clear();
    }
    ImGui::PopItemWidth();
    if (persist_n_ > 0 && have_ && cur_.n_samples() * size_t(persist_n_) > kGhostBudget)
        ImGui::TextColored(kColWarn, "bursts this long are not kept (%zu-sample budget)",
                           kGhostBudget);
    ImGui::Checkbox("I/Q plane", &show_xy_);

    ImGui::Separator();
    if (ImGui::Checkbox("time cursors A/B", &tcur_) && tcur_) place_tcur_ = true;
    ImGui::SameLine();
    if (ImGui::Checkbox("level a/b", &vcur_) && vcur_) place_vcur_ = true;
    if (tcur_) {
        if (ImGui::RadioButton("measure burst", !scope_cursors_)) {
            scope_cursors_ = false;
            restat();
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("measure A..B", scope_cursors_)) {
            scope_cursors_ = true;
            restat();
        }
    }

    ImGui::Separator();
    ImGui::PushItemWidth(-70);
    ImGui::InputText("file", export_path_, sizeof export_path_);
    ImGui::PopItemWidth();
    if (ImGui::Button("export burst")) {
        std::string err, written;
        if (export_file(export_path_, synthetic, &err, &written)) {
            msg_ = "wrote " + written;
            msg_err_ = false;
        } else {
            msg_ = err;
            msg_err_ = true;
        }
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(".sigmf-meta  SigMF: cf32 data + metadata (rate, centre, trigger)\n"
                          ".cfile       complex float32 -- GNU Radio File Source, inspectrum\n"
                          ".sc16        int16 I/Q, ADC counts (2048 = full scale)\n"
                          ".csv         t, I, Q, |x|, phase\n"
                          "Between the cursors when 'measure A..B' is selected.");
    if (!msg_.empty())
        ImGui::TextColored(msg_err_ ? kColBad : ImVec4(0.6f, 0.85f, 0.6f, 1.0f), "%s",
                           msg_.c_str());

    if (rederive && have_) {
        derive();
        restat();
    }
}

bool IqView::export_file(const std::string &path, bool synthetic, std::string *err,
                         std::string *written) {
    if (!have_) {
        *err = "no burst to export";
        return false;
    }
    size_t i0, i1;
    range(&i0, &i1);
    return write_iq_file(path, cur_, i0, i1,
                         synthetic ? "SYNTHETIC -- fabricated, no radio attached" : "", err,
                         written);
}

bool IqView::command(const std::vector<std::string> &a, bool synthetic, std::string *err) {
    const std::string &c = a[0];
    auto arg = [&](size_t i) -> std::string { return i < a.size() ? a[i] : std::string(); };
    if (c == "iqch") {
        const std::string ch = arg(1);
        const bool on = arg(2) != "0";
        if (ch == "i") show_[0] = on;
        else if (ch == "q") show_[1] = on;
        else if (ch == "mag") show_[2] = on;
        else {
            *err = "iqch: i, q or mag, then 0 or 1";
            return false;
        }
        fit_y_ = true;
        return true;
    }
    if (c == "iqy2") {
        const std::string m = arg(1);
        if (m == "none") y2_ = Y2::None;
        else if (m == "phase") y2_ = Y2::Phase;
        else if (m == "freq") y2_ = Y2::Freq;
        else {
            *err = "iqy2: none, phase or freq";
            return false;
        }
        fit_y2_ = true;
    } else if (c == "iqdc") {
        remove_dc_ = arg(1) != "0";
        fit_y_ = fit_xy_ = true;
    } else if (c == "iqx") {
        x_samples_ = arg(1) == "samples";
        fit_x_ = true;
        tcur_ = false;
    } else if (c == "iqpersist") {
        persist_n_ = std::max(0, std::min(16, std::atoi(arg(1).c_str())));
        ghosts_.clear();
        return true;
    } else if (c == "iqxy") {
        show_xy_ = arg(1) != "0";
        return true;
    } else if (c == "iqcursor") {
        const std::string k = arg(1);
        if (k == "off") {
            tcur_ = vcur_ = false;
        } else if ((k == "t" || k == "v") && a.size() >= 4) {
            const double x = std::atof(a[2].c_str()), y = std::atof(a[3].c_str());
            if (k == "t") {
                tcur_ = true;
                ca_ = x;
                cb_ = y;
            } else {
                vcur_ = true;
                va_ = x;
                vb_ = y;
            }
        } else {
            *err = "iqcursor: t A B | v A B | off  (t in seconds, or samples under iqx samples)";
            return false;
        }
        restat();
        return true;
    } else if (c == "iqscope") {
        scope_cursors_ = arg(1) == "cursors";
        restat();
        return true;
    } else if (c == "iqexport") {
        if (a.size() < 2) {
            *err = "iqexport: needs a path";
            return false;
        }
        std::string w;
        return export_file(a[1], synthetic, err, &w);
    } else {
        *err = "unknown command " + c;
        return false;
    }
    if (have_) {
        derive();
        restat();
    }
    return true;
}

}  // namespace sweepui
