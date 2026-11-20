#include "artifacts.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>

namespace sweepui {
namespace {

// Same shape parameters as the image check's floor estimator (imagecheck.cpp),
// and for the same reason: a quarter of a segment keeps the median inside one
// segment, so fastlock's per-segment offset cancels.
constexpr double kFloorFractionOfSegment = 0.25;
constexpr size_t kFloorMinBins = 4;
constexpr size_t kFloorMaxBins = 96;

// Below this many output bins per segment a fold has too few offsets to say
// anything: at full span and 4096 points a segment is ~14 bins, which passes;
// at 1024 points it is ~3, and "offset" there means "which third of the
// segment", which is not a property of the synthesizer.
constexpr double kMinBinsPerSegment = 8.0;

bool finite(float v) { return std::isfinite(v); }

// The LO step, read off the plan rather than assumed: the ownership width of a
// middle segment is the step under either dc policy (segments.h). The ends are
// avoided because they own out to their kept edge.
double plan_step(const std::vector<SegmentBand> &segs) {
    if (segs.size() >= 3) {
        const SegmentBand &m = segs[segs.size() / 2];
        return m.own_hi_hz - m.own_lo_hz;
    }
    if (segs.size() == 2) return segs[1].lo_hz - segs[0].lo_hz;
    if (segs.size() == 1) return segs[0].keep_hi_hz - segs[0].keep_lo_hz;
    return 0;
}

size_t derive_halfwidth(size_t cfg_hw, double step_hz, double dx) {
    if (cfg_hw) return cfg_hw;
    double want = (step_hz > 0 ? step_hz / dx : 0) * kFloorFractionOfSegment * 0.5;
    if (!(want >= double(kFloorMinBins))) want = double(kFloorMinBins);
    if (want > double(kFloorMaxBins)) want = double(kFloorMaxBins);
    return size_t(std::llround(want));
}

std::vector<int> owners_of(const std::vector<SegmentBand> &segs, const TraceAxis &ax) {
    std::vector<int> v;
    if (segs.empty()) return v;
    v.resize(ax.n);
    for (size_t i = 0; i < ax.n; ++i) v[i] = segment_at(segs, ax.x_at(i));
    return v;
}

// A blanked span reaches into a bin if it overlaps any part of it -- an output
// bin straddling the hole's edge is part fill and reads hot for the same reason.
bool touches_blank(const std::vector<SegmentBand> &segs, double x, double dx) {
    return in_blanked_span(segs, x) || in_blanked_span(segs, x - 0.5 * dx) ||
           in_blanked_span(segs, x + 0.5 * dx);
}

float median_of(std::vector<float> *v) {
    const size_t mid = v->size() / 2;
    std::nth_element(v->begin(), v->begin() + mid, v->end());
    return (*v)[mid];
}

// Number of groups when sorted positions are split wherever the gap exceeds
// `gap`. Contiguous segments are one place, not several witnesses.
size_t count_clusters(std::vector<double> los, double gap) {
    if (los.empty()) return 0;
    std::sort(los.begin(), los.end());
    size_t n = 1;
    for (size_t i = 1; i < los.size(); ++i)
        if (los[i] - los[i - 1] > gap) ++n;
    return n;
}

}  // namespace

const char *artifact_tag(uint8_t f) {
    // Most specific first: a recurrence or a terminated-input match is a
    // statement about *this* response, the comb and the hole are statements
    // about where it sits.
    if (f & kArtInternal) return "internal?";
    if (f & kArtLoSpur) return "LO spur?";
    if (f & kArtDcHole) return "LO/DC";
    if (f & kArtLoComb) return "LO comb";
    return "";
}

const char *artifact_explain(uint8_t f) {
    if (f & kArtInternal)
        return "this response was also present with the RX input terminated, at about "
               "this level: it is most likely generated inside the receiver";
    if (f & kArtLoSpur)
        return "a peak at this offset from the segment LO recurs in segments scattered "
               "across the span -- the synthesizer, not the air";
    if (f & kArtDcHole)
        return "inside the blanked hole around a segment LO, where LO leakage lands; "
               "under splice the value is a neighbour's, taken far from its own LO";
    if (f & kArtLoComb)
        return "most segments stand above their floor at this same offset from their "
               "LO: the receiver's own shape around DC, not a signal";
    return "";
}

double poisson_tail(size_t k, double lambda) {
    if (k == 0) return 1.0;
    if (!(lambda > 0)) return 0.0;
    // 1 - P(X <= k-1), summed in the direction that does not cancel.
    double term = std::exp(-lambda), cdf = term;
    for (size_t i = 1; i < k; ++i) {
        term *= lambda / double(i);
        cdf += term;
    }
    const double t = 1.0 - cdf;
    return t < 0 ? 0 : t;
}

// ---------------------------------------------------------------- mark

ArtifactResult mark_artifacts(const ImagePass &live, const ArtifactBaseline *base,
                              double live_gain_db, const ArtifactConfig &cfg) {
    ArtifactResult r;
    const TraceAxis &ax = live.axis;
    if (!ax.valid()) {
        r.err = "no sweep yet";
        return r;
    }
    if (ax.time) {
        r.err = "zero span has one parked LO and no grid to fold; nothing here to mark";
        return r;
    }
    if (live.db.size() != ax.n) {
        r.err = "the averaged trace does not match its own axis";
        return r;
    }

    const size_t n = ax.n;
    const double dx = ax.dx;
    r.axis = ax;
    r.flags.assign(n, 0);

    const std::vector<SegmentBand> &segs = live.segments;
    r.segments_known = !segs.empty();
    const double step = plan_step(segs);
    const size_t hw = derive_halfwidth(cfg.floor_halfwidth, step, dx);
    const std::vector<int> own = owners_of(segs, ax);
    const std::vector<float> exc = local_excursion(live.db, own, hw);

    // ---- 1. DC holes, by geometry.
    if (r.segments_known) {
        for (size_t i = 0; i < n; ++i)
            if (touches_blank(segs, ax.x_at(i), dx)) {
                r.flags[i] |= kArtDcHole;
                ++r.n_dc;
            }
    }

    // ... and the leakage skirt beyond it, only where it stands out.
    if (r.segments_known && cfg.lo_guard_hz > 0) {
        for (size_t i = 0; i < n; ++i) {
            if ((r.flags[i] & kArtDcHole) || own.empty() || own[i] < 0) continue;
            if (!finite(exc[i]) || exc[i] < float(cfg.spur_db)) continue;
            if (std::fabs(ax.x_at(i) - segs[size_t(own[i])].lo_hz) > cfg.lo_guard_hz + 0.5 * dx)
                continue;
            r.flags[i] |= kArtDcHole;
            ++r.n_dc;
        }
    }

    // ---- 2 and 3 need the grid and enough resolution to fold on.
    const double bins_per_seg = step > 0 ? step / dx : 0;
    if (!r.segments_known) {
        r.fold_note = "the segment grid was not reported, so nothing could be folded on "
                      "the LO";
    } else if (bins_per_seg < kMinBinsPerSegment) {
        char m[200];
        std::snprintf(m, sizeof m,
                      "%.1f bins per segment is too coarse to tell one LO offset from "
                      "another; add points or narrow the span",
                      bins_per_seg);
        r.fold_note = m;
    } else {
        r.fold_ran = true;
    }

    // Signed offset bucket per bin, relative to the owning LO. Bucketed at the
    // output bin width: the LO is not on the bin grid, so the same RF offset
    // lands in adjacent buckets in different segments, and the tests allow one
    // bucket of slack for it.
    std::vector<long long> bucket(n, 0);
    long long kmin = 0, kmax = 0;
    if (r.fold_ran) {
        for (size_t i = 0; i < n; ++i) {
            if (own[i] < 0) continue;
            bucket[i] = std::llround((ax.x_at(i) - segs[size_t(own[i])].lo_hz) / dx);
            kmin = std::min(kmin, bucket[i]);
            kmax = std::max(kmax, bucket[i]);
        }
    }

    // ---- 2. the fold median.
    //
    // Not on `exc`. The local-floor median is a quarter of a segment wide, and
    // the receiver's shape around DC is about that wide too -- the measured hump
    // is +3 dB out to several MHz -- so a bin inside it sits on a floor that is
    // itself raised, and its excursion reads ~0. The fold instead works on each
    // bin's level relative to the median of its *whole* segment: that still
    // cancels fastlock's per-segment offset, and it keeps broad structure.
    std::vector<float> fold_med;  // indexed by bucket - kmin; NaN = not judged
    std::vector<float> norm(n, std::numeric_limits<float>::quiet_NaN());
    if (r.fold_ran) {
        std::vector<std::vector<float>> per_seg(segs.size());
        for (size_t i = 0; i < n; ++i)
            if (own[i] >= 0 && finite(live.db[i]) && !(r.flags[i] & kArtDcHole))
                per_seg[size_t(own[i])].push_back(live.db[i]);
        std::vector<float> seg_med(segs.size(), std::numeric_limits<float>::quiet_NaN());
        for (size_t s = 0; s < segs.size(); ++s)
            if (!per_seg[s].empty()) seg_med[s] = median_of(&per_seg[s]);
        for (size_t i = 0; i < n; ++i)
            if (own[i] >= 0 && finite(live.db[i])) norm[i] = live.db[i] - seg_med[size_t(own[i])];

        const size_t nb = size_t(kmax - kmin + 1);
        std::vector<std::vector<float>> col(nb);
        for (size_t i = 0; i < n; ++i) {
            if (!finite(norm[i]) || (r.flags[i] & kArtDcHole)) continue;
            col[size_t(bucket[i] - kmin)].push_back(norm[i]);
        }
        fold_med.assign(nb, std::numeric_limits<float>::quiet_NaN());
        for (size_t b = 0; b < nb; ++b)
            if (col[b].size() >= cfg.min_fold_segments) fold_med[b] = median_of(&col[b]);

        // Report runs of elevated offsets, one entry per run.
        for (size_t b = 0; b < nb;) {
            if (!(fold_med[b] >= float(cfg.comb_db))) { ++b; continue; }
            size_t e = b, pk = b;
            while (e < nb && fold_med[e] >= float(cfg.comb_db)) {
                if (fold_med[e] > fold_med[pk]) pk = e;
                ++e;
            }
            CombOffset c;
            c.offset_hz = double((long long)pk + kmin) * dx;
            c.width_hz = double(e - b) * dx;
            c.median_db = fold_med[pk];
            c.segments = col[pk].size();
            r.comb.push_back(c);
            b = e;
        }

        // A bin at an elevated offset is marked unless it stands far above the
        // comb itself: a strong carrier that happens to sit on the hump is
        // still a carrier, and the comb says nothing about the part of it that
        // is more than spur_db higher than the receiver's own shape there.
        for (size_t i = 0; i < n; ++i) {
            if (!finite(norm[i])) continue;
            const float m = fold_med[size_t(bucket[i] - kmin)];
            if (!(m >= float(cfg.comb_db))) continue;
            if (norm[i] <= 0.0f || norm[i] > m + float(cfg.spur_db)) continue;
            r.flags[i] |= kArtLoComb;
        }
    }

    // ---- 3. recurrence.
    struct Peak {
        size_t bin, lo_bin, hi_bin;
        int seg;
        long long abs_bucket;
    };
    std::vector<Peak> peaks;
    if (r.fold_ran) {
        auto candidate = [&](size_t i) {
            return own[i] >= 0 && finite(exc[i]) && exc[i] >= float(cfg.spur_db) &&
                   !(r.flags[i] & (kArtDcHole | kArtLoComb));
        };
        for (size_t i = 0; i < n;) {
            if (!candidate(i)) { ++i; continue; }
            size_t j = i, pk = i;
            while (j < n && candidate(j) && own[j] == own[i]) {
                if (exc[j] > exc[pk]) pk = j;
                ++j;
            }
            peaks.push_back({pk, i, j, own[pk], std::llabs(bucket[pk])});
            i = j;
        }

        // How many segments would show a peak inside a +/-1 bucket window of a
        // given |offset| by chance, given how many peaks there are. A segment
        // owns about step/dx signed buckets, i.e. step/(2dx) distinct |offset|
        // buckets, and the window is three of them.
        //
        // Two corrections, both found by a scene of 300 random carriers that
        // produced a false spur without them. The offsets inside lo_guard_hz
        // cannot hold a candidate (anything standing out there is LO/DC), so
        // chance peaks crowd into the rest and the density is taken over those
        // only. And every window is a separate test, so alpha is split across
        // them (Bonferroni) -- otherwise ~27 windows at 1e-3 each would make
        // one false spur per few dozen quiet spans a certainty.
        const double guard_buckets = cfg.lo_guard_hz > 0 ? cfg.lo_guard_hz / dx : 0.0;
        const double abs_buckets = std::max(1.0, bins_per_seg * 0.5 - guard_buckets);
        const double p_window = std::min(1.0, 3.0 / abs_buckets);
        const double alpha = cfg.recur_alpha / abs_buckets;

        std::vector<char> is_spur(peaks.size(), 0);
        std::vector<size_t> rec_seg(peaks.size(), 0), rec_cl(peaks.size(), 0);
        for (size_t p = 0; p < peaks.size(); ++p) {
            std::vector<int> seen;
            std::vector<double> los;
            size_t in_window = 0;
            for (const Peak &q : peaks) {
                if (std::llabs(q.abs_bucket - peaks[p].abs_bucket) > 1) continue;
                ++in_window;
                if (std::find(seen.begin(), seen.end(), q.seg) != seen.end()) continue;
                seen.push_back(q.seg);
                los.push_back(segs[size_t(q.seg)].lo_hz);
            }
            const size_t clusters = count_clusters(los, cfg.cluster_gap_hz);
            rec_seg[p] = seen.size();
            rec_cl[p] = clusters;
            // The raster guard: however many segments, one place is one witness.
            if (clusters < cfg.min_recur) continue;

            // The chance rate comes from the peaks *outside* this window: the
            // ones inside are the hypothesis under test, and counting them as
            // background is how ten genuine spurs would talk themselves out of
            // being significant. Tested on distinct segments, less the peak's
            // own: several ripples of one wide signal in one segment are one
            // coincidence, not several.
            const double background = double(peaks.size() - in_window);
            const double lambda = background * p_window;
            if (poisson_tail(seen.size() - 1, lambda) >= alpha) continue;
            is_spur[p] = 1;
        }

        for (size_t p = 0; p < peaks.size(); ++p) {
            if (!is_spur[p]) continue;
            for (size_t i = peaks[p].lo_bin; i < peaks[p].hi_bin; ++i) {
                r.flags[i] |= kArtLoSpur;
                ++r.n_spur;
            }
            ArtifactFinding f;
            f.bin = peaks[p].bin;
            f.x_hz = ax.x_at(f.bin);
            f.flags = r.flags[f.bin];
            f.excursion_db = exc[f.bin];
            f.lo_offset_hz = f.x_hz - segs[size_t(peaks[p].seg)].lo_hz;
            f.recur_segments = rec_seg[p];
            f.recur_clusters = rec_cl[p];
            f.width_bins = peaks[p].hi_bin - peaks[p].lo_bin;
            r.findings.push_back(f);
        }
    }

    // ---- 4. baseline.
    if (base && base->valid()) {
        const TraceAxis &bx = base->pass.axis;
        if (bx.time) {
            r.baseline_note = "the baseline is a zero-span capture and cannot be compared "
                              "with a sweep";
        } else {
            r.baseline_used = true;
            const size_t bhw = derive_halfwidth(cfg.floor_halfwidth,
                                                plan_step(base->pass.segments), bx.dx);
            const std::vector<float> bexc = local_excursion(
                base->pass.db, owners_of(base->pass.segments, bx), bhw);

            const bool same_gain = std::fabs(base->gain_db - live_gain_db) < 0.05;
            const bool same_grid = std::fabs(base->seg_first_lo_hz -
                                             (segs.empty() ? 0 : segs.front().lo_hz)) < 1 &&
                                   std::fabs(base->seg_step_hz - step) < 1;
            if (!same_gain) {
                char m[200];
                std::snprintf(m, sizeof m,
                              "baseline taken at %.1f dB gain, live is %.1f dB: matched by "
                              "position only, levels not compared",
                              base->gain_db, live_gain_db);
                r.baseline_note = m;
            }
            if (!same_grid && base->seg_step_hz > 0 && !segs.empty()) {
                if (!r.baseline_note.empty()) r.baseline_note += ". ";
                r.baseline_note += "baseline was on a different LO grid, so its "
                                   "LO-relative responses are not where the live ones are";
            }

            // Half a bin of each grid either side: their bin centres need not
            // line up, and a max over the overlap errs toward matching, which
            // here means toward saying "internal" -- tolerable only because the
            // level test below still has to pass.
            const double reach = 0.5 * (dx + bx.dx);
            for (size_t i = 0; i < n; ++i) {
                if (!finite(exc[i]) || exc[i] < float(cfg.base_db * 0.5)) continue;
                const double x = ax.x_at(i);
                long long j0 = (long long)std::floor((x - reach - bx.x0) / bx.dx);
                long long j1 = (long long)std::floor((x + reach - bx.x0) / bx.dx);
                if (j1 < 0 || j0 >= (long long)bx.n) continue;
                j0 = std::max(0LL, j0);
                j1 = std::min((long long)bx.n - 1, j1);
                float best = std::numeric_limits<float>::quiet_NaN(), lvl = best;
                for (long long j = j0; j <= j1; ++j) {
                    const float v = bexc[size_t(j)];
                    if (finite(v) && (!finite(best) || v > best)) {
                        best = v;
                        lvl = base->pass.db[size_t(j)];
                    }
                }
                if (!(best >= float(cfg.base_db))) continue;
                if (same_gain && live.db[i] > lvl + float(cfg.base_margin_db)) continue;
                r.flags[i] |= kArtInternal;
                ++r.n_internal;
            }

            // One finding per run of internal bins that is not already a spur.
            for (size_t i = 0; i < n;) {
                if (!(r.flags[i] & kArtInternal) || (r.flags[i] & kArtLoSpur)) {
                    ++i;
                    continue;
                }
                size_t j = i, pk = i;
                while (j < n && (r.flags[j] & kArtInternal) && !(r.flags[j] & kArtLoSpur)) {
                    if (exc[j] > exc[pk]) pk = j;
                    ++j;
                }
                ArtifactFinding f;
                f.bin = pk;
                f.x_hz = ax.x_at(pk);
                f.flags = r.flags[pk];
                f.excursion_db = exc[pk];
                if (!own.empty() && own[pk] >= 0)
                    f.lo_offset_hz = f.x_hz - segs[size_t(own[pk])].lo_hz;
                f.width_bins = j - i;
                r.findings.push_back(f);
                i = j;
            }
        }
    }

    for (size_t i = 0; i < n; ++i)
        if (r.flags[i] & kArtLoComb) ++r.n_comb;

    // What nothing explained: local maxima over spur_db with no flag nearby.
    for (size_t i = 0; i < n; ++i) {
        if (!finite(exc[i]) || exc[i] < float(cfg.spur_db)) continue;
        bool peak = true, flagged = false;
        for (size_t j = i > 2 ? i - 2 : 0; j < std::min(n, i + 3); ++j) {
            if (finite(exc[j]) && exc[j] > exc[i]) peak = false;
            if (r.flags[j]) flagged = true;
        }
        if (!peak || flagged) continue;
        ArtifactFinding f;
        f.bin = i;
        f.x_hz = ax.x_at(i);
        f.excursion_db = exc[i];
        if (!own.empty() && own[i] >= 0) f.lo_offset_hz = f.x_hz - segs[size_t(own[i])].lo_hz;
        r.unmarked.push_back(f);
    }
    std::sort(r.unmarked.begin(), r.unmarked.end(),
              [](const ArtifactFinding &a, const ArtifactFinding &b) {
                  return a.excursion_db > b.excursion_db;
              });
    if (r.unmarked.size() > cfg.max_findings) r.unmarked.resize(cfg.max_findings);

    std::sort(r.findings.begin(), r.findings.end(),
              [](const ArtifactFinding &a, const ArtifactFinding &b) {
                  return a.excursion_db > b.excursion_db;
              });
    if (r.findings.size() > cfg.max_findings) {
        r.dropped = r.findings.size() - cfg.max_findings;
        r.findings.resize(cfg.max_findings);
    }

    std::string c = "Marks are suspicions, not removals: a real signal that sits exactly "
                    "where the receiver has its own response is marked with it.";
    if (!r.baseline_used)
        c += " No terminated-input baseline is loaded, so spurs at a fixed RF "
             "frequency (clock harmonics) are not marked at all.";
    r.caveat = c;
    r.valid = true;
    return r;
}

// ---------------------------------------------------------------- file

bool save_baseline(const std::string &path, const ArtifactBaseline &b, std::string *err) {
    if (!b.valid()) {
        *err = "no baseline to save";
        return false;
    }
    std::FILE *f = std::fopen(path.c_str(), "w");
    if (!f) {
        *err = path + ": " + std::strerror(errno);
        return false;
    }
    const TraceAxis &ax = b.pass.axis;
    std::fprintf(f, "# sweepsa-baseline: terminated RX input\n");
    std::fprintf(f, "# gain_db %.3f\n", b.gain_db);
    std::fprintf(f, "# sweeps %zu\n", b.pass.sweeps);
    std::fprintf(f, "# seg_first_lo_hz %.3f\n", b.seg_first_lo_hz);
    std::fprintf(f, "# seg_step_hz %.3f\n", b.seg_step_hz);
    std::fprintf(f, "# x0_hz %.6f\n", ax.x0);
    std::fprintf(f, "# dx_hz %.9f\n", ax.dx);
    std::fprintf(f, "# rbw_hz %.3f\n", ax.rbw_hz);
    std::fprintf(f, "freq_hz,db\n");
    for (size_t i = 0; i < ax.n; ++i) {
        if (finite(b.pass.db[i]))
            std::fprintf(f, "%.3f,%.3f\n", ax.x_at(i), double(b.pass.db[i]));
        else
            std::fprintf(f, "%.3f,nan\n", ax.x_at(i));
    }
    const bool ok = std::ferror(f) == 0;
    if (std::fclose(f) != 0 || !ok) {
        *err = path + ": write failed";
        return false;
    }
    return true;
}

bool load_baseline(const std::string &path, ArtifactBaseline *out, std::string *err) {
    std::ifstream in(path);
    if (!in) {
        *err = path + ": " + std::strerror(errno);
        return false;
    }
    ArtifactBaseline b;
    bool magic = false, have_x0 = false, have_dx = false;
    std::vector<float> db;
    std::string line;
    size_t lineno = 0;
    while (std::getline(in, line)) {
        ++lineno;
        if (line.empty()) continue;
        if (line[0] == '#') {
            if (line.find("sweepsa-baseline") != std::string::npos) magic = true;
            std::istringstream ss(line.substr(1));
            std::string key;
            double v = 0;
            if (!(ss >> key >> v)) continue;
            if (key == "gain_db") b.gain_db = v;
            else if (key == "sweeps") b.pass.sweeps = size_t(v);
            else if (key == "seg_first_lo_hz") b.seg_first_lo_hz = v;
            else if (key == "seg_step_hz") b.seg_step_hz = v;
            else if (key == "x0_hz") { b.pass.axis.x0 = v; have_x0 = true; }
            else if (key == "dx_hz") { b.pass.axis.dx = v; have_dx = true; }
            else if (key == "rbw_hz") b.pass.axis.rbw_hz = v;
            continue;
        }
        if (line.compare(0, 7, "freq_hz") == 0) continue;
        const size_t comma = line.find(',');
        if (comma == std::string::npos) {
            *err = path + ":" + std::to_string(lineno) + ": expected freq_hz,db";
            return false;
        }
        const std::string v = line.substr(comma + 1);
        if (v.compare(0, 3, "nan") == 0) {
            db.push_back(std::numeric_limits<float>::quiet_NaN());
            continue;
        }
        char *end = nullptr;
        const double d = std::strtod(v.c_str(), &end);
        if (end == v.c_str()) {
            *err = path + ":" + std::to_string(lineno) + ": bad level";
            return false;
        }
        db.push_back(float(d));
    }
    // The axis comes from the directives, not from the frequency column: a
    // column printed to the millihertz re-derives dx with rounding in it, and
    // bin i of a 32k-point trace would drift by a bin by the far end.
    if (!magic) {
        *err = path + ": not a sweepsa baseline (no '# sweepsa-baseline' line)";
        return false;
    }
    if (!have_x0 || !have_dx || !(b.pass.axis.dx > 0) || db.empty()) {
        *err = path + ": missing x0_hz/dx_hz or no data";
        return false;
    }
    b.pass.axis.n = db.size();
    b.pass.db = std::move(db);
    *out = std::move(b);
    return true;
}

}  // namespace sweepui
