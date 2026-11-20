// A fourth FrameSource: an RTL-SDR dongle on the host's own USB, swept by
// retuning it from the host.
//
// CLAUDE.md is a brief for an AD936x instrument, so like hackrfsource.h this
// needs its presence in the tree justified rather than assumed. Three reasons,
// in order of how much they matter:
//
//   1. **It is the cheapest possible second reference.** §2 lists a LibreSDR as
//      the validation radio and PROJECT-STATE.md notes that plugging the AntSDR
//      in costs the host its network. A £20 dongle that needs neither is the
//      difference between checking the UI against real RF today and not.
//   2. **It is the first host-side source that sweeps sweepd's own geometry.**
//      The HackRF is swept by its firmware on an interleaved grid whose LOs are
//      not uniform, so §7's segment-boundary overlay and §7's image check both
//      report themselves unavailable on it (hackrfsource.h). This source calls
//      `plan_sweep` -- the daemon's planner, not a copy of it -- so the LO grid
//      *is* §5's grid, the overlay draws, the image check runs, and `keep_hz`
//      and the DC policy are live knobs. Those three features could previously
//      only be exercised against the synthetic source or the E310.
//   3. It reconfigures while running, which sweepd still refuses.
//
// It is emphatically not a replacement for sweepd, and on the one number §1 is
// about it is the worst radio in the tree by a wide margin -- see "the sweep is
// the settle" below. Its value is narrow spans, where it is genuinely pleasant,
// and being attachable to any laptop.
//
// ------------------------------------------------------- host-side retuning
//
// There is no firmware sweep mode here. The AD936x has fastlock and the HackRF
// has `hackrf_init_sweep`; an RTL-SDR has neither, so the host does it:
//
//   set_center_freq -> wait -> discard -> read -> FFT -> stitch -> next LO
//
// That loop is serial by nature, which is why this source has one worker thread
// where the HackRF source has a USB callback plus a worker. There is nothing to
// overlap: the radio cannot be producing usable samples while it is retuning,
// and an N=1024 transform costs a few tens of microseconds against a per-hop
// cost measured in milliseconds. A pipeline here would add a thread and a
// synchronisation bug to save ~1% of the sweep.
//
// ------------------------------------------------- the sweep is the settle
//
// The per-hop cost is dominated, overwhelmingly, by waiting for the tuner --
// and by how long you are *told* to wait rather than by how long it needs.
// `rtl_power`'s retune (rtl-sdr/src/rtl_power.c, `retune()`) is:
//
//     rtlsdr_set_center_freq(d, freq);
//     usleep(5000);                       /* settling */
//     rtlsdr_read_sync(d, dump, 4096);    /* flush the pipeline */
//
// Those two numbers -- 5000 us and 4096 bytes -- are this source's defaults,
// and the citation is that file and nothing else. **They are not measured.** No
// dongle was attached when this was written, so `docs/rtlsdr.md` records them as
// inherited rather than characterised, and both are knobs (`settle_us`,
// `dump_bytes`) for exactly the reason §8 forbids hardcoded timing constants.
//
// The arithmetic is worth seeing before anyone is disappointed by the sweep
// rate. At fs = 2.4 MSPS, keep = fs/2 and therefore step = 600 kHz, one hop
// costs 5 ms of settle plus 1.28 ms of samples (the 4096-byte dump plus a
// 2048-byte read) -- so ~6.3 ms, of which 80% is the wait. The segment counts
// are the planner's, from tests/rtlsdr_test.cpp:
//
//   | span                    | segments | predicted |
//   |-------------------------|----------|-----------|
//   | 88-108 MHz (FM)         |       35 |  ~0.22 s  |
//   | 1200-1300 MHz           |      168 |  ~1.1 s   |
//   | 24-1766 MHz (R820T full)|     2905 |  ~18 s    |
//
// (2.4 GHz is not in that table because an R820T cannot reach it. This radio
// stops at 1766 MHz, which is one of the things §7's capability banner is for.)
//
// So ~80% of a sweep is that `usleep`, and measuring the real settling time on
// a particular dongle is worth more than every other optimisation available
// here combined. That measurement is §4b.2 for this radio, `rtl_probe` reports
// the hops/sec to make it with, and until someone does it the honest thing is
// to inherit a conservative number and say that is what happened.
//
// ------------------------------------------------------ what the geometry is
//
// `plan_sweep` (src/sweepd/plan.h), with:
//
//   keep_hz  = fs/2 by default -- the kept band is the LO +/- fs/4.
//   blank_hz = 25 kHz, the hole punched around each LO for its own leakage.
//   dc       = SpliceOverlap, so the step is keep/2 and each hole is covered
//              from both sides.
//
// keep = fs/2 rather than the HackRF's 3fs/4 is a deliberate choice about where
// the *splice fill* comes from. Under SpliceOverlap the value shown at segment
// k's centre was measured by k's neighbour at exactly keep/2 from that
// neighbour's own LO -- the extreme edge of its kept band. PROJECT-STATE.md
// records what that costs on the E310 ("the splice fill is hot": a +4.5 dB comb
// at every segment centre, scaling with keep_hz). At keep = 3fs/4 the fill would
// be drawn from 3fs/8 = 900 kHz out, which at fs = 2.4 MSPS is sitting on the
// R820T's automatic IF filter corner, and the comb would come back as a scallop
// instead. At keep = fs/2 the fill comes from fs/4, half of Nyquist, well inside
// the filter. `keep_hz` is config and the trade is monotonic in both directions:
// wider is faster and dirtier at the seams.
//
// ------------------------------------------------------- what is not readable
//
// One honesty point specific to this chip. The cold build on the AD936x writes
// an LO and then reads back what the driver actually tuned to, and every bin-to-
// RF mapping uses the read-back (plan.h, `Segment::lo_hz`). There is no
// equivalent here: `rtlsdr_get_center_freq` returns librtlsdr's cached copy of
// what it was *asked* for, not what the PLL reached. So the trace is built on
// the requested LO, and any rounding the R820T's synthesizer does is a small
// frequency error this source cannot see and does not pretend to. It is bounded
// by the tuner's own step and is far below one output bin at any useful span,
// but it is an assumption rather than a measurement and it is listed as one.
//
// Nor are dropped samples visible. `rtlsdr_read_sync` does not report a USB
// underrun, and a chunk missing from the middle of a buffer is a discontinuity
// inside a transform -- broadband, and shaped exactly like signal. This is why
// the default sample rate is 2.4 MSPS and not the 3.2 MSPS the device accepts:
// above roughly 2.4 the dongle drops samples on most hosts, nothing here can
// tell, and the result is a slightly noisier trace rather than an error.
// `rtl_power` caps itself at 2.8 MSPS for the same reason.
//
// --------------------------------------------------------------- not calibrated
//
// Everything §1 and §7 say applies here more strongly than anywhere else in the
// tree: an 8-bit ADC, a tuner with no preselector, and a gain list whose steps
// are 1-3 dB wide and whose absolute accuracy is nobody's claim. The trace is
// dBFS and UNCAL is lit, exactly as with sweepd.

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "plan.h"
#include "source.h"

namespace sweepui {

// Which tuner is in front of the RTL2832. Mirrors librtlsdr's own enum rather
// than reusing it, so this header compiles with no librtlsdr present -- the
// same reason `rtlsdr_available()` exists.
enum class RtlTuner { Unknown, E4000, FC0012, FC0013, FC2580, R820T, R828D };

const char *rtl_tuner_name(RtlTuner t);

// The tuner's published frequency range, per librtlsdr's own tuner drivers.
//
// **Not measured**, and §7 asks for measured capability rather than datasheet
// capability, so this is the weakest thing in the file and it is labelled as
// such in the banner it feeds. It is still better than the alternative: the
// range genuinely differs by a factor of two across these parts, and refusing
// to tune where the tuner cannot is more useful than sweeping a band that
// returns noise. `RtlSdrOptions::tune_min_hz` / `tune_max_hz` override it, which
// is how a §4a sensitivity sweep on a specific dongle gets to narrow it.
//
// `direct_sampling` non-zero bypasses the tuner entirely, so the range becomes
// the RTL2832's own first Nyquist zone and has nothing to do with `t`.
//
// False when the tuner is unknown: guessing a range for a part we could not
// identify is the one option with no defence.
bool rtl_tuner_range(RtlTuner t, int direct_sampling, long long *lo_hz, long long *hi_hz);

// The range of a nextrtl board (FC0013 plus an onboard HF upconverter) when the
// loaded librtlsdr is the nextrtl fork, which upconverts below 30 MHz on its
// own. Unlike the table above, part of this was measured -- see docs/rtlsdr.md.
void nextrtl_range(long long *lo_hz, long long *hi_hz);

// Nearest entry in the tuner's own gain list, which is what librtlsdr will
// accept; `tenths` is `rtlsdr_get_tuner_gains`' output, in tenths of a dB.
//
// Snapping rather than rejecting, and then *reporting the snapped value*: the
// R820T's list is 29 unevenly spaced entries from 0 to 49.6 dB, nobody types
// those from memory, and a gain readout showing what was asked for rather than
// what was set is §7's failure in miniature. A request above the top of the
// list therefore means "maximum", which is how `--rtl-gain max` works.
bool rtl_snap_gain(const std::vector<int> &tenths, double want_db, double *got_db);

struct RtlSdrOptions {
    // Defaults are the FM broadcast band, not the full tuner range, and that is
    // a considered choice: a full-span default here would be an 18-second sweep
    // and would read as a hung program. §6's "full span" soft key still reaches
    // the whole range, having been told what it costs.
    long long f_start_hz = 88000000LL;
    long long f_stop_hz = 108000000LL;

    // 2.4 MSPS is the fastest rate this dongle sustains without dropping
    // samples on most hosts -- see "what is not readable". librtlsdr accepts
    // 225001-300000 and 900001-3200000 Hz and nothing between.
    double fs_hz = 2400000.0;

    // No block-format cap here, unlike the HackRF: a segment is read with as
    // many `rtlsdr_read_sync` calls as it takes, so the FFT size is free. It
    // buys RBW at no cost in sweep time worth measuring, because the settle
    // dominates -- N=8192 at 2.4 MSPS is 3.4 ms of samples against a 5 ms wait.
    size_t n_fft = 1024;

    // 0 derives keep from the achieved sample rate as fs/2. See the header for
    // why fs/2 and not more.
    long long keep_hz = 0;

    // Half-width of the hole punched around each LO. The R820T's centre spike
    // has not been measured here; 25 kHz is wide enough for the usual one and
    // is the knob to widen if a comb of residual spikes survives at the segment
    // centres.
    long long blank_hz = 25000;

    sweepd::DcPolicy dc = sweepd::DcPolicy::SpliceOverlap;

    // §7's image check. Non-zero phases the whole LO grid; `image_check_shift_hz`
    // is the value that wants. Genuinely available on this radio, because it
    // tunes in Hz rather than in the HackRF firmware's whole megahertz.
    long long lo_shift_hz = 0;

    size_t n_points = 0;             // 0 derives it from the achieved RBW
    size_t max_points = 65536;

    sweepd::Detector detector = sweepd::Detector::Peak;
    sweepd::Window window = sweepd::Window::Hann;

    // §5's "AGC off. Manual gain." -- both of them, because this device has two
    // and turning off only the obvious one leaves the other running.
    // `gain_db` is snapped to the tuner's list; `rtl_agc` is the RTL2832's own
    // digital AGC, which is a separate switch and is off.
    double gain_db = 30.0;
    bool rtl_agc = false;

    // Crystal error, in parts per million. Worth setting: an uncorrected dongle
    // is routinely tens of ppm out, which at 1.7 GHz is tens of kilohertz.
    int ppm = 0;

    // 0 off, 1 = I branch, 2 = Q branch. Bypasses the tuner and samples HF
    // directly, which is the only way this radio sees anything below 24 MHz. Q
    // is the branch wired on the common RTL-SDR Blog dongles. Untested here.
    int direct_sampling = 0;

    // E4000 only; librtlsdr refuses it on an R820T. Left as a knob because a
    // dongle with an E4000 genuinely wants it.
    bool offset_tuning = false;

    // Puts ~4.5 V on the antenna port to feed an inline LNA. Off by default and
    // worth being deliberate about: there is nothing between it and whatever is
    // connected.
    bool bias_tee = false;

    // Tuner IF filter. 0 asks librtlsdr for automatic, which tracks the sample
    // rate. This API version cannot read back what was applied, so the plan is
    // checked against `fs_hz` when this is 0 rather than against a number we
    // would be inventing.
    long long bw_hz = 0;

    // ------------------------------------------------- the two settle knobs
    //
    // Inherited from rtl_power, not measured. See the header: together they are
    // most of the sweep time, so they are the first thing to measure on a real
    // dongle and the first thing to cut.
    unsigned settle_us = 5000;       // rtl_power's usleep(5000)
    size_t dump_bytes = 4096;        // rtl_power's BUFFER_DUMP

    double overload_frac = 0.98;     // §5's near-full-scale threshold

    // Override `rtl_tuner_range`, so a measured range can replace a published
    // one. 0 means "use the tuner's".
    long long tune_min_hz = 0;
    long long tune_max_hz = 0;

    // Which device, when more than one is attached. `serial` wins over `index`.
    std::string serial;
    int index = -1;

    size_t frame_queue_depth = 4;
};

// Everything the source learned by asking the hardware, as against what the
// options asked for. §7's capability banner wants the former.
struct RtlSdrInfo {
    std::string name;                 // rtlsdr_get_device_name
    std::string manufacturer, product, serial;
    RtlTuner tuner = RtlTuner::Unknown;
    uint32_t rtl_xtal_hz = 0, tuner_xtal_hz = 0;

    // The tuner's whole gain list, in tenths of a dB, as reported. Carried
    // because it is the only honest description of what "gain" means on this
    // radio: not a range with a step, a list.
    std::vector<int> gains_tenth_db;

    double fs_achieved_hz = 0;        // rtlsdr_get_sample_rate, not the request
    double gain_achieved_db = 0;      // after snapping to the list above
    long long tune_min_hz = 0, tune_max_hz = 0;
    int direct_sampling = 0;

    // Non-zero on a nextrtl board running the fork's librtlsdr: the HF
    // upconverter LO it adds below 30 MHz. Zero for any other dongle, and for a
    // nextrtl on the stock library, which can't reach HF.
    uint32_t nextrtl_hf_lo_hz = 0;
    bool nextrtl_strings = false;     // EEPROM says nextrtl, whatever the library

    std::string gain_list() const;    // "0.0 0.9 1.4 ... 49.6 dB", for the banner
};

// Checks and rounds a configuration against what librtlsdr and this source
// allow, without touching a device, so a bad request is refused with a sentence
// rather than as a driver error code halfway into a sweep.
bool rtl_validate(RtlSdrOptions *o, std::string *err);

// The PlanConfig the options imply once the achieved sample rate and the tuner
// range are known. Split out from the worker so tests can check the geometry --
// which is where a mistake is invisible -- with no device and no thread.
sweepd::PlanConfig rtl_plan_config(const RtlSdrOptions &o, double fs_achieved_hz,
                                   long long tune_lo_hz, long long tune_hi_hz);

// Bytes to ask `rtlsdr_read_sync` for when `n_fft` complex pairs are wanted,
// rounded up to the USB bulk granularity librtlsdr expects.
size_t rtl_read_bytes(size_t n_fft);

class RtlSdrSource : public FrameSource {
public:
    explicit RtlSdrSource(const RtlSdrOptions &opt);
    ~RtlSdrSource() override;

    // False with a reason if no device could be opened or configured. The
    // object is inert afterwards and safe to destroy.
    bool open(std::string *err);

    const char *name() const override { return "rtlsdr"; }
    bool poll(sweepd::proto::TraceMsg *out) override;
    SourceConfig config() const override;
    SourceStatus status() const override;

    // Live, all of them: the worker abandons the sweep in progress,
    // reconfigures and replans. A span or segmentation change moves the output
    // grid, so the UI sees `reconfiguring` briefly -- much more briefly than
    // sweepd, which has a profile table to rebuild.
    bool set_span(double f_start_hz, double f_stop_hz, std::string *err) override;
    bool set_points(size_t n_points, std::string *err) override;
    bool set_detector(sweepd::Detector d, std::string *err) override;
    bool set_window(sweepd::Window w, std::string *err) override;

    // Live, and really the segment plan's: this is the first host-side source
    // where `keep` and the DC policy mean what they mean in §5, because the grid
    // comes from `plan_sweep` rather than from a firmware's own idea of a sweep.
    bool set_segmentation(double keep_hz, const std::string &dc_policy,
                          std::string *err) override;

    // §7's image check, likewise genuinely available -- the LO grid is ours and
    // it is phased in Hz.
    bool set_lo_shift(double hz, std::string *err) override;

    // Refused, with the reason and with what it would take. Zero span is a
    // clean addition here and not a hard one -- the read path is already a
    // parked-LO read loop and `sweepd::ZeroSpan` is written against an
    // interface, not against the AD936x -- but it is a second mode with its own
    // trigger accounting and it is not this change.
    bool set_zero_span(double center_hz, size_t n_fft, size_t n_points, double overlap,
                       std::string *err) override;
    bool set_trigger(const std::string &mode, double level_db, double hyst_db,
                     double pretrig, std::string *err) override;
    bool set_sweep_mode(std::string *err) override;

    const RtlSdrInfo &info() const { return info_; }

private:
    struct Impl;
    std::unique_ptr<Impl> p_;
    RtlSdrInfo info_;
};

// True when librtlsdr was compiled in. sweepui builds and runs without it; the
// --rtl option then fails with an explanation instead of not existing, so a
// build without the library is distinguishable from a typo.
bool rtlsdr_available();

// Names of the attached dongles, for `--rtl-list`. Empty without librtlsdr.
std::vector<std::string> rtlsdr_devices();

}  // namespace sweepui
