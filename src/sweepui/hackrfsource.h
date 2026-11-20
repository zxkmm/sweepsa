// A third FrameSource: a HackRF One on the host's own USB, swept by the
// firmware's sweep mode.
//
// CLAUDE.md is a brief for an AD936x instrument and this is not that radio, so
// what it is doing in the tree needs saying plainly. It is a *second reference*,
// in the same sense §2's LibreSDR is: a way to put a known-shaped spectrum in
// front of sweepui that is neither fabricated (the synthetic source) nor
// dependent on the E310 being plugged in -- which, per the AntSDR-plug-in note
// in PROJECT-STATE.md, costs the host its network. It is also the only source in
// the tree that can genuinely reconfigure while running, which is what §6's
// soft keys were written against and what sweepd still refuses.
//
// It is emphatically not a replacement for sweepd. §1's whole reason for
// existing is that a HackRF needs ~750 ms for a full sweep where fastlock needs
// milliseconds, and nothing here changes that -- a 70 MHz - 6 GHz sweep at
// 20 MSPS is 594 tunings and lands around 0.4-0.5 s. That number is the point of
// comparison, not a disappointment.
//
// ------------------------------------------------------------- firmware sweep
//
// The HackRF's analogue of fastlock is `hackrf_init_sweep` +
// `hackrf_start_rx_sweep`: the host hands the device a frequency range and the
// device retunes itself between blocks, so the per-hop USB round trip that
// makes `hackrf_set_freq` hopeless for sweeping never happens. Samples come back
// as 16384-byte blocks, each prefixed with a 10-byte header
// `0x7F 0x7F <uint64 LE frequency>` naming the tuning that block was captured
// at.
//
// The geometry, measured on the attached device rather than assumed (see
// docs/hackrf.md):
//
//   step_width = fs, offset = 3*fs/8, style = INTERLEAVED
//
// gives block headers at `a, a+fs/4, a+fs, a+5fs/4, a+2fs, ...` -- two blocks
// per tuning step, the second offset by a quarter of the sample rate. A block
// whose header reads F was captured with its LO at F + 3fs/8, and the two
// quarter-band slices either side of the LO's own leakage are kept:
//
//   shifted bins [N/8, 3N/8)  ->  [F,        F + fs/4)
//   shifted bins [5N/8, 7N/8) ->  [F + fs/2, F + 3fs/4)
//
// The discarded middle is +/- fs/8 around the LO -- DC, where direct conversion
// puts the LO leakage §5 blanks -- and the discarded outside is the analog
// filter's skirt. The interleaved partner block is what fills the middle back
// in, so the two together tile the range with no gaps and the LO spike of each
// is covered by the other. That is §5's splice, arrived at by a different
// route, and it is why the stitch rule in stitch.h is shared rather than
// reimplemented here.
//
// ------------------------------------------------------------ the discovery sweep
//
// The block layout above is measured, and this file does not then hardcode it.
// The first sweep after every (re)configuration is a *discovery* sweep: block
// headers are collected and nothing is transformed, and the output grid, the
// covered range and the block count are derived from what actually arrived.
// Streaming starts on the second sweep.
//
// It costs one sweep period at startup and it buys the thing §8 asks for --
// no hardcoded constant standing in for something the hardware could just be
// asked about. A firmware that interleaves differently, or a sample rate whose
// sub-step is not fs/4, produces a correct grid here instead of a plausible
// trace with everything half a band out.
//
// -------------------------------------------------------- no segment overlay
//
// SourceConfig's segment-plan fields are deliberately left zero, so §7's
// boundary overlay reports itself unavailable rather than drawing an invented
// grid (source.h says this is the supported state for a source whose geometry
// does not fit).
//
// The reason is specific and worth recording. segments.h reconstructs the plan
// from a *uniform* LO grid -- `first_lo + k*step` -- and the interleaved sweep's
// LOs are not uniform: at fs = 20 MHz they sit at 107.5, 112.5, 127.5, 132.5 MHz
// and so on, alternating 5 MHz and 15 MHz apart. The seams are uniform (every
// 5 MHz) and it is tempting to report those as the centres, but a "centre" in
// that overlay is where the LO leakage is, and the seams are precisely where it
// is not. Drawing them there would attribute the artifact to the wrong
// frequency, which is the §7 failure the overlay exists to prevent. Teaching
// SegmentPlanInfo about paired LOs is the fix; it is not this file's job.
//
// --------------------------------------------------------------- not calibrated
//
// Everything §1 and §7 say about the E310 applies here at least as strongly.
// There is no preselector, the gain stages are coarse (LNA in 8 dB steps), and
// the amplitude of one 5 MHz slice is not comparable to its neighbour's. The
// trace is dBFS and UNCAL is lit, exactly as with sweepd.

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

struct HackRfOptions {
    // The HackRF One's documented range. Not the extended 0-7250 MHz mode: the
    // spec'd range is what §7's capability banner should show, and the parts
    // outside it have no response worth plotting.
    long long f_start_hz = 70000000LL;
    long long f_stop_hz = 6000000000LL;

    // 20 MSPS is the device's maximum and the sweep's speed. It is also the
    // segment width -- each tuning yields fs/2 of kept spectrum -- so lowering
    // it buys resolution and costs sweep time, proportionally and in both
    // directions. The MAX2837 accepts 2-20 MSPS.
    double fs_hz = 20000000.0;

    // Capped at 4096 by the firmware, not by choice: one tuning delivers one
    // 16384-byte block, 10 bytes of it header, so at most 8187 I/Q pairs are
    // available to transform and 4096 is the largest power of two that fits.
    // Finer RBW comes from lowering fs_hz, which is the honest trade.
    size_t n_fft = 4096;

    size_t n_points = 0;             // 0 derives it from the achieved RBW
    size_t max_points = 65536;

    sweepd::Detector detector = sweepd::Detector::Peak;
    sweepd::Window window = sweepd::Window::Hann;

    // §5's "AGC off. Manual gain." The HackRF has no AGC to switch off, so the
    // rule costs nothing here; these are the three stages, and the defaults are
    // hackrf_sweep's. RF is the ~11 dB amplifier at the antenna port, off by
    // default because it overloads on any real antenna in a city.
    int lna_db = 16;                 // 0-40, 8 dB steps  (MAX2837 IF)
    int vga_db = 20;                 // 0-62, 2 dB steps  (MAX2837 baseband)
    bool amp = false;                // the ~11 dB RF amplifier

    // 0 asks libhackrf for the nearest valid filter to fs_hz.
    //
    // This deliberately differs from hackrf_sweep, which takes the widest valid
    // filter *strictly below* fs -- 15 MHz at 20 MSPS. The kept slices reach
    // 3fs/8 = 7.5 MHz from the LO, which is exactly a 15 MHz filter's -3 dB
    // corner, so that choice puts a scallop at every 5 MHz seam. Asking for
    // 20 MHz instead leaves the kept edge at 0.75 of the corner and flattens it,
    // at the cost of less rejection of the alias that folds onto the kept edge
    // from 12.5 MHz out. Both are defensible; this one is config.
    double bb_filter_hz = 0;

    double overload_frac = 0.98;     // §5's near-full-scale threshold

    // Which device, when more than one is attached. Empty means the first.
    std::string serial;

    // Blocks buffered between the libusb callback and the DSP thread. The
    // callback may not block -- doing so stalls the USB stream and loses
    // tunings -- so on overflow the producer drops and counts, the same
    // contract server.h has with a slow client. A dropped block is a gap in the
    // sweep, so the count is surfaced rather than swallowed.
    size_t queue_blocks = 1024;

    size_t frame_queue_depth = 4;
};

// Everything the source learned by asking the hardware. §7's capability banner
// wants measured facts, not datasheet ones.
struct HackRfInfo {
    std::string serial;
    std::string board;
    std::string firmware;
    uint32_t usb_api = 0;
};

class HackRfSource : public FrameSource {
public:
    explicit HackRfSource(const HackRfOptions &opt);
    ~HackRfSource() override;

    // False with a reason if no device could be opened or configured. The
    // object is inert afterwards and safe to destroy.
    bool open(std::string *err);

    const char *name() const override { return "hackrf"; }
    bool poll(sweepd::proto::TraceMsg *out) override;
    SourceConfig config() const override;
    SourceStatus status() const override;

    // A retune the setter accepted and the device then refused. The worker
    // falls back to the last configuration that ran, so the sweep keeps going
    // and this is where the reason goes.
    bool take_error(std::string *msg) override;

    // All four are genuinely live on this radio: the worker stops the sweep,
    // reconfigures, and runs a fresh discovery sweep. A span or sample-rate
    // change moves the output grid, so the UI sees `reconfiguring` for one
    // sweep period, exactly as it does across a sweepd profile rebuild.
    //
    // set_span clamps to the tunable range (config().tune_min_hz/tune_max_hz),
    // as sweepd's plan does, because "center 473m" at full span asks for a
    // negative start. The config published afterwards reports the clamped span.
    // A span lying wholly outside the tunable range is refused.
    bool set_span(double f_start_hz, double f_stop_hz, std::string *err) override;
    bool set_points(size_t n_points, std::string *err) override;
    bool set_detector(sweepd::Detector d, std::string *err) override;
    bool set_window(sweepd::Window w, std::string *err) override;

    // Refused, with the reason naming the knob that would actually move it:
    // on this radio the kept width per tuning is fs/2 and the DC hole is fs/4,
    // both fixed by the sample rate, and there is no splice/blank choice
    // because the interleaved partner block always fills the hole.
    bool set_segmentation(double keep_hz, const std::string &dc_policy,
                          std::string *err) override;

    // Not implemented here, and refused rather than faked. See the notes on
    // each in the .cpp -- zero span needs a second non-sweep RX path, and the
    // image check needs the LO grid phased by a non-integer number of MHz,
    // which the firmware's uint16 MHz range list cannot express directly.
    bool set_zero_span(double center_hz, size_t n_fft, size_t n_points, double overlap,
                       std::string *err) override;
    bool set_trigger(const std::string &mode, double level_db, double hyst_db,
                     double pretrig, std::string *err) override;
    bool set_lo_shift(double hz, std::string *err) override;
    bool set_sweep_mode(std::string *err) override;

    const HackRfInfo &info() const { return info_; }

private:
    struct Impl;
    std::unique_ptr<Impl> p_;
    HackRfInfo info_;
};

// True when libhackrf was compiled in. sweepui builds and runs without it; the
// --hackrf option then fails with an explanation instead of not existing, so a
// build without the library is distinguishable from a typo.
bool hackrf_available();

}  // namespace sweepui
