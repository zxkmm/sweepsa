// sweepui -- the host-side display. CLAUDE.md §6, delivery steps §10.3 and
// §10.4.
//
// Two sources, one interface. Without --connect it runs against the built-in
// synthetic generator and no radio exists; with --connect it decodes the §5
// stream from a running sweepd. Everything above FrameSource is identical in
// both cases, which is what building the UI first was for -- and the SYNTHETIC
// badge is the only thing on screen that says which one you are looking at.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include "app.h"
#include "freq.h"
#include "hackrfsource.h"
#include "netsource.h"
#include "rtlsdrsource.h"
#include "synthetic.h"

using namespace sweepui;

namespace {

void usage() {
    std::fprintf(stderr,
"usage: sweepui [options]\n"
"\n"
"  With no --connect, runs against a built-in synthetic source: there is no\n"
"  radio involved and every value displayed is fabricated; the window says so.\n"
"\n"
"  --connect HOST[:PORT]  decode the §5 stream from a running sweepd\n"
"                         (sweepd --no-csv --listen 5556). Default port 5556.\n"
"                         IPv6 as [ADDR%%IFACE]:PORT, e.g. a USB gadget's\n"
"                         link-local [fe80::205:f7ff:fe0c:7228%%usb0]:5556.\n"
"                         The options below configure the synthetic source and\n"
"                         are ignored when --connect is given -- with a real\n"
"                         daemon the span, points, detector and window are the\n"
"                         daemon's, taken from its argv.\n"
"\n"
"  --hackrf           sweep a HackRF One on this host's USB instead. A second\n"
"                     reference source, not a replacement for sweepd: a full\n"
"                     70 MHz - 6 GHz sweep is ~594 tunings and takes roughly\n"
"                     half a second, which is the comparison §1 exists to make.\n"
"                     Unlike sweepd it does reconfigure while running, so\n"
"                     --start/--stop/--points/--detector/--window apply, and so\n"
"                     do the span, points, detector and window soft keys.\n"
"                     Zero span, the image check and the segment overlay are\n"
"                     not available on it and say so when asked.\n"
"  --hackrf-rate HZ   sample rate, 2-20 MHz. This is the segment width: each\n"
"                     tuning keeps fs/2, so lowering it buys RBW and costs\n"
"                     sweep time in the same proportion.      (default 20m)\n"
"  --hackrf-fft N     FFT size, power of two, 64-4096. The cap is the block\n"
"                     format's, not a choice -- one tuning delivers one 16384\n"
"                     byte block. Lower --hackrf-rate for finer RBW. (default 4096)\n"
"  --hackrf-lna DB    IF gain, 0-40 in 8 dB steps                  (default 16)\n"
"  --hackrf-vga DB    baseband gain, 0-62 in 2 dB steps            (default 20)\n"
"  --hackrf-amp       enable the ~11 dB RF amplifier. Off by default because it\n"
"                     overloads on any real antenna in a city.\n"
"  --hackrf-bw HZ     baseband filter. 0 asks for the nearest valid filter to\n"
"                     the sample rate, which is wider than hackrf_sweep picks\n"
"                     and flattens the seam at each 5 MHz boundary.  (default 0)\n"
"  --hackrf-serial S  which device, when more than one is attached\n"
"\n"
"  --rtl              sweep an RTL-SDR dongle on this host's USB instead. The\n"
"                     cheapest second reference in the tree, and the only host\n"
"                     source that sweeps §5's own segment grid -- it calls the\n"
"                     daemon's planner, so §7's segment overlay and §7's image\n"
"                     check both work on it, which they do not on --hackrf.\n"
"                     It is also by far the slowest: a full 24-1766 MHz sweep is\n"
"                     ~2900 segments and ~18 s, because each hop pays a settle\n"
"                     the host has to guess at. Narrow spans are where it is\n"
"                     pleasant. Zero span and the video trigger are not\n"
"                     implemented on it and say so when asked.\n"
"  --rtl-list         print the attached dongles and exit\n"
"  --rtl-rate HZ      sample rate. librtlsdr takes 225001-300000 and\n"
"                     900001-3200000 and nothing between; above ~2.4m the\n"
"                     dongle drops samples and cannot report it.  (default 2.4m)\n"
"  --rtl-fft N        FFT size, power of two, 64-65536. No block-format cap\n"
"                     here, unlike the HackRF: a bigger FFT buys RBW and costs\n"
"                     almost nothing, since the settle dominates. (default 1024)\n"
"  --rtl-keep HZ      usable width per segment; 0 derives fs/2. Wider is a\n"
"                     faster sweep with dirtier seams, because the splice fill\n"
"                     is then drawn from further off the neighbour's LO.\n"
"                                                                  (default 0)\n"
"  --rtl-blank HZ     half-width blanked around each LO for its own leakage.\n"
"                     The R820T's centre spike has not been measured; widen\n"
"                     this if a comb survives at the segment centres.\n"
"                                                                (default 25k)\n"
"  --rtl-dc POLICY    splice|blank. blank is twice as fast and leaves a hole at\n"
"                     every segment centre, visibly.           (default splice)\n"
"  --rtl-gain DB      snapped to the tuner's own gain list, and the snapped\n"
"                     value is what the status bar shows. A number above the\n"
"                     top of the list means maximum.                (default 30)\n"
"  --rtl-ppm N        crystal correction. Worth setting: an uncorrected dongle\n"
"                     is routinely tens of ppm out.                  (default 0)\n"
"  --rtl-bw HZ        tuner IF filter; 0 asks for automatic          (default 0)\n"
"  --rtl-settle US    per-hop settle. This is ~80%% of the sweep time and it is\n"
"                     inherited from rtl_power rather than measured, so it is\n"
"                     the first thing worth measuring on a real dongle --\n"
"                     `rtl_probe --settle-scan` is how.           (default 5000)\n"
"  --rtl-dump BYTES   discarded after each retune to flush the USB pipeline\n"
"                     (default 4096, which is rtl_power's)\n"
"  --rtl-direct N     0 off, 1 = I branch, 2 = Q branch. Bypasses the tuner and\n"
"                     samples HF directly, which is the only way this radio\n"
"                     sees below 24 MHz. Untested here.             (default 0)\n"
"  --rtl-offset-tuning  E4000 only; librtlsdr refuses it on an R820T\n"
"  --rtl-bias-tee     put ~4.5 V on the antenna port to feed an inline LNA.\n"
"                     Off by default, and there is nothing protecting whatever\n"
"                     is connected.\n"
"  --rtl-tune-min HZ  override the tuner's published range with a measured one.\n"
"  --rtl-tune-max HZ  Both must be given to take effect.\n"
"  --rtl-serial S     which dongle, by serial\n"
"  --rtl-index N      which dongle, by index\n"
"\n"
"  --start FREQ       sweep start          (default 70m)\n"
"  --stop FREQ        sweep stop           (default 6g)\n"
"  --points N         points per trace     (default 8192)\n"
"  --rate HZ          sweeps per second    (default 20)\n"
"  --segments N       segments per sweep, for the DC holes and the overload\n"
"                     mask                 (default 149)\n"
"  --no-holes         do not punch dc_policy=blank NaN holes\n"
"  --image-rej DB     how far below its parent the generator puts each mirror\n"
"                     image, for §7's image check    (default 42; 999 = none)\n"
"  --detector NAME    peak|negpeak|sample|rms          (default peak)\n"
"  --window NAME      rect|hann|blackmanharris|flattop (default hann)\n"
"  --seed N           generator seed, for reproducible screenshots (default 1)\n"
"  --geometry WxH     window size, e.g. 1600x1400              (default 1600x950)\n"
"  --wf-cols N        waterfall history width          (default 2048)\n"
"  --wf-rows N        waterfall history depth          (default 1024)\n"
"  --no-vsync         uncap the frame rate; use with --perf to check §6's 60 fps\n"
"  --perf             show the frame-time overlay at startup\n"
"  --capture FILE     render a few frames, write FILE, exit. PNG, unless the\n"
"                     name ends in .ppm. Same code path as the Export soft key.\n"
"  --capture-after N  frames to render first                    (default 60)\n"
"  --do \"CMD\"         press a soft key from the command line; repeatable, run\n"
"                     in order once a sweep has arrived. This is how the UI is\n"
"                     driven on a compositor that will not let an external tool\n"
"                     click a window. Examples:\n"
"                       --do \"trace 2 max hold\" --do \"peak\" --do \"csv out.csv\"\n"
"                     trace N MODE | avg N COUNT | band 0|1 | clear traces\n"
"                     scale log|lin   graticule log, or linear in amplitude\n"
"                     active N | marker N on|off|at FREQ|trace T|delta R|noise B\n"
"                     peak | next | left | right | excursion DB | threshold DB\n"
"                     center FREQ | span FREQ | ref DB | pause 0|1\n"
"                     keep FREQ|splice|blank   segment width and DC policy; the\n"
"                       sweep rate is the segment count, and the segment count\n"
"                       is the span over the step. Costs a profile rebuild.\n"
"                     gaintable FILE|builtin|stock   RX gain table: one table\n"
"                       over the whole tuner keeps the gain index from stepping\n"
"                       between bands; builtin is sweepd's default, stock the\n"
"                       driver's own. Costs a profile rebuild. gaintables/README.md.\n"
"                     zspan on|off|FREQ [fft N] [points N] [overlap F]\n"
"                     iq on|off|FREQ | iq run|stop|single | iq n N | iq decim D\n"
"                     iq trig none|normal|auto | iq src i|q|mag | iq slope rise|fall\n"
"                     iq level FS | iq hyst FS | iq pretrig F | iq auto SECONDS\n"
"                       raw IQ time sink: I and Q against time at a parked LO\n"
"                     iqch i|q|mag 0|1 | iqy2 none|phase|freq | iqdc 0|1\n"
"                     iqx time|samples | iqpersist N | iqxy 0|1\n"
"                     iqcursor t A B | iqcursor v A B | iqcursor off\n"
"                     iqscope burst|cursors | iqexport PATH (.sigmf-meta .cfile .sc16 .csv)\n"
"                     trig free|rising|falling | trig level|hyst|pretrig VALUE\n"
"                     csv PATH | png PATH | save PATH | load PATH\n"
"                     ampcorr PATH|off | segments 0|1\n"
"                     imgchk [N] | imgchk off | imgchk?\n"
"                     imgchk-excursion DB | imgchk-drop DB\n"
"                       §7's image check. Two passes with the LO grid phased\n"
"                       between them; it does not block, so follow it with a\n"
"                       `wait` long enough for 2*N sweeps and a rebuild, then\n"
"                       `imgchk?` to print the verdicts.\n"
"  --do-after N       frames to draw before running them        (default 30)\n"
"  --ampcorr FILE     §7 amplitude correction: a 'freq_hz,offset_db' CSV. Until\n"
"                     one is loaded the instrument reads dBFS and UNCAL is lit.\n"
"                     The file is never extrapolated, so the units only become\n"
"                     dBm while the swept span fits inside its frequency range.\n"
"                     See docs/amplitude-correction.md for how to generate one.\n"
"  -h, --help\n"
"\n"
"  FREQ accepts instrument suffixes: 2.4g, 100m, 455k, or plain Hz.\n");
}

bool freq_arg(const char *s, double *out) {
    if (parse_frequency(s, out)) return true;
    std::fprintf(stderr, "cannot read \"%s\" as a frequency\n", s);
    return false;
}

}  // namespace

int main(int argc, char **argv) {
    SyntheticOptions so;
    AppOptions ao;
    NetOptions no;
    HackRfOptions ho;
    RtlSdrOptions ro;
    bool connect = false;
    bool hackrf = false;
    bool rtl = false;
    bool rtl_list = false;
    bool synthetic_opt_given = false;

    // Tracked separately from the values because the shared --start/--stop/
    // --points defaults are the synthetic source's, and 70 MHz - 6 GHz is not a
    // sensible default span for a dongle that stops at 1766 MHz. An RTL-SDR
    // therefore keeps its own defaults unless these were actually typed.
    bool start_given = false, stop_given = false, points_given = false;

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

        double f = 0;
        if (a == "--connect") {
            std::string h = need();
            // "[fe80::1%usb0]:5556" -- a bracketed IPv6 literal, or a bare one
            // with no port. Only a single colon can be the port separator.
            if (!h.empty() && h[0] == '[') {
                const size_t close = h.find(']');
                if (close == std::string::npos) {
                    std::fprintf(stderr, "--connect: unclosed [ in \"%s\"\n", h.c_str());
                    return 2;
                }
                if (close + 1 < h.size() && h[close + 1] == ':')
                    no.port = uint16_t(std::atoi(h.c_str() + close + 2));
                h = h.substr(1, close - 1);
            } else if (h.find(':') == h.rfind(':') && h.find(':') != std::string::npos) {
                const size_t colon = h.rfind(':');
                no.port = uint16_t(std::atoi(h.c_str() + colon + 1));
                h = h.substr(0, colon);
            }
            if (h.empty()) {
                std::fprintf(stderr, "--connect needs a host\n");
                return 2;
            }
            no.host = h;
            connect = true;
        } else if (a == "--port") {
            no.port = uint16_t(std::atoi(need()));
        } else if (a == "--hackrf") {
            hackrf = true;
        } else if (a == "--hackrf-rate") {
            if (!freq_arg(need(), &f)) return 2;
            ho.fs_hz = f;
        } else if (a == "--hackrf-fft") {
            ho.n_fft = size_t(std::strtoul(need(), nullptr, 10));
        } else if (a == "--hackrf-lna") {
            ho.lna_db = std::atoi(need());
        } else if (a == "--hackrf-vga") {
            ho.vga_db = std::atoi(need());
        } else if (a == "--hackrf-amp") {
            ho.amp = true;
        } else if (a == "--hackrf-bw") {
            if (!freq_arg(need(), &f)) return 2;
            ho.bb_filter_hz = f;
        } else if (a == "--hackrf-serial") {
            ho.serial = need();
        } else if (a == "--rtl") {
            rtl = true;
        } else if (a == "--rtl-list") {
            rtl_list = true;
        } else if (a == "--rtl-rate") {
            if (!freq_arg(need(), &f)) return 2;
            ro.fs_hz = f;
        } else if (a == "--rtl-fft") {
            ro.n_fft = size_t(std::strtoul(need(), nullptr, 10));
        } else if (a == "--rtl-keep") {
            if (!freq_arg(need(), &f)) return 2;
            ro.keep_hz = (long long)llround(f);
        } else if (a == "--rtl-blank") {
            if (!freq_arg(need(), &f)) return 2;
            ro.blank_hz = (long long)llround(f);
        } else if (a == "--rtl-dc") {
            if (!sweepd::dc_policy_from_string(need(), &ro.dc)) {
                std::fprintf(stderr, "--rtl-dc wants splice or blank\n");
                return 2;
            }
        } else if (a == "--rtl-gain") {
            ro.gain_db = std::strtod(need(), nullptr);
        } else if (a == "--rtl-ppm") {
            ro.ppm = std::atoi(need());
        } else if (a == "--rtl-bw") {
            if (!freq_arg(need(), &f)) return 2;
            ro.bw_hz = (long long)llround(f);
        } else if (a == "--rtl-settle") {
            ro.settle_us = unsigned(std::strtoul(need(), nullptr, 10));
        } else if (a == "--rtl-dump") {
            ro.dump_bytes = size_t(std::strtoul(need(), nullptr, 10));
        } else if (a == "--rtl-direct") {
            ro.direct_sampling = std::atoi(need());
        } else if (a == "--rtl-offset-tuning") {
            ro.offset_tuning = true;
        } else if (a == "--rtl-bias-tee") {
            ro.bias_tee = true;
        } else if (a == "--rtl-tune-min") {
            if (!freq_arg(need(), &f)) return 2;
            ro.tune_min_hz = (long long)llround(f);
        } else if (a == "--rtl-tune-max") {
            if (!freq_arg(need(), &f)) return 2;
            ro.tune_max_hz = (long long)llround(f);
        } else if (a == "--rtl-serial") {
            ro.serial = need();
        } else if (a == "--rtl-index") {
            ro.index = std::atoi(need());
        } else if (a == "--start") {
            if (!freq_arg(need(), &f)) return 2;
            so.f_start_hz = (long long)llround(f);
            start_given = true;
            synthetic_opt_given = true;
        } else if (a == "--stop") {
            if (!freq_arg(need(), &f)) return 2;
            so.f_stop_hz = (long long)llround(f);
            stop_given = true;
            synthetic_opt_given = true;
        } else if (a == "--points") {
            so.n_points = size_t(std::strtoul(need(), nullptr, 10));
            points_given = true;
            synthetic_opt_given = true;
        } else if (a == "--rate") {
            so.sweeps_per_s = std::strtod(need(), nullptr);
            synthetic_opt_given = true;
        } else if (a == "--image-rej") {
            so.image_rej_db = std::strtod(need(), nullptr);
        } else if (a == "--segments") {
            so.n_segments = uint32_t(std::strtoul(need(), nullptr, 10));
            synthetic_opt_given = true;
        } else if (a == "--no-holes") {
            so.blank_holes = false;
            synthetic_opt_given = true;
        } else if (a == "--detector") {
            if (!sweepd::detector_from_string(need(), &so.detector)) {
                std::fprintf(stderr, "unknown detector\n");
                return 2;
            }
            synthetic_opt_given = true;
        } else if (a == "--window") {
            if (!sweepd::window_from_string(need(), &so.window)) {
                std::fprintf(stderr, "unknown window\n");
                return 2;
            }
            synthetic_opt_given = true;
        } else if (a == "--seed") {
            so.seed = uint32_t(std::strtoul(need(), nullptr, 10));
            synthetic_opt_given = true;
        } else if (a == "--geometry") {
            // Here for the same reason --capture is: this host's compositor will
            // not let an external tool size or screenshot a GLFW window, so the
            // only way to see a panel that falls below the fold is to ask for a
            // taller one.
            const char *geo = need();
            const char *x = std::strchr(geo, 'x');
            if (!x) {
                std::fprintf(stderr, "--geometry wants WxH, e.g. 1600x950\n");
                return 2;
            }
            ao.width = int(std::strtol(geo, nullptr, 10));
            ao.height = int(std::strtol(x + 1, nullptr, 10));
            if (ao.width < 640 || ao.height < 480) {
                std::fprintf(stderr, "--geometry: %dx%d is too small to lay out\n", ao.width,
                             ao.height);
                return 2;
            }
        } else if (a == "--wf-cols") {
            ao.waterfall_cols = size_t(std::strtoul(need(), nullptr, 10));
        } else if (a == "--wf-rows") {
            ao.waterfall_rows = size_t(std::strtoul(need(), nullptr, 10));
        } else if (a == "--no-vsync") {
            ao.vsync = false;
        } else if (a == "--perf") {
            ao.show_perf = true;
        } else if (a == "--capture") {
            ao.capture_path = need();
        } else if (a == "--capture-after") {
            ao.capture_after = std::atoi(need());
        } else if (a == "--do") {
            ao.commands.push_back(need());
        } else if (a == "--do-after") {
            ao.command_after = std::atoi(need());
        } else if (a == "--ampcorr") {
            ao.ampcorr_path = need();
        } else if (a == "-h" || a == "--help") {
            usage();
            return 0;
        } else {
            std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
            usage();
            return 2;
        }
    }

    if (so.f_stop_hz <= so.f_start_hz) {
        std::fprintf(stderr, "stop must be above start\n");
        return 2;
    }

    if (rtl_list) {
        if (!rtlsdr_available()) {
            std::fprintf(stderr,
                         "sweepui: built without librtlsdr, so there is nothing to "
                         "list. Install librtlsdr-dev (or rtl-sdr) and reconfigure "
                         "CMake.\n");
            return 1;
        }
        const std::vector<std::string> devs = rtlsdr_devices();
        if (devs.empty()) {
            std::fprintf(stderr,
                         "sweepui: no RTL-SDR found. If one is attached, the kernel DVB "
                         "driver has probably claimed it: blacklist dvb_usb_rtl28xxu.\n");
            return 1;
        }
        for (const std::string &d : devs) std::printf("%s\n", d.c_str());
        return 0;
    }

    if (int(connect) + int(hackrf) + int(rtl) > 1) {
        std::fprintf(stderr,
                     "--connect, --hackrf and --rtl are three different radios; pick one\n");
        return 2;
    }

    std::unique_ptr<FrameSource> src;
    if (rtl) {
        // Shared with the synthetic source where they mean the same thing, and
        // left alone where they do not: the span defaults are §6's full range
        // and this radio's is a twentieth of it. See the note on start_given.
        if (start_given) ro.f_start_hz = so.f_start_hz;
        if (stop_given) ro.f_stop_hz = so.f_stop_hz;
        if (points_given) ro.n_points = so.n_points;
        ro.detector = so.detector;
        ro.window = so.window;

        std::unique_ptr<RtlSdrSource> r(new RtlSdrSource(ro));
        std::string err;
        if (!r->open(&err)) {
            std::fprintf(stderr, "sweepui: %s\n", err.c_str());
            return 1;
        }
        const RtlSdrInfo &in = r->info();
        std::fprintf(stderr, "sweepui: RTL-SDR %s, %s tuner, serial %s\n",
                     in.name.c_str(), rtl_tuner_name(in.tuner),
                     in.serial.empty() ? "?" : in.serial.c_str());
        if (in.nextrtl_hf_lo_hz)
            std::fprintf(stderr,
                         "sweepui: nextrtl, HF upconverted below 30 MHz with a %.6f MHz LO\n",
                         in.nextrtl_hf_lo_hz / 1e6);
        else if (in.nextrtl_strings)
            std::fprintf(stderr,
                         "sweepui: a nextrtl board on a librtlsdr without nextrtl support: "
                         "no HF, plain FC0013 range.\n         Build against the fork, see "
                         "docs/rtlsdr.md.\n");
        std::fprintf(stderr, "sweepui: %.4f MSPS achieved, gain %.1f dB of [%s]\n",
                     in.fs_achieved_hz / 1e6, in.gain_achieved_db,
                     in.gain_list().c_str());
        // Said once, out loud, because it is the number that decides whether
        // this radio is pleasant or useless for the span that was asked for.
        std::fprintf(stderr,
                     "sweepui: per-hop settle is %u us, inherited from rtl_power and "
                     "NOT measured.\n         It is most of the sweep time; see "
                     "docs/rtlsdr.md and `rtl_probe --settle-scan`.\n",
                     ro.settle_us);
        src = std::move(r);
    } else if (hackrf) {
        // The span, points, detector and window options are shared with the
        // synthetic source rather than duplicated: on this radio they are all
        // live, so they mean the same thing in both places.
        ho.f_start_hz = so.f_start_hz;
        ho.f_stop_hz = so.f_stop_hz;
        ho.detector = so.detector;
        ho.window = so.window;
        // The synthetic default is 8192 points; a HackRF's point count should
        // follow the RBW it actually achieved unless asked otherwise, which is
        // what 0 means here.
        ho.n_points = points_given ? so.n_points : 0;

        std::unique_ptr<HackRfSource> h(new HackRfSource(ho));
        std::string err;
        if (!h->open(&err)) {
            std::fprintf(stderr, "sweepui: %s\n", err.c_str());
            return 1;
        }
        std::fprintf(stderr, "sweepui: HackRF %s, %s, fw %s\n", h->info().board.c_str(),
                     h->info().serial.c_str(), h->info().firmware.c_str());
        src = std::move(h);
    } else if (connect) {
        // Said out loud rather than silently ignored: with a real daemon these
        // are sweepd's to decide, and a user who typed --points 4096 and saw
        // 1024 points would reasonably conclude the UI was broken.
        if (synthetic_opt_given)
            std::fprintf(stderr,
                         "sweepui: --connect given, so the synthetic-source options are "
                         "ignored;\n         span, points, detector and window come from "
                         "sweepd's own argv.\n");
        std::fprintf(stderr, "sweepui: connecting to %s:%u\n", no.host.c_str(),
                     unsigned(no.port));
        src.reset(new NetSource(no));
    } else {
        src.reset(new SyntheticSource(so));
    }

    App app(std::move(src), ao);
    std::string err;
    if (!app.init(&err)) {
        std::fprintf(stderr, "sweepui: %s\n", err.c_str());
        return 1;
    }
    app.run();
    return 0;
}
