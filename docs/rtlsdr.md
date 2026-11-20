> [!IMPORTANT]
> The rtlsdr support is not included in the release binary, due to librtlsdr's license. Compile sweepui from this codebase if you want to use SweepSA with rtlsdr.

# The RTL-SDR source

`sweepui --rtl` sweeps an RTL-SDR dongle on the host's own USB, and
`rtl_probe` drives the same source with no display.

**Read this first: nothing in this file was measured on a dongle.** No RTL-SDR
was attached when the source was written. `docs/hackrf.md` and
`docs/characterization.md` record measurements; this file records *derivations*
and *inheritances*, and it is explicit about which each number is. Every line
below is one of:

| tag | meaning |
|-----|---------|
| **derived** | arithmetic from the configuration, checked in `tests/rtlsdr_test.cpp` |
| **inherited** | taken from `rtl_power` or librtlsdr's own source, cited |
| **published** | somebody's datasheet, believed but unverified here |
| **MEASURE** | a blank waiting for hardware, with the command that fills it |

CLAUDE.md §8 forbids hardcoded timing constants and §7 wants measured
capability rather than datasheet capability. This source cannot satisfy either
yet, so it satisfies the next best thing: every such number is config, and this
file says out loud that it is not a measurement.

## Why it is in the tree

Not as a replacement for `sweepd`, and on §1's own metric it is the worst radio
here — see the sweep budget below. Three reasons, in order:

1. **It is the cheapest possible second reference.** §2 nominates a LibreSDR as
   the validation radio, and PROJECT-STATE.md notes that plugging the AntSDR in
   costs the host its network. A dongle needs neither.
2. **It is the only host-side source that sweeps §5's own segment grid.** It
   calls `sweepd::plan_sweep` — the daemon's planner, not a copy — so the LO
   grid *is* the daemon's grid. That makes three things work here that the
   HackRF source has to refuse:
   - §7's **segment boundary overlay**, because the LO grid is uniform and
     reconstructible (the HackRF's interleaved LOs are not);
   - §7's **image check**, because the grid can be phased by an arbitrary number
     of hertz (the HackRF firmware's range list is `uint16` MHz);
   - `keep_hz` and the **DC policy** as live knobs, meaning what §5 means.

   Before this, those could only be exercised against the synthetic source or
   the E310.
3. It reconfigures while running, which `sweepd` still refuses.

## The geometry — derived

`plan_sweep` with `keep = fs/2`, `blank = ±25 kHz`, `dc = splice`, so the step
is `keep/2 = fs/4`.

At fs = 2.4 MSPS: **keep 1.2 MHz, step 600 kHz**, the kept band is the LO
± 600 kHz and the blanked hole is the LO ± 25 kHz.

**Why `keep = fs/2` and not the HackRF's `3fs/4`.** Under `splice` the value
shown at segment *k*'s centre was measured by *k*'s neighbour at exactly one
step from that neighbour's own LO — the extreme edge of its kept band.
PROJECT-STATE.md records what that costs on the E310 ("the splice fill is
+4.5 dB hot", scaling with `keep_hz`). At `keep = 3fs/4` the fill would come
from `3fs/8` = 900 kHz out, which at 2.4 MSPS is sitting on the R820T's
automatic IF filter corner, so the comb would return as a *scallop* instead. At
`keep = fs/2` the fill comes from `fs/4`, half of Nyquist, well inside any sane
filter.

`--rtl-keep` is the knob and the trade is monotonic both ways: wider is a faster
sweep with dirtier seams. `tests/rtlsdr_test.cpp` pins both directions.

> **MEASURE.** Whether the comb or the scallop actually appears, and at what
> level. `rtl_probe --start 98m --stop 108m --csv a.csv --keep 1.2m` against
> `--keep 1.8m`, then compare the level at the segment centres. The overlay
> (`--do "segments 1"` in `sweepui`) is what marks them.

**The DC blank is a guess.** ±25 kHz is wide enough for the usual R820T centre
spike and nothing here has seen this one.

> **MEASURE.** Park on a quiet band and look at the residual at the segment
> centres with the overlay on. Widen `--rtl-blank` until it goes.

## The sweep budget — derived, and it is the headline

There is no firmware sweep mode. The AD936x has fastlock and the HackRF has
`hackrf_init_sweep`; an RTL-SDR has neither, so the **host** retunes:

```
set_center_freq → wait → discard → read → FFT → stitch → next LO
```

One hop at the defaults costs 5 ms of settle plus 1.28 ms of samples (a
4096-byte discard plus a 2048-byte read at 2.4 MSPS) ≈ **6.3 ms, of which 80% is
the wait.** Segment counts are the planner's:

| span | segments | predicted |
|------|----------|-----------|
| 88–108 MHz (FM) | 35 | ~0.22 s |
| 1200–1300 MHz | 168 | ~1.1 s |
| 24–1766 MHz (R820T full) | 2905 | ~18 s |

For comparison, on the same full-span job: a HackRF takes **729 ms** measured
(`docs/hackrf.md`), and fastlock's whole reason for existing is that it should
take milliseconds. An RTL-SDR is ~25× slower than the HackRF and it is not close
to being a contest. **Narrow spans are where this radio is pleasant**, and the
FM band at ~4.5 sweeps/s genuinely is.

2.4 GHz is absent from that table because an R820T cannot reach it. This radio
stops at 1766 MHz, which is one of the things §7's capability banner is for.

## The settle — inherited, and the one number worth measuring

Both settle constants come from `rtl-sdr/src/rtl_power.c`, function `retune()`:

```c
rtlsdr_set_center_freq(d, (uint32_t)freq);
/* wait for settling and flush buffer */
usleep(5000);
rtlsdr_read_sync(d, &dump, BUFFER_DUMP, &n_read);   /* BUFFER_DUMP = 1<<12 */
```

So `settle_us = 5000` and `dump_bytes = 4096`, **inherited**, cited to that
file and to nothing else. Note what `rtl_power` does *not* do: it does not
`rtlsdr_reset_buffer` per hop. It resets once at startup and flushes each retune
with a discarded read instead, because a FIFO reset costs a USB control transfer
on a path that is already the bottleneck. This source does the same.

Since the settle is ~80% of the sweep, measuring it is worth more than every
other optimisation available here combined. That is CLAUDE.md §4b.2 for this
radio, and `rtl_probe` has a mode for it:

```
rtl_probe --start 98m --stop 100m --settle-scan 0,250,500,1000,2000,5000
```

Point it at a band with a strong steady carrier — an FM broadcast station is
ideal — and read the table downward. The settle is long enough at the point
where `max` stops climbing and `median` stops falling; everything longer is
sweep time spent on nothing.

> **MEASURE.** The settle scan above. If the real figure is, say, 500 µs, the
> full-span sweep drops from ~18 s to ~5 s.

## Tuner ranges — published, not measured

`rtl_tuner_range()`, keyed on `rtlsdr_get_tuner_type()`. The part *is* queried;
the range attached to it is a datasheet number.

| tuner | range | note |
|-------|-------|------|
| R820T / R828D | 24–1766 MHz | the common case |
| E4000 | 52–2200 MHz | |
| FC0012 | 22–948.6 MHz | |
| FC0013 | 22–1100 MHz | |
| FC2580 | 146–924 MHz | **two bands**, 146–308 and 438–924, with a hole between |
| unknown | *refused* | |
| any, with direct sampling | 0.1–14.4 MHz | the RTL2832's first Nyquist zone |
| nextrtl, on the fork | 0.1–1700 MHz | see *nextrtl* below; partly **measured** |

Two honesty notes. The FC2580's gap is not modelled — `PlanConfig` takes one
interval, so a sweep across the hole shows it as noise rather than as a gap; an
FC2580 is rare enough that teaching the planner about disjoint tuner bands for
it would be work spent on nobody's dongle. And an **unidentified tuner is
refused outright** rather than given a guessed range, because every other row
here is at least somebody's datasheet and that one would be nothing at all.
`--rtl-tune-min` / `--rtl-tune-max` override the table, which is how a §4a-style
sensitivity sweep on a specific dongle gets to narrow it.

> **MEASURE.** §4a for this radio: step a known source across the range and find
> where the response actually falls off. The table above is what the banner
> currently shows; a measured curve is what §7 asks for.

## Amplitude — derived scale, no accuracy claim

Samples are unsigned 8-bit centred on **127.5**, the arithmetic midpoint of
0…255, and normalised by the same number, so a signal swinging the full 0…255
reads exactly 0 dBFS. That matches `SweepDevice::full_scale()`'s 2048 for the
AD936x and `hackrfsource.cpp`'s 128 for int8.

librtlsdr's own tools use 127.4, an empirical correction for the RTL2832's
residual DC offset. The difference lands entirely in the DC bin, which this
source blanks in every segment, so the arithmetic midpoint is the defensible
choice and the fudge would buy nothing.

Gain is **not a range with a step** — it is a list. `rtlsdr_get_tuner_gains`
returns 29 unevenly spaced entries from 0 to 49.6 dB on an R820T, librtlsdr
rejects anything not on it, and a rejected gain would mean a whole sweep at
whatever the gain already was. So requests are snapped to the nearest entry and
**the snapped value is what the status bar reports**. The 30 dB default snaps to
29.7. A request above the top of the list means maximum, which is how
`--rtl-gain max`-style usage works without a separate flag.

Both AGCs are off, per §5's "AGC off. Ever." This device has two —
`rtlsdr_set_tuner_gain_mode(dev, 1)` for the tuner and `rtlsdr_set_agc_mode(dev, 0)`
for the RTL2832's digital AGC — and turning off only the obvious one leaves the
other wandering the amplitude between segments.

None of this is calibrated. The trace is dBFS and UNCAL is lit, exactly as with
`sweepd`; §7's amplitude correction applies here unchanged.

> **MEASURE.** The `docs/hackrf.md` cross-check, repeated: same band and gain
> against `rtl_power`, with the ~6 dB normalisation difference and the RBW ratio
> predicted in advance.

## Two things this radio cannot tell you

Both are assumptions, listed as assumptions.

- **The achieved LO is not readable.** The AD936x cold build writes an LO and
  reads back what the driver actually tuned to, and every bin-to-RF mapping uses
  the read-back (`plan.h`, `Segment::lo_hz`). `rtlsdr_get_center_freq` returns
  librtlsdr's cached copy of what it was *asked* for, not what the PLL reached.
  So the trace is built on the requested LO and any synthesizer rounding is a
  small frequency error this source cannot see. It is bounded by the tuner's own
  step and is far below one output bin at any useful span — but it is not
  measured. Set `--rtl-ppm`; an uncorrected dongle is routinely tens of ppm out,
  which at 1.7 GHz is tens of kilohertz.
- **Dropped samples are invisible.** `rtlsdr_read_sync` does not report a USB
  underrun, and a chunk missing from the middle of a buffer is a discontinuity
  inside a transform — broadband, and shaped exactly like signal. This is why
  the default rate is 2.4 MSPS and not the 3.2 MSPS the device accepts;
  `rtl_power` caps itself at 2.8 for the same reason. Above 2.4 MSPS the
  capability banner says so.

What the source *can* count, it does, into the status line: tunes the driver
refused, short reads, and output bins nothing covered.

## Checking it

`rtl_probe` prints the trace as numbers, the way `sweep_client` does for the TCP
source and `hackrf_probe` for the HackRF:

```
rtl_probe --list
rtl_probe --start 88m --stop 108m --peaks 12
rtl_probe --start 88m --stop 108m --sweeps 20 --csv fm.csv
rtl_probe --start 98m --stop 100m --settle-scan 0,250,500,1000,2000,5000
```

The line worth reading first is `uncovered N of M bins`. Under the default
`splice` policy it should be **0**; anything else means the segment geometry and
the output grid disagree, which is the failure that looks like a working trace.

With no dongle attached, `tests/rtlsdr_test.cpp` still checks everything that
does not need one — the derived plan, the overlay's agreement with it, the gain
snapping against a real R820T list, and the librtlsdr rate windows edge by edge.
It runs in the ordinary `ctest` suite.

## Not implemented, and why

- **Zero span and the video trigger.** Refused with the reason. This is the
  source where zero span would be *easiest* — the read path is already a
  parked-LO read loop, and `sweepd::ZeroSpan` is written against the
  `ZeroSpanDevice` interface rather than against the AD936x, so the engine, the
  trigger and the duty accounting would all be reused. What it still needs is a
  second mode in the worker and its own status fields. It is the next thing this
  source should grow, and given how slowly it sweeps it is arguably where the
  radio is most useful.
- **A second thread.** Deliberately absent. The loop is serial by nature: the
  radio cannot produce usable samples while it is retuning, and an N=1024
  transform costs tens of microseconds against a 6.3 ms hop. A pipeline would
  add a thread and a synchronisation bug to save ~1% of the sweep.

## nextrtl

nextrtl is an RTL2832U + FC0013 board with an onboard HF upconverter. Its
librtlsdr fork (`nextrtl-librtlsdr`) recognises the board by its EEPROM strings
(`zxkmm` / `nextrtl`) and, below 30 MHz, switches to the upconverter and tunes
the FC0013 to f + LO, so this source sees ordinary frequencies.

**Building against the fork.** pkg-config finds whichever librtlsdr comes first,
so point it at the fork's local install:

```sh
cmake --install <fork>/build --prefix <fork>/install   # once, in the fork
cmake -S . -B build/host -G Ninja -DCMAKE_PREFIX_PATH=<fork>/install
```

The configure line prints the library path it picked. The binaries carry it as
RUNPATH, so they load the fork without `LD_LIBRARY_PATH`. If the cache already
holds the system library, add `-U 'RTLSDR_*' -U __pkg_config_checked_RTLSDR
-U 'pkgcfg_lib_RTLSDR_*'` once.

**Detection.** The fork exports `rtlsdr_get_nextrtl_hf_lo()`. The source
declares it weak, so on the stock library it is null and a nextrtl board gets
the plain FC0013 row. `sweepui` says so on stderr rather than silently losing HF.

**The range.**
- Low end, 0.1 MHz: **derived**, same floor and reasoning as direct sampling.
- High end, 1.7 GHz: **measured** on the rev 1 board (2026-10-08). The FC0013
  PLL locks to 1.7 GHz, and a −60 dBm 1575.52 MHz tone on the main input reached
  ~73 dB over the noise, well past the 1.1 GHz the FC0013 row claims. Above
  1575 MHz only the PLL lock is measured.

**Things that differ from a plain dongle:**
- **The 30 MHz seam.** Segments either side of 30 MHz come through different
  paths (upconverter vs direct), so expect a level step there.
- **The LO restarts.** On rev 1, the 125 MHz LO is powered only on the HF path,
  so every sweep that crosses 30 MHz cold-starts it. The inherited 5 ms settle
  was never chosen with an oscillator start-up in mind.
  > **MEASURE.** `rtl_probe --start 1m --stop 60m --settle-scan 5000,20000,50000`
  > with a steady HF carrier, and compare the first HF segments.
- **Own responses.** Crystal harmonics land in the HF window through the
  upconverter: 144 MHz = 5 × 28.8 shows at 19.0 MHz.
- **Frequency error.** The rev 1 LO is a free-running oscillator calibrated per
  board into the EEPROM (`nextrtl_cal.py` in the fork). Uncalibrated, HF reads a
  few kHz off; `--ppm` covers the 28.8 MHz crystal only.
- **L-band jack.** The fork uses the main input above 1 GHz unless
  `NEXTRTL_LBAND=1`; the dedicated L-band input measured ~31 dB weaker.

