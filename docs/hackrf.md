# The HackRF source — measured

`sweepui --hackrf` sweeps a HackRF One on the host's own USB. This file records
what was measured on the attached device rather than what the datasheet or
`hackrf_sweep` implies, on the same terms as `docs/characterization.md`: every
constant in `src/sweepui/hackrfsource.cpp` should cite a line here.

**Device.** HackRF One, serial `…d63861dc2e50ab53`, firmware `31b25a44`
(API 1.13), hardware r10. `hackrf_info` reports *"Hardware does not appear to
have been manufactured by Great Scott Gadgets"* — it is a clone, which matters
only in that its amplitude has no more claim to being calibrated than the
E310's.

## Why it is in the tree

Not as a replacement for `sweepd`. §1 exists because a HackRF needs ~750 ms for
a full sweep where fastlock needs milliseconds, and the measurement below
confirms that number rather than beating it. It is here as a second reference
source — like §2's LibreSDR — and because it is the only source in the tree that
can genuinely reconfigure while running, which is what §6's soft keys were
written against and what `sweepd` still refuses.

## The block layout, measured

`hackrf_init_sweep` is called with `step_width = fs`, `offset = 3·fs/8`,
`style = INTERLEAVED`, one 16384-byte block per tuning.

Observed over 100–180 MHz at 20 MSPS, the block header frequencies within one
sweep are

```
100, 105, 120, 125, 140, 145, 160, 165   MHz
```

— two blocks per 20 MHz step, the second offset by `fs/4`. Strictly ascending,
then it wraps, which is how a completed sweep is detected.

A block whose header reads `F` was tuned with its LO at `F + 3·fs/8`. Taking
`Fft::power_spectrum`'s fftshifted output (DC at `N/2`), the two usable
quarter-bands either side of the LO's own leakage are

| shifted bins   | frequencies            | at fs = 20 MHz |
|----------------|------------------------|----------------|
| `[N/8, 3N/8)`  | `[F, F + fs/4)`        | `[F, F+5)` MHz |
| `[5N/8, 7N/8)` | `[F + fs/2, F+3fs/4)`  | `[F+10, F+15)` MHz |

The discarded middle is ±`fs/8` around the LO — DC, where direct conversion puts
the leakage §5 blanks. The discarded outside is the analog filter's skirt. The
interleaved partner block fills the middle back in, so the pair tiles the step
with no gaps and each one's LO spike is covered by the other's measured data.
That is §5's splice reached by a different route, which is why `stitch.h` is
shared with `sweepd` rather than reimplemented.

**This layout is not hardcoded.** The first sweep after every (re)configuration
is a *discovery* sweep: header frequencies are collected, nothing is
transformed, and the grid, covered range and tuning count come from what
actually arrived. A firmware that interleaves differently produces a correct
grid instead of a plausible trace half a band out.

Confirmation that it tiles: `hackrf_probe` reports **0 uncovered bins of 65536**
at full span and **0 of 2730** over 88–108 MHz. A geometry error anywhere would
show as NaN holes there.

## Sweep rate

| span | tunings | sweep time | rate |
|------|---------|-----------|------|
| 88–108 MHz | 2 | 2.4 ms | 410 sweeps/s |
| 1200–1280 MHz | 8 | 9.7 ms | 103 sweeps/s |
| 70 MHz – 6 GHz | 594 | **729 ms** | 1.37 sweeps/s |

**816–825 tunings/s in every case**, which is the honest figure: the rate is set
by the per-tuning cost and nothing else. Each tuning captures 8192 samples
(410 µs at 20 MSPS) and costs ~1.22 ms, so ~810 µs per tuning is retune and USB
overhead — twice the time is spent moving as listening.

The 729 ms full-span figure is CLAUDE.md §1's "~750 ms a HackRF needs",
measured. That is the comparison the whole project is against.

Measure the rate over many sweeps. Blocks arrive in 16-block USB transfers, so
over two or three sweeps of a narrow span the readout measures a burst and
reports an impossible number (6867 sweeps/s was seen over 3 sweeps of a 2-tuning
span; the same span settles to 410 over 300).

## Amplitude, cross-checked against `hackrf_sweep`

Same band (1200–1280 MHz), same gains (LNA 16, VGA 20, no RF amp), noise floor
only — no antenna attached.

| | mean floor |
|---|---|
| `hackrf_sweep -w 100000` | −76.68 dB |
| `hackrf_probe` (N=4096, Hann) | −82.13 dBFS |
| difference | **−5.45 dB** |

Predicted from the two known differences:

- **+6.02 dB.** We normalise by `S1 = Σw ≈ N/2`, so a full-scale complex
  sinusoid reads 0 dBFS (`dsp.h`). `hackrf_sweep` divides by `N`, so its scale
  is 6 dB low and its numbers are not dBFS in any convention.
- **−11.35 dB.** Our RBW is 7326 Hz against its 100 kHz, and noise power per bin
  is proportional to RBW.

Net **−5.33 dB** predicted against **−5.45 dB** measured — a 0.12 dB agreement.
The scale is right, and it is right for the reason claimed rather than by
coincidence.

This says nothing about absolute accuracy. There is no preselector and no
correction file, so the trace is dBFS and UNCAL is lit, exactly as with
`sweepd`. §7's amplitude correction applies here unchanged.

## Baseband filter

Default is the nearest valid filter to `fs` — 20 MHz at 20 MSPS — where
`hackrf_sweep` takes the widest filter *strictly below* `fs`, 15 MHz. The kept
slices reach 3·fs/8 = 7.5 MHz from the LO, which is exactly a 15 MHz filter's
−3 dB corner, so that choice puts a scallop at every 5 MHz seam. 20 MHz leaves
the kept edge at 0.75 of the corner, at the cost of less rejection of the alias
folding onto that edge from 12.5 MHz out. Both are defensible; `--hackrf-bw`
is the knob.

## Limits, and which are the firmware's

- **FFT size caps at 4096.** One tuning delivers one 16384-byte block, 10 bytes
  of it header, leaving 8187 I/Q pairs — 4096 is the largest power of two that
  fits. This is the block format, not a choice. Finer RBW comes from lowering
  `--hackrf-rate`, which costs sweep time proportionally.
- **Spans are whole megahertz.** The firmware's range list is `uint16` MHz. The
  sweep rounds outward and the output grid is clipped back to the request.
- **No segment overlay.** §7's overlay reconstructs a *uniform* LO grid, and the
  interleaved sweep's LOs are not uniform — at 20 MSPS they sit at 107.5, 112.5,
  127.5, 132.5 MHz, alternately 5 and 15 MHz apart. The seams *are* uniform
  (every 5 MHz) and it is tempting to report those as centres, but a centre in
  that overlay is where the LO leakage is and the seams are precisely where it
  is not. Reporting them would attribute the artifact to the wrong frequency,
  which is the failure §7 exists to prevent, so the plan fields are left zero and
  the overlay says it is unavailable. Teaching `SegmentPlanInfo` about paired
  LOs is the fix.
- **No zero span, no image check.** Both refuse with the reason. Zero span needs
  the non-sweep RX path (`hackrf_start_rx` with the LO parked), which is a
  separate streaming mode; the image check needs the LO grid phased by a
  fraction of a megahertz, which the range list cannot express.

## Checking it

`hackrf_probe` drives the source with no display and prints the trace as
numbers, the way `tools/sweep_client` does for the TCP source:

```
hackrf_probe --start 88m --stop 108m --sweeps 300 --peaks 12
hackrf_probe --start 70m --stop 6g  --sweeps 6 --csv full.csv
```

The line worth reading first is `uncovered N of M bins`. It should be 0.
