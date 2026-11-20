# Amplitude correction

CLAUDE.md §7 requires the amplitude correction to be documented, not just
implemented: *"load `(freq_hz, offset_db)` CSV, interpolate, apply. Document how
to generate one against a known reference."* This is that document.

Until a correction is loaded the instrument reads **dBFS**, the **UNCAL** badge
is lit, and every export says so in its header. That is the honest default and
nothing here changes it automatically.

The loader is `src/sweepui/ampcorr.{h,cpp}`. The generator is `tools/ampcal`,
which runs on the device and produces the file from the AD936x's own DDS over a
TX→RX cable.

---

## What the correction is, and what it is not

`sweepsa` measures the level at the ADC, in dBFS — a number about the *receiver*,
not about the antenna connector. Between the connector and the ADC sit a balun,
a matching network, cabling, the AD936x front end and a gain stage, and their
combined response is a function of frequency that nobody has characterised for
your board. The correction file is that function, measured by you, for your
installation.

Consequently:

- **It is per-installation and it is yours.** It depends on your cabling, your
  gain setting and your board. Nobody else's file is valid for your hardware,
  and this repository deliberately ships none.
- **It is not a calibration certificate.** There is no preselector, no step
  attenuator and no factory cal (CLAUDE.md §1). A correction makes the numbers
  *traceable to your reference* and nothing more.
- **It is invalidated by anything that changes the gain** — and, on this
  instrument, by anything that changes the *span*. See "Conditions" below.

`sweepsa` takes whatever reference you use at face value. If you have a
calibrated signal generator or a calibrated spectrum analyser, the file is as
good as that instrument. If you have a cheap generator and a guess, the file is
as good as that. The tool does not rank your equipment; it applies your numbers.

---

## Absolute or relative: the thing to settle first

The correction is arithmetic on one equation:

```
offset_db(f) = reference_dbm(f) − measured_dbfs(f)
```

`measured_dbfs(f)` is what the instrument reads when a tone of known level is
applied. `reference_dbm(f)` is **that known level, at the RX connector**. The
whole question is whether you know it.

**If you know it** — a calibrated generator, a source you have characterised
with a VNA or a power meter — the offsets are powers and the file is
**absolute**. Loading it puts `sweepui` into dBm and extinguishes UNCAL. This is
the file §7 is asking for.

**If you do not** — and on this bench the only source is the AD936x's own
transmitter through a coax loop — then what you measured is the **round trip**:
the transmitter's frequency response, times the cable's, times the receiver's.
Dividing that out flattens the trace, which is genuinely useful, but the result
is a *shape with an arbitrary constant in it*, not a power. That file is
**relative**. `sweepui` applies it in full and keeps UNCAL lit, and the Amplitude
group says why.

`ampcal` writes the distinction into the file so the two can never be confused
later:

```
# sweepsa-ampcorr: relative
```

There is no honest way to promote one to the other after the fact. A relative
file plus a measured level at one frequency *would* be enough — that is exactly
what `--ref-dbm` does — but the level has to be measured, not assumed.

> A relative correction still buys the two things most of this instrument's use
> needs: a flat baseline, so a signal's shape is not the receiver's shape, and
> valid **differences** between two points on the trace. It does not buy an
> absolute number, and it never claims to.

---

## File format

Two columns, `freq_hz,offset_db`. The offset is **added** to the measured dBFS
to give dBm:

```
dBm(f) = dBFS(f) + offset_db(f)
```

So an offset is normally *negative* when the receiver has gain: 30 dB of gain
ahead of the ADC means a −40 dBm input reads about −10 dBFS, and the offset that
recovers the input level is about −30 dB.

```csv
# sweepsa amplitude correction
# sweepsa-ampcorr: absolute
# sweepsa-ampcorr-gain-db: 30
freq_hz,offset_db
70000000,-28.4
100000000,-28.9
500000000,-30.2
1000000000,-31.6
2000000000,-33.9
3000000000,-36.8
6000000000,-42.1
```

The parser accepts, because these are things real files have:

- `#` comments and blank lines
- a header row (one, before any data)
- CRLF line endings and a UTF-8 BOM
- comma **or** whitespace separation
- extra columns after the first two — a date, a note — which are ignored
- rows in any order; they are sorted on load

It **refuses**, loudly, rather than half-understanding:

- fewer than two distinct frequencies — a single number is a reference-level
  offset, not a correction curve, and applying it everywhere is exactly the
  extrapolation §7 exists to prevent
- a non-positive frequency, which is almost always swapped columns
- two different offsets for the same frequency
- any line that is not a number pair
- a known `# sweepsa-ampcorr-*` directive with an unreadable value

A file that fails to parse leaves the previous correction in place. A bad file
must not quietly de-calibrate a working session.

### The directives

Optional `#` comment lines the loader reads. Every other reader of the file — a
spreadsheet, a plotting script, an older build of this program — still sees
exactly two columns and is unaffected.

| directive | meaning |
|---|---|
| `# sweepsa-ampcorr: absolute\|relative` | what the offsets mean. **Absent is read as `absolute`**, which is what a hand-written §7 file intends. |
| `# sweepsa-ampcorr-gain-db: 30` | the RX gain it was measured at |
| `# sweepsa-ampcorr-span-hz: 70000000 6000000000` | the span it was measured over |
| `# sweepsa-ampcorr-detector: peak` | the detector it was measured with |
| `# sweepsa-ampcorr-adc-hz: 491520000` | the ADC clock behind the sample rate. The rate governor changes it, and the gain moves ~0.5 dB with it (characterization.md, "The pedestal follows the ADC clock") |
| `# sweepsa-ampcorr-note: free text` | shown verbatim in the Amplitude group |

An **unknown** directive key is ignored, so a future `ampcal` writing a seventh
line does not make its files unloadable by this build.

`ampcal` always writes the kind explicitly, so its own output is never read by
the absent-means-absolute fallback. **The cost of using comments is that a round
trip through a spreadsheet usually drops them.** If you edit a relative
correction in a spreadsheet, put the first line back.

### Conditions

A correction is only valid under the conditions it was measured under, and the
last three directives are there so `sweepui` can check them against the live
sweep and say when they differ.

| condition | why it matters |
|---|---|
| RX gain | the whole curve shifts with it |
| **span** | a different span is a different segment grid, so a different set of fastlock predecessors — up to 6.3 dB (`characterization.md`, "Not preserved: amplitude") |
| detector | a peak-detected bin and an RMS-detected bin differ by a decimation-dependent amount |

None of the three is *enforced*. The operator may have a reason, and a refusal
would be worse than the error it prevents. They are reported, in the Amplitude
group, next to the file's name.

---

## Generating one with `tools/ampcal`

### What it has to be measured *through*

`ampcal` has two modes and the default is the non-obvious one.

**`--via sweep` (default)** — `ampcal` drives only the transmitter. The level is
read off a running `sweepd`'s 0x02 frames over TCP, exactly as `sweepui` would
read them.

This is deliberate. The level after a `fastlock_recall` depends on **where the
LO was parked beforehand**, over a 6.3 dB spread, repeatable to ±0.04 dB, and
the recorded consequence is that "the offset becomes a fixed per-segment
constant that the per-installation amplitude correction absorbs" — which is only
true if the correction is measured through the same recalls, in the same order,
that the instrument will use. Measuring through the sweep also absorbs, for
free, everything else the sweep does to amplitude: the splice comb at segment
centres, the detector's decimation bias, the window's scalloping, the RX port
the cold build chose.

**Measured, 2026-09-12** (full detail in `characterization.md`, "The amplitude
correction has to be measured through the sweep") — the same grid, both ways,
905–1095 MHz in 5 MHz steps, 11 segments, peak detector, 30 dB gain:

| | |
|---|---|
| smooth response, seen by **both** modes | **7.2 dB** across 190 MHz |
| fine structure seen by `sweep` and **not** by `direct` | **3.4 dB peak-to-peak**, sd 0.59 dB |
| run-to-run repeatability of the `sweep` curve | **sd 0.065 dB**, worst point 0.23 dB |

So a correction taken by writing the LO directly would be a clean measurement of
a path the instrument never takes, and would leave **3.4 dB** of
segment-to-segment error uncorrected while looking like a calibration. The
residual repeats to 0.07 dB, so it is a correctable systematic, not noise.

**`--via direct`** — an ordinary LO write, then a capture straight from libiio.
No fastlock, no splice, no detector. This measures **the front end alone**,
which makes it the right tool for §4a.1's sensitivity-vs-frequency curve and the
wrong one for a correction that will be applied to swept traces. It needs the
DMA, so `sweepd` must be stopped first.

### Safety

`ampcal` drives a transmitter into a receiver input over a cable with no pad.
The AD936x RX absolute maximum is about **+2.5 dBm**; the TX reaches about
**+5 dBm** at minimum attenuation. `ampcal` and `toneg` both refuse an
attenuation below 30 dB without `--i-have-a-pad`, and both silence the
transmitter on the way out, `ampcal` including on Ctrl-C. After a `kill -9`, the
DDS lives in the chip and keeps transmitting: run `toneg --off`.

### 1. Start the daemon on the span you intend to use

The correction will carry this span, so sweep the span you actually want
corrected.

```sh
ssh e310 '/tmp/sweepd --no-csv --listen 0.0.0.0:5556 --start 900m --stop 1100m \
            --points 4096 --gain 30'
```

### 2. Find the attenuation

Run a short grid with `-v` and read the `snr` column. Aim for **20–40 dB above
the local floor** and no `OVERLOAD`. `--atten` is attenuation, so a **smaller
number is louder**.

```sh
ssh e310 '/tmp/ampcal --start 910m --stop 1090m --step 20m -v'
```

```
    910.000 MHz    -44.22 dBFS  floor   -83.17  snr  38.95
    930.000 MHz    -44.26 dBFS  floor   -83.31  snr  39.05
    ...
```

60 dB of attenuation gave 39 dB of SNR on this bench, which is comfortable.

### 3. Take the measurement

```sh
ssh e310 '/tmp/ampcal --step 5m --avg 6 --out /tmp/correction.csv'   # ~7 s
scp -O e310:/tmp/correction.csv .
```

**Leave `--start` and `--stop` off.** In sweep mode `ampcal` defaults the grid to
the daemon's own span, and that is not a convenience: the correction is never
extrapolated, so a grid that stops one step short of either edge produces a file
that loads, applies, and still leaves UNCAL lit — with nothing at measurement
time to suggest why. Give `--start`/`--stop` only when you deliberately want a
sub-range, and expect the units to stay dBFS if you do; `ampcal` says so before
it starts.

Two things it does at the edges, both reported when they happen:

- the last grid step rarely divides the range exactly, so the top edge is added
  as an extra point rather than being left a step short;
- the synthesizer cannot land exactly on the span edge — asking for 1100 MHz
  gave 1099999763 Hz here — and coverage is an exact test, so the two end rows
  are **snapped onto the span edges** when they are within 10 kHz. That moves a
  *label* by a few hundred hertz, three orders of magnitude inside one 48.8 kHz
  output bin. Every other row keeps the frequency the hardware actually produced.

`--avg` sweeps are averaged per point, in **linear power** (§5's RMS rule), and
`--settle` sweeps are discarded after each retune because the sweeps already in
flight were taken with the tone somewhere else.

Grid spacing is a judgement call, not a constant: the file is linearly
interpolated between points, so the grid has to be fine enough that the response
is straight between neighbours. 5 MHz resolves the per-segment structure at an
18 MHz segment pitch; 25 MHz (the default) would average across it and produce a
smooth curve that is wrong everywhere.

### 4. Check it repeats

Run it twice and difference the two files. The run-to-run spread is the floor on
what the correction can be worth. On this bench it is **0.065 dB sd against a
7.5 dB curve** — a 100:1 return.

### 5. Watch the overload flag

An overloaded front end produces convincing fake levels, and a clipped reading
is *compressed*, so an overloaded point yields an offset that is too small — a
correction that quietly under-corrects. `ampcal` warns; the status bar and the
frame's `overload_mask` report it during normal use.

---

## Generating one by hand

If you have a proper reference instrument, you do not need `ampcal` at all. The
procedure is the same whichever instrument it is.

1. **Fix the gain and leave it fixed.** AGC is off by design (§5). Note the RX
   gain you used and put it in `# sweepsa-ampcorr-gain-db:` — it is part of what
   the file means.
2. **Connect the reference to the RX port through the cabling you will actually
   use.** The cable is part of what you are measuring. If you calibrate through
   a short SMA lead and then run through 3 m of RG-58, the file is wrong by the
   loss of that cable.
3. **At each frequency `f`:** set the reference to a known level `P_ref` dBm —
   well above the noise floor and well below overload, −40 dBm is usually a good
   choice — park `sweepsa` on it and read the marker level `P_meas` in dBFS.
   Use the **flat-top** window: it is the amplitude-accurate one (§5), and the
   difference from Hann on a tone is not negligible when the whole point is
   amplitude.
4. **Record `offset_db = P_ref − P_meas`.**
5. **Step and repeat.** Space the points closely enough that linear
   interpolation between them is defensible. The response is smooth over most of
   the range and changes fastest at band edges and near filter corners, so
   concentrate points there rather than spacing them uniformly.
6. **Record the span and detector too**, in the directives, and re-read
   "Conditions" above. A hand-made file measured by parking on each frequency is
   *not* measured through the sweep, so it does not absorb the per-segment
   fastlock offsets — 3.4 dB peak-to-peak on this bench.

A zero-span parked read is the steadiest way to take step 3, because the trace
is a time series at one frequency and the marker is not chasing a sweep.

### Coverage matters more than point count

The correction is **never extrapolated**. Outside the range the file measured,
bins are left uncorrected and the units stay dBFS with UNCAL lit, however many
points the file has inside its range.

This is deliberate. A front end rolls off hardest exactly where a measurement
sweep tends to stop, so clamping to the nearest endpoint would apply the largest
correction precisely where it is least justified — and it would produce a
confident dBm number for a frequency nobody measured.

So: **make the file span at least the widest span you intend to read in dBm.**
If you calibrate 900 MHz–1.1 GHz and then view full span, the instrument will
correctly refuse to call it dBm and will tell you what fraction of the span is
uncovered.

---

## Making a loopback correction absolute

You need to know what the source delivers **to the RX connector**. Two routes.

### A flat source — `--ref-dbm`

If the level is flat across the grid, or flat enough, measure it once with a
power meter or a VNA at the end of the cable that would otherwise plug into RX1,
and pass it:

```sh
ampcal --step 5m --ref-dbm -45.0 --out correction.csv
```

The file is then absolute and `sweepui` reads dBm.

**On this bench a TX→RX loopback is not flat** — that is the 7.2 dB the
measurement above found, and most of it is the transmitter. So `--ref-dbm`
against the on-chip TX is only honest if you have *first* characterised the TX
output across the grid, which is the next section.

### A characterised source — `--ref-csv`

Measure the source's output level at each grid frequency and write a
`freq_hz,dbm` table. Same two-column format, same rules — interpolated between
points, never extrapolated past the ends.

With the owner's **LibreVNA**:

1. Disconnect the TX→RX cable at the RX end and put it into the VNA's port.
2. Step `toneg` across the same grid the correction will use, at the **same
   `--atten` and `--scale`** `ampcal` will use, and record the level the VNA
   reads at each point.
3. Write those to `txlevel.csv` as `freq_hz,dbm`.
4. Reconnect the cable to RX1 and run `ampcal --ref-csv txlevel.csv --step 5m`.

The result is absolute to whatever the VNA's own accuracy is, plus the
repeatability of re-seating the connector — which is the honest limit and is
worth writing into `--note`.

Anything that changes the source level between step 2 and step 4 — a different
`--atten`, a different cable, a different connector torque — invalidates it.
That is why `ampcal` records the attenuation and the tone offset in the file's
note automatically.

**This has not been done yet.** Every correction taken on this bench so far is
relative.

---

## Reading `ampcal`'s output

```
# CLAUDE.md §7 amplitude correction, generated by tools/ampcal
# sweepsa-ampcorr: relative
# RELATIVE: measured against a source of unknown level, so these
# offsets flatten the frequency response and carry no reference.
# dBFS + offset is NOT dBm. sweepui keeps UNCAL lit for this file.
# sweepsa-ampcorr-gain-db: 30.00
# sweepsa-ampcorr-span-hz: 900000000 1100000000
# sweepsa-ampcorr-detector: peak
# sweepsa-ampcorr-note: via sweep, 41 of 41 grid points usable, TX atten 60.00 dB, tone offset 5.000 MHz
#
# freq_hz,offset_db
900000000,-0.332
904999763,-0.363
...
```

The frequencies are the ones the hardware **actually produced** — the driver
coerces the LO and the DDS quantises its offset, and `ampcal` reads both back
rather than indexing the table by what it asked for. The two end rows are the
documented exception.

Points where the tone was not found, or was under `--min-snr` of the local
floor, are **omitted rather than guessed**. A gap in the middle is interpolated
across; a gap at either end shortens the corrected range. `ampcal` says how many
it dropped.

---

## Using it

```sh
# at startup
sweepui --connect 192.168.1.10:5556 --ampcorr my-e310-30db.csv

# or from the Amplitude soft-key group: type the path, press "load correction"

# or from a script, through the same methods the soft keys call
sweepui --do "ampcorr my-e310-30db.csv"
sweepui --do "ampcorr off"
```

When an **absolute** file that spans the sweep loads you should see, together:

- the trace y-axis relabel from `dBFS` to `dBm`
- the **UNCAL** badge go out in the status bar
- `units: dBm` in the Amplitude group, with the file's kind, point count and range
- exported CSV headers change to `# units=dBm uncal=false`

If UNCAL stays lit with a file loaded, the Amplitude group says which of the two
reasons it is: the file is **relative**, or the swept span reaches outside it.
The exported CSV header carries the same sentence, so a file read later cannot
be mistaken for an uncorrected one.

**The SYNTHETIC badge is a separate claim and is unaffected.** Loading a
correction over the synthetic source will relabel the axis to dBm, because the
arithmetic is real; the data is still fabricated and still says so. Nothing in
the correction path makes invented numbers into measurements.

### It clears held traces

Loading or unloading a correction clears the held and averaged traces, for the
same reason a retune does (§10.5): a bin corrected by −31 dB and the same bin
uncorrected are not two measurements of one thing, and a Max Hold spanning the
change would be a picture of neither.

---

## Where it is applied, and why there

In `sweepui`, at the single point a frame enters the display pipeline, so the
waterfall, all four traces, their holds and averages, the markers and the CSV
export all read the same corrected array. `src/sweepui/ampcorr.h` carries the
full argument; the short version is three points:

1. **The device cannot hold the file.** The E310's rootfs is a RAM initramfs and
   is wiped on every boot (§2). A calibration that has to be re-uploaded every
   power cycle is not a calibration.
2. **The wire stays honest.** `proto.h` says the 0x02 payload is dBFS. If the
   daemon corrected, the same frame type would carry dBFS sometimes and dBm
   other times with nothing in the header to say which, and `sweep_client`'s CSV
   would silently change meaning.
3. **One ingest point means no second path to drift out of sync** — the same
   argument zero span made for riding the 0x02 frame rather than inventing 0x03.

`sweepd` therefore reports `uncal: true` always, and always will. That is not a
placeholder: it is the daemon truthfully saying it has applied no correction.

---

## Verification

`tests/ampcorr_test.cpp` pins the arithmetic, the parser and the directives,
including the traps that produce a plausible number rather than an error:
extrapolation past the measured range, indexing a zero-span *time* axis as if it
were frequency, and a relative file being read as dBm.

### Against the synthetic source

900–1100 MHz, 4096 points, by exporting CSV with and without a correction and
taking the median shift per bin over 4096 bins:

| correction file | region | measured shift | expected |
|---|---|---|---|
| flat −20 dB, 880–1120 MHz | whole span | −20.03 dB | −20.00 |
| sloped −10→−30 dB, 880–1120 MHz | 900–950 MHz | −13.70 dB | −13.75 |
| sloped −10→−30 dB, 880–1120 MHz | 1050–1100 MHz | −26.17 dB | −26.25 |
| narrow −20 dB, **980–1020 MHz only** | inside | −20.02 dB | −20.00 |
| narrow −20 dB, **980–1020 MHz only** | below 975 MHz | −0.01 dB | **0.00** |
| narrow −20 dB, **980–1020 MHz only** | above 1025 MHz | −0.03 dB | **0.00** |

The residuals are the median estimator over different noise realisations. The
last two rows are the ones that matter: they are the no-extrapolation rule
holding to within 0.03 dB.

The units followed correctly in all three cases — `dBm`/`uncal=false` for the two
files that span the sweep, `dBFS`/`uncal=true` for the narrow one, which also
reported `3276 of 4096 bins (80%) are outside the correction`.

### Against the radio — 2026-09-12

900–1100 MHz, 4096 points, 11 segments, files from `ampcal`, exported CSV
headers:

| file loaded | header |
|---|---|
| none | `units=dBFS uncal=true` — *no amplitude correction is loaded* |
| relative, spans the sweep | `units=dBFS uncal=true` — *the correction is RELATIVE, so the frequency response is removed but there is no reference level* |
| absolute, 905–1095 on a 900–1100 sweep | `units=dBFS uncal=true` — *the correction does not span this trace* |
| absolute, spans the sweep | **`units=dBm uncal=false`** |

That last row is the first time this instrument has reached dBm against real
hardware. Retuning the daemon to 950–1050 MHz with that file still loaded raises,
in the Amplitude group: *"correction conditions differ from this sweep: measured
over 900 – 1100 MHz, sweeping 950 – 1050 MHz (different hop order)"*.

---

## What this does not fix

- **It is not a preselector.** An image response is an image response whether or
  not its amplitude is corrected. §7's image check is the separate feature.
- **It does not survive a span change**, a gain change or a detector change.
  Those are reported, not corrected.
- **Temperature is not in it.** CLAUDE.md's own list of what bounds this
  instrument includes SDR temperature. A correction taken cold and applied hot
  is wrong by an amount nothing here measures.
- **A relative file is not dBm** and no amount of covering the span makes it so.
