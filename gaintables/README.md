# RX gain tables for `sweepd --gain-table`

`sweepd` loads `adi_1300_4000_wide.txt` by default; CMake compiles it in, so
changing the default means editing that file and rebuilding. `--gain-table
stock` (or "driver's own" in `sweepui`) leaves the driver's own per-band tables
alone. `--gain-table FILE` loads any other table. Why this table, and why gain
29 against 30 matters more than which table:
`docs/characterization.md`, "A built-in single table".

Files in the AD936x driver's `gain_table_config` format. Load one with
`sweepd --gain-table FILE`, or from `sweepui` (Amplitude → gain table). The
reason to want one is in `src/sweepd/gaintable.h`: a single table over the
whole tuner keeps the RX gain index constant across bands a fastlock sweep
crosses without the driver re-selecting anything.

| file | what | in git |
|---|---|---|
| `adi_1300_4000_wide.txt` | **sweepd's default, compiled in.** ADI's 1.3–4 GHz full table from their no-OS driver, every value unchanged, band widened to 0–6 GHz. BSD-3-Clause: `LICENSE-adi-no-OS.txt`. `--gain-table builtin`. | yes |
| `e310_tezuka_stock.txt` | The driver's own three-band table (0–1.3, 1.3–4, 4–6 GHz), read off the E310 under Tezuka v0.3.24 on 2026-10-21. What `stock` puts back. | yes |
| `satsagen_ad9361_cgt.txt` | SATSAGEN 0.9.5.1's `ad9361_CGT`: one band, 0–6 GHz, 0–79 dB. | **no** |

The SATSAGEN file is not in the repository. SATSAGEN's licence forbids
redistributing or deriving from any part of it, so the copy here stays local
(`.gitignore`). To recreate it, unpack `satsagen_<ver>_setup.exe` with 7-Zip
(InstallShield → MSI → `Data1.cab`) and copy `ad9361_CGT1` here under that
name.

The driver refuses writes over 4096 bytes. `sweepd` compacts a larger table
(spaces after commas, `0x00` → `0`) without changing a value and refuses one
that still does not fit. The stock list is 5075 bytes as the driver prints it
and 3608 compacted.
