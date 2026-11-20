// A custom AD936x RX gain table, loaded through the driver's
// `gain_table_config` attribute.
//
// Why load one at all. The driver keeps one manual-gain table per RF band
// (stock: 0-1.3, 1.3-4 and 4-6 GHz) and a real LO write re-selects the table
// and rewrites the gain index in register 0x109. fastlock_recall does neither,
// so a sweep runs every recalled segment on whatever table and index the last
// real LO write left behind -- the 7 dB cliff at 3 GHz of 2026-09-13
// (docs/characterization.md, "The retune had a side effect") and the
// register-restore workaround in IioSweepDevice::retune both come from that.
// One table covering the whole tuning range has no band boundary to cross:
// measured 2026-10-21, with SATSAGEN's `ad9361_CGT` loaded, a 30 dB request
// sits on index 0x1E at every LO from 70 MHz to 5.5 GHz, where the stock tables
// move it 0x21 -> 0x23 -> 0x2C (docs/characterization.md, "One gain table").
//
// The price is that the stock tables are per band for a reason: they absorb
// the LNA's gain change with frequency, so "30 dB" means roughly the same
// thing everywhere. A single table makes the index constant and lets the
// analog gain drift smoothly with frequency instead -- a slope, which §7's
// amplitude correction removes by construction, where a step it cannot.
//
// Nothing here touches a device. The write and the stock snapshot are in
// device.{h,cpp}; this is the text, so tests/gaintable_test.cpp can pin it.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sweepd {

// The attribute refuses a write longer than this with EFBIG, and takes the
// whole table in one write() call: measured 2026-10-21 on Tezuka v0.3.24
// (docs/characterization.md, "One gain table"). The stock three-band list is
// 5075 bytes as the driver prints it, so even the driver's own tables need
// compacting before they can be written back.
constexpr size_t kGainTableMaxBytes = 4096;

struct GainTableBand {
    long long start_hz = 0, end_hz = 0;
    int dest = 0;
    int n_rows = 0;
    int gain_min_db = 0, gain_max_db = 0;   // first column, as hardwaregain_available reports it
    std::vector<long> cells;                // n_rows x 4, row-major, for compare()
};

struct GainTableInfo {
    std::vector<GainTableBand> bands;
    // The gain every band can deliver: the intersection of their ranges. A
    // manual gain outside it would be clamped by the driver in some band and
    // not others, which is the step this exists to remove.
    int gain_lo_db = 0, gain_hi_db = 0;
    long long covers_lo_hz = 0, covers_hi_hz = 0;
};

// Parses the driver's format -- an optional <list> of
//   <gaintable AD9361 type=FULL dest=3 start=0 end=6000000000>
//   gain, lna, mixer_tia_lpf, dc_cal  (decimal or 0x hex)
//   </gaintable>
// -- and refuses anything else, so a wrong file is refused here with a line
// number rather than by the driver with EINVAL.
bool parse_gain_table(const std::string &text, GainTableInfo *out, std::string *err);

// Shrinks a table to fit kGainTableMaxBytes without changing a single value:
// drops the space after commas and spells 0x00 as 0 and 0x0N as 0xN. Returns
// the text unchanged when it already fits. The caller re-parses the result and
// compares it with same_cells(), so a transform that changed a value would be
// caught rather than written.
std::string compact_gain_table(const std::string &text);

// True when two parsed tables hold the same bands and the same numbers.
bool same_cells(const GainTableInfo &a, const GainTableInfo &b);

// Parse, compact if needed, check that compacting changed nothing, and check
// the size. On success *wire is what to write to the attribute.
bool prepare_gain_table(const std::string &text, std::string *wire, GainTableInfo *info,
                        std::string *err);

// For the profile-cache key: 0 means the driver's own tables.
uint64_t gain_table_hash(const std::string &wire);

// The table sweepd loads unless told otherwise: ADI's own 1.3-4 GHz full
// table, every register and dB value as in their no-OS driver, with its band
// widened to 0-6 GHz (gaintables/adi_1300_4000_wide.txt, compiled in by
// CMake; licence in gaintables/LICENSE-adi-no-OS.txt). Inside 1.3-4 GHz it is
// exactly the driver's own table, so "30 dB" there means what it always did;
// outside it the index stays put instead of jumping to another band's
// ladder. Why this one: docs/characterization.md, "A built-in single table".
const char *builtin_gain_table();
const char *builtin_gain_table_name();

}  // namespace sweepd
