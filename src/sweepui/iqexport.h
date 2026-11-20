// Saving a raw IQ burst (proto.h, "0x03 iq") in the formats the tools people
// already have will open. No ImGui and no GL, so tests/iqexport_test.cpp reads
// every file back.
//
//   .cfile        complex float32, interleaved, little-endian, no header: GNU
//                 Radio's File Source with type "complex", inspectrum, baudline.
//                 Carries no rate and no frequency, so the UI says them when it
//                 writes one.
//   .sc16 / .iq16 int16 I/Q pairs scaled so full scale is the burst's own
//                 full_scale (2048 for the AD936x): the ADC's own counts when
//                 the burst was not decimated. Values past +/-32767 clamp.
//   .csv          t_s, i, q, mag, phase_deg -- with the burst's settings in
//                 '#' comment lines first, because a column of numbers that has
//                 lost its sample rate is not a measurement any more.
//   .sigmf-meta / .sigmf-data / .sigmf
//                 a SigMF recording: cf32_le data plus JSON metadata with the
//                 rate, the centre frequency, the hardware string and the
//                 trigger as an annotation. The rate and the frequency travel
//                 with the samples, which is what the format is for.
//
// Every writer takes a sample range so "export between the cursors" is the same
// code as "export the burst".

#pragma once

#include <cstddef>
#include <string>

#include "proto.h"

namespace sweepui {

enum class IqFileKind { Cfile, Sc16, Csv, SigMF };

// From the extension; false if it is none of the above.
bool iq_kind_from_path(const std::string &path, IqFileKind *out);

// Writes samples [i0, i1) of `m` to `path`. `note` goes into the CSV header and
// the SigMF description -- the UI passes "SYNTHETIC" for a fabricated source,
// so a file from the fake cannot later pass for a capture. For SigMF, `path`
// may name either half or the bare ".sigmf"; both files are written next to
// each other and `*written` (if given) names the data file.
bool write_iq_file(const std::string &path, const sweepd::proto::IqMsg &m, size_t i0,
                   size_t i1, const std::string &note, std::string *err,
                   std::string *written = nullptr);

}  // namespace sweepui
