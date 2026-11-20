#pragma once

// libhackrf, loaded at run time rather than linked.
//
// A released sweepui has to start on a machine with no libhackrf at all, and
// --hackrf there should fail with a sentence that says what to install rather
// than the loader refusing to start the program. Loading it on demand also
// keeps libusb, which libhackrf pulls in and which is LGPL, out of sweepui's
// link: the binary release ships both as separate, replaceable libraries.
//
// The header is still needed at build time, for the types and for decltype on
// each function, so the signatures cannot drift from the library's.

#include <string>

#include <libhackrf/hackrf.h>

namespace sweepui {
namespace hackrf_dl {

struct Api {
    decltype(&::hackrf_init) init;
    decltype(&::hackrf_exit) exit;
    decltype(&::hackrf_open) open;
    decltype(&::hackrf_open_by_serial) open_by_serial;
    decltype(&::hackrf_close) close;
    decltype(&::hackrf_error_name) error_name;
    decltype(&::hackrf_board_id_read) board_id_read;
    decltype(&::hackrf_board_id_name) board_id_name;
    decltype(&::hackrf_board_partid_serialno_read) board_partid_serialno_read;
    decltype(&::hackrf_version_string_read) version_string_read;
    decltype(&::hackrf_usb_api_version_read) usb_api_version_read;
    decltype(&::hackrf_set_sample_rate) set_sample_rate;
    decltype(&::hackrf_compute_baseband_filter_bw) compute_baseband_filter_bw;
    decltype(&::hackrf_set_baseband_filter_bandwidth) set_baseband_filter_bandwidth;
    decltype(&::hackrf_set_lna_gain) set_lna_gain;
    decltype(&::hackrf_set_vga_gain) set_vga_gain;
    decltype(&::hackrf_set_amp_enable) set_amp_enable;
    decltype(&::hackrf_init_sweep) init_sweep;
    decltype(&::hackrf_start_rx_sweep) start_rx_sweep;
    decltype(&::hackrf_stop_rx) stop_rx;
};

// The loaded library, or null with *err saying what is missing and how to get
// it. Looks next to the executable first (where a binary release puts it),
// then wherever the system's loader looks. Loads once; later calls are free.
const Api *load(std::string *err);

}  // namespace hackrf_dl
}  // namespace sweepui
