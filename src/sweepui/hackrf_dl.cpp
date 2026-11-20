#include "hackrf_dl.h"

#include <mutex>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace sweepui {
namespace hackrf_dl {
namespace {

#if defined(_WIN32)
const char *const kNames[] = {"libhackrf.dll", "hackrf.dll"};
const char *const kInstall =
    "libhackrf.dll was not found next to sweepui.exe. Use a sweepsa release, "
    "which ships it, or copy libhackrf.dll and libusb-1.0.dll there.";
#elif defined(__APPLE__)
const char *const kNames[] = {"libhackrf.0.dylib", "/opt/homebrew/lib/libhackrf.0.dylib",
                              "/usr/local/lib/libhackrf.0.dylib"};
const char *const kInstall = "libhackrf was not found. Install it with: brew install hackrf";
#else
const char *const kNames[] = {"libhackrf.so.0", "libhackrf.so"};
const char *const kInstall =
    "libhackrf was not found. Install it with your package manager, e.g. "
    "sudo apt install libhackrf0 (Debian/Ubuntu) or sudo pacman -S hackrf (Arch).";
#endif

std::string exe_dir() {
#ifdef _WIN32
    char buf[MAX_PATH];
    const DWORD n = GetModuleFileNameA(nullptr, buf, MAX_PATH);
    std::string p(buf, n);
    const size_t s = p.find_last_of("\\/");
#else
    Dl_info info;
    if (!dladdr(reinterpret_cast<void *>(&exe_dir), &info) || !info.dli_fname) return "";
    std::string p(info.dli_fname);
    const size_t s = p.find_last_of('/');
#endif
    return s == std::string::npos ? "" : p.substr(0, s + 1);
}

void *open_lib(const std::string &path) {
#ifdef _WIN32
    return reinterpret_cast<void *>(LoadLibraryA(path.c_str()));
#else
    return dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
#endif
}

void *symbol(void *lib, const char *name) {
#ifdef _WIN32
    return reinterpret_cast<void *>(GetProcAddress(static_cast<HMODULE>(lib), name));
#else
    return dlsym(lib, name);
#endif
}

template <typename F>
bool bind(void *lib, F *slot, const char *name, std::string *missing) {
    *slot = reinterpret_cast<F>(symbol(lib, name));
    if (!*slot) *missing += std::string(missing->empty() ? "" : ", ") + name;
    return *slot != nullptr;
}

std::once_flag g_once;
Api g_api;
const Api *g_loaded = nullptr;
std::string g_err;

void load_once() {
    void *lib = nullptr;
    // A bare name: next to the executable first, which is where a release
    // puts it, then the system's search path.
    const std::string here = exe_dir();
    for (const char *n : kNames) {
        if (n[0] != '/' && !here.empty() && (lib = open_lib(here + n))) break;
        if ((lib = open_lib(n))) break;
    }
    if (!lib) {
        g_err = kInstall;
        return;
    }
    std::string missing;
#define BIND(field, name) bind(lib, &g_api.field, #name, &missing)
    BIND(init, hackrf_init);
    BIND(exit, hackrf_exit);
    BIND(open, hackrf_open);
    BIND(open_by_serial, hackrf_open_by_serial);
    BIND(close, hackrf_close);
    BIND(error_name, hackrf_error_name);
    BIND(board_id_read, hackrf_board_id_read);
    BIND(board_id_name, hackrf_board_id_name);
    BIND(board_partid_serialno_read, hackrf_board_partid_serialno_read);
    BIND(version_string_read, hackrf_version_string_read);
    BIND(usb_api_version_read, hackrf_usb_api_version_read);
    BIND(set_sample_rate, hackrf_set_sample_rate);
    BIND(compute_baseband_filter_bw, hackrf_compute_baseband_filter_bw);
    BIND(set_baseband_filter_bandwidth, hackrf_set_baseband_filter_bandwidth);
    BIND(set_lna_gain, hackrf_set_lna_gain);
    BIND(set_vga_gain, hackrf_set_vga_gain);
    BIND(set_amp_enable, hackrf_set_amp_enable);
    BIND(init_sweep, hackrf_init_sweep);
    BIND(start_rx_sweep, hackrf_start_rx_sweep);
    BIND(stop_rx, hackrf_stop_rx);
#undef BIND
    if (!missing.empty()) {
        // Too old to sweep: hackrf_init_sweep arrived with the 2017 releases.
        g_err = "the libhackrf found is too old; it lacks " + missing + ". Update it.";
        return;
    }
    g_loaded = &g_api;
}

}  // namespace

const Api *load(std::string *err) {
    std::call_once(g_once, load_once);
    if (!g_loaded && err) *err = g_err;
    return g_loaded;
}

}  // namespace hackrf_dl
}  // namespace sweepui
