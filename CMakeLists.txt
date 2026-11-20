cmake_minimum_required(VERSION 3.16)
project(sweepsa C CXX)   # C is required: pffft and kissfft are C sources

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# Public tree: sweepui and the device-independent libraries it shares with
# sweepd (DSP, segment plan, wire protocol, IQ, generator, gain-table text).
# sweepd itself is not included; prebuilt binaries are on the Releases page.
# These libraries keep their sweepd_ names and src/sweepd/ location so they
# stay byte-identical to the daemon's copies.
#
# The DSP core has no device dependency, so it is built and tested natively.
# This is where §5's correctness traps get pinned down: derived RBW, and the
# RMS detector averaging power rather than dB.
# stitch.cpp lives here rather than with the sweep engine because it has no
# device dependency either, and because sweepui's HackRF source needs the same
# ownership rule without wanting the profile table that comes with
# sweepd_sweep. See src/sweepd/stitch.h.
add_library(sweepd_dsp STATIC
  src/sweepd/dsp.cpp
  src/sweepd/stitch.cpp
  third_party/pffft/pffft.c)
target_include_directories(sweepd_dsp PUBLIC src/sweepd third_party/pffft)
target_compile_options(sweepd_dsp PRIVATE -O2 -Wall -Wextra -Wno-psabi)

# The segment plan alone (§5): where each LO goes, what is kept, how the DC hole
# is covered. Pure arithmetic -- no POSIX, no device, no allocation beyond the
# segment vector -- because three things now need it and one of them is sweepui,
# which §6 requires build on Windows. See src/sweepd/plan.h for why it is not
# simply part of sweepd_profiles.
add_library(sweepd_plan STATIC src/sweepd/plan.cpp)
target_include_directories(sweepd_plan PUBLIC src/sweepd)
target_compile_options(sweepd_plan PRIVATE -O2 -Wall -Wextra -Wno-psabi)

# The §5 wire protocol: framing, the 0x02 trace frame and the flat JSON control
# channel. No libiio, no device, no sockets -- this is a contract between two
# binaries built in different phases and, per §6, on different operating
# systems, and it compiles into sweepui unchanged.
#
# Split from the server below on purpose: server.cpp is POSIX sockets and
# pthreads, which sweepui has no use for and Windows has no version of.
add_library(sweepd_proto STATIC src/sweepd/proto.cpp)
target_include_directories(sweepd_proto PUBLIC src/sweepd)
target_compile_options(sweepd_proto PRIVATE -O2 -Wall -Wextra -Wno-psabi)

# Raw IQ (the 0x03 burst): decimator, edge trigger and blind statistics. Pure,
# so sweepd, the synthetic source and the UI's readouts run the same code.
add_library(sweepd_iq STATIC src/sweepd/iq.cpp)
target_include_directories(sweepd_iq PUBLIC src/sweepd)
target_compile_options(sweepd_iq PRIVATE -O2 -Wall -Wextra -Wno-psabi)

# A custom AD936x gain table's text (src/sweepd/gaintable.h): parsing, the
# 4096-byte compaction and the cache-key hash. Pure, so sweepui checks a file
# with the same code before sending it, and on Windows.
# The built-in table, sweepd's default, is the text file below compiled in:
# edit or swap the file, not C++. Origin and licence in
# gaintables/LICENSE-adi-no-OS.txt; the measurement that chose it in
# docs/characterization.md, "A built-in single table".
set(GT_BUILTIN_FILE gaintables/adi_1300_4000_wide.txt)
get_filename_component(GT_BUILTIN_NAME ${GT_BUILTIN_FILE} NAME_WE)
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/${GT_BUILTIN_FILE} GT_BUILTIN_TEXT)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${GT_BUILTIN_FILE})
configure_file(src/sweepd/gaintable_builtin.cpp.in
               ${CMAKE_CURRENT_BINARY_DIR}/gaintable_builtin.cpp @ONLY)
add_library(sweepd_gaintable STATIC src/sweepd/gaintable.cpp
                                    ${CMAKE_CURRENT_BINARY_DIR}/gaintable_builtin.cpp)
target_include_directories(sweepd_gaintable PUBLIC src/sweepd)
target_compile_options(sweepd_gaintable PRIVATE -O2 -Wall -Wextra -Wno-psabi)

# The tracking generator's (SNA) plan arithmetic. Pure.
add_library(sweepd_trackplan STATIC src/sweepd/trackplan.cpp)
target_include_directories(sweepd_trackplan PUBLIC src/sweepd)
target_link_libraries(sweepd_trackplan PUBLIC sweepd_dsp)
target_compile_options(sweepd_trackplan PRIVATE -O2 -Wall -Wextra -Wno-psabi)

# The signal generator's waveforms and sweep arithmetic. Pure.
add_library(sweepd_gen STATIC src/sweepd/gen.cpp)
target_include_directories(sweepd_gen PUBLIC src/sweepd)
target_compile_options(sweepd_gen PRIVATE -O2 -Wall -Wextra -Wno-psabi)

  enable_testing()

  # The parts of sweepui with no ImGui, GL or window-system dependency: the
  # frequency-entry convention, the display decimator, the synthetic source and
  # the §10.4 TCP source. Separated so they are tested natively and so the
  # pieces most likely to be silently wrong -- decimation, unit parsing and now
  # the wire decode -- are not locked inside a binary that needs a display to
  # run. netsource.cpp carries its own socket shim rather than linking
  # sweepd_server, which is POSIX-only and which sweepui must not need in order
  # to build on Windows (§6).
  add_library(sweepui_core STATIC
    src/sweepui/freq.cpp
    src/sweepui/decimate.cpp
    src/sweepui/trace.cpp
    src/sweepui/marker.cpp
    src/sweepui/export.cpp
    src/sweepui/iqexport.cpp
    src/sweepui/ampcorr.cpp
    src/sweepui/tracknorm.cpp
    src/sweepui/persist.cpp
    src/sweepui/measure.cpp
    src/sweepui/nf.cpp
    src/sweepui/segments.cpp
    src/sweepui/imagecheck.cpp
    src/sweepui/artifacts.cpp
    src/sweepui/synthetic.cpp
    src/sweepui/netsource.cpp
    src/sweepui/hackrfsource.cpp
    src/sweepui/rtlsdrsource.cpp)
  target_include_directories(sweepui_core PUBLIC src/sweepui)
  # sweepd_plan, not sweepd_profiles: the RTL-SDR source sweeps §5's own LO grid
  # by calling the daemon's planner rather than reimplementing it, and plan.cpp
  # is the POSIX-free half that can be linked here. See src/sweepd/plan.h.
  target_link_libraries(sweepui_core PUBLIC sweepd_dsp sweepd_plan sweepd_trackplan sweepd_gen sweepd_proto sweepd_iq
                                          sweepd_gaintable pthread)
  if(WIN32)
    target_link_libraries(sweepui_core PUBLIC ws2_32)
  endif()
  target_compile_options(sweepui_core PRIVATE -O2 -Wall -Wextra)

  # ------------------------------------------------------------------ libhackrf
  #
  # Optional, and deliberately not fatal when missing. A HackRF is a second
  # reference source (see src/sweepui/hackrfsource.h), not something sweepui
  # needs in order to be useful -- the synthetic source and the §10.4 TCP
  # client are both unaffected. Without the library hackrfsource.cpp still
  # compiles, and --hackrf fails at runtime with a sentence naming the missing
  # package rather than with an unknown-argument error.
  option(SWEEPSA_WITH_HACKRF "Build the HackRF source if libhackrf is present" ON)
  if(SWEEPSA_WITH_HACKRF)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
      pkg_check_modules(HACKRF QUIET libhackrf)
    endif()
    if(NOT HACKRF_FOUND)
      # MSYS2 and some distributions ship the header without a .pc file.
      find_path(HACKRF_HEADER_DIR libhackrf/hackrf.h)
      if(HACKRF_HEADER_DIR)
        set(HACKRF_FOUND TRUE)
        set(HACKRF_INCLUDE_DIRS "${HACKRF_HEADER_DIR}")
      endif()
    endif()
    if(HACKRF_FOUND)
      # Header only: libhackrf is loaded at run time (src/sweepui/hackrf_dl.h),
      # so a sweepui built here still starts on a machine without it, and
      # libusb (LGPL) never enters sweepui's own link.
      message(STATUS "libhackrf ${HACKRF_VERSION}: the --hackrf source is enabled (loaded at run time)")
      target_compile_definitions(sweepui_core PUBLIC SWEEPSA_HAVE_HACKRF)
      target_include_directories(sweepui_core PUBLIC ${HACKRF_INCLUDE_DIRS})
      target_sources(sweepui_core PRIVATE src/sweepui/hackrf_dl.cpp)
      target_link_libraries(sweepui_core PUBLIC ${CMAKE_DL_LIBS})
    else()
      message(STATUS "libhackrf not found: --hackrf will report it as missing")
    endif()
  endif()

  # ------------------------------------------------------------------ librtlsdr
  #
  # Same contract as libhackrf above: optional, never fatal, and the option
  # still exists without it so a missing package is distinguishable from a typo.
  # See src/sweepui/rtlsdrsource.h for what an RTL-SDR is doing in a brief about
  # an AD936x -- briefly, it is the only host-side source that sweeps §5's own
  # segment grid, so it is the only one §7's overlay and image check can be
  # exercised against without the E310 on the bench.
  option(SWEEPSA_WITH_RTLSDR "Build the RTL-SDR source if librtlsdr is present" ON)
  if(SWEEPSA_WITH_RTLSDR)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
      # IMPORTED_TARGET links by full path, so the build tree's RPATH points at
      # whichever librtlsdr was found. That matters for a patched one, e.g. the
      # nextrtl fork: -DCMAKE_PREFIX_PATH=<fork>/install picks it over /usr/lib
      # at both link and run time. See docs/rtlsdr.md.
      pkg_check_modules(RTLSDR QUIET IMPORTED_TARGET librtlsdr)
    endif()
    if(RTLSDR_FOUND)
      message(STATUS "librtlsdr ${RTLSDR_VERSION} (${RTLSDR_LINK_LIBRARIES}): the --rtl source is enabled")
      target_compile_definitions(sweepui_core PUBLIC SWEEPSA_HAVE_RTLSDR)
      target_link_libraries(sweepui_core PUBLIC PkgConfig::RTLSDR)
    else()
      message(STATUS "librtlsdr not found: --rtl will report it as missing")
    endif()
  endif()

  add_executable(dsp_test tests/dsp_test.cpp)
  target_link_libraries(dsp_test PRIVATE sweepd_dsp m)
  target_compile_options(dsp_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME dsp COMMAND dsp_test)

  add_executable(stitch_test tests/stitch_test.cpp)
  target_link_libraries(stitch_test PRIVATE sweepd_dsp m)
  target_compile_options(stitch_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME stitch COMMAND stitch_test)

  add_executable(gen_test tests/gen_test.cpp)
  target_link_libraries(gen_test PRIVATE sweepd_gen m)
  target_compile_options(gen_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME gen COMMAND gen_test)

  add_executable(gaintable_test tests/gaintable_test.cpp)
  target_link_libraries(gaintable_test PRIVATE sweepd_gaintable)
  target_compile_options(gaintable_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME gaintable COMMAND gaintable_test)

  add_executable(proto_test tests/proto_test.cpp)
  target_link_libraries(proto_test PRIVATE sweepd_proto m)
  target_compile_options(proto_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME proto COMMAND proto_test)

  add_executable(iq_test tests/iq_test.cpp)
  target_link_libraries(iq_test PRIVATE sweepd_iq sweepd_proto m)
  target_compile_options(iq_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME iq COMMAND iq_test)

  # The host end of the wire: connects to a running sweepd, decodes 0x02 frames
  # and prints the same CSV sweepd writes locally. §10.4 wires sweepui to
  # sweepd; this proves the protocol before there is a GUI to blame.
  add_executable(sweep_client tools/sweep_client/main.cpp)
  target_link_libraries(sweep_client PRIVATE sweepd_proto sweepd_dsp m)
  target_compile_options(sweep_client PRIVATE -O2 -Wall -Wextra)

  # The same job for the HackRF source: drive it with no display and print the
  # trace as numbers, so it can be checked before it is looked at.
  add_executable(hackrf_probe tools/hackrf_probe/main.cpp)
  target_link_libraries(hackrf_probe PRIVATE sweepui_core m)
  target_compile_options(hackrf_probe PRIVATE -O2 -Wall -Wextra)

  # And again for the RTL-SDR source. This one carries a little more weight than
  # its HackRF counterpart: the per-hop settle is most of the sweep time and is
  # inherited from rtl_power rather than measured, so `--settle-scan` is how
  # someone with a dongle turns it into a number. See docs/rtlsdr.md.
  add_executable(rtl_probe tools/rtl_probe/main.cpp)
  target_link_libraries(rtl_probe PRIVATE sweepui_core m)
  target_compile_options(rtl_probe PRIVATE -O2 -Wall -Wextra)

  # The RTL-SDR source's hardware-free half: the tuner ranges, the gain
  # snapping, the validation and -- the part where a mistake is invisible -- the
  # PlanConfig the options derive. Runs with no dongle and no librtlsdr.
  add_executable(rtlsdr_test tests/rtlsdr_test.cpp)
  target_link_libraries(rtlsdr_test PRIVATE sweepui_core m)
  target_compile_options(rtlsdr_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME rtlsdr COMMAND rtlsdr_test)

  # ------------------------------------------------------------------ sweepui
  #
  # §6, delivery step §10.3. Host only: the E310 has no display and the UI is
  # explicitly presentation and control (§3).
  add_executable(freq_test tests/freq_test.cpp)
  target_link_libraries(freq_test PRIVATE sweepui_core m)
  target_compile_options(freq_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME freq COMMAND freq_test)

  add_executable(decimate_test tests/decimate_test.cpp)
  target_link_libraries(decimate_test PRIVATE sweepui_core m)
  target_compile_options(decimate_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME decimate COMMAND decimate_test)

  # §10.5. The instrument features, all of them host-testable because none of
  # them touches ImGui: what a hold does with an unmeasured bin, what an average
  # does with dB, and what a marker reads when it is pointed somewhere the radio
  # is not sweeping. Every failure in here is a confident wrong number rather
  # than a crash, which is why they are pinned rather than eyeballed.
  add_executable(trace_test tests/trace_test.cpp)
  target_link_libraries(trace_test PRIVATE sweepui_core m)
  target_compile_options(trace_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME trace COMMAND trace_test)

  add_executable(marker_test tests/marker_test.cpp)
  target_link_libraries(marker_test PRIVATE sweepui_core m)
  target_compile_options(marker_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME marker COMMAND marker_test)

  add_executable(export_test tests/export_test.cpp)
  target_link_libraries(export_test PRIVATE sweepui_core m)
  target_compile_options(export_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME export COMMAND export_test)

  add_executable(iqexport_test tests/iqexport_test.cpp)
  target_link_libraries(iqexport_test PRIVATE sweepui_core m)
  target_compile_options(iqexport_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME iqexport COMMAND iqexport_test)

  # §7's amplitude correction. Pure arithmetic and a file parser, so the two
  # ways it can be plausibly wrong -- extrapolating past the measured range, and
  # indexing a zero-span time axis as if it were frequency -- are pinned here
  # rather than discovered as a confident dBm number on screen.
  add_executable(nf_test tests/nf_test.cpp)
  target_link_libraries(nf_test PRIVATE sweepui_core m)
  target_compile_options(nf_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME nf COMMAND nf_test)

  add_executable(measure_test tests/measure_test.cpp)
  target_link_libraries(measure_test PRIVATE sweepui_core m)
  target_compile_options(measure_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME measure COMMAND measure_test)

  add_executable(persist_test tests/persist_test.cpp)
  target_link_libraries(persist_test PRIVATE sweepui_core m)
  target_compile_options(persist_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME persist COMMAND persist_test)

  add_executable(tracknorm_test tests/tracknorm_test.cpp)
  target_link_libraries(tracknorm_test PRIVATE sweepui_core m)
  target_compile_options(tracknorm_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME tracknorm COMMAND tracknorm_test)

  add_executable(ampcorr_test tests/ampcorr_test.cpp)
  target_link_libraries(ampcorr_test PRIVATE sweepui_core m)
  target_compile_options(ampcorr_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME ampcorr COMMAND ampcorr_test)

  # §7's segment boundary overlay. The important test in here is the drift
  # guard: sweepui reconstructs the LO grid from what `config` reports, which is
  # a second implementation of sweepd::plan_sweep's geometry, so this links the
  # *real* plan_sweep and compares every edge across a spread of configs
  # including the ones where the tuner clamp bites. An overlay that drifts would
  # blame an artifact on the wrong frequency, silently.
  add_executable(segments_test tests/segments_test.cpp)
  target_link_libraries(segments_test PRIVATE sweepui_core m)
  target_compile_options(segments_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME segments COMMAND segments_test)

  add_executable(imagecheck_test tests/imagecheck_test.cpp)
  target_link_libraries(imagecheck_test PRIVATE sweepui_core m)
  target_compile_options(imagecheck_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME imagecheck COMMAND imagecheck_test)

  # The receiver-artifact marker: a simulated receiver on the real plan_sweep,
  # carrying every artifact class it must mark and the real carriers it must not.
  add_executable(artifacts_test tests/artifacts_test.cpp)
  target_link_libraries(artifacts_test PRIVATE sweepui_core m)
  target_compile_options(artifacts_test PRIVATE -O2 -Wall -Wextra)
  add_test(NAME artifacts COMMAND artifacts_test)

  # The windowed binary. Optional, because everything above is useful on a
  # machine with no GL -- and because a missing submodule should say so rather
  # than failing somewhere inside a backend.
  option(SWEEPSA_BUILD_UI "Build the sweepui GUI" ON)
  if(SWEEPSA_BUILD_UI AND NOT EXISTS "${CMAKE_SOURCE_DIR}/third_party/imgui/imgui.cpp")
    message(STATUS "third_party/imgui is empty -- run 'git submodule update --init'. "
                   "Skipping sweepui.")
    set(SWEEPSA_BUILD_UI OFF)
  endif()

  if(SWEEPSA_BUILD_UI)
    find_package(OpenGL REQUIRED)

    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
    add_subdirectory(third_party/glfw EXCLUDE_FROM_ALL)

    # Dear ImGui, docking branch (§6), plus the two backends. Built here rather
    # than consumed as a package because ImGui ships no build system by design.
    add_library(imgui STATIC
      third_party/imgui/imgui.cpp
      third_party/imgui/imgui_draw.cpp
      third_party/imgui/imgui_tables.cpp
      third_party/imgui/imgui_widgets.cpp
      third_party/imgui/backends/imgui_impl_glfw.cpp
      third_party/imgui/backends/imgui_impl_opengl3.cpp)
    target_include_directories(imgui PUBLIC third_party/imgui third_party/imgui/backends)
    target_link_libraries(imgui PUBLIC glfw OpenGL::GL)
    target_compile_options(imgui PRIVATE -O2 -w)

    add_library(implot STATIC
      third_party/implot/implot.cpp
      third_party/implot/implot_items.cpp)
    target_include_directories(implot PUBLIC third_party/implot)
    target_link_libraries(implot PUBLIC imgui)
    target_compile_options(implot PRIVATE -O2 -w)

    add_executable(sweepui
      src/sweepui/main.cpp
      src/sweepui/app.cpp
      src/sweepui/iqview.cpp
      src/sweepui/gl.cpp
      src/sweepui/waterfall.cpp)
    target_link_libraries(sweepui PRIVATE sweepui_core implot imgui glfw OpenGL::GL m)
    target_compile_options(sweepui PRIVATE -O2 -Wall -Wextra)

    # The version the About window shows. ./sbt reconfigures on every build,
    # so git describe stays current; the release workflow passes the tag.
    set(SWEEPSA_VERSION "" CACHE STRING "Version shown in sweepui's About window (default: git describe)")
    set(_sweepsa_version "${SWEEPSA_VERSION}")
    if(NOT _sweepsa_version)
      execute_process(COMMAND git describe --tags --always --dirty
                      WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
                      OUTPUT_VARIABLE _sweepsa_version
                      OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    endif()
    if(NOT _sweepsa_version)
      set(_sweepsa_version "dev")
    endif()
    target_compile_definitions(sweepui PRIVATE SWEEPSA_VERSION="${_sweepsa_version}")
  endif()
