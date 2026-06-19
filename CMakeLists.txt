cmake_minimum_required(VERSION 3.18)
project(ev1sim LANGUAGES C CXX)  # C is required by the vendored SDL3 (C library)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enable Objective-C++ up front — before any FetchContent that may enable OBJC
# (SDL3 does, for its macOS joystick/haptic backend) — so the OBJCXX compile
# rules are fully initialized for src/MacOSPlatform.mm regardless of subproject
# language-enablement order.
if(APPLE)
    enable_language(OBJCXX)
    set(CMAKE_OBJCXX_STANDARD 17)
endif()


# ── Chrono (required for the sim app; optional for a unit-tests-only build) ──
# Auto-detect local deps/ install; override with -DChrono_DIR=<path>
if(NOT Chrono_DIR AND EXISTS "${CMAKE_SOURCE_DIR}/deps/chrono-install/lib/cmake/Chrono")
    set(Chrono_DIR "${CMAKE_SOURCE_DIR}/deps/chrono-install/lib/cmake/Chrono")
endif()
# The unit tests are Chrono-free (no VehicleWorld/SimApp in the test target), so
# CI can build + run them without a (slow-to-build) Chrono install.  Pass
# -DEV1SIM_TESTS_ONLY=ON to relax the Chrono requirement and skip the sim app.
# Default OFF keeps the normal `find_package(Chrono REQUIRED)` behaviour intact.
option(EV1SIM_TESTS_ONLY "Build only the Chrono-free unit tests (no sim app)" OFF)
if(EV1SIM_TESTS_ONLY)
    find_package(Chrono QUIET COMPONENTS Vehicle Irrlicht CONFIG)
else()
    find_package(Chrono REQUIRED COMPONENTS Vehicle Irrlicht CONFIG)
endif()

if(Chrono_FOUND)
    # Chrono uses old-style CMake variables rather than imported targets.
    # Apply its compiler flags, include dirs, and linker flags globally for
    # all targets that need Chrono.
    include_directories(${CHRONO_INCLUDE_DIRS})
    add_compile_options(${CHRONO_CXX_FLAGS})

    # Chrono's chrono-config.cmake hard-codes `-Wl,-no_compact_unwind` into
    # CHRONO_LINKER_FLAGS.  On Apple Silicon that strips the __unwind_info
    # section the runtime needs to walk the stack on `throw`, silently breaking
    # every try/catch in the program (uncaught exceptions abort with libc++abi
    # even when a matching handler is in main).  Drop it.
    list(REMOVE_ITEM CHRONO_LINKER_FLAGS "-Wl,-no_compact_unwind")
    add_link_options(${CHRONO_LINKER_FLAGS})

    # Ensure Chrono shared libraries are found at runtime via RPATH.
    set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
    # Extract library directories from CHRONO_LIBRARIES for RPATH.
    set(_chrono_rpaths "")
    foreach(_lib ${CHRONO_LIBRARIES})
        get_filename_component(_dir "${_lib}" DIRECTORY)
        list(APPEND _chrono_rpaths "${_dir}")
    endforeach()
    list(REMOVE_DUPLICATES _chrono_rpaths)
    set(CMAKE_BUILD_RPATH "${_chrono_rpaths}")

    # ev1sim's render helpers (VehicleLights, CameraManager, FloatingUiPanel,
    # WiperRenderer) reference Irrlicht symbols directly (e.g.
    # irr::video::IdentityMaterial), so the Irrlicht library must be explicit on
    # the link line.  Modern GNU ld will NOT inherit it transitively from
    # Chrono's libChronoEngine_irrlicht ("DSO missing from command line");
    # macOS's linker tolerates the transitive reference, Linux does not.  The
    # headers already arrive via CHRONO_INCLUDE_DIRS.  If Irrlicht isn't a
    # standalone library here (some macOS/Chrono setups), leave it empty and
    # fall back to Chrono's transitive link.
    find_library(IRRLICHT_LIBRARY NAMES Irrlicht)
    if(NOT IRRLICHT_LIBRARY)
        set(IRRLICHT_LIBRARY "")
    endif()
else()
    # Chrono-free build (EV1SIM_TESTS_ONLY).  The unit tests still include
    # Irrlicht headers (via VehicleLights/VehiclePanels) and call into it from
    # the render helpers, so locate Irrlicht directly instead of via Chrono.
    message(STATUS "Chrono not found — building Chrono-free unit tests only "
                   "(EV1SIM_TESTS_ONLY); the ev1sim app target is skipped.")
    find_path(IRRLICHT_INCLUDE_DIR NAMES irrlicht.h PATH_SUFFIXES irrlicht)
    find_library(IRRLICHT_LIBRARY NAMES Irrlicht)
    # Fail fast with an actionable message rather than a murky compile/link error
    # later if Irrlicht (still needed by the render helpers in the test sources)
    # is missing in a tests-only build.
    if(NOT IRRLICHT_INCLUDE_DIR OR NOT IRRLICHT_LIBRARY)
        message(FATAL_ERROR
            "EV1SIM_TESTS_ONLY needs Irrlicht (headers + library), but it was not "
            "found.\n  Install it — Ubuntu: 'sudo apt install libirrlicht-dev'; "
            "macOS: 'brew install irrlicht'.\n  Or build the full app against "
            "Chrono instead (leave EV1SIM_TESTS_ONLY=OFF).")
    endif()
    include_directories(${IRRLICHT_INCLUDE_DIR})
endif()

# ── nlohmann/json (header-only, fetched at configure time) ──────────────────
include(FetchContent)
FetchContent_Declare(json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(json)

# ── SDL3 (wheel input + force feedback) — optional, compile-out-able ─────────
# Prefer a system/Homebrew SDL3 (fast); otherwise vendor it via FetchContent
# (matches the json/Catch2 pattern).  Only the joystick + haptic subsystems are
# used — never video/audio — so SDL coexists with Irrlicht's window/GL context.
option(EV1SIM_HAVE_WHEEL_IO "Build SDL3 wheel input + force feedback" ON)
set(EV1SIM_WHEEL_IO_SRC "")
set(EV1SIM_WHEEL_IO_TEST_SRC "")
set(_ev1sim_wheel_io FALSE)
if(EV1SIM_HAVE_WHEEL_IO)
    find_package(SDL3 QUIET CONFIG)
    if(NOT SDL3_FOUND)
        message(STATUS "SDL3: not found via find_package — vendoring via FetchContent")
        set(SDL_SHARED       OFF CACHE BOOL "" FORCE)
        set(SDL_STATIC       ON  CACHE BOOL "" FORCE)
        set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
        set(SDL_EXAMPLES     OFF CACHE BOOL "" FORCE)
        # Build SDL with its default subsystems.  Disabling SDL_VIDEO in a static
        # build leaves SDL_events.c referencing video-side symbols that are then
        # undefined at link (SDL_PumpEventsInternal, ...).  We only *use*
        # joystick + haptic at runtime; the unused backends are harmless in a
        # static lib, and SDL3::SDL3 propagates the needed frameworks transitively.
        FetchContent_Declare(SDL3
            GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
            GIT_TAG        release-3.2.0
            GIT_SHALLOW    TRUE
        )
        FetchContent_MakeAvailable(SDL3)
    else()
        message(STATUS "SDL3: using system package (${SDL3_DIR})")
    endif()
    set(EV1SIM_WHEEL_IO_SRC
        src/SdlContext.cpp
        src/WheelInputController.cpp
        src/ForceFeedback.cpp)
    set(EV1SIM_WHEEL_IO_TEST_SRC tests/test_wheel_input.cpp)
    set(_ev1sim_wheel_io TRUE)
endif()

# ── electricsim wire-truth substrate / connector (optional) ─────────────────
# If the electricsim repo is checked out next to this one (or pointed at via
# -DELECTRICSIM_DIR=<path>), build a small static library from its wire-truth
# substrate sources (no simavr / libelf / firmware needed) and link ev1sim's
# ExternalSimConnector against it.  Otherwise fall back to a stub so
# --external-sim is a no-op.
#
# Wire-truth Phase 4 (2026-06-17): electricsim deleted the host-side message
# queue (signal_catalog / protocol / shm_transport / bridge_adapter) and unified
# every wire onto one shared-memory WireTable.  ev1sim's connector is now a pure
# WireTable client (via WireTruthChassis), so "external sim available" ==
# "the wire-truth substrate is present", and there is no separate connector lib
# to build.  The stale-check therefore probes the SUBSTRATE sources (which exist
# on a wire-truth electricsim tree) rather than the now-deleted transport ones —
# a transport-free tree is VALID, not stale.

# Source files we need from the electricsim tree (substrate).  Used both as a
# stale-path probe and as the actual library inputs below — keep in sync.  These
# are the files the connector depends on at link/compile time.
set(_electricsim_sources
    "src/io/wire_table.cpp"
    "src/io/topology/env_open.cpp")

# Headers the connector compiles against: the generated topology (kWire*,
# kTopologyHash, declare_all) and ev1_chassis_signals.hpp (the canonical
# chassis-ID drift-guard static_asserts in ExternalSimConnector.cpp).  Both
# survive on a wire-truth tree; probe them so a partial checkout is "stale".
set(_electricsim_headers
    "src/io/topology/topology_generated.h"
    "src/io/ev1_chassis_signals.hpp")

# Helper: returns TRUE in ${_out_var} if every required source + header exists
# under ${_root}.  Probing every file (not just one) catches partially-deleted
# trees — e.g. a git worktree removed while CMake's cache still pointed at it —
# that would otherwise pass a single-file check at configure time only to fail
# later at compile time.
function(_ev1sim_check_electricsim_tree _root _out_var)
    foreach(_src IN LISTS _electricsim_sources _electricsim_headers)
        if(NOT EXISTS "${_root}/${_src}")
            set(${_out_var} FALSE PARENT_SCOPE)
            return()
        endif()
    endforeach()
    set(${_out_var} TRUE PARENT_SCOPE)
endfunction()

# Stale-cache guard: if ELECTRICSIM_DIR is set (typically from a prior
# -D… persisted in CMakeCache.txt) but no longer points at a usable tree,
# warn loudly and clear it so the search below can pick a valid candidate.
if(ELECTRICSIM_DIR)
    _ev1sim_check_electricsim_tree("${ELECTRICSIM_DIR}" _ev1sim_dir_ok)
    if(NOT _ev1sim_dir_ok)
        message(WARNING
            "ELECTRICSIM_DIR='${ELECTRICSIM_DIR}' is stale "
            "(required wire-truth substrate sources/headers missing) — clearing "
            "and re-searching. Pass -DELECTRICSIM_DIR=<path> to pin a tree.")
        unset(ELECTRICSIM_DIR CACHE)
        unset(ELECTRICSIM_DIR)
    endif()
endif()

if(NOT ELECTRICSIM_DIR)
    # Walk up a few levels — handles both the main checkout and the
    # .claude/worktrees/<name> case where the worktree lives several
    # directories below the shared workspace root.
    foreach(_cand
            "${CMAKE_SOURCE_DIR}/../electricsim"
            "${CMAKE_SOURCE_DIR}/../../electricsim"
            "${CMAKE_SOURCE_DIR}/../../../electricsim"
            "${CMAKE_SOURCE_DIR}/../../../../electricsim"
            "${CMAKE_SOURCE_DIR}/../../../../../electricsim")
        _ev1sim_check_electricsim_tree("${_cand}" _ev1sim_cand_ok)
        if(_ev1sim_cand_ok)
            get_filename_component(ELECTRICSIM_DIR "${_cand}" ABSOLUTE)
            break()
        endif()
    endforeach()
endif()

# Build the substrate lib when a usable tree was found.  This is the SAME
# ELECTRICSIM_DIR tree the topology hash is generated from (no vendored copy to
# drift).  EV1SIM_HAVE_WIRE_TRUTH and EV1SIM_HAVE_EXTERNAL_SIM are now the same
# capability — the substrate being present — but both names are kept: the
# connector's guarded code (publish/consume path) is under
# EV1SIM_HAVE_EXTERNAL_SIM, and WireTruthChassis.cpp's real-vs-stub switch is
# under EV1SIM_HAVE_WIRE_TRUTH.
set(EV1SIM_HAVE_EXTERNAL_SIM FALSE)
set(EV1SIM_HAVE_WIRE_TRUTH FALSE)
if(ELECTRICSIM_DIR)
    _ev1sim_check_electricsim_tree("${ELECTRICSIM_DIR}" _ev1sim_dir_final_ok)
    if(_ev1sim_dir_final_ok)
        message(STATUS "electricsim wire-truth substrate: ${ELECTRICSIM_DIR}/src/io")
        message(STATUS "electricsim connector: ${ELECTRICSIM_DIR} (wire-truth client)")
        set(_electricsim_sources_abs "")
        foreach(_src IN LISTS _electricsim_sources)
            list(APPEND _electricsim_sources_abs "${ELECTRICSIM_DIR}/${_src}")
        endforeach()
        add_library(electricsim_wire_substrate STATIC ${_electricsim_sources_abs})
        target_include_directories(electricsim_wire_substrate PUBLIC
            "${ELECTRICSIM_DIR}/src/io")
        find_package(Threads REQUIRED)
        target_link_libraries(electricsim_wire_substrate PUBLIC Threads::Threads)
        set(EV1SIM_HAVE_EXTERNAL_SIM TRUE)
        set(EV1SIM_HAVE_WIRE_TRUTH TRUE)
    endif()
endif()
if(NOT EV1SIM_HAVE_WIRE_TRUTH)
    message(STATUS "electricsim wire-truth substrate: not found "
                   "(set ELECTRICSIM_DIR to enable external sim; "
                   "ExternalSimConnector + WireTruthChassis compile as a disabled no-op)")
endif()

# ── macOS Objective-C++ support ─────────────────────────────────────────────
# (OBJCXX is enabled near the top, before FetchContent; here we only pick the
#  platform source.)
if(APPLE)
    set(MACOS_PLATFORM_SRC src/MacOSPlatform.mm)
else()
    set(MACOS_PLATFORM_SRC)
endif()

# ── Main executable (Chrono/Irrlicht sim app; skipped in a tests-only build) ─
if(Chrono_FOUND)
add_executable(ev1sim
    src/main.cpp
    src/SimApp.cpp
    src/VehicleWorld.cpp
    src/CommandDriver.cpp
    src/KeyboardInputController.cpp
    src/CameraManager.cpp
    src/Config.cpp
    src/Scenario.cpp
    src/ScriptedDriver.cpp
    src/Telemetry.cpp
    src/HornAudio.cpp
    src/SounderAudio.cpp
    src/VehicleLights.cpp
    src/VehiclePanels.cpp
    src/WiperRenderer.cpp
    src/PhysicalWorld.cpp
    src/FloatingUiPanel.cpp
    src/ExternalSimConnector.cpp
    src/WireTruthChassis.cpp
    ${EV1SIM_WHEEL_IO_SRC}
    ${MACOS_PLATFORM_SRC}
)

target_include_directories(ev1sim PRIVATE src)
target_link_libraries(ev1sim PRIVATE
    ${CHRONO_LIBRARIES}
    ${IRRLICHT_LIBRARY}   # explicit Irrlicht .so — render helpers reference it directly (GNU ld)
    nlohmann_json::nlohmann_json
)
# External-sim == wire-truth substrate present (Phase 4). One lib carries both:
# the substrate's PUBLIC include dir (${ELECTRICSIM_DIR}/src/io) gives the
# connector its topology_generated.h + ev1_chassis_signals.hpp, and linking it
# pulls in the WireTable + env_open object code. Both compile defs are set so
# the connector's guarded path (EV1SIM_HAVE_EXTERNAL_SIM) and WireTruthChassis's
# real impl (EV1SIM_HAVE_WIRE_TRUTH) both light up.
if(EV1SIM_HAVE_WIRE_TRUTH)
    target_link_libraries(ev1sim PRIVATE electricsim_wire_substrate)
    target_compile_definitions(ev1sim PRIVATE
        EV1SIM_HAVE_EXTERNAL_SIM=1
        EV1SIM_HAVE_WIRE_TRUTH=1)
endif()
if(_ev1sim_wheel_io)
    target_link_libraries(ev1sim PRIVATE SDL3::SDL3)
    target_compile_definitions(ev1sim PRIVATE EV1SIM_HAVE_WHEEL_IO=1)
endif()

# Link macOS frameworks for platform fixes and audio.
if(APPLE)
    target_link_libraries(ev1sim PRIVATE
        "-framework Cocoa"
        "-framework OpenGL"
        "-framework AudioToolbox"
    )
endif()

# Pass Chrono's data directory so vehicle JSON definitions are found at runtime.
target_compile_definitions(ev1sim PRIVATE
    CHRONO_DATA_DIR="${CHRONO_DATA_DIR}"
)

# ── Copy EV1 vehicle data into the Chrono data directory ───────────────────
# This makes the EV1 JSON definitions and meshes discoverable via
# vehicle::GetDataFile("ev1/...").
file(GLOB_RECURSE EV1_DATA_FILES "${CMAKE_SOURCE_DIR}/data/vehicle/ev1/*")
foreach(_f ${EV1_DATA_FILES})
    file(RELATIVE_PATH _rel "${CMAKE_SOURCE_DIR}/data/vehicle/ev1" "${_f}")
    configure_file("${_f}" "${CHRONO_DATA_DIR}/vehicle/ev1/${_rel}" COPYONLY)
endforeach()
endif()  # Chrono_FOUND (main executable)

# ── Unit tests (optional) ──────────────────────────────────────────────────
option(BUILD_TESTS "Build unit tests" ON)
if(BUILD_TESTS)
    enable_testing()

    FetchContent_Declare(Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.5.2
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(Catch2)

    # The wire-truth round-trip test creates the shared WireTable (creator role,
    # declare_all + the real topology hash) and reads it back through
    # WireTruthChassis, so it only compiles when the substrate is present.
    set(EV1SIM_WIRE_TRUTH_TEST_SRC "")
    if(EV1SIM_HAVE_WIRE_TRUTH)
        set(EV1SIM_WIRE_TRUTH_TEST_SRC tests/test_wire_truth_chassis.cpp)
    endif()

    add_executable(ev1sim_tests
        tests/test_config.cpp
        tests/test_driver_command.cpp
        tests/test_keyboard_input.cpp
        tests/test_external_sim_connector.cpp
        tests/test_scripted_driver.cpp
        tests/test_level_file.cpp
        tests/test_vehicle_lights.cpp
        tests/test_physical_world.cpp
        tests/test_wiper_renderer.cpp
        tests/test_floating_ui.cpp
        tests/test_throttle_bus.cpp
        tests/test_scenario.cpp
        tests/test_brake_pedal.cpp
        tests/test_brake_drum.cpp
        tests/test_brake_bias.cpp
        tests/test_brake_decel.cpp
        tests/test_aerodynamics.cpp
        tests/test_suspension_geometry.cpp
        tests/test_input_actions.cpp
        tests/test_hood.cpp
        ${EV1SIM_WHEEL_IO_TEST_SRC}
        ${EV1SIM_WIRE_TRUTH_TEST_SRC}
        src/Config.cpp
        src/KeyboardInputController.cpp
        src/ExternalSimConnector.cpp
        src/WireTruthChassis.cpp
        src/Scenario.cpp
        src/ScriptedDriver.cpp
        src/VehicleLights.cpp
        src/WiperRenderer.cpp
        src/PhysicalWorld.cpp
        src/FloatingUiPanel.cpp
        src/VehiclePanels.cpp
        ${EV1SIM_WHEEL_IO_SRC}
    )

    target_include_directories(ev1sim_tests PRIVATE src)
    target_compile_definitions(ev1sim_tests PRIVATE
        EV1SIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    )
    target_link_libraries(ev1sim_tests PRIVATE
        Catch2::Catch2WithMain
        nlohmann_json::nlohmann_json
        ${CHRONO_LIBRARIES}
        ${IRRLICHT_LIBRARY}   # empty when Chrono provides Irrlicht; set in the tests-only build
    )
    # External-sim == wire-truth substrate present (Phase 4); see the app target.
    if(EV1SIM_HAVE_WIRE_TRUTH)
        target_link_libraries(ev1sim_tests PRIVATE electricsim_wire_substrate)
        target_compile_definitions(ev1sim_tests PRIVATE
            EV1SIM_HAVE_EXTERNAL_SIM=1
            EV1SIM_HAVE_WIRE_TRUTH=1)
    endif()
    if(_ev1sim_wheel_io)
        target_link_libraries(ev1sim_tests PRIVATE SDL3::SDL3)
        target_compile_definitions(ev1sim_tests PRIVATE EV1SIM_HAVE_WHEEL_IO=1)
    endif()

    include(CTest)
    include(Catch)
    catch_discover_tests(ev1sim_tests)

    # ── ABS integration gate (opt-in; needs a built electricsim) ─────────────
    # The brake-plant ABS behaviour can only be validated with ev1sim AND
    # electricsim running together over the shm bus — the unit suite above
    # cannot exercise it (no moving plant under the ABS algorithm).
    # scripts/abs_regression.sh runs the 6-scenario BTCM-on/off sweep and checks
    # headline metrics against scripts/abs_baseline.txt (pinned electricsim
    # commit).  Run it before merging anything that touches the brake plant.
    #
    #   make test-integration   # build ev1sim + run the gate
    #   ctest -L integration    # same, when configured -DEV1SIM_INTEGRATION_TESTS=ON
    #
    # Deliberately kept out of the default `ctest` (slow, multi-process, and
    # needs electricsim controllers built at ELECTRICSIM_DIR).
    # Requires the ev1sim app target (Chrono), so skip it in a tests-only build.
    if(EV1SIM_HAVE_EXTERNAL_SIM AND Chrono_FOUND)
        add_custom_target(test-integration
            COMMAND ${CMAKE_COMMAND} -E env "ELECTRICSIM_DIR=${ELECTRICSIM_DIR}"
                    bash ${CMAKE_SOURCE_DIR}/scripts/abs_regression.sh
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            USES_TERMINAL
            COMMENT "Running ABS regression gate (6-scenario sweep vs baseline)")
        add_dependencies(test-integration ev1sim)

        option(EV1SIM_INTEGRATION_TESTS
               "Register the ABS regression sweep as a CTest (label: integration)" OFF)
        if(EV1SIM_INTEGRATION_TESTS)
            add_test(NAME abs_regression
                     COMMAND bash ${CMAKE_SOURCE_DIR}/scripts/abs_regression.sh
                     WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
            set_tests_properties(abs_regression PROPERTIES
                LABELS "integration"
                ENVIRONMENT "ELECTRICSIM_DIR=${ELECTRICSIM_DIR}"
                TIMEOUT 1800)
        endif()
    endif()
endif()
