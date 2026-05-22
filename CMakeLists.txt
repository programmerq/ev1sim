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


# ── Chrono (must be pre-installed with Vehicle + Irrlicht modules) ──────────
# Auto-detect local deps/ install; override with -DChrono_DIR=<path>
if(NOT Chrono_DIR AND EXISTS "${CMAKE_SOURCE_DIR}/deps/chrono-install/lib/cmake/Chrono")
    set(Chrono_DIR "${CMAKE_SOURCE_DIR}/deps/chrono-install/lib/cmake/Chrono")
endif()
find_package(Chrono REQUIRED COMPONENTS Vehicle Irrlicht CONFIG)

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

# ── electricsim connector (optional) ────────────────────────────────────────
# If the electricsim repo is checked out next to this one (or pointed at via
# -DELECTRICSIM_DIR=<path>), we build a small static library from its I/O
# sources (no simavr / libelf / firmware needed) and link the main executable
# against it.  Otherwise we fall back to a stub so --external-sim is a no-op.

# Source files we need from the electricsim tree.  Used both as a stale-path
# probe and as the actual library inputs below — keep these two lists in sync.
set(_electricsim_sources
    "src/io/signal_catalog.cpp"
    "src/io/protocol.cpp"
    "src/io/shm_transport.cpp"
    "src/io/bridge_adapter.cpp")

# Helper: returns TRUE in ${_out_var} if every required source exists under
# ${_root}.  Probing every source (not just the header) catches partially-
# deleted trees — e.g. a git worktree that was removed while CMake's cache
# still pointed at it — that would otherwise pass a single-file check at
# configure time only to fail later at compile time.
function(_ev1sim_check_electricsim_tree _root _out_var)
    foreach(_src IN LISTS _electricsim_sources)
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
            "(required sources missing) — clearing and re-searching. "
            "Pass -DELECTRICSIM_DIR=<path> to pin a specific tree.")
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

set(EV1SIM_HAVE_EXTERNAL_SIM FALSE)
if(ELECTRICSIM_DIR AND EXISTS "${ELECTRICSIM_DIR}/src/io/shm_transport.hpp")
    message(STATUS "electricsim connector: ${ELECTRICSIM_DIR}")
    set(_electricsim_sources_abs "")
    foreach(_src IN LISTS _electricsim_sources)
        list(APPEND _electricsim_sources_abs "${ELECTRICSIM_DIR}/${_src}")
    endforeach()
    add_library(electricsim_connector STATIC ${_electricsim_sources_abs})
    target_include_directories(electricsim_connector PUBLIC
        "${ELECTRICSIM_DIR}/src/io")
    set(EV1SIM_HAVE_EXTERNAL_SIM TRUE)
else()
    message(STATUS "electricsim connector: not found (set ELECTRICSIM_DIR to enable external sim)")
endif()

# ── macOS Objective-C++ support ─────────────────────────────────────────────
# (OBJCXX is enabled near the top, before FetchContent; here we only pick the
#  platform source.)
if(APPLE)
    set(MACOS_PLATFORM_SRC src/MacOSPlatform.mm)
else()
    set(MACOS_PLATFORM_SRC)
endif()

# ── Main executable ─────────────────────────────────────────────────────────
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
    src/VehicleLights.cpp
    src/VehiclePanels.cpp
    src/WiperRenderer.cpp
    src/PhysicalWorld.cpp
    src/FloatingUiPanel.cpp
    src/ExternalSimConnector.cpp
    ${EV1SIM_WHEEL_IO_SRC}
    ${MACOS_PLATFORM_SRC}
)

target_include_directories(ev1sim PRIVATE src)
target_link_libraries(ev1sim PRIVATE
    ${CHRONO_LIBRARIES}
    nlohmann_json::nlohmann_json
)
if(EV1SIM_HAVE_EXTERNAL_SIM)
    target_link_libraries(ev1sim PRIVATE electricsim_connector)
    target_compile_definitions(ev1sim PRIVATE EV1SIM_HAVE_EXTERNAL_SIM=1)
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
        tests/test_input_actions.cpp
        ${EV1SIM_WHEEL_IO_TEST_SRC}
        src/Config.cpp
        src/KeyboardInputController.cpp
        src/ExternalSimConnector.cpp
        src/Scenario.cpp
        src/ScriptedDriver.cpp
        src/VehicleLights.cpp
        src/WiperRenderer.cpp
        src/PhysicalWorld.cpp
        src/FloatingUiPanel.cpp
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
    )
    if(EV1SIM_HAVE_EXTERNAL_SIM)
        target_link_libraries(ev1sim_tests PRIVATE electricsim_connector)
        target_compile_definitions(ev1sim_tests PRIVATE EV1SIM_HAVE_EXTERNAL_SIM=1)
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
    if(EV1SIM_HAVE_EXTERNAL_SIM)
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
