cmake_minimum_required(VERSION 3.18)
project(ev1sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

# ── electricsim connector (optional) ────────────────────────────────────────
# If the electricsim repo is checked out next to this one (or pointed at via
# -DELECTRICSIM_DIR=<path>), we build a small static library from its I/O
# sources (no simavr / libelf / firmware needed) and link the main executable
# against it.  Otherwise we fall back to a stub so --external-sim is a no-op.
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
        if(EXISTS "${_cand}/src/io/shm_transport.hpp")
            get_filename_component(ELECTRICSIM_DIR "${_cand}" ABSOLUTE)
            break()
        endif()
    endforeach()
endif()

set(EV1SIM_HAVE_EXTERNAL_SIM FALSE)
if(ELECTRICSIM_DIR AND EXISTS "${ELECTRICSIM_DIR}/src/io/shm_transport.hpp")
    message(STATUS "electricsim connector: ${ELECTRICSIM_DIR}")
    add_library(electricsim_connector STATIC
        "${ELECTRICSIM_DIR}/src/io/signal_catalog.cpp"
        "${ELECTRICSIM_DIR}/src/io/protocol.cpp"
        "${ELECTRICSIM_DIR}/src/io/shm_transport.cpp"
        "${ELECTRICSIM_DIR}/src/io/bridge_adapter.cpp")
    target_include_directories(electricsim_connector PUBLIC
        "${ELECTRICSIM_DIR}/src/io")
    set(EV1SIM_HAVE_EXTERNAL_SIM TRUE)
else()
    message(STATUS "electricsim connector: not found (set ELECTRICSIM_DIR to enable external sim)")
endif()

# ── macOS Objective-C++ support ─────────────────────────────────────────────
if(APPLE)
    enable_language(OBJCXX)
    set(CMAKE_OBJCXX_STANDARD 17)
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
    src/ScriptedDriver.cpp
    src/Telemetry.cpp
    src/HornAudio.cpp
    src/VehicleLights.cpp
    src/VehiclePanels.cpp
    src/ExternalSimConnector.cpp
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
        src/Config.cpp
        src/KeyboardInputController.cpp
        src/ExternalSimConnector.cpp
        src/ScriptedDriver.cpp
    )

    target_include_directories(ev1sim_tests PRIVATE src)
    target_link_libraries(ev1sim_tests PRIVATE
        Catch2::Catch2WithMain
        nlohmann_json::nlohmann_json
        ${CHRONO_LIBRARIES}
    )
    if(EV1SIM_HAVE_EXTERNAL_SIM)
        target_link_libraries(ev1sim_tests PRIVATE electricsim_connector)
        target_compile_definitions(ev1sim_tests PRIVATE EV1SIM_HAVE_EXTERNAL_SIM=1)
    endif()

    include(CTest)
    include(Catch)
    catch_discover_tests(ev1sim_tests)
endif()
