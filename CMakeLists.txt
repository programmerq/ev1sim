cmake_minimum_required(VERSION 3.18)
project(ev1sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Chrono (must be pre-installed with Vehicle + Irrlicht modules) ──────────
find_package(Chrono REQUIRED COMPONENTS Vehicle Irrlicht CONFIG)

# ── nlohmann/json (header-only, fetched at configure time) ──────────────────
include(FetchContent)
FetchContent_Declare(json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(json)

# ── Main executable ─────────────────────────────────────────────────────────
add_executable(ev1sim
    src/main.cpp
    src/SimApp.cpp
    src/VehicleWorld.cpp
    src/CommandDriver.cpp
    src/KeyboardInputController.cpp
    src/CameraManager.cpp
    src/Config.cpp
    src/Telemetry.cpp
)

target_include_directories(ev1sim PRIVATE src)
target_link_libraries(ev1sim PRIVATE
    ${CHRONO_TARGETS}
    nlohmann_json::nlohmann_json
)

# Pass Chrono's data directory so vehicle JSON definitions are found at runtime.
target_compile_definitions(ev1sim PRIVATE
    CHRONO_DATA_DIR="${CHRONO_DATA_DIR}"
)

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
        src/Config.cpp
        src/KeyboardInputController.cpp
    )

    target_include_directories(ev1sim_tests PRIVATE src)
    target_link_libraries(ev1sim_tests PRIVATE
        Catch2::Catch2WithMain
        nlohmann_json::nlohmann_json
        ${CHRONO_TARGETS}
    )

    include(CTest)
    include(Catch)
    catch_discover_tests(ev1sim_tests)
endif()
