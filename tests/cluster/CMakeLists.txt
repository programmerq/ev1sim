# Standalone build of the IPC instrument-cluster panel tests
# (docs/ipc_rsa_display_plan.md Phase 1: BusSnoop + InstrumentClusterPanel).
#
# Why a separate project: the root CMakeLists.txt is being changed by other open
# work, so these sources join the ev1sim_tests target in a follow-up. Until then
# this builds exactly the new sources and their two test files:
#
#   cmake -S tests/cluster -B build-cluster                # against tests/electricsim_stub
#   cmake -S tests/cluster -B build-cluster -DEV1SIM_CLUSTER_WIRE_TRUTH=OFF  # decode only
#   cmake --build build-cluster && ctest --test-dir build-cluster --output-on-failure
#
# The follow-up is two lines in the root ev1sim_tests list
# (tests/test_bus_snoop.cpp + tests/test_instrument_cluster_panel.cpp, with
# src/BusSnoop.cpp + src/InstrumentClusterPanel.cpp) and the same two sources in
# the ev1sim app target; this directory and its CI job go away then.
cmake_minimum_required(VERSION 3.18)
project(ev1sim_cluster_tests LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(EV1SIM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

option(EV1SIM_CLUSTER_WIRE_TRUTH
       "Build against the external sim's wire substrate (the TX-cell snoop path)" ON)
set(ELECTRICSIM_DIR "${EV1SIM_ROOT}/tests/electricsim_stub" CACHE PATH
    "External sim tree providing src/io (defaults to the vendored stub)")

include(FetchContent)
FetchContent_Declare(Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.5.2
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(Catch2)

find_path(IRRLICHT_INCLUDE_DIR NAMES irrlicht.h PATH_SUFFIXES irrlicht)
find_library(IRRLICHT_LIBRARY NAMES Irrlicht)
if(NOT IRRLICHT_INCLUDE_DIR OR NOT IRRLICHT_LIBRARY)
    message(FATAL_ERROR "The cluster panel tests need Irrlicht (Ubuntu: libirrlicht-dev).")
endif()

add_executable(ev1sim_cluster_tests
    "${EV1SIM_ROOT}/tests/test_bus_snoop.cpp"
    "${EV1SIM_ROOT}/tests/test_instrument_cluster_panel.cpp"
    "${EV1SIM_ROOT}/src/BusSnoop.cpp"
    "${EV1SIM_ROOT}/src/InstrumentClusterPanel.cpp"
)
target_include_directories(ev1sim_cluster_tests PRIVATE "${EV1SIM_ROOT}/src")
target_include_directories(ev1sim_cluster_tests SYSTEM PRIVATE "${IRRLICHT_INCLUDE_DIR}")
target_link_libraries(ev1sim_cluster_tests PRIVATE Catch2::Catch2WithMain ${IRRLICHT_LIBRARY})

if(EV1SIM_CLUSTER_WIRE_TRUTH)
    # The same substrate files the root build compiles for the frame snoop.
    add_library(cluster_wire_substrate STATIC
        "${ELECTRICSIM_DIR}/src/io/wire_table.cpp"
        "${ELECTRICSIM_DIR}/src/io/gm8192/gm8192_frame.c"
        "${ELECTRICSIM_DIR}/src/io/gm8192/gm8192_rx_framer.cpp"
        "${ELECTRICSIM_DIR}/src/io/uart/uart_rx.cpp"
        "${ELECTRICSIM_DIR}/src/io/uart/uart_tx.cpp")
    target_include_directories(cluster_wire_substrate PUBLIC
        "${ELECTRICSIM_DIR}/src/io" "${ELECTRICSIM_DIR}/src")
    find_package(Threads REQUIRED)
    target_link_libraries(cluster_wire_substrate PUBLIC Threads::Threads)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        target_link_libraries(cluster_wire_substrate PUBLIC rt)
    endif()
    target_link_libraries(ev1sim_cluster_tests PRIVATE cluster_wire_substrate)
    target_compile_definitions(ev1sim_cluster_tests PRIVATE EV1SIM_HAVE_WIRE_TRUTH=1)
endif()

enable_testing()
include(CTest)
include(Catch)
catch_discover_tests(ev1sim_cluster_tests)
