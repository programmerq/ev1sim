// Nothing in src/ may read the host's clock into a computation.
//
// The freshness windows in ExternalSimConnector aged on steady_clock, and the
// ABS/EMB brake path reads those every physics sub-step — so a host scheduling
// stall changed the vehicle's speed, and the same armed co-sim scenario
// produced a different trace run to run.  The ambient temperature published on
// the chassis bus was the host's local time of day, so it also changed with
// when and where the run happened.  Both are fixed; this is what keeps them
// fixed.
//
// The rule is not "never call a clock" — real-time pacing has to, and the
// exceptions below say which calls those are and why.  The rule is that every
// such call is named here, so adding one is a deliberate act with a reviewer
// attached rather than a line that reads as ordinary C++.
//
// Scanning source text is a blunt instrument, and deliberately so: the defect
// it catches is a call site appearing where nobody was looking, which is
// exactly what a text scan sees and a behavioural test does not.  SimApp.cpp
// in particular is compiled into no test binary, so an assertion about its
// behaviour cannot be written here at all.

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#ifndef EV1SIM_SOURCE_DIR
#define EV1SIM_SOURCE_DIR "."
#endif

namespace {

/// A call that reads the host's clock, and the file:reason it is allowed.
struct Exception {
    const char* file;    ///< basename within src/
    const char* reason;
};

/// Every permitted host-clock read in src/.  Adding an entry is the point at
/// which someone has to justify one — the justification is the entry.
const std::vector<Exception>& AllowedHostClockReads() {
    static const std::vector<Exception> kAllowed = {
        {"SimApp.cpp",
         "realtime pacing only: wall_start anchors sleep_until so a headless "
         "run with simulation.realtime can be watched at 1x. Nothing reads it "
         "back into a value the vehicle depends on."},
    };
    return kAllowed;
}

/// Substrings that constitute reading the host clock.  Type names alone are
/// not a read — `std::chrono::steady_clock::duration` in a duration_cast is
/// spelling out a type, so the ::now() / call form is what is matched.
const std::vector<std::string>& HostClockReads() {
    static const std::vector<std::string> kReads = {
        "steady_clock::now(",
        "system_clock::now(",
        "high_resolution_clock::now(",
        "localtime_r(",
        "localtime_s(",
        "std::time(",
        "clock_gettime(",
    };
    return kReads;
}

struct Hit {
    std::string file;
    int         line;
    std::string text;
};

std::vector<Hit> ScanSrc() {
    std::vector<Hit> hits;
    const std::filesystem::path src = std::filesystem::path(EV1SIM_SOURCE_DIR) / "src";
    REQUIRE(std::filesystem::is_directory(src));
    std::vector<std::filesystem::path> files;
    for (const auto& e : std::filesystem::directory_iterator(src)) {
        if (!e.is_regular_file()) continue;
        const auto ext = e.path().extension().string();
        if (ext == ".cpp" || ext == ".h" || ext == ".mm") files.push_back(e.path());
    }
    // Directory order is unspecified; sort so a failure message is stable.
    std::sort(files.begin(), files.end());
    REQUIRE_FALSE(files.empty());

    for (const auto& p : files) {
        std::ifstream f(p);
        REQUIRE(f.is_open());
        std::string line;
        int n = 0;
        while (std::getline(f, line)) {
            ++n;
            // A comment mentioning the old call is documentation, not a call.
            const auto code = line.substr(0, line.find("//"));
            for (const auto& needle : HostClockReads()) {
                if (code.find(needle) != std::string::npos) {
                    hits.push_back({p.filename().string(), n, line});
                    break;
                }
            }
        }
    }
    return hits;
}

bool IsAllowed(const std::string& file) {
    const auto& allowed = AllowedHostClockReads();
    return std::any_of(allowed.begin(), allowed.end(),
                       [&](const Exception& e) { return file == e.file; });
}

}  // namespace

TEST_CASE("src/ reads the host clock only where an exception says it may",
          "[determinism][HostClock]") {
    const auto hits = ScanSrc();

    std::ostringstream unexpected;
    int n_unexpected = 0;
    for (const auto& h : hits) {
        if (IsAllowed(h.file)) continue;
        ++n_unexpected;
        unexpected << "\n  " << h.file << ":" << h.line << "  " << h.text;
    }

    INFO("Host-clock reads with no exception in AllowedHostClockReads():"
         << unexpected.str()
         << "\n\nIf this is pacing, add the file and the reason to the list in "
            "tests/test_host_clock_reads.cpp. If a value the vehicle depends on "
            "is being computed from it, that value changes run to run: use "
            "ExternalSimConnector's sim clock instead.");
    CHECK(n_unexpected == 0);
}

TEST_CASE("the host-clock scan can see its input and can fail on it",
          "[determinism][HostClock]") {
    // A scan that matched nothing — wrong directory, wrong extensions, a
    // pattern list that no longer matches how the calls are spelled — would
    // report a clean tree forever.  So: the scan must find the one read that
    // is supposed to be there, and its patterns must match real call text.
    const auto hits = ScanSrc();
    REQUIRE_FALSE(hits.empty());

    const bool found_pacing =
        std::any_of(hits.begin(), hits.end(), [](const Hit& h) {
            return h.file == "SimApp.cpp" &&
                   h.text.find("steady_clock::now(") != std::string::npos;
        });
    INFO("The scan no longer finds the realtime-pacing clock read in "
         "SimApp.cpp. Either that read moved (update the exception) or the "
         "scan stopped matching call text and can no longer fail.");
    CHECK(found_pacing);

    // And every exception must correspond to a read that is actually there —
    // otherwise the list rots into a permanent blanket over a whole file.
    for (const auto& e : AllowedHostClockReads()) {
        const bool used = std::any_of(hits.begin(), hits.end(),
                                      [&](const Hit& h) { return h.file == e.file; });
        INFO("Exception for " << e.file << " no longer matches any host-clock "
             "read; remove it so the file is covered again.");
        CHECK(used);
    }
}
