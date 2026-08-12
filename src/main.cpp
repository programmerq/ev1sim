#include "Config.h"
#include "SimApp.h"

#include <fstream>
#include <iostream>
#include <string>

// Returns the config path and whether the user named it.  The distinction
// matters: see the check in main().
static std::string FindConfigPath(int argc, char* argv[], bool& explicit_out) {
    for (int i = 1; i < argc - 1; ++i) {
        if (std::string(argv[i]) == "--config") {
            explicit_out = true;
            return argv[i + 1];
        }
    }
    explicit_out = false;
    return "config/default.json";
}

int main(int argc, char* argv[]) {
    // 1. Load config (JSON file, then CLI overrides).
    bool config_was_named = false;
    std::string config_path = FindConfigPath(argc, argv, config_was_named);

    // A config the user NAMED and that does not open is fatal.  Config::
    // LoadFromFile warns and returns built-in defaults, which is right for the
    // implicit config/default.json — but for a named path it is a silent
    // substitution of a different experiment, and those are hard to see
    // precisely because the run still succeeds.
    //
    // 2026-08-12, the case that motivated this: the coastdown was documented
    // as `--headless --start-propulsion-enabled --scenario …` with no
    // --config.  Run where config/default.json resolves, it loads the milford
    // level, the car leaves it mid-launch and the solver diverges to 370 m/s.
    // Run from anywhere else, it falls back to a rigid_plane and produces a
    // perfectly ordinary coastdown topping out at 30.15 m/s.  Same command,
    // either outcome, decided by the working directory, and the only signal is
    // one stderr line that a stdout redirect hides.  Four months of coastdown
    // numbers were quoted from a run condition nobody could identify.
    //
    // This does not make the implicit default fatal — running with no flags at
    // all is allowed to fall back, and that is documented behaviour.  It makes
    // the SCRIPTED case, the one that ends up in a doc or a CI step, say so.
    if (config_was_named) {
        std::ifstream probe(config_path);
        if (!probe.is_open()) {
            std::cerr << "[Fatal] --config " << config_path
                      << " cannot be opened.\n"
                      << "        Refusing to fall back to built-in defaults: "
                         "you asked for a specific\n"
                      << "        experiment and this would silently run a "
                         "different one.\n";
            return 2;
        }
    }

    Config config = Config::LoadFromFile(config_path);
    config.ApplyCliOverrides(argc, argv);

    // 2. Run.  SimApp::Run returns one of SimApp::kExit* codes — propagate
    //    to the shell so CI can distinguish normal completion, scenario
    //    timeout, SIGINT, and usage errors.
    try {
        SimApp app(config);
        return app.Run();
    } catch (const std::exception& e) {
        std::cerr << "[Fatal] " << e.what() << "\n";
        return 1;
    }
}
