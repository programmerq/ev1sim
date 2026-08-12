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

    // A config the user NAMED and that does not open is fatal; the implicit
    // default is still allowed to fall back.  The reasoning, and the coastdown
    // case that motivated it, are on Config::NamedConfigFault — which lives in
    // Config so the unit suite can pin this rule in every build lane, not only
    // the ones that link this file.
    if (const std::string fault =
            Config::NamedConfigFault(config_path, config_was_named);
        !fault.empty()) {
        std::cerr << "[Fatal] " << fault << "\n";
        return 2;
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
