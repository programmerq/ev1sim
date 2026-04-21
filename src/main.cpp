#include "Config.h"
#include "SimApp.h"

#include <iostream>
#include <string>

static std::string FindConfigPath(int argc, char* argv[]) {
    for (int i = 1; i < argc - 1; ++i) {
        if (std::string(argv[i]) == "--config")
            return argv[i + 1];
    }
    return "config/default.json";
}

int main(int argc, char* argv[]) {
    // 1. Load config (JSON file, then CLI overrides).
    std::string config_path = FindConfigPath(argc, argv);
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
