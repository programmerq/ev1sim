#include "ClimateLoads.h"

namespace ev1sim {

const char* ClimateLoadColumn(ClimateLoad load) {
    switch (load) {
        case ClimateLoad::kCompressor:         return "compressor_energised";
        case ClimateLoad::kWindshieldHeat:     return "windshield_heat_energised";
        case ClimateLoad::kRearDefogPrimary:   return "rear_defog_primary_energised";
        case ClimateLoad::kRearDefogSecondary: return "rear_defog_secondary_energised";
        case ClimateLoad::kRadiatorFan:        return "radiator_fan_energised";
    }
    return "";
}

std::optional<ClimateLoad> ClimateLoadForColumn(const std::string& column) {
    for (int i = 0; i < kNumClimateLoads; ++i) {
        const auto load = static_cast<ClimateLoad>(i);
        if (column == ClimateLoadColumn(load)) return load;
    }
    return std::nullopt;
}

std::wstring FormatClimateLoadLabel(const wchar_t* name, std::optional<bool> on) {
    std::wstring label = name;
    if (!on) return label + L": ---";
    return label + (*on ? L": ON" : L": OFF");
}

std::wstring FormatRearDefogGridsLabel(std::optional<bool> primary,
                                       std::optional<bool> secondary) {
    const std::wstring name = L"Rear defog grids";
    if (!primary && !secondary) return name + L": ---";
    const int lit = (primary.value_or(false) ? 1 : 0) + (secondary.value_or(false) ? 1 : 0);
    if (lit == 2) return name + L": BOTH";
    if (lit == 1) return name + L": ONE";
    return name + L": OFF";
}

}  // namespace ev1sim
