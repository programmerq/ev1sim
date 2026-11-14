#include "ChargerStatus.h"

#include <cmath>
#include <cstdio>

namespace ev1sim {

namespace {

std::wstring Widen(const std::string& s) { return std::wstring(s.begin(), s.end()); }

std::wstring Fixed1(const std::optional<double>& v) {
    if (!v) return L"---";
    wchar_t buf[32];
    std::swprintf(buf, sizeof(buf) / sizeof(buf[0]), L"%.1f", *v);
    return buf;
}

std::wstring Whole(const std::optional<int>& v) {
    if (!v) return L"---";
    return std::to_wstring(*v);
}

}  // namespace

std::string ChargerStateName(std::uint32_t state) {
    switch (static_cast<ChargerState>(state)) {
        case ChargerState::kWaitForCoupler: return "WAIT FOR PADDLE";
        case ChargerState::kIdle:           return "IDLE";
        case ChargerState::kCharging:       return "CHARGING";
        case ChargerState::kTaper:          return "TAPER";
        case ChargerState::kComplete:       return "COMPLETE";
        case ChargerState::kFault:          return "FAULT";
    }
    return "STATE " + std::to_string(state);
}

std::string ChargerFaultNames(std::uint32_t bits) {
    struct Named {
        std::uint32_t bit;
        const char* name;
    };
    static constexpr Named kNames[] = {
        {ChargerFault::kOvertemp, "OVERTEMP"},
        {ChargerFault::kOvercurrent, "OVERCURRENT"},
        {ChargerFault::kCommLoss, "COMM LOSS"},
        {ChargerFault::kAcInputBrownout, "MAINS LOW"},
        {ChargerFault::kCouplerOvertemp, "PADDLE OVERTEMP"},
        {ChargerFault::kInternalAbort, "ABORTED"},
    };
    std::string out;
    auto add = [&out](const std::string& name) {
        if (!out.empty()) out += ", ";
        out += name;
    };
    std::uint32_t known = 0;
    for (const auto& n : kNames) {
        known |= n.bit;
        if (bits & n.bit) add(n.name);
    }
    for (int i = 0; i < 32; ++i) {
        const std::uint32_t bit = 1u << i;
        if ((bits & bit) && !(known & bit)) add("BIT " + std::to_string(i));
    }
    return out.empty() ? "none" : out;
}

std::wstring FormatChargerStateLabel(const ChargerStatus& s) {
    return L"Charger: " + (s.state ? Widen(ChargerStateName(*s.state)) : std::wstring(L"---"));
}

std::wstring FormatChargerOutputLabel(const ChargerStatus& s) {
    // The cell counts charging current negative; show the charge rate.
    std::optional<double> rate;
    if (s.current_a) rate = std::fabs(*s.current_a);
    return L"Charge: " + Fixed1(rate) + L" A at " + Fixed1(s.voltage_v) + L" V";
}

std::wstring FormatChargerTempsLabel(const ChargerStatus& s) {
    return L"Charger temp " + Whole(s.charger_temp_c) + L" C, paddle " +
           Whole(s.coupler_temp_c) + L" C";
}

std::wstring FormatChargerMainsLabel(const ChargerStatus& s) {
    std::optional<int> volts;
    if (s.ac_input_v) volts = static_cast<int>(std::lround(*s.ac_input_v));
    return L"Mains: " + Whole(volts) + L" V";
}

std::wstring FormatChargerFaultsLabel(const ChargerStatus& s) {
    return L"Charger faults: " +
           (s.fault_bits ? Widen(ChargerFaultNames(*s.fault_bits)) : std::wstring(L"---"));
}

}  // namespace ev1sim
