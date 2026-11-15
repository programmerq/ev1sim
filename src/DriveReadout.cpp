#include "DriveReadout.h"

namespace ev1sim {

namespace f = readout_format;

const std::array<ReadoutRow, kNumDriveReadings> kDriveReadingInfo = {{
    {"wheel_speed_lf_rad_s", "Left front wheel", f::RadPerSec, "BRAKE CONTROLLER"},
    {"wheel_speed_rf_rad_s", "Right front wheel", f::RadPerSec, "BRAKE CONTROLLER"},
    {"wheel_speed_lr_rad_s", "Left rear wheel", f::RadPerSec, "BRAKE CONTROLLER"},
    {"wheel_speed_rr_rad_s", "Right rear wheel", f::RadPerSec, "BRAKE CONTROLLER"},
    {"reference_speed_mps", "Reference speed", f::KphFromMps, "BRAKE CONTROLLER"},
    {"antilock_lamp", "ANTILOCK lamp", f::OnOff, "BRAKE CONTROLLER"},
    {"brake_lamp", "BRAKE lamp", f::OnOff, "BRAKE CONTROLLER"},
    {"low_traction_lamp", "LOW TRACTION lamp", f::OnOff, "BRAKE CONTROLLER"},
    {"park_brake_lamp", "PARK BRAKE lamp", f::OnOff, "BRAKE CONTROLLER"},
    {"pack_v", "Pack at the contactors", f::Volts, "HIGH VOLTAGE"},
    {"inverter_cap_v", "Inverter capacitor", f::Volts, "HIGH VOLTAGE"},
    {"isolation_kohm", "Isolation to chassis", f::Kohm, "HIGH VOLTAGE"},
    {"traction_inverter_delivering", "Traction inverter delivering", f::OnOff, "HIGH VOLTAGE"},
    {"apm_key_on", "Key on", f::OnOff, "ACCESSORY POWER MODULE"},
    {"apm_run_mode", "Run mode", FormatApmRunMode, "ACCESSORY POWER MODULE"},
    {"apm_module_temp_c", "Module temp", f::DegC, "ACCESSORY POWER MODULE"},
    {"apm_12v_in_band", "12 V in band", f::OnOff, "ACCESSORY POWER MODULE"},
    {"apm_setpoint_high", "Setpoint command", FormatApmSetpoint, "ACCESSORY POWER MODULE"},
    {"apm_setpoint_stale", "Setpoint command stale", f::OnOff, "ACCESSORY POWER MODULE"},
    {"apm_subsystems", "Subsystems", FormatApmSubsystems, "ACCESSORY POWER MODULE"},
    {"apm_lamp_requests", "Lamp requests", FormatApmLampRequests, "ACCESSORY POWER MODULE"},
    {"apm_active_faults", "Active internal faults", f::Count, "ACCESSORY POWER MODULE"},
    {"apm_windshield_feedback_v", "Windshield heat feedback", f::Volts,
     "ACCESSORY POWER MODULE"},
    {"apm_load_shed_count", "Load-shed activations", f::Count, "ACCESSORY POWER MODULE"},
    {"htcm_wake_up", "Climate controller wake-up", f::OnOff, "OTHER MODULES"},
    {"ipc_active_codes", "Cluster active trouble codes", f::Count, "OTHER MODULES"},
}};

namespace {

struct NamedBit {
    unsigned bit;
    const char* name;
};

template <std::size_t N>
std::string MaskNames(double v, const NamedBit (&names)[N]) {
    const unsigned bits = static_cast<unsigned>(v);
    std::string out;
    auto add = [&out](const std::string& name) {
        if (!out.empty()) out += ", ";
        out += name;
    };
    unsigned known = 0;
    for (const auto& n : names) {
        known |= n.bit;
        if (bits & n.bit) add(n.name);
    }
    for (int i = 0; i < 8; ++i) {
        const unsigned bit = 1u << i;
        if ((bits & bit) && !(known & bit)) add("BIT " + std::to_string(i));
    }
    return out.empty() ? "none" : out;
}

}  // namespace

std::string FormatApmRunMode(double v) {
    static const char* const kNames[] = {"OFF", "ACCESSORY", "RUN", "CHARGE", "LOCK"};
    const int mode = static_cast<int>(v);
    if (mode >= 0 && mode < 5) return kNames[mode];
    return "MODE " + std::to_string(mode);
}

std::string FormatApmSetpoint(double v) { return v != 0.0 ? "HIGH (14.5 V)" : "LOW"; }

std::string FormatApmSubsystems(double v) {
    static constexpr NamedBit kNames[] = {
        {apm_scan::kSubsysRegulating, "REGULATING"},
        {apm_scan::kSubsysLoadShedActive, "LOAD SHED"},
        {apm_scan::kSubsysHvPresent, "HV PRESENT"},
    };
    return MaskNames(v, kNames);
}

std::string FormatApmLampRequests(double v) {
    static constexpr NamedBit kNames[] = {
        {apm_scan::kLampServiceSoon, "SERVICE SOON"},
        {apm_scan::kLampServiceNow, "SERVICE NOW"},
        {apm_scan::kLampTemp, "TEMP"},
    };
    return MaskNames(v, kNames);
}

std::string FormatDriveReading(DriveReading r, std::optional<double> value) {
    return FormatReadoutRow(kDriveReadingInfo[static_cast<int>(r)], value);
}

std::vector<std::string> DriveReadoutLines(const DriveReadings& readings) {
    return ReadoutLines(kDriveReadingInfo.data(), readings.data(), readings.size());
}

std::optional<DriveReading> DriveReadingForColumn(const std::string& column) {
    const auto i =
        ReadoutRowForColumn(kDriveReadingInfo.data(), kDriveReadingInfo.size(), "drive_", column);
    if (!i) return std::nullopt;
    return static_cast<DriveReading>(*i);
}

void DrawDriveReadout(irr::IrrlichtDevice* device, const DriveReadings& readings) {
    DrawReadoutPanel(device, "DRIVE AND POWER                 [J next page]",
                     DriveReadoutLines(readings));
}

}  // namespace ev1sim
