#include "ClimateReadout.h"

#include <cstdint>

namespace ev1sim {

namespace f = readout_format;

const std::array<ReadoutRow, kNumClimateReadings> kClimateReadingInfo = {{
    {"outside_air_temp_c", "Outside air", f::DegC, "SENSORS"},
    {"inlet_temp_c", "Cabin inlet air", f::DegC, "SENSORS"},
    {"pack_temp_c", "Battery pack", f::DegC, "SENSORS"},
    {"pim_temp_c", "Inverter (PIM)", f::DegC, "SENSORS"},
    {"coolant_temp_c", "Coolant", f::DegC, "SENSORS"},
    {"coolant_pump_temp_c", "Coolant at pump", f::DegC, "SENSORS"},
    {"drive_motor_coolant_temp_c", "Drive motor coolant", f::DegC, "SENSORS"},
    {"coolant_level_low", "Coolant level low", f::OnOff, "SENSORS"},
    {"refrig_low_kpa", "Refrigerant low side", f::Kpa, "SENSORS"},
    {"refrig_high_kpa", "Refrigerant high side", f::Kpa, "SENSORS"},
    {"refrig_temp_c", "Refrigerant", f::DegC, "SENSORS"},
    {"refrig_discharge_temp_c", "Compressor discharge", f::DegC, "SENSORS"},
    {"mode_door", "Mode door", f::OfFull255, "CONTROLLER OUTPUTS"},
    {"outside_air_valve", "Outside-air valve", f::OnOff, "CONTROLLER OUTPUTS"},
    {"reversing_valve", "Heat-pump reversing valve", f::OnOff, "CONTROLLER OUTPUTS"},
    {"three_way_coolant_valve", "Three-way coolant valve", f::OnOff, "CONTROLLER OUTPUTS"},
    {"coolant_recovery_valve", "Coolant recovery valve", f::OnOff, "CONTROLLER OUTPUTS"},
    {"heater_valve_feed", "Heater valve feed", f::OnOff, "CONTROLLER OUTPUTS"},
    {"coolant_pump_feed", "Coolant pump feed", f::OnOff, "CONTROLLER OUTPUTS"},
    {"compressor_duty_pct", "Speed command duty", f::Percent, "COMPRESSOR DRIVE"},
    {"compressor_freq_hz", "Target drive frequency", f::Hz, "COMPRESSOR DRIVE"},
    {"compressor_drive_enabled", "Drive enabled", f::OnOff, "COMPRESSOR DRIVE"},
    {"compressor_interlock_closed", "Motor interlock closed", f::OnOff, "COMPRESSOR DRIVE"},
    {"compressor_inverter_fault", "Inverter fault", f::OnOff, "COMPRESSOR DRIVE"},
    {"windshield_heat_commanded", "Commanded", f::OnOff, "WINDSHIELD HEAT DRIVE"},
    {"windshield_heat_driving", "Driving", f::OnOff, "WINDSHIELD HEAT DRIVE"},
    {"windshield_rail_v", "Output rail", f::Volts, "WINDSHIELD HEAT DRIVE"},
}};

double DegCFromQ8Cell(unsigned bits) {
    return static_cast<std::int16_t>(static_cast<std::uint16_t>(bits)) / 256.0;
}

std::string FormatClimateReading(ClimateReading r, std::optional<double> value) {
    return FormatReadoutRow(kClimateReadingInfo[static_cast<int>(r)], value);
}

std::vector<std::string> ClimateReadoutLines(const ClimateReadings& readings) {
    return ReadoutLines(kClimateReadingInfo.data(), readings.data(), readings.size());
}

std::optional<ClimateReading> ClimateReadingForColumn(const std::string& column) {
    const auto i = ReadoutRowForColumn(kClimateReadingInfo.data(), kClimateReadingInfo.size(),
                                       "climate_", column);
    if (!i) return std::nullopt;
    return static_cast<ClimateReading>(*i);
}

void DrawClimateReadout(irr::IrrlichtDevice* device, const ClimateReadings& readings) {
    DrawReadoutPanel(device, "CLIMATE                         [J next page]",
                     ClimateReadoutLines(readings));
}

}  // namespace ev1sim
