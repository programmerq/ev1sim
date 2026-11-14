#include "ClimateReadout.h"

#include <irrlicht.h>

#include <cmath>
#include <cstdint>
#include <cstdio>

namespace ev1sim {

const std::array<ClimateReadingInfo, kNumClimateReadings> kClimateReadingInfo = {{
    {"outside_air_temp_c", "Outside air", ClimateUnit::kDegC, "SENSORS"},
    {"inlet_temp_c", "Cabin inlet air", ClimateUnit::kDegC, "SENSORS"},
    {"pack_temp_c", "Battery pack", ClimateUnit::kDegC, "SENSORS"},
    {"pim_temp_c", "Inverter (PIM)", ClimateUnit::kDegC, "SENSORS"},
    {"coolant_temp_c", "Coolant", ClimateUnit::kDegC, "SENSORS"},
    {"coolant_pump_temp_c", "Coolant at pump", ClimateUnit::kDegC, "SENSORS"},
    {"drive_motor_coolant_temp_c", "Drive motor coolant", ClimateUnit::kDegC, "SENSORS"},
    {"coolant_level_low", "Coolant level low", ClimateUnit::kOnOff, "SENSORS"},
    {"refrig_low_kpa", "Refrigerant low side", ClimateUnit::kKpa, "SENSORS"},
    {"refrig_high_kpa", "Refrigerant high side", ClimateUnit::kKpa, "SENSORS"},
    {"refrig_temp_c", "Refrigerant", ClimateUnit::kDegC, "SENSORS"},
    {"refrig_discharge_temp_c", "Compressor discharge", ClimateUnit::kDegC, "SENSORS"},
    {"mode_door", "Mode door", ClimateUnit::kPosition, "CONTROLLER OUTPUTS"},
    {"outside_air_valve", "Outside-air valve", ClimateUnit::kOnOff, "CONTROLLER OUTPUTS"},
    {"reversing_valve", "Heat-pump reversing valve", ClimateUnit::kOnOff, "CONTROLLER OUTPUTS"},
    {"three_way_coolant_valve", "Three-way coolant valve", ClimateUnit::kOnOff,
     "CONTROLLER OUTPUTS"},
    {"coolant_recovery_valve", "Coolant recovery valve", ClimateUnit::kOnOff,
     "CONTROLLER OUTPUTS"},
    {"heater_valve_feed", "Heater valve feed", ClimateUnit::kOnOff, "CONTROLLER OUTPUTS"},
    {"coolant_pump_feed", "Coolant pump feed", ClimateUnit::kOnOff, "CONTROLLER OUTPUTS"},
    {"compressor_duty_pct", "Speed command duty", ClimateUnit::kPercent, "COMPRESSOR DRIVE"},
    {"compressor_freq_hz", "Target drive frequency", ClimateUnit::kHz, "COMPRESSOR DRIVE"},
    {"compressor_drive_enabled", "Drive enabled", ClimateUnit::kOnOff, "COMPRESSOR DRIVE"},
    {"compressor_interlock_closed", "Motor interlock closed", ClimateUnit::kOnOff,
     "COMPRESSOR DRIVE"},
    {"compressor_inverter_fault", "Inverter fault", ClimateUnit::kOnOff, "COMPRESSOR DRIVE"},
    {"windshield_heat_commanded", "Commanded", ClimateUnit::kOnOff, "WINDSHIELD HEAT DRIVE"},
    {"windshield_heat_driving", "Driving", ClimateUnit::kOnOff, "WINDSHIELD HEAT DRIVE"},
    {"windshield_rail_v", "Output rail", ClimateUnit::kVolts, "WINDSHIELD HEAT DRIVE"},
}};

double DegCFromQ8Cell(unsigned bits) {
    return static_cast<std::int16_t>(static_cast<std::uint16_t>(bits)) / 256.0;
}

std::string FormatClimateReading(ClimateReading r, std::optional<double> value) {
    const ClimateReadingInfo& info = kClimateReadingInfo[static_cast<int>(r)];
    std::string out = std::string(info.label) + ": ";
    if (!value) return out + "---";
    char buf[48];
    switch (info.unit) {
        case ClimateUnit::kDegC:    std::snprintf(buf, sizeof(buf), "%.1f C", *value); break;
        case ClimateUnit::kKpa:     std::snprintf(buf, sizeof(buf), "%.0f kPa", *value); break;
        case ClimateUnit::kVolts:   std::snprintf(buf, sizeof(buf), "%.1f V", *value); break;
        case ClimateUnit::kPercent: std::snprintf(buf, sizeof(buf), "%.0f %%", *value); break;
        case ClimateUnit::kHz:      std::snprintf(buf, sizeof(buf), "%.0f Hz", *value); break;
        case ClimateUnit::kOnOff:   std::snprintf(buf, sizeof(buf), "%s", *value != 0.0 ? "ON" : "OFF"); break;
        case ClimateUnit::kPosition:
            std::snprintf(buf, sizeof(buf), "%.0f of 255", *value);
            break;
    }
    return out + buf;
}

std::vector<std::string> ClimateReadoutLines(const ClimateReadings& readings) {
    std::vector<std::string> lines;
    const char* group = nullptr;
    for (int i = 0; i < kNumClimateReadings; ++i) {
        const ClimateReadingInfo& info = kClimateReadingInfo[i];
        if (group == nullptr || std::string(group) != info.group) {
            group = info.group;
            lines.emplace_back(group);
        }
        lines.push_back("  " + FormatClimateReading(static_cast<ClimateReading>(i), readings[i]));
    }
    return lines;
}

std::optional<ClimateReading> ClimateReadingForColumn(const std::string& column) {
    static const std::string kPrefix = "climate_";
    if (column.compare(0, kPrefix.size(), kPrefix) != 0) return std::nullopt;
    const std::string rest = column.substr(kPrefix.size());
    for (int i = 0; i < kNumClimateReadings; ++i)
        if (rest == kClimateReadingInfo[i].column) return static_cast<ClimateReading>(i);
    return std::nullopt;
}

void DrawClimateReadout(irr::IrrlichtDevice* device, const ClimateReadings& readings) {
    if (device == nullptr) return;
    irr::video::IVideoDriver* drv = device->getVideoDriver();
    irr::gui::IGUIEnvironment* gui = device->getGUIEnvironment();
    irr::gui::IGUIFont* font = gui ? gui->getBuiltInFont() : nullptr;
    if (drv == nullptr || font == nullptr) return;

    using irr::video::SColor;
    const std::vector<std::string> lines = ClimateReadoutLines(readings);
    const int line_h = 13, pad = 6, panel_w = 300, margin = 12, top = 234;
    const int panel_h = pad * 2 + line_h * static_cast<int>(lines.size() + 1);
    const auto screen = drv->getScreenSize();
    const int px = static_cast<int>(screen.Width) - panel_w - margin;
    drv->draw2DRectangle(SColor(190, 15, 15, 25),
                         irr::core::recti(px, top, px + panel_w, top + panel_h));
    int ty = top + pad;
    auto draw = [&](const std::string& text, SColor col) {
        const irr::core::stringw ws(text.c_str());
        font->draw(ws, irr::core::recti(px + pad, ty, px + panel_w - pad, ty + line_h), col);
        ty += line_h;
    };
    draw("CLIMATE                         [J to toggle]", SColor(255, 255, 220, 80));
    for (const std::string& line : lines) {
        const bool header = line.empty() || line[0] != ' ';
        const bool unknown = line.size() >= 3 && line.compare(line.size() - 3, 3, "---") == 0;
        draw(line, header ? SColor(255, 150, 190, 255)
                          : unknown ? SColor(255, 130, 130, 130) : SColor(255, 220, 220, 220));
    }
}

}  // namespace ev1sim
