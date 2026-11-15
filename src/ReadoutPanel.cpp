#include "ReadoutPanel.h"

#include <irrlicht.h>

#include <cstdio>

namespace ev1sim {

namespace readout_format {

namespace {
std::string Printf(const char* fmt, double v) {
    char buf[48];
    std::snprintf(buf, sizeof(buf), fmt, v);
    return buf;
}
}  // namespace

std::string DegC(double v) { return Printf("%.1f C", v); }
std::string Kpa(double v) { return Printf("%.0f kPa", v); }
std::string Volts(double v) { return Printf("%.1f V", v); }
std::string Percent(double v) { return Printf("%.0f %%", v); }
std::string Hz(double v) { return Printf("%.0f Hz", v); }
std::string OnOff(double v) { return v != 0.0 ? "ON" : "OFF"; }
std::string OfFull255(double v) { return Printf("%.0f of 255", v); }
std::string Count(double v) { return Printf("%.0f", v); }
std::string Kohm(double v) { return Printf("%.0f kohm", v); }
std::string RadPerSec(double v) { return Printf("%.1f rad/s", v); }
std::string KphFromMps(double v) { return Printf("%.1f km/h", v * 3.6); }

}  // namespace readout_format

std::string FormatReadoutRow(const ReadoutRow& row, std::optional<double> value) {
    return std::string(row.label) + ": " + (value ? row.format(*value) : std::string("---"));
}

std::vector<std::string> ReadoutLines(const ReadoutRow* rows, const std::optional<double>* values,
                                      std::size_t count) {
    std::vector<std::string> lines;
    const char* group = nullptr;
    for (std::size_t i = 0; i < count; ++i) {
        if (group == nullptr || std::string(group) != rows[i].group) {
            group = rows[i].group;
            lines.emplace_back(group);
        }
        lines.push_back("  " + FormatReadoutRow(rows[i], values[i]));
    }
    return lines;
}

std::optional<std::size_t> ReadoutRowForColumn(const ReadoutRow* rows, std::size_t count,
                                               const std::string& prefix,
                                               const std::string& column) {
    if (column.compare(0, prefix.size(), prefix) != 0) return std::nullopt;
    const std::string rest = column.substr(prefix.size());
    for (std::size_t i = 0; i < count; ++i)
        if (rest == rows[i].column) return i;
    return std::nullopt;
}

ReadoutPage NextReadoutPage(ReadoutPage page) {
    switch (page) {
        case ReadoutPage::kNone:    return ReadoutPage::kClimate;
        case ReadoutPage::kClimate: return ReadoutPage::kDrive;
        case ReadoutPage::kDrive:   return ReadoutPage::kNone;
    }
    return ReadoutPage::kNone;
}

void DrawReadoutPanel(irr::IrrlichtDevice* device, const std::string& title,
                      const std::vector<std::string>& lines) {
    if (device == nullptr) return;
    irr::video::IVideoDriver* drv = device->getVideoDriver();
    irr::gui::IGUIEnvironment* gui = device->getGUIEnvironment();
    irr::gui::IGUIFont* font = gui ? gui->getBuiltInFont() : nullptr;
    if (drv == nullptr || font == nullptr) return;

    using irr::video::SColor;
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
    draw(title, SColor(255, 255, 220, 80));
    for (const std::string& line : lines) {
        const bool header = line.empty() || line[0] != ' ';
        const bool unknown = line.size() >= 3 && line.compare(line.size() - 3, 3, "---") == 0;
        draw(line, header ? SColor(255, 150, 190, 255)
                          : unknown ? SColor(255, 130, 130, 130) : SColor(255, 220, 220, 220));
    }
}

}  // namespace ev1sim
