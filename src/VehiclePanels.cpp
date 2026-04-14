#include "VehiclePanels.h"

#include <iostream>

using irr::video::SColor;

VehiclePanels::VehiclePanels() {
    m_panels[static_cast<int>(PanelID::HOOD)]       = {"HOOD"};
    m_panels[static_cast<int>(PanelID::TRUNK)]      = {"TRUNK"};
    m_panels[static_cast<int>(PanelID::DOOR_LEFT)]  = {"DOOR_L"};
    m_panels[static_cast<int>(PanelID::DOOR_RIGHT)] = {"DOOR_R"};
}

void VehiclePanels::Toggle(PanelID panel) {
    int idx = static_cast<int>(panel);
    if (idx < 0 || idx >= NUM_PANELS) return;

    auto& p = m_panels[idx];
    p.open = !p.open;
    std::cout << "[VehiclePanels] " << p.name << ": "
              << (p.open ? "OPEN" : "CLOSED") << "\n";
}

bool VehiclePanels::IsOpen(PanelID panel) const {
    int idx = static_cast<int>(panel);
    return (idx >= 0 && idx < NUM_PANELS) ? m_panels[idx].open : false;
}

// ---------------------------------------------------------------------------
// HUD — panel status on the top-down vehicle diagram
// ---------------------------------------------------------------------------

void VehiclePanels::DrawHUD(irr::IrrlichtDevice* device) const {
    if (!device) return;

    auto* driver = device->getVideoDriver();
    auto* gui    = device->getGUIEnvironment();
    auto* font   = gui->getBuiltInFont();
    if (!driver || !font) return;

    auto screen = driver->getScreenSize();

    // Match the VehicleLights HUD panel position and car geometry.
    const int W = 120;
    const int H = 200;
    const int margin = 12;
    const int ox = screen.Width - W - margin;
    const int oy = screen.Height - H - margin;
    const int cx = ox + W / 2;
    const int car_top = oy + 22;
    const int car_bot = oy + H - 10;
    const int car_hw = 22;

    SColor closed_color(120, 40, 50, 40);
    SColor open_color(220, 60, 220, 60);

    // Hood — horizontal bar at front of car.
    {
        SColor c = IsOpen(PanelID::HOOD) ? open_color : closed_color;
        int y = car_top + 14;
        driver->draw2DRectangle(c,
            irr::core::recti(cx - 10, y, cx + 10, y + 3));
    }

    // Trunk — horizontal bar at rear of car.
    {
        SColor c = IsOpen(PanelID::TRUNK) ? open_color : closed_color;
        int y = car_bot - 16;
        driver->draw2DRectangle(c,
            irr::core::recti(cx - 10, y, cx + 10, y + 3));
    }

    // Left door — vertical bar on left side.
    {
        SColor c = IsOpen(PanelID::DOOR_LEFT) ? open_color : closed_color;
        int x = cx - car_hw - 1;
        int y1 = car_top + 45;
        int y2 = car_top + 85;
        driver->draw2DRectangle(c,
            irr::core::recti(x - 2, y1, x, y2));
    }

    // Right door — vertical bar on right side.
    {
        SColor c = IsOpen(PanelID::DOOR_RIGHT) ? open_color : closed_color;
        int x = cx + car_hw + 1;
        int y1 = car_top + 45;
        int y2 = car_top + 85;
        driver->draw2DRectangle(c,
            irr::core::recti(x, y1, x + 2, y2));
    }
}
