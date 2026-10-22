#include "WiperRenderer.h"

#include <cmath>
#include <cstring>
#include <cwchar>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Irrlicht headers — only needed for DrawHUD / ApplyToScene, both of which
// are guarded by a device/node pointer check, so the headers compile fine in
// headless unit-test builds too.
#include "irrlicht.h"

using namespace irr;
using namespace irr::scene;
using namespace irr::video;

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

WiperRenderer::WiperRenderer() = default;

// ---------------------------------------------------------------------------
// Cadence table
// ---------------------------------------------------------------------------

double WiperRenderer::CadenceForMode(Mode mode) {
    switch (mode) {
        case Mode::INT:  return kCadenceInt;
        case Mode::LOW:  return kCadenceLow;
        case Mode::HIGH: return kCadenceHigh;
        default:         return 0.0;
    }
}

// ---------------------------------------------------------------------------
// Tick
// ---------------------------------------------------------------------------

void WiperRenderer::Tick(double dt_s, std::uint8_t motor_cmd_raw) {
    Mode mode = Mode::OFF;
    switch (motor_cmd_raw) {
        case 1: mode = Mode::INT;  break;
        case 2: mode = Mode::LOW;  break;
        case 3: mode = Mode::HIGH; break;
        default: mode = Mode::OFF; break;
    }
    Tick(dt_s, mode);
}

void WiperRenderer::Tick(double dt_s, Mode mode) {
    m_mode = mode;

    const double cadence = CadenceForMode(mode);
    if (cadence <= 0.0 || dt_s <= 0.0) {
        // OFF — phase frozen; blade stays at park position.
        return;
    }

    // One revolution (2π) corresponds to one complete out-and-back sweep.
    // phase_dot = cadence_per_minute × (1/60) × 2π  rad/s
    const double phase_dot = (cadence / 60.0) * (2.0 * M_PI);
    m_phase_rad += phase_dot * dt_s;

    // Wrap to [0, 2π) to keep accumulator bounded across any dt.
    m_phase_rad = std::fmod(m_phase_rad, 2.0 * M_PI);
    if (m_phase_rad < 0.0)
        m_phase_rad += 2.0 * M_PI;
}

// ---------------------------------------------------------------------------
// Scene-graph
// ---------------------------------------------------------------------------

bool WiperRenderer::FindWiperNode(ISceneNode* root) {
    // The EV1 chassis model (ev1_chassis_vis.obj) has no wiper mesh.
    // This method always returns false until wiper geometry is added.
    (void)root;
    m_wiper_node = nullptr;
    return false;
}

void WiperRenderer::ApplyToScene() {
    if (!m_wiper_node) return;
    // TODO: when a wiper mesh node exists, apply rotation here:
    //   float angle_deg = GetAngleDeg();
    //   m_wiper_node->setRotation(core::vector3df(0.0f, angle_deg, 0.0f));
}

// ---------------------------------------------------------------------------
// Angle accessor
// ---------------------------------------------------------------------------

double WiperRenderer::SweepFromPlantPosition(std::uint8_t position_pct) {
    const double p = position_pct > 100 ? 100.0 : static_cast<double>(position_pct);
    return p <= 50.0 ? p / 50.0 : (100.0 - p) / 50.0;
}

double WiperRenderer::GetSweepFraction() const {
    if (m_plant_position_pct) return SweepFromPlantPosition(*m_plant_position_pct);
    return (1.0 - std::cos(m_phase_rad)) / 2.0;
}

float WiperRenderer::GetAngleDeg() const {
    return static_cast<float>(std::sin(m_phase_rad)) * m_max_angle_deg;
}

// ---------------------------------------------------------------------------
// HUD
// ---------------------------------------------------------------------------

// Build the sweep bar for non-OFF modes: 8 cells, park at the left, the far
// end of the sweep at the right, the blade's cell marked. Plain ASCII: the
// HUD's built-in Irrlicht font has no box-drawing or arrow glyphs.
static void BuildSweepBar(double sweep01, wchar_t* out, int out_len) {
    constexpr int kCells = 8;
    const int pos = static_cast<int>(sweep01 * (kCells - 1) + 0.5);
    const int clamped = (pos < 0) ? 0 : (pos >= kCells ? kCells - 1 : pos);

    wchar_t bar[kCells + 1];
    for (int c = 0; c < kCells; ++c) bar[c] = (c == clamped) ? L'#' : L'-';
    bar[kCells] = L'\0';

    // Format: "[--#-----]"
    int written = 0;
    out[written++] = L'[';
    for (int c = 0; c < kCells && written < out_len - 2; ++c)
        out[written++] = bar[c];
    out[written++] = L']';
    out[written]   = L'\0';
}

void WiperRenderer::DrawHUD(irr::IrrlichtDevice* device) const {
    if (!device) return;

    auto* driver = device->getVideoDriver();
    auto* gui    = device->getGUIEnvironment();
    auto* font   = gui->getBuiltInFont();
    if (!driver || !font) return;

    auto screen = driver->getScreenSize();

    // WIPER panel: sits below the LIGHTS panel.
    // LIGHTS panel: bottom-right, W=120 H=200, margin=12.
    // We go 8 px above the LIGHTS panel's top edge (LIGHTS oy = H - 200 - 12,
    // so WIPER bot = H - 200 - 12 - 8 = H - 220).
    const int W      = 120;
    const int H      = 36;
    const int margin = 12;
    const int ox     = screen.Width - W - margin;
    const int oy     = screen.Height - 220 - H;   // just above the LIGHTS panel

    // Background + border.
    driver->draw2DRectangle(SColor(140, 10, 10, 20),
        irr::core::recti(ox, oy, ox + W, oy + H));
    driver->draw2DRectangleOutline(
        irr::core::recti(ox, oy, ox + W, oy + H),
        SColor(200, 60, 60, 80));

    // Title line.
    font->draw(L"WIPER", irr::core::recti(ox, oy + 2, ox + W, oy + 14),
               SColor(255, 180, 180, 200), true, false);

    // Mode / animation line.
    wchar_t label[64];
    switch (m_mode) {
        case Mode::OFF:
            ::wcsncpy(label, L"OFF", 63);
            break;
        case Mode::INT:
        case Mode::LOW:
        case Mode::HIGH: {
            const wchar_t* mode_str =
                (m_mode == Mode::INT)  ? L"INT " :
                (m_mode == Mode::LOW)  ? L"LOW " : L"HIGH";
            wchar_t bar[16];
            BuildSweepBar(GetSweepFraction(), bar, 16);
            // Combine: "INT [--#-----]"
            ::wcsncpy(label, mode_str, 63);
            ::wcsncat(label, bar, 63 - static_cast<int>(::wcslen(label)));
            break;
        }
        default:
            ::wcsncpy(label, L"???", 63);
            break;
    }
    label[63] = L'\0';
    font->draw(label, irr::core::recti(ox, oy + 17, ox + W, oy + 30),
               SColor(255, 200, 220, 200), true, false);
}
