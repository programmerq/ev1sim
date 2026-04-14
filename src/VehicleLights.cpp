#include "VehicleLights.h"

#include <cmath>
#include <iostream>

using namespace irr;
using namespace irr::scene;
using namespace irr::video;

// ---------------------------------------------------------------------------
// Light ID names
// ---------------------------------------------------------------------------

const char* LightIDName(LightID id) {
    switch (id) {
        case LightID::LHBH:  return "LHBH";
        case LightID::LLBH:  return "LLBH";
        case LightID::RHBH:  return "RHBH";
        case LightID::RLBH:  return "RLBH";
        case LightID::LFTS:  return "LFTS";
        case LightID::RFTS:  return "RFTS";
        case LightID::LFML:  return "LFML";
        case LightID::RFML:  return "RFML";
        case LightID::LRSL:  return "LRSL";
        case LightID::RRSL:  return "RRSL";
        case LightID::LRTL:  return "LRTL";
        case LightID::RRTL:  return "RRTL";
        case LightID::LRTS:  return "LRTS";
        case LightID::RRTS:  return "RRTS";
        case LightID::LRSM:  return "LRSM";
        case LightID::RRSM:  return "RRSM";
        case LightID::CHMSL: return "CHMSL";
        case LightID::LBL:   return "LBL";
        case LightID::RBL:   return "RBL";
        default:              return "???";
    }
}

// ---------------------------------------------------------------------------
// Lamp group names (for debug output)
// ---------------------------------------------------------------------------

const char* VehicleLights::GroupName(LampGroup g) {
    switch (g) {
        case LampGroup::HEADLAMP_HI_LEFT:         return "HEADLAMP_HI_LEFT";
        case LampGroup::HEADLAMP_HI_RIGHT:        return "HEADLAMP_HI_RIGHT";
        case LampGroup::HEADLAMP_LO_LEFT:         return "HEADLAMP_LO_LEFT";
        case LampGroup::HEADLAMP_LO_RIGHT:        return "HEADLAMP_LO_RIGHT";
        case LampGroup::SIGNAL_FRONT_TURN_LEFT:   return "SIGNAL_FRONT_TURN_LEFT";
        case LampGroup::SIGNAL_FRONT_TURN_RIGHT:  return "SIGNAL_FRONT_TURN_RIGHT";
        case LampGroup::SIGNAL_FRONT_MARKER_LEFT: return "SIGNAL_FRONT_MARKER_LEFT";
        case LampGroup::SIGNAL_FRONT_MARKER_RIGHT:return "SIGNAL_FRONT_MARKER_RIGHT";
        case LampGroup::STOPLAMP_LEFT:            return "STOPLAMP_LEFT";
        case LampGroup::STOPLAMP_RIGHT:           return "STOPLAMP_RIGHT";
        case LampGroup::SIGNAL_REAR_LEFT:         return "SIGNAL_REAR_LEFT";
        case LampGroup::SIGNAL_REAR_RIGHT:        return "SIGNAL_REAR_RIGHT";
        case LampGroup::SIDE_MARKER_REAR_LEFT:    return "SIDE_MARKER_REAR_LEFT";
        case LampGroup::SIDE_MARKER_REAR_RIGHT:   return "SIDE_MARKER_REAR_RIGHT";
        case LampGroup::BACKUP_LEFT:              return "BACKUP_LEFT";
        case LampGroup::BACKUP_RIGHT:             return "BACKUP_RIGHT";
        case LampGroup::CHMSL:                    return "CHMSL";
        default:                                   return "???";
    }
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

VehicleLights::VehicleLights() {
    for (int i = 0; i < NUM_LIGHTS; ++i)
        m_state[i] = false;
    for (int i = 0; i < NUM_GROUPS; ++i)
        m_groups[i].buffer_index = -1;
}

// ---------------------------------------------------------------------------
// LightID -> LampGroup mapping
// ---------------------------------------------------------------------------

VehicleLights::LampGroup VehicleLights::GroupFor(LightID id) {
    switch (id) {
        case LightID::LHBH:  return LampGroup::HEADLAMP_HI_LEFT;
        case LightID::LLBH:  return LampGroup::HEADLAMP_LO_LEFT;
        case LightID::RHBH:  return LampGroup::HEADLAMP_HI_RIGHT;
        case LightID::RLBH:  return LampGroup::HEADLAMP_LO_RIGHT;
        case LightID::LFTS:  return LampGroup::SIGNAL_FRONT_TURN_LEFT;
        case LightID::RFTS:  return LampGroup::SIGNAL_FRONT_TURN_RIGHT;
        case LightID::LFML:  return LampGroup::SIGNAL_FRONT_MARKER_LEFT;
        case LightID::RFML:  return LampGroup::SIGNAL_FRONT_MARKER_RIGHT;
        case LightID::LRSL:  return LampGroup::STOPLAMP_LEFT;
        case LightID::RRSL:  return LampGroup::STOPLAMP_RIGHT;
        case LightID::LRTL:  return LampGroup::STOPLAMP_LEFT;   // shares lens
        case LightID::RRTL:  return LampGroup::STOPLAMP_RIGHT;  // shares lens
        case LightID::LRTS:  return LampGroup::SIGNAL_REAR_LEFT;
        case LightID::RRTS:  return LampGroup::SIGNAL_REAR_RIGHT;
        case LightID::LRSM:  return LampGroup::SIDE_MARKER_REAR_LEFT;
        case LightID::RRSM:  return LampGroup::SIDE_MARKER_REAR_RIGHT;
        case LightID::CHMSL: return LampGroup::CHMSL;
        case LightID::LBL:   return LampGroup::BACKUP_LEFT;
        case LightID::RBL:   return LampGroup::BACKUP_RIGHT;
        default:              return LampGroup::HEADLAMP_HI_LEFT;
    }
}

// ---------------------------------------------------------------------------
// State access
// ---------------------------------------------------------------------------

void VehicleLights::SetState(LightID id, bool on) {
    int idx = static_cast<int>(id);
    if (idx >= 0 && idx < NUM_LIGHTS)
        m_state[idx] = on;
}

bool VehicleLights::GetState(LightID id) const {
    int idx = static_cast<int>(id);
    return (idx >= 0 && idx < NUM_LIGHTS) ? m_state[idx] : false;
}

// ---------------------------------------------------------------------------
// Demo mode -- blink each light at a different frequency
// ---------------------------------------------------------------------------

void VehicleLights::UpdateDemoMode(double sim_time) {
    for (int i = 0; i < NUM_LIGHTS; ++i) {
        double period = 1.0 / BLINK_FREQ[i];
        double phase  = std::fmod(sim_time, period);
        m_state[i] = (phase < period * 0.5);
    }
}

// ---------------------------------------------------------------------------
// Classify glass type from Irrlicht material properties
// ---------------------------------------------------------------------------

VehicleLights::GlassType VehicleLights::ClassifyGlass(const SMaterial& mat) {
    // Light materials have d < 1.0 in the MTL -> alpha < 255.
    // Reflectors (redglass) are set to d=1.0 (opaque) so they are skipped.
    u32 alpha = mat.DiffuseColor.getAlpha();
    if (alpha >= 250)
        return GlassType::NONE;

    u32 r = mat.DiffuseColor.getRed();
    u32 g = mat.DiffuseColor.getGreen();
    u32 b = mat.DiffuseColor.getBlue();

    // Orange glass: Kd 0.70 0.30 0.00 -> (179, 77, 0)
    if (r > 120 && g > 40 && g < 120 && b < 30)
        return GlassType::ORANGE;

    // Red glass: Kd 0.50 0.05 0.05 -> (128, 13, 13)
    if (r > 80 && g < 50 && b < 50)
        return GlassType::RED;

    // Clear glass: Kd 0.70 0.70 0.70 -> (179, 179, 179)
    if (r > 100 && g > 100 && b > 100)
        return GlassType::CLEAR;

    return GlassType::NONE;
}

// ---------------------------------------------------------------------------
// Compute emissive colour for a lamp group
// ---------------------------------------------------------------------------

SColor VehicleLights::ComputeGroupEmissive(LampGroup group) const {
    // Collect which bulbs in this group are on.
    bool any_on = false;
    bool stop_on = false;  // high-intensity stop lamp filament
    bool tail_on = false;  // low-intensity tail lamp filament

    for (int i = 0; i < NUM_LIGHTS; ++i) {
        LightID lid = static_cast<LightID>(i);
        if (GroupFor(lid) == group && m_state[i]) {
            any_on = true;
            if (lid == LightID::LRSL || lid == LightID::RRSL)
                stop_on = true;
            if (lid == LightID::LRTL || lid == LightID::RRTL)
                tail_on = true;
        }
    }

    if (!any_on)
        return SColor(0, 0, 0, 0);

    switch (group) {
        case LampGroup::HEADLAMP_HI_LEFT:
        case LampGroup::HEADLAMP_HI_RIGHT:
            return SColor(255, 220, 220, 255);  // Cool white (hi beam projector)

        case LampGroup::HEADLAMP_LO_LEFT:
        case LampGroup::HEADLAMP_LO_RIGHT:
            return SColor(255, 255, 240, 180);  // Warm white (lo beam)

        case LampGroup::SIGNAL_FRONT_TURN_LEFT:
        case LampGroup::SIGNAL_FRONT_TURN_RIGHT:
        case LampGroup::SIGNAL_FRONT_MARKER_LEFT:
        case LampGroup::SIGNAL_FRONT_MARKER_RIGHT:
        case LampGroup::SIGNAL_REAR_LEFT:
        case LampGroup::SIGNAL_REAR_RIGHT:
            return SColor(255, 255, 180, 20);  // Amber

        case LampGroup::STOPLAMP_LEFT:
        case LampGroup::STOPLAMP_RIGHT:
            // Dual-filament: stop (bright) vs tail (dim).
            if (stop_on)
                return SColor(255, 255, 40, 10);   // Bright red (braking)
            else
                return SColor(255, 120, 15, 5);    // Dim red (tail light)

        case LampGroup::CHMSL:
            return SColor(255, 255, 40, 10);  // Red

        case LampGroup::SIDE_MARKER_REAR_LEFT:
        case LampGroup::SIDE_MARKER_REAR_RIGHT:
            return SColor(255, 180, 25, 8);   // Dim red (side marker)

        case LampGroup::BACKUP_LEFT:
        case LampGroup::BACKUP_RIGHT:
            return SColor(255, 240, 240, 240);  // White

        default:
            return SColor(0, 0, 0, 0);
    }
}

// ---------------------------------------------------------------------------
// Scene graph traversal -- find the chassis mesh node
// ---------------------------------------------------------------------------

void VehicleLights::FindChassisMeshNode(ISceneNode* node) {
    if (!node) return;

    u32 mat_count = node->getMaterialCount();
    u32 best_count = m_chassis_node ? m_chassis_node->getMaterialCount() : 0;

    if (mat_count > best_count && mat_count >= 10) {
        m_chassis_node = node;
    }

    for (auto* child : node->getChildren())
        FindChassisMeshNode(child);
}

// ---------------------------------------------------------------------------
// Build mapping from lamp groups to mesh buffer indices
// ---------------------------------------------------------------------------

void VehicleLights::BuildBufferMapping() {
    if (!m_chassis_node) return;

    IMesh* mesh = nullptr;

    if (m_chassis_node->getType() == ESNT_MESH) {
        mesh = static_cast<IMeshSceneNode*>(m_chassis_node)->getMesh();
    } else if (m_chassis_node->getType() == ESNT_ANIMATED_MESH) {
        auto* amesh = static_cast<IAnimatedMeshSceneNode*>(m_chassis_node)->getMesh();
        if (amesh)
            mesh = amesh->getMesh(0);
    }

    if (!mesh) {
        std::cerr << "[VehicleLights] Could not access chassis mesh data\n";
        return;
    }

    u32 buf_count = mesh->getMeshBufferCount();
    std::cout << "[VehicleLights] Chassis mesh has " << buf_count << " material buffers\n";

    for (u32 i = 0; i < buf_count; ++i) {
        IMeshBuffer* buf = mesh->getMeshBuffer(i);
        u32 vert_count = buf->getVertexCount();

        if (vert_count == 0) continue;

        // Compute average vertex position for this buffer.
        // OBJ/Chrono coordinates: X=forward, Y=left, Z=up.
        float sum_x = 0, sum_y = 0, sum_z = 0;
        for (u32 v = 0; v < vert_count; ++v) {
            auto pos = buf->getPosition(v);
            sum_x += pos.X;
            sum_y += pos.Y;
            sum_z += pos.Z;
        }
        float avg_x = sum_x / vert_count;
        float avg_y = sum_y / vert_count;
        float avg_z = sum_z / vert_count;

        const SMaterial& mat = buf->getMaterial();
        GlassType glass = ClassifyGlass(mat);

        u32 r = mat.DiffuseColor.getRed();
        u32 g = mat.DiffuseColor.getGreen();
        u32 b = mat.DiffuseColor.getBlue();
        u32 a = mat.DiffuseColor.getAlpha();

        const char* glass_names[] = {"OPAQUE", "CLEAR", "ORANGE", "RED"};
        std::cout << "[VehicleLights]   Buffer " << i
                  << ": " << vert_count << " verts"
                  << "  avg=(" << avg_x << ", " << avg_y << ", " << avg_z << ")"
                  << "  rgba=(" << r << "," << g << "," << b << "," << a << ")"
                  << "  " << glass_names[static_cast<int>(glass)]
                  << "\n";

        if (glass == GlassType::NONE) continue;

        // Classify by glass type + position.
        //   X > 0 = front of car,  X < 0 = rear
        //   Y > 0 = left side,     Y <= 0 = right side
        LampGroup group;

        switch (glass) {
            case GlassType::CLEAR:
                if (avg_x > 0) {
                    // Front clear glass = headlamps.
                    // Inner half (|avg_y| < 0.5) = hi beam projector,
                    // outer half (|avg_y| >= 0.5) = lo beam.
                    if (avg_y > 0) {
                        group = (std::fabs(avg_y) > 0.5f)
                            ? LampGroup::HEADLAMP_LO_LEFT
                            : LampGroup::HEADLAMP_HI_LEFT;
                    } else {
                        group = (std::fabs(avg_y) > 0.5f)
                            ? LampGroup::HEADLAMP_LO_RIGHT
                            : LampGroup::HEADLAMP_HI_RIGHT;
                    }
                } else {
                    group = (avg_y > 0) ? LampGroup::BACKUP_LEFT
                                        : LampGroup::BACKUP_RIGHT;
                }
                break;

            case GlassType::ORANGE:
                if (avg_x > 0) {
                    // Front orange glass — sub-classify turn vs marker by
                    // X position.  The forward-facing turn signal faces
                    // have higher avg X (~1.98) than the side-facing
                    // marker faces (~1.83).
                    if (avg_x > 1.9f) {
                        group = (avg_y > 0) ? LampGroup::SIGNAL_FRONT_TURN_LEFT
                                            : LampGroup::SIGNAL_FRONT_TURN_RIGHT;
                    } else {
                        group = (avg_y > 0) ? LampGroup::SIGNAL_FRONT_MARKER_LEFT
                                            : LampGroup::SIGNAL_FRONT_MARKER_RIGHT;
                    }
                } else {
                    group = (avg_y > 0) ? LampGroup::SIGNAL_REAR_LEFT
                                        : LampGroup::SIGNAL_REAR_RIGHT;
                }
                break;

            case GlassType::RED:
                if (std::fabs(avg_y) < 0.15f) {
                    group = LampGroup::CHMSL;
                } else if (std::fabs(avg_y) > 0.65f) {
                    // Very outboard red glass = rear side markers.
                    group = (avg_y > 0) ? LampGroup::SIDE_MARKER_REAR_LEFT
                                        : LampGroup::SIDE_MARKER_REAR_RIGHT;
                } else {
                    group = (avg_y > 0) ? LampGroup::STOPLAMP_LEFT
                                        : LampGroup::STOPLAMP_RIGHT;
                }
                break;

            default:
                continue;
        }

        m_groups[static_cast<int>(group)].buffer_index = i;
        std::cout << "[VehicleLights]     -> " << GroupName(group) << "\n";
    }

    // Report unmapped groups
    for (int i = 0; i < NUM_GROUPS; ++i) {
        if (m_groups[i].buffer_index < 0) {
            std::cout << "[VehicleLights]   WARNING: "
                      << GroupName(static_cast<LampGroup>(i)) << " not mapped\n";
        }
    }
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

bool VehicleLights::Initialize(ISceneManager* smgr) {
    if (!smgr) return false;

    m_chassis_node = nullptr;
    FindChassisMeshNode(smgr->getRootSceneNode());

    if (!m_chassis_node) {
        std::cerr << "[VehicleLights] Chassis mesh node not found in scene graph\n";
        return false;
    }

    std::cout << "[VehicleLights] Found chassis node with "
              << m_chassis_node->getMaterialCount() << " materials\n";

    BuildBufferMapping();
    return true;
}

// ---------------------------------------------------------------------------
// Apply light states to Irrlicht materials
// ---------------------------------------------------------------------------

void VehicleLights::ApplyToScene() {
    if (!m_chassis_node) return;

    for (int g = 0; g < NUM_GROUPS; ++g) {
        int buf_idx = m_groups[g].buffer_index;
        if (buf_idx < 0) continue;

        SColor emissive = ComputeGroupEmissive(static_cast<LampGroup>(g));
        SMaterial& mat = m_chassis_node->getMaterial(buf_idx);
        mat.EmissiveColor = emissive;
    }
}

// ---------------------------------------------------------------------------
// HUD: top-down vehicle silhouette with bulb indicators
// ---------------------------------------------------------------------------

void VehicleLights::DrawHUD(irr::IrrlichtDevice* device) const {
    if (!device) return;

    auto* driver = device->getVideoDriver();
    auto* gui    = device->getGUIEnvironment();
    auto* font   = gui->getBuiltInFont();
    if (!driver || !font) return;

    auto screen = driver->getScreenSize();

    // Panel dimensions and position (bottom-right corner).
    const int W = 120;   // panel width
    const int H = 200;   // panel height
    const int margin = 12;
    const int ox = screen.Width - W - margin;   // origin X (top-left of panel)
    const int oy = screen.Height - H - margin;  // origin Y

    // Semi-transparent background.
    driver->draw2DRectangle(SColor(140, 10, 10, 20),
        irr::core::recti(ox, oy, ox + W, oy + H));
    // Border.
    driver->draw2DRectangleOutline(
        irr::core::recti(ox, oy, ox + W, oy + H),
        SColor(200, 60, 60, 80));

    // Car body outline — a simple rounded-ish shape using layered rects.
    const int cx = ox + W / 2;   // center X of car
    const int car_top = oy + 22;
    const int car_bot = oy + H - 10;
    const int car_h   = car_bot - car_top;
    const int car_hw  = 22;  // half-width of car body

    // Main body rect.
    driver->draw2DRectangleOutline(
        irr::core::recti(cx - car_hw, car_top + 8, cx + car_hw, car_bot - 8),
        SColor(160, 80, 80, 100));
    // Front taper.
    driver->draw2DRectangleOutline(
        irr::core::recti(cx - car_hw + 4, car_top, cx + car_hw - 4, car_top + 12),
        SColor(160, 80, 80, 100));
    // Rear taper.
    driver->draw2DRectangleOutline(
        irr::core::recti(cx - car_hw + 4, car_bot - 12, cx + car_hw - 4, car_bot),
        SColor(160, 80, 80, 100));

    // Title.
    font->draw(L"LIGHTS", irr::core::recti(ox, oy + 4, ox + W, oy + 16),
               SColor(255, 180, 180, 200), true, false);

    struct BulbPos { LightID id; int dx; int dy; };

    // Positions relative to car center / car_top.
    // Front of car is at top of diagram, rear at bottom.
    // Screen-left = car-left (driver's side).
    const int fwd = 4;           // front edge
    const int rear = car_h - 4;  // rear edge

    const BulbPos bulbs[] = {
        // Front headlamps: outer = lo beam, inner = hi beam
        { LightID::LLBH,  -15, fwd      },  // Left lo beam (outer)
        { LightID::LHBH,   -8, fwd      },  // Left hi beam (inner)
        { LightID::RHBH,    8, fwd      },  // Right hi beam (inner)
        { LightID::RLBH,   15, fwd      },  // Right lo beam (outer)
        // Front turn signals
        { LightID::LFTS,  -22, fwd + 10 },  // Left front turn
        { LightID::RFTS,   22, fwd + 10 },  // Right front turn
        // Front marker lamps
        { LightID::LFML,  -25, fwd + 20 },  // Left front marker
        { LightID::RFML,   25, fwd + 20 },  // Right front marker

        // Rear stop/tail lamps: inner = stop (hi), outer = tail (lo)
        { LightID::LRSL,  -10, rear     },  // Left stop (hi)
        { LightID::LRTL,  -16, rear     },  // Left tail (lo)
        { LightID::RRTL,   16, rear     },  // Right tail (lo)
        { LightID::RRSL,   10, rear     },  // Right stop (hi)
        // CHMSL
        { LightID::CHMSL,   0, rear - 8 },
        // Rear turn signals
        { LightID::LRTS,  -22, rear - 4 },  // Left rear turn
        { LightID::RRTS,   22, rear - 4 },  // Right rear turn
        // Rear side markers (outboard, on bumper sides)
        { LightID::LRSM,  -28, rear - 14 }, // Left rear side marker
        { LightID::RRSM,   28, rear - 14 }, // Right rear side marker
        // Backup lamps
        { LightID::LBL,    -4, rear + 6 },  // Left backup
        { LightID::RBL,     4, rear + 6 },  // Right backup
    };

    const int dot = 3;  // half-size of indicator square

    for (const auto& b : bulbs) {
        bool on = GetState(b.id);
        int bx = cx + b.dx;
        int by = car_top + b.dy;

        SColor color;
        SColor dim(255, 35, 35, 40);

        if (!on) {
            color = dim;
        } else {
            switch (b.id) {
                case LightID::LHBH: case LightID::RHBH:
                    color = SColor(255, 220, 220, 255);  // Cool white (hi beam)
                    break;
                case LightID::LLBH: case LightID::RLBH:
                    color = SColor(255, 255, 240, 180);  // Warm white (lo beam)
                    break;
                case LightID::LFTS: case LightID::RFTS:
                case LightID::LFML: case LightID::RFML:
                case LightID::LRTS: case LightID::RRTS:
                    color = SColor(255, 255, 180, 20);   // Amber
                    break;
                case LightID::LRSL: case LightID::RRSL:
                case LightID::CHMSL:
                    color = SColor(255, 255, 30, 10);    // Bright red
                    break;
                case LightID::LRTL: case LightID::RRTL:
                    color = SColor(255, 160, 20, 5);     // Dim red (tail)
                    break;
                case LightID::LRSM: case LightID::RRSM:
                    color = SColor(255, 180, 25, 8);     // Red (side marker)
                    break;
                case LightID::LBL: case LightID::RBL:
                    color = SColor(255, 240, 240, 240);  // White
                    break;
                default:
                    color = SColor(255, 200, 200, 200);
                    break;
            }
        }

        driver->draw2DRectangle(color,
            irr::core::recti(bx - dot, by - dot, bx + dot, by + dot));
    }

    // Labels along the edges.
    auto label = [&](int x, int y, const wchar_t* text, bool right_align = false) {
        int tx = right_align ? x - 42 : x + 5;
        font->draw(text, irr::core::recti(tx, y - 5, tx + 45, y + 7),
                   SColor(200, 140, 140, 160), right_align, false);
    };

    // Left side labels.
    label(cx - car_hw - 8, car_top + fwd,       L"HEAD",  true);
    label(cx - car_hw - 8, car_top + fwd + 10,  L"TURN",  true);
    label(cx - car_hw - 8, car_top + fwd + 20,  L"MRKR",  true);
    label(cx - car_hw - 8, car_top + rear,      L"STOP",  true);
    label(cx - car_hw - 8, car_top + rear - 4,  L"TURN",  true);
    label(cx - car_hw - 8, car_top + rear - 14, L"SIDE",  true);

    // Right side labels.
    label(cx + car_hw + 4, car_top + fwd,       L"HEAD");
    label(cx + car_hw + 4, car_top + fwd + 10,  L"TURN");
    label(cx + car_hw + 4, car_top + fwd + 20,  L"MRKR");
    label(cx + car_hw + 4, car_top + rear,      L"STOP");
    label(cx + car_hw + 4, car_top + rear - 4,  L"TURN");
    label(cx + car_hw + 4, car_top + rear - 14, L"SIDE");

    // Center labels.
    label(cx + 6, car_top + rear - 8, L"CHMSL");
    label(cx - 4, car_top + rear + 6, L"BKP");
}
