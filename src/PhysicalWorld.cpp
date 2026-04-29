#include "PhysicalWorld.h"

namespace ev1sim {

// ---------------------------------------------------------------------------
// CombinationSwitch
// ---------------------------------------------------------------------------

void CombinationSwitch::cycle_h() {
    switch (m_position) {
        case Position::OFF:  m_position = Position::PARK; break;
        case Position::PARK: m_position = Position::ON;   break;
        case Position::ON:   m_position = Position::HI;   break;
        case Position::HI:   m_position = Position::OFF;  break;
    }
}

void CombinationSwitch::set_flash_to_pass(bool held) {
    m_flash_to_pass = held;
}

bool CombinationSwitch::pin_low_beam_out() const {
    return m_position == Position::ON;
}

bool CombinationSwitch::pin_flash_to_pass_out() const {
    // At HI position, the lever is mechanically interlocked — suppress pin B.
    if (m_position == Position::HI) return false;
    return m_flash_to_pass;
}

bool CombinationSwitch::pin_park_headlamp_out() const {
    return m_position == Position::PARK ||
           m_position == Position::ON   ||
           m_position == Position::HI;
}

}  // namespace ev1sim
