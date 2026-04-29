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

// ---------------------------------------------------------------------------
// BrakeSwitch
// ---------------------------------------------------------------------------

bool BrakeSwitch::update(double brake_travel) {
    if (!m_pressed && brake_travel >= m_on_threshold) {
        m_pressed = true;
    } else if (m_pressed && brake_travel < m_off_threshold) {
        m_pressed = false;
    }
    return m_pressed;
}

// ---------------------------------------------------------------------------
// PrndSelector
// ---------------------------------------------------------------------------

void PrndSelector::cycle_up() {
    switch (m_position) {
        case Position::P: m_position = Position::R; break;
        case Position::R: m_position = Position::N; break;
        case Position::N: m_position = Position::D; break;
        case Position::D: break;  // clamp at D
    }
}

void PrndSelector::cycle_down() {
    switch (m_position) {
        case Position::P: break;  // clamp at P
        case Position::R: m_position = Position::P; break;
        case Position::N: m_position = Position::R; break;
        case Position::D: m_position = Position::N; break;
    }
}

// Truth table from propulsion manual p. 343 (Gray-coded with even parity):
//   P: A=0 B=1 C=1 D=0   (0110)
//   R: A=0 B=0 C=1 D=1   (0011)
//   N: A=1 B=0 C=1 D=0   (1010)
//   D: A=1 B=0 C=0 D=1   (1001)

bool PrndSelector::pin_a() const {
    return m_position == Position::N || m_position == Position::D;
}

bool PrndSelector::pin_b() const {
    return m_position == Position::P;
}

bool PrndSelector::pin_c() const {
    return m_position == Position::P || m_position == Position::R ||
           m_position == Position::N;
}

bool PrndSelector::pin_d() const {
    return m_position == Position::R || m_position == Position::D;
}

}  // namespace ev1sim
