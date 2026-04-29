#include "PhysicalWorld.h"

#include <irrlicht.h>
#include <cstdio>

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

// ---------------------------------------------------------------------------
// TurnSignalStalk
// ---------------------------------------------------------------------------

// Auto-cancel tuning constants.
//
// kTravelThreshold:      Steering deflection in the active direction (0..1 range)
//                        past which "you've turned" — after this the machine
//                        watches for return-to-center.  0.33 ≈ 30° of 90° max.
// kReturnThreshold:      After travel is confirmed, returning PAST this value
//                        in the opposite direction triggers the cancel.
//                        Small positive value so the wheel only needs to cross
//                        center by a tiny amount.
// kOppositeCancelThreshold: Steering in the WRONG direction past this threshold
//                        sustained for kOppositeCancelTime_s triggers wrong-side
//                        cancel (e.g. left-signal while turning right).
// kOppositeCancelTime_s: How long (seconds) the driver must hold the opposite
//                        steering past threshold before wrong-side cancel fires.
static constexpr double kTravelThreshold         = 0.33;
static constexpr double kReturnThreshold         = 0.05;
static constexpr double kOppositeCancelThreshold = 0.50;
static constexpr double kOppositeCancelTime_s    = 0.50;

void TurnSignalStalk::reset_ac_state() {
    m_ac_state           = AcState::Inactive;
    m_opp_cancel_accum_s = 0.0;
}

void TurnSignalStalk::toggle_left() {
    switch (m_position) {
        case Position::OFF:   m_position = Position::LEFT;  break;
        case Position::LEFT:  m_position = Position::OFF;   break;
        case Position::RIGHT: m_position = Position::LEFT;  break;
    }
    reset_ac_state();
    // If we just became active, enter WaitingForTravel.
    if (m_position == Position::LEFT)
        m_ac_state = AcState::WaitingForTravel;
}

void TurnSignalStalk::toggle_right() {
    switch (m_position) {
        case Position::OFF:   m_position = Position::RIGHT; break;
        case Position::RIGHT: m_position = Position::OFF;   break;
        case Position::LEFT:  m_position = Position::RIGHT; break;
    }
    reset_ac_state();
    // If we just became active, enter WaitingForTravel.
    if (m_position == Position::RIGHT)
        m_ac_state = AcState::WaitingForTravel;
}

void TurnSignalStalk::update_for_steering(double s, double dt_s) {
    // Nothing to track when stalk is OFF.
    if (m_position == Position::OFF) {
        reset_ac_state();
        return;
    }

    // Convention: positive s = steering LEFT, negative s = steering RIGHT.
    // For LEFT signal: "active direction" is positive s (left), "opposite" is negative.
    // For RIGHT signal: "active direction" is negative s (right), "opposite" is positive.
    const bool is_left   = (m_position == Position::LEFT);
    const double active  = is_left ?  s : -s;  // positive = toward active signal side
    const double opp     = is_left ? -s :  s;  // positive = toward opposite side

    switch (m_ac_state) {
        case AcState::Inactive:
            // Shouldn't happen (we set WaitingForTravel on toggle), but be safe.
            m_ac_state = AcState::WaitingForTravel;
            break;

        case AcState::WaitingForTravel:
            // Check for wrong-side cancel: driver steers hard the wrong way.
            if (opp > kOppositeCancelThreshold) {
                m_opp_cancel_accum_s += dt_s;
                if (m_opp_cancel_accum_s >= kOppositeCancelTime_s) {
                    // Wrong-side cancel.
                    m_position           = Position::OFF;
                    m_auto_cancel_event  = true;
                    reset_ac_state();
                    return;
                }
            } else {
                m_opp_cancel_accum_s = 0.0;
            }
            // Check for normal travel threshold met in the active direction.
            if (active > kTravelThreshold) {
                m_ac_state           = AcState::WaitingForReturn;
                m_opp_cancel_accum_s = 0.0;  // reset opp timer for this new phase
            }
            break;

        case AcState::WaitingForReturn:
            // Check for wrong-side cancel even in this phase.
            if (opp > kOppositeCancelThreshold) {
                m_opp_cancel_accum_s += dt_s;
                if (m_opp_cancel_accum_s >= kOppositeCancelTime_s) {
                    m_position          = Position::OFF;
                    m_auto_cancel_event = true;
                    reset_ac_state();
                    return;
                }
            } else {
                m_opp_cancel_accum_s = 0.0;
            }
            // Return-to-center cancel: active-direction steering dropped back
            // below the small return threshold (close to or past center).
            if (active < kReturnThreshold) {
                m_position          = Position::OFF;
                m_auto_cancel_event = true;
                reset_ac_state();
            }
            break;
    }
}

bool TurnSignalStalk::consume_auto_cancel_event() {
    const bool was = m_auto_cancel_event;
    m_auto_cancel_event = false;
    return was;
}

// ---------------------------------------------------------------------------
// PhysicalWorld::DrawHUD
// ---------------------------------------------------------------------------

void PhysicalWorld::DrawHUD(irr::IrrlichtDevice* device,
                             std::uint8_t rsa_run_mode,
                             bool has_rsa_run_mode) const {
    if (!device) return;
    auto* gui  = device->getGUIEnvironment();
    auto* font = gui->getBuiltInFont();
    if (!font) return;

    // Position: left-column, below the telemetry lines.
    // Telemetry occupies roughly lines 0-4 at y=10 with h=18; start at y=110.
    const int x = 10;
    int y = 110;
    const int h = 18;
    const int col_w = 400;

    using irr::video::SColor;
    using irr::core::rect;
    using irr::core::stringw;

    auto draw = [&](const char* text, SColor col) {
        stringw ws(text);
        font->draw(ws, rect<irr::s32>(x, y, x + col_w, y + h), col);
        y += h;
    };

    // --- KEY state ---
    // Top line: actual RSA run mode from bus.
    // Bottom line: locally requested state from K cycler.
    {
        const char* actual_str = "---";
        SColor actual_col(255, 160, 160, 160);  // grey = unknown
        if (has_rsa_run_mode) {
            switch (rsa_run_mode) {
                case 0: actual_str = "OFF"; actual_col = SColor(255, 160, 160, 160); break;
                case 1: actual_str = "ACC"; actual_col = SColor(255, 255, 200,  80); break;
                case 2: actual_str = "RUN"; actual_col = SColor(255,  80, 220,  80); break;
                default: actual_str = "?";  break;
            }
        }
        char buf[64];
        std::snprintf(buf, sizeof(buf), "KEY: %s", actual_str);
        draw(buf, actual_col);

        const char* req_str = m_rsa_keypad.expected_state_name();
        SColor req_col(255, 200, 200, 200);
        if      (m_rsa_keypad.expected_state() == RsaKeypadDriver::ExpectedState::RUN)
            req_col = SColor(255,  80, 220,  80);
        else if (m_rsa_keypad.expected_state() == RsaKeypadDriver::ExpectedState::ACC)
            req_col = SColor(255, 255, 200,  80);
        std::snprintf(buf, sizeof(buf), "REQ: %s", req_str);
        draw(buf, req_col);
    }

    y += 4;  // small gap between groups

    // --- Combination switch ---
    {
        const char* cs_names[] = {"OFF", "PARK", "ON", "HI"};
        int cs_idx = static_cast<int>(m_comb_sw.position());
        const char* ftp_str = m_comb_sw.flash_to_pass_held() ? "ON" : "-";
        char buf[80];
        std::snprintf(buf, sizeof(buf), "HEADLAMPS: %-4s  FTP: %s",
                      cs_names[cs_idx], ftp_str);
        SColor cs_col = (cs_idx > 0) ? SColor(255, 255, 220,  80)
                                      : SColor(255, 160, 160, 160);
        draw(buf, cs_col);
    }

    y += 4;

    // --- PRND ---
    {
        const char* prnd_names[] = {"P", "R", "N", "D"};
        int prnd_idx = static_cast<int>(m_prnd_sel.position());
        char buf[40];
        std::snprintf(buf, sizeof(buf), "GEAR: %s", prnd_names[prnd_idx]);
        SColor prnd_col = (prnd_idx == 3) ? SColor(255,  80, 220,  80)  // D = green
                        : (prnd_idx == 1) ? SColor(255, 255, 100, 100)  // R = red
                        :                   SColor(255, 220, 220, 220);  // P/N = white
        draw(buf, prnd_col);
    }

    y += 4;

    // --- Turn signals + hazard ---
    {
        using P = TurnSignalStalk::Position;
        const auto ts_pos = m_turn_stalk.position();
        SColor amber(255, 255, 180, 0);
        SColor grey(255, 160, 160, 160);

        const char* left_str  = (ts_pos == P::LEFT)  ? "<" : " ";
        const char* right_str = (ts_pos == P::RIGHT) ? ">" : " ";
        const char* hz_str    = m_hazard_sw.on() ? "ON" : "-";
        char buf[80];
        std::snprintf(buf, sizeof(buf), "TURN: %s OFF %s   HAZARD: %s",
                      left_str, right_str, hz_str);
        bool turn_active = (ts_pos != P::OFF);
        bool hz_active   = m_hazard_sw.on();
        SColor turn_col  = (turn_active || hz_active) ? amber : grey;
        draw(buf, turn_col);
    }
}

// ---------------------------------------------------------------------------
// RsaKeypadDriver
// ---------------------------------------------------------------------------

void RsaKeypadDriver::cycle_k() {
    // Advance state and arm the appropriate one-shot outputs.
    // Mode button enum values per the prompt:
    //   0=NONE, 1=OFF, 2=ACC, 3=RUN, 4=START
    switch (m_expected) {
        case ExpectedState::OFF:
            // OFF → RUN: user enters code (code_ok=1) then presses RUN.
            m_code_ok_oneshot     = true;
            m_mode_button_oneshot = 3;   // RUN
            m_expected = ExpectedState::RUN;
            break;
        case ExpectedState::RUN:
            // RUN → ACC: no code needed, just press ACC.
            m_code_ok_oneshot     = false;
            m_mode_button_oneshot = 2;   // ACC
            m_expected = ExpectedState::ACC;
            break;
        case ExpectedState::ACC:
            // ACC → OFF: press OFF.
            m_code_ok_oneshot     = false;
            m_mode_button_oneshot = 1;   // OFF
            m_expected = ExpectedState::OFF;
            break;
    }
}

const char* RsaKeypadDriver::expected_state_name() const {
    switch (m_expected) {
        case ExpectedState::OFF: return "OFF";
        case ExpectedState::RUN: return "RUN";
        case ExpectedState::ACC: return "ACC";
    }
    return "OFF";
}

void RsaKeypadDriver::clear_oneshots() {
    m_code_ok_oneshot     = false;
    m_mode_button_oneshot = 0;
}

}  // namespace ev1sim
