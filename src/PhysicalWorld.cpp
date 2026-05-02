#include "PhysicalWorld.h"

#include <irrlicht.h>
#include <cmath>
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
// BrakePedal
// ---------------------------------------------------------------------------

double BrakePedal::pressure_for_travel(double travel, const Calibration& cal) {
    if (travel < 0.0) travel = 0.0;
    if (travel > 1.0) travel = 1.0;

    double p_kpa;
    if (travel < cal.dead_band) {
        p_kpa = 0.0;
    } else if (travel < cal.transition) {
        p_kpa = cal.k1_kpa_per_unit * (travel - cal.dead_band);
    } else {
        const double soft_segment =
            cal.k1_kpa_per_unit * (cal.transition - cal.dead_band);
        p_kpa = soft_segment +
                cal.k2_kpa_per_unit * (travel - cal.transition);
    }
    if (p_kpa > cal.max_pressure_kpa) p_kpa = cal.max_pressure_kpa;
    return p_kpa;
}

double BrakePedal::update(double travel) {
    m_pressure_kpa = pressure_for_travel(travel, m_cal);
    return m_pressure_kpa;
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
// PhysicalWorld::DrawSnapshotOverlay
// ---------------------------------------------------------------------------

void PhysicalWorld::DrawSnapshotOverlay(irr::IrrlichtDevice* device,
                                         bool show,
                                         std::uint8_t rsa_run_mode,
                                         bool has_rsa_run_mode,
                                         int trip_reset_count) const {
    if (!device || !show) return;
    auto* drv  = device->getVideoDriver();
    auto* gui  = device->getGUIEnvironment();
    auto* font = gui->getBuiltInFont();
    if (!drv || !font) return;

    using irr::video::SColor;
    using irr::core::rect;
    using irr::core::stringw;

    // Panel geometry: right side, below the lights/panels HUD.
    const int panel_w = 360;
    const int panel_h = 210;
    const int margin  = 12;
    auto screen = drv->getScreenSize();
    const int px = screen.Width - panel_w - margin;
    const int py = margin;  // top of screen, right column

    // Translucent dark background.
    drv->draw2DRectangle(SColor(190, 15, 15, 25),
        irr::core::recti(px, py, px + panel_w, py + panel_h));

    const int tx = px + 6;
    int ty = py + 4;
    const int line_h = 18;
    const int col_w  = panel_w - 12;

    SColor hdr(255, 255, 220, 80);
    SColor sep(255, 100, 100, 120);
    SColor grey(255, 160, 160, 160);
    SColor grn(255,  80, 220,  80);
    SColor amber(255, 255, 180,  0);
    SColor red(255, 255, 100, 100);
    SColor white(255, 220, 220, 220);

    auto draw = [&](const char* text, SColor col) {
        stringw ws(text);
        font->draw(ws, rect<irr::s32>(tx, ty, tx + col_w, ty + line_h), col);
        ty += line_h;
    };

    // Header.
    draw("PHYSICAL WORLD                [Z to toggle]", hdr);
    draw("------------------------------------------", sep);

    // KEY: actual RSA run mode.
    {
        const char* actual_str = "---";
        SColor actual_col = grey;
        if (has_rsa_run_mode) {
            switch (rsa_run_mode) {
                case 0: actual_str = "OFF"; actual_col = grey;  break;
                case 1: actual_str = "ACC"; actual_col = amber; break;
                case 2: actual_str = "RUN"; actual_col = grn;   break;
                default: actual_str = "?";  break;
            }
        }
        char buf[64];
        std::snprintf(buf, sizeof(buf), "KEY:        %s", actual_str);
        draw(buf, actual_col);
    }

    // HEADLAMPS + flash-to-pass.
    {
        const char* cs_names[] = {"OFF", "PARK", "ON", "HI"};
        int cs_idx = static_cast<int>(m_comb_sw.position());
        const char* ftp_str = m_comb_sw.flash_to_pass_held() ? "ON" : "-";
        char buf[80];
        std::snprintf(buf, sizeof(buf), "HEADLAMPS:  %-4s FTP: %s",
                      cs_names[cs_idx], ftp_str);
        SColor cs_col = (cs_idx > 0) ? amber : grey;
        draw(buf, cs_col);
    }

    // GEAR.
    {
        const char* prnd_names[] = {"P", "R", "N", "D"};
        int prnd_idx = static_cast<int>(m_prnd_sel.position());
        char buf[40];
        std::snprintf(buf, sizeof(buf), "GEAR:       %s", prnd_names[prnd_idx]);
        SColor prnd_col = (prnd_idx == 3) ? grn
                        : (prnd_idx == 1) ? red
                        :                   white;
        draw(buf, prnd_col);
    }

    // TURN + HAZARD.
    {
        using P = TurnSignalStalk::Position;
        const auto ts_pos = m_turn_stalk.position();
        const char* turn_str =
            (ts_pos == P::LEFT)  ? "LEFT" :
            (ts_pos == P::RIGHT) ? "RIGHT" : "OFF";
        const char* hz_str = m_hazard_sw.on() ? "ON" : "-";
        char buf[80];
        std::snprintf(buf, sizeof(buf), "TURN:       %-5s    HAZARD: %s",
                      turn_str, hz_str);
        bool active = (ts_pos != P::OFF) || m_hazard_sw.on();
        draw(buf, active ? amber : grey);
    }

    // WIPER + WASH.
    {
        const char* wiper_names[] = {"OFF", "INT", "LOW", "HIGH"};
        int widx = static_cast<int>(m_wiper_stalk.position());
        char buf[80];
        std::snprintf(buf, sizeof(buf), "WIPER:      %-4s     WASH:  -",
                      wiper_names[widx]);
        SColor wiper_col = (widx > 0) ? white : grey;
        draw(buf, wiper_col);
    }

    // CRUISE SET point (show requested state).
    {
        const char* req_str = m_rsa_keypad.expected_state_name();
        // Cruise stalk has no readable "engaged" state without a controller;
        // show the RSA-requested key state as the cruise readout is in DrawHUD.
        (void)req_str;
        // Just show a static placeholder — cruise ECU drives the real state.
        draw("CRUISE:     -        SET:    -", grey);
    }

    // DOORS: lock + ajar.
    {
        auto lock_str = [](DoorLocks::State s) -> const char* {
            return (s == DoorLocks::State::LOCKED) ? "LOCKED" : "unlocked";
        };
        // "central lock" summary.
        const char* central = m_door_locks.any_locked() ? "LOCKED" : "unlocked";
        char buf[100];
        std::snprintf(buf, sizeof(buf), "DOORS:     [%s]", central);
        SColor door_col = m_door_locks.any_locked() ? amber : grey;
        draw(buf, door_col);

        // Per-door detail line.
        char buf2[100];
        std::snprintf(buf2, sizeof(buf2), "  D=%-8s P=%-8s Tk=%-8s",
                      lock_str(m_door_locks.driver()),
                      lock_str(m_door_locks.passenger()),
                      lock_str(m_door_locks.trunk()));
        draw(buf2, grey);
    }

    // TRIP RST count.
    {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "TRIP RST:   pressed %d time%s",
                      trip_reset_count,
                      trip_reset_count == 1 ? "" : "s");
        draw(buf, trip_reset_count > 0 ? white : grey);
    }
}

// ---------------------------------------------------------------------------
// RsaKeypadDriver
// ---------------------------------------------------------------------------

// Spacing between scheduled events (digit pulses and the final mode press).
static constexpr double kKeypadIntervalS = 0.100;  // 100 ms between events

// Digit-to-button mapping for EV1 community notation:
//   tap digits (lower):         1->btn0, 3->btn1, 5->btn2, 7->btn3, 9->btn4
//   long-press digits (higher): 2->btn0, 4->btn1, 6->btn2, 8->btn3, 0->btn4
static RsaKeypadDriver::DigitEntry digit_char_to_entry_(char c) {
    RsaKeypadDriver::DigitEntry e{};
    switch (c) {
        case '1': e.button_index = 0; e.long_press = false; break;
        case '2': e.button_index = 0; e.long_press = true;  break;
        case '3': e.button_index = 1; e.long_press = false; break;
        case '4': e.button_index = 1; e.long_press = true;  break;
        case '5': e.button_index = 2; e.long_press = false; break;
        case '6': e.button_index = 2; e.long_press = true;  break;
        case '7': e.button_index = 3; e.long_press = false; break;
        case '8': e.button_index = 3; e.long_press = true;  break;
        case '9': e.button_index = 4; e.long_press = false; break;
        case '0': e.button_index = 4; e.long_press = true;  break;
        default:  e.button_index = 0; e.long_press = false; break; // fallback
    }
    return e;
}

void RsaKeypadDriver::init_default_code_() {
    // Default: "111111" — six taps of button 0.
    m_code_len = kMaxCodeLen;
    for (int i = 0; i < kMaxCodeLen; ++i) {
        m_code[i].button_index = 0;
        m_code[i].long_press   = false;
    }
}

void RsaKeypadDriver::set_code_string(const char* code_str) {
    if (code_str == nullptr) { init_default_code_(); return; }
    // Validate length == 6 and all digits 0-9.
    int len = 0;
    while (code_str[len] != '\0' && len <= kMaxCodeLen) ++len;
    if (len != kMaxCodeLen) { init_default_code_(); return; }
    for (int i = 0; i < kMaxCodeLen; ++i) {
        if (code_str[i] < '0' || code_str[i] > '9') { init_default_code_(); return; }
    }
    m_code_len = kMaxCodeLen;
    for (int i = 0; i < kMaxCodeLen; ++i) {
        m_code[i] = digit_char_to_entry_(code_str[i]);
    }
}

void RsaKeypadDriver::cycle_k() {
    switch (m_expected) {
        case ExpectedState::OFF:
            // OFF → RUN: schedule code digits then a RUN press.
            // Initialise default code on first cycle if not yet set.
            if (m_code_len == 0) init_default_code_();
            m_expected    = ExpectedState::RUN;
            m_sched_state = CycleState::EmittingDigits;
            m_digits_sent = 0;
            m_timer_s     = 0.0;  // first digit fires on next update() call
            break;
        case ExpectedState::RUN:
            // RUN → ACC: immediate ACC press; no digit entry needed.
            m_expected            = ExpectedState::ACC;
            m_sched_state         = CycleState::Idle;
            m_pending.mode_button = 2;  // ACC
            break;
        case ExpectedState::ACC:
            // ACC → OFF: immediate OFF press.
            m_expected            = ExpectedState::OFF;
            m_sched_state         = CycleState::Idle;
            m_pending.mode_button = 1;  // OFF
            break;
    }
}

void RsaKeypadDriver::update(double dt_s) {
    if (m_sched_state == CycleState::Idle) return;

    m_timer_s -= dt_s;
    if (m_timer_s > 0.0) return;

    // Timer fired.
    if (m_sched_state == CycleState::EmittingDigits) {
        if (m_digits_sent < m_code_len) {
            const DigitEntry& e = m_code[m_digits_sent];
            // Encode: 1 = tap (lower digit), 2 = long-press (higher digit).
            m_pending.button_value[e.button_index] =
                static_cast<std::uint8_t>(e.long_press ? 2u : 1u);
            ++m_digits_sent;
            m_timer_s = kKeypadIntervalS;
            if (m_digits_sent >= m_code_len) {
                // Last digit sent; schedule the mode press next.
                m_sched_state = CycleState::EmittingMode;
            }
        }
    } else if (m_sched_state == CycleState::EmittingMode) {
        m_pending.mode_button = 3;  // RUN
        m_sched_state         = CycleState::Idle;
        m_timer_s             = 0.0;
    }
}

RsaKeypadDriver::KeypadFireSet RsaKeypadDriver::consume_fires_now() {
    KeypadFireSet fires = m_pending;
    m_pending = KeypadFireSet{};
    return fires;
}

const char* RsaKeypadDriver::expected_state_name() const {
    switch (m_expected) {
        case ExpectedState::OFF: return "OFF";
        case ExpectedState::RUN: return "RUN";
        case ExpectedState::ACC: return "ACC";
    }
    return "OFF";
}

// ---------------------------------------------------------------------------
// RsaExteriorKeypad
// ---------------------------------------------------------------------------

void RsaExteriorKeypad::press_button(int button_idx, bool long_press) {
    if (button_idx < 0 || button_idx >= 5) return;
    m_tap_value[button_idx] = static_cast<std::uint8_t>(long_press ? 2u : 1u);
}

std::uint8_t RsaExteriorKeypad::button_value(int idx) const {
    if (idx < 0 || idx >= 5) return 0;
    return m_tap_value[idx];
}

void RsaExteriorKeypad::clear_oneshots() {
    for (int i = 0; i < 5; ++i) m_tap_value[i] = 0;
}

// Digit-to-button+long_press mapping (same as interior keypad notation):
//   tap digits (lower):         1->btn0, 3->btn1, 5->btn2, 7->btn3, 9->btn4
//   long-press digits (higher): 2->btn0, 4->btn1, 6->btn2, 8->btn3, 0->btn4
static void ext_keypad_digit_to_entry_(char c, int& btn_out, bool& long_out) {
    switch (c) {
        case '1': btn_out = 0; long_out = false; break;
        case '2': btn_out = 0; long_out = true;  break;
        case '3': btn_out = 1; long_out = false; break;
        case '4': btn_out = 1; long_out = true;  break;
        case '5': btn_out = 2; long_out = false; break;
        case '6': btn_out = 2; long_out = true;  break;
        case '7': btn_out = 3; long_out = false; break;
        case '8': btn_out = 3; long_out = true;  break;
        case '9': btn_out = 4; long_out = false; break;
        case '0': btn_out = 4; long_out = true;  break;
        default:  btn_out = 0; long_out = false; break;
    }
}

void RsaExteriorKeypad::enter_code_sequence(const char* code_str) {
    if (!code_str) return;
    int len = 0;
    while (code_str[len] != '\0' && len <= kMaxCodeLen) ++len;
    if (len != kMaxCodeLen) return;

    m_seq_len   = kMaxCodeLen;
    m_seq_pos   = 0;
    m_seq_timer = 0.0;  // first digit fires on the very next tick
    m_seq_active = true;
    for (int i = 0; i < kMaxCodeLen; ++i) {
        int btn = 0;
        bool lp = false;
        ext_keypad_digit_to_entry_(code_str[i], btn, lp);
        m_seq[i].button_idx = static_cast<std::uint8_t>(btn);
        m_seq[i].long_press = lp;
    }
}

void RsaExteriorKeypad::update(double dt_s) {
    if (!m_seq_active) return;
    m_seq_timer -= dt_s;
    // timer <= 0 means fire immediately (including first digit at t=0)
}

bool RsaExteriorKeypad::consume_sequence_fire() {
    if (!m_seq_active) return false;
    if (m_seq_timer > 0.0) return false;

    // Fire the current digit.
    const auto& e = m_seq[m_seq_pos];
    m_tap_value[e.button_idx] = static_cast<std::uint8_t>(e.long_press ? 2u : 1u);

    ++m_seq_pos;
    if (m_seq_pos >= m_seq_len) {
        m_seq_active = false;
        m_seq_len    = 0;
        m_seq_pos    = 0;
    } else {
        // Schedule next digit after 100 ms.
        static constexpr double kSeqIntervalS = 0.100;
        m_seq_timer = kSeqIntervalS;
    }
    return true;
}

bool RsaExteriorKeypad::sequence_in_progress() const {
    return m_seq_active;
}

// ---------------------------------------------------------------------------
// AmbientTempSensor
// ---------------------------------------------------------------------------

void AmbientTempSensor::update(double time_of_day_hours) {
    // Diurnal sinusoid — peak at phase_offset_hours, trough 12 h earlier/later.
    // sin argument: 2π * (hour - peak_hour) / 24
    // At peak (hour == phase_offset_hours), sin == 0... wait, that's the zero
    // crossing.  For the peak to occur at phase_offset_hours we shift by
    // (phase_offset - 6) so sin reaches +1 at the desired hour:
    //   sin(2π*(h - (peak - 6))/24) = sin(2π*(h - peak)/24 + π/2) = cos(...)
    // Equivalently:
    //   angle = 2π * (hour - phase_offset_hours) / 24
    //   temp  = mean + amp * cos(angle)   [cos = sin + 90°]
    const double two_pi = 2.0 * 3.14159265358979323846;
    const double angle  = two_pi * (time_of_day_hours - m_cfg.phase_offset_hours) / 24.0;
    m_temp_c       = m_cfg.mean_temp_c + m_cfg.diurnal_amp_c * std::cos(angle)
                     + m_cfg.seed_offset_c;
    // Humidity inversely correlated with temperature.
    m_humidity_pct = m_cfg.mean_humidity_pct - m_cfg.diurnal_humidity_amp * std::cos(angle);
    // Clamp humidity to valid range.
    if (m_humidity_pct < 0.0)   m_humidity_pct = 0.0;
    if (m_humidity_pct > 100.0) m_humidity_pct = 100.0;
}

}  // namespace ev1sim
