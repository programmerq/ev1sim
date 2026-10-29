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
    return m_position == Position::ON || m_position == Position::HI;
}

bool CombinationSwitch::pin_flash_to_pass_out() const {
    return m_position == Position::HI || m_flash_to_pass;
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
    // Default: "11111" — five taps of button 0.
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

void RsaKeypadDriver::enqueue_mode_(std::uint8_t mode_switches) {
    if (m_mode_queue_len >= kMaxModeQueue) return;  // overflow guard (never hit)
    m_mode_queue[m_mode_queue_len++] = mode_switches;
    // If the scheduler is idle, arm it to drain the queue on the next update().
    if (m_sched_state == CycleState::Idle) {
        m_sched_state = CycleState::EmittingMode;
        m_timer_s     = 0.0;
    }
}

void RsaKeypadDriver::cycle_k() {
    // Faithful detent order: OFF → ACC → RUN → OFF.  See the class comment and
    // the external sim's boot-keying-race supplement note.
    switch (m_expected) {
        case ExpectedState::OFF:
            // OFF → ACC: schedule the code digits (open the RSA auth window),
            // then an ACC mode press.  Initialise the default code on the
            // first cycle if not yet set.
            if (m_code_len == 0) init_default_code_();
            m_expected    = ExpectedState::ACC;
            m_sched_state = CycleState::EmittingDigits;
            m_digits_sent = 0;
            m_timer_s     = 0.0;  // first digit fires on next update() call
            // The ACC press is enqueued; the EmittingDigits→EmittingMode
            // hand-off (in update()) drains it once the code is fully sent.
            enqueue_mode_(kModeSwOffAcc);  // OFF/ACC key -> ACC
            break;
        case ExpectedState::ACC:
            // ACC → RUN: a RUN press.  Queued behind any code digits still in
            // flight (and behind the pending ACC press) so RUN is never sent
            // before authentication completes — this is what makes the co-sim
            // "key_on_cycle x2" boot recipe deterministically reach RUN even
            // when the two cycles are only ~300 ms apart.
            m_expected = ExpectedState::RUN;
            enqueue_mode_(kModeSwRun);
            break;
        case ExpectedState::RUN:
            // RUN → OFF: an OFF press (always allowed; turns the car off).
            m_expected = ExpectedState::OFF;
            enqueue_mode_(kModeSwLock);    // LOCK key -> OFF
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
                // Last digit sent; drain the queued mode press(es) next.
                m_sched_state = CycleState::EmittingMode;
            }
        }
    } else if (m_sched_state == CycleState::EmittingMode) {
        if (m_mode_queue_len > 0) {
            // Emit the head of the FIFO this tick; shift the rest down.
            m_pending.mode_switches = m_mode_queue[0];
            for (int i = 1; i < m_mode_queue_len; ++i) {
                m_mode_queue[i - 1] = m_mode_queue[i];
            }
            --m_mode_queue_len;
        }
        if (m_mode_queue_len > 0) {
            // More presses queued — fire the next one a tick later so the RSA
            // sees them on distinct ticks.
            m_timer_s = kKeypadIntervalS;
        } else {
            m_sched_state = CycleState::Idle;
            m_timer_s     = 0.0;
        }
    }
}

RsaKeypadDriver::KeypadFireSet RsaKeypadDriver::consume_fires_now() {
    KeypadFireSet fires = m_pending;
    m_pending = KeypadFireSet{};
    return fires;
}

void RsaKeypadDriver::press_mode_button(std::uint8_t mode_switch) {
    switch (mode_switch) {
        case kModeSwLock:
            m_expected = ExpectedState::OFF;
            break;
        case kModeSwOffAcc:
            // A tap from RUN does nothing (the RSA needs a one-second hold).
            if (m_expected == ExpectedState::OFF) m_expected = ExpectedState::ACC;
            break;
        case kModeSwRun:
            m_expected = ExpectedState::RUN;
            break;
        default:
            return;  // not a single console mode button
    }
    enqueue_mode_(mode_switch);
}

void RsaKeypadDriver::press_digit(int button_idx, bool long_press) {
    if (button_idx < 0 || button_idx >= 5) return;
    m_pending.button_value[button_idx] = static_cast<std::uint8_t>(long_press ? 2u : 1u);
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
    if (button_idx < 0 || button_idx >= 5 || m_count >= kQueueLen) return;
    m_queue[(m_head + m_count) % kQueueLen] = {static_cast<std::uint8_t>(button_idx), long_press};
    ++m_count;
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
    constexpr int kCodeLen = 5;
    int len = 0;
    while (code_str[len] != '\0' && len <= kCodeLen) ++len;
    if (len != kCodeLen) return;
    for (int i = 0; i < kCodeLen; ++i) {
        int btn = 0;
        bool lp = false;
        ext_keypad_digit_to_entry_(code_str[i], btn, lp);
        press_button(btn, lp);
    }
}

void RsaExteriorKeypad::update(double dt_s) {
    constexpr double kTimeEpsS = 1e-9;   // sim time is a sum of float steps
    if (m_phase != Phase::Idle) m_timer -= dt_s;
    if (m_phase == Phase::Held && m_timer <= kTimeEpsS) {
        m_phase = Phase::Gap;
        m_timer = kGapS;
    }
    if (m_phase == Phase::Gap && m_timer <= kTimeEpsS) {
        m_phase  = Phase::Idle;
        m_button = -1;
    }
    if (m_phase == Phase::Idle && m_count > 0) {
        const Press p = m_queue[m_head];
        m_head = (m_head + 1) % kQueueLen;
        --m_count;
        m_phase  = Phase::Held;
        m_button = p.button_idx;
        m_timer  = p.long_press ? kLongHoldS : kTapHoldS;
    }
}

// ---------------------------------------------------------------------------
// AmbientTempSensor
// ---------------------------------------------------------------------------

double AmbientTempSensor::hour_of_day(double start_hour_h, double sim_time_s) {
    const double h = start_hour_h + sim_time_s / 3600.0;
    return h - 24.0 * std::floor(h / 24.0);
}

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


// ---------------------------------------------------------------------------
// PowerSteeringPumpMotor — per-phase BLDC on the PSCM inverter.
// ---------------------------------------------------------------------------

namespace {
constexpr double kTwoPi = 6.283185307179586;
constexpr double kSqrt3Over2 = 0.8660254037844386;

double Clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Inverse Park + Clarke (amplitude-invariant): rotor-frame d/q onto phase A/B/C.
double DqToPhase(double d, double q, double theta, int phase) {
    const double alpha = d * std::cos(theta) - q * std::sin(theta);
    const double beta  = d * std::sin(theta) + q * std::cos(theta);
    switch (phase) {
        case 0:  return alpha;
        case 1:  return -0.5 * alpha + kSqrt3Over2 * beta;
        default: return -0.5 * alpha - kSqrt3Over2 * beta;
    }
}
}  // namespace

double PowerSteeringPumpMotor::hydraulic_load_from_steering(double steering) const {
    const double onset = m_cfg.stop_load_onset;
    if (onset >= 1.0) return std::fabs(steering) >= 1.0 ? 1.0 : 0.0;
    return Clamp((std::fabs(steering) - onset) / (1.0 - onset), 0.0, 1.0);
}

double PowerSteeringPumpMotor::pump_torque_nm(double omega_m) const {
    // The pressure torque fades in over the first ~10 rad/s so a stopped
    // rotor is not held by a torque it has no speed to react against.
    const double pressure = m_cfg.rest_load_torque_nm +
        m_load * (m_cfg.stop_load_torque_nm + m_cfg.stop_load_nms * omega_m);
    return pressure * std::tanh(omega_m / 10.0);
}

double PowerSteeringPumpMotor::electrical_hz() const {
    return m_cfg.pole_pairs * m_omega_m / kTwoPi;
}
double PowerSteeringPumpMotor::actual_speed() const {
    if (m_cfg.max_output_hz <= 0.0) return 0.0;
    return Clamp(electrical_hz() / m_cfg.max_output_hz, 0.0, 1.0);
}
double PowerSteeringPumpMotor::shaft_rpm() const { return m_omega_m * 60.0 / kTwoPi; }
double PowerSteeringPumpMotor::phase_current_a(int phase) const {
    return DqToPhase(m_id, m_iq, m_theta_e, phase);
}
double PowerSteeringPumpMotor::phase_voltage_v(int phase) const {
    return DqToPhase(m_vd, m_vq, m_theta_e, phase);
}
double PowerSteeringPumpMotor::torque_nm() const {
    return 1.5 * m_cfg.pole_pairs * m_cfg.flux_linkage_wb * m_iq;
}
double PowerSteeringPumpMotor::load_torque_nm() const {
    return pump_torque_nm(m_omega_m) + m_cfg.friction_nms * m_omega_m;
}
double PowerSteeringPumpMotor::input_power_w() const {
    return 1.5 * (m_vd * m_id + m_vq * m_iq);
}
double PowerSteeringPumpMotor::copper_loss_w() const {
    return 1.5 * m_cfg.phase_resistance_ohm * (m_id * m_id + m_iq * m_iq);
}
std::uint16_t PowerSteeringPumpMotor::case_temp_q8(double c) {
    if (!(c > 0.0)) return 0;
    const double q8 = std::round(c * 256.0);
    return q8 >= 65535.0 ? 65535 : static_cast<std::uint16_t>(q8);
}
double PowerSteeringPumpMotor::pump_heat_w() const {
    return m_cfg.pump_heat_fraction * pump_torque_nm(m_omega_m) * m_omega_m;
}
double PowerSteeringPumpMotor::friction_loss_w() const {
    return m_cfg.friction_nms * m_omega_m * m_omega_m;
}

void PowerSteeringPumpMotor::step_electrical(double h, double f_target_hz) {
    const Config& c = m_cfg;
    const int    p   = c.pole_pairs;
    const double R   = c.phase_resistance_ohm;
    const double L   = c.phase_inductance_h;
    const double lam = c.flux_linkage_wb;

    // Inverter frequency reference ramps toward the command.
    const double max_step = c.ramp_hz_per_s * h;
    m_ref_hz += Clamp(f_target_hz - m_ref_hz, -max_step, max_step);

    const bool driving = m_present && (m_ref_hz > 0.0 || f_target_hz > 0.0);
    double omega_e = p * m_omega_m;

    if (!driving) {
        // Phases open: no current, no torque; the rotor coasts.
        m_id = m_iq = m_vd = m_vq = 0.0;
        m_int_speed = m_int_d = m_int_q = 0.0;
        m_ref_hz = m_present ? m_ref_hz : 0.0;
    } else {
        // Speed loop → q-axis current reference (d-axis held at zero).
        const double kt  = 1.5 * p * lam;
        const double wb  = c.speed_loop_bw_rad_s;
        const double kp_w = c.rotor_inertia_kgm2 * wb / kt;
        const double ki_w = kp_w * wb / 5.0;
        const double w_ref = kTwoPi * m_ref_hz / p;
        const double e_w = w_ref - m_omega_m;
        double iq_ref = kp_w * e_w + m_int_speed;
        const double iq_sat = Clamp(iq_ref, -c.current_limit_a, c.current_limit_a);
        if (iq_sat == iq_ref || (iq_ref > iq_sat) != (e_w > 0.0)) m_int_speed += ki_w * e_w * h;
        iq_ref = iq_sat;

        // Current loops with back-EMF and cross-coupling feed-forward.
        const double kp_i = L * c.current_loop_bw_rad_s;
        const double ki_i = R * c.current_loop_bw_rad_s;
        const double e_d = 0.0 - m_id;
        const double e_q = iq_ref - m_iq;
        double vd = kp_i * e_d + m_int_d - omega_e * L * m_iq;
        double vq = kp_i * e_q + m_int_q + omega_e * (L * m_id + lam);
        const double vmag = std::sqrt(vd * vd + vq * vq);
        const bool limited = vmag > c.max_phase_voltage_v;
        if (limited) {
            const double k = c.max_phase_voltage_v / vmag;
            vd *= k;
            vq *= k;
        } else {
            m_int_d += ki_i * e_d * h;
            m_int_q += ki_i * e_q * h;
        }
        m_vd = vd;
        m_vq = vq;

        // Motor electrical state in the rotor frame.
        const double did = (m_vd - R * m_id + omega_e * L * m_iq) / L;
        const double diq = (m_vq - R * m_iq - omega_e * (L * m_id + lam)) / L;
        m_id += did * h;
        m_iq += diq * h;
    }

    // Shaft.
    const double t_net = torque_nm() - load_torque_nm();
    m_omega_m += t_net / c.rotor_inertia_kgm2 * h;
    if (m_omega_m < 0.0 && !driving) m_omega_m = 0.0;   // the pump does not run backwards
    omega_e = p * m_omega_m;
    m_theta_e = std::fmod(m_theta_e + omega_e * h, kTwoPi);
    if (m_theta_e < 0.0) m_theta_e += kTwoPi;

    // Case: copper, friction and pump-churn losses in, conduction to ambient out.
    const double loss = copper_loss_w() + friction_loss_w() + pump_heat_w();
    m_case_c += (loss - (m_case_c - m_ambient_c) / c.case_to_ambient_k_per_w) /
                c.case_heat_capacity_j_per_k * h;
}

void PowerSteeringPumpMotor::update(double dt, double commanded_speed) {
    m_cmd = Clamp(commanded_speed, 0.0, 1.0);
    if (dt <= 0.0) return;
    const double f_target = m_present ? m_cmd * m_cfg.max_output_hz : 0.0;
    const double sub = m_cfg.substep_s > 0.0 ? m_cfg.substep_s : dt;
    const int n = static_cast<int>(std::ceil(dt / sub));
    const double h = dt / n;
    for (int i = 0; i < n; ++i) step_electrical(h, f_target);
}

}  // namespace ev1sim
