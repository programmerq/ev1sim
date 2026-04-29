#pragma once

#include <cstdint>

// Forward declaration — avoid pulling Irrlicht into every translation unit
// that includes PhysicalWorld.h.
namespace irr { class IrrlichtDevice; }

namespace ev1sim {

/// The combination (turn/headlamp/cruise) stalk on the left column.
/// Models the four rotary positions plus the momentary flash-to-pass lever.
///
/// Pin truth table per the EV1 service manual (6-way blue 12084699 connector):
///
///   Position  flash_to_pass  low_beam_out (C)  flash_to_pass_out (B)  park_headlamp_out (F)
///   --------  -------------  ----------------  ---------------------  ---------------------
///   OFF       false          0                 0                      0
///   OFF       true           0                 1                      0
///   PARK      false          0                 0                      1
///   PARK      true           0                 1                      1
///   ON        false          1                 0                      1
///   ON        true           1                 1                      1
///   HI        false          0                 0                      1
///   HI        true (locked)  0                 0                      1  (interlock suppresses B)
///
/// Note: hi-beam-on is NOT a fourth output pin.  LHJB derives it internally
/// as (pos == HI) || (flash_to_pass_out == 1).  Do not add a fourth chassis
/// bus signal for it — that would be wrong per the schematic.
class CombinationSwitch {
public:
    enum class Position { OFF, PARK, ON, HI };

    /// Advance the rotary position: OFF → PARK → ON → HI → OFF.
    void cycle_h();

    /// Set flash-to-pass momentary lever state.
    /// pass true while the key is held, false on release.
    /// At HI position the lever is mechanically interlocked; the held bit is
    /// ignored and pin_flash_to_pass_out() always returns false.
    void set_flash_to_pass(bool held);

    Position position()           const { return m_position; }
    bool     flash_to_pass_held() const { return m_flash_to_pass; }

    // Three meaningful output pins (B, C, F) per the service manual.
    // Published as wire-level booleans on the chassis bus (IDs 4040–4042).
    bool pin_low_beam_out()       const;   // pin C, YEL 525B  (ID 4040)
    bool pin_flash_to_pass_out()  const;   // pin B, PPL 524B  (ID 4041)
    bool pin_park_headlamp_out()  const;   // pin F, LTBLU 74  (ID 4042)

private:
    Position m_position     = Position::OFF;
    bool     m_flash_to_pass = false;
};

/// Brake pedal switch with hysteresis.
///
/// Models the discrete brake-light switch on the EV1 brake pedal.  The switch
/// closes (pressed() == true) when brake travel reaches or exceeds the ON
/// threshold and opens (pressed() == false) when it falls below the OFF
/// threshold — classic Schmitt-trigger hysteresis to prevent chatter near the
/// switch point.  Default thresholds match the EV1 service-manual spec:
///   ON  threshold: 0.05 (5% of full travel)
///   OFF threshold: 0.03 (3% of full travel)
class BrakeSwitch {
public:
    static constexpr double kDefaultOnThreshold  = 0.05;
    static constexpr double kDefaultOffThreshold = 0.03;

    explicit BrakeSwitch(double on_threshold  = kDefaultOnThreshold,
                         double off_threshold = kDefaultOffThreshold)
        : m_on_threshold(on_threshold), m_off_threshold(off_threshold) {}

    /// Update the switch state from the current normalized brake travel (0..1).
    /// Returns the new pressed() state.
    bool update(double brake_travel);

    /// Most-recently-computed switch state.
    bool pressed() const { return m_pressed; }

private:
    double m_on_threshold;
    double m_off_threshold;
    bool   m_pressed = false;
};

/// PRND floor selector lever.
///
/// Models the physical floor-mounted PRND selector lever on the EV1.
/// The four wire-level outputs encode the current position using the Gray-code
/// truth table from propulsion manual p. 343 (also in pim_prnd.c):
///
///   Position  A      B      C      D
///   --------  -----  -----  -----  -----
///   P         false  true   true   false   (0110 = 0x6)
///   R         false  false  true   true    (0011 = 0x3)
///   N         true   false  true   false   (1010 = 0xA)
///   D         true   false  false  true    (1001 = 0x9)
///   D is an even-parity bit over A,B,C,D.
///
/// cycle_up() advances P→R→N→D; cycle_down() reverses.  Both clamp at the ends.
class PrndSelector {
public:
    enum class Position { P, R, N, D };

    /// Advance one step toward D (P→R→N→D; stays at D if already at D).
    void cycle_up();

    /// Advance one step toward P (D→N→R→P; stays at P if already at P).
    void cycle_down();

    Position position() const { return m_position; }

    /// Wire-level pin outputs per propulsion manual p. 343.
    /// Published as 1-bit booleans on the chassis bus (IDs 4050-4053).
    bool pin_a() const;
    bool pin_b() const;
    bool pin_c() const;
    bool pin_d() const;

private:
    Position m_position = Position::P;   // default: Park
};

/// Charge coupler presence sensor.
///
/// Models the J1772/Avcon paddle mating detection on the EV1 charge port.
/// Stubbed false for now — present() always returns false until a future
/// floating-UI panel or animation hook calls set_present(true).
///
/// TODO: when the charge-door animation is added, hook into this component
/// and publish kSigChargerCouplerPresent (chassis ID 4060) to signal the
/// electricsim side that the paddle is mated.
class ChargeCoupler {
public:
    bool present() const { return m_present; }
    void set_present(bool p) { m_present = p; }

private:
    bool m_present = false;  // future UI toggle
};

/// Turn-signal stalk (right column, left of steering wheel).
///
/// Three positions: OFF, LEFT, RIGHT.  Toggle semantics:
///   toggle_left():  OFF → LEFT; LEFT → OFF; RIGHT → LEFT
///   toggle_right(): OFF → RIGHT; RIGHT → OFF; LEFT → RIGHT
///
/// Auto-cancel: after the driver turns the steering wheel past a travel
/// threshold in the active direction and then returns past center, the stalk
/// automatically returns to OFF.  Also cancels if the driver steers sustained
/// in the opposite direction past a higher threshold (wrong-side cancel).
///
/// Call update_for_steering() once per tick with the normalized steering
/// value (-1..+1, positive = left) from DriverCommand.steering.
/// After the call, consume_auto_cancel_event() returns true for exactly one
/// tick if an auto-cancel just occurred.
class TurnSignalStalk {
public:
    enum class Position { OFF, LEFT, RIGHT };

    /// Cycle the stalk toward LEFT.  OFF→LEFT, LEFT→OFF, RIGHT→LEFT.
    void toggle_left();

    /// Cycle the stalk toward RIGHT.  OFF→RIGHT, RIGHT→OFF, LEFT→RIGHT.
    void toggle_right();

    /// Update the auto-cancel state machine from current steering input.
    /// steering_normalized: -1..+1 (positive = left, matching Chrono convention).
    /// dt_s: elapsed time in seconds since last call (used for timed opposite-cancel).
    /// Sets an internal flag readable via consume_auto_cancel_event() if a cancel fires.
    void update_for_steering(double steering_normalized, double dt_s);

    /// Returns true and clears the flag if an auto-cancel fired this tick.
    bool consume_auto_cancel_event();

    Position position()    const { return m_position; }
    bool active_left()     const { return m_position == Position::LEFT;  }
    bool active_right()    const { return m_position == Position::RIGHT; }

private:
    /// Internal auto-cancel state machine states (per active signal direction).
    enum class AcState {
        Inactive,         ///< Not tracking (stalk is OFF).
        WaitingForTravel, ///< Stalk active; waiting for steering past travel threshold.
        WaitingForReturn, ///< Travel seen; waiting for steering to return past center.
    };

    /// Reset auto-cancel state machine (called on any manual toggle).
    void reset_ac_state();

    Position m_position = Position::OFF;

    AcState  m_ac_state           = AcState::Inactive;
    double   m_opp_cancel_accum_s = 0.0;  ///< Accumulated time steering past opp threshold.
    bool     m_auto_cancel_event  = false; ///< Cleared by consume_auto_cancel_event().
};

/// Hazard warning switch (dashboard pushbutton).
///
/// Simple latching toggle: on/off.  When on, ev1sim publishes
/// kSigDriverHazardRequest (ID 6944) as true, which LHJB uses to override
/// both turn-signal sides so all four corners blink in unison.
class HazardSwitch {
public:
    void toggle() { m_on = !m_on; }
    bool on() const { return m_on; }

private:
    bool m_on = false;
};

/// RSA exterior keypad + mode-button simulator.
///
/// The EV1 RSA (Remote Security Access) module controls the vehicle's run
/// mode.  To start the vehicle the user must:
///   1. Enter the correct 5-digit code on the door-pillar keypad
///      (ev1sim simulates this via kSigDriverRsaKeypadCodeOk = 6970)
///   2. Press the RUN button on the RSA HMI
///      (ev1sim simulates this via kSigDriverRsaModeButton = 6971)
///
/// The K key in ev1sim cycles a local "expected key state" through:
///   OFF → RUN  (publishes code_ok=1 for one tick, then mode_button=RUN)
///   RUN → ACC  (publishes mode_button=ACC; no code_ok needed)
///   ACC → OFF  (publishes mode_button=OFF)
///   OFF → RUN  (wraps back to start)
///
/// Both outputs are momentary one-shots lasting exactly one tick; the next
/// tick they return to zero.  RSA latches the mode internally.
///
/// TODO: per-digit keypad fidelity (digit-by-digit entry) is deferred —
///       ev1sim currently sends the aggregate "code_ok" shortcut signal.
class RsaKeypadDriver {
public:
    enum class ExpectedState { OFF, RUN, ACC };

    /// Advance the cycle: OFF → RUN → ACC → OFF → …
    void cycle_k();

    /// Current locally-expected RSA state (for HUD display).
    ExpectedState expected_state() const { return m_expected; }

    /// Name string for HUD/logging ("OFF", "RUN", "ACC").
    const char* expected_state_name() const;

    /// One-shot momentary output for this tick.
    /// Returns true for exactly one tick after the OFF→RUN transition.
    bool code_ok_press_now() const { return m_code_ok_oneshot; }

    /// One-shot momentary output for this tick.
    /// Returns 0=NONE, 2=ACC, 3=RUN, 1=OFF per the enum in the prompt.
    uint8_t mode_button_press_now() const { return m_mode_button_oneshot; }

    /// Clear the one-shot outputs.  Call once per tick after consuming.
    void clear_oneshots();

private:
    ExpectedState m_expected        = ExpectedState::OFF;
    bool          m_code_ok_oneshot = false;
    uint8_t       m_mode_button_oneshot = 0;
};

/// Container for all physical-world input components.
/// Future additions (wiper stalk, etc.) join here.
class PhysicalWorld {
public:
    CombinationSwitch&       combination_switch()       { return m_comb_sw; }
    const CombinationSwitch& combination_switch() const { return m_comb_sw; }

    BrakeSwitch&       brake_switch()       { return m_brake_sw; }
    const BrakeSwitch& brake_switch() const { return m_brake_sw; }

    ChargeCoupler&       charge_coupler()       { return m_charge_coupler; }
    const ChargeCoupler& charge_coupler() const { return m_charge_coupler; }

    PrndSelector&       prnd_selector()       { return m_prnd_sel; }
    const PrndSelector& prnd_selector() const { return m_prnd_sel; }

    TurnSignalStalk&       turn_signal_stalk()       { return m_turn_stalk; }
    const TurnSignalStalk& turn_signal_stalk() const { return m_turn_stalk; }

    HazardSwitch&       hazard_switch()       { return m_hazard_sw; }
    const HazardSwitch& hazard_switch() const { return m_hazard_sw; }

    RsaKeypadDriver&       rsa_keypad()       { return m_rsa_keypad; }
    const RsaKeypadDriver& rsa_keypad() const { return m_rsa_keypad; }

    /// Draw HUD overlays for: key state, combination switch, PRND selector,
    /// and turn signals/hazard.  Call between BeginScene and EndScene.
    /// rsa_run_mode: most recently received RSA run mode (0=OFF,1=ACC,2=RUN;
    ///               pass 0xFF if not yet received).
    void DrawHUD(irr::IrrlichtDevice* device,
                 std::uint8_t rsa_run_mode, bool has_rsa_run_mode) const;

private:
    CombinationSwitch m_comb_sw;
    BrakeSwitch       m_brake_sw;
    ChargeCoupler     m_charge_coupler;
    PrndSelector      m_prnd_sel;
    TurnSignalStalk   m_turn_stalk;
    HazardSwitch      m_hazard_sw;
    RsaKeypadDriver   m_rsa_keypad;
};

}  // namespace ev1sim
