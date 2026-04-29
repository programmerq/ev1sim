#pragma once

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

/// Container for all physical-world input components.
/// Future additions (hazard switch, wiper stalk, RSA keypad, etc.) join here.
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

private:
    CombinationSwitch m_comb_sw;
    BrakeSwitch       m_brake_sw;
    ChargeCoupler     m_charge_coupler;
    PrndSelector      m_prnd_sel;
};

}  // namespace ev1sim
