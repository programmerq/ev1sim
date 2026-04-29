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

/// Container for all physical-world input components.
/// Future additions (hazard switch, wiper stalk, RSA keypad, etc.) join here.
class PhysicalWorld {
public:
    CombinationSwitch&       combination_switch()       { return m_comb_sw; }
    const CombinationSwitch& combination_switch() const { return m_comb_sw; }

private:
    CombinationSwitch m_comb_sw;
};

}  // namespace ev1sim
