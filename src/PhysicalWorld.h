#pragma once

#include <cmath>
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

/// Brake pedal master-cylinder pressure model.
///
/// Converts pedal travel (normalized 0..1) into a master-cylinder pressure
/// in kPa using a two-stage linear curve that approximates the feel of a
/// physical brake pedal with a two-stage hardware spring:
///
///   travel < dead_band                -> 0 kPa             (free play, no fluid takeup)
///   travel < transition               -> k1 × (t - dead_band)
///   travel ≥ transition               -> k1×(transition - dead_band) + k2×(t - transition)
///
/// The simulator-side model is intentionally simple: no fluid compressibility,
/// no caliper-vs-master volume tracking, no temperature effects.  The two-
/// segment shape captures the dominant feel: an initially soft segment as
/// the master cylinder cup engages, then a much firmer segment once the
/// fluid has nowhere to go.
///
/// Default values are educated guesses tuned to land at ~12 MPa near full
/// travel — typical for a passenger-car master cylinder under hard braking.
/// Real EV1 manual data would refine these; tune to match the user's sim-rig
/// pedal feel until then.
///
/// Published on the chassis bus as kSigChassisBrakeMasterPressureKpa (4074)
/// every tick on change.  BTCM consumes it as the primary brake-effort
/// input; the existing kSigDriverBrakeSwitch (6904) keeps its role as the
/// ABS pump-prime threshold.
class BrakePedal {
public:
    struct Calibration {
        double dead_band  = 0.07;   ///< travel below this → 0 pressure
        double transition = 0.35;   ///< travel above this → use k2 slope
        double k1_kpa_per_unit = 8000.0;   ///< soft initial takeup stage
        double k2_kpa_per_unit = 18000.0;  ///< firm fluid-pressure stage
        double max_pressure_kpa = 15000.0; ///< cap (saturation)
    };

    BrakePedal() = default;
    explicit BrakePedal(const Calibration& cal) : m_cal(cal) {}

    /// Update from normalized pedal travel (0..1).  Stores the most recent
    /// pressure on the component for accessor + bus-publish use.
    /// Returns the computed pressure in kPa.
    double update(double travel);

    /// Most-recently-computed master cylinder pressure (kPa).
    double pressure_kpa() const { return m_pressure_kpa; }

    /// Stateless conversion — exposed for unit tests and other callers
    /// that want to compute a pressure without owning a BrakePedal.
    static double pressure_for_travel(double travel, const Calibration& cal);

    const Calibration& calibration() const { return m_cal; }
    void set_calibration(const Calibration& cal) { m_cal = cal; }

private:
    Calibration m_cal;
    double m_pressure_kpa = 0.0;
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

    // Combination-switch (connector 12092237) output cavities, published
    // wire-level on the chassis bus.  Pure reflection of stalk position — as
    // with CombinationSwitch's pins, run1/power gating is the consuming
    // module's job (LHJB), not ours.
    //   pin C, LT BLU/WHT 1414 "LEFT-TURN OUT"   (chassis ID 4044)
    //   pin B, DK BLU/WHT 1415 "RIGHT-TURN OUT"  (chassis ID 4043)
    bool pin_left_turn_out()  const { return active_left();  }
    bool pin_right_turn_out() const { return active_right(); }

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

    // Combination-switch (12092237) cavity D is the 640H "HAZARD-SWITCH B+"
    // supply *into* the switch; what crosses to the modules is the derived
    // hazard-active state (chassis ID 4045), not the raw B+ probe — mirroring
    // how CombinationSwitch publishes derived pin truth, never a supply pin.
    bool pin_hazard_out() const { return m_on; }

private:
    bool m_on = false;
};

/// Driver horn button (the horn pad on the wheel / steering column).
///
/// A single contact.  On the real EV1 it is circuit 28 "HORN OUT" (BLK),
/// cavity H of the turn/hazard combination switch (connector 12092237),
/// landing at LHJB J2-D16 "HORN COMMAND".  Held while pressed.  LHJB energizes
/// BOTH the 400 Hz (low) and 500 Hz (high) sounders together — the hi/lo split
/// lives only on the OUTPUT side (relay drives 4020/4021), it is not a driver
/// choice.  (DriverCommand.horn_low/horn_high remain a keyboard-only debug aid.)
class HornButton {
public:
    void set_held(bool held) { m_held = held; }
    bool held() const { return m_held; }

    // Cavity H, BLK 28 "HORN OUT" — single horn command wire (chassis ID 4046).
    bool pin_horn_out() const { return m_held; }

private:
    bool m_held = false;
};

/// RSA interior keypad + mode-button simulator.
///
/// The EV1 RSA (Remote Security Access) module controls the vehicle's run
/// mode.  To start the vehicle the user must:
///   1. Enter the correct 6-digit interior code on the RSA keypad by
///      pressing the per-digit button signals (6975-6979).
///   2. Press the RUN button on the RSA HMI (6971).
///
/// The K key in ev1sim cycles a local "expected key state" through:
///   OFF → RUN  (schedules the configured code digits spaced 100 ms apart, then RUN)
///   RUN → ACC  (immediate ACC mode-button pulse; no digit re-entry needed)
///   ACC → OFF  (immediate OFF mode-button pulse)
///   OFF → RUN  (wraps back to start)
///
/// Long-press encoding (Option A):
///   Each button signal (6975-6979) carries a uint8 payload:
///     0 = idle (no press)
///     1 = tap  (lower digit: 1, 3, 5, 7, 9)
///     2 = long-press (higher digit: 2, 4, 6, 8, 0)
///
/// The digit sequence runs through a tick-driven scheduler.  Call update()
/// once per render frame; consume_fires_now() to get the resulting signals.
///
/// TODO: hold-time threshold for distinguishing tap vs long in real UI (≥500 ms).
class RsaKeypadDriver {
public:
    enum class ExpectedState { OFF, RUN, ACC };

    /// Aggregate of signals to fire on this tick.
    struct KeypadFireSet {
        /// Per-button value (index 0..4, signals 6975-6979).
        /// 0 = idle, 1 = tap (lower digit), 2 = long-press (higher digit).
        std::uint8_t button_value[5] = {};
        std::uint8_t mode_button = 0; ///< 0=none, 1=OFF, 2=ACC, 3=RUN
    };

    /// Set the code that will be entered on the next OFF → RUN transition.
    /// code_str must be exactly 6 digits (0-9) in EV1 community notation:
    ///   tap digits (lower):        1, 3, 5, 7, 9
    ///   long-press digits (higher): 2, 4, 6, 8, 0
    /// Default if not called: "111111" (six taps of button 1).
    /// Call before cycle_k() to take effect on the next K press.
    void set_code_string(const char* code_str);

    /// Advance the cycle: OFF → RUN → ACC → OFF → …
    void cycle_k();

    /// Tick the internal scheduler by dt_s seconds.  Should be called once
    /// per render/headless loop frame before consume_fires_now().
    void update(double dt_s);

    /// Consume the fires for the current tick (returns and clears in one go).
    KeypadFireSet consume_fires_now();

    /// Current locally-expected RSA state (for HUD display).
    ExpectedState expected_state() const { return m_expected; }

    /// Name string for HUD/logging ("OFF", "RUN", "ACC").
    const char* expected_state_name() const;

    /// Per-digit entry: button index (0..4) + whether it's a long-press.
    struct DigitEntry {
        std::uint8_t button_index = 0;
        bool         long_press   = false;
    };

private:
    enum class CycleState { Idle, EmittingDigits, EmittingMode };

    static constexpr int kMaxCodeLen = 6;

    ExpectedState m_expected     = ExpectedState::OFF;
    CycleState    m_sched_state  = CycleState::Idle;
    int           m_digits_sent  = 0;    ///< how many digit pulses have fired
    double        m_timer_s      = 0.0;  ///< seconds until next event fires

    /// Sequence of digits to emit on next OFF→RUN cycle.
    DigitEntry    m_code[kMaxCodeLen];
    int           m_code_len     = kMaxCodeLen;  ///< always 6

    KeypadFireSet m_pending{};           ///< accumulated this tick; cleared by consume

    /// Initialise m_code to default "111111" (six button-0 taps).
    void init_default_code_();
};

/// IPC cluster trip-reset button (momentary dash button).
///
/// Single-shot press: press() marks a pending event; consume_press_event()
/// returns true once and clears the flag.  SimApp drives the consume path
/// each frame and publishes kSigDriverIpcTripResetButton (6952) when true.
class IpcTripResetButton {
public:
    /// Signal a button press (user tapped the I key in ev1sim).
    void press() { m_pending = true; }

    /// Consume the pending press event — returns true once then clears.
    bool consume_press_event() {
        bool v = m_pending;
        m_pending = false;
        return v;
    }

private:
    bool m_pending = false;
};

/// Cruise-control stalk — faithful three-contact model of the raw chassis
/// cavities kSigCruiseSw_{SetCoast,ResumeAccel,OnOff}Out (4047/4048/4049,
/// circuits 84/87/397 at PIM J1).
///
/// The real stalk presents three raw contacts to PIM; PIM's tap/hold decoder
/// (pim_cruise_input) does ALL interpretation: a brief SET/COAST close → SET,
/// a sustained close → SPEED_DOWN (likewise RESUME/ACCEL → RESUME / SPEED_UP),
/// and the ON/OFF contact is the master arm (falling edge → CANCEL; while open,
/// SET/RESUME sit electrically dead).  ev1sim must NOT pre-decode tap-vs-hold —
/// it only opens and closes the contacts; the held duration alone decides.
///
/// ev1sim drives the contacts faithfully:
///   * keyboard: SET/COAST closed while G or '-' is held; RESUME/ACCEL while
///     Y or '+' is held (held duration alone selects tap vs. hold downstream).
///   * UI panel: a momentary SET/RES press → a brief synthesized close that
///     decodes as a tap; SPEED+/- press → a sustained close that decodes as a
///     hold (one speed step per click).
///   * ON/OFF: a master latch — there is no dedicated key, so any SET/RESUME
///     activity auto-arms it and CANCEL (N key / UI CANCEL) opens it.  This
///     mirrors a real driver flipping ON before SET and CANCEL to disarm.
class CruiseStalk {
public:
    /// Per-frame evolution.  Keyboard held-contact state is passed in directly
    /// (level, not edge); UI presses and the cancel request (below) fold in.
    /// Contact closures are sampled BEFORE the synthesized-close timers decay
    /// so a single-frame click is always observed as ≥1 closed frame.
    void update(double dt, bool kbd_set_coast_held, bool kbd_resume_accel_held) {
        m_set_coast_closed =
            kbd_set_coast_held || (m_ui_set_coast_s > 0.0);
        m_resume_accel_closed =
            kbd_resume_accel_held || (m_ui_resume_accel_s > 0.0);

        // ON/OFF master latch: any SET/RESUME activity arms it; CANCEL opens it.
        if (m_set_coast_closed || m_resume_accel_closed) m_on_off_closed = true;
        if (m_cancel_pending) { m_on_off_closed = false; m_cancel_pending = false; }

        // Decay the UI synthesized-close timers (after sampling, above).
        if (m_ui_set_coast_s > 0.0) {
            m_ui_set_coast_s -= dt;
            if (m_ui_set_coast_s < 0.0) m_ui_set_coast_s = 0.0;
        }
        if (m_ui_resume_accel_s > 0.0) {
            m_ui_resume_accel_s -= dt;
            if (m_ui_resume_accel_s < 0.0) m_ui_resume_accel_s = 0.0;
        }
    }

    // Floating-UI panel actions (momentary).  SET/RES → brief tap close;
    // SPEED+/- → sustained hold close; CANCEL → open the master latch.
    void press_set()        { m_ui_set_coast_s    = kTapCloseSeconds; }
    void press_resume()     { m_ui_resume_accel_s = kTapCloseSeconds; }
    void press_speed_down() { m_ui_set_coast_s    = kHoldCloseSeconds; }
    void press_speed_up()   { m_ui_resume_accel_s = kHoldCloseSeconds; }
    void press_cancel()     { m_cancel_pending = true; }

    // Faithful contact outputs — what PIM's three wires see this frame.
    bool set_coast_contact()    const { return m_set_coast_closed; }
    bool resume_accel_contact() const { return m_resume_accel_closed; }
    bool on_off_contact()       const { return m_on_off_closed; }

private:
    // A tap close must read below PIM_CRUISE_INPUT_TAP_HOLD_MS (500 ms); a hold
    // close must exceed it (and persist long enough for one ~100 ms repeat).
    static constexpr double kTapCloseSeconds  = 0.05;  //  50 ms → tap
    static constexpr double kHoldCloseSeconds = 0.60;  // 600 ms → hold (1 step)

    double m_ui_set_coast_s      = 0.0;
    double m_ui_resume_accel_s   = 0.0;
    bool   m_cancel_pending      = false;
    bool   m_set_coast_closed    = false;
    bool   m_resume_accel_closed = false;
    bool   m_on_off_closed       = false;
};

/// Wiper stalk (right column, four rotary positions + momentary wash button).
///
/// cycle_position() advances: OFF → INT → LOW → HIGH → OFF (wrapping).
/// press_wash() marks a momentary wash event; consume_wash() reads and clears.
/// position() reflects the current stalk setting (enum matches the wire encoding
/// for kSigDriverWiperSwitch: OFF=0, INT=1, LOW=2, HIGH=3).
class WiperStalk {
public:
    enum class Position { OFF = 0, INT = 1, LOW = 2, HIGH = 3 };

    /// Advance the rotary position: OFF → INT → LOW → HIGH → OFF.
    void cycle_position() {
        switch (m_position) {
            case Position::OFF:  m_position = Position::INT;  break;
            case Position::INT:  m_position = Position::LOW;  break;
            case Position::LOW:  m_position = Position::HIGH; break;
            case Position::HIGH: m_position = Position::OFF;  break;
        }
    }

    /// Mark a momentary wash-button press (M key in ev1sim).
    void press_wash() { m_wash = true; }

    /// Consume the pending wash press — returns true once then clears.
    bool consume_wash() {
        bool v = m_wash;
        m_wash = false;
        return v;
    }

    Position position() const { return m_position; }

    // Wiper/washer switch (connector 12092254) output cavities, published
    // wire-level on the chassis bus.  HI layers on top of the LOW "request"
    // wire so the consuming decode `hi?HIGH : request?LOW : delay?INT : OFF`
    // is order-robust regardless of whether request is also asserted at HIGH.
    //   pin B, GRA 112   "WIPER DELAY OUT"    INT only      (chassis ID 4054)
    //   pin C, DK GRN 113 "REQUEST OUT"        LOW or HIGH   (chassis ID 4055)
    //   pin D, PPL 92    "HI OUT"             HIGH only     (chassis ID 4056)
    //   pin E, PNK 228C  "WASHER-SWITCH OUT"  wash pressed   (chassis ID 4057)
    // TODO(verify): confirm the position->cavity truth table against EV1 ESM
    // p.643; RHJB's decode must mirror exactly what lands here.
    bool pin_delay_out()   const { return m_position == Position::INT; }
    bool pin_request_out() const {
        return m_position == Position::LOW || m_position == Position::HIGH;
    }
    bool pin_hi_out()            const { return m_position == Position::HIGH; }
    bool pin_washer_switch_out() const { return m_wash; }

private:
    Position m_position = Position::OFF;
    bool     m_wash     = false;
};

/// Power window switches (driver + passenger × up + down).
///
/// EV1 is a 2-seater: driver window and passenger window, each with an up and
/// a down momentary switch.  Each switch publishes a boolean on the main
/// harness segment while held (true = held, false = released):
///   kSigDriverPowerWindowDriverUp    (6980)
///   kSigDriverPowerWindowDriverDown  (6981)
///   kSigDriverPowerWindowPassengerUp   (6982)
///   kSigDriverPowerWindowPassengerDown (6983)
///
/// No keyboard binding — the floating UI panel will call press()/release()
/// when its widget set expands.  consumer = RSA (future round).
class PowerWindows {
public:
    enum class Window    { DRIVER, PASSENGER };
    enum class Direction { NONE, UP, DOWN };

    /// Hold a switch while pressed.  Overwrites any previous direction for
    /// that window (press DOWN while already UP → state becomes DOWN).
    void press(Window w, Direction d) {
        (w == Window::DRIVER ? m_driver : m_passenger) = d;
    }

    /// Release: return the window to NONE (no switch held).
    void release(Window w) {
        (w == Window::DRIVER ? m_driver : m_passenger) = Direction::NONE;
    }

    Direction state(Window w) const {
        return (w == Window::DRIVER) ? m_driver : m_passenger;
    }

    bool driver_up()      const { return m_driver    == Direction::UP;   }
    bool driver_down()    const { return m_driver    == Direction::DOWN;  }
    bool passenger_up()   const { return m_passenger == Direction::UP;   }
    bool passenger_down() const { return m_passenger == Direction::DOWN;  }

private:
    Direction m_driver    = Direction::NONE;
    Direction m_passenger = Direction::NONE;
};

/// Door lock state model.
///
/// Tracks the lock/unlock state of driver door, passenger door, and trunk.
/// All doors default to UNLOCKED (vehicle starts unlocked per docs/TODO.md).
///
/// No keyboard binding for now — lock toggles will land via the floating-UI
/// panel (see docs/TODO.md "Floating UI panel" item).
///
/// No bus signal pinning for now — wait until an electricsim consumer wants it.
/// TODO: when an RSA central locking consumer exists in electricsim, add a
///       chassis bus signal (e.g. a per-door lock state signal) and publish it
///       from SimApp alongside the other PhysicalWorld signals.
class DoorLocks {
public:
    enum class State { LOCKED, UNLOCKED };

    State driver()    const { return m_driver; }
    State passenger() const { return m_passenger; }
    State trunk()     const { return m_trunk; }

    void lock_all() {
        m_driver    = State::LOCKED;
        m_passenger = State::LOCKED;
        m_trunk     = State::LOCKED;
    }

    void unlock_all() {
        m_driver    = State::UNLOCKED;
        m_passenger = State::UNLOCKED;
        m_trunk     = State::UNLOCKED;
    }

    void toggle_driver() {
        m_driver = (m_driver == State::LOCKED) ? State::UNLOCKED : State::LOCKED;
    }

    void toggle_passenger() {
        m_passenger = (m_passenger == State::LOCKED) ? State::UNLOCKED : State::LOCKED;
    }

    void toggle_trunk() {
        m_trunk = (m_trunk == State::LOCKED) ? State::UNLOCKED : State::LOCKED;
    }

    /// Set the lock state for a specific door directly.
    /// Used by the RSA mirror path (kSigChassisDoorLockCmd* 4084/4085).
    void set_driver(State s)    { m_driver    = s; }
    void set_passenger(State s) { m_passenger = s; }
    void set_trunk(State s)     { m_trunk     = s; }

    /// True if any door is locked (e.g. for "central lock state" telltale).
    bool any_locked() const {
        return m_driver    == State::LOCKED ||
               m_passenger == State::LOCKED ||
               m_trunk     == State::LOCKED;
    }

private:
    State m_driver    = State::UNLOCKED;  // doors default unlocked per docs/TODO.md
    State m_passenger = State::UNLOCKED;
    State m_trunk     = State::UNLOCKED;
};

/// Naive almanac-style ambient temperature + humidity sensor.
///
/// Models a smooth diurnal (24-hour) sinusoidal cycle.  No live weather API —
/// values are fully deterministic for a given Config and time-of-day input.
///
/// Diurnal model:
///   temp_c        = mean + diurnal_amp * sin(2π * (hour - phase_offset) / 24)
///   humidity_pct  = mean - diurnal_humidity_amp * sin(2π * (hour - phase_offset) / 24)
///   (humidity inversely correlated with temperature)
///
/// Defaults model a moderate spring day (18°C mean, 8°C swing, 55% RH,
/// peak temperature at 14:00 local time).
class AmbientTempSensor {
public:
    /// Configurable parameters; defaults model a moderate spring day.
    struct Config {
        double mean_temp_c            = 18.0;   ///< mean of the daily temp cycle (°C)
        double diurnal_amp_c          = 8.0;    ///< peak-to-trough swing / 2 (°C)
        double mean_humidity_pct      = 55.0;   ///< mean relative humidity (%)
        double diurnal_humidity_amp   = 15.0;   ///< humidity amplitude (inversely correlated)
        double phase_offset_hours     = 14.0;   ///< hour-of-day when peak temp occurs
        double seed_offset_c          = 0.0;    ///< per-run noise offset (0 = fully deterministic)
    };

    /// Replace configuration; takes effect on the next update() call.
    void set_config(const Config& cfg) { m_cfg = cfg; }

    /// Recompute temp + humidity from the current time of day.
    /// @param time_of_day_hours  Local time expressed in hours [0..24).
    void update(double time_of_day_hours);

    double temp_c()       const { return m_temp_c; }
    double humidity_pct() const { return m_humidity_pct; }

private:
    Config m_cfg{};
    double m_temp_c       = 18.0;
    double m_humidity_pct = 55.0;
};

/// Seatbelt buckle sensors — driver seat and passenger seat.
///
/// Models the two reed-switch-style buckle sensors in the EV1.  Both default
/// to buckled (true) matching the pre-UI behaviour where ev1sim always reported
/// the driver as buckled.
///
/// Signals:
///   kSigDriverSeatbeltBuckled (6964)          — driver
///   kSigDriverSeatbeltBuckledPassenger (6965) — passenger
///
/// TODO(consumer): IPC seatbelt-light telltale should wire passenger signal
/// (6965) — deferred to a future electricsim round.
class Seatbelts {
public:
    bool driver_buckled()    const { return m_driver;    }
    bool passenger_buckled() const { return m_passenger; }

    void set_driver(bool buckled)    { m_driver    = buckled; }
    void set_passenger(bool buckled) { m_passenger = buckled; }

    /// Toggle driver buckle state: BUCKLED ↔ UNBUCKLED.
    void toggle_driver()    { m_driver    = !m_driver;    }
    /// Toggle passenger buckle state: BUCKLED ↔ UNBUCKLED.
    void toggle_passenger() { m_passenger = !m_passenger; }

private:
    bool m_driver    = true;   // default: buckled (matches pre-UI behaviour)
    bool m_passenger = true;   // default: buckled
};

/// RSA exterior pillar keypad (5 buttons: 1/2, 3/4, 5/6, 7/8, 9/0).
///
/// Mirrors the encoding of the interior RsaKeypadDriver but driven directly
/// by the floating-UI panel (mouse-clickable) rather than the K key scheduler.
///
/// Long-press encoding (Option A, same as interior):
///   0 = idle, 1 = tap (lower digit), 2 = long-press (higher digit)
///
/// press_button(idx, long_press) queues a momentary value for the next tick.
/// clear_oneshots() resets all button values to idle after they are consumed.
/// enter_code_sequence(code_str) queues a sequence of individual taps/longs
/// that fire one per tick — used by the "Enter 111111" convenience button.
class RsaExteriorKeypad {
public:
    /// Queue a momentary button press on button idx (0..4).
    /// If long_press=false, sets value=1 (tap); if true, sets value=2 (long-press).
    void press_button(int button_idx, bool long_press);

    /// Read current value for button idx (0=idle, 1=tap, 2=long-press).
    std::uint8_t button_value(int idx) const;

    /// Convenience: true if button idx has a tap queued.
    bool button_tap(int idx) const  { return button_value(idx) == 1; }
    /// Convenience: true if button idx has a long-press queued.
    bool button_long(int idx) const { return button_value(idx) == 2; }

    /// Reset all button values to idle (call after consuming them each tick).
    void clear_oneshots();

    /// Queue the full 6-digit code as a sequence of button presses.
    /// code_str must be exactly 6 digit characters (0-9).
    /// Digits map to buttons: 1→btn0 tap, 2→btn0 long, 3→btn1 tap,
    /// 4→btn1 long, 5→btn2 tap, 6→btn2 long, 7→btn3 tap, 8→btn3 long,
    /// 9→btn4 tap, 0→btn4 long.
    /// Each press fires on a separate tick driven by tick_sequence()/
    /// consume_sequence_fires() below.
    void enter_code_sequence(const char* code_str);

    /// Tick the sequence emitter by dt_s seconds (100 ms between digits).
    void update(double dt_s);

    /// Consume any sequence fires for this tick — merges into pending button values.
    /// Returns true if a sequence digit fired this tick.
    bool consume_sequence_fire();

    /// True if a code sequence is currently in progress.
    bool sequence_in_progress() const;

private:
    // Per-button momentary values (0=idle, 1=tap, 2=long).
    std::uint8_t m_tap_value[5] = {};

    // Sequence emitter (for enter_code_sequence).
    static constexpr int kMaxCodeLen = 6;
    struct SeqEntry { std::uint8_t button_idx; bool long_press; };
    SeqEntry m_seq[kMaxCodeLen];
    int      m_seq_len    = 0;
    int      m_seq_pos    = 0;
    double   m_seq_timer  = 0.0;  // seconds until next digit fires
    bool     m_seq_active = false;
};

/// Door handle pull attempts (driver door + passenger door).
///
/// Momentary one-shot events — each tick the attempt flag is set, consumed by
/// SimApp which checks DoorLocks state and decides what to do.
///
/// Behavior when consumed (SimApp integration):
///   - If the corresponding door is LOCKED: emit log "Door X: LOCKED — try keypad code"
///     and publish the attempt signal (6990/6991) momentarily.
///   - If the corresponding door is UNLOCKED: open the door (toggle VehiclePanels
///     ajar state) and publish the attempt signal momentarily.
class DoorHandles {
public:
    /// Queue a momentary driver-door handle pull attempt.
    void attempt_driver()    { m_driver    = true; }
    /// Queue a momentary passenger-door handle pull attempt.
    void attempt_passenger() { m_passenger = true; }

    /// True if a driver-door attempt is queued.
    bool driver_attempt()    const { return m_driver;    }
    /// True if a passenger-door attempt is queued.
    bool passenger_attempt() const { return m_passenger; }

    /// Reset both attempt flags (call after consuming them each tick).
    void clear_oneshots() { m_driver = false; m_passenger = false; }

private:
    bool m_driver    = false;
    bool m_passenger = false;
};

/// Door-lock motor + mechanical lock stroke (one peripheral instance per door:
/// LH = driver, RH = passenger).
///
/// Consumes the RHJB dual-H-bridge motor-leg drives (chassis bus):
///   lock_drive    LOCK leg energised  → motor runs toward LOCKED
///   unlock_drive  UNLOCK leg energised → motor runs toward UNLOCKED
/// The motor runs in the direction whose drive is active and stops at the
/// end-of-travel limit.  Both drives high (an illegal H-bridge shoot-through
/// command) or both low → motor off (no motion).
///
/// Wire-level mapping (cavity → chassis-bus signal), authoritative for the
/// electricsim router — see docs/peripherals.md:
///   LH (driver):    lock=294A RHJB J9.C5 → 4092,  unlock=295A RHJB J9.C6 → 4093
///   RH (passenger): lock=294C RHJB J3.A6 → 4094,  unlock=295C RHJB J3.A7 → 4095
///
/// Mechanical model: a normalized stroke position in [0,1] (0 = UNLOCKED end
/// of travel, 1 = LOCKED end of travel).  While a single drive is active the
/// stroke advances at 1/traverse_time per second toward that end and clamps at
/// the limit.  stroke() reports LOCKED / UNLOCKED / MID_STROKE.
class DoorLockMotor {
public:
    enum class Stroke { UNLOCKED, MID_STROKE, LOCKED };

    struct Config {
        /// End-to-end travel time (UNLOCKED↔LOCKED).  Typical GM door-lock
        /// motor stroke is ~0.4–0.6 s; the default 0.5 s sits comfortably
        /// inside the RHJB DLM's 600 ms drive pulse, so the latch reaches its
        /// end-of-travel within a single pulse.
        double traverse_time_s = 0.5;
    };

    DoorLockMotor() = default;
    explicit DoorLockMotor(const Config& cfg) : m_cfg(cfg) {}

    /// Advance the mechanical stroke for dt seconds given the two H-bridge legs.
    void update(double dt, bool lock_drive, bool unlock_drive) {
        if (dt <= 0.0 || m_cfg.traverse_time_s <= 0.0) return;
        const double step = dt / m_cfg.traverse_time_s;
        if (lock_drive && !unlock_drive) {
            m_pos += step;            // run toward LOCKED
        } else if (unlock_drive && !lock_drive) {
            m_pos -= step;            // run toward UNLOCKED
        }
        // both high or both low → motor off (hold position).
        if (m_pos > 1.0) m_pos = 1.0; // end-of-travel limit (LOCKED)
        if (m_pos < 0.0) m_pos = 0.0; // end-of-travel limit (UNLOCKED)
    }

    /// Normalized stroke position: 0 = UNLOCKED end, 1 = LOCKED end.
    double position() const { return m_pos; }

    /// true while a single drive is moving the pawl off an end-of-travel limit.
    bool moving() const { return m_pos > 0.0 && m_pos < 1.0; }

    Stroke stroke() const {
        if (m_pos >= 1.0) return Stroke::LOCKED;
        if (m_pos <= 0.0) return Stroke::UNLOCKED;
        return Stroke::MID_STROKE;
    }

    const char* stroke_name() const {
        switch (stroke()) {
            case Stroke::LOCKED:   return "LOCKED";
            case Stroke::UNLOCKED: return "UNLOCKED";
            default:               return "MID-STROKE";
        }
    }

    /// Seed the stroke to a known position (tests / initial-state config).
    void set_position(double pos01) {
        m_pos = pos01 < 0.0 ? 0.0 : (pos01 > 1.0 ? 1.0 : pos01);
    }

private:
    Config m_cfg{};
    double m_pos = 0.0;   // default UNLOCKED (matches DoorLocks default)
};

/// Piezo sounder — the LHJB turn/hazard "click" (and any future chime).
///
/// Consumes a single boolean drive (the LHJB flasher's piezo square-wave
/// output, chassis ID 4096).  The flasher toggles the drive each flash
/// half-cycle; the sounder emits an audible tick while the drive is high.
/// No frequency model — the piezo is a fixed-pitch element; a fancier model
/// could carry a frequency/level later.
///
/// The piezo is a real LHJB-internal component the printed EV1 schematics are
/// silent on (no first-class component_id) — see docs/peripherals.md.
class Sounder {
public:
    /// Update from the piezo drive line.  Call once per tick.  Each rising
    /// edge (false→true) counts one audible click.
    void update(bool drive) {
        if (drive && !m_drive) ++m_click_count;   // rising edge
        m_drive = drive;
    }

    /// true while the piezo is energised — the audible-output signal a future
    /// 3D-sim audio backend plays.
    bool sounding() const { return m_drive; }

    /// Number of rising edges (clicks) since construction.
    unsigned long click_count() const { return m_click_count; }

private:
    bool          m_drive       = false;
    unsigned long m_click_count = 0;
};

/// Power-steering pump motor (3-phase BLDC driven by the PSCM HV inverter).
///
/// Minimum-viable plant: consumes a single commanded speed (normalized 0..1,
/// the PSCM's q8 pump-speed command / 255) and tracks an actual speed with a
/// first-order spin-up/spin-down lag.  Real BLDC commutation across the three
/// molex phases (A/B/C) is intentionally ignored — per-phase modeling is
/// future work.
///
/// The motor body also closes the HV interlock loop (molex.D/E): while the
/// peripheral is present/connected, interlock_closed() is true, which the PSCM
/// senses.  set_present(false) opens the loop for fault injection.
///
/// Wire-level mapping (see docs/peripherals.md):
///   PSCM molex.A/B/C → pump phase_A/B/C   (commanded speed, chassis 4097)
///   pump molex.D/E   → PSCM HV INTERLOCK   (interlock-closed, chassis 4098)
class PowerSteeringPumpMotor {
public:
    struct Config {
        /// First-order response time constant (s): the actual speed chases the
        /// commanded speed with this lag.  Default ~0.15 s (a responsive
        /// accessory BLDC).  0 → instantaneous plant.
        double response_tau_s = 0.15;
    };

    PowerSteeringPumpMotor() = default;
    explicit PowerSteeringPumpMotor(const Config& cfg) : m_cfg(cfg) {}

    /// Advance the plant.  commanded_speed is normalized 0..1 and clamped.
    void update(double dt, double commanded_speed) {
        m_cmd = commanded_speed < 0.0 ? 0.0
                                      : (commanded_speed > 1.0 ? 1.0 : commanded_speed);
        if (dt <= 0.0 || m_cfg.response_tau_s <= 0.0) {
            m_actual = m_cmd;   // instantaneous plant
            return;
        }
        // Exponential approach: actual += (cmd - actual)·(1 - e^{-dt/tau}).
        const double alpha = 1.0 - std::exp(-dt / m_cfg.response_tau_s);
        m_actual += (m_cmd - m_actual) * alpha;
    }

    double commanded_speed() const { return m_cmd; }     // 0..1
    double actual_speed()    const { return m_actual; }  // 0..1, after lag

    /// HV interlock loop (molex.D/E): closed while the motor is present.
    bool interlock_closed() const { return m_present; }
    void set_present(bool present) { m_present = present; }

private:
    Config m_cfg{};
    double m_cmd     = 0.0;
    double m_actual  = 0.0;
    bool   m_present = true;   // motor present → interlock loop closed
};

/// Container for all physical-world components: driver-operated inputs
/// (switches, pedals, keypads) and actuator/plant models driven by the
/// external electrical sim (door-lock motors, sounder, steering pump).
class PhysicalWorld {
public:
    CombinationSwitch&       combination_switch()       { return m_comb_sw; }
    const CombinationSwitch& combination_switch() const { return m_comb_sw; }

    BrakeSwitch&       brake_switch()       { return m_brake_sw; }
    const BrakeSwitch& brake_switch() const { return m_brake_sw; }

    BrakePedal&       brake_pedal()       { return m_brake_pedal; }
    const BrakePedal& brake_pedal() const { return m_brake_pedal; }

    ChargeCoupler&       charge_coupler()       { return m_charge_coupler; }
    const ChargeCoupler& charge_coupler() const { return m_charge_coupler; }

    PrndSelector&       prnd_selector()       { return m_prnd_sel; }
    const PrndSelector& prnd_selector() const { return m_prnd_sel; }

    TurnSignalStalk&       turn_signal_stalk()       { return m_turn_stalk; }
    const TurnSignalStalk& turn_signal_stalk() const { return m_turn_stalk; }

    HazardSwitch&       hazard_switch()       { return m_hazard_sw; }
    const HazardSwitch& hazard_switch() const { return m_hazard_sw; }

    HornButton&       horn_button()       { return m_horn_button; }
    const HornButton& horn_button() const { return m_horn_button; }

    RsaKeypadDriver&       rsa_keypad()       { return m_rsa_keypad; }
    const RsaKeypadDriver& rsa_keypad() const { return m_rsa_keypad; }

    IpcTripResetButton&       ipc_trip_reset()       { return m_ipc_trip_reset; }
    const IpcTripResetButton& ipc_trip_reset() const { return m_ipc_trip_reset; }

    CruiseStalk&       cruise_stalk()       { return m_cruise_stalk; }
    const CruiseStalk& cruise_stalk() const { return m_cruise_stalk; }

    WiperStalk&       wiper_stalk()       { return m_wiper_stalk; }
    const WiperStalk& wiper_stalk() const { return m_wiper_stalk; }

    DoorLocks&       door_locks()       { return m_door_locks; }
    const DoorLocks& door_locks() const { return m_door_locks; }

    Seatbelts&       seatbelts()       { return m_seatbelts; }
    const Seatbelts& seatbelts() const { return m_seatbelts; }

    AmbientTempSensor&       ambient_temp_sensor()       { return m_ambient_temp; }
    const AmbientTempSensor& ambient_temp_sensor() const { return m_ambient_temp; }

    PowerWindows&       power_windows()       { return m_power_windows; }
    const PowerWindows& power_windows() const { return m_power_windows; }

    RsaExteriorKeypad&       rsa_exterior_keypad()       { return m_rsa_ext_keypad; }
    const RsaExteriorKeypad& rsa_exterior_keypad() const { return m_rsa_ext_keypad; }

    DoorHandles&       door_handles()       { return m_door_handles; }
    const DoorHandles& door_handles() const { return m_door_handles; }

    // Door-lock motor plant — two instances: LH (driver) and RH (passenger).
    DoorLockMotor&       door_lock_motor_lh()       { return m_door_lock_motor_lh; }
    const DoorLockMotor& door_lock_motor_lh() const { return m_door_lock_motor_lh; }
    DoorLockMotor&       door_lock_motor_rh()       { return m_door_lock_motor_rh; }
    const DoorLockMotor& door_lock_motor_rh() const { return m_door_lock_motor_rh; }

    Sounder&       sounder()       { return m_sounder; }
    const Sounder& sounder() const { return m_sounder; }

    PowerSteeringPumpMotor&       power_steering_pump()       { return m_steering_pump; }
    const PowerSteeringPumpMotor& power_steering_pump() const { return m_steering_pump; }

    /// Draw HUD overlays for: key state, combination switch, PRND selector,
    /// and turn signals/hazard.  Call between BeginScene and EndScene.
    /// rsa_run_mode: most recently received RSA run mode (0=OFF,1=ACC,2=RUN;
    ///               pass 0xFF if not yet received).
    void DrawHUD(irr::IrrlichtDevice* device,
                 std::uint8_t rsa_run_mode, bool has_rsa_run_mode) const;

    /// Draw the physical-world snapshot overlay panel.
    /// Lists all PhysicalWorld component states; default hidden.
    /// Toggle with Z key via ConsumeSnapshotToggle().
    /// @param device    Irrlicht device (null-safe; skips draw if null)
    /// @param show      true = panel visible, false = hidden
    /// @param rsa_run_mode  most recently received RSA run mode
    /// @param has_rsa_run_mode  true if run mode has been received
    /// @param trip_reset_count  number of IPC trip-reset presses so far
    void DrawSnapshotOverlay(irr::IrrlichtDevice* device,
                             bool show,
                             std::uint8_t rsa_run_mode,
                             bool has_rsa_run_mode,
                             int trip_reset_count) const;

private:
    CombinationSwitch  m_comb_sw;
    BrakeSwitch        m_brake_sw;
    BrakePedal         m_brake_pedal;
    ChargeCoupler      m_charge_coupler;
    PrndSelector       m_prnd_sel;
    TurnSignalStalk    m_turn_stalk;
    HazardSwitch       m_hazard_sw;
    HornButton         m_horn_button;
    RsaKeypadDriver    m_rsa_keypad;
    IpcTripResetButton m_ipc_trip_reset;
    CruiseStalk        m_cruise_stalk;
    WiperStalk         m_wiper_stalk;
    DoorLocks          m_door_locks;
    Seatbelts          m_seatbelts;
    AmbientTempSensor  m_ambient_temp;
    PowerWindows       m_power_windows;
    RsaExteriorKeypad  m_rsa_ext_keypad;
    DoorHandles        m_door_handles;
    // Actuator/plant peripherals driven by the external electrical sim.
    DoorLockMotor          m_door_lock_motor_lh;   // driver door
    DoorLockMotor          m_door_lock_motor_rh;   // passenger door
    Sounder                m_sounder;
    PowerSteeringPumpMotor m_steering_pump;
};

}  // namespace ev1sim
