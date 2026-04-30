#pragma once

#include "VehicleLights.h"
#include "VehiclePanels.h"
#include "VehicleState.h"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

/// Connects the vehicle simulator to an external electric simulator over the
/// electricsim shared-memory I/O fabric.
///
/// The connection is non-blocking: Start() returns immediately, and the
/// transport is (re)opened lazily inside Tick().  If the electricsim library
/// is not available at build time the connector compiles as a stub that
/// reports "not available" and does nothing — callers can unconditionally
/// drive it.
///
/// Endpoints exposed (see Endpoints()):
///   - 19 bulb command inputs       (electric sim -> ev1sim), one per LightID
///   - 2  horn tone command inputs  (electric sim -> ev1sim), low + high
///   - 4  panel ajar switch outputs (ev1sim -> electric sim), hood/trunk/L/R door
///   - 15 vehicle dynamics outputs  (ev1sim -> electric sim), float32 each:
///        speed, accel (long/lat), yaw rate, throttle/brake commands,
///        per-wheel angular speeds (FL/FR/RL/RR), per-wheel slip ratios
///
/// Signal IDs live in the 4000-block so they don't collide with the harness
/// example's 3000-block.
///
/// Integration boundary
/// --------------------
/// This connector exposes ev1sim as a flat list of buttons, lights, switches,
/// and sensors on a dedicated chassis bus.  It is deliberately ignorant of
/// electricsim's module structure — there is no concept of BTCM, BPM, AD,
/// charger, or any other ECU here.  When adding new endpoints, name them in
/// vehicle terms (`vehicle.panel.*`, `vehicle.dynamics.*`, `avr0.lights.*`),
/// never in ECU/module terms.  Anything that needs to know "which ECU
/// consumes this signal" lives on the electricsim side of the boundary.
class ExternalSimConnector {
public:
    struct Options {
        bool        enabled          = false;
        std::string bus_name         = "electricsim_chassis_bus";
        std::string main_harness_bus_name = "electricsim_ev1_bus";
        // How often to republish SignalDefine frames so other bus peers can
        // introspect our endpoints.
        double      presence_period_s = 2.0;
        // Back-off between reconnect attempts when the transport fails to open.
        double      reconnect_period_s = 1.0;
    };

    enum class Status {
        Disabled,        // --external-sim is off
        Unavailable,     // built without electricsim; always disconnected
        Connecting,      // enabled, transport not yet open
        Connected,       // transport open, poll/publish live
    };

    /// Endpoints exposed (see Endpoints()):
    ///   - 19 bulb command inputs            (electric sim → ev1sim)
    ///   - 2  horn tone command inputs        (electric sim → ev1sim)
    ///   - 4  panel ajar switch outputs       (ev1sim → electric sim)
    ///   - 3  combination switch pin outputs  (ev1sim → electric sim)
    ///     4040 combination_switch.low_beam_out         (pin C, YEL 525B)
    ///     4041 combination_switch.flash_to_pass_out    (pin B, PPL 524B)
    ///     4042 combination_switch.park_headlamp_out    (pin F, LTBLU 74)
    ///   - 15 vehicle dynamics outputs        (ev1sim → electric sim)

    /// An externally-visible endpoint on the I/O fabric.
    struct Endpoint {
        std::uint32_t signal_id;
        const char*   qualified_name;   // e.g. "vehicle.body.lhbh.bulb_feed_line"
        const char*   short_name;       // e.g. "lhbh_bulb_feed_line"
        bool          input_to_sim;     // true: electric sim drives this; false: ev1sim publishes
    };

    ExternalSimConnector();
    explicit ExternalSimConnector(const Options& options);
    ~ExternalSimConnector();

    ExternalSimConnector(const ExternalSimConnector&) = delete;
    ExternalSimConnector& operator=(const ExternalSimConnector&) = delete;

    // ---------------------------------------------------------------------
    // Lifecycle
    // ---------------------------------------------------------------------
    /// Kick off the first connection attempt.  Safe to call on a disabled
    /// connector (does nothing).
    void Start();

    /// Drain incoming frames, publish outgoing deltas, and handle any
    /// reconnection retries.  Call once per render frame; sim_time_s is used
    /// for the presence / reconnect timers.
    void Tick(double sim_time_s);

    /// Release the transport; further Tick() calls become no-ops until Start().
    void Stop();

    Status      GetStatus()     const;
    bool        IsConnected()   const { return GetStatus() == Status::Connected; }
    const char* StatusString()  const;
    const Options& GetOptions() const { return m_opts; }

    // ---------------------------------------------------------------------
    // State accessors (valid regardless of connection status)
    // ---------------------------------------------------------------------
    /// Command most recently received from the electric sim for a given bulb.
    /// Returns false if the endpoint has never been written to (i.e. electric
    /// sim hasn't published it yet).
    bool GetBulbCmd(LightID id) const;

    /// Two-tone horn commands from the electric sim.
    bool GetHornLowCmd()  const;
    bool GetHornHighCmd() const;

    /// Whether we've ever received a command for any bulb.  SimApp uses this
    /// to decide whether the external sim is actively driving the lamps.
    bool HasReceivedBulbData() const;

    /// Outgoing panel ajar state — SimApp writes this every frame; the
    /// connector publishes a DeltaBatch only when a value changes.
    void SetPanelSensor(PanelID panel, bool ajar);
    bool GetPanelSensor(PanelID panel) const;

    /// Outgoing vehicle dynamics snapshot — SimApp writes this every frame
    /// and Tick() publishes it as a float32 DeltaBatch so the electric sim
    /// can model BTCM, regen braking, ABS, etc.
    void SetVehicleState(const VehicleState& state);

    /// Outgoing combination switch pin states — three meaningful output pins
    /// (B, C, F) per the service manual for the 6-way blue 12084699 connector.
    /// Published as wire-level booleans on the chassis bus (IDs 4040-4042).
    void SetCombSwOutputs(bool low_beam, bool flash_to_pass, bool park_headlamp);

    /// Outgoing driver inputs — published to the main harness segment
    /// (electricsim_ev1_bus, IDs 6900-6903).
    /// q8 values: brake/throttle 0..255 (normalized × 256), steering signed
    /// degrees × 256, gear enum 0=P 1=R 2=N 3=D.
    void SetDriverBrakePedalQ8(std::uint8_t q8);
    void SetDriverThrottleQ8(std::uint8_t q8);
    void SetDriverSteeringDegQ8(std::int16_t q8);
    void SetDriverGearSelector(std::uint8_t enum_v);

    /// Outgoing brake-switch state (ID 6904) — derived from brake travel
    /// hysteresis in PhysicalWorld::BrakeSwitch.  Encoded as 1-byte uint8
    /// (0=released, 1=applied).  Published on main harness segment.
    void SetDriverBrakeSwitch(bool pressed);

    /// Outgoing seatbelt buckle state (ID 6964).  Encoded as 1-byte uint8
    /// (0=unbuckled, 1=buckled).  Published on main harness segment.
    /// Defaults to true (driver always buckled) until a UI toggle is added —
    /// see docs/TODO.md for the floating-UI panel item.
    void SetDriverSeatbeltBuckled(bool buckled);

    /// Outgoing turn-signal stalk position (IDs 6948, 6949).
    /// Encoded as 1-byte uint8 boolean each (0=inactive, 1=active).
    /// Published on the main harness segment in lockstep with
    /// electricsim/src/io/ev1_driver_inputs.hpp kSigDriverTurnSignalLeft/Right.
    void SetDriverTurnSignalLeft(bool active);
    void SetDriverTurnSignalRight(bool active);

    /// Outgoing hazard switch request (ID 6944).
    /// Encoded as 1-byte uint8 boolean (0=off, 1=on).
    /// Published on the main harness segment in lockstep with
    /// electricsim/src/io/ev1_driver_inputs.hpp kSigDriverHazardRequest.
    void SetDriverHazardRequest(bool on);

    /// Outgoing charge coupler presence — publishes delta on change on the
    /// chassis segment (ID 4060).  Stubbed false today; future floating-UI
    /// panel or charge-door animation will call set_present(true).
    void SetChargeCouplerPresent(bool present);

    /// Outgoing PRND selector wire-level pin states — four lines from the
    /// floor selector lever to the PIM (IDs 4050-4053, chassis segment).
    /// Encoding: Gray-coded with even parity per propulsion manual p. 343.
    ///   PARK:    a=0 b=1 c=1 d=0
    ///   REVERSE: a=0 b=0 c=1 d=1
    ///   NEUTRAL: a=1 b=0 c=1 d=0
    ///   DRIVE:   a=1 b=0 c=0 d=1
    /// SimApp calls this each tick with the PrndSelector's current pin outputs.
    void SetPrndSelector(bool a, bool b, bool c, bool d);

    /// Outgoing RSA per-digit keypad button signals (IDs 6975-6979, main harness).
    /// Encoded as 1-byte uint8 with long-press support (Option A encoding):
    ///   0 = idle (no press this tick)
    ///   1 = tap  (enters the lower digit: 1, 3, 5, 7, 9 respectively)
    ///   2 = long-press (enters the higher digit: 2, 4, 6, 8, 0 respectively)
    ///   Button1 (6975): "1/2" button
    ///   Button2 (6976): "3/4" button
    ///   Button3 (6977): "5/6" button
    ///   Button4 (6978): "7/8" button
    ///   Button5 (6979): "9/0" button
    /// (Slot 6970 is reserved — was kSigDriverRsaKeypadCodeOk; do not reuse.)
    void SetDriverRsaKeypadButton1(std::uint8_t value);
    void SetDriverRsaKeypadButton2(std::uint8_t value);
    void SetDriverRsaKeypadButton3(std::uint8_t value);
    void SetDriverRsaKeypadButton4(std::uint8_t value);
    void SetDriverRsaKeypadButton5(std::uint8_t value);

    /// Outgoing RSA mode-button press (ID 6971, main harness segment).
    /// Momentary 1-tick pulse indicating the user's button press.
    /// Encoded as 1-byte uint8: 0=NONE, 1=OFF, 2=ACC, 3=RUN, 4=START.
    void SetDriverRsaModeButton(std::uint8_t button_enum);

    /// Outgoing IPC trip-reset button (ID 6952, main harness segment).
    /// Momentary 1-byte uint8 bool (0=idle, 1=pressed this tick).
    /// Published by ev1sim on the I key.
    void SetDriverIpcTripReset(bool pressed);

    /// Outgoing cruise-control stalk momentary buttons (IDs 6953-6957).
    /// Each is a 1-byte uint8 bool (0=idle, 1=pressed this tick).
    /// G=SET, Y=RESUME, N=CANCEL, +(=)=SPEED_UP, -=SPEED_DOWN in ev1sim.
    void SetDriverCruiseSet(bool pressed);
    void SetDriverCruiseResume(bool pressed);
    void SetDriverCruiseCancel(bool pressed);
    void SetDriverCruiseSpeedUp(bool pressed);
    void SetDriverCruiseSpeedDown(bool pressed);

    /// Outgoing wiper stalk position (ID 6958, main harness segment).
    /// 1-byte uint8 enum: 0=OFF, 1=INT, 2=LOW, 3=HIGH.
    /// V key in ev1sim cycles: OFF → INT → LOW → HIGH → OFF.
    void SetDriverWiperSwitch(std::uint8_t position);

    /// Outgoing wiper wash request (ID 6959, main harness segment).
    /// Momentary 1-byte uint8 bool (0=idle, 1=pressed this tick).
    /// M key in ev1sim.
    void SetDriverWiperWashRequest(bool pressed);

    /// Outgoing motor RPM (ID 4070, chassis segment, float32 LE).
    /// Motor shaft speed in RPM (positive forward).
    void SetMotorRpm(float rpm);

    /// Outgoing motor torque (ID 4071, chassis segment, float32 LE).
    /// Motor shaft torque in Nm (signed; positive = driving torque).
    void SetMotorTorqueNm(float torque_nm);

    /// Incoming RSA run-mode broadcast (ID 5711, main harness segment).
    /// uint8 enum: 0=OFF, 1=ACC, 2=RUN.  RSA publishes this on every tick.
    /// Returns the most recently received value, or 0xFF if never received.
    std::uint8_t GetRsaRunMode() const;

    /// Returns true if we have received at least one RSA run-mode broadcast.
    bool HasReceivedRunMode() const;

    /// ABS phase decoded from BTCM front solenoid (iso, dump) tuples.
    /// BTCM publishes kSigSolFL_ISO (5010), kSigSolFL_DMP (5011),
    /// kSigSolFR_ISO (5012), kSigSolFR_DMP (5013) on the main harness segment.
    ///
    /// Decoding table:
    ///   iso=0, dump=0 → APPLY  (full local brake)
    ///   iso=1, dump=0 → HOLD   (freeze previous brake torque)
    ///   iso=1, dump=1 → DUMP   (release toward zero — 0.2× local brake)
    ///   iso=0, dump=1 → invalid; treated as APPLY
    struct AbsPhaseFront {
        enum class Phase { APPLY, HOLD, DUMP };
        Phase fl       = Phase::APPLY;
        Phase fr       = Phase::APPLY;
        bool  fl_fresh = false;   ///< true if FL data arrived within freshness_window
        bool  fr_fresh = false;   ///< true if FR data arrived within freshness_window
    };

    /// Return the current per-wheel ABS phase for the front axle.
    /// A wheel's phase is APPLY and marked stale (!fresh) when no iso/dump
    /// update has arrived within freshness_window.
    AbsPhaseFront GetAbsPhaseFront(
        std::chrono::milliseconds freshness_window) const;

    // ---------------------------------------------------------------------
    // Endpoint registry (static — stable across instances)
    // ---------------------------------------------------------------------
    static const Endpoint* Endpoints();
    static int             EndpointCount();
    static const Endpoint* FindEndpoint(std::uint32_t signal_id);

    // ---------------------------------------------------------------------
    // Test hooks — feed the connector synthetic delta records as though they
    // arrived over the bus.  Used by unit tests; not part of the runtime path.
    // ---------------------------------------------------------------------
    void DebugInjectDelta(std::uint32_t signal_id, bool value);

private:
    Options m_opts;

    struct State;
    std::unique_ptr<State> m_state;
};
