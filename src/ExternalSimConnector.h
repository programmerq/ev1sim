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

    /// Outgoing turn/hazard combination-switch (12092237) and wiper/washer
    /// switch (12092254) pin states — published wire-level on the chassis bus.
    /// Turn/hazard: right/left turn, hazard-active, single horn cmd (4043-4046).
    /// Wiper: delay/request/hi/washer (4054-4057).
    void SetTurnHazSwOutputs(bool right_turn, bool left_turn, bool hazard, bool horn);
    void SetWiperWasherSwOutputs(bool delay, bool request, bool hi, bool washer);

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
    /// Driven by PhysicalWorld::Seatbelts.driver_buckled() each tick.
    void SetDriverSeatbeltBuckled(bool buckled);

    /// Outgoing passenger seatbelt buckle state (ID 6965).  Mirrors driver
    /// (6964) for the passenger seat.  Published on main harness segment.
    /// Driven by PhysicalWorld::Seatbelts.passenger_buckled() each tick.
    /// TODO(consumer): IPC seatbelt-light telltale — deferred to future round.
    void SetDriverSeatbeltBuckledPassenger(bool buckled);

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

    /// Outgoing cruise-control switch RAW CONTACTS — chassis cavities
    /// kSigCruiseSw_{SetCoast,ResumeAccel,OnOff}Out (4047/4048/4049, circuits
    /// 84/87/397 at PIM J1).  Each is a 1-byte uint8 bool: 1 = contact closed.
    /// ev1sim does NOT pre-decode tap vs. hold — it only opens/closes the
    /// contacts; PIM's tap/hold decoder (pim_cruise_input) turns a brief
    /// SET/COAST close into SET and a sustained one into SPEED_DOWN (likewise
    /// RESUME/ACCEL → RESUME / SPEED_UP), and a falling edge of ON/OFF into
    /// CANCEL (while ON/OFF is open the other two sit electrically dead).
    void SetCruiseSetCoastContact(bool closed);
    void SetCruiseResumeAccelContact(bool closed);
    void SetCruiseOnOffContact(bool closed);

    /// Outgoing wiper stalk position (ID 6958, main harness segment).
    /// 1-byte uint8 enum: 0=OFF, 1=INT, 2=LOW, 3=HIGH.
    /// V key in ev1sim cycles: OFF → INT → LOW → HIGH → OFF.
    void SetDriverWiperSwitch(std::uint8_t position);

    /// Outgoing wiper wash request (ID 6959, main harness segment).
    /// Momentary 1-byte uint8 bool (0=idle, 1=pressed this tick).
    /// M key in ev1sim.
    void SetDriverWiperWashRequest(bool pressed);

    /// 3D-sim integration contract — driver buttons and discrete sensors
    /// (main harness segment).  See electricsim docs/3d_sim_contract.md
    /// §1.3 (buttons) and §1.4 (discrete sensors).  All publish on change
    /// with -1 sentinel for first-publish.
    ///
    /// (Horn requests 6940/6941 removed — the driver horn is a single contact,
    ///  circuit 28, now published as the chassis horn cavity 4046 via
    ///  SetTurnHazSwOutputs.  The bulb-side horn drive lines 4020/4021 — the two
    ///  sounders — are unchanged and still flow electric→ev1sim for audio.)

    /// Headlight switch position (6942): 0=OFF, 1=PARK, 2=ON (low beam),
    /// 3=HI (high beam).  Mirror of CombinationSwitch::Position.
    void SetDriverHeadlightSwitch(std::uint8_t position);

    /// Headlight dim request (6943): momentary bool — the flash-to-pass
    /// lever held state from CombinationSwitch::flash_to_pass_held().
    void SetDriverHeadlightDimRequest(bool held);

    /// Telltale-test request (6945): momentary bool.  No ev1sim UI source
    /// yet — defaults to false.  Reserved for a future bulb-check button.
    void SetDriverTelltaleTestRequest(bool pressed);

    /// Park-brake set / release momentary requests (6946/6947).  These
    /// are derived edge events from DriverCommand.parking_brake: a
    /// false→true transition fires set; true→false fires release.
    /// Both are one-tick pulses; the caller is responsible for emitting
    /// exactly one set pulse per engage event and one release pulse per
    /// disengage event.
    void SetDriverParkBrakeSetRequest(bool pressed);
    void SetDriverParkBrakeReleaseRequest(bool pressed);

    /// Key position (6966): 0=OFF, 1=ACC, 2=RUN, 3=START.  ev1sim does
    /// not model a key-cycle state machine — the simulator behaves as if
    /// the vehicle is always RUN.  Default value is 2 (RUN); the setter
    /// allows future host code to drive ACC/START transitions when key
    /// modeling lands.
    void SetSensorKeyPosition(std::uint8_t position);

    /// Outgoing power window switch states (IDs 6980-6983, main harness segment).
    /// Each is a 1-byte uint8 momentary bool (0=released, 1=held this tick).
    /// EV1 is a 2-seater: driver window (up/down) + passenger window (up/down).
    /// No keyboard binding — floating UI panel will drive press()/release() when
    /// its widget set expands.  consumer = RSA (window-motor logic), future round.
    void SetDriverPowerWindowDriverUp(bool held);
    void SetDriverPowerWindowDriverDown(bool held);
    void SetDriverPowerWindowPassengerUp(bool held);
    void SetDriverPowerWindowPassengerDown(bool held);

    /// Outgoing RSA exterior pillar keypad button signals (IDs 6985-6989, main harness).
    /// Same Option A encoding as interior keypad (6975-6979):
    ///   0 = idle, 1 = tap (lower digit), 2 = long-press (higher digit)
    ///   ExteriorKeypad1 (6985): "1/2" button
    ///   ExteriorKeypad2 (6986): "3/4" button
    ///   ExteriorKeypad3 (6987): "5/6" button
    ///   ExteriorKeypad4 (6988): "7/8" button
    ///   ExteriorKeypad5 (6989): "9/0" button
    /// Consumer = RSA exterior keypad logic (future round).
    void SetDriverRsaExteriorKeypad1(std::uint8_t value);
    void SetDriverRsaExteriorKeypad2(std::uint8_t value);
    void SetDriverRsaExteriorKeypad3(std::uint8_t value);
    void SetDriverRsaExteriorKeypad4(std::uint8_t value);
    void SetDriverRsaExteriorKeypad5(std::uint8_t value);

    /// Outgoing door handle pull attempt signals (IDs 6990-6991, main harness).
    /// Momentary 1-byte uint8 bool (0=idle, 1=handle pulled this tick).
    /// ev1sim publishes these when the floating-UI door handle buttons are clicked.
    /// Consumer = RSA (decides whether to unlock based on authorization state).
    void SetDriverDoorHandleAttemptDriver(bool attempted);
    void SetDriverDoorHandleAttemptPassenger(bool attempted);

    /// Outgoing motor RPM (ID 4070, chassis segment, float32 LE).
    /// Motor shaft speed in RPM (positive forward).
    void SetMotorRpm(float rpm);

    /// Outgoing motor torque (ID 4071, chassis segment, float32 LE).
    /// Motor shaft torque in Nm (signed; positive = driving torque).
    void SetMotorTorqueNm(float torque_nm);

    /// Outgoing ambient air temperature (ID 4090, chassis segment, float32 LE).
    /// Published by ev1sim's AmbientTempSensor on each tick (epsilon-gated).
    /// Intended consumers: HTCM (heat-pump model), BPM (battery thermal).
    void SetAmbientTempC(float temp_c);

    /// Outgoing ambient relative humidity (ID 4091, chassis segment, float32 LE).
    /// Range: 0..100%.  Published by ev1sim's AmbientTempSensor on each tick
    /// (epsilon-gated).  Inversely correlated with temperature by the diurnal model.
    void SetAmbientHumidityPct(float humidity_pct);

    /// Outgoing brake master cylinder pressure (ID 4074, chassis segment, float32 LE).
    /// Computed by BrakePedal from pedal travel; consumed by BTCM as the
    /// primary brake-effort input.  Units: kPa.
    void SetBrakeMasterPressureKpa(float pressure_kpa);

    /// Incoming RSA run-mode broadcast (ID 5711, main harness segment).
    /// uint8 enum: 0=OFF, 1=ACC, 2=RUN.  RSA publishes this on every tick.
    /// Returns the most recently received value, or 0xFF if never received.
    std::uint8_t GetRsaRunMode() const;

    /// Returns true if we have received at least one RSA run-mode broadcast.
    bool HasReceivedRunMode() const;

    /// ABS phase decoded from BTCM front solenoid (iso_close, dump_open) tuples.
    ///
    /// The connector subscribes to two parallel publications of the same
    /// actuator state:
    ///   * Chassis-bus segment (preferred): IDs 4147-4150
    ///       4147 iso_close FL, 4148 iso_close FR, 4149 dump_open FL,
    ///       4150 dump_open FR (all uint8 bool, "1" = closed/open).
    ///   * Main-harness segment (legacy fallback): IDs 5010-5013
    ///       kSigSolFL_ISO/DMP and kSigSolFR_ISO/DMP.
    /// Whichever source first delivers data for a given wheel becomes
    /// authoritative for that wheel; if a chassis-bus update has ever
    /// arrived for a wheel it is preferred over the main-harness value.
    ///
    /// Decoding table (iso=iso_close, dump=dump_open):
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

    /// Rear EMB motor commands from BTCM, subscribed in parallel on two segments:
    ///   * Chassis-bus (preferred): IDs 4151 (LR) / 4152 (RR), float32 LE.
    ///   * Main harness (legacy fallback): kSigRearMotorLR/RR = 5014/5015.
    /// Float in [-1, +1]: +1=apply, 0=idle, -1=release.  Whichever source
    /// has ever delivered a value for a given wheel becomes authoritative;
    /// if a chassis-bus value has ever arrived it is preferred.
    /// fresh = true when the producer is alive (per the BTCM canonical-frame
    /// heartbeat freshness_window).
    struct RearEmbCmd {
        float lr        = 0.0f;
        float rr        = 0.0f;
        bool  lr_fresh  = false;
        bool  rr_fresh  = false;
    };
    RearEmbCmd GetRearEmbCmd(
        std::chrono::milliseconds freshness_window) const;

    /// Per-front-wheel cylinder pressure published by BTCM on the chassis
    /// bus (IDs 4153 FL / 4154 FR, float32 LE, kPa).  This is the
    /// post-modulator pressure measured downstream of the iso/dump
    /// solenoid pair — distinct from the master-cylinder pressure
    /// (kSigChassisBrakeMasterPressureKpa = 4074) that ev1sim itself
    /// publishes upstream.  Use it for instrumentation/HUD or for a
    /// future pressure-driven brake plant; the existing brake plant
    /// gates torque on AbsPhaseFront and does not need this directly.
    /// fresh = true when the BTCM is alive per the freshness_window.
    struct FrontWheelCylinderPressures {
        float fl_kpa  = 0.0f;
        float fr_kpa  = 0.0f;
        bool  fl_fresh = false;
        bool  fr_fresh = false;
    };
    FrontWheelCylinderPressures GetFrontWheelCylinderPressuresKpa(
        std::chrono::milliseconds freshness_window) const;

    // ---------------------------------------------------------------------
    // Endpoint registry (static — stable across instances)
    // ---------------------------------------------------------------------
    static const Endpoint* Endpoints();
    static int             EndpointCount();
    static const Endpoint* FindEndpoint(std::uint32_t signal_id);

    /// Incoming wiper motor command (ID 4080, chassis segment).
    /// Received from RHJB. uint8 enum: 0=OFF, 1=INT, 2=LOW, 3=HIGH.
    /// Returns 0xFF if never received.
    std::uint8_t GetWiperMotorCommand() const;
    bool         HasReceivedWiperMotorCommand() const;

    /// Throttle command from PIM (ID 4073, chassis segment) — q8 throttle.
    /// q8=0 → zero throttle, q8=255 → full throttle.
    /// fresh = true when the last update arrived within freshness_window.
    /// ever_received = true after the first message; false on a cold start.
    /// Used by SimApp::ApplyElectronicsThrottle() to decide whether to
    /// override the local pedal in "electronics" drive mode.
    struct ThrottleCmd {
        std::uint8_t q8            = 0xFFu;
        bool         fresh         = false;
        bool         ever_received = false;
    };
    ThrottleCmd GetThrottleCmd(
        std::chrono::milliseconds freshness_window) const;

    /// Incoming washer pump command (ID 4081, chassis segment).
    /// Received from RHJB. true = pump active, false = idle.
    bool GetWasherPumpCommand() const;
    bool HasReceivedWasherPumpCommand() const;

    /// Incoming door lock commands from RSA (IDs 4084/4085, chassis segment).
    /// uint8: 0=unlocked, 1=locked.  Returns 0xFF if never received.
    /// side: 0=driver, 1=passenger.
    std::uint8_t GetDoorLockCmd(int side) const;
    bool         HasReceivedDoorLockCmd(int side) const;

    /// Incoming power window motor commands from RSA (IDs 4086/4087, chassis segment).
    /// uint8: 0=stop, 1=up, 2=down.  Returns 0xFF if never received.
    /// side: 0=driver, 1=passenger.
    std::uint8_t GetPowerWindowMotor(int side) const;
    bool         HasReceivedPowerWindowMotor(int side) const;

    /// Incoming HVAC blower level from HTCM (ID 4082, chassis segment).
    /// uint8: 0=OFF, 1=LOW, 2=MED, 3=HIGH.  Returns 0xFF if never received.
    std::uint8_t GetHvacBlowerLevel() const;
    bool         HasReceivedHvacBlowerLevel() const;

    /// Incoming defrost grid active flag from HTCM (ID 4083, chassis segment).
    /// bool: true = rear defrost grid energised.  Returns false if never received.
    bool GetDefrostGridActive() const;
    bool HasReceivedDefrostGridActive() const;

    /// Incoming RSA shift-blocked cue (ID 4088, chassis segment).
    /// Published by RSA when a P→non-P shift is refused (brake switch not pressed).
    /// Level signal: true while the driver holds a non-PARK request without brake.
    /// bool: true = shift blocked, false = no block.  Returns false if never received.
    bool GetRsaShiftBlocked() const;
    bool HasReceivedRsaShiftBlocked() const;

    /// Incoming door-lock motor leg drives from RHJB (IDs 4092-4095, chassis segment).
    /// RHJB's dual-H-bridge drives both door-lock motors in lockstep; each motor
    /// has a LOCK leg and an UNLOCK leg.  leg index: 0=LH lock, 1=LH unlock,
    /// 2=RH lock, 3=RH unlock.  bool: true = leg energised.  Out-of-range → false.
    /// Consumed by PhysicalWorld::DoorLockMotor (×2).
    bool GetDoorLockMotorDrive(int leg) const;
    bool HasReceivedDoorLockMotorDrive(int leg) const;

    /// Incoming sounder / piezo drive from the LHJB flasher (ID 4096, chassis segment).
    /// bool: true = piezo energised (the TURN/HAZ "click").  Returns false if
    /// never received.  Consumed by PhysicalWorld::Sounder.
    bool GetSounderPiezoDrive() const;
    bool HasReceivedSounderPiezoDrive() const;

    /// Incoming power-steering pump speed command from PSCM (ID 4097, chassis segment).
    /// uint8 q8: 0=stopped, 255=full.  Returns 0xFF if never received.
    /// Consumed by PhysicalWorld::PowerSteeringPumpMotor.
    std::uint8_t GetSteeringPumpSpeedCmdQ8() const;
    bool         HasReceivedSteeringPumpSpeedCmd() const;

    /// Outgoing power-steering pump HV interlock-closed (ID 4098, chassis segment).
    /// The pump motor body closes the molex.D/E HV interlock loop while present;
    /// PSCM senses this as interlock-closed.  Published on change; defaults true
    /// (motor present).  Set false for fault injection (motor unplugged).
    void SetSteeringPumpInterlockClosed(bool closed);

    /// Incoming IPC seatbelt telltale — driver seat (ID 4130, chassis segment).
    /// Published by IPC when the driver seat is unbuckled AND speed > ~8 km/h.
    /// bool: true = lamp on (unbuckled at speed), false = lamp off.
    /// Returns false if never received.
    bool GetIpcSeatbeltTelltaleDriver() const;
    bool HasReceivedIpcSeatbeltTelltaleDriver() const;

    /// Incoming IPC seatbelt telltale — passenger seat (ID 4131, chassis segment).
    /// Published by IPC when the passenger seat is unbuckled AND speed > ~8 km/h.
    /// bool: true = lamp on (unbuckled at speed), false = lamp off.
    /// Returns false if never received.
    bool GetIpcSeatbeltTelltalePassenger() const;
    bool HasReceivedIpcSeatbeltTelltalePassenger() const;

    /// Incoming IPC trip distance (ID 4132, chassis segment).
    /// Published by IPC controller on change (epsilon ~0.5 m) after each supervisor tick.
    /// float32 LE, metres.  Resets to 0.0 on trip-reset button press (kSigDriverIpcTripResetButton).
    /// Returns -1.0f (sentinel) if never received.
    float GetIpcTripDistanceM() const;
    bool  HasReceivedIpcTripDistance() const;

    /// Incoming IPC brake telltale (ID 4134, chassis segment).
    /// Published by IPC when the BTCM reports brake_ind (DTC 42).
    /// bool: true = lamp on, false = lamp off.  Returns false if never received.
    bool GetIpcBrakeTelltale() const;
    bool HasReceivedIpcBrakeTelltale() const;

    /// Incoming IPC park brake telltale (ID 4135, chassis segment).
    /// Published by IPC when the BTCM reports park_brake_ind (DTC 44).
    /// bool: true = lamp on, false = lamp off.  Returns false if never received.
    bool GetIpcParkBrakeTelltale() const;
    bool HasReceivedIpcParkBrakeTelltale() const;

    /// Incoming IPC antilock telltale (ID 4136, chassis segment).
    /// Published by IPC when the BTCM reports antilock_ind (DTC 41).
    /// bool: true = lamp on, false = lamp off.  Returns false if never received.
    bool GetIpcAntilockTelltale() const;
    bool HasReceivedIpcAntilockTelltale() const;

    /// Incoming IPC low-traction telltale (ID 4137, chassis segment).
    /// Published by IPC when the BTCM reports low_trac_ind (DTC 43).
    /// bool: true = lamp on, false = lamp off.  Returns false if never received.
    bool GetIpcLowTracTelltale() const;
    bool HasReceivedIpcLowTracTelltale() const;

    /// Incoming IPC airbag telltale (ID 4138, chassis segment).
    /// Published by IPC when ipc_supervisor_set_airbag_input(true) is active (DTC 40).
    /// bool: true = lamp on, false = lamp off.  Returns false if never received.
    bool GetIpcAirBagTelltale() const;
    bool HasReceivedIpcAirBagTelltale() const;

    /// Incoming IPC service-now telltale (ID 4140, chassis segment).
    /// Published by IPC when SERVICE_NOW DTC is active (DTCs 31/33/35 via HTCM/PCM/BPM).
    /// bool: true = lamp on, false = lamp off.  Returns false if never received.
    bool GetIpcServiceNowTelltale() const;
    bool HasReceivedIpcServiceNowTelltale() const;

    /// Incoming IPC check-messages telltale (ID 4141, chassis segment).
    /// Aggregate: any comm-loss DTC, any telltale-request DTC, or any BTCM indicator DTC.
    /// bool: true = lamp on, false = lamp off.  Returns false if never received.
    bool GetIpcCheckMessagesTelltale() const;
    bool HasReceivedIpcCheckMessagesTelltale() const;

    /// Incoming IPC temp telltale (ID 4142, chassis segment).
    /// Published by IPC when TEMP DTC is active (DTCs 32/34/36 via HTCM/PCM/BPM).
    /// bool: true = lamp on, false = lamp off.  Returns false if never received.
    bool GetIpcTempTelltale() const;
    bool HasReceivedIpcTempTelltale() const;

    /// Incoming IPC battery-life telltale (ID 4143, chassis segment).
    /// Published by IPC when DTC 38 is active via IPC_REQ_BATTERY_LIFE_FROM_BPM.
    /// bool: true = lamp on, false = lamp off.  Returns false if never received.
    bool GetIpcBatteryLifeTelltale() const;
    bool HasReceivedIpcBatteryLifeTelltale() const;

    /// Incoming IPC reduced-performance telltale (ID 4144, chassis segment).
    /// Published by IPC when DTC 37 is active via IPC_REQ_REDUCED_PERF_FROM_PCM.
    /// bool: true = lamp on, false = lamp off.  Returns false if never received.
    bool GetIpcReducedPerfTelltale() const;
    bool HasReceivedIpcReducedPerfTelltale() const;

    /// Incoming IPC check-tire-pressure telltale (ID 4145, chassis segment).
    /// Published by IPC when DTC 39 is active via IPC_REQ_CHECK_TIRE_PRESS_FROM_RSA.
    /// bool: true = lamp on, false = lamp off.  Returns false if never received.
    bool GetIpcCheckTirePressTelltale() const;
    bool HasReceivedIpcCheckTirePressTelltale() const;

    /// Incoming PIM cruise-control active flag (ID 5860, main harness segment).
    /// Published by PIM each tick on change.
    /// bool: true = cruise ACTIVE (engaged), false = STANDBY or OFF.
    /// Returns false if never received.
    bool GetPimCruiseActive() const;
    bool HasReceivedPimCruiseActive() const;

    /// Incoming PIM cruise-control setpoint (ID 5861, main harness segment).
    /// float32 LE, target speed in m/s.  Non-zero while ACTIVE or STANDBY;
    /// zero only when state == OFF (setpoint fully cleared).
    /// Returns 0.0f if never received.
    float GetPimCruiseSetpointMps() const;
    bool  HasReceivedPimCruiseSetpointMps() const;

    /// Incoming BPM pack voltage (ID 4139, chassis segment).
    /// uint32 LE, millivolts (mV).  BPM publishes on change (epsilon ~50 mV)
    /// while key-on.  Range: 0..~360 000 mV (0..360 V nominal).
    /// Returns 0 if never received.
    std::uint32_t GetBpmPackVoltageMv() const;
    bool          HasReceivedBpmPackVoltage() const;

    /// Current vehicle speed from the ev1sim physics model (m/s).
    /// Derived from the VehicleState snapshot set each tick via SetVehicleState().
    /// Returns -1.0f if SetVehicleState() has never been called.
    /// This is ev1sim's own output (ID 4100); no bus subscription required.
    float GetVehicleSpeedMps() const;
    bool  HasVehicleSpeed() const;

    // ---------------------------------------------------------------------
    // Test hooks — feed the connector synthetic delta records as though they
    // arrived over the bus.  Used by unit tests; not part of the runtime path.
    // ---------------------------------------------------------------------
    void DebugInjectDelta(std::uint32_t signal_id, bool value);

    /// Inject a uint8 signal value (e.g. wiper motor command).
    /// Used by unit tests; not part of the runtime path.
    void DebugInjectU8(std::uint32_t signal_id, std::uint8_t value);

    /// Inject a float signal value (rear EMB motor commands, cruise setpoint, etc.).
    /// Updates the corresponding state field + timestamp so freshness-window
    /// logic works correctly.  Used by unit tests.
    void DebugInjectFloat(std::uint32_t signal_id, float value);

    /// Inject a uint32 signal value (BPM pack voltage, etc.).
    /// Used by unit tests; not part of the runtime path.
    void DebugInjectU32(std::uint32_t signal_id, std::uint32_t value);

private:
    Options m_opts;

    struct State;
    std::unique_ptr<State> m_state;
};
