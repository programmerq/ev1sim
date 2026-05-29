# ev1sim peripheral specs

Authoritative catalog of the **actuator / plant peripherals** ev1sim models on
the far side of the electricsim outputs, plus their **wire-level mapping**
(harness cavity → chassis-bus signal ID).  electricsim's routers/controllers
treat this file as the reference for which chassis-bus ID a given module output
publishes onto.

These peripherals are modelled as plain, Chrono-free state machines in
[`src/PhysicalWorld.h`](../src/PhysicalWorld.h) and are reachable through the
`PhysicalWorld` container.  The chassis-bus contract (signal IDs, encodings,
endpoint directions, decode) lives in
[`src/ExternalSimConnector.cpp`](../src/ExternalSimConnector.cpp).

> **Signal-ID allocation.**  All IDs below were allocated **ev1sim-side first**
> so electricsim's controllers have a fixed wire contract to publish against.
> They have no `electricsim::io::kSigChassis*` counterpart yet, so they are
> intentionally **absent from the compile-time drift guard** in
> `ExternalSimConnector.cpp` (adding a `static_assert` for an ID electricsim
> doesn't define would break the integrated build).  When electricsim adds the
> canonical constants, move each into the guard — see the "pending electricsim
> adoption" block in that file for the suggested canonical names.

All three peripherals' chassis IDs live in the body/actuator neighbourhood of
the `4000..4199` chassis-bus block (see [ARCHITECTURE.md](../ARCHITECTURE.md)).
Consumer wiring into the live `SimApp` tick + 3D render/audio is **deferred**
(tracked in [TODO.md](TODO.md)); the spec + signal IDs are enough for
electricsim to start publishing.

---

## `door_lock_motor` — RHJB dual-H-bridge door-lock motors (LH + RH)

electricsim's `rhjb_door_lock` drives **both** door-lock motors in lockstep
from a 4-output dual-H-bridge.  ev1sim instantiates the peripheral **twice**
(`PhysicalWorld::door_lock_motor_lh()` = driver door,
`door_lock_motor_rh()` = passenger door).  Each instance consumes a LOCK leg
and an UNLOCK leg and exposes a mechanical lock stroke
(`UNLOCKED` / `MID_STROKE` / `LOCKED`).

* The motor runs in the direction whose drive is active and stops at the
  end-of-travel limit.
* Both drives high (illegal H-bridge shoot-through) **or** both low → motor off.
* Default `traverse_time_s = 0.5 s` (typical GM door-lock motor stroke
  ~0.4–0.6 s).  This sits inside the RHJB DLM's default **600 ms** drive pulse,
  so the latch reaches its end-of-travel within a single pulse.

### Wire-level mapping

| Motor leg (RHJB output) | Circuit | RHJB cavity | Chassis-bus signal ID | Endpoint (qualified name) | Dir |
|-------------------------|---------|-------------|-----------------------|---------------------------|-----|
| LH motor **LOCK**       | 294A    | J9.C5       | **4092**              | `vehicle.body.door_lock_motor.lh_lock_drive`   | RHJB → ev1sim |
| LH motor **UNLOCK**     | 295A    | J9.C6       | **4093**              | `vehicle.body.door_lock_motor.lh_unlock_drive` | RHJB → ev1sim |
| RH motor **LOCK**       | 294C    | J3.A6       | **4094**              | `vehicle.body.door_lock_motor.rh_lock_drive`   | RHJB → ev1sim |
| RH motor **UNLOCK**     | 295C    | J3.A7       | **4095**              | `vehicle.body.door_lock_motor.rh_unlock_drive` | RHJB → ev1sim |

Wire level: `uint8`/bool, `1` = leg energised.

Connector accessors: `GetDoorLockMotorDrive(leg)` / `HasReceivedDoorLockMotorDrive(leg)`
where `leg` ∈ {0=LH lock, 1=LH unlock, 2=RH lock, 3=RH unlock}.

---

## `sounder` — LHJB flasher piezo "click"

electricsim's `lhjb_flasher` produces a piezo square-wave (the TURN/HAZ "click"
you hear in a real GM vehicle).  ev1sim's `PhysicalWorld::sounder()` consumes
the boolean drive and exposes an audible-output signal for the 3D-sim audio
contract: `sounding()` is high while the piezo is energised, and each rising
edge increments `click_count()` (one click per flash half-cycle).  No frequency
model — the piezo is a fixed-pitch element.

### Wire-level mapping

| Source                       | Chassis-bus signal ID | Endpoint (qualified name)         | Dir |
|------------------------------|-----------------------|-----------------------------------|-----|
| LHJB flasher piezo output    | **4096**              | `vehicle.body.sounder.piezo_drive` | LHJB → ev1sim |

Wire level: `uint8`/bool, `1` = piezo energised.
Connector accessors: `GetSounderPiezoDrive()` / `HasReceivedSounderPiezoDrive()`.

The LHJB-internal piezo is a real component that the printed EV1 schematics are
silent on by design (no first-class `component_id`).  If electricsim's redux
side wants to first-class it, a `module_lhjb_piezo` entry in
`manual_supplements.yaml` (electricsim's evidence channel) is the natural home —
but ev1sim does not require it.

---

## `power_steering_pump_motor` — PSCM-driven steering pump

The PSCM is drawn as an HV inverter on `batt-731`; its molex 3-phase outputs
(`molex.A/B/C`) drive a steering pump motor, and the motor body returns the HV
interlock loop on `molex.D/E`.  ev1sim's `PhysicalWorld::power_steering_pump()`
is the **minimum-viable plant**: it consumes a single commanded pump speed and
tracks an actual speed with a first-order spin-up/spin-down lag
(`response_tau_s`, default 0.15 s).  Real per-phase BLDC commutation is
intentionally ignored — phase-level modelling is future work.

The motor body closes the HV interlock loop while present; `interlock_closed()`
is what the PSCM senses.  `set_present(false)` opens the loop for fault
injection.

### Wire-level mapping

| Connection                         | Chassis-bus signal ID | Endpoint (qualified name)                       | Dir |
|------------------------------------|-----------------------|-------------------------------------------------|-----|
| PSCM `molex.A/B/C` → pump phases   | **4097**              | `vehicle.steering.pump_motor.speed_cmd_q8`      | PSCM → ev1sim |
| pump `molex.D/E` HV interlock loop | **4098**              | `vehicle.steering.pump_motor.interlock_closed`  | ev1sim → PSCM |

Encodings: `4097` is `uint8` q8 (`0` = stopped, `255` = full); `4098` is
`uint8`/bool (`1` = loop closed / motor present).
Connector accessors: `GetSteeringPumpSpeedCmdQ8()` / `HasReceivedSteeringPumpSpeedCmd()`
(in) and `SetSteeringPumpInterlockClosed(bool)` (out, published on change).

This unlocks PSCM's deferred plant model in electricsim: PSCM receives its
`pump_engaged` command, publishes `4097`, and reads `4098` as interlock-closed.

---

## Signal-ID summary (this round)

| ID   | Peripheral                  | Encoding   | Direction      |
|------|-----------------------------|------------|----------------|
| 4092 | door_lock_motor LH lock     | bool       | RHJB → ev1sim  |
| 4093 | door_lock_motor LH unlock   | bool       | RHJB → ev1sim  |
| 4094 | door_lock_motor RH lock     | bool       | RHJB → ev1sim  |
| 4095 | door_lock_motor RH unlock   | bool       | RHJB → ev1sim  |
| 4096 | sounder piezo drive         | bool       | LHJB → ev1sim  |
| 4097 | power_steering_pump speed   | uint8 q8   | PSCM → ev1sim  |
| 4098 | power_steering_pump interlock | bool     | ev1sim → PSCM  |
