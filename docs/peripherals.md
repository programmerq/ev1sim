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

> **Signal-ID allocation.**  Most IDs below were allocated **ev1sim-side first**
> so electricsim's controllers have a fixed wire contract to publish against.
> Those still awaiting a `electricsim::io::kSigChassis*` counterpart are
> intentionally **absent from the compile-time drift guard** in
> `ExternalSimConnector.cpp` (adding a `static_assert` for an ID electricsim
> doesn't define would break the integrated build).  When electricsim adds the
> canonical constants, move each into the guard — see the "pending electricsim
> adoption" block in that file for the suggested canonical names.
>
> **An ID that has been adopted must move into the guard.**  The four door-lock
> motor legs sat on their ev1sim-side allocation (4092-4095) after electricsim
> adopted the same four wires at 4182-4185; the wire-truth consumer overlay
> delivered the adopted IDs, this connector still dispatched on the retired
> ones, and every leg the RHJB published was dropped without a word.  Those four
> are guarded now.  The guard is the only thing that makes an allocation and its
> adoption fail loudly instead of quietly.

These peripherals' chassis IDs live in the body/actuator neighbourhood of the
`4000..4199` chassis-bus block (see [ARCHITECTURE.md](../ARCHITECTURE.md)).

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
| LH motor **LOCK**       | 294A    | J9.C5       | **4182**              | `vehicle.body.door_lock_motor.lh_lock_drive`   | RHJB → ev1sim |
| LH motor **UNLOCK**     | 295A    | J9.C6       | **4183**              | `vehicle.body.door_lock_motor.lh_unlock_drive` | RHJB → ev1sim |
| RH motor **LOCK**       | 294C    | J3.A6       | **4184**              | `vehicle.body.door_lock_motor.rh_lock_drive`   | RHJB → ev1sim |
| RH motor **UNLOCK**     | 295C    | J3.A7       | **4185**              | `vehicle.body.door_lock_motor.rh_unlock_drive` | RHJB → ev1sim |

Wire level: `uint8`/bool, `1` = leg energised.  These are electricsim's own
`kSigChassisRhjbDlm*` IDs (chassis contract 1.9.0), pinned by `static_assert` in
`ExternalSimConnector.cpp`.  The retired ev1sim-side allocation was 4092-4095.

Connector accessors: `GetDoorLockMotorDrive(leg)` / `HasReceivedDoorLockMotorDrive(leg)`
where `leg` ∈ {0=LH lock, 1=LH unlock, 2=RH lock, 3=RH unlock}.

### Scenario CSV columns, and which layer each reports

Three layers, three column families. Which one a rule should score depends on
what that rule is auditing.

| Column | Reports | Source |
|---|---|---|
| `door_lock_motor_{lh,rh}_{lock,unlock}` | the **relay leg**, exactly as published | chassis 4182-4185 |
| `door_lock_motor_*_drive` | the same value, older spelling | chassis 4182-4185 |
| `door_lock_winding_{lh,rh}_{lock,unlock}` | the **winding**: its leg driven **and** the opposite leg not | `DoorLockMotor` |
| `door_lock_stroke_{lh,rh}` | pawl travel, `0.0` unlocked stop → `1.0` locked stop | `DoorLockMotor` |
| `door_lock_motor_{lh,rh}_stalled` | winding live with the pawl already at that stop | `DoorLockMotor` |
| `door_lock_state_{driver,passenger}` | the latched result, the same value published on 4165/4166 | `DoorLocks` |
| `door_lock_switch_{lh,rh}_{lock,unlock}` | the rocker contact ev1sim published | chassis 4170-4173 |

**A rule auditing the relays must score the LEG.** The upstream interlock rule
is "a LOCK press must never energise the UNLOCK leg", and its whole purpose is
catching both relays closed at once. The winding column reads `0` throughout a
lock pulse whatever the junction block does with the unlock relay — closing both
relays parks both brushes on one rail — so scoring the winding there would make
that rule unable to fail on the fault it names. The winding is the honest input
to the plant and the honest thing to model; it is the wrong answer to hand a
rule that is auditing relays.

---

## `door_lock_switch` — the door-mounted lock/unlock rockers (LH + RH)

The other end of the same loop, and the reason the motors ever move.  Each door
carries a momentary rocker with a LOCK contact and an UNLOCK contact to ground;
electricsim's `rhjb_door_lock` ORs the LH and RH inputs per direction and fires a
**one-shot on the rising edge**, so a rocker held closed never re-pulses.
`PhysicalWorld::door_lock_switch_lh()` / `_rh()` model that: a press closes one
contact for `press_time_s` (default 0.25 s) and then opens it, and always
survives at least one `update()` so a coarse tick cannot step over a whole press.

ev1sim is the **only** producer of these four cells.  Without them the lock
module sees no edge, drives no leg, and every downstream door-lock observable
reads a flat zero that looks exactly like a healthy quiescent car.

### Wire-level mapping

| Switch contact | Circuit | RHJB cavity | Chassis-bus signal ID | Endpoint (qualified name) | Dir |
|----------------|---------|-------------|-----------------------|---------------------------|-----|
| LH **LOCK**    | 780A    | J9.C13      | **4170**              | `vehicle.body.door_lock_switch.lh_lock_out`   | ev1sim → RHJB |
| LH **UNLOCK**  | 781A    | J9.D16      | **4171**              | `vehicle.body.door_lock_switch.lh_unlock_out` | ev1sim → RHJB |
| RH **LOCK**    | 780C    | J3.A2       | **4172**              | `vehicle.body.door_lock_switch.rh_lock_out`   | ev1sim → RHJB |
| RH **UNLOCK**  | 781C    | J3.A1       | **4173**              | `vehicle.body.door_lock_switch.rh_unlock_out` | ev1sim → RHJB |

Wire level: bool, `1` = contact closed.  Published on change, with the first
publish forced so the lock module's edge detector seeds from a defined all-open
state.  These are electricsim's `kSigDoorLockSw_*` IDs and are `static_assert`-pinned.

Connector accessor: `SetDoorLockSwitchContacts(lh_lock, lh_unlock, rh_lock, rh_unlock)`.
Headless scenarios reach it through the `door_lock_switch` action (see
[`src/Scenario.h`](../src/Scenario.h)).

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

## Signal-ID summary

| ID   | Peripheral                  | Encoding   | Direction      |
|------|-----------------------------|------------|----------------|
| 4096 | sounder piezo drive         | bool       | LHJB → ev1sim  |
| 4097 | power_steering_pump speed   | uint8 q8   | PSCM → ev1sim  |
| 4098 | power_steering_pump interlock | bool     | ev1sim → PSCM  |
| 4170 | door_lock_switch LH lock    | bool       | ev1sim → RHJB  |
| 4171 | door_lock_switch LH unlock  | bool       | ev1sim → RHJB  |
| 4172 | door_lock_switch RH lock    | bool       | ev1sim → RHJB  |
| 4173 | door_lock_switch RH unlock  | bool       | ev1sim → RHJB  |
| 4182 | door_lock_motor LH lock     | bool       | RHJB → ev1sim  |
| 4183 | door_lock_motor LH unlock   | bool       | RHJB → ev1sim  |
| 4184 | door_lock_motor RH lock     | bool       | RHJB → ev1sim  |
| 4185 | door_lock_motor RH unlock   | bool       | RHJB → ev1sim  |

(4092-4095 are the retired ev1sim-side door-lock-motor allocation; electricsim
adopted the same four wires at 4182-4185.)
