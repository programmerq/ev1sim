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

## `sounder` — turn-signal and hazard tick

The tick you hear with a turn signal or the hazard on comes from a piezo
sounder, X56, soldered on the LHJB's own board. The LHJB's TURN/HAZ/DRL module
drives it for the ON half of each flash. electricsim models it as a real
circuit and its solver publishes the sounder as a load on the
`TURN_TICK_SOUNDER_DRIVE` conductor (wire id 1741; electricsim
`docs/3d_sim_contract.md` section 2.6). Like every two-sided load it reads 0 when
the LHJB, its B+ or the sounder's lead is lost.

ev1sim's `PhysicalWorld::sounder()` turns that drive into cabin ticks:
`update(drive, sim_time_s)` counts one tick per rising edge and stamps it with
the sim time, `take_pending_ticks()` hands the ticks to the audio backend, and
`last_tick_period_s()` is the flash period in sim time. SimApp plays one short
piezo burst (`CabinAudio::PlayTick()`, about 12 ms, a by-ear estimate) per
tick. The flash timing comes from electricsim's sim clock, so a paused sim
holds the flash phase and plays nothing. The IPC's extended-turn-signal chime
is a different sound, the cluster's own.

### Wire-level mapping

| Source                         | Wire (electricsim)                     | ev1sim input id | Endpoint (qualified name)          | Dir |
|--------------------------------|----------------------------------------|-----------------|------------------------------------|-----|
| LHJB tick sounder X56 (a load) | `TURN_TICK_SOUNDER_DRIVE` (1741, bit)  | **4096**        | `vehicle.body.sounder.piezo_drive` | LHJB → ev1sim |

`WireTruthChassis`'s consumer registry maps the conductor onto 4096.
Connector accessors: `GetSounderPiezoDrive()` / `HasReceivedSounderPiezoDrive()`.
The headless scenario CSV column is `sounder_piezo_drive`.

---

## `power_steering_pump_motor` — PSCM-driven steering pump

The PSCM is drawn as an HV inverter on `batt-731`; its molex 3-phase outputs
(`molex.A/B/C`) drive a steering pump motor, and the motor body returns the HV
interlock loop on `molex.D/E`.  ev1sim's `PhysicalWorld::power_steering_pump()`
is a **per-phase BLDC model**:

- The commanded speed is a three-phase output frequency, up to the printed
  83 Hz ceiling.  The inverter ramps its reference toward it, commutates on the
  rotor angle and regulates speed through cascaded speed and current loops,
  limited to elec-363's 180 VAC output.
- The motor is integrated in the rotor (d/q) frame on a 20 µs sub-step and
  projected onto phases A/B/C: `phase_current_a(n)`, `phase_voltage_v(n)`,
  `electrical_angle_rad()`, plus torque, shaft speed and input power.
- The pump load is a rest torque plus a pressure torque that builds as the
  wheel nears full lock (`hydraulic_load_from_steering`, elec-363's LOW and
  HIGH LOAD lines).  At 82 Hz the drawn power is ~80 W at rest and ~825 W
  held at a stop, inside the printed envelope.
- Copper and friction losses and a share of the pump's churning heat a
  one-node case model that cools toward the ambient sensor's temperature;
  a sustained full-lock hold reaches the PSCM's 150 °C switch in about ten
  minutes.

The manual prints the drive envelope, not the motor, so every motor, pump and
loop constant in `PowerSteeringPumpMotor::Config` is `@inferred`; the thermal
constants follow electricsim's PSCM plant.

The motor body closes the HV interlock loop while present; `interlock_closed()`
is what the PSCM senses.  `set_present(false)` opens the loop for fault
injection and disconnects the phases, so the rotor coasts.

### Wire-level mapping

| Connection                         | Chassis-bus signal ID | Endpoint (qualified name)                       | Dir |
|------------------------------------|-----------------------|-------------------------------------------------|-----|
| PSCM `molex.A/B/C` → pump phases   | **4097**              | `vehicle.steering.pump_motor.speed_cmd_q8`      | PSCM → ev1sim |
| pump `molex.D/E` HV interlock loop | **4098**              | `vehicle.steering.pump_motor.interlock_closed`  | ev1sim → PSCM |
| pump case sense (circuit 900A)     | wire cell             | `CHASSIS_PS_MOTOR_CASE_TEMP_Q8`                 | ev1sim → PSCM |

Encodings: `4097` is `uint8` q8 (`0` = stopped, `255` = full); `4098` is
`uint8`/bool (`1` = loop closed / motor present); the case temperature is
`uint16` Q8 °C.
Connector accessors: `GetSteeringPumpSpeedCmdQ8()` / `HasReceivedSteeringPumpSpeedCmd()`
(in), `SetSteeringPumpInterlockClosed(bool)` (out, published on change) and
`SetSteeringPumpCaseTempQ8(uint16)` (out, every tick once set).  ev1sim writes
the case temperature only after the first pump command arrives: unwritten
tells the PSCM this fleet has no pump peripheral, so its own thermal model
stays the source.

electricsim's PSCM does not publish `4097` or read `4098` yet: neither has a
cell in its topology (both are on the connector's pending-adoption list), and
the PSCM drives its own pump plant.  Until it publishes its inverter drive,
ev1sim's pump stays still in a co-simulated fleet and the case-temperature cell
stays unwritten.

---

## `power_window_motor` — window glass (LH + RH)

Each door's window glass rides a reversible motor across two legs that the
LHJB (driver) and RHJB (passenger) switch to B+ through their window relays.
electricsim publishes the legs as conductors and `WireTruthChassis::window_motor_cmd`
reads them as one command per side: `0` stopped, `1` UP leg alone, `2` DOWN leg
alone. Both legs live, or both dark, is stopped: the motor turns only while the
legs differ.

| Side | UP leg | DOWN leg |
|---|---|---|
| LH / driver (LHJB) | `CHASSIS_LH_WINDOW_UP_DRIVE` (1513) | `CHASSIS_LH_WINDOW_DOWN_DRIVE` (1514) |
| RH / passenger (RHJB) | `CHASSIS_RH_WINDOW_UP_DRIVE` | `CHASSIS_RH_WINDOW_DOWN_DRIVE` |

`PhysicalWorld::StepPowerWindowPlant()` advances `PowerWindowMotor` per side
from those commands, every tick in both loops: the glass runs between its
closed and fully-open stops at a fixed rate and stalls against a stop while
still driven. The full travel time, 4 s, is an **estimate** (a typical car
window); no EV1 figure is in hand. The body diagram draws each window as a bar
inside its door, full length when closed, amber while moving.

Connector accessors: `GetPowerWindowMotor(side)` / `HasReceivedPowerWindowMotor(side)`.

### Locks by hand

The floating UI panel's lock rows, the `door_lock_all` / `door_unlock_all`
scenario actions and `body.door_locks_locked_at_start` set a lock the way the
knob inside the door does (`PhysicalWorld::SetLockByHand`). A door's knob is
linked to its lock actuator, so the door-lock motor's stroke moves with it, and
the next motor pulse from the RHJB starts from where the knob left it.

---

## Signal-ID summary

| ID   | Peripheral                  | Encoding   | Direction      |
|------|-----------------------------|------------|----------------|
| 4096 | turn/hazard tick sounder drive (wire 1741) | bool | LHJB → ev1sim  |
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
