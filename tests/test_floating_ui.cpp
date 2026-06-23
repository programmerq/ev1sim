#include <catch2/catch_test_macros.hpp>

// FloatingUiPanel pure label helpers — testable without Irrlicht window.
#include "FloatingUiPanel.h"
#include "PhysicalWorld.h"
#include "KeyboardInputController.h"

#include <limits>

// ---------------------------------------------------------------------------
// Label format tests (pure, no Irrlicht)
// ---------------------------------------------------------------------------

TEST_CASE("FormatHazardLabel: OFF when hazard not active", "[FloatingUI]") {
    CHECK(FormatHazardLabel(false) == L"Hazard: OFF");
}

TEST_CASE("FormatHazardLabel: ON when hazard active", "[FloatingUI]") {
    CHECK(FormatHazardLabel(true) == L"Hazard: ON");
}

TEST_CASE("FormatLockAllLabel: 'Lock All' when none locked", "[FloatingUI]") {
    CHECK(FormatLockAllLabel(false) == L"Lock All");
}

TEST_CASE("FormatLockAllLabel: 'Unlock All' when any locked", "[FloatingUI]") {
    CHECK(FormatLockAllLabel(true) == L"Unlock All");
}

TEST_CASE("FormatDoorLabel: unlocked state", "[FloatingUI]") {
    CHECK(FormatDoorLabel(L"Driver",    false) == L"Driver: unlocked");
    CHECK(FormatDoorLabel(L"Passenger", false) == L"Passenger: unlocked");
    CHECK(FormatDoorLabel(L"Trunk",     false) == L"Trunk: unlocked");
}

TEST_CASE("FormatDoorLabel: locked state", "[FloatingUI]") {
    CHECK(FormatDoorLabel(L"Driver",    true) == L"Driver: locked");
    CHECK(FormatDoorLabel(L"Passenger", true) == L"Passenger: locked");
    CHECK(FormatDoorLabel(L"Trunk",     true) == L"Trunk: locked");
}

TEST_CASE("FormatCouplerLabel: UNPLUGGED when not present", "[FloatingUI]") {
    CHECK(FormatCouplerLabel(false) == L"Coupler: UNPLUGGED");
}

TEST_CASE("FormatCouplerLabel: PLUGGED when present", "[FloatingUI]") {
    CHECK(FormatCouplerLabel(true) == L"Coupler: PLUGGED");
}

TEST_CASE("FormatExtKeypadButtonLabel: all five buttons", "[FloatingUI]") {
    CHECK(FormatExtKeypadButtonLabel(0) == L"[1/2]");
    CHECK(FormatExtKeypadButtonLabel(1) == L"[3/4]");
    CHECK(FormatExtKeypadButtonLabel(2) == L"[5/6]");
    CHECK(FormatExtKeypadButtonLabel(3) == L"[7/8]");
    CHECK(FormatExtKeypadButtonLabel(4) == L"[9/0]");
}

TEST_CASE("FormatExtKeypadButtonLabel: out-of-range returns fallback", "[FloatingUI]") {
    CHECK(FormatExtKeypadButtonLabel(-1) == L"[?]");
    CHECK(FormatExtKeypadButtonLabel(5)  == L"[?]");
}

TEST_CASE("FormatDoorHandleLabel: driver and passenger", "[FloatingUI]") {
    CHECK(FormatDoorHandleLabel(L"Driver")    == L"Handle: Driver");
    CHECK(FormatDoorHandleLabel(L"Passenger") == L"Handle: Passenger");
}

// ---------------------------------------------------------------------------
// Callback invocation tests — use real PhysicalWorld objects
// ---------------------------------------------------------------------------

TEST_CASE("FloatingUI callback: hazard toggle inverts state", "[FloatingUI][Callbacks]") {
    ev1sim::PhysicalWorld world;
    REQUIRE_FALSE(world.hazard_switch().on());

    // Simulate the callback that SimApp would wire.
    auto cb = [&]() { world.hazard_switch().toggle(); };

    cb();
    CHECK(world.hazard_switch().on());
    CHECK(FormatHazardLabel(world.hazard_switch().on()) == L"Hazard: ON");

    cb();
    CHECK_FALSE(world.hazard_switch().on());
    CHECK(FormatHazardLabel(world.hazard_switch().on()) == L"Hazard: OFF");
}

TEST_CASE("FloatingUI callback: lock_all / unlock_all via label-driven logic",
          "[FloatingUI][Callbacks]") {
    ev1sim::PhysicalWorld world;
    // Starting state: all unlocked → label says "Lock All".
    REQUIRE_FALSE(world.door_locks().any_locked());
    CHECK(FormatLockAllLabel(world.door_locks().any_locked()) == L"Lock All");

    // Simulate the "Lock All / Unlock All" button callback.
    auto cb = [&]() {
        if (world.door_locks().any_locked())
            world.door_locks().unlock_all();
        else
            world.door_locks().lock_all();
    };

    cb();  // locks all
    CHECK(world.door_locks().any_locked());
    CHECK(FormatLockAllLabel(world.door_locks().any_locked()) == L"Unlock All");

    cb();  // unlocks all
    CHECK_FALSE(world.door_locks().any_locked());
    CHECK(FormatLockAllLabel(world.door_locks().any_locked()) == L"Lock All");
}

TEST_CASE("FloatingUI callback: toggle_driver changes only driver",
          "[FloatingUI][Callbacks]") {
    ev1sim::PhysicalWorld world;
    using S = ev1sim::DoorLocks::State;

    auto cb = [&]() { world.door_locks().toggle_driver(); };
    cb();
    CHECK(world.door_locks().driver()    == S::LOCKED);
    CHECK(world.door_locks().passenger() == S::UNLOCKED);
    CHECK(world.door_locks().trunk()     == S::UNLOCKED);
    CHECK(FormatDoorLabel(L"Driver", world.door_locks().driver() == S::LOCKED)
          == L"Driver: locked");
}

TEST_CASE("FloatingUI callback: charge coupler toggle flips present flag",
          "[FloatingUI][Callbacks]") {
    ev1sim::PhysicalWorld world;
    REQUIRE_FALSE(world.charge_coupler().present());

    auto cb = [&]() {
        world.charge_coupler().set_present(!world.charge_coupler().present());
    };

    cb();
    CHECK(world.charge_coupler().present());
    CHECK(FormatCouplerLabel(world.charge_coupler().present()) == L"Coupler: PLUGGED");

    cb();
    CHECK_FALSE(world.charge_coupler().present());
    CHECK(FormatCouplerLabel(world.charge_coupler().present()) == L"Coupler: UNPLUGGED");
}

// ---------------------------------------------------------------------------
// Wave 2 label helpers
// ---------------------------------------------------------------------------

TEST_CASE("FormatWiperLabel: all four positions", "[FloatingUI]") {
    CHECK(FormatWiperLabel(0) == L"Wiper: OFF");
    CHECK(FormatWiperLabel(1) == L"Wiper: INT");
    CHECK(FormatWiperLabel(2) == L"Wiper: LOW");
    CHECK(FormatWiperLabel(3) == L"Wiper: HIGH");
}

TEST_CASE("FormatWiperLabel: out-of-range returns fallback", "[FloatingUI]") {
    CHECK(FormatWiperLabel(-1) == L"Wiper: ?");
    CHECK(FormatWiperLabel(4)  == L"Wiper: ?");
}

TEST_CASE("FormatWashLabel: static label", "[FloatingUI]") {
    CHECK(FormatWashLabel() == L"Wash");
}

TEST_CASE("FormatCruiseLabel: action appended", "[FloatingUI]") {
    CHECK(FormatCruiseLabel(L"SET")    == L"Cruise: SET");
    CHECK(FormatCruiseLabel(L"RES")    == L"Cruise: RES");
    CHECK(FormatCruiseLabel(L"CANCEL") == L"Cruise: CANCEL");
    CHECK(FormatCruiseLabel(L"+")      == L"Cruise: +");
    CHECK(FormatCruiseLabel(L"-")      == L"Cruise: -");
}

TEST_CASE("FormatTripResetLabel: static label", "[FloatingUI]") {
    CHECK(FormatTripResetLabel() == L"Trip Reset");
}

TEST_CASE("FormatSeatbeltLabel: driver buckled / unbuckled", "[FloatingUI]") {
    CHECK(FormatSeatbeltLabel(L"D", true)  == L"Seatbelt D: BUCKLED");
    CHECK(FormatSeatbeltLabel(L"D", false) == L"Seatbelt D: UNBUCKLED");
}

TEST_CASE("FormatSeatbeltLabel: passenger buckled / unbuckled", "[FloatingUI]") {
    CHECK(FormatSeatbeltLabel(L"P", true)  == L"Seatbelt P: BUCKLED");
    CHECK(FormatSeatbeltLabel(L"P", false) == L"Seatbelt P: UNBUCKLED");
}

TEST_CASE("FloatingUI callback: wiper cycle updates label", "[FloatingUI][Callbacks]") {
    ev1sim::PhysicalWorld world;
    using P = ev1sim::WiperStalk::Position;
    REQUIRE(world.wiper_stalk().position() == P::OFF);

    auto cb = [&]() { world.wiper_stalk().cycle_position(); };

    cb();
    CHECK(world.wiper_stalk().position() == P::INT);
    CHECK(FormatWiperLabel(static_cast<int>(world.wiper_stalk().position()))
          == L"Wiper: INT");

    cb(); cb(); cb();  // LOW → HIGH → OFF
    CHECK(world.wiper_stalk().position() == P::OFF);
    CHECK(FormatWiperLabel(static_cast<int>(world.wiper_stalk().position()))
          == L"Wiper: OFF");
}

TEST_CASE("FloatingUI callback: seatbelt driver toggle", "[FloatingUI][Callbacks]") {
    ev1sim::PhysicalWorld world;
    REQUIRE(world.seatbelts().driver_buckled());

    auto cb = [&]() { world.seatbelts().toggle_driver(); };

    cb();
    CHECK_FALSE(world.seatbelts().driver_buckled());
    CHECK(FormatSeatbeltLabel(L"D", world.seatbelts().driver_buckled())
          == L"Seatbelt D: UNBUCKLED");

    cb();
    CHECK(world.seatbelts().driver_buckled());
    CHECK(FormatSeatbeltLabel(L"D", world.seatbelts().driver_buckled())
          == L"Seatbelt D: BUCKLED");
}

TEST_CASE("FloatingUI callback: seatbelt passenger toggle leaves driver unchanged",
          "[FloatingUI][Callbacks]") {
    ev1sim::PhysicalWorld world;
    REQUIRE(world.seatbelts().passenger_buckled());

    world.seatbelts().toggle_passenger();
    CHECK_FALSE(world.seatbelts().passenger_buckled());
    CHECK(world.seatbelts().driver_buckled());  // driver unaffected
}

// ---------------------------------------------------------------------------
// UI mode toggle state machine in KeyboardInputController
// ---------------------------------------------------------------------------

TEST_CASE("KeyboardInputController: TAB produces ConsumeUiModeToggle one-shot",
          "[FloatingUI][Keyboard]") {
    KeyboardInputController::Rates r;
    KeyboardInputController ctrl(r);

    // Initially no toggle pending.
    CHECK_FALSE(ctrl.ConsumeUiModeToggle());

    // Press TAB.
    ctrl.SetKeyPressed(irr::KEY_TAB, true);
    ctrl.Update(0.016);
    CHECK(ctrl.ConsumeUiModeToggle() == true);
    CHECK(ctrl.ConsumeUiModeToggle() == false);  // consumed

    // Release and re-press — fires again.
    ctrl.SetKeyPressed(irr::KEY_TAB, false);
    ctrl.Update(0.016);
    ctrl.SetKeyPressed(irr::KEY_TAB, true);
    ctrl.Update(0.016);
    CHECK(ctrl.ConsumeUiModeToggle() == true);
    CHECK(ctrl.ConsumeUiModeToggle() == false);
}

TEST_CASE("KeyboardInputController: holding TAB does not repeat toggle",
          "[FloatingUI][Keyboard]") {
    KeyboardInputController::Rates r;
    KeyboardInputController ctrl(r);

    ctrl.SetKeyPressed(irr::KEY_TAB, true);
    ctrl.Update(0.016);
    CHECK(ctrl.ConsumeUiModeToggle() == true);   // edge on first frame

    ctrl.Update(0.016);
    CHECK(ctrl.ConsumeUiModeToggle() == false);  // still held — no repeat

    ctrl.Update(0.016);
    CHECK(ctrl.ConsumeUiModeToggle() == false);
}

// ---------------------------------------------------------------------------
// HVAC status label helpers (FormatHvacBlowerLabel, FormatDefrostGridLabel)
// ---------------------------------------------------------------------------

TEST_CASE("FormatHvacBlowerLabel: all four HTCM levels", "[FloatingUI][HVAC]") {
    CHECK(FormatHvacBlowerLabel(0u) == L"Blower: OFF");
    CHECK(FormatHvacBlowerLabel(1u) == L"Blower: LOW");
    CHECK(FormatHvacBlowerLabel(2u) == L"Blower: MED");
    CHECK(FormatHvacBlowerLabel(3u) == L"Blower: HIGH");
}

TEST_CASE("FormatHvacBlowerLabel: never-received sentinel (0xFF) shows placeholder",
          "[FloatingUI][HVAC]") {
    CHECK(FormatHvacBlowerLabel(0xFFu) == L"Blower: ---");
}

TEST_CASE("FormatHvacBlowerLabel: unknown value shows fallback", "[FloatingUI][HVAC]") {
    CHECK(FormatHvacBlowerLabel(4u)   == L"Blower: ?");
    CHECK(FormatHvacBlowerLabel(200u) == L"Blower: ?");
}

TEST_CASE("FormatDefrostGridLabel: OFF when not active", "[FloatingUI][HVAC]") {
    CHECK(FormatDefrostGridLabel(false) == L"Defrost: OFF");
}

TEST_CASE("FormatDefrostGridLabel: ON when active", "[FloatingUI][HVAC]") {
    CHECK(FormatDefrostGridLabel(true) == L"Defrost: ON");
}

// ---------------------------------------------------------------------------
// IPC LCD telltale label helpers (FormatIpcSeatbeltTelltaleLabel)
// ---------------------------------------------------------------------------

TEST_CASE("FormatIpcSeatbeltTelltaleLabel: driver lamp off (received)",
          "[FloatingUI][IPC]") {
    CHECK(FormatIpcSeatbeltTelltaleLabel(L"D", false, /*ever_received=*/true)
          == L"Seatbelt (D): OFF");
}

TEST_CASE("FormatIpcSeatbeltTelltaleLabel: driver lamp on (received)",
          "[FloatingUI][IPC]") {
    CHECK(FormatIpcSeatbeltTelltaleLabel(L"D", true, /*ever_received=*/true)
          == L"Seatbelt (D): ON");
}

TEST_CASE("FormatIpcSeatbeltTelltaleLabel: driver never-received shows placeholder",
          "[FloatingUI][IPC]") {
    // lamp_on is ignored when ever_received is false.
    CHECK(FormatIpcSeatbeltTelltaleLabel(L"D", false, /*ever_received=*/false)
          == L"Seatbelt (D): ---");
    CHECK(FormatIpcSeatbeltTelltaleLabel(L"D", true, /*ever_received=*/false)
          == L"Seatbelt (D): ---");
}

TEST_CASE("FormatIpcSeatbeltTelltaleLabel: passenger lamp off (received)",
          "[FloatingUI][IPC]") {
    CHECK(FormatIpcSeatbeltTelltaleLabel(L"P", false, /*ever_received=*/true)
          == L"Seatbelt (P): OFF");
}

TEST_CASE("FormatIpcSeatbeltTelltaleLabel: passenger lamp on (received)",
          "[FloatingUI][IPC]") {
    CHECK(FormatIpcSeatbeltTelltaleLabel(L"P", true, /*ever_received=*/true)
          == L"Seatbelt (P): ON");
}

TEST_CASE("FormatIpcSeatbeltTelltaleLabel: passenger never-received shows placeholder",
          "[FloatingUI][IPC]") {
    CHECK(FormatIpcSeatbeltTelltaleLabel(L"P", false, /*ever_received=*/false)
          == L"Seatbelt (P): ---");
}

// ---------------------------------------------------------------------------
// IPC BTCM / airbag telltale label helper (FormatIpcTelltaleLampLabel)
// Chassis bus signals 4134–4138.
// ---------------------------------------------------------------------------

TEST_CASE("FormatIpcTelltaleLampLabel: lamp off (received)",
          "[FloatingUI][IPC]") {
    CHECK(FormatIpcTelltaleLampLabel(L"Brake", false, /*ever_received=*/true)
          == L"Brake: OFF");
}

TEST_CASE("FormatIpcTelltaleLampLabel: lamp on (received)",
          "[FloatingUI][IPC]") {
    CHECK(FormatIpcTelltaleLampLabel(L"Brake", true, /*ever_received=*/true)
          == L"Brake: ON");
}

TEST_CASE("FormatIpcTelltaleLampLabel: never-received shows placeholder",
          "[FloatingUI][IPC]") {
    // lamp_on is irrelevant when ever_received=false.
    CHECK(FormatIpcTelltaleLampLabel(L"Brake", false, /*ever_received=*/false)
          == L"Brake: ---");
    CHECK(FormatIpcTelltaleLampLabel(L"Brake", true, /*ever_received=*/false)
          == L"Brake: ---");
}

TEST_CASE("FormatIpcTelltaleLampLabel: ParkBrake name works",
          "[FloatingUI][IPC]") {
    CHECK(FormatIpcTelltaleLampLabel(L"ParkBrake", true,  true) == L"ParkBrake: ON");
    CHECK(FormatIpcTelltaleLampLabel(L"ParkBrake", false, true) == L"ParkBrake: OFF");
    CHECK(FormatIpcTelltaleLampLabel(L"ParkBrake", false, false) == L"ParkBrake: ---");
}

TEST_CASE("FormatIpcTelltaleLampLabel: ABS name works",
          "[FloatingUI][IPC]") {
    CHECK(FormatIpcTelltaleLampLabel(L"ABS", true,  true) == L"ABS: ON");
    CHECK(FormatIpcTelltaleLampLabel(L"ABS", false, true) == L"ABS: OFF");
    CHECK(FormatIpcTelltaleLampLabel(L"ABS", false, false) == L"ABS: ---");
}

TEST_CASE("FormatIpcTelltaleLampLabel: LowTrac name works",
          "[FloatingUI][IPC]") {
    CHECK(FormatIpcTelltaleLampLabel(L"LowTrac", true,  true) == L"LowTrac: ON");
    CHECK(FormatIpcTelltaleLampLabel(L"LowTrac", false, true) == L"LowTrac: OFF");
    CHECK(FormatIpcTelltaleLampLabel(L"LowTrac", false, false) == L"LowTrac: ---");
}

TEST_CASE("FormatIpcTelltaleLampLabel: AirBag name works",
          "[FloatingUI][IPC]") {
    CHECK(FormatIpcTelltaleLampLabel(L"AirBag", true,  true) == L"AirBag: ON");
    CHECK(FormatIpcTelltaleLampLabel(L"AirBag", false, true) == L"AirBag: OFF");
    CHECK(FormatIpcTelltaleLampLabel(L"AirBag", false, false) == L"AirBag: ---");
}

// ---------------------------------------------------------------------------
// RSA shift-blocked cue label helper (FormatRsaShiftBlockedLabel)
// Chassis bus signal 4088; 0=not blocked, 1=shift blocked this tick.
// ---------------------------------------------------------------------------

TEST_CASE("FormatRsaShiftBlockedLabel: never-received shows placeholder",
          "[FloatingUI][RSA]") {
    // blocked value is irrelevant when ever_received=false.
    CHECK(FormatRsaShiftBlockedLabel(false, /*ever_received=*/false) == L"Shift: ---");
    CHECK(FormatRsaShiftBlockedLabel(true,  /*ever_received=*/false) == L"Shift: ---");
}

TEST_CASE("FormatRsaShiftBlockedLabel: received and not blocked shows OK",
          "[FloatingUI][RSA]") {
    CHECK(FormatRsaShiftBlockedLabel(false, /*ever_received=*/true) == L"Shift: OK");
}

TEST_CASE("FormatRsaShiftBlockedLabel: received and blocked shows BRAKE TO SHIFT",
          "[FloatingUI][RSA]") {
    CHECK(FormatRsaShiftBlockedLabel(true, /*ever_received=*/true) == L"Shift: BRAKE TO SHIFT");
}

// ---------------------------------------------------------------------------
// PRND gear selector label helper (FormatPrndGearLabel)
// ---------------------------------------------------------------------------

TEST_CASE("FormatPrndGearLabel: all four positions", "[FloatingUI][PRND]") {
    // PrndSelector::Position: 0=P, 1=R, 2=N, 3=D
    CHECK(FormatPrndGearLabel(0) == L"Gear: P");
    CHECK(FormatPrndGearLabel(1) == L"Gear: R");
    CHECK(FormatPrndGearLabel(2) == L"Gear: N");
    CHECK(FormatPrndGearLabel(3) == L"Gear: D");
}

TEST_CASE("FormatPrndGearLabel: out-of-range returns fallback", "[FloatingUI][PRND]") {
    CHECK(FormatPrndGearLabel(-1) == L"Gear: ?");
    CHECK(FormatPrndGearLabel(4)  == L"Gear: ?");
}

TEST_CASE("FormatPrndGearLabel: reflects PhysicalWorld default position (P)",
          "[FloatingUI][PRND]") {
    ev1sim::PhysicalWorld world;
    // Default PRND is P (position 0).
    const int pos = static_cast<int>(world.prnd_selector().position());
    CHECK(pos == 0);
    CHECK(FormatPrndGearLabel(pos) == L"Gear: P");
}

TEST_CASE("FormatPrndGearLabel: reflects PhysicalWorld cycle_up to D",
          "[FloatingUI][PRND]") {
    ev1sim::PhysicalWorld world;
    // Cycle P → R → N → D.
    world.prnd_selector().cycle_up();
    CHECK(FormatPrndGearLabel(static_cast<int>(world.prnd_selector().position()))
          == L"Gear: R");
    world.prnd_selector().cycle_up();
    CHECK(FormatPrndGearLabel(static_cast<int>(world.prnd_selector().position()))
          == L"Gear: N");
    world.prnd_selector().cycle_up();
    CHECK(FormatPrndGearLabel(static_cast<int>(world.prnd_selector().position()))
          == L"Gear: D");
}

// ---------------------------------------------------------------------------
// PIM cruise status label helper (FormatPimCruiseStatusLabel)
// ---------------------------------------------------------------------------

TEST_CASE("FormatPimCruiseStatusLabel: never-received shows placeholder",
          "[FloatingUI][Cruise]") {
    // lamp_on and setpoint are ignored when ever_received_active is false.
    CHECK(FormatPimCruiseStatusLabel(/*ever_received=*/false, false, 0.0f)
          == L"Cruise: ---");
    CHECK(FormatPimCruiseStatusLabel(/*ever_received=*/false, true,  25.0f)
          == L"Cruise: ---");
}

TEST_CASE("FormatPimCruiseStatusLabel: received and inactive shows OFF",
          "[FloatingUI][Cruise]") {
    CHECK(FormatPimCruiseStatusLabel(/*ever_received=*/true, /*active=*/false, 0.0f)
          == L"Cruise: OFF");
    // Setpoint is ignored when cruise is not active.
    CHECK(FormatPimCruiseStatusLabel(/*ever_received=*/true, /*active=*/false, 20.0f)
          == L"Cruise: OFF");
}

TEST_CASE("FormatPimCruiseStatusLabel: active with setpoint shows speed and ON",
          "[FloatingUI][Cruise]") {
    // "Cruise: 23.5 m/s ON"
    CHECK(FormatPimCruiseStatusLabel(/*ever_received=*/true, /*active=*/true, 23.5f)
          == L"Cruise: 23.5 m/s ON");
}

TEST_CASE("FormatPimCruiseStatusLabel: active with zero setpoint (edge case)",
          "[FloatingUI][Cruise]") {
    CHECK(FormatPimCruiseStatusLabel(/*ever_received=*/true, /*active=*/true, 0.0f)
          == L"Cruise: 0.0 m/s ON");
}

// ---------------------------------------------------------------------------
// RSA run-mode label helper (FormatRsaRunModeLabel)
// kSigRunModeBroadcast (5711): 0=OFF, 1=ACC, 2=RUN.
// START (4) only appears on mode-button input (6971), never on the broadcast.
// ---------------------------------------------------------------------------

TEST_CASE("FormatRsaRunModeLabel: never-received shows placeholder",
          "[FloatingUI][RunMode]") {
    // mode value is irrelevant when ever_received=false.
    CHECK(FormatRsaRunModeLabel(0u, /*ever_received=*/false) == L"Mode: ---");
    CHECK(FormatRsaRunModeLabel(2u, /*ever_received=*/false) == L"Mode: ---");
}

TEST_CASE("FormatRsaRunModeLabel: mode OFF (0)", "[FloatingUI][RunMode]") {
    CHECK(FormatRsaRunModeLabel(0u, /*ever_received=*/true) == L"Mode: OFF");
}

TEST_CASE("FormatRsaRunModeLabel: mode ACC (1)", "[FloatingUI][RunMode]") {
    CHECK(FormatRsaRunModeLabel(1u, /*ever_received=*/true) == L"Mode: ACC");
}

TEST_CASE("FormatRsaRunModeLabel: mode RUN (2)", "[FloatingUI][RunMode]") {
    CHECK(FormatRsaRunModeLabel(2u, /*ever_received=*/true) == L"Mode: RUN");
}

TEST_CASE("FormatRsaRunModeLabel: unknown enum shows fallback with raw byte",
          "[FloatingUI][RunMode]") {
    // Values beyond 0-2 should not appear on the broadcast, but are handled
    // gracefully so a mis-encoded or future enum value doesn't crash.
    CHECK(FormatRsaRunModeLabel(3u, /*ever_received=*/true) == L"Mode: ?(3)");
    CHECK(FormatRsaRunModeLabel(0xFFu, /*ever_received=*/true) == L"Mode: ?(255)");
}

// ---------------------------------------------------------------------------
// IPC trip distance label helper (FormatIpcTripDistanceLabel)
// Signal 4132: float32 LE, metres.  Displayed as km with one decimal place.
// ---------------------------------------------------------------------------

TEST_CASE("FormatIpcTripDistanceLabel: never-received shows placeholder",
          "[FloatingUI][IPC][Trip]") {
    CHECK(FormatIpcTripDistanceLabel(/*ever_received=*/false, 0.0f) == L"Trip: ---");
    CHECK(FormatIpcTripDistanceLabel(/*ever_received=*/false, -1.0f) == L"Trip: ---");
}

TEST_CASE("FormatIpcTripDistanceLabel: zero metres shows 0.0 km",
          "[FloatingUI][IPC][Trip]") {
    CHECK(FormatIpcTripDistanceLabel(/*ever_received=*/true, 0.0f) == L"Trip: 0.0 km");
}

TEST_CASE("FormatIpcTripDistanceLabel: 12300 metres shows 12.3 km",
          "[FloatingUI][IPC][Trip]") {
    CHECK(FormatIpcTripDistanceLabel(/*ever_received=*/true, 12300.0f) == L"Trip: 12.3 km");
}

TEST_CASE("FormatIpcTripDistanceLabel: 1000 metres shows 1.0 km",
          "[FloatingUI][IPC][Trip]") {
    CHECK(FormatIpcTripDistanceLabel(/*ever_received=*/true, 1000.0f) == L"Trip: 1.0 km");
}

TEST_CASE("FormatIpcTripDistanceLabel: 500 metres shows 0.5 km",
          "[FloatingUI][IPC][Trip]") {
    CHECK(FormatIpcTripDistanceLabel(/*ever_received=*/true, 500.0f) == L"Trip: 0.5 km");
}

// ---------------------------------------------------------------------------
// Pedal percent label helpers (FormatPedalPercentLabel)
// value_0_to_1: clamped 0..1, displayed as integer percent (no trailing decimal).
// ---------------------------------------------------------------------------

TEST_CASE("FormatPedalPercentLabel: throttle 0% and 100% endpoints",
          "[FloatingUI][Pedal]") {
    CHECK(FormatPedalPercentLabel("Throttle", 0.0)  == L"Throttle: 0%");
    CHECK(FormatPedalPercentLabel("Throttle", 1.0)  == L"Throttle: 100%");
}

TEST_CASE("FormatPedalPercentLabel: brake 0% and 100% endpoints",
          "[FloatingUI][Pedal]") {
    CHECK(FormatPedalPercentLabel("Brake", 0.0) == L"Brake: 0%");
    CHECK(FormatPedalPercentLabel("Brake", 1.0) == L"Brake: 100%");
}

TEST_CASE("FormatPedalPercentLabel: intermediate values 25/50/75 percent",
          "[FloatingUI][Pedal]") {
    CHECK(FormatPedalPercentLabel("Throttle", 0.25) == L"Throttle: 25%");
    CHECK(FormatPedalPercentLabel("Throttle", 0.50) == L"Throttle: 50%");
    CHECK(FormatPedalPercentLabel("Throttle", 0.75) == L"Throttle: 75%");
}

TEST_CASE("FormatPedalPercentLabel: values outside [0,1] are clamped",
          "[FloatingUI][Pedal]") {
    // Below 0 → clamped to 0%
    CHECK(FormatPedalPercentLabel("Throttle", -0.5) == L"Throttle: 0%");
    CHECK(FormatPedalPercentLabel("Brake",    -1.0) == L"Brake: 0%");
    // Above 1 → clamped to 100%
    CHECK(FormatPedalPercentLabel("Throttle", 1.5)  == L"Throttle: 100%");
    CHECK(FormatPedalPercentLabel("Brake",    2.0)  == L"Brake: 100%");
}

TEST_CASE("FormatPedalPercentLabel: NaN shows placeholder",
          "[FloatingUI][Pedal]") {
    const double nan_val = std::numeric_limits<double>::quiet_NaN();
    CHECK(FormatPedalPercentLabel("Throttle", nan_val) == L"Throttle: ---");
    CHECK(FormatPedalPercentLabel("Brake",    nan_val) == L"Brake: ---");
}

TEST_CASE("FormatPedalPercentLabel: rounding — 0.474 rounds to 47, 0.475 rounds to 48",
          "[FloatingUI][Pedal]") {
    CHECK(FormatPedalPercentLabel("Throttle", 0.474) == L"Throttle: 47%");
    CHECK(FormatPedalPercentLabel("Throttle", 0.475) == L"Throttle: 48%");
}

// ---------------------------------------------------------------------------
// Headlamp status label helper (FormatHeadlampStatusLabel)
// Derived from bulb feed line cache: LLBH/RLBH = low beam, LHBH/RHBH = high beam.
// Priority: HIGH > LOW > OFF.  ever_received=false shows "---".
// ---------------------------------------------------------------------------

TEST_CASE("FormatHeadlampStatusLabel: never-received shows placeholder",
          "[FloatingUI][Headlamp]") {
    // ever_received=false; low/high values are irrelevant.
    CHECK(FormatHeadlampStatusLabel(false, false, /*ever_received=*/false)
          == L"Headlamps: ---");
    CHECK(FormatHeadlampStatusLabel(true,  false, /*ever_received=*/false)
          == L"Headlamps: ---");
    CHECK(FormatHeadlampStatusLabel(false, true,  /*ever_received=*/false)
          == L"Headlamps: ---");
    CHECK(FormatHeadlampStatusLabel(true,  true,  /*ever_received=*/false)
          == L"Headlamps: ---");
}

TEST_CASE("FormatHeadlampStatusLabel: received and both off shows OFF",
          "[FloatingUI][Headlamp]") {
    CHECK(FormatHeadlampStatusLabel(false, false, /*ever_received=*/true)
          == L"Headlamps: OFF");
}

TEST_CASE("FormatHeadlampStatusLabel: received and low only shows LOW",
          "[FloatingUI][Headlamp]") {
    CHECK(FormatHeadlampStatusLabel(true,  false, /*ever_received=*/true)
          == L"Headlamps: LOW");
}

TEST_CASE("FormatHeadlampStatusLabel: received and high only shows HIGH",
          "[FloatingUI][Headlamp]") {
    CHECK(FormatHeadlampStatusLabel(false, true,  /*ever_received=*/true)
          == L"Headlamps: HIGH");
}

TEST_CASE("FormatHeadlampStatusLabel: received and both on shows HIGH (priority)",
          "[FloatingUI][Headlamp]") {
    // High beam takes priority over low when both report on.
    CHECK(FormatHeadlampStatusLabel(true,  true,  /*ever_received=*/true)
          == L"Headlamps: HIGH");
}

// ---------------------------------------------------------------------------
// Turn-signal status label helper (FormatTurnSignalStatusLabel)
// Derived from bulb feed line cache: LFTS|LRTS = left, RFTS|RRTS = right.
// ever_received=false shows "---".  Flashes ON/OFF each half-cycle (expected).
// ---------------------------------------------------------------------------

TEST_CASE("FormatTurnSignalStatusLabel: left never-received shows placeholder",
          "[FloatingUI][TurnSignal]") {
    CHECK(FormatTurnSignalStatusLabel(L"L", false, /*ever_received=*/false)
          == L"L Turn: ---");
    CHECK(FormatTurnSignalStatusLabel(L"L", true,  /*ever_received=*/false)
          == L"L Turn: ---");
}

TEST_CASE("FormatTurnSignalStatusLabel: right never-received shows placeholder",
          "[FloatingUI][TurnSignal]") {
    CHECK(FormatTurnSignalStatusLabel(L"R", false, /*ever_received=*/false)
          == L"R Turn: ---");
    CHECK(FormatTurnSignalStatusLabel(L"R", true,  /*ever_received=*/false)
          == L"R Turn: ---");
}

TEST_CASE("FormatTurnSignalStatusLabel: left received and inactive shows OFF",
          "[FloatingUI][TurnSignal]") {
    CHECK(FormatTurnSignalStatusLabel(L"L", false, /*ever_received=*/true)
          == L"L Turn: OFF");
}

TEST_CASE("FormatTurnSignalStatusLabel: left received and active shows ON",
          "[FloatingUI][TurnSignal]") {
    CHECK(FormatTurnSignalStatusLabel(L"L", true, /*ever_received=*/true)
          == L"L Turn: ON");
}

TEST_CASE("FormatTurnSignalStatusLabel: right received and inactive shows OFF",
          "[FloatingUI][TurnSignal]") {
    CHECK(FormatTurnSignalStatusLabel(L"R", false, /*ever_received=*/true)
          == L"R Turn: OFF");
}

TEST_CASE("FormatTurnSignalStatusLabel: right received and active shows ON",
          "[FloatingUI][TurnSignal]") {
    CHECK(FormatTurnSignalStatusLabel(L"R", true, /*ever_received=*/true)
          == L"R Turn: ON");
}

// ---------------------------------------------------------------------------
// Vehicle speed label helper (FormatVehicleSpeedLabel)
// Source: ev1sim VehicleState.speed_mps (ev1sim publishes kSigChassisSpeedMps=4100).
// ever_received=false shows "Speed: ---".
// Format: "Speed: N.N m/s (NN km/h)" — one decimal for m/s, integer km/h.
// ---------------------------------------------------------------------------

TEST_CASE("FormatVehicleSpeedLabel: never-received shows placeholder",
          "[FloatingUI][Speed]") {
    CHECK(FormatVehicleSpeedLabel(/*ever_received=*/false, 0.0f)
          == L"Speed: ---");
    // even with a non-zero speed, never-received takes priority
    CHECK(FormatVehicleSpeedLabel(/*ever_received=*/false, 27.78f)
          == L"Speed: ---");
}

TEST_CASE("FormatVehicleSpeedLabel: zero speed shows 0.0 m/s and 0 km/h",
          "[FloatingUI][Speed]") {
    CHECK(FormatVehicleSpeedLabel(/*ever_received=*/true, 0.0f)
          == L"Speed: 0.0 m/s (0 km/h)");
}

TEST_CASE("FormatVehicleSpeedLabel: 100 km/h = 27.78 m/s",
          "[FloatingUI][Speed]") {
    // 100 km/h = 100/3.6 ≈ 27.777... m/s → "27.8 m/s (100 km/h)"
    CHECK(FormatVehicleSpeedLabel(/*ever_received=*/true, 100.0f / 3.6f)
          == L"Speed: 27.8 m/s (100 km/h)");
}

TEST_CASE("FormatVehicleSpeedLabel: 50 km/h = 13.89 m/s",
          "[FloatingUI][Speed]") {
    // 50 km/h = 50/3.6 ≈ 13.888... m/s → "13.9 m/s (50 km/h)"
    CHECK(FormatVehicleSpeedLabel(/*ever_received=*/true, 50.0f / 3.6f)
          == L"Speed: 13.9 m/s (50 km/h)");
}

// ---------------------------------------------------------------------------
// HVAC driver-control labels (Round 2 — surfacing the HVAC control panel)
// ---------------------------------------------------------------------------

TEST_CASE("FormatHvacSetpointLabel: one-decimal Celsius", "[FloatingUI]") {
    CHECK(FormatHvacSetpointLabel(21.0) == L"HVAC Temp: 21.0 C");
    CHECK(FormatHvacSetpointLabel(16.5) == L"HVAC Temp: 16.5 C");
    CHECK(FormatHvacSetpointLabel(30.0) == L"HVAC Temp: 30.0 C");
}

TEST_CASE("FormatHvacFanLabel: OFF/LOW/MED/HIGH + fallback", "[FloatingUI]") {
    CHECK(FormatHvacFanLabel(0) == L"HVAC Fan: OFF");
    CHECK(FormatHvacFanLabel(1) == L"HVAC Fan: LOW");
    CHECK(FormatHvacFanLabel(2) == L"HVAC Fan: MED");
    CHECK(FormatHvacFanLabel(3) == L"HVAC Fan: HIGH");
    CHECK(FormatHvacFanLabel(9) == L"HVAC Fan: ?");
}

TEST_CASE("FormatHvacModeLabel: FACE/BILEVEL/FEET/DEFROST + fallback", "[FloatingUI]") {
    CHECK(FormatHvacModeLabel(0) == L"HVAC Mode: FACE");
    CHECK(FormatHvacModeLabel(1) == L"HVAC Mode: BILEVEL");
    CHECK(FormatHvacModeLabel(2) == L"HVAC Mode: FEET");
    CHECK(FormatHvacModeLabel(3) == L"HVAC Mode: DEFROST");
    CHECK(FormatHvacModeLabel(9) == L"HVAC Mode: ?");
}

TEST_CASE("FormatHvacAcLabel / FormatHvacDefrostLabel: ON/OFF", "[FloatingUI]") {
    CHECK(FormatHvacAcLabel(true)  == L"HVAC A/C: ON");
    CHECK(FormatHvacAcLabel(false) == L"HVAC A/C: OFF");
    CHECK(FormatHvacDefrostLabel(true)  == L"HVAC Defrost: ON");
    CHECK(FormatHvacDefrostLabel(false) == L"HVAC Defrost: OFF");
}

// Cross-check the wire-level accessors feed the labels correctly: cycle the
// HvacControls model and confirm the uint8 enums map to the expected labels.
TEST_CASE("FloatingUI: HvacControls cycle drives fan/mode labels", "[FloatingUI][Callbacks]") {
    ev1sim::HvacControls h;
    CHECK(FormatHvacFanLabel(h.fan_u8())   == L"HVAC Fan: OFF");
    CHECK(FormatHvacModeLabel(h.mode_u8()) == L"HVAC Mode: FACE");
    h.cycle_fan();   // OFF -> LOW
    h.cycle_mode();  // FACE -> BILEVEL
    CHECK(FormatHvacFanLabel(h.fan_u8())   == L"HVAC Fan: LOW");
    CHECK(FormatHvacModeLabel(h.mode_u8()) == L"HVAC Mode: BILEVEL");
    h.toggle_ac();
    CHECK(FormatHvacAcLabel(h.ac_on()) == L"HVAC A/C: ON");
}

// --- Steering-angle indicator -------------------------------------------------

TEST_CASE("FormatSteeringAngleLabel: centered shows no side", "[FloatingUI]") {
    CHECK(FormatSteeringAngleLabel(0.0)   == L"Steering: 0.0 deg");
    CHECK(FormatSteeringAngleLabel(0.02)  == L"Steering: 0.0 deg");  // within deadband
    CHECK(FormatSteeringAngleLabel(-0.02) == L"Steering: 0.0 deg");
}

TEST_CASE("FormatSteeringAngleLabel: positive = left, negative = right", "[FloatingUI]") {
    CHECK(FormatSteeringAngleLabel(12.3)  == L"Steering: 12.3 deg L");
    CHECK(FormatSteeringAngleLabel(-7.0)  == L"Steering: 7.0 deg R");
}

TEST_CASE("FormatSteeringAngleLabel: rounds to one decimal", "[FloatingUI]") {
    CHECK(FormatSteeringAngleLabel(12.34)  == L"Steering: 12.3 deg L");
    CHECK(FormatSteeringAngleLabel(-12.36) == L"Steering: 12.4 deg R");
}

TEST_CASE("FormatSteeringAngleLabel: non-finite shows placeholder", "[FloatingUI]") {
    CHECK(FormatSteeringAngleLabel(std::numeric_limits<double>::quiet_NaN())
          == L"Steering: ---");
    CHECK(FormatSteeringAngleLabel(std::numeric_limits<double>::infinity())
          == L"Steering: ---");
}

// --- Power-window momentary hold-button label -------------------------------

TEST_CASE("FormatPowerWindowButtonLabel: appends [ON] only while held", "[FloatingUI]") {
    CHECK(FormatPowerWindowButtonLabel(L"Drv Window Up", false) == L"Drv Window Up");
    CHECK(FormatPowerWindowButtonLabel(L"Drv Window Up", true)  == L"Drv Window Up [ON]");
    CHECK(FormatPowerWindowButtonLabel(L"Pass Window Down", true)
          == L"Pass Window Down [ON]");
}

TEST_CASE("FormatPowerWindowButtonLabel: null text is treated as empty", "[FloatingUI]") {
    CHECK(FormatPowerWindowButtonLabel(nullptr, false) == L"");
    CHECK(FormatPowerWindowButtonLabel(nullptr, true)  == L" [ON]");
}

// --- Hood two-stage latch status label --------------------------------------

TEST_CASE("FormatHoodStateLabel: CLOSED / POPPED / OPEN + fallback", "[FloatingUI]") {
    CHECK(FormatHoodStateLabel(0) == L"Hood: CLOSED");
    CHECK(FormatHoodStateLabel(1) == L"Hood: POPPED (safety catch)");
    CHECK(FormatHoodStateLabel(2) == L"Hood: OPEN");
    CHECK(FormatHoodStateLabel(9) == L"Hood: ?");
}
