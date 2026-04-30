#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ExternalSimConnector.h"

#include <set>
#include <string>

using Catch::Matchers::WithinAbs;

// ---------------------------------------------------------------------------
// Endpoint registry
// ---------------------------------------------------------------------------

TEST_CASE("Endpoint table covers every device exactly once", "[ExternalSim]") {
    constexpr int kNumDynamics    = 18;  // 10 chassis/actuator + 4 wheel_omega + 4 slip_ratio
    constexpr int kNumCombSw      = 3;   // combination switch: low_beam, flash_to_pass, park_headlamp
    constexpr int kNumChargeCplr  = 1;   // charge coupler present (4060, stub)
    constexpr int kNumPrnd        = 4;   // PRND selector lines (4050-4053)
    constexpr int kNumMotor       = 2;   // motor_rpm (4070), motor_torque_nm (4071)
    constexpr int kNumThrottleCmd = 1;   // throttle_cmd_q8 (4073) — PIM → ev1sim
    constexpr int kNumBrake       = 1;   // master_cylinder_pressure_kpa (4074) — ev1sim → BTCM
    constexpr int kNumWiper       = 2;   // wiper_motor_command (4080), washer_pump_command (4081)
    // Driver inputs on the main harness segment (electricsim_ev1_bus), output
    // from ev1sim: brake_pedal_q8 (6900), steering_deg_q8 (6901),
    // gear_selector (6902), throttle_q8 (6903), brake_switch (6904),
    // hazard_request (6944), turn_signal_left (6948), turn_signal_right (6949),
    // seatbelt_buckled (6964), rsa_mode_button (6971),
    // rsa_keypad_button1 (6975), button2 (6976), button3 (6977),
    // button4 (6978), button5 (6979),
    // ipc_trip_reset (6952), cruise_set (6953), cruise_resume (6954),
    // cruise_cancel (6955), cruise_speed_up (6956), cruise_speed_down (6957),
    // wiper_switch (6958), wiper_wash_request (6959).
    // (6970 is reserved — not registered as an endpoint.)
    constexpr int kNumDriverInputs = 23;
    const int expected = NUM_LIGHTS + 2 + VehiclePanels::NUM_PANELS +
                         kNumCombSw + kNumChargeCplr + kNumPrnd + kNumMotor +
                         kNumThrottleCmd + kNumBrake + kNumWiper +
                         kNumDynamics + kNumDriverInputs;
    REQUIRE(ExternalSimConnector::EndpointCount() == expected);

    // Unique signal IDs and names.
    std::set<std::uint32_t> ids;
    std::set<std::string>   qualified;
    std::set<std::string>   shorts;
    int bulb_count           = 0;
    int horn_count           = 0;
    int panel_count          = 0;
    int comb_sw_count        = 0;
    int charge_coupler_count = 0;
    int prnd_count           = 0;
    int dynamics_count       = 0;   // includes motor_rpm + motor_torque_nm
    int driver_input_count   = 0;

    const auto* eps = ExternalSimConnector::Endpoints();
    for (int i = 0; i < ExternalSimConnector::EndpointCount(); ++i) {
        const auto& e = eps[i];
        CHECK(ids.insert(e.signal_id).second);
        CHECK(qualified.insert(e.qualified_name).second);
        CHECK(shorts.insert(e.short_name).second);

        if (e.signal_id >= 4000 && e.signal_id <= 4018) {
            CHECK(e.input_to_sim);          // bulb feed lines flow into ev1sim
            ++bulb_count;
        } else if (e.signal_id == 4020 || e.signal_id == 4021) {
            CHECK(e.input_to_sim);          // horn drive lines too
            ++horn_count;
        } else if (e.signal_id >= 4030 && e.signal_id <= 4033) {
            CHECK_FALSE(e.input_to_sim);    // panel sensors are outputs
            ++panel_count;
        } else if (e.signal_id >= 4040 && e.signal_id <= 4042) {
            CHECK_FALSE(e.input_to_sim);    // combination switch outputs
            ++comb_sw_count;
        } else if (e.signal_id >= 4050 && e.signal_id <= 4053) {
            CHECK_FALSE(e.input_to_sim);    // PRND selector lines are outputs
            ++prnd_count;
        } else if (e.signal_id == 4060) {
            CHECK_FALSE(e.input_to_sim);    // charge coupler present is an output
            ++charge_coupler_count;
        } else if (e.signal_id == 4070 || e.signal_id == 4071) {
            CHECK_FALSE(e.input_to_sim);    // motor RPM + torque are outputs
            ++dynamics_count;
        } else if (e.signal_id == 4073) {
            CHECK(e.input_to_sim);          // PIM throttle command flows into ev1sim
            ++dynamics_count;
        } else if (e.signal_id == 4074) {
            CHECK_FALSE(e.input_to_sim);    // brake master pressure flows out to BTCM
            ++dynamics_count;
        } else if (e.signal_id == 4080 || e.signal_id == 4081) {
            CHECK(e.input_to_sim);          // wiper motor + washer pump cmds flow into ev1sim
            ++dynamics_count;
        } else if ((e.signal_id >= 4100 && e.signal_id <= 4109) ||
                   (e.signal_id >= 4110 && e.signal_id <= 4113) ||
                   (e.signal_id >= 4120 && e.signal_id <= 4123)) {
            CHECK_FALSE(e.input_to_sim);    // dynamics signals are outputs
            ++dynamics_count;
        } else if ((e.signal_id >= 6900 && e.signal_id <= 6904) ||
                   e.signal_id == 6944 ||
                   e.signal_id == 6948 ||
                   e.signal_id == 6949 ||
                   e.signal_id == 6964 ||
                   e.signal_id == 6971 ||
                   (e.signal_id >= 6975 && e.signal_id <= 6979) ||
                   (e.signal_id >= 6952 && e.signal_id <= 6959)) {
            // Driver inputs on the main harness segment — outputs from ev1sim.
            // 6900=brake_pedal_q8  6901=steering_deg_q8  6902=gear_selector
            // 6903=throttle_q8     6904=brake_switch
            // 6944=hazard_request  6948=turn_signal_left  6949=turn_signal_right
            // 6952=ipc_trip_reset  6953=cruise_set  6954=cruise_resume
            // 6955=cruise_cancel   6956=cruise_speed_up  6957=cruise_speed_down
            // 6958=wiper_switch    6959=wiper_wash_request
            // 6964=seatbelt_buckled  6971=rsa_mode_button
            // 6975..6979=rsa_keypad_button[1..5]  (6970 reserved; not registered)
            CHECK_FALSE(e.input_to_sim);
            ++driver_input_count;
        } else {
            FAIL("Unexpected signal_id " << e.signal_id);
        }
    }
    CHECK(bulb_count           == NUM_LIGHTS);
    CHECK(horn_count           == 2);
    CHECK(panel_count          == VehiclePanels::NUM_PANELS);
    CHECK(comb_sw_count        == kNumCombSw);
    CHECK(prnd_count           == kNumPrnd);
    CHECK(charge_coupler_count == kNumChargeCplr);
    // dynamics_count includes the 18 original dynamics + 2 motor signals (4070-4071)
    // + throttle command (4073) + 2 wiper signals (4080-4081).
    CHECK(dynamics_count       == kNumDynamics + kNumMotor + kNumThrottleCmd +
                                  kNumBrake + kNumWiper);
    CHECK(driver_input_count   == kNumDriverInputs);
}

TEST_CASE("FindEndpoint returns bulb feed-line rows", "[ExternalSim]") {
    // Signal 4000 is BACKUP_LEFT in the electric sim's LightIdx enum, which
    // on our side is LBL (Left Backup Lamp).
    const auto* lbl = ExternalSimConnector::FindEndpoint(4000);
    REQUIRE(lbl != nullptr);
    CHECK(std::string(lbl->qualified_name) == "vehicle.body.lbl.bulb_feed_line");
    CHECK(lbl->input_to_sim);

    // One past the last bulb ID is unused.
    CHECK(ExternalSimConnector::FindEndpoint(4019) == nullptr);
}

TEST_CASE("FindEndpoint returns dynamics rows", "[ExternalSim]") {
    // speed_mps lives at 4100.
    const auto* spd = ExternalSimConnector::FindEndpoint(4100);
    REQUIRE(spd != nullptr);
    CHECK(std::string(spd->qualified_name) == "vehicle.dynamics.speed_mps");
    CHECK_FALSE(spd->input_to_sim);   // output from ev1sim

    // Rear-right slip ratio is the last dynamics signal at 4123.
    const auto* sr = ExternalSimConnector::FindEndpoint(4123);
    REQUIRE(sr != nullptr);
    CHECK(std::string(sr->qualified_name) == "vehicle.dynamics.slip_ratio_rr");
    CHECK_FALSE(sr->input_to_sim);

    // Brake-actuator-state signals (added alongside independent front/rear
    // brake-actuator dynamics) live at 4107 (front_brake_pressure) and 4108
    // (rear_brake_position).  These are the actual hardware-side values
    // after lag/rate-limit, distinct from the 4105/4106 commanded inputs.
    const auto* fbp = ExternalSimConnector::FindEndpoint(4107);
    REQUIRE(fbp != nullptr);
    CHECK(std::string(fbp->qualified_name) == "vehicle.dynamics.front_brake_pressure");
    CHECK_FALSE(fbp->input_to_sim);

    const auto* rbp = ExternalSimConnector::FindEndpoint(4108);
    REQUIRE(rbp != nullptr);
    CHECK(std::string(rbp->qualified_name) == "vehicle.dynamics.rear_brake_position");
    CHECK_FALSE(rbp->input_to_sim);

    // Steering torque (front-axle Mz sum, fed to force-feedback peers) at 4109.
    const auto* st = ExternalSimConnector::FindEndpoint(4109);
    REQUIRE(st != nullptr);
    CHECK(std::string(st->qualified_name) == "vehicle.dynamics.steering_torque");
    CHECK_FALSE(st->input_to_sim);

    // 4114-4119 stays a gap between wheel-omega and slip-ratio blocks.
    CHECK(ExternalSimConnector::FindEndpoint(4114) == nullptr);
}

TEST_CASE("Bulb signal IDs mirror the electric sim's LightIdx order",
          "[ExternalSim]") {
    // Signal slot -> expected LightID mapping, derived from the electric sim's
    // LightIdx enum.  If either side changes, this test should fail loudly.
    struct Row { std::uint32_t sid; LightID expect; };
    const Row rows[] = {
        {4000, LightID::LBL},    {4001, LightID::RBL},
        {4002, LightID::LHBH},   {4003, LightID::RHBH},
        {4004, LightID::LLBH},   {4005, LightID::RLBH},
        {4006, LightID::LRSM},   {4007, LightID::RRSM},
        {4008, LightID::LFML},   {4009, LightID::RFML},
        {4010, LightID::LFTS},   {4011, LightID::RFTS},
        {4012, LightID::LRTS},   {4013, LightID::RRTS},
        {4014, LightID::LRSL},   {4015, LightID::CHMSL}, {4016, LightID::RRSL},
        // ev1sim-only tail filaments, past the shared range.
        {4017, LightID::LRTL},   {4018, LightID::RRTL},
    };
    for (const auto& r : rows) {
        ExternalSimConnector c;
        c.DebugInjectDelta(r.sid, true);
        INFO("signal_id " << r.sid << " -> LightID idx "
                          << static_cast<int>(r.expect));
        CHECK(c.GetBulbCmd(r.expect));
        // Only the expected LightID should have been flipped.
        for (int i = 0; i < NUM_LIGHTS; ++i) {
            if (static_cast<LightID>(i) != r.expect)
                CHECK_FALSE(c.GetBulbCmd(static_cast<LightID>(i)));
        }
    }
}

// ---------------------------------------------------------------------------
// Disabled connector (default) must be safe to drive every frame.
// ---------------------------------------------------------------------------

TEST_CASE("Disabled connector is inert", "[ExternalSim]") {
    ExternalSimConnector c;
    CHECK(c.GetStatus() == ExternalSimConnector::Status::Disabled);
    CHECK_FALSE(c.IsConnected());
    CHECK(std::string(c.StatusString()) == "disabled");

    // Repeated Tick/Start/Stop must not throw or change status.
    c.Start();
    c.Tick(0.0);
    c.Tick(0.1);
    c.Stop();
    CHECK(c.GetStatus() == ExternalSimConnector::Status::Disabled);

    // No bulb/horn/panel data until an external sim publishes.
    CHECK_FALSE(c.HasReceivedBulbData());
    CHECK_FALSE(c.GetBulbCmd(LightID::LHBH));
    CHECK_FALSE(c.GetHornLowCmd());
    CHECK_FALSE(c.GetHornHighCmd());
}

// ---------------------------------------------------------------------------
// Enabled-but-unavailable / enabled-connecting status strings.
// ---------------------------------------------------------------------------

TEST_CASE("Enabled connector reports a non-disabled status", "[ExternalSim]") {
    ExternalSimConnector::Options opts;
    opts.enabled = true;
    // Use a nonce bus name so we don't collide with a real run on the machine.
    opts.bus_name = "ev1sim_unit_test_bus_ignore_me";
    opts.reconnect_period_s = 60.0;  // don't thrash in tests
    ExternalSimConnector c(opts);

    // Either "connecting" (real build) or "unavailable" (stub build) — both
    // are fine; what matters is that it is not disabled.
    const std::string s = c.StatusString();
    CHECK_FALSE(s == "disabled");
    CHECK_FALSE(c.IsConnected());
}

// ---------------------------------------------------------------------------
// DebugInjectDelta — the inbound-decode test hook — must update the latched
// command state the exact same way a real DeltaBatch would.
// ---------------------------------------------------------------------------

TEST_CASE("Injected deltas latch bulb commands", "[ExternalSim]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.HasReceivedBulbData());

    // First slot = BACKUP_LEFT in the electric sim = LBL here.
    c.DebugInjectDelta(4000, true);
    CHECK(c.GetBulbCmd(LightID::LBL));
    CHECK(c.HasReceivedBulbData());

    c.DebugInjectDelta(4000, false);
    CHECK_FALSE(c.GetBulbCmd(LightID::LBL));
    CHECK(c.HasReceivedBulbData());   // latched on first ever write

    // CHMSL sits at slot 15 (between STOPLAMP_LEFT and STOPLAMP_RIGHT).
    c.DebugInjectDelta(4015, true);
    CHECK(c.GetBulbCmd(LightID::CHMSL));

    // Last slot of the ev1sim range is RRTL (tail filament).
    c.DebugInjectDelta(4018, true);
    CHECK(c.GetBulbCmd(LightID::RRTL));
}

TEST_CASE("Injected deltas latch horn commands", "[ExternalSim]") {
    ExternalSimConnector c;
    c.DebugInjectDelta(4020, true);
    c.DebugInjectDelta(4021, true);
    CHECK(c.GetHornLowCmd());
    CHECK(c.GetHornHighCmd());

    c.DebugInjectDelta(4020, false);
    CHECK_FALSE(c.GetHornLowCmd());
    CHECK(c.GetHornHighCmd());
}

TEST_CASE("Combination switch endpoints have correct IDs and direction", "[ExternalSim]") {
    const auto* lb = ExternalSimConnector::FindEndpoint(4040);
    REQUIRE(lb != nullptr);
    CHECK(std::string(lb->qualified_name) ==
          "vehicle.body.combination_switch.low_beam_out");
    CHECK_FALSE(lb->input_to_sim);

    const auto* ftp = ExternalSimConnector::FindEndpoint(4041);
    REQUIRE(ftp != nullptr);
    CHECK(std::string(ftp->qualified_name) ==
          "vehicle.body.combination_switch.flash_to_pass_out");
    CHECK_FALSE(ftp->input_to_sim);

    const auto* ph = ExternalSimConnector::FindEndpoint(4042);
    REQUIRE(ph != nullptr);
    CHECK(std::string(ph->qualified_name) ==
          "vehicle.body.combination_switch.park_headlamp_out");
    CHECK_FALSE(ph->input_to_sim);
}

TEST_CASE("Motor RPM and torque endpoints have correct IDs and direction", "[ExternalSim]") {
    const auto* rpm = ExternalSimConnector::FindEndpoint(4070);
    REQUIRE(rpm != nullptr);
    CHECK(std::string(rpm->qualified_name) == "vehicle.dynamics.motor_rpm");
    CHECK(std::string(rpm->short_name)     == "motor_rpm");
    CHECK_FALSE(rpm->input_to_sim);   // output from ev1sim

    const auto* torq = ExternalSimConnector::FindEndpoint(4071);
    REQUIRE(torq != nullptr);
    CHECK(std::string(torq->qualified_name) == "vehicle.dynamics.motor_torque_nm");
    CHECK(std::string(torq->short_name)     == "motor_torque_nm");
    CHECK_FALSE(torq->input_to_sim);  // output from ev1sim
}

TEST_CASE("RSA keypad endpoints have correct IDs and direction", "[ExternalSim]") {
    // Slot 6970 is reserved — must NOT be registered.
    CHECK(ExternalSimConnector::FindEndpoint(6970) == nullptr);

    const auto* mode_btn = ExternalSimConnector::FindEndpoint(6971);
    REQUIRE(mode_btn != nullptr);
    CHECK(std::string(mode_btn->qualified_name) == "vehicle.driver.rsa_mode_button");
    CHECK_FALSE(mode_btn->input_to_sim);  // output from ev1sim

    // Per-digit keypad buttons 6975-6979.
    const auto* btn1 = ExternalSimConnector::FindEndpoint(6975);
    REQUIRE(btn1 != nullptr);
    CHECK(std::string(btn1->qualified_name) == "vehicle.driver.rsa_keypad_button1");
    CHECK_FALSE(btn1->input_to_sim);

    const auto* btn5 = ExternalSimConnector::FindEndpoint(6979);
    REQUIRE(btn5 != nullptr);
    CHECK(std::string(btn5->qualified_name) == "vehicle.driver.rsa_keypad_button5");
    CHECK_FALSE(btn5->input_to_sim);
}

TEST_CASE("New driver-input endpoints (6952-6959) have correct IDs and direction", "[ExternalSim]") {
    // IPC trip-reset (6952).
    const auto* ipc = ExternalSimConnector::FindEndpoint(6952);
    REQUIRE(ipc != nullptr);
    CHECK(std::string(ipc->qualified_name) == "vehicle.driver.ipc_trip_reset_button");
    CHECK_FALSE(ipc->input_to_sim);

    // Cruise stalk: SET=6953, RESUME=6954, CANCEL=6955, SPEED_UP=6956, SPEED_DOWN=6957.
    const auto* cset = ExternalSimConnector::FindEndpoint(6953);
    REQUIRE(cset != nullptr);
    CHECK(std::string(cset->qualified_name) == "vehicle.driver.cruise_set");
    CHECK_FALSE(cset->input_to_sim);

    const auto* cresume = ExternalSimConnector::FindEndpoint(6954);
    REQUIRE(cresume != nullptr);
    CHECK(std::string(cresume->qualified_name) == "vehicle.driver.cruise_resume");
    CHECK_FALSE(cresume->input_to_sim);

    const auto* ccancel = ExternalSimConnector::FindEndpoint(6955);
    REQUIRE(ccancel != nullptr);
    CHECK(std::string(ccancel->qualified_name) == "vehicle.driver.cruise_cancel");
    CHECK_FALSE(ccancel->input_to_sim);

    const auto* cup = ExternalSimConnector::FindEndpoint(6956);
    REQUIRE(cup != nullptr);
    CHECK(std::string(cup->qualified_name) == "vehicle.driver.cruise_speed_up");
    CHECK_FALSE(cup->input_to_sim);

    const auto* cdown = ExternalSimConnector::FindEndpoint(6957);
    REQUIRE(cdown != nullptr);
    CHECK(std::string(cdown->qualified_name) == "vehicle.driver.cruise_speed_down");
    CHECK_FALSE(cdown->input_to_sim);

    // Wiper: switch=6958, wash=6959.
    const auto* wsw = ExternalSimConnector::FindEndpoint(6958);
    REQUIRE(wsw != nullptr);
    CHECK(std::string(wsw->qualified_name) == "vehicle.driver.wiper_switch");
    CHECK_FALSE(wsw->input_to_sim);

    const auto* wwash = ExternalSimConnector::FindEndpoint(6959);
    REQUIRE(wwash != nullptr);
    CHECK(std::string(wwash->qualified_name) == "vehicle.driver.wiper_wash_request");
    CHECK_FALSE(wwash->input_to_sim);
}

TEST_CASE("New driver-input setters (6952-6959) store without crashing", "[ExternalSim]") {
    ExternalSimConnector c;
    CHECK_NOTHROW(c.SetDriverIpcTripReset(true));
    CHECK_NOTHROW(c.SetDriverIpcTripReset(false));
    CHECK_NOTHROW(c.SetDriverCruiseSet(true));
    CHECK_NOTHROW(c.SetDriverCruiseResume(true));
    CHECK_NOTHROW(c.SetDriverCruiseCancel(true));
    CHECK_NOTHROW(c.SetDriverCruiseSpeedUp(true));
    CHECK_NOTHROW(c.SetDriverCruiseSpeedDown(true));
    CHECK_NOTHROW(c.SetDriverWiperSwitch(0));  // OFF
    CHECK_NOTHROW(c.SetDriverWiperSwitch(1));  // INT
    CHECK_NOTHROW(c.SetDriverWiperSwitch(2));  // LOW
    CHECK_NOTHROW(c.SetDriverWiperSwitch(3));  // HIGH
    CHECK_NOTHROW(c.SetDriverWiperWashRequest(true));
    CHECK_NOTHROW(c.Tick(0.0));
}

TEST_CASE("SetMotorRpm and SetMotorTorqueNm store without crashing", "[ExternalSim]") {
    ExternalSimConnector c;
    CHECK_NOTHROW(c.SetMotorRpm(1500.0f));
    CHECK_NOTHROW(c.SetMotorTorqueNm(-12.5f));
    CHECK_NOTHROW(c.Tick(0.0));
}

TEST_CASE("SetDriverRsaKeypadButtons and SetDriverRsaModeButton store without crashing", "[ExternalSim]") {
    ExternalSimConnector c;
    // 0=idle, 1=tap (lower digit), 2=long-press (higher digit).
    CHECK_NOTHROW(c.SetDriverRsaKeypadButton1(1));  // tap button 1 -> digit '1'
    CHECK_NOTHROW(c.SetDriverRsaKeypadButton2(2));  // long-press button 2 -> digit '4'
    CHECK_NOTHROW(c.SetDriverRsaKeypadButton3(0));  // idle
    CHECK_NOTHROW(c.SetDriverRsaKeypadButton4(0));  // idle
    CHECK_NOTHROW(c.SetDriverRsaKeypadButton5(0));  // idle
    CHECK_NOTHROW(c.SetDriverRsaModeButton(3));     // RUN
    CHECK_NOTHROW(c.Tick(0.0));
}

TEST_CASE("SetDriverRsaKeypadButton long-press value 2 stores without crashing", "[ExternalSim]") {
    ExternalSimConnector c;
    // Exercise all three values for one button.
    CHECK_NOTHROW(c.SetDriverRsaKeypadButton1(0));  // idle
    CHECK_NOTHROW(c.SetDriverRsaKeypadButton1(1));  // tap
    CHECK_NOTHROW(c.SetDriverRsaKeypadButton1(2));  // long-press
    CHECK_NOTHROW(c.Tick(0.0));
}

TEST_CASE("GetRsaRunMode returns 0xFF before any run-mode received", "[ExternalSim]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.HasReceivedRunMode());
    CHECK(c.GetRsaRunMode() == 0xFFu);
}

TEST_CASE("SetCombSwOutputs stores without crashing", "[ExternalSim]") {
    ExternalSimConnector c;
    CHECK_NOTHROW(c.SetCombSwOutputs(true, false, true));
    CHECK_NOTHROW(c.Tick(0.0));
}

TEST_CASE("ChargeCoupler endpoint has correct ID and direction", "[ExternalSim]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4060);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.body.charge_coupler.present");
    CHECK(std::string(ep->short_name)     == "charge_coupler_present");
    CHECK_FALSE(ep->input_to_sim);   // output from ev1sim
}

TEST_CASE("SetChargeCouplerPresent stores without crashing", "[ExternalSim]") {
    ExternalSimConnector c;
    CHECK_NOTHROW(c.SetChargeCouplerPresent(false));
    CHECK_NOTHROW(c.SetChargeCouplerPresent(true));
    CHECK_NOTHROW(c.Tick(0.0));
}

TEST_CASE("Injected deltas to panel IDs are ignored", "[ExternalSim]") {
    // Panel sensors are outputs — the electric sim must not be able to
    // drive our local ajar state by publishing on those IDs.
    ExternalSimConnector c;
    c.DebugInjectDelta(4030, true);   // HOOD
    CHECK_FALSE(c.GetPanelSensor(PanelID::HOOD));
}

TEST_CASE("Injected deltas to dynamics IDs are silently dropped", "[ExternalSim]") {
    // Dynamics signals are outputs — inbound writes must be ignored.
    ExternalSimConnector c;
    c.DebugInjectDelta(4100, true);   // speed_mps
    CHECK_FALSE(c.HasReceivedBulbData());
}

TEST_CASE("Unknown signal IDs are silently dropped", "[ExternalSim]") {
    ExternalSimConnector c;
    c.DebugInjectDelta(9999, true);
    CHECK_FALSE(c.HasReceivedBulbData());
}

// ---------------------------------------------------------------------------
// Panel setter round-trip.
// ---------------------------------------------------------------------------

TEST_CASE("Panel ajar state round-trips through the connector", "[ExternalSim]") {
    ExternalSimConnector c;

    c.SetPanelSensor(PanelID::TRUNK, true);
    CHECK(c.GetPanelSensor(PanelID::TRUNK));
    CHECK_FALSE(c.GetPanelSensor(PanelID::HOOD));

    c.SetPanelSensor(PanelID::HOOD, true);
    c.SetPanelSensor(PanelID::TRUNK, false);
    CHECK(c.GetPanelSensor(PanelID::HOOD));
    CHECK_FALSE(c.GetPanelSensor(PanelID::TRUNK));
}

// ---------------------------------------------------------------------------
// VehicleState publishing round-trip (stub build: just tests the store).
// ---------------------------------------------------------------------------

TEST_CASE("SetVehicleState stores values without crashing", "[ExternalSim]") {
    ExternalSimConnector c;

    VehicleState vs{};
    vs.speed_mps      = 13.4;
    vs.accel_long     = -2.1;
    vs.accel_lat      =  0.5;
    vs.yaw_rate       =  0.3;
    vs.applied_throttle    = 0.0;
    vs.applied_front_brake = 0.6;
    vs.applied_rear_brake  = 0.6;
    vs.wheel_omega    = {12.0, 12.1, 11.9, 11.8};
    vs.slip_ratio     = {0.05, 0.05, 0.04, 0.04};

    // Must not throw; Tick is a no-op in the stub build but must not crash.
    CHECK_NOTHROW(c.SetVehicleState(vs));
    CHECK_NOTHROW(c.Tick(0.0));
}

// ---------------------------------------------------------------------------
// BTCM front ABS solenoid — GetAbsPhaseFront freshness and phase decoding.
// ---------------------------------------------------------------------------

TEST_CASE("GetAbsPhaseFront returns stale APPLY when no solenoid data received",
          "[ExternalSim][ABS]") {
    ExternalSimConnector c;
    // No inject — timestamps are 0, freshness window is 200 ms.
    auto phase = c.GetAbsPhaseFront(std::chrono::milliseconds(200));
    using Phase = ExternalSimConnector::AbsPhaseFront::Phase;
    CHECK_FALSE(phase.fl_fresh);
    CHECK_FALSE(phase.fr_fresh);
    // Stale path always returns APPLY (safe default).
    CHECK(phase.fl == Phase::APPLY);
    CHECK(phase.fr == Phase::APPLY);
}

TEST_CASE("GetAbsPhaseFront decodes APPLY phase: iso=0, dump=0",
          "[ExternalSim][ABS]") {
    ExternalSimConnector c;
    // Inject iso=0, dump=0 for both wheels → APPLY.
    c.DebugInjectDelta(5010, false);  // FL_ISO = 0
    c.DebugInjectDelta(5011, false);  // FL_DMP = 0
    c.DebugInjectDelta(5012, false);  // FR_ISO = 0
    c.DebugInjectDelta(5013, false);  // FR_DMP = 0

    auto phase = c.GetAbsPhaseFront(std::chrono::milliseconds(200));
    using Phase = ExternalSimConnector::AbsPhaseFront::Phase;
    CHECK(phase.fl_fresh);
    CHECK(phase.fr_fresh);
    CHECK(phase.fl == Phase::APPLY);
    CHECK(phase.fr == Phase::APPLY);
}

TEST_CASE("GetAbsPhaseFront decodes HOLD phase: iso=1, dump=0",
          "[ExternalSim][ABS]") {
    ExternalSimConnector c;
    c.DebugInjectDelta(5010, true);   // FL_ISO = 1
    c.DebugInjectDelta(5011, false);  // FL_DMP = 0
    c.DebugInjectDelta(5012, true);   // FR_ISO = 1
    c.DebugInjectDelta(5013, false);  // FR_DMP = 0

    auto phase = c.GetAbsPhaseFront(std::chrono::milliseconds(200));
    using Phase = ExternalSimConnector::AbsPhaseFront::Phase;
    CHECK(phase.fl_fresh);
    CHECK(phase.fr_fresh);
    CHECK(phase.fl == Phase::HOLD);
    CHECK(phase.fr == Phase::HOLD);
}

TEST_CASE("GetAbsPhaseFront decodes DUMP phase: iso=1, dump=1",
          "[ExternalSim][ABS]") {
    ExternalSimConnector c;
    c.DebugInjectDelta(5010, true);   // FL_ISO = 1
    c.DebugInjectDelta(5011, true);   // FL_DMP = 1
    c.DebugInjectDelta(5012, true);   // FR_ISO = 1
    c.DebugInjectDelta(5013, true);   // FR_DMP = 1

    auto phase = c.GetAbsPhaseFront(std::chrono::milliseconds(200));
    using Phase = ExternalSimConnector::AbsPhaseFront::Phase;
    CHECK(phase.fl_fresh);
    CHECK(phase.fr_fresh);
    CHECK(phase.fl == Phase::DUMP);
    CHECK(phase.fr == Phase::DUMP);
}

TEST_CASE("GetAbsPhaseFront treats invalid iso=0,dump=1 as APPLY",
          "[ExternalSim][ABS]") {
    ExternalSimConnector c;
    // Invalid combination: iso=0, dump=1 → treated as APPLY.
    c.DebugInjectDelta(5010, false);  // FL_ISO = 0
    c.DebugInjectDelta(5011, true);   // FL_DMP = 1  (invalid)
    c.DebugInjectDelta(5012, false);  // FR_ISO = 0
    c.DebugInjectDelta(5013, true);   // FR_DMP = 1  (invalid)

    auto phase = c.GetAbsPhaseFront(std::chrono::milliseconds(200));
    using Phase = ExternalSimConnector::AbsPhaseFront::Phase;
    CHECK(phase.fl_fresh);
    CHECK(phase.fr_fresh);
    CHECK(phase.fl == Phase::APPLY);
    CHECK(phase.fr == Phase::APPLY);
}

TEST_CASE("GetAbsPhaseFront marks stale after zero-length freshness window",
          "[ExternalSim][ABS]") {
    ExternalSimConnector c;
    c.DebugInjectDelta(5010, true);
    c.DebugInjectDelta(5011, false);
    c.DebugInjectDelta(5012, true);
    c.DebugInjectDelta(5013, false);

    // A zero-length freshness window means everything is immediately stale.
    auto phase = c.GetAbsPhaseFront(std::chrono::milliseconds(0));
    CHECK_FALSE(phase.fl_fresh);
    CHECK_FALSE(phase.fr_fresh);
}

TEST_CASE("GetAbsPhaseFront can mix fresh and stale per wheel",
          "[ExternalSim][ABS]") {
    ExternalSimConnector c;
    // Only inject FL signals; FR has no data.
    c.DebugInjectDelta(5010, true);   // FL_ISO = 1
    c.DebugInjectDelta(5011, false);  // FL_DMP = 0  → FL=HOLD
    // FR signals not injected → timestamps remain 0 → stale.

    auto phase = c.GetAbsPhaseFront(std::chrono::milliseconds(200));
    using Phase = ExternalSimConnector::AbsPhaseFront::Phase;
    CHECK(phase.fl_fresh);
    CHECK_FALSE(phase.fr_fresh);
    CHECK(phase.fl == Phase::HOLD);
    CHECK(phase.fr == Phase::APPLY);  // stale defaults to APPLY
}
