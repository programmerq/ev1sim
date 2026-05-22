#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ExternalSimConnector.h"

#include <chrono>
#include <set>
#include <string>
#include <thread>

using Catch::Matchers::WithinAbs;

// ---------------------------------------------------------------------------
// Endpoint registry
// ---------------------------------------------------------------------------

TEST_CASE("Endpoint table covers every device exactly once", "[ExternalSim]") {
    constexpr int kNumDynamics      = 18;  // 10 chassis/actuator + 4 wheel_omega + 4 slip_ratio
    constexpr int kNumCombSw        = 3;   // headlamp switch: low_beam, flash_to_pass, park_headlamp (4040-4042)
    constexpr int kNumTurnHazSw     = 4;   // turn/hazard switch: right/left turn, hazard, horn (4043-4046)
    constexpr int kNumWiperSw       = 4;   // wiper/washer switch: delay, request, hi, washer (4054-4057)
    constexpr int kNumChargeCplr    = 1;   // charge coupler present (4060, stub)
    constexpr int kNumPrnd          = 4;   // PRND selector lines (4050-4053)
    constexpr int kNumMotor         = 2;   // motor_rpm (4070), motor_torque_nm (4071)
    constexpr int kNumSimTime       = 1;   // sim_time_ns (4075) — ev1sim → electricsim
    constexpr int kNumThrottleCmd   = 1;   // throttle_cmd_q8 (4073) — PIM → ev1sim
    constexpr int kNumBrake         = 1;   // master_cylinder_pressure_kpa (4074) — ev1sim → BTCM
    constexpr int kNumWiper         = 2;   // wiper_motor_command (4080), washer_pump_command (4081)
    constexpr int kNumHvac          = 2;   // hvac_blower_level (4082), defrost_grid_active (4083)
    constexpr int kNumAmbient       = 2;   // ambient_temp_c (4090), ambient_humidity_pct (4091)
    constexpr int kNumDoorLockPw    = 4;   // door_lock_cmd driver/passenger (4084/4085)
                                           // + power_window_motor driver/passenger (4086/4087)
    constexpr int kNumRsaShiftBlocked = 1; // rsa_shift_blocked (4088)
    constexpr int kNumIpcTelltale   = 2;   // ipc_seatbelt_telltale_driver (4130),
                                           // ipc_seatbelt_telltale_passenger (4131)
    constexpr int kNumIpcTripDist   = 1;   // ipc_trip_distance_m (4132)
    constexpr int kNumIpcBtcmTelltale = 5; // ipc_brake_telltale (4134), park_brake (4135),
                                           // antilock (4136), low_trac (4137), air_bag (4138)
    constexpr int kNumIpcExtraTelltale = 6; // service_now (4140), check_messages (4141),
                                            // temp (4142), battery_life (4143),
                                            // reduced_perf (4144), check_tire_press (4145)
    constexpr int kNumBpmPackVoltage  = 1;  // bpm_pack_voltage_mv (4139) — BPM → ev1sim
    constexpr int kNumBtcmChassisActuator = 8; // BTCM iso/dump/EMB/cylpress on chassis bus
                                               // (4147 iso_close FL, 4148 iso_close FR,
                                               //  4149 dump_open FL, 4150 dump_open FR,
                                               //  4151 EMB motor LR, 4152 EMB motor RR,
                                               //  4153 cyl press FL kPa, 4154 cyl press FR kPa)
    constexpr int kNumExtContract = 16;        // 3D-sim contract publishes on main harness:
                                               //  6920 body_velocity_mps, 6921/6922 long/lat accel,
                                               //  6930/6931 pose X/Y, 6932 pose yaw_rad,
                                               //  6942 headlight switch, 6943 headlight dim,
                                               //  6945 telltale test, 6946/6947 park brake set/release,
                                               //  6960/6961 door open driver/passenger,
                                               //  6962 hood open, 6963 trunk open,
                                               //  6966 key position
                                               //  (horn 6940/6941 removed — now chassis cavity 4046)
    // Driver inputs on the main harness segment (electricsim_ev1_bus), output
    // from ev1sim: brake_pedal_q8 (6900), steering_deg_q8 (6901),
    // gear_selector (6902), throttle_q8 (6903), brake_switch (6904),
    // hazard_request (6944), turn_signal_left (6948), turn_signal_right (6949),
    // seatbelt_buckled (6964), seatbelt_buckled_passenger (6965),
    // rsa_mode_button (6971),
    // rsa_keypad_button1 (6975), button2 (6976), button3 (6977),
    // button4 (6978), button5 (6979),
    // ipc_trip_reset (6952), cruise_set (6953), cruise_resume (6954),
    // cruise_cancel (6955), cruise_speed_up (6956), cruise_speed_down (6957),
    // wiper_switch (6958), wiper_wash_request (6959),
    // power_window_driver_up (6980), power_window_driver_down (6981),
    // power_window_passenger_up (6982), power_window_passenger_down (6983),
    // rsa_exterior_keypad1 (6985), rsa_exterior_keypad2 (6986),
    // rsa_exterior_keypad3 (6987), rsa_exterior_keypad4 (6988),
    // rsa_exterior_keypad5 (6989),
    // door_handle_attempt_driver (6990), door_handle_attempt_passenger (6991).
    // (6970 is reserved — not registered as an endpoint.)
    constexpr int kNumDriverInputs = 35;  // +1 for passenger seatbelt (6965)
    const int expected = NUM_LIGHTS + 2 + VehiclePanels::NUM_PANELS +
                         kNumCombSw + kNumTurnHazSw + kNumWiperSw +
                         kNumChargeCplr + kNumPrnd + kNumMotor +
                         kNumSimTime +
                         kNumThrottleCmd + kNumBrake + kNumWiper +
                         kNumHvac + kNumAmbient + kNumDoorLockPw +
                         kNumRsaShiftBlocked +
                         kNumIpcTelltale + kNumIpcTripDist + kNumIpcBtcmTelltale +
                         kNumIpcExtraTelltale + kNumBpmPackVoltage +
                         kNumBtcmChassisActuator + kNumExtContract +
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
    int turn_haz_count       = 0;   // turn/hazard switch cavities (4043-4046)
    int wiper_sw_count       = 0;   // wiper/washer switch cavities (4054-4057)
    int charge_coupler_count = 0;
    int prnd_count           = 0;
    int dynamics_count       = 0;   // includes motor_rpm + motor_torque_nm
    int hvac_count            = 0;   // hvac_blower_level (4082) + defrost_grid_active (4083)
    int door_lock_pw_count    = 0;   // door lock cmds (4084/4085) + pw motor cmds (4086/4087)
    int rsa_shift_blocked_count = 0; // rsa_shift_blocked (4088)
    int bpm_pack_voltage_count    = 0;   // bpm_pack_voltage_mv (4139)
    int ipc_telltale_count        = 0;   // IPC seatbelt telltales (4130/4131)
    int ipc_trip_dist_count       = 0;   // IPC trip distance (4132)
    int ipc_btcm_telltale_count   = 0;   // IPC BTCM/airbag telltales (4134–4138)
    int ipc_extra_telltale_count  = 0;   // IPC extra LCD telltales (4140–4145)
    int btcm_chassis_actuator_count = 0; // BTCM chassis-bus actuator state (4147-4154)
    int ext_contract_count          = 0; // 3D-sim contract publishes (6920-6932 + 6960-6963)
    int driver_input_count          = 0;

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
            CHECK_FALSE(e.input_to_sim);    // headlamp switch outputs
            ++comb_sw_count;
        } else if (e.signal_id >= 4043 && e.signal_id <= 4046) {
            CHECK_FALSE(e.input_to_sim);    // turn/hazard switch + horn cavities (outputs)
            ++turn_haz_count;
        } else if (e.signal_id >= 4054 && e.signal_id <= 4057) {
            CHECK_FALSE(e.input_to_sim);    // wiper/washer switch cavities (outputs)
            ++wiper_sw_count;
        } else if (e.signal_id >= 4050 && e.signal_id <= 4053) {
            CHECK_FALSE(e.input_to_sim);    // PRND selector lines are outputs
            ++prnd_count;
        } else if (e.signal_id == 4060) {
            CHECK_FALSE(e.input_to_sim);    // charge coupler present is an output
            ++charge_coupler_count;
        } else if (e.signal_id == 4070 || e.signal_id == 4071) {
            CHECK_FALSE(e.input_to_sim);    // motor RPM + torque are outputs
            ++dynamics_count;
        } else if (e.signal_id == 4075) {
            CHECK_FALSE(e.input_to_sim);    // sim-time clock is an output (ev1sim → electricsim)
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
        } else if (e.signal_id == 4082 || e.signal_id == 4083) {
            CHECK(e.input_to_sim);          // HTCM blower level + defrost grid flow into ev1sim
            ++hvac_count;
        } else if (e.signal_id >= 4084 && e.signal_id <= 4087) {
            CHECK(e.input_to_sim);          // door lock cmds (4084/4085) + pw motor cmds (4086/4087) flow into ev1sim
            ++door_lock_pw_count;
        } else if (e.signal_id == 4088) {
            CHECK(e.input_to_sim);          // RSA shift-blocked cue flows into ev1sim
            ++rsa_shift_blocked_count;
        } else if (e.signal_id == 4139) {
            CHECK(e.input_to_sim);          // BPM pack voltage flows into ev1sim
            ++bpm_pack_voltage_count;
        } else if (e.signal_id == 4130 || e.signal_id == 4131) {
            CHECK(e.input_to_sim);          // IPC seatbelt telltales flow into ev1sim
            ++ipc_telltale_count;
        } else if (e.signal_id == 4132) {
            CHECK(e.input_to_sim);          // IPC trip distance flows into ev1sim
            ++ipc_trip_dist_count;
        } else if (e.signal_id >= 4134 && e.signal_id <= 4138) {
            CHECK(e.input_to_sim);          // IPC BTCM/airbag telltales flow into ev1sim
            ++ipc_btcm_telltale_count;
        } else if (e.signal_id >= 4140 && e.signal_id <= 4145) {
            CHECK(e.input_to_sim);          // IPC extra LCD telltales flow into ev1sim
            ++ipc_extra_telltale_count;
        } else if (e.signal_id >= 4147 && e.signal_id <= 4154) {
            CHECK(e.input_to_sim);          // BTCM per-wheel actuator state flows into ev1sim
            ++btcm_chassis_actuator_count;
        } else if ((e.signal_id >= 6920 && e.signal_id <= 6922) ||
                   (e.signal_id >= 6930 && e.signal_id <= 6932) ||
                   (e.signal_id >= 6942 && e.signal_id <= 6943) ||
                    e.signal_id == 6945 ||
                   (e.signal_id >= 6946 && e.signal_id <= 6947) ||
                   (e.signal_id >= 6960 && e.signal_id <= 6963) ||
                    e.signal_id == 6966) {
            // 3D-sim contract publishes — main harness, ev1sim → electricsim.
            // 6920 body_velocity_mps, 6921/6922 long/lat accel,
            // 6930-6932 pose X/Y/yaw,
            // 6942 headlight switch, 6943 headlight dim,
            // 6945 telltale test, 6946/6947 park brake set/release,
            // 6960-6963 door/hood/trunk open sensors,
            // 6966 key position.
            CHECK_FALSE(e.input_to_sim);
            ++ext_contract_count;
        } else if (e.signal_id == 4090 || e.signal_id == 4091) {
            CHECK_FALSE(e.input_to_sim);    // ambient temp + humidity are outputs from ev1sim
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
                   e.signal_id == 6965 ||
                   e.signal_id == 6971 ||
                   (e.signal_id >= 6975 && e.signal_id <= 6979) ||
                   (e.signal_id >= 6952 && e.signal_id <= 6959) ||
                   (e.signal_id >= 6980 && e.signal_id <= 6983) ||
                   (e.signal_id >= 6985 && e.signal_id <= 6991)) {
            // Driver inputs on the main harness segment — outputs from ev1sim.
            // 6900=brake_pedal_q8  6901=steering_deg_q8  6902=gear_selector
            // 6903=throttle_q8     6904=brake_switch
            // 6944=hazard_request  6948=turn_signal_left  6949=turn_signal_right
            // 6952=ipc_trip_reset  6953=cruise_set  6954=cruise_resume
            // 6955=cruise_cancel   6956=cruise_speed_up  6957=cruise_speed_down
            // 6958=wiper_switch    6959=wiper_wash_request
            // 6964=seatbelt_buckled  6965=seatbelt_buckled_passenger
            // 6971=rsa_mode_button
            // 6975..6979=rsa_keypad_button[1..5]  (6970 reserved; not registered)
            // 6980=power_window_driver_up   6981=power_window_driver_down
            // 6982=power_window_passenger_up  6983=power_window_passenger_down
            // 6985..6989=rsa_exterior_keypad[1..5]
            // 6990=door_handle_attempt_driver  6991=door_handle_attempt_passenger
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
    CHECK(turn_haz_count       == kNumTurnHazSw);
    CHECK(wiper_sw_count       == kNumWiperSw);
    CHECK(prnd_count           == kNumPrnd);
    CHECK(charge_coupler_count == kNumChargeCplr);
    // dynamics_count: 18 original dynamics + 2 motor (4070-4071)
    // + sim-time (4075) + throttle cmd (4073) + brake pressure (4074)
    // + 2 wiper (4080-4081) + 2 ambient env (4090-4091).
    CHECK(dynamics_count       == kNumDynamics + kNumMotor + kNumSimTime +
                                  kNumThrottleCmd +
                                  kNumBrake + kNumWiper + kNumAmbient);
    // hvac_count: blower level (4082) + defrost grid (4083) — from HTCM.
    CHECK(hvac_count           == kNumHvac);
    // door_lock_pw_count: door lock cmds (4084/4085) + power window motor cmds (4086/4087).
    CHECK(door_lock_pw_count      == kNumDoorLockPw);
    // rsa_shift_blocked_count: RSA shift-blocked cue (4088).
    CHECK(rsa_shift_blocked_count == kNumRsaShiftBlocked);
    // bpm_pack_voltage_count: BPM pack voltage (4139) — uint32 mV, from BPM.
    CHECK(bpm_pack_voltage_count  == kNumBpmPackVoltage);
    // ipc_telltale_count: driver seatbelt telltale (4130) + passenger (4131) — from IPC.
    CHECK(ipc_telltale_count      == kNumIpcTelltale);
    // ipc_trip_dist_count: trip distance (4132) — from IPC.
    CHECK(ipc_trip_dist_count     == kNumIpcTripDist);
    // ipc_btcm_telltale_count: brake (4134) + park_brake (4135) + antilock (4136)
    //                          + low_trac (4137) + air_bag (4138) — from IPC.
    CHECK(ipc_btcm_telltale_count == kNumIpcBtcmTelltale);
    // ipc_extra_telltale_count: service_now (4140) + check_messages (4141) + temp (4142)
    //                           + battery_life (4143) + reduced_perf (4144) + check_tire_press (4145).
    CHECK(ipc_extra_telltale_count == kNumIpcExtraTelltale);
    // btcm_chassis_actuator_count: iso_close FL/FR (4147/4148) + dump_open FL/FR (4149/4150)
    //                             + EMB motor LR/RR (4151/4152) + cyl pressure FL/FR (4153/4154).
    CHECK(btcm_chassis_actuator_count == kNumBtcmChassisActuator);
    // ext_contract_count: body_velocity (6920) + long/lat accel (6921/6922)
    //                   + pose X/Y/yaw (6930-6932) + door/hood/trunk sensors (6960-6963).
    CHECK(ext_contract_count == kNumExtContract);
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

TEST_CASE("Ambient environment endpoints have correct IDs and direction", "[ExternalSim]") {
    const auto* temp = ExternalSimConnector::FindEndpoint(4090);
    REQUIRE(temp != nullptr);
    CHECK(std::string(temp->qualified_name) == "vehicle.environment.ambient_temp_c");
    CHECK(std::string(temp->short_name)     == "ambient_temp_c");
    CHECK_FALSE(temp->input_to_sim);   // output from ev1sim

    const auto* hum = ExternalSimConnector::FindEndpoint(4091);
    REQUIRE(hum != nullptr);
    CHECK(std::string(hum->qualified_name) == "vehicle.environment.ambient_humidity_pct");
    CHECK(std::string(hum->short_name)     == "ambient_humidity_pct");
    CHECK_FALSE(hum->input_to_sim);    // output from ev1sim
}

TEST_CASE("SetAmbientTempC and SetAmbientHumidityPct store without crashing", "[ExternalSim]") {
    ExternalSimConnector c;
    CHECK_NOTHROW(c.SetAmbientTempC(18.0f));
    CHECK_NOTHROW(c.SetAmbientTempC(-5.0f));
    CHECK_NOTHROW(c.SetAmbientHumidityPct(55.0f));
    CHECK_NOTHROW(c.SetAmbientHumidityPct(0.0f));
    CHECK_NOTHROW(c.SetAmbientHumidityPct(100.0f));
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

// Liveness gate: freshness = (BTCM heartbeat alive) AND (pin ever seen).
// Each test that expects fresh=true injects signal 5050 first to drive
// the heartbeat timestamp.  The bool argument is ignored — DebugInjectDelta's
// 5050 case just stamps NowNs() into btcm_uart_frame_ns.
TEST_CASE("GetAbsPhaseFront decodes APPLY phase: iso=0, dump=0",
          "[ExternalSim][ABS]") {
    ExternalSimConnector c;
    c.DebugInjectDelta(5050, true);   // BTCM heartbeat — drives btcm_alive.
    // Inject iso=0, dump=0 for both wheels → APPLY.
    c.DebugInjectDelta(5010, false);  // FL_ISO = 0
    c.DebugInjectDelta(5011, false);  // FL_DMP = 0
    c.DebugInjectDelta(5012, false);  // FR_ISO = 0
    c.DebugInjectDelta(5013, false);  // FR_DMP = 0

    auto phase = c.GetAbsPhaseFront(std::chrono::milliseconds(3000));
    using Phase = ExternalSimConnector::AbsPhaseFront::Phase;
    CHECK(phase.fl_fresh);
    CHECK(phase.fr_fresh);
    CHECK(phase.fl == Phase::APPLY);
    CHECK(phase.fr == Phase::APPLY);
}

TEST_CASE("GetAbsPhaseFront decodes HOLD phase: iso=1, dump=0",
          "[ExternalSim][ABS]") {
    ExternalSimConnector c;
    c.DebugInjectDelta(5050, true);   // BTCM heartbeat.
    c.DebugInjectDelta(5010, true);   // FL_ISO = 1
    c.DebugInjectDelta(5011, false);  // FL_DMP = 0
    c.DebugInjectDelta(5012, true);   // FR_ISO = 1
    c.DebugInjectDelta(5013, false);  // FR_DMP = 0

    auto phase = c.GetAbsPhaseFront(std::chrono::milliseconds(3000));
    using Phase = ExternalSimConnector::AbsPhaseFront::Phase;
    CHECK(phase.fl_fresh);
    CHECK(phase.fr_fresh);
    CHECK(phase.fl == Phase::HOLD);
    CHECK(phase.fr == Phase::HOLD);
}

TEST_CASE("GetAbsPhaseFront decodes DUMP phase: iso=1, dump=1",
          "[ExternalSim][ABS]") {
    ExternalSimConnector c;
    c.DebugInjectDelta(5050, true);   // BTCM heartbeat.
    c.DebugInjectDelta(5010, true);   // FL_ISO = 1
    c.DebugInjectDelta(5011, true);   // FL_DMP = 1
    c.DebugInjectDelta(5012, true);   // FR_ISO = 1
    c.DebugInjectDelta(5013, true);   // FR_DMP = 1

    auto phase = c.GetAbsPhaseFront(std::chrono::milliseconds(3000));
    using Phase = ExternalSimConnector::AbsPhaseFront::Phase;
    CHECK(phase.fl_fresh);
    CHECK(phase.fr_fresh);
    CHECK(phase.fl == Phase::DUMP);
    CHECK(phase.fr == Phase::DUMP);
}

TEST_CASE("GetAbsPhaseFront treats invalid iso=0,dump=1 as APPLY",
          "[ExternalSim][ABS]") {
    ExternalSimConnector c;
    c.DebugInjectDelta(5050, true);   // BTCM heartbeat.
    // Invalid combination: iso=0, dump=1 → treated as APPLY.
    c.DebugInjectDelta(5010, false);  // FL_ISO = 0
    c.DebugInjectDelta(5011, true);   // FL_DMP = 1  (invalid)
    c.DebugInjectDelta(5012, false);  // FR_ISO = 0
    c.DebugInjectDelta(5013, true);   // FR_DMP = 1  (invalid)

    auto phase = c.GetAbsPhaseFront(std::chrono::milliseconds(3000));
    using Phase = ExternalSimConnector::AbsPhaseFront::Phase;
    CHECK(phase.fl_fresh);
    CHECK(phase.fr_fresh);
    CHECK(phase.fl == Phase::APPLY);
    CHECK(phase.fr == Phase::APPLY);
}

TEST_CASE("GetAbsPhaseFront marks stale after zero-length freshness window",
          "[ExternalSim][ABS]") {
    ExternalSimConnector c;
    c.DebugInjectDelta(5050, true);   // BTCM heartbeat.
    c.DebugInjectDelta(5010, true);
    c.DebugInjectDelta(5011, false);
    c.DebugInjectDelta(5012, true);
    c.DebugInjectDelta(5013, false);

    // A zero-length freshness window means the heartbeat is immediately stale.
    auto phase = c.GetAbsPhaseFront(std::chrono::milliseconds(0));
    CHECK_FALSE(phase.fl_fresh);
    CHECK_FALSE(phase.fr_fresh);
}

TEST_CASE("GetAbsPhaseFront stays fresh through HOLD↔DUMP cycling",
          "[ExternalSim][ABS]") {
    // BTCM publishes solenoid state on change only.  During HOLD↔DUMP
    // cycling iso stays at 1 the whole time, so its per-pin timestamp
    // never refreshes — only dump toggles.  Freshness is gated on the
    // 5 Hz heartbeat (5050), not on the per-pin age, so as long as
    // the heartbeat is alive the last-known iso/dump values stay
    // authoritative.
    ExternalSimConnector c;
    c.DebugInjectDelta(5050, true);   // BTCM heartbeat.
    // Initial APPLY sample, then transition to HOLD → DUMP → HOLD → ...
    c.DebugInjectDelta(5010, false);  // FL_ISO = 0 (APPLY)
    c.DebugInjectDelta(5011, false);  // FL_DMP = 0
    c.DebugInjectDelta(5012, false);
    c.DebugInjectDelta(5013, false);

    // Now ABS engages: iso goes high once and stays high.
    c.DebugInjectDelta(5010, true);
    c.DebugInjectDelta(5012, true);

    // Refresh the heartbeat and toggle dump (HOLD → DUMP).  Iso's
    // per-pin timestamp does not refresh; the heartbeat does.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    c.DebugInjectDelta(5050, true);   // refresh heartbeat.
    c.DebugInjectDelta(5011, true);
    c.DebugInjectDelta(5013, true);

    auto phase = c.GetAbsPhaseFront(std::chrono::milliseconds(3000));
    using Phase = ExternalSimConnector::AbsPhaseFront::Phase;
    CHECK(phase.fl_fresh);
    CHECK(phase.fr_fresh);
    CHECK(phase.fl == Phase::DUMP);
    CHECK(phase.fr == Phase::DUMP);
}

TEST_CASE("GetAbsPhaseFront can mix fresh and stale per wheel",
          "[ExternalSim][ABS]") {
    ExternalSimConnector c;
    c.DebugInjectDelta(5050, true);   // BTCM heartbeat.
    // Only inject FL signals; FR has no data.
    c.DebugInjectDelta(5010, true);   // FL_ISO = 1
    c.DebugInjectDelta(5011, false);  // FL_DMP = 0  → FL=HOLD
    // FR signals not injected → timestamps remain 0 → stale.

    auto phase = c.GetAbsPhaseFront(std::chrono::milliseconds(3000));
    using Phase = ExternalSimConnector::AbsPhaseFront::Phase;
    CHECK(phase.fl_fresh);
    CHECK_FALSE(phase.fr_fresh);
    CHECK(phase.fl == Phase::HOLD);
    CHECK(phase.fr == Phase::APPLY);  // stale defaults to APPLY
}

// ---------------------------------------------------------------------------
// BTCM chassis-bus mirror (4147-4154): preference rules and cylinder pressure.
// ---------------------------------------------------------------------------

TEST_CASE("GetAbsPhaseFront prefers chassis-bus iso/dump over main-harness",
          "[ExternalSim][ABS][chassis-bus]") {
    ExternalSimConnector c;
    c.DebugInjectDelta(5050, true);   // BTCM heartbeat.
    // Main-harness says APPLY (iso=0, dump=0) for both wheels.
    c.DebugInjectDelta(5010, false);
    c.DebugInjectDelta(5011, false);
    c.DebugInjectDelta(5012, false);
    c.DebugInjectDelta(5013, false);
    // Chassis-bus mirror says DUMP (iso=1, dump=1) for both wheels.
    c.DebugInjectDelta(4147, true);   // iso_close FL
    c.DebugInjectDelta(4149, true);   // dump_open FL
    c.DebugInjectDelta(4148, true);   // iso_close FR
    c.DebugInjectDelta(4150, true);   // dump_open FR

    auto phase = c.GetAbsPhaseFront(std::chrono::milliseconds(3000));
    using Phase = ExternalSimConnector::AbsPhaseFront::Phase;
    CHECK(phase.fl_fresh);
    CHECK(phase.fr_fresh);
    CHECK(phase.fl == Phase::DUMP);   // chassis-bus wins
    CHECK(phase.fr == Phase::DUMP);
}

TEST_CASE("GetAbsPhaseFront falls back to main-harness when chassis-bus silent",
          "[ExternalSim][ABS][chassis-bus]") {
    ExternalSimConnector c;
    c.DebugInjectDelta(5050, true);   // BTCM heartbeat.
    // Only main-harness publishes; chassis-bus signals never arrive.
    c.DebugInjectDelta(5010, true);
    c.DebugInjectDelta(5011, false);  // FL = HOLD
    c.DebugInjectDelta(5012, true);
    c.DebugInjectDelta(5013, true);   // FR = DUMP

    auto phase = c.GetAbsPhaseFront(std::chrono::milliseconds(3000));
    using Phase = ExternalSimConnector::AbsPhaseFront::Phase;
    CHECK(phase.fl_fresh);
    CHECK(phase.fr_fresh);
    CHECK(phase.fl == Phase::HOLD);
    CHECK(phase.fr == Phase::DUMP);
}

TEST_CASE("GetAbsPhaseFront source preference is per-wheel",
          "[ExternalSim][ABS][chassis-bus]") {
    ExternalSimConnector c;
    c.DebugInjectDelta(5050, true);   // BTCM heartbeat.
    // FL only on chassis bus; FR only on main harness.
    c.DebugInjectDelta(4147, true);   // FL iso_close
    c.DebugInjectDelta(4149, true);   // FL dump_open  → FL=DUMP via chassis
    c.DebugInjectDelta(5012, true);   // FR_ISO main harness
    c.DebugInjectDelta(5013, false);  // FR_DMP main harness → FR=HOLD

    auto phase = c.GetAbsPhaseFront(std::chrono::milliseconds(3000));
    using Phase = ExternalSimConnector::AbsPhaseFront::Phase;
    CHECK(phase.fl_fresh);
    CHECK(phase.fr_fresh);
    CHECK(phase.fl == Phase::DUMP);
    CHECK(phase.fr == Phase::HOLD);
}

TEST_CASE("GetRearEmbCmd prefers chassis-bus EMB cmd over main-harness",
          "[ExternalSim][ABS][chassis-bus]") {
    ExternalSimConnector c;
    c.DebugInjectDelta(5050, true);    // BTCM heartbeat.
    c.DebugInjectFloat(5014, 0.0f);    // main harness LR = idle
    c.DebugInjectFloat(5015, 0.0f);    // main harness RR = idle
    c.DebugInjectFloat(4151, 1.0f);    // chassis LR = full apply
    c.DebugInjectFloat(4152, -1.0f);   // chassis RR = release

    auto cmd = c.GetRearEmbCmd(std::chrono::milliseconds(3000));
    CHECK(cmd.lr_fresh);
    CHECK(cmd.rr_fresh);
    CHECK(cmd.lr ==  1.0f);   // chassis wins
    CHECK(cmd.rr == -1.0f);
}

TEST_CASE("GetRearEmbCmd falls back to main-harness when chassis-bus silent",
          "[ExternalSim][ABS][chassis-bus]") {
    ExternalSimConnector c;
    c.DebugInjectDelta(5050, true);    // BTCM heartbeat.
    c.DebugInjectFloat(5014, 0.7f);    // main harness LR
    c.DebugInjectFloat(5015, -0.3f);   // main harness RR
    // chassis 4151/4152 never injected.

    auto cmd = c.GetRearEmbCmd(std::chrono::milliseconds(3000));
    CHECK(cmd.lr_fresh);
    CHECK(cmd.rr_fresh);
    CHECK(cmd.lr ==  0.7f);
    CHECK(cmd.rr == -0.3f);
}

TEST_CASE("GetFrontWheelCylinderPressuresKpa stale by default",
          "[ExternalSim][ABS][chassis-bus]") {
    ExternalSimConnector c;
    auto p = c.GetFrontWheelCylinderPressuresKpa(std::chrono::milliseconds(3000));
    CHECK_FALSE(p.fl_fresh);
    CHECK_FALSE(p.fr_fresh);
    CHECK(p.fl_kpa == 0.0f);
    CHECK(p.fr_kpa == 0.0f);
}

TEST_CASE("GetFrontWheelCylinderPressuresKpa returns injected values when fresh",
          "[ExternalSim][ABS][chassis-bus]") {
    ExternalSimConnector c;
    c.DebugInjectDelta(5050, true);     // BTCM heartbeat.
    c.DebugInjectFloat(4153, 2450.0f);  // FL = 2450 kPa
    c.DebugInjectFloat(4154, 1750.0f);  // FR = 1750 kPa

    auto p = c.GetFrontWheelCylinderPressuresKpa(std::chrono::milliseconds(3000));
    CHECK(p.fl_fresh);
    CHECK(p.fr_fresh);
    CHECK(p.fl_kpa == 2450.0f);
    CHECK(p.fr_kpa == 1750.0f);
}

// ---------------------------------------------------------------------------
// Door lock subscription (chassis bus 4084/4085)
// ---------------------------------------------------------------------------

TEST_CASE("Door lock cmd subscription: default never-received sentinel",
          "[ExternalSim][DoorLock]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.HasReceivedDoorLockCmd(0));   // driver — never received
    CHECK_FALSE(c.HasReceivedDoorLockCmd(1));   // passenger — never received
    CHECK(c.GetDoorLockCmd(0) == 0xFFu);
    CHECK(c.GetDoorLockCmd(1) == 0xFFu);
}

TEST_CASE("Door lock cmd subscription: DebugInjectU8 delivers driver cmd",
          "[ExternalSim][DoorLock]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4084, 0u);  // 4084 = kSigChassisDoorLockCmdDriver, 0=unlocked
    CHECK(c.HasReceivedDoorLockCmd(0));
    CHECK(c.GetDoorLockCmd(0) == 0u);  // unlocked
    CHECK_FALSE(c.HasReceivedDoorLockCmd(1));   // passenger untouched
}

TEST_CASE("Door lock cmd subscription: DebugInjectU8 delivers passenger cmd",
          "[ExternalSim][DoorLock]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4085, 1u);  // 4085 = kSigChassisDoorLockCmdPassenger, 1=locked
    CHECK(c.HasReceivedDoorLockCmd(1));
    CHECK(c.GetDoorLockCmd(1) == 1u);  // locked
    CHECK_FALSE(c.HasReceivedDoorLockCmd(0));   // driver untouched
}

TEST_CASE("Door lock cmd subscription: out-of-range side returns never-received",
          "[ExternalSim][DoorLock]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4084, 0u);
    CHECK_FALSE(c.HasReceivedDoorLockCmd(-1));
    CHECK_FALSE(c.HasReceivedDoorLockCmd(2));
    CHECK(c.GetDoorLockCmd(-1) == 0xFFu);
    CHECK(c.GetDoorLockCmd(2)  == 0xFFu);
}

// ---------------------------------------------------------------------------
// Power window motor subscription (chassis bus 4086/4087)
// ---------------------------------------------------------------------------

TEST_CASE("Power window motor subscription: default never-received sentinel",
          "[ExternalSim][PowerWindow]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.HasReceivedPowerWindowMotor(0));   // driver
    CHECK_FALSE(c.HasReceivedPowerWindowMotor(1));   // passenger
    CHECK(c.GetPowerWindowMotor(0) == 0xFFu);
    CHECK(c.GetPowerWindowMotor(1) == 0xFFu);
}

TEST_CASE("Power window motor subscription: DebugInjectU8 delivers driver motor cmd",
          "[ExternalSim][PowerWindow]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4086, 1u);  // 4086 = kSigChassisPowerWindowMotorDriver, 1=up
    CHECK(c.HasReceivedPowerWindowMotor(0));
    CHECK(c.GetPowerWindowMotor(0) == 1u);   // up
    CHECK_FALSE(c.HasReceivedPowerWindowMotor(1));   // passenger untouched
}

TEST_CASE("Power window motor subscription: DebugInjectU8 delivers passenger motor cmd",
          "[ExternalSim][PowerWindow]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4087, 2u);  // 4087 = kSigChassisPowerWindowMotorPassenger, 2=down
    CHECK(c.HasReceivedPowerWindowMotor(1));
    CHECK(c.GetPowerWindowMotor(1) == 2u);   // down
    CHECK_FALSE(c.HasReceivedPowerWindowMotor(0));   // driver untouched
}

TEST_CASE("FindEndpoint returns door lock and power window motor cmd rows",
          "[ExternalSim][DoorLock][PowerWindow]") {
    // Door lock cmds are inputs to ev1sim (electricsim/RSA → ev1sim).
    const auto* dl_drv = ExternalSimConnector::FindEndpoint(4084);
    REQUIRE(dl_drv != nullptr);
    CHECK(std::string(dl_drv->qualified_name) == "vehicle.body.door_lock_cmd.driver");
    CHECK(dl_drv->input_to_sim);

    const auto* dl_pax = ExternalSimConnector::FindEndpoint(4085);
    REQUIRE(dl_pax != nullptr);
    CHECK(std::string(dl_pax->qualified_name) == "vehicle.body.door_lock_cmd.passenger");
    CHECK(dl_pax->input_to_sim);

    const auto* pw_drv = ExternalSimConnector::FindEndpoint(4086);
    REQUIRE(pw_drv != nullptr);
    CHECK(std::string(pw_drv->qualified_name) == "vehicle.body.power_window_motor.driver");
    CHECK(pw_drv->input_to_sim);

    const auto* pw_pax = ExternalSimConnector::FindEndpoint(4087);
    REQUIRE(pw_pax != nullptr);
    CHECK(std::string(pw_pax->qualified_name) == "vehicle.body.power_window_motor.passenger");
    CHECK(pw_pax->input_to_sim);
}

// ---------------------------------------------------------------------------
// HVAC blower level subscription (chassis bus 4082)
// ---------------------------------------------------------------------------

TEST_CASE("HVAC blower level subscription: default never-received sentinel",
          "[ExternalSim][HVAC]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.HasReceivedHvacBlowerLevel());
    // 0xFF is the "never received" sentinel.
    CHECK(c.GetHvacBlowerLevel() == 0xFFu);
}

TEST_CASE("HVAC blower level subscription: DebugInjectU8 delivers all four levels",
          "[ExternalSim][HVAC]") {
    // Encoding per htcm_supervisor.h: 0=OFF, 1=LOW, 2=MED, 3=HIGH.
    for (std::uint8_t level = 0; level <= 3; ++level) {
        ExternalSimConnector c;
        c.DebugInjectU8(4082, level);  // 4082 = kSigChassisHvacBlowerLevel
        CHECK(c.HasReceivedHvacBlowerLevel());
        CHECK(c.GetHvacBlowerLevel() == level);
    }
}

TEST_CASE("HVAC blower level subscription: FindEndpoint returns correct metadata",
          "[ExternalSim][HVAC]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4082);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.hvac.blower_level");
    CHECK(std::string(ep->short_name)     == "hvac_blower_level");
    CHECK(ep->input_to_sim);   // HTCM publishes → ev1sim subscribes
}

// ---------------------------------------------------------------------------
// HVAC defrost grid subscription (chassis bus 4083)
// ---------------------------------------------------------------------------

TEST_CASE("HVAC defrost grid subscription: default never-received / inactive",
          "[ExternalSim][HVAC]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.HasReceivedDefrostGridActive());
    CHECK_FALSE(c.GetDefrostGridActive());   // default false (safe/off)
}

TEST_CASE("HVAC defrost grid subscription: DebugInjectU8 activates defrost",
          "[ExternalSim][HVAC]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4083, 1u);  // 4083 = kSigChassisDefrostGridActive, 1=active
    CHECK(c.HasReceivedDefrostGridActive());
    CHECK(c.GetDefrostGridActive());
}

TEST_CASE("HVAC defrost grid subscription: DebugInjectU8 deactivates defrost",
          "[ExternalSim][HVAC]") {
    ExternalSimConnector c;
    // Activate then deactivate.
    c.DebugInjectU8(4083, 1u);
    CHECK(c.GetDefrostGridActive());
    c.DebugInjectU8(4083, 0u);  // 0=inactive
    CHECK(c.HasReceivedDefrostGridActive());
    CHECK_FALSE(c.GetDefrostGridActive());
}

TEST_CASE("HVAC defrost grid subscription: FindEndpoint returns correct metadata",
          "[ExternalSim][HVAC]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4083);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.hvac.defrost_grid_active");
    CHECK(std::string(ep->short_name)     == "hvac_defrost_grid_active");
    CHECK(ep->input_to_sim);   // HTCM publishes → ev1sim subscribes
}

// ---------------------------------------------------------------------------
// RSA shift-blocked cue subscription (chassis bus 4088)
// ---------------------------------------------------------------------------

TEST_CASE("RSA shift-blocked subscription: default never-received / not blocked",
          "[ExternalSim][RSA]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.HasReceivedRsaShiftBlocked());
    CHECK_FALSE(c.GetRsaShiftBlocked());   // default false (no block)
}

TEST_CASE("RSA shift-blocked subscription: DebugInjectU8 sets blocked true",
          "[ExternalSim][RSA]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4088, 1u);  // 4088 = kSigChassisRsaShiftBlocked, 1=blocked
    CHECK(c.HasReceivedRsaShiftBlocked());
    CHECK(c.GetRsaShiftBlocked());
}

TEST_CASE("RSA shift-blocked subscription: DebugInjectU8 clears blocked",
          "[ExternalSim][RSA]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4088, 1u);
    CHECK(c.GetRsaShiftBlocked());
    c.DebugInjectU8(4088, 0u);  // 0=not blocked
    CHECK(c.HasReceivedRsaShiftBlocked());  // has_received latches on first write
    CHECK_FALSE(c.GetRsaShiftBlocked());
}

TEST_CASE("RSA shift-blocked subscription: FindEndpoint returns correct metadata",
          "[ExternalSim][RSA]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4088);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.body.rsa.shift_blocked");
    CHECK(std::string(ep->short_name)     == "rsa_shift_blocked");
    CHECK(ep->input_to_sim);   // RSA publishes → ev1sim subscribes
}

// ---------------------------------------------------------------------------
// IPC seatbelt telltale subscription (chassis bus 4130/4131)
// ---------------------------------------------------------------------------

TEST_CASE("IPC seatbelt telltale subscription: default never-received / off",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.HasReceivedIpcSeatbeltTelltaleDriver());
    CHECK_FALSE(c.GetIpcSeatbeltTelltaleDriver());    // default false (lamp off)
    CHECK_FALSE(c.HasReceivedIpcSeatbeltTelltalePassenger());
    CHECK_FALSE(c.GetIpcSeatbeltTelltalePassenger());
}

TEST_CASE("IPC seatbelt telltale subscription: DebugInjectU8 turns driver lamp on",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4130, 1u);  // 4130 = kSigChassisIpcSeatbeltTelltaleDriver, 1=lamp on
    CHECK(c.HasReceivedIpcSeatbeltTelltaleDriver());
    CHECK(c.GetIpcSeatbeltTelltaleDriver());
    // Passenger unaffected.
    CHECK_FALSE(c.HasReceivedIpcSeatbeltTelltalePassenger());
    CHECK_FALSE(c.GetIpcSeatbeltTelltalePassenger());
}

TEST_CASE("IPC seatbelt telltale subscription: DebugInjectU8 turns driver lamp off",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4130, 1u);
    CHECK(c.GetIpcSeatbeltTelltaleDriver());
    c.DebugInjectU8(4130, 0u);  // 0=lamp off
    CHECK(c.HasReceivedIpcSeatbeltTelltaleDriver());
    CHECK_FALSE(c.GetIpcSeatbeltTelltaleDriver());
}

TEST_CASE("IPC seatbelt telltale subscription: DebugInjectU8 turns passenger lamp on",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4131, 1u);  // 4131 = kSigChassisIpcSeatbeltTelltalePassenger, 1=lamp on
    CHECK(c.HasReceivedIpcSeatbeltTelltalePassenger());
    CHECK(c.GetIpcSeatbeltTelltalePassenger());
    // Driver unaffected.
    CHECK_FALSE(c.HasReceivedIpcSeatbeltTelltaleDriver());
    CHECK_FALSE(c.GetIpcSeatbeltTelltaleDriver());
}

TEST_CASE("IPC seatbelt telltale subscription: DebugInjectU8 turns passenger lamp off",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4131, 1u);
    CHECK(c.GetIpcSeatbeltTelltalePassenger());
    c.DebugInjectU8(4131, 0u);  // 0=lamp off
    CHECK(c.HasReceivedIpcSeatbeltTelltalePassenger());
    CHECK_FALSE(c.GetIpcSeatbeltTelltalePassenger());
}

TEST_CASE("IPC seatbelt telltale subscription: FindEndpoint returns driver metadata",
          "[ExternalSim][IPC]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4130);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.ipc.seatbelt_telltale_driver");
    CHECK(std::string(ep->short_name)     == "ipc_seatbelt_telltale_driver");
    CHECK(ep->input_to_sim);   // IPC publishes → ev1sim subscribes
}

TEST_CASE("IPC seatbelt telltale subscription: FindEndpoint returns passenger metadata",
          "[ExternalSim][IPC]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4131);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.ipc.seatbelt_telltale_passenger");
    CHECK(std::string(ep->short_name)     == "ipc_seatbelt_telltale_passenger");
    CHECK(ep->input_to_sim);   // IPC publishes → ev1sim subscribes
}

// ---------------------------------------------------------------------------
// PIM cruise-control subscription (main harness bus 5860 / 5861)
// These signals live on the main harness segment (same as ABS/RSA), so they
// are subscribed from main_transport but NOT registered as endpoints.
// ---------------------------------------------------------------------------

TEST_CASE("PIM cruise subscription: default never-received sentinel",
          "[ExternalSim][Cruise]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.HasReceivedPimCruiseActive());
    CHECK_FALSE(c.GetPimCruiseActive());          // default false (off/standby)
    CHECK_FALSE(c.HasReceivedPimCruiseSetpointMps());
    CHECK(c.GetPimCruiseSetpointMps() == 0.0f);   // default zero
}

TEST_CASE("PIM cruise subscription: DebugInjectU8 sets cruise active (5860)",
          "[ExternalSim][Cruise]") {
    ExternalSimConnector c;
    // 5860 = kSigPimCruiseActive, 1 = engaged.
    c.DebugInjectU8(5860, 1u);
    CHECK(c.HasReceivedPimCruiseActive());
    CHECK(c.GetPimCruiseActive());
    // Setpoint still not received.
    CHECK_FALSE(c.HasReceivedPimCruiseSetpointMps());
}

TEST_CASE("PIM cruise subscription: DebugInjectU8 clears cruise active (5860)",
          "[ExternalSim][Cruise]") {
    ExternalSimConnector c;
    c.DebugInjectU8(5860, 1u);   // engage
    CHECK(c.GetPimCruiseActive());
    c.DebugInjectU8(5860, 0u);   // disengage
    CHECK(c.HasReceivedPimCruiseActive());
    CHECK_FALSE(c.GetPimCruiseActive());
}

TEST_CASE("PIM cruise subscription: DebugInjectFloat delivers setpoint (5861)",
          "[ExternalSim][Cruise]") {
    ExternalSimConnector c;
    // 5861 = kSigPimCruiseSetpointMps, float32.
    c.DebugInjectFloat(5861, 23.5f);
    CHECK(c.HasReceivedPimCruiseSetpointMps());
    CHECK_THAT(c.GetPimCruiseSetpointMps(), WithinAbs(23.5f, 0.001f));
    // Active flag still not received.
    CHECK_FALSE(c.HasReceivedPimCruiseActive());
}

TEST_CASE("PIM cruise subscription: setpoint zero delivered correctly",
          "[ExternalSim][Cruise]") {
    ExternalSimConnector c;
    c.DebugInjectFloat(5861, 0.0f);
    CHECK(c.HasReceivedPimCruiseSetpointMps());
    CHECK_THAT(c.GetPimCruiseSetpointMps(), WithinAbs(0.0f, 0.0001f));
}

TEST_CASE("PIM cruise subscription: 5860/5861 not in endpoint table (main harness bus)",
          "[ExternalSim][Cruise]") {
    // Cruise signals live on the main harness bus, same as ABS/RSA — they are
    // subscribed from main_transport but NOT registered as chassis-bus endpoints.
    CHECK(ExternalSimConnector::FindEndpoint(5860) == nullptr);
    CHECK(ExternalSimConnector::FindEndpoint(5861) == nullptr);
}

// ---------------------------------------------------------------------------
// IPC trip distance subscription (chassis bus 4132)
// ---------------------------------------------------------------------------

TEST_CASE("IPC trip distance subscription: default never-received sentinel",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.HasReceivedIpcTripDistance());
    // Default sentinel is -1.0f (not 0.0) so the caller can distinguish
    // "never received" from "trip just reset to zero".
    CHECK_THAT(c.GetIpcTripDistanceM(), WithinAbs(-1.0f, 0.001f));
}

TEST_CASE("IPC trip distance subscription: DebugInjectFloat delivers distance (4132)",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    // 4132 = kSigChassisIpcTripDistanceM.
    c.DebugInjectFloat(4132, 12300.0f);  // 12.3 km
    CHECK(c.HasReceivedIpcTripDistance());
    CHECK_THAT(c.GetIpcTripDistanceM(), WithinAbs(12300.0f, 0.01f));
}

TEST_CASE("IPC trip distance subscription: DebugInjectFloat delivers zero after reset (4132)",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    c.DebugInjectFloat(4132, 5000.0f);
    CHECK_THAT(c.GetIpcTripDistanceM(), WithinAbs(5000.0f, 0.01f));
    // Simulate a trip-reset: IPC publishes 0.0.
    c.DebugInjectFloat(4132, 0.0f);
    CHECK(c.HasReceivedIpcTripDistance());
    CHECK_THAT(c.GetIpcTripDistanceM(), WithinAbs(0.0f, 0.001f));
}

TEST_CASE("IPC trip distance subscription: FindEndpoint returns correct metadata (4132)",
          "[ExternalSim][IPC]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4132);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.ipc.trip_distance_m");
    CHECK(std::string(ep->short_name)     == "ipc_trip_distance_m");
    CHECK(ep->input_to_sim);   // IPC publishes → ev1sim subscribes
}

// ---------------------------------------------------------------------------
// IPC BTCM / airbag telltale subscription (chassis bus 4134–4138)
// ---------------------------------------------------------------------------

TEST_CASE("IPC BTCM telltales: default never-received / off",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.HasReceivedIpcBrakeTelltale());
    CHECK_FALSE(c.GetIpcBrakeTelltale());
    CHECK_FALSE(c.HasReceivedIpcParkBrakeTelltale());
    CHECK_FALSE(c.GetIpcParkBrakeTelltale());
    CHECK_FALSE(c.HasReceivedIpcAntilockTelltale());
    CHECK_FALSE(c.GetIpcAntilockTelltale());
    CHECK_FALSE(c.HasReceivedIpcLowTracTelltale());
    CHECK_FALSE(c.GetIpcLowTracTelltale());
    CHECK_FALSE(c.HasReceivedIpcAirBagTelltale());
    CHECK_FALSE(c.GetIpcAirBagTelltale());
}

TEST_CASE("IPC brake telltale: DebugInjectU8 turns lamp on/off (4134)",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4134, 1u);  // 4134 = kSigChassisIpcBrakeTelltale, 1=lamp on
    CHECK(c.HasReceivedIpcBrakeTelltale());
    CHECK(c.GetIpcBrakeTelltale());
    // Others unaffected.
    CHECK_FALSE(c.HasReceivedIpcParkBrakeTelltale());
    CHECK_FALSE(c.HasReceivedIpcAntilockTelltale());
    // Turn off.
    c.DebugInjectU8(4134, 0u);
    CHECK(c.HasReceivedIpcBrakeTelltale());
    CHECK_FALSE(c.GetIpcBrakeTelltale());
}

TEST_CASE("IPC park brake telltale: DebugInjectU8 turns lamp on/off (4135)",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4135, 1u);  // 4135 = kSigChassisIpcParkBrakeTelltale
    CHECK(c.HasReceivedIpcParkBrakeTelltale());
    CHECK(c.GetIpcParkBrakeTelltale());
    CHECK_FALSE(c.HasReceivedIpcBrakeTelltale());
    c.DebugInjectU8(4135, 0u);
    CHECK_FALSE(c.GetIpcParkBrakeTelltale());
}

TEST_CASE("IPC antilock telltale: DebugInjectU8 turns lamp on/off (4136)",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4136, 1u);  // 4136 = kSigChassisIpcAntilockTelltale
    CHECK(c.HasReceivedIpcAntilockTelltale());
    CHECK(c.GetIpcAntilockTelltale());
    CHECK_FALSE(c.HasReceivedIpcLowTracTelltale());
    c.DebugInjectU8(4136, 0u);
    CHECK_FALSE(c.GetIpcAntilockTelltale());
}

TEST_CASE("IPC low-trac telltale: DebugInjectU8 turns lamp on/off (4137)",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4137, 1u);  // 4137 = kSigChassisIpcLowTracTelltale
    CHECK(c.HasReceivedIpcLowTracTelltale());
    CHECK(c.GetIpcLowTracTelltale());
    CHECK_FALSE(c.HasReceivedIpcAirBagTelltale());
    c.DebugInjectU8(4137, 0u);
    CHECK_FALSE(c.GetIpcLowTracTelltale());
}

TEST_CASE("IPC airbag telltale: DebugInjectU8 turns lamp on/off (4138)",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4138, 1u);  // 4138 = kSigChassisIpcAirBagTelltale
    CHECK(c.HasReceivedIpcAirBagTelltale());
    CHECK(c.GetIpcAirBagTelltale());
    CHECK_FALSE(c.HasReceivedIpcBrakeTelltale());
    c.DebugInjectU8(4138, 0u);
    CHECK_FALSE(c.GetIpcAirBagTelltale());
}

TEST_CASE("IPC BTCM telltales: all five turn on independently",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4134, 1u);
    c.DebugInjectU8(4135, 1u);
    c.DebugInjectU8(4136, 1u);
    c.DebugInjectU8(4137, 1u);
    c.DebugInjectU8(4138, 1u);
    CHECK(c.GetIpcBrakeTelltale());
    CHECK(c.GetIpcParkBrakeTelltale());
    CHECK(c.GetIpcAntilockTelltale());
    CHECK(c.GetIpcLowTracTelltale());
    CHECK(c.GetIpcAirBagTelltale());
}

TEST_CASE("IPC brake telltale: FindEndpoint returns correct metadata (4134)",
          "[ExternalSim][IPC]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4134);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.ipc.brake_telltale");
    CHECK(std::string(ep->short_name)     == "ipc_brake_telltale");
    CHECK(ep->input_to_sim);
}

TEST_CASE("IPC park brake telltale: FindEndpoint returns correct metadata (4135)",
          "[ExternalSim][IPC]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4135);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.ipc.park_brake_telltale");
    CHECK(std::string(ep->short_name)     == "ipc_park_brake_telltale");
    CHECK(ep->input_to_sim);
}

TEST_CASE("IPC antilock telltale: FindEndpoint returns correct metadata (4136)",
          "[ExternalSim][IPC]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4136);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.ipc.antilock_telltale");
    CHECK(std::string(ep->short_name)     == "ipc_antilock_telltale");
    CHECK(ep->input_to_sim);
}

TEST_CASE("IPC low-trac telltale: FindEndpoint returns correct metadata (4137)",
          "[ExternalSim][IPC]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4137);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.ipc.low_trac_telltale");
    CHECK(std::string(ep->short_name)     == "ipc_low_trac_telltale");
    CHECK(ep->input_to_sim);
}

TEST_CASE("IPC airbag telltale: FindEndpoint returns correct metadata (4138)",
          "[ExternalSim][IPC]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4138);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.ipc.air_bag_telltale");
    CHECK(std::string(ep->short_name)     == "ipc_air_bag_telltale");
    CHECK(ep->input_to_sim);
}

// ---------------------------------------------------------------------------
// IPC extra LCD telltale subscriber tests (signals 4140-4145)
// ---------------------------------------------------------------------------

TEST_CASE("IPC service-now telltale: default is false / not received",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.GetIpcServiceNowTelltale());
    CHECK_FALSE(c.HasReceivedIpcServiceNowTelltale());
}

TEST_CASE("IPC service-now telltale: inject ON then OFF",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4140, 1u);
    CHECK(c.GetIpcServiceNowTelltale());
    CHECK(c.HasReceivedIpcServiceNowTelltale());
    c.DebugInjectU8(4140, 0u);
    CHECK_FALSE(c.GetIpcServiceNowTelltale());
    CHECK(c.HasReceivedIpcServiceNowTelltale());
}

TEST_CASE("IPC service-now telltale: FindEndpoint returns correct metadata (4140)",
          "[ExternalSim][IPC]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4140);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.ipc.service_now_telltale");
    CHECK(std::string(ep->short_name)     == "ipc_service_now_telltale");
    CHECK(ep->input_to_sim);
}

TEST_CASE("IPC check-messages telltale: default is false / not received",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.GetIpcCheckMessagesTelltale());
    CHECK_FALSE(c.HasReceivedIpcCheckMessagesTelltale());
}

TEST_CASE("IPC check-messages telltale: inject ON then OFF",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4141, 1u);
    CHECK(c.GetIpcCheckMessagesTelltale());
    CHECK(c.HasReceivedIpcCheckMessagesTelltale());
    c.DebugInjectU8(4141, 0u);
    CHECK_FALSE(c.GetIpcCheckMessagesTelltale());
    CHECK(c.HasReceivedIpcCheckMessagesTelltale());
}

TEST_CASE("IPC check-messages telltale: FindEndpoint returns correct metadata (4141)",
          "[ExternalSim][IPC]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4141);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.ipc.check_messages_telltale");
    CHECK(std::string(ep->short_name)     == "ipc_check_messages_telltale");
    CHECK(ep->input_to_sim);
}

TEST_CASE("IPC temp telltale: default is false / not received",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.GetIpcTempTelltale());
    CHECK_FALSE(c.HasReceivedIpcTempTelltale());
}

TEST_CASE("IPC temp telltale: inject ON then OFF",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4142, 1u);
    CHECK(c.GetIpcTempTelltale());
    CHECK(c.HasReceivedIpcTempTelltale());
    c.DebugInjectU8(4142, 0u);
    CHECK_FALSE(c.GetIpcTempTelltale());
    CHECK(c.HasReceivedIpcTempTelltale());
}

TEST_CASE("IPC temp telltale: FindEndpoint returns correct metadata (4142)",
          "[ExternalSim][IPC]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4142);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.ipc.temp_telltale");
    CHECK(std::string(ep->short_name)     == "ipc_temp_telltale");
    CHECK(ep->input_to_sim);
}

TEST_CASE("IPC battery-life telltale: default is false / not received",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.GetIpcBatteryLifeTelltale());
    CHECK_FALSE(c.HasReceivedIpcBatteryLifeTelltale());
}

TEST_CASE("IPC battery-life telltale: inject ON then OFF",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4143, 1u);
    CHECK(c.GetIpcBatteryLifeTelltale());
    CHECK(c.HasReceivedIpcBatteryLifeTelltale());
    c.DebugInjectU8(4143, 0u);
    CHECK_FALSE(c.GetIpcBatteryLifeTelltale());
    CHECK(c.HasReceivedIpcBatteryLifeTelltale());
}

TEST_CASE("IPC battery-life telltale: FindEndpoint returns correct metadata (4143)",
          "[ExternalSim][IPC]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4143);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.ipc.battery_life_telltale");
    CHECK(std::string(ep->short_name)     == "ipc_battery_life_telltale");
    CHECK(ep->input_to_sim);
}

TEST_CASE("IPC reduced-perf telltale: default is false / not received",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.GetIpcReducedPerfTelltale());
    CHECK_FALSE(c.HasReceivedIpcReducedPerfTelltale());
}

TEST_CASE("IPC reduced-perf telltale: inject ON then OFF",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4144, 1u);
    CHECK(c.GetIpcReducedPerfTelltale());
    CHECK(c.HasReceivedIpcReducedPerfTelltale());
    c.DebugInjectU8(4144, 0u);
    CHECK_FALSE(c.GetIpcReducedPerfTelltale());
    CHECK(c.HasReceivedIpcReducedPerfTelltale());
}

TEST_CASE("IPC reduced-perf telltale: FindEndpoint returns correct metadata (4144)",
          "[ExternalSim][IPC]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4144);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.ipc.reduced_perf_telltale");
    CHECK(std::string(ep->short_name)     == "ipc_reduced_perf_telltale");
    CHECK(ep->input_to_sim);
}

TEST_CASE("IPC check-tire-press telltale: default is false / not received",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.GetIpcCheckTirePressTelltale());
    CHECK_FALSE(c.HasReceivedIpcCheckTirePressTelltale());
}

TEST_CASE("IPC check-tire-press telltale: inject ON then OFF",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4145, 1u);
    CHECK(c.GetIpcCheckTirePressTelltale());
    CHECK(c.HasReceivedIpcCheckTirePressTelltale());
    c.DebugInjectU8(4145, 0u);
    CHECK_FALSE(c.GetIpcCheckTirePressTelltale());
    CHECK(c.HasReceivedIpcCheckTirePressTelltale());
}

TEST_CASE("IPC check-tire-press telltale: FindEndpoint returns correct metadata (4145)",
          "[ExternalSim][IPC]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4145);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.ipc.check_tire_press_telltale");
    CHECK(std::string(ep->short_name)     == "ipc_check_tire_press_telltale");
    CHECK(ep->input_to_sim);
}

TEST_CASE("IPC extra telltales: all six ON simultaneously (4140-4145)",
          "[ExternalSim][IPC]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4140, 1u);
    c.DebugInjectU8(4141, 1u);
    c.DebugInjectU8(4142, 1u);
    c.DebugInjectU8(4143, 1u);
    c.DebugInjectU8(4144, 1u);
    c.DebugInjectU8(4145, 1u);
    CHECK(c.GetIpcServiceNowTelltale());
    CHECK(c.GetIpcCheckMessagesTelltale());
    CHECK(c.GetIpcTempTelltale());
    CHECK(c.GetIpcBatteryLifeTelltale());
    CHECK(c.GetIpcReducedPerfTelltale());
    CHECK(c.GetIpcCheckTirePressTelltale());
    CHECK(c.HasReceivedIpcServiceNowTelltale());
    CHECK(c.HasReceivedIpcCheckMessagesTelltale());
    CHECK(c.HasReceivedIpcTempTelltale());
    CHECK(c.HasReceivedIpcBatteryLifeTelltale());
    CHECK(c.HasReceivedIpcReducedPerfTelltale());
    CHECK(c.HasReceivedIpcCheckTirePressTelltale());
}

// ---------------------------------------------------------------------------
// BPM pack voltage (ID 4139, chassis segment) — new in this round.
// ---------------------------------------------------------------------------

TEST_CASE("BPM pack voltage: default is 0 / not received",
          "[ExternalSim][BPM]") {
    ExternalSimConnector c;
    CHECK(c.GetBpmPackVoltageMv() == 0u);
    CHECK_FALSE(c.HasReceivedBpmPackVoltage());
}

TEST_CASE("BPM pack voltage: inject nominal 312 V = 312000 mV",
          "[ExternalSim][BPM]") {
    ExternalSimConnector c;
    c.DebugInjectU32(4139u, 312000u);
    CHECK(c.GetBpmPackVoltageMv() == 312000u);
    CHECK(c.HasReceivedBpmPackVoltage());
}

TEST_CASE("BPM pack voltage: inject then update to new value",
          "[ExternalSim][BPM]") {
    ExternalSimConnector c;
    c.DebugInjectU32(4139u, 300000u);
    CHECK(c.GetBpmPackVoltageMv() == 300000u);
    c.DebugInjectU32(4139u, 285500u);
    CHECK(c.GetBpmPackVoltageMv() == 285500u);
    CHECK(c.HasReceivedBpmPackVoltage());
}

TEST_CASE("BPM pack voltage: FindEndpoint returns correct metadata (4139)",
          "[ExternalSim][BPM]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4139u);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.bpm.pack_voltage_mv");
    CHECK(std::string(ep->short_name)     == "bpm_pack_voltage_mv");
    CHECK(ep->input_to_sim);
}

// ---------------------------------------------------------------------------
// Vehicle speed accessor (derived from SetVehicleState, ID 4100 published out).
// ---------------------------------------------------------------------------

TEST_CASE("GetVehicleSpeedMps: returns -1 and HasVehicleSpeed false before SetVehicleState",
          "[ExternalSim][Speed]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.HasVehicleSpeed());
    CHECK(c.GetVehicleSpeedMps() == -1.0f);
}

TEST_CASE("GetVehicleSpeedMps: returns speed after SetVehicleState",
          "[ExternalSim][Speed]") {
    ExternalSimConnector c;
    VehicleState vs;
    vs.speed_mps = 27.78;  // ~100 km/h
    c.SetVehicleState(vs);
    CHECK(c.HasVehicleSpeed());
    // Use approximate comparison — float conversion from double loses precision.
    const float got = c.GetVehicleSpeedMps();
    CHECK(got > 27.0f);
    CHECK(got < 28.0f);
}
