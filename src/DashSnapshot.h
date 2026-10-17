#pragma once

#include <cstdint>
#include <optional>

// ---------------------------------------------------------------------------
// DashSnapshot — what the instrument cluster can show, decoded off the bus.
//
// Phase 1 of docs/ipc_rsa_display_plan.md: the dashboard is fed by snooping
// the GM-8192 application frames the modules already broadcast (BusSnoop),
// not by one chassis cell per display value. Each member group below is ONE
// frame; a group is nullopt until that frame has been decoded and again once
// it goes stale, so the panel shows "---" instead of a frozen last value.
//
// Field layouts are the external sim's frame contracts (frame spec
// §9.4.x via redux bus/messages/uart/*.yaml), cited per group. Units are the
// wire's own engineering units after its documented scaling; nothing here is
// re-derived from physics.
// ---------------------------------------------------------------------------

namespace ev1sim {

// PRND Mode Request, $41 byte [3] bits 3-7. Only five 5-bit codes are
// defined; anything else is kUnknown (the cluster shows no gear).
enum class DashGear : std::uint8_t {
    kError,
    kPark,
    kReverse,
    kNeutral,
    kDrive,
    kUnknown,
};

// $41 PCM Data Response (PIM → bus, 120 ms), N = 7.
// Contract: the external sim's ev1/pim/pim_uart_frame.h.
struct DashPcmStatus {
    std::uint8_t speed_kph = 0;         // wire [6], 1 km/h/count, 0..162
    DashGear     gear = DashGear::kUnknown;
    std::uint8_t prnd_code = 0;         // raw 5-bit code (kept for diagnostics)
    bool service_soon_tt = false;       // [2] b0
    bool reduced_perf_tt = false;       // [2] b1
    bool pcm_temp_tt = false;           // [2] b2
    bool charge_mode_display = false;   // [2] b4
    bool pcm_service_now_tt = false;    // [3] b1
    bool pcm_connection_detect = false; // [3] b2
};

// $40 PCM Data Poll (BPM → PCM, 120 ms), N = 6 — carries the pack readout.
// Contract: the external sim's ev1/bpm/bpm_uart_frame.h.
struct DashPackStatus {
    std::uint16_t pack_voltage_v = 0;   // wire [2], 2 V/count
    float pack_current_a = 0.0f;        // [3..4] bits 10-0, 0.5 A/count − 400 A
    bool charger_connected = false;     // [3..4] b15 BPM Connector Detect
    bool pack_malfunction = false;      // b14
    bool pack_voltage_fault = false;    // b13
    bool pack_current_fault = false;    // b12
    bool auto_disconnect_open = false;  // b11
    std::uint8_t pack_soc_pct = 0;      // wire [6], %, 0..100
};

// $20 DSCM Data Poll (BPM → IPC, 960 ms), N = 5 — the cluster-display push.
// Contract: the external sim's ev1/bpm/bpm_uart_frame.h.
struct DashBpmDisplay {
    std::uint8_t user_soc_pct = 0;      // wire [2], 1 %/count
    std::uint8_t ttf_hours = 0;         // wire [3], time to full charge
    std::uint8_t ttf_minutes = 0;       // wire [4]
    bool battery_life_tt = false;       // [5..6] b15 Pack Damage ("Battery Life")
    bool bpm_temp_tt = false;           // b14
    bool bpm_service_now_tt = false;    // b13
    std::uint8_t power_capability = 0;  // b12-10, level 0 (none) .. 7 (max)
    std::uint16_t range_km = 0;         // b9-0, 1 km/count, 0..999
};

// $91 BTCM Data Response (BTCM → bus, 120 ms), N = 11 — brake telltales.
// Contract: the external sim's ev1/btcm/btcm_uart_frame.h.
struct DashBrakeStatus {
    bool park_brake = false;            // [2] b0
    bool service_brake = false;         // [2] b1
    bool brake_tt = false;              // [2] b2 Service Brake Telltale
    bool antilock_tt = false;           // [2] b3 Anti-Lock Failure Telltale
    bool abs_tcs_active = false;        // [2] b5
};

// $21 DSCM Data Response (IPC → BPM, 960 ms), N = 3 — the cluster's own odometer.
// Contract: the external sim's ev1/ipc/ipc_uart_frame.h.
struct DashOdometer {
    std::uint32_t season_odometer_dkm = 0;  // 22-bit, 0.1 km/count
    bool odometer_fault = false;            // [2] b7
    bool bar_gage_power_use = false;        // [2] b6 (true = Power Use, false = SOC)
};

// The IPC's own lamp outputs. These are NOT decoded from frames: the IPC
// combines requests from several modules into each lamp, so the lamps' truth is
// the IPC lamp cells ev1sim already consumes (chassis 4130-4145, read through
// ExternalSimConnector). Whoever builds the snapshot copies them in; BusSnoop
// leaves this group empty. One source per lamp, as the plan's risk list asks.
// Each lamp is nullopt until its cell has been written.
struct DashLamps {
    std::optional<bool> seatbelt_driver;     // 4130
    std::optional<bool> seatbelt_passenger;  // 4131
    std::optional<bool> brake;               // 4134
    std::optional<bool> park_brake;          // 4135
    std::optional<bool> antilock;            // 4136
    std::optional<bool> low_traction;        // 4137
    std::optional<bool> airbag;              // 4138
    std::optional<bool> service_now;         // 4140
    std::optional<bool> check_messages;      // 4141
    std::optional<bool> temp;                // 4142
    std::optional<bool> battery_life;        // 4143
    std::optional<bool> reduced_perf;        // 4144
    std::optional<bool> check_tire;          // 4145
    std::optional<bool> service_soon;        // CHASSIS_IPC_SERVICE_SOON_TELLTALE
    std::optional<bool> wait;                // CHASSIS_IPC_WAIT_TELLTALE
};

// The RSA console's outputs (docs/ipc_rsa_display_plan.md Phase 2). Like the
// lamps these are the RSA's own output cells, not a frame: the same cells the
// external sim's wire-mode RSA panel reads (examples/ev1_rsa_panel/
// wire_panel_sink.cpp), so the two front-ends show the same state. Each is
// nullopt until its cell has been written.
struct DashRsaConsole {
    std::optional<bool> run;            // RSA_RUN1_OUT
    std::optional<bool> acc;            // CHASSIS_RSA_ACC_OUT
    std::optional<bool> valid_code;     // CHASSIS_RSA_VALID_CODE
    std::optional<bool> security;       // CHASSIS_RSA_SECURITY_INDICATOR
    std::optional<bool> shift_blocked;  // CHASSIS_RSA_SHIFT_BLOCKED
    std::optional<bool> btsi_solenoid;  // CHASSIS_RSA_BTSI_SOLENOID
    std::optional<bool> led_park;       // CHASSIS_RSA_SELECTOR_LED_PARK
    std::optional<bool> led_reverse;    // CHASSIS_RSA_SELECTOR_LED_REVERSE
    std::optional<bool> led_neutral;    // CHASSIS_RSA_SELECTOR_LED_NEUTRAL
    std::optional<bool> led_drive;      // CHASSIS_RSA_SELECTOR_LED_DRIVE
    std::optional<std::uint8_t> dim_step;  // CHASSIS_RSA_DIM_LEVEL_STEP, 0..10
};

struct DashSnapshot {
    std::optional<DashPcmStatus>  pcm;         // $41
    std::optional<DashPackStatus> pack;        // $40
    std::optional<DashBpmDisplay> display;     // $20
    std::optional<std::uint8_t>   power_bars;  // $90 bargraph, 0..11 lit from the bottom
    std::optional<DashBrakeStatus> brake;      // $91
    std::optional<DashOdometer>   odometer;    // $21
    std::optional<DashLamps>      lamps;       // IPC lamp cells, not a frame
    std::optional<float>          trip_m;      // IPC trip cell 4132 (metres), not a frame
    std::optional<DashRsaConsole> rsa;         // RSA output cells, not a frame
};

}  // namespace ev1sim
