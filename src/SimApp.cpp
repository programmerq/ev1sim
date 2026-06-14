#include "SimApp.h"
#include "BrakeDrum.h"
#include "MacOSPlatform.h"

#include "chrono_vehicle/ChEngine.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <ctime>
#include <iostream>
#include <thread>

using namespace chrono;
using namespace chrono::vehicle;

namespace {
// Headless-only: flipped by SIGINT so the run loop can exit cleanly.
// Static lifetime is fine — only SimApp::RunHeadless installs/reads it.
std::atomic<bool> g_stop_requested{false};

extern "C" void HeadlessSigintHandler(int) {
    g_stop_requested.store(true, std::memory_order_relaxed);
}

// Cross-platform SIGINT install/restore.  POSIX uses sigaction so the previous
// disposition can be restored exactly; MSVC's <csignal> only offers signal(),
// which is sufficient for a Ctrl-C-to-quit handler.
#ifdef _WIN32
using SigintPrev = void (*)(int);
inline SigintPrev InstallSigint(void (*handler)(int)) {
    return std::signal(SIGINT, handler);
}
inline void RestoreSigint(SigintPrev prev) { std::signal(SIGINT, prev); }
#else
using SigintPrev = struct sigaction;
inline SigintPrev InstallSigint(void (*handler)(int)) {
    struct sigaction new_sa{}, old_sa{};
    new_sa.sa_handler = handler;
    sigemptyset(&new_sa.sa_mask);
    sigaction(SIGINT, &new_sa, &old_sa);
    return old_sa;
}
inline void RestoreSigint(SigintPrev prev) { sigaction(SIGINT, &prev, nullptr); }
#endif
}  // namespace

// ---------------------------------------------------------------------------
SimApp::SimApp(const Config& config) : m_config(config) {
    const bool headless = m_config.simulation.headless;

    // Vehicle dynamics authority (config knob).  Validated already in
    // Config::ApplyCliOverrides; defensively normalize unknown values to
    // "local" so a typo never wedges the car.
    m_driver_mode = m_config.vehicle_dynamics.driver;
    if (m_driver_mode != "local" && m_driver_mode != "electronics") {
        std::cerr << "[SimApp] unknown vehicle_dynamics.driver='"
                  << m_driver_mode << "' — falling back to 'local'\n";
        m_driver_mode = "local";
    }
    m_throttle_freshness_window = std::chrono::milliseconds(
        static_cast<int>(m_config.vehicle_dynamics.throttle_freshness_window_ms));
    if (m_driver_mode == "electronics") {
        std::cout << "[SimApp] Vehicle driver = electronics "
                     "(throttle from PIM via kSigChassisThrottleCmdQ8 4073, "
                     "fallback window "
                  << m_config.vehicle_dynamics.throttle_freshness_window_ms
                  << " ms)\n";
    }
    // Bypass the RSA propulsion gate when explicitly opted in (scenario
    // smoke tests / standalone runs without electricsim).  Default behavior
    // requires RSA to broadcast RUN before propulsion engages.
    if (m_config.vehicle_dynamics.start_propulsion_enabled) {
        m_propulsion_enabled = true;
        std::cout << "[SimApp] Propulsion gate bypassed at startup "
                     "(vehicle_dynamics.start_propulsion_enabled=true)\n";
    }

    // 1. Physics world.
    m_world = std::make_unique<VehicleWorld>(m_config);

    // 2. Keyboard input — only when we have an Irrlicht window to attach to.
    if (!headless) {
        KeyboardInputController::Rates rates;
        rates.steer_rate        = m_config.input.steer_rate;
        rates.steer_return_rate = m_config.input.steer_return_rate;
        rates.throttle_rise     = m_config.input.throttle_rise_rate;
        rates.brake_rise        = m_config.input.brake_rise_rate;
        m_keyboard = std::make_unique<KeyboardInputController>(rates);
    }

#ifdef EV1SIM_HAVE_WHEEL_IO
    // SDL3 wheel + force feedback — only when enabled in the bindings config and
    // a window is present.  Degrades silently to keyboard if SDL init or device
    // open fails (important for the no-hardware / CI case).
    if (!headless && m_config.input.wheel.enabled) {
        m_sdl = std::make_unique<ev1sim::SdlContext>();
        if (m_sdl->ok()) {
            m_wheel = std::make_unique<ev1sim::WheelInputController>(
                m_sdl.get(), m_config.input.wheel);
        }
        // ForceFeedback is attached lazily in the render loop once a wheel is
        // actually present, so it works even when the wheel is hot-plugged
        // after startup (and is dropped + re-attached across disconnects).
    }
#endif

    // 3. Telemetry.
    m_telemetry = std::make_unique<Telemetry>(
        m_config.telemetry.log_rate_hz,
        m_config.telemetry.log_to_file,
        m_config.telemetry.log_file,
        m_config.telemetry.show_hud);

    // 4. Horn audio — CoreAudio on macOS, no-op elsewhere.  Safe headless.
    m_horn = std::make_unique<HornAudio>();
    m_sounder_audio = std::make_unique<SounderAudio>();

    // 4b. Physical-world inputs — constructed early (before visualization) so
    //     floating-UI panel lambdas can safely dereference m_physical during
    //     their initial label evaluation in AddButton().
    m_physical = std::make_unique<ev1sim::PhysicalWorld>();
    // Optional initial-state config: start with the cabin locked (DoorLocks
    // otherwise defaults UNLOCKED).  Applied once, here at init.
    if (m_config.body.door_locks_locked_at_start)
        m_physical->door_locks().lock_all();

    // 4c. Vehicle panels (hood, trunk, doors) — constructed early, alongside
    //     m_physical and for the same reason: a floating-UI label lambda (the
    //     hood-state status row) dereferences m_panels during its initial label
    //     evaluation in AddButton(), which runs inside SetupVisualization()
    //     below.  Built here it is non-null by then.  Pure state until panel
    //     OBJs exist — no Irrlicht dependency at construction, safe headless.
    m_panels = std::make_unique<VehiclePanels>();

    // 5. Visualization (creates window).  Skipped entirely in headless mode;
    //    no Irrlicht device, no window, no OpenGL context.
    if (!headless) {
        SetupVisualization();

        // 6. Camera manager — needs the Irrlicht camera node from vis.
        auto* cam_node = m_vis->GetActiveCamera();
        m_camera = std::make_unique<CameraManager>(
            cam_node, m_config.camera.chase_distance, m_config.camera.chase_height);
        m_camera->SetModeFromString(m_config.camera.default_mode);

        // Register camera manager's mouse handler with Irrlicht.
        m_vis->AddUserEventReceiver(m_camera.get());

        // 7. Vehicle lights — needs Irrlicht scene graph to be populated.
        m_lights = std::make_unique<VehicleLights>();

        // 7b. Floating UI panel — anchored top-left, below the banner area.
        //     All button callbacks are registered here; the panel starts hidden.
        //     The panel is shown/hidden via TAB (UI mode toggle).
        {
            auto* gui = m_vis->GetDevice()->getGUIEnvironment();
            // Offset 10 px from left, 10 px from top.  Panel is 220 px wide.
            m_floating_ui = std::make_unique<FloatingUiPanel>(gui, 10, 10, 220, 22);

            // Register as user event receiver so GUI clicks fire our callbacks.
            m_vis->AddUserEventReceiver(m_floating_ui.get());

            // --- Hazard toggle ---
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    return FormatHazardLabel(m_physical->hazard_switch().on());
                },
                [this]() {
                    m_physical->hazard_switch().toggle();
                    std::cout << "[UI] Hazard: "
                              << (m_physical->hazard_switch().on() ? "ON" : "OFF") << "\n";
                });

            // --- Door locks: Lock All / Unlock All (one button, label flips) ---
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    return FormatLockAllLabel(m_physical->door_locks().any_locked());
                },
                [this]() {
                    if (m_physical->door_locks().any_locked())
                        m_physical->door_locks().unlock_all();
                    else
                        m_physical->door_locks().lock_all();
                    std::cout << "[UI] Doors: "
                              << (m_physical->door_locks().any_locked()
                                      ? "some locked" : "all unlocked") << "\n";
                });

            // --- Door locks: Driver ---
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    return FormatDoorLabel(L"Driver",
                        m_physical->door_locks().driver() == ev1sim::DoorLocks::State::LOCKED);
                },
                [this]() {
                    m_physical->door_locks().toggle_driver();
                    using S = ev1sim::DoorLocks::State;
                    std::cout << "[UI] Driver door: "
                              << (m_physical->door_locks().driver() == S::LOCKED
                                      ? "locked" : "unlocked") << "\n";
                });

            // --- Door locks: Passenger ---
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    return FormatDoorLabel(L"Passenger",
                        m_physical->door_locks().passenger() == ev1sim::DoorLocks::State::LOCKED);
                },
                [this]() {
                    m_physical->door_locks().toggle_passenger();
                    using S = ev1sim::DoorLocks::State;
                    std::cout << "[UI] Passenger door: "
                              << (m_physical->door_locks().passenger() == S::LOCKED
                                      ? "locked" : "unlocked") << "\n";
                });

            // --- Door locks: Trunk ---
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    return FormatDoorLabel(L"Trunk",
                        m_physical->door_locks().trunk() == ev1sim::DoorLocks::State::LOCKED);
                },
                [this]() {
                    m_physical->door_locks().toggle_trunk();
                    using S = ev1sim::DoorLocks::State;
                    std::cout << "[UI] Trunk: "
                              << (m_physical->door_locks().trunk() == S::LOCKED
                                      ? "locked" : "unlocked") << "\n";
                });

            // --- Charge coupler toggle ---
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    return FormatCouplerLabel(m_physical->charge_coupler().present());
                },
                [this]() {
                    m_physical->charge_coupler().set_present(
                        !m_physical->charge_coupler().present());
                    std::cout << "[UI] Charge coupler: "
                              << (m_physical->charge_coupler().present()
                                      ? "PLUGGED" : "UNPLUGGED") << "\n";
                });

            // --- Exterior Keypad section: 5 buttons (tap = lower digit) ---
            // Button labels are static; clicking sends a tap (value=1).
            for (int ki = 0; ki < 5; ++ki) {
                m_floating_ui->AddButton(
                    [ki]() -> std::wstring {
                        return FormatExtKeypadButtonLabel(ki);
                    },
                    [this, ki]() {
                        m_physical->rsa_exterior_keypad().press_button(ki, /*long_press=*/false);
                        static const char* kDigits[] = {"1","3","5","7","9"};
                        std::cout << "[UI] Ext keypad: btn" << (ki+1)
                                  << " tap (digit " << kDigits[ki] << ")\n";
                    });
            }

            // --- Exterior Keypad convenience macro: Enter "111111" ---
            m_floating_ui->AddButton(
                []() -> std::wstring { return L"[Enter \"111111\"]"; },
                [this]() {
                    m_physical->rsa_exterior_keypad().enter_code_sequence("111111");
                    std::cout << "[UI] Ext keypad: enter_code_sequence(\"111111\") queued\n";
                });

            // --- Door Handles ---
            m_floating_ui->AddButton(
                []() -> std::wstring { return FormatDoorHandleLabel(L"Driver"); },
                [this]() {
                    m_physical->door_handles().attempt_driver();
                    using S = ev1sim::DoorLocks::State;
                    if (m_physical->door_locks().driver() == S::LOCKED) {
                        std::cout << "[SimApp] Door driver: LOCKED — try keypad code\n";
                    } else {
                        m_panels->Toggle(PanelID::DOOR_LEFT);
                        std::cout << "[SimApp] Door driver: UNLOCKED — "
                                  << (m_panels->IsOpen(PanelID::DOOR_LEFT) ? "ajar" : "closed")
                                  << "\n";
                    }
                });

            m_floating_ui->AddButton(
                []() -> std::wstring { return FormatDoorHandleLabel(L"Passenger"); },
                [this]() {
                    m_physical->door_handles().attempt_passenger();
                    using S = ev1sim::DoorLocks::State;
                    if (m_physical->door_locks().passenger() == S::LOCKED) {
                        std::cout << "[SimApp] Door passenger: LOCKED — try keypad code\n";
                    } else {
                        m_panels->Toggle(PanelID::DOOR_RIGHT);
                        std::cout << "[SimApp] Door passenger: UNLOCKED — "
                                  << (m_panels->IsOpen(PanelID::DOOR_RIGHT) ? "ajar" : "closed")
                                  << "\n";
                    }
                });

            // --- Hood (two-stage latch: CLOSED -> POPPED -> OPEN) ---
            // The primary ajar sensor (6962) trips as soon as the hood pops, so
            // POPPED and OPEN are indistinguishable on the bus; this status row
            // shows the true mechanical state.  The three click actions mirror
            // the real releases (interior cable lever, then the safety catch);
            // raise is gated behind a pop, just like the real latch.
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    return FormatHoodStateLabel(
                        static_cast<int>(m_panels->hood().state()));
                },
                []() {});   // no-op: display-only status row
            m_floating_ui->AddButton(
                []() -> std::wstring { return L"Hood: Interior Release"; },
                [this]() { m_panels->hood().interior_release(); });
            m_floating_ui->AddButton(
                []() -> std::wstring { return L"Hood: Raise (safety catch)"; },
                [this]() { m_panels->hood().raise(); });
            m_floating_ui->AddButton(
                []() -> std::wstring { return L"Hood: Lower / Latch"; },
                [this]() { m_panels->hood().lower_latch(); });

            // --- Wiper: cycle position (OFF → INT → LOW → HIGH → OFF) ---
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    return FormatWiperLabel(
                        static_cast<int>(m_physical->wiper_stalk().position()));
                },
                [this]() {
                    m_physical->wiper_stalk().cycle_position();
                    const char* wiper_names[] = {"OFF", "INT", "LOW", "HIGH"};
                    int widx = static_cast<int>(m_physical->wiper_stalk().position());
                    std::cout << "[UI] Wiper: " << wiper_names[widx] << "\n";
                });

            // --- Wiper wash (momentary) ---
            m_floating_ui->AddButton(
                []() -> std::wstring { return FormatWashLabel(); },
                [this]() {
                    m_physical->wiper_stalk().press_wash();
                    std::cout << "[UI] Wiper wash\n";
                });

            // --- Cruise stalk: SET, RESUME, CANCEL, +, - ---
            m_floating_ui->AddButton(
                []() -> std::wstring { return FormatCruiseLabel(L"SET"); },
                [this]() {
                    m_physical->cruise_stalk().press_set();
                    std::cout << "[UI] Cruise: SET\n";
                });
            m_floating_ui->AddButton(
                []() -> std::wstring { return FormatCruiseLabel(L"RES"); },
                [this]() {
                    m_physical->cruise_stalk().press_resume();
                    std::cout << "[UI] Cruise: RESUME\n";
                });
            m_floating_ui->AddButton(
                []() -> std::wstring { return FormatCruiseLabel(L"CANCEL"); },
                [this]() {
                    m_physical->cruise_stalk().press_cancel();
                    std::cout << "[UI] Cruise: CANCEL\n";
                });
            m_floating_ui->AddButton(
                []() -> std::wstring { return FormatCruiseLabel(L"+"); },
                [this]() {
                    m_physical->cruise_stalk().press_speed_up();
                    std::cout << "[UI] Cruise: SPEED+\n";
                });
            m_floating_ui->AddButton(
                []() -> std::wstring { return FormatCruiseLabel(L"-"); },
                [this]() {
                    m_physical->cruise_stalk().press_speed_down();
                    std::cout << "[UI] Cruise: SPEED-\n";
                });

            // --- IPC trip-reset (momentary) ---
            m_floating_ui->AddButton(
                []() -> std::wstring { return FormatTripResetLabel(); },
                [this]() {
                    m_physical->ipc_trip_reset().press();
                    ++m_trip_reset_count;
                    std::cout << "[UI] IPC trip-reset pressed\n";
                });

            // --- Seatbelt: Driver toggle ---
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    return FormatSeatbeltLabel(L"D",
                        m_physical->seatbelts().driver_buckled());
                },
                [this]() {
                    m_physical->seatbelts().toggle_driver();
                    std::cout << "[UI] Seatbelt D: "
                              << (m_physical->seatbelts().driver_buckled()
                                      ? "BUCKLED" : "UNBUCKLED") << "\n";
                });

            // --- Seatbelt: Passenger toggle ---
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    return FormatSeatbeltLabel(L"P",
                        m_physical->seatbelts().passenger_buckled());
                },
                [this]() {
                    m_physical->seatbelts().toggle_passenger();
                    std::cout << "[UI] Seatbelt P: "
                              << (m_physical->seatbelts().passenger_buckled()
                                      ? "BUCKLED" : "UNBUCKLED") << "\n";
                });

            // --- HVAC status (display-only; no click action) ---
            // Subscribes to HTCM chassis-bus signals 4082 (blower level) and
            // 4083 (defrost grid).  Labels update each frame via UpdateLabels().
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"Blower: n/a";
                    return FormatHvacBlowerLabel(
                        m_external_sim->GetHvacBlowerLevel());
                },
                []() {});   // no-op: display-only row

            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"Defrost: n/a";
                    return FormatDefrostGridLabel(
                        m_external_sim->GetDefrostGridActive());
                },
                []() {});   // no-op: display-only row

            // --- HVAC driver controls (drive ev1sim::HvacControls; the
            //     requests are published to HTCM on chassis 4124-4128 each tick).
            //     Display row for the setpoint + buttons to adjust/cycle/toggle. ---
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    return FormatHvacSetpointLabel(
                        m_physical->hvac_controls().temp_setpoint_c());
                },
                []() {});   // display-only row (setpoint value)
            m_floating_ui->AddButton(
                []() -> std::wstring { return L"HVAC Temp +"; },
                [this]() { m_physical->hvac_controls().adjust_setpoint_c(0.5); });
            m_floating_ui->AddButton(
                []() -> std::wstring { return L"HVAC Temp -"; },
                [this]() { m_physical->hvac_controls().adjust_setpoint_c(-0.5); });
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    return FormatHvacFanLabel(m_physical->hvac_controls().fan_u8());
                },
                [this]() { m_physical->hvac_controls().cycle_fan(); });
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    return FormatHvacModeLabel(m_physical->hvac_controls().mode_u8());
                },
                [this]() { m_physical->hvac_controls().cycle_mode(); });
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    return FormatHvacAcLabel(m_physical->hvac_controls().ac_on());
                },
                [this]() { m_physical->hvac_controls().toggle_ac(); });
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    return FormatHvacDefrostLabel(m_physical->hvac_controls().defrost_on());
                },
                [this]() { m_physical->hvac_controls().toggle_defrost(); });

            // --- IPC LCD telltale status (display-only; no click action) ---
            // Subscribes to IPC chassis-bus signals 4130 (driver seatbelt telltale)
            // and 4131 (passenger seatbelt telltale).  Lamp is ON when the seat is
            // unbuckled AND vehicle speed > ~8 km/h (IPC supervisor threshold).
            // Labels update each frame via UpdateLabels().
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"Seatbelt (D): n/a";
                    return FormatIpcSeatbeltTelltaleLabel(
                        L"D",
                        m_external_sim->GetIpcSeatbeltTelltaleDriver(),
                        m_external_sim->HasReceivedIpcSeatbeltTelltaleDriver());
                },
                []() {});   // no-op: display-only row

            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"Seatbelt (P): n/a";
                    return FormatIpcSeatbeltTelltaleLabel(
                        L"P",
                        m_external_sim->GetIpcSeatbeltTelltalePassenger(),
                        m_external_sim->HasReceivedIpcSeatbeltTelltalePassenger());
                },
                []() {});   // no-op: display-only row

            // --- PRND gear selector (display-only; read from local PhysicalWorld) ---
            // Reads m_physical->prnd_selector().position() directly — ev1sim is the
            // source of truth.  No bus subscription required.
            // Shows "Gear: P / R / N / D" based on current selector position.
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    return FormatPrndGearLabel(
                        static_cast<int>(m_physical->prnd_selector().position()));
                },
                []() {});   // no-op: display-only row

            // --- PRND shift buttons (mirror the , / . keyboard shift actions) ---
            // cycle_up advances P→R→N→D; cycle_down reverses (both clamp at the
            // ends).  Click advances the local PrndSelector — the new position
            // is published via SetDriverGearSelector and shown in the row above.
            m_floating_ui->AddButton(
                []() -> std::wstring { return L"Gear Up: P>R>N>D"; },
                [this]() { PrndUp(); });
            m_floating_ui->AddButton(
                []() -> std::wstring { return L"Gear Down: D>N>R>P"; },
                [this]() { PrndDown(); });

            // --- Power windows (momentary press-and-hold; bus 6980-6983) ---
            // These panel buttons are the only driver input for the windows
            // (no keyboard source).  While a button is held the matching switch
            // signal is asserted; releasing it — or hiding the panel — returns
            // the window to NONE.  set_held() coordinates Up vs Down so the two
            // buttons for one window don't clobber each other.
            {
                using PwWin = ev1sim::PowerWindows::Window;
                using PwDir = ev1sim::PowerWindows::Direction;
                auto add_window_btn =
                    [this](const wchar_t* label, PwWin w, PwDir d) {
                        m_floating_ui->AddHoldButton(
                            [this, label, w, d]() -> std::wstring {
                                return FormatPowerWindowButtonLabel(
                                    label,
                                    m_physical->power_windows().state(w) == d);
                            },
                            [this, w, d](bool held) {
                                m_physical->power_windows().set_held(w, d, held);
                            });
                    };
                add_window_btn(L"Drv Window Up",    PwWin::DRIVER,    PwDir::UP);
                add_window_btn(L"Drv Window Down",  PwWin::DRIVER,    PwDir::DOWN);
                add_window_btn(L"Pass Window Up",   PwWin::PASSENGER, PwDir::UP);
                add_window_btn(L"Pass Window Down", PwWin::PASSENGER, PwDir::DOWN);
            }

            // --- PIM cruise-control status (display-only; main harness bus 5860/5861) ---
            // Subscribes to PIM signals kSigPimCruiseActive (5860) and
            // kSigPimCruiseSetpointMps (5861) on the main harness segment.
            // Shows "Cruise: OFF", "Cruise: 23.5 m/s ON", or "Cruise: ---" before
            // first frame arrives.  Labels update each frame via UpdateLabels().
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"Cruise: n/a";
                    return FormatPimCruiseStatusLabel(
                        m_external_sim->HasReceivedPimCruiseActive(),
                        m_external_sim->GetPimCruiseActive(),
                        m_external_sim->GetPimCruiseSetpointMps());
                },
                []() {});   // no-op: display-only row

            // --- IPC trip distance (display-only; chassis bus 4132) ---
            // Subscribes to kSigChassisIpcTripDistanceM (4132) published by IPC each tick
            // (epsilon-gated, ~0.5 m).  Converts metres to km for display.
            // Shows "Trip: 12.3 km" or "Trip: ---" before first frame arrives.
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"Trip: n/a";
                    return FormatIpcTripDistanceLabel(
                        m_external_sim->HasReceivedIpcTripDistance(),
                        m_external_sim->GetIpcTripDistanceM());
                },
                []() {});   // no-op: display-only row

            // --- IPC BTCM / airbag telltales (display-only; chassis bus 4134–4138) ---
            // Subscribes to kSigChassisIpcBrakeTelltale (4134) published by IPC each tick.
            // Driven by BTCM brake_ind (DTC 42).  Shows "Brake: ON / OFF / ---".
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"Brake: n/a";
                    return FormatIpcTelltaleLampLabel(
                        L"Brake",
                        m_external_sim->GetIpcBrakeTelltale(),
                        m_external_sim->HasReceivedIpcBrakeTelltale());
                },
                []() {});   // no-op: display-only row

            // Subscribes to kSigChassisIpcParkBrakeTelltale (4135) published by IPC.
            // Driven by BTCM park_brake_ind (DTC 44).  Shows "ParkBrake: ON / OFF / ---".
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"ParkBrake: n/a";
                    return FormatIpcTelltaleLampLabel(
                        L"ParkBrake",
                        m_external_sim->GetIpcParkBrakeTelltale(),
                        m_external_sim->HasReceivedIpcParkBrakeTelltale());
                },
                []() {});   // no-op: display-only row

            // Subscribes to kSigChassisIpcAntilockTelltale (4136) published by IPC.
            // Driven by BTCM antilock_ind (DTC 41).  Shows "ABS: ON / OFF / ---".
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"ABS: n/a";
                    return FormatIpcTelltaleLampLabel(
                        L"ABS",
                        m_external_sim->GetIpcAntilockTelltale(),
                        m_external_sim->HasReceivedIpcAntilockTelltale());
                },
                []() {});   // no-op: display-only row

            // Subscribes to kSigChassisIpcLowTracTelltale (4137) published by IPC.
            // Driven by BTCM low_trac_ind (DTC 43).  Shows "LowTrac: ON / OFF / ---".
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"LowTrac: n/a";
                    return FormatIpcTelltaleLampLabel(
                        L"LowTrac",
                        m_external_sim->GetIpcLowTracTelltale(),
                        m_external_sim->HasReceivedIpcLowTracTelltale());
                },
                []() {});   // no-op: display-only row

            // Subscribes to kSigChassisIpcAirBagTelltale (4138) published by IPC.
            // Driven by ipc_supervisor_set_airbag_input (DTC 40).
            // Shows "AirBag: ON / OFF / ---".
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"AirBag: n/a";
                    return FormatIpcTelltaleLampLabel(
                        L"AirBag",
                        m_external_sim->GetIpcAirBagTelltale(),
                        m_external_sim->HasReceivedIpcAirBagTelltale());
                },
                []() {});   // no-op: display-only row

            // --- IPC extra LCD telltales (display-only; chassis bus 4140-4145) ---
            // Subscribes to kSigChassisIpcServiceNowTelltale (4140).
            // Driven by SERVICE_NOW DTCs 31/33/35 (HTCM/PCM/BPM telltale requests).
            // Shows "Service Now: ON / OFF / ---".
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"Service Now: n/a";
                    return FormatIpcTelltaleLampLabel(
                        L"Service Now",
                        m_external_sim->GetIpcServiceNowTelltale(),
                        m_external_sim->HasReceivedIpcServiceNowTelltale());
                },
                []() {});   // no-op: display-only row

            // Subscribes to kSigChassisIpcCheckMessagesTelltale (4141).
            // Aggregate: any comm-loss DTC, any telltale-request DTC, or any BTCM indicator DTC.
            // Shows "CheckMsg: ON / OFF / ---".
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"CheckMsg: n/a";
                    return FormatIpcTelltaleLampLabel(
                        L"CheckMsg",
                        m_external_sim->GetIpcCheckMessagesTelltale(),
                        m_external_sim->HasReceivedIpcCheckMessagesTelltale());
                },
                []() {});   // no-op: display-only row

            // Subscribes to kSigChassisIpcTempTelltale (4142).
            // Driven by TEMP DTCs 32/34/36 (HTCM/PCM/BPM telltale requests).
            // Shows "Temp: ON / OFF / ---".
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"Temp: n/a";
                    return FormatIpcTelltaleLampLabel(
                        L"Temp",
                        m_external_sim->GetIpcTempTelltale(),
                        m_external_sim->HasReceivedIpcTempTelltale());
                },
                []() {});   // no-op: display-only row

            // Subscribes to kSigChassisIpcBatteryLifeTelltale (4143).
            // Driven by DTC 38 via IPC_REQ_BATTERY_LIFE_FROM_BPM.
            // Shows "BattLife: ON / OFF / ---".
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"BattLife: n/a";
                    return FormatIpcTelltaleLampLabel(
                        L"BattLife",
                        m_external_sim->GetIpcBatteryLifeTelltale(),
                        m_external_sim->HasReceivedIpcBatteryLifeTelltale());
                },
                []() {});   // no-op: display-only row

            // Subscribes to kSigChassisIpcReducedPerfTelltale (4144).
            // Driven by DTC 37 via IPC_REQ_REDUCED_PERF_FROM_PCM.
            // Shows "RedPerf: ON / OFF / ---".
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"RedPerf: n/a";
                    return FormatIpcTelltaleLampLabel(
                        L"RedPerf",
                        m_external_sim->GetIpcReducedPerfTelltale(),
                        m_external_sim->HasReceivedIpcReducedPerfTelltale());
                },
                []() {});   // no-op: display-only row

            // Subscribes to kSigChassisIpcCheckTirePressTelltale (4145).
            // Driven by DTC 39 via IPC_REQ_CHECK_TIRE_PRESS_FROM_RSA.
            // Shows "TirePress: ON / OFF / ---".
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"TirePress: n/a";
                    return FormatIpcTelltaleLampLabel(
                        L"TirePress",
                        m_external_sim->GetIpcCheckTirePressTelltale(),
                        m_external_sim->HasReceivedIpcCheckTirePressTelltale());
                },
                []() {});   // no-op: display-only row

            // --- RSA shift-blocked cue (display-only; chassis bus 4088) ---
            // Subscribes to kSigChassisRsaShiftBlocked (4088) published by RSA.
            // Shows "Shift: BRAKE TO SHIFT" when a P→non-P shift is refused,
            // "Shift: OK" when no block is active, "Shift: ---" before first frame.
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"Shift: n/a";
                    return FormatRsaShiftBlockedLabel(
                        m_external_sim->GetRsaShiftBlocked(),
                        m_external_sim->HasReceivedRsaShiftBlocked());
                },
                []() {});   // no-op: display-only row

            // --- RSA run-mode status (display-only; main harness bus 5711) ---
            // Subscribes to kSigRunModeBroadcast (5711) published by RSA each tick.
            // Shows "Mode: OFF / ACC / RUN" or "Mode: ---" before first frame arrives.
            // Note: the broadcast signal carries only 0=OFF, 1=ACC, 2=RUN — the START
            // enum from the mode-button input (6971) never appears on the broadcast.
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"Mode: n/a";
                    return FormatRsaRunModeLabel(
                        m_external_sim->GetRsaRunMode(),
                        m_external_sim->HasReceivedRunMode());
                },
                []() {});   // no-op: display-only row

            // --- Throttle percent (display-only; local DriverCommand cache) ---
            // Reads m_last_cmd.throttle — the final post-override throttle applied
            // to Chrono each tick.  No bus subscription; pure local state.
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    return FormatPedalPercentLabel("Throttle", m_last_cmd.throttle);
                },
                []() {});   // no-op: display-only row

            // --- Brake percent (display-only; local DriverCommand cache) ---
            // Reads m_last_cmd.front_brake — the front pedal travel value [0..1]
            // set by KeyboardInputController (keyboard input controller sets
            // front_brake to the driver pedal position; front/rear are identical
            // for keyboard input).  No bus subscription; pure local state.
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    return FormatPedalPercentLabel("Brake", m_last_cmd.front_brake);
                },
                []() {});   // no-op: display-only row

            // --- Headlamp status (display-only; bulb feed line cache) ---
            // Reads GetBulbCmd for low-beam bulbs (LLBH, RLBH) and high-beam
            // bulbs (LHBH, RHBH) from the electricsim bulb-feed-line cache.
            // No new bus subscription — bulb states are already received as part
            // of the 4000-block signals.  Priority: HIGH > LOW > OFF.
            // Shows "Headlamps: OFF / LOW / HIGH / ---" (--- before first bulb frame).
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"Headlamps: n/a";
                    const bool low  = m_external_sim->GetBulbCmd(LightID::LLBH)
                                   || m_external_sim->GetBulbCmd(LightID::RLBH);
                    const bool high = m_external_sim->GetBulbCmd(LightID::LHBH)
                                   || m_external_sim->GetBulbCmd(LightID::RHBH);
                    return FormatHeadlampStatusLabel(low, high,
                        m_external_sim->HasReceivedBulbData());
                },
                []() {});   // no-op: display-only row

            // --- Left turn-signal status (display-only; bulb feed line cache) ---
            // Reads GetBulbCmd for LFTS (left front turn signal) and LRTS (left
            // rear turn signal) from the electricsim bulb-feed-line cache.
            // No new bus subscription.  Turn signal flashes ~1 Hz so the label
            // alternates ON/OFF rapidly — the user can see the flash visually.
            // Shows "L Turn: ON / OFF / ---" (--- before first bulb frame).
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"L Turn: n/a";
                    const bool active = m_external_sim->GetBulbCmd(LightID::LFTS)
                                     || m_external_sim->GetBulbCmd(LightID::LRTS);
                    return FormatTurnSignalStatusLabel(L"L", active,
                        m_external_sim->HasReceivedBulbData());
                },
                []() {});   // no-op: display-only row

            // --- Right turn-signal status (display-only; bulb feed line cache) ---
            // Reads GetBulbCmd for RFTS (right front turn signal) and RRTS (right
            // rear turn signal) from the electricsim bulb-feed-line cache.
            // No new bus subscription.  Same flash-visible pattern as left.
            // Shows "R Turn: ON / OFF / ---" (--- before first bulb frame).
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"R Turn: n/a";
                    const bool active = m_external_sim->GetBulbCmd(LightID::RFTS)
                                     || m_external_sim->GetBulbCmd(LightID::RRTS);
                    return FormatTurnSignalStatusLabel(L"R", active,
                        m_external_sim->HasReceivedBulbData());
                },
                []() {});   // no-op: display-only row

            // --- Vehicle speed (display-only; local VehicleState snapshot, ID 4100) ---
            // Reads ev1sim's own published speed (kSigChassisSpeedMps = 4100) via the
            // VehicleState snapshot that SimApp writes each tick via SetVehicleState().
            // No additional bus subscription needed — ev1sim is the publisher.
            // Shows "Speed: 12.3 m/s (44 km/h)" or "Speed: ---" before first physics tick.
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"Speed: n/a";
                    return FormatVehicleSpeedLabel(
                        m_external_sim->HasVehicleSpeed(),
                        m_external_sim->GetVehicleSpeedMps());
                },
                []() {});   // no-op: display-only row

            // --- Steering angle indicator (display-only; ev1sim physics) ---
            // Front road-wheel angle from the VehicleState snapshot (rad → deg).
            // Positive = left.  Shows "Steering: 12.3 deg L/R" or "Steering: ---".
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim || !m_external_sim->HasVehicleState())
                        return L"Steering: ---";
                    const double deg = m_external_sim->GetSteeringAngleRad()
                                       * 180.0 / 3.14159265358979323846;
                    return FormatSteeringAngleLabel(deg);
                },
                []() {});   // no-op: display-only row

            // --- BPM pack voltage (display-only; chassis bus 4139) ---
            // Subscribes to kSigChassisBpmPackVoltageMv (4139) published by BPM on change
            // (epsilon ~50 mV) while key-on.  Converts uint32 mV → V for display.
            // Shows "PackVolt: 312.0 V" or "PackVolt: ---" before first BPM frame arrives.
            m_floating_ui->AddButton(
                [this]() -> std::wstring {
                    if (!m_external_sim) return L"PackVolt: n/a";
                    return FormatBpmPackVoltageLabel(
                        m_external_sim->HasReceivedBpmPackVoltage(),
                        m_external_sim->GetBpmPackVoltageMv());
                },
                []() {});   // no-op: display-only row
        }
    }

    // 8. Vehicle panels (hood, trunk, doors) are constructed earlier (step 4c,
    //    before SetupVisualization) so the floating-UI hood-state label can
    //    dereference m_panels during its initial evaluation.

    // 8b. Wiper renderer — phase-based sweep animation driven by RHJB motor command.
    m_wiper = std::make_unique<WiperRenderer>();

    // 9. External electrical-simulator connector.  Non-blocking — if the
    //    electric sim isn't running yet, the connector retries each Tick().
    ExternalSimConnector::Options ext_opts;
    ext_opts.enabled            = m_config.external_sim.enabled;
    ext_opts.bus_name           = m_config.external_sim.bus_name;
    ext_opts.reconnect_period_s = m_config.external_sim.reconnect_period_s;
    m_external_sim = std::make_unique<ExternalSimConnector>(ext_opts);
    m_external_sim->Start();
    if (ext_opts.enabled) {
        std::cout << "[SimApp] External sim: enabled, bus='"
                  << ext_opts.bus_name << "' ("
                  << m_external_sim->StatusString() << ")\n";
    }

    // Data-driven scenario harness — JSON file with timed events + stats
    // capture.  Wins over the built-in ScriptedDriver if both are configured.
    if (!m_config.scenario.path.empty()) {
        auto loaded = ev1sim::Scenario::LoadFromFile(m_config.scenario.path);
        if (loaded) {
            m_scenario = std::make_unique<ev1sim::Scenario>(std::move(*loaded));
            // Scenario file's driver_mode + max_time_s override config defaults
            // unless the config field is non-trivial (already set by the user).
            if (!m_scenario->driver_mode().empty() &&
                m_config.vehicle_dynamics.driver == "local") {
                m_driver_mode = m_scenario->driver_mode();
                if (m_driver_mode != "local" && m_driver_mode != "electronics") {
                    std::cerr << "[SimApp] scenario driver_mode='"
                              << m_driver_mode << "' invalid — using 'local'\n";
                    m_driver_mode = "local";
                }
            }
            if (m_scenario->max_time_s() > 0.0 &&
                m_config.simulation.max_time_s == 0.0) {
                m_config.simulation.max_time_s = m_scenario->max_time_s();
            }
            m_scenario->OpenStats();
            if (m_config.scripted.enabled) {
                std::cerr << "[SimApp] scenario file overrides scripted "
                             "driver — disabling scripted\n";
                m_config.scripted.enabled = false;
            }
        }
    }

    // Scripted driver (optional).  Currently one built-in scenario:
    // accel → hold → brake → done.
    if (m_config.scripted.enabled) {
        ScriptedDriver::Params p;
        p.target_speed_mps   = m_config.scripted.target_speed_kph / 3.6;
        p.hold_time_s        = m_config.scripted.hold_time_s;
        p.stop_threshold_mps = m_config.scripted.stop_threshold_mps;
        m_scripted = std::make_unique<ScriptedDriver>(p);
    }

    m_lights_demo = m_config.lights.demo_mode;

    m_paused = m_config.start_paused;

    // Startup banner — always printed regardless of headless/interactive mode.
    std::cout << "[SimApp] Vehicle: KEY OFF — press K to cycle RSA state (OFF→RUN→ACC→OFF)\n";

    // Warn loudly when an external-sim scenario runs without realtime
    // pacing.  ev1sim and the electricsim controllers (BTCM, PIM, RSA)
    // each run on their own simulated clocks: ev1sim ticks chrono
    // physics, BTCM advances the simavr-emulated AVR.  When ev1sim is
    // unpaced (`realtime: false`) it can run many times faster than the
    // BTCM, and the brake event finishes in BTCM-wall-clock-seconds
    // long before the firmware has had a chance to engage ABS.  Setting
    // `realtime: true` paces ev1sim against wall clock so both sides
    // see the same event durations.  The headline ABS-test scenarios
    // depend on this — without it, results are unreliable.
    if (m_config.external_sim.enabled &&
        m_scenario && m_scenario->has_stats() &&
        !m_config.simulation.realtime) {
        std::cerr << "\n";
        std::cerr << "[SimApp] WARNING: scenario has stats logging and "
                     "external_sim is enabled, but simulation.realtime is "
                     "false.\n";
        std::cerr << "[SimApp]          ev1sim will run as fast as the host "
                     "allows while electricsim controllers (BTCM/PIM/RSA)\n";
        std::cerr << "[SimApp]          run on their own simavr clocks — "
                     "the two sides will see different event durations\n";
        std::cerr << "[SimApp]          and ABS results will be unreliable.  "
                     "Set \"simulation.realtime\": true.\n";
        std::cerr << "\n";
    }

    if (headless) {
        std::cout << "[SimApp] Headless mode — no window.  Exits ";
        if (m_scripted)
            std::cout << "when the scripted scenario completes";
        if (m_config.simulation.max_time_s > 0.0) {
            if (m_scripted) std::cout << ", ";
            std::cout << "at sim_time " << m_config.simulation.max_time_s << "s";
        }
        std::cout << ", or on SIGINT.\n";
    } else {
        std::cout << "[SimApp] Ready.  Controls: WASD=drive  Space=park brake  "
                     "P=pause  R=respawn  C=camera  Scroll=zoom  B=horn  O=hi  L=lo  "
                     "H=headlights  K=key-on  .=PRND-up  ,=PRND-down  "
                     "Q=turn-L  E=turn-R  X=hazard  "
                     "F=hood  T=trunk  [=doorL  ]=doorR  Z=snapshot  "
                     "TAB=UI panel  Esc=quit\n";
        if (m_paused)
            std::cout << "[SimApp] Started PAUSED — press P to begin simulation\n";
    }
}

SimApp::~SimApp() = default;

// ---------------------------------------------------------------------------
void SimApp::SetupVisualization() {
    m_vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    m_vis->SetWindowTitle("EV1 Simulator");
    m_vis->SetWindowSize(1280, 720);
    m_vis->SetDriverType(irr::video::EDT_OPENGL);

    // Chase camera defaults (we override positioning via CameraManager,
    // but Chrono needs a chase camera configured to initialise properly).
    m_vis->SetChaseCamera(
        ChVector3d(0, 0, 1.5),  // track point on vehicle
        m_config.camera.chase_distance,
        0.5);

    m_vis->Initialize();
    m_vis->AddSkyBox();

    // Directional sun light with ambient from the environment preset.
    // AddLightDirectional(elevation_deg, azimuth_deg, ambient, specular, diffuse).
    const auto& env = m_config.environment;
    m_vis->AddLightDirectional(
        static_cast<float>(env.sun_elevation_deg), 60.0f,
        chrono::ChColor(static_cast<float>(env.ambient_r),
                        static_cast<float>(env.ambient_g),
                        static_cast<float>(env.ambient_b)),
        chrono::ChColor(0.3f, 0.3f, 0.3f),   // specular
        chrono::ChColor(1.0f, 1.0f, 0.9f));  // diffuse (warm sun)
    m_vis->AttachVehicle(&m_world->GetVehicle());

    // macOS platform fixes (no-ops on other platforms).
    macos_activate_app();
    macos_setup_menu_bar();
    macos_fix_retina_viewport();
    macos_enable_fullscreen();

    // Register keyboard handler.
    m_vis->AddUserEventReceiver(m_keyboard.get());
}

// ---------------------------------------------------------------------------
void SimApp::ApplyAbsFrontBrake(double time, double dt_s,
                                double local_front_brake) {
    using Phase = ExternalSimConnector::AbsPhaseFront::Phase;

    const auto abs_phase = m_external_sim->GetAbsPhaseFront(kAbsFreshnessWindow);

    // Log freshness transitions (one line per wheel).
    if (abs_phase.fl_fresh != m_abs_fl_was_fresh) {
        if (abs_phase.fl_fresh)
            std::cout << "[SimApp] front-brake from BTCM (live) wheel=FL\n";
        else
            std::cout << "[SimApp] front-brake fallback (BTCM stale) wheel=FL\n";
        m_abs_fl_was_fresh = abs_phase.fl_fresh;
    }
    if (abs_phase.fr_fresh != m_abs_fr_was_fresh) {
        if (abs_phase.fr_fresh)
            std::cout << "[SimApp] front-brake from BTCM (live) wheel=FR\n";
        else
            std::cout << "[SimApp] front-brake fallback (BTCM stale) wheel=FR\n";
        m_abs_fr_was_fresh = abs_phase.fr_fresh;
    }

    // BTCM-off / stale path: the EV1's front brake hydraulic line bypasses
    // the BTCM modulator when the controller is unpowered — the iso/dump
    // solenoids are normally-open / normally-closed so without current the
    // master cylinder pressure flows directly to the front calipers.  We
    // model that here by leaving the symmetric front_pressure from
    // CommandDriver::ApplyBrakes() (set inside VehicleWorld::Synchronize)
    // in effect.  Nothing to do.
    if (!abs_phase.fl_fresh && !abs_phase.fr_fresh) {
        // Keep prev values in sync with local so HOLD doesn't freeze at stale values.
        m_abs_fl_prev = local_front_brake;
        m_abs_fr_prev = local_front_brake;
        return;
    }

    // Finite-rate hydraulic model.  Real ABS valves are flow-limited: a
    // single dump pulse takes ~30 ms to bleed pressure to a third, not
    // an instantaneous step to zero.  Modeling APPLY as exponential
    // rise toward MC pressure and DUMP as exponential decay toward 0
    // matches the published service-bay numbers and avoids the
    // pathological behavior of the old step-function model (which
    // dropped pressure to 0.2 × MC every dump tick — effectively
    // "open-circuit" braking through the ABS unit during long DUMP
    // sequences).
    //
    // Time constants are first-pass estimates — refined in future
    // tuning passes once we have GM EV1 hardware reference data.
    //
    // **2026-05-18 tuning attempt — mu_jump (~50% FL/FR time-locked).**
    // The prior tau_dump = 60 ms over-estimated the dump-valve bleed
    // time, leaving plant pressure too high during the asphalt→ice
    // transition: the wheel went negative-rotating before the
    // pressure could fall, so the ABS algorithm couldn't see spin-up
    // and the slip ratio pinned at 1.  Lowered tau_dump from 60 ms to
    // 30 ms — close to the manual's "1/3 of pressure in 30 ms" claim
    // (analytically tau = 30 ms / ln(3) ≈ 27.3 ms).  30 ms is a clean
    // round number close to the analytic target.
    //
    // **Validated 2026-05-21 — keep 30 ms.**  Full 6-scenario BTCM-on/off
    // sweep (electricsim controllers rebuilt at 77e2a1f9) vs the dc63934
    // baseline: mu_jump FL/FR time-locked dropped ~50% → 36%/32% (the goal);
    // diagonal_mu also improved (5% locked); high_mu 149.4 m, split_mu
    // 73.4 m, brake_and_steer 60.8 m all within ~1% of baseline with heading
    // preserved; no scenario regressed.  The earlier "0 phase transitions"
    // scare was a stale-controller artifact, not the plant.  Regression gate
    // for this knob now lives in scripts/abs_regression.sh + abs_baseline.txt
    // (`make test-integration`); see electricsim/docs/btcm_deferred_todos.md §8.
    const double tau_apply = 0.005;  // s, caliper-fill time constant
    const double tau_dump  = 0.030;  // s, dump-valve release time constant
                                     // (was 0.060; validated 2026-05-21)
    const double alpha_apply =
        dt_s >= tau_apply ? 1.0 : (1.0 - std::exp(-dt_s / tau_apply));
    const double alpha_dump  =
        dt_s >= tau_dump  ? 1.0 : (1.0 - std::exp(-dt_s / tau_dump));

    auto modulate = [&](Phase phase, double prev, double local) -> double {
        switch (phase) {
            case Phase::APPLY: return prev + alpha_apply * (local - prev);
            case Phase::HOLD:  return prev;
            case Phase::DUMP:  return prev * (1.0 - alpha_dump);
        }
        return local;  // unreachable
    };

    const double fl = modulate(abs_phase.fl, m_abs_fl_prev, local_front_brake);
    const double fr = modulate(abs_phase.fr, m_abs_fr_prev, local_front_brake);

    m_abs_fl_prev = fl;
    m_abs_fr_prev = fr;

    // Apply per-wheel front brakes via Chrono API, overriding the symmetric
    // front_pressure CommandDriver::ApplyBrakes() already set in Synchronize().
    m_world->ApplyFrontBrakePerWheel(time, fl, fr);
}

// ---------------------------------------------------------------------------
void SimApp::ApplyRearEmbBrake(double time, double /*local_rear_brake*/) {
    // BTCM-failure model: real EV1 rear brakes are PURELY electromechanical.
    // There's no hydraulic backup line from the master cylinder, unlike the
    // front calipers.  When the BTCM is not commanding the rear motors —
    // either powered down or signals stale — the rear shoes free-roll and
    // contribute zero brake force.  This is the safety design we model
    // here, even though it's an unexpected operating state.  The unused
    // `local_rear_brake` arg is intentional; we never fall back to it.

    const auto cmd = m_external_sim->GetRearEmbCmd(kAbsFreshnessWindow);

    if (cmd.lr_fresh != m_rear_lr_was_fresh) {
        std::cout << (cmd.lr_fresh
                          ? "[SimApp] rear-brake from BTCM (live) wheel=RL\n"
                          : "[SimApp] rear-brake INACTIVE (BTCM stale) wheel=RL — "
                            "no hydraulic backup\n");
        m_rear_lr_was_fresh = cmd.lr_fresh;
    }
    if (cmd.rr_fresh != m_rear_rr_was_fresh) {
        std::cout << (cmd.rr_fresh
                          ? "[SimApp] rear-brake from BTCM (live) wheel=RR\n"
                          : "[SimApp] rear-brake INACTIVE (BTCM stale) wheel=RR — "
                            "no hydraulic backup\n");
        m_rear_rr_was_fresh = cmd.rr_fresh;
    }

    // Convert the [-1, +1] motor command to a clamping force.  +1 = full apply
    // (max shoe force), 0 or negative = no force (motor idling or retracting).
    const ev1sim::BrakeDrum::Params drum;
    auto cmd_to_force = [&drum](float c) {
        const double clipped = c < 0.0f ? 0.0 : (c > 1.0f ? 1.0 : double{c});
        return clipped * drum.max_shoe_force_n;
    };

    // Wheel angular velocity needed for the self-energizing factor.
    // VehicleState.wheel_omega is indexed FL, FR, RL, RR.
    const auto state = m_world->GetState();
    const double omega_rl = state.wheel_omega[2];
    const double omega_rr = state.wheel_omega[3];

    auto torque_to_ratio = [&](double torque_nm) {
        const double ratio = torque_nm / kRearBrakeMaxTorqueNm;
        return ratio < 0.0 ? 0.0 : (ratio > 1.0 ? 1.0 : ratio);
    };

    // Per-wheel: BTCM command if fresh, else 0 (rear has no hydraulic
    // fallback path).  Always call ApplyRearBrakePerWheel so we actively
    // zero the rear when BTCM is off — otherwise Chrono would carry the
    // last value forever.
    const double rl_ratio = cmd.lr_fresh
        ? torque_to_ratio(ev1sim::BrakeDrum::torque_magnitude_nm(
              cmd_to_force(cmd.lr), omega_rl, drum))
        : 0.0;
    const double rr_ratio = cmd.rr_fresh
        ? torque_to_ratio(ev1sim::BrakeDrum::torque_magnitude_nm(
              cmd_to_force(cmd.rr), omega_rr, drum))
        : 0.0;
    m_world->ApplyRearBrakePerWheel(time, rl_ratio, rr_ratio);
}

// ---------------------------------------------------------------------------
void SimApp::ApplyElectronicsThrottle(DriverCommand& cmd) {
    if (m_driver_mode != "electronics") return;

    const auto bus = m_external_sim->GetThrottleCmd(m_throttle_freshness_window);

    if (bus.fresh != m_throttle_bus_was_fresh) {
        if (bus.fresh) {
            std::cout << "[SimApp] throttle from PIM (live) q8="
                      << static_cast<int>(bus.q8) << "\n";
        } else {
            std::cout << "[SimApp] throttle fallback (PIM stale) — "
                         "using local pedal\n";
        }
        m_throttle_bus_was_fresh = bus.fresh;
    }

    if (!bus.fresh) return;
    cmd.throttle = static_cast<double>(bus.q8) / 255.0;
}

void SimApp::ApplyElectronicsSteering(DriverCommand& cmd) {
    if (m_driver_mode != "electronics") return;

    const auto bus = m_external_sim->GetSteeringCmd(m_steering_freshness_window);

    if (bus.fresh != m_steering_bus_was_fresh) {
        if (bus.fresh) {
            std::cout << "[SimApp] steering from bus (live) value="
                      << bus.value << "\n";
        } else {
            std::cout << "[SimApp] steering fallback (bus stale) — "
                         "using local input\n";
        }
        m_steering_bus_was_fresh = bus.fresh;
    }

    if (!bus.fresh) return;
    // Normalized steering; clamp defensively to the valid [-1, +1] range.
    double v = static_cast<double>(bus.value);
    if (v < -1.0) v = -1.0;
    if (v >  1.0) v =  1.0;
    cmd.steering = v;
}

// ---------------------------------------------------------------------------
// Consume the electricsim-driven body actuator peripherals (specs added in the
// door-lock-motor / sounder / power-steering-pump round).  Advances the
// PhysicalWorld plant models from the connector's latched chassis-bus inputs.
// Called once per tick from both the windowed and headless loops, after the
// connector has drained inbound frames.
void SimApp::ConsumeBodyActuatorPeripherals(double dt) {
    if (!m_external_sim) return;

    // --- Power-steering pump motor (chassis 4097 in) ---
    // PSCM publishes a q8 commanded speed; drive the plant (0..1 normalized).
    {
        double cmd01 = 0.0;
        if (m_external_sim->HasReceivedSteeringPumpSpeedCmd())
            cmd01 = static_cast<double>(m_external_sim->GetSteeringPumpSpeedCmdQ8()) / 255.0;
        m_physical->power_steering_pump().update(dt, cmd01);
    }

    // --- Sounder / piezo "tick" (chassis 4096 in) ---
    // The LHJB flasher gates the piezo on/off each flash half-cycle; that
    // toggling IS the audible TURN/HAZ tick.  Drive the buzz from sounding()
    // (continuous while energised); SounderAudio is a no-op off macOS.
    {
        m_physical->sounder().update(m_external_sim->GetSounderPiezoDrive());
        if (m_sounder_audio)
            m_sounder_audio->SetSounding(m_physical->sounder().sounding());
    }

    // --- Door-lock motors LH/RH (chassis 4092-4095 in) ---
    // Prefer the RHJB dual-H-bridge motor-leg drives when present: advance each
    // motor and mirror its end-of-travel stroke into DoorLocks.  When the legs
    // are not published yet (electricsim hasn't adopted 4092-4095) the motors
    // never move and the high-level 4084/4085 mirror remains authoritative.
    {
        using S = ev1sim::DoorLocks::State;
        using Stroke = ev1sim::DoorLockMotor::Stroke;
        const bool have_legs =
            m_external_sim->HasReceivedDoorLockMotorDrive(0) ||
            m_external_sim->HasReceivedDoorLockMotorDrive(1) ||
            m_external_sim->HasReceivedDoorLockMotorDrive(2) ||
            m_external_sim->HasReceivedDoorLockMotorDrive(3);
        if (have_legs) {
            ev1sim::DoorLockMotor* motors[2] = {
                &m_physical->door_lock_motor_lh(),   // door 0 = driver
                &m_physical->door_lock_motor_rh()};  // door 1 = passenger
            for (int door = 0; door < 2; ++door) {
                const bool lock_drv   = m_external_sim->GetDoorLockMotorDrive(door * 2);
                const bool unlock_drv = m_external_sim->GetDoorLockMotorDrive(door * 2 + 1);
                motors[door]->update(dt, lock_drv, unlock_drv);
                const Stroke stroke = motors[door]->stroke();
                if (static_cast<int>(stroke) == m_last_dlm_stroke[door]) continue;
                m_last_dlm_stroke[door] = static_cast<int>(stroke);
                // Only an end-of-travel limit changes the latched lock state.
                if (stroke == Stroke::LOCKED || stroke == Stroke::UNLOCKED) {
                    const S st = (stroke == Stroke::LOCKED) ? S::LOCKED : S::UNLOCKED;
                    if (door == 0) m_physical->door_locks().set_driver(st);
                    else           m_physical->door_locks().set_passenger(st);
                    std::cout << "[SimApp] Door-lock motor "
                              << (door == 0 ? "LH/driver" : "RH/passenger")
                              << " reached " << motors[door]->stroke_name() << "\n";
                    if (m_sounder_audio) m_sounder_audio->PlayClick();  // solenoid click
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
int SimApp::Run() {
    // Some scenarios drive or assert on signals that only the external
    // electronics sim (electricsim: BTCM ABS, PIM cruise, ...) produces.  Run
    // them only when that sim can actually be present.  Note ev1sim *creates*
    // the shared-memory bus and the electricsim controllers attach to it, so a
    // "connected" transport just means our own bus is open — it does not prove
    // a controller is there, and controllers attach during the loop, after
    // this point.  What we *can* tell at startup is when the external sim can
    // never appear: a build without electricsim (Status::Unavailable) or
    // --external-sim off (Status::Disabled).  In those cases skip cleanly
    // (exit 0) rather than failing assertions or burning a realtime run.
    if (m_scenario && m_scenario->requires_external_sim()) {
        const auto status = m_external_sim->GetStatus();
        if (status == ExternalSimConnector::Status::Unavailable ||
            status == ExternalSimConnector::Status::Disabled) {
            std::cout << "[SimApp] SKIPPED scenario '" << m_scenario->name()
                      << "': requires the external electronics sim, which is "
                         "not available (status: " << m_external_sim->StatusString()
                      << ").  Build with electricsim (ELECTRICSIM_DIR) and run "
                         "with --external-sim on.\n";
            return kExitSuccess;
        }
    }

    if (m_config.simulation.headless) {
        // Guard against the hang-forever foot-gun: headless with no terminator
        // and no scripted scenario would loop until SIGINT, which is a bad
        // default for CI.  Require at least one way to exit automatically.
        const bool has_max_time = m_config.simulation.max_time_s > 0.0;
        const bool has_scripted = m_config.scripted.enabled;
        const bool has_scenario = m_scenario != nullptr;
        if (!has_max_time && !has_scripted && !has_scenario) {
            std::cerr << "[SimApp] --headless requires at least one of "
                         "--max-time <s>, --scenario <file>, or a scripted "
                         "scenario (e.g. --scripted-accel-brake).  Otherwise "
                         "the run can only be ended by SIGINT.\n";
            return kExitUsage;
        }
        return RunHeadless();
    }
    return RunWithVisualization();
}

// ---------------------------------------------------------------------------
int SimApp::RunWithVisualization() {
    double step     = m_config.simulation.step_size_s;
    double render_dt = 1.0 / std::max(1, m_config.simulation.render_fps);
    int    steps_per_frame = std::max(1, static_cast<int>(std::round(render_dt / step)));
    double max_time = m_config.simulation.max_time_s;

    while (m_vis->Run()) {
        // --- Input (once per render frame) ---
        // Keyboard always runs to provide one-shot actions (pause, camera,
        // quit, etc.), but the drive command comes from the scripted driver
        // when configured — useful for visually debugging a scenario.
        DriverCommand cmd = m_keyboard->Update(render_dt);
#ifdef EV1SIM_HAVE_WHEEL_IO
        // Wheel overlays the keyboard's analog axes + adds its mapped buttons
        // when a device is present; the keyboard stays the fallback driver.
        if (m_sdl) m_sdl->Pump();
        if (m_wheel) {
            DriverCommand w = m_wheel->Update(render_dt);
            if (m_wheel->HasDevice()) {
                // Lazily attach force feedback on first connect / hot-plug.
                if (!m_ffb && m_config.input.ffb.enabled) {
                    m_ffb = std::make_unique<ev1sim::ForceFeedback>(
                        m_wheel->JoystickHandle(), m_config.input.ffb);
                }
                cmd.steering    = w.steering;
                cmd.throttle    = w.throttle;
                cmd.front_brake = w.front_brake;
                cmd.rear_brake  = w.rear_brake;
                cmd.horn_low    = cmd.horn_low  || w.horn_low;
                cmd.horn_high   = cmd.horn_high || w.horn_high;
            } else if (m_ffb) {
                m_ffb.reset();  // wheel gone — drop FFB so it re-attaches on reconnect
            }
            for (ev1sim::InputAction a : m_wheel->PendingActions()) DispatchAction(a);
            m_wheel->ClearPendingActions();
        }
#endif
        if (m_scripted && !m_paused)
            cmd = m_scripted->Update(m_world->GetState());

        // Scenario harness — fires timed events + holds set_throttle/brake
        // overrides.  Runs before the bus-throttle override so the bus
        // value (PIM-computed) wins when in electronics mode.
        if (m_scenario && !m_paused) {
            m_scenario->Tick(m_world->GetSimTime(),
                              m_world->GetState(), *this, cmd);
        }

        // --- Bus-mediated throttle override (electronics drive mode) ---
        // No-op in "local" mode.  Must run before the propulsion gate so
        // a stale-fallback to the local pedal still respects KEY OFF.
        ApplyElectronicsThrottle(cmd);
        ApplyElectronicsSteering(cmd);

        // --- Propulsion enable gate (KEY OFF override) ---
        // While m_propulsion_enabled is false, clamp brakes at full and zero
        // throttle so Chrono reflects the vehicle's actual "key off" state.
        if (!m_propulsion_enabled) {
            cmd.throttle    = 0.0;
            cmd.front_brake = 1.0;
            cmd.rear_brake  = 1.0;
        }

        // Snapshot the final post-override command for the floating-UI display rows.
        m_last_cmd = cmd;

        m_world->GetDriver().SetCommand(cmd);

        // Handle one-shot actions.
        if (cmd.reset_vehicle)
            m_world->ResetVehicle();
        if (m_keyboard->ConsumeCameraCycle())
            m_camera->CycleMode();
        if (m_keyboard->ConsumePauseToggle()) {
            m_paused = !m_paused;
            std::cout << (m_paused ? "[SimApp] PAUSED" : "[SimApp] RESUMED") << std::endl;
        }
        if (m_keyboard->QuitRequested())
            break;
        // K = cycle RSA key state: OFF → RUN → ACC → OFF → …
        // Publishes momentary RSA keypad code_ok and mode_button signals.
        if (m_keyboard->ConsumeKeyOnToggle()) {
            m_physical->rsa_keypad().cycle_k();
            std::cout << "[SimApp] Key request: "
                      << m_physical->rsa_keypad().expected_state_name() << "\n";
        }
        if (m_keyboard->ConsumeHeadlightToggle()) {
            // H cycles the combination switch through OFF → PARK → ON → HI.
            // The wire-level pin states are published on the chassis bus and
            // LHJB drives the actual bulb feed lines back from there.
            if (m_lights_demo != "off") {
                std::cout << "[SimApp] Lights demo OFF (was " << m_lights_demo << ")\n";
                m_lights_demo = "off";
            }
            m_physical->combination_switch().cycle_h();
            const char* names[] = {"OFF", "PARK", "ON", "HI"};
            int idx = static_cast<int>(m_physical->combination_switch().position());
            std::cout << "[SimApp] CombSw: " << names[idx] << "\n";
        }
        // U is momentary flash-to-pass (pulled-back lever).  Update each frame.
        m_physical->combination_switch().set_flash_to_pass(
            m_keyboard->IsFlashToPassHeld());

        // '.' / ',' cycle PRND up/down (clamped at ends; no wraparound).
        if (m_keyboard->ConsumePrndUp()) {
            m_physical->prnd_selector().cycle_up();
            const char* prnd_names[] = {"P", "R", "N", "D"};
            int idx = static_cast<int>(m_physical->prnd_selector().position());
            std::cout << "[SimApp] PRND -> " << prnd_names[idx] << "\n";
        }
        if (m_keyboard->ConsumePrndDown()) {
            m_physical->prnd_selector().cycle_down();
            const char* prnd_names[] = {"P", "R", "N", "D"};
            int idx = static_cast<int>(m_physical->prnd_selector().position());
            std::cout << "[SimApp] PRND -> " << prnd_names[idx] << "\n";
        }
        // Q = turn signal LEFT; E = turn signal RIGHT; X = hazard toggle.
        if (m_keyboard->ConsumeTurnSignalLeft()) {
            m_physical->turn_signal_stalk().toggle_left();
            const auto pos = m_physical->turn_signal_stalk().position();
            using P = ev1sim::TurnSignalStalk::Position;
            const char* ts_name = (pos == P::LEFT) ? "LEFT" : (pos == P::RIGHT) ? "RIGHT" : "OFF";
            std::cout << "[SimApp] Turn signal: " << ts_name << "\n";
        }
        if (m_keyboard->ConsumeTurnSignalRight()) {
            m_physical->turn_signal_stalk().toggle_right();
            const auto pos = m_physical->turn_signal_stalk().position();
            using P = ev1sim::TurnSignalStalk::Position;
            const char* ts_name = (pos == P::LEFT) ? "LEFT" : (pos == P::RIGHT) ? "RIGHT" : "OFF";
            std::cout << "[SimApp] Turn signal: " << ts_name << "\n";
        }
        if (m_keyboard->ConsumeHazardToggle()) {
            m_physical->hazard_switch().toggle();
            std::cout << "[SimApp] Hazard: "
                      << (m_physical->hazard_switch().on() ? "ON" : "OFF") << "\n";
        }

        // I = IPC trip-reset (momentary).
        if (m_keyboard->ConsumeIpcTripReset()) {
            m_physical->ipc_trip_reset().press();
            ++m_trip_reset_count;
            std::cout << "[SimApp] IPC trip-reset pressed\n";
        }

        // Z = toggle physical-world snapshot overlay.
        if (m_keyboard->ConsumeSnapshotToggle())
            m_show_snapshot = !m_show_snapshot;

        // TAB = toggle UI mode (mouse cursor + floating panel).
        if (m_keyboard->ConsumeUiModeToggle()) {
            m_ui_mode = !m_ui_mode;
            // Camera: stop consuming mouse events while UI mode is active.
            m_camera->SetGrabbingMouse(!m_ui_mode);
            // Show/hide OS cursor.
            m_vis->GetDevice()->getCursorControl()->setVisible(m_ui_mode);
            // Show/hide floating panel.
            if (m_floating_ui)
                m_floating_ui->SetVisible(m_ui_mode);
            std::cout << "[SimApp] UI mode: " << (m_ui_mode ? "ON" : "OFF") << "\n";
        }

        // Cruise stalk (faithful chassis-cavity model): the SET/COAST (G or '-')
        // and RESUME/ACCEL (Y or '+') contacts are sampled as HELD state at
        // publish time via CruiseSetCoastContactClosed /
        // CruiseResumeAccelContactClosed and fed to cruise_stalk().update();
        // PIM decodes tap vs. hold from how long each stays closed.  N taps
        // CANCEL — momentarily opening the ON/OFF master latch.  (The legacy
        // per-action one-shots ConsumeCruiseSet/Resume/SpeedUp/SpeedDown are
        // intentionally NOT consumed here; the held contacts supersede them.)
        if (m_keyboard->ConsumeCruiseCancel()) {
            m_physical->cruise_stalk().press_cancel();
            std::cout << "[SimApp] Cruise: CANCEL\n";
        }

        // V = wiper cycle (OFF → INT → LOW → HIGH → OFF).
        if (m_keyboard->ConsumeWiperCycle()) {
            m_physical->wiper_stalk().cycle_position();
            const char* wiper_names[] = {"OFF", "INT", "LOW", "HIGH"};
            int widx = static_cast<int>(m_physical->wiper_stalk().position());
            std::cout << "[SimApp] Wiper: " << wiper_names[widx] << "\n";
        }

        // M = wiper wash (momentary).
        if (m_keyboard->ConsumeWiperWash()) {
            m_physical->wiper_stalk().press_wash();
            std::cout << "[SimApp] Wiper wash\n";
        }

        // Auto-cancel turn signal from steering travel.
        // run every frame regardless of pause so the state machine stays current.
        m_physical->turn_signal_stalk().update_for_steering(cmd.steering, render_dt);
        if (m_physical->turn_signal_stalk().consume_auto_cancel_event())
            std::cout << "[SimApp] Turn signal: AUTO-CANCEL\n";

        // --- Physics sub-stepping (skipped when paused) ---
        if (!m_paused) {
            for (int i = 0; i < steps_per_frame; ++i) {
                double t = m_world->GetSimTime();
                m_world->Synchronize(t);
                // Per-wheel BTCM ABS modulation (front axle).
                // Must follow Synchronize() (which calls ApplyBrakes internally)
                // so we can override the symmetric front pressure when BTCM is live.
                ApplyAbsFrontBrake(t, step, cmd.front_brake);
                // Per-wheel BTCM rear EMB integration.  Mirror pattern;
                // converts motor cmd → shoe force → drum torque per wheel
                // and applies to axle 1 via VehicleWorld::ApplyRearBrakePerWheel.
                ApplyRearEmbBrake(t, cmd.rear_brake);
                m_world->Advance(step);
            }
        }

        // --- Visualisation sync/advance ---
        double t = m_world->GetSimTime();
        m_vis->Synchronize(t, m_world->GetDriver().GetInputs());
        m_vis->Advance(render_dt);

        // --- Camera override ---
        m_camera->Update(m_world->GetPose());

        // --- Floating UI panel label refresh ---
        if (m_floating_ui)
            m_floating_ui->UpdateLabels();

        // --- Panel toggles (1-4) ---
        for (int i = 0; i < VehiclePanels::NUM_PANELS; ++i) {
            if (m_keyboard->ConsumePanelToggle(i))
                m_panels->Toggle(static_cast<PanelID>(i));
        }

        // --- Vehicle lights ---
        // Lazy-init: the Irrlicht scene nodes are created during the
        // first Synchronize/Advance cycle, so we initialise on first use.
        if (!m_lights->IsInitialized()) {
            auto* smgr = m_vis->GetDevice()->getSceneManager();
            m_lights->Initialize(smgr);
        }

        // --- External sim sync (publish panel sensors + dynamics + combo
        //     switch + driver inputs; drain bulb/horn cmds) ---
        for (int i = 0; i < VehiclePanels::NUM_PANELS; ++i) {
            m_external_sim->SetPanelSensor(static_cast<PanelID>(i),
                                           m_panels->IsOpen(static_cast<PanelID>(i)));
        }
        m_external_sim->SetVehicleState(m_world->GetState());
        // Switch wire-level outputs on the chassis bus (cavity fidelity):
        //   headlamp rotary    12084699  IDs 4040-4042
        //   turn/hazard stalk  12092237  IDs 4043-4046 (incl. single horn cmd)
        //   wiper/washer stalk 12092254  IDs 4054-4057
        // Published here, before the wiper wash one-shot is consumed below.
        {
            const auto& cs = m_physical->combination_switch();
            m_external_sim->SetCombSwOutputs(cs.pin_low_beam_out(),
                                             cs.pin_flash_to_pass_out(),
                                             cs.pin_park_headlamp_out());
            // Driver horn is a single contact (circuit 28); reflect the
            // keyboard/HW horn into the switch object, then publish its cavity.
            m_physical->horn_button().set_held(cmd.horn_low || cmd.horn_high);
            const auto& ts = m_physical->turn_signal_stalk();
            m_external_sim->SetTurnHazSwOutputs(
                ts.pin_right_turn_out(), ts.pin_left_turn_out(),
                m_physical->hazard_switch().pin_hazard_out(),
                m_physical->horn_button().pin_horn_out());
            const auto& ws = m_physical->wiper_stalk();
            m_external_sim->SetWiperWasherSwOutputs(
                ws.pin_delay_out(), ws.pin_request_out(),
                ws.pin_hi_out(), ws.pin_washer_switch_out());
        }
        // Charge coupler presence (chassis bus, ID 4060).  Stubbed false until
        // a floating-UI panel or charge-door animation sets it.
        m_external_sim->SetChargeCouplerPresent(
            m_physical->charge_coupler().present());
        // Power-steering pump HV interlock loop (chassis bus, ID 4098).
        // The pump motor body closes molex.D/E while present; PSCM senses it.
        m_external_sim->SetSteeringPumpInterlockClosed(
            m_physical->power_steering_pump().interlock_closed());
        // PRND selector wire-level pins (chassis bus, IDs 4050-4053).
        {
            const auto& prnd = m_physical->prnd_selector();
            m_external_sim->SetPrndSelector(prnd.pin_a(), prnd.pin_b(),
                                            prnd.pin_c(), prnd.pin_d());
        }
        // Driver inputs (main harness segment, IDs 6900-6903).
        {
            auto clamp01_q8 = [](double v) -> std::uint8_t {
                if (v < 0.0) v = 0.0;
                if (v > 1.0) v = 1.0;
                return static_cast<std::uint8_t>(v * 255.0 + 0.5);
            };
            // Steering Q8 in degrees, signed.  cmd.steering is normalized
            // -1..+1 (Chrono convention); map to ±90 deg as a placeholder.
            auto steer_deg_q8 = [](double v) -> std::int16_t {
                if (v < -1.0) v = -1.0;
                if (v >  1.0) v =  1.0;
                double deg_q8 = v * 90.0 * 256.0;
                if (deg_q8 >  32767.0) deg_q8 =  32767.0;
                if (deg_q8 < -32768.0) deg_q8 = -32768.0;
                return static_cast<std::int16_t>(deg_q8);
            };
            m_external_sim->SetDriverBrakePedalQ8(clamp01_q8(cmd.front_brake));
            m_external_sim->SetDriverThrottleQ8(clamp01_q8(cmd.throttle));
            m_external_sim->SetDriverSteeringDegQ8(steer_deg_q8(cmd.steering));
            // Gear selector: map PrndSelector::Position to enum (P=0, R=1, N=2, D=3).
            m_external_sim->SetDriverGearSelector(
                static_cast<std::uint8_t>(m_physical->prnd_selector().position()));
            // Brake switch (6904): derive from brake travel with hysteresis.
            bool brake_sw = m_physical->brake_switch().update(cmd.front_brake);
            m_external_sim->SetDriverBrakeSwitch(brake_sw);
            // Master cylinder pressure (4074): two-stage curve from pedal travel.
            // Drives BTCM brake-effort + the new rear EMB consumer.
            const double pressure_kpa =
                m_physical->brake_pedal().update(cmd.front_brake);
            m_external_sim->SetBrakeMasterPressureKpa(
                static_cast<float>(pressure_kpa));
            // Seatbelt driver (6964) + passenger (6965): driven by
            // PhysicalWorld::Seatbelts; default buckled, toggleable via UI.
            m_external_sim->SetDriverSeatbeltBuckled(
                m_physical->seatbelts().driver_buckled());
            m_external_sim->SetDriverSeatbeltBuckledPassenger(
                m_physical->seatbelts().passenger_buckled());
            // Turn signal stalk (6948, 6949) and hazard switch (6944).
            m_external_sim->SetDriverTurnSignalLeft(
                m_physical->turn_signal_stalk().active_left());
            m_external_sim->SetDriverTurnSignalRight(
                m_physical->turn_signal_stalk().active_right());
            m_external_sim->SetDriverHazardRequest(
                m_physical->hazard_switch().on());
            // RSA keypad buttons (6975-6979) and mode button (6971) — tick the
            // scheduler and consume whatever it has ready for this frame.
            // button_value encoding: 0=idle, 1=tap, 2=long-press.
            m_physical->rsa_keypad().update(render_dt);
            {
                auto fires = m_physical->rsa_keypad().consume_fires_now();
                m_external_sim->SetDriverRsaKeypadButton1(fires.button_value[0]);
                m_external_sim->SetDriverRsaKeypadButton2(fires.button_value[1]);
                m_external_sim->SetDriverRsaKeypadButton3(fires.button_value[2]);
                m_external_sim->SetDriverRsaKeypadButton4(fires.button_value[3]);
                m_external_sim->SetDriverRsaKeypadButton5(fires.button_value[4]);
                m_external_sim->SetDriverRsaModeButton(fires.mode_button);
            }
            // IPC trip-reset (6952): consume one-shot event and publish.
            m_external_sim->SetDriverIpcTripReset(
                m_physical->ipc_trip_reset().consume_press_event());
            // Cruise stalk: evolve the faithful 3-contact model from the keyboard
            // held contacts (UI presses + N cancel already folded in via press_*),
            // then publish the raw chassis cavities kSigCruiseSw_* (4047-4049).
            m_physical->cruise_stalk().update(
                render_dt,
                m_keyboard->CruiseSetCoastContactClosed(),
                m_keyboard->CruiseResumeAccelContactClosed());
            m_external_sim->SetCruiseSetCoastContact(
                m_physical->cruise_stalk().set_coast_contact());
            m_external_sim->SetCruiseResumeAccelContact(
                m_physical->cruise_stalk().resume_accel_contact());
            m_external_sim->SetCruiseOnOffContact(
                m_physical->cruise_stalk().on_off_contact());
            m_external_sim->SetDriverWiperSwitch(
                static_cast<std::uint8_t>(m_physical->wiper_stalk().position()));
            m_external_sim->SetDriverWiperWashRequest(
                m_physical->wiper_stalk().consume_wash());
            // Power window switches (6980-6983) — no keyboard source;
            // floating UI panel will call press()/release() when expanded.
            {
                const auto& pw = m_physical->power_windows();
                m_external_sim->SetDriverPowerWindowDriverUp(pw.driver_up());
                m_external_sim->SetDriverPowerWindowDriverDown(pw.driver_down());
                m_external_sim->SetDriverPowerWindowPassengerUp(pw.passenger_up());
                m_external_sim->SetDriverPowerWindowPassengerDown(pw.passenger_down());
            }
            // HVAC driver controls (4124-4128) — no keyboard source yet; the
            // floating UI panel will drive the setters when expanded.  Publish
            // the current panel state each tick so the HTCM-facing wire format
            // is locked (mirrors the PowerWindows posture above).
            {
                const auto& hv = m_physical->hvac_controls();
                m_external_sim->SetHvacTempSetpointC(
                    static_cast<float>(hv.temp_setpoint_c()));
                m_external_sim->SetHvacFanRequest(hv.fan_u8());
                m_external_sim->SetHvacModeRequest(hv.mode_u8());
                m_external_sim->SetHvacAcRequest(hv.ac_on());
                m_external_sim->SetHvacDefrostRequest(hv.defrost_on());
            }
            // Door lock STATE feedback (4155-4157) — publish the resulting
            // DoorLocks state (after the door_lock_motor / RSA-cmd mirror) so an
            // RSA/IPC central-locking consumer can confirm the actuated state.
            {
                using S = ev1sim::DoorLocks::State;
                const auto& dl = m_physical->door_locks();
                m_external_sim->SetDoorLockState(dl.driver()    == S::LOCKED,
                                                 dl.passenger() == S::LOCKED,
                                                 dl.trunk()     == S::LOCKED);
            }
            // RSA exterior keypad (6985-6989) and door handle attempts (6990-6991).
            // Tick the sequence emitter then consume any pending fires.
            m_physical->rsa_exterior_keypad().update(render_dt);
            m_physical->rsa_exterior_keypad().consume_sequence_fire();
            m_external_sim->SetDriverRsaExteriorKeypad1(
                m_physical->rsa_exterior_keypad().button_value(0));
            m_external_sim->SetDriverRsaExteriorKeypad2(
                m_physical->rsa_exterior_keypad().button_value(1));
            m_external_sim->SetDriverRsaExteriorKeypad3(
                m_physical->rsa_exterior_keypad().button_value(2));
            m_external_sim->SetDriverRsaExteriorKeypad4(
                m_physical->rsa_exterior_keypad().button_value(3));
            m_external_sim->SetDriverRsaExteriorKeypad5(
                m_physical->rsa_exterior_keypad().button_value(4));
            m_physical->rsa_exterior_keypad().clear_oneshots();
            m_external_sim->SetDriverDoorHandleAttemptDriver(
                m_physical->door_handles().driver_attempt());
            m_external_sim->SetDriverDoorHandleAttemptPassenger(
                m_physical->door_handles().passenger_attempt());
            m_physical->door_handles().clear_oneshots();
            // 3D-sim contract buttons + key position (6940-6947, 6966).
            PushExtContractDriverInputs(cmd);
        }
        // Ambient temp + humidity (chassis bus 4090-4091).
        // Time-of-day from system clock → local time → fractional hours.
        {
            const auto now_wall = std::chrono::system_clock::now();
            const std::time_t now_t = std::chrono::system_clock::to_time_t(now_wall);
            std::tm local_tm{};
#if defined(_WIN32)
            localtime_s(&local_tm, &now_t);
#else
            localtime_r(&now_t, &local_tm);
#endif
            const double tod_hours = local_tm.tm_hour
                                   + local_tm.tm_min  / 60.0
                                   + local_tm.tm_sec  / 3600.0;
            m_physical->ambient_temp_sensor().update(tod_hours);
            m_external_sim->SetAmbientTempC(
                static_cast<float>(m_physical->ambient_temp_sensor().temp_c()));
            m_external_sim->SetAmbientHumidityPct(
                static_cast<float>(m_physical->ambient_temp_sensor().humidity_pct()));
        }
        // Motor RPM + torque (chassis bus 4070-4071). DC pack current (4072) is
        // electricsim/PIM's to derive from these — ev1sim publishes only the
        // mechanical operating point.
        {
            auto* engine = m_world->GetVehicle().GetEngine().get();
            float motor_rpm = 0.0f;
            float motor_torque = 0.0f;
            if (engine) {
                // GetMotorSpeed() returns rad/s; convert to RPM.
                const double omega  = engine->GetMotorSpeed();
                const double torque = engine->GetOutputMotorshaftTorque();
                motor_rpm     = static_cast<float>(omega * 60.0 / (2.0 * 3.14159265358979323846));
                motor_torque  = static_cast<float>(torque);
            }
            m_external_sim->SetMotorRpm(motor_rpm);
            m_external_sim->SetMotorTorqueNm(motor_torque);
        }
        m_external_sim->Tick(t);

        // Subscribe to RSA run-mode broadcast and update propulsion gate.
        if (m_external_sim->HasReceivedRunMode()) {
            const std::uint8_t run_mode = m_external_sim->GetRsaRunMode();
            // RSA run modes: 0=OFF, 1=ACC, 2=RUN (per rsa_scan.h).
            const bool new_prop = (run_mode == 2 /*RUN*/);
            if (new_prop != m_propulsion_enabled) {
                m_propulsion_enabled = new_prop;
                const char* mode_names[] = {"OFF", "ACC", "RUN"};
                const char* mode_str = (run_mode < 3) ? mode_names[run_mode] : "UNKNOWN";
                std::cout << "[SimApp] Run mode (from RSA): " << mode_str << "\n";
            }
        }

        const bool ext_driving_bulbs =
            m_external_sim->IsConnected() && m_external_sim->HasReceivedBulbData();

        if (m_lights_demo == "blink") {
            // All bulbs blink at unique frequencies for identification.
            m_lights->UpdateDemoMode(t);
        } else if (m_lights_demo == "chase") {
            // One bulb at a time, ~0.2s each, walking around the vehicle.
            m_lights->UpdateChaseDemo(t);
        } else if (ext_driving_bulbs) {
            // Bulb state is fully authored by the external electrical sim.
            for (int i = 0; i < NUM_LIGHTS; ++i) {
                m_lights->SetState(static_cast<LightID>(i),
                                   m_external_sim->GetBulbCmd(static_cast<LightID>(i)));
            }
        } else {
            // No demo, no external sim — all bulbs stay off.  We deliberately
            // do NOT have a local-fallback headlamp path: the user wants the
            // ECU pipeline to be the only source of bulb state, so missing
            // ECU output makes the missing wire visible rather than masking it.
            for (int i = 0; i < NUM_LIGHTS; ++i)
                m_lights->SetState(static_cast<LightID>(i), false);
        }

        m_lights->ApplyToScene();

        // --- Wiper renderer tick ---
        // Pull the latest motor command from RHJB (0xFF if never received →
        // Tick() treats it as OFF since no valid enum matches).
        {
            const std::uint8_t wiper_cmd =
                m_external_sim->HasReceivedWiperMotorCommand()
                    ? m_external_sim->GetWiperMotorCommand()
                    : 0u;  // default OFF when no RHJB command yet
            m_wiper->Tick(render_dt, wiper_cmd);
        }
        m_wiper->ApplyToScene();

        // --- Door lock mirror from RSA (chassis bus 4084/4085) ---
        // RSA publishes desired lock state; ev1sim mirrors it to DoorLocks.
        {
            using S = ev1sim::DoorLocks::State;
            for (int side = 0; side < 2; ++side) {
                if (!m_external_sim->HasReceivedDoorLockCmd(side)) continue;
                const std::uint8_t cmd = m_external_sim->GetDoorLockCmd(side);
                if (cmd == m_last_door_lock_cmd[side]) continue;
                m_last_door_lock_cmd[side] = cmd;
                const S new_state = (cmd == 1u) ? S::LOCKED : S::UNLOCKED;
                if (side == 0) m_physical->door_locks().set_driver(new_state);
                else           m_physical->door_locks().set_passenger(new_state);
                std::cout << "[SimApp] Door lock " << (side == 0 ? "driver" : "passenger")
                          << ": " << (new_state == S::LOCKED ? "LOCKED" : "UNLOCKED") << "\n";
            }
        }

        // --- Power window motor log (chassis bus 4086/4087) — no visual yet ---
        {
            static const char* kPwNames[] = {"STOP", "UP", "DOWN"};
            static const char* kSideNames[] = {"driver", "passenger"};
            for (int side = 0; side < 2; ++side) {
                if (!m_external_sim->HasReceivedPowerWindowMotor(side)) continue;
                const std::uint8_t cmd = m_external_sim->GetPowerWindowMotor(side);
                if (cmd == m_last_pw_motor_cmd[side]) continue;
                m_last_pw_motor_cmd[side] = cmd;
                const char* name = (cmd <= 2u) ? kPwNames[cmd] : "UNKNOWN";
                // TODO: drive window position animation when geometry is available.
                std::cout << "[SimApp] Power window " << kSideNames[side]
                          << ": " << name << "\n";
            }
        }

        // --- Body actuator peripherals (door-lock motors / sounder / pump) ---
        ConsumeBodyActuatorPeripherals(render_dt);

        // --- Render ---
        m_vis->BeginScene();
        macos_apply_viewport();   // override glViewport for Retina / resize
        m_vis->Render();
        // Ask the world for the truthful terrain label — it reflects the
        // actual loaded terrain, including the rigid-plane fallback when
        // a requested level file was missing or invalid.
        auto mu = m_world->GetWheelFrictions();
        const auto abs_phase = m_external_sim->GetAbsPhaseFront(kAbsFreshnessWindow);
        m_telemetry->DrawHUD(m_vis->GetDevice(),
                             m_world->GetState(),
                             m_camera->GetModeName(),
                             m_world->GetTerrainLabel(),
                             mu.mu,
                             &abs_phase);
        m_lights->DrawHUD(m_vis->GetDevice());
        m_panels->DrawHUD(m_vis->GetDevice());
        m_wiper->DrawHUD(m_vis->GetDevice());
        m_physical->DrawHUD(m_vis->GetDevice(),
                            m_external_sim->GetRsaRunMode(),
                            m_external_sim->HasReceivedRunMode());
        m_physical->DrawSnapshotOverlay(m_vis->GetDevice(),
                                        m_show_snapshot,
                                        m_external_sim->GetRsaRunMode(),
                                        m_external_sim->HasReceivedRunMode(),
                                        m_trip_reset_count);

        // Draw PAUSED overlay.
        if (m_paused) {
            auto* gui  = m_vis->GetDevice()->getGUIEnvironment();
            auto* font = gui->getBuiltInFont();
            if (font) {
                auto dim = m_vis->GetDevice()->getVideoDriver()->getScreenSize();
                irr::core::recti rect(dim.Width / 2 - 40, 30, dim.Width / 2 + 40, 50);
                font->draw(L"[ PAUSED ]", rect,
                           irr::video::SColor(255, 255, 200, 0), true, true);
            }
        }

        // Help overlay toggle (? key).
        if (m_keyboard->ConsumeHelpToggle())
            m_show_help = !m_show_help;
        // Auto-hide after first 5 sim seconds.
        const double sim_now = m_world->GetSimTime();
        if (m_show_help && sim_now > m_help_hide_time) {
            // Auto-hide only once (disable the timer by setting to a huge value).
            m_show_help = false;
            m_help_hide_time = 1e18;
        }

        // Draw keyboard help overlay.
        if (m_show_help) {
            auto* drv  = m_vis->GetDevice()->getVideoDriver();
            auto* gui  = m_vis->GetDevice()->getGUIEnvironment();
            auto* font = gui->getBuiltInFont();
            if (drv && font) {
                auto dim = drv->getScreenSize();
                // Translucent dark background box.
                const int bx = dim.Width / 2 - 220;
                const int by = 60;
                const int bw = 440;
                const int bh = 312;
                drv->draw2DRectangle(
                    irr::video::SColor(180, 20, 20, 30),
                    irr::core::recti(bx, by, bx + bw, by + bh));

                const int h = 16;
                int ry = by + 6;
                irr::video::SColor hdr(255, 255, 220,  80);
                irr::video::SColor dim_col(255, 160, 160, 160);
                irr::video::SColor norm(255, 220, 220, 220);

                auto drawL = [&](const wchar_t* text, irr::video::SColor col) {
                    font->draw(text,
                        irr::core::rect<irr::s32>(bx + 8, ry, bx + bw - 8, ry + h),
                        col);
                    ry += h;
                };

                drawL(L"KEYBOARD CONTROLS              [? to toggle]", hdr);
                drawL(L"-----------------------------------------------", dim_col);
                drawL(L"WASD/Arrows  drive (throttle, brake, steering)", norm);
                drawL(L"Space        parking brake",                       norm);
                drawL(L"K            key cycle: OFF -> RUN -> ACC -> OFF", norm);
                drawL(L"H            headlamp: OFF -> PARK -> ON -> HI",  norm);
                drawL(L"U            flash-to-pass (held)",                norm);
                drawL(L"Q / E        turn signal left / right (toggle)",  norm);
                drawL(L"X            hazard toggle",                       norm);
                drawL(L", / .        PRND down / up",                      norm);
                drawL(L"B / O / L    horn (both / high / low)",           norm);
                drawL(L"F            hood pop/close (panel: full stage)",  norm);
                drawL(L"T            trunk toggle",                        norm);
                drawL(L"[ / ]        door L / R toggle",                  norm);
                drawL(L"Z            physical-world snapshot overlay",     norm);
                drawL(L"TAB          UI panel (mouse clicks)",            norm);
                drawL(L"C            camera mode cycle",                   norm);
                drawL(L"P            pause",                               norm);
                drawL(L"R            respawn",                             norm);
                drawL(L"Esc          quit",                                norm);
            }
        }

        m_vis->EndScene();

        // --- Horn audio (external sim commands OR'd with keyboard input) ---
#ifdef EV1SIM_HAVE_WHEEL_IO
        // Force feedback from the current front-axle self-aligning torque
        // (zeroed while paused or with no haptic device).
        if (m_ffb) {
            const auto& vs = m_world->GetState();
            m_ffb->Update(vs.steering_torque, vs.speed_mps, !m_paused);
        }
#endif
        bool horn_low  = cmd.horn_low;
        bool horn_high = cmd.horn_high;
        if (m_external_sim->IsConnected()) {
            horn_low  = horn_low  || m_external_sim->GetHornLowCmd();
            horn_high = horn_high || m_external_sim->GetHornHighCmd();
        }
        m_horn->SetTones(horn_low, horn_high);

        // --- Telemetry logging ---
        m_telemetry->Record(m_world->GetState(), render_dt);

        // --- Scenario stats sampling ---
        if (m_scenario) {
            m_scenario->MaybeSampleStats(m_world->GetSimTime(),
                                          m_world->GetState(),
                                          *m_external_sim, cmd);
        }

        // --- Realtime pacing ---
        if (m_config.simulation.realtime)
            m_realtime_timer.Spin(step * steps_per_frame);

        // --- Scripted-scenario complete ---
        if (m_scripted && m_scripted->IsDone()) {
            std::cout << "[SimApp] Scripted scenario complete at t="
                      << m_world->GetSimTime() << "s — exiting.\n";
            if (m_scenario) m_scenario->Close();
            return kExitSuccess;
        }

        // --- Scenario complete (data-driven) ---
        if (m_scenario && m_scenario->IsDone(m_world->GetSimTime())) {
            const bool failed = m_scenario->IsScenarioFailed();
            std::cout << "[SimApp] Scenario complete at t="
                      << m_world->GetSimTime() << "s "
                      << "(asserts: " << m_scenario->PassedAssertions()
                      << " passed, "  << m_scenario->FailedAssertions()
                      << " failed) — exiting.\n";
            m_scenario->Close();
            return failed ? kExitScenarioAssertion : kExitSuccess;
        }

        // --- Max-time exit (shared with headless) ---
        if (max_time > 0.0 && m_world->GetSimTime() >= max_time) {
            const bool scripted_unfinished = m_scripted && !m_scripted->IsDone();
            if (scripted_unfinished) {
                std::cerr << "[SimApp] max_time_s reached with scripted "
                             "scenario still in phase '"
                          << m_scripted->PhaseName() << "' — timeout.\n";
                return kExitTimeout;
            }
            std::cout << "[SimApp] max_time_s reached — exiting.\n";
            if (m_scenario) m_scenario->Close();
            return kExitSuccess;
        }
    }
    // Window closed / Esc pressed — normal exit.
    if (m_scenario) m_scenario->Close();
    return kExitSuccess;
}

// ---------------------------------------------------------------------------
int SimApp::RunHeadless() {
    // Install SIGINT handler so Ctrl-C breaks out of the loop cleanly.
    g_stop_requested.store(false, std::memory_order_relaxed);
    SigintPrev old_sa = InstallSigint(&HeadlessSigintHandler);

    const double step     = m_config.simulation.step_size_s;
    const double tick_dt  = 1.0 / std::max(1, m_config.simulation.render_fps);
    const int    steps_per_tick =
        std::max(1, static_cast<int>(std::round(tick_dt / step)));
    const double max_time = m_config.simulation.max_time_s;

    // Default driver command — zero throttle/brake/steering when no scripted
    // driver is configured.  Apply startup override (brakes clamped, throttle
    // zeroed) so the initial SetCommand also reflects "key off".
    {
        DriverCommand init_cmd{};
        if (!m_propulsion_enabled) {
            init_cmd.front_brake = 1.0;
            init_cmd.rear_brake  = 1.0;
        }
        m_world->GetDriver().SetCommand(init_cmd);
    }

    const auto wall_start = std::chrono::steady_clock::now();

    while (!g_stop_requested.load(std::memory_order_relaxed)) {
        // --- Scripted driver (if any) — reads previous-tick state and
        //     emits a new command before we step physics.
        DriverCommand cmd{};
        if (m_scripted) {
            cmd = m_scripted->Update(m_world->GetState());
        }

        // Scenario harness — fires timed events + holds set_throttle/brake
        // overrides.  Headless paths don't pause, so always tick.
        if (m_scenario) {
            m_scenario->Tick(m_world->GetSimTime(),
                              m_world->GetState(), *this, cmd);
        }

        // --- Bus-mediated throttle override (electronics drive mode) ---
        // In headless mode there is no keyboard pedal, so the local
        // fallback is the scripted driver (or zero throttle).  When the
        // bus is fresh, PIM's commanded throttle replaces the local value.
        ApplyElectronicsThrottle(cmd);
        ApplyElectronicsSteering(cmd);

        // --- Propulsion enable gate (KEY OFF override) ---
        // In headless mode no keyboard presses cycle the RSA state.
        // Propulsion is enabled only if RSA broadcasts RUN on the bus.
        if (!m_propulsion_enabled) {
            cmd.throttle    = 0.0;
            cmd.front_brake = 1.0;
            cmd.rear_brake  = 1.0;
        }

        m_world->GetDriver().SetCommand(cmd);

        // Auto-cancel turn signal from steering travel (headless: scripted driver
        // may supply non-zero steering; keyboard path is absent).
        m_physical->turn_signal_stalk().update_for_steering(cmd.steering, tick_dt);
        if (m_physical->turn_signal_stalk().consume_auto_cancel_event())
            std::cout << "[SimApp] Turn signal: AUTO-CANCEL\n";

        // --- Physics sub-stepping (no pause control in headless) ---
        for (int i = 0; i < steps_per_tick; ++i) {
            const double t = m_world->GetSimTime();
            m_world->Synchronize(t);
            // Per-wheel BTCM ABS modulation (front axle).
            // Must follow Synchronize() so we override the symmetric front
            // pressure when BTCM is live.
            ApplyAbsFrontBrake(t, step, cmd.front_brake);
            // Per-wheel BTCM rear EMB integration.
            ApplyRearEmbBrake(t, cmd.rear_brake);
            m_world->Advance(step);
        }

        const double t = m_world->GetSimTime();

        // --- External sim sync (panel sensors + dynamics + combo switch
        //     + driver inputs; bulb/horn cmds drained inside Tick()) ---
        for (int i = 0; i < VehiclePanels::NUM_PANELS; ++i) {
            m_external_sim->SetPanelSensor(
                static_cast<PanelID>(i),
                m_panels->IsOpen(static_cast<PanelID>(i)));
        }
        m_external_sim->SetVehicleState(m_world->GetState());
        // Combination switch: in headless mode no keyboard cycles it, so the
        // pin states stay at their default (OFF position → all pins low).
        // Still publish so downstream consumers see a defined initial state.
        {
            const auto& cs = m_physical->combination_switch();
            m_external_sim->SetCombSwOutputs(cs.pin_low_beam_out(),
                                             cs.pin_flash_to_pass_out(),
                                             cs.pin_park_headlamp_out());
            // Turn/hazard/horn (12092237) + wiper (12092254) output cavities.
            m_physical->horn_button().set_held(cmd.horn_low || cmd.horn_high);
            const auto& ts = m_physical->turn_signal_stalk();
            m_external_sim->SetTurnHazSwOutputs(
                ts.pin_right_turn_out(), ts.pin_left_turn_out(),
                m_physical->hazard_switch().pin_hazard_out(),
                m_physical->horn_button().pin_horn_out());
            const auto& ws = m_physical->wiper_stalk();
            m_external_sim->SetWiperWasherSwOutputs(
                ws.pin_delay_out(), ws.pin_request_out(),
                ws.pin_hi_out(), ws.pin_washer_switch_out());
        }
        // Charge coupler presence (chassis bus, ID 4060).  Stubbed false until
        // a floating-UI panel or charge-door animation sets it.
        m_external_sim->SetChargeCouplerPresent(
            m_physical->charge_coupler().present());
        // Power-steering pump HV interlock loop (chassis bus, ID 4098).
        // The pump motor body closes molex.D/E while present; PSCM senses it.
        m_external_sim->SetSteeringPumpInterlockClosed(
            m_physical->power_steering_pump().interlock_closed());
        // PRND selector wire-level pins (chassis bus, IDs 4050-4053).
        // In headless mode stays at default Park; no keyboard cycling.
        {
            const auto& prnd = m_physical->prnd_selector();
            m_external_sim->SetPrndSelector(prnd.pin_a(), prnd.pin_b(),
                                            prnd.pin_c(), prnd.pin_d());
        }
        // Driver inputs — publish the override-adjusted values so the bus
        // reflects what the vehicle is actually doing (IDs 6900-6903).
        {
            auto clamp01_q8 = [](double v) -> std::uint8_t {
                if (v < 0.0) v = 0.0;
                if (v > 1.0) v = 1.0;
                return static_cast<std::uint8_t>(v * 255.0 + 0.5);
            };
            m_external_sim->SetDriverBrakePedalQ8(clamp01_q8(cmd.front_brake));
            m_external_sim->SetDriverThrottleQ8(clamp01_q8(cmd.throttle));
            m_external_sim->SetDriverSteeringDegQ8(0);  // no keyboard in headless
            // Gear selector: map PrndSelector::Position to enum (P=0, R=1, N=2, D=3).
            m_external_sim->SetDriverGearSelector(
                static_cast<std::uint8_t>(m_physical->prnd_selector().position()));
            // Brake switch (6904): derive from brake travel with hysteresis.
            bool brake_sw = m_physical->brake_switch().update(cmd.front_brake);
            m_external_sim->SetDriverBrakeSwitch(brake_sw);
            // Master cylinder pressure (4074): two-stage curve from pedal travel.
            // Drives BTCM brake-effort + the new rear EMB consumer.
            const double pressure_kpa =
                m_physical->brake_pedal().update(cmd.front_brake);
            m_external_sim->SetBrakeMasterPressureKpa(
                static_cast<float>(pressure_kpa));
            // Seatbelt driver (6964) + passenger (6965): driven by
            // PhysicalWorld::Seatbelts; default buckled.  No UI toggle in
            // headless mode — state stays at default (both buckled).
            m_external_sim->SetDriverSeatbeltBuckled(
                m_physical->seatbelts().driver_buckled());
            m_external_sim->SetDriverSeatbeltBuckledPassenger(
                m_physical->seatbelts().passenger_buckled());
            // Turn signal stalk (6948, 6949) and hazard switch (6944).
            // In headless mode no keyboard cycles them; publish stable defaults
            // (both false) so LHJB sees a defined initial state.
            m_external_sim->SetDriverTurnSignalLeft(
                m_physical->turn_signal_stalk().active_left());
            m_external_sim->SetDriverTurnSignalRight(
                m_physical->turn_signal_stalk().active_right());
            m_external_sim->SetDriverHazardRequest(
                m_physical->hazard_switch().on());
            // RSA keypad signals — headless: no key presses; tick scheduler
            // and publish whatever it has (likely idle zeros).
            // button_value encoding: 0=idle, 1=tap, 2=long-press.
            m_physical->rsa_keypad().update(tick_dt);
            {
                auto fires = m_physical->rsa_keypad().consume_fires_now();
                m_external_sim->SetDriverRsaKeypadButton1(fires.button_value[0]);
                m_external_sim->SetDriverRsaKeypadButton2(fires.button_value[1]);
                m_external_sim->SetDriverRsaKeypadButton3(fires.button_value[2]);
                m_external_sim->SetDriverRsaKeypadButton4(fires.button_value[3]);
                m_external_sim->SetDriverRsaKeypadButton5(fires.button_value[4]);
                m_external_sim->SetDriverRsaModeButton(fires.mode_button);
            }
            // IPC trip-reset (6952) + cruise stalk cavities (4047-4049), wiper.
            // Headless: no keyboard/UI input; evolve the cruise model with both
            // contacts open so the ON/OFF latch stays disarmed and the bus sees
            // stable zeros.
            m_external_sim->SetDriverIpcTripReset(
                m_physical->ipc_trip_reset().consume_press_event());
            m_physical->cruise_stalk().update(tick_dt, false, false);
            m_external_sim->SetCruiseSetCoastContact(
                m_physical->cruise_stalk().set_coast_contact());
            m_external_sim->SetCruiseResumeAccelContact(
                m_physical->cruise_stalk().resume_accel_contact());
            m_external_sim->SetCruiseOnOffContact(
                m_physical->cruise_stalk().on_off_contact());
            m_external_sim->SetDriverWiperSwitch(
                static_cast<std::uint8_t>(m_physical->wiper_stalk().position()));
            m_external_sim->SetDriverWiperWashRequest(
                m_physical->wiper_stalk().consume_wash());
            // Power window switches (6980-6983) — no keyboard source;
            // floating UI panel will call press()/release() when expanded.
            {
                const auto& pw = m_physical->power_windows();
                m_external_sim->SetDriverPowerWindowDriverUp(pw.driver_up());
                m_external_sim->SetDriverPowerWindowDriverDown(pw.driver_down());
                m_external_sim->SetDriverPowerWindowPassengerUp(pw.passenger_up());
                m_external_sim->SetDriverPowerWindowPassengerDown(pw.passenger_down());
            }
            // HVAC driver controls (4124-4128) — no keyboard source yet; the
            // floating UI panel will drive the setters when expanded.  Publish
            // the current panel state each tick so the HTCM-facing wire format
            // is locked (mirrors the PowerWindows posture above).
            {
                const auto& hv = m_physical->hvac_controls();
                m_external_sim->SetHvacTempSetpointC(
                    static_cast<float>(hv.temp_setpoint_c()));
                m_external_sim->SetHvacFanRequest(hv.fan_u8());
                m_external_sim->SetHvacModeRequest(hv.mode_u8());
                m_external_sim->SetHvacAcRequest(hv.ac_on());
                m_external_sim->SetHvacDefrostRequest(hv.defrost_on());
            }
            // Door lock STATE feedback (4155-4157) — publish the resulting
            // DoorLocks state (after the door_lock_motor / RSA-cmd mirror) so an
            // RSA/IPC central-locking consumer can confirm the actuated state.
            {
                using S = ev1sim::DoorLocks::State;
                const auto& dl = m_physical->door_locks();
                m_external_sim->SetDoorLockState(dl.driver()    == S::LOCKED,
                                                 dl.passenger() == S::LOCKED,
                                                 dl.trunk()     == S::LOCKED);
            }
            // RSA exterior keypad (6985-6989) and door handle attempts (6990-6991).
            // Headless: no UI source; tick the sequence emitter and publish zeros.
            m_physical->rsa_exterior_keypad().update(tick_dt);
            m_physical->rsa_exterior_keypad().consume_sequence_fire();
            m_external_sim->SetDriverRsaExteriorKeypad1(
                m_physical->rsa_exterior_keypad().button_value(0));
            m_external_sim->SetDriverRsaExteriorKeypad2(
                m_physical->rsa_exterior_keypad().button_value(1));
            m_external_sim->SetDriverRsaExteriorKeypad3(
                m_physical->rsa_exterior_keypad().button_value(2));
            m_external_sim->SetDriverRsaExteriorKeypad4(
                m_physical->rsa_exterior_keypad().button_value(3));
            m_external_sim->SetDriverRsaExteriorKeypad5(
                m_physical->rsa_exterior_keypad().button_value(4));
            m_physical->rsa_exterior_keypad().clear_oneshots();
            m_external_sim->SetDriverDoorHandleAttemptDriver(
                m_physical->door_handles().driver_attempt());
            m_external_sim->SetDriverDoorHandleAttemptPassenger(
                m_physical->door_handles().passenger_attempt());
            m_physical->door_handles().clear_oneshots();
            // 3D-sim contract buttons + key position (6940-6947, 6966).
            PushExtContractDriverInputs(cmd);
        }
        // Ambient temp + humidity (chassis bus 4090-4091).
        {
            const auto now_wall = std::chrono::system_clock::now();
            const std::time_t now_t = std::chrono::system_clock::to_time_t(now_wall);
            std::tm local_tm{};
#if defined(_WIN32)
            localtime_s(&local_tm, &now_t);
#else
            localtime_r(&now_t, &local_tm);
#endif
            const double tod_hours = local_tm.tm_hour
                                   + local_tm.tm_min  / 60.0
                                   + local_tm.tm_sec  / 3600.0;
            m_physical->ambient_temp_sensor().update(tod_hours);
            m_external_sim->SetAmbientTempC(
                static_cast<float>(m_physical->ambient_temp_sensor().temp_c()));
            m_external_sim->SetAmbientHumidityPct(
                static_cast<float>(m_physical->ambient_temp_sensor().humidity_pct()));
        }
        // Motor RPM + torque (chassis bus 4070-4071). DC pack current (4072) is
        // electricsim/PIM's to derive from these — ev1sim publishes only the
        // mechanical operating point.
        {
            auto* engine = m_world->GetVehicle().GetEngine().get();
            float motor_rpm = 0.0f;
            float motor_torque = 0.0f;
            if (engine) {
                const double omega  = engine->GetMotorSpeed();
                const double torque = engine->GetOutputMotorshaftTorque();
                motor_rpm     = static_cast<float>(omega * 60.0 / (2.0 * 3.14159265358979323846));
                motor_torque  = static_cast<float>(torque);
            }
            m_external_sim->SetMotorRpm(motor_rpm);
            m_external_sim->SetMotorTorqueNm(motor_torque);
        }
        m_external_sim->Tick(t);

        // Subscribe to RSA run-mode broadcast and update propulsion gate.
        if (m_external_sim->HasReceivedRunMode()) {
            const std::uint8_t run_mode = m_external_sim->GetRsaRunMode();
            const bool new_prop = (run_mode == 2 /*RUN*/);
            if (new_prop != m_propulsion_enabled) {
                m_propulsion_enabled = new_prop;
                const char* mode_names[] = {"OFF", "ACC", "RUN"};
                const char* mode_str = (run_mode < 3) ? mode_names[run_mode] : "UNKNOWN";
                std::cout << "[SimApp] Run mode (from RSA): " << mode_str << "\n";
            }
        }

        // --- Wiper renderer tick (headless — no DrawHUD, just phase tracking) ---
        {
            const std::uint8_t wiper_cmd =
                m_external_sim->HasReceivedWiperMotorCommand()
                    ? m_external_sim->GetWiperMotorCommand()
                    : 0u;
            m_wiper->Tick(tick_dt, wiper_cmd);
        }

        // --- Door lock mirror from RSA (chassis bus 4084/4085) ---
        {
            using S = ev1sim::DoorLocks::State;
            for (int side = 0; side < 2; ++side) {
                if (!m_external_sim->HasReceivedDoorLockCmd(side)) continue;
                const std::uint8_t cmd = m_external_sim->GetDoorLockCmd(side);
                if (cmd == m_last_door_lock_cmd[side]) continue;
                m_last_door_lock_cmd[side] = cmd;
                const S new_state = (cmd == 1u) ? S::LOCKED : S::UNLOCKED;
                if (side == 0) m_physical->door_locks().set_driver(new_state);
                else           m_physical->door_locks().set_passenger(new_state);
                std::cout << "[SimApp] Door lock " << (side == 0 ? "driver" : "passenger")
                          << ": " << (new_state == S::LOCKED ? "LOCKED" : "UNLOCKED") << "\n";
            }
        }

        // --- Power window motor log (chassis bus 4086/4087) — no visual yet ---
        {
            static const char* kPwNames[] = {"STOP", "UP", "DOWN"};
            static const char* kSideNames[] = {"driver", "passenger"};
            for (int side = 0; side < 2; ++side) {
                if (!m_external_sim->HasReceivedPowerWindowMotor(side)) continue;
                const std::uint8_t cmd = m_external_sim->GetPowerWindowMotor(side);
                if (cmd == m_last_pw_motor_cmd[side]) continue;
                m_last_pw_motor_cmd[side] = cmd;
                const char* name = (cmd <= 2u) ? kPwNames[cmd] : "UNKNOWN";
                // TODO: drive window position animation when geometry is available.
                std::cout << "[SimApp] Power window " << kSideNames[side]
                          << ": " << name << "\n";
            }
        }

        // --- Body actuator peripherals (door-lock motors / sounder / pump) ---
        ConsumeBodyActuatorPeripherals(tick_dt);

        // --- Horn audio (external-sim-driven only in headless) ---
        bool horn_low = false, horn_high = false;
        if (m_external_sim->IsConnected()) {
            horn_low  = m_external_sim->GetHornLowCmd();
            horn_high = m_external_sim->GetHornHighCmd();
        }
        m_horn->SetTones(horn_low, horn_high);

        // --- Telemetry logging ---
        m_telemetry->Record(m_world->GetState(), tick_dt);

        // --- Realtime pacing (sim-time vs wall-time) ---
        if (m_config.simulation.realtime) {
            const auto target = wall_start +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(t));
            std::this_thread::sleep_until(target);
        }

        // --- Scenario stats sampling ---
        if (m_scenario) {
            m_scenario->MaybeSampleStats(t, m_world->GetState(),
                                          *m_external_sim, cmd);
        }

        // --- Scripted-scenario complete ---
        if (m_scripted && m_scripted->IsDone()) {
            std::cout << "[SimApp] Scripted scenario complete at t="
                      << t << "s — exiting.\n";
            if (m_scenario) m_scenario->Close();
            RestoreSigint(old_sa);
            return kExitSuccess;
        }

        // --- Scenario complete (data-driven) ---
        if (m_scenario && m_scenario->IsDone(t)) {
            const bool failed = m_scenario->IsScenarioFailed();
            std::cout << "[SimApp] Scenario complete at t="
                      << t << "s "
                      << "(asserts: " << m_scenario->PassedAssertions()
                      << " passed, "  << m_scenario->FailedAssertions()
                      << " failed) — exiting.\n";
            m_scenario->Close();
            RestoreSigint(old_sa);
            return failed ? kExitScenarioAssertion : kExitSuccess;
        }

        // --- Max-time exit ---
        if (max_time > 0.0 && t >= max_time) {
            const bool scripted_unfinished = m_scripted && !m_scripted->IsDone();
            if (scripted_unfinished) {
                std::cerr << "[SimApp] max_time_s reached with scripted "
                             "scenario still in phase '"
                          << m_scripted->PhaseName() << "' — timeout.\n";
                RestoreSigint(old_sa);
                return kExitTimeout;
            }
            std::cout << "[SimApp] max_time_s reached — exiting.\n";
            if (m_scenario) m_scenario->Close();
            RestoreSigint(old_sa);
            return kExitSuccess;
        }
    }

    // Fell out of the loop -> SIGINT was the only possible cause.
    std::cout << "[SimApp] SIGINT — exiting.\n";
    if (m_scenario) m_scenario->Close();
    RestoreSigint(old_sa);
    return kExitInterrupted;
}

// ---------------------------------------------------------------------------
// Scenario hooks — dispatch physical-world events from a loaded Scenario.
// All hooks are no-ops when m_physical is null (shouldn't happen — SimApp
// always constructs PhysicalWorld), but defensive null-checks make these
// safe in tests that mock SimApp.
// ---------------------------------------------------------------------------
void SimApp::PushExtContractDriverInputs(const DriverCommand& cmd) {
    if (!m_external_sim) return;
    // (Horn 6940/6941 removed — the driver horn is a single contact (circuit
    //  28), published as the chassis horn cavity 4046 alongside the turn/hazard
    //  cavities in the render/headless publish blocks above.)
    // Headlight switch (6942) — CombinationSwitch::Position maps cleanly:
    //   OFF=0, PARK=1, ON=2, HI=3 — same numbering as the contract spec.
    if (m_physical) {
        m_external_sim->SetDriverHeadlightSwitch(static_cast<std::uint8_t>(
            m_physical->combination_switch().position()));
        // Headlight dim request (6943) — flash-to-pass lever held state.
        m_external_sim->SetDriverHeadlightDimRequest(
            m_physical->combination_switch().flash_to_pass_held());
    }
    // Telltale test (6945) — no UI source yet; stub at false.
    m_external_sim->SetDriverTelltaleTestRequest(false);
    // Park brake set/release (6946/6947) — edge-detect on the level
    // state from DriverCommand.parking_brake.  A false→true transition
    // fires the one-tick set request; true→false fires release.
    const bool pb_now = cmd.parking_brake;
    const bool pb_set_edge     = pb_now && !m_park_brake_prev;
    const bool pb_release_edge = !pb_now &&  m_park_brake_prev;
    m_external_sim->SetDriverParkBrakeSetRequest(pb_set_edge);
    m_external_sim->SetDriverParkBrakeReleaseRequest(pb_release_edge);
    m_park_brake_prev = pb_now;
    // Key position (6966) — ev1sim does not model a key-cycle state
    // machine; the simulator runs as if the vehicle is always RUN (2).
    // Future work: derive ACC/START transitions when an ignition model
    // lands in PhysicalWorld.
    m_external_sim->SetSensorKeyPosition(2);
}

void SimApp::KeyOnCycle() {
    if (m_physical) m_physical->rsa_keypad().cycle_k();
}
void SimApp::HeadlightCycle() {
    if (m_physical) m_physical->combination_switch().cycle_h();
}
void SimApp::PrndUp() {
    if (m_physical) m_physical->prnd_selector().cycle_up();
}
void SimApp::PrndDown() {
    if (m_physical) m_physical->prnd_selector().cycle_down();
}
void SimApp::TurnSignalLeft() {
    if (m_physical) m_physical->turn_signal_stalk().toggle_left();
}
void SimApp::TurnSignalRight() {
    if (m_physical) m_physical->turn_signal_stalk().toggle_right();
}
void SimApp::HazardToggle() {
    if (m_physical) m_physical->hazard_switch().toggle();
}
void SimApp::IpcTripResetPress() {
    if (m_physical) m_physical->ipc_trip_reset().press();
}
void SimApp::CruiseSet() {
    if (m_physical) m_physical->cruise_stalk().press_set();
}
void SimApp::CruiseResume() {
    if (m_physical) m_physical->cruise_stalk().press_resume();
}
void SimApp::CruiseCancel() {
    if (m_physical) m_physical->cruise_stalk().press_cancel();
}
void SimApp::CruiseSpeedUp() {
    if (m_physical) m_physical->cruise_stalk().press_speed_up();
}
void SimApp::CruiseSpeedDown() {
    if (m_physical) m_physical->cruise_stalk().press_speed_down();
}

void SimApp::DispatchAction(ev1sim::InputAction action) {
    using A = ev1sim::InputAction;
    switch (action) {
        case A::HeadlightCycle:  HeadlightCycle();    break;
        case A::TurnSignalLeft:  TurnSignalLeft();    break;
        case A::TurnSignalRight: TurnSignalRight();   break;
        case A::HazardToggle:    HazardToggle();      break;
        case A::WiperCycle:
            if (m_physical) m_physical->wiper_stalk().cycle_position();
            break;
        case A::WiperWash:
            if (m_physical) m_physical->wiper_stalk().press_wash();
            break;
        case A::CruiseSet:       CruiseSet();         break;
        case A::CruiseResume:    CruiseResume();      break;
        case A::CruiseCancel:    CruiseCancel();      break;
        case A::CruiseSpeedUp:   CruiseSpeedUp();     break;
        case A::CruiseSpeedDown: CruiseSpeedDown();   break;
        case A::PrndUp:          PrndUp();            break;
        case A::PrndDown:        PrndDown();          break;
        case A::KeyOnCycle:      KeyOnCycle();        break;
        case A::IpcTripReset:    IpcTripResetPress(); break;
        case A::ResetVehicle:    if (m_world) m_world->ResetVehicle(); break;
        case A::CameraCycle:     if (m_camera) m_camera->CycleMode();  break;
        case A::PauseToggle:
            m_paused = !m_paused;
            std::cout << (m_paused ? "[SimApp] PAUSED" : "[SimApp] RESUMED") << std::endl;
            break;
        case A::SnapshotToggle:  m_show_snapshot = !m_show_snapshot;   break;
        // Horn is a held/level input folded into DriverCommand, not dispatched.
        // The remaining actions have no button hook yet (need DriverCommand-level
        // or window-state plumbing) — log + ignore so a stray binding is visible.
        case A::Horn:
        case A::ParkingBrakeToggle:
        case A::UiModeToggle:
        case A::HelpToggle:
        case A::Quit:
        case A::None:
        default:
            std::cout << "[SimApp] wheel action '"
                      << ev1sim::InputActionToString(action)
                      << "' not wired for button dispatch (ignored)\n";
            break;
    }
}
