#pragma once

#include "VehicleLights.h"
#include "VehiclePanels.h"

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
///   - 19 bulb command inputs      (electric sim -> ev1sim), one per LightID
///   - 2  horn tone command inputs (electric sim -> ev1sim), low + high
///   - 4  panel ajar switch outputs (ev1sim -> electric sim), hood/trunk/L/R door
///
/// Signal IDs live in the 4000-block so they don't collide with the harness
/// example's 3000-block.
class ExternalSimConnector {
public:
    struct Options {
        bool        enabled          = false;
        std::string bus_name         = "electricsim_harness_bus";
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

    /// An externally-visible endpoint on the I/O fabric.
    struct Endpoint {
        std::uint32_t signal_id;
        const char*   qualified_name;   // e.g. "vehicle.body.lhbh.bulb_cmd"
        const char*   short_name;       // e.g. "lhbh_bulb_cmd"
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
