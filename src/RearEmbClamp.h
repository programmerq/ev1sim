#pragma once

#include <limits>

#include "BrakeDrum.h"

namespace ev1sim {

/// Rear brake "Maximum Torque" (N·m) — converts the physical drum torque from
/// the BrakeDrum model into the [0,1] ratio Chrono's brake takes.  MUST equal
/// data/vehicle/ev1/brake/EV1_BrakeSimple_Rear.json's "Maximum Torque"
/// (Round 4 brake bias).  Lives here, in a Chrono-free header, so the unit
/// suite can check the JSON against the code constant itself rather than a
/// re-typed literal; SimApp::kRearBrakeMaxTorqueNm aliases it.
inline constexpr double kRearBrakeMaxTorqueNm = 480.0;

/// Shoe clamping force held by one electromechanical (EMB) rear brake,
/// driven by the BTCM's signed rear-motor command
/// (CHASSIS_BTCM_EMB_MOTOR_CMD_LR / _RR).
///
/// The rear brake is a drum whose shoes are pushed by "an electric motor pack
/// (motor and gear assembly) ... rather than a hydraulic wheel cylinder"
/// (@source:manual brakes-312 REAR BRAKE APPLY SYSTEM).  So the BTCM command
/// is a motor drive, not a force: the BTCM drives two output pins per corner
/// (apply, release) and electricsim publishes their signed duty in [-1, 1].  The model
/// integrates that drive into actuator travel and derives the shoe force from
/// where the actuator is:
///
///   +1  motor drives the shoes out.  From home the actuator first takes up
///       the running clearance with no force (the motor runs against "little
///       or no mechanical resistance", @source:manual brakes-254 DTC 147),
///       then the shoes meet the drum and force rises with further travel
///       until it reaches the EMB's maximum, where the motor stalls.
///   -1  motor backs the shoes off: travel and force fall at the same rate
///       until the actuator is at home, the stop the scan tool reports as
///       "RELEASE when motor in home position" (@source:manual brakes-73
///       LR/RR MOTOR).
///    0  motor not driven.  In the BTCM's anti-lock HOLD this is the pawl
///       latch holding the actuator where it is (btcm_rear.c
///       regulation_motor_cmd_: inlet closed -> command 0, latch asserted),
///       so the travel, and with it the clamp force, is KEPT, not dropped.
///       Reading the 0 as "no force" would release the rear axle every time
///       the anti-lock kernel holds it, which on a low-grip stop is most of
///       the stop.
///
/// A fractional command scales the motor speed: the BTCM current-controls the
/// apply motor (US5163743 Fig 10), and its duty reaches ev1sim on the lead
/// conductors as (APPLY − RELEASE) / 1000.
///
/// A stale or absent BTCM means the actuator is at home and the force is zero:
/// the EV1 rear brake has no hydraulic backup, and a module that is not
/// commanding the motors is not holding a latch either (SimApp's BTCM-failure
/// model).  When the BTCM comes back, the clearance has to be taken up again
/// before the rear brakes.
///
/// Travel is measured in seconds of full-speed motor drive from home, so the
/// two Travel values are the times the motor needs for each phase.  Both are
/// inferred (the manual gives no stroke or apply time), chosen so a full
/// stroke at full drive takes the same 300 ms as BrakeActuator's rear
/// rate limit; they are not EV1 facts.  The force rises linearly with
/// travel past contact (one spring-like compliance for shoes, drum and gear
/// train), also an inference.
class RearEmbClamp {
public:
    struct Travel {
        /// Full-speed drive time to take up the running clearance from home
        /// to shoe-on-drum contact (s).  Inferred.
        double clearance_s = 0.050;
        /// Full-speed drive time from contact to the EMB's maximum shoe
        /// force, where the motor stalls (s).  Inferred.
        double rise_s = 0.250;
    };

    RearEmbClamp() = default;
    explicit RearEmbClamp(const Travel& travel) : m_travel(travel) {}

    /// Advance with this step's command over dt_s seconds and return the shoe
    /// force (N).  Without dt_s the actuator settles within the step: an APPLY
    /// reaches the maximum force, a RELEASE reaches home, and a HOLD keeps
    /// where it is.  SimApp::ApplyRearEmbBrake passes the physics step.
    double Update(float cmd, bool fresh, const BrakeDrum::Params& p,
                  double dt_s = kSettleWithinStep) {
        if (!fresh) {
            m_travel_s = 0.0;
        } else if (cmd != 0.0f) {
            const double speed =
                cmd > 1.0f ? 1.0 : (cmd < -1.0f ? -1.0 : double{cmd});
            m_travel_s += speed * dt_s;
            const double stall = m_travel.clearance_s + m_travel.rise_s;
            if (!(m_travel_s > 0.0)) m_travel_s = 0.0;  // home stop
            if (m_travel_s > stall) m_travel_s = stall;  // motor stall
        }
        // cmd == 0 on a fresh BTCM: hold. The travel is whatever it was.
        m_force_n = ForceAtTravel(p);
        return m_force_n;
    }

    double ForceN() const { return m_force_n; }

    /// Actuator travel from home, in seconds of full-speed drive.
    double TravelS() const { return m_travel_s; }

    /// True once the shoes are against the drum.
    bool InContact() const { return m_travel_s > m_travel.clearance_s; }

    /// True when the actuator is at its home (released) stop.
    bool AtHome() const { return m_travel_s <= 0.0; }

    const Travel& TravelParams() const { return m_travel; }

    static constexpr double kSettleWithinStep =
        std::numeric_limits<double>::infinity();

private:
    double ForceAtTravel(const BrakeDrum::Params& p) const {
        const double past_contact = m_travel_s - m_travel.clearance_s;
        if (past_contact <= 0.0) return 0.0;
        if (m_travel.rise_s <= 0.0 || past_contact >= m_travel.rise_s) {
            return p.max_shoe_force_n;
        }
        return p.max_shoe_force_n * (past_contact / m_travel.rise_s);
    }

    Travel m_travel;
    double m_travel_s = 0.0;
    double m_force_n = 0.0;
};

}  // namespace ev1sim
