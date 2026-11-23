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
inline constexpr double kRearBrakeMaxTorqueNm = 513.0;

/// Shoe clamping force held by one electromechanical (EMB) rear brake,
/// driven by the BTCM's signed rear apply motor command and its park latch
/// motor.
///
/// The rear brake is a drum whose shoes are pushed by "an electric motor pack
/// (motor and gear assembly) ... rather than a hydraulic wheel cylinder"
/// (@source:manual brakes-312 REAR BRAKE APPLY SYSTEM).  The BTCM drives two
/// leads per apply motor and electricsim publishes their signed duty in
/// [-1, 1] (electricsim docs/3d_sim_contract.md).  The model integrates
/// actuator travel and derives the shoe force from where the actuator is:
/// past the running clearance, force rises linearly with travel to the EMB's
/// maximum.
///
/// The apply is current-controlled: motor torque, and so the clamp it can
/// hold, follows the current (@source:patent US5366281 Fig 10), and a shoe
/// return spring backdrives the motor whenever its torque is short of the
/// clamp (@source:patent US5366281 rehome Mode 1: "the return spring ... back
/// drives the motor").  So:
///
///   c > 0  the actuator moves toward the travel at which the clamp is
///          c × maximum: out at c × full speed while short of it, back at the
///          spring's speed while past it.  It holds there for as long as the
///          current lasts.
///   c = 0  the motor is not driven: past the running clearance the spring
///          backdrives it until the shoes leave the drum.  Inside the
///          clearance nothing pushes the shoes, so the actuator stays.
///   c < 0  the motor backs the shoes off at |c| × full speed to the home
///          stop, "RELEASE when motor in home position" (@source:manual
///          brakes-73 LR/RR MOTOR).
///
/// The PARK LATCH is a second motor (@source:patent US5366281 Fig 18a/18b):
/// driven to the end of its latch stroke it sets the latch, driven to the end
/// of its unlatch stroke it clears it, and undriven it stays as it is.  While
/// set, the actuator cannot move toward home (a ratchet), so the clamp holds
/// with no current at all, including with the BTCM dead.
///
/// A stale or absent BTCM drives nothing: the apply motor is undriven (c = 0),
/// so an unlatched rear brake lets go and a latched one keeps its clamp.  The
/// EV1 rear has no hydraulic backup.
///
/// Travel is measured in seconds of full-speed motor drive from home.  The
/// clearance, rise, spring speed and latch stroke are inferred (the manual
/// gives none of them), chosen so a full stroke at full drive takes 300 ms;
/// they are not EV1 facts.  The force rises linearly with travel past contact
/// (one spring-like compliance for shoes, drum and gear train), also an
/// inference.
class RearEmbClamp {
public:
    struct Travel {
        /// Full-speed drive time to take up the running clearance from home
        /// to shoe-on-drum contact (s).  Inferred.
        double clearance_s = 0.050;
        /// Full-speed drive time from contact to the EMB's maximum shoe
        /// force, where the motor stalls (s).  Inferred.
        double rise_s = 0.250;
        /// Speed, as a fraction of full drive, at which the return spring
        /// backdrives an actuator holding more clamp than its current
        /// supports.  Inferred.
        double spring_speed = 1.0;
        /// Park latch motor drive time from one end of its stroke to the
        /// other (s).  Inferred, inside the BTCM's 50 ms latch drive.
        double latch_stroke_s = 0.040;
    };

    RearEmbClamp() = default;
    explicit RearEmbClamp(const Travel& travel) : m_travel(travel) {}

    /// Advance with this step's apply command `cmd` and park latch drive
    /// `latch` (+1 latch lead, -1 unlatch lead, 0 undriven) over dt_s seconds
    /// and return the shoe force (N).  Without dt_s every move completes
    /// within the step.  SimApp::ApplyRearEmbBrake passes the physics step.
    double Update(float cmd, int latch, bool fresh, const BrakeDrum::Params& p,
                  double dt_s = kSettleWithinStep) {
        StepLatch(latch, dt_s);
        const double c = !fresh ? 0.0
                         : cmd > 1.0f ? 1.0
                         : (cmd < -1.0f ? -1.0 : double{cmd});
        if (c > 0.0) {
            const double target = m_travel.clearance_s + c * m_travel.rise_s;
            if (m_travel_s < target) {
                MoveToward(target, c * dt_s);
            } else {
                MoveToward(target, m_travel.spring_speed * dt_s);
            }
        } else if (c < 0.0) {
            MoveToward(0.0, -c * dt_s);
        } else if (m_travel_s > m_travel.clearance_s) {
            MoveToward(m_travel.clearance_s, m_travel.spring_speed * dt_s);
        }
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

    /// True while the park latch is set.
    bool Latched() const { return m_latched; }

    const Travel& TravelParams() const { return m_travel; }

    static constexpr double kSettleWithinStep =
        std::numeric_limits<double>::infinity();

private:
    void StepLatch(int latch, double dt_s) {
        if (latch == 0) return;
        const double stroke = m_travel.latch_stroke_s;
        m_latch_s += (latch > 0 ? dt_s : -dt_s);
        if (m_latch_s >= stroke) {
            m_latch_s = stroke;
            m_latched = true;
        } else if (m_latch_s <= 0.0) {
            m_latch_s = 0.0;
            m_latched = false;
        }
    }

    /// Move travel toward `target` by at most `step` (s); a set latch stops
    /// any move toward home.
    void MoveToward(double target, double step) {
        if (m_travel_s > target) {
            if (m_latched) return;
            m_travel_s = (m_travel_s - target <= step) ? target : m_travel_s - step;
        } else if (m_travel_s < target) {
            m_travel_s = (target - m_travel_s <= step) ? target : m_travel_s + step;
        }
    }

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
    double m_latch_s = 0.0;
    bool m_latched = false;
    double m_force_n = 0.0;
};

}  // namespace ev1sim
