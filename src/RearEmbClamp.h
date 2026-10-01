#pragma once

#include "BrakeDrum.h"

namespace ev1sim {

/// Shoe clamping force held by one electromechanical (EMB) rear brake,
/// driven by the BTCM's signed rear-motor command
/// (CHASSIS_BTCM_EMB_MOTOR_CMD_LR / _RR).
///
/// The command is an H-bridge direction, not a force: the BTCM drives two
/// output pins per corner (apply, release) and electricsim publishes them as
/// +1, -1 or 0. What each means for the brake:
///
///   +1  motor drives the shoes on: clamp force goes to the commanded level
///       (full scale here, since the wire carries direction only);
///   -1  motor backs the shoes off: the brake releases;
///    0  motor not driven. In the BTCM's anti-lock HOLD this is the pawl
///       latch holding the actuator where it is (btcm_rear.c
///       regulation_motor_cmd_: inlet closed -> command 0, latch asserted),
///       so the clamp force is KEPT, not dropped.
///
/// Until 2026-09-28 SimApp::ApplyRearEmbBrake mapped every command at or
/// below 0 to zero shoe force, so a rear anti-lock HOLD released the rear
/// axle outright. On a low-grip stop, where the rears spend most of the stop
/// in HOLD, that took the whole rear axle out of the stop.
///
/// A stale or absent BTCM still means zero force: the EV1 rear brake has no
/// hydraulic backup, and a module that is not commanding the motors is not
/// holding a latch either (SimApp's BTCM-failure model).
class RearEmbClamp {
public:
    /// Advance with this step's command and return the shoe force (N).
    double Update(float cmd, bool fresh, const BrakeDrum::Params& p) {
        if (!fresh) {
            m_force_n = 0.0;
        } else if (cmd > 0.0f) {
            const double c = cmd > 1.0f ? 1.0 : double{cmd};
            m_force_n = c * p.max_shoe_force_n;
        } else if (cmd < 0.0f) {
            m_force_n = 0.0;
        }
        // cmd == 0 on a fresh BTCM: hold. The force is whatever it was.
        return m_force_n;
    }

    double ForceN() const { return m_force_n; }

private:
    double m_force_n = 0.0;
};

}  // namespace ev1sim
