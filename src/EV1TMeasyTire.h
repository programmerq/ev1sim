#pragma once

// EV1 tire: Chrono's TMeasy with first-order longitudinal tire dynamics.
//
// WHY THIS CLASS EXISTS
// ---------------------
// Chrono 9's ChTMeasyTire is a STEADY-STATE slip model: every step it maps
// the current slip sx = -(vx - R*omega) / (R*|omega| + vnum) straight to a
// force, and the vehicle integrates the wheel spin with that force frozen for
// the step.  The force's sensitivity to wheel speed is dFx/domega ~ C*R/v
// (C = slip stiffness dFx/dsx), so the wheel-spin equation behaves like a
// damper whose coefficient grows as 1/v.  An explicitly applied damper is
// stable only while h * C * R^2 / (I * v) < 2 — i.e. above a critical speed
// that scales with the step h.  Below it the wheel over-corrects every step
// and the slip rings.  At the 1 ms step the EV1 runs, that is a sustained
// +-20..30 % slip oscillation from ~1.7 m/s (where TMeasy's friction blend
// starts handing the force from the Dahl model to the slip curve) up to
// ~5-7 m/s, measured on a dry half-pedal launch with no external fleet.
// The driven fronts ring in anti-phase (the open differential's symmetric
// mode carries the motor inertia and stays stable; the anti-phase mode sees
// only wheel inertia), the undriven rears ring on their own.  It halves at a
// 0.5 ms step and is gone at 0.25 ms: a numerical instability, not physics.
//
// THE FIX IS THE PHYSICS CHRONO 9 DROPPED
// ---------------------------------------
// A real tire does not generate slip force instantaneously: the tread and
// carcass deflect first.  TMeasy's own formulation (Rill; Hirschberg, Rill &
// Weinfurter, "Tire model TMeasy", 2007) models that as a spring cx and
// damper dx in SERIES with the slip-force characteristic, a first-order
// "tire dynamics" state xe.  Chrono 8.0 implemented exactly that
// (ChTMeasyTire.cpp, m_consider_relaxation, cx = 0.9*CZ,
// dx = damping_ratio*sqrt(cx*m_tire)); Chrono 9 removed it.  This class puts
// the longitudinal half back, as a correction to the force the base class
// computed:
//
//     ks      = fos / vta                  (slip force per m/s sliding speed)
//     Fx_ss   = -ks * vsx                  (the base's steady-state TMeasy Fx)
//     xe_dot  = (Fx_ss - cx*xe) / (dx + ks)        [semi-implicit in xe]
//     Fx_dyn  = cx*xe + dx*xe_dot
//
// With the series spring the force's sensitivity to wheel speed is bounded by
// dx (plus cx*h), not by C/v, so the step no longer has to shrink as the car
// slows.  In steady state xe_dot -> 0 and Fx_dyn -> Fx_ss exactly, so
// coastdown, top speed and any steady slip are unchanged; only the transient
// (a relaxation length of ~ C/cx, 0.2-0.3 m here) is new.
//
// Scope: longitudinal only, and only on the TMeasy share of the force — the
// low-speed Dahl bristle model (below the friction blend, < 1 m/s) is left
// exactly as Chrono computes it, so standstill holding is untouched.

#include "chrono_vehicle/wheeled_vehicle/tire/TMeasyTire.h"

#include <memory>
#include <string>

namespace ev1sim {

class EV1TMeasyTire : public chrono::vehicle::TMeasyTire {
  public:
    /// Reads the stock TMeasy JSON plus the "Longitudinal Tire Dynamics"
    /// block ("Stiffness [N/m]", "Damping [Ns/m]").  A file without the block
    /// behaves exactly like Chrono's TMeasyTire.
    explicit EV1TMeasyTire(const std::string& filename);

    double GetLongitudinalTireStiffness() const { return m_cx; }
    double GetLongitudinalTireDamping() const { return m_dx; }
    /// Current tread deflection state xe [m].
    double GetLongitudinalDeflection() const { return m_xe; }

  protected:
    virtual void Advance(double step) override;

  private:
    double m_cx = 0.0;  ///< longitudinal tire (tread+carcass) stiffness [N/m]
    double m_dx = 0.0;  ///< longitudinal tire damping [N.s/m]
    double m_xe = 0.0;  ///< longitudinal deflection state [m]
};

/// Build the EV1 tire from its JSON (the factory VehicleWorld and the plant
/// tests share, so both run the same tire).
std::shared_ptr<EV1TMeasyTire> ReadEV1TireJSON(const std::string& filename);

}  // namespace ev1sim
