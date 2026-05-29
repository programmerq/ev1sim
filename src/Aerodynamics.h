#pragma once

namespace ev1sim {

/// Body aerodynamic drag model (Chrono-free).
///
/// Steady-state drag on a road vehicle is the classic quadratic:
///
///   F_drag = 0.5 * rho * Cd * A * v^2
///
/// where:
///   rho — air density (kg/m^3)
///   Cd  — drag coefficient (dimensionless)
///   A   — reference frontal area (m^2)
///   v   — speed through the air (m/s)
///
/// This mirrors exactly what Chrono::Vehicle's
/// `ChChassis::SetAerodynamicDrag(Cd, area, air_density)` applies internally
/// at the chassis COM each Synchronize.  Keeping the formula + the EV1
/// constants here gives one source of truth that the unit tests pin, so the
/// applied drag is verifiable without booting Chrono.
///
/// The EV1 is the relevant reference point: its production drag coefficient of
/// 0.19 was the lowest of any mass-produced car of its era, and most of that
/// efficiency advantage lived in cutting aerodynamic losses at speed.
///
/// All calculations are stateless and use double precision.  Lift, yaw-angle
/// (crosswind) effects, and ground-effect are intentionally ignored — this is
/// pure longitudinal drag.  Each entry point has an EV1-default overload and an
/// explicit-`Params` overload (a default `Params{}` argument can't be formed
/// inside this class while it is still incomplete).
class Aerodynamics {
public:
    /// GM-published EV1 drag coefficient.
    static constexpr double kEV1DragCoefficient = 0.19;

    /// EV1 reference frontal area (m^2), ~20.3 sq ft.  Not quoted to high
    /// precision in the literature; refine if a measured figure surfaces.
    static constexpr double kEV1FrontalAreaM2 = 1.89;

    /// ISA sea-level air density (kg/m^3) at 15 °C / 101.325 kPa.
    static constexpr double kAirDensityIsaSeaLevel = 1.225;

    struct Params {
        double cd           = kEV1DragCoefficient;
        double frontal_area = kEV1FrontalAreaM2;
        double air_density  = kAirDensityIsaSeaLevel;
    };

    /// Drag force magnitude (N, always >= 0) at the given airspeed (m/s).
    /// Magnitude only — the direction (opposing travel) is the caller's job.
    static double drag_force_n(double speed_mps, const Params& p) {
        return 0.5 * p.air_density * p.cd * p.frontal_area * speed_mps * speed_mps;
    }
    static double drag_force_n(double speed_mps) {
        return drag_force_n(speed_mps, Params{});
    }

    /// Lumped drag area CdA (m^2) — a handy single-number comparison metric.
    static double cda_m2(const Params& p) { return p.cd * p.frontal_area; }
    static double cda_m2() { return cda_m2(Params{}); }

    /// Mechanical power (W) required to overcome drag at a steady speed:
    /// P = F * v.  Useful as a range / efficiency sanity check.
    static double drag_power_w(double speed_mps, const Params& p) {
        return drag_force_n(speed_mps, p) * speed_mps;
    }
    static double drag_power_w(double speed_mps) {
        return drag_power_w(speed_mps, Params{});
    }
};

}  // namespace ev1sim
