#pragma once

namespace ev1sim {

/// DC traction-pack current model (Chrono-free).
///
/// Chrono's `ChEngineSimpleMap` gives shaft torque and speed but no electrical
/// current.  This recovers the **DC bus current** an EV ammeter would show from
/// the mechanical operating point:
///
///   P_mech = T · ω                                  (signed: + motoring, − regen)
///   P_drive = P_mech / η_drive   (motoring)         (you draw *more* than P_mech)
///           = P_mech · η_regen   (regen)            (you recover *less* than |P_mech|)
///   P_pack = P_drive + P_accessory                  (12 V hotel load is always drawn)
///   I_dc   = P_pack / V_pack
///
/// Sign convention: **positive current = discharge** (power leaving the pack),
/// negative = regen charging.  This is the conventional EV energy-display sign.
///
/// Defaults are EV1 figures: a 312 V nominal pack (26 modules × 12 V
/// nominal) and a 102 kW peak motor.  The previous 343.2 V (26 × 13.2 V)
/// was a charged/float per-module figure misread as nominal — every
/// derived current was ~10% low for a given power.  Reactive magnetizing current is intentionally
/// ignored — it circulates in the inverter and does not appear on the DC bus.
/// Stateless, double precision.  Each entry point has an EV1-default overload
/// and an explicit-`Params` overload (a default `Params{}` argument can't be
/// formed inside this class while it is still incomplete).
///
/// ── KNOWN LIMITATION: regen is display-only; no regen↔friction blend (S15) ──
/// The signed current this class produces is a *readout* of the mechanical
/// operating point (chassis bus `motor_current_a`, 4072) — an ammeter value.
/// It does NOT feed wheel torque.  Friction braking (front BrakeSimple disc,
/// rear BrakeDrum EMB) is computed entirely independently of motor current,
/// and `DriverCommand` has no regen / propulsion-torque channel into the brake
/// path.  Consequences:
///   - There is no regen+friction *blend*: a brake application does not reduce
///     friction torque by the regen contribution, and releasing throttle does
///     not add a regen braking torque to the wheels (the coast-torque map in
///     EV1_EngineSimpleMap.json is residual drag, explicitly NOT commanded
///     regen — see docs/ev1_chrono_audit.md §3).
///   - The safety-relevant **regen-cutout fail-safe** (regen drops out → the
///     friction system must backfill the lost deceleration) is therefore
///     **untestable in-sim**: the two systems are not coupled, so there is no
///     regen torque to cut and no friction backfill to verify.
/// This is documented rather than fabricated: faithfully modelling the blend
/// would need EV1 regen-torque-vs-speed and blend-handover constants the
/// manuals do not provide, and inventing them would be worse than the honest
/// gap.  See docs/ev1_chrono_audit.md §15 and docs/TODO.md for the follow-up
/// marker (gated on measured/traceable EV1 regen-blend structure).
class MotorCurrent {
public:
    /// EV1 pack nominal: 26 modules × 12 V nominal = 312 V.  (13.2 V per
    /// module is a charged/float value, not nominal — see class comment.)
    static constexpr double kPackVoltageV = 312.0;

    /// Combined inverter + motor efficiency while motoring.
    static constexpr double kDriveEfficiency = 0.90;

    /// Combined efficiency of the regen path back into the pack.
    static constexpr double kRegenEfficiency = 0.90;

    /// 12 V accessory ("hotel") load drawn through the DC-DC converter (W),
    /// referred to the HV bus.  Always present; ~1.2 kW covers lights, pumps,
    /// and the vehicle computers.
    static constexpr double kAccessoryLoadW = 1200.0;

    struct Params {
        double pack_voltage_v   = kPackVoltageV;
        double drive_efficiency = kDriveEfficiency;
        double regen_efficiency = kRegenEfficiency;
        double accessory_load_w = kAccessoryLoadW;
    };

    /// Electrical power drawn from the pack (W, signed: + discharge, − charge)
    /// for a signed shaft torque (N·m) and speed (rad/s).
    static double pack_power_w(double torque_nm, double speed_rad_s, const Params& p) {
        const double p_mech = torque_nm * speed_rad_s;
        double p_drive;
        if (p_mech >= 0.0) {
            // Motoring: draw more than P_mech.  Guard the division so a
            // misconfigured (non-positive) efficiency can't leak inf/NaN into
            // published telemetry — fall back to a lossless pass-through.
            p_drive = (p.drive_efficiency > 0.0) ? p_mech / p.drive_efficiency
                                                 : p_mech;
        } else {
            // Regen: recover less than |P_mech| (multiplication is always finite).
            p_drive = p_mech * p.regen_efficiency;
        }
        return p_drive + p.accessory_load_w;
    }
    static double pack_power_w(double torque_nm, double speed_rad_s) {
        return pack_power_w(torque_nm, speed_rad_s, Params{});
    }

    /// DC bus current (A, signed: + discharge from pack, − regen charge).
    static double dc_bus_current_a(double torque_nm, double speed_rad_s, const Params& p) {
        if (p.pack_voltage_v <= 0.0) return 0.0;   // defensive
        return pack_power_w(torque_nm, speed_rad_s, p) / p.pack_voltage_v;
    }
    static double dc_bus_current_a(double torque_nm, double speed_rad_s) {
        return dc_bus_current_a(torque_nm, speed_rad_s, Params{});
    }
};

}  // namespace ev1sim
