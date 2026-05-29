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
/// Defaults are EV1 Gen 2 (NiMH) figures: a 343.2 V pack (26 × 13.2 V modules)
/// and a 102 kW peak motor.  Reactive magnetizing current is intentionally
/// ignored — it circulates in the inverter and does not appear on the DC bus.
/// Stateless, double precision.  Each entry point has an EV1-default overload
/// and an explicit-`Params` overload (a default `Params{}` argument can't be
/// formed inside this class while it is still incomplete).
class MotorCurrent {
public:
    /// EV1 Gen 2 NiMH pack: 26 modules × 13.2 V nominal.
    static constexpr double kPackVoltageV = 343.2;

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
        const double p_mech  = torque_nm * speed_rad_s;
        const double p_drive = (p_mech >= 0.0) ? p_mech / p.drive_efficiency
                                               : p_mech * p.regen_efficiency;
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
