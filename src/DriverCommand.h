#pragma once

// Command structure sent from any input source to the vehicle.
// Decoupled from Chrono types so future socket/playback controllers
// can produce these without depending on the physics engine.
struct DriverCommand {
    double throttle    = 0.0;   // 0..1
    double front_brake = 0.0;   // 0..1  (first pass: keyboard sets both identically)
    double rear_brake  = 0.0;   // 0..1
    double steering    = 0.0;   // -1..1  (positive = left turn, Chrono convention)
    bool   parking_brake  = false;
    bool   reset_vehicle  = false;
};
