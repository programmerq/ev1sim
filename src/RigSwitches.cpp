#include "RigSwitches.h"

#include "PhysicalWorld.h"

namespace ev1sim {

void ApplyRigSwitch(PhysicalWorld& world, RigSwitchTarget target, bool closed) {
    using T   = RigSwitchTarget;
    using Win = PowerWindows::Window;
    using Dir = PowerWindows::Direction;
    using Fan = HvacControls::Fan;
    using Mode = HvacControls::Mode;
    auto& hvac = world.hvac_controls();
    switch (target) {
        case T::None:                break;
        case T::SeatbeltDriver:      world.seatbelts().set_driver(closed); break;
        case T::SeatbeltPassenger:   world.seatbelts().set_passenger(closed); break;
        case T::ChargeCoupler:       world.charge_coupler().set_present(closed); break;
        case T::AirConditioning:     hvac.set_ac(closed); break;
        case T::Defrost:             hvac.set_defrost(closed); break;
        case T::WindowDriverUp:      world.power_windows().set_held(Win::DRIVER, Dir::UP, closed); break;
        case T::WindowDriverDown:    world.power_windows().set_held(Win::DRIVER, Dir::DOWN, closed); break;
        case T::WindowPassengerUp:   world.power_windows().set_held(Win::PASSENGER, Dir::UP, closed); break;
        case T::WindowPassengerDown: world.power_windows().set_held(Win::PASSENGER, Dir::DOWN, closed); break;
        case T::HvacFanOff:          if (closed) hvac.set_fan(Fan::OFF); break;
        case T::HvacFanLow:          if (closed) hvac.set_fan(Fan::LOW); break;
        case T::HvacFanMed:          if (closed) hvac.set_fan(Fan::MED); break;
        case T::HvacFanHigh:         if (closed) hvac.set_fan(Fan::HIGH); break;
        case T::HvacModeFace:        if (closed) hvac.set_mode(Mode::FACE); break;
        case T::HvacModeBilevel:     if (closed) hvac.set_mode(Mode::BILEVEL); break;
        case T::HvacModeFeet:        if (closed) hvac.set_mode(Mode::FEET); break;
        case T::HvacModeDefrost:     if (closed) hvac.set_mode(Mode::DEFROST); break;
    }
}

}  // namespace ev1sim
