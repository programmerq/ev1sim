#pragma once

// Numeric ids for the cells ev1sim writes.  Needs the external sim's
// wire_table.hpp, so it is included only under EV1SIM_HAVE_WIRE_TRUTH.

#include "wire_table.hpp"   // electricsim::io::WireId, ElementStateId

namespace ev1sim {

// ProducerWireId — the numeric id of a cell ev1sim writes.  A plant or
// semantic cell's constant is a plain WireId.  A driver switch the topology
// models as a contact is an ElementStateId: ev1sim (the driver's hand) writes
// whether the contact is closed, and the wiring solver turns that into the
// conductors the modules read.  Both go over the same transport.  There is no
// ConductorId overload: ev1sim never writes a conductor.
inline constexpr electricsim::io::WireId ProducerWireId(electricsim::io::WireId id) { return id; }
inline constexpr electricsim::io::WireId ProducerWireId(electricsim::io::ElementStateId id) {
    return static_cast<electricsim::io::WireId>(id);
}

}  // namespace ev1sim
