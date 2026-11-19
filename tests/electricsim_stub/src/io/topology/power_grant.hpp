// src/io/topology/power_grant.hpp — the one way to write a `class: effect` cell.
//
// An effect is a physical outcome a module reports about a load it drives: the blower
// turning, the defog grid heating, the drive motor carrying current. The outcome happens
// in the load, so it can be non-zero only while the load is energised. Each effect names
// its load's conductor in config/topology.yaml (`granted_by:`); this header reads that
// conductor and hands back the PowerGrant WireTable::write_effect_* demands. A dead grant
// writes zero, whatever the module asked for.

#pragma once

#include "../wire_table.hpp"
#include "feed_query.hpp"
#include "topology/topology_generated.h"

namespace electricsim {
namespace io {

class PowerGrantIssuer {
 public:
  // The grant for `effect` as the solver last published its granting conductor. An
  // effect with no granting conductor, or one the solver has never published, is dead.
  static PowerGrant issue(const WireTable& table, EffectId effect) {
    const ConductorId c = ::electricsim::topology::granting_conductor(effect);
    return PowerGrant{effect, c, energised(table, c)};
  }
};

// Write an effect with the grant its load holds right now. Returns whether the load is
// energised (and so whether `value` landed rather than zero). The typed forms mirror
// WireTable's write_* names.
inline bool write_effect_bit(WireTable& t, EffectId id, bool v) {
  const PowerGrant g = PowerGrantIssuer::issue(t, id);
  return t.write_effect_bit(id, v, g) && g.live();
}
inline bool write_effect_byte(WireTable& t, EffectId id, std::uint8_t v) {
  const PowerGrant g = PowerGrantIssuer::issue(t, id);
  return t.write_effect_byte(id, v, g) && g.live();
}
inline bool write_effect_uint16(WireTable& t, EffectId id, std::uint16_t v) {
  const PowerGrant g = PowerGrantIssuer::issue(t, id);
  return t.write_effect_uint16(id, v, g) && g.live();
}
inline bool write_effect_uint32(WireTable& t, EffectId id, std::uint32_t v) {
  const PowerGrant g = PowerGrantIssuer::issue(t, id);
  return t.write_effect_uint32(id, v, g) && g.live();
}
inline bool write_effect_float32(WireTable& t, EffectId id, float v) {
  const PowerGrant g = PowerGrantIssuer::issue(t, id);
  return t.write_effect_float32(id, v, g) && g.live();
}

}  // namespace io
}  // namespace electricsim
