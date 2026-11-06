#pragma once

// Where a tire's longitudinal grip peaks, as a function of road friction.
//
// Chrono's TMeasy scales the WHOLE slip curve by road friction: on a road at
// a quarter of the tire's reference friction both the peak force and the slip
// at which it occurs shrink to a quarter.  Published mu-slip curves do not do
// that.  The peak slip falls on slippery roads, but much more slowly than the
// friction does: about 17 % on dry asphalt, 13 % wet, 6 % on snow and 3 % on
// ice.  This table carries those points, and the EV1 tire moves its
// longitudinal peak (and its sliding onset by the same factor) to follow it
// instead of following friction.
//
// The table is data (EV1_TMeasyTire.json "Longitudinal Grip Peak"); this
// header is only the lookup.  Pure C++, no Chrono, so the lookup is
// unit-tested on its own.

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace ev1sim {

struct TireGripPeak {
    std::vector<double> friction;   ///< peak road friction, ascending
    std::vector<double> peak_slip;  ///< longitudinal slip at that peak

    bool empty() const { return friction.empty(); }

    /// Throws unless the table is usable: equal lengths, at least two points,
    /// friction strictly ascending, every slip positive.
    void validate() const {
        if (friction.size() != peak_slip.size() || friction.size() < 2)
            throw std::invalid_argument("TireGripPeak: need >= 2 paired points");
        for (std::size_t i = 0; i < friction.size(); ++i) {
            if (!(peak_slip[i] > 0.0))
                throw std::invalid_argument("TireGripPeak: peak slip must be > 0");
            if (i > 0 && !(friction[i] > friction[i - 1]))
                throw std::invalid_argument("TireGripPeak: friction must ascend");
        }
    }

    /// Peak slip at road friction `mu`: linear between points, held at the end
    /// points outside the table.
    double slip_at(double mu) const {
        if (mu <= friction.front()) return peak_slip.front();
        if (mu >= friction.back()) return peak_slip.back();
        const auto it = std::upper_bound(friction.begin(), friction.end(), mu);
        const std::size_t hi = static_cast<std::size_t>(it - friction.begin());
        const std::size_t lo = hi - 1;
        const double t = (mu - friction[lo]) / (friction[hi] - friction[lo]);
        return peak_slip[lo] + t * (peak_slip[hi] - peak_slip[lo]);
    }

    /// Factor on the tire's reference-friction peak slip for a road at `mu`,
    /// where the tire's slip parameters were written at `ref_mu`.  1 at the
    /// reference; TMeasy's own factor at the same road is mu / ref_mu.
    double slip_scale(double mu, double ref_mu) const {
        return slip_at(mu) / slip_at(ref_mu);
    }
};

}  // namespace ev1sim
