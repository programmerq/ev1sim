#!/usr/bin/env python3
"""
Generate the level/flat_diagonal_mu.json file for the diagonal-µ
ABS scenario.

Real automotive "diagonal split-µ" testing puts FL+RR on one
friction surface and FR+RL on the other.  In a chrono flat-patch
ground, this can't be achieved as a single static layout because
patches are world-frame rectangles while the wheels track in chassis-
frame; we'd need the friction at each axle's x-position to differ
left vs right, with the LEFT/RIGHT pattern *flipping* every wheelbase.
The closest faithful model is therefore:

  - LEFT-side / RIGHT-side stripes that alternate every wheelbase
  - Within any wheelbase-long stretch the road is "split-µ"
    (asphalt one side, ice the other) — but as the car drives one
    wheelbase forward, the split flips so the formerly-asphalt side
    becomes ice and vice versa
  - The result is that at any instant during the brake event, the
    front and rear axles are on opposite splits — one diagonal pair
    on asphalt, the other on ice.  This is the diagonal pattern.

Period = 2 × wheelbase = 5.024 m.  Each stripe is wheelbase = 2.512 m
long, so the front axle and rear axle are always one stripe apart.

Acceleration runway (x < runway_x_end) is uniform asphalt so the
car can build speed before entering the diagonal zone.  Brake
fires inside the diagonal zone (set by the scenario's
wait_for_speed + at_time_s).
"""

import json
import sys
from pathlib import Path

WHEELBASE_M    = 2.512
ROAD_WIDTH_M   = 30.0           # full road width (y-range)
HALF_LANE_M    = ROAD_WIDTH_M / 2
# The runway has to hold the WHOLE pre-brake half of abs_diagonal_mu.json:
# the ramped launch to 18 m/s AND a settle off the throttle, both finished
# before the stripes start.  Measured on this vehicle, the launch alone needs
# ~63.5 m; the runway used to be 70 m with a 5 m spawn inset, so entry speed
# landed 1.5 m short of the boundary and the brake fired on the same tick the
# wait_for_speed barrier released — zero settle.  112.5 m covers the launch
# plus a ~2.5 s / ~44 m coasting settle before the stripes begin.
RUNWAY_START_X = -122.5
RUNWAY_END_X   = -10.0          # diagonal zone starts here
DIAGONAL_LEN_M = 250.0          # diagonal zone length
DIAGONAL_END_X = RUNWAY_END_X + DIAGONAL_LEN_M

ASPHALT_MU = 0.9
ICE_MU     = 0.08

ASPHALT_STYLE = {
    "type": "plane", "surface": "asphalt", "friction": ASPHALT_MU,
    "texture": "road.jpg", "texture_scale": 5.0,
}
ICE_STYLE = {
    "type": "plane", "surface": "ice", "friction": ICE_MU,
    "texture": "chrono:textures/bluewhite.png", "texture_scale": 10.0,
}


def make_patch(cx: float, cy: float, w: float, h: float, style: dict) -> dict:
    out = dict(style)
    out["center"] = [cx, cy, 0.0]
    out["size"]   = [w, h]
    return out


def build_level() -> dict:
    patches = []

    # 1. Acceleration runway: full asphalt before the diagonal zone.
    runway_w = RUNWAY_END_X - RUNWAY_START_X
    runway_cx = (RUNWAY_START_X + RUNWAY_END_X) / 2
    patches.append(make_patch(runway_cx, 0.0, runway_w, ROAD_WIDTH_M, ASPHALT_STYLE))

    # 2. Diagonal zone: alternating wheelbase-long stripes.
    #    Stripe N has phase A (+y asphalt, -y ice) when N is even,
    #    phase B (+y ice, -y asphalt) when N is odd.
    n_stripes = int(DIAGONAL_LEN_M / WHEELBASE_M) + 1
    for n in range(n_stripes):
        stripe_x0 = RUNWAY_END_X + n * WHEELBASE_M
        stripe_cx = stripe_x0 + WHEELBASE_M / 2
        if stripe_cx > DIAGONAL_END_X:
            break
        # +y patch and -y patch.
        if n % 2 == 0:
            top_style    = ASPHALT_STYLE
            bottom_style = ICE_STYLE
        else:
            top_style    = ICE_STYLE
            bottom_style = ASPHALT_STYLE
        patches.append(make_patch(stripe_cx,  HALF_LANE_M / 2,
                                  WHEELBASE_M, HALF_LANE_M, top_style))
        patches.append(make_patch(stripe_cx, -HALF_LANE_M / 2,
                                  WHEELBASE_M, HALF_LANE_M, bottom_style))

    # 3. Tail asphalt past the diagonal zone (in case the car overshoots).
    tail_w = 60.0
    tail_cx = DIAGONAL_END_X + tail_w / 2
    patches.append(make_patch(tail_cx, 0.0, tail_w, ROAD_WIDTH_M, ASPHALT_STYLE))

    return {
        "spawn": {"x": RUNWAY_START_X + 5.0, "y": 0.0, "z": 0.5,
                   "yaw_deg": 0.0},
        "patches": patches,
    }


def main(argv: list[str]) -> int:
    level = build_level()
    out_path = Path(argv[1]) if len(argv) > 1 \
        else Path(__file__).parent.parent / "level" / "flat_diagonal_mu.json"
    out_path.write_text(json.dumps(level, indent=2) + "\n")
    print(f"[diagonal-mu] wrote {out_path}  "
          f"({len(level['patches'])} patches; "
          f"diagonal zone {RUNWAY_END_X}..{DIAGONAL_END_X} m)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
