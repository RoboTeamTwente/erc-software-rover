# Maps

Everything here is in **ERC yard coordinates**: origin at start point S1, +Y towards S2,
+X to the right seen from above, Z up, metres. Nav2's `map` frame is this frame.

| File | What it is |
|---|---|
| `mars_yard_prior.pgm` / `.yaml` | Cost map of the yard for `nav2_map_server` (`mode: raw`): 0 = free, 1-99 = harder terrain, 100 = not drivable. 0.1 m per cell. |
| `mars_yard_prior.png` | Preview: elevation, cost map, and an example A* route. |
| `erc_2026_points.yaml` | Start points, waypoints, landmarks and the deep sampling point (P1) from the 2026 Mars Yard Update Report Rev 1.1. Single source for the sim world and the RTAB-Map marker priors. |

## Rebuilding the cost map

Run on a laptop (needs `numpy`, `scipy`, `pillow`, `matplotlib`), from the repository root:

```bash
python3 src/rtt_navigation/tools/build_prior_map.py \
    src/rtt_gazebo_sim/models/erc2026_terrain/meshes/erc2026_visual.obj \
    src/rtt_gazebo_sim/models/erc2026_terrain/meshes/textures/erc2026_orthophoto.jpg \
    src/rtt_navigation/maps \
    --slope-lethal 20 --slope-free 5 --rough-lethal 0.12
```

- `--slope-lethal`: steepest slope the rover may drive, in degrees. Steeper cells get cost 100.
- `--slope-free`: slopes below this cost nothing.
- `--rough-lethal`: bump or dip height (relative to the surrounding 0.5 m) that blocks the rover, in metres.

The rover's size is **not** baked into the map. Nav2's inflation layer adds it from the
`footprint` in `config/navigation.yaml`.

## A new yard model

`erc2026_terrain` is already in ERC X/Y, so `THETA_DEG`, `TX`, `TY` at the top of
`build_prior_map.py` are 0. `PLANE` is the levelling to the report's heights; it must match the
link pose in `rtt_gazebo_sim/models/erc2026_terrain/model.sdf` (`LEVEL` in
`rtt_gazebo_sim/tools/gen_world.py`). Landmark posts are not in the scan, so the tool marks them
lethal (`--landmark-radius`, default 0.18 m).
