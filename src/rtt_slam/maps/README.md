# Maps

Everything here is in **ERC yard coordinates**: origin at start point S1, +Y towards S2,
+X to the right seen from above, Z up, metres. Nav2's `map` frame is this frame.

| File | What it is |
|---|---|
| `mars_yard_prior.pgm` / `.yaml` | Cost map of the yard for `nav2_map_server` (`mode: raw`): 0 = free, 1-99 = harder terrain, 100 = not drivable. 0.1 m per cell. |
| `mars_yard_prior.png` | Preview: elevation, cost map, and an example A* route. |
| `erc_2026_points.yaml` | Start points, waypoints and landmarks from the 2026 Mars Yard Update Report Rev 1.1. |

## Rebuilding the cost map

Run on a laptop (needs `numpy`, `scipy`, `pillow`, `matplotlib`), from the repository root:

```bash
python3 src/rtt_slam/tools/build_prior_map.py \
    src/rtt_gazebo_sim/models/mars_yard/meshes/terrain_visual.obj \
    src/rtt_gazebo_sim/models/mars_yard/materials/textures/terrain.jpg \
    src/rtt_slam/maps \
    --slope-lethal 20 --slope-free 5 --rough-lethal 0.12
```

- `--slope-lethal`: steepest slope the rover may drive, in degrees. Steeper cells get cost 100.
- `--slope-free`: slopes below this cost nothing.
- `--rough-lethal`: bump or dip height (relative to the surrounding 0.5 m) that blocks the rover, in metres.

The rover's size is **not** baked into the map. Nav2's inflation layer adds it from the
`footprint` in `config/navigation.yaml`.

## A new yard model

The transform from the model's own coordinates to ERC coordinates is hard-coded at the top of
`build_prior_map.py` (`THETA_DEG`, `TX`, `TY`, `PLANE`), together with the report's points.
It was fitted for the 2026 model. A new model needs new values there, and the same pose in
`rtt_gazebo_sim/worlds/mars_yard.sdf`.
