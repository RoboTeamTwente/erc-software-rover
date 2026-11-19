# rtt_gz_worlds

Gazebo Harmonic world of the **ERC 2026 Mars Yard**, built from the files ERC published:
the 3D mesh, the orthophoto, `Coordinates_MarsYard2026.txt` and `Coord_Sys.pdf`.
It includes the terrain, the 15 ArUco landmark posts, and all official points.

```bash
colcon build --packages-select rtt_gz_worlds
source install/setup.bash
ros2 launch rtt_gz_worlds erc2026.launch.py                 # with coloured S/W/P point markers
ros2 launch rtt_gz_worlds erc2026.launch.py markers:=false  # clean world, use this for SLAM
```

No rover is spawned yet.

## Frame

Gazebo world frame = **ERC local frame**: origin at start point S1, +X / +Y as drawn in
`Coord_Sys.pdf`, Z up, metres. So an ERC coordinate can be used in Gazebo as-is.
S1 and S2 are ERC's calibration points (x = 0).

- The raw `Model3D_mesh2.obj` is already Z-up in this frame. (Blender shows it differently
  because its OBJ importer converts Y-up to Z-up by default.)
- The `.txt` lists **Y before X**. `tools/gen_world.py` handles that.
- The orthophoto GeoTIFF is in the same frame (1 cm/px, top-left corner at x = -18.552, y = 35.316),
  so it's used directly as the terrain texture. W1 sits exactly on a yellow marker visible in it.

### Levelling (please confirm)

The coordinate file gives a height for every point (`H (local)`). The 3D mesh disagrees with those
heights by a tilt of about **1.35° about X** and **-0.60° about Y**. That's about 0.6 m at the far end
of the yard: the file puts S2 0.14 m above S1, but the mesh puts it 0.46 m below.

`model.sdf` rotates the terrain slightly so it matches the file. The fit uses only the 19 points that
are on the ground (S, W, P) and leaves them within 3 cm RMS. We assume the file's heights are the
gravity-true reference; nobody has confirmed that. To keep the mesh as ERC delivered it, set
`LEVEL` in `tools/gen_world.py` to zeros and re-run it.

## Points (`config/erc2026_points.yaml`)

| prefix | type | count | in the world |
|---|---|---|---|
| S | start | 9 | green marker (markers world only) |
| W | waypoint | 9 | blue marker (markers world only) |
| L | landmark | 15 | ArUco post model (always) |
| P | deep_sampling | 1 | magenta marker (markers world only) |

Fields:
- `x`, `y`, `z`: as given by ERC.
- `ground_z`: height of the simulated terrain at that point. Use it as the spawn height.

**Start heading:** per the rulebook, the start position and a "limited heading" are drawn at each
trial. The heading isn't in the file, so pass it when spawning.

## Landmarks (`models/erc2026_landmark_NN`)

Built to rulebook fig. 9 and 10:
- **Box:** 250 × 250 × 320 mm, sitting on a pole.
- **Tag:** 150 mm ArUco marker plus a number panel, same on all four faces.
- **Height:** tag centre 417 mm above ground.
- **IDs:** landmark N uses **OpenCV `DICT_5X5_250` id 50 + N, i.e. ids 51–65**. All 15 were checked
  visually against rulebook fig. 10, and all 60 faces decode to the right id when rendered through the
  mesh's UVs.

Things that are open or assumed:
- **ID range:** the rulebook text says ids "51-64", which is 14 ids for 15 landmarks. The figure
  matches 51–65. The final list is promised in Update Report #3.
- **Box height:** the rulebook text says 310 mm, the drawing says 320 mm. 320 is used.
- **Pole diameter:** not dimensioned; ~48 mm from the drawing.
- **Yaw:** unknown, so faces are aligned with X/Y.
- **Landmark heights:** the rulebook says landmark coordinates are given at the tag centre height. But
  in the coordinate file, the L heights match the ground (mean +1 cm vs terrain), not ground + 0.417 m.
  So the posts stand on the terrain at (x, y), with the tag at 0.417 m.
- **Not in the scan:** the posts weren't there when the mesh and orthophoto were captured.

## Meshes

| file | source | size |
|---|---|---|
| `erc2026_visual.obj` + orthophoto texture | 10 cm vertex clustering, 289k triangles | 26 MB + 5 MB |
| `erc2026_collision.obj` | 20 cm vertex clustering, 76k triangles, with vertex normals | 4.8 MB |

The original mesh has 1.68M triangles (161 MB), which is over GitHub's 100 MB file limit.
Measured against it, the decimated surfaces differ by a median of 0.2 cm (visual) and 0.4 cm
(collision) in height; 95th percentile 3.7 cm and 5.0 cm.

The original stores colour per vertex and has no UVs or `.mtl`, so `build_meshes.py` adds planar
UVs that map onto the orthophoto.

### Regenerating

The raw ERC files aren't in the repo. Download them from the ERC drive, then:

```bash
python3 tools/build_meshes.py Model3D_mesh2.obj orthophoto.tif models/erc2026_terrain \
    --visual-cell 0.10 --collision-cell 0.20
python3 tools/gen_world.py Coordinates_MarsYard2026.txt .
```

Needs numpy, scipy, Pillow and OpenCV (works with Ubuntu 24.04's `python3-opencv` 4.6).

## Known limits

- The terrain ends at the edge of the scan. There's no ground around it.
- Cameras render the point markers, so use `markers:=false` for SLAM/RTAB-Map runs.
- Not yet tested in Gazebo on our machines. Everything was generated and checked offline.
