#!/usr/bin/env python3
"""Generate the Gazebo side of the ERC 2026 Mars Yard in rtt_gazebo_sim (all but the meshes):

  models/erc2026_terrain/model.sdf     terrain model incl. levelling pose
  models/erc2026_landmark_NN/          15 ArUco landmark posts (DICT_5X5, IDs 51-65)
  models/erc2026_points/               visual-only markers for S / W / P points
  worlds/mars_yard.sdf                 terrain + landmarks (default world of sim_launch.py)
  worlds/mars_yard_markers.sdf         terrain + landmarks + point markers

and print the RTAB-Map Marker/Priors line for rtt_slam/config/rtabmap.yaml.

Points come from rtt_slam/maps/erc_2026_points.yaml (the repo's single source).
Run build_meshes.py first (terrain heights are read from erc2026_visual.obj).

usage (from the repo root):
  python3 src/rtt_gazebo_sim/tools/gen_world.py src/rtt_slam/maps/erc_2026_points.yaml src/rtt_gazebo_sim
Needs numpy, scipy, pyyaml, Pillow, OpenCV (cv2 with aruco; Ubuntu's python3-opencv works).
"""
import os
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial import cKDTree

# ---------------------------------------------------------------------------
# Levelling of the scan mesh to ERC's 'H (local)' heights. Fitted on the 19
# ground points (S, W, P) only; see README.md. Applied to the terrain link.
LEVEL = dict(z=-0.0524, roll=0.02354, pitch=-0.01056)

# Point prefixes in Coordinates_MarsYard2026.txt
TYPES = {
    "S": ("start", "0.1 0.8 0.1 1"),          # green
    "W": ("waypoint", "0.1 0.3 1.0 1"),       # blue
    "L": ("landmark", None),                  # modelled as real ArUco posts
    "P": ("deep_sampling", "0.9 0.1 0.9 1"),  # magenta
}

# Landmark geometry (ERC rulebook fig. 9, metres)
ARUCO_ID_OFFSET = 50          # landmark N -> DICT_5X5 id 50+N (checked against rulebook fig. 10)
BOX = 0.250                   # box width / depth
BOX_H = 0.320                 # box height (drawing; text says 310 mm)
TAG_CENTRE_Z = 0.417          # ground -> tag centre
TAG_ABOVE_BOX_BOTTOM = 0.1155  # measured on the drawing
BOX_BOTTOM_Z = TAG_CENTRE_Z - TAG_ABOVE_BOX_BOTTOM  # ~0.30 m
POLE_R = 0.024                # pole radius (not dimensioned; ~48 mm on the drawing)
SINK = 0.02                   # push posts this far into the ground so they never float
PX_PER_M = 2000               # face texture resolution (2 px/mm)


def read_points(path):
    """(name, x, y, h) from rtt_slam/maps/erc_2026_points.yaml, any group (S, W, L, P)."""
    import yaml
    data = yaml.safe_load(open(path))
    pts = []
    for group in data.values():
        for name, p in group.items():
            pts.append((name, float(p["x"]), float(p["y"]), float(p["h"])))
    return pts


def level_matrix():
    cr, sr = np.cos(LEVEL["roll"]), np.sin(LEVEL["roll"])
    cp, sp = np.cos(LEVEL["pitch"]), np.sin(LEVEL["pitch"])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return Ry @ Rx  # SDF rpy convention with yaw = 0


def terrain_heights(obj_path, pts):
    """Height of the levelled terrain (as simulated) at each point."""
    V = []
    with open(obj_path) as fh:
        for line in fh:
            if line.startswith("v "):
                V.append(line[2:])
    V = np.array(" ".join(V).split(), float).reshape(-1, 3)
    V = V @ level_matrix().T
    V[:, 2] += LEVEL["z"]
    tree = cKDTree(V[:, :2])
    out = []
    for _, x, y, _ in pts:
        idx = tree.query_ball_point([x, y], 0.12) or [tree.query([x, y])[1]]
        out.append(float(np.median(V[idx, 2])))
    return out


# ---------------------------------------------------------------------------
def aruco_image(marker_id, px):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    if hasattr(cv2.aruco, "generateImageMarker"):  # OpenCV >= 4.7
        return cv2.aruco.generateImageMarker(d, marker_id, px)
    return cv2.aruco.drawMarker(d, marker_id, px)  # OpenCV 4.6 (Ubuntu 24.04 apt)


def font(size):
    for f in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
              "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"):
        if os.path.exists(f):
            return ImageFont.truetype(f, size)
    return ImageFont.load_default()


def face_texture(n):
    """One landmark face, 250 x 320 mm, as seen from outside (top row = top)."""
    s = PX_PER_M / 1000.0  # px per mm
    W, H = round(250 * s), round(320 * s)
    img = Image.new("RGB", (W, H), (205, 205, 200))  # box colour
    d = ImageDraw.Draw(img)

    def rect(x0, y0_from_bottom, w, h, **kw):  # mm, measured from face bottom-left
        d.rectangle([round(x0 * s), round(H - (y0_from_bottom + h) * s),
                     round((x0 + w) * s) - 1, round(H - y0_from_bottom * s) - 1], **kw)

    a4_x, a4_y = (250 - 210) / 2, (320 - 297) / 2          # A4 sheet centred
    rect(a4_x, a4_y, 210, 297, fill=(255, 255, 255))
    tag_x, tag_y = (250 - 150) / 2, a4_y + 30                # 150 mm tag, 30 mm above A4 bottom
    tag = Image.fromarray(aruco_image(ARUCO_ID_OFFSET + n, round(150 * s))).convert("RGB")
    img.paste(tag, (round(tag_x * s), round(H - (tag_y + 150) * s)))
    num_y = tag_y + 150 + (a4_y + 297 - (tag_y + 150) - 70) / 2  # 150 x 70 mm number box
    rect(tag_x, num_y, 150, 70, fill=(240, 240, 240), outline=(60, 60, 60), width=round(1.5 * s))
    f = font(round(55 * s))
    cx, cy = (tag_x + 75) * s, H - (num_y + 35) * s
    d.text((cx, cy), str(n), fill=(0, 0, 0), font=f, anchor="mm")
    return img


def landmark_obj(path_obj, path_mtl, tex_name):
    """Textured box (4 identical faces) from BOX_BOTTOM_Z to BOX_BOTTOM_Z + BOX_H."""
    hw, z0, z1 = BOX / 2, BOX_BOTTOM_Z, BOX_BOTTOM_Z + BOX_H
    up = np.array([0.0, 0.0, 1.0])
    v, vt, vn, faces_side, faces_cap = [], [], [], [], []
    for n in ([1, 0, 0], [0, 1, 0], [-1, 0, 0], [0, -1, 0]):
        n = np.array(n, float)
        right = np.cross(-n, up)  # viewer outside, looking along -n
        c = n * hw
        corners = [c - right * hw + up * z0, c + right * hw + up * z0,
                   c + right * hw + up * z1, c - right * hw + up * z1]  # BL BR TR TL
        base = len(v)
        v += corners
        vt += [(0, 0), (1, 0), (1, 1), (0, 1)]
        vn.append(n)
        k = len(vn)
        faces_side += [((base, base + 1, base + 2), k), ((base, base + 2, base + 3), k)]
    for z, nz in ((z1, 1.0), (z0, -1.0)):  # top / bottom caps
        base = len(v)
        sq = [(-hw, -hw), (hw, -hw), (hw, hw), (-hw, hw)]
        v += [np.array([x, y, z]) for x, y in (sq if nz > 0 else sq[::-1])]
        vn.append(np.array([0, 0, nz]))
        k = len(vn)
        faces_cap += [((base, base + 1, base + 2), k), ((base, base + 2, base + 3), k)]
    with open(path_obj, "w") as fh:
        fh.write(f"# ERC landmark box, generated by gen_world.py\nmtllib {os.path.basename(path_mtl)}\n")
        for p in v:
            fh.write("v %.4f %.4f %.4f\n" % tuple(p))
        for t in vt:
            fh.write("vt %.4f %.4f\n" % t)
        for nn in vn:
            fh.write("vn %.1f %.1f %.1f\n" % tuple(nn))
        fh.write("usemtl face\n")
        for (a, b, c), k in faces_side:
            fh.write(f"f {a+1}/{a+1}/{k} {b+1}/{b+1}/{k} {c+1}/{c+1}/{k}\n")
        fh.write("usemtl box\n")
        for (a, b, c), k in faces_cap:
            fh.write(f"f {a+1}//{k} {b+1}//{k} {c+1}//{k}\n")
    with open(path_mtl, "w") as fh:
        fh.write("newmtl face\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nd 1\nillum 1\n"
                 f"map_Kd textures/{tex_name}\n\n"
                 "newmtl box\nKa 0.8 0.8 0.78\nKd 0.8 0.8 0.78\nKs 0 0 0\nd 1\nillum 1\n")


def landmark_model(pkg, n):
    name = f"erc2026_landmark_{n:02d}"
    mdir = f"{pkg}/models/{name}"
    os.makedirs(f"{mdir}/meshes/textures", exist_ok=True)
    tex = f"landmark_{n:02d}.png"
    face_texture(n).save(f"{mdir}/meshes/textures/{tex}")
    landmark_obj(f"{mdir}/meshes/landmark.obj", f"{mdir}/meshes/landmark.mtl", tex)
    pole_len = BOX_BOTTOM_Z + SINK
    box_cz = BOX_BOTTOM_Z + BOX_H / 2
    open(f"{mdir}/model.config", "w").write(f"""<?xml version="1.0"?>
<model>
  <name>ERC 2026 landmark {n}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>ERC landmark post {n}: 250x250x320 mm box on a pole, ArUco DICT_5X5 id {ARUCO_ID_OFFSET + n}
    (150 mm) on all four faces, tag centre {TAG_CENTRE_Z} m above the model origin (= ground).</description>
</model>
""")
    open(f"{mdir}/model.sdf", "w").write(f"""<?xml version="1.0"?>
<!-- Generated by tools/gen_world.py. Origin = ground point under the pole axis.
     ArUco DICT_5X5 id {ARUCO_ID_OFFSET + n}, 150 mm, tag centre at z = {TAG_CENTRE_Z} m. -->
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="pole">
        <pose>0 0 {pole_len / 2 - SINK:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>{POLE_R}</radius><length>{pole_len:.4f}</length></cylinder></geometry>
      </collision>
      <visual name="pole">
        <pose>0 0 {pole_len / 2 - SINK:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>{POLE_R}</radius><length>{pole_len:.4f}</length></cylinder></geometry>
        <material><ambient>0.55 0.4 0.25 1</ambient><diffuse>0.55 0.4 0.25 1</diffuse></material>
      </visual>
      <collision name="box">
        <pose>0 0 {box_cz:.4f} 0 0 0</pose>
        <geometry><box><size>{BOX} {BOX} {BOX_H}</size></box></geometry>
      </collision>
      <visual name="box">
        <geometry><mesh><uri>model://{name}/meshes/landmark.obj</uri></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>
""")
    return name


# ---------------------------------------------------------------------------
def terrain_sdf():
    return f"""<?xml version="1.0"?>
<!-- Generated by tools/gen_world.py. ERC 2026 Mars Yard terrain, built from the ERC
     3D mesh (Model3D_mesh2.obj) and orthophoto with tools/build_meshes.py.

     The mesh is already Z-up and in the ERC local frame (origin at S1).
     The link pose only LEVELS the mesh to the heights in ERC's coordinate file
     ('H (local)'): the mesh is tilted ~1.35 deg about X and ~-0.60 deg about Y
     relative to them. Fitted on the 19 ground points (S, W, P): 3 cm RMS.
     The same levelling is used by rtt_slam/tools/build_prior_map.py (PLANE); change both together. -->
<sdf version="1.9">
  <model name="erc2026_terrain">
    <static>true</static>
    <link name="terrain">
      <pose>0 0 {LEVEL['z']} {LEVEL['roll']} {LEVEL['pitch']} 0</pose>

      <!-- 20 cm vertex clustering, ~76k triangles: cheap for physics -->
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://erc2026_terrain/meshes/erc2026_collision.obj</uri>
          </mesh>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.8</mu>
              <mu2>0.8</mu2>
            </ode>
          </friction>
        </surface>
      </collision>

      <!-- 10 cm vertex clustering, ~289k triangles, 1 cm/px orthophoto texture -->
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://erc2026_terrain/meshes/erc2026_visual.obj</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""


def markers_sdf(pts, gz):
    links = []
    for (n, x, y, _), g in zip(pts, gz):
        rgba = TYPES[n[0]][1]
        if rgba is None:
            continue
        links.append(f"""    <link name="{n}">
      <pose>{x:.3f} {y:.3f} {g:.3f} 0 0 0</pose>
      <visual name="pole">
        <pose>0 0 0.30 0 0 0</pose>
        <geometry><cylinder><radius>0.03</radius><length>0.60</length></cylinder></geometry>
        <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>
        <cast_shadows>false</cast_shadows>
      </visual>
      <visual name="ball">
        <pose>0 0 0.65 0 0 0</pose>
        <geometry><sphere><radius>0.08</radius></sphere></geometry>
        <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>
        <cast_shadows>false</cast_shadows>
      </visual>
    </link>""")
    return f"""<?xml version="1.0"?>
<!-- Generated by tools/gen_world.py. Visual-only markers (no collision) for the official
     ERC 2026 start (green), waypoint (blue) and deep sampling (magenta) points.
     Landmarks are real ArUco post models instead. Cameras see these markers,
     so use worlds/mars_yard.sdf (the default) for SLAM tests. -->
<sdf version="1.9">
  <model name="erc2026_points">
    <static>true</static>
{chr(10).join(links)}
  </model>
</sdf>
"""


def world_sdf(landmarks, with_markers):
    inc = "".join(f"""
    <include>
      <uri>model://{name}</uri>
      <pose>{x:.3f} {y:.3f} {g:.3f} 0 0 0</pose>
    </include>""" for name, x, y, g in landmarks)
    markers = """
    <include>
      <uri>model://erc2026_points</uri>
    </include>""" if with_markers else ""
    return f"""<?xml version="1.0"?>
<!-- Generated by tools/gen_world.py. ERC 2026 Mars Yard{' with point markers' if with_markers else ''}.
     World frame = ERC yard frame (origin S1, +Y towards S2, Z up). -->
<sdf version="1.9">
  <world name="mars_yard">
    <physics name="default_physics" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.55 0.7 0.9 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <!-- ERC 2026 terrain. The model is already in the ERC yard frame (origin S1, +Y towards S2)
         and levelled to the report's heights inside its model.sdf, so no pose here. -->
    <include>
      <uri>model://erc2026_terrain</uri>
    </include>

    <!-- ERC landmarks (ArUco posts). Yaw is unknown, so faces are aligned with X/Y. -->{inc}
{markers}
  </world>
</sdf>
"""


def marker_priors(pts):
    """RTAB-Map Marker/Priors: tag centre = ERC ground height h + 0.417 m, on the pole axis.
    Angles are 0: with Marker/VarianceOrientationIgnored only the position is used."""
    return "|".join(f"{ARUCO_ID_OFFSET + int(n[1:])} {x:.3f} {y:.3f} {h + TAG_CENTRE_Z:.3f} 0 0 0"
                    for n, x, y, h in sorted(pts, key=lambda p: int(p[0][1:]) if p[0][0] == "L" else 0)
                    if n[0] == "L")


def main():
    pts_yaml, pkg = sys.argv[1], sys.argv[2]
    pts = read_points(pts_yaml)
    gz = terrain_heights(f"{pkg}/models/erc2026_terrain/meshes/erc2026_visual.obj", pts)
    for sub in ("worlds", "models/erc2026_points"):
        os.makedirs(f"{pkg}/{sub}", exist_ok=True)

    open(f"{pkg}/models/erc2026_terrain/model.sdf", "w").write(terrain_sdf())
    open(f"{pkg}/models/erc2026_points/model.sdf", "w").write(markers_sdf(pts, gz))
    open(f"{pkg}/models/erc2026_points/model.config", "w").write("""<?xml version="1.0"?>
<model>
  <name>ERC 2026 points</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Visual markers for the official ERC 2026 start, waypoint and deep sampling points.</description>
</model>
""")
    landmarks = []
    for (n, x, y, _), g in zip(pts, gz):
        if n.startswith("L"):
            landmarks.append((landmark_model(pkg, int(n[1:])), x, y, g))  # origin on the ground
    open(f"{pkg}/worlds/mars_yard.sdf", "w").write(world_sdf(landmarks, False))
    open(f"{pkg}/worlds/mars_yard_markers.sdf", "w").write(world_sdf(landmarks, True))
    print(f"{len(pts)} points, {len(landmarks)} landmark models written")
    print("\nMarker/Priors for rtt_slam/config/rtabmap.yaml:")
    print(f'Marker/Priors: "{marker_priors(pts)}"')


if __name__ == "__main__":
    main()
