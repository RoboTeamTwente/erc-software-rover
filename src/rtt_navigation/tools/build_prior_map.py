#!/usr/bin/env python3
"""Mars Yard mesh -> elevation map -> traversability cost map in ERC coordinates (Nav2 map_server format).

Offline tool: run on a laptop, not on the rover (needs numpy, scipy, pillow, matplotlib). Output:
  <out>/mars_yard_prior.pgm + .yaml   cost map for nav2_map_server (mode: raw, 0 = free ... 100 = lethal)
  <out>/mars_yard_elevation.npy       elevation grid [m] in ERC heights (NaN = no data)
  <out>/mars_yard_prior.png           preview with the ERC points and an example A* route

Usage:
  python3 build_prior_map.py <terrain.obj> <texture.jpg> <out_dir>
      [--slope-lethal 20] [--slope-free 5] [--rough-lethal 0.12] [--radius 0.36]
      [--landmark-radius 0.18] [--start S1] [--goals W2,W5,W6,W8]

Input: rtt_gazebo_sim/models/erc2026_terrain (the ERC 2026 scan in its own frame, which is already
the ERC yard frame horizontally: checked against the survey markers at S1-S9, W1, W2 in the ERC
orthophoto to ~1.5 cm). Only the levelling to the report's heights is applied here (PLANE); it is
the same levelling as the <pose> in erc2026_terrain/model.sdf (fitted on S1-S9, W1-W9, P1).
"""
import argparse
import heapq
import itertools

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy import ndimage

# ---- ERC frame: p_mesh = R(theta) * p_erc + t ;  z_mesh - H_erc = c0 + c1*X + c2*Y --------------
# erc2026_terrain is already in ERC X/Y: no rotation, no shift.
THETA_DEG, TX, TY = 0.0, 0.0, 0.0
# Levelling, small-angle form of erc2026_terrain/model.sdf <pose>0 0 -0.0524 0.02354 -0.01056 0</pose>:
# c0 = -z, c1 = pitch, c2 = -roll.
PLANE = (0.0524, -0.01056, -0.02354)

ERC_POINTS = {  # name: (X, Y, H)  -- 2026 MY Update Report Rev 1.1, "Mars Yard (All Points)"
    'S1': (0.000, 0.000, 0.000), 'S2': (0.000, 26.427, 0.138), 'S3': (10.709, 6.918, 0.067),
    'S4': (11.210, 17.881, 0.100), 'S5': (13.223, 25.658, -0.035), 'S6': (-9.428, 15.824, -0.302),
    'S7': (-8.970, 22.831, -0.180), 'S8': (-15.676, 4.991, -0.280), 'S9': (7.160, -4.762, -0.095),
    'L1': (3.183, 8.012, 0.361), 'L2': (7.269, 9.482, 0.281), 'L3': (7.878, 17.583, 0.398),
    'L4': (9.225, 22.389, 0.437), 'L5': (3.518, 23.990, 0.272), 'L6': (0.882, 16.870, 0.259),
    'L7': (-3.944, 21.415, 0.104), 'L8': (-5.491, 16.334, 0.061), 'L9': (-7.695, 13.528, -0.051),
    'L10': (-1.610, 12.602, 0.290), 'L11': (-7.715, 9.721, 0.006), 'L12': (-4.311, 4.442, -0.111),
    'L13': (-5.720, 28.118, -0.163), 'L14': (-11.438, 5.230, -0.078), 'L15': (6.483, 1.102, 0.239),
    'W1': (1.951, 9.759, -0.053), 'W2': (4.170, 19.499, 0.105), 'W3': (7.013, 23.149, 0.205),
    'W4': (-1.701, 18.665, -0.156), 'W5': (-5.339, 19.034, -0.259), 'W6': (-7.675, 8.197, -0.179),
    'W7': (-2.108, 7.682, -0.163), 'W8': (5.939, 6.790, -0.065), 'W9': (5.088, 13.092, 0.011),
}


def load_obj(path):
    verts, uvs, tris, tri_uv = [], [], [], []
    with open(path) as f:
        for line in f:
            if line.startswith('v '):
                verts.append([float(v) for v in line.split()[1:4]])
            elif line.startswith('vt '):
                uvs.append([float(v) for v in line.split()[1:3]])
            elif line.startswith('f '):
                parts = [p.split('/') for p in line.split()[1:]]
                idx = [int(p[0]) - 1 for p in parts]
                uv = [int(p[1]) - 1 if len(p) > 1 and p[1] else 0 for p in parts]
                for k in range(1, len(idx) - 1):
                    tris.append([idx[0], idx[k], idx[k + 1]])
                    tri_uv.append([uv[0], uv[k], uv[k + 1]])
    return np.array(verts), np.array(uvs), np.array(tris), np.array(tri_uv)


def mesh_to_erc(V):
    th = np.radians(THETA_DEG)
    c, s = np.cos(th), np.sin(th)
    dx, dy = V[:, 0] - TX, V[:, 1] - TY
    X, Y = c * dx + s * dy, -s * dx + c * dy                       # inverse rotation
    H = V[:, 2] - (PLANE[0] + PLANE[1] * X + PLANE[2] * Y)        # remove the model's tilt + offset
    return np.c_[X, Y, H]


def rasterise(V, UV, T, TUV, tex, res):
    """Highest surface point per cell (+ its texture colour)."""
    lo = np.floor(V[:, :2].min(0) / res) * res
    hi = np.ceil(V[:, :2].max(0) / res) * res
    nx, ny = int(round((hi[0] - lo[0]) / res)), int(round((hi[1] - lo[1]) / res))
    elev = np.full((ny, nx), -np.inf)
    color = np.zeros((ny, nx, 3), np.uint8)
    th, tw = tex.shape[:2]
    rng = np.random.default_rng(0)
    A, B, C = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
    ab, ac = B[:, :2] - A[:, :2], C[:, :2] - A[:, :2]
    area = 0.5 * np.abs(ab[:, 0] * ac[:, 1] - ab[:, 1] * ac[:, 0])   # top-down area of each triangle
    n_samp = np.maximum(3, np.ceil(area / (res * res) * 8).astype(int))
    for n in np.unique(n_samp):
        sel = np.where(n_samp == n)[0]
        r1, r2 = rng.random((len(sel), n, 1)), rng.random((len(sel), n, 1))
        s = np.sqrt(r1)
        w0, w1, w2 = 1 - s, s * (1 - r2), s * r2
        P = (w0 * A[sel, None] + w1 * B[sel, None] + w2 * C[sel, None]).reshape(-1, 3)
        uv = (w0 * UV[TUV[sel, 0], None] + w1 * UV[TUV[sel, 1], None] + w2 * UV[TUV[sel, 2], None]).reshape(-1, 2)
        ix = np.clip(((P[:, 0] - lo[0]) / res).astype(int), 0, nx - 1)
        iy = np.clip(((P[:, 1] - lo[1]) / res).astype(int), 0, ny - 1)
        order = np.argsort(P[:, 2])
        ix, iy, z, uv = ix[order], iy[order], P[order, 2], uv[order]
        better = z > elev[iy, ix]
        elev[iy[better], ix[better]] = z[better]
        px = np.clip((uv[better, 0] * tw).astype(int), 0, tw - 1)
        py = np.clip(((1 - uv[better, 1]) * th).astype(int), 0, th - 1)
        color[iy[better], ix[better]] = tex[py, px]
    valid = np.isfinite(elev)
    # close pin-holes left by sampling (cells surrounded by data)
    holes = ndimage.binary_fill_holes(valid) & ~valid
    if holes.any():
        idx = ndimage.distance_transform_edt(~valid, return_distances=False, return_indices=True)
        elev[holes] = elev[idx[0][holes], idx[1][holes]]
        color[holes] = color[idx[0][holes], idx[1][holes]]
        valid = valid | holes
    elev[~valid] = np.nan
    return elev, color, valid, lo


def terrain_cost(elev, valid, res, slope_free, slope_lethal, rough_lethal):
    filled = np.where(valid, elev, np.nanmean(elev))
    smooth = ndimage.uniform_filter(filled, size=int(round(0.5 / res)) | 1)
    gy, gx = np.gradient(smooth, res)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    rough = np.abs(filled - smooth)
    c_slope = np.clip((slope - slope_free) / (slope_lethal - slope_free), 0, 1)
    c_rough = np.clip(rough / rough_lethal, 0, 1)
    cost = np.round(99 * np.maximum(c_slope, c_rough)).astype(np.int16)
    cost[(slope >= slope_lethal) | (rough >= rough_lethal)] = 100
    cost[~ndimage.binary_erosion(valid, iterations=2)] = 100       # outside the model: keep out
    return cost, slope, rough


def astar(cost, blocked, start, goal, res, weight):
    """8-connected A*. Step cost = distance * (1 + weight * cell cost / 100)."""
    ny, nx = cost.shape
    if blocked[start] or blocked[goal]:
        return None, np.inf
    moves = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1), (-1, -1, 2 ** 0.5), (-1, 1, 2 ** 0.5), (1, -1, 2 ** 0.5), (1, 1, 2 ** 0.5)]
    g = {start: 0.0}
    parent = {}
    heap = [(0.0, start)]
    done = set()
    while heap:
        _, cur = heapq.heappop(heap)
        if cur in done:
            continue
        done.add(cur)
        if cur == goal:
            path = [cur]
            while path[-1] in parent:
                path.append(parent[path[-1]])
            return path[::-1], g[goal]
        for dy, dx, d in moves:
            nb = (cur[0] + dy, cur[1] + dx)
            if not (0 <= nb[0] < ny and 0 <= nb[1] < nx) or blocked[nb]:
                continue
            ng = g[cur] + d * res * (1 + weight * cost[nb] / 100.0)
            if ng < g.get(nb, np.inf):
                g[nb] = ng
                parent[nb] = cur
                h = res * np.hypot(nb[0] - goal[0], nb[1] - goal[1])      # admissible: cost factor >= 1
                heapq.heappush(heap, (ng + h, nb))
    return None, np.inf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('obj'); ap.add_argument('texture'); ap.add_argument('out')
    ap.add_argument('--res', type=float, default=0.10)
    ap.add_argument('--slope-free', type=float, default=5.0, help='slopes below this cost nothing [deg]')
    ap.add_argument('--slope-lethal', type=float, default=20.0, help='steepest slope the rover may drive [deg]')
    ap.add_argument('--rough-lethal', type=float, default=0.12, help='bump/dip height that blocks the rover [m]')
    ap.add_argument('--radius', type=float, default=0.36, help='rover inscribed radius for the demo route [m]')
    ap.add_argument('--landmark-radius', type=float, default=0.18,
                    help='landmark posts (250 mm box) are marked lethal within this radius [m]; 0 = off')
    ap.add_argument('--weight', type=float, default=3.0, help='how strongly the demo route avoids costly cells')
    ap.add_argument('--start', default='S1')
    ap.add_argument('--goals', default='W2,W5,W6,W8')
    a = ap.parse_args()

    V, UV, T, TUV = load_obj(a.obj)
    E = mesh_to_erc(V)
    tex = np.asarray(Image.open(a.texture).convert('RGB'))
    elev, color, valid, lo = rasterise(E, UV, T, TUV, tex, a.res)
    cost, slope, rough = terrain_cost(elev, valid, a.res, a.slope_free, a.slope_lethal, a.rough_lethal)
    ny, nx = cost.shape
    # landmark posts are not in the scan (placed after it was taken): mark them as obstacles
    if a.landmark_radius > 0:
        gy, gx = np.mgrid[0:ny, 0:nx]
        cx, cy = lo[0] + (gx + 0.5) * a.res, lo[1] + (gy + 0.5) * a.res
        for k, (x, y, _) in ERC_POINTS.items():
            if k[0] == 'L':
                cost[np.hypot(cx - x, cy - y) <= a.landmark_radius] = 100
    print(f'grid {nx} x {ny} cells at {a.res} m; ERC X [{lo[0]:.1f}, {lo[0] + nx * a.res:.1f}], Y [{lo[1]:.1f}, {lo[1] + ny * a.res:.1f}]')
    inner = ndimage.binary_erosion(valid, iterations=5)
    print(f'inside the yard: {100 * (cost[inner] == 100).mean():.0f}% lethal, {100 * (cost[inner] == 0).mean():.0f}% free, '
          f'{100 * ((cost[inner] > 0) & (cost[inner] < 100)).mean():.0f}% graded')

    # ---- Nav2 map_server files (mode raw: pixel value = occupancy 0..100; first image row = max Y) ----
    Image.fromarray(np.flipud(cost).astype(np.uint8)).save(f'{a.out}/mars_yard_prior.pgm')
    with open(f'{a.out}/mars_yard_prior.yaml', 'w') as f:
        f.write('# Traversability prior of the ERC Mars Yard, in ERC coordinates (origin S1, +Y towards S2).\n'
                f'# Built by build_prior_map.py: slope free <{a.slope_free} deg, lethal >={a.slope_lethal} deg, roughness lethal >={a.rough_lethal} m.\n'
                'image: mars_yard_prior.pgm\nmode: raw\n'
                f'resolution: {a.res}\norigin: [{lo[0]:.3f}, {lo[1]:.3f}, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    np.save(f'{a.out}/mars_yard_elevation.npy', elev)

    def cell(name):
        x, y, _ = ERC_POINTS[name]
        return (int((y - lo[1]) / a.res), int((x - lo[0]) / a.res))

    # height check against the survey
    chk = [(k, elev[cell(k)] - v[2]) for k, v in ERC_POINTS.items() if k[0] in 'SW']
    print(f'elevation vs surveyed H at S/W points: RMS {np.sqrt(np.mean([d * d for _, d in chk])) * 100:.1f} cm, worst {max(abs(d) for _, d in chk) * 100:.1f} cm')

    # ---- demo: reachability + best visiting order with A* ----
    blocked = ndimage.binary_dilation(cost >= 100, structure=ndimage.generate_binary_structure(2, 1),
                                      iterations=int(np.ceil(a.radius / a.res)))
    free_lbl, _ = ndimage.label(~blocked, structure=np.ones((3, 3)))
    comp = {k: free_lbl[cell(k)] for k in ERC_POINTS if k[0] in 'SW'}
    main_comp = comp[a.start]
    print(f'reachable from {a.start} (rover radius {a.radius} m): ' + ' '.join(k for k, c in comp.items() if c == main_comp and c != 0))
    unreachable = [k for k, c in comp.items() if c != main_comp or c == 0]
    print('NOT reachable: ' + (' '.join(f'{k}{"(blocked cell)" if comp[k] == 0 else ""}' for k in unreachable) or 'none'))

    goals = a.goals.split(',')
    nodes = [a.start] + goals
    legs = {}
    for p, q in itertools.combinations(nodes, 2):
        path, c = astar(cost, blocked, cell(p), cell(q), a.res, a.weight)
        legs[(p, q)] = legs[(q, p)] = (path, c)
    best = min(itertools.permutations(goals), key=lambda o: sum(legs[(u, v)][1] for u, v in zip((a.start,) + o, o + (a.start,))))
    order = (a.start,) + best + (a.start,)
    total_len = 0.0
    route = []
    for u, v in zip(order, order[1:]):
        path = legs[(u, v)][0]
        path = path if path[0] == cell(u) else path[::-1]
        pts = np.array([(lo[0] + (c + 0.5) * a.res, lo[1] + (r + 0.5) * a.res) for r, c in path])
        total_len += np.linalg.norm(np.diff(pts, axis=0), axis=1).sum()
        route.append(pts)
    straight = sum(np.hypot(ERC_POINTS[u][0] - ERC_POINTS[v][0], ERC_POINTS[u][1] - ERC_POINTS[v][1]) for u, v in zip(order, order[1:]))
    worst = max(itertools.permutations(goals), key=lambda o: sum(legs[(u, v)][1] for u, v in zip((a.start,) + o, o + (a.start,))))
    cost_of = lambda o: sum(legs[(u, v)][1] for u, v in zip((a.start,) + o, o + (a.start,)))
    print(f'example mission {a.start} -> {{{", ".join(goals)}}} -> {a.start}: best order {" -> ".join(order)}')
    print(f'  path length {total_len:.1f} m (straight lines {straight:.1f} m); cost {cost_of(best):.1f} vs worst order {cost_of(worst):.1f}')

    # ---- preview ----
    ext = [lo[0], lo[0] + nx * a.res, lo[1], lo[1] + ny * a.res]
    fig, ax = plt.subplots(1, 3, figsize=(21, 8.2))
    im = ax[0].imshow(elev, origin='lower', extent=ext, cmap='terrain'); ax[0].set_title('elevation map [m, ERC heights]')
    plt.colorbar(im, ax=ax[0], shrink=0.75)
    shown = np.where(valid, cost, np.nan).astype(float)
    im = ax[1].imshow(shown, origin='lower', extent=ext, cmap='RdYlGn_r', vmin=0, vmax=100)
    ax[1].set_title(f'cost map (0 free ... 100 lethal: slope >= {a.slope_lethal:.0f} deg or roughness >= {a.rough_lethal * 100:.0f} cm)')
    plt.colorbar(im, ax=ax[1], shrink=0.75)
    ax[2].imshow(color, origin='lower', extent=ext)
    ax[2].imshow(np.where(blocked & valid, 1.0, np.nan), origin='lower', extent=ext, cmap='Reds', vmin=0, vmax=1.3, alpha=0.45)
    for pts in route:
        ax[2].plot(pts[:, 0], pts[:, 1], '-', color='cyan', lw=2.5)
    ax[2].set_title(f'example A* route: {" > ".join(order)}  ({total_len:.0f} m)\nred = not drivable for a {a.radius} m radius rover')
    for axx in ax:
        for k, (x, y, _) in ERC_POINTS.items():
            col = {'S': 'blue', 'L': 'black', 'W': 'magenta'}[k[0]]
            axx.plot(x, y, 'x' if k[0] == 'L' else 'o', color=col, ms=5, mew=1.5)
            axx.annotate(k, (x, y), color=col, fontsize=8, fontweight='bold', xytext=(3, 3), textcoords='offset points')
        axx.set_xlabel('ERC X [m]'); axx.set_ylabel('ERC Y [m]'); axx.set_aspect('equal')
    plt.tight_layout(); plt.savefig(f'{a.out}/mars_yard_prior.png', dpi=80)


if __name__ == '__main__':
    main()
