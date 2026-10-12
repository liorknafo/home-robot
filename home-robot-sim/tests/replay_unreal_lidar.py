"""Offline, evaluation-only replay of an Unreal full-mapping trial's LiDAR into the robot's grid (task K, LIO-118). Never
imported or read by the controller.
Usage: replay_unreal_lidar.py REPORT_DIR [--old-rule] [--steps N] [--register estimate|truth] [--out MAP_DIR]

Unreal's LD14 is one line trace per beam, in the body's own (tilting) scan plane, against the house's visible collision
triangles (RobotPawn::SampleLidar; HouseScene builds every collision mesh complex-as-simple). The trial trace logs the
robot's true pose, pitch, roll and height every ~0.75 s (evaluation only). Through the trace rows in mapping this re-casts
the 667 beams of each scan against the same triangles (Content/RobotData/house-meshes.json placed as HouseScene places them),
applies the D200 range model (D200Model.h: bias +0.5 and jitter 0.25 of the accuracy bound, 2 % dropout, dark targets
unrated beyond 6 m) and integrates the scan into a LidarGrid at the robot's OWN pose estimate (status.pose_m, interpolated to
the scan's instant), as LidarBrain.observe_scan does, or at the true pose (--register truth). The same scans can be fed to
another grid-update rule and the maps compared (--old-rule: the update before task K).
Approximations: `steps` scans per trace row, poses interpolated between rows and the nearest row's pitch, roll and height
(Unreal: about 6 scans per second, gated by integrate_scan's 2.5 cm / 0.06 rad), no motion within a scan, no camera /
contact cells.
"""
from pathlib import Path
import argparse, json, math, sys
import numpy as np
R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R / 'controller'))
from autonomy.lidar import LidarGrid   # noqa: E402

BEAMS = 667


def robot(root=R):
    """(LiDAR mount x, y, z above the floor; body origin above base_link, RobotPawn BodyHeight) from root's robot parameters."""
    p = json.loads((Path(root) / 'Content/RobotData/robot-parameters.json').read_text())
    return [float(v) for v in p['lidar_mount_m']], float(p.get('base_link_height_m', .085))


LIDAR, BASE_H_M = robot()


def house_triangles(root=R):
    """Every collision triangle of the house in the controller's world frame (m; x forward, y LEFT, z up), and whether its
    object is white paint (the D200 bright class)."""
    scene = json.loads((Path(root) / 'Content/RobotData/house-scene.json').read_text())
    assets = json.loads((Path(root) / 'Content/RobotData' / scene['mesh_file']).read_text())['assets']
    tris = []; bright = []
    for o in scene['objects']:
        if not o.get('collision'):
            continue
        cx, cy, cz = o['center']; a = math.radians(o.get('yaw', 0))
        for part in assets[o['mesh']]['parts']:
            v = np.asarray(part['vertices'], float).reshape(-1, 3); t = np.asarray(part['triangles'], int).reshape(-1, 3)
            # HouseScene: SetActorLocationAndRotation(Center, FRotator(0, yaw, 0)): Unreal yaw turns +X toward +Y.
            x = v[:, 0] * math.cos(a) - v[:, 1] * math.sin(a) + cx; y = v[:, 0] * math.sin(a) + v[:, 1] * math.cos(a) + cy
            w = np.c_[x / 100, -y / 100, (v[:, 2] + cz) / 100]
            tris.append(w[t]); bright += [o.get('material') == 'white_paint'] * len(t)
    return np.concatenate(tris), np.array(bright, bool)


def body_axes(truth, mount=None):
    """Origin of the LiDAR and the body's x, y (left), z axes in the world, from the evaluation-only truth (Unreal pitch + =
    nose up, roll + = right side down; tests/run_physics.body_to_world)."""
    P, Q = math.radians(truth['pitch_deg']), math.radians(truth['roll_deg']); Y = -truth['yaw_rad']
    SP, CP, SR, CR, SY, CY = math.sin(P), math.cos(P), math.sin(Q), math.cos(Q), math.sin(Y), math.cos(Y)

    def rot(lx, ly, lz):   # robot-frame vector (y left) -> world vector (y left)
        wx = lx * CP * CY + ly * (SR * SP * CY - CR * SY) - lz * (CR * SP * CY + SR * SY)
        wy = lx * CP * SY + ly * (SR * SP * SY + CR * CY) + lz * (CY * SR - CR * SP * SY)
        wz = lx * SP - ly * SR * CP + lz * CR * CP
        return np.array([wx, -wy, wz])
    lidar, base_h = mount or (LIDAR, BASE_H_M); p = truth['position_m']
    origin = np.array([p[0], p[1], p[2] + base_h]) + rot(lidar[0], -lidar[1], lidar[2] - base_h)
    return origin, rot(1, 0, 0), -rot(0, 1, 0), rot(0, 0, 1)


def cast(tris, bright, truth, rng, max_range=8., mount=None):
    """Ranges (m, -1 = no reading) of one 667-beam scan from the true pose: the scan plane's cut through every triangle
    is a segment; each beam is a ray in that plane (exact for a planar scan)."""
    o, ex, ey, ez = body_axes(truth, mount)
    d = (tris - o) @ ez                                   # signed distances of the vertices to the scan plane
    d = np.where(np.abs(d) < 1e-9, 1e-9, d)
    cross = (d.min(1) < 0) & (d.max(1) > 0)
    T = tris[cross]; D = d[cross]; B = bright[cross]
    pts = []
    for i, j in ((0, 1), (1, 2), (2, 0)):
        s = D[:, i] / (D[:, i] - D[:, j]); ok = (D[:, i] < 0) != (D[:, j] < 0)
        pts.append(np.where(ok[:, None], T[:, i] + (T[:, j] - T[:, i]) * s[:, None], np.nan))
    pts = np.stack(pts, 1)                                # (n, 3 edges, 3): exactly two edges cross per triangle
    seg = np.stack([np.where(np.isnan(pts[:, 0, :1]), pts[:, 1], pts[:, 0]),
                    np.where(np.isnan(pts[:, 2, :1]), pts[:, 1], pts[:, 2])], 1)
    uv = np.stack([(seg - o) @ ex, (seg - o) @ ey], -1)   # in-plane coordinates about the LiDAR
    near = np.linalg.norm(uv, axis=-1).min(1) < max_range + 1
    uv = uv[near]; B = B[near]
    ang = -2 * math.pi * np.arange(BEAMS) / BEAMS; dirs = np.c_[np.cos(ang), np.sin(ang)]
    a = uv[:, 0]; e = uv[:, 1] - a
    den = dirs[:, None, 0] * e[None, :, 1] - dirs[:, None, 1] * e[None, :, 0]
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (a[None, :, 0] * e[None, :, 1] - a[None, :, 1] * e[None, :, 0]) / den
        u = (a[None, :, 0] * dirs[:, None, 1] - a[None, :, 1] * dirs[:, None, 0]) / den
    ok = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1) & (t <= max_range)
    t = np.where(ok, t, np.inf); k = t.argmin(1); r = t[np.arange(BEAMS), k]; dark = ~B[k]
    # D200Model::measure with RobotPawn's defaults (bias .5, jitter .25, dropout .02).
    bound = np.where(r <= .5, np.where(dark, .007, .005), np.where(r <= 1., np.where(dark, .012, .010),
                     np.where(dark, r * np.where(r <= 4., .012, .015), r * np.where(r <= 6., .010, .015))))
    lim = np.where(dark, 6., 8.)
    out = np.round((r + bound * (.5 + .25 * rng.standard_normal(BEAMS))) * 1000) / 1000
    bad = ~np.isfinite(r) | (r < .1) | (r > lim) | (rng.random(BEAMS) < .02) | (out < .1) | (out > lim)
    return np.where(bad, -1., out)


def scan_to_points(ranges, lidar=None):
    """scan_points() of a scan with no motion skew: base-frame points of the valid beams."""
    ang = -2 * math.pi * np.arange(BEAMS) / BEAMS; ok = ranges > 0
    return np.ascontiguousarray(np.c_[ranges[ok] * np.cos(ang[ok]), ranges[ok] * np.sin(ang[ok])] + np.array((lidar or LIDAR)[:2]), np.float32)


def snapshot(report):
    """The trial's frozen source (robot parameters and house), or this checkout when the report has none."""
    s = Path(report) / 'source-snapshot'
    return s if (s / 'Content/RobotData/robot-parameters.json').exists() else R


def rows_of(report):
    rows = [json.loads(l) for l in (Path(report) / 'trace.jsonl').read_text().splitlines()]
    return [r for r in rows if r['status'].get('mode') == 'mapping' and r.get('display_truth_evaluation_only', {}).get('available')
            and 'pitch_deg' in r['display_truth_evaluation_only']]


def trajectory(rows, steps):
    """(truth, estimated pose, sim time) at `steps` instants per trace row. Each row's status (the robot's pose estimate) and
    truth are sampled at different sim times (the truth 0-0.4 s later, up to 9 deg of heading apart in a spin), so both are
    interpolated to the same instants: between consecutive status times, unless a gap exceeds 2 s. Position and heading are
    interpolated; pitch, roll and height are the nearest truth row's (the trace samples them every ~0.75 s: draws of the
    robot's tilt, not a smooth signal), so each tilt sample is used about `steps` times and its spread is kept."""
    st = np.array([r['status']['sim_time_s'] for r in rows]); est = np.array([r['status']['pose_m'] for r in rows], float)
    tr = [r['display_truth_evaluation_only'] for r in rows]; tt = np.array([t['sim_time_s'] for t in tr])
    txy = np.array([t['position_m'][:2] for t in tr]); tyaw = np.unwrap([t['yaw_rad'] for t in tr]); eyaw = np.unwrap(est[:, 2])
    out = []
    for i in range(len(rows)):
        n = steps if i + 1 < len(rows) and st[i + 1] - st[i] <= 2. else 1
        for k in range(n):
            tau = st[i] + (st[i + 1] - st[i]) * k / n if n > 1 else st[i]
            j = int(np.argmin(np.abs(tt - tau)))
            truth = dict(tr[j], position_m=[float(np.interp(tau, tt, txy[:, 0])), float(np.interp(tau, tt, txy[:, 1])), tr[j]['position_m'][2]],
                         yaw_rad=float(np.interp(tau, tt, tyaw)))
            pose = np.array([np.interp(tau, st, est[:, 0]), np.interp(tau, st, est[:, 1]), math.remainder(float(np.interp(tau, st, eyaw)), 2 * math.pi)])
            out.append((i, truth, pose, float(tau)))
    return out


def to_map(origin, truth):
    """The true pose in the trial's map frame; origin: trial.json map_origin_ground_truth_evaluation_only (the world pose of
    the map origin)."""
    a = origin['yaw_rad']; d = np.asarray(truth['position_m'][:2]) - np.asarray(origin['position_m'][:2]); c, s = math.cos(a), math.sin(a)
    return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], math.remainder(truth['yaw_rad'] - a, 2 * math.pi)])


def replay(report, grid_factory=LidarGrid, steps=4, seed=0, on_scan=None, integrate=None, register='estimate'):
    """Integrate `steps` re-cast scans per mapping trace row (the trace logs ~1.3 rows per second; Unreal integrated ~3.9
    scans per second in trial full-mapping-20261012T013115) into grid_factory(), with the trial's own robot and house (its
    source snapshot), each cast from the truth and integrated at the robot's estimate interpolated to the same instant
    (trajectory); register='truth' integrates at the true pose instead (no localisation error: the grid rule alone).
    integrate(grid, points, pose, sim_time_s) replaces grid.integrate_scan(points, pose, sim_time_s) when given;
    on_scan(grid, row, t) is called after each scan (t from the first row). Returns the grid. For the rule before task K,
    set the grid's near_slack to inf."""
    root = snapshot(report); mount = robot(root); tris = house_triangles(root)
    rng = np.random.default_rng(seed); g = grid_factory(); g.sensor_xy = np.array(mount[0][:2])
    rows = rows_of(report); t0 = rows[0]['status']['sim_time_s']
    origin = json.loads((Path(report) / 'trial.json').read_text())['map_origin_ground_truth_evaluation_only']
    for i, truth, pose, stamp in trajectory(rows, steps):
        points = scan_to_points(cast(*tris, truth, rng, mount=mount), mount[0])
        if register == 'truth':
            pose = to_map(origin, truth)
        (integrate or type(g).integrate_scan)(g, points, pose, stamp)
        if on_scan:
            on_scan(g, rows[i], stamp - t0)
    return g


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('report'); ap.add_argument('--steps', type=int, default=4); ap.add_argument('--out')
    ap.add_argument('--old-rule', action='store_true', help='the grid update before task K (near_slack = inf)')
    ap.add_argument('--register', choices=('estimate', 'truth'), default='estimate')
    a = ap.parse_args()

    def grid():
        g = LidarGrid()
        if a.old_rule:
            g.near_slack = math.inf
        return g
    g = replay(a.report, grid_factory=grid, steps=a.steps, register=a.register)
    if a.out:
        Path(a.out).mkdir(parents=True, exist_ok=True); g.save(Path(a.out))
    print(json.dumps(g.summary(), indent=1))
