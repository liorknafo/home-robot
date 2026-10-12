"""Sync the round robot's geometry, masses, centre of mass and inertia from the sibling CAD package into
Content/RobotData/robot-parameters.json, and copy assembly-meshes.json.

Run the CAD build (build_robot.py, then validate_meshes.py) first.
    python Scripts/sync_robot_data.py [--cad PATH/TO/home-robot-r1/cad] [--dry-run]

Inputs (CAD, millimetres): cad/parameters.json, cad/component-manifest.json, cad/assembly-meshes.json and
cad/mass-properties.json. Output keys are metres in the robot frame: x forward, y LEFT, z up from the floor, origin
base_link = centre of the round shell = drive-axle midpoint, at floor level. RobotPawn reads the geometry keys at
BeginPlay; the controller reads the same keys.

Masses, centre of mass and inertia come ONLY from cad/mass-properties.json (written by build_robot.py from the CAD
solids: printed parts at their fill, bought parts at their CAD centroids, no ballast). Its records whose separate_body
is true ('Wheels': both drive wheels; 'Caster ball') are the simulator's own wheel / caster bodies; every other record
is the chassis Body. The file's body / total masses and COMs are re-derived from its records and from the wheel and
caster positions (parameters.json) and must agree; a missing or inconsistent file is an error (no fallback to the
older id-based estimate, whose part ids no longer exist in the CAD). The whole-robot COM must stay at least 12 mm
behind the drive axle (COM_LIMIT_M), and the CAD's worst sensitivity case (cad/validation.json com_estimate) at least 10 mm
(COM_WORST_LIMIT_M); user decision 2026-10-11, task F (was 15 mm, nominal only).

CAD contract (parameters.json, mm): rear_caster = [x, y, z] where z is the BALL BOTTOM above the floor with the robot at
rest (0 = touching; 0..3 mm accepted, anything else raises). bump_zone_edges_deg (optional, default
[-90,-55,-20,20,55,90]) is the full ascending list (the positive half [20,55,90] is mirrored) and its outer values must be
-/+bumper_span_deg (the HALF-span, default = the outer edge). cliff_sensors are [x, y] mm, 60 mm <= radius < shell_radius;
cliff_z 20..80 mm. base_link_height (optional, default 85) is the simulated body-origin height. DriveCore (and robot_client odometry) still compile wheel radius .04 / track .228, so
those, and the axle-through-base_link layout, are guarded below.
"""
from pathlib import Path
import argparse, json, math, shutil, subprocess, sys

ROOT = Path(__file__).resolve().parents[1]
COM_LIMIT_M = -.012                                     # robot COM (nominal) must be at least 12 mm behind the drive axle
COM_WORST_LIMIT_M = -.010                               # ... and the CAD's worst sensitivity case at least 10 mm (task F, user decision 2026-10-11)
# Values the code relies on (DriveCore.h, controller odometry): a change needs a code edit first. camera_height / lidar_x /
# lidar_height are the R1 mounts (low-profile spec 2026-10-06 sec. 2: LiDAR plane 147.5 mm; task F 2026-10-11: camera in the
# bumper band at 80 mm, chosen with tests/camera_height_sweep.py and the Unreal shower-tray drives, and the LD14 turned round,
# spin axis x 54: as far forward as its 0.1 m blind circle stays inside the pressed bumper, user decision; task L 2026-10-12
# (LIO-120, user request): the LD14 20 mm lower, LiDAR plane 127.5 mm, robot top 135.15 mm, the lowest the CAD scan-band rule
# allows); RobotPawn and the controller read them from robot-parameters.json, so this guard only makes a further CAD move a
# deliberate step. The LD14's 180 deg turn (parameters.json lidar_yaw_deg) only matters to a real-robot driver: the simulator
# reports scans in the robot frame.
GUARDED = {'wheel_diameter': 80, 'wheel_track': 228, 'wheel_x': 0, 'camera_baseline': 0, 'camera_height': 80.0,
           'lidar_x': 54, 'lidar_height': 127.5}
REQUIRED = ['outer_radius', 'shell_radius', 'rear_caster', 'cliff_sensors', 'cliff_z', 'camera_x']
MASS_PROPERTIES = 'mass-properties.json'
SEPARATE_BODIES = ('Wheels', 'Caster ball')            # mass-properties.json records the simulator models as their own bodies
MASS_TOL_KG, COM_TOL_MM, POS_TOL_MM = .0005, .1, .5     # consistency tolerances (the CAD rounds masses to 0.01 g, positions to .01 mm)
DEFAULT_ZONE_EDGES = [-90, -55, -20, 20, 55, 90]        # bits 0..4: right, front_right, centre, front_left, left
ZONE_NAMES_5 = ['right', 'front_right', 'centre', 'front_left', 'left']
BASE_LINK_HEIGHT_MM = 85                                # RobotPawn BodyHeight (compiled): the controller's base_link + offset


def fail(msg):
    raise ValueError(msg)


def cad_worst_com(cad):
    """(worst sensitivity-case COM x in mm, case name) from cad/validation.json com_estimate, or None when it is missing."""
    try:
        c = json.loads((cad / 'validation.json').read_text())['com_estimate']
        return float(c['sensitivity_worst_com_x_mm']), c.get('sensitivity_worst_case', '?')
    except (OSError, KeyError, ValueError, TypeError):
        return None


def bbox_centre(b):
    return [(b[i] + b[i + 3]) / 2 for i in range(3)]


def cad_commit(cad):
    """Short git commit that last changed the CAD directory (and '+dirty' when it has uncommitted changes), or None."""
    try:
        sha = subprocess.run(['git', '-C', str(cad), 'log', '-1', '--format=%h', '--', '.'], capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(['git', '-C', str(cad), 'status', '--porcelain', '--', '.'], capture_output=True, text=True, check=True).stdout.strip()
        return sha + ('+dirty' if dirty else '')
    except (OSError, subprocess.CalledProcessError):
        return None


def load_mass_properties(cad, prm, axle_mm, half_track_mm, wheel_r_mm, rc, ball_r_mm):
    """cad/mass-properties.json -> (body records, wheel mass each, wheel z, ball mass, ball centre, body mass, body COM,
    total mass, robot COM, body inertia, info). Raises on a missing file or any inconsistency."""
    path = cad / MASS_PROPERTIES
    if not path.is_file():
        fail(f'{path} is missing: the CAD build (build_robot.py) writes it and it is the only source of masses, centre of mass '
             'and inertia; nothing written')
    mp = json.loads(path.read_text())
    for k in ('body_mass_kg', 'body_com_mm', 'total_mass_kg', 'total_com_mm', 'body_inertia_kg_m2', 'caster_x_mm', 'wheel_x_mm', 'records'):
        if k not in mp:
            fail(f'{path} lacks {k!r}; nothing written')
    recs = mp['records']
    for r in recs:
        if not (isinstance(r.get('item'), str) and isinstance(r.get('mass_kg'), (int, float)) and r['mass_kg'] > 0
                and isinstance(r.get('position_mm'), list) and len(r['position_mm']) == 3 and 'separate_body' in r):
            fail(f'{path}: malformed record {r!r}')
    names = [r['item'] for r in recs]
    if len(set(names)) != len(names):
        fail(f'{path}: duplicate record items {sorted(n for n in set(names) if names.count(n) > 1)}')
    sep = {r['item']: r for r in recs if r['separate_body']}
    if set(sep) != set(SEPARATE_BODIES):
        fail(f'{path}: separate_body records must be exactly {SEPARATE_BODIES}, got {sorted(sep)}')
    body = [r for r in recs if not r['separate_body']]
    # Axle and caster positions: the file must describe the same robot as parameters.json.
    if abs(mp['wheel_x_mm'] - axle_mm) > 1e-6 or abs(mp['caster_x_mm'] - rc[0]) > 1e-6:
        fail(f'{path}: wheel_x_mm {mp["wheel_x_mm"]} / caster_x_mm {mp["caster_x_mm"]} disagree with parameters.json '
             f'wheel_x {axle_mm} / rear_caster x {rc[0]}')
    wheels, ball = sep['Wheels'], sep['Caster ball']
    wp, bp = wheels['position_mm'], ball['position_mm']
    if abs(wp[0] - axle_mm) > POS_TOL_MM or abs(wp[1]) > POS_TOL_MM or abs(wp[2] - wheel_r_mm) > POS_TOL_MM:
        fail(f'{path}: Wheels record (the pair) at {wp} mm; expected x = wheel_x {axle_mm}, y 0, z = wheel radius {wheel_r_mm}')
    ball_want = [rc[0], rc[1], ball_r_mm + (rc[2] if len(rc) > 2 else 0)]
    if math.dist(bp, ball_want) > POS_TOL_MM:
        fail(f'{path}: Caster ball record at {bp} mm; expected the ball centre {ball_want} mm (rear_caster + ball radius)')
    # Body: re-derive mass and COM from the records.
    mb = sum(r['mass_kg'] for r in body)
    cb = [sum(r['mass_kg'] * r['position_mm'][i] for r in body) / mb for i in range(3)]
    if abs(mb - mp['body_mass_kg']) > MASS_TOL_KG or math.dist(cb, mp['body_com_mm']) > COM_TOL_MM:
        fail(f'{path}: body_mass_kg {mp["body_mass_kg"]} / body_com_mm {mp["body_com_mm"]} disagree with its records '
             f'({mb:.5f} kg at {[round(x, 3) for x in cb]} mm)')
    # Whole robot as the simulator builds it: Body + one wheel each side (track/2) + the caster ball.
    wheel_each = wheels['mass_kg'] / 2
    others = [(wheel_each, [wp[0], half_track_mm, wp[2]]), (wheel_each, [wp[0], -half_track_mm, wp[2]]), (ball['mass_kg'], bp)]
    mt = mb + sum(m for m, _ in others)
    ct = [(mb * cb[i] + sum(m * p[i] for m, p in others)) / mt for i in range(3)]
    if abs(mt - mp['total_mass_kg']) > MASS_TOL_KG or math.dist(ct, mp['total_com_mm']) > COM_TOL_MM:
        fail(f'{path}: total_mass_kg {mp["total_mass_kg"]} / total_com_mm {mp["total_com_mm"]} disagree with the body, wheels '
             f'and caster ball ({mt:.5f} kg at {[round(x, 3) for x in ct]} mm)')
    inertia = mp['body_inertia_kg_m2']
    if (len(inertia) != 3 or any(len(row) != 3 for row in inertia) or any(abs(inertia[i][j] - inertia[j][i]) > 1e-9 for i in range(3) for j in range(3))
            or any(inertia[i][i] <= 0 for i in range(3))
            or any(inertia[a][a] + inertia[b][b] < inertia[c][c] - 1e-9 for a, b, c in ((0, 1, 2), (1, 2, 0), (2, 0, 1)))):
        fail(f'{path}: body_inertia_kg_m2 {inertia} is not a symmetric inertia tensor with positive principal values')
    info = dict(file=str(path), cad_commit=cad_commit(cad), body_inertia_about=mp.get('body_inertia_about', 'body_com_mm'),
                body_mass_kg=mp['body_mass_kg'], body_com_mm=mp['body_com_mm'], total_mass_kg=mp['total_mass_kg'],
                total_com_mm=mp['total_com_mm'])
    return dict(body=body, wheel_each=wheel_each, wheels=wheels, ball=ball, body_mass=mb, body_com=cb, total=mt, robot_com=ct,
                inertia=inertia, info=info)


def zone_edges(raw):
    e = sorted(float(x) for x in raw)
    if e and e[0] > 0:
        e = [-x for x in reversed(e)] + e
    if len(e) < 2 or any(b <= a for a, b in zip(e, e[1:])):
        fail(f'bump_zone_edges_deg must be strictly ascending, got {raw}')
    return e


def centre_zone(e):
    return next(i for i in range(len(e) - 1) if e[i + 1] >= 0)


def switch_masks(e):
    """(left, right) microswitch zone masks: the centre zone trips both; left = centre and every zone to its left (+az)."""
    c = centre_zone(e)
    return sum(1 << i for i in range(c, len(e) - 1)), sum(1 << i for i in range(c + 1))


def order_cliffs(raw):
    """[[x, y] mm, ...] -> metres in FL, FR, RL, RR order (y left); exactly one sensor per quadrant."""
    if len(raw) != 4 or any(len(p) < 2 for p in raw):
        fail(f'cliff_sensors must hold 4 [x, y] pairs (FL, FR, RL, RR), got {raw}')
    if any(not 60 <= math.hypot(p[0], p[1]) for p in raw):
        fail(f'cliff_sensors must be millimetres at least 60 mm from base_link (metres or cm given?), got {raw}')
    out = []
    for fx, fy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        match = [p for p in raw if p[0] * fx > 0 and p[1] * fy > 0]
        if len(match) != 1:
            fail(f'cliff_sensors need exactly one sensor per quadrant (FL, FR, RL, RR), got {raw}')
        out.append([match[0][0] / 1000, match[0][1] / 1000])
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--cad', type=Path, default=ROOT.parent / 'home-robot-r1/cad')
    ap.add_argument('--dest', type=Path, default=ROOT / 'Content/RobotData')
    ap.add_argument('--dry-run', action='store_true', help='compute and validate, print the result, write nothing')
    args = ap.parse_args(argv)
    cad, dest = args.cad, args.dest
    prm = json.loads((cad / 'parameters.json').read_text())
    for name, value in GUARDED.items():
        if prm.get(name) != value:
            fail(f'{name}={prm.get(name)!r} (expected {value}); update DriveCore/RobotPawn/controller and this guard before syncing')
    missing = [k for k in REQUIRED if k not in prm]
    if missing:
        fail(f'CAD parameters.json lacks {missing}; the round-robot CAD build must export them')
    parts = {x['id']: x for x in json.loads((cad / 'component-manifest.json').read_text())}
    meshes = {x['id']: x for x in json.loads((cad / 'assembly-meshes.json').read_text())}
    # The visual assembly must carry NVIDIA's own Jetson Nano B01 model (the CAD imports its STEP), never a stand-in box.
    jet = meshes.get('jetson')
    if jet is None or 'NVIDIA' not in jet.get('name', '') or len(jet.get('vertices', [])) // 3 < 10000:
        fail('assembly-meshes.json has no NVIDIA Jetson Nano B01 STEP mesh (id jetson); rebuild the CAD before syncing')
    p = dest / 'robot-parameters.json'
    data = json.loads(p.read_text())

    # ---------------- geometry (mm -> m) ----------------
    axle = prm['wheel_x'] / 1000
    # rigid_radius_m is the fixed REAR shell (rear_shell_radius, 155.5 mm in the round CAD); shell_radius is the moving
    # bumper shell under the foam skin. Older exports without rear_shell_radius use shell_radius for both.
    skin, bumper_shell = prm['outer_radius'] / 1000, prm['shell_radius'] / 1000
    rigid = prm.get('rear_shell_radius', prm['shell_radius']) / 1000
    if 'rear_shell_radius' not in prm:
        print('warning: parameters.json has no rear_shell_radius; rigid_radius_m = shell_radius', file=sys.stderr)
    rc = prm['rear_caster']
    if len(rc) < 2:
        fail(f'rear_caster must be [x, y, z] mm, got {rc}')
    rc_z = (rc[2] if len(rc) > 2 else 0) / 1000
    # z is the ball BOTTOM above the floor at rest (0 = touching). A ball-centre height (12.7) would float the simulated
    # caster; a negative value would bury it.
    if not 0 <= rc_z * 1000 <= 3:
        fail(f'rear_caster z = {rc_z * 1000} mm must be the ball bottom above the floor, 0..3 mm (0 = touching), got {rc}')
    if abs(rc[0]) < 30:
        fail(f'rear_caster x {rc[0]} must be millimetres at least 30 mm from the axle (metres given?), got {rc}')
    if 'caster_ball_radius' in prm:                         # the CAD's key (build_robot.py RB)
        ball_r = prm['caster_ball_radius'] / 1000
    elif 'caster_ball_diameter' in prm:
        ball_r = prm['caster_ball_diameter'] / 2000
    else:
        ball_r = .0127
        print('warning: parameters.json has neither caster_ball_radius nor caster_ball_diameter; using a 12.7 mm ball radius',
              file=sys.stderr)
    if not .003 <= ball_r <= .03:
        fail(f'caster ball radius {ball_r * 1000} mm must be millimetres, 3..30 mm')
    shells = [parts[i]['bounds'] for i in parts if i.startswith(('P_bumper', 'P_rear_shell'))]
    shell_bottom = (min(b[2] for b in shells) if shells else prm.get('shell_bottom', 18)) / 1000
    # The cover may be one part (P_lid) or printed sectors (P_lid_front, ...); lid posts and the LiDAR tray are not the cover.
    lids = [parts[i]['bounds'] for i in parts if i.startswith('P_lid') and not i.startswith(('P_lid_post', 'P_lidar'))]
    body_top = (prm['body_top'] if 'body_top' in prm else max(b[5] for b in lids) if lids else 140) / 1000
    # Bumper skin top edge: the probe rows stop 2 mm under it (RobotGeometry::bump_rows); the collider above it is conservative.
    bumpers = [parts[i]['bounds'] for i in parts if i.startswith('P_bumper')]
    bz = prm.get('bumper_z')
    bumper_top = (max(b[5] for b in bumpers) if bumpers else bz[1] if bz else body_top * 1000) / 1000
    if bumpers and bz and abs(max(b[5] for b in bumpers) - bz[1]) > 1:
        print(f'warning: P_bumper bounds top {max(b[5] for b in bumpers)} mm vs bumper_z {bz} mm; the bounds are used', file=sys.stderr)
    if not shell_bottom + .01 < bumper_top <= body_top + 1e-9:
        fail(f'bumper top {bumper_top} m must lie between the shell bottom + 10 mm ({shell_bottom + .01} m) and the body top {body_top} m')
    rear_shells = [parts[i]['bounds'] for i in parts if i.startswith('P_rear_shell')]
    if rear_shells and abs(-min(b[0] for b in rear_shells) / 1000 - rigid) > .001:
        print(f'warning: P_rear_shell bounds reach x {min(b[0] for b in rear_shells)} mm but rigid_radius_m is {rigid} m', file=sys.stderr)
    edges = zone_edges(prm.get('bump_zone_edges_deg', DEFAULT_ZONE_EDGES))
    span = float(prm.get('bumper_span_deg', edges[-1]))
    if abs(edges[0] + span) > 1e-6 or abs(edges[-1] - span) > 1e-6:
        fail(f'bump_zone_edges_deg {edges} must run from -bumper_span_deg to +bumper_span_deg ({span}; the half-span)')
    left_mask, right_mask = switch_masks(edges)
    geometry = dict(
        drive_axle_x_m=axle, wheel_radius_m=prm['wheel_diameter'] / 2000, wheel_track_m=prm['wheel_track'] / 1000,
        body_radius_m=skin, rigid_radius_m=rigid, bumper_shell_radius_m=bumper_shell,
        bumper_span_deg=span, bump_zone_edges_deg=edges,
        bump_zone_names=ZONE_NAMES_5 if len(edges) == 6 else [f'zone_{i}' for i in range(len(edges) - 1)],
        bump_left_switch_zone_mask=left_mask, bump_right_switch_zone_mask=right_mask,
        rear_caster_xy_m=[rc[0] / 1000, rc[1] / 1000], rear_caster_z_m=rc_z, caster_x_m=rc[0] / 1000,
        caster_ball_radius_m=ball_r,
        cliff_sensors_m=order_cliffs(prm['cliff_sensors']), cliff_z_m=prm['cliff_z'] / 1000,
        camera_mount_m=[prm['camera_x'] / 1000, prm.get('camera_y', 0) / 1000, prm['camera_height'] / 1000],
        lidar_mount_m=[prm['lidar_x'] / 1000, prm.get('lidar_y', 0) / 1000, prm['lidar_height'] / 1000],
        shell_bottom_z_m=shell_bottom, body_top_z_m=body_top, bumper_top_z_m=bumper_top,
        base_link_height_m=prm.get('base_link_height', BASE_LINK_HEIGHT_MM) / 1000)
    if not 20 <= prm['cliff_z'] <= 80:
        fail(f'cliff_z {prm["cliff_z"]} must be millimetres, 20..80 (the cliff gate is 0.02-0.08 m around the nominal 42.5 mm)')
    if abs(geometry['base_link_height_m'] - BASE_LINK_HEIGHT_MM / 1000) > 1e-6:
        fail(f'base_link_height {prm["base_link_height"]} mm differs from the compiled RobotPawn BodyHeight {BASE_LINK_HEIGHT_MM} mm')
    # The same mounts relative to the body origin (base_link_height_m above the floor): the controller's "base_link + offset"
    # convention (mono.camera() z .065 above .085). camera_mount_m / lidar_mount_m z stay heights above the floor.
    h = geometry['base_link_height_m']
    geometry['camera_mount_from_body_origin_m'] = [*geometry['camera_mount_m'][:2], round(geometry['camera_mount_m'][2] - h, 6)]
    geometry['lidar_mount_from_body_origin_m'] = [*geometry['lidar_mount_m'][:2], round(geometry['lidar_mount_m'][2] - h, 6)]
    for k in ('bumper_axis_deg',):
        if k in prm:
            geometry[k] = prm[k]
    if 'bumper_travel' not in prm:
        print('warning: parameters.json has no bumper_travel; using 5 mm for the simulated bumper hard stop', file=sys.stderr)
    geometry['bumper_travel_m'] = prm.get('bumper_travel', 5) / 1000
    if not 0 < geometry['bumper_travel_m'] < .015:
        fail('bumper_travel must be between 0 and 15 mm (the simulated probes start 15 mm inside the skin)')
    # Consistency the simulator relies on (RobotPawn probes run 1.5 cm inside to 0.2 cm outside the skin).
    if not rigid < skin < rigid + .015:
        fail(f'outer_radius {skin} m must lie outside the rear shell radius {rigid} m by less than 15 mm')
    if not bumper_shell < skin:
        fail(f'outer_radius {skin} m must lie outside shell_radius {bumper_shell} m (the bumper shell under the foam)')
    if not 0 < geometry['bumper_span_deg'] <= 180:
        fail('bumper_span_deg must be in (0, 180]')
    if prm['wheel_track'] / 2000 + .012 >= rigid:
        fail('drive wheels do not fit inside the rigid shell')
    if not (rc[0] / 1000 < axle and math.hypot(rc[0], rc[1]) / 1000 < rigid):
        fail(f'rear_caster {rc} must be behind the axle and inside the shell')
    for x, y in geometry['cliff_sensors_m']:
        if math.hypot(x, y) >= rigid:
            fail(f'cliff sensor ({x}, {y}) m lies outside the rigid shell')
    if not shell_bottom < body_top:
        fail(f'shell bottom {shell_bottom} m must be below the body top {body_top} m')
    # The meshes must agree with parameters.json (RobotPawn re-centres tyre meshes on the axle from these keys).
    for ident, want in (('tire_left', [prm['wheel_x'], prm['wheel_track'] / 2]), ('tire_right', [prm['wheel_x'], -prm['wheel_track'] / 2]),
                        ('caster_ball', rc[:2])):
        if ident in parts:
            got = bbox_centre(parts[ident]['bounds'])
            if math.hypot(got[0] - want[0], got[1] - want[1]) > 3:
                print(f'warning: {ident} mesh centre {got[:2]} mm vs parameters {want} mm', file=sys.stderr)
    if 'caster_ball' in parts:                             # cross-check the z meaning and the diameter against the ball mesh
        b = parts['caster_ball']['bounds']
        for axis, ext in (('x', b[3] - b[0]), ('y', b[4] - b[1])):
            if abs(ext - 2000 * ball_r) > 1:
                fail(f'caster ball radius {ball_r * 1000} mm but the caster_ball mesh {axis} extent is {ext:.2f} mm (expected {2000 * ball_r:.2f} +/- 1)')
        if abs((b[5] - b[2]) - (b[3] - b[0])) <= 1:        # a full ball: its lowest point is the ball bottom
            if abs(b[2] - rc_z * 1000) > 1.5:
                fail(f'rear_caster z {rc_z * 1000} mm but the caster_ball mesh bottom is at {b[2]} mm')
        else:
            print(f'warning: caster_ball mesh is not a full ball (z extent {b[5] - b[2]:.1f} mm, diameter {b[3] - b[0]:.1f} mm); '
                  'rear_caster z not cross-checked', file=sys.stderr)
    # Whole-robot top and the LiDAR turret (the only part above the LiDAR scan plane): an overhang whose underside lies
    # between lidar_mount_m z and robot_top_z_m is invisible to the LiDAR, above the bumper and hits the turret.
    top = max(x['bounds'][5] for x in parts.values())
    if 'height_mm' in prm and abs(top - prm['height_mm']) > 1:
        print(f'warning: manifest top {top} mm vs parameters height_mm {prm["height_mm"]} mm; the manifest is used', file=sys.stderr)
    geometry['robot_top_z_m'] = round(top / 1000, 6)
    if 'lidar_turret' in parts:
        b = parts['lidar_turret']['bounds']
        geometry['lidar_turret_xy_m'] = [round(v / 1000, 6) for v in bbox_centre(b)[:2]]
        geometry['lidar_turret_radius_m'] = round(max(b[3] - b[0], b[4] - b[1]) / 2000, 6)
        geometry['lidar_turret_z_m'] = [round(b[2] / 1000, 6), round(b[5] / 1000, 6)]
    else:
        print('warning: no lidar_turret in the component manifest; lidar_turret_* not written', file=sys.stderr)

    # ---------------- masses, centre of mass and inertia: cad/mass-properties.json only ----------------
    m = load_mass_properties(cad, prm, prm['wheel_x'], prm['wheel_track'] / 2, prm['wheel_diameter'] / 2, rc, ball_r * 1000)
    info = m['info']
    records = [dict(item=r['item'], mass_kg=r['mass_kg'], position_mm=r['position_mm'], basis=r.get('basis', 'CAD mass-properties.json'))
               for r in m['body']]
    separate = [dict(item=r['item'], mass_kg=r['mass_kg'], position_mm=r['position_mm'], basis=r.get('basis', 'CAD mass-properties.json'))
                for r in (m['wheels'], m['ball'])]
    # The file's own values (verified against its records above, to its rounding).
    body, com = info['body_mass_kg'], list(info['body_com_mm'])
    total, robot_com = info['total_mass_kg'], list(info['total_com_mm'])
    margin = robot_com[0] / 1000 - axle
    print(f'body {body:.4f} kg, body COM ({com[0]:.2f}, {com[1]:.2f}, {com[2]:.2f}) mm; robot {total:.4f} kg, '
          f'robot COM ({robot_com[0]:.2f}, {robot_com[1]:.2f}, {robot_com[2]:.2f}) mm; axle x {axle * 1000:.1f} mm; '
          f'from {MASS_PROPERTIES} (CAD {info["cad_commit"] or "not a git checkout"})')
    print(f'body inertia about the body COM (kg m2): {m["inertia"]}')
    print(f'records re-derived: body {m["body_mass"]:.5f} kg at {[round(x, 3) for x in m["body_com"]]} mm, robot {m["total"]:.5f} kg '
          f'at {[round(x, 3) for x in m["robot_com"]]} mm')
    if 'com_estimate_mm' in prm:
        est = prm['com_estimate_mm']
        if isinstance(est, (int, float)):
            est = [est, 0, 0]
        if isinstance(est, list) and est and all(isinstance(x, (int, float)) for x in est):
            print(f'CAD com_estimate_mm {est} vs synced robot COM: dx {robot_com[0] - est[0]:+.2f} mm')
        else:
            print(f'warning: com_estimate_mm {est!r} is not a number or a list of numbers; not compared', file=sys.stderr)
    worst = cad_worst_com(cad)
    if worst is None:                                     # no worst case to check: hold the one COM we have to the worst-case limit
        print(f'warning: no worst-case COM in {cad / "validation.json"} (com_estimate.sensitivity_worst_com_x_mm); checking the nominal '
              f'COM against the worst-case limit {COM_WORST_LIMIT_M * 1000:.0f} mm only', file=sys.stderr)
        checks = [('robot COM x', margin, COM_WORST_LIMIT_M)]
    else:
        print(f'CAD worst-case COM x {worst[0]:+.2f} mm ({worst[1]})')
        checks = [('robot COM x', margin, COM_LIMIT_M), (f'worst-case COM x ({worst[1]})', worst[0] / 1000 - axle, COM_WORST_LIMIT_M)]
    for name, value, limit in checks:
        if value > limit:
            fail(f'{name} is {value * 1000:+.1f} mm from the drive axle; it must be <= {limit * 1000:.0f} mm '
                 '(behind the axle, so the robot rests on the rear caster). Move mass rearward or add ballast; nothing written.')

    data.pop('sync_status', None)                         # clears the pre-sync marker
    data.update(geometry)
    h, cam_z = geometry['base_link_height_m'], geometry['camera_mount_m'][2]
    data.update(records=records, separate_body_records=separate, body_mass_kg=body, total_mass_kg=total,
                wheel_mass_kg=m['wheel_each'], caster_ball_mass_kg=m['ball']['mass_kg'],
                estimated_body_com_mm=com, body_com_x_cm=com[0] / 10, body_com_z_cm=com[2] / 10,
                robot_com_mm=robot_com, robot_com_behind_axle_m=-margin,
                body_inertia_kg_m2=m['inertia'], body_inertia_about=info['body_inertia_about'],
                mass_properties_source=dict(file='home-robot-r1/cad/' + MASS_PROPERTIES, cad_commit=info['cad_commit']),
                frame='metres unless noted; robot frame x forward, y left, z up. base_link = round-shell centre = '
                      'drive-axle midpoint, AT FLOOR LEVEL; every mount z (camera_mount_m, lidar_mount_m, cliff_z_m, '
                      'shell_bottom_z_m, body_top_z_m, robot_top_z_m) is a height above the floor. base_link_height_m is the nominal '
                      'body-origin height (RobotPawn BodyHeight) used by the controller convention base_link + offset, e.g. '
                      f'camera z {h:g} + {round(cam_z - h, 6):g} = {cam_z:g} above the floor; camera_mount_from_body_origin_m and '
                      'lidar_mount_from_body_origin_m give the mounts in that body-origin form (z minus base_link_height_m). '
                      'rigid_radius_m is the fixed rear shell, bumper_shell_radius_m the moving bumper shell under the '
                      'skin (body_radius_m); bumper_top_z_m is the skin top edge (no bumper sensing above it); body_top_z_m the lid '
                      'top; robot_top_z_m the whole robot top (LiDAR turret cap); lidar_turret_* the turret cylinder (centre xy, '
                      'radius, z range), the only part above the LiDAR plane. rear_caster_z_m is the ball bottom above the floor at '
                      'rest (0 = touching). bump_zone_edges_deg is the full ascending list: zone i (bump_zones bit i, named '
                      'bump_zone_names[i]) spans edges[i]..edges[i+1], azimuth about base_link, + = left; its outer values '
                      'are -/+bumper_span_deg (the half-span). bump_left/right_switch_zone_mask: zones that trip each switch. '
                      'Masses kg, positions mm: records = the chassis Body (sum = body_mass_kg); separate_body_records = the '
                      'drive-wheel pair (wheel_mass_kg each) and the caster ball; body_inertia_kg_m2 is about estimated_body_com_mm.')
    lim = [x for x in data.get('limitations', []) if not x.startswith(('Chassis inertia', 'Printed-part masses', 'Masses, centre of mass',
                                                                       'Sensing blind band', 'Camera window'))]
    bt, lz = round(1000 * geometry['bumper_top_z_m'], 2), round(1000 * geometry['lidar_mount_m'][2], 2)
    rt = round(1000 * geometry.get('robot_top_z_m', geometry['lidar_mount_m'][2]), 2)
    lim.append(f'Sensing blind band: thin objects confined to {bt:g}-{lz:g} mm (above the bumper skin top, bumper_top_z_m, and '
               f'below the LiDAR scan plane) and object undersides at {lz:g}-{rt:g} mm (between the LiDAR plane and the robot top, '
               'robot_top_z_m) are invisible to the LiDAR and above the bumper; the upper structure (lid, LiDAR tray and turret; '
               'above the LiDAR plane only the turret) collides with them. The forward camera marks them on the map '
               '(floor_parallax: 3 cm up to robot_top_z_m + 2 cm, within 1.2 m ahead, while the robot drives toward them); stall '
               'marks (lidar_brain.mark_stall: the wheels turn, the LiDAR shows no motion, nothing is sensed), at most '
               'STALL_MARK_LIMIT per session, stay the backstop for what it misses (beside or behind the robot, a plain edge '
               'seen obliquely, floor too plain to measure the robot\'s motion).')
    cw = prm.get('camera_window')
    if cw:
        lim.append(f"Camera window (task F): the real bumper has a {cw['at_skin']['width_mm']:g} mm wide window at z "
                   f"{cw['at_skin']['z_mm'][0]:g}-{cw['at_skin']['z_mm'][1]:g} mm round the camera lens, across the bumper seam "
                   '(cad/parameters.json camera_window). The bumper probes here are uniform, so an object small enough to pass that '
                   'window, which would reach the lens unsensed on the real robot, trips the simulated bumper instead.')
    lim += ['Masses, centre of mass and body_inertia_kg_m2 come from the CAD (cad/mass-properties.json): printed parts at CAD volume x '
            'PETG 1.27 g/cm3 x a per-part fill, bought parts at their recorded masses at CAD solid centroids, each group a uniform '
            'solid of its CAD envelope for the inertia. Not weighed or measured.',
            'Chassis inertia and upper collider in Unreal: RobotPawn sets the Body inertia from body_inertia_kg_m2 (principal moments '
            'and axes, after the COM nudge; log INERTIA_FIX, state physics_body_inertia_kg_m2), not from its collider. Above the body '
            'collider (up to the lid top, body_top_z_m) only the LiDAR column collides: a 32-gon prism circumscribing the turret '
            'cylinder (lidar_turret_xy_m, lidar_turret_radius_m) from the lid top to the robot top; the parts of '
            'the LiDAR tray and LD14 housing outside the turret footprint are not colliders.']
    data['limitations'] = lim
    if args.dry_run:
        print(json.dumps({k: v for k, v in data.items() if k not in ('records', 'separate_body_records')}, indent=2))
        return data
    p.write_text(json.dumps(data, indent=2))
    shutil.copy2(cad / 'assembly-meshes.json', dest / 'assembly-meshes.json')
    print(f'CAD synced; robot mass {total:.4f} kg, COM {-margin * 1000:.2f} mm behind the axle. '
          'Restart Unreal and rerun tests.')
    return data


if __name__ == '__main__':
    main()
