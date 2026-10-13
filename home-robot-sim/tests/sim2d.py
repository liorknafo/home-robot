"""Offline closed-loop 2D harness for LidarBrain (no Unreal, no UDP).

Emulates the simulator <-> autonomy_service boundary without Unreal or UDP:
  * 20 Hz robot_state dicts with the same keys RobotPawn::SendState emits
    (sim_time_s, armed, fault, sensors{left_ticks,right_ticks,encoder_cpr,bumper,
    cliff_m,gyro_rad_s,specific_force_m_s2,lidar}).
  * LiDAR scans in the D200 wire format (ranges_mm, time_offsets_ms,
    angle_increment -2*pi/667, sensor_in_base_m = lidar_mount_m) built ONLY from
    tall walls (LiDAR plane at lidar_mount_m[2]); low obstacles and overhangs are invisible to it.
  * ROUND robot, geometry read from Content/RobotData/robot-parameters.json:
    rigid body = disc of rigid_radius_m about base_link; sensing skin = arc of
    body_radius_m about base_link from -bumper_span_deg to +bumper_span_deg (continuous,
    trips 1.5 mm outside the skin, reported on a 4 deg grid), reporting bumper_left/right (+/-20 deg centre band trips both), a
    latched 5-bit bump_zones mask (bump_zone_edges_deg) and a touch counter. It
    senses walls, low objects and curbs (>= 2 cm). A bump disarms the drive.
  * Pressed rule, firmware='spec' (default; Source/HomeRobotSim/DriveCore.h since the round robot): every new press
    disarms the drive, whatever the current command (DriveCore.h update(): press=bumper&&!bump -> disarm(); the wheel
    setpoints restart from 0). While the bumper stays pressed, command() may re-arm it only for a reverse (v <= 0 with
    |w| <= 2|v|/track, i.e. neither wheel forward) or a pure spin (v == 0, |w| <= .5) TOWARD the
    pressed side: left switch w >= 0, right switch w <= 0, both either.
    Known remaining simplifications against DriveCore.h (Unreal runs DriveCore.h itself): any disarm stops the wheels at once
    (DriveCore cuts the torque and the robot coasts); a forward command while pressed disarms here (DriveCore stays armed and
    ramps both setpoints to 0 at 0.25 m/s2); a pure spin here is v == 0 (DriveCore: |v| < 1e-4).
    Optional press roll-on (press_roll_m, default 0; task P): DriveCore cuts the torque on a press and the robot coasts on into
    the skin until the bumper stop. A press rising edge while driving forward keeps the wheel speeds for up to press_roll_m of
    travel (encoders and body alike), ending before the rigid disc would touch. Unreal: 5.8 mm after the trip (task G's
    trial); the shower-tray touches of trials full-mapping-20261012T183238 / T192331 at 8-10 cm/s rolled on past where the
    back-off's first reverse tick had frozen the end of the driven trail.
    firmware='drivecore': the PREVIOUS (box-robot) DriveCore.h, kept as a negative control: a press rising edge
    disarms; while pressed command() re-arms only for v <= 0 and |w| <= bump_yaw (.3) and never
    disarms on a later command; update() clamps BOTH wheel setpoints to <= 0 while pressed (a
    commanded pure spin becomes a pivot about one wheel that moves base_link backward).
  * Reverse audit: Bridge records the true base_link path; reverse_audit() measures
    how far any motion under a v < 0 command went from the ground base_link had
    covered before (the driven trail), so scenarios can fail on blind reversing.
  * Unicycle kinematics about the drive axle, drive_axle_x_m ahead of base_link
    (0 on the round robot: a turn in place leaves base_link where it is).
  * Optional 'overhangs': furniture whose UNDERSIDE lies in the blind band between the LiDAR scan plane
    (lidar_mount_m z) and the robot top (robot_top_z_m), e.g. the Unreal sofa base (underside 130 mm) if the plane drops below
    it (task L's 20 mm candidate: plane 127.5, top 135.15 mm; at 147.5 / 155.15 mm no house furniture is there). The beam passes
    under it (no return), it is above the bumper skin (no trip) and not a cliff: the controller gets NO signal.
    Only the LiDAR turret (lidar_turret_*), the one part above the scan plane, hits its edge: the body stops
    there (counted in contacts and overhang_hits) while the wheels may still tick.
  * Optional 'ledges' (task K): furniture whose UNDERSIDE lies just BELOW the LiDAR scan plane (ledge_drop, default
    1.75 cm: the Unreal sofa base, bottom 13 cm, under task K's 14.75 cm plane; a lower plane (task L) brings the house's bed
    frame and TV console, bottoms 12.5 / 12.46 cm, closer to it). Unreal's scan plane tilts with the body; each scan here
    draws a tilt (LEDGE_TILT: a level robot most of the time, 1-3.5 deg of pitch and up to 1.5 deg of roll about 1 scan in 7,
    as the truth pitch / roll of trial full-mapping-20261012T013115), and a beam returns from a ledge edge only while it is
    not more than ledge_drop below the plane there: close beams return, far ones may pass under (and on to the wall). The
    turret meets the edge as for an overhang; the body passes under it. Without ledges no tilt is drawn.
  * Optional 'sticky' strips where translation or turning slips (encoders tick,
    body does not move, gyro ~0) to emulate rug-edge / threshold stalls. Kind 'catch': a rug-edge strip the rear
    caster (rear_caster_xy_m) cannot climb onto: FORWARD translation that would carry the caster point from outside
    the strip into it is blocked at any speed (the straight retry's .06-.10 m/s included); the body has already
    passed over the low edge (no bumper, no LiDAR), the wheels keep turning. Reversing, turning and driving away are
    not blocked. Permanent, or transient with a hold time: the strip lets go `hold` s after it first blocked.
    Counted in catch_ticks (not a rigid-body contact: no body_blocked).
  * Optional spin slip (spin_slip_m_per_turn, default 0): while turning in place,
    base_link drifts by that much per full turn in a random direction drawn at the
    start of each spin (run_physics bounds real drift at ~1 cm per turn). The
    encoders and gyro do not see the drift; the LiDAR does.
  * Sensor errors (bay B11): Sim.state() passes the ideal gyro, accelerometer and encoders through the simulator's
    error model (controller/sim_sensor_model.py, Content/RobotData/sensor-errors.json: the configured MPU-6050 board,
    wheel and track calibration errors), exactly where autonomy_service applies it. sensor_errors=None follows the
    file / R1_SENSOR_ERRORS; the IMU noise sequence is seeded with the Sim seed. With errors on, Bridge first writes
    the robot's IMU level-calibration file into the brain's root, made by the robot's own bring-up procedure
    (controller/imu_level_calibration.synthetic), as the simulator provides it for the Unreal runs. Then (ruling PB28)
    Bridge powers the robot on at its start pose until its IMU start-up calibration is complete, before the scenario
    clock starts (Bridge.power_on).
The controller only ever sees the state dicts; world geometry stays here.
"""
import json
import math
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'controller'))
from sim_sensor_model import SensorErrorModel   # noqa: E402  (simulator side, never imported by the robot code)

PARAMETERS = json.loads((Path(__file__).resolve().parents[1] / 'Content/RobotData/robot-parameters.json').read_text())
AXLE = float(PARAMETERS.get('drive_axle_x_m', 0.))          # drive axle ahead of base_link (0: round robot)
TRACK = float(PARAMETERS['wheel_track_m'])
WHEEL_R = float(PARAMETERS['wheel_radius_m'])
CPR = float(PARAMETERS['encoder_cpr'])
SKIN_R = float(PARAMETERS['body_radius_m'])                 # contact skin (bumper) radius
BODY_R = float(PARAMETERS.get('rigid_radius_m', SKIN_R))    # rigid body: disc about base_link
SPAN = float(PARAMETERS.get('bumper_span_deg', 90))
EDGES = sorted(float(e) for e in PARAMETERS.get('bump_zone_edges_deg', [20, 55, 90]) if float(e) > 0)   # positive half (the sync writes the full list)
CLIFF_XY = np.array(PARAMETERS['cliff_sensors_m'], float)   # native order FL, FR, RL, RR
CASTER_XY = np.array(PARAMETERS.get('rear_caster_xy_m', [PARAMETERS.get('caster_x_m', -.075), 0.]), float)
CLIFF_Z = float(PARAMETERS['cliff_z_m'])
LIDAR_MOUNT = [float(v) for v in PARAMETERS['lidar_mount_m']]
# The parts above the LiDAR plane: the turret cylinder (centre xy about base_link, radius), and the robot top.
TURRET_XY = np.array(PARAMETERS.get('lidar_turret_xy_m', LIDAR_MOUNT[:2]), float)
TURRET_R = float(PARAMETERS.get('lidar_turret_radius_m', .03))
ROBOT_TOP = float(PARAMETERS.get('robot_top_z_m', LIDAR_MOUNT[2] + .008))
SKIN_REACH = .0015                                          # skin trips this far outside body_radius_m
PRESSED_SPIN_MAX = .5                                       # rad/s, spec-firmware pure spin while pressed
DRIVECORE_BUMP_YAW = .3                                     # DriveCore.h bump_yaw: |w| limit for re-arming while pressed


def LEDGE_TILT(rng):
    """(pitch, roll) rad of one scan's plane, for ledges: level within 0.15 deg 6 scans in 7; else 1-3.5 deg of pitch and up to
    1.5 deg of roll, either sign (trial full-mapping-20261012T013115's truth: |pitch| > 1 deg in 10 % of samples, > 2 deg in 5 %,
    p99 4 deg; |roll| > 0.3 deg in 10 %, p99 1.5 deg)."""
    if rng.random() < 6 / 7:
        return tuple(np.radians(rng.normal(0, .15, 2)))
    return (math.radians(rng.choice((-1, 1)) * rng.uniform(1., 3.5)), math.radians(rng.uniform(-1.5, 1.5)))


def pressed_motion_ok(v, w, left=True, right=True):
    """DriveCore (round robot): commands accepted while the bumper is pressed. A pure spin only
    toward the pressed side (left switch: w >= 0, right switch: w <= 0, both: either)."""
    toward = (left and right) or (left and w >= 0) or (right and w <= 0)
    return (v <= 0 and abs(w) <= 2 * abs(v) / TRACK + 1e-9) or (v == 0 and abs(w) <= PRESSED_SPIN_MAX and toward)


def _shell(az):
    """Sensing probe 1 mm outside the skin at azimuth az (deg) about base_link."""
    a = math.radians(az)
    return (SKIN_R + .001) * np.array([math.cos(a), math.sin(a)])


# Sensing skin: reported on a 4 deg grid over -SPAN..SPAN about base_link (see Sim.bumper_azimuths).
AZ = np.arange(-SPAN, SPAN + .1, 4.)
BUMPER = np.array([_shell(a) for a in AZ])
ZONE_BITS = [(-EDGES[2], -EDGES[1]), (-EDGES[1], -EDGES[0]), (-EDGES[0], EDGES[0]), (EDGES[0], EDGES[1]), (EDGES[1], EDGES[2] + .1)]
ZONES = np.array([next(i for i, (lo, hi) in enumerate(ZONE_BITS) if lo <= a < hi) for a in AZ])
# Rigid body perimeter samples (every 5 deg) of the disc, for low-object polygon tests.
BODY = BODY_R * np.c_[np.cos(np.radians(np.arange(0, 360, 5))), np.sin(np.radians(np.arange(0, 360, 5)))]


def _seg_hits(origin, dirs, segs, max_range=8.):
    """Ray/segment intersection; returns range per ray (inf if none)."""
    return _seg_ranges(origin, dirs, segs, max_range).min(axis=1) if len(segs) else np.full(len(dirs), np.inf)


def _seg_ranges(origin, dirs, segs, max_range=8.):
    """Ray/segment intersection; (rays, segments) ranges, inf where a ray misses a segment."""
    if not len(segs):
        return np.full((len(dirs), 0), np.inf)
    a = segs[:, 0, :]; b = segs[:, 1, :]
    e = b - a                                     # (S,2)
    d = dirs[:, None, :]                          # (R,1,2)
    w = a[None, :, :] - origin                    # (1,S,2)
    den = d[..., 0] * e[None, :, 1] - d[..., 1] * e[None, :, 0]
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (w[..., 0] * e[None, :, 1] - w[..., 1] * e[None, :, 0]) / den
        u = (w[..., 0] * d[..., 1] - w[..., 1] * d[..., 0]) / den
    ok = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1) & (t <= max_range)
    return np.where(ok, t, np.inf)


def box(cx, cy, sx, sy):
    x0, x1, y0, y1 = cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])


def poly_segments(poly):
    return [(poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))]


def inside(poly, p):
    x, y = p; n = len(poly); c = False
    for i in range(n):
        x1, y1 = poly[i]; x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            c = not c
    return c


def seg_point_distance(segs, p):
    a = segs[:, 0]; b = segs[:, 1]; ab = b - a
    t = np.clip(np.sum((p - a) * ab, axis=1) / np.maximum(1e-12, np.sum(ab * ab, axis=1)), 0, 1)
    return np.linalg.norm(a + t[:, None] * ab - p, axis=1)


class World:
    def __init__(self, walls, low=(), sticky=(), cliffs=(), curbs=(), overhangs=(), overhang_z=None, ledges=(), ledge_drop=.0175, overhang_stops_wheels=False):
        """walls: list of ((x0,y0),(x1,y1)) tall segments (LiDAR-visible).
        low: list of polygons for objects below the LiDAR plane (bumper-visible only).
        overhangs: list of polygons (footprints) of furniture whose underside is at overhang_z (default: midway
          between the LiDAR plane and the robot top), which must lie in that blind band: no LiDAR return, no bumper,
          no cliff; the LiDAR turret collides with the polygon edge (Sim.overhang_blocked).
        sticky: list of (polygon, kind) or (polygon, kind, hold_s) with kind in {'turn','stop','catch'}
          ('catch': see the module notes; hold_s None = permanent):
          'turn' -> in-place turns slip (no yaw), 'stop' -> translation < 6 cm/s slips.
        cliffs: list of polygons where the floor is missing (cliff sensors).
        ledges: list of polygons (footprints) of furniture whose underside is ledge_drop BELOW the LiDAR plane (module
          notes): LiDAR-visible edges while the tilted beam is high enough there; the turret collides with them."""
        self.walls = np.array(walls, float).reshape(-1, 2, 2)
        self.low = [np.asarray(p, float) for p in low]
        self.low_segs = np.array([s for p in self.low for s in poly_segments(p)], float).reshape(-1, 2, 2)
        self.sticky = [(np.asarray(s[0], float), s[1]) for s in sticky]
        self.catch_hold = [s[2] if len(s) > 2 else None for s in sticky]
        self.cliffs = [np.asarray(p, float) for p in cliffs]
        # curbs: low objects the wheels/caster cannot climb; block motion and touch the skin, no LiDAR
        self.curbs = [np.asarray(p, float) for p in curbs]
        self.overhangs = [np.asarray(p, float) for p in overhangs]
        # overhang_stops_wheels: the turret on an overhang stops the wheels too (Unreal: the drive torque cap holds them; task M's
        # trial full-mapping-20261012T144002), so no encoder ticks while it pushes: no jam by wheel slip, only pushed_stalled.
        self.overhang_stops_wheels = bool(overhang_stops_wheels)
        self.ledges = [np.asarray(p, float) for p in ledges]
        self.ledge_segs = np.array([s for p in self.ledges for s in poly_segments(p)], float).reshape(-1, 2, 2)
        self.ledge_drop = float(ledge_drop)
        self.overhang_z = (LIDAR_MOUNT[2] + ROBOT_TOP) / 2 if overhang_z is None else float(overhang_z)
        if self.overhangs and not LIDAR_MOUNT[2] < self.overhang_z < ROBOT_TOP:
            raise ValueError(f'overhang underside {self.overhang_z} m must lie between the LiDAR plane {LIDAR_MOUNT[2]} m and the robot top '
                             f'{ROBOT_TOP} m (lower: the LiDAR sees it, use walls; higher: the robot passes under it)')


class Sim:
    def __init__(self, world, pose=(0., 0., 0.), seed=0, lidar_noise_m=.004, reverse_on_bump=True, spin_slip_m_per_turn=0., firmware='spec',
                 sensor_errors=None, press_roll_m=0.):
        assert firmware in ('spec', 'drivecore')
        self.press_roll = float(press_roll_m); self.roll_left = 0.; self.roll_speeds = (0., 0.)
        self.sensor_model = SensorErrorModel(enabled=sensor_errors, run_seed=seed)
        self.reverse_on_bump = reverse_on_bump; self.firmware = firmware; self.fw_bump = False
        self.spin_slip = spin_slip_m_per_turn / (2 * math.pi); self.slip_dir = None; self.spin_drift = 0.
        self.w = world; self.pose = np.array(pose, float); self.t = 0.
        self.left_ticks = 0.; self.right_ticks = 0.; self.gyro = 0.
        self.armed = False; self.estop = False; self.cmd = (0., 0.); self.cmd_time = -1e9
        self.wl = 0.; self.wr = 0.               # ramped wheel surface speeds
        self.rng = np.random.default_rng(seed); self.noise = lidar_noise_m
        self.lidar_index = 0; self.lidar_frame = 0; self.lidar_start = 0.; self.ranges = []; self.times = []
        self.scan = None; self.bump = False; self.fault = 'disarmed'; self.contacts = 0
        self.bump_events = 0; self.zones = 0; self.left = self.right = False; self.overhang_hits = 0
        self.catch_ticks = 0; self.catch_entered = {}
        self.tilt = (0., 0.); self.tilt_rng = np.random.default_rng(seed + 7919)   # ledges only (own stream: other scenarios unchanged)
        self.truth_log = []

    # ---- simulator side -------------------------------------------------
    def base_to_world(self, pts, pose=None):
        p = self.pose if pose is None else pose
        c, s = math.cos(p[2]), math.sin(p[2])
        return np.asarray(pts) @ np.array([[c, s], [-s, c]]) + p[:2]

    def bumper_azimuths(self, pose):
        """Continuous sensing skin: every object edge (walls, low objects, curbs) that
        comes within SKIN_R + SKIN_REACH of base_link over azimuths -SPAN..SPAN trips
        it. The touched azimuth range is reported on the 4 deg probe grid (AZ), as
        RobotPawn's bump columns would; a corner between two grid azimuths reports
        the nearest one. (Point probes alone let a corner slip between them and past
        the 1.5 mm gap to the rigid disc.)"""
        segs = [np.asarray(self.w.walls, float).reshape(-1, 2, 2)]
        segs += [np.array(poly_segments(poly), float).reshape(-1, 2, 2) for poly in list(self.w.low) + list(self.w.curbs)]
        segs = np.concatenate(segs) if segs else np.empty((0, 2, 2))
        if not len(segs):
            return []
        c, s = math.cos(pose[2]), math.sin(pose[2]); rot = np.array([[c, -s], [s, c]])
        a = (segs[:, 0] - pose[:2]) @ rot; d = (segs[:, 1] - pose[:2]) @ rot - a
        reach = SKIN_R + SKIN_REACH
        dd = np.maximum(1e-12, (d * d).sum(1)); ad = (a * d).sum(1); aa = (a * a).sum(1)
        disc = ad * ad - dd * (aa - reach * reach)
        hits = set()
        for i in np.flatnonzero(disc > 0):
            root = math.sqrt(disc[i]); t0 = max(0., (-ad[i] - root) / dd[i]); t1 = min(1., (-ad[i] + root) / dd[i])
            if t0 > t1:
                continue
            tc = min(t1, max(t0, -ad[i] / dd[i]))
            angles = [math.degrees(math.atan2(*(a[i] + t * d[i])[::-1])) for t in (t0, tc, t1)]
            lo, mid, hi = angles[0], angles[1], angles[2]
            span = (hi - lo + 180) % 360 - 180          # signed short arc lo -> hi (passes mid)
            lo, hi = (lo, lo + span) if span >= 0 else (lo + span, lo)
            grid = [az for az in AZ if any(lo - 1e-9 <= az + k <= hi + 1e-9 for k in (-360, 0, 360))]
            if not grid and abs(mid) <= SPAN:
                grid = [AZ[int(np.argmin(np.abs(AZ - mid)))]]
            hits.update(float(g) for g in grid)
        return sorted(hits)

    def bumper_hit(self, pose):
        return bool(self.bumper_azimuths(pose))

    def body_blocked(self, pose):
        """Physical penetration test of the rigid disc (not the bumper probe)."""
        if len(self.w.walls) and seg_point_distance(self.w.walls, pose[:2]).min() < BODY_R:
            return True
        for poly in list(self.w.low) + list(self.w.curbs):
            # Exact disc / polygon overlap: an edge within BODY_R of the centre, or the centre inside.
            if inside(poly, pose[:2]) or seg_point_distance(np.array(poly_segments(poly), float), pose[:2]).min() < BODY_R:
                return True
        return self.overhang_blocked(pose)

    def overhang_blocked(self, pose):
        """The LiDAR turret disc (lidar_turret_xy_m / lidar_turret_radius_m) overlaps an overhang or ledge footprint."""
        overhangs = list(getattr(self.w, 'overhangs', ())) + list(getattr(self.w, 'ledges', ()))
        if not len(overhangs):
            return False
        c = self.base_to_world(TURRET_XY[None, :], pose)[0]
        return any(inside(poly, c) or seg_point_distance(np.array(poly_segments(poly), float), c).min() < TURRET_R for poly in overhangs)

    def cliff_ranges(self):
        pts = self.base_to_world(CLIFF_XY)
        return [-1. if any(inside(poly, q) for poly in self.w.cliffs) else CLIFF_Z for q in pts]

    def unsafe(self):
        hits = self.bumper_azimuths(self.pose)
        if hits and not self.bump:
            self.bump_events += 1
        self.bump = bool(hits); self.left = any(a >= -20 for a in hits); self.right = any(a <= 20 for a in hits)
        for a in hits:
            self.zones |= 1 << int(ZONES[list(AZ).index(a)])
        cliff = self.cliff_ranges()
        if self.bump:
            return 'bumper'
        if any(c < .02 or c > .08 for c in cliff):
            return 'cliff_or_invalid'
        return None

    def command(self, v, w, arm=False, stop=False):
        """Same semantics as DriveCore::command."""
        if stop:
            self.estop = True; self.armed = False
        # DriveCore.h caps reverse at max_reverse (.06 m/s); the previous core allowed -.15.
        self.cmd = (float(np.clip(v, -.15 if self.firmware == 'drivecore' else -.06, .15)), float(np.clip(w, -1, 1))); self.cmd_time = self.t
        reason = self.unsafe()
        if self.firmware == 'drivecore':
            # DriveCore::command: fault is the last update()'s hazard; bump the last update()'s bumper.
            hazard = reason not in (None, 'bumper')
            v, w = self.cmd
            if arm and not self.estop and not hazard and (not self.fw_bump or (v <= 0 and abs(w) <= DRIVECORE_BUMP_YAW)):
                self.armed = True
            return
        if arm and not self.estop and (reason is None or (reason == 'bumper' and self.reverse_on_bump and pressed_motion_ok(*self.cmd, self.left, self.right))):
            self.armed = True

    def clear_stop(self):
        """Same as DriveCore::clear_stop (the motor_power_on / release_estop command): the stop latch released, disarmed."""
        self.estop = False; self.armed = False

    def _sticky(self):
        return [k for poly, k in self.w.sticky if inside(poly, self.pose[:2])]

    def _caught(self, mv, mw, dt):
        """Forward motion (mv, mw) this step would carry the rear caster into a 'catch' strip that still holds
        (sticky kind 'catch', see the module notes). The trial pose uses the same unicycle step as step()."""
        catches = [i for i, (poly, k) in enumerate(self.w.sticky) if k == 'catch']
        if not catches:
            return False
        yaw = self.pose[2]; axle = self.pose[:2] + AXLE * np.array([math.cos(yaw), math.sin(yaw)])
        axle = axle + mv * dt * np.array([math.cos(yaw + mw * dt / 2), math.sin(yaw + mw * dt / 2)]); yaw2 = yaw + mw * dt
        trial = np.r_[axle - AXLE * np.array([math.cos(yaw2), math.sin(yaw2)]), yaw2]
        c0 = self.base_to_world(CASTER_XY[None, :])[0]; c1 = self.base_to_world(CASTER_XY[None, :], trial)[0]
        hold = getattr(self.w, 'catch_hold', [None] * len(self.w.sticky))
        for i in catches:
            poly = self.w.sticky[i][0]
            if inside(poly, c0) or not inside(poly, c1):
                continue
            if hold[i] is None or self.t - self.catch_entered.setdefault(i, self.t) < hold[i]:
                return True
        return False

    def step(self, dt=.01):
        reason = self.unsafe()
        if self.firmware == 'drivecore':
            # DriveCore::update: a press rising edge, a hazard, estop or the watchdog disarm; while pressed
            # both wheel setpoints are clamped to <= 0; then max-speed scaling and the ramp.
            press = self.bump and not self.fw_bump; self.fw_bump = self.bump
            if (reason not in (None, 'bumper')) or press or self.estop or self.t - self.cmd_time > .35:
                self.armed = False
        else:
            # DriveCore::update (DriveCore.h:46): every press rising edge disarms, whatever the current command; the wheel
            # setpoints restart from 0 (below) and only command() re-arms, for a pressed-allowed motion.
            press = self.bump and not self.fw_bump; self.fw_bump = self.bump
            bump_reverse_ok = reason == 'bumper' and self.reverse_on_bump and pressed_motion_ok(*self.cmd, self.left, self.right)
            if press or (reason and not bump_reverse_ok) or self.estop or self.t - self.cmd_time > .35:
                self.armed = False
        if press and self.press_roll and self.wl + self.wr > 0:
            self.roll_left = self.press_roll; self.roll_speeds = (self.wl, self.wr)   # coasting on into the skin (press_roll_m)
        rolling = self.roll_left > 0
        if rolling:
            self.wl, self.wr = self.roll_speeds; self.roll_left -= (self.wl + self.wr) / 2 * dt
            if self.roll_left <= 0:
                self.roll_left = 0.; self.roll_speeds = None   # coasted to a stop: the next step starts from rest
        elif self.roll_speeds is None or not self.armed:
            self.roll_speeds = (0., 0.)                        # (DriveCore restarts the wheel setpoints from 0 on a re-arm)
            self.wl = self.wr = 0.
        else:
            v, w = self.cmd; ls, rs = v - w * TRACK / 2, v + w * TRACK / 2
            if self.firmware == 'drivecore':
                if self.bump: ls, rs = min(ls, 0.), min(rs, 0.)
                scale = max(1., max(abs(ls), abs(rs)) / .15); ls, rs = ls / scale, rs / scale
            ramp = lambda old, want: old + float(np.clip(want - old, -.25 * dt, .25 * dt))
            self.wl = ramp(self.wl, ls); self.wr = ramp(self.wr, rs)
        v = (self.wl + self.wr) / 2; w = (self.wr - self.wl) / TRACK
        kinds = self._sticky()
        slip_v = 'stop' in kinds and abs(v) < .06 and abs(v) > 1e-4
        slip_w = 'turn' in kinds and abs(w) > .05 and abs(v) < .03
        mv = 0. if slip_v else v; mw = 0. if slip_w else w
        if mv > 0 and self._caught(mv, mw, dt):
            mv = 0.; self.catch_ticks += 1
        yaw = self.pose[2]; axle = self.pose[:2] + AXLE * np.array([math.cos(yaw), math.sin(yaw)])
        axle = axle + mv * dt * np.array([math.cos(yaw + mw * dt / 2), math.sin(yaw + mw * dt / 2)]); yaw2 = yaw + mw * dt
        new = np.r_[axle - AXLE * np.array([math.cos(yaw2), math.sin(yaw2)]), yaw2]
        if self.spin_slip and abs(mw) > .05 and abs(mv) < .01:
            if self.slip_dir is None:
                a = self.rng.uniform(-math.pi, math.pi); self.slip_dir = np.array([math.cos(a), math.sin(a)])
            step = self.spin_slip * abs(mw) * dt; new[:2] += step * self.slip_dir; self.spin_drift += step
        elif abs(mw) <= .05:
            self.slip_dir = None
        held = False
        if rolling and self.body_blocked(new):
            self.roll_left = 0.; self.roll_speeds = (0., 0.); self.wl = self.wr = mv = mw = 0.; new = self.pose.copy()   # the bumper stop ends the roll-on
        if (mv or mw) and self.body_blocked(new) and not self.body_blocked(self.pose):
            ov = bool(self.overhang_blocked(new)); self.contacts += 1; self.overhang_hits += ov
            held = ov and self.w.overhang_stops_wheels
            mv = mw = 0.; new = self.pose.copy()       # contact stops the body; wheels may still tick
        self.pose = new; self.gyro = mw
        k = 0. if held else dt / (2 * math.pi * WHEEL_R) * CPR
        self.left_ticks += self.wl * k; self.right_ticks += self.wr * k
        self._lidar(dt)
        self.t += dt
        self.fault = reason or ('motor_power_off' if self.estop else 'watchdog' if self.t - self.cmd_time > .35 and self.cmd_time > -1e8 else 'disarmed' if not self.armed else 'none')
        self.truth_log.append((self.t, *self.pose, self.fault))

    def _lidar(self, dt):
        self.carry = getattr(self, 'carry', 0.) + dt * 4000.; count = int(self.carry); self.carry -= count
        if not count:
            return
        idx = self.lidar_index + np.arange(count)
        idx = idx[idx < 667] if self.lidar_index + count > 667 else idx
        angles = -2 * math.pi * idx / 667.
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        origin = self.pose[:2] + np.array([c * LIDAR_MOUNT[0] - s * LIDAR_MOUNT[1], s * LIDAR_MOUNT[0] + c * LIDAR_MOUNT[1]])
        dirs = np.c_[np.cos(angles + self.pose[2]), np.sin(angles + self.pose[2])]
        r = _seg_hits(origin, dirs, self.w.walls)
        if len(getattr(self.w, 'ledge_segs', ())):
            if self.lidar_index == 0:
                self.tilt = LEDGE_TILT(self.tilt_rng)
            # Height of the beam at each ledge edge relative to the level plane: range x slope along the beam's azimuth.
            t = _seg_ranges(origin, dirs, self.w.ledge_segs)
            slope = math.tan(self.tilt[0]) * np.cos(angles) + math.tan(self.tilt[1]) * np.sin(angles)
            r = np.minimum(r, np.where(t * slope[:, None] >= -self.w.ledge_drop, t, np.inf).min(axis=1))
        r = r + self.rng.normal(0, self.noise, len(r))
        if self.lidar_index == 0:
            self.lidar_start = self.t
        self.ranges += [(-1 if not np.isfinite(x) or x < .1 else round(x * 1000)) for x in r]
        self.times += [round((self.t - self.lidar_start) * 1000)] * len(r)
        self.lidar_index += len(idx)
        if self.lidar_index >= 667:
            self.lidar_frame += 1
            self.scan = dict(frame=self.lidar_frame, start_time_s=self.lidar_start, sim_time_s=self.t, angle_min_rad=0,
                             angle_increment_rad=-2 * math.pi / 667., range_min_m=.1, range_max_m=8, ranges_mm=self.ranges,
                             time_offsets_ms=self.times, sensor_in_base_m=list(LIDAR_MOUNT))
            self.ranges = []; self.times = []; self.lidar_index = 0

    def state(self):
        return self.sensor_model.apply(self.ideal_state())

    def ideal_state(self):
        sensors = dict(left_ticks=round(self.left_ticks), right_ticks=round(self.right_ticks), encoder_cpr=CPR,
                       bumper=self.bump, bumper_left=self.left, bumper_right=self.right, bumper_events=self.bump_events, bump_zones=self.zones,
                       cliff_m=self.cliff_ranges(), gyro_rad_s=[0., 0., self.gyro], specific_force_m_s2=[0., 0., 9.8])
        self.zones = 0   # zones latch between 20 Hz state messages
        if self.scan is not None:
            sensors['lidar'] = self.scan
        return dict(sim_time_s=self.t, armed=self.armed, fault=self.fault, sensors=sensors)


class Bridge:
    """Mirrors autonomy_service.Application.run + RobotIO for one brain.

    rearm_hook(brain) -> bool lets a redesigned brain ask for re-arming (today
    the service only arms once on mode start: autonomy_service.py:18-19,94)."""
    def __init__(self, brain, sim, rearm_hook=None):
        self.b = brain; self.sim = sim; self.arm_pending = False; self.rearm_hook = rearm_hook
        self.trace = []
        if sim.sensor_model.enabled and getattr(brain, 'root', None) is not None:
            # The robot's bring-up level calibration for this board (the real robot keeps it on disk).
            from imu_level_calibration import synthetic
            from autonomy.imu import CALIBRATION_FILE
            if not (Path(brain.root) / CALIBRATION_FILE).exists():
                synthetic(brain.root, SensorErrorModel(enabled=True, run_seed=sim.sensor_model.run_seed + 1000,
                                                       config=dict(sim.sensor_model.config, seed=sim.sensor_model.seed)))   # the same board
        self.power_on_s = self.power_on()

    def power_on(self, limit=10., dt=.05):
        """Ruling PB28: the scenario clock starts only after the robot reports its IMU start-up calibration complete (the
        real robot calibrates at power-on, before anyone starts Mapping). The robot rests at its start pose, motors off,
        and the brain gets rest states timestamped BEFORE the scenario's sim time, without stepping the simulator, so the
        LiDAR and slip random streams (and every scenario's numbers) are those of a robot that was already calibrated.
        Returns the power-on seconds it took (None without an IMU calibration)."""
        imu = getattr(self.b, 'imu', None)
        if imu is None:
            return None
        t0 = self.sim.t - limit
        for k in range(int(limit / dt)):
            if imu.startup_done:
                return round(k * dt, 2)
            self.b.predict(self.sim.sensor_model.apply(dict(self.sim.ideal_state(), sim_time_s=t0 + k * dt)))
        raise RuntimeError('the IMU start-up calibration did not complete during power-on')

    def start_mapping(self):
        self.b.start_mapping(self.sim.t); self.arm_pending = True; self.sim.clear_stop()

    def run(self, seconds, stop_when_idle=True, on_tick=None):
        end = self.sim.t + seconds
        while self.sim.t < end:
            state = self.sim.state()
            self.b.predict(state)
            if self.b.mode != 'idle':
                if 'lidar' in state['sensors']:
                    self.b.observe_scan(state['sensors']['lidar'])
                self.b.camera.last_stamp = state['sim_time_s']   # camera stub: fresh, no detections
            v, w = self.b.command(state); self.b.requested_motion = (v, w)
            active = self.b.mode != 'idle'
            if self.rearm_hook and active and self.rearm_hook(self.b):
                self.arm_pending = True
            if state['armed']:
                self.arm_pending = False
            self.sim.command(v, w, arm=self.arm_pending and active, stop=not active)
            self.trace.append(dict(t=state['sim_time_s'], mode=self.b.mode, phase=self.b.phase, msg=self.b.message,
                                   v=v, w=w, armed=state['armed'], fault=state['fault'], est=self.b.localizer.pose.copy(),
                                   truth=self.sim.pose.copy()))
            if on_tick:
                on_tick(self)
            path = [self.sim.pose[:2].copy()]
            for _ in range(5):
                self.sim.step(.01); path.append(self.sim.pose[:2].copy())
            self.trace[-1]['path'] = np.array(path)
            if stop_when_idle and not active:
                break
        return self.trace


def backward_tick(r):
    """The tick's TRUE base_link path moved backward (> .5 mm against its heading): a reverse
    command, the wheel ramp after one, or a pivot the motor core made out of a pressed spin."""
    path = r.get('path')
    if path is None:
        return False
    heading = np.array([math.cos(r['truth'][2]), math.sin(r['truth'][2])])
    return float((path[-1] - path[0]) @ heading) < -.0005


def reverse_audit(trace, tolerance=None):
    """Blind-reverse audit from the TRUE base_link path. A tick is a reverse when its command has
    v < 0 or base_link actually moved backward (backward_tick: the wheel ramp after a reverse command,
    or a clamp-made pivot). Trail = every position base_link reached in the other ticks, plus the start.
    Every position reached in a reverse tick must lie within `tolerance` of that trail: by default the
    controller's on_trail band (2.5 cm) plus the largest estimate-to-truth error measured in this
    trace (the controller checks its estimate; it plans straight reverses within 1.5 cm).
    max_off_trail_m is also reported against the bare 2.5 cm (within_2_5cm). No reverse episode
    (reverse ticks not separated by 5 cm of forward travel) may exceed REVERSE_LIMIT (.15 m) + 1 cm.
    Returns dict(reverse_ticks, reverse_m, max_episode_m, max_off_trail_m, off_trail_ticks, tolerance_m,
    loc_err_m, within_2_5cm, ok)."""
    # The estimate lives in the map frame (the start pose); align it to the world with the first record.
    first = next((r for r in trace if 'est' in r), None); loc_err = 0.
    if first is not None:
        e0, g0 = np.asarray(first['est'], float), np.asarray(first['truth'], float); d = g0[2] - e0[2]
        rot = np.array([[math.cos(d), -math.sin(d)], [math.sin(d), math.cos(d)]])
        loc_err = max(float(np.linalg.norm(rot @ (np.asarray(r['est'], float)[:2] - e0[:2]) + g0[:2] - np.asarray(r['truth'], float)[:2])) for r in trace if 'est' in r)
    if tolerance is None:
        tolerance = .025 + loc_err
    trail = []; worst = 0.; off = 0; ticks = 0; length = 0.; episode = 0.; longest = 0.; forward = 0.
    for k, r in enumerate(trace):
        path = r.get('path')
        if path is None:
            continue
        if not trail:
            trail.append(path[:1])
        step = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        if r['v'] < 0 or backward_tick(r):
            ticks += 1; length += step; episode += step; longest = max(longest, episode); forward = 0.
            known = np.vstack(trail)
            gap = float(np.linalg.norm(path[1:, None, :] - known[None, :, :], axis=2).min(axis=1).max())
            worst = max(worst, gap); off += gap > tolerance
        else:
            trail.append(path[1:])
            # A reverse episode ends after 5 cm of forward travel (LidarBrain.REVERSE_EPISODE_RESET).
            forward += step if r['v'] > 0 else 0.
            if forward >= .05: episode = 0.
    # One episode may cover REVERSE_LIMIT (.15 m) plus the last tick and the wheel ramp's stopping distance.
    return dict(reverse_ticks=ticks, reverse_m=round(length, 4), max_episode_m=round(longest, 4), max_off_trail_m=round(worst, 4), off_trail_ticks=off,
                tolerance_m=round(tolerance, 4), loc_err_m=round(loc_err, 4), within_2_5cm=worst <= .025, ok=off == 0 and longest <= .15 + .01)


# ---- closed-loop scenario: kitchen corner ---------------------------------------------
def kitchen_corner(gap=.01, diagonal=True):
    """Two walls meeting at 90 deg at (0, 0) (a 3 x 2.4 m room in +x, +y) and the round
    robot nose-in: its skin `gap` from both walls (base_link SKIN_R + gap from each).
    diagonal=True faces the corner (heading -135 deg); False faces the x=0 wall
    (heading 180 deg), so the y=0 wall is beside it on its left.
    Everything the robot can explore (its goal) lies behind it. Returns (world, pose)."""
    walls = [((0., 0.), (3., 0.)), ((3., 0.), (3., 2.4)), ((3., 2.4), (0., 2.4)), ((0., 2.4), (0., 0.)),
             ((1.6, 1.4), (2.2, 1.4))]          # a counter island so the room is not an empty box
    d = SKIN_R + gap
    pose = (d, d, -3 * math.pi / 4) if diagonal else (d, d, math.pi)
    return World(walls), pose


def run_kitchen_corner(brain_factory, gap=.01, diagonal=True, seconds=40., seed=0, deadline=20., spin_slip_m_per_turn=0., require_no_reverse=None, safe_wait_ok=False,
                       firmware='spec', pressed_spin_pure=None):
    """Mapping run from the corner. require_no_reverse (default: gap >= 3 mm): no reverse command at
    all during the corner escape window (until `deadline`, which covers the turn, the exit and the first
    metre of driving), whether or not the robot has left yet; the robot turns in place, slowly when close.
    Later reverses elsewhere in the room are only audited. Every run also fails
    on any reverse that leaves the true driven trail (reverse_audit). Pass: by `deadline` s the robot has turned around
    (heading changed > 90 deg, i.e. it faces away from the wall it was nosed into; the
    room diagonal is only 135 deg from the side layout's start heading, so a stricter
    threshold failed a robot that correctly drove out diagonally) and left (base_link
    > .25 m from the start) with zero
    rigid-body contacts and zero bumper touches. Also reports base_link drift during
    the initial turn in place (before any translation command) and whether it
    reversed before leaving.
    safe_wait_ok (slip-stress variants whose gap is below the spin drift budget): also pass when the robot
    did not escape but ended SAFE: zero rigid-body contacts, no reverse command in the whole run, the
    reverse audit clean and the session still alive (not idle). 'escaped' reports which of the two.
    'No reverse' is checked on the TRUE motion too (backward_ticks: base_link moved backward in a tick,
    e.g. a pivot the motor core made out of a pressed spin), not only on v < 0 commands.
    firmware: Sim pressed rule ('spec' or 'drivecore'); pressed_spin_pure: set on the brain if given."""
    if require_no_reverse is None:
        require_no_reverse = gap >= .003 - 1e-9
    world, pose = kitchen_corner(gap, diagonal)
    brain = brain_factory(); sim = Sim(world, pose=pose, seed=seed, spin_slip_m_per_turn=spin_slip_m_per_turn, firmware=firmware)
    if pressed_spin_pure is not None: brain.pressed_spin_pure = pressed_spin_pure
    bridge = Bridge(brain, sim, rearm_hook=lambda b: getattr(b, 'wants_arm', False))
    for _ in range(10):
        brain.predict(sim.state()); [sim.step(.01) for _ in range(5)]
    start = np.array(pose, float); state = dict(left=None, turned=None, contacts=None, bumps=None, drift=0., reversed=False, reverse_cmds=0, spin_from=None, backward=0, any_backward=0)

    def tick(br):
        t = sim.t; p = sim.pose; r = br.trace[-1]
        moved = float(np.linalg.norm(p[:2] - start[:2])); turned = abs(math.atan2(math.sin(p[2] - start[2]), math.cos(p[2] - start[2])))
        if state['turned'] is None and turned > math.pi / 2: state['turned'] = t
        if state['left'] is None and moved > .25: state['left'] = t
        if state['left'] is None and r['v'] < 0: state['reversed'] = True
        if r['v'] < 0 and t <= deadline: state['reverse_cmds'] += 1
        # base_link drift during the initial turn in place: until the first translation command.
        if state['spin_from'] is None and r['v'] != 0: state['spin_from'] = t
        if state['spin_from'] is None: state['drift'] = max(state['drift'], moved)
        if t <= deadline: state['contacts'] = sim.contacts; state['bumps'] = sim.bump_events
    bridge.start_mapping(); bridge.run(seconds, on_tick=tick)
    audit = reverse_audit(bridge.trace)
    backward = sum(1 for r in bridge.trace if r['t'] <= deadline and backward_tick(r))
    state['any_backward'] = sum(1 for r in bridge.trace if backward_tick(r) and r['v'] >= 0 and not any(q['v'] < 0 for q in bridge.trace if r['t'] - .6 <= q['t'] <= r['t']))
    escaped = (state['turned'] is not None and state['left'] is not None and state['left'] <= deadline and state['turned'] <= deadline
               and state['contacts'] == 0 and state['bumps'] == 0 and not (require_no_reverse and (state['reverse_cmds'] or backward)) and audit['ok'])
    safe = (sim.contacts == 0 and not any(r['v'] < 0 for r in bridge.trace) and not any(backward_tick(r) for r in bridge.trace) and audit['ok'] and brain.mode != 'idle')
    ok = escaped or (safe_wait_ok and safe)
    return dict(passed=bool(ok), escaped=bool(escaped), safe=bool(safe), gap_m=gap, diagonal=diagonal, spin_slip_m_per_turn=spin_slip_m_per_turn, turned_s=state['turned'], left_s=state['left'], body_contacts=state['contacts'],
                bumps=state['bumps'], max_spin_drift_m=round(state['drift'], 4), total_spin_slip_m=round(sim.spin_drift, 4), reversed_before_leaving=state['reversed'], reverse_commands=state['reverse_cmds'],
                backward_ticks=backward, uncommanded_backward_ticks=state['any_backward'], firmware=firmware, pressed_spin_pure=getattr(brain, 'pressed_spin_pure', None),
                require_no_reverse=require_no_reverse, max_spin_w_in_follow=round(max((abs(r['w']) for r in bridge.trace if r['phase'] == 'follow' and r['v'] == 0), default=0.), 3),
                reverse_audit=audit, mode=brain.mode, message=brain.message, end_pose=np.round(sim.pose, 3).tolist())


if __name__ == '__main__':
    import sys, tempfile
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'controller'))
    from autonomy.lidar_brain import LidarBrain
    for name in sys.argv[1:] or ['kitchen_corner']:
        tmp = tempfile.TemporaryDirectory()
        result = run_kitchen_corner(lambda: LidarBrain(tmp.name), diagonal=name != 'kitchen_corner_side')
        print(name, result, flush=True)


def run_side_press(brain_factory, az_deg=70., seconds=30., seed=0, wall=False, firmware='spec', pressed_spin_pure=None, give_up=False):
    """Pressed contact with no driven trail: a low post (bumper-only, LiDAR-blind)
    touches the skin at azimuth az_deg at start. Expected: the controller spins in
    place so the contact azimuth moves toward 0 (never past the skin end onto the
    rigid rear shell) and NEVER reverses off its (empty) driven trail. The post stays
    .5 mm outside the skin, inside the 1.5 mm trip reach, so the skin stays pressed:
    with no trail the robot cannot legally move further (the firmware refuses forward
    motion while pressed and a reverse would be blind): it must stand still, keep the session alive and
    wait for the bumper to release (waiting_safely). Reports the touched azimuths over time, rigid-body contacts, whether the
    contact ever left the arc, and the reverse audit.
    wall=True: a tall wall (LiDAR-visible) tangent to the skin at az_deg instead of
    the post; after the spin the nose faces the wall, still pressed: same outcome.
    firmware: Sim pressed rule; pressed_spin_pure: set on the brain if given. A brain that does NOT
    spin while pressed (the clamping DriveCore.h rule) must instead stand still: base_link may not move
    (< 1 mm), no spin command while pressed, the contact azimuth unchanged. Every run fails on any
    tick whose TRUE base_link motion was backward (backward_tick: a reverse or a clamp-made pivot).
    give_up=True (run it longer than CONTACT_WAIT_S): the robot must instead end the session, standing still, with the
    'Stuck against an obstacle' message (task G: bounded waiting, not until the time budget)."""
    walls = [((-1.5, -1.5), (1.5, -1.5)), ((1.5, -1.5), (1.5, 1.5)), ((1.5, 1.5), (-1.5, 1.5)), ((-1.5, 1.5), (-1.5, -1.5)),
             ((.6, .3), (.9, .3))]
    a = math.radians(az_deg); n = np.array([math.cos(a), math.sin(a)]); t = np.array([-n[1], n[0]])
    near = (SKIN_R + .0005) * n          # post face 0.5 mm outside the skin: the 1 mm probe is inside it
    post = np.array([near + .02 * t, near - .02 * t, near + .04 * n - .02 * t, near + .04 * n + .02 * t])
    if wall:
        walls = walls + [(tuple(near - .6 * t), tuple(near + .6 * t))]; post = near + .01 * n   # 'post' centre: just behind the wall face
        world = World(walls)
    else:
        world = World(walls, low=[post])
    brain = brain_factory(); sim = Sim(world, pose=(0., 0., 0.), seed=seed, firmware=firmware)
    if pressed_spin_pure is not None: brain.pressed_spin_pure = pressed_spin_pure
    expect_spin = bool(getattr(brain, 'pressed_spin_pure', True))
    bridge = Bridge(brain, sim, rearm_hook=lambda b: getattr(b, 'wants_arm', False))
    for _ in range(10):
        brain.predict(sim.state()); [sim.step(.01) for _ in range(5)]
    seen = []; centre = post.mean(axis=0) if post.ndim == 2 else post

    def tick(br):
        hits = sim.bumper_azimuths(sim.pose)
        rel = centre - sim.pose[:2]; az = math.degrees(math.atan2(rel[1], rel[0]) - sim.pose[2]); az = (az + 180) % 360 - 180
        seen.append((round(sim.t, 2), round(az, 1), bool(hits), br.b.phase, br.trace[-1]['v'], br.trace[-1]['w']))
    bridge.start_mapping(); bridge.run(seconds, on_tick=tick)
    pressed = [s for s in seen if s[2]]
    released = next((s[0] for s in seen if s[0] > 1 and not s[2]), None)
    audit = reverse_audit(bridge.trace); reversed_ = any(r['v'] < 0 for r in bridge.trace)
    tail = [r for r in bridge.trace if r['t'] >= bridge.trace[-1]['t'] - 5.]
    waiting_safely = (brain.mode == 'mapping' and brain.phase == 'contact' and 'Waiting for the bumper to release' in brain.message
                      and all(r['v'] == 0 and r['w'] == 0 for r in tail))
    gave_up = brain.mode == 'idle' and 'Stuck against an obstacle' in brain.message and all(r['v'] == 0 and r['w'] == 0 for r in tail)
    if give_up:
        waiting_safely = gave_up
    final_az = next((s[1] for s in reversed(pressed)), None)
    backward = sum(1 for r in bridge.trace if backward_tick(r)); moved = float(np.linalg.norm(sim.pose[:2]))
    pressed_spin = any(s[2] and s[4] == 0 and s[5] != 0 for s in seen)
    if expect_spin:
        ok = ((released is not None or waiting_safely) and max((abs(s[1]) for s in pressed), default=0) <= abs(az_deg) + 2
              and (released is not None or (final_az is not None and abs(final_az) <= 20)))
    else:
        ok = waiting_safely and not pressed_spin and moved < .001 and final_az is not None and abs(final_az - az_deg) <= 2
    ok = bool(ok and sim.contacts == 0 and not reversed_ and backward == 0 and audit['ok'] and (gave_up or not give_up))
    return dict(start_az=az_deg, firmware=firmware, pressed_spin_pure=getattr(brain, 'pressed_spin_pure', None), backward_ticks=backward, moved_m=round(moved, 4), pressed_spin=pressed_spin, contact_az_at_release=next((s[1] for s in reversed(seen) if s[2] and s[0] <= (released or 1e9)), None),
                max_abs_contact_az_while_pressed=max((abs(s[1]) for s in pressed), default=None), final_contact_az=final_az, released_s=released,
                body_contacts=sim.contacts, spun=any(s[4] == 0 and s[5] != 0 and s[3] == 'contact' for s in seen), reversed=reversed_,
                waiting_safely=waiting_safely, gave_up=gave_up, reverse_audit=audit,
                passed=ok,
                mode=brain.mode, message=brain.message, events=[e['message'] for e in brain.events][:6])


# ---- closed-loop scenario: overhang in the LiDAR's blind band ------------------------------------
def _legs(x0, x1, y0, y1, s=.04):
    """Four s x s legs (LiDAR-visible: they reach the floor) at the corners of a footprint."""
    out = []
    for x, y in ((x0 + s / 2, y0 + s / 2), (x1 - s / 2, y0 + s / 2), (x0 + s / 2, y1 - s / 2), (x1 - s / 2, y1 - s / 2)):
        b = box(x, y, s, s); out += [(tuple(b[i]), tuple(b[(i + 1) % 4])) for i in range(4)]
    return out


def overhang_world(kind, underside=None, stops_wheels=False):
    """A 4 x 3 m room with furniture whose underside is `underside` (default: midway between the LiDAR plane and the robot
    top; it was a fixed 150 mm for the 147.5 / 155.15 mm robot, which task L's lower candidates put above the robot top) on 4 cm
    corner legs. 'sofa': 2 x .85 m against the top wall (the LiDAR sees the legs and the wall under it);
    'table': a 1 x .6 m island (coffee table). stops_wheels: World overhang_stops_wheels. Start pose (0, 0, 0) = map frame."""
    room = [((-1, -1.5), (3, -1.5)), ((3, -1.5), (3, 1.5)), ((3, 1.5), (-1, 1.5)), ((-1, 1.5), (-1, -1.5))]
    fp = {'sofa': (.3, 2.3, .65, 1.5), 'table': (.8, 1.8, -.3, .3)}[kind]
    top = box((fp[0] + fp[1]) / 2, (fp[2] + fp[3]) / 2, fp[1] - fp[0], fp[3] - fp[2])
    return World(room + _legs(*fp), overhangs=[top], overhang_z=underside, overhang_stops_wheels=stops_wheels)


def run_overhang(brain_factory, kind, goals=(), seconds=800., goal_seconds=200., seed=0, sensor_errors=None, stops_wheels=False):
    """Map overhang_world(kind) (then Go-to each of `goals`). The overhang gives the controller no signal: no LiDAR
    return, no bumper, no cliff; only the LiDAR turret meets its edge, the body stops and the wheels still turn.
    Reports every hit episode (TRUE base_link position; consecutive blocked ticks are one episode) and the controller's
    stall marks (LidarBrain.stall_marks, map frame = world frame). PASS: mapping finished normally (complete), each
    Go-to 'Arrived' within 3 cm; the reverse audit is clean (no blind reverse, episodes within REVERSE_LIMIT); no
    rigid-body contact except the turret on the overhang and no bumper touch; every hit spot is recorded (a stall mark
    within .30 m of it); a marked spot is never hit again after its mark (no hit within .10 m of it later) and no spot
    takes more than 3 hits (the jam retries). seed and sensor_errors: as Sim. stops_wheels: the turret's push stops the wheels
    (overhang_world), so only LidarBrain.pushed_stalled can notice it."""
    brain = brain_factory(); sim = Sim(overhang_world(kind, stops_wheels=stops_wheels), seed=seed, sensor_errors=sensor_errors)
    bridge = Bridge(brain, sim, rearm_hook=lambda b: getattr(b, 'wants_arm', False))
    for _ in range(10):
        brain.predict(sim.state()); [sim.step(.01) for _ in range(5)]
    state = dict(hits=0, inside=False, seen=set()); episodes = []; marks = []

    def tick(br):
        hit = sim.overhang_hits > state['hits']; state['hits'] = sim.overhang_hits
        if hit and not state['inside']:
            episodes.append((round(sim.t, 1), sim.pose[:2].copy()))
        state['inside'] = hit
        for m in getattr(brain, 'stall_marks', []):      # (a Go-to starts a fresh list: track marks by position)
            key = tuple(np.round(np.asarray(m, float), 4))
            if key not in state['seen']:
                state['seen'].add(key); marks.append((round(sim.t, 1), np.asarray(m, float).copy()))
    bridge.start_mapping(); bridge.run(seconds, on_tick=tick)
    mapped = dict(complete=bool(brain.complete), t=round(sim.t), message=brain.message)
    trips = []
    for goal in goals:
        try:
            brain.begin_go_to(list(goal), sim.t)
        except ValueError as e:
            trips.append(dict(goal=goal, accepted=False, message=str(e))); continue
        bridge.arm_pending = True; sim.clear_stop(); t0 = sim.t; bridge.run(goal_seconds, on_tick=tick)
        trips.append(dict(goal=goal, accepted=True, message=brain.message, time_s=round(sim.t - t0, 1),
                          end_to_goal_cm=round(100 * float(np.linalg.norm(sim.pose[:2] - np.asarray(goal))), 1)))
    audit = reverse_audit(bridge.trace)
    recorded = all(any(np.linalg.norm(p - m) < .30 for _, m in marks) for _, p in episodes)
    rehit = [(t, np.round(p, 3).tolist()) for t, p in episodes if any(tm < t and np.linalg.norm(p - m) < .10 for tm, m in marks)]
    per_spot = max((sum(np.linalg.norm(p - q) < .10 for _, q in episodes) for _, p in episodes), default=0)
    ok = (mapped['complete'] and all(x['accepted'] and x['message'].startswith('Arrived.') and x['end_to_goal_cm'] <= 3 for x in trips)
          and audit['ok'] and sim.contacts == sim.overhang_hits and sim.bump_events == 0 and recorded and not rehit and per_spot <= 3)
    return dict(passed=bool(ok), kind=kind, mapping=mapped, goto=trips, hit_episodes=len(episodes), stall_marks=len(marks),
                hit_spots=[(t, np.round(p, 2).tolist()) for t, p in episodes], marks=[(t, np.round(m, 2).tolist()) for t, m in marks],
                all_hits_recorded=recorded, rehit_after_mark=rehit, max_hits_per_spot=per_spot, body_blocks=sim.contacts,
                overhang_block_ticks=sim.overhang_hits, bumps=sim.bump_events, contacts=len(brain.contacts), skips=len(brain.blacklist),
                jams=len(brain.jams), reverse_audit=audit, mode=brain.mode)


# ---- closed-loop scenario: floor the robot can see but never enter (task H) ------------------------------------------
def pocket_world(kind):
    """A 5 x 4 m room (x -1..4, y -2..2) with a 2 x 2 m pen in its corner (x 2..4, y 0..2), fenced on its two open sides by
    4 cm posts that reach the floor; the LiDAR sees between them into the pen. 'posts': 25 cm apart, 21 cm between faces
    (the body is 2 x body_radius_m = 32 cm wide): the pen can never be entered, like the floor under the sofa in the Unreal
    house (task H). 'door': the same with one 34 cm gap between faces: the body fits (1 cm each side) but the planner never
    plans through it, not even at the narrow-passage clearance (40 cm). Start pose (0, 0, 0) = map frame."""
    room = [((-1, -2), (4, -2)), ((4, -2), (4, 2)), ((4, 2), (-1, 2)), ((-1, 2), (-1, -2))]
    south = {'posts': [2. + .25 * k for k in range(8)], 'door': [2., 2.25, 2.5, 2.88, 3.13, 3.38, 3.63, 3.88]}[kind]
    centres = [(x, 0.) for x in south] + [(2., .25 * k) for k in range(1, 8)]
    posts = []
    for c in centres:
        b = box(c[0], c[1], .04, .04); posts += [(tuple(b[i]), tuple(b[(i + 1) % 4])) for i in range(4)]
    return World(room + posts)


def run_pocket(brain_factory, kind, seconds=900., seed=0, sensor_errors=None):
    """Map pocket_world(kind). At the end, on the robot's own map: the safe floor its planner cannot reach (the pen,
    'unreached_m2') and how much of it the body could enter ('enterable_m2', grid.unreached_enterable). PASS, 'posts':
    mapping complete although more than 1.5 m2 of safe floor is unreached (the scenario exercises the rule), less than 1.5 m2
    of it enterable. 'door': mapping NOT complete, with more than 1.5 m2 of the pen counted as enterable (a robot boxed out
    by its own margins never reports the house done). Both: the robot stopped on its own within `seconds` (less than the
    brain's time budget), clean reverse audit, no rigid-body contact, no bumper touch. seed and sensor_errors: as Sim."""
    brain = brain_factory(); sim = Sim(pocket_world(kind), seed=seed, sensor_errors=sensor_errors)
    bridge = Bridge(brain, sim, rearm_hook=lambda b: getattr(b, 'wants_arm', False))
    for _ in range(10):
        brain.predict(sim.state()); [sim.step(.01) for _ in range(5)]
    bridge.start_mapping(); trace = bridge.run(seconds)
    g = brain.grid; pose = brain.localizer.pose; safe = g.masks()[3]; reach = g.reachable(pose[:2], safe)
    unreached = float((safe & ~reach).sum() * g.res ** 2); enterable = float(g.unreached_enterable(pose[:2], safe).sum() * g.res ** 2)
    audit = reverse_audit(trace); clean = audit['ok'] and sim.contacts == 0 and sim.bump_events == 0
    finished = brain.mode == 'idle'                     # stopped on its own: `seconds` is less than the brain's time budget
    ok = clean and finished and (brain.complete and unreached > 1.5 and enterable < 1.5 if kind == 'posts' else not brain.complete and enterable > 1.5)
    return dict(passed=bool(ok), kind=kind, t=round(sim.t), complete=bool(brain.complete), unreached_m2=round(unreached, 2), enterable_m2=round(enterable, 2),
                free=round(g.summary()['free_area_m2'], 2), contacts=len(brain.contacts), skips=len(brain.blacklist), body_blocks=sim.contacts,
                bumps=sim.bump_events, reverse_audit=audit, message=brain.message, events=[e['message'] for e in brain.events][-4:])


# ---- closed-loop scenario: furniture whose edges the LiDAR returns only from close by (task K) --------------------------
LEDGE_FOOTPRINT = (.9, 3.0, 1.15, 2.0)       # x0, x1, y0, y1: a 2.1 x 0.85 m sofa base, its back against the north wall


def ledge_world():
    """A 5 x 4 m room (x -1..4, y -2..2) with a sofa-like ledge against its north wall (LEDGE_FOOTPRINT): its underside is
    1.75 cm below the LiDAR plane (World ledges), like the Unreal sofa base, so beams return from its edges from close by
    and pass under them from farther away when the robot tilts; beams skimming past its ends cross their cells too. The floor
    under it is seen through but never enterable: the turret meets its edge. Start pose (0, 0, 0) = map frame."""
    room = [((-1, -2), (4, -2)), ((4, -2), (4, 2)), ((4, 2), (-1, 2)), ((-1, 2), (-1, -2))]
    x0, x1, y0, y1 = LEDGE_FOOTPRINT
    return World(room, ledges=[box((x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0)])


def ledge_outline(g):
    """The map cells of the ledge's outline on its three free sides (the fourth is against the wall)."""
    x0, x1, y0, y1 = LEDGE_FOOTPRINT
    return sorted({g.cell((xx, y0 + .001)) for xx in np.arange(x0 + .01, x1, .01)} | {g.cell((xe, yy)) for xe in (x0 + .001, x1 - .001)
                                                                                  for yy in np.arange(y0 + .01, y1 - .05, .01)})


def ledge_floor(g, pose):
    """The robot's map of the floor under the ledge (its footprint less 5 cm all round): safe cells the planner reaches
    ('reachable_m2': coverage targets it can never reach), safe cells the body could enter by the task H rule
    ('enterable_m2': they hold the map open), and the outline's open cells (log-odds <= 0.8)."""
    x0, x1, y0, y1 = LEDGE_FOOTPRINT
    n = g.n; ii, jj = np.mgrid[0:n, 0:n]; x = g.origin[0] + (jj + .5) * g.res; y = g.origin[1] + (ii + .5) * g.res
    under = (x > x0 + .05) & (x < x1 - .05) & (y > y0 + .05) & (y < y1 - .05)
    safe = g.masks()[3]; reach = g.reachable(pose[:2], safe); ent = g.unreached_enterable(pose[:2], safe); outline = ledge_outline(g)
    return dict(reachable_m2=round(float((reach & under).sum() * g.res ** 2), 3), enterable_m2=round(float((ent & under).sum() * g.res ** 2), 3),
                outline_open=int(sum(g.logodds[c] <= .8 for c in outline)), outline_cells=len(outline))


LEDGE_CLOSE_M = .5     # an outline cell mapped occupied with its closest hit within this range counts as 'seen from close by'
LEDGE_NEAR_M = .75     # reported only: the same within .75 m (Unreal trial 4 replay: hits still outnumber passes there)


def run_ledge(brain_factory, seconds=900., seed=0, sensor_errors=None):
    """Map ledge_world(). PASS: mapping complete, the robot stopped on its own within `seconds` (less than the brain's time
    budget); every outline cell the map holds as occupied from a hit within LEDGE_CLOSE_M (grid.hit_range) stays occupied
    for the rest of mapping ('lost': those that did not; before task K far beams passing under the ledge or skimming past its
    ends erased them) and at least one was (else 'not exercised'); at the end no floor under the ledge is reachable by the
    planner (no target the robot can never reach); clean reverse audit, no rigid-body contact (the turret never met the
    edge), no bumper touch. Also reported: the end state of the outline and of the floor under it ('enterable_m2' counts
    toward mapping_complete's 1.5 m2 when an end was never seen from close by and is mapped narrowly open) and 'timeline':
    (t, open outline cells, reachable m2, enterable m2) every 25 s. seed and sensor_errors: as Sim."""
    brain = brain_factory(); sim = Sim(ledge_world(), seed=seed, sensor_errors=sensor_errors)
    bridge = Bridge(brain, sim, rearm_hook=lambda b: getattr(b, 'wants_arm', False))
    for _ in range(10):
        brain.predict(sim.state()); [sim.step(.01) for _ in range(5)]
    timeline = []; nxt = [25.]; close = {}; lost = {}; near = {}; near_lost = {}

    def tick(br):
        if brain.mode != 'mapping':
            return
        g = brain.grid
        for c in ledge_outline(g):
            for seen, gone, limit in ((close, lost, LEDGE_CLOSE_M), (near, near_lost, LEDGE_NEAR_M)):
                if c not in seen and g.logodds[c] > .8 and g.hit_range[c] <= limit:
                    seen[c] = round(sim.t, 1)
                elif c in seen and c not in gone and g.logodds[c] <= .8:
                    gone[c] = round(sim.t, 1)
        if sim.t >= nxt[0]:
            nxt[0] += 25.; f = ledge_floor(g, brain.localizer.pose); timeline.append((round(sim.t), f['outline_open'], f['reachable_m2'], f['enterable_m2']))
    bridge.start_mapping(); trace = bridge.run(seconds, on_tick=tick)
    floor = ledge_floor(brain.grid, brain.localizer.pose)
    audit = reverse_audit(trace); clean = audit['ok'] and sim.contacts == 0 and sim.bump_events == 0
    ok = clean and brain.mode == 'idle' and brain.complete and len(close) > 0 and not lost and floor['reachable_m2'] == 0
    return dict(passed=bool(ok), t=round(sim.t), complete=bool(brain.complete), close_seen_cells=len(close), first_close_s=min(close.values(), default=None),
                lost=len(lost), lost_cells=sorted((t, [round(float(v), 3) for v in brain.grid.world(c)]) for c, t in lost.items())[:6],
                near_seen_cells=len(near), near_lost=len(near_lost),
                verdict=None if close else 'not exercised: no outline cell seen from close by', **floor, unvisited_m2=round(brain.unvisited_area(), 3),
                contacts=len(brain.contacts), skips=len(brain.blacklist), body_blocks=sim.contacts, turret_hits=sim.overhang_hits,
                bumps=sim.bump_events, reverse_audit=audit, message=brain.message, timeline=timeline, events=[e['message'] for e in brain.events][-4:])


# ---- closed-loop scenario: bumper contact with no straight trail behind (task G) ---------------------------------
# Scripted approaches, (v, w, seconds) each: the robot drives them under its own command() guard, then a low step
# (6 cm shower-tray edge: bumper only, LiDAR-blind) appears `gap` ahead of its skin and it drives straight on until it
# touches. 'weave': the last 4 s of the Unreal trial full-mapping-20261011T054731 (the local planner's +/-.4 rad/s
# commits, 10 cm/s) before it met the shower tray; 'spin': 30 cm straight, then a quarter turn in place; 'spin_start':
# a quarter turn in place before anything else (the trail behind is only the start position).
CONTACT_APPROACHES = {'weave': [(.04, .4, .65), (.1, -.4, .65), (.1, -.4, .65), (.04, -.4, .65), (.04, 0., .65), (.1, .4, .65)],
                      'spin': [(.1, 0., 3.), (0., .4, math.pi / 2 / .4)],
                      'spin_start': [(0., .4, math.pi / 2 / .4)],
                      # task P: Unreal trial full-mapping-20261012T183238's last 2 s before the shower tray: straight at 10 cm/s, then the
                      # local planner's -.4 rad/s commit for .5 s (11 deg), so the trail behind the touch curves away from the heading.
                      'turn': [(.1, 0., 2.), (.1, -.4, .5)],
                      # task P: Unreal trial full-mapping-20261012T220759 (task N) at 585-587 s: weaving, the local planner's reverse arc
                      # (-.04 m/s, .4 rad/s), then forward on an arc into the touch: the drive after the reverse was not on the trail.
                      'reverse_arc': [(.1, 0., 2.), (.1, .4, .6), (-.04, .4, .6), (.1, .4, .5)]}


def run_contact_recovery(brain_factory, approach='weave', gap=.02, seconds=60., seed=0, sensor_errors=None, release_s=15., leave_m=.3,
                         press_roll_m=0.):
    """Mapping start, then the CONTACT_APPROACHES[approach] script; then a wide low step appears `gap` ahead of the skin
    and the robot drives straight (10 cm/s) until its bumper touches the step. From the touch on the real controller
    decides everything. PASS: the bumper releases within release_s of the touch; by the end the robot has left the contact
    phase and is at least leave_m from where it touched (it got off the step and went on mapping); zero rigid-body
    blocks; the reverse audit clean (no reverse off the TRUE driven trail, episodes within REVERSE_LIMIT); the touch is
    on the map (a contact recorded, contact cells within 15 cm of the touched skin). Before task G the robot backed off
    one tick or not at all and then waited for the bumper to release until the time budget ran out. press_roll_m: as Sim (task P:
    the body rolls on into the skin after the touch, past where the back-off's first reverse tick began)."""
    brain = brain_factory(); sim = Sim(World([((-1, -1.5), (3, -1.5)), ((3, -1.5), (3, 1.5)), ((3, 1.5), (-1, 1.5)), ((-1, 1.5), (-1, -1.5)),
                                              ((.5, -1.1), (.5, -.8)), ((2.3, -.9), (2.3, -.6))]), seed=seed, sensor_errors=sensor_errors,
                                       press_roll_m=press_roll_m)
    bridge = Bridge(brain, sim, rearm_hook=lambda b: getattr(b, 'wants_arm', False))
    for _ in range(10):
        brain.predict(sim.state()); [sim.step(.01) for _ in range(5)]
    script = CONTACT_APPROACHES[approach]; real = brain.drive_command; st = dict(t0=None, step=None, touched=None, touch_pose=None, released=None)

    def scripted(state):
        # The approach replaces only the motion choice; command() (the firmware and reverse guards) still applies.
        if st['touched'] is not None or brain.phase == 'contact' or brain.phase not in ('plan', 'follow') or not brain.imu.ready_to_move():
            return real(state)
        t = state['sim_time_s']; st['t0'] = t if st['t0'] is None else st['t0']; k = t - st['t0']; brain.phase = 'follow'
        for v, w, d in script:
            if k < d:
                return v, w
            k -= d
        if st['step'] is None:
            # The step: 1.2 m wide, .5 m deep, its face `gap` ahead of the skin, square to the heading (TRUE pose).
            p = sim.pose; u = np.array([math.cos(p[2]), math.sin(p[2])]); n = np.array([-u[1], u[0]]); c = p[:2] + (SKIN_R + gap) * u
            st['step'] = np.array([c - .6 * n, c + .6 * n, c + .6 * n + .5 * u, c - .6 * n + .5 * u])
            sim.w.low.append(st['step']); sim.w.low_segs = np.array([s for q in sim.w.low for s in poly_segments(q)], float).reshape(-1, 2, 2)
        return .1, 0.

    brain.drive_command = scripted

    def tick(br):
        if sim.bump and st['touched'] is None:
            st['touched'] = sim.t; st['touch_pose'] = sim.pose.copy()
        if st['touched'] is not None and st['released'] is None and not sim.bump:
            st['released'] = sim.t
    bridge.start_mapping(); bridge.run(seconds, on_tick=tick)
    audit = reverse_audit(bridge.trace)
    touched = st['touched'] is not None
    after = [r for r in bridge.trace if touched and r['t'] >= st['touched']]
    away = float(np.linalg.norm(sim.pose[:2] - st['touch_pose'][:2])) if touched else 0.
    released = None if st['released'] is None else round(st['released'] - st['touched'], 2)
    skin = st['touch_pose'][:2] + SKIN_R * np.array([math.cos(st['touch_pose'][2]), math.sin(st['touch_pose'][2])]) if touched else None
    cells = np.argwhere(brain.grid.camera_obstacles) if touched else np.empty((0, 2))
    marked = bool(len(brain.contacts)) and bool(len(cells)) and min(float(np.linalg.norm(brain.grid.world(c) - skin)) for c in cells) <= .15
    reversed_m = sum(float(np.linalg.norm(np.diff(r['path'], axis=0), axis=1).sum()) for r in after if r['v'] < 0 and 'path' in r)
    ok = (touched and released is not None and released <= release_s and brain.phase != 'contact' and away >= leave_m and sim.contacts == 0
          and audit['ok'] and marked)
    return dict(passed=bool(ok), approach=approach, gap_m=gap, seed=seed, touched_s=None if not touched else round(st['touched'], 2), released_after_s=released,
                reversed_after_touch_m=round(reversed_m, 4), away_from_touch_m=round(away, 3), contact_marked=marked, contacts=len(brain.contacts),
                bumps=sim.bump_events, body_blocks=sim.contacts, reverse_audit=audit, mode=brain.mode, phase=brain.phase, message=brain.message,
                events=[e['message'] for e in brain.events][2:8])


def run_retreat_after_reverse(brain_factory, seconds=20., seed=0, sensor_errors=None, back_m=.015, forward_s=.55):
    """Task P: Unreal trial full-mapping-20261012T184036 under the dining table (GI on). Mapping start, then a scripted drive: 30 cm
    straight at 10 cm/s, a stop, a short reverse (4 cm/s for back_m), a stop, then forward on the local planner's arc (5 cm/s, -.4 rad/s) for
    forward_s (~2.7 cm: past where the reverse began, less than the 5 cm that closes a reverse episode); then it stands still and
    the stall retreat starts (LidarBrain.begin_retreat, as after 6 s without progress). From then on the real controller decides.
    Before task P the open episode had fixed the newest end of the trail where the reverse began, base_link stood ~1 cm past it,
    and command()'s guard refused the planned retreat (in Unreal 16 times, 126 s). PASS: the retreat starts and base_link (truth)
    moves back at least 2.5 cm, no 'Reverse refused' event after it began, the reverse audit clean, 0 body blocks, mapping on."""
    brain = brain_factory(); sim = Sim(World([((-1, -1.5), (3, -1.5)), ((3, -1.5), (3, 1.5)), ((3, 1.5), (-1, 1.5)), ((-1, 1.5), (-1, -1.5)),
                                              ((.5, -1.1), (.5, -.8)), ((2.3, -.9), (2.3, -.6))]), seed=seed, sensor_errors=sensor_errors)
    bridge = Bridge(brain, sim, rearm_hook=lambda b: getattr(b, 'wants_arm', False))
    for _ in range(10):
        brain.predict(sim.state()); [sim.step(.01) for _ in range(5)]
    script = [(.1, 0., 3.), (0., 0., .6), (-.04, 0., back_m / .04), (0., 0., .5), (.05, -.4, forward_s), (0., 0., .5)]
    real = brain.drive_command; st = dict(t0=None, retreat=None, began=False, pose=None, events=None)

    def scripted(state):
        if st['retreat'] is not None or brain.phase not in ('plan', 'follow') or not brain.imu.ready_to_move():
            return real(state)
        t = state['sim_time_s']; st['t0'] = t if st['t0'] is None else st['t0']; k = t - st['t0']; brain.phase = 'follow'
        for v, w, d in script:
            if k < d:
                return v, w
            k -= d
        st['retreat'] = t; st['pose'] = sim.pose.copy(); st['events'] = len(brain.events); st['began'] = brain.begin_retreat(t)
        if not st['began']:
            brain.phase = 'plan'; brain.plan_not_before = t
        return real(state)

    brain.drive_command = scripted
    bridge.start_mapping(); bridge.run(seconds)
    audit = reverse_audit(bridge.trace); back = 0.; refused = 0
    if st['pose'] is not None:
        u = np.array([math.cos(st['pose'][2]), math.sin(st['pose'][2])])
        back = max([0.] + [float(-(r['truth'][:2] - st['pose'][:2]) @ u) for r in bridge.trace if r['t'] >= st['retreat'] and r['t'] <= st['retreat'] + 3.])
        refused = sum('Reverse refused' in e['message'] for e in brain.events[st['events']:])
    ok = bool(st['began'] and back >= .025 and refused == 0 and audit['ok'] and sim.contacts == 0 and brain.mode != 'idle')
    return dict(passed=ok, seed=seed, retreat_began=bool(st['began']), moved_back_m=round(back, 4), refused=refused, body_blocks=sim.contacts,
                reverse_audit=audit, mode=brain.mode, phase=brain.phase, message=brain.message,
                events=[e['message'] for e in brain.events[(st['events'] or 0):][:6]])
