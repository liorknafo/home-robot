"""Robot geometry: the single source of truth for every controller module.

Loaded once from Content/RobotData/robot-parameters.json (synced from the CAD).
Robot frame: base_link at the centre of the round body, x forward, y left,
metres. The drive axle passes through base_link (AXLE_X = 0), so a turn in
place rotates the body about its own centre and changes no distance to
anything around it.

Only keys that OLD (box-robot) parameter files lack have fallbacks, and each
fallback is the old robot's value or a conservative bound, never a guess that
makes the robot look smaller than it is. The round-robot keys are written
explicitly in robot-parameters.json.

Derived clearances (all distances from base_link; see derive()):
  DRIVE_CLEARANCE  translation clearance: R + ~4 cm (localisation .025 + tracking
                   .015), rounded up to 5 mm -> .20 for R = .1595.
  BODY_CLEAR       "clear of the body" for already-driven cells and margin exits:
                   R + 8 mm, rounded up to 5 mm -> .17.
  TURN_MARGIN      pure-spin margin, 1 cm = base_link drift during a turn-around
                   (wheel slip; run_physics bounds it at 1 cm per full turn, so
                   <= 5 mm per half turn) + 5 mm LiDAR range error left after the
                   per-beam TEMPORAL average of the scans taken while base_link
                   stands still (lidar_brain.spin_points: 4 mm beam jitter / sqrt(8)
                   = 1.4 mm, so 5 mm is > 3.5 sigma; the beam matching adds < .6 mm).
                   No spatial smoothing is applied, so the closest point of a
                   corner or post is never averaged outward.
  TURN_CLEAR       R + TURN_MARGIN. A spin about base_link is allowed iff every
                   measured point is at least this far from base_link.
  CLOSE_SPIN_CLEAR R + CLOSE_SPIN_MARGIN (3 mm). 'Close spin': when TURN_CLEAR refuses a spin
                   but nothing touches, a SLOW spin (lidar_brain.CLOSE_SPIN_W) is allowed when
                   every beam-averaged point is at least this far from base_link (front-half
                   points may instead be verified by the released bumper skin, see
                   lidar_brain.close_spin_clear). A pure spin changes no distance, so this is
                   the replacement for reversing blind: the robot never reverses off its trail.
  PLAN_RADIUS      A* inflation: R + localisation .025 + half a 5 cm cell diagonal
                   .0354 + tracking .025 = R + .0854 <= .25.
  DRIVE_CLEARANCE_LIDAR  translation clearance against the LIVE LiDAR points only: R + tracking/
                   braking .015 + LiDAR range error .01 (single scan, 4 mm sigma), rounded up to 5 mm
                   -> .185 for R = .1595. The points are measured in the robot frame, so localisation
                   error does not apply to them (DRIVE_CLEARANCE's .025 share). Used only by the slow
                   final approach to a Go-to target that lies closer to an obstacle than the planner's
                   safe mask (lidar_brain.approach_command); camera / contact cells are map-frame and
                   keep DRIVE_CLEARANCE there. TURN_CLEAR < DRIVE_CLEARANCE_LIDAR < DRIVE_CLEARANCE.
  GOAL_CLEAR       R - 2 cm: a Go-to target outside the safe mask is accepted when its cell-centre
                   clearance to occupied (and to unmapped) cells is at least this. Below R because the
                   distance between 5 cm cell centres differs from the true target-to-surface distance by up
                   to a cell: a click 2 cm from a wall (true R + 2 cm) is 3 or 4 cells (.15 / .20) from the
                   wall's occupied cell depending on where the wall sits in its cell, and both are accepted;
                   2 cells (.10, a body overlapping the wall) are not. The live approach decides how close the
                   robot really gets (and reports 'Stopped N cm short' when the body does not fit).
                   R - one 5 cm cell < GOAL_CLEAR < PLAN_RADIUS.
  FIT_CLEAR        R - 5 cm (one cell of quantization, no localisation or tracking margin), at most 11.1 cm: the round body
                   may fit at a cell whose centre is at least this far from every occupied cell centre. On the 5 cm grid it
                   must lie between the cell-centre distances 2 cells (10 cm) and sqrt(5) cells (11.18 cm), where every value
                   gives the same result (11.1 cm caps it there for a larger body, e.g. the old box robot's
                   parameter file). For a body wider than 30.4 cm (asserted): between two obstacles whose cells lie 5
                   apart on a row (a gap of at most hypot(30, 5) = 30.4 cm; the body is 2R = 31.8 cm wide) no cell is this
                   clear; 6 apart (up to 35 cm) the middle ones are; and at any angle a gap that could be 2R or wider is
                   never called closed (task H: 6650 random doorways, the widest called closed at most 30.4 cm). R - 2 cm
                   (GOAL_CLEAR's allowance) is exact only along a row: it called oblique gaps of up to 33.5 cm closed.
                   Mapping completion (grid.unreached_enterable) leaves out only floor that no path of such cells joins to
                   the robot: walled in for the body itself, not merely cut off by the planner's margins.
"""
import json
import math
from pathlib import Path

PARAMETERS_PATH = Path(__file__).resolve().parents[2] / 'Content/RobotData/robot-parameters.json'


def _load(path=PARAMETERS_PATH):
    try:
        return json.loads(Path(path).read_text())
    except FileNotFoundError:
        return {}


PARAMETERS = _load()
_P = PARAMETERS


def _ceil5mm(x):
    return math.ceil(round(x / .005, 6)) * .005


# ---- measured / CAD geometry ------------------------------------------------
# Old (box robot) parameter files have no body radius: .16205 m was the box robot's
# whole CAD envelope about base_link (lidar.py derivation), a safe upper bound. A
# missing drive_axle_x_m (no parameter file at all) falls back to the box robot's
# .042: turn_clear then checks TURN_CLEAR + .042 about a point .042 ahead, which
# contains the round body's TURN_CLEAR disc too, so it is conservative for both.
R = float(_P.get('body_radius_m', .16205))              # contact skin (bumper) radius
R_RIGID = float(_P.get('rigid_radius_m', R))             # rigid rear shell radius (<= R)
AXLE_X = float(_P.get('drive_axle_x_m', .042))           # drive axle ahead of base_link (0 on the round robot)
TRACK = float(_P.get('wheel_track_m', .228))
WHEEL_R = float(_P.get('wheel_radius_m', .04))
CPR = float(_P.get('encoder_cpr', 4741.44))
BUMPER_SPAN_DEG = float(_P.get('bumper_span_deg', 90))   # skin covers azimuth -span..+span about base_link
# bump_zone_edges_deg: the sync writes the full ascending list (-90,-55,-20,20,55,90); older files held only the
# positive half (20,55,90). The zones are symmetric, so keep the positive half either way.
BUMP_ZONE_EDGES_DEG = sorted(abs(float(a)) for a in _P.get('bump_zone_edges_deg', [20, 55, 90]) if float(a) > 0)
REAR_CASTER_XY = tuple(float(v) for v in _P.get('rear_caster_xy_m', [_P.get('caster_x_m', -.098), 0.]))
# Cliff ToF sensors in native (emission) order FL, FR, RL, RR; xy about base_link.
CLIFF_XY = tuple(tuple(float(v) for v in xy) for xy in
                 _P.get('cliff_sensors_m', [[.105, .072], [.105, -.072], [-.108, .072], [-.108, -.072]]))
CLIFF_Z = float(_P.get('cliff_z_m', .0425))              # sensor origin height above the floor (= nominal range)
CAMERA_MOUNT = tuple(float(v) for v in _P.get('camera_mount_m', [.117, 0., .15]))   # x, y, height above floor
LIDAR_MOUNT = tuple(float(v) for v in _P.get('lidar_mount_m', [.02, 0., .19]))      # x, y, scan-plane height
BASE_Z = float(_P.get('base_link_height_m', _P.get('base_link_z_m', .085)))            # nominal base_link height above the floor (RobotPawn BodyHeight)

# Bumper zones about base_link, bit 0 right .. bit 4 left: edges +/-20, +/-55, +/-90 give
# [(-90,-55), (-55,-20), (-20,20), (20,55), (55,90)].
_E = BUMP_ZONE_EDGES_DEG
BUMP_ZONES = [(-_E[2], -_E[1]), (-_E[1], -_E[0]), (-_E[0], _E[0]), (_E[0], _E[1]), (_E[1], _E[2])]


def shell_point(azimuth_deg):
    """Point on the contact skin at an azimuth about base_link, and the outward normal."""
    a = math.radians(azimuth_deg)
    d = (math.cos(a), math.sin(a))
    return (R * d[0], R * d[1]), d


# ---- derived clearances (metres from base_link) --------------------------------
DRIVE_MARGIN = .04                       # localisation .025 + tracking/braking .015
BODY_MARGIN = .008
SPIN_DRIFT = .005                        # base_link drift per half turn in place (wheel slip)
SPIN_RANGE_ERROR = .005                  # residual range error after the per-beam temporal average
TURN_MARGIN = SPIN_DRIFT + SPIN_RANGE_ERROR
CLOSE_SPIN_MARGIN = .003                 # close spin: beam-averaged points >= R + 3 mm (not touching)
PLAN_MARGIN = .025 + .05 / math.sqrt(2) + .025   # localisation + half cell diagonal + tracking = .0854
PLAN_RADIUS = .25
TRACK_MARGIN = .015                      # tracking/braking (the non-localisation share of DRIVE_MARGIN)
LIDAR_RANGE_ERROR = .01                  # single-scan LiDAR range error bound (4 mm sigma, 2.5 sigma)
DRIVE_LIDAR_MARGIN = TRACK_MARGIN + LIDAR_RANGE_ERROR   # live LiDAR points: no localisation share
GOAL_MARGIN = -.02                       # near-obstacle Go-to target: cell-centre clearance >= R - 2 cm (cell quantization)


def derive(radius, rigid_radius=None):
    """Clearances for a round body of contact-skin radius `radius`; raises if inconsistent."""
    rigid_radius = radius if rigid_radius is None else rigid_radius
    out = dict(DRIVE_CLEARANCE=_ceil5mm(radius + DRIVE_MARGIN), BODY_CLEAR=_ceil5mm(radius + BODY_MARGIN),
               TURN_CLEAR=radius + TURN_MARGIN)
    # Only geometric invariants: the spin and driving circles contain the body, driving keeps
    # more room than a spin, the planner inflation is the largest and fits the body + errors.
    assert rigid_radius <= radius + 1e-9, 'the rigid shell must lie inside the contact skin'
    assert radius < out['TURN_CLEAR'] and radius < out['BODY_CLEAR'], 'clearances must contain the body'
    assert out['TURN_CLEAR'] < out['DRIVE_CLEARANCE'] <= PLAN_RADIUS, 'driving needs more room than a spin, less than planning'
    assert radius + PLAN_MARGIN <= PLAN_RADIUS, f'body radius {radius} m does not fit the {PLAN_RADIUS} m planning radius'
    return out


_D = derive(R, R_RIGID)
DRIVE_CLEARANCE = _D['DRIVE_CLEARANCE']
BODY_CLEAR = _D['BODY_CLEAR']
TURN_CLEAR = _D['TURN_CLEAR']
CLOSE_SPIN_CLEAR = R + CLOSE_SPIN_MARGIN
assert R < CLOSE_SPIN_CLEAR < TURN_CLEAR, 'a close spin is the band between the skin and TURN_CLEAR'
DRIVE_CLEARANCE_LIDAR = _ceil5mm(R + DRIVE_LIDAR_MARGIN)
assert TURN_CLEAR < DRIVE_CLEARANCE_LIDAR < DRIVE_CLEARANCE, 'live-LiDAR driving needs more room than a spin, less than map-frame driving'
GOAL_CLEAR = R + GOAL_MARGIN
assert R - .05 < GOAL_CLEAR < PLAN_RADIUS, 'a near-obstacle Go-to target may undercut R by less than one 5 cm cell (quantization) and lies inside the planning margin'
FIT_CLEAR = min(R - .05, .111)           # the body may fit here (cell quantization only, no error margin), see the docstring
assert .10 < FIT_CLEAR < .05 * math.sqrt(5) and 2 * R > math.hypot(.30, .05), 'FIT_CLEAR: re-derive it for this body radius on the 5 cm grid (docstring)'
NARROW_RADIUS = DRIVE_CLEARANCE          # narrow-passage planning clearance (exact for a round body)
# Intentional literals left in controller/autonomy that look like robot geometry but are not
# (test_whole_house_pacing keeps an allow-list of the .20 ones):
#   lidar_brain: REVERSE_LIMIT .15 (per reverse episode), driven-trail segment gap .05 (the visit path is
#     never used for reversing), trail tolerance .025 (planned straight reverses .015), trail-end slack
#     .005, bend slack .01, reverse settle .5 s, episode reset .05 m of forward travel, contact
#     back-off cap .12, retreat lengths .035/.15/.08/.30, mark depths .05/.08 outside the skin (> half a
#     5 cm cell diagonal), unexplained-stall back-off STALL_BACKOFF .05 / mark spacing STALL_MARK_SPACING .10 / jam settle
#     JAM_SETTLE_S 1 s, cliff range gate .02-.08, tilt limit .20 rad, spin-averaging gates
#     SPIN_STILL_M .003 / SPIN_STILL_YAW .003 rad / SPIN_NEAR .30, mid-spin hysteresis SPIN_RAW_ALLOWANCE .012
#     (3 sigma of single-scan range noise, not body geometry), close spin CLOSE_SPIN_W .25 rad/s, its
#     front-half LiDAR slack CLOSE_SPIN_FRONT_SLACK .005 (3 sigma of the beam-averaged range noise), its
#     raw-contact count CLOSE_SPIN_CONTACT_POINTS 2, skin-end entry CLOSE_SPIN_ENTRY_DEG 20, exit run
#     CLOSE_SPIN_EXIT .15 m at CLOSE_SPIN_EXIT_V .06 m/s, close-spin stall .8 s below 30 % of the command,
#     retry memory .03 m / 60 s, the reverse guard look-ahead .02 m, passage_width
#     window .10/.25 (where a narrow stall is measured), the near-obstacle Go-to approach's APPROACH_* speeds,
#     sampling, tolerances and timeouts (.05 m/s cap, .01 m/s stop, .3 s stopping time, 1 mm sampling, 5 mm
#     reached, .03 m arrived, .03/.25 rad heading, .4/.1 rad/s turn, 1 rad/m curvature cap, .05 m straight
#     finish, .5 m look-ahead, 1 m staging window, 60 s) and its on-approach-line band .05. There is NO blind (off-trail) reverse;
#   local_planner: NEARBY .30 (narrow-passage-nearby speed cap), 1 cm distance field, .0142 cell
#     diagonal and .0284 (two diagonals: the band the quantized field cannot decide);
#   grid.py: legacy stereo Grid planning radius PLAN_RADIUS + .01 (.26, its original margin);
#   lidar.py / localization.py: scan-match and keyframe gates .20 m / .20 rad, inlier .065;
#   grid.py, brain.py, stereo.py, mono.py (legacy stereo / camera): stereo near blind zone .20, stopping
#     distance .12 + deceleration .20 m/s^2, recovery yaw offsets .20/.40 rad, height bands
#     .035-.27 / .03-.24, stereo CAMERA_OFFSET; floor_parallax hmin .03.
