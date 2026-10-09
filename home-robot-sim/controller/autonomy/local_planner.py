"""Short motion-sequence search using measured occupancy and current returns.

The drive axle passes through base_link (geometry.AXLE_X = 0 on the round
robot): a turn in place rotates the round body about its own centre, sweeps no
new floor and changes no distance to anything. Driving keeps DRIVE_CLEARANCE
from every measured point; a pure spin is decided by the circle rule alone
(turn_clear). The first command is applied and the search repeats with fresh
measurements. No reference house geometry, object labels, or simulator poses
are available to this module.
"""
import math
import cv2
import numpy as np
from .geometry import R, AXLE_X, TURN_CLEAR, DRIVE_CLEARANCE, BODY_CLEAR, PLAN_RADIUS

NEARBY = .30   # 'a narrow passage is actually nearby' (planner speed cap); not body geometry


def turn_clear(cloud, angles=None, margin=None):
    """Exact circle rule for a turn in place. The body is a disc of radius R
    about base_link and the drive axle passes through base_link, so a pure spin
    changes no distance: it is allowed, for ANY angle, iff every given point is
    at least R + TURN_MARGIN (TURN_CLEAR) from base_link. No point is smoothed,
    merged or dropped here: LiDAR range noise is removed beforehand by averaging
    each beam over the scans taken from the same spot (LidarBrain.spin_points),
    which is unbiased for any surface shape; camera and contact cells are used
    raw. TURN_MARGIN covers base_link drift during the spin and the residual
    range error. With a non-zero AXLE_X the body centre circles the axle at
    |AXLE_X|, so the rule is applied about the axle with |AXLE_X| added.
    `angles` is accepted for old callers and ignored."""
    cloud = np.asarray(cloud, float).reshape(-1, 2)
    if not len(cloud):
        return True
    limit = (TURN_CLEAR if margin is None else R + margin) + abs(AXLE_X)
    return bool(np.hypot(cloud[:, 0] - AXLE_X, cloud[:, 1]).min() >= limit)


def choose_velocity(brain, target, distance):
    pose = brain.localizer.pose
    cloud = brain.obstacle_points()
    # For the circle rule: beam-averaged LiDAR points plus the measured camera /
    # contact cells without the planning margin.
    # spin_rule adds the mid-spin hysteresis (LidarBrain.spin_rule); it applies only where
    # base_link is now, i.e. to a spin chosen as the first command.
    core, spin_margin = brain.spin_rule() if hasattr(brain, 'spin_rule') else (cloud, None)
    grid = brain.grid
    if len(cloud) < 60:
        return 0., 0.
    # Safe cells, plus a no-closer exit if a new margin surrounds the robot.
    safe = brain.motion_mask() if hasattr(brain, 'motion_mask') else grid.masks()[3]
    # A pure spin is judged by the circle rule (turn_clear) on measured points
    # and only needs the measured free floor the body already stands on.
    free = grid.masks()[0]
    route = np.asarray(brain.path)
    if len(route) < 2:
        route = np.array([pose[:2], target])
    segments = np.diff(route, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    remaining = np.r_[np.cumsum(lengths[::-1])[::-1], 0.]

    # Fine local distance field. Subtract a full cell diagonal so quantization
    # cannot overstate clearance (field <= true distance <= field + 2 diagonals).
    # Points outside this 4 m square cannot affect a six-second trajectory at <= 0.1 m/s.
    size, resolution, middle = 401, .01, 200
    empty = np.ones((size, size), np.uint8)
    indices = np.floor(cloud / resolution + middle).astype(int)
    inside = (indices >= 0).all(axis=1) & (indices < size).all(axis=1)
    empty[indices[inside, 1], indices[inside, 0]] = 0
    clearance = cv2.distanceTransform(empty, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * resolution - .0142
    # Normally DRIVE_CLEARANCE (20 cm). After a bump the robot may already be
    # closer; it may then move only along sweeps that never come closer than it is now.
    # Both are true base_link-to-point distances: a sweep the quantized field cannot
    # decide (within two cell diagonals of the limit) is checked against the points
    # themselves, so a passage only a few mm wider than 2 x DRIVE_CLEARANCE, which
    # the narrow-passage planner routes through, stays drivable.
    distances = np.linalg.norm(cloud, axis=1)
    limit = min(DRIVE_CLEARANCE, float(distances.min()) - .003)
    nearby_points = cloud[distances < 1.]
    undecided = .0284

    c, s = math.cos(pose[2]), math.sin(pose[2])
    rotation = np.array([[c, s], [-s, c]])

    def path_cost(candidate):
        world = candidate[:2] @ rotation + pose[:2]
        fraction = np.clip(np.sum((world - route[:-1]) * segments, axis=1) /
                           np.maximum(1e-9, lengths ** 2), 0, 1)
        projected = route[:-1] + fraction[:, None] * segments
        lateral = np.linalg.norm(projected - world, axis=1)
        nearest = int(np.argmin(lateral))
        direction = route[min(len(route) - 1, nearest + 5)] - projected[nearest]
        angle = math.atan2(direction[1], direction[0]) - (pose[2] + candidate[2])
        heading_error = abs(math.atan2(math.sin(angle), math.cos(angle)))
        left = remaining[nearest] - fraction[nearest] * lengths[nearest]
        # No required final orientation. Reduce heading cost near the goal.
        return left + 2 * lateral[nearest] + .08 * min(1., left / .2) * heading_error

    duration = 1.
    times = np.linspace(0, duration, 16)
    maximum = min(.10, distance * .45)
    # A narrow passage actually nearby: slowly. The narrow-passage clearance
    # alone does not slow open-floor driving.
    if grid.radius < PLAN_RADIUS and np.linalg.norm(cloud, axis=1).min() < NEARBY:
        maximum = min(maximum, .06)
    if brain.camera.status['state'] == 'insufficient features':
        maximum = min(maximum, .04)
    controls = [(v, w, brain.predicted_centres(v, w, times))
                for v in sorted(set([0., min(.05, maximum), maximum, -.04]))
                for w in [-.4, 0., .4]]
    # A reverse episode may cover at most REVERSE_LIMIT in total (LidarBrain.command enforces it).
    reverse_budget = brain.reverse_budget() if hasattr(brain, 'reverse_budget') else math.inf
    beam = [(path_cost(np.zeros(3)), np.zeros(3), None, 0.)]
    best = None
    for depth in range(6):
        choices = {}
        for _, current, first, running_cost in beam:
            c, s = math.cos(current[2]), math.sin(current[2])
            local_rotation = np.array([[c, s], [-s, c]])
            for v, w, centres in controls:
                xy = centres @ local_rotation + current[:2]
                local_cells = np.floor(xy / resolution + middle).astype(int)
                if np.any(local_cells < 0) or np.any(local_cells >= size):
                    continue
                separation = clearance[local_cells[:, 1], local_cells[:, 0]]
                spin = v == 0 and w != 0
                if spin:
                    # Circle rule: allowed iff every measured point is >= TURN_CLEAR
                    # from base_link, however close the 20 cm driving circle is.
                    c, s = math.cos(current[2]), math.sin(current[2])
                    if not turn_clear((core - current[:2]) @ np.array([[c, -s], [s, c]]),
                                      margin=spin_margin if first is None else None):
                        continue
                elif separation.min() < limit:
                    if separation.min() + undecided < limit:
                        continue
                    exact = np.linalg.norm(xy[:, None, :] - nearby_points[None, :, :], axis=2).min()
                    if exact < limit:
                        continue
                world = xy @ rotation + pose[:2]
                cells = np.floor((world - grid.origin) / grid.res).astype(int)[:, ::-1]
                if (np.any(cells < 0) or np.any(cells >= grid.n) or
                        not brain.cells_clear(free if spin else safe, cells)):
                    continue
                # A rear-facing camera is not available: reverse only along the
                # trail the chassis has just driven (within 2.5 cm).
                if v < 0 and (not brain.on_trail(world) or (first is None and abs(v) * duration > reverse_budget)):
                    continue
                endpoint = np.r_[xy[-1], current[2] + w * duration]
                first_command = first if first is not None else (v, w)
                # Cost throughout the sequence prevents 'wait, then move'
                # winning again on every update and postponing motion forever.
                cost = path_cost(endpoint)
                accumulated = running_cost + .06 * cost + .003 * abs(w) + (.003 if v < 0 else 0)
                score = cost + accumulated + .003 / max(.02, separation[-1] - BODY_CLEAR)
                key = tuple(np.rint(endpoint / [.02, .02, .15]).astype(int))
                if key not in choices or score < choices[key][0]:
                    choices[key] = (score, endpoint, first_command, accumulated)
        if not choices:
            break
        beam = sorted(choices.values(), key=lambda item: item[0])[:24]
        best = beam[0]
    return best[2] if best is not None else (0., 0.)
