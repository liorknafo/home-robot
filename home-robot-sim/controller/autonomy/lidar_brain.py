"""Autonomous exploration and navigation using 2D LiDAR, wheels and gyro.
The camera adds confirmed sparse low obstacles and checks motion consistency.
Bumps and wheel jams are recoverable observations: the contact is recorded on
the map, the robot reverses briefly along the path it has driven forward (the
camera cannot see behind it) and replans. The robot NEVER reverses off its own
driven trail (no rear bumper, rear camera or low-object sensing): the trail
holds only positions reached driving forward or turning, a reverse may not run
past a trail end, one reverse episode covers at most REVERSE_LIMIT, and a final
guard in command() drops any reverse that breaks these rules. While the bumper
is pressed command() also sends only what the motor core executes as commanded
(the repo's DriveCore.h clamps both wheels to reverse while pressed, so a spin
there would pivot base_link backward: no spin at all while pressed unless
PRESSED_SPIN_PURE). Where a turn in place is refused only by the TURN_CLEAR
margin it turns slowly in place instead ('close spin'). Pressed with no driven
path behind and no allowed spin, it stands still and waits for the bumper to
release (the session stays alive). A
target that cannot be reached is skipped, not the session. Only cliff, tilt,
sensor loss, a user stop, lost localization, repeated contacts in one area, the
time budget or an exhausted recovery budget end a mapping run. A Go-to target
closer to an obstacle than the planner's safe mask (but with room for the body)
is reached by the normal route to the nearest safe cell, then a slow, forward-only
final approach checked every tick against the live LiDAR (APPROACH_*).
"""
import json,math,time,heapq,cv2,numpy as np
from pathlib import Path
from collections import deque
from types import SimpleNamespace
from .brain import Brain
from .lidar import LidarLocalizer,LidarGrid,scan_points,transform,relocalization_failure_message
from .stereo import wrap
from .mono import MonoObserver
from .floor_parallax import LowObstacleCamera
from .imu import ImuCalibration,GRAVITY_GATE
from .lidar import inverse
from .local_planner import choose_velocity,turn_clear,NEARBY
from .geometry import (AXLE_X,LIDAR_MOUNT,PLAN_RADIUS,TRACK as WHEEL_TRACK,WHEEL_R,CLIFF_XY,CLIFF_Z,BASE_Z,BUMP_ZONES,shell_point as _shell_point,
                       DRIVE_CLEARANCE,BODY_CLEAR,TURN_CLEAR,TURN_MARGIN,SPIN_DRIFT,NARROW_RADIUS,R,R_RIGID,CLOSE_SPIN_CLEAR,BUMPER_SPAN_DEG,
                       DRIVE_CLEARANCE_LIDAR,GOAL_CLEAR)
SQRT2=math.sqrt(2)
REVERSE_LIMIT=.15   # metres; reversing only ever retraces the driven path
# NARROW_RADIUS (geometry: DRIVE_CLEARANCE = R + 4 cm -> .20) is the fallback planning clearance for narrow
# passages; exact for the round body, turning included. The local planner still checks measured points.
# Firmware rule while the bumper is pressed (Source/HomeRobotSim/DriveCore.h): re-arm only for a reverse with
# both wheels <= 0 (|w| <= 2|v|/track) or a pure spin (|w| <= .5) TOWARD the pressed side (left switch w >= 0,
# right w <= 0, both either); anything else holds both wheels at zero. PRESSED_SPIN_PURE=True matches it: a
# pressed robot may spin toward the contact (|w| <= CONTACT_SPIN), and reverses only along the driven trail with
# |w| <= BUMP_YAW. command() enforces this (firmware_executes) whatever drive_command asks. Older cores clamped
# both wheel setpoints to <= 0 while pressed, which turns a spin into a backward one-wheel pivot; if a pressed
# spin's wheel odometry shows backward travel the controller switches to that clamp rule for the session.
PRESSED_SPIN_PURE=True;BUMP_YAW=.3;PIVOT_DETECT_M=.0005
CONTACT_SPIN=.25    # rad/s; pure spin toward the pressed side (only with PRESSED_SPIN_PURE); <= BUMP_YAW and CLOSE_SPIN_W
# Circle-rule measurements: each LiDAR beam is averaged over the scans taken while the robot
# stood on this spot with this heading (scan pose within SPIN_STILL_M and SPIN_STILL_YAW of the
# current scan's pose, last SPIN_WINDOW_S, at most SPIN_SCANS), see spin_points. A refused spin
# is re-measured (standing still up to SPIN_WAIT_S, until SPIN_SCANS scans) before any escape
# motion; escapes need at least SPIN_SCANS_MIN. SPIN_STILL_YAW: .003 rad moves a beam line .5 mm
# at .17 m (the per-beam lateral gate), so scans taken at other headings never match anyway.
SPIN_SCANS=8;SPIN_SCANS_MIN=6;SPIN_WINDOW_S=2.5;SPIN_WAIT_S=2.5;SPIN_STILL_M=.003;SPIN_STILL_YAW=.003;SPIN_NEAR=.30
# Spin hysteresis: a pure spin changes no distance, so once the circle rule accepted a spin from
# this spot (see spin_rule) it is not aborted mid-turn because a scan taken WHILE turning (beams
# on new lines, nothing to average with) looks closer. While the commanded motion stays a pure
# spin and base_link stays within SPIN_DRIFT of where the rule last passed, it continues unless
# a point comes closer than TURN_CLEAR - SPIN_RAW_ALLOWANCE (3 sigma of the 4 mm single-scan
# range noise): something genuinely new (a moving obstacle), not noise.
SPIN_RAW_ALLOWANCE=.012
# Close spin (no blind reversing). The rear half is an unsensed rigid shell and the camera looks
# forward, so a reverse may only retrace the driven trail (on_trail). When the circle rule refuses a
# spin (a point inside TURN_CLEAR) but nothing touches, the robot turns in place SLOWLY instead: a
# pure spin about base_link changes no distance to anything. Allowed (close_spin_clear) when every
# beam-averaged point is >= CLOSE_SPIN_CLEAR (R + 3 mm); a point in the bumper's front half may
# instead be verified by the released skin (it trips on contact) as long as the LiDAR average does
# not put it more than CLOSE_SPIN_FRONT_SLACK inside the skin (3 sigma of the averaged range noise,
# 4 mm / sqrt(SPIN_SCANS_MIN)): with 4 mm range noise the minimum over ~60 averaged wall beams reads
# 2-3 mm short of the truth, so the LiDAR alone cannot confirm a 3-5 mm gap.
# DEVIATION FROM THE LITERAL USER RULE, NEEDS USER SIGN-OFF: the user's rule is 'every beam-averaged
# point >= R + 3 mm' and 'abort when a raw point comes within R'. With 4 mm LiDAR noise the first
# refuses the 3 and 5 mm kitchen corners the same rule requires to turn in place, and the second
# fires on noise every scan at those gaps. So front-half (and skin-entry) points are verified by the
# released skin instead, and the abort uses what is measurable (close_spin_contact, close_spin_stalled,
# a bumper press on the side the firmware refuses to turn toward: left switch w >= 0 only, right
# switch w <= 0 only); then it re-measures. A point the turn carries from the front past a skin end
# onto the rigid rear shell (R_RIGID, 1.5 mm inside the skin) then has 1.5 mm + the measured gap of
# clearance; base_link drift during the turn (SPIN_DRIFT, up to 5 mm per half turn) can exceed that
# at a 3 mm gap, and the rear has no contact sensor (only a gyro stall shows a rear touch).
CLOSE_SPIN_W=.25;CLOSE_SPIN_FRONT_SLACK=.005;CLOSE_SPIN_CONTACT_POINTS=2
# A point up to CLOSE_SPIN_ENTRY_DEG behind a skin end, that the chosen spin direction turns into the
# front half, counts as skin-verified (it reaches the skin within 20 deg of turning; e.g. a wall
# tangent at the skin end, +/-90 deg, has its nearby returns just behind it). After the close spin a
# straight forward drive of at most CLOSE_SPIN_EXIT (at CLOSE_SPIN_EXIT_V) leaves what is now behind and
# restores spin room.
CLOSE_SPIN_ENTRY_DEG=20;CLOSE_SPIN_EXIT=.15;CLOSE_SPIN_EXIT_V=.06
# What a close spin can actually measure while turning (no averaged beams exist mid-turn: every scan
# is taken at a new heading): (1) the bumper skin, front half only; (2) the gyro: a turn that the
# body cannot complete (a rigid-shell touch, typically in the unsensed rear, after base_link drift)
# shows as |gyro z| below CLOSE_SPIN_STALL_FRACTION of the command for CLOSE_SPIN_STALL_S (beyond the
# wheel ramp); (3) LiDAR returns that place an object INSIDE the body by more than the single-scan
# noise (CLOSE_SPIN_CONTACT_POINTS returns, SPIN_RAW_ALLOWANCE inside the skin in front, inside the
# rigid shell R_RIGID behind), i.e. something new or a large pose error. The REAR HAS NO CONTACT
# SENSOR: a rear touch of a few mm is below the 4 mm single-scan range noise, so only (2) reveals it.
# A close spin that ended in a contact or stall is not retried in the same direction from the same
# spot (CLOSE_SPIN_RETRY_M, CLOSE_SPIN_RETRY_S) while the other direction is allowed.
CLOSE_SPIN_STALL_S=.8;CLOSE_SPIN_STALL_FRACTION=.3;CLOSE_SPIN_RETRY_M=.03;CLOSE_SPIN_RETRY_S=60.
# Reverse permission (the user rule: reverse only over ground the body has already driven over).
# The driven trail is base_link's position every 1 cm while driving FORWARD or turning. After any reverse
# (reversed_since_forward, set by command(); a timer alone is not enough) nothing is recorded until base_link
# has driven FORWARD_CLEAR_M forward from where the forward drive began (and REVERSE_SETTLE_S has passed, the
# wheel ramp), and the first sample after it starts a new run of the trail (trail_breaks): it is never
# joined to the samples before the reverse, so no chord across reversed-over ground counts as driven, and a
# reverse can never extend the ground it is checked against. on_trail: base_link within TRAIL_TOL (2.5 cm) of that polyline (segments of at most
# TRAIL_GAP; the coarse visit path is NOT used), but never more than TRAIL_CAP_TOL beyond an end of
# the trail (its oldest point, or a break), measured along the trail (past an interior sample only
# TRAIL_JOINT, which covers the outer side of a bend): the band is lateral tolerance for pose error,
# not room to lengthen the trail into unseen ground. The newest driven position (trail_head: base_link
# now, or where the current reverse episode began) closes the gap to the newest 1 cm sample.
# Straight reverses planned ahead (contact back-off, retreat,
# margin escape) keep TRAIL_PLAN_TOL so a straight line against a curved trail plus pose error stays
# inside the band. command() is the final guard: each reverse command must keep the next
# REVERSE_GUARD_M (this tick + the wheel ramp's stopping distance, with margin) on the trail, and one
# reverse episode (reverse commands not separated by REVERSE_EPISODE_RESET of forward travel) may
# cover at most REVERSE_LIMIT in total.
TRAIL_TOL=.025;TRAIL_CAP_TOL=.005;TRAIL_JOINT=.01;TRAIL_GAP=.05;TRAIL_PLAN_TOL=.015
REVERSE_GUARD_M=.02;REVERSE_SETTLE_S=.5;REVERSE_EPISODE_RESET=.05;FORWARD_CLEAR_M=.01
# A reverse episode ends only after forward travel of at least max(REVERSE_EPISODE_RESET, the distance it
# reversed), so repeated forward / back cycles cannot walk the robot backward along its trail.
NARROW_BLOCK_S=300  # seconds a narrow-passage stall keeps narrow planning away from that pinch
STALL_MARK_SPACING=.10   # m; a second unexplained stall this close to a marked one is skipped, not marked again (mark_stall)
STALL_BACKOFF=.05        # m; back-off after a marked stall: its nearest mark (R + .05 ahead) ends R + .10 = .259 away, outside PLAN_RADIUS,
                         # and the LiDAR turret (it reaches .05 from base_link) can spin again. It can follow up to two .035 jam retreats
                         # in the same reverse episode (the retries end where the retreats began), so it stays short.
STALL_MARK_LIMIT=8       # unexplained stalls marked per session; the 8th is marked and ends the session (mark_stall). Each mark costs
                         # at most three pushes (often the LiDAR turret against furniture: note_jam's three jams), so <= 24 per session.
                         # The overhang scenarios need 5 (sofa: 2 m long, three open edges) and 1 per Go-to (table); the caster-catch
                         # rug edge 1. More than that in one session is not a piece of furniture but something systemic (a room of
                         # low frames, a bad pose): stop and let the user look rather than keep pushing.
JAM_SETTLE_S=1.          # s standing still after a jam before acting on it (retreat, retry, skip or mark_stall): the wheel ramp (.4 s
                         # from .1 m/s) still pushes, and while it pushes the pose estimate swings 1-2.5 cm ahead of the blocked body
                         # (odometry vs LiDAR). Reversing at once counted that swing back as reverse travel: two retreats used up
                         # ~.13 of the .15 m episode budget for ~.03 m of real motion, leaving no back-off after a marked stall.
# Go-to target closer to an obstacle than the planner's safe mask (begin_go_to accepts it when its cell-centre
# clearance to occupied and unmapped cells is >= GOAL_CLEAR = R - 2 cm, which covers the 5 cm cell quantization;
# the approach decides how close the body really gets). The normal A* route (radius unchanged)
# goes to a STAGING cell: the reachable safe cell nearest the target, preferring one with a straight line of
# measured free cells (clearance >= GOAL_CLEAR) to it. Then phase 'approach': turn in place to face the target
# (the circle rule, as every spin), then drive straight at it, FORWARD ONLY, re-checking every tick: the body
# centre's positions over the stopping distance keep DRIVE_CLEARANCE_LIDAR from every LIVE LiDAR point (robot
# frame, carried from the scan pose to now by odometry), DRIVE_CLEARANCE from camera / contact cells (map frame)
# and stay on measured free cells; a point already inside its limit may not come any closer (per point, no slack).
# Steering: the arc through the target tangent to the heading (curvature 2 sin(error) / distance, at most
# APPROACH_CURVE; straight in the last APPROACH_NEAR), commanded as w = curvature x v, so the checked arc is the
# driven one. Speed: min(APPROACH_V, room / APPROACH_STOP_S), where room is the distance along that arc to the
# first position that breaks a check (or to the target); the speed thus falls to zero on the closest safe point.
# One tick (.05 s) plus the wheel ramp's stop (.2 s at .25 m/s2 from .05 m/s) is < APPROACH_STOP_S. Below
# APPROACH_V_MIN it stops there. 'Arrived' when the stop is within APPROACH_ARRIVED of the target, otherwise
# 'Stopped N cm short'; arrival is then confirmed over five LiDAR scans at the stop point. A bumper contact
# ends the approach through the normal contact handling (record, back off along the driven trail) and the
# Go-to ends where that leaves the robot. EVERY Go-to target ends this way: a safe-mask target is followed on
# its route as before, and when the follow phase stops (within .10 of it) more than APPROACH_ARRIVED away, the
# same approach (same checks) covers the rest. Mapping viewpoints are unchanged. Not body geometry: APPROACH_V .05 m/s, APPROACH_V_MIN .01 m/s,
# APPROACH_STOP_S .3 s, APPROACH_STEP 1 mm (line sampling), APPROACH_REACHED 5 mm (target reached along the
# line), APPROACH_ARRIVED .03 m, APPROACH_ALIGN .03 rad / APPROACH_REALIGN .25 rad (heading to the target),
# APPROACH_W .4 rad/s / APPROACH_W_MIN .1 rad/s (turn in place), APPROACH_CURVE 1 rad/m and APPROACH_NEAR .05 m
# (steering), APPROACH_LOOK .5 m (look-ahead), APPROACH_STAGING_M 1 m (staging search window), APPROACH_TIMEOUT_S 60 s.
APPROACH_V=.05;APPROACH_V_MIN=.01;APPROACH_STOP_S=.3;APPROACH_STEP=.001;APPROACH_REACHED=.005;APPROACH_ARRIVED=.03
APPROACH_ALIGN=.03;APPROACH_REALIGN=.25;APPROACH_W=.4;APPROACH_W_MIN=.1;APPROACH_CURVE=1.;APPROACH_NEAR=.05;APPROACH_LOOK=.5
APPROACH_STAGING_M=1.;APPROACH_TIMEOUT_S=60.
TIME_BUDGET=1500    # seconds of mapping; ~48 m2 at <=10 cm/s with recoveries
VISIT_RADIUS=1.2    # metres of travel within which floor counts as seen by the low-obstacle camera (range 1.2 m)
UNVISITED_DONE=.5   # m2 of reachable, never-approached floor still allowed at completion
HAZARDS=['cliff_or_invalid','tilt']
# Attitude while driving with no cliff floor plane (rugs, thresholds; bay B11): gravity = specific force minus the
# body's own acceleration from the wheels (dv/dt forward, v x yaw rate sideways), blended in slowly (time constant
# ATTITUDE_MOVING_TAU) when its norm is within MOVING_GRAVITY_GATE of the calibrated gravity. It bounds the roll /
# pitch drift from the gyro's x/y bias between stops (an MPU-6050 warming up drifts up to ~.004 deg/s per s).
ATTITUDE_MOVING_TAU=4.;MOVING_GRAVITY_GATE=.15
# Bumper skin (robot frame, metres): the front half of the round body, a circle
# of radius R about base_link from azimuth -90 to +90 deg. BUMP_ZONES (geometry,
# from bump_zone_edges_deg): bits 0 right .. 4 left by azimuth about base_link,
# edges +/-20, +/-55, +/-90 deg.
def shell_point(azimuth):
    point,direction=_shell_point(azimuth);return np.array(point),np.array(direction)
def contact_marks(bits):
    """Robot-frame marks for a touch on bumper zones bits (0 right .. 4 left): 5 and 8 cm outside the skin,
    inside the touched object (its surface is at the skin), so 5 cm grid snapping cannot pull a mark in front of it."""
    return np.array([point+depth*direction for i in bits for az in np.linspace(*BUMP_ZONES[i],4) for point,direction in [shell_point(az)] for depth in (.05,.08)])
def contact_azimuth(bits):
    """Mean azimuth (deg, about base_link) of the touched zones; 0 when unknown."""
    return float(np.mean([sum(BUMP_ZONES[i])/2 for i in bits])) if bits else 0.

def downward_floor_pose(ranges):
    """Local floor attitude/height from calibrated body-mounted range origins.

    ROS body coordinates, metres, in native sensor order. Reject missing,
    out-of-range or non-coplanar returns; steps need not share one floor plane.
    This consumes four measured ranges, never simulator body orientation.
    """
    ranges=np.asarray(ranges,float)
    # Range gate .02-.08 m around the nominal CLIFF_Z (sensor hardware/hazard gate, as in the sim).
    if ranges.shape!=(4,) or not np.all((ranges>.02)&(ranges<.08)):return None
    design=np.c_[np.array(CLIFF_XY),np.ones(4)]
    slope=np.linalg.lstsq(design,ranges,rcond=None)[0]
    if np.max(np.abs(design@slope-ranges))>.0015:return None
    roll=math.atan(slope[1]);pitch=math.atan(-slope[0]*math.cos(roll))
    if math.hypot(roll,pitch)>.20:return None
    # Sensor origins sit BASE_Z-CLIFF_Z below base_link; height is base_link's offset from nominal.
    height=(slope[2]+BASE_Z-CLIFF_Z)*math.cos(roll)*math.cos(pitch)-BASE_Z
    return np.array([roll,pitch,height])

class LidarBrain(Brain):
    def __init__(self,root,backend=None):
        super().__init__(root,'orb');self.features=SimpleNamespace(backend='d200_lidar');self.localizer=LidarLocalizer();self.grid=LidarGrid();self.camera_index=None;self.last_scan=-1;self.scan_count=0;self.visits=[];self.camera=MonoObserver();self.low_camera=LowObstacleCamera();self.tilt_rad=0.;self.attitude=np.zeros(2);self.attitude_history=deque(maxlen=200);self.camera_height_offset=0.;self.attitude_prev_gyro=np.zeros(3);self.attitude_time=None;self.attitude_initialized=False;self.gravity_samples=deque(maxlen=8);self.encoder_cpr=4741.44;self.pressed_spin_pure=PRESSED_SPIN_PURE;self.imu=ImuCalibration(self.root);self.reset_recovery(0.);self.reset_approach();self.message='Ready: LiDAR navigation with camera obstacle observations.'
    def reset_recovery(self,stamp):
        # Driven trail (1 cm spacing) is the only ground known clear behind the robot.
        self.relaxed_exit=False;self.contact_spin=None;self.spin_commit=None;self.scan_pose=None;self.spin_wait_from=None;self.spin_scans=deque(maxlen=SPIN_SCANS);self.bumper_sides=(False,False);self.reverse_refused=False;self.reverse_episode=None;self.last_reverse_stamp=-1e9;self.trail_break=False;self.trail_breaks=deque(maxlen=600);self.reversed_since_forward=False;self.forward_mark=None;self.pressed_refused=False;self.pressed_spin_odo=None;self.close_spin_blocked=None;self.close_spin_stall=None;self.gyro_z=0.;self.stuck_skips=0;self.stuck_anchor=None;self.trail=deque(maxlen=600);self.contacts=[];self.trouble=[];self.jams=[];self.stall_marks=[];self.wants_arm=False;self.bump_events=None;self.bumper_pressed=False;self.disarmed_since=None;self.tilted_since=None
        self.odometry=deque(maxlen=240);self.lidar_track=deque(maxlen=40);self.jam_reset=stamp;self.skips=0;self.coverage_mark=0.
        self.margin_anchor=None;self.margin_attempts=0;self.stall_position=None;self.stall_attempts=0;self.retreat_after_localization=False;self.translation_anchor=None;self.translation_since=stamp
    def reset_approach(self,near=False):
        """Final-approach state of a Go-to target outside the safe mask (see APPROACH_*): near (the
        target needs the approach), the staging cell, the running approach, its stop point (confirmed
        instead of the target) and result, and whether a contact ended it."""
        self.near_goal=near;self.staging=None;self.approach=None;self.approach_stop=None;self.approach_result=None;self.approach_contact=False
    def start_mapping(self,stamp):
        self.reset_recovery(stamp);self.reset_approach()
        super().start_mapping(stamp);self.localizer=LidarLocalizer();self.grid=LidarGrid();self.phase='sense';self.scan_count=0;self.last_scan=-1;self.visits=[];self.camera_index=None;self.current_cloud=np.empty((0,2));self.camera=MonoObserver();self.low_camera=LowObstacleCamera();self.failed_views=None;self.frontier_target=None;self.event('Mapping: measuring the room with LiDAR, then choosing unexplored space.')
    def begin_go_to(self,goal,stamp):
        self.reset_recovery(stamp)
        if self.mode=='mapping':self.save_map(False,'Switched to navigation')
        # The mapping session's visit path is coverage bookkeeping only; navigation starts afresh.
        self.visits=[]
        pointer=self.root/'Saved/Maps/latest-lidar-map.json'
        if not pointer.exists():raise ValueError('Create a LiDAR map first')
        folder=self.root/'Saved/Maps'/json.loads(pointer.read_text())['directory'];grid=LidarGrid();grid.load(folder);loc=LidarLocalizer();loc.load(folder)
        # Same map and a verified fix that odometry has carried since (e.g. the last arrival):
        # check that pose locally instead of a whole-map search from nothing.
        kept=loc.adopt_verified_pose(self.localizer)
        near=self.near_goal_check(grid,goal)
        self.reset_approach(near)
        self.grid=grid;self.localizer=loc;self.goal=np.asarray(goal,float);self.map_folder=folder;self.mode='go_to';self.phase='relocalizing';self.phase_started=stamp;self.started=stamp;self.path=[];self.arrival_frames=0;self.frame=None;self.last_scan=-1;self.motor_seen_armed=False;self.last_progress=stamp;self.progress_pose=np.zeros(3);self.camera=MonoObserver();self.low_camera=LowObstacleCamera();self.event('Checking the last verified position against the saved map before moving.' if kept else 'Locating the robot from LiDAR geometry in the saved map before moving.')
    @staticmethod
    def near_goal_check(grid,goal):
        """Go-to target acceptance. False: a safe-mask target (planned exactly as before). True: a target
        outside the safe mask that the final approach may serve: on measured free floor (not unknown, not a
        camera / contact cell) with cell-centre clearance >= GOAL_CLEAR (R - 2 cm: up to one 5 cm cell of
        quantization) to occupied cells and to unmapped cells. Raises ValueError otherwise (the dashboard
        shows the message)."""
        c=grid.cell(goal)
        if not grid.valid(c):raise ValueError('Choose a target inside mapped free space with enough clearance')
        free,_,_,safe,clearance=grid.masks()
        if safe[c]:return False
        if not free[c] or getattr(grid,'camera_obstacles',np.zeros_like(free))[c]:raise ValueError('Choose a target inside mapped free space with enough clearance')
        if clearance[c]<GOAL_CLEAR-1e-9:raise ValueError("That target is closer to an obstacle than the robot's body allows.")
        mapped=cv2.distanceTransform(free.astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*grid.res
        if mapped[c]<GOAL_CLEAR-1e-9:raise ValueError('Choose a target inside mapped free space with enough clearance')
        return True
    def observe_scan(self,scan):
        if scan['frame']==self.last_scan:return
        self.last_scan=scan['frame'];stamp=scan['sim_time_s'];self.frame=SimpleNamespace(stamp=stamp,index=scan['frame']);self.scan_count+=1
        points=scan_points(scan,self.localizer.history);self.current_cloud=points;self.grid.sensor_xy=np.asarray(scan['sensor_in_base_m'][:2],float)
        locating=self.phase in ['relocalizing','recover_localization'];good,pose=self.localizer.update_scan(points,stamp,self.mode=='mapping' and not locating,locating)
        # current_cloud is in the base frame of THIS pose (the scan's), not of the localizer pose
        # that predict() advances with odometry between scans; spin_points averages in it.
        self.scan_pose=np.asarray(pose,float).copy()
        if good and locating and self.imu.ready_to_move():self.phase='plan';self.phase_started=stamp;self.plan_not_before=stamp;self.last_progress=stamp;self.progress_pose=pose.copy();self.translation_anchor=None;self.event('Position verified across three LiDAR scans. Planning a route.')
        if good and self.mode=='mapping':self.grid.integrate_scan(points,pose)
        if good and (not self.visits or np.linalg.norm(pose[:2]-self.visits[-1])>.15):self.visits.append(pose[:2].copy())
        if good:
            self.note_spin_scan(points,pose,stamp)
            self.lidar_track.append((stamp,pose.copy()))
            self.record_trail(pose,stamp)
            # The round body is standing on this cell, so a camera or contact
            # obstacle recorded here was wrong or has moved.
            cell=self.grid.cell(pose[:2])
            if self.grid.valid(cell) and self.grid.camera_obstacles[cell]:self.grid.camera_obstacles[cell]=self.grid.camera_core[cell]=False;self.grid.logodds[cell]=-1.
        if self.phase=='confirm_arrival':
            self.arrival_frames=self.arrival_frames+1 if good and np.linalg.norm(pose[:2]-self.arrival_point())<.10 else 0
            if self.arrival_frames>=5:self.mode='idle';self.phase='idle';self.path=[];self.event(self.arrival_message(pose))
    def record_trail(self,pose,stamp):
        """Driven trail: base_link every 1 cm, reached driving forward or turning only. After a reverse
        nothing is added until base_link has driven FORWARD_CLEAR_M forward again (reversed_since_forward,
        cleared only in command() by forward travel) and REVERSE_SETTLE_S has passed, and the first sample
        after it is a run start (trail_breaks: not joined to the samples before), so a reverse is always
        checked against ground driven before it began and can never creep off it."""
        if self.requested_motion[0]<0 or stamp-self.last_reverse_stamp<REVERSE_SETTLE_S or getattr(self,'reversed_since_forward',False):self.trail_break=True;return
        if not self.trail or np.linalg.norm(pose[:2]-self.trail[-1])>=.01:
            if self.trail and not self.trail_break and np.linalg.norm(pose[:2]-self.trail[-1])<.10:self.grid.add_driven(self.trail[-1],pose[:2])
            if self.trail and self.trail_break:self.trail_breaks.append(tuple(float(x) for x in pose[:2]))
            self.trail.append(np.asarray(pose[:2],float).copy());self.trail_break=False
    def note_contact(self,stamp,kind,zones=None):
        """Record a touched obstacle ahead and start a short verified back-off."""
        if self.phase=='contact' and stamp-self.phase_started<10:return
        p=self.localizer.pose
        # Mark the touched part of the skin: zone bits (0 right .. 4 left, azimuth
        # about base_link), an [left, centre, right] list, or the whole front when
        # unknown. Marks sit 5 and 8 cm outside the skin, inside the touched object
        # (the surface is at the skin), so 5 cm grid snapping cannot pull a mark in front of it.
        if isinstance(zones,(list,tuple)):zones=(zones[2] and 3)|(zones[1] and 4)|(zones[0] and 24) if any(zones) else 0
        known=[i for i in range(5) if int(zones or 0)>>i&1];bits=known or [1,2,3]
        self.grid.add_camera_obstacles(transform(contact_marks(bits),p),margin=0)
        self.contacts.append(p[:2].copy());near=sum(np.linalg.norm(c-p[:2])<.5 for c in self.contacts)
        if self.mode=='mapping' and self.goal is not None and (np.linalg.norm(self.goal-p[:2])<.6 or near>=2):self.blacklist.append(self.goal.copy())
        if near>=4 or len(self.contacts)>=15:self.idle('Repeated contacts in one area. Stopped safely; partial map saved.');return
        # A close spin that ended in this contact is not retried in the same direction from this spot.
        m=getattr(self,'maneuver',None)
        if m and m.get('kind')=='close_spin':self.close_spin_blocked=dict(xy=p[:2].copy(),direction=m['direction'],stamp=stamp);self.maneuver=None
        self.start_contact_phase(stamp,kind,known)
    def start_contact_phase(self,stamp,kind,known,recorded=True,max_backoff=.12):
        """Contact phase: a short straight back-off along the driven trail if there is trail room;
        otherwise (no reverse off the trail, ever) a spin toward the contact where the firmware allows a
        pure spin while pressed, else stand still until the bumper releases. recorded=False: the bumper
        is still pressed outside the contact phase (no new touch, nothing new to record)."""
        p=self.localizer.pose
        # A contact during a Go-to's final approach (or while confirming where it stopped) ends that approach:
        # after this contact phase the Go-to ends where it leaves the robot (plan()).
        if self.mode=='go_to' and (self.phase=='approach' or getattr(self,'approach_stop',None) is not None):self.approach_contact=True;self.approach=None
        self.backoff=max(0.,min(max_backoff,self.reverse_budget(),self.reverse_room(REVERSE_LIMIT)-.015));self.escape_origin=p.copy();self.phase='contact';self.phase_started=stamp;self.path=[];self.next_plan=0;self.maneuver=None
        # No verified reverse room: spin in place (a spin about base_link moves no
        # part of the round body closer to anything) so that the contact azimuth
        # moves toward 0 deg, i.e. the contact comes to the front and never slides
        # back past the skin end (+/-90 deg) onto the unsensed rigid rear shell.
        # Turning by +a moves a contact at azimuth a to 0 (the firmware allows a pressed
        # spin only toward the pressed side, which is this direction). Unknown zones: no spin.
        # There is NO reverse off the driven trail: a contact in front with no trail room
        # waits for the skin to release (then the planner turns in place or drives away).
        # With the clamping motor core (PRESSED_SPIN_PURE False) a pressed spin would pivot backward: no spin.
        az=contact_azimuth(known)
        if self.backoff<.02:self.backoff=0.
        self.contact_spin=math.radians(az) if self.backoff<.02 and abs(az)>=10 and known and self.pressed_spin_pure else None
        what=f'Contact ({kind}) recorded on the map' if recorded else f'Still pressed against an obstacle ({kind})'
        if self.backoff>=.02:self.event(f'{what}. Reversing {100*self.backoff:.0f} cm along the path just driven, then replanning.')
        elif self.contact_spin is not None:self.event(f'{what} at {az:.0f} deg. No driven path behind; turning in place to bring the contact to the front.')
        else:self.event(f'{what}. No driven path behind and reversing there would be blind; waiting for the bumper to release.')
    @staticmethod
    def pressed_sides(sensors):
        """(left, right) bumper switches while pressed: bumper_left/right, else from the zone bits
        (left switch: zones 2-4, azimuth >= -20 deg; right: zones 0-2) or a [left, centre, right] list.
        (False, False) when the side is unknown."""
        if 'bumper_left' in sensors or 'bumper_right' in sensors:return bool(sensors.get('bumper_left')),bool(sensors.get('bumper_right'))
        zones=sensors.get('bump_zones',sensors.get('bumper_zones'))
        if isinstance(zones,(list,tuple)):return bool(zones[0] or zones[1]),bool(zones[1] or zones[2])
        bits=int(zones or 0);return bool(bits&0b11100),bool(bits&0b00111)
    def spin_allowed(self,direction):
        """Firmware rule for a pure spin (sign of w): free while released; while the bumper is pressed
        never with the clamping motor core (DriveCore.h: it would pivot backward about a wheel), and with
        a pure-spin core (PRESSED_SPIN_PURE) only toward the pressed side (left switch: w >= 0, right
        switch: w <= 0, both: either)."""
        if not self.bumper_pressed:return True
        if not getattr(self,'pressed_spin_pure',PRESSED_SPIN_PURE):return False
        left,right=self.bumper_sides
        return (left and right) or (left and direction>0) or (right and direction<0)
    def firmware_executes(self,v,w):
        """True when the motor core executes (v, w) as commanded in the current bumper state. Released:
        anything. Pressed: what the DriveCore.h clamp (both wheel setpoints <= 0, re-arm only for v <= 0 and
        |w| <= BUMP_YAW) leaves unchanged, i.e. a straight or arc reverse with both wheels <= 0; with a
        pure-spin core also a pure spin toward the pressed side at |w| <= CONTACT_SPIN."""
        if not self.bumper_pressed:return True
        ls,rs=v-w*WHEEL_TRACK/2,v+w*WHEEL_TRACK/2
        if ls<=1e-9 and rs<=1e-9 and abs(w)<=BUMP_YAW+1e-9:return True
        return bool(v==0 and abs(w)<=CONTACT_SPIN+1e-9 and self.spin_allowed(w))
    def check_pressed_pivot(self,sensors):
        """Defence against a firmware mismatch: a pure spin sent while pressed (PRESSED_SPIN_PURE) whose
        wheel odometry shows base_link travelling backward (mean wheel travel <= -PIVOT_DETECT_M) means the
        motor core clamps the wheels (DriveCore.h): switch to the clamp rule for the rest of the session."""
        v,w=getattr(self,'requested_motion',(0.,0.))
        if not (self.bumper_pressed and v==0 and w!=0 and 'left_ticks' in sensors):self.pressed_spin_odo=None;return
        ticks=np.array([sensors['left_ticks'],sensors['right_ticks']],float)
        if self.pressed_spin_odo is None:self.pressed_spin_odo=ticks;return
        travel=float((ticks-self.pressed_spin_odo).mean())*2*math.pi*WHEEL_R/self.encoder_cpr
        if travel<=-PIVOT_DETECT_M and self.pressed_spin_pure:
            self.pressed_spin_pure=False;self.contact_spin=None
            self.event('The motor core turned a pressed spin into a backward pivot (it clamps the wheels while the bumper is pressed). No more spins while pressed.')
    def trail_head(self):
        """The newest driven position, which the 1 cm samples lag by up to a sample: base_link now,
        when it got here driving forward or turning AND the newest sample was recorded after the last
        reverse (no reverse episode, no reverse since that sample, none within REVERSE_SETTLE_S); during a
        reverse episode the position where it began (if that was such a position); otherwise None."""
        ep=getattr(self,'reverse_episode',None)
        if ep is not None:return ep['start']
        now=getattr(self.frame,'stamp',None) if getattr(self,'frame',None) is not None else None
        if getattr(self,'requested_motion',(0.,0.))[0]<0 or (now is not None and now-getattr(self,'last_reverse_stamp',-1e9)<REVERSE_SETTLE_S):return None
        if getattr(self,'reversed_since_forward',False) or getattr(self,'trail_break',False):return None
        return self.localizer.pose[:2]
    def trail_segments(self):
        """Segments of the driven trail (consecutive 1 cm samples, then trail_head, at most TRAIL_GAP
        apart and not across a break: a sample in trail_breaks starts a new run, it was the first one
        recorded after a reverse), with flags marking trail ends (caps): the first / last sample of each
        unbroken run. An
        isolated sample is a zero-length segment capped at both ends. Rebuilt only when the trail or its
        head changes; the motion search queries it hundreds of times per decision."""
        head=self.trail_head();head=None if head is None else np.asarray(head,float)
        if head is not None and self.trail and np.linalg.norm(head-self.trail[-1])<.001:head=None
        key=(len(self.trail),tuple(self.trail[0]) if self.trail else None,tuple(self.trail[-1]) if self.trail else None,None if head is None else tuple(np.round(head,5)))
        if getattr(self,'trail_key',None)!=key:
            path=np.asarray(list(self.trail)+([head] if head is not None else []),float).reshape(-1,2)
            if not len(path):cache=(np.empty((0,2)),np.empty((0,2)),np.empty(0,bool),np.empty(0,bool))
            else:
                ok=np.linalg.norm(np.diff(path,axis=0),axis=1)<=TRAIL_GAP
                breaks=set(getattr(self,'trail_breaks',()))
                if breaks:ok&=np.array([tuple(float(x) for x in q) not in breaks for q in path[1:]],bool)
                first=np.r_[True,~ok];last=np.r_[~ok,True]          # run starts / ends, per sample
                single=first&last
                a=np.vstack((path[:-1][ok],path[single]));b=np.vstack((path[1:][ok],path[single]))
                cap_a=np.r_[first[:-1][ok],np.ones(single.sum(),bool)];cap_b=np.r_[last[1:][ok],np.ones(single.sum(),bool)]
                cache=(a,b,cap_a,cap_b)
            self.trail_key=key;self.trail_cache=cache
        return self.trail_cache
    def trail_distance(self,points):
        """Distance from points to the driven trail (1 cm samples, forward driving and turning only).
        Beyond a trail end (cap) by more than TRAIL_CAP_TOL along the trail the distance is infinite:
        the tolerance band is lateral (pose error), it does not lengthen the trail."""
        a,b,cap_a,cap_b=self.trail_segments();points=np.atleast_2d(np.asarray(points,float))
        if not len(a):return np.full(len(points),np.inf)
        ab=b-a;l2=(ab*ab).sum(1);length=np.sqrt(l2)
        raw=np.einsum('pkd,kd->pk',points[:,None,:]-a[None],ab)/np.maximum(1e-12,l2)
        t=np.clip(raw,0,1)
        d=np.linalg.norm(points[:,None,:]-(a[None]+t[...,None]*ab[None]),axis=2)
        # Past a segment's end the distance counts only up to TRAIL_JOINT along it (to cover the outer
        # side of a bend between 1 cm samples), and only TRAIL_CAP_TOL past a trail end: a round band
        # around end samples would let a reverse lengthen the trail by up to TRAIL_TOL.
        beyond=np.where(raw<0,-raw*length,np.where(raw>1,(raw-1)*length,0.))
        capped=((raw<0)&cap_a[None])|((raw>1)&cap_b[None])
        point=l2<1e-12                                                  # isolated sample: a cap all round
        beyond=np.where(point[None],d,beyond);capped=capped|point[None]
        d=np.where(beyond>np.where(capped,TRAIL_CAP_TOL,TRAIL_JOINT),np.inf,d)
        return d.min(axis=1)
    def on_trail(self,points,tolerance=TRAIL_TOL):
        return bool((self.trail_distance(points)<=tolerance).all())
    def reverse_budget(self):
        """Reverse distance still allowed in the current reverse episode (REVERSE_LIMIT in total)."""
        ep=getattr(self,'reverse_episode',None)
        return REVERSE_LIMIT if ep is None else max(0.,REVERSE_LIMIT-ep['travelled'])
    def reverse_room(self,limit=REVERSE_LIMIT,tolerance=TRAIL_PLAN_TOL):
        """Straight reverse distance that stays on ground the chassis has driven over (within
        `tolerance` of the driven trail) and within the episode's reverse budget."""
        p=self.localizer.pose;steps=np.arange(.01,min(limit,self.reverse_budget()+.03)+1e-6,.01)
        if not len(steps):return 0.
        off=self.trail_distance(p[:2]-steps[:,None]*np.array([math.cos(p[2]),math.sin(p[2])]))>tolerance
        if not off.any():return float(steps[-1])
        first=int(off.argmax());return float(steps[first-1]) if first else 0.
    def jammed(self,stamp):
        """Wheels report travel or turning that LiDAR scan matching does not show."""
        odometry=[x for x in self.odometry if x[0]>=max(stamp-2,self.jam_reset)];track=[x for x in self.lidar_track if x[0]>=max(stamp-2,self.jam_reset)]
        if len(odometry)<20 or len(track)<5 or track[-1][0]-track[0][0]<1.2:return False
        # Net wheel travel and turn over the window, compared like for like with
        # the net LiDAR displacement, so rocking back and forth is not a jam.
        wheels=(np.array(odometry[-1][1:],float)-np.array(odometry[0][1:],float))*(2*math.pi*WHEEL_R/self.encoder_cpr)
        travel=abs(float(wheels.mean()));turn=abs(float(wheels[1]-wheels[0]))/WHEEL_TRACK
        moved=float(np.linalg.norm(track[-1][1][:2]-track[0][1][:2]));rotated=abs(wrap(track[-1][1][2]-track[0][1][2]))
        return moved<.012 and (travel>.06 or (turn>.35 and rotated<.05))
    def predict(self,state):
        sensors=state.get('sensors',{});stamp=state['sim_time_s'];fault=state.get('fault','none')
        if 'left_ticks' in sensors:self.odometry.append((stamp,sensors['left_ticks'],sensors['right_ticks']));self.encoder_cpr=sensors.get('encoder_cpr',self.encoder_cpr)
        # The bumper event counter catches touches shorter than one 20 Hz sample.
        events=sensors.get('bumper_events');missed=events is not None and self.bump_events is not None and events>self.bump_events
        if events is not None:self.bump_events=events
        pressed=bool(sensors.get('bumper')) or fault=='bumper';touched=(pressed and not self.bumper_pressed) or missed;self.bumper_pressed=pressed
        self.bumper_sides=self.pressed_sides(sensors) if pressed else (False,False)
        self.check_pressed_pivot(sensors)
        if self.mode!='idle' and fault not in HAZARDS and (pressed or missed):
            # One contact per touch (rising edge), however long it stays pressed.
            zones=sensors.get('bump_zones',sensors.get('bumper_zones'))
            if not zones and ('bumper_left' in sensors or 'bumper_right' in sensors):
                left,right=bool(sensors.get('bumper_left')),bool(sensors.get('bumper_right'));zones=[left and not right,left and right,right and not left]
            if touched:self.note_contact(stamp,'front bumper',zones)
            elif pressed and self.mode in ('mapping','go_to') and self.phase not in ('contact','relocalizing','recover_localization','idle'):
                # Still pressed outside the contact phase (pressed at the start, or a missed edge): only the
                # contact phase's motions are legal while pressed, so go back to it (no new contact record).
                if isinstance(zones,(list,tuple)):zones=(zones[2] and 3)|(zones[1] and 4)|(zones[0] and 24) if any(zones) else 0
                self.start_contact_phase(stamp,'front bumper',[i for i in range(5) if int(zones or 0)>>i&1],recorded=False)
            # Handled as an obstacle observation, so the base class must not stop the run.
            state=dict(state,fault='none',sensors=dict(sensors,bumper=False));sensors=state['sensors']
        # IMU calibration (imu.py): gyro bias from the start-up rest and every later stop (ZUPT); accelerometer offsets
        # from the bring-up level calibration file plus the online correction from the cliff ToF floor plane. Everything
        # below, the base class and the localizer use the corrected values; the gravity gate is relative to the
        # calibrated gravity magnitude (a raw MPU-6050 norm can be 0.8 m/s2 off 9.8).
        cliff=np.asarray(sensors.get('cliff_m',[]),float)
        floor_pose=downward_floor_pose(cliff)
        gyro_c,force_c=self.imu.update(stamp,sensors,state.get('armed',False),self.requested_motion,None if floor_pose is None else floor_pose[:2])
        corrected={k:v.tolist() for k,v in (('gyro_rad_s',gyro_c),('specific_force_m_s2',force_c)) if v is not None}
        if corrected:sensors=dict(sensors,**corrected);state=dict(state,sensors=sensors)
        force=force_c if force_c is not None else np.array([0.,0.,self.imu.gravity])
        measured=np.array([math.atan2(force[1],force[2]),math.atan2(-force[0],math.hypot(force[1],force[2]))])
        gyro=gyro_c if gyro_c is not None else np.zeros(3);self.gyro_z=float(gyro[2])
        gravity=abs(np.linalg.norm(force)-self.imu.gravity)<GRAVITY_GATE
        if 'left_ticks' in sensors and 'right_ticks' in sensors:
            travel=(sensors['left_ticks']+sensors['right_ticks'])/2*2*math.pi*WHEEL_R/sensors.get('encoder_cpr',self.encoder_cpr);prev=getattr(self,'wheel_travel',None)
            if prev is not None and 0<stamp-prev[0]<.5:
                speed=(travel-prev[1])/(stamp-prev[0]);old=getattr(self,'wheel_speed',None)
                self.wheel_accel=0. if old is None else (speed-old)/(stamp-prev[0]);self.wheel_speed=speed
            else:self.wheel_speed=self.wheel_accel=None
            self.wheel_travel=(stamp,travel)
        if not self.attitude_initialized:
            # Gravity samples only at rest (imu.at_rest), and the attitude starts only once the gyro bias is known
            # (provisional estimate): it integrates the gyro from then on.
            if gravity and self.imu.still and np.linalg.norm(gyro)<.1:self.gravity_samples.append(measured)
            else:self.gravity_samples.clear()
            if len(self.gravity_samples)>=5 and (self.imu.bias_valid or gyro_c is None):self.attitude=np.median(self.gravity_samples,axis=0);self.attitude_initialized=True
        elif self.attitude_time is not None:
            dt=stamp-self.attitude_time
            if 0<dt<.5:
                gx,gy,gz=.5*(gyro+self.attitude_prev_gyro);roll,pitch=self.attitude
                self.attitude+=np.array([gx+math.sin(roll)*math.tan(pitch)*gy+math.cos(roll)*math.tan(pitch)*gz,math.cos(roll)*gy-math.sin(roll)*gz])*dt
                # At rest by the IMU's own test (no command for the wheel ramp, encoders unchanged, steady): a robot still
                # braking after a stop command is not at rest, and its deceleration must not tilt the attitude.
                stationary=self.imu.still
                if stationary and gravity:self.attitude+=(measured-self.attitude)*min(.15,dt*2)
                elif not stationary and getattr(self,'wheel_speed',None) is not None:
                    own=force-np.array([self.wheel_accel,self.wheel_speed*gyro[2],0.])
                    if abs(np.linalg.norm(own)-self.imu.gravity)<MOVING_GRAVITY_GATE:
                        self.attitude+=(np.array([math.atan2(own[1],own[2]),math.atan2(-own[0],math.hypot(own[1],own[2]))])-self.attitude)*min(.05,dt/ATTITUDE_MOVING_TAU)
        self.floor_range_valid=floor_pose is not None and self.attitude_initialized
        if floor_pose is not None and self.attitude_initialized:
            self.attitude=floor_pose[:2];self.camera_height_offset=float(np.clip(floor_pose[2],-.02,.04))
        elif len(cliff)==4 and np.all((cliff>.02)&(cliff<.08)):
            # Plane fit failed (a step under one sensor): mean range at the mean sensor position.
            roll,pitch=self.attitude;mx,my=np.mean(np.array(CLIFF_XY),axis=0)
            self.camera_height_offset=float(np.clip((np.mean(cliff)+BASE_Z-CLIFF_Z)*math.cos(roll)*math.cos(pitch)+mx*math.sin(pitch)-my*math.sin(roll)*math.cos(pitch)-BASE_Z,-.02,.04))
        self.attitude_prev_gyro=gyro.copy();self.attitude_time=stamp;self.attitude_history.append((stamp,np.r_[self.attitude,self.camera_height_offset,float(self.floor_range_valid)]));self.tilt_rad=float(np.linalg.norm(self.attitude))
        # The localizer's heading uncertainty grows with the bias filter's current sigma (it grows between stops).
        self.localizer.gyro_bias_sigma=float(math.sqrt(self.imu.P[2]))
        super().predict(state)
    def camera_pose(self,stamp):
        h=list(self.localizer.history)
        if h:
            times=np.array([x[0] for x in h]);poses=np.array([x[1] for x in h]);poses[:,2]=np.unwrap(poses[:,2]);pose=np.array([np.interp(stamp,times,poses[:,j]) for j in range(3)])
        else:pose=self.localizer.pose.copy()
        attitude=np.r_[self.attitude,self.camera_height_offset,float(getattr(self,'floor_range_valid',False))]
        if self.attitude_history:
            times=np.array([x[0] for x in self.attitude_history]);angles=np.array([x[1] for x in self.attitude_history]);attitude=np.array([np.interp(stamp,times,angles[:,j]) for j in range(4)])
        return np.r_[pose,attitude]
    def observe(self,left,right,meta):
        self.camera_index=meta['frame']
        geometry_ready=self.attitude_initialized and self.localizer.uncertainty<=.08 and self.phase not in ['relocalizing','recover_localization'] and self.tilt_rad<=.15
        if self.map_folder is not None:
            self.map_folder.mkdir(parents=True,exist_ok=True)
            with (self.map_folder/'camera-inputs.jsonl').open('a') as log:
                log.write(json.dumps(dict(meta=meta,estimated_pose=self.camera_pose(meta['sim_time_s']).tolist(),geometry_ready=geometry_ready))+'\n')
        if not geometry_ready:
            self.low_camera.reset_history();self.camera=MonoObserver();self.camera.status['state']='waiting for stable, localized view';self.camera.status['attitude_deg']=np.degrees(self.attitude).tolist();self.camera.status['pose_at_image']=self.camera_pose(meta['sim_time_s']).tolist();self.camera.status['image_stamp']=meta['sim_time_s'];self.camera.last_stamp=meta['sim_time_s'];return
        points=self.camera.process(left,meta,self.camera_pose(meta['sim_time_s']));self.camera.status['attitude_deg']=np.degrees(self.attitude).tolist();self.camera.status['pose_at_image']=self.camera_pose(meta['sim_time_s']).tolist();self.camera.status['image_stamp']=meta['sim_time_s']
        q=self.localizer.quality
        if self.camera.bad_frames>=3 and q.get('fraction',0)>.85 and q.get('median_error_m',1)<.015 and meta['sim_time_s']-self.localizer.last_visual_time<.5:
            # The image-motion check can disagree when turning in front of a
            # plain wall. A strong, fresh LiDAR match outranks it.
            self.camera=MonoObserver();self.camera.last_stamp=meta['sim_time_s'];self.event('Camera motion check disagreed, but the LiDAR match is strong. Continuing.');return
        if self.camera.bad_frames>=3:
            self.localizer.invalidate();self.localizer.pending=None;self.localizer.relocalization_hits=0
            self.phase='recover_localization';self.phase_started=meta['sim_time_s'];self.path=[]
            self.camera=MonoObserver();self.camera.last_stamp=meta['sim_time_s'];self.low_camera.reset_history()
            self.event('Camera motion disagrees. Stopped to verify location against three LiDAR scans before replanning.');return
        # Low obstacles come from the dense floor-parallax detector; MonoObserver
        # remains for its camera/odometry motion-consistency check only.
        points=self.low_camera.process(left,meta,self.camera_pose(meta['sim_time_s']))
        if self.mode!='idle' and len(points):
            # Confirmed cells (two views, two points each); the planner's 25 cm
            # clearance already pads them, so no extra cell margin is added.
            self.grid.add_camera_obstacles(points,margin=0);self.next_plan=0
            if self.map_folder is not None:
                self.map_folder.mkdir(parents=True,exist_ok=True)
                with (self.map_folder/'camera-observations.jsonl').open('a') as log:log.write(json.dumps(dict(stamp=meta['sim_time_s'],estimated_pose=self.camera_pose(meta['sim_time_s']).tolist(),points=points.tolist(),checks=self.low_camera.status))+'\n')

    def save_map(self,complete,reason):
        if self.map_folder is None:return
        self.localizer.save(self.map_folder);self.grid.save(self.map_folder)
        safe=self.grid.masks()[3];reachable=self.grid.reachable(self.localizer.pose[:2],safe)
        manifest=dict(sensor_configuration='d200_single_camera',complete=complete,reason=reason,unresolved_frontier_count=len(self.blacklist),camera_role='multi-view sparse low-obstacle map and motion consistency check',camera_obstacle_cells=int(self.grid.camera_obstacles.sum()),unvisited_navigable_area_m2=self.unvisited_area(),disconnected_safe_area_m2=float((safe&~reachable).sum()*self.grid.res**2),planning_radius_m=self.grid.radius,scope='observed reachable space; sparse camera coverage does not guarantee all low objects or cables',pose_confidence_is_heuristic=True,keyframes=len(self.localizer.keyframes),contacts=len(self.contacts),skipped_targets=len(self.blacklist),trouble_spots=len(self.trouble),**self.grid.summary())
        (self.map_folder/'manifest.json').write_text(json.dumps(manifest,indent=2))
        (self.map_folder/'exploration.json').write_text(json.dumps(dict(visits=[p.tolist() for p in self.visits],viewpoints=[p.tolist() for p in self.scanned_positions],blocked_goals=[p.tolist() for p in self.blacklist],contacts=[p.tolist() for p in self.contacts],trouble_spots=[p.tolist() for p in self.trouble]),indent=2))
        for name in ['latest-map.json','latest-lidar-map.json']:(self.root/'Saved/Maps'/name).write_text(json.dumps({'directory':self.map_folder.name}))
    def unvisited_mask(self):
        free,_,_,safe,_=self.grid.masks()
        covered=np.zeros_like(safe,np.uint8)
        for v in self.visits:
            y,x=self.grid.cell(v)
            if self.grid.valid((y,x)):covered[y,x]=1
        # Travel distance through measured free space, not Euclidean distance
        # through walls. A glimpse into a room does not count as visiting it.
        kernel=np.array([[0,1,0],[1,1,1],[0,1,0]],np.uint8)
        for _ in range(int(VISIT_RADIUS/self.grid.res)):
            covered=cv2.dilate(covered,kernel)&free.astype(np.uint8)
        unvisited=safe&~covered.astype(bool)&self.grid.reachable(self.localizer.pose[:2],safe);r=int(.35/self.grid.res)
        # Targets skipped after contacts or stalls are reported, not chased forever.
        for xy in self.blacklist+self.trouble:
            y,x=self.grid.cell(xy);unvisited[max(0,y-r):y+r+1,max(0,x-r):x+r+1]=False
        return unvisited
    def unvisited_area(self):
        return float(self.unvisited_mask().sum()*self.grid.res**2)
    def travel_distances(self,pose):
        """Driving distance (m) to every cell the planner may cross, from pose."""
        free,_,_,safe,clearance=self.grid.masks();start=self.grid.cell(pose[:2])
        if not self.grid.valid(start):return {}
        mask=self.grid.traversable(start,free,safe,clearance);dist={start:0.};queue=[(0.,start)]
        while queue:
            d,c=heapq.heappop(queue)
            if d>dist.get(c,math.inf):continue
            for dy,dx,step in ((1,0,1.),(-1,0,1.),(0,1,1.),(0,-1,1.),(1,1,SQRT2),(1,-1,SQRT2),(-1,1,SQRT2),(-1,-1,SQRT2)):
                n=(c[0]+dy,c[1]+dx)
                if not self.grid.valid(n) or not mask[n]:continue
                if dy and dx and (not mask[c[0],n[1]] or not mask[n[0],c[1]]):continue
                nd=d+step*self.grid.res
                if nd<dist.get(n,math.inf):dist[n]=nd;heapq.heappush(queue,(nd,n))
        return dist
    def coverage_viewpoints(self,pose):
        cells=np.argwhere(self.unvisited_mask()&self.grid.reachable(pose[:2]));proposals=[]
        # Filter connectivity before limiting expensive A* calls. Previously
        # 24 nearby disconnected cells could hide an entire reachable room.
        # Order by travel distance through the planning mask, not straight-line
        # distance: the nearest cell through a wall can be a trip across the house.
        travel=self.travel_distances(pose)
        for cell in cells:
            xy=self.grid.world(cell)
            if any(np.linalg.norm(xy-v)<.35 for v in self.blacklist+self.trouble):continue
            proposals.append((travel.get(tuple(cell),np.linalg.norm(xy-pose[:2])+100),xy))
        for _,xy in sorted(proposals,key=lambda item:item[0]):
            route=self.grid.path(pose[:2],xy)
            if route:return [(1,xy,route)]
        return []
    def clearance_escape(self):
        """Leave a newly inflated margin along a measured clear straight line.

        This is a short recovery, never a reduced-clearance global route. No
        rotation, unknown-space entry or approach toward the nearby obstacle.
        """
        p=self.localizer.pose;free,_,_,safe,clearance=self.grid.masks();start=self.grid.cell(p[:2])
        if not self.grid.valid(start) or not free[start]:return None
        cloud=self.obstacle_points()
        if len(cloud)<60:return None
        minimum=float(np.linalg.norm(cloud,axis=1).min())
        if minimum<BODY_CLEAR:return None
        # A straight run (forward DRIVE_CLEARANCE = 20 cm) that ends DRIVE_CLEARANCE + 2.5 cm from everything.
        # Reversing: at most REVERSE_LIMIT (and only along the driven trail); the end must still be clear.
        for sign in [1.,-1.]:
            run=DRIVE_CLEARANCE if sign>0 else min(REVERSE_LIMIT,self.reverse_budget()-.01)
            offsets=np.linspace(0,run,21)*sign
            line=p[:2]+offsets[:,None]*np.array([math.cos(p[2]),math.sin(p[2])])
            cells=[self.grid.cell(xy) for xy in line]
            if not all(self.grid.valid(c) and free[c] for c in cells):continue
            if not safe[cells[-1]]:continue
            if sign<0 and (run<.05 or self.reverse_room(run+.01)<run):continue
            distances=np.linalg.norm(cloud[None,:,:]-np.c_[offsets,np.zeros(len(offsets))][:,None,:],axis=2).min(axis=1)
            if distances.min()<minimum-.003 or distances[-1]<DRIVE_CLEARANCE+.025:continue
            return sign
        return None
    def begin_clearance_escape(self,stamp):
        p=self.localizer.pose[:2]
        if getattr(self,'margin_anchor',None) is None or np.linalg.norm(p-self.margin_anchor)>.40:
            self.margin_anchor=p.copy();self.margin_attempts=0
        if self.margin_attempts>=3:
            # Re-entering the same margin is not progress. Do not reset this
            # budget on a plan/follow transition or a tiny reverse movement.
            if self.begin_retreat(stamp):return True
            self.skip_goal('Repeated clearance recovery made no progress.',stamp);return True
        sign=self.clearance_escape()
        if sign is None:
            heading=self.margin_exit_heading()
            if heading is None:return False
            self.margin_attempts+=1;self.escape_heading=heading;self.phase='escape_turn';self.phase_started=stamp;self.path=[]
            self.event('Turning toward measured clear space before leaving the new obstacle margin.');return True
        self.margin_attempts+=1;self.escape_sign=sign;self.escape_origin=self.localizer.pose.copy();self.phase='escape_margin';self.phase_started=stamp;self.path=[]
        self.event('New obstacle margin surrounds the current position. Moving slowly into measured clear space before replanning.');return True
    def margin_exit_heading(self):
        p=self.localizer.pose;free,_,_,safe,clearance=self.grid.masks();start=self.grid.cell(p[:2]);cloud=self.obstacle_points()
        if not self.grid.valid(start) or not free[start] or len(cloud)<60:return None
        minimum=float(np.linalg.norm(cloud,axis=1).min())
        # The turn is about base_link: the circle rule (same TURN_CLEAR as turn_clear,
        # rotation_clear) decides it for every angle; no per-angle sweep is needed.
        if not self.spin_clear():return None
        run=DRIVE_CLEARANCE
        for delta in sorted(np.arange(-math.pi,math.pi+.01,math.pi/12),key=abs):
            if abs(delta)<.05:continue
            # Base_link after the turn (stays put when AXLE_X=0).
            shift=self.predicted_centres(0.,delta,np.array([1.]))[0]
            direction=np.array([math.cos(p[2]+delta),math.sin(p[2]+delta)])
            origin=p[:2]+np.array([[math.cos(p[2]),-math.sin(p[2])],[math.sin(p[2]),math.cos(p[2])]])@shift
            line=origin+np.linspace(0,run,21)[:,None]*direction;cells=[self.grid.cell(xy) for xy in line]
            if not all(self.grid.valid(c) and free[c] for c in cells) or not safe[cells[-1]]:continue
            offsets=shift+np.linspace(0,run,21)[:,None]*np.array([math.cos(delta),math.sin(delta)])
            distances=np.linalg.norm(cloud[None,:,:]-offsets[:,None,:],axis=2).min(axis=1)
            if distances.min()>=minimum-.003 and distances[-1]>DRIVE_CLEARANCE+.025:return wrap(p[2]+delta)
        return None
    def straight_corridor_clear(self,sign,distance,previously_traversed=False):
        p=self.localizer.pose;free=self.grid.masks()[0];offsets=np.linspace(0,distance,16)
        line=p[:2]+sign*offsets[:,None]*np.array([math.cos(p[2]),math.sin(p[2])])
        if any(not self.grid.valid(self.grid.cell(xy)) or not free[self.grid.cell(xy)] for xy in line):return False
        # Rear low obstacles are invisible to the forward camera and the LiDAR
        # (scan plane geometry.LIDAR_MOUNT[2], 19 cm), so reversing may only
        # retrace ground the body has covered.
        if previously_traversed and self.reverse_room(distance+.005)<distance-.005:return False
        cloud=self.obstacle_points()
        if len(cloud)<60:return False
        gaps=np.linalg.norm(cloud[None,:,:]-sign*np.c_[offsets,np.zeros(len(offsets))][:,None,:],axis=2).min(axis=1)
        # Moving away from something already close (a bump) is allowed; no
        # sample may come closer than now, nor closer than DRIVE_CLEARANCE otherwise.
        return bool((gaps>=min(DRIVE_CLEARANCE,gaps[0]-.003)).all() and gaps[-1]>=min(DRIVE_CLEARANCE,gaps[0]))
    def retreat_clear(self,distance=.08):
        return self.straight_corridor_clear(-1,distance,True)
    def begin_retreat(self,stamp,replan=False):
        p=self.localizer.pose
        if getattr(self,'stall_position',None) is None or np.linalg.norm(p[:2]-self.stall_position)>.5:self.stall_attempts=0
        self.retry_straight=not replan and self.straight_corridor_clear(1,.30)
        # Never further than the verified driven trail allows, and at most 15 cm.
        self.retreat_distance=min(.15 if replan else .035 if self.retry_straight else REVERSE_LIMIT,self.reverse_budget(),self.reverse_room(REVERSE_LIMIT+.03)-.025)
        if self.stall_attempts>=2 or self.retreat_distance<.03 or not self.retreat_clear(self.retreat_distance+.025):return False
        self.stall_attempts+=1;self.stall_position=p[:2].copy();self.escape_origin=p.copy();self.phase='retreat';self.phase_started=stamp;self.path=[];self.turn_stall_since=None
        self.event('Movement stalled. Making a short measured retreat, then retrying the clear corridor without turning.' if self.retry_straight else 'Movement stalled with an obstacle ahead. Backing into the previously traversed corridor to replan.');return True
    def note_jam(self,p,stamp):
        self.jams.append(p[:2].copy());self.jam_reset=stamp
        if sum(np.linalg.norm(j-p[:2])<.3 for j in self.jams)>=3:
            if not self.mark_stall(p,stamp):self.skip_goal('Wheels repeatedly slip at this spot.',stamp)
            return False
        self.event('Wheels turn but LiDAR shows no motion; probably caught on an edge.');return True
    def mark_stall(self,p,stamp):
        """A forward drive the wheels cannot push through (jammed: wheels turn, the LiDAR shows no motion) with the
        bumper released and no LiDAR return at the body is an obstacle no sensor sees: an overhang whose underside lies
        between the LiDAR plane (LIDAR_MOUNT z) and the robot top, which only the LiDAR turret reaches (a sofa or bed
        frame at ~15 cm), or something caught under the body. Called when note_jam gives up on the spot: the session has
        recorded three jams within .3 m of it. Those may be this spot's own jam and its two straight retries, or fewer
        here plus earlier jams nearby, so a stall next to an earlier marked or skipped spot is marked on its first jam.
        It is then handled as a bumper contact in an unknown front zone: the same marks at the body edge (note_contact),
        the contact phase's short reverse along the driven trail (never off it) and a replan, which now routes around
        the marks. Unlike a bumper contact it does not count toward the 'repeated contacts' stop; STALL_MARK_LIMIT caps
        the marks (and so the pushes) per session: the last one is marked, then the session stops with a message. Returns
        False (nothing marked; the caller skips the target as before) for a jam while turning or reversing, with the
        bumper pressed, when a LiDAR return at the body explains it, or where a stall mark already lies within
        STALL_MARK_SPACING."""
        if self.bumper_pressed or not getattr(self,'jam_forward',False):return False
        if any(np.linalg.norm(q-p[:2])<STALL_MARK_SPACING for q in self.stall_marks):return False
        lidar=self.live_lidar_points()
        if len(lidar) and ((np.linalg.norm(lidar,axis=1)<R+.01)&(lidar[:,0]>0)).any():return False   # the LiDAR explains it: already mapped
        self.grid.add_camera_obstacles(transform(contact_marks([1,2,3]),p),margin=0);self.stall_marks.append(p[:2].copy());self.trouble.append(p[:2].copy())
        self.event('Pushing forward does not move the robot here, yet the bumper is released and the LiDAR sees nothing: '
                   'probably an overhang above the LiDAR beam or something under the body.')
        if len(self.stall_marks)>=STALL_MARK_LIMIT:
            # Turret safety: no more pushes into what no sensor sees. The marks stay on the saved (partial) map.
            self.idle(f'Stopped: blocked at {len(self.stall_marks)} spots this session where the bumper and the LiDAR sense '
                      'nothing (probably furniture whose underside is above the LiDAR beam, which only the LiDAR turret '
                      'touches). The spots are marked on the partial map; check them before mapping again.');return True
        self.start_contact_phase(stamp,'stall, nothing sensed',[],max_backoff=STALL_BACKOFF);return True
    def begin_jam_settle(self,stamp,reason):
        """A jam: stop and stand still JAM_SETTLE_S (phase 'jam_settle') before the usual handling (note_jam, then a
        measured retreat and straight retry, or skip the target), so it starts from a settled pose estimate."""
        self.jam_forward=self.requested_motion[0]>0;self.jam_reason=reason;self.jam_from_phase=self.phase
        self.phase='jam_settle';self.phase_started=stamp;self.maneuver=None
    def handle_turn_stall(self,stamp):
        # Verify the pose against the LiDAR map before a bounded recovery.
        self.turn_stall_since=None;self.requested_motion=(0.,0.);self.localizer.invalidate();self.localizer.pending=None;self.localizer.relocalization_hits=0
        self.retreat_after_localization=True;self.retreat_requires_replan=False;self.phase='recover_localization';self.phase_started=stamp;self.path=[]
        self.event('Turning stalled. Stopped to verify position before attempting a clear retreat.')
    def translation_stalled(self,stamp):
        # Heading oscillation, wheel ticks and repeated replans are not forward
        # progress. Allow a full initial turn before requiring displacement.
        p=self.localizer.pose[:2]
        # A close spin (slow turn in place, up to ~15 s) is legitimate progress, not a stall.
        m=getattr(self,'maneuver',None)
        if m and m.get('kind')=='close_spin':self.translation_since=stamp
        if getattr(self,'translation_anchor',None) is None or np.linalg.norm(p-self.translation_anchor)>.08:
            self.translation_anchor=p.copy();self.translation_since=stamp
        return stamp-self.translation_since>12
    def passage_width(self,ahead=.10,window=.25):
        """Measured width (m) of the passage around the point `ahead` m in front of base_link:
        nearest raw obstacle point on the left plus on the right of the heading line, within
        `window` m along it (single scan: includes the range noise)."""
        cloud=self.obstacle_points()
        if not len(cloud):return math.inf
        near=cloud[np.abs(cloud[:,0]-ahead)<=window];left=near[near[:,1]>0,1];right=-near[near[:,1]<0,1]
        return float((left.min() if len(left) else math.inf)+(right.min() if len(right) else math.inf))
    def prune_narrow_blocks(self,stamp):
        times=getattr(self.grid,'narrow_block_times',None)
        if not times or len(times)!=len(self.grid.narrow_blocks):return
        keep=[i for i,t in enumerate(times) if stamp-t<NARROW_BLOCK_S]
        if len(keep)<len(times):self.grid.narrow_blocks=[self.grid.narrow_blocks[i] for i in keep];self.grid.narrow_block_times=[times[i] for i in keep]
    def plan(self,stamp):
        self.prune_narrow_blocks(stamp)
        pose=self.localizer.pose
        if getattr(self,'retreat_after_localization',False):
            self.retreat_after_localization=False
            if self.begin_retreat(stamp,getattr(self,'retreat_requires_replan',False)):return
            if self.mode=='go_to':self.idle('Turning blocked and no verified retreat available. Partial progress saved.');return
            self.skip_goal('Turning was blocked and there is no verified path behind.',stamp);return
        # A robot inside an obstacle margin is not special-cased: the planner
        # leaves it along free cells that get no closer to anything, along its
        # own driven trail, or at the narrow-passage clearance (grid.traversable).
        self.review_frontier_target()
        # Narrow-passage clearance (and its slow speed) serves one target only.
        self.restore_radius()
        if self.mode=='go_to':
            route_goal=self.goal
            # A contact ended the final approach: the Go-to ends where the contact handling left the robot.
            if getattr(self,'approach_contact',False):self.finish_approach(stamp,'contact');return
            if getattr(self,'near_goal',False):
                self.staging=self.staging_cell(pose)
                if self.staging is None:self.idle('No safe path to that position in the saved map.');return
                if self.on_approach_line(pose):self.begin_approach(stamp);return
                route_goal=self.staging
            self.path=self.grid.path(pose[:2],route_goal)
            if not self.path:self.idle('No safe path to that position in the saved map.');return
            self.event('Following the planned path with live LiDAR position correction.' if route_goal is self.goal else
                       'Following the planned path to the closest safe point beside the target, with live LiDAR position correction.')
        else:
            candidates=self.grid.frontiers(pose,self.blacklist+self.scanned_positions)
            if not candidates and self.grid.radius>NARROW_RADIUS:
                # Unseen space outranks revisiting: try frontiers behind a narrow
                # gap (20 cm clearance, slow) before any coverage viewpoint.
                normal=self.grid.radius;self.grid.radius=NARROW_RADIUS
                candidates=self.grid.frontiers(pose,self.blacklist+self.scanned_positions)
                if candidates:self.normal_radius=normal;self.event('Unexplored space lies behind a narrow gap. Planning through it with 20 cm clearance, slowly.')
                else:self.grid.radius=normal
            kind='frontier'
            if not candidates:
                candidates=self.coverage_viewpoints(pose);kind='coverage'
            if not candidates and self.grid.radius>NARROW_RADIUS:
                # Known open floor cut off from the robot is often behind a gap the
                # 25 cm planning clearance closes (a door beside a chair). Plan
                # through it once with 20 cm, driving slowly, then return to 25 cm.
                safe=self.grid.masks()[3]
                if (safe&~self.grid.reachable(pose[:2],safe)).sum()*self.grid.res**2>1:
                    normal=self.grid.radius;self.grid.radius=NARROW_RADIUS
                    candidates=self.grid.frontiers(pose,self.blacklist+self.scanned_positions);kind='frontier'
                    if not candidates:candidates=self.coverage_viewpoints(pose);kind='coverage'
                    if candidates:self.normal_radius=normal;self.event('Remaining space lies behind a narrow passage. Planning through it with 20 cm clearance, slowly.')
                    else:self.grid.radius=normal
            if not candidates:
                self.no_frontier_scans+=1
                if self.no_frontier_scans<3:self.phase='sense';self.phase_started=stamp;self.event('Checking remaining uncertainty before finishing.');return
                self.finish_mapping('No further reachable observation viewpoints');return
            _,self.goal,self.path=max(candidates,key=lambda x:x[0]);self.goal_kind=kind;self.no_frontier_scans=0;self.event('Exploring a viewpoint chosen from the LiDAR map.')
            self.frontier_target=self.unknown_edge(self.goal) if kind=='frontier' else None
        self.phase='follow';self.phase_started=stamp;self.next_plan=stamp+1;self.last_progress=stamp;self.progress_pose=pose.copy()
    def mapping_complete(self):
        # Skipped targets are reported in the manifest; they do not hide or
        # replace any frontier or unvisited space the robot can still reach.
        pose=self.localizer.pose;safe=self.grid.masks()[3];reachable=self.grid.reachable(pose[:2],safe)
        # A robot boxed in by its own margins must never report the house done:
        # only small pockets (behind furniture, inside a tray) may stay unreached.
        if (safe&~reachable).sum()*self.grid.res**2>1.5 or reachable.sum()*self.grid.res**2<1:return False
        if self.stuck():return False
        return not self.grid.frontiers(pose,self.blacklist+self.scanned_positions+self.trouble) and self.localizer.uncertainty<.08 and self.grid.summary()['free_area_m2']>1 and self.unvisited_area()<UNVISITED_DONE
    def stuck(self):
        """Two or more skipped targets in a row from this very spot."""
        a=getattr(self,'stuck_anchor',None)
        return getattr(self,'stuck_skips',0)>=2 and a is not None and np.linalg.norm(self.localizer.pose[:2]-a)<.05
    def finish_mapping(self,reason):
        if getattr(self,'normal_radius',None):self.grid.radius=self.normal_radius;self.normal_radius=None
        complete=bool(self.mapping_complete());self.save_map(complete,reason);self.complete=complete;self.mode='idle';self.phase='idle';self.path=[];self.wants_arm=False
        self.event('Mapping finished for observed reachable space.' if complete else 'Mapping stopped with unvisited or unreachable observed areas. Partial map saved.')
    def unknown_edge(self,goal):
        """Unknown cells bordering measured free floor within the frontier gain
        window (31 x 31 cells) of a chosen viewpoint."""
        free,_,unknown,_,_=self.grid.masks();y,x=self.grid.cell(goal);r=15
        window=np.zeros_like(unknown);window[max(0,y-r):y+r+1,max(0,x-r):x+r+1]=True
        near_free=cv2.dilate(free.astype(np.uint8),np.ones((5,5),np.uint8))>0
        return np.argwhere(unknown&window&near_free)
    def review_frontier_target(self):
        """After an attempt on a frontier (arrival and scan, lost route or skip):
        if most of its unknown edge is still unknown, that attempt did not help.
        Cells two attempts failed to reveal stop generating frontiers."""
        cells=getattr(self,'frontier_target',None);self.frontier_target=None
        if cells is None or not len(cells):return
        unknown=self.grid.masks()[2];left=unknown[cells[:,0],cells[:,1]]
        if left.mean()<.6:return
        if getattr(self,'failed_views',None) is None or self.failed_views.shape!=unknown.shape:self.failed_views=np.zeros(unknown.shape,np.uint8)
        stuck=cells[left];self.failed_views[stuck[:,0],stuck[:,1]]+=1
        self.grid.unobservable=self.failed_views>=2
    def coverage_goal_covered(self):
        if getattr(self,'goal_kind',None)!='coverage' or self.goal is None:return False
        cell=self.grid.cell(self.goal)
        return self.grid.valid(cell) and not self.unvisited_mask()[cell]
    def route_lost(self,stamp):
        """The route to the target closed (often thin chair legs that the LiDAR
        resolves only up close). Choose again; a target lost twice is left, so the
        robot does not alternate across the house between two such targets."""
        g=self.goal.copy();self.lost_routes=getattr(self,'lost_routes',[])+[g]
        if self.mode=='mapping' and self.grid.radius>NARROW_RADIUS:
            # Closed at the normal clearance, usually at a doorway or beside furniture seen up
            # close: before crossing the house to another target, try this one through the gap
            # at the narrow-passage clearance (slowly). If that route closes too, the target
            # has been lost twice and is left below.
            normal=self.grid.radius;self.grid.radius=NARROW_RADIUS;path=self.grid.path(self.localizer.pose[:2],g)
            if path:
                self.normal_radius=normal;self.path=path;self.phase='follow';self.phase_started=stamp;self.next_plan=stamp+1;self.last_progress=stamp;self.progress_pose=self.localizer.pose.copy();self.path_failed_since=None
                self.event('The route to this target closed at the normal clearance. Continuing through the narrow gap with 20 cm clearance, slowly.');return
            self.grid.radius=normal
        if self.mode=='mapping' and sum(np.linalg.norm(x-g)<.3 for x in self.lost_routes)>=2:
            self.blacklist.append(g);self.event('The route to this target closed again on approach. Leaving it and mapping elsewhere.')
        else:self.event('The route to this target closed. Choosing the next target.')
        self.path=[];self.phase='plan';self.plan_not_before=stamp+.5
    def restore_radius(self):
        """Leave narrow-passage mode, but only where the normal clearance fits."""
        if not getattr(self,'normal_radius',None):return
        narrow=self.grid.radius;self.grid.radius=self.normal_radius;cell=self.grid.cell(self.localizer.pose[:2])
        if self.grid.valid(cell) and self.grid.masks()[3][cell]:self.normal_radius=None
        else:self.grid.radius=narrow
    def skip_goal(self,reason,stamp):
        """Give up on the current target, not the session; remember the spot."""
        p=self.localizer.pose;self.restore_radius()
        if self.mode!='mapping':self.idle(reason+' Stopped.');return
        # Repeated skips without leaving one spot mean the robot is stuck, not that targets in
        # other rooms are unreachable: keep those targets, and do not report the map complete.
        anchor=getattr(self,'stuck_anchor',None)
        if anchor is not None and np.linalg.norm(p[:2]-anchor)<.05:self.stuck_skips+=1
        else:self.stuck_anchor=p[:2].copy();self.stuck_skips=1
        stuck=self.stuck_skips>=2
        if self.goal is not None and not (stuck and np.linalg.norm(self.goal-p[:2])>1.):self.blacklist.append(self.goal.copy())
        self.trouble.append(p[:2].copy());free=self.grid.summary()['free_area_m2']
        # Budget: consecutive skips that add no newly mapped area end the run.
        self.skips=1 if free>self.coverage_mark+.5 else self.skips+1;self.coverage_mark=max(self.coverage_mark,free)
        self.stall_position=None;self.stall_attempts=0;self.margin_anchor=None;self.margin_attempts=0;self.translation_anchor=None;self.retreat_after_localization=False
        self.path=[];self.phase='plan';self.phase_started=stamp;self.plan_not_before=stamp+.5;self.last_progress=stamp;self.progress_pose=p.copy();self.jam_reset=stamp
        # Every skip blacklists its target, so candidates run out; the budget
        # only bounds a long run of skips that adds no newly mapped area.
        if self.skips>20:self.finish_mapping(reason+' Many targets in a row could not be reached.');return
        if stuck:
            # Phantom blocking is often a pose error: verify the pose before trying again.
            loc=self.localizer;loc.invalidate();loc.pending=None;loc.relocalization_hits=0;self.phase='recover_localization';self.phase_started=stamp
            self.event(reason+' No progress from this spot on repeated targets; verifying position before trying again.');return
        self.event(reason+' Skipping this target and choosing another.')
    def motion_mask(self):
        free,_,_,safe,clearance=self.grid.masks();return self.grid.traversable(self.grid.cell(self.localizer.pose[:2]),free,safe,clearance)
    def obstacle_points(self,core=False):
        # core=True: measured camera/contact cells only (no planning margin),
        # for exact chassis-sweep checks of turns in place.
        lidar=self.current_cloud if self.current_cloud.shape[1:]==(2,) else np.empty((0,2));cells=np.argwhere(self.grid.camera_core if core else self.grid.camera_obstacles)
        if not len(cells):return lidar
        world=np.array([self.grid.world(c) for c in cells]);world=world[np.linalg.norm(world-self.localizer.pose[:2],axis=1)<1.5]
        return np.vstack((lidar,inverse(world,self.localizer.pose)))
    def note_spin_scan(self,points,pose,stamp):
        """Keep the near returns of each scan (world frame) for the per-beam average in spin_points."""
        pts=np.asarray(points,float).reshape(-1,2);near=pts[np.hypot(pts[:,0],pts[:,1])<SPIN_NEAR]
        self.spin_scans.append((float(stamp),np.asarray(pose,float).copy(),transform(near,pose) if len(near) else np.empty((0,2))))
    def current_scan_pose(self):
        """Pose at which current_cloud was measured (the localizer pose before the first scan)."""
        p=getattr(self,'scan_pose',None);return self.localizer.pose if p is None else p
    def still_scans(self):
        """Earlier scans taken from this spot with this heading: scan pose within SPIN_STILL_M and
        SPIN_STILL_YAW of the current scan's pose, newer than SPIN_WINDOW_S. Scans taken while
        turning are excluded: their beams lie on other lines and could not be averaged anyway."""
        p=self.current_scan_pose();now=self.frame.stamp if getattr(self,'frame',None) is not None else None
        if now is None:return []
        return [(t,q,w) for t,q,w in getattr(self,'spin_scans',()) if t!=now and now-t<=SPIN_WINDOW_S and np.linalg.norm(q[:2]-p[:2])<SPIN_STILL_M and abs(wrap(q[2]-p[2]))<SPIN_STILL_YAW]
    def spin_scan_count(self):
        """Number of scans averaged into the circle-rule distances (1 = the current scan alone)."""
        return 1+len(self.still_scans())
    def spin_points(self):
        """Points for the circle rule (turn_clear): the current LiDAR returns, each near one
        (within SPIN_NEAR) replaced by the mean range ALONG ITS OWN BEAM over the scans taken
        from this spot (still_scans): an earlier return counts when it lies within .5 mm of
        the current beam line (from the LiDAR mount) and within 2 cm along it, i.e. it is the
        same beam, or a beam hitting the same surface spot, measured again. Range noise is
        along the beam, so this average is unbiased for any surface shape, unlike averaging
        neighbouring beams, which pushes the closest point of a corner or post outward. The
        .5 mm lateral gate bounds what a sharp (60 deg) wedge can add to < 1 mm. Measured
        camera/contact cells are added raw (no planning margin). Earlier returns are expressed
        in the frame of the CURRENT SCAN's pose (current_scan_pose), the frame current_cloud is in."""
        p=self.localizer.pose;sp=self.current_scan_pose();lidar=self.current_cloud if self.current_cloud.shape[1:]==(2,) else np.empty((0,2))
        key=(getattr(self.frame,'stamp',None) if getattr(self,'frame',None) is not None else None,tuple(np.round(p,5)),tuple(np.round(sp,6)),len(lidar),id(self.current_cloud),int(self.grid.camera_core.sum()),len(getattr(self,'spin_scans',())))
        if getattr(self,'spin_cache_key',None)==key:return self.spin_cache
        lidar=np.asarray(lidar,float).copy();d=np.hypot(lidar[:,0],lidar[:,1]) if len(lidar) else np.empty(0);near=np.flatnonzero(d<SPIN_NEAR)
        history=self.still_scans() if len(near) else []
        if history:
            origin=np.asarray(getattr(self.grid,'sensor_xy',LIDAR_MOUNT[:2]),float)
            ray=lidar[near]-origin;t0=np.linalg.norm(ray,axis=1);u=ray/np.maximum(t0,1e-9)[:,None]
            old=[inverse(world,sp) for _,_,world in history if len(world)]
            if old:
                q=np.vstack(old)-origin                                   # earlier returns, from the mount
                along=u@q.T;lateral=np.abs(u[:,0:1]*q[None,:,1]-u[:,1:2]*q[None,:,0])
                same=(lateral<=.0005)&(np.abs(along-t0[:,None])<.02)
                mean=(t0+(same*along).sum(axis=1))/(1+same.sum(axis=1))
                lidar[near]=origin+mean[:,None]*u
        cells=np.argwhere(self.grid.camera_core)
        if len(cells):
            world=np.array([self.grid.world(c) for c in cells]);world=world[np.linalg.norm(world-p[:2],axis=1)<1.5]
            if len(world):lidar=np.vstack((lidar,inverse(world,p)))
        self.spin_cache_key=key;self.spin_cache=lidar;return lidar
    def spin_rule(self):
        """(points, margin) for the circle rule of a pure spin from here: turn_clear(points,
        margin=margin). Normally the beam-averaged spin_points with the full TURN_CLEAR
        (margin None); each pass records the spot (spin_commit). While the robot is executing
        a pure spin (last requested motion v == 0, w != 0) within SPIN_DRIFT of the last spot
        where the full rule passed, the margin is TURN_MARGIN - SPIN_RAW_ALLOWANCE (hysteresis,
        see SPIN_RAW_ALLOWANCE)."""
        points=self.spin_points();here=self.current_scan_pose()[:2]
        if turn_clear(points):self.spin_commit=here.copy();return points,None
        c=getattr(self,'spin_commit',None);v,w=getattr(self,'requested_motion',(0.,0.))
        if c is not None and v==0 and abs(w)>1e-6 and np.linalg.norm(here-c)<=SPIN_DRIFT:return points,TURN_MARGIN-SPIN_RAW_ALLOWANCE
        return points,None
    def spin_clear(self):
        points,margin=self.spin_rule();return turn_clear(points,margin=margin)
    def rotation_clear(self):
        # Same circle rule (TURN_CLEAR about base_link) as turn_clear, on beam-averaged points.
        return len(self.obstacle_points())>=60 and self.spin_clear()
    @staticmethod
    def predicted_centres(v,w,times):
        """base_link path for a (v,w) command. Commands refer to the drive axle,
        AXLE_X ahead of base_link; the round robot has AXLE_X=0, so this is the
        plain unicycle arc and a pure turn leaves base_link where it is."""
        times=np.asarray(times,float);angle=w*times
        axle=np.c_[v*times,np.zeros(len(times))] if abs(w)<1e-6 else np.c_[v/w*np.sin(angle),v/w*(1-np.cos(angle))]
        return axle+AXLE_X*np.c_[1-np.cos(angle),-np.sin(angle)] if AXLE_X else axle
    @staticmethod
    def cells_clear(mask,cells):
        # Adjacent samples can cross a cell corner between sample times.
        # Include both bordering cells, as A* does for diagonal steps.
        return bool(mask[cells[:,0],cells[:,1]].all() and mask[cells[:-1,0],cells[1:,1]].all() and mask[cells[1:,0],cells[:-1,1]].all())
    def velocity_still_clear(self,v,w,duration):
        p=self.localizer.pose;cloud=self.obstacle_points()
        if len(cloud)<60:return False
        relative=self.predicted_centres(v,w,np.linspace(0,duration,max(2,int(duration/.025)+1)))
        c,s=math.cos(p[2]),math.sin(p[2]);world=relative@np.array([[c,s],[-s,c]])+p[:2]
        cells=np.array([self.grid.cell(xy) for xy in world])
        if any(not self.grid.valid(tuple(cell)) for cell in cells):return False
        if v==0 and w:
            # Pure spin about base_link: no distance changes, so the circle rule
            # alone decides (every measured point >= TURN_CLEAR), whatever the
            # 20 cm driving circle says; the body stays on its own measured free floor.
            return self.spin_clear() and self.cells_clear(self.grid.masks()[0],cells)
        gaps=np.linalg.norm(cloud[None,:,:]-relative[:,None,:],axis=2).min(axis=1)
        # Driving: never closer than DRIVE_CLEARANCE, or than now when already closer.
        if (gaps<min(DRIVE_CLEARANCE,gaps[0]-.003)).any():return False
        if not self.cells_clear(self.motion_mask(),cells):return False
        if v<0 and not self.on_trail(world):return False
        return True
    def turn_escape(self,target):
        """Fallback when the turn toward the route is blocked both ways (a point inside TURN_CLEAR).
        Tried in order, and none of them ever reverses off the driven trail:
        1. a clear partial turn followed by a short straight FORWARD drive that never approaches
           anything and ends where the turn toward the route is clear. On the round robot
           (AXLE_X=0) the circle rule does not depend on the angle, so only k=0 (drive straight
           away from what blocks the spin, then turn) can ever help; the k>=1 search stops at once;
        2. a close spin: turn in place SLOWLY when nothing touches (close_spin_maneuver); a pure
           spin changes no distance, so this is the normal way out of a nose-in corner;
        3. the shortest clear arc that gains spin room: forward while the bumper is released, or
           reversing ALONG THE DRIVEN TRAIL only (spin_room_escape).
        Returns a maneuver or None."""
        p=self.localizer.pose;cloud=self.obstacle_points();core=self.spin_points()
        if len(cloud)<60:return None
        mask=self.motion_mask();best=None
        for direction in (1.,-1.):
            for k in range(32):
                a=direction*.1*k
                if k and not turn_clear(core):break
                # Turning in place pivots about the axle; base_link shifts by AXLE_X terms (0 here).
                c,s=math.cos(a),math.sin(a);shift=np.array([AXLE_X-AXLE_X*c,-AXLE_X*s]);back=np.array([[c,-s],[s,c]])
                turned=(cloud-shift)@back;turned_core=(core-shift)@back;limit=min(DRIVE_CLEARANCE,float(np.linalg.norm(turned,axis=1).min())-.003)
                h=p[2]+a;u=np.array([math.cos(h),math.sin(h)]);origin=p[:2]+np.array([[math.cos(p[2]),-math.sin(p[2])],[math.sin(p[2]),math.cos(p[2])]])@shift
                for d in np.arange(.05,.41,.05):
                    line=origin+np.linspace(0,d,int(round(d/.01))+1)[:,None]*u
                    cells=np.array([self.grid.cell(xy) for xy in line])
                    if any(not self.grid.valid(tuple(c_)) for c_ in cells) or not self.cells_clear(mask,cells):break
                    gaps=np.linalg.norm(turned[None,:,:]-np.c_[np.linspace(0,d,len(line)),np.zeros(len(line))][:,None,:],axis=2).min(axis=1)
                    if gaps.min()<limit:break
                    error=wrap(math.atan2(target[1]-line[-1][1],target[0]-line[-1][0])-h)
                    if abs(error)>1.2:
                        here=turned_core-[d,0.]
                        if not turn_clear(here):continue
                    cost=d+.1*abs(a)
                    if best is None or cost<best['cost']:best=dict(cost=cost,turn=a,distance=float(d),heading0=float(p[2]))
                    break
        if best is not None:return best
        return self.close_spin_maneuver(target) or self.spin_room_escape()
    def close_spin_clear(self,direction,points=None):
        """Close-spin rule (CLOSE_SPIN_W) for a slow pure spin in `direction` (sign of w), allowed
        although a point lies inside TURN_CLEAR because nothing touches. Every beam-averaged point
        (spin_points) must be at least CLOSE_SPIN_CLEAR (R + 3 mm) from base_link, except, while the
        bumper skin is released (it trips on contact: a direct measurement where it covers):
          * a point in the skin's front half (|azimuth| <= BUMPER_SPAN_DEG), and
          * a point at most CLOSE_SPIN_ENTRY_DEG behind a skin end that this spin direction turns
            INTO the front half (it reaches the skin first, so a touch would be felt at once);
        each provided its LiDAR average is not more than CLOSE_SPIN_FRONT_SLACK inside the skin (an
        object the skin cannot feel). Any other rear point (unsensed rigid shell) needs the full
        R + 3 mm. Needs SPIN_SCANS_MIN averaged scans taken from this spot and heading."""
        if self.spin_scan_count()<SPIN_SCANS_MIN:return False
        points=self.spin_points() if points is None else np.asarray(points,float).reshape(-1,2)
        if not len(points):return True
        d=np.hypot(points[:,0],points[:,1]);close=d<CLOSE_SPIN_CLEAR
        if not close.any():return True
        if self.bumper_pressed:return False
        az=np.degrees(np.arctan2(points[close,1],points[close,0]))*(1. if direction>0 else -1.)
        # Mirrored so the spin moves every point toward smaller az: entering the front from az > 90.
        sensed=(np.abs(az)<=BUMPER_SPAN_DEG)|((az>BUMPER_SPAN_DEG)&(az<=BUMPER_SPAN_DEG+CLOSE_SPIN_ENTRY_DEG))
        return bool((sensed&(d[close]>=R-CLOSE_SPIN_FRONT_SLACK)).all())
    def close_spin_contact(self):
        """LiDAR evidence of overlap during a close spin: CLOSE_SPIN_CONTACT_POINTS raw returns (single
        scan, or measured camera / contact cells) more than the single-scan range noise
        SPIN_RAW_ALLOWANCE (3 sigma) inside the body: inside the skin R in the front half, inside the
        rigid shell R_RIGID behind it. A pure spin changes no distance, so only something new (a
        moving object) or a pose error can cause this. It does NOT detect a touch of a few mm (below
        the range noise); the bumper does that in front and only a spin stall (close_spin_stalled)
        reveals it behind (the rear has no contact sensor)."""
        raw=self.obstacle_points(core=True)
        if not len(raw):return False
        d=np.hypot(raw[:,0],raw[:,1]);front=np.abs(np.degrees(np.arctan2(raw[:,1],raw[:,0])))<=BUMPER_SPAN_DEG
        return int((d<np.where(front,R,R_RIGID)-SPIN_RAW_ALLOWANCE).sum())>=CLOSE_SPIN_CONTACT_POINTS
    def close_spin_stalled(self,m,stamp):
        """The gyro shows the commanded close spin is not turning (|gyro z| < CLOSE_SPIN_STALL_FRACTION
        of the command for CLOSE_SPIN_STALL_S): the body is held, e.g. a rigid-shell touch."""
        if stamp is None:return False
        if abs(getattr(self,'gyro_z',0.))>=CLOSE_SPIN_STALL_FRACTION*CLOSE_SPIN_W:m['slow_since']=None;return False
        if m.get('slow_since') is None:m['slow_since']=stamp
        return stamp-m['slow_since']>CLOSE_SPIN_STALL_S
    def close_spin_retry_blocked(self,direction):
        """True when a close spin in `direction` ended in a contact or stall at this spot recently."""
        b=getattr(self,'close_spin_blocked',None);now=getattr(self.frame,'stamp',None) if getattr(self,'frame',None) is not None else None
        if not b or b['direction']!=direction:return False
        return np.linalg.norm(self.localizer.pose[:2]-b['xy'])<CLOSE_SPIN_RETRY_M and (now is None or now-b['stamp']<CLOSE_SPIN_RETRY_S)
    def close_spin_maneuver(self,target):
        """A slow turn in place (|w| = CLOSE_SPIN_W) when TURN_CLEAR refuses the spin but
        close_spin_clear allows it, followed by a short straight forward exit. The turn ends at the
        first heading (5 deg steps) from which a straight forward drive of at most CLOSE_SPIN_EXIT
        never comes closer to any averaged point and ends with spin room (every averaged point >=
        TURN_CLEAR + 4 mm; where no heading gives that, 1 cm further than now) on measured free
        floor; after the exit the normal planner (normal-speed turns) takes over. Direction: prefer
        one that keeps the close points (inside TURN_CLEAR) in the bumper's front half for most
        (>= half) of the turn, where a touch would be sensed; among those (or if neither does) the
        shorter turn. While pressed only a direction the firmware allows (spin_allowed). `target`
        is unused (the route is re-planned with spin room after the exit)."""
        p=self.localizer.pose;points=self.spin_points()
        if not len(points):return None
        d=np.hypot(points[:,0],points[:,1]);close=points[d<TURN_CLEAR];az=np.arctan2(close[:,1],close[:,0])
        mask=self.motion_mask();run=np.linspace(0,CLOSE_SPIN_EXIT,int(round(CLOSE_SPIN_EXIT/.005))+1);start=float(d.min())
        allowed=[s_ for s_ in (1.,-1.) if self.spin_allowed(s_) and self.close_spin_clear(s_,points)]
        for need in (TURN_CLEAR+.004,start+.01):
            options=[]
            for direction in allowed:
                for theta in np.radians(np.arange(5,360,5)):
                    a=direction*theta;line=run[:,None]*np.array([math.cos(a),math.sin(a)])
                    gaps=np.linalg.norm(points[None,:,:]-line[:,None,:],axis=2).min(axis=1)
                    if (gaps<start-.001).any() or gaps[-1]<need:continue
                    k=max(6,int(np.argmax(gaps>=need)));line=line[:k+1]      # the shortest exit that gains the room (>= 3 cm)
                    cells=np.array([self.grid.cell(xy) for xy in transform(line,p)])
                    if any(not self.grid.valid(tuple(c_)) for c_ in cells) or not self.cells_clear(mask,cells):continue
                    sweep=np.radians(np.arange(0,math.degrees(theta)+1e-6,2.))
                    rel=(az[None,:]-direction*sweep[:,None]+math.pi)%(2*math.pi)-math.pi
                    front=float((np.abs(np.degrees(rel))<=BUMPER_SPAN_DEG).mean()) if len(az) else 1.
                    exit_=float(run[k])
                    options.append(dict(kind='close_spin',direction=direction,turn=float(a),front=front,distance=exit_,heading0=float(p[2]),
                                        turned=0.,last=float(p[2]),timeout=float(theta/CLOSE_SPIN_W+exit_/CLOSE_SPIN_EXIT_V+5)))
                    break
            if options:
                # Not the direction whose close spin just ended in a contact / stall here, if the other works.
                options=[m for m in options if not self.close_spin_retry_blocked(m['direction'])] or options
                most=[m for m in options if m['front']>=.5] or options
                return min(most,key=lambda m:abs(m['turn']))
        return None
    def close_spin_exit_clear(self,remaining):
        """During the exit after a close spin (verified on averaged points before the turn): stop on
        actual contact (close_spin_contact), off measured free floor (the line was checked against the
        motion mask before the turn; that mask is rebuilt from each new cell), or when a raw point in FRONT comes
        closer than now by more than the single-scan noise (SPIN_RAW_ALLOWANCE) or within
        DRIVE_CLEARANCE of the remaining line when it is not already that close. Points behind are
        being left (the line was checked never to approach them), so their single-scan noise
        cannot stop the exit."""
        if self.close_spin_contact():return False
        p=self.localizer.pose;line=np.linspace(0,max(.01,remaining),11)
        world=transform(np.c_[line,np.zeros(len(line))],p);cells=np.array([self.grid.cell(xy) for xy in world])
        if any(not self.grid.valid(tuple(c_)) for c_ in cells) or not self.cells_clear(self.grid.masks()[0],cells):return False
        cloud=self.obstacle_points();ahead=cloud[cloud[:,0]>0] if len(cloud) else cloud
        if not len(ahead):return True
        gaps=np.linalg.norm(ahead[None,:,:]-np.c_[line,np.zeros(len(line))][:,None,:],axis=2).min(axis=1)
        return bool((gaps>=min(DRIVE_CLEARANCE,gaps[0]-SPIN_RAW_ALLOWANCE)).all())
    def arc_check(self,v,w,length):
        """Arc (v,w) of `length` m from the current pose. None if it would come closer than
        now (-3 mm) to any measured point, leave measured free floor (forward: the motion
        mask, as for driving) or, reversing, leave the driven trail ANYWHERE (on_trail);
        otherwise the circle-rule distance (beam-averaged points) from its end point."""
        p=self.localizer.pose;cloud=self.obstacle_points();spin=self.spin_points()
        if len(cloud)<60:return None
        rel=self.predicted_centres(v,w,np.linspace(0,length/abs(v),max(2,int(round(length/.005))+1)))
        world=transform(rel,p)
        if v<0 and not self.on_trail(world):return None
        gaps=np.linalg.norm(cloud[None,:,:]-rel[:,None,:],axis=2).min(axis=1)
        if (gaps<gaps[0]-.003).any():return None
        cells=np.array([self.grid.cell(xy) for xy in world])
        if any(not self.grid.valid(tuple(c_)) for c_ in cells):return None
        if not self.cells_clear(self.grid.masks()[0] if v<0 else self.motion_mask(),cells):return None
        return float(np.linalg.norm(spin-rel[-1],axis=1).min())
    def spin_room_escape(self):
        """Last fallback when the circle rule refuses every spin (points closer than TURN_CLEAR) and
        neither a straight forward drive nor a close spin helps: the shortest arc that never comes
        closer to anything and ends where every averaged point is >= TURN_CLEAR + 4 mm. Forward arcs
        only while the bumper is released (the pressed rule refuses any forward motion); reverse arcs
        only along the driven trail (arc_check), never onto unseen floor behind the unsensed rear
        shell. Reverse arcs keep |w| <= 2|v|/track, so the motor core accepts them while pressed.
        Needs SPIN_SCANS_MIN averaged scans."""
        if self.spin_scan_count()<SPIN_SCANS_MIN:return None
        p=self.localizer.pose;best=None
        for v in ((-.04,) if self.bumper_pressed else (-.04,.04)):
            for w in (0.,.15,-.15,.3,-.3):
                for length in np.arange(.01,(REVERSE_LIMIT if v<0 else .15)+1e-6,.01):
                    end=self.arc_check(v,w,length)
                    if end is None:break
                    if end<TURN_CLEAR+.004:continue
                    cost=length+.01*abs(w)/.15
                    if best is None or cost<best['cost']:best=dict(kind='arc',cost=cost,v=v,w=w,distance=float(length),heading0=float(p[2]),turn=0.)
                    break
        return best
    def spin_wait(self,stamp):
        """A spin was refused on fewer than SPIN_SCANS beam-averaged scans (all taken standing
        still at this heading): stand still and re-measure (at most SPIN_WAIT_S per spot and
        heading) before any fallback motion, so a gap the full average resolves as spin-clear
        is turned in at normal speed, and a close spin or escape is judged on averaged beams."""
        if stamp is None or self.spin_scan_count()>=SPIN_SCANS:return False
        p=self.localizer.pose;w=getattr(self,'spin_wait_from',None)
        if w is None or np.linalg.norm(w[1][:2]-p[:2])>SPIN_STILL_M or abs(wrap(w[1][2]-p[2]))>.05 or stamp<w[0]:self.spin_wait_from=w=(stamp,p.copy())
        return stamp-w[0]<SPIN_WAIT_S
    def begin_turn_escape(self,target,context,stamp):
        m=self.turn_escape(target)
        if not m:return None
        m.update(context=context,since=stamp if stamp is not None else 0.,start=None);self.maneuver=m
        if m.get('kind')=='arc':self.event(f"Too close to turn in place. {'Backing' if m['v']<0 else 'Driving'} {100*m['distance']:.0f} cm along a clear arc{' on the path just driven' if m['v']<0 else ''} to make room for the turn.")
        elif m.get('kind')=='close_spin':self.event(f"Closer than the turning margin but nothing touches: turning slowly in place ({math.degrees(abs(m['turn'])):.0f} deg {'left' if m['direction']>0 else 'right'}); a spin changes no distance.")
        else:self.event('Too close to turn toward the route. Turning partly, then moving a short clear distance forward.')
        return self.run_maneuver(context,stamp)
    def run_maneuver(self,context,stamp):
        m=getattr(self,'maneuver',None)
        if not m:return None
        p=self.localizer.pose
        if m['context']!=context or (stamp is not None and stamp-m['since']>m.get('timeout',20)):self.maneuver=None;return None
        if m.get('kind')=='close_spin':
            if m.get('start') is None:
                m['turned']+=wrap(p[2]-m['last']);m['last']=float(p[2]);left=m['turn']-m['turned']
                if abs(left)>=.04 and left*m['direction']>0:
                    # Abort only on actual contact (raw returns inside the skin, or a press on the side
                    # the firmware refuses to turn toward); the robot then stands still and re-measures.
                    stalled=self.close_spin_stalled(m,stamp)
                    if self.close_spin_contact() or not self.spin_allowed(m['direction']) or stalled:
                        self.close_spin_blocked=dict(xy=p[:2].copy(),direction=m['direction'],stamp=stamp if stamp is not None else 0.)
                        self.maneuver=None;self.event('Close turn stopped: '+('the turn stalled (gyro)' if stalled else 'contact measured')+'. Re-measuring before the next move.');return (0.,0.)
                    return (0.,m['direction']*CLOSE_SPIN_W)
                m['start']=p[:2].copy()
            # Straight forward exit, away from what is now behind: never closer than now.
            if m.get('settle') is None:
                done=np.linalg.norm(p[:2]-m['start'])
                if done<m['distance']-.005 and not self.bumper_pressed and self.close_spin_exit_clear(m['distance']-done):return (CLOSE_SPIN_EXIT_V,0.)
                m['settle']=stamp if stamp is not None else 0.
            # Stand still until the beams are averaged again (as spin_wait), so the next decision (a
            # normal turn now that there is room) is not made on one noisy scan.
            if stamp is not None and self.spin_scan_count()<SPIN_SCANS and stamp-m['settle']<SPIN_WAIT_S:return (0.,0.)
            self.maneuver=None;return None
        if m.get('kind')=='arc':
            if m.get('start') is None:m['start']=p.copy()
            done=np.linalg.norm(p[:2]-m['start'][:2])
            if done>=m['distance']-.003 or self.arc_check(m['v'],m['w'],max(.005,m['distance']-done)) is None:self.maneuver=None;return None
            return (m['v'],m['w'])
        left=m['turn']-wrap(p[2]-m['heading0'])
        if m.get('start') is None:
            if abs(left)>.04:
                w=.4*math.copysign(1,left)
                if self.velocity_still_clear(0.,w,max(.2,abs(left)/.4)):return (0.,w)
                self.maneuver=None;return None
            m['start']=p[:2].copy()
        if np.linalg.norm(p[:2]-m['start'])<m['distance']-.01 and self.velocity_still_clear(.05,0.,.5):return (.05,0.)
        self.maneuver=None;return None
    def follow_velocity(self,target,distance,stamp=None):
        context=(self.phase_started,tuple(self.goal) if self.goal is not None else None)
        velocity=self.run_maneuver(context,stamp)
        if velocity is not None:self.motion_until=stamp or 0;return velocity
        if stamp is not None and getattr(self,'motion_context',None)==context and stamp<getattr(self,'motion_until',0):
            v,w=self.motion_velocity
            if self.velocity_still_clear(v,w,self.motion_until-stamp+.15):return v,w
        p=self.localizer.pose;error=wrap(math.atan2(target[1]-p[1],target[0]-p[0])-p[2]);velocity=None
        facing=getattr(self,'facing',None) if getattr(self,'facing_context',None)==context else None
        if abs(error)>1.2 or (facing and abs(error)>.5):
            # Route starts behind the robot. Reversing is limited to the driven
            # trail, so turn in place to face it first if the body sweep is clear.
            # Keep one turn direction: near 180 degrees the error sign flips.
            preferred=facing or math.copysign(1,error)
            for direction in [preferred,-preferred]:
                # Commit only to a turn whose whole sweep (to within .5 rad of
                # the route) is clear. Next to low objects each direction was
                # often clear for part of the way, and the robot swung back and
                # forth forever; otherwise the local search may move first.
                sweep=abs(error) if direction*error>0 else 2*math.pi-abs(error)
                if self.velocity_still_clear(0.,.4*direction,max(1.,(sweep-.45)/.4)):velocity=(0.,.4*direction);self.facing=direction;self.facing_context=context;break
            if velocity is None and self.spin_wait(stamp):velocity=(0.,0.)
        else:self.facing=None
        if velocity is None and abs(error)>1.2:velocity=self.begin_turn_escape(target,context,stamp)
        if velocity is None:
            velocity=choose_velocity(self,target,distance)
            # Nothing to drive and no spin (points inside TURN_CLEAR): gain spin room first.
            if not any(velocity) and stamp is not None and not self.rotation_clear() and not self.spin_wait(stamp):
                velocity=self.begin_turn_escape(target,context,stamp) or velocity
        if stamp is not None:
            # The motor core ramps wheel speed. Rapidly replacing a one-second
            # search action caused alternating wheel targets before the robot
            # could execute either action, especially at a rug edge.
            self.motion_context=context;self.motion_velocity=velocity;self.motion_until=stamp+.65 if any(velocity) else stamp
        return velocity
    def path_target(self):
        p=self.localizer.pose;route=np.asarray(self.path)
        nearest=int(np.argmin(np.linalg.norm(route-p[:2],axis=1)));safe=self.motion_mask()
        for index in range(min(len(route)-1,nearest+7),nearest,-1):
            # Sample the actual segment from the measured position. A rounded
            # cell-centre ray can miss a forbidden corner next to a doorway.
            line=np.linspace(p[:2],route[index],max(2,int(np.linalg.norm(route[index]-p[:2])/.005)+1))
            if self.cells_clear(safe,np.array([self.grid.cell(xy) for xy in line])):return route[index]
        return route[nearest]
    # ---- final approach to a Go-to target closer to an obstacle than the safe mask (APPROACH_*) ----
    def route_goal(self):
        """Where the A* route goes: the staging cell while a near-obstacle target is approached, else the goal."""
        st=getattr(self,'staging',None)
        return st if getattr(self,'near_goal',False) and st is not None and self.mode=='go_to' else self.goal
    def arrival_point(self):
        """Position confirm_arrival checks: where the final approach stopped, else the goal."""
        st=getattr(self,'approach_stop',None);return self.goal if st is None else st
    def approach_line_clear(self,a,b,free,clearance,need=GOAL_CLEAR):
        """Straight line a -> b (map frame) on measured free cells (no camera / contact cell, so the LiDAR sees
        along it) whose cell-centre clearance is at least `need` all along (GOAL_CLEAR: a body can plausibly
        follow it; 0: line of sight only)."""
        n=max(2,int(np.linalg.norm(np.asarray(b)-a)/.01)+1);cells=np.floor((np.linspace(a,b,n)-self.grid.origin)/self.grid.res).astype(int)[:,::-1]
        if (cells<0).any() or (cells>=self.grid.n).any():return False
        y,x=cells[:,0],cells[:,1]
        return bool(free[y,x].all() and not self.grid.camera_obstacles[y,x].any() and (clearance[y,x]>=need-1e-9).all())
    def staging_cell(self,pose):
        """The reachable safe cell nearest the target (within APPROACH_STAGING_M) with a straight measured-free
        line to it, preferring a line with GOAL_CLEAR clearance all along (approach_line_clear); None when no
        reachable safe cell that near even sees the target along measured free cells."""
        free,_,_,safe,clearance=self.grid.masks();reach=self.grid.reachable(pose[:2],safe)
        cy,cx=self.grid.cell(self.goal);r=int(math.ceil(APPROACH_STAGING_M/self.grid.res))
        y0,x0=max(0,cy-r),max(0,cx-r);cells=np.argwhere(reach[y0:cy+r+1,x0:cx+r+1])+[y0,x0]
        if not len(cells):return None
        xy=self.grid.origin+(cells[:,::-1]+.5)*self.grid.res;d=np.linalg.norm(xy-self.goal,axis=1)
        order=[i for i in np.argsort(d) if d[i]<=APPROACH_STAGING_M][:200]
        for need in (GOAL_CLEAR,0.):
            for i in order:
                if self.approach_line_clear(xy[i],self.goal,free,clearance,need):return xy[i].copy()
        return None
    def on_approach_line(self,pose):
        """Already where the approach can start: within the follow arrival radius (.10) of the staging cell,
        or (e.g. after a recovery) on the measured free line from the staging cell to the target."""
        if np.linalg.norm(self.staging-pose[:2])<.10:return True
        ab=self.goal-self.staging;t=float(np.clip((pose[:2]-self.staging)@ab/max(1e-9,ab@ab),0,1))
        if np.linalg.norm(self.staging+t*ab-pose[:2])>.05:return False
        free,_,_,_,clearance=self.grid.masks();return self.approach_line_clear(pose[:2],self.goal,free,clearance)
    def begin_approach(self,stamp):
        self.path=[];self.phase='approach';self.phase_started=stamp;self.approach=dict(stage='turn',started=stamp);self.approach_stop=None;self.approach_result=None;self.spin_wait_from=None
        # The wall wording when an obstacle is within the planning margin + .10 of the target (cell centres).
        c=self.grid.cell(self.goal);near=getattr(self,'near_goal',False) or (self.grid.valid(c) and self.grid.masks()[4][c]<PLAN_RADIUS+.10)
        self.event('Approaching the target close to the wall, slowly.' if near else 'Approaching the target slowly.')
    def live_lidar_points(self):
        """The current LiDAR returns in the robot frame NOW: current_cloud is in the frame of the scan's pose,
        carried to the current (odometry-advanced) pose. Robot-frame data: no localisation error applies."""
        lidar=self.current_cloud if self.current_cloud.shape[1:]==(2,) else np.empty((0,2))
        sp=self.current_scan_pose();p=self.localizer.pose
        if not len(lidar) or (np.abs(sp[:2]-p[:2]).max()<1e-9 and abs(wrap(sp[2]-p[2]))<1e-9):return np.asarray(lidar,float)
        return inverse(transform(np.asarray(lidar,float),sp),p)
    def camera_cell_points(self):
        """Camera / contact cells (map frame, cell centres) within 1.5 m, in the robot frame."""
        cells=np.argwhere(self.grid.camera_obstacles)
        if not len(cells):return np.empty((0,2))
        world=self.grid.origin+(cells[:,::-1]+.5)*self.grid.res;world=world[np.linalg.norm(world-self.localizer.pose[:2],axis=1)<1.5]
        return inverse(world,self.localizer.pose) if len(world) else np.empty((0,2))
    def approach_sweep(self,k,length):
        """Forward arc of curvature k (rad/m) from the current pose, sampled every APPROACH_STEP up to
        `length`: (room, reason) = the distance driven before the first sample that breaks a check, and that
        check ('lidar', 'camera', 'map'), or (length, None). Checks: >= DRIVE_CLEARANCE_LIDAR from every live
        LiDAR point, >= DRIVE_CLEARANCE from every camera / contact cell, and on measured free cells. A point
        already inside its limit (e.g. beside or behind the robot) may not come any closer, per point and with
        no slack: velocity_still_clear's 3 mm 'not closer than now' allowance on the nearest point would let an
        approach, which drives TOWARD things, ratchet inward a few mm per tick."""
        s=np.arange(0.,max(0.,length)+1e-9,APPROACH_STEP);s=np.r_[s,length] if s[-1]<length-1e-9 else s
        rel=self.predicted_centres(1.,k,s);lidar=self.live_lidar_points()                   # unit speed: time = arc length
        if len(lidar)<60:return 0.,'lidar'
        bad=np.zeros(len(s),bool);why=np.full(len(s),'',object);far=length+DRIVE_CLEARANCE+.05
        for name,pts,limit in (('lidar',lidar,DRIVE_CLEARANCE_LIDAR),('camera',self.camera_cell_points(),DRIVE_CLEARANCE)):
            pts=pts[np.linalg.norm(pts,axis=1)<far] if len(pts) else pts     # farther points cannot reach the line
            if not len(pts):continue
            gaps=np.linalg.norm(rel[:,None,:]-pts[None,:,:],axis=2)
            hit=(gaps<np.minimum(limit,gaps[0]-1e-9)[None,:]).any(axis=1)&~bad;why[hit]=name;bad|=hit
        free=self.grid.masks()[0];cells=np.floor((transform(rel,self.localizer.pose)-self.grid.origin)/self.grid.res).astype(int)[:,::-1]
        inside=(cells>=0).all(axis=1)&(cells<self.grid.n).all(axis=1);ok=np.zeros(len(s),bool)
        ok[inside]=free[cells[inside,0],cells[inside,1]]&~self.grid.camera_obstacles[cells[inside,0],cells[inside,1]]
        hit=~ok&~bad;why[hit]='map';bad|=hit
        if not bad.any():return float(s[-1]),None
        k=int(np.argmax(bad));return (float(s[k-1]) if k else 0.),why[k]
    def approach_clear(self,v,k):
        """Every-tick check of a forward approach command (v, curvature k): the positions over max(v x APPROACH_STOP_S,
        the stopping distance from the last commanded speed u) pass approach_sweep. That stopping distance is
        one 20 Hz tick plus the DriveCore.h wheel ramp (.25 m/s2): u x .05 + u^2 / .5 <= u x APPROACH_STOP_S / 2
        for u <= APPROACH_V."""
        u=max(0.,float(getattr(self,'requested_motion',(0.,0.))[0]));reach=max(v,u/2)*APPROACH_STOP_S
        return v>0 and self.approach_sweep(k,reach)[0]>=reach-1e-9
    def approach_command(self,t):
        """Phase 'approach': turn in place to face the target (circle rule), then drive straight at it,
        forward only and slowly, stopping on the target or on the closest safe point (see APPROACH_*)."""
        a=self.approach;p=self.localizer.pose
        if a is None:self.begin_approach(t);a=self.approach
        if t-a['started']>APPROACH_TIMEOUT_S:return self.finish_approach(t,'time')
        if self.jammed(t):return self.finish_approach(t,'jam')
        d=self.goal-p[:2];dist=float(np.linalg.norm(d))
        if dist<=APPROACH_REACHED:return self.finish_approach(t,'goal')
        error=wrap(math.atan2(d[1],d[0])-p[2])
        if a['stage']=='turn':
            if abs(error)>APPROACH_ALIGN:
                w=float(np.clip(1.5*error,-APPROACH_W,APPROACH_W));w=math.copysign(max(abs(w),APPROACH_W_MIN),w)
                if self.velocity_still_clear(0.,w,max(.2,abs(error)/abs(w))):self.spin_wait_from=None;return 0.,w
                if self.spin_wait(t):return 0.,0.
                return self.finish_approach(t,'turn')
            a['stage']='drive'
        along=float(d@np.array([math.cos(p[2]),math.sin(p[2])]))
        if along<=APPROACH_REACHED:return self.finish_approach(t,'goal')
        if dist>APPROACH_NEAR and abs(error)>APPROACH_REALIGN:a['stage']='turn';return 0.,0.
        # The arc through the target, tangent to the heading (straight in the last APPROACH_NEAR).
        k=float(np.clip(2*math.sin(error)/dist,-APPROACH_CURVE,APPROACH_CURVE)) if dist>APPROACH_NEAR else 0.
        room,why=self.approach_sweep(k,min(along,APPROACH_LOOK))
        v=min(APPROACH_V,room/APPROACH_STOP_S)
        if v<APPROACH_V_MIN:return self.finish_approach(t,why or 'goal')
        # A sudden drop in the room (a new scan) while still fast: brake this tick, re-check on the next.
        if not self.approach_clear(v,k):return 0.,0.
        return v,k*v
    def finish_approach(self,t,reason):
        """End the approach where the robot is; confirm that position (confirm_arrival) before Idle."""
        p=self.localizer.pose;lidar=self.live_lidar_points();cam=self.camera_cell_points()
        # Skin gaps for the message: to the nearest live LiDAR return, and to the nearest camera / contact cell's
        # near half (the obstacle lies somewhere in that 5 cm cell, so its centre distance overstates the gap).
        gap=lambda pts,extra=0.:float(np.linalg.norm(pts,axis=1).min()-R-extra) if len(pts) else None
        self.approach_result=dict(reason=reason,lidar_gap=gap(lidar),camera_gap=gap(cam,self.grid.res/2))
        self.approach=None;self.approach_stop=p[:2].copy();self.path=[];self.phase='confirm_arrival';self.phase_started=t;self.arrival_frames=0
        short=float(np.linalg.norm(self.goal-p[:2]))
        self.event('At the goal. Confirming position before Idle.' if short<=APPROACH_ARRIVED else self.short_message(short)+' Confirming position before Idle.')
        return 0.,0.
    def short_message(self,short):
        r=self.approach_result or {};why=r.get('reason');lg,cg=r.get('lidar_gap'),r.get('camera_gap')
        cm=lambda x:max(0,int(round(100*x)))
        text={'lidar':'the closest safe point'+(f' (about {cm(lg)} cm from the wall)' if lg is not None else ''),
              'camera':'the closest safe point'+(f' (about {cm(cg)} cm from an obstacle the camera or bumper found)' if cg is not None else ''),
              'map':'the closest safe point (the floor beyond it is not mapped as free)','goal':'the closest point on the approach line',
              'turn':'turning toward it is not clear here','jam':'the wheels slip here','time':'the approach took too long',
              'contact':'the bumper touched an obstacle on the way; the contact is now on the map'}.get(why,'the closest safe point')
        return f'Stopped {cm(short)} cm short of the target: {text}.'
    def arrival_message(self,pose):
        short=float(np.linalg.norm(self.goal-np.asarray(pose)[:2]))
        if getattr(self,'approach_stop',None) is None or short<=APPROACH_ARRIVED:return 'Arrived. Position checked across five LiDAR scans; now Idle.'
        return self.short_message(short)+' Position checked across five LiDAR scans; now Idle.'
    def reverse_on_trail(self,v,w,distance=REVERSE_GUARD_M):
        """The next `distance` m of a reverse command keep base_link on the driven trail (on_trail)."""
        rel=self.predicted_centres(v,w,np.linspace(0,distance/max(abs(v),1e-6),9));return self.on_trail(transform(rel,self.localizer.pose))
    def command(self,state):
        """drive_command, with a final guard (no rear bumper, rear camera or low-object sensing behind
        the robot): a reverse is sent only while its next REVERSE_GUARD_M stays on the driven trail
        (which reversing never extends, see observe_scan) and its reverse episode has covered less
        than REVERSE_LIMIT."""
        v,w=self.drive_command(state);p=self.localizer.pose
        # Pressed: send only what the motor core executes as commanded (firmware_executes). With the
        # clamping DriveCore.h core any spin or forward command would become a backward pivot / nothing.
        if not self.firmware_executes(v,w):
            if not self.pressed_refused:self.event('Motion held: while the bumper is pressed the motor core would not execute it as commanded (it may pivot the body backward).')
            self.pressed_refused=True;v,w=0.,0.
        else:self.pressed_refused=False
        ep=getattr(self,'reverse_episode',None)
        if v<0:
            if ep is None:
                # The episode starts here: trail_head is this position if it was reached driving forward
                # after the newest trail sample; otherwise the episode has no start of its own (None).
                head=self.trail_head();ep=self.reverse_episode=dict(start=None if head is None else np.asarray(head,float).copy(),last=p[:2].copy(),travelled=0.,forward_from=None)
            ep['travelled']+=float(np.linalg.norm(p[:2]-ep['last']));ep['last']=p[:2].copy();ep['forward_from']=None
            if ep['travelled']>=REVERSE_LIMIT or not self.reverse_on_trail(v,w):
                if not self.reverse_refused:self.event('Reverse refused: it would leave the path the robot has driven.' if ep['travelled']<REVERSE_LIMIT else 'Reverse refused: this reverse already covered the reverse limit.')
                self.reverse_refused=True;return 0.,0.
            self.last_reverse_stamp=state.get('sim_time_s',self.last_reverse_stamp)
            # Nothing is added to the trail until FORWARD_CLEAR_M of forward driving after this reverse.
            self.reversed_since_forward=True;self.forward_mark=None;self.trail_break=True
        else:
            if v>0 and self.reversed_since_forward:
                # Forward progress along the heading from where forward driving began (the wheel ramp may
                # first carry the body a little further back), after the settle time.
                if self.forward_mark is None:self.forward_mark=p.copy()
                u=np.array([math.cos(self.forward_mark[2]),math.sin(self.forward_mark[2])])
                if float((p[:2]-self.forward_mark[:2])@u)>=FORWARD_CLEAR_M and state.get('sim_time_s',0.)-self.last_reverse_stamp>=REVERSE_SETTLE_S:self.reversed_since_forward=False;self.forward_mark=None
            elif v<=0 and abs(w)>0 and self.forward_mark is not None:self.forward_mark=None
            if ep is not None:
                # The episode ends after max(REVERSE_EPISODE_RESET, the distance reversed) of forward travel
                # (turning does not end it): forward / back cycles cannot walk the robot back along its trail.
                if v>0 and ep['forward_from'] is None:ep['forward_from']=p[:2].copy()
                if ep['forward_from'] is not None and np.linalg.norm(p[:2]-ep['forward_from'])>=max(REVERSE_EPISODE_RESET,ep['travelled']):self.reverse_episode=None
        self.reverse_refused=False;return v,w
    def drive_command(self,state):
        t=state['sim_time_s'];loc=self.localizer;p=loc.pose
        if self.mode=='idle':self.wants_arm=False;return 0.,0.
        if getattr(self,'stall_position',None) is not None and np.linalg.norm(p[:2]-self.stall_position)>.5:
            self.stall_position=None;self.stall_attempts=0
        fault=state.get('fault','unknown')
        if state.get('armed'):self.motor_seen_armed=True;self.disarmed_since=None;self.wants_arm=False
        elif fault=='motor_power_off' and (self.motor_seen_armed or t-self.started>2):self.idle('Motor power was switched off. Stopped; progress saved.');return 0.,0.
        else:
            # A contact between 20 Hz samples, or the start of the run: ask the
            # service to arm again. While touching, the motor core accepts only
            # reverse, which the contact phase below commands.
            if fault in ['disarmed','watchdog'] and self.motor_seen_armed and self.bump_events is None and self.requested_motion[0]>.02 and self.mode=='mapping':self.note_contact(t,'unexpected motor stop')
            self.wants_arm=True;self.disarmed_since=t if self.disarmed_since is None else self.disarmed_since
            # Pressed in the contact phase the drive may stay disarmed (the firmware arms only for an
            # allowed motion): that is the expected state while waiting for the release, not a motor fault.
            if self.phase=='contact' and self.bumper_pressed:self.disarmed_since=t
            if t-self.disarmed_since>12:self.idle('Motor could not be armed ('+fault+'). Stopped; progress saved.');return 0.,0.
            if self.phase!='contact':return 0.,0.
        if t-self.started>TIME_BUDGET:
            if self.mode=='mapping':self.finish_mapping('Time budget reached.')
            else:self.idle('Time budget reached; unfinished map saved.')
            return 0.,0.
        scan_age=t-(self.frame.stamp if self.frame is not None else self.started)
        camera_age=t-(self.camera.last_stamp if self.camera.last_stamp is not None else self.started)
        if scan_age>3:self.idle('LiDAR readings stopped. Partial progress saved.');return 0.,0.
        # The robot stands still while re-verifying its position; a camera gap
        # then is not a failure. Any motion requires fresh images below.
        if camera_age>3 and self.phase not in ['relocalizing','recover_localization']:self.idle('Camera images stopped. Low-obstacle coverage unavailable.');return 0.,0.
        if self.frame is None or scan_age>.6 or camera_age>.8:return 0.,0.
        # Tilt stop: > .15 rad for 10 s. Margin note (B11): where the cliff floor plane is invalid the attitude comes from the
        # IMU, whose online offset correction (imu.ACCEL_ONLINE_LIMIT, tau 30 s) treats the floor plane as level; resting on a
        # sloped floor it can learn up to +/-.5 m/s2 (~3 deg) of slope as an offset and carry it into floor-invalid areas,
        # i.e. up to ~3 deg of the .15 rad (8.6 deg) margin.
        if not self.attitude_initialized or self.tilt_rad>.15:
            self.tilted_since=t if self.tilted_since is None else self.tilted_since
            if t-self.tilted_since>10:self.idle('Robot remained tilted or attitude unknown. Stopped; progress saved.')
            return 0.,0.
        self.tilted_since=None
        if self.phase in ['relocalizing','recover_localization']:
            if t-self.phase_started>45:self.idle(relocalization_failure_message(loc.last_global) if loc.last_global else 'Could not confirm the LiDAR position against the saved map; not moving.');return 0.,0.
            return 0.,0.
        if loc.uncertainty>.15 or t-loc.last_visual_time>1:
            self.phase='recover_localization';self.phase_started=t;loc.pending=None;loc.relocalization_hits=0;self.event('LiDAR position confidence dropped. Stopped to locate against saved geometry.');return 0.,0.
        if self.phase=='sense':
            if t-self.phase_started>1.2 and self.scan_count>=5:
                # The first move waits for the start-up gyro-bias calibration (imu.GYRO_STARTUP_S of rest).
                if self.imu.ready_to_move():self.phase='plan';self.plan_not_before=t
                elif t-self.phase_started>30:self.idle('The gyro start-up calibration needs the robot standing still for 2 s and it did not. Stopped; progress saved.')
                else:self.event('Calibrating the gyro bias at rest before the first move.')
            return 0.,0.
        if self.phase=='plan':
            if t>=self.plan_not_before:self.plan(t);self.plan_not_before=t+.5
            return 0.,0.
        if self.phase=='confirm_arrival':
            if np.linalg.norm(self.arrival_point()-p[:2])>=.10:self.phase='plan';self.arrival_frames=0
            elif t-self.phase_started>5:self.idle('Arrival could not be confirmed.')
            return 0.,0.
        if self.phase=='jam_settle':
            # After a jam (begin_jam_settle): stand still until the wheels have stopped and the pose settled, then the
            # usual jam handling from the settled pose.
            if t-self.phase_started>=JAM_SETTLE_S:
                self.phase=self.jam_from_phase
                # A straight retry that jammed back at the spot its retreat left (within the retreat + 2 cm, settled pose)
                # re-drove only the retreat's ground: its reverse episode goes on (REVERSE_EPISODE_RESET was never really
                # covered; the pose swing while pushing can make it look covered).
                if self.phase=='recovery_advance' and getattr(self,'retry_episode',None) is not None and np.linalg.norm(p[:2]-self.escape_origin[:2])<=self.retreat_distance+.02:
                    self.reverse_episode=dict(self.retry_episode,forward_from=None,last=p[:2].copy())
                if self.note_jam(p,t) and not self.begin_retreat(t):self.skip_goal(self.jam_reason,t)
            return 0.,0.
        if self.phase!='contact':self.waiting_release=False
        if self.phase=='contact':
            if getattr(self,'contact_spin',None) is not None:
                # No verified reverse room: turn in place so the contact azimuth moves toward
                # 0 deg (a pure-spin core allows a pressed spin only toward the pressed side: this way;
                # contact_spin is never set for the clamping DriveCore.h core).
                left=self.contact_spin-wrap(p[2]-self.escape_origin[2])
                if self.bumper_pressed and abs(left)>.05 and t-self.phase_started<10 and self.spin_allowed(left):return 0.,math.copysign(CONTACT_SPIN,left)
                # Contact in front (or released): reverse straight away from it if the trail allows.
                self.contact_spin=None;self.escape_origin=p.copy();self.phase_started=t
                self.backoff=max(0.,min(.12,self.reverse_budget(),self.reverse_room(REVERSE_LIMIT)-.015))
                if self.backoff<.02:self.backoff=0.
            if self.backoff<.02:
                # No driven trail behind: NEVER reverse (the rear is unsensed and unseen). Released:
                # replan (the planner turns in place, slowly when close, or drives away). Still
                # pressed: the firmware refuses forward motion and a spin can only keep the contact
                # in front, so wait for the skin to release, then stop safely.
                if not self.bumper_pressed:
                    self.phase='plan';self.plan_not_before=t+.3;self.translation_anchor=None;self.last_progress=t;self.progress_pose=p.copy();self.jam_reset=t;return 0.,0.
                # Still pressed: stand still and keep the session alive (no blind reverse, no forward
                # push). Bumper and LiDAR are re-checked every tick; the robot replans the moment the
                # skin releases (the obstacle moved or was moved). Only the session's own limits
                # (time budget, a user stop) end the run here.
                if t-self.phase_started>10 and not getattr(self,'waiting_release',False):
                    self.waiting_release=True;self.event('Pressed against an obstacle with no driven path behind it: reversing there would be blind and the bumper refuses forward motion. Waiting for the bumper to release.')
                return 0.,0.
            travelled=np.linalg.norm(p[:2]-self.escape_origin[:2])
            # Short straight reverse on the verified trail; anything unexpected
            # ends the back-off early and the planner takes over from here.
            if (travelled>=self.backoff-.005 and not self.bumper_pressed) or t-self.phase_started>10 or abs(wrap(p[2]-self.escape_origin[2]))>.35 or not self.straight_corridor_clear(-1,max(.01,self.backoff-travelled),True):
                # Still pressed: the back-off cannot go on, and nothing else is legal while pressed (no
                # forward push, no blind reverse): stay in the contact phase and wait for the release.
                if self.bumper_pressed:self.backoff=0.;self.phase_started=t;return 0.,0.
                self.phase='plan';self.plan_not_before=t+.3;self.translation_anchor=None;self.last_progress=t;self.progress_pose=p.copy();self.jam_reset=t;return 0.,0.
            return -.05,0.
        if self.phase=='escape_margin':
            if self.grid.masks()[3][self.grid.cell(p[:2])]:self.phase='plan';self.plan_not_before=t;self.translation_anchor=None;return 0.,0.
            if t-self.phase_started>3 and np.linalg.norm(p[:2]-self.escape_origin[:2])<.02:
                # A measured-clear corridor can still contain an unobserved
                # low object. Stop, verify pose, and back out before replanning;
                # repeating the same forward push cannot resolve this stall.
                loc.invalidate();loc.pending=None;loc.relocalization_hits=0
                self.retreat_after_localization=True;self.retreat_requires_replan=True
                self.phase='recover_localization';self.phase_started=t;self.path=[]
                self.event('Clearance escape stalled. Verifying position before backing out and choosing a new approach.');return 0.,0.
            if t-self.phase_started>7 or np.linalg.norm(p[:2]-self.escape_origin[:2])>(DRIVE_CLEARANCE+.02 if self.escape_sign>0 else REVERSE_LIMIT):
                self.skip_goal('Could not leave the obstacle margin.',t);return 0.,0.
            if self.clearance_escape()!=self.escape_sign:
                self.phase='plan';self.plan_not_before=t+.5;return 0.,0.
            return .04*self.escape_sign,0.
        if self.phase=='escape_turn':
            error=wrap(self.escape_heading-p[2])
            if abs(error)<.04:
                self.phase='plan';self.plan_not_before=t;self.translation_anchor=None;self.relaxed_exit=True;return 0.,0.
            if t-self.phase_started>22 or not self.rotation_clear():
                self.skip_goal('Cannot safely turn out of the obstacle margin.',t);return 0.,0.
            return 0.,float(np.clip(error,-.18,.18))
        if self.phase=='retreat':
            travelled=np.linalg.norm(p[:2]-self.escape_origin[:2])
            if travelled>=self.retreat_distance:
                if not self.retry_straight:self.phase='plan';self.plan_not_before=t;self.translation_anchor=None;self.last_progress=t;self.progress_pose=p.copy();return 0.,0.
                self.phase='recovery_advance';self.escape_origin=p.copy();self.phase_started=t;self.retry_episode=None if self.reverse_episode is None else dict(self.reverse_episode)
                self.event('Retrying the measured clear corridor at low speed, holding the current heading.');return 0.,0.
            error=wrap(self.escape_origin[2]-p[2])
            # Check the remaining motion plus a braking allowance. Requiring
            # another fixed 8 cm on every tick prematurely aborted a 3.5 cm
            # recovery just before it reached its own stopping point.
            if t-self.phase_started>8 or abs(error)>.35 or not self.retreat_clear(self.retreat_distance-travelled+.025):
                self.skip_goal('The measured retreat could not continue safely.',t);return 0.,0.
            return -.05,float(np.clip(error,-.12,.12))
        if self.phase=='recovery_advance':
            distance=np.linalg.norm(p[:2]-self.escape_origin[:2])
            if distance>=.28:self.phase='plan';self.plan_not_before=t;self.last_progress=t;self.progress_pose=p.copy();self.translation_anchor=None;return 0.,0.
            if self.jammed(t):self.begin_jam_settle(t,'The straight retry did not get through.');return 0.,0.
            if t-self.phase_started>7:
                if not self.begin_retreat(t):self.skip_goal('The straight retry did not get through.',t)
                return 0.,0.
            if abs(wrap(p[2]-self.escape_origin[2]))>.15 or not self.straight_corridor_clear(1,max(.12,.30-distance)):
                self.phase='plan';self.plan_not_before=t+.5;self.event('Measured clearance changed during the retry. Replanning.');return 0.,0.
            # Slowly when LiDAR/camera points are near ahead.
            ahead=self.obstacle_points();ahead=ahead[ahead[:,0]>0] if len(ahead) else ahead
            return (.06 if len(ahead) and float(np.linalg.norm(ahead,axis=1).min())<NEARBY else .10),0.
        if self.phase=='approach':return self.approach_command(t)
        if self.phase!='follow':return 0.,0.
        route_goal=self.route_goal()
        distance=np.linalg.norm(route_goal-p[:2])
        if distance<.10:
            self.translation_anchor=None
            self.restore_radius()
            if self.mode=='mapping':self.scanned_positions.append(p[:2].copy());self.phase='sense';self.phase_started=t;self.save_map(False,'Exploration in progress');self.event('At a new viewpoint. Measuring the surrounding space.')
            # Go-to: the final approach covers the rest unless already within APPROACH_ARRIVED of the target.
            elif route_goal is not self.goal or np.linalg.norm(self.goal-p[:2])>APPROACH_ARRIVED:self.begin_approach(t)
            else:self.phase='confirm_arrival';self.phase_started=t;self.arrival_frames=0;self.event('At the goal. Confirming position before Idle.')
            return 0.,0.
        if self.jammed(t):
            # Typically the caster or a wheel caught on a rug edge or threshold:
            # back off a little on the driven trail, then retry straight.
            self.begin_jam_settle(t,'Wheels turn but the robot does not move.')
            return 0.,0.
        if self.translation_stalled(t):
            if self.grid.radius<PLAN_RADIUS:
                # Narrow-passage route that the measured points do not allow (DRIVE_CLEARANCE on
                # both sides): remember the pinch just ahead so narrow planning stops choosing it
                # for NARROW_BLOCK_S. The measured width is logged: a stall in a passage at least
                # 2 x DRIVE_CLEARANCE wide is a planner / local-planner mismatch, not a tight gap.
                width=self.passage_width()
                self.grid.narrow_blocks.append(p[:2]+.10*np.array([math.cos(p[2]),math.sin(p[2])]))
                self.grid.narrow_block_times.append(t)
                self.event(f'The narrow passage ahead is too tight for the driving clearance (measured width {100*width:.1f} cm, '
                           f'needs {200*DRIVE_CLEARANCE:.0f} cm). Not planning through it again for {NARROW_BLOCK_S/60:.0f} min.')
            if not self.begin_retreat(t):self.skip_goal('No translational progress toward this target.',t)
            return 0.,0.
        if np.linalg.norm(p[:2]-self.progress_pose[:2])>.035 or abs(wrap(p[2]-self.progress_pose[2]))>.12:self.progress_pose=p.copy();self.last_progress=t
        if t-self.last_progress>6 and self.begin_retreat(t):return 0.,0.
        if t-self.last_progress>18:
            self.skip_goal('No progress toward this target.',t);return 0.,0.
        if t>=self.next_plan and self.mode=='mapping' and getattr(self,'goal_kind',None)=='coverage':
            # A coverage target is a floor cell the camera has not yet passed
            # within range. Once the drive there has covered it, stop going.
            if self.coverage_goal_covered():
                self.goal_kind=None;self.restore_radius();self.path=[];self.phase='plan';self.plan_not_before=t
                self.event('Target area covered on the way. Choosing the next target.');return 0.,0.
        if t>=self.next_plan:
            path=self.grid.path(p[:2],route_goal);self.next_plan=t+1
            # Near narrow gaps a route can flicker as each scan moves a margin by
            # one cell. Keep the committed target and its previous route (live
            # collision checks still apply) unless the failure lasts 5 s.
            if path:self.path=path;self.path_failed_since=None
            else:
                self.path_failed_since=t if getattr(self,'path_failed_since',None) is None else self.path_failed_since
                if not self.path or t-self.path_failed_since>5:self.path_failed_since=None;self.route_lost(t);return 0.,0.
        return self.follow_velocity(self.path_target(),distance,t)
    def status(self,state=None):
        s=super().status(state);s.update(camera=self.camera.status,low_obstacle_camera=self.low_camera.status,contacts=len(self.contacts),skipped_targets=len(self.blacklist),camera_obstacle_cells=int(self.grid.camera_obstacles.sum()),sensor_configuration='d200_single_camera',lidar_scan=self.last_scan,stereo_frame=self.camera_index,lidar_points=len(self.current_cloud),lidar_quality=self.localizer.quality,imu=self.imu.status(),attitude_deg=np.degrees(self.attitude).round(3).tolist(),attitude_initialized=bool(self.attitude_initialized),heading_uncertainty_rad=round(float(getattr(self.localizer,'heading_uncertainty',0.)),5),map_saved=(self.root/'Saved/Maps/latest-lidar-map.json').exists());return s
