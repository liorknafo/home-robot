"""IMU calibration on the robot (bay B11): what a real MPU-6050 (GY-521) needs before its numbers mean anything.

Measured sensors only: never simulator truth, never the simulator's error model (controller/sim_sensor_model.py).

Gyro bias. The zero-rate offset of an MPU-6050 is up to +/-20 deg/s per axis and drifts as the enclosure warms.
  * Start-up calibration: while the robot stands still (no motion command for ZUPT_SETTLE_S, or motors off; both
    encoders unchanged, gyro and accelerometer steady) the bias is estimated per axis. A provisional estimate
    exists after GYRO_PROVISIONAL_N still samples (the attitude may initialise); the start-up calibration is done
    after GYRO_STARTUP_S of continuous rest, and the robot does not leave its first measuring stop before that
    (LidarBrain 'sense', and a Go-to's relocalization).
  * Zero-velocity updates (ZUPT): every later stop that passes the same rest test refines the bias, after its first
    ZUPT_SKIP_S (the body still settling). The estimate is a per-axis Kalman filter: the bias is a random walk
    (GYRO_Q, the warm-up drift) measured by every still sample (GYRO_R).
  * Yaw coupling (bring-up spin calibration): cross-axis sensitivity (+/-2 %) and the board's mounting misalignment
    make the x / y gyro follow the yaw rate. On a flat floor the robot spins in place one turn each way; the
    difference of the two directions' mean x / y readings over the difference of their z readings is that coupling
    (the bias cancels), stored in the calibration file and removed from every sample.
  * The gyro scale factor (+/-3 %) is NOT calibrated: the LiDAR fixes the heading every scan, and the heading
    uncertainty grows with HEADING_SCALE_UNC per radian turned plus the bias filter's sigma (localization.py).

Accelerometer. Its zero-g offsets (up to +/-50 mg x/y, +/-80 mg z) tilt the measured gravity vector and change its
norm: with the old fixed gate | |f| - 9.8 | < 0.3 m/s2 a real board could never initialise its attitude.
  * Level calibration (bring-up, once, BRINGUP.md): the robot at rest on a flat floor at two headings 180 deg apart;
    offsets = mean specific force - g x the gravity direction given by the cliff ToF floor plane (two headings
    cancel the floor's own tilt). level_calibration() is that procedure; the file is
    <root>/Content/RobotData/imu-calibration.json. The simulator writes the same file with the same procedure
    (controller/imu_level_calibration.py).
  * Gate: a sample counts as gravity when | |f - offsets| - calibrated gravity | < GRAVITY_GATE.
  * Online offset correction: at rest with a valid cliff floor plane (downward_floor_pose), the expected specific
    force is g x that plane's gravity direction; the residual is low-pass filtered (ACCEL_ONLINE_TAU) into an
    online offset, bounded by ACCEL_ONLINE_LIMIT (the datasheet's temperature drift plus margin). It keeps the
    roll/pitch offsets right as the board warms, and it also lets a board without a calibration file converge.
"""
import json,math,time
from pathlib import Path
import numpy as np

CALIBRATION_FILE='Content/RobotData/imu-calibration.json'
G_LOCAL=9.80               # m/s2: reference gravity of the level calibration (local g; the simulator uses 9.8)
GRAVITY_GATE=.3            # m/s2: a sample is a gravity sample when | |f| - calibrated gravity | < this
GYRO_STARTUP_S=2.          # s of continuous rest: start-up gyro-bias calibration
GYRO_PROVISIONAL_N=5       # still samples before a provisional bias (enough for the attitude to initialise)
# IMU sampling (ruling PB27; BRINGUP section 14): the Pico runs the MPU-6050 with DLPF_CFG = 4 (gyro bandwidth 20 Hz,
# accelerometer 21 Hz) and forwards samples at IMU_RATE_HZ, or averages them to it. The noise-dependent constants below
# derive from these numbers, so a firmware rate change cannot silently change what they mean.
IMU_DLPF_CFG=4
IMU_BANDWIDTH_HZ=20.       # Hz, gyro bandwidth at DLPF_CFG 4 (datasheet register map)
IMU_RATE_HZ=20.            # Hz, samples the robot receives
GYRO_NOISE_DENSITY=math.radians(.005)   # rad/s per sqrt(Hz), datasheet rate noise spectral density
GYRO_SAMPLE_SIGMA=GYRO_NOISE_DENSITY*math.sqrt(IMU_BANDWIDTH_HZ)  # rad/s per sample inside the DLPF bandwidth (~.0004)
ZUPT_SETTLE_S=.5           # s after the last motion command before a stop can count as rest (the wheel ramp stops in ~.4 s)
ZUPT_SKIP_S=.15            # s at the start of a rest within ZUPT_RECENT_S of driving whose samples are not used for the bias:
ZUPT_RECENT_S=3.           # the body is still settling (Unreal: 1-2 samples of .005-.01 rad/s with 1-2 encoder ticks); after a long
                           # drive the bias filter's gain is ~.9, so one settling sample would set the bias. Time-based: the
                           # same .15 s at any sample rate
STILL_TICKS=1              # encoder ticks per wheel allowed between consecutive samples at rest
STILL_GYRO_STEP=max(.01,25*GYRO_SAMPLE_SIGMA)   # rad/s: largest sample-to-sample change of any gyro axis at rest
STILL_ACCEL_STEP=.3        # m/s2: the same for the accelerometer (noise sigma ~.018)
# (rad/s)^2 per sample the bias filter assumes: 2 x the noise; samples closer than the filter's time scale are correlated, so
# at most ~2 x bandwidth independent samples per second count.
GYRO_R=(2*GYRO_SAMPLE_SIGMA)**2*max(1.,IMU_RATE_HZ/(2*IMU_BANDWIDTH_HZ))
GYRO_Q=3e-7                # (rad/s)^2 per s: bias random walk, covers the warm-up drift (<= .004 deg/s per s)
GYRO_P0=math.radians(20)**2   # initial ZRO tolerance, +/-20 deg/s (datasheet)
GYRO_OUTLIER=5.            # a 'still' sample further than this many sigma from the bias is not rest (after start-up): a slow
                           # rotation the encoders cannot see (a hand tilting the robot) is not absorbed into the bias; the
                           # rest stays refused until the robot drives again (rest_vetoed)
ACCEL_ONLINE_TAU=30.       # s: time constant of the online offset correction (at rest, floor plane valid)
ACCEL_ONLINE_LIMIT=np.array([.5,.5,.8])   # m/s2: bound of the online correction per axis. NOTE: the floor plane is the
                           # reference, so a robot resting on a SLOPED floor (or with cliff ToF offsets) learns that slope as an
                           # offset, up to +/-.5 m/s2 (~3 deg) with tau 30 s, and carries it into floor-invalid areas (see the
                           # tilt-stop margin in lidar_brain.drive_command)
# Bring-up level-calibration checks (single source; BRINGUP section 14 quotes these numbers). The stored offsets also absorb
# the accelerometer's sensitivity (+/-3 % of g, z) and cross-axis (+/-2 % of g, x / y) at level, so an in-spec MPU-6050
# reaches +/-.69 / .69 / 1.08 m/s2: the offset bounds only catch gross faults. The real fault detectors are the two headings'
# residual difference (floor not flat, robot moved) and the floor attitude (tilted board or cliff ToF offsets).
LEVEL_OFFSET_LIMIT_M_S2=(.7,.7,1.1)
LEVEL_HEADING_DIFF_LIMIT_M_S2=.2   # per axis, |residual heading A - residual heading B| = 2 x floor tilt x g (.2 ~ .6 deg)
LEVEL_FLOOR_ATTITUDE_LIMIT_DEG=.5  # cliff floor-plane roll / pitch at rest
YAW_COUPLING_LIMIT=.05             # cross-axis 2 % + mounting tilt
HEADING_SCALE_UNC=.03      # rad per rad turned: gyro scale-factor tolerance (not calibrated)
HEADING_BIAS_UNC=math.radians(.05)   # rad/s: gyro bias left after the start-up calibration / ZUPT

def gravity_direction(roll,pitch):
    """Unit specific force at rest in the body frame (x forward, y left, z up) for ROS roll / pitch."""
    return np.array([-math.sin(pitch),math.sin(roll)*math.cos(pitch),math.cos(roll)*math.cos(pitch)])

def yaw_coupling(spins):
    """Bring-up spin calibration: spins = (samples spinning one way, samples spinning the other way) in place on a
    flat floor, each a list of dicts with 'gyro' (3, rad/s, raw). Returns the x / y gyro output per unit yaw rate."""
    a,b=(np.mean([s['gyro'] for s in spin],axis=0) for spin in spins);dz=a[2]-b[2]
    if min(len(spins[0]),len(spins[1]))<20 or abs(dz)<.4:raise ValueError('spin calibration needs >= 20 samples each way at >= .2 rad/s')
    # A body that spins while tilted (cliff floor plane: roll r, pitch p) really turns about the vertical, which has
    # body x / y components (-tan p / cos r, tan r) per unit z rate: that part is motion, not coupling.
    floors=[s['floor'] for spin in spins for s in spin if s.get('floor') is not None]
    r,p=np.mean(floors,axis=0) if floors else (0.,0.)
    return (a[:2]-b[:2])/dz-np.array([-math.tan(p)/math.cos(r),math.tan(r)])

def calibration_checks(cal):
    """The bring-up checks of a calibration dict (limits above). All must hold before the file is used."""
    off=np.abs(np.asarray(cal['accel_offset_m_s2'],float));res=np.asarray(cal.get('heading_residual_m_s2') or [],float).reshape(-1,3)
    floor=np.degrees(np.abs(np.asarray(cal.get('floor_attitude_rad') or [[0.,0.]],float)))
    return dict(offsets_within_bounds=bool(np.all(off<=np.asarray(LEVEL_OFFSET_LIMIT_M_S2))),
                heading_residuals_agree=bool(len(res)<2 or np.max(np.ptp(res,axis=0))<=LEVEL_HEADING_DIFF_LIMIT_M_S2),
                floor_level=bool(np.max(floor)<=LEVEL_FLOOR_ATTITUDE_LIMIT_DEG),
                yaw_coupling_within_bounds=bool(np.all(np.abs(np.asarray(cal.get('gyro_yaw_coupling',[0.,0.]),float))<=YAW_COUPLING_LIMIT)))

def level_calibration(headings,gravity=G_LOCAL,note='',spins=None):
    """Bring-up level calibration from rest samples at one or more headings (two, 180 deg apart, cancel the
    floor's own tilt). headings: list of sample lists; each sample is a dict with 'force' (3, m/s2), 'gyro'
    (3, rad/s) and 'floor' ((roll, pitch) of the cliff ToF floor plane, or None). spins: optional samples of the
    spin calibration (yaw_coupling). Returns the calibration dict."""
    residuals=[];floors=[];gyros=[];count=0;spread=0.
    for samples in headings:
        f=np.array([s['force'] for s in samples],float);fl=[s['floor'] for s in samples if s.get('floor') is not None]
        if len(f)<20:raise ValueError('level calibration needs at least 20 rest samples per heading')
        if len(fl)<len(samples)//2:raise ValueError('the cliff floor plane was not valid at rest: place the robot on a flat, plain floor')
        floor=np.mean(np.asarray(fl,float),axis=0);floors.append(floor.tolist())
        g=np.array([s['gyro'] for s in samples],float);gyros.append(g.mean(0))
        if np.max(np.ptp(g,axis=0))>4*STILL_GYRO_STEP or np.max(np.ptp(f,axis=0))>4*STILL_ACCEL_STEP:raise ValueError('the robot moved during the level calibration')
        residuals.append(f.mean(0)-gravity*gravity_direction(*floor));count+=len(f);spread=max(spread,float(np.max(f.std(0))))
    offset=np.mean(residuals,axis=0)
    cal=dict(schema='r1-imu-calibration/1',procedure='level calibration at bring-up: the robot still (no motion command) on a flat floor, at each heading; '
                'offset = mean specific force - g x gravity direction of the cliff ToF floor plane, averaged over the headings (two headings '
                '180 deg apart cancel the floor tilt)',accel_offset_m_s2=offset.round(6).tolist(),gravity_m_s2=gravity,headings=len(headings),
                samples=count,floor_attitude_rad=floors,heading_residual_m_s2=[r.round(6).tolist() for r in residuals],
                accel_noise_m_s2=round(spread,6),gyro_mean_at_calibration_rad_s=np.mean(gyros,axis=0).round(7).tolist(),
                gyro_yaw_coupling=[0.,0.] if spins is None else yaw_coupling(spins).round(6).tolist(),
                spin_samples=None if spins is None else [len(s) for s in spins],
                spin_floor_attitude_rad=None if spins is None else [np.mean([s['floor'] for s in spin if s.get('floor') is not None] or [[0.,0.]],axis=0).round(6).tolist() for spin in spins],note=note,created=time.strftime('%Y-%m-%dT%H:%M:%S'))
    cal['checks']=calibration_checks(cal)
    cal['limits']=dict(offset_m_s2=list(LEVEL_OFFSET_LIMIT_M_S2),heading_residual_difference_m_s2=LEVEL_HEADING_DIFF_LIMIT_M_S2,
                       floor_attitude_deg=LEVEL_FLOOR_ATTITUDE_LIMIT_DEG,yaw_coupling=YAW_COUPLING_LIMIT)
    return cal

class ImuCalibration:
    """Per-sample gyro-bias estimation and accelerometer correction for one robot (see the module doc)."""
    def __init__(self,root=None):
        self.path=None if root is None else Path(root)/CALIBRATION_FILE;self.loaded=False
        self.accel_offset=np.zeros(3);self.gravity=G_LOCAL;self.level_source='missing';self.coupling=np.zeros(2)
        self.online=np.zeros(3);self.online_updates=0
        self.bias=np.zeros(3);self.P=np.full(3,GYRO_P0);self.bias_valid=False;self.startup_done=False
        self.still=False;self.still_n=0;self.still_since=None;self.zupt_samples=0;self.samples=0
        self.prev=None;self.motion_stamp=-1e9;self.last_stamp=None;self.rest_vetoed=False
    def load(self):
        self.loaded=True
        if self.path is None or not self.path.exists():return
        try:
            d=json.loads(self.path.read_text());self.accel_offset=np.asarray(d['accel_offset_m_s2'],float).reshape(3);self.gravity=float(d['gravity_m_s2']);self.level_source=str(self.path.name)
            self.coupling=np.asarray(d.get('gyro_yaw_coupling',[0.,0.]),float).reshape(2)
        except (OSError,ValueError,KeyError):self.level_source='unreadable'
    @property
    def offsets(self):return self.accel_offset+self.online
    def correct_force(self,f):return np.asarray(f,float)-self.offsets
    def correct_gyro(self,w):
        w=np.asarray(w,float)-self.bias;w[:2]-=self.coupling*w[2];return w
    def at_rest(self,stamp,ticks,gyro,force,armed,requested):
        """Rest test from measurements and the robot's own command: no motion command for ZUPT_SETTLE_S (or motors
        off), both encoders unchanged, gyro and accelerometer steady sample to sample."""
        if armed and (abs(requested[0])>1e-6 or abs(requested[1])>1e-6):self.motion_stamp=stamp
        if self.prev is None:return False
        # A rest the outlier gate refused stays refused until the robot drives again (a command or encoder motion): while
        # it is refused no update happens, P grows and the gate would otherwise open within seconds (a slow hand tilt).
        if stamp-self.motion_stamp<ZUPT_SETTLE_S:self.rest_vetoed=False;return False
        pt,pg,pf=self.prev
        if ticks is not None and pt is not None and (abs(ticks[0]-pt[0])>STILL_TICKS or abs(ticks[1]-pt[1])>STILL_TICKS):self.rest_vetoed=False;return False
        if gyro is not None and pg is not None and np.max(np.abs(gyro-pg))>STILL_GYRO_STEP:return False
        if force is not None and pf is not None and np.max(np.abs(force-pf))>STILL_ACCEL_STEP:return False
        if getattr(self,'rest_vetoed',False):return False
        if gyro is not None and self.startup_done and np.any(np.abs(gyro-self.bias)>GYRO_OUTLIER*np.sqrt(self.P+GYRO_R)):self.rest_vetoed=True;return False
        return True
    def update(self,stamp,sensors,armed=False,requested=(0.,0.),floor=None):
        """One sample. Returns (gyro, force) corrected (None where the sensor is absent). floor: (roll, pitch) of the
        cliff ToF floor plane when valid, else None."""
        if not self.loaded:self.load()
        gyro=np.asarray(sensors['gyro_rad_s'],float).reshape(3) if 'gyro_rad_s' in sensors else None
        force=np.asarray(sensors['specific_force_m_s2'],float).reshape(3) if 'specific_force_m_s2' in sensors else None
        ticks=(float(sensors['left_ticks']),float(sensors['right_ticks'])) if 'left_ticks' in sensors and 'right_ticks' in sensors else None
        dt=0. if self.last_stamp is None else min(1.,max(0.,stamp-self.last_stamp));self.last_stamp=stamp;self.samples+=1
        self.P=self.P+GYRO_Q*dt
        self.still=self.at_rest(stamp,ticks,gyro,force,armed,requested);self.prev=(ticks,gyro,force)
        if not self.still:self.still_n=0;self.still_since=None
        else:
            self.still_n+=1;self.still_since=stamp if self.still_since is None else self.still_since
            settling=stamp-self.motion_stamp<ZUPT_RECENT_S and stamp-self.still_since<ZUPT_SKIP_S-1e-6
            if gyro is not None and not settling:
                k=self.P/(self.P+GYRO_R);self.bias=self.bias+k*(gyro-self.bias);self.P=(1-k)*self.P;self.zupt_samples+=1
                if self.still_n>=GYRO_PROVISIONAL_N:self.bias_valid=True
                if stamp-self.still_since>=GYRO_STARTUP_S-1e-9 and self.still_n>=GYRO_PROVISIONAL_N:self.startup_done=True
            if force is not None and floor is not None:
                residual=self.correct_force(force)-self.gravity*gravity_direction(*floor)
                self.online=np.clip(self.online+residual*min(1.,dt/ACCEL_ONLINE_TAU),-ACCEL_ONLINE_LIMIT,ACCEL_ONLINE_LIMIT);self.online_updates+=1
        return (None if gyro is None else self.correct_gyro(gyro)),(None if force is None else self.correct_force(force))
    def ready_to_move(self):
        """Start-up gyro calibration done (or no IMU samples at all: nothing to calibrate, e.g. a unit test that
        sets the attitude directly)."""
        return self.startup_done or self.samples==0
    def status(self):
        g=self.gravity;o=self.offsets
        return dict(gyro_bias_deg_s=np.degrees(self.bias).round(4).tolist(),gyro_bias_sigma_deg_s=np.degrees(np.sqrt(self.P)).round(4).tolist(),
                    gyro_startup_calibrated=self.startup_done,gyro_bias_provisional=self.bias_valid,at_rest=self.still,zupt_samples=self.zupt_samples,
                    level_calibration=self.level_source,accel_offset_m_s2=o.round(4).tolist(),accel_online_offset_m_s2=self.online.round(4).tolist(),
                    online_roll_pitch_offset_deg=[round(math.degrees(self.online[1]/g),3),round(math.degrees(-self.online[0]/g),3)],gravity_m_s2=g,
                    gyro_yaw_coupling=self.coupling.round(5).tolist())
