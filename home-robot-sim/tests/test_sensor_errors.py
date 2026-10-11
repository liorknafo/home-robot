"""Bay B11: the simulator's IMU / odometry error model (controller/sim_sensor_model.py) and the robot's IMU
calibration (controller/autonomy/imu.py). Truth below is the test's own synthetic trajectory, used only to score."""
import ast,copy,itertools,json,math,os,re,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[1];CONTROLLER=ROOT/'controller'
sys.path.insert(0,str(CONTROLLER));sys.path.insert(0,str(ROOT/'tests'))
import sim_sensor_model as ssm
from sim_sensor_model import SensorErrorModel,G_UNIT
import imu_level_calibration as calibration_tool
from autonomy import imu
from autonomy.imu import ImuCalibration,gravity_direction,level_calibration,GRAVITY_GATE,HEADING_SCALE_UNC
from autonomy.lidar_brain import LidarBrain,downward_floor_pose
from autonomy.geometry import CLIFF_XY,CLIFF_Z,BASE_Z,CPR,WHEEL_R,TRACK
G=9.8;DT=.05;DEG=math.pi/180

def model(seed=None,run_seed=0,**override):
    """A board from the config (or another seed); override= sets drawn constants (worst cases)."""
    config=ssm.load_config()
    if seed is not None:config['seed']=seed
    m=SensorErrorModel(enabled=True,run_seed=run_seed,config=config)
    for k,v in override.items():setattr(m,k,np.asarray(v,float) if isinstance(v,(list,tuple,np.ndarray)) else v)
    return m
def clone(m,run_seed):
    """The same board with its own noise and a cold clock (another power-on of the same robot)."""
    c=copy.deepcopy(m);c.noise=np.random.default_rng([c.seed,run_seed,11]);c.t_on=0.;c.last_time=None;return c
def calibrated_root(m):
    """A robot root holding the bring-up level calibration made by the robot's procedure for board m."""
    root=tempfile.mkdtemp();calibration_tool.synthetic(root,clone(m,777));return root
def cliff_for(roll,pitch,height=0.):
    """Cliff ToF ranges of a body at (roll, pitch, height) over a flat floor (inverse of downward_floor_pose)."""
    s=np.array([-math.tan(pitch)/math.cos(roll),math.tan(roll),(height+BASE_Z)/(math.cos(roll)*math.cos(pitch))-BASE_Z+CLIFF_Z])
    return (np.c_[np.array(CLIFF_XY),np.ones(4)]@s).tolist()
def body_rates(roll,pitch,droll,dpitch,dyaw):
    return np.array([droll-dyaw*math.sin(pitch),dpitch*math.cos(roll)+dyaw*math.sin(roll)*math.cos(pitch),-dpitch*math.sin(roll)+dyaw*math.cos(roll)*math.cos(pitch)])
def state(m,t,omega=(0,0,0),force=(0,0,G),ticks=(0,0),cliff=None,armed=False):
    s=dict(gyro_rad_s=list(map(float,omega)),specific_force_m_s2=list(map(float,force)),left_ticks=int(round(ticks[0])),right_ticks=int(round(ticks[1])),encoder_cpr=CPR)
    if cliff is not None:s['cliff_m']=list(cliff)
    return m.apply(dict(sim_time_s=t,armed=armed,fault='none',sensors=s))
def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def worst_boards():
    """Every constant error at its bound, in four sign patterns (plus the warm-up drift at its bound)."""
    out=[]
    for sx,sy,sz in [(1,1,1),(-1,-1,-1),(1,-1,1),(-1,1,-1)]:
        s=np.array([sx,sy,sz],float);cross=np.full((3,3),.02*sx)-np.diag([.02*sx]*3)
        out.append(model(gyro_zro=s*20*DEG,gyro_tc=s*.16*DEG,gyro_scale=s*.03,gyro_cross=cross,accel_offset=s*np.array([50,50,80])*1e-3*G_UNIT,
                         accel_tc=s*np.array([.5,.5,.857])*1e-3*G_UNIT,accel_scale=s*.03,accel_cross=cross))
    return out

class SensorErrorModelStatistics(unittest.TestCase):
    def test_board_is_seeded_and_switchable(self):
        a,b=model(),model();self.assertEqual(a.board(),b.board())
        self.assertNotEqual(a.board(),model(seed=a.seed+1).board())
        raw=dict(sim_time_s=1.,sensors=dict(gyro_rad_s=[0,0,0],specific_force_m_s2=[0,0,G],left_ticks=5,right_ticks=5),ground_truth_evaluation_only=dict(yaw_rad=1.))
        off=SensorErrorModel(enabled=False);self.assertIs(off.apply(raw),raw)
        out=a.apply(raw);self.assertNotEqual(out['sensors']['gyro_rad_s'],[0,0,0]);self.assertIs(out['ground_truth_evaluation_only'],raw['ground_truth_evaluation_only'])
        config=ssm.load_config();self.assertEqual(config['errors'],'on')                   # default on (benchmarks)
        old=os.environ.get('R1_SENSOR_ERRORS')
        try:
            os.environ['R1_SENSOR_ERRORS']='off';self.assertFalse(SensorErrorModel().enabled)
            os.environ['R1_SENSOR_ERRORS']='on';self.assertTrue(SensorErrorModel(config=dict(config,errors='off')).enabled)
        finally:
            if old is None:os.environ.pop('R1_SENSOR_ERRORS',None)
            else:os.environ['R1_SENSOR_ERRORS']=old
    def test_every_number_has_a_source(self):
        config=ssm.load_config()
        def walk(d,path):
            for k,v in d.items():
                if isinstance(v,dict) and ('value' in v or 'xy' in v):
                    self.assertIn(v.get('source'),('datasheet','assumed','sim'),f'{path}.{k} has no source');self.assertTrue(v.get('ref'),f'{path}.{k} has no reference')
                elif isinstance(v,dict):walk(v,f'{path}.{k}')
        walk(config,'')
    def test_constant_errors_stay_within_the_json_bounds(self):
        c=ssm.load_config();gy,ac,od=c['gyro'],c['accelerometer'],c['odometry']
        boards=[model(seed=s) for s in range(400)]
        def check(values,bound):
            v=np.abs(np.array(values));self.assertTrue(np.all(v<=bound+1e-12));self.assertGreater(v.max(),.97*bound)   # uniform: it reaches the bound
        check([np.degrees(b.gyro_zro) for b in boards],gy['zero_rate_offset_dps']['value'])
        check([b.gyro_scale for b in boards],gy['scale_factor_tolerance']['value'])
        check([b.gyro_cross[~np.eye(3,dtype=bool)] for b in boards],gy['cross_axis']['value'])
        check([np.degrees(b.gyro_tc) for b in boards],gy['zro_temperature_coefficient_dps_per_c']['value'])
        mg=np.array([b.accel_offset/G_UNIT*1e3 for b in boards]);check(mg[:,:2],ac['zero_g_offset_mg']['xy']);check(mg[:,2],ac['zero_g_offset_mg']['z'])
        check([b.accel_scale for b in boards],ac['sensitivity_tolerance']['value']);check([b.accel_cross[~np.eye(3,dtype=bool)] for b in boards],ac['cross_axis']['value'])
        check([b.wheel_error for b in boards],od['wheel_radius_error']['value']);check([b.track_error for b in boards],od['track_error']['value'])
    def test_noise_density_quantization_and_full_scale(self):
        c=ssm.load_config();m=model(rise=0.);n=20000;bw=c['imu_sampling']
        w=np.array([m.gyro([0,0,0]) for _ in range(n)]);f=np.array([m.accel([0,0,G]) for _ in range(n)])
        # Per-sample sigma = density x sqrt(DLPF bandwidth) (ruling PB27: DLPF_CFG 4); the quantization adds < 1 %.
        self.assertTrue(np.allclose(np.degrees(w.std(0))/math.sqrt(bw['gyro_bandwidth_hz']),c['gyro']['noise_density_dps_per_rthz']['value'],rtol=.05),np.degrees(w.std(0)))
        self.assertTrue(np.allclose(f.std(0)/G_UNIT*1e6/math.sqrt(bw['accel_bandwidth_hz']),c['accelerometer']['noise_density_ug_per_rthz']['value'],rtol=.05),f.std(0))
        for x in (w,f):                                                  # white: no sample-to-sample correlation
            d=x-x.mean(0);self.assertTrue(np.all(np.abs((d[1:]*d[:-1]).mean(0)/d.var(0))<.03))
        self.assertTrue(np.allclose(w.mean(0),m.gyro_zro,atol=4*m.gyro_sigma/math.sqrt(n)+m.gyro_lsb))
        self.assertTrue(np.allclose(f.mean(0),m.accel_matrix@[0,0,G]+m.accel_offset,atol=4*m.accel_sigma/math.sqrt(n)+m.accel_lsb))
        lsb_g=math.radians(1/131);lsb_a=G_UNIT/16384
        self.assertTrue(np.allclose(w/lsb_g,np.round(w/lsb_g),atol=1e-6));self.assertTrue(np.allclose(f/lsb_a,np.round(f/lsb_a),atol=1e-6))
        self.assertTrue(np.allclose(m.gyro([20.,-20.,20.]),np.array([1,-1,1])*32767*lsb_g))     # +/-250 deg/s clips
        self.assertEqual(m.accel([0.,0.,40.])[2],32767*lsb_a)                                     # +/-2 g clips
    def test_firmware_sampling_is_the_same_on_both_sides(self):
        # Ruling PB27: DLPF_CFG 4 (gyro 20 Hz, accelerometer 21 Hz), samples at 20 Hz. The sim's model and the robot's
        # noise-dependent constants must describe the same firmware.
        c=ssm.load_config();s=c['imu_sampling']
        self.assertEqual((imu.IMU_DLPF_CFG,imu.IMU_BANDWIDTH_HZ,imu.IMU_RATE_HZ),(s['dlpf_cfg'],s['gyro_bandwidth_hz'],c['sample_rate_hz']['value']))
        self.assertAlmostEqual(imu.GYRO_SAMPLE_SIGMA,model().gyro_sigma)
        self.assertAlmostEqual(math.sqrt(imu.GYRO_R),2*imu.GYRO_SAMPLE_SIGMA)                 # 20 Hz <= 2 x bandwidth: no correlation factor
    def test_scale_cross_axis_and_warm_up(self):
        m=model();m.gyro_sigma=m.accel_sigma=0.;t=600.
        cols=np.array([m.gyro(np.eye(3)[i],t)-m.gyro_bias(t) for i in range(3)]).T
        self.assertTrue(np.allclose(cols,m.gyro_matrix,atol=2*m.gyro_lsb))
        cols=np.array([m.accel(G*np.eye(3)[i],t)-m.accel_bias(t) for i in range(3)]).T/G
        self.assertTrue(np.allclose(cols,m.accel_matrix,atol=2*m.accel_lsb/G))
        rise=m.rise*(1-math.exp(-1))
        self.assertTrue(np.allclose(m.gyro_bias(m.tau),m.gyro_zro+m.gyro_tc*rise));self.assertTrue(np.allclose(m.gyro_bias(1e6),m.gyro_zro+m.gyro_tc*m.rise))
        self.assertTrue(np.allclose(m.accel_bias(m.tau),m.accel_offset+m.accel_tc*rise))
        # The power-on clock follows sim time; an Unreal reset (time back to 0) does not rewind the warm-up.
        for t in (100.,100.05,100.1,0.,.05):m.apply(dict(sim_time_s=t,sensors={}))
        self.assertAlmostEqual(m.t_on,.15)
    def test_encoder_calibration_errors(self):
        m=model();el,er=m.wheel_error;et=m.track_error
        l,r=m.ticks(100000,100000);self.assertAlmostEqual((l+r)/2,(1e5/(1+el)+1e5/(1+er))/2,delta=1)
        l,r=m.ticks(-100000,100000);self.assertAlmostEqual((r-l)/2,(1e5/(1+el)+1e5/(1+er))/2*(1+et),delta=1)
        self.assertEqual(model(rise=0.).ticks(0,0),(0,0))
    def test_robot_code_never_sees_the_model_or_simulator_truth(self):
        for p in (CONTROLLER/'autonomy').rglob('*.py'):
            tree=ast.parse(p.read_text());docs={id(n.body[0].value) for n in ast.walk(tree) if isinstance(n,(ast.Module,ast.ClassDef,ast.FunctionDef)) and n.body and isinstance(n.body[0],ast.Expr)}
            imports={a.name.split('.')[0] for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names}|{(n.module or '').split('.')[0] for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)}
            self.assertNotIn('sim_sensor_model',imports,p.name)
            strings=[n.value for n in ast.walk(tree) if isinstance(n,ast.Constant) and isinstance(n.value,str) and id(n) not in docs]
            names={n.id for n in ast.walk(tree) if isinstance(n,ast.Name)}|{n.attr for n in ast.walk(tree) if isinstance(n,ast.Attribute)}
            for key in ('sensor-errors','sensor_errors','ground_truth'):self.assertFalse(any(key in s for s in strings),f'{p.name} uses {key}')
            self.assertFalse(names&{'SensorErrorModel','sensor_model','ground_truth_evaluation_only'},p.name)
        service=(CONTROLLER/'autonomy_service.py').read_text()
        strip=service.index("state={k:raw[k] for k in ['sim_time_s','sensors','armed','fault','stereo_frame','stereo_available']}")
        self.assertLess(strip,service.index('state=self.sensor_model.apply(state)'))

class RobotImuCalibration(unittest.TestCase):
    def test_installed_and_in_spec_calibrations_pass_the_documented_bounds(self):
        # The bounds are imu.LEVEL_* (single source, BRINGUP section 14 quotes them). An in-spec board stores up to
        # .69 / .69 / 1.08 m/s2 (offset + 3 % sensitivity + 2 % cross-axis): every one must pass, the installed file too.
        # The installed file carries the checks it passed and the limits they were made against: the current ones.
        cal=json.loads((ROOT/imu.CALIBRATION_FILE).read_text())
        self.assertEqual(cal.get('checks'),imu.calibration_checks(cal));self.assertTrue(all(cal['checks'].values()),cal['checks'])
        self.assertEqual(cal.get('limits'),imu.calibration_limits())
        worst=np.array([.05,.05,.08])*G_UNIT+np.array([.02,.02,.03])*G;self.assertTrue(np.all(worst<=np.asarray(imu.LEVEL_OFFSET_LIMIT_M_S2)),worst)
        for s in range(300):
            m=model(seed=s);off=(m.accel_matrix-np.eye(3))@[0,0,G]+m.accel_offset
            self.assertTrue(imu.calibration_checks(dict(accel_offset_m_s2=off.tolist()))['offsets_within_bounds'],(s,off))
        self.assertFalse(imu.calibration_checks(dict(accel_offset_m_s2=[0.,0.,imu.LEVEL_OFFSET_LIMIT_M_S2[2]+.05]))['offsets_within_bounds'])
        bad=dict(accel_offset_m_s2=[0.,0.,0.],heading_residual_m_s2=[[0.,0.,0.],[.3,0.,0.]]);self.assertFalse(imu.calibration_checks(bad)['heading_residuals_agree'])
        self.assertFalse(imu.calibration_checks(dict(accel_offset_m_s2=[0.,0.,0.],floor_attitude_rad=[[0.,.02]]))['floor_level'])
    def test_bringup_quotes_the_imu_numbers(self):
        # The numbers live in imu.py; BRINGUP section 14 (R1 repo) quotes them. R1_ROOT picks the R1 checkout (default: the
        # home-robot-r1 folder next to this SIM folder); without one the test is skipped, visibly (a SIM worktree has none).
        bringup=Path(os.environ.get('R1_ROOT') or ROOT.parent/'home-robot-r1')/'BRINGUP.md'
        if not bringup.exists():self.skipTest(f'no R1 BRINGUP.md at {bringup} (set R1_ROOT)')
        text=bringup.read_text();x,y,z=imu.LEVEL_OFFSET_LIMIT_M_S2
        for quote in (f'|x|, |y| ≤ {x}',f'|z| ≤ {z} m/s²',f'{imu.LEVEL_HEADING_DIFF_LIMIT_M_S2} m/s²',f'{imu.LEVEL_FLOOR_ATTITUDE_LIMIT_DEG}°',f'≤ {imu.YAW_COUPLING_LIMIT}','DLPF_CFG = 4'):
            self.assertTrue(quote in text,f'{bringup} does not quote {quote!r}')
    def test_warm_calibration_passes_the_documented_bounds(self):
        # Carry 20: BRINGUP prefers a calibration after ~10 min powered. The stored offsets then also hold the zero-g
        # temperature term (+/-.5 / .5 / .857 mg per C x the warm-up, 9.5 C after 10 min): a corner board stores .733 / .733 /
        # 1.158 m/s2, beyond the old .7 / .7 / 1.1 on every axis. It must pass, through the robot's own procedure, in all 8
        # sign patterns; so must 300 drawn boards, cold or warm.
        lim=np.asarray(imu.LEVEL_OFFSET_LIMIT_M_S2);warm=600.
        for signs in itertools.product((1.,-1.),repeat=3):
            s=np.array(signs);cross=np.zeros((3,3));cross[:2,2]=s[:2]*.02                     # the cross-axis terms the level offsets see
            m=model(accel_offset=s*np.array([50,50,80])*1e-3*G_UNIT,accel_scale=s*.03,accel_cross=cross,accel_tc=s*np.array([.5,.5,.857])*1e-3*G_UNIT)
            expected=(m.accel_matrix-np.eye(3))@[0,0,G]+m.accel_bias(warm)
            self.assertTrue(np.all(np.abs(expected)>[.7,.7,1.1]),expected)
            board=clone(m,777);board.t_on=warm
            with tempfile.TemporaryDirectory() as root:
                path,cal=calibration_tool.synthetic(root,board)                                   # write() raises if a check fails
            self.assertTrue(all(cal['checks'].values()),(signs,cal['checks']))
            self.assertTrue(np.allclose(cal['accel_offset_m_s2'],expected,atol=.01),(signs,cal['accel_offset_m_s2'],expected))
        for k in range(300):
            m=model(seed=k)
            for t_on in (0.,warm):
                off=(m.accel_matrix-np.eye(3))@[0,0,G]+m.accel_bias(t_on);self.assertTrue(np.all(np.abs(off)<=lim),(k,t_on,off))
    def test_write_installs_only_a_complete_passing_calibration(self):
        # Carry 22: write() runs the checks itself (a dict's own 'checks' are not trusted, and a dict without them is not a
        # pass), refuses an incomplete dict, and stores the checks with the limits they were made against.
        with tempfile.TemporaryDirectory() as root:good=calibration_tool.synthetic(root,clone(model(),5))[1]
        raw={k:v for k,v in good.items() if k not in ('checks','limits')};too_big=[0.,0.,imu.LEVEL_OFFSET_LIMIT_M_S2[2]+.05]
        for label,cal in (('no checks',dict(raw,accel_offset_m_s2=too_big)),('checks that claim a pass',dict(good,accel_offset_m_s2=too_big)),
                          ('incomplete',dict(accel_offset_m_s2=[0.,0.,0.],gravity_m_s2=9.8))):
            with tempfile.TemporaryDirectory() as root:
                with self.assertRaises(ValueError,msg=label):calibration_tool.write(cal,root)
                self.assertFalse((Path(root)/imu.CALIBRATION_FILE).exists(),label)
                self.assertTrue((Path(root)/imu.CALIBRATION_FILE).with_name('imu-calibration.rejected.json').exists(),label)
        with tempfile.TemporaryDirectory() as root:
            saved=json.loads(calibration_tool.write(raw,root).read_text())
        self.assertEqual(saved['checks'],imu.calibration_checks(saved));self.assertTrue(all(saved['checks'].values()));self.assertEqual(saved['limits'],imu.calibration_limits())
    def test_level_calibration_recovers_the_board_and_cancels_floor_tilt(self):
        m=model();body=(.0012,-.0009)                                   # body attitude on its wheels vs the floor (cliff plane)
        for tilt in (np.zeros(2),np.array([.5,-.3])*DEG):              # the floor's own tilt (body frame, heading 0)
            def rest(sign,k0):
                g_att=np.array(body)+sign*tilt;return [state(m,k0+k*DT,force=G*gravity_direction(*g_att),cliff=cliff_for(*body)) for k in range(100)]
            two=level_calibration([calibration_tool.to_samples(rest(1,0)),calibration_tool.to_samples(rest(-1,10))],imu.G_LOCAL)
            one=level_calibration([calibration_tool.to_samples(rest(1,20))],imu.G_LOCAL)
            for cal,bound in ((two,True),(one,not tilt.any())):
                c=ImuCalibration();c.loaded=True;c.accel_offset=np.array(cal['accel_offset_m_s2']);c.gravity=cal['gravity_m_s2']
                errs=[]
                for att in ((0,0),(3*DEG,-2*DEG),(-5*DEG,4*DEG)):        # level and tilted, true gravity attitude
                    f=np.mean([c.correct_force(m.accel(G*gravity_direction(*att))) for _ in range(40)],axis=0)
                    est=(math.atan2(f[1],f[2]),math.atan2(-f[0],math.hypot(f[1],f[2])));errs.append(np.degrees(np.abs(np.subtract(est,att))).max())
                if bound:self.assertLess(max(errs),.35,(tilt,errs))   # scale / cross-axis (not calibrated) leave < .35 deg up to 5 deg
                else:self.assertGreater(max(errs),.25)                # one heading keeps the floor's own tilt
    def test_spin_calibration_measures_the_yaw_coupling(self):
        # Spinning one way, then the other, on a flat floor; the body rests tilted (pitch .5 deg, the cliff floor plane
        # sees it), so the spin axis has a real body-x component that is motion, not coupling.
        for m in [model(rise=0.)]+[model(rise=0.,**{k:getattr(b,k) for k in ('gyro_zro','gyro_scale','gyro_cross')}) for b in worst_boards()]:
            p=.5*DEG;axis=np.array([-math.sin(p),0.,math.cos(p)])
            spins=[calibration_tool.to_samples([state(m,30+d*20+k*DT,omega=d*.5*axis,cliff=cliff_for(0.,p),armed=True) for k in range(500)]) for d in (1,-1)]
            c=imu.yaw_coupling(spins);M=m.gyro_matrix
            self.assertTrue(np.allclose(c,M[:2,2]/M[2,2],atol=5e-4),(c,M[:2,2]/M[2,2]))
            naive=(np.mean([s['gyro'] for s in spins[0]],0)-np.mean([s['gyro'] for s in spins[1]],0))
            self.assertGreater(abs(naive[0]/naive[2]-M[0,2]/M[2,2]),.006)          # without the tilt term: off by ~tan(.5 deg)
    def test_gyro_bias_calibration_converges_at_rest(self):
        boards=[model()]+[model(seed=s) for s in range(20)]+worst_boards()
        for m in boards:
            c=ImuCalibration();provisional=None
            for k in range(60):                                          # 3 s at rest, motors off
                s=state(m,k*DT);c.update(k*DT,s['sensors'])
                if c.bias_valid and provisional is None:provisional=k*DT
            self.assertTrue(c.startup_done);self.assertLessEqual(provisional,.3)
            self.assertLess(np.degrees(np.abs(c.bias-m.gyro_bias(m.t_on))).max(),.05,m.board())
    def test_zupt_ignores_the_settling_after_a_stop(self):
        # Unreal after a stop command: the wheel ramp (~.6 s from a turning drive: outer wheel ~.15 m/s at .25 m/s2), then 1-2 samples of residual body rotation (.005-.01 rad/s)
        # with 1-tick encoder steps (reports/physics-traces.json), then exactly still. A 1.2 s stop (LidarBrain 'sense').
        def run(skip):
            old=imu.ZUPT_SKIP_S;imu.ZUPT_SKIP_S=skip
            try:
                m=model(rise=0.);c=ImuCalibration();t=0.;ticks=np.zeros(2);errs=[]
                for k in range(60):s=state(m,t);c.update(t,s['sensors']);t+=DT              # power-on rest
                for cycle in range(6):
                    for k in range(int(20/DT)):                                                 # 20 s turning in place
                        w=.3;ticks+=np.array([-1,1])*w*TRACK/2*DT/(2*math.pi*WHEEL_R)*CPR
                        s=state(m,t,omega=(0,0,w),ticks=ticks,armed=True);c.update(t,s['sensors'],armed=True,requested=(0.,w));t+=DT
                    stop=[.3*(1-j/12) for j in range(12)]+[.009,.006]+[0.]*5                    # ramp, settling, a short rest
                    for j,w in enumerate(stop):
                        ticks+=np.array([-1,1])*(w*TRACK/2*DT/(2*math.pi*WHEEL_R)*CPR if j<12 else (1 if w else 0))
                        s=state(m,t,omega=(0,0,w),ticks=ticks,armed=True);c.update(t,s['sensors'],armed=True,requested=(0.,0.));t+=DT
                    errs.append(abs(np.degrees(c.bias[2]-m.gyro_bias()[2])))
                return max(errs)
            finally:imu.ZUPT_SKIP_S=old
        with_skip,without=run(imu.ZUPT_SKIP_S),run(0.)
        self.assertLess(with_skip,.05);self.assertGreater(without,.05);self.assertGreater(without,3*with_skip)   # measured .012 vs .073 deg/s
    def test_settling_skip_is_time_based(self):
        # The skip covers >= .15 s of a rest after driving at any sample rate: 20 Hz (PB27), or samples arriving at 100 Hz.
        # A timing test only: the samples come every 1 / rate s, while the robot's noise constants stay those of IMU_RATE_HZ
        # (test_gyro_r_follows_the_sample_rate covers their rate law) and the model's noise that of its DLPF bandwidth.
        for rate in (20.,100.):
            dt=1/rate;m=model(rise=0.);c=ImuCalibration();t=0.;ticks=np.zeros(2)
            for k in range(int(3*rate)):c.update(t,state(m,t)['sensors']);t+=dt                    # power-on rest
            for k in range(int(5*rate)):                                                             # 5 s turning
                ticks+=np.array([-1,1])*.3*TRACK/2*dt/(2*math.pi*WHEEL_R)*CPR
                c.update(t,state(m,t,omega=(0,0,.3),ticks=ticks,armed=True)['sensors'],armed=True,requested=(0.,.3));t+=dt
            n0=c.zupt_samples;first_used=None;first_still=None
            for k in range(int(2*rate)):                                                             # stopped (instantly)
                c.update(t,state(m,t,ticks=ticks,armed=True)['sensors'],armed=True,requested=(0.,0.))
                if c.still and first_still is None:first_still=t
                if c.zupt_samples>n0 and first_used is None:first_used=t
                t+=dt
            self.assertIsNotNone(first_used);self.assertGreaterEqual(first_used-first_still,imu.ZUPT_SKIP_S-1e-9,rate)
            self.assertLess(first_used-first_still,imu.ZUPT_SKIP_S+dt+1e-9,rate)
    def test_gyro_r_follows_the_sample_rate(self):
        # PB27: samples closer than the DLPF's time scale are correlated, so at most ~2 x bandwidth of them per second count as
        # independent: GYRO_R = (2 sigma)^2 x max(1, rate / (2 x bandwidth)). 20 Hz (the firmware rule) takes the first branch.
        r=(2*imu.GYRO_SAMPLE_SIGMA)**2
        self.assertEqual(imu.GYRO_R,imu.gyro_r(imu.IMU_RATE_HZ));self.assertAlmostEqual(imu.gyro_r(20.),r);self.assertAlmostEqual(imu.gyro_r(40.),r)
        self.assertAlmostEqual(imu.gyro_r(100.),2.5*r);self.assertAlmostEqual(imu.gyro_r(200.),5*r)
    def test_wheels_turning_with_a_steady_gyro_is_not_rest(self):
        # STILL_TICKS: the wheels slip on the spot (encoders count, the body and the gyro do not move): not a ZUPT.
        m=model(rise=0.);c=ImuCalibration();t=0.
        for k in range(60):c.update(t,state(m,t)['sensors']);t+=DT
        bias=c.bias.copy();n0=c.zupt_samples;ticks=np.zeros(2)
        for k in range(60):
            ticks+=np.array([12.,12.]);c.update(t,state(m,t,ticks=ticks,armed=True)['sensors'],armed=True,requested=(0.,0.));t+=DT
            self.assertFalse(c.still)
        self.assertEqual(c.zupt_samples,n0);self.assertTrue(np.allclose(c.bias,bias))
    def test_a_slow_hand_tilt_after_start_up_is_not_absorbed(self):
        # GYRO_OUTLIER: someone tilts the standing robot slowly (.35 deg/s about x, wheels still, steady): the encoders and
        # the sample-to-sample steadiness cannot see it, the outlier gate must, and the bias must not follow it.
        m=model(rise=0.);c=ImuCalibration();t=0.
        for k in range(100):c.update(t,state(m,t)['sensors']);t+=DT
        self.assertTrue(c.startup_done);bias=c.bias.copy();rate=math.radians(.35)
        for k in range(100):
            roll=rate*k*DT;c.update(t,state(m,t,omega=(rate,0,0),force=G*gravity_direction(roll,0.))['sensors']);t+=DT
        self.assertLess(np.degrees(abs(c.bias[0]-bias[0])),.05);self.assertFalse(c.still)
    def test_a_refused_rest_is_released_once_the_gyro_is_back_inside_the_gate(self):
        # Carry 18: a hand tilts the standing robot for 10 s (GYRO_OUTLIER refuses the rest: rest_vetoed), then lets go and the
        # robot stands still at its new attitude, motors off (handled once more for .2 s after .6 s). The refusal must hold
        # through the whole tilt (its gate is the one at the refusal: P grows while refused, so the live gate would open after
        # ~3 s) and end REST_VETO_RELEASE_S of steady rest inside that gate after the last handling, without driving: the bias
        # updates (ZUPT) and the at-rest attitude fusion resume.
        m=model(rise=0.);c=ImuCalibration();t=0.
        for k in range(100):c.update(t,state(m,t)['sensors']);t+=DT
        bias=c.bias.copy();rate=math.radians(.35);vetoed=[]
        for k in range(200):
            roll=rate*k*DT;c.update(t,state(m,t,omega=(rate,0,0),force=G*gravity_direction(roll,0.))['sensors']);t+=DT
            vetoed.append(c.status()['rest_vetoed'])
        self.assertTrue(all(vetoed[2:]),vetoed.index(False,2) if False in vetoed[2:] else None)
        self.assertLess(np.degrees(abs(c.bias[0]-bias[0])),.05)
        held=G*gravity_direction(roll,0.);n0=c.zupt_samples;released=None
        for k in range(80):
            wobble=12<=k<16;c.update(t,state(m,t,omega=(.05*(-1)**k,0.,0.) if wobble else (0.,0.,0.),force=held)['sensors'])
            if released is None and not c.rest_vetoed:released=k*DT
            t+=DT
        self.assertIsNotNone(released,'the refused rest never ends')
        steady=17*DT                                                     # the first steady sample after the handling
        self.assertTrue(steady+imu.REST_VETO_RELEASE_S-1e-9<=released<=steady+imu.REST_VETO_RELEASE_S+2*DT,released)
        self.assertTrue(c.still);self.assertGreater(c.zupt_samples,n0);self.assertFalse(c.status()['rest_vetoed'])
        self.assertLess(np.degrees(np.abs(c.bias-m.gyro_bias(m.t_on))).max(),.05)
    def test_zupt_tracks_the_warm_up_drift_between_drives(self):
        for m in worst_boards()[:2]:
            c=ImuCalibration();t=0.;ticks=np.zeros(2);worst=0.;yaw_rate=.3
            for cycle in range(55):                                      # 25 s turning in place, 3 s stop; ~26 min (the benchmark)
                for k in range(500+60):
                    moving=k<500;w=yaw_rate if moving else 0.;ticks+=np.array([-1,1])*w*TRACK/2*DT/(2*math.pi*WHEEL_R)*CPR
                    s=state(m,t,omega=(0,0,w),ticks=ticks,armed=True);c.update(t,s['sensors'],armed=True,requested=(0.,w))
                    t+=DT
                if cycle>0:worst=max(worst,np.degrees(abs(c.bias[2]-m.gyro_bias(m.t_on)[2])))
            self.assertLess(worst,.05);self.assertGreater(np.degrees(abs(m.gyro_bias(m.t_on)[2]-m.gyro_bias(0)[2])),1.5)   # it did drift > 1.5 deg/s
    def test_turn_in_place_heading_error_is_bounded(self):
        # Gyro-only dead reckoning (no LiDAR): after the start-up calibration, two full turns in place.
        for m in [model()]+worst_boards()[:2]:
            b=LidarBrain(calibrated_root(m));t=0.;ticks=np.zeros(2);yaw=0.;raw_yaw=0.
            for k in range(int(3/DT)):b.predict(state(m,t));t+=DT
            b.localizer.heading_uncertainty=.005;start=b.localizer.pose[2]
            for k in range(int(4*math.pi/.5/DT)+40):
                w=.5 if k<int(4*math.pi/.5/DT) else 0.;b.requested_motion=(0.,w);ticks+=np.array([-1,1])*w*TRACK/2*DT/(2*math.pi*WHEEL_R)*CPR
                s=state(m,t,omega=(0,0,w),ticks=ticks,armed=True);raw_yaw+=s['sensors']['gyro_rad_s'][2]*DT;b.predict(s);yaw+=w*DT;t+=DT
            err=abs(wrap(b.localizer.pose[2]-start-yaw));scale=abs(m.gyro_matrix[2,2]-1)*4*math.pi
            self.assertLessEqual(err,b.localizer.heading_uncertainty)          # the robot's uncertainty model covers it
            self.assertLess(abs(err-scale),math.radians(.25)*(t-3))           # beyond the (uncalibrated) scale error: < .25 deg/s of bias
            self.assertGreater(abs(raw_yaw-yaw),10*b.localizer.heading_uncertainty)   # without the start-up calibration: far off
    def test_turn_in_place_heading_with_lidar_in_the_loop(self):
        from sim2d import World,Sim,Bridge
        walls=[((-1,-1.5),(3,-1.5)),((3,-1.5),(3,1.5)),((3,1.5),(-1,1.5)),((-1,1.5),(-1,-1.5)),((1,-1.5),(1,.6)),((-.6,.9),(-.4,.9))]
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);sim=Sim(World(walls),sensor_errors=True);br=Bridge(b,sim,rearm_hook=lambda x:getattr(x,'wants_arm',False))
            self.assertTrue((Path(tmp)/imu.CALIBRATION_FILE).exists())
            for _ in range(10):b.predict(sim.state());[sim.step(.01) for _ in range(5)]
            br.start_mapping();trace=br.run(70)
        spins=[abs(wrap(r['est'][2]-r['truth'][2])) for r in trace if abs(r['w'])>.05 and abs(r['v'])<.01 and r['mode']!='idle']
        self.assertGreater(len(spins),100);self.assertLess(math.degrees(max(spins)),1.5)
        self.assertTrue(b.imu.startup_done);self.assertGreater(b.imu.zupt_samples,40)

class RobotAttitude(unittest.TestCase):
    def run_brain(self,m,root,segments,mapping=True,t0=0.):
        """Drive LidarBrain.predict (and drive_command's tilt gate) along synthetic truth segments:
        (seconds, v m/s, w rad/s, roll(t) rad, pitch(t) rad, floor valid). Returns per-sample truth/estimate rows."""
        b=LidarBrain(root);rows=[];t=t0;ticks=np.zeros(2);prev=None;v_prev=0.;v_act=w_act=0.
        if mapping:b.mode='mapping';b.phase='sense';b.started=t0;b.phase_started=t0;b.motor_seen_armed=True;b.localizer.uncertainty=.025
        for seconds,v,w,roll_f,pitch_f,floor in segments:
            n=int(round(seconds/DT))
            for k in range(n):
                u=k/n;roll,pitch=roll_f(u),pitch_f(u)
                if prev is None:prev=(roll,pitch)
                dr,dp=(roll-prev[0])/DT,(pitch-prev[1])/DT;prev=(roll,pitch)
                # The body follows the command through the motor core's wheel ramp (.25 m/s2; ~2.2 rad/s2 turning).
                v_act+=float(np.clip(v-v_act,-.25*DT,.25*DT));w_act+=float(np.clip(w-w_act,-2.2*DT,2.2*DT))
                a=np.array([(v_act-v_prev)/DT,v_act*w_act,0.]);v_prev=v_act
                force=G*gravity_direction(roll,pitch)+a
                ticks+=np.array([v_act-w_act*TRACK/2,v_act+w_act*TRACK/2])*DT/(2*math.pi*WHEEL_R)*CPR
                s=state(m,t,omega=body_rates(roll,pitch,dr,dp,w_act),force=force,ticks=ticks,cliff=cliff_for(roll,pitch) if floor else None,armed=True)
                b.requested_motion=(v,w);b.predict(s)
                if mapping and b.mode!='idle':
                    b.frame=SimpleNamespace(stamp=t,index=k);b.camera.last_stamp=t;b.localizer.last_visual_time=t;b.localizer.uncertainty=.025
                    b.drive_command(dict(s,armed=True,fault='none'))
                rows.append(dict(t=t,truth=(roll,pitch),est=tuple(b.attitude),init=b.attitude_initialized,mode=b.mode,message=b.message,tilt=b.tilt_rad))
                t+=DT
        return b,rows
    def test_attitude_initialises_and_stays_within_1_deg_of_truth(self):
        c=lambda x:(lambda u:x)
        ramp=lambda a,b_:(lambda u:a+(b_-a)*u)
        hump=lambda h:(lambda u:h*math.sin(math.pi*u))
        segments=[(3,0,0,c(0),c(0),True),(10,.1,0,c(0),c(0),True),(2,.1,0,c(0),hump(2*DEG),False),     # a threshold, floor plane lost
                  (4,.1,.2,ramp(0,3*DEG),c(0),True),(15,.1,0,c(3*DEG),c(0),False),                      # a tilted strip, IMU only
                  (5,0,0,c(3*DEG),c(0),False),(5,.1,0,ramp(3*DEG,0),ramp(0,-2*DEG),True),(5,.08,-.3,c(0),c(-2*DEG),False),(5,0,0,c(0),c(-2*DEG),False)]
        for m in [model()]+worst_boards():
            for warm in (0.,600.):
                m.t_on=warm;m.last_time=None
                b,rows=self.run_brain(m,calibrated_root(m),segments,mapping=False,t0=0.)
                first=next(r['t'] for r in rows if r['init']);self.assertLessEqual(first,.5)
                err=max(np.degrees(np.abs(np.subtract(r['est'],r['truth']))).max() for r in rows if r['init'])
                self.assertLess(err,1.,(m.board(),warm,err))
    def test_datasheet_corners_calibrate_and_initialise(self):
        """Seed-independent corners: every sign combination of the largest gyro zero-rate offset (+/-20 deg/s per axis)
        and the largest accelerometer offsets (+/-50 / 50 / 80 mg), scale errors +/-3 % with the same signs (the z norm
        error adds up: the old fixed gate fails). The start-up gyro calibration converges within .05 deg/s, and with the
        level calibration the attitude initialises at once and stays within 1 deg, level and on a 5 deg tilt."""
        flat=lambda u:0.
        for signs in itertools.product((1.,-1.),repeat=3):
            s=np.array(signs);m=model(gyro_zro=s*20*DEG,accel_offset=s*np.array([50,50,80])*1e-3*G_UNIT,gyro_scale=s*.03,accel_scale=s*.03)
            raw=m.accel_matrix@[0,0,G]+m.accel_offset;self.assertGreater(abs(np.linalg.norm(raw)-9.8),.3)
            c=ImuCalibration()
            for k in range(60):c.update(k*DT,state(m,k*DT)['sensors'])
            self.assertTrue(c.startup_done);self.assertLess(np.degrees(np.abs(c.bias-m.gyro_bias(m.t_on))).max(),.05,signs)
            m.t_on=0.;m.last_time=None
            b,rows=self.run_brain(m,calibrated_root(m),[(3,0,0,flat,flat,True),(3,.1,0,lambda u:5*DEG*u*s[0],lambda u:-5*DEG*u*s[1],False),
                                                        (5,0,0,lambda u:5*DEG*s[0],lambda u:-5*DEG*s[1],False)],mapping=False)
            first=next(r['t'] for r in rows if r['init']);self.assertLessEqual(first,.5,signs)
            err=max(np.degrees(np.abs(np.subtract(r['est'],r['truth']))).max() for r in rows if r['init']);self.assertLess(err,1.,(signs,err))
            root=calibrated_root(m);cal=json.loads((Path(root)/imu.CALIBRATION_FILE).read_text())
            self.assertTrue(all(imu.calibration_checks(cal).values()),(signs,cal['accel_offset_m_s2'],imu.calibration_checks(cal)))
    def test_old_fixed_gate_rejects_the_configured_board_new_gate_accepts(self):
        m=model();raw=np.mean([state(m,k*DT)['sensors']['specific_force_m_s2'] for k in range(40)],axis=0)
        self.assertGreater(abs(np.linalg.norm(raw)-9.8),.3)              # the stress board fails |f| - 9.8 < .3 uncalibrated
        b,rows=self.run_brain(m,calibrated_root(m),[(2,0,0,lambda u:0.,lambda u:0.,True)],mapping=False)
        self.assertTrue(b.attitude_initialized);self.assertLess(abs(np.linalg.norm(b.imu.correct_force(raw))-b.imu.gravity),GRAVITY_GATE/3)
    def test_without_a_calibration_file_the_floor_plane_brings_the_offsets_in(self):
        m=model();b,rows=self.run_brain(m,tempfile.mkdtemp(),[(120,0,0,lambda u:0.,lambda u:0.,True)],mapping=False)
        self.assertEqual(b.imu.level_source,'missing');first=next(r['t'] for r in rows if r['init'])
        self.assertLess(first,60.);self.assertLess(np.degrees(np.abs(b.attitude)).max(),.5)
    def test_tilt_stop_fires_on_a_real_tilt(self):
        m=model();tilt=.25
        b,rows=self.run_brain(m,calibrated_root(m),[(3,0,0,lambda u:0.,lambda u:0.,True),(.5,0,0,lambda u:0.,lambda u:tilt*u,False),(14,0,0,lambda u:0.,lambda u:tilt,False)])
        stop=next((r for r in rows if r['mode']=='idle'),None)
        self.assertIsNotNone(stop);self.assertIn('tilted',stop['message']);self.assertTrue(10.<=stop['t']-3.<=11.)
    def test_tilt_stop_never_fires_on_bias_alone(self):
        # Level floor, every constant error at its bound, warm-up drift at its fastest; mostly driving with short stops,
        # the floor plane lost for 10 s in every 30 s (rugs, thresholds). 10 min.
        level=lambda u:0.;segments=[]
        for k in range(20):segments+=[(8,.1,.25*(-1)**k,level,level,True),(10,.1,-.2,level,level,False),(10,.12,0,level,level,True),(2,0,0,level,level,True)]
        for m in worst_boards():
            b,rows=self.run_brain(m,calibrated_root(m),[(3,0,0,level,level,True)]+segments)
            self.assertTrue(all(r['mode']=='mapping' for r in rows),next((r['message'] for r in rows if r['mode']!='mapping'),''))
            self.assertLess(math.degrees(max(r['tilt'] for r in rows if r['init'])),1.)

class RobotStartupWait(unittest.TestCase):
    """Ruling PB28, carry 19 and 22: the robot does not move before its start-up gyro calibration is done (imu.GYRO_STARTUP_S of
    continuous rest), and says so when that rest never comes, in Mapping and in a Go-to's relocalization."""
    def test_mapping_at_power_on_does_not_move_before_the_start_up_calibration(self):
        from sim2d import World,Sim,Bridge
        walls=[((-1,-1.5),(3,-1.5)),((3,-1.5),(3,1.5)),((3,1.5),(-1,1.5)),((-1,1.5),(-1,-1.5)),((1,-1.5),(1,.6))]
        with tempfile.TemporaryDirectory() as tmp:
            sim=Sim(World(walls),sensor_errors=True);br=Bridge(LidarBrain(tmp),sim,rearm_hook=lambda x:getattr(x,'wants_arm',False))
            b=br.b=LidarBrain(tmp)          # a fresh power-on at the start pose: not the Bridge's PB28 pre-calibrated brain
            self.assertFalse(b.imu.startup_done);rows=[]
            br.start_mapping();t0=sim.t
            br.run(15,on_tick=lambda x:rows.append((x.trace[-1]['t']-t0,x.trace[-1]['v'],x.trace[-1]['w'],b.imu.startup_done)))
        moving=[r for r in rows if abs(r[1])>1e-6 or abs(r[2])>1e-6]
        self.assertTrue(moving,'mapping never started moving')
        self.assertTrue(all(r[3] for r in moving),next((r for r in moving if not r[3]),None))
        ready=next(r[0] for r in rows if r[3]);self.assertGreaterEqual(ready,imu.GYRO_STARTUP_S);self.assertGreaterEqual(moving[0][0],ready)
    def stand(self,b,m,seconds):
        """predict + drive_command for a robot on a level floor that is handled now and then: 1 s still, .5 s wobbled (gyro
        steps above STILL_GYRO_STEP), so the attitude initialises but 2 s of continuous rest never come. Returns (t, v, w)."""
        rows=[];t=0.
        for k in range(int(seconds/DT)):
            s=state(m,t,omega=(.05*(-1)**k,0.,0.) if k%30>=20 else (0.,0.,0.),cliff=cliff_for(0.,0.),armed=True);b.requested_motion=(0.,0.);b.predict(s)
            if b.mode!='idle':
                b.frame=SimpleNamespace(stamp=t,index=k);b.camera.last_stamp=t;b.localizer.last_visual_time=t;b.localizer.uncertainty=.025
                rows.append((t,)+tuple(b.drive_command(dict(s,armed=True,fault='none'))))
            t+=DT
        return rows
    def test_mapping_idles_with_its_own_message_when_never_still(self):
        m=model();b=LidarBrain(calibrated_root(m));b.mode='mapping';b.phase='sense';b.started=b.phase_started=0.;b.motor_seen_armed=True;b.scan_count=10
        rows=self.stand(b,m,40.)
        self.assertTrue(b.attitude_initialized);self.assertFalse(b.imu.startup_done);self.assertTrue(all(v==0 and w==0 for _,v,w in rows))
        self.assertEqual(b.mode,'idle');self.assertIn('gyro start-up calibration needs the robot standing still',b.message)
        self.assertTrue(30.<=rows[-1][0]<=31.,rows[-1][0])
    def test_go_to_relocalization_idles_with_the_gyro_message_when_never_still(self):
        m=model();b=LidarBrain(calibrated_root(m));b.mode='go_to';b.phase='relocalizing';b.started=b.phase_started=0.;b.motor_seen_armed=True;b.goal=np.array([1.,0.])
        rows=self.stand(b,m,50.)
        self.assertFalse(b.imu.startup_done);self.assertTrue(all(v==0 and w==0 for _,v,w in rows))
        self.assertEqual(b.mode,'idle');self.assertIn('gyro start-up calibration needs the robot standing still',b.message,b.message)
        self.assertTrue(30.<=rows[-1][0]<=31.,rows[-1][0])

if __name__=='__main__':unittest.main()
