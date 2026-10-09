"""SIMULATOR ONLY: realistic IMU and odometry errors (bay B11).

Unreal sends an ideal gyro (the physics body's exact angular velocity), an ideal accelerometer (finite-differenced
velocity + g) and encoders with exact calibration. This layer turns them into what an MPU-6050 (GY-521) and
imperfectly calibrated wheels deliver, using Content/RobotData/sensor-errors.json (datasheet / assumed values,
one seed). It is applied where Unreal's raw state enters the controller: autonomy_service.RobotIO (after simulator
truth is stripped), robot_client.RobotClient.receive and the 2D test harness (tests/sim2d.py). The robot code
(controller/autonomy) never imports this module or reads that file; it must cope with the errors on its own
(controller/autonomy/imu.py).

Model, per robot_state sample (body axes as the robot sees them):
  gyro  w_m = Q(clip(M_g w + b_g0 + k_g dT(t) + n_g))     M_g = I + diag(scale) + cross-axis (off-diagonal)
  accel f_m = Q(clip(M_a f + b_a0 + k_a dT(t) + n_a))
  dT(t) = rise (1 - exp(-t / tau)): warm-up from power-on (the first sample this instance sees)
  n ~ N(0, (density x sqrt(sample rate))^2) per axis, Q = the LSB quantization, clip = the full scale
  encoders: per-wheel effective radius (ticks / (1 + e_wheel)), effective track (L-R difference x (1 + e_track))
"""
import json,math,os
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
CONFIG_PATH=ROOT/'Content/RobotData/sensor-errors.json'
G_UNIT=9.80665          # m/s2 per g: the accelerometer's LSB unit

def load_config(path=CONFIG_PATH):
    return json.loads(Path(path).read_text())

def errors_enabled(config,override=None):
    """Explicit override (True/False) > environment R1_SENSOR_ERRORS (on/off) > the file's 'errors'."""
    if override is not None:return bool(override)
    env=os.environ.get('R1_SENSOR_ERRORS','').strip().lower()
    if env in ('on','1','true','yes'):return True
    if env in ('off','0','false','no'):return False
    return str(config.get('errors','on')).lower()=='on'

class SensorErrorModel:
    """One simulated board (constant errors drawn from the config seed) plus its white noise and warm-up clock."""
    def __init__(self,enabled=None,run_seed=0,config=None,config_path=CONFIG_PATH):
        self.config=config if config is not None else load_config(config_path)
        self.enabled=errors_enabled(self.config,enabled);self.run_seed=int(run_seed)
        # R1_SENSOR_SEED: another board for a contrast run (only when no config was passed in explicitly).
        self.seed=int(os.environ['R1_SENSOR_SEED']) if config is None and os.environ.get('R1_SENSOR_SEED','').strip() else int(self.config['seed'])
        c=self.config;g=c['gyro'];a=c['accelerometer'];o=c['odometry'];w=c['warm_up']
        self.rate=float(c['sample_rate_hz']['value'])
        self.gyro_lsb=math.radians(1/g['sensitivity_lsb_per_dps']['value']);self.gyro_full=32767*self.gyro_lsb
        self.accel_lsb=G_UNIT/a['sensitivity_lsb_per_g']['value'];self.accel_full=32767*self.accel_lsb
        self.gyro_sigma=math.radians(g['noise_density_dps_per_rthz']['value'])*math.sqrt(self.rate)
        self.accel_sigma=a['noise_density_ug_per_rthz']['value']*1e-6*G_UNIT*math.sqrt(self.rate)
        self.rise=float(w['temperature_rise_c']['value']);self.tau=float(w['time_constant_s']['value'])
        # The board: drawn once, in a fixed order, from the config seed (uniform within each bound).
        r=np.random.default_rng(self.seed);u=lambda bound,n:r.uniform(-1,1,n)*np.asarray(bound,float)
        self.gyro_zro=u(math.radians(g['zero_rate_offset_dps']['value']),3)                     # rad/s
        self.gyro_scale=u(g['scale_factor_tolerance']['value'],3)
        self.gyro_cross=self._cross(u(g['cross_axis']['value'],6))
        self.gyro_tc=u(math.radians(g['zro_temperature_coefficient_dps_per_c']['value']),3)     # rad/s per C
        mg=a['zero_g_offset_mg'];tc=a['zero_g_temperature_coefficient_mg_per_c']
        self.accel_offset=u(np.array([mg['xy'],mg['xy'],mg['z']])*1e-3*G_UNIT,3)               # m/s2
        self.accel_scale=u(a['sensitivity_tolerance']['value'],3)
        self.accel_cross=self._cross(u(a['cross_axis']['value'],6))
        self.accel_tc=u(np.array([tc['xy'],tc['xy'],tc['z']])*1e-3*G_UNIT,3)                   # m/s2 per C
        self.wheel_error=u(o['wheel_radius_error']['value'],2)                                   # left, right
        self.track_error=float(u(o['track_error']['value'],1)[0])
        self.noise=np.random.default_rng([self.seed,self.run_seed,11])
        self.t_on=0.;self.last_time=None
    @staticmethod
    def _cross(v):
        m=np.zeros((3,3));m[[0,0,1,1,2,2],[1,2,0,2,0,1]]=v;return m
    @property
    def gyro_matrix(self):return np.eye(3)+np.diag(self.gyro_scale)+self.gyro_cross
    @property
    def accel_matrix(self):return np.eye(3)+np.diag(self.accel_scale)+self.accel_cross
    def temperature_rise(self,t_on=None):
        t=self.t_on if t_on is None else t_on;return self.rise*(1-math.exp(-max(0.,t)/self.tau))
    def gyro_bias(self,t_on=None):
        """Additive gyro error (rad/s) at power-on time t_on: initial ZRO + warm-up drift (for tests and offline scoring)."""
        return self.gyro_zro+self.gyro_tc*self.temperature_rise(t_on)
    def accel_bias(self,t_on=None):
        return self.accel_offset+self.accel_tc*self.temperature_rise(t_on)
    def advance(self,sim_time):
        if self.last_time is not None:self.t_on+=min(.5,max(0.,float(sim_time)-self.last_time))
        self.last_time=float(sim_time);return self.t_on
    def gyro(self,w,t_on=None):
        w=self.gyro_matrix@np.asarray(w,float)+self.gyro_bias(t_on)+self.noise.normal(0,self.gyro_sigma,3)
        return np.round(np.clip(w,-self.gyro_full,self.gyro_full)/self.gyro_lsb)*self.gyro_lsb
    def accel(self,f,t_on=None):
        f=self.accel_matrix@np.asarray(f,float)+self.accel_bias(t_on)+self.noise.normal(0,self.accel_sigma,3)
        return np.round(np.clip(f,-self.accel_full,self.accel_full)/self.accel_lsb)*self.accel_lsb
    def ticks(self,left,right):
        """Encoder counts the robot would read if its wheels' effective radii and track differed from nominal."""
        ml=left/(1+self.wheel_error[0]);mr=right/(1+self.wheel_error[1])
        mean=(ml+mr)/2;half=(mr-ml)/2*(1+self.track_error)
        return int(round(mean-half)),int(round(mean+half))
    def apply(self,state):
        """A copy of a robot_state with the measured sensors replaced (gyro, accelerometer, encoders). Other
        fields, including any simulator truth a caller kept for evaluation, are passed through untouched."""
        if not self.enabled or not isinstance(state,dict) or not isinstance(state.get('sensors'),dict):return state
        t=self.advance(state.get('sim_time_s',self.last_time or 0.))
        s=dict(state['sensors'])
        if 'gyro_rad_s' in s:s['gyro_rad_s']=self.gyro(s['gyro_rad_s'],t).tolist()
        if 'specific_force_m_s2' in s:s['specific_force_m_s2']=self.accel(s['specific_force_m_s2'],t).tolist()
        if 'left_ticks' in s and 'right_ticks' in s:s['left_ticks'],s['right_ticks']=self.ticks(s['left_ticks'],s['right_ticks'])
        return dict(state,sensors=s)
    def describe(self):
        return dict(enabled=self.enabled,seed=self.seed,run_seed=self.run_seed,config=str(CONFIG_PATH.relative_to(ROOT)))
    def board(self):
        """The drawn constants in readable units (reports and tests; never given to the robot)."""
        return dict(gyro_zero_rate_offset_dps=np.degrees(self.gyro_zro).round(4).tolist(),gyro_scale_error=self.gyro_scale.round(5).tolist(),
                    gyro_cross_axis=self.gyro_cross.round(5).tolist(),gyro_zro_tc_dps_per_c=np.degrees(self.gyro_tc).round(5).tolist(),
                    accel_offset_mg=(self.accel_offset/G_UNIT*1e3).round(2).tolist(),accel_scale_error=self.accel_scale.round(5).tolist(),
                    accel_cross_axis=self.accel_cross.round(5).tolist(),accel_tc_mg_per_c=(self.accel_tc/G_UNIT*1e3).round(4).tolist(),
                    wheel_radius_error=self.wheel_error.round(5).tolist(),track_error=round(self.track_error,5))

def rest_states(model,seconds,gravity=None,start=0.,rate=None,cliff_m=None):
    """Simulated robot_states of a robot standing level and still on a flat floor (ideal values before the model):
    used where no simulator runs (the 2D harness, tests) to feed the robot's own level-calibration procedure."""
    g=float(model.config['gravity_m_s2']['value']) if gravity is None else gravity;rate=rate or model.rate;out=[]
    for k in range(int(round(seconds*rate))):
        sensors=dict(gyro_rad_s=[0.,0.,0.],specific_force_m_s2=[0.,0.,g],left_ticks=0,right_ticks=0)
        if cliff_m is not None:sensors['cliff_m']=list(cliff_m)
        out.append(model.apply(dict(sim_time_s=start+k/rate,armed=False,fault='disarmed',sensors=sensors)))
    return out
