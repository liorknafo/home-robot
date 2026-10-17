"""Local-only Unreal bridge. Robot control consumes sensors, never simulator truth.
The gyro, accelerometer and encoders pass through the simulator's sensor error model (sim_sensor_model, B11)
unless sensor_errors=False (exact-regression runs of the simulator physics, e.g. tests/run_physics.py)."""
import json,math,socket,time
from sim_sensor_model import SensorErrorModel
from autonomy.geometry import WHEEL_R,TRACK,AXLE_X
from dataclasses import dataclass
@dataclass
class Odometry:
    x:float=0.;y:float=0.;yaw:float=0.
    last_left:float|None=None;last_right:float|None=None
    def update(self,sensors):
        left,right=sensors['left_ticks'],sensors['right_ticks']
        if self.last_left is not None:
            k=2*math.pi*WHEEL_R/sensors['encoder_cpr']
            dl=(left-self.last_left)*k;dr=(right-self.last_right)*k
            ds=(dl+dr)/2;da=(dr-dl)/TRACK
            # Integrates base_link; the axle is AXLE_X ahead of it (0 on the round robot).
            self.x+=ds*math.cos(self.yaw+da/2)+AXLE_X*(math.cos(self.yaw)-math.cos(self.yaw+da))
            self.y+=ds*math.sin(self.yaw+da/2)+AXLE_X*(math.sin(self.yaw)-math.sin(self.yaw+da));self.yaw+=da
        self.last_left,self.last_right=left,right
        return self
class RobotClient:
    def __init__(self,sensor_errors=None):
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.sensor_model=SensorErrorModel(enabled=sensor_errors)
        self.sock.bind(('127.0.0.1',8768));self.sock.settimeout(2.)
        self.odom=Odometry();self.state=None
    def send(self,v=0.,w=0.,**fields):
        payload={'v_m_s':v,'w_rad_s':w,**fields}
        self.sock.sendto(json.dumps(payload,allow_nan=False).encode(),('127.0.0.1',8767))
    def receive(self):
        while True:
            # Windows turns each earlier datagram that found no simulator listening into a reset on a later recv; skip them.
            try:raw=self.sock.recv(65535);break
            except ConnectionResetError:continue
        msg=self.sensor_model.apply(json.loads(raw));self.state=msg;self.odom.update(msg['sensors']);return msg
    def close(self):
        self.send(motor_power_off=True);self.sock.close()
    def settle(self,seconds=2):
        self.send(reset=True);self.odom=Odometry();start=None;deadline=time.monotonic()+15
        while True:
            s=self.receive();self.send()
            if start is None:
                if s['sim_time_s']>.5:
                    if time.monotonic()>deadline:raise TimeoutError('Reset was not acknowledged')
                    continue
                start=s['sim_time_s'];self.odom=Odometry();self.odom.update(s['sensors'])
            if s['sim_time_s']-start>=seconds:return s
    def hold(self,v,w,seconds,arm=False,capture=None):
        rows=[];start=None;first=True
        while True:
            self.send(v,w,arm=arm and first,**({'capture':capture} if capture is not None else {}));first=False
            s=self.receive();rows.append(s)
            if start is None:start=s['sim_time_s']
            if s['sim_time_s']-start>=seconds:return rows

def wrap(a):return math.atan2(math.sin(a),math.cos(a))
