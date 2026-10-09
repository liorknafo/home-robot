"""IMU level calibration at bring-up (bay B11): writes <root>/Content/RobotData/imu-calibration.json.

The procedure is the robot's (autonomy.imu.level_calibration): the robot stands still on a flat, plain floor with the
motors off; SECONDS of samples are recorded at one heading, the robot is turned 180 deg in place (by hand on the real
robot) and SECONDS more are recorded. Per-axis accelerometer offsets = mean specific force - g x the gravity
direction of the cliff ToF floor plane, averaged over the two headings (that cancels the floor's own tilt).
Then the spin calibration (autonomy.imu.yaw_coupling): the robot spins in place, SPIN_TURNS turns at +SPIN_W and
SPIN_TURNS at -SPIN_W (motors on, open floor: the circle rule); the x / y gyro output per unit yaw rate is stored. Two
turns each way: the body's own vibration while spinning (Unreal: ~1 deg/s rms per axis) averages out to ~.001.

Sample sources:
  live (default)  the running Unreal simulator, through RobotClient (so the sensor error model of
                  Content/RobotData/sensor-errors.json applies, exactly as for the robot). It binds UDP 8768 like the
                  autonomy service, so the service must be stopped. The robot is placed with Unreal's reset command
                  (the simulator's 'turn it by hand'), at --x/--y with yaw --yaw + 180 deg first, then --yaw, so it is
                  left at (--x, --y, --yaw), motors off.
  --synthetic     no simulator: the error model applied to a level robot at rest (the 2D harness, tests).
"""
import argparse,json,math,sys,time
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from autonomy.imu import level_calibration,CALIBRATION_FILE,G_LOCAL
from autonomy.lidar_brain import downward_floor_pose
from autonomy.geometry import CLIFF_Z
ROOT=Path(__file__).resolve().parents[1]

def to_samples(states):
    """robot_state dicts (measured sensors only) -> procedure samples."""
    out=[]
    for s in states:
        x=s['sensors'];floor=downward_floor_pose(np.asarray(x.get('cliff_m',[]),float)) if 'cliff_m' in x else None
        out.append(dict(force=list(map(float,x['specific_force_m_s2'])),gyro=list(map(float,x['gyro_rad_s'])),floor=None if floor is None else floor[:2].tolist()))
    return out

def write(calibration,root=ROOT):
    path=Path(root)/CALIBRATION_FILE;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(calibration,indent=2)+'\n');return path

SPIN_W=.5;SPIN_TURNS=2   # spin calibration: rate (rad/s) and full turns each way

def synthetic(root,model=None,seconds=5.,note=None):
    """Calibration file for a simulated robot without Unreal: the configured board at rest on a level floor, then
    spinning in place one turn each way."""
    from sim_sensor_model import SensorErrorModel,rest_states
    model=model or SensorErrorModel(enabled=True,run_seed=1)
    headings=[to_samples(rest_states(model,seconds,start=h*(seconds+5),cliff_m=[CLIFF_Z]*4)) for h in range(2)]
    n=int(SPIN_TURNS*2*math.pi/SPIN_W*model.rate);g=float(model.config['gravity_m_s2']['value'])
    spins=[to_samples([model.apply(dict(sim_time_s=30.+d*20+k/model.rate,armed=True,fault='none',sensors=dict(gyro_rad_s=[0.,0.,d*SPIN_W],specific_force_m_s2=[0.,0.,g])))
                       for k in range(n)]) for d in (1,-1)]
    cal=level_calibration(headings,G_LOCAL,note or f'synthetic: the level and spin calibration procedure on the simulated board (sensor-errors seed {model.seed}), level floor',spins)
    return write(cal,root),cal

def live(x=0.,y=0.,yaw=0.,seconds=5.,settle=2.,root=ROOT):
    from robot_client import RobotClient
    from sim_sensor_model import SensorErrorModel
    r=RobotClient();r.sensor_model=SensorErrorModel(run_seed=1);r.sock.settimeout(3.);headings=[];log=[]
    try:
        for heading in (yaw+math.pi,yaw):
            r.send(reset=True,x_m=x,y_m=y,yaw_rad=heading,motor_power_off=True);deadline=time.monotonic()+15
            while True:                                  # reset acknowledged: Unreal's clock restarts at 0
                s=r.receive();r.send(motor_power_off=True)
                if s['sim_time_s']<=.5:break
                if time.monotonic()>deadline:raise TimeoutError('Unreal did not acknowledge the reset')
            t0=s['sim_time_s'];rows=[]
            while True:
                s=r.receive();r.send(motor_power_off=True)
                if s['sim_time_s']-t0<settle:continue
                rows.append({k:s[k] for k in ('sim_time_s','armed','fault','sensors')})
                if s['sim_time_s']-t0>=settle+seconds:break
            for row in rows:row['sensors']={k:v for k,v in row['sensors'].items() if k!='lidar'}
            headings.append(to_samples(rows));log.append(dict(heading_rad=heading,samples=len(rows),armed_any=any(t['armed'] for t in rows)))
        spins=[]
        for d in (1,-1):                                 # spin in place one turn each way (after the wheel ramp)
            rows=[];t0=None;armed=False
            while True:
                r.send(0.,d*SPIN_W,**({} if armed else dict(arm=True,motor_power_on=True)));s=r.receive();armed=armed or s['armed']
                if not armed:continue
                t0=s['sim_time_s'] if t0 is None else t0
                if s['sim_time_s']-t0>1.:rows.append({k:s[k] for k in ('sim_time_s','armed','fault','sensors')})
                if s['sim_time_s']-t0>=1.+SPIN_TURNS*2*math.pi/SPIN_W:break
            for row in rows:row['sensors']={k:v for k,v in row['sensors'].items() if k!='lidar'}
            spins.append(to_samples(rows));log.append(dict(spin_rad_s=d*SPIN_W,samples=len(rows),faults=sorted({t['fault'] for t in rows})))
            for _ in range(30):r.send(0.,0.);r.receive()
        r.send(reset=True,x_m=x,y_m=y,yaw_rad=yaw,motor_power_off=True)    # back to the start pose, motors off
        for _ in range(60):r.receive();r.send(motor_power_off=True)
    finally:r.close()
    cal=level_calibration(headings,G_LOCAL,f'live: the running Unreal simulator through RobotClient with the sensor error model ({r.sensor_model.describe()}); '
                          f'robot placed at ({x}, {y}) with yaw {math.degrees(yaw)+180:.0f} deg then {math.degrees(yaw):.0f} deg by Unreal resets, {settle} s settle, {seconds} s each; '
                          f'then {SPIN_TURNS} turns in place each way at {SPIN_W} rad/s; finally reset to the start pose',spins)
    cal['collection']=log;return write(cal,root),cal

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--synthetic',action='store_true');p.add_argument('--seconds',type=float,default=5.);p.add_argument('--x',type=float,default=0.)
    p.add_argument('--y',type=float,default=0.);p.add_argument('--yaw',type=float,default=0.);p.add_argument('--root',type=Path,default=ROOT)
    a=p.parse_args()
    path,cal=synthetic(a.root,seconds=a.seconds) if a.synthetic else live(a.x,a.y,a.yaw,a.seconds,root=a.root)
    print(path);print(json.dumps({k:cal[k] for k in ('accel_offset_m_s2','gravity_m_s2','samples','floor_attitude_rad','heading_residual_m_s2')}))
