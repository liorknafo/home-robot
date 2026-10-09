"""Offline localization scoring of a full-mapping trial (bay B11). Read-only, evaluation only: never imported or read
by the controller. Usage: score_localization.py [--power-on-sim-s S] [reports/full-mapping-<stamp> ...] (default: the
latest trial; S = sim time of the autonomy service's first sample, for the gyro-bias tracking check).

Time-aligned errors: the robot's own estimate at each camera exposure (Saved/Maps/<map>/camera-inputs.jsonl, written
by LidarBrain.observe: the pose interpolated at the image time) against the simulator truth saved with that frame
(Saved/Stereo/<recording>/frame_N.json), in the world frame through the trial's map origin. Also the trace-based
position error of score_full_mapping.py (status pose vs the frame truth; includes the status-to-capture skew), the
same statistic as the B10 report, and the heading error on the same rows. With a B11 trace that logged the display
truth (roll / pitch) next to the status, the attitude error too."""
from pathlib import Path
import json,math,sys
import numpy as np
R=Path(__file__).resolve().parents[1]
def wrap(a):return (np.asarray(a)+np.pi)%(2*np.pi)-np.pi
def stats(x,scale=1.,digits=3):
    x=np.abs(np.asarray(x,float))*scale
    return dict(n=int(len(x)),median=round(float(np.median(x)),digits),p95=round(float(np.percentile(x,95)),digits),max=round(float(x.max()),digits)) if len(x) else dict(n=0)
def score(D,power_on_sim_s=None):
    D=Path(D);trial=json.loads((D/'trial.json').read_text());results=json.loads((D/'results.json').read_text()) if (D/'results.json').exists() else {}
    origin=trial['map_origin_ground_truth_evaluation_only'];o=np.array(origin['position_m'][:2]);oy=origin['yaw_rad'];c,s=math.cos(oy),math.sin(oy)
    rows=[json.loads(l) for l in (D/'trace.jsonl').read_text().splitlines()]
    saved=results.get('saved_map') or next((r['status'].get('map_folder') for r in rows if r['status'].get('map_folder')),None)
    folder=R/'Saved/Maps'/saved;rec=R/'Saved/Stereo'/trial['recording'];first=trial.get('first_capture_sim_time_s',0.)
    pos=[];yaw=[];ready=[];t=[]
    for line in (folder/'camera-inputs.jsonl').read_text().splitlines():
        x=json.loads(line);f=rec/f"frame_{x['meta']['frame']:06}.json"
        if not f.exists():continue
        truth=json.loads(f.read_text())['ground_truth_evaluation_only'];p=np.array(x['estimated_pose'][:3])
        world=o+np.array([c*p[0]-s*p[1],s*p[0]+c*p[1]])
        pos.append(np.linalg.norm(world-np.array(truth['position_m'][:2])));yaw.append(wrap(oy+p[2]-truth['yaw_rad']));ready.append(bool(x.get('geometry_ready')));t.append(x['meta']['sim_time_s']-first)
    pos,yaw,ready,t=map(np.array,(pos,yaw,ready,t))
    trace=[r for r in rows if 'ground_truth_evaluation_only' in r]
    tpos=[r['position_error_m'] for r in trace];tyaw=[wrap(oy+r['status']['pose_m'][2]-r['ground_truth_evaluation_only']['yaw_rad']) for r in trace]
    out=dict(trial=D.name,saved_map=saved,recording=trial['recording'],
             time_aligned_at_exposure=dict(frames=int(len(pos)),position_error_cm=stats(pos,100,2),heading_error_deg=stats(yaw,180/math.pi,3),
                                           heading_error_deg_geometry_ready=stats(yaw[ready],180/math.pi,3),signed_heading_mean_deg=round(float(np.degrees(yaw.mean())),3) if len(yaw) else None),
             trace_status_vs_capture=dict(rows=len(trace),position_error_cm=stats(tpos,100,2),heading_error_deg=stats(tyaw,180/math.pi,3),
                                          note='status can be newer than the capture (skew x speed / turn rate); the B10 report quoted this position statistic'))
    att=[r for r in rows if 'display_truth_evaluation_only' in r and r['status'].get('attitude_initialized') and r['display_truth_evaluation_only'].get('available')]
    if att:
        # Unreal pitch is nose up, ROS pitch nose down; Unreal roll and ROS roll share the sign here (checked on the
        # level floor and the scenario data).
        er=[r['status']['attitude_deg'][0]-r['display_truth_evaluation_only']['roll_deg'] for r in att]
        ep=[r['status']['attitude_deg'][1]+r['display_truth_evaluation_only']['pitch_deg'] for r in att]
        out['attitude_status_vs_display_truth']=dict(rows=len(att),roll_error_deg=stats(er,1,3),pitch_error_deg=stats(ep,1,3),tilt_error_deg=stats(np.hypot(er,ep),1,3),
            note='status attitude vs the /api/truth sample read right after it (up to ~0.3 s apart)')
    imu=[r['status']['imu'] for r in rows if isinstance(r['status'].get('imu'),dict)]
    if imu:out['robot_imu_last']=imu[-1]
    cfg=next((r['status'].get('sim_sensor_errors') for r in rows if r['status'].get('sim_sensor_errors')),None)
    if imu and cfg and cfg.get('enabled') and power_on_sim_s is not None:
        # Evaluation only: the robot's gyro-bias estimate against the simulator's true additive bias at that moment
        # (the error model with the run's seed; its clock started at the service's first sample, power_on_sim_s).
        sys.path.insert(0,str(R/'controller'));from sim_sensor_model import SensorErrorModel,load_config
        model=SensorErrorModel(enabled=True,config=dict(load_config(),seed=cfg['seed']))
        err=np.array([np.array(r['status']['imu']['gyro_bias_deg_s'])-np.degrees(model.gyro_bias(r['status']['sim_time_s']-power_on_sim_s)) for r in rows if isinstance(r['status'].get('imu'),dict)])
        drift=np.degrees(model.gyro_bias(rows[-1]['status']['sim_time_s']-power_on_sim_s)-model.gyro_bias(rows[0]['status']['sim_time_s']-power_on_sim_s))
        out['gyro_bias_tracking_deg_s']=dict(seed=cfg['seed'],power_on_sim_s=power_on_sim_s,z_error=stats(err[:,2],1,4),x_error=stats(err[:,0],1,4),y_error=stats(err[:,1],1,4),
                                             board_drift_during_run=drift.round(3).tolist())
    events=[r['status'].get('message','') for r in rows]
    out['tilt_or_attitude_stops']=sum('tilted or attitude unknown' in e for e in events);out['gyro_calibration_waits']=sum('Calibrating the gyro' in e for e in events)
    return out
if __name__=='__main__':
    args=sys.argv[1:];power=None
    if '--power-on-sim-s' in args:i=args.index('--power-on-sim-s');power=float(args[i+1]);del args[i:i+2]
    dirs=args or [R/'reports'/json.loads((R/'reports/latest-full-mapping-test.json').read_text())['directory']]
    print(json.dumps([score(d if Path(d).is_absolute() else R/d,power) for d in dirs],indent=2))
