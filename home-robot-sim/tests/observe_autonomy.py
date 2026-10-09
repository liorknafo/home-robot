"""Independent evaluator only. Never sends a pose, route or sensor value to the robot."""
import time,json,urllib.request,argparse,math
from pathlib import Path
R=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--seconds',type=int,default=1800);p.add_argument('--output',default='autonomy-live-trace.jsonl');a=p.parse_args()
end=time.monotonic()+a.seconds;last=None
with (R/'reports'/a.output).open('w') as stream:
 while time.monotonic()<end:
  try:
   status=json.load(urllib.request.urlopen('http://127.0.0.1:8771/api/status',timeout=2))
   folder=R/'Saved/Stereo'/json.loads((R/'Saved/Stereo/latest-session.json').read_text())['directory'];idx=status.get('stereo_frame');key=(folder.name,idx,status.get('phase'))
   if idx and key!=last:
    meta=json.loads((folder/f'frame_{idx:06}.json').read_text());truth=meta['ground_truth_evaluation_only'];pose=status['pose_m'];age=status['sim_time_s']-meta['sim_time_s']
    row=dict(status=status,recording=folder.name,ground_truth_evaluation_only=truth,camera_status_age_s=age,position_error_m=math.hypot(pose[0]-truth['position_m'][0],pose[1]-truth['position_m'][1]),origin_assumption='Fresh simulator start, mapping origin aligned with spawn; status may be up to one capture period newer than evaluation frame')
    stream.write(json.dumps(row)+'\n');stream.flush();last=key
  except (OSError,ValueError,KeyError,TypeError):pass
  time.sleep(.7)
