"""One destination request, then read-only observation until autonomous Idle."""
import json,time,urllib.request,hashlib,math
from pathlib import Path
R=Path(__file__).resolve().parents[1]
def get(route):
 with urllib.request.urlopen('http://127.0.0.1:8771'+route,timeout=4) as f:return f.read()
def status():return json.loads(get('/api/status'))
initial=status()
if initial.get('mode')!='idle' or not initial.get('connected'):raise RuntimeError('Expected connected Idle robot')
goal=[0.,0.];folder=R/'reports'/time.strftime('go-to-%Y%m%dT%H%M%S');folder.mkdir()
sources=list((R/'controller').rglob('*.py'))+list((R/'Source/HomeRobotSim').glob('*.cpp'))+list((R/'Source/HomeRobotSim').glob('*.h'))+list((R/'Config').glob('*.ini'))+[R/'Content/RobotData/house-scene.json',R/'Content/RobotData/robot-parameters.json']
hashes={str(p.relative_to(R)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
mapping=R/'reports'/json.loads((R/'reports/latest-full-mapping-test.json').read_text())['directory'];mapping_trial=json.loads((mapping/'trial.json').read_text());origin=mapping_trial['map_origin_ground_truth_evaluation_only']
manifest=dict(goal=goal,start=initial,source_sha256=hashes,external_actions=[{'mode':'go_to','goal':goal}],policy='No resets or steering during navigation. Current physical pose is not sent to controller.',started_unix=time.time())
(folder/'trial.json').write_text(json.dumps(manifest,indent=2));(R/'reports/latest-go-to-test.json').write_text(json.dumps({'directory':folder.name}))
req=urllib.request.Request('http://127.0.0.1:8771/api/mode',data=json.dumps({'mode':'go_to','goal':goal}).encode(),headers={'Content-Type':'application/json'})
with urllib.request.urlopen(req) as response:assert response.status==202
print('TRIAL',folder.name,flush=True);active=False;start=time.monotonic();phases=[];last_report=0
with (folder/'trace.jsonl').open('w') as log:
 while time.monotonic()-start<1100:
  s=status();active=active or s['mode']=='go_to';phases.append(s['phase']);log.write(json.dumps(s)+'\n');log.flush()
  if time.monotonic()-last_report>30:print(json.dumps({k:s[k] for k in ['mode','phase','message','pose_m','camera_obstacle_cells']}),flush=True);last_report=time.monotonic()
  if (active and s['mode']=='idle') or (not active and time.monotonic()-start>8):
   (folder/'final-status.json').write_text(json.dumps(s,indent=2));(folder/'final-map.png').write_bytes(get('/api/map.png'));(folder/'final-camera.png').write_bytes(get('/api/camera.png'))
   session=R/'Saved/Stereo'/json.loads((R/'Saved/Stereo/latest-session.json').read_text())['directory'];meta=json.loads((session/'latest.json').read_text());truth=meta['ground_truth_evaluation_only'];c=math.cos(origin['yaw_rad']);ss=math.sin(origin['yaw_rad']);worldgoal=[origin['position_m'][0]+c*goal[0]-ss*goal[1],origin['position_m'][1]+ss*goal[0]+c*goal[1]];error=math.dist(worldgoal,truth['position_m'][:2]);arrived=s['message'].startswith('Arrived.')
   result=dict(passed=arrived and error<.15 and 'relocalizing' in phases and 'follow' in phases,arrived=arrived,goal_m=goal,actual_goal_distance_evaluation_only_m=error,phases=list(dict.fromkeys(phases)),reason=s['message'],duration_wall_s=time.monotonic()-start,external_actions=manifest['external_actions'],source_unchanged=all(hashlib.sha256(p.read_bytes()).hexdigest()==hashes[str(p.relative_to(R))] for p in sources),camera_obstacle_cells=s['camera_obstacle_cells'])
   (folder/'results.json').write_text(json.dumps(result,indent=2));print('FINISHED',json.dumps(result),flush=True);break
  time.sleep(.4)
