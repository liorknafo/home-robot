"""Start Mapping once, then observe only. No velocity, waypoints, resets or retries.
Simulator truth is read solely into an evaluation report, never sent to the robot.
"""
from pathlib import Path
import json,time,hashlib,urllib.request,math,shutil
R=Path(__file__).resolve().parents[1]
def get(route):
    with urllib.request.urlopen('http://127.0.0.1:8771'+route,timeout=3) as f:return f.read()
def status():return json.loads(get('/api/status'))
def load(p):return json.loads(p.read_text())
initial=status()
if initial.get('mode')!='idle' or not initial.get('connected'):raise RuntimeError('Start only with connected simulator and Idle robot')
pointer=R/'Saved/Stereo/latest-session.json'
folder=R/'Saved/Stereo'/load(pointer)['directory'] if pointer.exists() else None
previous_frame=load(folder/'latest.json')['frame'] if folder and (folder/'latest.json').exists() else 0
D=R/'reports'/time.strftime('full-mapping-%Y%m%dT%H%M%S');D.mkdir()
sources=list((R/'controller').rglob('*.py'))+list((R/'Source/HomeRobotSim').glob('*.cpp'))+list((R/'Source/HomeRobotSim').glob('*.h'))+list((R/'Config').glob('*.ini'))+list((R/'Content/RobotData').glob('*.json'))+list((R/'Content/HouseMaterials').glob('*.uasset'))+list((R/'Content/RobotMaterials').glob('*.uasset'))
hashes={str(p.relative_to(R)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
manifest=dict(start_state=initial,previous_recording_frame=previous_frame,recording=folder.name if folder else None,source_sha256=hashes,external_actions=[{'mode':'mapping'}],policy='One Mapping start. All subsequent operations are read-only observations. No steering, goals, reset, retry or software changes during this trial.',robot_simulation_time_limit_s=1500,started_unix=time.time())
(D/'trial.json').write_text(json.dumps(manifest,indent=2));(R/'reports/latest-full-mapping-test.json').write_text(json.dumps({'directory':D.name}))
for source in sources:
 dst=D/'source-snapshot'/source.relative_to(R);dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dst)
(D/'test-setup.json').write_text(json.dumps(dict(reset_to_standard_spawn=True,position_m=[0,0],yaw_rad=0,reason='Fresh Unreal launch before test; no resets or steering after Mapping start',sensor_configuration='d200_single_camera_with_visual_obstacles'),indent=2))
request=urllib.request.Request('http://127.0.0.1:8771/api/mode',data=b'{"mode":"mapping"}',headers={'Content-Type':'application/json'})
with urllib.request.urlopen(request,timeout=3) as response:assert response.status==202
print('TRIAL',D.name,flush=True)
origin=None;active=False;last_key=None;last_image=0;start=time.monotonic();last_report=0
with (D/'trace.jsonl').open('w') as stream:
 while time.monotonic()-start<3600:
  try:
   s=status();active=active or s['mode']=='mapping'
   current_folder=R/'Saved/Stereo'/load(R/'Saved/Stereo/latest-session.json')['directory']
   if origin is None and current_folder!=folder:
    folder=current_folder;previous_frame=0;manifest['recording']=folder.name;manifest['previous_recording_frame']=0
   if origin is None and (folder/f'frame_{previous_frame+1:06}.json').exists():
    m=load(folder/f'frame_{previous_frame+1:06}.json');origin=m['ground_truth_evaluation_only'];manifest['map_origin_ground_truth_evaluation_only']=origin;manifest['first_capture_sim_time_s']=m['sim_time_s'];(D/'trial.json').write_text(json.dumps(manifest,indent=2))
   key=(s['stereo_frame'],s['mode'],s['phase'])
   if active and key!=last_key:
    row=dict(observed_unix=time.time(),status=s)
    # B11, evaluation only: the simulator's body attitude for scoring the robot's roll / pitch offline (display endpoint).
    try:row['display_truth_evaluation_only']=json.loads(get('/api/truth'))
    except (OSError,ValueError):pass
    if s['stereo_frame'] and origin:
     m=load(folder/f'frame_{s["stereo_frame"]:06}.json');truth=m['ground_truth_evaluation_only'];row['ground_truth_evaluation_only']=truth;row['capture_sim_time_s']=m['sim_time_s'];p=s['pose_m'];c=math.cos(origin['yaw_rad']);ss=math.sin(origin['yaw_rad']);world=[origin['position_m'][0]+c*p[0]-ss*p[1],origin['position_m'][1]+ss*p[0]+c*p[1]];row['estimated_world_xy_evaluation_only']=world;row['position_error_m']=math.dist(world,truth['position_m'][:2]);row['capture_status_age_s']=s['sim_time_s']-m['sim_time_s']
    stream.write(json.dumps(row)+'\n');stream.flush();last_key=key
   if active and time.monotonic()-last_image>30:
    (D/'latest-map.png').write_bytes(get('/api/map.png'));(D/'latest-camera.png').write_bytes(get('/api/camera.png'));last_image=time.monotonic()
   if active and time.monotonic()-last_report>30:
    print(json.dumps({k:s[k] for k in ['mode','phase','message','pose_m','map','sim_time_s']}),flush=True);last_report=time.monotonic()
   if active and s['mode']=='idle':
    (D/'final-status.json').write_text(json.dumps(s,indent=2));(D/'final-map.png').write_bytes(get('/api/map.png'));(D/'final-camera.png').write_bytes(get('/api/camera.png'));manifest['ended_unix']=time.time();manifest['termination']='robot independently returned to Idle';manifest['source_unchanged']=all(hashlib.sha256(p.read_bytes()).hexdigest()==hashes[str(p.relative_to(R))] for p in sources);(D/'trial.json').write_text(json.dumps(manifest,indent=2));print('FINISHED',s['message'],flush=True);break
  except (OSError,ValueError,KeyError) as exc:
   (D/'observer-errors.txt').open('a').write(str(exc)+'\n')
  time.sleep(.75)
 else:
  manifest['termination']='observer time limit; no command sent to robot';(D/'trial.json').write_text(json.dumps(manifest,indent=2))
