"""Unreal replay of task G: a bumper contact with the bathroom shower tray (6 cm, LiDAR-blind) with no straight trail behind.
Usage: unreal_contact_replay.py APPROACH [seconds_after_touch=60]   APPROACH: weave | spin | spin_start | turn
Live simulator only, under the .sim-lock (docs/WORKFLOW.md), with the autonomy service STOPPED (this script binds UDP 8768).
It resets the robot near the tray, runs the autonomy Application in-process with the controller of THIS tree (camera frames are read
from the simulator's Saved/Stereo in SIM_ROOT, default the main SIM folder), starts Mapping, drives a scripted approach into the
tray's front edge under the robot's own command() guard (as sim2d.run_contact_recovery does), and from the touch on lets the
controller decide everything.
  weave: the last 4 s of trial full-mapping-20261011T054731 before it met the tray: its start pose there (truth at sim 426.0 s) and
         decision commands, then straight on at 10 cm/s (that trial then waited, pressed, for 1110 s);
  spin: 17 cm east along the tray front (skin 1.6 cm from it), a quarter turn in place to face it, then straight on;
  spin_start: the quarter turn at the start of the session (no trail behind but the start position), then straight on;
  turn (task P): the last 1.4 s of GI trial full-mapping-20261012T183238 before it met the tray: its truth pose at sim 537.97 s, 10 cm/s
         straight for .95 s, the local planner's -.4 rad/s commit for .45 s, then straight on (that trial's back-off was refused
         by the guard after the body rolled on into the skin, and it stopped 'Stuck' 70 s later).
Writes $SIM_ROOT/reports/contact-replay-<approach>-<time>/ (decisions.jsonl, result.json). Simulator truth is read for the report
only (the Application's display endpoint); it never reaches the brain.
PASS: released within 15 s of the touch, out of the contact phase and >= .3 m from the touch at the end, no tick off the driven trail
(reverse_audit's off_trail_ticks == 0) and no commanded reverse episode over .16 m. reverse_audit's own 'ok' is reported but not
required: its episode length also counts base_link moving backward without a reverse command, which here includes the drift of a
turn in place (the weave replay: 7.4 cm of it during a ~150 deg turn)."""
import json,math,os,socket,sys,threading,time
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parents[1]
SIM=Path(os.environ.get('SIM_ROOT','/Users/liorknafo/Documents/Codex/2026-10-03/i-x20/outputs/home-robot-sim'))
sys.path.insert(0,str(HERE/'controller'));sys.path.insert(0,str(HERE/'tests'))
import autonomy_service as svc
from sim2d import CONTACT_APPROACHES,reverse_audit
approach=sys.argv[1];after=float(sys.argv[2]) if len(sys.argv)>2 else 60.
START={'weave':(4.015,-1.657,-1.313),'spin':(3.90,-1.955,0.),'spin_start':(4.07,-1.955,0.),'turn':(4.0645,-1.845,-1.4422)}[approach]
SCRIPT={'weave':CONTACT_APPROACHES['weave'],'spin':[(.1,0.,1.7),(0.,-.4,math.pi/2/.4)],'spin_start':[(0.,-.4,math.pi/2/.4)],'turn':[(.1,0.,.95),(.1,-.4,.45)]}[approach]
OUT=SIM/'reports'/time.strftime(f'contact-replay-{approach}-%Y%m%dT%H%M%S');OUT.mkdir(parents=True)
(HERE/'reports').mkdir(exist_ok=True)   # the Application writes reports/autonomy-status.json under its own tree
# Reset the robot (simulator command, before the Application binds the state port) and wait until it rests there.
cmd=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);st=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);st.bind(('127.0.0.1',8768));st.settimeout(1.)
def send(**o):cmd.sendto(json.dumps(o).encode(),('127.0.0.1',8767))
send(capture=False);send(v_m_s=0.,w_rad_s=0.,arm=False,motor_power_off=True);time.sleep(.5)
send(reset=True,x_m=START[0],y_m=START[1],z_m=0.,yaw_rad=START[2]);time.sleep(1.)
for _ in range(80):
    s=json.loads(st.recv(65535));g=s.get('ground_truth_evaluation_only') or {};p=g.get('position_m') or [9,9]
    if math.dist(p[:2],START[:2])<.03 and not s['sensors'].get('bumper'):break
    time.sleep(.1)
else:raise RuntimeError(f'reset to {START} not reached: {p}')
st.close();cmd.close()
# Camera frames come from the simulator's folder; everything else (maps, status) stays in this tree.
read_pair=svc.Application.read_pair
def read_sim_pair(self,state):
    root=svc.ROOT;svc.ROOT=SIM
    try:return read_pair(self,state)
    finally:svc.ROOT=root
svc.Application.read_pair=read_sim_pair
app=svc.Application('lidar');b=app.brain;real=b.drive_command;state=dict(t0=None,touch=None,touch_truth=None,released=None,step=False)
def scripted(s):
    if state['touch'] is not None or b.contacts or b.phase=='contact' or b.phase not in ('plan','follow') or not b.imu.ready_to_move():return real(s)
    t=s['sim_time_s'];state['t0']=t if state['t0'] is None else state['t0'];k=t-state['t0'];b.phase='follow'
    for v,w,d in SCRIPT:
        if k<d:return v,w
        k-=d
    return .1,0.
b.drive_command=scripted
rows=[];command=b.command
def logged(s):
    # One row per simulator state (the display truth arrives with it), with the command decided on it.
    v,w=command(s);truth=app.io.truth_for_display() or {}
    if rows and rows[-1]['t']==s['sim_time_s']:rows[-1].update(v=v,w=w);return v,w
    rows.append(dict(t=s['sim_time_s'],truth_t=truth.get('sim_time_s'),v=v,w=w,phase=b.phase,bumper=bool(s['sensors'].get('bumper')),est=b.localizer.pose.tolist(),
                     truth=truth.get('position_m',[0,0])[:2]+[truth.get('yaw_rad',0.)],msg=b.message))
    return v,w
b.command=logged
thread=threading.Thread(target=app.run,daemon=True);thread.start()
deadline=time.monotonic()+30
while not (getattr(b.imu,'startup_done',False) and app.io.latest()):
    if time.monotonic()>deadline:raise RuntimeError('IMU start-up calibration did not complete')
    time.sleep(.2)
app.commands.put({'mode':'mapping'});start=time.monotonic();end=None
while True:
    time.sleep(.1)
    if rows and state['touch'] is None and (b.contacts or rows[-1]['bumper']):state['touch']=rows[-1]['t'];state['touch_truth']=rows[-1]['truth'];end=time.monotonic()+after
    if state['touch'] is not None and state['released'] is None and rows and not rows[-1]['bumper'] and rows[-1]['t']>state['touch']:state['released']=rows[-1]['t']
    if (end and time.monotonic()>end) or b.mode=='idle' and time.monotonic()-start>5 or time.monotonic()-start>after+120:break
app.stop=True;thread.join(timeout=5)
with (OUT/'decisions.jsonl').open('w') as f:
    for r in rows:f.write(json.dumps(r)+'\n')
# Reverse audit on the TRUE path (sim2d.reverse_audit: every position reached under a reverse command, or moving backward, must lie
# within 2.5 cm + the estimate-to-truth error of where the body went otherwise). Each tick's path runs from the previous state's
# truth to this one's, under the command decided on the previous state. Its episode length also counts ticks whose body moved
# backward without a reverse command (a clamp-made pivot in sim2d; here also the base_link drift of a turn in place), so the
# commanded reverse episodes (reverse commands not separated by 5 cm of forward travel) are reported on their own.
rows=[r for r in rows if r['truth_t'] is not None]
trace=[dict(t=r['t'],v=q['v'],w=q['w'],est=np.array(r['est']),truth=np.array(r['truth']),path=np.array([q['truth'][:2],r['truth'][:2]])) for q,r in zip(rows,rows[1:])]
audit=reverse_audit(trace) if trace else {}
episodes=[];run=0.;forward=0.
for r in trace:
    step=float(np.linalg.norm(r['path'][1]-r['path'][0]))
    if r['v']<0:run+=step;forward=0.
    else:
        forward+=step if r['v']>0 else 0.
        if forward>=.05 and run:episodes.append(round(run,4));run=0.
if run:episodes.append(round(run,4))
touch=state['touch'];tt=state['touch_truth']
away=float(math.dist(rows[-1]['truth'][:2],tt[:2])) if touch is not None else 0.
released=None if state['released'] is None else round(state['released']-touch,2)
result=dict(approach=approach,start=START,touched_sim_s=touch,touch_truth=tt,released_after_s=released,away_from_touch_m=round(away,3),contacts=len(b.contacts),
            final_mode=b.mode,final_phase=b.phase,final_message=b.message,reverse_audit=audit,commanded_reverse_episodes_m=episodes,
            reverse_refused=sum('Reverse refused' in e['message'] for e in b.events),
            passed=bool(touch is not None and released is not None and released<=15 and b.phase!='contact' and away>=.3 and audit.get('off_trail_ticks')==0
                        and max(episodes,default=0.)<=.16),
            events=[e['message'] for e in b.events][-12:])
(OUT/'result.json').write_text(json.dumps(result,indent=1,default=float));print(json.dumps(result,default=float),flush=True)
