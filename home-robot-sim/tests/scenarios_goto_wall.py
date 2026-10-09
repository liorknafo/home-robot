"""Closed-loop sim2d scenarios: Go-to targets close to walls (the final approach, lidar_brain APPROACH_*).
Usage: scenarios_goto_wall.py [name ...]   (default: all; each runs in its own fresh Python process, in parallel)

Each scenario maps the same room with the round robot (Mapping until it finishes, so the saved map is the
controller's own), then sends one Go-to target near a wall, then a Go-to back to the middle of the room from
wherever the first one ended (leaving the near-wall spot). Map frame = world frame (mapping starts at the world
origin, heading 0). The bottom wall (y = Y_WALL) and left wall (x = X_WALL) sit 1.5 cm inside a 5 cm grid cell
(the robot-side cell edge at -1.0): a 2 cm-gap target is 4 cells (.20) from the occupied cells. The top wall sits
3.5 cm inside its cell: there the same target is 3 cells (.15) from them. Both are accepted (cell-centre
clearance >= GOAL_CLEAR = R - 2 cm); 2 cells (.10) are not.
  wall_2cm / wall_5cm / wall_10cm / wall_20cm   target with the skin 2 / 5 / 10 / 20 cm from the bottom wall
                                    (10 and 20 cm are safe-mask targets: the normal route, then the final approach)
  wall_2cm_unaligned                the 2 cm target at the top wall (3 cells from its occupied cells: accepted)
  wall_overlap                      a target 10 cm from the bottom wall (body overlapping it): rejected,
                                    'closer to an obstacle than the robot's body allows'
  corner                            target with the skin 3 cm from both walls of the bottom-left corner
  low_camera                        target (3 cm gap) beside a 10 cm deep low box against the wall, LiDAR-blind;
                                    the camera 'detects' it (box outline within 1.2 m and +/-35 deg of the true
                                    heading is added as camera cells, as LidarBrain.observe does)
  low_bumper                        the same, no camera detections: only the bumper finds the box
  wall_2cm_s1/_s2, corner_s1/_s2, low_camera_s1/_s2   the same with other LiDAR noise seeds (sim2d Sim seed)
  corner_then_mapping               the corner target, then MAPPING is started from where it stopped (instead of
                                    the Go-to away): it must turn and leave (> .25 m) within 40 s with no reverse
                                    command, no backward motion, 0 rigid-body blocks and 0 bumper touches
Reported per Go-to: accepted / rejected, final message, true skin gap to the nearest wall or low object at the
end (and the minimum during the run), rigid-body blocks, bumper touches, reverse ticks, reverse audit
(sim2d.reverse_audit on the whole session), forward-only approach, sim time.
PASS (near-wall Go-to): accepted (rejected with the clear message for wall_overlap), ended Idle after a
confirmed 'Arrived' or 'Stopped N cm short', 0 rigid-body blocks, 0 bumper touches (low_bumper: the box is
touched at most twice and the Go-to ends 'Stopped ... short'), no reverse command during the approach, and the
reverse audit clean; wall_5cm / 10cm / 20cm also end within 3 cm of the target's skin gap. wall_10cm / 20cm are
safe-mask targets: routed exactly as before, then (since the follow phase stops within .10 m) the final approach.
PASS (Go-to away, open floor): 'Arrived' within 3 cm of its target, 0 rigid-body blocks, 0 bumper touches, and
no reverse command and no backward base_link motion until base_link is .25 m from where it started (it turns in
place and drives forward). Later in that route the normal rules apply (the local planner may back up a few mm along the
trail it just drove near the target: reverse_cmds counts them, the reverse audit checks them).
"""
import math,os,subprocess,sys,tempfile,numpy as np
from pathlib import Path
P=str(Path(__file__).resolve().parents[1])
sys.path.insert(0,P+'/controller');sys.path.insert(0,P+'/tests')
from autonomy.lidar_brain import LidarBrain
from sim2d import World,Sim,Bridge,box,poly_segments,seg_point_distance,reverse_audit,backward_tick,SKIN_R

X_WALL,Y_WALL,X_RIGHT,Y_TOP=-1.015,-1.015,2.2,1.335
WALLS=[((X_WALL,Y_WALL),(X_RIGHT,Y_WALL)),((X_RIGHT,Y_WALL),(X_RIGHT,Y_TOP)),((X_RIGHT,Y_TOP),(X_WALL,Y_TOP)),((X_WALL,Y_TOP),(X_WALL,Y_WALL)),
       ((1.2,.3),(1.6,.3)),((1.6,.3),(1.6,.5)),((1.6,.5),(1.2,.5)),((1.2,.5),(1.2,.3))]      # a counter island
LOW_BOX=box(.1,Y_WALL+.05,.2,.1)                                                          # 20 x 10 cm, against the wall
AWAY=(.4,.1)
SC={'wall_2cm':dict(goal=(.6,Y_WALL+SKIN_R+.02)),'wall_5cm':dict(goal=(.6,Y_WALL+SKIN_R+.05),gap=.05),'wall_10cm':dict(goal=(.6,Y_WALL+SKIN_R+.10),gap=.10),
    'wall_20cm':dict(goal=(.6,Y_WALL+SKIN_R+.20),gap=.20),
    'wall_2cm_unaligned':dict(goal=(.6,Y_TOP-SKIN_R-.02)),'wall_overlap':dict(goal=(.6,Y_WALL+.10),expect_reject=True),
    'corner':dict(goal=(X_WALL+SKIN_R+.03,Y_WALL+SKIN_R+.03)),
    'low_camera':dict(goal=(.3,Y_WALL+SKIN_R+.03),low=True,camera=True),'low_bumper':dict(goal=(.3,Y_WALL+SKIN_R+.03),low=True,bumper_ok=True),
    'corner_then_mapping':dict(goal=(X_WALL+SKIN_R+.03,Y_WALL+SKIN_R+.03),then='mapping')}
# Other LiDAR noise realizations (sim2d seed) of the closest targets.
SC.update({f'{n}_s{k}':dict(SC[n],seed=k) for n in ('wall_2cm','corner','low_camera') for k in (1,2)})


def true_gap(world,xy):
    """Skin gap (m) from the true base_link position to the nearest wall or low object."""
    segs=[np.asarray(world.walls,float).reshape(-1,2,2)]+[np.array(poly_segments(p),float) for p in list(world.low)+list(world.curbs)]
    return float(seg_point_distance(np.concatenate(segs),np.asarray(xy,float)).min()-SKIN_R)


def camera_hook(world,fov=math.radians(35),reach=1.2):
    """Stand-in for the floor-parallax camera: outline points of the low objects within reach and the field of
    view of the TRUE pose are added as camera cells (margin 0) while a mode is active, as LidarBrain.observe does."""
    outline=np.vstack([np.linspace(a,b,max(2,int(np.linalg.norm(b-a)/.01)+1)) for p in world.low for a,b in poly_segments(p)])
    def tick(br):
        b,p=br.b,br.sim.pose
        if b.mode=='idle':return
        rel=outline-p[:2];d=np.linalg.norm(rel,axis=1);az=np.arctan2(rel[:,1],rel[:,0])-p[2];az=(az+math.pi)%(2*math.pi)-math.pi
        seen=outline[(d<reach)&(np.abs(az)<fov)]
        if len(seen):
            before=int(b.grid.camera_obstacles.sum());b.grid.add_camera_obstacles(seen,margin=0)
            if int(b.grid.camera_obstacles.sum())>before:b.next_plan=0
    return tick


def go_to(br,goal,seconds=150.,on_tick=None,world=None):
    b,sim=br.b,br.sim;start=len(br.trace);t0=sim.t;c0,k0=sim.contacts,sim.bump_events
    try:b.begin_go_to(list(goal),sim.t)
    except ValueError as e:return dict(accepted=False,message=str(e))
    br.arm_pending=True;sim.power_on();gaps=[]
    def tick(x):
        gaps.append(true_gap(world,x.sim.pose[:2]))
        if on_tick:on_tick(x)
    br.run(seconds,on_tick=tick)
    seg=br.trace[start:];origin=seg[0]['truth'][:2]
    # 'Leaving': until base_link is .25 m from where this Go-to started.
    left=next((i for i,r in enumerate(seg) if np.linalg.norm(r['truth'][:2]-origin)>.25),len(seg))
    return dict(accepted=True,near=bool(b.near_goal),message=b.message,mode=b.mode,time_s=round(sim.t-t0,1),
                left_s=round(seg[left]['t']-t0,1) if left<len(seg) else None,
                reverse_cmds_leaving=sum(r['v']<0 for r in seg[:left]),backward_ticks_leaving=sum(backward_tick(r) for r in seg[:left]),
                end_gap_cm=round(100*true_gap(world,sim.pose[:2]),1),min_gap_cm=round(100*min(gaps),1) if gaps else None,
                end_to_goal_cm=round(100*float(np.linalg.norm(sim.pose[:2]-np.asarray(goal))),1),body_blocks=sim.contacts-c0,bumps=sim.bump_events-k0,
                reverse_cmds=sum(r['v']<0 for r in seg),backward_ticks=sum(backward_tick(r) for r in seg),
                approach_reverse=sum(r['v']<0 for r in seg if r['phase']=='approach'),approach_max_v=round(max([r['v'] for r in seg if r['phase']=='approach'] or [0.]),3),
                events=[e['message'] for e in b.events[-6:]])


def start_mapping_from_here(br,world,seconds=40.):
    """Start Mapping where the robot stands; report when it has left (> .25 m) and how (a new map frame, so the
    reverse audit covers this run alone)."""
    sim=br.sim;start=len(br.trace);t0=sim.t;c0,k0=sim.contacts,sim.bump_events;origin=sim.pose[:2].copy();left=[None];gaps=[]
    def tick(x):
        gaps.append(true_gap(world,x.sim.pose[:2]))
        if left[0] is None and np.linalg.norm(x.sim.pose[:2]-origin)>.25:left[0]=round(x.sim.t-t0,1)
    br.start_mapping();br.run(seconds,on_tick=tick);seg=br.trace[start:]
    return dict(trace_start=start,left_s=left[0],mode=br.b.mode,phase=br.b.phase,message=br.b.message,min_gap_cm=round(100*min(gaps),1),body_blocks=sim.contacts-c0,bumps=sim.bump_events-k0,
                reverse_cmds=sum(r['v']<0 for r in seg),backward_ticks=sum(backward_tick(r) for r in seg),reverse_audit=reverse_audit(seg))


def run(name):
    cfg=SC[name];tmp=tempfile.TemporaryDirectory()
    b=LidarBrain(tmp.name);sim=Sim(World(WALLS),seed=cfg.get('seed',0));br=Bridge(b,sim,rearm_hook=lambda brain:getattr(brain,'wants_arm',False))
    for _ in range(10):b.predict(sim.state());[sim.step(.01) for _ in range(5)]
    br.start_mapping();br.run(800)
    mapped=dict(complete=bool(b.complete),t=round(sim.t),contacts=sim.contacts,max_err=round(max(np.linalg.norm(r['est'][:2]-r['truth'][:2]) for r in br.trace),4))
    world=World(WALLS,low=[LOW_BOX] if cfg.get('low') else ())
    sim.w=world                                                      # the low box was put down after mapping
    near=go_to(br,cfg['goal'],on_tick=camera_hook(world) if cfg.get('camera') else None,world=world)
    if cfg.get('then')=='mapping':
        away=start_mapping_from_here(br,world);audit=reverse_audit(br.trace[:away.pop('trace_start')])
        ok_map=away['left_s'] is not None and away['left_s']<=40 and away['body_blocks']==0 and away['bumps']==0 and away['reverse_cmds']==0 and away['backward_ticks']==0 and away['reverse_audit']['ok']
        stopped_ok=near.get('message','').startswith(('Arrived.','Stopped '))
        ok=bool(mapped['complete'] and near['accepted'] and stopped_ok and near['body_blocks']==0 and near['bumps']==0 and near['approach_reverse']==0 and ok_map and audit['ok'])
        print(name,'PASS' if ok else 'FAIL',dict(goal=[round(x,4) for x in cfg['goal']],mapping=mapped,near=near,mapping_from_there=away,reverse_audit=audit),flush=True)
        return ok
    away=go_to(br,AWAY,world=world) if near['accepted'] or cfg.get('expect_reject') else None
    audit=reverse_audit(br.trace)
    stopped_ok=near.get('message','').startswith(('Arrived.','Stopped '))
    if cfg.get('expect_reject'):
        ok_near=not near['accepted'] and 'closer to an obstacle than' in near['message']
    else:
        ok_near=near['accepted'] and near['mode']=='idle' and stopped_ok and near['body_blocks']==0 and near['approach_reverse']==0 and near['approach_max_v']<=.05+1e-9
        ok_near&=(near['bumps']<=2 and near['message'].startswith('Stopped ')) if cfg.get('bumper_ok') else near['bumps']==0
        if 'gap' in cfg:ok_near&=abs(near['end_gap_cm']-100*cfg['gap'])<=100*.03
    # Leaving where the first Go-to ended: no reverse at all until .25 m away. The away target is open floor: it must
    # end 'Arrived' within 3 cm (the final approach).
    left_forward=not near.get('accepted') or (away is not None and away['left_s'] is not None and away['reverse_cmds_leaving']==0 and away['backward_ticks_leaving']==0)
    ok_away=away is not None and away['message'].startswith('Arrived.') and away['end_to_goal_cm']<=3 and away['body_blocks']==0 and away['bumps']==0 and left_forward
    ok=bool(mapped['complete'] and mapped['contacts']==0 and ok_near and ok_away and audit['ok'])
    print(name,'PASS' if ok else 'FAIL',dict(goal=[round(x,4) for x in cfg['goal']],mapping=mapped,near=near,away=away,reverse_audit=audit),flush=True)
    return ok


if __name__=='__main__':
    names=sys.argv[1:] or list(SC)
    if len(names)>1:
        from concurrent.futures import ThreadPoolExecutor
        env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1')
        call=lambda n:subprocess.run([sys.executable,'-B',__file__,n],capture_output=True,text=True,env=env)
        ok=True
        with ThreadPoolExecutor(int(os.environ.get('JOBS',6))) as pool:
            for n,r in zip(names,pool.map(call,names)):
                sys.stdout.write(r.stdout);sys.stderr.write(r.stderr[-3000:] if r.returncode not in (0,1) else '');sys.stdout.flush();ok&=r.returncode==0
        print(f"go-to near wall: {'all PASS' if ok else 'FAILURES'} ({len(names)} scenarios)",flush=True)
        sys.exit(0 if ok else 1)
    sys.exit(0 if all([run(n) for n in names]) else 1)
