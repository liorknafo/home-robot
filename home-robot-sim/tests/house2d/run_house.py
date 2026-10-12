"""Closed-loop whole-house mapping run in the 2D harness. Usage: run_house.py LABEL [seconds] [seed] [cam] [ovcam]
Outputs (LABEL.log, LABEL.json, map-LABEL/) go to $HOUSE2D_OUT (default <sim>/reports/house2d).
CTRL / TESTS (env) point at another controller / tests tree, e.g. a robot-parameters variant made by make_variant.py.
Evaluation only (bay B13 additions, the brain is not touched): time per target kind (frontier vs coverage, read from the
brain's goal_kind), the unvisited floor every 25 s, and at the end which mapping_complete() condition fails."""
import sys,time,json,tempfile,numpy as np,os
from pathlib import Path
from house_world import load_world,HouseSim,P,low_samples,camera_view,overhang_samples,overhang_view
from sim2d import Bridge,reverse_audit
import sim2d
sys.path.insert(0,os.environ.get('TESTS',P+'/tests'))
import importlib.util
spec=importlib.util.spec_from_file_location('score',os.environ.get('TESTS',P+'/tests')+'/score_full_mapping.py');score=importlib.util.module_from_spec(spec);spec.loader.exec_module(score)
from autonomy.lidar_brain import LidarBrain,UNVISITED_DONE
import autonomy.geometry as geometry
label=sys.argv[1];limit=float(sys.argv[2]) if len(sys.argv)>2 else 1600;seed=int(sys.argv[3]) if len(sys.argv)>3 else 0;cam='cam' in sys.argv[4:];ovcam='ovcam' in sys.argv[4:]   # ovcam: the camera's band pass on overhangs (task M)
OUT=Path(os.environ.get('HOUSE2D_OUT',P+'/reports/house2d'));OUT.mkdir(parents=True,exist_ok=True)
tmp=tempfile.TemporaryDirectory();b=LidarBrain(tmp.name);world=load_world();sim=HouseSim(world,seed=seed);samples=low_samples(world);ov_samples=overhang_samples(world)
br=Bridge(b,sim,rearm_hook=lambda brain:getattr(brain,'wants_arm',False))
for _ in range(10):b.predict(sim.state());[sim.step(.01) for _ in range(5)]
log=open(OUT/f'{label}.log','w');t0=time.time();seen=set();last={'goal':None,'ev':0,'print':0,'phase':None}
setup=dict(controller=str(Path(geometry.__file__).resolve().parents[1]),sim2d=str(Path(sim2d.__file__).resolve()),lidar_mount_m=list(geometry.LIDAR_MOUNT),
           sim_lidar_mount_m=list(sim2d.LIDAR_MOUNT),sim_turret_xy_m=sim2d.TURRET_XY.tolist(),turret_radius_m=sim2d.TURRET_R,slices=os.path.basename(__import__('house_world').SLICES),
           sensor_errors=sim.sensor_model.describe() if hasattr(sim.sensor_model,'describe') else bool(sim.sensor_model.enabled))
log.write('SETUP '+json.dumps(setup)+'\n')
# Stage bookkeeping (evaluation only): each tick is charged to the kind of the current target, from the target choice
# until the next choice ('none' before the first target and after a coverage target is covered on the way).
stage={'frontier':0.,'coverage':0.,'none':0.};kinds=[];timeline=[];first_cov=[None];lt=[sim.t]
# 'Target area covered on the way' (task H): the robot's distance to the target each time; beyond VISIT_RADIUS (1.2 m) the target
# left the unvisited mask without the camera passing it (the reachability flicker task H removed).
covered_on_way=[]
def tick(br):
    b=br.b
    if cam and b.mode!='idle' and sim.t-last.get('cam',0)>=.5:
        last['cam']=sim.t;q=camera_view(sim,samples)
        if len(q):
            e=b.localizer.pose;c,s_=np.cos(e[2]),np.sin(e[2]);b.grid.add_camera_obstacles(q@np.array([[c,s_],[-s_,c]])+e[:2],margin=0)
        if ovcam:
            q=overhang_view(sim,*ov_samples,moving=b.requested_motion[0]>.02)
            if len(q):
                e=b.localizer.pose;c,s_=np.cos(e[2]),np.sin(e[2]);w=q@np.array([[c,s_],[-s_,c]])+e[:2];b.grid.add_camera_obstacles(w,margin=0);b.next_plan=0
                b.grid.add_overhang_shadow(w,e[:2]+geometry.CAMERA_MOUNT[0]*np.array([c,s_]))   # as LidarBrain.observe does for band-pass edges
    dt=sim.t-lt[0];lt[0]=sim.t
    if b.mode=='mapping':
        k=getattr(b,'goal_kind',None) or 'none';stage[k]=stage.get(k,0.)+dt
        if k in ('frontier','coverage') and (not kinds or kinds[-1][1]!=k):kinds.append((round(sim.t,1),k))
        if k=='coverage' and first_cov[0] is None:first_cov[0]=sim.t
        if sim.t-last.get('uv',-1e9)>=25:
            last['uv']=sim.t;timeline.append((round(sim.t),round(b.unvisited_area(),2),round(b.grid.summary()['free_area_m2'],1),k))
    g=None if b.goal is None else tuple(np.round(b.goal,2))
    if g!=last['goal'] or b.phase!=last['phase'] and b.phase in ('plan','contact','retreat','escape_margin','recovery_advance','idle'):
        log.write(f"{sim.t:7.1f} {b.phase:16s} goal={g} kind={getattr(b,'goal_kind',None)} pose={np.round(sim.pose[:2],2).tolist()} r={b.grid.radius}\n");last['goal']=g
    last['phase']=b.phase
    if sim.contacts>last.get('blocks',0):
        if not last.get('blocking'):
            r=br.trace[-1];log.write(f"{sim.t:7.1f}   BODY-BLOCK phase={b.phase} v={r['v']:.3f} w={r['w']:.2f} pose={np.round(sim.pose,2).tolist()} bump={sim.bump}\n")
        last['blocking']=True
    else:last['blocking']=False
    last['blocks']=sim.contacts
    for e in b.events:
        k=(e['time'],e['message'])
        if k in seen:continue
        seen.add(k);log.write(f"{sim.t:7.1f}   EVENT {e['message']}\n")
        if e['message'].startswith('Target area covered on the way') and b.goal is not None:covered_on_way.append(round(float(np.linalg.norm(b.localizer.pose[:2]-b.goal)),2))
    if sim.t-last['print']>100:
        last['print']=sim.t;log.write(f"{sim.t:7.1f} --- free {b.grid.summary()['free_area_m2']:.1f} wall {time.time()-t0:.0f}s\n");log.flush()
if os.environ.get('LOGFRONT'):
    from autonomy.lidar import LidarGrid
    import cv2
    orig=LidarGrid.frontiers
    def logged(self,pose,blacklist=()):
        out=orig(self,pose,blacklist)
        if out and b.phase=='plan':
            sc,xy,route=max(out,key=lambda c:c[0]);L=float(np.linalg.norm(np.diff(np.asarray(route),axis=0),axis=1).sum())
            free,occ,unk,safe,cl=self.masks();ign=getattr(self,'unobservable',None)
            if ign is not None:unk=unk&~ign
            y,x=self.cell(xy);win=unk[max(0,y-15):y+16,max(0,x-15):x+16]
            n,lab,st,_=cv2.connectedComponentsWithStats(unk.astype(np.uint8),8)
            ids=set(np.unique(lab[max(0,y-15):y+16,max(0,x-15):x+16]))-{0}
            sizes=sorted([int(st[i,4]) for i in ids if st[i,4]>=12],reverse=True)[:4]
            log.write(f"{sim.t:7.1f}   FRONTIER choice xy={np.round(xy,2).tolist()} value={sc*(1+L):.0f} route={L:.1f}m n={len(out)} comps={sizes} r={self.radius}\n")
        return out
    LidarGrid.frontiers=logged
t_start=sim.t;br.start_mapping();br.run(limit,on_tick=tick)
b.grid.save(OUT/f'map-{label}')
import pickle;pickle.dump(dict(visits=[np.asarray(v) for v in b.visits],blacklist=b.blacklist,trouble=b.trouble,truth=np.array([r[1:4] for r in sim.truth_log[::20]])),open(OUT/f'map-{label}/extra.pkl','wb'))
xx,yy,occ,reach=score.reference((0,0));g=b.grid
mapped=np.zeros_like(reach)
for i in range(xx.shape[0]):
    for j in range(xx.shape[1]):
        c=g.cell((xx[i,j],yy[i,j]))
        if g.valid(c) and g.logodds[c]<-.8:mapped[i,j]=True
truth=np.array([r[1:3] for r in sim.truth_log[::50]])
rooms={}
for name,(x0,x1,y0,y1) in score.ROOMS:
    reg=reach&(xx>=x0)&(xx<x1)&(yy>=y0)&(yy<y1);ent=bool(((truth[:,0]>x0+.15)&(truth[:,0]<x1-.15)&(truth[:,1]>y0+.15)&(truth[:,1]<y1-.15)).any())
    rooms[name]=(ent,round(float((mapped&reg).sum()/max(1,reg.sum())),3))
err=max(np.linalg.norm(r['est'][:2]-r['truth'][:2]) for r in br.trace)
# End state (evaluation only): the mapping_complete() conditions, and where the unvisited floor lies (the 2D map frame is
# the world frame: the robot starts at the origin, yaw 0).
import cv2
pose=b.localizer.pose;safe=g.masks()[3];reachable=g.reachable(pose[:2],safe);uv=b.unvisited_mask()
blockers=dict(frontiers_left=len(g.frontiers(pose,b.blacklist+b.scanned_positions+b.trouble)),unvisited_m2=round(float(uv.sum()*g.res**2),3),unvisited_done_m2=UNVISITED_DONE,
              disconnected_safe_m2=round(float((safe&~reachable).sum()*g.res**2),3),
              unreached_enterable_m2=round(float(g.unreached_enterable(pose[:2],safe).sum()*g.res**2),3) if hasattr(g,'unreached_enterable') else None,uncertainty=round(float(b.localizer.uncertainty),3),stuck=bool(b.stuck()),planning_radius_m=g.radius)
def room_of(xy):
    for name,(x0,x1,y0,y1) in score.ROOMS:
        if x0<=xy[0]<x1 and y0<=xy[1]<y1:return name
    return 'outside rooms'
uv_rooms={}
for cell in np.argwhere(uv):
    n=room_of(g.world(cell));uv_rooms[n]=round(uv_rooms.get(n,0.)+g.res**2,3)
n,lab,st,cen=cv2.connectedComponentsWithStats(uv.astype(np.uint8),connectivity=8)
clusters=sorted([dict(area_m2=round(float(st[i,4]*g.res**2),3),centre_xy=np.round(g.world((int(round(cen[i][1])),int(round(cen[i][0])))),2).tolist(),room=room_of(g.world((int(round(cen[i][1])),int(round(cen[i][0])))))) for i in range(1,n)],key=lambda c:-c['area_m2'])
elapsed=sim.t-t_start
# Contact phases (evaluation only, task G): the total and the longest time the brain spent in its contact phase (a robot pressed
# against something it cannot leave used to stay there until the time budget), and the reverse audit on the TRUE path.
spans=[];since=None
for r in br.trace:
    if r['phase']=='contact':since=r['t'] if since is None else since;last_contact=r['t']
    elif since is not None:spans.append(last_contact-since);since=None
if since is not None:spans.append(br.trace[-1]['t']-since)
audit=reverse_audit(br.trace)
res=dict(label=label,seed=seed,cam=cam,lidar_mount_m=list(geometry.LIDAR_MOUNT),turret_xy_m=sim2d.TURRET_XY.tolist(),sim_t=round(sim.t),mapping_s=round(elapsed,1),wall_s=round(time.time()-t0),complete=b.complete,message=b.message,rooms=rooms,passed=all(e and f>=.9 for e,f in rooms.values()) and bool(b.complete),
         contacts=len(b.contacts),skips=len(b.blacklist),trouble=len(b.trouble),body_blocks=sim.contacts,bumps=sim.bump_events,max_err=round(float(err),3),unvisited=round(b.unvisited_area(),3),free=round(b.grid.summary()['free_area_m2'],1),
         stage_s={k:round(v,1) for k,v in stage.items()},first_coverage_s=None if first_cov[0] is None else round(first_cov[0]-t_start,1),kind_switches=len(kinds)-1 if kinds else 0,
         returns_to_frontier=sum(1 for i in range(1,len(kinds)) if kinds[i][1]=='frontier'),blockers=blockers,unvisited_by_room=uv_rooms,unvisited_clusters=clusters[:8],
         unvisited_timeline=[(round(t-t_start),u,f,k) for t,u,f,k in timeline],kinds=kinds,
         covered_on_the_way_m=covered_on_way,contact_phases=len(spans),contact_phase_s=round(sum(spans),1),longest_contact_phase_s=round(max(spans,default=0.),1),reverse_audit=audit)
log.write(json.dumps(res)+'\n');log.close();(OUT/f'{label}.json').write_text(json.dumps(res,indent=1));print(json.dumps({k:v for k,v in res.items() if k not in ('unvisited_timeline','kinds','unvisited_clusters')}))
