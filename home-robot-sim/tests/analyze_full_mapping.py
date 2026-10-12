"""Offline, evaluation-only analysis of one Unreal full-mapping trial (bay B13). Never imported or read by the controller.
Usage: analyze_full_mapping.py REPORT_DIR [--json OUT]
- Rooms: score_full_mapping's computation, re-run from REPORT_DIR's own copies (occupancy.npz, trial.json, trace.jsonl,
  frozen house-scene), so an old trial is scored exactly like a new one.
- Stages: seconds on frontier vs coverage targets from status.goal_kind (logged by autonomy_service from bay B13 on).
  Trials without it get proxies only: the first coverage-only event ('Target area covered on the way') and the time
  the robot's observed area reached 98 % / 99 % of its final value.
- End state: the robot's own saved map (Saved/Maps/<saved_map>) and exploration.json are loaded into a LidarBrain to
  re-evaluate the mapping_complete() conditions and locate the unvisited floor (map -> world with the trial's
  evaluation-only origin), with the current controller's rule (mapping_complete_recomputed; task H: the pockets the body could
  enter, recomputed_unreached_enterable_m2 and each disconnected cluster's 'enterable'). unvisited(t): the same mask with the final map and the visits rebuilt from the trace poses up
  to t (an approximation: the map at t was smaller)."""
from pathlib import Path
import json,math,sys,tempfile
import numpy as np,cv2
R=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(R/'controller'));sys.path.insert(0,str(R/'tests'))
import score_full_mapping as S
from autonomy.lidar_brain import LidarBrain,UNVISITED_DONE
def analyze(D):
    D=Path(D);trial=json.loads((D/'trial.json').read_text());final=json.loads((D/'final-status.json').read_text())
    rows=[json.loads(l) for l in (D/'trace.jsonl').read_text().splitlines()];truth_rows=[r for r in rows if 'ground_truth_evaluation_only' in r]
    mm=json.loads((D/'map-manifest.json').read_text()) if (D/'map-manifest.json').exists() else None
    prev=json.loads((D/'results.json').read_text()) if (D/'results.json').exists() else {}
    # --- rooms, exactly as score_full_mapping.score() ---
    md=np.load(D/'occupancy.npz');log=md['logodds'];seen=md['seen'];res=float(md['res']);mo=md['origin']
    origin=trial['map_origin_ground_truth_evaluation_only'];xy0=origin['position_m'][:2];ang=origin['yaw_rad'];c,s=math.cos(ang),math.sin(ang)
    fs=D/'source-snapshot/Content/RobotData/house-scene.json';xx,yy,occupied,reachable=S.reference(xy0,fs if fs.exists() else None)
    dx=xx-xy0[0];dy=yy-xy0[1];mx=c*dx+s*dy;my=-s*dx+c*dy;ix=np.floor((mx-mo[0])/res).astype(int);iy=np.floor((my-mo[1])/res).astype(int)
    inside=(ix>=0)&(iy>=0)&(ix<log.shape[1])&(iy<log.shape[0]);ix=np.clip(ix,0,log.shape[1]-1);iy=np.clip(iy,0,log.shape[0]-1);mapped=inside&(log[iy,ix]<-.8)
    actual=np.array([r['ground_truth_evaluation_only']['position_m'][:2] for r in truth_rows]);rooms={}
    for name,(x0,x1,y0,y1) in S.ROOMS:
        region=reachable&(xx>=x0)&(xx<x1)&(yy>=y0)&(yy<y1);n=int(region.sum())
        ent=bool(((actual[:,0]>x0+.15)&(actual[:,0]<x1-.15)&(actual[:,1]>y0+.15)&(actual[:,1]<y1-.15)).any())
        rooms[name]=(ent,round(float((mapped&region).sum()/max(1,n)),3))
    t0=rows[0]['status']['sim_time_s'];duration=final['sim_time_s']-truth_rows[0]['status']['sim_time_s']
    # --- stages ---
    st=[r['status'] for r in rows];times=np.array([x['sim_time_s'] for x in st])-t0
    has_kind=any('goal_kind' in x for x in st);stage=None;first_cov=None;kinds=[]
    if has_kind:
        stage={'frontier':0.,'coverage':0.,'none':0.}
        for i in range(len(st)-1):
            if st[i]['mode']!='mapping':continue
            k=st[i].get('goal_kind') or 'none';stage[k]+=times[i+1]-times[i]
            if k in ('frontier','coverage') and (not kinds or kinds[-1][1]!=k):kinds.append((round(float(times[i]),1),k))
            if k=='coverage' and first_cov is None:first_cov=round(float(times[i]),1)
        stage={k:round(v,1) for k,v in stage.items()}
    events={}
    for x,t in zip(st,times):
        for e in x.get('events',[]):
            events.setdefault((e['time'],e['message']),float(t))
    ev=sorted(((t,m) for (_,m),t in events.items()),key=lambda a:a[0])
    covered_on_way=[t for t,m in ev if m.startswith('Target area covered on the way')]
    # Distance from the robot to the coverage target at each 'covered on the way' (> VISIT_RADIUS 1.2 m straight-line
    # means the target left the unvisited mask without the robot driving within camera range: reachability / safe flicker).
    cow_dist=[];seen_ev=set();prev_goal=None
    for x,t in zip(st,times):
        for e in x.get('events',[]):
            k=(e['time'],e['message'])
            if k in seen_ev:continue
            seen_ev.add(k)
            if e['message'].startswith('Target area covered on the way') and prev_goal is not None:cow_dist.append(round(float(np.linalg.norm(np.array(x['pose_m'][:2])-prev_goal)),2))
        if x.get('goal_m') is not None:prev_goal=np.array(x['goal_m'][:2])
    obs=np.array([x['map']['observed_area_m2'] for x in st]);free=np.array([x['map']['free_area_m2'] for x in st])
    def reach_frac(a,f):
        idx=np.argmax(a>=f*a[-1]);return round(float(times[idx]),1)
    proxies=dict(first_covered_on_the_way_s=round(covered_on_way[0],1) if covered_on_way else None,covered_on_the_way_events=len(covered_on_way),covered_on_the_way_distance_m=cow_dist,covered_on_the_way_beyond_1p2m=sum(d>1.2 for d in cow_dist),
                 observed_98pct_s=reach_frac(obs,.98),observed_99pct_s=reach_frac(obs,.99),free_98pct_s=reach_frac(free,.98),free_99pct_s=reach_frac(free,.99),final_observed_m2=round(float(obs[-1]),2),final_free_m2=round(float(free[-1]),2))
    counts=dict(contacts=final.get('contacts'),skipped_targets=final.get('skipped_targets'),
                jams=sum('jam' in m.lower() or 'caught' in m.lower() for _,m in ev),narrow_gap_plans=sum('narrow' in m.lower() for _,m in ev),
                route_closed=sum(m.startswith('The route to this target closed') for _,m in ev),no_progress=sum(m.startswith('No progress') for _,m in ev))
    # --- end state from the robot's own saved map ---
    end=None;timeline=[]
    folder=R/'Saved/Maps'/prev['saved_map'] if prev.get('saved_map') else None
    if folder and folder.exists():
        b=LidarBrain(tempfile.mkdtemp());b.grid.load(folder);ex=json.loads((folder/'exploration.json').read_text());man=json.loads((folder/'manifest.json').read_text())
        b.blacklist=[np.array(p) for p in ex['blocked_goals']];b.trouble=[np.array(p) for p in ex['trouble_spots']];b.scanned_positions=[np.array(p) for p in ex['viewpoints']]
        b.visits=[np.array(p) for p in ex['visits']];b.grid.radius=man.get('planning_radius_m',b.grid.radius);b.localizer.pose=np.array(final['pose_m'],float)
        g=b.grid;pose=b.localizer.pose;safe=g.masks()[3];rch=g.reachable(pose[:2],safe);uv=b.unvisited_mask()
        def to_world(xy):return [xy0[0]+c*xy[0]-s*xy[1],xy0[1]+s*xy[0]+c*xy[1]]
        def room_of(w):
            for name,(x0,x1,y0,y1) in S.ROOMS:
                if x0<=w[0]<x1 and y0<=w[1]<y1:return name
            return 'outside rooms'
        byroom={}
        for cell in np.argwhere(uv):
            n=room_of(to_world(g.world(cell)));byroom[n]=round(byroom.get(n,0.)+res**2,3)
        n,lab,stt,cen=cv2.connectedComponentsWithStats(uv.astype(np.uint8),connectivity=8)
        clusters=sorted([dict(area_m2=round(float(stt[i,4]*res**2),3),centre_world_xy=np.round(to_world(g.world((int(round(cen[i][1])),int(round(cen[i][0]))))),2).tolist(),
                              room=room_of(to_world(g.world((int(round(cen[i][1])),int(round(cen[i][0]))))))) for i in range(1,n)],key=lambda q:-q['area_m2'])[:8]
        dsafe=(safe&~rch).astype(np.uint8);n2,lab2,st2,cen2=cv2.connectedComponentsWithStats(dsafe,connectivity=8)
        # Task H: which of those pockets the round body could enter (grid.unreached_enterable; None for a controller without it).
        enter=g.unreached_enterable(pose[:2],safe) if hasattr(g,'unreached_enterable') else None
        dclusters=sorted([dict(area_m2=round(float(st2[i,4]*res**2),3),centre_world_xy=np.round(to_world(g.world((int(round(cen2[i][1])),int(round(cen2[i][0]))))),2).tolist(),
                               room=room_of(to_world(g.world((int(round(cen2[i][1])),int(round(cen2[i][0])))))),enterable=None if enter is None else bool(enter[lab2==i].any())) for i in range(1,n2)],key=lambda q:-q['area_m2'])[:8]
        end=dict(saved_map=folder.name,disconnected_safe_clusters=dclusters,manifest_complete=man['complete'],manifest_reason=man['reason'],manifest_unvisited_m2=round(man['unvisited_navigable_area_m2'],3),
                 manifest_disconnected_safe_m2=round(man['disconnected_safe_area_m2'],3),recomputed_unvisited_m2=round(float(uv.sum()*res**2),3),unvisited_done_m2=UNVISITED_DONE,
                 frontiers_left=len(g.frontiers(pose,b.blacklist+b.scanned_positions+b.trouble)),recomputed_disconnected_safe_m2=round(float((safe&~rch).sum()*res**2),3),
                 recomputed_unreached_enterable_m2=None if enter is None else round(float(enter.sum()*res**2),3),mapping_complete_recomputed=bool(b.mapping_complete()),
                 final_pose_uncertainty=final.get('pose_uncertainty_indicator_m'),planning_radius_m=g.radius,skipped=len(b.blacklist),trouble=len(b.trouble),viewpoints=len(b.scanned_positions),
                 unvisited_by_room=byroom,unvisited_clusters=clusters)
        # unvisited(t) with the final map and visits rebuilt from the trace poses (15 cm rule, as LidarBrain.observe_scan)
        final_visits=b.visits;vis=[];k=0
        for T in list(np.arange(0,times[-1],50))+[times[-1]]:
            while k<len(st) and times[k]<=T:
                p=np.array(st[k]['pose_m'][:2])
                if st[k]['mode']=='mapping' and (not vis or np.linalg.norm(p-vis[-1])>.15):vis.append(p)
                k+=1
            b.visits=vis;timeline.append((round(float(T)),round(float(b.unvisited_area()),2)))
        b.visits=final_visits
    return dict(dir=D.name,lidar=_lidar_from_snapshot(D),complete=bool(mm['complete']) if mm else None,stop_message=final['message'],
                duration_sim_s=round(duration,1),rooms=rooms,results_json_rooms={r['room']:round(r['mapped_free_fraction'],3) for r in prev.get('rooms',[])},
                distance_m=round(float(np.linalg.norm(np.diff(actual,axis=0),axis=1).sum()),1),stage_s=stage,first_coverage_s=first_cov,
                kind_switches=len(kinds)-1 if kinds else None,returns_to_frontier=sum(1 for i in range(1,len(kinds)) if kinds[i][1]=='frontier') if kinds else None,kinds=kinds,
                proxies=proxies,counts=counts,end=end,unvisited_timeline_final_map=timeline,source_unchanged=trial.get('source_unchanged'),sensor_errors=final.get('sim_sensor_errors'))
def _lidar_from_snapshot(D):
    p=D/'source-snapshot/Content/RobotData/robot-parameters.json'
    if not p.exists():return None
    q=json.loads(p.read_text());return dict(lidar_mount_m=q.get('lidar_mount_m'),lidar_turret_xy_m=q.get('lidar_turret_xy_m'))
if __name__=='__main__':
    out=analyze(sys.argv[1]);text=json.dumps(out,indent=1)
    if '--json' in sys.argv:Path(sys.argv[sys.argv.index('--json')+1]).write_text(text)
    print(text)
