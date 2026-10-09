"""Offline regression for controller/autonomy/floor_parallax.py over full robot recordings.

The detector sees ONLY what the controller saw: recorded camera images plus the meta/estimated_pose
rows the robot logged in Saved/Maps/<map>/camera-inputs.jsonl. Ground truth (frame_*.json
'ground_truth_evaluation_only' and the scene JSON boxes) is read ONLY by the scorer below.
Writes nothing into the project; prints per-obstacle detection range and false cells.
  range        = distance from the robot's TRUE base position to a confirmed cell when it was confirmed
  contact      = first frame where the true body outline is <1 cm from the box, scored with the chassis each
                 recording was MADE with (RUN_CHASSIS: every listed run is a box-robot run, the +-12x10 cm box
                 with bumper rays at 13 cm); --chassis round scores them against the round body (disc of
                 geometry.R about base_link) instead, e.g. to see how the new body would have fared
  touch point  = chassis point nearest the obstacle at contact; reported against the nearest cell confirmed earlier
  false cells  = confirmed cells >20 cm (10-20 cm listed too) from every scene box reaching 2-30 cm height
PASS needs: zero cells >20 cm from obstacles, and every obstacle the robot touched confirmed before contact at >=0.4 m.

  cd <scratch> && PYTHONDONTWRITEBYTECODE=1 <proj>/.venv/bin/python -B <proj>/tests/floor_parallax_regression.py [--runs tray,blocks,slippers|all] [--serial]
"""
import sys,json,math,time,argparse
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'controller'))
from autonomy.floor_parallax import LowObstacleCamera
from autonomy.geometry import R as BODY_R
RUNS={'tray':('run_20261003T174803_25338464523071','map_20261003T205718',('Shower tray',)),
      'blocks':('run_20261003T170837_25281687464214','map_20261003T200839',('Toy block','Slipper','Magazine stack')),
      'slippers':('run_20261003T172451_25305060337602','map_20261003T202506',('Slipper','Toy block','Magazine stack')),
      # --runs all adds: the live-benchmark tray abort (211601, angled approach) and three old-scene false-alarm checks.
      'tray2':('run_20261003T181553_25378555223140','map_20261003T211601',('Shower tray','Slipper','Toy block')),
      'fp1':('run_20261003T171512_25291170870447','map_20261003T201514',('Slipper','Toy block','Magazine stack')),
      'fp2':('run_20261003T155829_25180711877766','map_20261003T185832',('Slipper','Toy block','Magazine stack')),
      'fp3':('run_20261003T154428_25160528801582','map_20261003T184431',('Slipper','Toy block','Magazine stack'))}
LOW=('Shower tray','Toy block','Slipper','Magazine stack')
# Chassis each recording was driven with ('box' = old 24 x 20 cm robot; add 'round' for round-robot recordings).
RUN_CHASSIS={name:'box' for name in RUNS}

# ---------------- evaluation-only truth (never passed to the detector) ----------------
RES=.01;ORG=np.array([-3.4,-3.4]);N=int(9.6/RES)
def scene(map_name):
    snap=ROOT/f"reports/full-mapping-{map_name.split('_')[1]}/source-snapshot/Content/RobotData/house-scene.json"
    path=snap if snap.exists() else ROOT/'Content/RobotData/house-scene.json';out=[]
    for o in json.loads(path.read_text())['objects']:
        c=np.array(o['center'],float)/100;s=np.array(o['size'],float)/100
        out.append(dict(name=o['name'],group=o.get('group'),collision=o.get('collision',False),cx=c[0],cy=-c[1],sx=s[0],sy=s[1],yaw=-math.radians(o.get('yaw',0)),bot=c[2]-s[2]/2,top=c[2]+s[2]/2))
    return out,path
def corners(b,pad=0.):
    c,s=math.cos(b['yaw']),math.sin(b['yaw']);hx,hy=b['sx']/2+pad,b['sy']/2+pad
    return np.array([[hx,hy],[-hx,hy],[-hx,-hy],[hx,-hy]])@np.array([[c,s],[-s,c]])+[b['cx'],b['cy']]
def raster(bs,pad=0.):
    m=np.zeros((N,N),np.uint8)
    for b in bs:cv2.fillPoly(m,[((corners(b,pad)-ORG)/RES).astype(np.int32)],1)
    return m.astype(bool)
def lookup(m,xy):
    ij=((np.asarray(xy)[:,:2]-ORG)/RES).astype(int);ok=(ij>=0).all(1)&(ij<N).all(1);out=np.zeros(len(ij),bool);out[ok]=m[ij[ok,1],ij[ok,0]];return out
def box_distance(b,xy):
    c,s=math.cos(b['yaw']),math.sin(b['yaw']);d=np.asarray(xy)-[b['cx'],b['cy']];lx=d[:,0]*c+d[:,1]*s;ly=-d[:,0]*s+d[:,1]*c
    return np.hypot(np.maximum(0,np.abs(lx)-b['sx']/2),np.maximum(0,np.abs(ly)-b['sy']/2))
# Round body: perimeter samples (every 2 deg) of the disc of radius R about base_link.
CHASSIS=BODY_R*np.c_[np.cos(np.radians(np.arange(0,360,2))),np.sin(np.radians(np.arange(0,360,2)))]
OLD_BOX_CHASSIS=np.r_[[[x,y] for x in np.linspace(-.12,.12,25) for y in (-.10,.10)],[[x,y] for x in (-.12,.12,.13) for y in np.linspace(-.10,.10,21)]]
def chassis_gap(pos,yaw,b):
    c,s=math.cos(yaw),math.sin(yaw);return float(box_distance(b,CHASSIS@np.array([[c,s],[-s,c]])+pos[:2]).min())
def touch_point(pos,yaw,bs):
    c,s=math.cos(yaw),math.sin(yaw);p=CHASSIS@np.array([[c,s],[-s,c]])+pos[:2];return p[np.argmin(np.min([box_distance(b,p) for b in bs],0))]
def align(est,gt):
    ma,mb=est.mean(0),gt.mean(0);U,_,Vt=np.linalg.svd((est-ma).T@(gt-mb));R=Vt.T@U.T
    if np.linalg.det(R)<0:Vt[1]*=-1;R=Vt.T@U.T
    return R,mb-R@ma

def replay(rec,map_name,kw,frames=None,verbose=False):
    rows=[json.loads(l) for l in (ROOT/'Saved/Maps'/map_name/'camera-inputs.jsonl').open()]
    if frames:rows=[r for r in rows if frames[0]<=r['meta']['frame']<=frames[1]]
    cam=LowObstacleCamera(**kw);conf=[];ms=[];states={}
    for r in rows:
        meta,pose=r['meta'],np.array(r['estimated_pose'])
        if not r.get('geometry_ready',True):cam.reset_history();continue
        img=cv2.imread(str(ROOT/'Saved/Stereo'/rec/('left_%06d.png'%meta['frame'])))
        if img is None:continue
        t0=time.perf_counter();new=cam.process(img,meta,pose);ms.append((time.perf_counter()-t0)*1000)
        states[cam.status['state']]=states.get(cam.status['state'],0)+1
        for p in new:conf.append((meta['frame'],p,pose.copy()))
        if verbose and (len(new) or meta['frame']%50==0):print(meta['frame'],cam.status)
    return rows,conf,np.array(ms),states

def score(name,rec,map_name,focus,rows,conf,ms,states):
    objs,path=scene(map_name);F={}
    for r in rows:
        j=json.loads((ROOT/'Saved/Stereo'/rec/('frame_%06d.json'%r['meta']['frame'])).read_text());F[r['meta']['frame']]=j['ground_truth_evaluation_only']
    ks=sorted(F);R,t=align(np.array([next(r['estimated_pose'][:2] for r in rows if r['meta']['frame']==k) for k in ks]),np.array([F[k]['position_m'][:2] for k in ks]))
    solid=[b for b in objs if b['collision'] and b['group'] not in ('floor_finish','ceiling','floor') and b['top']>=.02 and b['bot']<=.30]
    flat=[b for b in objs if b['group'] in ('rug','threshold')]
    dist=cv2.distanceTransform((~raster(solid)).astype(np.uint8),cv2.DIST_L2,5)*RES;flatm=raster(flat,.03)
    C=np.array([R@c[1]+t for c in conf]).reshape(-1,2);ij=np.clip(((C-ORG)/RES).astype(int),0,N-1);d=dist[ij[:,1],ij[:,0]] if len(C) else np.empty(0)
    print(f'\n== {name}: {rec} + {map_name} (truth: {path.relative_to(ROOT)})')
    print(f'frames {len(ms)}  ms/frame mean {ms.mean():.1f} median {np.median(ms):.1f} p95 {np.percentile(ms,95):.1f} max {ms.max():.1f}  (gray conversion+detector+voter)')
    print('states',dict(sorted(states.items(),key=lambda x:-x[1])))
    print(f'confirmed cells {len(C)}; >20 cm from any real obstacle {int((d>.2).sum())}; 10-20 cm {int(((d>.1)&(d<=.2)).sum())}; on rug/mat/threshold footprint and >10 cm from obstacles {int((lookup(flatm,C)&(d>.1)).sum()) if len(C) else 0}')
    for i in np.flatnonzero(d>.1):
        near=min(solid,key=lambda b:float(box_distance(b,C[i:i+1])[0]))
        print(f'   false? cell {np.round(C[i],2).tolist()} frame {conf[i][0]} dist {d[i]:.2f} m (nearest {near["name"]})')
    ok=bool(len(C)==0 or (d<=.2).all())
    for oname in focus:
        bs=[b for b in objs if b['name']==oname];m=raster(bs,.06);hit=lookup(m,C) if len(C) else np.zeros(0,bool)
        contact=next((k for k in ks if min(chassis_gap(np.array(F[k]['position_m']),F[k]['yaw_rad'],b) for b in bs)<.01),None)
        closest=min((min(chassis_gap(np.array(F[k]['position_m']),F[k]['yaw_rad'],b) for b in bs),k) for k in ks)
        if not hit.any():print(f'  {oname}: NOT confirmed; closest chassis gap {closest[0]*100:.1f} cm at frame {closest[1]}; contact frame {contact}');ok&=contact is None;continue
        rng=[float(np.linalg.norm(C[i]-np.array(F[conf[i][0]]['position_m'][:2]))) for i in np.flatnonzero(hit)]
        fr=[conf[i][0] for i in np.flatnonzero(hit)];first=int(np.argmin(fr))
        stamp=lambda k:next(r['meta']['sim_time_s'] for r in rows if r['meta']['frame']==k)
        before=f'{stamp(contact)-stamp(fr[first]):.1f} s before contact (frame {contact})' if contact else f'no contact; closest chassis gap {closest[0]*100:.1f} cm at frame {closest[1]}'
        cover=''
        if contact:ok&=any(f<contact and r>=.4 for f,r in zip(fr,rng))
        if contact:
            tp=touch_point(np.array(F[contact]['position_m']),F[contact]['yaw_rad'],bs);pre=[C[i] for i in np.flatnonzero(hit) if conf[i][0]<contact]
            cover=f'; touch point {np.round(tp,2).tolist()} -> nearest earlier cell {min(np.linalg.norm(c-tp) for c in pre)*100:.0f} cm' if pre else '; touch point not covered'
        print(f'  {oname}: {int(hit.sum())} cells; first at frame {fr[first]} range {rng[first]:.2f} m, {before}; ranges {min(rng):.2f}-{max(rng):.2f} m; cells before contact {sum(1 for f in fr if contact is None or f<contact)}{cover}')
    print('  result','PASS' if ok else 'FAIL');return dict(cells=len(C),far=int((d>.2).sum()),near=int(((d>.1)&(d<=.2)).sum()),ms=round(float(ms.mean()),1),passed=ok)

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--runs',default='tray,blocks,slippers');a.add_argument('--serial',action='store_true',help='no backward-flow worker thread')
    a.add_argument('--frames',default=None,help='first:last frame');a.add_argument('-v',action='store_true')
    a.add_argument('--chassis',choices=('recorded','round','box'),default='recorded',help='chassis for contact scoring (default: the one each run was recorded with)')
    a.add_argument('--old-box-chassis',action='store_true',help='same as --chassis box');o=a.parse_args()
    if o.old_box_chassis:o.chassis='box'
    ROUND_CHASSIS=CHASSIS
    kw=dict(parallel=False) if o.serial else {};fr=tuple(map(int,o.frames.split(':'))) if o.frames else None;total={}
    for name in (RUNS if o.runs=='all' else o.runs.split(',')):
        rec,map_name,focus=RUNS[name];rows,conf,ms,states=replay(rec,map_name,kw,fr,o.v)
        CHASSIS=OLD_BOX_CHASSIS if (RUN_CHASSIS[name] if o.chassis=='recorded' else o.chassis)=='box' else ROUND_CHASSIS
        total[name]=score(name,rec,map_name,focus,rows,conf,ms,states)
    print('\nsummary',json.dumps(total));sys.exit(0 if all(v['passed'] for v in total.values()) else 1)
