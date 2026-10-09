"""Read-only ground-truth evaluator. Never imported or read by the controller.
Room geometry, ground truth and scores are used here only after the trial.
"""
from pathlib import Path
import json,math,shutil,hashlib
import numpy as np,cv2
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[1]
ROOMS=[('Living room',[-3,1,-1,3]),('Kitchen / dining',[1,5,-.7,3]),('Bedroom',[-3,1,-3,-1]),('Study',[1,3.2,-3,-.7]),('Bathroom',[3.2,5,-3,-.7])]
def reference(origin,scene_path=None):
    scene=json.loads((scene_path or R/'Content/RobotData/house-scene.json').read_text());res=.05;x=np.arange(-3,5,res)+res/2;y=np.arange(-3,3,res)+res/2;xx,yy=np.meshgrid(x,y);occupied=np.zeros_like(xx,bool)
    for obj in scene['objects']:
        if not obj['collision'] or obj['group'] in ['floor','ceiling','rug','threshold']:continue
        c=np.array(obj['center'])/100;s=np.array(obj['size'])/100
        if 'mesh_bounds_local_cm' in obj:
            lower,upper=np.asarray(obj['mesh_bounds_local_cm'])/100;offset=(lower+upper)/2;angle=math.radians(obj['yaw']);c+=np.array([offset[0]*math.cos(angle)-offset[1]*math.sin(angle),offset[0]*math.sin(angle)+offset[1]*math.cos(angle),offset[2]]);s=upper-lower
        if c[2]-s[2]/2>=.27 or c[2]+s[2]/2<=.035:continue
        angle=math.radians(obj['yaw']);dx=xx-c[0];dy=-yy-c[1];u=dx*math.cos(angle)+dy*math.sin(angle);v=-dx*math.sin(angle)+dy*math.cos(angle)
        occupied|=(np.abs(u)<=s[0]/2)&(np.abs(v)<=s[1]/2)
    # Border padding treats the house exterior as an obstacle.
    free=(~occupied).astype(np.uint8);padded=np.pad(free,1);clearance=cv2.distanceTransform(padded,cv2.DIST_L2,5)[1:-1,1:-1]*res;safe=(clearance>=.19)
    _,labels=cv2.connectedComponents(safe.astype(np.uint8),connectivity=8);col=int((origin[0]+3)/res);row=int((origin[1]+3)/res)
    if not(0<=row<len(y) and 0<=col<len(x)) or labels[row,col]==0:raise ValueError('Evaluation origin is not in reference navigable space')
    reachable=(labels==labels[row,col]);return xx,yy,occupied,reachable

def score():
    D=R/'reports'/json.loads((R/'reports/latest-full-mapping-test.json').read_text())['directory'];trial=json.loads((D/'trial.json').read_text())
    if not (D/'final-status.json').exists():raise ValueError('Trial is still running; final scoring waits for robot Idle')
    final=json.loads((D/'final-status.json').read_text());rows=[json.loads(l) for l in (D/'trace.jsonl').read_text().splitlines()];rows=[r for r in rows if 'ground_truth_evaluation_only' in r]
    pointer=json.loads((R/'Saved/Maps/latest-map.json').read_text());folder=R/'Saved/Maps'/pointer['directory'];manifest=json.loads((folder/'manifest.json').read_text());mapdata=np.load(folder/'occupancy.npz');log=mapdata['logodds'];seen=mapdata['seen'];res=float(mapdata['res']);mo=mapdata['origin'];origin=trial['map_origin_ground_truth_evaluation_only'];xy0=origin['position_m'][:2];angle=origin['yaw_rad'];c,s=math.cos(angle),math.sin(angle)
    frozen_scene=D/'source-snapshot/Content/RobotData/house-scene.json'
    xx,yy,occupied,reachable=reference(xy0,frozen_scene if frozen_scene.exists() else None);dx=xx-xy0[0];dy=yy-xy0[1];mx=c*dx+s*dy;my=-s*dx+c*dy;ix=np.floor((mx-mo[0])/res).astype(int);iy=np.floor((my-mo[1])/res).astype(int);inside=(ix>=0)&(iy>=0)&(ix<log.shape[1])&(iy<log.shape[0]);ix=np.clip(ix,0,log.shape[1]-1);iy=np.clip(iy,0,log.shape[0]-1);mapped=inside&(log[iy,ix]<-.8);observed=inside&(seen[iy,ix]>0)
    actual=np.array([r['ground_truth_evaluation_only']['position_m'][:2] for r in rows]);estimated=np.array([r['estimated_world_xy_evaluation_only'] for r in rows]);rooms=[]
    for name,(x0,x1,y0,y1) in ROOMS:
        region=reachable&(xx>=x0)&(xx<x1)&(yy>=y0)&(yy<y1);n=int(region.sum());entered=bool(((actual[:,0]>x0+.15)&(actual[:,0]<x1-.15)&(actual[:,1]>y0+.15)&(actual[:,1]<y1-.15)).any())
        rooms.append(dict(room=name,entered=entered,reference_navigable_center_area_m2=n*.05**2,observed_fraction=float((observed&region).sum()/max(1,n)),mapped_free_fraction=float((mapped&region).sum()/max(1,n))))
    distances=np.linalg.norm(np.diff(actual,axis=0),axis=1);elapsed=final['sim_time_s']-rows[0]['status']['sim_time_s'];errors=np.array([r['position_error_m'] for r in rows]);passed=all(r['entered'] and r['mapped_free_fraction']>=.90 for r in rooms) and manifest['complete'] and trial['external_actions']==[{'mode':'mapping'}] and trial.get('source_unchanged') is True
    result=dict(passed=passed,success_criterion='Every reference reachable room entered and at least 90% of its conservative navigable center space classified free, with robot reporting completion. This is a test criterion, not a guarantee of exhaustive mapping.',external_control_actions=trial['external_actions'],source_unchanged=trial.get('source_unchanged'),robot_stop_reason=final['message'],robot_map_complete=manifest['complete'],saved_map=pointer['directory'],duration_sim_s=elapsed,duration_wall_s=trial['ended_unix']-trial['started_unix'],rooms=rooms,rooms_entered=sum(r['entered'] for r in rooms),reference_room_count=len(rooms),mapped_free_fraction=float((mapped&reachable).sum()/reachable.sum()),reference_navigable_center_area_m2=float(reachable.sum()*.05**2),reference_nonconnected_navigable_space_note='Excluded by connected component at initial pose; conservative mesh bounds inflated by 19 cm',robot_map_statistics=final['map'],distance_traveled_m=float(distances.sum()),position_error_m=dict(median=float(np.median(errors)),max=float(errors.max()),p95=float(np.percentile(errors,95))),position_error_timing_note='Status can be newer than camera truth. Errors include capture-to-status skew; they are evaluation only.',limitations=['Reference occupancy uses conservative object bounds; native collisions use visible mesh surfaces in scene version 3','Rugs and 8/10 mm thresholds are considered traversable; shower tray is an obstacle','90% criterion does not demand seeing under inaccessible furniture','No camera noise or real house/Jetson validation'])
    (D/'results.json').write_text(json.dumps(result,indent=2));shutil.copy2(folder/'occupancy.npz',D/'occupancy.npz');shutil.copy2(folder/'manifest.json',D/'map-manifest.json')
    fig,axes=plt.subplots(1,2,figsize=(12,5),constrained_layout=True)
    background=np.full((*xx.shape,3),.94);background[occupied]=[.24,.28,.28];background[reachable&~mapped]=[.95,.80,.58];background[reachable&mapped]=[.53,.77,.64]
    axes[0].imshow(background,origin='lower',extent=[-3,5,-3,3]);axes[0].plot(actual[:,0],actual[:,1],color='#d76627',lw=1.5,label='Actual path (scoring only)');axes[0].plot(estimated[:,0],estimated[:,1],color='#2877a4',lw=1,label='Robot estimate');axes[0].scatter(*xy0,c='#172b32',marker='*',s=85,zorder=5)
    for name,(x0,x1,y0,y1) in ROOMS:axes[0].text((x0+x1)/2,(y0+y1)/2,name,ha='center',va='center',fontsize=8,bbox=dict(fc='white',alpha=.8,ec='none',pad=2))
    axes[0].set(title='Green: mapped free · amber: missing / not free',xlabel='World X (m)',ylabel='World Y (m)');axes[0].legend(loc='upper left',fontsize=8);axes[0].set_aspect('equal')
    t=np.array([r['status']['sim_time_s'] for r in rows]);axes[1].plot(t-t[0],errors*100,color='#2877a4');axes[1].set(title='Position estimate error (includes timing skew)',xlabel='Elapsed simulation time (s)',ylabel='Error (cm)');axes[1].grid(alpha=.25);fig.suptitle('Unaided full-house mapping trial · '+('PASS' if passed else 'DID NOT COMPLETE'),fontsize=15);fig.savefig(D/'coverage-and-path.png',dpi=155);plt.close(fig)
    print(json.dumps(result,indent=2));return result
if __name__=='__main__':score()
