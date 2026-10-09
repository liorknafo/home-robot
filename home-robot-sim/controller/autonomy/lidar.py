"""Polar scans -> motion-compensated points -> geometric scan matching.
No world geometry, simulator transforms, semantic room labels or camera depth.
Noise, planar scan and confidence limits are documented in lidar-review.html.
"""
import math,json,cv2,numpy as np
from collections import deque
from .localization import Localizer
from .stereo import wrap
from .grid import Grid
from .geometry import PLAN_RADIUS,R,LIDAR_MOUNT

def transform(points,pose):
    c,s=math.cos(pose[2]),math.sin(pose[2]);return np.asarray(points)@np.array([[c,s],[-s,c]])+pose[:2]
def inverse(points,pose):
    c,s=math.cos(pose[2]),math.sin(pose[2]);return (np.asarray(points)-pose[:2])@np.array([[c,-s],[s,c]])
def voxel(points,res=.025):
    if len(points)==0:return np.empty((0,2),np.float32)
    _,idx=np.unique(np.floor(points/res).astype(np.int32),axis=0,return_index=True)
    return np.ascontiguousarray(points[idx],np.float32)

def scan_points(scan,history=()):
    """Missing/invalid beams are unknown, never treated as free to max range."""
    r=np.asarray(scan['ranges_mm'],float)/1000 if 'ranges_mm' in scan else np.asarray(scan['ranges_m'],float);a=scan['angle_min_rad']+np.arange(len(r))*scan['angle_increment_rad']
    valid=np.isfinite(r)&(r>=scan['range_min_m'])&(r<=scan['range_max_m'])
    offset=np.asarray(scan['sensor_in_base_m'][:2]);p=np.column_stack((r*np.cos(a),r*np.sin(a)))+offset
    p=p[valid];times=(scan['start_time_s']+np.asarray(scan['time_offsets_ms'])/1000 if 'time_offsets_ms' in scan else np.asarray(scan['times_s']))[valid]
    if len(history)>=2 and len(p):
        h=list(history);ht=np.array([x[0] for x in h]);hp=np.array([x[1] for x in h]);yaw=np.unwrap(hp[:,2]);end=scan['sim_time_s']
        pose=np.array([np.interp(end,ht,hp[:,0]),np.interp(end,ht,hp[:,1]),np.interp(end,ht,yaw)])
        theta=np.interp(times,ht,yaw);x=np.interp(times,ht,hp[:,0]);y=np.interp(times,ht,hp[:,1]);c,s=np.cos(theta),np.sin(theta)
        world=np.column_stack((c*p[:,0]-s*p[:,1]+x,s*p[:,0]+c*p[:,1]+y));p=inverse(world,pose)
    return np.ascontiguousarray(p,np.float32)

# Whole-map relocalization needs more agreement than tracking (fit() accepts .68): a result
# must explain GLOBAL_ACCEPT of the scan and beat any other distinct place by GLOBAL_MARGIN.
GLOBAL_ACCEPT=.80;GLOBAL_MARGIN=.10
# A verified pose stays the relocalization starting point while odometry has carried the
# robot less than this since the fix (e.g. idle between Go-to targets).
PRIOR_TRAVEL_M=.5
def relocalization_failure_message(q):
    if q.get('ambiguous'):return 'Two places in the saved map fit the LiDAR view about equally (%d%% and %d%% of points); not moving.'%(100*q['best'],100*q['second'])
    best=q.get('best')
    return 'No place in the saved map matches the LiDAR view'+(' (best %d%% of points, %d%% needed)'%(100*best,100*GLOBAL_ACCEPT) if best else '')+'; not moving.'

class LidarGrid(Grid):
    def path_step_cost(self,clearance):
        # Keep room for steering and pose corrections. The former 5 cm / d
        # term scarcely distinguished a one-cell-wide route from open floor.
        # Narrow passages remain possible when no wider route exists.
        return 1+(.12/max(.04,clearance-self.radius+.04))**2
    def __init__(self,*args,**kwargs):
        # camera_obstacles holds the planning cells (with margin); camera_core
        # only the measured cells, used for exact body-sweep checks.
        super().__init__(*args,**kwargs);self.camera_obstacles=np.zeros_like(self.seen,bool);self.camera_core=np.zeros_like(self.seen,bool);self.driven=np.zeros_like(self.seen,bool)
        # Round body radius R (.1595; it turns about base_link, so turning stays
        # inside it) + localization .025 + half a 5 cm cell diagonal .03536 +
        # tracking allowance .025 = R + .0854, rounded up (geometry.PLAN_RADIUS).
        self.radius=PLAN_RADIUS;self.sensor_xy=np.array(LIDAR_MOUNT[:2])
        # Pinch points where the local planner stalled in narrow-passage mode: the 5 cm grid
        # (cell centres) routed through a gap whose measured points leave less than
        # DRIVE_CLEARANCE on both sides. Narrow-passage planning avoids them afterwards, for
        # NARROW_BLOCK_S (LidarBrain prunes older ones, so one stall does not close a real
        # doorway for the rest of the run); narrow_block_times holds when each was added.
        self.narrow_blocks=[];self.narrow_block_times=[]
    def masks(self):
        free,occupied,unknown,safe,clearance=super().masks()
        if self.radius<PLAN_RADIUS and getattr(self,'narrow_blocks',None):
            safe=safe.copy();reach=int(np.ceil(.08/self.res))
            for xy in self.narrow_blocks:
                cy,cx=self.cell(xy)
                for y in range(cy-reach,cy+reach+1):
                    for x in range(cx-reach,cx+reach+1):
                        if self.valid((y,x)) and np.linalg.norm(self.world((y,x))-xy)<=.08+1e-9:safe[y,x]=False
        return free,occupied,unknown,safe,clearance
    def add_camera_obstacles(self,world,margin=1):
        for point in world:
            cell=self.cell(point[:2])
            if self.valid(cell):
                # One-cell uncertainty margin around triangulated features; a
                # bumper contact is located directly and uses no margin.
                y,x=cell;self.camera_obstacles[max(0,y-margin):min(self.n,y+margin+1),max(0,x-margin):min(self.n,x+margin+1)]=True;self.camera_core[y,x]=True
        self.logodds[self.camera_obstacles]=5;self.seen[self.camera_obstacles]=np.maximum(1,self.seen[self.camera_obstacles])
    def save(self,folder):
        super().save(folder);np.savez_compressed(folder/'camera-obstacles.npz',occupied=self.camera_obstacles,core=self.camera_core,driven=self.driven)
    def load(self,folder):
        super().load(folder);p=folder/'camera-obstacles.npz';d=np.load(p,allow_pickle=False) if p.exists() else {}
        self.camera_obstacles=d['occupied'] if 'occupied' in d else np.zeros_like(self.seen,bool);self.camera_core=d['core'] if 'core' in d else self.camera_obstacles.copy()
        self.driven=d['driven'] if 'driven' in d else np.zeros_like(self.seen,bool)
    def add_driven(self,a,b):
        """Record ground the robot's centre drove over (a 3-cell, 4-connected corridor)."""
        ca,cb=self.cell(a),self.cell(b)
        if not(self.valid(ca) and self.valid(cb)):return
        line=np.zeros(self.seen.shape,np.uint8);cv2.line(line,(ca[1],ca[0]),(cb[1],cb[0]),1,1)
        self.driven|=cv2.dilate(line,np.ones((3,3),np.uint8))>0
        self.logodds[self.camera_obstacles]=5
    def image(self,*args,**kwargs):
        im=super().image(*args,**kwargs);im[np.flipud(self.camera_obstacles)]=[40,125,225];return im
    def integrate_scan(self,points,pose):
        if len(points)<60:return
        if self.last_pose is not None and np.linalg.norm(pose[:2]-self.last_pose[:2])<.025 and abs(wrap(pose[2]-self.last_pose[2]))<.06 and self.updates>=5:return
        self.last_pose=pose.copy();self.updates+=1
        # Free rays start at the LiDAR origin (scan sensor_in_base_m, stored by LidarBrain.observe_scan).
        free=np.zeros_like(self.seen,np.uint8);occupied=np.zeros_like(free);start=self.cell(transform(np.array([getattr(self,'sensor_xy',LIDAR_MOUNT[:2])]),pose)[0]);world=transform(points,pose)
        for point in world:
            end=self.cell(point)
            if not self.valid(end):continue
            cv2.line(free,(start[1],start[0]),(end[1],end[0]),1,1);occupied[end]=1
        # The measured robot footprint is known occupied by the robot itself.
        center=self.cell(pose[:2]);cv2.circle(free,(center[1],center[0]),2,1,-1)
        f=(free>0)&(occupied==0)&~self.camera_obstacles;o=occupied>0
        self.logodds[f]=np.maximum(-5,self.logodds[f]-.7);self.logodds[o]=np.minimum(5,self.logodds[o]+1.0)
        seen=f|o;self.seen[seen]=np.minimum(65534,self.seen[seen])+1

class LidarLocalizer(Localizer):
    def __init__(self):
        super().__init__(None);self.points=np.empty((0,2),np.float32);self.tree=None;self.normals=None;self.last_key_pose=None;self.quality={};self.pending=None
        self.map_name=None;self.search=None;self.last_good_stamp=None;self.global_origin=False;self.last_global={}
    def rebuild(self):
        self.points=voxel(self.points);self.search=None
        if len(self.points)<20:return
        self.tree=cv2.flann_Index(self.points,dict(algorithm=1,trees=4))
        ids,_=self.tree.knnSearch(self.points,6,params={'checks':32});neighbors=self.points[ids];center=neighbors.mean(axis=1);d=neighbors-center[:,None,:];cov=np.einsum('nki,nkj->nij',d,d);_,vectors=np.linalg.eigh(cov);self.normals=vectors[:,:,0]
    def fit(self,points,initial,limit=.30,iterations=18):
        if self.tree is None or len(points)<60:return None,{}
        pose=np.array(initial,float).copy();points=points[::max(1,len(points)//450)]
        for _ in range(iterations):
            transformed=transform(points,pose).astype(np.float32);ids,d2=self.tree.knnSearch(transformed,1,params={'checks':32});ids=ids[:,0];distance=np.sqrt(d2[:,0]);ok=distance<limit
            if ok.sum()<60:return None,{}
            p=transformed[ok];q=self.points[ids[ok]];n=self.normals[ids[ok]];residual=np.sum((p-q)*n,axis=1);rel=p-pose[:2]
            j=np.column_stack((n[:,0],n[:,1],-n[:,0]*rel[:,1]+n[:,1]*rel[:,0]));weight=np.minimum(1.,.025/np.maximum(.001,np.abs(residual)));h=j.T@(weight[:,None]*j);b=j.T@(weight*residual)
            eig=np.linalg.eigvalsh(h)
            if eig[0]<2 or eig[-1]/eig[0]>2000:return None,{'degenerate':True}
            delta=-np.linalg.solve(h+np.eye(3)*1e-6,b);delta[:2]=np.clip(delta[:2],-.10,.10);delta[2]=np.clip(delta[2],-.10,.10);pose+=delta;pose[2]=wrap(pose[2])
            if np.linalg.norm(delta)<.0001:break
        world=transform(points,pose).astype(np.float32);_,d2=self.tree.knnSearch(world,1,params={'checks':48});distance=np.sqrt(d2[:,0]);inliers=distance<.065
        quality={'inliers':int(inliers.sum()),'fraction':float(inliers.mean()),'median_error_m':float(np.median(distance)),'degenerate':False}
        return (pose if inliers.mean()>.68 and np.median(distance)<.035 else None),quality
    def _search_space(self):
        """Distance field of the map points (5 cm) and candidate robot positions: one per 20 cm
        cell, within 3 m of a saved observation pose and at least 12 cm from mapped geometry."""
        if self.search is not None:return self.search
        res=.05;o=self.points.min(0)-1.;n=np.ceil((self.points.max(0)+1.-o)/res).astype(int)+1
        img=np.ones((n[1],n[0]),np.uint8);ij=((self.points-o)/res).astype(int);img[ij[:,1],ij[:,0]]=0
        dist=cv2.distanceTransform(img,cv2.DIST_L2,5)*res
        keys=np.array([k['pose'][:2] for k in self.keyframes])[::3] if self.keyframes else self.points.mean(0)[None]
        gx,gy=np.meshgrid(np.arange(o[0],o[0]+n[0]*res,.10),np.arange(o[1],o[1]+n[1]*res,.10));xy=np.c_[gx.ravel(),gy.ravel()]
        xy=xy[np.min(np.linalg.norm(xy[:,None,:]-keys[None,:,:],axis=2),axis=1)<3.]
        gi=((xy-o)/res).astype(int);xy=xy[dist[gi[:,1],gi[:,0]]>.12]
        _,first=np.unique(np.round(xy/.2).astype(int),axis=0,return_index=True)
        self.search=(dist,o,res,xy[np.sort(first)]);return self.search
    def global_match(self,points):
        """Whole-map search with no trusted starting pose (correlative, then scan fit).
        Coarse: 20 cm / 6 deg with a wide 0.30 m kernel; the best 24 distinct peaks are
        refined on a 5 cm / 1.5 deg lattice (0.10 m kernel) and the best 10 are fitted.
        A result must explain GLOBAL_ACCEPT of the points and beat every other distinct
        fit by GLOBAL_MARGIN: on 2026-10-04 a place 4.3 m away explained 68 % of one
        house scan, which tracking-level acceptance (.68) let through."""
        if self.tree is None or len(points)<60:return None,{'no_match':True,'best':None}
        dist,o,res,xy=self._search_space();pts=points[::max(1,len(points)//90)]
        def score(cand,rotated,kernel):
            ij=np.floor((cand[:,None,:]+rotated[None,:,:]-o)/res).astype(int)
            d=dist[np.clip(ij[...,1],0,dist.shape[0]-1),np.clip(ij[...,0],0,dist.shape[1]-1)];return np.clip(1-d/kernel,0,1).mean(1)
        def rotate(a):
            c,s=math.cos(a),math.sin(a);return pts@np.array([[c,s],[-s,c]])
        coarse=[]
        for a in np.arange(-math.pi,math.pi,math.radians(6)):
            sc=score(xy,rotate(a),.30);top=np.argpartition(-sc,min(12,len(sc)-1))[:12];coarse+=[(sc[i],xy[i,0],xy[i,1],a) for i in top]
        peaks=[]
        for c in sorted(coarse,key=lambda c:-c[0]):
            if any(math.hypot(c[1]-p[1],c[2]-p[2])<.35 and abs(wrap(c[3]-p[3]))<math.radians(25) for p in peaks):continue
            peaks.append(c)
            if len(peaks)>=24:break
        offsets=np.array([(dx,dy) for dx in np.arange(-.15,.151,.05) for dy in np.arange(-.15,.151,.05)]);fine=[]
        for _,x,y,a0 in peaks:
            cand=np.array([x,y])+offsets;best=None
            for a in a0+np.radians(np.arange(-6,6.1,1.5)):
                sc=score(cand,rotate(a),.10);i=int(np.argmax(sc))
                if best is None or sc[i]>best[0]:best=(sc[i],np.array([cand[i,0],cand[i,1],wrap(a)]))
            fine.append(best)
        fitted=[]
        for _,seed in sorted(fine,key=lambda f:-f[0])[:10]:
            pose,q=self.fit(points,seed,.35,30)
            if pose is None or any(np.linalg.norm(pose[:2]-f[1][:2])<.15 and abs(wrap(pose[2]-f[1][2]))<.15 for f in fitted):continue
            fitted.append((q['fraction'],pose,q))
        fitted.sort(key=lambda f:-f[0])
        if not fitted or fitted[0][0]<GLOBAL_ACCEPT:return None,{'no_match':True,'best':fitted[0][0] if fitted else None}
        if len(fitted)>1 and fitted[1][0]>fitted[0][0]-GLOBAL_MARGIN:return None,{'ambiguous':True,'best':fitted[0][0],'second':fitted[1][0]}
        return fitted[0][1],fitted[0][2]
    def adopt_verified_pose(self,old):
        """Continue from a pose verified on this same map (the previous Go-to's arrival, or the
        mapping run that saved it), carried by odometry since, instead of a whole-map search."""
        if old is None or self.map_name is None or getattr(old,'map_name',None)!=self.map_name:return False
        if getattr(old,'last_good_stamp',None) is None or old.unverified_travel_m>=PRIOR_TRAVEL_M:return False
        self.pose=old.pose.copy();self.history=deque(old.history,maxlen=100);self.last_state=old.last_state;self.previous_gyro=old.previous_gyro
        self.last_good_stamp=old.last_good_stamp;self.unverified_travel_m=old.unverified_travel_m;self.heading_uncertainty=getattr(old,'heading_uncertainty',self.heading_uncertainty);self.tracking='checking the last verified position';return True
    def update_scan(self,points,stamp,mapping=False,relocalizing=False):
        pred=self.at(stamp)
        if self.tree is None:
            if not mapping or len(points)<120:return False,pred
            # The first scan defines the map coordinate frame, never the house frame.
            self.pose[:]=0;self.history.clear();self.history.append((stamp,self.pose.copy()));pred=self.pose.copy();self.points=transform(points,pred);self.rebuild();result=pred;quality={'inliers':len(points),'fraction':1.,'median_error_m':0.}
        elif relocalizing:
            if self.pending is not None:
                result,quality=self.fit(points,self.pending,.35)
                # A whole-map candidate keeps the whole-map standard while it is confirmed.
                if result is not None and self.global_origin and quality['fraction']<GLOBAL_ACCEPT:result=None
            else:
                # After a brief disagreement, or while a verified pose is still carried by
                # odometry (PRIOR_TRAVEL_M), the odometry pose is close: try a wide local fit
                # before the whole-map search. Three consistent matches are still required.
                verified=self.last_good_stamp is not None and (stamp-self.last_good_stamp<15 or self.unverified_travel_m<PRIOR_TRAVEL_M)
                result,quality=self.fit(points,pred,.35,30) if verified else (None,{})
                self.global_origin=result is None
                if result is None:result,quality=self.global_match(points);self.last_global=quality
            if result is not None:
                if self.pending is not None and np.linalg.norm(result[:2]-self.pending[:2])<.10 and abs(wrap(result[2]-self.pending[2]))<.12:self.relocalization_hits+=1
                else:self.relocalization_hits=1
                self.pending=result.copy()
                if self.relocalization_hits<3:self.tracking='confirming LiDAR map match';return False,pred
            else:self.pending=None;self.relocalization_hits=0
        else:
            result,quality=self.fit(points,pred)
            if result is not None and (np.linalg.norm(result[:2]-pred[:2])>.20 or abs(wrap(result[2]-pred[2]))>.20):result=None
        self.quality=quality;self.visual_inliers=quality.get('inliers',0)
        if result is None:self.uncertainty+=.025;self.tracking='LiDAR geometry uncertain';return False,pred
        self.motion_residual_m=float(np.linalg.norm(result[:2]-pred[:2]));correction=result-pred;correction[2]=wrap(correction[2]);self.pose+=correction;self.pose[2]=wrap(self.pose[2]);self.history=type(self.history)([(t,np.array([p[0]+correction[0],p[1]+correction[1],wrap(p[2]+correction[2])])) for t,p in self.history],maxlen=100)
        self.last_visual_time=self.last_good_stamp=stamp;self.unverified_travel_m=0.;self.uncertainty=.025;self.heading_uncertainty=.005;self.tracking='LiDAR scan matched to map';self.motion_inconsistent=False
        if mapping and (self.last_key_pose is None or np.linalg.norm(result[:2]-self.last_key_pose[:2])>.20 or abs(wrap(result[2]-self.last_key_pose[2]))>.45):
            self.keyframes.append({'pose':result.copy()});self.last_key_pose=result.copy();self.points=np.vstack((self.points,transform(points,result)));self.rebuild()
        return True,result
    def save(self,folder):
        folder.mkdir(parents=True,exist_ok=True);self.map_name=folder.name;np.savez_compressed(folder/'lidar-map.npz',points=self.points,poses=np.array([k['pose'] for k in self.keyframes]).reshape(-1,3))
    def load(self,folder):
        d=np.load(folder/'lidar-map.npz',allow_pickle=False);self.map_name=folder.name;self.points=d['points'];self.keyframes=[{'pose':p} for p in d['poses']];self.rebuild();self.uncertainty=.5;self.tracking='LiDAR map loaded; position not yet located'
