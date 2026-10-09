"""Camera low-obstacle detector: dense floor-plane parallax plus multi-view cell voting.

Finds objects 3-19 cm tall that the 19 cm LiDAR plane passes over (shower tray, slippers, toy
blocks). Inputs: one camera image, its calibration and timestamp, and the robot's OWN estimated
pose at the image time (LidarBrain.camera_pose: x,y,yaw,roll,pitch,height_offset,floor_valid).
No simulator depth, labels, house geometry or ground-truth pose is read.

Per frame (5 Hz):
 1 reference = earlier frame (<=3 s) whose camera centre is 6-16 cm away and |dyaw|<=.35 rad
 2 warp the reference by the floor homography predicted from the robot's poses; DIS flow both ways
 3 measure the floor motion: RANSAC homography on corner-like floor pixels, decomposed with the
   calibrated camera height as scale; accepted only if it agrees with the measured robot motion
   (rotation <.04 rad, |t| within 30 %, direction cos>.9). Otherwise nothing: unknown, never free.
 4 aperture-safe parallax: the residual against the fitted floor is measured only along the local
   image gradient, converted to displacement along the epipolar line and triangulated. Height
   above the fitted floor must be 3-19 cm with h-2*sigma_h>2.5 cm, sigma_range<5 cm, range<1.2 m; near-vertical
   image edges are not used (occlusion edges of legs and door frames, wall shadows).
   Floor-level things (8-10 mm thresholds, rugs, bath mat) stay far below these gates.
 5 vote: a 5 cm map cell is confirmed once >=2 frames whose camera centres are >=3 cm apart each
   put >=2 points in it. Only newly confirmed cells are returned; reset_history() keeps them.
"""
import math,time,cv2,numpy as np
from concurrent.futures import ThreadPoolExecutor
from .mono import camera
from .geometry import CAMERA_MOUNT,LIDAR_MOUNT
# Plausible camera height: the calibrated mount (geometry.CAMERA_MOUNT, 15 cm) +/- 3 cm of body motion.
CAMERA_HEIGHT_RANGE=(CAMERA_MOUNT[2]-.03,CAMERA_MOUNT[2]+.03)
from .stereo import wrap
_POOL=[]
def _pool():
    # One worker runs the backward flow beside the forward flow (separate DIS objects): same numbers, ~5 ms less per frame.
    if not _POOL:_POOL.append(ThreadPoolExecutor(1,thread_name_prefix='floor-parallax'))
    return _POOL[0]

def _K(meta):return np.array([[float(meta['fx']),0,float(meta['cx'])],[0,float(meta['fy']),float(meta['cy'])],[0,0,1.]])
def _hmap(h,p):
    q=np.c_[p,np.ones(len(p))]@h.T;return q[:,:2]/q[:,2:3]
def floor_homography(pose_r,pose_t,k):
    """Image homography reference->current for the z=0 floor, from the robot's own poses."""
    rr,cr=camera(pose_r);rt,ct=camera(pose_t);n=np.array([0,0,1.])
    m=rt.T@(rr-np.outer(cr-ct,n@rr)/cr[2]);h=k@m@np.linalg.inv(k);return h/h[2,2]

class FloorParallax:
    """One-frame measurement: returns (Nx3 map points [x,y,height], Nx2 pixels) of unconfirmed raised structure."""
    def __init__(self,min_base=.06,max_base=.16,max_age=3.,max_dyaw=.35,hmin=.03,hmax=LIDAR_MOUNT[2],max_range=1.2,grad=6.,hconf=.025,
                 max_sig_r=.05,min_dg=.35,max_edge_cos=.95,vmin=12,step=2,history=32,parallel=True):
        self.cfg=dict(min_base=min_base,max_base=max_base,max_age=max_age,max_dyaw=max_dyaw,hmin=hmin,hmax=hmax,max_range=max_range,grad=grad,
                      hconf=hconf,max_sig_r=max_sig_r,min_dg=min_dg,max_edge_cos=max_edge_cos,vmin=vmin,step=step,history=history)
        self.dis=[cv2.DISOpticalFlow_create(cv2.DISOPTICAL_FLOW_PRESET_MEDIUM) for _ in range(2)];self.parallel=parallel;self.hist=[];self.grid=None;self.status={'state':'waiting for images'}
    def reset(self):self.hist=[];self.status={'state':'history cleared'}
    def _reference(self,pose,stamp):
        c=self.cfg;ct=camera(pose)[1];best=None
        for g,p,s,cp in self.hist:
            b=float(np.linalg.norm(cp-ct))
            if not 0<stamp-s<=c['max_age'] or abs(wrap(pose[2]-p[2]))>c['max_dyaw'] or not c['min_base']<=b<=c['max_base']:continue
            if best is None or b>best[0]:best=(b,g,p,s)
        return best
    def process(self,gray,meta,pose):
        c=self.cfg;k=_K(meta);stamp=float(meta['sim_time_s']);pose=np.asarray(pose,float);H,W=gray.shape;empty=(np.empty((0,3)),np.empty((0,2)))
        if self.hist and stamp==self.hist[-1][2]:self.status={'state':'repeated image'};return empty
        if self.hist and (self.hist[-1][0].shape!=gray.shape or stamp<self.hist[-1][2]):self.hist=[]
        ref=self._reference(pose,stamp);self.hist=[x for x in self.hist if stamp-x[2]<=c['max_age']][-(c['history']-1):]+[(gray,pose.copy(),stamp,camera(pose)[1])]
        if ref is None:self.status={'state':'needs 6 cm of travel'};return empty
        base,gr,pr,sr=ref;info=dict(baseline_m=round(base,4),reference_age_s=round(stamp-sr,3))
        h0=floor_homography(pr,pose,k);warped=cv2.warpPerspective(gr,h0,(W,H));valid=cv2.warpPerspective(np.full_like(gr,255),h0,(W,H),flags=cv2.INTER_NEAREST)>0
        # Flow on the full image (pyramid context matters); per-pixel work only below row cy+vmin, the only rows any later step uses.
        if self.parallel:job=_pool().submit(self.dis[1].calc,warped,gray,None);f=self.dis[0].calc(gray,warped,None);b=job.result()
        else:f=self.dis[0].calc(gray,warped,None);b=self.dis[1].calc(warped,gray,None)
        s=c['step'];lo=int(math.floor(k[1,2]+min(c['vmin'],25)))//s*s;top=max(0,lo-8)
        if self.grid is None or self.grid[0]!=(H,W,s,lo):
            ys,xs=np.mgrid[lo:H:s,0:W:s];self.grid=((H,W,s,lo),ys.ravel(),xs.ravel(),ys.shape)
        _,ys,xs,shape=self.grid;fs=f[ys,xs]
        back=cv2.remap(b,(xs+fs[:,0]).astype(np.float32).reshape(shape),(ys+fs[:,1]).astype(np.float32).reshape(shape),cv2.INTER_LINEAR).reshape(-1,2)
        ok=valid[ys,xs]&(np.hypot(*(fs+back).T)<.5);cur=gray[top:];yc=ys-top
        gx=cv2.Sobel(cur,cv2.CV_32F,1,0,ksize=3);gy=cv2.Sobel(cur,cv2.CV_32F,0,1,ksize=3)
        Jxx=cv2.blur(gx*gx,(5,5))[yc,xs];Jxy=cv2.blur(gx*gy,(5,5))[yc,xs];Jyy=cv2.blur(gy*gy,(5,5))[yc,xs]
        tr=Jxx+Jyy;lmin=tr/2-np.sqrt(np.maximum(0,tr*tr/4-(Jxx*Jyy-Jxy**2)))
        th=.5*np.arctan2(2*Jxy,Jxx-Jyy);g=np.c_[np.cos(th),np.sin(th)];pt=np.c_[xs,ys].astype(float)
        rr,cr=camera(pr);rt,ct=camera(pose);er=rt.T@rr;et=rt.T@(cr-ct);d=float(np.clip(cr[2],*CAMERA_HEIGHT_RANGE));bl=float(np.linalg.norm(et))
        corner=ok&(lmin>150)&(pt[:,1]>k[1,2]+25)
        if corner.sum()<80:self.status=dict(state='too little floor texture',**info);return empty
        hf,inl=cv2.findHomography(_hmap(np.linalg.inv(h0),pt[corner]+fs[corner]),pt[corner],cv2.RANSAC,1.0,maxIters=500)
        if hf is None or inl.mean()<=.5:self.status=dict(state='floor motion uncertain',**info);return empty
        _,Rs,Ts,Ns=cv2.decomposeHomographyMat(hf,k);cand=[]
        for Ri,Ti,Ni in zip(Rs,Ts,Ns):
            Ti=Ti[:,0]*d;Ni=Ni[:,0];L=np.linalg.norm(Ti);re=np.linalg.norm(cv2.Rodrigues(Ri@er.T)[0])
            if Ni[1]<.9 or re>.04 or not .7*bl<L<1.3*bl or Ti@et/max(1e-9,L*bl)<.9:continue
            cand.append((re+abs(L-bl),Ri,Ti,Ni))
        if not cand:self.status=dict(state='floor motion disagrees with robot motion',**info);return empty
        _,R,t,n=min(cand,key=lambda x:x[0]);e=k@t;e=e[:2]/e[2] if abs(e[2])>1e-9 else np.array([1e6,1e6])
        dv=pt-e;rn=np.linalg.norm(dv,axis=1);dv/=np.maximum(rn,1e-9)[:,None];dg=np.sum(dv*g,1)
        sd=np.sqrt(np.maximum(0,Jxx*dv[:,0]**2+2*Jxy*dv[:,0]*dv[:,1]+Jyy*dv[:,1]**2))/8
        # Near-vertical image edges (gradient within ~18 deg of horizontal) are skipped: their normal flow is horizontal, the
        # direction corrupted by occlusion edges of legs/door frames, wall shadows and yaw error. Low obstacles show their
        # near-horizontal top/bottom edges. Recordings: removes every false cell >20 cm from an obstacle, keeps all approaches.
        idx=np.flatnonzero(ok&(np.abs(dg)>c['min_dg'])&(sd>c['grad'])&(rn>20)&(pt[:,1]>k[1,2]+c['vmin'])&(np.abs(g[:,0])<c['max_edge_cos']))
        if len(idx)<20:self.status=dict(state='little texture',**info);return empty
        P=pt[idx];D=dv[idx];G=g[idx];hinv=np.linalg.inv(hf)
        # Normal-flow parallax: only the flow component along the image gradient is trusted (aperture problem).
        mu=np.sum(((P+fs[idx])-_hmap(h0@hinv,P))*G,1)/np.sum(D*G,1)
        P0=k@np.c_[np.eye(3),np.zeros(3)];P1=k@np.c_[R,t]
        def tri(sel,m):
            X=cv2.triangulatePoints(P0,P1,_hmap(hinv,P[sel]+m[:,None]*D[sel]).T,P[sel].T).T;return X[:,:3]/X[:,3:4]
        with np.errstate(divide='ignore',invalid='ignore'):
            X=tri(np.arange(len(P)),mu);hgt=d-X@n;rng=np.hypot(X[:,0],X[:,2])
            keep=np.isfinite(X).all(1)&(X[:,2]>.15)&((X@R.T+t)[:,2]>.12)&(hgt>c['hmin'])&(hgt<c['hmax'])&(rng<c['max_range']+bl);kk=np.flatnonzero(keep)
            if len(kk):
                Xa=tri(kk,mu[kk]-.7);Xb=tri(kk,mu[kk]+.7)
                sh=np.abs((d-Xb@n)-(d-Xa@n))/2;sr=np.abs(np.hypot(Xb[:,0],Xb[:,2])-np.hypot(Xa[:,0],Xa[:,2]))/2
                keep[kk]&=(hgt[kk]-2*sh>c['hconf'])&(sr<c['max_sig_r'])
        world=X[keep]@rr.T+cr;world[:,2]=hgt[keep]
        self.status=dict(state='floor motion verified',points=int(keep.sum()),**info);return world,P[keep]

class CellVoter:
    """Confirm a map cell after >=need frames (camera centres >=min_move apart) each put >=min_points in it."""
    def __init__(self,res=.05,need=2,min_points=2,min_move=.03,max_age=None):
        self.res=res;self.need=need;self.min_points=min_points;self.min_move=min_move;self.max_age=max_age;self.hits={};self.confirmed=set()
    def clear_pending(self):self.hits={}
    def add(self,world,camera_xy,stamp=0.):
        """Returns (Nx2 centres of newly confirmed cells, mask of the input points that fell in them)."""
        new=[];world=np.asarray(world).reshape(-1,3) if len(world) else np.empty((0,3))
        if not len(world):return np.empty((0,2)),np.zeros(0,bool)
        ij=np.floor(world[:,:2]/self.res).astype(int);u,inv,cnt=np.unique(ij,axis=0,return_inverse=True,return_counts=True);cam=np.asarray(camera_xy,float)[:2]
        for cell,m in zip(map(tuple,u.tolist()),cnt):
            if m<self.min_points or cell in self.confirmed:continue
            h=self.hits.setdefault(cell,[])
            if self.max_age is not None:h[:]=[x for x in h if stamp-x[1]<=self.max_age]
            if not h or np.linalg.norm(h[-1][0]-cam)>self.min_move:h.append((cam,stamp))
            if len(h)>=self.need:self.confirmed.add(cell);self.hits.pop(cell,None);new.append(cell)
        fresh=set(new);mask=np.array([cell in fresh for cell in map(tuple,u.tolist())],bool)[inv.ravel()]
        return (np.array(new,float).reshape(-1,2)+.5)*self.res,mask
    def cells(self):return (np.array(sorted(self.confirmed),float).reshape(-1,2)+.5)*self.res

class LowObstacleCamera:
    """Controller-facing detector. process() -> (N,2) NEWLY confirmed obstacle points in map metres
    (feed to LidarGrid.add_camera_obstacles). reset_history() forgets images and unconfirmed votes
    (call whenever the pose is not trustworthy) but keeps confirmed cells. status is JSON-safe."""
    def __init__(self,res=.05,vote_max_age=None,**detector):
        self.detector=FloorParallax(**detector);self.voter=CellVoter(res=res,max_age=vote_max_age);self.last_stamp=None;self.frames=0;self.status={'state':'waiting for images','confirmed_cells':0}
    def reset_history(self):
        self.detector.reset();self.voter.clear_pending()
        self.status=dict(state='waiting for stable, localized view',new_cells=0,confirmed_cells=len(self.voter.confirmed),obstacle_pixels=[])
    def confirmed_points(self):return self.voter.cells()
    def process(self,image,meta,pose):
        t0=time.perf_counter();gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY) if image.ndim==3 else image.copy()  # history keeps it
        pose=np.asarray(pose,float);world,pixels=self.detector.process(gray,meta,pose)
        new,mask=self.voter.add(world,camera(pose)[1][:2],float(meta['sim_time_s']));self.last_stamp=float(meta['sim_time_s']);self.frames+=1
        self.status=dict(self.detector.status,frame=meta.get('frame'),image_stamp=self.last_stamp,candidate_points=int(len(world)),new_cells=int(len(new)),
                         confirmed_cells=len(self.voter.confirmed),pending_cells=len(self.voter.hits),obstacle_pixels=np.round(pixels[mask][:40],1).tolist(),
                         ms=round((time.perf_counter()-t0)*1000,1))
        return new
