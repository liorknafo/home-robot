"""Camera low-obstacle and overhang detector: dense plane parallax plus multi-view cell voting.

Finds what the LiDAR's single scan plane cannot: objects 3 cm tall up to the LiDAR plane (LIDAR_MOUNT z) that it passes over
(shower tray, slippers, toy blocks), and furniture whose underside lies in its blind band, between the plane and the robot top
(ROBOT_TOP, the LiDAR turret cap): the beam passes under it and only the turret meets it (task M, LIO-121: the Unreal sofa base,
underside 130 mm, against task L's plane 127.5 / top 135.15 mm). Everything from 3 cm up to ROBOT_TOP + OVERHANG_MARGIN is
reported; higher things (table tops, a 20 cm underside) are not. Inputs: one camera image, its calibration and timestamp, and
the robot's OWN estimated pose at the image time (LidarBrain.camera_pose: x,y,yaw,roll,pitch,height_offset,floor_valid).
No simulator depth, labels, house geometry or ground-truth pose is read.

Per frame (5 Hz):
 1 reference = earlier frame (<=3 s) whose camera centre is 6-16 cm away and |dyaw|<=.35 rad
 2 warp the reference by the floor homography predicted from the robot's poses; DIS flow both ways
 3 measure the floor motion: RANSAC homography on corner-like floor pixels, decomposed with the
   calibrated camera height as scale; accepted only if it agrees with the measured robot motion
   (rotation <.04 rad, |t| within 30 %, direction cos>.9). Otherwise nothing: unknown, never free.
 4 floor pass (rows below cy+vmin): aperture-safe parallax: the residual against the fitted floor is measured only along the
   local image gradient, converted to displacement along the epipolar line and triangulated. Height above the fitted floor
   must be 3 cm..hmax with h-2*sigma_h>2.5 cm, sigma_range<5 cm, range<1.2 m; near-vertical image edges are not used (occlusion
   edges of legs and door frames, wall shadows). Floor-level things (8-10 mm thresholds, rugs, bath mat) stay far below these gates.
 4b band pass (rows above, where everything higher than the 8 cm camera appears): the same measurement against the rotation
   alone (the plane at infinity) with the floor-fitted motion (_band). An overhang shows little more than its bottom edge, a
   long straight edge: its flow round trip is checked across it only, the edge must make >= 37 deg with the epipolar line
   (band_min_dg) and each point's uncertainty grows as that angle shrinks. Recorded Unreal drives at the sofa (task M): main's
   detector 0 cells in 5/5; this one 11-17 cells per drive, the first 0.42-0.79 m before the lens reaches the face (the
   turret is 62 mm behind the lens). Viewed obliquely the edge is ranged short: cells up to ~25 cm in front of it. Band-pass
   points lower than OVERHANG_MIN_H (the LiDAR plane - 3 cm) are low obstacles, never overhangs: legs and posts (task Q).
 5 vote: a 5 cm map cell is confirmed once >=2 frames whose camera centres are >=3 cm apart each
   put >=2 points in it. Only newly confirmed cells are returned; reset_history() keeps them. A cell confirmed by band-pass
   points at least OVERHANG_MIN_H high only, one view from within OVERHANG_MAX_RANGE, with no lower point ever within one cell
   of it, is an overhang edge
   (new_overhangs): a low object taller than the camera also shows its lower part to the floor pass, a leg its lower part to
   the band pass; an overhang has nothing below its edge. The controller maps the free floor behind an overhang edge as under
   it (LidarGrid.add_overhang_shadow).

OVERHANG_MARGIN (2 cm above ROBOT_TOP): nose-up pitch raises the turret's front edge (x .084 m) 5 mm at 3.5 deg and 8-9 mm at
6 deg (the largest pitch in the Unreal trials, a rug edge); the camera height scales every height (2 mm on 80 mm is 4 mm at
150 mm) and heights are measured from the floor the robot stands on (a rug is 6-9 mm); the triangulation noise is 3-10 mm at
0.6-1.2 m. The price: furniture the robot would clear by less than 2 cm is also kept off its map.
"""
import math,time,cv2,numpy as np
from concurrent.futures import ThreadPoolExecutor
from .mono import camera
from .geometry import CAMERA_MOUNT,LIDAR_MOUNT,ROBOT_TOP
# Plausible camera height: the calibrated mount (geometry.CAMERA_MOUNT z, 8 cm since task F) +/- 3 cm of body motion.
CAMERA_HEIGHT_RANGE=(CAMERA_MOUNT[2]-.03,CAMERA_MOUNT[2]+.03)
# Highest point reported: the robot top (geometry.ROBOT_TOP, the LiDAR turret cap) plus OVERHANG_MARGIN (see the docstring).
OVERHANG_MARGIN=.02
HMAX=ROBOT_TOP+OVERHANG_MARGIN
# Lowest band-pass point that counts (task Q, LIO-137): the LiDAR plane less 3 cm. The band pass measures every image row above
# the floor pass's, which at 1 m reaches down to ~5 cm; the floor pass skips near-vertical image edges there, the band pass
# measures them as well. In task N's Unreal trial 2 it took dining-chair legs 0.8-1.3 m away for overhang edges: points 31-87 mm
# high, ranged up to 11 cm short, and the edge cells and the floor mapped behind them sealed the robot into a pocket. Something
# lower than the LiDAR plane is not in its blind band; every point on the Unreal sofa base was 110-155 mm (recorded drives:
# oblique views range it short and low, the rug lowers it ~1 cm). Band-pass points below this veto the overhang label within
# one cell, as floor-pass points do (CellVoter); they still confirm cells as low obstacles (the band pass's rows also find the
# 6 cm shower tray's edge from 0.9 m, which only the camera sees; dropping them cost 1-3 of 3-5 tray cells per recorded trial).
OVERHANG_MIN_H=LIDAR_MOUNT[2]-.03
# An overhang edge also needs one of its confirming views from within this range (task Q): seen obliquely the band pass ranges an
# edge short (sofa: cells up to 25 cm in front of it), the more so the farther. Replays of 4 Unreal trials: every sofa-base
# overhang cell had a view within 0.8 m (0 of 94 lost); the chair-leg cells the old rule took for overhang edges had 14 of 20
# none. A cell seen only from farther is a plain obstacle (no floor mapped under it).
OVERHANG_MAX_RANGE=.8
from .stereo import wrap
_POOL=[]
def _pool():
    # One worker runs the backward flow beside the forward flow (separate DIS objects): same numbers, ~5 ms less per frame.
    if not _POOL:_POOL.append(ThreadPoolExecutor(1,thread_name_prefix='floor-parallax'))
    return _POOL[0]

def _K(meta):return np.array([[float(meta['fx']),0,float(meta['cx'])],[0,float(meta['fy']),float(meta['cy'])],[0,0,1.]])
def _hmap(h,p):
    q=np.c_[p,np.ones(len(p))]@h.T;return q[:,:2]/q[:,2:3]
def plane_homography(pose_r,pose_t,k,height=0.):
    """Image homography reference->current for the horizontal plane z=height (0: the floor), from the robot's own poses."""
    rr,cr=camera(pose_r);rt,ct=camera(pose_t);n=np.array([0,0,1.])
    m=rt.T@(rr-np.outer(cr-ct,n@rr)/(cr[2]-height));h=k@m@np.linalg.inv(k);return h/h[2,2]
def floor_homography(pose_r,pose_t,k):
    """Image homography reference->current for the z=0 floor, from the robot's own poses."""
    return plane_homography(pose_r,pose_t,k)
def _consistent(f,b,valid,ys,xs,shape,g=None):
    """Forward flow at the grid pixels and the mask where the backward flow returns within half a pixel; with g (unit image
    gradients at the pixels) only across the edge, along g: along a clean straight edge the motion along it is undefined
    (aperture), so the full 2D round trip fails there although the normal flow, the only part used, is consistent."""
    fs=f[ys,xs];back=cv2.remap(b,(xs+fs[:,0]).astype(np.float32).reshape(shape),(ys+fs[:,1]).astype(np.float32).reshape(shape),cv2.INTER_LINEAR).reshape(-1,2)
    err=np.hypot(*(fs+back).T) if g is None else np.abs(np.sum((fs+back)*g,1))
    return fs,valid[ys,xs]&(err<.5)
def _gradient(J):
    Jxx,Jxy,Jyy=J;th=.5*np.arctan2(2*Jxy,Jxx-Jyy);return np.c_[np.cos(th),np.sin(th)]
def _tensor(img,ys,xs):
    """Structure tensor (5x5 box of Sobel products) of img at the pixels (ys, xs)."""
    gx=cv2.Sobel(img,cv2.CV_32F,1,0,ksize=3);gy=cv2.Sobel(img,cv2.CV_32F,0,1,ksize=3)
    return cv2.blur(gx*gx,(5,5))[ys,xs],cv2.blur(gx*gy,(5,5))[ys,xs],cv2.blur(gy*gy,(5,5))[ys,xs]

class FloorParallax:
    """One-frame measurement: returns (Nx3 map points [x,y,height], Nx2 pixels) of unconfirmed raised structure."""
    def __init__(self,min_base=.06,max_base=.16,max_age=3.,max_dyaw=.35,hmin=.03,hmax=HMAX,max_range=1.2,grad=6.,hconf=.025,
                 max_sig_r=.05,min_dg=.35,max_edge_cos=.95,vmin=12,step=2,history=32,band=True,band_min_dg=.6,band_max_sig_r=.05,parallel=True):
        self.cfg=dict(min_base=min_base,max_base=max_base,max_age=max_age,max_dyaw=max_dyaw,hmin=hmin,hmax=hmax,max_range=max_range,grad=grad,
                      hconf=hconf,max_sig_r=max_sig_r,min_dg=min_dg,max_edge_cos=max_edge_cos,vmin=vmin,step=step,history=history,band=band,band_min_dg=band_min_dg,band_max_sig_r=band_max_sig_r)
        self.dis=[cv2.DISOpticalFlow_create(cv2.DISOPTICAL_FLOW_PRESET_MEDIUM) for _ in range(2)];self.parallel=parallel;self.hist=[];self.grid=None;self.status={'state':'waiting for images'}
        # The band pass flows on the image rows above the floor pass (its own DIS pair: another image size).
        self.dis_band=[cv2.DISOpticalFlow_create(cv2.DISOPTICAL_FLOW_PRESET_MEDIUM) for _ in range(2)];self.band_grid=None
    def reset(self):self.hist=[];self.status={'state':'history cleared'}
    def _reference(self,pose,stamp):
        c=self.cfg;ct=camera(pose)[1];best=None
        for g,p,s,cp in self.hist:
            b=float(np.linalg.norm(cp-ct))
            if not 0<stamp-s<=c['max_age'] or abs(wrap(pose[2]-p[2]))>c['max_dyaw'] or not c['min_base']<=b<=c['max_base']:continue
            if best is None or b>best[0]:best=(b,g,p,s)
        return best
    def _flow(self,cur,warped,dis):
        if self.parallel:job=_pool().submit(dis[1].calc,warped,cur,None);f=dis[0].calc(cur,warped,None);return f,job.result()
        return dis[0].calc(cur,warped,None),dis[1].calc(warped,cur,None)
    def _measure(self,pt,fs,J,ok,sel,e,hwarp,hfit,P0,P1,R,t,n,d,bl,min_dg=None,scaled=False,max_sig_r=None):
        """Raised structure among the grid pixels pt where sel: aperture-safe parallax against a horizontal plane whose
        reference->current homography from the fitted floor motion is hfit (the reference was pre-warped by hwarp, the same
        plane from the robot's poses), triangulated with the fitted motion (P0 reference, P1 current). Returns (X in the
        reference camera frame, height above the fitted floor, pixels) of the points that pass every gate, or None when fewer
        than 20 pixels qualify."""
        c=self.cfg;Jxx,Jxy,Jyy=J;min_dg=c['min_dg'] if min_dg is None else min_dg;max_sig_r=c['max_sig_r'] if max_sig_r is None else max_sig_r
        g=_gradient(J)
        dv=pt-e;rn=np.linalg.norm(dv,axis=1);dv/=np.maximum(rn,1e-9)[:,None];dg=np.sum(dv*g,1)
        sd=np.sqrt(np.maximum(0,Jxx*dv[:,0]**2+2*Jxy*dv[:,0]*dv[:,1]+Jyy*dv[:,1]**2))/8
        # Near-vertical image edges (gradient within ~18 deg of horizontal) are skipped: their normal flow is horizontal, the
        # direction corrupted by occlusion edges of legs/door frames, wall shadows and yaw error. Low obstacles show their
        # near-horizontal top/bottom edges, overhangs their bottom edge. Recordings: removes every false cell >20 cm from an
        # obstacle, keeps all approaches.
        idx=np.flatnonzero(ok&sel&(np.abs(dg)>min_dg)&(sd>c['grad'])&(rn>20)&(np.abs(g[:,0])<c['max_edge_cos']))
        if len(idx)<20:return None
        P=pt[idx];D=dv[idx];G=g[idx];hinv=np.linalg.inv(hfit)
        # Normal-flow parallax: only the flow component along the image gradient is trusted (aperture problem).
        mu=np.sum(((P+fs[idx])-_hmap(hwarp@hinv,P))*G,1)/np.sum(D*G,1)
        def tri(sel,m):
            X=cv2.triangulatePoints(P0,P1,_hmap(hinv,P[sel]+m[:,None]*D[sel]).T,P[sel].T).T;return X[:,:3]/X[:,3:4]
        with np.errstate(divide='ignore',invalid='ignore'):
            X=tri(np.arange(len(P)),mu);hgt=d-X@n;rng=np.hypot(X[:,0],X[:,2])
            keep=np.isfinite(X).all(1)&(X[:,2]>.15)&((X@R.T+t)[:,2]>.12)&(hgt>c['hmin'])&(hgt<c['hmax'])&(rng<c['max_range']+bl);kk=np.flatnonzero(keep)
            if len(kk):
                w=.7/np.abs(np.sum(D[kk]*G[kk],1)) if scaled else .7;Xa=tri(kk,mu[kk]-w);Xb=tri(kk,mu[kk]+w)
                sh=np.abs((d-Xb@n)-(d-Xa@n))/2;sr=np.abs(np.hypot(Xb[:,0],Xb[:,2])-np.hypot(Xa[:,0],Xa[:,2]))/2
                keep[kk]&=(hgt[kk]-2*sh>c['hconf'])&(sr<max_sig_r)
        return X[keep],hgt[keep],P[keep]
    def _band(self,gray,gr,pr,pose,k,lo,R,t,n,d,e,P0,P1,bl):
        """The band pass: image rows above the floor pass (row lo up), where everything higher than the camera appears
        (overhangs in the LiDAR's blind band, the upper part of low objects taller than the camera). The floor homography
        means nothing there (those rays meet the floor behind the camera), and a horizontal plane at the band's height
        stretches everything off that plane so much that the flow fails (recorded sofa approaches: 0.3-1.2 % of these pixels
        flow-consistent, against 10-16 % with this). So the reference is pre-warped by the rotation alone (the plane at
        infinity) and the parallax is measured against it, triangulated with the same floor-fitted motion."""
        c=self.cfg;H,W=gray.shape;s=c['step'];hc=min(H,lo+8)
        if lo<=0:return None
        rr=camera(pr)[0];rt=camera(pose)[0];kinv=np.linalg.inv(k);h0=k@rt.T@rr@kinv;h0/=h0[2,2]
        warped=cv2.warpPerspective(gr,h0,(W,hc));valid=cv2.warpPerspective(np.full_like(gr,255),h0,(W,hc),flags=cv2.INTER_NEAREST)>0
        cur=np.ascontiguousarray(gray[:hc]);f,b=self._flow(cur,warped,self.dis_band)
        if self.band_grid is None or self.band_grid[0]!=(hc,W,s,lo):
            ys,xs=np.mgrid[0:lo:s,0:W:s];self.band_grid=((hc,W,s,lo),ys.ravel(),xs.ravel(),ys.shape)
        _,ys,xs,shape=self.band_grid;J=_tensor(cur,ys,xs);fs,ok=_consistent(f,b,valid,ys,xs,shape,_gradient(J))
        return self._measure(np.c_[xs,ys].astype(float),fs,J,ok,np.ones(len(ys),bool),e,h0,k@R@kinv,P0,P1,R,t,n,d,bl,c['band_min_dg'],True,c['band_max_sig_r'])
    def _motion(self,gray,ref,pose,k):
        """Steps 2-3 for one reference: floor pre-warp, flow, the measured floor motion. Returns a dict (R, t, n, d, ... and the
        floor-pass samples), or the status string of the check that failed."""
        c=self.cfg;H,W=gray.shape;base,gr,pr,sr=ref
        h0=floor_homography(pr,pose,k);warped=cv2.warpPerspective(gr,h0,(W,H));valid=cv2.warpPerspective(np.full_like(gr,255),h0,(W,H),flags=cv2.INTER_NEAREST)>0
        # Flow on the full image (pyramid context matters); per-pixel work only below row cy+vmin, the only rows the floor pass uses.
        f,b=self._flow(gray,warped,self.dis)
        s=c['step'];lo=int(math.floor(k[1,2]+min(c['vmin'],25)))//s*s;top=max(0,lo-8)
        if self.grid is None or self.grid[0]!=(H,W,s,lo):
            ys,xs=np.mgrid[lo:H:s,0:W:s];self.grid=((H,W,s,lo),ys.ravel(),xs.ravel(),ys.shape)
        _,ys,xs,shape=self.grid;fs,ok=_consistent(f,b,valid,ys,xs,shape);J=_tensor(gray[top:],ys-top,xs);Jxx,Jxy,Jyy=J
        tr=Jxx+Jyy;lmin=tr/2-np.sqrt(np.maximum(0,tr*tr/4-(Jxx*Jyy-Jxy**2)));pt=np.c_[xs,ys].astype(float)
        rr,cr=camera(pr);rt,ct=camera(pose);er=rt.T@rr;et=rt.T@(cr-ct);d=float(np.clip(cr[2],*CAMERA_HEIGHT_RANGE));bl=float(np.linalg.norm(et))
        corner=ok&(lmin>150)&(pt[:,1]>k[1,2]+25)
        if corner.sum()<80:return 'too little floor texture'
        hf,inl=cv2.findHomography(_hmap(np.linalg.inv(h0),pt[corner]+fs[corner]),pt[corner],cv2.RANSAC,1.0,maxIters=500)
        if hf is None or inl.mean()<=.5:return 'floor motion uncertain'
        _,Rs,Ts,Ns=cv2.decomposeHomographyMat(hf,k);cand=[]
        for Ri,Ti,Ni in zip(Rs,Ts,Ns):
            Ti=Ti[:,0]*d;Ni=Ni[:,0];L=np.linalg.norm(Ti);re=np.linalg.norm(cv2.Rodrigues(Ri@er.T)[0])
            if Ni[1]<.9 or re>.04 or not .7*bl<L<1.3*bl or Ti@et/max(1e-9,L*bl)<.9:continue
            cand.append((re+abs(L-bl),Ri,Ti,Ni))
        if not cand:return 'floor motion disagrees with robot motion'
        _,R,t,n=min(cand,key=lambda x:x[0]);e=k@t;e=e[:2]/e[2] if abs(e[2])>1e-9 else np.array([1e6,1e6])
        return dict(R=R,t=t,n=n,d=d,bl=bl,e=e,h0=h0,hf=hf,pt=pt,fs=fs,J=J,ok=ok,lo=lo,rr=rr,cr=cr,gr=gr,pr=pr,P0=k@np.c_[np.eye(3),np.zeros(3)],P1=k@np.c_[R,t])
    def _world(self,part,m):
        if part is None:return None
        X,hgt,P=part;w=X@m['rr'].T+m['cr'];w[:,2]=hgt;return w,P
    def process(self,gray,meta,pose):
        c=self.cfg;k=_K(meta);stamp=float(meta['sim_time_s']);pose=np.asarray(pose,float);empty=(np.empty((0,3)),np.empty((0,2)));self.last_band=np.zeros(0,bool)
        if self.hist and stamp==self.hist[-1][2]:self.status={'state':'repeated image'};return empty
        if self.hist and (self.hist[-1][0].shape!=gray.shape or stamp<self.hist[-1][2]):self.hist=[]
        ref=self._reference(pose,stamp)
        self.hist=[x for x in self.hist if stamp-x[2]<=c['max_age']][-(c['history']-1):]+[(gray,pose.copy(),stamp,camera(pose)[1])]
        if ref is None:self.status={'state':'needs 6 cm of travel'};return empty
        base,_,_,sr=ref;info=dict(baseline_m=round(base,4),reference_age_s=round(stamp-sr,3))
        m=self._motion(gray,ref,pose,k)
        if isinstance(m,str):self.status=dict(state=m,**info);return empty
        g=lambda m:(m['R'],m['t'],m['n'],m['d'],m['e'],m['P0'],m['P1'],m['bl'])
        parts=[self._world(self._measure(m['pt'],m['fs'],m['J'],m['ok'],m['pt'][:,1]>k[1,2]+c['vmin'],m['e'],m['h0'],m['hf'],m['P0'],m['P1'],m['R'],m['t'],m['n'],m['d'],m['bl']),m)]
        if c['band']:parts.append(self._world(self._band(gray,m['gr'],m['pr'],pose,k,m['lo'],*g(m)),m))
        if all(x is None for x in parts):self.status=dict(state='little texture',**info);return empty
        world,P=(np.concatenate(v) for v in zip(*[x if x is not None else (np.empty((0,3)),np.empty((0,2))) for x in parts]))
        band=int(sum(len(x[0]) for x in parts[1:] if x is not None));self.last_band=np.arange(len(world))>=len(world)-band
        self.status=dict(state='floor motion verified',points=int(len(world)),band_points=band,blind_band_points=int((world[:,2]>LIDAR_MOUNT[2]).sum()),**info);return world,P

class CellVoter:
    """Confirm a map cell after >=need frames (camera centres >=min_move apart) each put >=min_points in it. With band (a mask of
    the points from the detector's band pass) it also tells overhang edges apart: a cell confirmed by band-pass points at least
    overhang_min_h high (OVERHANG_MIN_H) only, one of its views from within overhang_max_range (OVERHANG_MAX_RANGE), with no
    lower point ever within one cell of it (a low object taller than the
    camera shows its lower part to the floor pass, a leg or post its lower part to the band pass; an overhang has nothing below
    its edge). Lower band-pass points still confirm a cell, as a low obstacle (task Q: see OVERHANG_MIN_H)."""
    def __init__(self,res=.05,need=2,min_points=2,min_move=.03,max_age=None,overhang_min_h=None,overhang_max_range=None):
        self.res=res;self.need=need;self.min_points=min_points;self.min_move=min_move;self.max_age=max_age;self.hits={};self.confirmed=set()
        self.overhang_min_h=OVERHANG_MIN_H if overhang_min_h is None else overhang_min_h
        self.overhang_max_range=OVERHANG_MAX_RANGE if overhang_max_range is None else overhang_max_range
        self.floor_seen=set();self.floor_votes=set();self.overhangs=set();self.new_overhangs=np.empty((0,2))
    def clear_pending(self):self.hits={}
    def add(self,world,camera_xy,stamp=0.,band=None):
        """Returns (Nx2 centres of newly confirmed cells, mask of the input points that fell in them); new_overhangs: the centres
        of those that are overhang edges."""
        new=[];world=np.asarray(world).reshape(-1,3) if len(world) else np.empty((0,3));self.new_overhangs=np.empty((0,2))
        if not len(world):return np.empty((0,2)),np.zeros(0,bool)
        ij=np.floor(world[:,:2]/self.res).astype(int);u,inv,cnt=np.unique(ij,axis=0,return_inverse=True,return_counts=True);cam=np.asarray(camera_xy,float)[:2]
        if band is not None:
            high=np.asarray(band,bool)&(world[:,2]>=self.overhang_min_h)
            low={tuple(c) for c in ij[~high].tolist()};self.floor_seen|=low
        for cell,m in zip(map(tuple,u.tolist()),cnt):
            if m<self.min_points or cell in self.confirmed:continue
            h=self.hits.setdefault(cell,[])
            if self.max_age is not None:h[:]=[x for x in h if stamp-x[1]<=self.max_age]
            if band is None or cell in low:self.floor_votes.add(cell)
            if not h or np.linalg.norm(h[-1][0]-cam)>self.min_move:h.append((cam,stamp))
            if len(h)>=self.need:
                self.confirmed.add(cell);self.hits.pop(cell,None);new.append(cell)
                near=min(np.linalg.norm(c-(np.array(cell)+.5)*self.res) for c,_ in h)<=self.overhang_max_range
                if near and cell not in self.floor_votes and not any((cell[0]+a,cell[1]+b) in self.floor_seen for a in (-1,0,1) for b in (-1,0,1)):self.overhangs.add(cell)
        fresh=set(new);mask=np.array([cell in fresh for cell in map(tuple,u.tolist())],bool)[inv.ravel()]
        self.new_overhangs=(np.array([c for c in new if c in self.overhangs],float).reshape(-1,2)+.5)*self.res
        return (np.array(new,float).reshape(-1,2)+.5)*self.res,mask
    def cells(self):return (np.array(sorted(self.confirmed),float).reshape(-1,2)+.5)*self.res

class LowObstacleCamera:
    """Controller-facing detector. process() -> (N,2) NEWLY confirmed obstacle points in map metres
    (feed to LidarGrid.add_camera_obstacles); new_overhangs: those that are overhang edges, seen from camera_xy (feed both to
    LidarGrid.add_overhang_shadow). reset_history() forgets images and unconfirmed votes
    (call whenever the pose is not trustworthy) but keeps confirmed cells. status is JSON-safe."""
    def __init__(self,res=.05,vote_max_age=None,**detector):
        self.detector=FloorParallax(**detector);self.voter=CellVoter(res=res,max_age=vote_max_age);self.new_overhangs=np.empty((0,2));self.camera_xy=None;self.last_stamp=None;self.frames=0;self.status={'state':'waiting for images','confirmed_cells':0}
    def reset_history(self):
        self.detector.reset();self.voter.clear_pending()
        self.status=dict(state='waiting for stable, localized view',new_cells=0,confirmed_cells=len(self.voter.confirmed),obstacle_pixels=[])
    def confirmed_points(self):return self.voter.cells()
    def forget(self,points):
        """The map released these cells (the LiDAR cleared what stood there, or the robot stood on them): confirm them afresh
        if the camera sees something there again."""
        v=self.voter
        for p in np.asarray(points,float).reshape(-1,2):
            c=tuple(np.floor(p/v.res).astype(int).tolist());v.confirmed.discard(c);v.overhangs.discard(c);v.hits.pop(c,None)
    def process(self,image,meta,pose):
        t0=time.perf_counter();gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY) if image.ndim==3 else image.copy()  # history keeps it
        pose=np.asarray(pose,float);world,pixels=self.detector.process(gray,meta,pose)
        self.camera_xy=camera(pose)[1][:2].copy();new,mask=self.voter.add(world,self.camera_xy,float(meta['sim_time_s']),band=self.detector.last_band);self.last_stamp=float(meta['sim_time_s']);self.frames+=1
        self.new_overhangs=self.voter.new_overhangs
        self.status=dict(self.detector.status,frame=meta.get('frame'),image_stamp=self.last_stamp,candidate_points=int(len(world)),new_cells=int(len(new)),new_overhang_cells=int(len(self.new_overhangs)),
                         confirmed_cells=len(self.voter.confirmed),pending_cells=len(self.voter.hits),obstacle_pixels=np.round(pixels[mask][:40],1).tolist(),
                         ms=round((time.perf_counter()-t0)*1000,1))
        return new
