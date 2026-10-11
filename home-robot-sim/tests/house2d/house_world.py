"""Whole-house 2D world for tests/sim2d.py, sliced from the Blender house meshes.
HOUSE_SLICES (env): the slices file (default house_slices_lp1475.npz, the low-profile R1: LiDAR plane .1475 m, robot top
.15515 m; house_slices.npz is the pre-low-profile LiDAR plane .19 m, see slice_house.py). Its optional 'overhang' slice (the blind band between the LiDAR plane
and the robot top) becomes sim2d overhangs: what lies there and not in the LiDAR or low slices touches only the turret.
The emulated camera (camera_view) sits at geometry.CAMERA_MOUNT."""
import sys,math,numpy as np,cv2
from types import SimpleNamespace
import os
from pathlib import Path
P=str(Path(__file__).resolve().parents[2])   # the sim root (this file lives in <sim>/tests/house2d)
sys.path.insert(0,os.environ.get('CTRL',P+'/controller'));sys.path.insert(0,os.environ.get('TESTS',P+'/tests'))
from sim2d import World,Sim,seg_point_distance
HERE=__file__.rsplit('/',1)[0]
SLICES=os.environ.get('HOUSE_SLICES',HERE+'/house_slices_lp1475.npz')
if not SLICES.startswith('/'):SLICES=HERE+'/'+SLICES
def load_world():
    s=np.load(SLICES);walls=s['lidar'];low=s['low']
    res=.01;x0,y0=-4.2,-3.3;W,H=int(8.4/res),int(6.6/res)
    def raster(segs,thick=1):
        m=np.zeros((H,W),np.uint8)
        for a,b in segs:
            pa=(int((a[0]-x0)/res),int((a[1]-y0)/res));pb=(int((b[0]-x0)/res),int((b[1]-y0)/res));cv2.line(m,pa,pb,1,thick)
        return m
    tall=cv2.dilate(raster(walls),np.ones((5,5),np.uint8))
    lowm=raster(low)&~tall.astype(bool)
    contours,_=cv2.findContours(lowm.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    polys=[]
    for c in contours:
        if cv2.contourArea(c)<2 and len(c)<3:continue
        hull=cv2.convexHull(c)[:,0,:].astype(float)
        if len(hull)<3:continue
        polys.append(np.c_[hull[:,0]*res+x0,hull[:,1]*res+y0])
    overhangs=[];oz=None
    if 'overhang' in s.files and len(s['overhang']):
        # Blind-band geometry not already explained by the LiDAR plane (tall) or the low slices (the body meets it lower down).
        ov=raster(s['overhang'])&~tall.astype(bool)&~cv2.dilate(raster(low),np.ones((5,5),np.uint8)).astype(bool)
        contours,_=cv2.findContours(ov.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            hull=cv2.convexHull(c)[:,0,:].astype(float)
            if len(hull)>=3:overhangs.append(np.c_[hull[:,0]*res+x0,hull[:,1]*res+y0])
        oz=float(s['heights'].mean())
    return World([tuple(map(tuple,w)) for w in walls],low=polys,**(dict(overhangs=overhangs,overhang_z=oz) if overhangs else {}))
class HouseSim(Sim):
    """Contact checks only consider geometry near the robot (speed)."""
    def __init__(self,world,**kw):
        super().__init__(world,**kw);self.local=None;self.local_t=-1;self.centres=[(p.mean(0),np.linalg.norm(p-p.mean(0),axis=1).max()) for p in world.low]
        self.ov_centres=[(p.mean(0),np.linalg.norm(p-p.mean(0),axis=1).max()) for p in getattr(world,'overhangs',[])]
    def refresh(self):
        if self.local is not None and self.t-self.local_t<.1:return
        self.local_t=self.t;p=self.pose[:2];W=self.w
        near=seg_point_distance(W.walls,p)<.6
        self.local=SimpleNamespace(walls=W.walls[near],low=[poly for poly,(c,r) in zip(W.low,self.centres) if np.linalg.norm(c-p)<r+.6],curbs=[],cliffs=[],sticky=[],
                                   overhangs=[poly for poly,(c,r) in zip(getattr(W,'overhangs',[]),self.ov_centres) if np.linalg.norm(c-p)<r+.6])
    def _swap(self,fn,*a):
        self.refresh();w=self.w;self.w=self.local
        try:return fn(*a)
        finally:self.w=w
    def bumper_azimuths(self,pose):return self._swap(super().bumper_azimuths,pose)
    def body_blocked(self,pose):return self._swap(super().body_blocked,pose)
    def overhang_blocked(self,pose):return self._swap(super().overhang_blocked,pose)
from sim2d import _seg_hits
def low_samples(world,step=.02):
    pts=[]
    for poly in world.low:
        for i in range(len(poly)):
            a,b=poly[i],poly[(i+1)%len(poly)];n=max(1,int(np.linalg.norm(b-a)/step))
            pts+= [a+(b-a)*k/n for k in range(n)]
    return np.array(pts).reshape(-1,2)
from autonomy.geometry import CAMERA_MOUNT
# Nearest floor the 640 x 360, 84 deg HFOV camera sees: its height over tan(VFOV/2) (.21 m at the .1075 m mount, .30 at .15).
NEAR=max(.15,CAMERA_MOUNT[2]/(math.tan(math.radians(42))*360/640))
def camera_view(sim,samples,fov=math.radians(84),rng=1.2,near=NEAR):
    """Emulated floor-parallax detections: low-object outline points in the camera's
    view (camera at geometry.CAMERA_MOUNT; floor visible from near to rng metres), not hidden by tall geometry.
    Returns points in the robot base frame."""
    p=sim.pose;c,s=math.cos(p[2]),math.sin(p[2]);cam=p[:2]+CAMERA_MOUNT[0]*np.array([c,s])
    d=samples-cam;r=np.linalg.norm(d,axis=1);ang=np.arctan2(d[:,1],d[:,0])-p[2];ang=(ang+math.pi)%(2*math.pi)-math.pi
    keep=(r<rng)&(r>near)&(np.abs(ang)<fov/2)
    if not keep.any():return np.zeros((0,2))
    q=samples[keep];dirs=(q-cam)/r[keep,None];hit=_seg_hits(cam,dirs,sim.local_walls_cam(cam,rng))
    q=q[hit>r[keep]-.02]
    return (q-p[:2])@np.array([[c,-s],[s,c]])
def _local_walls_cam(self,cam,rng):
    near=seg_point_distance(self.w.walls,cam)<rng+.05
    return self.w.walls[near]
HouseSim.local_walls_cam=_local_walls_cam
