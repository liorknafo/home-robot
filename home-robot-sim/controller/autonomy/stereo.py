"""Rectified stereo triangulation with consistency and depth-uncertainty checks."""
from dataclasses import dataclass
import math,cv2,numpy as np
AXIS=np.array([[0,0,1],[-1,0,0],[0,-1,0]],float)
CAMERA_OFFSET=np.array([.12272,.031,.15])
def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def rotation(yaw):
    c,s=math.cos(yaw),math.sin(yaw);return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def camera_transform(pose):
    r=rotation(pose[2]);return r@AXIS,np.array([pose[0],pose[1],0.])+r@CAMERA_OFFSET

def optical_to_world(points,pose):
    r,t=camera_transform(pose);return points@r.T+t

def world_to_base_pose(rv,tv):
    r=cv2.Rodrigues(rv)[0].T;cp=-r@tv.reshape(3);forward=r[:,2];yaw=math.atan2(forward[1],forward[0]);base=cp-rotation(yaw)@CAMERA_OFFSET
    if abs(base[2])>.14:return None
    return np.array([base[0],base[1],yaw])
@dataclass
class StereoFrame:
    index:int;stamp:float;gray:np.ndarray;rgb:np.ndarray;depth:np.ndarray;valid:np.ndarray;K:np.ndarray;features:dict;points:np.ndarray;depth_sigma:np.ndarray
class Stereo:
    def __init__(self,features):
        self.features=features
        args=dict(numDisparities=128,blockSize=5,P1=200,P2=800,disp12MaxDiff=1,uniquenessRatio=10,speckleWindowSize=80,speckleRange=2,mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        self.left=cv2.StereoSGBM_create(minDisparity=0,**args);self.right=cv2.StereoSGBM_create(minDisparity=-128,**args)
    def process(self,left,right,metadata):
        # Whitelist calibration/timestamp fields. Simulator evaluation fields are never read.
        fx,fy,cx,cy,b=(float(metadata[k]) for k in ['fx','fy','cx','cy','baseline_m'])
        l=cv2.cvtColor(left,cv2.COLOR_BGR2GRAY);r=cv2.cvtColor(right,cv2.COLOR_BGR2GRAY)
        dl=self.left.compute(l,r).astype(float)/16;dr=self.right.compute(r,l).astype(float)/16
        yy,xx=np.indices(l.shape);xr=np.rint(xx-dl).astype(int);inside=(xr>=0)&(xr<l.shape[1]);sample=dr[yy,np.clip(xr,0,l.shape[1]-1)]
        valid=inside&(dl>1)&(dl<127)&(np.abs(dl+sample)<1.25)
        z=np.divide(fx*b,dl,out=np.zeros_like(dl),where=valid)
        # 0.5 pixel is an assumed disparity error bound, not a calibrated uncertainty.
        sigma=np.divide(.5*z*z,fx*b,out=np.zeros_like(z),where=valid)
        valid&=(z>.19)&(z<3.5)&(sigma<.08*z);z[~valid]=0
        f=self.features.extract(l);uv=f['uv'];p=np.zeros((len(uv),3),np.float32)
        for i,(u,v) in enumerate(uv):
            x,y=int(round(u)),int(round(v))
            if not(1<=x<l.shape[1]-1 and 1<=y<l.shape[0]-1):continue
            patch=z[y-1:y+2,x-1:x+2];vals=patch[patch>0]
            if len(vals)<5:continue
            depth=float(np.median(vals))
            if np.std(vals)>.07*depth:continue
            p[i]=[(u-cx)*depth/fx,(v-cy)*depth/fy,depth]
        return StereoFrame(int(metadata['frame']),float(metadata['sim_time_s']),l,left,z,valid,np.array([[fx,0,cx],[0,fy,cy],[0,0,1]],float),f,p,sigma)
    @staticmethod
    def cloud(frame,stride=5):
        yy,xx=np.mgrid[0:frame.gray.shape[0]:stride,0:frame.gray.shape[1]:stride];keep=frame.valid[yy,xx];x,y=xx[keep],yy[keep];z=frame.depth[y,x];k=frame.K
        return np.column_stack(((x-k[0,2])*z/k[0,0],(y-k[1,2])*z/k[1,1],z))
