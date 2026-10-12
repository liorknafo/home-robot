"""Slice the Blender house meshes into 2D segments (robot map frame: x fwd, y left = -Unreal Y).
Usage: slice_house.py OUT.npz [lidar_z [robot_top_z]]
  lidar: the LiDAR scan plane (default .19, the pre-low-profile robot). low: slices at .03/.06/.10/.14/.17 m that lie
  below the LiDAR plane (bumper/body-only objects). overhang (only when robot_top_z is given): a slice midway between
  the LiDAR plane and the robot top, the blind band where only the LiDAR turret can touch (house_world subtracts
  everything the LiDAR or the low slices already cover). The R1 since task L: slice_house.py house_slices_lp1275.npz .1275 .13515
  (before task L: house_slices_lp1475.npz .1475 .15515)"""
import json,sys,numpy as np
from pathlib import Path
P=str(Path(__file__).resolve().parents[2])   # the sim root (this file lives in <sim>/tests/house2d)
d=json.load(open(P+'/Content/RobotData/house-meshes.json'))
scene=json.load(open(P+'/Content/RobotData/house-scene.json'))
import math
def placed():
    # Unreal: actor at centre (cm) with yaw (deg, about +Z, left-handed frame).
    for o in scene['objects']:
        if not o['collision'] or o['group'] in ('floor','ceiling','rug','threshold'):continue
        a=d['assets'][o['mesh']];c=np.array(o['center'],float);yaw=math.radians(o.get('yaw',0))
        R=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
        for p in a['parts']:
            v=(np.array(p['vertices'],float).reshape(-1,3)@R.T+c)/100.;t=np.array(p['triangles'],int).reshape(-1,3)
            yield v,t
PLACED=list(placed())
def slice_at(z0,collision_only=True):
    segs=[]
    if True:
        for v,t in PLACED:
            if v[:,2].min()>z0 or v[:,2].max()<z0:continue
            dz=v[:,2]-z0;tri=dz[t];s=np.sign(tri)
            mixed=~((s>0).all(1)|(s<0).all(1))
            for tr in t[mixed]:
                pts=[]
                for i,j in ((0,1),(1,2),(2,0)):
                    a1,b1=v[tr[i]],v[tr[j]];da,db=a1[2]-z0,b1[2]-z0
                    if (da<0)!=(db<0) and da!=db:
                        f=da/(da-db);q=a1+f*(b1-a1);pts.append((q[0],-q[1]))
                if len(pts)==2 and np.hypot(pts[0][0]-pts[1][0],pts[0][1]-pts[1][1])>1e-4:segs.append(pts)
    return np.array(segs,float).reshape(-1,2,2)
LIDAR_Z=float(sys.argv[2]) if len(sys.argv)>2 else .19
TOP_Z=float(sys.argv[3]) if len(sys.argv)>3 else None
out={}
out['lidar']=slice_at(LIDAR_Z)
low=[slice_at(z) for z in (.03,.06,.10,.14,.17) if z<LIDAR_Z]
out['low']=np.concatenate(low) if low else np.zeros((0,2,2))
if TOP_Z is not None:
    assert LIDAR_Z<TOP_Z,'the robot top must lie above the LiDAR plane'
    out['overhang']=slice_at((LIDAR_Z+TOP_Z)/2);out['heights']=np.array([LIDAR_Z,TOP_Z])
np.savez_compressed(sys.argv[1],**out)
print({k:v.shape for k,v in out.items()})
