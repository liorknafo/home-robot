"""Camera mount sweep (task F): how far ahead do the two camera detectors first mark a low object, per camera height?

Evaluation only, no Unreal. The scene and the renderer are test_floor_parallax's synthetic ones (a textured floor and a
textured axis-aligned box, ray cast from mono.camera()), so every mount is rendered and detected with the same code path
as the robot's own camera tests. For each mount (x, 0, z) the robot drives straight at the object at 5 Hz and the script
records, for FloorParallax (LowObstacleCamera) and for MonoObserver:
  first_m   the robot base's distance to the object's near face when the first cell / point on the object is reported,
  cells     how many reports landed on the object, false = how many landed elsewhere (bare floor).
The level camera is the only pose the simulator and the controller support (no mount pitch anywhere).

Usage: .venv/bin/python tests/camera_height_sweep.py [--json OUT] [--heights 0.05,0.06,...] [--x 0.146]
"""
import sys,json,argparse
from pathlib import Path
import numpy as np,cv2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'controller'));sys.path.insert(0,str(Path(__file__).resolve().parent))
from autonomy.floor_parallax import LowObstacleCamera
from autonomy.mono import MonoObserver
from recorded_camera import recorded_mount
from test_floor_parallax import render,FX,CX,CY

OBJECTS={'slipper 27x11x6 cm':((0,-.055,0),(.27,.055,.06)),'toy block 10x10x6 cm':((0,0,0),(.10,.10,.06)),'low block 20x10x4 cm':((0,-.05,0),(.20,.05,.04))}

def run(mount,obj,start=1.6,step=.02,frames=70):
    lo,hi=OBJECTS[obj];box=((start+lo[0],lo[1],lo[2]),(start+hi[0],hi[1],hi[2]))
    on=lambda q,m=.06:box[0][0]-m<=q[0]<=box[1][0]+m and box[0][1]-m<=q[1]<=box[1][1]+m
    fp,mo=LowObstacleCamera(),MonoObserver();res={'floor_parallax':[],'mono':[]}
    with recorded_mount(mount):
        for i in range(frames):
            pose=np.array([step*i,0.,0.]);gap=box[0][0]-pose[0]-mount[0]   # lens face to the object's near face
            if gap<.05:break
            gray=render(pose,box);meta=dict(frame=i,fx=FX,fy=FX,cx=CX,cy=CY,sim_time_s=.2*i)
            for q in fp.process(gray,meta,pose):res['floor_parallax'].append((box[0][0]-pose[0],bool(on(q))))
            for q in mo.process(cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR),meta,pose):res['mono'].append((box[0][0]-pose[0],bool(on(q[:2]))))
    out={}
    for k,v in res.items():
        hits=[d for d,ok in v if ok]
        out[k]=dict(first_m=round(max(hits),3) if hits else None,cells=len(hits),false=sum(1 for _,ok in v if not ok))
    return out

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--json');ap.add_argument('--heights',default='0.045,0.055,0.065,0.075,0.085,0.1075')
    ap.add_argument('--x',type=float,default=.146);ap.add_argument('--objects',default=','.join(OBJECTS));a=ap.parse_args()
    rows=[]
    for z in [float(v) for v in a.heights.split(',')]:
        for obj in a.objects.split(','):
            r=run((a.x,0.,z),obj);rows.append(dict(mount_m=[a.x,0,z],object=obj,**r))
            print(f"z {z:.4f}  {obj:22s}  parallax first {r['floor_parallax']['first_m']} m ({r['floor_parallax']['cells']} cells, {r['floor_parallax']['false']} false)"
                  f"   mono first {r['mono']['first_m']} m ({r['mono']['cells']} pts, {r['mono']['false']} false)",flush=True)
    if a.json:Path(a.json).write_text(json.dumps(dict(note=__doc__.split('\n')[0],rows=rows),indent=1))

if __name__=='__main__':main()
