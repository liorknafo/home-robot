import sys,tempfile,math,numpy as np,pickle
from house_world import load_world,HouseSim,P
from sim2d import Bridge
import autonomy.lidar_brain as LB
from autonomy.lidar_brain import LidarBrain
from autonomy import local_planner as LP
tmp=tempfile.TemporaryDirectory();b=LidarBrain(tmp.name);sim=HouseSim(load_world(),seed=0)
br=Bridge(b,sim,rearm_hook=lambda brain:getattr(brain,'wants_arm',False))
for _ in range(10):b.predict(sim.state());[sim.step(.01) for _ in range(5)]
br.start_mapping();br.run(float(sys.argv[1]))
orig_fv=b.follow_velocity;orig_cv=LP.choose_velocity
def fv(target,distance,stamp=None):
    p=b.localizer.pose;err=LB.wrap(math.atan2(target[1]-p[1],target[0]-p[0])-p[2])
    r=orig_fv(target,distance,stamp)
    print(f"t={stamp:.2f} target={np.round(target,2)} err={err:.2f} facing={getattr(b,'facing',None)} out={np.round(r,3)} path0={np.round(b.path[0],2)} len={len(b.path)} goal={np.round(b.goal,2)}")
    return r
b.follow_velocity=fv
br.run(3)
