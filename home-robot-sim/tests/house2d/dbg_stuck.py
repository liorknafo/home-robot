import sys,tempfile,numpy as np
from house_world import load_world,HouseSim,P
from sim2d import Bridge
from autonomy.lidar_brain import LidarBrain
tmp=tempfile.TemporaryDirectory();b=LidarBrain(tmp.name);sim=HouseSim(load_world(),seed=0)
br=Bridge(b,sim,rearm_hook=lambda brain:getattr(brain,'wants_arm',False))
for _ in range(10):b.predict(sim.state());[sim.step(.01) for _ in range(5)]
br.start_mapping();br.run(float(sys.argv[1]))
n0=len(br.trace);br.run(4)
for r in br.trace[n0::4]:print(round(r['t'],2),r['phase'],round(r['v'],3),round(r['w'],3),r['armed'],r['fault'],np.round(r['truth'],3).tolist(),np.round(r['est'],3).tolist())
print('body_blocked',sim.body_blocked(sim.pose),'bumper',sim.bumper_azimuths(sim.pose))
print('msg',b.message)
