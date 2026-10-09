from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parents[1]/'reports/vision';d=json.loads((R/'vision-results.json').read_text());rows=[x for x in d['records'] if x['tracked']];p=np.array([x['position_m'] for x in rows]);g=np.array([x['evaluation_only_position_m'] for x in rows]);fig,ax=plt.subplots(figsize=(7.1,4.0),layout='constrained');ax.plot(g[:,0],g[:,1],color='#8b9d95',lw=3,label='Unreal position (evaluation only)');ax.plot(p[:,0],p[:,1],color='#1e776d',lw=1.8,label='Estimated from stereo images');ax.scatter([0],[0],color='#bc8331',zorder=5,label='Start');ax.set(xlabel='Forward X (metres)',ylabel='Left Y (metres)',title='Camera motion during the test route');ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.18);ax.legend(loc='best',fontsize=8);fig.savefig(R/'trajectory.png',dpi=150);plt.close(fig)
text=(R/'stereo-map.ply').read_text();lines=text.split('end_header\n',1)[1].splitlines();pts=np.array([[float(x) for x in line.split()[:3]] for line in lines]);rgb=np.array([[int(x)/255 for x in line.split()[3:]] for line in lines]);sel=(pts[:,2]>-.10)&(pts[:,2]<2)&(np.abs(pts[:,0])<6)&(np.abs(pts[:,1])<6)
fig,ax=plt.subplots(figsize=(8,6),layout='constrained');ax.scatter(pts[sel,0],pts[sel,1],c=rgb[sel],s=1,alpha=.5);ax.plot(p[:,0],p[:,1],color='#bc4935',lw=2);ax.set(xlabel='Forward X (metres)',ylabel='Left Y (metres)',title='Stereo map: points above the floor');ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.18);fig.savefig(R/'map-top-view.png',dpi=160);plt.close(fig);print('Saved trajectory and map plots',len(pts),'points')
