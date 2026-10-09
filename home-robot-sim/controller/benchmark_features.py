"""Benchmark matching on saved stereo frames on the current host, not the Jetson."""
import sys,time,json,argparse,platform
from pathlib import Path
import cv2,numpy as np
from autonomy.features import Features
from autonomy.stereo import Stereo
p=argparse.ArgumentParser();p.add_argument('--backend',default='orb');a=p.parse_args();root=Path(__file__).resolve().parents[1]
folder=root/'Saved/Stereo/run_20261003T122733_24876962811593';extractor=Features(a.backend)
if hasattr(extractor,'torch'):extractor.torch.set_num_threads(4)
processor=Stereo(extractor);records=[];previous=None
for i in [82,83,84,85,86]:
    meta=json.loads((folder/f'frame_{i:06}.json').read_text());left=cv2.imread(str(folder/meta['left']));right=cv2.imread(str(folder/meta['right']));start=time.perf_counter();frame=processor.process(left,right,meta)
    pairs=extractor.match(previous.features,frame.features) if previous else np.empty((0,2),int)
    records.append(dict(frame=i,total_ms=(time.perf_counter()-start)*1000,features=len(frame.features['uv']),triangulated_features=int((frame.points[:,2]>0).sum()),temporal_matches=len(pairs),valid_stereo_fraction=float(frame.valid.mean())));previous=frame
result=dict(backend=a.backend,host=platform.machine(),device=getattr(extractor,'device','CPU'),is_jetson_benchmark=False,records=records)
(root/f'reports/feature-benchmark-{a.backend}.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
