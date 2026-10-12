"""Summarize run_house.py results: summarize.py DIR [GROUP_REGEX] -> a markdown table per run and per group (stdout),
plus DIR/summary.json. GROUP_REGEX (default '^(.*)_s\\d+$') takes the group name from each label.
The disconnected safe floor (safe floor the planner cannot reach; mapping_complete() allows at most 1.5 m2 of the part the
body could enter, task H) is located from each run's saved grid (map-LABEL) at the final true pose (extra.pkl; evaluation only).
Run it with the same CTRL as the runs (the grid classes come from the controller)."""
import json,os,pickle,re,sys
from pathlib import Path
import numpy as np,cv2
HERE=Path(__file__).resolve().parent;SIM=HERE.parents[1]
sys.path.insert(0,os.environ.get('CTRL',str(SIM/'controller')));sys.path.insert(0,str(SIM/'tests'))
from autonomy.lidar import LidarGrid
import score_full_mapping as S
D=Path(sys.argv[1]);pat=re.compile(sys.argv[2] if len(sys.argv)>2 else r'^(.*)_s\d+$')
def room_of(xy):
    for name,(x0,x1,y0,y1) in S.ROOMS:
        if x0<=xy[0]<x1 and y0<=xy[1]<y1:return name
    return 'outside rooms'
def pockets(label):
    m=D/f'map-{label}'
    if not (m/'occupancy.npz').exists():return None
    g=LidarGrid();g.load(m);ex=pickle.load(open(m/'extra.pkl','rb'));pose=ex['truth'][-1]
    safe=g.masks()[3];rch=g.reachable(pose[:2],safe);d=(safe&~rch).astype(np.uint8)
    n,lab,st,cen=cv2.connectedComponentsWithStats(d,connectivity=8);out={}
    for i in range(1,n):
        r=room_of(g.world((int(round(cen[i][1])),int(round(cen[i][0])))));out[r]=round(out.get(r,0.)+float(st[i,4]*g.res**2),3)
    return out
rows=[]
for p in sorted(D.glob('*.json')):
    if p.name=='summary.json':continue
    r=json.loads(p.read_text());r['disconnected_by_room']=pockets(r['label']);rows.append(r)
ROOMS=[n for n,_ in S.ROOMS]
print('| run | complete | mapping s | rooms (L/K/B/S/Ba) | frontier s | coverage s | none s | 1st coverage s | unvisited m2 | disconnected safe m2 | of it enterable m2 | covered on the way (> 1.2 m) | frontiers left | contacts | body blocks | skips |')
print('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
for r in rows:
    b=r['blockers'];st=r['stage_s']
    print(f"| {r['label']} | {'yes' if r['complete'] else 'no'} | {r['mapping_s']:.0f} | {' / '.join(f'{r['rooms'][n][1]:.3f}' for n in ROOMS)} | {st['frontier']:.0f} | {st['coverage']:.0f} | {st['none']:.0f} | "
          f"{r['first_coverage_s'] if r['first_coverage_s'] is not None else '-'} | {b['unvisited_m2']:.2f} | {b['disconnected_safe_m2']:.2f} | {'-' if b.get('unreached_enterable_m2') is None else format(b['unreached_enterable_m2'],'.2f')} | "
          f"{'-' if r.get('covered_on_the_way_m') is None else f"{len(r['covered_on_the_way_m'])} ({sum(d>1.2 for d in r['covered_on_the_way_m'])})"} | {b['frontiers_left']} | {r['contacts']} | {r['body_blocks']} | {r['skips']} |")
groups={}
for r in rows:
    m=pat.match(r['label']);groups.setdefault(m.group(1) if m else r['label'],[]).append(r)
print('\n| group | n | complete | mapping s median (min-max) | frontier s median | coverage s median | 1st coverage s median | unvisited m2 median | disconnected m2 median | frontiers left (sum) | min room fraction | contacts (sum) | body blocks (sum) |')
print('|---|---|---|---|---|---|---|---|---|---|---|---|---|')
summary={}
for k,rs in groups.items():
    med=lambda f:float(np.median([f(r) for r in rs]));fc=[r['first_coverage_s'] for r in rs if r['first_coverage_s'] is not None]
    s=dict(n=len(rs),complete=sum(bool(r['complete']) for r in rs),mapping_s_median=med(lambda r:r['mapping_s']),mapping_s_min=min(r['mapping_s'] for r in rs),mapping_s_max=max(r['mapping_s'] for r in rs),
           frontier_s_median=med(lambda r:r['stage_s']['frontier']),coverage_s_median=med(lambda r:r['stage_s']['coverage']),first_coverage_s_median=float(np.median(fc)) if fc else None,
           unvisited_m2_median=med(lambda r:r['blockers']['unvisited_m2']),disconnected_m2_median=med(lambda r:r['blockers']['disconnected_safe_m2']),frontiers_left_sum=sum(r['blockers']['frontiers_left'] for r in rs),
           min_room_fraction=min(min(v[1] for v in r['rooms'].values()) for r in rs),contacts_sum=sum(r['contacts'] for r in rs),body_blocks_sum=sum(r['body_blocks'] for r in rs))
    summary[k]=s
    print(f"| {k} | {s['n']} | {s['complete']}/{s['n']} | {s['mapping_s_median']:.0f} ({s['mapping_s_min']:.0f}-{s['mapping_s_max']:.0f}) | {s['frontier_s_median']:.0f} | {s['coverage_s_median']:.0f} | "
          f"{s['first_coverage_s_median'] if s['first_coverage_s_median'] is not None else '-'} | {s['unvisited_m2_median']:.2f} | {s['disconnected_m2_median']:.2f} | {s['frontiers_left_sum']} | {s['min_room_fraction']:.3f} | {s['contacts_sum']} | {s['body_blocks_sum']} |")
(D/'summary.json').write_text(json.dumps(dict(groups=summary,runs=[{k:v for k,v in r.items() if k not in ('unvisited_timeline','kinds')} for r in rows]),indent=1))
