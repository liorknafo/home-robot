"""Make a robot-parameters variant tree for the 2D harness without touching the sim's own files.
Usage: make_variant.py OUTDIR '{"lidar_mount_m":[0.06,0,0.1275],"lidar_turret_xy_m":[0.06,0]}'
Copies controller/*.py (recursive), tests/sim2d.py, tests/score_full_mapping.py and Content/RobotData/{robot-parameters,
sensor-errors,house-scene}.json into OUTDIR (same relative layout, so geometry.py, sim2d.py and sim_sensor_model.py find
their files there), applies the overrides to the copied robot-parameters.json, and writes OUTDIR/variant.json (overrides
and sha256 of every copied file vs the sim's own). Run the harness against it with CTRL=OUTDIR/controller TESTS=OUTDIR/tests.
A lidar_mount_m override also updates the derived lidar_mount_from_body_origin_m (x, y; z minus base_link_height_m)."""
import hashlib,json,shutil,sys
from pathlib import Path
SIM=Path(__file__).resolve().parents[2]
out=Path(sys.argv[1]).resolve();over=json.loads(sys.argv[2]) if len(sys.argv)>2 else {}
files=[p for p in (SIM/'controller').rglob('*.py') if '__pycache__' not in p.parts]+[SIM/'tests/sim2d.py',SIM/'tests/score_full_mapping.py']+\
      [SIM/'Content/RobotData'/n for n in ('robot-parameters.json','sensor-errors.json','house-scene.json')]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
record={}
for p in files:
    rel=p.relative_to(SIM);dst=out/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dst);record[str(rel)]=dict(source_sha256=sha(p))
prm_path=out/'Content/RobotData/robot-parameters.json';prm=json.loads(prm_path.read_text())
for k,v in over.items():
    if k not in prm:raise SystemExit(f'unknown parameter {k}')
    prm[k]=v
if 'lidar_mount_m' in over and 'lidar_mount_from_body_origin_m' in prm:
    m=over['lidar_mount_m'];prm['lidar_mount_from_body_origin_m']=[m[0],m[1],round(m[2]-prm['base_link_height_m'],6)]
prm_path.write_text(json.dumps(prm,indent=2))
for rel in record:record[rel]['copy_sha256']=sha(out/rel);record[rel]['identical']=record[rel]['copy_sha256']==record[rel]['source_sha256']
(out/'variant.json').write_text(json.dumps(dict(sim=str(SIM),overrides=over,files=record),indent=1))
changed=[r for r,v in record.items() if not v['identical']]
print(json.dumps(dict(variant=str(out),overrides=over,files=len(record),changed=changed)))
