"""Cut the dining-chair camera fixture (task Q, LIO-137) from task N's Unreal mapping trial 2 (full-mapping-20261012T221959,
GI on, LED on, errors on). Data preparation only.

Usage: cut_chair_fixture.py STEREO_DIR MAP_DIR REPORT_DIR OUT_DIR
  STEREO_DIR  the trial's camera recording (Saved/Stereo/run_20261012T191953_39931529059935)
  MAP_DIR     the trial's map folder (Saved/Maps/map_20261012T221959: camera-inputs.jsonl, what the controller fed the detector)
  REPORT_DIR  the trial's report (trace.jsonl: its first row's sim time is trial time 0)
In that trial the band pass confirmed dining-chair legs 0.8-1.3 m away as overhang edges; the floor mapped behind them sealed the
robot into a pocket by the dining table (task Q report, section 1). Two clips (trial time, s) end at those confirmations of the chair south of the table (its south-east leg), seen from the
bathroom door (robot heading north):
  chair_south_350  343.5-351.0
  chair_south_937  930.0-937.5
(The live confirmation at the chair east of the table, 865 s, does not repeat offline; the cause was not found: the frames the
controller logged around it are contiguous. test_floor_parallax feeds the voter that leg's measured points instead.)
Each row is what the controller gave the detector: the grey image, its meta, the robot's OWN estimated pose
(LidarBrain.camera_pose) and geometry_ready, as in tests/fixtures/floor_parallax."""
import sys, json
from pathlib import Path
import cv2
CLIPS = {'chair_south_350': (343.5, 351.0), 'chair_south_937': (930.0, 937.5)}
stereo, mapdir, report, out = (Path(a) for a in sys.argv[1:5])
out.mkdir(parents=True, exist_ok=True)
t0 = json.loads((report / 'trace.jsonl').open().readline())['status']['sim_time_s']
frames = {}
for p in stereo.glob('frame_*.json'):
    d = json.loads(p.read_text()); frames[round(d['sim_time_s'], 3)] = d
rows = [json.loads(l) for l in (mapdir / 'camera-inputs.jsonl').open()]
seqs = {}
for name, (a, b) in CLIPS.items():
    seq = []
    for r in rows:
        t = r['meta']['sim_time_s'] - t0; d = frames.get(round(r['meta']['sim_time_s'], 3))
        if not a <= t <= b or d is None: continue
        img = f"{name}_{d['frame']:06d}.png"
        cv2.imwrite(str(out / img), cv2.imread(str(stereo / d['left']), cv2.IMREAD_GRAYSCALE), [cv2.IMWRITE_PNG_COMPRESSION, 9])
        meta = {k: r['meta'][k] for k in ('frame', 'sim_time_s', 'fx', 'fy', 'cx', 'cy')}
        seq.append(dict(image=img, meta=meta, estimated_pose=r['estimated_pose'], geometry_ready=r['geometry_ready'], trial_time_s=round(t, 2)))
    seqs[name] = seq
(out / 'inputs.json').write_text(json.dumps(dict(
    camera_mount_m=[.146, 0., .08], robot='20 mm lower LD14 (task L): LiDAR plane 127.5 mm, robot top 135.15 mm; camera LED (task N) on',
    source='task N mapping trial 2, full-mapping-20261012T221959 (GI on, errors on); see cut_chair_fixture.py', sequences=seqs), indent=1))
print({k: len(v) for k, v in seqs.items()})
