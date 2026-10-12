"""Cut the five camera image fixtures from an Unreal mapping run, at the robot's CURRENT camera mount (task F; generalises the
bay B10 scratch tool fixtool.py). Evaluation and data preparation only: nothing here runs on the robot.

Usage: cut_camera_fixtures.py MAP_FOLDER STEREO_FOLDER [--benchmark REPORT_DIR] [--write OUT_DIR]
  MAP_FOLDER     Saved/Maps/map_<stamp> (camera-inputs.jsonl: the brain's own per-image record)
  STEREO_FOLDER  Saved/Stereo/run_<stamp> (left_<frame>.png, BGR, 640 x 360)
Without --write it only reports the episodes it finds and how the tests' assertions fare on them; with --write it writes
OUT_DIR/<set>/inputs.json + PNGs in the same formats as tests/fixtures (then copy the sets that pass over the old ones).

Truth is never a detector input: episodes are chosen from the robot's own estimated poses; the scene positions below (slipper,
shower tray, threshold, the empty bedroom patch) are the tests' offline annotations, used only to pick episodes and to assert.
Episode rules (the same as B10):
  slipper-approach / floor-rejection  every frame from the first through the closest estimated approach to the slipper
  slipper-side-approach               a window that starts at a tracker reseed (geometry_ready False -> True) >= 0.7 m from the
                                      slipper and comes within 0.8 m of it
  bedroom-edges                       the prefix up to the first frame in the bedroom + 60 frames (grey PNGs)
  floor_parallax bathroom             ready frames in the bathroom heading south toward the 6 cm tray, the longest such run before
                                      the first bumper contact there (grey); threshold: every 3rd frame within 0.6 m of x = 1.0,
                                      |y| < 0.4 (grey)
"""
import sys, json, math, argparse, shutil
from pathlib import Path
import numpy as np, cv2
SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM / 'controller')); sys.path.insert(0, str(SIM / 'tests'))
from autonomy.mono import MonoObserver
from autonomy.floor_parallax import LowObstacleCamera
from autonomy.geometry import CAMERA_MOUNT

SLIPPER = np.array([-1.29, -.18])
TRAY = ((3.25, 4.99), (-3.00, -2.04)); BATH_FREE = ((3.84, 4.28), (-1.98, -1.10)); THRESHOLD_FREE = ((.40, 1.60), (-.28, .28))
def inside(p, box): return box[0][0] <= p[0] <= box[0][1] and box[1][0] <= p[1] <= box[1][1]


def load(mapdir, stereo):
    rows = [json.loads(l) for l in (mapdir / 'camera-inputs.jsonl').read_text().splitlines() if l.strip()]
    return [r for r in rows if (stereo / f"left_{r['meta']['frame']:06}.png").exists()]


def img(stereo, r, gray): return cv2.imread(str(stereo / f"left_{r['meta']['frame']:06}.png"), cv2.IMREAD_GRAYSCALE if gray else cv2.IMREAD_COLOR)


def mono_hits(stereo, sel, gray=False, reset_on_unready=False):
    obs = MonoObserver(); hits = []; pts = []
    for r in sel:
        if reset_on_unready and not r.get('geometry_ready', True): obs = MonoObserver(); continue
        p = obs.process(img(stereo, r, gray), r['meta'], np.array(r['estimated_pose']))
        if len(p):
            pts.extend(p.tolist())
            if np.any(np.linalg.norm(p[:, :2] - SLIPPER, axis=1) < .19): hits.append(float(np.linalg.norm(np.array(r['estimated_pose'][:2]) - SLIPPER)))
    return hits, np.asarray(pts).reshape(-1, 3)


def parallax(stereo, sel):
    cam = LowObstacleCamera(); out = []
    for r in sel:
        if not r.get('geometry_ready', True): cam.reset_history(); continue
        new = cam.process(img(stereo, r, True), r['meta'], np.array(r['estimated_pose']))
        out += [(r['meta']['frame'], p, float(np.linalg.norm(p - np.array(r['estimated_pose'][:2])))) for p in new]
    return out


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('map'); ap.add_argument('stereo'); ap.add_argument('--benchmark', default='')
    ap.add_argument('--write'); ap.add_argument('--contact-frame', type=int, help='first bumper contact at the shower tray (from the run log)')
    a = ap.parse_args(); mapdir, stereo = Path(a.map), Path(a.stereo)
    rows = load(mapdir, stereo); pose = np.array([r['estimated_pose'][:3] for r in rows])
    print('mount', CAMERA_MOUNT, 'rows with images', len(rows), 'frames', rows[0]['meta']['frame'], '..', rows[-1]['meta']['frame'])
    rep = {}; sets = {}
    dsl = np.linalg.norm(pose[:, :2] - SLIPPER, axis=1); end = int(dsl.argmin()) + 1
    hits, pts = mono_hits(stereo, rows[:end])
    rep['slipper-approach'] = dict(rows=[0, end], frames=[rows[0]['meta']['frame'], rows[end - 1]['meta']['frame']], closest_m=round(float(dsl.min()), 3),
                                   first_hit_m=round(hits[0], 3) if hits else None, ok=bool(hits) and hits[0] > .60)
    sets['slipper-approach'] = (rows[:end], False, ['image', 'meta', 'estimated_pose'])
    d0 = np.linalg.norm(pts[:, :2], axis=1) if len(pts) else np.zeros(0); dsp = np.linalg.norm(pts[:, :2] - SLIPPER, axis=1) if len(pts) else np.zeros(0)
    rep['floor-rejection'] = dict(rows=[0, end], points=len(pts), near_origin=int((d0 < .35).sum()), at_slipper=int((dsp < .20).sum()),
                                  ok=bool(len(pts) > 0 and not (d0 < .35).any() and (dsp < .20).any()))
    sets['floor-rejection'] = (rows[:end], False, ['image', 'meta', 'estimated_pose'])
    cands = []
    for i in range(1, len(rows)):
        if rows[i].get('geometry_ready') and not rows[i - 1].get('geometry_ready'):
            j = min(len(rows), i + 60); dd = np.linalg.norm(pose[i:j, :2] - SLIPPER, axis=1)
            if len(dd) and dd.min() < .8 and dd[0] > .7:
                k = i + int(dd.argmin()) + 1; h, _ = mono_hits(stereo, rows[i:k])
                cands.append(dict(rows=[i, k], frames=[rows[i]['meta']['frame'], rows[k - 1]['meta']['frame']], start_m=round(float(dd[0]), 3),
                                  closest_m=round(float(dd.min()), 3), first_hit_m=round(h[0], 3) if h else None, ok=bool(h) and h[0] > .60))
    rep['slipper-side-approach'] = cands
    good = [c for c in cands if c['ok']]
    if good: sets['slipper-side-approach'] = (rows[good[0]['rows'][0]:good[0]['rows'][1]], False, ['image', 'meta', 'estimated_pose'])
    inbed = np.where((pose[:, 1] < -1.2) & (pose[:, 0] > -3) & (pose[:, 0] < 1))[0]
    bend = int(inbed[0]) + 60 if len(inbed) else min(len(rows), 346)
    _, bpts = mono_hits(stereo, rows[:bend], gray=True, reset_on_unready=True)
    ph = int(((bpts[:, 0] > -.7) & (bpts[:, 0] < .2) & (bpts[:, 1] > -2.6) & (bpts[:, 1] < -1.4)).sum()) if len(bpts) else 0
    rep['bedroom-edges'] = dict(rows=[0, bend], frames=[rows[0]['meta']['frame'], rows[bend - 1]['meta']['frame']], phantom=ph, points=len(bpts), ok=ph == 0)
    sets['bedroom-edges'] = (rows[:bend], True, ['meta', 'estimated_pose', 'geometry_ready', 'image'])
    # Bathroom: ready frames in the bathroom heading south (yaw within 45 deg of -pi/2), before the first tray contact.
    south = np.abs(np.angle(np.exp(1j * (pose[:, 2] + math.pi / 2)))) < math.radians(45)
    inb = (pose[:, 0] > 3.3) & (pose[:, 0] < 5) & (pose[:, 1] < -1.0) & (pose[:, 1] > -2.1) & south & np.array([bool(r.get('geometry_ready')) for r in rows])
    cf = a.contact_frame or 10 ** 9
    runs_, cur = [], []
    for i in range(len(rows)):
        if inb[i] and rows[i]['meta']['frame'] < cf and (not cur or i == cur[-1] + 1): cur.append(i)
        else:
            if len(cur) >= 10: runs_.append(cur)
            cur = [i] if inb[i] and rows[i]['meta']['frame'] < cf else []
    if len(cur) >= 10: runs_.append(cur)
    bath = max(runs_, key=len) if runs_ else []
    out = parallax(stereo, [rows[i] for i in bath]) if bath else []
    tray = [c for c in out if inside(c[1], TRAY)]
    rep['floor_parallax.bathroom'] = dict(rows=[bath[0], bath[-1] + 1] if bath else None, frames=[rows[bath[0]]['meta']['frame'], rows[bath[-1]]['meta']['frame']] if bath else None,
                                          tray_cells=len(tray), farthest_m=round(max((c[2] for c in tray), default=0), 3), other_cells=len(out) - len(tray),
                                          ok=len(tray) >= 3 and max((c[2] for c in tray), default=0) >= .40 and len(out) == len(tray))
    thr = [i for i in np.where((np.abs(pose[:, 0] - 1.0) < .6) & (np.abs(pose[:, 1]) < .4))[0]][::3]
    tout = parallax(stereo, [rows[i] for i in thr])
    rep['floor_parallax.threshold'] = dict(rows=len(thr), frames=[rows[thr[0]]['meta']['frame'], rows[thr[-1]]['meta']['frame']] if thr else None, cells=len(tout), ok=len(thr) >= 5 and not tout)
    print(json.dumps(rep, indent=1, default=str))
    if not a.write: return
    out_dir = Path(a.write); src = dict(recording=str(stereo.relative_to(SIM)) if stereo.is_relative_to(SIM) else str(stereo),
                                          robot_log=str((mapdir / 'camera-inputs.jsonl').relative_to(SIM)) if mapdir.is_relative_to(SIM) else str(mapdir / 'camera-inputs.jsonl'),
                                          benchmark=a.benchmark, camera_mount_m=list(CAMERA_MOUNT), cut_by='tests/cut_camera_fixtures.py')
    def put(name, sel, gray, keys, prefix='left', extra=None):
        d = out_dir / name; d.mkdir(parents=True, exist_ok=True); frames = []
        for r in sel:
            fn = f"{prefix}_{r['meta']['frame']:06}.png"; im = img(stereo, r, gray); cv2.imwrite(str(d / fn), im)
            row = dict(image=fn, meta=r['meta'], estimated_pose=r['estimated_pose'], geometry_ready=r.get('geometry_ready'))
            frames.append({k: row[k] for k in keys})
        return frames
    for name, (sel, gray, keys) in sets.items():
        fr = put(name, sel, gray, keys)
        doc = dict(source=dict(src, frames=f"{fr[0]['meta']['frame']}-{fr[-1]['meta']['frame']}"), camera_mount_m=list(CAMERA_MOUNT), frames=fr)
        (out_dir / name / 'inputs.json').write_text(json.dumps(doc, indent=1))
    if bath:
        fp = dict(source=dict(src, conversion='left_<frame>.png (BGR) -> grey PNG; meta, estimated_pose and geometry_ready copied verbatim from the robot log',
                              bathroom=f"frames {rows[bath[0]]['meta']['frame']}-{rows[bath[-1]]['meta']['frame']}: in the bathroom, heading south toward the 6 cm shower tray",
                              threshold=f"every 3rd frame within 0.6 m of the 10 mm threshold at x = 1.0"),
                  camera_mount_m=list(CAMERA_MOUNT), contact_frame=a.contact_frame,
                  sequences=dict(bathroom=put('floor_parallax', [rows[i] for i in bath], True, ['image', 'meta', 'estimated_pose', 'geometry_ready'], 'bathroom'),
                                 threshold=put('floor_parallax', [rows[i] for i in thr], True, ['image', 'meta', 'estimated_pose', 'geometry_ready'], 'threshold')))
        (out_dir / 'floor_parallax' / 'inputs.json').write_text(json.dumps(fp, indent=1))
    print('written to', out_dir)


if __name__ == '__main__': main()
