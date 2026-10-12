"""Cut the sofa-approach camera fixture (task M, LIO-121) from Unreal straight drives at the sofa base. Data preparation only.

Usage: cut_sofa_fixture.py DRIVES_DIR OUT_DIR [label ...]
  DRIVES_DIR  one folder per drive (frame_<n>.json + left_<n>.png as Unreal's Saved/Stereo writes them), recorded with the
              autonomy service stopped: reset to a start pose in front of the sofa, 0.1 m/s straight ahead, camera at 5 Hz,
              until the LiDAR turret met the base (robot at the 20 mm lower LiDAR of task L: plane 127.5, top 135.15 mm).
Writes OUT_DIR/inputs.json and grey PNGs, keeping the frames up to the last one with the lens still in front of the base's box
face (x -2.015, house-scene.json). No controller ran, so there is no estimated pose: each row's 'pose' is the TRUE pose from the
frame's evaluation block, level (roll = pitch = 0) with the camera height Unreal reports, as task F's tray drives used. The test
feeds it to the detector the way a well-localized robot would; it never reaches the robot."""
import sys, json, math
from pathlib import Path
import cv2
FACE_X = -2.015
src, out = Path(sys.argv[1]), Path(sys.argv[2]); labels = sys.argv[3:] or ['straight_y150', 'angled_p045']
out.mkdir(parents=True, exist_ok=True); seqs = {}
for lab in labels:
    rows = []
    for d in sorted((json.loads(p.read_text()) for p in (src / lab).glob('frame_*.json')), key=lambda d: d['frame']):
        lens = d['camera_position_evaluation_only_m']
        if lens[0] <= FACE_X: break
        g = d['ground_truth_evaluation_only']; name = f"{lab}_{d['frame']:06d}.png"
        cv2.imwrite(str(out / name), cv2.imread(str(src / lab / d['left']), cv2.IMREAD_GRAYSCALE))
        rows.append(dict(image=name, meta=dict(frame=d['frame'], sim_time_s=d['sim_time_s'], fx=d['fx'], fy=d['fy'], cx=d['cx'], cy=d['cy']),
                         pose=[g['position_m'][0], g['position_m'][1], g['yaw_rad'], 0., 0., round(lens[2] - .08, 6), 1.],
                         lens_xy_evaluation_only=[round(lens[0], 4), round(lens[1], 4)]))
    seqs[lab] = rows
(out / 'inputs.json').write_text(json.dumps(dict(camera_mount_m=[.146, 0., .08], robot='20 mm lower LD14 (task L): LiDAR plane 127.5 mm, robot top 135.15 mm',
                                                 pose_note='pose = true level pose (no controller ran); see cut_sofa_fixture.py', sequences=seqs), indent=1))
print({k: len(v) for k, v in seqs.items()})
