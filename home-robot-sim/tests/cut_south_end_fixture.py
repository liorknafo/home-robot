"""Cut the sofa south-end camera fixture (task Q, LIO-137) from task N's Unreal Go-to series with the camera LED on (GI on, errors
on; recording run_20261012T190051_39904127377979, the approach 'angled_from_north' to a target under the sofa on task M's
open-sofa map). Data preparation only.

Usage: cut_south_end_fixture.py STEREO_DIR OUT_DIR
Keeps the frames of sim time 302.0-309.6 s: the robot curves round to the sofa's south end and turns into the 0.57 m gap between
its south legs; at 309.6 s its LiDAR turret meets the end's unconfirmed east part (task N report 4a). The controller's own pose
estimates of that Go-to are not kept (the loaded map's camera log mixes sessions), so each row's 'pose' is the TRUE pose from
the frame's evaluation block, level, with the camera height Unreal reports, as cut_sofa_fixture.py does; the test feeds it to the
detector the way a well-localized robot would. 'turret_face_gap_m' (evaluation only): the turret's distance to the end's face."""
import sys, json, math
from pathlib import Path
import cv2
src, out = Path(sys.argv[1]), Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
FACE_Y, TURRET_X, TURRET_R = .33, .054, .0299          # house-scene.json sofa base south face; robot-parameters.json turret
rows = []
for d in sorted((json.loads(p.read_text()) for p in src.glob('frame_*.json')), key=lambda d: d['sim_time_s']):
    if not 302.0 <= d['sim_time_s'] <= 309.6: continue
    g = d['ground_truth_evaluation_only']; lens = d['camera_position_evaluation_only_m']; name = f"south_end_{d['frame']:06d}.png"
    cv2.imwrite(str(out / name), cv2.imread(str(src / d['left']), cv2.IMREAD_GRAYSCALE), [cv2.IMWRITE_PNG_COMPRESSION, 9])
    ty = g['position_m'][1] + TURRET_X * math.sin(g['yaw_rad'])
    rows.append(dict(image=name, meta=dict(frame=d['frame'], sim_time_s=d['sim_time_s'], fx=d['fx'], fy=d['fy'], cx=d['cx'], cy=d['cy']),
                     pose=[g['position_m'][0], g['position_m'][1], g['yaw_rad'], 0., 0., round(lens[2] - .08, 6), 1.],
                     turret_face_gap_m=round(FACE_Y - (ty + TURRET_R), 4)))
(out / 'inputs.json').write_text(json.dumps(dict(camera_mount_m=[.146, 0., .08], robot='20 mm lower LD14 (task L): LiDAR plane 127.5 mm, robot top 135.15 mm; camera LED (task N) on',
                                                 pose_note='pose = true level pose; see cut_south_end_fixture.py', sequences=dict(angled_from_north=rows)), indent=1))
print(len(rows), 'frames')
