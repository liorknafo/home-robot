"""Cut tests/fixtures/tv-console-tilt from Unreal trial full-mapping-20261013T010812 (task Q's trial 2; task R, LIO-153). Evaluation
only, never imported by the controller.
Usage: cut_tv_console_fixture.py REPORT_DIR [OUT_DIR] | cut_tv_console_fixture.py --threshold REPORT_DIR (cut_threshold)

In that trial the robot turned in place at (0.10, 1.89) from 102.5 s to 110.6 s with one wheel on the edge of the 6 mm living rug:
pitch -1.5 to -2.6 deg (nose down), roll +-1.2 to 1.5 deg (the trace's evaluation-only truth). The TV console 0.33 m east of it
has its underside at 124.6 mm, 2.9 mm below the LiDAR plane (127.5 mm, task L): tilted, the beams drop ~17 mm by the console,
pass under it and return the partition 0.5 m behind instead of the console's 1.65 m face. The robot's estimate moved 8.8 cm east,
then kept a 6-7.5 cm offset until 240 s (why is not known: the replay below returns to about 1 cm once level); after a jam on the
10 mm threshold at 245.6 s relocalization failed ("No place in the saved map matches").

The scans are re-cast with tests/replay_unreal_lidar.py (the same 667 beams against the house's triangles from the true pose,
pitch, roll and height; D200 range model) and run through the localizer's own mapping loop from 0 s with perfect odometry
(prediction = previous estimate + the true motion), exactly as test_lidar.TvConsoleTiltTests does: that loop reproduces the jump
(9.4 cm at 108 s, +x). Stored: the localizer's map points and keyframe state just before the tilt (102.3 s), then each scan of
102.5-112 s (base-frame points), its sim time and the true pose (map frame)."""
from pathlib import Path
import json,math,sys
import numpy as np
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'controller'));sys.path.insert(0,str(R/'tests'))
import replay_unreal_lidar as RP
from autonomy.lidar import LidarLocalizer,transform
from autonomy.stereo import wrap
CUT,END=102.3,112.
def cut(report,out):
    report=Path(report);root=RP.snapshot(report);mount=RP.robot(root);tris=RP.house_triangles(root);rows=RP.rows_of(report);t0=rows[0]['status']['sim_time_s']
    origin=json.loads((report/'trial.json').read_text())['map_origin_ground_truth_evaluation_only'];rng=np.random.default_rng(0)
    loc=LidarLocalizer();est=None;prev=None;scans=[];state=None
    for i,truth,pose,tau in RP.trajectory(rows,4):
        t=tau-t0
        if t>END:break
        tp=RP.to_map(origin,truth);p=RP.scan_to_points(RP.cast(*tris,truth,rng,mount=mount),mount[0])
        if t>=CUT:
            if state is None:state=dict(points=loc.points.copy(),key=np.array(loc.last_key_pose),est=est.copy(),true=prev.copy())
            scans.append((tau,p,tp));continue
        if est is None:
            est=tp.copy();loc.points=transform(p,est).astype(np.float32);loc.rebuild();loc.last_key_pose=est.copy();prev=tp;continue
        c,s=math.cos(prev[2]),math.sin(prev[2]);d=tp[:2]-prev[:2];delta=np.array([c*d[0]+s*d[1],-s*d[0]+c*d[1],wrap(tp[2]-prev[2])])
        c,s=math.cos(est[2]),math.sin(est[2]);pred=np.array([est[0]+c*delta[0]-s*delta[1],est[1]+s*delta[0]+c*delta[1],wrap(est[2]+delta[2])]);prev=tp
        loc.pose=pred.copy();loc.history.clear();loc.history.append((tau,pred.copy()))
        good,res=loc.update_scan(p,tau,mapping=True);est=np.asarray(res,float).copy()
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(out/'scans.npz',map_points=state['points'],last_key_pose=state['key'],start_estimate=state['est'],start_true=state['true'],
                        stamps=np.array([s[0] for s in scans]),true_poses=np.array([s[2] for s in scans]),counts=np.array([len(s[1]) for s in scans]),
                        points=np.concatenate([s[1] for s in scans]).astype(np.float32))
    (out/'README.md').write_text('# TV console tilt fixture (task R, LIO-153)\n\nCut by `tests/cut_tv_console_fixture.py` from Unreal trial '
        '`full-mapping-20261013T010812` (task Q trial 2): the LiDAR re-cast from the evaluation-only true pose, pitch and roll '
        '(tests/replay_unreal_lidar.py), the localizer map as its own mapping loop had built it by %.1f s, and %d scans of %.1f-%.1f s '
        'while the robot turned in place tilted on the living rug\'s edge next to the TV console. See the script\'s docstring.\n'%(CUT,len(scans),scans[0][0]-t0,scans[-1][0]-t0))
    return len(scans)


def cut_threshold(report,out):
    """tests/fixtures/threshold-offset from full-mapping-20261013T023710 (task R's Unreal trial 2): the robot came down the 10 mm
    threshold at x ~1.0 heading west at 950.5 s (pitch -7.5 deg), its estimate ended 0.39 m off, and the first seen-through filter
    (judged from the prediction) then let tracking accept poses 0.20-0.22 m off on the level scans that followed, for 18 s. Stored: a
    reference map of the scans before 945 s at their true poses, and the level scans of 951.2-952.0 s with their true poses."""
    report=Path(report);root=RP.snapshot(report);mount=RP.robot(root);tris=RP.house_triangles(root);rows=RP.rows_of(report);t0=rows[0]['status']['sim_time_s']
    origin=json.loads((report/'trial.json').read_text())['map_origin_ground_truth_evaluation_only'];rng=np.random.default_rng(0);pts=[];last=None;scans=[]
    for i,truth,pose,tau in RP.trajectory(rows,4):
        t=tau-t0;tp=RP.to_map(origin,truth)
        if t<945.:
            if last is not None and np.linalg.norm(tp[:2]-last[:2])<.2 and abs(wrap(tp[2]-last[2]))<.45:continue
            pts.append(transform(RP.scan_to_points(RP.cast(*tris,truth,rng,mount=mount),mount[0]),tp));last=tp
        elif 951.1<=t<=952.1:scans.append((tau,RP.scan_to_points(RP.cast(*tris,truth,rng,mount=mount),mount[0]),tp))
        elif t>952.1:break
    out=Path(out);out.mkdir(parents=True,exist_ok=True);from autonomy.lidar import voxel
    np.savez_compressed(out/'scans.npz',map_points=voxel(np.vstack(pts).astype(np.float32)),stamps=np.array([s[0] for s in scans]),true_poses=np.array([s[2] for s in scans]),
                        counts=np.array([len(s[1]) for s in scans]),points=np.concatenate([s[1] for s in scans]).astype(np.float32))
    (out/'README.md').write_text('# Threshold offset fixture (task R, LIO-153)\n\nCut by `tests/cut_tv_console_fixture.py --threshold` from Unreal trial '
        '`full-mapping-20261013T023710` (task R trial 2): a reference map of the scans before 945 s at their true poses, and %d level scans of '
        '951.2-952.0 s just after the robot came down the 10 mm threshold (re-cast from the true pose). See `cut_threshold`.\n'%len(scans))
    return len(scans)


if __name__=='__main__':
    if '--threshold' in sys.argv:print(cut_threshold([a for a in sys.argv[1:] if a!='--threshold'][0],R/'tests/fixtures/threshold-offset'),'scans')
    else:print(cut(sys.argv[1],sys.argv[2] if len(sys.argv)>2 else R/'tests/fixtures/tv-console-tilt'),'scans')
