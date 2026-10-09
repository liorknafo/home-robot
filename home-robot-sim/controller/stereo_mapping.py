"""Offline stereo visual odometry and point-cloud mapping from Unreal camera pairs.

Uses image features and stereo depth only for pose. Simulator positions are used
after estimation for scoring. This is local odometry, without loop closure.
"""
import argparse,json,math
from pathlib import Path
import cv2
import numpy as np

def run(folder,out):
    if (folder/"latest-session.json").exists():folder=folder/json.loads((folder/"latest-session.json").read_text())["directory"]
    out.mkdir(parents=True,exist_ok=True)
    files=sorted(folder.glob('frame_*.json'))
    if len(files)<2:raise RuntimeError('Capture at least two stereo pairs first')
    orb=cv2.ORB_create(nfeatures=2200,fastThreshold=7,edgeThreshold=12)
    matcher=cv2.BFMatcher(cv2.NORM_HAMMING)
    stereo=cv2.StereoSGBM_create(minDisparity=0,numDisparities=96,blockSize=5,P1=8*25,P2=32*25,disp12MaxDiff=1,uniquenessRatio=8,speckleWindowSize=60,speckleRange=2,mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    # Camera optical x-right/y-down/z-forward -> robot x-forward/y-left/z-up.
    axis=np.array([[0,0,1],[-1,0,0],[0,-1,0]],float)
    pose=np.eye(4);previous=None;records=[];cloud=[];colors=[];truth_origin=None
    for i,file in enumerate(files):
        m=json.loads(file.read_text());left=cv2.imread(str(folder/m['left']));right=cv2.imread(str(folder/m['right']))
        if left is None or right is None:raise RuntimeError(f'Missing pair for {file.name}')
        l=cv2.cvtColor(left,cv2.COLOR_BGR2GRAY);r=cv2.cvtColor(right,cv2.COLOR_BGR2GRAY)
        disparity=stereo.compute(l,r).astype(np.float32)/16
        depth=np.divide(m['fx']*m['baseline_m'],disparity,out=np.zeros_like(disparity),where=disparity>1)
        valid=(depth>.15)&(depth<6)
        K=np.array([[m['fx'],0,m['cx']],[0,m['fy'],m['cy']],[0,0,1]],float)
        key,desc=orb.detectAndCompute(l,None);ok=True;inlier_count=0;matches_count=0
        if previous is not None:
            pk,pd,pdepth,pvalid=previous;objects=[];pixels=[]
            if pd is not None and desc is not None:
                for pair in matcher.knnMatch(pd,desc,k=2):
                    if len(pair)<2:continue
                    a,b=pair
                    if a.distance>=.72*b.distance:continue
                    u,v=pk[a.queryIdx].pt;x,y=int(round(u)),int(round(v))
                    if not (0<=x<l.shape[1] and 0<=y<l.shape[0] and pvalid[y,x]):continue
                    z=float(pdepth[y,x]);objects.append([(u-m['cx'])*z/m['fx'],(v-m['cy'])*z/m['fy'],z]);pixels.append(key[a.trainIdx].pt)
            matches_count=len(objects);ok=False
            if matches_count>=10:
                ok,rv,tv,inliers=cv2.solvePnPRansac(np.array(objects,np.float32),np.array(pixels,np.float32),K,None,iterationsCount=200,reprojectionError=2.,confidence=.999,flags=cv2.SOLVEPNP_ITERATIVE)
                inlier_count=0 if inliers is None else len(inliers)
                ok=bool(ok and inlier_count>=10 and np.linalg.norm(tv)<.5)
                if ok:
                    step=np.eye(4);step[:3,:3]=cv2.Rodrigues(rv)[0];step[:3,3]=tv[:,0];pose=pose@np.linalg.inv(step)
            # On loss, terminate rather than invent a pose or inject simulator truth.
            if not ok:
                cv2.imwrite(str(out/'tracking-loss.png'),left)
                records.append(dict(frame=m['frame'],sim_time_s=m['sim_time_s'],tracked=False,matches=matches_count,inliers=inlier_count));break
        position=axis@pose[:3,3]
        row=dict(frame=m['frame'],sim_time_s=m['sim_time_s'],tracked=bool(ok),matches=matches_count,inliers=inlier_count,position_m=position.tolist(),valid_depth_fraction=float(valid.mean()))
        gt=m.get('camera_position_evaluation_only_m')
        if gt is not None:
            if truth_origin is None:truth_origin=np.array(gt)
            delta=np.array(gt)-truth_origin;row['evaluation_only_position_m']=delta.tolist();row['evaluation_only_error_m']=float(np.linalg.norm(position-delta))
        records.append(row)
        if i%3==0:
            yy,xx=np.mgrid[0:l.shape[0]:5,0:l.shape[1]:5];mask=valid[yy,xx];u=xx[mask];v=yy[mask];z=depth[v,u]
            pts=np.column_stack(((u-m['cx'])*z/m['fx'],(v-m['cy'])*z/m['fy'],z))
            world=(pts@pose[:3,:3].T+pose[:3,3])@axis.T;cloud.append(world);colors.append(left[v,u,::-1])
        if i==0:
            cv2.imwrite(str(out/'stereo-left.png'),left);cv2.imwrite(str(out/'stereo-right.png'),right)
            shown=cv2.applyColorMap(np.uint8(np.clip(disparity/48,0,1)*255),cv2.COLORMAP_TURBO);shown[~valid]=0;cv2.imwrite(str(out/'stereo-disparity.png'),shown)
        previous=(key,desc,depth,valid)
    tracked=[x for x in records if x['tracked']];errors=[x['evaluation_only_error_m'] for x in tracked if 'evaluation_only_error_m' in x]
    summary=dict(recording_directory=folder.name,frames_available=len(files),frames_tracked=len(tracked),tracking_complete=len(tracked)==len(files),position_source='stereo images only; no encoder or simulator pose feedback',loop_closure=False,max_error_m=max(errors) if errors else None,final_position_m=tracked[-1]['position_m'],records=records)
    (out/'vision-results.json').write_text(json.dumps(summary,indent=2))
    if cloud:
        points=np.concatenate(cloud);rgb=np.concatenate(colors);keep=np.unique(np.round(points/.025).astype(int),axis=0,return_index=True)[1];points=points[keep];rgb=rgb[keep]
        with (out/'stereo-map.ply').open('w') as f:
            f.write(f'ply\nformat ascii 1.0\nelement vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
            for p,c in zip(points,rgb):f.write(' '.join(f'{x:.4f}' for x in p)+' '+' '.join(str(int(x)) for x in c)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k!='records'},indent=2))
if __name__=='__main__':
    root=Path(__file__).resolve().parents[1];p=argparse.ArgumentParser();p.add_argument('--input',type=Path,default=root/'Saved/Stereo');p.add_argument('--output',type=Path,default=root/'reports/vision');a=p.parse_args();run(a.input,a.output)
