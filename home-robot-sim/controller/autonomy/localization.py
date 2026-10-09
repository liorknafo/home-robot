"""Wheel/gyro prediction, stereo visual correction and saved-keyframe relocalization.
Confidence bounds are prototype heuristics, not calibrated statistical guarantees.
"""
import math,json,cv2,numpy as np
from collections import deque
from .stereo import wrap,camera_transform,world_to_base_pose,optical_to_world
from .geometry import AXLE_X,WHEEL_R
from .imu import HEADING_SCALE_UNC,HEADING_BIAS_UNC
# Heading uncertainty (rad) right after a LiDAR / camera fix; it grows with the gyro error model between fixes.
HEADING_FIX_UNC=.005
class Localizer:
    def __init__(self,features):
        self.features=features;self.pose=np.zeros(3);self.last_state=None;self.history=deque(maxlen=100);self.previous=None;self.keyframes=[]
        self.uncertainty=.025;self.last_visual_time=-1.;self.tracking='initializing';self.visual_inliers=0;self.relocalization_hits=0;self.relocalization_candidate=None;self.previous_gyro=0.;self.motion_mismatch_hits=0;self.motion_inconsistent=False;self.motion_residual_m=0.;self.unverified_travel_m=0.;self.heading_uncertainty=HEADING_FIX_UNC
    def invalidate(self):
        self.uncertainty=max(.5,self.uncertainty);self.heading_uncertainty=max(.1,self.heading_uncertainty);self.tracking='unlocalized after possible contact';self.previous=None;self.relocalization_hits=0;self.last_visual_time=-1.
    def check_motion(self,pred,result):
        # Both estimates refer to the same camera timestamp. Disagreement can
        # indicate slip/contact or a bad visual match; neither is trusted blindly.
        self.motion_residual_m=float(np.linalg.norm(result[:2]-pred[:2]))
        mismatch=self.motion_residual_m>.035 or abs(wrap(result[2]-pred[2]))>.12
        self.motion_mismatch_hits=self.motion_mismatch_hits+1 if mismatch else max(0,self.motion_mismatch_hits-1)
        self.motion_inconsistent=self.motion_mismatch_hits>=3
        return self.motion_inconsistent
    def predict(self,stamp,sensors):
        l,r=sensors['left_ticks'],sensors['right_ticks']
        if self.last_state is not None:
            t,pl,pr=self.last_state;dt=stamp-t
            if 0<dt<1.:
                k=2*math.pi*WHEEL_R/sensors['encoder_cpr'];ds=((l-pl)+(r-pr))*k/2
                gyro=float(sensors['gyro_rad_s'][2]);da=.5*(gyro+self.previous_gyro)*dt
                old_yaw=self.pose[2];new_yaw=old_yaw+da
                # Encoders measure the drive axle, AXLE_X ahead of base_link. On the
                # round robot AXLE_X=0: a turn in place leaves base_link where it is.
                self.pose[0]+=ds*math.cos(old_yaw+da/2)+AXLE_X*(math.cos(old_yaw)-math.cos(new_yaw))
                self.pose[1]+=ds*math.sin(old_yaw+da/2)+AXLE_X*(math.sin(old_yaw)-math.sin(new_yaw));self.pose[2]=wrap(new_yaw)
                # Gyro error model (imu.py, B11): the scale factor (+/-3 %, not calibrated) per radian turned and the bias
                # left after the start-up calibration / ZUPT per second grow the heading uncertainty; travel turns it
                # into lateral position uncertainty.
                self.heading_uncertainty+=abs(da)*HEADING_SCALE_UNC+dt*max(HEADING_BIAS_UNC,getattr(self,'gyro_bias_sigma',0.))
                self.uncertainty+=abs(ds)*(.06+self.heading_uncertainty)+abs(da)*.003;self.unverified_travel_m+=abs(ds)+abs(da)*.05
        self.previous_gyro=float(sensors['gyro_rad_s'][2]);self.last_state=(stamp,l,r);self.history.append((stamp,self.pose.copy()))
    def at(self,stamp):
        if not self.history:return self.pose.copy()
        return min(self.history,key=lambda x:abs(x[0]-stamp))[1].copy()
    @staticmethod
    def pnp(objects,pixels,K,min_inliers=24):
        if len(objects)<min_inliers:return None,0
        ok,rv,tv,idx=cv2.solvePnPRansac(np.asarray(objects,np.float32),np.asarray(pixels,np.float32),K,None,iterationsCount=160,reprojectionError=1.8,confidence=.999,flags=cv2.SOLVEPNP_ITERATIVE)
        n=0 if idx is None else len(idx)
        if not ok or n<min_inliers or n/len(objects)<.45:return None,n
        use=idx[:,0];uv=np.asarray(pixels)[use]
        if np.ptp(uv[:,0])<70 or np.ptp(uv[:,1])<35:return None,n
        rv,tv=cv2.solvePnPRefineLM(np.asarray(objects,np.float32)[use],uv.astype(np.float32),K,None,rv,tv)
        pose=world_to_base_pose(rv,tv)
        return pose,n
    def temporal(self,frame):
        if self.previous is None:return None,0
        prev,pose=self.previous;ids=np.flatnonzero(prev.points[:,2]>0)
        if len(ids)<12:return None,0
        a=prev.features['uv'][ids].reshape(-1,1,2)
        b,ok,err=cv2.calcOpticalFlowPyrLK(prev.gray,frame.gray,a,None,winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01))
        back,ok2,_=cv2.calcOpticalFlowPyrLK(frame.gray,prev.gray,b,None,winSize=(21,21),maxLevel=3)
        use=(ok[:,0]>0)&(ok2[:,0]>0)&(np.linalg.norm(back[:,0]-a[:,0],axis=1)<1.0)&(err[:,0]<22)
        result,n=self.pnp(optical_to_world(prev.points[ids[use]],pose),b[use,0],frame.K)
        if result is None:
            pairs=self.features.match(prev.features,frame.features)
            pairs=pairs[prev.points[pairs[:,0],2]>0]
            result,n=self.pnp(optical_to_world(prev.points[pairs[:,0]],pose),frame.features['uv'][pairs[:,1]],frame.K)
        return result,n
    def recognize(self,frame,global_search=False):
        candidates=self.keyframes
        if not global_search:
            candidates=[x for x in candidates if np.linalg.norm(x['pose'][:2]-self.pose[:2])<2.2 and abs(wrap(x['pose'][2]-self.pose[2]))<1.2]
            near=sorted(candidates,key=lambda k:np.linalg.norm(k['pose'][:2]-self.pose[:2])+.4*abs(wrap(k['pose'][2]-self.pose[2])))[:4]
            # Include older anchors, rather than only recent drifting keyframes.
            candidates=candidates[:4]+[k for k in near if all(k is not x for x in candidates[:4])]
        elif self.features.backend=='orb' and len(candidates)>10:
            # Coarse descriptor search bounds the cost as the map grows. Exact
            # metric PnP still decides whether a candidate is accepted.
            def coarse(f):
                ids=np.arange(min(384,len(f['uv'])))
                return {k:(v[ids] if k in ['uv','descriptors'] else v) for k,v in f.items()}
            query=coarse(frame.features)
            ranks=[(len(self.features.match(coarse(k['features']),query)),k) for k in candidates]
            candidates=[k for _,k in sorted(ranks,key=lambda x:-x[0])[:8]]
        scored=[]
        for key in candidates:
            pairs=self.features.match(key['features'],frame.features)
            if len(pairs)>=15:scored.append((len(pairs),key,pairs))
        for _,key,pairs in sorted(scored,key=lambda x:-x[0])[:5]:
            result,n=self.pnp(key['world_points'][pairs[:,0]],frame.features['uv'][pairs[:,1]],frame.K,50)
            if result is not None:return result,n
        return None,0
    def update(self,frame,mapping=False,relocalizing=False):
        pred=self.at(frame.stamp)
        result,n=self.recognize(frame,True) if relocalizing else self.temporal(frame)
        # Local map landmarks periodically anchor the estimate; never use simulator pose.
        if not relocalizing and (result is None or frame.index%10==0):
            anchored,an=self.recognize(frame,relocalizing)
            if anchored is not None:result,n=anchored,an
        good=result is not None
        if good and not relocalizing:
            good=np.linalg.norm(result[:2]-pred[:2])<max(.35,3*self.uncertainty) and abs(wrap(result[2]-pred[2]))<.45
        if relocalizing:
            if good:
                if self.relocalization_candidate is not None and np.linalg.norm(result[:2]-self.relocalization_candidate[:2])<.12 and abs(wrap(result[2]-self.relocalization_candidate[2]))<.20:self.relocalization_hits+=1
                else:self.relocalization_hits=1
                self.relocalization_candidate=result.copy()
                good=self.relocalization_hits>=3
            else:self.relocalization_hits=0
        if good and not relocalizing:self.check_motion(pred,result)
        if good:
            if relocalizing:self.motion_mismatch_hits=0;self.motion_inconsistent=False
            # A validated metric camera pose anchors the capture timestamp fully.
            # Retaining part of wheel/gyro prediction here feeds its bias into
            # every subsequent visual reference and compounds drift.
            gain=1.
            correction=(result-pred);correction[2]=wrap(correction[2]);correction*=gain
            self.pose+=correction;self.pose[2]=wrap(self.pose[2]);pred+=correction;pred[2]=wrap(pred[2])
            self.history=deque(((t,p+correction) for t,p in self.history),maxlen=100)
            self.uncertainty=.04 if relocalizing else max(.025,self.uncertainty*.65);self.heading_uncertainty=HEADING_FIX_UNC;self.last_visual_time=frame.stamp;self.tracking='visual';self.visual_inliers=n
        elif mapping and not self.keyframes and int((frame.points[:,2]>0).sum())>=35:
            self.uncertainty=.025;self.last_visual_time=frame.stamp;self.tracking='map origin';self.visual_inliers=0
        else:self.tracking='predicting' if not relocalizing else 'searching saved landmarks';self.visual_inliers=n
        if self.motion_inconsistent:self.uncertainty=max(.2,self.uncertainty)
        self.previous=(frame,pred.copy())
        if mapping and (good or not self.keyframes) and self.uncertainty<.12:self.add_keyframe(frame,pred)
        return good,pred
    def add_keyframe(self,frame,pose):
        ids=np.flatnonzero(frame.points[:,2]>0)
        if len(ids)<60:return
        if self.keyframes:
            p=self.keyframes[-1]['pose']
            if np.linalg.norm(p[:2]-pose[:2])<.20 and abs(wrap(p[2]-pose[2]))<.22:return
        # Retain features having validated triangulated depth, for future metric PnP.
        f=frame.features;feat={k:(v[ids].copy() if k in ['uv','descriptors'] else v.copy()) for k,v in f.items()}
        self.keyframes.append(dict(pose=pose.copy(),features=feat,world_points=optical_to_world(frame.points[ids],pose).astype(np.float32)))
    def view_support(self,xy,yaw):
        """Expected feature spread from measured landmarks, with no scene model."""
        keys=sorted(self.keyframes,key=lambda k:np.linalg.norm(k['pose'][:2]-xy))[:6]
        if not keys:return 0.
        points=np.concatenate([k['world_points'][::4] for k in keys])
        r,t=camera_transform(np.array([xy[0],xy[1],yaw]));opt=(points-t)@r
        use=(opt[:,2]>.25)&(opt[:,2]<3.3)
        opt=opt[use]
        if not len(opt):return 0.
        uv=opt[:,:2]/opt[:,2,None];uv=uv[(abs(uv[:,0])<.80)&(abs(uv[:,1])<.45)]
        # Image-bin occupancy avoids overvaluing many nearly identical features.
        bins=np.floor((uv+[.8,.45])/[.2,.15]).astype(int)
        return min(1.,len(np.unique(bins,axis=0))/24.)
    def save(self,folder):
        folder.mkdir(parents=True,exist_ok=True)
        data={}
        for i,k in enumerate(self.keyframes):
            data[f'p{i}']=k['pose'];data[f'w{i}']=k['world_points']
            for a,v in k['features'].items():data[f'{a}{i}']=v
        np.savez_compressed(folder/'landmarks.npz',**data)
        (folder/'landmarks.json').write_text(json.dumps({'backend':self.features.backend,'keyframes':len(self.keyframes),'frame':'map origin at mapping start; metres, X forward Y left','calibration_assumption':'rectified 62 mm stereo; fixed camera mounting'},indent=2))
    def load(self,folder):
        meta=json.loads((folder/'landmarks.json').read_text())
        if meta['backend']!=self.features.backend:raise ValueError('Saved map feature backend does not match active detector')
        d=np.load(folder/'landmarks.npz',allow_pickle=False);self.keyframes=[]
        for i in range(meta['keyframes']):self.keyframes.append(dict(pose=d[f'p{i}'],world_points=d[f'w{i}'],features={a:d[f'{a}{i}'] for a in ['uv','descriptors','size']}))
        self.previous=None;self.relocalization_hits=0;self.uncertainty=.5;self.tracking='unlocalized'
