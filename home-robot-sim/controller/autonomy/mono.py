"""Supplemental camera motion check and sparse low-obstacle reconstruction.

Inputs: RGB pixels, calibration, and timestamped estimated LiDAR/encoder poses.
No simulator depth, object labels, house geometry or ground-truth pose is used.
Relative geometry comes from observed floor motion, with calibrated camera
height supplying metric scale and measured motion checking the solution.
Stationary monocular images do not establish depth. The calibrated mount is
at geometry.CAMERA_MOUNT (8 cm above the floor since task F, in the bumper band); downward ranges compensate small body movements.
"""
import math
import cv2
import numpy as np
from .stereo import wrap
from .geometry import CAMERA_MOUNT,BASE_Z


def camera(pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    roll,pitch=pose[3:5] if len(pose)>=5 else (0.,0.)
    cr,sr=math.cos(roll),math.sin(roll);cp,sp=math.cos(pitch),math.sin(pitch)
    base=np.array([[c,-s,0],[s,c,0],[0,0,1.]]) @ np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]]) @ np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    # Optical right/down/forward expressed in the map's right-handed frame.
    rotation = base @ np.array([[0,0,1],[-1,0,0],[0,-1,0.]])
    height_offset=pose[5] if len(pose)>=6 else 0.
    # Mount from geometry.CAMERA_MOUNT (x, y, height above floor); base_link nominally BASE_Z above the floor.
    centre = np.array([pose[0],pose[1],BASE_Z+height_offset])+base@np.array([CAMERA_MOUNT[0],CAMERA_MOUNT[1],CAMERA_MOUNT[2]-BASE_Z])
    return rotation, centre


def project(points, pose, k):
    rotation, centre = camera(pose)
    optical = (points-centre) @ rotation
    pixels = optical @ k.T
    return pixels[:, :2] / pixels[:, 2:3], optical[:, 2]


def reconstruct(a, b, pose_a, pose_b, k):
    """Return world points plus a conservative validity mask for triangulation."""
    ra, ca = camera(pose_a); rb, cb = camera(pose_b)
    pa = k @ np.column_stack((ra.T, -ra.T @ ca))
    pb = k @ np.column_stack((rb.T, -rb.T @ cb))
    homogeneous = cv2.triangulatePoints(pa, pb, a.T.astype(float), b.T.astype(float)).T
    with np.errstate(divide='ignore', invalid='ignore'):
        world = homogeneous[:, :3] / homogeneous[:, 3:4]
        ua, za = project(world, pose_a, k); ub, zb = project(world, pose_b, k)
    rays_a = np.column_stack((a, np.ones(len(a)))) @ np.linalg.inv(k).T @ ra.T
    rays_b = np.column_stack((b, np.ones(len(b)))) @ np.linalg.inv(k).T @ rb.T
    rays_a /= np.linalg.norm(rays_a, axis=1)[:, None]
    rays_b /= np.linalg.norm(rays_b, axis=1)[:, None]
    sine = np.linalg.norm(np.cross(rays_a, rays_b), axis=1)
    error = np.maximum(np.linalg.norm(ua-a, axis=1), np.linalg.norm(ub-b, axis=1))
    valid = (np.isfinite(world).all(axis=1) & (za>.15) & (zb>.15) & (za<2.5) &
             (zb<2.5) & (sine>.012) & (error<1.2))
    # Pixel-error approximation, not a calibrated statistical bound.
    vertical_error = .7/k[0, 0]*np.maximum(za, zb)*(1+np.abs(rays_a[:, 2])/np.maximum(sine, .001))
    return world, valid, vertical_error


def motion_agreement(a, b, pose_a, pose_b, k):
    """Check observed correspondences against the expected epipolar geometry.

    This can veto conflicting motion; it cannot verify translation scale or
    every kind of wheel slip. Low texture/coverage returns unknown, not safe.
    """
    if len(a)<40 or np.ptp(a[:, 0])<150 or np.ptp(a[:, 1])<65:
        return {'state':'insufficient features', 'tracks':len(a), 'residual_px':None}
    ra, ca = camera(pose_a); rb, cb = camera(pose_b)
    r = rb.T @ ra; t = rb.T @ (ca-cb)
    aa = np.column_stack((a, np.ones(len(a)))); bb = np.column_stack((b, np.ones(len(b))))
    inv = np.linalg.inv(k)
    if np.linalg.norm(t)<.004:
        h = k @ r @ inv; predicted = aa @ h.T
        error = np.linalg.norm(predicted[:, :2]/predicted[:, 2:3]-b, axis=1)
    else:
        tx = np.array([[0, -t[2], t[1]], [t[2], 0, -t[0]], [-t[1], t[0], 0]])
        f = inv.T @ tx @ r @ inv; lb = aa @ f.T; la = bb @ f
        error = np.abs(np.sum(bb*lb, axis=1))/np.sqrt(np.maximum(1e-16, np.sum(lb[:, :2]**2+la[:, :2]**2, axis=1)))
    residual = float(np.median(error))
    return {'state':'disagreement' if residual>6.0 else 'consistent', 'tracks':len(a), 'residual_px':residual}


def above_floor_evidence(a, b, pose_a, pose_b, k):
    """Reject the dominant near-floor image motion before using sparse depth.

    LiDAR pose errors can turn triangulated floor points into false obstacles.
    A broad lower-image homography estimates observed floor motion directly;
    it must also agree with the calibrated floor hypothesis. Insufficient
    support yields unknown, never a made-up obstacle or a free-space claim.
    """
    lower=(a[:,1]>k[1,2]+40)&(b[:,1]>k[1,2]+30)
    if lower.sum()<45:return np.zeros(len(a),bool), 'insufficient floor evidence'
    h, mask=cv2.findHomography(a[lower],b[lower],cv2.RANSAC,2.0)
    if h is None or mask.sum()<35 or mask.mean()<.4:return np.zeros(len(a),bool), 'floor motion uncertain'
    aa=a[lower][mask[:,0]>0]
    if np.ptp(aa[:,0])<150 or np.ptp(aa[:,1])<35:return np.zeros(len(a),bool), 'floor coverage insufficient'
    ra,ca=camera(pose_a)
    rays=np.column_stack((aa,np.ones(len(aa))))@np.linalg.inv(k).T@ra.T
    ground=ca+(-ca[2]/rays[:,2])[:,None]*rays
    predicted,_=project(ground,pose_b,k)
    fitted=cv2.perspectiveTransform(aa[:,None,:].astype(np.float32),h)[:,0]
    if np.median(np.linalg.norm(fitted-predicted,axis=1))>12:
        return np.zeros(len(a),bool), 'floor hypothesis inconsistent'
    floor_uv=cv2.perspectiveTransform(a[:,None,:].astype(np.float32),h)[:,0]
    return np.linalg.norm(floor_uv-b,axis=1)>3.0, 'floor motion verified'


def floor_reconstruction(a,b,pose_a,pose_b,k):
    """Recover relative camera geometry from observed floor motion.

    Small IMU attitude errors overwhelm the parallax of a short camera baseline.
    Decomposing the measured floor homography estimates that relative rotation
    and the floor normal from pixels. Calibrated camera height supplies scale;
    measured robot movement selects/checks the physically plausible solution.
    Neither simulator geometry nor its true pose is an input.
    """
    empty=(np.zeros((len(a),3)),np.zeros(len(a),bool),np.ones(len(a)))
    lower=(a[:,1]>k[1,2]+40)&(b[:,1]>k[1,2]+30)
    if lower.sum()<45:return *empty,'insufficient floor evidence'
    h,mask=cv2.findHomography(a[lower],b[lower],cv2.RANSAC,2.0)
    if h is None or mask.sum()<35 or mask.mean()<.4:return *empty,'floor motion uncertain'
    support=a[lower][mask[:,0]>0]
    if np.ptp(support[:,0])<150 or np.ptp(support[:,1])<35:return *empty,'floor coverage insufficient'
    ra,ca=camera(pose_a);rb,cb=camera(pose_b);expected_r=rb.T@ra;expected_t=rb.T@(ca-cb)
    baseline=np.linalg.norm(expected_t);distance=float(np.clip(ca[2],CAMERA_MOUNT[2]-.03,CAMERA_MOUNT[2]+.03))
    if baseline<.035:return *empty,'needs movement'
    count,rotations,translations,normals=cv2.decomposeHomographyMat(h,k);candidates=[]
    for r,t,n in zip(rotations,translations,normals):
        t=t[:,0]*distance;n=n[:,0]
        if n[1]<.85 or abs(n[0])>.35 or abs(n[2])>.35:continue
        rotation_error=np.linalg.norm(cv2.Rodrigues(r@expected_r.T)[0]);length=np.linalg.norm(t)
        if rotation_error>.15 or not .4*baseline<length<2.5*baseline:continue
        direction=float(t@expected_t/max(1e-8,length*baseline))
        if direction<.6:continue
        candidates.append((rotation_error+abs(length-baseline),r,t,n))
    if not candidates:return *empty,'floor geometry not supported by motion'
    _,r,t,n=min(candidates,key=lambda x:x[0])
    p0=k@np.column_stack((np.eye(3),np.zeros(3)));p1=k@np.column_stack((r,t))
    homogeneous=cv2.triangulatePoints(p0,p1,a.T.astype(float),b.T.astype(float)).T
    with np.errstate(divide='ignore',invalid='ignore'):
        points=homogeneous[:,:3]/homogeneous[:,3:4];next_points=points@r.T+t
        ua=points@k.T;ub=next_points@k.T
        error=np.maximum(np.linalg.norm(ua[:,:2]/ua[:,2:3]-a,axis=1),np.linalg.norm(ub[:,:2]/ub[:,2:3]-b,axis=1))
    rays_a=np.column_stack((a,np.ones(len(a))))@np.linalg.inv(k).T
    rays_b=np.column_stack((b,np.ones(len(b))))@np.linalg.inv(k).T@r
    rays_a/=np.linalg.norm(rays_a,axis=1)[:,None];rays_b/=np.linalg.norm(rays_b,axis=1)[:,None]
    sine=np.linalg.norm(np.cross(rays_a,rays_b),axis=1)
    floor_uv=cv2.perspectiveTransform(a[:,None,:].astype(np.float32),h)[:,0]
    raised=np.linalg.norm(floor_uv-b,axis=1)>3.0
    valid=np.isfinite(points).all(axis=1)&(points[:,2]>.15)&(next_points[:,2]>.15)&(points[:,2]<2.5)&(sine>.012)&(error<1.3)&raised
    visual_pose=np.r_[pose_a[:3],math.atan2(n[0],n[1]),math.asin(np.clip(n[2],-1,1)),0.]
    rotation,centre=camera(visual_pose);centre[2]=distance;world=points@rotation.T+centre
    sigma=.7/k[0,0]*points[:,2]*(1+np.abs(rays_a@n)/np.maximum(sine,.001))
    return world,valid,sigma,'floor geometry verified'


class MonoObserver:
    def __init__(self):
        self.gray = None; self.uv = None; self.anchor_uv = None; self.anchor_pose = None
        self.previous_pose = None; self.last_stamp = None; self.anchor_stamp = None
        self.candidates = {}; self.bad_frames = 0; self.confirmed_total = 0
        self.status = {'state':'waiting for images', 'tracks':0, 'obstacle_points':0,
                       'confirmed_total':0, 'motion':{'state':'not checked'}, 'depth':'needs movement'}

    def seed(self, gray, pose, stamp):
        # Allocate tracks across image tiles. Strong terrazzo corners must not
        # consume the whole budget and hide weaker outlines on shoes and toys.
        points=[]
        for y in range(0,gray.shape[0],90):
            for x in range(0,gray.shape[1],80):
                found=cv2.goodFeaturesToTrack(gray[y:y+90,x:x+80],maxCorners=45,qualityLevel=.001,minDistance=3,blockSize=3)
                if found is not None:points.extend((found[:,0]+[x,y]).tolist())
        self.uv=np.asarray(points,np.float32).reshape(-1,2)
        self.anchor_uv = self.uv.copy(); self.anchor_pose = pose.copy(); self.anchor_stamp = stamp
        self.candidates = {}; self.ids = np.arange(len(self.uv))

    def process(self, image, meta, pose):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim==3 else image
        stamp = meta['sim_time_s']; k = np.array([[meta['fx'], 0, meta['cx']], [0, meta['fy'], meta['cy']], [0, 0, 1.]])
        self.status['obstacle_points'] = 0; self.status['obstacle_pixels'] = []; confirmed = np.empty((0, 3))
        if self.gray is None or self.last_stamp is None or stamp-self.last_stamp>1 or stamp<=self.last_stamp:
            self.seed(gray, pose, stamp); self.status['state'] = 'collecting views'; self.bad_frames = 0
        else:
            a = self.uv.reshape(-1, 1, 2)
            if len(a):
                b, ok, error = cv2.calcOpticalFlowPyrLK(self.gray, gray, a, None, winSize=(21, 21), maxLevel=3)
                back, ok2, _ = cv2.calcOpticalFlowPyrLK(gray, self.gray, b, None, winSize=(21, 21), maxLevel=3)
                use = ((ok[:, 0]>0) & (ok2[:, 0]>0) & (error[:, 0]<22) &
                       (np.linalg.norm(back[:, 0]-a[:, 0], axis=1)<.7))
                old = a[use, 0]; self.uv = b[use, 0]; self.anchor_uv = self.anchor_uv[use]; self.ids = self.ids[use]
                agreement = motion_agreement(old, self.uv, self.previous_pose, pose, k)
                self.bad_frames = self.bad_frames+1 if agreement['state']=='disagreement' else 0
                self.status['motion'] = agreement
            baseline = np.linalg.norm(camera(pose)[1]-camera(self.anchor_pose)[1])
            self.status.update(state='tracking' if len(self.uv)>=40 else 'insufficient features', tracks=len(self.uv),
                               baseline_m=float(baseline), depth='needs movement' if baseline<.075 else 'triangulating')
            if len(self.uv) and baseline>=.075 and self.bad_frames==0:
                world,valid,sigma,floor_state=floor_reconstruction(self.anchor_uv,self.uv,self.anchor_pose,pose,k)
                self.status['floor']=floor_state
                low = valid & (world[:, 2]>.03+2*sigma) & (world[:, 2]<.24) & (sigma<.025)
                # Optical flow along a plain edge is ambiguous (the aperture
                # problem). Forward/backward tracking alone does not detect it.
                # Require measurable image variation in both patch directions.
                eigen=cv2.cornerEigenValsAndVecs(gray,7,3)
                uv=np.rint(self.uv).astype(int);uv[:,0]=np.clip(uv[:,0],0,gray.shape[1]-1);uv[:,1]=np.clip(uv[:,1],0,gray.shape[0]-1)
                values=eigen[uv[:,1],uv[:,0],:2];weak=values.min(axis=1);strong=values.max(axis=1)
                low &= (weak>5e-5)&(weak>.01*np.maximum(1e-9,strong))
                promoted = []; promoted_pixels = []
                for point, idx, pixel in zip(world[low], self.ids[low], self.uv[low]):
                    old = self.candidates.get(int(idx))
                    if old is not None and np.linalg.norm(camera(pose)[1]-camera(old[1])[1])>=.025:
                        # Independent floor fits move otherwise consistent
                        # sparse points by several centimetres. This bound is
                        # below the map's 5 cm observation margin; retain all
                        # height, parallax and independent-track support gates.
                        if np.linalg.norm(point-old[0])<.04:
                            promoted.append((point+old[0])/2); promoted_pixels.append(pixel)
                        self.candidates[int(idx)] = (point.copy(), pose.copy())
                    elif old is None:self.candidates[int(idx)] = (point.copy(), pose.copy())
                # A single feature cannot establish an obstacle. Require support
                # from another independent track within 7 cm in this third view.
                if len(promoted)>=2:
                    pts = np.asarray(promoted); distance = np.linalg.norm(pts[:, None, :]-pts[None, :, :], axis=2)
                    pixels=np.asarray(promoted_pixels);spread=np.abs(pixels[:,None,:]-pixels[None,:,:])
                    # Several samples of one vertical/horizontal edge are not
                    # independent support for a low obstacle.
                    support = ((distance<.07)&(distance>.003)&(spread[:,:,0]>2)&(spread[:,:,1]>2)).sum(axis=1)>=1
                    confirmed = pts[support]
                    self.status['obstacle_pixels'] = np.asarray(promoted_pixels)[support].tolist()
                self.confirmed_total += len(confirmed)
                self.status.update(obstacle_points=len(confirmed), confirmed_total=self.confirmed_total)
            if len(self.uv)<100 or baseline>.24 or abs(wrap(pose[2]-self.anchor_pose[2]))>.65 or stamp-self.anchor_stamp>5:
                self.seed(gray, pose, stamp)
        self.gray = gray; self.previous_pose = pose.copy(); self.last_stamp = stamp
        return confirmed
