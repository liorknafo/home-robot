import sys,json,unittest,cv2
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'controller'))
from autonomy.floor_parallax import LowObstacleCamera,CellVoter,FloorParallax
from autonomy.lidar import LidarGrid
from autonomy.geometry import CAMERA_MOUNT,LIDAR_MOUNT
import autonomy.mono as mono
from recorded_camera import RECORDED_CAMERA_MOUNT,recorded_mount,fixture_mount

FIXTURE=Path(__file__).parent/'fixtures/floor_parallax'
# Offline annotations from Content/RobotData/house-scene.json boxes (map frame of this run is within ~2 cm of it).
# Assertions only; never detector inputs. The fixture was re-recorded on 2026-10-09 (bay B10 phase 2) with the round,
# low-profile robot at geometry.CAMERA_MOUNT (inputs.json camera_mount_m, z 0.1075): its front bumper met the tray at
# inputs.json contact_frame (1544, after a turn), after the bathroom clip ends. Every replay runs under
# recorded_mount(MOUNT), the recording's extrinsics (the 2026-10-03 clip was taken at the 0.15 m mount, contact ~1381).
TRAY=((3.25,4.99),(-3.00,-2.04))        # 6 cm shower tray footprint (front face y=-2.13) plus cell slack
BATH_FREE=((3.84,4.28),(-1.98,-1.10))   # bath mat (0.7 cm), its far edge and bare floor between toilet and vanity
THRESHOLD_FREE=((.40,1.60),(-.28,.28))  # 10 mm hardwood threshold at x=1.0 and the open floor around it
def inside(p,box):return box[0][0]<=p[0]<=box[0][1] and box[1][0]<=p[1]<=box[1][1]

DOC=json.loads((FIXTURE/'inputs.json').read_text());MOUNT=fixture_mount(DOC);CONTACT_FRAME=DOC.get('contact_frame',1381)
def frames(name):
    doc=DOC
    return [(cv2.imread(str(FIXTURE/r['image']),cv2.IMREAD_GRAYSCALE),r['meta'],np.array(r['estimated_pose']),r['geometry_ready']) for r in doc['sequences'][name]]
def replay(rows,camera=None,bgr=False):
    camera=camera or LowObstacleCamera();out=[]
    with recorded_mount(MOUNT):
        for gray,meta,pose,ready in rows:
            if not ready:camera.reset_history();continue
            new=camera.process(cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR) if bgr else gray,meta,pose);out.append((meta['frame'],new,pose,dict(camera.status)))
    return camera,out

# Synthetic scene for the CURRENT mount: a textured floor and a textured box, rendered by ray casting from
# mono.camera() at geometry.CAMERA_MOUNT (no depth, labels or poses other than the robot's own enter the detector).
FX,CX,CY,IMG_W,IMG_H=355.396,319.5,179.5,640,360       # the recorded camera's calibration (fixtures' meta)
def _texture(seed,shape):
    rng=np.random.default_rng(seed);fine=cv2.GaussianBlur(rng.normal(0,1,shape).astype(np.float32),(0,0),2.)
    coarse=cv2.GaussianBlur(rng.normal(0,1,shape).astype(np.float32),(0,0),12.)
    return np.clip(128+38*(fine-fine.mean())/fine.std()+20*(coarse-coarse.mean())/coarse.std(),0,255).astype(np.float32)
TEX_RES,FLOOR_ORIGIN=.002,(-.5,-1.5)                    # 2 mm texels; floor texture covers x -0.5..3.5, y -1.5..1.5
FLOOR_TEX,BOX_TEX=_texture(1,(1500,2000)),_texture(2,(400,400))
def render(pose,box):
    """Grey image from the camera at robot pose (x, y, yaw): floor z=0 and an axis-aligned box ((x0,y0,0),(x1,y1,h))."""
    rot,c=mono.camera(pose);u,v=np.meshgrid(np.arange(IMG_W)+0.,np.arange(IMG_H)+0.)
    d=np.stack([(u-CX)/FX,(v-CY)/FX,np.ones_like(u)],-1)@rot.T
    with np.errstate(divide='ignore',invalid='ignore'):
        tf=np.where(d[...,2]<-1e-9,-c[2]/d[...,2],np.inf);fin=np.isfinite(tf)
        x=np.where(fin,c[0]+tf*d[...,0],0);y=np.where(fin,c[1]+tf*d[...,1],0)
        floor=cv2.remap(FLOOR_TEX,np.clip((x-FLOOR_ORIGIN[0])/TEX_RES,-1,FLOOR_TEX.shape[1]).astype(np.float32),
                        np.clip((y-FLOOR_ORIGIN[1])/TEX_RES,-1,FLOOR_TEX.shape[0]).astype(np.float32),cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=128)
        img=np.where(fin,floor,200.)
        lo,hi=np.array(box[0],float),np.array(box[1],float);t0=(lo-c)/d;t1=(hi-c)/d;tmin=np.minimum(t0,t1);tmax=np.maximum(t0,t1)
        tn=tmin.max(-1);hit=(tn<tmax.min(-1))&(tn>0)&(tn<tf);axis=tmin.argmax(-1);q=c+np.where(hit,tn,0)[...,None]*d
        a=np.where(axis==0,q[...,1],q[...,0]);b=np.where(axis==2,q[...,1],q[...,2])
        face=cv2.remap(BOX_TEX,np.clip((a+.3)/TEX_RES,-1,400).astype(np.float32),np.clip((b+.3)/TEX_RES,-1,400).astype(np.float32),cv2.INTER_LINEAR,borderMode=cv2.BORDER_REFLECT)
        img=np.where(hit,face*np.where(axis==2,1.,np.where(axis==0,.75,.6)),img)
    return np.clip(img,0,255).astype(np.uint8)
def drive_past(box,frames=32,step=.02):
    """Drive straight along +x at `step` per 5 Hz frame; returns [(robot x, confirmed point)]."""
    camera=LowObstacleCamera();out=[]
    for i in range(frames):
        pose=np.array([step*i,0.,0.])
        for q in camera.process(render(pose,box),dict(frame=i,fx=FX,fy=FX,cx=CX,cy=CY,sim_time_s=.2*i),pose):out.append((pose[0],q))
    return out

class FloorParallaxTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bath=frames('bathroom');cls.threshold=frames('threshold');cls.camera,cls.out=replay(cls.bath)
    def confirmed(self,out):return [(k,p,float(np.linalg.norm(p-pose[:2]))) for k,new,pose,_ in out for p in new]

    def test_recorded_shower_tray_is_confirmed_before_contact_beyond_40_cm(self):
        tray=[c for c in self.confirmed(self.out) if inside(c[1],TRAY)]
        self.assertGreaterEqual(len(tray),3,'The 6 cm tray front must be marked over more than one cell')
        self.assertGreaterEqual(max(c[2] for c in tray),.40,'Tray must be confirmed while the robot is still >=0.4 m away')
        self.assertLess(min(c[0] for c in tray),CONTACT_FRAME)

    def test_bath_mat_threshold_and_open_floor_are_never_confirmed(self):
        cells=self.confirmed(self.out)
        self.assertFalse([c for c in cells if inside(c[1],BATH_FREE)],'Bath mat or bare floor became an obstacle')
        self.assertFalse([c for c in cells if not inside(c[1],TRAY)],'Only the tray may be confirmed in the bathroom clip')
        _,out=replay(self.threshold);verified=sum(s['state']=='floor motion verified' for *_,s in out)
        self.assertGreaterEqual(verified,5,'Threshold clip must actually be measured, not skipped')
        self.assertEqual(self.confirmed(out),[],'10 mm threshold or open floor became an obstacle')
        for c in self.confirmed(out):self.assertFalse(inside(c[1],THRESHOLD_FREE))

    def test_reset_history_keeps_confirmed_cells(self):
        camera,out=replay(self.bath[:23]);before=camera.confirmed_points().copy()
        self.assertGreater(len(before),0)
        camera.reset_history();self.assertEqual(camera.detector.hist,[]);self.assertEqual(camera.voter.hits,{})
        np.testing.assert_array_equal(camera.confirmed_points(),before)
        gray,meta,pose,_=self.bath[23]
        with recorded_mount(MOUNT):new=camera.process(gray,meta,pose)
        self.assertEqual(new.shape,(0,2));self.assertEqual(camera.status['state'],'needs 6 cm of travel')
        np.testing.assert_array_equal(camera.confirmed_points(),before)
        _,rest=replay(self.bath[24:],camera);again=[tuple(p) for _,new,_,_ in rest for p in new]
        self.assertFalse(set(again)&set(map(tuple,before)),'Confirmed cells must be reported once')

    def test_api_shape_types_grid_alignment_and_determinism(self):
        grid=LidarGrid();seen=set()
        for k,new,pose,status in self.out:
            self.assertIsInstance(new,np.ndarray);self.assertEqual(new.ndim,2);self.assertEqual(new.shape[1],2);self.assertEqual(new.dtype,float)
            json.dumps(status)
            for key in ('state','new_cells','confirmed_cells','candidate_points','obstacle_pixels','ms'):self.assertIn(key,status)
            for p in new:
                cell=grid.cell(p);self.assertNotIn(cell,seen);seen.add(cell)
                np.testing.assert_allclose(grid.world(cell),p,atol=1e-9)
        points=np.vstack([new for _,new,_,_ in self.out]);grid.add_camera_obstacles(points)
        self.assertTrue(all(grid.camera_obstacles[c] for c in seen))
        _,bgr=replay(self.bath,bgr=True);_,serial=replay(self.bath,LowObstacleCamera(parallel=False))
        for run in (bgr,serial):
            for a,b in zip(self.out,run):np.testing.assert_array_equal(a[1],b[1])

    @unittest.expectedFailure
    def test_fixtures_match_the_current_camera_mount(self):
        """OPEN GAP (L11, bay B10): every camera image fixture must be recorded at the robot's camera mount (camera_mount_m,
        geometry.CAMERA_MOUNT, z .1075). B10 phase 2 re-recorded bedroom-edges, floor-rejection and floor_parallax (bathroom
        and threshold) in Unreal at that mount (inputs.json camera_mount_m). slipper-approach and slipper-side-approach are
        still the 2026-10-03 recordings at RECORDED_CAMERA_MOUNT (.117, 0, .15): in the B10 benchmark run the robot passed
        the slipper only head-on (closest 0.43 m) and the mono observer first marked it at 0.54 m, short of the 0.60 m their
        tests require, and there was no side approach from a tracker reseed. This test fails on purpose (an expected
        failure) and turns into an 'unexpected success' once all five sets are at the current mount."""
        for name in ('bedroom-edges','floor-rejection','floor_parallax','slipper-approach','slipper-side-approach'):
            doc=json.loads((FIXTURE.parent/name/'inputs.json').read_text())
            np.testing.assert_allclose(fixture_mount(doc),CAMERA_MOUNT,atol=1e-9,err_msg=f'{name}: re-record this camera fixture at camera_mount_m')

    def test_fixture_replay_uses_the_recorded_camera_mount(self):
        # The fixtures stay valid when the robot's camera moves: they are replayed with the extrinsics they were recorded
        # with, and the robot's own mount (geometry.CAMERA_MOUNT) is restored afterwards.
        with recorded_mount():np.testing.assert_allclose(mono.camera(np.zeros(3))[1],RECORDED_CAMERA_MOUNT,atol=1e-9)
        np.testing.assert_allclose(mono.camera(np.zeros(3))[1],CAMERA_MOUNT,atol=1e-9)

    def test_synthetic_low_box_is_confirmed_at_the_current_camera_mount(self):
        # 6 cm box (the shower tray's height), 10 x 10 cm, its front face 1.2 m ahead of the start; the robot drives toward it.
        box=((1.2,0.,0.),(1.3,.1,.06));hits=drive_past(box)
        near=lambda q,m:box[0][0]-m<=q[0]<=box[1][0]+m and box[0][1]-m<=q[1]<=box[1][1]+m
        on_box=[(x,q) for x,q in hits if near(q,.05)]
        self.assertGreaterEqual(len(on_box),3,'The box must be marked over more than one cell')
        self.assertGreaterEqual(max(float(np.hypot(q[0]-x,q[1])) for x,q in on_box),.60,'Box must be confirmed while the robot is still >= 0.6 m away')
        self.assertEqual([q.tolist() for x,q in hits if not near(q,.10)],[],'Only the box may be confirmed')
        self.assertLess(.06,LIDAR_MOUNT[2],'the test object must lie under the LiDAR plane')

    def test_synthetic_bare_floor_is_never_confirmed_at_the_current_camera_mount(self):
        self.assertEqual(drive_past(((9.,9.,0.),(9.1,9.1,.06))),[],'Bare textured floor became an obstacle')

    def test_voter_needs_two_views_three_cm_apart(self):
        v=CellVoter();pts=np.array([[1.01,2.01,.05],[1.02,2.02,.06],[1.03,2.04,.05],[1.31,2.01,.05]])
        self.assertEqual(v.add(pts,[0,0],0)[0].shape,(0,2))
        self.assertEqual(v.add(pts,[.02,0],.2)[0].shape,(0,2),'Same viewpoint is not independent evidence')
        new,mask=v.add(pts,[.04,0],.4)
        np.testing.assert_allclose(new,[[1.025,2.025]]);self.assertEqual(mask.tolist(),[True,True,True,False])
        self.assertEqual(v.add(pts,[.1,0],.6)[0].shape,(0,2),'A confirmed cell is reported once')
        self.assertEqual(v.add(np.empty((0,3)),[0,0],1)[0].shape,(0,2))

    def test_stationary_camera_gives_no_depth(self):
        gray,meta,pose,_=self.bath[10];d=FloorParallax()
        with recorded_mount(MOUNT):
            for i in range(4):
                world,pixels=d.process(gray,dict(meta,sim_time_s=meta['sim_time_s']+.2*i),pose)
                self.assertEqual(world.shape,(0,3));self.assertEqual(pixels.shape,(0,2))
            self.assertEqual(d.status['state'],'needs 6 cm of travel');n=len(d.hist)
            d.process(gray,dict(meta,sim_time_s=meta['sim_time_s']+.6),pose);self.assertEqual(d.status['state'],'repeated image');self.assertEqual(len(d.hist),n)

if __name__=='__main__':unittest.main()
