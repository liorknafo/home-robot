import sys,json,unittest,cv2
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'controller'))
from autonomy.floor_parallax import LowObstacleCamera,CellVoter,FloorParallax
from autonomy.lidar import LidarGrid
from autonomy.geometry import CAMERA_MOUNT,LIDAR_MOUNT,ROBOT_TOP
import autonomy.floor_parallax as floor_parallax
import autonomy.mono as mono
from recorded_camera import RECORDED_CAMERA_MOUNT,recorded_mount,fixture_mount

FIXTURE=Path(__file__).parent/'fixtures/floor_parallax'
# Offline annotations from Content/RobotData/house-scene.json boxes (map frame of this run is within ~2 cm of it).
# Assertions only; never detector inputs. The fixture was re-recorded on 2026-10-09 (bay B10 phase 2) with the round,
# low-profile robot at the camera mount of that time (inputs.json camera_mount_m, (0.117, 0, 0.1075)): its front bumper met
# the tray at inputs.json contact_frame (1544, after a turn), after the bathroom clip ends. Every replay runs under
# recorded_mount(MOUNT), the recording's extrinsics (the 2026-10-03 clip was taken at the 0.15 m mount, contact ~1381).
# Sensitivity (carry item 4, task F): the bathroom clip 1450-1480 is a narrow pass. Started at each of frames 1450-1457 it
# meets the tray rules (>= 3 cells, one >= 0.40 m away) from 5 of the 8 starts, with 2-6 tray cells and never a cell
# elsewhere. Task F's Unreal run at the bumper-band mount (0.146, 0, 0.08) could not replace it: the cutter's own episode
# (the longest ready run toward the tray before contact) gave 2 tray cells (first 1.03 m away), and ten straight drives at
# the tray a median of 2 cells (found 10/10, first a median 0.94 m away). The 3-cell rule is at the edge of what the
# detector gives at that mount: a perception-margin item.
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
WALL_TEX=_texture(3,(1200,2400))
# A plain dark face with faint texture, like the Unreal sofa's upholstery: its only strong feature is its bottom edge.
PLAIN_TEX=(60+.15*(BOX_TEX-128)).astype(np.float32)
TEXTURES={'box':BOX_TEX,'wall':WALL_TEX,'plain':PLAIN_TEX}
def render(pose,box):
    """Grey image from the camera at robot pose (x, y, yaw): floor z=0 and axis-aligned boxes. box: ((x0,y0,z0),(x1,y1,z1))
    textured like a box, or a list of (lo, hi, texture name). Faces seen from below (undersides) are dark, as in shadow."""
    boxes=[(box[0],box[1],'box')] if len(box)==2 and len(box[0])==3 and np.isscalar(box[0][0]) else box
    rot,c=mono.camera(pose);u,v=np.meshgrid(np.arange(IMG_W)+0.,np.arange(IMG_H)+0.)
    d=np.stack([(u-CX)/FX,(v-CY)/FX,np.ones_like(u)],-1)@rot.T
    with np.errstate(divide='ignore',invalid='ignore'):
        tf=np.where(d[...,2]<-1e-9,-c[2]/d[...,2],np.inf);fin=np.isfinite(tf)
        x=np.where(fin,c[0]+tf*d[...,0],0);y=np.where(fin,c[1]+tf*d[...,1],0)
        floor=cv2.remap(FLOOR_TEX,np.clip((x-FLOOR_ORIGIN[0])/TEX_RES,-1,FLOOR_TEX.shape[1]).astype(np.float32),
                        np.clip((y-FLOOR_ORIGIN[1])/TEX_RES,-1,FLOOR_TEX.shape[0]).astype(np.float32),cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=128)
        img=np.where(fin,floor,200.);depth=tf
        for lo,hi,tex in boxes:
            lo,hi=np.array(lo,float),np.array(hi,float);t0=(lo-c)/d;t1=(hi-c)/d;tmin=np.minimum(t0,t1);tmax=np.maximum(t0,t1)
            tn=tmin.max(-1);hit=(tn<tmax.min(-1))&(tn>0)&(tn<depth);axis=tmin.argmax(-1);q=c+np.where(hit,tn,0)[...,None]*d
            a=np.where(axis==0,q[...,1],q[...,0]);b=np.where(axis==2,q[...,1],q[...,2]);src=TEXTURES[tex]
            face=cv2.remap(src,np.clip((a+.3)/TEX_RES,-1,src.shape[1]).astype(np.float32),np.clip((b+.3)/TEX_RES,-1,src.shape[0]).astype(np.float32),cv2.INTER_LINEAR,borderMode=cv2.BORDER_REFLECT)
            shade=np.where(axis==2,np.where(q[...,2]<c[2]+1e-6,1.,.08),np.where(axis==0,.75,.6))
            img=np.where(hit,face*shade,img);depth=np.where(hit,tn,depth)
    return np.clip(img,0,255).astype(np.uint8)
def drive_past(box,frames=32,step=.02):
    """Drive straight along +x at `step` per 5 Hz frame; returns [(robot x, confirmed point)]."""
    camera=LowObstacleCamera();out=[]
    for i in range(frames):
        pose=np.array([step*i,0.,0.])
        for q in camera.process(render(pose,box),dict(frame=i,fx=FX,fy=FX,cx=CX,cy=CY,sim_time_s=.2*i),pose):out.append((pose[0],q))
    return out

# Overhang in the LiDAR's blind band (task M, LIO-121): a sofa-like base, its underside midway between the LiDAR plane and the
# robot top, front face 1.6 m ahead of the start, a textured wall behind it. The robot drives straight at it until the lens is
# 8 cm from the face (the LiDAR turret, 62 mm behind the lens, meets it once the lens is 62 mm past the face).
FACE_X=1.6
def overhang_drive(underside=None,tex='plain',step=.02):
    """[(lens-to-face distance, confirmed cell)] for the drive described above."""
    z0=(LIDAR_MOUNT[2]+ROBOT_TOP)/2 if underside is None else underside
    scene=[((FACE_X,-1.,z0),(FACE_X+.83,1.,.37),tex),((3.4,-2.,0.),(3.5,2.,2.4),'wall')];camera=LowObstacleCamera();out=[]
    for i in range(80):
        pose=np.array([step*i,0.,0.]);gap=FACE_X-pose[0]-CAMERA_MOUNT[0]
        if gap<.08:break
        for q in camera.process(render(pose,scene),dict(frame=i,fx=FX,fy=FX,cx=CX,cy=CY,sim_time_s=.2*i),pose):out.append((gap,q))
    return out
def near_face(q,ahead=.10):return FACE_X-ahead<=q[0]<=FACE_X+.83 and -1.05<=q[1]<=1.05

# Unreal sofa approaches (task M): tests/fixtures/sofa-approach, cut by cut_sofa_fixture.py from straight drives at 0.1 m/s with
# no controller, the robot at task L's 20 mm lower LiDAR (plane 127.5, top 135.15 mm). Every such drive ended with the LiDAR
# turret jammed against the base. Poses are the true level poses (no controller ran). Offline annotation: the sofa's
# upholstered base footprint (house-scene.json), underside 130 mm, its bottom edge rounded back up to ~33 mm.
SOFA=json.loads((FIXTURE.parent/'sofa-approach/inputs.json').read_text());SOFA_BASE=((-2.845,-2.015),(.33,2.41))
def sofa_off(q):return float(np.hypot(max(SOFA_BASE[0][0]-q[0],0,q[0]-SOFA_BASE[0][1]),max(SOFA_BASE[1][0]-q[1],0,q[1]-SOFA_BASE[1][1])))
def sofa_drive(name,overhangs=None,**detector):
    """[(lens-to-face distance, confirmed cell)] for one recorded drive; overhangs (a list) collects the cells reported as
    overhang edges."""
    camera=LowObstacleCamera(**detector);out=[]
    with recorded_mount(tuple(SOFA['camera_mount_m'])):
        for r in SOFA['sequences'][name]:
            gray=cv2.imread(str(FIXTURE.parent/'sofa-approach'/r['image']),cv2.IMREAD_GRAYSCALE)
            for q in camera.process(gray,r['meta'],np.array(r['pose'])):out.append((r['lens_xy_evaluation_only'][0]-SOFA_BASE[0][1],q))
            if overhangs is not None:overhangs+=[q for q in camera.new_overhangs]
    return out

# Dining chairs (task Q, LIO-137): tests/fixtures/dining-chairs, cut by cut_chair_fixture.py from task N's Unreal mapping trial 2
# (GI on, errors on, the robot's own estimated poses). There the band pass took the south chair's south-east leg, 0.8-1.3 m
# away, for an overhang edge (points 31-87 mm high, ranged up to 11 cm short), and the cells sealed the 0.40 m gap between the
# leg and the wall. Offline annotation (house-scene.json): the chair's legs and the wall / door casing south of it.
CHAIRS=json.loads((FIXTURE.parent/'dining-chairs/inputs.json').read_text())
CHAIR_PARTS=[(3.628,3.660,-.240,-.208),(3.628,3.660,.128,.160),(3.280,3.312,-.240,-.208),(3.280,3.312,.128,.160),
             (2.494,3.656,-.756,-.644),(3.650,3.670,-.764,-.636)]
def chair_off(q):return min(float(np.hypot(max(b[0]-q[0],0,q[0]-b[1]),max(b[2]-q[1],0,q[1]-b[3]))) for b in CHAIR_PARTS)
def chair_clip(name):
    """(confirmed cells, cells reported as overhang edges) for one clip, fed as LidarBrain.observe feeds the detector."""
    camera=LowObstacleCamera();cells=[];ov=[]
    with recorded_mount(tuple(CHAIRS['camera_mount_m'])):
        for r in CHAIRS['sequences'][name]:
            if not r['geometry_ready']:camera.reset_history();continue
            cells+=list(camera.process(cv2.imread(str(FIXTURE.parent/'dining-chairs'/r['image']),cv2.IMREAD_GRAYSCALE),r['meta'],np.array(r['estimated_pose'])))
            ov+=list(camera.new_overhangs)
    return cells,ov

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
        """OPEN GAP (L11): every camera image fixture must be recorded at the robot's camera mount (camera_mount_m,
        geometry.CAMERA_MOUNT: (0.146, 0, 0.08) since task F, in the bumper band). Task F re-recorded in Unreal
        (full-mapping-20261011T183022, tests/cut_camera_fixtures.py) and only bedroom-edges passed its test at that mount;
        it was replaced. floor-rejection and floor_parallax stay the B10 recordings at (0.117, 0, 0.1075); slipper-approach and
        slipper-side-approach the 2026-10-03 recordings at RECORDED_CAMERA_MOUNT (.117, 0, .15). Why: the run passed the
        slipper at 0.44 m closest and the mono observer never marked it (slipper-approach, and floor-rejection, which needs a
        hit at the slipper, fail); there was no side approach after a tracker reseed; the bathroom episode gave 2 tray cells
        (3 needed); the threshold episode confirmed 2 cells on the 10 mm threshold (none allowed). In three straight drives
        at the slipper at that mount the floor-parallax detector marked it 0.89-1.07 m ahead (>= 0.60 m), but the mono
        observer never did (once in 18 such drives at 75-107.5 mm). This test fails on purpose (an expected failure) and
        turns into an 'unexpected success' once all five sets are at the current mount."""
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

    def test_detector_reports_heights_up_to_the_robot_top_plus_margin(self):
        # The band's ceiling follows the synced CAD (robot-parameters.json robot_top_z_m), as LIDAR_MOUNT does.
        top=json.loads((Path(__file__).resolve().parents[1]/'Content/RobotData/robot-parameters.json').read_text())['robot_top_z_m']
        self.assertAlmostEqual(ROBOT_TOP,top)
        self.assertAlmostEqual(FloorParallax().cfg['hmax'],ROBOT_TOP+floor_parallax.OVERHANG_MARGIN)
        self.assertGreater(floor_parallax.OVERHANG_MARGIN,0);self.assertLess(ROBOT_TOP+floor_parallax.OVERHANG_MARGIN,.20)

    def test_synthetic_plain_overhang_in_the_blind_band_is_confirmed_before_the_turret_reaches_it(self):
        # Its plain face gives only its bottom edge; the scan plane passes under it. Main's detector reported nothing here.
        hits=overhang_drive()
        on=[g for g,q in hits if near_face(q,.05)]
        self.assertGreaterEqual(len(on),3,'The overhang edge must be marked over more than one cell')
        self.assertGreaterEqual(max(on),.40,'Confirmed while the lens is still >= 0.40 m from the face')
        self.assertEqual([q.tolist() for g,q in hits if not near_face(q)],[],'Cells only at the face (<= 10 cm in front) or under the overhang')

    def test_synthetic_textured_overhang_is_confirmed_from_beyond_0_8_m(self):
        hits=overhang_drive(tex='box')
        self.assertGreaterEqual(max(g for g,q in hits if near_face(q,.05)),.80)
        self.assertEqual([q.tolist() for g,q in hits if not near_face(q)],[])

    def test_synthetic_overhang_above_the_robot_top_and_margin_is_ignored(self):
        # A 20 cm underside clears the robot (top + margin < 20 cm): the floor under it must stay usable.
        self.assertGreater(.20,FloorParallax().cfg['hmax'])
        for tex in ('plain','box'):self.assertEqual([q.tolist() for g,q in overhang_drive(.20,tex)],[])

    def test_recorded_sofa_base_is_confirmed_before_the_turret_reaches_it(self):
        # Main's detector (band=False, hmax = LiDAR plane) confirms nothing in either drive.
        for name,first,ahead in (('straight_y150',.40,.075),('angled_p045',.35,.25)):
            hits=sofa_drive(name);on=[g for g,q in hits if sofa_off(q)<=.075]
            self.assertGreaterEqual(len(on),3,name);self.assertGreaterEqual(max(on),first,name)
            # Viewed obliquely (26 deg here) the same image edge is ranged short, also with true poses: cells (centres) up to 25 cm
            # in front of the face, between the robot and the real edge, never anywhere else. Straight on: on the outline.
            self.assertEqual([q.tolist() for g,q in hits if sofa_off(q)>ahead],[],name)

    def test_band_pass_leaves_the_floor_pass_unchanged(self):
        _,a=replay(self.bath[:30],LowObstacleCamera(parallel=False));_,b=replay(self.bath[:30],LowObstacleCamera(parallel=False,band=False))
        for x,y in zip(a,b):
            self.assertEqual(x[3]['state'],y[3]['state'])
        cam=FloorParallax(parallel=False);ref=FloorParallax(parallel=False,band=False)
        with recorded_mount(MOUNT):
            for gray,meta,pose,ready in self.bath[:30]:
                if not ready:cam.reset();ref.reset();continue
                w,p=cam.process(gray,meta,pose);w0,p0=ref.process(gray,meta,pose);low=p[:,1]>meta['cy']+cam.cfg['vmin']
                np.testing.assert_array_equal(w[low],w0);np.testing.assert_array_equal(p[low],p0)

    def test_voter_tells_overhang_edges_from_low_objects(self):
        # Band-pass points only, nothing from the floor pass within a cell: an overhang edge. A floor-pass point next to it (the
        # lower part of a low object taller than the camera): not one.
        v=CellVoter();edge=np.array([[1.01,0.01,.13],[1.02,.02,.14]]);low=np.array([[1.31,.01,.05],[1.32,.02,.06]])
        pts=np.vstack([edge,low,low+[0,0,.07]]);band=np.array([True,True,False,False,True,True])
        v.add(pts,[.5,0],0,band=band);new,_=v.add(pts,[.54,0],.4,band=band)   # views from within OVERHANG_MAX_RANGE of the edge
        self.assertEqual(len(new),2);np.testing.assert_allclose(v.new_overhangs,[[1.025,.025]])

    def test_band_pass_points_below_the_blind_band_are_never_an_overhang(self):
        # A chair leg seen in the band pass's rows: points 6-8.5 cm high (below the LiDAR plane - 3 cm), nothing from the floor
        # pass (it skips the leg's vertical edges). They confirm a low obstacle, and the high points beside them are no overhang
        # edge; high points with nothing low near them still are.
        v=CellVoter();lo=floor_parallax.OVERHANG_MIN_H
        self.assertAlmostEqual(lo,LIDAR_MOUNT[2]-.03)
        leg=np.array([[1.01,.01,.06],[1.02,.02,.085]]);top=np.array([[1.06,.01,.11],[1.07,.02,.12]]);edge=np.array([[2.01,.01,lo+.002],[2.02,.02,.14]])
        pts=np.vstack([leg,top,edge]);band=np.ones(len(pts),bool)
        v.add(pts,[1.3,0],0,band=band);new,mask=v.add(pts,[1.34,0],.4,band=band)
        np.testing.assert_allclose(sorted(new.tolist()),[[1.025,.025],[1.075,.025],[2.025,.025]]);np.testing.assert_allclose(v.new_overhangs,[[2.025,.025]])
        self.assertTrue(mask.all())

    def test_an_overhang_edge_needs_a_view_from_within_0_8_m(self):
        # Seen only from 0.95-1.0 m (where oblique views range an edge short) the cell is a plain obstacle; one of its two views
        # from 0.6 m makes it an overhang edge.
        self.assertAlmostEqual(floor_parallax.OVERHANG_MAX_RANGE,.8)
        edge=np.array([[1.01,.01,.13],[1.02,.02,.14]]);band=np.ones(2,bool)
        far=CellVoter();far.add(edge,[0.,0.],0,band=band);new,_=far.add(edge,[.05,0.],.4,band=band)
        self.assertEqual(len(new),1);self.assertEqual(len(far.new_overhangs),0)
        near=CellVoter();near.add(edge,[0.,0.],0,band=band);new,_=near.add(edge,[.42,0.],.4,band=band)
        np.testing.assert_allclose(near.new_overhangs,[[1.025,.025]])

    def test_the_north_east_chair_leg_that_sealed_the_robot_is_not_an_overhang_edge(self):
        # Task N trial 2 (and task P trial 1): the cell (4.425, 0.725) on a dining-chair leg was confirmed as an overhang edge and the
        # floor mapped behind it sealed the corridor east of the table. Offline the detector does not repeat that confirmation (cause
        # not found), so its measurements feed the voter directly: the band-pass points it gives on that leg at 865.1 / 865.3 s
        # (61-66 mm high, show_points.py), from two camera positions 6 cm apart on the robot's path, 0.99 and 1.05 m from the cell
        # (offline the detector's views of that leg are 0.98-1.07 m away; live it was confirmed with the robot 1.12 m away). Too low
        # and too far: plain obstacles (the leg's cell and the one north of it), no overhang edge.
        pts=np.array([[4.448,.730,.066],[4.425,.742,.061],[4.429,.760,.066],[4.427,.752,.061]]);band=np.ones(len(pts),bool)
        v=CellVoter();v.add(pts,[4.203,-.296],0.,band=band);new,_=v.add(pts,[4.233,-.244],.2,band=band)
        np.testing.assert_allclose(sorted(new.tolist()),[[4.425,.725],[4.425,.775]]);self.assertEqual(v.new_overhangs.tolist(),[])

    def test_recorded_dining_chair_legs_are_not_overhang_edges(self):
        # Task N's trial 2: before the fix both clips gave overhang edges up to 10.7 cm in front of the leg; the floor mapped
        # behind such edges sealed the robot in. Their cells stay, as low obstacles beside the leg (within 12 cm).
        for name in CHAIRS['sequences']:
            cells,ov=chair_clip(name)
            self.assertEqual([q.tolist() for q in ov],[],name);self.assertGreater(len(cells),0,name)
            self.assertEqual([q.tolist() for q in cells if chair_off(q)>.12],[],name)

    def test_recorded_sofa_base_is_reported_as_an_overhang_edge(self):
        for name in ('straight_y150','angled_p045'):
            ov=[];hits=sofa_drive(name,ov)
            self.assertGreaterEqual(len(ov),3,name);self.assertTrue(all(sofa_off(q)<=.25 for q in ov),name)

    def test_a_box_taller_than_the_camera_is_not_an_overhang(self):
        # 12 cm tall (above the 8 cm camera): the band pass sees its top edge, the floor pass its lower part.
        box=((1.2,0.,0.),(1.3,.1,.12));camera=LowObstacleCamera();ov=[];cells=[]
        for i in range(40):
            pose=np.array([.02*i,0.,0.])
            if box[0][0]-pose[0]-CAMERA_MOUNT[0]<.10:break
            cells+=list(camera.process(render(pose,box),dict(frame=i,fx=FX,fy=FX,cx=CX,cy=CY,sim_time_s=.2*i),pose));ov+=list(camera.new_overhangs)
        self.assertGreaterEqual(len(cells),3);self.assertEqual([q.tolist() for q in ov],[])

if __name__=='__main__':unittest.main()
