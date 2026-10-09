import sys, unittest, tempfile, json, cv2
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'controller'))
from autonomy.mono import project, reconstruct, motion_agreement, MonoObserver, above_floor_evidence, floor_reconstruction
from autonomy.lidar import LidarGrid
from autonomy.lidar_brain import LidarBrain, downward_floor_pose
from autonomy.geometry import CLIFF_XY, CLIFF_Z, BASE_Z, CAMERA_MOUNT
from recorded_camera import recorded_mount, fixture_mount   # each image fixture is replayed at the camera mount it was recorded with

K=np.array([[355.,0,319.5],[0,355.,179.5],[0,0,1.]])

class MonoTests(unittest.TestCase):
    def test_recorded_distant_edges_do_not_block_the_bedroom_floor(self):
        fixture=Path(__file__).parent/'fixtures/bedroom-edges';observer=MonoObserver();accepted=[];doc=json.loads((fixture/'inputs.json').read_text())
        with recorded_mount(fixture_mount(doc)):
            for row in doc['frames']:
                if not row['geometry_ready']:observer=MonoObserver();continue
                accepted.extend(observer.process(cv2.imread(str(fixture/row['image']),0),row['meta'],np.array(row['estimated_pose'])).tolist())
        points=np.asarray(accepted).reshape(-1,3)
        # Offline annotation of an empty floor patch; never an observer input.
        phantom=(points[:,0]>-.7)&(points[:,0]<.2)&(points[:,1]>-2.6)&(points[:,1]<-1.4)
        self.assertFalse(phantom.any(),'Distant wall/door edges became close obstacles')
    def test_recorded_side_approach_confirms_slipper_before_stopping_distance(self):
        fixture=Path(__file__).parent/'fixtures/slipper-side-approach';observer=MonoObserver();hits=[]
        with recorded_mount():
            for row in json.loads((fixture/'inputs.json').read_text())['frames']:
                points=observer.process(cv2.imread(str(fixture/row['image'])),row['meta'],np.array(row['estimated_pose']))
                if len(points) and np.any(np.linalg.norm(points[:,:2]-[-1.29,-.18],axis=1)<.19):
                    hits.append(np.linalg.norm(np.array(row['estimated_pose'][:2])-[-1.29,-.18]))
        self.assertTrue(hits);self.assertGreater(hits[0],.60)

    def test_recorded_floor_is_not_obstacle_but_slipper_is(self):
        fixture=Path(__file__).parent/'fixtures/floor-rejection'
        observer=MonoObserver();accepted=[];doc=json.loads((fixture/'inputs.json').read_text())
        with recorded_mount(fixture_mount(doc)):
            for row in doc['frames']:
                accepted.extend(observer.process(cv2.imread(str(fixture/row['image'])),row['meta'],np.array(row['estimated_pose'])).tolist())
        points=np.asarray(accepted)
        self.assertGreater(len(points),0)
        # Scene locations are assertions only, never observer inputs.
        self.assertFalse(np.any(np.linalg.norm(points[:,:2],axis=1)<.35),'False obstacle in the clear starting area')
        self.assertTrue(np.any(np.linalg.norm(points[:,:2]-[-1.29,-.18],axis=1)<.20),'Slipper must remain detectable')

    def test_camera_disagreement_stops_for_lidar_relocalization(self):
        with tempfile.TemporaryDirectory() as d:
            b=LidarBrain(d);b.start_mapping(0);b.attitude_initialized=True;b.localizer.uncertainty=.025
            b.camera.bad_frames=3;b.camera.process=lambda *args:np.empty((0,3))
            b.observe(np.zeros((360,640,3),np.uint8),None,dict(frame=1,sim_time_s=1))
            self.assertEqual(b.mode,'mapping');self.assertEqual(b.phase,'recover_localization')
            self.assertEqual(b.localizer.relocalization_hits,0);self.assertEqual(b.path,[])
            self.assertEqual(b.command(dict(sim_time_s=1.1,armed=True)),(0.,0.))

    def test_downward_ranges_recover_camera_tilt_without_body_truth(self):
        roll,pitch=.025,-.04;height=BASE_Z+.005
        # Cliff ToF layout and origin depth below base_link (BASE_Z-CLIFF_Z) from robot-parameters.json.
        xy=np.array(CLIFF_XY);self.assertEqual(xy.shape,(4,2))
        # Independently intersect the four body-down rays with a horizontal floor.
        up=np.array([-np.sin(pitch),np.sin(roll)*np.cos(pitch),np.cos(roll)*np.cos(pitch)])
        ranges=(height+xy@up[:2])/up[2]-(BASE_Z-CLIFF_Z)
        recovered=downward_floor_pose(ranges)
        np.testing.assert_allclose(recovered,[roll,pitch,.005],atol=1e-10)
        ranges[0]+=.016
        self.assertIsNone(downward_floor_pose(ranges),'A step must not become a trusted floor plane')

    def test_recorded_house_slipper_detected_before_contact(self):
        fixture=Path(__file__).parent/'fixtures/slipper-approach'
        rows=json.loads((fixture/'inputs.json').read_text())['frames'];observer=MonoObserver();hits=[]
        with recorded_mount():
            for row in rows:
                self.assertEqual(set(row),{'image','meta','estimated_pose'})
                points=observer.process(cv2.imread(str(fixture/row['image'])),row['meta'],np.array(row['estimated_pose']))
                self.assertEqual(len(points),len(observer.status['obstacle_pixels']))
                # Known object location is evaluation only; it never enters process.
                if len(points) and np.any(np.linalg.norm(points[:,:2]-[-1.29,-.18],axis=1)<.19):
                    hits.append(np.linalg.norm(np.array(row['estimated_pose'][:2])-[-1.29,-.18]))
        self.assertTrue(hits,'No camera evidence at the slipper in recorded house images')
        self.assertGreater(hits[0],.60,'Obstacle must be mapped well before the robot reaches it')

    def test_images_to_confirmed_low_obstacle_without_depth_input(self):
        rng=np.random.default_rng(46)
        floor=np.c_[rng.uniform(.32,1.25,600),rng.uniform(-.6,.6,600),np.zeros(600)]
        toy=np.c_[rng.uniform(.52,.62,50),rng.uniform(-.13,-.04,50),np.full(50,.06)]
        # A solid 6 cm block shows the camera its front face as well as its top. From the 10.75 cm low-profile mount the
        # top alone is seen 6 deg from edge-on and falls in the image rows of the floor ~1.1 m away (no trackable
        # corners: 0 of 8 noise seeds confirm it); the textured front face (x = .52) restores a realistic toy.
        toy=np.vstack((toy,np.c_[np.full(30,.52),rng.uniform(-.13,-.04,30),rng.uniform(0.,.06,30)]))
        points=np.vstack((floor,toy));colors=rng.integers(20,150,len(points));observer=MonoObserver();accepted=[]
        for i in range(16):
            x=i*.01;depth=points[:,0]-x-CAMERA_MOUNT[0]
            # Independent pinhole rasterizer: no depth/labels enter process().
            uv=np.c_[319.5-355*(points[:,1]-CAMERA_MOUNT[1])/depth,179.5+355*(CAMERA_MOUNT[2]-points[:,2])/depth]
            im=np.full((360,640,3),230,np.uint8)
            for (u,v),z,col in zip(uv,depth,colors):
                if z>.1 and 3<u<637 and 3<v<357:cv2.rectangle(im,(int(u)-2,int(v)-2),(int(u)+2,int(v)+2),(int(col),)*3,-1)
            accepted.extend(observer.process(im,dict(fx=355,fy=355,cx=319.5,cy=179.5,sim_time_s=i*.2),np.array([x,0,0])).tolist())
        self.assertGreaterEqual(len(accepted),4)
        self.assertTrue(np.all(np.abs(np.array(accepted)[:,2]-.06)<.025))
    def test_cliff_ranges_correct_small_body_height_changes(self):
        with tempfile.TemporaryDirectory() as d:
            b=LidarBrain(d)
            for i in range(8):b.predict(dict(sim_time_s=i*.03,armed=False,sensors=dict(specific_force_m_s2=[0,0,9.8],gyro_rad_s=[0,0,0],cliff_m=[CLIFF_Z+.005]*4,left_ticks=0,right_ticks=0,encoder_cpr=4000)))
            self.assertAlmostEqual(b.camera_pose(.21)[5],.005)
    def test_startup_acceleration_cannot_initialize_camera_tilt(self):
        with tempfile.TemporaryDirectory() as d:
            b=LidarBrain(d)
            def sample(t,force):return dict(sim_time_s=t,armed=False,sensors=dict(specific_force_m_s2=force,gyro_rad_s=[0,0,0],left_ticks=0,right_ticks=0,encoder_cpr=4000))
            b.predict(sample(0,[50,0,-20]));self.assertFalse(b.attitude_initialized)
            for i in range(1,8):b.predict(sample(i*.03,[0,0,9.8]))
            self.assertTrue(b.attitude_initialized);self.assertLess(b.tilt_rad,.001)
    def test_camera_pose_interpolates_exposure_instead_of_using_last_sample(self):
        with tempfile.TemporaryDirectory() as d:
            b=LidarBrain(d);b.localizer.history.extend([(1,np.array([0.,0,3.13])),(1.1,np.array([.01,0,-3.13]))])
            p=b.camera_pose(1.05);self.assertAlmostEqual(p[0],.005);self.assertAlmostEqual(p[2],np.pi)
    def test_camera_adapter_waits_for_motion_samples_through_exposure(self):
        import autonomy_service as service
        old=service.ROOT
        with tempfile.TemporaryDirectory() as d:
            service.ROOT=Path(d);folder=Path(d)/'Saved/Stereo/test';folder.mkdir(parents=True)
            (folder.parent/'latest-session.json').write_text('{"directory":"test"}')
            (folder/'latest.json').write_text(json.dumps(dict(frame=1,sim_time_s=1,fx=355,fy=355,cx=319.5,cy=179.5,baseline_m=0,left='left.png')))
            cv2.imwrite(str(folder/'left.png'),np.zeros((20,20,3),np.uint8))
            app=SimpleNamespace(brain=LidarBrain(d),last_pair=None,last_predict=.98)
            try:
                self.assertIsNone(service.Application.read_pair(app,{'sim_time_s':1.02}))
                app.last_predict=1.04;self.assertIsNotNone(service.Application.read_pair(app,{'sim_time_s':1.04}))
            finally:service.ROOT=old
    def test_imu_camera_tilt_keeps_floor_at_floor_height(self):
        rng=np.random.default_rng(44);points=np.c_[rng.uniform(.5,1.1,100),rng.uniform(-.3,.3,100),np.zeros(100)]
        pa=np.array([0.,0,0,.015,-.025]);pb=np.array([.14,0,.02,-.015,.025])
        a,_=project(points,pa,K);b,_=project(points,pb,K);world,valid,_=reconstruct(a,b,pa,pb,K)
        self.assertGreater(valid.sum(),70);self.assertLess(np.abs(world[valid,2]).max(),1e-8)
    def test_floor_motion_rejects_floor_and_keeps_raised_toy(self):
        rng=np.random.default_rng(8)
        floor=np.c_[rng.uniform(.45,1.,260),rng.uniform(-.4,.4,260),np.zeros(260)]
        toy=np.c_[rng.uniform(.5,.65,30),rng.uniform(-.2,.2,30),np.full(30,.06)]
        points=np.vstack((floor,toy));pa=np.zeros(3);pb=np.array([.12,.01,.02])
        a,_=project(points,pa,K);b,_=project(points,pb,K)
        raised,state=above_floor_evidence(a,b,pa,pb,K)
        self.assertEqual(state,'floor motion verified');self.assertFalse(raised[:260].any());self.assertGreater(raised[260:].sum(),20)
    def test_unreachable_known_area_cannot_be_reported_complete(self):
        with tempfile.TemporaryDirectory() as d:
            # Known, connected free space the robot has not visited blocks completion
            # even when no route or viewpoint can currently be produced.
            b=LidarBrain(d);b.start_mapping(0);b.grid.logodds[140:160,140:220]=-5;b.grid.seen[140:160,140:220]=5
            b.localizer.pose=np.array([0.,-.5,0]);b.visits=[np.array([0.,-.5])];b.grid.frontiers=lambda *args:[];b.grid.path=lambda *args:[];b.no_frontier_scans=3
            b.plan(1);self.assertEqual(b.mode,'idle');self.assertFalse(b.complete);self.assertIn('unvisited',b.message)
    def test_disconnected_pocket_does_not_block_completion(self):
        with tempfile.TemporaryDirectory() as d:
            # A free pocket with no connection to the robot (e.g. behind furniture) is reported, not chased forever.
            b=LidarBrain(d);b.start_mapping(0);b.grid.logodds[:]=5;b.grid.seen[:]=5;b.grid.logodds[140:181,140:181]=-5;b.grid.logodds[200:230,200:230]=-5
            b.localizer.pose=np.zeros(3);b.visits=[np.array([x,y]) for x in np.arange(-.8,.81,.2) for y in np.arange(-.8,.81,.2)];b.no_frontier_scans=3
            b.plan(1);self.assertEqual(b.mode,'idle');self.assertTrue(b.complete,b.message)
    def test_metric_multiview_reconstructs_low_objects_and_floor(self):
        rng=np.random.default_rng(23)
        points=np.c_[rng.uniform(.5,1.4,180),rng.uniform(-.3,.3,180),rng.choice([0.,.05,.10,-.08],180)]
        pa=np.array([0.,0,0]);pb=np.array([.12,.01,.03])
        a,_=project(points,pa,K);b,_=project(points,pb,K)
        world,valid,sigma=reconstruct(a,b,pa,pb,K)
        self.assertGreater(valid.sum(),120)
        self.assertLess(np.max(np.linalg.norm(world[valid]-points[valid],axis=1)),1e-8)
        low=valid&(world[:,2]>.03+2*sigma)&(world[:,2]<.24)
        self.assertGreater(low.sum(),30)
        self.assertTrue(np.all(points[low,2]>.03))
    def test_no_depth_from_identical_camera_positions(self):
        points=np.array([[.5,-.1,.05],[.5,.1,.05],[1,0,0]])
        uv,_=project(points,np.zeros(3),K)
        _,valid,_=reconstruct(uv,uv,np.zeros(3),np.zeros(3),K)
        self.assertFalse(valid.any())
    def test_motion_check_accepts_geometry_and_rejects_vertical_image_slip(self):
        rng=np.random.default_rng(11)
        points=np.c_[rng.uniform(.6,2,160),rng.uniform(-.7,.7,160),rng.uniform(0,.6,160)]
        pa=np.array([0.,0,0]);pb=np.array([.06,0,.08])
        a,_=project(points,pa,K);b,_=project(points,pb,K)
        self.assertEqual(motion_agreement(a,b,pa,pb,K)['state'],'consistent')
        self.assertEqual(motion_agreement(a,b+[0,20],pa,pb,K)['state'],'disagreement')
    def test_blank_image_is_unknown_and_creates_no_obstacle(self):
        m=MonoObserver();image=np.full((360,640,3),225,np.uint8)
        for i in range(8):
            points=m.process(image,dict(sim_time_s=i*.2,fx=355,fy=355,cx=319.5,cy=179.5),np.array([i*.02,0,0]))
            self.assertEqual(len(points),0)
        self.assertEqual(m.status['state'],'insufficient features')
    def test_lidar_cannot_erase_low_camera_obstacle_and_layer_survives_save(self):
        g=LidarGrid();g.add_camera_obstacles(np.array([[.5,0,.05]]));cell=g.cell([.5,0])
        for x in np.linspace(0,.3,12):
            angles=np.linspace(-.8,.8,300);g.integrate_scan(np.c_[2*np.cos(angles),2*np.sin(angles)],np.array([x,0,0]))
        self.assertGreater(g.logodds[cell],.8)
        with tempfile.TemporaryDirectory() as d:
            g.save(Path(d));loaded=LidarGrid();loaded.load(Path(d));self.assertTrue(loaded.camera_obstacles[cell]);self.assertFalse(loaded.masks()[3][cell])
    def test_camera_obstacle_reaches_local_motion_guard(self):
        with tempfile.TemporaryDirectory() as d:
            b=LidarBrain(d);b.current_cloud=np.empty((0,2));b.grid.add_camera_obstacles(np.array([[.25,0,.05]]))
            self.assertTrue(b.translation_blocked(b.obstacle_points(),.1,0))
    def test_sustained_sensor_loss_returns_to_idle(self):
        with tempfile.TemporaryDirectory() as d:
            b=LidarBrain(d);b.start_mapping(0);b.frame=SimpleNamespace(stamp=.1)
            self.assertEqual(b.command({'sim_time_s':3.2,'armed':True}),(0,0));self.assertEqual(b.mode,'idle')
            b.start_mapping(4);b.frame=SimpleNamespace(stamp=7.3)
            self.assertEqual(b.command({'sim_time_s':7.3,'armed':True}),(0,0));self.assertEqual(b.mode,'idle');self.assertIn('Camera',b.message)

if __name__=='__main__':unittest.main(verbosity=2)
