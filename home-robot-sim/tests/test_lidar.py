import unittest,sys,tempfile,json,math
from pathlib import Path
import numpy as np,cv2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'controller'))
from autonomy.lidar import LidarLocalizer,LidarGrid,scan_points,transform,inverse,relocalization_failure_message
from autonomy.lidar_brain import LidarBrain
from autonomy.geometry import LIDAR_MOUNT,CLIFF_Z

def room():
    x=np.linspace(-2,3,300);y=np.linspace(-1.8,2.5,250)
    return np.vstack([np.c_[x,np.full_like(x,-1.8)],np.c_[x,np.full_like(x,2.5)],np.c_[np.full_like(y,-2),y],np.c_[np.full_like(y,3),y],np.c_[np.linspace(.4,1.2,90),np.full(90,.7)],np.c_[np.full(100,.4),np.linspace(.2,.7,100)]])
class LidarTests(unittest.TestCase):
    def test_scan_match_corrects_biased_odometry_with_noise_and_dropout(self):
        loc=LidarLocalizer();loc.points=room();loc.rebuild();actual=np.array([.35,-.23,.18]);p=inverse(room(),actual);rng=np.random.default_rng(7)
        p=p[rng.random(len(p))>.15];p+=rng.normal(0,.005,p.shape)
        fit,q=loc.fit(p,actual+[.08,-.06,.06]);self.assertIsNotNone(fit);self.assertLess(np.linalg.norm(fit[:2]-actual[:2]),.015);self.assertLess(abs(fit[2]-actual[2]),.01)
    def test_single_featureless_wall_rejected_as_degenerate(self):
        loc=LidarLocalizer();loc.points=np.c_[np.linspace(-3,3,400),np.full(400,2.)];loc.rebuild();pose,q=loc.fit(loc.points,np.zeros(3));self.assertIsNone(pose);self.assertTrue(q.get('degenerate'))
    def test_invalid_ranges_never_clear_unknown(self):
        scan=dict(ranges_m=[-1,0,float('nan'),9],range_min_m=.1,range_max_m=8,angle_min_rad=0,angle_increment_rad=.1,sensor_in_base_m=list(LIDAR_MOUNT),times_s=[1]*4,sim_time_s=1)
        p=scan_points(scan);g=LidarGrid();g.integrate_scan(p,np.zeros(3));self.assertEqual(len(p),0);self.assertEqual(g.seen.sum(),0)
    def test_encoder_based_deskew_preserves_static_surface(self):
        # Robot rotates 0.1 rad during a sweep; each beam observes the same plane.
        angles=np.linspace(-.5,.5,100);times=np.linspace(0,.16,100);yaw=times/.16*.1;r=2/np.cos(angles+yaw)
        scan=dict(ranges_m=r,range_min_m=.1,range_max_m=8,angle_min_rad=angles[0],angle_increment_rad=angles[1]-angles[0],sensor_in_base_m=[0,0,.19],times_s=times,sim_time_s=.16)
        p=scan_points(scan,[(0,np.zeros(3)),(.16,np.array([0,0,.1]))]);world=transform(p,[0,0,.1]);self.assertLess(np.max(np.abs(world[:,0]-2)),1e-5)
    def test_go_to_loads_map_but_does_not_assume_start_pose(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);folder=root/'Saved/Maps/test';folder.mkdir(parents=True);loc=LidarLocalizer();loc.points=room();loc.keyframes=[{'pose':np.array([0.,0,0])},{'pose':np.array([.5,0,0])}];loc.save(folder);g=LidarGrid();g.logodds[145:176,145:176]=-5;g.seen[145:176,145:176]=4;g.save(folder);(folder.parent/'latest-lidar-map.json').write_text('{"directory":"test"}');b=LidarBrain(root);b.begin_go_to([0,0],1);self.assertEqual(b.phase,'relocalizing');self.assertGreater(b.localizer.uncertainty,.15)
    def test_global_relocalization_different_heading(self):
        loc=LidarLocalizer();loc.points=room();loc.keyframes=[{'pose':np.zeros(3)},{'pose':np.array([1.,0,0])}];loc.rebuild();actual=np.array([.15,-.08,1.4]);p=inverse(room(),actual);fit,q=loc.global_match(p);self.assertIsNotNone(fit);self.assertLess(np.linalg.norm(fit[:2]-actual[:2]),.02);self.assertLess(abs(fit[2]-actual[2]),.01)
    def test_contact_is_recorded_and_mapping_continues_without_new_map_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);b.start_mapping(0);sensors=dict(bumper=True,left_ticks=0,right_ticks=0,encoder_cpr=4741.44,gyro_rad_s=[0,0,0],specific_force_m_s2=[0,0,9.8],cliff_m=[CLIFF_Z]*4)
            b.predict(dict(sim_time_s=.1,fault='bumper',armed=False,sensors=sensors));self.assertEqual(b.mode,'mapping');self.assertEqual(b.phase,'contact');self.assertEqual(len(b.contacts),1)
            self.assertGreater(b.grid.camera_obstacles.sum(),0);self.assertEqual(b.grid.updates,0)
    def test_cliff_and_tilt_still_stop_mapping(self):
        for fault in ['cliff_or_invalid','tilt']:
            with self.subTest(fault=fault),tempfile.TemporaryDirectory() as tmp:
                b=LidarBrain(tmp);b.start_mapping(0);b.predict(dict(sim_time_s=.1,fault=fault,armed=False,sensors={'bumper':False}));self.assertEqual(b.mode,'idle');self.assertGreaterEqual(b.localizer.uncertainty,.5)
    def test_stale_scan_stops_motion(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);b.start_mapping(0);b.motor_seen_armed=True;from types import SimpleNamespace;b.frame=SimpleNamespace(stamp=1);self.assertEqual(b.command({'sim_time_s':2,'armed':True}),(0,0))
LOOKALIKE=Path(__file__).resolve().parent/'fixtures/relocalization-lookalike'
def lookalike_map():
    loc=LidarLocalizer();loc.load(LOOKALIKE);return loc,np.load(LOOKALIKE/'scans.npz')
def near(a,b,xy=.10,yaw=.10):
    return np.linalg.norm(np.asarray(a)[:2]-np.asarray(b)[:2])<xy and abs(math.atan2(math.sin(a[2]-b[2]),math.cos(a[2]-b[2])))<yaw
class RelocalizationTests(unittest.TestCase):
    """2026-10-04 dashboard session: a Go-to relocalized 4.3 m from the robot (68 % look-alike), then stayed lost."""
    def test_lookalike_passes_scan_fit_but_whole_map_search_finds_true_pose(self):
        loc,d=lookalike_map()
        # The look-alike explains about 68 % of the scan: at the tracking threshold (.68), so a plain
        # fit() accepts or rejects it depending on FLANN's randomized tree. Whole-map search must not.
        trap,q=loc.fit(d['goto_start_scan'],d['lookalike_pose'],.35,30)
        self.assertGreater(q['fraction'],.62,'fixture must still contain the look-alike')
        for name in ('goto_start','stopped'):
            pose,q=loc.global_match(d[name+'_scan'])
            self.assertIsNotNone(pose,f'{name}: {q}');self.assertTrue(near(pose,d[name+'_pose']),f'{name}: {pose} vs {d[name+"_pose"]}')
            self.assertGreaterEqual(q['fraction'],.80)
    def test_whole_map_search_reports_no_match_for_a_foreign_scan(self):
        loc,_=lookalike_map();pose,q=loc.global_match(inverse(room(),np.array([.2,.1,.3])).astype(np.float32))
        self.assertIsNone(pose);self.assertTrue(q.get('no_match') or q.get('ambiguous'),q)
        self.assertIn('saved map',relocalization_failure_message(q))
    def test_relocalization_starts_from_the_verified_pose_after_idle(self):
        loc,d=lookalike_map();old,_=lookalike_map();truth=d['goto_start_pose']
        old.pose[:]=truth+[.03,-.02,.02];old.last_good_stamp=10.;old.history.append((10.,old.pose.copy()));old.unverified_travel_m=0.
        self.assertTrue(loc.adopt_verified_pose(old))
        def no_global(points):raise AssertionError('whole-map search must not run while a verified pose is available')
        loc.global_match=no_global
        results=[loc.update_scan(d['goto_start_scan'],600.+k,relocalizing=True) for k in range(3)]
        self.assertEqual([r[0] for r in results],[False,False,True]);self.assertTrue(near(results[-1][1],truth,.03,.02))
    def test_verified_pose_not_adopted_from_another_map_or_after_travel(self):
        loc,_=lookalike_map();old,_=lookalike_map();old.last_good_stamp=10.
        old.unverified_travel_m=5.;self.assertFalse(loc.adopt_verified_pose(old))
        old.unverified_travel_m=0.;old.map_name='other';self.assertFalse(loc.adopt_verified_pose(old))
        self.assertFalse(loc.adopt_verified_pose(LidarLocalizer()))
    def test_go_to_after_arrival_keeps_the_verified_pose(self):
        import shutil
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);folder=root/'Saved/Maps/session';shutil.copytree(LOOKALIKE,folder);(folder.parent/'latest-lidar-map.json').write_text('{"directory":"session"}')
            b=LidarBrain(root);old=LidarLocalizer();old.load(folder);truth=np.load(folder/'scans.npz')['goto_start_pose']
            old.pose[:]=truth;old.last_good_stamp=50.;old.history.append((50.,truth.copy()));b.localizer=old
            b.begin_go_to([-2.35,-1.45],60.)
            self.assertIsNot(b.localizer,old);self.assertEqual(b.phase,'relocalizing');self.assertTrue(near(b.localizer.pose,truth,.001,.001))

TVTILT=Path(__file__).resolve().parent/'fixtures/tv-console-tilt/scans.npz'
class TvConsoleTiltTests(unittest.TestCase):
    """Task R (LIO-153). Task Q's Unreal trial 2 (full-mapping-20261013T010812) lost its position 8.8 cm by the TV console while
    the robot turned in place tilted 1.5-2.6 deg nose-down on the living rug's edge: the console's underside (124.6 mm) is 2.9 mm
    below the lowered LiDAR plane, so the tilted beams passed under it and returned the partition 0.5 m behind instead of its
    face. Those returns, about a fifth of each scan, pulled every fit ~2 cm east: in this fixture's loop 9.4 cm while tilted, back to
    about 1 cm once level. (In Unreal the offset then stayed 6-7.5 cm until 240 s; why is not known.) The fixture
    (tests/cut_tv_console_fixture.py) re-casts those scans from the true pose and tilt. Not fixed (backlog, LIO-110): leaving such
    returns out of the fit judged from the prediction locked Unreal trial 2 of task R onto a pose 0.21 m off after the 10 mm
    threshold (test below), and judged from the full fit's result it ended this fixture 7.7 cm off."""
    def setUp(self):
        cv2.setRNGSeed(0)      # the scan matcher's KD-tree index is randomized
    def run_fixture(self):
        d=np.load(TVTILT);loc=LidarLocalizer();loc.points=d['map_points'];loc.rebuild();loc.last_key_pose=d['last_key_pose'].copy()
        est=d['start_estimate'].copy();prev=d['start_true'].copy();errors=[];k=0
        for stamp,truth,n in zip(d['stamps'],d['true_poses'],d['counts']):
            points=d['points'][k:k+n];k+=n
            # Perfect odometry: the prediction is the previous estimate moved by the true motion.
            c,s=math.cos(prev[2]),math.sin(prev[2]);dx,dy=truth[:2]-prev[:2];delta=np.array([c*dx+s*dy,-s*dx+c*dy,math.remainder(truth[2]-prev[2],2*math.pi)])
            c,s=math.cos(est[2]),math.sin(est[2]);pred=np.array([est[0]+c*delta[0]-s*delta[1],est[1]+s*delta[0]+c*delta[1],est[2]+delta[2]]);prev=truth
            loc.pose=pred.copy();loc.history.clear();loc.history.append((float(stamp),pred.copy()))
            _,est=loc.update_scan(points,float(stamp),mapping=True);est=np.asarray(est,float).copy();errors.append(np.linalg.norm(est[:2]-truth[:2]))
        return np.array(errors)
    @unittest.expectedFailure
    def test_tilted_scans_passing_under_low_furniture_do_not_move_the_estimate(self):
        e=self.run_fixture();self.assertEqual(len(e),53)
        self.assertLess(e.max(),.03,'tracking error %.3f m while tilted next to the TV console'%e.max())
    def test_an_estimate_far_off_is_refused_not_locked_onto_a_false_pose(self):
        # Task R's Unreal trial 2 (full-mapping-20261013T023710): after the 10 mm threshold at 950.5 s the estimate was 0.39 m off.
        # A first fix for the class above judged "seen through" from that prediction, dropped the very returns that would correct
        # it, and tracking accepted poses 0.20-0.22 m off for the last 18 s. With all returns the true fit is more than .20 m from
        # such a prediction, so it is refused and the robot relocalizes (task Q's trial 4 recovered so within 2 s).
        d=np.load(Path(__file__).resolve().parent/'fixtures/threshold-offset/scans.npz');loc=LidarLocalizer();loc.points=d['map_points'];loc.rebuild();k=0
        for stamp,truth,n in zip(d['stamps'],d['true_poses'],d['counts']):
            points=d['points'][k:k+n];k+=n
            for off in ([.39,0,0],[.30,0,0]):
                pred=truth+off;loc.pose=pred.copy();loc.history.clear();loc.history.append((float(stamp),pred.copy()))
                good,pose=loc.update_scan(points,float(stamp))
                self.assertTrue(not good or np.linalg.norm(pose[:2]-truth[:2])<.05,(float(stamp),off,good,np.linalg.norm(pose[:2]-truth[:2])))

if __name__=='__main__':unittest.main(verbosity=2)
