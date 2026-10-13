"""Behavior tests with analytical geometry; not replacements for Unreal runs."""
import sys,unittest,tempfile,math,json
from pathlib import Path
import numpy as np,cv2
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'controller'))
from autonomy.features import Features
from autonomy.stereo import Stereo,optical_to_world,camera_transform,CAMERA_OFFSET
from autonomy import geometry
from autonomy.localization import Localizer
from autonomy.grid import Grid
from autonomy.brain import Brain
class RobotTests(unittest.TestCase):
    def test_metric_stereo_and_inconsistent_pair(self):
        rng=np.random.default_rng(4);left=rng.integers(0,255,(360,640,3),np.uint8);right=np.zeros_like(left);right[:,:-16]=left[:,16:]
        m=dict(fx=381.36,fy=381.36,cx=319.5,cy=179.5,baseline_m=.062,frame=1,sim_time_s=0);s=Stereo(Features());f=s.process(left,right,m)
        self.assertGreater(f.valid.mean(),.3);self.assertAlmostEqual(float(np.median(f.depth[f.valid])),381.36*.062/16,delta=.02)
        bad=s.process(left,left,m);self.assertLess(bad.valid.mean(),.01)
    def test_pnp_recovers_metric_pose_with_outliers(self):
        rng=np.random.default_rng(5);optical=np.column_stack((rng.uniform(-1,1,90),rng.uniform(-.8,.8,90),rng.uniform(1.5,3.2,90)));pose=np.array([1.2,-.6,.41]);world=optical_to_world(optical,pose);K=np.array([[380.,0,320],[0,380,180],[0,0,1]])
        pixels=optical[:,:2]/optical[:,2,None]*380+[320,180];pixels[:12]=rng.uniform([0,0],[640,360],(12,2));got,n=Localizer.pnp(world,pixels,K)
        self.assertIsNotNone(got);self.assertGreater(n,60);np.testing.assert_allclose(got,pose,atol=.015)
    def test_gyro_heading_does_not_follow_slipping_wheel_yaw(self):
        l=Localizer(Features());s=dict(left_ticks=0,right_ticks=0,encoder_cpr=4741.44,gyro_rad_s=[0,0,0]);l.predict(0,s);s.update(right_ticks=3000);l.predict(.1,s);self.assertAlmostEqual(l.pose[2],0)
    def test_unknown_cells_are_not_traversable(self):
        g=Grid(size=80);self.assertEqual(g.path([0,0],[1,0]),[])
    def test_path_goes_around_inflated_wall(self):
        g=Grid(size=100);g.logodds[:]=-4;g.seen[:]=5;y,x=g.cell([0,0]);g.logodds[y-16:y+16,x]=4
        path=g.path([-1,0],[1,0]);self.assertTrue(path);self.assertGreater(max(abs(p[1]) for p in path),.95)
    def test_too_narrow_gap_is_rejected(self):
        g=Grid(size=100);g.logodds[:]=4;y,x=g.cell([0,0]);g.logodds[y-2:y+3,:]=-4
        self.assertEqual(g.path([-1,0],[1,0]),[])
    def test_reflected_points_below_floor_do_not_make_obstacles(self):
        g=Grid();g.integrate(np.array([[0,.4,1],[.1,.35,1.2],[-.1,.45,1.1]]),np.zeros(3));self.assertEqual(int((g.logodds>.8).sum()),0)
    def test_idle_commands_zero_and_stale_frames_stop(self):
        with tempfile.TemporaryDirectory() as t:
            b=Brain(t);state=dict(sim_time_s=1,sensors={},fault='none',armed=True);self.assertEqual(b.command(state),(0.,0.));b.start_mapping(1);self.assertEqual(b.command(state),(0.,0.))
    def test_fault_returns_to_idle_without_arrival(self):
        with tempfile.TemporaryDirectory() as t:
            b=Brain(t);b.mode='go_to';b.phase='follow';self.assertEqual(b.command(dict(sim_time_s=4,fault='cliff_or_invalid')),(0.,0.));self.assertEqual(b.mode,'idle');self.assertNotIn('Arrived',b.message)
    def test_save_load_landmarks_do_not_store_simulator_position(self):
        with tempfile.TemporaryDirectory() as t:
            l=Localizer(Features());l.keyframes=[dict(pose=np.zeros(3),world_points=np.ones((20,3),np.float32),features=dict(uv=np.ones((20,2),np.float32),descriptors=np.zeros((20,32),np.uint8),size=np.array([640,360],np.float32)))];l.save(Path(t));other=Localizer(Features());other.load(Path(t));self.assertEqual(len(other.keyframes),1);self.assertEqual(other.tracking,'unlocalized')
    def test_arrival_correction_replans_if_position_moves_outside_goal(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as t:
            b=Brain(t);b.mode='go_to';b.phase='confirm_arrival';b.goal=np.array([0.,0.]);b.localizer.pose=np.array([.14,0.,0.]);b.localizer.last_visual_time=3.;b.frame=SimpleNamespace(stamp=3.)
            self.assertEqual(b.command(dict(sim_time_s=3.,fault='none')),(0.,0.));self.assertEqual(b.phase,'plan');self.assertNotIn('Arrived',b.message)
    def test_arrival_needs_fresh_localization(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as t:
            b=Brain(t);b.mode='go_to';b.phase='confirm_arrival';b.goal=np.zeros(2);b.localizer.last_visual_time=0.;b.frame=SimpleNamespace(stamp=3.)
            self.assertEqual(b.command(dict(sim_time_s=3.,fault='none')),(0.,0.));self.assertEqual(b.phase,'recover_localization')
    def test_unconfirmed_arrival_times_out_without_success(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as t:
            b=Brain(t);b.mode='go_to';b.phase='confirm_arrival';b.goal=np.zeros(2);b.localizer.last_visual_time=9.;b.frame=SimpleNamespace(stamp=9.)
            self.assertEqual(b.command(dict(sim_time_s=9.,fault='none')),(0.,0.));self.assertEqual(b.mode,'idle');self.assertNotIn('Arrived',b.message)
    def test_failed_frontier_cannot_be_reported_as_completed_mapping(self):
        with tempfile.TemporaryDirectory() as t:
            b=Brain(t);b.start_mapping(0);b.grid.logodds[130:190,130:190]=-5;b.grid.seen[130:190,130:190]=10
            b.grid.frontiers=lambda *args: [];b.no_frontier_scans=2;b.blacklist=[np.array([1.,1.])];b.plan(10)
            manifest=json.loads((b.map_folder/'manifest.json').read_text());self.assertFalse(manifest['complete']);self.assertEqual(b.mode,'idle');self.assertIn('partial',b.message)
    def test_recorded_magazine_allows_safe_in_place_turn(self):
        from types import SimpleNamespace
        # Copied from reports/full-mapping-20261003T160508/obstacle-diagnosis.json (stereo run, nearest point .335 m).
        recorded=ROOT/'tests/fixtures/magazine-turn/obstacle-diagnosis.json'
        xyz=np.array(json.loads(recorded.read_text())['near_points_robot_forward_left_height_m'])
        with tempfile.TemporaryDirectory() as t:
            b=Brain(t);b.mode='mapping';b.phase='follow';b.goal=np.array([0.,1.]);b.path=[np.array([0.,0.]),np.array([0.,.4]),b.goal];b.next_plan=100;b.frame=SimpleNamespace(stamp=1.);b.localizer.last_visual_time=1.
            b.current_cloud=np.column_stack((CAMERA_OFFSET[1]-xyz[:,1],CAMERA_OFFSET[2]-xyz[:,2],xyz[:,0]-CAMERA_OFFSET[0]))
            v,w=b.command(dict(sim_time_s=1.,fault='none',armed=True));self.assertEqual(v,0.);self.assertGreater(w,.1);self.assertEqual(b.phase,'follow')
    def test_forward_obstacle_requires_stop_before_stereo_blind_zone(self):
        points=np.array([[.33,-.015],[.33,-.005],[.33,.005],[.33,.015]])
        self.assertTrue(Brain.translation_blocked(points,.075,0.))
        self.assertFalse(Brain.translation_blocked(points+np.array([0.,.4]),.075,0.))
    def test_replanning_same_frontier_preserves_progress_deadline(self):
        with tempfile.TemporaryDirectory() as t:
            b=Brain(t);b.mode='mapping';b.goal=np.array([1.,0.]);b.last_progress=3.;b.progress_pose=np.zeros(3)
            route=[np.array([0.,0.]),np.array([.5,0.]),b.goal];b.grid.frontiers=lambda *args:[(1.,b.goal.copy(),route)]
            b.plan(10.);b.plan(17.);self.assertEqual(b.last_progress,3.)
    def test_stalled_replans_eventually_reject_target(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as t:
            b=Brain(t);b.mode='mapping';b.phase='follow';b.goal=np.array([1.,0.]);b.last_progress=1.;b.progress_pose=np.zeros(3);b.frame=SimpleNamespace(stamp=21.);b.localizer.last_visual_time=21.
            self.assertEqual(b.command(dict(sim_time_s=21.,fault='none',armed=True)),(0.,0.));self.assertEqual(b.phase,'scan');self.assertEqual(len(b.blacklist),1)
    def test_motor_disarm_cannot_leave_mapping_active(self):
        with tempfile.TemporaryDirectory() as t:
            b=Brain(t);b.start_mapping(0);b.motor_seen_armed=True
            self.assertEqual(b.command(dict(sim_time_s=4.,fault='motor_power_off',armed=False)),(0.,0.));self.assertEqual(b.mode,'idle');self.assertIn('Motor stopped',b.message)
    def test_frontier_does_not_target_unknown_across_wall(self):
        g=Grid(size=100);g.logodds[20:80,20:80]=-4;g.seen[20:80,20:80]=5
        g.logodds[19:81,19]=4;g.logodds[19:81,80]=4;g.logodds[19,19:81]=4;g.logodds[80,19:81]=4
        self.assertEqual(g.frontiers(np.zeros(3)),[])
    def test_rotation_in_place_keeps_base_link_fixed(self):
        # Flipped: the round robot's drive axle passes through base_link (drive_axle_x_m=0),
        # so a gyro-only turn no longer moves base_link on a 4.2 cm circle.
        self.assertEqual(geometry.AXLE_X,0.)
        l=Localizer(Features());state=dict(left_ticks=0,right_ticks=0,encoder_cpr=4741.44,gyro_rad_s=[0,0,1.])
        l.predict(0.,state)
        for k in range(1,11):l.predict(k*.05,state)
        np.testing.assert_allclose(l.pose,[0.,0.,.5],atol=1e-9)
        # Same for a wheel-measured spin (equal and opposite ticks), and for the
        # planner's motion model and the UDP client's odometry.
        l=Localizer(Features());l.predict(0.,dict(left_ticks=0,right_ticks=0,encoder_cpr=4741.44,gyro_rad_s=[0,0,1.]))
        l.predict(.5,dict(left_ticks=-500,right_ticks=500,encoder_cpr=4741.44,gyro_rad_s=[0,0,1.]))
        np.testing.assert_allclose(l.pose[:2],[0.,0.],atol=1e-9)
        from autonomy.lidar_brain import LidarBrain
        np.testing.assert_allclose(LidarBrain.predicted_centres(0.,.4,np.linspace(0,8,30)),0.,atol=1e-12)
        from robot_client import Odometry
        o=Odometry();o.update(dict(left_ticks=0,right_ticks=0,encoder_cpr=4741.44));o.update(dict(left_ticks=-900,right_ticks=900,encoder_cpr=4741.44))
        self.assertAlmostEqual(o.x,0.,places=12);self.assertAlmostEqual(o.y,0.,places=12);self.assertGreater(o.yaw,.4)
    def test_blank_view_stops_translation_and_reverses_scan(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as t:
            b=Brain(t);b.mode='mapping';b.phase='scan';b.weak_view_frames=3;b.frame=SimpleNamespace(stamp=1.);b.scan_direction=1.
            state=dict(sim_time_s=1.,fault='none',armed=True)
            self.assertEqual(b.command(state),(0.,0.));self.assertEqual(b.phase,'recover_view')
            self.assertEqual(b.command(state),(0.,-.14))
    def test_view_recovery_is_bounded_and_never_moves_forward(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as t:
            b=Brain(t);b.mode='mapping';b.phase='recover_view';b.frame=SimpleNamespace(stamp=56.)
            self.assertEqual(b.command(dict(sim_time_s=56.,fault='none',armed=True)),(0.,0.));self.assertEqual(b.mode,'idle')
    def test_feature_support_prefers_a_view_containing_landmarks(self):
        l=Localizer(Features());points=optical_to_world(np.array([[x,y,1.] for x in np.linspace(-.7,.7,25) for y in np.linspace(-.4,.4,20)]),np.zeros(3))
        l.keyframes=[dict(pose=np.zeros(3),world_points=points)]
        self.assertGreater(l.view_support(np.zeros(2),0.),.5);self.assertEqual(l.view_support(np.zeros(2),math.pi),0.)
    def test_contact_invalidates_pose_before_encoder_prediction(self):
        with tempfile.TemporaryDirectory() as t:
            b=Brain(t);b.mode='mapping';b.localizer.pose=np.array([1.,2.,.3]);before=b.localizer.pose.copy()
            b.predict(dict(sim_time_s=2.,fault='bumper',sensors=dict(bumper=True,left_ticks=5000,right_ticks=5000)))
            self.assertEqual(b.mode,'idle');self.assertGreaterEqual(b.localizer.uncertainty,.5);np.testing.assert_array_equal(b.localizer.pose,before)
    def test_repeated_visual_wheel_disagreement_invalidates_motion(self):
        l=Localizer(Features())
        for _ in range(3):l.check_motion(np.array([.07,0.,0.]),np.zeros(3))
        self.assertTrue(l.motion_inconsistent)
        l=Localizer(Features())
        for _ in range(10):l.check_motion(np.array([.012,0.,.01]),np.zeros(3))
        self.assertFalse(l.motion_inconsistent)
    def test_blocked_turn_does_not_trust_spinning_wheels(self):
        with tempfile.TemporaryDirectory() as t:
            b=Brain(t);b.mode='mapping';b.requested_motion=(0.,.16)
            for k in range(41):b.predict(dict(sim_time_s=k*.1,fault='none',sensors=dict(left_ticks=-k*15,right_ticks=k*15,encoder_cpr=4741.44,gyro_rad_s=[0,0,.002])))
            self.assertEqual(b.mode,'idle');self.assertGreaterEqual(b.localizer.uncertainty,.5);self.assertIn('slipping',b.message)
    def test_validated_camera_pose_does_not_inherit_prediction_bias(self):
        from types import SimpleNamespace
        l=Localizer(Features());l.previous=(None,np.zeros(3));l.recognize=lambda *args:(None,0)
        for k in range(1,31):
            last=l.previous[1].copy();l.pose=last+np.array([.012,0.,.025]);l.history.append((k,l.pose.copy()))
            l.temporal=lambda frame,previous=last:(previous+np.array([.01,0.,.02]),100)
            l.update(SimpleNamespace(stamp=k,index=1,points=np.ones((100,3))))
        np.testing.assert_allclose(l.pose,[.30,0.,.60],atol=1e-8)
    def test_view_recovery_requires_saved_landmarks_and_freezes_map(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as t:
            b=Brain(t);b.mode='mapping';b.phase='recover_view';b.localizer.keyframes=[{}]
            frame=SimpleNamespace(index=1,stamp=1.,points=np.ones((100,3)))
            b.stereo.process=lambda *args:frame;b.stereo.cloud=lambda frame:np.empty((0,3));calls=[]
            b.localizer.update=lambda frame,**kw:(calls.append(kw) or (False,np.zeros(3)))
            b.observe(None,None,{})
            self.assertTrue(calls[-1]['relocalizing']);self.assertFalse(calls[-1]['mapping']);self.assertEqual(b.grid.updates,0);self.assertEqual(b.phase,'recover_view')
            b.localizer.update=lambda frame,**kw:(True,np.zeros(3));b.observe(None,None,{})
            self.assertEqual(b.phase,'plan');self.assertEqual(b.grid.updates,0)
    def test_camera_led_is_on_while_a_task_drives_and_off_when_idle(self):
        # Task N (LIO-134): the bridge switches the camera LED with the task (capture active), fully on (no PWM bands).
        from autonomy_service import bridge_payload
        self.assertEqual(bridge_payload(.1,0.,False,True)['camera_led'],1.)
        self.assertEqual(bridge_payload(0.,0.,False,False)['camera_led'],0.)
        self.assertTrue(bridge_payload(0.,0.,True,True)['motor_power_on'])

    def test_brief_simulator_hitch_stops_motion_and_long_outage_disarms(self):
        from autonomy_service import command_freshness
        self.assertEqual(command_freshness(.1,.1),(False,False,'none'))
        stop,disarm,_=command_freshness(.1,1.2);self.assertTrue(stop);self.assertFalse(disarm)
        stop,disarm,_=command_freshness(.1,3.2);self.assertTrue(stop);self.assertTrue(disarm)
        stop,disarm,_=command_freshness(2.2,.1);self.assertTrue(stop);self.assertTrue(disarm)
    def test_recovery_returns_to_last_view_not_old_scan_direction(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as t:
            b=Brain(t);b.mode='mapping';b.phase='follow';b.scan_direction=-1.;b.requested_motion=(0.,.3);b.localizer.pose[2]=1.;b.last_good_yaw=.7;b.goal=np.array([1.,1.]);b.weak_view_frames=3;b.frame=SimpleNamespace(stamp=1.)
            state=dict(sim_time_s=1.,fault='none',armed=True);b.command(state)
            self.assertEqual(b.phase,'recover_view');self.assertAlmostEqual(b.recovery_target_yaw,.7);self.assertLess(b.command(state)[1],0);self.assertEqual(len(b.view_blacklist),1)
    def test_view_recovery_searches_room_after_short_return_attempt(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as t:
            b=Brain(t);b.mode='mapping';b.phase='recover_view';b.scan_direction=-1.;b.frame=SimpleNamespace(stamp=10.)
            v,w=b.command(dict(sim_time_s=10.,fault='none',armed=True));self.assertEqual(v,0.);self.assertLess(w,-.1);self.assertEqual(b.phase,'recover_view')
if __name__=='__main__':unittest.main(verbosity=2)
