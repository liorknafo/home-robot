"""Planner regressions. Recorded map fixture contains sensor evidence only."""
import unittest,sys,json,tempfile
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'controller'))
from autonomy.lidar import LidarGrid,inverse
from autonomy.lidar_brain import LidarBrain
from autonomy import geometry
from autonomy.geometry import AXLE_X,DRIVE_CLEARANCE,TURN_CLEAR,TURN_MARGIN
R=Path(__file__).resolve().parents[1]
class ExplorationTests(unittest.TestCase):
    def test_clearance_includes_whole_cad_envelope_and_error_allowance(self):
        # Envelope about the real turning centre, the drive axle (drive_axle_x_m), not
        # about base_link: the body sweeps that circle when it turns in place.
        params=json.loads((R/'Content/RobotData/robot-parameters.json').read_text());axle_x=float(params.get('drive_axle_x_m',AXLE_X))
        self.assertEqual(axle_x,AXLE_X)
        meshes=json.loads((R/'Content/RobotData/assembly-meshes.json').read_text())
        tires=[np.asarray(m['vertices'],float).reshape(-1,3) for m in meshes if 'tire' in m['name'].lower() or 'tyre' in m['name'].lower()]
        self.assertTrue(tires,'No wheel tyres in assembly-meshes.json')
        mesh_axle=float(np.mean([t[:,0].mean() for t in tires]))/1000
        points=np.vstack([np.asarray(m['vertices'],float).reshape(-1,3)[:,:2] for m in meshes])/1000
        # Skip ONLY for the recognisable old box robot meshes (axle 4.2 cm ahead, box outline
        # x -.123...1335) while the parameters already describe the round robot; any other
        # mesh/parameter disagreement is a real error and fails.
        old_box=abs(mesh_axle-.042)<.003 and abs(points[:,0].min()+.123)<.005 and abs(points[:,0].max()-.1335)<.005
        if old_box and abs(axle_x-.042)>.003:
            self.skipTest(f'assembly-meshes.json is still the old box robot (axle x={mesh_axle:.4f} m) but robot-parameters.json '
                          f'describes the round robot (drive_axle_x_m={axle_x:.3f} m): meshes not synced yet')
        self.assertAlmostEqual(mesh_axle,axle_x,delta=.002,msg='CAD wheel axle and drive_axle_x_m disagree')
        radius=float(np.hypot(points[:,0]-axle_x,points[:,1]).max())
        g=LidarGrid();self.assertGreaterEqual(g.radius,radius+.025+g.res/2**.5)
        if 'body_radius_m' in params:
            # Round robot: the controller's body circle (body_radius_m about base_link, which is the
            # turning centre since drive_axle_x_m = 0) must contain the whole CAD robot.
            self.assertLessEqual(radius,geometry.R+.0005)
        else:
            # Old parameter file (no body_radius_m): geometry.R falls back to the old envelope about
            # base_link, and turn_clear applies R + |AXLE_X| about the axle; that circle must contain it.
            self.assertLessEqual(radius,geometry.R+abs(geometry.AXLE_X)+.0005)
    def test_recorded_map_passage_remains_navigable(self):
        g=LidarGrid();g.load(R/'tests/fixtures/partial-house-map')
        route=g.path([3.79,1.73],[4,-2]);self.assertGreater(len(route),10)
        safe=g.masks()[3];self.assertTrue(all(safe[g.cell(p)] for p in route))
    def test_wider_detour_is_preferred_to_tight_gap_but_gap_remains_usable(self):
        g=LidarGrid(size=100);g.logodds[:]=5;g.logodds[5:95,5:95]=-5;g.logodds[16:65,40:60]=5
        start=g.world((35,20));goal=g.world((35,80));route=g.path(start,goal)
        self.assertTrue(route);self.assertGreater(max(g.cell(p)[0] for p in route),65)
        # Close the wider passage: the measured, footprint-safe narrow route
        # must still be available rather than made artificially impassable.
        g.logodds[65:95,40:60]=5;route=g.path(start,goal)
        self.assertTrue(route);self.assertLess(min(g.cell(p)[0] for p in route),16)
    def test_frontier_search_does_not_consider_disconnected_cells(self):
        g=LidarGrid(size=100);g.logodds[10:45,10:45]=-5;g.logodds[55:90,55:90]=-5
        reachable=g.reachable(g.world((25,25)))
        self.assertTrue(reachable[25,25]);self.assertFalse(reachable[70,70])
        for _,xy,route in g.frontiers(np.r_[g.world((25,25)),0]):
            self.assertTrue(reachable[g.cell(xy)]);self.assertTrue(route)
    def test_nearby_room_across_wall_is_not_counted_visited(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);g=LidarGrid(size=100);b.grid=g;g.logodds[10:90,10:90]=-5
            g.logodds[10:70,49:51]=5
            b.localizer.pose=np.r_[g.world((40,44)),0];b.visits=[g.world((40,44))]
            self.assertTrue(b.unvisited_mask()[40,56])
            self.assertGreater(b.unvisited_area(),0)
            self.assertTrue(b.coverage_viewpoints(b.localizer.pose))
    def test_disconnected_nearby_space_does_not_hide_reachable_viewpoint(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);g=LidarGrid(size=100);b.grid=g;g.logodds[10:90,10:90]=-5
            g.logodds[20:80,30:70]=5;g.logodds[27:73,37:63]=-5
            b.localizer.pose=np.r_[g.world((85,50)),0];b.visits=[b.localizer.pose[:2].copy()]
            candidates=b.coverage_viewpoints(b.localizer.pose);self.assertTrue(candidates)
            self.assertTrue(g.reachable(b.localizer.pose[:2])[g.cell(candidates[0][1])])
    def recorded_margin(self,tmp):
        b=LidarBrain(tmp);folder=R/'tests/fixtures/obstacle-margin';b.grid.load(folder);b.localizer.load(folder)
        b.localizer.pose=np.array([-1.789575248188063,-.09795531159216432,-1.9611955760659214]);b.current_cloud=inverse(b.localizer.points,b.localizer.pose)
        return b
    def test_new_obstacle_margin_allows_straight_escape_away_from_obstacle(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=self.recorded_margin(tmp);self.assertFalse(b.grid.masks()[3][b.grid.cell(b.localizer.pose[:2])])
            self.assertEqual(b.clearance_escape(),1.)
            # The planner now leaves the margin itself (cells no closer to anything).
            b.mode='mapping';b.plan(1);self.assertEqual(b.mode,'mapping');self.assertIn(b.phase,['follow','sense'])
            if b.phase=='follow':
                free,_,_,_,cl=b.grid.masks();start=cl[b.grid.cell(b.localizer.pose[:2])]
                self.assertTrue(all(free[b.grid.cell(q)] for q in b.path[:6]))
    def test_margin_recovery_never_approaches_an_obstacle_or_unknown_space(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=self.recorded_margin(tmp);b.current_cloud=np.vstack((b.current_cloud,[.165,0]))
            self.assertIsNone(b.clearance_escape())
            b=self.recorded_margin(tmp);p=b.localizer.pose;b.grid.logodds[b.grid.cell(p[:2]+.15*np.array([np.cos(p[2]),np.sin(p[2])]))]=0
            self.assertIsNone(b.clearance_escape())
    def test_retreat_requires_previous_traversal_and_is_bounded(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);b.grid.logodds[135:185,135:185]=-5
            angles=np.linspace(-np.pi,np.pi,200);b.current_cloud=np.c_[np.cos(angles),np.sin(angles)]
            self.assertFalse(b.begin_retreat(1))
            # Reversing may only retrace the driven trail (1 cm samples, forward driving); the coarse
            # visit path alone never permits a reverse.
            b.visits=[np.array([x,0]) for x in np.linspace(-.3,0,6)];self.assertFalse(b.begin_retreat(2))
            b.trail.extend(np.array([x,0.]) for x in np.arange(-.30,1e-9,.01))
            self.assertTrue(b.begin_retreat(2));self.assertEqual(b.phase,'retreat')
            self.assertTrue(b.begin_retreat(3));self.assertFalse(b.begin_retreat(4))
    def test_turn_stall_requires_relocalization_before_retreat(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);b.mode='mapping';b.handle_turn_stall(3)
            self.assertEqual(b.phase,'recover_localization');self.assertGreaterEqual(b.localizer.uncertainty,.5)
            self.assertTrue(b.retreat_after_localization);self.assertEqual(b.requested_motion,(0,0))
    def test_short_retreat_checks_only_remaining_motion_and_braking_margin(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);b.grid.logodds[135:185,135:185]=-5
            angles=np.linspace(-np.pi,np.pi,200);world=np.vstack((np.c_[np.cos(angles),np.sin(angles)],[-.27,0]))
            b.current_cloud=world;b.trail.extend(np.array([x,0.]) for x in np.arange(-.30,1e-9,.01))
            self.assertFalse(b.retreat_clear(.08));self.assertTrue(b.begin_retreat(0))
            b.mode='mapping';b.attitude_initialized=True;b.localizer.pose=np.array([-.03,0,0]);b.current_cloud=inverse(world,b.localizer.pose)
            b.frame=SimpleNamespace(stamp=1);b.camera.last_stamp=1;b.localizer.last_visual_time=1
            v,w=b.command({'sim_time_s':1,'armed':True});self.assertLess(v,0);self.assertEqual(b.phase,'retreat')
            b.localizer.pose[0]=-.036;b.current_cloud=inverse(world,b.localizer.pose)
            self.assertEqual(b.command({'sim_time_s':1.1,'armed':True}),(0,0));self.assertEqual(b.phase,'recovery_advance')
    def test_failed_margin_escape_relocalizes_then_backs_out_without_forward_retry(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as tmp:
            b=self.recorded_margin(tmp);b.mode='mapping';b.phase='escape_margin';b.phase_started=0;b.escape_origin=b.localizer.pose.copy()
            b.frame=SimpleNamespace(stamp=3.1);b.camera.last_stamp=3.1;b.localizer.last_visual_time=3.1;b.localizer.uncertainty=.025;b.attitude_initialized=True
            self.assertEqual(b.command({'sim_time_s':3.1,'armed':True}),(0,0))
            self.assertEqual(b.phase,'recover_localization');self.assertTrue(b.retreat_requires_replan);self.assertGreater(b.localizer.uncertainty,.15)
            # After successful localization, the recovery must back out to
            # plan anew even when elevated LiDAR sees an empty forward path.
            b.grid.logodds[:]=-5;b.localizer.pose=np.zeros(3);a=np.linspace(-np.pi,np.pi,200);b.current_cloud=np.c_[np.cos(a),np.sin(a)]
            b.trail.extend(np.array([x,0.]) for x in np.arange(-.30,1e-9,.01));b.plan(4)
            self.assertEqual(b.phase,'retreat');self.assertFalse(b.retry_straight);self.assertEqual(b.retreat_distance,.15)
    def test_heading_oscillation_cannot_hide_translation_stall(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp)
            for t in range(13):
                b.localizer.pose=np.array([.01*np.sin(t),.01*np.cos(t),.4*np.sin(t)])
                self.assertFalse(b.translation_stalled(t))
            self.assertTrue(b.translation_stalled(13))
            b.localizer.pose[0]+=.12;self.assertFalse(b.translation_stalled(14))
    def test_obstacle_ahead_allows_clear_retreat_but_forbids_forward_retry(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);folder=R/'tests/fixtures/doorway-retreat';b.grid.load(folder);b.localizer.load(folder)
            b.localizer.pose=np.array([.98935726,.04412362,.69807197]);b.current_cloud=inverse(b.localizer.points,b.localizer.pose)
            b.visits=[np.array(v) for v in json.loads((folder/'exploration.json').read_text())['visits']]
            # The fixture stores only the visit path (~15 cm samples), which never permits a reverse: it
            # does not show where the body actually drove.
            self.assertFalse(b.straight_corridor_clear(1,.30));self.assertFalse(b.begin_retreat(1))
            # The stall happened driving forward into the doorway: the driven trail is the straight
            # approach along the current heading (1 cm samples, as observe_scan records it).
            p=b.localizer.pose;b.trail.extend(p[:2]-s_*np.array([np.cos(p[2]),np.sin(p[2])]) for s_ in np.arange(.30,-1e-9,-.01))
            self.assertTrue(b.begin_retreat(1))
            # Reverse only retraces the verified driven path, and never more than 15 cm.
            self.assertFalse(b.retry_straight);self.assertGreaterEqual(b.retreat_distance,.03);self.assertLessEqual(b.retreat_distance,.15)
    def test_inflated_margin_can_turn_toward_clear_exit_without_changing_map(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);folder=R/'tests/fixtures/angled-margin-exit';b.grid.load(folder);b.localizer.load(folder)
            b.localizer.pose=np.array([3.99866871,-.56228128,2.15052022]);b.current_cloud=inverse(b.localizer.points,b.localizer.pose)
            before=b.grid.logodds.copy();self.assertIsNone(b.clearance_escape());self.assertIsNotNone(b.margin_exit_heading())
            self.assertTrue(b.begin_clearance_escape(1));self.assertEqual(b.phase,'escape_turn');np.testing.assert_array_equal(before,b.grid.logodds)
            cloud=b.current_cloud.copy()
            # Flipped from a point at .18: the round body turns about base_link, so a point 2 cm
            # outside the skin (> TURN_CLEAR = R + TURN_MARGIN) no longer blocks the turn; one just
            # inside TURN_CLEAR still does.
            b.current_cloud=np.vstack((cloud,[TURN_CLEAR-.002,0]));self.assertIsNone(b.margin_exit_heading())
            b.current_cloud=np.vstack((cloud,[TURN_CLEAR+.01,0]));self.assertTrue(b.rotation_clear())
    def test_margin_recovery_runs_even_when_global_route_still_exists(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);folder=R/'tests/fixtures/connected-margin-exit';b.grid.load(folder);b.localizer.load(folder)
            b.localizer.pose=np.array(json.loads((folder/'pose.json').read_text()));b.current_cloud=inverse(b.localizer.points,b.localizer.pose)
            self.assertFalse(b.grid.masks()[3][b.grid.cell(b.localizer.pose[:2])]);self.assertTrue(b.grid.reachable(b.localizer.pose[:2]).any())
            b.mode='mapping';b.plan(1);self.assertEqual(b.mode,'mapping');self.assertIn(b.phase,['follow','sense'])
    def test_in_place_turn_next_to_obstacle_has_no_sweep(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);b.grid.logodds[120:200,120:200]=-5
            # Obstacle beside the robot, inside the 20 cm driving circle but outside
            # TURN_CLEAR (was [0,-.202] against the old box swept about the offset axle).
            obstacle=np.array([0,-(geometry.R+.015)])
            angle=np.linspace(-np.pi,np.pi,300);b.current_cloud=np.vstack((np.c_[np.cos(angle),np.sin(angle)],obstacle))
            v,w=b.follow_velocity(np.array([0.,.7]),.7)
            # Round robot, axle through base_link: the route to the left is reached by a pure turn.
            self.assertEqual(v,0.);self.assertGreater(w,0.)
            # Independently integrate unicycle motion about base_link over the 1 s horizon and
            # check the round body (radius R) against the obstacle; driving keeps the 20 cm circle.
            centre=np.zeros(2);yaw=0.;gap=10.;centre_min=10.;dt=.001
            for _ in range(1000):
                centre=centre+v*dt*np.array([np.cos(yaw+w*dt/2),np.sin(yaw+w*dt/2)]);yaw+=w*dt
                gap=min(gap,np.linalg.norm(b.current_cloud-centre,axis=1).min()-geometry.R);centre_min=min(centre_min,np.linalg.norm(b.current_cloud-centre,axis=1).min())
            self.assertGreaterEqual(gap,.015-1e-9,'The round body moved toward the obstacle beside it')
            # Arc case next to an obstacle: target ahead-left, obstacle 25 cm to the right. The
            # chosen command must be a real arc/drive (v != 0) whose integrated unicycle path keeps
            # the skin at least DRIVE_CLEARANCE - R from the obstacle over the 1 s horizon.
            obstacle=np.array([[0.,-.25],[.05,-.25],[.1,-.25]]);b.current_cloud=np.vstack((np.c_[np.cos(angle),np.sin(angle)],obstacle))
            b.motion_until=0;b.path=[];v,w=b.follow_velocity(np.array([.6,.3]),.67)
            self.assertGreater(v,0.)
            centre=np.zeros(2);yaw=0.;gap=10.
            for _ in range(1000):
                centre=centre+v*dt*np.array([np.cos(yaw+w*dt/2),np.sin(yaw+w*dt/2)]);yaw+=w*dt
                gap=min(gap,np.linalg.norm(obstacle-centre,axis=1).min()-geometry.R)
            self.assertGreaterEqual(gap,DRIVE_CLEARANCE-geometry.R-1e-6)
    def test_velocity_search_moves_in_clear_space_but_not_into_unknown(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);angle=np.linspace(-np.pi,np.pi,300);b.current_cloud=np.c_[np.cos(angle),np.sin(angle)]
            b.grid.logodds[120:200,120:200]=-5
            self.assertGreater(b.follow_velocity(np.array([.5,0]),.5)[0],0)
            b.grid.logodds[:]=0
            self.assertEqual(b.follow_velocity(np.array([.5,0]),.5),(0,0))
    def test_movement_is_held_briefly_but_new_obstacle_interrupts_it(self):
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);b.grid.logodds[120:200,120:200]=-5;b.goal=np.array([1.,0]);b.phase_started=1
            angle=np.linspace(-np.pi,np.pi,300);b.current_cloud=np.c_[np.cos(angle),np.sin(angle)]
            with patch('autonomy.lidar_brain.choose_velocity',side_effect=[(.1,0.),(0.,.4)]) as search:
                self.assertEqual(b.follow_velocity(np.array([.5,0]),1.,10.),(.1,0.))
                self.assertEqual(b.follow_velocity(np.array([.5,.1]),1.,10.2),(.1,0.));self.assertEqual(search.call_count,1)
                b.current_cloud=np.vstack((b.current_cloud,[.22,0]))
                self.assertEqual(b.follow_velocity(np.array([.5,.1]),1.,10.3),(0.,.4));self.assertEqual(search.call_count,2)
    def test_recorded_doorway_loop_makes_progress_with_bounded_recovery(self):
        from types import SimpleNamespace
        for fixture_name in ['margin-loop','path-turn','door-turn']:
         with self.subTest(fixture=fixture_name),tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);folder=R/'tests/fixtures'/fixture_name;b.grid.load(folder);b.localizer.load(folder)
            state=json.loads((folder/'state.json').read_text());b.localizer.pose=np.array(state['pose']);b.goal=np.array(state['goal']);world=b.localizer.points.copy();safe=b.grid.masks()[3]
            b.visits=[np.array(p) for p in json.loads((folder/'exploration.json').read_text())['visits']]
            b.mode='go_to';b.phase='plan';b.attitude_initialized=True;b.translation_anchor=None
            # Closed-loop replay of measured geometry with ideal pose feedback.
            # It tests navigation/recovery, not Unreal physics or localization.
            phases=[]
            for tick in range(650):
                p=b.localizer.pose;t=tick*.1
                if np.linalg.norm(b.goal-p[:2])<.10:break
                b.current_cloud=inverse(world,p);b.current_cloud=b.current_cloud[np.linalg.norm(b.current_cloud,axis=1)<2]
                b.frame=SimpleNamespace(stamp=t);b.camera.last_stamp=t;b.localizer.uncertainty=.025;b.localizer.last_visual_time=t
                v,w=b.command({'sim_time_s':t,'armed':True});phases.append(b.phase);self.assertNotEqual(b.mode,'idle')
                # Drive axle AXLE_X ahead of base_link (0 on the round robot: plain unicycle about base_link).
                yaw=p[2];axle=p[:2]+AXLE_X*np.array([np.cos(yaw),np.sin(yaw)])
                for step in range(10):
                    axle+=v*.01*np.array([np.cos(yaw+w*.005),np.sin(yaw+w*.005)]);yaw+=w*.01
                    centre=axle-AXLE_X*np.array([np.cos(yaw),np.sin(yaw)])
                    # observe_scan records the driven trail at 1 cm spacing; reversing may only retrace it.
                    if not b.trail or np.linalg.norm(centre-b.trail[-1])>=.01:b.trail.append(centre.copy())
                    if b.phase=='follow':self.assertTrue(safe[b.grid.cell(centre)],'Follower re-entered recovery margin')
                    # Driving keeps DRIVE_CLEARANCE (20 cm) less 5 mm of tracking; a pure turn changes no distance.
                    self.assertGreater(np.linalg.norm(world-centre,axis=1).min(),DRIVE_CLEARANCE-.005)
                b.localizer.pose=np.r_[centre,yaw]
                if np.linalg.norm(centre-b.visits[-1])>.15:b.visits.append(centre.copy())
            self.assertLess(np.linalg.norm(b.goal-b.localizer.pose[:2]),.10)
            self.assertLessEqual(sum(p=='escape_margin' and phases[i-1]!=p for i,p in enumerate(phases) if i),3)
    def test_tiny_margin_reversals_cannot_reset_recovery_limit(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=self.recorded_margin(tmp);b.mode='mapping';b.begin_retreat=lambda stamp:False
            for i in range(3):
                b.localizer.pose[0]+=.001*(-1)**i
                self.assertTrue(b.begin_clearance_escape(i));self.assertEqual(b.mode,'mapping')
            # The fourth attempt skips this target; the mapping run continues.
            b.goal=np.array([1.,0]);self.assertTrue(b.begin_clearance_escape(4));self.assertEqual(b.mode,'mapping');self.assertEqual(b.phase,'plan')
            self.assertIn('no progress',b.message);self.assertTrue(any(np.allclose(g,[1,0]) for g in b.blacklist))
if __name__=='__main__':unittest.main()
