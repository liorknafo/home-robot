"""Mapping completion (task H, LIO-98): floor the robot can see but never enter does not hold the map open.

Unreal trial full-mapping-20261011T183022 mapped every room 95-100 % but never reported the map complete: mapping_complete()
allowed at most 1.5 m2 of safe floor the planner could not reach, and 1.56 m2 was floor under the sofa (the LiDAR sees under
its base, the robot does not fit) and inside the shower tray. Only floor the round body itself could never enter (no path of
cells clear of measured obstacles by its real size) is left out; floor cut off only by the planning margins, a planning
failure or an unknown gap still counts."""
import unittest,sys,tempfile
from pathlib import Path
import numpy as np,cv2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'controller'))
from autonomy.lidar import LidarGrid
from autonomy.lidar_brain import LidarBrain
from autonomy.geometry import R,FIT_CLEAR,PLAN_RADIUS,NARROW_RADIUS
LO,HI,POCKET=10,110,70     # room cells LO..HI-1 (5 x 5 m), pocket cells POCKET..HI-1 (2 x 2 m) in its far corner
def house(fence,b):
    """A measured 5 x 5 m room with measured walls (the outside unknown) and a 2 x 2 m pocket of measured free floor in
    its far corner, closed on its two open sides (the fence row and column POCKET-1) by `fence`:
      'wall': a continuous wall;
      'posts': one occupied cell every 5 cells (posts 25 cm apart: at most a 30.4 cm gap, the body is 2R = 31.8 cm wide);
      'marks': a continuous line of camera / contact marks (a low step the robot will not climb, e.g. a fully marked tray front);
      'door': a wall with a gap of 6 free cells (centres 35 cm apart: the body may fit, the planner never plans through it,
              not even at the narrow-passage clearance);
      'unknown': a wall with a stretch of 10 unknown cells (never measured: it may be open).
    The robot stands at cell (30, 30), localised, every cell it can reach covered by visits, no frontier."""
    g=LidarGrid(size=HI+10);b.grid=g;g.logodds[LO:HI,LO:HI]=-5
    g.logodds[LO-1,LO-1:HI+1]=5;g.logodds[HI,LO-1:HI+1]=5;g.logodds[LO-1:HI+1,LO-1]=5;g.logodds[LO-1:HI+1,HI]=5
    f=POCKET-1;fence_cells=[(f,c) for c in range(f,HI)]+[(r,f) for r in range(f+1,HI)]
    if fence=='marks':g.add_camera_obstacles(np.array([g.world(c) for c in fence_cells]),margin=0)
    else:
        for i,c in enumerate(fence_cells):g.logodds[c]=5 if fence!='posts' or i%5==0 else -5
    if fence=='door':g.logodds[f,85:91]=-5
    if fence=='unknown':g.logodds[f,85:95]=0
    b.localizer.pose=np.r_[g.world((30,30)),0.];b.localizer.uncertainty=.025;b.mode='mapping'
    b.visits=[g.world((r,c)) for r in range(LO,HI,10) for c in range(LO,HI,10)];return g
def area(mask,g):return float(mask.sum()*g.res**2)
class MappingCompletionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
    def brain(self,fence):
        b=LidarBrain(self.tmp.name);g=house(fence,b);return b,g
    def test_the_pocket_is_cut_off_from_the_planner_in_every_layout(self):
        # What every test below relies on: the pocket is more than 1.5 m2 of safe floor the planner cannot reach, so the
        # old rule (any unreached safe floor counts) refused completion in every layout.
        for fence in ('wall','posts','marks','door','unknown'):
            with self.subTest(fence=fence):
                b,g=self.brain(fence);safe=g.masks()[3];reach=g.reachable(b.localizer.pose[:2],safe)
                self.assertGreater(area(safe&~reach,g),1.5);self.assertGreater(area(reach,g),1.)
                self.assertFalse(g.frontiers(b.localizer.pose));self.assertLess(b.unvisited_area(),.5)
    def test_floor_walled_in_for_the_real_body_does_not_hold_the_map_open(self):
        # The sofa / dining-table pockets: floor the LiDAR measured between obstacles closer together than the body.
        for fence in ('wall','posts','marks'):
            with self.subTest(fence=fence):
                b,g=self.brain(fence)
                self.assertEqual(area(g.unreached_enterable(b.localizer.pose[:2]),g),0.)
                self.assertTrue(b.mapping_complete())
    def test_floor_the_body_could_enter_still_holds_the_map_open(self):
        # A robot boxed in by its own margins (a gap the body fits but the planner never uses) or by an unknown gap must not
        # report the house done.
        for fence in ('door','unknown'):
            with self.subTest(fence=fence):
                b,g=self.brain(fence)
                self.assertGreater(area(g.unreached_enterable(b.localizer.pose[:2]),g),1.5)
                self.assertFalse(b.mapping_complete())
    def test_the_door_is_closed_to_every_planning_clearance(self):
        # 'door' tests the margins, not a narrow passage the robot would plan through.
        b,g=self.brain('door');pose=b.localizer.pose
        for radius in (PLAN_RADIUS,NARROW_RADIUS):
            g.radius=radius;safe=g.masks()[3];reach=g.reachable(pose[:2],safe)
            self.assertFalse(reach[g.cell(g.world((90,90)))],radius)
        self.assertLess(FIT_CLEAR,R)
    def test_a_doorway_at_an_angle_that_the_body_might_fit_still_holds_the_map_open(self):
        # Review of task H: a 6 x 6 m room split by a wall at a 2:5 slope, its doorway's end cells 2 rows and 5 columns apart
        # (a true gap of 20.6-33.5 cm: the body, 31.8 cm, might fit). R - 2 cm (exact only along a row) called it closed and
        # left the 15 m2 beyond out; FIT_CLEAR must not. The same doorway along a row (end cells 5 apart, at most 30.4 cm)
        # is closed for the body, and the floor beyond is left out.
        for kind,(a,b),enterable in (('angled',((70,60),(72,65)),True),('row',((70,60),(70,65)),False)):
            with self.subTest(kind=kind):
                br=LidarBrain(self.tmp.name);g=LidarGrid(size=140);br.grid=g;lo,hi=10,130
                g.logodds[lo:hi,lo:hi]=-5;g.logodds[lo-1,lo-1:hi+1]=5;g.logodds[hi,lo-1:hi+1]=5;g.logodds[lo-1:hi+1,lo-1]=5;g.logodds[lo-1:hi+1,hi]=5
                wall=np.zeros(g.logodds.shape,np.uint8);dy,dx=(2,5) if kind=='angled' else (0,1)
                k=150//dx;cv2.line(wall,(a[1],a[0]),(a[1]-dx*k,a[0]-dy*k),1,1);cv2.line(wall,(b[1],b[0]),(b[1]+dx*k,b[0]+dy*k),1,1)
                inside=np.zeros_like(wall,bool);inside[lo:hi,lo:hi]=True;g.logodds[(wall>0)&inside]=5
                br.localizer.pose=np.r_[g.world((115,30)),0.];br.localizer.uncertainty=.025;br.mode='mapping'
                reach=g.reachable(br.localizer.pose[:2]);br.visits=[g.world(tuple(c)) for c in np.argwhere(reach)[::40]]
                self.assertGreater(area(g.masks()[3]&~reach,g),10.)
                self.assertFalse(g.frontiers(br.localizer.pose));self.assertLess(br.unvisited_area(),.5)
                self.assertEqual(area(g.unreached_enterable(br.localizer.pose[:2]),g)>10.,enterable)
                self.assertEqual(br.mapping_complete(),not enterable)
                self.assertEqual(area(g.unreached_enterable(br.localizer.pose[:2],fit=R-.02),g),0.,'R - 2 cm closes both')
    def test_with_no_reachable_floor_every_unreached_cell_counts(self):
        b,g=self.brain('posts');g.logodds[g.cell(b.localizer.pose[:2])]=5
        safe=g.masks()[3];self.assertFalse(g.reachable(b.localizer.pose[:2],safe).any())
        self.assertTrue((g.unreached_enterable(b.localizer.pose[:2],safe)==safe).all());self.assertFalse(b.mapping_complete())
    def test_robot_boxed_in_by_its_margins_never_reports_done(self):
        # The robot's own room (2 x 5 m) joins the rest of the house only through the 6-cell door: reachable floor left.
        b,g=self.brain('wall');g.logodds[LO:HI,50]=5;g.logodds[60:66,50]=-5
        self.assertLess(area(g.reachable(b.localizer.pose[:2]),g),10.)
        self.assertFalse(b.mapping_complete())
    def test_the_manifest_reports_the_floor_the_body_could_enter(self):
        b,g=self.brain('posts');b.map_folder=Path(self.tmp.name)/'map';(Path(self.tmp.name)/'Saved/Maps').mkdir(parents=True)
        b.save_map(True,'test');import json;m=json.loads((b.map_folder/'manifest.json').read_text())
        self.assertGreater(m['disconnected_safe_area_m2'],1.5);self.assertEqual(m['unreached_enterable_area_m2'],0.)
    def test_walled_in_floor_does_not_send_the_robot_through_narrow_passages(self):
        # Narrow-passage planning (20 cm clearance) is for floor the body can enter. The walled-in pocket is more than the
        # 1 m2 that triggers it; a small alcove behind an 8-cell door (narrow-passable) is not: with the pocket counted the
        # robot left for the alcove at 20 cm clearance; without it, the robot finishes.
        b,g=self.brain('posts');g.logodds[LO:40,90]=5;g.logodds[40,90:HI]=5;g.logodds[20:28,90]=-5   # alcove: cells 10-39 x 91-109
        pose=b.localizer.pose;b.visits=[g.world((r,c)) for r in range(LO,HI,10) for c in range(LO,HI,10) if not (r<=40 and c>=90)]
        alcove=area(g.unreached_enterable(pose[:2]),g);self.assertGreater(alcove,0.);self.assertLess(alcove,1.)
        self.assertLess(b.unvisited_area(),.01)
        g.radius=NARROW_RADIUS;self.assertTrue(b.coverage_viewpoints(pose),'the alcove is a viewpoint at the narrow clearance');g.radius=PLAN_RADIUS
        for k in range(3):b.plan(float(k))
        self.assertNotIn('narrow',' '.join(e['message'] for e in b.events))
        self.assertEqual(b.mode,'idle');self.assertTrue(b.complete)
        self.assertEqual(g.radius,PLAN_RADIUS)
class UnexploredSpaceTests(unittest.TestCase):
    """Task R (LIO-153). With the LiDAR 20 mm lower (task L) its beams pass under the bed frame (underside 125 mm, 2.5 mm below
    the scan plane) whenever the robot pitches nose-down a little, so the floor under the bed is mapped free. The frame's sides
    are returned too, so that floor is walled in for the body from the room. Yet task H's rule counted it in two Unreal trials
    (full-mapping-20261013T004512: 0.85 m2, 1.70 m2 in all; ...20261012T230227: 0.29 m2, 1.69 m2 in all; limit 1.5 m2): the
    body-fit way left the house through a 25 cm stretch of wall the LiDAR never saw behind the vanity, ran round the outside of
    the house through about 200 never-measured cells (unknown counts as open), and came back in through the wall hidden behind
    the bed. An unmeasured gap between measured floor still counts as open; unexplored space beyond the measured floor is no
    way in for the body."""
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
    def hidden_walls(self,fence='wall'):
        b=LidarBrain(self.tmp.name);g=house(fence,b)
        g.logodds[LO-1,26:31]=0      # 5 cells of outer wall never measured on the robot's side (behind the vanity)
        g.logodds[HI,86:91]=0        # and 5 behind the pocket (behind the bed, against the outer wall)
        return b,g
    def test_the_hidden_wall_stretches_are_no_frontier(self):
        # As in the house: a stretch of 5 unknown cells in a grazed wall is not a frontier (task B's 10 cm rule), yet the round
        # body fits through it on the map (15 cm from both wall ends, FIT_CLEAR 10.9 cm).
        b,g=self.hidden_walls();self.assertFalse(g.frontiers(b.localizer.pose))
        free,occupied,_,_,clearance=g.masks();self.assertGreaterEqual(clearance[LO-1,28],FIT_CLEAR)
    def test_floor_walled_in_from_the_room_does_not_count_through_unexplored_space_outside_the_walls(self):
        for fence in ('wall','posts','marks'):
            with self.subTest(fence=fence):
                b,g=self.hidden_walls(fence)
                self.assertEqual(area(g.unreached_enterable(b.localizer.pose[:2]),g),0.)
                self.assertTrue(b.mapping_complete())
    def test_an_unmeasured_gap_between_measured_floor_still_counts(self):
        # Task H's protection: a wall stretch nobody measured may be open (fence 'unknown': 10 unknown cells), here also with a
        # band of never-measured floor 60 cm deep on both sides of the gap, and with the hidden outer walls as well.
        for depth in (0,6):
            with self.subTest(depth=depth):
                b,g=self.hidden_walls('unknown');f=POCKET-1;g.logodds[f-depth:f+depth+1,85:95]=0
                self.assertGreater(area(g.unreached_enterable(b.localizer.pose[:2]),g),1.5)
                self.assertFalse(b.mapping_complete())
    def test_an_unmeasured_gap_at_the_grids_edge_still_counts(self):
        # astra's probe (review 1): the closing must not treat the space beyond the grid as "no floor". The robot's map is 16 m
        # square about its start, so a wall 7.5 m or more away lies at its edge. Task H's 10-cell gap, in the pocket's fence
        # right beside the room's outer wall, which is the grid's last row: before the padding the gap's cells were dropped and
        # the house reported complete.
        b=LidarBrain(self.tmp.name);g=LidarGrid(size=HI+1);b.grid=g;g.logodds[LO:HI,LO:HI]=-5
        g.logodds[LO-1,LO-1:HI+1]=5;g.logodds[HI,LO-1:HI+1]=5;g.logodds[LO-1:HI+1,LO-1]=5;g.logodds[LO-1:HI+1,HI]=5
        f=POCKET-1;g.logodds[f,f:HI]=5;g.logodds[f+1:HI,f]=5;g.logodds[HI-10:HI,f]=0
        b.localizer.pose=np.r_[g.world((30,30)),0.];b.localizer.uncertainty=.025;b.mode='mapping'
        b.visits=[g.world((r,c)) for r in range(LO,HI,10) for c in range(LO,HI,10)]
        self.assertEqual(g.n-1,HI,'the outer wall is the grid\'s last row')
        self.assertGreater(area(g.unreached_enterable(b.localizer.pose[:2]),g),1.5)
        self.assertFalse(b.mapping_complete())
    def test_unexplored_space_next_to_the_robots_floor_is_a_frontier(self):
        # What holds the map open when a way into a pocket would run through unexplored space wider than a gap: the frontier.
        b,g=self.brain_with_unexplored_band();self.assertTrue(g.frontiers(b.localizer.pose));self.assertFalse(b.mapping_complete())
    def brain_with_unexplored_band(self):
        b,g=self.hidden_walls('unknown');f=POCKET-1;g.logodds[f-15:f+16,80:100]=0;return b,g
class CoverageTargetTests(unittest.TestCase):
    def test_a_coverage_target_that_drops_out_of_reach_is_not_covered_on_the_way(self):
        # B13 / task F: 13 of 22 (and 17 of 55) 'Target area covered on the way' events came with the robot 1.2-4 m from its
        # target: the target cell left the unvisited mask because its reachability flickered (a narrow corridor between
        # chair legs), the robot chose another target, and the two ping-ponged until the time budget.
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);g=house('wall',b);b.visits=[b.localizer.pose[:2].copy()]
            b.goal=g.world((60,60));b.goal_kind='coverage'
            self.assertFalse(b.coverage_goal_covered())
            g.logodds[LO:HI,45]=5            # the corridor closes: the target is out of reach, not covered
            self.assertFalse(b.grid.reachable(b.localizer.pose[:2])[g.cell(b.goal)])
            self.assertFalse(b.coverage_goal_covered())
            g.logodds[LO:HI,45]=-5;b.visits.append(g.world((55,55)));self.assertTrue(b.coverage_goal_covered())
class RouteFlipTests(unittest.TestCase):
    def test_a_route_that_keeps_closing_is_lost_not_swapped_for_the_long_way_round(self):
        # Task H trial 2 (full-mapping-20261011T225055, the last 100 s): the short route to a target under the sofa closed
        # each time the robot came near (the LiDAR then returned the sofa's end) and reopened as it backed off; every
        # replan took the cheapest route, so the robot swapped between it and a 2 m longer one round the far end 7 times
        # and came no closer. A route that closes and leaves only a detour more than ROUTE_DETOUR_M longer is a closed
        # route (route_lost); lost twice, the target is left.
        sys.path.insert(0,str(Path(__file__).resolve().parent))
        from test_whole_house_pacing import room,ready
        from autonomy.lidar_brain import ROUTE_DETOUR_M
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);g=room(size=100);b.grid=g
            g.logodds[50,20:80]=5;g.logodds[50,46:56]=-5;g.logodds[50,70:80]=-5     # a wall with a short gap A and a far gap B
            near,far=g.world((30,30)),g.world((30,32));goal=g.world((70,30))
            b.localizer.pose=np.r_[near,0.];short=g.path(near,goal);self.assertTrue(short)
            g.logodds[50,46:56]=5;longway=g.path(near,goal);g.logodds[50,46:56]=-5
            length=lambda r:float(np.linalg.norm(np.diff(np.asarray(r),axis=0),axis=1).sum())
            self.assertGreater(length(longway),length(short)+ROUTE_DETOUR_M)
            t=10.;ready(b,t);b.goal=goal.copy();b.goal_kind='coverage';b.path=short;b.phase='follow';b.next_plan=t
            b.last_progress=t;b.progress_pose=b.localizer.pose.copy()
            def run(seconds):
                nonlocal t
                for k in range(int(seconds/.5)):
                    t+=.5;b.localizer.pose=np.r_[near if k%2 else far,0.];ready(b,t,stamp_only=True)
                    b.command(dict(sim_time_s=t,armed=True,fault='none'))
            for cycle in range(2):
                g.logodds[50,46:56]=5;run(7.)                      # gap A closes as the robot comes near...
                self.assertEqual(sum(np.linalg.norm(x-goal)<.3 for x in getattr(b,'lost_routes',[])),cycle+1)
                g.logodds[50,46:56]=-5;run(2.)                     # ...and reopens as it backs off
            self.assertTrue(any(np.linalg.norm(x-goal)<.3 for x in b.blacklist),'lost twice: the target is left')
    def test_a_route_closed_at_a_diagonal_corner_or_cut_off_from_the_robot_is_not_open(self):
        # Review 3: 'still open' must be the planner's judgement. Trial 3 (full-mapping-20261012T001143, kitchen, 1036 s)
        # swapped once from a 3.7 m route to a 6.0 m loop with no event: every cell of the old route was still traversable,
        # but path() would not use it (a diagonal step past two blocked corners, or no link from the robot's cell).
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);g=LidarGrid(size=60);b.grid=g;g.logodds[:]=-5;b.mode='mapping'
            blocked=np.zeros(g.logodds.shape,bool);plain=g.masks
            def masks():
                free,occupied,unknown,safe,clearance=plain();return free,occupied,unknown,safe&~blocked,clearance
            g.masks=masks
            route=[g.world((30,c)) for c in range(10,21)]+[g.world((31,c)) for c in range(21,41)]
            longway=[g.world((r,10)) for r in range(30,56)]+[g.world((55,c)) for c in range(11,41)]+[g.world((r,40)) for r in range(54,30,-1)]
            b.path=route;b.localizer.pose=np.r_[g.world((30,10)),0.]
            self.assertFalse(b.detour_only(longway),'the route is open: the long way is just a worse choice')
            blocked[30,21]=blocked[31,20]=True                   # the corners of the diagonal step only
            self.assertTrue(all(b.motion_mask()[g.cell(xy)] for xy in route),'every route cell is still traversable')
            self.assertTrue(b.detour_only(longway),'closed at a diagonal corner')
            blocked[:]=False;blocked[27:29,:]=True;b.localizer.pose=np.r_[g.world((25,10)),0.]   # a band between robot and route
            self.assertTrue(all(b.motion_mask()[g.cell(xy)] for xy in route))
            self.assertTrue(b.detour_only(longway),'cut off from the robot')
if __name__=='__main__':unittest.main()
