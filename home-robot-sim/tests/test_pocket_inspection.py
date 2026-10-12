"""Looking at a pocket's opening from close by before reporting the map incomplete (task K, LIO-118).

Unreal trial full-mapping-20261012T070020 stopped 'not complete' at 936 s: 1.56 m2 of floor the body might enter (task H's
grid.unreached_enterable) against the 1.5 m2 limit. 0.73 m2 of it was the floor under the sofa, counted because its north end
was mapped open: the robot never came closer than 0.44 m to it, and the sofa's ends return the LiDAR only from close by (the
task K rule keeps them once seen so). So before giving up on such floor the robot now drives once to the last cell its planner
reaches on the body's way into each pocket (LidarBrain.inspection_viewpoints) and looks: furniture is then mapped closed and
its floor stops counting; a real opening stays open and its floor still counts."""
import unittest,sys,tempfile
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'controller'));sys.path.insert(0,str(Path(__file__).resolve().parent))
from autonomy.lidar_brain import LidarBrain
from test_mapping_completion import house,area,LO,HI,POCKET

DOOR=np.mean([(POCKET-1,c) for c in range(85,91)],axis=0)   # the 'door' fence's 6-cell gap (row POCKET-1, columns 85-90)

class PocketInspectionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
    def brain(self,fence):
        b=LidarBrain(self.tmp.name);g=house(fence,b);return b,g

    def test_the_viewpoint_is_where_the_planner_stops_on_the_way_into_the_pocket(self):
        # 'door': 4 m2 behind a 35 cm gap the body may fit but the planner never uses. The viewpoint is a cell the planner
        # reaches, just short of the gap (the body's way in passes it), so the LiDAR sees the gap from close by.
        b,g=self.brain('door');pose=b.localizer.pose
        self.assertGreater(area(g.unreached_enterable(pose[:2]),g),1.5)
        views=b.inspection_viewpoints(pose);self.assertEqual(len(views),1,views)
        _,xy,route=views[0];safe=g.masks()[3];reach=g.reachable(pose[:2],safe)
        self.assertTrue(reach[g.cell(xy)] and route,'reachable, with a route')
        self.assertLess(np.linalg.norm(xy-g.world(tuple(int(v) for v in np.round(DOOR)))),.45,'just short of the gap')

    def test_plan_looks_once_before_reporting_the_map_incomplete(self):
        # With nothing else left the robot goes to look instead of finishing; a second time (nothing changed) it finishes, not
        # complete, as before.
        b,g=self.brain('door')
        for k in range(3):
            b.plan(float(k))
            if b.phase=='follow':break
        self.assertEqual((b.phase,b.goal_kind),('follow','inspect'))
        self.assertIn('from close by',b.events[-2]['message'])
        b.localizer.pose=np.r_[b.goal,0.];b.scanned_positions.append(b.goal.copy());b.phase='plan'
        for k in range(3):b.plan(10.+k)
        self.assertEqual(b.mode,'idle');self.assertFalse(b.complete)

    def test_a_look_that_maps_the_opening_closed_lets_the_map_complete(self):
        # What the look is for: the sofa's end returned from close by fills the gap, and the floor beyond stops counting.
        b,g=self.brain('door');b.plan(0.);self.assertEqual(b.goal_kind,'inspect')
        g.logodds[POCKET-1,85:91]=5
        self.assertEqual(area(g.unreached_enterable(b.localizer.pose[:2]),g),0.);self.assertTrue(b.mapping_complete())

    def test_no_look_when_the_floor_is_walled_in_or_within_the_limit(self):
        # Walled in for the body (posts 25 cm apart): nothing counts, nothing to look at. An alcove under the 1.5 m2 limit:
        # the map completes without a trip.
        b,g=self.brain('posts');self.assertEqual(b.inspection_viewpoints(b.localizer.pose),[])
        b,g=self.brain('posts');g.logodds[LO:40,90]=5;g.logodds[40,90:HI]=5;g.logodds[20:28,90]=-5
        b.visits=[g.world((r,c)) for r in range(LO,HI,10) for c in range(LO,HI,10) if not (r<=40 and c>=90)]
        self.assertLess(area(g.unreached_enterable(b.localizer.pose[:2]),g),1.5)
        self.assertEqual(b.inspection_viewpoints(b.localizer.pose),[])

if __name__=='__main__':unittest.main()
