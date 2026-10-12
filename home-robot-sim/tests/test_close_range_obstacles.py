"""Close-range obstacle evidence (task K, LIO-118): an obstacle cell seen from close by is not cleared by later beams that
pass it from much farther away.

Unreal trials (task H): the sofa base's bottom is 13 cm, the LiDAR plane was then 14.75 cm (task L lowers it). From close by the LiDAR returns the
base's ends; from farther away the beams that reach them skim past at a grazing angle through the ends' partly free 5 cm
cells, or pass under the base when the robot tilts (1 deg drops a beam 1.75 cm at 1 m). Re-cast offline against the house
(tests/replay_unreal_lidar.py, trial full-mapping-20261012T013115), the sofa's north-end cells got 708 hits / 43 passes from
0.3-0.5 m but 70 / 729 from 1-1.5 m and 130 / 2499 from 3-5 m, so far scans erased what close scans had measured and the
floor under the sofa reopened. The robot cannot tell a beam's height (its pitch estimate is off by up to ~1.3 deg), so the
rule is by range: a beam passing a cell farther than NEAR_SLACK_M beyond the closest range the cell was hit from cannot
lower it, until the cell has gone HOLD_S without a hit (a person or chair that moved away)."""
import unittest,sys,math,tempfile
from pathlib import Path
from unittest import mock
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'controller'))
from autonomy.lidar import LidarGrid,inverse,NEAR_SLACK_M,HOLD_S
from autonomy.lidar_brain import LidarBrain
from autonomy.geometry import LIDAR_MOUNT

OB_X,OB_HALF,WALL_X=1.025,.12,3.0     # a 24 cm obstacle face on a cell-centre line x = 1.025 m (|y| <= 12 cm), a wall at x = 3 m
OB_CELLS=[(y,x) for y in range(LidarGrid().cell((0,-.10))[0],LidarGrid().cell((0,.10))[0]+1) for x in [LidarGrid().cell((OB_X,0))[1]]]

def scan(pose,obstacle):
    """Base-frame points of 361 beams over +-60 deg ahead of the LiDAR at `pose`: each ends on the obstacle face if it crosses
    it and `obstacle` (the face returns this beam), else on the wall. obstacle False: the beam passes it (gone, or under)."""
    c,s=math.cos(pose[2]),math.sin(pose[2]);o=pose[:2]+np.array([c*LIDAR_MOUNT[0]-s*LIDAR_MOUNT[1],s*LIDAR_MOUNT[0]+c*LIDAR_MOUNT[1]])
    a=pose[2]+np.radians(np.linspace(-60,60,361));d=np.c_[np.cos(a),np.sin(a)]
    t=(WALL_X-o[0])/d[:,0];tob=(OB_X-o[0])/d[:,0];yob=o[1]+tob*d[:,1]
    if obstacle:t=np.where((tob>0)&(np.abs(yob)<=OB_HALF),tob,t)
    return inverse(o+t[:,None]*d,pose)

def run(g,x,obstacle,stamps,y=0.):
    """One scan per stamp from (x, y), the heading alternating 0 / 0.07 rad so every scan passes integrate_scan's motion gate."""
    for k,t in enumerate(stamps):
        pose=np.array([x,y,.07*(k%2)]);g.integrate_scan(scan(pose,obstacle),pose,t)

def occupied(g):return [bool(g.logodds[c]>.8) for c in OB_CELLS]
def sensor_range(x):return OB_X-(x+LIDAR_MOUNT[0])

CLOSE_X=OB_X-LIDAR_MOUNT[0]-.30          # the LiDAR 30 cm from the face
FAR_X=OB_X-LIDAR_MOUNT[0]-2.0             # 2 m from it

class CloseRangeObstacleTests(unittest.TestCase):
    def sighted(self):
        """A grid in which the face was hit from 30 cm (5 scans at t 0-4 s)."""
        g=LidarGrid();run(g,CLOSE_X,True,range(5));self.assertTrue(all(occupied(g)),'set-up: the face is mapped')
        self.assertTrue(all(abs(g.hit_range[c]-.30)<.04 for c in OB_CELLS),[g.hit_range[c] for c in OB_CELLS]);return g

    def test_far_beams_passing_under_do_not_clear_a_face_seen_from_close_by(self):
        # The sofa: seen from 30 cm, then 2 m away the beams pass under it (no return) for 20 scans. Before task K
        # 20 x -0.7 took the cells from +5 to free.
        g=self.sighted();run(g,FAR_X,False,np.arange(10,30))
        self.assertTrue(all(occupied(g)),[float(g.logodds[c]) for c in OB_CELLS])

    def test_moved_obstacle_seen_gone_from_as_close_clears_at_once(self):
        # A chair taken away while the robot is still there: beams from where it was seen pass through, and clear it as before.
        g=self.sighted();run(g,CLOSE_X,False,np.arange(5,15))
        self.assertFalse(any(occupied(g)))
        # A beam passing within NEAR_SLACK_M farther than the sighting counts too.
        g=self.sighted();run(g,OB_X-LIDAR_MOUNT[0]-(.30+NEAR_SLACK_M-.03),False,np.arange(5,15))
        self.assertFalse(any(occupied(g)))

    def test_a_cell_not_hit_for_hold_s_is_cleared_by_far_beams(self):
        # A person who walked away and is never seen there again: far beams clear the cells once HOLD_S has passed
        # since the last hit, not before.
        g=self.sighted();run(g,FAR_X,False,np.arange(4+HOLD_S-20,4+HOLD_S-5))
        self.assertTrue(all(occupied(g)))
        run(g,FAR_X,False,np.arange(4+HOLD_S+1,4+HOLD_S+16))
        self.assertFalse(any(occupied(g)))

    def test_a_far_hit_renews_the_hold(self):
        # The sofa still returns some far beams: each hit, from any range, restarts HOLD_S; the close range is kept.
        g=self.sighted();run(g,FAR_X,True,[HOLD_S-10])
        self.assertTrue(all(abs(g.hit_range[c]-.30)<.04 for c in OB_CELLS))
        run(g,FAR_X,False,np.arange(HOLD_S+30,HOLD_S+50))
        self.assertTrue(all(occupied(g)),'held: hit 40 s ago')
        run(g,FAR_X,False,np.arange(2*HOLD_S,2*HOLD_S+20))
        self.assertFalse(any(occupied(g)),'cleared: no hit for more than HOLD_S')

    def test_the_rule_compares_ranges(self):
        # An obstacle first seen from 2 m is cleared by beams from about 2 m (it moved), as before; beams passing it from
        # 3.5 m (farther than 2 m + NEAR_SLACK_M) do not clear it within the hold.
        g=LidarGrid();run(g,FAR_X,True,range(5));self.assertTrue(all(occupied(g)))
        self.assertTrue(all(abs(g.hit_range[c]-2.)<.04 for c in OB_CELLS))
        run(g,FAR_X-NEAR_SLACK_M/2,False,np.arange(5,25));self.assertFalse(any(occupied(g)))
        g=LidarGrid();run(g,FAR_X,True,range(5));run(g,FAR_X-1.5,False,np.arange(5,25));self.assertTrue(all(occupied(g)))

    def test_cleared_cell_forgets_its_close_range(self):
        # Once a held cell has been cleared it starts afresh: a new obstacle there seen only from 2 m is judged at 2 m.
        g=self.sighted();run(g,CLOSE_X,False,np.arange(5,15));self.assertFalse(any(occupied(g)))
        self.assertTrue(all(np.isinf(g.hit_range[c]) for c in OB_CELLS))
        run(g,FAR_X,True,np.arange(15,20));self.assertTrue(all(abs(g.hit_range[c]-2.)<.04 for c in OB_CELLS))

    def test_the_robot_standing_there_clears_a_held_cell(self):
        # The cells under the robot's own body (integrate_scan's footprint) are free whatever the LiDAR saw before.
        g=self.sighted();run(g,OB_X,False,np.arange(5,20))
        self.assertFalse(g.logodds[g.cell((OB_X,0.))]>.8)

    def test_the_held_map_is_never_freer_than_the_old_rule(self):
        # Safety: the rule only skips decrements, so on any sequence of scans every cell's log-odds is at least the old
        # rule's (near_slack = inf turns the rule off), no cell is free that the old rule would not call free, and cells
        # never hit are updated exactly as before.
        rng=np.random.default_rng(3)
        for trial in range(4):
            new,old=LidarGrid(size=140),LidarGrid(size=140);old.near_slack=math.inf;t=0.;hit_ever=np.zeros(new.seen.shape,bool)
            for k in range(120):
                x=rng.uniform(-2.5,.75);y=rng.uniform(-.3,.3);yaw=rng.uniform(-.3,.3);t+=rng.uniform(.2,40.);pose=np.array([x,y,yaw])
                pts=scan(pose,rng.random()<.5)
                for g in (new,old):g.integrate_scan(pts,pose,t)
                hit_ever|=np.isfinite(new.hit_range)
                self.assertTrue((new.logodds>=old.logodds-1e-6).all(),(trial,k))
                self.assertFalse(((new.logodds<-.8)&~(old.logodds<-.8)).any(),(trial,k))
            np.testing.assert_allclose(new.logodds[~hit_ever],old.logodds[~hit_ever])

    def test_mapping_passes_the_scan_time(self):
        # LidarBrain.observe_scan gives integrate_scan the scan's sim time (HOLD_S is in seconds).
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);b.start_mapping(0.);b.localizer.update_scan=lambda *a,**k:(True,np.zeros(3))
            ranges=[2000]*667;sc=dict(frame=7,start_time_s=12.,sim_time_s=12.5,angle_min_rad=0,angle_increment_rad=-2*math.pi/667,range_min_m=.1,
                                        range_max_m=8,ranges_mm=ranges,time_offsets_ms=[0]*667,sensor_in_base_m=list(LIDAR_MOUNT))
            with mock.patch.object(b.grid,'integrate_scan') as integrate:
                b.observe_scan(sc)
            self.assertEqual(integrate.call_args.args[2] if len(integrate.call_args.args)>2 else integrate.call_args.kwargs.get('stamp'),12.5)

if __name__=='__main__':unittest.main()
