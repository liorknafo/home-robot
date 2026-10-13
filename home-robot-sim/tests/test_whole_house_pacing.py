"""Regressions from whole-house runs: target choice, narrow gaps, getting unstuck."""
import unittest,sys,tempfile,math
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'controller'))
from autonomy.lidar import LidarGrid
from autonomy.lidar_brain import LidarBrain
from autonomy import geometry
from autonomy.geometry import R,TURN_CLEAR,TURN_MARGIN,DRIVE_CLEARANCE,BODY_CLEAR,PLAN_RADIUS,BUMP_ZONES
CONTROLLER=Path(__file__).resolve().parents[1]/'controller'
def open_floor(b,radius=1.5):
    """Measured free floor around base_link at the origin, walls `radius` away."""
    b.grid.logodds[120:200,120:200]=-5;b.localizer.pose=np.zeros(3)
    angle=np.linspace(-np.pi,np.pi,300);return radius*np.c_[np.cos(angle),np.sin(angle)]
def wall(p0,p1,step=.002):
    p0,p1=np.asarray(p0,float),np.asarray(p1,float);n=int(np.linalg.norm(p1-p0)/step)+1
    return p0+np.linspace(0,1,n)[:,None]*(p1-p0)
def raycast(segments,noise=0.,seed=0,origin=None):
    """One D200-like scan (667 beams from the LiDAR mount) of line segments, robot frame,
    Gaussian range jitter `noise` along each beam (sim2d uses 4 mm). Like the D200 (sim2d, D200Model),
    a beam whose first hit is nearer than 0.1 m returns nothing (task F: the near-field tests below then hold only
    while the pressed bumper skin stays >= 0.1 m from the LiDAR, which is what limits the LD14 to x 0.054)."""
    from autonomy.geometry import LIDAR_MOUNT
    o=np.array(LIDAR_MOUNT[:2] if origin is None else origin,float);rng=np.random.default_rng(seed)
    a=-2*np.pi*np.arange(667)/667;d=np.c_[np.cos(a),np.sin(a)];best=np.full(667,np.inf)
    for A,B in segments:
        A=np.asarray(A,float);e=np.asarray(B,float)-A;den=d[:,0]*e[1]-d[:,1]*e[0];w=A-o
        with np.errstate(divide='ignore',invalid='ignore'):
            t=(w[0]*e[1]-w[1]*e[0])/den;u=(w[0]*d[:,1]-w[1]*d[:,0])/den
        ok=(np.abs(den)>1e-12)&(t>0)&(u>=0)&(u<=1);best=np.where(ok&(t<best),t,best)
    keep=np.isfinite(best)&(best>=.1);r=best[keep]+rng.normal(0,noise,keep.sum())
    return o+r[:,None]*d[keep]
def corner(apex_distance,half_angle=math.pi/4,arm=.2,azimuth=0.):
    """Convex corner (box corner / door jamb) pointing at base_link, apex at apex_distance."""
    u=np.array([math.cos(azimuth),math.sin(azimuth)]);apex=apex_distance*u
    return [(apex,apex+arm*np.array([math.cos(azimuth+s*half_angle),math.sin(azimuth+s*half_angle)])) for s in (1,-1)]
def post(nearest,radius=.005,azimuth=0.,n=72):
    """Round post (chair leg) whose nearest point is `nearest` from base_link."""
    c=(nearest+radius)*np.array([math.cos(azimuth),math.sin(azimuth)]);t=np.linspace(0,2*np.pi,n+1)
    q=c+radius*np.c_[np.cos(t),np.sin(t)];return [(q[i],q[i+1]) for i in range(n)]
def feed_scans(b,segments,count,noise=.004,seed=0):
    """`count` scans of the same scene from the same pose, as observe_scan records them."""
    from types import SimpleNamespace
    for k in range(count):
        cloud=raycast(segments,noise,seed*100+k);b.frame=SimpleNamespace(stamp=.15*k,index=k)
        b.current_cloud=np.vstack((cloud,1.5*np.c_[np.cos(np.linspace(-np.pi,np.pi,300)),np.sin(np.linspace(-np.pi,np.pi,300))]))
        b.note_spin_scan(b.current_cloud,b.localizer.pose,.15*k)
def corner_walls(gap,layout):
    """Kitchen corner in the robot frame (heading 0): skin `gap` from both walls. 'diagonal' faces the
    corner (walls at +/-45 deg); 'side' has one wall ahead and one on the right."""
    d=R+gap
    if layout=='side':return [((d,-1.2),(d,1.2)),((-1.2,-d),(d,-d))]
    c=np.array([d*math.sqrt(2),0.]);back=1.2*np.array([-math.sqrt(.5),math.sqrt(.5)])
    return [(c,c+back),(c,c+back*[1,-1])]
def turned(segments,yaw):
    """The same (world-fixed) segments seen from base_link after it turned in place by yaw."""
    c,s=math.cos(yaw),math.sin(yaw);m=np.array([[c,-s],[s,c]])
    return [(np.asarray(a,float)@m,np.asarray(b,float)@m) for a,b in segments]
def seen_from(segments,pose):
    """World-fixed segments (the start frame) seen from base_link at pose (x, y, yaw)."""
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw);m=np.array([[c,-s],[s,c]]);o=np.array([x,y])
    return [((np.asarray(a,float)-o)@m,(np.asarray(b,float)-o)@m) for a,b in segments]
def random_scene(rng):
    """1-3 random walls / posts from inside the skin to 6 cm outside it, any azimuth."""
    segs=[]
    for _ in range(rng.integers(1,4)):
        az=rng.uniform(-math.pi,math.pi);dist=R+rng.uniform(-.002,.06)
        if rng.random()<.6:
            n=np.array([math.cos(az),math.sin(az)]);tv=np.array([-n[1],n[0]]);L=rng.uniform(.05,1.2)
            segs.append((dist*n-L*tv,dist*n+L*tv))
        else:segs+=post(dist,rng.uniform(.003,.03),az,24)
    return segs
def wrap_(a):return math.atan2(math.sin(a),math.cos(a))
def ready(b,t,stamp_only=False):
    """Fresh sensors and a valid pose at time t, as command() requires before it moves."""
    from types import SimpleNamespace
    if not stamp_only:b.mode='mapping';b.attitude_initialized=True;b.localizer.uncertainty=.025;b.motor_seen_armed=True;b.started=0.
    b.frame=SimpleNamespace(stamp=t,index=int(t*20));b.camera.last_stamp=t;b.localizer.last_visual_time=t
def json_keys(d):
    """All keys of a nested status dict."""
    out=set()
    if isinstance(d,dict):
        for k,v in d.items():out.add(k);out|=json_keys(v)
    return out
def room(size=100,lo=20,hi=80):
    """Measured free room with measured walls; outside is unknown."""
    g=LidarGrid(size=size);g.logodds[lo:hi,lo:hi]=-5
    g.logodds[lo-1,lo-1:hi+1]=5;g.logodds[hi,lo-1:hi+1]=5;g.logodds[lo-1:hi+1,lo-1]=5;g.logodds[lo-1:hi+1,hi]=5
    return g
class WholeHousePacingTests(unittest.TestCase):
    def test_clearance_is_exact_on_diagonals(self):
        # The 5x5 approximation gave 4.2 cells for a 3-3 diagonal and closed a
        # measured passage by a fraction of a millimetre.
        g=LidarGrid(size=40);g.logodds[:]=-5;g.logodds[10,10]=5
        self.assertAlmostEqual(g.masks()[4][13,13],3*math.sqrt(2)*g.res,places=4)
    def test_gap_in_a_measured_wall_is_not_a_frontier(self):
        g=room();pose=np.r_[g.world((50,50)),0]
        self.assertEqual(g.frontiers(pose),[])
        g.logodds[19,48:53]=0       # missed returns: a 25 cm unknown gap in the wall
        self.assertEqual(g.frontiers(pose),[])
        g.logodds[19,40:55]=0       # a 75 cm opening the LiDAR can look through
        self.assertTrue(g.frontiers(pose))
    def test_unobservable_frontier_is_dropped_after_two_failed_attempts(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);b.grid=room();g=b.grid;g.logodds[19,40:55]=0
            pose=np.r_[g.world((50,50)),0];b.localizer.pose=pose
            goal=g.frontiers(pose)[0][1]
            for attempt in range(2):
                self.assertTrue(g.frontiers(pose))
                # The visit (or a lost route) left the opening's unknown edge unknown.
                b.frontier_target=b.unknown_edge(goal);b.review_frontier_target()
            self.assertEqual(g.frontiers(pose),[])
            # A visit that reveals the edge is progress, not a failed attempt.
            b=LidarBrain(tmp);b.grid=room();g=b.grid;g.logodds[19,40:55]=0
            b.frontier_target=b.unknown_edge(goal);g.logodds[17:20,40:55]=-5;b.review_frontier_target()
            self.assertFalse(getattr(g,'unobservable',np.zeros(1,bool)).any())
    def test_nearer_frontier_beats_a_larger_unknown_area_across_the_house(self):
        # Everything is measured except a small pocket beside the robot and a
        # large unknown area 5 m away: the short drive comes first.
        g=LidarGrid(size=160);g.logodds[:]=5;g.logodds[20:40,10:150]=-5
        g.logodds[40:45,24:36]=0;g.logodds[40:150,120:140]=0
        pose=np.r_[g.world((30,30)),0];best=max(g.frontiers(pose),key=lambda c:c[0])
        self.assertLess(np.linalg.norm(best[1]-pose[:2]),1.)
    def test_route_lost_twice_leaves_the_target(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);b.mode='mapping';b.goal=np.array([1.,1.]);b.route_lost(1)
            self.assertEqual(b.phase,'plan');self.assertEqual(b.blacklist,[])
            b.goal=np.array([1.05,1.]);b.route_lost(2);self.assertEqual(len(b.blacklist),1)
    def test_coverage_target_counts_as_reached_once_covered_on_the_way(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);g=room();b.grid=g;b.mode='mapping'
            b.localizer.pose=np.r_[g.world((30,30)),0];b.visits=[b.localizer.pose[:2].copy()]
            b.goal=g.world((70,70));b.goal_kind='coverage'
            self.assertFalse(b.coverage_goal_covered())
            b.visits.append(g.world((62,62)));self.assertTrue(b.coverage_goal_covered())
            b.goal_kind='frontier';self.assertFalse(b.coverage_goal_covered())
    def test_bumped_points_beside_round_body_allow_full_turn_without_escape(self):
        # Flipped from 'blocked full turn uses partial turn': these points (.20-.22 m from
        # base_link) touched the old box swept about its axle 4.2 cm ahead; the round body
        # turns about base_link and they lie outside TURN_CLEAR (R + TURN_MARGIN), so both full
        # turns are clear and the robot simply turns in place, with no turn_escape maneuver.
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);ring=open_floor(b)
            b.current_cloud=np.vstack((ring,[[.134,-.16],[.144,-.16],[.134,-.15]]))
            self.assertTrue(b.velocity_still_clear(0.,.4,(math.pi-.45)/.4))
            self.assertTrue(b.velocity_still_clear(0.,-.4,(math.pi-.45)/.4))
            v,w=b.follow_velocity(np.array([-1.,0.]),1.,0.);self.assertEqual(v,0.);self.assertAlmostEqual(abs(w),.4)
            self.assertIsNone(getattr(b,'maneuver',None))
    def test_turn_escape_is_only_a_harmless_fallback(self):
        # A point inside TURN_CLEAR right behind blocks every spin; the k>=1 partial
        # turns of turn_escape are then blocked too (the circle rule ignores the angle).
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);ring=open_floor(b);b.current_cloud=np.vstack((ring,[[-(TURN_CLEAR-.003),0.]]))
            m=b.turn_escape(np.array([-1.,0.]))
            # Drive straight forward 5 cm (away from the point behind), then turn: no partial turn.
            self.assertIsNotNone(m);self.assertEqual(m['turn'],0.);self.assertAlmostEqual(m['distance'],.05)
            self.assertNotEqual(m.get('kind'),'arc')
class RoundBodyTests(unittest.TestCase):
    """The round robot turns about base_link (drive_axle_x_m = 0)."""
    def test_kitchen_corner_turns_in_place_without_reverse_or_escape(self):
        # Nose-in in a corner: walls ahead and to the right, both R + 1.5 cm from
        # base_link, target behind. A spin changes no distance, so: turn in place.
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);ring=open_floor(b);g=R+.015
            b.current_cloud=np.vstack((ring,wall([g,-1.2],[g,1.2]),wall([-1.2,-g],[g,-g])))
            b.turn_escape=lambda target:self.fail('turn_escape must not be needed next to an obstacle')
            for stamp in (0.,1.,2.):
                b.motion_until=0;v,w=b.follow_velocity(np.array([-1.,0.]),1.,stamp)
                self.assertEqual(v,0.,'no reverse, no forward: a pure turn in place');self.assertGreater(abs(w),.1)
                self.assertLessEqual(abs(w),.4)
    def test_point_just_inside_turn_clear_blocks_the_spin(self):
        # Behaviour first (API the box controller also had), so a regression to the box/axle sweep
        # fails on geometry: at az 2.5 rad a point TURN_CLEAR + 2 mm from base_link is ~.205 m from
        # the old axle, inside the old ~.217 m sweep, so the box rule refuses the spin it must allow.
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);ring=open_floor(b)
            for az in (0.,1.,2.5,-2.):
                inside=(TURN_CLEAR-.002)*np.array([math.cos(az),math.sin(az)]);outside=(TURN_CLEAR+.002)*np.array([math.cos(az),math.sin(az)])
                b.current_cloud=np.vstack((ring,outside))
                self.assertTrue(b.velocity_still_clear(0.,.4,1.));self.assertTrue(b.rotation_clear())
                b.current_cloud=np.vstack((ring,inside))
                self.assertFalse(b.velocity_still_clear(0.,.4,1.));self.assertFalse(b.velocity_still_clear(0.,-.4,1.))
                self.assertFalse(b.rotation_clear())
                b.motion_until=0;v,w=b.follow_velocity(np.array([-1.,0.]),1.,0.)
                self.assertFalse(v==0 and w!=0,'a spin was commanded with a point inside TURN_CLEAR')
                from autonomy.local_planner import turn_clear
                self.assertFalse(turn_clear(np.vstack((ring,inside))));self.assertTrue(turn_clear(np.vstack((ring,outside))))
    def test_lidar_jitter_on_a_wall_is_averaged_per_beam_over_still_scans(self):
        # Flipped from 'averaged over 1 cm of surface' (spatial smoothing pushed the nearest point
        # of a corner outward): each beam is now averaged over the scans taken from this spot.
        for seed in range(10):
            with tempfile.TemporaryDirectory() as tmp:
                # Wall 5 mm outside TURN_CLEAR, sim2d jitter sigma 4 mm along each beam.
                b=LidarBrain(tmp);open_floor(b);w_=[((TURN_CLEAR+.005,-.3),(TURN_CLEAR+.005,.3))]
                feed_scans(b,w_,1,seed=seed)
                self.assertFalse(b.rotation_clear(),'one noisy scan has returns inside TURN_CLEAR')
                feed_scans(b,w_,8,seed=seed);self.assertEqual(b.spin_scan_count(),8);self.assertTrue(b.rotation_clear())
                # The same wall 3 mm inside TURN_CLEAR still blocks after averaging.
                b=LidarBrain(tmp);open_floor(b);feed_scans(b,[((TURN_CLEAR-.003,-.3),(TURN_CLEAR-.003,.3))],8,seed=seed)
                self.assertFalse(b.rotation_clear())
    def test_corner_and_post_just_inside_turn_clear_block_the_spin(self):
        # The closest point of a convex shape is a local minimum of range: averaging
        # neighbouring beams moved it outward by 5-7 mm. Per-beam averaging does not.
        from autonomy.local_planner import turn_clear
        cases=[('90 deg box corner',corner(TURN_CLEAR-.004)),('60 deg wedge',corner(TURN_CLEAR-.004,math.pi/6)),
               ('1 cm post',post(TURN_CLEAR-.004)),('8 mm chair leg 1.2 cm before a wall',post(TURN_CLEAR-.004,.004,.2)+[((TURN_CLEAR+.008,-1),(TURN_CLEAR+.008,1))])]
        for name,segs in cases:
            self.assertFalse(turn_clear(raycast(segs)),name+' (noise-free scan)')
            for seed in range(3):
                with tempfile.TemporaryDirectory() as tmp:
                    b=LidarBrain(tmp);open_floor(b);feed_scans(b,segs,8,seed=seed)
                    self.assertFalse(b.rotation_clear(),f'{name}, seed {seed}: averaged scans let the spin through')
                    self.assertFalse(b.velocity_still_clear(0.,.4,1.))
        # The same shapes 1 cm further out allow it.
        for name,segs in [('box corner',corner(TURN_CLEAR+.008)),('post',post(TURN_CLEAR+.008))]:
            with tempfile.TemporaryDirectory() as tmp:
                b=LidarBrain(tmp);open_floor(b);feed_scans(b,segs,8);self.assertTrue(b.rotation_clear(),name)
    def test_protrusion_beside_a_wall_blocks_the_spin(self):
        # 1-3 returns 3 mm inside TURN_CLEAR within 1.5 cm of a wall 5 mm outside it, and a
        # camera core cell inside TURN_CLEAR next to a LiDAR wall: each is checked as measured.
        from autonomy.local_planner import turn_clear
        wall_=wall([TURN_CLEAR+.005,-.3],[TURN_CLEAR+.005,.3],.0015)
        for n in (1,2,3):
            bump=np.c_[np.full(n,TURN_CLEAR-.003),np.linspace(-.002,.002,n)]
            self.assertFalse(turn_clear(np.vstack((wall_,bump))),f'{n} protruding returns')
        self.assertFalse(turn_clear(np.vstack((wall_,[[R-.0025,.004]]))),'a return inside the skin')
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);open_floor(b);b.current_cloud=np.vstack((open_floor(b),-wall_))
            # A camera core cell (5 cm grid) whose centre lies inside TURN_CLEAR, behind the robot.
            x=next(x for x in np.arange(-.10,-.20,-.005) if .12<np.linalg.norm(b.grid.world(b.grid.cell((x,0.))))<TURN_CLEAR-.003)
            b.grid.add_camera_obstacles(np.array([[x,0.]]),margin=0)
            core=b.spin_points();self.assertLess(np.linalg.norm(core,axis=1).min(),TURN_CLEAR)
            self.assertFalse(b.rotation_clear())
    def test_spin_refused_on_one_noisy_scan_waits_for_averaged_scans(self):
        # Wall 1.5 cm outside TURN_CLEAR: single scans have returns inside it; the robot stands
        # still and re-measures (no reverse, no escape), then turns once SPIN_SCANS (8) scans are averaged.
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);open_floor(b);segs=[((TURN_CLEAR+.015,-1.2),(TURN_CLEAR+.015,1.2))]
            b.turn_escape=lambda target:self.fail('no fallback while the beams are still being averaged')
            feed_scans(b,segs,1,noise=.008)
            self.assertFalse(b.rotation_clear());b.motion_until=0
            self.assertEqual(b.follow_velocity(np.array([-1.,0.]),1.,0.),(0.,0.))
            feed_scans(b,segs,8,noise=.008);b.motion_until=0;v,w=b.follow_velocity(np.array([-1.,0.]),1.,1.)
            self.assertEqual(v,0.);self.assertAlmostEqual(abs(w),.4)
    def test_kitchen_corner_with_lidar_noise_turns_in_place(self):
        # Unit test (b) with the sim2d range noise (4 mm sigma) instead of exact points: walls ahead
        # and to the right R + 1.5 cm from base_link (5 mm outside TURN_CLEAR), 8 scans from this
        # spot. The per-beam average resolves it as spin-clear: a pure turn, no arc, no escape.
        for seed in range(6):
            with tempfile.TemporaryDirectory() as tmp:
                b=LidarBrain(tmp);open_floor(b);g=R+.015
                b.turn_escape=lambda target:self.fail('turn_escape must not be needed next to an obstacle')
                feed_scans(b,[((g,-1.2),(g,1.2)),((-1.2,-g),(g,-g))],8,noise=.004,seed=seed)
                b.motion_until=0;v,w=b.follow_velocity(np.array([-1.,0.]),1.,10.)
                self.assertEqual(v,0.);self.assertGreater(abs(w),.1);self.assertIsNone(getattr(b,'maneuver',None))
    def test_spin_average_uses_the_scan_pose_not_the_odometry_advanced_pose(self):
        # Between scans predict() advances the localizer pose with the gyro. The earlier returns must
        # be averaged in the frame of the pose the CURRENT scan was taken at: .003 rad of odometry yaw
        # used to move them .5 mm sideways, out of the per-beam gate, and a spin-clear wall looked closer.
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);open_floor(b);g=R+.015
            feed_scans(b,[((g,-1.2),(g,1.2)),((-1.2,-g),(g,-g))],8,noise=.004,seed=3)
            b.scan_pose=b.localizer.pose.copy();before=float(np.linalg.norm(b.spin_points(),axis=1).min())
            self.assertTrue(b.rotation_clear());self.assertEqual(b.spin_scan_count(),8)
            b.localizer.pose=b.localizer.pose+[0.,0.,.003]      # gyro prediction, same scan
            self.assertAlmostEqual(float(np.linalg.norm(b.spin_points(),axis=1).min()),before,places=6)
            self.assertTrue(b.rotation_clear());self.assertEqual(b.spin_scan_count(),8)
            # A scan taken at another heading (while turning) has its beams on other lines: it is not
            # counted as an averaged scan, so spin_wait / escapes see the true count.
            b.scan_pose=b.localizer.pose+[0.,0.,.05];self.assertEqual(b.spin_scan_count(),1)
    def test_accepted_spin_is_not_aborted_by_a_noisy_mid_turn_scan(self):
        # A pure spin changes no distance. Once the full rule accepted it here, a scan taken while
        # turning (nothing to average with) may dip below TURN_CLEAR by noise without stopping it;
        # a point TURN_CLEAR - SPIN_RAW_ALLOWANCE closer (something new), drifting off the spot,
        # or not spinning any more brings back the full rule.
        from autonomy.lidar_brain import SPIN_RAW_ALLOWANCE
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);ring=open_floor(b);b.scan_pose=b.localizer.pose.copy()
            b.current_cloud=np.vstack((ring,wall([TURN_CLEAR+.005,-1],[TURN_CLEAR+.005,1])));b.requested_motion=(0.,0.)
            self.assertTrue(b.velocity_still_clear(0.,.4,1.))
            b.requested_motion=(0.,.4);noisy=np.vstack((ring,[[TURN_CLEAR-.008,0.]]))
            b.current_cloud=noisy;self.assertTrue(b.velocity_still_clear(0.,.4,1.),'noise dip mid-spin')
            b.current_cloud=np.vstack((ring,[[TURN_CLEAR-SPIN_RAW_ALLOWANCE-.002,0.]]));self.assertFalse(b.velocity_still_clear(0.,.4,1.))
            b.current_cloud=noisy;b.requested_motion=(0.,0.);self.assertFalse(b.velocity_still_clear(0.,.4,1.),'not spinning: full rule')
            b.requested_motion=(0.,.4);b.scan_pose=b.scan_pose+[.006,0.,0.];self.assertFalse(b.velocity_still_clear(0.,.4,1.),'drifted off the spot')
    def test_spin_room_escape_never_drives_forward_while_the_bumper_is_pressed(self):
        # DriveCore accepts only reverse or a pure spin while pressed; a forward arc would just disarm.
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);open_floor(b);g=R+.005
            feed_scans(b,[((g,-1.2),(g,1.2)),((-1.2,-g),(g,-g))],8,noise=0.)
            tried=[];b.arc_check=lambda v,w,length:tried.append(v)
            b.bumper_pressed=True;b.spin_room_escape();self.assertTrue(tried);self.assertTrue(all(v<0 for v in tried))
            tried.clear();b.bumper_pressed=False;b.spin_room_escape();self.assertIn(.04,tried)
    def test_route_closed_at_normal_clearance_continues_through_the_narrow_gap(self):
        # house2d rr_nocam: the route into the Study closed at the doorway three times (normal
        # clearance .25); each time the robot crossed the house to a far target and never mapped the
        # Study interior. The same target is now tried at NARROW_RADIUS first; lost again -> left.
        from autonomy.lidar_brain import NARROW_RADIUS
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);g=room(size=100);b.grid=g;b.mode='mapping'
            g.logodds[49,20:80]=5;g.logodds[49,46:54]=-5         # wall with an 8-cell gap: .20 m from its centre cells
            start=g.world((30,50));goal=g.world((70,50));b.localizer.pose=np.r_[start,0.];b.goal=goal.copy()
            self.assertFalse(g.path(start,goal),'closed at the normal clearance')
            b.route_lost(5.)
            self.assertEqual(b.phase,'follow');self.assertEqual(g.radius,NARROW_RADIUS);self.assertTrue(b.path);self.assertEqual(b.blacklist,[])
            b.route_lost(9.);self.assertEqual(len(b.blacklist),1,'lost again through the narrow gap: leave it')
    def test_no_blind_reverse_code_remains(self):
        # User rule: reverse only over ground the robot has already driven (and so seen); the
        # off-trail 'blind' back-off and arcs are gone entirely, not just disabled.
        import autonomy.lidar_brain as lb
        text=(CONTROLLER/'autonomy/lidar_brain.py').read_text()
        for name in ('CONTACT_BLIND_BACKOFF','BLIND_REVERSE','begin_blind_backoff','contact_blind','blind_reverse_needed','blind_reverses',"'blind'"):
            self.assertNotIn(name,text);self.assertFalse(hasattr(lb,name))
        self.assertLessEqual(lb.CLOSE_SPIN_W,.25)
        self.assertAlmostEqual(geometry.CLOSE_SPIN_CLEAR,R+.003);self.assertTrue(R<geometry.CLOSE_SPIN_CLEAR<TURN_CLEAR)
        # The contact spin is slow: within the controller's pressed-yaw limit and the motor core's pressed spin limit.
        self.assertLessEqual(lb.CONTACT_SPIN,min(lb.CLOSE_SPIN_W,lb.BUMP_YAW));self.assertEqual(lb.BUMP_YAW,.3)
        # Default: the motor core (DriveCore.h) passes a pure spin toward the pressed side unclamped.
        self.assertTrue(lb.PRESSED_SPIN_PURE)
        core=CONTROLLER.parent/'Source/HomeRobotSim/DriveCore.h'
        if core.exists():
            text=core.read_text().replace(' ','')
            # Pure spin while pressed (toward the pressed side, |w| <= bump_spin) and no per-wheel clamp; reverse cap.
            self.assertIn('bump_spin=.5',text);self.assertLessEqual(lb.CONTACT_SPIN,.5)
            self.assertIn('w*bump_side>=0',text);self.assertNotIn('ls=std::min(ls,0.)',text,'DriveCore.h clamps again: set PRESSED_SPIN_PURE False')
            self.assertIn('max_reverse=.06',text)
    def test_sim2d_spec_firmware_disarms_on_every_new_press(self):
        # DriveCore.h update() (line 46: press=bumper&&!bump ... if(hazard||press||estop||...)disarm()) disarms on every new
        # press, whatever the current command; disarm() zeroes the wheel setpoints and only command() re-arms, for a
        # pressed-allowed motion. sim2d's default 'spec' firmware models that DriveCore.h (task F: it used to keep a
        # pressed-allowed spin or reverse armed, ramping on into the obstacle).
        from sim2d import World,Sim,SKIN_R,SKIN_REACH
        core=CONTROLLER.parent/'Source/HomeRobotSim/DriveCore.h'
        if core.exists():
            text=core.read_text().replace(' ','')
            self.assertIn('constboolpress=bumper&&!bump',text);self.assertIn('if(hazard||press||estop',text)
        for cmd in ((0.,.25),(0.,-.25),(-.03,0.)):            # pure spins (centre press: both switches, either way) and a reverse
            sim=Sim(World([((.17,-1.),(.17,1.))]),pose=(0.,0.,0.))   # a wall .17 m ahead: the skin and its trip reach are clear
            for _ in range(20):sim.command(*cmd,arm=True);sim.step(.01)
            self.assertTrue(sim.armed);self.assertFalse(sim.bump)
            sim.pose[0]=.17-SKIN_R-SKIN_REACH/2              # the wall now within the trip reach: a press rising edge
            sim.step(.01)
            self.assertTrue(sim.bump);self.assertFalse(sim.armed,f'{cmd}: a new press must disarm');self.assertEqual((sim.wl,sim.wr),(0.,0.))
            sim.command(*cmd,arm=True);self.assertTrue(sim.armed,f'{cmd}: command() re-arms a pressed-allowed motion')
            sim.step(.01);self.assertLessEqual(max(abs(sim.wl),abs(sim.wr)),.25*.01+1e-12,'the wheel setpoints restart from 0')
    def test_close_spin_rule(self):
        # Beam-averaged points >= R + 3 mm: a slow spin is allowed although they are inside TURN_CLEAR.
        # Front-half points are verified by the released skin (LiDAR not more than 5 mm inside it); a
        # point just behind a skin end counts only for the direction that turns it into the front.
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);ring=open_floor(b);feed_scans(b,[],8,noise=0.)
            at=lambda r,deg:r*np.array([[math.cos(math.radians(deg)),math.sin(math.radians(deg))]])
            ok=lambda pts,s:b.close_spin_clear(s,np.vstack((ring,pts)))
            for s in (1.,-1.):
                self.assertTrue(ok(at(R+.0035,150),s),'rear point >= R + 3 mm');self.assertFalse(ok(at(R+.002,150),s),'rear point < R + 3 mm')
                self.assertTrue(ok(at(R-.003,30),s),'front point, skin released');self.assertFalse(ok(at(R-.007,30),s),'LiDAR well inside the skin')
            # 100 deg (10 deg behind the left skin end): a + spin (w > 0) moves it to 90 deg first.
            self.assertTrue(ok(at(R+.001,100),1.));self.assertFalse(ok(at(R+.001,100),-1.))
            self.assertTrue(ok(at(R+.001,-100),-1.));self.assertFalse(ok(at(R+.001,-100),1.))
            self.assertFalse(ok(at(R+.001,115),1.),'more than CLOSE_SPIN_ENTRY_DEG behind the skin end')
            b.bumper_pressed=True;b.bumper_sides=(True,False)
            self.assertFalse(ok(at(R+.001,30),1.),'pressed: the skin no longer verifies the front')
            self.assertTrue(ok(at(R+.0035,30),1.))
            # A clamping motor core (the previous DriveCore.h): no spin at all while pressed (it would pivot backward).
            b.pressed_spin_pure=False
            for sides in ((True,False),(False,True),(True,True)):
                b.bumper_sides=sides;self.assertFalse(b.spin_allowed(1.) or b.spin_allowed(-1.),sides)
            # Pure-spin core: pressed left switch -> only w >= 0; right -> w <= 0; both -> either.
            b.pressed_spin_pure=True;b.bumper_sides=(True,False)
            self.assertTrue(b.spin_allowed(1.));self.assertFalse(b.spin_allowed(-1.))
            b.bumper_sides=(False,True);self.assertFalse(b.spin_allowed(1.));self.assertTrue(b.spin_allowed(-1.))
            b.bumper_sides=(True,True);self.assertTrue(b.spin_allowed(1.) and b.spin_allowed(-1.))
            b.bumper_pressed=False;b.bumper_sides=(False,False);self.assertTrue(b.spin_allowed(1.) and b.spin_allowed(-1.))
            # Too few averaged scans: no close spin (re-measure first).
            b=LidarBrain(tmp);ring=open_floor(b);feed_scans(b,[],2,noise=0.);self.assertFalse(b.close_spin_clear(1.,np.vstack((ring,at(R+.0035,150)))))
    def test_kitchen_corner_3_5_10_15mm_turns_in_place_with_no_reverse(self):
        # Nose-in in a corner with NO driven trail, target behind, 4 mm-sigma LiDAR noise, the scene
        # re-scanned from the current pose: the robot never reverses. It waits for averaged beams,
        # turns in place (slowly, <= .25 rad/s, when the averaged points are inside TURN_CLEAR; .4 at
        # 15 mm where they are not), then leaves forward along a line that moves away from both walls,
        # and keeps driving (the exit and the first route motions, 10 cm) without any reverse command.
        # Side layout (wall ahead and on the right): the close spin turns right (w < 0), keeping the
        # right wall on the bumper half. The gyro reads the commanded turn (no stall).
        from types import SimpleNamespace
        from autonomy.lidar_brain import CLOSE_SPIN_W
        for gap in (.003,.005,.010,.015):
            for layout in ('diagonal','side'):
                with tempfile.TemporaryDirectory() as tmp:
                    b=LidarBrain(tmp);ring=open_floor(b);segs=corner_walls(gap,layout);t=10.;cmds=[];left_at=None
                    self.assertEqual(len(b.trail),0)
                    for k in range(1100):
                        if k%3==0:
                            b.current_cloud=np.vstack((raycast(seen_from(segs,b.localizer.pose),.004,int(gap*1000)*7919+k),ring))
                            b.frame=SimpleNamespace(stamp=t,index=k);b.scan_pose=b.localizer.pose.copy();b.note_spin_scan(b.current_cloud,b.localizer.pose,t)
                            b.record_trail(b.localizer.pose,t)
                        v,w=b.follow_velocity(np.array([-1.,0.]),1.,t);b.requested_motion=(v,w);b.gyro_z=w
                        cmds.append((v,w,getattr(b,'maneuver',None) and b.maneuver.get('kind')))
                        self.assertGreaterEqual(v,0.,f'{gap} {layout}: reverse commanded with no driven trail')
                        if v>0 and left_at is None:left_at=len(cmds)-1;exit_yaw=b.localizer.pose[2]
                        p=b.localizer.pose;p[:2]+=v*.05*np.array([math.cos(p[2]),math.sin(p[2])]);p[2]+=w*.05;t+=.05
                        if left_at is not None and np.linalg.norm(p[:2])>.10:break
                    name=f'{gap*1000:.0f} mm {layout}'
                    self.assertIsNotNone(left_at,name+': never left the corner')
                    self.assertGreater(np.linalg.norm(b.localizer.pose[:2]),.10,name+': did not drive 10 cm away')
                    spins=[(v,w,kind) for v,w,kind in cmds[:left_at] if w]     # before the first forward command
                    self.assertTrue(spins,name+': no turn in place')
                    self.assertTrue(all(v==0 for v,w,_ in spins),name+': turned while translating')
                    self.assertGreater(abs(wrap_(exit_yaw)),1.,name+': did not turn away from the corner')
                    close=[w for v,w,kind in spins if kind=='close_spin']
                    if gap<.01+1e-9:self.assertTrue(close,name+': inside TURN_CLEAR, expected a close spin')
                    self.assertTrue(all(abs(w)<=CLOSE_SPIN_W+1e-9 for w in close),name)
                    if layout=='side' and close:self.assertTrue(all(w<0 for w in close),name+': should keep the right wall on the bumper half')
                    # The forward line it leaves along moves away from both walls (true geometry).
                    u=np.array([math.cos(exit_yaw),math.sin(exit_yaw)]);wall_pts=np.vstack([wall(a,b_,.001) for a,b_ in segs])
                    gaps=[np.linalg.norm(wall_pts-s*u,axis=1).min() for s in np.linspace(0,.05,11)]
                    self.assertGreaterEqual(min(gaps),gaps[0]-1e-4,name+': the exit approaches a wall')
    def test_no_reverse_off_an_empty_trail_property(self):
        # Property: with an EMPTY driven trail no escape or contact function ever commands a reverse,
        # whatever is nearby (random walls / posts / camera cells from inside the skin to 6 cm out,
        # any bumper state).
        from types import SimpleNamespace
        from autonomy.local_planner import choose_velocity
        rng=np.random.default_rng(11)
        def drive(b,target,t0):
            out=[]
            for k in range(12):
                t=t0+.05*k
                m=b.turn_escape(target)
                if m:out.append(('turn_escape',m.get('v',.05 if m.get('distance') else 0.)))
                m=b.close_spin_maneuver(target)
                if m:out.append(('close_spin_maneuver',0.))
                m=b.spin_room_escape()
                if m:out.append(('spin_room_escape',m['v']))
                out.append(('choose_velocity',choose_velocity(b,target,1.)[0]))
                v,w=b.follow_velocity(target,1.,t);b.requested_motion=(v,w);b.gyro_z=w;out.append(('follow_velocity',v))
                b.localizer.pose[2]+=w*.05
            return out
        cases=0
        for case in range(30):
            with tempfile.TemporaryDirectory() as tmp:
                b=LidarBrain(tmp);b.pressed_spin_pure=bool(case%2);ring=open_floor(b);segs=random_scene(rng)
                feed_scans(b,segs,8,seed=case)
                if rng.random()<.3:b.grid.add_camera_obstacles(np.array([[rng.uniform(-.3,.3),rng.uniform(-.3,.3)]]),margin=0)
                b.bumper_pressed=bool(rng.random()<.3);b.bumper_sides=(bool(rng.random()<.5),bool(rng.random()<.5)) if b.bumper_pressed else (False,False)
                target=rng.uniform(-1,1,2)
                self.assertEqual(len(b.trail),0)
                for name,v in drive(b,target,10.):self.assertGreaterEqual(v,0.,f'case {case}: {name} reversed with no driven trail')
                # Contact handling with no trail: any zone, pressed or released, never a reverse.
                ready(b,20.);b.phase='follow';b.bumper_pressed=False
                bits=int(rng.integers(0,32));b.predict(dict(sim_time_s=20.,fault='bumper',armed=False,sensors=dict(bumper=True,bump_zones=bits,left_ticks=0,right_ticks=0,encoder_cpr=4741.44,gyro_rad_s=[0,0,0],specific_force_m_s2=[0,0,9.8])))
                for k in range(260):
                    t=20.+.05*k;ready(b,t,stamp_only=True)
                    if k==120:b.bumper_pressed=bool(rng.random()<.5)
                    # drive_command: the contact / escape logic itself, before the reverse guard in command().
                    v,w=b.drive_command(dict(sim_time_s=t,armed=False,fault='bumper' if b.bumper_pressed else 'none'))
                    self.assertGreaterEqual(v,0.,f'case {case}: contact phase reversed with no driven trail ({b.phase})')
                    # Clamping motor core: a pressed spin would pivot backward, so none is ever asked for here,
                    # and the final guard sends nothing the core would not execute as commanded.
                    if b.bumper_pressed and not b.pressed_spin_pure:self.assertEqual(w,0.,f'case {case}: pressed spin with the clamping core')
                    if b.bumper_pressed and (v,w)!=(0.,0.):self.assertTrue(b.firmware_executes(v,w),f'case {case}: {(v,w)} pressed')
                    b.requested_motion=(v,w);b.gyro_z=w
                    b.localizer.pose[2]+=w*.05
                    if b.mode=='idle':break
                cases+=1
        self.assertEqual(cases,30)
    @staticmethod
    def driven_distance(q,runs):
        """Distance from q to the ground driven forward or turned on (the test's own record): the runs are
        polylines of positions reached by forward ticks, or by turn ticks at a spot reached driving forward,
        split at every reverse (no chord across reversed-over ground counts)."""
        best=np.inf
        for run in runs:
            path=np.asarray(run,float).reshape(-1,2)
            if not len(path):continue
            best=min(best,float(np.linalg.norm(path-q,axis=1).min()))
            if len(path)>1:
                a=path[:-1];ab=path[1:]-a;l2=np.maximum(1e-12,(ab*ab).sum(1))
                t=np.clip(((q-a)*ab).sum(1)/l2,0,1);best=min(best,float(np.linalg.norm(a+t[:,None]*ab-q,axis=1).min()))
        return best
    def closed_loop(self,b,plan,t=1.):
        """Run command() over a list of drive_command outputs, integrating the pose (50 ms ticks) and
        recording the trail every 3 ticks as observe_scan does (20 Hz commands, ~6.7 Hz scans). Returns
        (worst distance of a reverse-reached position from the driven record, longest reverse episode,
        metres reversed)."""
        runs=[[b.localizer.pose[:2].copy()]];episode=0.;fwd_since=0.;worst=0.;longest=0.;reversed_m=0.;since_forward=True
        for k,want in enumerate(plan):
            b.drive_command=lambda state,want=want:want
            ready(b,t,stamp_only=True);v,w=b.command(dict(sim_time_s=t,armed=True,fault='none'));b.requested_motion=(v,w)
            p=b.localizer.pose;before=p[:2].copy()
            p[:2]+=v*.05*np.array([math.cos(p[2]+w*.025),math.sin(p[2]+w*.025)]);p[2]+=w*.05;step=float(np.linalg.norm(p[:2]-before))
            if v<0:
                episode+=step;fwd_since=0.;longest=max(longest,episode);reversed_m+=step;since_forward=False
                worst=max(worst,self.driven_distance(p[:2],runs))
                if runs[-1]:runs.append([])
            elif v>0 or since_forward:
                if v>0:since_forward=True
                runs[-1].append(p[:2].copy())
                if v>0:fwd_since+=step
                if fwd_since>=.05:episode=0.
            t+=.05
            if k%3==2:b.record_trail(p,t)
        return worst,longest,reversed_m
    def test_reverse_stays_on_the_forward_driven_trail_closed_loop(self):
        # Closed loop, adversarial: drive_command is replaced by a random policy that keeps asking to
        # reverse (straight, arcs, after spins at the trail end), the pose integrates every command and
        # the trail grows exactly as observe_scan grows it (record_trail, every 3rd tick). Checked
        # INDEPENDENTLY of the controller's own trail: every position reached under a reverse command lies
        # within the 2.5 cm band of the positions the robot reached driving forward (or turning at such a
        # spot), with the record split at every reverse (closed_loop), and no reverse episode (reverses
        # not separated by 5 cm of forward travel) exceeds REVERSE_LIMIT.
        # The spec's numbers, not the module's (a test must not move with the code it checks).
        REVERSE_LIMIT,TRAIL_TOL=.15,.025
        rng=np.random.default_rng(5);reversed_total=0.
        choices=[(.05,0.),(.05,.4),(0.,.4),(0.,-.4),(-.05,0.),(-.05,0.),(-.05,.2),(-.05,-.2),(-.04,0.),(0.,0.),(-.05,-.4),(-.05,.4)]
        for case in range(24):
            with tempfile.TemporaryDirectory() as tmp:
                b=LidarBrain(tmp);open_floor(b,radius=3.);ready(b,1.);b.phase='follow'
                plan=[choices[i] for i in rng.integers(0,len(choices),40) for _ in range(int(rng.integers(5,40)))]
                worst,longest,reversed_m=self.closed_loop(b,plan);reversed_total+=reversed_m
                self.assertLessEqual(worst,TRAIL_TOL+.001,f'case {case}: a reverse left the forward-driven trail by {worst:.4f} m')
                self.assertLessEqual(longest,REVERSE_LIMIT+.003,f'case {case}: one reverse episode covered {longest:.3f} m')
        self.assertGreater(reversed_total,.3,'the policy must actually exercise reversing')
    def test_reversed_to_position_never_joins_the_trail(self):
        # Reviewer's chord probe: drive 40 cm along +x, reverse 3 cm, turn +90 deg, drive 4.5 cm forward,
        # turn to 160 deg, then keep asking to reverse (-.05, -.4). It used to record a sample reached by
        # reversing and join it to its neighbours (a chord over ground base_link never covered): base_link
        # then reached 2.96 cm from any forward-driven position. Now: within the 2.5 cm band.
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);open_floor(b,radius=3.);b.localizer.pose=np.array([-.40,0.,0.]);ready(b,1.);b.phase='follow'
            plan=[(.05,0.)]*160+[(-.05,0.)]*12+[(0.,.4)]*int(round(math.pi/2/.02))+[(.05,0.)]*18+[(0.,.4)]*int(round(math.radians(70)/.02))+[(-.05,-.4)]*120
            worst,longest,reversed_m=self.closed_loop(b,plan)
            self.assertGreater(reversed_m,.03);self.assertLessEqual(worst,.025+1e-6,f'{worst:.4f} m off the forward-driven trail')
    def test_reverse_guard_closed_loop(self):
        # The final guard in command(), with the trail growing as in the real loop (record_trail).
        REVERSE_LIMIT,TRAIL_TOL,TRAIL_CAP_TOL=.15,.025,.005     # the spec's numbers, not the module's
        def run(b,want,ticks,t=1.):
            b.drive_command=lambda state:want;n=0
            for k in range(ticks):
                ready(b,t,stamp_only=True);v,w=b.command(dict(sim_time_s=t,armed=True,fault='none'));b.requested_motion=(v,w)
                if v>=0:break
                p=b.localizer.pose;p[:2]+=v*.05*np.array([math.cos(p[2]),math.sin(p[2])]);p[2]+=w*.05;t+=.05;n+=1;b.record_trail(p,t)
            return n
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);open_floor(b);ready(b,1.);b.phase='follow'
            self.assertEqual(run(b,(-.05,0.),5),0,'empty trail: no reverse');self.assertIn('Reverse refused',b.message)
            # The trail records only positions reached driving forward or turning: none while reversing
            # or during the wheel ramp after it (REVERSE_SETTLE_S).
            b.trail.append(np.zeros(2));b.requested_motion=(-.05,0.);b.record_trail(np.array([-.02,0.,0.]),2.);self.assertEqual(len(b.trail),1)
            b.last_reverse_stamp=2.;b.requested_motion=(.05,0.);b.record_trail(np.array([-.03,0.,0.]),2.2);self.assertEqual(len(b.trail),1)
            b.record_trail(np.array([-.04,0.,0.]),2.6);self.assertEqual(len(b.trail),2);b.trail.clear()
            # After a reverse sent by command(), a timer is not enough: nothing is recorded until base_link
            # has driven FORWARD_CLEAR_M (1 cm) forward again, and that first sample starts a new run (not
            # joined to the samples before the reverse).
            b=LidarBrain(tmp);open_floor(b);ready(b,1.);b.phase='follow'
            for s in np.arange(.20,-1e-9,-.01):b.trail.append(np.array([-s,0.]))
            b.localizer.pose=np.array([0.,0.,0.]);self.assertGreater(run(b,(-.05,0.),10),5);n=len(b.trail)
            b.drive_command=lambda state:(0.,.4);t=5.
            for k in range(20):ready(b,t,stamp_only=True);v,w=b.command(dict(sim_time_s=t,armed=True,fault='none'));b.requested_motion=(v,w);b.localizer.pose[2]+=w*.05;t+=.05;b.record_trail(b.localizer.pose,t)
            self.assertEqual(len(b.trail),n,'turning at a reversed-to spot records nothing')
            self.assertTrue(np.allclose(b.trail_head(),0.),'the trail head is where the reverse began (driven forward), not the reversed-to spot')
            b.drive_command=lambda state:(.05,0.);h=b.localizer.pose[2];u=np.array([math.cos(h),math.sin(h)])
            for k in range(3):ready(b,t,stamp_only=True);v,w=b.command(dict(sim_time_s=t,armed=True,fault='none'));b.requested_motion=(v,w);b.localizer.pose[:2]+=v*.05*u;t+=.05;b.record_trail(b.localizer.pose,t)
            self.assertEqual(len(b.trail),n,'less than 1 cm forward: nothing yet')
            for k in range(6):ready(b,t,stamp_only=True);v,w=b.command(dict(sim_time_s=t,armed=True,fault='none'));b.requested_motion=(v,w);b.localizer.pose[:2]+=v*.05*u;t+=.05;b.record_trail(b.localizer.pose,t)
            self.assertGreater(len(b.trail),n);self.assertIn(tuple(float(x) for x in b.trail[n]),set(b.trail_breaks))
            a,c,_,_=b.trail_segments();self.assertFalse(any(np.allclose(q,b.trail[n-1]) and np.allclose(r,b.trail[n]) for q,r in zip(a,c)),'joined across the reverse')
            # Drove 40 cm along +x to the origin. (a) Straight back along it: stops at the episode budget.
            for s in np.arange(.40,-1e-9,-.01):b.trail.append(np.array([-s,0.]))
            b.localizer.pose=np.array([0.,0.,0.]);b.reverse_episode=None
            run(b,(-.05,0.),200);self.assertAlmostEqual(b.localizer.pose[0],-REVERSE_LIMIT,delta=.004)
            self.assertIn('reverse limit',b.message)
            # (b) Spun 90 deg at the trail end, then reversing across it (the reviewer's probe: it used to run
            # 0.5 m off the trail because each reversed pose joined the trail): stops inside the band.
            b=LidarBrain(tmp);open_floor(b);ready(b,1.);b.phase='follow'
            for s in np.arange(.40,-1e-9,-.01):b.trail.append(np.array([-s,0.]))
            b.localizer.pose=np.array([0.,0.,math.pi/2]);run(b,(-.05,0.),200)
            self.assertLessEqual(abs(b.localizer.pose[1]),TRAIL_TOL);self.assertGreater(len(b.trail),0)
            # (c) Reversing past the far END of the trail (its oldest sample): the guard's REVERSE_GUARD_M
            # look-ahead may reach at most TRAIL_CAP_TOL beyond it (the 2.5 cm band is lateral only).
            from autonomy.lidar_brain import REVERSE_GUARD_M
            b=LidarBrain(tmp);open_floor(b);ready(b,1.);b.phase='follow'
            for s in np.arange(.08,-1e-9,-.01):b.trail.append(np.array([-s,0.]))
            b.localizer.pose=np.array([0.,0.,0.]);run(b,(-.05,0.),200)
            self.assertGreaterEqual(b.localizer.pose[0]-REVERSE_GUARD_M,-.08-TRAIL_CAP_TOL-.003);self.assertLess(b.localizer.pose[0],-.05)
            # (d) A lone start sample (what observe_scan records before the robot drives) allows no reverse.
            b=LidarBrain(tmp);open_floor(b);ready(b,1.);b.phase='follow';b.trail.append(np.zeros(2))
            self.assertLessEqual(run(b,(-.05,0.),50)*.0025,TRAIL_CAP_TOL)
            # (e) The episode budget is restored only after forward driving of max(5 cm, the distance
            # reversed): 6 cm forward after a 15 cm reverse is not enough (forward / back cycles must not
            # walk the robot back along its trail), 15 cm is. After a 3 cm reverse, 5 cm restores it.
            b=LidarBrain(tmp);open_floor(b);ready(b,1.);b.phase='follow'
            for s in np.arange(.60,-1e-9,-.01):b.trail.append(np.array([-s,0.]))
            b.localizer.pose=np.array([0.,0.,0.]);run(b,(-.05,0.),200);self.assertLess(b.reverse_budget(),.01)
            b.drive_command=lambda state:(.05,0.);t=5.
            def forward(metres):
                nonlocal t
                start=b.localizer.pose[0]
                while b.localizer.pose[0]-start<metres-1e-9:
                    ready(b,t,stamp_only=True);v,w=b.command(dict(sim_time_s=t,armed=True,fault='none'));b.requested_motion=(v,w);b.localizer.pose[0]+=v*.05;t+=.05;b.record_trail(b.localizer.pose,t)
            forward(.06);self.assertLess(b.reverse_budget(),.01)
            forward(.10);self.assertEqual(b.reverse_budget(),REVERSE_LIMIT)
            run(b,(-.05,0.),12,t);self.assertLess(b.reverse_budget(),REVERSE_LIMIT-.02)
            b.drive_command=lambda state:(.05,0.);forward(.055);self.assertEqual(b.reverse_budget(),REVERSE_LIMIT)
    def test_visit_path_never_permits_a_reverse(self):
        # The coarse visit path (15-20 cm chords) is not driven ground: an L-shaped drive (14 cm east,
        # 6 cm north) leaves the chord (0,0)-(.14,.06) up to 5.9 cm from where the body drove.
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);open_floor(b)
            path=[np.array([x,0.]) for x in np.arange(0,.1401,.01)]+[np.array([.14,y]) for y in np.arange(.01,.0601,.01)]
            for q in path:
                if not b.visits or np.linalg.norm(q-b.visits[-1])>.15:b.visits.append(q.copy())
            self.assertEqual(len(b.visits),2)
            chord=.5*(b.visits[0]+b.visits[1]);self.assertFalse(b.on_trail(chord[None]))
            b.localizer.pose=np.array([.14,.06,math.atan2(.06,.14)]);self.assertEqual(b.reverse_room(),0.)
            ready(b,1.);b.phase='follow';b.drive_command=lambda state:(-.05,0.)
            self.assertEqual(b.command(dict(sim_time_s=1.,armed=True,fault='none')),(0.,0.))
            # Along the driven trail itself the same reverse is fine.
            for q in path:b.trail.append(q)
            b.localizer.pose=np.array([.14,.06,math.pi/2]);self.assertGreater(b.reverse_room(),.05)
    def test_go_to_starts_without_the_mapping_visits(self):
        from autonomy.lidar import LidarLocalizer
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);folder=root/'Saved/Maps/test';folder.mkdir(parents=True);loc=LidarLocalizer()
            a=np.linspace(-np.pi,np.pi,400);loc.points=np.c_[2*np.cos(a),2*np.sin(a)];loc.keyframes=[{'pose':np.array([0.,0,0])},{'pose':np.array([.5,0,0])}];loc.save(folder)
            g=LidarGrid();g.logodds[145:176,145:176]=-5;g.seen[145:176,145:176]=4;g.save(folder);(folder.parent/'latest-lidar-map.json').write_text('{"directory":"test"}')
            b=LidarBrain(root);b.visits=[np.array([x,0.]) for x in np.arange(-1.,.01,.16)];b.trail.append(np.zeros(2))
            b.begin_go_to([0,0],1);self.assertEqual(b.visits,[]);self.assertEqual(len(b.trail),0)
            b.localizer.pose=np.array([-.05,0.,0.]);self.assertEqual(b.reverse_room(),0.)
    def test_contact_with_empty_trail_spins_toward_the_contact_and_never_reverses(self):
        # Pure-spin motor core (the spec's rule): a pressed contact with no driven trail turns in place
        # TOWARD the contact (it moves toward 0 deg), slowly, never reversing. The clamping DriveCore.h core
        # (default) would pivot that spin backward, so there the robot stands still (w == 0) and waits.
        from types import SimpleNamespace
        from autonomy.lidar_brain import CONTACT_SPIN
        for (bit,sign),pure in [(c,p) for c in ((4,1.),(3,1.),(0,-1.),(1,-1.),(2,0.)) for p in (True,False)]:
            if not pure:sign=0.
            with tempfile.TemporaryDirectory() as tmp:
                b=LidarBrain(tmp);b.pressed_spin_pure=pure;ring=open_floor(b);b.current_cloud=np.vstack((ring,wall([R+.001,-.3],[R+.001,.3])));ready(b,1.)
                self.assertEqual(len(b.trail),0)
                sides=b.pressed_sides(dict(bump_zones=1<<bit))
                b.predict(dict(sim_time_s=1.,fault='bumper',armed=False,sensors=dict(bumper=True,bump_zones=1<<bit,left_ticks=0,right_ticks=0,encoder_cpr=4741.44,gyro_rad_s=[0,0,0],specific_force_m_s2=[0,0,9.8])))
                self.assertEqual(b.phase,'contact');self.assertEqual(b.backoff,0.);self.assertEqual(b.bumper_sides,sides)
                v,w=b.command(dict(sim_time_s=1.,armed=False,fault='bumper'))
                self.assertEqual(v,0.,'no reverse without a driven trail')
                if sign:
                    # Toward the contact (it moves toward 0 deg), the direction the firmware allows.
                    self.assertEqual(math.copysign(1,w),sign);self.assertTrue(b.spin_allowed(w));self.assertLessEqual(abs(w),CONTACT_SPIN)
                    b.localizer.pose[2]=b.contact_spin
                else:self.assertEqual(w,0.)
                # Contact in front, still pressed: stand still (never reverse) and keep the session alive,
                # waiting for the bumper to release (up to CONTACT_WAIT_S: see the test below).
                for t in np.arange(1.1,40.,.5):
                    ready(b,t,stamp_only=True);v,w=b.command(dict(sim_time_s=t,armed=False,fault='bumper'));self.assertEqual((v,w),(0.,0.))
                self.assertEqual(b.mode,'mapping');self.assertEqual(b.phase,'contact');self.assertIn('Waiting for the bumper to release',b.message)
                # The skin releases: replan at once.
                b.bumper_pressed=False;b.bumper_sides=(False,False);ready(b,40.,stamp_only=True)
                self.assertEqual(b.command(dict(sim_time_s=40.,armed=True,fault='none')),(0.,0.));self.assertEqual(b.phase,'plan')
                # Released instead: replan (the planner then turns in place or drives away).
                b=LidarBrain(tmp);b.pressed_spin_pure=pure;ring=open_floor(b);b.current_cloud=ring;ready(b,1.)
                b.predict(dict(sim_time_s=1.,fault='bumper',armed=False,sensors=dict(bumper=True,bump_zones=1<<bit,left_ticks=0,right_ticks=0,encoder_cpr=4741.44,gyro_rad_s=[0,0,0],specific_force_m_s2=[0,0,9.8])))
                b.bumper_pressed=False;b.bumper_sides=(False,False)
                self.assertEqual(b.command(dict(sim_time_s=1.05,armed=True,fault='none')),(0.,0.));self.assertEqual(b.phase,'plan')
    # Task G helpers: a pressed contact on the front centre (zone 2), and wheel ticks per metre.
    PRESS=dict(bumper=True,bump_zones=1<<2,encoder_cpr=4741.44,gyro_rad_s=[0,0,0],specific_force_m_s2=[0,0,9.8])
    PER_M=4741.44/(2*math.pi*geometry.WHEEL_R)
    def wheels(self,b,t,run,spin=0.,pressed=False,motion=None):
        """One predict() with both wheels at `run` m of travel (+/- `spin` m for a turn), requested_motion first."""
        if motion is not None:b.requested_motion=motion
        sensors=dict(self.PRESS,left_ticks=(run-spin)*self.PER_M,right_ticks=(run+spin)*self.PER_M)
        if not pressed:sensors.update(bumper=False,bump_zones=0)
        b.predict(dict(sim_time_s=t,fault='bumper' if pressed else 'none',armed=not pressed,sensors=sensors))
    def test_forward_run_is_the_net_forward_drive_since_the_last_turn_in_place(self):
        # CONTACT_*: what a creep may undo. Forward wheel travel adds, reversing takes it back (never below 0), a turn
        # in place (v == 0, w != 0) resets it; arcs and standing still keep it.
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);open_floor(b);b.mode='mapping'
            self.wheels(b,1.,0.,motion=(0.,0.));self.assertEqual(b.forward_run,0.)
            self.wheels(b,1.05,.03,motion=(.1,0.));self.assertAlmostEqual(b.forward_run,.03,places=4)
            self.wheels(b,1.1,.05,motion=(.1,.4));self.assertAlmostEqual(b.forward_run,.05,places=4)
            self.wheels(b,1.15,.05,motion=(0.,0.));self.assertAlmostEqual(b.forward_run,.05,places=4)
            self.wheels(b,1.2,.04,motion=(-.05,0.));self.assertAlmostEqual(b.forward_run,.04,places=4)
            self.wheels(b,1.25,-.02,motion=(-.05,0.));self.assertEqual(b.forward_run,0.)
            self.wheels(b,1.3,.0,motion=(.1,0.));self.assertAlmostEqual(b.forward_run,.02,places=4)
            self.wheels(b,1.35,.0,spin=.01,motion=(0.,.4));self.assertEqual(b.forward_run,0.,'a turn in place')
            self.wheels(b,1.4,.008,spin=.01,motion=(.1,0.));self.assertAlmostEqual(b.forward_run,.008,places=4)
    def test_pressed_back_off_is_planned_again_when_the_planned_length_stops_fitting(self):
        # Unreal trial full-mapping-20261011T054731 (the shower tray): the straight back-off was planned at 12 cm along a
        # curved trail, the next LiDAR scan moved the pose estimate and the straight line then fitted 1 mm short of the plan;
        # the back-off was dropped whole and the robot waited, pressed, until the time budget ended the run (1110 s).
        # Here: a trail curving left (radius .25 m, like the local planner's .4 rad/s commits at 10 cm/s), a pose correction
        # of 8 mm to the right and .04 rad after the contact. The back-off goes on, re-planned shorter (CONTACT_ATTEMPTS bound it).
        from autonomy.lidar_brain import CONTACT_ATTEMPTS
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);ring=open_floor(b);b.current_cloud=np.vstack((ring,wall([R+.001,-.3],[R+.001,.3])));ready(b,1.)
            for th in np.arange(.8,-1e-9,-.04):b.trail.append(np.array([-.25*math.sin(th),.25*(1-math.cos(th))]))
            b.predict(dict(sim_time_s=1.,fault='bumper',armed=False,sensors=dict(self.PRESS,left_ticks=0,right_ticks=0)))
            self.assertEqual(b.phase,'contact');self.assertAlmostEqual(b.backoff,.065);self.assertEqual(b.contact_attempts,1)
            self.assertEqual(b.command(dict(sim_time_s=1.,armed=True,fault='bumper')),(-.05,0.))
            b.localizer.pose=np.array([0.,-.008,.04])
            self.assertFalse(b.straight_corridor_clear(-1,b.backoff-np.linalg.norm(b.localizer.pose[:2]-b.escape_origin[:2]),True),'precondition: the rest no longer fits')
            ready(b,1.05,stamp_only=True);self.assertEqual(b.command(dict(sim_time_s=1.05,armed=True,fault='bumper')),(0.,0.))
            self.assertEqual(b.phase,'contact');self.assertAlmostEqual(b.backoff,.035);self.assertEqual(b.contact_attempts,2);self.assertIn('Still pressed. Reversing 4 cm',b.message)
            ready(b,1.1,stamp_only=True);self.assertEqual(b.command(dict(sim_time_s=1.1,armed=True,fault='bumper')),(-.05,0.))
            # Bounded: with the attempts used up, a refused rest is not planned again; the robot waits for the release.
            b.contact_attempts=CONTACT_ATTEMPTS;b.localizer.pose=np.array([0.,-.03,0.])
            ready(b,1.15,stamp_only=True);self.assertEqual(b.command(dict(sim_time_s=1.15,armed=True,fault='bumper')),(0.,0.));self.assertEqual(b.backoff,0.)
            ready(b,1.2,stamp_only=True);self.assertEqual(b.command(dict(sim_time_s=1.2,armed=True,fault='bumper')),(0.,0.));self.assertEqual(b.phase,'contact')
    def test_pressed_contact_right_after_a_turn_in_place_creeps_back_over_the_last_forward_drive(self):
        # 30 cm driven along +x, a quarter turn in place at the trail end, 8 mm forward, pressed on a low step: too little
        # trail for the 2 cm back-off, which used to mean waiting for ever. The creep (CONTACT_CREEP_V, look-ahead
        # CONTACT_CREEP_GUARD_M) undoes the 8 mm just driven and no more (forward_run: the wheels take it back).
        from autonomy.lidar_brain import CONTACT_CREEP_V
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);ring=open_floor(b);b.current_cloud=np.vstack((ring,wall([R+.001,-.3],[R+.001,.3])));ready(b,1.)
            for x in np.arange(-.30,1e-9,.01):b.trail.append(np.array([x,0.]))
            self.wheels(b,1.,0.,motion=(0.,0.));self.wheels(b,1.05,0.,spin=.18,motion=(0.,.4))
            b.localizer.pose=np.array([0.,0.,math.pi/2]);self.wheels(b,1.1,.004,spin=.18,motion=(.1,0.))
            b.localizer.pose=np.array([0.,.008,math.pi/2]);b.phase='follow'
            self.wheels(b,1.15,.008,spin=.18,pressed=True,motion=(.1,0.))
            self.assertEqual(b.phase,'contact');self.assertEqual(b.backoff_v,CONTACT_CREEP_V);self.assertGreater(b.backoff,.002);self.assertLessEqual(b.backoff,.008+1e-9)
            self.assertIn('Backing off',b.message)
            t=1.15;run=.008;moved=0.
            for k in range(60):
                ready(b,t,stamp_only=True);v,w=b.command(dict(sim_time_s=t,armed=True,fault='bumper'));b.requested_motion=(v,w)
                self.assertTrue(v==0 or v==-CONTACT_CREEP_V,(v,w));self.assertEqual(w,0.)
                b.localizer.pose[1]+=v*.05;moved-=v*.05;run+=v*.05;t+=.05;self.wheels(b,t,run,spin=.18,pressed=True)
            self.assertGreater(moved,.002);self.assertLessEqual(moved,.008+.001,'undoes at most the forward drive')
            self.assertEqual(b.backoff,0.,'still pressed (the test keeps the skin pressed): nothing more is legal');self.assertEqual(b.phase,'contact')
    def test_waiting_tries_the_back_off_again_once_the_wheels_have_stopped(self):
        # Pressed with too little forward drive to undo at the touch (0.3 mm), so it waits; the body then rolls on 6 mm (as
        # in the Unreal trial: 5.8 mm after the switch tripped). After CONTACT_SETTLE_S of waiting the back-off is planned
        # again, once: a creep over the ground just driven. A second roll-on later is not retried (once per contact).
        from autonomy.lidar_brain import CONTACT_CREEP_V,CONTACT_SETTLE_S
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);ring=open_floor(b);b.current_cloud=np.vstack((ring,wall([R+.001,-.3],[R+.001,.3])));ready(b,1.)
            for y in np.arange(-.30,1e-9,.01):b.trail.append(np.array([0.,y]))    # driven along +y, then a quarter turn to face +x
            self.wheels(b,1.,0.,motion=(0.,0.));self.wheels(b,1.05,0.,spin=.05,motion=(0.,-.4))
            self.wheels(b,1.1,.0003,spin=.05,pressed=True,motion=(.1,0.))
            self.assertEqual(b.phase,'contact');self.assertEqual(b.backoff,0.);self.assertLess(b.forward_run,.0005)
            t=1.1
            for k in range(4):ready(b,t,stamp_only=True);self.assertEqual(b.command(dict(sim_time_s=t,armed=False,fault='bumper')),(0.,0.));t+=.05
            b.localizer.pose=np.array([.0063,0.,0.]);self.wheels(b,t,.0063,spin=.05,pressed=True,motion=(0.,0.))
            self.assertAlmostEqual(b.forward_run,.0063,places=4)
            while t<1.1+CONTACT_SETTLE_S+.1:
                ready(b,t,stamp_only=True);v,w=b.command(dict(sim_time_s=t,armed=False,fault='bumper'));t+=.05
                if b.backoff:break
            self.assertEqual(b.backoff_v,CONTACT_CREEP_V);self.assertLessEqual(b.backoff,.0063+1e-9);self.assertIn('Still pressed. Backing off',b.message)
            ready(b,t,stamp_only=True);self.assertEqual(b.command(dict(sim_time_s=t,armed=True,fault='bumper')),(-CONTACT_CREEP_V,0.))
            self.assertTrue(b.wait_retried)
    def test_back_in_the_contact_phase_at_the_same_spot_keeps_the_attempts_and_the_wait(self):
        # Pressed, leaving the contact phase (e.g. to re-verify the position) and coming back still pressed at the same spot
        # does not restart CONTACT_ATTEMPTS or the CONTACT_WAIT_S wait; a press elsewhere (> CONTACT_SAME_M) or a new touch does.
        # A stall with no room to back off (bumper released) says so, not 'waiting for the bumper to release'.
        from autonomy.lidar_brain import CONTACT_SAME_M
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);ring=open_floor(b);b.current_cloud=np.vstack((ring,wall([R+.001,-.3],[R+.001,.3])));ready(b,1.)
            b.predict(dict(sim_time_s=1.,fault='bumper',armed=False,sensors=dict(self.PRESS,left_ticks=0,right_ticks=0)))
            from autonomy.lidar_brain import CONTACT_ATTEMPTS
            b.contact_attempts=CONTACT_ATTEMPTS;b.waiting_since=1.5;b.wait_retried=True;b.phase='recover_localization'
            for s_ in np.arange(.30,-1e-9,-.01):b.trail.append(np.array([-s_,0.]))    # trail room behind: only the used-up attempts stop a back-off
            b.start_contact_phase(20.,'front bumper',[2],recorded=False)
            self.assertEqual((b.contact_attempts,b.waiting_since,b.wait_retried),(CONTACT_ATTEMPTS,1.5,True));self.assertEqual(b.backoff,0.)
            b.trail.clear()
            b.localizer.pose=np.array([CONTACT_SAME_M+.01,0.,0.]);b.start_contact_phase(21.,'front bumper',[2],recorded=False)
            self.assertEqual((b.contact_attempts,b.waiting_since,b.wait_retried),(0,None,False))
            b.contact_attempts=3;b.start_contact_phase(22.,'front bumper',[2])
            self.assertEqual(b.contact_attempts,0,'a new touch')
            b.bumper_pressed=False;b.start_contact_phase(23.,'stall, nothing sensed',[],max_backoff=.05)
            self.assertEqual(b.backoff,0.);self.assertIn('No room to back off',b.message)
    def test_no_creep_without_a_forward_drive(self):
        # Pressed while turning in place (base_link drifted 6 mm during the spin, the trail holds that drift): no forward
        # drive to undo, so no reverse at all (the kitchen-corner slip stress case); the contact spin or the wait as before.
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);ring=open_floor(b);b.current_cloud=np.vstack((ring,wall([R+.001,-.3],[R+.001,.3])));ready(b,1.)
            b.trail.append(np.array([0.,-.006]));b.localizer.pose=np.array([0.,0.,1.2])
            self.wheels(b,1.,0.,motion=(0.,0.));self.wheels(b,1.05,0.,spin=.05,motion=(0.,.25))
            self.wheels(b,1.1,0.,spin=.05,pressed=True,motion=(0.,.25))
            self.assertEqual(b.phase,'contact');self.assertEqual(b.forward_run,0.);self.assertEqual(b.backoff,0.)
            for k in range(40):
                ready(b,1.1+.05*k,stamp_only=True);v,w=b.command(dict(sim_time_s=1.1+.05*k,armed=True,fault='bumper'));self.assertGreaterEqual(v,0.)
    def test_the_creep_guard_scales_with_speed(self):
        # command()'s look-ahead: REVERSE_GUARD_M at 5 cm/s, CONTACT_CREEP_GUARD_M at the creep's 2 cm/s or slower.
        # Toward the far end of a 4 cm trail the creep stops within its look-ahead less the end tolerance of it.
        from autonomy.lidar_brain import REVERSE_GUARD_M,CONTACT_CREEP_GUARD_M,CONTACT_CREEP_V,TRAIL_CAP_TOL
        self.assertEqual(LidarBrain.reverse_guard(-.05),REVERSE_GUARD_M);self.assertEqual(LidarBrain.reverse_guard(-.04),REVERSE_GUARD_M)
        self.assertEqual(LidarBrain.reverse_guard(-CONTACT_CREEP_V),CONTACT_CREEP_GUARD_M);self.assertLess(CONTACT_CREEP_GUARD_M,REVERSE_GUARD_M)
        for speed,guard in ((.05,REVERSE_GUARD_M),(CONTACT_CREEP_V,CONTACT_CREEP_GUARD_M)):
            with tempfile.TemporaryDirectory() as tmp:
                b=LidarBrain(tmp);open_floor(b);ready(b,1.);b.phase='follow';b.drive_command=lambda state,v=-speed:(v,0.)
                for x in np.arange(-.04,1e-9,.01):b.trail.append(np.array([x,0.]))
                t=1.
                for k in range(200):
                    ready(b,t,stamp_only=True);v,w=b.command(dict(sim_time_s=t,armed=True,fault='none'));b.requested_motion=(v,w);b.localizer.pose[0]+=v*.05;t+=.05
                self.assertGreaterEqual(b.localizer.pose[0]-guard,-.04-TRAIL_CAP_TOL-.003,speed);self.assertLess(b.localizer.pose[0]-guard,-.04-TRAIL_CAP_TOL+.003,speed)
    def test_pressed_with_nothing_legal_waits_at_most_contact_wait_then_stops_with_a_message(self):
        # Task G: no driven trail, no forward drive, contact in front: no legal motion. The robot waits for the release
        # (a person or a pet may move) at most CONTACT_WAIT_S, standing still, then ends the session with a message,
        # instead of waiting, pressed, until the time budget (the Unreal trial: 1110 s). Go-to ends the same way.
        from autonomy.lidar_brain import CONTACT_WAIT_S
        for mode in ('mapping','go_to'):
            with tempfile.TemporaryDirectory() as tmp:
                b=LidarBrain(tmp);ring=open_floor(b);b.current_cloud=np.vstack((ring,wall([R+.001,-.3],[R+.001,.3])));ready(b,1.);b.mode=mode;b.phase='follow'
                b.predict(dict(sim_time_s=1.,fault='bumper',armed=False,sensors=dict(self.PRESS,left_ticks=0,right_ticks=0)))
                self.assertEqual(b.phase,'contact');self.assertEqual(b.backoff,0.);self.assertIsNone(b.contact_spin)
                t=1.
                while t<1.+CONTACT_WAIT_S-.1:
                    ready(b,t,stamp_only=True);self.assertEqual(b.command(dict(sim_time_s=t,armed=False,fault='bumper')),(0.,0.));t+=.5
                self.assertEqual(b.mode,mode);self.assertIn(f'at most {CONTACT_WAIT_S:.0f} s',b.message)
                for k in range(4):
                    t+=.1;ready(b,t,stamp_only=True);self.assertEqual(b.command(dict(sim_time_s=t,armed=False,fault='bumper')),(0.,0.))
                self.assertEqual(b.mode,'idle');self.assertIn('Stuck against an obstacle',b.message);self.assertIn('no driven path behind',b.message)
                self.assertIn('partial map saved' if mode=='mapping' else 'Stopped.',b.message)
    def test_front_contact_with_a_driven_trail_backs_off_along_it(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);ring=open_floor(b);b.current_cloud=np.vstack((ring,wall([R+.001,-.3],[R+.001,.3])));ready(b,1.)
            for s in np.arange(.30,-1e-9,-.01):b.trail.append(np.array([-s,0.]))
            b.predict(dict(sim_time_s=1.,fault='bumper',armed=False,sensors=dict(bumper=True,bump_zones=1<<2,left_ticks=0,right_ticks=0,encoder_cpr=4741.44,gyro_rad_s=[0,0,0],specific_force_m_s2=[0,0,9.8])))
            self.assertEqual(b.phase,'contact');self.assertGreaterEqual(b.backoff,.02);self.assertIsNone(b.contact_spin)
            self.assertEqual(b.command(dict(sim_time_s=1.,armed=False,fault='bumper')),(-.05,0.))
    def press_on_trail(self,b,t=1.):
        """A front press at the origin, heading +x, after 30 cm driven along +x: the contact phase plans its back-off."""
        b.current_cloud=open_floor(b);ready(b,t)
        for s in np.arange(.30,-1e-9,-.01):b.trail.append(np.array([-s,0.]))
        b.predict(dict(sim_time_s=t,fault='bumper',armed=False,sensors=dict(self.PRESS,left_ticks=0,right_ticks=0)))
        self.assertEqual(b.phase,'contact');self.assertGreaterEqual(b.backoff,.02)
    def tick(self,b,t,armed=True):
        ready(b,t,stamp_only=True);v,w=b.command(dict(sim_time_s=t,armed=armed,fault='bumper' if b.bumper_pressed else 'none'));b.requested_motion=(v,w);return v,w
    def test_back_off_goes_on_when_the_body_rolls_on_after_its_first_reverse_tick(self):
        # Task P, Unreal trials full-mapping-20261012T183238 / T192331 (GI on, the shower tray): the press tripped at 8-10 cm/s,
        # the back-off was planned (8 / 4 cm) and its first reverse tick began the reverse episode, which fixed the newest end of
        # the driven trail there. The body then rolled on into the skin (DriveCore cuts the torque; the bumper stop is 7.9 mm past
        # the trip), so base_link stood past the end of its own trail: command()'s guard refused every reverse, the plan (which
        # never tested the first centimetre) still fitted, and nothing moved: 60 s pressed, then "Stuck" (trial 1), or 9 s until
        # LiDAR jitter freed it (trial 3). The episode's start follows base_link forward until it has moved back.
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);self.press_on_trail(b)
            self.assertEqual(self.tick(b,1.,armed=False),(-.05,0.))
            b.localizer.pose=np.array([.0075,0.,0.])                    # rolled on 7.5 mm into the skin
            t=1.05
            for k in range(4):self.assertEqual(self.tick(b,t),(-.05,0.),b.message);t+=.05
            self.assertTrue(np.allclose(b.trail_head(),[.0075,0.]),b.trail_head())
            for k in range(8):
                v,w=self.tick(b,t);self.assertEqual((v,w),(-.05,0.),b.message);b.localizer.pose[0]+=v*.05;t+=.05
            self.assertTrue(np.allclose(b.trail_head(),[.0075,0.]),'the start never follows base_link back')
            self.assertNotIn('Reverse refused',' '.join(e['message'] for e in b.events))
    def test_stall_retreat_after_driving_forward_past_where_a_short_reverse_began(self):
        # Task P, Unreal trial full-mapping-20261012T184036 (GI on, under the dining table): the local planner reversed 1.5 cm,
        # drove 2.7 cm forward on an arc (less than the 5 cm that closes a reverse episode) and stood 1 cm past where the reverse
        # began. Every stall retreat was planned, then refused by command()'s guard: the open episode had fixed the newest end of
        # the trail where the reverse began, and the samples after the reverse started a new run 1 cm or more past the old one.
        # 16 retreats, 126 s, until the narrow-gap rule gave up. Here the newest sample is where the reverse began and the robot
        # stops 8 mm past it, under the 1 cm that records a new sample: nothing after the reverse was ever recorded. Forward
        # driving after a reverse is driven ground from where it began; base_link is the newest end of the trail again.
        from autonomy.lidar_brain import TRAIL_TOL
        for arc in (0.,-.4):
            with tempfile.TemporaryDirectory() as tmp:
                b=LidarBrain(tmp);b.current_cloud=open_floor(b,radius=3.);ready(b,1.);b.phase='follow'
                for s in np.arange(.40,-1e-9,-.01):b.trail.append(np.array([-s,0.]))
                t=1.;plan=[(-.04,0.)]*8+[(.05,arc)]*10+[(0.,0.)]*10
                for want in plan:
                    b.drive_command=lambda state,want=want:want;v,w=self.tick(b,t);t+=.05
                    p=b.localizer.pose;p[:2]+=v*.05*np.array([math.cos(p[2]+w*.025),math.sin(p[2]+w*.025)]);p[2]+=w*.05;b.record_trail(p,t)
                del b.drive_command;p=b.localizer.pose.copy();u=np.array([math.cos(p[2]),math.sin(p[2])])
                self.assertIsNotNone(b.reverse_episode,'precondition: the episode is still open')
                self.assertGreater(p[0],.006,'precondition: past where the reverse began');self.assertLess(p[0],.01)
                line=p[:2]-np.arange(0,.06,.001)[:,None]*u
                self.assertTrue((b.trail_distance(line)<=TRAIL_TOL).all(),'base_link and the ground just behind it are on the trail')
                ready(b,t);b.phase='follow';self.assertTrue(b.begin_retreat(t));self.assertEqual(b.phase,'retreat')
                x0=p[:2].copy();n=len(b.events)
                for k in range(30):
                    v,w=self.tick(b,t)
                    if b.phase!='retreat':break
                    b.localizer.pose[:2]+=v*.05*np.array([math.cos(b.localizer.pose[2]),math.sin(b.localizer.pose[2])]);b.localizer.pose[2]+=w*.05;t+=.05
                self.assertNotIn('Reverse refused',' '.join(e['message'] for e in b.events[n:]),arc);self.assertGreaterEqual(np.linalg.norm(b.localizer.pose[:2]-x0),.03,arc)
    def test_planned_reverses_pass_the_final_guard_on_their_first_tick(self):
        # Task P: the back-off, the creep and the stall retreat plan a straight reverse with reverse_room, which looked 1 cm (the
        # creep: 1 mm) behind base_link and on in steps; command()'s guard (reverse_on_trail) checks base_link from where it is
        # now. Past the trail's end, or across a gap between trail runs, the plan fitted and the guard refused it every tick.
        # Any reverse they plan must pass the guard where it starts, and fit the episode budget the guard counts: command() adds the
        # move since the episode's last reverse tick (a forward drive too short to end the episode included) when the next reverse
        # starts; reverse_budget did not (2D house seed 0: a stall retreat planned with budget left, refused at once at the limit).
        from autonomy.lidar_brain import REVERSE_LIMIT,CONTACT_CREEP_V
        rng=np.random.default_rng(136);bad=[];planned=0
        with tempfile.TemporaryDirectory() as tmp:
            for k in range(400):
                b=LidarBrain(tmp);b.current_cloud=open_floor(b);ready(b,1.);bend=rng.uniform(-.25,.25)
                xy=[np.array([-s,0.]) for s in np.arange(.30,.10,-.01)];h=0.
                for i in range(10):h+=bend/10;xy.append(xy[-1]+.01*np.array([math.cos(h),math.sin(h)]))
                for q in xy:b.trail.append(q)
                if rng.random()<.4:                                      # the newest run starts after a gap (a reverse in between)
                    gap=rng.uniform(.008,.016);cut=len(xy)-rng.integers(2,6);d=xy[cut]-xy[cut-1];d/=np.linalg.norm(d)
                    for i in range(cut,len(xy)):b.trail[i]=b.trail[i]+(gap-.01)*d
                    b.trail_breaks.append(tuple(float(x) for x in b.trail[cut]))
                end=np.asarray(b.trail[-1]);b.localizer.pose=np.r_[end+rng.uniform(-.005,.015)*np.array([math.cos(h),math.sin(h)])+rng.uniform(-.01,.01)*np.array([-math.sin(h),math.cos(h)]),h+rng.uniform(-.2,.2)]
                ep=None
                if rng.random()<.5:                                      # an open episode whose start is the trail's end; it may have
                    # reversed up to 14 cm and its last reverse tick may lie up to 4 cm behind base_link (a drive forward since)
                    h_=b.localizer.pose[2];last=b.localizer.pose[:2]-rng.uniform(0,.04)*np.array([math.cos(h_),math.sin(h_)])
                    ep=b.reverse_episode=dict(start=end.copy(),last=last,travelled=rng.uniform(0,.14),forward_from=None);b.requested_motion=(-.05,0.);b.last_reverse_stamp=.99
                # What command() allows from here: the guard, and the episode budget after it adds the move since the last reverse tick.
                left=REVERSE_LIMIT if ep is None else REVERSE_LIMIT-ep['travelled']-float(np.linalg.norm(b.localizer.pose[:2]-ep['last']))
                b.bumper_pressed=True;b.forward_run=rng.uniform(0,.03)
                plan=b.backoff_plan(.12)
                if plan:
                    planned+=1
                    if not b.reverse_on_trail(-plan[1],0.) or plan[0]>left+1e-9:bad.append(('backoff',k,plan,round(left,4)))
                room=min(.035,b.reverse_budget(),b.reverse_room(REVERSE_LIMIT+.03)-.025)
                if room>=.03 and b.retreat_clear(room+.025):
                    planned+=1
                    if not b.reverse_on_trail(-.05,0.) or room>left+1e-9:bad.append(('retreat',k,room,round(left,4)))
        self.assertGreater(planned,100);self.assertEqual(bad,[],f'{len(bad)} of {planned} planned reverses refused at their first tick')
    def test_a_back_off_the_guard_refuses_is_planned_again_not_repeated(self):
        # Task P safety net: if the final guard refuses the back-off's reverse anyway, the contact phase plans again from where the
        # robot is (within CONTACT_ATTEMPTS, then the bounded wait) instead of commanding the refused reverse every tick until
        # its 10 s time-out; a refused stall retreat ends at once.
        from autonomy.lidar_brain import CONTACT_ATTEMPTS
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);self.press_on_trail(b)
            self.assertEqual(self.tick(b,1.,armed=False),(-.05,0.))
            b.reverse_on_trail=lambda *a,**k:False                     # the guard refuses from here on
            t=1.05
            for k in range(20):self.tick(b,t);t+=.05
            self.assertEqual(b.contact_attempts,CONTACT_ATTEMPTS);self.assertEqual(b.backoff,0.);self.assertEqual(b.phase,'contact')
            b=LidarBrain(tmp);b.current_cloud=open_floor(b);ready(b,1.)
            for s in np.arange(.30,-1e-9,-.01):b.trail.append(np.array([-s,0.]))
            b.phase='follow';self.assertTrue(b.begin_retreat(1.));self.assertEqual(b.phase,'retreat')
            self.assertEqual(self.tick(b,1.),(-.05,0.));b.reverse_on_trail=lambda *a,**k:False
            self.assertEqual(self.tick(b,1.05),(0.,0.));self.tick(b,1.1);self.assertNotEqual(b.phase,'retreat')
    def test_close_spin_aborts_only_on_actual_contact(self):
        from types import SimpleNamespace
        from autonomy.lidar_brain import CLOSE_SPIN_W,SPIN_RAW_ALLOWANCE
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);ring=open_floor(b);segs=corner_walls(.005,'diagonal')
            feed_scans(b,segs,8,seed=4)
            target=np.array([-1.,0.]);v,w=b.follow_velocity(target,1.,1.1)
            self.assertEqual(v,0.);self.assertAlmostEqual(abs(w),CLOSE_SPIN_W);self.assertEqual(b.maneuver['kind'],'close_spin')
            b.requested_motion=(v,w);direction=b.maneuver['direction']
            base=raycast(segs,.004,77)
            # Single-scan noise dips (one return 1.3 cm inside the skin, several 8 mm inside) do not stop it.
            for extra in ([[R-.013,0.]],[[R-.008,.01],[R-.008,-.01],[R-.008,.02]]):
                b.current_cloud=np.vstack((base,ring,extra));b.localizer.pose[2]+=.05*w
                self.assertEqual(b.follow_velocity(target,1.,1.15),(0.,w))
            # Two returns more than 3 sigma inside the skin: actual contact -> stop and re-measure.
            b.current_cloud=np.vstack((base,ring,[[R-SPIN_RAW_ALLOWANCE-.003,0.],[R-SPIN_RAW_ALLOWANCE-.003,.005]]))
            self.assertEqual(b.follow_velocity(target,1.,1.2),(0.,0.));self.assertIsNone(b.maneuver)
            # A press on the side the firmware refuses to turn toward also stops it.
            b=LidarBrain(tmp);ring=open_floor(b);feed_scans(b,segs,8,seed=4)
            v,w=b.follow_velocity(target,1.,1.1);direction=b.maneuver['direction']
            b.bumper_pressed=True;b.bumper_sides=(direction<0,direction>0)
            self.assertEqual(b.follow_velocity(target,1.,1.15),(0.,0.))
            for pure in (True,False):
                b=LidarBrain(tmp);b.pressed_spin_pure=pure;ring=open_floor(b);feed_scans(b,segs,8,seed=4)
                v,w=b.follow_velocity(target,1.,1.1);direction=b.maneuver['direction']
                b.bumper_pressed=True;b.bumper_sides=(direction>0,direction<0)
                # Pure-spin core: pressed on the side it turns toward is allowed. Clamping core (default):
                # any pressed spin would pivot backward, so it stops.
                self.assertEqual(b.follow_velocity(target,1.,1.15),(0.,w) if pure else (0.,0.),f'pure={pure}')
    def test_close_spin_stall_and_contact_are_not_retried_the_same_way(self):
        # A close spin that the gyro shows is not turning (a rigid-shell touch the rear cannot sense)
        # stops after CLOSE_SPIN_STALL_S; the next close spin from that spot uses the other direction
        # when it is allowed. A bumper contact during a close spin does the same.
        from autonomy.lidar_brain import CLOSE_SPIN_W,CLOSE_SPIN_STALL_S
        target=np.array([-1.,0.]);segs=corner_walls(.005,'diagonal')
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);ring=open_floor(b);feed_scans(b,segs,8,seed=4)
            v,w=b.follow_velocity(target,1.,1.1);self.assertEqual(b.maneuver['kind'],'close_spin');first=b.maneuver['direction']
            b.requested_motion=(v,w);b.gyro_z=w*.9
            self.assertEqual(b.follow_velocity(target,1.,1.15),(0.,w),'turning normally')
            b.gyro_z=0.;t=1.2
            while t<1.2+CLOSE_SPIN_STALL_S-.05:self.assertEqual(b.follow_velocity(target,1.,t),(0.,w),'the wheel ramp / a short dip is not a stall');t+=.05
            self.assertEqual(b.follow_velocity(target,1.,t+.1),(0.,0.));self.assertIsNone(b.maneuver);self.assertIn('stalled',b.message)
            m=b.close_spin_maneuver(target);self.assertIsNotNone(m);self.assertEqual(m['direction'],-first,'retry the other way')
            b.localizer.pose[:2]+=.04;self.assertFalse(b.close_spin_retry_blocked(first),'another spot: no memory')
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);ring=open_floor(b);feed_scans(b,segs,8,seed=4);b.mode='mapping'
            v,w=b.follow_velocity(target,1.,1.1);first=b.maneuver['direction']
            b.note_contact(1.15,'front bumper',[first<0,False,first>0])
            self.assertIsNone(b.maneuver);self.assertTrue(b.close_spin_retry_blocked(first))
    # Task J helpers: a close spin out of the 5 mm diagonal corner, and one predict() tick with the bumper zone bits `bits` (0: released).
    def close_spin_start(self,tmp,pure=True,trail=False):
        b=LidarBrain(tmp);b.pressed_spin_pure=pure;open_floor(b);feed_scans(b,corner_walls(.005,'diagonal'),8,seed=4);ready(b,1.1);b.phase='follow'
        if trail:
            for s_ in np.arange(.30,-1e-9,-.01):b.trail.append(np.array([-s_,0.]))
        v,w=b.follow_velocity(np.array([-1.,0.]),1.,1.1);self.assertEqual(b.maneuver['kind'],'close_spin');b.requested_motion=(v,w)
        return b,b.maneuver['direction'],w
    def bump_tick(self,b,t,bits):
        b.predict(dict(sim_time_s=t,fault='bumper' if bits else 'none',armed=not bits,sensors=dict(bumper=bool(bits),bump_zones=bits,left_ticks=0,right_ticks=0,
                       encoder_cpr=4741.44,gyro_rad_s=[0,0,0],specific_force_m_s2=[0,0,9.8])))
    def test_close_spin_goes_on_through_a_press_on_the_side_it_turns_toward(self):
        # Task J (LIO-115), the slipping 3 mm side corner: a close spin's drift pressed the skin on the side the spin turns toward.
        # Every press ended the spin and blocked that direction (note_contact), so the next close spin went the other way, started
        # at the skin's trip point and pressed again at once: four presses in 4 s ended the session. The pure-spin core executes
        # that same spin while pressed, and it brings the contact toward 0 deg (run_maneuver's abort rule already let it go on), so
        # with no back-off room the turn goes on: the contact is recorded and counts for the area give-up, and when the press
        # reaches the front centre (both switches) the contact phase takes over, recording nothing new.
        from autonomy.lidar_brain import CLOSE_SPIN_W
        target=np.array([-1.,0.])
        with tempfile.TemporaryDirectory() as tmp:
            b,d,w=self.close_spin_start(tmp);toward=1<<(4 if d>0 else 0)
            self.bump_tick(b,1.15,toward)
            self.assertEqual(b.phase,'follow');self.assertEqual(b.maneuver['kind'],'close_spin');self.assertEqual(len(b.contacts),1)
            self.assertTrue(b.grid.camera_obstacles.any(),'the contact is on the map');self.assertFalse(b.close_spin_retry_blocked(d));self.assertIn('continuing the turn',b.message)
            self.bump_tick(b,1.2,toward);self.assertEqual(b.phase,'follow','still pressed, no new edge: the turn goes on')
            self.assertEqual(b.follow_velocity(target,1.,1.2),(0.,w));self.assertAlmostEqual(abs(w),CLOSE_SPIN_W);self.assertTrue(b.firmware_executes(0.,w))
            self.bump_tick(b,1.25,1<<2)
            self.assertEqual(b.phase,'contact');self.assertIsNone(b.maneuver);self.assertEqual(len(b.contacts),1);self.assertIn('Still pressed',b.message)
        # As before: a press on the side the firmware refuses, a centre press, the clamping core, or a driven trail behind (back off along it).
        for case in ('refused','centre','clamping','trail'):
            with tempfile.TemporaryDirectory() as tmp:
                b,d,w=self.close_spin_start(tmp,pure=case!='clamping',trail=case=='trail')
                bits={'refused':1<<(0 if d>0 else 4),'centre':1<<2}.get(case,1<<(4 if d>0 else 0))
                self.bump_tick(b,1.15,bits)
                self.assertEqual(b.phase,'contact',case);self.assertIsNone(b.maneuver,case);self.assertEqual(len(b.contacts),1,case);self.assertTrue(b.close_spin_retry_blocked(d),case)
                if case=='trail':self.assertGreaterEqual(b.backoff,.02)
        # Only the live close spin goes on: one left behind in self.maneuver after the robot left 'follow' (plan, a margin escape) or
        # replanned (a new follow context) does not, so the press starts the contact phase.
        for stale in ('plan','escape_margin','new_context'):
            with tempfile.TemporaryDirectory() as tmp:
                b,d,w=self.close_spin_start(tmp)
                if stale=='new_context':b.phase_started+=1.
                else:b.phase=stale
                self.bump_tick(b,1.15,1<<(4 if d>0 else 0))
                self.assertEqual(b.phase,'contact',stale);self.assertIsNone(b.maneuver,stale);self.assertEqual(len(b.contacts),1,stale)
    def test_presses_during_a_close_spin_still_count_for_the_repeated_contact_give_up(self):
        # The area give-up (four contacts within 0.5 m end the session) also counts the presses a close spin turns through.
        with tempfile.TemporaryDirectory() as tmp:
            b,d,w=self.close_spin_start(tmp);toward=1<<(4 if d>0 else 0)
            for k in range(4):
                self.assertEqual(b.mode,'mapping');self.bump_tick(b,1.15+.1*k,toward);self.bump_tick(b,1.2+.1*k,0)
                if k==0:self.assertEqual(b.phase,'follow');self.assertIn('Bumper released. Finishing the slow turn',b.message)
            self.assertEqual(len(b.contacts),4);self.assertEqual(b.mode,'idle');self.assertIn('Repeated contacts in one area',b.message)
    def test_close_spin_contact_check_uses_the_rigid_shell_behind(self):
        # Overlap evidence: two raw returns > SPIN_RAW_ALLOWANCE inside the skin in front, inside the
        # rigid shell (R_RIGID) behind. Noise-level dips are not contact.
        from autonomy.lidar_brain import SPIN_RAW_ALLOWANCE
        from autonomy.geometry import R_RIGID
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);ring=open_floor(b)
            at=lambda r,deg:[r*math.cos(math.radians(deg)),r*math.sin(math.radians(deg))]
            for pts,hit in (([at(R-SPIN_RAW_ALLOWANCE-.002,10),at(R-SPIN_RAW_ALLOWANCE-.002,12)],True),
                            ([at(R-SPIN_RAW_ALLOWANCE-.002,10)],False),
                            ([at(R_RIGID-SPIN_RAW_ALLOWANCE-.002,170),at(R_RIGID-SPIN_RAW_ALLOWANCE-.002,172)],True),
                            ([at(R-SPIN_RAW_ALLOWANCE-.0005,170),at(R-SPIN_RAW_ALLOWANCE-.0005,172)],R-SPIN_RAW_ALLOWANCE-.0005<R_RIGID-SPIN_RAW_ALLOWANCE)):
                b.current_cloud=np.vstack((ring,pts));self.assertEqual(b.close_spin_contact(),hit,pts)
    def test_margin_escape_reverse_is_bounded_by_the_reverse_limit(self):
        # Inside a new obstacle margin with a wall 18 cm ahead and a driven trail behind: the measured
        # escape is a reverse along the trail, at most REVERSE_LIMIT long (it used to run .20-.22 m).
        REVERSE_LIMIT=.15
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);ring=open_floor(b);b.current_cloud=np.vstack((ring,wall([.18,-.3],[.18,.3])));ready(b,1.)
            for y in np.arange(-.3,.31,.05):b.grid.logodds[b.grid.cell(np.array([.20,y]))]=5
            for s_ in np.arange(.40,-1e-9,-.01):b.trail.append(np.array([-s_,0.]))
            self.assertFalse(b.grid.masks()[3][b.grid.cell(np.zeros(2))])
            self.assertEqual(b.clearance_escape(),-1.)
            b.phase='escape_margin';b.escape_sign=-1.;b.phase_started=.9;b.escape_origin=np.array([REVERSE_LIMIT-.01,0.,0.])
            self.assertEqual(b.command(dict(sim_time_s=1.,armed=True,fault='none')),(-.04,0.),'still within the limit')
            b.escape_origin=np.array([REVERSE_LIMIT+.002,0.,0.])
            self.assertEqual(b.command(dict(sim_time_s=1.05,armed=True,fault='none')),(0.,0.));self.assertNotEqual(b.phase,'escape_margin')
    def test_spin_room_escape_never_reverses_off_the_trail(self):
        # Skin 5 mm from both walls, no driven trail: the arc fallback may only drive forward.
        from autonomy.geometry import TRACK
        for layout in ('diagonal','side'):
            with tempfile.TemporaryDirectory() as tmp:
                b=LidarBrain(tmp);open_floor(b);feed_scans(b,corner_walls(.005,layout),8)
                m=b.spin_room_escape();self.assertTrue(m is None or m['v']>0,layout)
                for v in (-.04,):
                    for w in (0.,.15,-.15):self.assertIsNone(b.arc_check(v,w,.01),'a 1 cm reverse arc with no trail')
                # Along a straight trail behind: reverse arcs on it are allowed again.
                for s in np.arange(.15,-1e-9,-.01):b.trail.append(np.array([-s,0.]))
                self.assertIsNotNone(b.arc_check(-.04,0.,.05))
                self.assertLessEqual(.15*2*.04/TRACK,.3)
    def test_recovery_advance_is_slow_only_with_a_measured_point_nearby(self):
        # Replaces the infrared light-bump slow-down: .06 m/s when a LiDAR/camera point lies
        # within NEARBY ahead, else .10 m/s.
        from types import SimpleNamespace
        from autonomy.local_planner import NEARBY
        for extra,speed in ((np.empty((0,2)),.10),(np.array([[.15,.21]]),.06)):
            with tempfile.TemporaryDirectory() as tmp:
                b=LidarBrain(tmp);ring=open_floor(b);b.mode='mapping';b.attitude_initialized=True
                b.current_cloud=np.vstack((ring,extra));b.frame=SimpleNamespace(stamp=1.);b.camera.last_stamp=1.;b.localizer.last_visual_time=1.;b.localizer.uncertainty=.025
                b.phase='recovery_advance';b.escape_origin=np.zeros(3);b.phase_started=1.;b.motor_seen_armed=True
                if len(extra):self.assertLess(np.linalg.norm(extra[0]),NEARBY)
                self.assertEqual(b.command(dict(sim_time_s=1.,armed=True,fault='none')),(speed,0.))
    def test_repeated_skips_from_one_spot_keep_far_targets_and_block_completion(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);open_floor(b);b.mode='mapping'
            b.goal=np.array([2.,0.]);b.skip_goal('No translational progress toward this target.',1.)
            self.assertEqual(len(b.blacklist),1);self.assertFalse(b.stuck())
            b.goal=np.array([-3.,1.]);b.skip_goal('No translational progress toward this target.',13.)
            self.assertEqual(len(b.blacklist),1,'a target in another room is not blamed for the robot being stuck')
            self.assertTrue(b.stuck());self.assertFalse(b.mapping_complete());self.assertEqual(b.phase,'recover_localization')
            b.localizer.pose=np.array([.3,0.,0.]);self.assertFalse(b.stuck())
    def test_narrow_passage_stall_blocks_that_pinch_for_narrow_planning_only(self):
        # house2d rr2_nocam: the narrow-passage planner (5 cm cell centres, radius .20) kept routing
        # through a pinch whose measured points were .384 m apart (< 2 x DRIVE_CLEARANCE); the local
        # planner correctly refused and the robot lost ~55 s retrying it.
        from autonomy.lidar_brain import NARROW_RADIUS
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);g=room(size=100);b.grid=g;b.mode='mapping';b.goal=g.world((70,50));b.path=[g.world((50,50)),b.goal]
            b.localizer.pose=np.r_[g.world((50,50)),0.];g.radius=NARROW_RADIUS;b.normal_radius=PLAN_RADIUS
            b.translation_anchor=b.localizer.pose[:2].copy();b.translation_since=0.;b.progress_pose=b.localizer.pose.copy();b.last_progress=13.;b.next_plan=1e9;b.phase='follow'
            b.goal_kind='frontier';b.motion_until=0
            cell=g.cell(b.localizer.pose[:2]+[.10,0]);self.assertTrue(g.masks()[3][cell])
            from types import SimpleNamespace
            b.attitude_initialized=True;b.frame=SimpleNamespace(stamp=13.);b.camera.last_stamp=13.;b.localizer.last_visual_time=13.;b.localizer.uncertainty=.025;b.motor_seen_armed=True
            b.current_cloud=np.c_[np.cos(np.linspace(-3,3,300)),np.sin(np.linspace(-3,3,300))]
            b.command(dict(sim_time_s=13.,armed=True,fault='none'))
            self.assertEqual(len(g.narrow_blocks),1)
            g.radius=NARROW_RADIUS;self.assertFalse(g.masks()[3][cell],'narrow planning avoids the pinch')
            g.radius=PLAN_RADIUS;self.assertTrue(g.masks()[3][cell],'normal planning is unaffected')
            self.assertIn('measured width',b.events[-2]['message'] if len(b.events)>1 else b.message)
            # One stall does not close the pinch for the rest of the run.
            from autonomy.lidar_brain import NARROW_BLOCK_S
            b.prune_narrow_blocks(13.+NARROW_BLOCK_S-1);self.assertEqual(len(g.narrow_blocks),1)
            b.prune_narrow_blocks(13.+NARROW_BLOCK_S+1);self.assertEqual(len(g.narrow_blocks),0)
            g.radius=NARROW_RADIUS;self.assertTrue(g.masks()[3][cell],'expired: narrow planning may try it again')
    def test_recorded_doorway_stall_drives_through_the_passage(self):
        # house2d rr_nocam replay: base_link .2166 m from the nearest wall of a .44 m passage. The
        # quantized field (true distance - up to 2.8 cm) refused every translation, the robot spun
        # back and forth for 70 s and then skipped targets in other rooms. Sweeps the field cannot
        # decide are now checked against the points: it drives, never closer than DRIVE_CLEARANCE.
        import json
        from autonomy.local_planner import choose_velocity
        folder=Path(__file__).resolve().parent/'fixtures/round-doorway-stall';state=json.loads((folder/'state.json').read_text())
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);b.grid.load(folder);b.grid.radius=state['radius'];b.localizer.pose=np.array(state['pose'])
            b.current_cloud=np.array(state['cloud']);b.path=[np.array(p) for p in state['path']];b.goal=np.array(state['goal'])
            for t in state['trail']:b.trail.append(np.array(t))
            cloud=b.obstacle_points();self.assertGreater(np.linalg.norm(cloud,axis=1).min(),DRIVE_CLEARANCE)
            v,w=choose_velocity(b,b.path_target(),float(np.linalg.norm(b.goal-b.localizer.pose[:2])))
            self.assertGreater(v,0.,'translational progress through the passage')
            rel=b.predicted_centres(v,w,np.linspace(0,1,41))
            self.assertGreaterEqual(np.linalg.norm(cloud[None]-rel[:,None],axis=2).min(),DRIVE_CLEARANCE)
    def test_bumper_zones_and_contact_marks_are_about_base_link(self):
        from autonomy.lidar_brain import shell_point,contact_azimuth
        self.assertEqual(BUMP_ZONES,[(-90.,-55.),(-55.,-20.),(-20.,20.),(20.,55.),(55.,90.)])
        for az in (-90,-55,-20,0,20,55,90):
            point,direction=shell_point(az)
            self.assertAlmostEqual(float(np.linalg.norm(point)),R);np.testing.assert_allclose(direction,[math.cos(math.radians(az)),math.sin(math.radians(az))])
        with tempfile.TemporaryDirectory() as tmp:
            for bit,(lo,hi) in enumerate(BUMP_ZONES):
                b=LidarBrain(tmp);b.mode='mapping';marks=[]
                b.grid.add_camera_obstacles=lambda world,margin=1,contact=False:marks.append(np.asarray(world))
                b.note_contact(1.,'test',1<<bit);m=np.vstack(marks)
                az=np.degrees(np.arctan2(m[:,1],m[:,0]));r=np.linalg.norm(m,axis=1)
                self.assertTrue(((az>=lo-1e-6)&(az<=hi+1e-6)).all(),f'zone {bit} marks outside {lo}..{hi} deg about base_link')
                np.testing.assert_allclose(np.unique(np.round(r,6)),[round(R+.05,6),round(R+.08,6)])
            self.assertEqual(contact_azimuth([4]),72.5);self.assertEqual(contact_azimuth([]),0.)
    def test_pressed_contact_without_reverse_room_spins_contact_toward_front(self):
        from types import SimpleNamespace
        from autonomy.lidar_brain import CONTACT_SPIN
        for bit,sign in ((4,1.),(3,1.),(0,-1.),(1,-1.)):
            with tempfile.TemporaryDirectory() as tmp:
                # Pure-spin motor core (the clamping default never spins while pressed: see the test above).
                b=LidarBrain(tmp);b.pressed_spin_pure=True;ring=open_floor(b);b.mode='mapping';b.attitude_initialized=True
                b.current_cloud=ring;b.frame=SimpleNamespace(stamp=1.);b.camera.last_stamp=1.;b.localizer.last_visual_time=1.;b.localizer.uncertainty=.025
                b.predict(dict(sim_time_s=1.,fault='bumper',armed=False,sensors=dict(bumper=True,bump_zones=1<<bit,left_ticks=0,right_ticks=0,encoder_cpr=4741.44,gyro_rad_s=[0,0,0],specific_force_m_s2=[0,0,9.8])))
                self.assertEqual(b.phase,'contact');self.assertLess(b.backoff,.02,'no driven trail: no reverse room')
                v,w=b.command(dict(sim_time_s=1.,armed=False,fault='bumper'))
                # Turning by +a moves a contact at azimuth a toward 0 deg (never back past +/-90 onto the rigid rear shell).
                self.assertEqual(v,0.);self.assertEqual(math.copysign(1,w),sign);self.assertLessEqual(abs(w),CONTACT_SPIN)
                # Once the contact is in front the spin stops.
                b.localizer.pose[2]=b.contact_spin;v,w=b.command(dict(sim_time_s=1.1,armed=False,fault='bumper'))
                self.assertIsNone(b.contact_spin);self.assertEqual(w,0.)
    @staticmethod
    def drivecore_pressed(v,w,track):
        """Python port of DriveCore.h while the bumper is pressed (re-armed for reverse_only, then
        update() clamps both wheel setpoints to <= 0): (arm allowed, executed v, executed w)."""
        arm=v<=0 and abs(w)<=.3
        ls,rs=min(v-w*track/2,0.),min(v+w*track/2,0.)
        return arm,(ls+rs)/2,(rs-ls)/track
    def test_pressed_commands_are_executed_unchanged_by_the_motor_core(self):
        # Property: whatever drive_command asks while the bumper is pressed, command() sends only what
        # DriveCore.h executes AS COMMANDED (no clamp-made pivot), and no reverse leaves the (empty) trail.
        # With the default (clamping core) that means no spin at all while pressed.
        from autonomy.geometry import TRACK
        from autonomy.lidar_brain import CONTACT_SPIN
        rng=np.random.default_rng(3)
        for pure in (False,True):
            for k in range(400):
                with tempfile.TemporaryDirectory() as tmp:
                    b=LidarBrain(tmp);b.pressed_spin_pure=pure;open_floor(b);ready(b,1.);b.phase='follow'
                    b.bumper_pressed=True;b.bumper_sides=[(True,False),(False,True),(True,True)][k%3]
                    want=(float(rng.choice([0.,0.,-.05,-.02,.05])),float(rng.choice([0.,.1,-.1,.18,-.18,.25,-.25,.4,-.4,.3,-.3])))
                    if k%2:   # half the cases with a straight driven trail behind
                        for s_ in np.arange(.30,-1e-9,-.01):b.trail.append(np.array([-s_,0.]))
                    b.drive_command=lambda state,want=want:want
                    v,w=b.command(dict(sim_time_s=1.,armed=True,fault='bumper'))
                    if (v,w)==(0.,0.):continue
                    arm,ev,ew=self.drivecore_pressed(v,w,TRACK)
                    if pure and v==0:
                        left,right=b.bumper_sides
                        self.assertTrue(abs(w)<=CONTACT_SPIN+1e-9 and ((left and right) or (left and w>0) or (right and w<0)),(want,(v,w)))
                    else:
                        self.assertTrue(arm,(want,(v,w)));self.assertAlmostEqual(ev,v);self.assertAlmostEqual(ew,w)
                    if not pure:self.assertTrue(v<0,f'pressed, clamping core: sent {(v,w)} for {want}')
                    if v<0:self.assertTrue(k%2,'a reverse with no driven trail')
    def test_pressed_robot_outside_the_contact_phase_returns_to_it(self):
        # The contact phase is entered on a rising edge only; a robot still pressed in follow / plan (it
        # started pressed, or an edge was missed) goes back to it without recording a new contact, and a
        # back-off that cannot continue while still pressed waits in the contact phase (never 'plan').
        sensors=dict(bumper=True,bump_zones=1<<2,left_ticks=0,right_ticks=0,encoder_cpr=4741.44,gyro_rad_s=[0,0,0],specific_force_m_s2=[0,0,9.8])
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);ring=open_floor(b);b.current_cloud=ring;ready(b,1.);b.phase='follow';b.bumper_pressed=True;b.bumper_sides=(True,True)
            b.predict(dict(sim_time_s=1.,fault='bumper',armed=False,sensors=sensors))
            self.assertEqual(b.phase,'contact');self.assertEqual(len(b.contacts),0);self.assertIn('Still pressed',b.message)
            for t in np.arange(1.,30.,.5):
                ready(b,t,stamp_only=True);self.assertEqual(b.command(dict(sim_time_s=t,armed=False,fault='bumper')),(0.,0.))
            self.assertEqual(b.phase,'contact');self.assertEqual(b.mode,'mapping')
            # With a trail: back off along it; when the 10 s back-off limit passes still pressed, wait.
            b=LidarBrain(tmp);ring=open_floor(b);b.current_cloud=np.vstack((ring,wall([R+.001,-.3],[R+.001,.3])));ready(b,1.)
            for s_ in np.arange(.30,-1e-9,-.01):b.trail.append(np.array([-s_,0.]))
            b.predict(dict(sim_time_s=1.,fault='bumper',armed=False,sensors=sensors));self.assertEqual(b.phase,'contact')
            self.assertEqual(b.command(dict(sim_time_s=1.,armed=True,fault='bumper')),(-.05,0.))
            ready(b,11.5,stamp_only=True);self.assertEqual(b.command(dict(sim_time_s=11.5,armed=True,fault='bumper')),(0.,0.))
            self.assertEqual(b.phase,'contact');self.assertEqual(b.backoff,0.)
    def test_pressed_spin_that_pivots_backward_switches_to_the_clamp_rule(self):
        # Firmware mismatch defence: told the core spins purely while pressed, but the wheel odometry
        # shows backward travel during a pressed spin (DriveCore.h clamps it into a pivot): stop spinning.
        from autonomy.geometry import WHEEL_R
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);b.pressed_spin_pure=True;open_floor(b);b.mode='mapping'
            per_m=4741.44/(2*math.pi*WHEEL_R);b.requested_motion=(0.,.25)
            base=dict(bumper=True,bump_zones=1<<4,encoder_cpr=4741.44,gyro_rad_s=[0,0,0],specific_force_m_s2=[0,0,9.8])
            b.bumper_pressed=True;b.predict(dict(sim_time_s=1.,fault='bumper',armed=True,sensors=dict(base,left_ticks=0,right_ticks=0)))
            b.requested_motion=(0.,.25);b.predict(dict(sim_time_s=1.05,fault='bumper',armed=True,sensors=dict(base,left_ticks=-.0004*per_m,right_ticks=0)))
            self.assertTrue(b.pressed_spin_pure,'0.2 mm: below the threshold')
            b.requested_motion=(0.,.25);b.predict(dict(sim_time_s=1.1,fault='bumper',armed=True,sensors=dict(base,left_ticks=-.0012*per_m,right_ticks=0)))
            self.assertFalse(b.pressed_spin_pure);self.assertIn('pivot',b.message)
            # A true pure spin (wheels opposite, no net travel) does not trigger it.
            b=LidarBrain(tmp);b.pressed_spin_pure=True;open_floor(b);b.mode='mapping';b.bumper_pressed=True;b.requested_motion=(0.,.25)
            for k in range(10):b.requested_motion=(0.,.25);b.predict(dict(sim_time_s=1.+.05*k,fault='bumper',armed=True,sensors=dict(base,left_ticks=-.001*k*per_m,right_ticks=.001*k*per_m)))
            self.assertTrue(b.pressed_spin_pure)
    def test_controller_does_not_read_infrared_light_bump(self):
        import re
        sources={p:p.read_text() for p in CONTROLLER.rglob('*.py')}
        for p,text in sources.items():
            for key in ('near_obstacle',):self.assertNotIn(key,text,f'{p.name} still uses {key}')
            hit=re.search(r'light.?bump|ir_intensity|infrared|proximity|[\'"]ir[\'"]',text,re.I)
            self.assertIsNone(hit,f'{p.name} still mentions an IR / proximity sensor: {hit.group(0) if hit else ""}')
        # Behaviour: the same brain state with and without IR readings gives the same command.
        from types import SimpleNamespace
        def run(extra):
            with tempfile.TemporaryDirectory() as tmp:
                b=LidarBrain(tmp);ring=open_floor(b);b.mode='mapping';b.phase='follow';b.attitude_initialized=True
                b.current_cloud=np.vstack((ring,wall([.30,-.3],[.30,.3])));b.frame=SimpleNamespace(stamp=1.,index=1);b.camera.last_stamp=1.
                b.localizer.last_visual_time=1.;b.localizer.uncertainty=.025;b.motor_seen_armed=True;b.started=0.
                b.goal=np.array([1.,.5]);b.path=[np.zeros(2),np.array([.5,.5]),b.goal];b.next_plan=1e9;b.last_progress=1.;b.progress_pose=np.zeros(3)
                sensors=dict(left_ticks=0,right_ticks=0,encoder_cpr=4741.44,gyro_rad_s=[0,0,0],specific_force_m_s2=[0,0,9.8],cliff_m=[.0425]*4,**extra)
                b.predict(dict(sim_time_s=1.,armed=True,fault='none',sensors=sensors))
                return b.command(dict(sim_time_s=1.,armed=True,fault='none',sensors=sensors)),b.follow_velocity(np.array([.5,.5]),1.,2.),b.status()
        (c0,f0,s0),(c1,f1,s1)=run({}),run(dict(light_bump=127,ir_intensity=[4000]*7))
        self.assertEqual(c0,c1);self.assertEqual(f0,f1)
        self.assertFalse(any(k in json_keys(s1) for k in ('light_bump','ir_intensity','near_obstacle')))
        # A light-bump reading no longer changes anything the planner sees.
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);b.predict(dict(sim_time_s=0.,armed=False,fault='none',sensors=dict(light_bump=127,ir_intensity=[4000]*7,left_ticks=0,right_ticks=0,encoder_cpr=4741.44,gyro_rad_s=[0,0,0],specific_force_m_s2=[0,0,9.8])))
            self.assertFalse(hasattr(b,'near_obstacle'))
    def test_all_clearances_come_from_geometry_and_are_consistent(self):
        import json,re
        from autonomy import local_planner,lidar_brain,lidar,grid
        params=json.loads(geometry.PARAMETERS_PATH.read_text())
        self.assertEqual(geometry.PARAMETERS_PATH,CONTROLLER.parent/'Content/RobotData/robot-parameters.json')
        self.assertEqual((R,geometry.R_RIGID,geometry.AXLE_X),(params['body_radius_m'],params['rigid_radius_m'],params['drive_axle_x_m']))
        self.assertEqual(geometry.AXLE_X,0.);self.assertLessEqual(geometry.R_RIGID,R)
        # TURN_MARGIN is the specified 1 cm (5 mm spin drift + 5 mm averaged range error).
        self.assertEqual(TURN_MARGIN,.01);self.assertAlmostEqual(TURN_CLEAR,R+TURN_MARGIN)
        self.assertAlmostEqual(TURN_MARGIN,geometry.SPIN_DRIFT+geometry.SPIN_RANGE_ERROR)
        # Derived, not fixed numbers: a CAD sync that changes R moves them consistently.
        self.assertEqual(DRIVE_CLEARANCE,geometry._ceil5mm(R+.04));self.assertEqual(BODY_CLEAR,geometry._ceil5mm(R+.008))
        self.assertEqual(geometry.derive(R,geometry.R_RIGID),dict(DRIVE_CLEARANCE=DRIVE_CLEARANCE,BODY_CLEAR=BODY_CLEAR,TURN_CLEAR=TURN_CLEAR))
        self.assertLessEqual(R+.0854,PLAN_RADIUS);self.assertEqual(PLAN_RADIUS,.25)
        self.assertTrue(R<TURN_CLEAR<DRIVE_CLEARANCE<=PLAN_RADIUS);self.assertTrue(R<BODY_CLEAR<DRIVE_CLEARANCE)
        self.assertEqual(lidar.LidarGrid().radius,PLAN_RADIUS)
        # The legacy stereo Grid keeps its original .26 inflation (stereo depth error), as before the port.
        self.assertAlmostEqual(grid.Grid().radius,PLAN_RADIUS+.01)
        self.assertEqual(lidar_brain.NARROW_RADIUS,DRIVE_CLEARANCE);self.assertEqual(lidar_brain.WHEEL_TRACK,params['wheel_track_m'])
        self.assertIs(local_planner.TURN_CLEAR,TURN_CLEAR);self.assertIs(lidar_brain.TURN_CLEAR,TURN_CLEAR)
        np.testing.assert_allclose(lidar_brain.CLIFF_XY,params['cliff_sensors_m'])
        # No stray robot-geometry literals (old axle .042, D shell .132, old turn/body/drive
        # thresholds .195/.20/.21/.225/.17, old envelope .162, old planning radius .26) in
        # controller code outside geometry.py, except the listed non-geometry .20 gates.
        stray=re.compile(r'(?<![\d.])0?\.(042|132|195|20|21|225|17|162|16205|26)(?!\d)')
        allowed={('lidar.py','result[:2]-pred[:2])>.20'),('lidar.py','last_key_pose[:2])>.20'),       # scan-match / keyframe gates
                 ('localization.py','pose[:2])<.20'),('localization.py','relocalization_candidate[2]))<.20'),('brain.py','offsets=[0.,-.20'),('brain.py','v*v/(2*.20)'),  # keyframe gate, yaw offsets, deceleration
                 ('grid.py','(dist>.20)'),('lidar_brain.py','math.hypot(roll,pitch)>.20'),('lidar_brain.py','reshape(-1,2),.20)')}  # stereo blind zone, tilt rad, visit gap
        for p in (CONTROLLER/'autonomy').glob('*.py'):
            if p.name=='geometry.py':continue
            for n,line in enumerate(p.read_text().splitlines(),1):
                code=line.split('#')[0]
                if stray.search(code) and any(p.name==f and key in code for f,key in allowed):continue
                self.assertIsNone(stray.search(code),f'{p.name}:{n}: {line.strip()[:80]}')
    def test_old_box_parameter_file_still_imports_and_spin_rule_is_conservative(self):
        # The old box robot's parameters (drive axle .042 ahead, no body_radius_m) or no file at
        # all must not stop the controller importing; the circle rule about the offset axle then
        # contains the whole old box swept about that axle, with the 1 cm margin.
        import importlib.util,json,os
        old=Path(__file__).resolve().parent/'fixtures/old-box-robot-parameters.json'
        for path in (old,Path('/nonexistent/robot-parameters.json')):
            spec=importlib.util.spec_from_file_location('geometry_old',CONTROLLER/'autonomy/geometry.py');g=importlib.util.module_from_spec(spec)
            source=(CONTROLLER/'autonomy/geometry.py').read_text().replace("PARAMETERS_PATH = Path(__file__).resolve().parents[2] / 'Content/RobotData/robot-parameters.json'",f"PARAMETERS_PATH = Path({str(path)!r})")
            exec(compile(source,'geometry_old.py','exec'),g.__dict__)
            self.assertEqual(g.AXLE_X,.042);self.assertAlmostEqual(g.R,.16205)
            # Old box (24 x 20 cm, x -.12...134 about base_link, half-width .132 ahead of the axle,
            # .12 behind it) swept about the axle: farthest corner .2016 m from the axle.
            box=np.array([[-.12,.12],[-.12,-.12],[.134,.132],[.134,-.132],[.042,.132],[.042,.12]])
            swept=float(np.hypot(box[:,0]-g.AXLE_X,box[:,1]).max())
            self.assertGreaterEqual(g.TURN_CLEAR+abs(g.AXLE_X),swept+.01)
            self.assertTrue(g.R<g.TURN_CLEAR<g.DRIVE_CLEARANCE<=g.PLAN_RADIUS)
        self.assertEqual(json.loads(old.read_text())['drive_axle_x_m'],.042)
# ---- Go-to targets close to walls: the final approach (lidar_brain APPROACH_*) ----
# Room around the origin on the LidarGrid's 5 cm cells: measured free cells 141..179, measured walls in rows /
# columns 140 and 180 (cell centres -.975 and 1.025 m), unknown outside. The LiDAR sees the walls at those
# cell centres (true surfaces), so a 2 cm-gap target is 4 cells (.20) from the wall cells.
from autonomy.geometry import DRIVE_CLEARANCE_LIDAR
from autonomy.lidar import transform
ROOM_LO,ROOM_HI=-.975,1.025
ROOM_WALLS=[((ROOM_LO,ROOM_LO),(ROOM_HI,ROOM_LO)),((ROOM_HI,ROOM_LO),(ROOM_HI,ROOM_HI)),((ROOM_HI,ROOM_HI),(ROOM_LO,ROOM_HI)),((ROOM_LO,ROOM_HI),(ROOM_LO,ROOM_LO))]
def wall_room(b):
    g=b.grid;g.logodds[141:180,141:180]=-5;g.seen[140:181,140:181]=5
    g.logodds[140,140:181]=5;g.logodds[180,140:181]=5;g.logodds[140:181,140]=5;g.logodds[140:181,180]=5
    return g
def go_to_ready(b,t,pose,goal,near=True):
    """A Go-to in progress with a verified pose (as after relocalization)."""
    ready(b,t);b.mode='go_to';b.reset_recovery(t);b.reset_approach(near);b.goal=np.asarray(goal,float);b.localizer.pose=np.asarray(pose,float).copy()
def drive_loop(b,t,seconds,segments,stop=None,ramp=.25,record=None):
    """Closed loop with ideal localization: command() at 20 Hz, unicycle about base_link with the DriveCore.h
    wheel ramp on the forward speed, one noise-free LiDAR scan (world `segments`) per tick at the true pose."""
    va=float(b.requested_motion[0]);rows=[]
    for k in range(int(seconds*20)):
        p=b.localizer.pose.copy();cloud=raycast(seen_from(segments,p));b.current_cloud=cloud;b.scan_pose=p.copy();ready(b,t,True)
        b.note_spin_scan(cloud,p,t);b.record_trail(p,t)
        v,w=b.command(dict(sim_time_s=t,armed=True,fault='none'));b.requested_motion=(v,w);rows.append((t,b.phase,v,w,p.copy()))
        for _ in range(5):
            va+=float(np.clip(v-va,-ramp*.01,ramp*.01));wa=w;q=b.localizer.pose;q[0]+=va*.01*math.cos(q[2]+wa*.005);q[1]+=va*.01*math.sin(q[2]+wa*.005);q[2]=wrap_(q[2]+wa*.01)
        t+=.05
        if record:record(b,t)
        if stop and stop(b):break
    return rows,t
def wall_gap(pose):
    """Distance from base_link to the nearest room wall (the true surfaces)."""
    x,y=pose[:2];return min(x-ROOM_LO,ROOM_HI-x,y-ROOM_LO,ROOM_HI-y)
class GoToNearWallTests(unittest.TestCase):
    def test_drive_clearance_lidar_is_derived_in_geometry(self):
        from autonomy.geometry import DRIVE_CLEARANCE_LIDAR,GOAL_CLEAR,DRIVE_LIDAR_MARGIN,TRACK_MARGIN,LIDAR_RANGE_ERROR
        self.assertEqual((TRACK_MARGIN,LIDAR_RANGE_ERROR),(.015,.01));self.assertAlmostEqual(DRIVE_LIDAR_MARGIN,.025)
        self.assertEqual(DRIVE_CLEARANCE_LIDAR,geometry._ceil5mm(R+.015+.01));self.assertAlmostEqual(DRIVE_CLEARANCE_LIDAR,.185)
        self.assertTrue(TURN_CLEAR<DRIVE_CLEARANCE_LIDAR<DRIVE_CLEARANCE);self.assertAlmostEqual(GOAL_CLEAR,R-.02);self.assertTrue(R-.05<GOAL_CLEAR<PLAN_RADIUS)
        from autonomy import lidar_brain;self.assertIs(lidar_brain.DRIVE_CLEARANCE_LIDAR,DRIVE_CLEARANCE_LIDAR)
    def test_near_wall_target_accepted_too_close_and_unmapped_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);g=wall_room(b);x=g.world((160,160))[0]
            row=lambda r:np.array([x,g.world((r,160))[1]])
            self.assertFalse(LidarBrain.near_goal_check(g,row(145)))           # 5 cells = .25: safe mask, planned as before
            self.assertTrue(LidarBrain.near_goal_check(g,row(144)))            # 4 cells = .20: final approach
            self.assertTrue(LidarBrain.near_goal_check(g,row(143)))            # 3 cells = .15 >= R - 2 cm (cell quantization)
            self.assertTrue(LidarBrain.near_goal_check(g,row(143)+[0.,-.024])) # anywhere in that cell (a body there overlaps the wall: the approach decides)
            with self.assertRaisesRegex(ValueError,"closer to an obstacle than the robot's body allows"):LidarBrain.near_goal_check(g,row(142))   # 2 cells = .10
            with self.assertRaisesRegex(ValueError,'inside mapped free space'):LidarBrain.near_goal_check(g,row(139))   # unknown, outside
            with self.assertRaisesRegex(ValueError,'inside mapped free space'):LidarBrain.near_goal_check(g,row(140))   # the wall cell
            g.add_camera_obstacles(np.array([g.world((150,150))]),margin=0)
            with self.assertRaisesRegex(ValueError,'inside mapped free space'):LidarBrain.near_goal_check(g,g.world((150,150)))   # camera / contact cell
            g.logodds[160:162,170:172]=0.                                     # an unmapped patch 2 cells (.10) away
            with self.assertRaisesRegex(ValueError,'inside mapped free space'):LidarBrain.near_goal_check(g,g.world((160,168)))
            self.assertTrue(LidarBrain.near_goal_check(g,g.world((160,167))))                                      # 3 cells (.15): accepted
    def test_dashboard_go_to_near_a_wall_is_accepted_and_plans_to_a_staging_cell(self):
        from autonomy.lidar import LidarLocalizer
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);folder=root/'Saved/Maps/test';folder.mkdir(parents=True)
            loc=LidarLocalizer();loc.points=np.vstack([wall(a,b,.02) for a,b in ROOM_WALLS]);loc.keyframes=[{'pose':np.zeros(3)},{'pose':np.array([.5,0,0])}];loc.save(folder)
            b=LidarBrain(root);wall_room(b);b.grid.save(folder);(folder.parent/'latest-lidar-map.json').write_text('{"directory":"test"}')
            x=b.grid.world((160,160))[0];near=np.array([x,b.grid.world((144,160))[1]])
            b=LidarBrain(root)
            with self.assertRaisesRegex(ValueError,'closer to an obstacle'):b.begin_go_to([x,b.grid.world((142,160))[1]],1.)
            b=LidarBrain(root);b.begin_go_to(near.tolist(),1.);self.assertEqual(b.phase,'relocalizing');self.assertTrue(b.near_goal)
            np.testing.assert_allclose(b.goal,near)                          # the dashboard keeps showing the clicked target
            b.localizer.pose=np.array([x,.4,0.]);b.mode='go_to';b.plan(2.)
            self.assertEqual(b.phase,'follow');self.assertTrue(b.grid.masks()[3][b.grid.cell(b.staging)])
            self.assertLess(np.linalg.norm(b.staging-near),.06)              # the safe cell next to it, straight line to the target
            np.testing.assert_allclose(b.path[-1],b.staging);self.assertIs(b.route_goal(),b.staging)
            self.assertIn('closest safe point beside the target',b.message)
    def test_staging_needs_a_line_of_sight_to_the_target(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);g=wall_room(b)
            g.logodds[141:180,181:220]=-5;g.logodds[140,180:221]=5;g.logodds[180,180:221]=5;g.logodds[140:181,220]=5   # a closed room east of it
            goal=g.world((160,176));self.assertTrue(LidarBrain.near_goal_check(g,goal))                                 # 4 cells from the shared wall
            go_to_ready(b,1.,np.r_[g.world((160,200)),0.],goal);b.phase='plan';b.plan(1.)
            self.assertEqual(b.mode,'idle');self.assertIn('No safe path to that position',b.message)   # safe cells beside it, but behind the wall
            go_to_ready(b,1.,np.r_[g.world((160,150)),0.],goal);b.phase='plan';b.plan(1.)
            self.assertEqual(b.phase,'follow');self.assertLess(b.staging[0],goal[0])                    # from inside its room: the cell beside it
    def test_approach_stops_at_the_live_lidar_limit_forward_only(self):
        from autonomy.lidar_brain import APPROACH_V
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);g=wall_room(b);x=g.world((160,160))[0]
            # Start beside the target facing along the wall (the turn comes first); the target lies 15 cm
            # from the wall, inside the .185 live-LiDAR limit, so the approach must stop on that limit.
            go_to_ready(b,1.,[x,-.6,0.],[x,ROOM_LO+.15]);b.begin_approach(1.)
            rows,t=drive_loop(b,1.,40.,ROOM_WALLS,stop=lambda b:b.phase!='approach')
            self.assertEqual(b.phase,'confirm_arrival');self.assertEqual(b.approach_result['reason'],'lidar')
            v=np.array([r[2] for r in rows]);self.assertTrue((v>=0).all(),'the approach never reverses')
            self.assertLessEqual(v.max(),APPROACH_V+1e-9)
            turn=[r for r in rows if r[2]==0 and r[3]!=0];self.assertTrue(turn and turn[0][0]<rows[[r[2]>0 for r in rows].index(True)][0])
            gaps=[wall_gap(r[4]) for r in rows];settled=wall_gap(b.localizer.pose)
            for _ in range(10):drive_loop(b,t,.05,ROOM_WALLS);t+=.05             # standing still: the wheels have stopped
            settled=wall_gap(b.localizer.pose)
            self.assertGreaterEqual(min(min(gaps),settled),DRIVE_CLEARANCE_LIDAR-.0005)   # never inside the live-LiDAR limit
            self.assertLess(settled,DRIVE_CLEARANCE_LIDAR+.006)                       # and it used the room up to it
            self.assertRegex(b.message,r'^Stopped \d+ cm short of the target: the closest safe point \(about \d+ cm from the wall\)')
    def test_approach_keeps_drive_clearance_from_a_camera_cell(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);g=wall_room(b);cell=g.world((154,160));g.add_camera_obstacles(np.array([cell]),margin=0)
            go_to_ready(b,1.,[cell[0],.2,-math.pi/2],[cell[0],cell[1]+.105]);b.begin_approach(1.)
            rows,t=drive_loop(b,1.,40.,ROOM_WALLS,stop=lambda b:b.phase!='approach')
            for _ in range(10):drive_loop(b,t,.05,ROOM_WALLS);t+=.05
            self.assertEqual(b.approach_result['reason'],'camera');self.assertTrue(all(r[2]>=0 for r in rows))
            d=[float(np.linalg.norm(r[4][:2]-cell)) for r in rows]+[float(np.linalg.norm(b.localizer.pose[:2]-cell))]
            self.assertGreaterEqual(min(d),DRIVE_CLEARANCE-.0005);self.assertLess(d[-1],DRIVE_CLEARANCE+.006)
            self.assertIn('from an obstacle the camera or bumper found',b.message)
    def test_approach_sweep_point_inside_the_limit_blocks_only_motion_toward_it(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);ring=open_floor(b);b.scan_pose=np.zeros(3)
            b.current_cloud=np.vstack((ring,[[0.,DRIVE_CLEARANCE_LIDAR-.005]]))      # beside, already inside the limit
            self.assertEqual(b.approach_sweep(0.,.2),(.2,None))                  # driving on moves away from it
            b.current_cloud=np.vstack((ring,[[-.03,DRIVE_CLEARANCE_LIDAR-.01]]))     # behind-left
            self.assertEqual(b.approach_sweep(0.,.2),(.2,None))
            b.current_cloud=np.vstack((ring,[[.03,DRIVE_CLEARANCE_LIDAR-.01]]))      # ahead-left: any step comes closer
            self.assertEqual(b.approach_sweep(0.,.2),(0.,'lidar'))
            b.current_cloud=np.vstack((ring,[[DRIVE_CLEARANCE_LIDAR+.05,0.]]))       # straight ahead: room up to the limit
            room,why=b.approach_sweep(0.,.2);self.assertEqual(why,'lidar');self.assertAlmostEqual(room,.05,delta=.0011)
            # Points measured at the last scan's pose are carried to the current pose by odometry.
            b.localizer.pose=np.array([.02,0.,0.]);room,_=b.approach_sweep(0.,.2);self.assertAlmostEqual(room,.03,delta=.0011)
    def test_approach_arrives_on_a_target_short_of_the_limit_and_confirms_there(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);g=wall_room(b);x=g.world((160,160))[0];goal=[x,ROOM_LO+.21]
            go_to_ready(b,1.,[x,-.55,-math.pi/2],goal);b.begin_approach(1.)
            rows,t=drive_loop(b,1.,40.,ROOM_WALLS,stop=lambda b:b.phase!='approach')
            self.assertEqual(b.approach_result['reason'],'goal');self.assertLess(np.linalg.norm(b.localizer.pose[:2]-goal),.01)
            self.assertEqual(b.message,'At the goal. Confirming position before Idle.')
            np.testing.assert_allclose(b.arrival_point(),b.approach_stop)
            self.assertEqual(b.arrival_message(b.localizer.pose),'Arrived. Position checked across five LiDAR scans; now Idle.')
    def test_approach_steers_along_the_checked_arc_onto_the_target(self):
        # Driving with the heading .08 rad off the target (below the .25 rad re-turn): the commanded w = curvature
        # x v makes the driven path the arc approach_sweep checked, and that arc ends on the target.
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);g=wall_room(b);x=g.world((160,160))[0];goal=np.array([x,ROOM_LO+.21]);start=np.array([x+.01,-.6])
            yaw=math.atan2(*(goal-start)[::-1])+.08;go_to_ready(b,1.,np.r_[start,yaw],goal);b.begin_approach(1.);b.approach['stage']='drive'
            first={};sweep=b.approach_sweep
            def spy(k,length):
                # The arc the first drive tick checked, in the map frame.
                if 'arc' not in first:first['arc']=transform(b.predicted_centres(1.,k,np.linspace(0,.15,31)),b.localizer.pose)
                return sweep(k,length)
            b.approach_sweep=spy
            rows,t=drive_loop(b,1.,40.,ROOM_WALLS,stop=lambda b:b.phase!='approach')
            self.assertEqual(b.approach_result['reason'],'goal');self.assertLess(np.linalg.norm(b.localizer.pose[:2]-goal),.006)
            self.assertFalse(any(r[2]==0 and r[3]!=0 for r in rows),'no turn in place')
            path=np.array([r[4][:2] for r in rows if r[2]>0]);arc=first['arc']
            near=np.linalg.norm(path[:,None]-arc[None],axis=2).min(axis=1)[np.linalg.norm(path-path[0],axis=1)<.15]
            self.assertLess(near.max(),.003,'the driven path follows the checked arc')
    def test_every_go_to_ends_with_the_final_approach(self):
        # Safe-mask targets (planned exactly as before) also finish with the approach once the follow phase
        # stops within .10: open floor and a target ~9 cm (skin) from a wall both end within .03 of the target.
        from autonomy.lidar_brain import APPROACH_ARRIVED
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);g=wall_room(b)
            for goal,wording in ((g.world((166,166)),'Approaching the target slowly.'),(g.world((145,170)),'Approaching the target close to the wall, slowly.')):
                self.assertFalse(LidarBrain.near_goal_check(g,goal))
                go_to_ready(b,1.,[.4,.4,math.pi],goal,near=False);b.phase='plan';b.plan_not_before=1.;phases=[]
                rows,t=drive_loop(b,1.,90.,ROOM_WALLS,stop=lambda b:b.phase not in ('follow','plan','approach'),record=lambda b,t:phases.append(b.phase))
                self.assertIn('approach',phases);self.assertEqual(b.phase,'confirm_arrival',b.message)
                self.assertLess(np.linalg.norm(b.localizer.pose[:2]-goal),APPROACH_ARRIVED);self.assertEqual(b.message,'At the goal. Confirming position before Idle.')
                self.assertTrue(any(e['message']==wording for e in b.events));self.assertTrue(all(r[2]>=0 for r in rows if r[1]=='approach'))
                self.assertLessEqual(max(r[2] for r in rows if r[1]=='approach'),.05+1e-9)
                self.assertEqual(b.arrival_message(b.localizer.pose),'Arrived. Position checked across five LiDAR scans; now Idle.')
            self.assertAlmostEqual(wall_gap(b.localizer.pose)-R,wall_gap(np.r_[goal,0.])-R,delta=APPROACH_ARRIVED)
    def test_go_to_already_within_the_tolerance_confirms_without_an_approach(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);g=wall_room(b);goal=g.world((166,166))
            go_to_ready(b,1.,np.r_[goal+[.02,0.],0.],goal,near=False);b.phase='follow';b.path=[goal+[.02,0.],goal];b.next_plan=100.
            rows,t=drive_loop(b,1.,.05,ROOM_WALLS)
            self.assertEqual(b.phase,'confirm_arrival');self.assertIsNone(b.approach_stop);self.assertEqual(rows[0][2:4],(0.,0.))
    def test_contact_during_the_approach_ends_the_go_to_after_the_back_off(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);g=wall_room(b);x=g.world((160,160))[0]
            go_to_ready(b,1.,[x,-.55,-math.pi/2],[x,ROOM_LO+.17]);b.begin_approach(1.)
            for s in np.arange(.2,-1e-9,-.01):b.trail.append(np.array([x,-.55+s]))
            b.predict(dict(sim_time_s=1.,fault='bumper',armed=False,sensors=dict(bumper=True,bump_zones=1<<2,left_ticks=0,right_ticks=0,encoder_cpr=4741.44,gyro_rad_s=[0,0,0],specific_force_m_s2=[0,0,9.8])))
            self.assertEqual(b.phase,'contact');self.assertTrue(b.approach_contact);self.assertGreater(b.grid.camera_obstacles.sum(),0)
            b.bumper_pressed=False;b.phase='plan';b.plan(2.)                  # the contact phase ended (released, backed off)
            self.assertEqual(b.phase,'confirm_arrival');self.assertIn('the bumper touched an obstacle',b.message)
    def test_next_go_to_leaves_the_near_wall_spot_without_reversing(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);g=wall_room(b);x=g.world((160,160))[0]
            go_to_ready(b,1.,[x,-.6,-math.pi/2],[x,ROOM_LO+.15]);b.begin_approach(1.)
            rows,t=drive_loop(b,1.,40.,ROOM_WALLS,stop=lambda b:b.phase!='approach')
            start=b.localizer.pose.copy();self.assertFalse(g.masks()[3][g.cell(start[:2])],'the stop point is inside the planning margin')
            # A new Go-to from there (begin_go_to resets the trail and the approach state): the next target is a safe cell.
            goal=g.world((166,166));go_to_ready(b,t,start,goal,near=False);b.phase='plan';b.plan_not_before=t
            rows,t=drive_loop(b,t,60.,ROOM_WALLS,stop=lambda b:b.phase not in ('follow','plan','approach'))
            self.assertEqual(b.phase,'confirm_arrival',b.message);self.assertTrue(all(r[2]>=0 for r in rows),'no reverse while leaving')
            self.assertGreaterEqual(min(wall_gap(r[4]) for r in rows),wall_gap(start)-.003,'never closer to the wall than the stop point')
            self.assertLess(np.linalg.norm(b.localizer.pose[:2]-goal),.03)            # the final approach: within .03 of the target
def stall_ready(b,t=1.):
    """A forward drive that just jammed at the origin (heading 0): open floor, a .30 m driven trail behind, bumper released."""
    ring=open_floor(b);b.current_cloud=ring;ready(b,t)
    for s_ in np.arange(.30,-1e-9,-.01):b.trail.append(np.array([-s_,0.]))
    b.phase='follow';b.requested_motion=(.1,0.);b.jam_forward=True;b.bumper_pressed=False
SENSORS=dict(left_ticks=0,right_ticks=0,encoder_cpr=4741.44,gyro_rad_s=[0,0,0],specific_force_m_s2=[0,0,9.8])
class StallMarkTests(unittest.TestCase):
    """lidar_brain.mark_stall / begin_jam_settle: a forward jam with nothing sensed (overhang above the LiDAR plane, caster
    caught) is marked like an unknown front contact, unless a refusal branch applies."""
    def ahead_cells(self,b):
        from autonomy.lidar_brain import contact_marks
        return [b.grid.cell(q) for q in contact_marks([1,2,3])]
    def test_unexplained_forward_stall_is_marked_ahead_and_backs_off_like_a_contact(self):
        from autonomy.lidar_brain import STALL_BACKOFF
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);stall_ready(b)
            self.assertTrue(b.mark_stall(np.zeros(3),1.))
            self.assertEqual(len(b.stall_marks),1);self.assertTrue(all(b.grid.camera_obstacles[c] for c in self.ahead_cells(b)))
            self.assertEqual(b.phase,'contact');self.assertEqual(len(b.contacts),0,'not a bumper contact (no repeated-contacts stop)')
            self.assertGreaterEqual(b.backoff,.02);self.assertLessEqual(b.backoff,STALL_BACKOFF+1e-9)
            self.assertEqual(b.command(dict(sim_time_s=1.,armed=True)),(-.05,0.),'the back-off reverses along the driven trail')
    def test_mark_stall_refusals_mark_nothing(self):
        from autonomy.lidar_brain import STALL_MARK_SPACING
        cases={'bumper pressed':lambda b:setattr(b,'bumper_pressed',True),
               'jam while turning':lambda b:setattr(b,'jam_forward',False),
               'LiDAR return at the body':lambda b:setattr(b,'current_cloud',np.vstack((b.current_cloud,wall([R+.005,-.1],[R+.005,.1])))),
               'stall mark within the spacing':lambda b:b.stall_marks.append(np.array([STALL_MARK_SPACING-.01,0.]))}
        for name,setup in cases.items():
            with self.subTest(name),tempfile.TemporaryDirectory() as tmp:
                b=LidarBrain(tmp);stall_ready(b);setup(b);marks=len(b.stall_marks);cells=int(b.grid.camera_obstacles.sum())
                self.assertFalse(b.mark_stall(np.zeros(3),1.))
                self.assertEqual(len(b.stall_marks),marks);self.assertEqual(int(b.grid.camera_obstacles.sum()),cells);self.assertEqual(b.phase,'follow')
        # Just outside the spacing a new mark is made.
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);stall_ready(b);b.stall_marks.append(np.array([STALL_MARK_SPACING+.01,0.]));self.assertTrue(b.mark_stall(np.zeros(3),1.))
    def test_refused_mark_falls_back_to_skipping_the_target(self):
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);stall_ready(b);b.goal=np.array([1.,0.]);b.jams=[np.array([-.05,0.]),np.array([-.02,0.])];b.bumper_pressed=True
            self.assertFalse(b.note_jam(np.zeros(3),1.))
            self.assertEqual(b.stall_marks,[]);self.assertIn('Wheels repeatedly slip at this spot.',b.message);self.assertEqual(b.phase,'plan')
    def test_jam_next_to_earlier_jams_is_marked_on_its_first_stall(self):
        # note_jam counts every jam within .3 m this session: next to two earlier jams, a new spot is marked at once.
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);stall_ready(b);b.jams=[np.array([-.25,0.]),np.array([-.2,.1])]
            self.assertFalse(b.note_jam(np.zeros(3),1.));self.assertEqual(len(b.stall_marks),1);self.assertEqual(b.phase,'contact')
    def test_stall_mark_limit_stops_the_session(self):
        from autonomy.lidar_brain import STALL_MARK_LIMIT
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);stall_ready(b);b.stall_marks=[np.array([1.+.3*k,1.]) for k in range(STALL_MARK_LIMIT-2)]
            self.assertTrue(b.mark_stall(np.zeros(3),1.));self.assertEqual(b.mode,'mapping');self.assertEqual(b.phase,'contact')
            b.phase='follow';b.localizer.pose=np.array([0.,-.5,0.]);b.jam_forward=True
            self.assertTrue(b.mark_stall(b.localizer.pose,2.));self.assertEqual(len(b.stall_marks),STALL_MARK_LIMIT)
            self.assertEqual(b.mode,'idle');self.assertTrue(b.message.startswith(f'Stopped: blocked at {STALL_MARK_LIMIT} spots'),b.message)
            self.assertEqual(b.command(dict(sim_time_s=2.1,armed=True)),(0.,0.))
    def test_jam_settles_before_it_is_handled(self):
        from autonomy.lidar_brain import JAM_SETTLE_S
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);stall_ready(b);b.begin_jam_settle(1.,'Wheels turn but the robot does not move.')
            self.assertEqual(b.phase,'jam_settle');self.assertTrue(b.jam_forward)
            for t in np.arange(1.,1.+JAM_SETTLE_S-.01,.1):
                ready(b,t,stamp_only=True);self.assertEqual(b.command(dict(sim_time_s=t,armed=True)),(0.,0.));self.assertEqual(b.phase,'jam_settle')
            ready(b,1.+JAM_SETTLE_S,stamp_only=True);b.command(dict(sim_time_s=1.+JAM_SETTLE_S,armed=True))
            self.assertNotEqual(b.phase,'jam_settle');self.assertEqual(len(b.jams),1,'handled once, from the settled pose')
    def test_bumper_press_during_jam_settle_switches_to_contact(self):
        from autonomy.lidar_brain import JAM_SETTLE_S
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);stall_ready(b);b.begin_jam_settle(1.,'Wheels turn but the robot does not move.')
            b.predict(dict(sim_time_s=1.2,fault='bumper',armed=False,sensors=dict(SENSORS,bumper=True,bump_zones=1<<2)))
            self.assertEqual(b.phase,'contact');self.assertEqual(len(b.contacts),1)
            ready(b,1.2+JAM_SETTLE_S,stamp_only=True);b.command(dict(sim_time_s=1.2+JAM_SETTLE_S,armed=False,fault='bumper'))
            self.assertEqual(b.phase,'contact','the settled jam handling must not override the contact');self.assertEqual(b.jams,[]);self.assertEqual(b.stall_marks,[])
if __name__=='__main__':unittest.main()
