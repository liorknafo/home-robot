"""Camera cells in the map (task M, LIO-121): an overhang in the LiDAR's blind band, confirmed by the camera, stays an obstacle;
something the LiDAR also saw is handed back to it when it moves away.

With task L's LiDAR 20 mm lower (plane 127.5 mm, robot top 135.15 mm) the Unreal sofa base (underside 130 mm) lies in the band
between the scan plane and the robot top: every beam passes under it, from close by and from far away, and only the LiDAR
turret meets it (5/5 Unreal drives at it touched). The camera now confirms its bottom edge (floor_parallax). Those cells must
not be undone by beams that legitimately pass under them, by task K's close-range hold (which ages LiDAR evidence after
HOLD_S), or by task H's completion rules (floor walled in by them is not floor the robot still has to visit). A person's feet
or a chair's base are camera cells beside legs the LiDAR sees: they are released once the LiDAR has cleared those legs by its
own rule (lidar.CAMERA_BACK_*)."""
import unittest,sys,math,tempfile,json
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'controller'))
from autonomy.lidar import LidarGrid,inverse,HOLD_S,CAMERA_BACK_RADIUS_M,OVERHANG_SHADOW_M
from autonomy.lidar_brain import LidarBrain,PUSH_STALL_S,PUSH_V
from autonomy.geometry import LIDAR_MOUNT
from test_mapping_completion import house,area,POCKET,HI

WALL_X=3.0
def scan(pose,posts=()):
    """Base-frame points of 361 beams over +-60 deg ahead of the LiDAR at `pose`, ending on the wall at x = WALL_X unless a
    post (x, y, half-width: a vertical strip facing the robot, e.g. a leg) is in the way. Everything else (an overhang's
    face, a foot) the beams pass over or under."""
    c,s=math.cos(pose[2]),math.sin(pose[2]);o=pose[:2]+np.array([c*LIDAR_MOUNT[0]-s*LIDAR_MOUNT[1],s*LIDAR_MOUNT[0]+c*LIDAR_MOUNT[1]])
    a=pose[2]+np.radians(np.linspace(-60,60,361));d=np.c_[np.cos(a),np.sin(a)];t=(WALL_X-o[0])/d[:,0]
    for px,py,half in posts:
        tp=(px-o[0])/d[:,0];yp=o[1]+tp*d[:,1];t=np.where((tp>0)&(tp<t)&(np.abs(yp-py)<=half),tp,t)
    return inverse(o+t[:,None]*d,pose)
def run(g,x,stamps,posts=(),y=0.,phase=0):
    """One scan per stamp from (x, y); the heading alternates 0 / 0.07 rad (from `phase`) so every scan passes the motion gate."""
    for k,t in enumerate(stamps):
        pose=np.array([x,y,.07*((k+phase)%2)]);g.integrate_scan(scan(pose,posts),pose,t)
FACE_X=1.025
def at(x):return x-LIDAR_MOUNT[0]          # robot x that puts the LiDAR at x
def cells(g,pts):return [g.cell(p) for p in pts]
def camera(g,cs):return [bool(g.camera_obstacles[c]) for c in cs]
def blocked(g,cs):return [bool(g.logodds[c]>.8) for c in cs]

class CameraOverhangMapTests(unittest.TestCase):
    def overhang(self):
        """A grid with the camera's cells along a 30 cm overhang face at x = FACE_X (nothing the LiDAR sees there)."""
        g=LidarGrid();face=[(FACE_X,y) for y in np.arange(-.15,.151,.05)];g.add_camera_obstacles(np.array(face),margin=0);return g,cells(g,face)

    def test_beams_passing_under_an_overhang_never_clear_it(self):
        # From 30 cm and from 2 m, 40 scans each, the last ones more than HOLD_S after the camera saw it: task K's hold ages
        # LiDAR evidence, not camera cells.
        g,cs=self.overhang();run(g,at(FACE_X-.30),np.arange(0,20));run(g,at(FACE_X-2.),np.arange(20,40))
        run(g,at(FACE_X-.30),np.arange(20,40)+2*HOLD_S);run(g,at(FACE_X-2.),np.arange(40,60)+2*HOLD_S)
        self.assertTrue(all(camera(g,cs)) and all(blocked(g,cs)),[float(g.logodds[c]) for c in cs])
        # The LiDAR's own evidence there says free: the beams really did pass under it.
        self.assertTrue(all(g.lidar_logodds[c]<-.8 for c in cs))

    def test_overhang_cells_stay_without_lidar_hits_so_nothing_hands_them_over(self):
        g,cs=self.overhang();run(g,at(FACE_X-.50),np.arange(0,60,.5))
        self.assertFalse(g.camera_backed[tuple(np.array(cs).T)].any());self.assertTrue(all(camera(g,cs)))

    def test_sporadic_hits_when_the_robot_tilts_do_not_hand_an_overhang_to_the_lidar(self):
        # A beam 2.5 mm under the sofa base returns from it only when the robot tilts nose-up: one scan in four here.
        g,cs=self.overhang()
        for k in range(40):run(g,at(FACE_X-.30),[k],posts=[(FACE_X,0.,.15)] if k%4==0 else [],phase=k)
        run(g,at(FACE_X-.30),np.arange(40,80))
        self.assertTrue(all(camera(g,cs)),[float(g.lidar_logodds[c]) for c in cs])

    def test_a_person_seen_by_both_is_released_when_the_lidar_clears_their_legs(self):
        # Legs 10 cm wide at x = FACE_X (the LiDAR returns them), the camera's cells at the legs, the feet 5-10 cm in front
        # and a toe 15 cm in front. The person walks away; the robot is still 40 cm from where they stood.
        g=LidarGrid();legs=[(FACE_X,0.,.05)];feet=[(FACE_X,0.),(FACE_X-.05,0.),(FACE_X-.10,.05),(FACE_X-.15,0.)]
        run(g,at(FACE_X-.40),np.arange(0,5),posts=legs);g.add_camera_obstacles(np.array(feet),margin=0);cs=cells(g,feet)
        run(g,at(FACE_X-.40),[5,6],posts=legs);self.assertTrue(all(camera(g,cs)))
        released=None
        for k in range(30):
            run(g,at(FACE_X-.40),[7+k],phase=k+1)
            if released is None and not any(camera(g,cs)):released=k+1
        self.assertIsNotNone(released,'the camera cells stay although the person left');self.assertLessEqual(released,10)
        self.assertFalse(any(blocked(g,cs)),'released, the cells take the LiDAR\'s own evidence: the beams passed over them')

    def test_a_person_seen_from_far_away_is_released_after_task_ks_hold(self):
        # Seen from 30 cm, then only beams from 2 m pass the spot: the LiDAR keeps the legs for HOLD_S (task K), and the
        # camera cells with them; afterwards both go.
        g=LidarGrid();legs=[(FACE_X,0.,.05)];feet=[(FACE_X-.05,0.)]
        run(g,at(FACE_X-.30),np.arange(0,5),posts=legs);g.add_camera_obstacles(np.array(feet),margin=0);cs=cells(g,feet)
        run(g,at(FACE_X-.30),[5],posts=legs);run(g,at(FACE_X-2.),np.arange(10,30))
        self.assertTrue(all(camera(g,cs)),'within HOLD_S the far beams clear neither')
        run(g,at(FACE_X-2.),np.arange(10,30)+HOLD_S);self.assertFalse(any(camera(g,cs)))

    def test_an_overhang_beside_legs_the_lidar_keeps_seeing_stays(self):
        # The sofa: legs at both ends of the face (the LiDAR sees them every scan), the face between them in the blind band.
        g=LidarGrid();legs=[(FACE_X+.05,-.20,.03),(FACE_X+.05,.20,.03)];face=[(FACE_X,y) for y in np.arange(-.20,.201,.05)]
        run(g,at(FACE_X-.30),np.arange(0,5),posts=legs);g.add_camera_obstacles(np.array(face),margin=0);cs=cells(g,face)
        run(g,at(FACE_X-.30),np.arange(5,40),posts=legs);run(g,at(FACE_X-2.),np.arange(40,80)+HOLD_S,posts=legs)
        self.assertTrue(g.camera_backed[tuple(np.array(cs).T)].any(),'set-up: the cells beside the legs are backed')
        self.assertTrue(all(camera(g,cs)) and all(blocked(g,cs)))

    def test_someone_standing_in_front_of_an_overhang_does_not_hand_it_over_when_they_leave(self):
        # The sofa's face (camera-confirmed overhang edges, legs at its ends 45 cm out), a person's shins 10 cm in front of it for
        # 10 scans, then gone (astra's probe: before the fix 5 of the 9 face cells were released and mapped free).
        g=LidarGrid();legs=[(FACE_X+.05,-.45,.03),(FACE_X+.05,.45,.03)];face=np.array([(FACE_X,y) for y in np.arange(-.20,.201,.05)])
        run(g,at(FACE_X-.40),np.arange(0,5),posts=legs);g.add_camera_obstacles(face,margin=0);g.add_overhang_shadow(face,(FACE_X-.40,0.))
        cs=cells(g,face);run(g,at(FACE_X-.40),np.arange(5,10),posts=legs)
        run(g,at(FACE_X-.40),np.arange(10,20),posts=legs+[(FACE_X-.10,0.,.04)]);run(g,at(FACE_X-.40),np.arange(20,40),posts=legs)
        self.assertTrue(all(camera(g,cs)) and all(blocked(g,cs)),[bool(g.camera_obstacles[c]) for c in cs]);self.assertEqual(g.released,[])

    def test_a_released_cell_can_be_confirmed_by_the_camera_again(self):
        # The person's feet are released when they leave (above); the map lists the cells, LidarBrain passes them to the camera's
        # voter, which then confirms them afresh if something stands there again.
        g=LidarGrid();legs=[(FACE_X,0.,.05)];feet=[(FACE_X-.05,0.)]
        run(g,at(FACE_X-.40),np.arange(0,5),posts=legs);g.add_camera_obstacles(np.array(feet),margin=0)
        run(g,at(FACE_X-.40),np.arange(5,40));self.assertEqual([g.cell(p) for p in g.released],cells(g,feet))
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);b.grid=g;cam=b.low_camera;pts=np.array([[feet[0][0],feet[0][1],.05]]*3)
            for k,c in enumerate([(0.,0.),(.05,0.)]):cam.voter.add(pts,c,k)
            self.assertEqual(len(cam.voter.add(pts,(.10,0.),2)[0]),0,'set-up: confirmed once, not again')
            b.forget_released_camera_cells();self.assertEqual(g.released,[])
            got=[len(cam.voter.add(pts,c,10+k)[0]) for k,c in enumerate([(.2,0.),(.25,0.)])]
        self.assertEqual(got,[0,1],'confirmed again after two fresh views')

    def test_bumper_and_stall_marks_are_never_handed_to_the_lidar(self):
        g=LidarGrid();legs=[(FACE_X,0.,.05)];marks=[(FACE_X-.05,0.)]
        run(g,at(FACE_X-.40),np.arange(0,5),posts=legs);g.add_camera_obstacles(np.array(marks),margin=0,contact=True);cs=cells(g,marks)
        run(g,at(FACE_X-.40),np.arange(5,40));self.assertTrue(all(camera(g,cs)) and all(blocked(g,cs)))

    def test_a_saved_map_keeps_which_cells_are_bumper_or_stall_marks(self):
        # And a loaded map does not count camera cells as LiDAR evidence.
        g=LidarGrid();g.add_camera_obstacles(np.array([[1.,0.]]),margin=0,contact=True);g.add_camera_obstacles(np.array([[2.,0.]]),margin=0)
        with tempfile.TemporaryDirectory() as tmp:
            g.save(Path(tmp));h=LidarGrid();h.load(Path(tmp))
        self.assertTrue(h.contact_core[g.cell((1.,0.))]);self.assertFalse(h.contact_core[g.cell((2.,0.))])
        self.assertTrue(h.camera_obstacles[g.cell((2.,0.))]);self.assertEqual(float(h.lidar_logodds[g.cell((2.,0.))]),0.)

    def test_the_release_radius_reaches_a_toe_and_no_farther(self):
        self.assertGreaterEqual(CAMERA_BACK_RADIUS_M,.15);self.assertLess(CAMERA_BACK_RADIUS_M,.25)

    def test_floor_behind_a_confirmed_overhang_edge_is_mapped_under_it(self):
        # The LiDAR sees free floor under the overhang up to a wall 0.9 m behind its edge; the camera confirmed the edge from 40 cm.
        g=LidarGrid();run(g,at(FACE_X-.40),np.arange(0,6),posts=[(FACE_X+.90,0.,.5)])
        edge=np.array([[FACE_X,y] for y in np.arange(-.10,.101,.05)]);g.add_camera_obstacles(edge,margin=0)
        n=g.add_overhang_shadow(edge,(FACE_X-.40,0.));self.assertGreater(n,0)
        behind=[g.cell((FACE_X+d,0.)) for d in np.arange(.05,OVERHANG_SHADOW_M-.02,.05)]
        self.assertTrue(all(g.camera_obstacles[c] and g.logodds[c]>.8 for c in behind),'the floor right behind the edge is under the overhang')
        self.assertFalse(g.camera_obstacles[g.cell((FACE_X+OVERHANG_SHADOW_M+.10,0.))],'no deeper than OVERHANG_SHADOW_M')
        self.assertFalse(g.camera_obstacles[g.cell((FACE_X-.10,0.))],'nothing in front of the edge')
        # LiDAR beams passing under it do not clear it, and the LiDAR never takes it over.
        run(g,at(FACE_X-.40),np.arange(6,40),posts=[(FACE_X+.90,0.,.5)]);self.assertTrue(all(g.camera_obstacles[c] for c in behind))

    def test_the_shadow_stops_at_the_first_cell_the_lidar_does_not_see_free(self):
        # A leg 25 cm behind the edge: the shadow ends there; unmeasured floor is never marked.
        g=LidarGrid();run(g,at(FACE_X-.40),np.arange(0,6),posts=[(FACE_X+.25,0.,.03)])
        g.add_overhang_shadow(np.array([[FACE_X,0.]]),(FACE_X-.40,0.))
        self.assertTrue(g.camera_obstacles[g.cell((FACE_X+.10,0.))]);self.assertFalse(g.camera_obstacles[g.cell((FACE_X+.40,0.))])
        h=LidarGrid();self.assertEqual(h.add_overhang_shadow(np.array([[FACE_X,0.]]),(FACE_X-.40,0.)),0,'nothing measured, nothing marked')

    def test_a_partly_confirmed_overhang_front_no_longer_counts_the_floor_under_it(self):
        # Task H's house, its 2 m pocket closed only by the room's walls on two sides; its open side is an overhang's front of
        # which the camera has confirmed the middle 1.2 m (a head-on approach), the LiDAR seeing free floor under all of it.
        with tempfile.TemporaryDirectory() as tmp:
            for shadow in (False,True):
                b=LidarBrain(tmp);g=house('door',b);f=POCKET-1;g.logodds[f,POCKET-1:HI]=-5;g.logodds[POCKET-1:HI,f]=5   # open front, walled side
                g.lidar_logodds=g.logodds.copy();front=[g.world((f,c)) for c in range(POCKET+8,POCKET+32)];g.add_camera_obstacles(np.array(front),margin=0)
                cam=g.world((f-8,POCKET+20))
                if shadow:g.add_overhang_shadow(np.array(front),cam)
                counted=area(g.unreached_enterable(b.localizer.pose[:2]),g);reach=area(g.reachable(b.localizer.pose[:2])&(np.arange(g.n)[:,None]>=POCKET),g)
                if not shadow:base=(counted,reach)
            self.assertLess(counted+reach,base[0]+base[1]-.9,f'without the shadow {base}, with it {(counted,reach)}')

    def test_floor_walled_in_by_camera_cells_stays_walled_in_while_beams_pass_under_them(self):
        # Task H's house: a 2 x 2 m pocket of measured free floor closed by a line of camera cells (here: the camera-confirmed
        # front and end of an overhang). Scans from inside the room pass under the line into the pocket, from close by and
        # from far away, past HOLD_S. The pocket stays out of reach of the body, so it neither counts toward completion nor
        # as unvisited floor.
        with tempfile.TemporaryDirectory() as tmp:
            b=LidarBrain(tmp);g=house('marks',b);x0=g.world((0,POCKET+20))[0];wall_y=g.world((HI,0))[1]
            # The robot faces the fence (row POCKET-1) from 30 cm and from 1.5 m; its beams end on the room's far wall.
            for k,(y,t) in enumerate([(g.world((POCKET-7,0))[1],s) for s in range(4)]+[(g.world((POCKET-31,0))[1],s) for s in range(4,8)]
                                     +[(g.world((POCKET-7,0))[1],HOLD_S+10+s) for s in range(6)]):
                pose=np.array([x0,y,math.pi/2+.07*(k%2)]);a=pose[2]+np.radians(np.linspace(-50,50,301))
                ends=pose[:2]+((wall_y-pose[1])/np.sin(a))[:,None]*np.c_[np.cos(a),np.sin(a)];g.integrate_scan(inverse(ends,pose),pose,t)
            fence=[(POCKET-1,c) for c in range(POCKET-1,HI)]
            self.assertTrue(all(g.camera_obstacles[c] for c in fence))
            self.assertEqual(area(g.unreached_enterable(b.localizer.pose[:2]),g),0.);self.assertTrue(b.mapping_complete())
            self.assertLess(b.unvisited_area(),.5)

class SofaSouthEndTests(unittest.TestCase):
    """The turn into the sofa's south end (task Q, LIO-137). tests/fixtures/sofa-south-end (cut_south_end_fixture.py): task N's
    Unreal Go-to 'angled_from_north' with the camera LED on, GI on, true level poses. The robot curves round into the 0.57 m gap
    between the sofa's south legs (x -2.74 and -2.12, y 0.46; the base's face at y 0.33) and its turret meets the end's east part
    at the clip's end. The camera confirms only the end's west part, beside the west leg (0.44 m before the turret arrives), and
    nothing of the stretch the robot turns into. No map or motion rule closed the gap safely (task Q report section 2: a rule
    joining a confirmed edge to the next leg falsely walled in the 2D house); a touch sensor on the LiDAR head is the
    recommendation. Kept as the fast reproduction: it passes once something stops the turret here."""
    LEGS=((-2.74,.46),(-2.12,.46));UNDER=(-2.45,1.0)
    @unittest.expectedFailure
    def test_recorded_south_end_approach_is_stopped_before_the_turret_arrives(self):
        import cv2
        from autonomy.floor_parallax import LowObstacleCamera
        from autonomy.geometry import NARROW_RADIUS
        from recorded_camera import recorded_mount
        F=Path(__file__).parent/'fixtures/sofa-south-end';D=json.loads((F/'inputs.json').read_text())
        # Measured free floor round the end (only the end leads under the sofa here), the legs as the LiDAR's own obstacles.
        g=LidarGrid();y0,x0=g.cell((-2.95,-.6));y1,x1=g.cell((-1.95,1.6));g.logodds[y0:y1,x0:x1]=-3;g.seen[y0:y1,x0:x1]=5
        for leg in self.LEGS:g.logodds[g.cell(leg)]=5
        g.lidar_logodds=g.logodds.copy();g.radius=NARROW_RADIUS;cam=LowObstacleCamera();closed=None;west=None
        with recorded_mount(tuple(D['camera_mount_m'])):
            for r in D['sequences']['angled_from_north']:
                new=cam.process(cv2.imread(str(F/r['image']),cv2.IMREAD_GRAYSCALE),r['meta'],np.array(r['pose']))
                if len(new):g.add_camera_obstacles(new,margin=0)
                if len(cam.new_overhangs):
                    g.add_overhang_shadow(cam.new_overhangs,cam.camera_xy)
                    if west is None:west=r['turret_face_gap_m']
                if closed is None and not len(g.path((-2.43,-.35),self.UNDER)):closed=r['turret_face_gap_m']
        self.assertIsNotNone(west,'the camera confirms the west part');self.assertGreaterEqual(west,.40)
        self.assertIsNotNone(closed,'the gap stays open: the turret meets the end');self.assertGreaterEqual(closed,.20)

class PushedStallTests(unittest.TestCase):
    """The LiDAR turret meeting furniture in the blind band stops the wheels in Unreal (full-mapping-20261012T144002: 0.10 m/s
    commanded for 6 s, no motion, the wheels stopped, so no jam and no stall mark; the robot pushed at the sofa's end for 270 s
    and stopped 'stuck'). LidarBrain.pushed_stalled reports it, and the drive loop handles it as a jam (then a stall mark)."""
    def brain(self,v,moved=0.,pressed=False,span=2.5):
        b=LidarBrain(tempfile.mkdtemp());b.jam_reset=0.;b.bumper_pressed=pressed
        for i in range(int(span*20)+1):b.sent_v.append((10.+i/20,v))
        for i in range(int(span*4)+1):b.lidar_track.append((10.+i/4,np.array([moved*i/(span*4),0.,0.])))
        return b,10.+span
    def test_pushing_forward_without_moving_is_reported(self):
        b,t=self.brain(.10);self.assertTrue(b.pushed_stalled(t))
    def test_not_when_it_moves_or_creeps_or_the_bumper_is_pressed(self):
        b,t=self.brain(.10,moved=.05);self.assertFalse(b.pushed_stalled(t),'moving')
        b,t=self.brain(PUSH_V/2);self.assertFalse(b.pushed_stalled(t),'a creep slower than PUSH_V')
        b,t=self.brain(.10,pressed=True);self.assertFalse(b.pushed_stalled(t),'a bumper contact has its own handling')
        b,t=self.brain(.10,span=PUSH_STALL_S/2);self.assertFalse(b.pushed_stalled(t),'too short')
    def test_the_three_drive_loops_call_it(self):
        # Wiring only: the behaviour in the drive loop is tested end to end by scenarios_round's overhang_sofa_wheels_stop.
        src=Path(__file__).resolve().parents[1].joinpath('controller/autonomy/lidar_brain.py').read_text()
        self.assertIn("if self.pushed_stalled(t):",src);self.assertIn("self.begin_jam_settle(t,'Pushing forward does not move the robot.')",src)
        self.assertEqual(src.count('self.pushed_stalled(t)'),3,'follow, straight retry and Go-to approach')

if __name__=='__main__':unittest.main()
