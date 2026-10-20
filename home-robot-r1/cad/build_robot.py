"""Home Robot R1 — editable CadQuery packaging model, units mm.

Round revision: the drive axle runs through base_link (x=0), the outer contact skin is a 158.9 mm circle (<= 160 with play, tilt and tolerance)
about the z axis (two floating front bumper arcs plus a fixed two-piece rear shell), so the robot turns in
place inside its own footprint. One rear ball caster, centre of mass behind the axle.
Run with cadquery (2.6.1 or 2.8.0) and trimesh. Most electronic parts are dimensioned envelopes; the LiDAR is
the manufacturer LD14 STEP (reference-models/LD14.step). The build ends with cad/check_printability.py
(support-free MK3S gate) and cad/validate_meshes.py. See design-review.html.

`build_robot.py --step-only` (LIO-170) builds the same parts but writes only cad/home-robot-r1.step (git-ignored, so the full-robot
STEP download on the review page works without a tracked file changing): no print-parts, coupons, meshes, JSON, validation or gates.
"""
from pathlib import Path
import json, math, itertools, subprocess, sys
import cadquery as cq
import trimesh
import numpy as np
from OCP.BRepTools import BRepTools

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from pcb import board_spec as BS   # B9: the board's part numbers and pin -> net map for the harness and BOM tables (pure Python)
STEP_ONLY='--step-only' in sys.argv[1:]   # LIO-170: write only cad/home-robot-r1.step (see the docstring)
PRINT=ROOT/'print-parts'; PRINT.mkdir(exist_ok=True)
COUPONS=PRINT/'coupons';COUPONS.mkdir(exist_ok=True)
if not STEP_ONLY:
    for f in list(PRINT.glob('*.stl'))+list(PRINT.glob('*.step'))+list(COUPONS.glob('*.st*')):f.unlink()   # no stale files from removed or renamed parts
STL_TOL=(.02,.1)   # printed-part STL chord tolerance (mm) and angular tolerance (rad): sag <= 0.02 mm on the R157 arcs
# Jetson Nano B01 = NVIDIA's STEP (reference-models/jetson-nano-b01, simplified by cad/simplify_jetson.py); JO = robot position of the STEP
# origin (mounting hole (0, 0)), PCB underside z 57 (see the Jetson block below).
JDIR=ROOT/'reference-models'/'jetson-nano-b01'
JS=json.loads((JDIR/'jetson-nano-b01-simplified.json').read_text())
JO=(25,-46,57)
# Low-profile height chain (L2): ONE derived height drives the whole upper structure. Lid underside = top of the 40 mm fan on the Jetson
# heatsink + a >= 5 mm air gap, rounded up to 0.5 mm. Every height that hung from the deck-era lid (underside z 143) drops by DZ: lid
# sectors and posts, bumper and rear-shell tops (2 mm under the lid), LiDAR tray and LD14, lid switches. The camera no longer hangs from it:
# task F put it in the bumper band, fixed to the chassis behind a window in the bumper (see the camera block).
FAN_H,AIRGAP,LID_T=10.0,5.0,3.0
FS_HEAD=(2.8,2.1)                                       # fan screw pan heads (M3, radius and height) on the fan frame corners
ZFAN_TOP=JO[2]+JS['heatsink']['top_z']+FAN_H           # 95.28
ZLID=math.ceil((ZFAN_TOP+AIRGAP)*2)/2                   # lid underside 100.5
ZLIDT=ZLID+LID_T                                        # lid top 103.5
DZ=143.0-ZLID                                           # 42.5 mm drop from the deck-era lid
# Task L (LIO-120, user's "option 2": as low as the rules allow): the LD14 laser plane sits where the scan-band rule puts its floor, the band
# (plane ± 4 mm) 20 mm over the lid top (validation lidar_mount.scan_band_vertical): 127.5, 20 mm under task F's 147.5 (190 - DZ).
SCAN_OVER_LID,SCAN_HALF_BAND=20.0,4.0
LIDAR_H=ZLIDT+SCAN_OVER_LID+SCAN_HALF_BAND             # 127.5
assert abs((190-DZ)-LIDAR_H-20.0)<1e-9
PARAMS=dict(base_length=244,base_width=200,base_bottom=29,base_thickness=4,
            wheel_diameter=80,wheel_track=228,wheel_x=0,
            camera_baseline=0,camera_height=80.0,camera_x=146,lidar_height=LIDAR_H,lidar_x=54,lidar_yaw_deg=180,
            lidar_note='Task F (user decision 2026-10-11, option B): the LD14 is turned 180 deg about its spin axis (motor toward the back), spin axis x 54: the most forward position at which its '
                       '0.1 m minimum range (LD14P) stays inside the bumper skin with a half pressed to its stop, so the scan sees whatever the bumper touches (a trial at x 109, its front 10 mm inside '
                       'the skin, left a 5 cm blind strip ahead of the bumper). Its 0 deg (tail/motor) direction points along robot -x: a driver scan angle theta (REP-103, CCW, 0 = LD14 tail) maps to '
                       'the robot-frame bearing wrap(theta + 180 deg) into (-180, 180]; a raw LD14 angle a (clockwise from the tail) maps to wrap(180 deg - a). The simulator reports scans in the robot frame already. '
                       'Task L (LIO-120, user request 2026-10-12): the LD14 sits 20 mm lower than in task F, laser plane 127.5 mm (was 147.5): the scan band (plane ± 4 mm) is then exactly 20 mm over '
                       'the lid top, the floor the scan-band rule allows. Its lugs sit straight on the tray plate (no towers); its motor drops into the tray\'s fan intake duct, 1.4 mm over the fan frame.',
            outer_radius=158.9,shell_radius=157.4,rear_shell_radius=152.5,base_radius=148,bumper_tip_radius=152.7,
            bumper_axis_deg=30,bumper_travel=5.0,bumper_z=[18,ZLID-2],bumper_span_deg=90,
            rear_caster=[-75,0,0],caster_ball_radius=12.7,
            cliff_sensors=[[105,72],[105,-72],[-110,81],[-110,-81]],cliff_z=42.5,cliff_fov_deg=25,
            camera_hfov_deg=84,camera_image_px=[640,360],camera_check_image_px=[640,480],camera_check_vfov_deg=68.06,
            camera_led_points=[[145.8,0.0,89.45]],camera_led_intensity_cd=10.8,camera_led_beam_deg=120,camera_led_cct_k=5000,camera_led_current_a=0.059,
            camera_led_note='Task N (LIO-134): one Samsung LM281B+ Pro 2835 LED (5000 K, 37.5 lm at 65 mA / 2.7 V, 120 deg) above the lens, facing forward, '
                            'its emitting face at x 145.8, z 89.45. From J15 pin 5 (5 V) through 39 ohm: (5 - 2.7) / 39 = 59 mA, so ~37.5 x 59 / 65 = 34 lm and '
                            '~34 / pi = 10.8 cd on axis (a near-Lambertian emitter). Switched low-side by a TN0702N3-G MOSFET from GP20 (J15 pin 3).',
            camera_x_note='camera_x = lens front face; the image frustum apex used for every clearance check is 3 mm behind it (x 143), the assumed worst entrance-pupil position; the lens solid centre is x 141.4. '
                         'Task F: the camera sits in the bumper band behind a window in the two bumper halves (level, axis z 80). Below 80 mm the simulator camera loses the 6 cm shower tray in Unreal '
                         '(straight drives at it: found in 6/10 at 75 mm, 10/10 at 80 mm); 80 is also the highest axis whose image still fits a window closed under the 98.5 mm bumper top.',
            camera_image_note='The simulator renders 640 × 360 (16:9, as the OV2735 1920 × 1080 mode); the CAD clearance check deliberately uses a taller 4:3 frustum (VFOV 68.06 deg), which is conservative.',
            max_speed_mps=0.15,max_speed_source='home-robot-sim DriveCore.h max_speed (forward)',
            reverse_speed_limit_m_s=0.06,reverse_speed_limit_source='firmware reverse speed cap: the simulator firmware caps reverse at 0.06 m/s (the controller never commands more than 0.05 m/s in reverse); the rear-cliff coast check uses it',
            drive_model=dict(max_torque_nm=.10,setpoint_ramp_m_s2=.25,dynamic_brake_nm_per_rad_s=.010,rolling_drag_nm=.003,axle_inertia_kg_m2=.0005,hazard_action='disarm: tl = tr = 0, the wheels then see the simulator dynamic brake (coast)',
                source='home-robot-sim DriveCore.h (max_torque, accel: every wheel setpoint, braking included, ramps at 0.25 m/s2; disarm on hazard) and RobotPawn.cpp / robot-parameters.json (0.010 N m s/rad dynamic brake while disarmed, 0.003 N m rolling drag, 0.0005 kg m2 reflected axle inertia)'),
            base_x_range=[-124,120],printer_bed=[250,210,210])
parts=[]; shapes={}; cables=[]; TESS={}; ALLOW={}; BH=[]; RENDER={}   # RENDER: id -> [(solid, colour)], a render-only detail model (checks use shapes[id])
COL={'print':'#49768a','cover':'#c5d4db','owned':'#32876b','buy':'#bd8a38',
     'metal':'#b5bcc3','black':'#30363b','connector':'#ddcbaa','pcb':'#226c59',
     'power':'#c45749','signal':'#528fbc','usb':'#343c49','lens':'#284350','red':'#c8322d'}
AX=PARAMS['wheel_x']; R_SKIN=PARAMS['outer_radius']; R_SH=PARAMS['shell_radius']; R_RS=PARAMS['rear_shell_radius']; R_BASE=PARAMS['base_radius']; R_TIP=PARAMS['bumper_tip_radius']
WALL=2.4; RWALL=2.8; R_RS_IN=R_RS-RWALL; ZB0,ZB1=PARAMS['bumper_z']; BASE_X=tuple(PARAMS['base_x_range'])   # base plate x -124..120: 4 mm further aft (battery)
RZ=(18,27.5)               # rail / arm / cam band: top 1.5 mm under P_base (1.0 under the 0.5 mm race strips); the balls carry the vertical load
# M3 × 3 brass heat-set inserts (Ø4.0 through holes in the 4 mm plate) take every base screw that is not a rear-shell screw (>= 5 mm PETG
# engagement there). Each is pressed flush with the face its screw enters; the last 1 mm of the hole stays open as a relief for displaced PETG.
INS_R,INS_L=2.0,3.0
def ins_z(face):return 30.0 if face=='top' else 29.0   # insert bottom z: top-face inserts span z 30..33, bottom-face inserts z 29..32
CLIFF=[tuple(c) for c in PARAMS['cliff_sensors']]          # FL, FR, RL, RR
CLIFF_IDS=[CLIFF[2],CLIFF[3],CLIFF[0],CLIFF[1]]            # cliff_1..4 keep their ids: RL, RR, FL, FR
CZ=PARAMS['cliff_z']; CX=PARAMS['rear_caster'][0]; RB=PARAMS['caster_ball_radius']
BX=-96                                                      # battery tray centre: tray rear wall flush with the rear base edge x -124 (COM behind the axle, no ballast)
# Jetson Nano B01 = NVIDIA's STEP (reference-models/jetson-nano-b01, simplified by cad/simplify_jetson.py) on four H05 standoffs: the two
# front ones 24 mm from P_base, the two rear ones 20 mm from the bay board, which they clamp (low profile: no deck).
# The STEP is turned +90 deg about z, so its 100 mm I/O edge faces forward (+x): the USB, barrel
# and Ethernet ports point into the front bay toward the camera mount, and the heatsink and fan (on the far half of the module) sit over
# the axle. JO = robot position of the STEP origin (mounting hole (0, 0)). Carrier PCB underside z 57: motor top 52.5 + the 1.93 mm of
# through-hole leads under the PCB (2.23 allowed) + >= 2 mm; the bumper rail-pin B screws (top z 53) lie ahead of the carrier edge.
# x: the centre-of-mass guard (COM >= 15 mm behind the axle, every sensitivity case >= 14 mm) sets it; the carrier covers the bay board's
# under-carrier zone (x >= -38.5) and its rear standoffs clamp the board (validation com_estimate.placement).
# JDIR, JS and JO are defined at the top (the lid height derives from the fan top).
def J(x,y,z=0):return (JO[0]-y,JO[1]+x,JO[2]+z)   # NVIDIA STEP frame -> robot frame: +90 deg about z, then JO
JC=J(46,22.5)                                               # carrier centre (STEP x -4..96, y -17..62)
JHOLES=[J(*p)[:2] for p in JS['mounting_holes']['xy']]      # 86 × 58 mm pattern: (JO_x, -46), (JO_x - 58, -46), (JO_x, 40), (JO_x - 58, 40)
JFAN=J(*JS['heatsink']['fan_centre'],JS['heatsink']['top_z'])   # fan centre on the heatsink top (32 × 32 mm screw pattern)
JPORT=JS['ports']
JPCB=(JO[0]-62,JO[0]+17,JO[1]-4,JO[1]+96)                   # carrier PCB footprint x0, x1, y0, y1

def box(x,y,z,dx,dy,dz):
    return cq.Workplane('XY').box(dx,dy,dz,centered=(True,True,False)).translate((x,y,z)).val()
def cyl(x,y,z,r,h,axis='z'):
    s=cq.Solid.makeCylinder(r,h,cq.Vector(0,0,0),cq.Vector(0,0,1))
    if axis=='y':s=s.rotate((0,0,0),(1,0,0),-90)
    if axis=='x':s=s.rotate((0,0,0),(0,1,0),90)
    return s.translate((x,y,z))
def tidy(s):   # ShapeUpgrade clean when it succeeds and stays valid
    try:
        c=s.clean();return c if c.isValid() else s
    except Exception:return s
def union(*s):
    return s[0].fuse(*s[1:]) if len(s)>1 else s[0]
def rounded(x,y,z,dx,dy,dz,r=4):
    return cq.Workplane('XY').box(dx,dy,dz,centered=(True,True,False)).edges('|Z').fillet(r).translate((x,y,z)).val()
def hole(s,x,y,z,r,h,axis='z'):return s.cut(cyl(x,y,z,r,h,axis))
def tear(x,y,z,r,h):
    # Vertical hole with a 45 degree teardrop roof toward +x: support-free when the part prints on its -x face.
    k=cq.Workplane('XY').polyline([(0,0),(r*.7071,r*.7071),(r*1.4142,0),(r*.7071,-r*.7071)]).close().extrude(h).val()
    return union(cyl(0,0,0,r,h),k).translate((x,y,z))
def tearh(a,z,r,r0,r1):
    # Horizontal radial hole at azimuth a (deg) from radius r0 to r1, centre height z, 45 deg roof up: support-free upright.
    return tear(0,0,0,r,r1-r0).rotate((0,0,0),(0,1,0),-90).rotate((0,0,0),(0,0,1),a+180).translate((*polar(r0,a),z))
def stadium(p,q,r,z,h):
    # Straight slot between centres p and q (plan view), radius r, from z to z+h.
    (x0,y0),(x1,y1)=p,q;L=math.dist(p,q)
    s=box(0,0,z,L,2*r,h).rotate((0,0,0),(0,0,1),math.degrees(math.atan2(y1-y0,x1-x0))).translate(((x0+x1)/2,(y0+y1)/2,0))
    return union(s,cyl(x0,y0,z,r,h),cyl(x1,y1,z,r,h))
def prism(pts,z,h):
    if sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(pts,pts[1:]+pts[:1]))<0:pts=pts[::-1]
    return cq.Workplane('XY').polyline(pts).close().extrude(h).translate((0,0,z)).val()
def polar(r,a):return (r*math.cos(math.radians(a)),r*math.sin(math.radians(a)))
def obox(c,u,du,dv,z,h):   # box centred at c with its first side (du) along unit vector u
    return box(0,0,z,du,dv,h).rotate((0,0,0),(0,0,1),math.degrees(math.atan2(u[1],u[0]))).translate((c[0],c[1],0))
def xz_fin(pts,y0,y1):   # vertical fin from an (x, z) outline, y0..y1
    return cq.Workplane('XZ').polyline(pts).close().extrude(-(y1-y0)).translate((0,y0,0)).val()
def yz_fin(pts,x0,x1):
    return cq.Workplane('YZ').polyline(pts).close().extrude(x1-x0).translate((x0,0,0)).val()
def near(a,b,m=0):
    A,B=a.BoundingBox(),b.BoundingBox()
    return A.xmin-m<B.xmax and B.xmin-m<A.xmax and A.ymin-m<B.ymax and B.ymin-m<A.ymax and A.zmin-m<B.zmax and B.zmin-m<A.zmax
def add(id,name,s,cat='buy',bom='',group='lower',check=False,printable=False,note='',explode=None,tess=None,print_rot_z=0,allow=(),pshape=None,print_flip=False,render=None):
    assert s.isValid(),f'Invalid CAD solid {id}'
    if render:   # drawn (GLB, assembly meshes, renders) as these coloured solids; every check still uses s, which must contain them
        assert all(r.isValid() for r,_ in render),f'Invalid render solid {id}';RENDER[id]=render
    if tess:TESS[id]=tess
    if allow:ALLOW[id]=set(allow)
    b=s.BoundingBox()
    p=dict(id=id,name=name,category=cat,color=COL.get(cat,cat),bom=bom,group=group,
      bounds=[round(v,3) for v in (b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax)],
      printable=printable,check=check,note=note,**({'vertex_colors':True} if render else {}),
      explode=explode or {'base':[0,0,0],'lower':[0,0,25],'upper':[0,0,85],'camera':[40,0,110],'cover':[0,0,170],'wheels':[0,0,0]}.get(group,[0,0,0]))
    parts.append(p); shapes[id]=s
    if printable:
        # Export each physical printed part at the origin, with a flat datum on bed (optionally turned upside down about x - print_flip: the
        # face that was on top goes on the bed - and then turned about z). pshape: the as-printed solid when the assembly shows it otherwise
        # (bumper leaf springs drawn deflected; the battery tray's hold-down fingers drawn as their own id).
        ps=pshape if pshape is not None else s
        r=ps.rotate((0,0,0),(1,0,0),180) if print_flip else ps
        r=r.rotate((0,0,0),(0,0,1),print_rot_z) if print_rot_z else r;rb=r.BoundingBox()
        t=(-(rb.xmin+rb.xmax)/2,-(rb.ymin+rb.ymax)/2,-rb.zmin);out=r.translate(t)
        p['export']=dict(rot_x_deg=180 if print_flip else 0,rot_z_deg=print_rot_z,translate_mm=[round(v,4) for v in t])
        if not STEP_ONLY:
            cq.exporters.export(out,str(PRINT/(id+'.stl')),tolerance=STL_TOL[0],angularTolerance=STL_TOL[1])
            cq.exporters.export(out,str(PRINT/(id+'.step')))
    return s
def screw(id,x,y,z,length=8,r=1.5,group='lower',allow=(),name='M3 fastener envelope',bom='H01/H02'):
    s=union(cyl(x,y,z,r,length),cyl(x,y,z+length,2.8,2.8))
    add(id,name,s,'metal',bom,group,check=True,allow=allow,note='Thread not modeled; head and shaft clearance only.')
INOTE='M3 machine screw into an M3 × 3 brass heat-set insert in P_base (shank drawn at the minor diameter); tip modelled at its real depth.'
def tfs(id,x,y,z0,z1,head='pan',group='base',allow=(),name='M3 thread-forming screw from below',bom='H10',note='Thread-forming M3 into a 2.6 mm PETG pilot; shank drawn at the minor diameter.'):
    # Screw driven upward from below into a 2.6 mm pilot: head (pan, recessed) or 90 deg countersunk at z0, shank r1.25 to z1.
    h=cyl(x,y,z0,2.75,2.0) if head=='pan' else cq.Solid.makeCone(3.36,1.25,2.11,cq.Vector(x,y,z0))
    add(id,name,union(h,cyl(x,y,z0,1.25,z1-z0)),'metal',bom,group,check=True,allow=allow,note=note)
def tfa(id,x,y,zseat,ztip,head='pan',group='lower',allow=(),name='M3 thread-forming screw from above',bom='H10',note=INOTE):
    # Screw driven downward into a blind Ø2.6 pilot: pan head (2.0) or button head (1.65) on the seat, or 90 deg countersunk head whose top is at zseat.
    if head=='csk':h=cq.Solid.makeCone(1.25,2.75,1.5,cq.Vector(x,y,zseat-1.5));zs=zseat-1.5
    else:h=cyl(x,y,zseat,2.75 if head=='pan' else 2.85,2.0 if head=='pan' else 1.65);zs=zseat
    add(id,name,union(h,cyl(x,y,ztip,1.25,zs-ztip)),'metal',bom,group,check=True,allow=allow,note=note)
INSERTS={}
def insert(id,x,y,face,allow=()):   # M3 × 3 brass heat-set insert in a Ø4.0 P_base through hole (thread bore r1.5 to knurl r2.0), flush with the screw face
    INSERTS[id]=dict(xy=(x,y),face=face)
    add(id,'M3 × 3 brass heat-set insert ('+face+' face)',cyl(x,y,ins_z(face),INS_R,INS_L).cut(cyl(x,y,28,1.5,6)),'metal','H12','base',check=True,allow=('P_base',)+tuple(allow),
        note='Pressed in with a soldering iron, flush with the '+face+' face of P_base, with the plate lying on a flat steel backing plate (the open last 1 mm of the Ø4.0 hole takes the displaced PETG; deburr or countersink the far face afterwards). Follow the insert maker\'s hole size.')
def wire(id,name,pts,r=1.4,cat='signal',bom='C11',group='lower',note='Harness route envelope; bend fillets and service loops must be adjusted at assembly.'):
    # r: one radius, or one per segment (a lead that changes section, e.g. a flat row of wires leaving a housing that gathers into a bundle)
    rs=list(r) if isinstance(r,(list,tuple)) else [r]*(len(pts)-1);assert len(rs)==len(pts)-1,id
    seg=[]
    for (a,b),rr in zip(zip(pts,pts[1:]),rs):
        a,b=cq.Vector(*a),cq.Vector(*b); v=b-a
        if v.Length>0.01:seg.append(cq.Solid.makeCylinder(rr,v.Length,a,v.normalized()))
    for k,p in enumerate(pts[1:-1]):seg.append(cq.Solid.makeSphere(min(rs[k],rs[k+1]),cq.Vector(*p),angleDegrees1=-90))
    s=cq.Compound.makeCompound(seg)
    add(id,name,s,cat,bom,group,note=note)
    length=sum(math.dist(a,b) for a,b in zip(pts,pts[1:]))
    cables.append(dict(id=id,name=name,route_mm=round(length),suggested_cut_mm=int(math.ceil((length*1.2+40)/50)*50),points=pts))
def connector(id,name,x,y,z,pins=2,family='XH',group='upper',rotation=0,check=False):
    pitch=2.5 if family=='XH' else 3.96
    width=(pins-1)*pitch+(5.0 if family=='XH' else 6.3)
    depth=5.8 if family=='XH' else 8.8
    # Distinct socket and mating housing. Contacts are represented as envelope only.
    s=union(box(0,0,0,width,depth,6.5),box(0,0,6.8,width-1,depth-1,5.5))
    if rotation:s=s.rotate((0,0,0),(0,0,1),rotation)
    return add(id,name,s.translate((x,y,z)),'connector','C05' if family=='VH' else 'C06/C08',group,check=check,note=f'{family} {pins}-way mated envelope. Verify purchased housing dimensions.')
def base_edge_r(a):   # radius of the P_base outline along azimuth a (deg): R148 disc clipped to 240 x 200
    c,s=abs(math.cos(math.radians(a))),abs(math.sin(math.radians(a)))
    xl=-BASE_X[0] if math.cos(math.radians(a))<0 else BASE_X[1]
    return min(R_BASE,xl/c if c>1e-9 else 1e9,100/s if s>1e-9 else 1e9)

# Front bumper guides (left half; the right half mirrors y). Each printed 90 deg arc half floats on its own:
# three pins fixed in it (countersunk M3 from below) travel 5 mm in P_base slots along the half's own axis,
# 30 deg inward of rearward, so every point of the 0..90 deg arc pushes along the travel (cos(az-30) >= 0.5)
# and the skin only ever moves inward. Two rail pins carry 693ZZ bearings that roll on the slot sides (lateral);
# the vertical support also rolls, on METAL races only: Ø4 steel balls in rail/arm-top pockets roll between a hardened
# spring-steel strip in the pocket floor and a strip bonded to the base underside; balls under a floating printed rider
# on each pin roll between a strip on the base top and a strip in the rider pocket ceiling. A compression spring above
# each rider (set at assembly to a 0.1 mm feeler gap above a brass stop tube) pulls the half up against its three
# down balls: zero vertical play at rest, independent of the printed plate thickness and pocket depths.
# In-plane play: the two rail bearings sit in slots at most 8.2 mm wide (±0.1 mm side play), so each half may shift ±0.1 mm sideways
# and yaw ±0.28 deg; the skin radius, the seam gap, the bumper-end gaps and the leaf contact are all checked over every feasible
# (travel, side shift, yaw) pose of the two slot stadiums (validation bumper_travel.planar_play).
BT=PARAMS['bumper_travel'];BA=math.radians(PARAMS['bumper_axis_deg'])
ft=(-math.cos(BA),-math.sin(BA));BD=(BT*ft[0],BT*ft[1],0)   # left half travel direction and full-travel shift
nrm=(-ft[1],ft[0])         # in-plane normal to the travel (+ toward the rail side of the switch frame)
G=3.75                     # half seam gap: 7.5 mm at rest, 2.5 mm (nominal) when both halves bottom out
RAILY=(11,23)              # left rail band under P_base (y); it runs along x into the shell at az ~6 deg
RAILP=[(112,17),(65,17)]   # rail pins A, B: Ø8 bearings in 8.2 mm slots along the travel (yaw arm 40.7 mm)
SLOTW=8.2                  # bearing slot width (max): ±0.1 mm side play. FDM slots print 0.1-0.3 undersize: print 8.2, file to 8.1-8.2 (go 8.1 / no-go 8.25, slot coupon)
SPLAY=SLOTW/2-4.0          # bearing side play (0.1); the slot ends are placed so the bearing centre stops at exactly s = 0 and s = 5.0 on the slot axis
PB=(55,88)                 # pin C on the outboard arm: vertical (roll) support only (slot 0.6 mm wider, 0.3 mm longer each end: never touches)
BR=2.0                     # Ø4 mm steel balls (AISI 52100, G10): rolling vertical contacts
BALL_DN=[(76,17),(98,17),(54,95)]   # rail/arm-top pockets: roll on the base underside race (base pushes the half down)
RIDE=dict(A=7.2,B=7.2,C=7.2)        # rider balls each side of the pin slot (along nrm): roll on the base top race (base pushes the half up)
RSIDES=dict(A=(1,-1),B=(1,-1),C=(1,-1)) # two balls per rider (rider B corners chamfered; the driver moved 3 mm aft for it)
BPL=2*BR+BT+2.5           # pocket length along the travel (11.5): free length 7.5 = ±3.75 about the rest position; the ball rolls BT/2 = 2.5 mm in its pocket (1.25 mm margin at the hard stop)
PKR=2.5                    # ball pocket half-width (Ø5 stadium: holds the 4 × 9 mm floor/ceiling race)
RACE_T,RACE_W,RACE_L,PRACE=0.5,3.6,8.0,(4.0,9.0)   # hardened spring-steel race strips: base strips 3.6 × 8, pocket strips 4 × 9, 0.5 thick
ZBU=29.0-RACE_T            # underside race face z 28.5
ZBD=ZBU-2*BR-RACE_T        # down-ball pocket floor z 24.0 (race 24.0..24.5, ball 24.5..28.5)
ZBT=33.0+RACE_T            # top race face z 33.5
ZRB,ZRC,ZRT=34.5,ZBT+2*BR+RACE_T,39.6   # rider underside, pocket ceiling (race 37.5..38.0) and top
RIDT=ZRT-ZRB
SPR=dict(k=2.0,free=7.1,stop_tube=5.0,gap=0.1,od=5.0)   # music-wire spring Ø5 OD × Ø0.5 wire, 2.0 N/mm; brass stop tube Ø4 × 5; 0.1 mm feeler gap
ZCB=36.2                   # rider counterbore ceiling over the clamp nut (nut top 35.8): 0.4 mm up-stop, 0.15..0.65 with print tolerance
PRE_P=round(SPR['k']*(SPR['free']-(SPR['stop_tube']+SPR['gap'])),2)   # preload per pin (N): spring set to stop tube + 0.1 mm feeler gap
ZW=ZRT+SPR['stop_tube']+SPR['gap']   # washer underside z 44.7 (set at assembly), nyloc above
SW=(78,34)                 # SS-5GL2 body centre under P_base; switch frame +X = against the travel, +Y toward the rail
# Omron SS-5GL2 = hinge ROLLER lever (Omron SS datasheet): body 19.8 × 10.2 × 6.4, Ø2.35 holes 9.5 apart 2.9 mm from the terminal edge,
# Ø4.8 × 3.2 roller on a 14.5 mm lever; FP max 19.3, OP 14.5 ± 0.8, OT min 1.2, MD max 0.8 (roller top above the hole line).
HY=-2.2                    # mounting-hole line in the body-centred switch frame (body Y -5.1..5.1, lever side +Y)
XR,RR,XH=6.2,2.4,-8.3      # roller centre X, roller radius, lever hinge X
YOP,YFP,OTM,MDM=HY+14.5,HY+19.3,1.2,0.8
YREST,YDW=13.9,11.65       # cam holds the roller top at 13.9 at rest (0.8 above the release position) and 11.65 on the dwell (0.65 past OP)
KR,S0=math.tan(math.radians(40)),0.9   # 40 deg ramp line; it starts 0.9 mm of travel past the roller centre, i.e. the round roller meets it from about s 0
S1=S0+(YREST-YDW)/KR;SWADJ=0.9         # end of ramp; ± slot adjustment of the switch along frame Y
# Leaf spring: constant-strength taper 2.5 -> 1.45 mm, 9.5 mm tall, 48 mm from the root to a Ø4 nose boss (sized for >= 0.5 N return force
# after 35 % creep at E 1700 MPa and t - 0.05 mm, and <= 15 MPa at the hard stop at E 2000).
LR0,LL,LT,LZB=(61,91),48.0,1.45,(18,27.5)  # leaf spring root (on the arm), length to the nose centre, minimum thickness, z band (9.5 tall)
LRB,LTR,NR=2.0,2.5,2.0     # root boss radius; constant-strength (parabolic) leaf: 2.5 mm at the boss edge, t ~ sqrt(distance to the nose), >= 1.45; nose boss radius
def tq_(q,dt=0.):return (LTR if q<=LRB else max(LT,LTR*math.sqrt(max(LL-q,0)/(LL-LRB))))+dt
lax=(math.sin(BA),-math.cos(BA))         # leaf axis: perpendicular to the travel, so the leaf only bends
LT0=(LR0[0]+LL*lax[0],LR0[1]+LL*lax[1]);PL=2.4   # leaf nose centre and its preload into the base abutment
ABD,ABDEP=(-4.0,3.0),6.0  # abutment extent along the leaf axis from the free nose centre (root side incl. a 1 mm 45 deg lead, tip side) and its depth along the travel
def swW(s):return s.mirror('XZ').rotate((0,0,0),(0,0,1),math.degrees(BA)).translate((SW[0],SW[1],0))   # switch frame -> world (left)
def swL(s):return s.translate((-SW[0],-SW[1],0)).rotate((0,0,0),(0,0,1),-math.degrees(BA)).mirror('XZ')
def swp(X,Y):c,s=math.cos(BA),math.sin(BA);return (SW[0]+X*c+Y*s,SW[1]+X*s-Y*c)
def mir(p,s):return (p[0],s*p[1])
def pp(p,k,v=nrm):return (p[0]+k*v[0],p[1]+k*v[1])
TOWERS=[(113.5,47),(105,37.5)]           # camera mount foot screws (M3 x 10, nuts under P_base; the old camera tower's holes)
MTAB=5.0                                  # motor-mount tab thickness (z 33..38)

# Lid posts (L2): with the deck gone they stand on P_base, from z 33 to the lid underside. The deck-era columns (-105, ±82) / (87, ±82) are
# blocked from the base up by the rear cliff mounts, sensors and leads, so each post takes the place of a deck-era harness saddle (no bundle
# uses them since L1) and its two M3 heat-set inserts at x ±8: rear pair at x -80 (aft of the bay board, x -66.5), front pair at x 85.
LIDPOSTS=[(-80,-94),(-80,94),(85,-94),(85,94)]
POST_R,POST_FOOT=4.5,(22,12,3)   # Ø9 column on a 22 × 12 × 3 foot (the foot screws sit beside the column)
LID_IDS=('P_lid_front','P_lid_left','P_lid_rear','P_lid_right')   # round cover sectors 300..37, 37..120, 120..215, 215..300 deg
# P03 5 V converter (bay plan): turned 90 deg in the front bay, ahead of the bay board's tongue (x 7.5) and aft of the bumper riders B at full
# travel (validation p03_clear_of_bumper_travel): x 9..47, |y| <= 31, on 1 mm foam tape (z 33-34), body z 34-48. A cable tie over its top runs
# through two P_base slots beside its short ends. Drawn as the real HW-316E board (LIO-169, below): its IN / OUT terminals are on the short ends.
P03C,P03S=(28,0),(38,62,14)   # P03 centre and size (x, y, z)
# LIO-169: the tie line at x 39.5 (controller ruling, option a): over the two cap tops, clear of the HW-316E's end terminals (x <= 36.0),
# the trimpot (x >= 41.7) and the heatsink edge (x <= 37.5); the slots, sized to the 2.5 mm tie (TIE below), keep it there (validation p03_model.tie).
P03_TIE_X=39.5
P03_TIES=[(P03_TIE_X,s*(P03S[1]/2+2.3)) for s in (-1,1)]   # 3.1 × 2.6 slots at y ±33.3: 1.0 mm off the P03 short ends
TIES=[(-36,-92)]+P03_TIES                     # zip-tie slots in P_base (the (20, ±24) pair went with the P03's old long-side-along-x pose)
# Motor inline joints, factory-cable coils and their cable ties (PB25; B9 carry items 3 / 10). Each joint (pigtail male housing + the motor cable's
# factory female housing, on edge, axis along +-y) rests on P_base just ahead of its motor, between the motor (x <= 12.5) and the front Jetson standoff
# (x >= 22). The factory cable's spare length is wound into a flat coil (MCOIL, 2.75 turns, CW seen from above on the left, mirrored on the right) on the base further out, clear of the
# wheels (|y| >= 104) and the fan. Each joint and each coil is held by a cable tie (TIE, 2.5 x 1.1 mm band) looped round it through P_base: the joint
# tie's inner leg passes down the motor opening (x < 15, beside the motor can), its outer leg through a slot; the coil tie straddles the coil's +x
# side through two slots. Both are cut and replaced when the joint is unplugged.
MJ=dict(x=16.5,t=2.6,w=15.24,z0=33.05,y0=38.0,male_l=8.0,female_l=14.0,engage=6.0,unplug=8.0,tie_y=52.0)
MJ['z']=round(MJ['z0']+MJ['w']/2,3)          # joint axis height (the six contacts stand in a vertical column)
MCOIL=dict(c=(25.0,72.0),r=6.0,turns=2.75,z_top=49.0,pitch=4.0,start_deg=180.0)   # B12a: z_top 48.5 -> 49 (3.5 turns: FAC_TURNS)
TIE=dict(w=2.5,t=1.1,gap=.3,slot_clr=.3,z_under=27.85,head=(3.5,5.0,4.5),head_z=40.0)   # band width / thickness, gap to what it holds, slot clearance, band underside
# under P_base (0.05 mm clear of it), and the ratchet head (x, y, z size) on the outer leg's outside face from z head_z. The ties have no clash allowance:
# the slots and the motor opening clear them, so the clash scan checks them against P_base and the joint like any part.
MOTOR_OPENING_X=15.0                          # P_base motor openings: x -15..15 (bcut 'motor opening')
def tie_legs(xin0,xout1,ztop):   # (inner leg x0..x1, outer leg x0..x1, top bar z0..z1) of a tie holding x in [xin0, xout1], z up to ztop
    return (xin0-TIE['gap']-TIE['t'],xin0-TIE['gap']),(xout1+TIE['gap'],xout1+TIE['gap']+TIE['t']),(ztop+TIE['gap'],ztop+TIE['gap']+TIE['t'])
_cx,_cy=MCOIL['c'];_bw=2.0                  # coil bundle radius (the factory cable drawn Ø4, as MC_R)
MOTOR_TIES={'joint':tie_legs(MJ['x']-MJ['t']/2,MJ['x']+MJ['t']/2,MJ['z0']+MJ['w']),
            'coil':tie_legs(_cx+MCOIL['r']-_bw,_cx+MCOIL['r']+_bw,MCOIL['z_top']+_bw)}
assert MOTOR_TIES['joint'][0][1]<MOTOR_OPENING_X,'the joint tie inner leg must pass down the motor opening'
MOTOR_TIE_SLOTS=[]   # (x0, x1, y centre) per slot, both sides
for _s in (1,-1):
    (_i,_o,_)=MOTOR_TIES['joint'];MOTOR_TIE_SLOTS.append((_o[0]-TIE['slot_clr'],_o[1]+TIE['slot_clr'],_s*MJ['tie_y']))
    (_i,_o,_)=MOTOR_TIES['coil'];MOTOR_TIE_SLOTS+=[(_i[0]-TIE['slot_clr'],_i[1]+TIE['slot_clr'],_s*_cy),(_o[0]-TIE['slot_clr'],_o[1]+TIE['slot_clr'],_s*_cy)]
# Power & I/O bay board (bay B8: the real routed board): one 4-layer 1.6 mm PCB at base level under the unchanged Jetson, bottom z 35.4 on four
# 2.4 mm printed PETG spacers (M3 × 8 pan head into heat-set inserts in P_base), clamped in the two rear Jetson standoff stacks: P_base → 2.4 mm
# printed clamp spacer → board → M2.5 × 20 mm F-F standoff → carrier (z 57), one M2.5 × 12 countersunk screw from under P_base. Everything placed
# or checked round the board comes from pcb/board_cad.json ($KPY pcb/export_cad.py, read from the routed KiCad board, robot frame): the outline,
# the M3 and clamp holes, every THT pad, the edge connectors (courtyard, F.Fab body, entry direction, mated height, wire bend, lay-flat reach),
# the module sockets and the Pico USB receptacle. The board itself is the fab STEP pcb/fab/r1_power_io.step (robot = STEP + (-66.5, 50, 35.4)).
BC=json.loads((ROOT/'pcb'/'board_cad.json').read_text());_BF=BC['frame']
ZBAY,BAY_T=_BF['z_bottom'],_BF['thickness'];ZBAY_TOP=round(ZBAY+BAY_T,3)   # board bottom, thickness, top (37.0)
LEAD_TRIM=_BF['lead_trim_mm']   # THT leads trimmed to <= 1.4 mm under the board (z 34.0): 1.0 mm over the base top (z 33)
assert ZBAY_TOP==_BF['z_top'] and _BF['step_to_robot_mm']==[-66.5,50.0,ZBAY],'board_cad.json frame'
BAY_OUTLINE=[tuple(p) for p in BC['outline_xy']]   # robot x, y: the board's Edge.Cuts (B2 outline: rear edge x -66.4, tongue |y| <= 33.4 to x 7.5)
BAY_MOUNTS=[tuple(h['xy']) for h in BC['holes']]   # M3 holes H1..H4 over the P_base inserts (H3/H4 beside the tongue centre: ruling PB4)
BAY_CLAMPS=[tuple(h['xy']) for h in BC['clamp_holes']]   # rear Jetson standoff stacks (JHOLES 3 and 1) clamp the board: Ø3.0 NPTH (ruling PB3)
assert all(math.dist(a,b)<1e-6 for a,b in zip(BAY_MOUNTS+BAY_CLAMPS,[(-19.5,38),(-19.5,-38),(-11.5,8.5),(-11.5,-8.5),(-33,40),(-33,-46)])),'holes moved: the P_base inserts and the Jetson holes are fixed'
CARRIER_X=-38.5          # height-zone boundary: the Jetson carrier's rear edge (x -37) less the 1.5 mm margin
UNDER_CARRIER_Z=54.77    # carrier underside z 57 less the 2.23 mm allowed for its through-hole leads: under-carrier part tops <= UNDER_CARRIER_Z - 1.5
BCONN=BC['connectors'];BMOD=BC['modules']
def _bctr(b):return ((b[0]+b[2])/2,(b[1]+b[3])/2)
BAY_CONN={r:tuple(round(v,2) for v in _bctr(c['body_xy'])) for r,c in BCONN.items()}   # connector body centres: the harness ends (B9 sets the cut lengths)
PICO_USB=BC['pico_usb']['face_centre_xyz']   # Pico micro-USB receptacle face centre (at the board's left edge); checked against the placed Pico STEP
# Rear-edge connector row (plugged and unplugged with the lid off): the real courtyards (x0, x1, y0, y1).
REAR_REFS=('J13','J6','J7','J12','J2','J1')
BAY_REAR_ROW={r:(BCONN[r]['courtyard_xy'][0],BCONN[r]['courtyard_xy'][2],BCONN[r]['courtyard_xy'][1],BCONN[r]['courtyard_xy'][3]) for r in REAR_REFS}
# Printed bearing ribs on P_base under the rear row, from the real THT pads (board_cad.json tht_pads, copper bounding boxes), each >= RIB_GAP from
# every pad in plan (the leads hang 1.4 mm into the 2.4 mm gap) and under the connectors' bodies: rib 0 under the XH row J13 / J6 / J7 / J12
# between their pin row and the Pico socket U1A, rib 1 under J2 / J1 between their pins and the F2 holder (it stops RIB_SEP short of rib 0).
# Each is located by a RIB_POCKET deep pocket in the P_base top (RIB_CLR a side: room for the elephant's foot) and bonded there with epoxy;
# where a pocket would come within RIB_WALL of another base hole (the caster insert under J2) the rib stands on the plain base top instead,
# its underside stepped up by RIB_POCKET (pocket_spans; BAY_RIBS is set after the base hole lists, at CASTER_SCREWS).
RIB_GAP,RIB_W,RIB_SEP,RIB_POCKET,RIB_CLR,RIB_WALL=1.0,2.0,1.6,0.6,0.25,1.3   # RIB_WALL: P_base between a pocket and any other base hole (the base-hole wall gate 1.2 + 0.1)
BPADS=BC['tht_pads']
def pad_gap(b):   # (plan gap, pad) from a box (x0, y0, x1, y1) to the nearest THT pad's copper box
    return min((math.hypot(max(p['bbox_xy'][0]-b[2],b[0]-p['bbox_xy'][2],0.),max(p['bbox_xy'][1]-b[3],b[1]-p['bbox_xy'][3],0.)),f"{p['ref']}.{p['pin'] or 'peg'}") for p in BPADS)
def rib_from_pads(refs,y0=None,y1=None,w=RIB_W):
    """(x0, x1, y0, y1) of a rib along y under refs' courtyards: RIB_GAP in front of their own pins, RIB_GAP short of the next pins over its span."""
    ys=[BAY_REAR_ROW[r][2] for r in refs]+[BAY_REAR_ROW[r][3] for r in refs];y0=min(ys) if y0 is None else y0;y1=max(ys) if y1 is None else y1
    x0=max(p['bbox_xy'][2] for p in BPADS if p['ref'] in refs)+RIB_GAP                                                  # behind: the connectors' own pins
    nxt=min(p['bbox_xy'][0] for p in BPADS if p['bbox_xy'][0]>x0-RIB_GAP and p['bbox_xy'][1]<y1 and p['bbox_xy'][3]>y0)   # ahead: the next pins over that span
    return (round(x0,3),round(min(x0+w,nxt-RIB_GAP),3),round(y0,3),round(y1,3))
def pocket_spans(rib,holes,min_len=3.0):
    """The y-spans (y0, y1) of a rib's locating pocket: the rib's span less every stretch where the pocket (RIB_CLR a side) would come within
    RIB_WALL of a base hole ((x, y), r); pieces shorter than min_len are dropped. Closed form, no search."""
    x0,x1,y0,y1=rib;px0,px1=x0-RIB_CLR,x1+RIB_CLR;spans=[(y0,y1)]
    for (hx,hy),r in holes:
        dx=max(px0-hx,0.,hx-px1);reach=RIB_WALL+r
        if dx>=reach:continue
        half=math.sqrt(reach*reach-dx*dx)+RIB_CLR;c0,c1=hy-half,hy+half
        spans=[q for lo,hi in spans for q in ((lo,min(hi,c0)),(max(lo,c1),hi)) if q[1]-q[0]>1e-9]
    return [(round(lo,3),round(hi,3)) for lo,hi in spans if hi-lo>=min_len]
# Rear-edge hold-down (B8, PB24 amended): the band behind the carrier has no legal M3 spot within 35 mm of J1 / J2 / J7 / J12 (pcb side: every
# candidate meets another net's pad or a power zone in its 4.5 mm washer, or the Pico's SMD column), so the battery tray's front wall (x -70..-68,
# 1.6 mm behind the board's rear edge) carries three compliant hold-down fingers. Each finger sits in a gap between two rear-row connector bodies
# (HD_PAIRS), reaches HD['reach'] over the board's rear edge and hangs from the free end of a cantilever cut out of the wall (HD['L'] long along y,
# the wall raised to HD['top'] over it, bending vertically within the print layers). Unloaded, its underside is HD['preload'] below the board top,
# so with the board in place the beam is lifted by that much and holds the rear edge down on the ribs with a few newtons (hd_spring: the
# stack-up, force and stress). The slot under each beam is horizontal (a sloped one would leave sub-1.2 mm in-layer slivers in the 2 mm wall)
# and bridged by 1.3 mm breakaway posts every HD['post_pitch'] (2 mm bridges: the printability gate), freed after printing by running a fine
# razor saw along the slot. They print with the tray (one part); the CAD draws the beams and fingers in their installed pose as
# their own id (bay_holddown), so the board-outline tools list them with the ribs as designed contacts. The tray goes in after the board: set it
# 2 mm aft of its place and slide it forward so the fingers ride up their lead-ins onto the board's top face.
HD=dict(reach=1.95,w=1.4,lead=1.15,preload=.35,stack=.30,L=31.0,top=43.0,slot=.6,post_w=1.3,post_pitch=3.3,end_slot=1.0,z_over=.1,finger_h=3.4,relief_r=.75,
        E=2000.,E_creep=1700.,nu=.35,yield_MPa=50.,stress_limit_MPa=15.)
HD_PAIRS=(('J6','J13',1),('J12','J7',1),('J1','J2',-1))   # the bodies the finger sits between, the way its beam runs to its root (+-y)
HD_MIN_GAP,TRAY_FACE,TRAY_BACK=2.2,-68.0,-70.0
BAY_EDGE_X=min(p[0] for p in BAY_OUTLINE)   # the board's rear edge (x -66.4)
HOLDDOWN=[]   # (finger centre y, gap between the bodies, beam direction)
for _a,_b,_d in HD_PAIRS:
    _lo,_hi=sorted((BCONN[_a]['body_xy'],BCONN[_b]['body_xy']),key=lambda q:q[1]);_g=round(_hi[1]-_lo[3],3)
    assert _g>=HD_MIN_GAP,(_a,_b,_g);HOLDDOWN.append((round((_lo[3]+_hi[1])/2,3),_g,_d))
# The board is captive under the Jetson carrier (the rear standoffs clamp it); its rear band (Pico USB, rear connector row, fuses) stays reachable
# with the lid off, but the under-carrier connectors and the P03 only before the Jetson goes on: this fixes the assembly and service order written
# into the E01, H05, P03 and board notes.
def pin1_end(ref):
    """(which end of the lay-flat header pin 1 is at, in robot terms; True when pin 1 is on the left for someone looking into the mated socket's
    mating face, along the entry direction, with the socket's tails up)."""
    p_=BCONN[ref]['pins_xy'];ex_,ey_=BCONN[ref]['entry_xy'];lx,ly=-ey_,ex_   # left = z x (entry direction)
    return ('rear' if p_[0][0]<p_[-1][0] else 'front'),(p_[0][0]-p_[-1][0])*lx+(p_[0][1]-p_[-1][1])*ly>0
PIN1={r:pin1_end(r) for r in ('J4','J5')};assert all(v[1] for v in PIN1.values()),PIN1   # the same rule fits both headers: red on the left
ASSY_ORDER=('Assembly order (bay board under the Jetson): 0. on the bench, make the two motor pigtails (C18) and plug them on. They differ in length and in pin-1 order, so tag each one "L / J4" (left motor) or "R / J5" (right motor) right after making it, before it is plugged on. '
            'The pre-cut length of the six wires of each pigtail is @@PRECUT_J4@@ mm for J4 (left) and @@PRECUT_J5@@ mm for J5 (right): the drawn route + terminations + the full @@SPREAD@@ mm pin spread, rounded up to the mm, because the six wires fan out @@HALF@@ mm either side of the bundle\'s centre line at both ends (the socket\'s six sleeves and the male housing\'s six cavities each span @@SPREAD@@ mm). '
            'Solder the socket end first (one wire lapped on each solder tail of the right-angle socket under its Ø1.8 sleeve, red wire on pin 1, the tails and wires pointing up), then push the socket onto its header; leave the male ends long: they are dressed along the route, trimmed to length and crimped in step 4. '
            f'J4 pin 1 = {PIN1["J4"][0]} end, J5 pin 1 = {PIN1["J5"][0]} end (the square pads); the pigtails are built alike, so the rule is the same for both: looking into the socket\'s mating face with its tails up, the red wire (pin 1) is on the left. '
            'The pigtails stay with the board. '
            'Power connectors (ruling PB30): the battery (J1, an AMASS XT30 PCB male), the master rocker (J2, a JST VH header) and the Jetson 5 V (J3, a Molex Micro-Fit 3.0 right-angle header) are three different connector families, so no lead fits a wrong header; '
            'tag the three leads anyway ("J1 BATT", "J2 ROCKER", "J3 JETSON 5V") right after making them. '
            'Battery lead polarity: J1 pin 1 is GND (silk "-") and pin 2 is BAT+ (silk "+"), so the lead\'s XT30U-F carries GND in cavity 1 (its "-" contact: battery -, black) and BAT+ in cavity 2 (its "+" contact: battery +, red); '
            'the XT30 mates only one way round, but a lead soldered the wrong way round still mates. Check it fail-safe before the fuses go in: '
            'with F1 and F2 out, plug W01 into J1 and into the battery and measure F1\'s input clip (its BAT+ side, F1 pin 1) against a GND pad (TP5 or an M3 hole ring): it must read positive, about the pack voltage (≥ 6 V; a part-discharged pack reads lower); a reversed lead reads negative and, '
            'with the fuses out, has no current path. Only then fit the fuses, always with the rocker off (F1 first, F2 last): a reversed lead with the fuses in is the costliest mistake on this board (it has no reverse-polarity protection); '
            '1. bond the two printed bearing ribs into their 0.6 mm locating pockets in the P_base top with epoxy (F04) and let it cure; fit the two front Jetson standoffs (24 mm stacks), screwed from below; '
            '2. set the four 2.4 mm printed spacers and the two printed clamp spacers on P_base, lower the bay board onto them and screw it down with four M3 × 8 pan-head screws into the inserts; '
            '3. fit the battery tray (its hold-down beams freed: breakaway posts cut): set it 2 mm aft of its place, slide it forward so its three spring fingers ride up their lead-ins onto the board\'s top face between the rear connectors, screw it down, and check each finger stands lifted: '
            'a 0.05 mm feeler does not pass under its tip; if it does, that finger has play (it is off the board): stick 0.1 mm PTFE tape (F03) under its tip as a shim, a layer at a time (about 0.3 mm at most), until the feeler no longer passes; '
            f'4. plug the under-carrier cables. Pigtails: dress each one as drawn, with no slack (it rides up with the board when the joint is unplugged): straight up out of the socket, toward the front about 12 mm over the board beside U5A (J9 on the right), out between the P03 and the motor face, and down to the inline joint just ahead of the motor (x {MJ["x"]-MJ["t"]/2:g}..{MJ["x"]+MJ["t"]/2:g}, |y| {MJ["y0"]:g}..{MJ["y0"]+MJ["male_l"]+MJ["female_l"]:g}); '
            'then trim each wire at its own cavity of the male housing (finished lengths @@FIN_J4@@ for J4 and @@FIN_J5@@ for J5, set by where each wire sits at both ends) and crimp its male pin into the 1×6 housing, red wire = pin 1. '
            'Factory cables: measure each one from the encoder cap to the female housing\'s rear face (the wire entry); readings of @@R_LO@@–@@R_HI@@ mm all fit (2–3 turns), and a longer cable up to @@R_MAX@@ mm still fits (up to 3.5 turns). Route the fixed path first (about @@FIXED@@ mm outside the coil): a flat row along the motor\'s inner end face toward the rear, round the motor\'s rear shoulder and forward over the motor top (never over the pigtail), down the motor\'s front shoulder under the bumper lead, and from the coil position into the housing; '
            f'wind the remainder (the measured length less the fixed path: @@REM_LO@@–@@REM_HI@@ mm) into the coil on the base ahead of the motor (2–3.5 turns, about {2*MCOIL["r"]+4:g} mm across, at x {MCOIL["c"][0]:g}, |y| {MCOIL["c"][1]:g}) and tie it through its two slots. '
            'Then mate each motor\'s factory female housing onto its pigtail\'s male housing: the joint is not keyed, so mate the pin-1 stripe to the pin-1 stripe (both halves are marked at the red wire) and check the colours run straight across; stand the joint on the base and tie it down with a cable tie down the motor opening and up through the slot beside it; bumpers J10/J11; front cliffs J8/J9; '
            f'5. set and verify the P03 output voltage on the bench, connect it to U5A/U5B (pin 1 is "+" on both terminals, silk "+", the screw nearer the robot centre line: U5A.1 to P03 IN+, U5B.1 to P03 OUT+, red wires on pin 1; the P03\'s IN terminal is on the end toward the robot\'s left, display up and toward the rear), then fit it on its foam tape in the front bay with the cable tie through the P_base slots at ({P03_TIES[1][0]:g}, ±{P03_TIES[1][1]:g}); '
            'before the Jetson is connected (W04 unplugged): battery in, F1 fitted, rocker on, and measure against a GND pad: U5A.1 reads positive, about the pack voltage, and U5B.1 positive 5.1 V (0 V or negative: the P03 leads are swapped or loose); then rocker off; '
            '6. stand the two rear 20 mm F-F standoffs on the board and screw each down with an M2.5 × 12 countersunk screw from under P_base through the base, the clamp spacer and the board (they clamp the board); '
            '7. fit the Jetson (four M2.5 top screws), then its six plugs; before the 5 V barrel plug goes in, check the Jetson lead W04 on J3 with its barrel plug free and the rocker on: the plug\'s centre reads positive 5.1 V against its sleeve (negative: the two Micro-Fit crimps are swapped), then rocker off and plug it in. '
            'Service: step 0, every time, before reaching inside: shut the Jetson down, switch the rocker off and unplug the battery (BAT+ is live and unfused at F1\'s input whenever the battery is plugged in). '
            'Everyday rule: the rocker off before plugging or unplugging the battery; shut the Jetson down before switching the rocker off; fit or pull a fuse only with the rocker off (rocker off, fit F2, rocker on). '
            'With the lid off, the board\'s rear band (Pico USB, rear connector row, the Jetson 5 V lead J3, fuses) is reachable; the motor inline joints, the other under-carrier connectors and the P03 (tie, trimpot) need the Jetson off '
            '(its six plugs: camera, LiDAR and Pico USB, Wi-Fi dongle, 5 V barrel plug, fan; then its four top screws). To take the board out: Jetson off; the two rear standoffs off (their M2.5 screws from under P_base); the battery out (strap, Tamiya plug); '
            'the four tray screws out, the tray slid 2 mm aft and lifted straight out (its springs come off the board edge; its walls are cut clear of the rear cliff sensor boards); the four M3 board screws out; '
            f'unplug the rear-band plugs, the motor inline joints (loosen each factory-cable coil tie first: the {MJ["unplug"]:g} mm each housing slides back comes out of the coil; then cut the joint ties and slide each factory housing off its pigtail; the pigtails stay on J4 / J5 and ride up with the board), the bumper and front cliff leads and the P03 leads; lift the board. Fit new ties when it goes back (retie the coils).')
BAY_ZONES=(f'Height zones: under the carrier (x >= {CARRIER_X:g}) every part top, including a mated plug and its wire bend, <= z {UNDER_CARRIER_Z-1.5:.2f} ({UNDER_CARRIER_Z-1.5-ZBAY_TOP:.2f} mm over the board top); '
           f'behind it (x < {CARRIER_X:g}) <= z {ZLID-3:g} (lid underside - 3 mm), exposed with the lid off.')
TRAYS=[(BX-21,-66),(BX+21,-66),(BX-21,66),(BX+21,66)]
CASTER_BC=16.5                              # caster flange screws on a 16.5 mm bolt circle: outside the Ø25.4 ball (12.7 + 2.75 head + 1.0)
CASTER_SCREWS=[(CX+CASTER_BC*math.cos(math.radians(a)),CASTER_BC*math.sin(math.radians(a))) for a in [90,210,330]]
# The ribs (see RIB_GAP) and their pockets, now that the base holes are known.
_BASE_HOLES=[(p,INS_R) for p in BAY_MOUNTS+TRAYS+CASTER_SCREWS+[(cx+d,cy) for cx,cy in LIDPOSTS for d in (-8,8)]]+[(p,2.55) for p in JHOLES]   # inserts Ø4.0; Jetson countersinks Ø5.1
_r0=rib_from_pads(('J13','J6','J7','J12'));_r1=rib_from_pads(('J2','J1'),y1=_r0[2]-RIB_SEP)
BAY_RIBS=[_r0,_r1];RIB_POCKETS=[pocket_spans(r,_BASE_HOLES) for r in BAY_RIBS]
for _rb,_pk in zip(BAY_RIBS,RIB_POCKETS):assert _rb[1]-_rb[0]>=1.5 and pad_gap((_rb[0],_rb[2],_rb[1],_rb[3]))[0]>=RIB_GAP-1e-6 and sum(b-a for a,b in _pk)>=.5*(_rb[3]-_rb[2]),(_rb,_pk)

# Base: R148 disc clipped to a 244 x 200 mm plate, x -124..120 (one piece keeps both motor mounts aligned).
# Every hole is recorded (BH: name, solid) for the pairwise wall check in validation.
base=box(sum(BASE_X)/2,0,29,BASE_X[1]-BASE_X[0],200,4).intersect(cyl(0,0,28,R_BASE,6))
def bcut(name,s):
    global base;base=base.cut(s);BH.append((name,s))
for sign in [-1,1]:
    bcut(f'motor opening {sign:+d}',box(AX,sign*68,25,30,72,12))   # motor opening y 32..104 (3 mm to the motor end)
    for x in [AX-19,AX+19]:bcut(f'motor mount {x:+.0f},{sign*94}',cyl(x,sign*94,28,1.7,8))
for i,(x,y) in enumerate(JHOLES):bcut(f'jetson standoff {i}',union(cyl(x,y,28,1.45,8),cq.Solid.makeCone(2.55,1.45,1.1,cq.Vector(x,y,29))))   # M2.5 countersunk from below: nothing protrudes under the base
def ins_hole(name,x,y):bcut(name,cyl(x,y,28,INS_R,6))   # Ø4.0 through hole for an M3 × 4 brass heat-set insert (flush with the face the screw enters)
for i,(x,y) in enumerate(BAY_MOUNTS):ins_hole(f'bay mount insert {i}',x,y)   # bay board: M3 × 8 pan head through the board and its 2.4 mm printed spacer into the insert
for k,((x0,x1,_,_),pk) in enumerate(zip(BAY_RIBS,RIB_POCKETS)):
    for j,(y0,y1) in enumerate(pk):bcut(f'bay rib pocket {k}.{j}',box((x0+x1)/2,(y0+y1)/2,33-RIB_POCKET,x1-x0+2*RIB_CLR,y1-y0+2*RIB_CLR,RIB_POCKET+1))   # rib locating datum: 0.6 mm pocket in the P_base top (prints on the bed face)
for i,(x,y) in enumerate(CLIFF):
    bcut(f'cliff window {i}',box(x,y,28,13,14,8))                   # 13 x 14 window keeps 2 mm around the 25 deg ToF cone
    for xx in [-9.5,9.5]:bcut(f'cliff screw {i}{xx:+.1f}',cyl(x+xx,y,28,1.2,8))
for i,(x,y) in enumerate(CASTER_SCREWS):ins_hole(f'caster insert {i}',x,y)     # insert from below: M3 × 5 pan head through the 2 mm flange, tip z 32
def pslot(p,r,ext,s):   # P_base slot swept by a bumper pin from rest to the 5 mm hard stop (side s)
    u=(ft[0],s*ft[1])
    return stadium((p[0]-ext*u[0],p[1]-ext*u[1]),(p[0]+(BT+ext)*u[0],p[1]+(BT+ext)*u[1]),r,28,6)
ABUF=NR-PL                  # abutment face along the travel, from the free nose centre: the Ø4 nose boss is pressed PL into it
ABPOLY=[(ABUF,ABD[0]+1.),(ABUF+1.,ABD[0]),(ABUF+ABDEP,ABD[0]),(ABUF+ABDEP,ABD[1]),(ABUF,ABD[1])]   # (along travel, along leaf axis): face d -3..+3, root-side 45 deg lead to d -4
def abut(s):   # leaf-spring abutment under P_base: face normal to the travel; it spans d -3..+3 about the free nose (bending shortening and play keep the contact >= 1 mm inside)
    u=(ft[0],s*ft[1]);d=(lax[0],s*lax[1]);T=mir(LT0,s)
    return prism([(T[0]+a*u[0]+b*d[0],T[1]+a*u[1]+b*d[1]) for a,b in ABPOLY],18.5,10.6)
SWLEAD=(-14,-6)                                           # switch lead hole in the switch frame (behind the body)
for s in [-1,1]:
    for x,y in TOWERS:bcut(f'tower screw {x},{s*y}',cyl(x,s*y,28,1.7,8))                          # camera mount feet
    for k,p in zip('AB',RAILP):bcut(f'rail slot {k}{s:+d}',pslot(mir(p,s),SLOTW/2,-SPLAY,s))   # rail pins: Ø8 bearing, 8.2 mm slot (±0.1 mm side play); centred bearing stops at s 0 and 5.0
    bcut(f'pin C slot {s:+d}',pslot(mir(PB,s),3.1,.3,s))                                  # pin C: no side or end contact
    for X in [-4.75,4.75]:bcut(f'switch slot {X:+.2f}{s:+d}',stadium(mir(swp(X,HY-SWADJ),s),mir(swp(X,HY+SWADJ),s),1.2,28,6))   # M2 clearance slots along frame Y: set the switch at assembly
    bcut(f'switch lead {s:+d}',cyl(*mir(swp(*SWLEAD),s),28,3,8))                               # bumper switch lead
    base=union(base,abut(s))
for i,(x,y) in enumerate(TRAYS):ins_hole(f'tray insert {i}',x,y)   # insert from above: M3 × 6 countersunk from inside the tray
for i,(x,y) in enumerate(TIES):bcut(f'tie slot {i}',box(x,y,28,10 if i==0 else TIE['w']+2*TIE['slot_clr'],2.6,8))   # the P03 pair: the tie's width + clearance along x
for i,(x0,x1,yc) in enumerate(MOTOR_TIE_SLOTS):bcut(f'motor tie slot {i}',box((x0+x1)/2,yc,28,x1-x0,TIE['w']+2*TIE['slot_clr'],8))   # B9: joint / coil cable-tie slots
for i,(cx,cy) in enumerate(LIDPOSTS):
    for xx in [cx-8,cx+8]:ins_hole(f'lid post {i+1} insert {xx}',xx,cy)   # insert from above: M3 × 6 pan head through the post foot, tip at z 30

# Rear shell foot ribs: azimuths chosen PER SIDE by a scan of the real base-edge hardware on that side (saddles are not
# mirror-symmetric). Each rib screw goes into a Ø8 × 5 mm boss under P_base (pilot 8 mm deep, 1 mm skin).
def ring_pts(c,r,n=36):return [(c[0]+r*math.cos(2*math.pi*k/n),c[1]+r*math.sin(2*math.pi*k/n)) for k in range(n)]
def box_pts(x0,y0,x1,y1,n=12):return [(x0+(x1-x0)*k/n,y) for k in range(n+1) for y in (y0,y1)]+[(x,y0+(y1-y0)*k/n) for k in range(n+1) for x in (x0,x1)]
RIBW,BOSSR=14,4.0
def rib_poly(a,r0,w=RIBW):
    c,s=math.cos(math.radians(a)),math.sin(math.radians(a));n=(-s,c);r1=R_RS-1
    return [(r0*c+w/2*n[0],r0*s+w/2*n[1]),(r1*c+w/2*n[0],r1*s+w/2*n[1]),(r1*c-w/2*n[0],r1*s-w/2*n[1]),(r0*c-w/2*n[0],r0*s-w/2*n[1])]
def poly_gap(poly,pts):   # plan distance from a convex polygon to a point cloud (0 when a point is inside)
    P=np.array(poly);Q=np.array(pts);best=1e9
    for a,b in zip(P,np.roll(P,-1,0)):
        ab=b-a;t=np.clip(((Q-a)@ab)/(ab@ab),0,1);best=min(best,float(np.min(np.linalg.norm(Q-(a+t[:,None]*ab),axis=1))))
    inside=np.all([((b-a)[0]*(Q[:,1]-a[1])-(b-a)[1]*(Q[:,0]-a[0]))>=0 for a,b in zip(P,np.roll(P,-1,0))],0)|np.all([((b-a)[0]*(Q[:,1]-a[1])-(b-a)[1]*(Q[:,0]-a[0]))<=0 for a,b in zip(P,np.roll(P,-1,0))],0)
    return 0. if inside.any() else best
def rib_screws(a):e=base_edge_r(a);return [polar(e-7,a),polar(e-17,a)]
RIB_HARD=[(f'rear cliff window + ToF cone {n}',ring_pts(c,9.6)) for n,c in zip(('RL','RR'),CLIFF[2:])]+[(f'rear cliff screw head {c[0]+dx:.0f},{c[1]:.0f}',ring_pts((c[0]+dx,c[1]),2.0)) for c in CLIFF[2:] for dx in (-9.5,9.5)]+\
         [('rear caster housing',ring_pts((CX,0),20)),('tyre/wheel left',box_pts(-40,105,40,123)),('tyre/wheel right',box_pts(-40,-123,40,-105))]+[(f'Jetson standoff {i+1} countersunk head',ring_pts(p,2.55)) for i,p in enumerate(JHOLES)]
RIB_SOFT=[(f'lid post foot {i+1}',box_pts(x-11,y-8,x+11,y+8)) for i,(x,y) in enumerate(LIDPOSTS)]+[('signal bundle lane y=+94',box_pts(-108,88,94,100)),('power bundle lane y=-94',box_pts(-96,-98,90,-90))]+\
         [(f'lid post {i+1}',ring_pts(p,4.5)) for i,p in enumerate(LIDPOSTS)]
HC=[(n,s.BoundingBox()) for n,s in BH if not n.startswith('motor opening')]   # every other base hole, as a plan circle/box
def hole_gap(p,r):   # plan wall from a circle (p, r) to every recorded base hole (bounding boxes; circles by their own radius)
    best=(1e9,'')
    for n,b in HC:
        if b.xlen<=7.3 and abs(b.xlen-b.ylen)<.05:d=math.dist(p,((b.xmin+b.xmax)/2,(b.ymin+b.ymax)/2))-b.xlen/2-r   # round hole
        else:d=math.hypot(max(b.xmin-p[0],0,p[0]-b.xmax),max(b.ymin-p[1],0,p[1]-b.ymax))-r                          # slot / window: its box
        best=min(best,(d,n))
    return best
INS_XY=BAY_MOUNTS+TRAYS+CASTER_SCREWS+[(cx+d,cy) for cx,cy in LIDPOSTS for d in (-8,8)]   # every heat-set insert (flush with both base faces)
rib_scan={'left':[],'right':[]}
for side,sg in [('left',1),('right',-1)]:
    for a0 in range(96,178):
        a=sg*a0;poly=rib_poly(a,base_edge_r(a)-25)
        hard=min((poly_gap(poly,q),n) for n,q in RIB_HARD);soft=min((poly_gap(poly,q),n) for n,q in RIB_SOFT)
        wall=min(hole_gap(p,BOSSR) for p in rib_screws(a))
        insg=min(poly_gap(poly,ring_pts(p_,INS_R)) for p_ in INS_XY)   # a rib foot must not bear on an insert end face
        rib_scan[side].append(dict(az=a,hard_gap_mm=round(hard[0],1),hard_nearest=hard[1],plan_gap_to_saddles_lanes_posts_mm=round(soft[0],1),soft_nearest=soft[1],boss_wall_to_base_holes_mm=round(wall[0],1),boss_nearest=wall[1],gap_to_inserts_mm=round(insg,1)))
def pick(side,lo,hi):
    ok=[r for r in rib_scan[side] if lo<=abs(r['az'])<=hi and r['hard_gap_mm']>=4 and r['boss_wall_to_base_holes_mm']>=1.5 and r['gap_to_inserts_mm']>=1.0]
    return max(ok,key=lambda r:(min(r['hard_gap_mm'],12)+.25*min(r['plan_gap_to_saddles_lanes_posts_mm'],12)+.1*min(r['boss_wall_to_base_holes_mm'],10),-abs(abs(r['az'])-(lo+hi)/2)))['az']
RIBS={side:[pick(side,112,140),pick(side,148,174)] for side in ('left','right')}
for side in RIBS:
    for a in RIBS[side]:
        for k,p in enumerate(rib_screws(a)):
            base=union(base,cyl(*p,24,BOSSR,5.01))                     # Ø8 × 5 boss under the base (prints upward: base prints top face down)
            bcut(f'rear shell pilot {side} {a} {k}',cyl(*p,23.5,1.3,8.5))   # Ø2.6 pilot z 24..32: 8 mm deep, 1 mm skin; M3 × 10 tip at z 30.5
add('P_base','Single-piece chassis base',base,'print','F01','base',printable=True,note='R148 disc clipped to 244 × 200 × 4 mm (x -124..120, z 29–33). PETG, print top face down (the leaf-spring abutments and the rear-shell screw bosses under the base then point up). '
    'Slicer: NO brim; skirt distance 1 mm or skirt off for this part (244 mm of the 250 mm X travel); glue stick or a textured sheet, and watch the first layer at both X extremes (outside most of the mesh-bed-leveling grid). '
    f'First print P_coupon (print-parts/coupons): FDM slots come out 0.1–0.3 mm undersize; print the rail slots at {SLOTW} mm and file every one to {SLOTW-.1:.1f}–{SLOTW} mm with the coupon as the go / no-go gauge (go {SLOTW-.1:.1f}, no-go {SLOTW+.05:.2f}: a Ø8 bearing must roll on one side, ±0.05–0.1 mm side play, never touch both sides); check that the pin-C sleeve slides in its 6.2 mm slot. '
    f'Four {SLOTW} mm rail-pin slots (693ZZ bearings) and two 6.2 mm pin-C slots, 5 mm travel 30° inward of rearward. The bumper balls never touch PETG: bond the 0.5 mm hardened spring-steel race strips ({RACE_W} × {RACE_L:.0f} mm) to the base underside over each down-ball track and to the base top beside each slot (positions in the CAD, epoxy). '
    'The vertical guide is spring-preloaded, so the printed plate thickness does not set any play. Slotted switch holes; the Jetson standoff screws (M2.5) countersunk from below; rear-shell screws in Ø8 × 5 bosses (8 mm thread-forming pilots, 6.5 mm engagement); '
    f'bay-board, battery-tray, caster and lid-post foot screws go into {len(INS_XY)} M3 × 3 brass heat-set inserts in Ø4.0 through holes, each pressed in flush with the face its screw enters (plate on a flat steel backing plate; deburr the far face).')
for i,(x,y) in enumerate(BAY_MOUNTS):insert(f'insert_bay_{i}',x,y,'top',allow=(f'bay_spacer_{i}',))
for i,(x,y) in enumerate(TRAYS):insert(f'insert_tray_{i}',x,y,'top',allow=('P_battery_tray',))
for i,(x,y) in enumerate(CASTER_SCREWS):insert(f'insert_caster_{i}',x,y,'bottom',allow=('caster_housing','P_battery_tray'))   # flush with the base top under the tray floor
for i,(cx,cy) in enumerate(LIDPOSTS):
    for j,xx in enumerate([cx-8,cx+8]):insert(f'insert_lid_post_{i+1}_{j}',xx,cy,'top',allow=(f'P_lid_post_{i+1}',))

# Battery tray and pack, as far aft as the base allows. Pack is inserted with its 140 mm dimension across robot.
tray=box(BX,0,33,56,146,2)
tray=union(tray,box(BX-27,0,35,2,146,7),box(BX+27,0,35,2,146,7),box(BX,-72,35,52,2,7),box(BX,72,35,52,2,7))
for x in [BX-24.75,BX+24.75]:   # strap slots through the floor only, inside the full 2 mm side walls
    for y in [-42,42]:tray=tray.cut(box(x,y,32,2.5,14,4))
for x,y in TRAYS:   # Ø9 corner bosses (the battery seats on them at z 37); M3 × 6 countersunk machine screws into the inserts, heads 1.5 mm below the seat
    tray=union(tray,cyl(x,y,35,4.5,2)).cut(union(cyl(x,y,32,1.7,6),cq.Solid.makeCone(1.7,3.0,1.3,cq.Vector(x,y,34.2)),cyl(x,y,35.5,3.0,2.5)))
for x in [BX-12,BX+12]:
    for y in [-30,30]:tray=union(tray,box(x,y,35,8,8,2))   # seat pads
# Removal path (review I3): the rear cliff PCBs (cliff_1 / cliff_2: x -116.5..-103.5, |y| >= 72, z >= 42.5) overhang the tray's side walls, so the
# floor and side walls are trimmed back to |y| <= TRAY_TRIM_Y under them (allowing for the TRAY_SLIDE aft slide and 1 mm): after its screws are out
# and the slide, the tray lifts straight out (validation bay_board.tray_removal). The battery keeps its lateral stops on the rest of the walls.
TRAY_SLIDE,TRAY_TRIM_Y=2.0,71.0
_rc=[p for p in CLIFF_IDS if p[0]<0];TRAY_TRIM_X=(min(x for x,_ in _rc)-6.5+TRAY_SLIDE-1,max(x for x,_ in _rc)+6.5+TRAY_SLIDE+1)   # cliff PCB 13 mm long in x
for sy in (-1,1):tray=tray.cut(box(sum(TRAY_TRIM_X)/2,sy*(TRAY_TRIM_Y+2),32,TRAY_TRIM_X[1]-TRAY_TRIM_X[0],4,12))
# Bay board rear-edge hold-down (HD, HOLDDOWN). Per finger: the wall over its beam raised to HD['top']; a horizontal slot under the beam (top
# ZS, HD['slot'] high) through the wall, ending in a round relief at the root; a vertical end slot past the free end; breakaway posts in the slot; the finger on the beam's free-end face (unloaded underside z_f = board top - preload, an HD['lead'] x 45 deg lead-in (B9: >= preload + stack + 0.1)
# under the tip). The STL holds it all as printed; the assembly draws the tray without the beams and bay_holddown = beams + fingers lifted by the
# preload (turned about the beam root).
HD_ZF=round(ZBAY_TOP-HD['preload'],3);HD_ZS=round(HD_ZF-HD['z_over'],3);HD_XT=round(BAY_EDGE_X+HD['reach'],3)
def hd_slot_poly(z_lo,z_hi):   # an xz polygon through the wall thickness between z ZS + z_lo and ZS + z_hi
    return [(TRAY_BACK-.01,HD_ZS+z_lo),(TRAY_FACE+.01,HD_ZS+z_lo),(TRAY_FACE+.01,HD_ZS+z_hi),(TRAY_BACK-.01,HD_ZS+z_hi)]
HD_BEAMS=[]   # per finger: dict(yc, gap, d, y_end, y_root, posts)
_raised,_cuts,_posts,_fing=[],[],[],[]
for yc,gap,d in HOLDDOWN:
    ye=yc-d*HD['w']/2;yr=yc+d*HD['L'];lo,hi=sorted((ye,yr));n=math.ceil((hi-lo-HD['post_w']/2)/HD['post_pitch'])
    posts=[round(ye+d*(HD['post_w']/2+i*HD['post_pitch']),3) for i in range(n)]
    HD_BEAMS.append(dict(yc=yc,gap=gap,d=d,y_end=round(ye,3),y_root=round(yr,3),posts=posts))
    _ra,_rb=(lo+hi)/2+d*1.0-(hi-lo+2.0+2*HD['relief_r'])/2,(lo+hi)/2+d*1.0+(hi-lo+2.0+2*HD['relief_r'])/2;_ra,_rb=max(_ra,-73.0),min(_rb,73.0)   # kept within the tray's side walls (y +-73)
    _raised.append(box((TRAY_BACK+TRAY_FACE)/2,(_ra+_rb)/2,42-.01,TRAY_FACE-TRAY_BACK,_rb-_ra,HD['top']-42+.01))
    _cuts+=[xz_fin(hd_slot_poly(-HD['slot'],0.),lo,hi),                                                      # the slot under the beam
            box((TRAY_BACK+TRAY_FACE)/2,ye-d*HD['end_slot']/2,HD_ZS-HD['slot'],TRAY_FACE-TRAY_BACK+.02,HD['end_slot'],HD['top']-HD_ZS+HD['slot']+1),   # end slot
            cyl(TRAY_BACK-.01,yr,HD_ZS-HD['slot']/2,HD['relief_r'],TRAY_FACE-TRAY_BACK+.02,'x')]   # root relief (keyhole)
    _posts+=[xz_fin(hd_slot_poly(-HD['slot']-.01,.01),q-HD['post_w']/2,q+HD['post_w']/2) for q in posts]
    f=box((TRAY_FACE+HD_XT)/2,yc,HD_ZF,HD_XT-TRAY_FACE,HD['w'],HD['finger_h'])
    _fing.append(f.cut(xz_fin([(HD_XT-HD['lead'],HD_ZF-.01),(HD_XT+.01,HD_ZF-.01),(HD_XT+.01,HD_ZF+HD['lead']+.01)],yc-HD['w']/2-.01,yc+HD['w']/2+.01)))
tray=union(tray,*_raised)
for c_ in _cuts:tray=tray.cut(c_)
_beams=[tray.intersect(xz_fin(hd_slot_poly(-.02,HD['top']+1-HD_ZS),*sorted((b_['y_end'],b_['y_root'])))) for b_ in HD_BEAMS]
tray_fixed=tidy(tray.cut(union(*_beams)))
def hd_spring():
    """The finger springs: beam section (wall thickness x from the slot top to HD['top']), bending k_b = 3EI/L^3 plus the twist of the
    beam under the finger's offset load (k_t = GJ / (L e^2)), the stack-up of the board top against the finger's rest, and the force and root
    bending stress at the minimum, nominal and maximum stack."""
    n=400;b=TRAY_FACE-TRAY_BACK;xs=(np.arange(n)+.5)/n*b;dx=b/n;bot=HD_ZS+0*xs;h=HD['top']-bot   # xs from the front face
    A=float((h*dx).sum());zc=float(((bot+h/2)*h*dx).sum()/A);I=float(((h**3/12+h*(bot+h/2-zc)**2)*dx).sum());c=max(HD['top']-zc,zc-HD_ZS)
    xc=TRAY_FACE-float(((xs)*h*dx).sum()/A);e=(BAY_EDGE_X+HD_XT-HD['lead'])/2-xc;hm=float(h.mean());r=hm/b;J=(1/3)*(1-.63/r+.052/r**5)*b**3*hm
    stack=dict(rib_height_mm=.10,rib_pocket_depth_mm=.10,board_thickness_mm=.16,tray_floor_mm=.10,finger_rest_over_floor_mm=.10,base_flatness_mm=.10)
    rss=math.sqrt(sum(v*v for v in stack.values()));lin=sum(stack.values())
    out=dict(section=dict(area_mm2=round(A,2),I_mm4=round(I,2),c_mm=round(c,2),J_mm4=round(J,2),load_offset_mm=round(e,2)),beam_length_mm=HD['L'],
             stack_up_tolerances_mm=stack,stack_rss_mm=round(rss,3),stack_linear_mm=round(lin,3),design_stack_mm=HD['stack'],preload_mm=HD['preload'])
    for E,key in ((HD['E'],'E2000'),(HD['E_creep'],'E1700')):
        G=E/(2*(1+HD['nu']));kb=3*E*I/HD['L']**3;kt=G*J/(HD['L']*e*e);k=1/(1/kb+1/kt)
        rows={}
        for lab,dd in (('min',HD['preload']-HD['stack']),('nominal',HD['preload']),('max',HD['preload']+HD['stack']),('linear_min',HD['preload']-lin),('linear_max',HD['preload']+lin)):
            F=k*dd;rows[lab]=dict(deflection_mm=round(dd,3),force_N=round(F,2),root_bending_MPa=round(F*HD['L']*c/I,1))
        out[key]=dict(k_bend_N_mm=round(kb,2),k_twist_N_mm=round(kt,2),k_N_mm=round(k,2),**rows)
    m=out['E2000']
    # B9 carry item 12: the tip's 45 deg lead-in must be taller than the board corner stands over the finger's rest at the worst stack, or the tip's
    # front face meets the corner and jams when the tray slides forward: lead >= preload + design stack + 0.1 (and, here, the linear worst case too).
    req=round(HD['preload']+HD['stack']+.1,3);req_lin=round(HD['preload']+lin+.1,3)
    out.update(lead_in_mm=HD['lead'],lead_in_required_mm=req,lead_in_covers_linear_worst_case=HD['lead']>=req_lin-1e-9,lead_in_linear_required_mm=req_lin,lead_in_ok=HD['lead']>=req-1e-9,
               contact_flat_mm=round(HD['reach']-HD['lead'],3))
    # Slot-end (root) relief: the beam meets the wall at the end of its 0.6 mm slot in a keyhole of radius relief_r. Stress concentration at that root,
    # estimated from Peterson's stepped-bar bending chart (D/d >= 2) at r/h (h: the beam depth): about 2.0-2.3 at r 0.6 mm (B8), 1.8-2.1 at r 0.75 (B9: the
    # largest relief that keeps the printability gate's 1.2 mm in-layer wall to the nearest breakaway post; r 0.9 left a 1.12 mm sliver), 1.7-1.9 at r 0.9.
    hb=round(HD['top']-HD_ZS,2);kt=(1.7,1.9) if HD['relief_r']>=.9 else ((1.8,2.1) if HD['relief_r']>=.75 else (2.0,2.3))
    out['root_relief']=dict(relief_r_mm=HD['relief_r'],beam_depth_mm=hb,r_over_h=round(HD['relief_r']/hb,3),kt_estimate=list(kt),
        local_peak_MPa=dict(max_stack=round(kt[1]*m['max']['root_bending_MPa'],1),linear_max=round(kt[1]*m['linear_max']['root_bending_MPa'],1)),yield_MPa=HD['yield_MPa'],
        note='estimate (chart read, not an FE result): the peak sits at the slot end, under the beam root; PETG creep is the long-term limit, which the feeler check at assembly and service catches')
    out['ok']=(out['design_stack_mm']>=rss-1e-9 and m['min']['force_N']>0 and 2.0<=m['nominal']['force_N']<=5.0 and m['max']['root_bending_MPa']<=HD['stress_limit_MPa'] and out['lead_in_ok'])
    return out
HD_SPRING=hd_spring();assert HD_SPRING['ok'],HD_SPRING
_hs=HD_SPRING['E2000']
HD_NPOSTS=sum(len(b_['posts']) for b_ in HD_BEAMS)
_rr=HD_SPRING['root_relief'];_lmin,_lmax=_hs['linear_min'],_hs['linear_max']
HD_TEXT=(f'{len(HOLDDOWN)} compliant hold-down fingers on its front wall (CAD id bay_holddown; at y {", ".join(f"{yc:.2f}" for yc,_,_ in HOLDDOWN)}, in the gaps of the bay board\'s rear connector row): each hangs from the free end of a '
         f'{HD["L"]:g} mm cantilever cut out of the wall (wall raised to z {HD["top"]:g} over it; it bends vertically, within the layers), reaches {HD["reach"]} mm over the board\'s rear edge (flat contact {HD_SPRING["contact_flat_mm"]:g} mm, then a {HD["lead"]} × 45° lead-in under the tip) and rests {HD["preload"]} mm below the board top, so the board lifts it by that much and it '
         f'holds the rear edge down on the ribs with {_hs["nominal"]["force_N"]} N ({_hs["min"]["force_N"]}–{_hs["max"]["force_N"]} N over the ±{HD["stack"]} mm design stack-up: rib, pocket, board 1.6 ± 0.16, tray floor, print tolerance; RSS {HD_SPRING["stack_rss_mm"]} mm), '
         f'root bending stress <= {_hs["max"]["root_bending_MPa"]} MPa ({100*_hs["max"]["root_bending_MPa"]/HD["yield_MPa"]:.0f} % of a {HD["yield_MPa"]:g} MPa PETG yield; a Ø{2*HD["relief_r"]:g} mm keyhole relief at each beam root keeps the local peak near {_rr["local_peak_MPa"]["max_stack"]:g} MPa, Kt about {_rr["kt_estimate"][0]:g}–{_rr["kt_estimate"][1]:g}, an estimate). '
         f'At the linear worst case (all six tolerances at their limits at once, ±{HD_SPRING["stack_linear_mm"]:g} mm, unlikely but possible): a finger can stand up to {-_lmin["deflection_mm"]:.2f} mm off the board (no hold-down there until shimmed), or its beam bends to {_lmax["root_bending_MPa"]:g} MPa '
         f'({100*_lmax["root_bending_MPa"]/HD["yield_MPa"]:.0f} % of yield, local peak about {_rr["local_peak_MPa"]["linear_max"]:g} MPa). The {HD["lead"]} mm lead-in clears the board corner even there ({HD_SPRING["lead_in_linear_required_mm"]:g} mm needed; '
         f'{HD_SPRING["lead_in_required_mm"]:g} mm at the design stack), so a finger rides up instead of jamming. '
         f'After printing, free the beams: run a fine razor saw (kerf <= 0.5 mm) along each of the three 0.6 mm slots under them, cutting its {HD_NPOSTS // 3} breakaway posts, so each beam springs freely. Fit the tray after the board: set it {TRAY_SLIDE:g} mm aft of its place and slide it forward, the fingers riding up their lead-ins onto the board top. '
         'Check: each finger stands lifted about 0.35 mm off its rest, and a 0.05 mm feeler does not pass under its tip. If it does, either the board is not seated on the ribs, a post is left, or the stack-up ran to the low end: then stick 0.1 mm PTFE tape (F03) under that finger\'s tip, '
         'a layer at a time (about 0.3 mm at most), until the feeler no longer passes (the shim remedy for visible play).')
add('P_battery_tray','Battery tray with strap slots and the bay board hold-down springs',tray_fixed,'print','F01','lower',printable=True,pshape=tidy(union(tray,*_posts,*_fing)),
    note='Four corner bosses and four pads seat the pack at z 37; four M3 × 6 countersunk machine screws into M3 heat-set inserts in P_base (heads 1.5 mm under the pack, 3.5 mm thread engagement). '
         f'Floor and side walls trimmed to |y| <= {TRAY_TRIM_Y:g} for x {TRAY_TRIM_X[0]:g}..{TRAY_TRIM_X[1]:g}, under the rear cliff sensor boards: with the battery and the screws out it slides {TRAY_SLIDE:g} mm aft and lifts straight out. '
         'The front wall carries '+HD_TEXT+' The STL is the tray as printed (one part, floor down, no supports).')
def hd_installed(k):   # beam + finger k turned about the beam root so the finger's contact rises by the preload
    b_=HD_BEAMS[k];ang=-b_['d']*math.degrees(HD['preload']/HD['L']);zp=(HD_ZS+HD['top'])/2
    return union(_beams[k],_fing[k]).rotate((TRAY_BACK,b_['y_root'],zp),(TRAY_FACE,b_['y_root'],zp),ang)
HDF=cq.Compound.makeCompound([hd_installed(k) for k in range(len(HD_BEAMS))])
add('bay_holddown',f'Bay board rear-edge hold-down springs ({len(HOLDDOWN)}, printed with the battery tray)',HDF,'print','F01','lower',check=True,allow=('P_battery_tray',),
    note='Part of the P_battery_tray print, drawn installed (each beam lifted by the board through its preload): '+HD_TEXT+' Drawn as its own id so pcb/extract_outline.py lists it with the ribs as a designed contact.')
for i,(x,y) in enumerate(TRAYS):tfa(f'tray_screw_{i}',x,y,35.5,29.5,head='csk',allow=('P_base','P_battery_tray',f'insert_tray_{i}'),name='Battery tray M3 × 6 countersunk machine screw (into insert)',bom='H13')
add('battery','Existing Tenergy NiMH battery envelope',rounded(BX,0,37,50,140,27,3),'owned','P01','lower',check=True,note='Envelope allowance: 140 × 50 × 27 mm. Physical pack not measured. Centre x = -96 (tray rear wall flush with the rear base edge) to keep the robot centre of mass behind the axle without ballast.')
for i,y in enumerate([-42,42]):
    strap=union(box(BX,y,64.3,52,10,1),box(BX-25.5,y,35,1,10,30),box(BX+25.5,y,35,1,10,30))
    add(f'strap_{i}','Battery retaining strap',strap,'black','C12','lower')
TAM=(-82,-16)   # Tamiya mated pair on the battery top, between the straps and clear of the lid-top switches' plan areas (aft of the bay board)
add('tamiya','J01 battery connector, mated pair (on the battery top)',rounded(TAM[0],TAM[1],65,14,30,11,2),'connector','C04','lower',
    note='Lies along y on 1 mm hook-and-loop tape on the pack top (z 65-76), x -89..-75, y -31..-1: 4.7 mm ahead of the master rocker body and 6 mm from the -y strap. An adapter lead (W01) runs from it over the tray front wall to J1 on the bay board\'s rear edge.')

# Differential drive on the base_link axle (x = 0), shaft fits and fasteners.
for side,sgn in [('left',1),('right',-1)]:
    # All motor shapes are first made for the left side; mirror for right.
    motor=union(cyl(AX,35,40,12.2,14.7,'y'),cyl(AX,49.7,40,12.2,30.8,'y'),cyl(AX,80.5,40,12.5,23.5,'y'))
    shaft=cyl(AX,104,40,2,12.5,'y')
    motor=union(motor,shaft)
    if sgn<0:motor=motor.mirror('XZ')
    add('motor_'+side,'Pololu 4867 '+side+' encoder motor',motor,'metal','E05','wheels',check=True,explode=[0,sgn*55,0],note='Dimensioned simplified envelope; reference vendor STEP is included separately.')
    # Face plate (x ±23, y 104..107, z 25..55) outside the base edge; two 9 × 19 × 5 mm tabs on the base top carry the wheel load, each with a
    # 45 deg gusset (9 wide, 8 mm legs) into the plate's inner face: support-free when printed '+y down' (outer face on the bed).
    bracket=box(AX,105.5,25,46,3,30)
    for x in [AX-20,AX+20]:
        xc=AX+math.copysign(18.5,x-AX);bracket=union(bracket,box(xc,97.5,33,9,19,MTAB),yz_fin([(104.01,33+MTAB-.01),(96,33+MTAB-.01),(104.01,33+MTAB+8)],xc-4.5,xc+4.5))   # tabs x ±(14..23): 1.5 mm to the motor, 2 mm to saddle 3
    bracket=hole(bracket,AX,102,40,4.2,7,'y')
    for x in [AX-8.5,AX+8.5]:bracket=hole(hole(bracket,x,102,40,1.65,7,'y'),x,106,40,3.05,2,'y')   # 1 mm counterbore: button head 1.85 mm from the wheel disc
    for x in [AX-19,AX+19]:bracket=hole(bracket,x,94,32,1.7,MTAB+3)
    if sgn<0:bracket=bracket.mirror('XZ')
    add('P_motor_mount_'+side,'Motor face mount '+side,bracket,'print','F01','wheels',printable=True,explode=[0,sgn*45,0],note='17 mm motor-face screw spacing. The two 9 × 5 mm tabs carry the wheel load into the base top through 45 deg gussets (9 wide, 8 mm legs) on the plate\'s inner face: the tab bending stress is in validation interlayer_stress.motor_mount. Print on the outer face (+y down), supports disabled; the two Ø6.1 × 1 counterbores open on the bed face and their 1.4 mm ring ceilings print as short bridges (a 0.2 mm sacrificial layer would fail the 1.2 mm wall gate on this small part): if they sag, clean the seat with a Ø6 drill. Proof-load before use: hang 3 kg (about 3 g of wheel load) from the wheel for 1 min, no whitening at the tab roots.')
    tire=cyl(AX,109,40,40,10,'y').cut(cyl(AX,108,40,33,12,'y'))
    wheel=cyl(AX,109.5,40,33,9,'y').cut(cyl(AX,108,40,2.15,12,'y'))
    for a in range(0,360,60):
        x=AX+21*math.cos(math.radians(a));z=40+21*math.sin(math.radians(a))
        wheel=hole(wheel,x,108,z,7,12,'y')
    if sgn<0:tire=tire.mirror('XZ');wheel=wheel.mirror('XZ')
    add('tire_'+side,'80 mm tire '+side,tire,'black','M01','wheels',check=True,explode=[0,sgn*80,0])
    add('wheel_'+side,'Wheel disc and 4 mm collet '+side,wheel,'metal','M01','wheels',explode=[0,sgn*80,0],note='The Ø4.3 collet bore sits on the Ø4 D-shaft (0.15 mm radial clearance as drawn: the collet clamps it). The hub face starts 5.5 mm past the motor face plate (1.85 mm past the recessed button heads); set the wheel on the shaft with a 1.5 mm feeler between hub and plate.')
    for x in [AX-19,AX+19]:screw(f'motor_base_{side}_{x}',x,sgn*94,29,4+MTAB,group='wheels',allow=('P_base','P_motor_mount_'+side))
    for x in [AX-8.5,AX+8.5]:
        s=union(cyl(x,100,40,1.5,6,'y'),cyl(x,106,40,2.85,1.65,'y'))   # ISO 7380 button head recessed 1 mm: 1.85 mm to the wheel disc, 4 mm into the motor face
        if sgn<0:s=s.mirror('XZ')
        add(f'motor_face_{side}_{x}','Motor M3 × 6 button-head face screw',s,'metal','H06','wheels',check=True,allow=('motor_'+side,'P_motor_mount_'+side),explode=[0,sgn*45,0])

# One rear ball caster at x -75 (about 23 % of the weight): the rear cliff sensors (x -110) lead its loaded contact by 35 mm in reverse.
ball=cq.Solid.makeSphere(RB,cq.Vector(CX,0,RB),angleDegrees1=-90)   # full sphere (the default is a hemisphere)
housing=cyl(CX,0,14,20,15).cut(cq.Solid.makeSphere(13,cq.Vector(CX,0,RB),angleDegrees1=-90))
add('caster_ball','Rear caster ball',ball,'metal','M02','base',check=True)
add('caster_housing','Rear caster housing envelope',housing,'black','M02','base',note='Housing outline remains provisional; height is 29 mm. Three M3 × 5 pan-head machine screws on a 16.5 mm bolt circle (outside the Ø25.4 ball: the heads can never touch it) through the 2 mm flange tabs into M3 heat-set inserts pressed into P_base from below; tips 1.0 mm under the base top. Drill the flange tabs to this pattern if the purchased caster differs.')
for i,(x,y) in enumerate(CASTER_SCREWS):tfs(f'caster_screw_{i}',x,y,25.0,32.0,allow=('P_base','caster_housing',f'insert_caster_{i}'),name='Caster M3 × 5 pan-head machine screw (into insert)',bom='H13',note=INOTE)   # head under a 2 mm flange tab (z 27..29) outside the ball: tip z 32.0, 1.0 mm under the base top

# Power & I/O bay board (bay B8): the routed board from its fab STEP (laminate and every fitted part, leads trimmed to LEAD_TRIM), its plug-in
# modules from the official models on their real socket pads (Raspberry Pi Pico R3, Pololu 4035 DRV8874 carrier; the GY-521 as a body at its
# PARTS h), the mated plugs on its edge connectors, the Pico USB plug at the receptacle, four 2.4 mm printed spacers with M3 × 8 screws into the base
# inserts, two printed clamp spacers under the rear Jetson standoffs, and the printed bearing ribs under the rear connector row.
def step_components(path):
    """{component name: located shape} of an assembly STEP's top-level components (KiCad names each part by its reference; its board, copper and
    via bodies are unnamed components whose product names are r1_power_io_PCB / _copper / _via)."""
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TDocStd import TDocStd_Document
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.TDF import TDF_LabelSequence,TDF_Label
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.IFSelect import IFSelect_RetDone
    doc=TDocStd_Document(TCollection_ExtendedString('XmlOcaf'));rd=STEPCAFControl_Reader();rd.SetNameMode(True)
    assert rd.ReadFile(str(path))==IFSelect_RetDone,path
    rd.Transfer(doc);st=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main());top=TDF_LabelSequence();st.GetFreeShapes(top);assert top.Length()==1
    def name(l):
        n=TDataStd_Name();return n.Get().ToExtString() if l.FindAttribute(TDataStd_Name.GetID_s(),n) else ''
    comps=TDF_LabelSequence();st.GetComponents_s(top.Value(1),comps);out={}
    for i in range(1,comps.Length()+1):
        l=comps.Value(i);ref=TDF_Label();st.GetReferredShape_s(l,ref);n=name(l)
        out[name(ref) if n.startswith('=>') or not n else n]=cq.Shape.cast(st.GetShape_s(l))
    return out
STEP_SKIP=('r1_power_io_copper','r1_power_io_via')   # 35 um copper skins and the via barrels: no volume to speak of, but 70k mesh vertices
ZTRIM=round(ZBAY-LEAD_TRIM,3)
_SC=step_components(ROOT/BC['step']);assert {'r1_power_io_PCB',*STEP_SKIP}<=set(_SC) and set(BC['parts'])>={n for n in _SC if not n.startswith('r1_')},sorted(_SC)
def _trim(s):   # STEP frame -> robot, THT leads cut at LEAD_TRIM under the board
    s=s.translate(cq.Vector(*_BF['step_to_robot_mm']))
    return s.intersect(box(-20,0,ZTRIM,300,300,200)) if s.BoundingBox().zmin<ZTRIM-1e-6 else s
BSTEP={n:_trim(s) for n,s in _SC.items() if n not in STEP_SKIP}
# STEP heights: max(model, PARTS h) (pcb/fab/ORDERING.md). Lower in the STEP than PARTS h: D1 (an envelope at its PARTS h joins the board), the XH
# headers J6-J13 (their mated plugs, bay_plug_*, stand at max(MATED_H, PARTS h)) and the module sockets (the modules plug in on top). Higher: U5A / U5B
# (the model's 13.8 mm is kept). The sets are asserted against that table, so a model or PARTS h change stops the build here.
SOCKETS={s for m in BMOD.values() for s in m['sockets']}
STEP_H={r:round(BSTEP[r].BoundingBox().zmax-ZBAY_TOP,2) for r in BSTEP if r in BC['parts']}
_lower={r for r,h in STEP_H.items() if BC['parts'][r]['h']>h+.2};_higher={r for r,h in STEP_H.items() if h>BC['parts'][r]['h']+.2}
assert _lower=={'D1',*(f'J{i}' for i in range(6,14)),*SOCKETS} and _higher=={'U5A','U5B'},(_lower,_higher)
_env=[]
for r in sorted(_lower-SOCKETS-set(BCONN)):
    q=BSTEP[r].BoundingBox();_env.append(box((q.xmin+q.xmax)/2,(q.ymin+q.ymax)/2,ZBAY_TOP,q.xlen,q.ylen,BC['parts'][r]['h']))
STEP_ENVELOPES=sorted(_lower-SOCKETS-set(BCONN))
bb=cq.Compound.makeCompound(list(BSTEP.values())+_env)
_bq=bb.BoundingBox()
add('bay_board','Power & I/O bay board: the routed 4-layer board with its fitted parts (pcb/fab/r1_power_io.step)',bb,'pcb','E17/E18','lower',check=True,tess=(.3,.5),
    note=f'KiCad fab STEP (pcb/fab/r1_power_io.step, robot = STEP + (-66.5, 50, 35.4)): the laminate and every fitted part model (the J1 XT30, J2 JST VH and J3 Micro-Fit headers, the FLR fuse holders and C1 as their envelopes), '
         f'copper skins and via barrels left out; THT leads cut at {LEAD_TRIM} mm under the board (z {ZTRIM:g}: the assembly trims them to <= {LEAD_TRIM} mm, 1.0 mm over the base top); '
         f'{", ".join(STEP_ENVELOPES)} at {"its" if len(STEP_ENVELOPES)==1 else "their"} PARTS h (STEP heights: max(model, PARTS h)). Outline x {_bq.xmin:.1f}..{_bq.xmax:.1f}, |y| <= 50 (tongue |y| <= 33.4 for x > -13.8), z {ZBAY:g}–{ZBAY_TOP:g}; '
         'on four 2.4 mm printed PETG spacers with M3 × 8 pan-head screws into heat-set inserts at '+', '.join(f'({x:g}, {y:g})' for x,y in BAY_MOUNTS)+' (Ø3.2 holes), and clamped at '
         +', '.join(f'({x:g}, {y:g})' for x,y in BAY_CLAMPS)+f' (Ø{BC["clamp_holes"][0]["d"]:g} NPTH, ruling PB3) between a printed clamp spacer and a rear 20 mm Jetson standoff; its rear edge held down by {len(HOLDDOWN)} fingers on the battery tray wall. '+BAY_ZONES+' '+ASSY_ORDER)
# Plug-in modules on their socket pads (as $SP/show/modules_bay.py places them for the viewer): the module PCB underside rests on its male header
# plastic (HDR_H) on top of the 8.5 mm sockets; the headers' short pin ends stand 3.0 mm over the module PCB underside (through the PCB).
HMOD,HDR_H=BMOD['Pico']['pcb_underside_z'],_BF['header_h']
assert all(m['pcb_underside_z']==HMOD for m in BMOD.values())
def rigid(p1_m,a_m,b_m,n_m,P1,A,Bv):
    """(R, t): model point p1_m -> P1, model axes (a, b, n) -> robot (A, B, +z). Fails if the socket layout would mirror the module."""
    M=np.column_stack([a_m,b_m,n_m]).astype(float);Rr=np.column_stack([A/np.linalg.norm(A),Bv/np.linalg.norm(Bv),[0,0,1]])
    R=Rr@np.linalg.inv(M);assert abs(np.linalg.det(R)-1)<1e-6,f'socket layout mirrors the module: det {np.linalg.det(R)}'
    return np.round(R,12),P1-R@np.asarray(p1_m,float)
def moved_shape(shape,R,t):
    from OCP.gp import gp_Trsf
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    tr=gp_Trsf();tr.SetValues(*[float(v) for v in (R[0,0],R[0,1],R[0,2],t[0],R[1,0],R[1,1],R[1,2],t[1],R[2,0],R[2,1],R[2,2],t[2])])
    return cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped,tr,True).Shape())
def spin(mod,sock,n):x,y=BMOD[mod]['sockets'][sock][str(n)];return np.array([x,y,HMOD])
def strip(P,Q,z0,z1,w=2.54):   # a pin-row strip from pin P to pin Q (plan), w wide, w/2 past the end pins, z0..z1
    d=Q-P;L=float(np.hypot(d[0],d[1]))+w
    return box(0,0,z0,L,w,z1-z0).rotate((0,0,0),(0,0,1),math.degrees(math.atan2(d[1],d[0]))).translate(((P[0]+Q[0])/2,(P[1]+Q[1])/2,0))
def headers(rows,pcb_t):   # male header plastic under the module PCB and the short pin ends (0.64 mm pins) through it, 3.0 mm over the PCB underside
    return [strip(P,Q,HMOD-HDR_H,HMOD) for P,Q in rows]+[strip(P,Q,HMOD+pcb_t,HMOD+3.0,.64) for P,Q in rows]
_pico=cq.importers.importStep(str(ROOT/'reference-models'/'pico-r3'/'Pico-R3.step')).val()
P1,P20,P40,P60=spin('Pico','U1A',1),spin('Pico','U1A',20),spin('Pico','U1B',1),spin('Pico','U1B',20)
R_,t_=rigid((1.61,-0.5,-49.63),(0,0,1),(1,0,0),(0,1,0),P1,P20-P1,P40-P1)   # Pico-R3.step: PCB y -0.5..0.5 (parts +y), pin 1 (GP0) at x 1.61, z -49.63, USB end z -51
_usbc=np.array([R_@np.array(c)+t_ for c in itertools.product((6.71,14.29),(0.5,3.02),(-52.3,-46.7))])   # micro-B receptacle (body y 0.5..3.02, shell tabs to z -52.3)
USB_FACE=(round(float(_usbc[:,0].mean()),3),round(float(_usbc[:,1].max()),3),round(float(_usbc[:,2].mean()),3))
assert all(abs(a-b)<.3 for a,b in zip(USB_FACE,PICO_USB)),(USB_FACE,PICO_USB)
add('bay_pico','Raspberry Pi Pico on sockets (official Pico R3 STEP)',cq.Compound.makeCompound([moved_shape(_pico,R_,t_)]+headers([(P1,P20),(P40,P60)],1.0)),'owned','E02','lower',check=True,tess=(.3,.5),allow=('pico_usb_plug',),
    note=f'Raspberry Pi Pico-R3.step on the U1A / U1B socket pads (pin 1 GP0 at U1A.1, not mirrored), PCB underside z {HMOD:g} on its male headers ({HDR_H:g} mm plastic on the 8.5 mm sockets). Behind the Jetson carrier '
         f'(exposed with the lid off), long axis along y, micro-USB receptacle face at ({USB_FACE[0]:.2f}, {USB_FACE[1]:.2f}, {USB_FACE[2]:.2f}), at the board\'s left edge.')
_drv=cq.importers.importStep(str(ROOT/'reference-models'/'pololu-4035-drv8874-carrier.step')).val()
for k,(mod,n,sd) in enumerate((('DRV_L',3,'left'),('DRV_R',4,'right'))):
    L1,L7,Q1,Q7=spin(mod,f'U{n}L',1),spin(mod,f'U{n}L',7),spin(mod,f'U{n}P',1),spin(mod,f'U{n}P',7)
    R_,t_=rigid((1.27,16.51,0),(0,-1,0),(1,0,0),(0,0,1),L1,L7-L1,Q1-L1)   # Pololu 4035 STEP: z-up, VM / logic row L at x 1.27 (pin 1 at y 16.51), VIN row P at x 13.97
    add(f'bay_drv_{k}',f'Pololu DRV8874 carrier on sockets, {sd} motor (official Pololu 4035 STEP)',cq.Compound.makeCompound([moved_shape(_drv,R_,t_)]+headers([(L1,L7),(Q1,Q7)],1.6)),'pcb','E16','lower',check=True,tess=(.3,.5),
        note=f'Pololu 4035 STEP on the U{n}L / U{n}P socket pads (turned 180 deg in place, PB15 / PB16: logic row toward the Pico, VIN / OUT row toward J{4 if n==3 else 5}; not mirrored), PCB underside z {HMOD:g}. Under the Jetson carrier.')
_r1,_r8=spin('IMU','U2',1),spin('IMU','U2',8);_d=(_r8-_r1)/np.linalg.norm(_r8-_r1);_e=np.array([-_d[1],_d[0],0.])   # row direction; module body side (+90 deg)
_ib=np.array(BMOD['IMU']['body_xy']);_ic=_ib.mean(0);_ia=math.degrees(math.atan2(_d[1],_d[0]))
def _imu_box(c,lx,ly,z0,h):return box(0,0,z0,lx,ly,h).rotate((0,0,0),(0,0,1),_ia).translate((float(c[0]),float(c[1]),0))
IMU_TOP=round(ZBAY_TOP+BC['parts']['U2']['h'],3)
_imu=[_imu_box(_ic,21.2,16.4,HMOD,1.6),_imu_box(_ic,4,4,HMOD+1.6,.9)]+[_imu_box(_ic+_d[:2]*s_+_e[:2]*4,3,1.6,HMOD+1.6,1.1) for s_ in (-5,5)]+headers([(_r1,_r8)],1.6)
add('bay_imu','GY-521 IMU on a socket (21.2 × 16.4 × 1.6 mm board, MPU-6050)',cq.Compound.makeCompound(_imu),'owned','E03','lower',check=True,
    note=f'GY-521 board (board_spec MODULES IMU) on the U2 socket pads, PCB underside z {HMOD:g}, parts and pin ends to z {IMU_TOP:g} (PARTS h). Under the Jetson carrier, over C3 (DRV_L\'s 100 uF V-chip): validation bay_board.c3_under_imu. Rigid attachment required (socket plus a screw or adhesive pad).')
# Mated plugs on the edge connectors (the harness housings; the P03 leads go straight into the screw terminals U5A / U5B): vertical XH / VH plugs on
# the header body up to max(MATED_H, PARTS h). J4 / J5 (right-angle 1x6 headers, B3): B3 sized their lay-flat zone for the motors' factory 1x6 housing
# (MOTOR_PLUG_L long from the header body face to the far end, courtyard + MATED_REACH: the B3 envelope, kept as the largest plug the zone takes). What
# is plugged there now is the C18 pigtail's board end (B9, carry item 2): a 1x6 2.54 mm RIGHT-ANGLE female socket header (RA_SOCKET; e.g. Sullins
# PPPC061LGBN-RC, verify) pushed onto the pins, its insulator lying on the board from the header body face, its six solder tails leaving its back face
# and turned UP; each wire is soldered to a tail tip under a heat-shrink sleeve, so the wires leave the board upward, well clear of the motor face, and
# bend over above the sleeves (the pigtail route below) - not along the pins into the 2.16 mm gap at the motor face (B8: 0.8 mm bend).
MOTOR_PLUG_L=14.0
RA_SOCKET=dict(part='Sullins PPPC061LGBN-RC: 1x6 2.54 mm right-angle female socket header, through-hole (2.54 mm insulator lying on the board, 8.51 mm deep, tails 3.15 mm; verify)',
               depth=8.51,h=2.54,tail_off=1.27,tail_len=3.15,sleeve_d=1.8,sleeve_past_tail=2.5,solder_overlap=2.0)
# h: the insulator (it lies on the board; contact axis at h / 2 = J4 / J5's pin height); tail_len: the tails stand 3.15 mm past the insulator face they
# leave (turned up: past its top face); tail_off: tail centre behind the rear face (0.050 in, from memory: verify on the Sullins drawing); the wire is
# lapped solder_overlap mm on each tail tip under a Ø sleeve_d heat-shrink that reaches sleeve_past_tail mm past the tip.
# B12a (ruling PB30): J1 / J2 / J3 are three connector families (XT30 / JST VH / Micro-Fit 3.0), each with its own mate (PLUG_BOM). J3 is a
# right-angle Micro-Fit header in the band: its receptacle enters its face at the board's right edge and stands out past it (board_cad
# offboard_reach: receptacle + crimp ends), at the header's height (MATED_H), the lead leaving its far end (plug_end_xy).
PLUG_BOM=dict(VH='C05',XT30='C19',MicroFit='C20')
PLUG_BOX={};SOCKET={}
for r,c in BCONN.items():
    if c['family']=='Terminal':continue
    bx0,by0,bx1,by1=c['body_xy']
    if c['vertical']:PLUG_BOX[r]=(bx0,by0,bx1,by1,ZBAY_TOP,ZBAY_TOP+max(c['mated_h'],BC['parts'][r]['h']))
    elif c['offboard_reach']:
        (ex,ey),(px,py)=c['entry_xy'],c['plug_end_xy'];face=(bx1 if ex>0 else bx0) if ex else (by1 if ey>0 else by0)
        PLUG_BOX[r]=((min(face,px),by0,max(face,px),by1) if ex else (bx0,min(face,py),bx1,max(face,py)))+(ZBAY_TOP,round(ZBAY_TOP+c['mated_h'],3))
    else:
        (ex,ey),(cx0,cy0,cx1,cy1)=c['entry_xy'],c['courtyard_xy'];W=2.54*len(c['pins_xy']);pc=np.mean(c['pins_xy'],0)
        assert not ex and r in ('J4','J5'),r   # only the motor headers lie on the board, entry along y
        far=cy1+c['mated_reach'] if ey>0 else cy0-c['mated_reach'];face=far-ey*MOTOR_PLUG_L;back=face+ey*RA_SOCKET['depth'];tail=back+ey*RA_SOCKET['tail_off']
        y0,y1=sorted((face,back));x0,x1=pc[0]-W/2,pc[0]+W/2
        PLUG_BOX[r]=(x0,y0,x1,y1,ZBAY_TOP,round(ZBAY_TOP+RA_SOCKET['h'],3))
        top=round(ZBAY_TOP+RA_SOCKET['h']+RA_SOCKET['tail_len']+RA_SOCKET['sleeve_past_tail'],3)   # sleeve tops (z 45.19): where the wires leave, upward
        ty0,ty1=sorted((back,tail+ey*RA_SOCKET['sleeve_d']/2))
        SOCKET[r]=dict(face_y=round(face,3),back_y=round(back,3),tail_y=round(tail,3),x=(round(x0,3),round(x1,3)),pc_x=round(float(pc[0]),3),top_z=top,
                       tails=(x0,ty0,x1,ty1,ZBAY_TOP,top),b3_far_y=round(far,3))
    x0,y0,x1,y1,z0,z1=PLUG_BOX[r]
    if r in SOCKET:
        tx0,ty0,tx1,ty1,tz0,tz1=SOCKET[r]['tails']
        add(f'bay_plug_{r}',f'{r}: C18 pigtail board end (right-angle female socket, tails and wire sleeves up)',union(box((x0+x1)/2,(y0+y1)/2,z0,x1-x0,y1-y0,z1-z0),box((tx0+tx1)/2,(ty0+ty1)/2,tz0,tx1-tx0,ty1-ty0,tz1-tz0)),
            'connector','C18','lower',check=True,allow=('bay_board',),
            note=f'{RA_SOCKET["part"]} on {r}\'s right-angle pins: insulator y {y0:.2f}..{y1:.2f}, z {z0:g}–{z1:g} on the board; its six tails turned up behind it (y {SOCKET[r]["tail_y"]:.2f}), a wire soldered to each tail tip under '
                 f'a Ø{RA_SOCKET["sleeve_d"]} heat-shrink sleeve, sleeve tops z {tz1:g} ({tz1-ZBAY_TOP:.2f} mm over the board, under-carrier limit {UNDER_CARRIER_Z-1.5-ZBAY_TOP:.2f}): the wires leave upward. '
                 f'Inside B3\'s lay-flat zone for {r} (the factory housing would end at y {SOCKET[r]["b3_far_y"]:.2f}). Plugged on the bench before the board goes in (PB25).')
    else:
        add(f'bay_plug_{r}',f'{r} mated plug ({c["family"]})',box((x0+x1)/2,(y0+y1)/2,z0,x1-x0,y1-y0,z1-z0),'connector',
            PLUG_BOM.get(c['family'],'C06' if len(c['pins_xy'])==5 else 'C08'),'lower',check=True,allow=('bay_board',),
            note=(f'Mated {c["family"]} receptacle on {r}\'s right-angle header, out past the board edge: x {x0:.2f}..{x1:.2f}, y {y0:.2f}..{y1:.2f}, z {z0:g}–{z1:g} (the receptacle and its crimp ends, {c["offboard_reach"]:g} mm past the header face); '
                  f'the lead leaves its far end ({c["entry"]} in board terms) and bends up ({c["offboard_bend"]:g} mm) into the W04 corridor. Verify the purchased housing.') if c['offboard_reach'] else
                 f'Mated {c["family"]} plug envelope on {r} (z {z0:g}–{z1:g}; the cable leaves {c["entry"]} in board terms, after a {c["bend"]:g} mm wire bend over the plug). Verify the purchased housing.')
PICO_PLUG=(11,18,7)   # micro-B plug overmold (x, y, z) out of the receptacle along +y, centred on the receptacle
add('pico_usb_plug','Pico micro-B cable plug (mated)',box(USB_FACE[0],USB_FACE[1]+PICO_PLUG[1]/2,USB_FACE[2]-PICO_PLUG[2]/2,*PICO_PLUG),'black','C02','lower',allow=('bay_pico',),
    note=f'Plugs along -y into the Pico micro-USB at the board\'s left edge, in the free space x {USB_FACE[0]-PICO_PLUG[0]/2:.1f}..{USB_FACE[0]+PICO_PLUG[0]/2:.1f}, y {USB_FACE[1]:.2f}..{USB_FACE[1]+PICO_PLUG[1]:.2f} '
         '(the old strip end), pushed in from above with the lid off (validation pico_usb_insert_path_clear). The W16 cable leaves along +y.')
for i,(x,y) in enumerate(BAY_MOUNTS):
    add(f'bay_spacer_{i}','Bay board 2.4 mm printed PETG spacer (Ø7 / Ø3.4)',hole(cyl(x,y,33,3.5,ZBAY-33),x,y,32,1.7,ZBAY-31),'print','F01','lower',check=True,printable=True,
        allow=('bay_board','P_base',f'insert_bay_{i}',f'bay_screw_{i}'),note='Printed flat in PETG, 100 % infill (it carries the screw clamp load); sits on the insert, under the board\'s M3 hole.')
    tfa(f'bay_screw_{i}',x,y,ZBAY_TOP,ZBAY_TOP-8,allow=('P_base',f'bay_spacer_{i}','bay_board',f'insert_bay_{i}'),name='Bay board M3 × 8 pan-head machine screw (through the board and its spacer into the insert; tip flush with the base underside)',bom='H13')
for x,y in BAY_CLAMPS:   # under the rear Jetson standoff stacks: P_base → clamp spacer → board → 20 mm standoff
    ji=min(range(len(JHOLES)),key=lambda k:math.dist(JHOLES[k],(x,y)));assert math.dist(JHOLES[ji],(x,y))<1e-6,'BAY_CLAMPS must sit on the rear Jetson holes'
    add(f'bay_clamp_spacer_{ji}',f'Bay board 2.4 mm printed PETG clamp spacer under the rear Jetson standoff {ji} (Ø7 / Ø2.9)',hole(cyl(x,y,33,3.5,ZBAY-33),x,y,32,1.45,ZBAY-31),'print','F01','lower',check=True,printable=True,
        allow=('bay_board','P_base',f'jetson_screw_{ji}'),note='Printed flat in PETG, 100 % infill: the board is clamped between it and the 20 mm F-F standoff by the M2.5 × 12 countersunk screw from under P_base.')
for k,((x0,x1,y0,y1),pk) in enumerate(zip(BAY_RIBS,RIB_POCKETS)):
    under=[r for r,(a0,a1,b0,b1) in BAY_REAR_ROW.items() if a0<x1 and x0<a1 and b0<y1 and y0<b1]
    rib=union(box((x0+x1)/2,(y0+y1)/2,33,x1-x0,y1-y0,ZBAY-33),*[box((x0+x1)/2,(a_+b_)/2,33-RIB_POCKET,x1-x0,b_-a_,RIB_POCKET+.01) for a_,b_ in pk])
    _plen=sum(b_-a_ for a_,b_ in pk)
    add(f'bay_rib_{k}',f'Bay board bearing rib {k} under the rear connector row ({", ".join(under)}; printed PETG)',rib,'print','F01','lower',check=True,printable=True,allow=('P_base','bay_board'),
        note=f'{x1-x0:.2f} × {y1-y0:.1f} mm bar along y (x {x0:g}..{x1:g}), {ZBAY-33:g} mm tall over the base top plus a {RIB_POCKET} mm key into its locating pocket in the P_base top ({RIB_CLR} mm a side) '
             f'over {_plen:.1f} of its {y1-y0:.1f} mm'+('' if _plen>=y1-y0-1e-6 else ' (elsewhere it stands on the plain base top, clear of the caster insert under it)')+
             f': it carries the board when the rear-edge connectors are plugged and unplugged. Bonded in its pocket with epoxy (assembly step 1, cure before the board goes on). >= {RIB_GAP} mm in plan from every THT pad of the real board '
             f"(leads trimmed to <= {LEAD_TRIM} mm; validation bay_board.rear_row_ribs). Print it on its long side ('+x down' or '-x down' in cad/printability.json: its largest flat face, so the key profile prints in the layer plane); check its {ZBAY-33:g} mm height with calipers.")
# Motor pigtails, inline joints and factory cables (ruling PB25 amended; B9 carry items 2, 3, 8-11, fix round 1). The factory motor cable is
# permanent at the motor and J4 / J5 cannot be unplugged with the motors fitted (B8 review I2), so each of J4 / J5 carries a short pigtail (BOM C18),
# plugged in on the bench before the board goes in; it stays with the board. The motor's factory female housing mates with the pigtail's male end at
# an inline joint (MJ) resting on P_base just ahead of the motor, reachable from above with the Jetson off. To take the board out, the joints are
# unplugged first (PB25 amended), so a pigtail rides up with the board and needs no slack.
#  - Pigtail: the board end is the right-angle socket on J4 / J5 (RA_SOCKET, SOCKET) with its six wires leaving its tails upward. Six 28 AWG
#    silicone wires (Ø1.0), colour-coded like the factory cable, rise out of the sleeves, turn toward the front (+x) and run as a Ø3 bundle at
#    z PG['run_z'] beside U5A / J9 (|y| 30.5, inboard of the factory leads' flat row on the motor face), step out to |y| 33 before the P03 corner
#    (x 9, |y| <= 31), pass between the P03 and the motor's inner end face, and turn down into the joint's male housing. Every corner is a fillet
#    of PG['r_fillet'] (centreline; the innermost wire bends at r_fillet - (r_run - r_wire) >= 3 x the wire OD). Finished length of a wire on the bundle
#    axis = the drawn route (sleeve top to the male housing's rear face) + the terminations drawn inside the socket's sleeve block and the housing
#    (PG['term']); the six sleeves span PG['pin_spread_mm'] along the header, so the outer wires are +-spread/2 off that axis (the rear pin's wire the
#    longest). Fix round 2: the wires are PRE-CUT to route + terminations + spread/2 (rounded up to the mm), the socket end soldered first, the bundle
#    dressed along its route and each wire then trimmed to its own length and crimped. Per side (J4's socket sits 4.3 mm further aft than J5's).
#  - Factory cable (200 mm): from the encoder exit on the motor's inner end face (board_spec MOTOR_CABLE_EXIT, z 43) as a flat row of leads along
#    the face toward -x (outboard of the pigtail, never over it: the pigtail's lift path stays clear), round the motor's -x shoulder, forward (+y)
#    beside it under the carrier, up outside the carrier's footprint (|y| > 50), along +x over the motor top at |y| MC['y_top'], down the motor's
#    front shoulder (x 11) under the bumper lead, into a flat
#    coil on the base ahead of the motor (MCOIL: the spare length, held by its tie) and out of the coil into the female housing's rear end. Corners
#    are fillets of MC['r_fillet']; the route uses the cable's length (no loose loop), checked in validation motor_joints.
def _polylen(pts):return sum(math.dist(a_,b_) for a_,b_ in zip(pts,pts[1:]))   # (not _pl: a later line rebinds that name)
# Pin spread (B9 fix round 2): the pigtail is drawn as one Ø3 bundle from the pin-row centre, but its six sleeves sit on the header's pins along x, so the outer
# wires are +-spread/2 off that centre line (the bundle runs toward +x: the rear pin's wire is the longest). Taken from board_cad's pad positions.
PIN_SPREAD={r:round(max(q[0] for q in BCONN[r]['pins_xy'])-min(q[0] for q in BCONN[r]['pins_xy']),2) for r in ('J4','J5')}
assert all(abs(PIN_SPREAD[r]-2.54*(len(BCONN[r]['pins_xy'])-1))<.01 and len({round(q[1],3) for q in BCONN[r]['pins_xy']})==1 for r in PIN_SPREAD) and len(set(PIN_SPREAD.values()))==1,PIN_SPREAD
PG=dict(r_wire=.5,r_run=1.5,r_fillet=4.5,od=1.0,run_y=30.5,step_y=33.0,run_z=49.3,pin_spread_mm=PIN_SPREAD['J4'],
        term=dict(solder_overlap_on_tail=RA_SOCKET['solder_overlap'],inside_sleeve_past_tail=RA_SOCKET['sleeve_past_tail'],inside_male_housing=4.0),
        wire='six 28 AWG silicone wires (Ø1.0 mm), colour-coded red, black, green, blue, yellow, white = pins 1-6 (pins 1-2, the motor pair: 26 AWG thin-wall PTFE / ETFE, OD <= 1.0 mm, preferred if available)')
MC=dict(r=2.0,r_face=1.0,r_fillet=4.5,y_top=52.6,z_top=54.8,length=200.0)   # r_face: the leads' flat row against the motor face (B3: 6 x 2 mm)
MC_R=MC['r']
def fillet_path(pts,R,step=30.):
    """A polyline with each interior corner replaced by a circular arc tangent to both legs (points every `step` deg). The arc radius is R, or
    the largest that fits when a leg is short (each interior leg is shared by two corners: half of it is available to each). Returns (points,
    [radius used at each corner])."""
    P=[np.array(p,float) for p in pts];out=[P[0]];radii=[]
    for k in range(1,len(P)-1):
        a=P[k]-P[k-1];b=P[k+1]-P[k];la,lb=float(np.linalg.norm(a)),float(np.linalg.norm(b));a=a/la;b=b/lb
        c=float(np.clip(a@b,-1,1));th=math.acos(c)
        if th<math.radians(1):out.append(P[k]);continue
        avail=min(la if k==1 else la/2,lb if k==len(P)-2 else lb/2)
        r=min(R,avail/math.tan(th/2));t=r*math.tan(th/2)
        S=P[k]-a*t;n=b-a*c;n=n/np.linalg.norm(n);C=S+n*r
        m=max(2,int(math.ceil(math.degrees(th)/step)))
        out+=[q for q in (C-n*r*math.cos(th*j/m)+a*r*math.sin(th*j/m) for j in range(m+1)) if np.linalg.norm(q-out[-1])>1e-6];radii.append(round(r,3))
    if np.linalg.norm(P[-1]-out[-1])>1e-6:out.append(P[-1])
    return [tuple(round(float(v),3) for v in q) for q in out],radii
def helix(c,r,z0,turns,a0,pitch,cw=True,step=30.):   # points of a flat coil: centre c, radius r, from azimuth a0 (deg) at z0, dropping pitch per turn
    n=int(round(turns*360/step));sgn=-1 if cw else 1
    return [(round(c[0]+r*math.cos(math.radians(a0+sgn*step*k)),3),round(c[1]+r*math.sin(math.radians(a0+sgn*step*k)),3),round(z0-pitch*step*k/360,3)) for k in range(n+1)]
MOTOR_CABLE,PIGTAIL,JOINT,COIL={},{},{},{}
for side,sg,ref,motor in (('left',1,'J4','motor_left'),('right',-1,'J5','motor_right')):
    so=SOCKET[ref];pcx=so['pc_x'];Y=lambda y:round(sg*y,3)
    x2=round(pcx+8.1,3)   # where the wires, turned over toward +x at the run height, reach the run's |y| (left: beside U5A; right: beside J9's plug)
    corners=[(pcx,so['tail_y'],so['top_z']),(pcx,so['tail_y'],PG['run_z']),(x2,Y(PG['run_y']),PG['run_z']),(4.5,Y(PG['run_y']),PG['run_z']),
             (8.0,Y(PG['step_y']),PG['run_z']),(MJ['x'],Y(PG['step_y']),PG['run_z']),(MJ['x'],Y(MJ['y0']),MJ['z'])]
    pts,rad=fillet_path(corners,PG['r_fillet'])
    nl=max(k for k,p in enumerate(pts) if p[0]<=max(q[0] for q in BAY_OUTLINE))+1   # the points over the board ride on it when it is lifted
    PIGTAIL[side]=dict(ref=ref,corners=corners,points=pts,fillet_radii=rad,lift_points=nl,route_exact=round(sum(math.dist(a_,b_) for a_,b_ in zip(pts,pts[1:])),2))
    wire('motor_pigtail_'+side,f'Motor pigtail {side}: {ref} right-angle socket (wires up) to the inline joint\'s male housing (PB25, B9)',pts,PG['r_run'],'signal','C18',
         note=f'{PG["wire"]}: soldered to the tails of the right-angle socket on {ref} (plugged on the bench, stays with the board), leaving upward; they turn over at >= {min(rad):.2f} mm (centreline), '
              f'run beside {"U5A" if sg>0 else "J9"} and between the P03 and the motor face, and end in the male housing at the inline joint.')
    cables[-1]['never_end']=['p03_regulator',motor]   # passed close by, never a termination: the generic wire check keeps WIRE_GAP to them
    # the exit: the socket with its tails and sleeves, the wires' rise and their first turn over (to the end of the first fillet arc)
    k1=next(k for k,p_ in enumerate(pts) if k>1 and abs(p_[2]-pts[k-1][2])<1e-6 and p_[2]>=PG['run_z']-1e-6)
    _ex=cq.Compound.makeCompound([cq.Solid.makeCylinder(PG['r_run'],math.dist(a_,b_),cq.Vector(*a_),(cq.Vector(*b_)-cq.Vector(*a_)).normalized()) for a_,b_ in zip(pts[:k1],pts[1:k1+1]) if math.dist(a_,b_)>.01])
    PIGTAIL[side].update(exit_points=k1+1,exit_to_motor_mm=round(min(_ex.distance(shapes[motor]),shapes[f'bay_plug_{ref}'].distance(shapes[motor])),2))
    _route=cables[-1]['route_mm'];PIGTAIL[side]['route']=_route
    jm=box(MJ['x'],sg*(MJ['y0']+MJ['male_l']/2),MJ['z0'],MJ['t'],MJ['male_l'],MJ['w'])
    jf=box(MJ['x'],sg*(MJ['y0']+MJ['male_l']+MJ['female_l']/2),MJ['z0'],MJ['t'],MJ['female_l'],MJ['w'])
    JOINT[side]=dict(male=jm,female=jf,axis=(0,sg,0),rear_y=sg*(MJ['y0']+MJ['male_l']+MJ['female_l']))
    add(f'motor_joint_{side}',f'Motor inline joint {side}: pigtail male housing + factory female housing (PB25)',cq.Compound.makeCompound([jm,jf]),'connector','C18/E05','lower',check=True,
        note=f'0.1 in 6-way inline joint resting on P_base on edge (the six contacts in a vertical column), axis along {"+" if sg>0 else "-"}y: the pigtail\'s male housing y {sg*MJ["y0"]:g}..{sg*(MJ["y0"]+MJ["male_l"]):g}, the motor cable\'s factory female '
             f'housing to {sg*(MJ["y0"]+MJ["male_l"]+MJ["female_l"]):g}; x {MJ["x"]-MJ["t"]/2:g}..{MJ["x"]+MJ["t"]/2:g}, z {MJ["z0"]:g}..{MJ["z0"]+MJ["w"]:.2f}. Polarity: unkeyed, so pin 1 is marked on both halves (paint-pen stripe at the red wire, '
             f'pigtail and factory housing) and the colours run straight across. Held down by its cable tie (motor_tie_joint_{side}); cut the tie, then unplug the female housing along the axis '
             f'(>= {MJ["unplug"]:g} mm clear, validation motor_joints_serviceable); reachable from above with the Jetson off.')
    # the joint's cable tie: a loop in the plane y = +-tie_y round the female housing, down the motor opening (inner leg) and through a P_base slot (outer leg)
    (ix0,ix1),(ox0,ox1),(tz0,tz1)=MOTOR_TIES['joint'];yt=sg*MJ['tie_y'];zu=TIE['z_under']
    tj=union(box((ix0+ix1)/2,yt,zu,ix1-ix0,TIE['w'],tz1-zu),box((ox0+ox1)/2,yt,zu,ox1-ox0,TIE['w'],tz1-zu),box((ix0+ox1)/2,yt,tz0,ox1-ix0,TIE['w'],tz1-tz0),box((ix0+ox1)/2,yt,zu,ox1-ix0,TIE['w'],TIE['t']),
             box(ox1+TIE['head'][0]/2,yt,TIE['head_z'],TIE['head'][0],TIE['head'][1],TIE['head'][2]))   # + the tie head on the outer leg's outside face
    add(f'motor_tie_joint_{side}',f'Cable tie holding the {side} motor inline joint down (through the motor opening and a P_base slot)',tj,'black','C12','lower',check=True,
        note=f'2.5 mm cable tie (C12) round the joint\'s female housing at y {yt:g}: down the motor opening beside the motor can (x {ix0:.1f}..{ix1:.1f}), under P_base, up through the slot at x {ox0-TIE["slot_clr"]:.1f}..{ox1+TIE["slot_clr"]:.1f}; '
             'pull it snug so the joint sits on the base (retention, B9 carry item 10). Cut and replace it to unplug the joint.')
    # factory cable: exit -> a flat row along the face toward -x -> round the -x shoulder -> forward under the carrier -> up outside its footprint ->
    # over the motor top -> the coil (spare length) -> the female housing's rear end
    ex=tuple(BC['motor_cables'][ref]['exit_xyz']);cx,cy=MCOIL['c'];cyc=sg*cy
    coil=helix((cx,cyc),MCOIL['r'],MCOIL['z_top'],MCOIL['turns'],MCOIL['start_deg'],MCOIL['pitch'],cw=(sg>0))   # the right coil mirrors the left one
    face,rf=fillet_path([ex,(-3.0,Y(34.0),44.0),(-9.0,Y(34.0),45.5)],MC['r_fillet'])   # the leads leave the encoder cap as a flat row along the face
    fc=[(-9.0,Y(34.0),45.5),(-12.8,Y(36.5),47.5),(-12.8,Y(50.5),51.0),(-10.5,Y(MC['y_top']),MC['z_top']),(11.0,Y(MC['y_top']),MC['z_top']),(11.0,Y(58.0),49.5),(11.0,Y(63.0),49.5)]   # down the motor's front shoulder under the bumper lead
    head,rh=fillet_path(fc+[coil[0],coil[1]],MC['r_fillet'])
    tail,rt=fillet_path([coil[-2],coil[-1],(MJ['x'],coil[-1][1],coil[-1][2]+2.0),(MJ['x'],JOINT[side]['rear_y'],MJ['z'])],MC['r_fillet'])
    MOTOR_CABLE[side]=face+head[1:]+coil[2:-2]+tail
    MC_RADII=[MC['r_face']]*(len(face)-1)+[MC['r']]*(len(MOTOR_CABLE[side])-len(face))
    COIL[side]=dict(points=coil,centre=(cx,cyc),r=MCOIL['r'],turns=MCOIL['turns'],fillet_radii=rf+rh+rt,face_points=len(face))
    wire('motor_cable_'+side,f'Factory 200 mm motor/encoder cable {side} (to the inline joint; spare length coiled, PB25 / B9)',MOTOR_CABLE[side],MC_RADII,'signal','E05',
         note=f'The motor\'s permanent 6-wire cable: a flat row along its inner end face toward the rear, round the motor\'s rear shoulder, forward beside it, over the motor top at |y| {MC["y_top"]:g}, {MCOIL["turns"]:g} turns of a Ø{2*MCOIL["r"]:g} mm coil (centreline) on the base at ({cx:g}, {cyc:g}) held by its tie, '
              'then into the inline joint\'s female housing. Not cut, not re-terminated.')
    (ix0,ix1),(ox0,ox1),(tz0,tz1)=MOTOR_TIES['coil']
    tc=union(box((ix0+ix1)/2,cyc,zu,ix1-ix0,TIE['w'],tz1-zu),box((ox0+ox1)/2,cyc,zu,ox1-ox0,TIE['w'],tz1-zu),box((ix0+ox1)/2,cyc,tz0,ox1-ix0,TIE['w'],tz1-tz0),box((ix0+ox1)/2,cyc,zu,ox1-ix0,TIE['w'],TIE['t']),
             box(ox1+TIE['head'][0]/2,cyc,TIE['head_z'],TIE['head'][0],TIE['head'][1],TIE['head'][2]))
    add(f'motor_tie_coil_{side}',f'Cable tie holding the {side} factory-cable coil down (two P_base slots)',tc,'black','C12','lower',check=True,
        note=f'2.5 mm cable tie (C12) over the coil\'s +x side at y {cyc:g}, through two P_base slots (x {ix0-TIE["slot_clr"]:.1f}..{ix1+TIE["slot_clr"]:.1f} and {ox0-TIE["slot_clr"]:.1f}..{ox1+TIE["slot_clr"]:.1f}): the slack store (B9 carry item 3).')
PG['term_mm']=round(sum(PG['term'].values()),2)
# B9 fix round 2 (controller ruling): the pigtail wires are PRE-CUT long, the socket end soldered first, the bundle dressed along its route, then each
# wire trimmed to its own length and crimped. B12a (final review N4): the wires fan out +-spread / 2 off the drawn bundle axis at BOTH ends - the
# socket's six sleeves along the header and the male housing's six cavities - so the pre-cut is route + terminations + the full pin spread (rounded up
# to the mm), and each wire is trimmed at its own cavity: its finished length lies within route + terminations +- the spread.
for s_ in PIGTAIL:
    pg_=PIGTAIL[s_];base_=pg_['route_exact']+PG['term_mm']
    pg_['finished_centre_mm']=round(base_,1)
    pg_['finished_range_mm']=[round(base_-PG['pin_spread_mm'],1),round(base_+PG['pin_spread_mm'],1)]
    pg_['precut_mm']=int(math.ceil(base_+PG['pin_spread_mm']-1e-9));pg_['precut_margin_mm']=round(pg_['precut_mm']-base_,2)
    assert PG['pin_spread_mm']-1e-9<=pg_['precut_margin_mm']<PG['pin_spread_mm']+1.0 and pg_['finished_range_mm'][1]<=pg_['precut_mm'],pg_
    c_=next(c for c in cables if c['id']==f'motor_pigtail_{s_}')
    c_.update(suggested_cut_mm=pg_['precut_mm'],pigtail=dict(part='C18 motor pigtail (B9)',board_end_part=RA_SOCKET['part'],
        joint_end_part='Pololu 1905 1x6 0.1 in crimp housing with six male crimp pins (or a 1x6 straight male header with the wires soldered on)',wire=PG['wire'],
        route_mm=pg_['route_exact'],terminations_mm=PG['term'],pin_spread_mm=PG['pin_spread_mm'],precut_mm=pg_['precut_mm'],precut_margin_mm=pg_['precut_margin_mm'],
        finished_centre_mm=pg_['finished_centre_mm'],finished_range_mm=pg_['finished_range_mm'],trim='at its own cavity of the male housing, after dressing',length_source='crimped',
        length_note=f'pre-cut length {pg_["precut_mm"]} mm per wire = the drawn route {pg_["route_exact"]:g} mm + {PG["term_mm"]:g} mm of terminations ({PG["term"]["solder_overlap_on_tail"]:g} lapped on the tail, {PG["term"]["inside_sleeve_past_tail"]:g} in the sleeve past the tip, '
                    f'{PG["term"]["inside_male_housing"]:g} in the male crimp housing) + the full {PG["pin_spread_mm"]:g} mm pin spread, rounded up to the mm: the six wires fan out {PG["pin_spread_mm"]/2:g} mm either side of the drawn bundle axis at both ends (the socket\'s six sleeves and the male housing\'s six cavities each span {PG["pin_spread_mm"]:g} mm). '
                    f'Build: cut bulk 28 AWG silicone to the pre-cut length, solder the socket end first, dress the bundle along its route, then trim each wire at its own cavity of the male housing (finished length about {pg_["finished_centre_mm"]:g} mm on the bundle axis, '
                    f'{pg_["finished_range_mm"][0]:g}-{pg_["finished_range_mm"][1]:g} mm by where the wire sits at the two ends) and crimp the male pin. '
                    'No slack: the joint is unplugged before the board is lifted (PB25 amended). Tag the pigtail "'+('L / J4' if s_=='left' else 'R / J5')+'" right after making it (the two differ in length and pin-1 order).',
        board_end=f'right-angle socket on {pg_["ref"]} (bench, before the board goes in), wires up',male_end=f'inline joint motor_joint_{s_}'))
# Factory cable: the fixed path (everything outside the coil) + the coil (the measured variable). Pololu's "200 mm" is to be measured to the female
# housing's rear face (the verify list asks); readings FAC_READINGS must all fit the coil's FAC_TURNS.
# B12a (final review N4): a cable longer than the 200 mm reading still fits - the coil takes up to 3.5 turns (FAC_TURNS; MCOIL z_top 49 keeps the 3.5th
# turn on the base top) - up to FAC_MAX_READING.
FAC_READINGS,FAC_TURNS,BASE_TOP=(186.0,200.0),(2.0,3.5),33.0
FAC={}
for s_ in COIL:
    fr_=_polylen(MOTOR_CABLE[s_]);cl_=_polylen(COIL[s_]['points']);fx_=round(fr_-cl_,1);tl_=cl_/MCOIL['turns']
    FAC[s_]=dict(route=fr_,coil_len=cl_,fixed=fx_,turn=tl_,turns_needed=[round((r_-fx_)/tl_,2) for r_ in FAC_READINGS],remainder=[round(r_-fx_,1) for r_ in FAC_READINGS],
                 bottom_z_at_max_turns=round(MCOIL['z_top']-MCOIL['pitch']*FAC_TURNS[1]-MC['r'],2))
    assert FAC_TURNS[0]<=FAC[s_]['turns_needed'][0] and FAC[s_]['turns_needed'][1]<=FAC_TURNS[1] and FAC_READINGS[0]<=fr_<=FAC_READINGS[1] and FAC[s_]['bottom_z_at_max_turns']>=BASE_TOP,FAC[s_]
    m_=next(c for c in cables if c['id']==f'motor_cable_{s_}');m_.update(suggested_cut_mm=int(MC['length']),factory=dict(length_mm=MC['length'],cut=False,coil=dict(centre_mm=list(COIL[s_]['centre']),r_mm=MCOIL['r'],turns=MCOIL['turns']),
        fixed_path_mm=fx_,cable_readings_mm=list(FAC_READINGS),coil_turns_allowed=list(FAC_TURNS),coil_turns_needed=FAC[s_]['turns_needed']),
        notes=f'Factory cable, not cut: measure it from the encoder cap to the female housing\'s rear face (readings of {FAC_READINGS[0]:g}-{FAC_READINGS[1]:g} mm all fit), route the fixed path first (about {fx_:.0f} mm outside the coil), '
              f'wind the remainder ({FAC[s_]["remainder"][0]:.0f}-{FAC[s_]["remainder"][1]:.0f} mm, {FAC_TURNS[0]:g}-{FAC_TURNS[1]:g} turns) into the coil ahead of the motor and tie it (assembly step 4); loosen the coil tie before unplugging the inline joint.')
assert f'{FAC["left"]["fixed"]:.0f}'==f'{FAC["right"]["fixed"]:.0f}' and FAC["left"]["remainder"]==FAC["right"]["remainder"],FAC   # the two sides are mirror images
FAC_MAX_READING=math.floor(FAC['left']['fixed']+FAC_TURNS[1]*FAC['left']['turn'])   # the longest reading the 3.5-turn coil takes
def _fin(s_):
    lo,hi=PIGTAIL[s_]['finished_range_mm'];return f'{lo:g}–{hi:g} mm'
# the assembly order is written before the pigtails and the factory cable are routed, so its numbers are tokens, resolved here (manifest notes already made included)
ASSY_TOKENS={'@@PRECUT_J4@@':str(PIGTAIL['left']['precut_mm']),'@@PRECUT_J5@@':str(PIGTAIL['right']['precut_mm']),'@@SPREAD@@':f'{PG["pin_spread_mm"]:g}','@@HALF@@':f'{PG["pin_spread_mm"]/2:g}',
    '@@FIN_J4@@':_fin('left'),'@@FIN_J5@@':_fin('right'),'@@R_LO@@':f'{FAC_READINGS[0]:g}','@@R_HI@@':f'{FAC_READINGS[1]:g}','@@FIXED@@':f'{FAC["left"]["fixed"]:.0f}',
    '@@REM_LO@@':f'{FAC["left"]["remainder"][0]:.0f}','@@REM_HI@@':f'{FAC["left"]["remainder"][1]:.0f}','@@R_MAX@@':f'{FAC_MAX_READING:g}'}
def assy_resolve(t):
    for k_,v_ in ASSY_TOKENS.items():t=t.replace(k_,v_)
    return t
ASSY_ORDER=assy_resolve(ASSY_ORDER)
for p_ in parts:p_['note']=assy_resolve(p_['note'])
assert '@@' not in ASSY_ORDER and not any('@@' in p_['note'] for p_ in parts),'unresolved assembly-order token'
# P03 drawn as the real board (LIO-169): the listing's generic "5A buck, 4-38 V to 1.25-36 V, LED display" module is the HW-316E V4.0
# (XL4015, blue PCB, silk "HW-316E V4.0."; layout from the DIYmore photo set of the same board, scaled to the 62 mm length, about ±1 mm).
# Board frame: u along the long side from the IN end (0..62), v from the top edge (input / output caps, trimpot; 0..37) to the display
# edge, z up from the PCB underside. Turned into the robot as a rotation (not a mirror): the IN end at +y (the robot's left: the end
# nearer U5A / U5B), the top edge forward (x 47), the display and buttons aft. The board stands on the measured 1 mm foam tape; every
# drawn part stays inside the measured 62 × 38 × 14 mm envelope (asserted), so the checks keep the envelope (shapes['p03_regulator']).
P03_Z0=34.0;P03_PCB=(62,37,1.6)
def p03_xy(u,v):return (P03C[0]+P03S[0]/2-v,P03C[1]+P03S[1]/2-u)
def p03_detail():
    ZT=P03_Z0+P03_PCB[2];R=[];NB={}
    def B(u,v,z,du,dv,dz):return box(u,-v,z,du,dv,dz)
    def C(u,v,z,r,h):return cyl(u,-v,z,r,h)
    def put(s,c,nm=None):
        R.append((s.rotate((0,0,0),(0,0,1),-90).translate((P03C[0]+P03S[0]/2,P03C[1]+P03S[1]/2,0)),c))
        if nm:b=R[-1][0].BoundingBox();q=(b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax);o=NB.get(nm,q);NB[nm]=tuple(round(f(a,b_),2) for f,a,b_ in zip((min,)*3+(max,)*3,o,q))
    pcb=B(P03_PCB[0]/2,P03_PCB[1]/2,P03_Z0,P03_PCB[0],P03_PCB[1],P03_PCB[2])
    for u,v in ((2.9,2.9),(59.1,2.9),(2.9,34.1),(59.1,34.1)):pcb=pcb.cut(C(u,v,P03_Z0-1,1.6,4))   # 4 × Ø3.2 M3 holes
    put(pcb,'#1f4ea6')
    # screw terminals (blue, 2 × 5.08 mm, KF301 style), flush with the short ends, wire entries outward, screws on top
    ents={}
    for nm,u0,v0,out in (('IN',0.0,11.0,-1),('OUT',54.4,13.5,1)):
        body=B(u0+3.8,v0+5.08,ZT,7.6,10.16,10.0);ue=u0 if out<0 else u0+7.6
        for k in (0,1):
            vc=v0+2.54+5.08*k
            body=body.cut(B(ue,vc,ZT+1.6,8.0,3.6,3.4)).cut(C(u0+3.8,vc,ZT+6.5,1.75,4))
            put(C(u0+3.8,vc,ZT+7.0,1.55,2.2).cut(B(u0+3.8,vc,ZT+8.6,0.6,3.4,1)),'#b5bcc3',f'{nm} terminal')
            ents[f'{nm}{"+-"[k]}']=(*p03_xy(ue,vc),ZT+3.3)
        put(body,'#2f74d0',f'{nm} terminal')
    # electrolytic V-chip caps (100 uF 50 V in, 220 uF 35 V out): black base, aluminium can with a vent cross
    for u,v in ((13.0,7.0),(51.0,7.0)):
        cn='input cap' if u<31 else 'output cap'
        put(B(u,v,ZT,8.3,8.3,1.5),'#222528',cn)
        put(C(u,v,ZT+0.5,4.0,10.0).cut(B(u,v,ZT+10.2,6,0.5,1)).cut(B(u,v,ZT+10.2,0.5,6,1)),'#c6cbd0',cn)
    # XL4015 (TO-263-5) and its clip-on 4-fin aluminium heatsink (in the kit; fitted, as in the seller's photo): the tallest part
    put(B(24.3,15.0,ZT,10.2,9.0,4.4),'#1b1b1d');put(B(24.3,20.15,ZT,8.0,1.3,0.25),'#b5bcc3')
    for k in range(5):put(B(24.3+(k-2)*1.7,9.0,ZT,0.8,3.0,0.6),'#b5bcc3')
    hs=B(24.3,15.0,ZT+4.4,12.0,11.0,1.4)
    for du in (-4.55,-1.52,1.52,4.55):hs=hs.fuse(B(24.3+du,15.0,ZT+5.8,1.1,11.0,P03_Z0+P03S[2]-ZT-5.8))
    put(hs,'#d4d8dc','XL4015 heatsink')
    # toroid inductor lying flat: yellow core, copper winding (22 turns, a 40 deg gap where the leads leave)
    tu,tv=37.8,14.5
    put(C(tu,tv,ZT+0.9,6.4,5.4).cut(C(tu,tv,ZT,3.6,8)),'#d9b23c','toroid')
    for k in range(22):
        a=-160+320*k/21;turn=box(5.0,0,ZT,4.6,0.9,7.2).cut(box(5.0,0,ZT+0.9,2.8,2,5.4))
        put(turn.rotate((0,0,0),(0,0,1),a).translate((tu,-tv,0)),'#c27a34','toroid')
    # Schottky SS54 (SMA, long axis along v), the M7 diode and the TO-92 near the IN terminal, and a few 0805s
    for u,v,du,dv in ((51.7,19.4,2.6,4.3),(11.0,23.5,4.3,2.6)):
        put(B(u,v,ZT,du,dv,2.1),'#1d1d1f')
        for s_ in (-1,1):put(B(u+s_*(du/2+.3) if du>dv else u,v+s_*(dv/2+.3) if dv>du else v,ZT,1.0 if du>dv else 1.6,1.6 if du>dv else 1.0,0.25),'#b5bcc3')
    put(C(15.5,19.5,ZT+1.5,2.3,4.5).intersect(B(15.5,19.5+0.6,ZT,5,3.4,7)),'#1d1d1f')
    for k in (-1,0,1):put(B(15.5+k*1.27,19.5,ZT,0.45,0.45,1.5),'#b5bcc3')
    for u,v,c in ((16.5,13.0,'#1c1c1c'),(45.5,16.0,'#1c1c1c'),(45.5,20.5,'#b89a6a'),(47.5,23.0,'#1c1c1c'),(14.8,26.0,'#b89a6a'),(28.0,4.0,'#1c1c1c'),(21.0,4.5,'#b89a6a')):
        put(B(u,v,ZT,1.25,2.0,0.6),c)
    # 3-digit 0.36" red 7-segment display (3631AS), reading the set 5.10 V; it overhangs the display edge by 1 mm
    put(B(31.0,31.0,ZT,30.0,14.0,7.0),'#e2e2dc','display');put(B(31.0,31.0,ZT+7.0,30.0,14.0,0.2),'#151515')
    SEG={'a':(0,-4.1,4.0,.9),'b':(2.2,-2.05,.9,3.6),'c':(2.2,2.05,.9,3.6),'d':(0,4.1,4.0,.9),'e':(-2.2,2.05,.9,3.6),'f':(-2.2,-2.05,.9,3.6),'g':(0,0,4.0,.9)}
    for k,(lit,dp) in enumerate((('afgcd',True),('bc',False),('abcdef',False))):
        uc=31.0+(k-1)*9.4
        for sg,(du,dv,w,h) in SEG.items():put(B(uc+du,31.0+dv,ZT+7.2,w,h,0.12),'#ff3b2f' if sg in lit else '#2a1210')
        put(B(uc+3.4,35.3,ZT+7.2,.9,.9,0.12),'#ff3b2f' if dp else '#2a1210')
    # two 6 × 6 tact buttons (input / output voltage readout; the display blanks with the other), their LEDs and the power LED
    for u in (11.4,49.5):
        put(B(u,33.3,ZT,6,6,3.1),'#25272a','buttons');put(B(u,33.3,ZT+3.1,6,6,0.3),'#b5bcc3','buttons');put(C(u,33.3,ZT+3.4,1.75,1.6),'#1d1d1f','buttons')
    for u,v in ((11.2,27.0),(49.5,27.5),(7.5,7.0)):put(B(u,v,ZT,1.25,2.0,0.8),'#ff6a5a')
    # blue 3296W multi-turn trimpot (W503) on the top edge, its brass screw on top at the IN end
    put(B(37.75,2.9,ZT,9.5,4.8,10.0),'#2a5fc9','trimpot');put(C(34.2,2.9,ZT+10.0,1.1,0.6).cut(B(34.2,2.9,ZT+10.3,0.4,3,1)),'#c9a14a','trimpot screw')
    R.append((box(*P03C,33,P03S[0]-4,P03S[1]-4,1.0),'#ece8dc'))   # 1 mm double-sided foam tape (robot frame)
    return R,ents,NB
P03_RENDER,P03_ENT,P03_PARTS=p03_detail()
_p03env=union(box(*P03C,33,P03S[0]-4,P03S[1]-4,1.0),box(*P03C,34,*P03S))
_rb=cq.Compound.makeCompound([s for s,_ in P03_RENDER]).BoundingBox()
assert _rb.xmin>=P03C[0]-P03S[0]/2-1e-6 and _rb.xmax<=P03C[0]+P03S[0]/2+1e-6 and abs(_rb.ymin)<=P03S[1]/2+1e-6 and _rb.ymax<=P03S[1]/2+1e-6 and _rb.zmin>=33-1e-6 and _rb.zmax<=34+P03S[2]+1e-6,'P03 detail leaves its measured envelope'
add('p03_regulator',f'P03 5 V converter, HW-316E XL4015 board with LED display ({P03S[1]} × {P03S[0]} × {P03S[2]} mm measured envelope) on foam tape',_p03env,'owned','P03','lower',check=True,render=P03_RENDER,tess=(.08,.2),
    note=f'Measured envelope {P03S[1]} × {P03S[0]} × {P03S[2]} mm on 1 mm double-sided foam tape (z 33–{34+P03S[2]}), turned 90 deg: long side along y, x {P03C[0]-P03S[0]/2:g}..{P03C[0]+P03S[0]/2:g}, |y| <= {P03S[1]/2:g}, '
         'in the front bay under the front of the Jetson carrier, ahead of the bay board\'s tongue (x 7.5) and aft of the bumper riders B at full travel (validation p03_clear_of_bumper_travel). '
         'Drawn as the real board (HW-316E V4.0: XL4015 with its clip-on heatsink, toroid inductor, 100 uF / 220 uF caps, SS54, 3-digit display, two buttons, blue 3296 trimpot, blue screw terminals); the checks use the envelope. '
         'IN terminal on the left short end (+y, the end nearer U5A / U5B), OUT on the right (−y), wire entries facing out along y; caps and trimpot along the front edge, display and buttons along the aft edge. '
         f'A cable tie over it through the P_base slots at ({P03_TIES[1][0]:g}, ±{P03_TIES[1][1]:g}) holds it: a 2.5 mm tie over the two cap tops, clear of the end terminals, the trimpot and the heatsink (validation p03_model.tie). Leads to U5A/U5B: see W_p03_in / W_p03_out. Its tie and trimpot are reachable only before the Jetson goes on. '+ASSY_ORDER)

# Jetson Nano B01 from NVIDIA's STEP (simplified: carrier, module, heatsink and every connector; the many-faced connectors are
# envelopes that keep their sockets; see reference-models/SOURCE.txt). The role -> solid map comes from the sidecar JSON; the
# bounding boxes are checked so a regenerated STEP with another solid order stops the build.
_js=cq.importers.importStep(str(JDIR/'jetson-nano-b01-simplified.step')).solids().vals()
assert len(_js)==JS['solids'] and all(abs(a-b)<.02 for s,bx in zip(_js,JS['bboxes']) for a,b in zip((lambda q:(q.xmin,q.ymin,q.zmin,q.xmax,q.ymax,q.zmax))(s.BoundingBox()),bx)),'jetson-nano-b01-simplified.step does not match its JSON'
def jpart(*roles):return cq.Compound.makeCompound([_js[i].rotate((0,0,0),(0,0,1),90).translate(cq.Vector(*JO)) for r in roles for i in JS['roles'][r]])
def jbox(x,y,z,dx,dy,dz):X,Y,Z=J(x,y,z);return box(X,Y,Z,dy,dx,dz)   # box given in the STEP frame (centre x, y, bottom z, sizes), turned with the board
JNOTE=(f'NVIDIA Jetson Nano Developer Kit P3450 B01 STEP (reference-models/jetson-nano-b01), turned +90 deg about z with its origin '
       f'(mounting hole 0,0) at ({JO[0]:g}, {JO[1]:g}), PCB underside z {JO[2]:g}: I/O row on the front (+x) long edge, barrel jack on the right (-y). '
       f'Carrier x {JPCB[0]:g}..{JPCB[1]:g}, y {JPCB[2]:g}..{JPCB[3]:g}. Mounting holes Ø{JS["mounting_holes"]["d"]} on 86 × 58 mm at {", ".join(f"({x:g}, {y:g})" for x,y in JHOLES)}.')
add('jetson','Owned Jetson Nano B01 carrier and module (NVIDIA STEP)',jpart('carrier','module','sodimm','hdmi_dp','micro_usb','gpio_header','csi','fan_header','other'),'owned','E01','upper',check=True,
    tess=(.2,.3),note=JNOTE+' HDMI/DP, micro-USB, SO-DIMM, CSI, 40-pin header and the M.2 slot are envelopes from their STEP bounding boxes (sockets kept as pockets).')
add('jetson_heatsink','Jetson Nano B01 heatsink (NVIDIA STEP)',jpart('heatsink'),'metal','E01','upper',allow=('jetson',),tess=(.2,.3),
    note=f'Top z {JFAN[2]:.2f}; fan screw holes Ø{JS["heatsink"]["fan_hole_d"]} on {JS["heatsink"]["fan_pitch"][0]:g} × {JS["heatsink"]["fan_pitch"][1]:g} mm about ({JFAN[0]:g}, {JFAN[1]:g}). The four module screws enter it (allowed overlap).')
assert abs(JFAN[2]+FAN_H-ZFAN_TOP)<1e-9
fan=box(JFAN[0],JFAN[1],JFAN[2],40,40,FAN_H).cut(cyl(JFAN[0],JFAN[1],JFAN[2]-1,17,12))
fan=union(fan,cyl(JFAN[0],JFAN[1],JFAN[2],7,9))
for a in range(0,360,72):
    blade=box(9,0,JFAN[2],18,6,2).rotate((0,0,0),(0,0,1),a).translate((JFAN[0],JFAN[1],0))
    fan=union(fan,blade)
for x,y in JS['heatsink']['fan_screw_holes']:fan=hole(fan,*J(x,y)[:2],JFAN[2]-1,1.7,12)
add('fan','40 mm Jetson cooling fan',fan,'black','E09','upper',note=f'40 × 40 × 10 mm (E09) on the heatsink fan-screw pattern ({JS["heatsink"]["fan_pitch"][0]:g} mm, holes Ø{JS["heatsink"]["fan_hole_d"]} in the fins), z {JFAN[2]:.2f}–{JFAN[2]+10:.2f}.')
FSCREWS=[J(x,y)[:2] for x,y in JS['heatsink']['fan_screw_holes']]
for i,(x,y) in enumerate(FSCREWS):   # M3 × 16 pan head on the fan frame corner, 6 mm into the heatsink fin hole (self-tapping); the head is what the tray's intake duct clears
    add(f'fan_screw_{i}','Fan M3 × 16 pan-head screw (E09 kit)',union(cyl(x,y,JFAN[2]-6,1.5,16),cyl(x,y,ZFAN_TOP,FS_HEAD[0],FS_HEAD[1])),'metal','E09','upper',allow=('fan','jetson_heatsink'),
        note=f'Envelope of the usual fan screw: head Ø{2*FS_HEAD[0]:g} × {FS_HEAD[1]:g} mm on the frame (z {ZFAN_TOP:.2f}–{ZFAN_TOP+FS_HEAD[1]:.2f}); check the delivered screws, the LiDAR tray\'s intake duct clears these heads (validation jetson_cooling.fan_skirt).')
add('usb_stack_0','Jetson stacked USB-A ports (right; NVIDIA STEP envelope)',jpart('usb_a_0'),'metal','E01','upper',allow=('J17_A','J27_A','jetson'))   # mated plugs
add('usb_stack_1','Jetson stacked USB-A ports (left; NVIDIA STEP envelope)',jpart('usb_a_1'),'metal','E01','upper',allow=('J18_A','J19','jetson'))
add('ethernet','Jetson RJ45 Ethernet port (NVIDIA STEP envelope)',jpart('rj45'),'metal','E01','upper',allow=('jetson',))
add('jetson_barrel_jack','Jetson 5 V DC barrel jack (NVIDIA STEP)',jpart('barrel_jack'),'black','E01','upper',allow=('J03_barrel','jetson'),
    note=f'5.5/2.1 mm jack, bore Ø{JPORT["barrel_jack"]["bore_d"]}, axis at y {J(JPORT["barrel_jack"]["x"],0)[1]:g}, z {J(0,0,JPORT["barrel_jack"]["z"])[2]:.2f}, facing +x. Barrel-jack power needs the J48 jumper fitted.')
JSTAND=JO[2]-33   # front standoff length: P_base top to the carrier underside (24 mm stack)
JSTAND_R=round(JO[2]-ZBAY_TOP,3)   # rear standoffs (on the bay board, which they clamp): board top to the carrier underside, one 20 mm F-F
assert JSTAND_R==20.0,'the rear Jetson standoff must stay a stock 20 mm M2.5 F-F (board top z 37.0, carrier underside z 57.0)'
JREAR=[min(range(len(JHOLES)),key=lambda k:math.dist(JHOLES[k],c)) for c in BAY_CLAMPS]   # JHOLES indices of the rear (clamping) stacks: 3, 1
for i,(x,y) in enumerate(JHOLES):
    if i in JREAR:
        z0,L,sl=ZBAY_TOP,JSTAND_R,12   # base → clamp spacer → board → 20 mm F-F standoff; the M2.5 × 12 screw from below passes base, spacer and board, 4 mm into the standoff
        add('jetson_standoff_'+str(i),f'Jetson rear M2.5 × {L:g} mm nylon F-F standoff (bay board to carrier; clamps the board)',hole(cyl(x,y,z0,3,L),x,y,z0-1,1.3,L+2),'connector','H14','upper',check=True,
            allow=('bay_board','jetson',f'jetson_screw_{i}'))
        add(f'jetson_screw_{i}','Jetson rear standoff M2.5 × 12 countersunk screw from under P_base (clamps the bay board)',union(cq.Solid.makeCone(2.5,1.25,1.25,cq.Vector(x,y,29)),cyl(x,y,29,1.25,sl)),'metal','H14','base',check=True,
            allow=('P_base',f'jetson_standoff_{i}','bay_board',f'bay_clamp_spacer_{i}'),note=f'Head flush in a 90 deg countersink in the base underside (nothing protrudes under the base); through the base, the 2.4 mm clamp spacer and the board, {29+sl-ZBAY_TOP:g} mm into the standoff thread.')
    else:
        add('jetson_standoff_'+str(i),f'Jetson front M2.5 × {JSTAND:g} mm insulating standoff (P_base to carrier)',hole(cyl(x,y,33,3,JSTAND),x,y,32,1.3,JSTAND+2),'connector','H05','upper',check=True,
            allow=('P_base','jetson',f'jetson_screw_{i}'))
        add(f'jetson_screw_{i}','Jetson front standoff M2.5 × 8 countersunk screw from under P_base',union(cq.Solid.makeCone(2.5,1.25,1.25,cq.Vector(x,y,29)),cyl(x,y,29,1.25,8)),'metal','H05','base',check=True,
            allow=('P_base',f'jetson_standoff_{i}'),note='Head flush in a 90 deg countersink in the base underside (nothing protrudes under the base); 4 mm into the standoff thread.')
# Plug ends (centre of the outer face) at the real ports; the plugs point forward (+x) into the front bay.
U0,U1,BJ,FH=JPORT['usb_a_0'],JPORT['usb_a_1'],JPORT['barrel_jack'],JPORT['fan_header']
JFH_BASE=FH['base_top_z']   # fan header plastic base top (STEP frame); the mated plug sits on it
UPLUG=(15,8,28)   # USB-A overmold width, height, length out of the socket face (+1 mm inside it); 8 mm tall fits the 8.5 mm stack pitch
def usbplug(p,k,out=UPLUG[2]):return jbox(p['x'],p['face_y']+1-(out+1)/2,p['centres_z'][k]-UPLUG[1]/2,UPLUG[0],out+1,UPLUG[1])
def plug_end(p,k,out=UPLUG[2]):return J(p['x'],p['face_y']-out,p['centres_z'][k])
P_CAM,P_LIDAR,P_PICO=plug_end(U0,0),plug_end(U0,1),plug_end(U1,0)
P_BAR=J(BJ['x'],BJ['face_y']-22,BJ['z'])           # straight 5.5/2.1 plug, 22 mm out of the jack face
P_FAN=J(*FH['centre_xy'],FH['top_z'])              # fan header pin tips
# Mated USB plug bodies and their bend room are explicit geometry.
add('J17_A','Camera USB-A plug (right stack, lower port)',usbplug(U0,0),'black','C01','upper')
add('J18_A','Pico USB-A plug (left stack, lower port)',usbplug(U1,0),'black','C02','upper')
add('J19','Wi-Fi USB dongle allowance (left stack, upper port)',usbplug(U1,1,22.5),'black','E08','upper')
add('J03_barrel','Jetson DC barrel plug allowance (straight 5.5/2.1, Ø10 × 22 mm out of the jack)',cyl(*J(BJ['x'],BJ['face_y']+1,BJ['z']),5,23,'x'),'black','C03','upper')
add('J20','Fan plug on the B01 4-pin fan header (mated envelope)',jbox(*FH['centre_xy'],JFH_BASE,FH['size'][0]+.4,FH['size'][1],FH['top_z']+1-JFH_BASE),'connector','E09','upper',allow=('jetson',))

# Four downward ToF sensors, brackets and wire exits. Rear pair in the rear corners beside the battery tray, as far aft as the
# base allows (x -110, CLIFF constant: 35 mm behind the loaded rear caster contact at x -75), so a reversing robot coasting on the motor
# dynamic brake (DriveCore disarms on a hazard) from the 0.06 m/s firmware reverse limit stops before the caster reaches a drop.
for i,(x,y) in enumerate(CLIFF_IDS):
    my,oy=(24,0) if x>0 else (14.4,math.copysign(.3,y))   # rear mounts |y| 74.1..88.5: 1.1 mm to the battery tray wall (|y| <= 73); ending there keeps >= 1.5 mm of rib inside the R148 clip
    mount=box(x,y+oy,33,24,my,3).cut(box(x,y,32,13,16,6))
    mount=union(mount,box(x-9.5,y+oy,36,5,my,9),box(x+9.5,y+oy,36,5,my,9))   # 5 mm ribs: 1.3 mm around each Ø2.4 hole
    for xx in [-9.5,9.5]:mount=hole(mount,x+xx,y,32,1.2,15)
    if x<0:mount=mount.intersect(cyl(0,0,30,R_BASE,20))   # rear corners: flush with the R148 base edge; the arc leaves >= 1.7 mm of rib at its top corner
    add('P_cliff_mount_'+str(i+1),'Downward sensor mount '+str(i+1),mount,'print','F01','lower',printable=True)
    sensor=union(box(x,y,45,13,18,1.6),box(x,y,CZ,5,3,2.5))
    add('cliff_'+str(i+1),'Downward VL53L0X '+str(i+1),sensor,'pcb','S04','lower',check=True,note='Optical face ≈42.5 mm from floor; test dark/shiny flooring and validity flags.')
    add('cliff_lead_'+str(i+1),'Sensor soldered lead / strain relief',box(x,y+7,47,11,4,5),'connector','C06','lower')

# Camera image frustum: ±42 deg horizontal, 4:3 (taller than the 16:9 stream: conservative). CAMO = apex at the lens front face (camera_x):
# the perimeter rod test uses it as the visible region (the smallest view at the skin).
# CAMB = apex 3 mm further back (the assumed worst entrance-pupil position): no part may enter that larger frustum either.
THF=math.tan(math.radians(PARAMS['camera_hfov_deg']/2));TVC=THF*PARAMS['camera_check_image_px'][1]/PARAMS['camera_check_image_px'][0];th,tv=THF,TVC
CAMF=cq.Vector(PARAMS['camera_x'],0,PARAMS['camera_height']);CAMO=CAMF;CAMB=CAMF-cq.Vector(3.0,0,0)
def frust(t0,t1,o=None):
    o=o if o is not None else CAMO
    w=lambda t:cq.Wire.makePolygon([o+cq.Vector(t,sy*th*t,sz*tv*t) for sy,sz in [(1,1),(-1,1),(-1,-1),(1,-1)]],close=True)
    return cq.Solid.makeLoft([w(t0),w(t1)])
# Task F: one camera in the bumper band, fixed to the chassis (its calibration never moves with a bump), looking out through a window
# cut across the seam of the two bumper halves (the window is defined with the bumper below). Level, axis z camera_height, lens face x camera_x.
# Lens 9.2 mm long in front of the 1.6 mm OV2735 PCB (25 x 25 envelope); the USB-C plug leaves the PCB back at its right edge.
CH,CXL=PARAMS['camera_height'],PARAMS['camera_x']
XPCB=CXL-9.2-1.6                          # PCB back face (x 135.2)
# Mount: the old tower's four feet and M3 x 10 screws (the P_base holes and nuts are unchanged), two 8 x 8 mm columns and an 8 x 6 mm
# crossbar at x 100-108 above the camera (z CBZ..CBZ+6: over the USB-C plug, so the camera cable runs straight back under it). Columns,
# crossbar and feet share the flat rear face x=100: it prints on that face with no support (vertical holes get 45 deg teardrop roofs).
CBZ=CH+3.0                                # crossbar underside (z 78): 2.5 mm over the plug top, the cable passes under it
CMX=(100,108)                             # columns and crossbar x
mount=box(sum(CMX)/2,0,CBZ,CMX[1]-CMX[0],104,6)
for y in [-1,1]:
    mount=union(mount,box(sum(CMX)/2,y*47,33,CMX[1]-CMX[0],8,CBZ+6-33),box(109,y*46,33,18,26,4))
    for x,yy in TOWERS:mount=mount.cut(tear(x,y*yy,32,1.7,7))
CARMY=(9.5,15.5)                          # holder arms |y| 9.5..15.5, z CBZ..CBZ+6 (over the plug), x 108..the holder plate
for y in [-1,1]:mount=mount.cut(cyl(CMX[0]-1,y*sum(CARMY)/2,CBZ+3,1.45,CMX[1]-CMX[0]+2,'x'))   # M2.5 x 12 from the crossbar rear face into the arm pilots
add('P_camera_mount','Camera mount (chassis-fixed, in the front bay)',mount,'print','F01','camera',printable=True,
    note=f'Two 8 × 8 mm columns (z 33–{CBZ+6:g}) on the old tower feet (18 × 26 × 4 mm; four M3 × 10 screws with nuts under P_base, the same holes), an 8 × 6 mm crossbar at x {CMX[0]}–{CMX[1]}, z {CBZ:g}–{CBZ+6:g}, '
         f'two M2.5 × 12 thread-forming screws from its rear face into the camera holder arms. Print on the flat rear face (x=100), no supports. '
         'Assembly: screw the mount to P_base before the bumper halves go on (its nuts sit under P_base in the bumper-rail keep-out, as the old tower\'s did); screw the camera board onto the holder, '
         'plug the USB-C cable into it, fit the holder to the crossbar, then run the cable back under the crossbar to the Jetson\'s lower right USB port (W17) and plug it there with the Jetson\'s other plugs.')
for i,(x,y) in enumerate([(x,s*y) for s in (1,-1) for x,y in TOWERS]):   # M3 x 10 from the foot top, thin nut under P_base
    add(f'tower_screw_{i}','Camera mount M3 × 10 screw and nut',union(cyl(x,y,26.6,1.5,10.4),cyl(x,y,37,2.8,2.8),cyl(x,y,26.6,3.2,2.4)),'metal','H13/H03','camera',check=True,allow=('P_base','P_camera_mount'),note='M3 × 10 socket screw (H13) with a nut (H03, 5.5 AF, 2.4 mm) under P_base; it lies inside the bumper-rail keep-out and is checked against the moving rails and cams.')
# Holder: a 3 mm plate behind the PCB (31 x 31, a 19 x 19 window for the parts on the PCB back, a notch through its right side for the
# USB-C plug) with two arms back to the crossbar. Printed plate-down (its front face on the bed): the arms grow straight up, no support.
HPL=3.0
holder=box(XPCB-HPL/2,0,CH-15.5,HPL,31,31).cut(box(XPCB-HPL/2,0,CH-9.5,HPL+2,19,19)).cut(box(XPCB-HPL/2,-13,CH-5.5,HPL+2,11,7))
for y in [-1,1]:holder=union(holder,box((CMX[1]+XPCB-HPL)/2,y*sum(CARMY)/2,CBZ,XPCB-HPL-CMX[1],CARMY[1]-CARMY[0],6)).cut(cyl(CMX[1]-1,y*sum(CARMY)/2,CBZ+3,1.25,9,'x'))
for y,z in [(-10.5,-10.5),(10.5,-10.5),(-10.5,10.5),(10.5,10.5)]:holder=holder.cut(cyl(XPCB-HPL-1,y,CH+z,.8,HPL+2,'x'))   # M2 x 6 thread-forming pilots for the PCB corners (verify the board's hole pattern)
add('P_camera_holder','Camera holder (PCB plate and arms)',holder,'print','F01','camera',printable=True,
    note=f'3 mm plate behind the camera PCB (x {XPCB-HPL:g}–{XPCB:g}, 31 × 31 mm, 19 × 19 mm window, USB-C notch at the right edge), four M2 × 6 thread-forming screws into Ø1.6 pilots at the PCB corners (±10.5 mm: '
         f'confirm the delivered board\'s hole pattern), two 6 × 6 mm arms back to the mount crossbar (M2.5 pilots). Print plate-down, no supports. The lens face sits at x {CXL:g}, {R_SKIN-CXL:.1f} mm behind the bumper skin.')
cam=box(XPCB+.8,0,CH-12.5,1.6,25,25)
add('camera_board','Waveshare OV2735 camera PCB envelope',cam,'pcb','E04','camera',check=True,allow=('J17_C','P_camera_holder'))
add('camera_lens','Single camera lens allowance',cyl(XPCB+1.6,0,CH,7,CXL-XPCB-1.6,'x'),'lens','E04','camera')
add('J17_C','Camera USB-C plug allowance',box(XPCB-7,-12,CH-4.5,14,10,5),'black','C01','camera')
# Camera LED (task N, LIO-134): one white 2835 LED above the lens, facing forward, chassis-fixed like the camera (a bump never moves
# it). It lights the floor and the LiDAR's blind band ahead through the camera window: from its face (x 145.8, y 0, z 89.45) the window
# (side faces from x 153, half-width 13.1, square for a 1 mm land to x 154; flat roof 93.1 and sill 65.9 right through the wall, whose
# outer edges set the limits) passes ±58 deg sideways and about 62 deg down to 17 deg up; within ~18 deg of straight ahead the 7.5 mm
# seam gap also passes 84 deg down to 55 deg up (to the seam backers at x 148). Its face is 0.2 mm behind the lens face's plane and
# 9.45 mm above the axis, facing forward: its light can reach the lens front only at 79 deg or more off the lens axis (no glare path),
# and it stays outside the image frustum.
# Height: the right half's seam backer stands over the window roof (z 93.1) at x 146.5-148.3 and moves back to x 142.2 at the hard
# stop, so everything forward of x 140 stays under z 91.0 (>= 2.1 mm under it at every travel, the 2 mm rigid-part rule), and over
# the lens allowance (z 87) by 0.9 mm: a 3.1 mm band for the LED stack.
# The LED (2.8 x 3.5 x 0.8 mm) is soldered across a cut in a 10 x 3.1 mm piece of 0.8 mm copper-clad board (the two copper halves are
# its pads; the two wires solder to them beside the LED and run back along the shelf block's sides). A printed shelf holds that board:
# a 2 mm plate on the camera PCB's front, clamped under its two upper corner screws (M2 x 8 instead of x 6), and a 10 mm wide block
# forward over the lens allowance (0.9 mm above it) with the board glued to its front face. Printed on the plate's back face, no supports.
LEDF=PARAMS['camera_led_points'][0]                     # LED emitting face (x, y, z)
LEDB0=LEDF[0]-0.8-0.8                                     # carrier board back face (x 144.2): LED 0.8 mm + board 0.8 mm
LEDZ0,LEDZ1=87.9,91.0                                     # shelf block and board: z 87.9 (0.9 mm over the lens allowance) .. 91.0 (2.1 mm under the backer)
PLZ1=94.0                                                 # the plate (x <= 138.8, 3.4 mm behind the backer's hard stop) rises to z 94 round the screws
shelf=union(box(XPCB+1.6+1.0,0,LEDZ0,2.0,26,PLZ1-LEDZ0),                # plate on the PCB front (x 136.8..138.8), y -13..13
            box((XPCB+1.6+2.0+LEDB0)/2,0,LEDZ0,LEDB0-(XPCB+1.6+2.0),10,LEDZ1-LEDZ0))   # block forward to the board (x 138.8..144.2)
for y in (-10.5,10.5):shelf=shelf.cut(cyl(XPCB,y,CH+10.5,1.1,4,'x'))    # M2 clearance holes at the PCB's upper corner screws
add('P_camera_led_shelf','Camera LED shelf (on the camera PCB, over the lens)',shelf,'print','F01','camera',printable=True,check=True,allow=('camera_board','camera_led_board'),
    note=f'2 mm plate on the camera PCB front (x {XPCB+1.6:g}–{XPCB+3.6:g}, y ±13, z {LEDZ0:g}–{PLZ1:g}), clamped under the PCB\'s two upper corner screws (M2 × 8 thread-forming instead of × 6; Ø2.2 holes), '
         f'and a 10 mm wide, {LEDZ1-LEDZ0:.1f} mm high block forward over the lens allowance to x {LEDB0:g}, 0.9 mm above it and 2.1 mm under the bumper seam backer at its hard stop; the LED board is glued to its front face, its two wires run back along the block\'s sides. '
         'Print on the plate\'s back face, no supports. Check on the delivered camera that the lens holder stays under z 87 (the CAD allowance is a Ø14 cylinder).')
add('camera_led_board','Camera LED carrier (10 × 3.1 mm piece of 0.8 mm copper-clad board, copper cut into two pads)',box(LEDB0+.4,0,LEDZ0,0.8,10,LEDZ1-LEDZ0),'pcb','E20','camera',check=True,allow=('P_camera_led_shelf','camera_led'))
add('camera_led','Camera LED (Samsung LM281B+ Pro 2835, 5000 K)',box(LEDF[0]-.4,LEDF[1],LEDF[2]-1.4,0.8,3.5,2.8),'lens','E19','camera',check=True,allow=('camera_led_board',),
    note='Emitting face at x 145.8, z 89.45; 3.5 mm along y, 2.8 mm along z. Anode / cathode: see the datasheet\'s polarity mark (BRINGUP section 10).')

# Real LiDAR: LDROBOT LD14 manufacturer STEP (Waveshare D200 kit download, reference-models/LD14.step).
# STEP frame: spin axis at x=y=0, tail/motor (the LD14 0 deg direction) toward +x. Its two optical
# windows span z 8.4..16.0, so the laser plane is taken at their centre, z=12.2. Task F (user decision): turned
# lidar_yaw_deg (180) about its spin axis, motor toward the back; spin axis at (lidar_x, 0), as far forward as its 0.1 m blind
# circle stays inside the pressed bumper (validation lidar_mount.min_range); laser plane at lidar_height; LD14 0 deg along robot -x.
LX,LZ,LYAW=PARAMS['lidar_x'],PARAMS['lidar_height']-12.2,PARAMS['lidar_yaw_deg']
def ldp(s):return s.rotate((0,0,0),(0,0,1),LYAW).translate((LX,0,LZ))   # LD14 STEP frame -> robot frame
def lxy(x,y):c,s=math.cos(math.radians(LYAW)),math.sin(math.radians(LYAW));return (round(LX+c*x-s*y,6),round(s*x+c*y,6))   # STEP plan point -> robot
LDS=cq.importers.importStep(str(ROOT/'reference-models'/'LD14.step')).solids().vals()
def ld(vol):   # pick an LD14 STEP solid by its volume (mm3) and move it to the robot frame
    s=min(LDS,key=lambda s:abs(s.Volume()-vol));assert abs(s.Volume()-vol)<1,vol
    return ldp(s)
def sector(a0,a1,z0,z1,r0,r1):
    pts=[(0,0)]+[(40*math.cos(math.radians(a)),40*math.sin(math.radians(a))) for a in np.linspace(a0,a1,6)]
    return prism(pts,z0,z1-z0).intersect(cyl(0,0,z0,r1,z1-z0).cut(cyl(0,0,z0-1,r0,z1-z0+2)))
# The STEP turret is an open shell, so it is rebuilt as a solid from its measured faces: outer cone
# r(z) = 30.015 - 0.0314 z (narrows upward: r 29.92 at z 2.95, 29.41 at z 19.18), top chamfer to r 26.6
# at z 19.85, cap recess, two cover screws, windows.
tr=lambda z:30.015-.0314*z
tur=union(cq.Solid.makeCone(tr(2.95),tr(19.18),16.23,cq.Vector(0,0,2.95)),cq.Solid.makeCone(tr(19.18),26.6,.67,cq.Vector(0,0,19.18)))
tur=tur.cut(cyl(0,0,16.9,5.95,4))
for y in [-23.5,23.5]:tur=tur.cut(cyl(0,y,16.2,2.46,4))
win=[(44.1,68.6,8.48,15.93),(109.3,142.6,8.41,16.0)]   # emitter/receiver openings as found in the STEP
for w in win:tur=tur.cut(sector(*w,28.2,31))
LDNOTE=f'LDROBOT LD14 manufacturer STEP (reference-models/LD14.step), turned {LYAW:g} deg about its spin axis (motor toward the back, task F user decision): spin axis x={LX:g}, y=0; laser plane z={PARAMS["lidar_height"]:g} = STEP window-band centre z=12.2 (windows z 8.4-16.0) + {LZ:g} mm.'+' LD14 0 deg (tail/motor direction) points along robot -x. LD14 angles increase clockwise seen from above: robot bearing = wrap(180 deg - raw angle); for a driver scan already in REP-103 form (CCW, 0 = tail) the bearing is wrap(scan angle + 180 deg), wrapped into (-180, 180]. The D200 kit ships the LD14P: confirm lug positions and window height on the delivered unit.'
add('lidar','LD14 housing, lugs and gear cover (manufacturer STEP)',cq.Compound.makeCompound([ld(7932.7),ld(631.0)]),'black','E15','cover',check=True,tess=(1.0,1.0),note=LDNOTE)
add('lidar_turret','LD14 rotating turret',ldp(tur),'black','E15','cover',check=True,tess=(.15,.2),note='Outer envelope rebuilt from the STEP turret shell (that shell is open and cannot form a solid): cone r 29.92 at STEP z 2.95 narrowing to 29.41 at z 19.18, top chamfer to r 26.6 at z 19.85. Rotates: never clamp, touch or route cable near it.')
add('lidar_motor','LD14 spin motor (manufacturer STEP)',ld(5523.7),'metal','E15','cover',check=True,tess=(.5,.5))
add('lidar_cap','LD14 red top cap',ld(219.8),'red','E15','cover',tess=(.2,.3))
add('lidar_lens','LD14 emitter and receiver windows',cq.Compound.makeCompound([ldp(sector(*w,28.2,29.3)) for w in win]),'lens','E15','cover',tess=(.2,.3),note='Window positions follow the STEP snapshot; the turret spins at about 6 Hz. Keep the band z '+f'{PARAMS["lidar_height"]-4:g}-{PARAMS["lidar_height"]+4:g}'+' mm clear all round.')
add('lidar_connector','LD14 4-pin 1.25 mm side-entry socket (STEP)',ld(62.5),'connector','E15','cover')
# LiDAR tray (task L): a flat plate on the lid top whose top face is the LD14 lug seat (the lugs are the only part of the LD14 that touches it;
# no towers: the LD14 sits 20 mm lower than in task F). M3 × 3 heat-set inserts flush with the plate top take the three lug screws (M3 × 5
# socket heads from above, in the lugs' counterbores). Task F: the LD14 is turned round (motor toward the back), its front lug at x 89.3.
# Task I: the lid is closed over the front bay, so the whole plate rests on the lid.
LUGS=[lxy(-35.3,0),lxy(36,18),lxy(36,-18)]   # STEP lug holes (Ø3.24 through a 2 mm web, Ø5.96 counterbore from above) in the robot frame: front ear, two tail lugs
lug_z=LZ-8.0                        # lug undersides (the lowest points of the LD14 housing; its flat underside is 2.8 mm higher)
TRAYH=[(-35,-38),(-35,38),(83,-35),(83,35)]   # rear pair clear of the fan window and the J27 USB socket; front pair on solid lid (behind the old front notch, task F)
ZT0=ZLIDT                           # tray plate on the lid top
TRAY_T=round(lug_z-ZT0,3)           # 3.8: the plate top is the lug seat
assert TRAY_T>=3.0,TRAY_T
# Jetson cooling (L2, task L). The 40 mm fan (z ZFAN_TOP-10..ZFAN_TOP) sits under the rear of the tray, and the robot centre (0, 0), where the
# lid seams meet, lies INSIDE the fan footprint, so lid vent slots over the fan would cross the seams. The tray plate carries the grille instead
# (GR_W slots on a 6.5 mm pitch inside r 17.5 about the fan centre, over the Ø34 fan bore) and, under it, the intake duct that seals the
# fan's suction side from the warm body: a 1.3 mm wall ending SK_GAP above the fan frame, inside the fan screw heads (until task L this was
# a skirt under the lid). Task L: the LD14 motor (Ø24.4, bottom rim 1.4 mm over the fan frame) straddles the fan's front face and covers
# 2.8 mm of the bore's edge, so the duct takes it in: its outline is the r 17.5 window circle united with a circle 1.0 mm round the motor,
# and beyond the fan's front face a bowl closes the duct under the motor. The bowl's faces slope at 42 deg from vertical (the tray prints
# top face down, so they print without supports); its back wall stands SLOT_G in front of the fan's front face. The lid has one opening
# round the duct. Air path: room air over the tray -> grille and the ring round the motor -> duct -> fan. Numbers: validation jetson_cooling.
FW=(JFAN[0],JFAN[1],17.5)           # fan window (now the duct's circle about the fan centre): centre and radius (the E09 bore is r 17)
SK_T,SK_GAP=1.3,0.9                 # duct wall and its gap to the fan top (0.9: the fan bypass keeps >= 1 % margin under its 25 % limit; feeler go 0.5 / no-go 1.0)
SK_FEELER=(0.5,1.0)                 # duct-gap limits at assembly (go / no-go; task L checks them with modelling clay, the duct edge is out of a feeler's reach): >= 0.5 no contact, <= 1.0 inside the bypass limit
ZSK=ZFAN_TOP+SK_GAP                 # duct wall bottom over the fan frame
GR_W,GR_P,GR_N,GR_R=5.2,6.5,2,FW[2]   # slot width (task L: 5.2, was 4.5: 1.3 mm ribs give back the open area the motor and the rib round its hole take), pitch, slots each side of the centre one, grille radius
_mb=shapes['lidar_motor'].BoundingBox()
MOT=((_mb.xmin+_mb.xmax)/2,(_mb.ymin+_mb.ymax)/2,(_mb.xmax-_mb.xmin)/2,_mb.zmin)   # LD14 motor axis (x, y), radius, lowest point (the hub, beyond the fan's front face)
MOT_CLR,SLOT_G,DUCT_FIT,BOWL_T,BOWL_DEG,BOWL_FLOOR=1.0,0.5,0.3,1.2,42.0,6.0   # motor to every printed part; bowl back wall to the fan's front face; duct walls in the lid opening; bowl wall; bowl slope from vertical; bowl floor under its rim
XFAN=JFAN[0]+20.0                   # the fan's front face (40 mm frame)
R_MI=MOT[2]+MOT_CLR                 # duct circle round the motor (inner face)
ZBOWL=MOT[3]-MOT_CLR                # bowl rim: 1.0 mm under the motor hub
def dprism(dr,z0,h):return union(cyl(FW[0],FW[1],z0,FW[2]+dr,h),cyl(MOT[0],MOT[1],z0,R_MI+dr,h))
def halfspace_above(px,pz,tilt):   # everything above the plane through (px, *, pz) that falls toward +x at tilt deg from horizontal
    return box(0,0,0,600,600,300).rotate((0,0,0),(0,1,0),tilt).translate((px,0,pz))
_m=1/math.tan(math.radians(BOWL_DEG));_tv=BOWL_T*math.hypot(1,_m)   # drop per mm across a 42 deg face; vertical thickness of the bowl wall
_tilt=90-BOWL_DEG
X_BI=XFAN+SLOT_G-(ZBOWL-_tv-ZFAN_TOP)/_m   # inner top edge of the bowl's back wall, so its outer face clears the fan's front face by SLOT_G at the frame top
def bowl_region(dz,zf):   # air side of the bowl face (dz = 0) or of its outer face (dz = wall thickness): over the cone about the motor axis, over the back plane and over a flat floor at zf
    za=ZBOWL-dz-R_MI*_m
    return cq.Solid.makeCone(0,(ZT0-za)/_m,ZT0-za,cq.Vector(MOT[0],MOT[1],za)).intersect(halfspace_above(X_BI,ZBOWL-dz,_tilt)).intersect(box(MOT[0],MOT[1],zf,100,100,ZT0-zf))
ZBF=ZBOWL-BOWL_FLOOR                # the bowl's flat floor (a short bridge in print: the two 42 deg faces would otherwise meet in a knife-thin crease)
bowl=bowl_region(_tv,ZBF-BOWL_T).cut(bowl_region(0.,ZBF)).intersect(cyl(MOT[0],MOT[1],ZFAN_TOP-40,R_MI+SK_T,ZBOWL-ZFAN_TOP+40)).intersect(box(XFAN+SLOT_G+100,0,ZFAN_TOP-40,200,200,ZBOWL-ZFAN_TOP+40))
duct=dprism(SK_T,ZSK,ZT0-ZSK+.5).cut(dprism(0.,ZSK-1,ZT0-ZSK+3))   # the duct wall down to SK_GAP over the fan frame
duct=union(duct,dprism(SK_T,ZBOWL-.01,ZSK-ZBOWL+.02).cut(dprism(0.,ZBOWL-1,ZSK-ZBOWL+2)).intersect(box(XFAN+SLOT_G+100,0,ZBOWL-1,200,200,ZSK-ZBOWL+2)),bowl)   # beyond the fan's front face: down to the bowl rim
# Fan screw heads the duct wall would pass within 0.5 mm of in plan: a radial notch lifts the wall over the head (0.5 mm above its top).
DUCT_NOTCH=[]
for j,(hx,hy) in enumerate(FSCREWS):
    for (cx,cy,ri) in ((FW[0],FW[1],FW[2]),(MOT[0],MOT[1],R_MI)):
        d_=math.hypot(hx-cx,hy-cy);ro=ri+SK_T;rh=FS_HEAD[0]+.5
        if d_-ro>=rh or d_+rh<=ri:continue
        ph=math.degrees(math.acos(max(-1,min(1,(ro*ro+d_*d_-rh*rh)/(2*ro*d_)))))+2.0;ps=math.degrees(math.atan2(hy-cy,hx-cx))
        zt=ZFAN_TOP+FS_HEAD[1]+.5
        _a=np.linspace(-ph,ph,7)
        wedge=prism([(cx+(ri-.5)*math.cos(math.radians(ps+a)),cy+(ri-.5)*math.sin(math.radians(ps+a))) for a in _a]+[(cx+(ro+2)*math.cos(math.radians(ps+a)),cy+(ro+2)*math.sin(math.radians(ps+a))) for a in _a[::-1]],ZSK-1,zt-ZSK+1)
        duct=duct.cut(wedge);DUCT_NOTCH.append(dict(fan_screw=j,centre=(cx,cy),half_angle_deg=round(ph,2),z_mm=round(zt,2),arc_mm=round(math.radians(2*ph)*ri,2)))
TRAYF=round(max(x for x,y in LUGS)+11,1)   # tray front edge 11 mm ahead of the front lug; its rear edge stays at x -45 behind the fan
tray=rounded((TRAYF-45)/2,0,ZT0,TRAYF+45,86,TRAY_T,8)
tray=union(tray,duct)
for x,y in TRAYH:tray=hole(tray,x,y,ZT0-1,1.7,TRAY_T+2)
GRILLE=union(*[box(FW[0]+k*GR_P,FW[1],ZT0-1,GR_W,2*GR_R+2,TRAY_T+2) for k in range(-GR_N,GR_N+1)]).intersect(cyl(FW[0],FW[1],ZT0-1,GR_R,TRAY_T+2))
GR_RIB=1.5                          # rib between the grille slots and the motor's hole (a slot edge running into the hole would leave a sliver of plate)
tray=tray.cut(GRILLE.cut(cyl(MOT[0],MOT[1],ZT0-2,R_MI+GR_RIB,TRAY_T+4))).cut(cyl(MOT[0],MOT[1],ZT0-1,R_MI,TRAY_T+2))   # fan grille over the duct and the motor's way through the plate (its 1.0 mm ring is room air too)
LD14_AP=(39.0,18.0,10,8)            # LD14 lead aperture in the lid (centre x, y, size x, y): just past the J28 plug's lead exit (task L: the plug sits 1.4 mm into the tray,
                                    # so the lead drops straight down beside it); 2.9 mm inside the 37 deg lid seam, clear of its r 50 dovetail tab; the plug passes it at assembly
# The LD14's circuit-board parts hang below its lug plane (to 1.9 mm under it: a second side-entry socket, chips, standoffs), so the plate has
# one opening under them, >= MOT_CLR round every part that comes within MOT_CLR of the plate top, merged with the opening round the J28 plug and
# the grommet lip (>= 1.1 mm round the lip). The lid under it is solid (except the grommeted lead aperture): room air only.
_low=[b_ for b_ in (ldp(x_).BoundingBox() for x_ in LDS if abs(x_.Volume()-7932.7)>1 and abs(x_.Volume()-5523.7)>1)
      if b_.zmin<lug_z+MOT_CLR and math.hypot((b_.xmin+b_.xmax)/2-MOT[0],(b_.ymin+b_.ymax)/2-MOT[1])>R_MI]   # (the motor's screw heads lie in its own hole)
TRAY_LEAD=(math.floor(10*min([32.0]+[b.xmin-MOT_CLR for b in _low]))/10,math.floor(10*min([4.5]+[b.ymin-MOT_CLR for b in _low]))/10,math.ceil(10*max([46.0]+[b.xmax+MOT_CLR for b in _low]))/10,math.ceil(10*max([23.0]+[b.ymax+MOT_CLR for b in _low]))/10)   # (x0, y0, x1, y1), rounded outward
assert TRAY_LEAD[0]>MOT[0]+R_MI+3 and TRAY_LEAD[2]<LUGS[0][0]-7 and max(abs(TRAY_LEAD[1]),abs(TRAY_LEAD[3]))<43-12,TRAY_LEAD   # clear of the motor hole and the front lug, side strips >= 12 mm
tray=tray.cut(rounded((TRAY_LEAD[0]+TRAY_LEAD[2])/2,(TRAY_LEAD[1]+TRAY_LEAD[3])/2,ZT0-1,TRAY_LEAD[2]-TRAY_LEAD[0],TRAY_LEAD[3]-TRAY_LEAD[1],TRAY_T+2,3))
for x,y in LUGS:tray=tray.cut(cyl(x,y,ZT0-1,INS_R,TRAY_T+2))   # Ø4.0 bores for the lug inserts (insert flush with the plate top, 0.8 mm open below it)
add('P_lidar_tray','LD14 tray and fan intake duct on the lid',tray,'print','F01','cover',printable=True,print_flip=True,allow=LID_IDS,
    note=f'{TRAYF+45:g} × 86 × {TRAY_T:g} mm plate on the lid top (x -45–{TRAYF:g}, z {ZT0:g}–{ZT0+TRAY_T:g}; four M3 holes at (-35, ±38) and ({TRAYH[2][0]:g}, ±35), M3 × 12 + nyloc nuts under the lid). '
         f'Its flat top is the LD14 seat (task L: no towers, the LD14 sits 20 mm lower than in task F): the three lugs rest on it at z {lug_z:g}, front ({LUGS[0][0]:g}, 0) and tail ({LUGS[1][0]:g}, ±18), each on an M3 × 3 brass heat-set insert pressed flush into the plate top '
         f'(Ø4.0 bore through the plate), with an M3 × 5 socket-head screw from above through the lug\'s Ø3.24 hole (its head sits in the lug\'s Ø5.96 counterbore; reachable from above with the LD14 fitted). Only the lugs touch the tray. '
         f'Fan grille over the fan: {2*GR_N+1} × {GR_W} mm slots on a {GR_P} mm pitch inside r {GR_R:g} about the fan centre ({FW[0]:.1f}, {FW[1]:.1f}); {GR_P-GR_W:.1f} mm ribs, stopped {GR_RIB:g} mm short of the motor hole, open to the room above. '
         f'Under the plate the intake duct ({SK_T} mm wall) passes the lid opening and ends {SK_GAP:g} mm over the fan frame (z {ZSK:.2f}), round the fan window circle and 1.0 mm round the LD14 motor, which drops into it through a Ø{2*R_MI:g} hole in the plate; '
         f'beyond the fan\'s front face a bowl under the motor closes it ({BOWL_DEG:g}° faces, back wall {SLOT_G:g} mm in front of the fan). At assembly check the duct gap to the fan frame with modelling clay (BRINGUP section 10; the duct edge cannot be reached with a feeler): {SK_FEELER[0]}–{SK_FEELER[1]} mm '
         f'(thinner: sand the duct end; thicker: shim the fan or the bypass limit is lost); check that light shows between the bowl and the fan\'s front face ({SLOT_G:g} mm), and that the LD14 motor keeps >= 0.5 mm over the fan frame (else 0.5 mm washers under the LD14 lugs, not under the tray). '
         f'Opening {TRAY_LEAD[2]-TRAY_LEAD[0]:g} × {TRAY_LEAD[3]-TRAY_LEAD[1]:g} mm at x {TRAY_LEAD[0]:g}–{TRAY_LEAD[2]:g}, y {TRAY_LEAD[1]:g}–{TRAY_LEAD[3]:g} under the LD14\'s circuit board (its parts hang up to 1.9 mm below the lugs), round the J28 plug and the lead grommet; the lid under it is solid. '
         f'Print top face down (STL bed-ready): the duct and bowl rise from the plate, no supports; the bowl\'s flat floor ({BOWL_FLOOR:g} mm under its rim) is a short bridge and its back-wall top edge a 24 mm bridge between the duct walls.')
for i,(x,y) in enumerate(LUGS):
    add(f'lidar_lug_insert_{i}','M3 × 3 brass heat-set insert (LD14 lug)',cyl(x,y,lug_z-INS_L,INS_R,INS_L).cut(cyl(x,y,lug_z-INS_L-1,1.5,INS_L+2)),'metal','H12','cover',check=True,allow=('P_lidar_tray','lidar'),
        note='Pressed flush into the tray plate top with a soldering iron, the plate supported on a block under each insert (the duct and bowl hang 15 mm under the plate, so it cannot lie flat top face up); the LD14 lug sits on it and an M3 × 5 socket-head screw from above takes 3 mm of thread.')
for i,(x,y) in enumerate(TRAYH):   # M3 × 12 from the tray top, nyloc nut under the lid (z ZLID-4..ZLID)
    add(f'lidar_tray_screw_{i}','LiDAR tray M3 × 12 screw and nyloc nut',union(cyl(x,y,ZLID-5.2,1.5,12),cyl(x,y,ZT0+TRAY_T,2.8,2.8),cyl(x,y,ZLID-4,3.2,4)),'metal','H09','cover',allow=LID_IDS+('P_lidar_tray',),note='Nut and washer seat on solid lid material (outside the fan window).')
add('lidar_adapter','Included D200 USB serial adapter allowance',box(15,-40,132-DZ,35,16,7),'pcb','E15','upper',check=True,note='CP2102 adapter shape and connector orientation provisional; inspect supplied kit.')
clip=union(box(16,-40,ZLID-3,34,22,3),box(16,-50,ZLID-14,34,2,11),box(9.5,-30,ZLID-14,21,2,11))   # side rails beside the adapter; the -x end stays open for the mated USB socket
add('P_lidar_adapter_clip','USB adapter clip beneath lid',clip,'print','F01','cover',printable=True,note='Secure below lid with two ties. Verify adapter fit; keep exposed contacts insulated.')
add('J27_A','LiDAR extension USB-A plug (aft stack, upper port)',usbplug(U0,1),'black','C16','upper')
add('J27_socket','LiDAR extension USB-A female allowance (mated on the adapter plug)',box(-13,-40,ZLID-10,22,17,10),'black','C16','upper',allow=('lidar_adapter',))
add('J28','LD14 lead plug, 1.25 mm 4-pin, mated',box(*lxy(15.02,-13.3),LZ-9.4,6.8,5,3.2),'connector','E15','cover',note=f'Plugs along -y (the LD14 is turned round) into the side-entry socket under the LD14 PCB; task L: the plug sits in the tray opening (its underside {ZT0+TRAY_T-(LZ-9.4):.1f} mm under the lug seat) and its lead turns straight down through the grommet in the lid aperture at ({LD14_AP[0]:g}, {LD14_AP[1]:g}). '
    'Assembly: lead up through the lid aperture from below, grommet round it, plug it into the LD14 before the LD14 goes onto the tray, then feed the slack down through the grommet as the LD14 is set down.')

# Round front bumper: two independently floating 90 deg arc halves about (0,0), outer shell R157.4, 2.4 mm wall (WALL2 above ZW2, 41 mm under the top: interlayer
# strength for hits at the top rim) with a 6 mm 45 deg inner top rim (stiffening ring), z 18..ZB1 (2 mm under the lid; 98.5 in the low profile), full
# height all round (task F: the camera notch is gone). The first 3.5 deg from the seam keep the thin wall, the first 5.5 deg have no rim (seam backer clearance).
# The camera looks out through a window cut across the seam (WIN below): half of it in each half, so every edge of it moves with a half. 1.5 mm EVA foam to the R158.9
# skin up to az 70; az 70..90 is bare PETG with PTFE film tape (low object friction where the push along the travel is smallest),
# and az 78..90 tapers from R157.4 to R152.7, outside the start of the R152.5 rear shell (ramped down to R151.5), so a contact arriving
# from behind always meets a ramp and an object leaving the bumper end never meets a fixed face. Each seam end is a 20 deg lead-in ramp
# 2.5 mm deep through foam and shell, so an object crossing the seam meets a face that pushes the other half along its travel.
# Each half rides on a rail (y 11..23) and an outboard arm under P_base with three pins fixed in it (countersunk from below) whose
# 693ZZ bearings / sleeve run in P_base slots; the slot ends are the rest stop and the 5 mm hard stop. Vertical loads roll on Ø4 balls
# between steel races (spring-preloaded, zero play). A printed constant-strength leaf spring on the arm, perpendicular to the travel,
# bears with a Ø4 nose on a base abutment. A 40 deg ramp + dwell cam on the rail presses the SS-5GL2 roller; the switch is set at
# assembly to operate at 2.25 mm.
def dsec(pts,f):return cq.Face.makeFromWires(cq.Wire.makePolygon([cq.Vector(*f(n,z)) for n,z in pts],close=True))
def arc(sec,r,a0,a1):   # revolve an (inset from r, z) section about the z axis from azimuth a0 to a1 (deg)
    return cq.Solid.revolve(dsec(sec,lambda n,z:(r-n,0,z)),a1-a0,cq.Vector(0,0,0),cq.Vector(0,0,1)).rotate((0,0,0),(0,0,1),a0)
def ring(r0,r1,z0,z1,a0,a1):return arc([(0,z0),(r1-r0,z0),(r1-r0,z1),(0,z1)],r1,a0,a1)
def spiral(a0,a1,ro,ri,z0,z1,n=12):   # plan band whose outer/inner radii vary linearly with azimuth (ro, ri: (r at a0, r at a1))
    A=np.linspace(a0,a1,n);f=lambda r,t:r[0]+(r[1]-r[0])*(t-a0)/(a1-a0)
    return prism([polar(f(ro,t),t) for t in A]+[polar(f(ri,t),t) for t in A[::-1]],z0,z1-z0)
def sloft(sec,r0,r1,a0,a1,n=6):   # an (inset, z) section swept from radius r0 at azimuth a0 to r1 at a1 (ruled lofts, n steps)
    W=lambda r,a:cq.Wire.makePolygon([cq.Vector(*polar(r-m,a),z) for m,z in sec],close=True)
    A=np.linspace(a0,a1,n+1);Rr=np.linspace(r0,r1,n+1)
    return union(*[cq.Solid.makeLoft([W(Rr[k],A[k]),W(Rr[k+1],A[k+1])],True) for k in range(n)])
WALL2,ZW2=3.8,100.0-DZ     # upper wall (z >= ZW2 + 1) thickened inward by a 51 deg step at ZW2 (steeper than 45 with faceting margin); 41 mm under the top as before.
                           # L2: 3.2 -> 3.8 mm. The shorter gussets (rail top to the hit 60 mm instead of 102.5) are stiffer, so the same impact energy gives a
                           # larger equivalent force (95 N vs 57 N at 0.15 m/s): the wall above the gusset tip reached 21.6 MPa at 3.2 mm (limit 16.7); 3.65 mm
                           # just passes (16.6), 3.8 mm leaves about 8 % (15.3 MPa) for a model this rough (validation interlayer_stress)
SEC=[(0,ZB0),(WALL,ZB0),(WALL,ZW2),(WALL2,ZW2+1.25*(WALL2-WALL)),(WALL2,ZB1-2-6.6/6*(WALL+6-WALL2)),(WALL+6,ZB1-2),(WALL+6,ZB1),(0,ZB1)]   # wall + 6 mm inner top rim, underside 47.7 deg (faceting margin)
AZT,AZF,TW=78,70,2.0       # end taper start, foam end, taper wall
seamcut=box(0,G-100,ZB0-2,400,200,ZB1-ZB0+4)                # keeps y >= G (left half)
FR=frust(.5,400).intersect(box(0,0,0,1200,1200,400))   # camera image frustum (apex at the lens face), lens to floor
FRB=frust(.5,400,CAMB).intersect(box(0,0,0,1200,1200,400))   # the same with the apex 3 mm further back
# Camera window (task F), cut through both halves' shell and foam (symmetric about the seam, so the mirrored right half gets the same cut).
# Plan shape: side walls from an apex 7.5 mm behind the lens face (>= 2.3 mm normal to the 3-mm-back image frustum, >= 4.3 mm to the
# lens-face one along the 1 mm land; the bumper rules are 1.5 and 4.0 mm, the rest is for the halves' planar play), then a 1 mm land square
# to the inner wall face, then flaring at WIN_FLARE deg through the wall from x WIN_X + WIN_LAND so a pole caught at a window edge while the robot spins pushes that half along its travel (a 42 deg face is friction-locked at
# object mu 0.5: validation spin_sweep). Sill and roof are flat, at the frustum's bottom at the skin and its top at the shell's outer face
# from an apex 8 mm behind the lens (2.8 / 4.5 mm). The rest pose is the window's worst case: at full travel a half is 4.33 mm nearer
# the lens, where the image is 3.9 mm narrower per side than the 2.5 mm the half moves inward.
# Printing: above the window each half's wall is free at the seam end, so its first layer would hang in the air. The as-printed halves
# (print-parts STL) carry a 1.8 mm print-support plug filling the window at r WIN_PLUG_R (and, on the right half, a pillar under the
# upper seam backer); both are cut out after printing, like the battery tray's breakaway posts. The assembly shows the open window.
WIN_AY,WIN_AZ,WIN_X,WIN_LAND,WIN_FLARE=CAMO.x-7.5,CAMO.x-8.0,153.0,1.0,62.0   # the 1 mm land keeps the side face square to the inner wall face (no knife edge)
WIN_Z=(round(CH-(R_SKIN-WIN_AZ)*tv,1),round(CH+(R_SH-WIN_AZ)*tv,1))   # sill (frustum bottom at the skin), roof (frustum top at the shell's outer face)
WIN_PLUG_R=(154.8,156.6)                                            # print-support plug: inside the 2.4 mm seam-band wall (R155..157.4) and the 3.8 mm wall
def win_hw(x):return (x-WIN_AY)*th if x<=WIN_X else (WIN_X-WIN_AY)*th+max(0.,x-WIN_X-WIN_LAND)*math.tan(math.radians(WIN_FLARE))   # window half-width at x
def win_z(x):return WIN_Z                                             # window (bottom, top): flat
_wx=[WIN_AY+3.0,WIN_X,WIN_X+WIN_LAND,170.0]
WIN=prism([(x,win_hw(x)) for x in _wx]+[(x,-win_hw(x)) for x in _wx[::-1]],WIN_Z[0],WIN_Z[1]-WIN_Z[0])
WIN_PLUG=prism([(x,win_hw(x)+.3) for x in _wx]+[(x,-win_hw(x)-.3) for x in _wx[::-1]],WIN_Z[0]-.3,WIN_Z[1]-WIN_Z[0]+.6).intersect(ring(WIN_PLUG_R[0],WIN_PLUG_R[1],WIN_Z[0]-.3,WIN_Z[1]+.3,-30,30))   # 0.3 mm into the wall all round (a real overlap, no coincident faces)
AZW=math.degrees(math.atan2(win_hw(R_SKIN),R_SKIN))   # window half-width at the skin (deg)
RAMP_A=15.0;RAMP_D=round(R_SKIN-(R_SH-WALL+1.2),2)   # seam-end lead-in: angle from the tangent, depth below the skin (down to a 1.2 mm end land, the thinnest printable wall)
def seam_ramp():   # cutter for the outer seam-end corner (left half): everything outside a line 20 deg from the tangent through (R_SKIN-2.5, y=G)
    P1=(math.sqrt((R_SKIN-RAMP_D)**2-G*G),G);ae=math.degrees(math.atan2(G,P1[0]));h=math.radians(ae+90-RAMP_A)
    v=(math.cos(h),math.sin(h));n=(v[1],-v[0]);L=(RAMP_D+.6)/math.sin(math.radians(RAMP_A))
    q0=(P1[0]-v[0],P1[1]-v[1]);q1=(P1[0]+L*v[0],P1[1]+L*v[1])
    return prism([q0,q1,(q1[0]+8*n[0],q1[1]+8*n[1]),(q0[0]+8*n[0],q0[1]+8*n[1])],ZB0-2,ZB1-ZB0+4)
RAMPCUT=seam_ramp()
ZS0=ZB1-1.2                # seam band top (thin wall, no rim, 1.2 mm outer chamfer): the backer behind the seam rises to it
SECN0=[(0,ZB0),(WALL,ZB0),(WALL,ZB1),(1.2,ZB1),(0,ZB1-1.2)]   # seam band (az 0..3.5): thin wall keeps the seam-end inner corner at R_SH - WALL, no rim (backer clearance)
SECT=[(0,ZB0),(WALL,ZB0),(WALL,ZW2),(WALL2,ZW2+1.25*(WALL2-WALL)),(WALL2,ZB1),(1.2,ZB1),(0,ZB1-1.2)]   # az 3.5..5.5: the upper wall thickens, still no rim (task F: the
                           # wall step lies inside the camera window, which removes it there; at 4.5 it would leave a 0.9 mm strip beside the window side;
                           # the rim starts at 5.5, clear of the window's 93.1 mm roof)
bshell=union(arc(SECN0,R_SH,0,3.5),arc(SECT,R_SH,3.5,5.5),arc(SEC,R_SH,5.5,AZT),spiral(AZT,90,(R_SH,R_TIP),(R_SH-TW,R_TIP-TW),ZB0,ZB1)).cut(seamcut).cut(RAMPCUT).cut(WIN)
bplug=WIN_PLUG.cut(seamcut).cut(RAMPCUT)   # print-support plug of the left half's window (as printed only)
D=prism([(0,G)]+[polar(R_SH-1,a) for a in np.linspace(1.3,90,30)],17,13)   # plan sector of the left half (clips the under-base guides)
def csk(x,y):   # Ø3.2 hole + 90 deg countersink Ø6.9 at the lower edge: 45 deg walls, no support
    return union(cyl(x,y,17,1.6,13),cq.Solid.makeCone(4.45,1.6,2.85,cq.Vector(x,y,17)))
rail=prism([(56,RAILY[1]),(56,RAILY[0]+4),(60,RAILY[0]),(158,RAILY[0]),(158,RAILY[1])],RZ[0],RZ[1]-RZ[0]).intersect(D)   # top 1.5 mm under P_base; tail chamfered
arm=prism([(47,80),(61,80),(61,98),(58,101),(58,158),(52,158),(52,101),(47,96)],RZ[0],RZ[1]-RZ[0]).intersect(D)          # outboard arm: 14 wide at pin C, the arm ball (11.5 mm pocket) and the leaf root, 6 wide out to the shell
GUST=4.0                   # gusset thickness; each top edge rises at 45 deg into the wall instead of ending in a square tip
GTIP=(110-DZ,132-DZ)       # gusset tip heights on the wall: rail and arm (into the top rim); they follow the top down
RGY=21.0                   # rail gusset centre y (task F: from 17 to 21, on the rail's outer edge, so its tip clears the camera window flare)
gus=union(xz_fin([(127,RZ[1]),(155,RZ[1]),(155,GTIP[0]),(127,GTIP[0]-28)],RGY-GUST/2,RGY+GUST/2),     # rail-to-shell gusset (ends at x 155, inside the wall: at y 19-23 the shell's outer face is at x 155.8-156.2)
          yz_fin([(106,RZ[1]),(146,RZ[1]),(146,GTIP[1]),(106,GTIP[1]-40)],55-GUST/2,55+GUST/2))      # arm-to-shell gusset
def cpts(s0=S0):   # cam profile in the switch frame (roller-top line): rest flat, 40 deg ramp, dwell
    return [(XR-9,YREST),(XR+s0,YREST),(XR+S1,YDW),(XR+6.6,YDW)]
camL=prism(cpts()+[(XR+6.6,40),(XR-9,40)],RZ[0],RZ[1]-RZ[0])
cam=swW(camL).intersect(box(100,RAILY[0]+60,17,240,120,13))   # cam block on the rail's switch side, merged into the rail
def lpts(R0,dirv,u):   # leaf outline: Ø4 root boss, constant-strength taper (2.13 mm at the boss edge, >= 1.3), nose at LL
    qs=[-2.,LRB]+list(np.linspace(LRB,LL,25)[1:])
    return [(R0[0]+q*dirv[0]+tq_(q)/2*u[0],R0[1]+q*dirv[1]+tq_(q)/2*u[1]) for q in qs]+[(R0[0]+q*dirv[0]-tq_(q)/2*u[0],R0[1]+q*dirv[1]-tq_(q)/2*u[1]) for q in qs[::-1]]
fa=lambda q,d=0:(LR0[0]+q*lax[0]+(tq_(q)/2-d)*ft[0],LR0[1]+q*lax[1]+(tq_(q)/2-d)*ft[1])   # leaf face on the acute (+ft) side
qC=(LR0[0]-fa(0)[0])/lax[0];C0=fa(qC)                                                        # where that face meets the arm edge x=61
bis=(lax[0],lax[1]-1);bl=math.hypot(*bis);bis=(bis[0]/bl,bis[1]/bl)
fillet=prism([(C0[0]-.3,C0[1]),(C0[0]-.3,C0[1]-5.6),(C0[0]+4.3*bis[0],C0[1]+4.3*bis[1]),fa(qC+5.6,.3),fa(qC,.3)],LZB[0],LZB[1]-LZB[0])   # R1.5-equivalent root fillet in the 30 deg corner
leaf=union(prism(lpts(LR0,lax,ft),LZB[0],LZB[1]-LZB[0]),fillet)
def pocket(p,z0,h):return stadium((p[0]-(BPL/2-PKR)*ft[0],p[1]-(BPL/2-PKR)*ft[1]),(p[0]+(BPL/2-PKR)*ft[0],p[1]+(BPL/2-PKR)*ft[1]),PKR,z0,h)
def race(p,L,W,z0,u=ft):return obox(p,u,L,W,z0,RACE_T)   # race strip centred at p, long side along u
hguide=union(bshell,rail,cam,arm,gus)
for x,y in RAILP+[PB]:hguide=hguide.cut(csk(x,y))
for p in BALL_DN:hguide=hguide.cut(pocket(p,ZBD,RZ[1]-ZBD+1))    # Ø5 × 9.5 ball pockets in the rail / arm top (floor race 24.0..24.5)
hguide=tidy(hguide)                                    # shell + rail + cam + arm + gussets (rigid part)
bleaf=union(leaf,cyl(*LT0,LZB[0],NR,LZB[1]-LZB[0]),cyl(*LR0,LZB[0],LRB,LZB[1]-LZB[0]))   # as printed: free leaf, Ø4 nose boss, root boss
ZFT=114-DZ                 # foam top (unchanged by task F; the camera window cuts its top edge at the seam)
foam=union(ring(R_SH,R_SKIN,26,ZFT,0,AZF-4),spiral(AZF-4,AZF,(R_SKIN,R_SH+.02),(R_SH,R_SH),26,ZFT,6)).cut(seamcut).cut(RAMPCUT).cut(WIN)   # bevel-cut at the seam-end ramp; camera window
# Seam backer on the right half. The left half moves along (-cos30, -sin30) and the right half along (-cos30, +sin30), so in the right
# half's frame the left end's inner corner CL sweeps the rhombus CL + (-4.33 t1 + 4.33 t2, -2.5 t1 - 2.5 t2), t1, t2 in 0..1; its
# foremost point is CL.x - 4.33. The backer is one flat step facing +x at XSTEP = CL.x - 4.33 - 2.3 (2.3 mm to the swept region,
# >= 1.5 mm with the planar play), spanning the whole seam (y -4.35..+3.75), joined by a web behind the right end (y < -4.35) to the
# right wall. Its face normal +x pushes the right half along its travel with drive cos30 = 0.87 (no face parallel to the travel).
SEAM_C=2.3;CL=(math.sqrt((R_SH-WALL)**2-G*G),G);XSTEP=CL[0]+BT*ft[0]-SEAM_C;TSTEP,WEBT=1.8,2.4
TZ=ZB1-ZB0                                              # backer top = bumper top (a rod over the seam band's chamfer meets it); the camera window band (sill to roof) is cut out of it
tongue=union(box(XSTEP-TSTEP/2,(G-(G+.6))/2,ZB0,TSTEP,2*G+.6,TZ),   # step y -(G+0.6)..+G: >= 2 mm to the left rail edge (y 11) at both full travels

             box((XSTEP-TSTEP+R_SH-WALL+.6)/2,-(G+.6+WEBT/2),ZB0,R_SH-WALL+.6-(XSTEP-TSTEP),WEBT,TZ)).cut(box(150,0,WIN_Z[0],24,24,WIN_Z[1]-WIN_Z[0]))
bpillar=box(XSTEP-TSTEP/2,(G-(G+.6))/2,WIN_Z[0],TSTEP,2*G+.6,WIN_Z[1]-WIN_Z[0])   # print-support pillar under the upper backer (right half, as printed only)
# Deflected leaf (assembly pose): constant-strength taper integrated as a tip-loaded cantilever from the root-boss edge, with the
# bending shortening of the projected length, so the Ø4 nose sits on the abutment face. The printed part (P_bumper_*.stl) is the free leaf.
def leaf_curve(dl,n=60,dt=0.):   # (axial x(q), deflection w(q), slope th(q)) at q = 0..LL for a nose deflection dl
    q=np.linspace(0,LL,n*8+1);t=np.array([tq_(x,dt) for x in q]);kap=np.where(q>LRB,(LL-q)/t**3,0.)
    th=np.concatenate([[0],np.cumsum((kap[1:]+kap[:-1])/2*np.diff(q))]);w=np.concatenate([[0],np.cumsum((th[1:]+th[:-1])/2*np.diff(q))])
    sc=dl/w[-1]   # small-slope start
    for _ in range(6):   # rescale the curvature so that the exact (large-slope) tip deflection equals dl
        th2=th*sc;w=np.concatenate([[0],np.cumsum((np.sin(th2[1:])+np.sin(th2[:-1]))/2*np.diff(q))]);sc*=dl/w[-1]
    th=th*sc;x=np.concatenate([[0],np.cumsum((np.cos(th[1:])+np.cos(th[:-1]))/2*np.diff(q))]);w=np.concatenate([[0],np.cumsum((np.sin(th[1:])+np.sin(th[:-1]))/2*np.diff(q))])
    return q,x,w,th
def bent_leaf(sg,s,nose=True,root=True):   # deflected leaf solid for travel s (root moved s along the travel, nose held PL behind its free position)
    u=(ft[0],sg*ft[1]);d=(lax[0],sg*lax[1]);R0=(LR0[0]+s*u[0],sg*LR0[1]+s*u[1]);q,x,w,th=leaf_curve(PL+s);k=np.flatnonzero(q>=LRB-1e-9)
    kk=sorted(set(list(k[::4])+[len(q)-1]));c=[(R0[0]+x[i]*d[0]-w[i]*u[0],R0[1]+x[i]*d[1]-w[i]*u[1]) for i in kk]
    nn=[(math.sin(th[i])*d[0]+math.cos(th[i])*u[0],math.sin(th[i])*d[1]+math.cos(th[i])*u[1]) for i in kk]
    tt=[tq_(q[i])/2 for i in kk]
    sol=prism([(p[0]+t*n_[0],p[1]+t*n_[1]) for p,t,n_ in zip(c,tt,nn)]+[(p[0]-t*n_[0],p[1]-t*n_[1]) for p,t,n_ in list(zip(c,tt,nn))[::-1]],LZB[0],LZB[1]-LZB[0])
    parts_=[sol]+([cyl(*c[-1],LZB[0],NR,LZB[1]-LZB[0])] if nose else [])+([cyl(*R0,LZB[0],LRB,LZB[1]-LZB[0])] if root else [])
    return union(*parts_)
def bent_noseL(sg,s):   # deflected nose centre (world, side sg) at travel s
    u=(ft[0],sg*ft[1]);d=(lax[0],sg*lax[1]);q,x,w,th=leaf_curve(PL+s);R0=(LR0[0]+s*u[0],sg*LR0[1]+s*u[1])
    return (R0[0]+x[-1]*d[0]-w[-1]*u[0],R0[1]+x[-1]*d[1]-w[-1]*u[1])
def leaf_nose(s):   # nose-centre position along the leaf axis from the free nose (negative = toward the root) at travel s
    q,x,w,th=leaf_curve(PL+s);return float(x[-1]-LL),float(math.degrees(th[-1]))
BNOTE=('Print upright on the lower edge (z=18 on the bed), no supports (countersinks, rim, wall step and gussets are 45 deg). 90 deg arc about the robot centre, '
 f'R{R_SH} × {WALL} mm wall ({WALL2} mm above z {ZW2:g}) with a 6 mm 45 deg inner top rim, z 18–{ZB1:g} (full height all round; the first 3.5 deg from the seam keep the 2.4 mm wall and the first 5.5 deg have no rim); '
 f'Camera window across the seam (task F): z {WIN_Z[0]:g}–{WIN_Z[1]:g}, flared sides. The STL carries a 1.8 mm print-support plug in the window (the right half also a pillar under the upper seam backer): after printing, cut them out round their edges with a hobby knife or flush cutter and file flush; they only hold up the wall above the window while it prints. '
 f'1.5 mm EVA foam to the R{R_SKIN} skin from the seam to az 70 (bevel-cut along the seam-end ramp, cut out at the camera window); az 70–90 bare PETG covered with 0.1 mm PTFE film tape; az 78–90 tapers to R{R_TIP} (1.2 mm outside the start of the rear shell). '
 f'The seam end is a {RAMP_A:.0f} deg lead-in ramp {RAMP_D} mm deep through foam and shell. Before bonding the foam, check the printed outer radius with a printed R{R_SH} check template (+0.3 mm max; sand high spots) and use foam measured at 1.5 ± 0.2 mm. '
 f'Guide: a rail (y 11–23) and an outboard arm run 1.5 mm under P_base on three pins fixed by M3 × 35 countersunk screws from below; 4 mm gussets rise at 45 deg into the wall (to z {GTIP[0]} and {GTIP[1]}). '
 f'Rail pins A/B: Ø4.5 × 1.5 inner-race spacer, 693ZZ bearing rolling in a {SLOTW} mm P_base slot (filed to {SLOTW-.1:.1f}–{SLOTW}), Ø4.5 × 1 spacer, M3 thin clamp nut (threadlocker). Pin C: Ø5 × 6.5 sleeve in a 6.2 mm slot, thin clamp nut. '
 'Vertical guide, steel on steel only: three Ø4 balls in rail/arm-top pockets roll between a 0.5 mm hardened spring-steel strip in the pocket floor and a strip bonded under P_base; '
 'two balls under each floating printed rider roll between a strip on the base top and a strip in the rider pocket ceiling. Above each rider: brass stop tube Ø4 × 5, '
 'compression spring (Ø5 OD, 2.0 N/mm), Ø7 washer and nyloc nut; at assembly tighten the nyloc until a 0.1 mm feeler just fits between the washer and the stop tube '
 f'({PRE_P} N preload per pin). The springs pull the half up against its three down balls: zero vertical play at rest whatever the printed plate thickness (±0.15) or pocket depth. '
 "Slot ends give the rest stop and the hard stop (5.0 mm with the bearing centred, 4.9 mm on a slot side) along the half's own axis, 30 deg inward of rearward, so the skin only moves inward. "
 f'Leaf spring: constant-strength taper {LTR} → {LT} mm × 9.5 mm, {LL-LRB:.0f} mm free from its root boss (R1.5 root fillet) to a Ø{2*NR:.0f} nose boss, perpendicular to the travel, preloaded {PL} mm against a base abutment. '
 f'Print the leaf with 0.45 mm extrusion width and at least 3 perimeters (it is all perimeter); after printing, measure it at mid-length ({tq_(LL/2):.2f} mm nominal) and check the push force at 2.25 mm of travel with a gram gauge (validation leaf_spring.force_at_2p25mm_N; shim the abutment face with 0.1 mm PTFE tape if it is more than 25 % low). '
 'The rail cam (40 deg ramp, then a dwell parallel to the travel) presses the SS-5GL2 roller; the switch is set at assembly to operate at 2.25 mm (see S03). '
 f'Slicer: 4 perimeters (the walls and the 4 mm gussets are all perimeter), PETG at the high end of its range (about 245 °C) with part cooling ≤ 30 % for layer adhesion below print z {ZB1-ZB0-18:g}, then a height-range modifier from print z {ZB1-ZB0-18:g} to the top: '
 f'fan 50–60 % and 50 % speed for the 45 deg inner top rim (it would curl at low fan); a 5 mm brim on both arc ends plus mouse ears, reduced speed above print z {ZB1-ZB0-33:g} (tall part on the moving Y bed). '
 f'Release test before use: push the half at z {ZB1:g} into its hard stop with 50 N and drop the robot from 5 cm onto a hard floor; no whitening or cracks at the gusset joints or the wall step. Never lift the robot by a bumper half or the rear shell.')
SWNOTE=('Omron SS-5GL2 hinge ROLLER lever: body 19.8 × 10.2 × 6.4 mm hanging under P_base on two M2 × 14 screws with nuts on the base top, through '
 '2.4 × 4.2 mm slots along the switch Y axis (±0.9 mm adjustment), turned with the travel. Datasheet: FP max 19.3, OP 14.5 ± 0.8, OT min 1.2, MD max 0.8 mm '
 '(roller top above the hole line). Calibrated this way the roller sits about 1 mm above the release position at rest and 0.4–1.0 mm past OP on the dwell. Assembly: push the half in '
 'by 2.25 mm (gauge block at the rest stop; the roller force holds the bearing on its slot side), slide the switch toward the cam until it just operates, tighten. The adjustment nuts sit on the base top in the front bay, beside the bumper rider B.')
def rider_shape(x,y,k,c=2.0):   # floating rider: octagon (2 mm corner chamfers) around two ball pockets, counterbore over the clamp nut
    hn,hf=RIDE[k]+PKR+1.4,BPL/2+1.4
    L=[(hn-c,hf),(hn,hf-c),(hn,-hf+c),(hn-c,-hf),(-hn+c,-hf),(-hn,-hf+c),(-hn,hf-c),(-hn+c,hf)]
    r=prism([(x+a*nrm[0]+b*ft[0],y+a*nrm[1]+b*ft[1]) for a,b in L],ZRB,RIDT)
    return r.cut(union(cyl(x,y,ZRB-1,1.7,RIDT+2),cyl(x,y,ZRB-1,3.5,ZCB-ZRB+1)))
for side,sgn in [('left',1),('right',-1)]:
    m=(lambda s:s) if sgn>0 else (lambda s:s.mirror('XZ'))
    swb=union(box(0,0,22.6,19.8,10.2,6.4),box(0,-6.5,23.4,12,3.2,4.8))   # body + terminals (switch frame)
    lv=(XR-XH,YREST-RR-5.6);L_=math.hypot(*lv)
    sw=union(swb,obox(((XH+XR)/2,(5.6+YREST-RR)/2),(lv[0]/L_,lv[1]/L_),L_,.8,24.2,3.2),cyl(XR,YREST-RR,24.2,RR,3.2))   # + lever strip + Ø4.8 roller
    add('bumper_switch_'+side,'NC bumper microswitch '+side+' (SS-5GL2) under the base',m(swW(sw)),'black','S03','base',check=True,allow=('P_base','P_bumper_'+side),note=SWNOTE)
    for j,X in enumerate([-4.75,4.75]):
        x,y=mir(swp(X,HY),sgn)
        add(f'bumper_switch_screw_{side}_{j}','Switch M2 × 14 screw, washer and nut on the base top',union(cyl(x,y,21.0,1.9,1.6),cyl(x,y,21.0,1.0,15.6),cyl(x,y,33,2.4,1.6)),'metal','H08','base',check=True,allow=('P_base','bumper_switch_'+side))
    half=union(m(hguide),m(bleaf),m(bplug),*([] if sgn>0 else [tongue,bpillar]))        # as printed: free leaf; window print-support plug (and backer pillar) to cut out
    hasm=union(m(hguide),m(fillet),bent_leaf(sgn,0.),*([] if sgn>0 else [tongue]))     # assembly: leaf deflected onto its abutment
    add('P_bumper_'+side,'Front bumper arc half '+side,tidy(hasm),'print','F01/F02','lower',printable=True,explode=[60,sgn*35,0],pshape=tidy(half),
        note=BNOTE+(' Right half also carries the seam backer (a flat step facing forward across the 7.5 mm seam).' if sgn<0 else '')+' The STL is the as-printed half (free leaf); the assembly shows the leaf deflected onto its abutment.',tess=(.35,.035))
    add('bumper_foam_'+side,'Soft bumper facing '+side+' (EVA arc strip)',m(foam),'black','F02','lower',explode=[60,sgn*35,0],tess=(.35,.035),allow=('P_bumper_'+side,),note='Bonded to the shell (the tapered end overlaps the shell surface by the polygon chord sag only).')
    for k,(x,y) in zip('ABC',RAILP+[PB]):
        mid=union(cyl(x,y,RZ[1],2.25,29-RZ[1]),cyl(x,y,29,4,4),cyl(x,y,33,2.25,1)) if k<'C' else cyl(x,y,RZ[1],2.5,34-RZ[1])
        top=union(cyl(x,y,34,3.2,1.8),cyl(x,y,ZRT,2.0,SPR['stop_tube']),cyl(x,y,ZRT,SPR['od']/2,ZW-ZRT).cut(cyl(x,y,ZRT-1,2.0,9)),cyl(x,y,ZW,3.5,.5),cyl(x,y,ZW+.5,3.2,4))
        pin=union(cq.Solid.makeCone(3.36,1.5,1.86,cq.Vector(x,y,18)),cyl(x,y,18,1.5,35),mid,top)
        rid_id=f'P_bumper_rider_{side}_{k}'
        add(f'bumper_pin_{side}_{k}','Bumper guide pin: M3 × 35 countersunk, '+('Ø4.5 × 1.5 spacer, 693ZZ bearing, Ø4.5 × 1 spacer' if k<'C' else 'Ø5 × 6.5 sleeve')+', thin clamp nut, (rider), stop tube, spring, washer, nyloc nut',m(pin),'metal','H08','lower',check=True,
            allow=('P_base','P_bumper_'+side,rid_id),explode=[60,sgn*35,0],
            note='Fixed in the bumper half. '+(f'The 3 × 8 × 4 mm bearing rolls on one side of the {SLOTW} mm P_base slot (±{SPLAY:.1f} mm play); the Ø4.5 spacers bear only on its inner-ring land; the thin nut clamps the stack. ' if k<'C' else 'The sleeve sits in a 6.2 mm P_base slot with 0.6 mm side play: this pin carries only vertical (roll) loads. ')+
                 'The rider floats on the pin; the spring above it (set to a 0.1 mm feeler gap over the stop tube) gives '+str(PRE_P)+' N preload; the stop tube limits further compression to 0.1 mm.')
        for j,sg2 in enumerate(RSIDES[k]):
            q=pp((x,y),sg2*RIDE[k]);qb=(q[0]+BT/4*ft[0],q[1]+BT/4*ft[1]);bid=f'bumper_ball_{side}_{k}{j}'
            add(bid,'Ø4 steel ball under the rider (rolls between steel races)',m(cq.Solid.makeSphere(BR,cq.Vector(*q,ZBT+BR),angleDegrees1=-90)),'metal','H08','lower',check=True,allow=('P_base',rid_id,f'bumper_race_{side}_{k}{j}',f'base_race_{side}_{k}{j}'),explode=[60,sgn*35,0])
            add(f'bumper_race_{side}_{k}{j}',f'Rider pocket-ceiling race {PRACE[0]:.0f} × {PRACE[1]:.0f} × 0.5 (hardened spring steel, bonded)',m(race(q,PRACE[1],PRACE[0],ZRC-RACE_T)),'metal','H08','lower',check=True,allow=(rid_id,bid),explode=[60,sgn*35,0])
            add(f'base_race_{side}_{k}{j}',f'Base-top race strip {RACE_W} × {RACE_L:.0f} × 0.5 (hardened spring steel, bonded to P_base)',m(race(qb,RACE_L,RACE_W,33.0)),'metal','H08','base',check=True,allow=('P_base',bid))
        rid=rider_shape(x,y,k)
        for j,sg2 in enumerate(RSIDES[k]):rid=rid.cut(pocket(pp((x,y),sg2*RIDE[k]),ZRB-1,ZRC-ZRB+1))
        add(rid_id,'Bumper ball rider '+side+' '+k,m(rid),'print','F01','lower',printable=True,explode=[60,sgn*35,0],note=f'Floats on the pin above P_base, pushed down by the preload spring; two Ø4 balls in Ø5 × {BPL} pockets (steel ceiling races) roll on the base-top races beside the slot. Counterbore over the clamp nut. Print top face down.')
    for j,p in enumerate(BALL_DN):
        bid=f'bumper_ball_{side}_d{j}';pb_=(p[0]+BT/4*ft[0],p[1]+BT/4*ft[1])
        add(bid,'Ø4 steel ball in a rail/arm-top pocket (rolls between steel races under the base)',m(cq.Solid.makeSphere(BR,cq.Vector(*p,ZBD+RACE_T+BR),angleDegrees1=-90)),'metal','H08','lower',check=True,allow=('P_base','P_bumper_'+side,f'bumper_race_{side}_d{j}',f'base_race_{side}_d{j}'),explode=[60,sgn*35,0])
        add(f'bumper_race_{side}_d{j}',f'Pocket-floor race {PRACE[0]:.0f} × {PRACE[1]:.0f} × 0.5 (hardened spring steel, bonded)',m(race(p,PRACE[1],PRACE[0],ZBD)),'metal','H08','lower',check=True,allow=('P_bumper_'+side,bid),explode=[60,sgn*35,0])
        add(f'base_race_{side}_d{j}',f'Base-underside race strip {RACE_W} × {RACE_L:.0f} × 0.5 (hardened spring steel, bonded to P_base)',m(race(pb_,RACE_L,RACE_W,ZBU)),'metal','H08','base',check=True,allow=('P_base',bid))

# Controls: master rocker, motor toggle and arm button through the lid (L2: lowered with it by DZ, same plan positions). Each hangs inside
# the lid over the battery (pack top z 64), clear of the Tamiya pair on the pack top and of the bay board modules (x >= -59): validation
# lid_switches lists the gaps.
MS,TOG,ARM=(-100,0),(-85,67),(-80,-70)
add('master_switch','Master power rocker on the lid top',union(box(*MS,ZLIDT-25,12.6,19.8,25),box(*MS,ZLIDT,24,21,3)),'black','P08','cover',check=True,note=f'Envelope: snap-in body 12.6 × 19.8 × 25 mm (z {ZLIDT-25:g}–{ZLIDT:g}, through a 13.2 × 20.4 mm lid opening behind the LiDAR; file to the datasheet panel cut-out so the snap tabs hold) plus a 24 × 21 × 3 mm bezel on the lid top (z {ZLIDT:g}–{ZLIDT+3:g}), far below the LiDAR scan band. The body bottom is {ZLIDT-25-64:.1f} mm over the battery top. A 15.7 × 24.4 × 1.5 mm relief pocket under the lid (1.25 mm toward +x) leaves a 1.5 mm panel at the opening: choose a rocker whose datasheet panel range includes 1.5 mm (most snap-in rockers take 0.8–2 mm).')
stop=union(box(*TOG,ZLID-15,13,8,14),cyl(*TOG,ZLID-1,3,7),cyl(*TOG,ZLIDT+3,1.8,10))
add('motor_switch','Motor power ON/OFF toggle',stop,'black','S01','cover',check=True,note=f'Maintained SPST mini-toggle envelope (body z {ZLID-15:g}–{ZLID-1:g} under the lid); 6.6 mm printed panel hole (ream to the datasheet size). Switch interrupts relay coil; confirm purchased switch drawing.')
add('arm_button','Manual arm button',union(cyl(*ARM,ZLID-15,6,18),cyl(*ARM,ZLIDT,8,3.5)),'buy','S02','cover',note=f'12 mm panel button: body through the Ø12.6 printed lid hole (ream to the datasheet size; body z {ZLID-15:g}–{ZLIDT:g}), Ø16 cap on the lid top (z {ZLIDT:g}–{ZLIDT+3.5:g}).')
# Lid posts (L2): Ø9 printed columns from P_base to the lid underside, open-sided (wire access and ventilation). Each stands on a 22 × 12 × 3
# foot held by two M3 × 6 pan-head machine screws in the base inserts at x ±8 (the deck-era saddle inserts), beside the column; the lid
# screws down with one M3 × 6 pan-head machine screw into an M3 × 3 heat-set insert pressed flush into each post top (repeated removal).
for i,(x,y) in enumerate(LIDPOSTS):
    q=union(box(x,y,33,*POST_FOOT),cyl(x,y,33+POST_FOOT[2]-.01,POST_R,ZLID-33-POST_FOOT[2]+.01))
    for xx in [x-8,x+8]:q=hole(q,xx,y,32,1.7,6)
    q=q.cut(cyl(x,y,ZLID-INS_L-1,INS_R,INS_L+2))   # Ø4.0 × 4 insert bore: insert flush with the post top, 1 mm relief below
    add('P_lid_post_'+str(i+1),f'Lid post {i+1} (P_base to the lid underside)',q,'print','F01','upper',printable=True,
        note=f'Ø{2*POST_R:g} × {ZLID-36:g} mm column on a {POST_FOOT[0]} × {POST_FOOT[1]} × {POST_FOOT[2]} mm foot, z 33–{ZLID:g} (the lid rests on it). Two M3 × 6 pan-head machine screws through the foot into the P_base inserts at x ±8 '
             '(drive them beside the column); an M3 × 3 brass heat-set insert pressed flush into the post top (Ø4.0 × 4 bore) takes the lid screw. Print upright on the foot, no supports; a 3 mm brim helps the tall column.')
    for j,xx in enumerate([x-8,x+8]):tfa(f'lid_post_screw_{i+1}_{j}',xx,y,36,30,allow=('P_base','P_lid_post_'+str(i+1),f'insert_lid_post_{i+1}_{j}'),name='Lid post foot M3 × 6 pan-head machine screw (into insert)',bom='H13')
    add(f'insert_lid_top_{i+1}','M3 × 3 brass heat-set insert (lid post top)',cyl(x,y,ZLID-INS_L,INS_R,INS_L).cut(cyl(x,y,ZLID-INS_L-1,1.5,INS_L+2)),'metal','H12','upper',check=True,allow=('P_lid_post_'+str(i+1),)+LID_IDS,
        note='Pressed flush into the post top with a soldering iron (post standing on a flat plate); the lid screw takes 3 mm of thread.')
# Round cover matching the rear shell (R_RS), printed as four sectors: one disc would be 305 mm across (MK3S bed 250 x 210).
# The seams radiate from the centre at LID_SEAMS (deg) and start at the central opening round the tray's fan intake duct (L2: the centre lies
# inside the fan footprint). The 120 and 215 deg seams meet its fan-window circle: within R_KNEE of the window centre they turn radial to it,
# so those sectors meet the opening square-on (a seam ray from (0, 0) would cut that circle at 30 deg: a sliver tip in print). Task L: the
# opening also takes the duct round the LD14 motor, which contains (0, 0), so the 37 and 300 deg seams run straight along their rays from it
# (they meet its motor circle 24 and 36 deg from square). Two dovetail tabs per seam (0.15 mm clearance, glue them with CA) and the LiDAR tray,
# which bridges the opening, hold the sectors together.
LID_R=R_RS;LID_SEAMS=[37,120,215,300];LID_TABS=(50,95)   # inner tabs at r 50: beyond the 215 deg seam's knee (r 41.4 from the centre)
lid=cyl(0,0,ZLID,LID_R,LID_T)   # task I: closed over the front bay too (the deck-era notch at x 88..rim, |y| <= 58 held the camera tower and guard, which task F removed)
# Seam relief (task I): over the 7.5 mm bumper seam the closed rim (R152.5) would stand ahead of the seam backer (front face XSTEP), so a rod
# at the top of the bumper band pushed into the seam would meet the fixed lid before the sensed backer. A 1.5 mm pocket in the lid underside
# lifts it there to z 102: a Ø3 rod low enough to touch the bumper band (bottom <= ZB1, top <= ZB1 + 3 = 101.5) passes under it. The pocket runs
# back 1 mm past the backer at full travel and is 0.75 mm wider than the seam each side: a rod in the seam meets the backer and pushes it to its
# stop clear of the lid. A pocket, not a slot: a slot that deep would open over the camera lens (the backer is only 1.8 mm thick).
LID_SEAM_RELIEF=(round(XSTEP+BT*ft[0]-1.0,2),G+.75,1.5)   # (x where the pocket starts, half width, depth into the lid underside)
lid=lid.cut(box((LID_SEAM_RELIEF[0]+LID_R+6)/2,0,ZLID-1,LID_R+6-LID_SEAM_RELIEF[0],2*LID_SEAM_RELIEF[1],1+LID_SEAM_RELIEF[2]))
lid=lid.cut(dprism(SK_T+DUCT_FIT,ZLID-1,LID_T+2))   # task L: one opening round the tray's intake duct (its window and motor circles + the wall + 0.3 mm); the tray plate covers it (until task L: a Ø35 window with a skirt under the lid)
lid=hole(lid,*TOG,ZLID-1,3.3,LID_T+4);lid=hole(lid,*ARM,ZLID-1,6.3,LID_T+4);lid=lid.cut(box(*MS,ZLID-1,13.2,20.4,LID_T+4)).cut(box(MS[0]-.75,MS[1],ZLID-1,15.7,24.4,2.5))   # switch cut-outs 0.3 mm per side over each body (PETG holes print undersize; ream to the datasheet panel cut-out); 1.5 mm relief pocket under the rocker (1.5 mm panel for its snap tabs)
for x,y in LIDPOSTS+TRAYH:lid=hole(lid,x,y,ZLID-1,1.7,LID_T+4)
lid=lid.cut(box(LD14_AP[0],LD14_AP[1],ZLID-1,LD14_AP[2],LD14_AP[3],LID_T+4))   # LD14 lead aperture under the tray opening (closed by P_ld14_grommet round the lead)
R_KNEE=28.0   # the 120 / 215 deg seams run radially from the fan-window centre out to R_KNEE (so they meet the opening square-on), then along their rays from (0, 0)
KNEE_SEAMS=(120,215)   # the 37 / 300 deg seams start at (0, 0), inside the opening's motor circle, and run along their rays
def seam_knee(a):   # the point of the ray from (0, 0) at angle a that lies R_KNEE from the window centre (the origin is inside the window)
    u=(math.cos(math.radians(a)),math.sin(math.radians(a)));uc=u[0]*FW[0]+u[1]*FW[1];t=uc+math.sqrt(uc*uc-FW[0]**2-FW[1]**2+R_KNEE**2);return (t*u[0],t*u[1])
def seam_in(a):return [FW[:2],seam_knee(a)] if a % 360 in KNEE_SEAMS else [(0.,0.)]   # a seam's inner end(s), inside the opening
def lid_sector(a0,a1,r=LID_R+6):   # prism over the sector a0..a1 (deg, CCW), z ZLID-1..ZLIDT+4
    n=max(4,int((a1-a0)/2));pts=seam_in(a0)+[(r*math.cos(math.radians(a0+(a1-a0)*k/n)),r*math.sin(math.radians(a0+(a1-a0)*k/n))) for k in range(n+1)]+seam_in(a1)[::-1]
    if pts[-1]==pts[0]:pts=pts[:-1]   # both seams start at (0, 0)
    return cq.Workplane('XY').polyline(pts).close().extrude(LID_T+6).translate((0,0,ZLID-1)).val()
def lid_tab(a,rc,c=0.):   # dovetail across the seam at angle a, centred at radius rc, protruding 5 mm into the CCW sector; c: clearance
    u=(math.cos(math.radians(a)),math.sin(math.radians(a)));nv=(-u[1],u[0]);P=lambda sv,t:(sv*u[0]+t*nv[0],sv*u[1]+t*nv[1])
    pts=[P(rc-3-c,-1),P(rc+3+c,-1),P(rc+3+c,0),P(rc+4.5+c,5+c),P(rc-4.5-c,5+c),P(rc-3-c,0)]
    return cq.Workplane('XY').polyline(pts).close().extrude(LID_T).translate((0,0,ZLID)).val()
seams=sorted(LID_SEAMS);spans=[(seams[-1]-360,seams[0])]+[(seams[k],seams[k+1]) for k in range(len(seams)-1)]
for k,(a0,a1) in enumerate(spans):
    piece=lid.intersect(lid_sector(a0,a1))
    for rc in LID_TABS:
        piece=union(piece,lid_tab(a1,rc))                 # tabs on this sector's CCW seam reach into the next sector
        piece=piece.cut(lid_tab(a0,rc,.15))               # matching slots on its CW seam
    mid=(a0+a1)/2
    add(LID_IDS[k],f'Round cover sector {k+1} of 4 ({a0 % 360:.0f}–{a1:.0f} deg)',tidy(piece),'cover','F01','cover',printable=True,print_rot_z=-mid,print_flip=True,
        note=f'Round removable cover (R{LID_R:g}, z {ZLID:g}–{ZLIDT:g}, flush with the rear shell; the bumper moves below its rim) printed as four sectors because a 305 mm disc does not fit the MK3S bed. '
             f'Seams at 37/120/215/300 deg from the centre. The sectors meet round one central opening for the LiDAR tray\'s fan intake duct (task L: the duct and the LD14 motor in it pass through; {DUCT_FIT:g} mm round the duct wall); '
             f'the 120 and 215 deg seams turn radial to its fan-window circle inside r {R_KNEE:g} of the fan centre, the 37 and 300 deg seams meet its motor circle along their rays. The tray plate covers the opening. '
             f'Two dovetails per seam at r {LID_TABS[0]} and {LID_TABS[1]} (0.15 mm clearance). Dry-fit all four, then glue the dovetails with CA; '
             'the LiDAR tray and the four lid posts (one M3 × 6 screw into the insert in each post top) also tie the sectors. '
             f'The front sector covers the front bay (task I); over the bumper seam a {2*LID_SEAM_RELIEF[1]:g} mm wide, {LID_SEAM_RELIEF[2]:g} mm deep pocket in its underside runs from the rim back to x {LID_SEAM_RELIEF[0]:g}, so an object pushed into the seam meets the sensed seam backer, not the lid. Master rocker opening at (-100, 0) with a 1.5 mm relief pocket underneath (1.5 mm panel for the snap tabs) is in the rear sector. '
             'The STL is written bed-ready: top face down on the bed, no supports.')
# LD14 lead grommet (deferred L2 item): the lid aperture would let warm body air out under the LD14, beside the fan grille's intake (80 mm2 since
# task L, against the 25 % bypass limit of the grille), so a printed grommet fills it round the lead: a body in the aperture (GROM clr a side), a lip on the
# lid top over the aperture's x sides, a hole snug on the Ø2.4 lead and a slit to fit it round the lead (the J28 plug cannot pass). Its residual
# leak (validation jetson_cooling.fan_skirt.ld14_aperture_leak_mm2) is counted in the bypass term.
GROM=dict(clr=.1,hole_d=2.6,slit=.6,lip=1.4,lip_over=.8,z0=1.2)   # z0: the body starts 1.2 mm over the lid underside
_gx,_gy,_ax,_ay=LD14_AP
grom=union(box(_gx,_gy,ZLID+GROM['z0'],_ax-2*GROM['clr'],_ay-2*GROM['clr'],LID_T-GROM['z0']),box(_gx,_gy,ZLIDT,_ax+2*GROM['lip_over'],_ay-2*GROM['clr'],GROM['lip']))
_sl=_ax/2+GROM['lip_over']+1
grom=grom.cut(cyl(_gx,_gy,ZLID-1,GROM['hole_d']/2,LID_T+GROM['lip']+2)).cut(box(_gx+_sl/2,_gy,ZLID-1,_sl,GROM['slit'],LID_T+GROM['lip']+2))
add('P_ld14_grommet','LD14 lead grommet in the lid aperture',grom,'print','F01','cover',printable=True,print_flip=True,allow=LID_IDS,
    note=f'Fills the {_ax:g} × {_ay:g} mm lid aperture at ({_gx:g}, {_gy:g}) round the LD14 lead (W20): {GROM["clr"]} mm clearance a side, a {GROM["lip"]:g} mm lip resting on the lid top, a Ø{GROM["hole_d"]} hole snug on the Ø2.4 lead and a '
         f'{GROM["slit"]} mm slit to slip it round the lead after the J28 plug is through (cut the slit open with a knife if it prints closed). It keeps warm body air from leaking out beside the fan grille\'s intake. Printed lip down (STL bed-ready).')
for i,(x,y) in enumerate(LIDPOSTS):tfa(f'lid_screw_{i+1}',x,y,ZLIDT,ZLID-INS_L,group='cover',allow=LID_IDS+('P_lid_post_'+str(i+1),f'insert_lid_top_{i+1}'),name='Lid M3 × 6 pan-head machine screw (into the post-top insert)',bom='H13',
    note='Through the Ø3.4 lid hole into the M3 × 3 heat-set insert in the post top (3 mm of thread); the four lid screws are the only fasteners to undo to lift the cover.')

# Harness (bay plan B1, end points B8): the Jetson cables run to the port positions; the battery, lid-switch, motor, bumper and cliff leads end in
# the mated plugs on the bay board's connectors (BAY_CONN: the connector body centres in pcb/board_cad.json; B9 sets the cut lengths). Exit heights:
# each lead ends at its plug's top, where the wires leave it. Under the carrier the front cliff leads (J8, J9) leave the tongue forward and
# pass over the P03 (top z 48) below the carrier's leads (z 54.77); the bumper leads leave the side edges (J10/J11) outward. The Jetson 5 V lead
# leaves J3 (B12a: a right-angle Micro-Fit in the band at the right edge, ruling PB5) out of its receptacle's far end past the right edge (-y),
# bends up and runs forward along the W04 corridor (y -56) to the barrel plug.
# Behind the carrier the rear-edge leads drop onto the rear row in lanes: W01 / W02 from the pack top at x -67, W15 / W11 from the -y side at x
# -61 / -58 (z 58 / 62, over the Pico), W14 / W10 from the +y side. B12a: the battery lead drops straight onto J1's XT30 (its mated top z 59),
# and W15 steps out to x -57.5 between J1's plug and F2 there (its x -61 lane would cross the taller XT30).
XE=P_CAM[0]   # USB plug ends (x)
def bc(ref):   # a lead ends where its wires leave its mated plug: a vertical plug's top centre; an off-board plug's far end, half way up (B12a: J3)
    if BCONN[ref]['offboard_reach']:return (*BCONN[ref]['plug_end_xy'],round((PLUG_BOX[ref][4]+PLUG_BOX[ref][5])/2,2))
    return (*BAY_CONN[ref],round(PLUG_BOX[ref][5],2))
wire('W01','Battery adapter lead (Tamiya on the pack top → J1 XT30, bay board rear edge)',[(TAM[0],TAM[1]-14,70.5),(TAM[0],-36,70.5),(-67,BAY_CONN['J1'][1],70.5),(BAY_CONN['J1'][0],BAY_CONN['J1'][1],66),bc('J1')],2,'power','C10')   # off the pack's -y end, over the strap and the tray front wall, down onto the XT30 from above (B12a)
W04_Y,W04_Z=-56.0,58.0   # the W04 corridor (PB5: y -42..-57) outside the carrier's right edge (y -50), over the right motor (top z 52.2)
_w4=bc('J3');_w4o=_w4[1]-BCONN['J3']['offboard_bend']/2   # B12a: out of the Micro-Fit receptacle's far end (-y), then up into the corridor
wire('W04','5 V Jetson lead (J3 Micro-Fit, bay board right edge in the band → barrel plug)',[_w4,(_w4[0],_w4o,_w4[2]),(_w4[0],_w4o,W04_Z),(_w4[0]+8,W04_Y,W04_Z),(P_BAR[0]+14.5,W04_Y,W04_Z),(P_BAR[0]+14.5,P_BAR[1],P_BAR[2]),(P_BAR[0]-1,P_BAR[1],P_BAR[2])],2,'power','C03','upper')   # up out of the J3 plug, out over the right edge (-y), forward outside the carrier and over the right motor, then up to the plug (14.5 mm of bend room)
for i,(x,y) in enumerate(CLIFF_IDS):   # rear pair to the rear connector row (J6, J7), front pair over the motors' front, then over the P03 to the tongue's front edge (J8, J9)
    sg=1 if y>0 else -1;ref=('J6','J7','J8','J9')[i]
    if x<0:pts=[(x,y+7,49),(-107,84,56),(-70,84,56),(-60,60,58),(-60,37,58),bc(ref)] if y>0 else [(x,y+7,49),(-107,-76,56),(-70,-80,56),(-58,-68,62),(-58,10,62),(-62.5,13,60),bc(ref)]   # the left lead passes 3 mm inside the foot of lid post 2; the right one runs over the Pico at z 62 and drops behind it
    else:zf=56 if y>0 else 54.5;pts=[(x,y+7,49),(100,max(y+7,y),zf),(60,sg*50,zf),(46,BAY_CONN[ref][1],51),(6,BAY_CONN[ref][1],51),bc(ref)]   # the lead exits toward +y on both sides (the right one inboard); B9: the right one crosses under W04 (y -56, z 58) at z 54.5 (wire_to_wire)
    wire('W'+str(10+i),f'Cliff sensor lead {i+1} (to {ref} on the bay board)',pts,1,'signal','C06')
wire('W16','Pico USB cable (left stack → Pico on the bay board)',[(XE-1,P_PICO[1],P_PICO[2]),(XE+4.5,P_PICO[1],P_PICO[2]),(XE+4.5,58,64),(25,58,64),(25,97,60),(USB_FACE[0],97,58),(USB_FACE[0],97,USB_FACE[2]),(USB_FACE[0],USB_FACE[1]+PICO_PLUG[1]-1,USB_FACE[2])],2,'usb','C02','upper')   # round the carrier's left side and over the left motor, into the micro-B plug along -y
wire('W17','Single-camera USB cable',[(XE-1,P_CAM[1],P_CAM[2]),(XE+3.5,P_CAM[1],P_CAM[2]),(88,-10,66),(110,-12,CH-2),(XPCB-13,-12,CH-2)],2,'usb','C01','camera')   # task F: forward from the plug, rising gently under the camera mount crossbar into the USB-C plug behind the camera
wire('W19','LiDAR USB extension',[(XE-1,P_LIDAR[1],P_LIDAR[2]),(XE+3.5,P_LIDAR[1],P_LIDAR[2]),(82,-24,80),(60,-62,84),(-35,-62,86),(-35,-40,ZLID-9),(-22,-40,ZLID-6)],2,'usb','C16','upper')   # over the right motor lane, outside the carrier edge; passes 3 mm under the tray nut at (-35,-38)
wire('W20','LiDAR matched replacement / extension lead',[(28,-40,136-DZ),(30,-28,136-DZ),(33,4,ZLID-5.5),(LD14_AP[0],LD14_AP[1],ZLID-4),(LD14_AP[0],LD14_AP[1],ZLID+LID_T/2),(LD14_AP[0],LD14_AP[1],ZLIDT+GROM['lip']),(*lxy(15,-16),LZ-7.8)],1.2,'signal','C17','cover')   # task L: forward under the lid, straight up through the grommet beside the J28 plug and into its lead exit
wire('W02','Master rocker pair (J2, bay board rear edge ↔ lid rocker)',[(MS[0],5,ZLIDT-25),(MS[0],12,ZLIDT-29.5),(-67,12,ZLIDT-29.5),(-67,BAY_CONN['J2'][1],56),bc('J2')],2,'power','C10','cover')   # over the pack top beside the Tamiya pair, then down the x -67 lane onto the rear edge
wire('W14','Motor-toggle loop (J13, bay board rear edge ↔ lid toggle)',[(TOG[0],TOG[1],ZLID-15),(TOG[0],TOG[1],ZLID-22),(-66,62,68),bc('J13')],1.2,'signal','C08','cover')
_j1y=(BCONN['J1']['body_xy'][1],BCONN['J1']['body_xy'][3])   # B12a: past J1's XT30 (mated top z 59) at x -57.5, between its plug and F2
wire('W15','Arm button lead (J12, bay board rear edge ↔ lid button)',[(ARM[0],ARM[1],ZLID-15),(ARM[0],ARM[1],ZLID-22),(-61,-62,60),(-61,_j1y[0]-3,59.6),(-57.5,_j1y[0]-1,59.5),(-57.5,_j1y[1]+1,59.2),(-61,_j1y[1]+3,58.8),(-61,-2,58),bc('J12')],1,'signal','C08','cover')   # along the x -61 lane over J2, round J1's plug, then down beside the Pico
wire('W18','Fan lead',[(JFAN[0]+20,JFAN[1]+17,JFAN[2]+5),(P_FAN[0],JFAN[1]+17,JFAN[2]+5),(P_FAN[0],P_FAN[1],JFAN[2]-5),(P_FAN[0],P_FAN[1],P_FAN[2]+1)],.8,'signal','E09','upper')   # from the fan's front face, ahead of the heatsink, down onto J20
for side,sgn in [('left',1),('right',-1)]:
    # switch lead: from the terminals under the base, up through the Ø6 hole behind the switch, then aft along the side lane over the motor
    t0,t1,h=mir(swp(-4,-7.1),sgn),mir(swp(-13,-5.1),sgn),mir(swp(*SWLEAD),sgn)
    ref=('J10' if sgn>0 else 'J11');xb=BAY_CONN[ref][0]   # J10 / J11 on the side edges under the carrier: the lead drops outside the carrier edge (|y| 50) and enters the connector along -|y|
    wire('W_bumper_'+side,f'Bumper lead {side} (to {ref} on the bay board)',[(*t0,25.8),(*t1,25.8),(*h,27),(*h,45),(50,sgn*62,52),(xb,sgn*62,56),(xb,sgn*54,51),bc(ref)],1,'signal','C08')
# Fixed rear shell: two printed arcs (az ±92.5..180) at R152.5, 6.4 mm inside the bumper skin and inside the bumper end tip (R152.7;
# the shell start is ramped down to R151.5 over 3.5 deg, so the tip overlaps it by 1.2 mm), 2.8 mm wall with a 10 mm 45 deg inner top rim
# (stiffening ring), z 18..ZB1 (98.5). Nothing fixed ever rises above the bumper end: an object carried round a pressed half leaves the bumper end
# without meeting a fixed face, and one arriving from behind meets the bumper end ramp. A backer flange (R143..146, az ±88..94) closes the
# slot behind the bumper end. Foot ribs (per-side azimuths) bear on the base underside with 4 mm gussets up the wall to z ZGR (67.5) and take
# M3 × 10 thread-forming screws from below into P_base bosses; 3 mm vertical inner fins (6 mm deep, z 35..ZB1-14.5 (84), every 9-15 deg at
# azimuths chosen by a clearance scan) stiffen the wall against layer-peeling hits; a lap with two dowels (in 4 mm pads) at 180 deg.
RS_A0,RS_LEAD=92.5,(1.0,3.5)   # shell start (2 mm clear of the bumper end at full travel); lead-in ramp depth and azimuth length
def chamfer_lead():        # lead-in ramp on the outer start corner: R151.5 at the start to R152.5 3.5 deg later
    p1,p2=polar(R_RS-RS_LEAD[0],RS_A0),polar(R_RS,RS_A0+RS_LEAD[1])
    v=(p2[0]-p1[0],p2[1]-p1[1]);L=math.hypot(*v);v=(v[0]/L,v[1]/L);n=(v[1],-v[0])
    if n[0]*(p1[0]+p2[0])+n[1]*(p1[1]+p2[1])<0:n=(-n[0],-n[1])
    a,c=(p1[0]-2*v[0],p1[1]-2*v[1]),(p2[0]+.5*v[0],p2[1]+.5*v[1])
    return prism([a,c,(c[0]+4*n[0],c[1]+4*n[1]),(a[0]+4*n[0],a[1]+4*n[1])],ZB0-1,ZB1-ZB0+2)
SECR=[(0,ZB0),(RWALL,ZB0),(RWALL,ZB1-15.1),(RWALL+10,ZB1-4),(RWALL+10,ZB1-1.5),(RWALL+8.6,ZB1-1.5),(RWALL+1.2,ZB1-8.9),(RWALL,ZB1-8.9),(RWALL,ZB1),(0,ZB1)]   # 10 mm inner rim as a ~2.8 mm plate (underside 48 deg, 1.4 mm top land), open V groove above
RFL=(143.0,146.0,88,94)    # backer flange: radii and azimuths (left); >= 2 mm inside the bumper end at full travel
rs0=union(arc(SECR,R_RS,RS_A0,179.8).cut(chamfer_lead()),ring(RFL[0],RFL[1],ZB0,ZB1,RFL[2],RFL[3]),ring(RFL[0],R_RS_IN+.5,ZB0,ZB1,RS_A0,RFL[3]))
GUSR,ZGR=4.0,110.0-DZ      # rear rib gusset thickness and tip height on the wall (the top edge faces up: support-free at any slope); 31 mm under the top as before
def rib_solid(a):
    e=base_edge_r(a);r=prism(rib_poly(a,e-25),ZB0,29-ZB0)
    if R_RS_IN-e-1>=3:   # gusset up the wall where the rib reaches past the base edge
        r=union(r,cq.Workplane('XZ').polyline([(e+1,29),(R_RS_IN+.5,29),(R_RS_IN+.5,ZGR)]).close().extrude(GUSR/2,both=True).val().rotate((0,0,0),(0,0,1),a))
    for p in rib_screws(a):r=r.cut(union(cyl(*p,23.7,BOSSR+.3,6),cyl(*p,ZB0-1,3.1,3.45),cyl(*p,20.75,1.7,12)))   # boss pocket, Ø6.2 counterbore from below, 0.3 mm sacrificial bridge (print z 2.45-2.75), clearance
    return r
FIN_T,FIN_D,FIN_Z0=3.0,6.0,35.0   # inner stiffening fins: thickness, depth from the inner wall, wall-side bottom (45 deg lower end)
def fin(a):
    zi=ZB1-15.1+(FIN_D)*11.1/10+.4
    return cq.Workplane('XZ').polyline([(R_RS_IN+.5,FIN_Z0),(R_RS_IN-FIN_D,FIN_Z0+FIN_D+.5),(R_RS_IN-FIN_D,zi),(R_RS_IN+.5,ZB1-14.5)]).close().extrude(FIN_T/2,both=True).val().rotate((0,0,0),(0,0,1),a)
FIN_TARGETS=[104,113,131,140,165,174]
def pick_fins(sg,skip):   # per side: nearest azimuth to each target (±5 deg, >= 6 deg from a rib) whose fin keeps >= 1.5 mm to every other part and wire
    rows=[];chosen=[]
    for t in FIN_TARGETS:
        best=None
        for da in sorted(range(-5,6),key=abs):
            a0=t+da
            if any(abs(a0-abs(r_))<6 for r_ in RIBS['left' if sg>0 else 'right']) or a0>178:continue
            f=fin(sg*a0);g=min([(round(f.distance(shapes[i]),2),i) for i in shapes if i not in skip and near(f,shapes[i],2.)]+[(99.,'none within 2 mm')])
            if g[0]>=1.5:best=(sg*a0,g);break
        rows.append(dict(target_az=sg*t,chosen_az=best[0] if best else None,min_gap_mm=best[1][0] if best else None,nearest=best[1][1] if best else 'no clear azimuth within ±5 deg'))
        if best:chosen.append(best[0])
    return chosen,rows
DOWELS=[(183.5,36),(183.5,66)]   # L2: both inside the shorter lap (z 18..ZB1-17 = 81.5), pads z 31-41 and 61-71
lap=union(ring(R_RS_IN-2.5,R_RS_IN+.9,ZB0,ZB1-17,175,179.8),ring(R_RS_IN-2.5,R_RS_IN-.4,ZB0,ZB1-17,179.8,187))   # 0.4 mm radial lap clearance
for a,z in DOWELS:lap=union(lap,arc([(0,z-5),(2.0,z-3),(2.0,z+5),(0,z+5)],R_RS_IN-2.4,180.5,186.5))   # 2 mm inner pad (45 deg underside): 4 mm press length
for a,z in DOWELS:lap=lap.cut(tearh(a,z,1.55,R_RS_IN-5.6,R_RS_IN+.5))                   # Ø3.1 press fit in the tongue and pad
RSNOTE=(f'Fixed (unsensed) rear fender, R{R_RS} × {RWALL} mm wall with a 10 mm 45 deg inner top rim, z 18–{ZB1:g}, and {FIN_T:.0f} × {FIN_D:.0f} mm inner vertical fins (z {FIN_Z0:g}–{ZB1-14.5:g}, 45 deg lower ends). Print upright on the lower edge, no supports: foot ribs and '
 'gussets are solid to the bed, the Ø6.2 counterbores open on the bed face with a 0.3 mm sacrificial bridge (print z 2.45–2.75, two 0.15 mm or one 0.2 mm layer; the screw pierces it, or drill Ø3.4), dowel holes are teardrops. '
 f'Two foot ribs bear on the base underside; each takes two M3 × 10 pan-head thread-forming screws from below into Ø8 × 5 bosses on P_base (6.5 mm engagement), and carries a {GUSR:.0f} mm gusset up the wall to z {ZGR:.0f}. '
 f'The shell starts at az ±{RS_A0} just inside the bumper end, ramped from R{R_RS-RS_LEAD[0]} to R{R_RS} over {RS_LEAD[1]} deg, and a backer flange closes the slot behind the bumper end. '
 f'Slicer: 4 perimeters, PETG at about 245 °C with part cooling ≤ 30 % below print z {ZB1-ZB0-23:g} (layer adhesion), then a height-range modifier from print z {ZB1-ZB0-23:g}: fan 50–60 % and 50 % speed for the 45 deg inner rim; 5 mm brim on the arc ends plus mouse ears, reduced speed above print z {ZB1-ZB0-33:g}. '
 f'Release test: push at z {ZB1:g} with 30 N on the rear centre, no whitening. Never lift the robot by the shell.')
rsL=union(rs0,lap,*[rib_solid(a) for a in RIBS['left']])
FINS={}
FINS['left'],fin_rows_left=pick_fins(1,{'P_rear_shell_left'})
rsL=tidy(union(rsL,*[fin(a) for a in FINS['left']]))
add('P_rear_shell_left','Rear shell segment left (az 88–187 incl. flange and lap)',rsL,'print','F01','lower',printable=True,explode=[-50,40,0],note=RSNOTE+' Carries the 7 deg lap tongue (4 mm dowel pads) behind the right segment.',tess=(.35,.035))
rsR=union(rs0.mirror('XZ'),*[rib_solid(a) for a in RIBS['right']])
FINS['right'],fin_rows_right=pick_fins(-1,{'P_rear_shell_right'})
rsR=union(rsR,*[fin(a) for a in FINS['right']])
for a,z in DOWELS:rsR=rsR.cut(tearh(a,z,1.65,R_RS_IN-.5,R_RS+1))                     # Ø3.3 slip fit in the right segment (az 183.5 = -176.5 lies in the right segment)
add('P_rear_shell_right','Rear shell segment right (az -88 to -180)',tidy(rsR),'print','F01','lower',printable=True,explode=[-50,-40,0],note=RSNOTE,tess=(.35,.035))
for side,s in [('left',1),('right',-1)]:
    for j,a in enumerate(RIBS[side]):
        for k,p in enumerate(rib_screws(a)):tfs(f'rear_shell_screw_{side}_{j}{k}',*p,ZB0+.5,30.5,allow=('P_base','P_rear_shell_'+side),name='Rear shell M3 × 10 pan-head thread-forming screw')
DOWEL_L=8.0
for k,(a,z) in enumerate(DOWELS):
    add(f'rear_shell_dowel_{k}',f'Rear shell seam dowel Ø3 × {DOWEL_L:.0f}',cyl(0,0,0,1.5,DOWEL_L,'x').translate((R_RS-.1-DOWEL_L,0,z)).rotate((0,0,0),(0,0,1),a),'metal','H11','lower',check=True,allow=('P_rear_shell_left','P_rear_shell_right'),note='Ø3.1 press fit over 4 mm in the left lap tongue and its pad, Ø3.3 slip fit in the right segment wall (validation rear_shell.dowels); pressed in from outside, 0.1 mm under the shell surface.')

# Test coupon (not part of the robot, exported to print-parts/coupons and gated separately): one rail slot as modelled (file it to the
# 8.1-8.2 mm gauge width), one pin-C slot, one M2 switch slot, one Ø4.0 insert hole, one Ø2.6 × 8 pilot in an Ø8 boss (1 mm skin) and the
# rocker cut-out with its 1.5 mm panel.
cp=box(35,22.5,0,70,45,4)
cp=cp.cut(stadium((12-(BT/2-SPLAY),12),(12+(BT/2-SPLAY),12),SLOTW/2,-1,6)).cut(stadium((9.2,33),(14.8,33),3.1,-1,6)).cut(stadium((27,11.1),(27,12.9),1.2,-1,6)).cut(cyl(27,33,-1,INS_R,6))
cp=union(cp,cyl(38,22,3.99,BOSSR,5.01)).cut(cyl(38,22,1,1.3,9))
cp=cp.cut(box(57,22.5,-1,13.2,20.4,6)).cut(box(57,22.5,1.5,17.2,24.4,3))
assert cp.isValid()
if not STEP_ONLY:cq.exporters.export(cp,str(COUPONS/'P_coupon.stl'),tolerance=STL_TOL[0],angularTolerance=STL_TOL[1]);cq.exporters.export(cp,str(COUPONS/'P_coupon.step'))
COUPON=dict(id='P_coupon',name='Fit and tolerance test coupon (not on the robot)',file='print-parts/coupons/P_coupon.stl',size_mm=[70,45,9],
    features=[f'rail slot {SLOTW} wide (file to {SLOTW-.1:.1f}-{SLOTW}: a 693ZZ must roll on one side)','pin-C slot 6.2 wide','M2 switch slot 2.4 × 4.2','Ø4.0 insert hole (M3 × 3 insert, pressed flush on a steel plate)','Ø2.6 × 8 pilot in an Ø8 boss (M3 × 10 thread-forming screw)','rocker cut-out 13.2 × 20.4 with a 1.5 mm panel'])

# Export one named CAD assembly and the same geometry for browser/Blender/Unreal.
assembly=cq.Assembly(name='Home_Robot_R1')
def asm_add(p):   # a part into the STEP assembly: its render detail (RENDER) as a coloured sub-assembly, else its shape in its colour
    rgb=lambda c:cq.Color(*(int(c[i:i+2],16)/255 for i in (1,3,5)))
    if p['id'] not in RENDER:assembly.add(shapes[p['id']],name=p['id'],color=rgb(p['color']));return
    sub=cq.Assembly(name=p['id'])
    for k,(r,c) in enumerate(RENDER[p['id']]):sub.add(r,name=f"{p['id']}_{k}",color=rgb(c))
    assembly.add(sub)
if STEP_ONLY:   # same parts and colours as the loop below, without the meshes, JSON, validation and gates
    for p in parts:asm_add(p)
    assembly.save(str(ROOT/'cad'/'home-robot-r1.step'));print('STEP written:',ROOT/'cad'/'home-robot-r1.step',len(parts),'components');sys.exit(0)
scene=trimesh.Scene(); jsonparts=[]; MESH={}; MESHF={}
for p in parts:
    s=shapes[p['id']]; color=p['color'];rgb=tuple(int(color[i:i+2],16)/255 for i in (1,3,5))
    asm_add(p)
    if p['id'] in TESS:BRepTools.Clean_s(s.wrapped)   # re-mesh at the part's own (coarser/finer) tolerance
    vs,fs=s.tessellate(*TESS.get(p['id'],(.35,.35)))
    vertices=np.array([[v.x,v.y,v.z] for v in vs]);faces=np.array(fs);MESH[p['id']]=vertices;MESHF[p['id']]=faces   # MESH: the checked shape
    q=dict(p)
    if p['id'] in RENDER:   # drawn as the render detail: one mesh, a colour per vertex (palette index vpal in assembly-meshes.json)
        pal=sorted({c for _,c in RENDER[p['id']]});V_,F_,I_=[],[],[]
        for r,c in RENDER[p['id']]:
            rv,rf=r.tessellate(*TESS.get(p['id'],(.1,.2)));n0=sum(len(v) for v in V_)
            V_.append(np.array([[v.x,v.y,v.z] for v in rv]));F_.append(np.array(rf)+n0);I_+= [pal.index(c)]*len(rv)
        vertices,faces=np.vstack(V_),np.vstack(F_);q['palette']=pal;q['vpal']=I_
        _lin=lambda c:c/12.92 if c<=.04045 else ((c+.055)/1.055)**2.4   # glTF COLOR_0 is linear: the hex palette is sRGB
        vcol=np.array([[round(255*_lin(int(c[i:i+2],16)/255)) for i in (1,3,5)]+[255] for c in pal])[I_]
    else:vcol=np.tile(np.array([int(c*255) for c in rgb]+[255]),(len(vertices),1))
    mesh=trimesh.Trimesh(vertices=vertices/1000,faces=faces,process=False)
    mesh.visual.vertex_colors=vcol
    scene.add_geometry(mesh,node_name=p['id'],geom_name=p['id'])
    q['vertices']=[round(float(v),2) for v in vertices.reshape(-1)];q['faces']=[int(v) for v in faces.reshape(-1)]
    jsonparts.append(q)
assembly.save(str(ROOT/'cad'/'home-robot-r1.step'))
scene.export(str(ROOT/'cad'/'home-robot-r1.glb'))
(ROOT/'cad'/'assembly-meshes.json').write_text(json.dumps(jsonparts,separators=(',',':')))
(ROOT/'cad'/'component-manifest.json').write_text(json.dumps(parts,indent=2))
# Harness (B9): each lead that ends at the bay board gets its board end (connector ref, which end of the route), its cavity -> net list from board_spec
# NETS (so the harness is wired from the board's own netlist: J1 pin 1 = GND, pin 2 = BAT+, ruling PB20), the wire and suggested colours, and notes.
# The P03's two short leads into the screw terminals U5A / U5B are listed too (not drawn: they sit in the 1.9 mm between the terminals and the P03).
def pin_nets(ref):return {str(pn):net for net,pl in BS.NETS.items() for r_,pn in pl if r_==ref}
LEAD_FN={   # cut-list id: (board ref, end, wire, {pin: (function, colour)}, notes)
    'W01':('J1','last','2 × 18 AWG silicone (red, black)',{'1':('battery − (Tamiya −)','black'),'2':('battery + (Tamiya +)','red')},
           'Battery adapter lead: the Tamiya mate (C04) on the pack top to an AMASS XT30U-F (C19, the female half on the source side) on J1, the XT30 PCB male; tag it "J1 BATT". '
           'J1 (XT30), J2 (JST VH) and J3 (Micro-Fit 3.0) are three connector families (ruling PB30): this lead fits no other header. POLARITY, in plain words: on the board J1 pin 1 is GND and pin 2 is BAT+ '
           '(ruling PB20; silk "-" at pin 1, "+" at pin 2), so the black battery-minus wire goes on the XT30U-F\'s "−" contact (cavity 1, mates pin 1) and the red battery-plus wire on its "+" contact (cavity 2, mates pin 2). '
           'The XT30 mates only one way round, but a lead soldered the wrong way round still mates. Check it fail-safe before the fuses go in: '
           'with F1 and F2 out, plug W01 into J1 and the battery and measure F1\'s input clip (BAT+ side) against a GND pad: it must read positive, about the pack voltage (≥ 6 V; a part-discharged pack reads lower). A reversed lead reads negative and, with the fuses out, has no current path; '
           'with the fuses in it would put the battery backwards across the whole board, which has no reverse-polarity protection.'),
    'W02':('J2','last','2 × 18 AWG silicone (red, orange)',{'1':('to the master rocker (fused battery +)','red'),'2':('from the master rocker (switched battery +)','orange')},
           'Master rocker pair: JST VHR-2N (C05) on J2 to the rocker\'s two terminals (insulated spades, C12); tag it "J2 ROCKER". Both wires are battery + (fused / switched): never on GND. J2 is the board\'s only JST VH header (PB30).'),
    'W04':('J3','first','2 × 18 AWG silicone (red, black)',{'1':('5 V to the barrel plug centre','red'),'2':('GND to the barrel plug sleeve','black')},
           'Jetson 5 V lead: a Molex Micro-Fit 3.0 receptacle (43645-0200, two 43030-series 18 AWG crimps: C20) on J3 to the 5.5 / 2.1 mm barrel plug (C03), centre positive; tag it "J3 JETSON 5V". '
           'A Micro-Fit fits neither J1 (XT30) nor J2 (JST VH): ruling PB30. The receptacle plugs into J3 from beyond the board\'s right edge, its latch up. The Jetson needs its J48 jumper for barrel power.'),
    **{f'W{10+i}':(f'J{6+i}','last','5 × 26 AWG (red, black, blue, yellow, white)',{'1':('3.3 V to the sensor VIN','red'),'2':('GND','black'),'3':('SDA','blue'),'4':('SCL','yellow'),'5':(f'XSHUT{i+1}','white')},
           f'Cliff sensor {i+1} lead: JST XHP-5 (C06) on J{6+i}, soldered at the Pololu 2490 carrier (VIN, GND, SDA, SCL, XSHUT).') for i in range(4)},
    'W_bumper_left':('J10','last','2 × 26 AWG (white, black)',{'1':('BUMP_L: to the switch NC','white'),'2':('GND: to the switch COM','black')},'Left bumper switch lead: JST XHP-2 (C08) on J10; switch COM and NC, so an open wire reads as a hit.'),
    'W_bumper_right':('J11','last','2 × 26 AWG (white, black)',{'1':('BUMP_R: to the switch NC','white'),'2':('GND: to the switch COM','black')},'Right bumper switch lead: JST XHP-2 (C08) on J11; switch COM and NC, so an open wire reads as a hit.'),
    'W15':('J12','last','2 × 26 AWG (white, black)',{'1':('ARM: to the button','white'),'2':('GND: to the button','black')},'Arm button lead: JST XHP-2 (C08) on J12 to the normally-open button.'),
    'W14':('J13','last','2 × 24 AWG (red, orange)',{'1':('5 V to the motor toggle','red'),'2':('from the motor toggle to the relay coil +','orange')},'Motor-power toggle loop: JST XHP-2 (C08) on J13 to the toggle (insulated spades); opening it drops the relay.'),
    **{f'motor_pigtail_{s_}':(r_,'first','6 × 28 AWG silicone (red, black, green, blue, yellow, white)',{'1':('motor + (factory red)','red'),'2':('motor − (factory black)','black'),'3':('encoder GND (factory green)','green'),
           '4':('encoder VCC 5 V (factory blue)','blue'),'5':('encoder A (factory yellow)','yellow'),'6':('encoder B (factory white)','white')},
           f'Motor pigtail (C18) on {r_}: right-angle socket, wires up; colours as the factory cable so they run straight across the inline joint. Pin 1 (red) is marked with a paint-pen stripe on both halves of the joint. '
           f'Pre-cut {PIGTAIL[s_]["precut_mm"]} mm per wire, solder the socket end first, dress the bundle along its route, then trim each wire at its own cavity of the male housing and crimp (the wires fan out {PG["pin_spread_mm"]/2:g} mm either side of the bundle axis at both ends: at the socket\'s sleeves and at the housing\'s cavities); '
           f'tag the pigtail "{"L / J4" if s_=="left" else "R / J5"}" when it is made.') for s_,r_ in (('left','J4'),('right','J5'))},
    'W16':('PICO_USB','last','factory USB cable','', 'Pico USB cable: up-angled micro-B plug pushed in from above with the lid off.'),
}
for c in cables:
    if c['id'] in LEAD_FN:
        ref,end,wire_,fn,nt=LEAD_FN[c['id']]
        c.update(board_end=dict(ref=ref,end=end),wire=wire_,notes=nt)
        if ref in BS.EDGE_CONNECTORS:
            pm=pin_nets(ref);assert set(pm)==set(fn),(c['id'],pm,fn)
            c['pinout']=[dict(cavity=k,net=pm[k],function=fn[k][0],colour=fn[k][1]) for k in sorted(pm,key=int)]
# The P03 leads (not drawn): from the P03 terminal's two wire entries (their mid point) on its short end, out 2.5 mm, up and over the P03 top
# (z 49.5: over the 48 mm heatsink, under the carrier's 57) to just aft of the P03's aft face (x 8.2), down to the board terminal's wire entry.
# The HW-316E's terminals are on its short ends (IN at +y, OUT at -y), so these are not the ~10 mm face-to-face leads of the old envelope.
P03_LEADS={}
for ref,cid,side_txt,tm in (('U5A','W_p03_in','P03 input','IN'),('U5B','W_p03_out','P03 output','OUT')):
    cn=BCONN[ref];px=round(cn['courtyard_xy'][2],2);py=round(float(np.mean([q[1] for q in cn['pins_xy']])),2);zt=round(ZBAY_TOP+6,2)
    pm=pin_nets(ref);fn={'1':('to P03 IN+' if ref=='U5A' else 'from P03 OUT+','red'),'2':('to P03 IN−' if ref=='U5A' else 'from P03 OUT−','black')}
    e_=np.mean([P03_ENT[tm+'+'],P03_ENT[tm+'-']],axis=0).round(2);sy=1 if e_[1]>0 else -1;ye=sy*(P03S[1]/2+2.5)
    pts_=[tuple(float(v) for v in e_),(float(e_[0]),ye,float(e_[2])),(float(e_[0]),ye,49.5),(8.2,py,49.5),(8.2,py,zt),(px,py,zt)]
    L_=_polylen(pts_);P03_LEADS[cid]=dict(terminal=tm,end='+y (robot left)' if sy>0 else '-y (robot right)',entries_mm={k:[round(v,2) for v in P03_ENT[k]] for k in (tm+'+',tm+'-')},route_mm=round(L_,1))
    cables.append(dict(id=cid,name=f'{side_txt} leads (board {ref} screw terminal ↔ P03 {tm} terminal)',route_mm=round(L_),suggested_cut_mm=int(math.ceil((L_*1.2+40)/50)*50),points=pts_,modelled=False,
        board_end=dict(ref=ref,end='last'),wire='2 × 18 AWG silicone (red, black)',pinout=[dict(cavity=k,net=pm[k],function=fn[k][0],colour=fn[k][1]) for k in sorted(pm,key=int)],
        notes=f'Two leads between the P03\'s {tm} terminal (on its {"left (+y)" if sy>0 else "right (−y)"} short end, wire entries facing {"+y" if sy>0 else "−y"}; {tm}+ is the screw nearer the robot front) and the board\'s {ref} screw terminal (pin 1 = {pm["1"]}, pin 2 = GND): '
              f'about {L_:.0f} mm, out of the P03 end, up and over its top (under the Jetson carrier) and down just aft of it into {ref}. Not drawn; dress them at assembly. '
              + ('Battery side: a reversed P03 input is as costly as a reversed battery lead.' if ref=='U5A' else 'Set and check 5.1 V on the bench before these go on.')))
# Camera LED lead (task N, LIO-134; not drawn): soldered into the DNP spare row J15 (no header: a mated housing and its wire bend would
# stand about 17-20 mm, over the 16.27 mm under the Jetson carrier; soldered and laid flat it stays under 6 mm), along the board top to
# its front edge, the inline switch in heat-shrink in the front bay, then two wires along the camera cable W17 to a 2-pin JST-PH plug
# by the camera mount (so the camera unit comes out) and the LED carrier. Route length estimated from J15 to the LED.
_cy=BC['parts']['J15']['courtyard_xy'];_j15=[round((_cy[0]+_cy[2])/2,1),round((_cy[1]+_cy[3])/2,1),ZBAY_TOP]   # J15 centre (robot frame), board top
_led=PARAMS['camera_led_points'][0];_r=int(round(1.3*math.dist(_j15,_led)/10)*10)
cables.append(dict(id='W21',name='Camera LED lead (soldered into J15 → inline switch → JST-PH plug → LED above the camera lens)',route_mm=_r,
    suggested_cut_mm=int(math.ceil((_r*1.2+40)/50)*50),points=[tuple(_j15),tuple(_led)],modelled=False,
    wire='3 × 26 AWG silicone (white GP20 (SPARE3), red 5 V, black GND) from J15 to the inline switch, then 2 × 26 AWG (red, black) to the LED',
    pinout=[dict(cavity='3',net='SPARE3',function='GP20 (SPARE3): gate drive (through 100 Ω)',colour='white'),
            dict(cavity='5',net='5V',function='5 V to the 39 Ω resistor and the LED anode',colour='red'),
            dict(cavity='6',net='GND',function='GND: MOSFET source and the 100 kΩ pull-down',colour='black')],
    notes=f'Camera LED lead (task N, LIO-134; not drawn: route = 1.3 × the straight {round(math.dist(_j15,_led))} mm from J15 to the LED, an estimate). '
          'Three wires soldered into J15 holes 3, 5 and 6 from the top (J15 is a DNP pad row: no header, because a mated housing would not fit '
          'under the Jetson carrier), laid flat along the board top to its front edge (under 6 mm high). The inline switch (E21) sits in heat-shrink '
          'in the front bay, past the carrier\'s front edge; from it two wires follow the camera cable W17 to a 2-pin JST-PH plug (C21) by the camera '
          'mount, then run back along the sides of P_camera_led_shelf\'s block to the LED carrier (E20). Make it on the bench before the board goes in (BRINGUP.md section 10 item 6).'))
(ROOT/'cable-cut-list.json').write_text(json.dumps(cables,indent=2,ensure_ascii=False))


# ------------------------------------------------------------------------------------------------ checks
import time;T0=time.time()
byid={p['id']:p for p in parts}
def allowed(a,b):return b in ALLOW.get(a,()) or a in ALLOW.get(b,())
def sdist(a,b):   # minimum distance, guarded against a false 0 from BRepExtrema: its face classifier can accept an extremum on the untrimmed
    d=a.distance(b)   # surface of a large face (seen: the right bumper foam cylinder at s 2.5 vs the Jetson barrel jack, 37.3 mm apart). A 0 without
    if d<1e-6 and a.intersect(b).Volume()<1e-6:d=max(a.translate((e,0,0)).distance(b) for e in (.01,-.01))   # overlap volume is re-measured with a ±0.01 mm shift
    return d
def gaps(q,ids,m=6):   # sorted (gap, id) from shape q to the listed parts within m mm
    return sorted((round(sdist(q,shapes[i]),2),i) for i in ids if near(q,shapes[i],m))
NOW=lambda i:i.startswith(('W','motor_cable','motor_pigtail'))          # wires, harness sleeves and cable ties
SOLID=[p['id'] for p in parts if not NOW(p['id'])]
check=[]
for p in parts:
    if p['printable']:
        s=shapes[p['id']];r=p['export']['rot_z_deg'];b=(s.rotate((0,0,0),(0,0,1),r) if r else s).BoundingBox();dims=sorted([b.xlen,b.ylen])
        check.append(dict(part=p['id'],solid_valid=s.isValid(),size_mm=[round(b.xlen,2),round(b.ylen,2),round(b.zlen,2)],fits_mk3s=dims[1]<=250 and dims[0]<=210 and b.zlen<=210,
            fits_with_default_skirt=dims[1]+2*2.45<=250 and dims[0]+2*2.45<=210))
# Solid overlap scan over EVERY non-wire part (bought envelopes included); intended mates are listed with allow=.
clashes=[];intended=[]
import os;DEV=os.environ.get('R1_FAST')   # development switch: skips the two slow pairwise scans
for a,b in itertools.combinations([] if DEV else SOLID,2):
    aa,bb=byid[a]['bounds'],byid[b]['bounds']
    if all(min(aa[i+3],bb[i+3])-max(aa[i],bb[i])>0.05 for i in range(3)):
        v=shapes[a].intersect(shapes[b]).Volume()
        if v>.05:(intended if allowed(a,b) else clashes).append(dict(a=a,b=b,intersection_mm3=round(v,2)))
printed_overlaps=[c for c in clashes if c['a'].startswith('P_') or c['b'].startswith('P_')]
# Wires: every harness envelope against every solid part by MINIMUM GAP (>= 0.3 mm), except the parts holding one of its route
# points (terminals, connectors: within 2.5 mm of a route point). Gaps under 1 mm are listed for information.
WIRE_GAP=0.3
wire_clashes=[];wire_near=[]
for c in ([] if DEV else [c_ for c_ in cables if c_.get('modelled',True)]):
    w=shapes[c['id']]
    ends={i for i in SOLID if near(w,shapes[i],WIRE_GAP) and any(shapes[i].distance(cq.Vertex.makeVertex(*pt))<=2.5 for pt in c['points'])}-set(c.get('never_end',()))   # B9: parts a lead only passes are never its ends
    for i in SOLID:
        if i in ends or not near(w,shapes[i],1.0):continue
        d=w.distance(shapes[i])
        if d<WIRE_GAP:wire_clashes.append(dict(cable=c['id'],part=i,min_gap_mm=round(d,3),required_mm=WIRE_GAP))
        elif d<1.0:wire_near.append(dict(cable=c['id'],part=i,min_gap_mm=round(d,3)))
# Static printed parts: no undesigned near-contact (0 < gap < 1.0 mm) between any two of them (PETG print tolerance). The rear-shell
# lap (0.4 mm radial clearance by design) is the only listed exception.
PSTATIC=[p['id'] for p in parts if p['printable'] and not p['id'].startswith('P_bumper')]
printed_gaps=[]
for a,b in itertools.combinations([] if DEV else PSTATIC,2):
    if {a,b}=={'P_rear_shell_left','P_rear_shell_right'} or not near(shapes[a],shapes[b],1.0):continue
    d=shapes[a].distance(shapes[b])
    if 1e-3<d<1.0:printed_gaps.append(dict(a=a,b=b,gap_mm=round(d,3),required_mm=1.0))
print('overlap scans',round(time.time()-T0),'s',flush=True)

# Base: pairwise wall between every pair of P_base holes / pilots / slots (>= 1.2 mm).
base_walls=[]
for (n1,s1),(n2,s2) in itertools.combinations(BH,2):
    if n1.startswith('motor opening') or n2.startswith('motor opening') or not near(s1,s2,1.3):continue
    d=s1.distance(s2)
    if d<1.2:base_walls.append(dict(a=n1,b=n2,wall_mm=round(d,2)))
# Screw engagement: thread-forming screws need >= 5 mm in PETG; machine screws in the M3 × 3 brass inserts need >= 2.5 mm (5 threads)
# inside the insert; screws from below whose tips end in P_base keep them >= 0.8 mm inside its top face.
pilots=[]
for i in shapes:
    if not i.startswith(('rear_shell_screw','caster_screw','bay_screw','tray_screw','lid_post_screw')):continue
    b=shapes[i].BoundingBox();up=i.startswith(('rear_shell_screw','caster_screw'))
    tip=b.zmax if up else b.zmin;ins=[k for k in INSERTS if allowed(i,k)]
    if ins:z0=ins_z(INSERTS[ins[0]]['face']);kind,eng,req='M3 × 3 brass insert ('+INSERTS[ins[0]]['face']+' face)',(min(tip,z0+INS_L)-z0 if up else z0+INS_L-max(tip,z0)),2.5
    elif i.startswith('rear_shell_screw'):kind,eng,req='PETG boss, thread-forming',tip-24.0,5.0
    else:kind,eng,req='PETG, thread-forming',tip-33.0,5.0
    ok=eng>=req-1e-6 and (not up or tip<=33-.8)
    pilots.append(dict(screw=i,kind=kind,insert=ins[0] if ins else None,tip_z_mm=round(tip,2),engagement_mm=round(eng,2),required_engagement_mm=req,ok=ok))
for i in [k for k in shapes if k.startswith('lid_screw_')]:   # lid screws into the M3 × 3 inserts pressed flush into the post tops
    tip=shapes[i].BoundingBox().zmin;eng=ZLID-max(tip,ZLID-INS_L)
    pilots.append(dict(screw=i,kind='M3 × 3 brass insert (lid post top)',insert='insert_lid_top_'+i.rsplit('_',1)[1],tip_z_mm=round(tip,2),engagement_mm=round(eng,2),required_engagement_mm=2.5,ok=eng>=2.5-1e-6))
pilot_ok=all(p['ok'] for p in pilots)

# Bumper travel: each half moved along its own axis (30 deg inward of rearward); balls roll half the travel.
bumper=dict(travel_mm=BT,travel_dir_left=[round(v,4) for v in ft],travel_dir_right=[round(ft[0],4),round(-ft[1],4)],guide_angle_note='inward of rearward: left half moves along (-cos30, -sin30), right half along (-cos30, +sin30)',
            full_travel_shift_left_mm=[round(v,3) for v in BD[:2]],halves={},
            vertical_guide=dict(type='rolling Ø4 balls between 0.5 mm hardened spring-steel races, spring-preloaded',preload_per_pin_N=PRE_P,spring=SPR,stop_gap_mm=SPR['gap'],
                vertical_play_at_rest_mm=0.0,clamp_nut_to_rider_counterbore_mm=round(ZCB-35.8,2),
                tolerance_note='The half is pulled up against its three down balls by the pin springs, so its attitude is set by the base underside and three metal heights; '
                    'the printed base thickness (±0.15) and pocket depths (±0.1) move the rider up or down by at most ±0.25 mm, which only changes the spring length by the same amount '
                    '(preload ±0.5 N) because the 0.1 mm stop gap is set with a feeler at assembly. Rider-to-race running clearance 1.0 ± 0.25 mm; clamp nut to rider counterbore 0.4 ± 0.25 mm.'))
def mvec(side,s=BT):sg=1 if side=='left' else -1;return (s*ft[0],sg*s*ft[1],0)
MOVING=lambda side:('P_bumper_'+side,'bumper_foam_'+side,f'bumper_pin_{side}_',f'P_bumper_rider_{side}_',f'bumper_race_{side}_')
def moved(i,side,s):   # translation of part i when half 'side' is at travel s
    if i.startswith(MOVING(side)):return mvec(side,s)
    if i.startswith(f'bumper_ball_{side}_'):return mvec(side,s/2)
    return (0,0,0)
RACES=[i for i in shapes if i.startswith('base_race_')]
WIRES=[c['id'] for c in cables if c.get('modelled',True)]
STATES=[0.,1.25,2.5,3.75,5.0]
for side,sgn in [('left',1),('right',-1)]:
    M=(lambda s:s) if sgn>0 else (lambda s:s.mirror('XZ'))
    rigb=union(M(hguide),M(fillet),*([] if sgn>0 else [tongue]))   # rigid half without leaf and foam (its running gap to P_base)
    rig=union(rigb,M(foam))
    other_half='P_bumper_'+('right' if sgn>0 else 'left')
    others=[i for i in SOLID if not i.startswith(MOVING(side)+(f'bumper_ball_{side}_',)) and i!=other_half and not i.startswith('bumper_foam_') and i not in RACES]
    NOTG={'P_base','bumper_switch_'+side}   # running contacts: slots, abutment; the cam rolls on the switch roller
    rows={}
    for s_ in STATES:
        q=rig.translate(mvec(side,s_));g_=[x for x in gaps(q,others,15) if x[1] not in NOTG];gw=gaps(q,WIRES,8)
        lf=bent_leaf(sgn,s_);gl=[x for x in gaps(lf,others,6) if x[1] not in NOTG];lb=bent_leaf(sgn,s_,nose=False,root=False)
        rows[f's{s_}']=dict(min_gap_mm=g_[0][0] if g_ else None,nearest=[dict(part=k,gap_mm=v) for v,k in g_[:5]],wire_min_gap_mm=gw[0][0] if gw else None,wire_nearest=gw[0][1] if gw else 'none within 8 mm',
            leaf_min_gap_mm=gl[0][0] if gl else None,leaf_nearest=gl[0][1] if gl else 'none within 6 mm',rigid_half_gap_to_P_base_mm=round(rigb.translate(mvec(side,s_)).distance(shapes['P_base']),2),
            leaf_to_switch_mm=round(lf.distance(shapes['bumper_switch_'+side]),2),leaf_body_to_abutment_mm=round(lb.distance(abut(sgn)),2),
            nose_to_abutment_face_mm=round(cyl(*[c for c in bent_noseL(sgn,s_)],LZB[0],NR,LZB[1]-LZB[0]).distance(abut(sgn)),3))
    full=rig.translate(mvec(side))
    wheel=min((round(full.distance(shapes[k+'_'+side]),2),k+'_'+side) for k in ['tire','wheel','P_motor_mount','motor'])
    swbody=M(swW(union(box(0,0,22.6,19.8,10.2,6.4),box(0,-6.5,23.4,12,3.2,4.8))))
    pins={}
    mov_hw=[i for i in SOLID if i.startswith((f'bumper_pin_{side}_',f'P_bumper_rider_{side}_',f'bumper_ball_{side}_',f'bumper_race_{side}_'))]
    race_gap=(99.,'')
    for k in mov_hw:   # pins, bearings, spacers, riders, springs, balls, races, washers and nuts move with the half (balls half as far)
        row={}
        for s_ in STATES:
            q=shapes[k].translate(moved(k,side,s_));g_=[x for x in gaps(q,[i for i in others if i!='P_base'],6) if not allowed(k,x[1])];gw=gaps(q,WIRES,6)
            row[f's{s_}']=dict(min_gap_mm=g_[0][0] if g_ else None,nearest=g_[0][1] if g_ else 'none within 6 mm',wire_min_gap_mm=gw[0][0] if gw else None)
            for r_ in RACES:
                if r_.startswith(f'base_race_{side}_') and not allowed(k,r_) and near(q,shapes[r_],3):race_gap=min(race_gap,(round(q.distance(shapes[r_]),2),f'{k} / {r_} at s {s_}'))
        pins[k]=row
    for s_ in STATES:
        q=rigb.translate(mvec(side,s_))
        for r_ in RACES:
            if r_.startswith(f'base_race_{side}_') and near(q,shapes[r_],3):race_gap=min(race_gap,(round(q.distance(shapes[r_]),2),f'P_bumper_{side} / {r_} at s {s_}'))
    bumper['halves'][side]=dict(wheel_side_min_clearance_at_full_travel_mm=wheel[0],wheel_nearest=wheel[1],rigid_part_clearance=rows,
        gap_to_other_half_at_this_half_full_travel_mm=round(full.distance(shapes[other_half]),2),
        switch_body_gap_to_moving_half_mm={f's{s_}':round(swbody.distance(rig.translate(mvec(side,s_))),2) for s_ in STATES},
        rigid_part_overlap_with_P_base_mm3=round(rigb.intersect(shapes['P_base']).Volume(),3),
        leaf_preload_interference_with_P_base_mm3=round(M(bleaf).intersect(shapes['P_base']).Volume(),2),moving_hardware_clearances=pins,
        base_race_running_clearance_mm=race_gap[0],base_race_nearest=race_gap[1])
    h=bumper['halves'][side];h['switch_body_ok']=min(h['switch_body_gap_to_moving_half_mm'].values())>=1.0
    h['rigid_half_to_P_base_ok']=min(r['rigid_half_gap_to_P_base_mm'] for r in rows.values())>=1.0 and h['rigid_part_overlap_with_P_base_mm3']==0
    h['race_clearance_ok']=race_gap[0]>=.7
print('bumper clearances',round(time.time()-T0),'s',flush=True)
# Planar play: besides its travel s (0..5), each half may shift sideways e and yaw phi (about rail pin A) as far as the two rail-bearing
# slots allow (bearing centres inside the slot stadiums: axis s SPLAY..BT-SPLAY, radius SPLAY). Pin C never touches its slot.
A_PIN,B_PIN=np.array(RAILP[0],float),np.array(RAILP[1],float);FTv,NRv=np.array(ft),np.array(nrm)
YAW_ARM=abs(float((A_PIN-B_PIN)@FTv));PHMAX=2*SPLAY/YAW_ARM
def slot_ok(sv,ev):ds=max(0.,SPLAY-sv,sv-(BT-SPLAY));return ds*ds+ev*ev<=SPLAY*SPLAY+1e-9
def pin_b(sv,ev,ph):
    c,s_=math.cos(ph),math.sin(ph);dB=np.array([[c,-s_],[s_,c]])@(B_PIN-A_PIN)+A_PIN+sv*FTv+ev*NRv-B_PIN;return float(dB@FTv),float(dB@NRv)
def pose_ok(sv,ev,ph):return slot_ok(sv,ev) and slot_ok(*pin_b(sv,ev,ph))
POSES=[(float(sv),float(ev),float(ph)) for sv in np.round(np.arange(0,BT+1e-9,.25),3) for ev in np.linspace(-SPLAY,SPLAY,9) for ph in np.linspace(-PHMAX,PHMAX,13) if pose_ok(sv,ev,ph)]
POSES+=[(float(sv),float(ev),float(ph)) for sv in (.05,.1,4.9,4.95) for ev in np.linspace(-SPLAY,SPLAY,21) for ph in np.linspace(-PHMAX,PHMAX,41) if pose_ok(sv,ev,ph)]
def pose_xy(P,pose,sg=1):   # apply a left-frame pose to plan points P (n x 2) of side sg (the right half is the mirror image)
    sv,ev,ph=pose;Q=np.array(P,float)*[1,sg];c,s_=math.cos(ph),math.sin(ph)
    Q=(Q-A_PIN)@np.array([[c,s_],[-s_,c]])+A_PIN+sv*FTv+ev*NRv;return Q*[1,sg]
def pose_shape(S_,pose,sg=1):   # the same for a solid
    sv,ev,ph=pose;v=sv*FTv+ev*NRv
    return S_.rotate((A_PIN[0],sg*A_PIN[1],0),(A_PIN[0],sg*A_PIN[1],1),sg*math.degrees(ph)).translate((v[0],sg*v[1],0))
def extremes(svs):   # per travel: the feasible poses at the corners of the (side shift, yaw) region
    out=[]
    for sv in svs:
        P_=[p for p in POSES if abs(p[0]-sv)<1e-6] or [p for p in POSES if abs(p[0]-sv)<.06]
        if not P_:continue
        for key in (lambda p:p[1],lambda p:-p[1],lambda p:p[2],lambda p:-p[2],lambda p:p[1]+YAW_ARM*p[2],lambda p:-p[1]-YAW_ARM*p[2]):
            q=max(P_,key=key)
            if q not in out:out.append(q)
        if (sv,0.,0.) not in out and pose_ok(sv,0.,0.):out.append((sv,0.,0.))
    return out
bumper['planar_play']=dict(slot_width_max_mm=SLOTW,side_play_mm=SPLAY,yaw_arm_mm=round(YAW_ARM,1),max_yaw_deg=round(math.degrees(PHMAX),3),feasible_poses=len(POSES),
    pose='(s, e, phi): travel along the half axis, sideways shift along its normal, yaw about rail pin A; feasible when both bearing centres lie inside their slot stadiums',
    stop_range_mm=[round(min(p[0] for p in POSES if abs(p[1])<1e-9 and abs(p[2])<1e-9),3),round(max(p[0] for p in POSES),3)],
    hard_stop_note=f'The hard stop is at s = 5.0 with the bearing centred and {BT-SPLAY:.1f} mm with it on a slot side; rest is s = 0 (centred) to {SPLAY:.1f}.')
# Seam: minimum gap between the halves over the full grid of BOTH travels (t1 left, t2 right), then over the planar-play poses of both halves.
SC=box(152,0,ZB0-5,30,40,ZB1-ZB0+10)
bLc,bRc=union(hguide,foam).intersect(SC),union(hguide.mirror('XZ'),tongue,foam.mirror('XZ')).intersect(SC)
grid={f'{t1},{t2}':round(bLc.translate(mvec('left',t1)).distance(bRc.translate(mvec('right',t2))),2) for t1 in STATES for t2 in STATES}
gmin=min(grid,key=grid.get)
SEAMP=extremes([0.,2.5,4.0,4.5,4.75,4.9,4.95,5.0])
LPS={p:pose_shape(bLc,p,1) for p in SEAMP};RPS={p:pose_shape(bRc,p,-1) for p in SEAMP}
pgrid=sorted(((round(LPS[a].distance(RPS[b]),3),a,b) for a in SEAMP for b in SEAMP),key=lambda r:r[0])
bumper['seam']=dict(grid_gap_mm=grid,grid_note='key "t1,t2": left travel, right travel (mm), nominal (no side play)',min_gap_mm=grid[gmin],min_at_travels_mm=gmin,rest_gap_mm=grid['0.0,0.0'],
    left_full_travel_gap_mm=grid['5.0,0.0'],right_full_travel_gap_mm=grid['0.0,5.0'],both_full_travel_gap_mm=grid['5.0,5.0'],required_mm=1.5,
    with_planar_play=dict(poses_per_half=len(SEAMP),pairs=len(pgrid),min_gap_mm=pgrid[0][0],at=dict(left=[round(v,4) for v in pgrid[0][1]],right=[round(v,4) for v in pgrid[0][2]]),
        worst_five=[dict(gap_mm=g,left=[round(v,4) for v in a],right=[round(v,4) for v in b]) for g,a,b in pgrid[:5]]),
    tongue=f'Right-half backer: one flat step facing +x at x {XSTEP:.2f} across the seam (y -{G+.6:.2f}..+{G:.2f}), {SEAM_C} mm (design) from the region swept by the left end inner corner over every (t1, t2), joined to the right wall by a web behind the right end.')
bumper['seam']['ok']=bumper['seam']['min_gap_mm']>=1.5 and pgrid[0][0]>=1.5
bumper['seam_gap_one_half_pushed_mm']=bumper['seam']['left_full_travel_gap_mm']
# Mesh ray caster (Moller-Trumbore) for rod probes.
class Caster:
    def __init__(s,ids,shift={},rmin=0.,pose=None):
        T=[];I=[]
        for i in ids:
            t=MESH[i][MESHF[i]]+np.array(shift.get(i,(0,0,0)))
            if pose is not None and i in pose:sg_,pz=pose[i];t=np.concatenate([pose_xy(t.reshape(-1,3)[:,:2],pz,sg_),t.reshape(-1,3)[:,2:]],1).reshape(-1,3,3)
            if rmin>0:t=t[np.hypot(t[:,:,0],t[:,:,1]).max(1)>=rmin]
            T.append(t);I+=[i]*len(t)
        s.T=np.concatenate(T);s.I=np.array(I);s.lo=s.T.min(1);s.hi=s.T.max(1)
        a=np.degrees(np.arctan2(s.T[:,:,1],s.T[:,:,0]));am=a.mean(1,keepdims=True);d=(a-am+180)%360-180
        s.a0=(am[:,0]+d.min(1));s.a1=(am[:,0]+d.max(1));s.rlo=np.hypot(s.T[:,:,0],s.T[:,:,1]).min(1)
    def hits(s,o,d,k):   # first hit distance along each ray (o: n x 3, d: 3) among triangles k
        if not len(k):return np.full(len(o),np.inf),np.full(len(o),-1)
        T=s.T[k];v0=T[:,0];e1=T[:,1]-v0;e2=T[:,2]-v0;p=np.cross(d,e2);det=(e1*p).sum(1);ok=np.abs(det)>1e-12;inv=np.where(ok,1/np.where(ok,det,1),0)
        tv_=o[:,None,:]-v0[None];u=(tv_*p[None]).sum(2)*inv;q=np.cross(tv_,e1[None]);v=(q*d).sum(2)*inv;t=(e2[None]*q).sum(2)*inv
        h=ok[None]&(u>=-1e-9)&(v>=-1e-9)&(u+v<=1+1e-9)&(t>1e-9);t=np.where(h,t,np.inf);j=t.argmin(1)
        return t[np.arange(len(o)),j],np.where(np.isfinite(t[np.arange(len(o)),j]),k[j],-1)
    def normal(s,j,d):   # unit normal of triangle j, facing against the ray direction d
        T=s.T[j];n=np.cross(T[1]-T[0],T[2]-T[0]);n=n/np.linalg.norm(n);return -n if n@d>0 else n
    def sel(s,z0,z1,a=None,da=3.):
        m=(s.lo[:,2]<=z1)&(s.hi[:,2]>=z0)
        if a is not None:m&=(((a-s.a0+180)%360-180>=-da)&((s.a1-a+180)%360-180>=-da))|(s.rlo<40)
        return np.flatnonzero(m)
ROD=[(0,0),(1.5,0),(-1.5,0),(0,1.5),(0,-1.5),(1.06,1.06),(-1.06,1.06),(1.06,-1.06),(-1.06,-1.06)]   # Ø3 flat-ended rod: (lateral, vertical) samples
def rod_probe(C,a,z,start=200.):   # rod pushed radially inward at azimuth a, centre height z: (intrusion past the skin circle, first part)
    c,s_=math.cos(math.radians(a)),math.sin(math.radians(a));u=np.array([-c,-s_,0.]);n=np.array([-s_,c,0.])
    O=np.array([[start*c+lo*n[0],start*s_+lo*n[1],z+dz] for lo,dz in ROD]);k=C.sel(z-1.6,z+1.6,a)
    t,j=C.hits(O,u,k);m=int(np.argmin(t))
    return (round(float(t[m]-(start-R_SKIN)),2),C.I[j[m]]) if np.isfinite(t[m]) else (999.,'none')
# Seam intrusion: a round Ø3 rod pushed straight in (-x) at the seam, rod centres y -4..+4, at rest and with either or both halves already
# pushed to full travel. The face normal at the first hit sets the drive: the row trips only when (-n).t - 0.5 |(-n).normal_of_travel| >= 0.3
# (object friction 0.5); the rod then advances (t.n)/(d.n) per mm of travel and the switch makes at 2.25 mm of travel (calibration).
SEAMIDS=['P_bumper_left','P_bumper_right','bumper_foam_left','bumper_foam_right']
def seam_rods(shift,pushed=()):
    C=Caster(SEAMIDS,shift);out=[];dr=np.array([-1.,0,0])
    for z in [20,40,100-DZ,110-DZ,CH,85,90,ZS0-1.5]:   # z 67.5-85: through the camera window (no bumper part: 'no contact', camera_window lists them)
        for yc in np.arange(-4,4.01,.5):
            O=np.array([[200,yc+lo,z+dz] for lo,dz in ROD]);t,j=C.hits(O,dr,C.sel(z-1.6,z+1.6))
            m=int(np.argmin(t));xm=200-t[m]
            if not np.isfinite(t[m]):out.append(dict(z=round(z,2),rod_y=round(float(yc),2),first_contact='none',verdict='no contact'));continue
            prt=C.I[j[m]];n=C.normal(j[m],dr);sg_=1 if prt.endswith('left') else -1;tt=np.array([ft[0],sg_*ft[1],0]);nn=np.array([nrm[0],sg_*nrm[1],0])
            drive=float(-n@tt);side_=abs(float(-n@nn));eff=drive-.5*side_;adv=float((tt@n)/(dr@n)) if abs(dr@n)>1e-6 else 99.
            held=prt.endswith(pushed) if pushed else False   # that half is already at its hard stop: its switch is made
            trips=eff>=.3 or held;intr=float(R_SKIN-xm)
            out.append(dict(z=round(z,2),rod_y=round(float(yc),2),first_contact=prt,contact_x_mm=round(float(xm),2),face_normal=[round(float(v),3) for v in n],drive=round(drive,3),drive_with_friction_0p5=round(eff,3),
                rod_advance_per_mm_travel=round(adv,3),intrusion_mm=round(intr,2),intrusion_before_trip_mm=(round(intr,2) if held else round(intr+SCAL_*adv,2)) if trips else None,verdict=('sensed: that half is already at its hard stop' if held else 'trips') if trips else 'rigid (unsensed) contact'))
    return out
SCAL_=2.25
SH=lambda *sides:{i:mvec(sd) for sd in sides for i in ('P_bumper_'+sd,'bumper_foam_'+sd)}
rods=seam_rods({});rcases={k:seam_rods(v,p_) for k,v,p_ in [('left_full',SH('left'),('left',)),('right_full',SH('right'),('right',)),('both_full',SH('left','right'),('left','right'))]}
rigid=[r for r in rods if r['verdict'].startswith('rigid')]
bumper['seam_intrusion']=dict(rod='round Ø3 flat-ended rod, 9 ray samples over its section, centres y -4..+4, pushed along -x',rows=rods,
    max_intrusion_before_trip_mm=max(r['intrusion_before_trip_mm'] for r in rods if r.get('intrusion_before_trip_mm') is not None),
    max_contact_intrusion_mm=max(r['intrusion_mm'] for r in rods if 'intrusion_mm' in r),rigid_rows=rigid,
    pushed_cases={k:dict(max_contact_intrusion_mm=max(r['intrusion_mm'] for r in v if 'intrusion_mm' in r),first_contacts=sorted({r['first_contact'] for r in v}),rigid_rows=[r for r in v if r['verdict'].startswith('rigid')]) for k,v in rcases.items()},
    trip_rule='(-n).t - 0.5 |(-n).normal| >= 0.3 at the first contact face; rod advance per mm of travel = (t.n)/(d.n); trip at 2.25 mm of travel',
    limit_note='A rod narrower than the 7.5 mm seam passes between the seam-end ramps and meets the backer step (normal +x, drive 0.87), which must stay 2.3 mm (design) behind the region the left end sweeps: '
               'that sets the deepest unsensed intrusion of thin objects at the seam (max_intrusion_before_trip_mm). Wider objects meet a ramp or the skin first.')
bumper['seam_intrusion']['ok']=not rigid and not any(c['rigid_rows'] for c in bumper['seam_intrusion']['pushed_cases'].values())
# Leaf spring: constant-strength (parabolic) taper, bends perpendicular to its axis; free length from the root boss edge (PETG E ~2.0 GPa).
LH=LZB[1]-LZB[0]
def leaf_rate(E=2000.,dt=0.):
    qq=np.linspace(LRB,LL,800);tq=np.array([tq_(q,dt) for q in qq]);Iq=LH*tq**3/12
    return 1/float(np.trapezoid((LL-qq)**2/(E*Iq),qq)),(lambda F:float(np.max(6*F*(LL-qq)/(LH*tq**2))))
kl,sig=leaf_rate();dfl=PL+BT;klw,_=leaf_rate(1700.,-.05)
CREEP=.35
NOSE=[(s_,)+leaf_nose(s_) for s_ in np.round(np.arange(0,BT+1e-9,.25),2)]
dnose=lambda pz:leaf_nose(pz[0])[0]+pz[1]+pz[2]*float(np.array([-(LR0[1]-A_PIN[1]),LR0[0]-A_PIN[0]])@np.array(lax))   # nose contact along the leaf axis (side shift moves it 1:1, yaw about pin A)
dn=[dnose(p) for p in POSES];DNMIN,DNMAX=min(dn),max(dn)
bumper['leaf_spring']=dict(length_to_nose_mm=LL,free_length_from_boss_edge_mm=LL-LRB,height_mm=LH,thickness_mm=dict(at_boss_edge=LTR,minimum=LT,law=f't = {LTR} sqrt(({LL:.0f} - q) / {LL-LRB:.0f}), >= {LT} (constant bending stress)'),root_fillet_mm=1.5,root=LR0,nose=[round(v,2) for v in LT0],nose_boss_diameter_mm=2*NR,
    preload_interference_mm=PL,travel_axial_component_pct=round(100*abs(ft[0]*lax[0]+ft[1]*lax[1]),2),estimated_rate_N_per_mm=round(kl,3),force_at_rest_N=round(kl*PL,2),force_at_2p25mm_N=round(kl*(PL+2.25),2),force_at_hard_stop_N=round(kl*dfl,2),
    peak_stress_MPa_at_hard_stop=round(sig(kl*dfl),1),stress_MPa_at_rest=round(sig(kl*PL),1),peak_strain_pct_at_hard_stop=round(100*sig(kl*dfl)/2000,2),stress_limit_MPa=15.0,
    creep_relaxation_assumed=CREEP,force_at_rest_after_creep_N=round(kl*PL*(1-CREEP),2),required_after_creep_N=0.5,
    worst_case=dict(E_MPa=1700,thickness_minus_mm=.05,rate_N_per_mm=round(klw,3),force_at_rest_N=round(klw*PL,2),force_at_rest_after_creep_N=round(klw*PL*(1-CREEP),2)),
    previous_designs=dict(round_1=dict(rate_N_per_mm=.563,force_at_rest_N=.68,force_at_hard_stop_N=3.49),round_3=dict(rate_N_per_mm=.332,force_at_rest_N=.80,force_at_hard_stop_N=2.46,length_mm=44,thickness_mm=[2.1,1.25])),
    nose_contact=dict(method='deflected constant-strength leaf integrated with large slopes; the projected length shortens as it bends, pulling the Ø4 nose toward the root; side play moves it 1:1 along the leaf axis, yaw about pin A a little more',
        by_travel=[dict(s=s_,nose_d_mm=round(d_,3),tip_slope_deg=round(a_,1)) for s_,d_,a_ in NOSE],contact_d_range_with_play_mm=[round(DNMIN,3),round(DNMAX,3)],
        face_d_range_mm=[ABD[0]+1,ABD[1]],root_side_lead='45 deg, 1 mm',required_inside_face_mm=1.0,margin_root_side_mm=round(DNMIN-(ABD[0]+1),3),margin_tip_side_mm=round(ABD[1]-DNMAX,3)),
    note='Tip-loaded cantilever integrated over the parabolic section from the root-boss edge (the one-sided R1.5 fillet adds a little stiffness and is ignored). Printed upright with the half, so the leaf bends within its layers; the Ø4 nose bears on a P_base abutment whose face is normal to the travel. '
         '35 % relaxation is assumed for PETG at the 5 MPa rest stress over the product life. The worst case (E 1700, leaf 0.05 mm thin, 35 % creep) still returns the half with >= 0.5 N.')
LS=bumper['leaf_spring'];LS['nose_contact']['ok']=LS['nose_contact']['margin_root_side_mm']>=1.0 and LS['nose_contact']['margin_tip_side_mm']>=1.0
LS['axial_ok']=LS['travel_axial_component_pct']<10
LS['ok']=LS['axial_ok'] and LS['peak_stress_MPa_at_hard_stop']<=15.0 and LS['force_at_rest_after_creep_N']>=0.5 and LS['worst_case']['force_at_rest_after_creep_N']>=.5 and LS['nose_contact']['ok']
# Switch: roller-top height vs travel from the cam profile (switch frame), round roller, with lateral offsets of the half.
def roller_top(s,lat=0.,s0=S0):
    P=cpts(s0);X=np.linspace(XR-9,XR+6.6,3000);Y=np.interp(X,[p[0] for p in P],[p[1] for p in P])+lat
    dx=X-s-XR;k=np.abs(dx)<=RR
    return float(np.min(Y[k]-np.sqrt(RR*RR-dx[k]**2)))+RR if k.any() else 99.
SS=np.arange(0,5.001,.01);RT0=np.array([roller_top(s) for s in SS])
def sw_case(lat,yop=YOP):
    t=RT0+lat;op=float(SS[np.argmax(t<=yop)]) if (t<=yop).any() else None   # a lateral shift moves the whole cam line along the roller axis
    return dict(lateral_offset_mm=round(lat,3),roller_top_at_rest=round(t[0],2),release_margin_at_rest_mm=round(t[0]-(yop+MDM),2),operates_at_travel_mm=op,past_OP_at_hard_stop_mm=round(yop-t[-1],2))
SCAL=2.25   # set at assembly: operates at 2.25 mm with the half held on the roller-biased slot side
YCAL=float(RT0[int(round(SCAL/.01))])
REST=sw_case(0.,YCAL)   # at rest the roller force holds the bearing on its biased side: the calibrated position
def sw_ok(c):return c['operates_at_travel_mm'] is not None and abs(c['operates_at_travel_mm']-2.0)<=.5 and REST['release_margin_at_rest_mm']>=.2 and .3<=c['past_OP_at_hard_stop_mm']<=OTM-.1
cases=[sw_case(l,YCAL) for l in (0.,-.05,-.1,-.15,-.2)]
slot_cases=[]
for w in [8.1,8.15,SLOTW]:   # as-filed rail-slot width: side play p; the bearing can sit on either side during a hit (2p from calibration)
    p_=round((w-8.0)/2,3);cc=[sw_case(0.,YCAL),sw_case(-2*p_,YCAL)]
    slot_cases.append(dict(slot_width_mm=w,side_play_mm=round(p_,2),bearing_contact='one side at a time: rolls' if p_>=.05 else 'both sides: slides or binds (ream the slot)',
        operates_at_travel_range_mm=[cc[1]['operates_at_travel_mm'],cc[0]['operates_at_travel_mm']],release_margin_at_rest_mm=REST['release_margin_at_rest_mm'],
        past_OP_at_hard_stop_max_mm=max(c['past_OP_at_hard_stop_mm'] for c in cc),ok=p_>=.05 and all(sw_ok(c) for c in cc)))
bumper['switch']=dict(model='Omron SS-5GL2 (hinge roller lever, Ø4.8 roller on a 14.5 mm lever)',datasheet=dict(FP_max_mm=19.3,OP_mm='14.5 ± 0.8',OT_min_mm=OTM,MD_max_mm=MDM,OF_max_N=.49,source='Omron SS subminiature basic switch datasheet (en-ss.pdf), hinge roller lever table; heights of the roller top above the mounting-hole line'),
    calibration=f'OP tolerance (±0.8 mm) exceeds OT min (1.2 mm), so the switch slides ±0.9 mm in slotted P_base holes and is set at assembly to operate at {SCAL} mm of travel (gauge block at the rest stop) while the roller force holds the bearing on its slot side. '
                'During a hit the bearing may sit on the other side of the slot (2 × side play), which makes the switch operate earlier, never later.',
    cam=dict(ramp_deg=40,ramp_line_start_past_roller_centre_mm=S0,roller_meets_ramp_at_travel_mm=float(SS[np.argmax(YREST-RT0>.01)]),roller_top_rest=YREST,roller_top_dwell=YDW,nominal_OP=round(YOP,2),calibrated_OP=round(YCAL,3),
        note='The round Ø4.8 roller meets the 40 deg ramp line almost at once (lever moves from about s 0); there is no dead band.'),
    lever_deflection_vs_travel_mm=[(round(float(s),2),round(YREST-roller_top(s),3)) for s in np.arange(0,5.01,.25)],cases=cases,slot_width_cases=slot_cases,
    calibrated_play_cases_ok=all(sw_ok(c) for c in cases if c['lateral_offset_mm']>=-2*SPLAY-1e-9),slot_width_cases_ok=all(c['ok'] for c in slot_cases),
    required_as_filed_slot_width_mm=[8.1,SLOTW],
    planform_gap_to_tower_screws_mm=min(round(shapes['bumper_switch_left'].distance(shapes[f'tower_screw_{i}']),2) for i in range(4)),
    roller_force_on_half='At most OF 0.49 N along +Y (the half\'s +nrm direction): a light, constant side bias.',
    dog_direction_note='The cam approaches the roller from the lever-tip side (cam normal 54-62 deg from the roller path); the Omron SS datasheet text available has no dog-direction rule: confirm with Omron\'s basic-switch precautions before release.')
bumper['switch']['operates_by_travel_mm']=sw_case(0.,YCAL)['operates_at_travel_mm'];bumper['switch']['operates_ok']=bumper['switch']['calibrated_play_cases_ok'] and bumper['switch']['slot_width_cases_ok']
print('switch/seam/leaf',round(time.time()-T0),'s',flush=True)

# Preload balance: the three pin springs pull the half up (PRE_P each at the pins), its weight acts at its centre of mass, the leaf
# and the switch roller add small couples; the three down balls must all stay loaded (>= 0.5 N) for the attitude to be fixed at rest.
HALF=['P_bumper_left','bumper_foam_left']+[i for i in shapes if i.startswith(('bumper_pin_left_','bumper_race_left_d','bumper_ball_left_d'))]
RHO=lambda i:(1.27 if i.startswith('P_') else (.07 if 'foam' in i else 7.85))*1e-6
mh=sum(shapes[i].Volume()*RHO(i) for i in HALF);gc=[sum(shapes[i].Volume()*RHO(i)*getattr(shapes[i].Center(),a) for i in HALF)/mh for a in 'xyz'];Wh=mh*9.81
PINS=RAILP+[PB];u2=np.array(ft)
Fl=kl*PL;Ml=Fl*(31-(LZB[0]+LZB[1])/2)*np.array([-u2[1],u2[0]])   # leaf couple (abutment on the nose vs the rest stops at z 31)
Mr=.49*(25.8-31)*np.array([-nrm[1],nrm[0]])                       # roller side force couple (worst OF, +nrm at the roller, reacted at the bearings)
A3=np.array([[1,1,1],[p[1] for p in BALL_DN],[-p[0] for p in BALL_DN]],float)   # rows: sum R, sum R y, -sum R x of the down-ball reactions (they push the half down)
bvec=np.array([3*PRE_P-Wh,sum(PRE_P*p[1] for p in PINS)-Wh*gc[1]+Ml[0]+Mr[0],-(sum(PRE_P*p[0] for p in PINS)-Wh*gc[0])+Ml[1]+Mr[1]])
Rd=np.linalg.solve(A3,bvec)
VPRE=np.concatenate([Rd,[PRE_P/2]*6])   # preload contact loads: down balls, then rider balls (each spring splits over its two balls)
bumper['preload_balance']=dict(moving_half_mass_kg=round(mh,4),moving_half_com_mm=[round(v,1) for v in gc],spring_preload_per_pin_N=PRE_P,down_ball_reactions_N=[round(float(v),2) for v in Rd],
    rider_ball_loads_N=PRE_P/2,required_min_N=0.5,ok=bool(Rd.min()>=.5),note='The half hangs on three springs that pull it up against three down balls; at rest it cannot tilt inside any clearance. Riders and their balls rest on the base, not on the half.')
# Tilt only under load: the half may lower at a pin until that spring meets its stop tube (0.3 mm), and rise at a pin until the clamp nut meets
# the rider counterbore (0.8 mm); it cannot rise at a down ball. Vertices of that set give the largest outward skin motion under a hit.
row3=lambda x,y:np.array([1,y,-x])
TC=[(row3(*p),0.) for p in BALL_DN]+[(-row3(*p),SPR['gap']) for p in PINS]+[(row3(*p),ZCB-35.8) for p in PINS]
TV=[]
for t3 in itertools.combinations(range(len(TC)),3):
    Am=np.array([TC[i][0] for i in t3]);b_=np.array([TC[i][1] for i in t3])
    if abs(np.linalg.det(Am))<1e-9:continue
    q=np.linalg.solve(Am,b_)
    if all(a@q<=bb+1e-9 for a,bb in TC):TV.append(q)
def tilt_disp(q,x,y,z,zr=ZBU):w0,wx,wy=q;return (wy*(z-zr),-wx*(z-zr),w0+wx*y-wy*x)
SKINPTS=[(R_SKIN,a,z) for a in np.arange(1.5,AZF+.01,2) for z in (26,(26+ZFT)/2,ZFT)]+[(R_SH,a,ZB1) for a in np.arange(0,AZT+.01,2)]+[(R_SH-(R_SH-R_TIP)*(a-AZT)/(90-AZT),a,z) for a in np.arange(AZT,90.01,2) for z in (ZB0,(ZB0+ZB1)/2,ZB1)]
def tilt_r(shift=(0,0)):
    best=(0.,None)
    for q in TV:
        for r,a,z in SKINPTS:
            x,y=polar(r,a);dx,dy,_=tilt_disp(q,x,y,z);rr=math.hypot(x+shift[0]+dx,y+shift[1]+dy)
            if rr>best[0]:best=(rr,(round(float(a),1),z))
    return best
tr0,tr5=tilt_r(),tilt_r(BD[:2])
seam_dx=max(math.hypot(*tilt_disp(q,CL[0],CL[1],z)[:2]) for q in TV for z in (ZB0,(ZB0+ZS0)/2,ZS0)) if TV else 0.
lid_rise=max(tilt_disp(q,*polar(R_SH-WALL-6,a),ZB1)[2] for q in TV for a in np.arange(0,90.1,5)) if TV else 0.
bumper['tilt_under_load']=dict(vertices=len(TV),max_skin_r_mm_rest_pose=round(tr0[0],2),at=tr0[1],max_skin_r_mm_full_travel=round(tr5[0],2),max_horizontal_shift_at_seam_mm=round(seam_dx,2),
    max_rim_rise_mm=round(lid_rise,2),seam_min_gap_with_both_halves_tilted_mm=round(bumper['seam']['min_gap_mm']-2*seam_dx,2),
    note='Only under a load larger than the preload (a hit high on the skin): the half tilts within the 0.1 mm stop gap and the 0.4 mm clamp-nut gap. At rest the springs hold the nominal pose.')
bumper['tilt_under_load']['ok']=tr0[0]<=160.0 and tr5[0]<=160.0 and bumper['tilt_under_load']['seam_min_gap_with_both_halves_tilted_mm']>=0.5 and lid_rise<1.5

# Friction lock: can a hit at (point, height) slide the half against the guide friction?
# Lateral load and yaw go to the two 693ZZ rail-pin bearings (pin C has side play); pitch and roll go to the Ø4 balls
# (down: rail/arm-top balls under the base; up: rider balls on the base top). The vertical forces are shared as a compliant
# body would share them (equal contact stiffness, unilateral: the minimum-norm admissible set), not the cheapest set. This is the
# large-hit limit (the preload is small against a hit that reaches the stop gap); the preload adds a constant rolling drag.
A_,B_=np.array(RAILP[0],float),np.array(RAILP[1],float);FT,NR=np.array(ft),np.array(nrm)
VC=[(*p,ZBU,-1) for p in BALL_DN]+[(*pp(c,sg*RIDE[k]),ZBT,1) for k,c in zip('ABC',RAILP+[PB]) for sg in RSIDES[k]]
VA=np.array([[c[3],c[1]*c[3],-c[0]*c[3]] for c in VC])
def vsolve(b):   # V_i = max(0, a_i . lam) with sum V_i a_i = b: Newton on the convex potential
    lam=np.linalg.solve(VA.T@VA+1e-9*np.eye(3),b);g=lambda l:.5*np.sum(np.maximum(0,VA@l)**2)-b@l
    for _ in range(80):
        act=VA@lam>0;r=b-VA.T@np.maximum(0,VA@lam)
        if np.linalg.norm(r)<1e-9:break
        J=VA[act].T@VA[act]+1e-9*np.eye(3);d=np.linalg.solve(J,r);st=1.
        while st>1e-6 and g(lam+st*d)>g(lam)-1e-4*st*(r@d):st*=.5
        lam=lam+st*d
    return np.maximum(0,VA@lam)
def gsolve(H,F,mu_l,mu_v,it=6,full=False,slide=None):   # (drive along the travel after friction, lateral sum, vertical sum) per the push F; slide: mu of the single most loaded ball (seized, sliding)
    H=np.array(H,float);F=np.array(F,float);fr=[];mz=lambda p,f:p[0]*f[1]-p[1]*f[0]
    for _ in range(it):
        Mz=mz(H,F)+sum(mz(p,-m*FT) for p,m in fr)
        RA,RB=np.linalg.solve(np.array([[1,1],[mz(A_,NR),mz(B_,NR)]]),[-(F@NR),-Mz])
        hor=[(H,F),((*A_,31),RA*NR),((*B_,31),RB*NR)]+[(p,-m*FT) for p,m in fr]
        Mx=sum(-p[2]*f[1] for p,f in hor);My=sum(p[2]*f[0] for p,f in hor)
        V=vsolve(np.array([0,-Mx,-My]))
        km=int(np.argmax(V)) if slide is not None else -1
        fr=[((*A_,31),mu_l*abs(RA)),((*B_,31),mu_l*abs(RB))]+[((c[0],c[1],c[2]),(slide if kk==km else mu_v)*v) for kk,(c,v) in enumerate(zip(VC,V))]
    out=(float(F@FT-sum(m for _,m in fr)),float(abs(RA)+abs(RB)),float(V.sum()))
    return out+(V,abs(RA),abs(RB)) if full else out
def skin_r(t):return R_SKIN if t<=AZF else (R_SH if t<=AZT else R_SH-(R_SH-R_TIP)*(t-AZT)/(90-AZT))
def ztop(t):   # bumper top at the skin point of azimuth t (the seam band's 1.2 mm outer chamfer, then full height)
    return ZS0 if t<5.5 else ZB1
def in_win(t,z):   # the skin point at azimuth t, height z lies in the camera window (no bumper surface there)
    x,y=polar(skin_r(t),t);b,tp=win_z(x);return abs(y)<=win_hw(x) and b<=z<=tp
def hit(t,z,muc):   # hit point and force (unit normal push plus object-skin friction opposing the skin's slip)
    r=skin_r(t);n=np.array([-math.cos(math.radians(t)),-math.sin(math.radians(t))]);tg=FT-(FT@n)*n
    tg=tg/np.linalg.norm(tg) if np.linalg.norm(tg)>1e-9 else tg
    return (r*math.cos(math.radians(t)),r*math.sin(math.radians(t)),z),n-(muc if t<AZF else min(muc,.2))*tg
ZS=[20,40,60,80,ZB1];TH=list(range(2,91,2));MU_L,MU_R=.03,.03   # 693ZZ bearing drag, ball rolling on steel (conservative)
MU_RACE=.005               # rolling resistance of a Ø4 steel ball on a hardened steel strip (typical 0.001-0.003; 0.005 allows for dust)
def heights(t):return sorted({min(z,ztop(t)) for z in ZS if not in_win(t,min(z,ztop(t)))})
def mucrit(H,F,same=False):
    if gsolve(H,F,0 if same else MU_L,0)[0]<=0:return 0.
    lo,hi=0.,3.
    for _ in range(22):
        m=(lo+hi)/2
        if gsolve(H,F,m if same else MU_L,m)[0]>0:lo=m
        else:hi=m
    return round(lo,3)
rows=[]
for t in TH:
    H0,F0=hit(t,20,0);rows.append(dict(arc_deg=t,skin_r_mm=round(skin_r(t),2),push_along_travel=round(float(F0@FT),3),heights_mm=heights(t),mu_vert_crit=[mucrit(*hit(t,z,0)[:2]) for z in heights(t)],
        mu_crit_bearing_and_ball_equal=[mucrit(*hit(t,z,0)[:2],same=True) for z in heights(t)]))
def sensed(mu_l,mu_v,muc,slide=None):
    locked=[]
    for t in TH:
        for z in heights(t):
            H,F=hit(t,z,muc)
            if gsolve(H,F,mu_l,mu_v,slide=slide)[0]<=0:locked.append((t,z))
    arc_=max([t for t in TH if all((tt,z) not in locked for tt in TH if tt<=t for z in heights(tt))],default=0)
    return dict(arc_sensed_to_deg=arc_,locked_points=[dict(arc_deg=t,z=z) for t,z in locked][:40])
mins=min(min(r['mu_vert_crit']) for r in rows);mins_eq=min(min(r['mu_crit_bearing_and_ball_equal']) for r in rows)
des=sensed(MU_L,MU_R,0.)
DRAG=MU_RACE*float(VPRE.sum())+.49*math.sin(math.radians(40))   # preload rolling drag + roller reaction on the ramp (resists the travel)
bumper['friction_lock']=dict(method='Quasi-static rigid half; lateral reactions at the two rail-pin bearings; vertical reactions at the 9 ball contacts shared as a compliant body (equal stiffness, unilateral, minimum-norm), friction moments iterated. mu_vert_crit = vertical-contact friction at which a hit stops sliding the half (larger is better).',
    contacts=[dict(x=round(c[0],2),y=round(c[1],2),z=c[2],base_pushes='up' if c[3]>0 else 'down') for c in VC],rows=rows,min_mu_vert_crit=mins,required_min_mu_vert_crit=2*MU_R,
    min_mu_crit_bearing_and_ball_equal=mins_eq,race_rolling_friction=MU_RACE,required_min_mu_crit_equal=2*MU_RACE,
    design_case=f'rolling balls mu {MU_R} (conservative for steel races), 693ZZ bearings mu {MU_L}, frictionless contact',
    sensed_design=des,
    sensed_static_push_object_friction_0p5=sensed(MU_L,MU_R,.5),sensed_static_push_object_friction_0p8=sensed(MU_L,MU_R,.8),
    sensed_failure_case_balls_sliding_0p3=sensed(MU_L,.3,0.),sensed_one_ball_sliding_0p15=sensed(MU_L,MU_R,0.,slide=.15),
    one_ball_note='The single most loaded ball at each hit seized and sliding at mu 0.15 on its steel race (the others rolling): a partial failure the design must still sense.',
    races_note='Every ball runs between hardened steel strips (no PETG contact, no dimpling: see hertz). The sliding case is a failure mode (a ball seized in its pocket), not the design.',
    preload_rolling_drag_N=round(MU_RACE*float(VPRE.sum()),3),roller_ramp_reaction_N=round(.49*math.sin(math.radians(40)),3),
    object_friction_note='Static or side pushes: the skin slides along the object as the half moves along its 30 deg axis; EVA foam mu_c as stated on az < 70, PTFE-taped PETG (mu_c 0.2) on az 70..90.',
    heights_note=f'Full height {ZB1:g} ({ZS0:g} on the seam band chamfer); heights inside the camera window (|az| < {AZW:.1f} deg) are skipped; every listed height has bumper surface.')
for side in ['left','right']:
    bumper['halves'][side]['arc_sensed_to_deg']=des['arc_sensed_to_deg'];bumper['halves'][side]['friction_lock']=bool(des['locked_points'])
bumper['friction_lock']['margin_ok']=mins>=2*MU_R and mins_eq>=2*MU_RACE
# Contact stress at the trip force (push that reaches 2 mm of travel at each arc point and height) and at a 14 N wheel-stall push:
# ball on steel race (Hertz, hardened spring steel, static limit 2500 MPa); the PETG under a 0.5 mm strip (beam on an elastic
# foundation); the 693ZZ outer race rolling on the PETG slot side (line contact, 4 mm wide). PETG faces must stay <= 40 MPa at the trip force.
ES=1/(2*(1-.3**2)/210e3);EP=1/((1-.3**2)/210e3+(1-.38**2)/2000.)
p_ball=lambda F:1.5*F/(math.pi*(3*F*BR/(4*ES))**(2/3)) if F>0 else 0.
beta=(2000./(4*210e3*RACE_W*RACE_T**3/12))**.25
q_petg=lambda F:F*beta/(2*RACE_W)
p_line=lambda F:math.sqrt(max(F,0)/4.0*EP/(math.pi*4.0))
HZ=[]
for t in TH:
    for z in heights(t):
        H,F=hit(t,z,0);net,_,_,V,RAh,RBh=gsolve(H,F,MU_L,MU_R,full=True)
        Ftrip=(kl*(PL+2)+DRAG)/net if net>0 else 99.
        for lab,Fp in [('trip',Ftrip),('stall_14N',14.)]:
            Vt=VPRE+V*Fp;HZ.append(dict(arc_deg=t,z=z,case=lab,push_N=round(Fp,2),max_ball_load_N=round(float(Vt.max()),2),p0_ball_on_steel_MPa=round(p_ball(float(Vt.max()))),
                petg_under_strip_MPa=round(q_petg(float(Vt.max())),2),bearing_load_N=round(float(max(RAh,RBh)*Fp),2),bearing_on_petg_slot_MPa=round(p_line(float(max(RAh,RBh)*Fp)),1)))
hz=lambda case,k:max(r[k] for r in HZ if r['case']==case)
bumper['hertz']=dict(ball_race='0.5 mm hardened spring steel (blue-tempered 1095 or AISI 420, >= HRC 48) on every ball face; Ø4 G10 52100 balls',
    trip=dict(max_push_N=hz('trip','push_N'),max_ball_load_N=hz('trip','max_ball_load_N'),max_p0_ball_on_steel_MPa=hz('trip','p0_ball_on_steel_MPa'),max_petg_under_strip_MPa=hz('trip','petg_under_strip_MPa'),max_bearing_on_petg_slot_MPa=hz('trip','bearing_on_petg_slot_MPa')),
    stall_14N=dict(max_ball_load_N=hz('stall_14N','max_ball_load_N'),max_p0_ball_on_steel_MPa=hz('stall_14N','p0_ball_on_steel_MPa'),max_petg_under_strip_MPa=hz('stall_14N','petg_under_strip_MPa'),max_bearing_on_petg_slot_MPa=hz('stall_14N','bearing_on_petg_slot_MPa')),
    limits=dict(petg_face_at_trip_MPa=40,steel_race_static_MPa=2500),preload_contact_loads_N=[round(float(v),2) for v in VPRE],
    note='Ball faces are steel on both sides; PETG only carries the spread load of the strip. The bearing outer race rolls on the PETG slot side (line contact): reported against the same 40 MPa limit.')
bumper['hertz']['ok']=bumper['hertz']['trip']['max_petg_under_strip_MPa']<=40 and bumper['hertz']['trip']['max_bearing_on_petg_slot_MPa']<=40 and bumper['hertz']['trip']['max_p0_ball_on_steel_MPa']<=2500
bumper['hertz']['stall_note']='At a 14 N wheel-stall push held at the hard stop the most loaded ball reaches the p0 above: hardened (>= HRC 55) races may take micrometre brinell marks there; inspect the races at service. PETG faces stay under 40 MPa even then.'
bumper['trip_force_N']=dict(min=round(min(r['push_N'] for r in HZ if r['case']=='trip'),2),max=hz('trip','push_N'),resistance_at_2mm_N=round(kl*(PL+2)+DRAG,2),
    return_force_at_rest_after_creep_N=round(kl*PL*(1-CREEP),2),return_resistance_N=round(MU_RACE*float(VPRE.sum()),3),return_note='On the return stroke the roller pushes the cam back (helps); only the race rolling drag resists.')
bumper['leaf_spring']['returns_half_after_creep']=kl*PL*(1-CREEP)>=2*MU_RACE*float(VPRE.sum())
bumper['intended_contacts']='Balls roll between steel races (pocket floor / rider ceiling and the strips bonded to P_base); the rail-pin bearings roll in the P_base slots; the preloaded leaf nose bears on the P_base abutment; the cam holds the switch roller (pre-pressed at rest).'
# Interlayer (z) stress, rough: the halves print upright, so a hit high on the skin bends layer bonds at the gusset roots and in the wall
# above the gusset tips. Impact at the hard stop from the DriveCore speed (and twice it): 2 mm to trip, 10 ms latency, 2.0 m/s2 braking;
# equivalent static force from the impact energy and an assumed series stiffness (EVA patch 450 N/mm, two gussets 2 × 45 N/mm).
ZHL=ZB1-11.                # lower bumper hit height (z 130 before L2); the gusset cantilever length runs from the rail top to it
def interlayer(v0,Mtot,zh=ZB1):   # gate model, plus (informational) a series stiffness consistent with the hit height
    v2=max(v0*v0-2*2.0*max(0.,(BT-2.0)/1000-v0*.010),0.);E=.5*Mtot*v2
    kg=2*3*2000*(GUST*29**3/12)/(ZHL-RZ[1])**3;k=1/(1/450.+1/kg)   # gate: EVA patch + two gussets
    kw=3*2000*(80*WALL2**3/12)/(zh-min(GTIP))**3;k2=1/((1/450. if zh<=ZFT else 0.)+1/kg+1/kw)   # foam only below its top (z ZFT); + the wall's own bending above the gusset tip
    Sg=GUST*29**2/6;Sw=80*WALL2**2/6
    def st(k_):F=math.sqrt(2*E*1000*k_);return F,F/2*(zh-RZ[1])/Sg,F*(zh-min(GTIP))/Sw
    F,sg,sw=st(k);F2,sg2,sw2=st(k2)
    return dict(v0_mps=v0,hit_z_mm=zh,v_at_stop_mps=round(math.sqrt(v2),3),energy_J=round(E,4),series_stiffness_N_per_mm=round(k,1),equivalent_force_N=round(F,1),gusset_root_MPa=round(sg,1),wall_above_gusset_tip_MPa=round(sw,1),
        consistent_stiffness=dict(foam_in_series=zh<=ZFT,wall_bending_N_per_mm=round(kw,1),series_stiffness_N_per_mm=round(k2,1),equivalent_force_N=round(F2,1),gusset_root_MPa=round(sg2,1),wall_above_gusset_tip_MPa=round(sw2,1)))
def tsec(w,t,fins):   # plate w × t (outer face at 0) with vertical fins (thickness, depth) on its inner face: (I, distance of the outer face from the centroid)
    A=[(w*t,t/2,w*t**3/12)]+[(ft_*fd,t+fd/2,ft_*fd**3/12) for ft_,fd in fins];At=sum(a for a,_,_ in A);c=sum(a*y for a,y,_ in A)/At
    return sum(i+a*(y-c)**2 for a,y,i in A),c
# Envelope about the z axis (= the axle): every non-wire part at rest and with each bumper half at its hard stop; then the bumper skin over
# every feasible planar-play pose (travel, side shift, yaw), combined with the vertical tilt a hit can cause inside the stop gaps, plus the
# print / foam tolerance (shell radius +0.3, foam +0.2).
rmax=lambda v:float(np.max(np.hypot(v[:,0],v[:,1]))) if len(v) else 0.
env_rest=max((rmax(MESH[i]),i) for i in MESH if not NOW(i))
env_full=[max((rmax(MESH[i]+np.array(moved(i,side,BT))),i) for i in MESH if not NOW(i)) for side in ['left','right']]
wmax=max((rmax(MESH[i]),i) for i in MESH if NOW(i))
SKV=np.vstack([MESH['P_bumper_left'],MESH['bumper_foam_left']]);SKV=SKV[np.hypot(SKV[:,0],SKV[:,1])>=R_SH-4]
pr=[(rmax(pose_xy(SKV[:,:2],p_)),p_) for p_ in POSES];prm=max(pr,key=lambda r:r[0])
SKP=np.array([(*polar(r,a),z) for r,a,z in SKINPTS])
def tilt_play():
    best=(0.,None)
    for q in TV:
        D_=np.array([tilt_disp(q,x,y,z) for x,y,z in SKP])[:,:2]
        for p_ in POSES:
            r=rmax(pose_xy(SKP[:,:2]+D_,p_))
            if r>best[0]:best=(r,p_)
    return best
tpm=tilt_play()
TOL_R=(.3,.2)
env=dict(envelope_max_r_mm=round(max([env_rest[0]]+[e[0] for e in env_full]),2),limit_mm=160.0,about='z axis through base_link (0,0) = drive axle',
         rest=dict(r_mm=round(env_rest[0],2),part=env_rest[1]),full_travel=[dict(r_mm=round(e[0],2),part=e[1]) for e in env_full],
         under_load_tilt_max_r_mm=max(bumper['tilt_under_load']['max_skin_r_mm_rest_pose'],bumper['tilt_under_load']['max_skin_r_mm_full_travel']),
         planar_play=dict(max_r_mm=round(prm[0],3),pose_s_e_yawdeg=[prm[1][0],round(prm[1][1],3),round(math.degrees(prm[1][2]),3)],poses=len(POSES),method='every mesh vertex of the left half and its foam (the right half is its mirror image) over every feasible pose'),
         planar_play_with_tilt=dict(max_r_mm=round(tpm[0],3),pose_s_e_yawdeg=[tpm[1][0],round(tpm[1][1],3),round(math.degrees(tpm[1][2]),3)],method='skin sample points (every 2 deg, z 26/70/114 and the rim) tilted by every vertex of the stop-gap tilt set, then moved by every feasible planar pose'),
         print_tolerance_mm=dict(shell_radius=TOL_R[0],foam_thickness=TOL_R[1],note='checked with a printed radius template before the foam is bonded; foam measured 1.5 ± 0.2'),
         wires_max_r_mm=round(wmax[0],2),wires_max_r_part=wmax[1],turn_in_place_note='The axle is the circle centre, so a spin in place sweeps exactly this circle.')
env['worst_case_r_mm']=round(max(env['envelope_max_r_mm'],env['under_load_tilt_max_r_mm'],prm[0],tpm[0])+sum(TOL_R),3)
env['ok']=env['envelope_max_r_mm']<=160.0 and env['under_load_tilt_max_r_mm']<=160.0 and prm[0]<=160.0 and tpm[0]<=160.0 and env['worst_case_r_mm']<=160.0

# Static fasteners, inserts, nuts, standoffs, dowels and switch bodies: minimum gaps to every part they are not designed to touch.
FAST=[i for i in shapes if i.startswith(('fan_screw','motor_base','motor_face','tower_screw','rear_shell_screw','rear_shell_dowel','caster_screw','bay_screw','bay_spacer','bay_clamp_spacer','tray_screw','lid_post_screw','lid_screw','bumper_switch','lidar_tray_screw','jetson_standoff','jetson_screw','insert_','base_race_'))]
fast_rows=[];fast_bad=[]
for i in FAST:
    g_=[x for x in gaps(shapes[i],[j for j in SOLID if j!=i and not allowed(i,j)],6)]
    r=dict(part=i,min_gap_mm=g_[0][0] if g_ else None,nearest=g_[0][1] if g_ else 'none within 6 mm',required_mm=.5)
    fast_rows.append(r)
    if g_ and g_[0][0]<.5:fast_bad.append(r)
for side in ['left','right']:
    H_=bumper['halves'][side]
    for k,row in H_['moving_hardware_clearances'].items():
        for st,v in row.items():
            if v['min_gap_mm'] is not None and v['min_gap_mm']<2.0:fast_bad.append(dict(part=k,state=st,min_gap_mm=v['min_gap_mm'],nearest=v['nearest'],required_mm=2.0))
            if v['wire_min_gap_mm'] is not None and v['wire_min_gap_mm']<1.0:fast_bad.append(dict(part=k,state=st,min_gap_mm=v['wire_min_gap_mm'],nearest='a wire',required_mm=1.0))
    for st,v in H_['rigid_part_clearance'].items():
        if v['min_gap_mm'] is not None and v['min_gap_mm']<2.0:fast_bad.append(dict(part='P_bumper_'+side,state=st,min_gap_mm=v['min_gap_mm'],nearest=v['nearest'][0]['part'],required_mm=2.0))
        if v['wire_min_gap_mm'] is not None and v['wire_min_gap_mm']<1.0:fast_bad.append(dict(part='P_bumper_'+side,state=st,min_gap_mm=v['wire_min_gap_mm'],nearest=v['wire_nearest'],required_mm=1.0))
        if v['leaf_min_gap_mm'] is not None and v['leaf_min_gap_mm']<2.0:fast_bad.append(dict(part='P_bumper_'+side+' leaf',state=st,min_gap_mm=v['leaf_min_gap_mm'],nearest=v['leaf_nearest'],required_mm=2.0))
        if v['leaf_to_switch_mm']<1.0:fast_bad.append(dict(part='P_bumper_'+side+' leaf',state=st,min_gap_mm=v['leaf_to_switch_mm'],nearest='bumper_switch_'+side,required_mm=1.0))
        if v['leaf_body_to_abutment_mm']<.3:fast_bad.append(dict(part='P_bumper_'+side+' leaf body (without its nose)',state=st,min_gap_mm=v['leaf_body_to_abutment_mm'],nearest='P_base leaf abutment',required_mm=.3))
        if v['nose_to_abutment_face_mm']>.05:fast_bad.append(dict(part='P_bumper_'+side+' leaf nose',state=st,min_gap_mm=v['nose_to_abutment_face_mm'],nearest='P_base leaf abutment (must touch: preload)',required_mm=0.))
    if not H_['switch_body_ok']:fast_bad.append(dict(part='bumper_switch_'+side+' body',min_gap_mm=min(H_['switch_body_gap_to_moving_half_mm'].values()),nearest='P_bumper_'+side,required_mm=1.0))
    if not H_['rigid_half_to_P_base_ok']:fast_bad.append(dict(part='P_bumper_'+side+' rigid half',nearest='P_base',min_gap_mm=min(r['rigid_half_gap_to_P_base_mm'] for r in H_['rigid_part_clearance'].values()),required_mm=1.0))
    if not H_['race_clearance_ok']:fast_bad.append(dict(part='base races '+side,nearest=H_['base_race_nearest'],min_gap_mm=H_['base_race_running_clearance_mm'],required_mm=.7))
for sg_,sd in [(1,'left'),(-1,'right')]:
    d_=round(abut(sg_).distance(shapes['bumper_switch_'+sd]),2);bumper['halves'][sd]['abutment_to_switch_mm']=d_
    if d_<1.0:fast_bad.append(dict(part='P_base leaf abutment '+sd,nearest='bumper_switch_'+sd,min_gap_mm=d_,required_mm=1.0))
if bumper['switch']['planform_gap_to_tower_screws_mm']<2:fast_bad.append(dict(part='bumper_switch_left',nearest='tower screws',min_gap_mm=bumper['switch']['planform_gap_to_tower_screws_mm'],required_mm=2.0))
# Rolling balls: their steel race strips on P_base must sit on solid plate (>= 1.0 mm from every hole, slot and edge).
ball_tracks=[]
for r_ in RACES:
    s_=shapes[r_];d=min((round(s_.distance(h_),2),n) for n,h_ in BH if near(s_,h_,4)) if any(near(s_,h_,4) for _,h_ in BH) else (99.,'none')
    b=s_.BoundingBox();cs=[(b.xmin,b.ymin),(b.xmin,b.ymax),(b.xmax,b.ymin),(b.xmax,b.ymax)]
    edge=round(min(base_edge_r(math.degrees(math.atan2(y,x)))-math.hypot(x,y) for x,y in cs),2)
    ball_tracks.append(dict(race=r_,face='underside' if b.zmax<29.5 else 'top',gap_to_nearest_hole_mm=d[0],nearest=d[1],gap_to_base_edge_mm=edge))
ball_tracks_ok=all(b['gap_to_nearest_hole_mm']>=1.0 and b['gap_to_base_edge_mm']>=1.0 for b in ball_tracks)
print('fasteners',round(time.time()-T0),'s',flush=True)

# Camera: nothing (wires included) may enter the image frustum (4:3, ±42 deg horizontal) from the lens front face OR from an apex 3 mm
# further back (worst entrance pupil), with the bumper at rest and with each half at full travel; bumper parts keep >= 4 mm (normal) to the
# lens-face frustum and >= 1.5 mm to the 3-mm-back one.
camrows=[];camhit=[]
for st in ['rest','left full travel','right full travel']:
    for i in [p['id'] for p in parts]:
        if i=='camera_lens':continue
        s=shapes[i]
        if st!='rest':s=s.translate(moved(i,st.split()[0],BT))
        for lab,F_ in [('lens face apex',FR),('apex 3 mm back',FRB)]:
            if near(s,F_,1):
                v=s.intersect(F_).Volume()
                if v>1e-3:camhit.append(dict(state=st,frustum=lab,part=i,volume_mm3=round(v,3)))
    sh_=lambda i:shapes[i].translate(moved(i,st.split()[0],BT) if st!='rest' else (0,0,0))
    BUMP=['P_bumper_left','P_bumper_right','bumper_foam_left','bumper_foam_right']
    bs=[(round(sh_(i).distance(FR),2),i) for i in BUMP];bb=[(round(sh_(i).distance(FRB),2),i) for i in BUMP]
    camrows.append(dict(state=st,bumper_clearance_to_frustum_mm=min(bs)[0],nearest=min(bs)[1],bumper_clearance_to_frustum_apex_3mm_back_mm=min(bb)[0],
        holder_clearance_to_frustum_apex_3mm_back_mm=round(shapes['P_camera_holder'].distance(FRB),2)))
campl=[]
for sg_,sd in [(1,'left'),(-1,'right')]:   # the bumper halves in their planar-play extreme poses (rest and hard stop)
    for p_ in extremes([0.,.1,4.9,5.]):
        for i in ('P_bumper_'+sd,'bumper_foam_'+sd):
            q=pose_shape(shapes[i],p_,sg_);campl.append(dict(part=i,pose_s_e_yawdeg=[p_[0],round(p_[1],3),round(math.degrees(p_[2]),3)],to_frustum_mm=round(q.distance(FR),2),to_frustum_apex_back_mm=round(q.distance(FRB),2)))
camrows.append(dict(state='planar play extremes (rest and hard stop, both halves)',bumper_clearance_to_frustum_mm=min(r['to_frustum_mm'] for r in campl),nearest=min(campl,key=lambda r:r['to_frustum_mm'])['part'],
    bumper_clearance_to_frustum_apex_3mm_back_mm=min(r['to_frustum_apex_back_mm'] for r in campl),poses=len(campl)))
camera_fov=dict(lens_pose_mm=[CAMF.x,CAMF.y,CAMF.z],apex_back_mm=[CAMB.x,CAMB.y,CAMB.z],hfov_deg=PARAMS['camera_hfov_deg'],vfov_deg=round(2*math.degrees(math.atan(tv)),2),check_image_px=PARAMS['camera_check_image_px'],
    stream_image_px=PARAMS['camera_image_px'],lower_edge_depression_deg=dict(centre=round(math.degrees(math.atan(tv)),2),corner=round(math.degrees(math.atan(tv/math.hypot(1,th))),2)),
    bumper_clearance=camrows,required_bumper_clearance_mm=4.0,required_bumper_clearance_apex_back_mm=1.5,parts_in_frustum=camhit,
    pitch_tolerance_deg=round(min(r['bumper_clearance_to_frustum_mm'] for r in camrows)/((R_SKIN-CAMO.x)*math.radians(1)),1),
    note='Frusta from 0.5 mm in front of each apex to the floor; checked against every part including wires (lens excluded). Task F: the camera looks out through the bumper window '
         '(camera_window); the window keeps >= 4 mm to the lens-face frustum and >= 1.5 mm to the 3-mm-back one in every pose, including the planar-play extremes.')
# Task F: the turned-round LD14, its tray and lead sit above and behind the camera: their clearance to both frusta, reported (any
# intrusion is already in parts_in_frustum).
camera_fov['lidar_parts_clearance']={n:dict(to_frustum_mm=round(shapes[n].distance(FR),2),to_frustum_apex_3mm_back_mm=round(shapes[n].distance(FRB),2)) for n in ('lidar','lidar_motor','lidar_turret','P_lidar_tray','W20','J28') if n in shapes}
camera_fov['ok']=not camhit and min(r['bumper_clearance_to_frustum_mm'] for r in camrows)>=4.0 and min(r['bumper_clearance_to_frustum_apex_3mm_back_mm'] for r in camrows)>=1.5
# Camera window (task F): its size, the lens recess, and the gap from the camera parts to each moving half (shell, foam and seam backer) at
# every travel and in the planar-play extremes. Rule: >= 1.5 mm everywhere, and the lens face lies further behind the skin round the window
# than a half moves at its hard stop (+ 2 mm): an object too big to pass the window pushes a half to the trip point (2.25 mm) and on to the
# hard stop without reaching the lens.
CAMPARTS=['camera_lens','camera_board','P_camera_holder','J17_C','P_camera_led_shelf','camera_led_board','camera_led']   # task N: the LED parts sit in the same bay
wrows=[]
for sg_,sd in [(1,'left'),(-1,'right')]:
    for s_ in STATES:
        for i in ('P_bumper_'+sd,'bumper_foam_'+sd):
            q=shapes[i].translate(mvec(sd,s_));wrows.append(dict(part=i,pose=f's {s_}',gap_mm=min((round(q.distance(shapes[c]),2),c) for c in CAMPARTS)))
    for p_ in extremes([0.,.1,4.9,5.]):
        for i in ('P_bumper_'+sd,'bumper_foam_'+sd):
            q=pose_shape(shapes[i],p_,sg_);wrows.append(dict(part=i,pose=f'play s {p_[0]} e {round(p_[1],3)} yaw {round(math.degrees(p_[2]),3)} deg',gap_mm=min((round(q.distance(shapes[c]),2),c) for c in CAMPARTS)))
wmin=min(wrows,key=lambda r:r['gap_mm'][0])
rim=[polar(R_SKIN if win_z(R_SKIN)[0]<ZFT else R_SH,a) for a in np.linspace(-AZW,AZW,41)]   # the window's edge on the skin (foam at its sill)
camera_window=dict(lens_face_x_mm=CAMO.x,axis_z_mm=CH,flare_deg=WIN_FLARE,sill_and_roof_z_mm=list(WIN_Z),print_support=f'a 1.8 mm plug at r {WIN_PLUG_R[0]}..{WIN_PLUG_R[1]} in each half (and a pillar under the right half upper seam backer) in the as-printed STL, cut out after printing',
    at_skin=dict(half_width_deg=round(AZW,2),width_mm=round(2*win_hw(R_SKIN),1),z_mm=[round(v,1) for v in win_z(R_SKIN)]),
    at_inner_wall=dict(x_mm=WIN_X,width_mm=round(2*win_hw(WIN_X),1),z_mm=[round(v,1) for v in win_z(WIN_X)]),
    lens_recess_behind_skin_mm=round(min(x for x,_ in rim)-CAMO.x,2),hard_stop_travel_along_x_mm=round(-BT*ft[0],2),
    min_gap_camera_to_moving_halves_mm=wmin['gap_mm'][0],at=dict(part=wmin['part'],pose=wmin['pose'],camera_part=wmin['gap_mm'][1]),poses=len(wrows),
    rule='camera parts >= 1.5 mm from both halves (shell, foam, seam backer) at every travel and planar-play pose; lens recess >= hard-stop travel + 2 mm',
    residual=f'An object small enough to pass the window opening (about {2*win_hw(WIN_X):.0f} x {win_z(WIN_X)[1]-win_z(WIN_X)[0]:.0f} mm at the inner wall) can reach the lens without moving a half: '
             'the perimeter rods that do so are listed in rods_through_window. Anything larger meets a half first and trips it before the lens.')
camera_window['ok']=camera_window['min_gap_camera_to_moving_halves_mm']>=1.5 and camera_window['lens_recess_behind_skin_mm']>=camera_window['hard_stop_travel_along_x_mm']+2

# Cliff ToF cones (25 deg full angle from the optical face to the floor), wires and leads included, at rest and at full travel.
cliffrows=[]
for i,(x,y) in enumerate(CLIFF_IDS):
    cone=cq.Solid.makeCone(1.2+CZ*math.tan(math.radians(PARAMS['cliff_fov_deg']/2)),1.2,CZ,cq.Vector(x,y,0))
    best=(99.,'')
    for j in [p['id'] for p in parts]:
        if j==f'cliff_{i+1}':continue
        for st in ['rest','left','right']:
            s=shapes[j].translate(moved(j,st,BT)) if st!='rest' else shapes[j]
            if near(s,cone,4):best=min(best,(round(s.distance(cone),2),j+('' if st=='rest' else f' ({st} full travel)')))
    cliffrows.append(dict(sensor=f'cliff_{i+1}',xy=[x,y],min_gap_mm=best[0],nearest=best[1]))
cliff_fov=dict(fov_deg=PARAMS['cliff_fov_deg'],optical_face_z_mm=CZ,aperture_radius_mm=1.2,sensors=cliffrows,required_mm=2.0,ok=all(r['min_gap_mm']>=2.0 for r in cliffrows))

# LiDAR: no stationary part may have surface in the scan band (laser plane ± 4 mm) outside the turret (r > 30.2 about the spin axis).
LX_,LH_=PARAMS['lidar_x'],PARAMS['lidar_height'];band=[]
for i in MESH:
    if i in ('lidar_turret','lidar_cap','lidar_lens'):continue
    T=MESH[i][MESHF[i]];k=(T[:,:,2].max(1)>=LH_-4)&(T[:,:,2].min(1)<=LH_+4)
    if k.any():
        P=T[k].reshape(-1,3);P=np.vstack([P,T[k].mean(1)]);P=P[(P[:,2]>=LH_-4)&(P[:,2]<=LH_+4)] if len(P[(P[:,2]>=LH_-4)&(P[:,2]<=LH_+4)]) else T[k].mean(1)
        r=np.hypot(P[:,0]-LX_,P[:,1]).max()
        if r>30.2:band.append(dict(part=i,max_r_from_spin_axis_mm=round(float(r),2)))
ldall=cq.Compound.makeCompound([ldp(x) for x in LDS if abs(x.Volume()-7932.7)>1]+[shapes['lidar_turret']])
def _ldgap(a,b):   # task L: OCC's distance returns 0 for a threaded STEP screw (spline faces) of the LD14 motor that does not touch the tray; then use its bounding box (a lower bound)
    d=a.distance(b)
    if d<1e-6 and a.intersect(b).Volume()<1e-6:bb=b.BoundingBox();d=a.distance(box((bb.xmin+bb.xmax)/2,(bb.ymin+bb.ymax)/2,bb.zmin,bb.xlen,bb.ylen,bb.zlen))
    return d
lidar_check=dict(optical_plane_z_mm=LH_,step_window_band_z=[8.41,16.0],placement_offset_mm=[LX,0,round(LZ,2)],rotation_deg=LYAW,
    lug_underside_z_mm=round(lug_z,2),tray_overlap_with_lidar_mm3=round(shapes['P_lidar_tray'].intersect(cq.Compound.makeCompound([ldall,shapes['lidar']])).Volume(),3),
    tray_gap_to_non_lug_lidar_parts_mm=round(min(_ldgap(shapes['P_lidar_tray'],x_) for x_ in ldall.Solids()),2),tray_gap_to_motor_mm=round(shapes['P_lidar_tray'].distance(shapes['lidar_motor']),2),
    lead_gap_to_turret_mm=round(shapes['W20'].distance(shapes['lidar_turret']),2),parts_in_scan_band=band,
    method=f'Triangle samples of every part except the rotating turret, cap and windows, in z {LH_-4:g}..{LH_+4:g}, outside r 30.2 about the spin axis.')
tf=[]
for i in range(len(TRAYH)):
    q=shapes[f'lidar_tray_screw_{i}'];g_=[(round(q.distance(shapes[j]),2),j) for j in SOLID if j not in LID_IDS+('P_lidar_tray',) and not j.startswith('lidar_tray_screw') and near(q,shapes[j],6)]
    tf.append(dict(at=TRAYH[i],min_gap_mm=min(g_)[0] if g_ else None,nearest=min(g_)[1] if g_ else 'none within 6 mm'))
lidar_check['tray_fasteners']=tf;lidar_check['scan_band_ok']=not band
# Task F: the LD14 returns nothing under 0.1 m (D200 kit LD14P minimum range; the simulator's D200Model returns -1 there). An object
# touching the bumper skin, with the bumper pushed to its 5 mm hard stop, must stay outside that radius at every azimuth so the scan still
# sees what the bumper touches. This sets lidar_x: a trial at x 109 left a 5 cm blind strip ahead of the bumper, and the controller's
# near-field checks (approach stop, spin room) lost what was in it. The nearest skin point to the spin axis is straight ahead (az 0).
_rmin=min(math.hypot(R_SKIN*math.cos(math.radians(a))+BT*ft[0]-LX,R_SKIN*math.sin(math.radians(a))-abs(BT*ft[1])) for a in np.arange(-90,90.1,1))
lidar_check['min_range']=dict(ld14_min_range_mm=100.0,nearest_pressed_skin_to_spin_axis_mm=round(_rmin,1),margin_mm=round(_rmin-100.0,1),
    note='front skin (R_SKIN) with a half at its hard stop (4.33 mm rearward, 2.5 mm inward) to the LiDAR spin axis; above 100 mm the LD14 sees an object the bumper touches',ok=_rmin>=100.0)
SKIN_=cyl(0,0,LZ-30,R_SKIN+10,60).cut(cyl(0,0,LZ-31,R_SKIN,62))   # outside the bumper skin, round the LD14 height
_fg=min((round(shapes[n].distance(SKIN_),2),n) for n in ('lidar','lidar_motor','lidar_turret','lidar_connector'))
lidar_check['front_gap']=dict(required_mm=10.0,gap_to_skin_mm=_fg[0],nearest_part=_fg[1],ok=_fg[0]>=10.0,
    rule='every LD14 part (housing, motor, turret, connector) stays >= 10 mm inside the bumper skin R_SKIN')
_bt=max(shapes[i].BoundingBox().zmax for i in ('P_bumper_left','P_bumper_right'))
lidar_check['scan_band_vertical']=dict(band_z_mm=[LH_-4,LH_+4],lid_top_z_mm=ZLIDT,bumper_top_z_mm=round(_bt,2),tray_plate_top_z_mm=ZT0+TRAY_T,
    band_above_lid_mm=round(LH_-4-ZLIDT,2),band_above_bumper_mm=round(LH_-4-_bt,2),ok=LH_-4-ZLIDT>=20 and LH_-4-_bt>=20)
# Jetson cooling (L2, task L): lid underside >= fan top + 5 mm; the LiDAR tray's intake duct encloses the fan bore (its fan-window circle r 17.5)
# and the LD14 motor; room air over the tray feeds the grille and the 1.0 mm ring round the motor. Areas from a 0.25 mm plan grid over the fan bore
# (annulus r 7..17 of the E09 model), cross-checked with the built tray: open area = (bore annulus through the plate - tray material in it) / plate.
# Bypass (warm body air reaching the fan's suction side), measured on the built tray: along the duct's inner outline over the fan frame the wall
# ends SK_GAP over the frame (gap x outline length); where a fan screw head forced a notch, the notch height x its arc; beyond the fan's front face
# the bowl closes the duct and only the SLOT_G slot between its back wall and the fan's front face is open below (gap x the duct's width there);
# plus the LD14 lead aperture round its grommet. Until task L the duct was a skirt under the lid round the window alone.
FB=(17.0,7.0)
gx,gy=np.meshgrid(np.arange(-21,21.001,.25),np.arange(-21,21.001,.25));GX,GY=FW[0]+gx,FW[1]+gy;GR=np.hypot(gx,gy);CELL=.25**2
inbore=(GR<=FB[0])&(GR>=FB[1]);slot=np.zeros_like(GR,bool)
for k in range(-GR_N,GR_N+1):slot|=np.abs(GX-(FW[0]+k*GR_P))<=GR_W/2
_dm=np.hypot(GX-MOT[0],GY-MOT[1]);_ring=(_dm<=R_MI)&(_dm>MOT[2]);gopen=inbore&((slot&(GR<=GR_R)&(_dm>R_MI+GR_RIB))|_ring)   # grille slots (stopped GR_RIB short of the motor's hole) and the 1.0 mm ring round the motor in that hole (the motor itself is not open)
# Plan shadow over the bore of every part above the fan top (tray, lid, fan and its screws excluded): task I's overhang of the turned-round
# LD14, and since task L the motor itself, which hangs in the duct 1.4 mm over the bore's front edge. The gate counts shadowed open cells as
# closed; the side-entry curtain (clear height under the overhang × the shadow's width) is reported. Task L: the clear height is taken over
# the plate top where the overhang is above the plate (room air comes in over the plate, as to the ring under the LD14 housing), else
# over the fan frame (air from inside the duct).
under=np.zeros(GX.shape,bool);ZUN=np.full(GX.shape,np.inf);_ov=[]
for i in MESH:
    if i in ('P_lidar_tray','fan')+LID_IDS or i.startswith('fan_screw'):continue
    T=MESH[i][MESHF[i]];T=T[(T[:,:,2].min(1)>=ZFAN_TOP)&(T[:,:,0].min(1)<=GX.max())&(T[:,:,0].max(1)>=GX.min())&(T[:,:,1].min(1)<=GY.max())&(T[:,:,1].max(1)>=GY.min())]
    for t in T:
        (x0,y0,z0),(x1,y1,z1),(x2,y2,z2)=t;d=(y1-y2)*(x0-x2)+(x2-x1)*(y0-y2)
        if abs(d)<1e-9:continue
        m=(GX>=t[:,0].min())&(GX<=t[:,0].max())&(GY>=t[:,1].min())&(GY<=t[:,1].max())&(GR<=FW[2])
        if not m.any():continue
        a=((y1-y2)*(GX[m]-x2)+(x2-x1)*(GY[m]-y2))/d;b=((y2-y0)*(GX[m]-x2)+(x0-x2)*(GY[m]-y2))/d;c=1-a-b;h=(a>=-1e-9)&(b>=-1e-9)&(c>=-1e-9)
        if not h.any():continue
        k=np.flatnonzero(m.ravel())[h];under.flat[k]=True;ZUN.flat[k]=np.minimum(ZUN.flat[k],(a*z0+b*z1+c*z2)[h])
        if i not in _ov:_ov.append(i)
A_bore=float(inbore.sum()*CELL);A_open=float(gopen.sum()*CELL);A_shad=float((gopen&under).sum()*CELL)
_sh=under&(GR<=FW[2]);W_UN=float(GY[_sh].max()-GY[_sh].min()+.25) if _sh.any() else 0.;A_shad_ring=float((gopen&under&_ring).sum()*CELL)
H_UN=H_OVER=None
if A_shad:
    _zs=ZUN[gopen&under];_zb=np.where(_zs>=ZT0+TRAY_T,ZT0+TRAY_T,ZFAN_TOP);_k=int(np.argmin(_zs-_zb));H_UN=float(_zs[_k]-_zb[_k]);H_OVER='plate' if _zb[_k]>ZFAN_TOP else 'fan frame'
A_curt=round(H_UN*W_UN,1) if A_shad else None
ANN=cyl(FW[0],FW[1],ZT0,FB[0],TRAY_T).cut(cyl(FW[0],FW[1],ZT0-1,FB[1],TRAY_T+2));A_motor_cad=ANN.intersect(shapes['lidar_motor']).Volume()/TRAY_T   # task L: the motor stands in the plate's hole over the bore's edge
A_open_cad=(ANN.Volume()-ANN.intersect(shapes['P_lidar_tray']).Volume())/TRAY_T-A_motor_cad   # open through the plate: neither tray material nor the motor
TRAY_=shapes['P_lidar_tray'];_rear=TRAY_.intersect(box(XFAN-100,0,ZFAN_TOP-50,200,200,ZT0-ZFAN_TOP+50));_front=TRAY_.intersect(box(XFAN+100,0,ZFAN_TOP-50,200,200,ZT0-ZFAN_TOP+50))
ZSK_=min(v.Z for v in _rear.Vertices());GAP_=_rear.distance(shapes['fan']);GAP_SLOT=_front.distance(shapes['fan'])   # duct wall over the fan frame; bowl back wall (exact distances: bounding boxes are padded)
_zm=(ZSK_+ZLID)/2;SKSEC=TRAY_.intersect(dprism(SK_T+.5,_zm-.25,.5)).Volume()/.5;SKSEC0=dprism(SK_T,0,1).cut(dprism(0.,-1,3)).Volume()   # duct section at mid-height vs the unbroken outline
FSG=min(TRAY_.distance(shapes[f'fan_screw_{j}']) for j in range(len(FSCREWS)))
MGAP=shapes['lidar_motor'].distance(shapes['fan'])
def _outline_len():   # the duct's inner outline over the fan frame (x <= XFAN), without the notched arcs
    L=0.;st=.05
    for (cx,cy,ri),(ox,oy,orr) in (((FW[0],FW[1],FW[2]),(MOT[0],MOT[1],R_MI)),((MOT[0],MOT[1],R_MI),(FW[0],FW[1],FW[2]))):
        for a in np.arange(0,360,st):
            px,py=cx+ri*math.cos(math.radians(a)),cy+ri*math.sin(math.radians(a))
            if px>XFAN or math.hypot(px-ox,py-oy)<orr:continue
            if any((cx,cy)==tuple(n_['centre']) and abs((a-math.degrees(math.atan2(FSCREWS[n_['fan_screw']][1]-cy,FSCREWS[n_['fan_screw']][0]-cx))+180)%360-180)<=n_['half_angle_deg'] for n_ in DUCT_NOTCH):continue
            L+=math.radians(st)*ri
    return L
L_WALL=_outline_len();A_wall=GAP_*L_WALL
A_notch=sum((n_['z_mm']-ZFAN_TOP)*n_['arc_mm'] for n_ in DUCT_NOTCH)
_xs=XFAN+SLOT_G/2;L_SLOT=2*math.sqrt((R_MI+SK_T)**2-(_xs-MOT[0])**2);A_slot=GAP_SLOT*L_SLOT
_GS=shapes['P_ld14_grommet'].intersect(box(LD14_AP[0],LD14_AP[1],ZLID+LID_T/2-.25,LD14_AP[2]+4,LD14_AP[3]+4,.5)).Volume()/.5   # grommet section at mid-lid
A_ld14=round(LD14_AP[2]*LD14_AP[3]-_GS-math.pi*1.2**2,2)   # open area left in the aperture: round the lead, the slit, the side clearances
A_byp=A_wall+A_notch+A_slot+A_ld14
_ldw=min(shapes[n].distance(cyl(FW[0],FW[1],ZT0,GR_R,60)) for n in ('lidar','lidar_motor','lidar_turret'))   # plan gap from the LD14 to the column over the grille
# Exhaust (task I, the lid closed over the front bay): the warm air leaves under the lid rim through the ring gap over the bumper and
# rear-shell tops; its narrowest section is the gap times the circumference at the rear-shell rim's inner edge (the smaller of the two rims).
_rtop=max(float(MESH[i][:,2].max()) for i in ('P_bumper_left','P_bumper_right','P_rear_shell_left','P_rear_shell_right'))
EX_GAP=round(ZLID-_rtop,2);EX_R=R_RS-RWALL-10;A_ex=round(EX_GAP*2*math.pi*EX_R,1)
jetson_cooling=dict(fan_top_z_mm=round(ZFAN_TOP,2),lid_underside_z_mm=ZLID,lid_top_z_mm=ZLIDT,air_gap_fan_to_lid_mm=round(ZLID-ZFAN_TOP,2),required_air_gap_mm=AIRGAP,
    air_gap_fan_to_tray_grille_mm=round(ZT0-ZFAN_TOP,2),lid_fan_window=dict(centre_mm=[round(FW[0],2),round(FW[1],2)],diameter_mm=2*FW[2],area_mm2=round(math.pi*FW[2]**2),
        note=f'Task L: the fan-window circle is the tray duct\'s circle about the fan centre; the lid has one opening round the whole duct (the window and motor circles + the {SK_T} mm wall + {DUCT_FIT} mm).'),
    grille=dict(slots=f'{2*GR_N+1} × {GR_W} mm on a {GR_P} mm pitch inside r {GR_R:g}, no hub (task F: no lug tower over the fan); task L: the LD14 motor drops through a Ø{2*R_MI:g} hole in the plate, its 1.0 mm ring is open to the room',
        fan_bore_annulus_mm2=round(A_bore),open_area_over_bore_mm2=round(A_open),open_area_cad_mm2=round(A_open_cad),motor_in_plate_over_bore_mm2=round(A_motor_cad,1),
        open_fraction_of_bore=round(A_open/A_bore,3),open_area_shadowed_in_plan_mm2=round(A_shad,1),open_area_not_shadowed_mm2=round(A_open-A_shad),
        open_fraction_of_bore_not_shadowed=round((A_open-A_shad)/A_bore,3),bore_fraction_shadowed_in_plan=round(float((inbore&under).sum()/inbore.sum()),3)),
    intake=dict(over_grille=f'Room air through the grille and the 1.0 mm ring round the LD14 motor. Task L: the motor hangs in the intake duct, its rim {MGAP:.1f} mm over the fan frame, and covers the bore\'s front edge: '
                            f'the motor\'s own footprint is not open area. {A_shad:.1f} mm2 ({A_shad/A_open:.1%}) of the open area over the bore lies under the LD14 in plan, {A_shad_ring:.1f} mm2 of it the 1.0 mm ring round the motor; the gate counts that shadow as closed; '
                            f'those cells still draw air sideways {"over the plate, under the LD14 housing," if H_OVER=="plate" else "from inside the duct"} through a curtain about {A_curt:.0f} mm2 ({H_UN:.1f} mm clear over the {H_OVER} × {W_UN:.1f} mm across the window).'
                            if A_shad else 'open room air: nothing above the fan overhangs the bore in plan',
                lidar_gap_to_column_over_grille_mm=round(_ldw,2),parts_over_grille=_ov,clear_height_under_overhang_mm=round(H_UN,2) if A_shad else None,
                shadow_width_across_window_mm=round(W_UN,2),side_entry_curtain_mm2=A_curt,ld14_motor_gap_to_fan_mm=round(MGAP,2),
                method='plan shadow of the mesh triangles of every part above the fan top (tray, lid, fan and fan screws excluded) on the 0.25 mm grid over the bore; clear height = lowest overhanging surface over a shadowed open cell minus the plate top where that surface is above the plate (task L), else minus the fan top'),
    fan_skirt=dict(kind=f'intake duct under the LiDAR tray (task L; until then a skirt under the lid round the window): its fan-window circle r {FW[2]:g} and a circle r {R_MI:g} round the LD14 motor, {SK_T} mm wall; a bowl under the motor beyond the fan\'s front face',
        inner_r_mm=FW[2],wall_mm=SK_T,bottom_z_mm=round(ZSK_,2),gap_to_fan_frame_mm=round(GAP_,2),section_area_mm2=round(SKSEC,1),section_area_unbroken_mm2=round(SKSEC0,1),
        clearance_to_fan_screw_heads_mm=round(FSG,2),notches_over_fan_screw_heads=DUCT_NOTCH,
        wall_outline_over_frame_mm=round(L_WALL,1),bypass_area_skirt_mm2=round(A_wall,1),bypass_area_notches_mm2=round(A_notch,1),
        bowl_slot=dict(gap_to_fan_front_face_mm=round(GAP_SLOT,2),width_mm=round(L_SLOT,1),area_mm2=round(A_slot,1),rule=f'the bowl\'s back wall stays >= {SLOT_G} mm in front of the fan\'s front face'),
        ld14_aperture_leak_mm2=A_ld14,ld14_aperture_open_without_grommet_mm2=LD14_AP[2]*LD14_AP[3],bypass_area_mm2=round(A_byp,1),bypass_fraction_of_grille_open_area=round(A_byp/A_open_cad,3),bypass_limit_fraction=.25,
        feeler_go_nogo_mm=list(SK_FEELER),gap_at_bypass_limit_mm=round((.25*A_open_cad-(A_byp-A_wall))/L_WALL,2),bypass_margin_fraction=round(.25-A_byp/A_open_cad,3),
        rule='bypass (wall gap × the duct\'s inner outline over the fan frame + the notches + the bowl slot + the LD14 lead aperture left open round its grommet) <= 25 % of the grille open area over the bore (CAD: through the plate, less the tray and the LD14 motor standing in its hole), with >= 1 % margin; the no-go feeler is under the gap at the limit; duct unbroken (section within 1 %); >= 0.5 mm to the fan screw heads and the fan\'s front face; the duct circle >= the fan bore; the LD14 motor >= 1.0 mm from the fan'),
    bring_up_test='Lid-on load test before the robot is used: lid, LiDAR and tray fitted, sustained full CPU + GPU load for 20 min, log tegrastats (CPU/GPU/SoC temperatures and clocks) and compare with a lid-off run; the lid passes if the clocks never throttle. Carry into pcb/BRINGUP.md (L11).',
    exhaust=dict(path=f'The fan blows down through the heatsink fins; the warm air leaves the body through the {EX_GAP:g} mm ring gap between the bumper / rear-shell tops and the lid underside, the 7.5 mm bumper seam and the open ring between the base edge and the shells. '
                      'Task I closed the lid over the front bay (until then an open notch of about 7050 mm2, left from the old camera tower): the ring gap alone stays several times the grille\'s open area.',
        ring_gap_mm=EX_GAP,ring_gap_section_mm2=A_ex,ring_gap_section_radius_mm=EX_R,ring_gap_to_grille_open_area=round(A_ex/A_open_cad,2),
        rule='the ring gap section (gap × circumference at the rear-shell rim\'s inner edge) >= 2 × the grille open area (CAD); the seam, the camera window and the base ring are extra'),
    choice='The lid seams all meet near the centre (0, 0), which lies inside the fan footprint, so slots in the lid over the fan would cross them; so the grille is in the tray plate over the fan. The duct closes the 5.2 mm gap between the fan top and the lid, which would otherwise be a ~670 mm2 side inlet from inside the warm body. '
           'Task L: the lowered LD14\'s motor straddles the fan\'s front face, so the duct moved from the lid to the tray (one printed part, no joint) and takes the motor in, with a bowl under it beyond the fan.')
jetson_cooling['ok']=(jetson_cooling['air_gap_fan_to_lid_mm']>=AIRGAP and FB[0]<FW[2] and (A_open-A_shad)/A_bore>=.5 and abs(A_open-A_open_cad)<=.05*A_bore and A_ex>=2*A_open_cad
    and A_byp<=.24*A_open_cad and SK_FEELER[1]<=(.25*A_open_cad-(A_byp-A_wall))/L_WALL and abs(SKSEC-SKSEC0)<=.01*SKSEC0 and FSG>=.5 and GAP_SLOT>=SLOT_G-.01 and MGAP>=1.0)
# Lid switches: each hangs inside the lid over the pack; gaps to the battery top, the Tamiya pair and the bay board modules.
lid_switches={}
for i in ('master_switch','motor_switch','arm_button'):
    b_=shapes[i].BoundingBox();row=dict(body_bottom_z_mm=round(b_.zmin,2),above_battery_top_mm=round(b_.zmin-64,2))
    for j in ('battery','tamiya','bay_pico','bay_drv_0','bay_drv_1','bay_imu','strap_0','strap_1'):row[f'gap_to_{j}_mm']=round(shapes[i].distance(shapes[j]),2)
    lid_switches[i]=row
lid_switches_ok=all(r['above_battery_top_mm']>=10 and min(v for k,v in r.items() if k.startswith('gap_to_'))>=2 for r in lid_switches.values())
print('sensors',round(time.time()-T0),'s',flush=True)

# Perimeter rod intrusion: a Ø3 rod pushed radially inward every 0.5 deg at heights z 19..ZB1+0.5 (ZR). The first contact must be the bumper
# (sensed) or the fixed rear shell within 13.6 mm (7 mm to the R152.5 shell; 13.5 mm to the backer flange in the slot behind the bumper end);
# behind the bumper span (|az| > 90) the round lid's rim, flush with the rear shell (R_RS), counts as part of that rigid outline,
# unless the rod crosses the skin inside the camera window (task F): those rows reach the camera through the window unsensed and are the
# window's residual, listed in camera_window (objects that fit through the window opening).
CP=Caster(SOLID,rmin=95.)
def in_view(p):   # point inside the camera frustum
    d=p[0]-CAMO.x;return d>.5 and abs(p[1])<=th*d and abs(p[2]-CAMO.z)<=tv*d
def in_win_p(p):   # point inside the camera window cutter (the window opening and its flare)
    b,tp=win_z(p[0]);return abs(p[1])<=win_hw(p[0]) and b<=p[2]<=tp
ZR=[19,21,23,26,29,32,36,40,45,50]+[round(z-DZ,2) for z in (100,108,112,114,115.5,116.5,117.5,118.5,119.5,120.5,121.5,123,125,128,132,136,139,140.5,141.5)]   # fine steps around the camera notch top (they follow it down)
rod_rows=[];rod_bad=[];rod_cam=[]
for z in ZR:
    for a in np.arange(-180,180,.5):
        d_,prt=rod_probe(CP,float(a),z)
        c,s_=math.cos(math.radians(a)),math.sin(math.radians(a));n=(-s_,c)
        vis=any(in_view((R_SKIN*c+lo*n[0],R_SKIN*s_+lo*n[1],z+dz)) or in_win_p((R_SKIN*c+lo*n[0],R_SKIN*s_+lo*n[1],z+dz)) for lo,dz in ROD)
        kind='bumper' if prt.startswith(('P_bumper','bumper_foam')) else ('rear shell' if prt.startswith(('P_rear_shell','rear_shell_dowel')) or (prt in LID_IDS and abs(a)>90) else 'other')
        ok=kind=='bumper' or (kind=='rear shell' and d_<=13.6)
        if not ok:(rod_cam if vis else rod_bad).append(dict(az=float(a),z=z,intrusion_mm=d_,first_contact=prt))
        rod_rows.append((float(a),z,d_,prt))
def runs(lst):   # compress az runs
    out=[]
    for r in sorted(lst,key=lambda r:(r['z'],r['az'])):
        if out and out[-1]['z']==r['z'] and abs(out[-1]['az_to']+.5-r['az'])<1e-6:
            o=out[-1];o['az_to']=r['az'];o['max_intrusion_mm']=max(o['max_intrusion_mm'],r['intrusion_mm']);o['contacts']=sorted(set(o['contacts'])|{r['first_contact']})
        else:out.append(dict(z=r['z'],az_from=r['az'],az_to=r['az'],max_intrusion_mm=r['intrusion_mm'],contacts=[r['first_contact']]))
    return out
perim=dict(rod='Ø3 flat-ended rod (9 ray samples), radial, every 0.5 deg',heights_mm=ZR,rule='first contact P_bumper_*/bumper_foam_* (sensed) or P_rear_shell_* (or the flush round-lid rim behind the bumper span, |az| > 90) within 13.6 mm',
    max_intrusion_bumper_mm=max(r[2] for r in rod_rows if r[3].startswith(('P_bumper','bumper_foam'))),
    max_intrusion_rear_shell_mm=max([r[2] for r in rod_rows if r[3].startswith('P_rear_shell')],default=0),
    failures=runs(rod_bad),through_camera_window_not_bumper_sensed=runs(rod_cam))
perim['ok']=not rod_bad
perim['lid_seam_relief']=dict(kind='pocket in the lid underside (the lid stays closed from above)',x_from_mm=LID_SEAM_RELIEF[0],half_width_mm=LID_SEAM_RELIEF[1],depth_mm=LID_SEAM_RELIEF[2],
    ceiling_z_mm=ZLID+LID_SEAM_RELIEF[2],bumper_top_z_mm=ZB1,rod_top_at_highest_test_height_mm=max(ZR)+1.5,seam_backer_front_x_mm=round(XSTEP,2),seam_backer_front_x_at_full_travel_mm=round(XSTEP+BT*ft[0],2),
    note='task I: the lid is closed over the front bay; over the bumper seam its underside is lifted by this pocket, so a rod pushed into the seam meets the sensed seam backer (and pushes it to its stop) before the fixed lid rim')
print('rod scan',round(time.time()-T0),'s',flush=True)

# Spin sweep: an object fixed in the world at radius r_o (inside the skin circle, outside the rear shell) is carried round the
# robot by a spin in place, both directions. Every face it meets is listed; on a bumper half the contact force (normal plus
# object friction mu along the relative slip) must have a positive component along that half's travel.
def secs(ids,z):   # plan section segments (x0, y0, x1, y1) of each part's mesh at height z
    from trimesh.intersections import mesh_plane
    out={}
    for i in ids:
        L=mesh_plane(trimesh.Trimesh(MESH[i],MESHF[i],process=False),plane_normal=[0,0,1],plane_origin=[0,0,z])
        out[i]=np.asarray(L)[:,:,:2].reshape(-1,4) if len(L) else np.zeros((0,4))
    return out
def inside(S,p):   # even-odd test against closed section segments
    x0,y0,x1,y1=S.T;c=((y0>p[1])!=(y1>p[1]))&(p[0]<(x1-x0)*(p[1]-y0)/np.where(y1-y0==0,1e-12,y1-y0)+x0);return bool(c.sum()%2)
SW_IDS=['P_bumper_left','P_bumper_right','bumper_foam_left','bumper_foam_right','P_rear_shell_left','P_rear_shell_right']
SPIN_Z=[30,50,70,CH,ZHL]   # z 30/60/100/130 before L2 (top 141); now top ZB1; task F adds the camera axis height (window band)
SPIN_RO=[152.8,153.5,154.5,155.5,156.,157.,158.,round(R_SKIN-.2,2)]
spin=[]
for z in SPIN_Z:
    SEG=secs(SW_IDS,z)
    for ro in SPIN_RO:
        ev=[]   # (az, part, outward normal) of every face crossing on the circle r_o
        for i,S in SEG.items():
            if not len(S):continue
            x0,y0,x1,y1=S.T;dx,dy=x1-x0,y1-y0;A=dx*dx+dy*dy;B=2*(x0*dx+y0*dy);Cc=x0*x0+y0*y0-ro*ro;disc=B*B-4*A*Cc
            for k in np.flatnonzero((disc>=0)&(A>1e-12)):
                for sgn_ in (-1,1):
                    t=(-B[k]+sgn_*math.sqrt(disc[k]))/(2*A[k])
                    if not 0<=t<=1:continue
                    p=np.array([x0[k]+t*dx[k],y0[k]+t*dy[k]]);e=np.array([dx[k],dy[k]])/math.sqrt(A[k]);nn=np.array([e[1],-e[0]])
                    if inside(S,p+.05*nn):nn=-nn                                          # outward normal of the face
                    ev.append((math.degrees(math.atan2(p[1],p[0])),i,nn,p))
        for start,dirs in [(180.,(1,-1))]+([(0.,(1,-1))] if ro>=R_SH-WALL+.4 else []):   # from the free band behind the robot, and (outside the wall) from the seam gap
            for d in dirs:   # +1: the object moves +az relative to the robot (robot spins clockwise)
                cand=[]
                for a,i,nn,p in ev:
                    tdir=np.array([-p[1],p[0]])/ro
                    if (d*tdir)@nn>=0:continue                                            # leaving a part, not entering
                    da=((a-start)*d)%360
                    if da>1e-6:cand.append((da,a,i,nn,tdir))
                if not cand:continue
                da,a,i,nn,tdir=min(cand,key=lambda c:c[0])
                rec=dict(z=z,r_o=ro,start_az=start,spin='cw' if d>0 else 'ccw',first_face_part=i,az=round(a,2))
                if i.startswith('P_rear_shell'):rec.update(verdict='fixed rear face (unsensed)')
                else:
                    side='left' if i.endswith('left') else 'right';tr=np.array([ft[0],(1 if side=='left' else -1)*ft[1]])
                    v=d*tdir;slip=v-(v@nn)*nn;slip=slip/np.linalg.norm(slip) if np.linalg.norm(slip)>1e-9 else slip
                    mu=.5 if abs(a)<AZF else .2;a0=float(-nn@tr);a1=float(slip@tr);along=a0+mu*a1
                    muc=round(-a0/a1,2) if a0>0 and a1<0 else (9. if a0>0 else 0.)
                    pf=polar(ro,a);notch=abs(a)<AZW+1. and abs(pf[1])<=win_hw(pf[0])+.5 and win_z(pf[0])[0]-.5<=z<=win_z(pf[0])[1]+.5   # a camera window edge face (task F): it must be sensed like any other face
                    land=abs(a)<3. and ro<R_SH-WALL+1.2+.05   # the 1.2 mm radial land left at the seam end under the lead-in ramp (thinnest printable wall)
                    rec.update(mu=mu,along_travel=round(along,3),along_travel_frictionless=round(a0,3),mu_crit=muc,camera_window_edge=notch,seam_end_land=land,
                        verdict='moves the half (sensed)' if along>=.05 else ('friction-limited (unsensed at mu %.1f)'%mu if a0>0 else 'rigid snag'))
                spin.append(rec)
snags=[r for r in spin if not r['verdict'].endswith('(sensed)') and not r.get('seam_end_land')]
land_snags=[r for r in spin if not r['verdict'].endswith('(sensed)') and r.get('seam_end_land')]
limited=[r for r in spin if r['verdict'].startswith('friction-limited')]
notch_snags=[r for r in spin if not r['verdict'].endswith('(sensed)') and r.get('camera_window_edge')]
win_faces=[r for r in spin if r.get('camera_window_edge')]
# Pressed spin at the bumper end: an object at r_o (inside the skin, near the rear-shell radius) carried round by a spin in both directions.
# The bumper's outer radius at each azimuth and travel comes from rays cast at the shifted half; the half takes the smallest travel that
# keeps its skin at r_o. Pass: the first rigid contact is the bumper at its 5 mm stop (switch made) or there is no rigid contact at all;
# a contact with the fixed shell while the half is under 2 mm of travel fails.
# With planar play the half's skin at each travel is taken from its most outward (side shift, yaw) pose (conservative: needs more travel).
PLAYEX=[(0.,0.),(SPLAY,0.),(-SPLAY,0.),(SPLAY,-PHMAX),(-SPLAY,PHMAX),(0.,PHMAX/2),(0.,-PHMAX/2)]
CBS=[Caster(['P_bumper_left','bumper_foam_left'],pose={i:(1,(0.,e_,p_)) for i in ('P_bumper_left','bumper_foam_left')}) for e_,p_ in PLAYEX];CR=Caster(['P_rear_shell_left'])
AZS=np.round(np.arange(78,104.01,.1),2);SV=np.round(np.arange(0,BT+1e-9,.05),2);pressed=[]
for z in SPIN_Z:
    RBm=np.zeros((len(AZS),len(SV)));RS_=np.zeros(len(AZS))
    for ia,a in enumerate(AZS):
        c,s_=math.cos(math.radians(a)),math.sin(math.radians(a));d=np.array([-c,-s_,0.])
        O=np.array([[200*c-sv*ft[0],200*s_-sv*ft[1],z] for sv in SV])
        RBm[ia]=np.max([np.where(np.isfinite(t),200-t,0.) for t in [CB.hits(O,d,CB.sel(z-.2,z+.2,a,6.))[0] for CB in CBS]],0)
        t,_=CR.hits(np.array([[200*c,200*s_,z]]),d,CR.sel(z-.2,z+.2,a,6.));RS_[ia]=200-t[0] if np.isfinite(t[0]) else 0.
    for ro in [153.0,153.5,154.0,154.5,155.0,155.5]:
        for dirn,order in [(f'toward the shell (+az) from az {a0}',range(int(round((a0-78)/.1)),len(AZS))) for a0 in range(78,93,2)]+[('from behind (-az) from az 104',range(len(AZS)-1,-1,-1))]:
            res=dict(z=z,r_o=ro,direction=dirn,verdict='no rigid contact (passes)',max_travel_mm=0.)
            for ia in order:
                ok_s=np.flatnonzero(RBm[ia]<=ro+1e-9);sreq=float(SV[ok_s[0]]) if len(ok_s) else None
                if sreq is None:res.update(verdict='bumper at its hard stop: sensed rigid stop',az=float(AZS[ia]),travel_mm=BT);break
                res['max_travel_mm']=max(res['max_travel_mm'],sreq)
                if RS_[ia]>ro+1e-9:res.update(verdict='fixed shell contact with the switch made' if sreq>=2.0 else 'fixed shell contact, switch open (unsensed)',az=float(AZS[ia]),travel_mm=sreq);break
            res['ok']=not res['verdict'].endswith('(unsensed)');pressed.append(res)
spin_sweep=dict(objects_r_mm=SPIN_RO,heights_mm=SPIN_Z,first_faces=spin,rigid_snags=snags,friction_limited=limited,camera_window_edge_faces=len(win_faces),camera_window_edge_snags=notch_snags,seam_end_land_snags=land_snags,
    seam_end_land_note=f'Residual: an object thinner than the {2*G:g} mm seam that sits {R_SKIN-(R_SH-WALL+1.2):.1f}-{R_SKIN-(R_SH-WALL):.1f} mm inside the skin (r {R_SH-WALL:.1f}-{R_SH-WALL+1.2:.1f}) meets the 1.2 mm radial land under the seam-end ramp when the robot spins: unsensed. Thinner land would not print (1.2 mm gate); listed in summary.residual_unsensed.',
    pressed_spin_at_bumper_end=pressed,pressed_spin_ok=all(r['ok'] for r in pressed),ok=not snags and all(r['ok'] for r in pressed),
    verdict_rule='sensed only if the contact force (normal plus object friction mu along the slip) has a component >= 0.05 along the half travel; friction-limited contacts count as failures. The camera window edge faces (task F) are flagged camera_window_edge and must be sensed like any other face (their 62 deg flare gives the push along the travel).',
    planar_play='pressed spin: the half skin is taken from its most outward planar-play pose at every travel',
    controller_rule=f'A spin with a bumper pressed must stop (or reverse) before the pressed contact passes az ±90 deg; do not start a spin with an object within {R_SKIN-R_RS:.1f} mm outside the rear shell (r < {R_SKIN}) behind the axle.',
    note=f'First face met by an object fixed in the world while the robot spins, starting from the free band behind the robot (r {R_RS}..{R_SKIN}) and from the seam gap. Object friction mu 0.5 on the foam (az < 70), 0.2 on the PTFE-taped skin (az >= 70). '
         f'The az ±78..90 tapers end at R{R_TIP}, outside the start of the rear shell (ramped from R{R_RS-RS_LEAD[0]}): an object leaving a pressed bumper end never meets a fixed face, and one coming from behind meets the ramp. The seam ends are 20 deg lead-in ramps, so an object crossing the seam pushes the other half.')
print('spin sweep',round(time.time()-T0),'s',flush=True)

# Rear shell: rib azimuth scan, clearances, bed fit.
ECL=box(5,146,ZB0-2,70,36,ZB1-ZB0+4)   # the bumper end region and the shell start (left; the right is its mirror)
BEc,RSc=shapes['P_bumper_left'].intersect(ECL),shapes['P_rear_shell_left'].intersect(ECL)
endplay=sorted((round(pose_shape(BEc,p_,1).distance(RSc),3),p_) for p_ in extremes([0.,1.25,2.5,3.75,4.5,4.9,5.]))
rsr=dict(radius_mm=R_RS,wall_mm=RWALL,top_rim_mm=10,azimuth_deg=[RS_A0,180],z_mm=[ZB0,ZB1],lead_in=dict(depth_mm=RS_LEAD[0],over_deg=RS_LEAD[1],r_at_start_mm=R_RS-RS_LEAD[0]),backer_flange=dict(r_mm=RFL[:2],az_deg=RFL[2:]),
    bumper_tip_overlap_over_shell_start_mm=round(R_TIP-(R_RS-RS_LEAD[0]),2),fins=dict(thickness_mm=FIN_T,depth_mm=FIN_D,z_wall_mm=[FIN_Z0,ZB1-14.5],azimuths_deg=FINS,scan=dict(left=fin_rows_left,right=fin_rows_right),rule='>= 1.5 mm to every other part and wire; >= 6 deg from a foot rib'),
    rib_gusset=dict(thickness_mm=GUSR,tip_z_mm=ZGR),
    bumper_end_gap_with_planar_play_mm=dict(min=endplay[0][0],pose_s_e_yawdeg=[endplay[0][1][0],round(endplay[0][1][1],3),round(math.degrees(endplay[0][1][2]),3)],required_mm=1.5),
    bumper_tip_radius_mm=R_TIP,rib_azimuths_deg=RIBS,rib_width_mm=RIBW,rib_scan=rib_scan,
    rib_scan_note='Per side, real hardware: hard = rib footprint vs rear cliff windows + ToF cones + under-base cliff screw heads, caster housing, tyres, countersunk Jetson standoff screw heads; plan = vs the lid-post feet (the deck-era saddle footprints), harness lanes and lid posts (z-separated); boss wall = Ø8 boss vs every other P_base hole (>= 1.5).',
    bumper_end_gap_mm={f's{s_}':min(round(shapes['P_rear_shell_'+sd].distance(shapes['P_bumper_'+sd].translate(mvec(sd,s_))),2) for sd in ['left','right']) for s_ in STATES},
    lap_radial_clearance_mm=round(shapes['P_rear_shell_left'].distance(shapes['P_rear_shell_right']),3),dowel_holes_mm=dict(tongue_press=3.1,right_slip=3.3),dowel=dict(size=f'Ø3 × {DOWEL_L:.0f}',press_length_mm=round(R_RS_IN-.4-max(R_RS-.1-DOWEL_L,R_RS_IN-4.4),2),slip_length_mm=round(R_RS-.1-R_RS_IN,2),pad='2 mm inner pad, 45 deg underside, around each dowel'),
    power_leads_max_r_mm=round(max(rmax(MESH[i]) for i in ('W01','W04')),2))
rsr['bumper_end_gap_ok']=min(rsr['bumper_end_gap_mm'].values())>=2.0 and endplay[0][0]>=1.5
# Dowels: each must sit in a hole in BOTH segments (no solid overlap with either, and within 0.2 mm of each hole wall).
rsr['dowels']=[dict(dowel=f'rear_shell_dowel_{k}',az_deg=a,z_mm=z,**{f'overlap_{sd}_mm3':round(shapes[f'rear_shell_dowel_{k}'].intersect(shapes['P_rear_shell_'+sd]).Volume(),3) for sd in ('left','right')},
    **{f'gap_to_{sd}_mm':round(shapes[f'rear_shell_dowel_{k}'].distance(shapes['P_rear_shell_'+sd]),3) for sd in ('left','right')}) for k,(a,z) in enumerate(DOWELS)]
rsr['dowels_ok']=all(d['overlap_left_mm3']<=1e-3 and d['overlap_right_mm3']<=1e-3 and d['gap_to_left_mm']<=.2 and d['gap_to_right_mm']<=.2 for d in rsr['dowels'])

# Mass and centre of mass: printed parts PETG 1.27 g/cm3 (fill 1.0 when the mean wall 2V/A <= 3 mm, else 0.62) at their solid
# centroids; bought parts use the simulator record masses (robot-parameters.json) at their CAD centroids, the power stage and the
# controller group spread over their real parts; steel hardware from CAD volume (7.85 g/cm3). No ballast.
def ctr(ids):
    w=[(shapes[i].Volume(),shapes[i].Center()) for i in ids];V=sum(a for a,_ in w)
    return [round(sum(a*c.x for a,c in w)/V,1),round(sum(a*c.y for a,c in w)/V,1),round(sum(a*c.z for a,c in w)/V,1)]
segs=[(math.dist(a,b),[(a[k]+b[k])/2 for k in range(3)]) for c in cables if c.get('modelled',True) for a,b in zip(c['points'],c['points'][1:])]
wire_c=[round(sum(l*m[k] for l,m in segs)/sum(l for l,_ in segs),1) for k in range(3)]
STEEL=[p['id'] for p in parts if p['category']=='metal' and p['id'].startswith(('motor_base','motor_face','lidar_tray_screw','bumper_pin','bumper_ball','bumper_race','base_race','insert_','bumper_switch_screw','tower_screw','rear_shell_screw','rear_shell_dowel','caster_screw','bay_screw','jetson_screw','tray_screw','lid_post_screw','lid_screw'))]
PW=[('p03_regulator',25),('bay_board',40),('bay_drv_0',6),('bay_drv_1',6)]   # same 0.085 kg as the deck-era power stage: P03, the bay board (relay, fuses, capacitor, connectors) and the two DRV8874 carriers
CTRL=[('Pico and IMU on the bay board',.039,['bay_pico','bay_imu']),('cliff sensors',.008,[f'cliff_{k}' for k in range(1,5)]),('bumper switches',.003,['bumper_switch_left','bumper_switch_right']),('bay board headers and input stages',.005,['bay_board'])]
def bought(bat=1.,jet=1.,extra=()):
    B=[('Battery',.383*bat,ctr(['battery']),'Battery'),('Jetson + cooler',.22*jet,ctr(['jetson','jetson_heatsink','fan']),'Jetson + cooler'),('Left motor',.09,ctr(['motor_left']),'Left motor'),('Right motor',.09,ctr(['motor_right']),'Right motor')]
    B+=[('Controller and sensors: '+n,m,ctr(ids),'Controller and sensors') for n,m,ids in CTRL]
    B+=[('Power boards + relay: '+i,.085*g/sum(g for _,g in PW),ctr([i]),'Power boards + relay') for i,g in PW]
    B+=[('Motor toggle and master switches',.03,ctr(['motor_switch','master_switch','arm_button']),'Motor toggle and master switches'),('Wiring + connectors',.1,wire_c,'Wiring + connectors'),
        ('Fasteners, inserts, pins, springs, bearings, races and balls (steel, CAD volume)',round(sum(shapes[i].Volume() for i in STEEL)*7.85e-6,4),ctr(STEEL),'Fasteners'),('Caster housing',.03,ctr(['caster_housing']),'Caster housing'),
        ('Camera',.015,ctr(['camera_board','camera_lens']),'Camera'),('D200 LiDAR',.101,ctr(['lidar','lidar_turret','lidar_motor','lidar_cap']),'D200 LiDAR'),('LiDAR adapter and added cables',.035,ctr(['lidar_adapter']),'LiDAR adapter and added cables'),
        ('Bumper foam F02 (EVA ~0.07 g/cm3, estimate)',round(.07e-6*(shapes['bumper_foam_left'].Volume()*2),4),ctr(['bumper_foam_left','bumper_foam_right']),'Bumper foam'),
        ('Wheels (2 × 0.035)',.07,[AX,0,40],'Wheels'),('Caster ball',.068,[CX,0,RB],'Caster ball')]
    return [dict(item=n,mass_kg=round(m,5),position_mm=c,sim_item=s) for n,m,c,s in B]+[dict(item=n,mass_kg=m,position_mm=c,sim_item='Extra') for n,m,c in extra]
FILL_T=(3.0,3.6)   # mean wall 2V/A: <= 3.0 solid; >= 3.6 (= 2 sides × 4 perimeters × 0.45 mm, the slicer setting) 62 % effective; linear in between (no step: the
                   # former step at 3.0 gave the mirror-image rear shells 0.62 and 1.0 for walls of 3.03 and 2.99 mm)
def fill_of(t,step=False):
    if step:return 1.0 if t<=FILL_T[0] else .62
    return 1.0 if t<=FILL_T[0] else (.62 if t>=FILL_T[1] else round(1.0-.38*(t-FILL_T[0])/(FILL_T[1]-FILL_T[0]),3))
def printed(fill=None,step=False):
    out=[]
    for p in parts:
        if p['printable']:
            s=shapes[p['id']];t=2*s.Volume()/s.Area();f=fill if fill else fill_of(t,step);c=s.Center()
            out.append(dict(item=p['id'],mass_kg=round(s.Volume()*1.27e-6*f,5),position_mm=[round(c.x,1),round(c.y,1),round(c.z,1)],mean_wall_mm=round(t,2),fill=f,sim_item=p['id']))
    return out
def com_of(recs):
    MT=sum(r['mass_kg'] for r in recs);return MT,[sum(r['mass_kg']*r['position_mm'][k] for r in recs)/MT for k in range(3)]
recs=printed()+bought();MT,COMv=com_of(recs)
cl=(AX-COMv[0])/(AX-CX);g=9.81
def onset(c,slope=0.):return g*((AX-c[0])*math.cos(math.radians(slope))-c[2]*math.sin(math.radians(slope)))/c[2]
sens={}
for name,rr in [('all printed parts fill 1.0',printed(1.0)+bought()),('battery 10 % lighter',printed()+bought(bat=.9)),('Jetson 30 % lighter',printed()+bought(jet=.7)),
                ('+50 g at x=+100, z=60',printed()+bought(extra=[('extra',.05,[100,0,60])])),('fill 1.0 and battery 10 % lighter',printed(1.0)+bought(bat=.9))]:
    m_,c_=com_of(rr);sens[name]=dict(com_x_mm=round(c_[0],2),forward_pitch_onset_m_s2=round(onset(c_),2),forward_pitch_onset_5deg_downhill_m_s2=round(onset(c_,5),2))
_BH=[r for r in recs if r['item'] in ('P_bumper_left','P_bumper_right')];BHX=sum(r['mass_kg']*r['position_mm'][0] for r in _BH)/sum(r['mass_kg'] for r in _BH)
com=dict(total_mass_kg=round(MT,3),body_mass_without_wheels_and_caster_ball_kg=round(MT-.07-.068,3),printed_mass_kg=round(sum(r['mass_kg'] for r in recs if 'fill' in r),3),com_mm=[round(v,1) for v in COMv],
    com_x_limit_mm=-12,com_x_sensitivity_limit_mm=-10,downhill_pitch_onset_min_m_s2=round(3*PARAMS['drive_model']['setpoint_ramp_m_s2'],2),
    com_x_limit_source='user decision 2026-10-11 (task F): nominal <= -12 mm, every sensitivity case <= -10 mm (was -15 / -14), approved for an LD14 trial at x 109 and kept for x 54; '
                       'every case keeps its 5 deg-downhill pitch onset >= 3 × the DriveCore braking ramp (0.25 m/s2). The simulator sync guard (COM_LIMIT_M) checks the same two limits.',
    caster_load_fraction=round(cl,3),rear_caster_load_N=round(cl*MT*g,2),drive_wheel_load_N=round((1-cl)*MT*g,2),
    forward_pitch_onset_decel_m_s2=round(onset(COMv),2),forward_pitch_onset_on_5deg_downhill_m_s2=round(onset(COMv,5),2),backward_tip_onset_accel_m_s2=round(g*(COMv[0]-CX)/COMv[2],2),
    braking_requirement=(lambda cap,wd:f'Keep forward braking below the pitch onset (level {onset(COMv):.2f}, 5 deg downhill {onset(COMv,5):.2f} m/s2 nominal; {wd:.2f} on the downhill in the '
                        f'worst sensitivity case): DriveCore ramps every wheel setpoint, braking included, at {PARAMS["drive_model"]["setpoint_ramp_m_s2"]} m/s2 and a disarm coasts; its torque cap alone '
                        f'would allow about {cap:.2f} m/s2, {"above" if cap>wd else "below"} that worst downhill onset, so keep the ramp in the firmware, never short-brake the motors (the driver coasts '
                        'on disarm), and slow down on ramps.')(2*PARAMS["drive_model"]["max_torque_nm"]/(PARAMS["wheel_diameter"]/2000)/(MT+2*PARAMS["drive_model"]["axle_inertia_kg_m2"]/(PARAMS["wheel_diameter"]/2000)**2),
                        min(x['forward_pitch_onset_5deg_downhill_m_s2'] for x in sens.values())),
    placement=dict(note='No ballast (user decision): the existing heavy parts sit as far aft as the layout allows.',
        battery=dict(centre_x_mm=BX,limit='tray rear wall flush with the rear base edge (x -124); P_base is 244 mm long on the 250 mm MK3S bed, so the base cannot grow aft'),
        jetson=dict(carrier_centre_mm=[round(v,2) for v in JC[:2]],origin_mm=list(JO),carrier_x_mm=[JPCB[0],JPCB[1]],pcb_underside_z_mm=JO[2],fan_centre_mm=[round(v,2) for v in JFAN],mounting_holes_mm=[list(h) for h in JHOLES],
            limit=f'NVIDIA B01 STEP turned +90 deg (I/O row forward, barrel jack on the right), on two 24 mm standoff stacks from P_base (front) and two 20 mm F-F standoffs on the bay board (rear, clamping it): carrier x {JPCB[0]:g}..{JPCB[1]:g}. Its x was set in L1 by the COM sensitivity guard of that time (every case <= -14 mm): '
                  'with the spec position (origin x 38, carrier x -24..55) the L1 model gave COM x -15.6 and -13.4 in the +50 g-forward case (a fail then); origin x 30 was the most forward that passed (-14.1), origin x 25 kept 0.5 mm of margin there. '
                  'The rule now is the user\'s (task F, 2026-10-11): nominal <= -12 mm, every case <= -10 mm (com_x_limit_source); the Jetson has not moved. '
                  f'Bay board (B1): the Jetson stays at z {JO[2]:g}; its rear standoffs clamp the board, whose under-carrier modules (DRV8874 × 2, GY-521) stay <= z {UNDER_CARRIER_Z-1.5:.2f}.'),
        power_stage=f'bay board x {min(p[0] for p in BAY_OUTLINE):g}..{max(p[0] for p in BAY_OUTLINE):g} at base level under the Jetson, directly ahead of the battery tray (no room further aft); P03 turned 90 deg in the front bay (x {P03C[0]-P03S[0]/2:g}..{P03C[0]+P03S[0]/2:g}) ahead of the board\'s tongue',
        controller_lidar_camera=f'Task F: LD14 turned round, spin axis x {LX:g}, its 0.1 m blind circle inside the pressed bumper (user decision 2026-10-11, option B; COM rule relaxed to nominal <= -12 / worst <= -10 mm; the old spec x 20), camera lens face x {CAMO.x:g} in the bumper window; the bumper halves ({sum(r["mass_kg"] for r in _BH):.2f} kg at x ~{BHX:.0f}) are the main forward mass'),
    sensitivity=sens,records=recs,method='Printed: CAD volume × 1.27 g/cm3 × fill (mean wall 2V/A: 1.0 up to 3.0 mm, 0.62 from 3.6 mm = 2 × 4 perimeters × 0.45, linear in between) at the solid centroid. Bought: simulator record masses at CAD centroids (power stage and controller group spread over their parts). Steel from CAD volume. No ballast.')
com['sensitivity_worst_com_x_mm']=max(v['com_x_mm'] for v in sens.values());com['sensitivity_worst_case']=max(sens,key=lambda k:sens[k]['com_x_mm'])
com['sensitivity_min_forward_pitch_onset_m_s2']=min(v['forward_pitch_onset_m_s2'] for v in sens.values())
com['sensitivity_min_5deg_downhill_pitch_onset_m_s2']=min(v['forward_pitch_onset_5deg_downhill_m_s2'] for v in sens.values())
com['placement']['jetson']['limit']+=f' This build: COM x {com["com_mm"][0]:g}, worst case {com["sensitivity_worst_com_x_mm"]:g} ({com["sensitivity_worst_case"]}).'
_m0,_c0=com_of(printed(step=True)+bought());_m1,_c1=com_of(printed(step=True)+bought(extra=[('extra',.05,[100,0,60])]))
com['fill_model_check']=dict(previous_step_rule='fill 1.0 if 2V/A <= 3.0 mm else 0.62 (step)',com_x_mm=round(_c0[0],2),total_mass_kg=round(_m0,3),forward_pitch_onset_m_s2=round(onset(_c0),2),
    worst_case_plus_50g_front_com_x_mm=round(_c1[0],2),parts_affected={r['item']:r['fill'] for r in recs if 'fill' in r and r['fill'] not in (1.0,.62)},
    note='Not part of the pass criterion: the step rule is what the previous rounds used. Under it the rear shells (mean walls just over 3.0 mm, printed with 4 perimeters, i.e. solid) drop to 62 %; weigh the printed shells to settle it.')
com['ok']=com['com_mm'][0]<=com['com_x_limit_mm'] and com['sensitivity_worst_com_x_mm']<=com['com_x_sensitivity_limit_mm'] and com['sensitivity_min_5deg_downhill_pitch_onset_m_s2']>=com['downhill_pitch_onset_min_m_s2']
# Interlayer stress (rough, see interlayer()): impact at the hard stop at the DriveCore speed and at twice it, and a 14 N wheel-stall push,
# hit at ZHL (11 mm under the top) and at the top rim (ZB1).
IL_LIMIT=round(25/1.5,1)   # PETG interlayer (z) tensile strength ~25 MPa for well-fused layers, safety factor 1.5
vmax=PARAMS['max_speed_mps']
bcases=[interlayer(v,MT,zh) for v in (vmax,2*vmax) for zh in (ZHL,ZB1)]
bstall={f'z{zh:.0f}':dict(gusset_root_MPa=round(7*(zh-RZ[1])/(GUST*29**2/6),1),wall_above_gusset_tip_MPa=round(14*(zh-min(GTIP))/(80*WALL2**2/6),1)) for zh in (ZHL,ZB1)}
# Rear shell (fixed, bare PETG): the moment of a hit reaches the base only through the two foot-rib gussets per side; the critical layer
# sections are at the gusset tip (z ZGR: wall + nearby fins over an 80 mm effective width) and at the gusset root (z 29: wall + gusset).
# The outer face is in tension for a push from outside. Reverse impact: the robot's kinetic energy into the shell's own cantilever stiffness
# above the gusset tip (no foam, no travel: conservative).
def rs_case(sd,a,zh,F=None,v0=None):
    e=base_edge_r(a);dg=R_RS_IN+.5-(e+1);nf=sum(1 for f_ in FINS[sd] if abs(abs(f_)-abs(a))*math.pi/180*R_RS_IN<=40)
    It,ct=tsec(80,RWALL,[(FIN_T,FIN_D)]*nf);Ir,cr=tsec(80,RWALL,[(GUSR,dg)]);Lt=zh-ZGR
    if F is None:E_=.5*MT*v0*v0;k=3*2000*It/Lt**3;F=math.sqrt(2*E_*1000*k)
    return dict(side=sd,rib_az=a,hit_z_mm=zh,force_N=round(F,1),fins_within_40mm=nf,gusset_depth_at_root_mm=round(dg,1),
        tip_section_MPa=round(F*Lt/(It/ct),1),root_section_MPa=round(F*(zh-29)/(Ir/cr),1),wall_alone_at_tip_MPa=round(F*Lt/(80*RWALL**2/6),1))
rscases=[rs_case(sd,a,zh,F=14.) for sd in ('left','right') for a in RIBS[sd] for zh in (ZHL,ZB1)]+[rs_case(sd,a,zh,v0=vmax) for sd in ('left','right') for a in RIBS[sd] for zh in (ZHL,ZB1)]
# Motor-mount tabs: each wheel load reaches the base through two 9 × 5 mm tabs printed across their layers ('+y down'); the gusset
# deepens the root section to 13 mm. Static and a 3 g bump (8-10 mm step or 5 cm drop).
Nw1=(1-(AX-COMv[0])/(AX-CX))*MT*9.81/2;Mtab=Nw1/2*10+Nw1*8.5/2;Stab=9*(MTAB+8)**2/6;Stab0=9*MTAB**2/6
interlayer_check=dict(bumper=dict(cases=bcases,stall_push_14N=bstall),rear_shell=dict(cases=rscases,max_MPa=max(max(c['tip_section_MPa'],c['root_section_MPa']) for c in rscases),
        rear_centre_note='At az 180 (between the ±155 deg gussets) the shell carries a push horizontally round the arc to the gussets (bending within the layers, not across them); the 10 mm top rim and the fins act as a ring beam.'),
    motor_mount=dict(wheel_load_N=round(Nw1,2),tab_root_moment_Nmm=round(Mtab,1),section_modulus_mm3=round(Stab,1),static_MPa=round(Mtab/Stab,2),three_g_MPa=round(3*Mtab/Stab,2),three_g_with_Kt2_MPa=round(6*Mtab/Stab,2),
        tab_alone_three_g_MPa=round(3*Mtab/Stab0,2),note='The 45 deg gusset carries the root moment; beyond the gusset (y < 96) only the screw clamp load acts.'),
    limit_MPa=IL_LIMIT,
    assumptions=f'Bumper: hit at z {ZHL:g} or {ZB1:g} at the 5 mm hard stop; 2 mm to trip, 10 ms latency, 2.0 m/s2 braking; series stiffness: EVA patch 450 N/mm and two 4 mm gussets in bending; the wall above a gusset tip acts over an 80 mm effective width of the {WALL2} mm upper wall + 6 mm rim. '
        f'cases[].consistent_stiffness is informational (the gate uses the model above): the foam is in series only for hits at or below its top (z {ZFT:g}; both hit heights are above it), and the wall above the gusset tip adds its own bending stiffness 3E(80 t^3/12)/(z_hit - {min(GTIP):g})^3, E 2000 MPa; it is softer, so it gives a smaller force. '
        'Rear shell: 80 mm effective width at each foot-rib gusset, T-section with the fins inside that width. These are order-of-magnitude numbers: a printed half must pass the release proof test (50 N at z '+f'{ZB1:g}'+' into the stop, 5 cm drop).',
    mitigations=f'Bumper: 4 mm gussets to z {GTIP[0]} (rail) and {GTIP[1]} (arm), {WALL2} mm upper wall; rear shell: {GUSR:.0f} mm rib gussets to z {ZGR:.0f} and {FIN_T:.0f} × {FIN_D:.0f} mm fins; 4 perimeters, ~245 °C, cooling <= 30 % below the rim; firmware speed cap 0.15 m/s; never lift by the shells.')
interlayer_check['ok_at_drivecore_speed']=max(max(c['gusset_root_MPa'],c['wall_above_gusset_tip_MPa']) for c in bcases if c['v0_mps']==vmax)<=IL_LIMIT and interlayer_check['rear_shell']['max_MPa']<=IL_LIMIT and interlayer_check['motor_mount']['three_g_with_Kt2_MPa']<=IL_LIMIT
# Rear cliff lead over the loaded caster in reverse at the FIRMWARE REVERSE SPEED LIMIT (parameters reverse_speed_limit_m_s). DriveCore disarms on a hazard (tl = tr = 0), so the robot
# COASTS on the simulator's dynamic brake (0.010 N m s/rad per wheel, plus 0.003 N m rolling drag) with the reflected axle inertia in the
# effective mass. Required lead = latency travel + coast distance + half the VL53L0X floor-spot radius. The 'ok' uses the coast WITHOUT
# rolling drag (conservative).
DM=PARAMS['drive_model'];rw_=PARAMS['wheel_diameter']/2000;lat_s=.066
lead=round(min(CX-x for x,y in CLIFF[2:]),1)
SPOT=round(.5*CZ*math.tan(math.radians(PARAMS['cliff_fov_deg']/2)),1)
m_eff=MT+2*DM['axle_inertia_kg_m2']/rw_**2;tau=m_eff*rw_**2/(2*DM['dynamic_brake_nm_per_rad_s']);v_d=DM['rolling_drag_nm']*rw_/DM['dynamic_brake_nm_per_rad_s']
def coast_mm(v,drag=True):
    if not drag:return 1000*v*tau
    ts=tau*math.log((v+v_d)/v_d);return 1000*(tau*v-v_d*ts)
need_c=lambda v,drag=False:1000*v*lat_s+coast_mm(v,drag)+SPOT
a_brake_max=2*DM['max_torque_nm']/rw_/m_eff
tipb=round(g*(COMv[0]-CX)/COMv[2],2)
vsafe=max([v for v in np.arange(.01,.3001,.005) if need_c(v)<=lead],default=0.)
vrev=PARAMS['reverse_speed_limit_m_s']
# Spin cliff coverage: a spin in place next to a straight edge at distance D from the axle; a start heading is unsafe when a wheel
# centre-plane (0, ±114) crosses the edge before any cliff sensor has half its floor spot over it (both spin directions).
def spin_band(D,dirn,allow):
    bad=0;W_=[(0.,PARAMS['wheel_track']/2),(0.,-PARAMS['wheel_track']/2)]
    for p0 in np.arange(0,360,.5):
        c0,s0=math.cos(math.radians(p0)),math.sin(math.radians(p0))
        if any(c0*x-s0*y>D for x,y in W_) or any(c0*x-s0*y>D+allow for x,y in CLIFF):continue
        seen=False
        for dps in np.arange(0,360,.25):
            ps=math.radians(p0+dirn*dps);c,s_=math.cos(ps),math.sin(ps)
            if any(c*x-s_*y>D+allow for x,y in CLIFF):seen=True;break
            if any(c*x-s_*y>D for x,y in W_):break
        bad+=0 if seen else 1
    return bad*.5
spin_cov=[dict(edge_distance_mm=D_,unsafe_start_heading_band_deg=dict(ccw=spin_band(D_,1,SPOT),cw=spin_band(D_,-1,SPOT))) for D_ in (100,104,106,108,110,112,113)]
caster_layout=dict(rear_caster_xy=[CX,0],ball_radius_mm=RB,ball_bottom_z_mm=round(float(MESH['caster_ball'][:,2].min()),3),front_support='none (user decision): the centre of mass sits behind the axle',
    rear_cliff_lead_over_caster_reverse_mm=lead,rear_cliff_xy=[list(c) for c in CLIFF[2:]],front_cliff_lead_over_wheels_mm=round(min(x for x,y in CLIFF[:2])-AX,1),
    stopping=dict(reverse_speed_limit_m_s=vrev,reverse_speed_source=PARAMS['reverse_speed_limit_source'],max_forward_speed_mps=vmax,forward_speed_source=PARAMS['max_speed_source'],
        controller='DriveCore.h: a hazard disarms (tl = tr = 0); the wheels then see the simulator dynamic brake (coast)',
        detection_latency_s=lat_s,spot_allowance_mm=SPOT,effective_mass_kg=round(m_eff,3),coast_time_constant_s=round(tau,3),
        coast_without_rolling_drag=dict(speed_m_s=vrev,latency_travel_mm=round(1000*vrev*lat_s,1),distance_mm=round(coast_mm(vrev,False),1),required_lead_mm=round(need_c(vrev),1),margin_mm=round(lead-need_c(vrev),1),ok=lead>=need_c(vrev)),
        coast_with_rolling_drag=dict(speed_m_s=vrev,distance_mm=round(coast_mm(vrev,True),1),required_lead_mm=round(need_c(vrev,True),1),ok=lead>=need_c(vrev,True)),
        max_coast_safe_reverse_speed_mps=round(vsafe,3),
        if_reverse_were_not_capped=dict(speed_m_s=vmax,required_lead_mm=round(need_c(vmax),1),ok=lead>=need_c(vmax),note='for information: without the firmware reverse cap a 0.15 m/s reverse would need this lead; the firmware cap is REQUIRED'),
        active_brake_limit=dict(max_decel_with_torque_cap_m_s2=round(a_brake_max,2),note=f'DriveCore max_torque {DM["max_torque_nm"]} N m per wheel caps any active brake at {a_brake_max:.2f} m/s2; the coast is what the controller does today.'),
        backward_tip_onset_m_s2=tipb,max_drive_accel_m_s2=round(a_brake_max,2),
        note='Latency = two VL53L0X timing budgets. Reflected axle inertia (0.0005 kg m2 per wheel) is in the effective mass for the coast and the torque-limited brake.'),
    oblique_edges=dict(front=dict(sensor_xy=list(CLIFF[0]),wheel_contact_y_mm=PARAMS['wheel_track']/2,lead_formula_mm=f'{CLIFF[0][0]-AX} - {PARAMS["wheel_track"]/2-CLIFF[0][1]} tan(a)',
            max_edge_angle_deg_for_coast=round(math.degrees(math.atan((CLIFF[0][0]-AX-need_c(vmax))/(PARAMS['wheel_track']/2-CLIFF[0][1]))),1),
            edge_angle_deg_with_no_lead=round(math.degrees(math.atan((CLIFF[0][0]-AX)/(PARAMS['wheel_track']/2-CLIFF[0][1]))),1),
            limitation='An edge met more obliquely than these angles, or driven along within 42 mm of a wheel, drops a wheel before a front sensor trips (the spec fixes the front sensors at (105, ±72)). Needs a controller rule (slow down near mapped edges) or outboard front sensors: user decision.'),
        rear=f'Rear sensors lead the caster by {lead} + {abs(CLIFF[2][1])} tan(a): more for oblique edges.'),
    spin_cliff_coverage=dict(rows=spin_cov,method='Straight edge at distance D from the axle; every start heading (0.5 deg) and both spin directions; unsafe when a wheel centre-plane (y ±114) crosses the edge before any cliff sensor has half its floor spot over it (zero stopping distance assumed: optimistic).',
        limitation='Spinning in place next to an unmapped edge 104-113 mm from the axle can drop a wheel before any cliff sensor trips: the front sensors sit 56 deg from the wheel contacts. Needs a controller rule (no in-place spin near an unmapped edge after an oblique approach; back away first) or an extra sensor near az ±60-70 deg: user decision.'),
    cliff_radius_mm=[round(math.hypot(*c),1) for c in CLIFF],wheel_contact_radius_mm=PARAMS['wheel_track']/2)
caster_layout['ok']=lead>=30 and vrev<=vsafe and caster_layout['stopping']['coast_without_rolling_drag']['ok'] and abs(caster_layout['ball_bottom_z_mm'])<.05 and tipb>a_brake_max
# Threshold crossing (quasi-static, small pitch): square steps, traction mu 0.8. The body clearance is taken from every mesh vertex (not one
# assumed point); each case also needs the wheel torque within the DriveCore cap. 'pass' needs a 0.1 mu margin, >= 1.0 mm clearance and
# torque <= the cap; 'marginal' passes friction, torque and clearance without those margins.
BODYV=np.vstack([MESH[i] for i in SOLID if not i.startswith(('tire','wheel','caster_ball'))])
TCAP=DM['max_torque_nm']
def crossing(H,mu=.8):
    xg,zg=COMv[0]-AX,COMv[2];xc=CX-AX;rw=PARAMS['wheel_diameter']/2;W=MT*9.81;Nw=(1-(AX-COMv[0])/(AX-CX))*W
    pw=math.acos((rw-H)/rw);wheel=math.tan(pw);tq_w=Nw/2*rw/1000*math.sin(pw)
    pc=math.acos((RB-H)/RB);sp,cp=math.sin(pc),math.cos(pc)
    xe=xc+RB*sp;F=xg/(xe*cp);Nw_=1-F*cp;fwd_c=F*sp/Nw_ if Nw_>0 else 99.;tq_f=F*sp*W/2*rw/1000
    xe=xc-RB*sp;F=xg/(xe*cp-H*sp);Nw_=1-F*cp;rev_c=F*sp/Nw_ if Nw_>0 else 99.;tq_r=F*sp*W/2*rw/1000
    X_,Z_=BODYV[:,0],BODYV[:,2];xw=rw*math.sin(pw);xr=CX-RB*math.sin(pc)
    cl_fwd=float(np.min(Z_+H*(X_-CX)/(AX-CX)))          # wheels up on the step, caster on the floor: pitch about the caster contact (the tail drops)
    cl_rev=float(np.min(Z_+H*(X_-AX)/(CX-AX)))          # caster up on the step, wheels on the floor: pitch about the wheel contact (the nose drops)
    lvl_f=float(np.min(Z_[X_>xw-1]))-H;lvl_r=float(np.min(Z_[X_<xr+1]))-H   # level approach: nothing ahead of the first tyre / ball contact may hit the riser
    cases={}
    for d,cmu,cl_,tq_c,lab in [('forward',fwd_c,min(cl_fwd,lvl_f),tq_f,'lowest body point while the wheels are up (pitch about the caster), or the riser on the level approach'),('reverse',rev_c,min(cl_rev,lvl_r),tq_r,'lowest body point while the caster is up (pitch about the wheels), or the riser on the level approach')]:
        tq=max(tq_w,tq_c);ok=wheel<=mu and cmu<=mu and cl_>0 and tq<=TCAP;rob=ok and wheel<=mu-.1 and cmu<=mu-.1 and cl_>=1.0 and tq<=.9*TCAP
        why=[w for w,c in [('drive wheels cannot climb the edge (friction)',wheel>mu),('rear caster cannot be '+('dragged' if d=='forward' else 'pushed')+' over (friction)',cmu>mu),(lab+' strikes the floor',cl_<=0),
            (f'needs {tq:.3f} N m per wheel, above the DriveCore cap {TCAP} N m',tq>TCAP)] if c]
        cases[d]=dict(verdict='pass' if rob else ('marginal' if ok else 'fail'),**{'pass':rob},wheel_mu_required=round(wheel,3),caster_mu_required=round(cmu,3),body_floor_clearance_mm=round(cl_,2),clearance_point=lab,
            wheel_torque_required_Nm=dict(wheel_climb=round(tq_w,3),caster_over=round(tq_c,3),max=round(tq,3)),drivecore_cap_Nm=TCAP,
            fails_because=why or ([] if rob else ['margin: mu within 0.1 of 0.8, clearance under 1.0 mm or torque within 10 % of the cap']))
    return cases
cap={d:max([h for h in np.arange(.5,12.01,.5) if crossing(h)[d]['verdict']!='fail'],default=0.) for d in ('forward','reverse')}
threshold=dict(mu=.8,model='Planar quasi-static: the driven wheels climb if mu >= tan(phi_edge) and their torque N R sin(phi) is within the cap; the free ball caster is lifted by an edge reaction through its centre (ball-housing friction ignored), carried by wheel traction (torque = traction R / 2 per wheel); pitch about the support on the floor; clearance from every body mesh vertex. Momentum, tyre compliance and slip dynamics ignored.',
    step_8mm=crossing(8),step_10mm=crossing(10),capability_mm=dict(forward=cap['forward'],reverse=cap['reverse'],note='largest square step (0.5 mm steps) not rated fail'),
    motor_note='Pololu 4867 (99:1 MP 12 V) stall torque at 7.2 V is not in this package: the DriveCore cap 0.10 N m per wheel governs; verify the gearmotor rating before raising it.',
    options_for_user=['38 mm (r 19.05) caster ball: lowers the caster friction needed at 8-10 mm steps (needs a new caster and a lower mount)','raise or chamfer the bumper bottom edge at the front (z >= 23.5 for x > 140) for floor clearance when reversing over a step','raise the DriveCore torque cap after checking the gearmotor stall torque'])
print('com/threshold',round(time.time()-T0),'s',flush=True)


# Documentation generated from the CAD constants (parts-and-connections.json lines whose numbers the CAD sets), then the BOM consistency
# check: quantities of every fastener / hardware family, and the caster position and the sensor coordinates in the text. No ballast line (M03 removed).
BOMF=json.loads((ROOT/'parts-and-connections.json').read_text());BP={p['id']:p for p in BOMF['parts']}
RC=CLIFF[2];FC=CLIFF[0];CLs=caster_layout['stopping']
def fx(v):return f'{v:g}'
RCL_TXT=f'({fx(RC[0])}, ±{fx(abs(RC[1]))})';FCL_TXT=f'({fx(FC[0])}, ±{fx(abs(FC[1]))})'
BP['M02']['notes']=(f'ONE rear caster (no front support): ball centre at x = {fx(CX)} mm behind the axle; the caster carries about {cl:.2f} of the weight and the rear cliff sensors {RCL_TXT} lead it by {fx(lead)} mm in reverse. '
    f'DriveCore disarms on a hazard and the robot coasts on the motor dynamic brake: from the {vrev} m/s firmware reverse speed limit (parameters.json reverse_speed_limit_m_s; required) that needs {CLs["coast_without_rolling_drag"]["required_lead_mm"]} mm '
    f'({CLs["coast_without_rolling_drag"]["distance_mm"]} mm coast, no rolling drag, conservative), so no active brake is required (validation caster_layout.stopping); reversing faster than {CLs["max_coast_safe_reverse_speed_mps"]} m/s would not be covered. '
    '29 mm assembled height. CAD housing is a conservative envelope; three M3 × 5 pan-head machine screws (H13) on a 16.5 mm bolt circle, outside the Ø25.4 ball, through the 2 mm flange tabs into M3 brass heat-set inserts (H12) pressed into P_base from below. Drill the flange tabs to this pattern if the purchased caster differs.')
BOMF['parts']=[q for q in BOMF['parts'] if q['id']!='M03'];BP.pop('M03',None)   # ballast removed (user decision)
BP['S04']['notes']=f'USD 19.95 each checked 2026-10-03. Downward-facing sensors, optical face about {fx(CZ)} mm above the floor: front {FCL_TXT}, rear {RCL_TXT} beside the battery tray. VIN 3.3 V; XSHUT driven LOW or high impedance only. Reassign I2C addresses at boot. Test on actual floors; loss of reading requests stop.'
SWC=bumper['switch']['slot_width_cases'];opr=[c['operates_at_travel_range_mm'] for c in SWC]
BP['S03']['notes']=('Front left/right contact bumpers. Use COM + NC so an open wire also requests a stop. SS-5GL2 is the HINGE ROLLER lever (Ø4.8 × 3.2 roller on a 14.5 mm lever; Omron SS datasheet: OF max 0.49 N, FP max 19.3 mm, OP 14.5 ± 0.8 mm, OT min 1.2 mm, MD max 0.8 mm, heights of the roller top above the mounting-hole line). '
    'Each switch hangs under P_base on two M2 × 14 screws with washers and nuts on the base top, through 2.4 mm slots that let it slide ±0.9 mm along its own Y axis, turned 30° so the roller runs along the half\'s travel. '
    'The rail cam holds the roller above the release position at rest; the roller meets the 40° ramp from about 0 mm of travel and a dwell holds it past OP to the 5 mm stop. Because the OP tolerance (±0.8 mm) exceeds OT min (1.2 mm), the switch is set at assembly (from above, in the front bay): '
    'hold the half 2.25 mm in (gauge block at the rest stop; the roller force holds the bearing on its slot side), slide the switch toward the cam until it just operates, tighten. '
    f'During a hit the bearing can cross its slot (up to 2 × {SPLAY:.1f} mm), which makes the switch operate earlier, never later: operation stays within {min(o[0] for o in opr)}–{max(o[1] for o in opr)} mm for slot widths 8.1–{SLOTW} mm (validation.json bumper_travel.switch.slot_width_cases). '
    'Confirm the allowed dog direction with Omron\'s precautions (the cam meets the roller from the lever-tip side).')
BP['H08']['notes']=('Per robot (two 90° arc bumper halves, each floating 5 mm along its own axis 30° inward of rearward): six M3 × 35 countersunk screws (DIN 7991) fixed in the printed rails and arms from below; '
    'four 693ZZ ball bearings (3 × 8 × 4 mm) for the rail pins, each between a Ø3.2 × Ø4.5 × 1.5 mm and a Ø3.2 × Ø4.5 × 1 mm stainless spacer (no wider than the inner-ring land) under an M3 thin clamp nut (DIN 439, medium threadlocker); '
    'two Ø5 × 6.5 mm sleeves (M3 bore) under a thin clamp nut for pin C; six printed floating riders, each with a brass stop tube (Ø4 × 0.35 wall × 5 mm), a compression spring (music wire, Ø5 OD × 0.5 wire, free length 7.1 mm, about 2.0 N/mm), an Ø7 washer and an M3 nyloc nut: '
    f'tighten the nyloc until a 0.1 mm feeler just fits between the washer and the stop tube ({PRE_P} N preload per pin, zero vertical play at rest). Eighteen Ø4 mm chrome-steel balls (G10, plus spares): three per half in the rail/arm-top pockets and six per half under the riders. '
    f'Every ball runs between 0.5 mm hardened spring-steel races (blue-tempered 1095 or AISI 420, ≥ HRC 48, about 0.35 m of 12.7 mm strip, epoxy): eighteen {RACE_W} × {RACE_L:.0f} mm strips bonded to P_base (under each down-ball track and on the base top beside each slot) and eighteen {PRACE[0]:.0f} × {PRACE[1]:.0f} mm strips in the {BPL} mm pocket floors and rider pocket ceilings. '
    f'Four M2 × 14 screws, washers and nuts for the two switches. Bearing slots in P_base are printed {SLOTW} mm wide and filed to {SLOTW-.1:.1f}–{SLOTW} mm with P_coupon as the gauge (±0.05–0.1 mm side play: the planar-play checks use 0.1). '
    f'Critical vertical friction ≥ {bumper["friction_lock"]["min_mu_vert_crit"]} over the whole 0–90° arc at 20–{ZB1:g} mm, and ≥ {bumper["friction_lock"]["min_mu_crit_bearing_and_ball_equal"]} with bearing and ball friction equal (validation.json bumper_travel.friction_lock, .hertz).')
BP['H10']['notes']=('Thread-forming M3 for PETG into Ø2.6 pilots with at least 5 mm engagement: eight M3 × 10 pan-head screws from below for the four rear-shell foot ribs (heads in Ø6.2 counterbores over a 0.3 mm sacrificial bridge, into Ø8 × 5 mm bosses: 6.5 mm engagement) '
    '(the deck pillars and their four screws went with the deck in the low-profile layout). All other base screws are machine screws (H13) into brass inserts (H12) (validation.json blind_pilots).')
BP['H11']['item']=f'Ø3 × {DOWEL_L:.0f} mm steel dowel pins (ISO 8734)'
BP['H11']['notes']=(f'Radial dowels through the rear-shell lap joint at 180° (az 183.5°, z {DOWELS[0][1]:g} and {DOWELS[1][1]:g} mm): Ø3.1 press fit over {rsr["dowel"]["press_length_mm"]} mm in the left lap tongue and its 2 mm inner pad, Ø3.3 slip fit over {rsr["dowel"]["slip_length_mm"]} mm in the right segment wall; '
    'pressed in from outside to 0.1 mm under the shell surface (validation.json rear_shell.dowels; lap radial clearance 0.4 mm); teardrop holes print without support. Ream to size if the print is tight.')
BP['H12']['item']='M3 brass heat-set inserts, 3 mm long (for Ø4.0 mm holes)'
BP['H12']['notes']=('Pressed into Ø4.0 through holes in the 4 mm P_base with a soldering iron, flush with the face the screw enters, with the plate on a flat steel backing plate: the last 1 mm of each hole stays open and takes the displaced PETG (deburr or countersink the far face afterwards). '
    'From the top for the four bay-board spacers, four battery-tray corners and the eight lid-post foot screws; from below for the three caster screws; plus one pressed flush into the top of each of the four lid posts (lid screws) '
    'and three pressed flush into the LiDAR tray\'s plate top under the LD14 lugs (task L; the tray\'s 3.8 mm plate is too thin for thread-forming screws). Buy a pack of 50 (spares for the coupon). Follow the insert maker\'s hole size if it differs from Ø4.0.')
BP['H13']['notes']=('Into the H12 inserts (3 mm of thread in each, validation.json blind_pilots): four M3 × 8 pan head (bay board, through the board and a 2.4 mm printed spacer), four M3 × 6 countersunk (battery tray), three M3 × 5 pan head (caster), eight M3 × 6 pan head (lid-post feet), four M3 × 6 pan head (lid, into the inserts in the post tops). '
    'Plus four M3 × 10 socket head screws for the camera-mount feet (nuts H03; the old camera tower holes).')
BP['F01']['notes']=(f'Structural base, round bumper arcs, rear shell, riders, mounts, ventilated cover and the bay board\'s spacers and bearing ribs (no deck: low-profile layout) (about {com["printed_mass_kg"]} kg as modelled: solid walls ≤ 3 mm, 62 % from 3.6 mm (4 perimeters), linear between), plus the test coupon (print-parts/coupons/P_coupon.stl, about 12 g) and one failed arc (120–150 g each). '
    'User has Prusa MK3S (250 × 210 × 210 mm); every part and the coupon pass cad/check_printability.py. P_base (244 × 200 mm): no brim, skirt 1 mm or off. Arcs and shells: 4 perimeters, about 245 °C, part cooling ≤ 30 % up to the rim, then a 50–60 % fan height-range modifier for the inner top rims; brim on the arc ends.')
FOAM_L=round(2*R_SKIN*math.radians(AZF-3)/1000,2)
BP['F02']['notes']=(f'1.5 mm EVA foam tape (measure it: 1.5 ± 0.2 mm), {ZFT-26:g} mm wide (z 26–{ZFT:g}), on the outside of both bumper arcs from the seam-end ramps to az 70° (2 × {FOAM_L/2:.2f} m plus spare): it makes the {R_SKIN} mm contact skin. '
    f'Bevel-cut each foam end along the 20° seam-end ramp; keep the {2*G:g} mm seam free of foam. The outer 20° of each half (az 70–90, including the 78–90° taper to R{R_TIP}) is bare PETG covered with PTFE film tape (F03), so side and spin contacts slide instead of gripping. '
    f'Not on the fixed rear shell, which sits {R_SKIN-R_RS:.1f} mm inside the skin.')
BP['P08']['notes']=('Select a documented DC-rated snap-in rocker whose datasheet panel thickness range includes 1.5 mm; the CAD envelope is a 12.6 × 19.8 × 25 mm body (z '+f'{ZLIDT-25:g}–{ZLIDT:g}'+') through a 13.2 × 20.4 mm printed opening in the lid top at (-100, 0), '
    'with a 15.7 × 24.4 × 1.5 mm relief pocket under the lid so the panel is 1.5 mm at the opening (check the fit on P_coupon first; file to the datasheet panel cut-out so the snap tabs hold), plus a 24 × 21 × 3 mm bezel on the lid. The old rear panel is now inside the rear shell. Switch off after the Jetson has shut down.')
BP['E09']['notes']=('Use existing cooler if present. Fan header/plug depends on board revision. '
    f'CAD: 40 × 40 × 10 mm fan (Ø{2*FB[0]:g} bore) on the heatsink {JS["heatsink"]["fan_pitch"][0]:g} mm screw pattern with four M3 pan-head screws (heads ≤ Ø{2*FS_HEAD[0]:g} × {FS_HEAD[1]:g} mm). '
    f'The LiDAR tray seals its intake (task L; until then a skirt under the lid): its {SK_T} mm duct round the Ø{2*FW[2]:g} fan window and the LD14 motor ends {SK_GAP:g} mm above the fan frame, {jetson_cooling["fan_skirt"]["clearance_to_fan_screw_heads_mm"]} mm from the screw heads '
    f'(check the gap at assembly with modelling clay, BRINGUP section 10: {SK_FEELER[0]}–{SK_FEELER[1]} mm; a fan with a bore over Ø35 or larger screw heads needs the duct re-checked). '+jetson_cooling['bring_up_test'])
_jh=', '.join(f'({x:g}, {y:g})' for x,y in JHOLES)
BP['E01']['notes']=(f'Keep the existing carrier, module and heatsink. CAD = NVIDIA\'s official Jetson Nano Developer Kit P3450 B01 STEP (reference-models/jetson-nano-b01, simplified by cad/simplify_jetson.py): carrier 100 × 79 mm, heatsink top {JS["heatsink"]["top_z"]:.1f} mm above the PCB, I/O row (barrel jack, HDMI + DP, 2 × stacked USB-A, RJ45, micro-USB) '
    f'on the 100 mm edge facing forward (+x), barrel jack on the right; carrier PCB underside z {JO[2]:g} over the motors. Four Ø{JS["mounting_holes"]["d"]} mounting holes on 86 × 58 mm (hole pattern from the STEP) at {_jh} on H05 standoffs: two 24 mm stacks from P_base (front) and two 20 mm F-F standoffs on the bay board, which they clamp (rear). '
    'Check that the owned board is a B01 (two CSI connectors); an A02 has one CSI connector and other port positions. Barrel-jack power needs the J48 jumper fitted; '
    'the 5 V barrel lead assumes the 4 GB kit (a 2 GB kit needs its specified USB-C input). An Orin requires a revised power design. '+ASSY_ORDER)
BP['H05']['item']=f'M2.5 screws + nylon standoffs, front: two {JSTAND:g} mm stacks (12 mm M-F + 12 mm F-F) + four M2.5 × 6 pan-head carrier screws'
BP['H05']['notes']=(f'Front: two {JSTAND:g} mm M2.5 nylon standoff stacks from P_base to the Jetson carrier, each a 12 mm M-F (female end down, male stud up) under a 12 mm F-F, so both ends are female, '
    'on M2.5 × 8 countersunk screws from under P_base (flush in 90 deg countersinks). (A one-piece stock 25 mm F-F would raise the carrier to z 58; the CAD keeps z 57.) '
    'Four M2.5 × 6 pan-head screws hold the carrier on all four standoffs from the top (the two rear ones are H14). '
    f'Jetson B01 mounting holes Ø{JS["mounting_holes"]["d"]} on 86 × 58 mm (hole pattern taken from NVIDIA\'s STEP) at {_jh}. '
    f'Carrier underside z {JO[2]:g}: 4.5 mm over the motor tops (through-hole leads reach {-min(b[2] for b in JS["bboxes"]):.1f} mm under the PCB); Jetson isolated from P_base. '+ASSY_ORDER)
BP['H14']['item']=f'Rear Jetson clamp stack (bay board): 2 × M2.5 × {JSTAND_R:g} mm nylon F-F standoffs + 2 × M2.5 × 12 countersunk screws'
BP['H14']['notes']=(f'The two rear Jetson standoffs stand on the bay board (z {ZBAY_TOP:g}–{JO[2]:g}) and clamp it: each a stock {JSTAND_R:g} mm M2.5 nylon F-F standoff, screwed down with an M2.5 × 12 countersunk screw from under P_base '
    f'through the base, a 2.4 mm printed clamp spacer (bay_clamp_spacer_*) and the board\'s Ø{BC["clamp_holes"][0]["d"]:g} clamp hole (H5 / H6, ruling PB3), 4 mm into the standoff (assembly step 6). '
    'The carrier\'s top screws on them are in H05.')
BP['P03']['notes']=(f'March 2021 purchase. Measured envelope {P03S[1]} × {P03S[0]} × {P03S[2]} mm (CAD p03_regulator): on 1 mm double-sided foam tape on P_base in the front bay, turned 90 deg (long side along y, x {P03C[0]-P03S[0]/2:g}..{P03C[0]+P03S[0]/2:g}, |y| <= {P03S[1]/2:g}), '
    f'under the front of the Jetson carrier and ahead of the bay board\'s tongue, held by a cable tie through the P_base slots at ({P03_TIES[1][0]:g}, ±{P03_TIES[1][1]:g}). Board: HW-316E V4.0 (XL4015, 3-digit LED voltmeter); IN terminal on the left short end (+y), OUT on the right (−y): '
    f'leads of about {P03_LEADS["W_p03_in"]["route_mm"]:.0f} mm (IN) and {P03_LEADS["W_p03_out"]["route_mm"]:.0f} mm (OUT) over its top to U5A/U5B on the tongue\'s front edge. '
    'Adjust and verify regulated 5 V on the bench before it is fitted: its trimpot and tie are not reachable once the Jetson is on. Test at 6.0 V input with a 4.5 A load (the firmware low-battery stop, at the full Jetson current; BRINGUP.md section 5); rating alone is not acceptance. If it fails, the low-battery stop rises to the lowest passing input + 0.5 V. '+ASSY_ORDER)
# B9: the motor pigtails (C18), the board's connector table, the harness connection rows, the cut list in the BOM file and the spike check.
_jt=f'{MJ["x"]-MJ["t"]/2:g}..{MJ["x"]+MJ["t"]/2:g}';_jy=f'{MJ["y0"]:g}..{MJ["y0"]+MJ["male_l"]+MJ["female_l"]:g}'
_ex_motor=min(PIGTAIL[s_]['exit_to_motor_mm'] for s_ in PIGTAIL)
_c18=dict(id='C18',category='Connections',item='Motor pigtails (2): Sullins PPPC061LGBN-RC 1×6 right-angle female socket (board end, wires up) + Pololu 1905 1×6 crimp housing with male pins (joint end) + six 28 AWG silicone wires',
    quantity=2,unit='pigtails (made up)',status='Buy',new_cost_usd=17.0,model='signal_harness',
    source='https://www.digikey.com/en/products/detail/sullins-connector-solutions/PPPC061LGBN-RC/775939',source_label='Sullins PPPC061LGBN-RC at Digi-Key',   # LIO-173's link, kept here so a build does not revert it
    notes=(f'One per motor (ruling PB25 amended; B9). Board end: {RA_SOCKET["part"]}, pushed onto J4 / J5\'s right-angle pins with its six solder tails pointing upward; '
           f'each wire is lapped {RA_SOCKET["solder_overlap"]:g} mm on a tail tip and soldered under a Ø{RA_SOCKET["sleeve_d"]} mm heat-shrink sleeve, so the wires leave the board upward (socket {RA_SOCKET["h"]} mm tall on the board, tails and sleeves to {SOCKET["J4"]["top_z"]-ZBAY_TOP:.2f} mm over it, '
           f'inside the {UNDER_CARRIER_Z-1.5-ZBAY_TOP:.2f} mm under-carrier zone; the socket and the wires\' exit stay {_ex_motor:.1f} mm off the motor face) and turn over at >= {min(min(PIGTAIL[s_]["fillet_radii"]) for s_ in PIGTAIL)-(PG["r_run"]-PG["r_wire"]):.1f} mm radius for the innermost wire '
           f'(>= 3 × the {PG["od"]:g} mm insulation OD). Joint end: a Pololu 1905 1×6 0.1 in crimp housing with six male crimp pins (or a 1×6 straight male header with the wires soldered on if no DuPont crimper). '
           f'Wire: {PG["wire"]}, in the factory cable\'s colour order. Pre-cut length = the drawn route + {PG["term_mm"]:g} mm of terminations ({PG["term"]["solder_overlap_on_tail"]:g} lapped on the tail, {PG["term"]["inside_sleeve_past_tail"]:g} in the sleeve past the tip, '
           f'{PG["term"]["inside_male_housing"]:g} in the crimp housing) + the full {PG["pin_spread_mm"]:g} mm pin spread (the six wires fan out {PG["pin_spread_mm"]/2:g} mm either side of the drawn Ø3 bundle axis at the socket\'s sleeves and again at the male housing\'s cavities), rounded up to the mm: '
           f'left (J4) {PIGTAIL["left"]["precut_mm"]} mm, right (J5) {PIGTAIL["right"]["precut_mm"]} mm per wire. Make each pigtail like this: pre-cut the wires, solder the socket end first, dress the bundle along its route (assembly step 4), then trim each wire to its finished length at its own cavity of the male housing '
           f'(about {PIGTAIL["left"]["finished_centre_mm"]:g} mm on the J4 bundle axis, {PIGTAIL["right"]["finished_centre_mm"]:g} mm on J5\'s, each wire within the pin spread of that) and crimp the male pin; no slack (the joint is unplugged before the board is lifted, PB25 amended); '
           'cut from bulk wire and crimped (not a stock lead). Tag the two pigtails "L / J4" and "R / J5" right after making them (they differ in length and pin-1 order). '
           f'Built alike: looking into the socket\'s mating face with its tails up, the red wire is on the left (pin 1 on J4\'s {PIN1["J4"][0]} end, J5\'s {PIN1["J5"][0]} end). '
           'Polarity: the inline joint is not keyed, so pin 1 is marked on both halves: a paint-pen stripe on the pigtail\'s male housing at the red wire and on the factory female housing at its red wire; mate stripe to stripe and check the colours run straight across. '
           'Each joint is tied down on P_base and the factory cable\'s spare length is coiled and tied (C12 ties; assembly step 4). They stay with the board. '
           'About USD 17 for two (2 sockets about 2.2, the 6-colour 28 AWG set about 12, 2 housings + 12 pins about 3): estimate, verify.'))
BP['C18'].update(_c18)
def set_row(rid,cells):
    for row in BOMF['connectors']:
        if row[0]==rid:row[:]=[rid]+cells;return
    BOMF['connectors'].append([rid]+cells)
def pins_txt(ref,fn,word='cavity'):pm=pin_nets(ref);return ', '.join(f'{word} {k} = {pm[k]} ({fn[k][0]})' for k in sorted(pm,key=int))
set_row('J01',['Battery (lead W01)','Tamiya on the pack → adapter lead W01 → AMASS XT30U-F (C19) on board J1 (AMASS XT30UPB-M vertical PCB male, rear edge)',
    'J1: cavity 1 = GND (the XT30U-F\'s "−" contact: battery −, black), cavity 2 = BAT+ (its "+" contact: battery +, red) (ruling PB20; silk "-" / "+")','18 AWG',
    'Ruling PB30: J1 (XT30), J2 (JST VH) and J3 (Micro-Fit 3.0) are three connector families, so the battery lead fits no other header; tag it "J1 BATT". '
    'J1 pin 1 is GND and pin 2 is BAT+: the opposite of the usual "pin 1 = +" habit. Fail-safe check before the fuses go in: with F1 and F2 out, plug W01 into J1 and the battery and measure F1\'s input clip against a GND pad: positive, about the pack voltage (≥ 6 V; a part-discharged pack reads lower) '
    '(a reversed lead reads negative and has no current path with the fuses out). A reversed lead with the fuses in is the costliest mistake on this board (no reverse-polarity protection).'])
set_row('J02',['Regulator input (P03 leads)','Board U5A screw terminal (Phoenix MKDS 1,5/2-5,08) ← P03 input terminals',f'U5A: {pins_txt("U5A",{"1":("to P03 IN+",""),"2":("to P03 IN−","")},"pin")}','18 AWG',
    f'About {P03_LEADS["W_p03_in"]["route_mm"]:.0f} mm leads from the P03\'s IN terminal (its left short end, +y); a reversed P03 input is as costly as a reversed battery lead. Set and check 5.1 V on the bench first.'])
set_row('J03',['Jetson regulated power (lead W04)','Molex Micro-Fit 3.0 43645-0200 receptacle + 2 × 43030 crimps (C20) on board J3 (Molex 43650-0200 right-angle header in the band behind the Jetson carrier) → 5.5 / 2.1 mm barrel plug (provisional)',
    f'J3: {pins_txt("J3",{"1":("barrel centre +",""),"2":("barrel sleeve","")})} (silk "+" at pin 1)','18 AWG',
    'Ruling PB30: a Micro-Fit 3.0, so the Jetson lead fits neither the battery header (XT30) nor the rocker header (JST VH); tag it "J3 JETSON 5V". Micro-Fit 3.0 on 18 AWG, 2 circuits: 8.5 A per circuit (Molex derating table, from memory: verify) for the Jetson\'s 4.5 A. '
    'The receptacle plugs in from beyond the board\'s right edge (robot −y) and the lead runs along the W04 corridor to the barrel jack (bay ruling PB5). Replace the output end for the actual Jetson revision; barrel power needs the J48 jumper.'])
set_row('J04',['Regulator output (P03 leads)','P03 output terminals → board U5B screw terminal (Phoenix MKDS 1,5/2-5,08)',f'U5B: {pins_txt("U5B",{"1":("from P03 OUT+",""),"2":("from P03 OUT−","")},"pin")}','18 AWG',
    f'About {P03_LEADS["W_p03_out"]["route_mm"]:.0f} mm leads from the P03\'s OUT terminal (its right short end, −y) over its top. (The deck-era VH lead to the driver terminals is gone: motor power runs in the board copper.)'])
for rid,side,ref in (('J05','left','J4'),('J06','right','J5')):
    set_row(rid,[f'{side.capitalize()} motor cable','Factory female 1×6 → C18 pigtail male 1×6 at the inline joint; pigtail right-angle female socket → board '+ref+' (right-angle 1×6 male header)',
        f'{ref}: '+', '.join(f'pin {k} = {v} ({LEAD_FN[f"motor_pigtail_{side}"][3][k][1]}, {LEAD_FN[f"motor_pigtail_{side}"][3][k][0].split(" (")[0]})' for k,v in sorted(pin_nets(ref).items(),key=lambda q:int(q[0]))),'Factory cable + 28 AWG pigtail',
        f'PB25 / B9: the C18 pigtail (pre-cut length {PIGTAIL[side]["precut_mm"]} mm per wire = route + terminations + the full {PG["pin_spread_mm"]:g} mm pin spread; finished length about {PIGTAIL[side]["finished_centre_mm"]:g} mm on the bundle axis, {PIGTAIL[side]["finished_range_mm"][0]:g}-{PIGTAIL[side]["finished_range_mm"][1]:g} mm once each wire is trimmed at its cavity after dressing; tag it "{"L / J4" if side=="left" else "R / J5"}") stays plugged on {ref} (fitted on the bench); the motor cable mates with it at the inline joint ahead of the {side} motor (x {_jt}, |y| {_jy}), reachable with the Jetson off, '
        f'tied down on P_base; the factory cable is measured from the encoder cap to the female housing\'s rear face (readings of {FAC_READINGS[0]:g}-{FAC_READINGS[1]:g} mm fit), the fixed path routed first (about {FAC[side]["fixed"]:.0f} mm) and the remainder coiled ({FAC_TURNS[0]:g}-{FAC_TURNS[1]:g} turns) and tied ahead of the motor. The joint is not keyed: pin 1 (the red wire) is marked with a paint-pen stripe on both halves; mate stripe to stripe, colours straight across. '
        'Before the first power-up check the factory housing\'s cavity order is red, black, green, blue, yellow, white from pin 1 (board_spec MOTOR); the board\'s pinout assumes it.'])
set_row('J21',['Battery-voltage sensing','On the bay board (no harness)','VBAT_SW → R11 100k → GP26 (VSENSE) node → R12 33k → GND; C2 100 nF','—','At an 8.7 V pack the ADC reads about 2.16 V.'])
set_row('J22',['Master switch (lead W02)','JST VHR-2N on board J2 (B2P-VH, rear edge) → rocker terminals (insulated spades)',f'J2: {pins_txt("J2",{"1":("to the rocker",""),"2":("from the rocker","")})}','18 AWG',
    'Rocker on the lid top at (-100, 0): F1 output → rocker → switched battery bus. Both wires are battery +; shut the Jetson down before opening the switch. '
    'J2 is the only JST VH header on the board (J1 is an XT30, J3 a Micro-Fit: PB30), so the rocker pair fits no other header; tag it "J2 ROCKER".'])
set_row('J23',['Battery positive distribution','On the bay board (VBAT_SW copper; no WAGO bus)','J2.2 → F2 / U5A / R11 / R21','—','Replaced by the board.'])
set_row('J24',['Ground distribution','On the bay board (In1 GND plane; no WAGO bus)','Every GND pin of the board','—','Replaced by the board: the regulator, driver and signal grounds share the plane.'])
MATES={'J1':'AMASS XT30U-F (C19); lead W01','J2':'JST VHR-2N + 2 × SVH-21T-P1.1 (C05); lead W02','J3':'Molex 43645-0200 + 2 × 43030-series 18 AWG crimps (C20); lead W04',
    'J4':'C18 pigtail: right-angle 1×6 female socket, wires up','J5':'C18 pigtail: right-angle 1×6 female socket, wires up',
    **{f'J{6+i}':f'JST XHP-5 + 5 × SXH-001T-P0.6 (C06); lead W{10+i}' for i in range(4)},'J10':'JST XHP-2 + 2 × SXH-001T-P0.6 (C08); lead W_bumper_left','J11':'JST XHP-2 + 2 × SXH-001T-P0.6 (C08); lead W_bumper_right',
    'J12':'JST XHP-2 + 2 × SXH-001T-P0.6 (C08); lead W15','J13':'JST XHP-2 + 2 × SXH-001T-P0.6 (C08); lead W14','U5A':'P03 input leads (bare ends in the screw terminal)','U5B':'P03 output leads (bare ends in the screw terminal)'}
BOMF['bay_board_connectors']=[dict(ref=r,function=BS.PARTS[r]['value'],board_part=BS.PARTS[r]['mpn'],family=BCONN[r]['family'],entry=BS.EDGE_CONNECTORS[r],under_carrier=BCONN[r]['under_carrier'],
    pins=pin_nets(r),mates_with=MATES[r],lead=next((c['id'] for c in cables if (c.get('board_end') or {}).get('ref')==r),None)) for r in BS.EDGE_CONNECTORS]
# The spike (W4L report section 8) cable table: 'now' (the strip-era routes) and the (B) bay estimate, against the final routes. Net change = sum(final - now).
SPIKE=[('W01','Battery adapter W01 → J1',50,95),('W02','Rocker W02 → J2',72,118),('W14','Toggle loop W14 → J13',54,93),('W15','Arm button W15 → J12',50,144),('W10','Cliff 1 W10 → J6',61,116),
       ('W11','Cliff 2 W11 → J7',59,154),('W12','Cliff 3 W12 → J8',142,174),('W13','Cliff 4 W13 → J9',142,159),('W_bumper_left','Bumper L → J10',145,135),('W_bumper_right','Bumper R → J11',145,135),
       ('W04','Jetson 5 V W04 → J3',138,138),('W16','Pico USB W16',234,209),('W_p03_in','P03 lead U5A',115,10),('W_p03_out','P03 lead U5B',65,11),
       (('motor_pigtail_left','motor_pigtail_right'),'Motor harness L + R (strip era; now the PB25 pigtails)',50,0)]
_rt={c['id']:c['route_mm'] for c in cables}
sp_rows=[dict(lead=n,ids=list(i) if isinstance(i,tuple) else [i],now_mm=a_,spike_b_mm=b_,final_mm=sum(_rt[k] for k in (i if isinstance(i,tuple) else (i,)))) for i,n,a_,b_ in SPIKE]
BOMF['harness']=dict(note='cable-cut-list.json, written by cad/build_robot.py: route = the CAD route; cut = route × 1.2 + 40 mm rounded up to 50 mm, except the pigtails (pre-cut long by the pin spread, then each wire trimmed at its cavity after dressing and crimped), the factory motor cables (200 mm, not cut).',
    cut_list=[dict(id=c['id'],name=c['name'],board_end=(c.get('board_end') or {}).get('ref'),wire=c.get('wire'),route_mm=c['route_mm'],cut_mm=c['suggested_cut_mm'],
                   **({'cut_means':'pre-cut length (trim each wire at its cavity after dressing)','precut_mm':c['pigtail']['precut_mm'],'finished_centre_mm':c['pigtail']['finished_centre_mm'],'finished_range_mm':c['pigtail']['finished_range_mm'],'pin_spread_mm':c['pigtail']['pin_spread_mm']} if 'pigtail' in c else {}),
                   pinout=[f'{q["cavity"]} {q["net"]} ({q["colour"]})' for q in c.get('pinout',[])]) for c in cables],
    spike_check=dict(source='.superpowers/sdd/2026-10-06-r1-low-profile/task-W4L-report.md section 8 (rough; mm)',spike_net_change_mm=169,rows=sp_rows,
                     net_change_mm=sum(r['final_mm']-r['now_mm'] for r in sp_rows),spike_b_net_change_mm=sum(r['spike_b_mm']-r['now_mm'] for r in sp_rows),
                     note='the factory motor cables (200 mm), the USB / fan / LiDAR leads and the new factory-cable coils are outside the spike table, as there'))
BP['F01']['notes']+=(' Bay board printed parts: four 2.4 mm spacers (bay_spacer_0–3) and two clamp spacers (bay_clamp_spacer_1 / _3), 100 % infill; two bearing ribs (bay_rib_0 / _1, bonded with epoxy F04); '
    'the battery tray carries the three hold-down springs (bay_holddown, printed with P_battery_tray, beams freed with a razor saw); the LD14 lead grommet (P_ld14_grommet). '
    'Camera LED shelf (P_camera_led_shelf, task N): clamped on the camera board\'s front under its two upper screws, holds the LED carrier above the lens.')
BOMF['base_new_cost_usd']=round(sum(p_['new_cost_usd'] for p_ in BOMF['parts'] if p_['status']!='Conditional buy'),2)
(ROOT/'parts-and-connections.json').write_text(json.dumps(BOMF,indent=2,ensure_ascii=False))
bq={p['id']:p['quantity'] for p in BOMF['parts']}
def mcount(code,pref=None):return sum(1 for p in parts if code in p['bom'].split('/') and (pref is None or p['id'].startswith(pref)))
bom_rows=[dict(line='H10',item='M3 thread-forming screws (rear shell)',manifest=mcount('H10'),ordered=bq.get('H10')),
    dict(line='H12',item='M3 × 3 brass heat-set inserts',manifest=mcount('H12'),ordered=bq.get('H12')),
    dict(line='H13',item='M3 machine screws into the inserts + camera-mount M3 × 10',manifest=mcount('H13'),ordered=bq.get('H13')),
    dict(line='H11',item=f'Ø3 × {DOWEL_L:.0f} dowels',manifest=mcount('H11'),ordered=bq.get('H11')),
    dict(line='M02',item='rear ball caster',manifest=sum(1 for p in parts if p['id']=='caster_ball'),ordered=bq.get('M02')),
    dict(line='H14',item='rear clamp stacks (standoff + countersunk screw)',manifest=mcount('H14','jetson_standoff'),ordered=bq.get('H14')),
    dict(line='E16',item='DRV8874 carriers',manifest=mcount('E16'),ordered=bq.get('E16')),
    dict(line='C18',item='motor pigtails',manifest=mcount('C18','motor_pigtail'),ordered=bq.get('C18'))]
txt=dict(no_ballast_line='M03' not in BP and not any('ballast' in json.dumps(q,ensure_ascii=False).lower() for q in BOMF['parts']),reverse_limit_in_M02=f'{vrev} m/s' in BP['M02']['notes'],rear_cliff_in_M02=RCL_TXT in BP['M02']['notes'],rear_cliff_in_S04=RCL_TXT in BP['S04']['notes'],
    caster_x_in_M02=f'x = {fx(CX)} mm' in BP['M02']['notes'],dowel_size=f'Ø3 × {DOWEL_L:.0f}' in BP['H11']['item'],slot_width_in_H08=f'{SLOTW} mm' in BP['H08']['notes'],
    dropped_parts_gone=not any(w_ in json.dumps(BOMF['parts'],ensure_ascii=False) for w_ in ('D24V50F5','W_motor','MDD3A')) and 'optional_regulator_usd' not in BOMF,
    every_cad_code_is_a_line=all(code in BP for p in parts for code in filter(None,p['bom'].split('/'))),
    every_edge_connector_listed=sorted(r['ref'] for r in BOMF['bay_board_connectors'])==sorted(BS.EDGE_CONNECTORS) and all(r['board_part']==BS.PARTS[r['ref']]['mpn'] for r in BOMF['bay_board_connectors']))
bom_check=dict(rows=bom_rows,text_from_cad=txt,ok=all(r['manifest']==r['ordered'] for r in bom_rows) and all(txt.values()),note='The M02, S03, S04, H08, H10–H13, F01, F02 and P08 notes are written by build_robot.py from the CAD constants and results; text_from_cad confirms the sizes, mass and coordinates.')
# Robot top (L2 gate): the highest point of any part <= 160 mm; the LiDAR scan plane and the camera follow the lid down by DZ.
_top=max(parts,key=lambda p:p['bounds'][5])
robot_top=dict(height_mm=round(_top['bounds'][5],2),highest_part=_top['id'],limit_mm=160.0,drop_from_deck_era_lid_mm=DZ,lid_underside_z_mm=ZLID,lid_top_z_mm=ZLIDT,
    bumper_and_rear_shell_top_z_mm=ZB1,lidar_scan_plane_z_mm=PARAMS['lidar_height'],camera_lens_axis_z_mm=PARAMS['camera_height'],camera_mount_crossbar_z_mm=[CBZ,CBZ+6])
robot_top['ok']=robot_top['height_mm']<=robot_top['limit_mm']
# Bay board (B1): the turned P03 against the bumper at full travel, the Jetson height, the under-carrier height zone and the rear-row ribs.
# P03: every part that moves with a bumper half (ids bumper_* / P_bumper*, as pcb/extract_outline.py sweeps them) is swept through its 5 mm travel;
# the minimum gap over the sweep equals the distance from the part at rest to the P03 box swept the opposite way (Minkowski sum with the travel
# segment). It must stay >= 0.5 mm (the swept parts do not touch the P03 box inflated by 0.5 mm).
def hull2(pts):   # convex hull (monotone chain), counter-clockwise
    P=sorted(set(pts));h=[]
    for seq in (P,P[::-1]):
        k=len(h)
        for q in seq:
            while len(h)>=k+2 and (h[-1][0]-h[-2][0])*(q[1]-h[-2][1])-(h[-1][1]-h[-2][1])*(q[0]-h[-2][0])<=0:h.pop()
            h.append(q)
        h.pop()
    return h
_pb=shapes['p03_regulator'].BoundingBox();P03_GAP_MIN=0.5
p03_rows=[]
for i in [q['id'] for q in parts if q['id'].startswith(('bumper_','P_bumper'))]:
    sg=1 if 'left' in i else -1;assert ('left' in i)!=('right' in i),i
    d=(BT*ft[0],sg*BT*ft[1])   # full-travel shift of this half
    corners=[(x,y) for x in (_pb.xmin,_pb.xmax) for y in (_pb.ymin,_pb.ymax)]
    sweptbox=prism(hull2(corners+[(x-d[0],y-d[1]) for x,y in corners]),_pb.zmin,_pb.zmax-_pb.zmin)
    if near(shapes[i],sweptbox,20):p03_rows.append(dict(part=i,min_gap_over_travel_mm=round(sdist(shapes[i],sweptbox),2)))
p03_rows.sort(key=lambda r:r['min_gap_over_travel_mm'])
p03_gap=p03_rows[0]['min_gap_over_travel_mm'] if p03_rows else None
p03_clear=p03_gap is None or p03_gap>=P03_GAP_MIN
# Jetson: the carrier PCB (NVIDIA STEP 'carrier' solid) underside must stay at z 57.0 (the rear standoffs now stand on the board).
jetson_carrier_bottom_z=round(jpart('carrier').BoundingBox().zmin,3)
# Bay B8. Helpers: every non-wire part a probe solid overlaps (> 0.05 mm3), outside a skip set; the lid and everything on it (group 'cover') comes
# off for service, so plug paths from above may cross it.
def overlaps(q,skip=(),ids=None):
    out=[]
    for i in (SOLID if ids is None else ids):
        if i in skip or not near(q,shapes[i]):continue
        v=q.intersect(shapes[i]).Volume()
        if v>.05:out.append(dict(part=i,mm3=round(v,2)))
    return out
LIDSIDE={p['id'] for p in parts if p['group']=='cover'}
def boxgap(a,b):return math.hypot(max(a[0]-b[2],b[0]-a[2],0.),max(a[1]-b[3],b[1]-a[3],0.))   # plan gap of two (x0, y0, x1, y1) boxes
# Under-carrier height zone (the real board, modules and plugs): every solid of the bay geometry that reaches x >= CARRIER_X keeps its top <=
# UNDER_CARRIER_Z - 1.5 (solid bounding boxes, so a solid that straddles the zone edge counts whole).
UC_IDS=['bay_board','bay_pico','bay_drv_0','bay_drv_1','bay_imu','pico_usb_plug']+sorted(i for i in shapes if i.startswith('bay_plug_'))
def uc_top(i):
    t=[q.BoundingBox().zmax for q in shapes[i].Solids() if q.BoundingBox().xmax>=CARRIER_X]
    return round(max(t),3) if t else None
uc_rows=[dict(part=i,top_z_mm=uc_top(i)) for i in UC_IDS if uc_top(i) is not None]
bay_uc_max_top=max(r['top_z_mm'] for r in uc_rows) if uc_rows else None
bay_uc_ok=bay_uc_max_top is None or bay_uc_max_top<=UNDER_CARRIER_Z-1.5+1e-6
# Rear-row ribs (B1 key; review M3): each rear-edge connector has a rib touching the board underside under >= 50 % of its body's length along y,
# and the rib lies under the body itself (x overlap with the F.Fab body), not only under the courtyard.
rib_ids=[q['id'] for q in parts if q['id'].startswith('bay_rib_')];rib_rows={}
for ref in BAY_REAR_ROW:
    x0,y0,x1,y1=BCONN[ref]['body_xy']
    iv=sorted((max(y0,b[1]),min(y1,b[4])) for b in (byid[r]['bounds'] for r in rib_ids) if b[0]<x1 and x0<b[3] and abs(b[5]-ZBAY)<1e-3 and max(y0,b[1])<min(y1,b[4]))
    cov=0.;end=-1e9
    for a_,b_ in iv:
        a_=max(a_,end)
        if b_>a_:cov+=b_-a_;end=b_
    rib_rows[ref]=dict(body_x_mm=[x0,x1],body_y_mm=[y0,y1],rib_supported_length_mm=round(cov,2),fraction=round(cov/(y1-y0),3))
rib_pad=[dict(rib=r,gap_mm=round(g,3),nearest_pad=n,pockets_y_mm=RIB_POCKETS[k]) for k,r in enumerate(rib_ids) for g,n in [pad_gap((byid[r]['bounds'][0],byid[r]['bounds'][1],byid[r]['bounds'][3],byid[r]['bounds'][4]))]]
bay_ribs_ok=bool(rib_ids) and all(r['fraction']>=.5 for r in rib_rows.values()) and all(r['gap_mm']>=RIB_GAP-1e-6 for r in rib_pad)
# Review Focus 2 (bay_rear_connectors_supported): every rear-edge connector over a rib and its centre within 35 mm of a fixing: an M3 hole or clamp
# hole of the real board, or a rear-edge hold-down finger whose contact is real (review M1): the installed finger's underside on the board's top
# face (the STEP's seating plane, z ZBAY + 1.595: KiCad's laminate body stops 0.09 mm lower, without copper and mask) within 0.05 mm, a contact
# patch on the board (inside its outline, clear of every part), and no overlap with the board.
FIX_MAX=35.0;BOARD_TOP=round(ZBAY+1.595,3)
_lam=BSTEP['r1_power_io_PCB'];_outline=[tuple(q) for q in BAY_OUTLINE]
def _inside(pt,poly):
    c=False;x,y=pt
    for (x1,y1),(x2,y2) in zip(poly,poly[1:]+poly[:1]):
        if (y1>y)!=(y2>y) and x<(x2-x1)*(y-y1)/(y2-y1)+x1:c=not c
    return c
hd_rows=[]
for k,(bm,(yc,g,d)) in enumerate(zip(HD_BEAMS,HOLDDOWN)):
    fin=hd_installed(k).intersect(box((HD_XT+BAY_EDGE_X)/2,yc,HD_ZF-1,HD_XT-BAY_EDGE_X+.02,HD['w']+.4,4)).BoundingBox()   # the finger over the board, installed
    pa=(BAY_EDGE_X,yc-HD['w']/2,HD_XT-HD['lead'],yc+HD['w']/2)   # the flat contact patch (lead-in excluded)
    on_board=all(_inside(q,_outline) for q in ((pa[0]+.01,pa[1]),(pa[2],pa[1]),(pa[2],pa[3]),(pa[0]+.01,pa[3])))
    clear=[n for n,q in BSTEP.items() if n!='r1_power_io_PCB' and boxgap(pa,(q.BoundingBox().xmin,q.BoundingBox().ymin,q.BoundingBox().xmax,q.BoundingBox().ymax))<=0 and q.BoundingBox().zmax>ZBAY_TOP]
    gap_top=round(fin.zmin-BOARD_TOP,3);gap_lam=round(hd_installed(k).distance(_lam),3);area=round((pa[2]-pa[0])*(pa[3]-pa[1]),2) if on_board and not clear else 0.
    hd_rows.append(dict(box_xyz=[TRAY_FACE,round(yc-HD['w']/2,3),round(fin.zmin,3),HD_XT,round(yc+HD['w']/2,3),round(fin.zmax,3)],contact_xy=[round((pa[0]+pa[2])/2,3),round(yc,3)],
        gap_between_bodies_mm=g,beam=bm,finger_to_board_top_mm=gap_top,finger_to_laminate_body_mm=gap_lam,contact_patch_mm2=area,parts_under_patch=clear,
        ok=abs(gap_top)<=.05 and area>=1.0 and not clear))
hd_board=round(shapes['bay_holddown'].intersect(shapes['bay_board']).Volume(),3)
FIXINGS=[(f'M3 hole {h["ref"]}',tuple(h['xy'])) for h in BC['holes']]+[(f'clamp {h["ref"]} (rear Jetson standoff)',tuple(h['xy'])) for h in BC['clamp_holes']]+[(f'hold-down finger y {r["contact_xy"][1]:g}',tuple(r['contact_xy'])) for r in hd_rows if r['ok']]
def nearest_fixing(p):return min((round(math.dist(p,q),2),n) for n,q in FIXINGS)
rear_rows={}
for ref in REAR_REFS:
    d,n=nearest_fixing(BCONN[ref]['centre_xy'])
    rear_rows[ref]=dict(rib_fraction=rib_rows[ref]['fraction'],fixing_mm=d,nearest_fixing=n,without_holddown_mm=min(round(math.dist(BCONN[ref]['centre_xy'],q),2) for _,q in FIXINGS[:6]),
                        ok=rib_rows[ref]['fraction']>=.5 and d<=FIX_MAX)
bay_rear_ok=bay_ribs_ok and all(r['ok'] for r in rear_rows.values()) and all(r['ok'] for r in hd_rows) and hd_board<=.05 and HD_SPRING['ok']
# Carry item 2: every edge connector within 35 mm of a fixing, and a bearing under the board near the ones plugged with force (VH, the screw
# terminals, the rear row): a rib under >= 50 % of the body (rear row), else a printed spacer (bay or clamp, Ø7) within BEARING_MAX of it.
BEARING_MAX=10.0
SPACERS=[(f'bay_spacer_{i}',xy) for i,xy in enumerate(BAY_MOUNTS)]+[(f'clamp spacer {h["ref"]}',tuple(h['xy'])) for h in BC['clamp_holes']]
HIGH_FORCE={r for r,c in BCONN.items() if c['family'] in ('VH','XT30','MicroFit','Terminal')}|set(REAR_REFS)   # B12a: the power connectors
edge_rows={}
for ref,c in BCONN.items():
    d,n=nearest_fixing(c['centre_xy']);cx0,cy0,cx1,cy1=c['courtyard_xy']
    bg,bn=min((round(math.hypot(max(cx0-x,0,x-cx1),max(cy0-y,0,y-cy1))-3.5,2),sn) for sn,(x,y) in SPACERS)
    if ref in rib_rows:bear_ok,bear=rib_rows[ref]['fraction']>=.5,f'rib under {100*rib_rows[ref]["fraction"]:.0f} % of the body'
    else:bear_ok,bear=bg<=BEARING_MAX,f'{bn} {bg:g} mm from the courtyard'
    edge_rows[ref]=dict(fixing_mm=d,nearest_fixing=n,high_plug_force=ref in HIGH_FORCE,bearing=bear,bearing_ok=bear_ok,ok=d<=FIX_MAX and (bear_ok or ref not in HIGH_FORCE))
edge_ok=all(r['ok'] for r in edge_rows.values())
# Review Focus 5 (pico_usb_insert_path_clear): the micro-B plug, lowered from 30 mm above to its start point PICO_STROKE clear of the receptacle and
# pushed PICO_STROKE along -y into it, crosses nothing but the lid side (off for service); the slide may only meet the Pico it mates with.
PICO_STROKE=6.0
_pw,_pl,_ph=PICO_PLUG
usb_down=box(USB_FACE[0],USB_FACE[1]+PICO_STROKE+_pl/2,USB_FACE[2]-_ph/2,_pw,_pl,_ph+30)
usb_in=box(USB_FACE[0],USB_FACE[1]+(_pl+PICO_STROKE)/2,USB_FACE[2]-_ph/2,_pw,_pl+PICO_STROKE,_ph)
usb_hits=dict(lowered=overlaps(usb_down,LIDSIDE|{'pico_usb_plug'}),pushed_in=overlaps(usb_in,LIDSIDE|{'pico_usb_plug','bay_pico'}))
pico_usb_ok=not usb_hits['lowered'] and not usb_hits['pushed_in']
# Rear-band plugs (lid off): each plug's footprint, swept 30 mm up from its top, meets nothing but the lid side.
plug_paths={}
for ref in REAR_REFS+('J3',):
    x0,y0,x1,y1,z0,z1=PLUG_BOX[ref];plug_paths[ref]=overlaps(box((x0+x1)/2,(y0+y1)/2,z1,x1-x0,y1-y0,30),LIDSIDE|{f'bay_plug_{ref}'})
plug_paths_ok=not any(plug_paths.values())
def board_hits(q,own=()):   # the board's own parts a probe overlaps, component by component (review M2: a probe skips only its own connector's model)
    return [dict(part=f'bay_board:{n}',mm3=round(v,2)) for n,sh in list(BSTEP.items())+[(f'{r} envelope',e_) for r,e_ in zip(STEP_ENVELOPES,_env)]
            if n not in own and near(q,sh) for v in [q.intersect(sh).Volume()] if v>.05]
# The Jetson carrier's lead-tail allowance: a slab under the carrier PCB (its footprint, z UNDER_CARRIER_Z..57) in the clash scan; only the Jetson's
# own parts and its standoff stacks may be in it.
JOWN={'jetson','jetson_heatsink','usb_stack_0','usb_stack_1','ethernet','jetson_barrel_jack','fan'}|{i for i in shapes if i.startswith(('jetson_standoff','jetson_screw','fan_screw'))}
SLAB=box((JPCB[0]+JPCB[1])/2,(JPCB[2]+JPCB[3])/2,UNDER_CARRIER_Z,JPCB[1]-JPCB[0],JPCB[3]-JPCB[2],JO[2]-UNDER_CARRIER_Z)
slab_hits=overlaps(SLAB,JOWN)+overlaps(SLAB,ids=[c['id'] for c in cables if c.get('modelled',True)])
# Review Focus 1 (under_carrier_cable_paths_clear): a cable stub at every under-carrier connector, the connector's courtyard width, 15 mm in its entry
# direction. A top-entry plug's wires rise out of its top and turn through 90 deg within the bend allowance (the bend block over the plug body,
# plug top .. + CABLE_BEND), then run outward along the top of that allowance (the run block, CABLE_T thick). The J4 / J5 pigtails leave their
# lay-flat housings toward the motor and run in B3's motor-cable corridor (the band beside the motor's end face, from the plug's far side to the
# exit, board top to the exit height + T/2; a straight 15 mm stub would end in the motor 2 mm away); the screw terminals' wires enter their face
# over the body height. No stub may meet a non-wire part (nor the lead-tail slab) except the connector's own plug and model (review M2: the rest of
# the board counts, component by component) and the part the cable ends at.
CABLE_T,STUB_L=2.0,15.0
STUB_END={'J4':'motor_left','J5':'motor_right','U5A':'p03_regulator','U5B':'p03_regulator'}
def stubs(ref):
    c=BCONN[ref];ex,ey=c['entry_xy'];x0,y0,x1,y1=c['courtyard_xy']
    def face(q):return (q[2] if ex>0 else q[0]) if ex else (q[3] if ey>0 else q[1])
    def run(start,z0,z1):
        a,b=sorted((start,start+(ex or ey)*STUB_L))
        return box((a+b)/2,(y0+y1)/2,z0,b-a,y1-y0,z1-z0) if ex else box((x0+x1)/2,(a+b)/2,z0,x1-x0,b-a,z1-z0)
    if c['vertical']:
        bx0,by0,bx1,by1,_,zt=PLUG_BOX[ref];zb=zt+c['bend']
        return [('bend',box((bx0+bx1)/2,(by0+by1)/2,zt,bx1-bx0,by1-by0,zb-zt)),('run',run(face(c['body_xy']),zb-CABLE_T,zb))]
    if ref in BC['motor_cables']:
        m=BC['motor_cables'][ref];(ex_,_,ez_)=m['exit_xyz'];px0,_,px1=PLUG_BOX[ref][:3];by0,by1=m['band_y']
        a,b=min(px0,ex_-m['w']/2),max(px1,ex_+m['w']/2)
        return [('corridor',box((a+b)/2,(by0+by1)/2,ZBAY_TOP,b-a,by1-by0,ez_+m['t']/2-ZBAY_TOP))]
    return [('run',run(face(c['courtyard_xy']),ZBAY_TOP,ZBAY_TOP+c['mated_h']))]
stub_rows={}
for ref in sorted(r for r,c in BCONN.items() if c['under_carrier']):
    rows=[]
    for kind,q in stubs(ref):
        b=q.BoundingBox()
        rows.append(dict(block=kind,box_xyz=[round(v,2) for v in (b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax)],hits=overlaps(q,{'bay_board',f'bay_plug_{ref}',STUB_END.get(ref)})+board_hits(q,own=(ref,)),
                         slab_mm3=round(q.intersect(SLAB).Volume(),3),top_under_limit=b.zmax<=UNDER_CARRIER_Z-1.5+1e-6))
    stub_rows[ref]=dict(entry=BCONN[ref]['entry'],cable_ends_at=STUB_END.get(ref,'its harness lead'),blocks=rows,ok=all(not r['hits'] and r['slab_mm3']<=.05 and r['top_under_limit'] for r in rows))
stubs_ok=bool(stub_rows) and all(r['ok'] for r in stub_rows.values())
# Mating-axis sweeps (review, PB25): every plug can travel its engagement length + 1 mm along its pins, clear of everything but wires, its own
# connector and the lid side (and the Jetson, not fitted when they are plugged): vertical plugs up by the header's height (the plug clears the
# header shroud), lay-flat plugs along their pins by the 0.1 in post length (6 mm), the motor inline joints' female housings along their axis.
# J4 / J5 are exempt: their pigtails are plugged on the bench before the board is installed (PB25), and their sweep would end in the motor.
SWEEP_EXEMPT={'J4':'plugged on the bench before the board is installed (PB25)','J5':'plugged on the bench before the board is installed (PB25)'}
JETSIDE={p['id'] for p in parts if p['group']=='upper' and (p['id'].startswith(('jetson','usb_stack','ethernet','fan','J17','J18','J19','J20','J27','J03')) or p['id'] in ('lidar_adapter',))}
sweep_rows={}
for ref,(x0,y0,x1,y1,z0,z1) in sorted(PLUG_BOX.items()):
    c=BCONN[ref]
    if c['vertical']:ax,eng=(0,0,1),STEP_H[ref];q=box((x0+x1)/2,(y0+y1)/2,z0,x1-x0,y1-y0,z1-z0+eng+1)
    else:
        ax,eng=(*c['entry_xy'],0),6.0;L_=eng+1;ex,ey=c['entry_xy']
        q=box((x0+x1)/2+ex*L_/2,(y0+y1)/2+ey*L_/2,z0,x1-x0+abs(ex)*L_,y1-y0+abs(ey)*L_,z1-z0)
    h=overlaps(q,LIDSIDE|JETSIDE|{f'bay_plug_{ref}','bay_board'})+board_hits(q,own=(ref,))
    sweep_rows[ref]=dict(axis=list(ax),engagement_mm=eng,travel_mm=eng+1,hits=h,exempt=SWEEP_EXEMPT.get(ref),ok=not h or ref in SWEEP_EXEMPT)
for side,J_ in JOINT.items():
    sg=J_['axis'][1];L_=MJ['engage']+1;bq=J_['female'].BoundingBox()
    q=box((bq.xmin+bq.xmax)/2,(bq.ymin+bq.ymax)/2+sg*L_/2,bq.zmin,bq.xlen,bq.ylen+L_,bq.zlen)
    h=overlaps(q,{f'motor_joint_{side}',f'motor_tie_joint_{side}'})   # the joint's tie goes on after it is mated (and is cut to unplug it)
    sweep_rows[f'motor_joint_{side}']=dict(axis=list(J_['axis']),engagement_mm=MJ['engage'],travel_mm=L_,hits=h,exempt=None,ignored=f'motor_tie_joint_{side} (fitted after mating, cut to unplug)',ok=not h)
sweeps_ok=all(r['ok'] for r in sweep_rows.values()) and set(r for r,v in sweep_rows.items() if v['exempt'])=={'J4','J5'}
# PB25 amended / B9 (motor_joints_serviceable): (1) each inline joint's female housing slides MJ['unplug'] mm out along its axis clear of everything
# but wires (its tie is cut first); (2) the pigtail leaves its right-angle socket upward (carry item 2): the wires' exit direction is +z, every
# modelled bend of the bundle keeps its innermost wire at >= 3 x the insulation OD, the socket and wire exit stay under the under-carrier limit and
# >= 1 mm off the motor face, and the whole pigtail keeps WIRE_GAP to the motor and to the P03 (both gated here; neither is a route end of the
# pigtail in the generic wire check); (3) the board (with its modules, plugs) lifts 20 mm, the Jetson, rear standoffs, board screws, battery, tray
# and the joint ties out and the joints unplugged (the service sequence), without meeting anything, and the pigtails with their male housings ride
# up with it clear of every fixed part, the slid-back female housings and every other lead; (4) the pigtail's cut length is its drawn route plus its
# drawn terminations + half the pin spread (pre-cut, trimmed to length after dressing; no lift slack); (5) the joint is held down by its tie (item 10) and (6) the factory
# cable's route, with its spare length wound into the coil and tied down, takes the 200 mm cable, away from the wheels and the fan (item 3).
LIFT=20.0
LIFT_IDS=['bay_board','bay_pico','bay_drv_0','bay_drv_1','bay_imu']+sorted(i for i in shapes if i.startswith('bay_plug_'))
SERVICE_OUT={'pico_usb_plug','P_battery_tray','bay_holddown','battery','tamiya','strap_0','strap_1'}|{i for i in shapes if i.startswith(('bay_screw_','tray_screw_','jetson_standoff_1','jetson_standoff_3','jetson_screw_1','jetson_screw_3','motor_tie_joint_'))}
_lift_fixed=[i for i in SOLID if i not in set(LIFT_IDS)|SERVICE_OUT|LIDSIDE|JETSIDE and not i.startswith(('jetson_standoff','jetson_screw','motor_joint_'))] + [i for i in SOLID if i.startswith(('jetson_standoff_0','jetson_standoff_2','jetson_screw_0','jetson_screw_2'))]
def sweep_hits(moving,fixed,moves):
    """[hits] of each moving part translated by each move, against the fixed parts within reach (the move's length) of it (one distance each)."""
    reach=max(math.hypot(*m_) for m_ in moves)+.1;out=[]
    for i in moving:
        cand=[j for j in fixed if j!=i and near(shapes[i],shapes[j],reach) and sdist(shapes[i],shapes[j])<=reach]
        for m_ in moves:
            q=shapes[i].translate(m_);out+=[dict(part=i,move=list(m_),**h) for h in overlaps(q,ids=cand)]
    return out
LIFT_MOVES=[(0,0,dz) for dz in (1.,2.,5.,10.,15.,20.)]
lift_hits=sweep_hits(LIFT_IDS,_lift_fixed,LIFT_MOVES)
# PB25 amended: the joints are unplugged before the board is lifted (ties cut, each factory female housing slid MJ['unplug'] mm back along its axis), so
# each pigtail and its male housing ride up with the board: swept against the fixed parts, the slid-back female housings and every other lead.
UNPLUGGED={f'female housing {s_} (unplugged)':J_['female'].translate(cq.Vector(0,J_['axis'][1]*MJ['unplug'],0)) for s_,J_ in JOINT.items()}
_wires_fixed=[c['id'] for c in cables if c.get('modelled',True) and not c['id'].startswith('motor_pigtail')]
pig_lift_hits=[]
for side in JOINT:
    for nm,sh in ((f'motor_pigtail_{side}',shapes[f'motor_pigtail_{side}']),(f'male housing {side}',JOINT[side]['male'])):
        cand=[(j,shapes[j]) for j in _lift_fixed+_wires_fixed if near(sh,shapes[j],LIFT+.1)]+[(k,v) for k,v in UNPLUGGED.items() if near(sh,v,LIFT+.1)]
        for m_ in LIFT_MOVES:
            q=sh.translate(cq.Vector(*m_))
            pig_lift_hits+=[dict(part=nm,move=list(m_),hit=j,mm3=round(v_,2)) for j,fx in cand if near(q,fx) for v_ in [q.intersect(fx).Volume()] if v_>.05]
def _pl(pts):return _polylen(pts)   # (_pl itself is rebound by the Pico plug unpacking above, so it is defined here)
WHEELS=[i for i in shapes if i.startswith(('tire_','wheel_'))]
mj_rows={}
for side,J_ in JOINT.items():
    sg=J_['axis'][1];bq=J_['female'].BoundingBox();ref=PIGTAIL[side]['ref'];so=SOCKET[ref]
    q=box((bq.xmin+bq.xmax)/2,(bq.ymin+bq.ymax)/2+sg*MJ['unplug']/2,bq.zmin,bq.xlen,bq.ylen+MJ['unplug'],bq.zlen)
    unplug=overlaps(q,{f'motor_joint_{side}',f'motor_tie_joint_{side}'})
    pg=PIGTAIL[side];mot='motor_left' if sg>0 else 'motor_right';to_motor=round(shapes['motor_pigtail_'+side].distance(shapes[mot]),3)
    to_p03=round(shapes['motor_pigtail_'+side].distance(shapes['p03_regulator']),3)   # review minor 5: gated here, and the P03 is never a route end of the pigtail
    d0=np.subtract(pg['points'][1],pg['points'][0]);d0=[int(round(v)) for v in d0/np.linalg.norm(d0)]
    inner=round(min(pg['fillet_radii'])-(PG['r_run']-PG['r_wire']),3)
    uc=[round(shapes[f'bay_plug_{ref}'].BoundingBox().zmax,3),round(max(p_[2] for p_ in pg['points'] if p_[0]>=CARRIER_X)+PG['r_run'],3)]
    ex_motor=pg['exit_to_motor_mm']
    exit_=dict(board_end_part=RA_SOCKET['part'],socket_mm=dict(face_y=so['face_y'],back_y=so['back_y'],tail_y=so['tail_y'],sleeve_top_z=so['top_z']),wire_exit_dir=d0,wire_od_mm=PG['od'],
               fillet_radii_mm=pg['fillet_radii'],min_bend_radius_mm=inner,required_min_bend_mm=3*PG['od'],bend_note='innermost wire of the Ø3 bundle: fillet centreline radius - (bundle radius - wire radius)',
               top_z_mm=max(uc),under_carrier_limit_z_mm=round(UNDER_CARRIER_Z-1.5,2),to_motor_face_mm=ex_motor,pigtail_to_motor_mm=to_motor,pigtail_to_p03_mm=to_p03,required_gap_mm=WIRE_GAP,
               ok=d0==[0,0,1] and inner>=3*PG['od']-1e-9 and max(uc)<=UNDER_CARRIER_Z-1.5+1e-6 and ex_motor>=1.0 and to_motor>=WIRE_GAP and to_p03>=WIRE_GAP)
    L_nom=_pl(pg['points'])
    length=dict(route_mm=round(L_nom,2),terminations_mm=PG['term'],terminations_total_mm=PG['term_mm'],pin_spread_mm=PG['pin_spread_mm'],precut_mm=pg['precut_mm'],precut_margin_mm=pg['precut_margin_mm'],
                finished_centre_mm=pg['finished_centre_mm'],finished_range_mm=pg['finished_range_mm'],length_source='crimped',
                rule='PB25 amended + B9 fix 2 + B12a (final review N4): pre-cut = drawn route + drawn terminations + the full pin spread (the wires fan out +-spread / 2 at the socket and again at the male housing; rounded up to the mm); '
                     'socket end soldered first, bundle dressed along its route, each wire then trimmed at its own cavity (route + terminations +- the spread) and crimped; no lift slack: the joint is unplugged before the board lifts')
    length['ok']=(abs(L_nom-pg['route_exact'])<.02 and pg['precut_mm']>=L_nom+PG['term_mm']+PG['pin_spread_mm']-1e-9 and pg['precut_mm']<L_nom+PG['term_mm']+PG['pin_spread_mm']+1.0
                  and pg['finished_range_mm'][1]<=pg['precut_mm'])
    tid=f'motor_tie_joint_{side}';cid=f'motor_tie_coil_{side}'
    tie_hits=overlaps(shapes[tid],{tid})
    coil_hits=overlaps(shapes[cid],{cid})
    ret=dict(tie=tid,around=f'the female housing at y {sg*MJ["tie_y"]:g}',through='the motor opening (inner leg) and a P_base slot (outer leg)',gap_to_joint_mm=round(shapes[tid].distance(shapes[f'motor_joint_{side}']),3),
             gap_to_base_mm=round(shapes[tid].distance(shapes['P_base']),3),head_mm=list(TIE['head']),clashes=tie_hits,joint_on_base_gap_mm=round(shapes[f'motor_joint_{side}'].distance(shapes['P_base']),3),ok=not tie_hits)
    fcp=MOTOR_CABLE[side];fr=_pl(fcp);cpts=COIL[side]['points']
    fx_=FAC[side]   # B9 fix round 2: fixed path + coil; readings of FAC_READINGS (to the housing's rear face) all fit the coil's FAC_TURNS
    fac=dict(route_mm=round(fr,1),cable_mm=MC['length'],slack_at_housing_rear_face_mm=round(MC['length']-fr,1),fixed_path_mm=fx_['fixed'],cable_readings_mm=list(FAC_READINGS),coil_turns_allowed=list(FAC_TURNS),
             coil_turns_needed=fx_['turns_needed'],coil_bottom_z_at_max_turns_mm=fx_['bottom_z_at_max_turns'],reading_note='cable readings are to the female housing\'s rear face (the wire entry); the vendor\'s 200 mm is on the verify list',
             coil=dict(turn_length_mm=round(fx_['turn'],2),centre_mm=list(COIL[side]['centre']),r_mm=MCOIL['r'],turns=MCOIL['turns'],
             length_mm=round(_pl(cpts),1),inner_bend_radius_mm=MCOIL['r']-MC['r']+.5,z_mm=[round(min(p_[2] for p_ in cpts),2),round(max(p_[2] for p_ in cpts),2)]),slack_tie=cid,slack_tie_clashes=coil_hits,
             max_abs_y_mm=round(max(abs(p_[1]) for p_ in fcp)+MC['r'],2),to_wheels_mm=round(min(shapes['motor_cable_'+side].distance(shapes[w_]) for w_ in WHEELS if near(shapes['motor_cable_'+side],shapes[w_],40)) if any(near(shapes['motor_cable_'+side],shapes[w_],40) for w_ in WHEELS) else 40.,1),
             to_fan_mm=round(shapes['motor_cable_'+side].distance(shapes['fan']),1),min_fillet_radius_mm=min(COIL[side]['fillet_radii']),
             note='the leads leave the encoder cap as a flat row along the motor face toward the rear (drawn Ø2), then a Ø4 bundle round the rear shoulder and over the motor top; never over the pigtail; the spare length is the coil (measured variable: 2-3 turns), so no loose loop')
    fac['range_ok']=(FAC_READINGS[0]<=fr<=FAC_READINGS[1] and FAC_TURNS[0]<=fac['coil_turns_needed'][0] and fac['coil_turns_needed'][1]<=FAC_TURNS[1]
                     and abs(fac['fixed_path_mm']+fac['coil']['length_mm']-fac['route_mm'])<=.11 and fac['coil_bottom_z_at_max_turns_mm']>=BASE_TOP)
    fac['ok']=fac['range_ok'] and not coil_hits and fac['max_abs_y_mm']<=90.0 and fac['to_wheels_mm']>=10.0 and fac['to_fan_mm']>=20.0
    mj_rows[side]=dict(joint=f'motor_joint_{side}',female_box_mm=[round(v,2) for v in (bq.xmin,bq.ymin,bq.zmin,bq.xmax,bq.ymax,bq.zmax)],unplug_mm=MJ['unplug'],unplug_hits=unplug,unplug_ignored=f'{tid} (cut before unplugging)',
        pigtail_exit_bend=dict(wire_d_mm=PG['od'],inner_radius_mm=inner,to_motor_face_mm=to_motor,required_mm=WIRE_GAP),pigtail_exit=exit_,pigtail=length,
        pigtail_lift_hits=[h for h in pig_lift_hits if side in h['part']],retention=ret,factory_cable=fac,
        polarity='unkeyed 0.1 in joint: pin 1 marked on both halves (paint-pen stripe at the red wire), colours straight across',
        ok=not unplug and exit_['ok'] and length['ok'] and ret['ok'] and fac['ok'] and not [h for h in pig_lift_hits if side in h['part']])
motor_joints_ok=all(r['ok'] for r in mj_rows.values()) and not lift_hits and not pig_lift_hits
# B9 carry item 8: wire-to-wire. Every pair of harness envelopes within 5 mm of each other is measured (minimum distance; a pair that intersects
# is an overlap). The motor-side bundle - each pigtail and factory cable and every lead within 5 mm of one of them; B3's single-cable band now
# carries two cables - keeps WIRE_GAP between different cables; no other pair may interpenetrate (leads that share a lane may touch).
MOTOR_SIDE={f'motor_{k}_{s_}' for k in ('pigtail','cable') for s_ in ('left','right')}
ww_rows=[];ww_other=[]
for a_,b_ in itertools.combinations([c['id'] for c in cables if c.get('modelled',True)],2):
    if not near(shapes[a_],shapes[b_],5.0):continue
    d=round(shapes[a_].distance(shapes[b_]),3);v_=round(shapes[a_].intersect(shapes[b_]).Volume(),3) if d<1e-6 else 0.
    (ww_rows if a_ in MOTOR_SIDE or b_ in MOTOR_SIDE else ww_other).append(dict(pair=[a_,b_],min_gap_mm=d,overlap_mm3=v_))
wire_to_wire=dict(required_mm=WIRE_GAP,scope='motor-side bundle (pigtails, factory cables and every lead within 5 mm of them) at >= required_mm; every other pair within 5 mm: no interpenetration',
    rows=ww_rows,overlaps=[r for r in ww_rows if r['min_gap_mm']<WIRE_GAP],other_pairs=ww_other,other_interpenetrating=[r for r in ww_other if r['overlap_mm3']>.05])
wire_to_wire['ok']=not wire_to_wire['overlaps'] and not wire_to_wire['other_interpenetrating']
# Review I3: the tray's removal path. With the battery and its strap / Tamiya pair and the tray screws out, the tray (with its hold-down springs)
# slides TRAY_SLIDE aft along the base top and lifts 25 mm straight up: it meets nothing but P_base (the slide runs on it) and the lid side.
TRAY_IDS=('P_battery_tray','bay_holddown')
_tray_fixed=[i for i in SOLID if i not in set(TRAY_IDS)|SERVICE_OUT|LIDSIDE|{'P_base'} and not i.startswith(('insert_tray_','tray_screw_'))]
tray_hits=sweep_hits(TRAY_IDS,_tray_fixed,[(-.5,0,0),(-1,0,0),(-1.5,0,0),(-TRAY_SLIDE,0,0)]+[(-TRAY_SLIDE,0,.5*k) for k in range(1,51)])
tray_ok=not tray_hits
# Carry item 1 (amended by PB25, B9): B3's motor-cable model (board_cad.json motor_cables: the exit on the motor face at z 43, the band beside it)
# against the real routes: the factory cable starts at the exit and its flat lead row runs up the face inside the band; the pigtail starts at its
# socket's tails and its run beside U5A / J9, the P03 and the motor face (x from U5A's / J9's rear edge to the motor's front) lies in the band; the two
# cables in the band keep WIRE_GAP (wire_to_wire).
mc_rows={}
for side,ref in (('left','J4'),('right','J5')):
    m=BC['motor_cables'][ref];pg=PIGTAIL[side]['points'];fc=MOTOR_CABLE[side];nf=COIL[side]['face_points']
    xw=BCONN['U5A' if side=='left' else 'J9']['courtyard_xy'][0];run=[p_ for p_ in pg if xw<=p_[0]<=12.5];span=[round(min(abs(p_[1]) for p_ in run)-PG['r_run'],2),round(max(abs(p_[1]) for p_ in run)+PG['r_run'],2)]
    fspan=[round(min(abs(p_[1]) for p_ in fc[:nf])-MC['r_face'],2),round(max(abs(p_[1]) for p_ in fc[:nf]),2)]   # the exit point is on the face itself
    band=sorted(abs(v) for v in m['band_y']);pair=[r for r in ww_rows if set(r['pair'])=={f'motor_pigtail_{side}',f'motor_cable_{side}'}]
    mc_rows[side]=dict(factory_route_start=fc[:nf],pigtail_route=pg,exit_from_board_spec=m['exit_xyz'],factory_starts_at_exit=math.dist(fc[0],m['exit_xyz'])<.05,
        band_abs_y_from_board_spec=band,pigtail_run_abs_y_span=span,factory_face_abs_y_span=fspan,in_band=band[0]-1e-6<=span[0] and span[1]<=band[1]+1e-6 and band[0]-1e-6<=fspan[0] and fspan[1]<=band[1]+1e-6,
        exit_height_matches=abs(fc[0][2]-m['exit_xyz'][2])<.05,pigtail_starts_at_socket_tails=math.dist(pg[0],(SOCKET[ref]['pc_x'],SOCKET[ref]['tail_y'],SOCKET[ref]['top_z']))<.05,
        cables_apart_mm=pair[0]['min_gap_mm'] if pair else None,
        section=dict(board_spec_flat_mm=[m['w'],m['t']],pigtail_run_d_mm=2*PG['r_run'],factory_face_d_mm=2*MC['r_face'],factory_bundle_d_mm=2*MC['r'],note='B3 models the cable 6 × 2 flat; the CAD draws the factory leads Ø2 where they lie flat up the motor face, then a Ø4 bundle, and the pigtail as a Ø3 bundle of six Ø1.0 wires'))
mc_ok=all(r['factory_starts_at_exit'] and r['in_band'] and r['exit_height_matches'] and r['pigtail_starts_at_socket_tails'] and (r['cables_apart_mm'] or 0)>=WIRE_GAP for r in mc_rows.values())
# PB16 / review M9: C3 (DRV_L's 100 uF V-chip) at its worst-case height (5.8 + 0.3 = 6.1 mm, board_spec PART_H_TOL) under the GY-521 body.
# PB3 / review M7: both clamp holes are the board's Ø3.0 NPTH (a Ø2.95 probe clears the laminate, a Ø3.05 probe cuts it).
C3_WORST_H=6.1
_c3b=BSTEP['C3'].BoundingBox();_c3w=box((_c3b.xmin+_c3b.xmax)/2,(_c3b.ymin+_c3b.ymax)/2,ZBAY_TOP,_c3b.xlen,_c3b.ylen,C3_WORST_H)
c3_gap=round(_c3w.distance(shapes['bay_imu']),2)
_pcb=BSTEP['r1_power_io_PCB'];clamp_rows={}
for h in BC['clamp_holes']:
    _cx,_cy=h['xy'];clamp_rows[h['ref']]=dict(xy=h['xy'],d_mm=h['d'],ok=_pcb.intersect(cyl(_cx,_cy,ZBAY-1,1.475,4)).Volume()<1e-3 and _pcb.intersect(cyl(_cx,_cy,ZBAY-1,1.525,4)).Volume()>1e-3)
clamp_d_ok=all(r['ok'] for r in clamp_rows.values())
bay_ok=(p03_clear and abs(jetson_carrier_bottom_z-JO[2])<1e-6 and abs(jetson_carrier_bottom_z-57.0)<1e-6 and bay_uc_ok and bay_ribs_ok and bay_rear_ok and edge_ok and pico_usb_ok
        and plug_paths_ok and stubs_ok and mc_ok and c3_gap>=1.0 and clamp_d_ok and not slab_hits and sweeps_ok and motor_joints_ok and tray_ok)
bay_check=dict(outline_mm=dict(points=len(BAY_OUTLINE),x=[round(min(p[0] for p in BAY_OUTLINE),2),round(max(p[0] for p in BAY_OUTLINE),2)],area_mm2=BC['outline_area_mm2']),board_z_mm=[ZBAY,ZBAY_TOP],
    mounts_mm=BAY_MOUNTS,clamps_mm=BAY_CLAMPS,clamp_holes=clamp_rows,clamp_hole_d_in_step_ok=clamp_d_ok,carrier_x_mm=CARRIER_X,under_carrier_z_mm=UNDER_CARRIER_Z,
    source=dict(board='pcb/fab/r1_power_io.step (laminate + every fitted part; copper and vias left out)',data='pcb/board_cad.json',lead_trim_mm=LEAD_TRIM,lead_trim_z_mm=ZTRIM,
        lead_trim_method='the STEP leads (down to 1.9 mm under the board, untrimmed) are cut at 1.4 mm under the board, the assembly rule; every clash and rib check uses the cut leads',
        step_heights_mm=STEP_H,step_lower_than_parts_h=sorted(_lower),step_higher_than_parts_h=sorted(_higher),envelopes_at_parts_h=STEP_ENVELOPES,
        modules='bay_pico: reference-models/pico-r3/Pico-R3.step; bay_drv_0/1: reference-models/pololu-4035-drv8874-carrier.step; bay_imu: GY-521 body at PARTS h; all on the real socket pads, not mirrored'),
    p03=dict(box_mm=[round(v,2) for v in (_pb.xmin,_pb.ymin,_pb.zmin,_pb.xmax,_pb.ymax,_pb.zmax)],required_gap_mm=P03_GAP_MIN,min_gap_over_travel_mm=p03_gap,nearest=p03_rows[:5],ok=p03_clear,
        method='each bumper part at rest vs the P03 bounding box swept against its 5 mm travel (exact minimum gap over the sweep); all bumper_* / P_bumper* ids, as pcb/extract_outline.py sweeps them'),
    jetson_carrier_bottom_z_mm=jetson_carrier_bottom_z,
    under_carrier=dict(rows=uc_rows,max_top_z_mm=bay_uc_max_top,limit_z_mm=round(UNDER_CARRIER_Z-1.5,2),ok=bay_uc_ok),
    rear_row_ribs=dict(ribs=[dict(id=r,bounds=byid[r]['bounds']) for r in rib_ids],rows=rib_rows,rib_to_tht_pad=rib_pad,
        rule=f'a rib touching the board underside, under the connector body (x overlap with its F.Fab body), along >= 50 % of the body length (y); every rib >= {RIB_GAP} mm in plan from every THT pad of the real board (board_cad.json tht_pads, copper boxes); each pocket >= {RIB_WALL} mm of P_base from every other base hole',ok=bay_ribs_ok),
    rear_support=dict(rows=rear_rows,holddown_fingers=hd_rows,holddown_spring=HD_SPRING,holddown_overlap_with_board_mm3=hd_board,board_top_z_mm=BOARD_TOP,fixing_max_mm=FIX_MAX,ok=bay_rear_ok,
        no_hole_scan='No M3 hole fits the band within 35 mm of J1 / J2 / J7 / J12 on the routed board (0.25 mm grid: 4 mm to the edge, the 4.5 mm GND-only washer free of other-net pads, the Ø5.6 head clear of courtyards); '
                     'with the SMD parts movable the only spots lie under the Pico between its socket rows, which would mean moving its pull-up column and re-routing (PB1, B4-B7). The tray-wall springs hold the rear edge instead (PB24 amended).'),
    edge_connector_support=dict(rows=edge_rows,high_plug_force=sorted(HIGH_FORCE),bearing_max_mm=BEARING_MAX,ok=edge_ok),
    pico_usb_insert=dict(plug_mm=list(PICO_PLUG),receptacle_face_mm=USB_FACE,stroke_mm=PICO_STROKE,lowered_from_mm=30,hits=usb_hits,ignored='the lid side (group cover: off for service), the plug itself, the Pico it mates with (push-in only)',ok=pico_usb_ok),
    rear_plug_insert_paths=dict(rows=plug_paths,ok=plug_paths_ok),
    plug_mating_sweeps=dict(rows=sweep_rows,exempt=SWEEP_EXEMPT,rule='every plug travels its engagement + 1 mm along its pins clear of everything but wires, its own connector and the lid side; J4 / J5 exempt: plugged on the bench before the board is installed (PB25)',ok=sweeps_ok),
    motor_joints=dict(rows=mj_rows,board_lift_mm=LIFT,board_lift_hits=lift_hits,pigtail_lift_hits=pig_lift_hits,lifted=LIFT_IDS+['the pigtails and their male housings (joints unplugged, PB25 amended)'],removed_for_service=sorted(SERVICE_OUT),ok=motor_joints_ok),
    tray_removal=dict(slide_mm=TRAY_SLIDE,lift_mm=25,trim=dict(x_mm=list(TRAY_TRIM_X),y_abs_max_mm=TRAY_TRIM_Y),hits=tray_hits,ignored='P_base (the slide runs on it), the battery, its straps and Tamiya pair, the tray screws, the lid side',ok=tray_ok),
    carrier_lead_tail_slab=dict(box_mm=[JPCB[0],JPCB[2],UNDER_CARRIER_Z,JPCB[1],JPCB[3],JO[2]],hits=slab_hits,ok=not slab_hits),
    cable_stubs=dict(rows=stub_rows,length_mm=STUB_L,run_thickness_mm=CABLE_T,ok=stubs_ok),
    motor_cables=dict(rows=mc_rows,ok=mc_ok),
    c3_under_imu=dict(gap_mm=c3_gap,c3_worst_case_h_mm=C3_WORST_H,c3_worst_top_z_mm=round(ZBAY_TOP+C3_WORST_H,2),c3_model_top_z_mm=round(_c3b.zmax,2),imu_underside_z_mm=HMOD,required_mm=1.0,ok=c3_gap>=1.0),
    ok=bay_ok)
print('bay',json.dumps(dict(p03_gap=p03_gap,carrier_z=jetson_carrier_bottom_z,uc_top=bay_uc_max_top,ribs=bay_ribs_ok,rear=bay_rear_ok,edge=edge_ok,usb=pico_usb_ok,plugs=plug_paths_ok,stubs=stubs_ok,
    sweeps=sweeps_ok,joints=motor_joints_ok,tray=tray_ok,motor_cables=mc_ok,c3=c3_gap,clamp=clamp_d_ok,slab=len(slab_hits),spring=HD_SPRING['E2000']['nominal'])),flush=True)
report=dict(clash_scan=dict(scope=f'all {len(SOLID)} non-wire parts pairwise (solid intersection > 0.05 mm3; the bumper halves are modelled with the leaf deflected onto its abutment, so P_base is no longer exempt); fasteners, inserts, races and moving hardware by minimum gap; wires by minimum gap (>= {WIRE_GAP} mm to every non-terminal part)',clashes=clashes,intended_contacts=intended),
    wire_clashes=wire_clashes,wire_gaps_under_1mm=wire_near,minimum_wire_z_mm=min(p['bounds'][2] for p in parts if NOW(p['id'])),printed_parts=check,
    major_component_clashes=clashes,major_components_checked=len(SOLID),printed_part_overlaps=printed_overlaps,printed_part_near_contacts=dict(violations=printed_gaps,required_mm=1.0),
    fastener_clearances=dict(rows=fast_rows,violations=fast_bad,note='Static fasteners, inserts, nuts, module screws, standoffs, dowels, base races and switch bodies vs every part they are not designed to touch: >= 0.5 mm; moving pins, bearings, springs, riders, races and balls (5 positions) and the moving halves: >= 2.0 mm to parts, >= 1.0 mm to wires; rigid half to P_base >= 1.0 mm at every travel; base races to moving parts >= 0.7 mm; switch body vs moving half >= 1.0 mm; leaf vs switch >= 1.0 mm, leaf body vs its abutment >= 0.3 mm, leaf abutment vs switch >= 1.0 mm.'),
    base_hole_walls=dict(violations=base_walls,required_mm=1.2,holes_checked=len(BH)),blind_pilots=dict(rows=pilots,ok=pilot_ok,rule='thread-forming screws >= 5 mm engagement in PETG; machine screws >= 2.5 mm (5 threads) inside a brass insert; tips from below >= 0.8 mm inside the base top'),
    ball_tracks=dict(rows=ball_tracks,ok=ball_tracks_ok),
    carrier_lead_tail_slab=dict(hits=slab_hits,note='the Jetson carrier footprint, z 54.77..57 (2.23 mm of through-hole lead tails): only the Jetson and its standoff stacks may enter it'),
    clash_scan_empty=not clashes and not fast_bad and not wire_clashes and not base_walls and not printed_gaps and not slab_hits and wire_to_wire['ok'],wire_to_wire=wire_to_wire,
    envelope=env,envelope_max_r_mm=env['envelope_max_r_mm'],bumper_travel=bumper,perimeter_rod_intrusion=perim,spin_sweep=spin_sweep,rear_shell=rsr,camera_fov=camera_fov,camera_window=camera_window,cliff_fov=cliff_fov,lidar_mount=lidar_check,
    caster_layout=caster_layout,com_estimate=com,threshold_crossing=threshold,jetson_cooling=jetson_cooling,robot_top=robot_top,
    lid_switches=dict(rows=lid_switches,ok=lid_switches_ok,rule='switch body bottom >= 10 mm over the battery top (z 64); >= 2 mm to the battery, the Tamiya pair, the straps and the bay board modules'),
    interlayer_stress=interlayer_check,bom_consistency=bom_check,all_solids_valid=all(shapes[p['id']].isValid() for p in parts),test_coupon=COUPON,
    limitations=['Electronic envelopes and some mounting patterns remain assumptions; this is not a print-release or wiring-release check.',
                 'Friction-lock, Hertz, interlayer-stress, spin-sweep, coast and threshold-crossing results are quasi-static models; physical bump, tipping, stopping-distance, threshold and proof tests remain pending.',
                 'The camera window (task F) is open: an object small enough to pass it (about the size of the opening at the inner wall, camera_window.at_inner_wall) can reach the lens without moving a bumper half (camera_window.residual); anything larger trips the bumper first.',
                 'Spinning in place next to an unmapped edge 104-113 mm from the axle can drop a wheel before a cliff sensor trips (caster_layout.spin_cliff_coverage). User decision (controller rule or extra sensor).',
                 'Steps of 8 and 10 mm fail (threshold_crossing): the DriveCore torque cap and the 25.4 mm caster ball. User decision.'])
report.update(p03_clear_of_bumper_travel=p03_clear,p03_bumper_travel_min_gap_mm=p03_gap,jetson_carrier_bottom_z=jetson_carrier_bottom_z,bay_under_carrier_max_top_z=bay_uc_max_top,bay_ribs_under_rear_row=bay_ribs_ok,
    bay_rear_connectors_supported=bay_rear_ok,pico_usb_insert_path_clear=pico_usb_ok,under_carrier_cable_paths_clear=stubs_ok,motor_joints_serviceable=motor_joints_ok,bay_board=bay_check)
# LIO-169: the P03 drawn as the real HW-316E board. Its render solids stay inside the measured envelope (asserted where they are built).
# Its cable tie (a 2.5 mm band in the yz plane at x P03_TIE_X, held there by its 3.1 mm P_base slots) must clear the end terminals and their
# wire entries, the lead paths, the trimpot and its screw, the display and the buttons; it bears on the highest parts under its band.
_band=(P03_TIE_X-TIE['w']/2-TIE['slot_clr'],P03_TIE_X+TIE['w']/2+TIE['slot_clr'])   # where the band can sit in its slot
def _xgap(lo,hi):return round(max(_band[0]-hi,lo-_band[1]),2)   # > 0: clear along x by that much
_clear={k:_xgap(P03_PARTS[k][0],P03_PARTS[k][3]) for k in ('IN terminal','OUT terminal','trimpot','trimpot screw','display','buttons','XL4015 heatsink')}
_clear.update({f'{k} wire entry':_xgap(e[0]-1.8,e[0]+1.8) for k,e in sorted(P03_ENT.items())})
_clear.update({f'{c} lead path':_xgap(min(q[0] for q in v['points'][:3]),max(q[0] for q in v['points'][:3])) for c in ('W_p03_in','W_p03_out') for v in [next(c_ for c_ in cables if c_['id']==c)]})
_under=sorted(((round(b[5],2),k) for k,b in P03_PARTS.items() if b[0]<_band[1] and b[3]>_band[0]),reverse=True)
P03_TIE=dict(x_mm=P03_TIE_X,band_x_range_mm=[round(v,2) for v in _band],slots_mm=[[P03_TIE_X,y] for _,y in P03_TIES],slot_size_mm=[round(TIE['w']+2*TIE['slot_clr'],2),2.6],
    clearance_along_x_mm=_clear,parts_under_band_top_z_mm={k:z for z,k in _under},bears_on=[k for z,k in _under if z>=_under[0][0]-1e-6],
    clear=all(v>0 for v in _clear.values()))
assert P03_TIE['clear'],P03_TIE
report['p03_model']=dict(board='HW-316E V4.0 (XL4015 5 A buck, 3-digit LED voltmeter), the generic board sold as "DC-DC 5A Buck Converter 4-38V to 1.25-36V ... with LED Display" (Amazon B085T73CSD)',
    sources=['DIYmore product photos of the same HW-316E V4.0 board: https://www.diymore.cc/products/xl4015-5a-dc-dc-adjustable-step-down-board-led-voltmeter-power-supply-module-4-0-38v-to-1-25v-36v-diy-high-power-75w',
             'Amazon B085T73CSD listing (title only; the page body was not readable)','user measurement: 62 × 38 × 14 mm (the envelope, kept as the truth for size)'],
    render_solids=len(P03_RENDER),colours=len({c for _,c in P03_RENDER}),inside_envelope=True,top_z_mm=round(_rb.zmax,2),tallest='XL4015 clip-on heatsink (12.4 mm over the PCB top)',
    orientation='rotation, not a mirror: IN terminal on the +y short end (the robot left, the end nearer U5A / U5B), OUT on the -y end; top edge (caps, trimpot) forward, display and buttons aft',
    terminal_entries_mm={k:[round(v,2) for v in e] for k,e in sorted(P03_ENT.items())},leads=P03_LEADS,
    tie=P03_TIE,tie_clear_of_terminals=P03_TIE['clear'],
    open_item=dict(status='resolved',text='The first build (1b031a3) had the tie slots at (28, ±33.3), in front of the terminals\' wire entries. Resolved by the controller\'s ruling (option a): '
                   f'the tie line moved to x {P03_TIE_X:g} over the two cap tops (bears on: {", ".join(P03_TIE["bears_on"])}); the board stays on its foam tape at the same height.'))
rc=subprocess.run([sys.executable,str(ROOT/'cad'/'check_printability.py')]).returncode

rcc=subprocess.run([sys.executable,str(ROOT/'cad'/'check_printability.py'),str(COUPONS),str(ROOT/'cad'/'printability-coupons.json')]).returncode;rc=rc or rcc
prc=json.loads((ROOT/'cad'/'printability-coupons.json').read_text())
subprocess.run([sys.executable,str(ROOT/'cad'/'validate_meshes.py')],stdout=subprocess.DEVNULL)
pr=json.loads((ROOT/'cad'/'printability.json').read_text())
report['printability']=dict(all_pass=pr['all_pass'] and prc['all_pass'],parts_checked=pr['parts_checked'],parts_failed=pr['parts_failed'],details='cad/printability.json',coupons=dict(checked=prc['parts_checked'],failed=prc['parts_failed'],details='cad/printability-coupons.json'),
    slicer='P_base: no brim, skirt 1 mm or off; rear shells and bumper halves: 4 perimeters, ~245 °C, cooling <= 30 %, 5 mm mouse ears at the arc ends.')
BH_=bumper['halves']
# Camera window (task F): the rods and seam rods that enter it (they reach the camera unsensed: the residual) and its edge faces in the spin sweep.
wrods=perim['through_camera_window_not_bumper_sensed']
camera_window.update(rods_through_window=wrods,rod_rows_through_window=len(wrods),rod_first_contacts=sorted({c for r in wrods for c in r['contacts']}),
    seam_rod_heights_without_contact=sorted({r['z'] for r in bumper['seam_intrusion']['rows'] if r['first_contact']=='none'}),
    spin_edge_faces=len(win_faces),spin_edge_snags=notch_snags,
    widest_rod_through_window_az_deg=round(max([abs(r['az_from']) for r in wrods]+[abs(r['az_to']) for r in wrods],default=0.),1))
camera_window['ok']=camera_window['ok'] and not notch_snags
report['summary']=dict(envelope_ok=env['ok'],envelope_worst_case_r_mm=env['worst_case_r_mm'],clash_scan_empty=report['clash_scan_empty'],arc_sensed_to_deg=min(BH_[s]['arc_sensed_to_deg'] for s in BH_),
    friction_lock=any(BH_[s]['friction_lock'] for s in BH_),friction_margin_ok=bumper['friction_lock']['margin_ok'],hertz_ok=bumper['hertz']['ok'],preload_ok=bumper['preload_balance']['ok'],
    tilt_under_load_ok=bumper['tilt_under_load']['ok'],seam_ok=bumper['seam']['ok'],seam_min_gap_with_play_mm=bumper['seam']['with_planar_play']['min_gap_mm'],seam_intrusion_ok=bumper['seam_intrusion']['ok'],
    seam_max_intrusion_before_trip_mm=bumper['seam_intrusion']['max_intrusion_before_trip_mm'],switch_ok=bumper['switch']['operates_ok'],leaf_ok=bumper['leaf_spring']['ok'],leaf_axial_ok=bumper['leaf_spring']['axial_ok'],
    leaf_nose_contact_ok=bumper['leaf_spring']['nose_contact']['ok'],
    perimeter_rod_ok=perim['ok'],spin_sweep_ok=spin_sweep['ok'],pressed_spin_ok=spin_sweep['pressed_spin_ok'],rear_shell_dowels_ok=rsr['dowels_ok'],bumper_end_gap_ok=rsr['bumper_end_gap_ok'],
    camera_fov_ok=camera_fov['ok'],cliff_fov_ok=cliff_fov['ok'],lidar_band_ok=lidar_check['scan_band_ok'],lidar_min_range_ok=lidar_check['min_range']['ok'],lidar_front_gap_ok=lidar_check['front_gap']['ok'],lidar_scan_band_vertical_ok=lidar_check['scan_band_vertical']['ok'],jetson_cooling_ok=jetson_cooling['ok'],fan_bypass_fraction_of_grille=jetson_cooling['fan_skirt']['bypass_fraction_of_grille_open_area'],lid_switches_ok=lid_switches_ok,
    robot_top_ok=robot_top['ok'],robot_height_mm=robot_top['height_mm'],lidar_scan_plane_z_mm=robot_top['lidar_scan_plane_z_mm'],camera_lens_axis_z_mm=robot_top['camera_lens_axis_z_mm'],caster_layout_ok=caster_layout['ok'],com_ok=com['ok'],
    printability_ok=pr['all_pass'] and prc['all_pass'],screw_engagement_ok=pilot_ok,ball_tracks_ok=ball_tracks_ok,bom_consistency_ok=bom_check['ok'],interlayer_ok_at_drivecore_speed=interlayer_check['ok_at_drivecore_speed'],
    threshold_crossing={k:{d:v[d]['verdict'] for d in ('forward','reverse')} for k,v in threshold.items() if k.startswith('step')},threshold_capability_mm=threshold['capability_mm'],
    camera_window_ok=camera_window['ok'],camera_window=dict(width_mm=camera_window['at_skin']['width_mm'],z_mm=camera_window['at_skin']['z_mm'],lens_recess_mm=camera_window['lens_recess_behind_skin_mm'],
        min_gap_camera_to_moving_halves_mm=camera_window['min_gap_camera_to_moving_halves_mm'],spin_edge_snags=len(notch_snags)),
    residual_unsensed=dict(seam_end_land_spin_snags=len(land_snags),seam_thin_object_intrusion_before_trip_mm=bumper['seam_intrusion']['max_intrusion_before_trip_mm'],
        camera_window_rod_rows=len(wrods),camera_window_small_objects=camera_window['residual'],note='reported, not hidden: see spin_sweep.seam_end_land_note, bumper_travel.seam_intrusion and camera_window'),
    spin_cliff_coverage=dict(user_decision_required=True,worst_band_deg=max(max(r['unsafe_start_heading_band_deg'].values()) for r in caster_layout['spin_cliff_coverage']['rows'])),
    bay_ok=bay_ok,p03_clear_of_bumper_travel=p03_clear,p03_bumper_travel_min_gap_mm=p03_gap,jetson_carrier_bottom_z=jetson_carrier_bottom_z,bay_under_carrier_max_top_z=bay_uc_max_top,bay_ribs_under_rear_row=bay_ribs_ok,
    bay_rear_connectors_supported=bay_rear_ok,pico_usb_insert_path_clear=pico_usb_ok,under_carrier_cable_paths_clear=stubs_ok,motor_joints_serviceable=motor_joints_ok,bay_edge_connectors_supported=edge_ok,
    plug_mating_sweeps_clear=sweeps_ok,tray_removal_clear=tray_ok,wire_to_wire_ok=wire_to_wire['ok'],holddown_lead_in_ok=HD_SPRING['lead_in_ok'])
JD=lambda o:o.item() if hasattr(o,'item') else str(o)   # numpy scalars
(ROOT/'cad'/'validation.json').write_text(json.dumps(report,indent=2,default=JD))
# Mass properties for the simulator sync: records under the simulator's item names (wheels and caster ball are separate bodies there), at
# the CAD solid centroids, and the body inertia tensor about the body centre of mass (each group a uniform solid of its CAD envelope;
# printed parts at their fill, wiring as point masses along the routes).
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
def gprop(ids):
    P=GProp_GProps()
    for i in ids:q=GProp_GProps();BRepGProp.VolumeProperties_s(shapes[i].wrapped,q);P.Add(q)
    c=P.CentreOfMass();M=P.MatrixOfInertia();return P.Mass(),np.array([c.X(),c.Y(),c.Z()]),np.array([[M.Value(i,j) for j in (1,2,3)] for i in (1,2,3)])
RIDS={'Battery':['battery'],'Jetson + cooler':['jetson','jetson_heatsink','fan'],'Left motor':['motor_left'],'Right motor':['motor_right'],'Motor toggle and master switches':['motor_switch','master_switch','arm_button'],
    'Caster housing':['caster_housing'],'Camera':['camera_board','camera_lens'],'D200 LiDAR':['lidar','lidar_turret','lidar_motor','lidar_cap'],'LiDAR adapter and added cables':['lidar_adapter'],
    'Bumper foam F02 (EVA ~0.07 g/cm3, estimate)':['bumper_foam_left','bumper_foam_right']}
RIDS.update({'Controller and sensors: '+n:ids for n,_,ids in CTRL});RIDS.update({'Power boards + relay: '+i:[i] for i,_ in PW})
IREC=[]   # (mass, centre, inertia about own centre) in kg, mm, kg mm2
PRINTED_IDS={q['id'] for q in parts if q['printable']}   # printed parts (P_* and the bay board's printed spacers and ribs)
for r in recs:
    if r['sim_item'] in ('Wheels','Caster ball'):continue
    ids=[r['item']] if r['item'] in PRINTED_IDS else (STEEL if r['sim_item']=='Fasteners' else RIDS.get(r['item']))
    if r['sim_item']=='Fasteners':
        for i in STEEL:m_=shapes[i].Volume()*7.85e-6;V_,c_,I_=gprop([i]);IREC.append((m_,c_,I_*m_/V_))
    elif r['sim_item']=='Wiring + connectors':
        L_=sum(l for l,_ in segs);IREC+=[(r['mass_kg']*l/L_,np.array(c_),np.zeros((3,3))) for l,c_ in segs]
    elif ids:V_,c_,I_=gprop(ids);IREC.append((r['mass_kg'],c_,I_*r['mass_kg']/V_))
    else:IREC.append((r['mass_kg'],np.array(r['position_mm']),np.zeros((3,3))))
mI=sum(m for m,_,_ in IREC);cI=sum(m*c for m,c,_ in IREC)/mI
IB=sum(I+m*((d@d)*np.eye(3)-np.outer(d,d)) for m,c,I in IREC for d in [c-cI])
simrec={}
for r in recs:
    k=r['sim_item'];e=simrec.setdefault(k,dict(item=k,mass_kg=0.,m_pos=[0.,0.,0.]))
    e['mass_kg']+=r['mass_kg'];e['m_pos']=[e['m_pos'][j]+r['mass_kg']*r['position_mm'][j] for j in range(3)]
mp=[]
for k,e in simrec.items():
    pos=[round(e['m_pos'][j]/e['mass_kg'],2) for j in range(3)]
    mp.append(dict(item=k,mass_kg=round(e['mass_kg'],5),position_mm=pos,separate_body=k in ('Wheels','Caster ball'),basis='printed: CAD volume × PETG 1.27 × fill at the CAD solid centroid' if k in PRINTED_IDS else 'simulator record mass at the CAD centroid (steel from CAD volume)'))
mb=[r for r in mp if not r['separate_body']];MB=sum(r['mass_kg'] for r in mb);BCOM=[round(sum(r['mass_kg']*r['position_mm'][j] for r in mb)/MB,2) for j in range(3)]
(ROOT/'cad'/'mass-properties.json').write_text(json.dumps(dict(units='kg, mm, robot frame (base_link at the axle centre on the floor)',body_mass_kg=round(MB,4),
    body_com_mm=BCOM,total_mass_kg=round(MT,4),total_com_mm=[round(v,2) for v in COMv],
    body_inertia_kg_m2=[[round(float(v)*1e-6,6) for v in row] for row in IB],body_inertia_about='body_com_mm (the inertia set was built from the same records: centre '+str([round(float(v),2) for v in cI])+' mm)',
    caster_x_mm=CX,wheel_x_mm=AX,records=mp,
    sync_note='Scripts/sync_robot_data.py should take its records from here (CAD solid centroids, no ballast, fill per part), keep Wheels / Caster ball as their own bodies, set drive_axle_x_m = wheel_x / 1000 and caster_x_m = caster_x / 1000 from parameters.json, and use body_inertia_kg_m2 instead of a uniform box.'),indent=2,default=JD))
CR_=[CLIFF[2],CLIFF[3]];PARAMS['camera_window']=dict(lens_face_x_mm=CAMO.x,axis_z_mm=CH,at_skin=camera_window['at_skin'],at_inner_wall=camera_window['at_inner_wall'],lens_recess_mm=camera_window['lens_recess_behind_skin_mm'],note='task F: the camera looks out through a window cut across the bumper seam (half in each half); the bumper is full height (z '+f'{ZB1:g}'+') everywhere else')
PARAMS.update(com_estimate_mm=com['com_mm'],com_estimate_note='total COM including the wheels and the caster ball; body_com_mm excludes them (the simulator Body)',body_com_mm=BCOM,
    total_mass_estimate_kg=com['total_mass_kg'],body_mass_kg=round(MB,3),body_inertia_kg_m2=[[round(float(v)*1e-6,6) for v in row] for row in IB],
    skin_z=[26,ZFT],skin_az_deg=[round(math.degrees(math.asin(G/R_SKIN)),2),AZF],skin_note=f'EVA foam skin R{R_SKIN} for az ~4..70 (seam-end ramps from az {math.degrees(math.asin(G/R_SKIN)):.2f}); bare PTFE-taped shell R{R_SH} for 70..{AZT}, tapering to R{R_TIP} at 90',
    seam_mm=2*G,seam_end_ramp=dict(depth_mm=RAMP_D,angle_from_tangent_deg=RAMP_A),rear_shell_outer_az_deg=[RS_A0,180],rear_shell_lead_in=dict(r_at_start_mm=R_RS-RS_LEAD[0],over_deg=RS_LEAD[1]),rear_shell_flange_az_deg=[RFL[2],RFL[3]],rear_shell_flange_r_mm=[RFL[0],RFL[1]],rear_shell_z=[ZB0,ZB1],bumper_end_taper=dict(az_deg=[AZT,90],r_mm=[R_SH,R_TIP]),
    bumper_dir_left=[round(ft[0],4),round(ft[1],4)],bumper_dir_right=[round(ft[0],4),round(-ft[1],4)],switch_trip_mm=bumper['switch']['operates_by_travel_mm'],bumper_side_play_mm=SPLAY,bumper_max_yaw_deg=round(math.degrees(PHMAX),3),
    envelope_worst_case_r_mm=env['worst_case_r_mm'],
    floor_clearance_mm=round(float(BODYV[:,2].min()),2),floor_clearance_note=f'bumper, rear shell and pins z 18; caster housing z 14',
    height_mm=round(max(p['bounds'][5] for p in parts),1),rear_caster_ball_centre=[CX,0,RB],rear_caster_note='rear_caster is the floor contact point',
    base_x_note='P_base spans x -124..120 (centre x -2), R148 disc clipped to |y| <= 100',
    camera_lower_edge_bumper_clearance_mm=min(r['bumper_clearance_to_frustum_mm'] for r in camrows),
    coast_safe_reverse_speed_mps=caster_layout['stopping']['max_coast_safe_reverse_speed_mps'],caster_load_fraction=com['caster_load_fraction'],rear_caster_load_N=com['rear_caster_load_N'],
    rear_cliff_lead_over_caster_mm=caster_layout['rear_cliff_lead_over_caster_reverse_mm'],rear_cliff_coast_at_reverse_limit_mm=caster_layout['stopping']['coast_without_rolling_drag'],
    threshold_capability_mm=dict(**threshold['capability_mm'],step_8mm=threshold['step_8mm']['forward']['verdict']+' forward / '+threshold['step_8mm']['reverse']['verdict']+' reverse',
        step_10mm=threshold['step_10mm']['forward']['verdict']+' forward / '+threshold['step_10mm']['reverse']['verdict']+' reverse',meaning='CAD quasi-static analysis with the DriveCore torque cap: 8 and 10 mm thresholds are NOT a hardware capability; a simulator pass over them is not evidence the robot can cross them.'),
    sim_contract=dict(drive_axle_x_m=AX/1000,caster_x_m=CX/1000,caster_ball_centre_m=[CX/1000,0,RB/1000],wheel_offset_cm=[AX/10,round(PARAMS['wheel_track']/20,1),4.0],
        cliff_rays_m=[[x/1000,y/1000,CZ/1000] for x,y in CLIFF],cliff_order='FL, FR, RL, RR',
        collider=dict(shape='round: vertical cylinder sectors about (0,0)',front_arcs_r_m=round(R_SKIN/1000,4),front_arcs_az_deg=[-90,90],rear_shell_r_m=R_RS/1000,rear_shell_az_deg=[RS_A0,360-RS_A0],z_m=[ZB0/1000,ZB1/1000]),
        bumpers=dict(left_switch='az 0..+90 (left half)',right_switch='az 0..-90 (right half)',seam_mm=2*G,camera_window='see camera_window: no bumper surface inside the camera window round the lens (|az| < at_skin.half_width_deg, z at_skin.z_mm); full height elsewhere',zones_note='a centre hit can trip either or both switches'),
        reverse_speed_limit_m_s=PARAMS['reverse_speed_limit_m_s'],infrared_light_bump_sensors=0,mass_properties='cad/mass-properties.json (records, body COM, body inertia)',
        sync_guard='replace the wheel_x == 42 guard with the values here; drop the old bought-part positions'))
PARAMS['bay_board']=dict(outline_mm=BAY_OUTLINE,z_bottom_mm=ZBAY,thickness_mm=BAY_T,z_top_mm=ZBAY_TOP,mounts_mm=BAY_MOUNTS,clamps_mm=BAY_CLAMPS,carrier_x_mm=CARRIER_X,under_carrier_z_mm=UNDER_CARRIER_Z,
    connectors_mm=BAY_CONN,rear_row_courtyards_mm=BAY_REAR_ROW,ribs_mm=BAY_RIBS,holddown_fingers_mm=hd_rows,pico_usb_face_mm=USB_FACE,lead_trim_mm=LEAD_TRIM,
    p03_box_mm=[P03C[0]-P03S[0]/2,-P03S[1]/2,33,P03C[0]+P03S[0]/2,P03S[1]/2,34+P03S[2]],note='bay B8: the real routed board (pcb/fab/r1_power_io.step, pcb/board_cad.json); connectors_mm are the connector body centres (harness ends)')
(ROOT/'cad'/'parameters.json').write_text(json.dumps(PARAMS,indent=2,default=JD))
print(json.dumps(dict(components=len(parts),printed=len(check),clashes=clashes[:10],wire_clashes=wire_clashes[:10],fast_bad=fast_bad[:10],printed_gaps=printed_gaps[:10],summary=report['summary'],com=com['com_mm'],seconds=round(time.time()-T0)),indent=2,default=JD))
sys.exit(rc)
