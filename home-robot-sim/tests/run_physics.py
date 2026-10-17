"""Exercise the real Unreal/Chaos process; truth is read only for scoring."""
import hashlib, json, math, os, pathlib, socket, subprocess, sys, time
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'controller'))
from robot_client import RobotClient,wrap
ENGINE=pathlib.Path(os.environ.get('UE_ROOT','C:/Program Files/Epic Games/UE_5.8' if os.name=='nt' else '/Users/Shared/Epic Games/UE_5.8'))
EDITOR_CMD=ENGINE/('Engine/Binaries/Win64/UnrealEditor-Cmd.exe' if os.name=='nt' else 'Engine/Binaries/Mac/UnrealEditor-Cmd')
REPORT=ROOT/'reports';REPORT.mkdir(exist_ok=True)
results=[];traces={};sensor_cost={};TAGS_USED=set()
SOURCES=['Source/HomeRobotSim/DriveCore.h','Source/HomeRobotSim/RobotGeometry.h','Source/HomeRobotSim/RobotPawn.cpp','Source/HomeRobotSim/RobotPawn.h','Content/RobotData/robot-parameters.json']
import re as _re
# DriveCore caps reverse speed (max_reverse); reverse trials drive at that speed and get time for it.
MAX_REVERSE=float(_re.search(r'max_reverse=([0-9.]+)',(ROOT/'Source/HomeRobotSim/DriveCore.h').read_text().replace(' ','')).group(1))
SCENARIOS=['empty','obstacle','cliff','threshold','bumpbench'];EXTRA_RUNS=[('empty',['-NoCaster']),('bumpbench',['-BumperSparseColumns']),('threshold',['-MassScale=1.12'])]
# ---------------- Round robot geometry (robot-parameters.json, written by Scripts/sync_robot_data.py) ----------------
# Robot frame: x forward, y LEFT, origin base_link = shell centre = drive-axle midpoint. Lengths below in cm unless named _m.
PARAMS=json.loads((ROOT/'Content/RobotData/robot-parameters.json').read_text())
R_S=100*PARAMS.get('body_radius_m',.1595)              # bumper contact skin
R_B=100*PARAMS.get('rigid_radius_m',.158)              # rigid rear shell
R_F=min(R_S-100*PARAMS.get('bumper_travel_m',.005),R_B) # bumper hard stop (front collider radius), RobotGeometry::front_stop_radius
AXLE_M=PARAMS.get('drive_axle_x_m',0.);TRACK_M=PARAMS.get('wheel_track_m',.228);WHEEL_R_M=PARAMS.get('wheel_radius_m',.04)
CASTER_M=PARAMS.get('rear_caster_xy_m',[-.075,0.])      # the ONE rear ball caster
CLIFF_M=PARAMS.get('cliff_sensors_m',[[.105,.072],[.105,-.072],[-.108,.072],[-.108,-.072]])  # FL, FR, RL, RR
SPAN=PARAMS.get('bumper_span_deg',90.)
SHELL_BOTTOM=100*PARAMS.get('shell_bottom_z_m',.018)   # lowest rigid shell edge above the floor (cm)
BASE_H_M=PARAMS.get('base_link_height_m',.085)         # simulated body origin above base_link (RobotPawn BodyHeight)
WHEEL_R=100*WHEEL_R_M
def _hull(pts):
    pts=sorted(pts);cross=lambda o,a,b:(a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0]);lo=[];hi=[]
    for p in pts:
        while len(lo)>=2 and cross(lo[-2],lo[-1],p)<=0:lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi)>=2 and cross(hi[-2],hi[-1],p)<=0:hi.pop()
        hi.append(p)
    return lo[:-1]+hi[:-1]                              # counter-clockwise
# The pawn's body collider in plan (cm): convex hull of a 64-gon circumscribing R_F over the bumper span and R_B behind it.
COLLIDER=_hull([(r*math.cos(a),r*math.sin(a)) for a in (2*math.pi*(i+.5)/64 for i in range(64))
                for r in [(R_F if abs(math.degrees(math.remainder(a,2*math.pi)))<=SPAN else R_B)/math.cos(math.pi/64)]])
# Bottom ring samples for floor clearance: the collider outline plus the bumper SKIN (R_S, outside the collider's hard stop)
# over the span, whose bottom edge is at the same height (CAD bumper_z[0]) and grounds first when the nose pitches down.
SHELL_RING=(COLLIDER+[((a[0]+b[0])/2,(a[1]+b[1])/2) for a,b in zip(COLLIDER,COLLIDER[1:]+COLLIDER[:1])]
            +[(R_S*math.cos(math.radians(az)),R_S*math.sin(math.radians(az))) for az in range(-int(SPAN),int(SPAN)+1,2)])
def tip_angle_deg(reach):
    """Pitch (deg) about the drive axle at which the shell bottom edge `reach` cm from the axle meets a flat floor."""
    k=WHEEL_R-SHELL_BOTTOM;return math.degrees(math.asin(WHEEL_R/math.hypot(reach,k))-math.atan2(k,reach))
# Grounding angles are ABSOLUTE pitch from level (truth pitch_deg, + = nose up). The front uses the bumper skin (R_S): its
# bottom edge is at shell_bottom_z_m and lies outside the collider's hard stop, so it touches the floor first.
NOSE_DOWN_GROUND_DEG=tip_angle_deg(R_S-100*AXLE_M)    # the front skin bottom touches the floor (no front caster)
NOSE_UP_GROUND_DEG=tip_angle_deg(R_B+100*AXLE_M)      # the rear shell rim touches the floor (only without the caster)
CASTER_LIFT=100*PARAMS.get('rear_caster_z_m',0.)       # ball bottom above the wheel contact plane with the body level (cm)
CASTER_ARM=abs(100*CASTER_M[0]-100*AXLE_M)             # caster contact behind the axle (cm)
REST_PITCH_DEG=math.degrees(math.atan2(CASTER_LIFT,CASTER_ARM))  # nose-up resting pitch from the caster lift (0 when touching)
# Highest step the rear caster can stand on with both wheels on the floor before the front skin grounds (the lift is first
# taken up, then the body pitches nose-down about the axle).
CASTER_STEP_LIMIT_MM=10*(CASTER_LIFT+CASTER_ARM*math.tan(math.radians(NOSE_DOWN_GROUND_DEG)))
def static_nose_down_deg(step_mm):
    """Static nose-down pitch (deg) with the caster on a step of step_mm and both wheels on the floor."""
    return math.degrees(math.asin(max(-1.,min(1.,(step_mm/10-CASTER_LIFT)/CASTER_ARM))))
# Probe rows (cm), as RobotGeometry::bump_rows: shell bottom + 0.2 mm, the fixed rows up to 2 mm under the bumper top, and
# a top row there when the last fixed row is > 3 mm below it.
BUMPER_TOP=100*PARAMS.get('bumper_top_z_m',.12)
def _rows():
    top=min(BUMPER_TOP,100*PARAMS.get('body_top_z_m',.14))-.2;out=[SHELL_BOTTOM+.02]
    out+=[z for z in (2.8,3.6,4.4,5.2,6.5,8.0,9.5,11.0,12.5) if z>out[0]+.3 and z<=top+1e-9]
    if top-out[-1]>.3+1e-9:out.append(top)
    return out
BUMP_ROWS=_rows()
def collider_gap(point):
    """Signed plan distance (cm) from a robot-frame point to the body collider: + outside, - inside."""
    px,py=point;n=len(COLLIDER);best=math.inf;inside=True
    for i in range(n):
        (ax,ay),(bx,by)=COLLIDER[i],COLLIDER[(i+1)%n];ex,ey=bx-ax,by-ay
        if ex*(py-ay)-ey*(px-ax)<0:inside=False
        t=max(0.,min(1.,((px-ax)*ex+(py-ay)*ey)/(ex*ex+ey*ey)));best=min(best,math.hypot(px-ax-t*ex,py-ay-t*ey))
    return -best if inside else best
def _edges(e):
    e=sorted(e);return [-x for x in reversed(e)]+e if e[0]>0 else e
EDGES=_edges(PARAMS.get('bump_zone_edges_deg',[-90,-55,-20,20,55,90]))
CENTRE=next(i for i in range(len(EDGES)-1) if EDGES[i+1]>=0)
def zone_of(az):
    """Zone index of an azimuth (deg about base_link, + = left); an edge belongs to the zone nearer the centre (RobotGeometry.h)."""
    if az>=0:return next((i for i in range(CENTRE,len(EDGES)-1) if az<=EDGES[i+1]),len(EDGES)-2)
    return next((i for i in range(CENTRE,-1,-1) if az>=EDGES[i]),0)
def switches(az):
    z=zone_of(az);return z>=CENTRE,z<=CENTRE                 # (bumper_left, bumper_right)
COLUMN_COUNT=math.ceil(2*SPAN/4-1e-9)+1;COLUMN_STEP=2*SPAN/(COLUMN_COUNT-1)
SPARSE_AZ=[-SPAN+COLUMN_STEP*k for k in range(0,COLUMN_COUNT,4)]   # columns kept by -BumperSparseColumns
MIDS=[a+COLUMN_STEP*2 for a in SPARSE_AZ[:-1] if abs(a+COLUMN_STEP*2)<=50]  # azimuths midway between kept columns
MID_Y=[round((R_F+1)*math.sin(math.radians(m)),2) for m in MIDS]          # pole offsets touching the collider there
def truth(s):return s['ground_truth_evaluation_only']
def add(name,ok,**metrics):
    row=dict(test=name,passed=bool(ok),**metrics);results.append(row);print(json.dumps(row),flush=True)
FLAG_TAGS={'-NoCaster':'-no-caster','-BumperSparseColumns':'-sparse-columns'}
def tag_of(scenario,extra):
    """Unique log/trace tag per launch flag: every flag maps to its own suffix; -MassScale=1.12 is '-payload', any other
    scale '-payload-<scale>', so a second payload run gets its own tag instead of raising."""
    tags=[]
    for f in extra:
        if f in FLAG_TAGS:tags.append(FLAG_TAGS[f])
        elif f.startswith('-MassScale='):tags.append('-payload' if f=='-MassScale=1.12' else '-payload-'+f.split('=',1)[1].replace('.','p'))
        else:tags.append('-'+''.join(c if c.isalnum() else '-' for c in f.lstrip('-')).lower())
    return scenario+''.join(tags)
def launch(scenario,extra=()):
    tag=tag_of(scenario,extra)
    if tag in TAGS_USED:raise RuntimeError(f'run tag {tag} used twice; logs and traces would overwrite each other')
    TAGS_USED.add(tag)
    log=open(REPORT/f'physics-{tag}.log','w')
    p=subprocess.Popen([str(EDITOR_CMD),str(ROOT/'HomeRobotSim.uproject'),'/Engine/Maps/Entry','-game','-nullrhi','-unattended','-nosound','-nosplash',f'-Scenario={scenario}','-stdout',*extra],stdout=log,stderr=subprocess.STDOUT)
    deadline=time.monotonic()+150
    while time.monotonic()<deadline:
        if p.poll() is not None:raise RuntimeError(f'Unreal exited {p.returncode}; inspect {log.name}')
        if 'ROBOT READY' in pathlib.Path(log.name).read_text(errors='ignore'):return p,log
        time.sleep(.3)
    p.terminate();raise TimeoutError('Unreal startup timed out')
def stop(p,r,log):
    r.send(quit=True);r.sock.close()
    try:p.wait(timeout=10)
    except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=10)
    log.close()
    # Per-tick bumper query cost, logged by the pawn (wall-clock microseconds on this machine).
    lines=[l for l in pathlib.Path(log.name).read_text(errors='ignore').splitlines() if 'SENSOR_COST' in l]
    if lines:sensor_cost[pathlib.Path(log.name).stem.removeprefix('physics-')]={k:float(v) for k,v in (x.split('=') for x in lines[-1].split('SENSOR_COST',1)[1].split())}
def speed(s):return math.hypot(*truth(s)['velocity_m_s'][:2])
def x_of(s):return truth(s)['position_m'][0]
def world_x(s,px,py):
    """World x (m) of the robot-frame point (px, py) m."""
    t=truth(s);return t['position_m'][0]+px*math.cos(t['yaw_rad'])-py*math.sin(t['yaw_rad'])
def body_to_world(s,x,y,z):
    """Robot-frame point (cm; y left, z above the floor at rest) -> world (m; test frame x, y left, z) from ground truth.
    The body turns about its origin, BASE_H_M above base_link; truth pitch/roll are Unreal's (pitch + = nose UP, roll + =
    right side down), so this is Unreal's FRotationMatrix with y flipped."""
    t=truth(s);P,Q=math.radians(t['pitch_deg']),math.radians(t['roll_deg']);Y=-t['yaw_rad']
    SP,CP,SR,CR,SY,CY=math.sin(P),math.cos(P),math.sin(Q),math.cos(Q),math.sin(Y),math.cos(Y)
    lx,ly,lz=x/100,-y/100,z/100-BASE_H_M
    wx=lx*CP*CY+ly*(SR*SP*CY-CR*SY)-lz*(CR*SP*CY+SR*SY)
    wy=lx*CP*SY+ly*(SR*SP*SY+CR*CY)+lz*(CY*SR-CR*SP*SY)
    wz=lx*SP-ly*SR*CP+lz*CR*CP
    px,py,pz=t['position_m'];return px+wx,py-wy,pz+BASE_H_M+wz
# Raised surfaces per scenario (x0, x1, y0, y1, height m; test frame), as built in AHomeRobotGameMode::StartPlay.
FLOORS={'bumpbench':[(-2.03,-1.97,-2.95,-1.0,.008),(-.03,.03,-2.95,-1.0,.010),(2.0,2.6,-2.95,-1.0,.009)],
        'threshold':[(.77,.83,-.9,.9,.010)]}
def floor_z(scenario,x,y):
    return max((h for x0,x1,y0,y1,h in FLOORS.get(scenario,()) if x0<=x<=x1 and y0<=y<=y1),default=0.)
def shell_clearance(s,scenario):
    """Lowest gap (cm) between the body collider's bottom ring (shell_bottom_z_m) and the surface under it (truth pose)."""
    return min(100*(z-floor_z(scenario,x,y)) for x,y,z in (body_to_world(s,px,py,SHELL_BOTTOM) for px,py in SHELL_RING))
def nose_down(rows):return max(-truth(t)['pitch_deg'] for t in rows)
def peak_decel(rows):
    """Peak forward deceleration (m/s^2): from truth velocity between state messages, and from the IMU specific force."""
    f=[(t['sim_time_s'],truth(t)['velocity_m_s'][0]*math.cos(truth(t)['yaw_rad'])+truth(t)['velocity_m_s'][1]*math.sin(truth(t)['yaw_rad'])) for t in rows]
    return (max((-(b[1]-a[1])/(b[0]-a[0]) for a,b in zip(f,f[1:]) if b[0]>a[0]),default=0.),
            max(-t['sensors']['specific_force_m_s2'][0] for t in rows))
def stop_metrics(rows,scenario):
    """Pitch/deceleration/shell-floor metrics of a hard stop; ok when the front shell stays clear of the floor."""
    d,imu=peak_decel(rows);clear=min(shell_clearance(t,scenario) for t in rows);nd=nose_down(rows)
    return clear>.05 and nd<NOSE_DOWN_GROUND_DEG,dict(peak_nose_down_pitch_deg=nd,shell_grounding_pitch_deg=NOSE_DOWN_GROUND_DEG,
        min_shell_floor_clearance_cm=clear,peak_decel_m_s2=d,peak_imu_decel_m_s2=imu)
def push(r,v,w,seconds,arm=False):
    """Stream like the autonomy RobotIO: arm + motor ON on every packet until the drive reports armed."""
    rows=[];start=None;pending=arm
    while True:
        r.send(v,w,**({'arm':True,'motor_power_on':True} if pending else {}));s=r.receive();rows.append(s);pending=pending and not s['armed']
        if start is None:start=s['sim_time_s']
        if s['sim_time_s']-start>=seconds:return rows
def spin_until(r,w,angle,timeout,arm=True):
    """Command a pure spin (v=0) until |accumulated yaw| >= angle (rad) or timeout (sim s). Like a controller, it re-arms
    whenever the drive reports disarmed (a new bumper press disarms). Returns rows, turned (rad)."""
    rows=[];start=None;pending=arm;turned=0.;prev=None
    while True:
        r.send(0.,w,**({'arm':True,'motor_power_on':True} if pending else {}));s=r.receive();rows.append(s);pending=arm and not s['armed']
        y=truth(s)['yaw_rad'];turned+=0. if prev is None else wrap(y-prev);prev=y
        if start is None:start=s['sim_time_s']
        if abs(turned)>=angle or s['sim_time_s']-start>=timeout:return rows,turned
def drift_of(rows,origin):
    return max(math.dist(origin,truth(t)['position_m'][:2]) for t in rows)
def bump_backoff(r,s):
    """Bumper-only fault: forward/fast-turn re-arm refused, armed reverse backs off, normal drive after release."""
    x0=x_of(s);b=s['sensors'];e0=b['bumper_events']
    add('bump reported and held',b['bumper'] and e0==1 and b['last_fault_event']=='bumper' and s['fault']=='bumper' and not s['armed'],bumper_events=e0,last_fault_event=b['last_fault_event'],bumper_left=b['bumper_left'],bumper_right=b['bumper_right'])
    # (0, .7): spin faster than bump_spin .5; (-.05, .6): |w| > 2|v|/track = .44 -> both refused while pressed.
    fwd=push(r,.10,0,1,arm=True)+push(r,0,.7,.5,arm=True)+push(r,-.05,.6,.5,arm=True);ahead=max(x_of(t) for t in fwd)-x0
    add('bumped: forward and fast-turn re-arm refused',not any(t['armed'] for t in fwd) and all(t['sensors']['bumper'] for t in fwd) and ahead<.002,max_forward_m=ahead,fault=fwd[-1]['fault'])
    # Armed for reverse while still pressed, then a forward and a pivot command arrive before the bumper clears: DriveCore
    # holds BOTH wheel setpoints at zero (no one-wheel pivot), so no forward motion and no heading change.
    armed=push(r,-.05,0,.05,arm=True);xa=x_of(armed[-1]);ya=truth(armed[-1])['yaw_rad']
    held=push(r,.10,0,.5)+push(r,.05,.6,.5)
    fwd_m=max(x_of(t) for t in held)-xa;dyaw=math.degrees(max(abs(wrap(truth(t)['yaw_rad']-ya)) for t in held))
    add('bumped and armed for reverse: forward and pivot commands hold both wheels at zero (no forward motion, no yaw)',
        armed[-1]['armed'] and all(t['armed'] and t['sensors']['bumper'] for t in held) and fwd_m<.002 and dyaw<1,
        armed_for_reverse=armed[-1]['armed'],press_held_every_row=all(t['sensors']['bumper'] for t in held),max_forward_m=fwd_m,max_yaw_change_deg=dyaw)
    # Allowed while pressed: reverse with turn |w| <= 2|v|/track (0.4 <= 0.44 at v=-0.05): both wheels back, centre moves back.
    turn=push(r,-.05,.4,.5);xt=x_of(turn[-1]);dyt=math.degrees(wrap(truth(turn[-1])['yaw_rad']-truth(held[-1])['yaw_rad']))
    add('bumped: reverse with turn (v -0.05, w 0.4) accepted while pressed, centre moves back',all(t['armed'] for t in turn) and x_of(held[-1])-xt>=.003 and dyt>1.5
        and turn[-1]['sensors']['bumper_events']==e0,backoff_m=x_of(held[-1])-xt,yaw_change_deg=dyt,bumper_events=turn[-1]['sensors']['bumper_events'])
    fwd+=armed+held+turn
    back=push(r,-.05,0,1.6,arm=True);s=back[-1];x1=x_of(s);clear=next((t for t in back if not t['sensors']['bumper']),None);yaw=abs(wrap(truth(s)['yaw_rad']-truth(back[0])['yaw_rad']))
    add('bump back-off: armed reverse clears bumper',s['armed'] and x0-x1>=.05 and clear is not None and not s['sensors']['bumper'] and s['sensors']['bumper_events']==e0 and s['fault']=='none' and yaw<.1,backoff_m=x0-x1,cleared_after_m=x0-x_of(clear) if clear else None,yaw_change_rad=yaw,bumper_events=s['sensors']['bumper_events'],fault=s['fault'])
    again=push(r,0,0,.5)+push(r,.05,0,3);s=again[-1];x2=x_of(s)
    add('after release: forward resumes, re-contact trips again',max(x_of(t) for t in again)>x1+.04 and not s['armed'] and s['sensors']['bumper'] and s['sensors']['bumper_events']==e0+1 and abs(x2-x0)<.01 and speed(s)<.01,final_x_m=x2,bumper_events=s['sensors']['bumper_events'],fault=s['fault'])
    return fwd+back+again
# ---------------- Bumper skin acceptance bench (scenario 'bumpbench') ----------------
# Robot frame (cm): x forward, y LEFT, origin = base_link (shell centre = axle midpoint). Azimuth is measured about base_link,
# + = left. Skin radius R_S over +/-SPAN deg, zones from bump_zone_edges_deg (bits 0 right .. 4 left at +/-20/+/-55/+/-90),
# bumper_left = centre zone and left of it, bumper_right = centre zone and right of it. Bench layout:
# AHomeRobotGameMode::StartPlay 'bumpbench'.
BOXES={1.9:(3.0,1.0),2.5:(-1.5,1.0),3.5:(0.,1.0),4.5:(1.5,1.0)}  # 2 x 2 cm low boxes by height (cm): centre (m); 1.9 cm
                                                                  # is 1 mm above the shell bottom WITH A LEVEL BODY: only the lowest
                                                                  # probe row (0.8 mm margin; caster lift pitches the nose up ~0.38
                                                                  # deg per 0.5 mm, so low_boxes asserts that premise separately)
POLE=(-1.5,2.0)                                      # 2 cm diameter pole (m)
STEP_MM={'8 mm threshold strip':8,'10 mm threshold strip':10,'9 mm rug edge up':9,'9 mm rug edge down':9}
CROSSINGS=[  # name, first edge x (m), depth to the last edge (m), run-up before the first edge (m), start surface z (m), also in reverse
    ('8 mm threshold strip',-2.03,.06,.45,0.,True),('10 mm threshold strip',-.03,.06,.45,0.,True),('9 mm rug edge up',2.0,0.,.45,0.,False),('9 mm rug edge down',2.6,0.,.30,.009,False)]
def az_cases():
    """Straight approaches (v=.05) whose first contact is the box corner at the skin azimuth az: box centre
    (R_S cos az + 2.5, R_S sin az +/- 1) cm, 1.5 cm run-up. Expected switches and zone bit from the zone table."""
    out=[]
    for az in (0,30,-30,60,-60,75,-75):
        a=math.radians(az);left,right=switches(az)
        out.append((az,(R_S*math.cos(a)+2.5,R_S*math.sin(a)+(math.copysign(1,az) if az else 0.)),.05,0.,left,right,1<<zone_of(az)))
    return out
def skin_x(y):
    """Frontmost point (cm) of the round sensing skin at lateral offset y (cm)."""
    return math.sqrt(max(0.,R_S**2-y**2))
def clearance(point,radius):
    """Gap (cm) between a vertical pole (or a box's circumscribed circle) of `radius` at robot-frame point (cm) and the
    body collider (scoring only)."""
    return collider_gap(point)-radius
def robot_frame(s,point):
    """World point (m) -> robot frame (cm) from ground truth (scoring only)."""
    t=truth(s);c,n=math.cos(t['yaw_rad']),math.sin(t['yaw_rad']);dx,dy=point[0]-t['position_m'][0],point[1]-t['position_m'][1]
    return 100*(c*dx+n*dy),100*(-n*dx+c*dy)
def place_at(r,target,rel,yaw=0.,seconds=.6):
    """Place the robot so the world point `target` (m) is at robot-frame `rel` (cm) with heading yaw."""
    c,n=math.cos(yaw),math.sin(yaw);rx,ry=rel[0]/100,rel[1]/100
    return place(r,target[0]-(c*rx-n*ry),target[1]-(n*rx+c*ry),yaw,seconds)
def slim(rows):
    """Trace rows without the per-message geometry echo and LiDAR payload (bench rows are already compact)."""
    out=[]
    for t in rows:
        t={k:v for k,v in t.items() if k not in ('sim_geometry','preview')}
        if isinstance(t.get('sensors'),dict):t['sensors']={k:v for k,v in t['sensors'].items() if k!='lidar'}
        out.append(t)
    return out
def compact(rows):
    """Bench traces without the LiDAR payload."""
    keys=('bumper','bumper_left','bumper_right','bumper_events','bump_zones')
    return [dict(t=s['sim_time_s'],armed=s['armed'],fault=s['fault'],pose=truth(s)['position_m'][:2]+[truth(s)['yaw_rad']],pitch=truth(s)['pitch_deg'],roll=truth(s)['roll_deg'],**{k:s['sensors'][k] for k in keys}) for s in rows]
def place(r,x,y,yaw=0.,seconds=.6,z=0.):
    """Teleport (reset) the robot to a pose (m, rad) on a surface z m above the floor and let it settle."""
    r.send(reset=True,x_m=x,y_m=y,yaw_rad=yaw,**({'z_m':z} if z else {}));deadline=time.monotonic()+15;start=None
    while True:
        s=r.receive();r.send()
        if start is None:
            if s['sim_time_s']>.3:
                if time.monotonic()>deadline:raise TimeoutError('reset was not acknowledged')
                continue
            start=s['sim_time_s']
        if s['sim_time_s']-start>=seconds:return s
def drive_until_bump(r,v,w,seconds,e0,after=.4):
    """Arm and drive until bumper_events rises above e0 (then command zero for `after` s) or `seconds` pass."""
    rows=[];pending=True;start=None;trip=None
    while True:
        moving=trip is None
        r.send(v if moving else 0.,w if moving else 0.,**({'arm':True,'motor_power_on':True} if pending and moving else {}))
        s=r.receive();rows.append(s);pending=pending and not s['armed']
        if start is None:start=s['sim_time_s']
        if trip is None and s['sensors']['bumper_events']>e0:trip=len(rows)-1;stamp=s['sim_time_s']
        if trip is not None and s['sim_time_s']-stamp>=after:return rows,trip
        if trip is None and s['sim_time_s']-start>=seconds:return rows,None
def crossings(r):
    """Thresholds and rug edges at full speed, straight and at 45 deg. All edges are lines of constant x and travel is
    always toward +x. Every trial (forward and REVERSE) passes only when the WHOLE robot crossed (both drive wheels 3 cm and
    the rear ball caster 1.8 cm past the last edge, so a caster stalled at the edge fails; the run time leaves ~0.2 m of
    margin), there was no bump, the heading changed by < 5 deg (a 45 deg run must not slew square to the edge) and the
    shell bottom never came within 0.5 mm of the surface under it (shell_clearance, truth pose). The 8 mm rows are MARGINAL (static margin CASTER_STEP_LIMIT_MM - 8, about 0.6-0.9 mm of step, ~1.7 mm
    of skin clearance), so dynamic pitch overshoot as the caster drops can fail them; each row reports the static margin
    and the peak nose-down pitch overshoot over the static value. REVERSE (8/10 mm strips):
    the robot faces -x and backs over. Geometry: with the caster CASTER_M behind the axle and the shell bottom at
    SHELL_BOTTOM the front shell grounds once the caster stands CASTER_STEP_LIMIT_MM (8.9 mm for -7.5 cm / 1.8 cm) above
    the wheels' floor, so the 10 mm strips (both directions) and the 9 mm rug-down row are EXPECTED TO FAIL on shell
    clearance until the CAD moves the caster back (x <= about -9.5 cm) or raises the shell bottom (>= about 2.2 cm)."""
    rows_all=[]
    for name,x0,depth,lead,z,reverse in CROSSINGS:
        for direction in ('forward','reverse') if reverse else ('forward',):
            for deg in (0,45):
                a=math.radians(deg);y0=-2.0 if deg==0 else -2.3;sx,sy=x0-lead*math.cos(a),y0-lead*math.sin(a);last=x0+depth
                back=direction=='reverse';s0=place(r,sx,sy,a+(math.pi if back else 0.),z=z);e0=s0['sensors']['bumper_events'];yaw0=truth(s0)['yaw_rad']
                sp=MAX_REVERSE if back else .15;rows=push(r,-sp if back else sp,0,(lead+depth/math.cos(a)+.15)/sp+1.2,arm=True);rows_all+=rows;s=rows[-1]
                wheels=max(min(world_x(t,AXLE_M,TRACK_M/2),world_x(t,AXLE_M,-TRACK_M/2))-last for t in rows);caster=world_x(s,*CASTER_M)-last;bumps=s['sensors']['bumper_events']-e0
                touched=any(x['sensors']['bumper'] or x['sensors']['bump_zones'] for x in rows);crossed=wheels>=.03 and caster>=.018
                dyaw=math.degrees(abs(wrap(truth(s)['yaw_rad']-yaw0)));clear=min(shell_clearance(x,'bumpbench') for x in rows)
                add(f'{name} at {sp:.2f} m/s, {deg} deg{", reverse" if back else ""}: no bump, whole robot over, heading held, shell clear',
                    bumps==0 and not touched and crossed and dyaw<5 and clear>.05,
                    bump_events=bumps,zones_seen=touched,both_wheels_past_last_edge_m=wheels,caster_past_last_edge_m=caster,whole_robot_crossed=crossed,
                    start_yaw_deg=math.degrees(yaw0),final_yaw_deg=math.degrees(truth(s)['yaw_rad']),yaw_change_deg=dyaw,fault=s['fault'],armed=s['armed'],
                    min_shell_floor_clearance_cm=clear,peak_nose_down_pitch_deg=nose_down(rows),shell_grounding_pitch_deg=NOSE_DOWN_GROUND_DEG,
                    caster_step_limit_mm=CASTER_STEP_LIMIT_MM,step_mm=STEP_MM[name],static_step_margin_mm=CASTER_STEP_LIMIT_MM-STEP_MM[name],
                    static_nose_down_deg=static_nose_down_deg(STEP_MM[name]),nose_down_overshoot_deg=nose_down(rows)-static_nose_down_deg(STEP_MM[name]),
                    peak_pitch_deg=max(abs(truth(x)['pitch_deg']) for x in rows),peak_roll_deg=max(abs(truth(x)['roll_deg']) for x in rows))
    return rows_all
def box_gap(s,centre,half=.01):
    """Plan gap (cm) between a world axis-aligned square box (centre m, half-size m) and the body collider: + apart,
    <= 0 touching (box corners against COLLIDER, COLLIDER vertices against the square; scoring only)."""
    g=min(collider_gap(robot_frame(s,(centre[0]+sx*half,centre[1]+sy*half))) for sx in (-1,1) for sy in (-1,1))
    t=truth(s);c,n=math.cos(t['yaw_rad']),math.sin(t['yaw_rad']);px,py=t['position_m'][:2]
    for vx,vy in COLLIDER:
        dx=abs(px+(c*vx-n*vy)/100-centre[0])-half;dy=abs(py+(n*vx+c*vy)/100-centre[1])-half
        g=min(g,100*(math.hypot(max(dx,0.),max(dy,0.))+min(max(dx,dy),0.)))
    return g
def low_boxes(r):
    """2 x 2 cm boxes 1.9/2.5/3.5/4.5 cm tall met at az 0, +/-30, +/-60, +/-75 driving straight at 0.05 m/s. Each case
    needs the right switches and zone bit, no flag before the trip, and the trip no later than the first body contact
    (box outline within 0.5 mm of the collider, as pole_sweep), so a body that touches first and is deflected fails."""
    rows_all=[]
    for h,(bx,by) in BOXES.items():
        cases=[]
        premise=[]
        for az,(rx,ry),v,w,left,right,zones in az_cases():
            s0=place(r,bx-rx/100,by-ry/100);a=math.radians(az)
            probe_z=100*body_to_world(s0,R_S*math.cos(a),R_S*math.sin(a),BUMP_ROWS[0])[2]   # lowest probe row at the contact azimuth
            skin_z=100*body_to_world(s0,R_S*math.cos(a),R_S*math.sin(a),SHELL_BOTTOM)[2]    # skin / shell bottom edge there
            premise.append(dict(az=az,rest_pitch_deg=truth(s0)['pitch_deg'],lowest_probe_row_z_cm=probe_z,skin_bottom_z_cm=skin_z,ok=probe_z<h and skin_z<h))
            rows,trip=drive_until_bump(r,v,w,2.,s0['sensors']['bumper_events']);rows_all+=rows
            if trip is None:cases.append(dict(az=az,ok=False,tripped=False,premise_ok=premise[-1]['ok']));continue
            got=0
            for t in rows[trip:]:got|=t['sensors']['bump_zones']
            end=rows[-1]['sensors'];early=any(t['sensors']['bumper'] or t['sensors']['bump_zones'] for t in rows[:trip])
            gaps=[box_gap(t,(bx,by)) for t in rows];contact=next((i for i,g in enumerate(gaps) if g<=.05),None)
            ok=end['bumper'] and not early and (end['bumper_left'],end['bumper_right'],got)==(left,right,zones) and (contact is None or contact>=trip)
            cases.append(dict(az=az,ok=ok,left=end['bumper_left'],right=end['bumper_right'],zones=got,expected=[left,right,zones],events=end['bumper_events']-s0['sensors']['bumper_events'],
                              clearance_at_trip_cm=round(gaps[trip],3),contact_before_trip=contact is not None and contact<trip,premise_ok=premise[-1]['ok']))
        if h<2.:   # the 1.9 cm box exists to exercise ONLY the lowest probe row: check that premise on its own row first
            add(f'low box {h} cm premise: lowest probe row and skin bottom below the box top at rest (every azimuth)',all(p['ok'] for p in premise),
                box_top_cm=h,expected_rest_pitch_deg=REST_PITCH_DEG,rear_caster_z_m=PARAMS.get('rear_caster_z_m',0.),cases=premise)
        add(f'low box {h} cm at az 0/+-30/+-60/+-75: switches, zone bit, trip before body contact',all(c['ok'] for c in cases),cases=cases)
    return rows_all
def spin_clear(r):
    """Negative spin: the 4.5 cm box 1.5 cm outside the skin at az +/-85 (nearest point, any box orientation). A full 360 deg
    spin about base_link must not touch it or report a bump (the round robot can always turn in place)."""
    rows_all=[];bx,by=BOXES[4.5];d=R_S+1.5+math.sqrt(2)
    for az,w in ((85,.5),(-85,-.5)):
        a=math.radians(az);s0=place_at(r,(bx,by),(d*math.cos(a),d*math.sin(a)));e0=s0['sensors']['bumper_events'];p0=truth(s0)['position_m'][:2]
        rows,turned=spin_until(r,w,2*math.pi,25.);rows_all+=rows
        gap=min(clearance(robot_frame(t,(bx,by)),math.sqrt(2)) for t in rows);drift=drift_of(rows,p0)
        add(f'spin 360 deg next to a box 1.5 cm outside the skin at az {az:+d}: no bump, no contact',abs(turned)>=2*math.pi and rows[-1]['sensors']['bumper_events']==e0
            and not any(t['sensors']['bump_zones'] for t in rows) and gap>0 and drift<.01,turned_deg=math.degrees(turned),min_gap_cm=gap,max_base_link_drift_m=drift,
            bumper_events=rows[-1]['sensors']['bumper_events']-e0)
    return rows_all
def pressed_spin(r):
    """Pressed spin: the 2 cm pole held 2 mm inside the skin at az +62 (zone 4; 2.8 mm clear of the 64-gon bumper hard
    stop, so no physical contact), so the bumper is pressed at rest. A pure spin (0, +0.4 rad/s) re-arms while pressed and
    turns the robot TOWARD the contact: its azimuth must fall toward 0 deg and stay inside the sensed +/-span, the bumper
    must stay pressed on EVERY row (the nearest column is always <= 2 deg from the pole centre), the zones at the end must
    be those of the final contact azimuth (+/-3.5 deg, the pole's angular half-width), and the body must never touch the
    pole (gap > 0.5 mm, the pole_sweep contact definition; base_link drift is reported and must stay < 1 cm)."""
    rows_all=[];d=R_S-.2+1                                # pole surface 2 mm inside the skin, as a held press on the hardware
    for az,w in ((62,.4),(-62,-.4)):
        a=math.radians(az);s0=place_at(r,POLE,(d*math.cos(a),d*math.sin(a)));e0=s0['sensors']['bumper_events'];p0=truth(s0)['position_m'][:2]
        pressed0=s0['sensors']['bumper'] and not s0['armed']
        rows,turned=spin_until(r,w,math.radians(45),4.);rows_all+=rows
        azs=[math.degrees(math.atan2(*reversed(robot_frame(t,POLE)))) for t in rows]
        gaps=[clearance(robot_frame(t,POLE),1.) for t in rows];armed=[t for t in rows if t['armed']]
        rear=any(abs(x)>SPAN and g<=.05 for x,g in zip(azs,gaps))
        held=all(t['sensors']['bumper'] for t in rows);zones_end=rows[-1]['sensors']['bump_zones']
        allowed=0
        for x in (azs[-1]-3.5,azs[-1],azs[-1]+3.5):allowed|=1<<zone_of(x)
        drift=drift_of(rows,p0)
        ok=(pressed0 and len(armed)>=len(rows)-4 and held and bool(zones_end) and not zones_end&~allowed
            and abs(azs[-1])<abs(azs[0])-20 and max(abs(x) for x in azs)<=SPAN and not rear and min(gaps)>.05 and drift<.01)
        add(f'pressed spin at az {az:+d}: re-armed, press held, contact moves toward 0 deg, no body contact',ok,pressed_at_rest=pressed0,turned_deg=math.degrees(turned),
            contact_az_start_deg=azs[0],contact_az_end_deg=azs[-1],armed_rows=len(armed),rows=len(rows),min_gap_cm=min(gaps),rear_shell_contact=rear,
            max_base_link_drift_m=drift,press_held_every_row=held,new_bump_events=rows[-1]['sensors']['bumper_events']-e0,zones_end=zones_end,
            allowed_zones_end=allowed,expected_zone_end=1<<zone_of(azs[-1]))
    # Spinning AWAY from a pressed contact would carry it past +/-span onto the unsensed rear shell (the skin is only ~1.5 mm
    # outside it; a pressed obstacle sits inside the rigid radius). DriveCore allows a pressed spin only toward the pressed
    # side, so the arm is refused on every packet and the heading does not change.
    for az,w in ((62,-.4),(-62,.4)):
        a=math.radians(az);s0=place_at(r,POLE,(d*math.cos(a),d*math.sin(a)));y0=truth(s0)['yaw_rad'];e0=s0['sensors']['bumper_events']
        rows,turned=spin_until(r,w,math.radians(45),2.);rows_all+=rows
        gaps=[clearance(robot_frame(t,POLE),1.) for t in rows];dyaw=math.degrees(max(abs(wrap(truth(t)['yaw_rad']-y0)) for t in rows))
        add(f'pressed spin AWAY from the contact at az {az:+d} (w {w:+.1f}): refused, no turn toward the unsensed rear shell',
            s0['sensors']['bumper'] and not any(t['armed'] for t in rows) and all(t['sensors']['bumper'] for t in rows) and dyaw<1 and min(gaps)>.05,
            pressed_at_rest=s0['sensors']['bumper'],armed_rows=sum(t['armed'] for t in rows),rows=len(rows),max_yaw_change_deg=dyaw,min_gap_cm=min(gaps),
            new_bump_events=rows[-1]['sensors']['bumper_events']-e0,fault=rows[-1]['fault'])
    return rows_all
def held_press(r):
    """2.5 cm box (above the 1.8 cm shell bottom: the shell and bumper stop the robot) met at full speed. The hard stop
    must not pitch the front shell onto the floor (no front caster): nose-down pitch below the grounding angle and the
    shell bottom > 0.5 mm clear; peak deceleration (truth and IMU) is reported."""
    bx,by=BOXES[2.5];s0=place(r,bx-(R_S/100+.01+.10),by);e0=s0['sensors']['bumper_events']
    rows,trip=drive_until_bump(r,.15,0,3.,e0,after=.6);s=rows[-1];x0=x_of(s);clear_ok,m=stop_metrics(rows,'bumpbench')
    add('held press: 2.5 cm box at 0.15 m/s trips and stays pressed after the stop, front shell clear of the floor',trip is not None and s['sensors']['bumper'] and s['sensors']['bumper_events']==e0+1 and not s['armed'] and speed(s)<.01 and clear_ok,
        peak_speed_m_s=max(speed(t) for t in rows),box_face_x_after_stop_cm=robot_frame(s,(bx,by))[0]-1,zones=s['sensors']['bump_zones'],bumper_events=s['sensors']['bumper_events']-e0,**m)
    fwd=push(r,.10,0,1,arm=True)+push(r,0,.7,.5,arm=True);ahead=max(x_of(t) for t in fwd)-x0
    add('held press: forward and fast-turn re-arm refused',not any(t['armed'] for t in fwd) and all(t['sensors']['bumper'] for t in fwd) and ahead<.002,max_forward_m=ahead,fault=fwd[-1]['fault'])
    back=push(r,-.05,0,1.6,arm=True);s=back[-1];clear=next((t for t in back if not t['sensors']['bumper']),None)
    add('held press: reverse accepted and clears the bumper',s['armed'] and x0-x_of(s)>=.05 and clear is not None and not s['sensors']['bumper'] and s['sensors']['bumper_events']==e0+1 and s['fault']=='none',
        backoff_m=x0-x_of(s),cleared_after_m=x0-x_of(clear) if clear else None,fault=s['fault'])
    return rows+fwd+back
def pole_sweep(r,offsets):
    """2 cm pole met head-on at 0.05 m/s at lateral offsets (cm, + = left). A trial passes when the bumper trips no later
    than the first physical contact with the body collider and before any deflection (yaw < 0.3 deg); a pole that never
    reaches the collider must not trip either."""
    rows_all=[];trials=[]
    for yc in offsets:
        px=math.sqrt(max(0.,(R_S+2)**2-yc**2))                 # pole centre >= 1 cm clear of the skin in every direction
        s0=place(r,POLE[0]-px/100,POLE[1]-yc/100);yaw0=truth(s0)['yaw_rad']
        rows,trip=drive_until_bump(r,.05,0,2.,s0['sensors']['bumper_events'],after=.2);rows_all+=rows
        gaps=[clearance(robot_frame(t,POLE),1.) for t in rows];contact=next((i for i,g in enumerate(gaps) if g<=.05),None)
        at=rows[trip if trip is not None else -1];yaw=math.degrees(abs(wrap(truth(at)['yaw_rad']-yaw0)))
        ok=(trip is None and contact is None) or (trip is not None and (contact is None or contact>=trip) and yaw<.3)
        trials.append(dict(y_cm=yc,ok=ok,tripped=trip is not None,clearance_at_trip_cm=round(gaps[trip],3) if trip is not None else None,
                           contact_before_trip=contact is not None and (trip is None or contact<trip),yaw_deg=round(yaw,3),left=at['sensors']['bumper_left'],right=at['sensors']['bumper_right'],zones=at['sensors']['bump_zones']))
    return rows_all,trials
def pole_test(r):
    """Dense sweep in 5 mm steps plus the sparse control's between-column offsets (MID_Y), so the -BumperSparseColumns run
    differs from this one only by the flag."""
    offsets=sorted(set([round(-16.25+.5*i,2) for i in range(66)]+MID_Y))
    rows,trials=pole_sweep(r,offsets);bad=[t['y_cm'] for t in trials if not t['ok']]
    mid_ok=all(t['ok'] for t in trials if t['y_cm'] in MID_Y)
    add('2 cm pole swept across the round skin in 5 mm steps (+ sparse-control offsets): no gaps',not bad,trials=len(trials),gaps_y_cm=bad,sparse_control_offsets_ok=mid_ok,
        min_clearance_at_trip_cm=min((t['clearance_at_trip_cm'] for t in trials if t['tripped']),default=None),detail=trials)
    return rows
def reverse_cliff(r):
    """Backing toward a floor edge: the rear cliff sensors must stop the robot while the loaded rear caster still has floor
    under it (caster centre >= 5 mm before the edge), with no backward tip. Max reverse speed and the bump retreat speed."""
    rows_all=[];lead=100*(CASTER_M[0]-min(c[0] for c in CLIFF_M if c[0]<0))   # rear sensor ahead of the caster in reverse (cm)
    for v in (-MAX_REVERSE,-.05):   # the firmware's reverse cap and the bump retreat speed
        s0=place(r,.35,0.,math.pi);rows=push(r,v,0,(.80-.35)/abs(v)+2.5,arm=True);rows_all+=rows;s=rows[-1]
        caster=max(world_x(t,*CASTER_M) for t in rows);sensor=max(max(world_x(t,*c) for c in CLIFF_M if c[0]<0) for t in rows)
        pitch=max(abs(truth(t)['pitch_deg']) for t in rows);dz=min(truth(t)['position_m'][2] for t in rows)-truth(s0)['position_m'][2]
        add(f'reverse floor edge at {abs(v):.2f} m/s: rear cliff stop, caster stays on the floor',not s['armed'] and s['fault']=='cliff_or_invalid' and speed(s)<.01
            and caster<=.80-.005 and pitch<3 and dz>-.005,caster_max_x_m=caster,edge_x_m=.80,rear_sensor_max_x_m=sensor,peak_pitch_deg=pitch,min_dz_m=dz,final_x_m=x_of(s),fault=s['fault'],
            sensor_to_caster_lead_cm=lead,note='moving the caster rearward needs the rear cliff sensors moved back by the same amount')
    return rows_all
def edge_lead_cm(heading):
    """Geometric lead (cm) of the first cliff sensor over the first wheel contact and over the caster, travelling +x with
    the robot heading `heading` (rad, test frame)."""
    c,n=math.cos(heading),math.sin(heading);wx=lambda p:p[0]*c-p[1]*n
    xs=max(wx(p) for p in CLIFF_M);xw=max(wx((AXLE_M,TRACK_M/2)),wx((AXLE_M,-TRACK_M/2)));return 100*(xs-xw),100*(xs-wx(CASTER_M))
def angled_cliff(r):
    """Floor edge (x = 0.80) met at 30/45/60/75 deg, forward and in REVERSE, at 0.15 m/s: the cliff sensors must stop the
    robot with both drive-wheel contacts and the caster still >= 5 mm on the floor and no tip. The sensors' lead over the
    wheels shrinks with the approach angle (10.5 cos a - 4.2 sin a cm for the front pair with the wheels on the axle), so
    a failure here is a CAD cliff-sensor placement issue, not a simulator issue. Then a 360 deg spin with base_link
    12.5 cm from the edge (wheel outer edge ~0.6 cm from it): it may complete or be stopped by a cliff sensor, but no wheel
    or caster may leave the floor."""
    rows_all=[];EDGE=.80;HALF=TRACK_M/2+.005                # tyre outer edge (collision hull +/-0.5 cm)
    for back in (False,True):
        for deg in (30,45,60,75):
            a=math.radians(deg);heading=a+(math.pi if back else 0.);lead_w,lead_c=edge_lead_cm(heading)
            s0=place(r,EDGE-R_S/100-.10,-1.0,heading);z0=truth(s0)['position_m'][2]
            sp=MAX_REVERSE if back else .15;rows=push(r,-sp if back else sp,0,(.15/math.cos(a))/sp+2.5,arm=True);rows_all+=rows;s=rows[-1]
            wheels=max(max(world_x(t,AXLE_M,TRACK_M/2),world_x(t,AXLE_M,-TRACK_M/2)) for t in rows)
            outer=max(max(world_x(t,AXLE_M,HALF),world_x(t,AXLE_M,-HALF)) for t in rows);caster=max(world_x(t,*CASTER_M) for t in rows)
            pitch=max(abs(truth(t)['pitch_deg']) for t in rows);roll=max(abs(truth(t)['roll_deg']) for t in rows);dz=min(truth(t)['position_m'][2] for t in rows)-z0
            add(f'floor edge at {deg} deg{", reverse" if back else ""} ({sp:.2f} m/s): cliff stop, both wheels and the caster on the floor',
                not s['armed'] and s['fault']=='cliff_or_invalid' and speed(s)<.01 and wheels<=EDGE-.005 and caster<=EDGE-.005 and pitch<3 and roll<3 and dz>-.005,
                wheel_contact_max_x_m=wheels,wheel_outer_edge_max_x_m=outer,caster_max_x_m=caster,edge_x_m=EDGE,peak_pitch_deg=pitch,peak_roll_deg=roll,min_dz_m=dz,
                sensor_lead_over_wheels_cm=lead_w,sensor_lead_over_caster_cm=lead_c,fault=s['fault'],
                note='a failure is a CAD cliff-sensor placement issue (sensor lead over the wheels vs the coasting stop distance)')
    for w in (.5,-.5):
        s0=place(r,EDGE-.125,-1.0,math.pi/2);z0=truth(s0)['position_m'][2]
        rows,turned=spin_until(r,w,2*math.pi,16.);rows_all+=rows
        outer=max(max(world_x(t,AXLE_M,HALF),world_x(t,AXLE_M,-HALF)) for t in rows);caster=max(world_x(t,*CASTER_M) for t in rows)
        pitch=max(abs(truth(t)['pitch_deg']) for t in rows);roll=max(abs(truth(t)['roll_deg']) for t in rows);dz=min(truth(t)['position_m'][2] for t in rows)-z0
        add(f'spin {"+" if w>0 else "-"}360 deg with base_link 12.5 cm from a floor edge: completes or cliff-stops, wheels and caster stay on the floor',
            outer<=EDGE and caster<=EDGE-.005 and pitch<3 and roll<3 and dz>-.005,completed=abs(turned)>=2*math.pi,turned_deg=math.degrees(turned),
            cliff_stopped=any(t['fault']=='cliff_or_invalid' for t in rows),wheel_outer_edge_max_x_m=outer,caster_max_x_m=caster,peak_pitch_deg=pitch,peak_roll_deg=roll,min_dz_m=dz)
    return rows_all
def check_geometry(s):
    """The pawn echoes the geometry it used (sim_geometry); it must equal robot-parameters.json, with no fallback and no
    PRE-SYNC marker, and its lowest probe row must sit just above the shell bottom."""
    g=s.get('sim_geometry') or {};bad=[]
    for k in ('drive_axle_x_m','body_radius_m','rigid_radius_m','bumper_travel_m','wheel_track_m','wheel_radius_m','bumper_span_deg','cliff_z_m',
              'rear_caster_z_m','caster_ball_radius_m','shell_bottom_z_m','body_top_z_m','bumper_top_z_m','base_link_height_m'):
        if k in PARAMS and abs(g.get(k,1e9)-PARAMS[k])>1e-6:bad.append(k)
    for k in ('rear_caster_xy_m','camera_mount_m','lidar_mount_m','lidar_turret_xy_m','lidar_turret_z_m'):
        if k in PARAMS and (len(g.get(k,[]))!=len(PARAMS[k]) or any(abs(a-b)>1e-6 for a,b in zip(g.get(k,[]),PARAMS[k]))):bad.append(k)
    if 'lidar_turret_radius_m' in PARAMS and abs(g.get('lidar_turret_radius_m',1e9)-PARAMS['lidar_turret_radius_m'])>1e-6:bad.append('lidar_turret_radius_m')
    # LiDAR column collider (bay B10): turret cylinder from the lid top to the robot top, never above it.
    col=g.get('lidar_column_z_m') or [1e9,1e9]
    if not g.get('lidar_column') or abs(col[0]-PARAMS.get('body_top_z_m',1e9))>1e-6 or abs(col[1]-PARAMS.get('robot_top_z_m',1e9))>1e-6:bad.append('lidar_column_z_m')
    if 'body_inertia_kg_m2' in PARAMS and not g.get('body_inertia_from_file'):bad.append('body_inertia_kg_m2')
    ge=g.get('bump_zone_edges_deg',[])
    if len(ge)!=len(EDGES) or any(abs(a-b)>1e-6 for a,b in zip(ge,EDGES)):bad.append('bump_zone_edges_deg')
    gc,pc=g.get('cliff_sensors_m',[]),PARAMS.get('cliff_sensors_m',CLIFF_M)
    if len(gc)!=len(pc) or any(len(p)!=len(q) or any(abs(a-b)>1e-6 for a,b in zip(p,q)) for p,q in zip(gc,pc)):bad.append('cliff_sensors_m')
    rows=g.get('bump_rows_cm') or [1e9]
    if not SHELL_BOTTOM<rows[0]<=SHELL_BOTTOM+.05 or len(rows)!=len(BUMP_ROWS) or any(abs(a-b)>1e-6 for a,b in zip(rows,BUMP_ROWS)) or rows[-1]>BUMPER_TOP-.2+1e-6:
        bad.append('bump_rows_cm')
    src=g.get('source') or 'missing';low=src.lower()
    add('pawn geometry read from robot-parameters.json (no fallback, synced)',not bad and src.startswith('robot-parameters.json') and 'fallback' not in low and 'pre-sync' not in low
        and g.get('bump_columns')==COLUMN_COUNT,mismatched=bad,source=src,bump_columns=g.get('bump_columns'),expected_columns=COLUMN_COUNT,bump_rows_cm=g.get('bump_rows_cm'),expected_bump_rows_cm=BUMP_ROWS)
def run(scenario,extra=()):
    # Physics checks of the simulator itself: exact sensors (the B11 sensor error model is for the robot, not for these).
    p,log=launch(scenario,extra);r=RobotClient(sensor_errors=False);rows=[]
    try:
        # 'ROBOT READY' is logged in BeginPlay; engine init can take another ~3 s before the first tick, which raced the
        # client's 2 s receive timeout (and the cleanup then sent 'quit'). Wait for the first state message first.
        r.sock.settimeout(30);r.receive();r.sock.settimeout(2.)
        s=r.settle(2)
        if '-NoCaster' in extra:
            rows=r.hold(0,0,3)
            up=max(truth(x)['pitch_deg'] for x in rows)        # Unreal pitch: + = nose UP (tips BACK onto the rear rim)
            add('negative control: remove rear caster (COM behind the axle tips the body BACK onto the rear shell rim)',up>.7*NOSE_UP_GROUND_DEG,
                peak_nose_up_pitch_deg=up,expected_rim_contact_pitch_deg=NOSE_UP_GROUND_DEG,threshold_deg=.7*NOSE_UP_GROUND_DEG)
        elif scenario=='empty':
            check_geometry(s)
            com=truth(s)['com_base_m'];want=[x/1000 for x in PARAMS['estimated_body_com_mm']];err=math.dist(com,want)
            add('body COM matches robot-parameters within 0.5 mm',err<.0005,com_base_m=com,expected_m=want,error_mm=err*1000,com_behind_axle_m=AXLE_M-com[0])
            # Nothing welded into the body adds mass (the LiDAR column is a convex element of the body itself).
            mb=truth(s).get('physics_body_mass_kg')
            add('physics body mass equals body_mass_kg (no welded extra mass)',mb is not None and abs(mb-PARAMS['body_mass_kg'])<.003,
                physics_body_mass_kg=mb,expected_kg=PARAMS['body_mass_kg'],physics_robot_mass_kg=truth(s).get('physics_robot_mass_kg'),expected_robot_kg=PARAMS.get('total_mass_kg'))
            # Whole robot (welded body + wheels + caster ball, mass-weighted by the pawn): behind the axle and equal to robot_com_mm.
            rc=truth(s).get('robot_com_base_m');want_r=[x/1000 for x in PARAMS.get('robot_com_mm',[])];have=rc is not None and len(want_r)==3
            add('whole-robot COM at least 15 mm behind the axle and within 1 mm of robot_com_mm',have and rc[0]-AXLE_M<=-.015 and math.dist(rc,want_r)<.001,
                robot_com_base_m=rc,expected_m=want_r,error_mm=math.dist(rc,want_r)*1000 if have else None,robot_com_behind_axle_m=AXLE_M-rc[0] if rc else None)
            # Body inertia from body_inertia_kg_m2 (bay B10): the full tensor read back from the physics state (robot frame).
            bi=truth(s).get('physics_body_inertia_kg_m2');want_i=PARAMS.get('body_inertia_kg_m2')
            if want_i:
                mean=sum(want_i[k][k] for k in range(3))/3
                ierr=max(abs(bi[i][j]-want_i[i][j]) for i in range(3) for j in range(3))/mean if bi else None
                add('physics body inertia equals body_inertia_kg_m2 within 1 % of the mean moment',ierr is not None and ierr<.01,
                    physics_body_inertia_kg_m2=bi,expected_kg_m2=want_i,max_error_fraction_of_mean=ierr)
            add('resting stability',abs(truth(s)['pitch_deg'])<2 and abs(truth(s)['roll_deg'])<2 and speed(s)<.005,pitch_deg=truth(s)['pitch_deg'],roll_deg=truth(s)['roll_deg'],speed_m_s=speed(s))
            start=truth(s)['position_m'];rows=r.hold(.10,0,5,arm=True);s=rows[-1]
            d=math.dist(start[:2],truth(s)['position_m'][:2]);err=math.hypot(r.odom.x-truth(s)['position_m'][0],r.odom.y-truth(s)['position_m'][1])
            add('straight drive and encoder odometry',.40<d<.58 and err<.04 and all(x['armed'] for x in rows[2:]),distance_m=d,odometry_error_m=err,peak_pitch_deg=max(abs(truth(x)['pitch_deg']) for x in rows))
            rows+=r.hold(0,0,2);add('commanded stop',speed(rows[-1])<.01,final_speed_m_s=speed(rows[-1]))
            s=r.settle();p0=truth(s)['position_m'][:2];turn=r.hold(0,.5,3,arm=True);rows+=turn
            yaw=wrap(truth(turn[-1])['yaw_rad']);add('turn on the spot',1.1<yaw<1.9 and drift_of(turn,p0)<.01,yaw_rad=yaw,max_base_link_drift_m=drift_of(turn,p0))
            s=r.settle();p0=truth(s)['position_m'][:2];spin,turned=spin_until(r,.5,2*math.pi,25.);rows+=spin
            add('turn on the spot: 360 deg about base_link, drift < 1 cm',abs(turned)>=2*math.pi and drift_of(spin,p0)<.01,turned_deg=math.degrees(turned),
                max_base_link_drift_m=drift_of(spin,p0),seconds=spin[-1]['sim_time_s']-spin[0]['sim_time_s'],peak_pitch_deg=max(abs(truth(x)['pitch_deg']) for x in spin))
            r.settle();drive=r.hold(.10,0,2,arm=True);last=drive[-1]['sim_time_s'];watch=[]
            while True:
                s=r.receive();watch.append(s)
                if s['sim_time_s']-last>1.2:break
            rows+=drive+watch
            add('lost-command watchdog',not s['armed'] and s['fault']=='watchdog' and speed(s)<.01,final_fault=s['fault'],speed_m_s=speed(s))
            s=r.hold(.10,0,.5)[-1];add('watchdog requires explicit re-arm',not s['armed'])
            r.settle();r.hold(.10,0,2,arm=True);r.send(motor_power_off=True);s=r.hold(.10,0,1)[-1]
            add('motor OFF switch latch',not s['armed'] and s['fault']=='motor_power_off',final_speed_m_s=speed(s))
            r.send(motor_power_on=True);s=r.hold(.10,0,.5)[-1];add('motor ON requires separate arm',not s['armed'],last_fault_event=s['sensors']['last_fault_event'])
            add('no bumper events on open floor',not any(t['sensors']['bumper'] for t in rows) and s['sensors']['bumper_events']==0,bumper_events=s['sensors']['bumper_events'])
        elif scenario=='bumpbench' and '-BumperSparseColumns' in extra:
            g=s.get('sim_geometry') or {};flag=g.get('sparse_columns') is True and g.get('bump_columns')==len(SPARSE_AZ)
            rows,trials=pole_sweep(r,MID_Y);rows=compact(rows);missed=[t for t in trials if t['contact_before_trip']]
            add('negative control: sparse 16 deg columns miss a 2 cm pole between columns (contact before trip)',flag and bool(missed),
                sparse_columns_echo=g.get('sparse_columns'),bump_columns=g.get('bump_columns'),expected_columns=len(SPARSE_AZ),
                contact_azimuths_deg=MIDS,contact_before_trip_y_cm=[t['y_cm'] for t in missed],detail=trials)
        elif scenario=='bumpbench':
            rows=compact(crossings(r)+low_boxes(r)+spin_clear(r)+pressed_spin(r)+held_press(r)+pole_test(r))
        else:
            rows=r.hold(.10,0,14,arm=True);s=rows[-1];x=truth(s)['position_m'][0]
            if scenario=='obstacle':
                lo,hi=.70-R_S/100-.12,.70-R_S/100+.012   # obstacle face at x=.70 m
                clear_ok,m=stop_metrics(rows,'obstacle')
                add('front bumper stop (front skin clear of the floor)',not s['armed'] and lo<x<hi and speed(s)<.01 and clear_ok,final_x_m=x,bounds_m=[lo,hi],fault=s['fault'],speed_m_s=speed(s),**m)
                rows+=bump_backoff(r,s)
            if scenario=='cliff':
                wheel_edge=x+AXLE_M+WHEEL_R_M;pitch=max(abs(truth(t)['pitch_deg']) for t in rows)
                add('floor edge stop',not s['armed'] and .60<x<.748 and wheel_edge<.80 and pitch<3 and speed(s)<.01,final_x_m=x,front_wheel_edge_m=wheel_edge,
                    front_cliff_sensor_x_m=x+max(c[0] for c in CLIFF_M),peak_pitch_deg=pitch,fault=s['fault'])
                rev=push(r,-.05,0,1,arm=True);rows+=rev;s=rev[-1]
                add('floor edge stays a hard stop: reverse arm refused',not any(t['armed'] for t in rev) and s['fault']=='cliff_or_invalid' and abs(x_of(s)-x)<.003,final_x_m=x_of(s),fault=s['fault'])
                rows+=reverse_cliff(r)+angled_cliff(r)
            if scenario=='threshold':
                clear=min(shell_clearance(t,'threshold') for t in rows);dyaw=math.degrees(abs(wrap(truth(s)['yaw_rad']-truth(rows[0])['yaw_rad'])))
                caster=world_x(s,*CASTER_M)
                add('10 mm threshold with 12% heavier body' if extra else '10 mm threshold crossing',x>1.0 and caster>.83+.018 and s['armed'] and s['sensors']['bumper_events']==0 and clear>.05 and dyaw<5,
                    final_x_m=x,caster_x_m=caster,fault=s['fault'],bumper_events=s['sensors']['bumper_events'],min_shell_floor_clearance_cm=clear,yaw_change_deg=dyaw,
                    peak_nose_down_pitch_deg=nose_down(rows),caster_step_limit_mm=CASTER_STEP_LIMIT_MM,static_step_margin_mm=CASTER_STEP_LIMIT_MM-10,
                    peak_pitch_deg=max(abs(truth(t)['pitch_deg']) for t in rows))
        traces[tag_of(scenario,extra)]=slim(rows)
    finally:stop(p,r,log)
if __name__=='__main__':
    if 'sync_status' in PARAMS:raise SystemExit(f"robot-parameters.json is not synced from the CAD: {PARAMS['sync_status']}")
    probe=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    try:probe.bind(('127.0.0.1',8767))
    except OSError:raise SystemExit('Close the graphical robot simulation before running physics tests.')
    finally:probe.close()
    if '--stress-only' in sys.argv:
        prev=json.loads((REPORT/'physics-results.json').read_text())
        # Never merge results of another robot (the box robot, or a run before the round port) into all_passed.
        if 'round robot' not in str((prev.get('configuration') or {}).get('chassis','')) or not (prev.get('source_sha256') or {}).get('Source/HomeRobotSim/RobotGeometry.h'):
            raise SystemExit('reports/physics-results.json is not from the round-robot simulator; run the full suite first (no --stress-only).')
        results=prev['results']
        traces=json.loads((REPORT/'physics-traces.json').read_text())
    try:
        if '--stress-only' not in sys.argv:
            for scenario in SCENARIOS:run(scenario)
            for scenario,extra in EXTRA_RUNS[:-1]:run(scenario,extra)
        run(*EXTRA_RUNS[-1])
    finally:
        old=json.loads((REPORT/'physics-results.json').read_text()) if (REPORT/'physics-results.json').exists() else {}
        # Configuration is rebuilt from this robot's parameters (no box-robot or IR leftovers carried over).
        config={'total_nominal_mass_kg':PARAMS.get('total_mass_kg'),'body_mass_kg':PARAMS.get('body_mass_kg'),'robot_com_mm':PARAMS.get('robot_com_mm'),
                'wheel_axle_inertia_kg_m2':.0005,'max_torque_nm':PARAMS.get('max_drive_torque_nm',.1),'encoder_rate_frame':'wheel relative to chassis',
                'bumper_reach_ahead_of_origin_m':R_S/100,'step_height_m':.010,'sensor_model':'ideal rays; noise and delays unmodeled',
                'shell_grounding_pitch_deg':{'nose_down_front_skin':NOSE_DOWN_GROUND_DEG,'nose_up_rear_no_caster':NOSE_UP_GROUND_DEG,'rest_nose_up_from_caster_lift':REST_PITCH_DEG},
                'caster_step_limit_mm':CASTER_STEP_LIMIT_MM,
                'chassis':f'round robot, base_link = shell centre = drive-axle midpoint (drive_axle_x_m {AXLE_M}); body collider: convex 64-gon prism circumscribing the bumper hard stop r={R_F:.2f} cm (skin - bumper_travel) over +/-{SPAN:g} deg and the rigid rear shell r={R_B:.2f} cm, '
                          f'z {SHELL_BOTTOM:.1f}-{100*PARAMS.get("body_top_z_m",.14):.1f} cm; wheels r={WHEEL_R:.1f} cm at (0, +/-{50*TRACK_M:.1f}) cm; '
                          f'ONE rear ball caster at ({100*CASTER_M[0]:.1f}, {100*CASTER_M[1]:.1f}) cm, no front caster; LiDAR column: a 32-gon prism circumscribing the turret cylinder '
                          f'r={100*PARAMS.get("lidar_turret_radius_m",0):.2f} cm at {PARAMS.get("lidar_turret_xy_m")} m from the lid top {100*PARAMS.get("body_top_z_m",0):.2f} cm to the robot top {100*PARAMS.get("robot_top_z_m",0):.2f} cm, a second convex element of the body (nothing welded); '
                          'COM nudge = (desired - zero-nudge COM) x physics mass / body_mass_kg; body inertia = body_inertia_kg_m2 (principal moments and axes set after the nudge)',
                'bumper':f'round sensing skin r={R_S:.2f} cm about base_link over +/-{SPAN:g} deg; {COLUMN_COUNT} probe columns every {COLUMN_STEP:g} deg x {len(BUMP_ROWS)} rows, the lowest 0.2 mm above the shell bottom '
                         f'(z {SHELL_BOTTOM+.02:.2f} cm) then {BUMP_ROWS[1]:.1f}-{BUMP_ROWS[-1]:.1f} cm (2 mm under the bumper top {BUMPER_TOP:.1f} cm; the collider above it to the body top is conservative); rays 1.5 cm inside to 0.2 cm outside the skin, |n_z|>0.7 hits ignored; rigid rear shell unsensed; '
                         f'zones {EDGES} deg (bits 0..{len(EDGES)-2}, right to left), bumper_left = centre zone and left, bumper_right = centre zone and right, bump_zones latched per state message; '
                         'bumper-only fault re-arms for reverse (v<=0, |w|<=2|v|/track, both wheels <= 0) or a pure spin (|v|<1e-4, |w|<=0.5 rad/s) toward the pressed side only (left switch only: w>=0, right only: w<=0, both: either); disallowed pressed commands hold both wheels at zero; cliff/tilt stay hard stops',
                'negative_controls':'-NoCaster (tips back onto the rear shell rim), -BumperSparseColumns (every 4th column, 16 deg: a 2 cm pole between columns touches before the trip)'}
        tags={tag_of(x,()) for x in SCENARIOS}|{tag_of(x,e) for x,e in EXTRA_RUNS}
        costs={k:{m:v for m,v in c.items() if not m.startswith('ir_')} for k,c in {**old.get('sensor_cost_us',{}),**sensor_cost}.items() if k in tags}
        sha={p:(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() if (ROOT/p).exists() else None) for p in SOURCES}
        (REPORT/'physics-results.json').write_text(json.dumps({'engine':'Unreal Engine 5.8.1 / Chaos','results':results,'all_passed':all(x['passed'] for x in results),'configuration':config,
            'sensor_cost_us':costs,'source_sha256':sha},indent=2))
        (REPORT/'physics-traces.json').write_text(json.dumps(traces,separators=(',',':')))
