"""Small closed-loop sim2d scenarios for the ROUND robot controller (copy of ctrl-tests/scenarios.py).
Usage: scenarios_round.py [name ...]   (default: all mapping scenarios, then the kitchen-corner and side-press variants)
Each scenario runs in its own fresh Python process (several in parallel), so its numbers do not
depend on which scenarios ran before it; output is printed in the order given. JOBS=n sets how many run at once (default 6).
The statistical checks (STAT, rulings PB29 / PB32) run as 12 such processes each; one of their runs alone:
scenarios_round.py overhang_sofa@s3:on. Wall time at JOBS=6 (2026-10-11): the whole suite 467 s, the two statistical
checks alone ~4 min.
The statistical runs are deterministic per seed (identical output across invocations). catch_rug_edge's jam -> stall-mark path
(a stall mark at the rug edge) is exercised today by errors-on seeds 3 and 5 and errors-off seeds 0, 1, 2, 3 and 5; the other
runs drive off the catch without a jam. A code change that moves every seed of one error setting off that path turns the check
red on purpose (ruling PB32): keep the path covered (other seeds, or a scenario that reaches it) rather than drop the rule."""
import os,subprocess,sys,tempfile,numpy as np
from pathlib import Path
P=str(Path(__file__).resolve().parents[1])   # project root
sys.path.insert(0,P+'/controller');sys.path.insert(0,P+'/tests')
from autonomy.lidar_brain import LidarBrain,STALL_MARK_SPACING
from sim2d import World,Sim,Bridge,box,inside,poly_segments,seg_point_distance,run_kitchen_corner,run_side_press,run_overhang,reverse_audit,run_contact_recovery
walls=[((-1,-1.5),(3,-1.5)),((3,-1.5),(3,1.5)),((3,1.5),(-1,1.5)),((-1,1.5),(-1,-1.5)),((1,-1.5),(1,.6)),((-.6,.9),(-.4,.9)),((2.3,-.9),(2.3,-.6)),((.5,-1.1),(.5,-.8))]
SC={'tray':dict(low=[box(.45,0.,.5,.5)]),'tray_far':dict(low=[box(.75,-.9,.5,.4)]),'bath':dict(low=[box(2.3,1.1,.8,.4)]),'curb':dict(curbs=[box(.5,.6,.06,.8)]),'empty':{},'sticky':dict(sticky=[(box(.4,0,.3,.3),'turn'),(box(1.8,0,.3,.3),'stop')]),
    'clutter':dict(low=[box(1.8,-.3,.25,.12),box(-.3,.9,.06,.06),box(2.4,.8,.4,.3)],curbs=[box(.3,-.9,.3,.04)])}
# NO BLIND REVERSING (user rule): every scenario fails on any reverse that leaves the TRUE driven trail
# (sim2d.reverse_audit: base_link positions under v < 0, or moving backward, vs. positions reached otherwise;
# tolerance 2.5 cm + the run's measured estimate-to-truth error, within_2_5cm reported against the bare 2.5 cm).
# Firmware: sim2d firmware='spec' (default: pure spin toward the pressed side allowed) or 'drivecore' (the
# repo's DriveCore.h: wheels clamped to <= 0 while pressed, so a pressed 'spin' pivots base_link backward).
# The controller defaults to the DriveCore.h rule (PRESSED_SPIN_PURE False: no spin while pressed); the
# *_pure variants set pressed_spin_pure=True on the brain with the spec firmware.
# Kitchen corner: nose-in, skin `gap` from two walls meeting at 90 deg, everything to explore behind, no trail.
# Pass = turned around and left (> .25 m) by the deadline, 0 body contacts, 0 bumper touches, and NO reverse
# at all before leaving (every gap here is >= 3 mm: the robot turns in place, slowly inside TURN_CLEAR).
#   kitchen_corner(_side): the specified 1 cm gap (close spin: averaged points inside TURN_CLEAR), 25 s: a close spin
#           is at most .25 rad/s, so turning away from the side layout (up to ~185 deg) alone takes ~13 s; the 20 s of
#           the earlier controller relied on a blind reverse arc followed by a .4 rad/s turn.
#   *_3mm/_5mm: below TURN_MARGIN, 30 s (close spin; the front points are verified by the released skin).
#   *_slip: 1 cm of base_link drift per full turn in place (random direction), 25 s (close spin, as above): exercises the drift share of TURN_MARGIN.
#   *_15mm: the geometry of unit test (b) (walls R + 1.5 cm from base_link, 5 mm outside TURN_CLEAR) with the
#           4 mm-sigma LiDAR noise: must turn in place.
#   *_20mm: 1 cm outside TURN_CLEAR (beyond the 4 mm-sigma LiDAR noise after averaging) with 1 cm/turn spin slip: must turn in place.
#   *_drivecore: the 3/5/10/15 mm corners on the DriveCore.h firmware model: no reverse command and no backward
#           base_link motion at all within the escape window (a pressed spin there would pivot backward); same pass
#           rule and deadline as the spec-firmware variant of that gap (30 / 30 / 25 / 20 s).
#   *_3mm_slip_s<k> / *_5mm_slip_s<k> (seeds 0-5): SLIP STRESS below the drift budget. 1 cm/turn slip moves base_link up to
#           5 mm per half turn, more than a 3 or 5 mm gap (+ the 1.5 mm skin trip reach), so a close spin can press the skin;
#           pressed against a wall with no driven path behind, the only legal state is standing still (the firmware allows
#           only a spin toward the press, a reverse would be blind). Pass = escaped as above, OR safe: 0 rigid-body
#           contacts, no reverse command at all, clean reverse audit, session alive (waiting for release). The summary
#           line counts how many escaped.
KC={'kitchen_corner':dict(diagonal=True,deadline=25.),'kitchen_corner_side':dict(diagonal=False,deadline=25.),
    'kitchen_corner_3mm':dict(diagonal=True,gap=.003,deadline=30.),'kitchen_corner_side_3mm':dict(diagonal=False,gap=.003,deadline=30.),
    'kitchen_corner_5mm':dict(diagonal=True,gap=.005,deadline=30.),'kitchen_corner_side_5mm':dict(diagonal=False,gap=.005,deadline=30.),
    'kitchen_corner_slip':dict(diagonal=True,spin_slip_m_per_turn=.01,deadline=25.),'kitchen_corner_side_slip':dict(diagonal=False,spin_slip_m_per_turn=.01,deadline=25.),
    'kitchen_corner_12mm_slip':dict(diagonal=True,gap=.012,spin_slip_m_per_turn=.01,deadline=25.),'kitchen_corner_side_12mm_slip':dict(diagonal=False,gap=.012,spin_slip_m_per_turn=.01,deadline=25.),
    'kitchen_corner_15mm':dict(diagonal=True,gap=.015,require_no_reverse=True),'kitchen_corner_side_15mm':dict(diagonal=False,gap=.015,require_no_reverse=True),
    'kitchen_corner_15mm_s1':dict(diagonal=True,gap=.015,require_no_reverse=True,seed=1),'kitchen_corner_side_15mm_s2':dict(diagonal=False,gap=.015,require_no_reverse=True,seed=2),
    'kitchen_corner_20mm':dict(diagonal=True,gap=.02,spin_slip_m_per_turn=.01,require_no_reverse=True),'kitchen_corner_side_20mm':dict(diagonal=False,gap=.02,spin_slip_m_per_turn=.01,require_no_reverse=True)}
STRESS={f'kitchen_corner{side}_{mm}mm_slip_s{k}':dict(diagonal=not side,gap=mm/1000,spin_slip_m_per_turn=.01,deadline=30.,seed=k,safe_wait_ok=True)
        for mm in (3,5) for side in ('','_side') for k in range(6)}
KC.update(STRESS)
KC.update({f'kitchen_corner{side}_{mm}mm_drivecore':dict(diagonal=not side,gap=mm/1000,deadline={3:30.,5:30.,10:25.,15:20.}[mm],firmware='drivecore',require_no_reverse=True)
           for mm in (3,5,10,15) for side in ('','_side')})
KC.update({f'kitchen_corner{side}_3mm_slip_s{k}_drivecore':dict(diagonal=not side,gap=.003,spin_slip_m_per_turn=.01,deadline=30.,seed=k,safe_wait_ok=True,firmware='drivecore')
           for side in ('','_side') for k in range(3)})
STRESS.update({k:v for k,v in KC.items() if k.endswith('_drivecore') and '_slip_' in k})
# Pressed contact at the side with no driven trail, NEVER reverse (the trail is empty, so any reverse would be
# blind), and no backward base_link motion at all (sim2d.backward_tick). The skin stays pressed (contact .5 mm
# outside it), so the robot must end standing still, session alive, waiting for the bumper to release
# (sim2d.run_side_press 'waiting_safely').
#   default brain (pressed_spin_pure=True, DriveCore.h since the round robot): spin the contact toward 0 deg.
#   *_pure: the same with pressed_spin_pure set explicitly.
#   *_drivecore: the PREVIOUS clamping firmware model with a brain configured for it (pressed_spin_pure=False):
#     no spin while pressed, base_link does not move (< 1 mm), contact az unchanged (88/60/-60/30 deg posts).
#   *_mismatch: the previous clamping firmware with the default brain: the brain's first pressed spin turns into a
#     backward pivot; check_pressed_pivot must catch it within 2 mm and the robot then waits safely.
# side_wall_*: the contact is a tall wall, not a 4 cm post (after the spin the nose faces it, still pressed).
SP={'side_press_left':(70.,False),'side_press_right':(-70.,False),'side_wall_left':(70.,True),'side_wall_right':(-70.,True)}
SP.update({k+'_pure':v+('spec',True) for k,v in list(SP.items())})
SP.update({f'side_press_{a}_drivecore':(float(a),False,'drivecore',False) for a in (88,60,-60,30)})
SP.update({'side_wall_left_drivecore':(70.,True,'drivecore',False)})
SP.update({f'side_press_{a}_mismatch':(float(a),False,'drivecore',None) for a in (88,60,-60)})
# side_press_give_up: side_press_left run past CONTACT_WAIT_S (90 s): the post never moves, so the robot must end the
# session standing still with the 'Stuck against an obstacle' message (task G: a bounded wait, not the whole time budget).
GU={'side_press_give_up':dict(az_deg=70.,seconds=90.,give_up=True)}
# Bumper contact with no straight driven trail behind (task G, sim2d.run_contact_recovery): a scripted approach, then a low
# step (6 cm, bumper only) `gap` ahead of the skin, driven into at 10 cm/s; from the touch on the controller decides.
#   contact_weave: the Unreal trial full-mapping-20261011T054731's last 4 s before the shower tray (+/-.4 rad/s commits);
#     the planned straight back-off stopped fitting after one pose update and the robot then waited 1110 s, pressed.
#   contact_after_spin_<gap>: 30 cm straight, a quarter turn in place, then 3 / 5 / 10 / 20 mm to the step: too little trail
#     room for the 2 cm back-off, so the robot waited for ever (3 mm: the touch comes after 1.6 mm of driving, under the
#     first version's 2 mm creep minimum).
#   contact_spin_start_<gap>: the quarter turn at the session start (the trail behind is only the start position).
# PASS: released within 15 s of the touch, off the step (>= .3 m from the touch) and mapping on, 0 body blocks, the
# reverse audit clean, the contact on the map.
CR={'contact_weave':dict(approach='weave',gap=.02),'contact_after_spin_3mm':dict(approach='spin',gap=.003),'contact_after_spin_5mm':dict(approach='spin',gap=.005),'contact_after_spin_10mm':dict(approach='spin',gap=.01),
    'contact_after_spin_20mm':dict(approach='spin',gap=.02),'contact_spin_start_10mm':dict(approach='spin_start',gap=.01),'contact_spin_start_30mm':dict(approach='spin_start',gap=.03)}
# Overhang in the LiDAR's blind band (low-profile R1: LiDAR plane 147.5 mm, robot top 155 mm): furniture on legs with its
# underside at 150 mm gives no LiDAR return, no bumper touch and no cliff; only the LiDAR turret meets its edge (sim2d
# overhang_world / run_overhang). overhang_sofa: mapping beside a 2 x .85 m sofa against a wall; overhang_table: mapping
# around a 1 x .6 m coffee table; overhang_table_goto: the same room mapped, then Go-to targets straight across the
# table (the shortest route passes between its legs, under it), each must end 'Arrived'. PASS: see run_overhang
# (no blind reverse, not stuck, every hit spot recorded as an obstacle and never hit again, at most 3 hits per spot).
OH={'overhang_sofa':dict(kind='sofa'),'overhang_table':dict(kind='table'),'overhang_table_goto':dict(kind='table',goals=((1.3,-.9),(1.3,.9)))}
# Caster caught on a rug edge (sim2d sticky kind 'catch': forward motion that would carry the rear caster onto the strip
# is blocked at any speed, the wheels keep turning, nothing is sensed): the general jam handling (jam_settle, measured
# retreat, straight retry, then mark_stall), not only overhangs. Mapping in the room above, with:
#   catch_transient: a 3 x 80 cm strip on the way to the doorway that lets go 3 s after it first held: the straight
#     retry gets through. PASS: complete, at least one jam, NO stall mark, clean reverse audit, 0 body blocks/bumps.
#   catch_rug_edge: a permanent 3 x 80 cm rug edge in the right room; a statistical check (see STAT below). PASS per run
#     (ruling PB32): complete, the caster really caught on the edge (sim truth: at least one catch episode; a run that never
#     touched it fails as 'scenario not exercised'), clean reverse audit, 0 body blocks/bumps, free area within
#     CATCH_FREE_BOUND (.25 m2) of the same room without the strip ('empty', run in the process); a run that jammed has at
#     least one stall mark, no two marks closer than the robot's own STALL_MARK_SPACING, and at most CATCH_MARK_CAP marks
#     within CATCH_NEAR_M of the edge (a stall loop: the jam loop of B11 errors-on seed 16 made 23 trouble spots). A catch the
#     robot drove off on its own (no jam developed) needs no mark.
#   (A permanent strip ACROSS the doorway approach, box(.5,.6,.03,.8), is not passable as a test: the mark the stall
#   puts ahead of the body plus the 25 cm planning clearance closes the gap around the strip end, and mapping stops
#   with the right room unvisited; the original controller stops there too, after 23 catches instead of 4.)
CT={'catch_transient':dict(sticky=[(box(.5,.6,.03,.8),'catch',3.)]),'catch_rug_edge':dict(sticky=[(box(1.9,.3,.03,.8),'catch')])}
CATCH_FREE_BOUND=.25
CATCH_NEAR_M=.30    # m: 'near the edge' (note_jam's spot radius)
CATCH_MARK_CAP=3    # stall marks near the edge per run; the 12 PB29 runs (seeds 0-5, errors on / off) make at most 2
def run_catch(name,seed=0,sensor_errors=None):
    def mapping(world):
        tmp=tempfile.TemporaryDirectory();b=LidarBrain(tmp.name);sim=Sim(world,seed=seed,sensor_errors=sensor_errors);br=Bridge(b,sim,rearm_hook=lambda brain:getattr(brain,'wants_arm',False))
        for _ in range(10):b.predict(sim.state());[sim.step(.01) for _ in range(5)]
        st=dict(n=0,inside=False,episodes=0)
        def tick(x):
            hit=sim.catch_ticks>st['n'];st['n']=sim.catch_ticks;st['episodes']+=hit and not st['inside'];st['inside']=hit
        br.start_mapping();trace=br.run(800,on_tick=tick);return b,sim,trace,st['episodes']
    b,sim,trace,episodes=mapping(World(walls,**CT[name]));audit=reverse_audit(trace);marks=[np.asarray(m,float) for m in b.stall_marks]
    free=round(b.grid.summary()['free_area_m2'],2);close=min((float(np.linalg.norm(a-c)) for i,a in enumerate(marks) for c in marks[i+1:]),default=None)
    strip=CT[name]['sticky'][0][0];edge=np.array(poly_segments(strip))
    near=sum(inside(strip,m) or seg_point_distance(edge,m).min()<CATCH_NEAR_M for m in marks)
    r=dict(t=round(sim.t),complete=bool(b.complete),free=free,catch_episodes=episodes,catch_ticks=sim.catch_ticks,jams=len(b.jams),stall_marks=[np.round(m,2).tolist() for m in marks],
           marked=bool(near),marks_near_edge=int(near),closest_marks_m=None if close is None else round(close,3),contacts=len(b.contacts),skips=len(b.blacklist),body_blocks=sim.contacts,
           bumps=sim.bump_events,reverse_audit=audit,message=b.message)
    if episodes<1:r['passed']=False;r['verdict']='scenario not exercised: the caster never caught on the strip';return r
    ok=r['complete'] and audit['ok'] and sim.contacts==0 and sim.bump_events==0
    if name=='catch_transient':ok=ok and len(b.jams)>=1 and not marks
    else:
        ref=mapping(World(walls))[0];r['empty_free']=round(ref.grid.summary()['free_area_m2'],2)
        ok=(ok and free>=r['empty_free']-CATCH_FREE_BOUND and (not b.jams or len(marks)>=1) and (close is None or close>=STALL_MARK_SPACING)
            and near<=CATCH_MARK_CAP)
        r['verdict']='jammed and marked' if marks else 'jammed, no mark' if b.jams else 'caught, drove off on its own (no jam)'
    r['passed']=bool(ok);return r
# Statistical checks (rulings PB29, PB32). overhang_sofa and catch_rug_edge hinge on chance details of a single run: the last
# turret hit a few seconds before the session ends (too late to be marked), or a caster catch that the robot's arc carries off
# before it becomes a jam. Each runs over Sim seeds STAT_SEEDS (LiDAR noise, slip and the IMU noise sequence; the board is the
# configured one), with the sensor errors on and off (sensor_errors=True / False, whatever R1_SENSOR_ERRORS says), each run with
# its own PASS rule. The check fails if errors-on passes fewer seeds than errors-off minus 1, if a run crashes, or if a
# STAT_COVERAGE path was exercised by no run of one error setting. Run names: '<check>@s<seed>:<on|off>'; their lines say pass /
# fail in lower case (not a suite verdict). Wall time, determinism and the seeds that carry catch_rug_edge's mark path
# (STAT_COVERAGE 'marked': a stall mark within CATCH_NEAR_M of the rug edge): see the module docstring.
STAT=('overhang_sofa','catch_rug_edge');STAT_SEEDS=range(6)
STAT_COVERAGE={'catch_rug_edge':'marked'}   # a run key that must be True in >= 1 run per error setting
def stat_runs(name):return [f'{name}@s{k}:{e}' for e in ('on','off') for k in STAT_SEEDS]
def stat_verdict(name,results):
    """results: run name -> (return code (0 pass, 1 fail, else crashed), stdout). Returns (ok, summary line)."""
    n={e:sum(results[f'{name}@s{k}:{e}'][0]==0 for k in STAT_SEEDS) for e in ('on','off')};crashed=[r for r,(c,_) in results.items() if c not in (0,1)]
    key=STAT_COVERAGE.get(name);covered={e:[k for k in STAT_SEEDS if f"'{key}': True" in results[f'{name}@s{k}:{e}'][1]] for e in ('on','off')} if key else {}
    ok=not crashed and n['on']>=n['off']-1 and all(covered.values())
    return ok,(f"{name} {'PASS' if ok else 'FAIL'} statistical (rulings PB29, PB32), Sim seeds {STAT_SEEDS[0]}-{STAT_SEEDS[-1]}: errors on passed {n['on']}/{len(STAT_SEEDS)}, "
               f"errors off {n['off']}/{len(STAT_SEEDS)}; fails only if on < off - 1"+(f"; '{key}' seeds: on {covered['on']}, off {covered['off']} (>= 1 each)" if key else '')
               +(f'; crashed: {crashed}' if crashed else ''))
names=sys.argv[1:] or list(SC)+list(KC)+list(SP)+list(GU)+list(CR)+list(OH)+list(CT)
runs=[r for n in names for r in (stat_runs(n) if n in STAT else [n])]
if len(runs)>1:
    # One fresh process per scenario run (module-level state cannot leak between scenarios).
    from concurrent.futures import ThreadPoolExecutor
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1')
    run=lambda n:subprocess.run([sys.executable,'-B',__file__,n],capture_output=True,text=True,env=env)
    ok=True
    escaped=[]
    with ThreadPoolExecutor(int(os.environ.get('JOBS',6))) as pool:
        done=zip(runs,pool.map(run,runs))
        for n in names:
            batch=[next(done) for _ in (stat_runs(n) if n in STAT else [n])]
            for m,r in batch:sys.stdout.write(r.stdout);sys.stderr.write(r.stderr[-2000:] if r.returncode not in (0,1) else '')
            if n in STAT:
                good,line=stat_verdict(n,{m:(r.returncode,r.stdout) for m,r in batch});ok&=good;print(line)
            else:ok&=batch[0][1].returncode==0
            sys.stdout.flush()
            if n in STRESS:escaped.append("'escaped': True" in batch[0][1].stdout)
    if escaped:print(f'slip stress (gap below the drift budget): escaped {sum(escaped)}/{len(escaped)}, the rest ended safe (see each line)',flush=True)
    sys.exit(0 if ok else 1)
ok=True
for name in runs:
    tmp=tempfile.TemporaryDirectory()
    if '@' in name:                                   # one run of a statistical check
        check,run_id=name.split('@');seed,errors=run_id.lstrip('s').split(':');seed=int(seed);errors={'on':True,'off':False}[errors]
        r=run_catch(check,seed,errors) if check in CT else run_overhang(lambda:LidarBrain(tmp.name),**OH[check],seed=seed,sensor_errors=errors)
        ok&=r['passed'];print(' ',name,'pass' if r['passed'] else 'fail',{k:v for k,v in r.items() if k!='passed'},flush=True);continue
    if name in SP:
        az,wall_,fw,pure=(SP[name]+('spec',None))[:4]
        r=run_side_press(lambda:LidarBrain(tmp.name),az,wall=wall_,firmware=fw,pressed_spin_pure=pure)
        if name.endswith('_mismatch'):r['passed']=bool(r['waiting_safely'] and r['moved_m']<=.002 and r['backward_ticks']==0 and r['body_contacts']==0 and not r['reversed'])
        ok&=r['passed']
        print(name,'PASS' if r['passed'] else 'FAIL',{k:v for k,v in r.items() if k!='events'},flush=True);continue
    if name in GU:
        r=run_side_press(lambda:LidarBrain(tmp.name),**GU[name]);ok&=r['passed']
        print(name,'PASS' if r['passed'] else 'FAIL',{k:v for k,v in r.items() if k!='events'},flush=True);continue
    if name in CR:
        r=run_contact_recovery(lambda:LidarBrain(tmp.name),**CR[name]);ok&=r['passed']
        print(name,'PASS' if r['passed'] else 'FAIL',{k:v for k,v in r.items() if k!='passed'},flush=True);continue
    if name in CT:
        r=run_catch(name);ok&=r['passed']
        print(name,'PASS' if r['passed'] else 'FAIL',{k:v for k,v in r.items() if k!='passed'},flush=True);continue
    if name in OH:
        r=run_overhang(lambda:LidarBrain(tmp.name),**OH[name]);ok&=r['passed']
        print(name,'PASS' if r['passed'] else 'FAIL',r,flush=True);continue
    if name in KC:
        r=run_kitchen_corner(lambda:LidarBrain(tmp.name),**{'gap':.01,**KC[name]});ok&=r['passed']
        print(name,'PASS' if r['passed'] else 'FAIL',r,flush=True);continue
    b=LidarBrain(tmp.name);sim=Sim(World(walls,**SC[name]))
    br=Bridge(b,sim,rearm_hook=lambda brain:getattr(brain,'wants_arm',False))
    for _ in range(10):b.predict(sim.state());[sim.step(.01) for _ in range(5)]
    br.start_mapping();trace=br.run(800)
    err=max(np.linalg.norm(r['est'][:2]-r['truth'][:2]) for r in trace)
    audit=reverse_audit(trace)
    print(name,'t',round(sim.t),'complete',b.complete,'free',round(b.grid.summary()['free_area_m2'],2),'contacts',len(b.contacts),'skips',len(b.blacklist),'jams',len(b.jams),'body-blocks',sim.contacts,'bumps',sim.bump_events,'max err',round(err,3),'reverse',audit,'|',b.message,flush=True)
    ok&=bool(b.complete) and sim.contacts==0 and audit['ok']
sys.exit(0 if ok else 1)
