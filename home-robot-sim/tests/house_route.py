"""Known-waypoint experiment through three rooms; encoders only for control.
This does not implement obstacle avoidance or autonomous exploration.
"""
import json,sys,time,math,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'controller'))
from robot_client import RobotClient,wrap
r=RobotClient();rows=[];goals=[(1.85,0),(1.95,-1.20),(2.10,-1.20),(1.95,0),(.20,0)];completed=0;failure=None
try:
    r.settle();r.hold(0,0,1,capture=True);r.hold(0,0,.2,arm=True,capture=True)
    deadline=time.monotonic()+260
    for gx,gy in goals:
        while time.monotonic()<deadline:
            s=r.receive();o=r.odom;rows.append({'state':s,'wheel_odom':[o.x,o.y,o.yaw]})
            if not s['armed']:raise RuntimeError(s['fault'])
            d=math.hypot(gx-o.x,gy-o.y)
            if d<.045:
                r.send(capture=True);completed+=1;print('Reached waypoint',completed,flush=True);break
            e=wrap(math.atan2(gy-o.y,gx-o.x)-o.yaw)
            r.send(min(.10,d*.65) if abs(e)<.35 else 0,max(-.45,min(.45,1.4*e)),capture=True)
        else:raise TimeoutError('House route exceeded 260 seconds')
    r.hold(0,0,2,capture=True)
except Exception as exc:
    failure=str(exc);print('Route stopped:',failure,flush=True)
finally:
    r.send(capture=False);r.close()
    (ROOT/'reports/house-route-telemetry.json').write_text(json.dumps(rows,separators=(',',':')))
    report=dict(scene='house',floor='identical plain light 60 cm tiles, 2 mm grout; provisional dimensions',scene_sha256=hashlib.sha256((ROOT/'Content/RobotData/house-scene.json').read_bytes()).hexdigest(),goals_m=goals,controller_waypoints_completed=completed,controller_finished=completed==len(goals),failure=failure,control='wheel encoders only; predefined waypoints, no obstacle avoidance',rows=len(rows))
    if rows:
        report['last_wheel_odom']=rows[-1]['wheel_odom'];report['duration_sim_s']=rows[-1]['state']['sim_time_s'];report['last_evaluation_only']=rows[-1]['state']['ground_truth_evaluation_only']
    # Simulator truth is used only after the controller has stopped, for scoring.
    hits=[];i=0
    for row in rows:
        if i>=len(goals):break
        gx,gy=goals[i];px,py,_=row['wheel_odom']
        if math.hypot(gx-px,gy-py)<.045:
            gt=row['state']['ground_truth_evaluation_only']['position_m']
            hits.append(dict(waypoint=i+1,estimated_position_m=[px,py],evaluation_only_position_m=gt,evaluation_only_error_m=math.hypot(gx-gt[0],gy-gt[1])));i+=1
    report['waypoint_evaluation']=hits;report['physical_waypoint_tolerance_m']=.15
    report['physical_route_passed']=len(hits)==len(goals) and all(h['evaluation_only_error_m']<.15 for h in hits)
    if hits:report['final_goal_error_m']=hits[-1]['evaluation_only_error_m']
    report['status']='route passed' if report['physical_route_passed'] else 'physical route failed tolerance or did not finish'
    (ROOT/'reports/house-route.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
