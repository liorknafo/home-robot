"""Record a short encoder-controlled square for independent stereo processing."""
import json,sys,time,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'controller'))
from robot_client import RobotClient,wrap
r=RobotClient();rows=[]
try:
    r.settle();r.hold(0,0,1,capture=True);r.hold(0,0,.2,arm=True,capture=True)
    goals=[(.4,0),(.4,.4),(0,.4),(0,0)]
    deadline=time.monotonic()+150
    for gx,gy in goals:
        while time.monotonic()<deadline:
            s=r.receive();o=r.odom;rows.append({'state':s,'wheel_odom':[o.x,o.y,o.yaw]})
            if not s['armed']:raise RuntimeError(s['fault'])
            d=math.hypot(gx-o.x,gy-o.y)
            if d<.035:r.send(capture=True);break
            e=wrap(math.atan2(gy-o.y,gx-o.x)-o.yaw);r.send(min(.08,d*.7) if abs(e)<.45 else 0,max(-.5,min(.5,1.5*e)),capture=True)
        else:raise TimeoutError('Square route exceeded 150 seconds')
    r.hold(0,0,2,capture=True);r.send(capture=False)
    print('Square complete',r.odom,flush=True)
finally:
    r.send(capture=False);r.close();(ROOT/'reports/camera-route.json').write_text(json.dumps(rows,separators=(',',':')))
