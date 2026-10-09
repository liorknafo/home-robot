"""Follow a small square using encoder odometry, with no ground-truth feedback."""
import argparse,json,math,time
from pathlib import Path
from robot_client import RobotClient,wrap
p=argparse.ArgumentParser();p.add_argument('--side',type=float,default=.6);p.add_argument('--capture',action='store_true');args=p.parse_args()
r=RobotClient();log=[]
try:
    r.settle();r.hold(0,0,.2,arm=True,capture=args.capture)
    goals=[(args.side,0),(args.side,args.side),(0,args.side),(0,0)]
    started=time.monotonic()
    for gx,gy in goals:
        while time.monotonic()-started<180:
            s=r.receive();o=r.odom;d=math.hypot(gx-o.x,gy-o.y)
            log.append({'state':s,'wheel_odom':{'x':o.x,'y':o.y,'yaw':o.yaw}})
            if not s['armed']:raise RuntimeError('Robot disarmed: '+s['fault'])
            if d<.04:r.send();break
            err=wrap(math.atan2(gy-o.y,gx-o.x)-o.yaw)
            v=min(.10,d*.8)*max(0,math.cos(err)) if abs(err)<.6 else 0
            w=max(-.7,min(.7,2*err));r.send(v,w,capture=args.capture)
        else:raise TimeoutError('Route exceeded 180 seconds')
    r.hold(0,0,2)
    print('Encoder-odometry route finished. Position estimate:',r.odom)
finally:
    r.send(capture=False);r.close();out=Path(__file__).resolve().parents[1]/'reports/drive-demo.json';out.write_text(json.dumps(log,indent=2));print('Saved',out)
