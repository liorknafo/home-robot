"""Stationary finish capture. Start launch.command house polished (or matte) first.
Stop the autonomous service before using this fixture. Captures then quits Unreal.
Usage: python3 tests/capture_finish.py polished
"""
import sys,json,shutil,time
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'controller'))
from robot_client import RobotClient
c=RobotClient();c.settle();c.send(capture=True,camera_hz=5,motor_power_off=True)
rows=c.hold(0,0,4,capture=True);c.send(capture=False,motor_power_off=True)
folder=R/'Saved/Stereo'/json.loads((R/'Saved/Stereo/latest-session.json').read_text())['directory'];m=json.loads((folder/'latest.json').read_text());dest=R/'reports/surfaces';dest.mkdir(exist_ok=True)
for side in ['left','right']:shutil.copyfile(folder/m[side],dest/(sys.argv[1]+'-'+side+'.png'))
(dest/(sys.argv[1]+'.json')).write_text(json.dumps(dict(capture=m,state=rows[-1],recording=folder.name),indent=2))
print(sys.argv[1],folder.name,m['frame']);c.send(quit=True);c.close()
