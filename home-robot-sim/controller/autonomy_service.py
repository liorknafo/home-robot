"""Local robot modes and dashboard. Commands use measured LiDAR/IMU/encoders (or legacy stereo)."""
import argparse,json,socket,threading,time,queue,cv2,os,re,ipaddress,subprocess
from pathlib import Path
from collections import deque
from http.server import ThreadingHTTPServer,SimpleHTTPRequestHandler
from urllib.parse import urlparse,urlsplit,unquote
from autonomy.brain import Brain
from autonomy.lidar_brain import LidarBrain
# Simulator only: MPU-6050 / wheel-calibration errors on the raw Unreal sensors (Content/RobotData/sensor-errors.json).
from sim_sensor_model import SensorErrorModel
ROOT=Path(__file__).resolve().parents[1]
# Task N (LIO-134): the camera LED's duty while a task drives (bridge_payload). R1_CAMERA_LED=0 turns it off (simulator A/B runs only).
CAMERA_LED_DUTY=float(os.environ.get('R1_CAMERA_LED','1'))
def bridge_payload(v,w,arm,active):
    """The command sent to the simulator every 50 ms. Camera LED (task N, LIO-134): on while the robot drives a task (mapping
    or Go-to: capture active), off when idle. Fully on, no PWM dimming: a rolling-shutter camera would show PWM as bands."""
    payload=dict(v_m_s=v,w_rad_s=w,arm=arm,capture=active,camera_hz=5,motor_power_off=not active,camera_led=CAMERA_LED_DUTY if active else 0.)
    if arm:payload['motor_power_on']=True
    return payload
def command_freshness(decision_age,sensor_age):
    """Simulator bridge: stop motion immediately, disarm on sustained loss."""
    stop_motion=decision_age>.6 or sensor_age>.6
    reason='controller did not produce a fresh decision for 2 seconds' if decision_age>2 else 'simulator sensor updates stopped for 3 seconds' if sensor_age>3 else 'none'
    return stop_motion,reason!='none',reason
class RobotIO(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True);self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.sock.bind(('127.0.0.1',8768));self.sock.settimeout(.01);self.lock=threading.Lock();self.state=None;self.running=True;self.desired=(0.,0.,False);self.decision_time=0.;self.arm_pending=False;self.received_at=0.;self.next_send=0.;self.interlock_reason='none';self.last_interlock_reason='none';self.samples=deque(maxlen=512);self.sensor_model=SensorErrorModel()
    def activate(self):
        with self.lock:self.arm_pending=True
    def set(self,v,w,active):
        with self.lock:
            self.desired=(v,w,active);self.decision_time=time.monotonic()
            if not active:self.arm_pending=False
    def run(self):
        while self.running:
            now=time.monotonic()
            if now>=self.next_send:
                with self.lock:
                    v,w,active=self.desired
                    self.interlock_reason='none'
                    stop_motion,disarm,reason=command_freshness(now-self.decision_time,now-self.received_at)
                    if stop_motion:v=w=0.
                    if active and disarm:active=False;self.interlock_reason=reason
                    if self.interlock_reason!='none':self.last_interlock_reason=self.interlock_reason
                    arm=self.arm_pending and active
                payload=bridge_payload(v,w,arm,active)
                self.sock.sendto(json.dumps(payload).encode(),('127.0.0.1',8767));self.next_send=now+.05
            try:
                raw=json.loads(self.sock.recv(65535))
                # Simulator truth is intentionally discarded at the process boundary: the brain only ever gets
                # `state`. The true pose is kept apart for the 3D house view (display only, /api/truth).
                state={k:raw[k] for k in ['sim_time_s','sensors','armed','fault','stereo_frame','stereo_available']}
                # The sensors the robot gets are what a real GY-521 and imperfectly calibrated wheels would report.
                state=self.sensor_model.apply(state)
                truth=raw.get('ground_truth_evaluation_only')
                with self.lock:
                    self.state=state;self.samples.append(state);self.received_at=time.monotonic()
                    if truth:self.display_truth=dict(sim_time_s=raw['sim_time_s'],**{k:truth[k] for k in ('position_m','yaw_rad','pitch_deg','roll_deg') if k in truth})
                    if state['armed']:self.arm_pending=False
            except socket.timeout:pass
            except (ValueError,KeyError):pass
    def latest(self):
        with self.lock:return self.state
    def truth_for_display(self):
        """Simulator ground-truth pose for the 3D house view only; never passed to the brain."""
        with self.lock:return getattr(self,'display_truth',None)
    def drain(self):
        with self.lock:
            samples=list(self.samples);self.samples.clear();return samples
    def stop(self):
        self.running=False;self.sock.sendto(b'{"motor_power_off":true,"capture":false}',('127.0.0.1',8767));self.join(timeout=1);self.sock.close()
class Application:
    def __init__(self,backend):
        self.brain=LidarBrain(ROOT) if backend=="lidar" else Brain(ROOT,backend);self.io=RobotIO();self.commands=queue.Queue();self.status_data=b'{}';self.map_png=b'';self.camera_png=b'';self.last_pair=None;self.stop=False;self.last_status=0;self.last_predict=-1.;self.last_frame_wall=0.
        pointer=ROOT/'Saved/Maps'/('latest-lidar-map.json' if backend=='lidar' else 'latest-map.json')
        if pointer.exists():
            try:
                folder=ROOT/'Saved/Maps'/json.loads(pointer.read_text())['directory'];self.brain.grid.load(folder);self.brain.localizer.load(folder);self.brain.map_folder=folder
                self.brain.message='Saved map loaded. Choose a destination, or start a new mapping run.'
            except (OSError,ValueError,KeyError):self.brain.message='Saved map could not be loaded with this feature backend. Start Mapping.'
    def read_pair(self,state):
        try:
            pointer=ROOT/'Saved/Stereo/latest-session.json'
            if time.time()-pointer.stat().st_mtime>2:return None
            folder=ROOT/'Saved/Stereo'/json.loads(pointer.read_text())['directory'];meta=json.loads((folder/'latest.json').read_text());key=(folder.name,meta['frame'])
            if key==self.last_pair or abs(state['sim_time_s']-meta['sim_time_s'])>.8:return None
            # Wait for motion samples bracketing image exposure. Using the most
            # recent earlier pose creates false image-motion disagreement in turns.
            if isinstance(self.brain,LidarBrain) and meta['sim_time_s']>self.last_predict:return None
            # Only image names, timestamp and camera calibration leave this adapter.
            allowed={k:meta[k] for k in ['frame','sim_time_s','fx','fy','cx','cy','baseline_m']}
            if isinstance(self.brain,LidarBrain):
                if Path(meta['left']).name!=meta['left']:return None
                left=cv2.imread(str(folder/meta['left']))
                if left is None:return None
                self.last_pair=key;self.camera_png=cv2.imencode('.png',left)[1].tobytes();return left,None,allowed
            if any(Path(meta[k]).name!=meta[k] for k in ['left','right']):return None
            left=cv2.imread(str(folder/meta['left']));right=cv2.imread(str(folder/meta['right']))
            if left is None or right is None:return None
            self.last_pair=key;self.camera_png=cv2.imencode('.png',left)[1].tobytes();return left,right,allowed
        except (OSError,ValueError,KeyError):return None
    def run(self):
        self.io.start()
        try:
            while not self.stop:
                state=self.io.latest()
                if state is None:time.sleep(.03);continue
                while not self.commands.empty():
                    command=self.commands.get();mode=command.get('mode')
                    try:
                        if mode=='mapping':self.brain.start_mapping(state['sim_time_s']);self.io.activate();self.last_pair=None
                        elif mode=='go_to':self.brain.begin_go_to(command['goal'],state['sim_time_s']);self.io.activate();self.last_pair=None
                        elif mode=='idle':self.brain.idle('Stopped by user; current mapping progress saved.')
                    except (ValueError,OSError,KeyError) as exc:self.brain.idle(str(exc))
                for sample in self.io.drain():
                    if sample['sim_time_s']>self.last_predict:self.brain.predict(sample);self.last_predict=sample['sim_time_s']
                active=self.brain.mode!='idle'
                if active:
                    if isinstance(self.brain,LidarBrain) and 'lidar' in state['sensors']:self.brain.observe_scan(state['sensors']['lidar'])
                    pair=self.read_pair(self.io.latest() or state)
                    if pair:
                        self.brain.observe(*pair);self.last_frame_wall=time.monotonic()
                        if isinstance(self.brain,LidarBrain):
                            preview=pair[0].copy();camera_status=getattr(self.brain,'low_camera',self.brain.camera).status
                            for pixel in camera_status.get('obstacle_pixels',[]):
                                cv2.circle(preview,tuple(round(v) for v in pixel),6,(20,185,255),2)
                            label=f"Low obstacles: {camera_status.get('new_cells',0)} new, {camera_status.get('confirmed_cells',0)} confirmed | {camera_status.get('state','')}"
                            cv2.rectangle(preview,(0,0),(preview.shape[1],25),(30,30,30),-1)
                            cv2.putText(preview,label,(8,17),cv2.FONT_HERSHEY_SIMPLEX,.43,(245,245,245),1,cv2.LINE_AA)
                            self.camera_png=cv2.imencode('.png',preview)[1].tobytes()
                v,w=self.brain.command(state);active=self.brain.mode!='idle';self.brain.requested_motion=(v,w)
                # After a contact the brain asks to re-arm; the motor core then accepts reverse only until the bumper clears.
                if active and getattr(self.brain,'wants_arm',False):self.io.activate()
                self.io.set(v,w,active)
                now=time.monotonic()
                if now-self.last_status>.3:
                    status=self.brain.status(state);status['connected']=now-self.io.received_at<1;status['decision_command']=[v,w];status['bridge_interlock']=self.io.interlock_reason;status['last_bridge_interlock']=self.io.last_interlock_reason;status['sim_sensor_errors']=self.io.sensor_model.describe();self.status_data=json.dumps(status).encode();self.map_png=cv2.imencode('.png',self.brain.grid.image(self.brain.path,self.brain.localizer.pose,self.brain.goal))[1].tobytes();self.last_status=now
                    (ROOT/'reports/autonomy-status.json').write_bytes(self.status_data)
                time.sleep(.005)
        except Exception as exc:
            self.brain.idle('Controller error: '+str(exc));self.status_data=json.dumps(self.brain.status(self.io.latest())).encode();(ROOT/'reports/autonomy-error.json').write_bytes(self.status_data);raise
        finally:self.io.stop()
# What the dashboard pages load: autonomy.html, the two pages it links, their script, textures and scene data, and the
# test results lidar-review.html shows. When the service listens on the home network nothing else in the sim folder is
# served. Only fixed names and plain [A-Za-z0-9_-] folder names match, so '..', '%2e', '\', '//' and '%00' never do.
DASHBOARD_FILES=re.compile(r'/(?:autonomy|house-preview|lidar-review)\.html'
    r'|/assets/(?:three\.min\.js|terrazzo-floor\.png|white-paint\.png|house/[a-z]+\.png)'
    r'|/Content/RobotData/(?:house-scene|house-meshes|assembly-meshes|robot-parameters)\.json'
    r'|/reports/(?:house-assets-robot-camera\.png|latest-(?:full-mapping|camera|go-to)-test\.json|camera-lidar-tests\.txt'
    r'|d200-model-test\.txt|camera-replay\.json|house-native-collision\.json|sensor-model\.json)'
    r'|/reports/[A-Za-z0-9_-]+/(?:trial\.json|results\.json|camera-results\.json|coverage-and-path\.png|final-camera\.png'
    r'|first-slipper-detection\.png)')
def serves_lan(host):
    """True unless the service listens only on this Mac (127.x / localhost)."""
    if host=='localhost':return False
    try:return not ipaddress.ip_address(host).is_loopback
    except ValueError:return True
def local_host_name():
    """This Mac's Bonjour name (System Settings > General > Sharing), e.g. Liors-MacBook-Pro; it answers as <name>.local."""
    try:return subprocess.run(['scutil','--get','LocalHostName'],capture_output=True,text=True,timeout=2).stdout.strip() or None
    except (OSError,subprocess.SubprocessError):return socket.gethostname().removesuffix('.local') or None
def lan_address():
    """The address other devices reach this Mac at (the one on the default route). Connecting a UDP socket sends nothing."""
    try:
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:s.connect(('192.0.2.1',9));return s.getsockname()[0]
    except OSError:return None
def dashboard_urls(host,port,name,lan_ip):
    """The dashboard addresses that answer for this --host."""
    page=f':{port}/autonomy.html'
    if not serves_lan(host):return ['http://127.0.0.1'+page]
    every=host=='0.0.0.0';address=lan_ip if every else host
    urls=(['http://127.0.0.1'+page] if every else [])+([f'http://{name}.local'+page] if name else [])+([f'http://{address}'+page] if address else [])
    return list(dict.fromkeys(urls))
class Handler(SimpleHTTPRequestHandler):
    app=None;root=ROOT;restricted=False;host_names=frozenset({'localhost'})
    def __init__(self,*a,**kw):super().__init__(*a,directory=str(self.root),**kw)
    def host_ok(self):
        """Host must be an IP address or one of this Mac's own names. A web page whose domain is re-pointed at this Mac
        (DNS rebinding) sends its own name, with a matching Origin, and is refused here."""
        try:name=urlsplit('//'+self.headers.get('Host','')).hostname
        except ValueError:return False
        if not name:return False
        try:ipaddress.ip_address(name);return True
        except ValueError:return name in self.host_names
    def send_head(self):
        # GET and HEAD of files both come here; /api routes are answered before.
        if self.restricted:
            path=unquote(urlsplit(self.path).path)
            if path=='/':self.send_response(302);self.send_header('Location','/autonomy.html');self.send_header('Content-Length','0');self.end_headers();return None
            if not DASHBOARD_FILES.fullmatch(path):self.send_error(404);return None
        return super().send_head()
    def do_HEAD(self):
        if self.restricted and not self.host_ok():self.send_error(403);return
        return super().do_HEAD()
    def do_GET(self):
        if self.restricted and not self.host_ok():self.send_error(403);return
        route=urlparse(self.path).path
        if route=='/api/truth':
            # Display only (3D house view): where the simulator has the robot. The controller never reads this.
            truth=self.app.io.truth_for_display()
            data=json.dumps(dict(available=truth is not None,**(truth or {}))).encode()
            self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data);return
        if route in ['/api/status','/api/map.png','/api/camera.png']:
            data={'/api/status':self.app.status_data,'/api/map.png':self.app.map_png,'/api/camera.png':self.app.camera_png}[route];self.send_response(200);self.send_header('Content-Type','application/json' if route.endswith('status') else 'image/png');self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data);return
        return super().do_GET()
    def do_POST(self):
        if self.path!='/api/mode':self.send_error(404);return
        # Same-page check: the dashboard posts from the address it was opened at (127.0.0.1, <name>.local or the LAN IP).
        origin=self.headers.get('Origin');host=self.headers.get('Host')
        if not self.host_ok() or origin and origin!='http://'+host:self.send_error(403);return
        try:
            n=int(self.headers.get('Content-Length','0'))
            if not 0<n<2048:raise ValueError('Invalid message length')
            obj=json.loads(self.rfile.read(n))
            if obj.get('mode') not in ['mapping','go_to','idle']:raise ValueError('Unknown mode')
            if obj['mode']=='go_to':
                goal=obj.get('goal');assert isinstance(goal,list) and len(goal)==2
                if not all(isinstance(x,(int,float)) and abs(x)<=7 for x in goal):raise ValueError('Invalid target')
            self.app.commands.put(obj);self.send_response(202);self.send_header('Content-Type','application/json');self.end_headers();self.wfile.write(b'{"queued":true}')
        except (ValueError,AssertionError):self.send_error(400,'Invalid mode or target')
    def log_message(self,*args):pass
def parse_args(argv=None):
    p=argparse.ArgumentParser();p.add_argument('--backend',default='lidar',choices=['lidar','orb','superpoint_lightglue']);p.add_argument('--port',type=int,default=8771)
    # 127.0.0.1: this Mac only, whole sim folder served. 0.0.0.0 (or this Mac's LAN address): phones and laptops on the
    # home Wi-Fi can open the dashboard, no password, and only the dashboard files are served (DASHBOARD_FILES).
    p.add_argument('--host',default='127.0.0.1',help='address to listen on; 0.0.0.0 opens the dashboard to the home network')
    return p.parse_args(argv)
def make_server(host,port,app,restricted=None,root=ROOT,local_name=None):
    name=local_name or local_host_name()
    names={'localhost',socket.gethostname().lower()}|({name.lower()+'.local'} if name else set())
    handler=type('DashboardHandler',(Handler,),dict(app=app,root=root,restricted=serves_lan(host) if restricted is None else restricted,host_names=frozenset(names)))
    return ThreadingHTTPServer((host,port),handler)
if __name__=='__main__':
    args=parse_args();app=Application(args.backend);Handler.app=app;server=make_server(args.host,args.port,app);threading.Thread(target=server.serve_forever,daemon=True).start()
    for url in dashboard_urls(args.host,args.port,local_host_name(),lan_address()):print(f'Robot controls: {url}',flush=True)
    if serves_lan(args.host):print('Open to the home network: no password, anyone on this Wi-Fi can switch modes. Only the dashboard files are served.',flush=True)
    try:app.run()
    except KeyboardInterrupt:app.stop=True
    finally:server.shutdown()
