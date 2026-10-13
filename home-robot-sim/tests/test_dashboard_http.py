"""Dashboard HTTP layer (LIO-135): --host flag, LAN allowlist, path traversal, Origin/Host checks. No Unreal needed:
the real Handler runs on an ephemeral port with a stub app and a temporary sim folder."""
import sys,json,queue,threading,tempfile,unittest,http.client
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'controller'))
import autonomy_service as service

# Every file the three dashboard pages load, as they request it; each one must be served on the LAN.
PAGE_FILES=['autonomy.html','house-preview.html','lidar-review.html','assets/three.min.js','assets/terrazzo-floor.png',
    'assets/white-paint.png','assets/house/wood.png','Content/RobotData/house-scene.json','Content/RobotData/house-meshes.json',
    'Content/RobotData/assembly-meshes.json','Content/RobotData/robot-parameters.json','reports/house-assets-robot-camera.png',
    'reports/latest-full-mapping-test.json','reports/latest-camera-test.json','reports/latest-go-to-test.json',
    'reports/full-mapping-20261012T192331/trial.json','reports/full-mapping-20261012T192331/results.json',
    'reports/full-mapping-20261012T192331/coverage-and-path.png','reports/full-mapping-20261012T192331/final-camera.png',
    'reports/full-mapping-20261003T202506/camera-results.json','reports/full-mapping-20261003T202506/first-slipper-detection.png',
    'reports/go-to-20261003T182443/results.json','reports/camera-lidar-tests.txt','reports/d200-model-test.txt']
# Files in the sim folder the dashboard does not need: served locally (unchanged), 404 on the LAN.
PRIVATE_FILES=['segmentation-review.html','controller/autonomy_service.py','Config/DefaultEngine.ini','reports/autonomy-status.json',
    'Saved/Maps/latest-lidar-map.json','.git/config','reports/full-mapping-20261012T192331/trace.jsonl']

def stub_app():
    return SimpleNamespace(status_data=b'{"mode":"idle"}',map_png=b'MAP',camera_png=b'CAM',commands=queue.Queue(),
        io=SimpleNamespace(truth_for_display=lambda:None))

class Server:
    def __init__(self,root,restricted):
        self.app=stub_app();self.server=service.make_server('127.0.0.1',0,self.app,restricted=restricted,root=root,local_name='Liors-MacBook-Pro')
        self.port=self.server.server_address[1];threading.Thread(target=self.server.serve_forever,args=(.02,),daemon=True).start()
    def request(self,method,path,body=None,headers=None):
        c=http.client.HTTPConnection('127.0.0.1',self.port,timeout=5)
        # skip_host lets a test send a Host header of its choosing (a phone opening the page by name or IP).
        c.putrequest(method,path,skip_host=bool(headers and 'Host' in headers),skip_accept_encoding=True)
        for k,v in (headers or {}).items():c.putheader(k,v)
        if body is not None:c.putheader('Content-Length',str(len(body)))
        c.endheaders(body);r=c.getresponse();data=r.read();c.close();return r.status,data,r
    def close(self):self.server.shutdown();self.server.server_close()

class Folder:
    def __enter__(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        for name in PAGE_FILES+PRIVATE_FILES:
            p=self.root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('file '+name)
        (self.root.parent/'outside-sim.txt').write_text('outside')
        return self.root
    def __exit__(self,*a):(self.root.parent/'outside-sim.txt').unlink(missing_ok=True);self.tmp.cleanup()

class HostFlag(unittest.TestCase):
    def test_default_host_is_loopback_and_port_unchanged(self):
        args=service.parse_args([]);self.assertEqual(args.host,'127.0.0.1');self.assertEqual(args.port,8771);self.assertEqual(args.backend,'lidar')
    def test_host_flag_is_accepted(self):
        self.assertEqual(service.parse_args(['--host','0.0.0.0','--port','9000']).host,'0.0.0.0')
    def test_only_non_loopback_addresses_restrict_the_folder(self):
        for host in ['127.0.0.1','localhost','127.0.0.2']:self.assertFalse(service.serves_lan(host),host)
        for host in ['0.0.0.0','192.168.1.201','Liors-MacBook-Pro.local']:self.assertTrue(service.serves_lan(host),host)
    def test_default_server_is_not_restricted_and_lan_server_is(self):
        s=service.make_server('127.0.0.1',0,stub_app())
        try:self.assertFalse(s.RequestHandlerClass.restricted)
        finally:s.server_close()

class StartupUrls(unittest.TestCase):
    def test_loopback_prints_only_the_local_url(self):
        self.assertEqual(service.dashboard_urls('127.0.0.1',8771,'Liors-MacBook-Pro','192.168.1.201'),['http://127.0.0.1:8771/autonomy.html'])
    def test_all_interfaces_print_local_name_and_lan_ip(self):
        self.assertEqual(service.dashboard_urls('0.0.0.0',8771,'Liors-MacBook-Pro','192.168.1.201'),
            ['http://127.0.0.1:8771/autonomy.html','http://Liors-MacBook-Pro.local:8771/autonomy.html','http://192.168.1.201:8771/autonomy.html'])
    def test_one_lan_address_prints_name_and_that_address(self):
        # Bound to one LAN address, 127.0.0.1 does not answer, so it is not printed.
        self.assertEqual(service.dashboard_urls('192.168.1.201',8771,'Liors-MacBook-Pro','10.0.0.5'),
            ['http://Liors-MacBook-Pro.local:8771/autonomy.html','http://192.168.1.201:8771/autonomy.html'])
    def test_bound_by_name_prints_it_once(self):
        self.assertEqual(service.dashboard_urls('Liors-MacBook-Pro.local',8771,'Liors-MacBook-Pro','192.168.1.201'),['http://Liors-MacBook-Pro.local:8771/autonomy.html'])
    def test_missing_name_or_ip_is_skipped(self):
        self.assertEqual(service.dashboard_urls('0.0.0.0',8771,None,None),['http://127.0.0.1:8771/autonomy.html'])

class LanAllowlist(unittest.TestCase):
    def setUp(self):
        self.folder=Folder();self.root=self.folder.__enter__();self.lan=Server(self.root,True);self.local=Server(self.root,False)
    def tearDown(self):self.lan.close();self.local.close();self.folder.__exit__()
    def test_every_file_the_pages_load_is_served(self):
        for name in PAGE_FILES:
            status,data,_=self.lan.request('GET','/'+name);self.assertEqual((status,data),(200,('file '+name).encode()),name)
    def test_page_requests_with_query_strings_are_served(self):
        self.assertEqual(self.lan.request('GET','/autonomy.html?x=1')[0],200)
    def test_api_routes_are_served(self):
        self.assertEqual(self.lan.request('GET','/api/status')[:2],(200,b'{"mode":"idle"}'))
        self.assertEqual(self.lan.request('GET','/api/map.png?t=5')[:2],(200,b'MAP'))
        self.assertEqual(self.lan.request('GET','/api/camera.png?t=5')[:2],(200,b'CAM'))
        self.assertEqual(json.loads(self.lan.request('GET','/api/truth')[1]),{'available':False})
    def test_everything_else_is_404_on_the_lan(self):
        for name in PRIVATE_FILES+['controller/','reports/','assets/','missing.html']:
            self.assertEqual(self.lan.request('GET','/'+name)[0],404,name)
            self.assertEqual(self.lan.request('HEAD','/'+name)[0],404,'HEAD '+name)
    def test_root_redirects_to_the_dashboard_on_the_lan(self):
        status,_,r=self.lan.request('GET','/');self.assertEqual(status,302);self.assertEqual(r.getheader('Location'),'/autonomy.html')
    def test_local_server_still_serves_the_whole_folder(self):
        for name in PAGE_FILES+PRIVATE_FILES:self.assertEqual(self.local.request('GET','/'+name)[0],200,name)
        self.assertEqual(self.local.request('GET','/')[0],200)  # directory listing, as before
    def test_path_traversal_is_refused(self):
        for path in ['/../outside-sim.txt','/assets/../controller/autonomy_service.py','/assets/%2e%2e/controller/autonomy_service.py',
                '/assets/%2E%2E%2Fcontroller%2Fautonomy_service.py','/%2e%2e/outside-sim.txt','/assets/%252e%252e/autonomy.html',
                '/assets/..%5c..%5coutside-sim.txt','/reports/../autonomy.html','/reports/..%2f..%2foutside-sim.txt/results.json',
                '/reports/%2e%2e/results.json','//controller/autonomy_service.py','//../outside-sim.txt','/assets/house/../../.git/config','/autonomy.html%00.png',
                '/./autonomy.html','/assets//three.min.js','/reports/./results.json']:
            status,data,_=self.lan.request('GET',path);self.assertEqual(status,404,path);self.assertNotIn(b'outside',data,path)

class ModeOrigin(unittest.TestCase):
    BODY=json.dumps({'mode':'idle'}).encode()
    def setUp(self):
        self.folder=Folder();self.root=self.folder.__enter__();self.lan=Server(self.root,True);self.local=Server(self.root,False)
    def tearDown(self):self.lan.close();self.local.close();self.folder.__exit__()
    def post(self,server,host,origin):
        headers={'Host':host,'Content-Type':'application/json'}
        if origin:headers['Origin']=origin
        return server.request('POST','/api/mode',self.BODY,headers)[0]
    def test_page_opened_by_lan_name_or_ip_can_switch_modes(self):
        p=self.lan.port
        for host in [f'Liors-MacBook-Pro.local:{p}',f'liors-macbook-pro.local:{p}',f'192.168.1.201:{p}',f'127.0.0.1:{p}',f'localhost:{p}']:
            self.assertEqual(self.post(self.lan,host,'http://'+host),202,host)
        self.assertEqual(self.lan.app.commands.qsize(),5)
    def test_local_page_still_switches_modes(self):
        p=self.local.port;self.assertEqual(self.post(self.local,f'127.0.0.1:{p}',f'http://127.0.0.1:{p}'),202)
        self.assertEqual(self.post(self.local,f'127.0.0.1:{p}',None),202)  # curl / scripts send no Origin
    def test_cross_origin_post_is_rejected(self):
        p=self.lan.port
        for server in [self.lan,self.local]:
            self.assertEqual(self.post(server,f'192.168.1.201:{p}','http://evil.example'),403)
            self.assertEqual(self.post(server,f'192.168.1.201:{p}',f'http://192.168.1.201:{p+1}'),403)
            self.assertEqual(self.post(server,f'192.168.1.201:{p}',f'https://192.168.1.201:{p}'),403)
            self.assertEqual(self.post(server,f'127.0.0.1:{p}','null'),403)
        self.assertTrue(self.lan.app.commands.empty() and self.local.app.commands.empty())
    def test_dns_rebinding_host_is_rejected(self):
        # A web page on evil.example re-pointed at this Mac would send matching Host and Origin; only this Mac's own names
        # and IP literals are accepted.
        p=self.lan.port
        for server in [self.lan,self.local]:
            self.assertEqual(self.post(server,f'evil.example:{p}',f'http://evil.example:{p}'),403)
        self.assertEqual(self.lan.request('GET','/api/status',headers={'Host':f'evil.example:{p}'})[0],403)
        self.assertTrue(self.lan.app.commands.empty() and self.local.app.commands.empty())
    def test_missing_host_header_is_rejected_not_a_crash(self):
        c=http.client.HTTPConnection('127.0.0.1',self.lan.port,timeout=5);c.putrequest('POST','/api/mode',skip_host=True)
        c.putheader('Origin','http://x');c.putheader('Content-Length',str(len(self.BODY)));c.endheaders(self.BODY)
        self.assertEqual(c.getresponse().status,403);c.close()

if __name__=='__main__':unittest.main()
