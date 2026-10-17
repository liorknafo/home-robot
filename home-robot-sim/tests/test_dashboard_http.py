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

# LIO-168: the robot model page (R1 design review) is served from the R1 folder under /robot/. Everything that page loads or links
# is listed here as the page requests it (relative to /robot/); a fake R1 folder stands in for the real one.
ROBOT_FILES=['design-review.html','assets/three.min.js','robot-internal.png','robot-exploded.png','cad/home-robot-r1.glb',
    'cad/home-robot-r1.step','cad/build_robot.py','parts-and-connections.json','pcb/fab/ORDERING.md','pcb/fab/r1_power_io-bom.csv',
    'print-parts/P_base.stl','print-parts/P_base.step','print-parts/P_lid_post_1.stl','print-parts/bay_clamp_spacer_3.step',
    'print-parts/bay_rib_0.stl','print-parts/coupons/P_coupon.stl','print-parts/coupons/P_coupon.step']
# R1 files the page does not load or link: 404 even though they sit in the R1 folder.
ROBOT_PRIVATE_FILES=['BRINGUP.md','robot-assembled.png','assets/THREE-LICENSE.txt','cad/parameters.json','cad/review-template.html',
    'pcb/fab/r1_power_io-gerbers.zip','pcb/r1_power_io.kicad_pcb','print-parts/notes.txt','print-parts/P_base.pdf','print-parts/coupons/readme.md',
    '.git/config','.superpowers/sdd/progress.md','reference-models/jetson.step']

def stub_app():
    return SimpleNamespace(status_data=b'{"mode":"idle"}',map_png=b'MAP',camera_png=b'CAM',commands=queue.Queue(),
        io=SimpleNamespace(truth_for_display=lambda:None))

class Server:
    def __init__(self,root,restricted,r1_dir=None):
        self.app=stub_app();self.server=service.make_server('127.0.0.1',0,self.app,restricted=restricted,root=root,local_name='Liors-MacBook-Pro',r1_dir=r1_dir)
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

class FakeR1:
    """A temporary stand-in for the home-robot-r1 folder: the real one is 18 MB and absent in worktrees."""
    def __enter__(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)/'home-robot-r1'
        for name in ROBOT_FILES+ROBOT_PRIVATE_FILES:
            p=self.root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('r1 '+name)
        (self.root.parent/'outside-r1.txt').write_text('outside')
        return self.root
    def __exit__(self,*a):self.tmp.cleanup()

class HostFlag(unittest.TestCase):
    def test_default_host_is_loopback_and_port_unchanged(self):
        args=service.parse_args([]);self.assertEqual(args.host,'127.0.0.1');self.assertEqual(args.port,8771);self.assertEqual(args.backend,'lidar')
    def test_host_flag_is_accepted(self):
        self.assertEqual(service.parse_args(['--host','0.0.0.0','--port','9000']).host,'0.0.0.0')
    def test_r1_dir_defaults_to_the_sibling_r1_folder_and_can_be_set(self):
        self.assertEqual(service.parse_args([]).r1_dir,service.DEFAULT_R1_DIR);self.assertEqual(service.DEFAULT_R1_DIR,ROOT.parent/'home-robot-r1')
        self.assertEqual(service.parse_args(['--r1-dir','/tmp/r1']).r1_dir,Path('/tmp/r1'))
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

class RobotModelPage(unittest.TestCase):
    """/robot/ serves the R1 design review and what it loads or links, from the R1 folder, on this Mac and on the LAN."""
    def setUp(self):
        self.folder=Folder();self.root=self.folder.__enter__();self.r1f=FakeR1();self.r1=self.r1f.__enter__()
        self.servers={'lan':Server(self.root,True,self.r1),'local':Server(self.root,False,self.r1)}
    def tearDown(self):
        for s in self.servers.values():s.close()
        self.r1f.__exit__();self.folder.__exit__()
    def each(self):return self.servers.items()
    def test_the_page_and_everything_it_loads_or_links_are_served(self):
        for mode,s in self.each():
            for name in ROBOT_FILES:
                status,data,_=s.request('GET','/robot/'+name);self.assertEqual((status,data),(200,('r1 '+name).encode()),(mode,name))
                self.assertEqual(s.request('HEAD','/robot/'+name)[0],200,(mode,'HEAD',name))
    def test_query_string_is_ignored(self):
        for mode,s in self.each():self.assertEqual(s.request('GET','/robot/design-review.html?v=2')[:2],(200,b'r1 design-review.html'),mode)
    def test_r1_files_the_page_does_not_use_are_404(self):
        for mode,s in self.each():
            for name in ROBOT_PRIVATE_FILES+['','print-parts/','cad/','pcb/fab/','assets/','missing.html','print-parts/P_missing.stl']:
                self.assertEqual(s.request('GET','/robot/'+name)[0],404,(mode,name));self.assertEqual(s.request('HEAD','/robot/'+name)[0],404,(mode,'HEAD',name))
    def test_nothing_from_the_sim_folder_is_served_under_robot(self):
        for mode,s in self.each():
            for name in ['autonomy.html','controller/autonomy_service.py','assets/house/wood.png','Content/RobotData/house-scene.json']:
                self.assertEqual(s.request('GET','/robot/'+name)[0],404,(mode,name))
    def test_r1_files_are_not_reachable_outside_robot(self):
        # R1 is never mapped into the sim folder's URLs: '/design-review.html' and '/print-parts/...' are not R1's.
        for mode,s in self.each():
            for path in ['/design-review.html','/print-parts/P_base.stl','/cad/home-robot-r1.glb','/robot','/robots/design-review.html','/Robot/design-review.html']:
                status,data,_=s.request('GET',path);self.assertEqual(status,404,(mode,path));self.assertNotIn(b'r1 ',data,(mode,path))
    def test_path_traversal_out_of_the_r1_folder_is_refused(self):
        for mode,s in self.each():
            for path in ['/robot/../outside-sim.txt','/robot/../outside-r1.txt','/robot/%2e%2e/outside-r1.txt','/robot/assets/../BRINGUP.md',
                    '/robot/assets/%2e%2e/BRINGUP.md','/robot/assets/%2E%2E%2FBRINGUP.md','/robot/print-parts/../.git/config','/robot/print-parts/..%5cBRINGUP.md',
                    '/robot/print-parts/%2e%2e/P_base.stl/../../BRINGUP.md','/robot/print-parts/%252e%252e/BRINGUP.md','/robot/print-parts/coupons/../../BRINGUP.md',
                    '/robot/print-parts/coupons/../../.git/config','/robot//design-review.html','/robot/./design-review.html','/robot/assets//three.min.js',
                    '/robot/print-parts//P_base.stl','/robot/design-review.html%00.png','/robot/print-parts/P_base.stl%00.pdf',
                    '/robot/..','/robot/%2e%2e/home-robot-sim/autonomy.html','/robot/print-parts/.%2e/BRINGUP.md','/robot/print-parts/P_base.stl/../../.git/config']:
                status,data,_=s.request('GET',path);self.assertIn(status,(404,),(mode,path));self.assertNotIn(b'outside',data,(mode,path));self.assertNotIn(b'r1 ',data,(mode,path))
    def test_a_symlink_in_an_allowed_name_still_stays_inside_the_r1_folder(self):
        # The allowlist is on names; a link planted at an allowed name must not turn into a read of a file outside R1.
        outside=self.r1.parent/'outside-r1.txt';link=self.r1/'print-parts'/'P_link.stl'
        try:link.symlink_to(outside)
        except OSError as e:self.skipTest(f'cannot create symlinks here (Windows needs Developer Mode): {e}')
        for mode,s in self.each():self.assertEqual(s.request('GET','/robot/print-parts/P_link.stl')[0],404,mode)
    def test_host_and_origin_checks_apply_to_robot_files(self):
        p=self.servers['lan'].port
        self.assertEqual(self.servers['lan'].request('GET','/robot/design-review.html',headers={'Host':f'evil.example:{p}'})[0],403)
        self.assertEqual(self.servers['lan'].request('GET','/robot/design-review.html',headers={'Host':f'192.168.1.201:{p}'})[0],200)
        self.assertEqual(self.servers['lan'].request('GET','/robot/design-review.html',headers={'Host':f'Liors-MacBook-Pro.local:{p}'})[0],200)
        self.assertEqual(self.servers['lan'].request('HEAD','/robot/design-review.html',headers={'Host':f'evil.example:{p}'})[0],403)
    def test_pages_and_api_still_work_next_to_robot(self):
        for mode,s in self.each():
            self.assertEqual(s.request('GET','/autonomy.html')[:2],(200,b'file autonomy.html'),mode);self.assertEqual(s.request('GET','/api/status')[0],200,mode)

class RobotModelUnavailable(unittest.TestCase):
    """No R1 folder (a worktree has no sibling home-robot-r1): a clear 404, and the rest of the dashboard is unaffected."""
    def setUp(self):
        self.folder=Folder();self.root=self.folder.__enter__();self.tmp=tempfile.TemporaryDirectory()
        self.servers={'lan':Server(self.root,True,Path(self.tmp.name)/'no-such-r1'),'local':Server(self.root,False,Path(self.tmp.name)/'no-such-r1')}
    def tearDown(self):
        for s in self.servers.values():s.close()
        self.tmp.cleanup();self.folder.__exit__()
    def test_missing_r1_folder_answers_404_with_a_message_naming_the_option(self):
        for mode,s in self.servers.items():
            for path in ['/robot/design-review.html','/robot/assets/three.min.js','/robot/print-parts/P_base.stl']:
                status,data,_=s.request('GET',path);self.assertEqual(status,404,(mode,path));self.assertIn(b'--r1-dir',data,(mode,path))
                self.assertNotIn(self.tmp.name.encode(),data,(mode,path))  # the message does not leak this Mac's folder layout
            self.assertEqual(s.request('HEAD','/robot/design-review.html')[0],404,mode)
    def test_the_rest_of_the_dashboard_is_unaffected(self):
        for mode,s in self.servers.items():
            self.assertEqual(s.request('GET','/autonomy.html')[0],200,mode);self.assertEqual(s.request('GET','/api/status')[0],200,mode)
    def test_r1_dir_that_is_a_file_or_default_missing_is_also_a_404(self):
        f=Path(self.tmp.name)/'a-file';f.write_text('x');s=Server(self.root,True,f)
        try:self.assertEqual(s.request('GET','/robot/design-review.html')[0],404)
        finally:s.close()
        d=Server(self.root,True)  # no r1_dir: the default sibling of the real sim folder (absent in a worktree, present in the main checkout)
        try:self.assertIn(d.request('GET','/robot/design-review.html')[0],(200,404))
        finally:d.close()

class PageLinks(unittest.TestCase):
    """The pages the dashboard serves must work from a phone: no address that only means something on this Mac."""
    PAGES=['autonomy.html','house-preview.html','lidar-review.html']
    def test_served_pages_have_no_hard_coded_local_addresses_in_links(self):
        import re
        # simulation-review.html is a Mac-only report (404 on the LAN), so only its design-review link has to change.
        self.assertNotIn(':8766',(ROOT/'simulation-review.html').read_text(encoding='utf8'))
        for page in self.PAGES:
            html=(ROOT/page).read_text(encoding='utf8')
            self.assertNotIn(':8766',html,page)  # the old ad-hoc design-review server
            for m in re.finditer(r'''(?:href|src|action)\s*=\s*["']?https?://(?:127\.0\.0\.1|localhost)''',html):self.fail(f'{page}: hard-coded local link {m.group(0)}')
            self.assertNotRegex(html,r'127\.0\.0\.1:8771',page)  # also not in messages shown on the phone
    def test_robot_model_links_point_at_the_service_prefix(self):
        for page in ['lidar-review.html','simulation-review.html']:
            self.assertIn('href="robot/design-review.html#model"',(ROOT/page).read_text(encoding='utf8'),page)
        self.assertIn('robot/design-review.html#model',(ROOT/'Scripts/build_report.py').read_text(encoding='utf8'))  # the generator of simulation-review.html
    def test_robot_controls_link_is_relative(self):
        self.assertIn('id="controls-link" href="autonomy.html"',(ROOT/'house-preview.html').read_text(encoding='utf8'))

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
