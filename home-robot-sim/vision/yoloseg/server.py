"""Serves the latest annotated frame and detections over HTTP (PC vision service and Jetson runner).

    /                        a page showing the live annotated frame and detections (handy on the Jetson)
    /vision/frame.jpg        the latest frame with masks, boxes and labels
    /vision/detections.json  model, frame id, timings and detections of that frame
Python 3.6 compatible (no ThreadingHTTPServer).
"""
import ipaddress
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from urllib.parse import urlsplit

from . import pipeline

PAGE = b"""<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot camera - object recognition</title>
<style>body{margin:0;background:#eff3ef;color:#263c31;font:15px/1.5 system-ui,sans-serif}main{max-width:900px;margin:auto;padding:16px}
img{width:100%;border-radius:8px;background:#273b31;min-height:200px}p{color:#5b7064;font-size:13px}</style>
<main><h2>Robot camera &middot; object recognition</h2><img id="f" alt="Annotated camera frame"><p id="c">Waiting for frames.</p><p id="o"></p></main>
<script>let key=null;async function poll(){try{const v=await (await fetch('vision/detections.json',{cache:'no-store'})).json();
if(v.model){const k=v.run+':'+v.frame;if(k!==key){key=k;document.getElementById('f').src='vision/frame.jpg?f='+encodeURIComponent(k)}
document.getElementById('c').textContent=v.model+' on '+v.device+' | '+v.total_ms.toFixed(1)+' ms per frame (inference '+v.ms.inference.toFixed(1)+' ms) | '+v.fps.toFixed(1)+' fps';
const n={};for(const d of v.detections)n[d.cls]=(n[d.cls]||0)+1;document.getElementById('o').textContent=v.detections.length?'Seen: '+Object.entries(n).map(([k,c])=>c>1?k+' x'+c:k).join(', '):'No objects above the confidence threshold.'}}
catch(e){document.getElementById('c').textContent='Not connected.'}setTimeout(poll,200)}poll();</script>"""


class ResultStore(object):
    """Latest annotated JPEG and detection info, shared between the inference loop and the HTTP threads."""

    def __init__(self, model_name, device_name):
        self.lock = threading.Lock()
        self.jpeg, self.info, self.updated = b'', {}, 0.0
        self.base = dict(model=model_name, device=device_name)
        self.frame_times = []

    def publish(self, segmenter, image, result, frame_id, run='live', extra=None):
        """Draw (torch), encode the finished image and store it with its detections."""
        jpeg = pipeline.encode_jpeg(pipeline.draw(image, result, segmenter.names))
        self._store(jpeg, segmenter, result, frame_id, run, extra)

    def publish_detections(self, segmenter, result, frame_id, run='live', extra=None):
        """Detections only (no drawing, no JPEG): the fastest mode on the Jetson."""
        self._store(b'', segmenter, result, frame_id, run, extra)

    def _store(self, jpeg, segmenter, result, frame_id, run, extra):
        now = time.time()
        self.frame_times = [t for t in self.frame_times if now - t < 2.0] + [now]
        fps = (len(self.frame_times) - 1) / max(1e-6, self.frame_times[-1] - self.frame_times[0]) if len(self.frame_times) > 1 else 0.0
        info = dict(self.base, segmentation=True, frame=frame_id, run=run, detections=segmenter.describe(result),
                    ms=dict((k, round(v, 2)) for k, v in result['ms'].items()), fps=round(fps, 2))
        info['total_ms'] = round(sum(result['ms'].values()), 2)
        info['inference_ms'] = info['total_ms']           # the dashboard shows this one
        info.update(extra or {})
        with self.lock:
            self.jpeg, self.info, self.updated = jpeg, info, now

    def snapshot(self):
        with self.lock:
            age = round(time.time() - self.updated, 2) if self.updated else None
            return self.jpeg, dict(self.info, age_s=age)


class _Handler(BaseHTTPRequestHandler):
    store = None
    allow_names = ('localhost',)

    def host_ok(self):
        # An IP address or an allowed name, so a web page whose domain is re-pointed here (DNS rebinding) is refused.
        name = urlsplit('//' + self.headers.get('Host', '')).hostname
        try:
            ipaddress.ip_address(name)
            return True
        except (ValueError, TypeError):
            return name in self.allow_names

    def send(self, body, kind):
        self.send_response(200)
        self.send_header('Content-Type', kind)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Access-Control-Allow-Origin', '*')   # read-only display data (the dashboard is on port 8771)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        try:
            self.wfile.write(body)
        except (ConnectionAbortedError, ConnectionResetError, BrokenPipeError):
            pass                                               # the page reloaded or dropped a superseded request

    def do_GET(self):
        if not self.host_ok():
            self.send_error(403)
            return
        jpeg, info = self.store.snapshot()
        route = urlsplit(self.path).path
        if route == '/vision/detections.json':
            self.send(json.dumps(info).encode(), 'application/json')
        elif route == '/vision/frame.jpg' and jpeg:
            self.send(jpeg, 'image/jpeg')
        elif route in ('/', '/index.html'):
            self.send(PAGE, 'text/html; charset=utf-8')
        else:
            self.send_error(404)

    def log_message(self, *args):
        pass


class _Server(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = False      # a second copy on the same port fails loudly instead of sharing it (Windows)


def serve(store, host, port, allow_names=('localhost',)):
    handler = type('Handler', (_Handler,), dict(store=store, allow_names=tuple(allow_names)))
    _Server((host, port), handler).serve_forever()
