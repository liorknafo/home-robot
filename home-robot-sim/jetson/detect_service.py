"""YOLO26 instance segmentation on the Jetson Nano: camera -> TensorRT -> PyTorch post-processing -> web view.

No OpenCV in the pipeline: frames come from GStreamer (or HTTP / image files) as numpy buffers into torch CUDA tensors;
letterbox, top-k, masks and drawing are torch / torchvision (labels with PIL); torchvision encodes the final JPEG.

    python3 detect_service.py --engine models/yolo26n-seg-384x640.engine --source usb
    python3 detect_service.py --engine ... --source csi                      # CSI camera (nvarguscamerasrc)
    python3 detect_service.py --engine ... --source http://PC:8771/api/camera.png   # the simulator's robot camera
    python3 detect_service.py --engine ... --source 'frames/*.png' --benchmark 200  # timing, no web view
Then open http://<jetson-ip>:8772/ (live annotated view) or read /vision/detections.json.
Python 3.6 / torch 1.10 / torchvision 0.11 / TensorRT 8.2 (JetPack 4.6); also runs on a PC with TensorRT 10+.
"""
import argparse
import glob
import os
import socket
import sys
import threading
import time

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'vision'))
from yoloseg import pipeline                       # noqa: E402
from yoloseg.backends import TensorRTBackend       # noqa: E402
from yoloseg.server import ResultStore, serve      # noqa: E402


class GstSource(object):
    """Frames from a GStreamer pipeline ending in 'appsink name=sink' with RGBA output (no row padding)."""

    def __init__(self, description, device):
        import gi
        gi.require_version('Gst', '1.0')
        from gi.repository import Gst
        Gst.init(None)
        self.Gst, self.device = Gst, device
        self.pipeline = Gst.parse_launch(description)
        self.sink = self.pipeline.get_by_name('sink')
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('GStreamer could not start: %s' % description)
        self.count = 0

    def read(self):
        sample = self.sink.emit('pull-sample')               # blocks until the next frame
        if sample is None:
            raise EOFError('camera stream ended')
        caps = sample.get_caps().get_structure(0)
        w, h = caps.get_value('width'), caps.get_value('height')
        buf = sample.get_buffer()
        ok, info = buf.map(self.Gst.MapFlags.READ)
        if not ok:
            raise RuntimeError('could not map a camera buffer')
        try:
            rgba = np.frombuffer(info.data, dtype=np.uint8, count=w * h * 4).reshape(h, w, 4)
            frame = torch.from_numpy(rgba.copy()).to(self.device, non_blocking=True)
        finally:
            buf.unmap(info)
        self.count += 1
        return frame[:, :, :3].permute(2, 0, 1).contiguous(), self.count

    def close(self):
        self.pipeline.set_state(self.Gst.State.NULL)


def usb_pipeline(device, width, height, fps, out_w, out_h, mjpeg):
    raw = 'image/jpeg,width=%d,height=%d,framerate=%d/1 ! jpegdec' if mjpeg else 'video/x-raw,width=%d,height=%d,framerate=%d/1'
    return ('v4l2src device=%s ! ' % device + raw % (width, height, fps) +
            ' ! videoconvert ! videoscale ! video/x-raw,format=RGBA,width=%d,height=%d ! '
            'appsink name=sink max-buffers=1 drop=true sync=false' % (out_w, out_h))


def csi_pipeline(sensor, width, height, fps, out_w, out_h):
    return ('nvarguscamerasrc sensor-id=%d ! video/x-raw(memory:NVMM),width=%d,height=%d,framerate=%d/1 ! '
            'nvvidconv ! video/x-raw,format=RGBA,width=%d,height=%d ! appsink name=sink max-buffers=1 drop=true sync=false'
            % (sensor, width, height, fps, out_w, out_h))


class HttpSource(object):
    """Polls an image URL, e.g. the dashboard's robot camera http://<pc>:8771/api/camera.png; skips repeated frames."""

    def __init__(self, url, device, period=0.05):
        self.url, self.device, self.period, self.last, self.count = url, device, period, None, 0

    def read(self):
        from urllib.request import urlopen
        while True:
            data = urlopen(self.url, timeout=5).read()
            if data and data != self.last:
                self.last = data
                self.count += 1
                return pipeline.decode_image(data, self.device), self.count
            time.sleep(self.period)

    def close(self):
        pass


class FileSource(object):
    """Image files (a glob), looping."""

    def __init__(self, pattern, device):
        self.files = sorted(glob.glob(pattern))
        if not self.files:
            raise IOError('no images match %s' % pattern)
        self.device, self.count = device, 0

    def read(self):
        path = self.files[self.count % len(self.files)]
        self.count += 1
        return pipeline.read_image(path, self.device), self.count

    def close(self):
        pass


def open_source(args, device):
    s = args.source
    if s == 'usb':
        return GstSource(usb_pipeline(args.device, args.cam_width, args.cam_height, args.cam_fps, args.width, args.height,
                                      args.mjpeg), device)
    if s == 'csi':
        return GstSource(csi_pipeline(args.sensor_id, args.cam_width, args.cam_height, args.cam_fps, args.width,
                                      args.height), device)
    if s == 'gst':
        return GstSource(args.gst, device)
    if s.startswith('http://') or s.startswith('https://'):
        return HttpSource(s, device)
    return FileSource(s, device)


def benchmark(segmenter, source, n, draw):
    timings = []
    for i in range(n + 10):
        image, _ = source.read()
        torch.cuda.synchronize()
        start = time.time()
        result = segmenter(image)
        if draw:
            pipeline.encode_jpeg(pipeline.draw(image, result, segmenter.names))
        total = (time.time() - start) * 1000
        if i >= 10:                                   # skip warm-up frames
            timings.append(dict(result['ms'], total=total))
    print('%d frames, median ms (%s):' % (n, 'with drawing + JPEG' if draw else 'detections only'))
    for key in ('preprocess', 'inference', 'postprocess', 'total'):
        values = sorted(t[key] for t in timings)
        print('  %-11s %7.2f' % (key, values[len(values) // 2]))
    print('  => %.1f frames per second' % (1000.0 / sorted(t['total'] for t in timings)[len(timings) // 2]))


def loop(segmenter, source, store, draw):
    last_print, frames = time.time(), 0
    while True:
        image, frame_id = source.read()
        result = segmenter(image)
        if draw:
            store.publish(segmenter, image, result, frame_id)
        else:
            store.publish_detections(segmenter, result, frame_id)
        frames += 1
        if time.time() - last_print > 5:
            print('%.1f fps | last frame: %d objects, %.1f ms (inference %.1f ms)' % (
                frames / (time.time() - last_print), len(result['boxes']), sum(result['ms'].values()), result['ms']['inference']))
            last_print, frames = time.time(), 0


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--engine', required=True, help='TensorRT engine from build_engine.py (built on this device)')
    p.add_argument('--source', default='usb', help="usb | csi | gst | http(s)://... image URL | image glob, e.g. 'frames/*.png'")
    p.add_argument('--device', default='/dev/video0', help='USB camera device')
    p.add_argument('--mjpeg', action='store_true', help='USB camera sends MJPEG (needed for 720p/1080p at 30 fps on most UVC cameras)')
    p.add_argument('--sensor-id', type=int, default=0, help='CSI camera index')
    p.add_argument('--cam-width', type=int, default=1280)
    p.add_argument('--cam-height', type=int, default=720)
    p.add_argument('--cam-fps', type=int, default=30)
    p.add_argument('--width', type=int, default=640, help='frame size handed to the model (GStreamer scales to it)')
    p.add_argument('--height', type=int, default=360)
    p.add_argument('--gst', help='custom GStreamer pipeline for --source gst; must end in "appsink name=sink" with RGBA')
    p.add_argument('--conf', type=float, default=0.35)
    p.add_argument('--max-det', type=int, default=100)
    p.add_argument('--no-draw', action='store_true', help='detections JSON only (skips drawing and JPEG: faster)')
    p.add_argument('--host', default='0.0.0.0', help='0.0.0.0: reachable from other computers on the network')
    p.add_argument('--port', type=int, default=8772)
    p.add_argument('--benchmark', type=int, metavar='N', help='time N frames, print the result and exit')
    args = p.parse_args()

    if not torch.cuda.is_available():
        sys.exit('PyTorch has no CUDA here: install the JetPack PyTorch wheel (JETSON.md step 3)')
    device = 'cuda'
    backend = TensorRTBackend(args.engine, device)
    names = backend.names or ['class%d' % i for i in range(80)]
    segmenter = pipeline.Segmenter(backend, names, backend.input_size, conf=args.conf, max_det=args.max_det,
                                   end2end=backend.end2end)
    segmenter(torch.zeros(3, args.height, args.width, dtype=torch.uint8, device=device))   # warm-up
    source = open_source(args, device)
    print('engine %s (input %dx%d), source %s' % (os.path.basename(args.engine), backend.input_size[0],
                                                   backend.input_size[1], args.source))
    try:
        if args.benchmark:
            benchmark(segmenter, source, args.benchmark, not args.no_draw)
            return
        store = ResultStore(os.path.splitext(os.path.basename(args.engine))[0], torch.cuda.get_device_name(0))
        hostname = socket.gethostname()
        threading.Thread(target=serve, args=(store, args.host, args.port, ('localhost', hostname, hostname + '.local')),
                         daemon=True).start()
        print('Live view: http://%s:%d/  (detections: /vision/detections.json)' % (
            hostname if args.host == '0.0.0.0' else args.host, args.port))
        loop(segmenter, source, store, not args.no_draw)
    except KeyboardInterrupt:
        pass
    finally:
        source.close()


if __name__ == '__main__':
    main()
