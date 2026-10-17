"""Live object detection and instance segmentation on the simulated robot camera, for the dashboard (display only).

Reads the frames Unreal writes to Saved/Stereo/<run>/ (latest-session.json -> latest.json -> left_NNNNNN.png), the same
files the autonomy service reads, runs the model on each new frame on this PC's GPU and serves the result:
    /vision/frame.jpg         the camera frame with masks, boxes and labels drawn on it
    /vision/detections.json   frame number, sim time, model, timings and the detections (class, confidence, box)
The pipeline is PyTorch end to end (yoloseg: decode, letterbox, model, top-k, masks, drawing); OpenCV is not used.
Nothing is sent to the robot controller; the robot does not act on these detections. The Unreal camera only captures
while the robot is mapping or going to a position (5 Hz), so the view holds the last frame when the robot is idle.

    vision\\.venv\\Scripts\\python vision\\vision_service.py [--model yolo26n-seg.pt | <file>.engine] [--port 8772]
"""
import argparse, json, os, sys, threading, time, urllib.request
from pathlib import Path

import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import yolov12_compat
from yoloseg import pipeline
from yoloseg.backends import load_backend
from yoloseg.server import ResultStore, serve

yolov12_compat.install()
ROOT = Path(__file__).resolve().parents[1]
STEREO = ROOT / 'Saved' / 'Stereo'
WEIGHTS = Path(__file__).resolve().parent / 'weights'
# Weights Ultralytics cannot download by name: YOLOv12 segmentation from the YOLOv12 authors (run via yolov12_compat).
AUTHOR_WEIGHTS = {f'yolov12{s}-seg.pt': f'https://github.com/sunsmarterjie/yolov12/releases/download/seg/yolov12{s}-seg.pt'
                  for s in 'nsmlx'}


def latest_frame():
    folder = STEREO / json.loads((STEREO / 'latest-session.json').read_text())['directory']
    meta = json.loads((folder / 'latest.json').read_text())
    if Path(meta['left']).name != meta['left']:
        return None
    return (folder.name, meta['frame']), folder / meta['left'], meta


def run(segmenter, store, device):
    key = None
    while True:
        try:
            found = latest_frame()
        except (OSError, ValueError, KeyError):
            found = None
        if found is None or found[0] == key:
            time.sleep(.03)
            continue
        try:
            image = pipeline.read_image(str(found[1]), device)
        except (OSError, RuntimeError):      # Unreal may still be writing the PNG; try again on the next pass
            time.sleep(.02)
            continue
        key, meta = found[0], found[2]
        result = segmenter(image)
        store.publish(segmenter, image, result, meta['frame'], run=key[0], extra=dict(sim_time_s=meta.get('sim_time_s')))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--model', default='yolo26n-seg.pt',
                        help='Ultralytics weights (vision/weights, or a name downloaded on first use) or a TensorRT .engine '
                             'from export_onnx.py + build_engine.py. yolov12n-seg.pt is fetched from the YOLOv12 authors\' '
                             'release (Ultralytics has no YOLO12 segmentation weights)')
    parser.add_argument('--imgsz', type=int, nargs=2, default=[384, 640], metavar=('H', 'W'),
                        help='model input; 384 x 640 fits the 16:9 camera (ignored for an engine: its own size)')
    parser.add_argument('--conf', type=float, default=.35)
    parser.add_argument('--port', type=int, default=8772)
    parser.add_argument('--host', default='127.0.0.1')
    args = parser.parse_args()
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model = args.model
    if not model.endswith('.engine'):
        WEIGHTS.mkdir(exist_ok=True)
        os.chdir(WEIGHTS)              # Ultralytics downloads named weights into the working folder
        if model in AUTHOR_WEIGHTS and not Path(model).exists():
            urllib.request.urlretrieve(AUTHOR_WEIGHTS[model], model)
    backend = load_backend(model, device)
    size = getattr(backend, 'input_size', None) or tuple(args.imgsz)
    segmenter = pipeline.Segmenter(backend, backend.names, size, conf=args.conf, end2end=backend.end2end)
    segmenter(torch.zeros(3, 360, 640, dtype=torch.uint8, device=device))   # warm-up: CUDA init, kernel selection
    device_name = torch.cuda.get_device_name(0) if device == 'cuda' else 'CPU (no CUDA GPU found)'
    store = ResultStore(Path(model).stem, device_name)
    threading.Thread(target=run, args=(segmenter, store, device), daemon=True).start()
    print(f'{Path(model).stem} ({"TensorRT" if model.endswith(".engine") else "PyTorch"}, input {size[0]}x{size[1]}) on {device_name}')
    print(f'Serving http://{args.host}:{args.port}/vision/frame.jpg for the dashboard; frames from {STEREO}')
    serve(store, args.host, args.port)


if __name__ == '__main__':
    main()
