"""Export YOLO26-seg to ONNX for TensorRT on the Jetson Nano (run on the PC; needs the vision venv: ultralytics, onnx).

The graph returns the NMS-free (one2one) head's raw outputs, the same tensors as yoloseg's TorchBackend:
    images (1, 3, H, W) float 0..1  ->  preds (1, A, 4 + nc + nm), proto (1, nm, H/4, W/4)
Ultralytics' own end-to-end export adds a top-k with a Mod operator that TensorRT < 8.5 cannot parse (the Nano has 8.2),
so the top-k stays outside, in torch (yoloseg.pipeline.topk_detections). Opset 12, fixed input size, FP32 (the engine
builder turns on FP16). Writes <name>.onnx and <name>.json (class names, input size) for build_engine.py and checks the
ONNX against PyTorch with ONNX Runtime on a camera frame.

    vision\\.venv\\Scripts\\python vision\\export_onnx.py [--weights yolo26n-seg.pt] [--imgsz 384 640] [--out jetson\\models]
"""
import argparse, collections, json, sys
from pathlib import Path

import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from yoloseg.backends import configure_head
from yoloseg.pipeline import letterbox, read_image

# Operators TensorRT 8.2's ONNX parser rejects or that need a plugin on the Nano; the export must not contain them.
TRT82_UNSUPPORTED = {'Mod', 'NonZero', 'Loop', 'If', 'Scan', 'NonMaxSuppression', 'LayerNormalization', 'Einsum',
                     'ScatterElements', 'GridSample', 'Trilu'}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--weights', default='yolo26n-seg.pt', help='Ultralytics weights (vision/weights or a name to download)')
    parser.add_argument('--imgsz', type=int, nargs=2, default=[384, 640], metavar=('H', 'W'),
                        help='fixed model input; multiples of 32. 384 x 640 fits the 16:9 camera with little padding')
    parser.add_argument('--opset', type=int, default=12)
    parser.add_argument('--out', default=str(HERE.parent / 'jetson' / 'models'))
    parser.add_argument('--frame', help='image to check ONNX Runtime against PyTorch (default: latest simulator frame)')
    args = parser.parse_args()
    export(args.weights, args.imgsz, args.opset, args.out, args.frame)


def export(weights_name='yolo26n-seg.pt', imgsz=(384, 640), opset=12, out=None, frame=None):
    """Writes <out>/<weights stem>-<H>x<W>.onnx and .json, checks them, and returns the ONNX path."""
    H, W = imgsz
    if H % 32 or W % 32:
        sys.exit('--imgsz must be multiples of 32')

    from ultralytics import YOLO
    weights = HERE / 'weights' / weights_name
    model = YOLO(str(weights) if weights.exists() else weights_name).model.float().eval()
    end2end = configure_head(model)
    if not end2end:
        print('Note: not an end-to-end (YOLO26) head; the engine will need NMS on the Jetson (slower).')
    model.fuse()                                   # conv + BN; drops the unused head branch
    out_dir = Path(out or HERE.parent / 'jetson' / 'models')
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = '%s-%dx%d' % (Path(weights_name).stem, H, W)
    onnx_path = out_dir / (stem + '.onnx')

    x = torch.zeros(1, 3, H, W)
    with torch.no_grad():
        model(x)                                   # builds the anchors for this input size
        torch.onnx.export(model, x, str(onnx_path), opset_version=opset, input_names=['images'],
                          output_names=['preds', 'proto'], do_constant_folding=True, dynamo=False)
    try:
        import onnxslim
        import onnx
        onnx.save(onnxslim.slim(onnx.load(str(onnx_path))), str(onnx_path))
    except ImportError:
        print('onnxslim not installed: ONNX left unsimplified')

    import onnx
    graph = onnx.load(str(onnx_path))
    onnx.checker.check_model(graph)
    ops = collections.Counter(node.op_type for node in graph.graph.node)
    bad = sorted(set(ops) & TRT82_UNSUPPORTED)
    shapes = {o.name: [d.dim_value for d in o.type.tensor_type.shape.dim] for o in graph.graph.output}
    names = [model.names[i] for i in range(len(model.names))]
    meta = dict(names=names, input_size=[H, W], end2end=end2end, nc=len(names), nm=shapes['proto'][1], opset=opset,
                weights=weights_name, outputs=shapes)
    (out_dir / (stem + '.json')).write_text(json.dumps(meta, indent=1))
    print('wrote %s (%.1f MB) and %s.json' % (onnx_path, onnx_path.stat().st_size / 1e6, stem))
    print('outputs %s' % shapes)
    print('operators: ' + ', '.join('%s %d' % kv for kv in sorted(ops.items())))
    if bad:
        sys.exit('NOT TensorRT 8.2 compatible: contains %s' % ', '.join(bad))
    print('TensorRT 8.2 check: none of %s' % ', '.join(sorted(TRT82_UNSUPPORTED)))

    try:
        import onnxruntime as ort
    except ImportError:
        print('onnxruntime not installed: skipped the ONNX vs PyTorch check')
        return onnx_path
    frame = frame or latest_sim_frame()
    image = read_image(frame, 'cpu') if frame else torch.randint(0, 255, (3, 360, 640), dtype=torch.uint8)
    inp, _ = letterbox(image, (H, W))
    with torch.no_grad():
        ref = model(inp)
    got = ort.InferenceSession(str(onnx_path), providers=['CPUExecutionProvider']).run(None, {'images': inp.numpy()})
    for name, a, b in zip(['preds', 'proto'], ref, got):
        print('ONNX Runtime vs PyTorch %s: max abs difference %.2e' % (name, float((a - torch.from_numpy(b)).abs().max())))
    return onnx_path


def latest_sim_frame():
    stereo = HERE.parent / 'Saved' / 'Stereo'
    frames = sorted(stereo.glob('*/left_*.png'), key=lambda p: p.stat().st_mtime)
    return str(frames[-1]) if frames else None


if __name__ == '__main__':
    main()
