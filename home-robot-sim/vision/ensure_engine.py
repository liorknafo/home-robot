"""Find or build this PC's TensorRT engine for the vision service; prints ENGINE=<path> on stdout.

An engine only runs on the GPU and TensorRT version that built it, so the file name carries both:
    jetson/models/<weights stem>-<H>x<W>-pc-trt<TensorRT major>-<gpu>.engine
A missing or unloadable engine is built on the fly: the ONNX is exported first if needed (export_onnx.py), then the
engine (jetson/build_engine.py, about 1-2 minutes, first time only). Progress goes to stderr. Exits 1 when TensorRT or a
CUDA GPU is missing or a step fails; Scripts/start-vision.cmd then falls back to PyTorch.

    vision\\.venv\\Scripts\\python vision\\ensure_engine.py [--weights yolo26n-seg.pt] [--imgsz 384 640]
"""
import argparse, contextlib, re, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
MODELS = HERE.parent / 'jetson' / 'models'


def engine_loads(path, trt):
    logger = trt.Logger(trt.Logger.ERROR)
    trt.init_libnvinfer_plugins(logger, '')
    return trt.Runtime(logger).deserialize_cuda_engine(path.read_bytes()) is not None


def ensure(weights, imgsz):
    import tensorrt as trt
    import torch
    if not torch.cuda.is_available():
        raise RuntimeError('no CUDA GPU')
    gpu = re.sub(r'[^a-z0-9]+', '', re.sub(r'nvidia|geforce', '', torch.cuda.get_device_name(0).lower()))   # e.g. rtx4090
    stem = '%s-%dx%d' % (Path(weights).stem, imgsz[0], imgsz[1])
    engine = MODELS / ('%s-pc-trt%s-%s.engine' % (stem, trt.__version__.split('.')[0], gpu))
    if engine.exists() and engine_loads(engine, trt):
        return engine
    print('No usable TensorRT engine for this PC yet: building %s (first time only, about 1-2 minutes)...' % engine.name)
    onnx = MODELS / (stem + '.onnx')
    if not onnx.exists() or not onnx.with_suffix('.json').exists():
        sys.path.insert(0, str(HERE))
        from export_onnx import export
        export(weights, imgsz, out=MODELS)
    sys.path.insert(0, str(HERE.parent / 'jetson'))
    from build_engine import build
    build(str(onnx), str(engine))
    if not engine_loads(engine, trt):
        raise RuntimeError('the new engine does not load')
    return engine


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--weights', default='yolo26n-seg.pt')
    parser.add_argument('--imgsz', type=int, nargs=2, default=[384, 640], metavar=('H', 'W'))
    args = parser.parse_args()
    try:
        with contextlib.redirect_stdout(sys.stderr):         # stdout carries only the engine path
            engine = ensure(args.weights, args.imgsz)
    except (ImportError, RuntimeError, OSError, SystemExit) as exc:
        print('TensorRT engine not available (%s)' % (exc or type(exc).__name__), file=sys.stderr)
        return 1
    print('ENGINE=%s' % engine)      # TensorRT's native logger may also write to stdout: callers read this line
    return 0


if __name__ == '__main__':
    sys.exit(main())
