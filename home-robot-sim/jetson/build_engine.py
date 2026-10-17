"""Build a TensorRT engine from export_onnx.py's ONNX. Run it ON the device that will use the engine: an engine only works
with the GPU and TensorRT version it was built with (Jetson Nano: JetPack 4.6, TensorRT 8.2; also TensorRT 10+ on a PC).

    python3 build_engine.py models/yolo26n-seg-384x640.onnx            # -> models/yolo26n-seg-384x640.engine (FP16)

Copies the ONNX's .json (class names, input size) next to the engine; detect_service.py reads both. Takes several
minutes on the Nano. Python 3.6 compatible. Command-line equivalent on the Nano:
    /usr/src/tensorrt/bin/trtexec --onnx=models/X.onnx --saveEngine=models/X.engine --fp16 --workspace=1024
"""
import argparse
import os
import shutil
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('onnx')
    parser.add_argument('--out', help='engine path (default: next to the ONNX, .engine)')
    parser.add_argument('--fp32', action='store_true', help='keep FP32 (default FP16: about 2x faster on the Nano)')
    parser.add_argument('--workspace', type=int, default=1024, help='builder workspace in MB (the Nano shares 4 GB with the CPU)')
    args = parser.parse_args()
    build(args.onnx, args.out, args.fp32, args.workspace)


def build(onnx_path, out=None, fp32=False, workspace=1024):
    """Builds and writes the engine (plus the class-name .json next to it); returns the engine path."""
    import tensorrt as trt
    out = out or os.path.splitext(onnx_path)[0] + '.engine'
    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    major = int(trt.__version__.split('.')[0])
    flags = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH) if major < 10 else 0   # explicit batch is default in 10+
    network = builder.create_network(flags)
    onnx_parser = trt.OnnxParser(network, logger)
    with open(onnx_path, 'rb') as f:
        if not onnx_parser.parse(f.read()):
            for i in range(onnx_parser.num_errors):
                print(onnx_parser.get_error(i))
            sys.exit('TensorRT %s could not parse %s' % (trt.__version__, onnx_path))
    config = builder.create_builder_config()
    if hasattr(config, 'set_memory_pool_limit'):                         # TensorRT >= 8.4
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, workspace << 20)
    else:                                                                 # TensorRT 8.2
        config.max_workspace_size = workspace << 20
    has_flag = hasattr(trt.BuilderFlag, 'FP16')        # gone in TensorRT 11 (strongly typed: precision from the ONNX)
    fp16 = not fp32 and has_flag and getattr(builder, 'platform_has_fast_fp16', True)
    if fp16:
        config.set_flag(trt.BuilderFlag.FP16)
    elif not fp32 and not has_flag:
        print('TensorRT %s has no FP16 builder flag (precision follows the ONNX): building FP32' % trt.__version__)
    for i in range(network.num_inputs):
        print('input  %s %s' % (network.get_input(i).name, tuple(network.get_input(i).shape)))
    for i in range(network.num_outputs):
        print('output %s %s' % (network.get_output(i).name, tuple(network.get_output(i).shape)))
    print('Building %s engine with TensorRT %s (several minutes on a Jetson Nano)...' % ('FP16' if fp16 else 'FP32', trt.__version__))
    start = time.time()
    engine = builder.build_serialized_network(network, config)
    if engine is None:
        sys.exit('Engine build failed (see the TensorRT messages above)')
    with open(out, 'wb') as f:
        f.write(engine)
    meta = os.path.splitext(onnx_path)[0] + '.json'
    if os.path.exists(meta) and os.path.abspath(meta) != os.path.abspath(os.path.splitext(out)[0] + '.json'):
        shutil.copy(meta, os.path.splitext(out)[0] + '.json')
    print('wrote %s (%.1f MB) in %.0f s' % (out, os.path.getsize(out) / 1e6, time.time() - start))
    return out


if __name__ == '__main__':
    main()
