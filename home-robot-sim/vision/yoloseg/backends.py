"""Model backends returning (preds, proto) for pipeline.Segmenter.

TorchBackend    Ultralytics .pt weights in PyTorch (PC; needs the ultralytics package).
TensorRTBackend a serialized TensorRT engine (Jetson Nano: TensorRT 8.2; PC: TensorRT 10). Input and outputs live in
                torch CUDA tensors, so no pycuda and no copies through the CPU.
Python 3.6 / torch 1.10 compatible.
"""
import json
import os

import torch


def configure_head(model):
    """Make an Ultralytics segmentation model return (preds, proto) tensors, as the exported ONNX does.

    YOLO26 (end-to-end head): the one2one branch's decoded boxes, scores and mask coefficients for every anchor, without
    the built-in top-k step (its Mod operator needs TensorRT >= 8.5; the Jetson Nano has 8.2). The top-k runs in torch
    (pipeline.topk_detections). Other heads (e.g. YOLOv12) keep their classic output and use NMS. Returns end2end.
    """
    head = model.model[-1]
    end2end = getattr(head, 'one2one_cv2', None) is not None
    if end2end:
        head.end2end = True                       # one2one branch, x1 y1 x2 y2 boxes
        head.postprocess = lambda y: y            # skip the top-k: (B, A, 4 + nc + nm) per anchor
    head.export = True                            # forward returns (preds, proto) only
    return end2end


class TorchBackend(object):
    def __init__(self, weights, device='cuda', half=True):
        from ultralytics import YOLO
        model = YOLO(weights).model
        self.end2end = configure_head(model)
        self.names = [model.names[i] for i in range(len(model.names))]
        self.half = half and str(device).startswith('cuda')
        self.model = model.to(device).eval()
        if self.half:
            self.model.half()
        self.device = device

    def __call__(self, x):
        preds, proto = self.model(x.half() if self.half else x)
        return preds, proto


_DTYPES = None


def _torch_dtype(trt, dtype):
    global _DTYPES
    if _DTYPES is None:
        _DTYPES = {trt.float32: torch.float32, trt.float16: torch.float16, trt.int32: torch.int32, trt.int8: torch.int8,
                   trt.bool: torch.bool}
    return _DTYPES[dtype]


class TensorRTBackend(object):
    """Runs an engine built by build_engine.py. Works with the TensorRT 8.2 binding API and the >= 8.5 tensor API."""

    def __init__(self, engine_path, device='cuda'):
        import tensorrt as trt
        self.trt, self.device = trt, torch.device(device)
        logger = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(logger, '')
        with open(engine_path, 'rb') as f:
            self.engine = trt.Runtime(logger).deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError('Could not load %s: build the engine on this device with this TensorRT version' % engine_path)
        self.context = self.engine.create_execution_context()
        self.stream = torch.cuda.Stream(self.device)
        self.tensor_api = hasattr(self.engine, 'num_io_tensors')      # TensorRT >= 8.5
        self.inputs, self.outputs, self.order = {}, {}, []
        if self.tensor_api:
            for i in range(self.engine.num_io_tensors):
                name = self.engine.get_tensor_name(i)
                is_input = self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT
                self._alloc(name, self.engine.get_tensor_shape(name), self.engine.get_tensor_dtype(name), is_input)
        else:
            for i in range(self.engine.num_bindings):
                name = self.engine.get_binding_name(i)
                self._alloc(name, self.engine.get_binding_shape(i), self.engine.get_binding_dtype(i),
                            self.engine.binding_is_input(i))
        meta = os.path.splitext(engine_path)[0] + '.json'
        self.meta = json.load(open(meta)) if os.path.exists(meta) else {}
        self.names = self.meta.get('names', [])
        self.end2end = self.meta.get('end2end', True)
        self.input_size = tuple(self.input.shape[2:])

    def _alloc(self, name, shape, dtype, is_input):
        tensor = torch.empty(tuple(shape), dtype=_torch_dtype(self.trt, dtype), device=self.device)
        (self.inputs if is_input else self.outputs)[name] = tensor
        self.order.append(name)
        if is_input:
            self.input = tensor

    def __call__(self, x):
        caller = torch.cuda.current_stream(self.device)
        self.stream.wait_stream(caller)                  # x is ready before TensorRT reads it
        with torch.cuda.stream(self.stream):              # a non-default stream: TensorRT avoids extra synchronisation
            self.input.copy_(x.to(self.input.dtype))
            tensors = dict(self.inputs, **self.outputs)
            if self.tensor_api:
                for name in self.order:
                    self.context.set_tensor_address(name, tensors[name].data_ptr())
                ok = self.context.execute_async_v3(self.stream.cuda_stream)
            else:
                ok = self.context.execute_async_v2([tensors[n].data_ptr() for n in self.order], self.stream.cuda_stream)
        if not ok:
            raise RuntimeError('TensorRT inference failed')
        caller.wait_stream(self.stream)                  # the outputs are ready before the caller uses them
        return self.outputs['preds'], self.outputs['proto']


def load_backend(model, device='cuda'):
    """A .engine file -> TensorRTBackend, anything else -> TorchBackend (Ultralytics weights)."""
    if str(model).endswith('.engine'):
        return TensorRTBackend(model, device)
    return TorchBackend(model, device)
