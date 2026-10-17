"""YOLO26 instance segmentation without OpenCV: PyTorch from decoded image to the annotated result image.

Shared by the PC vision service (PyTorch model or TensorRT engine) and the Jetson Nano runner (TensorRT 8.2), so it
stays compatible with JetPack 4.6: Python 3.6, torch 1.10, torchvision 0.11.

    pipeline  letterbox, NMS-free top-k decoding, masks, drawing (all torch / torchvision tensors)
    backends  TorchBackend (Ultralytics weights) and TensorRTBackend (.engine built from export_onnx.py's ONNX)
"""
