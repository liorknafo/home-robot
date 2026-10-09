"""RGB-only semantic segmentation experiment; no navigation or motor interface.

Both models receive camera pixels only. Softmax values are model scores, not
calibrated probabilities of traversability. A 'floor' label does not prove a
surface is clear, level, solid or within the robot's climbing capability.
"""
from pathlib import Path
import time
import cv2
import numpy as np


class FCNSegmenter:
    def __init__(self, directory):
        directory = Path(directory)
        self.net = cv2.dnn.readNetFromONNX(str(directory / 'fcn_resnet18.onnx'))
        self.labels = (directory / 'classes.txt').read_text().splitlines()
        self.floor_id = self.labels.index('floor')

    def predict(self, rgb):
        started = time.perf_counter()
        x = cv2.resize(rgb, (512, 400)).astype(np.float32) / 255
        x = (x - np.array([.485, .456, .406], np.float32)) / np.array([.229, .224, .225], np.float32)
        self.net.setInput(x.transpose(2, 0, 1)[None])
        logits = self.net.forward()[0].transpose(1, 2, 0)
        logits = cv2.resize(logits, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_LINEAR)
        return decode(logits, self.floor_id, started)


class SegFormerSegmenter:
    def __init__(self, directory):
        import torch
        from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation
        self.torch = torch
        torch.set_num_threads(4)
        self.processor = SegformerImageProcessor.from_pretrained(directory, local_files_only=True)
        self.model = SegformerForSemanticSegmentation.from_pretrained(
            directory, local_files_only=True, use_safetensors=True).eval()
        self.labels = [self.model.config.id2label[i] for i in range(self.model.config.num_labels)]
        self.floor_id = self.labels.index('floor')

    def predict(self, rgb):
        from PIL import Image
        started = time.perf_counter()
        inputs = self.processor(images=Image.fromarray(rgb), return_tensors='pt')
        with self.torch.inference_mode():
            logits = self.model(**inputs).logits
            logits = self.torch.nn.functional.interpolate(
                logits, size=rgb.shape[:2], mode='bilinear', align_corners=False)
        return decode(logits[0].permute(1, 2, 0).numpy(), self.floor_id, started)


def decode(logits, floor_id, started):
    scores = np.exp(logits - logits.max(axis=2, keepdims=True))
    scores /= scores.sum(axis=2, keepdims=True)
    return dict(labels=logits.argmax(axis=2).astype(np.uint8),
                floor_score=scores[:, :, floor_id], confidence=scores.max(axis=2),
                elapsed_ms=1000 * (time.perf_counter() - started))


def evidence_view(rgb, prediction, floor_id, minimum_score=.70):
    """Display only: green floor prediction, amber other, grey uncertain/dark.

    No pixel is converted to a safe-space or metric obstacle claim here.
    Darkness is explicitly unknown even when the network is confident.
    """
    bright = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY) >= 12
    confident = (prediction['confidence'] >= minimum_score) & bright
    floor = (prediction['labels'] == floor_id) & confident
    other = (prediction['labels'] != floor_id) & confident
    colors = np.full_like(rgb, [132, 141, 155])
    colors[floor] = [26, 190, 120]
    colors[other] = [239, 157, 45]
    return np.clip(.65 * rgb + .35 * colors, 0, 255).astype(np.uint8)
