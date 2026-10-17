"""Torch-only YOLO segmentation pipeline: letterbox -> model -> NMS-free top-k -> masks -> drawing.

Images are uint8 RGB tensors shaped (3, H, W) on any device. Nothing here uses OpenCV; JPEG encoding of the finished
image (encode_jpeg) is the only step that may fall back to it. Compatible with Python 3.6 / torch 1.10 / torchvision 0.11.

Model output contract (TorchBackend and the exported ONNX / TensorRT engine give the same tensors):
    preds  (1, A, 4 + nc + nm)  end-to-end (one2one) head: x1 y1 x2 y2 in input pixels, class scores after sigmoid,
                                mask coefficients; one entry per anchor, no NMS needed (YOLO26).
           (1, 4 + nc + nm, A)  classic head (end2end False, e.g. YOLOv12): x y w h, needs NMS.
    proto  (1, nm, h, w)        mask prototypes at 1/4 of the input resolution.
"""
import time

import torch
import torch.nn.functional as F

# Ultralytics' default colours, RGB.
PALETTE = [(4, 42, 255), (11, 219, 235), (243, 243, 243), (0, 223, 183), (17, 31, 104), (255, 111, 221),
           (255, 68, 79), (204, 237, 0), (0, 243, 68), (189, 0, 255), (0, 180, 255), (221, 0, 186), (0, 255, 255),
           (38, 192, 0), (1, 255, 179), (125, 36, 255), (123, 0, 104), (255, 27, 108), (252, 128, 129), (255, 84, 61)]
FONT_CANDIDATES = ['arial.ttf', '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
                   '/usr/share/fonts/dejavu/DejaVuSans.ttf', 'DejaVuSans.ttf']


def letterbox(image, size, pad_value=114.0):
    """Resize (keeping aspect) and centre-pad a uint8 (3, h, w) image to size (H, W), as Ultralytics' LetterBox does.

    Returns the model input (1, 3, H, W) float in 0..1 on the image's device and (ratio, pad_top, pad_left).
    """
    _, h, w = image.shape
    H, W = size
    r = min(float(H) / h, float(W) / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    x = image.unsqueeze(0).float()
    if (nh, nw) != (h, w):
        x = F.interpolate(x, size=(nh, nw), mode='bilinear', align_corners=False)
    top, left = int(round((H - nh) / 2.0 - 0.1)), int(round((W - nw) / 2.0 - 0.1))
    x = F.pad(x, (left, W - nw - left, top, H - nh - top), value=pad_value)
    return x / 255.0, (r, top, left)


def topk_detections(preds, nc, conf, max_det):
    """End-to-end (one2one) head: the anchors with the best class score, then the best (anchor, class) pairs.

    Same selection as Ultralytics' Detect.postprocess, so YOLO26 needs no NMS. preds is (A, 4 + nc + nm).
    """
    boxes, scores, coeffs = preds[:, :4], preds[:, 4:4 + nc], preds[:, 4 + nc:]
    k = min(max_det, scores.shape[0])
    anchors = scores.max(1)[0].topk(k)[1]
    values, flat = scores[anchors].flatten().topk(k)
    anchor = anchors[torch.div(flat, nc, rounding_mode='floor')]
    keep = values > conf
    anchor = anchor[keep]
    return boxes[anchor], values[keep], (flat % nc)[keep], coeffs[anchor]


def nms_detections(preds, nc, conf, max_det, iou=0.7):
    """Classic head (needs torchvision): x y w h boxes, per-class NMS. preds is (4 + nc + nm, A)."""
    from torchvision.ops import batched_nms
    preds = preds.transpose(0, 1)
    xywh, scores, coeffs = preds[:, :4], preds[:, 4:4 + nc], preds[:, 4 + nc:]
    best, cls = scores.max(1)
    keep = best > conf
    xywh, best, cls, coeffs = xywh[keep], best[keep], cls[keep], coeffs[keep]
    boxes = torch.cat([xywh[:, :2] - xywh[:, 2:] / 2, xywh[:, :2] + xywh[:, 2:] / 2], 1)
    keep = batched_nms(boxes, best, cls, iou)[:max_det]
    return boxes[keep], best[keep], cls[keep], coeffs[keep]


def instance_masks(coeffs, proto, boxes, input_size, info, image_size, chunk=16):
    """Masks (N, h, w) bool at the original image size, as Ultralytics' full-resolution masks: coefficients x
    prototypes, letterbox padding removed, upsampled to the image, then cut to each box (original image pixels)."""
    nm, ph, pw = proto.shape
    H, W = input_size
    h, w = image_size
    if coeffs.shape[0] == 0:
        return torch.zeros((0, h, w), dtype=torch.bool, device=proto.device)
    logits = (coeffs @ proto.reshape(nm, -1)).reshape(-1, ph, pw)
    r, top, left = info
    nh, nw = int(round(h * r)), int(round(w * r))
    s = float(ph) / H                                  # prototypes are 1/4 of the input
    edges = [top * s, (top + nh) * s, left * s, (left + nw) * s]
    xs = torch.arange(w, device=proto.device, dtype=logits.dtype).view(1, 1, w)
    ys = torch.arange(h, device=proto.device, dtype=logits.dtype).view(1, h, 1)
    out = []
    for i in range(0, logits.shape[0], chunk):         # bounds the float (n, h, w) intermediate on the Jetson
        part = logits[i:i + chunk]
        if all(float(e).is_integer() for e in edges):   # padding falls on whole prototype pixels
            part = part[:, int(edges[0]):int(edges[1]), int(edges[2]):int(edges[3])]
        else:                                          # otherwise unpad at the input size
            part = F.interpolate(part[None], size=(H, W), mode='bilinear', align_corners=False)[0]
            part = part[:, top:top + nh, left:left + nw]
        part = F.interpolate(part[None], size=(h, w), mode='bilinear', align_corners=False)[0]
        b = boxes[i:i + chunk, :, None, None]
        inside = (xs >= b[:, 0]) & (xs < b[:, 2]) & (ys >= b[:, 1]) & (ys < b[:, 3])
        out.append((part > 0) & inside)                # logit > 0 is sigmoid > 0.5
    return torch.cat(out)


class Segmenter(object):
    """Runs a backend on uint8 RGB images and returns detections with masks at the image's own resolution."""

    def __init__(self, backend, names, input_size=(384, 640), conf=0.35, max_det=300, end2end=True):
        self.backend, self.names, self.input_size = backend, list(names), tuple(input_size)
        self.conf, self.max_det, self.end2end = conf, max_det, end2end
        self.nc = len(self.names)

    def __call__(self, image):
        """image: uint8 (3, h, w) RGB tensor on the backend's device. Returns a dict of tensors and timings (ms)."""
        sync = image.is_cuda and torch.cuda.synchronize or (lambda: None)
        t0 = time.time()
        x, info = letterbox(image, self.input_size)
        sync()
        t1 = time.time()
        with torch.no_grad():
            preds, proto = self.backend(x)
            sync()
            t2 = time.time()
            preds, proto = preds[0].float(), proto[0].float()
            select = topk_detections if self.end2end else nms_detections
            boxes_in, scores, classes, coeffs = select(preds, self.nc, self.conf, self.max_det)
            r, top, left = info
            h, w = image.shape[1:]
            boxes = boxes_in.clone()
            boxes[:, [0, 2]] = ((boxes[:, [0, 2]] - left) / r).clamp(0, w)
            boxes[:, [1, 3]] = ((boxes[:, [1, 3]] - top) / r).clamp(0, h)
            masks = instance_masks(coeffs, proto, boxes, self.input_size, info, (h, w))
        sync()
        t3 = time.time()
        return dict(boxes=boxes, scores=scores, classes=classes.long(), masks=masks,
                    ms=dict(preprocess=(t1 - t0) * 1000, inference=(t2 - t1) * 1000, postprocess=(t3 - t2) * 1000))

    def describe(self, result):
        """JSON-ready list of detections: class name, confidence, box (original image pixels)."""
        return [dict(cls=self.names[int(c)], conf=round(float(s), 3), box=[round(float(v), 1) for v in b])
                for b, s, c in zip(result['boxes'].tolist(), result['scores'].tolist(), result['classes'].tolist())]


_FONT = None


def _font(size=13):
    global _FONT
    if _FONT is None:
        from PIL import ImageFont
        for path in FONT_CANDIDATES:
            try:
                _FONT = ImageFont.truetype(path, size)
                break
            except (IOError, OSError):
                continue
        else:
            _FONT = ImageFont.load_default()
    return _FONT


def _text_size(font, text):
    if hasattr(font, 'getbbox'):                       # Pillow >= 8
        x0, y0, x1, y1 = font.getbbox(text)
        return x1 - x0, y1 - y0
    return font.getsize(text)                          # older Pillow (JetPack 4.6)


def _labels(image, boxes, labels, colors):
    """White label text on a filled tag in the box colour, above each box (inside it at the top edge). PIL, not OpenCV."""
    import numpy as np
    from PIL import Image, ImageDraw
    pil = Image.fromarray(image.permute(1, 2, 0).numpy())
    canvas, font = ImageDraw.Draw(pil), _font()
    for (x1, y1, x2, y2), text, color in zip(boxes, labels, colors):
        tw, th = _text_size(font, text)
        top = y1 - th - 4 if y1 - th - 4 >= 0 else y1
        canvas.rectangle([x1, top, x1 + tw + 6, top + th + 4], fill=tuple(color))
        canvas.text((x1 + 3, top + 1), text, fill=(255, 255, 255), font=font)
    return torch.from_numpy(np.asarray(pil).copy()).permute(2, 0, 1)


def draw(image, result, names, alpha=0.45, width=2):
    """The final image: masks blended in torch, boxes (torchvision), labels (PIL). Returns uint8 (3, h, w) on CPU."""
    from torchvision.utils import draw_bounding_boxes
    img = image.float()
    order = result['scores'].argsort()                 # most confident drawn last, on top
    colors = [PALETTE[int(c) % len(PALETTE)] for c in result['classes'].tolist()]
    for i in order.tolist():
        m = result['masks'][i]
        color = torch.tensor(colors[i], dtype=img.dtype, device=img.device)[:, None]
        img[:, m] = img[:, m] * (1 - alpha) + color * alpha
    out = img.round().clamp(0, 255).to(torch.uint8).cpu()
    if result['boxes'].shape[0] == 0:
        return out
    boxes = result['boxes'].cpu()
    out = draw_bounding_boxes(out, boxes, colors=colors, width=width)
    labels = ['%s %.2f' % (names[int(c)], float(s)) for c, s in zip(result['classes'].tolist(), result['scores'].tolist())]
    return _labels(out, boxes.round().int().tolist(), labels, colors)


# torchvision.io's image operators exist only when torchvision was built with libjpeg / libpng (setup_jetson.sh installs
# them first; some prebuilt wheels lack them). Without them, Pillow decodes and encodes instead. Never OpenCV.
_TV_IMAGE_IO = None


def _tv_image_io():
    global _TV_IMAGE_IO
    if _TV_IMAGE_IO is None:
        try:
            from torchvision.io import encode_jpeg as tv_encode
            tv_encode(torch.zeros(3, 8, 8, dtype=torch.uint8))
            _TV_IMAGE_IO = True
        except (ImportError, RuntimeError):
            _TV_IMAGE_IO = False
    return _TV_IMAGE_IO


def encode_jpeg(image, quality=88):
    """JPEG bytes of a finished uint8 (3, h, w) CPU image."""
    if _tv_image_io():
        from torchvision.io import encode_jpeg as tv_encode
        return bytes(tv_encode(image.contiguous(), quality=quality).numpy().tobytes())
    import io
    from PIL import Image
    out = io.BytesIO()
    Image.fromarray(image.permute(1, 2, 0).contiguous().numpy()).save(out, format='JPEG', quality=quality)
    return out.getvalue()


def decode_image(data, device):
    """uint8 (3, h, w) RGB tensor on device from PNG/JPEG bytes (torchvision.io, or Pillow; no OpenCV)."""
    import numpy as np
    if _tv_image_io():
        from torchvision.io import decode_image as tv_decode, ImageReadMode
        raw = torch.from_numpy(np.frombuffer(data, dtype=np.uint8).copy())
        return tv_decode(raw, mode=ImageReadMode.RGB).to(device)
    import io
    from PIL import Image
    rgb = np.asarray(Image.open(io.BytesIO(data)).convert('RGB'))
    return torch.from_numpy(rgb.copy()).permute(2, 0, 1).contiguous().to(device)


def read_image(path, device):
    with open(path, 'rb') as f:
        return decode_image(f.read(), device)
