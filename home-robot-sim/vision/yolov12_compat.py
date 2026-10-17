"""Run the YOLOv12 authors' checkpoints (github.com/sunsmarterjie/yolov12, e.g. yolov12n-seg.pt) on current Ultralytics.

Ultralytics ships detection weights only for YOLO12 (yolo12n.pt); the segmentation weights come from the authors' release.
Their area-attention layer (AAttn) differs from Ultralytics' port: separate `qk` and `v` convolutions instead of one
`qkv`, and a 5x5 position encoding applied to v before the area split. A pickled checkpoint keeps the authors' layers but
runs Ultralytics' forward(), which fails ('AAttn' object has no attribute 'qkv'). install() makes AAttn run the authors'
forward for modules that have `qk`; Ultralytics' own models are unaffected. The attention itself is computed with
torch's scaled_dot_product_attention, which is the authors' non-flash path (softmax(q k^T / sqrt(d)) v) fused.
Adapted from the authors' AGPL-3.0 code (ultralytics/nn/modules/block.py, AAttn.forward).
"""
import torch.nn.functional as F
from ultralytics.nn.modules import block


def _authors_forward(self, x):
    B, C, H, W = x.shape
    N = H * W
    qk = self.qk(x).flatten(2).transpose(1, 2)
    v = self.v(x)
    pp = self.pe(v)
    v = v.flatten(2).transpose(1, 2)
    if self.area > 1:
        qk = qk.reshape(B * self.area, N // self.area, C * 2)
        v = v.reshape(B * self.area, N // self.area, C)
        B, N, _ = qk.shape
    q, k = qk.split([C, C], dim=2)
    heads = lambda t: t.view(B, N, self.num_heads, self.head_dim).transpose(1, 2)   # (B, heads, N, head_dim)
    x = F.scaled_dot_product_attention(heads(q), heads(k), heads(v)).transpose(1, 2)  # (B, N, heads, head_dim)
    if self.area > 1:
        x = x.reshape(B // self.area, N * self.area, C)
        B, N, _ = x.shape
    x = x.reshape(B, H, W, C).permute(0, 3, 1, 2)
    return self.proj(x + pp)


def install():
    if getattr(block.AAttn, '_authors_compat', False):
        return
    ultralytics_forward = block.AAttn.forward

    def forward(self, x):
        return _authors_forward(self, x) if hasattr(self, 'qk') else ultralytics_forward(self, x)

    block.AAttn.forward = forward
    block.AAttn._authors_compat = True
