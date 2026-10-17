#!/usr/bin/env bash
# One-time setup of the Jetson Nano (JetPack 4.6.x, L4T R32.6/R32.7, Python 3.6) for detect_service.py:
# GStreamer for the camera, PyTorch 1.10 (NVIDIA's JetPack wheel), torchvision 0.11.1 built from source, and checks.
# TensorRT 8.2 and its Python bindings come with JetPack. Takes about 1 to 1.5 hours (mostly the torchvision build).
#   bash setup_jetson.sh
set -euo pipefail

if ! grep -q "R32" /etc/nv_tegra_release 2>/dev/null; then
  echo "This script is for JetPack 4.x (L4T R32) on a Jetson Nano; found: $(head -1 /etc/nv_tegra_release 2>/dev/null || echo none)"
  exit 1
fi
cd "$(dirname "$0")"
WORK="$HOME/jetson-build"
mkdir -p "$WORK"

echo "== 1/5 system packages (camera via GStreamer, build tools, image libraries for torchvision)"
sudo apt-get update
sudo apt-get install -y python3-pip python3-dev libopenblas-base libopenmpi-dev libomp-dev \
  libjpeg-dev zlib1g-dev libpng-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev \
  python3-gi gir1.2-gstreamer-1.0 gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad v4l-utils fonts-dejavu-core git

echo "== 2/5 Python packages (last versions that support Python 3.6)"
python3 -m pip install --user --upgrade "pip<22"
python3 -m pip install --user "Cython<3" "numpy<1.20" "pillow<9"

echo "== 3/5 PyTorch 1.10.0 for JetPack 4.6 (NVIDIA wheel, see https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048)"
if python3 -c "import torch, sys; sys.exit(0 if torch.__version__.startswith('1.10') and torch.cuda.is_available() else 1)" 2>/dev/null; then
  echo "torch 1.10 with CUDA already installed"
else
  WHEEL="$WORK/torch-1.10.0-cp36-cp36m-linux_aarch64.whl"
  [ -f "$WHEEL" ] || wget -O "$WHEEL" https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl
  python3 -m pip install --user "$WHEEL"
fi

echo "== 4/5 torchvision 0.11.1 from source (no Jetson wheel exists; ~45-60 min)"
if python3 -c "import torchvision, sys; sys.exit(0 if torchvision.__version__.startswith('0.11') else 1)" 2>/dev/null; then
  echo "torchvision 0.11 already installed"
else
  [ -d "$WORK/torchvision" ] || git clone --branch v0.11.1 --depth 1 https://github.com/pytorch/vision "$WORK/torchvision"
  (cd "$WORK/torchvision" && BUILD_VERSION=0.11.1 MAX_JOBS=2 python3 setup.py install --user)
fi

echo "== 5/5 checks"
python3 - <<'EOF'
import torch, torchvision, tensorrt
from torchvision.io import decode_image, encode_jpeg
print('torch', torch.__version__, 'CUDA', torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else '')
print('torchvision', torchvision.__version__, '(image decode/encode available)')
print('TensorRT', tensorrt.__version__)
import gi; gi.require_version('Gst', '1.0'); from gi.repository import Gst; Gst.init(None)
print('GStreamer', Gst.version_string())
EOF
echo "Setup done. Next: copy the ONNX from the PC into models/ and run: python3 build_engine.py models/<name>.onnx"
