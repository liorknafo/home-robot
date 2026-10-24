# YOLO26 segmentation on the Jetson Nano with TensorRT

This guide gets the robot's object recognition running on the Jetson Nano. It is the same model and pipeline that the simulator dashboard
runs on the PC. The model is YOLO26n-seg (boxes, masks and labels for COCO's 80 classes). It runs through a TensorRT FP16 engine, and the
camera image goes through PyTorch only (no OpenCV).

```
PC (Windows, vision venv)                      Jetson Nano (JetPack 4.6, Python 3.6)
-------------------------                      ------------------------------------------------------------
vision/export_onnx.py                          jetson/build_engine.py            (once, ~5-15 min)
  yolo26n-seg.pt -> models/X.onnx + X.json  -->  X.onnx -> models/X.engine (TensorRT 8.2, FP16)
                                               jetson/detect_service.py
                                                 camera (GStreamer) -> torch CUDA tensor -> letterbox (torch)
                                                 -> TensorRT engine -> top-k + masks (torch) -> draw (torch/torchvision)
                                                 -> JPEG -> http://<jetson>:8772/
```

## Why it is built this way

- **No Ultralytics on the Jetson.** Ultralytics needs Python 3.7 or newer, and JetPack 4.6 has Python 3.6. The Nano only runs TensorRT plus a
  small torch package (`vision/yoloseg`).
- **No built-in end-to-end export.** Ultralytics' own YOLO26 export adds a top-k step with a `Mod` operator. TensorRT older than 8.5 cannot
  parse it, and the Nano has 8.2.
  - `export_onnx.py` exports the NMS-free (one2one) head's raw outputs instead: boxes, class scores, mask coefficients and mask
    prototypes.
  - `yoloseg.pipeline.topk_detections` then does the same top-k in torch. No NMS is needed.
- **No OpenCV until the final image exists.**
  - Frames come from GStreamer (or HTTP or image files) as numpy buffers and go straight into torch CUDA tensors.
  - Letterbox, top-k, masks and drawing are torch and torchvision. The label text uses PIL.
  - The JPEG is encoded with torchvision, or Pillow if torchvision lacks image support.
- **Checked on the PC against Ultralytics** (60 simulator frames):
  - Detections are identical.
  - Boxes are within 0.6 px.
  - Mask IoU is 1.000.
  - The TensorRT engine output matches PyTorch.

## What you need

- **Jetson Nano 4 GB** (B01 or A02, the same SD card image) with **JetPack 4.6.x** (L4T R32.6 / R32.7) on a 32 GB+ SD card, connected
  to the same network as the PC. If a freshly written card shows nothing but the power LED, the board's QSPI bootloader predates
  JetPack 4.5: flash it once with SDK Manager (Force Recovery: jumper A02 J40 pins 3-4, or B01 J50 pins 9-10, at power-on).
- **The robot camera:**
  - USB UVC camera (the OV2735 module, 1080p): `--source usb`.
  - A CSI camera supported by JetPack (IMX219 / IMX477) also works: `--source csi`. The OV5647 Pi camera is not supported by stock
    JetPack 4.6.
- **On the PC:** the vision venv `home-robot-sim\vision\.venv`. `Scripts\start-vision.cmd` creates it. The export also needs `onnx` (plus
  optionally `onnxslim` and `onnxruntime`):
  `home-robot-sim\vision\.venv\Scripts\python -m pip install onnx onnxslim onnxruntime`.

Commands marked **PC** run in PowerShell from `D:\Coding_projects\home-robot`. Commands marked **Jetson** run in a terminal on the Nano
(or over `ssh`).

## Step 1 — Prepare the Jetson (power mode and swap)

**Jetson:**

```bash
sudo nvpmodel -m 0          # 10 W "MAXN" mode
sudo jetson_clocks          # fixed maximum clocks (repeat after each reboot, or see step 9)
# 4 GB swap file: the torchvision build needs more than the Nano's 4 GB RAM
sudo fallocate -l 4G /var/swapfile && sudo chmod 600 /var/swapfile && sudo mkswap /var/swapfile && sudo swapon /var/swapfile
echo '/var/swapfile swap swap defaults 0 0' | sudo tee -a /etc/fstab
```

Check: `free -h` shows about 4 GB of swap.

## Step 2 — Copy the code to the Jetson

The Jetson needs two folders, in the same layout as on the PC: `home-robot-sim/vision/yoloseg` and `home-robot-sim/jetson`.

**PC** (replace `robot` with the Jetson's user name and `jetson.local` with its name or IP address):

```bash
ssh robot@jetson.local "mkdir -p ~/home-robot-sim/vision"
scp -r home-robot-sim\vision\yoloseg robot@jetson.local:~/home-robot-sim/vision/
scp -r home-robot-sim\jetson robot@jetson.local:~/home-robot-sim/
```

The `jetson` folder may already contain the PC's own engine (`*-pc-trt*.engine`, used by the dashboard). It is useless on the Nano and
safe to delete there.

## Step 3 — Install PyTorch, torchvision and the camera libraries (once, about 1 to 1.5 hours)

**Jetson:**

```bash
cd ~/home-robot-sim/jetson
bash setup_jetson.sh
```

The script installs:

- **GStreamer** and the PyGObject bindings, for the camera.
- **Build tools and image libraries** (`libjpeg-dev`, `libpng-dev`), so torchvision can decode and encode images.
- **NVIDIA's PyTorch 1.10.0 wheel for JetPack 4.6** (Python 3.6).
- **torchvision 0.11.1**, built from source. There is no Jetson wheel, so this takes 45-60 minutes.

It ends with a check. A good result looks like this:

```
torch 1.10.0 CUDA True NVIDIA Tegra X1
torchvision 0.11.1 (image decode/encode available)
TensorRT 8.2.1.8
GStreamer GStreamer 1.14.5
```

If the wheel download fails, take the current JetPack 4.6 / Python 3.6 link for torch 1.10 from NVIDIA's
[PyTorch for Jetson](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048) thread. Save the wheel as
`~/jetson-build/torch-1.10.0-cp36-cp36m-linux_aarch64.whl` and run the script again. It skips the steps that are already done.

## Step 4 — Export the model to ONNX (PC)

**PC:**

```bash
home-robot-sim\vision\.venv\Scripts\python home-robot-sim\vision\export_onnx.py
```

This writes `home-robot-sim\jetson\models\yolo26n-seg-384x640.onnx` (11 MB) and `yolo26n-seg-384x640.json`, which holds the class names
and input size. It then prints the following checks; all of them should pass:

- **Outputs:** `preds [1, 5040, 116]` and `proto [1, 32, 96, 160]`.
- **TensorRT 8.2 check:** none of the operators it cannot parse.
- **ONNX Runtime vs PyTorch:** max difference about 1e-3 (boxes, in pixels) and 1e-5 (masks).

Options:

- `--imgsz H W` sets the model input. The default is 384 x 640, which fits the 16:9 camera with little padding. Use multiples of 32;
  smaller is faster, for example `--imgsz 320 576`.
- `--weights` takes another YOLO26 `-seg` model, for example a fine-tuned one.

Copy both files to the Jetson. **PC:**

```bash
scp home-robot-sim\jetson\models\yolo26n-seg-384x640.onnx home-robot-sim\jetson\models\yolo26n-seg-384x640.json robot@jetson.local:~/home-robot-sim/jetson/models/
```

## Step 5 — Build the TensorRT engine (Jetson, once per model, about 5-15 minutes)

**Jetson:**

```bash
cd ~/home-robot-sim/jetson
python3 build_engine.py models/yolo26n-seg-384x640.onnx
```

This writes `models/yolo26n-seg-384x640.engine` (FP16) and copies the `.json` next to it.

- **Engines are not portable.** Build the engine on the Nano itself and rebuild it after a JetPack or TensorRT upgrade.
- **Out of memory?** Add `--workspace 512`, or close the desktop with `sudo systemctl isolate multi-user.target`.
- **Command-line equivalent:**
  `/usr/src/tensorrt/bin/trtexec --onnx=models/yolo26n-seg-384x640.onnx --saveEngine=models/yolo26n-seg-384x640.engine --fp16 --workspace=1024`.
  Then copy the `.json` yourself.

## Step 6 — Check the camera (Jetson)

```bash
v4l2-ctl --list-devices                               # the USB camera, usually /dev/video0
v4l2-ctl -d /dev/video0 --list-formats-ext            # look for MJPG 1280x720 30 fps
gst-launch-1.0 -e v4l2src device=/dev/video0 num-buffers=60 ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegdec ! fakesink
```

- The last command should end with `Got EOS` after about 2 seconds.
- Most UVC cameras only reach 720p at 30 fps in MJPG, so use `--mjpeg` below. If yours lists raw YUYV at the size you want, leave
  `--mjpeg` out.
- For a CSI camera, test with:

  ```bash
  gst-launch-1.0 -e nvarguscamerasrc num-buffers=60 ! 'video/x-raw(memory:NVMM),width=1280,height=720' ! fakesink
  ```

## Step 7 — Measure the speed (Jetson)

```bash
python3 detect_service.py --engine models/yolo26n-seg-384x640.engine --source usb --mjpeg --benchmark 200
python3 detect_service.py --engine models/yolo26n-seg-384x640.engine --source usb --mjpeg --benchmark 200 --no-draw
```

Each command prints the median preprocess, inference (TensorRT), postprocess (top-k and masks) and total times, and the frames per
second. The first includes drawing and JPEG encoding, as the live view does. The second, `--no-draw`, is detections only, which is what
the robot's controller would use.

- **Expected speed:** not measured on a Nano yet. For reference, YOLOv8n detection runs at about 19 fps in FP16 at 640 px on the
  original Nano ([Q-engineering](https://github.com/Qengineering/YoloV8-TensorRT-Jetson_Nano)). YOLO26n-seg at 384 x 640 also produces
  masks. Expect roughly 8-15 fps; your benchmark is the real number.
- **On the PC** (RTX 4090, TensorRT 11, same scripts): 4.6 ms inference, 145 fps detections only, 78 fps with drawing.

## Step 8 — Run it live (Jetson)

```bash
python3 detect_service.py --engine models/yolo26n-seg-384x640.engine --source usb --mjpeg
```

Open **`http://<jetson-ip>:8772/`** in a browser on the PC or a phone on the same network. It shows the live camera with masks, boxes and
labels, the timings and the frame rate. Programs can read the detections from `http://<jetson-ip>:8772/vision/detections.json`. Every
5 seconds the Jetson terminal prints the frame rate and the last frame's timings.

Useful options:

| Option | Meaning |
|---|---|
| `--source usb` / `csi` / `gst` | USB camera (`--device /dev/video0`), CSI camera (`--sensor-id 0`), or your own pipeline (`--gst "... ! appsink name=sink"` with RGBA output) |
| `--source http://...` | Poll an image URL (see "Testing without a camera") |
| `--source 'frames/*.png'` | Loop over image files |
| `--cam-width/--cam-height/--cam-fps` | Camera mode, default 1280 x 720 at 30 |
| `--width/--height` | Frame size handed to the model, default 640 x 360 (GStreamer scales). Masks and boxes are at this size |
| `--conf` | Confidence threshold, default 0.35 |
| `--no-draw` | Detections JSON only: no drawing or JPEG, fastest |
| `--port` | Web port, default 8772 |

## Testing without a camera

- **From the simulator on the PC:** the PC dashboard serves the simulated robot camera.
  1. Start the autonomy service so other computers can reach it. **PC:**

     ```bash
     .\home-robot-sim\Scripts\start-autonomy.cmd --host 0.0.0.0
     ```

     Allow it through the Windows firewall when asked, and click Start Mapping in the dashboard.
  2. **Jetson:**

     ```bash
     python3 detect_service.py --engine models/yolo26n-seg-384x640.engine --source http://<PC-IP>:8771/api/camera.png
     ```

  This image carries the dashboard's dark "Low obstacles" strip at the top.
- **From saved frames:** copy a few frames from `home-robot-sim\Saved\Stereo\run_*\left_*.png` to the Jetson's `jetson/frames/`, then use
  `--source 'frames/*.png'`.

## Step 9 — Start it at boot (optional, Jetson)

```bash
sudo tee /etc/systemd/system/robot-vision.service >/dev/null <<EOF
[Unit]
Description=Robot camera object recognition (YOLO26 TensorRT)
After=network-online.target
[Service]
User=$USER
WorkingDirectory=$HOME/home-robot-sim/jetson
ExecStartPre=/usr/bin/sudo /usr/bin/jetson_clocks
ExecStart=/usr/bin/python3 detect_service.py --engine models/yolo26n-seg-384x640.engine --source usb --mjpeg
Restart=on-failure
[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload && sudo systemctl enable --now robot-vision
journalctl -u robot-vision -f        # its output
```

`ExecStartPre` needs password-less sudo for `jetson_clocks`. Remove that line if you don't want it.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `TensorRT 8.2.x could not parse ...onnx` | Re-export with `--opset 11`, check that `export_onnx.py` reported "TensorRT 8.2 check: none of ...", and copy the new ONNX |
| `Could not load ...engine` | The engine was built on another device or TensorRT version: run `build_engine.py` on this Nano |
| `PyTorch has no CUDA here` | A CPU-only torch is installed (for example from plain `pip install torch`): `pip3 uninstall torch` and rerun `setup_jetson.sh` |
| setup check says no image decode/encode, or `No such operator image::decode_image` | torchvision was built without libjpeg/libpng. The pipeline falls back to Pillow (works, a little slower). To fix it: `sudo apt-get install libjpeg-dev libpng-dev`, delete `~/jetson-build/torchvision`, rerun `setup_jetson.sh` |
| `GStreamer could not start` / no frames | Check step 6, then set `--cam-width/--cam-height/--cam-fps` to a mode the camera lists. With MJPG modes, add `--mjpeg` |
| Low frame rate | `sudo nvpmodel -m 0 && sudo jetson_clocks`, use `--no-draw`, or export a smaller `--imgsz` (for example 320 576) and rebuild the engine |
| The engine build is killed | Out of memory: add the swap file (step 1), close the desktop, use `--workspace 512` |

## Files

| File | Where it runs | What it does |
|---|---|---|
| `vision/yoloseg/pipeline.py` | PC and Jetson | Letterbox, NMS-free top-k, masks, drawing, image decode/encode. Torch / torchvision only |
| `vision/yoloseg/backends.py` | PC and Jetson | `TensorRTBackend` (TensorRT 8.2 binding API and the 8.5+ tensor API, buffers in torch CUDA tensors) and `TorchBackend` (Ultralytics weights, PC) |
| `vision/yoloseg/server.py` | PC and Jetson | Web view `/`, `/vision/frame.jpg`, `/vision/detections.json` |
| `vision/export_onnx.py` | PC | YOLO26-seg to ONNX (opset 12, TensorRT 8.2 check, ONNX Runtime check) |
| `jetson/setup_jetson.sh` | Jetson | One-time installation |
| `jetson/build_engine.py` | Jetson (also PC) | ONNX to TensorRT engine (FP16 where supported) |
| `jetson/detect_service.py` | Jetson (also PC) | Camera to engine to live view and detections, plus `--benchmark` |
| `vision/vision_service.py` | PC | The dashboard's version: simulator frames with the same pipeline. `Scripts\start-vision.cmd` (and `launch.cmd`) run it on a TensorRT engine for the PC's GPU by default |
| `vision/ensure_engine.py` | PC | Finds this PC's engine (`jetson/models/*-pc-trt<version>-<gpu>.engine`) or builds it on the fly: ONNX export plus engine build, about 1-2 minutes the first time |

## What has and has not been verified

These have been verified on the PC:

- The torch pipeline reproduces Ultralytics' predictions: identical detections, mask IoU 1.000.
- The ONNX contains no operators TensorRT 8.2 rejects, and ONNX Runtime matches PyTorch.
- A TensorRT 11 engine built by `build_engine.py` matches PyTorch, run through `TensorRTBackend` and `detect_service.py` (file, HTTP
  and benchmark modes).
- The pipeline, drawing, JPEG and web server run unchanged under **Python 3.6.13, torch 1.10.0, torchvision 0.11.1 and Pillow 8.4**, the
  JetPack 4.6 versions (CPU), with identical results. A static check confirms the Jetson code needs Python 3.5 or newer.

These have **not** been tested yet, because they need the Nano itself:

- TensorRT 8.2 parsing and building the engine.
- `TensorRTBackend`'s TensorRT 8.2 binding-API path.
- The GStreamer camera sources.
- The Nano's actual speed.

Steps 5 to 7 are where these show up. Report any error from them and it can be fixed.
