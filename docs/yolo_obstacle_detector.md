# RE-RASSOR YOLO Obstacle Detector

ROS2 package for real-time obstacle detection and distance estimation using YOLO.

## Overview

This node subscribes to the rover's camera feed, runs YOLO inference on each
frame, and publishes detected obstacles as a `YoloMsgArray` with bounding box
coordinates, class labels, and distance to each obstacle in metres.

## Pipeline

/camera/color/image_raw   → YOLO inference → /re_rassor/vision/yolo_detections
/camera/depth/image_raw   → depth sampling → (distance per detection)
/camera/depth/camera_info → intrinsics     → (angle per detection)

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | Input color stream |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Input depth stream |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |
| `/re_rassor/vision/yolo_detections` | `re_rassor_interfaces/YoloMsgArray` | Detection output |
| `/re_rassor/vision/annotated_image` | `sensor_msgs/Image` | Debug image with boxes drawn |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_path` | `yolov8n.pt` | Path to YOLO weights file |
| `confidence` | `0.4` | Detection confidence threshold (0.0-1.0) |
| `img_size` | `640` | YOLO input resolution (lower = faster) |
| `patch_size` | `7` | Depth sampling patch size in pixels |
| `min_depth_m` | `0.3` | Minimum valid depth in metres |
| `max_depth_m` | `6.0` | Maximum valid depth in metres |

## Installation

Install the ultralytics dependency:

```bash
pip3 install ultralytics --break-system-packages
```

Build the package:

```bash
colcon build --packages-select re_rassor_interfaces re_rassor_yolo_detector
source install/setup.bash
```

## Running

**With default YOLOv8n weights (downloads automatically on first run):**

```bash
ros2 run re_rassor_yolo_detector yolo_detector
```

**With custom lunar terrain weights:**

```bash
ros2 run re_rassor_yolo_detector yolo_detector \
    --ros-args -p model_path:=/path/to/best.pt -p confidence:=0.2
```

**Via the full rover launch file:**

```bash
ros2 launch re_rassor_bringup re_rassor_full.launch.py
```

## Weights

The default `yolov8n.pt` weights are pretrained on the COCO dataset and detect
80 common object classes. For lunar terrain obstacle detection, use the custom
weights trained on lunar surface imagery.

Custom weights are not included in the repository. To use them:

1. Obtain `best.pt` from the team's shared drive or training output
2. Copy it to the rover:
```bash
   scp best.pt ubuntu@<rover-ip>:~/best.pt
```
3. Run the node with the custom weights:
```bash
   ros2 run re_rassor_yolo_detector yolo_detector \
       --ros-args -p model_path:=/home/ubuntu/best.pt -p confidence:=0.2
```

## Test Utilities

Test scripts are provided in the `scripts/` folder for offline development.

**Static image publisher** — publishes a single image at 10 Hz:
```bash
python3 src/re_rassor_yolo_detector/scripts/image_publisher.py
```

**Fake depth publisher** — publishes a flat 2.0m depth for testing sensor fusion:
```bash
python3 src/re_rassor_yolo_detector/scripts/fake_depth_publisher.py
```

**Video/folder publisher** — cycles through a folder of images:
```bash
python3 src/re_rassor_yolo_detector/scripts/video_publisher.py
```

See each script for configuration options such as `IMAGE_PATH`, `FAKE_DEPTH_METRES`, and `IMAGE_FOLDER`.

## Output Format

Each detection in `YoloMsgArray` contains:

| Field | Type | Description |
|-------|------|-------------|
| `name` | string | Detected class label (e.g. "rock") |
| `x, y` | float64 | Bounding box centre in pixels |
| `width, height` | float64 | Bounding box size in pixels |
| `norm_x, norm_y` | float64 | Normalised centre (0.0-1.0) |
| `norm_width, norm_height` | float64 | Normalised size (0.0-1.0) |
| `min_angle, max_angle` | float64 | Horizontal angular extent in radians |
| `distance` | float64 | Distance to obstacle in metres (-1.0 if unavailable) |

## Important: Color Stream Requirement

This node requires `/camera/color/image_raw`. In the full rover launch file
`enable_color` is set to `true` to support this node. On Le Potato (USB 2.0)
this may cause bandwidth issues. If color is disabled, the node will launch
but produce no detections.

## Notes

- On the Raspberry Pi 4, use `img_size:=320` for faster inference
- `distance: -1.0` means no valid depth data was available for that detection
- On first run, `yolov8n.pt` is downloaded automatically (~6MB).
  If the rover has no internet access, manually copy the weights file
  to the rover and specify the full path via the `model_path` parameter
- The Astra Pro color and depth streams may cause USB bandwidth issues on
  Le Potato (USB 2.0) — test before enabling both streams simultaneously
