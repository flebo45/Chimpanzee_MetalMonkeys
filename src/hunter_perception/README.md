# 👁️ Hunter Perception

**Hunter Perception** is the vision processing unit of the Hunter Robot. It combines Deep Learning object detection with probabilistic state estimation to provide a robust, real-time tracking stream of the target (Red Ball).

## 📋 Overview

This package transforms raw video data into semantic target information. It addresses common computer vision challenges such as:
- **Noise**: Smoothing jittery bounding boxes.
- **Occlusion**: Detecting when the target is partially hidden.
- **Data Dropouts**: Predicting the target's position when detection fails temporarily.

The core pipeline runs at the camera's frame rate (approx. 10-30 Hz) and fuses **YOLOv8** detections with a **Kalman Filter**.

---

## ⚙️ Processing Pipeline

1.  **Acquisition**: Receives `sensor_msgs/Image` from Gazebo (`/camera/image_raw`).
2.  **Inference (YOLOv8)**:
    - The image is passed to the `YoloDetector`.
    - It searches for the "Sports Ball" class (COCO ID 32).
    - If multiple balls are found, it selects the **largest** one (assuming it is the closest).
3.  **State Estimation (Kalman Filter)**:
    - **Prediction Step**: The filter predicts the ball's new position based on its previous velocity.
    - **Update Step**:
        - *If detected*: The filter corrects its prediction using the YOLO bounding box center.
        - *If NOT detected*: The filter relies solely on the prediction ("Ghost Tracking").
4.  **Logic & Encoding**:
    - Checks for **Occlusion** by analyzing the bounding box aspect ratio (a sphere should be roughly 1:1).
    - Encodes the state into a custom protocol.
5.  **Publishing**: Sends the data to the Control and Telemetry layers.

---

## 📡 Interfaces & Protocol

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/vision/target` | `geometry_msgs/Point` | **Custom Protocol** (see below). |
| `/vision/is_visible` | `std_msgs/Bool` | Simple flag: `True` if tracking/predicting, `False` if lost. |
| `/vision/debug_image` | `sensor_msgs/Image` | Video feed with bounding boxes and debug text. |

### ⚠️ Custom Protocol: `/vision/target`

To minimize message overhead, we overload the `geometry_msgs/Point` message:

- **`x`**: Horizontal error from image center (pixels). `0` = Centered.
- **`y`**: Vertical error (pixels).
- **`z`**: **Status / Area Payload**:
    - **`z > 0`**: **Valid Detection**. Value is the Bounding Box Area (pixels²). Used for distance estimation.
    - **`z = -1.0`**: **Occluded**. Target is visible but distorted (e.g., half behind a wall).
    - **`0 < z < 0.1`**: **Prediction**. Target is lost; this is the Kalman Filter's best guess.

---

## 🧩 Modules

### 1. `vision_node.py`
The ROS 2 wrapper. It handles subscriptions, synchronizes the pipeline, and manages the "Target Lost" timer (threshold: 4.0 seconds).

### 2. `yolo_detector.py`
A wrapper around the `ultralytics` library.
- **Hardware Acceleration**: Automatically uses CUDA (GPU) if available, otherwise CPU.
- **Heuristic**: Implements the "Largest Area" logic to filter false positives or background objects.

### 3. `kalman_filter.py`
A linear Discrete Kalman Filter (DKF) implementation.
- **Model**: Constant Velocity (CV).
- **State**: `[x, y, vx, vy]`.
- **Measurement**: `[x, y]`.

---

## ⚙️ Configuration

Parameters can be set in `hunter_bringup/config/params.yaml` or via command line.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_path` | `"yolov8n.pt"` | Path to the YOLO weights file. |
| `conf_threshold` | `0.4` | Minimum confidence (0-1) to accept a detection. |
| `target_class_id` | `32` | COCO Class ID to track (32 = Sports Ball). |
| `debug_window` | `true` | Whether to open a local OpenCV window (GUI). |

---

## 💻 Usage

Usually launched via `hunter_bringup`.
Run standalone (for testing vision without the robot moving):

```bash
ros2 run hunter_perception vision_node --ros-args -p debug_window:=true
```

*Note: Requires a running simulation or camera driver publishing to `/camera/image_raw`.*
