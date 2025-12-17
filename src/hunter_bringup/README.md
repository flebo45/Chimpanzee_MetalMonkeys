# 🚀 Hunter Bringup

**Hunter Bringup** is the orchestration package for the Hunter Robot project. It provides the central launch files and configuration parameters necessary to start the entire autonomous system with a single command.

## 📋 Overview

This package serves as the "Master Switch" for the project. Instead of launching the simulation, perception, control, and telemetry nodes individually, `hunter_bringup` coordinates them all, ensuring they start in the correct order and with the correct configuration.

It is responsible for:
1.  **Simulation**: Triggering the Gazebo environment and spawning the robot (`hunter_description`).
2.  **Perception**: Starting the YOLOv8 vision pipeline (`hunter_perception`).
3.  **Control**: Activating the Behavior Tree brain (`hunter_control`).
4.  **Telemetry**: Launching the data streaming bridge (`hunter_telemetry`).
5.  **Utilities**: Starting the battery simulation node.

---

## 📂 Package Structure

```plaintext
hunter_bringup/
├── config/
│   └── params.yaml       # Global configuration parameters (Vision, etc.)
├── launch/
│   └── bringup.launch.py # Main launch file
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 🚀 Launch Files

### `bringup.launch.py`

This is the primary entry point. It performs the following sequence:

1.  **Include `hunter_sim.launch.py`**:
    - Starts Gazebo.
    - Spawns the **Chimpanzee Robot** and the **Red Ball**.
    - Starts the `ros_gz_bridge` to connect ROS 2 topics with Gazebo.
    - Starts the `robot_state_publisher`.

2.  **Start `vision_node`**:
    - Loads the YOLOv8 model.
    - Subscribes to camera images.
    - Publishes target detection data.
    - **Parameters**: Loaded from `config/params.yaml`.

3.  **Start `control_node`**:
    - Initializes the Behavior Tree.
    - Subscribes to sensors and vision.
    - Publishes velocity commands (`/cmd_vel`).

4.  **Start `telemetry_node`**:
    - Connects to the WebSocket dashboard.
    - Streams video and data.

5.  **Start `battery_node`**:
    - Simulates battery drain based on robot movement.

---

## ⚙️ Configuration

### `config/params.yaml`

This file contains runtime parameters for the nodes launched by this package.

```yaml
/vision_node:
  ros__parameters:
    model_path: "yolov8n.pt"   # Path to the YOLO weights file
    conf_threshold: 0.4        # Minimum confidence to accept a detection
    target_class_id: 32        # COCO Class ID (32 = Sports Ball)
    debug_window: true         # Show local OpenCV window (set false for headless)
```

---

## 💻 Usage

To start the entire system (do the following commands otside the `/src` folder):

```bash
colcon build 
source install/setup.bash 
ros2 launch hunter_bringup bringup.launch.py
```
---

## 🔗 Dependencies

This package depends on the other modules in the stack:
- `hunter_description`
- `hunter_perception`
- `hunter_control`
- `hunter_telemetry`
