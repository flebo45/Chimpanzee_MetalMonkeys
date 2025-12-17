# 🐵 Chimpanzee MetalMonkeys - Hunter Robot Stack

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-34495e?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Simulation-Gazebo-orange?logo=gazebo&logoColor=white)
![Python](https://img.shields.io/badge/Language-Python_3.10-blue?logo=python&logoColor=white)
![YOLOv8](https://img.shields.io/badge/AI-YOLOv8-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

## 📋 Project Overview

**Chimpanzee Robot** is a comprehensive ROS 2 autonomous navigation and tracking system designed to simulate a mobile robot capable of detecting, tracking, and chasing a dynamic target (a Red Ball) in a physics-based environment.

This project demonstrates a full-stack robotics pipeline, integrating:
- **Deep Learning Perception**: Real-time object detection using YOLOv8.
- **State Estimation**: Kalman Filtering for robust tracking and occlusion handling.
- **Behavioral Control**: A modular Behavior Tree (using `py_trees`) for decision making.
- **Simulation**: High-fidelity simulation in Gazebo with ROS 2 integration.
- **Telemetry**: Real-time data streaming and video feed to a web dashboard via WebSockets.

---

## 🏗️ System Architecture

The system is composed of four primary ROS 2 packages, each handling a specific layer of the robotic stack:

### 1. Perception Layer (`hunter_perception`)
Acts as the robot's eyes. It processes raw camera feeds to extract semantic information.
- **YOLOv8 Detector**: Identifies the target (Red Ball) in the image.
- **Kalman Filter**: Estimates the target's position and velocity, allowing the robot to "hallucinate" the target's location during temporary occlusions ("Ghost Tracking").
- **Occlusion Logic**: Analyzes bounding box aspect ratios to detect partial occlusions.

### 2. Control Layer (`hunter_control`)
Acts as the robot's brain. It uses a **Behavior Tree** to prioritize actions based on sensor data.
- **Safety First**: High-priority nodes prevent collisions (`EvasiveManeuver`).
- **Tracking**: Uses PID controllers to chase the target (`TrackReal`) or its predicted position (`TrackGhost`).
- **Recovery**: specialized behaviors for when the target is lost or too close (`SpinSearch`, `BlindBackUp`).
- **Battery Simulation**: Simulates power drain based on motor usage.

### 3. Simulation Layer (`hunter_description`)
Provides the physical world and robot model.
- **URDF/Xacro**: Defines the robot's kinematics and sensors (Lidar, Camera).
- **Gazebo World**: A custom hexagon world for testing.
- **ROS-GZ Bridge**: Handles communication between ROS 2 topics and Gazebo transport.

### 4. Telemetry Layer (`hunter_telemetry`)
Acts as the monitoring interface.
- **Data Aggregation**: Collects system state (Battery, Vision, Lidar, Velocity).
- **WebSocket Stream**: Pushes JSON telemetry and Base64-encoded video frames to an external dashboard (default: `http://localhost:5000`).

---

## 📦 Prerequisites & Requirements

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS
- **ROS Distribution**: ROS 2 Jazzy 
- **Simulation**: Gazebo

### ROS 2 Packages
Ensure you have the standard ROS 2 tools and the Gazebo bridge installed:
```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-xacro \
    ros-jazzy-cv-bridge \
    python3-colcon-common-extensions \ 
    ros-jazzy-py-trees 
```

### Python Dependencies
The project relies on several Python libraries for AI and networking. Install them via `pip`:
```bash
pip install -r requirements.txt
```
*Key libraries: `ultralytics` (YOLO), `filterpy` (Kalman), `python-socketio` (Telemetry), `opencv-python`.*

---

## 🚀 Installation & Build

1. **Clone the Repository**
   Navigate to your ROS 2 workspace `src` folder:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url> Chimpanzee_MetalMonkeys
   ```

2. **Install Dependencies**
   Use `rosdep` to install any missing system dependencies:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the Workspace**
   ```bash
   colcon build --symlink-install
   ```

4. **Source the Environment**
   ```bash
   source install/setup.bash
   ```

---

## 🎮 Usage Guide

### 1. Launch the Full Stack
The `hunter_bringup` package provides a master launch file that starts everything: Simulation, Perception, Control, and Telemetry.

```bash
ros2 launch hunter_bringup bringup.launch.py
```
*This will open Gazebo and start all background nodes.*

### 2. Teleoperate the Target (Red Ball)
To test the robot's tracking capabilities, you can manually control the target ball using the keyboard. Open a new terminal:

```bash
source install/setup.bash
ros2 run hunter_control ball_teleop
```
**Controls:**
- **W / S**: Move Forward / Backward (Hold)
- **A / D**: Rotate 90° Left / Right (Press)

### 3. Monitor Telemetry
Ensure your dashboard server is running (external to this repo). The robot will attempt to connect to `http://localhost:5000`.
- **Video Stream**: `/vision/debug_image`
- **Data**: Battery, Lidar distance, Tracking error.

---

## 🧩 Node Reference

| Node Name | Package | Description | Subscribed Topics | Published Topics |
|-----------|---------|-------------|-------------------|------------------|
| **vision_node** | `hunter_perception` | Runs YOLOv8 & Kalman Filter. | `/camera/image_raw` | `/vision/target`<br>`/vision/is_visible`<br>`/vision/debug_image` |
| **behavior_tree_node** | `hunter_control` | Executes the main control logic. | `/scan`<br>`/vision/*`<br>`/battery/status` | `/cmd_vel` |
| **telemetry_node** | `hunter_telemetry` | Streams data to Web Dashboard. | `/vision/*`<br>`/scan`<br>`/cmd_vel`<br>`/battery/status` | *(WebSockets)* |
| **battery_node** | `hunter_control` | Simulates battery discharge. | `/cmd_vel` | `/battery/status` |
| **ball_teleop** | `hunter_control` | Manual control for the target ball. | *(Keyboard Input)* | `/ball_cmd_vel` |
| **ros_gz_bridge** | `ros_gz_bridge` | Bridges ROS 2 and Gazebo. | `/cmd_vel`<br>`/ball_cmd_vel` | `/odom`<br>`/scan`<br>`/camera/image_raw` |



---

## ⚙️ Configuration

### Vision Parameters
Located in `src/hunter_bringup/config/params.yaml`:
- `model_path`: Path to YOLO weights (default: `yolov8n.pt`).
- `conf_threshold`: Confidence threshold for detection (default: `0.4`).
- `debug_window`: Enable/Disable local OpenCV window (default: `true`).

### Simulation Bridge
Located in `src/hunter_description/config/bridge.yaml`. Defines the topic mapping between ROS 2 and Gazebo.

---

## 📂 Project Structure

```plaintext
Chimpanzee_MetalMonkeys/
├── README.md                   # This file
├── requirements.txt            # Python dependencies
├── yolov8n.pt                  # YOLOv8 model weights
├── schema_*.md                 # Architecture diagrams
├── src/
│   ├── hunter_bringup/         # Launch files and high-level config
│   ├── hunter_control/         # Behavior Tree, Battery Sim, Teleop
│   ├── hunter_description/     # URDF, Worlds, Bridge Config
│   ├── hunter_perception/      # Vision pipeline (YOLO + Kalman)
│   └── hunter_telemetry/       # WebSocket Telemetry
```

---

## 🛠️ Troubleshooting

- **Gazebo not launching?**
  Ensure you have `ros-jazzy-ros-gz` installed and your graphics drivers are up to date.
- **"Target Lost" immediately?**
  Check lighting in Gazebo or lower the `conf_threshold` in `params.yaml`.
- **Robot not moving?**
  Check if the **Battery** is drained (monitor `/battery/status`). If 0%, restart the simulation.
- **YOLO Error?**
  Ensure `ultralytics` is installed and `yolov8n.pt` is present in the working directory.

---

## 👥 Authors

**Metal Monkeys Team** - *ISR Lab Project 2025*
