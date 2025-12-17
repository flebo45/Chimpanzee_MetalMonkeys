# 📡 Hunter Telemetry

**Hunter Telemetry** is the communication bridge between the robot's internal ROS 2 network and the external world. It aggregates real-time system data and streams it to a web-based dashboard for monitoring and debugging.

## 📋 Overview

In a production robotic system, it is crucial to monitor the robot's internal state without interfering with its control loops. This package acts as a **Data Sink**: it passively subscribes to various topics, processes the data (e.g., compressing video), and pushes it to a WebSocket server.

Key capabilities:
- **Data Aggregation**: Combines vision, lidar, battery, and velocity data into a single synchronized JSON payload.
- **Video Streaming**: Encodes ROS images into JPEG/Base64 for low-latency web display.
- **Decoupling**: Ensures that the heavy lifting of visualization (rendering charts, decoding video) happens on the client side, not on the robot.

---

## ⚙️ How It Works

The `telemetry_node` operates on a 10Hz timer loop for telemetry and an event-driven callback for video.

1.  **Connection**: On startup, it attempts to connect to a Socket.IO server at `http://localhost:5000`. It handles automatic reconnection if the server goes down.
2.  **Data Collection**:
    - **Vision**: Extracts target error and area from `/vision/target`.
    - **Lidar**: Calculates the minimum distance in the front sector from `/scan`.
    - **Battery**: Reads the State of Charge from `/battery/status`.
    - **Control**: Monitors `/cmd_vel` to report what the robot *intends* to do.
3.  **Transmission**:
    - **Telemetry**: Emits the `robot_telemetry` event with a JSON object.
    - **Video**: Emits the `robot_video` event with a Base64 string.

---

## 📊 Data Protocol

### JSON Payload (`robot_telemetry`)

```json
{
  "error_x": 12.5,       // Horizontal pixel error (PID input)
  "area": 4500.0,        // Target area (Distance proxy)
  "distance": 1.2,       // Minimum obstacle distance (meters)
  "cmd_vel_x": 0.2,      // Linear velocity (m/s)
  "cmd_vel_z": -0.1,     // Angular velocity (rad/s)
  "visible": true,       // Is target tracked?
  "battery_level": 85.0  // Battery percentage
}
```

### Video Stream (`robot_video`)

- **Format**: JPEG Image.
- **Encoding**: Base64 String.
- **Resolution**: Downscaled to 320x240 for bandwidth optimization.
- **Quality**: 50% JPEG compression.

---

## 💻 Usage

This node is typically launched via `hunter_bringup`, but can be run standalone:

```bash
ros2 run hunter_telemetry telemetry_node
```

### Prerequisites
For this node to be useful, you need a **Socket.IO Server** running on port 5000. If the server is not running, the node will log connection errors but will continue to function (it won't crash ROS).

---

## 📂 Code Structure

```plaintext
hunter_telemetry/
├── hunter_telemetry/
│   ├── __init__.py
│   └── telemetry_node.py  # Main logic
├── package.xml
└── setup.py
```
