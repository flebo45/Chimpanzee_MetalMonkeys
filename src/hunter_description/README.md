# 🌍 Hunter Description

**Hunter Description** provides the physical assets and simulation environment for the Hunter Robot project. It defines *what* the robot is (URDF), *where* it lives (Gazebo World), and *how* it communicates with the simulation (ROS-GZ Bridge).

## 📋 Overview

This package is responsible for the "Physical Layer" of the system. It does not contain control logic or perception algorithms; instead, it ensures that the simulated robot behaves realistically and exposes the correct interfaces (sensors and actuators) to ROS 2.

Key components:
1.  **Robot Model**: A differential drive robot defined in Xacro/URDF, equipped with a Lidar and a Camera.
2.  **Simulation World**: A custom Gazebo environment (`hexagon_world.sdf`) with walls to contain the robot and the target.
3.  **Target**: A standalone SDF model for the Red Ball (`rc_ball.sdf`).
4.  **Bridge**: The configuration for `ros_gz_bridge` to translate topics between ROS 2 (DDS) and Gazebo (Ignition Transport).

---

## 📂 Package Structure

```plaintext
hunter_description/
├── config/
│   └── bridge.yaml          # ROS <-> Gazebo topic mapping
├── launch/
│   └── hunter_sim.launch.py # Launch file for Gazebo + Spawning
├── urdf/
│   ├── robot.urdf.xacro     # Main robot description
│   └── rc_ball.sdf          # Red Ball model
├── worlds/
│   └── hexagon_world.sdf    # Simulation environment
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 🤖 Robot Model (`robot.urdf.xacro`)

The **Hunter Robot** is a differential drive mobile base.
- **Chassis**: Cylindrical base.
- **Drive**: Two active wheels + one caster wheel.
- **Sensors**:
    - **Lidar**: Mounted on top, provides 360° laser scans (`/scan`).
    - **Camera**: Front-facing, provides RGB images (`/camera/image_raw`).
- **Plugins**: Uses `diff_drive` plugin for movement and sensor plugins for data generation.

---

## 🌉 ROS-Gazebo Bridge (`config/bridge.yaml`)

Since Gazebo and ROS 2 use different communication protocols, a bridge is required. This configuration file defines the translation rules:

| ROS 2 Topic | Gazebo Topic | Direction | Type | Description |
|-------------|--------------|-----------|------|-------------|
| `/cmd_vel` | `/cmd_vel` | ROS -> GZ | `Twist` | Robot movement commands |
| `/odom` | `/odom` | GZ -> ROS | `Odometry` | Robot position feedback |
| `/scan` | `/scan` | GZ -> ROS | `LaserScan` | Lidar sensor data |
| `/camera/image_raw` | `/camera/image_raw` | GZ -> ROS | `Image` | Camera video feed |
| `/ball_cmd_vel` | `/model/rc_ball/cmd_vel` | ROS -> GZ | `Twist` | Teleop commands for the ball |
| `/tf` | `/tf` | GZ -> ROS | `TFMessage` | Coordinate transforms |

---

## 🚀 Launching the Simulation

While usually started via `hunter_bringup`, you can launch the simulation standalone to inspect the model or world:

```bash
ros2 launch hunter_description hunter_sim.launch.py
```

**What happens:**
1.  **Gazebo Starts**: Loads `worlds/hexagon_world.sdf`.
2.  **Robot Spawns**: The Xacro is parsed to XML and spawned at `(-4.0, 0.0)`.
3.  **Ball Spawns**: The Red Ball is spawned at `(-2.0, 0.0)`.
4.  **Bridge Starts**: Topics begin flowing between ROS and Gazebo.
5.  **State Publisher**: `robot_state_publisher` starts broadcasting TFs.

---

## 🛠️ Customization

- **Modify the World**: Edit `worlds/hexagon_world.sdf` to add obstacles or change lighting.
- **Modify the Robot**: Edit `urdf/robot.urdf.xacro` to change sensor placement or wheel physics.
- **Add Topics**: If you add a new sensor, remember to add a corresponding entry in `config/bridge.yaml`.
