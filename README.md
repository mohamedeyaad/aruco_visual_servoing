# aruco_visual_servoing

**Autonomous visual servoing package for a mobile robot. Implements ArUco marker detection, ID sorting, and target centering in Gazebo Harmonic with ROS 2 Jazzy.**

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![License](https://img.shields.io/badge/License-MIT-green)

## 📖 Overview

This package provides a complete visual servoing pipeline for a mobile robot in a simulated environment. The system enables a differential drive robot to autonomously identify multiple ArUco markers, sort them by ID, and sequentially visit them. It utilizes a Proportional (P) controller for precise alignment and distance keeping, bridging the gap between perception (OpenCV) and actuation (ROS 2 Control).

## ✨ Key Features

* **Autonomous Exploration:** Rotates to scan the environment and build a map of available ArUco markers.
* **Sequential Chasing:** Automatically sorts detected markers and visits them in ascending numerical order.
* **Visual Servoing Control:**
    * **Angular:** Aligns the robot's heading to the marker's center.
    * **Linear:** Approaches the target to a precise distance of 1.0 meter.
* **Visual Feedback:** Publishes a debug topic `/aruco_target_circled` highlighting the active target in real-time.
* **Simulation Ready:** Includes a custom Gazebo world with a circular array of markers and a URDF robot model.

## 🤖 Real Robot Deployment

The control logic was successfully deployed on a **Husarion ROSbot 2**.

[![ArUco Chaser Demo](https://img.youtube.com/vi/FSaDam3q0-Y/0.jpg)](https://youtu.be/FSaDam3q0-Y)

*Click the image above to watch the simulation in action.*

## 🛠️ Tech Stack

* **Framework:** ROS 2 Jazzy
* **Simulation:** Gazebo Harmonic (GZ Sim)
* **Language:** Python 3.10+
* **Libraries:** `opencv-python`, `cv_bridge`, `ros2_aruco`
* **Middleware:** `ros_gz_bridge`

## ⚙️ Installation

### Prerequisites
Ensure you have **ROS 2 Jazzy** and **Gazebo Harmonic** installed. This package depends on the `ros2_aruco` package from JMU-ROBOTICS-VIVA for marker detection.

```bash
# Clone the specific ArUco dependency
cd ~/ros2_ws/src
git clone https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco.git
````

### Build

Clone this repository and build the workspace:

```bash
# Clone this package
cd ~/ros2_ws/src
git clone https://github.com/mohamedeyaad/aruco_visual_servoing.git

# Build dependencies and source
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 🚀 Usage

### 1\. Launch Simulation

This brings up the Gazebo environment, spawns the robot, and establishes the ROS-GZ bridges.

```bash
ros2 launch aruco_visual_servoing simulation.launch.py
```

### 2\. Launch the Detector & Controller

Start the main visual servoing logic:

```bash
ros2 launch aruco_visual_servoing servoing.launch.py
```

## 🧠 Architecture

The core logic (`visual_servoing_node.py`) implements a Finite State Machine (FSM):

1.  **SEARCHING:** The robot performs a 360° scan to populate a set of unique marker IDs.
2.  **ALIGNING:**
      * **Angular Control:** Minimizes the horizontal error (x-offset) of the marker in the camera frame.
      * **Linear Control:** Minimizes the depth error to maintain a 1.0m standoff distance.
3.  **VISITING:** The robot pauses at the target, verifies the ID, and creates a visual overlay before proceeding to the next target in the sequence.

## 🚙 Skid-Steer Variant

This package supports different kinematic configurations. A **Skid-Steer (4-wheel)** implementation is available in a separate branch. This variant replaces the differential drive controller with a skid-steer plugin and updates the URDF to a 4-wheel chassis.

**To use the skid-steer robot:**

1.  Switch branches:
    ```bash
    git checkout skid-steer
    ```
2.  Rebuild the package (necessary to update robot description and plugins):
    ```bash
    colcon build
    source install/setup.bash
    ```
3.  Launch normally:
    ```bash
    ros2 launch aruco_visual_servoing simulation.launch.py
    ```

## 📦 Topics

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands sent to the robot controller |
| `/aruco_markers` | `ros2_aruco_interfaces/ArucoMarkers` | Detected marker poses and IDs |
| `/aruco_target_circled` | `sensor_msgs/Image` | Processed camera feed with target highlights |
| `/camera/image_raw` | `sensor_msgs/Image` | Raw camera stream from Gazebo |

## 📂 Project Structure

```text
aruco_visual_servoing/
├── config/
│   └── ros_gz_bridge.yaml       # Bridge configuration for topics
├── launch/
│   └── simulation.launch.py     # Main launch file
├── rviz/
│   └── aruco_visual_servoing.rviz
├── urdf/
│   ├── aruco_bot_diff.xacro     # Main robot description
│   ├── aruco_bot_diff.gazebo.xacro # Gazebo plugins
│   └── materials.xacro          # Color definitions
├── worlds/
│   └── aruco_world.world        # Gazebo world with 5 markers
├── aruco_visual_servoing/
│   └── test.py                  # Main Python Control Node
├── package.xml
└── setup.py
```
