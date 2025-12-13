# Experimental_Robotics_course_assignment1
SimpleArucoMarkerDetectionAndVisualServoing
# Experimental Robotics Course - Assignment 1

A ROS 2 package for a differential drive robot that autonomously detects, inventories, and navigates to ArUco markers in a specific sequence.

## 📝 Project Overview

This project implements a **Finite State Machine (FSM)** to control a robot in a Gazebo simulation. The robot performs the following tasks:
1.  **Scan:** Rotates to identify all visible ArUco markers in the environment.
2.  **Sort:** Stores the detected marker IDs and sorts them from lowest to highest.
3.  **Navigate:** Uses **Visual Servoing** to approach each marker sequentially (Id 1 $\to$ Id 2 $\to$ ...).
4.  **Visualize:** Provides a real-time GUI view with state overlays and target highlighting.

## ⚙️ Architecture

The solution is built on **ROS 2** (C++) and uses `OpenCV` for logic and visualization.

### Finite State Machine (FSM)
The node `aruco_mission_node` manages the robot's behavior through five states:
* **SCANNING:** The robot rotates in place to build a map of available marker IDs.
* **SORTING:** Once `target_marker_count` is reached, the list of IDs is sorted.
* **NAVIGATING:** The robot uses a Proportional Controller (P-Controller) to center the target in the camera frame and drive towards it.
* **BACKING_UP:** Upon reaching the target (within `stop_distance`), the robot reverses to clear the field of view.
* **FINISHED:** All targets have been visited.

### Control Logic (Visual Servoing)
The robot subscribes to `/aruco_detections` (Pose) for control and `/camera/image_raw` for visualization.
* **Angular Velocity:** Minimizes the horizontal error (x-axis) to keep the marker centered.
* **Linear Velocity:** Minimizes the distance error (z-axis) to approach the marker.

## 📦 Dependencies

* **ROS 2** Jazzy
* **OpenCV** (`libopencv-dev`)
* **aruco_opencv** (for detection)
* **cv_bridge**
* **ros_aruco_opencv** 
* **image_transport**
* **geometry_msgs**

## 🛠️ Installation

1.  **Clone the repository** into your ROS 2 workspace:
    ```bash
    cd ~/ros2_ws/src
    git clone [https://github.com/Melasmar-711/Experimental_Robotics_course_assignment1.git](https://github.com/Melasmar-711/Experimental_Robotics_course_assignment1.git)
    ```

2.  **Install dependencies:**
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    git clone https://github.com/fictionlab/ros_aruco_opencv.git #if you don't already have it
    ```

3.  **Build the package:**
    ```bash
    cd ~/ros2_ws
    colcon build 
    source install/setup.bash
    ```

## 🚀 Usage

### 1. Launch the Simulation
Start your Gazebo world and the ArUco detector node.
*Note: Ensure the ArUco detector is using the correct dictionary (e.g., `DICT_ARUCO_ORIGINAL`) to match the simulation models and the correct camera topic in the `ros_aruco_opencv/aruco_opencv/config/aruco_tracker.yaml *
*Note : replace the `aruco_tracker.launch.xml` in the ros_aruco_opencv/aruco_opencv/launch with the one included in this pkg to ignore board detections 

```bash
ros2 launch assign1 spawn_robot.launch.py
ros2 launch aruco_opencv aruco_tracker.launch.xml
ros2 run assign1 aruco_node 
