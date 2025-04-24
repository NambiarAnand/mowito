# Project MOWITO 

This repository contains two main components:

- A behavior tree application built using `BehaviorTree.CPP`
- A ROS 2 workspace for dummy_usb_cam module

---

## 📁 1. BehaviorTree (`task/`)

### ✅ Prerequisites

- `BehaviorTree.CPP` library installed (e.g., v3.x)
- All dependencies (e.g., Boost, gtest, etc.)

### ▶️ Run Instructions

After building the application:

```bash
cd task/build
./task
```

## 📁 2. ROS2_WS 

### ✅ Prerequisites

- Tested on Ubuntu 22.04 ROS2 Humble Distro

### ▶️ Run Instructions

After cloning the folder

```bash
cd ros2_ws
colcon build
source install/setup.bash
ros2 launch camera_processing camera_processing.launch.py

# Switch to grayscale
ros2 service call /set_grayscale std_srvs/srv/SetBool "{data: true}"

# Switch back to color
ros2 service call /set_grayscale std_srvs/srv/SetBool "{data: false}"

# Install image tools if needed
sudo apt install ros-humble-image-tools

# View original image
ros2 run image_tools showimage --ros-args --remap image:=/image_raw

# View converted image
ros2 run image_tools showimage --ros-args --remap image:=/converted_image
```
