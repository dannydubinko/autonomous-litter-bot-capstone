# autonomous-litter-bot-capstone
Go1 Litter(Garabge) Collecting Autononomus Robot

Install Dependencies 

```sudo apt update
sudo apt install -y \
  ros-jazzy-rviz2 \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-nav2-rviz-plugins \
  python3-colcon-common-extensions \
  libi2c-dev \
  i2c-tools
``` 

Build the workspace locally 

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the same drivers
git clone https://github.com/Slamtec/sllidar_ros2.git
git clone https://github.com/kimsniper/ros2_mpu6050.git

cd ~/ros2_ws
# This will check if your laptop has all the small background libraries needed
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

# Fix libray for jazzy
Navigate to `~/workspaces/ros2_ws/src/ros2_mpu6050/include/ros2_mpu6050/mpu6050.h` 

Add
```#include <array>
#include <cstdint>
```

Navigate to `src/mpu6050_node.cpp`

```
#include <stdexcept> // For runtime_error
```

Navigate to `~/workspaces/ros2_ws/src/ros2_mpu6050/CMakeLists.txt`

add after line 6
```
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

Navigate to `~/workspaces/ros2_ws/src/ros2_mpu6050/include/ros2_mpu6050/mpu6050.h`

add 
```
#include <array>
```


# Install Realsense Library

# Install CV Dependencies

```
cd ~/ros2_ws
# We do NOT use --system-site-packages this time
python3 -m venv venv 
source venv/bin/activate
```
```
# Upgrade pip first to avoid issues with older wheel formats
pip install --upgrade pip

# Install the core AI and Computer Vision stack
pip install ultralytics openvino-dev opencv-python numpy

# Install RealSense (Note: See the Hardware Warning below)
pip install pyrealsense2
```




# Build
Navigate back to your ros2 workspace

```
cd ~/capstone-group8/ros2_ws
```

```
colcon build --symlink-install
source install/setup.bash
```
<!-- 
# IMU 6050 Pin Out
Test IMU 

```
sudo apt update && sudo apt install i2c-tools
i2cdetect -y 1
``` -->

# Run Code

## Run LiDAR

### Set Up LiDAR (run once )
```
sudo usermod -aG dialout $USER
ls /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0
```
### Test LiDAR Node

```
ros2 launch sllidar_ros2 sllidar_c1_launch.py async_mode:=True
```

## Run Realsense and YOLO Model

