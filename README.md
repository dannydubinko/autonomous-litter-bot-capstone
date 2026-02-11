# autonomous-litter-bot-capstone
Go1 Litter(Garabge) Collecting Autononomus Robot

[IMPORTANT]
My Package is `autonomous_litter_bot_package

Package `ros2_mpu6050` and `sllidar_ros2` are not mine. I am updating the code to function with my system and with ROS2 Jazzy.

Please go support them at their individual repos :) <br />

https://github.com/Slamtec/sllidar_ros2.git
https://github.com/kimsniper/ros2_mpu6050.git

# Begin Here

Install Dependencies 

```bash
sudo apt update
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

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src



cd ~/ros2_ws
# This will check if your laptop has all the small background libraries needed
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

# Fix libray for jazzy
Navigate to `~/workspaces/ros2_ws/src/ros2_mpu6050/include/ros2_mpu6050/mpu6050.h` 

Add
```c++
#include <array>
#include <cstdint>
```

Navigate to `src/mpu6050_node.cpp`

```c++
#include <stdexcept> // For runtime_error
```

Navigate to `~/workspaces/ros2_ws/src/ros2_mpu6050/CMakeLists.txt`

add after line 6
```c++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

Navigate to `~/workspaces/ros2_ws/src/ros2_mpu6050/include/ros2_mpu6050/mpu6050.h`

add 
```c++
#include <array>
```


# Install Realsense Library

## 🔧 Building pyrealsense2 on Raspberry Pi 5 (Source Build)

The standard `pip install pyrealsense2` does not work on the Raspberry Pi 5 (ARM64). You must build the library from source using the **RSUSB backend** to ensure compatibility.

### Build Instructions

Run the following commands to build and install the Python bindings:

```bash
# 1. Install dependencies
sudo apt-get update && sudo apt-get install -y \
    git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev \
    cmake build-essential python3-dev python3-setuptools

# 2. Clone the repository
cd ~
git clone [https://github.com/IntelRealSense/librealsense.git](https://github.com/IntelRealSense/librealsense.git)
cd librealsense

# 3. Apply udev rules (Required for USB access)
sudo ./scripts/setup_udev_rules.sh

# 4. Create build directory
mkdir build && cd build

# 5. Configure CMake (Force RSUSB backend for Pi 5 stability)
cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true \
    -DPYTHON_EXECUTABLE=$(which python3) \
    -DCMAKE_BUILD_TYPE=Release \
    -DFORCE_RSUSB_BACKEND=true \
    -DBUILD_EXAMPLES=false \
    -DBUILD_GRAPHICAL_EXAMPLES=false

# 6. Compile and Install (Takes ~15-20 mins)
make -j4
sudo make install

# Install CV Dependencies

```bash
cd ~/ros2_ws
# We do NOT use --system-site-packages this time
python3 -m venv venv 
source venv/bin/activate
```
```bash
# Upgrade pip first to avoid issues with older wheel formats
pip install --upgrade pip

# Install the core AI and Computer Vision stack
pip install ultralytics openvino-dev opencv-python numpy

# Install RealSense (Note: See the Hardware Warning below)
pip install pyrealsense2
```




# Build
Navigate back to your ros2 workspace

```bash
cd ~/capstone-group8/ros2_ws
```

```bash
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
```bash
sudo usermod -aG dialout $USER
ls /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0
```

### Create TF Tree
```bash
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id laser & ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link
```

inside where the yaml file is held

```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file mapper_params_online_async.yaml
```
### Test LiDAR Node

```bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py async_mode:=True
```

```bash
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id imu_link
```

```bash
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
  -p laser_scan_topic:=/scan \
  -p odom_topic:=/odom \
  -p publish_tf:=true \
  -p base_frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p init_pose_from_topic:=""
```

### Launch RViz
```bash
ros2 run rviz2 rviz2
```



## Run IMU
### Calibrate IMU 
```bash
ros2 run ros2_mpu6050 ros2_mpu6050_calibrate
```
Copy values into `ros2_mpu6050/config/params.yaml`

DO NOT COPY the `accel_z_offset'

```bash
colcon build --symlink-install
source install/setup.bash
```

```bash
ros2 launch autonomous_litter_bot_capstone imu_system.launch.py
```

## Run Realsense and YOLO Model

### Launch Trash Detection Node
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch autonomous_litter_bot_package trash_detection.launch.py
```

## Open Image Feed from Realsense in a Fresh Terminal

```bash
source /opt/ros/jazzy/setup.bash
ros2 run rqt_image_view rqt_image_view
```
After click the drop down in the top left and clip /debug_image as the topic and the feed should show up including visual info on detected objects.

# Step By Step Launch Intructions Manual (Daniel)

## Launch imu 
### IMU Pin Out for Pi5

| 3.3V | 5V  |<br />
| SDA  | 5V  |<br />
| SCL  | GND | 

VCC -> 3V

### Launch Instructions

```bash
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --symlink-install --packages-select autonomous_litter_bot_package
source install/setup.bash
```

#### Launch RVIZ and URDF
```bash
ros2 launch autonomous_litter_bot_package display.launch.py
```

#### Launch IMU data
```bash
ros2 launch autonomous_litter_bot_package imu_system.launch.py
```
https://docs.ros.org/en/noetic/api/robot_localization/html/preparing_sensor_data.html

https://docs.ros.org/en/jazzy/p/slam_toolbox/

https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher-py.html