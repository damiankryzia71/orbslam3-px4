# Instructions on how to use ORB-SLAM3 inside of a PX4 Gazebo simulation on Ubuntu 22

### Prerequisites
Follow the instructions in [this repository](https://github.com/damiankryzia71/orbslam3-px4-qgc-ubuntu22) to install PX4, QGroundControl, and ORB-SLAM3.

### 1. Run the Gazebo simulation
Build PX4 and run a Gazebo-simulated drone. Here, I tested with the depth camera model inside the Baylands world.
```bash
cd PX4-Autopilot/
make px4_sitl gz_x500_depth_baylands
```
### 2. View the camera feed inside QGroundControl
Run the QGroundControl app image.
```bash
./QGroundControl.AppImage
```
Inside QGroundControl, navigate to Application Settings -> General -> Video Settings and enable streaming from UDP h.264 Video Stream.
Leave the other options as default.
Now you should see the video feed from your Gazebo simulation inside QGroundControl.

### 3. Monocular/Grayscale Setup
The simulated camera should be publishing to a Gazebo topic. To list available topics, open a new terminal and run:
```bash
gz topic -l
```
Find a topic that publishes camera image data. In my case, it is:
```bash
/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image
```
Confirm that the topic is receiving image data by running:
```bash
gz topic -e -t /world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image
```
First, it's good to preview the images from Gazebo without running ORB-SLAM3.
To do so, put the `gz_preview_mono.cpp` file inside the `ORB-SLAM3/Examples/Monocular` directory.
Naviagate to the `CMakeLists.txt` file in the main `ORB-SLAM3` directory.
Under the line:
```cmake
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)
```
Add the following:
```cmake
add_executable(gz_preview_mono
        Examples/Monocular/gz_preview_mono.cpp)
target_link_libraries(gz_preview_mono gz-transport13 gz-msgs10 ${PROJECT_NAME})
```
Rebuild ORB-SLAM3 (in the ORB-SLAM3 directory).
```bash
./buid.sh
```
Run the executable with the Gazebo topic name as an argument.
```bash
./Examples/Monocular/gz_preview_mono /world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image
```
A window should open and you should see the feed from the Gazebo camera.
