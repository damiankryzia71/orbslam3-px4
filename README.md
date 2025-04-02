# Instructions on how to use ORB-SLAM3 inside of a PX4 Gazebo simulation on Ubuntu 22

## Prerequisites
Follow the instructions in [this repository](https://github.com/damiankryzia71/orbslam3-px4-qgc-ubuntu22) to install PX4, QGroundControl, and ORB-SLAM3.

## 1. Run the Gazebo simulation
To run a simulated drone with a camera, you will need to install GStreamer dependencies.
```bash
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
```
Build PX4 and run a Gazebo-simulated drone. Here, I tested with the depth camera model inside the Baylands world.
```bash
cd PX4-Autopilot/
make px4_sitl gz_x500_depth_baylands
```
## 2. View the camera feed inside QGroundControl
Run the QGroundControl app image.
```bash
./QGroundControl.AppImage
```
Inside QGroundControl, navigate to Application Settings -> General -> Video Settings and enable streaming from UDP h.264 Video Stream.
Leave the other options as default.

![QGC Settings](https://github.com/damiankryzia71/orbslam3-gz-ubuntu22/blob/1434da47aa1d87834c4a4d755039aa4110919705/screenshots/Screenshot%20from%202025-04-01%2018-06-33.png)

Now you should see the video feed from your Gazebo simulation inside QGroundControl.

## 3. Gazebo Development Setup
In order to make Gazebo libraries compile with ORB-SLAM3, we need to make small changes to soem Gazebo code.

Open the files `Node.hh` and `NodeShared.hh`. 

On my end, they are located in `/usr/include/gz/transport13/gz/transport`.

Both of the files should contain this line:
```cpp
public: std::optional<TopicStatistics> TopicStats(
        const std::string &_topic) const;
```
In both of the files, change it to:
```cpp
public: TopicStatistics TopicStats(
        const std::string &_topic) const;
```

## 4. Monocular/Grayscale Setup
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

Add the following somewhere at the top of the file:
```cmake
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
```
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
./build.sh
```
Run the executable with the Gazebo topic name as an argument while the simulation is running.
```bash
./Examples/Monocular/gz_preview_mono /world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image
```
A window should open, and you should see the feed from the Gazebo camera.

![Gazebo Preview Mono](https://github.com/damiankryzia71/orbslam3-gz-ubuntu22/blob/6e42ab974aca4b9628ace8618b3cfea9a7614ded/screenshots/Screenshot%20from%202025-04-01%2020-49-08.png)

Now, we are ready to run ORB-SLAM3 with the Gazebo simulation.

Put the `gz_video_mono.cpp` file in the `ORB-SLAM3/Examples/Monocular` directory.

In the `CMakeList.txt` file, under the line:
```cmake
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)
```
Add the following:
```cmake
add_executable(gz_video_mono
        Examples/Monocular/gz_video_mono.cpp)
target_link_libraries(gz_video_mono gz-transport13 gz-msgs10 ${PROJECT_NAME})
```
The gz_video_mono script converts the Gazebo images to grayscale and feeds them to `ORBSLAM3::System::TrackMonocular()`.

Rebuild ORB-SLAM3 (in the ORB-SLAM3 directory).
```bash
./build.sh
```

The last step needed is configuring the camera. In the `ORB-SLAM3/Examples/Monocular` directory, create the configuration file:
```bash
touch px4_camera_mono.yaml
```
To find out the Gazebo camera parameters, find the cmaera info topic while the simulation is running:
```bash
gz topic -l
```
In my case, it's `/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info`.

Echo the topic to the terminal with:
```bash
gz topic -e -t /world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info
```
While using one of ORB-SLAM3's example configuration files as a template, use the topic data to configure the camera and save the file.

Finally, with the simulation and QGroundControl running, run the program with the ORB-SLAM3 vocabulary file, the camera configuration file, and the Gazebo topic as arguments.
```bash
./Examples/Monocular/gz_video_mono ./Vocabulary/ORBvoc.txt ./Examples/Monocular/px4_camera_mono.yaml /world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image
```
You might need to move the simulated drone, for example, by running the takeoff command in QGroundControl, for the ORB-SLAM3 viewer to load properly.

You should see ORB-SLAM3 running within the simulated environment in the ORB-SLAM3 viewer.

![ORB-SLAM3 Viewer Mono](https://github.com/damiankryzia71/orbslam3-gz-ubuntu22/blob/71b52f816e1c0693b1930a2f86232f5bcdc629fc/screenshots/Screenshot%20from%202025-04-01%2021-13-42.png)
