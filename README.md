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

### 3. Listen to the Gazebo topic.
The simulated camera should be publishing to a Gazebo topic. To list available topics, open a new terminal and run:
```bash
gz topic -l
```
Find a topic that publishes camera image data. In my case, it is:
```bash
/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image
```
Create a new C++ file. This program will subscribe to the Gazebo topic. Replace the topic name in GAZEBO_TOPIC if it is different on your end.
*The complete program with my configuration is in gz_video.cpp in this repository.*
```cpp
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>

#include "System.h"

#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>
#include <mutex>
#include <atomic>

using namespace std;

const string GAZEBO_TOPIC = "/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image";
```
