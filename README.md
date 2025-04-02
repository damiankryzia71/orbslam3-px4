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
