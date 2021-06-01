# StereoVision
StereoVision Project with ZED &amp; Intel RealSense Camera

## Project purpose

- The purpose was creating image segmentation and obtaining [stixel](https://github.com/gishi523/stixel-world) image with stereo camera
- Example demonstration of [a stixel project](https://www.youtube.com/watch?v=i8dcQYPC2kg)

## Hardware Tools
- [ZED Mini](https://www.stereolabs.com/zed-mini/)
- [Intel Realsense](https://www.intelrealsense.com/depth-camera-d435i/)
- [Jetson TX2](https://developer.nvidia.com/embedded/jetson-tx2-developer-kit)

## Software Tools
- Ubuntu 18
- [ZED SDK](https://www.stereolabs.com/developers/release/)
- [Intel Realsense](https://www.intelrealsense.com/developers/)
- [CUDA](https://developer.nvidia.com/cuda-downloads)
- [CMake](https://cmake.org/)
- [Doxygen](https://www.doxygen.nl/index.html)
- [ROS](https://www.ros.org/)
- [OpenCV](https://opencv.org/)


##
The image below shows an example with ZED camera & Jetson TX2 how to seperate the ground and decide horizon line at the image. The image was obtained by creating v-disp image with Opencv histogram function


![v-disp](https://user-images.githubusercontent.com/42723084/120318314-02c72300-c2e0-11eb-81c9-b72f9744a6bf.png)

