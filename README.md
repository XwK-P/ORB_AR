# ORB_AR
**Authors:** Puyang Wang, Raul Mur-Artal *et al*([ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2))

ORB_AR is a real-time AR application using ORB-SLAM2 as camera tracking method. Currently, it is only compatiable with stereo camera and does not contains stereo calibration part. Users should calibrate their stereo camera on their own and change ``` Extrinsics.xml```, ```Remap.xml``` and ```StereoCam.yaml``` accordingly.

**IMPORTANT**: This project is only tested on OS X 10.11.4 with OpenCV 3.0 and a [modified version of ORB-SLAM2] (https://github.com/XwK-P/ORB_SLAM2). For Linux users, you have to compile the [modified version of ORB-SLAM2] (https://github.com/XwK-P/ORB_SLAM2) and copy the library ```libORB_SLAM2.so``` to ```/lib```.

## Dependencies
* OpenCV
* OpenGL
* GLEW
* GLM
* GLFW
* all dependencices of ORB-SLAM2
* libpng

## Demonstration
<iframe width="420" height="315" src="https://www.youtube.com/embed/xljy3JuiB3w" frameborder="0" allowfullscreen></iframe>
