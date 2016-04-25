//
// Created by Puyang Wang on 4/16/16.
//

#ifndef ORB_SLAM_H
#define ORB_SLAM_H
#include "ORB_SLAM/System.h"

glm::mat4 getViewMatrix(bool slamMode);
glm::mat4 getInitModelMatrix();
Mat getCameraMatrix();
void stereoRemap(Mat frame_left, Mat frame_right, Mat& frame_left_rectified, Mat& frame_right_rectified);
bool initTracking(const char * Remap_path, const char * Extrincis_path);
bool TryInitModelMatrix(Mat frame_left, bool success);
bool trackStereo(Mat CameraPose);

#endif //BOB_AR_ORB_SLAM_H
