#include <iostream>

#include <opencv2/opencv.hpp>

// Include GLM
#include <glm/glm.hpp>
//#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>

#include "ORB_SLAM/System.h"

using namespace std;
using namespace cv;

Mat map11, map12, map21, map22, RCalib, TCalib, P1, P2, K;
Mat R, tvec, initRvec, initTvec;
cv::Mat cvToGl = cv::Mat::zeros(4, 4, CV_64F);

Mat getCameraMatrix(){
    return K;
}

void stereoRemap(Mat frame_left, Mat frame_right, Mat& frame_left_rectified, Mat& frame_right_rectified){

    remap(frame_left, frame_left_rectified, map11, map12, cv::INTER_LINEAR);
    remap(frame_right, frame_right_rectified, map21, map22, cv::INTER_LINEAR);

}

glm::mat4 getViewMatrix(bool slamMode){
    glm::mat4 V;
    Mat viewMatrix = cv::Mat::zeros(4, 4, CV_64FC1);

    if (slamMode){
        R.convertTo(R, CV_64F);
        tvec.convertTo(tvec, CV_64F);

        for(unsigned int row=0; row<3; ++row)
        {
            for(unsigned int col=0; col<3; ++col)
            {
                viewMatrix.at<double>(row, col) = R.at<double>(row, col);
            }
            viewMatrix.at<double>(row, 3) = tvec.at<double>(row, 0);
        }
        viewMatrix.at<double>(3, 3) = 1.0f;

        viewMatrix = cvToGl * viewMatrix;
    } else{
        viewMatrix = cvToGl;
    }

    viewMatrix.convertTo(viewMatrix, CV_32F);


    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++) {
            V[i][j] = viewMatrix.at<float>(j,i);
        }
    }

    return V;


}

glm::mat4 getInitModelMatrix(){
    glm::mat4 initModelMatrix;
    Mat initR;
    Mat viewMatrix = cv::Mat::zeros(4, 4, CV_64FC1);
    Rodrigues(initRvec, initR);

    for(unsigned int row=0; row<3; ++row)
    {
        for(unsigned int col=0; col<3; ++col)
        {
            viewMatrix.at<double>(row, col) = initR.at<double>(row, col);
        }
        viewMatrix.at<double>(row, 3) = initTvec.at<double>(row, 0);
    }
    viewMatrix.at<double>(3, 3) = 1.0f;

    //viewMatrix = cvToGl * viewMatrix;

    viewMatrix.convertTo(viewMatrix, CV_32F);


    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++) {
            initModelMatrix[i][j] = viewMatrix.at<float>(j,i);
        }
    }

    return initModelMatrix;
}

bool initTracking(const char * Remap_path, const char * Extrinsics_path){

    cvToGl.at<double>(0, 0) = 1.0f;
    cvToGl.at<double>(1, 1) = -1.0f;
// Invert the y axis
    cvToGl.at<double>(2, 2) = -1.0f;
// invert the z axis
    cvToGl.at<double>(3, 3) = 1.0f;


    cv::FileStorage f1,f2;
    f1.open(Remap_path, cv::FileStorage::READ);
    f2.open(Extrinsics_path, cv::FileStorage::READ);

    if (f1.isOpened()&&f2.isOpened()){
        f1["map11"] >> map11;
        f1["map12"] >> map12;
        f1["map21"] >> map21;
        f1["map22"] >> map22;
        f2["R"] >> RCalib;
        f2["T"] >> TCalib;
        f2["P1"] >> P1;
        f2["P2"] >> P2;
        f1.release();
        f2.release();
    } else{
        cout << "Couldn't open Remap.xml or Extrinsics.xml" << endl;
        return 0;
    }

    P1.rowRange(0,3).colRange(0,3).copyTo(K);

    return 1;
}

bool trackStereo(Mat CameraPose){

    if (CameraPose.empty()) {
        return 0;
    }

    CameraPose.rowRange(0,3).colRange(0,3).copyTo(R);
    CameraPose.rowRange(0,3).col(3).copyTo(tvec);

    return 1;
}

bool TryInitModelMatrix(Mat frame_left, bool slamMode){

    if (slamMode)
        return 1;

    Mat frame_left_rectified;
    vector<cv::Point2f> ImagePoints_1;
    vector<Point3f> ObjectPoints;

    cv::Size ChessboardSize(8,6);

    for (int p = 0; p < ChessboardSize.height; p++) {
        for (int q = 0; q < ChessboardSize.width; q++) {
            ObjectPoints.push_back(Point3f(q,p,0));
        }
    }
    bool found_left;


    remap(frame_left, frame_left_rectified, map11, map12, cv::INTER_LINEAR);

    found_left = cv::findChessboardCorners(frame_left_rectified, ChessboardSize, ImagePoints_1, cv::CALIB_CB_FAST_CHECK);

    if (found_left) {

        solvePnP(ObjectPoints, ImagePoints_1, K, noArray(), initRvec, initTvec);

        return 1;

    } else
        return 0;
}