//
// Created by Puyang Wang on 4/15/16.
//

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <string.h>
#include <opencv2/core.hpp>
#include <iostream>

// Include GLM
#include <glm/glm.hpp>
//#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>

using namespace cv;
using namespace std;

Mat CameraMatrix, DistCoeffs, rvec, tvec;
cv::Mat cvToGl = cv::Mat::zeros(4, 4, CV_64F);

glm::mat4 getViewMatrix(){
    glm::mat4 V;
    Mat R;
    Mat viewMatrix = cv::Mat::zeros(4, 4, CV_64FC1);
    Rodrigues(rvec, R);

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

    viewMatrix.convertTo(viewMatrix, CV_32F);


    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++) {
            V[i][j] = viewMatrix.at<float>(j,i);
        }
    }

    return V;
}

bool loadIntrinsics(const char * K_path,const char * D_path){

    cvToGl.at<double>(0, 0) = 1.0f;
    cvToGl.at<double>(1, 1) = -1.0f;
// Invert the y axis
    cvToGl.at<double>(2, 2) = -1.0f;
// invert the z axis
    cvToGl.at<double>(3, 3) = 1.0f;

    FileStorage f1, f2;

    f1.open(K_path, FileStorage::READ);
    f2.open(D_path, FileStorage::READ);

    if (f1.isOpened() && f2.isOpened()) {
        f1["K"] >> CameraMatrix;
        f2["D"] >> DistCoeffs;
        f1.release();
        f2.release();
        return 1;
    }else{
        cout << "Couldn't open Intrinsics.xml or Distortion.xml!" << endl;
        return 0;
    }
}


bool track_chessboard(Mat ImgOrigin) {

    Size ChessboardSize(8,6);
    bool found;

    vector<Point3f> ObjectPoints;
    vector<Point2f> ImagePoints;

    for (int p = 0; p < ChessboardSize.height; p++) {
        for (int q = 0; q < ChessboardSize.width; q++) {
            ObjectPoints.push_back(Point3f(q,p,0));
        }
    }

    found = findChessboardCorners(ImgOrigin, ChessboardSize, ImagePoints, CALIB_CB_FAST_CHECK);
    if (found) {
        solvePnP(ObjectPoints, ImagePoints, CameraMatrix, DistCoeffs, rvec, tvec);
        return 1;
    } else
        return 0;
}

