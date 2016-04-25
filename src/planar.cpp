//
// Created by Puyang Wang on 4/17/16.
//

// Include GLM
#include <glm/glm.hpp>
//#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>

#include <opencv2/opencv.hpp>
#include "planar_tracking.h"
#include "utils.h"


using namespace cv;
using namespace std;

const double ransac_thresh = 2.5f; // RANSAC inlier threshold
const double nn_match_ratio = 0.8f; // Nearest-neighbour matching ratio


glm::mat4 Tracker::getInitModelMatrix(){

    glm::mat4 initModelMatrix;
    Mat initR;
    Mat viewMatrix = cv::Mat::zeros(4, 4, CV_64FC1);
    Rodrigues(rvec, initR);

    for(unsigned int row=0; row<3; ++row)
    {
        for(unsigned int col=0; col<3; ++col)
        {
            viewMatrix.at<double>(row, col) = initR.at<double>(row, col);
        }
        viewMatrix.at<double>(row, 3) = tvec.at<double>(row, 0);
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

void Tracker::setFirstFrame(const char * first_frame_path, const char * bb_path)
{
    first_frame = imread(first_frame_path);
    vector<KeyPoint> kp;

    object_bb.push_back(Point2f(0,0));
    object_bb.push_back(Point2f(8,0));
    object_bb.push_back(Point2f(8,6));
    object_bb.push_back(Point2f(0,6));

    vector<Point2f> bb;
         FileStorage fs(bb_path, FileStorage::READ);
         if(fs["bounding_box"].empty()) {
             cerr << "Couldn't read bounding_box from " << bb_path << endl;
         }
    fs["bounding_box"] >> bb;

    Mat H;
    H = findHomography(bb, object_bb);

    detector->detectAndCompute(first_frame, noArray(), kp, first_desc);

    vector<Point2f> tmp_kp_orgn, tmp_kp_homo;

    first_kp = kp;

    for (int i = 0; i <= kp.size(); i++) {
        tmp_kp_orgn.push_back(Point2f(kp[i].pt));
    }

    perspectiveTransform(tmp_kp_orgn, tmp_kp_homo, H);

    for (int i = 0; i <= kp.size(); i++) {
        first_kp[i].pt = tmp_kp_homo[i];
    }

    //object_bb = bb;
}

bool Tracker::process(const Mat frame_left_rectified, bool slamMode)
{

    if (slamMode)
        return 1;

    vector<KeyPoint> kp;
    vector<Point3f> ObjectPoints;
    vector<Point2f> ImagePoints;
    Mat desc;
    detector->detectAndCompute(frame_left_rectified, noArray(), kp, desc);

    vector< vector<DMatch> > matches;
    vector<KeyPoint> matched1, matched2;
    matcher->knnMatch(first_desc, desc, matches, 2);
    for(unsigned i = 0; i < matches.size(); i++) {
        if(matches[i][0].distance < nn_match_ratio * matches[i][1].distance) {
            matched1.push_back(first_kp[matches[i][0].queryIdx]);
            matched2.push_back(      kp[matches[i][0].trainIdx]);
        }
    }

    Mat inlier_mask, homography;

    if(matched1.size() >= 20) {
        homography = findHomography(Points(matched1), Points(matched2),
                                    RANSAC, ransac_thresh, inlier_mask);
    }

    if(matched1.size() < 20 || homography.empty()) {
        return 0;
    }


    for(unsigned i = 0; i < matched1.size(); i++) {
        if(inlier_mask.at<uchar>(i)) {
            ObjectPoints.push_back(Point3f(matched1[i].pt.x, matched1[i].pt.y, 0));
            ImagePoints.push_back(Point2f(matched2[i].pt.x, matched2[i].pt.y));
        }
    }

    //solvePnPRansac(ObjectPoints, ImagePoints_1, CameraMatrix_1, DistCoeffs_1, rvec, tvec, true, 1000, 2.0, 0.99, inlier_mask, SOLVEPNP_ITERATIVE);
    solvePnP(ObjectPoints, ImagePoints, K, noArray(), rvec, tvec);

    return 1;
}
