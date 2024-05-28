#ifndef STEREOTRIANGULATION_H
#define STEREOTRIANGULATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "json.hpp"

using namespace cv;
using namespace std;
using json = nlohmann::json;

class StereoTriangulation {
public:
    StereoTriangulation(const std::string& filename) {
        loadCalibrationData(filename);
    }
    void loadCalibrationData(const std::string& filename);
	Mat undistortAndRemapLeft(const Mat& inputImageLeft);
    Mat undistortAndRemapRight(const Mat& inputImageRight);
    Point2f undistortPoint2f(const Point2f& pointDistorted);
    void calculateDepthFromPoints(const Point2f& pointsL, const Point2f& pointsR);
    float calculateDistancePointsPairs(const Point2f& pointsL_A, const Point2f& pointsR_A, const Point2f& pointsL_B, const Point2f& pointsR_B);
    Vec3f calculate2DTo3D(const Point2f& pt_left, const Point2f& pt_right);
    float calculateDistance(const Vec3f& pt_3D_A, const Vec3f& pt_3D_B);

private:
    Mat cameraMatrixL, distL, cameraMatrixR, distR;
    Mat rot, trans, Q;
    Mat projMatrixL, projMatrixR;
    Mat rectificationMatrixL, rectificationMatrixR;
    double baseline;
    double focalLength;
    Mat mapL1, mapL2;
    Mat mapR1, mapR2;
};

#endif // STEREOTRIANGULATION_H
