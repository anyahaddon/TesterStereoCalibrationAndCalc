#ifndef STEREOCALIBRATION_H
#define STEREOCALIBRATION_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <vector>
#include <string>
#include "json.hpp"


using namespace cv;
using namespace std;
using json = nlohmann::json;

class StereoCalibration {
public:
    StereoCalibration(int squaresX, int squaresY, float squareLength, float markerLength, int dictionaryId);
    void findCharucoCornersAndCalibrate(const vector<string>& imagesLeft, const vector<string>& imagesRight);
    void findCharucoCornersAndCalibrate2(const std::vector<std::string>& imagesLeft, const std::vector<std::string>& imagesRight);
    void saveCalibrationData(const string& filename) const;

    // TEMP ADDITION
    Mat undistortAndRemapLeft(const Mat& inputImageLeft);
    Mat undistortAndRemapRight(const Mat& inputImageRight);
    Point2f undistortPoint2fL(const Point2f& pointDistorted);
    Point2f undistortPoint2fR(const Point2f& pointDistorted);
    void calculateDepthFromPoints(const Point2f& pointL, const Point2f& pointR);
    float calculateDistancePointsPairs(const Point2f& pointsL_A, const Point2f& pointsR_A, const Point2f& pointsL_B, const Point2f& pointsR_B);
    Vec3f calculate2DTo3D(const Point2f& pt_left, const Point2f& pt_right);
    float calculateDistance(const Vec3f& pt_3D_A, const Vec3f& pt_3D_B);


private:
    int squaresX;
    int squaresY;
    float squareLength;
    float markerLength;
    int dictionaryId;

    Mat cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR;
    Mat newCameraMatrixL, newCameraMatrixR;
    Mat rot, trans, essentialMatrix, fundamentalMatrix, Q;
    Mat projMatrixL, projMatrixR;  // ADDITION
    Mat rectificationMatrixL, rectificationMatrixR;  // ADDITION
    Mat mapL1, mapL2;  // ADDITION
    Mat mapR1, mapR2;  // ADDITION
    double baseline;  // ADDITION

    int numberOfFrames;
    double rmsL, rmsR, rmsStereo;
    vector<vector<Point3f>> objpoints;
    vector<vector<Point2f>> imgpointsL, imgpointsR;
};

#endif // STEREOCALIBRATION_H
