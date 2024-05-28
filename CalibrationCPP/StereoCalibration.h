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
    void saveCalibrationData(const string& filename) const;

private:
    int squaresX;
    int squaresY;
    float squareLength;
    float markerLength;
    int dictionaryId;

    Mat cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR;
    Mat newCameraMatrixL, newCameraMatrixR;
    Mat rot, trans, essentialMatrix, fundamentalMatrix, Q;
    int numberOfFrames;
    double rmsL, rmsR, rmsStereo;
    vector<vector<Point3f>> objpoints;
    vector<vector<Point2f>> imgpointsL, imgpointsR;
};

#endif // STEREOCALIBRATION_H
