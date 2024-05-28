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
    void calculateDepthFromPoints(const Point2f& pointsL, const Point2f& pointsR);

private:
    Mat cameraMatrixL, distL, cameraMatrixR, distR;
    Mat rot, trans, Q;
    Mat projMatrixL, projMatrixR;
    double baseline;
    double focalLength;
};

#endif // STEREOTRIANGULATION_H
