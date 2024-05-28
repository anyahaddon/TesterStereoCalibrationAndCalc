#include "StereoTriangulation.h"
#include <fstream>


using json = nlohmann::json;
using namespace cv;

// Helper function to convert a JSON array to cv::Mat
cv::Mat jsonToMat(const json& j) {
    std::vector<double> array = j.get<std::vector<double>>();
    cv::Mat mat(array.size(), 1, CV_64F, array.data());
    return mat;
}

cv::Mat jsonToMat2D(const json& j) {
    std::vector<std::vector<double>> array2D = j.get<std::vector<std::vector<double>>>();
    cv::Mat mat(array2D.size(), array2D[0].size(), CV_64F);
    for (size_t i = 0; i < array2D.size(); i++)
        for (size_t j = 0; j < array2D[0].size(); j++)
            mat.at<double>(i, j) = array2D[i][j];
    return mat;
}

void StereoTriangulation::loadCalibrationData(const std::string& filename) {
    // Open the file
    std::ifstream file(filename);

    // Parse the file into a JSON object
    json calibrationData;
    file >> calibrationData;

    // Load camera parameters from JSON using the helper function
    cameraMatrixL = jsonToMat2D(calibrationData["LeftCalibrationCameraData"]["CameraMatrix"]);
    distL = jsonToMat2D(calibrationData["LeftCalibrationCameraData"]["DistortionCoefficients"]);

    cameraMatrixR = jsonToMat2D(calibrationData["RightCalibrationCameraData"]["CameraMatrix"]);
    distR = jsonToMat2D(calibrationData["RightCalibrationCameraData"]["DistortionCoefficients"]);

    rot = jsonToMat2D(calibrationData["CalibrationStereoCameraData"]["Rotation"]);
    trans = jsonToMat2D(calibrationData["CalibrationStereoCameraData"]["Translation"]);
    Q = jsonToMat2D(calibrationData["CalibrationStereoCameraData"]["Q"]);

    // Compute the projection matrices for triangulation
    Mat R1, R2, P1, P2;
    stereoRectify(cameraMatrixL, distL, cameraMatrixR, distR, Size(), rot, trans, R1, R2, P1, P2, Q);

    projMatrixL = P1;
    projMatrixR = P2;

    // Extracting and calculating the absolute value of the baseline
    baseline = trans.at<double>(0, 0);

    if (baseline < 0) {
        baseline = -baseline;
    }

    double focalLength = cameraMatrixL.at<double>(0, 0); // fx value from the camera matrix
}



 // Calculate the depth from two image points
void StereoTriangulation::calculateDepthFromPoints(const Point2f& pointL, const Point2f& pointR) {
    double disparity = pointL.x - pointR.x;
    if (disparity == 0) {
        cout << "Invalid disparity (zero). Depth cannot be calculated." << endl;
        return;
    }

    double focalLength = cameraMatrixL.at<double>(0, 0); // fx value from the camera matrix
    double depth = (focalLength * baseline) / disparity;

    cout << "Calculated Depth: " << depth << " meters" << endl;
}