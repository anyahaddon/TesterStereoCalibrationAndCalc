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
    stereoRectify(cameraMatrixL/*K1*/, distL/*D1*/, cameraMatrixR/*K2*/, distR/*D2*/, Size(), rot, trans, R1, R2, P1, P2, Q);

    rectificationMatrixL = R1;
    rectificationMatrixR = R2;
    projMatrixL = P1;
    projMatrixR = P2;

    // Extracting and calculating the absolute value of the baseline
    baseline = trans.at<double>(0, 0);

    if (baseline < 0) {
        baseline = -baseline;
    }

    double focalLength = cameraMatrixL.at<double>(0, 0); // fx value from the camera matrix
    

}

Mat StereoTriangulation::undistortAndRemapLeft(const Mat& inputImageLeft) {
    Mat undistortedImage;

	if (mapL1.empty()) {
        initUndistortRectifyMap(cameraMatrixL, distL, rectificationMatrixL, projMatrixL, inputImageLeft.size(), CV_16SC2/*CV_32FC1???*/, mapL1, mapL2);

        //NN ???initUndistortRectifyMap(cameraMatrixL, distCoeffsL, noArray(), cameraMatrixL, imageSize, CV_16SC2, map1L, map2L);
        //EGODR
        //cv::stereoRectify(K[0], k[0], K[1], k[1], frameSize, R1, cam_tvecs[1], H0, H1,
        //    P0rect, P1rect, Qdtd);
        //cv::Mat map0X, map0Y, map1X, map1Y;
        //// precompute the remapping maps
        //cv::initUndistortRectifyMap(cameraMatrixL/*K[0]*/, distL/*k[0]*/, rectificationMatrixL/*H0*/, projMatrixL/*P0rect*/, inputImageLeft.size(), CV_32F, mapL1, mapL2);
	}

    remap(inputImageLeft, undistortedImage, mapL1, mapL2, INTER_LINEAR);

    return undistortedImage;
}

Mat StereoTriangulation::undistortAndRemapRight(const Mat& inputImageRight) {
    Mat undistortedImage;

    if (mapR1.empty()) {
        initUndistortRectifyMap(cameraMatrixR, distR, rectificationMatrixR, projMatrixR, inputImageRight.size(), CV_16SC2/*CV_32FC1???*/, mapR1, mapR2);
    }


    remap(inputImageRight, undistortedImage, mapR1, mapR2, INTER_LINEAR);

    return undistortedImage;
}


Point2f StereoTriangulation::undistortPoint2f(const Point2f& pointDistorted) {

    // Undistort points
    vector<Point2f> pts_distorted = { pointDistorted };
    vector<Point2f> pts_undistorted;

    undistortPoints(pts_distorted, pts_undistorted, cameraMatrixL/*K1*/, distL/*D1*/, rectificationMatrixL/*R1*/, projMatrixL/*P1*/);

    // The undistorted and rectified points
    Point2f pt = pts_undistorted[0];

    return pt;
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



float StereoTriangulation::calculateDistancePointsPairs(const Point2f& pointsL_A, const Point2f& pointsR_A, const Point2f& pointsL_B, const Point2f& pointsR_B) {

	Vec3f pt_3D_A = calculate2DTo3D(pointsL_A, pointsR_A);
	Vec3f pt_3D_B = calculate2DTo3D(pointsL_B, pointsR_B);

	float distance = calculateDistance(pt_3D_A, pt_3D_B);

	return distance;
}



Vec3f StereoTriangulation::calculate2DTo3D(const Point2f& pt_left, const Point2f& pt_right) {

    // Calculate disparity
    double disparity = pt_left.x - pt_right.x;

    // Baseline (distance between the two camera centers)
    double baseline = norm(trans);

    // Intrinsic parameters
    double fx = cameraMatrixL.at<double>(0, 0); // Focal length in x-axis
    double cx = cameraMatrixL.at<double>(0, 2); // Principal point x-coordinate
    double cy = cameraMatrixL.at<double>(1, 2); // Principal point y-coordinate

    // Compute 3D point
    Vec3f pt_3D;
    pt_3D[0] = (pt_left.x - cx) * baseline / disparity;
    pt_3D[1] = (pt_left.y - cy) * baseline / disparity;
    pt_3D[2] = fx * baseline / disparity;

	return pt_3D;
}



float StereoTriangulation::calculateDistance(const Vec3f& pt_3D_A, const Vec3f& pt_3D_B) {
    // Extract coordinates
    float x1 = pt_3D_A[0], y1 = pt_3D_A[1], z1 = pt_3D_A[2];
    float x2 = pt_3D_B[0], y2 = pt_3D_B[1], z2 = pt_3D_B[2];

    // Compute the differences
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;

    // Compute the Euclidean distance
    return sqrt(dx * dx + dy * dy + dz * dz);
}
