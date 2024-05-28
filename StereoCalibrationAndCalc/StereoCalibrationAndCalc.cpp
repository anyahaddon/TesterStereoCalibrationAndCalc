#include <vector>
#include "StereoCalibrationAndCalc.h"
#include <stdexcept>


/// <summary>
/// Load calibration data from JSON string into suitable data structures
/// </summary>
/// <param name="jsonCalibrationData"></param>
/// <returns>true if the necessary calibration data was extracted from the JSON</returns>
bool StereoCalibrationAndCalc::LoadFromJSON(const std::string& jsonCalibrationData) {
	bool ret = false;

    return ret;
}


/// <summary>
/// Write the calibration data from the class into a JSON format
/// </summary>
/// <param name="jsonCalibrationData"></param>
/// <returns>true if the calibration data was successfully written into the JSON format.</returns>
bool StereoCalibrationAndCalc::SaveToJSON(std::string& jsonCalibrationData) {
    bool ret = false;

    return ret;
}


/// <summary>
/// Load calibration data from JSON string that is in the Calib IO JSON format
/// In order to save into our format alson OpenCV calls are made to adjust the data
/// </summary>
/// <param name="jsonCalibrationData"></param>
/// <returns></returns>
bool LoadCalibIOFormatFromJSON(const std::string& jsonCalibrationData) {
    bool ret = false;



    // Calibration exported from Calibrator for a system of two cameras
    // (stereo calibration)
    std::string jsonFilePath = std::string("c:/temp") + "/stereo_calibration.json";

    // Read most common calibration parameters
    std::vector<cv::Matx33d> K;
    std::vector<cv::Vec<double, 14>> k;
    std::vector<cv::Vec3d> cam_rvecs;
    std::vector<cv::Vec3d> cam_tvecs;
    readCalibParameters(jsonFilePath, K, k, cam_rvecs, cam_tvecs);

    // Example: project a 3D point into both cameras
    std::vector<cv::Point3d> Q = { cv::Point3d(0.4, -0.02, 2.3) }; //[m]
    std::cout << "Q: " << Q << std::endl;
    std::vector<cv::Point2d> q0, q1;
    cv::projectPoints(Q, cam_rvecs[0], cam_tvecs[0], K[0], k[0], q0); // camera 0
    cv::projectPoints(Q, cam_rvecs[1], cam_tvecs[1], K[1], k[1], q1); // camera 1
    std::cout << "q0: " << q0[0] << std::endl;
    std::cout << "q1: " << q1[0] << std::endl;

    // undistort pixel coordinates
    std::vector<cv::Point2d> q0u, q1u;
    cv::undistortPoints(q0, q0u, K[0], k[0], cv::noArray(), K[0]);
    cv::undistortPoints(q1, q1u, K[1], k[1], cv::noArray(), K[1]);
    // undistorted points now obey the pinhole models K0 and K1

    // Example: compute the fundamental and essential matrices
    cv::Matx33d R0, R1;
    cv::Rodrigues(cam_rvecs[0], R0);
    cv::Rodrigues(cam_rvecs[1], R1);
    // cross product operator for the translation
    cv::Matx33d tx(0.0, -cam_tvecs[1][2], cam_tvecs[1][1], //
        cam_tvecs[1][2], 0.0, -cam_tvecs[1][0], //
        -cam_tvecs[1][1], cam_tvecs[1][0], 0.0);
    // essential matrix
    cv::Matx33d E = tx * R1;
    // fundamental matrix
    cv::Matx33d F = K[1].t().inv() * E * K[0].inv();
    // check - should be close to zero:
    std::cout << "q1u^T * F * q0u: "
        << cv::Vec3d(q1u[0].x, q1u[0].y, 1.0).t() * F *
        cv::Vec3d(q0u[0].x, q0u[0].y, 1.0)
        << std::endl;

    // Example: triangulate back to 3D
    // construct linear projection matrices (P = K[R|t])
    // for camera 0 this will be K[I|0] as camera 0 is located at the origin of
    // the global/world coordinate system
    cv::Mat Rt0, Rt1;
    cv::hconcat(R0, cam_tvecs[0], Rt0);
    cv::hconcat(R1, cam_tvecs[1], Rt1);
    cv::Matx34d P0 = cv::Mat(K[0] * Rt0);
    cv::Matx34d P1 = cv::Mat(K[1] * Rt1);
    // correct matches - assuming gaussian noise this gives the max likelihood
    // 'optimal triangulation' result. without this step, triangulation will be
    // faster but non statistically optimal.
    std::vector<cv::Point2d> q0c, q1c;
    cv::correctMatches(F, q0u, q1u, q0c, q1c);
    // triangulate (DLT algorithm)
    cv::Vec4d Qhom;
    cv::triangulatePoints(P0, P1, q0c, q1c, Qhom);
    // convert to non-homogenous
    cv::Point3d Qtriang{ Qhom(0) / Qhom(3), Qhom(1) / Qhom(3), Qhom(2) / Qhom(3) };
    std::cout << "Qtriang: " << Qtriang << std::endl;

    // Example: compute rectifying homographies which can warp images to match
    // line by line
    cv::Matx33d H0, H1;
    cv::Matx34d P0rect, P1rect;
    cv::Matx44d Qdtd;
    cv::Size frameSize(5120, 3840);
    cv::stereoRectify(K[0], k[0], K[1], k[1], frameSize, R1, cam_tvecs[1], H0, H1,
        P0rect, P1rect, Qdtd);

    return ret;
}


/// <summary>
/// Ideally, this function should calibrate the stereo camera using the provided image sets
/// and identify which calibration target is being used (i.e., Checkerboard, Circles, or 
/// Charuco). If this isn't feasible, create a dedicated method for each
/// target type, e.g., CalibrateCheckerboard, CalibrateCircles, CalibrateCharuco.
/// The images will either be .png or .jpeg.
/// Assumes that the left and right file lists contain corresponding images in the same 
/// order.
/// Stores the calibration data within the class for later use.
/// </summary>
/// <param name="leftFileList"></param>
/// <param name="rightFileList"></param>
/// <param name="jsonCalibrationData"></param>
/// <returns></returns>
bool StereoCalibrationAndCalc::Calibrate(const std::vector<std::string>& leftFileList, const std::vector<std::string>& rightFileList) {
    bool ret = false;

    return ret;
}

/// <summary>
/// Called to load the 2D points from a stereo camera into the class.
/// The class can hold two sets of points, POINTS_A and POINTS_B for each camera.
/// Use cases:
/// 1. Load a corresponding pair of 2D points for TARGET_A and a corresponding pair for TARGET_B (typically used to measure the length of a fish).
/// 2. Load a corresponding pair of 2D points for TARGET_A only (used when the head and tail of the fish are not visible in both cameras) (Note: GetLengthBetweenPoints() will not function properly in this scenario).
/// </summary>
/// <param name="pointsSet">TARGET_A or TARGET_B</param>
/// <param name="cameraSide">LEFT or RIGHT</param>
/// <param name="X"></param>
/// <param name="Y"></param>
void StereoCalibrationAndCalc::LoadStereo2DPoint(enum TargetSet targetSet, enum CameraSide cameraSide, double X, double Y) {

	// Load 2D (X,Y) into suitable storage for the indicated target set and camera side
	// i.e. either TARGET_A/LEFT, TARGET_A/RIGHT, TARGET_B/LEFT, TARGET_B/RIGHT

    return;
}


/// <summary>
/// Returns the distance from the origin to the 3D point of the indicated target set.
/// Nicolai, is the origin considered as the left camera in stereo projection?
/// This function requires that a corresponding point is loaded for both cameras to operate correctly.
/// </summary>
/// <param name="targetSet"></param>
/// <returns>measurment</returns>
double StereoCalibrationAndCalc::GetDistanceFromOrigin(enum TargetSet targetSet) {
    double ret = 0;

    return ret;
}


/// <summary>
/// Nicolai, is there a standard method to measure precision in stereo projection math?
/// For example, when a user selects a corresponding point on the other camera that does not lie on the epipolar line.
/// This function requires that a corresponding point is loaded for both cameras to operate.
/// </summary>
/// <param name="targetSet"></param>
/// <returns>??</returns>
double StereoCalibrationAndCalc::GetPrecision(enum TargetSet targetSet) {
    double ret = 0;

    return ret;
}


/// <summary>
/// Returns the 3D distance between the TARGET_A and TARGET_B points.
/// Requires that two sets of corresponding points be loaded for both cameras to function correctly.
/// </summary>
/// <returns>measurement</returns>
double StereoCalibrationAndCalc::GetLengthBetweenPoints() {
    double ret = 0;

    return ret;
}


/// <summary>
/// Returns the 3D X coordinate for the indicated target.
/// For that function to work a corresponding point must be loaded for both cameras.
/// </summary>
/// <param name="targetSet"></param>
/// <returns></returns>
double StereoCalibrationAndCalc::Get3DX(enum TargetSet targetSet) {
    double ret = 0;

    return ret;
}


/// <summary>
/// Returns the 3D Y coordinate for the indicated target.
/// For that function to work a corresponding point must be loaded for both cameras.
/// </summary>
/// <param name="targetSet"></param>
/// <returns></returns>

double StereoCalibrationAndCalc::Get3DY(enum TargetSet targetSet) {
    double ret = 0;

    return ret;
}


/// <summary>
/// Returns the 3D Z coordinate for the indicated target.
/// For that function to work a corresponding point must be loaded for both cameras.
/// </summary>
/// <param name="targetSet"></param>
/// <returns></returns>

double StereoCalibrationAndCalc::Get3DZ(enum TargetSet targetSet) {
    double ret = 0;

    return ret;
}

