#include "StereoCalibration.h"
#include <fstream>
#include <opencv2/aruco/charuco.hpp>
#include "json.hpp"


StereoCalibration::StereoCalibration(int squaresX, int squaresY, float squareLength, float markerLength, int dictionaryId)
    : squaresX(squaresX), squaresY(squaresY), squareLength(squareLength), markerLength(markerLength), dictionaryId(dictionaryId) {}

void StereoCalibration::findCharucoCornersAndCalibrate(const vector<string>& imagesLeft, const vector<string>& imagesRight) {

    auto dictionary = cv::aruco::getPredefinedDictionary(dictionaryId);
    aruco::CharucoBoard board(cv::Size(squaresX, squaresY), squareLength, markerLength, dictionary);
    auto params = cv::aruco::DetectorParameters();
    auto charucoParams = cv::aruco::CharucoParameters();
    aruco::CharucoDetector detector(board, charucoParams, params);
    

    vector<Mat> allCharucoCornersL, allCharucoCornersR;
    vector<Mat> allCharucoIdsL, allCharucoIdsR;
    vector<Mat> allImagePointsL, allImagePointsR;
    vector<Mat> allObjectPointsL, allObjectPointsR;

    cv::Size imageSize;

    //cv::Mat cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR;
    Mat rvecsL, tvecsL, rvecsR, tvecsR;
    // Create a vector for this stdDeviationsIntrinsics
    std::vector<float> stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors;

    numberOfFrames = imagesLeft.size();

    for (size_t i = 0; i < imagesLeft.size(); i++) {
        cv::Mat imgL = cv::imread(imagesLeft[i]);
        cv::Mat imgR = cv::imread(imagesRight[i]);

        Mat currentCharucoCornersL, currentCharucoCornersR;
        Mat currentCharucoIdsL, currentCharucoIdsR;

        std::cout << "Detecting markers in image..." << std::endl;



        Mat currentObjectPointsL, currentObjectPointsR;
        Mat currentImagePointsL, currentImagePointsR;

        detector.detectBoard(imgL, currentCharucoCornersL, currentCharucoIdsL);
        cout << "Detected markers: " << currentCharucoCornersL.size() << endl;
        detector.detectBoard(imgR, currentCharucoCornersR, currentCharucoIdsR);

        cout << "Detected markers in left image: " << currentCharucoCornersL.size() << endl;

        board.matchImagePoints(currentCharucoCornersL, currentCharucoIdsL, currentObjectPointsL, currentImagePointsL);
        board.matchImagePoints(currentCharucoCornersR, currentCharucoIdsR, currentObjectPointsR, currentImagePointsR);

        cout << "Matched image points in left image: " << currentImagePointsL.size() << endl;

        // Draw detected markers on the images
        //cv::aruco::drawDetectedMarkers(imgL, currentCharucoCornersL);
        //cv::aruco::drawDetectedMarkers(imgR, currentCharucoCornersR);

        // Optionally display the images with detected markers
        cv::imshow("Detected Markers - Left Image", imgL);
        cv::imshow("Detected Markers - Right Image", imgR);
        //cv::waitKey(0);  // Wait for a key press to continue to the next pair of images

        if (!currentCharucoCornersL.empty() && !currentCharucoCornersR.empty()) {
            allCharucoCornersL.push_back(currentCharucoCornersL);
            allCharucoCornersR.push_back(currentCharucoCornersR);
            allCharucoIdsL.push_back(currentCharucoIdsL);
            allCharucoIdsR.push_back(currentCharucoIdsR);
            allImagePointsL.push_back(currentImagePointsL);
            allImagePointsR.push_back(currentImagePointsR);
            allObjectPointsR.push_back(currentObjectPointsR);
            allObjectPointsL.push_back(currentObjectPointsL);
        }

        imageSize = imgL.size();
    }


    cout << allObjectPointsR.size() << endl;
    

    cout << "Total number of image pairs processed: " << allImagePointsL.size() << endl;

    
    rmsL = calibrateCamera(allObjectPointsL, allImagePointsL, imageSize, cameraMatrixL, distCoeffsL, noArray(), noArray());
    //newCameraMatrixL = getOptimalNewCameraMatrix(cameraMatrixL, distCoeffsL, imageSize, 1);


    rmsR = calibrateCamera(allObjectPointsL, allImagePointsR, imageSize, cameraMatrixR, distCoeffsR, noArray(), noArray());
    //newCameraMatrixR = getOptimalNewCameraMatrix(cameraMatrixR, distCoeffsR, imageSize, 1);

    cout << "Calibrating stereo camera..." << endl;

    int flags = CALIB_FIX_INTRINSIC;
    TermCriteria criteria_stereo(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001);

    rmsStereo = stereoCalibrate(allObjectPointsL, allImagePointsL, allImagePointsR, newCameraMatrixL, distCoeffsL, newCameraMatrixR, distCoeffsR, imageSize, rot, trans, essentialMatrix, fundamentalMatrix, noArray(), flags);

    cout << rmsStereo << endl;
    cout << "Stereo rectification..." << endl;

    auto rectifyScale = 1; // 0=full crop, 1=no crop
    // Output 3x3 rectification transform (rotation matrix) for the first camera.
    
    cv::Mat R1, R2, P1, P2;

    // Calculate Q matrix for stereo rectification
    stereoRectify(cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR, imageSize, rot, trans, R1, R2, P1, P2, Q);

    for (size_t i = 0; i < imagesLeft.size(); i++) {
        cv::Mat imgL = cv::imread(imagesLeft[i]);
        cv::Mat imgR = cv::imread(imagesRight[i]);

        auto show_1 = imgL.clone();
        auto show_2 = imgR.clone();

        resize(show_1, show_1, Size(), 0.5, 0.5);
        resize(show_2, show_2, Size(), 0.5, 0.5);

        imshow("Left Image", show_1);
        imshow("Right Image", show_2);
        

        // After stereo rectification
        Mat map1L, map2L, map1R, map2R;
        initUndistortRectifyMap(cameraMatrixL, distCoeffsL, noArray(), cameraMatrixL, imageSize, CV_16SC2, map1L, map2L);
        initUndistortRectifyMap(cameraMatrixR, distCoeffsR, noArray(), cameraMatrixR, imageSize, CV_16SC2, map1R, map2R);

        // Assuming imgL and imgR are your left and right images
        Mat imgRectifiedL, imgRectifiedR;
        remap(imgL, imgRectifiedL, map1L, map2L, INTER_LINEAR);
        remap(imgR, imgRectifiedR, map1R, map2R, INTER_LINEAR);

        // Downscale the rectified images for display
        resize(imgRectifiedL, imgRectifiedL, Size(), 0.5, 0.5);
        resize(imgRectifiedR, imgRectifiedR, Size(), 0.5, 0.5);

        // Display the rectified images
        imshow("Rectified Left Image", imgRectifiedL);
        imshow("Rectified Right Image", imgRectifiedR);
        //waitKey(0);
    }
    
}


template<typename T>
vector<vector<T>> matToVector(const Mat& mat) {
    // Check if the matrix is empty

    vector<vector<T>> vec(mat.rows, vector<T>(mat.cols));
    for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; ++j) {
            vec[i][j] = mat.at<T>(i, j);
        }
    }
    return vec;
}


// Adjust your saveCalibrationData to use the proper data type
void StereoCalibration::saveCalibrationData(const string& filename) const {

    cout << cameraMatrixL << endl;
    cout << distCoeffsL << endl;
    cout << cameraMatrixR << endl;
    cout << distCoeffsR << endl;
    nlohmann::json calibrationData = {
        {"Description", "Stereo camera calibration using ChArUco markers"},
        {"LeftCalibrationCameraData", {
            {"RMS", rmsL},
            {"CameraMatrix", matToVector<double>(cameraMatrixL)},
            {"DistortionCoefficients", matToVector<double>(distCoeffsL)},
            {"ImageTotal", numberOfFrames},
            {"ImageUseable", numberOfFrames}
        }},
        {"RightCalibrationCameraData", {
            {"RMS", rmsR},
            {"CameraMatrix", matToVector<double>(cameraMatrixR)},
            {"DistortionCoefficients", matToVector<double>(distCoeffsR)},
            {"ImageTotal", numberOfFrames},
            {"ImageUseable", numberOfFrames}
        }},
        {"CalibrationStereoCameraData", {
            {"RMS", rmsStereo},
            {"Rotation", matToVector<double>(rot)},
            {"Translation", matToVector<double>(trans)},
            {"Q", matToVector<double>(Q)},
            {"ImageTotal", numberOfFrames},
            {"ImageUseable", numberOfFrames}
        }}
    };

    cout << calibrationData.dump(4) << endl;

    ofstream o(filename);
    o << setw(4) << calibrationData << endl;
    o.close();
}