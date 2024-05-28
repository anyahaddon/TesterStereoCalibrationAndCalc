#include "StereoCalibration.h"
#include <fstream>
#include <opencv2/aruco/charuco.hpp>
#include "json.hpp"
#include <chrono>//???


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

        //???std::cout << "Detecting markers in image..." << std::endl;



        Mat currentObjectPointsL, currentObjectPointsR;
        Mat currentImagePointsL, currentImagePointsR;

        detector.detectBoard(imgL, currentCharucoCornersL, currentCharucoIdsL);
        detector.detectBoard(imgR, currentCharucoCornersR, currentCharucoIdsR);
        std::cout << i << ": Detected markers in left image: " << currentCharucoCornersL.size() << ", right image : " << currentCharucoCornersR.size() << endl;


        board.matchImagePoints(currentCharucoCornersL, currentCharucoIdsL, currentObjectPointsL, currentImagePointsL);
        board.matchImagePoints(currentCharucoCornersR, currentCharucoIdsR, currentObjectPointsR, currentImagePointsR);

        std::cout << i << ": Matched image points in left image: " << currentImagePointsL.size() << "Corners:(L total=" << currentCharucoCornersL.total() << ", channels = " << currentCharucoCornersL.channels() << ")," << "(R total = " << currentCharucoCornersR.total() << ", channels = " << currentCharucoCornersR.channels() << ")" << endl;

      
        // Draw detected markers on the images
        try
        {
            cv::aruco::drawDetectedMarkers(imgL, currentCharucoCornersL, currentCharucoIdsL);
            cv::aruco::drawDetectedMarkers(imgR, currentCharucoCornersR, currentCharucoIdsR);
        }
		catch (const std::exception& e)
		{
			std::cerr << "drawDetectedMarkers:" << e.what() << '\n';
		}

        // Optionally display the images with detected markers
        // Determine the scale factor
        double scaleFactor = 0.2;  // Scale down from the original size

        // Calculate new dimensions
        int newWidth = static_cast<int>(imgL.cols * scaleFactor);
        int newHeight = static_cast<int>(imgL.rows * scaleFactor);

        // Resize the image
        cv::Mat imgLResized;
        cv::Mat imgRResized;
        cv::resize(imgL, imgLResized, cv::Size(newWidth, newHeight));
        cv::resize(imgR, imgRResized, cv::Size(newWidth, newHeight));


        cv::imshow("Detected Markers - Left Image", imgLResized);
        cv::imshow("Detected Markers - Right Image", imgRResized);
        cv::waitKey(500);  // Wait for a key press to continue to the next pair of images

		// Number of Image points and object points must match for stereo calibration
		// We use the object point from the left image for both images so currentObjectPointsL.size()
		// must be the same as both currentImagePointsL.size() and currentImagePointsR.size()
        if (currentObjectPointsL.size() != currentImagePointsL.size() ||
            currentObjectPointsL.size() != currentImagePointsR.size())
        {
			std::cerr << i << ": Error: Number of image points and object points must match for stereo calibration" << std::endl;
			continue;
        }


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


    std::cout << allObjectPointsR.size() << endl;
    

    std::cout << "***Total number of image pairs processed: " << allImagePointsL.size() << endl;

    
    rmsL = calibrateCamera(allObjectPointsL, allImagePointsL, imageSize, cameraMatrixL, distCoeffsL, noArray(), noArray());
    //newCameraMatrixL = getOptimalNewCameraMatrix(cameraMatrixL, distCoeffsL, imageSize, 1);


    rmsR = calibrateCamera(allObjectPointsR, allImagePointsR, imageSize, cameraMatrixR, distCoeffsR, noArray(), noArray());
    //newCameraMatrixR = getOptimalNewCameraMatrix(cameraMatrixR, distCoeffsR, imageSize, 1);

    std::cout << "Calibrating stereo camera..." << endl;

    int flags = CALIB_FIX_INTRINSIC;
    TermCriteria criteria_stereo(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001);

    rmsStereo = stereoCalibrate(allObjectPointsL, allImagePointsL, allImagePointsR, newCameraMatrixL, distCoeffsL, newCameraMatrixR, distCoeffsR, imageSize, rot, trans, essentialMatrix, fundamentalMatrix, noArray(), flags);

    std::cout << rmsStereo << endl;
    std::cout << "Stereo rectification..." << endl;

    auto rectifyScale = 1; // 0=full crop, 1=no crop
    // Output 3x3 rectification transform (rotation matrix) for the first camera.
    
    cv::Mat R1, R2, P1, P2;

    // Calculate Q matrix for stereo rectification
    cv::stereoRectify(cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR, imageSize, rot, trans, R1, R2, P1, P2, Q);

    rectificationMatrixL = R1;  // ADDITION
    rectificationMatrixR = R2;  // ADDITION
    projMatrixL = P1;  // ADDITION
    projMatrixR = P2;  // ADDITION

    // Extracting and calculating the absolute value of the baseline
    baseline = trans.at<double>(0, 0);

    if (baseline < 0) {
        baseline = -baseline;
    }



    auto start = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < imagesLeft.size(); i++) {
        cv::Mat imgL = cv::imread(imagesLeft[i]);
        cv::Mat imgR = cv::imread(imagesRight[i]);

        auto show_1 = imgL.clone();
        auto show_2 = imgR.clone();

        cv::resize(show_1, show_1, Size(), 0.5, 0.5);
        cv::resize(show_2, show_2, Size(), 0.5, 0.5);

        cv::imshow("Left Image", show_1);
        cv::imshow("Right Image", show_2);
        
        auto start = std::chrono::high_resolution_clock::now();

        // After stereo rectification
        Mat map1L, map2L, map1R, map2R;
        initUndistortRectifyMap(cameraMatrixL, distCoeffsL, noArray(), cameraMatrixL, imageSize, CV_16SC2, map1L, map2L);
        initUndistortRectifyMap(cameraMatrixR, distCoeffsR, noArray(), cameraMatrixR, imageSize, CV_16SC2, map1R, map2R);

        // Assuming imgL and imgR are your left and right images
        Mat imgRectifiedL, imgRectifiedR;
        cv::remap(imgL, imgRectifiedL, map1L, map2L, INTER_LINEAR);
        cv::remap(imgR, imgRectifiedR, map1R, map2R, INTER_LINEAR);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        std::cout << i << ": Time taken: " << duration.count() << " seconds" << std::endl;


        // Downscale the rectified images for display
        cv::resize(imgRectifiedL, imgRectifiedL, Size(), 0.5, 0.5);
        cv::resize(imgRectifiedR, imgRectifiedR, Size(), 0.5, 0.5);

        // Display the rectified images
        cv::imshow("Rectified Left Image", imgRectifiedL);
        cv::imshow("Rectified Right Image", imgRectifiedR);
        waitKey(500);
    }
    
}




/// <summary>
/// 
/// </summary>
/// <param name="imagesLeft"></param>
/// <param name="imagesRight"></param>
void StereoCalibration::findCharucoCornersAndCalibrate2(const vector<string>& imagesLeft, const vector<string>& imagesRight) {

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

        detector.detectBoard(imgL, currentCharucoCornersL, currentCharucoIdsL);
        detector.detectBoard(imgR, currentCharucoCornersR, currentCharucoIdsR);
        std::cout << i << ": Detected markers in left image: " << currentCharucoCornersL.size() << ", right image : " << currentCharucoCornersR.size() << endl;


        Mat currentObjectPointsL, currentObjectPointsR;
        Mat currentImagePointsL, currentImagePointsR;


        board.matchImagePoints(currentCharucoCornersL, currentCharucoIdsL, currentObjectPointsL, currentImagePointsL);
        board.matchImagePoints(currentCharucoCornersR, currentCharucoIdsR, currentObjectPointsR, currentImagePointsR);

//        std::cout << i << ": Matched image points in left image: " << currentImagePointsL.size() << "Corners:(L total=" << currentCharucoCornersL.total() << ", channels = " << currentCharucoCornersL.channels() << ")," << "(R total = " << currentCharucoCornersR.total() << ", channels = " << currentCharucoCornersR.channels() << ")" << endl;


        // Draw detected markers on the images
        try
        {
            cv::aruco::drawDetectedCornersCharuco(imgL, currentCharucoCornersL, currentCharucoIdsL);
            cv::aruco::drawDetectedCornersCharuco(imgR, currentCharucoCornersR, currentCharucoIdsR);
//???            cv::aruco::drawDetectedMarkers(imgL, currentCharucoCornersL, currentCharucoIdsL);
//???            cv::aruco::drawDetectedMarkers(imgR, currentCharucoCornersR, currentCharucoIdsR);
        }
        catch (const std::exception& e)
        {
            std::cerr << "drawDetectedMarkers:" << e.what() << '\n';
        }

        // Optionally display the images with detected markers
        // Determine the scale factor
        double scaleFactor = 0.4; // Scale down from the original size

        // Calculate new dimensions
        int newWidth = static_cast<int>(imgL.cols * scaleFactor);
        int newHeight = static_cast<int>(imgL.rows * scaleFactor);

        // Resize the image
        cv::Mat imgLResized;
        cv::Mat imgRResized;
        cv::resize(imgL, imgLResized, cv::Size(newWidth, newHeight));
        cv::resize(imgR, imgRResized, cv::Size(newWidth, newHeight));


        cv::imshow("Detected Markers - Left Image", imgLResized);
        cv::imshow("Detected Markers - Right Image", imgRResized);
        cv::waitKey(500);  // Wait for a key press to continue to the next pair of images

        // Number of Image points and object points must match for stereo calibration
        // We use the object point from the left image for both images so currentObjectPointsL.size()
        // must be the same as both currentImagePointsL.size() and currentImagePointsR.size()
        if (currentObjectPointsL.size() != currentImagePointsL.size() ||
            currentObjectPointsL.size() != currentImagePointsR.size())
        {
            std::cerr << i << ": Error: Number of image points and object points must match for stereo calibration" << std::endl;
            continue;
        }


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


    std::cout << allObjectPointsR.size() << endl;


    std::cout << "***Total number of image pairs processed: " << allImagePointsL.size() << endl;


    rmsL = calibrateCamera(allObjectPointsL, allImagePointsL, imageSize, cameraMatrixL, distCoeffsL, noArray(), noArray());
    //newCameraMatrixL = getOptimalNewCameraMatrix(cameraMatrixL, distCoeffsL, imageSize, 1);


    rmsR = calibrateCamera(allObjectPointsR, allImagePointsR, imageSize, cameraMatrixR, distCoeffsR, noArray(), noArray());
    //newCameraMatrixR = getOptimalNewCameraMatrix(cameraMatrixR, distCoeffsR, imageSize, 1);

    std::cout << "Calibrating stereo camera..." << endl;

    int flags = CALIB_FIX_INTRINSIC;
    TermCriteria criteria_stereo(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001);

    rmsStereo = stereoCalibrate(allObjectPointsL, allImagePointsL, allImagePointsR, newCameraMatrixL, distCoeffsL, newCameraMatrixR, distCoeffsR, imageSize, rot, trans, essentialMatrix, fundamentalMatrix, noArray(), flags);

    std::cout << rmsStereo << endl;
    std::cout << "Stereo rectification..." << endl;

    auto rectifyScale = 1; // 0=full crop, 1=no crop
    // Output 3x3 rectification transform (rotation matrix) for the first camera.

    cv::Mat R1, R2, P1, P2;

    // Calculate Q matrix for stereo rectification
    cv::stereoRectify(cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR, imageSize, rot, trans, R1, R2, P1, P2, Q);

    rectificationMatrixL = R1;  // ADDITION
    rectificationMatrixR = R2;  // ADDITION
    projMatrixL = P1;  // ADDITION
    projMatrixR = P2;  // ADDITION

    // Extracting and calculating the absolute value of the baseline
    baseline = trans.at<double>(0, 0);

    if (baseline < 0) {
        baseline = -baseline;
    }


    /*
    auto start = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < imagesLeft.size(); i++) {
        cv::Mat imgL = cv::imread(imagesLeft[i]);
        cv::Mat imgR = cv::imread(imagesRight[i]);

        auto show_1 = imgL.clone();
        auto show_2 = imgR.clone();

        cv::resize(show_1, show_1, Size(), 0.5, 0.5);
        cv::resize(show_2, show_2, Size(), 0.5, 0.5);

        cv::imshow("Left Image", show_1);
        cv::imshow("Right Image", show_2);

        auto start = std::chrono::high_resolution_clock::now();

        // After stereo rectification
        Mat map1L, map2L, map1R, map2R;
        initUndistortRectifyMap(cameraMatrixL, distCoeffsL, noArray(), cameraMatrixL, imageSize, CV_16SC2, map1L, map2L);
        initUndistortRectifyMap(cameraMatrixR, distCoeffsR, noArray(), cameraMatrixR, imageSize, CV_16SC2, map1R, map2R);

        // Assuming imgL and imgR are your left and right images
        Mat imgRectifiedL, imgRectifiedR;
        cv::remap(imgL, imgRectifiedL, map1L, map2L, INTER_LINEAR);
        cv::remap(imgR, imgRectifiedR, map1R, map2R, INTER_LINEAR);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        std::cout << i << ": Time taken: " << duration.count() << " seconds" << std::endl;


        // Downscale the rectified images for display
        cv::resize(imgRectifiedL, imgRectifiedL, Size(), 0.3, 0.3);
        cv::resize(imgRectifiedR, imgRectifiedR, Size(), 0.3, 0.3);

        // Display the rectified images
        cv::imshow("Rectified Left Image", imgRectifiedL);
        cv::imshow("Rectified Right Image", imgRectifiedR);
        waitKey(500);
    }
    */

    std::string dirPath = "C:/Users/tobyh/source/repos/TesterStereoCalibrationAndCalc/CalibrationFrames";
    string imagesLeftTarget = dirPath + "/../TestMeasurementFrames/044L([2392,2113] - [3052,2100] length=523mm)_00_03_57_83.png";
    string imagesRightTarget = dirPath + "/../TestMeasurementFrames/044R([1962,2171] - [2581,2206] length=523mm)_00_03_59_56.png";
    cv::Mat imgL = imread(imagesLeftTarget, IMREAD_COLOR);
    cv::Mat imgR = imread(imagesRightTarget, IMREAD_COLOR);

    auto show_1 = imgL.clone();
    auto show_2 = imgR.clone();

    cv::resize(show_1, show_1, Size(), 0.5, 0.5);
    cv::resize(show_2, show_2, Size(), 0.5, 0.5);

    cv::imshow("Left Image", show_1);
    cv::imshow("Right Image", show_2);

    // After stereo rectification
    Mat map1L, map2L, map1R, map2R;
    initUndistortRectifyMap(cameraMatrixL, distCoeffsL, noArray(), cameraMatrixL, imageSize, CV_16SC2, map1L, map2L);
    initUndistortRectifyMap(cameraMatrixR, distCoeffsR, noArray(), cameraMatrixR, imageSize, CV_16SC2, map1R, map2R);

    // Assuming imgL and imgR are your left and right images
    Mat imgRectifiedL, imgRectifiedR;
    cv::remap(imgL, imgRectifiedL, map1L, map2L, INTER_LINEAR);
    cv::remap(imgR, imgRectifiedR, map1R, map2R, INTER_LINEAR);

    // Save the undistorted image
    imwrite("c:\\temp\\testleftout2.png", imgRectifiedL);
    imwrite("c:\\temp\\testrightout2.png", imgRectifiedR);


    // Downscale the rectified images for display
    cv::resize(imgRectifiedL, imgRectifiedL, Size(), 0.3, 0.3);
    cv::resize(imgRectifiedR, imgRectifiedR, Size(), 0.3, 0.3);

    // Display the rectified images
    cv::imshow("Rectified Left Image", imgRectifiedL);
    cv::imshow("Rectified Right Image", imgRectifiedR);

	// Three different calcs only applicable if calib is with the 044 images
    Point2f pointsLRaw_A(2392.0, 2113.0);
    Point2f pointsRRaw_A(1962.0, 2171.0);
    Point2f pointsLRaw_B(3052.0, 2100.0);
    Point2f pointsRRaw_B(2581.0, 2206.0);

    float distance = calculateDistancePointsPairs(pointsLRaw_A, pointsRRaw_A, pointsLRaw_B, pointsRRaw_B);
    cout << "findCharucoCornersAndCalibrate2: If 044 Distance between points (NO Undistort): " << distance << " meters" << endl;

    Point2f pointsLUndist_A = undistortPoint2fL(pointsLRaw_A);
    Point2f pointsRUndist_A = undistortPoint2fR(pointsRRaw_A);
    Point2f pointsLUndist_B = undistortPoint2fL(pointsLRaw_B);
    Point2f pointsRUndist_B = undistortPoint2fR(pointsRRaw_B);

    distance = calculateDistancePointsPairs(pointsLUndist_A, pointsRUndist_A, pointsLUndist_B, pointsRUndist_B);
    cout << "findCharucoCornersAndCalibrate2: If 044 Distance between points (Undistort): " << distance << " meters" << endl;

	// On the rectified images
    Point2f pointsLRemapped_A(2387.0, 2131.0);
    Point2f pointsRRemapped_A(1942.0, 2215.0);
    Point2f pointsLRemapped_B(3065.0, 2121.0);
    Point2f pointsRRemapped_B(2580.0, 2214.0);

    distance = calculateDistancePointsPairs(pointsLRemapped_A, pointsRRemapped_A, pointsLRemapped_B, pointsRRemapped_B);
    cout << "findCharucoCornersAndCalibrate2: If 044 Distance between points (Points from the rectified image): " << distance << " meters" << endl;


    // Two different calcs only applicable if calib is with the 043 images
    // 043 Left [2563,1815] - [3018,1808]
	// 043 Right [2414,1903] - [2868,1913]
    Point2f pointsLRaw_A2(2563.0, 1815.0);
    Point2f pointsRRaw_A2(2414.0, 1903.0);
    Point2f pointsLRaw_B2(3018.0, 1808.0);
    Point2f pointsRRaw_B2(2868.0, 1913.0);

    distance = calculateDistancePointsPairs(pointsLRaw_A2, pointsRRaw_A2, pointsLRaw_B2, pointsRRaw_B2);
    cout << "findCharucoCornersAndCalibrate2: If 034 Distance between points (NO Undistort): " << distance << " meters" << endl;

    Point2f pointsLUndist_A2 = undistortPoint2fL(pointsLRaw_A2);
    Point2f pointsRUndist_A2 = undistortPoint2fR(pointsRRaw_A2);
    Point2f pointsLUndist_B2 = undistortPoint2fL(pointsLRaw_B2);
    Point2f pointsRUndist_B2 = undistortPoint2fR(pointsRRaw_B2);

    distance = calculateDistancePointsPairs(pointsLUndist_A2, pointsRUndist_A2, pointsLUndist_B2, pointsRUndist_B2);
    cout << "findCharucoCornersAndCalibrate2: If 034 Distance between points (Undistort): " << distance << " meters" << endl;

    waitKey(0);


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




/// <summary>
/// TEMP ADDITION
/// </summary>
/// <param name="inputImageLeft"></param>
/// <returns></returns>

Mat StereoCalibration::undistortAndRemapLeft(const Mat& inputImageLeft) {
    Mat undistortedImage;

    if (mapL1.empty()) {
        initUndistortRectifyMap(cameraMatrixL, distCoeffsL, rectificationMatrixL, projMatrixL, inputImageLeft.size(), CV_16SC2/*CV_32FC1???*/, mapL1, mapL2);

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

Mat StereoCalibration::undistortAndRemapRight(const Mat& inputImageRight) {
    Mat undistortedImage;

    if (mapR1.empty()) {
        initUndistortRectifyMap(cameraMatrixR, distCoeffsR, rectificationMatrixR, projMatrixR, inputImageRight.size(), CV_16SC2/*CV_32FC1???*/, mapR1, mapR2);
    }


    remap(inputImageRight, undistortedImage, mapR1, mapR2, INTER_LINEAR);

    return undistortedImage;
}


Point2f StereoCalibration::undistortPoint2fL(const Point2f& pointDistorted) {

    // Undistort points
    vector<Point2f> pts_distorted = { pointDistorted };
    vector<Point2f> pts_undistorted;

    undistortPoints(pts_distorted, pts_undistorted, cameraMatrixL/*K1*/, distCoeffsL/*D1*/, rectificationMatrixL/*R1*/, projMatrixL/*P1*/);

    // The undistorted and rectified points
    Point2f pt = pts_undistorted[0];

    return pt;
}

Point2f StereoCalibration::undistortPoint2fR(const Point2f& pointDistorted) {

    // Undistort points
    vector<Point2f> pts_distorted = { pointDistorted };
    vector<Point2f> pts_undistorted;

    undistortPoints(pts_distorted, pts_undistorted, cameraMatrixR/*K1*/, distCoeffsR/*D1*/, rectificationMatrixR/*R1*/, projMatrixR/*P1*/);

    // The undistorted and rectified points
    Point2f pt = pts_undistorted[0];

    return pt;
}


// Calculate the depth from two image points
void StereoCalibration::calculateDepthFromPoints(const Point2f& pointL, const Point2f& pointR) {
    double disparity = pointL.x - pointR.x;
    if (disparity == 0) {
        cout << "Invalid disparity (zero). Depth cannot be calculated." << endl;
        return;
    }

    double focalLength = cameraMatrixL.at<double>(0, 0); // fx value from the camera matrix
    double depth = (focalLength * baseline) / disparity;

    cout << "Calculated Depth: " << depth << " meters" << endl;
}



float StereoCalibration::calculateDistancePointsPairs(const Point2f& pointsL_A, const Point2f& pointsR_A, const Point2f& pointsL_B, const Point2f& pointsR_B) {

    Vec3f pt_3D_A = calculate2DTo3D(pointsL_A, pointsR_A);
    Vec3f pt_3D_B = calculate2DTo3D(pointsL_B, pointsR_B);

    float distance = calculateDistance(pt_3D_A, pt_3D_B);

    return distance;
}



Vec3f StereoCalibration::calculate2DTo3D(const Point2f& pt_left, const Point2f& pt_right) {

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



float StereoCalibration::calculateDistance(const Vec3f& pt_3D_A, const Vec3f& pt_3D_B) {
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
