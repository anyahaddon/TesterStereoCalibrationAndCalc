#include "StereoCalibration.h"
#include "StereoTriangulation.h"

int main() {
    int squaresX = 14; // Number of squares in X direction
    int squaresY = 9; // Number of squares in Y direction
    float squareLength = 0.04f; // Square length in meters
    float markerLength = 0.03f; // Marker length in meters
    int dictionaryId = aruco::DICT_5X5_250;

    vector<string> imagesLeft = {"/Users/nicolainielsen/Desktop/StereoCalibrationAndCalc/CalibrationFrames/043Charuco/left.png", "/Users/nicolainielsen/Desktop/StereoCalibrationAndCalc/CalibrationFrames/043Charuco/left2.png"};
    vector<string> imagesRight = {"/Users/nicolainielsen/Desktop/StereoCalibrationAndCalc/CalibrationFrames/043Charuco/right.png", "/Users/nicolainielsen/Desktop/StereoCalibrationAndCalc/CalibrationFrames/043Charuco/right2.png"};

    /*StereoCalibration calib(squaresX, squaresY, squareLength, markerLength, dictionaryId);
    cout << "Calibrating stereo cameras..." << endl;
    calib.findCharucoCornersAndCalibrate(imagesLeft, imagesRight);
    cout << "Calibration complete!" << endl;
    calib.saveCalibrationData("calibration_data.json");
    cout << "Calibration data saved to calibration_data.json" << endl;*/
    
    StereoTriangulation triang("calibration_data.json");
    // Example points in left and right images
    Point2f pointLeft(400, 120);
    Point2f pointRight(350, 120); // Example disparity of 10 pixels
    triang.calculateDepthFromPoints(pointLeft, pointRight);
    cout << "Triangulating points..." << endl;

    return 0;
}
