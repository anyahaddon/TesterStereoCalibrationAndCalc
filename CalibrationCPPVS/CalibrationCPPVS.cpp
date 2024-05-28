// CalibrationCPPVS.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <sys/stat.h>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>  // compile with c++17
#include <regex>

#include "StereoCalibration.h"
#include "StereoTriangulation.h"


namespace fs = std::filesystem;

std::chrono::milliseconds extractTime(const std::string& fileName);
void loadFileList(const std::string& searchPattern, std::vector<std::string>& fileList);

int main() {

    int squaresX = 14; // Number of squares in X direction
    int squaresY = 9; // Number of squares in Y direction
    float squareLength = 0.04f; // Square length in meters
    float markerLength = 0.03f; // Marker length in meters
    int dictionaryId = cv::aruco::DICT_5X5_250;

    // Paths
    std::string dirPathNicolai = "c:/Users/nicolainielsen/Desktop/StereoCalibrationAndCalc/CalibrationFrames";
	std::string dirPathToby = "C:/Users/tobyh/source/repos/TesterStereoCalibrationAndCalc/CalibrationFrames";
	std::string dirPath;

    // Find if Toby or Nicolai
    struct stat info;
    if (stat(dirPathNicolai.c_str(), &info) != 0) {
        dirPath = dirPathToby;
    }
    else if (info.st_mode & S_IFDIR) {
        dirPath = dirPathNicolai;
    }

    // Example: "C:/Users/tobyh/source/repos/TesterStereoCalibrationAndCalc/CalibrationFrames/044Charuco/044L(Rig V3, On Land, New A2 Charuco and A3 Circles Test)_00_01_30_98.png";


    // Do calibration
	std::string imagesLeftSearch = dirPath + "/044Charuco/044L*.png"; // Path to the left images
	std::string imagesRightSearch = dirPath + "/044Charuco/044R*.png"; // Path to the right images
	//???std::string imagesLeftSearch = dirPath + "/043Charuco/left*.png"; // Path to the left images
	//???std::string imagesRightSearch = dirPath + "/043Charuco/right*.png"; // Path to the right images
	//std::string imagesLeftSearch = dirPath + "/043Charuco/043L*.png"; // Path to the left images
	//std::string imagesRightSearch = dirPath + "/043Charuco/043R*.png"; // Path to the right images

	std::vector<std::string> imagesLeft;
	std::vector<std::string> imagesRight;

	// Load images
	loadFileList(imagesLeftSearch, imagesLeft);
	loadFileList(imagesRightSearch, imagesRight);

	for (size_t i = 0; i < imagesLeft.size(); i++) {
	
		fs::path pathLeft(imagesLeft[i]);
		fs::path pathRight(imagesRight[i]);

		auto time1 = extractTime(pathLeft.filename().string());
		auto time2 = extractTime(pathRight.filename().string());

		// Calculate the time difference
		auto difference = time2 - time1;

		// Convert the difference to seconds
		double secondsDifference = std::chrono::duration<double>(difference).count();


		std::cout << i << ": File L:" << pathLeft.filename().string() << ", R:" << pathRight.filename().string() << ", diff:" << secondsDifference << " secs" << std::endl;
	}


    StereoCalibration calib(squaresX, squaresY, squareLength, markerLength, dictionaryId);
	//goto enter;

    cout << "Calibrating stereo cameras..." << endl;
    calib.findCharucoCornersAndCalibrate2(imagesLeft, imagesRight);
    cout << "Calibration complete!" << endl;
    calib.saveCalibrationData("calibration_data.json");
    cout << "Calibration data saved to calibration_data.json" << endl;


enter:

    StereoTriangulation triang("calibration_data.json");
    // Example points in left and right images
    cv::Point2f pointLeft(400, 120);
	cv::Point2f pointRight(350, 120); // Example disparity of 10 pixels
    triang.calculateDepthFromPoints(pointLeft, pointRight);
    cout << "Triangulating points..." << endl;




    Point2f pointsLRaw_A(2392.0, 2113.0);
    Point2f pointsRRaw_A(1962.0, 2171.0);
    Point2f pointsLRaw_B(3052.0, 2100.0);
    Point2f pointsRRaw_B(2581.0, 2206.0);

    float distance = calib.calculateDistancePointsPairs(pointsLRaw_A, pointsRRaw_A, pointsLRaw_B, pointsRRaw_B);
	cout << "Distance between points (NO Undistort): " << distance << " meters" << endl;

	Point2f pointsLUndist_A = calib.undistortPoint2fL(pointsLRaw_A);
	Point2f pointsRUndist_A = calib.undistortPoint2fR(pointsRRaw_A);
	Point2f pointsLUndist_B = calib.undistortPoint2fL(pointsLRaw_B);
	Point2f pointsRUndist_B = calib.undistortPoint2fR(pointsRRaw_B);

	distance = calib.calculateDistancePointsPairs(pointsLUndist_A, pointsRUndist_A, pointsLUndist_B, pointsRUndist_B);
	cout << "Distance between points (Undistort): " << distance << " meters" << endl;

	string imagesLeftTarget = dirPath + "/../TestMeasurementFrames/044L([2392,2113] - [3052,2100] length=523mm)_00_03_57_83.png";
	string imagesRightTarget = dirPath + "/../TestMeasurementFrames/044R([1962,2171] - [2581,2206] length=523mm)_00_03_59_56.png";

	Mat inputImageLeft = imread(imagesLeftTarget, IMREAD_COLOR);
	Mat inputImageRight = imread(imagesRightTarget, IMREAD_COLOR);

	Mat outputImageLeft = calib.undistortAndRemapLeft(inputImageLeft);
	Mat outputImageRight = calib.undistortAndRemapLeft(inputImageRight);

	// Save the undistorted image
	imwrite("c:\\temp\\testleftout.png", outputImageLeft);
	imwrite("c:\\temp\\testrightout.png", outputImageRight);

    return 0;
}


std::chrono::milliseconds extractTime(const std::string& fileName) {
	// Define a regular expression to match the time format (hh_mm_ss_hh)
	std::regex timeRegex(R"(_(\d{2})_(\d{2})_(\d{2})_(\d{2})\.png$)");
	std::smatch match;

	// Check if the filename matches the regular expression
	if (std::regex_search(fileName, match, timeRegex)) {
		if (match.size() == 5) { // 1 whole match + 4 groups
			int hours = std::stoi(match[1].str());
			int minutes = std::stoi(match[2].str());
			int seconds = std::stoi(match[3].str());
			int hundredths = std::stoi(match[4].str());

			// Convert the extracted time to milliseconds
			std::chrono::milliseconds duration =
				std::chrono::hours(hours) +
				std::chrono::minutes(minutes) +
				std::chrono::seconds(seconds) +
				std::chrono::milliseconds(hundredths * 10);

			return duration;
		}
	}

	throw std::invalid_argument("Filename does not match the expected format.");
}

void loadFileList(const std::string& searchPattern, std::vector<std::string>& fileList) {
	fileList.clear();  // Clear the file list before starting

	// Extract directory path and file pattern from the search pattern
	std::regex pattern(R"(^(.*[\\\/])([^\\\/]+)$)");
	std::smatch matches;

	if (!std::regex_match(searchPattern, matches, pattern) || matches.size() != 3) {
		std::cerr << "Invalid search pattern: " << searchPattern << std::endl;
		return;
	}

	fs::path directoryPath = matches[1].str();
	std::string filenamePattern = matches[2].str();
	std::string extension = fs::path(filenamePattern).extension().string();

	try {
		if (fs::is_directory(directoryPath)) {
			for (const auto& entry : fs::directory_iterator(directoryPath)) {
				const auto& path = entry.path();
				const auto& filename = path.filename().string();

				// Use regex to match the file pattern (convert wildcard to regex)
				std::regex fileRegex(std::regex_replace(filenamePattern, std::regex(R"(\*)"), R"(.*)") + "$");

				if (entry.is_regular_file() &&
					std::regex_match(filename, fileRegex) &&
					path.extension() == extension) {
					fileList.push_back(path.string());  // Add the full path to the list
				}
			}
		}
		else {
			std::cerr << "Provided path is not a directory: " << directoryPath << std::endl;
		}
	}
	catch (const fs::filesystem_error& e) {
		std::cerr << "Filesystem error: " << e.what() << std::endl;
	}
	catch (const std::exception& e) {
		std::cerr << "General error: " << e.what() << std::endl;
	}
}

