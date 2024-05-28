#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>
#include "StereoCalibrationAndCalc.h"
#include <regex>


namespace fs = std::filesystem;  // Use namespace alias for simplicity

void loadFileList(const std::string& searchPath, std::vector<std::string>& leftFileList);
bool writeStringToFile(const std::string& filename, const std::string& content);
std::string readStringFromFile(const std::string& filename);


int main()
{
	// Search paths for the calibration frames
	std::string searchPath043FramesLeft  = "..\\CalibrationFrames\\043Charuco\\043L*.png";
	std::string searchPath043FramesRight = "..\\CalibrationFrames\\043Charuco\\043R*.png";
	std::string searchPath034FramesLeft  = "..\\CalibrationFrames\\034Circles3m\\034Circles3mL*.jpeg";
	std::string searchPath034FramesRight = "..\\CalibrationFrames\\034Circles3m\\034Circles3mR*.jpeg";
	std::string searchPath044CharucoFramesLeft = "..\\CalibrationFrames\\044Charuco\\044L*.png";
	std::string searchPath044CharucoFramesRight = "..\\CalibrationFrames\\044Charuco\\044R*.png";
	std::string searchPath044CirclesFramesLeft = "..\\CalibrationFrames\\044Circles\\044L*.png";
	std::string searchPath044CirclesFramesRight = "..\\CalibrationFrames\\044Circles\\044R*.png";


	StereoCalibrationAndCalc stereoCalibrationAndCalc;
	std::vector<std::string> leftFileList, rightFileList;
	std::string jsonCalibrationData = "";



	while (true)
	{

		std::cout << "1. Calibrate using the 043 frames set, calibration frames containing Charuco 9x14, 400mmx600mm, DICT_5X5, Checker 40mm, Marker 30mm. (Shot inside, low light)" << std::endl;
		std::cout << "2. Calibrate using the 034 frames set, calibration frames containing Asymmetric Circles 4x11, Vertical or Hortizontal Distance Between Circles 54.6mm (Shot in water)" << std::endl;
		std::cout << "3. Calibrate using the 044 frames set, calibration frames containing Charuco 9x14, 400mmx600mm, DICT_5X5, Checker 40mm, Marker 30mm (Shot outside)" << std::endl;
		std::cout << "4. Calibrate using the 044 frames set, calibration frames containing Asymmetric Circles 4x11, Vertical or Hortizontal Distance Between Circles 54.6mm (Shot outside)" << std::endl;
		std::cout << "5. Save calibration JSON to file." << std::endl;
		std::cout << "6. Load calibration JSON from file." << std::endl;
		std::cout << "7. Test Points from set 043. Get Length,Distance and X,Y,Z." << std::endl;
		std::cout << "8. Test Points from set 034. Get Length,Distance and X,Y,Z." << std::endl;
		std::cout << "9. Test Points from set 044. Get Length,Distance and X,Y,Z." << std::endl;
		
		std::cout << "x.Exit." << std::endl;
		std::cout << std::endl;

		char c = getchar();


		if (c == '1')
		{
			// Calibrate using 043 frames set
			loadFileList(searchPath043FramesLeft, leftFileList);
			loadFileList(searchPath043FramesRight, rightFileList);

			// Returns the calibration data in JSON format
			if (stereoCalibrationAndCalc.Calibrate(leftFileList, rightFileList) == true)
				std::cout << "Successful calilbration." << std::endl;
			else
				std::cout << "Unsuccessful calilbration." << std::endl;

		}
		else if (c == '2')
		{
			// Calibrate using 034 frames set
			loadFileList(searchPath034FramesLeft, leftFileList);
			loadFileList(searchPath034FramesRight, rightFileList);

			// Returns the calibration data in JSON format
			if (stereoCalibrationAndCalc.Calibrate(leftFileList, rightFileList) == true)
				std::cout << "Successful calilbration." << std::endl;
			else
				std::cout << "Unsuccessful calilbration." << std::endl;
		}
		else if (c == '3')
		{
			// Calibrate using 044 Charuco frames set
			loadFileList(searchPath044CharucoFramesLeft, leftFileList);
			loadFileList(searchPath044CharucoFramesRight, rightFileList);

			// Returns the calibration data in JSON format
			if (stereoCalibrationAndCalc.Calibrate(leftFileList, rightFileList) == true)
				std::cout << "Successful calilbration." << std::endl;
			else
				std::cout << "Unsuccessful calilbration." << std::endl;
		}
		else if (c == '4')
		{
			// Calibrate using 044 Circles frames set
			loadFileList(searchPath044CirclesFramesLeft, leftFileList);
			loadFileList(searchPath044CirclesFramesRight, rightFileList);

			// Returns the calibration data in JSON format
			if (stereoCalibrationAndCalc.Calibrate(leftFileList, rightFileList) == true)
				std::cout << "Successful calilbration." << std::endl;
			else
				std::cout << "Unsuccessful calilbration." << std::endl;
		}
		else if (c == '5')
		{
			// Save calibration JSON to file			
			if (stereoCalibrationAndCalc.SaveToJSON(jsonCalibrationData) == true)
			{
				std::cout << "JSON Successfully created." << std::endl;
				if (writeStringToFile("Calibration data.json", jsonCalibrationData) == true)
					std::cout << "JSON Successfully saved to file." << std::endl;
				else
					std::cout << "JSON not saved to file." << std::endl;
			}
			else
				std::cout << "JSON not created." << std::endl;
		}
		else if (c == '6')
		{
			// Load calibration JSON from file
			if (stereoCalibrationAndCalc.LoadFromJSON(jsonCalibrationData) == true)
			{
				std::cout << "JSON Successfully loaded." << std::endl;

				jsonCalibrationData = readStringFromFile("Calibration data.json");
				if (jsonCalibrationData != "")
					std::cout << "JSON Successfully read from file." << std::endl;
				else
					std::cout << "JSON not read from file." << std::endl;
			}
			else
				std::cout << "JSON not loaded." << std::endl;
		}
		else if (c == '7')
		{
			std::cout << "*** Ensure calibration for either 043Charuco is loaded ***" << std::endl;
			// Test Points from set 043. Get Length,Distance and X,Y,Z
			
			// Camera Left: These Points are from a image "\StereoCalibrationAndCalc\TestMeasurementFrames\043L([2563,1815] - [3018,1808] length=523mm)_00_02_31_30.png"
			stereoCalibrationAndCalc.LoadStereo2DPoint(StereoCalibrationAndCalc::TARGET_A, StereoCalibrationAndCalc::LEFT, 2563.0, 1815.0);
			stereoCalibrationAndCalc.LoadStereo2DPoint(StereoCalibrationAndCalc::TARGET_B, StereoCalibrationAndCalc::LEFT, 3018.0, 1808.0);

			// Camera Right: These Points are from a image "\StereoCalibrationAndCalc\TestMeasurementFrames\043R([2414,1903] - [2868,1913] length=523mm)_00_02_32_96.png"
			stereoCalibrationAndCalc.LoadStereo2DPoint(StereoCalibrationAndCalc::TARGET_A, StereoCalibrationAndCalc::RIGHT, 2414.0, 1903.0);
			stereoCalibrationAndCalc.LoadStereo2DPoint(StereoCalibrationAndCalc::TARGET_B, StereoCalibrationAndCalc::RIGHT, 2868.0, 1913.0);


			std::cout << "Distance from origin to 3D point of target set A: " << stereoCalibrationAndCalc.GetDistanceFromOrigin(StereoCalibrationAndCalc::TARGET_A) << std::endl;
			std::cout << "Distance from origin to 3D point of target set B: " << stereoCalibrationAndCalc.GetDistanceFromOrigin(StereoCalibrationAndCalc::TARGET_B) << std::endl;

			std::cout << "Precision of target set A: " << stereoCalibrationAndCalc.GetPrecision(StereoCalibrationAndCalc::TARGET_A) << std::endl;
			std::cout << "Precision of target set B: " << stereoCalibrationAndCalc.GetPrecision(StereoCalibrationAndCalc::TARGET_B) << std::endl;

			std::cout << "Length between points: " << stereoCalibrationAndCalc.GetLengthBetweenPoints() << ", we are hoping for ~523mm" << std::endl;
		}
		else if (c == '8')
		{
			std::cout << "*** Ensure calibration for either 034Circles is loaded ***" << std::endl;
			// Test Points from set 034. Get Length,Distance and X,Y,Z
			// 
			// Camera Left: These Points are from a image "\StereoCalibrationAndCalc\TestMeasurementFrames\034L ([2003,1495] - [3422,1341] length=560mm)_00_03_40_63.png"
			stereoCalibrationAndCalc.LoadStereo2DPoint(StereoCalibrationAndCalc::TARGET_A, StereoCalibrationAndCalc::LEFT, 2003.0, 1495.0);
			stereoCalibrationAndCalc.LoadStereo2DPoint(StereoCalibrationAndCalc::TARGET_B, StereoCalibrationAndCalc::LEFT, 3422.0, 1341.0);

			// Camera Right: These Points are from a image "\StereoCalibrationAndCalc\TestMeasurementFrames\034R ([769,1462] - [1709,1407] length=560mm)_00_03_32_39.png" 
			stereoCalibrationAndCalc.LoadStereo2DPoint(StereoCalibrationAndCalc::TARGET_A, StereoCalibrationAndCalc::RIGHT, 769.0, 1462.0);
			stereoCalibrationAndCalc.LoadStereo2DPoint(StereoCalibrationAndCalc::TARGET_B, StereoCalibrationAndCalc::RIGHT, 1709.0, 1407.0);


			std::cout << "Distance from origin to 3D point of target set A: " << stereoCalibrationAndCalc.GetDistanceFromOrigin(StereoCalibrationAndCalc::TARGET_A) << std::endl;
			std::cout << "Distance from origin to 3D point of target set B: " << stereoCalibrationAndCalc.GetDistanceFromOrigin(StereoCalibrationAndCalc::TARGET_B) << std::endl;

			std::cout << "Precision of target set A: " << stereoCalibrationAndCalc.GetPrecision(StereoCalibrationAndCalc::TARGET_A) << std::endl;
			std::cout << "Precision of target set B: " << stereoCalibrationAndCalc.GetPrecision(StereoCalibrationAndCalc::TARGET_B) << std::endl;

			std::cout << "Length between points: " << stereoCalibrationAndCalc.GetLengthBetweenPoints() << ", we are hoping for ~560mm" << std::endl;

		}
		else if (c == '9')
		{
			std::cout << "*** Ensure calibration for either 044Charuco or 044Circles is loaded ***" << std::endl;
			// Test Points from set 044. Get Length,Distance and X,Y,Z
			// 
			// Camera Left: These Points are from a image "\StereoCalibrationAndCalc\TestMeasurementFrames\044L([2392,2113] - [3052,2100] length=523mm)_00_03_57_83.png"
			stereoCalibrationAndCalc.LoadStereo2DPoint(StereoCalibrationAndCalc::TARGET_A, StereoCalibrationAndCalc::LEFT, 2392.0, 2113.0);
			stereoCalibrationAndCalc.LoadStereo2DPoint(StereoCalibrationAndCalc::TARGET_B, StereoCalibrationAndCalc::LEFT, 3052.0, 2100.0);

			// Camera Right: These Points are from a image "\StereoCalibrationAndCalc\TestMeasurementFrames\044R([1962,2171] - [2581,2206] length=523mm)_00_03_59_56.png" 
			stereoCalibrationAndCalc.LoadStereo2DPoint(StereoCalibrationAndCalc::TARGET_A, StereoCalibrationAndCalc::RIGHT, 1962.0, 2171.0);
			stereoCalibrationAndCalc.LoadStereo2DPoint(StereoCalibrationAndCalc::TARGET_B, StereoCalibrationAndCalc::RIGHT, 2581.0, 2206.0);


			std::cout << "Distance from origin to 3D point of target set A: " << stereoCalibrationAndCalc.GetDistanceFromOrigin(StereoCalibrationAndCalc::TARGET_A) << std::endl;
			std::cout << "Distance from origin to 3D point of target set B: " << stereoCalibrationAndCalc.GetDistanceFromOrigin(StereoCalibrationAndCalc::TARGET_B) << std::endl;

			std::cout << "Precision of target set A: " << stereoCalibrationAndCalc.GetPrecision(StereoCalibrationAndCalc::TARGET_A) << std::endl;
			std::cout << "Precision of target set B: " << stereoCalibrationAndCalc.GetPrecision(StereoCalibrationAndCalc::TARGET_B) << std::endl;

			std::cout << "Length between points: " << stereoCalibrationAndCalc.GetLengthBetweenPoints() << ", we are hoping for ~523mm" << std::endl;

			}
		else if (c == 'x' || c == 'X')
		{
			break;
		}
	}
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


bool writeStringToFile(const std::string& filename, const std::string& content) {
	
	bool ret = true;

	// Create an ofstream instance that will handle the file writing
	std::ofstream fileStream(filename, std::ios::out | std::ios::trunc); // std::ios::trunc mode to ensure the file is cleared if it already exists

	// Check if the file stream is successfully opened
	if (!fileStream.is_open()) {
		std::cerr << "Failed to open file: " << filename << std::endl;
		return false;  // Make sure to return false if file opening fails
	}

	// Write the content to the file
	fileStream << content;

	// Optionally check for write errors
	if (!fileStream.good()) {
		std::cerr << "Error occurred while writing to the file: " << filename << std::endl;
		ret = false;
	}

	// Close the file stream
	fileStream.close();

	return ret;
}

std::string readStringFromFile(const std::string& filename) {
	// Create an ifstream instance to handle the file reading
	std::ifstream fileStream(filename);

	// Check if the file stream is successfully opened
	if (!fileStream.is_open()) {
		std::cerr << "Failed to open file: " << filename << std::endl;
		return "";  // Return empty string on failure to open file
	}

	// Read the contents of the file
	std::stringstream buffer;
	buffer << fileStream.rdbuf();  // Read the buffer of the file into a stringstream

	// Close the file stream
	fileStream.close();

	// Return the contents of the stringstream which contains the file data
	return buffer.str();
}

