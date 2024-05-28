#pragma once
#include <string>


class StereoCalibrationAndCalc {
private:
	// Add suitable data structures to hold the calibration data

	// Add suitable data structures to hold the 2D points from the stereo cameras

public:
    bool LoadFromJSON(const std::string& jsonCalibrationData);
    bool SaveToJSON(std::string& jsonCalibrationData);
    bool LoadCalibIOFormatFromJSON(const std::string& jsonCalibrationData);

    bool Calibrate(const std::vector<std::string>& leftFileList, const std::vector<std::string>& rightFileList);

    
	enum TargetSet { TARGET_A, TARGET_B };
    enum CameraSide { LEFT, RIGHT };

    void LoadStereo2DPoint(enum TargetSet targetSet, enum CameraSide cameraSide, double X, double Y);


    double GetDistanceFromOrigin(enum TargetSet targetSet);
	double GetPrecision(enum TargetSet targetSet);
    double GetLengthBetweenPoints();

    double Get3DX(enum TargetSet targetSet);
    double Get3DY(enum TargetSet targetSet);
    double Get3DZ(enum TargetSet targetSet);
};

