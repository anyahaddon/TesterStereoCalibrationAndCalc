import json
import numpy as np 


class CalibrationCameraData:
    def __init__(self):
        super().__init__()
        
        self.retRMS = 0
        self.mtx = []
        self.dist = []
        imageTotal = 0
        imageUseable = 0

class CalibrationStereoCameraData:
    def __init__(self):
        super().__init__()
        
        self.retRMS = 0
        self.rotation = []
        self.translation = []       
        imageTotal = 0
        imageUseable = 0

class CalibrationData:
    def __init__(self):
        super().__init__()
                
        self.Description = ""
        
        self.LeftCalibrationCameraData = CalibrationCameraData()
        self.RightCalibrationCameraData = CalibrationCameraData()
        self.CalibrationStereoCameraData = CalibrationStereoCameraData()
        
    
    def SaveCameraCalibration(self, fileSpec):
            try:
                # Define a custom encoder to handle the numpy data types
                def custom_encoder(obj):
                    if isinstance(obj, CalibrationCameraData):
                        return {
                            "RMS": obj.retRMS,
                            "CameraMatrix": obj.mtx,
                            "DistortionCoefficients": obj.dist,    #.flatten().tolist() if obj.dist.size else []
                            "ImageTotal": obj.imageTotal,
                            "ImageUseable": obj.imageUseable

                        }
                    elif isinstance(obj, CalibrationStereoCameraData):
                        return {
                            "RMS": obj.retRMS,
                            "Rotation": obj.rotation,
                            "Translation": obj.translation,     #.flatten().tolist() if obj.translation.size else []
                            "ImageTotal": obj.imageTotal,
                            "ImageUseable": obj.imageUseable
                        }
                    elif isinstance(obj, np.ndarray):
                        return obj.tolist()  # Convert numpy array to Python list
                    raise TypeError(f"Object of type {type(obj)} is not JSON serializable")


                # Convert the Python object to JSON using the custom encoder
                json_data = json.dumps(self.__dict__, default=custom_encoder)

                # Write the JSON data to the specified file
                with open(fileSpec, "w") as json_file:
                    json_file.write(json_data)
                print(f"Calibration data saved to {fileSpec}")
                return True
            except Exception as e:
                print(f"Error saving calibration data: {e}")
                return False


    # def LoadCameraCalibration(self, fileSpec):
    #     with open(fileSpec, "r") as json_file:
    #         json_data = json.load(json_file)

    #     # Update the properties individually from the loaded JSON data
    #     self.Description = json_data.get("Description", "")
    #     self.LeftCalibrationCameraData = json_data.get("LeftCalibrationCameraData", None)
    #     self.RightCalibrationCameraData = json_data.get("RightCalibrationCameraData", None)
    #     self.CalibrationStereoCameraData = json_data.get("CalibrationStereoCameraData", None)
    
    def LoadCameraCalibration(self, fileSpec):
        with open(fileSpec, "r") as json_file:
            json_data = json.load(json_file)
            
        self.Description = json_data["Description"]
        
        # Left Camera Data
        left_data = json_data["LeftCalibrationCameraData"]
        self.LeftCalibrationCameraData.mtx = np.array(left_data["CameraMatrix"])
        self.LeftCalibrationCameraData.dist = np.array(left_data["DistortionCoefficients"])
        
        # Right Camera Data
        right_data = json_data["RightCalibrationCameraData"]
        self.RightCalibrationCameraData.mtx = np.array(right_data["CameraMatrix"])
        self.RightCalibrationCameraData.dist = np.array(right_data["DistortionCoefficients"])
        
        # Stereo Camera Data
        stereo_data = json_data["CalibrationStereoCameraData"]
        self.CalibrationStereoCameraData.retRMS = stereo_data["RMS"]
        self.CalibrationStereoCameraData.rotation = np.array(stereo_data["Rotation"])
        self.CalibrationStereoCameraData.translation = np.array(stereo_data["Translation"])
                
        print(f"Calibration data loaded from {fileSpec}")
        return True
        