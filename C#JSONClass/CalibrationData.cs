using MathNet.Numerics.LinearAlgebra;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System;
using System.Collections.Generic;
using System.Linq;
using Emgu.CV;
using Emgu.CV.Structure;
using System.Windows.Media;

namespace SurveyorV1
{
    /// <summary>
    /// CalibrationData Version 1.3 
    /// Added RMS to CalibrationCameraData
    /// Moved alway from MathNET to use native Emgu types
    /// </summary>
    public class CalibrationCameraData
    {
        [JsonProperty(nameof(RMS))]
        public double RMS { get; set; }

        [JsonProperty("CameraMatrix")]
        [JsonConverter(typeof(MatrixJsonConverter))]
        public Emgu.CV.Matrix<double>? Mtx { get; set; }   // Camera Matrix 3x3
        
        
        [JsonProperty("DistortionCoefficients")]
        [JsonConverter(typeof(MatrixJsonConverter))]
        public Emgu.CV.Matrix<double>? Dist { get; set; }   // Distortion coefficient Matrix 4x1


        [JsonProperty(nameof(ImageTotal))]
        public int ImageTotal { get; set; }     // Total number of images supplied to the calibration process

        [JsonProperty(nameof(ImageUseable))]
        public int ImageUseable { get; set; }   // Number of images that were possible to use in the calibration process

        public override int GetHashCode()
        {
            return HashCode.Combine(RMS, Mtx, Dist, ImageTotal, ImageUseable);
        }


        public override bool Equals(object? obj)
        {
            if (obj is CalibrationCameraData other)
            {
                return RMS == other.RMS &&
                       ImageTotal == other.ImageTotal &&
                       ImageUseable == other.ImageUseable &&
                       (Mtx == null && other.Mtx == null || Mtx?.Equals(other.Mtx) == true) &&
                       (Dist == null && other.Dist == null || Dist?.Equals(other.Dist) == true);
            }
            return false;
        }

        public static bool operator ==(CalibrationCameraData left, CalibrationCameraData right)
        {

            return EqualityComparer<CalibrationCameraData>.Default.Equals(left, right);
        }

        public static bool operator !=(CalibrationCameraData left, CalibrationCameraData right)
        {
            return !(left == right);
        }

        public void Clear()
        {
            RMS = 0;
            Mtx = null;
            Dist = null;
            ImageTotal = 0;
            ImageUseable = 0;           
        }
    }

    public class CalibrationCameraData_Legacy
    {
        [JsonProperty(nameof(RMS))]
        public double RMS { get; set; }

        [JsonProperty("CameraMatrix")]
        [JsonConverter(typeof(MatrixJsonConverter_Legacy))]
        public MathNet.Numerics.LinearAlgebra.Matrix<double>? Mtx { get; set; }


        [JsonProperty("DistortionCoefficients")]
        [JsonConverter(typeof(VectorJsonConverter_Legacy))]
        public MathNet.Numerics.LinearAlgebra.Vector<double>? Dist { get; set; }


        [JsonProperty(nameof(ImageTotal))]
        public int ImageTotal { get; set; }     // Total number of images supplied to the calibration process

        [JsonProperty(nameof(ImageUseable))]
        public int ImageUseable { get; set; }   // Number of images that were possible to use in the calibration process


        public override int GetHashCode()
        {
            return HashCode.Combine(RMS, Mtx, Dist, ImageTotal, ImageUseable);            
        }

        public override bool Equals(object? obj)
        {
            if (obj is CalibrationCameraData_Legacy other)
            {
                return RMS == other.RMS &&
                       ImageTotal == other.ImageTotal &&
                       ImageUseable == other.ImageUseable &&
                       (Mtx == null && other.Mtx == null || Mtx?.Equals(other.Mtx) == true) &&
                       (Dist == null && other.Dist == null || Dist?.Equals(other.Dist) == true);
            }
            return false;
        }

        public static bool operator ==(CalibrationCameraData_Legacy left, CalibrationCameraData_Legacy right)
        {

            return EqualityComparer<CalibrationCameraData_Legacy>.Default.Equals(left, right);
        }

        public static bool operator !=(CalibrationCameraData_Legacy left, CalibrationCameraData_Legacy right)
        {
            return !(left == right);
        }
    }


    public class CalibrationStereoCameraData
    {
        [JsonProperty(nameof(RMS))]
        public double RMS { get; set; } = -1;


        [JsonProperty(nameof(Rotation))]
        [JsonConverter(typeof(MatrixJsonConverter))]
        public Emgu.CV.Matrix<double>? Rotation { get; set; }       // Rotation matrix 3x3


        [JsonProperty(nameof(Translation))]
        [JsonConverter(typeof(MatrixJsonConverter))]
        public Emgu.CV.Matrix<double>? Translation { get; set; }     // Translation vector 3x1


        [JsonProperty(nameof(ImageTotal))]
        public int ImageTotal { get; set; }     // Total number of images supplied to the calibration process

        [JsonProperty(nameof(ImageUseable))]
        public int ImageUseable { get; set; }   // Number of images that were possible to use in the calibration process



        public override int GetHashCode()
        {
            return HashCode.Combine(RMS, Rotation, Translation, ImageTotal, ImageUseable);
        }

        public override bool Equals(object? obj)
        {
            if (obj is CalibrationStereoCameraData other)
            {
                return RMS == other.RMS &&
                       ImageTotal == other.ImageTotal &&
                       ImageUseable == other.ImageUseable &&
                       (Rotation == null && other.Rotation == null || Rotation?.Equals(other.Rotation) == true) &&
                       (Translation == null && other.Translation == null || Translation?.Equals(other.Translation) == true);
            }
            return false;
        }

        public static bool operator ==(CalibrationStereoCameraData left, CalibrationStereoCameraData right)
        {

            return EqualityComparer<CalibrationStereoCameraData>.Default.Equals(left, right);
        }

        public static bool operator !=(CalibrationStereoCameraData left, CalibrationStereoCameraData right)
        {
            return !(left == right);
        }

        public void Clear()
        {
            RMS = 0;
            Rotation = null;
            Translation = null;
            ImageTotal = 0;
            ImageUseable = 0;
        }
    }

    public class CalibrationStereoCameraData_Legacy
    {
        [JsonProperty(nameof(RMS))]
        public double RMS { get; set; } = -1;


        [JsonProperty(nameof(Rotation))]
        [JsonConverter(typeof(MatrixJsonConverter_Legacy))]
        public MathNet.Numerics.LinearAlgebra.Matrix<double>? Rotation { get; set; }


        [JsonProperty(nameof(Translation))]
        [JsonConverter(typeof(VectorJsonConverter_Legacy))]
        public MathNet.Numerics.LinearAlgebra.Vector<double>? Translation { get; set; }


        [JsonProperty(nameof(ImageTotal))]
        public int ImageTotal { get; set; }     // Total number of images supplied to the calibration process

        [JsonProperty(nameof(ImageUseable))]
        public int ImageUseable { get; set; }   // Number of images that were possible to use in the calibration process



        public override int GetHashCode()
        {
            return HashCode.Combine(RMS, Rotation, Translation, ImageTotal, ImageUseable);
        }

        public override bool Equals(object? obj)
        {
            if (obj is CalibrationStereoCameraData_Legacy other)
            {
                return RMS == other.RMS &&
                       ImageTotal == other.ImageTotal &&
                       ImageUseable == other.ImageUseable &&
                       (Rotation == null && other.Rotation == null || Rotation?.Equals(other.Rotation) == true) &&
                       (Translation == null && other.Translation == null || Translation?.Equals(other.Translation) == true);
            }
            return false;
        }

        public static bool operator ==(CalibrationStereoCameraData_Legacy left, CalibrationStereoCameraData_Legacy right)
        {

            return EqualityComparer<CalibrationStereoCameraData_Legacy>.Default.Equals(left, right);
        }

        public static bool operator !=(CalibrationStereoCameraData_Legacy left, CalibrationStereoCameraData_Legacy right)
        {
            return !(left == right);
        }
    }


    public class CalibrationData
    {
        public string Description { get; set; } = "";

        public CalibrationCameraData LeftCalibrationCameraData { get; set; } = new();
        public CalibrationCameraData RightCalibrationCameraData { get; set; } = new();
        public CalibrationStereoCameraData CalibrationStereoCameraData { get; set; } = new();


        public CalibrationData()
        {
        }


        /// <summary>
        /// Load the calibration data from a JSON file
        /// </summary>
        /// <param name="fileSpec"></param>
        /// <returns></returns>
        public int LoadFromFile(string fileSpec)
        {
            int ret = 0;

            try
            {
                // Reset
                Clear();

                string json = System.IO.File.ReadAllText(fileSpec);

                ret = LoadFromJson(json);
            }
            catch (Exception ex)
            {
                // Log the exception details
                Console.WriteLine($"Error loading calibration data: {ex.Message}");
                return -2; // Error code for exception
            }

            return ret;
        }


        /// <summary>
        /// Load the calibration data from a JSON string
        /// </summary>
        /// <param name="json"></param>
        /// <returns></returns>
        public int LoadFromJson(string json)
        {
            int ret = 0;

            try
            {
                // Reset
                Clear();

                var settings = new JsonSerializerSettings();
                settings.Converters.Add(new MatrixJsonConverter());
                settings.Converters.Add(new VectorJsonConverter());

                var calibrationData = JsonConvert.DeserializeObject<CalibrationData>(json, settings);

#if DEBUG
                if (calibrationData is not null)
                {
                    Console.WriteLine($"Description:{calibrationData.Description}");
                    Console.WriteLine($"Left:");
                    if (calibrationData.LeftCalibrationCameraData.Mtx is not null)
                        Console.WriteLine($"   Camera Matrix: {calibrationData.LeftCalibrationCameraData.Mtx.ToString()}");
                    if (calibrationData.LeftCalibrationCameraData.Dist is not null)
                        Console.WriteLine($"   Distortion Coefficients: {calibrationData.LeftCalibrationCameraData.Dist.ToString()}");

                    Console.WriteLine($"Right:");
                    if (calibrationData.RightCalibrationCameraData.Mtx is not null)
                        Console.WriteLine($"   Camera Matrix: {calibrationData.RightCalibrationCameraData.Mtx.ToString()}");
                    if (calibrationData.RightCalibrationCameraData.Dist is not null)
                        Console.WriteLine($"   Distortion Coefficients: {calibrationData.RightCalibrationCameraData.Dist.ToString()}");

                    Console.WriteLine("Stereo:");
                    Console.WriteLine($"   RMS: {calibrationData.CalibrationStereoCameraData.RMS}");
                    if (calibrationData.CalibrationStereoCameraData.Rotation is not null)
                        Console.WriteLine($"   Rotation: {calibrationData.CalibrationStereoCameraData.Rotation.ToString()}");
                    if (calibrationData.CalibrationStereoCameraData.Translation is not null)
                        Console.WriteLine($"   Translation: {calibrationData.CalibrationStereoCameraData.Translation.ToString()}");
                }
#endif

                // Load this class
                if (calibrationData is not null)
                {
                    Description = calibrationData.Description;

                    if (ret == 0)
                    {
                        if (calibrationData.LeftCalibrationCameraData.Mtx is not null)
                            LeftCalibrationCameraData.Mtx = calibrationData.LeftCalibrationCameraData.Mtx;
                        else
                        {
                            Console.WriteLine("Calibration.Load()   LeftCalibrationCameraData.Mtx is null");
                            ret = -4;
                        }

                        if (calibrationData.LeftCalibrationCameraData.Dist is not null)
                            LeftCalibrationCameraData.Dist = calibrationData.LeftCalibrationCameraData.Dist;
                        else
                        {
                            Console.WriteLine("Calibration.Load()   LeftCalibrationCameraData.Dist is null");
                            ret = -5;
                        }

                        LeftCalibrationCameraData.RMS = calibrationData.LeftCalibrationCameraData.RMS;
                        LeftCalibrationCameraData.ImageTotal = calibrationData.LeftCalibrationCameraData.ImageTotal;
                        LeftCalibrationCameraData.ImageUseable = calibrationData.LeftCalibrationCameraData.ImageUseable;
                    }


                    if (ret == 0)
                    {
                        if (calibrationData.RightCalibrationCameraData.Mtx is not null)
                            RightCalibrationCameraData.Mtx = calibrationData.RightCalibrationCameraData.Mtx;
                        else
                        {
                            Console.WriteLine("Calibration.Load()   RightCalibrationCameraData.Mtx is null");
                            ret = -8;
                        }

                        if (calibrationData.RightCalibrationCameraData.Dist is not null)
                            RightCalibrationCameraData.Dist = calibrationData.RightCalibrationCameraData.Dist;
                        else
                        {
                            Console.WriteLine("Calibration.Load()   RightCalibrationCameraData.Dist is null");
                            ret = -9;
                        }

                        RightCalibrationCameraData.RMS = calibrationData.RightCalibrationCameraData.RMS;
                        RightCalibrationCameraData.ImageTotal = calibrationData.RightCalibrationCameraData.ImageTotal;
                        RightCalibrationCameraData.ImageUseable = calibrationData.RightCalibrationCameraData.ImageUseable;
                    }

                    if (ret == 0)
                    {
                        CalibrationStereoCameraData.RMS = calibrationData.CalibrationStereoCameraData.RMS;
                        CalibrationStereoCameraData.ImageTotal = calibrationData.CalibrationStereoCameraData.ImageTotal;
                        CalibrationStereoCameraData.ImageUseable = calibrationData.CalibrationStereoCameraData.ImageUseable;

                        if (calibrationData.CalibrationStereoCameraData.Rotation is not null)
                            CalibrationStereoCameraData.Rotation = calibrationData.CalibrationStereoCameraData.Rotation;
                        else
                        {
                            Console.WriteLine("Calibration.Load()   CalibrationStereoCameraData.Rotation is null");
                            ret = -12;
                        }

                        if (calibrationData.CalibrationStereoCameraData.Translation is not null)
                            CalibrationStereoCameraData.Translation = calibrationData.CalibrationStereoCameraData.Translation;
                        else
                        {
                            Console.WriteLine("Calibration.Load()   CalibrationStereoCameraData.Translation is null");
                            ret = -13;
                        }
                    }
                }
                else
                {
                    Console.WriteLine("Calibration.Load()   calibrationData is null");
                    ret = -1;
                }

            }
            catch (Exception ex)
            {
                // Log the exception details
                Console.WriteLine($"Error loading calibration data: {ex.Message}");
                return -2; // Error code for exception
            }

            return ret;
        }


        /// <summary>
        /// Save the calibration data to a JSON file
        /// </summary>
        /// <param name="fileSpec"></param>
        /// <returns></returns>
        public int SaveToFile(string fileSpec)
        {
            int ret = 0;

            try
            {
                string json;

                ret = SaveToJson(out json);

                if (ret == 0)
                    System.IO.File.WriteAllText(fileSpec, json);
            }
            catch (Exception ex)
            {
                // Log the exception details
                Console.WriteLine($"Error saving calibration data: {ex.Message}");
                return -2; // Error code for exception
            }

            return ret;
        }


        /// <summary>
        /// Save the calibration data to a JSON string
        /// </summary>
        /// <param name="json"></param>
        /// <returns></returns>
        public int SaveToJson(out string json)
        {
            int ret = 0;

            // Reset
            json = "";

            try
            {
                var settings = new JsonSerializerSettings
                {
                    Formatting = Formatting.None,      //.Indented, // For pretty-printing the JSON
                    Converters = new List<JsonConverter> { new MatrixJsonConverter(), new VectorJsonConverter() }
                };

                json = JsonConvert.SerializeObject(this, settings);
            }
            catch (Exception ex)
            {
                // Log the exception details
                Console.WriteLine($"Error saving calibration data: {ex.Message}");
                return -2; // Error code for exception
            }

            return ret;
        }


        /// <summary>
        /// Clear all values in the CalibrationData object
        /// </summary>
        public void Clear()
        {
            Description = "";
            //??? Create Clear() function in CalibrationCameraData() and CalibrationStereoCameraData() classes
            LeftCalibrationCameraData.Clear();
            RightCalibrationCameraData.Clear();
            CalibrationStereoCameraData.Clear();
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(Description,
                                    LeftCalibrationCameraData.GetHashCode(),
                                    RightCalibrationCameraData.GetHashCode(),
                                    CalibrationStereoCameraData.GetHashCode());
        }

        public override bool Equals(object? obj)
        {
            if (obj is CalibrationData_Legacy other)
            {
                return Description == other.Description &&
                       LeftCalibrationCameraData.Equals(other.LeftCalibrationCameraData) &&
                       RightCalibrationCameraData.Equals(other.RightCalibrationCameraData) &&
                       CalibrationStereoCameraData.Equals(other.CalibrationStereoCameraData);
            }
            return false;
        }

        public static bool operator ==(CalibrationData left, CalibrationData right)
        {

            return EqualityComparer<CalibrationData>.Default.Equals(left, right);
        }

        public static bool operator !=(CalibrationData left, CalibrationData right)
        {
            return !(left == right);
        }
    }


    public class CalibrationData_Legacy
    {
        public string Description { get; set; } = "";

        public CalibrationCameraData_Legacy LeftCalibrationCameraData { get; set; } = new();
        public CalibrationCameraData_Legacy RightCalibrationCameraData { get; set; } = new();
        public CalibrationStereoCameraData_Legacy CalibrationStereoCameraData { get; set; } = new();


        public CalibrationData_Legacy()
        {
        }


        /// <summary>
        /// Load the calibration data from a JSON file
        /// </summary>
        /// <param name="fileSpec"></param>
        /// <returns></returns>
        public int LoadFromFile(string fileSpec)
        {
            int ret = 0;

            try
            {
                // Reset
                Clear();

                string json = System.IO.File.ReadAllText(fileSpec);

                ret = LoadFromJson(json);
            }
            catch (Exception ex)
            {
                // Log the exception details
                Console.WriteLine($"Error loading calibration data: {ex.Message}");
                return -2; // Error code for exception
            }

            return ret;
        }


        /// <summary>
        /// Load the calibration data from a JSON string
        /// </summary>
        /// <param name="json"></param>
        /// <returns></returns>
        public int LoadFromJson(string json)
        {
            int ret = 0;

            try
            {
                // Reset
                Clear();

                var settings = new JsonSerializerSettings();
                settings.Converters.Add(new MatrixJsonConverter_Legacy());
                settings.Converters.Add(new VectorJsonConverter_Legacy());

                var calibrationData = JsonConvert.DeserializeObject<CalibrationData_Legacy>(json, settings);

#if DEBUG
                if (calibrationData is not null)
                {
                    Console.WriteLine($"Description:{calibrationData.Description}");
                    Console.WriteLine($"Left:");
                    if (calibrationData.LeftCalibrationCameraData.Mtx is not null)
                        Console.WriteLine($"   Camera Matrix: {calibrationData.LeftCalibrationCameraData.Mtx.ToString()}");
                    if (calibrationData.LeftCalibrationCameraData.Dist is not null)
                        Console.WriteLine($"   Distortion Coefficients: {calibrationData.LeftCalibrationCameraData.Dist.ToString()}");

                    Console.WriteLine($"Right:");
                    if (calibrationData.RightCalibrationCameraData.Mtx is not null)
                        Console.WriteLine($"   Camera Matrix: {calibrationData.RightCalibrationCameraData.Mtx.ToString()}");
                    if (calibrationData.RightCalibrationCameraData.Dist is not null)
                        Console.WriteLine($"   Distortion Coefficients: {calibrationData.RightCalibrationCameraData.Dist.ToString()}");

                    Console.WriteLine("Stereo:");
                    Console.WriteLine($"   RMS: {calibrationData.CalibrationStereoCameraData.RMS}");
                    if (calibrationData.CalibrationStereoCameraData.Rotation is not null)
                        Console.WriteLine($"   Rotation: {calibrationData.CalibrationStereoCameraData.Rotation.ToString()}");
                    if (calibrationData.CalibrationStereoCameraData.Translation is not null)
                        Console.WriteLine($"   Translation: {calibrationData.CalibrationStereoCameraData.Translation.ToString()}");
                }
#endif

                // Load this class
                if (calibrationData is not null)
                {
                    Description = calibrationData.Description;

                    if (ret == 0)
                    {
                        if (calibrationData.LeftCalibrationCameraData.Mtx is not null)
                            LeftCalibrationCameraData.Mtx = calibrationData.LeftCalibrationCameraData.Mtx;
                        else
                        {
                            Console.WriteLine("Calibration.Load()   LeftCalibrationCameraData.Mtx is null");
                            ret = -4;
                        }

                        if (calibrationData.LeftCalibrationCameraData.Dist is not null)
                            LeftCalibrationCameraData.Dist = calibrationData.LeftCalibrationCameraData.Dist;
                        else
                        {
                            Console.WriteLine("Calibration.Load()   LeftCalibrationCameraData.Dist is null");
                            ret = -5;
                        }

                        LeftCalibrationCameraData.RMS = calibrationData.LeftCalibrationCameraData.RMS;
                        LeftCalibrationCameraData.ImageTotal = calibrationData.LeftCalibrationCameraData.ImageTotal;
                        LeftCalibrationCameraData.ImageUseable = calibrationData.LeftCalibrationCameraData.ImageUseable;
                    }


                    if (ret == 0)
                    {
                        if (calibrationData.RightCalibrationCameraData.Mtx is not null)
                            RightCalibrationCameraData.Mtx = calibrationData.RightCalibrationCameraData.Mtx;
                        else
                        {
                            Console.WriteLine("Calibration.Load()   RightCalibrationCameraData.Mtx is null");
                            ret = -8;
                        }

                        if (calibrationData.RightCalibrationCameraData.Dist is not null)
                            RightCalibrationCameraData.Dist = calibrationData.RightCalibrationCameraData.Dist;
                        else
                        {
                            Console.WriteLine("Calibration.Load()   RightCalibrationCameraData.Dist is null");
                            ret = -9;
                        }

                        RightCalibrationCameraData.RMS = calibrationData.RightCalibrationCameraData.RMS;
                        RightCalibrationCameraData.ImageTotal = calibrationData.RightCalibrationCameraData.ImageTotal;
                        RightCalibrationCameraData.ImageUseable = calibrationData.RightCalibrationCameraData.ImageUseable;
                    }

                    if (ret == 0)
                    {
                        CalibrationStereoCameraData.RMS = calibrationData.CalibrationStereoCameraData.RMS;
                        CalibrationStereoCameraData.ImageTotal = calibrationData.CalibrationStereoCameraData.ImageTotal;
                        CalibrationStereoCameraData.ImageUseable = calibrationData.CalibrationStereoCameraData.ImageUseable;

                        if (calibrationData.CalibrationStereoCameraData.Rotation is not null)
                            CalibrationStereoCameraData.Rotation = calibrationData.CalibrationStereoCameraData.Rotation;
                        else
                        {
                            Console.WriteLine("Calibration.Load()   CalibrationStereoCameraData.Rotation is null");
                            ret = -12;
                        }

                        if (calibrationData.CalibrationStereoCameraData.Translation is not null)
                            CalibrationStereoCameraData.Translation = calibrationData.CalibrationStereoCameraData.Translation;
                        else
                        {
                            Console.WriteLine("Calibration.Load()   CalibrationStereoCameraData.Translation is null");
                            ret = -13;
                        }
                    }
                }
                else
                {
                    Console.WriteLine("Calibration.Load()   calibrationData is null");
                    ret = -1;
                }

            }
            catch (Exception ex)
            {
                // Log the exception details
                Console.WriteLine($"Error loading calibration data: {ex.Message}");
                return -2; // Error code for exception
            }

            return ret;
        }


        /// <summary>
        /// Save the calibration data to a JSON file
        /// </summary>
        /// <param name="fileSpec"></param>
        /// <returns></returns>
        public int SaveToFile(string fileSpec)
        {
            int ret = 0;

            try
            {
                string json;

                ret = SaveToJson(out json);

                if (ret == 0)
                    System.IO.File.WriteAllText(fileSpec, json);
            }
            catch (Exception ex)
            {
                // Log the exception details
                Console.WriteLine($"Error saving calibration data: {ex.Message}");
                return -2; // Error code for exception
            }

            return ret;
        }


        /// <summary>
        /// Save the calibration data to a JSON string
        /// </summary>
        /// <param name="json"></param>
        /// <returns></returns>
        public int SaveToJson(out string json)
        {
            int ret = 0;

            // Reset
            json = "";

            try
            {
                var settings = new JsonSerializerSettings
                {
                    Formatting = Formatting.None,      //.Indented, // For pretty-printing the JSON
                    Converters = new List<JsonConverter> { new MatrixJsonConverter_Legacy(), new VectorJsonConverter_Legacy() }
                };

                json = JsonConvert.SerializeObject(this, settings);                
            }
            catch (Exception ex)
            {
                // Log the exception details
                Console.WriteLine($"Error saving calibration data: {ex.Message}");
                return -2; // Error code for exception
            }

            return ret;
        }


        /// <summary>
        /// Clear all values in the CalibrationData object
        /// </summary>
        public void Clear()
        {
            Description = "";
            //??? Create Clear() function in CalibrationCameraData() and CalibrationStereoCameraData() classes
            LeftCalibrationCameraData.Mtx = null;
            LeftCalibrationCameraData.Dist = null;
            LeftCalibrationCameraData.RMS = 0;
            LeftCalibrationCameraData.ImageTotal = 0;
            LeftCalibrationCameraData.ImageUseable = 0;
            RightCalibrationCameraData.Mtx = null;
            RightCalibrationCameraData.Dist = null;
            RightCalibrationCameraData.RMS = 0;
            RightCalibrationCameraData.ImageTotal = 0;
            RightCalibrationCameraData.ImageUseable = 0;
            CalibrationStereoCameraData.RMS = 0;
            CalibrationStereoCameraData.ImageTotal = 0;
            CalibrationStereoCameraData.ImageUseable = 0;
            CalibrationStereoCameraData.Rotation = null;
            CalibrationStereoCameraData.Translation = null;
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(Description, 
                                    LeftCalibrationCameraData.GetHashCode(), 
                                    RightCalibrationCameraData.GetHashCode(), 
                                    CalibrationStereoCameraData.GetHashCode());
        }

        public override bool Equals(object? obj)
        {
            if (obj is CalibrationData_Legacy other)
            {
                return Description == other.Description &&
                       LeftCalibrationCameraData.Equals(other.LeftCalibrationCameraData) &&
                       RightCalibrationCameraData.Equals(other.RightCalibrationCameraData) &&
                       CalibrationStereoCameraData.Equals(other.CalibrationStereoCameraData);
            }
            return false;
        }

        public static bool operator ==(CalibrationData_Legacy left, CalibrationData_Legacy right)
        {
           
            return EqualityComparer<CalibrationData_Legacy>.Default.Equals(left, right);
        }

        public static bool operator !=(CalibrationData_Legacy left, CalibrationData_Legacy right)
        {
            return !(left == right);
        }
    }



    public class MatrixJsonConverter : JsonConverter
    {
        public override bool CanConvert(Type objectType)
        {
            return objectType == typeof(Emgu.CV.Matrix<double>);
        }

        public override object? ReadJson(JsonReader reader, Type objectType, object? existingValue, JsonSerializer serializer)
        {
            Emgu.CV.Matrix<double>? matrix = null;

            var array = JArray.Load(reader);

            if (array is not null && array.Count > 0 && array[0].HasValues)
            {
                int rows = array.Count;
                int cols = array[0].Count();
                matrix = new Emgu.CV.Matrix<double>(rows, cols);

                for (int i = 0; i < rows; i++)
                {
                    for (int j = 0; j < cols; j++)
                    {
#pragma warning disable CS8604
                        matrix[i, j] = (double)array[i][j];
#pragma warning restore CS8604
                    }
                }
            }

            return matrix;
        }

        public override void WriteJson(JsonWriter writer, object? value, JsonSerializer serializer)
        {
            Emgu.CV.Matrix<double>? matrix = value as Emgu.CV.Matrix<double>;

            if (matrix is not null)
            {
                var rows = matrix.Rows;
                var cols = matrix.Cols;

                writer.WriteStartArray();
                for (int i = 0; i < rows; i++)
                {
                    writer.WriteStartArray();
                    for (int j = 0; j < cols; j++)
                    {
                        writer.WriteValue(matrix[i, j]);
                    }
                    writer.WriteEndArray();
                }
                writer.WriteEndArray();
            }
        }
    }

    public class MatrixJsonConverter_Legacy : JsonConverter
    {
        public override bool CanConvert(Type objectType)
        {
            return objectType == typeof(MathNet.Numerics.LinearAlgebra.Matrix<double>);
        }

        public override object? ReadJson(JsonReader reader, Type objectType, object? existingValue, JsonSerializer serializer)
        {
            MathNet.Numerics.LinearAlgebra.Matrix<double>? matrix = null;

            var array = JArray.Load(reader);

            if (array is not null)
            {
                matrix = MathNet.Numerics.LinearAlgebra.Matrix<double>.Build.Dense(array.Count, array[0].Count());

                for (int i = 0; i < array.Count; i++)
                {
                    for (int j = 0; j < array[i].Count(); j++)
                    {
#pragma warning disable CS8604
                        matrix[i, j] = (double)array[i][j];
#pragma warning restore CS8604
                    }
                }
            }

            return matrix;
        }

        public override void WriteJson(JsonWriter writer, object? value, JsonSerializer serializer)
        {
            MathNet.Numerics.LinearAlgebra.Matrix<double>? matrix = (MathNet.Numerics.LinearAlgebra.Matrix<double>?)value;

            if (matrix is not null)
            {
                var rows = matrix.RowCount;
                var cols = matrix.ColumnCount;

                writer.WriteStartArray();
                for (int i = 0; i < rows; i++)
                {
                    writer.WriteStartArray();
                    for (int j = 0; j < cols; j++)
                    {
                        writer.WriteValue(matrix[i, j]);
                    }
                    writer.WriteEndArray();
                }
                writer.WriteEndArray();
            }
        }
    }


    public class VectorJsonConverter : JsonConverter
    {
        public override bool CanConvert(Type objectType)
        {
            return objectType == typeof(MCvPoint3D32f);
        }

        public override object ReadJson(JsonReader reader, Type objectType, object? existingValue, JsonSerializer serializer)
        {
            var array = JArray.Load(reader);

            if (array != null && array.Count == 3)
            {
                return new MCvPoint3D32f(
                    (float)array[0],
                    (float)array[1],
                    (float)array[2]);
            }
            else
            {
                throw new JsonSerializationException("Unexpected JSON structure for MCvPoint3D64f deserialization.");
            }
        }

        public override void WriteJson(JsonWriter writer, object? value, JsonSerializer serializer)
        {
            var vector = value as MCvPoint3D32f?;

            if (vector != null)
            {
                writer.WriteStartArray();
                writer.WriteValue(vector.Value.X);
                writer.WriteValue(vector.Value.Y);
                writer.WriteValue(vector.Value.Z);
                writer.WriteEndArray();
            }
        }
    }



    public class VectorJsonConverter_Legacy : JsonConverter
    {
        public override bool CanConvert(Type objectType)
        {
            return objectType == typeof(Vector<double>);
        }

        public override object ReadJson(JsonReader reader, Type objectType, object? existingValue, JsonSerializer serializer)
        {
            var outerArray = JArray.Load(reader);

            // Determine the structure of the array and process accordingly
            if (outerArray.Count > 0 && outerArray.First is JArray firstInnerArray)
            {
                if (outerArray.Count == 1)
                {
                    // Single inner array with multiple elements (DistortionCoefficients)
                    return ProcessOneDimensionalArray(firstInnerArray);
                }
                else
                {
                    // Multiple inner arrays with single element (Translation)
                    return ProcessTwoDimensionalArray(outerArray);
                }
            }
            else
            {
                throw new JsonSerializationException("Unexpected JSON structure for vector deserialization.");
            }
        }

        private Vector<double> ProcessOneDimensionalArray(JArray array)
        {
            var vector = Vector<double>.Build.Dense(array.Count);
            for (int i = 0; i < array.Count; i++)
            {
                vector[i] = (double)array[i];
            }
            return vector;
        }

        private Vector<double> ProcessTwoDimensionalArray(JArray array)
        {
            var vector = Vector<double>.Build.Dense(array.Count);
            for (int i = 0; i < array.Count; i++)
            {
                var innerArray = array[i] as JArray;
                if (innerArray != null && innerArray.Count == 1)
                {
#pragma warning disable CS8604
                    vector[i] = (double)innerArray.First;
#pragma warning restore CS8604
                }
                else
                {
                    throw new JsonSerializationException("Each inner array should contain exactly one element.");
                }
            }
            return vector;
        }

        public override void WriteJson(JsonWriter writer, object? value, JsonSerializer serializer)
        {
            var vector = (Vector<double>?)value;

            if (vector is not null)
            {
                // Assuming "Translation" vectors will always have fewer elements than "DistortionCoefficients" vectors
                // Adjust this condition based on your specific data
                if (vector.Count <= 3) // Likely a "Translation" vector
                {
                    writer.WriteStartArray();
                    for (int i = 0; i < vector.Count; i++)
                    {
                        writer.WriteStartArray(); // Start inner array for each element
                        writer.WriteValue(vector[i]);
                        writer.WriteEndArray(); // End inner array
                    }
                    writer.WriteEndArray();
                }
                else // Likely a "DistortionCoefficients" vector
                {
                    writer.WriteStartArray();
                    writer.WriteStartArray(); // Start single inner array
                    for (int i = 0; i < vector.Count; i++)
                    {
                        writer.WriteValue(vector[i]);
                    }
                    writer.WriteEndArray(); // End single inner array
                    writer.WriteEndArray();
                }
            }
        }
    }
}

