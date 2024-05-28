using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using System;
using System.Drawing;
using System.IO;
using System.Windows;


namespace SurveyorV1
{
    /// <summary>
    /// StereoProjection Version 1.2
    /// This class is used to calculate the distance between a pair of corresponding 2D points in the left and right images
    /// It intentionally uses a mixture of Emgu.CV and MathNET.Numerics types and also System.Drawing (where the System.Windows.Point and System.Drawing.Point types are not compatible)
    /// </summary>

    public class StereoProjection
    {
        private Reporter report;
        private Project.DataClass.CalibrationClass calibrationClass;

        public StereoProjection(Reporter _report, Project.DataClass.CalibrationClass _calibrationClass)
        {
            report = _report;
            calibrationClass = _calibrationClass;
        }


        /// <summary>
        /// Calculate the distance between a pair of corresponding 2D points in the left and right images
        /// If there are no calibrartion data instance available, then return -1
        /// If there are multiple calibrartion data instances available then only return calculations based of the preferred calbration data instance, 
        /// the calculations from any other calibration data instance will be returned in the output window.
        /// </summary>
        /// <param name="LPointA"></param>
        /// <param name="LPointB"></param>
        /// <param name="RPointA"></param>
        /// <param name="RPointB"></param>
        /// <returns></returns>
        public double Distance(System.Windows.Point? LPointA, System.Windows.Point? LPointB, System.Windows.Point? RPointA, System.Windows.Point? RPointB, bool ReportDepth)
        {
            double distance = 0;

            if (calibrationClass is not null && calibrationClass.CalibrationDataList is not null)
            {
                if (LPointA is not null && RPointA is not null && LPointB is not null && RPointB is not null)
                {
                    // Calculate the distance between the two 3D points using the preferred calibration data instance
                    CalibrationData? calibrationDataPreferred = calibrationClass.CalibrationDataList[calibrationClass.PreferredCalibrationDataIndex];
                    if (calibrationDataPreferred is not null)
                    {
                        MCvPoint3D64f? vecA = Convert2DTo3D(calibrationDataPreferred, (System.Windows.Point)LPointA, (System.Windows.Point)RPointA, true/*TrueUndistort*/);
                        MCvPoint3D64f? vecB = Convert2DTo3D(calibrationDataPreferred, (System.Windows.Point)LPointB, (System.Windows.Point)RPointB, true/*TrueUndistort*/);

                        if (vecA is not null && vecB is not null)
                        {
                            distance = DistanceBetween3DPoints((MCvPoint3D64f)vecA, (MCvPoint3D64f)vecB);

                            // Now calculate the using the undistorted points (//??? Will probably delete this later)
                            MCvPoint3D64f? vecADirtorted = Convert2DTo3D(calibrationDataPreferred, (System.Windows.Point)LPointA, (System.Windows.Point)RPointA, false/*TrueUndistort*/);
                            MCvPoint3D64f? vecBDirtorted = Convert2DTo3D(calibrationDataPreferred, (System.Windows.Point)LPointB, (System.Windows.Point)RPointB, false/*TrueUndistort*/);

                            if (vecADirtorted is not null && vecBDirtorted is not null)
                            {
                                double distanceDirtorted = DistanceBetween3DPoints((MCvPoint3D64f)vecADirtorted, (MCvPoint3D64f)vecBDirtorted);


                                if (ReportDepth == true)
                                {
                                    double depthA = CalculateDistanceFromOrigin((MCvPoint3D64f)vecA);
                                    double depthB = CalculateDistanceFromOrigin((MCvPoint3D64f)vecB);

                                    report.Out(Reporter.WarningLevel.Info, "", $"Length between red and green points = {distance:F1}mm (distorted:{distanceDirtorted:F1}mm) using preferred Calibration Data[{calibrationDataPreferred.Description}].  Distance from camera red point:{depthA / 1000:F2}m, green point:{depthB / 1000:F2}m.  Undistort delta: Ref({(vecADirtorted.Value.X - vecA.Value.X):F2}, {(vecADirtorted.Value.Y - vecA.Value.Y):F2}) Green({(vecBDirtorted.Value.X - vecB.Value.X):F2}, {(vecBDirtorted.Value.Y - vecB.Value.Y):F2})");
                                }
                                else
                                    report.Out(Reporter.WarningLevel.Info, "", $"Length between red and green points = {distance:F1}mm (distorted:{distanceDirtorted:F1}mm) using preferred Calibration Data[{calibrationDataPreferred.Description}].");
                            }
                        }


                        // Calculate the distance and report only using the non-preferred calibration data instance (if any)
                        if (calibrationClass.CalibrationDataList.Count > 1)
                        {
                            foreach (CalibrationData calibrationData in calibrationClass.CalibrationDataList)
                            {
                                // Ignore the preferred calibration data instance
                                if (calibrationData != calibrationDataPreferred)
                                {
                                    MCvPoint3D64f? vecA2 = Convert2DTo3D(calibrationData, (System.Windows.Point)LPointA, (System.Windows.Point)RPointA, true/*TrueUndistort*/);
                                    MCvPoint3D64f? vecB2 = Convert2DTo3D(calibrationData, (System.Windows.Point)LPointB, (System.Windows.Point)RPointB, true/*TrueUndistort*/);

                                    if (vecA2 is not null && vecB2 is not null)
                                    {
                                        double distance2 = DistanceBetween3DPoints((MCvPoint3D64f)vecA2, (MCvPoint3D64f)vecB2);
                                        double depthA = CalculateDistanceFromOrigin((MCvPoint3D64f)vecA2);
                                        double depthB = CalculateDistanceFromOrigin((MCvPoint3D64f)vecB2);

                                        report.Out(Reporter.WarningLevel.Info, "", $"---Length using non-preferred Calibration Data[{calibrationData.Description}] Length = {distance2:F1}mm, Distance from camera red point:{depthA / 1000:F2}m, green point:{depthB / 1000:F2}m");

                                    }
                                }
                            }
                        }
                    }
                }
            }

            return distance;
        }


        /// <summary>
        /// Calculate the distance from the cameras to corresponding 2D points in the left and right images
        /// </summary>
        /// <param name="LPoint"></param>
        /// <param name="LPoint"></param>
        /// <returns></returns>
        public double Depth(System.Windows.Point? LPoint, System.Windows.Point? RPoint)
        {
            double depth = 0;
            if (calibrationClass is not null && calibrationClass.CalibrationDataList is not null)
            {
                if (LPoint is not null && RPoint is not null)
                {
                    // Calculate the distance between the two 3D points using the preferred calibration data instance
                    CalibrationData? calibrationDataPreferred = calibrationClass.CalibrationDataList[calibrationClass.PreferredCalibrationDataIndex];
                    if (calibrationDataPreferred is not null)
                    {
                        MCvPoint3D64f? vec = Convert2DTo3D(calibrationDataPreferred, (System.Windows.Point)LPoint, (System.Windows.Point)RPoint, true/*TrueUndistort*/);                       

                        if (vec is not null)
                        {
                            depth = CalculateDistanceFromOrigin((MCvPoint3D64f)vec);
                            report.Out(Reporter.WarningLevel.Info, "", $"*Depth calcs using preferred Calibration Data[{calibrationDataPreferred.Description}] Depth = {depth/1000:F2}m");
                        }


                        // Calculate the distance and report only using the non-preferred calibration data instance (if any)
                        if (calibrationClass.CalibrationDataList.Count > 1)
                        {
                            foreach (CalibrationData calibrationData in calibrationClass.CalibrationDataList)
                            {
                                // Ignore the preferred calibration data instance
                                if (calibrationData != calibrationDataPreferred)
                                {
                                    MCvPoint3D64f? vec2 = Convert2DTo3D(calibrationData, (System.Windows.Point)LPoint, (System.Windows.Point)RPoint, true/*TrueUndistort*/);                                   

                                    if (vec2 is not null)
                                    {
                                        double depth2 = CalculateDistanceFromOrigin((MCvPoint3D64f)vec2);

                                        report.Out(Reporter.WarningLevel.Info, "", $"Depth calcs using non-preferred Calibration Data[{calibrationData.Description}] Depth = {depth2/1000:F2}m");
                                    }
                                }
                            }
                        }

                    }
                }
            }

            return depth;
        }


        /// <summary>
        /// Convert a matched left and right 2D points to a real world 3D point
        /// </summary>
        /// <param name="cd"></param>
        /// <param name="pL2D"></param>
        /// <param name="pR2D"></param>
        /// <returns></returns>
        public static MCvPoint3D64f? Convert2DTo3D(CalibrationData cd, System.Windows.Point PointL2D, System.Windows.Point PointR2D, bool TrueUndistortedFalseDistorted)
        {
            //MCvPoint2D64f L2D = new MCvPoint2D64f((double)pL2D.X, (double)pL2D.Y);
            //MCvPoint2D64f R2D = new MCvPoint2D64f((double)pR2D.X, (double)pR2D.Y);
            MathNet.Numerics.LinearAlgebra.Vector<double> L2D;
            MathNet.Numerics.LinearAlgebra.Vector<double> R2D;

            // Undort the points if necessary
            if (TrueUndistortedFalseDistorted == true)
            {
                System.Windows.Point _pointL2D = UndistortPoint(cd.LeftCalibrationCameraData, PointL2D);
                System.Windows.Point _pointR2D = UndistortPoint(cd.RightCalibrationCameraData, PointR2D);

                L2D = new MathNet.Numerics.LinearAlgebra.Double.DenseVector(new double[] { _pointL2D.X, _pointL2D.Y });
                R2D = new MathNet.Numerics.LinearAlgebra.Double.DenseVector(new double[] { _pointR2D.X, _pointR2D.Y });
            }
            else 
            {
                L2D = new MathNet.Numerics.LinearAlgebra.Double.DenseVector(new double[] { PointL2D.X, PointL2D.Y });
                R2D = new MathNet.Numerics.LinearAlgebra.Double.DenseVector(new double[] { PointR2D.X, PointR2D.Y });
            }


            MathNet.Numerics.LinearAlgebra.Vector<double> ?vector = Convert2DTo3D(cd, L2D, R2D);

            if (vector is not null)
                return new MCvPoint3D64f(vector[0], vector[1], vector[2]);
            else
                return null;
        }


        //public static MCvPoint3D64f? Convert2DTo3D(CalibrationData cd, MCvPoint2D64f LPoint, MCvPoint2D64f RPoint, bool TrueUndistortedFalseDistorted)
        //{
        //    if (cd.LeftCalibrationCameraData.Mtx != null &&
        //        cd.RightCalibrationCameraData.Mtx != null &&
        //        cd.CalibrationStereoCameraData.Rotation != null &&
        //        cd.CalibrationStereoCameraData.Translation != null)
        //    {
        //        // Prepare projection matrices
        //        Matrix<double> leftProj = new Matrix<double>(3, 4);
        //        cd.LeftCalibrationCameraData.Mtx.CopyTo(leftProj.GetSubRect(new Rectangle(0, 0, 3, 3))); // Copy the 3x3 part
        //        for (int i = 0; i < 3; i++)
        //        {
        //            leftProj[i, 3] = 0; // No translation for the left camera
        //        }
        //        Matrix<double> rightProj = new Matrix<double>(3, 4);
        //        cd.RightCalibrationCameraData.Mtx.CopyTo(rightProj.GetSubRect(new Rectangle(0, 0, 3, 3))); // Copy the 3x3 part
        //        for (int i = 0; i < 3; i++)
        //        {
        //            rightProj[i, 3] = cd.CalibrationStereoCameraData.Translation[i, 0]; // Set translation for the right camera
        //        }

        //        MCvPoint2D64f _lPoint;
        //        MCvPoint2D64f _rPoint;

        //        // Undistort points if necessary
        //        if (TrueUndistortedFalseDistorted == true)
        //        {
        //            _lPoint = UndistortPoint(cd.LeftCalibrationCameraData, LPoint);
        //            _rPoint = UndistortPoint(cd.RightCalibrationCameraData, RPoint);
        //        }
        //        else
        //        {
        //            _lPoint = LPoint;
        //            _rPoint = RPoint;
        //        }

        //        // Prepare points for triangulation
        //        Emgu.CV.Matrix<double> leftPoints = new Emgu.CV.Matrix<double>(2, 1);
        //        Emgu.CV.Matrix<double> rightPoints = new Emgu.CV.Matrix<double>(2, 1);
        //        leftPoints[0, 0] = _lPoint.X;
        //        leftPoints[1, 0] = _lPoint.Y;
        //        rightPoints[0, 0] = _rPoint.X;
        //        rightPoints[1, 0] = _rPoint.Y;

        //        // Triangulate
        //        Emgu.CV.Matrix<double> points4D = new Emgu.CV.Matrix<double>(4, 1);
        //        CvInvoke.TriangulatePoints(leftProj, rightProj, leftPoints, rightPoints, points4D);

        //        // Convert to 3D
        //        double w = points4D[3, 0];
        //        return new MCvPoint3D64f(points4D[0, 0] / w, points4D[1, 0] / w, points4D[2, 0] / w);
        //    }

        //    return null;
        //}

        /// <summary>
        /// Convert a matched left and right 2D points to a real world 3D point
        /// Uses MathNET matrix and vector types but has Calibration data that uses EmguCV matrix types
        /// </summary>
        /// <param name="cd"></param>
        /// <param name="L2D"></param>
        /// <param name="R2D"></param>
        /// <returns></returns>
        public static MathNet.Numerics.LinearAlgebra.Vector<double>? Convert2DTo3D(CalibrationData cd, MathNet.Numerics.LinearAlgebra.Vector<double> L2D, MathNet.Numerics.LinearAlgebra.Vector<double> R2D)
        {
            if (cd.LeftCalibrationCameraData.Mtx is not null &&
                cd.RightCalibrationCameraData.Mtx is not null &&
                cd.CalibrationStereoCameraData.Rotation is not null &&
                cd.CalibrationStereoCameraData.Translation is not null)
            {
                var RT_L = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix.CreateIdentity(3).Append(MathNet.Numerics.LinearAlgebra.Double.DenseMatrix.Create(3, 1, 0));
                var P_L = ConvertEmguMatrixToMathNetMatrix(cd.LeftCalibrationCameraData.Mtx).Multiply(RT_L);

                var RT_R = ConvertEmguMatrixToMathNetMatrix(cd.CalibrationStereoCameraData.Rotation).Append(ConvertEmguMatrixToMathNetVector(cd.CalibrationStereoCameraData.Translation).ToColumnMatrix());
                var P_R = ConvertEmguMatrixToMathNetMatrix(cd.RightCalibrationCameraData.Mtx).Multiply(RT_R);

                return DirectLinearTransformation(P_L, P_R, L2D, R2D);
            }

            return null;
        }


        public static MathNet.Numerics.LinearAlgebra.Vector<double> DirectLinearTransformation(MathNet.Numerics.LinearAlgebra.Matrix<double> P1, MathNet.Numerics.LinearAlgebra.Matrix<double> P2, MathNet.Numerics.LinearAlgebra.Vector<double> point1, MathNet.Numerics.LinearAlgebra.Vector<double> point2)
        {
            var A = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix.OfRowArrays(
                (point1[1] * P1.Row(2) - P1.Row(1)).ToArray(),
                (P1.Row(0) - point1[0] * P1.Row(2)).ToArray(),
                (point2[1] * P2.Row(2) - P2.Row(1)).ToArray(),
                (P2.Row(0) - point2[0] * P2.Row(2)).ToArray()
            );

            var B = A.TransposeThisAndMultiply(A);
            var svd = B.Svd(true);
            var Vh = svd.VT;
            var triangulatedPoint = Vh.Row(3).SubVector(0, 3) / Vh[3, 3];

            Console.WriteLine("Triangulated point: ");
            Console.WriteLine(triangulatedPoint);
            return triangulatedPoint;
        }


        public static MathNet.Numerics.LinearAlgebra.Matrix<double> ConvertEmguMatrixToMathNetMatrix(Emgu.CV.Matrix<double> emguMatrix)
        {
            int rows = emguMatrix.Rows;
            int cols = emguMatrix.Cols;
            MathNet.Numerics.LinearAlgebra.Matrix<double> mathNetMatrix = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix.Create(rows, cols, 0);

            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    mathNetMatrix[i, j] = emguMatrix[i, j];
                }
            }

            return mathNetMatrix;
        }


        public static MathNet.Numerics.LinearAlgebra.Vector<double> ConvertEmguMatrixToMathNetVector(Emgu.CV.Matrix<double> emguMatrix)
        {
            // Check if the matrix is one-dimensional
            if (emguMatrix.Rows != 1 && emguMatrix.Cols != 1)
            {
                throw new ArgumentException("The matrix is not one-dimensional and cannot be converted to a vector.");
            }

            // Determine the length of the vector
            int length = Math.Max(emguMatrix.Rows, emguMatrix.Cols);
            MathNet.Numerics.LinearAlgebra.Vector<double> mathNetVector = MathNet.Numerics.LinearAlgebra.Double.DenseVector.Create(length, 0);

            for (int i = 0; i < length; i++)
            {
                mathNetVector[i] = (emguMatrix.Rows == 1) ? emguMatrix[0, i] : emguMatrix[i, 0];
            }

            return mathNetVector;
        }


        /// <summary>
        /// Used to undistort 2D MCvPoint2D64f point using calibration data
        /// </summary>
        /// <param name="point"></param>
        /// <param name="cameraMatrix"></param>
        /// <param name="distCoeffs"></param>
        /// <returns></returns>
        public static MCvPoint2D64f UndistortPoint(CalibrationCameraData ccd, MCvPoint2D64f point)
        {
            // Convert the input point to a VectorOfPoint2D32F
            VectorOfPointF distortedPoints = new VectorOfPointF(new PointF[] { new PointF((float)point.X, (float)point.Y) });

            // Create a VectorOfPoint2D32F to hold the undistorted point
            VectorOfPointF undistortedPoints = new VectorOfPointF(1);

            // Perform undistortion
            CvInvoke.UndistortPoints(distortedPoints, undistortedPoints, ccd.Mtx, ccd.Dist, null, ccd.Mtx);

            // Retrieve the undistorted point
            PointF undistortedPointF = undistortedPoints[0];

            // Convert the undistorted point back to MCvPoint2D64f
            return new MCvPoint2D64f(undistortedPointF.X, undistortedPointF.Y);
        }


        /// <summary>
        /// Used to undistort 2D System.Windows.Point point using calibration data
        /// </summary>
        /// <param name="point"></param>
        /// <param name="cameraMatrix"></param>
        /// <param name="distCoeffs"></param>
        /// <returns></returns>
        public static System.Windows.Point UndistortPoint(CalibrationCameraData ccd, System.Windows.Point point)
        {
            // Convert the input point to a VectorOfPoint2D32F
            VectorOfPointF distortedPoints = new VectorOfPointF(new PointF[] { new PointF((float)point.X, (float)point.Y) });

            // Create a VectorOfPoint2D32F to hold the undistorted point
            VectorOfPointF undistortedPoints = new VectorOfPointF(1);

            // Perform undistortion
            CvInvoke.UndistortPoints(distortedPoints, undistortedPoints, ccd.Mtx, ccd.Dist, null, ccd.Mtx);

            // Retrieve the undistorted point
            PointF undistortedPointF = undistortedPoints[0];

            // Convert the undistorted point back to System.Windows.Point
            return new System.Windows.Point(undistortedPointF.X, undistortedPointF.Y);
        }


        /// <summary>
        /// Returns the distance between the two 3D points
        /// </summary>
        /// <param name="point1"></param>
        /// <param name="point2"></param>
        /// <returns></returns>
        public static double DistanceBetween3DPoints(MCvPoint3D64f point1, MCvPoint3D64f point2)
        {
            return Math.Sqrt(Math.Pow(point2.X - point1.X, 2) + Math.Pow(point2.Y - point1.Y, 2) + Math.Pow(point2.Z - point1.Z, 2));
        }




        



        //public static double DistanceBetween3DPoints_Legacy(Vector<double> point1, Vector<double> point2)
        //{
        //    return Math.Sqrt(Math.Pow(point2[0] - point1[0], 2) + Math.Pow(point2[1] - point1[1], 2) + Math.Pow(point2[2] - point1[2], 2));
        //}


        /// <summary>
        /// Calculates the miniumum distance there ever is between the two lines. 
        /// The first line is defined by the two points p1a and p1b. 
        /// The second line is defined by the two points p2a and p2b.
        /// </summary>
        /// <param name="p1a"></param>
        /// <param name="p1b"></param>
        /// <param name="p2a"></param>
        /// <param name="p2b"></param>
        /// <returns></returns>
        //public static double CalculateMinimumDistance(Vector<double> p1a, Vector<double> p1b, Vector<double> p2a, Vector<double> p2b)
        //{
        //    var u = p1b - p1a;
        //    var v = p2b - p2a;
        //    var w = p1a - p2a;

        //    double a = u.DotProduct(u);
        //    double b = u.DotProduct(v);
        //    double c = v.DotProduct(v);
        //    double d = u.DotProduct(w);
        //    double e = v.DotProduct(w);
        //    double D = a * c - b * b;

        //    double sc, tc;

        //    if (D < 1e-10)
        //    {
        //        sc = 0.0;
        //        tc = (b > c ? d / b : e / c);
        //    }
        //    else
        //    {
        //        sc = (b * e - c * d) / D;
        //        tc = (a * e - b * d) / D;
        //    }

        //    var dP = w + sc * u - tc * v;
        //    return dP.L2Norm();
        //}


        /// <summary>
        /// Calculate the distance this 3D point is from the origin
        /// </summary>
        /// <param name="vector"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentException"></exception>
        public static double CalculateDistanceFromOrigin(MCvPoint3D64f vector)
        {
            double x = vector.X;
            double y = vector.Y;
            double z = vector.Z;

            return Math.Sqrt(x * x + y * y + z * z);
        }
    }
}
