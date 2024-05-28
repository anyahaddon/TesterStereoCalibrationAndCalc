import cv2 as cv
import glob
import numpy as np
import matplotlib.pyplot as plt
import math
import os.path
import re
from datetime import timedelta

from cameraCalibration import CalibrationData, CalibrationCameraData, CalibrationStereoCameraData

global_desired_width = 3840 / 4
global_desired_height = 2160 / 4
 
def display_resized_image(image, desired_width, desired_height, title):
    """
    Display a resized image in a window while preserving its aspect ratio.

    Args:
        image (numpy.ndarray): The input image.
        desired_width (int): The desired width for the window.
        desired_height (int): The desired height for the window.

    Returns:
        None
    """
    # Calculate the new dimensions while preserving the aspect ratio
    if len(image.shape) == 3:
        height, width, _ = image.shape
    else:
        height, width = image.shape
        
    aspect_ratio = width / height

    if aspect_ratio > 1:
        new_width = int(desired_width)
        new_height = int(desired_width / aspect_ratio)
    else:
        new_height = int(desired_height)
        new_width = int(desired_height * aspect_ratio)

    # Resize the image to the calculated dimensions
    image_resized = cv.resize(image, (new_width, new_height))

    # Display the resized image in a window
    cv.imshow(title, image_resized)
    
     
  
############################################
# Find Calibration Object Points
# Returns: 0 if successful
#          -1 if no images found
#          -2 if images are not the same size
#          -3 if no calibration board found
def findCalibrationObjectPoints(images_folder, 
                                ChessPatternSize, ChessPatternRows, ChessPatternColumns, 
                                CirclesPatternSize, CirclesPatternRows, CirclesPatternColumns):
    
    # Read the images from the folder
    images_names = glob.glob(images_folder)
    images_names.sort()
    
    images = []
    imageIndex = 0
    for imname in images_names:
        print(F"Reading image {imageIndex}: {os.path.basename(imname)}")
        im = cv.imread(imname, 1)
        images.append(im)
        imageIndex += 1
        
    if len(images) == 0:
        print(F"Error: No images found in [{images_folder}]")
        return -1

    # Check all the images are the same size
    imageWidth = images[0].shape[1]
    imageHeight = images[0].shape[0]
    for frame in images:
        if (imageWidth != frame.shape[1] or imageHeight != frame.shape[0]):
            print(F"Error: All images must be the same size. Seen at least ({imageWidth}x{imageHeight}) and ({frame.shape[1]}x{frame.shape[0]})")
            return -2

    # Coordinates of squares in the checkerboard world space
    objpChess = np.zeros((ChessPatternRows * ChessPatternColumns, 3), np.float32)
    objpChess[:,:2] = np.mgrid[0:ChessPatternRows,0:ChessPatternColumns].T.reshape(-1,2)
    objpChess *= ChessPatternSize
    criteriaChess = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)  #criteria used by checkerboard pattern detector.            
    convolutionSizeChess = (11, 11)  #Convolution size used to improve corner detection. Don't make this too large.
    # Prepare object points based on the circle size
    objpCircles = np.zeros((CirclesPatternRows * CirclesPatternColumns, 3), np.float32)
    objpCircles[:, :2] = np.mgrid[0:CirclesPatternRows,0:CirclesPatternColumns].T.reshape(-1,2)      #objpCircles[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    #print(F"objpCircles: {objpCircles}")
    
    objpCircles[0]  = (0  , 0  , 0)
    objpCircles[1]  = (0  , 1.0 , 0)
    objpCircles[2]  = (0  , 2.0, 0)
    objpCircles[3]  = (0  , 3.0, 0)
    objpCircles[4]  = (0.5 , 0.5 , 0)
    objpCircles[5]  = (0.5 , 1.5, 0)
    objpCircles[6]  = (0.5 , 2.5, 0)
    objpCircles[7]  = (0.5 , 3.5, 0)
    objpCircles[8]  = (1.0 , 0  , 0)
    objpCircles[9]  = (1.0 , 1.0 , 0)
    objpCircles[10] = (1.0 , 2.0, 0)
    objpCircles[11] = (1.0 , 3.0, 0)
    objpCircles[12] = (1.5, 0.5,  0)
    objpCircles[13] = (1.5, 1.5, 0)
    objpCircles[14] = (1.5, 2.5, 0)
    objpCircles[15] = (1.5, 3.5, 0)
    objpCircles[16] = (2.0, 0  , 0)
    objpCircles[17] = (2.0, 1.0 , 0)
    objpCircles[18] = (2.0, 2.0, 0)
    objpCircles[19] = (2.0, 3.0, 0)
    objpCircles[20] = (2.5, 0.5 , 0)
    objpCircles[21] = (2.5, 1.5, 0)
    objpCircles[22] = (2.5, 2.5, 0)
    objpCircles[23] = (2.5, 3.5, 0)
    objpCircles[24] = (3.0, 0  , 0)
    objpCircles[25] = (3.0, 1.0 , 0)
    objpCircles[26] = (3.0, 2.0, 0)
    objpCircles[27] = (3.0, 3.0, 0)
    objpCircles[28] = (3.5, 0.5 , 0)
    objpCircles[29] = (3.5, 1.5, 0)
    objpCircles[30] = (3.5, 2.5, 0)
    objpCircles[31] = (3.5, 3.5, 0)
    objpCircles[32] = (4.0, 0  , 0)
    objpCircles[33] = (4.0, 1.0 , 0)
    objpCircles[34] = (4.0, 2.0, 0)
    objpCircles[35] = (4.0, 3.0, 0)
    objpCircles[36] = (4.5, 0.5 , 0)
    objpCircles[37] = (4.5, 1.5, 0)
    objpCircles[38] = (4.5, 2.5, 0)
    objpCircles[39] = (4.5, 3.5, 0)
    objpCircles[40] = (5.0, 0  , 0)
    objpCircles[41] = (5.0, 1.0 , 0)
    objpCircles[42] = (5.0, 2.0, 0)
    objpCircles[43] = (5.0, 3.0, 0)
    objpCircles *= CirclesPatternSize
    #print(F"objpCircles*size: {objpCircles}")
    criteriaCircles = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    
    # Count if number of each type of calibration board we see in the
    # different images. Once we see three of any type of board we can
    # assume that is calibration board we are using.
    ChessboardCount = 0
    ArucoCount = 0
    CirclesCount = 0
    
    CheckForChessboard = False
    CheckForArucoBoard = False
    CheckForCirclesBoard = False
        
    # Check if we are looking for a chessboard
    if (ChessPatternSize != 0 and ChessPatternRows > 0 and ChessPatternColumns > 0):
        CheckForChessboard = True
    
    # Check if we are looking for a circle board
    if (CirclesPatternSize != 0 and CirclesPatternRows > 0 and CirclesPatternColumns > 0):
        CheckForCirclesBoard = True
    
         
    # Chess Image and Object points
    imgpointsChess = [] # 2d points in image plane.
    objpointsChess = [] # 3d point in real world space
    # Aruco Image and Object points
    imgpointsAruco = [] # 2d points in image plane.
    objpointsAruco = [] # 3d point in real world space
    # Circles Image and Object points
    imgpointsCircles = [] # 2d points in image plane.
    objpointsCircles = [] # 3d point in real world space
    # Setup SimpleBlobDetector parameters.
    blobParams = cv.SimpleBlobDetector_Params()
    # Change thresholds
    blobParams.minThreshold = 8
    blobParams.maxThreshold = 255
    # Filter by Area.
    blobParams.filterByArea = True
    blobParams.minArea = 64     # minArea may be adjusted to suit for your experiment
    blobParams.maxArea = 2500   # maxArea may be adjusted to suit for your experiment
    # Filter by Circularity
    blobParams.filterByCircularity = True
    blobParams.minCircularity = 0.1
    # Filter by Convexity
    blobParams.filterByConvexity = True
    blobParams.minConvexity = 0.87
    # Filter by Inertia
    blobParams.filterByInertia = True
    blobParams.minInertiaRatio = 0.01
    # Create a detector with the parameters
    blobDetector = cv.SimpleBlobDetector_create(blobParams)
    

    # Loop through all the images
    index = 0
    for frame in images:
        # Convert to grayscale
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            
        imgpointsChessItem = None
        objpointsChessItem = None
        imgpointsArucoItem = None 
        objpointsArucoItem = None
        imgpointsCirclesItem = None
        objpointsCirclesItem = None
             
        # Check for a chessboard
        ret, corners = cv.findChessboardCorners(gray, (ChessPatternRows, ChessPatternColumns), None)
 
        if ret == True:
            ChessboardCount += 1
            
            # Opencv can attempt to improve the checkerboard coordinates
            corners = cv.cornerSubPix(gray, corners, convolutionSizeChess, (-1, -1), criteriaChess)

            # Accumulate the object points and image points
            imgpointsChessItem = corners
            objpointsChessItem = objpChess

        else:
            # Detect blobs.
            keypoints = blobDetector.detect(gray)
            imgage_with_keypoints = cv.drawKeypoints(frame, keypoints, np.array([]), (0,255,0), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            image_with_keypoints_gray = cv.cvtColor(imgage_with_keypoints, cv.COLOR_BGR2GRAY)

            # check for the circle grid
            ret, centers = cv.findCirclesGrid(image_with_keypoints_gray, (CirclesPatternRows, CirclesPatternColumns), None, flags = cv.CALIB_CB_ASYMMETRIC_GRID)
            
            if ret == True:                                
                CirclesCount += 1

                 # Refine the detected centers to subpixel accuracy
                #centers = cv.cornerSubPix(gray, centers, (11, 11), (-1, -1), criteriaCircles)

                # Accumulate the object points and image points
                imgpointsCirclesItem = centers
                objpointsCirclesItem = objpCircles
         
                         
                image_with_keypoints_gray_resize = cv.resize(imgage_with_keypoints, (0, 0), fx=0.3, fy=0.3)
                cv.imshow("Image Blobs", image_with_keypoints_gray_resize) # display
                #cv.waitKey(2)     
                image_with_centers = cv.drawChessboardCorners(frame, (4,11), corners, ret)           
                image_with_centers_resize = cv.resize(image_with_centers, (0, 0), fx=0.3, fy=0.3)
                cv.imshow("Image Marked Up", image_with_centers_resize) # display
                cv.waitKey(2)
            else:
                print(F"Error: No calibration board found in image: {os.path.basename(images_names[index])}")

        
        # List to lists
        imgpointsChess.append(imgpointsChessItem)
        objpointsChess.append(objpointsChessItem)
        imgpointsAruco.append(imgpointsArucoItem)
        objpointsAruco.append(objpointsArucoItem)
        imgpointsCircles.append(imgpointsCirclesItem)
        objpointsCircles.append(objpointsCirclesItem)


        # If we seen three of any type of board then we can assume that is the 
        # board we are using.  The checks are slow so this is to speed things up
        if ChessboardCount >= 3:
            CheckForArucoBoard = False
            CheckForCirclesBoard = False            
        elif ArucoCount >= 3:
            CheckForChessboard = False
            CheckForCirclesBoard = False
        elif CirclesCount >= 3:
            CheckForChessboard = False
            CheckForArucoBoard = False
    
        index += 1
        
    # Return the name of the type of calibration board found
    boardType = None
    objpoints = None
    imgpoints = None
    patterRows = 0
    patterColumns = 0
    if CheckForChessboard == True and ChessboardCount > 0:
        boardType = "Chess"
        objpoints = objpointsChess
        imgpoints = imgpointsChess
        patterRows = ChessPatternRows
        patterColumns = ChessPatternColumns
        ret = 0
    elif CheckForArucoBoard == True and ArucoCount > 0:
        boardType = "Aruco"
    elif CheckForCirclesBoard == True and CirclesCount > 0:
        boardType = "Circles"
        objpoints = objpointsCircles
        imgpoints = imgpointsCircles
        patterRows = CirclesPatternRows
        patterColumns = CirclesPatternColumns
        ret = 0
    else:
        print("Error: No calibration board found")
        ret = -3
    
    return ret, boardType, objpoints, imgpoints, imageWidth, imageHeight, patterRows, patterColumns

   
############################################
# Calibrate the camera
# Returns: 0 if successful
#          
#          
def calibrate_camera(channel, objpoints, imgpoints, imageWidth, imageHeight):
                              
    retRMS, mtx, dist, _, _ = cv.calibrateCamera(objpoints, imgpoints, (imageWidth, imageHeight), None, None)
    #print(F"{channel} RSM:", retRMS)
    #print('camera matrix:\n', mtx)
    #print('distortion coeffs:', dist)
    #print('Rs:\n', rvecs)
    #print('Ts:\n', tvecs)
 
    return retRMS, mtx, dist


############################################
# Calibrate stereo cameras
# Returns: 0 if successful
#          
#  
def stereo_calibrate(objpoints, imgpoints_left, imgpoints_right, mtxL, distL, mtxR, distR, imageWidth, imageHeight):

 
    #change this if stereo calibration not good.
    #criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
    stereocalibration_flags = 0
    #stereocalibration_flags = cv.CALIB_FIX_INTRINSIC
    retRMS, CM1, dist1, CM2, dist2, R, T, E, F = cv.stereoCalibrate(objpoints, imgpoints_left, imgpoints_right, 
                                                                    mtxL, distL,
                                                                    mtxR, distR, 
                                                                    (imageWidth, imageHeight), 
                                                                    criteria = criteria, flags = stereocalibration_flags)
 
    #print('Stereo RMS', retRMS)
    return retRMS, R, T 


     
#####################
# Original function
def calibrate_camera_Legacy(images_folder):
    images_names = glob.glob(images_folder)
    images = []
    for imname in images_names:
        im = cv.imread(imname, 1)
        images.append(im)
 
 
    #criteria used by checkerboard pattern detector.
    #Change this if the code can't find the checkerboard
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
    rows = 6 #number of checkerboard rows.
    columns = 9 #number of checkerboard columns.
    #world_scaling = 29.7 #change this to the real world square size. Or not.
    world_scaling = 34.85 #change this to the real world square size. Or not.
 
    #coordinates of squares in the checkerboard world space
    objp = np.zeros((rows*columns,3), np.float32)
    objp[:,:2] = np.mgrid[0:rows,0:columns].T.reshape(-1,2)
    objp = world_scaling* objp
 
    #frame dimensions. Frames should be the same size.
    width = images[0].shape[1]
    height = images[0].shape[0]
 
    #Pixel coordinates of checkerboards
    imgpoints = [] # 2d points in image plane.
 
    #coordinates of the checkerboard in checkerboard world space.
    objpoints = [] # 3d point in real world space
 
 
    for frame in images:
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
             
        #find the checkerboard
        ret, corners = cv.findChessboardCorners(gray, (rows, columns), None)
 
        if ret == True:
 
            #Convolution size used to improve corner detection. Don't make this too large.
            conv_size = (11, 11)
 
            #opencv can attempt to improve the checkerboard coordinates
            corners = cv.cornerSubPix(gray, corners, conv_size, (-1, -1), criteria)
            
            cv.drawChessboardCorners(frame, (rows,columns), corners, ret)
            #cv.imshow('img', frame)
            display_resized_image(frame, global_desired_width, global_desired_height, 'img')
            cv.waitKey(500)
 
            objpoints.append(objp)
            imgpoints.append(corners)
 
 
 
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, (width, height), None, None)
    print('RSM:', ret)
    print('camera matrix:\n', mtx)
    print('distortion coeffs:', dist)
    print('Rs:\n', rvecs)
    print('Ts:\n', tvecs)
 
    return mtx, dist
 
 
def stereo_calibrate_Legacy(mtx1, dist1, mtx2, dist2, framesL_folder, framesR_folder):
    #read the synched frames
    c1_images_names = sorted(glob.glob(framesL_folder))
    c2_images_names = sorted(glob.glob(framesR_folder))
     
    c1_images = []
    c2_images = []
    for im1, im2 in zip(c1_images_names, c2_images_names):
        _im = cv.imread(im1, 1)
        c1_images.append(_im)
 
        _im = cv.imread(im2, 1)
        c2_images.append(_im)
 
    #change this if stereo calibration not good.
    #criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
    rows = 6 #number of checkerboard rows.
    columns = 9 #number of checkerboard columns.
    world_scaling = 34.85 #change this to the real world square size. Or not.
 
    #coordinates of squares in the checkerboard world space
    objp = np.zeros((rows*columns,3), np.float32)
    objp[:,:2] = np.mgrid[0:rows,0:columns].T.reshape(-1,2)
    objp = world_scaling* objp
 
    #frame dimensions. Frames should be the same size.
    width = c1_images[0].shape[1]
    height = c1_images[0].shape[0]
 
    #Pixel coordinates of checkerboards
    imgpoints_left = [] # 2d points in image plane.
    imgpoints_right = []
 
    #coordinates of the checkerboard in checkerboard world space.
    objpoints = [] # 3d point in real world space
 
    for frame1, frame2 in zip(c1_images, c2_images):
        gray1 = cv.cvtColor(frame1, cv.COLOR_BGR2GRAY)
        gray2 = cv.cvtColor(frame2, cv.COLOR_BGR2GRAY)
        c_ret1, corners1 = cv.findChessboardCorners(gray1, (rows, columns), None)
        c_ret2, corners2 = cv.findChessboardCorners(gray2, (rows, columns), None)
 
        if c_ret1 == True and c_ret2 == True:
            corners1 = cv.cornerSubPix(gray1, corners1, (11, 11), (-1, -1), criteria)
            corners2 = cv.cornerSubPix(gray2, corners2, (11, 11), (-1, -1), criteria)
 
            cv.drawChessboardCorners(frame1, (rows,columns), corners1, c_ret1)
            display_resized_image(frame1, global_desired_width, global_desired_height, 'Left')
 
            cv.drawChessboardCorners(frame2, (rows,columns), corners2, c_ret2)
            display_resized_image(frame2, global_desired_width, global_desired_height, 'Right')

            cv.waitKey(500)
 
            objpoints.append(objp)
            imgpoints_left.append(corners1)
            imgpoints_right.append(corners2)
 
    stereocalibration_flags = cv.CALIB_FIX_INTRINSIC
    retRMS, CM1, dist1, CM2, dist2, R, T, E, F = cv.stereoCalibrate(objpoints, imgpoints_left, imgpoints_right, mtx1, dist1,
                                                                 mtx2, dist2, (width, height), criteria = criteria, flags = stereocalibration_flags)
 
    print('rmse:', retRMS)
    print('R:', R)
    print('T:', T)
    return retRMS, R, T, 



def DLT(P1, P2, point1, point2):

    A = [point1[1]*P1[2,:] - P1[1,:],
            P1[0,:] - point1[0]*P1[2,:],
            point2[1]*P2[2,:] - P2[1,:],
            P2[0,:] - point2[0]*P2[2,:]
        ]
    A = np.array(A).reshape((4,4))
    #print('A: ')
    #print(A)

    B = A.transpose() @ A
    from scipy import linalg
    U, s, Vh = linalg.svd(B, full_matrices = False)

    print('Triangulated point: ')
    print(Vh[3,0:3]/Vh[3,3])
    return Vh[3,0:3]/Vh[3,3]

 
def distance_3d(point1, point2):
    x1, y1, z1 = point1
    x2, y2, z2 = point2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

 
def triangulateWithDisplay(mtx1, mtx2, R, T, fileFilterLeft, fileFilterRight):
 
#############################################
# 008 
# Left                          Right
# (1939,1383) - (2557,1360)     (1599,1419) - (2258,1426)                   
# (2260,1313) - (2264,1435)     (1922,1365) - (1927,1490)
# Left:  008L (Chessboard A4 Board on Land)-frame00001400.jpeg
# Right: 008R (Chessboard A4 Board on Land)-frame00001399.jpeg
#    24 Oct 2023 (Using MSPaint to get the pixel coordinates)
#    uvsL = [[1939,1383],[2557,1360],[2260,1313],[2264,1435]]  
#    uvsR = [[1599,1419],[2258,1426],[1922,1365],[1927,1490]]
    uvsL = [[1940.03,1379.31],[2559.00,1357.27],[2258.53,1313.20],[2260.53,1435.40]]    
    uvsR = [[1597.50,1421.37],[2260.53,1427.38],[1922.00,1361.28],[1922.00,1485.47]]
#############################################
# 009
# Left                          Right
# (2383,1305) - (3193,1278)     (2088,1367) - (2845,1351)
# (2756,1220) - (2767,1367)     (2421,1284) - (2435,1440)
# Left:  009L (Chessboard A4 Board in Water)-frame00003400.jpeg
# Right: 009R (Chessboard A4 Board in Water)-frame00003399.jpeg
#    24 Oct 2023 (Using MSPaint to get the pixel coordinates)
#    uvsL = [[2383,1305],[3193,1278],[2756,1220],[2767,1367]]
#    uvsL = [[2088,1367],[2845,1351],[2421,1284],[2435,1440]]
#    02 Nov 2023 (Using MeasurementPointControl class)    
    #uvsL = [[2382.72,1305.19],[3198.00,1277.15],[2753.30,1219.06],[2767.32,1367.29]]
    #uvsR = [[2086.26,1365.29],[2845.45,1353.27],[2420.78,1285.16],[2434.80,1437.40]]
    
#############################################
# Original code
#    uvsL = [[458, 86], [451, 164], [287, 181],
#            [196, 383], [297, 444], [564, 194],
#            [562, 375], [596, 520], [329, 620],
#            [488, 622], [432, 52], [489, 56]]
 
#    uvsR = [[540, 311], [603, 359], [542, 378],
#            [525, 507], [485, 542], [691, 352],
#            [752, 488], [711, 605], [549, 651],
#            [651, 663], [526, 293], [542, 290]]
 
    uvsL = np.array(uvsL)
    uvsR = np.array(uvsR)
 

    L_images_names = sorted(glob.glob(fileFilterLeft))
    R_images_names = sorted(glob.glob(fileFilterRight))
    L_images_name = L_images_names[0]
    R_images_name = R_images_names[0]
    frameL = cv.imread(L_images_name)
    frameR = cv.imread(R_images_name)
    print(F"Do triangulation using left:{L_images_name}, right:{R_images_name}")
 
    plt.imshow(frameL[:,:,[2,1,0]])
    #plt.scatter(uvsL[:,0], uvsL[:,1])
    plt.show() #this call will cause a crash if you use cv.imshow() above. Comment out cv.imshow() to see this.
 
    plt.imshow(frameR[:,:,[2,1,0]])
    #plt.scatter(uvsR[:,0], uvsR[:,1])
    plt.show()#this call will cause a crash if you use cv.imshow() above. Comment out cv.imshow() to see this
 
    #RT matrix for Left is identity.
    RT_L = np.concatenate([np.eye(3), [[0],[0],[0]]], axis = -1)
    P_L = mtx1 @ RT_L #projection matrix for Left
 
    #RT matrix for Right is the R and T obtained from stereo calibration.
    RT_R = np.concatenate([R, T], axis = -1)
    P_R = mtx2 @ RT_R #projection matrix for Right
 
 
    lowestX = 99999
    hightestX = -99999
    lowestY = 99999
    hightestY = -99999
    lowestZ = 99999
    hightestZ = -99999
    
    p3ds = []
    for uv1, uv2 in zip(uvsL, uvsR):
        _p3d = DLT(P_L, P_R, uv1, uv2)
        p3ds.append(_p3d)
        if (hightestX < _p3d[0]):
            hightestX = _p3d[0]
        if (lowestX > _p3d[0]):
            lowestX = _p3d[0]
        if (hightestY < _p3d[1]):
            hightestY = _p3d[1]
        if (lowestY > _p3d[1]):
            lowestY = _p3d[1]
        if (hightestZ < _p3d[2]):
            hightestZ = _p3d[2]
        if (lowestZ > _p3d[2]):
            lowestZ = _p3d[2]
        
    p3ds = np.array(p3ds)
 
    from mpl_toolkits.mplot3d import Axes3D
 
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim3d(lowestX-10, hightestX+10)
    ax.set_ylim3d(lowestY-10, hightestY+10)
    ax.set_zlim3d(lowestZ-10, hightestZ+10)
 
    #connections = [[0,1], [1,2], [2,3], [3,4]] #, [1,5], [5,6], [6,7], [1,8], [1,9], [2,8], [5,9], [8,9], [0, 10], [0, 11]]
    #for _c in connections:
    #    print(p3ds[_c[0]])
    #    print(p3ds[_c[1]])
    #    ax.plot(xs = [p3ds[_c[0],0], p3ds[_c[1],0]], ys = [p3ds[_c[0],1], p3ds[_c[1],1]], zs = [p3ds[_c[0],2], p3ds[_c[1],2]], c = 'red')
     
    lineLenth1 = distance_3d(p3ds[0], p3ds[1]) 
    lineLenth2 = distance_3d(p3ds[2], p3ds[3]) 
     
        
    ax.plot(xs = [p3ds[0,0], p3ds[1,0]], ys = [p3ds[0,1], p3ds[1,1]], zs = [p3ds[0,2], p3ds[1,2]], c = 'red')
    ax.plot(xs = [p3ds[2,0], p3ds[3,0]], ys = [p3ds[2,1], p3ds[3,1]], zs = [p3ds[2,2], p3ds[3,2]], c = 'red')
    #ax.plot(xs = [p3ds[_c[0],0], p3ds[_c[1],0]], ys = [p3ds[_c[0],1], p3ds[_c[1],1]], zs = [p3ds[_c[0],2], p3ds[_c[1],2]], c = 'red')
    ax.set_title(F"This figure can be rotated, line1={lineLenth1:.2f}, line2={lineLenth2:.2f}.")
    #uncomment to see the triangulated pose. This may cause a crash if youre also using cv.imshow() above.
    plt.show()


def triangulateNoDisplay(mtx1, mtx2, R, T, LCoordinateDump, RLCoordinateDump):
 

    uvsL = np.array(LCoordinateDump)  
    uvsR = np.array(RCoordinateDump)
 
    #RT matrix for Left is identity.
    RT_L = np.concatenate([np.eye(3), [[0],[0],[0]]], axis = -1)
    P_L = mtx1 @ RT_L #projection matrix for Left
 
    #RT matrix for Right is the R and T obtained from stereo calibration.
    RT_R = np.concatenate([R, T], axis = -1)
    P_R = mtx2 @ RT_R #projection matrix for Right
  
    p3ds = []
    for uv1, uv2 in zip(uvsL, uvsR):
        _p3d = DLT(P_L, P_R, uv1, uv2)
        p3ds.append(_p3d)
            
    p3ds = np.array(p3ds)
      
    lineLenth1 = distance_3d(p3ds[0], p3ds[1]) 
    lineLenth2 = distance_3d(p3ds[2], p3ds[3]) 
     
    print( F"line1={lineLenth1}, line2={lineLenth2}.")   # :.2f
    
    distance = calculate_minimum_distance(p3ds[0], p3ds[1], p3ds[2], p3ds[3])
    print(F"Minimum distance between these two lines is:{distance}")


def calculate_minimum_distance(p1a, p1b, p2a, p2b):
    u = np.array(p1b) - np.array(p1a)
    v = np.array(p2b) - np.array(p2a)
    w = np.array(p1a) - np.array(p2a)

    a = np.dot(u, u)
    b = np.dot(u, v)
    c = np.dot(v, v)
    d = np.dot(u, w)
    e = np.dot(v, w)
    D = a * c - b * b

    if D < np.finfo(float).eps:
        sc = 0.0
        tc = d / b if b > c else e / c
    else:
        sc = (b * e - c * d) / D
        tc = (a * e - b * d) / D

    dP = w + sc * u - tc * v
    return np.linalg.norm(dP)


# These three lists are aligned.  If a point is removed from one list then the same point must be removed from the other lists
import numpy as np

def DiscardBlankPointAndMaintainAlignedLists(Lobjpoints, Limgpoints, Rimgpoints):
    LobjpointsTemp = []
    LimgpointsTemp = []
    RimgpointsTemp = []

    if len(Lobjpoints) == len(Limgpoints) and len(Lobjpoints) == len(Rimgpoints):
        for obj, imgL, imgR in zip(Lobjpoints, Limgpoints, Rimgpoints):
            if obj is not None and imgL is not None and imgR is not None and \
               np.size(obj) > 0 and np.size(imgL) > 0 and np.size(imgR) > 0:
                LobjpointsTemp.append(obj)
                LimgpointsTemp.append(imgL)
                RimgpointsTemp.append(imgR)
    else:
        print(F"Error: Lobjpoints, Limgpoints, and Rimgpoints must be the same length, len(Lobjpoints) = {len(Lobjpoints)}, len(Limgpoints) = {len(Limgpoints)}, len(Rimgpoints) = {len(Rimgpoints)}")

    return LobjpointsTemp, LimgpointsTemp, RimgpointsTemp


# Extract the frame index from the file name
# Example file name: 008L (Chessboard A4 Board on Land)-frame00001400.jpeg
#                    025ChArUco1.5mR-frame00046338.jpeg
def GetFrameIndexFromFileName(fileName):
    frameIndex = -1
    fileNameParts = fileName.split("-frame")
    if len(fileNameParts) == 2:
        frameIndex = int(fileNameParts[1].split(".")[0])
    return frameIndex

# Extract the timespan from the file name
def extract_time_span(filename):
    # Regular expression to match the time span in the filename
    regex = r"_(\d{2})_(\d{2})_(\d{2})_(\d{2})\."
    
    # Search for matches in the filename
    match = re.search(regex, filename)
    
    if match:
        # Extract hours, minutes, seconds, and hundredths of a second
        hours = int(match.group(1))
        minutes = int(match.group(2))
        seconds = int(match.group(3))
        hundredths_of_second = int(match.group(4))
        
        # Convert hundredths of a second to milliseconds and create a timedelta
        milliseconds = hundredths_of_second * 10
        return timedelta(hours=hours, minutes=minutes, seconds=seconds, milliseconds=milliseconds)
    else:
        # Return a zero duration timedelta if no match is found
        return timedelta(0)
    

# Assumes that the fileFilterLeft and fileFilterRight are both sorted
# Extract the frame index from the first left and first right file names and calculate the frame offset
# Then check that all the other left and right file names have the same frame offset
def CheckFrameIndexHaveAConsistentOffset(fileFilterLeft, fileFilterRight):
    ret = False

    leftImageFiles = sorted(glob.glob(fileFilterLeft))
    rightImageFiles = sorted(glob.glob(fileFilterRight))

    # Look for a frame index in the file name    
    if GetFrameIndexFromFileName(leftImageFiles[0]) != -1:


        if  len(fileFilterLeft) == len(fileFilterRight):
            if len(fileFilterLeft) > 0:
                calcOffset = True
                ret = True
                for left, right in zip(leftImageFiles, rightImageFiles):
                    if calcOffset == True:
                        frameIndexLeft = GetFrameIndexFromFileName(left)
                        frameIndexRight = GetFrameIndexFromFileName(right)
                        frameOffset = frameIndexLeft - frameIndexRight
                        calcOffset = False

                    frameIndexLeft = GetFrameIndexFromFileName(left)
                    frameIndexRight = GetFrameIndexFromFileName(right)
                    if frameIndexLeft - frameIndexRight != frameOffset:
                        ret = False
                        break
    # look for a timespan in the file name
    else:
        if  len(fileFilterLeft) == len(fileFilterRight):
            if len(fileFilterLeft) > 0:
                calcOffset = True
                ret = True
                for left, right in zip(leftImageFiles, rightImageFiles):
                    if calcOffset == True:
                        timeSpanLeft = extract_time_span(left)
                        timeSpanRight = extract_time_span(right)
                        timeSpanOffset = timeSpanLeft - timeSpanRight

                    timeSpanLeft = extract_time_span(left)
                    timeSpanRight = extract_time_span(right)
                    if timeSpanLeft - timeSpanRight != timeSpanOffset:
                        ret = False
                        break
        

    return ret


# Example usage
#p1a = [1, 2, 3]
#p1b = [4, 5, 6]
#p2a = [7, 8, 9]
#p2b = [10, 11, 12]


import cv2 as cv
import numpy as np
import glob

def stereo_calibrate_circles(fileFilterLeft, fileFilterRight):
    # Termination criteria for the iterative optimization algorithm
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points for a 6x9 grid (or 9x6, depending on orientation)
    objp = np.zeros((4*11, 3), np.float32)
    objp[:, :2] = np.mgrid[0:4, 0:11].T.reshape(-1, 2) * 22  # Circle diameter

    # Arrays to store object points and image points from all the images
    objpoints = []  # 3D points in real-world space
    imgpoints_left = []  # 2D points in image plane for left camera
    imgpoints_right = []  # 2D points in image plane for right camera

    # Reading images
    images_left = glob.glob(fileFilterLeft)
    images_right = glob.glob(fileFilterRight)

    for left_img_path, right_img_path in zip(images_left, images_right):
        img_left = cv.imread(left_img_path)
        img_right = cv.imread(right_img_path)
        gray_left = cv.cvtColor(img_left, cv.COLOR_BGR2GRAY)
        gray_right = cv.cvtColor(img_right, cv.COLOR_BGR2GRAY)

        # Find the circle grid pattern
        ret_left, centers_left = cv.findCirclesGrid(gray_left, (4, 11), None, cv.CALIB_CB_ASYMMETRIC_GRID)
        ret_right, centers_right = cv.findCirclesGrid(gray_right, (4, 11), None, cv.CALIB_CB_ASYMMETRIC_GRID)

        # If found, add object points, image points
        if ret_left and ret_right:
            objpoints.append(objp)

            # Refining the positions
            centers_left = cv.cornerSubPix(gray_left, centers_left, (11, 11), (-1, -1), criteria)
            centers_right = cv.cornerSubPix(gray_right, centers_right, (11, 11), (-1, -1), criteria)

            imgpoints_left.append(centers_left)
            imgpoints_right.append(centers_right)
            
            print(F"Found circles in left:{os.path.basename(left_img_path)} = {len(centers_left)}, right:{os.path.basename(right_img_path)} = {len(centers_right)}, Object Points:{len(objpoints)}")
            
            # Draw and display the corners
            imgLeft = cv.drawChessboardCorners(img_left.copy(), (4, 11), centers_left, ret_left)
            imgRight = cv.drawChessboardCorners(img_right.copy(), (4, 11), centers_right, ret_right)
            imgLeft = cv.resize(imgLeft, (0, 0), fx=0.25, fy=0.25)
            imgRight = cv.resize(imgRight, (0, 0), fx=0.25, fy=0.25)
            cv.imshow('Left', imgLeft)
            cv.imshow('Right', imgRight)
            cv.waitKey(1000)

    # Camera calibration for both cameras
    RMSL, mtxL, distL, _, _ = cv.calibrateCamera(objpoints, imgpoints_left, gray_left.shape[::-1], None, None)
    RMSR, mtxR, distR, _, _ = cv.calibrateCamera(objpoints, imgpoints_right, gray_right.shape[::-1], None, None)

    # Stereo calibration
    flags = 0
 #   flags |= cv.CALIB_FIX_INTRINSIC
    retRMS, _, _, _, _, R, T, _, _ = cv.stereoCalibrate(objpoints, imgpoints_left, imgpoints_right, mtxL, distL, mtxR, distR, gray_left.shape[::-1], criteria=criteria, flags=flags)

    print("Left Camera Matrix:", mtxL)
    print("Left Distortion Coefficient:", distL)
    print("Right Camera Matrix:", mtxR)
    print("Right Distortion Coefficient:", distR)
    print("Left Camera RMS:", RMSL)
    print("Right Camera RMS:", RMSR)
    print("Stereo RMS:", retRMS)
    print("Rotation Matrix:", R)
    print("Translation Vector:", T)

    return mtxL, distL, mtxR, distR, RMSL, RMSR, R, T, retRMS



import cv2.aruco as aruco


def stereo_calibrate_charuco(fileFilterLeft, fileFilterRight):
    
    mtxL = None
    distL = None
    mtxR = None
    distR = None 
    ret_left = None
    ret_right = None
    R = None
    T = None
    RMSL = None
    RMSR = None
    retRMS = None
    
    # Termination criteria for the corner refinement process
    #criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 13, 0.001)

    # Create the dictionary and board object based on the provided specifications
    #charuco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_100)
    #charuco_board = cv.aruco.CharucoBoard((7, 5), 0.042, 0.021, charuco_dict)


#######################
    # Define the ArUco dictionary (e.g., cv.aruco.DICT_6X6_250)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
    # Define the ArUco parameters
    aruco_params = aruco.DetectorParameters()
    # Define the ChArUco board
    charuco_board = aruco.CharucoBoard((9, 14), 40, 30, dictionary=aruco_dict)
    aruco_detector = cv.aruco.ArucoDetector(aruco_dict, aruco_params)
    
#######################

    # Arrays to store object points and image points from all the images
    all_corners_left = []  # Corners detected in all images for left camera
    all_ids_left = []  # Corresponding ids for the corners detected in left camera

    all_corners_right = []  # Corners detected in all images for right camera
    all_ids_right = []  # Corresponding ids for the corners detected in right camera

    # Reading images
    images_left = glob.glob(fileFilterLeft)
    images_right = glob.glob(fileFilterRight)

    fileIndex = 0
    for left_img_path, right_img_path in zip(images_left, images_right):
        img_left = cv.imread(left_img_path)
        img_right = cv.imread(right_img_path)
        gray_left = cv.cvtColor(img_left, cv.COLOR_BGR2GRAY)
        gray_right = cv.cvtColor(img_right, cv.COLOR_BGR2GRAY)

        # Detect ArUco markers in the images
        #corners_left, ids_left, _ = cv.aruco.detectMarkers(gray_left, charuco_dict)
        #corners_right, ids_right, _ = cv.aruco.detectMarkers(gray_right, charuco_dict)
        corners_left, ids_left, _ = aruco_detector.detectMarkers(gray_left)
        corners_right, ids_right, _ = aruco_detector.detectMarkers(gray_right)

        # If markers were detected, process to ChArUco corners
        if ids_left is not None and ids_right is not None:
            # Interpolate the ChArUco corners
            ret_left, charuco_corners_left, charuco_ids_left = cv.aruco.interpolateCornersCharuco(corners_left, ids_left, gray_left, charuco_board)
            ret_right, charuco_corners_right, charuco_ids_right = cv.aruco.interpolateCornersCharuco(corners_right, ids_right, gray_right, charuco_board)

            # If ChArUco corners were detected, add them to the lists
            if ret_left and ret_right:
                all_corners_left.append(charuco_corners_left)
                all_ids_left.append(charuco_ids_left)

                all_corners_right.append(charuco_corners_right)
                all_ids_right.append(charuco_ids_right)
                
                status = "Charuco in left and right images"
            else:
                status = "interpolateCornersCharuco failed L={ret_left}, R={ret_right}"
        else:
            if ids_left is None and ids_right is  None:
                status = "detectMarkers failed on both left and right images"
            elif ids_left is  None:
                status = "detectMarkers failed on the left image"
            else:
                status = "detectMarkers failed on the right image"
                
        print(F"{fileIndex}: {os.path.basename(left_img_path)} & {os.path.basename(left_img_path)} {status}")
        fileIndex += 1


    if (len(all_corners_left) > 0):
        # Calibrate the stereo camera system using ChArUco boards
        ret_left, mtxL, distL, _, _ = cv.aruco.calibrateCameraCharuco(all_corners_left, all_ids_left, charuco_board, gray_left.shape[::-1], None, None)
        ret_right, mtxR, distR, _, _ = cv.aruco.calibrateCameraCharuco(all_corners_right, all_ids_right, charuco_board, gray_right.shape[::-1], None, None)

        # Stereo calibration
        retRMS, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv.stereoCalibrate(
            all_corners_left, all_corners_right, all_ids_left, all_ids_right, charuco_board, mtxL, distL, mtxR, distR, gray_left.shape[::-1], None, None, criteria=criteria, flags=cv.CALIB_FIX_INTRINSIC
        )
    else:
        print("No ChArUco corners were detected in the images. Please try again with different images.")
    
    
    print("Left Camera Matrix:", mtxL)
    print("Left Distortion Coefficient:", distL)
    print("Right Camera Matrix:", mtxR)
    print("Right Distortion Coefficient:", distR)
    print("Left Camera RMS:", ret_left)
    print("Right Camera RMS:", ret_right)
    print("Stereo RMS:", retRMS)
    print("Rotation Matrix:", R)
    print("Translation Vector:", T)

    return mtxL, distL, mtxR, distR, ret_left, ret_right, R, T, retRMS


def display_resized_image(image, desired_width, desired_height, title):
    """
    Display a resized image in a window while preserving its aspect ratio.

    Args:
        image (numpy.ndarray): The input image.
        desired_width (int): The desired width for the window.
        desired_height (int): The desired height for the window.

    Returns:
        None
    """
    # Calculate the new dimensions while preserving the aspect ratio
    if len(image.shape) == 3:
        height, width, _ = image.shape
    else:
        height, width = image.shape
        
    aspect_ratio = width / height

    if aspect_ratio > 1:
        new_width = int(desired_width)
        new_height = int(desired_width / aspect_ratio)
    else:
        new_height = int(desired_height)
        new_width = int(desired_height * aspect_ratio)

    # Resize the image to the calculated dimensions
    image_resized = cv.resize(image, (new_width, new_height))

    # Display the resized image in a window
    cv.imshow(title, image_resized)




def GitHub_Circles_Calibration(fileFilterLeft, fileFilterRight):
    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    ########################################Blob Detector##############################################

    # Setup SimpleBlobDetector parameters.
    blobParams = cv.SimpleBlobDetector_Params()

    # Change thresholds
    blobParams.minThreshold = 8
    blobParams.maxThreshold = 255

    # Filter by Area.
    blobParams.filterByArea = True
    blobParams.minArea = 64     # minArea may be adjusted to suit for your experiment
    blobParams.maxArea = 2500   # maxArea may be adjusted to suit for your experiment

    # Filter by Circularity
    blobParams.filterByCircularity = True
    blobParams.minCircularity = 0.1

    # Filter by Convexity
    blobParams.filterByConvexity = True
    blobParams.minConvexity = 0.87

    # Filter by Inertia
    blobParams.filterByInertia = True
    blobParams.minInertiaRatio = 0.01

    # Create a detector with the parameters
    blobDetector = cv.SimpleBlobDetector_create(blobParams)

    ###################################################################################################

    ###################################################################################################

    # Original blob coordinates, supposing all blobs are of z-coordinates 0
    # And, the distance between every two neighbour blob circle centers is 72 centimetres
    # In fact, any number can be used to replace 72.
    # Namely, the real size of the circle is pointless while calculating camera calibration parameters.
    objp = np.zeros((44, 3), np.float32)
    objp[0]  = (0  , 0  , 0)
    objp[1]  = (0  , 72 , 0)
    objp[2]  = (0  , 144, 0)
    objp[3]  = (0  , 216, 0)
    objp[4]  = (36 , 36 , 0)
    objp[5]  = (36 , 108, 0)
    objp[6]  = (36 , 180, 0)
    objp[7]  = (36 , 252, 0)
    objp[8]  = (72 , 0  , 0)
    objp[9]  = (72 , 72 , 0)
    objp[10] = (72 , 144, 0)
    objp[11] = (72 , 216, 0)
    objp[12] = (108, 36,  0)
    objp[13] = (108, 108, 0)
    objp[14] = (108, 180, 0)
    objp[15] = (108, 252, 0)
    objp[16] = (144, 0  , 0)
    objp[17] = (144, 72 , 0)
    objp[18] = (144, 144, 0)
    objp[19] = (144, 216, 0)
    objp[20] = (180, 36 , 0)
    objp[21] = (180, 108, 0)
    objp[22] = (180, 180, 0)
    objp[23] = (180, 252, 0)
    objp[24] = (216, 0  , 0)
    objp[25] = (216, 72 , 0)
    objp[26] = (216, 144, 0)
    objp[27] = (216, 216, 0)
    objp[28] = (252, 36 , 0)
    objp[29] = (252, 108, 0)
    objp[30] = (252, 180, 0)
    objp[31] = (252, 252, 0)
    objp[32] = (288, 0  , 0)
    objp[33] = (288, 72 , 0)
    objp[34] = (288, 144, 0)
    objp[35] = (288, 216, 0)
    objp[36] = (324, 36 , 0)
    objp[37] = (324, 108, 0)
    objp[38] = (324, 180, 0)
    objp[39] = (324, 252, 0)
    objp[40] = (360, 0  , 0)
    objp[41] = (360, 72 , 0)
    objp[42] = (360, 144, 0)
    objp[43] = (360, 216, 0)
    ###################################################################################################

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints_left = [] # 2d points in image plane.
    imgpoints_right = [] # 2d points in image plane.


    # Reading images
    images_left = glob.glob(fileFilterLeft)
    images_right = glob.glob(fileFilterRight)

    fileIndex = 0
    found = 0
    for left_img_path, right_img_path in zip(images_left, images_right):
        img_left = cv.imread(left_img_path)
        img_right = cv.imread(right_img_path)
        gray_left = cv.cvtColor(img_left, cv.COLOR_BGR2GRAY)
        gray_right = cv.cvtColor(img_right, cv.COLOR_BGR2GRAY)

        # Detect blobs.
        keypoints_left = blobDetector.detect(gray_left)
        keypoints_right = blobDetector.detect(gray_right)

        # Draw detected blobs as red circles. This helps cv2.findCirclesGrid() .
        im_with_keypoints_left = cv.drawKeypoints(img_left, keypoints_left, np.array([]), (0,255,0), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        im_with_keypoints_gray_left = cv.cvtColor(im_with_keypoints_left, cv.COLOR_BGR2GRAY)
        retL, corners_left = cv.findCirclesGrid(im_with_keypoints_left, (4,11), None, flags = cv.CALIB_CB_ASYMMETRIC_GRID)   # Find the circle grid

        im_with_keypoints_right = cv.drawKeypoints(img_right, keypoints_right, np.array([]), (0,255,0), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        im_with_keypoints_gray_right = cv.cvtColor(im_with_keypoints_right, cv.COLOR_BGR2GRAY)
        retR, corners_right = cv.findCirclesGrid(im_with_keypoints_right, (4,11), None, flags = cv.CALIB_CB_ASYMMETRIC_GRID)   # Find the circle grid

        im_with_keypoints_left2 = cv.resize(im_with_keypoints_left, (0, 0), fx=0.5, fy=0.5)
        im_with_keypoints_right2 = cv.resize(im_with_keypoints_right, (0, 0), fx=0.5, fy=0.5)
        cv.imshow("Left", im_with_keypoints_left2) # display
        cv.imshow("Right", im_with_keypoints_right2) # display
        cv.waitKey(2)

        
        if retL == True and retR == True:
            objpoints.append(objp)  # Certainly, every loop objp is the same, in 3D.

            #corners2_left = cv.cornerSubPix(im_with_keypoints_gray_left, corners_left, (11,11), (-1,-1), criteria)    # Refines the corner locations.
            corners2_left = corners_left
            imgpoints_left.append(corners2_left)
            
   
            #corners2_right = cv.cornerSubPix(im_with_keypoints_gray_right, corners_right, (11,11), (-1,-1), criteria)    # Refines the corner locations.
            corners2_right = corners_right
            imgpoints_right.append(corners2_right)
            

            # Draw and display the corners.
            im_with_keypoints_left = cv.drawChessboardCorners(img_left, (4,11), corners2_left, retL)
            im_with_keypoints_right = cv.drawChessboardCorners(img_right, (4,11), corners2_right, retR)
            found += 1

        im_with_keypoints_left = cv.resize(im_with_keypoints_left, (0, 0), fx=0.5, fy=0.5)
        im_with_keypoints_right = cv.resize(im_with_keypoints_right, (0, 0), fx=0.5, fy=0.5)
        #cv.imshow("Left", im_with_keypoints_left) # display
        #cv.imshow("Right", im_with_keypoints_right) # display
        #cv.waitKey(2)

    # When everything done, release the capture
    cv.destroyAllWindows()
    #print("Option e: Left Object Points:")
    #print(objpoints)
    #print("Option e: Left Image Points:")
    #print(imgpoints_left)
    #print("Option e: Width:{gray_left.shape[0]}, Height:{gray_left.shape[1]}")
    
    #print(F"Number of image points sets = {len(imgpoints_left)}")
    
    ret_left, mtxL, distL, _, _ = cv.calibrateCamera(objpoints, imgpoints_left, gray_left.shape[::-1], None, None)
    #print(F"Option e: RMSL = {ret_left}")
    #print(F"Option e: mtxL = {mtxL}")
    #print(F"Option e: distL = {distL}")
    
    ret_right, mtxR, distR, _, _ = cv.calibrateCamera(objpoints, imgpoints_right, gray_right.shape[::-1], None, None)

    # Stereo calibration
    flags = 0
    #flags |= cv.CALIB_FIX_INTRINSIC  # Removing this seems to improve the stereo RMS
    retRMS, _, _, _, _, R, T, _, _ = cv.stereoCalibrate(objpoints, imgpoints_left, imgpoints_right, mtxL, distL, mtxR, distR, gray_left.shape[::-1], criteria=criteria, flags=flags)


    print("Left Camera Matrix:", mtxL)
    print("Left Distortion Coefficient:", distL)
    print("Right Camera Matrix:", mtxR)
    print("Right Distortion Coefficient:", distR)
    print("Left Camera RMS:", ret_left)
    print("Right Camera RMS:", ret_right)
    print("Stereo RMS:", retRMS)
    print("Rotation Matrix:", R)
    print("Translation Vector:", T)

    return  mtxL, distL, mtxR, distR, ret_left, ret_right, R, T, retRMS







 
#################### MAIN ####################
mtxL = None
distL = None
mtxR = None
distR = None
R = None
T = None
CalibLoaded = False
 
# 023 Point1
L1 = [[1851,1145],[1997,1136],[1921,1121],[1925,1154]]
R1 = [[1914,1132],[2062,1121],[1985,1109],[1988,1143]]

# 023 Point2
L2 = [[1800,1245],[1994,1254],[1899,1224],[1894,1273]]
R2 = [[1792,1233],[1987,1242],[1890,1212],[1889,1260]]

# 023 Point3
L3 = [[1822,1202],[2207,1208],[2012,1155],[2011,1251]]
R3 = [[1540,1188],[1917,1193],[1728,1140],[1724,1237]]

# 023 Point4
L4 = [[2028,1218],[2563,1229],[2288,1154],[2288,1286]]
R4 = [[1546,1204],[2041,1216],[1780,1140],[1785,1272]]

# 023 Point5
L5 = [[2766,1582],[1749,1519],[2248,1670],[2267,1416]]
R5 = [[604,1485],[1536,1577],[1050,1644],[1057,1396]]

importDirectory = "C:/Users/tobyh/OneDrive/Docs/Training/Opwall/SVS2/SVS2 Media/Frames"
calibDirectory = "C:/Users/tobyh/OneDrive/Docs/Training/Opwall/SVS2/SVS2 Media/Calib Files"

while 1==1:
    ##### User Menu #####
    print("This script creates a file containing calibration information for stereo vision")
    print("")
    print("Select from the following options:")
    print(F"1. Calibrate from images from a selected folder where [{importDirectory}] is the default directory")
    print(F"2. Calibrate from images in the Calibration/ folder")
    print(F"3. Triangulate Test Point 1:{L1} & {R1}")
    print(F"4. Triangulate Test Point 2:{L2} & {R2}")
    print(F"5. Triangulate Test Point 3:{L3} & {R3}")
    print(F"6. Triangulate Test Point 4:{L4} & {R4}")
    print(F"7. Triangulate Test Point 5:{L5} & {R5}")
    print(F"8. Legacy Calibrate from images from a selected folder where [{importDirectory}] is the default directory")
    print(F"9. Triangulate images in the Trangulation/ folder")
    print(F"a. Calibrate using ChatGPT code from circles images from a selected folder where [{importDirectory}] is the default directory")
    print(F"b. Calibrate using ChatGPT code from charuco images from a selected folder where [{importDirectory}] is the default directory")
    print(F"c. Show a Charuco board ")
    print(F"d. Show a Cirlces board ")
    print(F"e. Calibrate using GitHub example from circles images from a selected folder where [{importDirectory}] is the default directory")
    print(F"q. Exit")
    keyPressed = input("")

    if keyPressed == "1" or keyPressed == "2" or keyPressed == "8" or keyPressed == "a" or keyPressed == "b" or keyPressed == "e":
        if keyPressed == "1":
            importDirectoryUserInput = input("Enter a directory to import images from or press enter to use the default directory ["+importDirectory+"]:")
            if (importDirectoryUserInput != ""):
                importDirectory = importDirectoryUserInput
            print("Enter the string to be used in the file filter e.g. 008 to find 008L*.jpeg/png and 008R*.jpeg/png files")
            filterNumber = input("")
            print("Enter the picture file extension to use i.e. either jpeg or png:")
            filterExt = input("")

            fileFilterLeft = os.path.join(importDirectory, filterNumber+'L*.'+filterExt)
            fileFilterRight = os.path.join(importDirectory, filterNumber+'R*.'+filterExt)
            calibFileSpec = F"{calibDirectory}/{filterNumber.strip()}.calib"

        if keyPressed == "2":
            print("")
            print("Enter the three digit number to be used in the file filter in the Calibration/ directory e.g. 008 to find Calibration/008L*.jpeg and Calibration/008R*.jpeg files")
            filterNumber = input("")
            fileFilterLeft = 'Calibration/'+filterNumber+'L*.jpeg'  # {.jpeg,.png}
            fileFilterRight = 'Calibration/'+filterNumber+'R*.jpeg'
            calibFileSpec = F"Calibration/{filterNumber.strip()}.calib"

        if keyPressed == "8":
            importDirectoryUserInput = input("Enter a directory to import images from or press enter to use the default directory ["+importDirectory+"]:")
            if (importDirectoryUserInput != ""):
                importDirectory = importDirectoryUserInput
            print("Enter the string to be used in the file filter e.g. 008 to find 008L*.jpeg and 008R*.jpeg files")
            filterNumber = input("")
            fileFilterLeft = os.path.join(importDirectory, filterNumber+'L*.jpeg')
            fileFilterRight = os.path.join(importDirectory, filterNumber+'R*.jpeg')
            calibFileSpec = F"{calibDirectory}/{filterNumber.strip()}.calib"
            mtxL, distL = calibrate_camera_Legacy(fileFilterLeft)
            mtxR, distR = calibrate_camera_Legacy(fileFilterRight)

            retRMS, R, T = stereo_calibrate_Legacy(mtxL, distL, mtxR, distR, fileFilterLeft, fileFilterRight)
            exit(0)
        
        if keyPressed == "a":
            importDirectoryUserInput = input("Enter a directory to import images from or press enter to use the default directory ["+importDirectory+"]:")
            if (importDirectoryUserInput != ""):
                importDirectory = importDirectoryUserInput
            print("Enter the string to be used in the file filter e.g. 008 to find 008L*.jpeg and 008R*.jpeg files")
            filterNumber = input("")
            fileFilterLeft = os.path.join(importDirectory, filterNumber+'L*.jpeg')
            fileFilterRight = os.path.join(importDirectory, filterNumber+'R*.jpeg')
            mtxL, distL, mtxR, distR, RMSL, RMSR, R, T, retRMS = stereo_calibrate_circles(fileFilterLeft, fileFilterRight)
            exit(0)
  
        if keyPressed == "b":
            importDirectoryUserInput = input("Enter a directory to import images from or press enter to use the default directory ["+importDirectory+"]:")
            if (importDirectoryUserInput != ""):
                importDirectory = importDirectoryUserInput
            print("Enter the string to be used in the file filter e.g. 008 to find 008L*.jpeg and 008R*.jpeg files")
            filterNumber = input("")
            #fileFilterLeft = os.path.join(importDirectory, filterNumber+'L*.jpeg')
            #fileFilterRight = os.path.join(importDirectory, filterNumber+'R*.jpeg')
            fileFilterLeft = os.path.join(importDirectory, filterNumber+'L*.png')
            fileFilterRight = os.path.join(importDirectory, filterNumber+'R*.png')

            mtxL, distL, mtxR, distR, RMSL, RMSR, R, T, retRMS = stereo_calibrate_charuco(fileFilterLeft, fileFilterRight)
            exit(0)
  
        if keyPressed == "e":
            importDirectoryUserInput = input("Enter a directory to import images from or press enter to use the default directory ["+importDirectory+"]:")
            if (importDirectoryUserInput != ""):
                importDirectory = importDirectoryUserInput
            print("Enter the string to be used in the file filter e.g. 008 to find 008L*.jpeg and 008R*.jpeg files")
            filterNumber = input("")
            fileFilterLeft = os.path.join(importDirectory, filterNumber+'L*.jpeg')
            fileFilterRight = os.path.join(importDirectory, filterNumber+'R*.jpeg')
            mtxL, distL, mtxR, distR, ret_left, ret_right, R, T, retRMS = GitHub_Circles_Calibration(fileFilterLeft, fileFilterRight)            
            exit(0)
            
        
        # Check that the frame index at the end of the file name for each image 
        # has a constant offset between the left and right images
        if CheckFrameIndexHaveAConsistentOffset(fileFilterLeft, fileFilterRight) == True:        
            
            CalibLoaded = False 
                        
            # Pattern paramters
            ChessPatternSize = 34.85  # 34.85mm is the size of the squares on the A3 board
            ChessPatternRows = 6
            ChessPatternColumns = 9 
            CirclesPatternSize = 54.6 # (Actual distance between centres) / SQR(POW(0.5,2)+POW(0.5,2))   # 38.75mm distance between circles   # 22   # 22mm is the size of the circles on the A3 board
            CirclesPatternRows = 4
            CirclesPatternColumns = 11 
        
            # Find the left image object and image points and calibrate the left camera
            Lret, LboardType, Lobjpoints, Limgpoints, LimageWidth, LimageHeight, LpatterRows, LpatterColumns = findCalibrationObjectPoints(fileFilterLeft, 
                                                                                                                                        ChessPatternSize, ChessPatternRows, ChessPatternColumns, 
                                                                                                                                        CirclesPatternSize, CirclesPatternRows, CirclesPatternColumns)
            # Find the right image object and image points and calibrate the right camera
            Rret, RboardType, Robjpoints, Rimgpoints, RimageWidth, RimageHeight, RpatterRows, RpatterColumns = findCalibrationObjectPoints(fileFilterRight, 
                                                                                                                                        ChessPatternSize, ChessPatternRows, ChessPatternColumns, 
                                                                                                                                        CirclesPatternSize, CirclesPatternRows, CirclesPatternColumns)
            # Parse the Lobjpoints, Limgpoints & Rimgpoints lists and discard any points that are not in all three lists
            LimageTotal = len(Limgpoints)
            RimageTotal = len(Rimgpoints)
            Lobjpoints, Limgpoints, Rimgpoints = DiscardBlankPointAndMaintainAlignedLists(Lobjpoints, Limgpoints, Rimgpoints)


            if Lret == 0:
                #print("Option 1: Object Points:")
                #print(Lobjpoints)
                #print("Option 1: Image Points:")
                #print(Limgpoints)
                #print(F"Option 1: Width:{LimageWidth}, Height: {LimageHeight}")
                RMSL, mtxL, distL = calibrate_camera("Left", Lobjpoints, Limgpoints, LimageWidth, LimageHeight)
                #print(F"Option 1: RMS:{RMSL}")
                #print(F"Option 1: Camera Matrix:{mtxL}")
                #print(F"Option 1: Distortion Coefficients:{distL}")
            else:    
                print("Image and object points not found. Left camera calibration failed!")
            
            if Rret == 0:        
                RMSR, mtxR, distR = calibrate_camera("Right", Lobjpoints, Rimgpoints, RimageWidth, RimageHeight)   # Ok to use left Lobjpoints
            else:    
                print("Image and object points not found. Right camera calibration failed!")

            
            print(F"Number of image points sets = {len(Limgpoints)}")
     

                    
            # Calibrate the stereo cameras if both cameras are calibrated 
            # and the images are the same size
            if (Lret == 0 and Rret == 0):
                if (LboardType ==  RboardType):
                    if (LimageWidth == RimageWidth and LimageHeight == RimageHeight):
                        if (len(Limgpoints) == len(Rimgpoints) == len(Lobjpoints)):                    
                            retRMS, R, T = stereo_calibrate(Lobjpoints, Limgpoints, Rimgpoints, mtxL, distL, mtxR, distR, LimageWidth, LimageHeight)
                
                            # Save the calibration data        
                            calibration_Data = CalibrationData()
                            calibration_Data.Description = F"{LboardType} A3 calibration board {filterNumber}"
                            calibration_Data.LeftCalibrationCameraData.mtx = mtxL
                            calibration_Data.LeftCalibrationCameraData.dist = distL
                            calibration_Data.LeftCalibrationCameraData.retRMS = RMSL
                            calibration_Data.LeftCalibrationCameraData.imageTotal = LimageTotal
                            calibration_Data.LeftCalibrationCameraData.imageUseable = len(Limgpoints)
                            calibration_Data.RightCalibrationCameraData.mtx = mtxR
                            calibration_Data.RightCalibrationCameraData.dist = distR
                            calibration_Data.RightCalibrationCameraData.retRMS = RMSR
                            calibration_Data.RightCalibrationCameraData.imageTotal = RimageTotal
                            calibration_Data.RightCalibrationCameraData.imageUseable = len(Rimgpoints)
                            calibration_Data.CalibrationStereoCameraData.rotation = R
                            calibration_Data.CalibrationStereoCameraData.translation = T
                            calibration_Data.CalibrationStereoCameraData.retRMS = retRMS
                            calibration_Data.CalibrationStereoCameraData.imageTotal = LimageTotal
                            calibration_Data.CalibrationStereoCameraData.imageUseable = len(Limgpoints)
                                
                                
                            print(F"Description:{calibration_Data.Description}")
                            print(F"Left:   (RMS{calibration_Data.LeftCalibrationCameraData.retRMS})")
                            print(F"   Camera Matrix: {calibration_Data.LeftCalibrationCameraData.mtx}")
                            print(F"   Distortion Coefficients: {calibration_Data.LeftCalibrationCameraData.dist}")
                            print(F"Right:   (RMS{calibration_Data.RightCalibrationCameraData.retRMS})")
                            print(F"   Camera Matrix: {calibration_Data.RightCalibrationCameraData.mtx}")
                            print(F"   Distortion Coefficients: {calibration_Data.RightCalibrationCameraData.dist}")
                            print(F"Stereo:")
                            print(F"   RMS: {calibration_Data.CalibrationStereoCameraData.retRMS}")
                            print(F"   Rotation: {calibration_Data.CalibrationStereoCameraData.rotation}")
                            print(F"   Translation: {calibration_Data.CalibrationStereoCameraData.translation}")
    
                            
                            # Remove an ? from the file name
                            calibFileSpec = calibFileSpec.replace("?", "").strip()
                            
                            # Try to save the calibration file                   
                            if (calibration_Data.SaveCameraCalibration(calibFileSpec) == True):
                                print("Calibration complete!")
                                CalibLoaded = True
                            else:
                                print(F"Calibration Failed to save to {calibFileSpec}!")                        
                        else:
                            print(F"Number of image points and object points do not match. Calibration Failed! LImgPoints={len(Limgpoints)}, RImgPoints={len(Rimgpoints)}, LObjPoints={len(Lobjpoints)}")
                    else:
                        print("Images are not the same size. Calibration Failed!")
                else:
                    print(F"Type of calibration boards do not match. Left:{LboardType}, Right:{RboardType}. Calibration Failed!")

        
    elif keyPressed == "3" or keyPressed == "4" or keyPressed == "5" or keyPressed == "6" or keyPressed == "7" or keyPressed == "9":
        if keyPressed == "3":
            LCoordinateDump = L1
            RCoordinateDump = R1
        if keyPressed == "4":
            LCoordinateDump = L2
            RCoordinateDump = R2
        if keyPressed == "5":
            LCoordinateDump = L3
            RCoordinateDump = R3
        if keyPressed == "6":
            LCoordinateDump = L4
            RCoordinateDump = R4
        if keyPressed == "7":
            LCoordinateDump = L5
            RCoordinateDump = R5
            
        if keyPressed == "3" or keyPressed == "4" or keyPressed == "5" or keyPressed == "6" or keyPressed == "7":
            filterNumber = input("Enter the three digit number of the calibration file: ")
            calibFileSpec = F"{calibDirectory}/{filterNumber}.calib"
                
        if keyPressed == "9":
            print("")
            print("Enter the three digit number to be used in the file filter e.g. 008 to find 008L*.jpeg and 008R*.jpeg files")
            filterNumber = input("")
            calibFileSpec = F"Calibration/{filterNumber}.calib"
        
        
        loadCalibFileRequest = False
        # Check if the is a matching calibration file        
        if os.path.isfile(calibFileSpec) and CalibLoaded == False:
            print(F"Calibration file {calibFileSpec} exists. Loading calibration data.")
            loadCalibFileRequest = True
        elif os.path.isfile(calibFileSpec) and CalibLoaded == True:
            print(F"Calibration data is already loaded or do you want to load from file {calibFileSpec}. Type Y to load from file.")
            yesnoAnswer = input("")
            if yesnoAnswer == "Y" or yesnoAnswer == "y":
                loadCalibFileRequest = True
        
        if loadCalibFileRequest == True:
            calibration_Data = CalibrationData()
            if (calibration_Data.LoadCameraCalibration(calibFileSpec) == True):
                mtxL = calibration_Data.LeftCalibrationCameraData.mtx
                distL = calibration_Data.LeftCalibrationCameraData.dist
                mtxR = calibration_Data.RightCalibrationCameraData.mtx
                distR = calibration_Data.RightCalibrationCameraData.dist
                R = calibration_Data.CalibrationStereoCameraData.rotation
                T = calibration_Data.CalibrationStereoCameraData.translation
                
                # Display Calibration Data
                print(F"Description:{calibration_Data.Description}")
                print(F"Left:   (RMS{calibration_Data.LeftCalibrationCameraData.retRMS})")
                print(F"   Camera Matrix: {calibration_Data.LeftCalibrationCameraData.mtx}")
                print(F"   Distortion Coefficients: {calibration_Data.LeftCalibrationCameraData.dist}")
                print(F"Right:   (RMS{calibration_Data.RightCalibrationCameraData.retRMS})")
                print(F"   Camera Matrix: {calibration_Data.RightCalibrationCameraData.mtx}")
                print(F"   Distortion Coefficients: {calibration_Data.RightCalibrationCameraData.dist}")
                print(F"Stereo:")
                print(F"   RMS: {calibration_Data.CalibrationStereoCameraData.retRMS}")
                print(F"   Rotation: {calibration_Data.CalibrationStereoCameraData.rotation}")
                print(F"   Translation: {calibration_Data.CalibrationStereoCameraData.translation}")
                            
                CalibLoaded = True
            else:
                CalibLoaded = False
            
        if CalibLoaded == True: 
            if keyPressed == "3" or keyPressed == "4" or keyPressed == "5" or keyPressed == "6" or keyPressed == "7":
                triangulateNoDisplay(mtxL, mtxR, R, T, LCoordinateDump, RCoordinateDump)
                       
            if keyPressed == "9":
                fileFilterLeft = 'Trangulation/'+filterNumber+'L*.jpeg'
                fileFilterRight = 'Trangulation/'+filterNumber+'R*.jpeg'        
                triangulateWithDisplay(mtxL, mtxR, R, T, fileFilterLeft, fileFilterRight)
        else:
            print("Calibration data is not loaded. Please calibrate first.")

    elif keyPressed == "c":
        
        #import cv2, PIL, os
        from mpl_toolkits.mplot3d import Axes3D
        import matplotlib.pyplot as plt
        import matplotlib as mpl
        #import pandas as pd
        #%matplotlib nbagg

        print('CharucoBoard_create' in dir(cv.aruco))  # This should return True if the function is available

        workdir = "./"
        #aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        #aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        #board = cv.aruco.CharucoBoard_create(7, 5, 1, .8, aruco_dict)
        aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_100)
        board = cv.aruco.CharucoBoard((5, 7), 42, 21, aruco_dict)

        #imboard = board.draw((2000, 2000))
        # Assuming 'board' is your CharucoBoard object
        width, height = 2160, 3840  # Define the size of the image
        imboard = np.zeros((height, width, 3), dtype=np.uint8)

        # Draw the Charuco board on the image
        imboard = cv.aruco.drawPlanarBoard(board, (width, height), marginSize=0, borderBits=1)

        #cv.imwrite(workdir + "chessboard.tiff", imboard)
        imboard = cv.imread( R"C:\Users\tobyh\OneDrive\Pictures\Samsung Gallery\DCIM\Camera\20231210_000427.jpg")
        fig = plt.figure()
        ax = fig.add_subplot(1,1,1)
        plt.imshow(imboard, cmap = mpl.cm.gray, interpolation = "nearest")
        ax.axis("off")
        plt.show()
                    
        #fileFilterLeft = R"C:\Users\tobyh\OneDrive\Docs\Training\Opwall\SVS2\SVS2 Media\Frames\028Charuco2mL*.jpeg"
        #fileFilterLeft = workdir + "chessboard.tiff"
        #fileFilterLeft = R"C:\Users\tobyh\OneDrive\Docs\Training\Opwall\SVS2\SVS2 Media\Frames\034CharUco2mL-frame00025222.jpeg"
        #fileFilterLeft = R"C:\Users\tobyh\OneDrive\Pictures\Samsung Gallery\DCIM\Camera\20231210_000427.jpg"
        fileFilterLeft = R"C:\Users\tobyh\OneDrive\Docs\Training\Opwall\SVS2\SVS2 Media\Frames\025Charuco2.0mL*.jpeg"
        images_left = glob.glob(fileFilterLeft)
        images_left.sort()
        images = images_left
       # print(images) 
        
        def read_chessboards(images):
            """
            Charuco base pose estimation.
            """
            print("POSE ESTIMATION STARTS:")
            allCorners = []
            allIds = []
            
            
            decimator = 0
            # SUB PIXEL CORNER DETECTION CRITERION
            #criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.00001)
            criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

            for im in images:
                print("=> Processing image {0}".format(im))
                frame = cv.imread(im)
                gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
                corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray, aruco_dict)

                if len(corners)>0:
                    # SUB PIXEL DETECTION
                    for corner in corners:
                        cv.cornerSubPix(gray, corner,
                                        winSize = (3,3),
                                        zeroZone = (-1,-1),
                                        criteria = criteria)
                    res2 = cv.aruco.interpolateCornersCharuco(corners,ids,gray,board)
                    if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%1==0:
                        allCorners.append(res2[1])
                        allIds.append(res2[2])

                    if allIds is not None and allCorners is not None:
                        image = cv.aruco.drawDetectedMarkers(frame, corners, ids)
                        image = cv.aruco.drawDetectedCornersCharuco(image, res2[1], res2[2])
                        # Resize the image to 500x500 pixels
                        height, width = image.shape[:2]
                        resized_image = cv.resize(image, (int(width/4), int(height/4)))
                        # Display the resized image
                        cv.imshow('Resized Charuco Corners', resized_image)
                        cv.waitKey(0)
                        cv.destroyAllWindows()

                decimator+=1

            imsize = gray.shape
            return allCorners,allIds,imsize
        
        allCorners,allIds,imsize=read_chessboards(images)

        print(F"Charuco Corner count = {len(allCorners)}")
        print(F"Charuco marker count = {len(allIds)}")

        def calibrate_camera(allCorners,allIds,imsize):
            """
            Calibrates the camera using the dected corners.
            """
            print("CAMERA CALIBRATION")

            cameraMatrixInit = np.array([[ 1000.,    0., imsize[0]/2.],
                                        [    0., 1000., imsize[1]/2.],
                                        [    0.,    0.,           1.]])

            distCoeffsInit = np.zeros((5,1))
            flags = (cv.CALIB_USE_INTRINSIC_GUESS + cv.CALIB_RATIONAL_MODEL + cv.CALIB_FIX_ASPECT_RATIO)
            #flags = (cv2.CALIB_RATIONAL_MODEL)
            (ret, camera_matrix, distortion_coefficients0,
            rotation_vectors, translation_vectors,
            stdDeviationsIntrinsics, stdDeviationsExtrinsics,
            perViewErrors) = cv.aruco.calibrateCameraCharucoExtended(
                            charucoCorners=allCorners,
                            charucoIds=allIds,
                            board=board,
                            imageSize=imsize,
                            cameraMatrix=cameraMatrixInit,
                            distCoeffs=distCoeffsInit,
                            flags=flags,
                            criteria=(cv.TERM_CRITERIA_EPS & cv.TERM_CRITERIA_COUNT, 10000, 1e-9))

            return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors

        if (len(allCorners) > 0 and len(allIds) > 0 ):
            ret, mtx, dist, rvecs, tvecs = calibrate_camera(allCorners,allIds,imsize)
            print(F"Stereo RMS={ret}") 
      
    elif keyPressed == "d":              
        pattern_size = (4, 3)  # Number of circles across (columns) and down (rows)
        pattern_type = cv.CALIB_CB_ASYMMETRIC_GRID  # Use asymmetric grid
        board_image = np.zeros((480, 640), dtype=np.uint8)  # Create a blank image (modify the size as needed)
        centers = [cv.KeyPoint(x * 40, y * 40, 20) for y in range(4) for x in range(3)]  # Modify spacing and size as needed
        centers = np.array([kp.pt for kp in centers], dtype=np.float32)
        #cv.drawChessboardCorners(board_image, pattern_size, centers, True)
        cv.imshow('Circles Grid', board_image)
        cv.waitKey(0)
        cv.destroyAllWindows()
        #cv.imwrite('circles_grid.jpg', board_image)


    elif keyPressed == "q":
        exit(0)
        
