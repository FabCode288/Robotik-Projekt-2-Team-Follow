import cv2 as cv
import numpy as np

calibration_data = np.load('calibration.npz') #muss noch erstellt werden
camera_matrix = calibration_data['camera_matrix']
distortion_coefficients = calibration_data['distortion_coefficients']

# Create an ArUco dictionary -> anpassen
aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)

# Create the ArUco parameters -> anpassen
aruco_params = cv.aruco.DetectorParameters_create()


# Open a connection to the camera 
cap = cv.VideoCapture(0)  # Use 0 for /dev/video0

while True:
    # Capture a frame
    ret, frame = cap.read()

    # Undistort the frame
    undistorted_frame = cv.undistort(frame, camera_matrix, distortion_coefficients)

    img_gray = cv.cvtColor(undistorted_frame, cv.COLOR_BGR2GRAY)

    # Detect ArUco markers in the undistorted frame
    corners, ids, rejected = cv.aruco.detectMarkers(undistorted_frame, aruco_dict, parameters=aruco_params)

    if ids is not None:
        #aruco detected, abstand berechnen
        abstand = 20
    else:
        abstand = None

    

# Release the camera and close all windows
cap.release()
cv.destroyAllWindows()