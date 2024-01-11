import cv2 as cv
import numpy as np
import os

class Kamera:
    def __init__(self):
        self.load_calibration()
        self.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
        self.aruco_params = cv.aruco.DetectorParameters_create()
        self.cap = cv.VideoCapture(0) #capturing default camera feed
        self.last_detected_distance = None # set the variable to save the last calculated distance

    def load_calibration(self):
        # Use the path specified in the calibration file
        calibration_data_path = 'calib_data/calibration.npz'
        print(f"Calibration file path: {calibration_data_path}")

        if not os.path.exists(calibration_data_path):
            print(f"Error: Calibration file '{calibration_data_path}' not found.")
            return False

        self.calibration_data = np.load(calibration_data_path)
        self.camera_matrix = self.calibration_data['camera_matrix']
        self.distortion_coefficients = self.calibration_data['distortion_coefficients']
        return True

    def calculate_distance(self):
        while True:
            ret, frame = self.cap.read() #capture the frame from the camera
            if not ret:
                print("Camera not available.")
                break
            
            undistorted_frame = cv.undistort(frame, self.camera_matrix, self.distortion_coefficients)
            img_gray = cv.cvtColor(undistorted_frame, cv.COLOR_BGR2GRAY)
            
            corners, _, _ = cv.aruco.detectMarkers(img_gray, self.aruco_dict, parameters=self.aruco_params)

                # detect Aruco markers
            if corners is not None and len(corners) > 0:
                rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
                    corners, 6, self.camera_matrix, self.distortion_coefficients
                ) #finding the pose of aruco marker

                distance = np.sqrt(tvecs[0][0][2] ** 2 + tvecs[0][0][0] ** 2 + tvecs[0][0][1] ** 2) #calculate distance
                self.last_detected_distance = distance #store the calculated value
                print(f"Distance: {distance:.2f}")
            else:
                print("No ArUco markers detected.")
                self.last_detected_distance = None # if no aruco marker detected

kamera = Kamera()
kamera.calculate_distance()

