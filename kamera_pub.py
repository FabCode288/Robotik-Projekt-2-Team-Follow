import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Kamera(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
        self.aruco_params = cv.aruco.DetectorParameters_create()
        self.cap = cv.VideoCapture(0)

        self.load_calibration()

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def load_calibration(self):
        # Use the path specified in the calibration file
        calibration_data_path = 'calib_data/calibration.npz'  # Replace with your specified path
        print(f"Calibration file path: {calibration_data_path}")

        if not os.path.exists(calibration_data_path):
            print(f"Error: Calibration file '{calibration_data_path}' not found.")
            return False

        self.calibration_data = np.load(calibration_data_path)
        self.camera_matrix = self.calibration_data['camera_matrix']
        self.distortion_coefficients = self.calibration_data['distortion_coefficients']
        return True

    def run(self):
        ret, frame = self.cap.read()
        if not ret:
            return None, None
        
        undistorted_frame = cv.undistort(frame, self.camera_matrix, self.distortion_coefficients)
        img_gray = cv.cvtColor(undistorted_frame, cv.COLOR_BGR2GRAY)
        
        corners, ids, _ = cv.aruco.detectMarkers(img_gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
                corners, 6, self.camera_matrix, self.distortion_coefficients
            )
            distances = [np.sqrt(tvecs[i][0][2] ** 2 + tvecs[i][0][0] ** 2 + tvecs[i][0][1] ** 2) for i in range(len(ids))]
            return ids, distances
        else:
            return None, None

    def timer_callback(self):
        marker_ids, distances = self.run()
        if marker_ids is not None:
            msg = String()
            msg.data = f"Marker IDs: {marker_ids}, Distances: {distances}"
            self.publisher_.publish(msg)

def main():
    rclpy.init()
    minimal_publisher = Kamera()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
import cv2 as cv
import numpy as np
import os

class Kamera:

    def __init__(self):
        self.load_calibration()
        self.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
        self.aruco_params = cv.aruco.DetectorParameters_create()
        self.cap = cv.VideoCapture(0)

    def load_calibration(self):
        # Use the path specified in the calibration file
        calibration_data_path = 'calib_data/calibration.npz'  # Replace with your specified path
        print(f"Calibration file path: {calibration_data_path}")

        if not os.path.exists(calibration_data_path):
            print(f"Error: Calibration file '{calibration_data_path}' not found.")
            return False

        self.calibration_data = np.load(calibration_data_path)
        self.camera_matrix = self.calibration_data['camera_matrix']
        self.distortion_coefficients = self.calibration_data['distortion_coefficients']
        return True

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            
            undistorted_frame = cv.undistort(frame, self.camera_matrix, self.distortion_coefficients)
            img_gray = cv.cvtColor(undistorted_frame, cv.COLOR_BGR2GRAY)
            
            corners, ids, _ = cv.aruco.detectMarkers(img_gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
                    corners, 6, self.camera_matrix, self.distortion_coefficients
                )
                distances = [np.sqrt(tvecs[i][0][2] ** 2 + tvecs[i][0][0] ** 2 + tvecs[i][0][1] ** 2) for i in range(len(ids))]
                print(f"Marker IDs: {ids}, Distances: {distances}")

    def main(self):
        self.run()

if __name__ == '__main__':
    kamera = Kamera()
    kamera.main()
"""
