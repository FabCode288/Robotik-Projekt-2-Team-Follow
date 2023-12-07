import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node

class Kamera(Node):

    def __init__(self):

        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.calibration_data = np.load('calibration.npz') #muss noch erstellt werden
        self.camera_matrix = self.calibration_data['camera_matrix']
        self.distortion_coefficients = self.calibration_data['distortion_coefficients']

        # Create an ArUco dictionary -> anpassen
        self.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)

        # Create the ArUco parameters -> anpassen
        self.aruco_params = cv.aruco.DetectorParameters_create()


        # Open a connection to the camera 
        self.cap = cv.VideoCapture(0)  # Use 0 for /dev/video0

        self.abstand = None

    def run(self):        
        # Capture a frame
        ret, frame = self.cap.read()
        # Undistort the frame
        undistorted_frame = cv.undistort(frame, self.camera_matrix, self.distortion_coefficients)
        img_gray = cv.cvtColor(undistorted_frame, cv.COLOR_BGR2GRAY)
        # Detect ArUco markers in the undistorted frame
        corners, ids = cv.aruco.detectMarkers(img_gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            #aruco detected, abstand berechnen
            abstand = 20
            return float(abstand)
        else:
            return None

    
    def timer_callback(self):
         self.publisher_.publish(self.run())

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Kamera()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
       