import rclpy
import cv2 as cv
import numpy as np
import os
import time

from rclpy.node import Node
from std_msgs.msg import Float32

"""
Publisher class for the distance to the ahead robot and the distance to the white line
"""

class ArucoDistancePublisher(Node):

    def __init__(self):
        super().__init__('aruco_distance_publisher')
        self.publisher_distance_to_robot = self.create_publisher(Float32, 'aruco_distance', 10) #Creating a publsiher to publish the distance to 
        self.publisher_distance_to_line = self.create_publisher(Float32, 'line_distance', 10)
        self.cap = cv.VideoCapture(0)
        self.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_7X7_250)
        self.aruco_params = cv.aruco.DetectorParameters_create()
        self.camera_matrix = None 
        self.distortion_coefficients = None 

        timer_period = 0.2  # Publishes data with a frequency of 5Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Load calibration data and set camera matrix and distortion coefficients
        self.load_calibration_data()
        
    """
    Method loading the camera calibration file.
    """

    def load_calibration_data(self):
        calibration_data_path = '/home/ubuntu/ros2_ws_bot/src/kamera/calib_data/calibration.npz'
        if not os.path.exists(calibration_data_path):
            self.get_logger().warning(f"Error: Calibration file '{calibration_data_path}' not found.")
            return

        self.calibration_data = np.load(calibration_data_path)
        self.camera_matrix = self.calibration_data['camera_matrix']
        self.distortion_coefficients = self.calibration_data['distortion_coefficients']

    def timer_callback(self):
        ret, frame = self.cap.read()  # Capture frame from the camera
        time_stamp = time.time()
        if not ret:
            self.get_logger().warning('No frame received.')
            return

        distance_robot = self.calculate_distance_to_robot(frame) #Publishs the Distance to the Robot ahead, if an aruco marker is detected
        if distance_robot is not None:
            self.publish_distance_robot(distance_robot)

        distance_line = self.calculate_distance_to_line(frame)  #Publishs the Distance to the Line, if a line is detected
        if distance_line is not None:
            self.publish_distance_line(distance_line, time_stamp)

    """
    Method calculating the distance to the robot/aruco marker.
    If an aruco marker is detected the method returns the distance.
    If no aruco marler is detected the method returns an error value of -1.0 
    """

    def calculate_distance_to_robot(self, frame):
        img_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, _, _ = cv.aruco.detectMarkers(img_gray, self.aruco_dict, parameters=self.aruco_params)

        if corners is not None and len(corners) > 0:
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
                corners, 7, self.camera_matrix, self.distortion_coefficients
            )
            distance = np.sqrt(tvecs[0][0][2] ** 2 + tvecs[0][0][0] ** 2 + tvecs[0][0][1] ** 2)
            return distance/100
        else:
            return -1.0
        
    """
    Method calculating the distance to the white line in pixel for every frame.
    Depending on the position of the line in the frame the Method returns a positiv or a negative value.
    If the line is on the left ahnd side if the robot the value is negativ.
    If the line is on right hand side of the robot the value is positiv.
    If no line is detected the method returns an error value of 66666.
    """
        
    def calculate_distance_to_line(self, image):

        h, w = image.shape[:2]
        cropped_image = image[h//2:, :]

        # Convert the image into Grayscale
        gray = cv.cvtColor(cropped_image, cv.COLOR_BGR2GRAY)
        gray = cv.medianBlur(gray, 5)
        gray = cv.GaussianBlur(gray, (7,7), 3)

        # Thresholds for the white line
        lower_white = 120
        upper_white = 255

        # Extract the white line from the image
        mask = cv.inRange(gray, lower_white, upper_white)
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Choose the biggest contour
            largest_contour = max(contours, key=cv.contourArea)

        # Find the edges of the contour        
            edges = cv.Canny(gray, 20, 700, apertureSize=5)           

        # Find lines
            lines = cv.HoughLines(edges, 1, np.pi / 180, 190)

         # Check if line were found, exclude horizontal lines
            linecheck = False
            if lines is not None:
                linecheck = True

            M = cv.moments(largest_contour)
            
            if(M["m00"] != 0 and linecheck):
                centroid_x = int(M["m10"] / M["m00"])

                return centroid_x - image.shape[1] // 2 + 30
            else:
                return 66666
        else:
            return 66666

    def publish_distance_robot(self, distance_robot):
        msg_robot = Float32()
        msg_robot.data = float(distance_robot)
        self.publisher_distance_to_robot.publish(msg_robot)
        self.get_logger().info('Publishing distance to robot: "%s"' % msg_robot.data)
        
    def publish_distance_line(self, distance_line, time_stamp):
        msg_line = Float32()
        msg_line.data = [float(distance_line), float(time_stamp)]
        self.publisher_distance_to_line.publish(msg_line)
        self.get_logger().info('Publishing distance to line: "%s"' % msg_line.data)


def main(args=None):
    rclpy.init(args=args)
    aruco_distance_publisher = ArucoDistancePublisher()
    try:
        rclpy.spin(aruco_distance_publisher)
    except KeyboardInterrupt:
        pass
    aruco_distance_publisher.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
