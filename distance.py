import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2 as cv
import numpy as np
import os

class ArucoDistancePublisher(Node):

    def __init__(self):
        super().__init__('aruco_distance_publisher')
        self.publisher_distance_to_robot = self.create_publisher(Float32, 'aruco_distance', 10)
        self.publisher_distance_to_line = self.create_publisher(Float32, 'line_distance', 10)
        self.cap = cv.VideoCapture(0)
        self.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_7X7_250)
        self.aruco_params = cv.aruco.DetectorParameters_create()
        self.camera_matrix = None  # Placeholder for camera matrix
        self.distortion_coefficients = None  # Placeholder for distortion coefficients

        timer_period = 0.2  # Publishes data every 0.2 seconds (5Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Load calibration data and set camera matrix and distortion coefficients
        self.load_calibration_data()

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
        if not ret:
            self.get_logger().warning('No frame received.')
            return

        distance_robot = self.calculate_distance_to_robot(frame) #Publishs the Distance to the Robot ahead, if an aruco marker is detected
        if distance_robot is not None:
            self.publish_distance_robot(distance_robot)

        distance_line = self.calculate_distance_to_line(frame)  #Publishs the Distance to the Line, if a line is detected
        if distance_line is not None:
            self.publish_distance_line(distance_line)

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
        
    def calculate_distance_to_line(self, image):

        h, w = image.shape[:2]
        cropped_image = image[h//2:, :]

        # Convert the image into the HSV Space
        hsv = cv.cvtColor(cropped_image, cv.COLOR_BGR2HSV)
        
        # Thresholds for the white line
        lower_white = np.array([0, 0, 120])
        upper_white = np.array([255, 50, 255])
        
        # Extract the white line from the image
        mask = cv.inRange(hsv, lower_white, upper_white)
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Choose the biggest contour
            largest_contour = max(contours, key=cv.contourArea)

            # Draw the largest contour onto a black canvas
            black = np.zeros_like(mask)
            cv.drawContours(black, [largest_contour], -1, (255, 255, 255), thickness=cv.FILLED)

            # Find the edges of the contour        
            edges = cv.Canny(black, 20, 700, apertureSize=5)

            # Dilate the edges
            kernel = np.ones((3,3), np.uint8)
            dilated_edges = cv.dilate(edges, kernel, iterations=1)

            # Find lines
            lines = cv.HoughLines(dilated_edges, 1, np.pi / 180, 180)

            # Check if line were found, exclude horizontal lines
            linecheck = False
            if lines is not None:
                for line in lines:
                    rho, theta = line[0]
                    if abs(theta) < np.pi/3:
                        linecheck = True

            # Calculate the centroid of the contour
            M = cv.moments(largest_contour)
            if(M["m00"] != 0 and linecheck):
                centroid_x = int(M["m10"] / M["m00"])
                return centroid_x - image.shape[1] // 2
            else:
                return 66666

        else:
            return 66666
        
    def publish_distance_robot(self, distance_robot):
        msg_robot = Float32()
        msg_robot.data = float(distance_robot)
        self.publisher_distance_to_robot.publish(msg_robot)
        self.get_logger().info('Publishing distance to robot: "%s"' % msg_robot.data)
        
    def publish_distance_line(self, distance_line):
        msg_line = Float32()
        msg_line.data = float(distance_line)
        self.publisher_distance_to_line.publish(msg_line)
        self.get_logger().info('Publishing distance to line: "%s"' % msg_line.data)

def main(args=None):
    rclpy.init(args=args)
    aruco_distance_publisher = ArucoDistancePublisher()
    rclpy.spin(aruco_distance_publisher)
    aruco_distance_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
