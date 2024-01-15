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
        calibration_data_path = '/home/ubuntu/kamera/calib_data/calibration.npz'
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

        distance_robot = self.calculate_distance_to_robot(frame)
        if distance_robot is not None:
            self.publish_distance_robot(distance_robot)

        distance_line = self.calculate_distance_to_line(frame)
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
            return distance
        else:
            return -1.0

    def calculate_distance_to_line(self, frame):
    
        # Konvertiere das Bild in den HSV-Farbraum
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        # Definiere den Farbbereich der weißen Linie
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([255, 50, 255])
        
        # Extrahiere die weiße Linie im Bild
        mask = cv.inRange(hsv, lower_white, upper_white)
        
        # Finde Konturen im Bild
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            # Nehme die größte Kontur (Annahme: Die größte Kontur ist die Linie)
            largest_contour = max(contours, key=cv.contourArea)
            
            # Berechne den Mittelpunkt der Linie
            M = cv.moments(largest_contour)
            try:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                distance_line = cx - (frame.shape[1] // 2)

            except:
                pass
            
            # Berechne die Distanz vom Mittelpunkt des Bildes zur Linie
            
            
            # Zeichne den Mittelpunkt und die Linie auf das Bild
            #cv.circle(frame, (cx, cy), 5, (0, 255, 0), -1)  # Mittelpunkt in grün zeichnen
            #cv.line(frame, (frame.shape[1] // 2, 0), (frame.shape[1] // 2, frame.shape[0]), (0, 0, 255), 2)  # Linie in rot zeichnen

            # Zeige das Bild an
            #cv.imshow('Detected Line', frame)
            #cv.waitKey(0)
            #cv2.destroyAllWindows()
            
            return distance_line
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
