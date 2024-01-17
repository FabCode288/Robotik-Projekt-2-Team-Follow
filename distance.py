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

    def calculate_distance_to_line(self, frame):
       
        h, w = frame.shape[:2]
        frame = frame[h//2:, :]

    # Wandle das Bild in ein Binärbild um
        frame_binary = self.convert_to_binary(frame)

        # Führe Erosion und Dilatation durch, um kleine Lücken oder Störungen zu schließen
        kernel = np.ones((5, 5), np.uint8)
        frame_binary = cv.erode(frame_binary, kernel, iterations=3)
        frame_binary = cv.dilate(frame_binary, kernel, iterations=3)
        
        # Finde Konturen im Bild
        contours, _ = cv.findContours(frame_binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            # Nehme die größte Kontur (Annahme: Die größte Kontur ist die Linie)
            largest_contour = max(contours, key=cv.contourArea)
            
            # Berechne die Momente der Kontur
            moments = cv.moments(largest_contour)
            
            if moments["m00"] != 0:
                # Berechne den Schwerpunkt der Kontur
                cx = int(moments["m10"] / moments["m00"])
                cy = int(moments["m01"] / moments["m00"])
                return (cx - (w // 2))  # Abstand vom Mittelpunkt des Bildes zu 5% vom unteren Rand der erkannten Linie
            
            else:
                return 66666
         
    def find_line_HSV(self):
        height, width, _ = frame.shape
        frame = frame[height//3:, :]

        
       # Konvertiere das Bild in den HSV-Farbraum
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        # Extrahiere den S-Kanal im HSV-Farbraum
        s_channel = hsv[:, :, 1]
        
        # Definiere den optimierten Farbbereich der weißen Linie basierend auf dem S-Kanal
        lower_white = np.array([0, 0, 120])
        upper_white = np.array([180, 60, 255])
        
        # Extrahiere die weiße Linie im Bild
        mask = cv.inRange(hsv, lower_white, upper_white)
        mask = cv.bitwise_and(mask, s_channel)  # Nutze den S-Kanal zur Verbesserung der Farbsegmentierung
        
        # Führe Erosion und Dilatation durch, um kleine Lücken oder Störungen zu schließen
        kernel = np.ones((5,5),np.uint8)
        mask = cv.erode(mask, kernel, iterations=0)
        mask = cv.dilate(mask, kernel, iterations=0)
        
        # Finde Konturen im Bild
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            # Nehme die größte Kontur (Annahme: Die größte Kontur ist die Linie)
            largest_contour = max(contours, key=cv.contourArea)
            
            # Extrahiere die äußersten Punkte der Kontur
            ext_left = tuple(largest_contour[largest_contour[:, :, 0].argmin()][0])
            ext_right = tuple(largest_contour[largest_contour[:, :, 0].argmax()][0])

            # Berechne den Mittelpunkt der Linie auf der y-Höhe von 1% über dem unteren Rand der Linie
            distance_from_bottom = int((ext_left[1] + ext_right[1]) * 0.01)  # 1% vom unteren Rand der erkannten Linie
            line_height = max(ext_left[1], ext_right[1]) - distance_from_bottom  # Höhe der unteren 1 %
            cx = (ext_left[0] + ext_right[0]) // 2

            M = cv.moments(largest_contour)
            try:
                centroid_x = int(M["m10"] / M["m00"])
                centroid_y = int(M["m01"] / M["m00"])
            except:
                return None
            #cv.circle(frame, (centroid_x, centroid_y), 10, (23, 255, 255), -1)  # Mittelpunkt 5% vom unteren Rand der Linie in grün zeichnen


            # Zeichne den Mittelpunkt und die Linie auf das Bild
            #cv.circle(frame, (cx, line_height), 10, (0, 255, 0), -1)  # Mittelpunkt 5% vom unteren Rand der Linie in grün zeichnen
            #cv.line(frame, (width // 2, line_height), (width // 2, max(ext_left[1], ext_right[1])), (0, 0, 255), 5)  # Linie in rot zeichnen

            # Zeichne die Kontur des erkannten weißen Streifens
            #cv.drawContours(frame, [largest_contour], -1, (255, 0, 0), 5)

            # Zeichne eine gestrichelte Linie senkrecht in die Mitte des Bildes
            #cv.line(frame, (width // 2, 0), (width // 2, height), (255, 255, 255), 1, cv.LINE_AA)

            # Skaliere das Bild auf 20% der Originalgröße
            #scaled_frame = cv.resize(frame, (int(width * 0.7), int(height * 0.4)))

            # Zeige das skalierte Bild an
            #cv.imshow('Detected Line', scaled_frame)
            #cv.waitKey(0)
            #cv.destroyAllWindows()
        
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

    def convert_to_binary(self, image):
    # Konvertiere das Bild in Graustufen
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        
        # Wende Schwellenwert an, um ein Binärbild zu erstellen
        _, binary_image = cv.threshold(gray, 127, 255, cv.THRESH_BINARY)
        
        return binary_image

def main(args=None):
    rclpy.init(args=args)
    aruco_distance_publisher = ArucoDistancePublisher()
    rclpy.spin(aruco_distance_publisher)
    aruco_distance_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()