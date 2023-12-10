import cv2
import numpy as np
import os

class Calibration:
    def __init__(self, image_dir_path="images", calib_data_path="../calib_data"):
        self.CHESS_BOARD_DIM = (9, 6)
        self.SQUARE_SIZE = 25
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.image_dir_path = image_dir_path
        self.calib_data_path = calib_data_path

    def perform_calibration(self):
        CHECK_DIR = os.path.isdir(self.calib_data_path)

        if not CHECK_DIR:
            os.makedirs(self.calib_data_path)
            print("Directory created")
        else:
            print("Directory exists")

        obj_3d = np.zeros((self.CHESS_BOARD_DIM[0]*self.CHESS_BOARD_DIM[1], 3), np.float32)
        obj_3d[:, :2] = np.mgrid[0:self.CHESS_BOARD_DIM[0], 0:self.CHESS_BOARD_DIM[1]].T.reshape(-1, 2)
        obj_3d *= self.SQUARE_SIZE

        obj_points_3d = []
        img_points_2d = []

        files = os.listdir(self.image_dir_path)
        for file in files:
            imagePath = os.path.join(self.image_dir_path, file)

            image = cv2.imread(imagePath)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, self.CHESS_BOARD_DIM, None)
            if ret:
                obj_points_3d.append(obj_3d)
                corners2 = cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), self.criteria)
                img_points_2d.append(corners2)

                img = cv2.drawChessboardCorners(image, self.CHESS_BOARD_DIM, corners2, ret)
                cv2.imshow('Corners', img)
                cv2.waitKey(500)
        
        cv2.destroyAllWindows()

        ret, camera_matrix, distortion_coeff, rvecs, tvecs = cv2.calibrateCamera(
            obj_points_3d, img_points_2d, gray.shape[::-1], None, None
        )
        print("Calibrated")
        np.savez(
            f"{self.calib_data_path}/calibration.npz",
            camera_matrix=camera_matrix,
            distortion_coefficients=distortion_coeff,
            rvecs=rvecs,
            tvecs=tvecs,
        )
        data = np.load(f"{self.calib_data_path}/calibration.npz")
        cam_matrix = data["camera_matrix"]
        dist_coef = data["distortion_coefficients"]
        r_vector = data["rvecs"]
        t_vector = data["tvecs"]

        print("Loaded calibration data successfully")

# Usage:
calibration = Calibration()
calibration.perform_calibration()
