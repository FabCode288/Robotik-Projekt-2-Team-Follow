import cv2
import os

class ImageStorage:
    def __init__(self):
        self.CHESS_BOARD_DIM = (9, 6)
        self.n = 0  # Number of images taken
        self.max_images = 10  # Maximum number of images to capture
        self.image_dir_path = "images"

        CHECK_DIR = os.path.isdir(self.image_dir_path)
        if not CHECK_DIR:
            os.makedirs(self.image_dir_path)
            print("Directory created")
        else:
            print("Directory exists")

        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)  # Adjust the epsilon value

        self.cap = cv2.VideoCapture(0)

    def save_image(self):
        ret, frame = self.cap.read()
        if not ret:
            return None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.CHESS_BOARD_DIM)

        if ret:
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
            if corners_refined is not None:
                cv2.imwrite(f"{self.image_dir_path}/image{self.n}.png", frame)
                print(f"Saved image number {self.n}")
                self.n += 1
            else:
                print("Corner refinement failed.")
        else:
            print("No corners detected.")

    def start_capture(self):
        while self.n < self.max_images:
            self.save_image()

        self.cap.release()

storage = ImageStorage()
storage.start_capture()
