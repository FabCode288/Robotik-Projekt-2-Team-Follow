import cv2
import numpy as np


cap = cv2.VideoCapture(0)

dist = 0
focal = 500
pix = 30
size = 4

# 
def get_dist(rectange_params,image):
    
    pix = rectange_params[1][0]
    print(pix)
    #calculate distance
    dist = (size*focal)/pix

kernel = np.ones((3,3),'uint8')



#cv2.namedWindow('Object Dist Measure ',cv2.WINDOW_NORMAL)
#cv2.resizeWindow('Object Dist Measure ', 700,600)


while True:
    success, img = cap.read() 
    

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
    
        if (cv2.contourArea(contour) > 100 and cv2.contourArea(contour) < 306000):
            
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

    
            img = get_dist(rect, img)

    cv2.imshow('Object Dist Measure', img)
    cv2.waitKey(0)


cv2.destroyAllWindows()
