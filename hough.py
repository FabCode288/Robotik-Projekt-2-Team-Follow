import cv2
import numpy as np

def find_parallel_lines(image_path, angular_tolerance=10):

   

    img = cv2.imread(image_path)
    

    h, w = img.shape[:2]
    cropped = img[h//2:, :]
    
    hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
        
    # Definiere den Farbbereich der weißen Linie
    lower_white = np.array([0, 0, 120])
    upper_white = np.array([255, 50, 255])
    
    # Extrahiere die weiße Linie im Bild
    mask = cv2.inRange(hsv, lower_white, upper_white)
    
    # Finde Konturen im Bild
    
    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #filter = cv2.medianBlur(gray,5)
    

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
            # Nehme die größte Kontur (Annahme: Die größte Kontur ist die Linie)
            largest_contour = max(contours, key=cv2.contourArea)
            # Berechne den Mittelpunkt der Linie
            M = cv2.moments(largest_contour)
            try:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                distance_to_line = cx - (img.shape[1] // 2)
                print( distance_to_line)
            except:
                pass
    else:
            return 66666   
             
    black = np.zeros_like(mask)

    cv2.drawContours(black, [largest_contour], -1, (255, 255, 255), thickness=cv2.FILLED)
            
    edges = cv2.Canny(black, 20, 700, apertureSize=5)

    kernel = np.ones((3,3), np.uint8)
    dilated_edges = cv2.dilate(edges, kernel, iterations=1)

    lines = cv2.HoughLines(dilated_edges, 1, np.pi / 180, 180)
    #print(lines)

    if lines is not None:
        for line in lines:
            rho, theta = line[0]
            if abs(theta) < np.pi/3:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 2000 * (-b))
                y1 = int(y0 + 2000 * (a))
                x2 = int(x0 - 2000 * (-b))
                y2 = int(y0 - 2000 * (a))
                cv2.line(cropped, (x1, y1), (x2, y2), (0,255,0), 2)
                x = (rho - cy * np.sin(theta)) / np.cos(theta)
                #print(f"x-coordinate at y={20}: {x}")
                distance_line = x - (img.shape[1] // 2)
                print(distance_line)
            
       
 

    cv2.imshow('Lines', cropped)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Usage
find_parallel_lines("D:/Uni/proj2/testbilder/bot1.jpg")
  