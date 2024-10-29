import cv2
import numpy as np
import imutils

def find_pattern_orrientation(contours):
    # max area contour index
    max_area_index = 0
    max_area = 0
    index_to_centroid = {}
    area = cv2.contourArea(contours[0])
    print(f"Area: {area}")

    if(len(contours) == 0):
        return 0
    for i, c in enumerate(contours):
        area = cv2.contourArea(c)
        if area > max_area:
            max_area = area
            max_area_index = i
        
        # Calculate centroid
        M = cv2.moments(c)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0
        
        index_to_centroid[i] = (cX, cY)
    # draw arrows from all other centroids to the centroid of the largest area contour
    for i, c in index_to_centroid.items():
        if i == max_area_index:
            continue
        x, y = c
        cv2.arrowedLine(img, (x, y), index_to_centroid[max_area_index], (0, 255, 0), 2, tipLength=0.2)
    # Calculate orientation of the pattern using the centroid
    # Get the centroid of the largest area contour
    cX, cY = index_to_centroid[max_area_index]
    print(f"Centroid: {cX}, {cY}")
    # get average angle from all other centroids to the centroid of the largest area contour
    angles = []
    for i, c in index_to_centroid.items():
        if i == max_area_index:
            continue
        x, y = c
        angle = np.arctan2(cY - y, cX - x) * 180 / np.pi
        angles.append(angle)
    print(f"Angles: {angles}")
    avg_angle = np.mean(angles)
    # convert angle to 0-360
    if avg_angle < 0:
        avg_angle += 360
    print(f"Average Angle: {avg_angle}")


    



img = cv2.imread('Samples/pattern1.png')
img = imutils.rotate(img, 45)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# threshold the image for black color
mask = cv2.inRange(gray, 100, 255)
# flip the mask horizontaly
# mask = cv2.bitwise_not(mask)
cv2.imshow("mask", mask)
# contours= cv2.threshold(mask, 100, 255, cv2.THRESH_BINARY)
contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contour = imutils.grab_contours(contours)
find_pattern_orrientation(contour)
cv2.drawContours(img, contour, -1, (0, 255, 0), 2)
cv2.imshow("img", img)
cv2.waitKey(0)
cv2.destroyAllWindows()