import cv2
import numpy as np
import hsvMaskUtility as hlpr
import imutils
import random as rng

img = cv2.imread('Samples/resized_rotated_frame.png')
img = imutils.resize(img, width=900)
hsvImage = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

lower = (26, 42, 167)
upper = (179, 255, 255)
# uncomment code to open window to get mask boundaries

lower,upper = hlpr.getMaskBoundary(img)
print(lower)
print(upper)
# '''

mask = hlpr.GetMask(hsvImage, lower, upper,3)


contours = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(contours)


contours_poly = [None]*len(contours)
boundRect = [None]*len(contours)
centers = [None]*len(contours)
radius = [None]*len(contours)

for i, c in enumerate(cnts):
    contours_poly[i] = cv2.approxPolyDP(c, 3, True)
    boundRect[i] = cv2.boundingRect(contours_poly[i])
    centers[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])


drawing = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)

for i in range(len(cnts)):
    color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
    rectColor = (255,255,255)
    cv2.rectangle(drawing, (int(boundRect[i][0]), int(boundRect[i][1])), 
        (int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])), rectColor, 2)
    cv2.circle(drawing, (int(centers[i][0]), int(centers[i][1])), int(radius[i]), color, 2)

cv2.imshow('drawing', drawing)
cv2.imshow("img", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
