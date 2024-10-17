import cv2
import numpy as np
import imutils
import hsvMaskUtility as hlpr
import random as rng
import time
import argparse

'''
python ballDetectionFromVideo.py  --video Samples\filename  
'''
lower = (26, 42, 167)
upper = (179, 255, 255)
color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
rectColor = (255,255,255)

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())

vs = cv2.VideoCapture(args["video"])
# allow the camera or video file to warm up
time.sleep(2.0)
# keep looping
while True:
    # grab the current frame
    frame = vs.read()
    # handle the frame from VideoCapture or VideoStream
    frame = frame[1] if args.get("video", False) else frame
    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
    if frame is None:
        break
    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(frame, width=600)
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = hlpr.GetMask(hsvImage, lower, upper,3)

    contours = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(contours)

    contours_poly = [None]*len(cnts)
    boundRect = [None]*len(cnts)
    centers = [None]*len(cnts)
    radius = [None]*len(cnts)

    if len(cnts)>0:
        for i, c in enumerate(cnts):
            contours_poly[i] = cv2.approxPolyDP(c, 3, True)
            boundRect[i] = cv2.boundingRect(contours_poly[i])
            centers[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])


    drawing = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)

    for i in range(len(cnts)):
        # cv2.drawContours(drawing, contours_poly, i, color)
        cv2.rectangle(drawing, (int(boundRect[i][0]), int(boundRect[i][1])), 
            (int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])), rectColor, 2)
        cv2.circle(drawing, (int(centers[i][0]), int(centers[i][1])), int(radius[i]), color, 2)
	# if the 'q' key is pressed, stop the loop
    # cv2.drawContours(drawing, cnts, -1, (0, 255, 0), 3)
    cv2.imshow('drawing', drawing)
    cv2.imshow("frame", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cv2.waitKey(0)
cv2.destroyAllWindows()