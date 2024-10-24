# helper 
import cv2
import numpy as np
import imutils
  
def GetMask(hsv, lower_color, upper_color,filter_radius):
    """
    Returns the contours generated from the given color range
    """
    
    # Threshold the HSV image to get only cloth colors
    mask = cv2.inRange(hsv, lower_color, upper_color)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=4)
    
    #use a median filter to get rid of speckle noise
    median = cv2.medianBlur(mask,filter_radius)

    return median


def nothing(x): 
    pass
def getMaskBoundary(img):
    # Creating a window with black image 
    cv2.namedWindow('image') 
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = []
    upper = []

    # creating trackbars for color change 
    cv2.createTrackbar('Hue Min', 'image', 0, 179, nothing) 
    cv2.createTrackbar('Hue Max', 'image', 179, 179, nothing) 
    
    cv2.createTrackbar('Sat Min', 'image', 0, 255, nothing) 
    cv2.createTrackbar('Sat Max', 'image', 255, 255, nothing) 
     
    cv2.createTrackbar('Val Min', 'image', 0, 255, nothing) 
    cv2.createTrackbar('Val Max', 'image', 255, 255, nothing) 

    
    while(True): 
        # for button pressing and changing 
        k = cv2.waitKey(1) & 0xFF
        if k == 27: 
            break
    
        # get current positions of all Three trackbars 
        hmn = cv2.getTrackbarPos('Hue Min', 'image') 
        hmx = cv2.getTrackbarPos('Hue Max', 'image')
        smin = cv2.getTrackbarPos('Sat Min', 'image') 
        smx = cv2.getTrackbarPos('Sat Max', 'image')
        vmn = cv2.getTrackbarPos('Val Min', 'image') 
        vmx = cv2.getTrackbarPos('Val Max', 'image') 
    
        # display color mixture 
        lower = np.array([hmn, smin, vmn])
        upper = np.array([hmx, smx, vmx])
        imgMASK = cv2.inRange(imgHSV, lower, upper)
        cv2.imshow('mask', imgMASK) 
    
    # close the window
    cv2.destroyAllWindows()
    return lower,upper