import cv2
import numpy as np
import imutils
import hsvMaskUtility as hlpr

def findHeadAndTail(img):
    # bot is segmented to tail and head
    lower_tail = (110,136,193)
    upper_tail = (129,184,255)
    # lower,upper = hlpr.getMaskBoundary(img)
    # print(f"Lower: {lower}, Upper: {upper}")
    # lower_tail = np.array(lower)
    # upper_tail = np.array(upper)

    lower_head = (159,139,226)
    upper_head = (179,215,255)
    # lower,upper = hlpr.getMaskBoundary(img)
    # print(f"Lower: {lower}, Upper: {upper}")
    # lower_head = np.array(lower)
    # upper_head = np.array(upper)

    HsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    tail_mask = cv2.inRange(HsvImg, lower_tail, upper_tail)
    head_mask = cv2.inRange(HsvImg, lower_head, upper_head)
    tail_mask = cv2.erode(tail_mask, None, iterations=1)
    tail_mask = cv2.dilate(tail_mask, None, iterations=4)
    head_mask = cv2.erode(head_mask, None, iterations=1)
    head_mask = cv2.dilate(head_mask, None, iterations=4)
    # cv2.imshow('Tail Mask', tail_mask)
    # if cv2.waitKey(0) & 0xFF == ord('q'):
    #     cv2.destroyAllWindows()
    # cv2.imshow('Head Mask', head_mask)
    # if cv2.waitKey(0) & 0xFF == ord('q'):
    #     cv2.destroyAllWindows()
    contours_tail, _ = cv2.findContours(tail_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_head, _ = cv2.findContours(head_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Check if any contours were found for tail
    if not contours_tail:
        raise ValueError("No contours found for tail")

    # find largest contour in tail
    max_area = 0
    max_area_index = 0
    for i, c in enumerate(contours_tail):
        area = cv2.contourArea(c)
        if area > max_area:
            max_area = area
            max_area_index = i
    # Calculate centroid of tail
    M = cv2.moments(contours_tail[max_area_index])
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        cX, cY = 0, 0
    tail_centroid = (cX, cY)

    # Check if any contours were found for head
    if not contours_head:
        raise ValueError("No contours found for head")

    # find largest contour in head
    max_area = 0
    max_area_index = 0
    for i, c in enumerate(contours_head):
        area = cv2.contourArea(c)
        if area > max_area:
            max_area = area
            max_area_index = i
    # Calculate centroid of head
    M = cv2.moments(contours_head[max_area_index])
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        cX, cY = 0, 0
    head_centroid = (cX, cY)

    return tail_centroid, head_centroid

def findOrientation(tail_centroid, head_centroid):
    # Calculate orientation of the pattern using the centroid
    # Get the centroid of the largest area contour
    cX, cY = tail_centroid
    # get average angle from all other centroids to the centroid of the largest area contour
    angles = []
    x, y = head_centroid
    angle = np.arctan2(cY - y, cX - x) * 180 / np.pi
    angles.append(angle)
    avg_angle = np.mean(angles)
    # convert angle to 0-360
    if avg_angle < 0:
        avg_angle += 360
    return avg_angle

def drawOrientation(img, tail_centroid, head_centroid):
    cv2.arrowedLine(img, tail_centroid, head_centroid, (0, 255, 0), 2, tipLength=0.2)

def getBotData(img):
    try:
        tail_centroid, head_centroid = findHeadAndTail(img)
    except ValueError as e:
        print(f"Error in finding head or tail: {e}")
        return None, None, None

    try:
        orientation = findOrientation(tail_centroid, head_centroid)
    except Exception as e:
        print(f"Error in finding orientation: {e}")
        return tail_centroid, head_centroid, None

    return tail_centroid, head_centroid, orientation

def main(img):
    # img = cv2.imread('Samples/bot.jpg')
    # img = imutils.resize(img, width=600)
    tail_centroid, head_centroid, orientation = getBotData(img)
    if tail_centroid is None or head_centroid is None:
        print("Error: Could not find tail or head centroids.")
        return
    center = ((tail_centroid[0] + head_centroid[0]) // 2, (tail_centroid[1] + head_centroid[1]) // 2)
    drawOrientation(img, tail_centroid, head_centroid)
    cv2.circle(img, center, 1, (0, 255, 0), -1)
    cv2.circle(img, tail_centroid, 1, (0, 0, 255), -1)
    cv2.circle(img, head_centroid, 1, (255, 0, 0), -1)
    print(f"orientation: {orientation}")
    cv2.imshow('Bot Orientation', img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

if __name__ == "__main__":
    # main()
    vs = cv2.VideoCapture('http://192.168.149.102:8080/video')
    if not vs.isOpened():
        raise IOError("Cannot open video file")
    while True:
        ret, frame = vs.read()
        if not ret:
            break
        main(frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    vs.release()
    cv2.destroyAllWindows()