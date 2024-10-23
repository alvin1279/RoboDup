import cv2
import numpy as np
import imutils
from collections import defaultdict
import hsvMaskUtility as hlpr
from scipy.spatial import distance as dist
from centroidClass import CentroidTracker
import path_finder as pf


# HSV range for mask
lower = (26, 42, 167)
upper = (179, 255, 255)

# temperory start and end points
start = (100, 100)  # Example start point, adjust as per your image
end = (500, 400) 
vs = cv2.VideoCapture('Samples/vid1.mp4')
if not vs.isOpened():
    raise IOError("Cannot open video file")

fps = vs.get(cv2.CAP_PROP_FPS)
time_interval = 1 / fps
path_image = np.zeros((500, 900, 3), np.uint8)
delay_between_frames = 50
# Initialize centroid tracker
ct = CentroidTracker()

# Function to draw tracking information on the frame (parallelized version)
def draw_tracking_info(frame, blank_image, cnts, centroid, objectID, objcVector, speed):
    for c in cnts:
        (x, y, w, h) = cv2.boundingRect(c)
        if (int(x + w / 2), int(y + h / 2)) == centroid:
            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)


            # Extend arrow endpoint based on angle
            # Get the x and y components of the vector
            vx, vy = objcVector  # The movement vector
            scale_factor = 10

            # Extend the line from the centroid using the vector
            xext = int(centroid[0] + vx)
            yext = int(centroid[1] + vy)

            # Draw arrow for movement direction
            cv2.arrowedLine(frame, centroid, (xext, yext), (0, 255, 0), 2, tipLength=0.2)

            # Display object ID and speed
            cv2.putText(frame, f"ID {objectID} Speed: {speed:.2f} px/sec", 
                        (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Draw all this in blank image
            cv2.rectangle(blank_image, (x, y), (x + w, y + h), (255, 255, 255), -1)
            cv2.arrowedLine(blank_image, centroid, (xext, yext), (0, 255, 0), 2, tipLength=0.2)
            cv2.putText(blank_image, f"ID {objectID} Speed: {speed:.2f} px/sec", 
                        (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
# Function to process all tracked objects in parallel
def process_tracked_objects(frame, blank_image, cnts, ct, time_interval):

    for objectID, obj in ct.objects.items():
        centroid = obj.centroid
        object_vector = obj.vector
        displacement = obj.displacement
        current_speed = 0
        
        if len(obj.centroid_history) > 1:            
            current_speed = displacement / time_interval
        else:
            current_speed = 0

        # Submit each task to the executor
        draw_tracking_info(frame, blank_image, cnts, centroid, objectID, object_vector, current_speed)
        
def draw_path(bitwise_not, blank_image, path_points, start, end):
    grey_color = (25, 25, 25)  # Grey color for the lines
    red = (0, 0, 255)  # Red color for the lines

    start_point = start  # Use the provided start point directly

    for path in path_points:
        # Extract new start and end points from path points
        new_start = path[0]
        new_end = path[1]
        
        # Draw line from start point to new start
        cv2.line(bitwise_not, start_point, new_start, grey_color, 2)
        cv2.line(blank_image, start_point, new_start, red, 2)
        
        # Update start_point to be new_end for the next iteration
        start_point = new_end

    # After the loop, connect the last end point to the final goal
    end_point = end
    cv2.line(bitwise_not, start_point, end_point, grey_color, 2)
    cv2.line(blank_image, start_point, end_point, red, 2)

# Main loop for video processing
while True:
    ret, frame = vs.read()
    if not ret:
        break

    # Preprocess the frame
    frame = imutils.resize(frame, width=900)
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create blank image same as frame
    blank_image = np.zeros((frame.shape[0], frame.shape[1], 3), np.uint8)

    # Get mask for object detection
    mask = hlpr.GetMask(hsvImage, lower, upper, 3)
    bitwise_not = cv2.bitwise_not(mask)

    # Find contours
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(contours)

    # Extract centroids from contours
    inputCentroids = []
    for c in cnts:
        if cv2.contourArea(c) < 100:
            continue
        (x, y, w, h) = cv2.boundingRect(c)
        centroid = (int(x + w / 2), int(y + h / 2))
        inputCentroids.append(centroid)

    # Update the tracker with new centroids
    objects = ct.update(inputCentroids)
    process_tracked_objects(frame,blank_image,cnts,ct,time_interval)


    # get path points to target object
    path_points = pf.get_paths(cnts, start, end)
    draw_path(bitwise_not, blank_image, path_points,start, end)

    cv2.imshow('bitwise_not',bitwise_not)
    cv2.imshow("blank_image", blank_image)
    # reduce nummber to 1 to go back to normal frame rate
    key = cv2.waitKey(200) & 0xFF
    if key == ord("q"):
        break

# Cleanup
vs.release()
cv2.destroyAllWindows()