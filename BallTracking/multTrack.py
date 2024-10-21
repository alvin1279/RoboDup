import cv2
import numpy as np
import imutils
import math
from collections import defaultdict
import hsvMaskUtility as hlpr
from scipy.spatial import distance as dist
from centroidClass import CentroidTracker

previous_positions = {}  # Dictionary to store previous positions of objects
current_positions = {} 



# Function to calculate Euclidean distance between two points
def calculate_distance(point1, point2):
    return math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)




# Video capture and object tracking
vs = cv2.VideoCapture('Samples/vid1.mp4')
fps = vs.get(cv2.CAP_PROP_FPS)

# Initialize centroid tracker
ct = CentroidTracker()
# Variable to store previous positions and speeds of objects
previous_positions = defaultdict(lambda: None)
prev_centroid = defaultdict(lambda: None)
speeds = {}
# List to hold last N speed values for smoothing
last_speeds = defaultdict(lambda: [])


# Maximum number of speed values to consider for smoothing
SPEED_WINDOW_SIZE = 5
lower = (26, 42, 167)
upper = (179, 255, 255)

while True:
    ret, frame = vs.read()
    if not ret:
        break
    # Preprocess the frame
    frame = imutils.resize(frame, width=900)
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = hlpr.GetMask(hsvImage,lower,upper,3)
    # Find contours of objects
    cv2.imshow("mask", mask)
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(contours)

    # Extract centroids of each object
    inputCentroids = []
    for c in cnts:
        if cv2.contourArea(c) < 100:
            continue
        (x, y, w, h) = cv2.boundingRect(c)
        centroid = (int(x + w / 2), int(y + h / 2))
        # centroid = cv2.GaussianBlur(centroid, (5, 5), 0)
        inputCentroids.append(centroid)

    # Update tracker with the new centroids
    objects = ct.update(inputCentroids)
    delta_x, delta_y = 0, 0
    angle_degrees = 0
    # Calculate speed for each tracked object
    for objectID, centroid in objects.items():
        if previous_positions[objectID] is not None:
            displacement = calculate_distance(previous_positions[objectID], centroid)
            time_interval = 1 / fps
            speeds[objectID] = displacement / time_interval  # Speed in pixels/sec
            # speeds[objectID] = update_kalman(kalman,speeds[objectID])
            
            last_speeds[objectID].append(speeds[objectID])
            if len(last_speeds[objectID]) > SPEED_WINDOW_SIZE:
                last_speeds[objectID].pop(0)  # Keep only the latest N speeds

            # Calculate the moving average speed
            smoothed_speed = sum(last_speeds[objectID]) / len(last_speeds[objectID])
            if smoothed_speed < 30 :
                smoothed_speed = 0
            speeds[objectID] = smoothed_speed

            prev_centroid = previous_positions[objectID]
            delta_x = centroid[0] - prev_centroid[0]  # Change in X
            delta_y = centroid[1] - prev_centroid[1]  # Change in Y
        else:
            delta_x, delta_y = 0, 0  # Initial position, no movement yet

        if delta_x != 0 or delta_y != 0:  # Prevent division by zero
            angle = math.atan2(delta_y, delta_x)  # Angle in radians
            angle_degrees = math.degrees(angle)  # Convert to degrees
        else:
            angle_degrees = 0  # No movement, default to 0 degrees

        # Normalize the angle to [0, 360)
        if angle_degrees < 0:
            angle_degrees += 360
        
        # Draw bounding box and speed for each object
        for c in cnts:
            if previous_positions[objectID] is not None:
                prev_centroid = previous_positions[objectID]
            else:
                prev_centroid = centroid
                
            (x, y, w, h) = cv2.boundingRect(c)
            if (int(x + w / 2), int(y + h / 2)) == centroid:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.arrowedLine(frame, tuple(prev_centroid), tuple(centroid), (0, 255, 0), 2, tipLength=0.2)
                cv2.putText(frame, f"ID {objectID} Speed: {speeds.get(objectID, 0):.2f} px/sec",
                            (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Update previous position
        previous_positions[objectID] = centroid

    # Show frame
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

# Cleanup
vs.release()
cv2.destroyAllWindows()