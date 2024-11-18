import json
import cv2
import imutils
import hsvMaskUtility as hlpr
# HSV range for ball mask
# jasira camera values
# lower = (22, 36, 218)
# upper = (50, 255, 255)
lower = (22, 36, 218)
upper = (50, 255, 255)
time_interval = 1
grey_color = (25, 25, 25)  # Grey color for the lines
red = (0, 0, 255)  # Red color for the lines

def load_frame_data():
    with open('Datas/final_warped.json', 'r') as json_file:
        json_data = json.load(json_file)
        transformed_left_goal_post = json_data['transformed_left_goal_post']
        transformed_right_goal_post = json_data['transformed_right_goal_post']
        redux = json_data['redux']
        warp_matrix = json_data['warp_matrix']
        shape = tuple(map(int, json_data['shape']))
        width = json_data['width']
        height = json_data['height']
    return transformed_left_goal_post, transformed_right_goal_post, redux, warp_matrix, shape, width, height

def load_video_stream(source):
    vs = cv2.VideoCapture(source)
    if not vs.isOpened():
        raise IOError("Cannot open video file")
    fps = vs.get(cv2.CAP_PROP_FPS)
    time_interval = 1 / fps
    delay_between_frames = 50
    return vs

def get_ball_mask(frame):
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # uncomment to get mask boundary test
    """
    lower, upper = hlpr.getMaskBoundary(frame)
    print(lower)
    print(upper)
    """
    mask = hlpr.GetMask(hsvImage, lower, upper, 3)
    return mask
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

# get input centroids, and also draws rectangle around the object
def get_ball_bounding_rects(mask,image):
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(contours)
    # Extract centroids from contours
    bounding_rects = []
    for c in cnts:
        area = cv2.contourArea(c)
        if area < 300:
            continue
        bounding_rects.append(cv2.boundingRect(c))
    return bounding_rects

def draw_ball_tracking_info(ball_objects, images=None):
    for obj in ball_objects.values():
        centroid = obj.centroid
        object_vector = obj.vector
        displacement = obj.displacement
        x,y,w,h = obj.bounding_rectangle
        for img in images:
            draw_ball_datas(img,obj)

def draw_ball_datas(img,obj):
    current_speed = 0
    centroid = obj.centroid
    # Get the x and y components of the vector
    vx, vy  = obj.vector
    displacement = obj.displacement
    current_speed = 0
    x,y,w,h = obj.bounding_rectangle
    # Extend arrow endpoint based on angle
    # =   # The movement vector
    scale_factor = 10
    # Extend the line from the centroid using the vector
    xext = int(centroid[0] + vx)
    yext = int(centroid[1] + vy)
    
    if len(obj.centroid_history) > 1:            
        current_speed = displacement / time_interval
    else:
        current_speed = 0
    # Draw bounding box
    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.arrowedLine(img, centroid, (xext, yext), (0, 255, 0), 2, tipLength=0.2)
    # Display object ID and speed
    cv2.putText(img, f"ID {obj.objectID} Speed:  px/sec",
                (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

def draw_path( path_points, start, end, images=None):

    start_point = start  # Use the provided start point directly

    for path in path_points:
        # Extract new start and end points from path points
        new_start = path[0]
        new_end = path[1]
        
        # Draw line from start point to new start
        for img in images:    
            cv2.line(img, start_point, new_start, grey_color, 2)
            cv2.line(img, start_point, new_start, red, 2)
        
        # Update start_point to be new_end for the next iteration
        start_point = new_end

    # After the loop, connect the last end point to the final goal
    end_point = end
    for img in images:    
        cv2.line(img, start_point, end_point, grey_color, 2)
        cv2.line(img, start_point, end_point, red, 2)
