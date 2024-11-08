import cv2
import numpy as np
import imutils
from collections import defaultdict
from collections import deque
import hsvMaskUtility as hlpr
from scipy.spatial import distance as dist
import json
import time

from centroidClass import CentroidTracker
import path_finder as pf
import BallSelector as Selector
import DetectBot
import BotMover
import VideoProcessor

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



# temperory start and end points
start = (100, 100)  # Example start point, adjust as per your image
end = (300, 300) 

# initialising flags
already_selected_flag = False
ball_selected_flag = False
selectedBall = None
bot_detected = False
goal_location = "left"
near_target = False
time_started_flag = False

contour_areas = deque(maxlen=15)
contour_radius = deque(maxlen=15)
average_radius = 30

# ask for goal post choice
goal_post_choice = input("Choose goal post (left/right): ").strip().lower()
if goal_post_choice == 'left':
    goal_location = "left"
elif goal_post_choice == 'right':
    goal_location = "right"
else:
    print("Invalid choice. Defaulting to left goal post.")
    goal_location = "left"

# load frame data
transformed_left_goal_post, transformed_right_goal_post, redux, warp_matrix, shape, width, height = load_frame_data()
# construct a smaller rectangle inside shape to avoid boundary errors and corners
x_offset = 5
y_offset = 5
x_boundaries = shape[0] + x_offset, shape[0] - x_offset
y_boundaries = shape[1] + y_offset, shape[1] - y_offset


# Load video file / camera
vs = VideoProcessor.load_video_stream('http://192.168.83.138:8080/video')
# Initialize centroid tracker
ct = CentroidTracker()
# Initialise BotMover
bt = BotMover.BotMover(shape,x_boundaries,y_boundaries,goal_location)

selected_ball,CornerObjs = None, None

# blank image for path and objects to be drawn
path_image = np.zeros((500, 900, 3), np.uint8)
start_time = time.time()

while True:
    initiate_movement = False
    if not time_started_flag:
        start_time = time.time()
        time_started_flag = True
    if time.time() - start_time > 3:
        print('endtiming')
        time_started_flag = False
        start_time = time.time()
        initiate_movement = True
    ret, frame = vs.read()

    if not ret:
        break
    # apply same width to calibration to avoid warping errors
    frame = imutils.resize(frame, width=900)
    # Apply the perspective warp to the frame
    frame = cv2.warpPerspective(frame, np.array(warp_matrix), (width, height)) 


    # Create blank image same as frame
    blank_image = np.zeros((frame.shape[0], frame.shape[1], 3), np.uint8)
    # Get mask for object detection
    ball_mask = VideoProcessor.get_ball_mask(frame)

    bitwise_not = cv2.bitwise_not(mask)

    bounding_rects = VideoProcessor.get_ball_bounding_rects(ball_mask)
    # Update the tracker with new centroids
    objects = ct.update(bounding_rects) # contains ball as objects
    # draw ball tracking info
    draw_ball_tracking_info([frame, blank_image], objects)
    
    # find the bot
    bot_data = DetectBot.getBotData(frame)
    if bot_data[0] == None or bot_data[1] == None or bot_data[2] == None:
        bot_detected = False
    else:
        bt.update_bot_data(bot_data)
        tail_centroid,head_centroid,_ = bot_data
        bot_center = ((tail_centroid[0] + head_centroid[0]) // 2, (tail_centroid[1] + head_centroid[1]) // 2)
        DetectBot.drawOrientation(frame,tail_centroid,head_centroid)
        bot_detected = True
        start = bot_center
    # select a ball
    if initiate_movement:
        if not already_selected_flag and bot_detected:
            # Select a ball
            tail_centroid, head_centroid, _ = bot_data
            bot_center = ((tail_centroid[0] + head_centroid[0]) // 2, (tail_centroid[1] + head_centroid[1]) // 2)
            selected_ball, CornerObjs = Selector.selectBall(objects, x_boundaries, y_boundaries, bot_center, goal_location)
            if selected_ball is not None:
                already_selected_flag = True
                ball_selected_flag = True
                end = selected_ball.centroid
        else:
            # set selected_flag to false if selected_ball is not in objects
            for obj in objects.values():
                if obj == selected_ball:
                    break
                else:
                    already_selected_flag = False
        if ball_selected_flag and bot_detected and selected_ball and already_selected_flag:
            # move the bot
            print('initiating movement')
            bt.move_to_selected_ball(selected_ball.centroid)
            near_target = bt.near_target
        else:
            print('moving to default location')
            if goal_location == 'left':
                bt.orient_and_move_to_location(shape[0] // 2 + 30, shape[1] // 30)
            else:
                bt.orient_and_move_to_location(shape[0] // 2 - 30, shape[1] // 30)
    # if ball_selected_flag and bot_detected and near_target:
    #     bt.near_target_motions(objects[selected_ball],bot)
    # get path points to target object
    path_points = pf.get_paths(cnts, start, end)
    # print(start,end)
    draw_path(bitwise_not, frame, path_points,start, end)

    cv2.imshow('mask',mask)
    cv2.imshow("blank_image", frame)
    # reduce nummber to 1 to go back to normal frame rate
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

# Cleanup
vs.release()
cv2.destroyAllWindows()