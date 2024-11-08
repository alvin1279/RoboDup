import cv2
import numpy as np
import imutils
import json
import time

from centroidClass import CentroidTracker
import path_finder as pf
from BallSelector import BallSelector
import DetectBot
import BotMover
import VideoProcessor

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

selected_ball, CornerObjs = None, None
# blank image for path and objects to be drawn
start_time = time.time()

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

def ask_goal_post():
    # ask for goal post choice
    goal_post_choice = input("Choose goal post (left/right): ").strip().lower()
    if goal_post_choice == 'left':
        goal_location = "left"
    elif goal_post_choice == 'right':
        goal_location = "right"
    else:
        print("Invalid choice. Defaulting to left goal post.")
        goal_location = "left"
    return goal_location

def time_interval_checker(interval):
    global time_started_flag, start_time
    if not time_started_flag:
        start_time = time.time()
        time_started_flag = True
    if time.time() - start_time > interval:
        print('endtiming')
        time_started_flag = False
        start_time = time.time()
        return True
    return False

def process_frame(frame, ct, goal_location):
    # apply same width to calibration to avoid warping errors
    frame = imutils.resize(frame, width=900)
    # Apply the perspective warp to the frame
    frame = cv2.warpPerspective(frame, np.array(warp_matrix), (width, height)) 
    # Get mask for object detection
    ball_mask = VideoProcessor.get_ball_mask(frame)

    bounding_rects = VideoProcessor.get_ball_bounding_rects(ball_mask)
    # Update the tracker with new centroids
    objects = ct.update(bounding_rects) # contains ball as objects
    # find the bot
    bot_data = DetectBot.getBotData(frame)

    return objects, bot_data

def process_bot_movement(objects, bot_data, bt, selector, shape):
    # initate movement every 1 second
    initiate_movement = time_interval_checker(1)
    global already_selected_flag, ball_selected_flag, selected_ball, bot_detected, near_target, start, end

    if bot_data[0] is None or bot_data[1] is None or bot_data[2] is None:
        bot_detected = False
    else:
        bt.update_bot_data(bot_data)
        tail_centroid, head_centroid, _ = bot_data
        bot_center = ((tail_centroid[0] + head_centroid[0]) // 2, (tail_centroid[1] + head_centroid[1]) // 2)
        DetectBot.drawOrientation(frame, tail_centroid, head_centroid)
        bot_detected = True
        start = bot_center

    # select a ball
    if initiate_movement:
        if not already_selected_flag and bot_detected:
            # Select a ball
            tail_centroid, head_centroid, _ = bot_data
            bot_center = ((tail_centroid[0] + head_centroid[0]) // 2, (tail_centroid[1] + head_centroid[1]) // 2)
            selected_ball, CornerObjs = selector.select_ball(objects, bot_center)
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
    
    return start, end

def main():
    goal_location = ask_goal_post()
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
    bt = BotMover.BotMover(shape, x_boundaries, y_boundaries, goal_location)
    # Initialize BallSelector
    selector = BallSelector(x_boundaries, y_boundaries, goal_location)

    while True:
        ret, frame = vs.read()

        if not ret:
            break 
        # Create blank image same as frame
        blank_image = np.zeros((frame.shape[0], frame.shape[1], 3), np.uint8)
        
        # Process frame
        objects, bot_data = process_frame(frame, ct, goal_location)

        # draw ball tracking info
        draw_ball_tracking_info([frame, blank_image], objects)
        start, end = process_bot_movement(objects, bot_data, bt, selector, shape)
        path_points = pf.get_paths(cnts, start, end)
       
        # if ball_selected_flag and bot_detected and near_target:
        #     bt.near_target_motions(objects[selected_ball],bot)
        # get path points to target object
        # print(start,end)
        # draw path
        videoProcessor.draw_path([blank_image, frame], path_points, start, end)

        cv2.imshow('blank_image', blank_image)
        cv2.imshow("frame", frame)
        # reduce nummber to 1 to go back to normal frame rate
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    # Cleanup
    vs.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"An error occurred: {e}")