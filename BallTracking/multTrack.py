import cv2
import numpy as np
import imutils
import json
import time
import traceback

from multiprocessing import Process, Queue, Event
from collections import deque
from centroidClass import CentroidTracker
import path_finder as pf
from BallSelector import BallSelector
import DetectBot
import BotMover
import VideoProcessor

# Temporary start and end points
start = (100, 100)  # Example start point, adjust as per your image
end = (300, 300) 

# Initializing flags
already_selected_flag = False
ball_selected_flag = False
selectedBall = None
bot_detected = False
goal_location = "left"
near_target = False
time_started_flag = False

selected_ball, region = None, None
path = []
start_time = time.time()
frame_counter = 0

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
    goal_post_choice = input("Choose goal post (left/right): ").strip().lower()
    return "left" if goal_post_choice != 'right' else "right"

def time_interval_checker(interval):
    global time_started_flag, start_time
    if not time_started_flag:
        start_time = time.time()
        time_started_flag = True
    if time.time() - start_time > interval:
        time_started_flag = False
        start_time = time.time()
        return True
    return False

def process_frame(frame, ct, goal_location, warp_matrix, width, height):
    frame = imutils.resize(frame, width=900)
    frame = cv2.warpPerspective(frame, np.array(warp_matrix), (width, height)) 
    ball_mask = VideoProcessor.get_ball_mask(frame)
    bounding_rects = VideoProcessor.get_ball_bounding_rects(ball_mask)
    objects = ct.update(bounding_rects)  # Contains ball objects
    bot_data = DetectBot.getBotData(frame)
    return objects, bot_data, frame

def get_path(selector, bt):
    path = deque()  # Initialize path as a deque (stack-like behavior)
    if len(selector.balls_zone_positive_x) > 0:
        selector.select_ball_non_edge_positive()
        point = bt.get_shifted_location(selector.selected_ball[0].centroid)
        path.append(point)  # Append to the end of the deque
        path.append(selector.selected_ball[0].centroid)
    bt.path = path
        

def process_bot_movement(objects, bot_data, bt, selector, shape, goal_location, frame):
    global bot_detected, near_target, start, end, path

    initiate_movement = time_interval_checker(1)
    
    if bot_data[0] is None or bot_data[1] is None or bot_data[2] is None:
        bot_detected = False
    else:
        bot_detected = True
        bt.update_bot_data(bot_data)
        selector.update_zones(bot_data[0], objects)
        tail_centroid, head_centroid, _ = bot_data
        bot_center = ((tail_centroid[0] + head_centroid[0]) // 2, (tail_centroid[1] + head_centroid[1]) // 2)
        start = bot_center
        if selector.ball_selected_flag:
            selector.check_selected_ball(objects)
            # bt.move_to_selected_ball_in_between(selector.selected_ball[0])
        else:
            if len(bt.path) > 0:
                distance = 100
                if bt.target_reached or bt.target is None:
                    bt.target = bt.path.popleft()
                    distance = np.linalg.norm(np.array(bot_center) - np.array(bt.target))
                if distance < 30:
                    bt.target_reached = True
                if not bt.target_reached:
                    bt.move_to_location(bt.target)
                    
            elif not bt.target_reached and len(bt.path) == 0:
                get_path(selector,bt)
                bt.target_reached = False

def bot_movement_process(bt, selector, shape, goal_location, frame_queue, bot_data_queue):
    try:
        print("core 3 method called")
        while True:
            if frame_queue.empty() or bot_data_queue.empty():
                # print("Queues are empty, waiting for more data...")
                continue  # Wait and keep looping without exiting
            frame = frame_queue.get()
            bot_data, objects = bot_data_queue.get()
            process_bot_movement(objects, bot_data, bt, selector, shape, goal_location, frame)
    except Exception as e:
        print(f"Error in bot_movement_process: {e}")
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()  # Ensure all windows are closed properly
def draw_tracked_frame(frame_queue, bot_data_queue):
    try:
        print("core 2 draw method called")
        while True:
            if frame_queue.empty() or bot_data_queue.empty():
                # print("Queues are empty, waiting for more data...")
                continue  # Wait and keep looping without exiting
            frame = frame_queue.get()
            bot_data, objects = bot_data_queue.get()
            tail_centroid, head_centroid, _ = bot_data
            VideoProcessor.draw_ball_tracking_info(frame, objects)
            DetectBot.drawOrientation(frame, tail_centroid, head_centroid)
            cv2.imshow('drawn frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except Exception as e:
        print(f"Error in bot_movement_process: {e}")
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()  # Ensure all windows are closed properly
MAX_QUEUE_SIZE = 10

def main():
    global frame_counter
    goal_location = ask_goal_post()
    transformed_left_goal_post, transformed_right_goal_post, redux, warp_matrix, shape, width, height = load_frame_data()

    vs = VideoProcessor.load_video_stream('Samples/rec1.mp4')
    ct = CentroidTracker()
    bt = BotMover.BotMover(shape, (shape[0] + 5, shape[0] - 5), (shape[1] + 5, shape[1] - 5), goal_location)
    selector = BallSelector(goal_location, shape)

    frame_queue = Queue()
    bot_data_queue = Queue()
    draw_tracked_frame_process = Process(target=draw_tracked_frame, args=(frame_queue, bot_data_queue))
    draw_tracked_frame_process.daemon = True
    draw_tracked_frame_process.start()
    bot_process = Process(target=bot_movement_process, args=(bt, selector, shape, goal_location, frame_queue, bot_data_queue))
    bot_process.daemon = True
    bot_process.start()

    try:
        while True:
            if not bot_process.is_alive():
                print("bot_movement_process encountered an error. Restarting...")
                bot_process.terminate()
                bot_process.join()
                bot_process = Process(target=bot_movement_process, args=(bt, selector, shape, goal_location, frame_queue, bot_data_queue))
                bot_process.start()

            ret, frame = vs.read()
            if not ret:
                break

            objects, bot_data, retFrame = process_frame(frame, ct, goal_location, warp_matrix, width, height)
            frame_queue.put(retFrame)
            bot_data_queue.put((bot_data, objects))
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        vs.release()
        draw_tracked_frame_process.terminate()
        bot_process.terminate()
        draw_tracked_frame_process.join()
        bot_process.join()
        cv2.destroyAllWindows()
        print('Resources released and windows destroyed.')

if __name__ == "__main__":
    try:
        main()
        print('main ended')
    except Exception as e:
        print(f"An error occurred: {e}")
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()  # Ensure all windows are closed properly
        print('Final cleanup done.')
