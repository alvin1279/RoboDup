import cv2
import numpy as np
import imutils
import json
import time
from multiprocessing import Process, Queue
from queue import Full

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
    frame = imutils.resize(frame, width=1200)
    frame = cv2.warpPerspective(frame, np.array(warp_matrix), (width, height)) 
    ball_mask = VideoProcessor.get_ball_mask(frame)
    bounding_rects = VideoProcessor.get_ball_bounding_rects(ball_mask,frame)
    objects = ct.update(bounding_rects)  # Contains ball objects
    bot_data = DetectBot.getBotData(frame)
    tail,head,_ = bot_data
    DetectBot.drawOrientation(frame,tail,head)
    return objects, bot_data, frame

def process_bot_movement(objects, bot_data, bt, selector, shape, goal_location, frame):
    global bot_detected, near_target, start, end, path

    initiate_movement = time_interval_checker(1)

    if bot_data[0] is None or bot_data[1] is None or bot_data[2] is None:
        bot_detected = False
    else:
        bt.update_bot_data(bot_data)
        selector.update_zones(bot_data[0], objects)
        tail_centroid, head_centroid, _ = bot_data
        bot_center = ((tail_centroid[0] + head_centroid[0]) // 2, (tail_centroid[1] + head_centroid[1]) // 2)
        DetectBot.drawOrientation(frame, tail_centroid, head_centroid)
        bot_detected = True
        start = bot_center

    if initiate_movement:
        if not selector.already_selected_flag and bot_detected:
            if len(selector.balls_zone_positive_x) > 0:
                selector.select_ball_non_edge_positive()
                end = selector.selected_ball.centroid
            elif len(selector.balls_zone_negative_x) > 0:
                negative_zone_path = bt.get_path_behind_negative_zone(selector.balls_zone_negative_x[-1],selector.negative_zone)
                for point in negative_zone_path:
                    end = point
                    bot_location_angle = bt.get_bot_location_angle(bot_center, point)
                    bot_location_angle, bot_angle = bt.adjusted_bot_angle(bot_location_angle)
                    angle_difference = bot_location_angle - bot_angle
                    command = ''
                    if abs(angle_difference) > 10:
                        command += bt.orient_bot(angle_difference)
                    command += bt.move_to_location(end)
                    print(command)
                    # send command and wait for it to execute
                    time.sleep(0.5)
        else:
            selector.already_selected_flag = any(obj == selector.selected_ball for obj in objects.values())

        if selector.ball_selected_flag and bot_detected and selector.selected_ball and selector.already_selected_flag:
            if selector.region == 1:
                bt.move_to_selected_ball_in_between(selector.selected_ball)
            near_target = bt.near_target
        else:
            if goal_location == 'left':
                bt.orient_and_move_to_location(shape[0] // 2 + 30, shape[1] // 30)
            else:
                bt.orient_and_move_to_location(shape[0] // 2 - 30, shape[1] // 30)
    
    return start, end

def bot_movement_process(bt, selector, shape, goal_location, frame_queue, bot_data_queue):
    try:
        while True:
            if not frame_queue.empty() and not bot_data_queue.empty():
                frame = frame_queue.get()
                bot_data, objects = bot_data_queue.get()
                start, end = process_bot_movement(objects, bot_data, bt, selector, shape, goal_location, frame)
                cv2.imshow('robo frame', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except Exception as e:
        print(f"Error in bot_movement_process: {e}")

# class AutoRemovingQueue(Queue):
#     def __init__(self, maxsize=0):
#         super().__init__(maxsize)
#
#     def put(self, item, block=True, timeout=None):
#         # Remove items if the queue is full
#         while self.full():
#             try:
#                 self.get_nowait()
#             except Empty:
#                 pass
#         # Put the new item into the queue with keyword arguments specified
#         super().put(item, block=block, timeout=timeout)
# Define the maximum size for the queues
MAX_QUEUE_SIZE = 10

def main():
    goal_location = ask_goal_post()
    transformed_left_goal_post, transformed_right_goal_post, redux, warp_matrix, shape, width, height = load_frame_data()

    x_offset = 5
    y_offset = 5
    x_boundaries = shape[0] + x_offset, shape[0] - x_offset
    y_boundaries = shape[1] + y_offset, shape[1] - y_offset

    vs = VideoProcessor.load_video_stream('http://192.168.104.247:8080//video')
    ct = CentroidTracker()
    bt = BotMover.BotMover(shape, x_boundaries, y_boundaries, goal_location)
    selector = BallSelector(goal_location, shape)

    # Create queues with a maximum size
    frame_queue = Queue()
    bot_data_queue = Queue()


    # Start bot movement process
    bot_process = Process(target=bot_movement_process, args=(bt, selector, shape, goal_location, frame_queue, bot_data_queue))
    bot_process.daemon = True  # Make bot_movement_process a daemon
    bot_process.start()

    while True:
        # Restart bot_movement_process if it crashes
        if not bot_process.is_alive():
            print("bot_movement_process encountered an error. Restarting...")
            bot_process = Process(target=bot_movement_process, args=(bt, selector, shape, goal_location, frame_queue, bot_data_queue))
            bot_process.start()

        ret, frame = vs.read()
        if not ret:
            break
        objects, bot_data,frame = process_frame(frame, ct, goal_location, warp_matrix, width, height)
        blank_image = np.zeros((frame.shape[0], frame.shape[1], 3), np.uint8)
        VideoProcessor.draw_ball_tracking_info(objects,[frame,blank_image])

        # Put items in the queue, automatically removing old values if necessary
        frame_queue.put(frame)
        bot_data_queue.put((bot_data, objects))

        cv2.imshow('blank_image', blank_image)
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    vs.release()
    cv2.destroyAllWindows()
    # bot_process.terminate()
    # bot_process.join()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"An error occurred: {e}")
