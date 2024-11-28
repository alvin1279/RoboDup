import cv2
import numpy as np
import imutils
import json
import time
import traceback
import websocket

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
    # frame = imutils.resize(frame, width=900)
    frame = cv2.warpPerspective(frame, np.array(warp_matrix), (width, height)) 
    ball_mask = VideoProcessor.get_ball_mask(frame)
    bounding_rects = VideoProcessor.get_ball_bounding_rects(ball_mask)
    objects = ct.update(bounding_rects)  # Contains ball objects
    bot_data = DetectBot.getBotData(frame)
    return objects, bot_data, frame

def select_ball_and_set_path(selector, bt):
    print('negative ba;lls',len(selector.balls_zone_negative_x))
    print('positive ba;lls',len(selector.balls_zone_positive_x))
    if len(selector.balls_zone_positive_x) > 0:
        print('selecting positive ball')
        selector.select_ball_non_edge_positive()
        shifted_point = bt.get_shifted_location(selector.selected_ball.centroid)
        bt.path['intermediate'] = int(shifted_point[0]),int(shifted_point[1])
        bt.path['final'] = int(selector.selected_ball.centroid[0]),int(selector.selected_ball.centroid[1])
        bt.orient = True
    elif len(selector.balls_zone_negative_x) > 0:
        print('selecting negative path')
        selector.select_ball_non_edge_negative()
        y_channel_mid_point = bt.get_y_channel_midpoint(selector.balls_zone_negative_x)
        bt.path['intermediate'] = (bt.bot_center[0], y_channel_mid_point)
        bt.path['final'] = (selector.balls_zone_negative_x[-1].centroid[0]+60,y_channel_mid_point)
        bt.orient = True

def process_bot_movement(objects, bot_data, bt, selector, shape, goal_location, frame,ws):
    global bot_detected, near_target, start, end, path

    initiate_movement = time_interval_checker(1)
    
    if bot_data[0] is None or bot_data[1] is None or bot_data[2] is None:
        print("Bot not detected. Waiting for bot detection...")
    else:
        bt.update_bot_data(bot_data)
        if bt.bot_near_boundary:
            print("Bot near boundary. Resetting flags...")
            # selector.reset_flags()
            # bt.reset_flags()
            # ws.send('s02')
            # time.sleep(1)
            # ws.send('Fff')
            # time.sleep(2)
            # ws.send('B35r03l01')
            # time.sleep(5)
            # # do random stuff here with 10s delay added
            # return
        selector.update_zones(bot_data[0], objects)
        tail_centroid, head_centroid, _ = bot_data
        bot_center = ((tail_centroid[0] + head_centroid[0]) // 2, (tail_centroid[1] + head_centroid[1]) // 2)
        start = bot_center
        if not selector.already_selected_flag:
            print('selecting ball')
            select_ball_and_set_path(selector,bt)

        if selector.ball_selected_flag:
            if not selector.check_ball_still_exist(objects):
                print ("Ball not found. Resetting flags...")
                selector.reset_flags()
                bt.reset_flags()
                return
            else:   
                # checking if path exist
                ws.send('s01')
                initate_bot_movement(bt, selector,ws)
        else:
            # initiate random movement here
            pass
def initate_bot_movement(bt, selector,ws):
    print(bt.path)
    if bt.path['intermediate'] is not None:
        print('selecting Intermediate')
        bt.current_target = 'intermediate'
    elif bt.path['final'] is not None:
        bt.current_target = 'final'
        print('selecting final')
    else :
        bt.current_target = None
        # reset everything if no path exist
        selector.reset_flags()
        bt.reset_flags()
    if bt.current_target is not None:
        if not bt.orient:
            print("moving staright")
            move_straight(bt)
            ws.send(bt.bot_command)
            print(bt.bot_command)
        else:
            angle_differnce = bt.get_angle_differnce(bt.path[bt.current_target])
            if abs(angle_differnce) < 5:
                bt.orient = False
            else:
                print('orienting bot')
                bt.orient_bot(angle_differnce)
                ws.send(bt.bot_command)

def move_straight(bt):
    current_target_location = bt.path[bt.current_target]
    distance  = np.sqrt((bt.bot_center[0] - current_target_location[0])**2 + (bt.bot_center[1] - current_target_location[1])**2)
    print(distance)
    if distance < 30:
        print(bt.current_target)
        bt.path[bt.current_target] = None
        bt.orient = True
    else:
        bt.move_to_location(current_target_location)
        print(bt.bot_command)

def bot_movement_process(bt, selector, shape, goal_location, frame_queue, bot_data_queue, path_queue,ws):
    try:
        print("core 3 method called")
        while True:
            if frame_queue.empty() or bot_data_queue.empty():
                # print("Queues are empty, waiting for more data...")
                continue  # Wait and keep looping without exiting
            frame = frame_queue.get()
            bot_data, objects = bot_data_queue.get()
            process_bot_movement(objects, bot_data, bt, selector, shape, goal_location, frame,ws)

            # Send path to the queue
            if not path_queue.full():
                path = []
                path.append(bt.bot_center)
                if bt.path['intermediate'] is not None:
                    path.append(bt.path['intermediate'])                    
                if bt.path['final'] is not None:
                    path.append(bt.path['final'])
                path_queue.put(path)
    except Exception as e:
        print(f"Error in bot_movement_process: {e}")
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()  # Ensure all windows are closed properly
def draw_tracked_frame(frame_queue, bot_data_queue,path_queue):
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

             # Draw the path if available
            if not path_queue.empty():
                path = path_queue.get()
                print('path',path)
                for i in range(len(path) - 1):
                    if path[i +1] is not None:
                        cv2.line(frame, path[i], path[i + 1], (0, 255, 255), 2)  # Green line for path

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
    bot_ip = "ws://192.168.57.196:81"  # Replace with your bot's IP address

    # # jasira Ip: 192.168.54.196:81
    # # hashar: 192.168.57.196:81
    ws = websocket.WebSocket()
    ws.connect(bot_ip)
    # ws = 2
    ws.send('Bf3')
    transformed_left_goal_post, transformed_right_goal_post, redux, warp_matrix, shape, width, height = load_frame_data()

    vs = VideoProcessor.load_video_stream('http://localhost:4747/video')
    # vs = VideoProcessor.load_video_stream('Samples/rec1.mp4')
    ct = CentroidTracker()
    # print(shape)
    bt = BotMover.BotMover(shape, (0, shape[0]), (0, shape[1]), goal_location)
    selector = BallSelector(goal_location, shape)

    print('shape',shape)
    frame_queue = Queue()
    bot_data_queue = Queue()
    path_queue = Queue()  # New queue for path data

    draw_tracked_frame_process = Process(target=draw_tracked_frame, args=(frame_queue, bot_data_queue,path_queue))
    draw_tracked_frame_process.daemon = True
    draw_tracked_frame_process.start()
    bot_process = Process(target=bot_movement_process, args=(bt, selector, shape, goal_location, frame_queue, bot_data_queue, path_queue,ws))
    bot_process.daemon = True
    bot_process.start()

    try:
        while True:
            if not bot_process.is_alive():
                print("bot_movement_process encountered an error. Restarting...")
                bot_process.terminate()
                bot_process.join()
                bot_process = Process(target=bot_movement_process, args=(bt, selector, shape, goal_location, frame_queue, bot_data_queue,path_queue,ws))
                bot_process.start()

            ret, frame = vs.read()
            # print('frameSize',frame.shape)
            if not ret:
                break

            objects, bot_data, retFrame = process_frame(frame, ct, goal_location, warp_matrix, width, height)
            # print('retFramSize',retFrame.shape)
            print('frame size', retFrame.shape)
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
