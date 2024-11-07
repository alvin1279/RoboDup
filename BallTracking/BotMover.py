import numpy as np
import path_finder as pf
from collections import deque

class BotMover:
    def __init__(self, shape,x_boundaries, y_boundaries,goal_location):
        self.shape = shape
        self.bot_data = None
        self.x_boundaries = x_boundaries
        self.y_boundaries = y_boundaries
        self.goal_location = goal_location
        # store current Bot data
        self.tail_centroid = None
        self.head_centroid = None
        self.angle = None
        self.bot_center = None
        self.bot_hemisphere = None
        # stores bot data after a movement is initiated
        self.predicted_angle = None
        self.predicted_position = None
        self.previous_rotation_scale = 1
        self.previous_distance_scale = 2
        self.history = deque(maxlen=10)  # Adjust maxlen as needed
        # PID controller parameters
        self.Kp = 1.0
        self.Ki = 0.1
        self.Kd = 0.05
        self.integral = 0
        self.previous_angle_error = 0
        self.previous_distance_error = 0
        # flags
        self.intersection = False
        self.near_target = False


    def updateBotData(self, bot_data):
        tail_centroid, head_centroid, angle = bot_data
        self.bot_data = bot_data
        self.tail_centroid = tail_centroid
        self.head_centroid = head_centroid
        self.angle = self.normalize_angle(angle)
        self.bot_center = ((tail_centroid[0] + head_centroid[0]) // 2, (tail_centroid[1] + head_centroid[1]) // 2)
        
        # Determine the hemisphere of the bot
        self.bot_hemisphere = 'top' if self.bot_center[1] > self.shape[1] // 2 else 'bottom'
        
        # Adjust scales with PID if there is previous bot data

    # start the bot movement
    def move_bot(self, selected_centroid, all_ball_objects):
        if self.bot_data is None:
            raise ValueError("Bot data is not updated")
        # recalculate the predicted values based on the previous movement
        if self.predicted_angle is not None:
            angle_error = self.angle - self.predicted_angle
            angle_error = self.normalize_angle(angle_error)
            distance_error = np.linalg.norm(np.array(self.bot_center) - np.array(self.predicted_position))
            self.adjust_angle_scales_with_pid(angle_error)
            self.adjust_distance_scales_with_pid(distance_error)

        # calculate the distance and angle between bot center and selected object
        bot_in_between = self.check_bot_in_between(self.tail_centroid, selected_centroid)
        if bot_in_between:
            return false
            self.get_behing_ball(selected_centroid,all_ball_objects)
        else:
            self.move_directly(selected_centroid, all_ball_objects)

    def move_directly(self, selected_centroid, all_ball_objects):
        distance = np.sqrt((self.bot_center[0] - selected_centroid[0]) ** 2 + (self.bot_center[1] - selected_centroid[1]) ** 2)
        # move only till specific distance
        if distance > 90:
            self.near_target = False
            intersection_points = self.check_interSection(selected_centroid, all_ball_objects)
            intersection_points = None
            if intersection_points is None:
                self.generate_movement_comand(selected_centroid, distance)
                self.intersection = False
            else:
                self.intersection = True
        else:
            self.near_target = True

    def check_interSection(self, selected_centroid, all_ball_objects):
        if self.bot_data is None:
            raise ValueError("Bot data is not updated")
        # check if the selected object is in the path of the bot
        m,c = pf.line_equation(self.bot_center, selected_centroid)
        intersection_points = None
        for obj in all_ball_objects:
            if obj.centroid == selected_centroid:
                continue
            else:
                intersection_points = pf.line_circle_intersection(m,c, selected_centroid,7)
                break
        return intersection_points



    def generate_movement_comand(self, selected_centroid, distance):
        if self.bot_data is None:
            raise ValueError("Bot data is not updated")
        # move the bot to the selected ball
        movement_command = ''
        
        tail_centroid, head_centroid, angle = self.bot_data
        bot_center = ((tail_centroid[0] + head_centroid[0]) // 2, (tail_centroid[1] + head_centroid[1]) // 2)
        bot_ball_angle = np.arctan2(selected_centroid[1] - bot_center[1], selected_centroid[0] - bot_center[0]) * 180 / np.pi
        bot_angle = self.angle
        print('Bot angle:', angle)
        # angle = self.normalize_angle(angle)
        # adjust angle until almost till zero
        if angle > 90 and angle < 270:
            # do left if bot is in top half to avoid hitting the wall
            if self.bot_hemisphere == 'top':
                direction = 'r'
            else :
                direction = 'l'
            movement_command += direction + '10' # adjust value according to bot response
        else:
            movement_command += self.determine_rotation(angle)
            movement_command += 'f' + self.decaToHex(self.previous_distance_scale)
        # send the movement command to the bot
        movement_command += 's00'
        print(f"Bot Movement: {movement_command}")
        self.predic_movement_updates(distance)

    def determine_rotation(self, angle_differnce):
        direction = ''
        rotation_scale_hex = self.decaToHex(self.previous_rotation_scale)
        if angle_differnce > 0:
            return 'r'
        else: 
            return 'l'    
    def predic_movement_updates(self, distance):
        # Calculate predicted values based on scaling factors
        predicted_angle = self.normalize_angle(self.angle + (10 * self.previous_rotation_scale))

        predicted_position = (self.bot_center[0] + 10 * self.previous_distance_scale * np.cos(np.radians(predicted_angle)),
                              self.bot_center[1] + 10 * self.previous_distance_scale * np.sin(np.radians(predicted_angle)))
        
        self.predicted_angle = predicted_angle
        self.predicted_position = predicted_position
        # self.history.append((predicted_angle, predicted_distance, rotation_scale, distance_scale))

    def adjust_distance_scales_with_pid(self,error):
        # PID controller logic
        self.integral += error
        derivative = error - self.previous_distance_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_distance_error = error

        # Adjust rotation scale based on PID output
        self.previous_distance_scale = max(1, min(10, self.previous_distance_scale + output))

    def adjust_angle_scales_with_pid(self, error):
        # PID controller logic
        self.integral += error
        derivative = error - self.previous_angle_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_angle_error = error

        # Adjust rotation scale based on PID output
        self.previous_rotation_scale = max(1, min(10, self.previous_rotation_scale + output))
    def get_behing_ball(self, selected_centroid,all_ball_objects):
        # get the position behind the ball
        if self.bot_hemisphere == 'top':
            new_centroid = (selected_centroid[0], selected_centroid[1] - 10)
        else:
            new_centroid = (selected_centroid[0], selected_centroid[1] + 10)
        self.move_directly(new_centroid, all_ball_objects)
    def near_target_motions(selected_centroid):
        print("Bot is near target")
        distance = np.sqrt((self.bot_center[0] - selected_centroid[0]) ** 2 + (self.bot_center[1] - selected_centroid[1]) ** 2)
        
    def move_to_location(location):
        print(f"Moving to location {location}")
        # Move the bot to the location
        locx,locy = location
        bot_location_angle = np.arctan2(locy - self.bot_center[1], locx - self.bot_center[0]) * 180 / np.pi
        distance = np.sqrt((self.bot_center[0] - locx) ** 2 + (self.bot_center[1] - locy) **
        bot_angle = self.angle
        angle_differnce =bot_location_angle - bot_angle 
        if abs(angle_differnce) > 10:
            # Rotate the bot to the location
            rotation_direction = self.determine_rotation(angle_differnce)
            movement_command = rotation_direction + slef.decaToHex(self.previous_rotation_scale)
        else:
            if distance > 90:
                movement_command = 'F' + self.decaToHex(self.previous_distance_scale)
            else:
                movement_command = 's00'
    @staticmethod
    def normalize_angle(angle):
        return angle % 360

    @staticmethod
    def decaToHex(dec):
        clamped_value = max(0,min(255,int(dec)))
        return hex(clamped_value).split('x')[-1].zfill(2)

    @staticmethod
    def hexToDec(hex):
        return int(hex, 16)


