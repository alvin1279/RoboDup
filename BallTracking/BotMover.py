import numpy as np
import path_finder as pf
from collections import deque

line_equation_matrix = []
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
        self.bot_quadrant = None

    def update_bot_data(self, bot_data):
        tail_centroid, head_centroid, angle = bot_data
        self.bot_data = bot_data
        self.tail_centroid = tail_centroid
        self.head_centroid = head_centroid
        self.angle = self.normalize_angle(angle)
        self.bot_center = ((tail_centroid[0] + head_centroid[0]) // 2, (tail_centroid[1] + head_centroid[1]) // 2)
        
        # Determine the hemisphere of the bot
        self.bot_hemisphere = 'top' if self.bot_center[1] > self.shape[1] // 2 else 'bottom'
        self.bot_quadrant = self.determine_quadrant(self.bot_center)
    
    # Adjust scales with PID if there is previous bot data
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
    # all movements based on left side goal post
    def move_to_selected_ball(self, selected_ball):
        print('moving to selected ball')
        location = selected_ball.centroid
        dy = location[1] - self.bot_center[1]
        dx = location[0] - self.bot_center[0]

        if abs(dy) > 10:  # sets bot and ball in same horizontal line
        # use pre calculated line equations to shift to favourable angles later
        # assuming the selected ball is in front of the bot and goal post
            y_shifted_location = (self.bot_center[0], self.bot_center[1] + dy)
            self.orient_and_move_to_location(y_shifted_location)
        else:
            self.orient_and_move_to_location(location)

    # Orient bot to location and move to the location
    def orient_and_move_to_location(self,location):
        print(f"Moving to location {location}")
        movement_command = 's00'
        bot_location_angle = self.get_bot_location_angle(self.bot_center,location)
        bot_location_angle, bot_angle = self.adjusted_bot_angle(bot_location_angle)
        angle_differnce =bot_location_angle - bot_angle
        # Rotate the bot to align with the location position
        if abs(angle_differnce) > 10:
            rotation_direction = self.get_rotation_direction(angle_differnce)
            # Adjust the scale based on the difference in angle
            movement_command = rotation_direction + self.decaToHex(self.previous_rotation_scale)
        # if bot is in same aligment as location go forward
        else:
            # Adjust the scale based on distance
            movement_command = 'f' + self.decaToHex(self.previous_distance_scale)

    def get_rotation_direction(self, angle_differnce):
        if angle_differnce > 0:
            return 'l'
        else: 
            return 'r'     
    # shifting by 90 degrees to avoid 360 to 1 flipping
    def adjusted_bot_angle(self, bot_location_angle):
        location_quadrant = self.determine_quadrant(bot_location_angle)
        bot_angle = self.angle
        if location_quadrant ==1  or location_quadrant ==2:
            if bot_angle < 180:
                bot_location_angle = (bot_location_angle + 90)  % 360
                bot_angle = (bot_angle + 90) % 360
            else:
                bot_location_angle = (bot_location_angle - 90)  % 360
                bot_angle = (bot_angle - 90) % 360
        return bot_location_angle, bot_angle
    def get_bot_location_angle(self, location):
        angle = np.arctan2(location[1] - self.bot_center[1], location[0] - self.bot_center[0])
        return self.normalize_angle(angle * 180 / np.pi)

    @staticmethod
    def decaToHex(dec):
        clamped_value = max(0,min(255,int(dec)))
        return hex(clamped_value).split('x')[-1].zfill(2)
    @staticmethod
    def hexToDec(hex):
        return int(hex, 16)
    @staticmethod
    def normalize_angle(angle):
        return (angle + 360) % 360
    @staticmethod
    def determine_quadrant(angle):
        if angle >= 0 and angle < 90:
            return 1
        elif angle >= 90 and angle < 180:
            return 2
        elif angle >= 180 and angle < 270:
            return 3
        else:
            return 4

