import numpy as np
import path_finder as pf
from collections import deque

line_equation_matrix = []
# all movements based on left side goal post
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
        self.bot_command = ''
        self.y_channel_mid_point = None
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
        self.y_channel_found = False

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
    
    def get_path_behind_negative_zone(self,selected_ball,negative_zone_balls):
        print('moving behind negative zone')
        ball,distance,point_angle = selected_ball
        path = []
        y_channel_mid_point = self.get_y_path_channel(self, negative_zone_balls)
        self.y_channel_mid_point = y_channel_mid_point
        path.append(self.bot_center[0], y_channel_mid_point)
        path.append(self.bot_center[0], ball.centroid[1]+10)
        return path

    def move_to_location(self, location):
        print(f"Moving to location {location}")
        bot_location_angle = self.get_bot_location_angle(self.bot_center,location)
        angle_differnce =bot_location_angle - self.angle
        if abs(angle_differnce) > 10:
            self.orient_and_move_to_location(self,location,bot_location_angle)
        else:
            distance = self.get_distance(self.bot_center,location)
            self.move_forward(distance)
    def get_y_path_channel(self, all_objects):
        # Width of the y channel to look for
        y_channel_width = 10
        bot_y = self.bot_center[1]

        # Get all y-coordinates of object centroids
        y_positions = sorted(obj.centroid[1] for obj, _, _ in all_objects)

        # If no objects remain, return bot_y as the only channel available
        if not y_positions:
            return bot_y

        # Define bounds based on object positions
        y_min = 0
        y_max = max(y_positions)  # No need to check if empty, handled above

        # Initialize variables to track the closest gap
        closest_gap_distance = float('inf')
        channel_mid_point = bot_y  # Default to bot's y position if no valid gap is found

        # Loop through sorted y-positions to find gaps
        for i in range(len(y_positions) - 1):
            gap_start = y_positions[i]
            gap_end = y_positions[i + 1]
            gap_size = gap_end - gap_start

            # Check if the gap is large enough for the required channel width
            if gap_size >= y_channel_width:
                # Calculate center of the gap
                gap_center = (gap_start + gap_end) / 2
                # Calculate distance from bot_y to gap center
                gap_distance = abs(gap_center - bot_y)

                # If this gap is closer to the bot than previous gaps, update the closest gap
                if gap_distance < closest_gap_distance:
                    closest_gap_distance = gap_distance
                    channel_mid_point = gap_center

        # If no valid channel was found (channel_mid_point == bot_y) and there are objects left, try again by removing the last object
        if channel_mid_point == bot_y and len(all_objects) > 1:
            # Remove the last object and call the method recursively
            return self.get_y_path_channel(all_objects[:-1])

        # Return the y-coordinate of the midpoint of the closest empty y channel found
        return int(channel_mid_point)

    def move_to_selected_ball_in_between(self, selected_ball):
        print('moving to selected ball')
        ball,distance,point_angle = selected_ball
        location = ball.centroid

        dy = location[1] - self.bot_center[1]

        if abs(dy) > 10:  # sets bot and ball in same horizontal line
            self.shift_to_location(location,dy)
        else:
            self.orient_and_move_to_location(location,point_angle)
    def shift_to_location(self,location,dy):
        bot_location_angle = self.get_bot_location_angle(self.bot_center,location)
        # use pre calculated line equations to shift to favourable angles later
        dx = location[0] - self.bot_center[0]
        if dx > 20:
            shifted_location = (self.bot_center[0], self.bot_center[1] + dy)
            self.orient_and_move_to_location(shifted_location,bot_location_angle)
        else:
            shifted_location = (self.bot_center[0]+15, self.bot_center[1] - dy)
            self.orient_and_move_to_location(shifted_location)
    # Orient bot to location and move to the location
    def orient_and_move_to_location(self,location,bot_location_angle):
        print(f"Moving to location {location}")
        bot_location_angle, bot_angle = self.adjusted_bot_angle(bot_location_angle)
        angle_differnce =bot_location_angle - bot_angle
        # Rotate the bot to align with the location position
        if abs(angle_differnce) > 10:
            self.orient_bot(angle_differnce)
        else:
            # if bot is in same aligment as location go forward
            # Adjust the scale based on distance
            self.move_forward(self.previous_distance_scale)
    def move_forward(self,distance):
        print(f"Moving forward {distance}")
        movement_command = 'f' + self.decaToHex(self.previous_distance_scale)
        self.bot_command = movement_command
    def orient_bot(self,angle_differnce):
        rotation_direction = self.get_rotation_direction(angle_differnce)
        # Adjust the scale based on the difference in angle
        movement_command = rotation_direction + self.decaToHex(self.previous_rotation_scale)
        self.bot_command = movement_command
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

