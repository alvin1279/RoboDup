import numpy as np
import path_finder as pf
from collections import deque
import websocket


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

        self.current_target = None
        self.path = {'intermediate':None,'final':None}

        self.previous_rotation_scale = 1
        self.previous_distance_scale = 2
        # flags
        self.target_reached = False
        self.intersection = False
        self.near_target = False
        self.bot_quadrant = None
        self.y_channel_found = False
        self.bot_near_boundary = False

    def update_bot_data(self, bot_data):
        tail_centroid, head_centroid, angle = bot_data
        self.bot_data = bot_data
        self.tail_centroid = tail_centroid
        self.head_centroid = head_centroid
        self.angle = self.normalize_angle(angle)
        self.bot_center = ((tail_centroid[0] + head_centroid[0]) // 2, (tail_centroid[1] + head_centroid[1]) // 2)
        
        # Determine the hemisphere of the bot
        self.bot_hemisphere = 'top' if self.bot_center[1] > self.shape[1] // 2 else 'bottom'
        self.bot_quadrant = self.determine_quadrant(self.angle)
        self.bot_near_boundary = self.near_boundary()
    # method to check if bot is 10 near any of the boundaries
    def near_boundary(self):
        return (self.bot_center[0] <= self.x_boundaries[0] + 10 or
                self.bot_center[0] >= self.x_boundaries[1] - 10 or
                self.bot_center[1] <= self.y_boundaries[0] + 10 or
                self.bot_center[1] >= self.y_boundaries[1] - 10)


    def reset_flags(self):
        self.target_reached = False
        self.intersection = False
        self.near_target = False
        self.y_channel_found = False
        self.path = {'intermediate':None,'final':None}

    def get_y_channel_midpoint(self, all_objects):
        # Width of the y channel to look for
        y_channel_width = 10
        bot_y = self.bot_center[1]

        # Extract image height from the shape
        image_height = self.shape[0]

        # Get all y-coordinates of object centroids
        y_positions = sorted(obj.centroid[1] for obj in all_objects)

        # Include image boundaries (0 and image height) as additional points
        y_positions = [0] + y_positions + [image_height]

        # Initialize variables to track the largest gap
        largest_gap_size = 0
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

                # If this gap is larger than previous gaps, update the largest gap
                if gap_size > largest_gap_size:
                    largest_gap_size = gap_size
                    channel_mid_point = gap_center

        # Return the y-coordinate of the midpoint of the largest empty y channel found
        return int(channel_mid_point)

    def get_shifted_location(self,location):
        # use pre calculated line equations to shift to favourable angles later
        # 10 pixel is 2 cm
        return (location[0]+ 160, location[1])

    def move_to_location(self,location):
        bot_location_angle = self.get_bot_location_angle(location)
        adjusted_bot_angle, bot_angle = self.adjusted_bot_angle(bot_location_angle)
        angle_differnce = adjusted_bot_angle - bot_angle
        if abs(angle_differnce) > 10:
            direction = self.get_rotation_direction(angle_differnce)
            self.bot_command = direction + self.decaToHex(self.previous_rotation_scale)
        else:
            # distance = self.get_distance(self.bot_center,location)
            self.move_forward()
        print('command',self.bot_command)
    # Orient bot to location and move to the location
    def move_forward(self):
        # print(f"Moving forward {distance}")
        movement_command = 'f' + '03'
        if self.current_target == 'final':
            movement_command = 'F' + '0f'
        self.bot_command = movement_command
    def get_rotation_direction(self, angle_differnce):
        if angle_differnce > 0:
            return 'r'
        else: 
            return 'l'     
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

