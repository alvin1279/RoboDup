import numpy as np

class BallSelector:
    def __init__(self, goal_location, shape, buffer_width=30,):
        self.goal_location = goal_location
        self.shape = shape  # shape as (width, height)
        self.buffer_width = buffer_width  # adjustable buffer width

        # Define x and y boundaries based on shape dimensions
        self.x_boundaries = (0, shape[1])
        self.y_boundaries = (0, shape[0])

        # Zones based on x and y positions
        self.buffer_zone_x = []
        self.buffer_zone_y = []

        # Ball zones based on x positions (relative to bot)
        self.balls_zone_positive_x = []
        self.balls_zone_negative_x = []
        self.selected_ball = None
        self.region = 0

        # flags
        self.already_selected_flag = False
        self.ball_selected_flag = False

        #post
        # self.transformed_left_goal_post = transformed_left_goal_post

    def calculate_angles(self, reference_point, centroids):
        """
        Calculate the angles of each centroid relative to the reference point.
        Angles are computed in radians, then converted to degrees and normalized.
        """
        delta_x = centroids[:, 0] - reference_point[0]
        delta_y = centroids[:, 1] - reference_point[1]
        
        # Calculate angles using arctan2 (in radians)
        angles_rad = np.arctan2(delta_y, delta_x)
        
        # Convert radians to degrees
        angles_deg = np.degrees(angles_rad)
        
        # Normalize the angles to range [0, 360)
        angles_deg = (angles_deg + 360) % 360
        
        return angles_deg

    def update_zones(self, bot_centroid, objects):
        # Clear zones for a new update
        self.buffer_zone_x.clear()
        self.buffer_zone_y.clear()
        self.balls_zone_positive_x.clear()
        self.balls_zone_negative_x.clear()

        # Extract centroids and calculate distances
        object_ids = list(objects.keys())
        centroids = np.array([obj.centroid for obj in objects.values()])
        distances = np.linalg.norm(centroids - bot_centroid, axis=1)

        # Determine which objects are near each boundary
        near_left_boundary = centroids[:, 0] <= self.x_boundaries[0] + self.buffer_width
        near_right_boundary = centroids[:, 0] >= self.x_boundaries[1] - (self.buffer_width +30)
        near_top_boundary = centroids[:, 1] <= self.y_boundaries[0] + self.buffer_width
        near_bottom_boundary = centroids[:, 1] >= self.y_boundaries[1] - self.buffer_width

        # Combine all buffer conditions to get a complete buffer mask
        buffer_mask = near_left_boundary | near_right_boundary | near_top_boundary | near_bottom_boundary

        # Separate objects into buffer and non-buffer zones
        buffer_indices = np.where(buffer_mask)[0]
        non_buffer_indices = np.where(~buffer_mask)[0]

        # Determine objects in x-based ball zones relative to the bot (excluding buffer zones)
        positive_x_indices = non_buffer_indices[centroids[non_buffer_indices, 0] < bot_centroid[0]]
        negative_x_indices = non_buffer_indices[centroids[non_buffer_indices, 0] > bot_centroid[0]]

        # Sort objects in each zone by distance
        sorted_positive_x = positive_x_indices[np.argsort(distances[positive_x_indices])]
        sorted_negative_x = negative_x_indices[np.argsort(distances[negative_x_indices])]

        # Store sorted objects in ball zones
        self.balls_zone_positive_x = [objects[object_ids[i]] for i in sorted_positive_x]
        self.balls_zone_negative_x = [objects[object_ids[i]] for i in sorted_negative_x]

        # Store objects near each boundary in buffer zones
        self.buffer_zone_x = [objects[object_ids[i]] for i in buffer_indices if near_left_boundary[i] or near_right_boundary[i]]
        self.buffer_zone_y = [objects[object_ids[i]] for i in buffer_indices if near_top_boundary[i] or near_bottom_boundary[i]]
    
    def select_ball_non_edge(self):
        # Select ball based on its zone (ignoring buffer zone balls)
        if len(self.balls_zone_positive_x) > 0:
            return 1, self.balls_zone_positive_x[0]
        elif len(self.balls_zone_negative_x) > 0:
            return -1, self.balls_zone_negative_x[0]
        else:
            return 0, None
    def select_ball_non_edge_positive(self):
        self.selected_ball = self.balls_zone_positive_x[0]
        self.already_selected_flag = True
        self.ball_selected_flag = True
        self.region = 1
    def select_ball_non_edge_negative(self):
        self.selected_ball = self.balls_zone_negative_x[0]
        self.already_selected_flag = True
        self.ball_selected_flag = True
        self.region = -1
    # method to check if selected object is in still in the zone
    # if not reset flags
    def check_ball_still_exist(self,objects):
        for ball in objects.values():
            selected_centroid = self.selected_ball.centroid
            distance = np.sqrt((selected_centroid[0] - ball.centroid[0]) ** 2 + (
                        selected_centroid[1] - ball.centroid[1]) ** 2)
            print(distance)
            if distance <10:
                return True
        return False


    def reset_flags(self):
        self.already_selected_flag = False
        self.ball_selected_flag = False
        self.selected_ball = None
        self.region = 0