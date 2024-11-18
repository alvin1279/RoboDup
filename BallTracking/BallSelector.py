import numpy as np

class BallSelector:
    def __init__(self, goal_location, shape, buffer_width=20):
        self.goal_location = goal_location
        self.shape = shape  # shape as (width, height)
        self.buffer_width = buffer_width  # adjustable buffer width

        # Define x and y boundaries based on shape dimensions
        self.x_boundaries = (0, shape[0])
        self.y_boundaries = (0, shape[1])

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
        centroids = np.array([obj.centroid for obj in objects.values()])

        distances = np.linalg.norm(centroids - bot_centroid, axis=1)

        print(centroids)
        print(self.x_boundaries)
        # Calculate normalized angles between bot and each object
        angles_deg = self.calculate_angles(bot_centroid, centroids)

        # Determine which objects are in buffer zones near each boundary
        # Determine which objects are in buffer zones near each boundary
        near_left_boundary = centroids[:, 0] <= self.x_boundaries[0] + self.buffer_width
        near_right_boundary = centroids[:, 0] >= self.x_boundaries[1] - self.buffer_width
        near_top_boundary = centroids[:, 1] <= self.y_boundaries[0] + self.buffer_width
        near_bottom_boundary = centroids[:, 1] >= self.y_boundaries[1] - self.buffer_width

        # Combine all buffer conditions to get a complete buffer mask
        buffer_mask = near_left_boundary | near_right_boundary | near_top_boundary | near_bottom_boundary

        # Filter out buffer objects
        non_buffer_centroids = centroids[~buffer_mask]
        non_buffer_distances = distances[~buffer_mask]
        non_buffer_angles = angles_deg[~buffer_mask]

        # Determine objects in x-based ball zones relative to the bot (excluding buffer zones)
        positive_x_mask = non_buffer_centroids[:, 0] > bot_centroid[0]
        negative_x_mask = non_buffer_centroids[:, 0] < bot_centroid[0]

        # Sort objects in each zone by distance and angle, only for non-buffer objects
        pos_x_indices = np.argsort(non_buffer_distances[positive_x_mask])
        neg_x_indices = np.argsort(non_buffer_distances[negative_x_mask])

        # Store sorted objects in ball zones (excluding buffer zones)
        self.balls_zone_positive_x = [(objects[list(objects.keys())[i]], non_buffer_distances[i], non_buffer_angles[i]) for i in pos_x_indices]
        self.balls_zone_negative_x = [(objects[list(objects.keys())[i]], non_buffer_distances[i], non_buffer_angles[i]) for i in neg_x_indices]

        # Store objects near each boundary in buffer zones
        self.buffer_zone_x = [(objects[list(objects.keys())[i]], distances[i], angles_deg[i]) for i in np.where(near_left_boundary | near_right_boundary)[0]]
        self.buffer_zone_y = [(objects[list(objects.keys())[i]], distances[i], angles_deg[i]) for i in np.where(near_top_boundary | near_bottom_boundary)[0]]

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