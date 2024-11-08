import numpy as np

class BallSelector:
    def __init__(self, x_boundaries, y_boundaries, goal_location):
        self.x_boundaries = x_boundaries
        self.y_boundaries = y_boundaries
        self.goal_location = goal_location

    def select_ball(self, obj_list, bot_centroid):
        min_distance = float('inf')
        corner_balls = []
        selected_ball = None
        centroids = np.array([obj.centroid for obj in obj_list.values()])
        bot_centroid_array = np.array(bot_centroid)
        distances = np.linalg.norm(centroids - bot_centroid_array, axis=1)

        for i, obj_id in enumerate(obj_list):
            obj = obj_list[obj_id]
            centroid = obj.centroid
            if self.check_bot_in_between(bot_centroid, centroid):
                continue
            dist = distances[i]
            if dist < min_distance:
                min_distance = dist
                selected_ball = obj

        return selected_ball, corner_balls

    def check_bot_in_between(self, bot_location, selected_centroid):
        if self.goal_location == 'left':
            if bot_location[0] < selected_centroid[0]:
                return True
        else:
            if bot_location[0] > selected_centroid[0]:
                return True
        return False