import numpy as np
from scipy.cluster.hierarchy import centroid


def selectBall(objList, x_boundaries, y_boundaries, bot_centroid,goal_location):
    min_distance = float('inf')
    corner_balls = []
    selected_ball = None
    centroids=[]
    centroids = np.array([obj.centroid for obj in objList.values()])
    bot_centroid_array = np.array(bot_centroid)
    distances = np.linalg.norm(centroids - bot_centroid_array, axis=1)

    for i, objId in enumerate(objList):
        obj = objList[objId]
        centroid = obj.centroid
        check_bot_in_between = check_bot_in_between(bot_centroid, centroid, goal_location)
        if check_bot_in_between:
            continue
        # if x_boundaries[0] < centroid[0] < x_boundaries[1] and y_boundaries[0] < centroid[1] < y_boundaries[1]:
        dist = distances[i]
        if dist < min_distance:
            min_distance = dist
            selected_ball = obj
    # else:
    #     corner_balls.append(obj)

    return selected_ball, corner_balls
def check_bot_in_between(self, bot_location, selected_centroid):
    if goal_location == 'left':
        if bot_location[0] < selected_centroid[0]:
            return True
    else:
        if bot_location[0] > selected_centroid[0]:
            return True
    return False