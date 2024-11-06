import numpy as np
from scipy.cluster.hierarchy import centroid


def selectBall(objList, x_boundaries, y_boundaries, bot_centroid):
    min_distance = float('inf')
    corner_balls = []
    selected_ball = None
    centroids=[]

    centroids = np.array([obj.centroid for obj in objList])
    bot_centroid_array = np.array(bot_centroid)
    distances = np.linalg.norm(centroids - bot_centroid_array, axis=1)

    for i, objId in enumerate(objList):
        obj = objList[objId]
        centroid = obj.centroid
        if x_boundaries[0] < centroid[0] < x_boundaries[1] and y_boundaries[0] < centroid[1] < y_boundaries[1]:
            dist = distances[i]
            if dist < min_distance:
                min_distance = dist
                selected_ball = obj
        else:
            corner_balls.append(obj)

    return selected_ball, corner_balls
