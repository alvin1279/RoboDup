import math
from collections import deque

class TrackedObject:
    def __init__(self, objectID, centroid, bounding_rectangle, maxHistory=5):
        # Assign an ID to the object and initialize properties
        self.objectID = objectID
        self.bounding_rectangle = bounding_rectangle
        self.centroid = centroid
        self.centroid_history = deque([centroid], maxlen=maxHistory)  # Using deque for centroid history
        self.maxHistory = maxHistory
        self.disappeared = 0
        self.average_centroid = centroid
        self.displacement = 0
        self.direction = 0
        self.vector = (0, 0)
        self.vector_history = deque([(0, 0)], maxlen=maxHistory)  # Using deque for vector history
        self.displacement_history = deque([0], maxlen=maxHistory)  # Using deque for displacement history

    def update_centroid(self, new_centroid, new_bounding_rectangle):
        # Update the current centroid and maintain centroid history
        self.update_movements(new_centroid)
        self.centroid_history.append(new_centroid)
        self.bounding_rectangle = new_bounding_rectangle

    def mark_disappeared(self):
        # Increase the disappeared count for the object
        self.disappeared += 1

    def reset_disappeared(self):
        # Reset disappeared count when object is visible again
        self.disappeared = 0

    def update_movements(self, new_centroid):
        # Update both vector and smooth it using vector history
        self.centroid = new_centroid
        self.update_vector(new_centroid)
        self.update_displacement()
        self.update_centroid_average()

    def update_centroid_average(self):
        # Compute the average of the centroid history
        self.average_centroid = tuple(map(lambda x: int(sum(x) / len(x)), zip(*self.centroid_history)))

    def update_displacement(self):
        delta_x, delta_y = self.vector
        self.displacement = (delta_x**2 + delta_y**2)**0.5
        self.displacement_history.append(self.displacement)

    def update_vector(self, new_centroid):
        # Calculate the x and y components of the vector
        vector_x = new_centroid[0] - self.average_centroid[0]
        vector_y = new_centroid[1] - self.average_centroid[1]
        self.vector = (vector_x, vector_y)
        # Add the new vector to the vector history and smooth it
        self.vector_history.append(self.vector)
        self.smooth_vector()

    def smooth_vector(self):
        # Compute the average vector from vector history for smoothing
        avg_vector_x = sum(v[0] for v in self.vector_history) / len(self.vector_history)
        avg_vector_y = sum(v[1] for v in self.vector_history) / len(self.vector_history)
        self.vector = (avg_vector_x, avg_vector_y)  # Set the smoothed vector
