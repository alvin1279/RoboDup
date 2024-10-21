# Centroid Tracker Class
from scipy.spatial import distance as dist
from collections import OrderedDict


class CentroidTracker:
    def __init__(self, maxDisappeared=50):
        # Initialize the next object ID to be assigned
        self.nextObjectID = 0
        # A dictionary to map object IDs to centroids
        self.objects = OrderedDict()
        # A dictionary to track how long an object has been missing
        self.disappeared = OrderedDict()
        # Max number of frames object can be missing before removal
        self.maxDisappeared = maxDisappeared

    def register(self, centroid):
        # when registering an object we use the next available object
        # ID to store the centroid
        self.objects[self.nextObjectID] = centroid
        self.disappeared[self.nextObjectID] = 0
        self.nextObjectID += 1

    def deregister(self, objectID):
        # Remove the object ID from tracking
        del self.objects[objectID]
        del self.disappeared[objectID]

    def update(self, inputCentroids):
        # If no centroids are detected, mark existing objects as missing
        if len(inputCentroids) == 0:
            for objectID in list(self.objects.keys()):
                self.disappeared[objectID] += 1
                if self.disappeared[objectID] > self.maxDisappeared:
                    self.deregister(objectID)
            return self.objects

        # If there are no existing objects, register new centroids
        if len(self.objects) == 0:
            for i in range(0, len(inputCentroids)):
                self.register(inputCentroids[i])

        else:
            # Calculate distances between new centroids and existing objects
            objectIDs = list(self.objects.keys())
            objectCentroids = list(self.objects.values())
            distances = dist.cdist(objectCentroids,inputCentroids)

            # Find minimum distance pairs (objects to centroids)
             

            # Mark matched objects and update their centroids
            usedRows = set()
            usedCols = set()

            for (row, col) in zip(rows, cols):
                if row in usedRows or col in usedCols:
                    continue

                objectID = objectIDs[row]
                self.objects[objectID] = inputCentroids[col]
                self.disappeared[objectID] = 0

                usedRows.add(row)
                usedCols.add(col)

            # Register new centroids that don't match existing objects
            for col in range(0, len(inputCentroids)):
                if col not in usedCols:
                    self.register(inputCentroids[col])

            # Deregister objects that are no longer detected
            for row in range(0, len(objectCentroids)):
                if row not in usedRows:
                    objectID = objectIDs[row]
                    self.disappeared[objectID] += 1
                    if self.disappeared[objectID] > self.maxDisappeared:
                        self.deregister(objectID)

        return self.objects