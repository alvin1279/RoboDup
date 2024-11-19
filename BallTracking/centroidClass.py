from collections import OrderedDict
from scipy.spatial import distance as dist
from TrackedObjectClass import TrackedObject
import numpy as np

class CentroidTracker:
    def __init__(self, maxDisappeared=50, maxDistance=50):
        self.nextObjectID = 0
        self.objects = {}
        self.disappeared = {}
        self.maxDisappeared = maxDisappeared
        self.maxDistance = maxDistance

    def register(self, centroid, bounding_rectangle):
        self.objects[self.nextObjectID] = TrackedObject(self.nextObjectID, centroid, bounding_rectangle)
        self.disappeared[self.nextObjectID] = 0
        self.nextObjectID += 1

    def deregister(self, objectID):
        del self.objects[objectID]
        del self.disappeared[objectID]

    def update(self, rects):
        if len(rects) == 0:
            for objectID in list(self.disappeared.keys()):
                self.disappeared[objectID] += 1
                if self.disappeared[objectID] > self.maxDisappeared:
                    self.deregister(objectID)
            return self.objects

        inputCentroids = np.zeros((len(rects), 2), dtype="int")
        bounding_rectangles = []

        for (i, (startX, startY, width, hight)) in enumerate(rects):
            cX = int(startX + width/2)
            cY = int(startY + hight/2)
            inputCentroids[i] = (cX, cY)
            bounding_rectangles.append((startX, startY, width, hight))

        if len(self.objects) == 0:
            for i in range(0, len(inputCentroids)):
                self.register(inputCentroids[i], bounding_rectangles[i])
        else:
            objectIDs = list(self.objects.keys())
            objectCentroids = [obj.centroid for obj in self.objects.values()]

            D = dist.cdist(np.array(objectCentroids), inputCentroids)

            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]

            usedRows = set()
            usedCols = set()

            # Loop over the combination of the (row, col) index tuples
            for (row, col) in zip(rows, cols):
                # If we have already examined either the row or column value before, ignore it
                if row in usedRows or col in usedCols:
                    continue

                # Get the object ID for the current row
                objectID = objectIDs[row]
                # Update the centroid and bounding rectangle for the current object
                self.objects[objectID].update_centroid(inputCentroids[col], bounding_rectangles[col])
                # Reset the disappeared counter for the current object
                self.disappeared[objectID] = 0

                # Mark the row and column as used
                usedRows.add(row)
                usedCols.add(col)

            # Find the unused row and column indices
            unusedRows = set(range(0, D.shape[0])).difference(usedRows)
            unusedCols = set(range(0, D.shape[1])).difference(usedCols)

            # Loop over the unused row indices
            for row in unusedRows:
                # Get the object ID for the current row
                objectID = objectIDs[row]
                # Increment the disappeared counter for the current object
                self.disappeared[objectID] += 1
                # If the disappeared counter exceeds the maximum allowed, deregister the object
                if self.disappeared[objectID] > self.maxDisappeared:
                    self.deregister(objectID)

            # Loop over the unused column indices
            for col in unusedCols:
                # Register a new object with the centroid and bounding rectangle
                self.register(inputCentroids[col], bounding_rectangles[col])

        # Return the set of tracked objects
        return self.objects