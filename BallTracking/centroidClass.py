from collections import OrderedDict
from scipy.spatial import distance as dist
from TrackedObjectClass import TrackedObject

class CentroidTracker:
    def __init__(self, maxDisappeared=50, maxHistory=5):
        self.nextObjectID = 0
        self.objects = OrderedDict()  # Holds the TrackedObject instances
        self.ObjectCentroids = OrderedDict()
        self.maxDisappeared = maxDisappeared
        self.maxHistory = maxHistory

    def register(self, centroid):
        # Create a new tracked object and assign it the next available object ID
        obj = TrackedObject(self.nextObjectID, centroid, maxHistory=self.maxHistory)
        self.objects[self.nextObjectID] = obj
        self.ObjectCentroids[self.nextObjectID] = centroid
        self.nextObjectID += 1

    def deregister(self, objectID):
        # Remove the object from tracking
        del self.objects[objectID]
        del self.ObjectCentroids[objectID]

    def update(self, inputCentroids):
        # If no centroids are detected, mark existing objects as disappeared
        if len(inputCentroids) == 0:
            for objectID in list(self.objects.keys()):
                obj = self.objects[objectID]
                obj.mark_disappeared()

                if obj.disappeared > self.maxDisappeared:
                    self.deregister(objectID)

            return self.get_all_objects()

        # If no objects are currently tracked, register all centroids as new objects
        if len(self.objects) == 0:
            for centroid in inputCentroids:
                self.register(centroid)

        else:
            # Grab the current object IDs and their corresponding centroids
            objectIDs = list(self.objects.keys())
            objectCentroids = [obj.centroid for obj in self.objects.values()]

            # Compute the distance between each pair of object centroids and input centroids
            distances = dist.cdist(objectCentroids, inputCentroids)

            # Find the smallest value in each row (min distances), and then sort rows by the min values
            rows = distances.min(axis=1).argsort()
            cols = distances.argmin(axis=1)[rows]

            # Find the smallest value in each row (min distances), and then sort rows by the min values
            rows = distances.min(axis=1).argsort()
            cols = distances.argmin(axis=1)[rows]

            usedRows = set()
            usedCols = set()

            # Use ThreadPoolExecutor for parallel updating of objects

            for (row, col) in zip(rows, cols):
                if row in usedRows or col in usedCols:
                    continue

                objectID = objectIDs[row]
                obj = self.objects[objectID]
                self.update_object(obj, inputCentroids[col])

                usedRows.add(row)
                usedCols.add(col)


            # Register new input centroids as new objects
            for col in range(len(inputCentroids)):
                if col not in usedCols:
                    self.register(inputCentroids[col])

            # Mark remaining objects as disappeared
            for row in range(len(objectCentroids)):
                if row not in usedRows:
                    objectID = objectIDs[row]
                    obj = self.objects[objectID]
                    obj.mark_disappeared()

                    if obj.disappeared > self.maxDisappeared:
                        self.deregister(objectID)

        return self.get_all_objects()

    def update_object(self, obj, new_centroid):
        # Update the object with its new centroid and movement
        obj.update_centroid(new_centroid)
        obj.reset_disappeared()  # Reset the disappearance count when the object is re-associated

    def get_all_objects(self):
        # Return current objects and their average centroids
        objects = {objectID: obj for objectID, obj in self.objects.items()}
        return objects