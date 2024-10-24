# pathfinder tes
import numpy as np
from scipy.spatial import distance as dist
import cv2


def get_paths(contours, start, end):
    # get le
    intersection_points = []
    # Get the line equation of the start and end points
    line_eq = line_equation(start,end)
    sorted_map ={ }
    # Loop through the image and get the path
    for contour in contours:
        # get contoour radius
        (x,y),radius = cv2.minEnclosingCircle(contour)
        centroid = (int(x),int(y))
        # Get the intersection points
        points = line_circle_intersection(line_eq[0], line_eq[1], centroid, radius)
        if points is None:
            continue
        print('Intersection happened')
        print(radius)
        start_new = points[0]
        end_new = points[1]
        distance = np.linalg.norm(np.array(centroid) - np.array(start))
        p1Dist = np.linalg.norm(np.array(start_new) - np.array(start))
        p2Dist = np.linalg.norm(np.array(end_new) - np.array(start))
        if p1Dist > p2Dist:
            start_new,end_new = end_new,start_new
            
        
        start_new = (int(start_new[0]),int(start_new[1]))
        end_new = (int(end_new[0]),int(end_new[1]))
        intersection_points.append((start_new,end_new,distance))
        # make start_new and end_new ints values
        print(f"start_new: {start_new}, end_new: {end_new}, distance: {distance}")

    # Sort the intersection points
    intersection_points.sort(key=lambda x: x[2])

    return intersection_points

# Example usage:
# centroid_list = [(x1, y1), (x2, y2), ...]  # Replace with actual centroids
# start = (start_x, start_y)  # Replace with actual start coordinates
# end = (end_x, end_y)  # Replace with actual end coordinates
# result = get_paths(centroid_list, start, end)
# print(result)

def line_equation(start,end):
    # get the line equation of the start and end points
    m = (end[1] - start[1])/(end[0] - start[0])
    c = start[1] - m*start[0]
    return m,c

# method to check if a line passe through a circle
def line_circle_intersection(m, c, center, r):
    # Calculate the quadratic coefficients
    A = 1 + m**2
    B = -2 * center[0] + 2 * m * (c - center[1])
    C = center[0]**2 + (c - center[1])**2 - r**2

    # Calculate the discriminant
    discriminant = B**2 - 4 * A * C

    if discriminant < 0:
        return None  # No intersection

    # Calculate the x values of the intersection points
    sqrt_discriminant = np.sqrt(discriminant)
    x1 = (-B + sqrt_discriminant) / (2 * A)
    x2 = (-B - sqrt_discriminant) / (2 * A)

    # Calculate the corresponding y values
    y1 = m * x1 + c
    y2 = m * x2 + c

    return (x1, y1), (x2, y2)
