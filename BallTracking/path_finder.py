import numpy as np
from scipy.spatial import distance as dist
import cv2

def get_paths(contours, start, end):
    intersection_points = []
    line_eq = line_equation(start, end)
    sorted_map = {}
    for contour in contours:
        (x, y), radius = cv2.minEnclosingCircle(contour)
        centroid = (int(x), int(y))
        points = line_circle_intersection(line_eq[0], line_eq[1], centroid, radius)
        if points is None:
            continue
        print('Intersection happened')
        # print(radius)
        start_new = points[0]
        end_new = points[1]
        distance = np.linalg.norm(np.array(centroid) - np.array(start))
        p1Dist = np.linalg.norm(np.array(start_new) - np.array(start))
        p2Dist = np.linalg.norm(np.array(end_new) - np.array(start))
        if p1Dist > p2Dist:
            start_new, end_new = end_new, start_new

        start_new = (int(start_new[0]), int(start_new[1]))
        end_new = (int(end_new[0]), int(end_new[1]))
        intersection_points.append((start_new, end_new, distance))
        print(f"start_new: {start_new}, end_new: {end_new}, distance: {distance}")

    intersection_points.sort(key=lambda x: x[2])
    return intersection_points

def line_equation(start, end):
    if end[0] == start[0]:  # Vertical line
        return None, start[0]
    m = (end[1] - start[1]) / (end[0] - start[0])
    c = start[1] - m * start[0]
    return m, c

def line_circle_intersection(m, c, center, r):
    if m is None:  # Vertical line
        x = c
        y1 = center[1] + np.sqrt(r**2 - (x - center[0])**2)
        y2 = center[1] - np.sqrt(r**2 - (x - center[0])**2)
        return (x, y1), (x, y2)

    A = 1 + m**2
    B = -2 * center[0] + 2 * m * (c - center[1])
    C = center[0]**2 + (c - center[1])**2 - r**2
    discriminant = B**2 - 4 * A * C

    if discriminant < 0:
        return None

    sqrt_discriminant = np.sqrt(discriminant)
    x1 = (-B + sqrt_discriminant) / (2 * A)
    x2 = (-B - sqrt_discriminant) / (2 * A)
    y1 = m * x1 + c
    y2 = m * x2 + c

    return (x1, y1), (x2, y2)
