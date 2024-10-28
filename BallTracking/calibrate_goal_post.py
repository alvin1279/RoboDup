import cv2
import json
import numpy as np
import tkinter as tk
from tkinter import messagebox

# Global variables to store the mouse click coordinates and points
mouseX, mouseY = 0, 0
selected_point_index = None

def click_vertix(event, x, y, flags, param):
    global mouseX, mouseY, selected_point_index
    if event == cv2.EVENT_LBUTTONDOWN:
        mouseX, mouseY = x, y
        print(f"Mouse clicked at: ({mouseX}, {mouseY})")
        selected_point_index = get_closest_point_index((mouseX, mouseY))
    elif event == cv2.EVENT_MOUSEMOVE and selected_point_index is not None:
        mouseX, mouseY = x, y
        adjust_point((mouseX, mouseY), selected_point_index)
    elif event == cv2.EVENT_LBUTTONUP:
        selected_point_index = None

def get_closest_point_index(click_point):
    min_distance = float('inf')
    closest_index = -1

    for i, point in enumerate(points):
        distance = np.linalg.norm(np.array(point) - np.array(click_point))
        if distance < min_distance:
            min_distance = distance
            closest_index = i

    return closest_index

def adjust_point(new_point, index):
    if index < 2:
        line_points = points[:2]
    else:
        line_points = points[2:]

    closest_point = get_closest_point_on_line(line_points, new_point)
    new_points[index] = closest_point

def get_closest_point_on_line(line_points, point):
    p1, p2 = np.array(line_points[0]), np.array(line_points[1])
    p = np.array(point)
    line_vec = p2 - p1
    point_vec = p - p1
    line_len = np.dot(line_vec, line_vec)
    if line_len == 0:
        return tuple(p1)
    projection = np.dot(point_vec, line_vec) / line_len
    projection = np.clip(projection, 0, 1)
    closest_point = p1 + projection * line_vec
    return tuple(closest_point.astype(int))

def draw_points(img, points, color):
    for i, point in enumerate(points):
        cv2.circle(img, point, 5, color, -1)

def draw_lines(img, points, color):
    for i in range(0, len(points), 2):
        cv2.line(img, points[i], points[i + 1], color, 2)

# Read JSON file
with open('rect.json', 'r') as f:
    data = json.load(f)
    rect_points = data["rect_points"]
    zoom = data["zoom"]

# Load image
img = cv2.imread("Samples/platform1.png")
img = cv2.resize(img, (img.shape[1] * zoom, img.shape[0] * zoom))

# Calculate points on the left and right lines of the rectangle
left_line_points = [rect_points[0], rect_points[3]]
right_line_points = [rect_points[1], rect_points[2]]

# Combine all points
points = left_line_points + right_line_points
new_points = points.copy()  # Create a copy for the new points

cv2.namedWindow('image')
cv2.setMouseCallback('image', click_vertix)

while True:
    imgcpy = img.copy()
    draw_points(imgcpy, points, (0, 255, 0))  # Draw original points in green
    draw_lines(imgcpy, points, (0, 255, 0))  # Draw original lines in green
    draw_points(imgcpy, new_points, (0, 0, 255))  # Draw new points in red
    draw_lines(imgcpy, new_points, (0, 0, 255))  # Draw new lines in red
    cv2.imshow('image', imgcpy)
    k = cv2.waitKey(20) & 0xFF
    if k == 27:
        break

# Save the updated points as goal post points left and right
# turn points to int
new_points = [[int(x) for x in point] for point in new_points]
left_goal_post = new_points[:2]
right_goal_post = new_points[2:]
data = {"left_goal_post": left_goal_post, "right_goal_post": right_goal_post, "zoom": zoom}
print(data)
if messagebox.askyesno("Save Changes", "Do you want to save the updated coordinates?"):
    with open('goal_post_points.json', 'w') as f:
        json.dump(data, f)
    print("Coordinates saved.")
else:
    print("Coordinates not saved.")

cv2.destroyAllWindows()
