import json
import cv2
import numpy as np
import imutils
import tkinter as tk
from tkinter import messagebox

# Global variables to store the mouse click coordinates and points
mouseX, mouseY = 0, 0
selected_point_index = None
points = []

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
    points[index] = new_point

def draw_points(img, points):
    for i, point in enumerate(points):
        color = (0, 255, 0) if i < 2 else (0, 0, 255)  # Green for left points, Red for right points
        cv2.circle(img, point, 2, color, -1)

def draw_lines(img, points):
    red = (0, 0, 255)
    green = (0, 255, 0)
    # Green for left and right lines
    cv2.line(img, points[0], points[1], red, 2)
    cv2.line(img, points[2], points[3], red, 2)
    # Red for top and bottom lines
    cv2.line(img, points[1], points[2],green, 2)
    cv2.line(img, points[3], points[0],green, 2)

# Load image
img = cv2.imread("Samples/platform1.png")
zoom = 2
img = cv2.resize(img, (img.shape[1] * zoom, img.shape[0] * zoom))
cv2.namedWindow('image')
cv2.setMouseCallback('image', click_vertix)

lower = (52, 21, 68)
upper = (138, 205, 243)
filter_radius = 1

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, lower, upper)
mask = cv2.dilate(mask, None, iterations=3)

# Use a median filter to get rid of speckle noise
median = cv2.medianBlur(mask, filter_radius)
contours = cv2.findContours(median, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(contours)

max_area = 0
max_cnt = None
for c in cnts:
    if cv2.contourArea(c) > max_area:
        max_area = cv2.contourArea(c)
        max_cnt = c

x, y, w, h = cv2.boundingRect(max_cnt)
rect_points = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]

points = rect_points

while True:
    imgcpy = img.copy()
    draw_points(imgcpy, points)
    draw_lines(imgcpy, points)
    cv2.imshow('image', imgcpy)
    k = cv2.waitKey(20) & 0xFF
    if k == 27:
        break
    elif k == ord('a'):
        print(f"Mouse coordinates: ({mouseX}, {mouseY})")

# write the output cords and zoom to a json file
output_data = {
    "rect_points": rect_points,
    "zoom": zoom
}
print(output_data)
root = tk.Tk()
root.withdraw()  # Hide the main window

if messagebox.askyesno("Save Changes", "Do you want to save the updated coordinates?"):
    with open('rect.json', 'w') as f:
        json.dump(output_data, f)
    print("Coordinates saved.")
else:
    print("Coordinates not saved.")

cv2.destroyAllWindows()
