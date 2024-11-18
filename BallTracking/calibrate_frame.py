import json
import cv2
import numpy as np
import imutils
import os

# Global variables to store mouse coordinates and points
mouseX, mouseY = 0, 0
selected_point_index = None
points, new_points = [], []
# Global variables to save updated points
updated_rect_points = []
updated_goal_line_points = []
# Colors
red = (0, 0, 255)
blue = (255, 0, 0)
green = (0, 255, 0)

# Mouse click handlers
def click_vertix_for_field(event, x, y, flags, param):
    global mouseX, mouseY, selected_point_index
    if event == cv2.EVENT_LBUTTONDOWN:
        mouseX, mouseY = x, y
        selected_point_index = get_closest_point_index((mouseX, mouseY))
    elif event == cv2.EVENT_MOUSEMOVE and selected_point_index is not None:
        mouseX, mouseY = x, y
        adjust_point_for_field((mouseX, mouseY), selected_point_index)
    elif event == cv2.EVENT_LBUTTONUP:
        selected_point_index = None

def click_vertix_for_goal(event, x, y, flags, param):
    global mouseX, mouseY, selected_point_index
    if event == cv2.EVENT_LBUTTONDOWN:
        mouseX, mouseY = x, y
        selected_point_index = get_closest_point_index((mouseX, mouseY))
    elif event == cv2.EVENT_MOUSEMOVE and selected_point_index is not None:
        mouseX, mouseY = x, y
        adjust_point_for_goal((mouseX, mouseY), selected_point_index)
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

def adjust_point_for_field(new_point, index):
    points[index] = new_point
    global updated_rect_points
    updated_rect_points = points.copy()  # Save updated points

def adjust_point_for_goal(new_point, index):
    if index < 2:
        line_points = points[:2]
    else:
        line_points = points[2:]

    closest_point = get_closest_point_on_line(line_points, new_point)
    new_points[index] = closest_point
    global updated_goal_line_points
    updated_goal_line_points = new_points.copy()  # Save updated goal points

def get_closest_point_on_line(line_points, point):
    p1, p2 = np.array(line_points[0]), np.array(line_points[1])
    line_vec = p2 - p1
    point_vec = np.array(point) - p1
    line_len = np.dot(line_vec, line_vec)
    if line_len == 0:
        return tuple(p1)
    projection = np.dot(point_vec, line_vec) / line_len
    projection = np.clip(projection, 0, 1)
    closest_point = p1 + projection * line_vec
    return tuple(closest_point.astype(int))

def draw_points(img, points, color=(0, 255, 0)):
    for point in points:
        cv2.circle(img, point, 5, color, -1)

def draw_goal_lines(img, goal_line_points, color=(0, 0, 255)):
    cv2.line(img, goal_line_points[0], goal_line_points[1], color, 2)

def draw_lines_rectangle(img, points):
    colors = [(0, 255, 0), (0, 0, 255)]
    for i in range(4):
        cv2.line(img, points[i], points[(i + 1) % 4], colors[i // 2], 2)


# Open the video file
video_path = 'Samples/field_1.mp4'  # Change to your video file path
cap = cv2.VideoCapture('http://192.168.104.247:8080//video')

# cap = cv2.VideoCapture(video_path)
file_path='Samples/resized_rotated_frame.png'
redux = 2
# Check if the video opened successfully
if not cap.isOpened():
    print("Error: Could not open video.")
else:
    # Read a single frame
    ret, frame = cap.read()  # ret is a boolean indicating success
    frame = imutils.resize(frame, width=1200)
    if ret:
        captured_frame = frame
        cv2.imshow('Captured Frame', captured_frame)

        # rotated_frame = cv2.rotate(captured_frame, cv2.ROTATE_90_CLOCKWISE)
        # Resize the rotated frame (e.g., to half the original size)
        # height, width = rotated_frame.shape[:2]
        # resized_frame = cv2.resize(rotated_frame, (width // redux, height // redux))

        # Show the rotated and resized frame
        cv2.imshow('Rotated and Resized Frame', captured_frame)
        cv2.waitKey(0)  # Wait until a key is pressed

        cv2.imwrite(file_path, captured_frame)
    else:
        print("Error: Could not read frame from video.")

# Release the video capture object
cap.release()
cv2.destroyAllWindows()

img = cv2.imread(file_path)
cv2.namedWindow('image')
cv2.setMouseCallback('image', click_vertix_for_field)

# Threshold parameters
lower, upper = (52, 21, 68), (138, 205, 243)
filter_radius = 1

# Convert image to HSV and filter
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, lower, upper)
mask = cv2.dilate(mask, None, iterations=3)
median = cv2.medianBlur(mask, filter_radius)

# Detect contours
contours = cv2.findContours(median, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(contours)

if cnts:  # Check if contours exist
    max_cnt = max(cnts, key=cv2.contourArea)

    # Get rectangle points
    x, y, w, h = cv2.boundingRect(max_cnt)
    rect_points = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
    points = rect_points.copy()

    while True:
        imgcpy = img.copy()
        draw_points(imgcpy, points)
        draw_lines_rectangle(imgcpy, points)
        cv2.imshow('image', imgcpy)
        k = cv2.waitKey(20) & 0xFF
        if k == 27:
            break
        elif k == ord('a'):
            print(f"Mouse coordinates: ({mouseX}, {mouseY})")
            print(f"Updated rectangle points: {updated_rect_points}")

    # Set goal line points
    left_line_points = [updated_rect_points[0], updated_rect_points[3]]
    right_line_points = [updated_rect_points[1], updated_rect_points[2]]
    points = left_line_points + right_line_points
    new_points = points.copy()
    cv2.setMouseCallback('image', click_vertix_for_goal)
    updated_goal_line_points = left_line_points + right_line_points

    while True:
        imgcpy = img.copy()
        draw_points(imgcpy, new_points, color=red)  # Draw new points in red
        draw_goal_lines(imgcpy, left_line_points)  # Draw original lines in green
        draw_goal_lines(imgcpy, right_line_points)  # Draw original lines in green
        draw_goal_lines(imgcpy, updated_goal_line_points[:2], blue)  # Draw new lines in blue
        draw_goal_lines(imgcpy, updated_goal_line_points[2:], blue)  # Draw new lines in blue
        cv2.imshow('image', imgcpy)
        k = cv2.waitKey(20) & 0xFF
        if k == 27:
            break
        elif k == ord('a'):
            print(f"Mouse coordinates: ({mouseX}, {mouseY})")
            print(f"Updated goal line points: {updated_goal_line_points}")
    # Drawing final image calibrated
    # De-scale all the points based on the zoom factor
    rect_points = [[int(x) for x in point] for point in updated_rect_points]
    left_goal_post = [[int(x) for x in point] for point in updated_goal_line_points[:2]]
    right_goal_post = [[int(x) for x in point] for point in updated_goal_line_points[2:]]

    for i in range(4):
        cv2.line(img, tuple(rect_points[i]), tuple(rect_points[(i + 1) % 4]), (0, 255, 0), 2)

    # Draw goal post lines in red
    cv2.line(img, tuple(left_goal_post[0]), tuple(left_goal_post[1]), (0, 0, 255), 2)
    cv2.line(img, tuple(right_goal_post[0]), tuple(right_goal_post[1]), (0, 0, 255), 2)
    cv2.imshow('image', img)
    cv2.waitKey(0)

    cv2.destroyAllWindows()

    img = cv2.imread(file_path)

    # Define the source points (the four corner points of your region)
    src_points = np.float32(rect_points)

    # Define the destination points, which form a rectangle
    # Calculate the width and height for the output based on source points
    width = int(max(np.linalg.norm(src_points[1] - src_points[0]), np.linalg.norm(src_points[3] - src_points[2])))
    height = int(max(np.linalg.norm(src_points[2] - src_points[1]), np.linalg.norm(src_points[3] - src_points[0])))
    dst_points = np.float32([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]])

    # Compute the perspective transform matrix
    M = cv2.getPerspectiveTransform(src_points, dst_points)

    # Apply the perspective warp to get the cropped and un-skewed image
    warped = cv2.warpPerspective(img, M, (width, height))

    # Print points and zoom
    print("Rectangle Points:", rect_points)
    print("redux:", redux)
    print("Left Goal Post Points:", left_goal_post)
    print("Right Goal Post Points:", right_goal_post)

    # Apply the warp matrix to these points
    transformed_left_goal_post = cv2.perspectiveTransform(np.array(left_goal_post, dtype=np.float32)[None, :, :], M)[0]
    transformed_right_goal_post = cv2.perspectiveTransform(np.array(right_goal_post, dtype=np.float32)[None, :, :], M)[0]
    # Save the updated points as goal post points left and right
    # turn points to int
    data = {
        "transformed_left_goal_post": transformed_left_goal_post.tolist(),
        "transformed_right_goal_post": transformed_right_goal_post.tolist(),
        "redux": redux,
        "warp_matrix": M.tolist(),
        "shape": warped.shape,
        "width": width,
        "height": height
    }

    print(data)
    # save to json
    user_input = input("Save this Data points to file? (yes/no): ").strip().lower()
    if user_input == "yes":
        os.makedirs('Datas', exist_ok=True)
        with open('Datas/final_warped.json', 'w') as f:
            json.dump(data, f)
        # Display the transformed goal post points
    print("Transformed Left Goal Post Points:", transformed_left_goal_post)
    print("Transformed Right Goal Post Points:", transformed_right_goal_post)

    # Optional: Draw these points on the warped image to verify
    for transformed_point in transformed_left_goal_post:  # Correct variable name
        cv2.circle(warped, tuple(transformed_point.astype(int)), 5, green, -1)
    for transformed_point in transformed_right_goal_post:  # Correct variable name
        cv2.circle(warped, tuple(transformed_point.astype(int)), 5, red, -1)

    cv2.imshow('warped', warped)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("No contours found in the image.")

