import cv2
import json

zoom = 2
# Load the rectangle points and zoom from rect.json
with open('rect.json', 'r') as f:
    rect_data = json.load(f)
    rect_points = rect_data["rect_points"]
    zoom = rect_data["zoom"]

# Load the goal post points from goal_post_points.json
with open('goal_post_points.json', 'r') as f:
    goal_post_data = json.load(f)
    left_goal_post = goal_post_data["left_goal_post"]
    right_goal_post = goal_post_data["right_goal_post"]

# Save all the points and zoom in a new json file
data = {"rect_points": rect_points, "left_goal_post": left_goal_post, "right_goal_post": right_goal_post, "zoom": zoom}
with open('field_boundary.json', 'w') as f:
    json.dump(data, f)

img = cv2.imread('Samples/platform1.png')

# de scale all the points
rect_points = [[int(x / zoom) for x in point] for point in rect_points]
left_goal_post = [[int(x / zoom) for x in point] for point in left_goal_post]
right_goal_post = [[int(x / zoom) for x in point] for point in right_goal_post]
print("Rectangle Points:", rect_points)
print("Zoom:", zoom)
print("Left Goal Post Points:", left_goal_post)
print("Right Goal Post Points:", right_goal_post)


# draw all these line on the image
# draw rectangle lines as green
# draw goal post lines with red
for i in range(4):
    cv2.line(img, tuple(rect_points[i]), tuple(rect_points[(i + 1) % 4]), (0, 255, 0), 2)
cv2.line(img, tuple(left_goal_post[0]), tuple(left_goal_post[1]), (0, 0, 255), 2)
cv2.line(img, tuple(right_goal_post[0]), tuple(right_goal_post[1]), (0, 0, 255), 2)
cv2.imshow('image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()