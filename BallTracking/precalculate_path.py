import cv2
import numpy as np
import time
import h5py
import os
from concurrent.futures import ProcessPoolExecutor

# Define colors
red = (0, 0, 255)
blue = (255, 0, 0)
green = (0, 255, 0)

# Class to represent line properties
class Line:
    def __init__(self, slope, intercept, end_point_index, start_point, distance):
        self.slope = slope
        self.intercept = intercept
        self.end_point_index = end_point_index
        self.start_point = start_point
        self.distance = distance

    def to_list(self):
        # Convert attributes to list for HDF5 storage
        return [self.slope, self.intercept, self.end_point_index,
                self.start_point[0], self.start_point[1], self.distance]

# Save points to HDF5
def save_points_hdf5(points, filename):
    with h5py.File(filename, 'w') as f:
        f.create_dataset('goal_line_points', data=points)

# Load points from HDF5
def load_points_hdf5(filename):
    with h5py.File(filename, 'r') as f:
        return list(f['goal_line_points'])

# Generate goal line points based on start and end positions
def generate_goal_line_points(start, end):
    return [(start[0], i) for i in range(start[1], end[1])]

# Generate line properties for each point to a set of goal line points
def generate_line_properties(point, goal_line_points):
    line_properties = []
    x_diff = max(abs(point[0] - goal_line_points[0][0]), 1)
    steps = max(len(goal_line_points) / x_diff, 1)

    for i in range(0, len(goal_line_points), int(steps)):
        distance = np.linalg.norm(np.array(point) - np.array(goal_line_points[i]))
        dx = goal_line_points[i][0] - point[0]
        dy = goal_line_points[i][1] - point[1]

        if dx == 0:
            m = float('inf')
            b = point[0]
        else:
            m = dy / dx
            b = point[1] - m * point[0]

        line = Line(m, b, i, point, int(distance))
        line_properties.append(line.to_list())

    return line_properties

# Process a chunk of rows and save line equations for each row
def process_row_chunk(start_row, end_row, img_width, x_offset, chunk_index):
    print(f"Processing chunk {chunk_index} from {start_row} to {end_row}")

    loaded_goal_line_points = load_points_hdf5('goal_line_points.h5')

    for i in range(start_row, end_row):
        line_equations_columns = []

        if i < x_offset:
            with h5py.File(f"paths/row_{i}.h5", "w") as f:
                f.create_dataset(f"line_equations_row_{i}", shape=(0,), dtype='float32')
            continue

        for j in range(img_width):
            point = (i, j)
            line_properties = generate_line_properties(point, loaded_goal_line_points)
            line_equations_columns.append(line_properties)

        with h5py.File(f"paths/row_{i}.h5", "w") as f:
            dataset_name = f"line_equations_row_{i}"
            f.create_dataset(dataset_name, data=np.array(line_equations_columns, dtype='float32'))

    print(f"Chunk {chunk_index} processed.")

# Read specific row data from HDF5 file
def read_hdf5_row(row_index):
    hdf5_filename = f"paths/row_{row_index}.h5"
    try:
        with h5py.File(hdf5_filename, "r") as f:
            dataset_name = f"line_equations_row_{row_index}"
            if dataset_name in f:
                return f[dataset_name][:]
            else:
                print(f"Dataset {dataset_name} does not exist in {hdf5_filename}.")
                return None
    except Exception as e:
        print(f"An error occurred while reading {hdf5_filename}: {e}")
        return None

# Retrieve line data for a specific point
def get_point_line_data(point):
    row, col = point
    row_data = read_hdf5_row(row)
    if row_data is not None and len(row_data) > col:
        return row_data[col]
    else:
        print(f"No data found for point {point}.")
        return None
def draw_line(img, line_equations, goal_line_points):
    if line_equations is not None and len(line_equations) > 0:
        middle_index = len(line_equations) // 2 

        for idx, line in enumerate(line_equations):
            _, _, end_point_index, _, _, _ = line
            end_point = goal_line_points[int(end_point_index)]

            # Set color to blue for most lines, red for the middle line
            color = red if idx == middle_index else blue  # Red for middle, blue for others
            cv2.line(img, point, end_point, color, 1)

if __name__ == '__main__':
    img = np.zeros((500, 500, 3), np.uint8)
    left_goal_start = (0, 200)
    left_goal_end = (0, 300)

    goal_line_points = generate_goal_line_points(left_goal_start, left_goal_end)
    save_points_hdf5(goal_line_points, 'goal_line_points.h5')
    print("Goal line points saved in HDF5 format.")

    img_height, img_width, _ = img.shape
    x_offset = 30
    num_processes = 6
    os.makedirs('paths', exist_ok=True)  # Ensure directory exists
    user_input = input("Data from previous computations may exists. Do you wish to compute anyway? (yes/no): ").strip().lower()
    if user_input != 'yes':
        print("Skipping computation based on previous results.")
        load_previous_data = True  # Set a flag to load existing data later in the code
    else:
        rows_per_process = img_height // num_processes
        row_chunks = [(i * rows_per_process, (i + 1) * rows_per_process) for i in range(num_processes)]
        if row_chunks[-1][1] < img_height:
            row_chunks[-1] = (row_chunks[-1][0], img_height)

        start_time = time.time()
        with ProcessPoolExecutor(max_workers=num_processes) as executor:
            futures = [executor.submit(process_row_chunk, start, end, img_width, x_offset, idx)
                    for idx, (start, end) in enumerate(row_chunks)]
            for future in futures:
                future.result()
        print('Process completed.')
        print("Matrix generation time:", time.time() - start_time)
        load_previous_data = False

    point = (300, 300)
    line_equations = get_point_line_data(point)

    imgcopy = img.copy()

    draw_line(imgcopy, line_equations, goal_line_points)
    cv2.line(img, left_goal_start, left_goal_end, green, 5)
    cv2.imshow("imgcopy", imgcopy)
    cv2.waitKey(0)
    cv2.destroyAllWindows()