import numpy as np
import matplotlib.pyplot as plt
import math
import heapq
import random  # Add this import statement for random module
import cv2 as cv
# Forward Kinematics for 3 DOF Robot Arm
def forward_kinematics(theta1, theta2, theta3):
    l1, l2, l3 = 100, 100, 100  # Length of each arm segment
    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)
    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)
    x3 = x2 + l3 * np.cos(theta1 + theta2 + theta3)
    y3 = y2 + l3 * np.sin(theta1 + theta2 + theta3)
    return (x3, y3), (x1, y1), (x2, y2)

# Inverse Kinematics for 3 DOF Robot Arm
def inverse_kinematics(x, y):
    l1, l2, l3 = 100, 100, 100  # Length of each arm segment
    d = np.sqrt(x**2 + y**2)
    theta1 = np.arctan2(y, x) - np.arccos((d**2 + l1**2 - l2**2) / (2 * l1 * d))
    theta2 = np.arccos((d**2 - l1**2 - l2**2) / (-2 * l1 * l2))
    theta3 = 0  # Assume the last joint is aligned for simplicity
    return theta1, theta2, theta3

# A* algorithm implementation
def a_star(map, start, goal):
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance
    
    def is_valid(x, y):
        if x < 0 or x >= map.shape[1] or y < 0 or y >= map.shape[0]:
            return False
        return map[y, x] == 0  # Check if the cell is free (not an obstacle)

    def reconstruct_path(came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        return path[::-1]

    open_list = []
    heapq.heappush(open_list, (0 + heuristic(start, goal), 0, start))
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    came_from = {}

    while open_list:
        _, current_g, current = heapq.heappop(open_list)

        if current == goal:
            return reconstruct_path(came_from, current)

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # 4-connected neighbors
            neighbor = (current[0] + dx, current[1] + dy)
            if not is_valid(neighbor[0], neighbor[1]):
                continue
            tentative_g_score = current_g + 1

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_list, (f_score[neighbor], tentative_g_score, neighbor))

    return []

# Map generation with obstacles
def make_map(sizeX: int, sizeY: int, obstacles_count: int = random.randint(1, 5)):
    obstacle_shape = {0: cv.MORPH_RECT, 1: cv.MORPH_CROSS, 2: cv.MORPH_ELLIPSE}
    map = np.zeros((sizeY, sizeX)).astype(np.uint8)  # Initialized map
    for i in range(obstacles_count):
        obstacle_size = (random.randint(1, math.floor(sizeX / 2)), random.randint(1, math.floor(sizeY / 2)))
        obstacle_type = obstacle_shape[random.randint(0, 2)]
        obstacle_position = (sizeX - obstacle_size[0] - 1, sizeY - obstacle_size[1] - 1)
        obstacle_position = (random.randint(0, obstacle_position[0]), random.randint(0, obstacle_position[1]))
        x, y = obstacle_position
        xm, ym = x + obstacle_size[0], y + obstacle_size[1]
        map[y:ym, x:xm] = np.bitwise_or(map[y:ym, x:xm], cv.getStructuringElement(obstacle_type, obstacle_size))
    return map

# Visualization of arm and path
def plot_arm_with_path(map, arm_angles, path, start, target):
    fig, ax = plt.subplots()
    ax.set_xlim(0, map.shape[1])
    ax.set_ylim(0, map.shape[0])

    ax.imshow(map, cmap='gray_r')

    # Draw target point
    ax.plot(target[0], target[1], 'ro', markersize=10)

    # Plot the arm with clear links (thicker lines for the arms)
    x0, y0 = 0, 0  # Starting point of the arm
    arm_points = []
    for i, angle in enumerate(arm_angles):
        if i == 0:
            x1, y1 = forward_kinematics(angle, 0, 0)[0]
        elif i == 1:
            x1, y1 = forward_kinematics(angle, arm_angles[1], 0)[0]
        else:
            x1, y1 = forward_kinematics(angle, arm_angles[1], arm_angles[2])[0]
        arm_points.append((x0, y0, x1, y1))  # Store the line segments
        x0, y0 = x1, y1

    # Draw each arm link with thick lines and color
    for (x0, y0, x1, y1) in arm_points:
        ax.plot([x0, x1], [y0, y1], color='green', lw=6)  # Thicker green links for arms

    # Draw joints with different color and larger size
    joint_positions = [start]  # Start joint
    x0, y0 = 0, 0
    for i, angle in enumerate(arm_angles):
        if i == 0:
            x1, y1 = forward_kinematics(angle, 0, 0)[1]
        elif i == 1:
            x1, y1 = forward_kinematics(angle, arm_angles[1], 0)[1]
        else:
            x1, y1 = forward_kinematics(angle, arm_angles[1], arm_angles[2])[1]
        joint_positions.append((x1, y1))
    
    # Draw joints as circles (with a distinct color)
    for joint in joint_positions:
        ax.plot(joint[0], joint[1], 'bo', markersize=10)  # Blue circles for joints

    # Plot the path from the end effector to the target
    for (x, y) in path:
        ax.plot(x, y, 'bs', markersize=5)  # Blue square for the path

    plt.show()

# Example usage
map_size = (200, 200)  # Dummy map for visualization
map = make_map(map_size[0], map_size[1], 3)  # Generate a map with obstacles

# Define start and target points
start = (10, 10)
target = (150, 150)

# Calculate the angles for the arm to reach the target using Inverse Kinematics
arm_angles = inverse_kinematics(target[0], target[1])

# Perform A* to find path from the end effector to the target
path = a_star(map, (int(start[0]), int(start[1])), (int(target[0]), int(target[1])))

# Visualize the arm, target, and A* path
plot_arm_with_path(map, arm_angles, path, start, target)
