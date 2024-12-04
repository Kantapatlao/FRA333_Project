import math
import os
import io
import numpy as np
import cv2
import pygame
import random
import heapq
from pygame.locals import *
from typing import List, Tuple, Optional

pygame.init()

# Window settings
width, height = 1280, 720
FPS = 60

# Load map
map_file_path = os.path.join(os.path.abspath("Map"), 'map5.npy')
map_array = np.load(map_file_path).astype(np.uint8)

# Map processing
map_height, map_width = map_array.shape
scale = 500 / max(map_width, map_height)
new_width = int(map_width * scale)
new_height = int(map_height * scale)

# Resize and convert map
resized_map = cv2.resize(map_array, (new_width, new_height), interpolation=cv2.INTER_NEAREST)
inv_resized_map = np.ones((new_height, new_width), dtype=np.uint8) * 255
inv_resized_map[resized_map == 1] = 0
rgb_map = cv2.cvtColor(inv_resized_map, cv2.COLOR_GRAY2RGB)

# Prepare map surface
map_file = io.BytesIO(cv2.imencode(".png", inv_resized_map)[1])
map_surface = pygame.image.frombuffer(rgb_map.tobytes(), (new_width, new_height), 'RGB')

# Colors
white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
yellow = (255, 255, 0)

# Robot arm parameters
Link_Lengths = [167, 167, 167]
map_x = (width - new_width) // 2
map_y = (height - new_height) // 2

# Change origin to bottom left corner of the map
origin = (map_x, map_y + new_height)
joint_radius = 5

class AStar:
    def __init__(self, map_surface, map_x, map_y, new_width, new_height):
        self.map_surface = map_surface
        self.map_x = map_x
        self.map_y = map_y
        self.new_width = new_width
        self.new_height = new_height

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Manhattan distance heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, node: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring nodes"""
        neighbors = [
            (node[0]+1, node[1]), (node[0]-1, node[1]),
            (node[0], node[1]+1), (node[0], node[1]-1),
            (node[0]+1, node[1]+1), (node[0]-1, node[1]-1),
            (node[0]+1, node[1]-1), (node[0]-1, node[1]+1)
        ]
        
        # Filter out wall collisions and out-of-bounds nodes
        return [
            n for n in neighbors 
            if not self.check_wall_collision(n, self.map_surface, self.new_width, self.new_height, width, height)
        ]

    def check_wall_collision(self, point, map_surface, new_width, new_height, width, height):
        map_x = (width - new_width) // 2
        map_y = (height - new_height) // 2

        relative_x = int(point[0] - map_x)
        relative_y = int(point[1] - map_y)

        if (0 <= relative_x < new_width and 0 <= relative_y < new_height):
            try:
                pixel_color = map_surface.get_at((relative_x, relative_y))
                return pixel_color == (0, 0, 0, 255)
            except Exception:
                return True

        return True

    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from start to goal"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """A* path finding algorithm"""
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))

        # Boundary checks
        if self.check_wall_collision(start, self.map_surface, self.new_width, self.new_height, width, height) or \
           self.check_wall_collision(goal, self.map_surface, self.new_width, self.new_height, width, height):
            return None

        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current_f, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # No path found

def generate_random_target(astar):
    while True:
        rand_x = random.randint(astar.map_x, astar.map_x + astar.new_width - 1)
        rand_y = random.randint(astar.map_y, astar.map_y + astar.new_height - 1)
        
        if not astar.check_wall_collision((rand_x, rand_y), astar.map_surface, astar.new_width, astar.new_height, width, height):
            return rand_x, rand_y

def interpolate_point(start, end, fraction):
    return (
        start[0] + (end[0] - start[0]) * fraction, 
        start[1] + (end[1] - start[1]) * fraction
    )

def inverse_kinematics(target, origin, link_lengths):
    x, y = target[0] - origin[0], -(target[1] - origin[1])
    d = math.sqrt(x**2 + y**2)

    max_reach = sum(link_lengths)
    min_reach = abs(link_lengths[0] - link_lengths[1])

    if d > max_reach or d < min_reach:
        return None

    angle1 = math.atan2(y, x)
    
    cos_theta = (link_lengths[0]**2 + d**2 - link_lengths[1]**2) / (2 * link_lengths[0] * d)
    cos_theta = max(min(cos_theta, 1), -1)
    
    angle2 = angle1 + math.acos(cos_theta)
    
    cos_theta2 = (link_lengths[0]**2 + link_lengths[1]**2 - d**2) / (2 * link_lengths[0] * link_lengths[1])
    cos_theta2 = max(min(cos_theta2, 1), -1)
    
    angle3 = math.pi - math.acos(cos_theta2)

    return [angle1, angle2, angle3]

def interpolate_angles(current_angles, target_angles, step_size=0.1):
    new_angles = []
    for current, target in zip(current_angles, target_angles):
        diff = target - current
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        
        if abs(diff) > step_size:
            new_angle = current + (step_size if diff > 0 else -step_size)
        else:
            new_angle = target
        
        new_angles.append(new_angle)
    
    return new_angles

class RobotArm:
    def __init__(self):
        self.joint_angles = [0, 0, 0]
    
    def get_joint_positions(self):
        joints = [origin]
        current_angle = 0
        x, y = origin
        for i, length in enumerate(Link_Lengths):
            current_angle += self.joint_angles[i]
            x += length * math.cos(current_angle)
            y += length * math.sin(current_angle)
            joints.append((x, y))
        return joints

    def draw(self, screen):
        joints = self.get_joint_positions()
        
        # Draw links
        for i in range(len(joints) - 1):
            pygame.draw.line(screen, black, joints[i], joints[i + 1], 4)
        
        # Draw joints
        for joint in joints[:-1]:
            pygame.draw.circle(screen, blue, (int(joint[0]), int(joint[1])), joint_radius)
        
        # Draw end effector
        pygame.draw.circle(screen, green, (int(joints[-1][0]), int(joints[-1][1])), 5)

def main():
    screen = pygame.display.set_mode((width, height))
    clock = pygame.time.Clock()
    robot = RobotArm()
    
    # Create A* path finder
    astar = AStar(map_surface, map_x, map_y, new_width, new_height)
    
    current_angles = [0, 0, 0]
    running = True
    
    # Initial target and path
    current_target = generate_random_target(astar)
    end_effector = robot.get_joint_positions()[-1]
    path = astar.find_path(end_effector, current_target)
    path_index = 0

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        screen.fill(white)
        screen.blit(map_surface, ((width - new_width) // 2, (height - new_height) // 2))
        
        # Draw current path
        if path:
            for i in range(len(path) - 1):
                pygame.draw.line(screen, yellow, path[i], path[i+1], 2)
        
        # Draw current target
        pygame.draw.circle(screen, red, current_target, 5)
        
        # Get current end effector position
        end_effector = robot.get_joint_positions()[-1]
        
        # Check if we need a new path
        if path and path_index < len(path):
            next_waypoint = path[path_index]
            
            # Calculate target angles to next waypoint
            target_angles = inverse_kinematics(next_waypoint, origin, Link_Lengths)
            
            if target_angles is not None:
                # Slower, more deliberate movement
                current_angles = interpolate_angles(current_angles, target_angles, step_size=0.03)
                robot.joint_angles = current_angles
                
                # Check if we've reached the current waypoint
                current_end_effector = robot.get_joint_positions()[-1]
                if math.dist(current_end_effector, next_waypoint) < 10:
                    path_index += 1
        
        # Draw robot arm
        robot.draw(screen)
        
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()

if __name__ == "__main__":
    main()