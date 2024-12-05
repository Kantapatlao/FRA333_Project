import math
import os
import io
import numpy as np
import cv2
import pygame
import random
from pygame.locals import *
from typing import List, Tuple, Optional

pygame.init()

# Window settings
width, height = 1920, 1000
FPS = 60

# Load map
map_file_path = os.path.join(os.getcwd(), 'Map', 'map1.npy')
if not os.path.exists(map_file_path):
    raise FileNotFoundError(f"Map file not found at {map_file_path}")
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
map_surface = pygame.image.frombuffer(rgb_map.tobytes(), (new_width, new_height), 'RGB')

# Colors
white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
yellow = (255, 255, 0)
magenta = (255, 0, 255)

# Robot arm parameters
Link_Lengths = [167, 167, 167]
map_x = (width - new_width) // 2
map_y = (height - new_height) // 2

# Change origin to bottom left corner of the map
origin = (map_x, map_y + new_height)
joint_radius = 5

class RobotArm:
    def __init__(self, origin, link_lengths):
        self.origin = origin
        self.link_lengths = link_lengths
        self.angles = [0, 0, 0]
        self.target = None

    def set_target(self, target):
        self.target = target

    def get_end_effector_position(self):
        joints = self.get_joint_positions()
        return joints[-1]

    def inverse_kinematics(self, target):
        try:
            x, y = target[0] - self.origin[0], self.origin[1] - target[1]
            d = math.sqrt(x**2 + y**2)
            max_reach = sum(self.link_lengths)
            min_reach = abs(self.link_lengths[0] - (self.link_lengths[1] + self.link_lengths[2]))

            # Check if the target is reachable
            if d > max_reach or d < min_reach:
                return None

            # Base angle
            base_angle = math.atan2(y, x)

            # Adjust for first link
            remaining_x = x - self.link_lengths[0] * math.cos(base_angle)
            remaining_y = y - self.link_lengths[0] * math.sin(base_angle)
            remaining_d = math.sqrt(remaining_x**2 + remaining_y**2)

            if remaining_d > self.link_lengths[1] + self.link_lengths[2]:
                return None  # Remaining target out of reach

            # Safe acos
            def safe_acos(value):
                return math.acos(max(min(value, 1), -1))

            # Recalculate angles
            remaining_angle = math.atan2(remaining_y, remaining_x)
            cos_theta2 = (self.link_lengths[1]**2 + remaining_d**2 - self.link_lengths[2]**2) / (2 * self.link_lengths[1] * remaining_d)
            angle2 = safe_acos(cos_theta2)

            cos_theta3 = (self.link_lengths[1]**2 + self.link_lengths[2]**2 - remaining_d**2) / (2 * self.link_lengths[1] * self.link_lengths[2])
            angle3 = safe_acos(cos_theta3)

            # Combine angles for both configurations
            theta1 = base_angle
            theta2 = remaining_angle - angle2 - theta1  # Elbow-up
            theta3 = math.pi - angle3  # Adjust for elbow-up

            return [theta1, theta2, theta3]

        except Exception as e:
            print(f"IK Error: {e}")
            return None

        
    def get_joint_positions(self):
        joints = [self.origin]
        x, y = self.origin
        current_angle = 0
        for i, length in enumerate(self.link_lengths):
            current_angle += self.angles[i]
            x += length * math.cos(current_angle)
            y -= length * math.sin(current_angle)
            joints.append((int(x), int(y)))
        return joints

    def bresenham(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            yield x0, y0
            if x0 == x1 and y0 == y1:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def check_collision(self, map_array):
        joints = self.get_joint_positions()
        for i in range(len(joints) - 1):
            x0, y0 = joints[i]
            x1, y1 = joints[i + 1]
            for px, py in self.bresenham(x0, y0, x1, y1):
                if 0 <= px < map_array.shape[1] and 0 <= py < map_array.shape[0]:
                    if map_array[py, px] == 0:
                        return True
        return False

    def draw(self, screen):
        joints = self.get_joint_positions()
        for i in range(len(joints) - 1):
            pygame.draw.line(screen, black, joints[i], joints[i + 1], 4)
        for joint in joints[:-1]:
            pygame.draw.circle(screen, blue, joint, joint_radius)
        pygame.draw.circle(screen, green, joints[-1], 5)

def generate_random_target(origin, new_width, new_height, link_lengths):
    max_reach = sum(link_lengths)
    min_reach = abs(link_lengths[0] - link_lengths[1])
    for _ in range(50):
        x = random.randint(origin[0], origin[0] + new_width)
        y = random.randint(origin[1] - new_height, origin[1])
        dx = x - origin[0]
        dy = origin[1] - y
        distance = math.sqrt(dx**2 + dy**2)
        if min_reach <= distance <= max_reach:
            return x, y
    return origin[0] + new_width // 2, origin[1] - new_height // 2

def main():
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Robot Arm Simulation")
    clock = pygame.time.Clock()
    robot = RobotArm(origin, Link_Lengths)
    target = generate_random_target(origin, new_width, new_height, Link_Lengths)
    robot.set_target(target)
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(white)
        screen.blit(map_surface, (map_x, map_y))
        target_angles = robot.inverse_kinematics(target)

        if target_angles:
            robot.angles = target_angles

        if robot.check_collision(resized_map):
            pygame.draw.rect(screen, yellow, (0, 0, 300, 50))
            warning_text = pygame.font.Font(None, 36).render("Collision Detected!", True, red)
            screen.blit(warning_text, (10, 10))

        robot.draw(screen)
        pygame.draw.circle(screen, red, target, 5)
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()

if __name__ == "__main__":
    main()