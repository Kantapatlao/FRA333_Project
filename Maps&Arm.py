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
width, height = 1920, 1000
FPS = 60

# Load map
map_file_path = os.path.join(os.path.abspath("Map"), 'map1.npy')
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

class RobotArm:
    def __init__(self, origin, link_lengths):
        self.origin = origin
        self.link_lengths = link_lengths
        self.angles = [0, 0, 0]
        self.target = None

    def inverse_kinematics(self, target):
        """Calculate joint angles to reach the target."""
        try:
            x, y = target[0] - self.origin[0], -(target[1] - self.origin[1])
            d = math.sqrt(x**2 + y**2)

            # Check if target is reachable
            max_reach = sum(self.link_lengths)
            min_reach = abs(self.link_lengths[0] - self.link_lengths[1])

            if d > max_reach or d < min_reach:
                return None

            # Calculate angles using law of cosines
            angle1 = math.atan2(y, x)
            
            cos_theta = (self.link_lengths[0]**2 + d**2 - self.link_lengths[1]**2) / (2 * self.link_lengths[0] * d)
            cos_theta = max(min(cos_theta, 1), -1)
            
            angle2 = angle1 + math.acos(cos_theta)
            
            cos_theta2 = (self.link_lengths[0]**2 + self.link_lengths[1]**2 - d**2) / (2 * self.link_lengths[0] * self.link_lengths[1])
            cos_theta2 = max(min(cos_theta2, 1), -1)
            
            angle3 = math.pi - math.acos(cos_theta2)

            return [angle1, angle2, angle3]
        except Exception as e:
            print(f"IK calculation error: {e}")
            return None

    def get_joint_positions(self):
        """Calculate joint positions based on current angles."""
        joints = [self.origin]
        current_angle = 0
        x, y = self.origin
        for i, length in enumerate(self.link_lengths):
            current_angle += self.angles[i]
            x += length * math.cos(current_angle)
            y += length * math.sin(current_angle)
            joints.append((x, y))
        return joints

    def draw(self, screen):
        """Draw the robot arm on the screen."""
        joints = self.get_joint_positions()
        
        # Draw links
        for i in range(len(joints) - 1):
            pygame.draw.line(screen, black, joints[i], joints[i + 1], 4)
        
        # Draw joints
        for joint in joints[:-1]:
            pygame.draw.circle(screen, blue, (int(joint[0]), int(joint[1])), joint_radius)
        
        # Draw end effector
        pygame.draw.circle(screen, green, (int(joints[-1][0]), int(joints[-1][1])), 5)

    def set_target(self, target):
        """Set the target for the robot arm."""
        self.target = target

    def get_end_effector_position(self):
        """Get the current end effector position."""
        joints = self.get_joint_positions()
        return joints[-1]

def generate_random_target(origin, new_width, new_height, link_lengths):
    """Generate a random target within the map and arm's reach."""
    max_attempts = 1
    for _ in range(max_attempts):
        x = random.randint(origin[0], origin[0] + new_width)
        y = random.randint(origin[1] - new_height, origin[1])
        
        # Check if point is within arm's reach
        dx = x - origin[0]
        dy = origin[1] - y  # Inverted y-coordinate
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance <= sum(link_lengths):
            return (x, y)
    
    # Fallback to origin if no valid point found
    return origin

def interpolate_angles(current_angles, target_angles, step_size=0.1):
    """Smoothly interpolate between current and target angles."""
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

def main():
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Robot Arm Simulation")
    clock = pygame.time.Clock()
    
    # Initialize robot arm
    robot = RobotArm(origin, Link_Lengths)
    
    # Initial state variables
    current_angles = [0, 0, 0]
    running = True
    
    # Generate first target
    target = generate_random_target(origin, new_width, new_height, Link_Lengths)
    robot.set_target(target)
    
    # Timing for target pause
    target_reached_time = 0
    target_pause_duration = 2000  # 2 seconds
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        # Clear screen
        screen.fill(white)
        screen.blit(map_surface, ((width - new_width) // 2, (height - new_height) // 2))
        
        # Get current end effector position
        end_effector = robot.get_end_effector_position()
        
        # Calculate IK for current target
        target_angles = robot.inverse_kinematics(target)
        
        if target_angles is not None:
            # Smoothly interpolate angles
            current_angles = interpolate_angles(current_angles, target_angles)
            robot.angles = current_angles
            
            # Check if target is reached
            # if math.dist(end_effector, target) < 5:
            #     current_time = pygame.time.get_ticks()
            #     if current_time - target_reached_time >= target_pause_duration:
            #         # Generate new target after pause
            #         target = generate_random_target(origin, new_width, new_height, Link_Lengths)
            #         robot.set_target(target)
            #         target_reached_time = current_time
            # else:
            #     target_reached_time = 0
        else:
            # Generate new target if current is unreachable
            target = generate_random_target(origin, new_width, new_height, Link_Lengths)
            robot.set_target(target)
   
        # Draw robot and target
        robot.draw(screen)
        pygame.draw.circle(screen, red, target, 5)
        
        # Update display
        pygame.display.flip()
        clock.tick(FPS)
    print(target_angles)
    print(current_angles)
    print(target)
    print(end_effector)
    pygame.quit()

if __name__ == "__main__":
    main()