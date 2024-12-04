import math
import os
import io
import numpy as np
import cv2
import pygame
import random
from pygame.locals import *
from Path_Finding.map_optimizer import Map

pygame.init()

# Window settings
width, height = 1280, 720
FPS = 60

map_file_path = os.path.join(os.path.abspath("Map"), 'map2.npy')

# โหลดข้อมูล numpy array จากไฟล์
map_array = np.load(map_file_path)
map_array = map_array.astype(np.uint8)

# ตรวจสอบขนาดของ map_array
map_height, map_width = map_array.shape

# คำนวณสเกลเพื่อให้ภาพใหญ่สุด 500 พิกเซล
scale = 500 / max(map_width, map_height)  # อัตราส่วนสำหรับขยาย
new_width = int(map_width * scale)
new_height = int(map_height * scale)

# ปรับขนาดภาพ
resized_map = cv2.resize(map_array, (new_width, new_height), interpolation=cv2.INTER_NEAREST)

# กลับสีภาพ
inv_resized_map = np.ones((new_height, new_width), dtype=np.uint8) * 255
inv_resized_map[resized_map == 1] = 0
rgb_map = cv2.cvtColor(inv_resized_map, cv2.COLOR_GRAY2RGB)

# save ภาพเป็น file on memory
map_file = io.BytesIO(cv2.imencode(".png", inv_resized_map)[1])

map_surface = pygame.image.frombuffer(rgb_map.tobytes(), (new_width, new_height), 'RGB')

# Colors Pallet
white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
gray = (128, 128, 128)
light_gray = (228, 228, 228)

# Robot arm parameters
Link_Lengths = [167, 167, 167]
origin = (((width - new_width) // 2), ((height + new_height) // 2))
joint_radius = 5

# Slider settings
slider_width, slider_height = 300, 10
slider_x, slider_y_start, slider_spacing = 950, 50, 60
slider_handle_radius = 10

display = pygame.display.set_mode((1280, 720))
bg = pygame.image.load(map_file, "foo.png")
node_map = Map(map_array)

def generate_random_target(map_surface, new_width, new_height, width, height):
    map_x = (width - new_width) // 2
    map_y = (height - new_height) // 2
    
    while True:
        rand_x = random.randint(map_x, map_x + new_width - 1)
        rand_y = random.randint(map_y, map_y + new_height - 1)
        if not check_wall_collision((rand_x, rand_y), map_surface, new_width, new_height, width, height):
            return rand_x, rand_y

def inverse_kinematics(target, origin, link_lengths):
    x, y = target[0] - origin[0], -(target[1] - origin[1])  # Adjust for screen coordinates
    d = math.sqrt(x**2 + y**2)
    
    max_reach = sum(link_lengths)
    min_reach = abs(link_lengths[0] - link_lengths[1])
    
    if d > max_reach or d < min_reach:
        return None  # Target out of reach

    # Law of Cosines to find angles
    angle1 = math.atan2(y, x)
    
    # Use law of cosines to compute joint angles
    cos_theta = (link_lengths[0]**2 + d**2 - link_lengths[1]**2) / (2 * link_lengths[0] * d)
    cos_theta = max(min(cos_theta, 1), -1)  # Ensure value is within [-1, 1]
    
    angle2 = angle1 + math.acos(cos_theta)
    
    # Compute third joint angle
    cos_theta2 = (link_lengths[0]**2 + link_lengths[1]**2 - d**2) / (2 * link_lengths[0] * link_lengths[1])
    cos_theta2 = max(min(cos_theta2, 1), -1)  # Ensure value is within [-1, 1]
    
    angle3 = math.pi - math.acos(cos_theta2)

    return [angle1, angle2, angle3]

def interpolate_angles(current_angles, target_angles, step_size=0.1):
    """
    Interpolate between current and target angles with a small step
    """
    new_angles = []
    for current, target in zip(current_angles, target_angles):
        # Calculate the shortest angle difference
        diff = target - current
        # Normalize the difference to the range [-pi, pi]
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        
        # Move towards the target by step_size
        if abs(diff) > step_size:
            new_angle = current + (step_size if diff > 0 else -step_size)
        else:
            new_angle = target
        
        new_angles.append(new_angle)
    
    return new_angles

def draw_map_border(screen, map_surface, width, height, new_width, new_height):
    # Calculate the position of the map on the screen
    map_x = (width - new_width) // 2
    map_y = (height - new_height) // 2
    
    # Draw border lines
    pygame.draw.rect(screen, black, 
        (map_x, map_y, new_width, new_height), 2  # Line thickness of 2 pixels
    )

def check_wall_collision(point, map_surface, new_width, new_height, width, height):
    # Calculate the map's position on the screen
    map_x = (width - new_width) // 2
    map_y = (height - new_height) // 2

    # Convert end effector screen coordinates to map surface coordinates
    relative_x = int(point[0] - map_x)
    relative_y = int(point[1] - map_y)

    # Check if the point is within the map surface boundaries
    if (0 <= relative_x < new_width and 0 <= relative_y < new_height):
        # Get the pixel color at the point
        try:
            pixel_color = map_surface.get_at((relative_x, relative_y))
            # Check if the pixel is black (wall)
            return pixel_color == (0, 0, 0, 255)
        except Exception as e:
            print(f"Error checking pixel: {e}")
            return False

    return False

def check_line_collision(start_point, end_point, map_surface, new_width, new_height, width, height, steps=20):
    # Check multiple points along the line for collision
    for i in range(steps + 1):
        t = i / steps
        x = start_point[0] + t * (end_point[0] - start_point[0])
        y = start_point[1] + t * (end_point[1] - start_point[1])
        
        if check_wall_collision((x, y), map_surface, new_width, new_height, width, height):
            return True
    return False

class RobotArm:
    def __init__(self):
        self.joint_angles = [0, 0, 0]
        self.font = pygame.font.SysFont('Arial', 18)
        self.collision_detected = False
        self.collision_points = []  # Store collision points
        self.collision_links = []   # Store links with collisions
    
    def check_arm_collision(self, map_surface, new_width, new_height, width, height):
        # Reset collision tracking
        self.collision_points = []
        self.collision_links = []
        
        # Get all joint positions
        joints = self.get_joint_positions()
        
        # Check each joint for collision
        for i, joint in enumerate(joints[:-1]):  # Exclude end effector
            if check_wall_collision(joint, map_surface, new_width, new_height, width, height):
                self.collision_points.append((joint, i))
        
        # Check each link for collision
        for i in range(len(joints) - 1):
            if check_line_collision(joints[i], joints[i+1], map_surface, new_width, new_height, width, height):
                self.collision_links.append((joints[i], joints[i+1], i))
        
        # Return True if any collision detected
        return len(self.collision_points) > 0 or len(self.collision_links) > 0  

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

    def draw_angle_labels(self, screen):
        for i in enumerate(self.sliders):
            angle_text = f"Joint {i+1}: {self.joint_angles[i]:.2f} rad"
            text_surface = self.font.render(angle_text, True, black)
            screen.blit(text_surface, (slider_x, slider_y_start + i * slider_spacing - 30))

    def draw(self, screen):
        joints = self.get_joint_positions()
        
        collision_detected = self.check_arm_collision(
            map_surface,  # Pygame surface of the map
            new_width,    # Width of the resized map
            new_height,   # Height of the resized map
            width,        # Total screen width
            height        # Total screen height
        )
        for i in range(len(joints) - 1):
            # Check if this link is in collision
            link_color = red if any(l[0] == joints[i] and l[1] == joints[i+1] for l in self.collision_links) else black
            pygame.draw.line(screen, link_color, joints[i], joints[i + 1], 6 if link_color == red else 4)
        
        for i, joint in enumerate(joints[:-1]):
            # Check if this joint is in collision
            joint_color = red if any(p[0] == joint for p in self.collision_points) else blue
            pygame.draw.circle(screen, joint_color, (int(joint[0]), int(joint[1])), joint_radius)
        
        end_effector_color = red if collision_detected else green
        pygame.draw.circle(screen, end_effector_color, 
                           (int(joints[-1][0]), int(joints[-1][1])), 5)
        
        for joint in joints[:-1]:
            pygame.draw.circle(screen, blue, (int(joint[0]), int(joint[1])), joint_radius)
            end_effector_color = red if self.collision_detected else green
            pygame.draw.circle(screen, end_effector_color, (int(joints[-1][0]), int(joints[-1][1])), 5)
            
        if collision_detected:
            warning_font = pygame.font.SysFont('Arial', 24)
            warning_text = warning_font.render("WALL COLLISION!", True, red)
            screen.blit(warning_text, (20, height - 50))
            
        debug_font = pygame.font.SysFont('Arial', 18)
        if self.collision_points:
            for point, joint_index in self.collision_points:
                debug_text = debug_font.render(
                    f"Collision at Joint {joint_index}", 
                    True, red
                )
                screen.blit(debug_text, (20, height - 100 - joint_index * 30))
                
        if self.collision_links:
            for start, end, link_index in self.collision_links:
                debug_text = debug_font.render(
                    f"Collision on Link {link_index}", 
                    True, red
                )
                screen.blit(debug_text, (20, height - 200 - link_index * 30))

        # Box position info
        info_box_x = 20
        info_box_y = 400
        line_height = 30
        
        # Background rectangle for position info
        info_width = 180
        info_height = (len(joints) + 1) * line_height + 10
        pygame.draw.rect(screen, (40, 40, 40), 
                        (info_box_x - 10, info_box_y - 10,info_width, info_height))
        # Draw joint positions
        for i, pos in enumerate(joints[:-1]):  # All joints except end effector
            # Convert coordinates to be relative to origin
            rel_x = pos[0] - origin[0]
            rel_y = -(pos[1] - origin[1])  
            
            text = f"Joint {i}: ({rel_x:.1f}, {rel_y:.1f})"
            text_surface = self.font.render(text, True, light_gray)
            screen.blit(text_surface, (info_box_x, info_box_y + i * line_height))

        # Draw end effector position
        end_pos = joints[-1]
        rel_x = end_pos[0] - origin[0]
        rel_y = -(end_pos[1] - origin[1])  
        
        ee_text = f"End Effector: ({rel_x:.1f}, {rel_y:.1f})"
        text_surface = self.font.render(ee_text, True, green)
        screen.blit(text_surface, (info_box_x, info_box_y + len(joints) * line_height))

        # Draw origin position
        pygame.draw.line(screen, black, (origin[0] - 10, origin[1]), (origin[0] + 10, origin[1]), 2)
        pygame.draw.line(screen, black, (origin[0], origin[1] - 10), (origin[0], origin[1] + 10), 2)

# Main function
def main():
    target = generate_random_target(map_surface, new_width, new_height, width, height)
    screen = pygame.display.set_mode((width, height))
    screen.fill(white)
    screen.blit(bg, (0, 0))
    clock = pygame.time.Clock()
    robot = RobotArm()
    running = True
    current_angles = [0, 0, 0]
    running = True
    
    while running:
        screen.fill(white)
        screen.blit(map_surface, ((width - new_width) // 2, (height - new_height) // 2))
        draw_map_border(screen, map_surface, width, height, new_width, new_height)
        
        pygame.draw.circle(screen, red, target, 5)
        
        # Compute target angles
        target_angles = inverse_kinematics(target, origin, Link_Lengths)
        
        if target_angles is not None:
            # Interpolate angles incrementally
            current_angles = interpolate_angles(current_angles, target_angles)
            
            # Assign interpolated angles to the robot
            robot.joint_angles = current_angles
            
            # Check if end effector has reached the target
            end_effector = robot.get_joint_positions()[-1]
            distance_to_target = math.dist(end_effector, target)
            
            # Print debug information
            print(f"Distance to target: {distance_to_target}")
            print(f"Current Angles: {[math.degrees(angle) for angle in current_angles]}")
            
            if distance_to_target < 10:  # Reached target
                print("Target reached!")
                pygame.time.delay(1000)  # Pause
                # Generate a new random target
                target = generate_random_target(map_surface, new_width, new_height, width, height)
        else:
            # If target is unreachable, display a message
            font = pygame.font.SysFont('Arial', 24)
            text = font.render("Target Unreachable!", True, red)
            screen.blit(text, (width // 2 - 100, height - 50))
        
        robot.draw(screen)
        
        pygame.display.flip()
        clock.tick(FPS)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

if __name__ == "__main__":
    main()