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

map_file_path = os.path.join(os.path.abspath("Map"), 'map1.npy')

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

def is_point_inside_map(point, map_surface, new_width, new_height, width, height):
    # Calculate the map's position on the screen
    map_x = (width - new_width) // 2
    map_y = (height - new_height) // 2

    # Convert point screen coordinates to map surface coordinates
    relative_x = int(point[0] - map_x)
    relative_y = int(point[1] - map_y)

    # Check if the point is within the map surface boundaries
    return (0 <= relative_x < new_width and 0 <= relative_y < new_height)

def draw_map_border(screen, map_surface, width, height, new_width, new_height):
    # Calculate the position of the map on the screen
    map_x = (width - new_width) // 2
    map_y = (height - new_height) // 2
    
    # Draw border lines
    pygame.draw.rect(screen, black, 
        (map_x, map_y, new_width, new_height), 1  # Line thickness of 2 pixels
    )

def check_wall_collision(point, map_surface, new_width, new_height, width, height):
    # Calculate the map's position on the screen
    if not is_point_inside_map(point, map_surface, new_width, new_height, width, height):
        return True  # Outside map is considered a collision
    
    map_x = (width - new_width) // 2
    map_y = (height - new_height) // 2

    # Convert end effector screen coordinates to map surface coordinates
    relative_x = int(point[0] - map_x)
    relative_y = int(point[1] - map_y)

    # Check if the point is within the map surface boundaries
 
    try:
        pixel_color = map_surface.get_at((relative_x, relative_y))
        # Check if the pixel is black (wall)
        return pixel_color == (0, 0, 0, 255)
    except Exception:
        return True  # Treat any out-of-bounds as collision

def check_line_collision(start_point, end_point, map_surface, new_width, new_height, width, height, steps=20):
    # Check multiple points along the line for collision
    for i in range(steps + 1):
        t = i / steps
        x = start_point[0] + t * (end_point[0] - start_point[0])
        y = start_point[1] + t * (end_point[1] - start_point[1])
        
        if check_wall_collision((x, y), map_surface, new_width, new_height, width, height):
            return True
    return False

class Slider:
    def __init__(self, x, y, width, height, min_val=-math.pi, max_val=math.pi):
        self.x, self.y = x, y
        self.width, self.height = width, height
        self.min_val, self.max_val = min_val, max_val
        self.handle_radius = slider_handle_radius
        self.value = 0
        self.selected = False

    def draw(self, screen):
        # Draw the slider track
        pygame.draw.rect(screen, gray, (self.x, self.y, self.width, self.height))
        # Draw the slider handle
        handle_x = self.x + (self.value - self.min_val) / (self.max_val - self.min_val) * self.width
        pygame.draw.circle(screen, light_gray, (int(handle_x), self.y + self.height // 2), self.handle_radius)

    def is_handle_clicked(self, pos):
        handle_x = self.x + (self.value - self.min_val) / (self.max_val - self.min_val) * self.width
        handle_y = self.y + self.height // 2
        return math.dist((handle_x, handle_y), pos) < self.handle_radius

    def update(self, mouse_x):
        if self.selected:
            normalized_x = max(self.x, min(mouse_x, self.x + self.width))
            self.value = self.min_val + (normalized_x - self.x) / self.width * (self.max_val - self.min_val)
            return True
        return False

class RobotArm:
    def __init__(self):
        self.joint_angles = [0, 0, 0]
        self.previous_safe_angles = [0, 0, 0]
        self.sliders = [Slider(slider_x, slider_y_start + i * slider_spacing, slider_width, slider_height) for i in range(3)]

        self.font = pygame.font.SysFont('Arial', 18)
        self.collision_detected = False
        self.collision_points = []  # Store collision points
        self.collision_links = []   # Store links with collisions
        
        self.joint_positions = []
        self.sliders = [Slider(slider_x, slider_y_start + i * slider_spacing, slider_width, slider_height) for i in range(3)]

        self.paths = []  # To store paths for each joint

        self.map_surface = None
        self.new_width = 0
        self.new_height = 0
        self.screen_width = 0
        self.screen_height = 0
        

    def calculate_joint_positions(self):
        # คำนวณตำแหน่งข้อต่อจากมุมปัจจุบัน
        self.joint_positions = [origin]
        current_angle = 0
        x, y = origin
        for i, length in enumerate(Link_Lengths):
            current_angle += self.joint_angles[i]
            x += length * math.cos(current_angle)
            y += length * math.sin(current_angle)
            self.joint_positions.append((x, y))

    def find_paths(self, goals):
        """
        สร้างเส้นทางของแขนกลไปยังตำแหน่งเป้าหมาย
        """
        self.paths = []  # Clear existing paths
        current_position = origin

        for goal in goals:
            path = []  # เก็บจุดระหว่างทางสำหรับข้อต่อนี้
            steps = 50  # จำนวนขั้นตอนในการเคลื่อนที่ไปยังเป้าหมาย

            for t in range(steps + 1):
                x = current_position[0] + (goal[0] - current_position[0]) * t / steps
                y = current_position[1] + (goal[1] - current_position[1]) * t / steps
                path.append((x, y))

            self.paths.append(path)
            current_position = goal  # อัปเดตตำแหน่งปัจจุบันไปยังเป้าหมายนี้
    def check_arm_collision(self, map_surface, new_width, new_height, width, height):
        # Reset collision tracking
        self.collision_points = []
        # self.collision_links = []
        
        # Get all joint positions
        joints = self.get_joint_positions()
        
        # Check each joint for collision
        for i, joint in enumerate(joints[:-1]):  # Exclude end effector
            if (not is_point_inside_map(joint, map_surface, new_width, new_height, width, height)or
                check_wall_collision(joint, map_surface, new_width, new_height, width, height)):
                self.collision_points.append((joint, i))
                return True
        # Check each link for collision
        # for i in range(len(joints) - 1):
        #     if check_line_collision(joints[i], joints[i+1], map_surface, new_width, new_height, width, height):
        #         self.collision_links.append((joints[i], joints[i+1], i))
        
        # # Return True if any collision detected
        # return len(self.collision_points) > 0 or len(self.collision_links) > 0  
        return False
    def check_link_inside_map(self, start_point, end_point, map_surface, new_width, new_height, width, height, steps=20):
        # Check multiple points along the line
        for i in range(steps + 1):
            t = i / steps
            x = start_point[0] + t * (end_point[0] - start_point[0])
            y = start_point[1] + t * (end_point[1] - start_point[1])
            
            # Check if point is outside map or hits a wall
            if (not is_point_inside_map((x, y), map_surface, new_width, new_height, width, height) or 
                check_wall_collision((x, y), map_surface, new_width, new_height, width, height)):
                return True
        return False
    
    def update_from_slider(self):
        updated_angles = self.joint_angles[:]
        for i, slider in enumerate(self.sliders):
            print(f"Updating slider {i}: current value={slider.value}")  # Debugging
            updated_angles[i] = slider.value
            self.joint_angles = updated_angles

            if (self.map_surface and self.check_arm_collision(self.map_surface, self.new_width, self.new_height, self.screen_width, self.screen_height)):
                print(f"Collision detected for joint {i}, reverting to previous safe angle")  # Debugging
                updated_angles[i] = self.previous_safe_angles[i]
                slider.value = self.previous_safe_angles[i]
            else:
                print(f"Joint {i} updated to {updated_angles[i]} without collision")  # Debugging
                self.previous_safe_angles[i] = updated_angles[i]

        self.joint_angles = updated_angles
    
    def set_map_parameters(self, map_surface, new_width, new_height, screen_width, screen_height):
        # Method to set map parameters
        self.map_surface = map_surface
        self.new_width = new_width
        self.new_height = new_height
        self.screen_width = screen_width
        self.screen_height = screen_height
        
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
        for i, slider in enumerate(self.sliders):
            angle_text = f"Joint {i+1}: {self.joint_angles[i]:.2f} rad"
            text_surface = self.font.render(angle_text, True, black)
            screen.blit(text_surface, (slider_x, slider_y_start + i * slider_spacing - 30))

    def draw(self, screen):
        print(f"Joint angles: {self.joint_angles}")  # Debugging
        print(f"Slider values: {[slider.value for slider in self.sliders]}")  # Debugging
        # Draw the map border
        draw_map_border(screen, self.map_surface, self.screen_width, self.screen_height, self.new_width, self.new_height)

        # Calculate joint positions
        self.calculate_joint_positions()

        # Draw links and joints
        for i in range(len(self.joint_positions) - 1):
            pygame.draw.line(screen, black, self.joint_positions[i], self.joint_positions[i + 1], 4)
            pygame.draw.circle(screen, blue, (int(self.joint_positions[i][0]), int(self.joint_positions[i][1])), joint_radius)
        pygame.draw.circle(screen, green, (int(self.joint_positions[-1][0]), int(self.joint_positions[-1][1])), joint_radius)

    def draw_sliders(self, screen):
        for slider in self.sliders:
            slider.draw(screen)

# Main function
def main():
    screen = pygame.display.set_mode((width, height))
    screen.fill(white)
    screen.blit(bg, (0, 0))
    clock = pygame.time.Clock()
    robot = RobotArm()
    running = True
    robot.set_map_parameters(map_surface, new_width, new_height, width, height)

    # Example goals for each joint
    goals = [
        (200, 200),  # Goal for joint 1
        (300, 300),  # Goal for joint 2
        (400, 400)   # Goal for end effector
    ]
    robot.find_paths(goals)

    while running:

        screen.fill(white)
        screen.blit(map_surface, ((width - new_width) // 2, (height - new_height) // 2))
        draw_map_border(screen, map_surface, width, height, new_width, new_height)
        robot.draw(screen)
        robot.draw_sliders(screen)
        pygame.display.flip()
        clock.tick(FPS)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    for slider in robot.sliders:
                        if slider.is_handle_clicked(event.pos):
                            slider.selected = True
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    for slider in robot.sliders:
                        slider.selected = False
            elif event.type == pygame.MOUSEMOTION:
                for slider in robot.sliders:
                    if slider.is_handle_clicked(event.pos):
                        print(f"Slider {robot.sliders.index(slider)} clicked")  # Debugging
                        slider.selected = True


    pygame.quit()
if __name__ == "__main__":
    main()