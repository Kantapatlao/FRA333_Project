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
Link_Lengths = [150, 150, 150]
origin = (((width - new_width) // 2), ((height + new_height) // 2))
joint_radius = 5

# Slider settings
slider_width, slider_height = 300, 10
slider_x, slider_y_start, slider_spacing = 950, 50, 60
slider_handle_radius = 15

display = pygame.display.set_mode((1280, 720))
bg = pygame.image.load(map_file, "foo.png")
node_map = Map(map_array)

def draw_map_border(screen, map_surface, width, height, new_width, new_height):
    # Calculate the position of the map on the screen
    map_x = (width - new_width) // 2
    map_y = (height - new_height) // 2
    
    # Draw border lines
    pygame.draw.rect(screen, black, 
        (map_x, map_y, new_width, new_height), 
        2  # Line thickness of 2 pixels
    )
    
class Slider:
    def __init__(self, x, y, width, height, min_val=-math.pi, max_val=math.pi):
        self.x, self.y = x, y
        self.width, self.height = width, height
        self.min_val, self.max_val = min_val, max_val
        self.handle_radius = slider_handle_radius
        self.value = 0
        self.selected = False

    def draw(self, screen):
        pygame.draw.rect(screen, gray, (self.x, self.y, self.width, self.height))
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
        self.sliders = [Slider(slider_x, slider_y_start + i * slider_spacing, slider_width, slider_height) for i in range(3)]
        self.font = pygame.font.SysFont('Arial', 18)

    def update_from_slider(self):
        for i, slider in enumerate(self.sliders):
            self.joint_angles[i] = slider.value

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
        joints = self.get_joint_positions()
        for i in range(len(joints) - 1):
            pygame.draw.line(screen, black, joints[i], joints[i + 1], 4)
        for joint in joints[:-1]:
            pygame.draw.circle(screen, blue, (int(joint[0]), int(joint[1])), joint_radius)
        pygame.draw.circle(screen, green, (int(joints[-1][0]), int(joints[-1][1])), 5)
        
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
                    if slider.update(event.pos[0]):
                        robot.update_from_slider()



    pygame.quit()

if __name__ == "__main__":
    main()