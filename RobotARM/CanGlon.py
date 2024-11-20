import math
import pygame
import numpy as np

pygame.init()

width = 1280
height = 720
FPS = 60

white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
gray = (128, 128, 128)
light_gray = (228, 228, 228)

# Slider
slider_width = 300
slider_height = 10
slider_x = 800
slider_y_start = 100
slider_spacing = 80
slider_handle_radius = 15

#แขน
Link_Lengths = [200, 200, 200]  
origin = (320, 700)
joint_radius = 10

class Slider:
    def __init__(self, x, y, width, height, min_val =- math.pi, max_val = math.pi):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.min_val = min_val
        self.max_val = max_val
        self.handle_radius = slider_handle_radius
        self.value = 0
        self.selected = False
    
    def draw(self, screen):
        pygame.draw.rect(screen, gray, (self.x, self.y, self.width, self.height))
        
        handle_x = self.x + (self.value - self.min_val) / (self.max_val - self.min_val) * self.width
        pygame.draw.circle(screen, light_gray, (int(handle_x), self.y + self.height//2), self.handle_radius)
        
    def is_handle_clicked(self, pos):        
        handle_x = self.x + (self.value - self.min_val) / (self.max_val - self.min_val) * self.width
        handle_y = self.y + self.height//2
        return math.dist((handle_x, handle_y), pos) < self.handle_radius
 
    def update(self, mouse_x):
        if self.selected:
            # Convert mouse position to value
            normalized_x = max(self.x, min(mouse_x, self.x + self.width))
            self.value = self.min_val + (normalized_x - self.x) / self.width * (self.max_val - self.min_val)
            return True
        return False


class RobotArm:
    def __init__(self):
        self.joint_angles = [0, 0, 0]
        self.selected_joint = None
        self.sliders = [
            Slider(slider_x, slider_y_start + i * slider_spacing, slider_width, slider_height)
            for i in range(3)
        ]
        self.font = pygame.font.SysFont('Arial', 24)
        
    def update_from_slider(self):
        for i, slider in enumerate(self.sliders):
            self.joint_angles[i] = slider.value
    
    def get_joint_positions(self):  
        joints = [origin]
        
        current_angle = self.joint_angles[0]  
        x = origin[0] + Link_Lengths[0] * math.cos(current_angle)
        y = origin[1] + Link_Lengths[0] * math.sin(current_angle)
        joints.append((x, y))
        
        current_angle += self.joint_angles[1]
        x += Link_Lengths[1] * math.cos(current_angle)
        y += Link_Lengths[1] * math.sin(current_angle)
        joints.append((x, y))
        
        current_angle += self.joint_angles[2]
        x += Link_Lengths[2] * math.cos(current_angle)
        y += Link_Lengths[2] * math.sin(current_angle)
        joints.append((x, y))
        
        return joints

    def draw_angle_labels(self, screen):
        for i, slider in enumerate(self.sliders):
            angle_text = f"Joint {i+1}: {self.joint_angles[i]:.2f} rad"
            text_surface = self.font.render(angle_text, True, black)
            screen.blit(text_surface, (slider_x, slider_y_start + i * slider_spacing - 30))


    def draw_position_info(self, screen):
        joints = self.get_joint_positions()
        
        # Draw box for position information
        info_box_x = 20
        info_box_y = 400
        line_height = 30
        
        # Draw background rectangle for position info
        info_width = 280
        info_height = (len(joints) + 1) * line_height + 20
        pygame.draw.rect(screen, (40, 40, 40), 
                        (info_box_x - 10, info_box_y - 10, 
                         info_width, info_height))

        # Draw joint positions
        for i, pos in enumerate(joints[:-1]):  # All joints except end effector
            # Convert coordinates to be relative to origin
            rel_x = pos[0] - origin[0]
            rel_y = -(pos[1] - origin[1])  # Invert Y for standard coordinate system
            
            text = f"Joint {i}: ({rel_x:.1f}, {rel_y:.1f})"
            text_surface = self.font.render(text, True, light_gray)
            screen.blit(text_surface, (info_box_x, info_box_y + i * line_height))

        # Draw end effector position
        end_pos = joints[-1]
        rel_x = end_pos[0] - origin[0]
        rel_y = -(end_pos[1] - origin[1])  # Invert Y for standard coordinate system
        
        ee_text = f"End Effector: ({rel_x:.1f}, {rel_y:.1f})"
        text_surface = self.font.render(ee_text, True, green)
        screen.blit(text_surface, (info_box_x, info_box_y + len(joints) * line_height))

        # Draw origin position
        pygame.draw.line(screen, white, (origin[0] - 10, origin[1]), (origin[0] + 10, origin[1]), 2)
        pygame.draw.line(screen, white, (origin[0], origin[1] - 10), (origin[0], origin[1] + 10), 2)
        
def main():
    screen = pygame.display.set_mode((width, height))
    clock = pygame.time.Clock()
    
    robot = RobotArm()
    
    running = True
    while running:
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

        screen.fill(white)
        
        pygame.draw.line(screen, (100, 100, 100), (origin[0], 0), (origin[0], height), 1) 
        pygame.draw.line(screen, (100, 100, 100), (0, origin[1]), (width, origin[1]), 1)
        
        joints = robot.get_joint_positions()  
        
        for i in range(len(joints)-1):
            pygame.draw.line(screen, black, joints[i], joints[i+1], 4)
        
        # Draw joints
        for i, joint in enumerate(joints[:-1]):
            pygame.draw.circle(screen, blue, (int(joint[0]), int(joint[1])), joint_radius)

        # Draw end effector
        pygame.draw.circle(screen, green, (int(joints[-1][0]), int(joints[-1][1])), 5)
        
        # Draw sliders and angle labels
        for slider in robot.sliders:
            slider.draw(screen)
        robot.draw_angle_labels(screen)
        
        robot.draw_position_info(screen)
        
        pygame.display.flip()
        clock.tick(FPS)
        
    pygame.quit()

if __name__ == "__main__":
    main()