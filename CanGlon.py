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

Link_Lengths = [200, 200, 200]  
origin = (320, 700)
joint_radius = 10

class RobotArm:
    def __init__(self):
        self.joint_angles = [0, 0, 0]
        self.selected_joint = None
        
    def update_selected_joint(self, mouse_pos):
        joints = self.get_joint_positions()  
        for i, joint in enumerate(joints):
            if math.dist(mouse_pos, joint) < 20:
                self.selected_joint = i
                return
        self.selected_joint = None  
    
    def update_angle(self, mouse_pos):
        if self.selected_joint is not None:  
            joints = self.get_joint_positions()  
            joint_pos = joints[self.selected_joint]
            
            dx = mouse_pos[0] - joint_pos[0]
            dy = mouse_pos[1] - joint_pos[1]
            new_angle = math.atan2(dy, dx)
            
            if self.selected_joint == 0:
                self.joint_angles[0] = new_angle
            elif self.selected_joint == 1:
                self.joint_angles[1] = new_angle - self.joint_angles[0]
            elif self.selected_joint == 2:
                self.joint_angles[2] = new_angle - (self.joint_angles[0] + self.joint_angles[1])
    
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


def main():
    screen = pygame.display.set_mode((width, height))
    clock = pygame.time.Clock()
    
    robot = RobotArm()
    dragging = False
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  
                    robot.update_selected_joint(event.pos)
                    dragging = True
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    dragging = False
                    robot.selected_joint = None
            elif event.type == pygame.MOUSEMOTION:
                if dragging:
                    robot.update_angle(event.pos)

        screen.fill(white)
        
        joints = robot.get_joint_positions()  
        
        for i in range(len(joints)-1):
            pygame.draw.line(screen, black, joints[i], joints[i+1], 4) 
        
        for i, joint in enumerate(joints):  
            color = red if i == robot.selected_joint else blue
            pygame.draw.circle(screen, color, (int(joint[0]), int(joint[1])), joint_radius)

        pygame.draw.circle(screen, green, (int(joints[-1][0]), int(joints[-1][1])), 5)
        
        pygame.display.flip()
        clock.tick(FPS)
        
    pygame.quit()

if __name__ == "__main__":
    main()