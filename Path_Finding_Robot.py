import os

import numpy as np

import pygame
from Path_Finding.map_optimizer import Discrete_map, Map
import RobotARM.constant as R_const
from RobotARM.robot import RobotArm
from Map_Utils.visualize_map import map2img

# Constant value
SCREEN_WIDTH = 1280
SCREEN_HEIGHT = 720

MAP_PATH = os.path.join(os.path.abspath("Map"), 'map1.npy')

def main():
    # Initialize Pygame and screen
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Robot Arm Collision Detection")

    # Load map
    np_map = np.load(MAP_PATH, allow_pickle=False)
    the_map = Map(np_map)

    # Create Robot Arm
    Robot = RobotArm([180,180,180])
    Robot.set_base_position(100, 600)

    # Fill screen with white
    screen.fill((255,255,255))
    
    # Draw map
    map2img(screen, np_map, 100, 100)

    # Demonstrate wall collision check
    map_boundary = {
        'x': 100,     # map start x
        'y': 100,     # map start y
        'width': 500, # map width
        'height': 500 # map height
    }
    
    wall_collision = Robot.check_wall_collision(
        map_boundary['x'], 
        map_boundary['y'], 
        map_boundary['width'], 
        map_boundary['height']
    )
    print(f"Wall Collision: {wall_collision}")

    # Prepare obstacles for object collision check
    # Convert map obstacles to Discrete_map objects
    obstacles = []
    for y in range(np_map.shape[0]):
        for x in range(np_map.shape[1]):
            if np_map[y, x] == 1:  # Assuming 1 represents obstacles
                obstacle = Discrete_map(x, y, 1, 1, 1)
                obstacles.append(obstacle)

    # Demonstrate object collision check
    object_collision = Robot.check_object_collision(obstacles)
    print(f"Object Collision: {object_collision}")

    # Visualization (optional)
    if wall_collision or object_collision:
        pygame.draw.rect(screen, (255, 0, 0), 
                         ((map_boundary['x'], map_boundary['y']), 
                          (map_boundary['width'], map_boundary['height'])), 
                         2)  # Red boundary if collision detected

    # Main game loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        pygame.display.flip()

    pygame.quit()
    return 0

if __name__ == "__main__":
    main()
