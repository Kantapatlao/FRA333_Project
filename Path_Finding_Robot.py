import os

import numpy as np

import pygame
from Path_Finding.map_optimizer import Map
from RobotARM.robot import RobotArm
from Map_Utils.visualize_map import map2img

# Constant value
SCREEN_WIDTH = 1280
SCREEN_HEIGHT = 720
MAP_PATH = os.path.join(os.path.abspath("Map"), 'map1.npy')

def main():

    # Initialized each module
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

    # Initialized Map system
    np_map = np.load(MAP_PATH, allow_pickle=False)
    the_map = Map(np_map)

    # Initialized Robot arm object
    Robot = RobotArm([180,180,180])
    Robot.forward_kinematic([-3.14,0,0])


    running = True

    while running:

        screen.fill((255,255,255))
        map2img(screen, np_map, 100, 100)
        Robot.draw_robot(screen, 100, 600, 720)

        if Robot.check_wall_collision(100,100,500,500):
            pygame.draw.rect(screen, (255,0,0), ((1000,100),(1100,200)))

        for event in pygame.event.get():
            
            if event.type == pygame.QUIT:
                running = False


        pygame.display.flip()

    pygame.quit()
    return 0












if __name__ == "__main__":
    main()



