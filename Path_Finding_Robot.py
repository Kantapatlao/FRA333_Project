import os
import math

import numpy as np

import pygame
from Path_Finding.map_optimizer import Discrete_map, Map
import RobotARM.constant as R_const
from RobotARM.robot import RobotArm
from Map_Utils.visualize_map import map2img
from A_Star.A_Star import A_Star

# Constant value
SCREEN_WIDTH = 1280
SCREEN_HEIGHT = 720
PI = math.pi

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
    Robot.forward_kinematic([PI/2, -PI, PI])
    Robot.set_base_position(R_const.ROBOT_COORDINATE_X, R_const.ROBOT_COORDINATE_Y)
    map2img(screen, np_map, 100, 100)

    target_x = 150
    target_y = 150

    A = A_Star()
    path = A.compute_path(target_x, target_y, the_map, Robot)
    
    
    


    running = True
    state = 0
    key_reset = True

    while running:

        screen.fill((255,255,255))
        map2img(screen, np_map, 100, 100)

        for event in pygame.event.get():
            
            if event.type == pygame.QUIT:
                running = False
            
            elif event.type == pygame.KEYDOWN and key_reset:
                key = pygame.key.get_pressed()
                if key[pygame.K_LEFT]:
                    state = max(0, state - 1)
                    key_reset = False

                if key[pygame.K_RIGHT]:
                    state = min(len(path) - 1, state + 1)
                    key_reset = False
                

            elif event.type == pygame.KEYUP:
                key_reset = True

        Robot.forward_kinematic(path[state].joint_sol)
        Robot.draw_robot(screen, R_const.ROBOT_COORDINATE_X, R_const.ROBOT_COORDINATE_Y)
        pygame.draw.circle(screen, (255,0,0), (target_x + 100, target_y + 100), radius=3)

        pygame.display.flip()

    pygame.quit()
    return 0












if __name__ == "__main__":
    main()



