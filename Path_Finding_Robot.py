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

    # Initialized each module
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

    # Initialized Map system
    np_map = np.load(MAP_PATH, allow_pickle=False)
    the_map = Map(np_map)

    # Initialized Robot arm object
    Robot = RobotArm([180,180,180])
    # Robot.forward_kinematic([3.14/4,0,0])
    Robot.set_base_position(100,600)
    screen.fill((255,255,255))
    map2img(screen, np_map, 100, 100)

    # TODO: Resolve this
    import random
    bX, bY = random.randint(0,20), random.randint(0,20)
    print(f"Random target: {bX, bY}")
    the_node = the_map.find_nearest_node(bX, bY)
    the_node = the_node.scale_discrete_map(R_const.SCALING, R_const.MAP_COORDINATE_X, R_const.MAP_COORDINATE_Y)

    sol = Robot.sequencial_IK_3(bX * 10, bY * 10)
    Robot.forward_kinematic(sol[0])

    pygame.draw.rect(screen, (150,100,0), ((the_node.posX, the_node.posY),(the_node.sizeX, the_node.sizeY)))
    Robot.draw_robot(screen, R_const.ROBOT_COORDINATE_X, R_const.ROBOT_COORDINATE_Y)
    pygame.draw.circle(screen, (255,0,0), (bX * 10 + 100, (bY * 10) + 100), radius=3)

    

    running = True

    while running:

        
        # if Robot.check_wall_collision(100,100,500,500):
        #     pygame.draw.rect(screen, (255,0,0), ((1000,100),(1100,200)))

        for event in pygame.event.get():
            
            if event.type == pygame.QUIT:
                running = False


        pygame.display.flip()

    pygame.quit()
    return 0












if __name__ == "__main__":
    main()



