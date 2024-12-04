import math
import numpy as np
import pygame

# Declare Robot arm object
class RobotArm:

    # Declare subclass of each joint of robot arm
    class _joint:
        def __init__(self, in_l) -> None:
           self.LENGTH = in_l
           self.angle = 0
           self.end_positionX = 0
           self.end_positionY = 0


    # Initialized Robot arm object
    def __init__(self, link_len) -> None:

        # Check type of lin_len   
        if type(link_len) is not list:
            raise TypeError("link_len only take list of int (integer) as input.")
        
        for i,l in enumerate(link_len):
            if type(l) is not int:
                raise TypeError(f'{i} element of link_len is not integer.')
            
            if l < 0:
                raise ValueError(f'{i} element of link_len can not be negative value.')
        
        self.base_position = None
        self.links = [self._joint(l) for l in link_len]


    # Calculate forward kinematic
    def forward_kinematic(self, joint):
        
        # Check input type
        if type(joint) is not list:
            raise TypeError("link_len only take list of int (integer) as input.")
        
        if len(joint) != len(self.links):
            raise ValueError(f"input joint count doesn't match object's joint count. Object has {len(self.links)} joint.",)

        # Check value in joint
        for i in range(0, len(joint)):
            
            # Try type casting
            try:
                joint[i] = float(joint[i])

            except (ValueError, TypeError, OverflowError) as e:
                raise RuntimeError(f"At joint {i}, {e}")
            
        
        # Calculate each joint
        self.links[0].angle = joint[0]
        j_buf = self.links[0].angle
        self.links[0].end_positionX = math.cos(j_buf) * self.links[0].LENGTH
        self.links[0].end_positionY = math.sin(j_buf) * self.links[0].LENGTH
        
        for i in range(1,len(joint)):

            
            self.links[i].angle = joint[i] + self.links[i-1].angle
            j_buf = self.links[i].angle
            self.links[i].end_positionX = self.links[i-1].end_positionX + (math.cos(j_buf) * self.links[i].LENGTH)
            self.links[i].end_positionY = self.links[i-1].end_positionY + (math.sin(j_buf) * self.links[i].LENGTH)

        return [(self.links[i].end_positionX, self.links[i].end_positionY) for i in range(len(self.links))]

        

            


                
    # Calculate inverse kinematic (Sequencial kinematic)


    # TODO: Calculate inverse kinematic (Newton-Raphson's method)
    



    # Check wall collision
    def check_wall_collision(self, map_x, map_y, map_size_x, map_size_y):

        # Check input type
        if type(map_x) is not int:
            raise TypeError("map_x only take integer as input.")

        # Check input type
        if type(map_y) is not int:
            raise TypeError("map_y only take integer as input.")

        # Check input type
        if type(map_size_x) is not int:
            raise TypeError("map_size_x only take integer as input.")

        # Check input type
        if type(map_size_y) is not int:
            raise TypeError("map_size_y only take integer as input.")
        
        # Check if robot have it's base position or not
        if self.base_position is None:
            return ValueError("Object's base_position isn't assigned yet. Draw the robot first.")
        
        for link in self.links:

            # Check left wall collision
            if link.end_positionX + self.base_position[0] < map_x:
                return True
            
            # Check right wall collision
            if link.end_positionX + self.base_position[0] > map_x + map_size_x:
                return True
            
            # Check top wall collision
            if self.base_position[1] - link.end_positionY < map_y:
                return True
            
            # Check bottom wall collision
            if self.base_position[1] - link.end_positionY > map_y + map_size_y:
                return True
            
        return False
    
    
    
    
    
    # Draw Robot
    def draw_robot(self, pygame_screen, base_x, base_y, y_max):

        # Check input type
        if type(base_x) is not int:
            raise TypeError("base_x only take integer as input.")
        
        if type(base_y) is not int:
            raise TypeError("base_y only take integer as input.")
        
        if type(y_max) is not int:
            raise TypeError("y_max only take integer as input.")
        
        self.base_position = (base_x, base_y)
        

        pygame.draw.line(pygame_screen, 
                        (0,255,0),
                        (base_x, base_y),
                        (self.links[0].end_positionX + base_x, base_y - self.links[0].end_positionY),
                        width=5
                        )

        for i,link in enumerate(self.links):
            pygame.draw.line(pygame_screen,
                            (0,255,0), 
                            (self.links[i-1].end_positionX + base_x, base_y - self.links[i-1].end_positionY),
                            (self.links[i].end_positionX + base_x, base_y - self.links[i].end_positionY),
                            width=5
                            )
            
        return None
    


    


    