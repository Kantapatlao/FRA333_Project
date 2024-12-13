import math
import numpy as np
import pygame
import RobotARM.constant as R_const
from Path_Finding.map_optimizer import Discrete_map

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
            
            if l <= 0:
                raise ValueError(f'{i} element of link_len can not be negative or zero.')
        
        self.base_position = None
        self.links = [self._joint(l) for l in link_len]


    # Set base position
    def set_base_position(self, base_x, base_y):
        self.base_position = (base_x, base_y)


    # Calculate forward kinematic
    def forward_kinematic(self, joint):
        
        # Check input type
        if type(joint) is not list:
            raise TypeError("joint only take list of real number as input.")
        
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

            
            self.links[i].angle = joint[i] 
            j_buf = self.links[i].angle + sum([l.angle for l in self.links[:i]])
            self.links[i].end_positionX = self.links[i-1].end_positionX + (math.cos(j_buf) * self.links[i].LENGTH)
            self.links[i].end_positionY = self.links[i-1].end_positionY + (math.sin(j_buf) * self.links[i].LENGTH)

        return [(self.links[i].end_positionX, self.links[i].end_positionY) for i in range(len(self.links))]

        

                
    # Calculate inverse kinematic (Sequencial inverse kinematic)
    def sequencial_IK_3(self, in_x, in_y):

        # Check input type
        if type(in_x) is not int:
            raise TypeError("in_x only takes integer as input.")
        
        if type(in_y) is not int:
            raise TypeError("in_y only takes integer as input.") 
        
        if self.base_position is None:
            raise ValueError("base_position is not assigned yet.")
        
        target_x = R_const.ROBOT_COORDINATE_X - R_const.MAP_COORDINATE_X + in_x
        target_y = R_const.ROBOT_COORDINATE_Y - R_const.MAP_COORDINATE_Y - in_y
        
        
        # Check if it's possible to reach or not
        link_len_sort = sorted([l.LENGTH for l in self.links], reverse=True)
        R_max = sum(link_len_sort)
        R_min = link_len_sort[0] - sum(link_len_sort[1:])
        Reach = math.sqrt(((target_x)** 2) + ((target_y) ** 2))

        if Reach < R_min or Reach > R_max:
            raise ValueError("Can't reach specify target, link not long enough.")
        
        # NOTE: In order for this algorithm to solve IK, it must have link2,3 longer than link 1.
        if self.links[0].LENGTH > self.links[1].LENGTH + self.links[2].LENGTH:
            raise SystemError("This algorithm can't solve robot arm with link 2,3 shorter than link 1.")
        

        # Check if joint 2,3 can reach on target on their own
        R2_max = self.links[1].LENGTH + self.links[2].LENGTH
        R2_min = self.links[1].LENGTH- self.links[2].LENGTH
        Reach2 = math.sqrt(((target_x - self.links[0].end_positionX) ** 2) + ((target_y - self.links[0].end_positionY) ** 2))

        solution = []
        theta1 = self.links[0].angle

        if Reach2 > R2_min and Reach2 < R2_max:
            # # If yes, 4 solution are calculated. (don't move j1, j2,j3 config1, j2,j3 config2)
            buf_target_x = target_x - (math.cos(theta1) * self.links[0].LENGTH)
            buf_target_y = target_y - (math.sin(theta1) * self.links[0].LENGTH)
            cos_theta3 = ((buf_target_x ** 2) + (buf_target_y ** 2) - (self.links[1].LENGTH ** 2) - (self.links[2].LENGTH ** 2)) / (2 * self.links[1].LENGTH * self.links[2].LENGTH)
            sin_theta3a = math.sqrt(1 - (cos_theta3 ** 2))
            sin_theta3b = -1 * sin_theta3a

            theta3a = math.atan2(sin_theta3a, cos_theta3)
            theta3b = math.atan2(sin_theta3b, cos_theta3)
            theta2a = math.atan2(buf_target_y, buf_target_x) - math.atan2(self.links[2].LENGTH * sin_theta3a, self.links[1].LENGTH + (self.links[2].LENGTH * cos_theta3))
            theta2b = math.atan2(buf_target_y, buf_target_x) - math.atan2(self.links[2].LENGTH * sin_theta3b, self.links[1].LENGTH + (self.links[2].LENGTH * cos_theta3))
            
            solution = [[theta1, theta2a - theta1, theta3a], [theta1, theta2b - theta1, theta3b]]
        
        # If no, 2 solution are calcualted. (move j1 closest, j2,3 config1, j2,j3 config2)
        # Solve for joint 1 angle
        theta1 = math.atan2(target_y, target_x) 

        # Solve for joint 2,3 angle
        buf_target_x = target_x - (math.cos(theta1) * self.links[0].LENGTH)
        buf_target_y = target_y - (math.sin(theta1) * self.links[0].LENGTH)
        cos_theta3 = ((buf_target_x ** 2) + (buf_target_y ** 2) - (self.links[1].LENGTH ** 2) - (self.links[2].LENGTH ** 2)) / (2 * self.links[1].LENGTH * self.links[2].LENGTH)
        sin_theta3a = math.sqrt(1 - (cos_theta3 ** 2))
        sin_theta3b = -1 * sin_theta3a

        theta3a = math.atan2(sin_theta3a, cos_theta3)
        theta3b = math.atan2(sin_theta3b, cos_theta3)
        theta2a = math.atan2(buf_target_y, buf_target_x) - math.atan2(self.links[2].LENGTH * sin_theta3a, self.links[1].LENGTH + (self.links[2].LENGTH * cos_theta3))
        theta2b = math.atan2(buf_target_y, buf_target_x) - math.atan2(self.links[2].LENGTH * sin_theta3b, self.links[1].LENGTH + (self.links[2].LENGTH * cos_theta3))
            
        solution = solution + [[theta1, theta2a - theta1, theta3a], [theta1, theta2b - theta1, theta3b]]    

        # Sorted according to which solution move joint the least (from current config)
        solution = sorted(solution, key = lambda sol: abs(sol[0] - self.links[0].angle) + abs(sol[1] - self.links[1].angle) + abs(sol[2] - self.links[2].angle))
        
        return solution

    # TODO: Calculate inverse kinematic (Rapid exploring random tree method) <-- Should be better than above
    



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
    
    
    # Check collision to obstacle
    def check_object_collision(self, obstacle_list):

        # Check input type
        if type(obstacle_list) is not list:
            raise TypeError("obstacle_list only take list of discrete map.")
        
        for i, o in enumerate(obstacle_list):
            if type(o) is not Discrete_map:
                raise TypeError(f"obstacle {i} is not Discrete_map object.")
            
        if self.base_position is None:
            raise ValueError("base_position isn't assigned yet.")    

        # Check if a single line collide with a square
        # line_start: (line start position x, line start position y)
        # line_end: (line end position x, line end position y)
        # rectangle: discrete map object
        def _check_line_rectangle_collide(line_start, line_end, rectangle):
            
            # No need to check type since this is should only be called by check_object_collision

            # Scale/remap to pygame's coordinate
            rectangle = rectangle.scale_discrete_map(R_const.SCALING, R_const.MAP_COORDINATE_X, R_const.MAP_COORDINATE_Y)
            
            # Get the line's inclination(m) and starting point(b) to calculate contact point
            line_m = (line_end[1] - line_start[1]) / (line_end[0] - line_start[0])
            line_b = line_start[1] - (line_m*line_start[0])

            # Check if line collide with rectangle's left edge
            rect_edge = rectangle.posX
            if line_start[0] <= rect_edge and line_end[0] >= rect_edge:
                contact_point = (line_m*rect_edge) + line_b
                if contact_point >= rectangle.posY and contact_point <= rectangle.posY + rectangle.sizeY:
                    return True


            # Check if line collide with rectangle's right edge
            rect_edge = rectangle.posX + rectangle.sizeX - 1
            if line_start[0] <= rect_edge and line_end[0] >= rect_edge:
                contact_point = (line_m*rect_edge) + line_b
                if contact_point >= rectangle.posY and contact_point <= rectangle.posY + rectangle.sizeY:
                    return True

            # Check if line collide with rectangle's top edge
            rect_edge = rectangle.posY
            if line_start[1] <= rect_edge and line_end[1] >= rect_edge:
                contact_point = (rect_edge - line_b) / line_m
                if contact_point >= rectangle.posX and contact_point <= rectangle.posX + rectangle.sizeX:
                    return True

            # Check if line collide with rectangle's bottom edge
            rect_edge = rectangle.posY + rectangle.sizeY - 1
            if line_start[1] <= rect_edge and line_end[1] >= rect_edge:
                contact_point = (rect_edge - line_b) / line_m
                if contact_point >= rectangle.posX and contact_point <= rectangle.posX + rectangle.sizeX:
                    return True

            return False
        
        # Check link 1
        for ob in obstacle_list:
            if _check_line_rectangle_collide((self.base_position[0], self.base_position[1]),
                                         (self.links[0].end_positionX + self.base_position[0], self.base_position[1] - self.links[0].end_positionY),
                                         ob):
                return True
            
        # Check link 2..
        for ilink in range(1,len(self.links)):
            for ob in obstacle_list:
                if _check_line_rectangle_collide((self.links[ilink-1].end_positionX + self.base_position[0], self.base_position[1] - self.links[ilink-1].end_positionY),
                                                 (self.links[ilink].end_positionX + self.base_position[0], self.base_position[1] - self.links[ilink].end_positionY),
                                                  ob):
                    return True
        
        return False
    
    # Draw Robot
    def draw_robot(self, pygame_screen, base_x=None, base_y=None):

        # If doesn't provide base location suggest that base location is already drawn
        if base_x is None or base_y is None:
            if self.base_position is None:
                raise ValueError("base position is never assigned.")
            
            
        else:
            # Check input type
            if type(base_x) is not int:
                raise TypeError("base_x only take integer as input.")

            if type(base_y) is not int:
                raise TypeError("base_y only take integer as input.")
        
        
            self.base_position = (base_x, base_y)
        

        pygame.draw.line(pygame_screen, 
                        (0,255,0),
                        (self.base_position[0], self.base_position[1]),
                        (self.links[0].end_positionX + self.base_position[0], self.base_position[1] - self.links[0].end_positionY),
                        width=5
                        )

        pygame.draw.circle(pygame_screen,
                        (0,0,255),
                        (self.base_position[0], self.base_position[1]),
                        radius=5)
        
        
        for i in range(1, len(self.links)):
            pygame.draw.line(pygame_screen,
                            (0,255,0), 
                            (self.links[i-1].end_positionX + self.base_position[0], self.base_position[1] - self.links[i-1].end_positionY),
                            (self.links[i].end_positionX + self.base_position[0], self.base_position[1] - self.links[i].end_positionY),
                            width=5
                            )
            
            pygame.draw.circle(pygame_screen,
                            (0,0,255),
                            (self.links[i-1].end_positionX + self.base_position[0], self.base_position[1] - self.links[i-1].end_positionY),
                            radius=5)
            
        return None
    


    


    