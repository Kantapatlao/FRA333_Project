import math
import numpy as np

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
    def __init__(self, baseX, baseY, link_len) -> None:

        # Check if baseX and baseY type
        if type(baseX) is not int:
            raise TypeError("baseX only take int (integer) as input.")
        
        if type(baseY) is not int:
            raise TypeError("baseY only take int (integer) as input.")
        
        if type(link_len) is not list:
            raise TypeError("link_len only take list of int (integer) as input.")
        
        for i,l in enumerate(link_len):
            if type(l) is not int:
                raise TypeError(f'{i} element of link_len is not integer.')
            
            if l < 0:
                raise ValueError(f'{i} element of link_len can not be negative value.')
        

        self.base_position = (baseX, baseY)
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
        self.links[0].end_positionX = self.base_position[0] + (math.cos(j_buf) * self.links[0].LENGTH)
        self.links[0].end_positionY = self.base_position[1] + (math.sin(j_buf) * self.links[0].LENGTH)
        
        for i in range(1,len(joint)):

            
            self.links[i].angle = joint[i] + self.links[i-1].angle
            j_buf = self.links[i].angle
            self.links[i].end_positionX = self.links[i-1].end_positionX + (math.cos(j_buf) * self.links[i].LENGTH)
            self.links[i].end_positionY = self.links[i-1].end_positionY + (math.sin(j_buf) * self.links[i].LENGTH)

        return [(self.links[i].end_positionX, self.links[i].end_positionY) for i in range(len(self.links))]

        

            


                
    # Calculate inverse kinematic (Sequencial kinematic)


    # TODO: Calculate inverse kinematic (Newton-Raphson's method)


    # Draw Robot


    


    


    