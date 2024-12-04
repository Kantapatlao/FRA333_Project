from robot import RobotArm
import math

r = RobotArm([180,180,180])

print(r.forward_kinematic([math.pi/4,math.pi/4,0]))

