# robot.py
**Object in robot.py file**
- Robot
    - set_base_position
    - forward_kinematic
    - sequencial_IK_3
    - check_wall_collision
    - check_object_collision
    - draw_robot

**Dependency package**
- math
- numpy
- pygame
- RobotARM.constant
- Path_Finding.map_optimizer


## RobotArm

**NAME:** \
RobotArm

**TYPE:**\
Custom object

**SYNOPSIS:** 
```
RobotArm()
```

**DESCRIPTION**\
Create a RobotArm object which hold robot link object and the base position. Each link object is a private object to RobotArm. Initial angle of each link is set to 0 by default.

**ATTRIBUTE**
- base_position: A tuple of base position of the Robot.
- links: A list of link object. Each contain:
    - LENGTH: Length of respected link object. Value should be constant.
    - angle: Current angle (in radian) at the beginning of the joint relative to previous joint/base. 
    - end_positionX: X coordinate of the end of the link.
    - end_positionY: Y coordinate of the end of the link.


**PARAMETER**
- link_len: List of length of each link, from base to end effector. **Only takes list of unsigned integers as input.**

**OPTIONS**\
NONE

**EXAMPLES**
```
R = RobotArm([200, 200, 200])
```

**RETURN_VALUE**\
A RobotArm object.

**DIAGNOSTICS**
- `link_len only take list of int (integer) as input.`
- `<number> element of link_len is not integer.`
- `<number> element of link_len can not be negative or zero.`\
link_len isn't the same type as what the function accept. Refer in PARAMETER topic. The function doesn't automatically type cast any variable, so floating point without decimal point isn't accept as well.

**BUGS**\
NONE

## set_base_position

**NAME:** \
set_base_position

**TYPE:**\
RobotArm's method

**SYNOPSIS:** 
```
set_base_position(self, base_x, base_y)
```

**DESCRIPTION**\
Update base_position attribute of current RobotArm object.


**PARAMETER**
- base_x: X position of RobotArm object's base. **Only takes integer as input.**
- base_y: Y position of RobotArm object's base. **Only takes integer as input.**


**OPTIONS**\
NONE

**EXAMPLES**
```
R.set_base_position(0, 0)
```

**RETURN_VALUE**\
NONE

**DIAGNOSTICS**\
None

**BUGS**\
NONE


## forward_kinematic

**NAME:** \
forward_kinematic

**TYPE:**\
RobotArm's method

**SYNOPSIS:** 
```
forward_kinematic(self, joint)
```

**DESCRIPTION**\
Compute forward kinematic of the RobotArm object according to input joint angle. Then update angle and end_positionX,Y of each link. Also return end_positionX,Y of each link.


**PARAMETER**
- joint: List of joint angle to be compute, list from base position to end effector. **Only takes list of floating point number as input.**

**OPTIONS**\
NONE

**EXAMPLES**
```
Link_pos = R.forward_kinematic([math.pi/2, -math.pi/2, 0])

# Link_pos[0] = X1, Y1
# Link_pos[1] = X2, Y2
# ...
```

**RETURN_VALUE**\
Return a list of tuple of X and Y coordinate at end point of each link.

**DIAGNOSTICS**\
- `joint only take list of real number as input`\
joint isn't the same type as what the function accept. Refer in PARAMETER topic. The function doesn't automatically type cast any variable, so integer isn't accept as well.

- `input joint count doesn't match object`\
number of joint angle at input doesn't match the number of joint of RobotArm object.

**BUGS**\
NONE


## check_wall_collision

**NAME:** \
check_wall_collision

**TYPE:**\
RobotArm's method

**SYNOPSIS:** 
```
check_wall_collision(self, map_x, map_y, map_size_x, map_size_y)
```

**DESCRIPTION**\
Compute if current configuration of RobotArm object collide with Map's wall or not.


**PARAMETER**
- map_x: Top-left X coordinate where the map start. **Only takes integer as input.**
- map_y: Top-left Y coordinate where the map start. **Only takes integer as input.**
- map_size_x: Size of map in X axis (width). **Only takes integer as input.**
- map_size_y: Size of map in Y axis (height). **Only takes integer as input.**

**OPTIONS**\
NONE

**EXAMPLES**
```
Collide = R.check_wall_collision(100,100,500,500)

# Collide = True
# Collide = False
```

**RETURN_VALUE**\
Return True if current configuration collide with map wall.
Return False if not.

**DIAGNOSTICS**\

Common issue arised when using this method.
- `<input variable> only take <type> as input.`
Input isn't the same type as what the function accept. Refer in PARAMETER topic. The function doesn't automatically type cast any variable, so floating point without decimal point isn't accept as well.

- `Object's base_position isn't assigned yet. Draw the robot first.`\
Base position of RobotArm object isn't assigned yet. Consider assigning them using set_base_position method. Or draw_robot() which can assign base_position as well.

**BUGS**\
NONE


## check_object_collision

**NAME:** \
check_object_collision

**TYPE:**\
RobotArm's method

**SYNOPSIS:** 
```
check_object_collision(self, obstacle_list)
```

**DESCRIPTION**\
Compute if current configuration of RobotArm object collide with obstacle or not.


**PARAMETER**
- obstacle_list: List of Discrete_map object that has value of 1 (is obstacle). Representing all collision element from full grid map. **Only take list of Discrete_map of input.**

**OPTIONS**\
NONE

**EXAMPLES**
```
Collide = R.check_object_collision(Map.obstacle_list)

# Collide = True
# Collide = False
```

**RETURN_VALUE**\
Return True if current configuration collide with any obstacle element in map. Return False if not.

**DIAGNOSTICS**\

Common issue arised when using this method.
- `<input variable> only take <type> as input.`
Input isn't the same type as what the function accept. Refer in PARAMETER topic.

- `obstacle <number> is not Discrete_map object.`
Specify element isn't the same type as what the function accept. Refer in PARAMETER topic.

- `Object's base_position isn't assigned yet. Draw the robot first.`\
Base position of RobotArm object isn't assigned yet. Consider assigning them using set_base_position method. Or draw_robot() which can assign base_position as well.

**BUGS**\
NONE


## draw_robot

**NAME:** \
draw_robot

**TYPE:**\
RobotArm's method

**SYNOPSIS:** 
```
draw_robot(self, pygame_screen, base_x=None, base_y=None)
```

**DESCRIPTION**\
Draw RobotArm on pygame screen according to current object's attribute.


**PARAMETER**
- pygame_screen: pygame_screen object.
- base_x: X coordinate of RobotArm object's base reference in pygame's coordinate. **Only take integer as input**
- base_y: Y coordinate of RobotArm object's base reference in pygame's coordinate. **Only take integer as input**

**OPTIONS**\
NONE

**EXAMPLES**
```
R.draw_robot(screen, 100, 600)
```

**RETURN_VALUE**\
NONE

**DIAGNOSTICS**

Common issue arised when using this method.
- `<input variable> only take <type> as input.`
Input isn't the same type as what the function accept. Refer in PARAMETER topic.

- `Object's base_position isn't assigned yet. Draw the robot first.`\
Base position of RobotArm object isn't assigned yet. Consider assigning them using set_base_position method. Or draw_robot() which can assign base_position as well.

**BUGS**\
NONE

## sequencial_IK_3

**NAME:** \
sequencial_IK_3

**TYPE:**\
RobotArm's method

**SYNOPSIS:** 
```
sequencial_IK_3(self, in_x, in_y)
```

**DESCRIPTION**\
Compute joint configuration of 3 joints robot arm that make the end effector touch in_x and in_y target coordinate. **This function is for demonstration purpose only.** The joint configuration is computed by moving 1st joint to point to the target. Then, normal 2 dimension inverse kinematic is computed for joint 2 and 3. If Joint 2,3 can reach target without moving joint 1, the solution also include them as well. Due to how inverse kinematic is solve, it may not find the most optimal path to move joint to specify target.


**PARAMETER**
- in_x: X coordinate of target that RobotArm object should moved to. **Only take integer as input**
- in_y: Y coordinate of target that RobotArm object should moved to.  **Only take integer as input**

**OPTIONS**\
NONE

**EXAMPLES**
```
solution = R.sequencial_IK_3(100, 600)
```

**RETURN_VALUE**\
Nested list of at most 4 joint solution, each solution hold 3 joint angle from base to end effector.

**DIAGNOSTICS**

Common issue arised when using this method.
- `<input variable> only take <type> as input.`
Input isn't the same type as what the function accept. Refer in PARAMETER topic.

- `Object's base_position isn't assigned yet. Draw the robot first.`\
Base position of RobotArm object isn't assigned yet. Consider assigning them using set_base_position method. Or draw_robot() which can assign base_position as well.

- This method is for demo purpose. The joint configuration may not be optimal.

**BUGS**\
- Due to unknown bug when checking if target position is reachable or not, (compute each link length whether fully extending or fully retract can reach specify position or not.) Initial suspection is miscalculate when transforming frame between robot's and map's.