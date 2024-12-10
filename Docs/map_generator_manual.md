# map_generator.ipynb
Object in map_generator.ipynp file:
- make_map

## A_star object

**NAME:** \
A_Star

**TYPE:**\
Custom object

**SYNOPSIS:** 
```
A_Star()
```

**DESCRIPTION**\
Initialized an empty object to hold data structure for later use in function compute_path. Data structure should be empty before computing path. No need to assign any value to object's attribute.

**OPTIONS**\
NONE

**EXAMPLES**
```
foo = A_Star()
```

**RETURN_VALUE**\
An empty A_Star object. This function shouldn't failed.

**DIAGNOSTICS**\
NONE

**BUGS**\
NONE

## compute_path()

**NAME:** \
A_Star

**TYPE:**\
A_Star's method

**SYNOPSIS:** 
```
foo.compute_path(goal_x, goal_y, map_input, robot_input)
```

**DESCRIPTION**\
Compute best task-space path(one that result in the least amount of joint angle moved) and without colliding with obstacle. Due to limitation of sequencial_IK_3 function, which can give out at most 4 joint solution despite infinite number of solutions being available. It may failed to compute path because all 4 solution provide by sequencial_IK_3 all collide with obstacle or wall.


**PARAMETER**
- goal_x: X position target in map's frame of reference (scaled to pygame's resolution). **Only takes integer as input.**
- goal_y: Y position target in map's frame of reference (scaled to pygame's resolution). **Only takes integer as input.**
- map_input: Map object that robot arm is in. **Only takes Map object as input.** 
- robot_input: Robot object that must move through map without colliding with object. **Only takes Robot object as input.** 

**OPTIONS**\
NONE

**EXAMPLES**
```
path = foo.compute_path(100,100, map, robot)

# Example result in path variable.
# Parent array (path[i]) hold each path node.
# Array nested inside (path[i][j]) hold each joint angle.
# path => [[0,0,0],[pi/2,pi/2,pi/2], ...]
```

**RETURN_VALUE**\
A 2D nested list of path to the goal coordinate. Each element in path hold joint configuration on each path node.

**DIAGNOSTICS**

Common issue arised when using this method.
- `<input variable> only take <type> as input.`

Input isn't the same type as what the function accept. Refer in PARAMETER topic. The function doesn't automatically type cast any variable, so floating point without decimal point isn't accept as well.

- `All config collide with wall/obstacle.`

Due to sequencial_IK_3, it can't find any solution that doesn't collide with wall or obstacle. Suggest using other inverse kinematic function (third party). First party inverse kinematic function will be implement later.

- `Maximum iteration reached.`

Loop count when searching map has reach maximum limit. This limit is just to prevent run away loop. If map can't be search with default limit. It's advices to tune constant value "MAX_ITER" according to specific use case.

**BUGS**
- Due to how sequencial_IK_3 work, which only give out 4 solutions, all 4 solution may collide with wall or obstacle which cause the search to fail. 
- Unknown bug when checking if target position is reachable or not, (compute each link length whether fully extending or fully retract can reach specify position or not.) Initial suspection is miscalculate when transforming frame between robot's and map's.