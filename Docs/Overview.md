




# Overview
List of public API in each file with overview of how each function work.

## A_Star.py
- A* object: A* compute object, declare before use.
    - compute_path: take goal coordinate, map and robot object to compute path in form of _A_Star_Node sequence to move through.

## map_generator.ipynb
- make map: From given map size and amout of obstacle, generate grid map in form of numpy's ndarray with obstacle being 1 and 0 being available path.

## map_optimizer.py
- Discrete map: An object hold data of map in form of X,Y position (top-left conner), X,Y size and it's value.
    - get_center_pos: Returns center position of the current instance of discrete map object.
    - get_bottom_right_pos: Returns bottom-right position of the current instance of discrete map object.
    - scale_discrete_map: Return a new discrete map object that scaled according to input.


- Map: An object that take grid map and compute into discrete map.
    - find_adjacent_node: From given discrete map object, return list of discrete map object that's adjacent to the left, right, top and bottom. Discrete map that only touch the corner don't count.
    - find_nearest_node: From given coordinate, return discretemap that given coordinate is in.
    - list_obstacle: From current instant of map object, return list of discrete map object that have value of 1 (is obstacle).
    - show_graph: From current instant of map object, print each node (discrete map object) and the property out.


## robot.py
- Robot: An object that hold joint angle, joint length of each joint and the base position.
    - set_base_position: Set X and Y position of the robot base.
    - forward_kinematic: From given list of joint angle, update current Robot object with given angle and calculated position of each link. Return ifnal position of each link.
    - sequencial_IK_3: From given X,Y coordinate, calculate up to 4 possible joint configuartion that make end effector reach X,Y coordinate. Computed joint configuration is return as list of joint configuration.
    - check_wall_collision: From given map size and position, check if current configuration of robot arm does collide with edge of map or not.
    - check_object_collision: From given obstacle ojbect (Discrete map object) list, check whether current configuration collide with any obstacle or not.
    - draw_robot: Draw robot onto pygame screen according to current configuration. Base position can be optionally set here.
