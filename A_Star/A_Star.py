
import math
import RobotARM.constant as R_const
from Path_Finding.map_optimizer import Map, Discrete_map
from RobotARM.robot import RobotArm

class A_Star:

    # Define each A* node
    class _A_Star_Node:
        
        # initialized a node
        def __init__(self, parent, sol, map_node, cost, heuristic):

            self.parent = parent
            self.parent_sol = sol
            self.sol_no = 0
            self.map_node = map_node

            # Cost: How much joint angle changes
            self.cost = cost
            # Heuristic: Eucledian distance function
            self.heuristic = heuristic

    # initialized the engine
    def __init__(self):

        self.graph_path = []
        self.end_node = []
        self.solution = []


    # From start node, compute path
    def compute_path(self, goal_x, goal_y, map_input, robot_input):

        # Function to find config
        def _find_config(map_node):
            IK_solution = robot_input.sequencial_IK_3(map_node.get_center_pos()[0] * R_const.SCALING, map_node.get_center_pos()[1] * R_const.SCALING)
            possible_solution = []
            for sol in IK_solution:

                robot_input.forward_kinematic(sol)

                if robot_input.check_wall_collision(R_const.MAP_COORDINATE_X, R_const.MAP_COORDINATE_Y, R_const.MAP_SIZE_X, R_const.MAP_SIZE_Y) == False:
                    if robot_input.check_object_collision(map_input.list_obstacle()) == False:
                    
                        possible_solution.append(sol)

            if len(possible_solution) == 0:
                raise RuntimeError("All config collide with wall/obstacle.")

            return possible_solution
        

        # Function to calculate cost
        def _calculate_cost(last_conf, now_conf):

            return sum([abs(n-l) for n,l in zip(now_conf, last_conf)])




        MAX_ITER = 10000

        # Check input type
        if type(goal_x) is not int:
            raise TypeError("goal_x only take unsigned integer as input.")
        
        if goal_x < 0:
            raise ValueError("goal_x's value can't be negative.")
        
        if type(goal_y) is not int:
            raise TypeError("goal_y only take unsigned integer as input.")
        
        if goal_y < 0:
            raise ValueError("goal_y's value can't be negative.")

        if type(map_input) is not Map:
            raise TypeError("map_input only take Map object as input.")
        
        if type(robot_input) is not RobotArm:
            raise TypeError("robot_input only take RobotArm object as input.")
        
        # Save current joint config
        start_x = robot_input.links[-1].end_positionX
        start_y = robot_input.links[-1].end_positionY
        start_config = [q.angle for q in robot_input.links]

        # Find nearest node
        start_node = map_input.find_nearest_node(math.floor(start_x / R_const.SCALING), math.floor(start_y / R_const.SCALING))

        # Find goal node
        goal_node = map_input.find_nearest_node(math.floor(goal_x / R_const.SCALING), math.floor(goal_y / R_const.SCALING))

        # If node is the same
        if start_node == goal_node:
            IK_solution = robot_input.sequencial_IK_3(goal_x, goal_y)
            possible_solution = []
            for sol in IK_solution:

                robot_input.forward_kinematic(sol)

                if robot_input.check_wall_collision(R_const.MAP_COORDINATE_X, R_const.MAP_COORDINATE_Y, R_const.MAP_SIZE_X, R_const.MAP_SIZE_Y) == False:
                    if robot_input.check_object_collision(map_input.list_obstacle()) == False:
                    
                        possible_solution.append(sol)

            if len(possible_solution) == 0:
                raise RuntimeError("All config collide with wall/obstacle.")

            robot_input.forward_kinematic(start_config)
            return possible_solution[0]

        # Find best config to reach first node
        for sol in _find_config(start_node):
            robot_input.forward_kinematic(sol)
            self.graph_path.append(self._A_Star_Node( None,
                                                      sol,
                                                      start_node,
                                                      _calculate_cost(start_config, sol), 
                                                      math.dist((robot_input.links[-1].end_positionX, robot_input.links[-1].end_positionY),
                                                                (goal_x, goal_y))
                                                    ))
            
        # Sorted graph_path according to evaluation function: cost + heuristic
        
        
    
        # Do A* search
        iter_count = 0
        best_goal = None

        while True:

            # Increment counter
            iter_count = iter_count + 1

            # Prevent runaway loop
            if iter_count > MAX_ITER:
                raise RuntimeError("Maximum iteration reached.")
            
            # Check if goal is reached
            if self.graph_path[0].map_node == goal_node:
                
                # Check if the goal is the best one
                if self.graph_path[0] == best_goal:
                    return self.solution
                
                else:
                    best_goal = self.graph_path[0]
            
            # Expand all node & all solution
            nearby_node = map_input.find_adjacent_node(self.graph_path[0].map_node)
            last_joint_config = self.graph_path[0].parent_sol

            # Find best config reach next node
            
                

            # Calculate cheapest node
            # nearby_node 
         
         
                
        

        


    def show_path(self):



        pass


    



    
        

        