
import math
import RobotARM.constant as R_const
from Path_Finding.map_optimizer import Map, Discrete_map
from RobotARM.robot import RobotArm

class A_Star:

    # Define each A* node
    class _A_Star_Node:
        
        # initialized a node
        def __init__(self, parent, sol, map_node, cost):

            self.parent = parent
            self.parent_sol = sol
            self.map_node = map_node

            # This is evaluation value: Cost(how much joint change) and Heuristic(Eucledian distance)
            self.cost = cost

    # initialized the engine
    def __init__(self):

        self.graph = []
        self.end_node = []
        self.solution = []


    # From start node, compute path
    def compute_path(self, goal_x, goal_y, map_input, robot_input):

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

        # Find best config reach first node
        for sol in robot_input.sequencial_IK_3(start_node.get_center_pos()[0] * R_const.SCALING, start_node.get_center_pos()[1] * R_const.SCALING):

            robot_input.forward_kinematic(sol)

            if robot_input.check_wall_collision(R_const.MAP_COORDINATE_X, R_const.MAP_COORDINATE_Y, R_const.MAP_SIZE_X, R_const.MAP_SIZE_Y) == False:
                if robot_input.check_object_collision(map_input.list_obstacle()) == False:
                    
                    cost = sum([abs(n-s) for n,s in zip(sol, start_config)])
                    heuristic = math.dist([robot_input.links[-1].end_positionX, robot_input.links[-1].end_positionY],[goal_x, goal_y])
                    self.graph = [self._A_Star_Node(None, sol, start_node, cost+heuristic)]
                    break

        if len(self.graph) == 0:
            raise RuntimeError("All config collide with wall/obstacle.")
        
        # Find goal node
        goal_node = map_input.find_nearest_node(math.floor(goal_x / R_const.SCALING), math.floor(goal_y / R_const.SCALING))


        return 0
    
        # Do A* search
        iter_count = 0

        while True:

            # Increment counter
            iter_count = iter_count + 1

            # Prevent runaway loop
            if iter_count > MAX_ITER:
                raise RuntimeError("Maximum iteration reached.")
            

            # Expand all node & all solution
            nearby_node = map_input.find_adjacent_node(self.graph[0].map_node)
            last_joint_config = self.graph

            # Calculate cheapest node
            nearby_node 
         
         
                
        

        


    def show_path(self):



        pass


    



    
        

        