import math
import numpy as np

# Binary tree object
class BT_Node:
    def __init__(self, data_in, posX_in, posY_in):
        
        self.data = data_in
        self.posX = posX_in
        self.posY = posY_in
        self.childA = None
        self.childB = None
        return None


# Discrete map object
class Discrete_map:
    def __init__(self, value_in, posX_in, posY_in, sizeX_in, sizeY_in):

        self.value = value_in
        self.posX = posX_in
        self.posY = posY_in
        self.sizeX = sizeX_in
        self.sizeY = sizeY_in

        return None

    def get_center_pos(self):

        x = self.posX + math.floor(self.sizeX / 2)
        y = self.posY + math.floor(self.sizeY / 2)

        return (x,y)
    

    def get_buttom_right_pos(self):

        return (self.posX + self.sizeX - 1, self.posY + self.sizeY - 1)


class Map:
    
    def __init__(self, in_map):

        MAX_COLLAPSE = 50
        
        # sanitize map input
        if type(in_map) is not np.ndarray:
            raise TypeError("in_map only take np.ndarray as input")
        
        if in_map.dtype != np.uint8:
            raise TypeError("in_map numpy array only takes uint8 as dtype")
        
        # Check if input only contain 0 and 1
        if np.unique(in_map).size == 1:
            if np.unique(in_map)[0] != 0 and np.unique(in_map)[0] != 1:
                raise ValueError("in_map numpy array should only contain 0 and 1")
        
        elif np.unique(in_map).size == 2:
            if np.unique(in_map)[0] != 0 or np.unique(in_map)[1] != 1:
                raise ValueError("in_map numpy array should only contain 0 and 1")
            
        else:
            raise ValueError("in_map numpy array should only contain 0 and 1")
        # Declare "Map" object's attribute
        
        # full_map: Hold grid map in form of numpy.ndarray
        # tree_map: Hold map in form of discretized map, inside binary tree
        # optimized_map: Hold discrete grid map in graph structure
        
        
        self.full_map = in_map
        self.tree_map = self.__map_discretizer_engine(BT_Node(self.full_map, 0, 0))
        self.optimized_map = self.__graph_dump_engine()

        # Since __map_collapser() can't garantee fully collapsed map in one iteration
        
        last_node_size = len(self.optimized_map)
        self.optimized_map = self.__map_collapser(self.optimized_map)
        now_node_size = len(self.optimized_map) 
        collapse_iter = 0

        # Keep looping until map won't collapse anymore 
        while last_node_size != now_node_size:

            last_node_size = now_node_size
            self.optimized_map = self.__map_collapser(self.optimized_map)
            now_node_size = len(self.optimized_map)

            # Max iteration stop condition
            if collapse_iter > MAX_COLLAPSE:
                print("Warning: Max iteration for doing map collapse is reached. Returning result.")
                break

            collapse_iter = collapse_iter + 1 

        
        return None
                
        
    
            

    # Turn map into discrete grid map
    def __map_discretizer_engine(self, node):

        # Recursive stop condition: If input map have the same value (if it's a single dot, it'll always have 1 value).
        if np.unique(node.data).size == 1:

            # Turn np.ndarray to discrete_map obj
            node.data = Discrete_map(
                value_in = np.unique(node.data)[0],
                posX_in = node.posX,
                posY_in = node.posY,
                sizeX_in = node.data.shape[1],
                sizeY_in = node.data.shape[0]
            )
            return node

        # Half the map, compute property, then delete data hold by parent node
        # Half wider side
        if (node.data.shape[0] > node.data.shape[1]):
            # Half row
            half_point = math.floor(node.data.shape[0] / 2)
            map_for_a = node.data[:half_point, :]
            map_for_b = node.data[half_point:, :]
            a_x = node.posX
            a_y = node.posY
            b_x = node.posX
            b_y = node.posY + half_point
            
            
        else:
            # Half column
            half_point = math.floor(node.data.shape[1] / 2)
            map_for_a = node.data[:, :half_point]
            map_for_b = node.data[:, half_point:]
            a_x = node.posX
            a_y = node.posY
            b_x = node.posX + half_point
            b_y = node.posY

        # Clean up data and assign to child node
        node.data = None
        node.childA = self.__map_discretizer_engine(BT_Node(map_for_a, a_x, a_y))
        node.childB = self.__map_discretizer_engine(BT_Node(map_for_b, b_x, b_y))

        return node
    

    # Join near by map with same value together
    # Note: Does not garantee to finished in single iteration
    def __map_collapser(self, graph):

        # Max iteration constant
        MAX_ITER = 10000


        # Get each discrete map node in form of list, seperate to two piles
        map_zero = []
        map_one = []

        for node in graph:
            if node.value == 1:
                map_one.append(node)

            elif node.value == 0:
                map_zero.append(node)

            else:
                raise ValueError(f'Unknown value: {node.value} in discrete map array.')
            


        # Collapse adjacent map (with value = 0) back together
        iter = 0

        # Iterate through each node
        while len(map_zero) > iter + 1:

            # Max iteration stop condition
            if iter > MAX_ITER:
                raise RuntimeError("Max iteration for collapsing map node is reached.")
            
            # Calculate position of adjacent node to search through
            near_node_column = (map_zero[iter].posX + map_zero[iter].sizeX, map_zero[iter].posY)
            near_node_row = (map_zero[iter].posX, map_zero[iter].posY + map_zero[iter].sizeY)

            # Search for adjacent node
            for node in map_zero[iter:]:
                
                # If found a node adjacent in the same row (but next column)
                if node.posX == near_node_column[0] and node.posY == near_node_column[1]:
                    
                    # Check if the row size are same if yes, join them together
                    if node.sizeY == map_zero[iter].sizeY:
                        
                        map_zero[iter].sizeX = map_zero[iter].sizeX + node.sizeX
                        map_zero.remove(node)
                        iter = iter - 1
                    
                        break

                # If found a node adjacent in the same column (but next row)
                if node.posX == near_node_row[0] and node.posY == near_node_row[1]:
                    
                    # Check if the column size are same if yes, join them together
                    if node.sizeX == map_zero[iter].sizeX:
                         
                        map_zero[iter].sizeY = map_zero[iter].sizeY + node.sizeY
                        map_zero.remove(node)
                        iter = iter - 1

                        break

            iter = iter + 1

        # Collapse adjacent map (with value = 1) back together
        iter = 0

        # Iterate through each node
        while len(map_one) > iter + 1:

            # Max iteration stop condition
            if iter > MAX_ITER:
                raise RuntimeError("Max iteration for collapsing map node is reached.")
            

            # Calculate position of adjacent node to search through
            near_node_column = (map_one[iter].posX + map_one[iter].sizeX, map_one[iter].posY)
            near_node_row = (map_one[iter].posX, map_one[iter].posY + map_one[iter].sizeY)

            # Search for adjacent node
            for node in map_one[iter:]:

                # If found a node adjacent in the same row (but next column)
                if node.posX == near_node_column[0] and node.posY == near_node_column[1]:
                    
                    # Check if the row size are same if yes, join them together
                    if node.sizeY == map_one[iter].sizeY:
                        
                        map_one[iter].sizeX = map_one[iter].sizeX + node.sizeX
                        map_one.remove(node)
                        iter = iter - 1
                    
                        break

                # If found a node adjacent in the same column (but next row)
                if node.posX == near_node_row[0] and node.posY == near_node_row[1]:
                    
                    # Check if the column size are same if yes, join them together
                    if node.sizeX == map_one[iter].sizeX:
                        
                        map_one[iter].sizeY = map_one[iter].sizeY + node.sizeY
                        map_one.remove(node)
                        iter = iter - 1

                        break

            iter = iter + 1
            

        return map_zero + map_one
    

    # Find adjacent node of input node
    def find_adjacent_node(self, input_node):

        # Check input type
        if type(input_node) is not Discrete_map:
            raise TypeError("input_node only take Discrete_map object as input.")
        

        # Declare a list to store adjacent node
        adjacent_node = []

        # find node that is adjacent to the right (ignore Y position(posY) for now)
        candidate_node = []
        for n in self.optimized_map:
            if n.posX == input_node.get_buttom_right_pos()[0] + 1 and n != input_node and n.value == 0:
                candidate_node.append(n)
        
        candidate_node = sorted(candidate_node, key = lambda n: n.posY)

        # Linear search adjacent node from canidate list
        did_truncate = False
        
        for i, n in enumerate(candidate_node):

            if n.posY == input_node.posY:
                candidate_node = candidate_node[i:]
                did_truncate = True
                break

            elif n.posY > input_node.posY:
                candidate_node = candidate_node[i-1:]
                did_truncate = True
                break

        if not did_truncate:
            candidate_node = candidate_node[len(candidate_node)-1:]

        for i, n in enumerate(candidate_node):

            if n.posY == input_node.posY + input_node.sizeY:
                candidate_node = candidate_node[:i+1]
                break

            elif n.posY > input_node.posY + input_node.sizeY:
                candidate_node = candidate_node[:i]
                break

            
        adjacent_node = adjacent_node + candidate_node

        # Adjacent to the bottom
        # find node that is adjacent to the bottom (ignore X position(posX) for now)
        candidate_node = []
        for n in self.optimized_map:
            if n.posY == input_node.get_buttom_right_pos()[1] + 1 and n != input_node and n.value == 0:
                candidate_node.append(n)
        
        candidate_node = sorted(candidate_node, key = lambda n: n.posX)

        # Linear search adjacent node from canidate list
        did_truncate = False
        for i, n in enumerate(candidate_node):

            if n.posX == input_node.posX:
                candidate_node = candidate_node[i:]
                did_truncate = True
                break

            elif n.posX > input_node.posX:
                candidate_node = candidate_node[i-1:]
                did_truncate = True
                break

        
        if not did_truncate:
            candidate_node = candidate_node[len(candidate_node)-1:]


        for i, n in enumerate(candidate_node):

            if n.posX == input_node.posX + input_node.sizeX:
                candidate_node = candidate_node[:i+1]
                break

            elif n.posX > input_node.posX + input_node.sizeX:
                candidate_node = candidate_node[:i]
                break

            
        adjacent_node = adjacent_node + candidate_node
        
        
        # Adjacent to the left
        
        
        
        # Adjacent to the top





        # return candidate_node
        return adjacent_node


    # Show content in graph_map
    def show_graph(self):

        # Show info of each node
        for node in self.optimized_map:
            print("Node: ", node)
            print("Position: ", node.posX, node.posY)
            print("Center Pos: ", node.get_center_pos())
            print("Size: ", node.sizeX, node.sizeY)
            print("Value: ", node.value)

        return None


    # Breadth first search engine
    def __graph_dump_engine(self):
        
        MAX_ITER = 100000

        # Breadth first search queue
        node_queue = [self.tree_map]
        selected_node = []
        queue_iter = 0

        # Go through list, append 
        while queue_iter + 1 <= len(node_queue):

            # Stop if maximum iteration is reached
            if queue_iter > MAX_ITER:
                raise StopIteration("Maximum iteration limit reached.")


            if node_queue[queue_iter].data is not None:
                selected_node.append(node_queue[queue_iter].data)

            if node_queue[queue_iter].childA is not None:
                node_queue.append(node_queue[queue_iter].childA)
            
            if node_queue[queue_iter].childB is not None:
                node_queue.append(node_queue[queue_iter].childB)

            queue_iter = queue_iter + 1



        return selected_node



        

    