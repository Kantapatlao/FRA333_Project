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


class Map:
    
    def __init__(self, in_map):
        
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
        self.optimized_map = self.__map_collapser()
        
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
    def __map_collapser(self):

        # Max iteration constant
        MAX_ITER = 10000


        # Get each discrete map node in form of list, seperate to two piles
        map_zero = []
        map_one = []

        for node in self.__graph_dump_engine():
            if node.data.value == 1:
                map_one.append(node)

            elif node.data.value == 0:
                map_zero.append(node)

            else:
                raise ValueError(f'Unknown value: {node.data.value} in discrete map array.')
            


        # Collapse adjacent map (with value = 0) back together
        iter = 0

        # Iterate through each node
        while len(map_zero) > iter + 1 and iter < MAX_ITER:

            # Max iteration stop condition
            if iter < MAX_ITER:
                raise RuntimeError("Max iteration for collapsing map node is reached.")
            
            # Calculate position of adjacent node to search through
            near_node_column = (map_zero[iter].posX + map_zero[iter].data.sizeX, map_zero[iter].posY)
            near_node_row = (map_zero[iter].posX, map_zero[iter].posY + map_zero[iter].data.sizeY)

            # Search for adjacent node
            for node in map_zero[iter:]:
                
                # If found a node adjacent in the same row (but next column)
                if node.posX == near_node_column[0] and node.posY == near_node_column[1]:
                    
                    # Check if the row size are same if yes, join them together
                    if node.data.sizeY == map_zero[iter].data.sizeY:
                        
                        map_zero[iter].data.sizeX = map_zero[iter].data.sizeX + node.data.sizeX
                        map_zero.remove(node)
                        iter = iter - 1
                    
                        break

                # If found a node adjacent in the same column (but next row)
                if node.posX == near_node_row[0] and node.posY == near_node_row[1]:
                    
                    # Check if the column size are same if yes, join them together
                    if node.data.sizeX == map_zero[iter].data.sizeX:
                         
                        map_zero[iter].data.sizeY = map_zero[iter].data.sizeY + node.data.sizeY
                        map_zero.remove(node)
                        iter = iter - 1

                        break

            iter = iter + 1

        # Collapse adjacent map (with value = 1) back together
        iter = 0

        # Iterate through each node
        while len(map_one) > iter + 1 and iter < MAX_ITER:

            # Max iteration stop condition
            if iter < MAX_ITER:
                raise RuntimeError("Max iteration for collapsing map node is reached.")
            

            # Calculate position of adjacent node to search through
            near_node_column = (map_one[iter].posX + map_one[iter].data.sizeX, map_one[iter].posY)
            near_node_row = (map_one[iter].posX, map_one[iter].posY + map_one[iter].data.sizeY)

            # Search for adjacent node
            for node in map_one[iter:]:

                # If found a node adjacent in the same row (but next column)
                if node.posX == near_node_column[0] and node.posY == near_node_column[1]:
                    
                    # Check if the row size are same if yes, join them together
                    if node.data.sizeY == map_one[iter].data.sizeY:
                        
                        map_one[iter].data.sizeX = map_one[iter].data.sizeX + node.data.sizeX
                        map_one.remove(node)
                        iter = iter - 1
                    
                        break

                # If found a node adjacent in the same column (but next row)
                if node.posX == near_node_row[0] and node.posY == near_node_row[1]:
                    
                    # Check if the column size are same if yes, join them together
                    if node.data.sizeX == map_one[iter].data.sizeX:
                        
                        map_one[iter].data.sizeY = map_one[iter].data.sizeY + node.data.sizeY
                        map_one.remove(node)
                        iter = iter - 1

                        break

            iter = iter + 1
            

        return map_zero + map_one
    

    # Show content in graph_map
    def show_graph(self):

        # get populated graph in form of list
        graph = self.optimized_map

        # Show info of each node
        for node in graph:
            print("Node: ", node)
            print("Position: ", node.data.posX, node.data.posY)
            print("Center Pos: ", node.data.get_center_pos())
            print("Size: ", node.data.sizeX, node.data.sizeY)
            print("Value: ", node.data.value)

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
                selected_node.append(node_queue[queue_iter])

            if node_queue[queue_iter].childA is not None:
                node_queue.append(node_queue[queue_iter].childA)
            
            if node_queue[queue_iter].childB is not None:
                node_queue.append(node_queue[queue_iter].childB)

            queue_iter = queue_iter + 1



        return selected_node



        

    