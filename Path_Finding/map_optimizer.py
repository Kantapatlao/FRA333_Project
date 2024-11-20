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
            raise TypeError("in_map numpy array only takes int8 as dtype")
        
        # Check if input only contain 0 and 1
        if np.unique(in_map).size != 2:
            raise ValueError("in_map numpy array should only contain 0 and 1")
        
        if np.unique(in_map)[0] != 0 and np.unique(in_map)[1] != 1:
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
    

    # Collapsed near by map with same value by joining them together
    def __map_collapser(self):

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


    

    # Dump content in graph_map
    def dump_graph(self):
        self.__graph_dump_engine(self.tree_map)
        return None

    # Internal recursive engine
    def __graph_dump_engine(self, node):

        # Recursive stop condition: Node that don't have child
        if node.childA is None:
            print("Node: ", node)
            print("Position: ", node.data.posX, node.data.posY)
            print("Center Pos: ", node.data.get_center_pos())
            print("Size: ", node.data.sizeX, node.data.sizeY)
            print("Value: ", node.data.value)

            return None

        else:
            self.__graph_dump_engine(node.childA)
            self.__graph_dump_engine(node.childB)

        return None