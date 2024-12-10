# map_optimizer.py
**Object in map_optimizer.py file**
- BT_Node

- Discrete_map
    - get_center_pos
    - get_bottom_righ_pos
    - scale_discrete_map

- Map
    - find_adjacent_node
    - find_nearest_node
    - list_obstacle
    - show_graph

**Dependency package**
- math
- numpy


## BT_Node object

**NAME:** \
BT_Node

**TYPE:**\
Custom object

**SYNOPSIS:** 
```
BT_Node(self, data_in, posX_in, posY_in)
```

**DESCRIPTION**\
A binary tree node, served for discretizing the grid map.

**ATTRIBUTE**
- data: Data of the node. (Either hold ndarray for grid map and Discrete_map)
- posX: X coordinate of the node. (Only use when dividing map. When map is successfully divided, attributed is not used.)
- poxY: Y coordinate of the node. (Only use when dividing map. When map is successfully divided, attributed is not used.)
- childA, childB: child node of current instant Binary tree node. Each hold a single BT_Node object.


**OPTIONS**\
NONE

**EXAMPLES**
```
foo = BT_Node(np.zeros((5,5)), 10, 10)
```

**RETURN_VALUE**\
An binary tree node object. This function shouldn't failed.

**DIAGNOSTICS**\
NONE

**BUGS**\
NONE


## Discrete map object

**NAME:** \
Discrete_map

**TYPE:**\
Custom object

**SYNOPSIS:** 
```
Discrete_map(self, value_in, posX_in, posY_in, sizeX_in, sizeY_in)
```

**DESCRIPTION**\
An object to represent part of a map as a discrete map. Instead of explain each coordinate if it's an obstacle or not, group them (if they're all obstacle or not obstacle) up into a square with known position and size.


**ATTRIBUTE**
- value: Hold value if current discrete map object is obstacle(1) or free path(0).
- posX, posY: Hold top-left coordinate of current discrete map object.
- sizeX, sizeY: Hold width and height of current discrete map object.

**OPTIONS**\
NONE

**EXAMPLES**
```
foo = Discrete_map(1,10,20,15,25)
```

**RETURN_VALUE**\
A Discrete_map node object.

**DIAGNOSTICS**
NONE

**BUGS**
- Discrete_map node initialization doesn't perform type-check. Manually creating Discrete_map object may result in **error**.


## get_center_pos

**NAME:** \
get_center_pos

**TYPE:**\
Discrete_map's method

**SYNOPSIS:** 
```
get_center_pos(self)
```

**DESCRIPTION**\
Return coordinate of center position of current instant Discrete_map object.


**PARAMETER**
- self: Instance of Discrete_map object to be computed.

**OPTIONS**\
NONE

**EXAMPLES**
```
cen_x, cen_y = foo.get_center_pos()
```

**RETURN_VALUE**\
A tuple of X and Y (respectively) coordinate of center of the current Discrete_map object.

**DIAGNOSTICS**\
NONE

**BUGS**\
NONE

## get_bottom_right_pos

**NAME:** \
get_bottom_right_pos

**TYPE:**\
Discrete_map's method

**SYNOPSIS:** 
```
get_bottom_right_pos(self)
```

**DESCRIPTION**\
Return coordinate of bottom-right position of current instant Discrete_map object.


**PARAMETER**
- self: Instance of Discrete_map object to be computed.

**OPTIONS**\
NONE

**EXAMPLES**
```
x, y = foo.get_bottom_right_pos()
```

**RETURN_VALUE**\
A tuple of X and Y (respectively) coordinate of bottom-right of the current Discrete_map object.

**DIAGNOSTICS**\
NONE

**BUGS**\
NONE

## scale_discrete_map
**NAME:** \
scale_discrete_map

**TYPE:**\
Discrete_map's method

**SYNOPSIS:** 
```
scale_discrete_map(self, scale, x_offset, y_offset)
```

**DESCRIPTION**\
Return a new instance of Discrete_map object where all attribute are scaled and offset according to specify parameter.


**PARAMETER**
- self: Instance of Discrete_map object to be computed.
- scale: Scale multiplier to original instance.
- x_offset: How many unit discrete map shift from left side after scaling.
- y_offset: How many unit discrete map shift from top side after scaling.

**OPTIONS**\
NONE

**EXAMPLES**
```
bar = foo.scale_discete_map(10, 100, 600)
```

**RETURN_VALUE**\
New Discrete_map object with all attributed scaled and offsetted.

**DIAGNOSTICS**\
NONE

**BUGS**\
NONE


## Map ojbect
**NAME:** \
Map

**TYPE:**\
Custom object

**SYNOPSIS:** 
```
Map(self, in_map)
```

**DESCRIPTION**\
An object that hold and compute normal grid map into discrete grid map. The initialization process consume a grid map in form of numpy's ndarray then compute into discrete map and store inside the object.


**ATTRIBUTE**
- full_map: Full grid map in form of numpy's array as input at initialization of the object.
- tree_map: Placeholder to hold binary tree when turning grid map into discretized map. This attributed should never be accessed outside the object's method.
- optimized_map: List of discrete_map that represent the input grid map. Order of discrete_map may not be sorted.
- obstacle_list: Exclusively store discrete_map that have value of 1 (is obstacle). Redundant to the function "list_obstacle"

**PARAMETER**
- in_map: Grid map input in numpy's ndarray form. Value of 1 indicate obstacle and 0 as free path.

**OPTIONS**\
NONE

**EXAMPLES**
```
foo = Map(np.zeros((10,10)))
```

**RETURN_VALUE**\
New Map object.

**DIAGNOSTICS**
- `<input variable> only take <type> as input.`\
Input isn't the same type as what the function accept. Refer in PARAMETER topic. The function doesn't automatically type cast any variable, so floating point without decimal point isn't accept as well.
- `in_map numpy array only takes uint8 as dtype`\
in_map parameter only take numpy's ndarray with dtype of uint8 only. The function doesn't automatically type variable.
- `in_map numpy array should only contain 0 and 1`\
in_map parameter only take numpy's ndarray that only carry 0(free path) and/or 1(obstacle). Other number will raised this exception.

**BUGS**\
NONE

## find_adjacent_node
**NAME:** \
find_adjacent_node

**TYPE:**\
Map object's method

**SYNOPSIS:** 
```
find_adjacent_node(self, input_node)
```

**DESCRIPTION**\
List all discrete map that's adjacent to the left, right, top and bottom of the input node according to list of discrete map in Map object. The function does expect that input_node is also within Map object's list. Manually definde input_node may cause function to error. This function doesn't return discrete map that just touch the corner. 

**PARAMETER**
- input_node: Specify discrete map which is rectangular area to search for adjacent node around it. 

**OPTIONS**\
NONE

**EXAMPLES**
```
Node_list = foo.find_adjacent_node(node)
```

**RETURN_VALUE**\
List of discrete map object which is adjacent to the top, bottom, left and right of the input_node.

**DIAGNOSTICS**
- `<input variable> only take <type> as input.`\
Input isn't the same type as what the function accept. Refer in PARAMETER topic. The function doesn't automatically type cast any variable, so floating point without decimal point isn't accept as well.


**BUGS**\
NONE


## find_nearest_node
**NAME:** \
find_nearest_node

**TYPE:**\
Map object's method

**SYNOPSIS:** 
```
find_nearest_node(self, x, y)
```

**DESCRIPTION**\
From given x and y coordinate, find node in Map object that input coordinate is in. 

**PARAMETER**
- x, y: X, Y Coordinate which the method will search for node that specify coordinate is in.

**OPTIONS**\
NONE

**EXAMPLES**
```
Node = foo.find_nearest_node(self, 10, 20)
```

**RETURN_VALUE**\
A discrete map object which specify input target is in.

**DIAGNOSTICS**
- `<input variable> only take <type> as input.`\
Input isn't the same type as what the function accept. Refer in PARAMETER topic. The function doesn't automatically type cast any variable, so floating point without decimal point isn't accept as well.


**BUGS**\
NONE

## list_obstacle
**NAME:** \
list_obstacle

**TYPE:**\
Map object's method

**SYNOPSIS:** 
```
list_obstacle(self)
```

**DESCRIPTION**\
Dump every Discrete_map object which have value of 1 (is obstacle) from Map object optimized_map attribute.

**PARAMETER**\
NONE

**OPTIONS**\
NONE

**EXAMPLES**
```
Obstacle_list = foo.list_obstacle()
```

**RETURN_VALUE**\
A list of discrete map object which is obstacle (value of 1).

**DIAGNOSTICS**\
NONE

**BUGS**\
NONE

## show_graph
**NAME:** \
show_graph

**TYPE:**\
Map object's method

**SYNOPSIS:** 
```
show_graph(self)
```

**DESCRIPTION**\
Dump every Discrete_map object from Map object optimized_map attribute and print attribute of each Discrete_map object.

**PARAMETER**\
NONE

**OPTIONS**\
NONE

**EXAMPLES**
```
foo.show_graph()
```

**RETURN_VALUE**
NONE

**DIAGNOSTICS**\
NONE

**BUGS**\
NONE