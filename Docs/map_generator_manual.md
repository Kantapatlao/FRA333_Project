# map_generator.ipynb
**Object in map_generator.ipynp file**
- make_map

**Dependency package**
- math
- random
- numpy
- cv2 (opencv2)

## make_map()

**NAME:** \
make_map

**TYPE:**\
Custom function

**SYNOPSIS:** 
```
make_map(sizeX, sizeY, obstacles_count)
```

**DESCRIPTION**\
Create a grid map with specify size in *sizeX* and *sizeY*. Then obstacle is added by the amount specify in *obstacles_count*. If *obstacles_count* isn't specify, random number of obstacle between 1 to 5 will be placed on the map. Obstacle will be place on random coordinate inside the grid map with random size and random shape (shape from opencv's morphological structuring element).

**PARAMETER**
- sizeX, sizeY: grid size of the map. Both, each **only take unsigned integer as input.** This function doesn't automatically type-cast variable.
- obstacles_count: Amount of obstacle placed in the map. **Only take unsigned integer as input.** This function doesn't automatically type-cast variable.

**OPTIONS**\
- obstacles_count can be left out, which will random amount of obstacle from 1 to 5 to be placed in the map.

**EXAMPLES**
```
map = make_map(50, 50)
map = make_map(50, 50, 3)
```

**RETURN_VALUE**\
2D numpy.ndarray with size as specify by sizeX and sizeY. Available path is label as 0 and obstable label as 1.

**DIAGNOSTICS**
- `<input variable> only take positive integer as input.`
Input isn't the same type as what the function accept. Refer in PARAMETER topic. The function doesn't automatically type cast any variable, so floating point without decimal point isn't accept as well.

**BUGS**\
NONE
