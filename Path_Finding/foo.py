import os
import random
import numpy as np
import matplotlib.pyplot as plt
from map_optimizer import Map

dir_path = os.path.abspath("Map/")
file_name = "map1.npy"


m = np.load((os.path.join(dir_path, file_name)), allow_pickle=False)
foo = Map(m)

# PosX: 18,31 Size: 13,6
for i, n in enumerate(foo.optimized_map):
    if n.posX == 18 and  n.posY == 31:
        print(i)
        break