import os
import random
import numpy as np
import matplotlib.pyplot as plt
from map_optimizer import Map

dir_path = os.path.abspath("Map/")
file_name = "map1.npy"


m = np.load((os.path.join(dir_path, file_name)), allow_pickle=False)
foo = Map(m)
random_node = random.randint(0,len(foo.optimized_map) - 1)
# random_node = 27
test_case = foo.optimized_map[random_node]

new_m = np.copy(m)
print("Testcase node: ", random_node)
new_m[test_case.posY: test_case.posY + test_case.sizeY , test_case.posX: test_case.posX + test_case.sizeX ] = 3
print("Node: ",test_case)
print("PosX: ", test_case.posX)
print("PosY: ", test_case.posY)
print("SizeX: ", test_case.sizeX)
print("SizeY: ", test_case.sizeY)

adjacent_node = foo.find_adjacent_node(test_case)


for n in adjacent_node:
    new_m[n.posY: n.posY + n.sizeY , n.posX: n.posX + n.sizeX ] = 2
    print("Node: ",n)
    print("PosX: ", n.posX)
    print("PosY: ", n.posY)
    print("SizeX: ", n.sizeX)
    print("SizeY: ", n.sizeY)


plt.imshow(new_m)
plt.show()
    