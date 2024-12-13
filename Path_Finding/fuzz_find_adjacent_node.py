import os
import random
import numpy as np
import matplotlib.pyplot as plt
from map_optimizer import Map

dir_path = os.path.abspath("Map/")
file_name = ["map1.npy","map2.npy","map3.npy","map4.npy","map5.npy"]

result_path = os.path.abspath("Adjacent Node/")

file_iter = 0

m = np.load((os.path.join(dir_path, file_name[file_iter])), allow_pickle=False)
foo = Map(m)

for i, node in enumerate(foo.optimized_map[:2]):
    new_m = np.copy(m)    
    new_m[node.posY: node.posY + node.sizeY , node.posX: node.posX + node.sizeX ] = 3

    adjacent_node = foo.find_adjacent_node(node)

    for n in adjacent_node:
        new_m[n.posY: n.posY + n.sizeY , n.posX: n.posX + n.sizeX ] = 2
    


    plt.imshow(new_m)
    plt.savefig(os.path.join(result_path, f'map1_node{i}.png'), format='png')
    # plt.show()
    