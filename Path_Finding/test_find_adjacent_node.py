import os
import random
import numpy as np
import matplotlib.pyplot as plt
from map_optimizer import Map

dir_path = os.path.abspath("Map/")
file_name = "map1.npy"


m = np.load((os.path.join(dir_path, file_name)), allow_pickle=False)
foo = Map(m)
test_case = foo.optimized_map
# for _ in range(10):
#     test_case.append(foo.optimized_map[random.randint(0, len(foo.optimized_map))])
# print(len(test_case))
print('*' * 30)

foo.show_graph()
    