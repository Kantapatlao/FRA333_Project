import os
import numpy as np
import matplotlib.pyplot as plt
from map_optimizer import Map

dir_path = os.path.abspath("Map/")
file_name = ["map1.npy","map2.npy","map3.npy","map4.npy","map5.npy"]


# print(os.path.join(dir_path, file_name[0]))
m = np.load((os.path.join(dir_path, file_name[0])), allow_pickle=False)
foo = Map(m)
foo.show_graph()
print('*' * 30)
plt.imshow(m, cmap='grey', interpolation='nearest')
plt.axis('off')
plt.show()
     