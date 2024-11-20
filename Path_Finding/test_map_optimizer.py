import numpy as np
from map_optimizer import Map



a = np.zeros((3,4))
b = np.ones((3,4))
c = np.zeros((6,2))
a = np.concatenate((a,b))
a = np.concatenate((a,c),axis=1)
a = a.astype(np.uint8)
print(a)
print('*' * 30)
foo = Map(a)
# foo.dump_graph()
print(foo.optimized_map)