#!/usr/bin/env python
import json
import numpy as np
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt
import numpy as np


with open("path_rrt_start_test_smooth.txt", "r") as ins:
  path_global_1 = []
  for line in ins:
    path_global_1.append([float(line) for line in line.split()])

with open("path_cpp.txt", "r") as ins:
  path_global_2 = []
  for line in ins:
    path_global_2.append([float(line) for line in line.split()])
    
#pp = path_global_2[::-1]  


with open("path_rrt_reset_test_smooth.txt", "r") as ins:
  path_global_3 = []
  for line in ins:
    path_global_3.append([float(line) for line in line.split()])

with open("path_rrt_land_test_smooth.txt", "r") as ins:
  path_global_4 = []
  for line in ins:
    path_global_4.append([float(line) for line in line.split()])

path_global_1.extend(path_global_2)
path_global_1.extend(path_global_3)
path_global_1.extend(path_global_4)

f = open("global_path_interface_smooth.txt",'w')
f.writelines("%s\n" % i for i in path_global_1)
f.close()

#with open("global_path_flight.txt", 'w') as fp:
#	fp.write('\n'.join('%s %s' % x for x in path_global_1))

plt.figure(figsize=(20, 20))
plt.plot(*zip(*path_global_1))
plt.show()
