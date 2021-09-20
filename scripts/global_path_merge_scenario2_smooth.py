#!/usr/bin/env python
import json
import numpy as np
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt
import numpy as np


#with open("path.txt", "r") as ins:
#    #path_global_1 = []
#    path_global_1 = [x.strip() for x in ins] #[tuple(map(float, i.split(','))) for i in ins]
#    #for line in ins:
#        #path_global_1.append([int(line) for line in line.split()])
#print(path_global_1)

with open("path_rrt_test__scenario2_smooth.txt", "r") as ins:
  path_global_1 = []
  for line in ins:
    path_global_1.append([float(line) for line in line.split()])

with open("path_rrt_reset_test_smooth_scenario2.txt", "r") as ins:
  path_global_2 = []
  for line in ins:
    path_global_2.append([float(line) for line in line.split()])
#pp = path_global_2[::-1]  
#print(pp)

with open("path_rrt_land_test_smooth_scenario2.txt", "r") as ins:
  path_global_3 = []
  for line in ins:
    path_global_3.append([float(line) for line in line.split()])
'''
with open("path_rrt_13.txt", "r") as ins:
  path_global_4 = []
  for line in ins:
    path_global_4.append([float(line) for line in line.split()])
'''
path_global_1.extend(path_global_2)
path_global_1.extend(path_global_3)
#path_global_1.extend(path_global_4)

f = open("path_global_scenario2_smooth.txt",'w')
f.writelines("%s\n" % i for i in path_global_1)
f.close()

plt.figure(figsize=(20, 20))
plt.plot(*zip(*path_global_1))
plt.show()
