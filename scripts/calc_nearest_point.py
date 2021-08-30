#!/usr/bin/env python

import numpy as np
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt


uav_location = input('uav_location:') #[1000,-900]
#def calc_point_of_entery (uav_location = [11,11]):
with open("path_cpp.txt", "r") as ins:
    path_cpp = []
    for line in ins:
        path_cpp.append([float(line) for line in line.split()])# here send a list path_cpp
samples = np.array(path_cpp)
samples.sort(kind='quicksort')
tree = KDTree(samples)
uav_location_ = np.array([uav_location]) #here pud a goal pos after rrt
closest_dist, closest_id = tree.query(uav_location_, k=1)  # closest point with respect to the uav location
point_of_entery = samples[closest_id]
print("The closest point is: ", point_of_entery)
new_path_cpp = path_cpp[int(closest_id):]
new_path_cpp.extend(path_cpp[:int(closest_id)])

#print(new_path_cpp)
#    return point_of_entery, new_path_cpp

with open('/home/valeriia/UAV_Swarm_gazebo/catkin_ws/src/coverage_planner/scripts/path_cpp_new.txt', 'w') as fp:
		fp.write('\n'.join('%s ' % x for x in new_path_cpp))


plt.figure(figsize=(20, 20))
plt.plot(*zip(*path_cpp))
plt.grid(True)
plt.show()
