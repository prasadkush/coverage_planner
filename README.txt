Instructions for building and running the package, taking input, visualization in rviz and 
a brief description of the code files.

Assumptions:

1) The environment is bounded by a convex polygon and contains obstacles as convex polygons
2) The robot can execute the motions up, left, down, right.


1. Making the coverage planner package:

This package is for ros kinetic. It may or may not run in newer or older versions.
Download the package in your catkin workspace and build the package using catkin make. 
It has dependencies already included in ros kinetic and on the boost library.


2. To run the package, execute the following command after having built the package:

rosrun coverage_planner coverage_planner_node

coverage_planner_node is name of the C++ executable


3. Visulization in rviz:

For visualizing in rviz, add 2 markers and a marker array as the display items.

The topics for markers are:
/visualization_marker
/visualization_marker_path

The topic for marker array is:
/visualization_marker_obs

Also add grid and axes for better display. Fixed frame will have the name 'map'.

NOTE: if topic of marker is added to marker array or vice-versa, ros will give an error.

4. Instructions for input from rviz:

The terminal will ask you to click points on rviz and then enter a digit.
To enter a polygon, enter its vertices by clicking points on rviz screen using the '2D Pose Estimate' button.
Adjacent vertices shall be entered in order. i.e., The points shall be clicked in either clockwise or counter-clockwise direction.
After entering polygon vertices, enter a digit on the terminal screen. If a convex polygon is not entered, the program will exit.

After entering the vertices, you shall now be able to visualize the polygon in blue.

Next, the terminal will ask you to enter number of obstacles. If a negative number is entered, the program will exit. 
The terminal will then ask you to enter obstacle points. As above, click points on rviz using '2D Pose Estimate' to form a convex polygon. After entering points for each obstacle, enter a digit.

After entering the obstacles as polygon vertices, you shall now be able to visualize the obstacles in green.

Once, done, the plan will be computed. The plan will be displayed in rviz in red.

5. Description of code files

The major source code lies in the file src/coverage_planner.cpp
The main function lies in src/coverage_planner_node.cpp
visualization code lies in include/coverage_planner/visualization.h
Data types for representing environments lie in environment.h and discr_environment.h








