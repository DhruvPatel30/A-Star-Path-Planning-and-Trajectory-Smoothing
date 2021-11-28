
# Implementation of A* and Path smoothing.

This project involves the implementation of A* planner. After obtaining the path, control commands (i.e. velocity to the left and right wheels) were provided to the turtlebot3 to follow the obtained trajectory.

Two controllers have been implemented, which can be found in the controllers folder.
***
    1. AstarPlanner - This implementation does not include path smoothing. The control commands are given to the robot with smoothing the path. So, the robot will take sharp 90 degree turns at the turning junctions.
    2. call_Astar   - This implementation smooths the sharp 90 degree turns using circular arc technique. 
***


## Environment

The environment consist of a 10*10 grid in WEBOTS, which can be seen below. The northwest corner is the start point of the robot and south east corner is the goal position of the robot. The black box denotes the obstacles in the environment. 

<img src="images/world.png" width="480">



