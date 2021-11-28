# A* Path Planning and Smoothing the path.

Implemented A* path planner and smoothed the curves of the path.

The repository includes:
    1. controllers
    2. robot_model
    3. worlds

controllers folder has 2 controllers in it
    1. AStarPlanner - This controller makes the robot follows the path obtained by A* planner without smoothing the curves in path.
    2. call_AStar   - This controller obtains the path from A* planner, smoothens the curves in path and makes robot follows the smoothened path.