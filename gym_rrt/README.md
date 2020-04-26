# RRT*: Multiple Tracks!

Here we implement a variation of RRT* algorithm in which we choose the next waypoint from a set of precomputed trajectories. At every step, we choose the next waypoint to track, by comparing the best waypoints from 5 different trajectories and follow the one which will avoid collisions as well as require the car to deviate from its current path the least.

The following parameters were tuned such that the collision frequency will be the least:
- SPEED: 1.0
- lookahead_d_: 1.8
- inflation_r_: 3
- goal_threshold_: 0.4

Changing these parameters might cause the car to collide with obstacles, irrespective of he map.

## Dependencies
- nav_msgs
- ackermann_msgs
- sensor_msgs
- geometry_msgs
- std_msgs
- tf
- tf2_ros
- tf2_geometry_msgs
- visualization_msgs


## How to run?
- If you do not have the gym environment, follow the instructions at the [link](https://github.com/f1tenth/f1tenth_gym_ros).
- Clone this repository and make sure everything builds while in master branch.
- In `gym_rrt/src/rrt.cpp` change the parameter `folder_path_` to the path of this repository on the system.
- Run `sudo ./docker.sh` in the gym folder.
- Run `roslaunch gym_rrt rrt_star.launch`

![Alt Text](https://github.com/akhibhat/f110_finalProject/raw/master/gym_rrt/rrt_star.gif)
