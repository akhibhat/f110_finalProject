# PURE PURSUIT!

Here we implement the Pure Pursuit algorithm to follow a precomputed set of waypoints. 

The waypoints were collected by running follow-the-gap algorithm on the Skirkanich map in the F1/10 simulator and recording the pose messages. These waypoints were then followed by the car with the following parameters:
- lookahead_distance: 1.8

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
- If you do not have the gym environment, follow the instructions at the [link](https://github.com/f1tenth/f1tenth_gym_ros)
- Clone this repository and make sure everything builds
- In `gym_pp/src/pure_pursuit.cpp`, change the parameter `filename_` to the path to the data file.
- Run `sudo ./docker.sh` in the gym folder
- Run `roslaunch gym_pp gym_pp.launch`
