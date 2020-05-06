# A Heirarchical Control Strategy for Multi-Agent Autonomous Racing

We use a Heirarchical Controller which employs a two-level structure: a high level planner which generates a reference trajectory that maiximizes the progress on the track ensuring obstacle avoidance. Next, we use MPC control technique for tracking the reference trajectory obtained from the planner and also try pure-pursuit control algorithm.

The file `src/planner.src` contains the code for the planner and generates a solution to the MPC problem. The solution consisting of the inputs, i.e. the steering angle and the velocity input, is then published as a custom message `Inputs.msg` over the `mpc_inputs` topic. The file `scripts/execute_mpc.py` subscribes to these inputs. If the mpc solution is feasible, it executes the most recent solution. Else, it continues executing the previous solution till it receives the next feasible solution.



## Planner

We use a modified follow-the-gap planner to come up with the best waypoint to pursue. The waypoints are selected from a set of five tracks around the race circuit which are pre-computed. The data can be found in the [`waypoints_data`](https://github.com/akhibhat/f110_finalProject/tree/master/waypoints_data) folder in the repository.

## MPC

To ensure dynamic feasibility, we track the reference trajectory using a MPC formulation, penalizing the deviation from reference trajecotry while satisfying workspace constraints. The constraints are obtained by finding the biggest gap around the best waypoint to follow. We employ the kinematic car model to cpature the car dynamics.

We use a 

## Pure Pursuit

We have also implemented a pure pursuit version of our code which runs surprisingly well. For this, we have two different modes of speed. When the opponent car is in front of us nearby, we switch to a low speed mode in order to avoid any kind of collision. On the other hand, when we are ahead of the opponent after overtaking, we switch to a high speed mode so that we can leave it behind.

Currently, we are in Pure Pursuit mode and in order to switch to MPC mode, you will need to toggle the parameter `use_mpc` in the file `config/params.yaml`

## Dependencies

- `osqp`
- `osqp-eigen`
- `Eigen`

## How to run?

- If you do not have the gym environment, follow the instructions at the [link](https://github.com/f1tenth/f1tenth_gym_ros)
- Clone this repository and build it. If it builds, you can skip step 3.
- If you couldn't build, you probably need the `osqp` and `osqp-eigen` libraries. Run the `get_osqp.sh` script with the command `sudo bash get_osqp.sh`
- Make sure the repository builds while in master branch
- In `config/params.yaml` replace the value of the parameter `folder_path` with the path to this repository on your device.
- Launch the gym environment by running `./docker.sh` in the gym folder
- In a different terminal window (source it first!), run the command `roslaunch planner planner.launch`
