#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>

#include <Eigen/Sparse>
#include "OsqpEigen/OsqpEigen.h"

using namespace Eigen;
using namespace std;

//state and input variables
const int nx = 3; //state variables
const int nu = 2; //input variables

class MPC{

public:
    MPC(ros::NodeHandle &nh);
    virtual ~MPC();

private:
	ros::NodeHandle nh_;

	vector<Eigen::Vector3d> ref_trajectory;
	vector<Eigen::VectorXd> ref_input;
	double current_speed;

	int N, nx, nu;
	double max_speed, max_steer, C_l, q_x, q_y, q_yaw, r_v, r_steer, Ts;

	Eigen::Matrix<double, nx, nx> Q;
    Eigen::Matrix<double, nu, nu> R;


public:

	void get_params(ros::NodeHandle& nh);
	void propagate_Dynamics(Eigen::VectorXd& state(nx), Eigen::VectorXd& input(nu), Eigen::VectorXd& next_state(nx), double dt);
	void get_linear_dynamics_car(Eigen::Matrix<double,nx,nx>& Ad, Eigen::Matrix<double,nx, nu>& Bd, Eigen::Matrix<double,nx,1>& hd, Eigen::Matrix<double,nx,1>& state, Eigen::Matrix<double,nu,1>& input);
	void get_half_space_constraints(vector<vector<double>>& constraint_points, double& A11, double& A12, double& A21, double &A22, double& B11; double& B12);
	void get_MPC_path(vector<Eigen::VectorXd>& ref_trajectory, vector<Eigen::VectorXd>& ref_input, double& current_speed);
	void execute_MPC_path(Eigen::VectorXd& QPSolution);
	void convert_waypoints_to_vector3d(vector<Waypoint>& waypoints);

};


