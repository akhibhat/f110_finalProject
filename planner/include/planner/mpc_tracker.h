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

public:

	void getParams(ros::NodeHandle& nh);
	void getReferenceTraj();
	void getDynamics();
	void getMPCpath();
	void publishMPCpath();
	void visualizeMPC();

};


