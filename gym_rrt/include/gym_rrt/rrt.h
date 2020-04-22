#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>

// Struct defining a Node object in the RRT tree
struct Node 
{
    Node() = default;
    
    Node(const double x, const double y, const int parent_idx):
        x(x), y(y), cost(0.0), parent_idx(parent_idx) {}

    Node(const double x, const double y, double cost, const int parent_idx):
        x(x), y(y), cost(cost), parent_idx(parent_idx) {}

    double x, y;
    double cost; // only for RRT*
    int parent_idx;
    bool is_root = false;
};

struct Waypoint
{
    double x, y;
    double heading, speed;

    Waypoint() = default;

    Waypoint(const geometry_msgs::PoseStamped::ConstPtr& pose_msg, double current_speed=0.1)
    {
        x = pose_msg->pose.position.x;
        y = pose_msg->pose.position.y;

        tf2::Quaternion q(pose_msg->pose.orientation.x,
                          pose_msg->pose.orientation.y,
                          pose_msg->pose.orientation.z,
                          pose_msg->pose.orientation.w);
        tf2::Matrix3x3 mat(q);

        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        heading = yaw;
        speed = current_speed;
    }

    Waypoint(const geometry_msgs::Pose& pose_msg, double current_speed=0.1)
    {
        x = pose_msg.position.x;
        y = pose_msg.position.y;

        tf2::Quaternion q(pose_msg.orientation.x,
                          pose_msg.orientation.y,
                          pose_msg.orientation.z,
                          pose_msg.orientation.w);
        tf2::Matrix3x3 mat(q);

        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        heading = yaw;
        speed = current_speed;
    }
};

class RRT
{
    public:
        RRT(ros::NodeHandle &nh);
        virtual ~RRT();

    private:
        ros::NodeHandle nh_;

        // Publishers and Subscribers
        ros::Subscriber pose_sub_;
        ros::Subscriber scan_sub_;
        ros::Publisher drive_pub_;
        ros::Publisher dynamic_pub_;
        ros::Publisher map_pub_;
        ros::Publisher tree_viz_pub_;

        // Visualization stuff
        ros::Publisher waypoint_viz_pub_;

        // tf stuff
        tf2_ros::TransformListener tf2_listener_;
        tf2_ros::Buffer tf_buffer_;
        geometry_msgs::TransformStamped tf_map_to_laser_;
        geometry_msgs::TransformStamped tf_laser_to_map_;

        nav_msgs::OccupancyGrid input_map_;

        // data parsing
        std::string filename_;
        std::string delimiter_;

        // create RRT params
        int max_iters_;
        int num_pts_collision_;
        double inflation_r_;
        double max_expansion_dist_;
        double goal_threshold_;
        double local_waypoint_tolerance_;
        double lookahead_d_;
        double local_lookahead_d_;

        std::vector<size_t> new_obstacles_;
        int clear_obstacles_count_;

        // Map frame pose
        double current_x_;
        double current_y_;

        // Global and local path
        std::vector<Waypoint> global_path_;
        std::vector<Node> local_path_;
        std::vector<std::vector<Waypoint>> trajectory_options_;
        int path_num_;

        // RRT*
        bool enable_rrt_star_;
        double search_radius_;
        

        // random generator
        std::mt19937 gen;
        std::uniform_real_distribution<> x_dist;
        std::uniform_real_distribution<> y_dist;

        // Subscriber callbacks
        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

        // RRT methods
        std::vector<double> sample();
        int nearestIdx(const std::vector<Node> &tree, const std::vector<double> &sampled_point);
        Node steer(const Node &nearest_node, const int nearest_node_index, const std::vector<double> &sampled_point);
        bool checkCollision(Node &nearest_node, Node &new_node);
        bool isGoal(Node &latest_added_node, double goal_x, double goal_y);
        std::vector<Node> findPath(std::vector<Node> &tree, Node &latest_added_node);
        bool isCollided(const double x_map, const double y_map);
        Waypoint findOptGlobalWaypoint(const Waypoint current_pose);
        std::vector<Waypoint> findWaypointOptions(const Waypoint current_pose);
        std::pair<Node, double> findLocalWaypoint(const Waypoint& current_pose);
        std::vector<int> expandObstacles(const double x_map, const double y_map);
        int getMapIdx(const double x_map, const double y_map);

        // RRT* methods
        double cost(std::vector<Node> &tree, Node &node);
        double lineCost(Node &n1, Node &n2);
        std::vector<int> near(std::vector<Node> &tree, Node &node);

        // Data parsing method
        std::vector<Waypoint> getOptimalWaypoints();
        std::vector<Waypoint> getTrackpoints(std::string filename);

        // Viz methods
        void visualize_trackpoints(double x_local, double y_local, double x_global, double y_global);
        void visualize_tree(std::vector<Node>& tree);
};
