#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>

#include <fstream>
#include <utility>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>

#include <dynamics/vehicle_state.h>

class Planner
{
    public:

        Planner(ros::NodeHandle &nh) : tf_listener_(tf_buffer_)
        {
            nh_ = nh;
            lookahead_d_ = 1.0;

            delimiter_ = ",";
            filename_ = "/home/akhilesh/f110_ws/src/final_project/data/pp.csv";

            ROS_INFO("Initialized constant!");
        }

        ~Planner(){}

        void initialize()
        {
            ROS_INFO("Initializing publishers and subscribers...");

            drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
            waypoint_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("waypoint_markers", 100);
            map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/cost_map", 1)

            ego_pose_sub_ = nh_.subscribe("/gt_pose", 1, &Planner::egoPoseCallback);
            scan_sub_ = nh_.subscribe("/scan", 1, &Planner::scanCallback);
            opp_odom_sub = nh_.subscribe("/opp_racecar/odom", 1, &Planner::oppOdomCallback);

            input_map_ = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(2)));

            if (input_map_.data.empty())
            {
                ROS_ERROR("Empty map received :(");
            }

            ROS_INFO("Received first map!");

            try
            {
                tf_laser_to_map_ = tf_buffer_.lookupTransform("map", "laser", ros::Time(0));
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }

            ROS_INFO("Reading waypoint data...");
            global_path_ = get_data();

            ROS_INFO("Stored the different tracks as a vector of vector of Waypoints");
        }

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

            Waypoint(const geometry_msgs::PoseStamped& pose_msg, double current_speed=0.1)
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

        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
        {
            updateStaticMap(scan_msg);


        }

        void egoPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
        {

        }

        void oppOdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
        {

        }


        void updateStaticMap(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
        {
            try
            {
                tf_laser_to_map_ = tf_buffer_.lookupTransform("map", "laser", ros::Time(0));
            }
            catch(tf::TransformException& e)
            {
                ROS_ERROR("%s", e.what());
                ros::Duration(0.1).sleep();
            }

            const auto translation = tf_laser_to_map_.transform.translation;
            const auto orientation = tf_laser_to_map_.transform.rotation;

            tf2::Quaternion q(orientation.x,
                              orientation.y,
                              orientation.z,
                              orientation.w);
            tf2::Matrix3x3 mat(q);

            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);

            const double angle_increment = scan_msg->angle_increment;
            const auto start = static_cast<int>(scan_msg->ranges.size()/6);
            const auto end = static_cast<int>(5*scan_msg->ranges.size()/6);
            double theta = scan_msg->angle_min + start*angle_increment;

            for (int i=start, i<end, i++)
            {
                const double hit = scan_msg->ranges[i];
                if (std::isnan(hit) || std::isinf(hit)) continue;

                const double x_base_link = hit * cos(theta);
                const double y_base_link = hit * sin(theta);

                const double x_map = x_base_link*cos(yaw) - y_base_link*sin(yaw) + translation.x;
                const double y_map = x_base_link*sin(yaw) + y_base_link*cos(yaw) + translation.y;

                std::vector<int> obstacleIdx = expandObstacles(x_map, y_map);

                for (int i=0; i<obstacleIdx.size(); i++)
                {
                    if (input_map_.data[obstacleIdx[i]] != 100)
                    {
                        input_map_.data[obstacleIdx[i]] = 100;
                        new_obstacles_.push_back(obstacleIdx[i]);
                    }
                }
                theta += angle_increment
            }

            clear_obstacles_count_++;
            if (clear_obstacles_count_ > 50)
            {
                for (int i=0; i<new_obstacles_.size(); i++)
                {
                    input_map_.data[new_obstacles_[i]] = 0;
                }

                new_obstacles_.clear();
                clear_obstacles_count_ = 0;
            }

            map_pub_.publish(input_map_);
            ROS_INFO("Map updated");
        }

        std::vector<int> expandObstacles(const double x_map, const double y_map)
        {
            std::vector<int> obstacleIdx;

            const auto x_map_idx = static_cast<int>((x_map - input_map_.info.origin.position.x)/input_map_.info.resolution);
            const auto y_map_idx = static_cast<int>((y_map - input_map_.info.origin.position.y)/input_map_.info.resolution);

            for (int i=x_map_idx-inflation_r_; i<x_map_idx+inflation_r; i++)
            {
                for (int j=y_map_idx-inflation_r_; j<y_map_idx+inflation_r_; j++)
                {
                    obstacleIdx.push_back(j*input_map_.info.width + i);
                }
            }

            return obstacleIdx
        }

}
