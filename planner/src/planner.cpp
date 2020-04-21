#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <fstream>
#include <utility>
#include <vector>
#include <string>
#include <chrono>
#include <boost/algorithm/string.hpp>

#include "dynamics/vehicle_state.h"
#include "planner/dubins_path.h"

class Planner
{
    private:
        ros::NodeHandle nh_;

        // Publishers & Subscribers
        ros::Subscriber ego_pose_sub_;
        ros::Subscriber scan_sub_;
        ros::Subscriber opp_odom_sub_;
        ros::Publisher map_pub_;
        ros::Publisher drive_pub_;
        ros::Publisher waypoint_viz_pub_;
        ros::Publisher pub_pred_markers_;

        // Other variables
        const double lookahead_d_;
        const double bubble_radius_;
        std::vector<double> steering_options_;
        const double min_dist_;
        const double gap_size_threshold_;
        const double gap_threshold_;

        // Scan parameters
        const auto start_idx_;
        const auto end_idx_;
        const double angle_increment_;
        bool truncate_;
        const double max_scan_;

        // data parsing
        std::string delimiter_;
        std::string filename_;

        // Map update parameters
        nav_msgs::OccupancyGrid input_map_;
        std::vector<size_t> new_obstacles_;
        int clear_obstacles_count_;

        // tf stuff
        tf2_ros::TransformListener tf2_listener_;
        tf2_ros::Buffer tf_buffer_;
        geometry_msgs::TransformStamped tf_map_to_laser_;
        geometry_msgs::TransformStamped tf_laser_to_map_;
        geometry_msgs::TransformStamped tf_opp_to_ego_;
        // geometry_msgs::TransformStamped tf_map_to_opp_base_;

        // Tracks of waypoints (currently just 1)
        std::vector<Waypoint> global_path_;
        bool follow_global_;

        // State variables
        State ego_car_;
        State opp_car_;


    public:

        Planner(ros::NodeHandle &nh) : tf2_listener_(tf_buffer_)
        {
            nh_ = nh;
            lookahead_d_ = 1.0;

            truncate_ = false;

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
            pub_pred_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/predicted_path",100);

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

            Waypoint(const nav_msgs::Odometry::ConstPtr& odom_msg)
            {
                x = odom_msg->pose.pose.position.x;
                y = odom_msg->pose.pose.position.y;

                tf2::Quaternion q(odom_msg->pose.pose.orientation.x,
                                  odom_msg->pose.pose.orientation.y,
                                  odom_msg->pose.pose.orientation.z,
                                  odom_msg->pose.pose.orientation.w);
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

            if (!truncate_)
            {
                const truncate_size = static_cast<size_t>((180/(scan_msg->angle_max - scan_msg->angle_min))*scan_msg->ranges.size());

                start_idx_ = (scan_msg->ranges.size()/2) - (truncate_size/2);
                end_idx_ = (scan_msg->ranges.size()/2) + (truncate_size/2);

                truncate_ = true;

                angle_increment_ = scan_msg->angle_increment
            }

            ROS_DEBUG("Got truncated start and end indices!");

            std::vector<double> filtered_ranges;
            for (size_t i=start_idx_; i<end_idx_; i++)
            {
                if (std::isnan(scan_msg->ranges[i]))
                {
                    filtered_ranges.push_back(0.0);
                }
                else if (scan_msg->ranges[i] > max_scan_ || std::isinf(scan_msg->ranges[i]))
                {
                    filtered_ranges.push_back(max_scan_);
                }
                else
                {
                    filtered_ranges.push_back(scan_msg->ranges[i]);
                }
            }

            ROS_DEBUG("Filtered scan ranges of nans and infs");

            const size_t closest_idx = closestPoint(filtered_ranges);

            const auto closest_dist = filtered_ranges[closest_idx];

            eliminate_bubble(&filtered_ranges, closest_idx, closest_dist);

            ROS_DEBUG("Eliminated safety bubble!");

            auto best_idx = findbestGapIdx(filtered_ranges);

            for (int i=0; i<best_idx.size(); i++)
            {
                steering_options_.push_back(angle_increment_*(best_idx[i]-(filtered_ranges.size()/2)))
            }
        }

        void egoOdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
        {
            ego_car_.x = odom_msg->pose.pose.position.x;
            ego_car_.y = odom_msg->pose.pose.position.y;

            tf2::Quaternion q(odom_msg->pose.pose.orientation.x,
                              odom_msg->pose.pose.orientation.y,
                              odom_msg->pose.pose.orientation.z,
                              odom_msg->pose.pose.orientation.w);
            tf2::Matrix3x3 mat(q);

            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);

            ego_car_.theta = yaw;
            ego_car_.velocity = odom_msg->twist.twist.linear.x;
            ego_car_.angular_velocity = odom_msg->twist.twist.angular.z;

            try
            {
                tf_opp_to_ego_ = tf_buffer_.lookupTransform("ego_racecar/base_link", "opp_racecar/base_link", ros::Time(0));
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(0.1).sleep();
            }

            const auto translation = tf_opp_to_ego_.transform.translation;
            const auto dist = sqrt(pow(translation.x, 2) + pow(translation.y, 2));

            //TODO Find all waypoint options from all trajectory options
            const auto waypoint_options = findWaypoints();
            const auto best_waypoint = checkFeasibility(waypoint_options);
        }

        //This method predicts the path of the opponent car and updates the path as obstacles in the map
        void oppOdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
        {   
            //record the current opponent pose as waypoint
            const auto current_pose = Waypoint(odom_msg);

            //note the time when the pose is recorded
            // std::chrono::steady_clock::time_point previous_time = std::chrono::steady_clock::now()

            opp_car_.theta = yaw;
            opp_car_.velocity = odom_msg->twist.twist.linear.x;
            opp_car_.angular_velocity = odom_msg->twist.twist.angular.z;

            //linear model weight factor
            double alpha = 0.8; //tunabel parameter

            //container to store the predicted poses of opponent car
            std::vector<double> x_opp_car_poses;
            std::vector<double> y_opp_car_poses;

            //store the current obtained pose as the first pose
            x_opp_car_poses.push_back(opp_car_.x);
            y_opp_car_poses.push_back(opp_car_.y);

            double last_x_opp_car = current_pose.x;
            double last_y_opp_car = current_pose.y;

            double next_x_opp = opp_car_.x;
            double next_y_opp = opp_car_.y;
            double initial_heading = current_pose.heading; //the current yaw of the odom.

            double opp_vel = current_pose.speed; //make sure the waypoint struct is not storing the default 0.1 always
            // double opp_vel = 2.5;
            // ROS_INFO("%f",opp_vel);

            const double pp_steering_angle = PPAngle(current_pose);

            for(int i=0; i<=10; i++)
            {

                double final_steering_angle = (1 - pow(alpha, i))*(pp_steering_angle)*3; //*3 is tunable parameter
                double net_heading = initial_heading + final_steering_angle;
                // ROS_INFO("pp_steering_angle: %f iteration i = %d", steering_angle, i);

                next_x_opp = next_x_opp + opp_vel*(cos(net_heading))*0.1; //(no of iterations * 0.1 = 1 second)
                next_y_opp = next_y_opp + opp_vel*(sin(net_heading))*0.1;

                auto current_pose = Waypoint();
                current_pose.x = next_x_opp;
                current_pose.y = next_y_opp;
                current_pose.heading = net_heading;


                x_opp_car_poses.push_back(next_x_opp);
                y_opp_car_poses.push_back(next_y_opp);
            }

            PublishPredictionMarkers(x_opp_car_poses, y_opp_car_poses);

            for (int i=0; i<x_opp_car_poses.size(), i++)
            {
                std::vector<int> obstacleIdx = expandObstacles(x_opp_car_poses[i], y_opp_car_poses[i]);

                for (int i=0; i<obstacleIdx.size(); i++)
                {
                    if (input_map_.data[obstacleIdx[i]] != 100)
                    {
                        input_map_.data[obstacleIdx[i]] = 100;
                        new_obstacles_.push_back(obstacleIdx[i]);
                    }
                }
            }

        }

        void PublishPredictionMarkers(const std::vector<double>& x_poses_ego_vehicle, const std::vector<double>& y_poses_ego_vehicle)
        {
            visualization_msgs::MarkerArray viz_msg;

            ROS_INFO("Publishing Prediction Markers");
            viz_msg.markers.clear();

            for(size_t i=0; i<x_poses_ego_vehicle.size(); i++)
            {
                visualization_msgs::Marker point;
                point.header.frame_id = "map";
                point.header.stamp = ros::Time::now();
                point.ns = "point_123";
                point.action =visualization_msgs::Marker::ADD;
                point.pose.orientation.w = 1.0;
                point.id = i;
                point.type = visualization_msgs::Marker::SPHERE;
                point.scale.x = 0.2;
                point.scale.y = 0.2;
                point.scale.z = 0.2;
                point.color.r = 1.0f;
                point.color.g = 0.0f;
                point.color.a = 1.0;
                point.pose.position.x = x_poses_ego_vehicle[i];
                point.pose.position.y = y_poses_ego_vehicle[i];
                point.lifetime = ros::Duration(10);
                viz_msg.markers.push_back(std::move(point));
            }
            pub_pred_markers_.publish(viz_msg);
            ROS_INFO("Published Prediction Markers");
        }

        //This method gives the steering angle based on pure-pursuit for opponent car
        double PPAngle(const Waypoint& current_pose)
        {   
            // Transform waypoints to baselink frame
            const auto transformed_waypoints = transform(global_path_, current_pose);

            // Find best waypoint to track
            const auto best_waypoint = find_best_waypoint(transformed_waypoints, lookahead_d_);

            // Transform the waypoint to base_link frame
            geometry_msgs::TransformStamped map_to_opp_base_link;
            map_to_opp_base_link = tf_buffer_.lookupTransform("opp_racecar/base_link", "map", ros::Time(0));

            geometry_msgs::Pose goal_waypoint;
            goal_waypoint.position.x = global_path_[best_waypoint].x;
            goal_waypoint.position.y = global_path_[best_waypoint].y;
            goal_waypoint.position.z = 0;
            goal_waypoint.orientation.x = 0;
            goal_waypoint.orientation.y = 0;
            goal_waypoint.orientation.z = 0;
            goal_waypoint.orientation.w = 1;

            tf2::doTransform(goal_waypoint, goal_waypoint, map_to_base_link);

            // add_waypoint_viz(goal_waypoint, "base_link", 0.0, 1.0, 0.0, 1.0, 0.2, 0.2, 0.2);

            double pp_steering_angle = 0.6*2*goal_waypoint.position.y/(lookahead_d_ * lookahead_d_);

            return pp_steering_angle
        }

        size_t find_best_waypoint(const std::vector<Waypoint>& waypoint_data_, double lookahead_d_)
        {   
            size_t last_waypt_idx;
            double closest_dist = std::numeric_limits<double>::max();
            const size_t waypoint_size = waypoint_data_.size();

            for (size_t i=0; i < waypoint_size; i++)
            {
                if (waypoint_data_[i].x < 0) continue;
                double d = sqrt(waypoint_data_[i].x*waypoint_data_[i].x + waypoint_data_[i].y*waypoint_data_[i].y);
                double diff = std::abs(d - lookahead_d_);

                if (diff < closest_dist)
                {
                    closest_dist = diff;
                    last_waypt_idx = i;
                }
            }

            return last_waypt_idx;
        }

        std::vector<Waypoint> transform(const std::vector<Waypoint>& waypoints, const Waypoint& current_pose)
        {
            geometry_msgs::TransformStamped tf_map_to_opp_base_link_;
            try
            {
                tf_map_to_opp_base_link_ = tf_buffer_.lookupTransform("opp_racecar/base_link", "map", ros::Time(0));
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(0.1).sleep();
            }

            std::vector<Waypoint> transformed_waypoints;

            for (int i=0; i<waypoints.size(); i++)
            {
                geometry_msgs::Pose trans_waypoint;
                trans_waypoint.position.x = waypoints[i].x;
                trans_waypoint.position.y = waypoints[i].y;
                trans_waypoint.position.z = 0;
                trans_waypoint.orientation.x = 0;
                trans_waypoint.orientation.y = 0;
                trans_waypoint.orientation.z = 0;
                trans_waypoint.orientation.w = 1;

                tf2::doTransform(trans_waypoint, trans_waypoint, map_to_base_link);

                transformed_waypoints.push_back(Waypoint(trans_waypoint));
            }

            return transformed_waypoints;
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

            ROS_INFO("Reading waypoint data...");
            global_path_ = get_data();

            ROS_INFO("Stored the different tracks as a vector of vector of Waypoints");

            for (int i=x_map_idx-inflation_r_; i<x_map_idx+inflation_r; i++)
            {
                for (int j=y_map_idx-inflation_r_; j<y_map_idx+inflation_r_; j++)
                {
                    obstacleIdx.push_back(j*input_map_.info.width + i);
                }
            }

            return obstacleIdx
        }

        size_t closestPoint(const std::vector<double>& scan_ranges)
        {
            const auto min_iter = std::min_element(scan_ranges.begin(), scan_ranges.end());
            return std::distance(scan_ranges.begin(), min_iter);
        }

        void eliminateBubble(std::vector<double>* scan_ranges, const size_t closest_idx, const double closest_dist)
        {
            const auto angle = bubble_radius_/closest_dist;
            const size_t start = round(closest_idx - (angle/angle_increment_));
            const size_t end = round(closest_idx + (angle/angle_increment_));

            if (end >= scan_ranges.size()) end = scan_ranges.size() - 1;

            if (start < 0) start = 0;

            for (size_t i=start; i<end; i++)
            {
                scan_ranges->at(i) = 0.0;
            }
        }


        std::vector<int> findbestGapIdx(const std::vector<double>& scan_ranges)
        {
            size_t current_start = 0;
            size_t current_size = 0;

            size_t current_idx = 0;

            std::vector<int> best_idx;

            while (current_idx < scan_ranges.size())
            {
                current_start = current_idx;
                current_size = 0;

                while (current_idx < scan_ranges.size() && scan_ranges[current_idx] > gap_threshold_)
                {
                    current_size++;
                    current_idx++;
                }

                if (current_size > gap_size_threshold_)
                {
                    size_t best = (current_start + current_start + current_size - 1)/2;
                    best_idx.push_back(static_cast<int>(best));
                }
            }

            //TODO Get gap positions in map frame for MPC constraints

            return best_idx;
        }

        std::vector<Waypoint> findWaypoints()
        {
            try
            {
                tf_map_to_laser_ = tf_buffer_.lookupTransform("laser", "map", ros::Time(0));
            }
            catch (tf::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(0.1).sleep();
            }

            std::vector<Waypoint> waypoints;

            for (int i=0; i<path_num_; i++)
            {
                double waypoint_d = std::numeric_limits<double>::max();
                int waypoint_idx = -1;

                for (int j=0; i<global_path_.size(); i++)
                {
                    geometry_msgs::Pose goal_waypoint;
                    goal_waypoint.position.x = global_path_[i][j].x;
                    goal_waypoint.position.y = global_path_[i][j].x;
                    goal_waypoint.position.z = 0;
                    goal_waypoint.orientation.x = 0;
                    goal_waypoint.orientation.y = 0;
                    goal_waypoint.orientation.z = 0;
                    goal_waypoint.orientation.w = 1;

                    tf2::doTransform(goal_waypoint, goal_waypoint, tf_map_to_laser_);

                    if (goal_waypoint.position.x < 0) continue;

                    double d = sqrt(pow(goal_waypoint.position.x,2) + pow(goal_waypoint.position.y,2));
                    double diff = std::abs(lookahead_d_ - d);

                    if (diff < waypoint_d)
                    {
                        const auto waypoint_map_idx = getMapIdx(global_path_[i][j].x, global_path_[i][j].y);
                        if (input_map_.data[waypoint_map_idx] == 100) continue;
                        waypoint_d = diff;
                        waypoint_idx = j;
                    }
                }

                waypoints.push_back(global_path_[i][waypoint_idx]);
            }

            return waypoints;
        }

        Waypoint checkFeasibility(std::vector<Waypoint> waypoint_options)
        {
            try
            {
                tf_map_to_laser_ = tf_buffer_.lookupTransform("laser", "map", ros::Time(0));
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(0.1).sleep();
            }

            for(int i=0; i<waypoint_options.size(); i++)
            {
                geometry_msgs::Pose goal_waypoint;
                goal_waypoint.position.x = waypoint_options[i].x;
                goal_waypoint.position.y = waypoint_options[i].y;
                goal_waypoint.position.z = 0;
                goal_waypoint.orientation.x = 0;
                goal_waypoint.orientation.y = 0;
                goal_waypoint.orientation.z = 0;
                goal_waypoint.orientation.w = 1;

                tf2::doTransform(goal_waypoint, goal_waypoint, tf_map_to_laser_);
            }
        }

}
