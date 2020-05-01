#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <planner/Inputs.h>
#include <planner/Bezier.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "OsqpEigen/OsqpEigen.h"
#include <unsupported/Eigen/MatrixFunctions>

#include <math.h>
#include <fstream>
#include <utility>
#include <vector>
#include <array>
#include <iostream>
#include <algorithm>
#include <string>
#include <chrono>
#include <boost/algorithm/string.hpp>

#include "planner/waypoint.h"
#include "planner/dubins.h"

const int nx_ = 3;
const int nu_ = 2;

enum rviz_id{
    CENTERLINE,
    CENTERLINE_POINTS,
    THETA_EST,
    PREDICTION,
    BORDERLINES,
    TRAJECTORY_REF,
    TRAJECTORIES,
    MAX_THETA,
    DEBUG
};

class Planner
{
    private:
        ros::NodeHandle nh_;

        // Publishers & Subscribers
        ros::Subscriber ego_odom_sub_;
        ros::Subscriber scan_sub_;
        ros::Subscriber opp_odom_sub_;
        ros::Publisher map_pub_;
        ros::Publisher drive_pub_;
        ros::Publisher waypoint_viz_pub_;
        ros::Publisher pub_pred_markers_;
        ros::Publisher mpc_viz_pub_;
        ros::Publisher traj_viz_pub_;
        ros::Publisher constraints_viz_pub_;
        ros::Publisher mpc_input_pub_;

        // Other variables
        double lookahead_d_;
        double bubble_radius_;
        std::vector<std::vector<double>> best_gaps_;
        std::vector<std::vector<size_t>> best_gap_dist_;
        double gap_size_threshold_;
        double gap_threshold_;
        double inflation_r_;
        double gap_bubble_;
        bool too_close_;
        double alpha_;
        double ref_alpha_;
        double max_acc_;

        // Subscriber Topics
        std::string scan_topic_;
        std::string ego_odom_topic_;
        std::string opp_odom_topic_;
        std::string map_topic_;
        
        // Publisher topics
        std::string drive_topic_;
        std::string waypoint_topic_;
        std::string costmap_topic_;
        std::string traj_topic_;
        std::string mpc_topic_;
        std::string constraints_topic_;
        std::string input_topic_;

        // TF frames
        std::string map_frame_;
        std::string ego_base_frame_;
        std::string opp_base_frame_;
        std::string ego_laser_frame_;
        std::string left_rear_frame_;
        std::string right_rear_frame_;

        // Scan parameters
        size_t start_idx_;
        size_t end_idx_;
        double angle_increment_;
        bool truncate_;
        double max_scan_;

        // data parsing
        std::string delimiter_;
        std::string folder_path_;
        std::string optimal_waypoint_file_;
        int path_num_;

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
        geometry_msgs::TransformStamped tf_left_to_map_;
        geometry_msgs::TransformStamped tf_right_to_map_;

        // Tracks of waypoints
        std::vector<Waypoint> global_path_;
        std::vector<std::vector<Waypoint>> trajectory_options_;
        bool follow_global_;
        int unique_id_;
        int num_pts_collision_;
        Waypoint prev_best_;

        // State variables
        geometry_msgs::Pose2D ego_car_;
        geometry_msgs::Pose2D opp_car_;
        double current_ego_vel_;

        // MPC variables
        std::vector<std::vector<double>> mpc_constraints_;
        double step_size_;
        double max_angular_vel_;
        std::vector<double> scan_hits_;
        double start_idx_theta_;
        double end_idx_theta_;
        
        int N_;
        // const int nx_ = 3;
        // const int nu_ = 2;
        double max_speed_, max_steer_, C_l_, q_x_, q_y_, q_yaw_, r_v_, r_steer_, Ts_;
        double xp1_, yp1_;
        double xp2_, yp2_;
        double xc1_, yc1_;
        double xc2_, yc2_;

        Eigen::Matrix<double, nx_, nx_> Q_;
        Eigen::Matrix<double, nu_, nu_> R_;


    public:

        Planner(ros::NodeHandle &nh) : tf2_listener_(tf_buffer_)
        {
            nh_ = nh;

            // Tuning parameters
            nh_.getParam("/lookahead_distance", lookahead_d_);
            nh_.getParam("/bubble_radius", bubble_radius_);
            nh_.getParam("/gap_threshold", gap_threshold_);
            nh_.getParam("/inflation_radius", inflation_r_);
            nh_.getParam("/gap_bubble", gap_bubble_);
            nh_.getParam("/max_scan", max_scan_);
            nh_.getParam("/delimiter", delimiter_);
            nh_.getParam("/folder_path", folder_path_);
            nh_.getParam("/collision_points", num_pts_collision_);
            nh_.getParam("/gap_size_threshold", gap_size_threshold_);
            nh_.getParam("/alpha", alpha_);
            nh_.getParam("/ref_alpha", ref_alpha_);
            nh_.getParam("/max_acc", max_acc_);

            // Subscriber Topics
            nh_.getParam("/scan_topic", scan_topic_);
            nh_.getParam("/ego_odom", ego_odom_topic_);
            nh_.getParam("/opp_odom", opp_odom_topic_);
            nh_.getParam("/map", map_topic_);

            // Publisher topics
            nh_.getParam("/drive_topic", drive_topic_);
            nh_.getParam("/waypoints", waypoint_topic_);
            nh_.getParam("/costmap", costmap_topic_);
            nh_.getParam("/trajectory", traj_topic_);
            nh_.getParam("/mpc", mpc_topic_);
            nh_.getParam("/constraints", constraints_topic_);
            nh_.getParam("/inputs", input_topic_);

            // TF Frames
            nh_.getParam("/map_frame", map_frame_);
            nh_.getParam("/ego_car", ego_base_frame_);
            nh_.getParam("/opp_car", opp_base_frame_);
            nh_.getParam("/ego_laser", ego_laser_frame_);
            nh_.getParam("/left_rear_wheel", left_rear_frame_);
            nh_.getParam("/right_rear_wheel", right_rear_frame_);

            // MPC variables
            nh_.getParam("N", N_);
            nh_.getParam("Ts", Ts_);
            nh_.getParam("max_speed", max_speed_);
            nh_.getParam("max_steer", max_steer_);
            nh_.getParam("C_l", C_l_);
            nh_.getParam("q_x", q_x_);
            nh_.getParam("q_y", q_y_);
            nh_.getParam("q_yaw", q_yaw_);
            nh_.getParam("r_v", r_v_);
            nh_.getParam("r_steer", r_steer_);
            nh_.getParam("step_size", step_size_);

            truncate_ = false;
            
            optimal_waypoint_file_ = folder_path_ + "/waypoints_data/wp_with_yaw.csv";
            path_num_ = 4;

            clear_obstacles_count_ = 0;

            unique_id_ = 0;

            max_angular_vel_ = 3.2;

            Q_.setZero();
            R_.setZero();
            Q_.diagonal() << q_x_, q_y_, q_yaw_;
            R_.diagonal() << r_v_, r_steer_;

            ROS_INFO("Initialized constant!");
        }

        ~Planner(){}

        void initialize()
        {
            ROS_INFO("Initializing publishers and subscribers...");

            input_map_ = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic_, ros::Duration(2)));

            if (input_map_.data.empty())
            {
                ROS_ERROR("Empty map received :(");
            }

            ROS_INFO("Received first map!");

            drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic_, 1);
            waypoint_viz_pub_ = nh_.advertise<visualization_msgs::Marker>(waypoint_topic_, 1);
            map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(costmap_topic_, 1);
            pub_pred_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/predicted_path", 100);
            mpc_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(mpc_topic_,10);
            traj_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(traj_topic_, 10);
            constraints_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(constraints_topic_, 10);
            mpc_input_pub_ = nh_.advertise<planner::Inputs>(input_topic_,1);

            scan_sub_ = nh_.subscribe(scan_topic_, 1, &Planner::scanCallback, this);
            opp_odom_sub_ = nh_.subscribe(opp_odom_topic_, 1, &Planner::oppOdomCallback, this);
            ego_odom_sub_ = nh_.subscribe(ego_odom_topic_, 1, &Planner::egoOdomCallback, this);

            ROS_INFO("Initialized publishers and subscribers!");

            try
            {
                tf_laser_to_map_ = tf_buffer_.lookupTransform(map_frame_, ego_laser_frame_, ros::Time(0));
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }

            ROS_INFO("Reading waypoint data...");
            global_path_ = getOptimalTrack();

            std::string file1 = folder_path_ + "/waypoints_data/yaw_wp_inner0.5.csv";
            std::string file2 = folder_path_ + "/waypoints_data/yaw_wp_inner0.75.csv";
            std::string file3 = folder_path_ + "/waypoints_data/yaw_wp_outer0.5.csv";
            std::string file4 = folder_path_ + "/waypoints_data/yaw_wp_outer1.0.csv";

            trajectory_options_.push_back(getTrack(file1));
            trajectory_options_.push_back(getTrack(file2));
            trajectory_options_.push_back(getTrack(file3));
            trajectory_options_.push_back(getTrack(file4));
        }

        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
        {
            updateStaticMap(scan_msg);

            if (!truncate_)
            {
                const size_t truncate_size = static_cast<size_t>((3.14/(scan_msg->angle_max - scan_msg->angle_min))*scan_msg->ranges.size());

                start_idx_ = (scan_msg->ranges.size()/2) - (truncate_size/2);
                end_idx_ = (scan_msg->ranges.size()/2) + (truncate_size/2);

                truncate_ = true;

                angle_increment_ = scan_msg->angle_increment;

                start_idx_theta_ = scan_msg->angle_min + start_idx_*angle_increment_;
                end_idx_theta_ = scan_msg->angle_min + end_idx_*angle_increment_;
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

            const auto closest_pt_it = std::min_element(filtered_ranges.begin(), filtered_ranges.end());
            auto closest_idx = std::distance(filtered_ranges.begin(), closest_pt_it);

            auto closest_dist = filtered_ranges[closest_idx];

            // eliminateBubble(&filtered_ranges, closest_idx, closest_dist);

            // ROS_DEBUG("Eliminated safety bubble!");

            scan_hits_ = filtered_ranges;

            // findbestGap(filtered_ranges, start_idx_theta_, end_idx_theta_);
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
            current_ego_vel_ = odom_msg->twist.twist.linear.x;

            std::vector<Waypoint> waypoint_options;

            waypoint_options = findWaypoints(lookahead_d_);
            std::vector<Waypoint> optimal;
            optimal = findOptimalWaypoint(lookahead_d_);
            waypoint_options.push_back(optimal[0]);
            
            auto best_waypoint = checkFeasibility(waypoint_options);

            add_waypoint_viz(best_waypoint, map_frame_,0.0,1.0,0.0,1.0,0.2,0.2,0.2);

            findGapConstraints(best_waypoint);

            //REFERENCE TRAJECTORY USING PURE PURSUIT
//            const double ref_steering_angle = ReferencePPAngle(best_waypoint);
//
//            double next_x = ego_car_.x;
//            double next_y = ego_car_.y;
//            double initial_yaw = ego_car_.theta;
//
            std::vector<Eigen::VectorXd> ref_trajectory;
            std::vector<Eigen::VectorXd> ref_input;
//
//            double ref_speed = current_ego_vel_;
//
//            Eigen::VectorXd traj(nx_);
//            Eigen::VectorXd input(nu_);
//
//            traj(0) = next_x;
//            traj(1) = next_y;
//            traj(2) = initial_yaw;
//
//            // if (ref_speed < 0.01)
//            // {
//            //     input(0) = 0.0;
//            // }
//            // else
//            // {
//            input(0) = ref_speed; //current_ego_vel_;
//            // }
//            input(1) = 0.0;
//
//            ref_trajectory.push_back(traj);
//            ref_input.push_back(input);
//
//            // ROS_INFO("Initial Speed: %f", ref_speed);
//
//            for (int i=0; i<N_; i++)
//            {
//                double final_steering_angle = (1 - pow(ref_alpha_, i))*(ref_steering_angle)*2.0;
//                double net_yaw = initial_yaw + final_steering_angle;
//
//                ref_speed += Ts_*max_acc_;
//                ref_speed = std::min(4.5, ref_speed);
//
//                // ROS_INFO("Speed at i: %d is : %f", i, ref_speed);
//                // ROS_INFO("TS_: %f, max_acc: %f", Ts_, max_acc_);
//
//                next_x = next_x + ref_speed*(cos(net_yaw))*Ts_;
//                next_y = next_y + ref_speed*(sin(net_yaw))*Ts_;
//
//                Eigen::VectorXd traj(nx_);
//                Eigen::VectorXd input(nu_);
//
//                traj(0) = next_x;
//                traj(1) = next_y;
//                traj(2) = net_yaw;
//
//                // if (current_ego_vel_ < 0.01)
//                // {
//                //     input(0) = 4.5;
//                // }
//                // else
//                // {
//                input(0) = ref_speed; //current_ego_vel_;
//                // }
//                input(1) = final_steering_angle;
//
//                ref_trajectory.push_back(traj);
//                ref_input.push_back(input);
//
//            }

            Curve* curve = new Bezier();
            curve->set_steps(2*N_);

            curve->add_way_point(Vector(ego_car_.x, ego_car_.y, ego_car_.theta));

            Waypoint final_waypoint;
            
            for (int i=0; i<3; i++)
            {
                std::vector<Waypoint> waypoints;
                double lookahead = i + 1.0;
                waypoints = findWaypoints(lookahead);
                std::vector<Waypoint> opt = findOptimalWaypoint(lookahead);
                waypoints.push_back(opt[0]);

                auto waypoint_b = checkFeasibility(waypoints);

                if (i==2)
                {
                    final_waypoint = waypoint_b;
                }

                curve->add_way_point(Vector(waypoint_b.x, waypoint_b.y, 0));
            }

            double yaw_diff = final_waypoint.heading - ego_car_.theta;

            auto ref_steering = ReferencePPAngle(final_waypoint);
            double ref_speed = current_ego_vel_;

            for (int i=0; i<2*N_; i++)
            {
                Eigen::VectorXd traj(nx_);

                traj(0) = curve->node(i).x;
                traj(1) = curve->node(i).y;

                traj(2) = (yaw_diff * i)/(2*N_);

                // std::cout << "Yaw angle at index " << i << ": " << traj(2) << "\n";

                ref_trajectory.push_back(traj);

                Eigen::VectorXd input(nu_);

                ref_speed += Ts_*max_acc_;
                ref_speed = std::min(4.5, ref_speed);

                input(0) = ref_speed;
                input(1) = traj(2)-ego_car_.theta;

                ref_input.push_back(input);
            }

            visualizeTrajectory(ref_trajectory);

            initMPC(ref_trajectory, ref_input);

//            publishPPSpeed(best_waypoint);
        }

        //This method predicts the path of the opponent car and updates the path as obstacles in the map
        void oppOdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
        {   
            //record the current opponent pose as waypoint
            const auto current_pose = Waypoint(odom_msg);

            //note the time when the pose is recorded
            opp_car_.x = odom_msg->pose.pose.position.x;
            opp_car_.y = odom_msg->pose.pose.position.y;
            
            tf2::Quaternion q(odom_msg->pose.pose.orientation.x,
                              odom_msg->pose.pose.orientation.y,
                              odom_msg->pose.pose.orientation.z,
                              odom_msg->pose.pose.orientation.w);
            tf2::Matrix3x3 mat(q);

            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);
            
            opp_car_.theta = yaw;

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

            const double pp_steering_angle = PPAngle(current_pose);

            for(int i=0; i<=5; i++)
            {

                double final_steering_angle = (1 - pow(alpha_, i))*(pp_steering_angle)*3; //*3 is tunable parameter
                double net_heading = initial_heading + final_steering_angle;

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

            for (int i=0; i<x_opp_car_poses.size(); i++)
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

            viz_msg.markers.clear();

            for(size_t i=0; i<x_poses_ego_vehicle.size(); i++)
            {
                visualization_msgs::Marker point;
                point.header.frame_id = map_frame_;
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
        }

        double ReferencePPAngle(const Waypoint& waypoint)
        {
            geometry_msgs::Pose goal;
            goal.position.x = waypoint.x;
            goal.position.y = waypoint.y;
            goal.position.z = 0;
            goal.orientation.x = 0;
            goal.orientation.y = 0;
            goal.orientation.z = 0;
            goal.orientation.w = 1;

            try
            {
                tf_map_to_laser_ = tf_buffer_.lookupTransform(ego_base_frame_, map_frame_, ros::Time(0));
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(0.1).sleep();
            }

            tf2::doTransform(goal, goal, tf_map_to_laser_);

            double steering_angle = (0.6*2*goal.position.y)/pow(lookahead_d_, 2);

            return steering_angle;
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
            map_to_opp_base_link = tf_buffer_.lookupTransform(opp_base_frame_, map_frame_, ros::Time(0));

            geometry_msgs::Pose goal_waypoint;
            goal_waypoint.position.x = global_path_[best_waypoint].x;
            goal_waypoint.position.y = global_path_[best_waypoint].y;
            goal_waypoint.position.z = 0;
            goal_waypoint.orientation.x = 0;
            goal_waypoint.orientation.y = 0;
            goal_waypoint.orientation.z = 0;
            goal_waypoint.orientation.w = 1;

            tf2::doTransform(goal_waypoint, goal_waypoint, map_to_opp_base_link);

            double pp_steering_angle = 0.6*2*goal_waypoint.position.y/(lookahead_d_ * lookahead_d_);

            return pp_steering_angle;
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
                tf_map_to_opp_base_link_ = tf_buffer_.lookupTransform(opp_base_frame_, map_frame_, ros::Time(0));
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

                tf2::doTransform(trans_waypoint, trans_waypoint, tf_map_to_opp_base_link_);

                transformed_waypoints.push_back(Waypoint(trans_waypoint));
            }

            return transformed_waypoints;
        }

        void updateStaticMap(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
        {
            try
            {
                tf_laser_to_map_ = tf_buffer_.lookupTransform(map_frame_, ego_laser_frame_, ros::Time(0));
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

            for (int i=start; i<end; i++)

            {
                theta += angle_increment;
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

            }

            clear_obstacles_count_++;
            if (clear_obstacles_count_ > 5)
            {
                for (int i=0; i<new_obstacles_.size(); i++)
                {
                    input_map_.data[new_obstacles_[i]] = 0;
                }

                new_obstacles_.clear();
                clear_obstacles_count_ = 0;
            }

            map_pub_.publish(input_map_);
            ROS_DEBUG("Map updated");
        }

        std::vector<int> expandObstacles(const double x_map, const double y_map)
        {
            std::vector<int> obstacleIdx;

            const auto x_map_idx = static_cast<int>((x_map - input_map_.info.origin.position.x)/input_map_.info.resolution);
            const auto y_map_idx = static_cast<int>((y_map - input_map_.info.origin.position.y)/input_map_.info.resolution);

            for (int i=x_map_idx-inflation_r_; i<x_map_idx+inflation_r_; i++)
            {
                for (int j=y_map_idx-inflation_r_; j<y_map_idx+inflation_r_; j++)
                {
                    obstacleIdx.push_back(j*input_map_.info.width + i);
                }
            }

            return obstacleIdx;
        }

        void eliminateBubble(std::vector<double>* scan_ranges, const size_t closest_idx, const double closest_dist)
        {
            const auto angle = bubble_radius_/closest_dist;
            auto start = round(closest_idx - (angle/angle_increment_));
            auto end = round(closest_idx + (angle/angle_increment_));

            if (end >= scan_ranges->size())
            {
                end = scan_ranges->size() - 1;
            }

            if (start < 0)
            {
                start = 0;
            }

            for (size_t i=start; i<end; i++)
            {
                scan_ranges->at(i) = 0.0;
            }
        }

        void findbestGap(const std::vector<double>& scan_ranges, double& start_idx_theta, double& end_idx_theta)
        {
            size_t max_start_idx = 0;
            size_t max_size_ = 0;
            size_t current_start;
            size_t current_size;

            size_t current_idx = 0;

            try
            {
                tf_laser_to_map_ = tf_buffer_.lookupTransform(map_frame_, ego_laser_frame_, ros::Time(0));
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
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
            
            while (current_idx < scan_ranges.size())
            {
                current_start = current_idx;
                current_size = 0;

                while ((current_idx < scan_ranges.size())  && (scan_ranges[current_idx] > gap_threshold_))
                {
                    current_size++;
                    current_idx++;
                }

                if (current_size > max_size_)
                {
                    max_start_idx = current_start;
                    max_size_ = current_size;

                    current_size = 0;
                    std::vector<double> gap;
                    std::vector<size_t> gap_dist;

                    const double start_angle = start_idx_theta + angle_increment_*max_start_idx;
                    const double end_angle = start_idx_theta + angle_increment_*(max_start_idx+max_size_-1);
            
                    gap_dist.push_back(scan_ranges[max_start_idx]);
                    gap_dist.push_back(scan_ranges[max_start_idx+max_size_-1]);

                    gap.push_back(start_angle);
                    gap.push_back(end_angle);

                    best_gaps_.push_back(gap);
                    best_gap_dist_.push_back(gap_dist);
                }
                current_idx++;
            }

            if (current_size > max_size_)
            {
                max_start_idx = current_start;
                max_size_ = current_size;

                std::vector<double> gap;
                std::vector<size_t> gap_dist;
                std::vector<double> start_gap_constraint;
                std::vector<double> end_gap_constraint;
        
                gap_dist.push_back(scan_ranges[max_start_idx]);
                gap_dist.push_back(scan_ranges[max_start_idx+max_size_-1]);
        
                const double start_angle = start_idx_theta + angle_increment_*max_start_idx;
                const double end_angle = start_idx_theta + angle_increment_*(max_start_idx+max_size_-1);

                gap.push_back(start_angle);
                gap.push_back(end_angle);

                best_gaps_.push_back(gap);
                best_gap_dist_.push_back(gap_dist);
            }
            
            std::vector<double> start_gap_constraint;
            std::vector<double> end_gap_constraint;

            const double start_angle = start_idx_theta + angle_increment_*max_start_idx;
            const double end_angle = start_idx_theta + angle_increment_*(max_start_idx+max_size_-1);

            const double start_x_base = scan_ranges[max_start_idx] * cos(start_angle);
            const double start_y_base = scan_ranges[max_start_idx] * sin(start_angle);

            const double end_x_base = scan_ranges[max_start_idx+max_size_-1] * cos(end_angle);
            const double end_y_base = scan_ranges[max_start_idx+max_size_-1] * sin(end_angle);

            xp1_ = start_x_base*cos(yaw) - start_y_base*sin(yaw) + translation.x;
            yp1_ = start_x_base*sin(yaw) + start_y_base*cos(yaw) + translation.y;

            xp2_ = end_x_base*cos(yaw) - end_y_base*sin(yaw) + translation.x;
            yp2_ = end_x_base*sin(yaw) + end_y_base*cos(yaw) + translation.y;
        }

        void findGapConstraints(Waypoint& best_waypoint)
        {
            try
            {
                tf_map_to_laser_ = tf_buffer_.lookupTransform(ego_laser_frame_, map_frame_, ros::Time(0));
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(0.1).sleep();
            }

            geometry_msgs::Pose car_frame;
            car_frame.position.x = best_waypoint.x;
            car_frame.position.y = best_waypoint.y;
            car_frame.position.z = 0;
            car_frame.orientation.x = 0;
            car_frame.orientation.y = 0;
            car_frame.orientation.z = 0;
            car_frame.orientation.w = 1;

            tf2::doTransform(car_frame, car_frame, tf_map_to_laser_);

            double waypoint_angle = atan2(car_frame.position.y, car_frame.position.x);

            // if (waypoint_angle < 0) continue; //TODO What to do?

            const int waypoint_idx = round((waypoint_angle+1.57)/angle_increment_);

            int right_idx, left_idx;

            right_idx = 0;
            left_idx = scan_hits_.size()-1;

            for (int i=waypoint_idx; i>0; i--)
            {
                if (scan_hits_[i] <= gap_threshold_)
                {
                    right_idx = i;
                    break;
                }
                
            }

            for (int i=waypoint_idx; i<scan_hits_.size(); i++)
            {
                if (scan_hits_[i] <= gap_threshold_)
                {
                    left_idx = i;
                    break;
                }
            }

            if (left_idx == right_idx)
            {
                ROS_INFO("Same indices!");
            }

            right_idx = right_idx + 15;
            left_idx = left_idx - 15;

            double right_angle = (right_idx) * angle_increment_ - 1.57;
            double left_angle = (left_idx) * angle_increment_ - 1.57;

            // std::cout << "Angle difference: " << abs(right_angle-left_angle) << "\n";

            try
            {
                tf_laser_to_map_ = tf_buffer_.lookupTransform(map_frame_, ego_laser_frame_, ros::Time(0));
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
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

            double start_x_base, end_x_base;
            double start_y_base, end_y_base;
            
            start_x_base = scan_hits_[right_idx] * cos(right_angle);
            start_y_base = scan_hits_[right_idx] * sin(right_angle);

            end_x_base = scan_hits_[left_idx] * cos(left_angle);
            end_y_base = scan_hits_[left_idx] * sin(left_angle);
 
            xp1_ = start_x_base*cos(yaw) - start_y_base*sin(yaw) + translation.x;
            yp1_ = start_x_base*sin(yaw) + start_y_base*cos(yaw) + translation.y;

            xp2_ = end_x_base*cos(yaw) - end_y_base*sin(yaw) + translation.x;
            yp2_ = end_x_base*sin(yaw) + end_y_base*cos(yaw) + translation.y;

            // std::cout << (yp2_-yp1_) << "\n";
            
            try
            {
                tf_left_to_map_ = tf_buffer_.lookupTransform(map_frame_, left_rear_frame_, ros::Time(0));
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(0.1).sleep();
            }

            const auto left_translation = tf_left_to_map_.transform.translation;

            xc2_ = left_translation.x;
            yc2_ = left_translation.y;

            try
            {
                tf_right_to_map_ = tf_buffer_.lookupTransform(map_frame_, right_rear_frame_, ros::Time(0));
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(0.1).sleep();
            }

            const auto right_translation = tf_right_to_map_.transform.translation;

            xc1_ = right_translation.x;
            yc1_ = right_translation.y;

            visualizeConstraints();
        }

        std::vector<Waypoint> findOptimalWaypoint(double& lookahead)
        {
            try
            {
                tf_map_to_laser_ = tf_buffer_.lookupTransform(ego_laser_frame_, map_frame_, ros::Time(0));
            }
            catch (tf::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(0.1).sleep();
            }

            double waypoint_d = std::numeric_limits<double>::max();
            int waypoint_idx = -1;

            std::vector<Waypoint> optimal_waypoint;

            for (int i=0; i<global_path_.size(); i++)
            {
                    
                if (checkCollision(global_path_[i])) continue;
                
                geometry_msgs::Pose goal_waypoint;
                goal_waypoint.position.x = global_path_[i].x;
                goal_waypoint.position.y = global_path_[i].y;
                goal_waypoint.position.z = 0;
                goal_waypoint.orientation.x = 0;
                goal_waypoint.orientation.y = 0;
                goal_waypoint.orientation.z = 0;
                goal_waypoint.orientation.w = 1;

                tf2::doTransform(goal_waypoint, goal_waypoint, tf_map_to_laser_);

                if (goal_waypoint.position.x < 0) continue;

                // if (abs(goal_waypoint.position.y > 1.5)) continue;

                double d = sqrt(pow(goal_waypoint.position.x, 2) + pow(goal_waypoint.position.y, 2));
                double diff = std::abs(lookahead - d);

                if (diff < waypoint_d)
                {
                    const auto waypoint_map_idx = getMapIdx(global_path_[i].x, global_path_[i].y);
                    if (input_map_.data[waypoint_map_idx] == 100) continue;
                    waypoint_d = diff;
                    waypoint_idx = i;
                }
            }

            optimal_waypoint.push_back(global_path_[waypoint_idx]);

            return optimal_waypoint;
        }

        std::vector<Waypoint> findWaypoints(double& lookahead)
        {
            try
            {
                tf_map_to_laser_ = tf_buffer_.lookupTransform(ego_laser_frame_, map_frame_, ros::Time(0));
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

                for (int j=0; j<trajectory_options_[i].size(); j++)
                {

                    if (checkCollision(trajectory_options_[i][j])) continue;

                    geometry_msgs::Pose goal_waypoint;
                    goal_waypoint.position.x = trajectory_options_[i][j].x;
                    goal_waypoint.position.y = trajectory_options_[i][j].y;
                    goal_waypoint.position.z = 0;
                    goal_waypoint.orientation.x = 0;
                    goal_waypoint.orientation.y = 0;
                    goal_waypoint.orientation.z = 0;
                    goal_waypoint.orientation.w = 1;

                    tf2::doTransform(goal_waypoint, goal_waypoint, tf_map_to_laser_);

                    if (goal_waypoint.position.x < 0) continue;

                    double d = sqrt(pow(goal_waypoint.position.x,2) + pow(goal_waypoint.position.y,2));
                    double diff = std::abs(lookahead - d);

                    if (diff < waypoint_d)
                    {
                        const auto waypoint_map_idx = getMapIdx(trajectory_options_[i][j].x, trajectory_options_[i][j].y);
                        if (input_map_.data[waypoint_map_idx] == 100) continue;
                        waypoint_d = diff;
                        waypoint_idx = j;
                    }
                }

                waypoints.push_back(trajectory_options_[i][waypoint_idx]);
            }

            return waypoints;
        }

        Waypoint checkFeasibility(std::vector<Waypoint> waypoint_options)
        {	
        	bool change = false;
            try
            {
                tf_map_to_laser_ = tf_buffer_.lookupTransform(ego_laser_frame_, map_frame_, ros::Time(0));
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(0.1).sleep();
            }

            Waypoint best_waypoint;

            geometry_msgs::Pose opt_waypoint;
            opt_waypoint.position.x = waypoint_options[path_num_].x;
            opt_waypoint.position.y = waypoint_options[path_num_].y;
            opt_waypoint.position.z = 0;
            opt_waypoint.orientation.x = 0.0;
            opt_waypoint.orientation.y = 0.0;
            opt_waypoint.orientation.z = 0.0;
            opt_waypoint.orientation.w = 1.0;

            tf2::doTransform(opt_waypoint, opt_waypoint, tf_map_to_laser_);

            double opt_steering = (0.8*2*opt_waypoint.position.y)/(pow(lookahead_d_, 2));

            try
            {
                tf_opp_to_ego_ = tf_buffer_.lookupTransform(ego_base_frame_, opp_base_frame_, ros::Time(0));
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(0.1).sleep();
            }

            const auto translation = tf_opp_to_ego_.transform.translation;
            const auto dist = sqrt(pow(translation.x, 2) + pow(translation.y, 2));

            if (dist <= 1.0 && translation.x > 0.25)
            {
                too_close_ = true;
            }
            else
            {
                too_close_ = false;
            }

            if (dist > gap_bubble_)
            {
                if (abs(opt_steering)<0.413)
                {
                    best_waypoint = waypoint_options[path_num_];
                    best_gaps_.clear();
                    // ROS_INFO("Choosing the optimal waypoint");
                    return best_waypoint;
                }
            }

            // ROS_INFO("Pass the if, dist: %f, translation: %f", dist, translation.x);

            double best_waypoint_cost = std::numeric_limits<double>::max();

            for(int i=0; i<waypoint_options.size()-1; i++)
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

                double steering_angle = atan2(goal_waypoint.position.x, goal_waypoint.position.y);

                //if (checkGap(steering_angle))
                //{
                    // Send in either states or just waypoints
                    // double cost = dubinsCost(ego_car_, goal_waypoint);

                double cost = waypointCost(goal_waypoint);
                    //TODO Check for obstacles as well

                if (cost < best_waypoint_cost)
                {	
                	change = true;
                    best_waypoint_cost = cost;
                    best_waypoint = waypoint_options[i];
                }
                //}
            }
            if (change != true)
            {
            	ROS_INFO("NO BEST WAYPOINT");
            }
            best_gaps_.clear();
            return best_waypoint;
        }

        double waypointCost(geometry_msgs::Pose waypoint)
        {
            double cost;

            cost = atan2(waypoint.position.x, waypoint.position.y);

            return abs(cost);
        }

        bool checkGap(double steering_angle)
        {
            for (int i=0; i<best_gaps_.size(); i++)
            {
                if (steering_angle<best_gaps_[i][0] || steering_angle>best_gaps_[i][1])
                {
                    return false;
                }
            }
            return true;
        }

        int getMapIdx(const double x_map, const double y_map)
        {
            const auto x_map_idx = static_cast<int>((x_map - input_map_.info.origin.position.x)/input_map_.info.resolution);
            const auto y_map_idx = static_cast<int>((y_map - input_map_.info.origin.position.y)/input_map_.info.resolution);

            return y_map_idx * input_map_.info.width + x_map_idx;
        }

        std::vector<Waypoint> getOptimalTrack()
        {
            std::ifstream file(optimal_waypoint_file_);
            if (!file)
            {
                std::cout << "Invalid path" << "\n";
            }

            std::vector<Waypoint> trajectory;

            std::string line = "";

            while (getline(file, line))
            {
                std::vector<std::string> vec;
                boost::algorithm::split(vec, line, boost::is_any_of(delimiter_));
                Waypoint waypoint{};
                waypoint.x = std::stod(vec[0]);
                waypoint.y = std::stod(vec[1]);
                waypoint.heading = std::stod(vec[2]);
                waypoint.speed = 0.0;

                trajectory.push_back(waypoint);
            }

            return trajectory;
        }

        std::vector<Waypoint> getTrack(std::string filename)
        {
            std::vector<Waypoint> trajectory;

            std::ifstream file(filename);
            if(!file)
            {
                std::cout << "Invalid path" << "\n";
            }

            std::string line = "";

            while (getline(file, line))
            {
                std::vector<std::string> vec;
                boost::algorithm::split(vec, line, boost::is_any_of(delimiter_));
                Waypoint waypoint{};
                waypoint.x = std::stod(vec[0]);
                waypoint.y = std::stod(vec[1]);
                waypoint.heading = std::stod(vec[2]);
                waypoint.speed = 0.0;

                trajectory.push_back(waypoint);
            }

            file.close();

            return trajectory;
        }

        void add_waypoint_viz(const Waypoint& waypoint, const std::string& frame_id,
                double r, double g, double b, double transparency=0.5,
                double scale_x=0.1, double scale_y=0.1, double scale_z=0.1)
        {
            visualization_msgs::Marker waypoint_marker;
            waypoint_marker.header.frame_id = frame_id;
            waypoint_marker.header.stamp = ros::Time();
            waypoint_marker.ns = "planner";
            // waypoint_marker.id = unique_id_;
            waypoint_marker.type = visualization_msgs::Marker::CUBE;
            waypoint_marker.action = visualization_msgs::Marker::ADD;
            waypoint_marker.pose.position.x = waypoint.x;
            waypoint_marker.pose.position.y = waypoint.y;
            waypoint_marker.pose.position.z = 0;
            waypoint_marker.pose.orientation.x = 0.0;
            waypoint_marker.pose.orientation.y = 0.0;
            waypoint_marker.pose.orientation.z = 0.0;
            waypoint_marker.pose.orientation.w = 1.0;
            waypoint_marker.scale.x = scale_x;
            waypoint_marker.scale.y = scale_y;
            waypoint_marker.scale.z = scale_z;
            waypoint_marker.color.a = transparency;
            waypoint_marker.color.r = r;
            waypoint_marker.color.g = g;
            waypoint_marker.color.b = b;

            waypoint_viz_pub_.publish(waypoint_marker);
            unique_id_++;
        }

        bool checkCollision(Waypoint &waypoint)
        {
            bool collision = false;

            double x_increment = (waypoint.x - ego_car_.x)/num_pts_collision_;
            double y_increment = (waypoint.y - ego_car_.y)/num_pts_collision_;

            double current_x = ego_car_.x;
            double current_y = ego_car_.y;

            for (int i=0; i<num_pts_collision_; i++)
            {
                current_x += x_increment;
                current_y += y_increment;

                if (isCollided(current_x, current_y))
                {
                    collision = true;
                    return collision;
                }
            }

            return collision;
        }

        bool isCollided(double x_map, double y_map)
        {
            const auto map_idx = getMapIdx(x_map, y_map);

            return (input_map_.data[map_idx] == 100);
        }

        void publishPPSpeed(const Waypoint &waypoint)
        {
            geometry_msgs::Pose goal;
            goal.position.x = waypoint.x;
            goal.position.y = waypoint.y;
            goal.position.z = 0;
            goal.orientation.x = 0;
            goal.orientation.y = 0;
            goal.orientation.z = 0;
            goal.orientation.w = 1;

            try
            {
                tf_map_to_laser_ = tf_buffer_.lookupTransform(ego_base_frame_, map_frame_, ros::Time(0));
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(0.1).sleep();
            }

            tf2::doTransform(goal, goal, tf_map_to_laser_);

            const double steering_angle = (0.6*2*goal.position.y)/pow(lookahead_d_, 2);
            
            ackermann_msgs::AckermannDriveStamped drive_msg;
            drive_msg.header.stamp = ros::Time::now();
            drive_msg.header.frame_id = ego_base_frame_;
            drive_msg.drive.steering_angle = steering_angle;

            if (steering_angle > 0.41)
            {
                drive_msg.drive.steering_angle = 0.41;
            }
            else if (steering_angle < -0.41)
            {
                drive_msg.drive.steering_angle = -0.41;
            }

            double HIGH_SPEED, MID_SPEED, LOW_SPEED;

            if (!too_close_)
            {
                HIGH_SPEED = 4.5;
                MID_SPEED = 4.5;
                LOW_SPEED = 3.5;
            }
            else
            {
//                ROS_INFO("In low speed mode");
                HIGH_SPEED = 3.0;
                MID_SPEED = 2.0;
                LOW_SPEED = 1.0;
            }

            if (abs(steering_angle) < 0.1745)
            {
                drive_msg.drive.speed = HIGH_SPEED;
            }
            else if (abs(steering_angle) >= 0.1745 && abs(steering_angle) < 0.3491)
            {
                drive_msg.drive.speed = MID_SPEED;
            }
            else
            {
                drive_msg.drive.speed = LOW_SPEED;
            }

            drive_pub_.publish(drive_msg);
        }

        void initMPC(std::vector<Eigen::VectorXd>& ref_trajectory, std::vector<Eigen::VectorXd>& ref_input)
        {
            // define the Hessian and Constraint matrix
            Eigen::SparseMatrix<double> H_matrix((N_+1)*(nx_+nu_), (N_+1)*(nx_+nu_));
            Eigen::SparseMatrix<double> A_c((N_+1)*nx_ + 2*(N_+1) + (N_+1)*nu_, (N_+1)*(nx_+nu_));

            // define the gradient vector
            Eigen::VectorXd g((N_+1)*(nx_+nu_));
            g.setZero();

            // the upper and lower bound constraint vectors
            Eigen::VectorXd lb((N_+1)*nx_ + 2*(N_+1) + (N_+1)*nu_);
            Eigen::VectorXd ub((N_+1)*nx_ + 2*(N_+1) + (N_+1)*nu_);

            // define the matrices (vectors) for state and control references
            // at each time step
            Eigen::Matrix<double, nx_, 1> x_ref;
            Eigen::Matrix<double, nu_, 1> u_ref;

            // define the matrices for discrete dynamics
            Eigen::Matrix<double, nx_, 1> hd;
            Eigen::Matrix<double, nx_, nx_> Ad;
            Eigen::Matrix<double, nx_, nu_> Bd;

            double A11, A12, A21, A22, B11, B22;
            halfSpaceConstraints(A11, A12, A21, A22, B11, B22);

            for (int i=0; i<N_+1; i++)
            {
                x_ref = ref_trajectory[i];
                u_ref = ref_input[i];
                getCarDynamics(Ad, Bd, hd, x_ref, u_ref);

                // fill the H_matrix with state cost Q for the first (N+1)*nx
                // diagonal and input cost R along the next (N+1)*nu diagonal
                if (i > 0)
                {
                    for (int row=0; row<nx_; row++)
                    {
                        H_matrix.insert(i*nx_ + row, i*nx_ + row) = Q_(row, row);
                    }

                    for (int row=0; row<nu_; row++)
                    {
                        H_matrix.insert(((N_+1)*nx_) + (i*nu_+row), ((N_+1)*nx_) + (i*nu_+row)) = R_(row, row);
                    }

                    g.segment<nx_>(i*nx_) << -Q_*x_ref;
                    g.segment<nu_>(((N_+1)*nx_) + i*nu_) << -R_*u_ref;
                }

                // fill the constraint matrix first with the dynamic constraint
                // x_k+1 = Ad*x_k + Bd*u_k + hd
                if (i < N_)
                {
                    for (int row=0; row<nx_; row++)
                    {
                        for (int col=0; col<nx_; col++)
                        {
                            A_c.insert((i+1)*nx_ + row, i*nx_ + col) = Ad(row, col);
                        }
                    }

                    for (int row=0; row<nx_; row++)
                    {
                        for (int col=0; col<nu_; col++)
                        {
                            A_c.insert((i+1)*nx_ + row, (N_+1)*nx_ + i*nu_ + col) = Bd(row, col);
                        }
                    }

                    lb.segment<nx_>((i+1)*nx_) = -hd;
                    ub.segment<nx_>((i+1)*nx_) = -hd;
                }

                for (int row=0; row<nx_; row++)
                {
                    A_c.insert(i*nx_+row, i*nx_+row)  = -1.0;
                }

                // fill Ax <= B
                A_c.insert(((N_+1)*nx_) + 2*i, (i*nx_))= A11;
                A_c.insert(((N_+1)*nx_) + 2*i, (i*nx_)+1) = A12;

                A_c.insert(((N_+1)*nx_) + 2*i+1, (i*nx_)) = A21;
                A_c.insert(((N_+1)*nx_) + 2*i+1, (i*nx_)+1) = A22;

                lb(((N_+1)*nx_) + 2*i) = -OsqpEigen::INFTY;
                ub(((N_+1)*nx_) + 2*i) = B11;

                lb(((N_+1)*nx_) + 2*i+1) = -OsqpEigen::INFTY;
                ub(((N_+1)*nx_) + 2*i+1) = B22;

                // fill u_min < u < u_max in A_c
                for(int row=0; row<nu_; row++)
                {
                    A_c.insert((N_+1)*nx_+2*(N_+1)+i*nu_+row, (N_+1)*nx_+i*nu_+row) = 1.0;
                }

                lb((N_+1)*nx_ + 2*(N_+1) + i*nu_) = 0.0;
                ub((N_+1)*nx_ + 2*(N_+1) + i*nu_) = max_speed_;

                lb((N_+1)*nx_ + 2*(N_+1) + i*nu_ + 1) = -max_steer_;
                ub((N_+1)*nx_ + 2*(N_+1) + i*nu_ + 1) = max_steer_;
            }

            // fill initial condition in lb and ub
            lb.head(nx_) = -ref_trajectory[0];
            ub.head(nx_) = -ref_trajectory[0];
            lb((N_+1)*nx_ + 2*(N_+1)) = 3.5;
            ub((N_+1)*nx_ + 2*(N_+1)) = 3.5;

            Eigen::SparseMatrix<double> H_matrix_T = H_matrix.transpose();
            Eigen::SparseMatrix<double> sparse_I((N_+1)*(nx_+nu_), (N_+1)*(nx_+nu_));
            sparse_I.setIdentity();

            H_matrix = 0.5*(H_matrix + H_matrix_T) + 0.0000001*sparse_I;

            // osqp Eigen solver from https://robotology.github.io/osqp-eigen/doxygen/doc/html/index.html
            // instantiate the solver
            OsqpEigen::Solver solver;

            solver.settings()->setWarmStart(true);
            solver.data()->setNumberOfVariables((N_+1)*(nx_+nu_));
            solver.data()->setNumberOfConstraints((N_+1)*nx_ + 2*(N_+1) + (N_+1)*nu_);

            if(!solver.data()->setHessianMatrix(H_matrix)) throw "failed to set Hessian";
            if(!solver.data()->setGradient(g)) throw "failed to set gradient";
            if(!solver.data()->setLinearConstraintsMatrix(A_c)) throw "failed to set constraint matrix";
            if(!solver.data()->setLowerBound(lb)) throw "failed to set lower bound";
            if(!solver.data()->setUpperBound(ub)) throw "failed to set upper bound";

            if(!solver.initSolver()) throw "failed to initialize solver";

            planner::Inputs mpc_input;
            mpc_input.header.stamp = ros::Time::now();
            mpc_input.header.frame_id = ego_base_frame_;

            if(!solver.solve())
            {
                mpc_input.feasible = false;
                mpc_input_pub_.publish(mpc_input);
                return;
            }

            Eigen::VectorXd QPSolution = solver.getSolution();

            visualizeMPC(QPSolution);

            const auto start_idx = (N_+1)*nx_;

            for (int i=start_idx; i<QPSolution.size(); i+=2)
            {
                mpc_input.speed.push_back(QPSolution(i));
                mpc_input.steering.push_back(QPSolution(i+1));
            }
            mpc_input.feasible = true;

            mpc_input_pub_.publish(mpc_input);

            solver.clearSolver();
        }

        void getCarDynamics(Eigen::Matrix<double,nx_,nx_>& Ad, Eigen::Matrix<double,nx_,nu_>& Bd, Eigen::Matrix<double,nx_,1>& hd, Eigen::Matrix<double,nx_,1>& state, Eigen::Matrix<double,nu_,1>& input)
        {
            double yaw = state(2);
            double v = input(0);
            double steer = input(1);

            Eigen::VectorXd dynamics(state.size());
            dynamics(0) = input(0)*cos(state(2));
            dynamics(1) = input(0)*sin(state(2));
            dynamics(2) = tan(input(1))*input(0)/C_l_;

            Eigen::Matrix<double,nx_,nx_> Ak, M12;
            Eigen::Matrix<double,nx_,nu_> Bk;

            Ak << 0.0, 0.0, (-v*sin(yaw)), 0.0, 0.0, (v*cos(yaw)), 0.0, 0.0, 0.0;
            Bk << cos(yaw), 0.0, sin(yaw), 0.0, tan(steer)/C_l_, v/(cos(steer)*cos(steer)*C_l_);

            // from document: https://www.diva-portal.org/smash/get/diva2:1241535/FULLTEXT01.pdf, page 50
            Eigen::Matrix<double,nx_+nx_,nx_+nx_> aux, M;
            aux.setZero();
            aux.block<nx_,nx_>(0,0) << Ak;
            aux.block<nx_,nx_>(0,nx_) << Eigen::Matrix3d::Identity();
            M = (aux*Ts_).exp();
            M12 = M.block<nx_,nx_>(0,nx_);

            Eigen::VectorXd hc(3);
            hc = dynamics - (Ak*state + Bk*input);

            // Discretize
            Ad = (Ak*Ts_).exp();
            Bd = M12*Bk;
            hd = M12*hc;
        }

        void halfSpaceConstraints(double& A11, double& A12, double& A21, double& A22, double& B11, double& B12)
        {

            A11 = yp1_ - yc1_;
            A12 = xc1_ - xp1_;
            A21 = yc2_ - yp2_;
            A22 = xp2_ - xc2_;

            B11 = -1*(yc1_*xp1_ - yp1_*xc1_);
            B12 = -1*(yp2_*xc2_ - yc2_*xp2_);
        }

        void visualizeTrajectory(std::vector<Eigen::VectorXd>& ref_traj)
        {
            visualization_msgs::Marker traj_ref;
            geometry_msgs::Point p;

            traj_ref.header.stamp = ros::Time::now();
            traj_ref.header.frame_id = "map";
            traj_ref.id = rviz_id::TRAJECTORY_REF;
            traj_ref.ns = "reference_traj";
            traj_ref.type = visualization_msgs::Marker::LINE_STRIP;
            traj_ref.scale.x = traj_ref.scale.y = 0.04;
            traj_ref.scale.z = 0.02;
            traj_ref.action = visualization_msgs::Marker::ADD;
            traj_ref.pose.orientation.w = 1.0;
            traj_ref.color.g = 1.0;
            traj_ref.color.a = 1.0;

            for (int i=0; i<ref_traj.size(); i++)
            {
                p.x = ref_traj[i](0);
                p.y = ref_traj[i](1);

                traj_ref.points.push_back(p);
            }

            visualization_msgs::MarkerArray traj_markers;
            traj_markers.markers.push_back(traj_ref);

            traj_viz_pub_.publish(traj_markers);
        }

        void visualizeConstraints()
        {

            visualization_msgs::Marker borderlines;
            borderlines.header.frame_id = "map";
            borderlines.id = rviz_id::BORDERLINES;
            borderlines.ns = "borderlines";
            borderlines.type = visualization_msgs::Marker::LINE_LIST;
            borderlines.scale.x = 0.08;
            borderlines.scale.y = 0.08;
            borderlines.action = visualization_msgs::Marker::ADD;
            borderlines.pose.orientation.w = 1.0;
            borderlines.color.r = 1.0;
            borderlines.color.a = 1.0;

//            std::cout << "XC1: " << xc1_ << "\n";
//            std::cout << "YC1: " << yc1_ << "\n";
//            std::cout << "XC2: " << xc2_ << "\n";
//            std::cout << "YC2: " << yc2_ << "\n";
//
//            std::cout << "XP1: " << xp1_ << "\n";
//            std::cout << "YP1: " << yp1_ << "\n";
//            std::cout << "XP2: " << xp2_ << "\n";
//            std::cout << "YP2: " << yp2_ << "\n";

            geometry_msgs::Point c1;
            c1.x = xc1_;
            c1.y = yc1_;

            geometry_msgs::Point c2;
            c2.x = xc2_;
            c2.y = yc2_;

            geometry_msgs::Point p1;
            p1.x = xp1_;
            p1.y = yp1_;

            geometry_msgs::Point p2;
            p2.x = xp2_;
            p2.y = yp2_;

            borderlines.points.push_back(c1);
            borderlines.points.push_back(p1);
            borderlines.points.push_back(c2);
            borderlines.points.push_back(p2);

            visualization_msgs::MarkerArray constraints;
            constraints.markers.push_back(borderlines);

            constraints_viz_pub_.publish(constraints);
        }

        void visualizeMPC(Eigen::VectorXd& QPSolution)
        {
            visualization_msgs::Marker pred_dots;
            pred_dots.header.frame_id = "map";
            pred_dots.id = rviz_id::PREDICTION;
            pred_dots.ns = "predicted_pose";
            pred_dots.type = visualization_msgs::Marker::POINTS;
            pred_dots.scale.x = 0.08;
            pred_dots.scale.y = 0.08;
            pred_dots.scale.z = 0.08;
            pred_dots.action = visualization_msgs::Marker::ADD;
            pred_dots.pose.orientation.w = 1.0;
            pred_dots.color.g = 1.0;
            pred_dots.color.r = 1.0;
            pred_dots.color.a = 1.0;
            for (int i=0; i<N_+1; i++)
            {
                geometry_msgs::Point p;
                p.x = QPSolution(i*nx_);
                p.y = QPSolution(i*nx_+1);
                pred_dots.points.push_back(p);
            }

            visualization_msgs::MarkerArray mpc_markers;
            mpc_markers.markers.push_back(pred_dots);

            mpc_viz_pub_.publish(mpc_markers);

        }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;

    Planner planner(nh);
    planner.initialize();
    ros::spin();

    return 0;
}
