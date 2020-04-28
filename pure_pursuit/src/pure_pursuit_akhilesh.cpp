#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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


class PurePursuit
{
    public:

        PurePursuit(ros::NodeHandle &nh) : tf_listener_(tf_buffer_)
        {
            nh_ = nh;
            // lookahead_d_ = 2.5;
            waypt_num_ = 0.0;
            delimiter_ = ",";
            filename_ = "/home/mihir/mihir_ws/src/f110_ros/pure_pursuit/best_skirk_nei.csv";
            // filename_ = "/home/mihir/mihir_ws/src/f110_ros/f110_finalProject/waypoints_data/best_skirk_nei_vel.csv";

            // nh_.getParam("/lookahead_distance", lookahead_d_);
            nh_.getParam("/bubble_radius_", bubble_radius_);
            nh_.getParam("/gap_threshold_", gap_threshold_);
            nh_.getParam("/max_scan_", max_scan_);
            nh_.getParam("/lookahead_d_", lookahead_d_);

            nh_.getParam("N_", N_);
            nh_.getParam("Ts_", Ts_);
            nh_.getParam("max_speed_", max_speed_);
            nh_.getParam("max_steer_", max_steer_);
            nh_.getParam("C_l_", C_l_);
            nh_.getParam("q_x_", q_x_);
            nh_.getParam("q_y_", q_y_);
            nh_.getParam("q_yaw_", q_yaw_);
            nh_.getParam("r_v_", r_v_);
            nh_.getParam("r_steer_", r_steer_);

            Q_.setZero();
            R_.setZero();
            Q_.diagonal() << q_x_, q_y_, q_yaw_;
            R_.diagonal() << r_v_, r_steer_;

            truncate_ = false;

            ROS_INFO("Initialized constants!");
        }

        ~PurePursuit(){}

        void initialize()
        {
            ROS_INFO("Initializing publishers and subscribers...");

            drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 100);
            waypoint_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("waypoint_markers", 100);
            pub_markers = nh_.advertise<visualization_msgs::MarkerArray>("/future_pose",100);
            mpc_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/mpc_path",100);
            mpc_markers = nh_.advertise<visualization_msgs::MarkerArray>("/mpc_markers",100);
            mpc_markers2 = nh_.advertise<visualization_msgs::MarkerArray>("/mpc_markers2",100);
            
            scan_sub_ = nh_.subscribe("/scan", 1, &PurePursuit::scanCallback, this);
            pose_sub_ = nh_.subscribe("/gt_pose", 1, &PurePursuit::poseCallback, this);

            ROS_INFO("Reading waypoint data...");
            global_path_ = get_data();

            ROS_INFO("Stored waypoints as vector of Waypoint");

            visualize_waypoint_data();
        }

        struct Waypoint
        {
            double x, y;
            double heading, speed, desired_vel, desired_steering;
        
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
                desired_vel = 0.0;
                desired_steering = 0.0;
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
                desired_vel = 0.0;
                desired_steering = 0.0;
            }
        };

        void add_waypoint_viz(const Waypoint& waypoint, const std::string& frame_id,
                double r, double g, double b, double transparency=0.5,
                double scale_x=0.1, double scale_y=0.1, double scale_z=0.1)
        {
            visualization_msgs::Marker waypoint_marker;
            waypoint_marker.header.frame_id = "base_link";
            waypoint_marker.header.stamp = ros::Time();
            waypoint_marker.ns = "pure_pursuit";
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
        }

        void visualize_waypoint_data()
        {
            for (int i=0; i<global_path_.size(); i++)
            {
                add_waypoint_viz(global_path_[i], "map", 0.0, 0.0, 1.0, 0.5);
            }
        }
        
        std::vector<Waypoint> get_data()
        {
            std::ifstream file(filename_);
            if (!file)
            {
                std::cout << "Invalid path" << "\n";
            }

            std::vector<Waypoint> waypoints;

            std::string line = "";

            while (getline(file, line))
            {
                std::vector<std::string> vec;
                boost::algorithm::split(vec, line, boost::is_any_of(delimiter_));
                Waypoint waypoint{};
                waypoint.x = std::stod(vec[0]);
                waypoint.y = std::stod(vec[1]);
                // waypoint.desired_vel = std::stod(vec[2]);
                // waypoint.desired_steering = 

                waypoints.push_back(waypoint);
            }

            file.close();

            return waypoints;
        }

        size_t find_best_waypoint(const std::vector<Waypoint>& waypoint_data_, double lookahead_d_)//, size_t& last_waypt_idx)
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

        std::vector<Waypoint> transform(const std::vector<Waypoint>& waypoints, const Waypoint& current_pose)//, const tf2_ros::Buffer& tf_buffer, const tf2_ros::TransformListener& tf_listener)
        {
            geometry_msgs::TransformStamped map_to_base_link;
            map_to_base_link = tf_buffer_.lookupTransform("base_link", "map", ros::Time(0));

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

        double PPAngle(const Waypoint& current_pose)
        {   
            // Transform waypoints to baselink frame
            const auto transformed_waypoints = transform(global_path_, current_pose);

            // Find best waypoint to track
            const auto best_waypoint = find_best_waypoint(transformed_waypoints, lookahead_d_);

            // Transform the waypoint to base_link frame
            geometry_msgs::TransformStamped map_to_base_link;
            map_to_base_link = tf_buffer_.lookupTransform("base_link", "map", ros::Time(0));

            geometry_msgs::Pose goal_waypoint;
            goal_waypoint.position.x = global_path_[best_waypoint].x;
            goal_waypoint.position.y = global_path_[best_waypoint].y;
            goal_waypoint.position.z = 0;
            goal_waypoint.orientation.x = 0;
            goal_waypoint.orientation.y = 0;
            goal_waypoint.orientation.z = 0;
            goal_waypoint.orientation.w = 1;

            tf2::doTransform(goal_waypoint, goal_waypoint, map_to_base_link);

            add_waypoint_viz(goal_waypoint, "base_link", 0.0, 1.0, 0.0, 1.0, 0.2, 0.2, 0.2);

            double pp_steering_angle = 0.6*2*goal_waypoint.position.y/(lookahead_d_ * lookahead_d_);

            return pp_steering_angle;
        }

        void PublishMarkers(const std::vector<double>& x_poses_ego_vehicle,
                                    const std::vector<double>& y_poses_ego_vehicle)
        {
            visualization_msgs::MarkerArray viz_msg;

            // ROS_INFO("Publishing Markers");
            viz_msg.markers.clear();

            for(size_t i=0; i<x_poses_ego_vehicle.size(); i++)
            {
                visualization_msgs::Marker point;
                // std::cout << "i: " << i << std::endl;
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
            pub_markers.publish(viz_msg);
            // ROS_INFO("Published Markers");
        }

        void PublishMPCPath(Eigen::VectorXd& QPSolution)
        {
            visualization_msgs::MarkerArray mpc_msg;

            // ROS_INFO("Publishing Markers");
            // mpc_msg.markers.clear();

            for(size_t i=0; i<N_ + 1; i++)
            {
                visualization_msgs::Marker point;
                // std::cout << "i: " << i << std::endl;
                point.header.frame_id = "map";
                point.header.stamp = ros::Time::now();
                point.ns = "point_123";
                point.action =visualization_msgs::Marker::ADD;
                point.pose.orientation.w = 1.0;
                point.id = i;
                point.type = visualization_msgs::Marker::SPHERE;
                point.scale.x = 0.1;
                point.scale.y = 0.1;
                point.scale.z = 0.1;
                point.color.r = 0.0f;
                point.color.g = 1.0f;
                point.color.a = 1.0;
                point.pose.position.x = QPSolution(i*nx_);
                point.pose.position.y = QPSolution(i*nx_ + 1);
                point.lifetime = ros::Duration(10);
                mpc_msg.markers.push_back(std::move(point));
                ROS_INFO("Infeasible Solution at %d: x: %f, y: %f", i, QPSolution(i*nx_), QPSolution(i*nx_ + 1) );
            }
            mpc_markers.publish(mpc_msg);
            ROS_INFO("Published Markers for infeasible Solution");
        }

        void PublishMPCPath2(Eigen::VectorXd& QPSolution)
        {
            visualization_msgs::MarkerArray mpc_msg2;

            // ROS_INFO("Publishing Markers");
            mpc_msg2.markers.clear();

            for(size_t i=0; i<N_ + 1; i++)
            {
                visualization_msgs::Marker point;
                // std::cout << "i: " << i << std::endl;
                point.header.frame_id = "map";
                point.header.stamp = ros::Time::now();
                point.ns = "point_123";
                point.action =visualization_msgs::Marker::ADD;
                point.pose.orientation.w = 1.0;
                point.id = i;
                point.type = visualization_msgs::Marker::SPHERE;
                point.scale.x = 0.1;
                point.scale.y = 0.1;
                point.scale.z = 0.1;
                point.color.r = 0.0f;
                point.color.g = 0.0f;
                point.color.b = 1.0f;
                point.color.a = 1.0;
                point.pose.position.x = QPSolution(i*nx_);;
                point.pose.position.y = QPSolution(i*nx_ + 1);;
                point.lifetime = ros::Duration(10);
                mpc_msg2.markers.push_back(std::move(point));
            }
            mpc_markers2.publish(mpc_msg2);
            // ROS_INFO("Published Markers");
        }

        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
        {
            // updateStaticMap(scan_msg);
            mpc_constraints_.clear();
            ROS_INFO("entering scanCallback");
            if (!truncate_)
            {
                const size_t truncate_size = static_cast<size_t>((3.14/(scan_msg->angle_max - scan_msg->angle_min))*scan_msg->ranges.size());

                start_idx_ = (scan_msg->ranges.size()/2) - (truncate_size/2);
                end_idx_ = (scan_msg->ranges.size()/2) + (truncate_size/2);

                truncate_ = true;

                angle_increment_ = scan_msg->angle_increment;
            }

            double start_idx_theta_ = scan_msg->angle_min + start_idx_*angle_increment_;
            double end_idx_theta_ = scan_msg->angle_min + end_idx_*angle_increment_;

            // ROS_INFO("start_idx_theta_: %f, end_idx_theta_: %f", start_idx_theta_, end_idx_theta_);
            // ROS_DEBUG("Got truncated start and end indices!");

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
                // ROS_INFO("after filtering at %d: %f", i, filtered_ranges[i]);
            }

            // // ROS_INFO("filtered_ranges size: %d", filtered_ranges.size());
            // for(int i=0; i< filtered_ranges.size(); i++)
            // {
            //     // ROS_INFO("Scan Value at %d: %f", i, filtered_ranges[i]);
            // }
            // ROS_DEBUG("Filtered scan ranges of nans and infs");

            const auto closest_pt_it = std::min_element(filtered_ranges.begin(), filtered_ranges.end());
            auto closest_idx = std::distance(filtered_ranges.begin(), closest_pt_it);

            auto closest_dist = filtered_ranges[closest_idx];

            eliminateBubble(&filtered_ranges, closest_idx, closest_dist);

            // ROS_DEBUG("Eliminated safety bubble!");

            findbestGap(filtered_ranges, start_idx_theta_, end_idx_theta_);
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

        void findbestGap(const std::vector<double>& scan_ranges, double& start_idx_theta_, double& end_idx_theta_)
        {
            size_t max_start_idx = 0;
            size_t max_size_ = 0;
            size_t current_start;
            size_t current_size;

            size_t current_idx = 0;

            // ROS_INFO("Scan ranges size insie findbestGap: %d", scan_ranges.size());
            try
            {
                tf_laser_to_map_ = tf_buffer_.lookupTransform("map", "laser", ros::Time(0));
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
                // ROS_INFO("inside ")
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
                    // std::vector<double> start_gap_constraint;
                    // std::vector<double> end_gap_constraint;

                    const double start_angle = start_idx_theta_ + angle_increment_*(max_start_idx);
                    const double end_angle = start_idx_theta_ + angle_increment_*(max_start_idx+max_size_-1);

                    gap.push_back(start_angle);
                    gap.push_back(end_angle);
            
                    gap_dist.push_back(scan_ranges[max_start_idx]);
                    gap_dist.push_back(scan_ranges[max_start_idx+max_size_-1]);

                    // ROS_INFO("max_start_idx: %d", max_start_idx);
                    // ROS_INFO("max_size_: %d", max_size_);


                    best_gaps_.push_back(gap);
                    best_gap_dist_.push_back(gap_dist);
                    // mpc_constraints_.push_back(start_gap_constraint);
                    // mpc_constraints_.push_back(end_gap_constraint);
                }
                current_idx++;
            }

            if (current_size > max_size_)
            {
                // ROS_INFO("Inside if loop");
                max_start_idx = current_start;
                max_size_ = current_size;

                std::vector<double> gap;
                std::vector<size_t> gap_dist;
                std::vector<double> start_gap_constraint;
                std::vector<double> end_gap_constraint;
        
                gap_dist.push_back(scan_ranges[max_start_idx]);
                gap_dist.push_back(scan_ranges[max_start_idx+max_size_-1]);

                // ROS_INFO("max_start_idx: %d", max_start_idx);
                // ROS_INFO("max_size_: %d", max_size_);
        
                const double start_angle = start_idx_theta_ + angle_increment_*(max_start_idx);
                const double end_angle = end_idx_theta_ + angle_increment_*(max_start_idx+max_size_-1);

                // const double start_x_base = scan_ranges[max_start_idx] * cos(start_angle);
                // const double start_y_base = scan_ranges[max_start_idx] * sin(start_angle);

                // const double end_x_base = scan_ranges[max_start_idx+max_size_-1] * cos(end_angle);
                // const double end_y_base = scan_ranges[max_start_idx+max_size_-1] * sin(end_angle);

                // const double start_x_map = start_x_base*cos(yaw) - start_y_base*sin(yaw) + translation.x;
                // const double start_y_map = start_x_base*sin(yaw) + start_y_base*cos(yaw) + translation.y;

                // const double end_x_map = end_x_base*cos(yaw) - end_y_base*sin(yaw) + translation.x;
                // const double end_y_map = end_x_base*sin(yaw) + end_y_base*cos(yaw) + translation.y;

                gap.push_back(start_angle);
                gap.push_back(end_angle);

                // start_gap_constraint.push_back(start_x_map);
                // start_gap_constraint.push_back(start_y_map);

                // end_gap_constraint.push_back(end_x_map);
                // end_gap_constraint.push_back(end_y_map);

                best_gaps_.push_back(gap);
                best_gap_dist_.push_back(gap_dist);
                // mpc_constraints_.push_back(start_gap_constraint);
                // mpc_constraints_.push_back(end_gap_constraint);
            }

            std::vector<double> start_gap_constraint;
            std::vector<double> end_gap_constraint;

            const double start_angle = start_idx_theta_ + angle_increment_*(max_start_idx);
            const double end_angle = start_idx_theta_ + angle_increment_*(max_start_idx+max_size_-1);

            const double start_x_base = scan_ranges[max_start_idx] * cos(start_angle);
            const double start_y_base = scan_ranges[max_start_idx] * sin(start_angle);

            const double end_x_base = scan_ranges[max_start_idx+max_size_-1] * cos(end_angle);
            const double end_y_base = scan_ranges[max_start_idx+max_size_-1] * sin(end_angle);

            const double start_x_map = start_x_base*cos(yaw) - start_y_base*sin(yaw) + translation.x;
            const double start_y_map = start_x_base*sin(yaw) + start_y_base*cos(yaw) + translation.y;

            const double end_x_map = end_x_base*cos(yaw) - end_y_base*sin(yaw) + translation.x;
            const double end_y_map = end_x_base*sin(yaw) + end_y_base*cos(yaw) + translation.y;

            
    
            start_gap_constraint.push_back(start_x_map);
            start_gap_constraint.push_back(start_y_map);
            
            end_gap_constraint.push_back(end_x_map);
            end_gap_constraint.push_back(end_y_map);

            mpc_constraints_.push_back(start_gap_constraint);
            mpc_constraints_.push_back(end_gap_constraint);

            ROS_INFO("constraints_size: %d", mpc_constraints_.size());


        }


        void initMPC(std::vector<Eigen::VectorXd>& ref_trajectory, std::vector<Eigen::VectorXd>& ref_input, double& current_ego_vel_)
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

            // ROS_INFO("Initialized everything in MPC");

            for (int i=0; i<N_+1; i++)
            {
                x_ref = ref_trajectory[i];
                u_ref = ref_input[i];
                getCarDynamics(Ad, Bd, hd, x_ref, u_ref);
                // ROS_INFO("Received Dynamics");
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
                // ROS_INFO("H_matrix done");

                // fill the constraint matrix first with the dynamic constraint
                // x_k+1 = Ad*x_k + Bd*u_k + hd
                if (i < N_)
                {
                    for (int row=0; row<nx_; row++)
                    {
                        for (int col=0; col<nx_; col++)
                        {
                            A_c.insert((i+1)*nx_ + row, i*nx_ + col) = Ad(row, col);
                            // ROS_INFO("Mil gaya bc");
                        }
                    }
                    // ROS_INFO("DEBUG");

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
                // ROS_INFO("A_c first part done");

                for (int row=0; row<nx_; row++)
                {
                    A_c.insert(i*nx_+row, i*nx_+row)  = -1.0;
                }
                // ROS_INFO("A_c second part done");

                // fill Ax <= B
                A_c.insert(((N_+1)*nx_) + 2*i, (i*nx_))= A11;
                A_c.insert(((N_+1)*nx_) + 2*i, (i*nx_)+1) = A12;

                A_c.insert(((N_+1)*nx_) + 2*i+1, (i*nx_)) = A21;
                A_c.insert(((N_+1)*nx_) + 2*i+1, (i*nx_)+1) = A22;
                // ROS_INFO("A_c third part done");

                lb(((N_+1)*nx_) + 2*i) = -OsqpEigen::INFTY;
                ub(((N_+1)*nx_) + 2*i) = B11;

                lb(((N_+1)*nx_) + 2*i+1) = -OsqpEigen::INFTY;
                ub(((N_+1)*nx_) + 2*i+1) = B22;
                // ROS_INFO("upper and lower bound first part done");

                // fill u_min < u < u_max in A_c
                for(int row=0; row<nu_; row++)
                {
                    A_c.insert((N_+1)*nx_+2*(N_+1)+i*nu_+row, (N_+1)*nx_+i*nu_+row) = 1.0;
                }
                // ROS_INFO("A_c fourth part done");

                lb((N_+1)*nx_ + 2*(N_+1) + i*nu_) = 0.0;
                ub((N_+1)*nx_ + 2*(N_+1) + i*nu_) = max_speed_;

                lb((N_+1)*nx_ + 2*(N_+1) + i*nu_ + 1) = -max_steer_;
                ub((N_+1)*nx_ + 2*(N_+1) + i*nu_ + 1) = max_steer_;
                // ROS_INFO("upper and lower bound second part done");
            }

            // ROS_INFO("MATRICES FILLED!");

            // fill initial condition in lb and ub
            lb.head(nx_) = -ref_trajectory[0];
            ub.head(nx_) = -ref_trajectory[0];
            lb((N_+1)*nx_ + 2*(N_+1)) = current_ego_vel_;
            ub((N_+1)*nx_ + 2*(N_+1)) = current_ego_vel_;

            Eigen::SparseMatrix<double> H_matrix_T = H_matrix.transpose();
            Eigen::SparseMatrix<double> sparse_I((N_+1)*(nx_+nu_), (N_+1)*(nx_+nu_));
            sparse_I.setIdentity();

            H_matrix = 0.5*(H_matrix + H_matrix_T) + 0.000001*sparse_I;

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

            // ROS_INFO("Passed first tests");
            if(!solver.initSolver()) throw "failed to initialize solver";

            // ROS_INFO("Passed second tests");
            if(!solver.solve()) {
                
                Eigen::VectorXd QPSolution = solver.getSolution();
                ROS_INFO("Infeasible Solution found!, size: %d", QPSolution.size());
                PublishMPCPath(QPSolution);
                return;
            }

            Eigen::VectorXd QPSolution = solver.getSolution();

            visualizeMPC(QPSolution, ref_trajectory);
            PublishMPCPath2(QPSolution);

            executeMPC(QPSolution);

            solver.clearSolver();
        }

        void halfSpaceConstraints(double& A11, double& A12, double& A21, double& A22, double& B11, double& B12)
        {
            double xc1, xc2, xp1, xp2;
            double yc1, yc2, yp1, yp2;

            ROS_INFO("mpc_constraints_ size: %d", mpc_constraints_.size() );

            xc1 = mpc_constraints_[4 - 2][0];
            yc1 = mpc_constraints_[4 - 2][1];

            xc2 = mpc_constraints_[4 - 1][0];
            yc2 = mpc_constraints_[4 - 1][1];

            xp1 = mpc_constraints_[4 - 4][0];
            yp1 = mpc_constraints_[4 - 4][1];

            xp2 = mpc_constraints_[4 - 3][0];
            yp2 = mpc_constraints_[4 - 3][1];
            // xc1 = -5.0;
            // yc1 = -3.0;

            // xc2 = -5.0;
            // yc2 = 3.0;

            // xp1 = 5.0;
            // yp1 = -3.0;

            // xp2 = 5.0;
            // yp2 = 3.0;

            ROS_INFO("xc1: %f yc1: %f", xc1, yc1);
            ROS_INFO("xc2: %f yc2: %f", xc2, yc2);
            ROS_INFO("xp1: %f yp1: %f", xp1, yp1);
            ROS_INFO("xp2: %f yp2: %f", xp2, yp2);

            A11 = yp1 - yc1;
            A12 = xc1 - xp1;
            A21 = yc2 - yp2;
            A22 = xp2 - xc2;

            // A11 = 0.0;
            // A12 = 10.0;
            // A21 = 0.0;
            // A22 = -10.0;

            B11 = -1*(yc1*xp1 - yp1*xc1);
            B12 = -1*(yp2*xc2 - yc2*xp2);

            // B11 = 30.0;
            // B12 = 30.0;

            ROS_INFO("A11: %f A12: %f", A11, A12);
            ROS_INFO("A21: %f A22: %f", A21, A22);
            ROS_INFO("B11: %f B12: %f", B11, B12);
            // ROS_INFO("xp2: %f yp2: %f", xp2, yp2);
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

        void executeMPC(Eigen::VectorXd& QPSolution)
        {
            double speed = QPSolution((N_+1)*nx_);
            double steering_angle = QPSolution((N_+1)*nx_+1);

            if (steering_angle > 0.415) {steering_angle=0.415;}
            if (steering_angle < -0.415) {steering_angle=-0.415;}

            ackermann_msgs::AckermannDriveStamped drive_msg;
            drive_msg.header.stamp = ros::Time::now();
            drive_msg.header.frame_id = "base_link";
            drive_msg.drive.speed = speed;
            drive_msg.drive.steering_angle = steering_angle;
            drive_msg.drive.steering_angle_velocity = 1.0;

            drive_pub_.publish(drive_msg);
            ROS_INFO("Path Executed");
        }

        void visualizeMPC(Eigen::VectorXd& QPSolution, std::vector<Eigen::VectorXd>& ref_traj)
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

            visualization_msgs::Marker pred_dots;
            pred_dots.header.frame_id = "map";
            pred_dots.id = rviz_id::PREDICTION;
            pred_dots.ns = "predicted_pose";
            pred_dots.type = visualization_msgs::Marker::POINTS;
            pred_dots.scale.x = 1.0;
            pred_dots.scale.y = 0.2;
            pred_dots.scale.z = 0.2;
            pred_dots.action = visualization_msgs::Marker::ADD;
            pred_dots.pose.orientation.w = 1.0;
            pred_dots.color.g = 0.0;
            pred_dots.color.r = 1.0;
            pred_dots.color.a = 1.0;
            for (int i=0; i<N_+1; i++)
            {
                geometry_msgs::Point p;
                p.x = QPSolution(i*nx_);
                p.y = QPSolution(i*nx_+1);
                pred_dots.points.push_back(p);
            }

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

            for (int i=0; i<2; i++)
            {
                geometry_msgs::Point border_point1;
                border_point1.x = mpc_constraints_[i][0];
                border_point1.y = mpc_constraints_[i][1];

                geometry_msgs::Point border_point2;
                border_point2.x = mpc_constraints_[i + 2][0];
                border_point2.y = mpc_constraints_[i + 2][1];

                borderlines.points.push_back(border_point1);
                borderlines.points.push_back(border_point2);
            }

            visualization_msgs::MarkerArray mpc_markers;
            // mpc_markers.markers.push_back(pred_dots);
            mpc_markers.markers.push_back(borderlines);
            // mpc_markers.markers.push_back(traj_ref);

            mpc_viz_pub_.publish(mpc_markers);
        }

        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
        {
            // Read pose message and store heading + global pose in a waypoint
            auto current_pose = Waypoint(pose_msg);

            ego_car_.x = current_pose.x;
            ego_car_.y = current_pose.y;
            ego_car_.theta = current_pose.heading;

            std::vector<double> ego_state1;
            ego_state1.push_back(ego_car_.x);
            ego_state1.push_back(ego_car_.y);

            mpc_constraints_.push_back(ego_state1);

            std::vector<double> ego_state2;
            ego_state2.push_back(ego_car_.x);
            ego_state2.push_back(ego_car_.y);
            mpc_constraints_.push_back(ego_state2);

            ROS_INFO("constraints_size after initial pose: %d", mpc_constraints_.size());

            //note the time when the pose is recorded
            // std::chrono::steady_clock::time_point previous_time = std::chrono::steady_clock::now();

            //linear model weight factor
            double alpha = 0.02;

            //container to store the predicted poses of opponent car
            std::vector<double> x_opp_car_poses;
            std::vector<double> y_opp_car_poses;
            std::vector<double> yaw_opp_car_poses;
            std::vector<double> steer_opp_car_poses;

            //store the current obtained pose as the first pose
            x_opp_car_poses.push_back(current_pose.x);
            y_opp_car_poses.push_back(current_pose.y);
            yaw_opp_car_poses.push_back(current_pose.heading);
            steer_opp_car_poses.push_back(0.0);

            double last_x_opp_car = current_pose.x;
            double last_y_opp_car = current_pose.y;

            double next_x_opp = current_pose.x;
            double next_y_opp = current_pose.y;
            double initial_heading = current_pose.heading; //the current yaw of the odom. Should be zero?

            // ROS_INFO("Current heading: %f", steering_angle);

            // double opp_vel = current_pose.speed;
            double opp_vel = 2.5;
            // ROS_INFO("%f",opp_vel);

            const double pp_steering_angle = PPAngle(current_pose);
            // ROS_INFO()
            

            for(int i=0; i<=N_; i++)
            {
                //note the current time
                // std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
                // size_t dt = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - previous_time).count();
                // ROS_INFO("dt: %f iteration i = %d", dt, i);

                // ROS_INFO("pp_steering_angle: %f at iteration i = %d", pp_steering_angle, i);

                double final_steering_angle = (1 - pow(alpha, i))*(pp_steering_angle)*2.0;
                steer_opp_car_poses.push_back(final_steering_angle); 
                double net_heading = initial_heading + final_steering_angle;
                yaw_opp_car_poses.push_back(net_heading);
                // ROS_INFO("pp_steering_angle: %f iteration i = %d", steering_angle, i);

                next_x_opp = next_x_opp + opp_vel*(cos(net_heading))*Ts_;
                next_y_opp = next_y_opp + opp_vel*(sin(net_heading))*Ts_;

                auto current_pose = Waypoint();
                current_pose.x = next_x_opp;
                current_pose.y = next_y_opp;
                current_pose.heading = net_heading;


                x_opp_car_poses.push_back(next_x_opp);
                y_opp_car_poses.push_back(next_y_opp);
            }

            PublishMarkers(x_opp_car_poses, y_opp_car_poses);

            std::vector<Eigen::VectorXd> ref_trajectory;
            std::vector<Eigen::VectorXd> ref_input;

            for(int i = 0; i < x_opp_car_poses.size(); i++)
            {
                Eigen::VectorXd traj(nx_);
                Eigen::VectorXd input(nu_);

                traj(0) = x_opp_car_poses[i];
                traj(1) = y_opp_car_poses[i];
                traj(2) = yaw_opp_car_poses[i];

                input(0) = opp_vel;
                input(1) = steer_opp_car_poses[i];

                ref_trajectory.push_back(traj);
                ref_input.push_back(input);
                
            }
            ROS_INFO("Exited reference loading!");

            initMPC(ref_trajectory, ref_input, opp_vel);

            // // Transform waypoints to baselink frame
            // const auto transformed_waypoints = transform(global_path_, current_pose);// tf_buffer_, tf_listener_);

            // // Find best waypoint to track
            // const auto best_waypoint = find_best_waypoint(transformed_waypoints, lookahead_d_);//, last_waypt_idx_);

            // // Transform the waypoint to base_link frame
            // geometry_msgs::TransformStamped map_to_base_link;
            // map_to_base_link = tf_buffer_.lookupTransform("base_link", "map", ros::Time(0));

            // geometry_msgs::Pose goal_waypoint;
            // goal_waypoint.position.x = global_path_[best_waypoint].x;
            // goal_waypoint.position.y = global_path_[best_waypoint].y;
            // goal_waypoint.position.z = 0;
            // goal_waypoint.orientation.x = 0;
            // goal_waypoint.orientation.y = 0;
            // goal_waypoint.orientation.z = 0;
            // goal_waypoint.orientation.w = 1;

            // tf2::doTransform(goal_waypoint, goal_waypoint, map_to_base_link);

            // add_waypoint_viz(goal_waypoint, "base_link", 0.0, 1.0, 0.0, 1.0, 0.2, 0.2, 0.2);

            // // Calculate steering angle
            // const double steering_angle = 0.6*2*goal_waypoint.position.y/(lookahead_d_ * lookahead_d_);
            // double current_speed = 4;
           
            // // Publish drive message
            // ackermann_msgs::AckermannDriveStamped drive_msg;
            // drive_msg.header.frame_id = "base_link";
            // drive_msg.drive.steering_angle = steering_angle;

            // // Threshold steering angle for steering lock and velocity for turns
            // if (steering_angle > 0.1)
            // {
            //     if (steering_angle > 0.2)
            //     {
            //         drive_msg.drive.speed = 4.0;
            //         if (steering_angle > 0.4)
            //         {
            //             drive_msg.drive.steering_angle = 0.4;
            //         }
            //     }
            //     else
            //     {
            //         drive_msg.drive.speed = 4.5;
            //     }
            // }
            // else if (steering_angle < -0.1)
            // {
            //     if (steering_angle < -0.2)
            //     {
            //         drive_msg.drive.speed = 4.0;
            //         if (steering_angle < -0.4)
            //         {
            //             drive_msg.drive.speed = -0.4;
            //         }
            //     }
            //     else
            //     {
            //         drive_msg.drive.speed = 4.5;
            //     }
            // }
            // else
            // {
            //     drive_msg.drive.speed = 4.5;
            // }
            // // drive_msg.drive.speed = global_path_[best_waypoint].desired_vel;
            
            // drive_pub_.publish(drive_msg);

        }

    private:
        ros::NodeHandle nh_;

        // Publishers & Subscribers
        ros::Subscriber pose_sub_;
        ros::Publisher drive_pub_;
        ros::Publisher waypoint_viz_pub_;
        ros::Publisher pub_markers;
        ros::Publisher mpc_markers;
        ros::Publisher mpc_markers2;
        ros::Subscriber scan_sub_;
        ros::Publisher mpc_viz_pub_;

        //state variables
        geometry_msgs::Pose2D ego_car_;

        //scan variables
        size_t start_idx_;
        size_t end_idx_;
        double angle_increment_;
        bool truncate_;
        double max_scan_;

        //planner variables
        double bubble_radius_;
        std::vector<std::vector<double>> best_gaps_;
        std::vector<std::vector<size_t>> best_gap_dist_;
        double gap_size_threshold_;
        double gap_threshold_;

        // Other variables
        double lookahead_d_;
        int waypt_num_;
        double speed_;
        size_t last_waypt_idx_;
        std::string filename_;
        std::string delimiter_;

        std::vector<Waypoint> global_path_;
        
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        geometry_msgs::TransformStamped tf_laser_to_map_;

        // MPC variables
        std::vector<std::vector<double>> mpc_constraints_;
        // vector<Eigen::Vector3d> ref_trajectory_;
        // vector<Eigen::VectorXd> ref_input_;

        int N_;
        // const int nx_ = 3;
        // const int nu_ = 2;
        double max_speed_, max_steer_, C_l_, q_x_, q_y_, q_yaw_, r_v_, r_steer_, Ts_;

        Eigen::Matrix<double, nx_, nx_> Q_;
        Eigen::Matrix<double, nu_, nu_> R_;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pure_pursuit");

    ros::NodeHandle nh;

    ROS_INFO("Initialized node, creating class object...");

    PurePursuit pp(nh);
    pp.initialize();

    ros::spin();
    return 0;
}































// void findbestGap(const std::vector<double>& scan_ranges, double& start_theta_, double& end_theta_)
//         {
//             size_t max_start_idx = 0;
//             size_t max_size_ = 0;
//             size_t current_start;
//             size_t current_size;

//             size_t current_idx = 0;
            
//             while (current_idx < scan_ranges.size())
//             {   
                
//                 current_start = current_idx;
//                 current_size = 0;

//                 // ROS_INFO("Entered first while in findbestGap, current_idx: %d, scan_values: %f", current_idx, scan_ranges[current_idx]);
//                 while ((current_idx < scan_ranges.size()) && (scan_ranges[current_idx] > gap_threshold_))
//                 {   ROS_INFO("entered second while in findbestGap");
//                     current_size++;
//                     current_idx++;
//                 }

//                 if (current_size > max_size_)
//                 {
//                     max_start_idx = current_start;
//                     max_size_ = current_size;
//                     current_size = 0;
//                     std::vector<double> gap;
//                     std::vector<size_t> gap_dist;
//                     std::vector<double> start_gap_constraint;
//                     std::vector<double> end_gap_constraint;

//                     gap.push_back(angle_increment_*(max_start_idx-scan_ranges.size()/2));
//                     gap.push_back(angle_increment_*(max_start_idx+max_size_-1-scan_ranges.size()/2));
            
//                     gap_dist.push_back(scan_ranges[max_start_idx]);
//                     gap_dist.push_back(scan_ranges[max_start_idx+max_size_-1]);

//                     try
//                     {
//                         tf_laser_to_map_ = tf_buffer_.lookupTransform("/map", "/laser", ros::Time(0));
//                     }
//                     catch(tf::TransformException& e)
//                     {
//                         ROS_ERROR("%s", e.what());
//                         ros::Duration(0.1).sleep();
//                     }

//                     const auto translation = tf_laser_to_map_.transform.translation;
//                     const auto orientation = tf_laser_to_map_.transform.rotation;

//                     tf2::Quaternion q(orientation.x,
//                                       orientation.y,
//                                       orientation.z,
//                                       orientation.w);
//                     tf2::Matrix3x3 mat(q);

//                     double roll, pitch, yaw;
//                     mat.getRPY(roll, pitch, yaw);

//                     double theta_gap_start = start_theta_ + angle_increment_*max_start_idx;
//                     double theta_gap_end = start_theta_ + angle_increment_*(max_start_idx + max_size_ - 1);

//                     const double x_base_link_start = scan_ranges[max_start_idx] * cos(theta_gap_start);
//                     const double y_base_link_start = scan_ranges[max_start_idx] * sin(theta_gap_start);

//                     const double x_map_start = x_base_link_start*cos(yaw) - y_base_link_start*sin(yaw) + translation.x;
//                     const double y_map_start = x_base_link_start*sin(yaw) + y_base_link_start*cos(yaw) + translation.y;

//                     const double x_base_link_end = scan_ranges[max_start_idx + max_size_-1] * cos(theta_gap_end);
//                     const double y_base_link_end = scan_ranges[max_start_idx + max_size_-1] * sin(theta_gap_end);

//                     const double x_map_end = x_base_link_end*cos(yaw) - y_base_link_end*sin(yaw) + translation.x;
//                     const double y_map_end = x_base_link_end*sin(yaw) + y_base_link_end*cos(yaw) + translation.y;
            
//                     start_gap_constraint.push_back(x_map_start);
//                     start_gap_constraint.push_back(y_map_start);
                    
//                     end_gap_constraint.push_back(x_map_end);
//                     end_gap_constraint.push_back(y_map_end);

//                     // start_gap_constraint.push_back(5.0);
//                     // start_gap_constraint.push_back(3.0);
                    
//                     // end_gap_constraint.push_back(5.0);
//                     // end_gap_constraint.push_back(-3.0);

//                     best_gaps_.push_back(gap);
//                     best_gap_dist_.push_back(gap_dist);
//                     mpc_constraints_.push_back(start_gap_constraint);
//                     mpc_constraints_.push_back(end_gap_constraint);

//                     ROS_INFO("start_gap: %f,  %f", start_gap_constraint[0], start_gap_constraint[1]);
//                     ROS_INFO("end_gap: %f,  %f", end_gap_constraint[0], end_gap_constraint[1]);
//                 }
//                 current_idx++;
//             }

//             if (current_size > max_size_)
//             {
//                 max_start_idx = current_start;
//                 max_size_ = current_size;

//                 std::vector<double> gap;
//                 std::vector<size_t> gap_dist;
//                 std::vector<double> start_gap_constraint;
//                 std::vector<double> end_gap_constraint;

//                 gap.push_back(angle_increment_*(max_start_idx-scan_ranges.size()/2));
//                 gap.push_back(angle_increment_*(max_start_idx+max_size_-1-scan_ranges.size()/2));
        
//                 gap_dist.push_back(scan_ranges[max_start_idx]);
//                 gap_dist.push_back(scan_ranges[max_start_idx+max_size_-1]);
        
//                 try
//                 {
//                     tf_laser_to_map_ = tf_buffer_.lookupTransform("/map", "laser", ros::Time(0));
//                 }
//                 catch(tf::TransformException& e)
//                 {
//                     ROS_ERROR("%s", e.what());
//                     ros::Duration(0.1).sleep();
//                 }

//                 const auto translation = tf_laser_to_map_.transform.translation;
//                 const auto orientation = tf_laser_to_map_.transform.rotation;

//                 tf2::Quaternion q(orientation.x,
//                                   orientation.y,
//                                   orientation.z,
//                                   orientation.w);
//                 tf2::Matrix3x3 mat(q);

//                 double roll, pitch, yaw;
//                 mat.getRPY(roll, pitch, yaw);

//                 double theta_gap_start = start_theta_ + angle_increment_*max_start_idx;
//                 double theta_gap_end = start_theta_ + angle_increment_*(max_start_idx + max_size_ - 1);

//                 const double x_base_link_start = scan_ranges[max_start_idx] * cos(theta_gap_start);
//                 const double y_base_link_start = scan_ranges[max_start_idx] * sin(theta_gap_start);

//                 const double x_map_start = x_base_link_start*cos(yaw) - y_base_link_start*sin(yaw) + translation.x;
//                 const double y_map_start = x_base_link_start*sin(yaw) + y_base_link_start*cos(yaw) + translation.y;

//                 const double x_base_link_end = scan_ranges[max_start_idx + max_size_-1] * cos(theta_gap_end);
//                 const double y_base_link_end = scan_ranges[max_start_idx + max_size_-1] * sin(theta_gap_end);

//                 const double x_map_end = x_base_link_end*cos(yaw) - y_base_link_end*sin(yaw) + translation.x;
//                 const double y_map_end = x_base_link_end*sin(yaw) + y_base_link_end*cos(yaw) + translation.y;
        
//                 start_gap_constraint.push_back(x_map_start);
//                 start_gap_constraint.push_back(y_map_start);
                
//                 end_gap_constraint.push_back(x_map_end);
//                 end_gap_constraint.push_back(y_map_end);

//                 // start_gap_constraint.push_back(5.0);
//                 // start_gap_constraint.push_back(3.0);
                
//                 // end_gap_constraint.push_back(5.0);
//                 // end_gap_constraint.push_back(-3.0);

//                 best_gaps_.push_back(gap);
//                 best_gap_dist_.push_back(gap_dist);
//                 mpc_constraints_.push_back(start_gap_constraint);
//                 mpc_constraints_.push_back(end_gap_constraint);

//                 ROS_INFO("start_gap: %f,  %f", start_gap_constraint[0], start_gap_constraint[1]);
//                 ROS_INFO("end_gap: %f,  %f", end_gap_constraint[0], end_gap_constraint[1]);
//             }

//             // ROS_INFO("Exited best gap successfully!");
//         }