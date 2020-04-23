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

class PurePursuit
{
    public:

        PurePursuit(ros::NodeHandle &nh) : tf_listener_(tf_buffer_)
        {
            nh_ = nh;
            lookahead_d_ = 1.8;
            waypt_num_ = 0.0;
            delimiter_ = ",";
            filename_ = "/home/mihir/mihir_ws/src/f110_ros/f110_finalProject/gym_pp/data/fg_pp.csv";
            ROS_INFO("Initialized constants!");
        }

        ~PurePursuit(){}

        void initialize()
        {
            ROS_INFO("Initializing publishers and subscribers...");

            drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
            waypoint_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("waypoint_markers", 100);
            
            pose_sub_ = nh_.subscribe("/gt_pose", 1, &PurePursuit::poseCallback, this);

            ROS_INFO("Reading waypoint data...");
            waypoint_data_ = get_data();

            ROS_INFO("Stored waypoints as vector of Waypoint");

            visualize_waypoint_data();
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

        void add_waypoint_viz(const Waypoint& waypoint, const std::string& frame_id,
                double r, double g, double b, double transparency=0.5,
                double scale_x=0.1, double scale_y=0.1, double scale_z=0.1)
        {
            visualization_msgs::Marker waypoint_marker;
            waypoint_marker.header.frame_id = "ego_racecar/base_link";
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
            for (int i=0; i<waypoint_data_.size(); i++)
            {
                add_waypoint_viz(waypoint_data_[i], "map", 0.0, 0.0, 1.0, 0.5);
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
                waypoint.speed = 0.0;

                waypoints.push_back(waypoint);
            }

            file.close();

            return waypoints;
        }

        size_t find_best_waypoint(const std::vector<Waypoint>& waypoint_data_, double lookahead_d_, size_t& last_waypt_idx)
        {
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

        std::vector<Waypoint> transform(const std::vector<Waypoint>& waypoints, const Waypoint& current_pose, const tf2_ros::Buffer& tf_buffer, const tf2_ros::TransformListener& tf_listener)
        {
            geometry_msgs::TransformStamped map_to_base_link;
            map_to_base_link = tf_buffer.lookupTransform("ego_racecar/base_link", "map", ros::Time(0));

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

        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
        {
            // Read pose message and store heading + global pose in a waypoint
            const auto current_pose = Waypoint(pose_msg);

            // Transform waypoints to baselink frame
            const auto transformed_waypoints = transform(waypoint_data_, current_pose, tf_buffer_, tf_listener_);

            // Find best waypoint to track
            const auto best_waypoint = find_best_waypoint(transformed_waypoints, lookahead_d_, last_waypt_idx_);

            // Transform the waypoint to base_link frame
            geometry_msgs::TransformStamped map_to_base_link;
            map_to_base_link = tf_buffer_.lookupTransform("ego_racecar/base_link", "map", ros::Time(0));

            geometry_msgs::Pose goal_waypoint;
            goal_waypoint.position.x = waypoint_data_[best_waypoint].x;
            goal_waypoint.position.y = waypoint_data_[best_waypoint].y;
            goal_waypoint.position.z = 0;
            goal_waypoint.orientation.x = 0;
            goal_waypoint.orientation.y = 0;
            goal_waypoint.orientation.z = 0;
            goal_waypoint.orientation.w = 1;

            tf2::doTransform(goal_waypoint, goal_waypoint, map_to_base_link);

            add_waypoint_viz(goal_waypoint, "base_link", 0.0, 1.0, 0.0, 1.0, 0.2, 0.2, 0.2);

            // Calculate steering angle
            const double steering_angle = 0.6*2*goal_waypoint.position.y/(lookahead_d_ * lookahead_d_);
            double current_speed = 4;
           
            // Publish drive message
            ackermann_msgs::AckermannDriveStamped drive_msg;
            drive_msg.header.frame_id = "ego_racecar/base_link";
            drive_msg.drive.steering_angle = steering_angle;

            // Threshold steering angle for steering lock and velocity for turns
            if (steering_angle > 0.1)
            {
                if (steering_angle > 0.2)
                {
                    drive_msg.drive.speed = 4.0;
                    if (steering_angle > 0.4)
                    {
                        drive_msg.drive.steering_angle = 0.4;
                    }
                }
                else
                {
                    drive_msg.drive.speed = 4.5;
                }
            }
            else if (steering_angle < -0.1)
            {
                if (steering_angle < -0.2)
                {
                    drive_msg.drive.speed = 4.0;
                    if (steering_angle < -0.4)
                    {
                        drive_msg.drive.speed = -0.4;
                    }
                }
                else
                {
                    drive_msg.drive.speed = 4.5;
                }
            }
            else
            {
                drive_msg.drive.speed = 4.5;
            }
            
            drive_pub_.publish(drive_msg);

        }

    private:
        ros::NodeHandle nh_;

        // Publishers & Subscribers
        ros::Subscriber pose_sub_;
        ros::Publisher drive_pub_;
        ros::Publisher waypoint_viz_pub_;

        // Other variables
        double lookahead_d_;
        int waypt_num_;
        double speed_;
        size_t last_waypt_idx_;
        std::string filename_;
        std::string delimiter_;

        std::vector<Waypoint> waypoint_data_;
        
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

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
