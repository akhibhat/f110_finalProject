#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>


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
        speed = odom_msg->twist.twist.linear.x;
    }
};
