#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
// TODO: include ROS msg type headers and libraries you need

class PurePursuit {
private:
    ros::NodeHandle n;
    ros::Publisher nav_pub;
    ros::Subscriber pose_sub;
    ackermann_msgs::AckermannDriveStamped drive_msg;

    double x_current, y_current, yaw_curr;

    // TODO: create ROS subscribers and publishers

public:
    PurePursuit() {
        n = ros::NodeHandle();
        x_current = y_current = yaw_curr = 0.0;

        // TODO: create ROS subscribers and publishers
        pose_sub = n.subscribe("/pose", 1, &ReactiveGapFollow::lidar_callback, this);
        nav_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav",1);
        
    }

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
    	x_current = pose_msg->pose.pose.position.x;
    	y_current = pose_msg->pose.pose.position.y;

    	tf::Quaternion q(
		    pose_msg->pose.pose.orientation.x,
		    pose_msg->pose.pose.orientation.y,
		    pose_msg->pose.pose.orientation.z,
		    pose_msg->pose.pose.orientation.w);

    	tf::Matrix3x3 m(q);
	    double roll, pitch, yaw;
	    m.getRPY(roll, pitch, yaw);
	    yaw_curr = yaw;
	    ROS_INFO("x: &f, y: &f, yaw_curr: &f", x_current, y_current, yaw_curr);

        // TODO: find the current waypoint to track using methods mentioned in lecture
        

        // TODO: transform goal point to vehicle frame of reference

        // TODO: calculate curvature/steering angle

        // TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pp;
    ros::spin();
    return 0;
}