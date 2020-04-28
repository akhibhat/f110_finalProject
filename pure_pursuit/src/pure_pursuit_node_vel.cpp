#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

using namespace std;
// TODO: include ROS msg type headers and libraries you need

class PurePursuit {
private:
    ros::NodeHandle n;
    ros::Publisher nav_pub;
    ros::Publisher vis_pub;
    ros::Publisher vis_array_pub;
    ros::Subscriber pose_sub;
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markers_array;
    ackermann_msgs::AckermannDriveStamped drive_msg;

    double x_current, y_current, yaw_curr, minDistance1, minDistance2, steering_angle;
    double waypoint_x, waypoint_y, steering_angle_new, speed_new;
    double speed;
    std::vector<vector<double>> waypoints, tranformed_waypoints;
    string filename_;
    double L;
    std::vector<double> distances;
    int minIndex1, minIndex2;
    geometry_msgs::TransformStamped map_to_base_link, base_link_to_map;
    // std::vector<double> waypoint_x, waypoint_y, waypoint_z; 

    // TODO: create ROS subscribers and publishers

public:
    PurePursuit() {
        n = ros::NodeHandle();
        x_current = y_current = yaw_curr = minDistance1 = minDistance2 = steering_angle = 0.0;
        filename_ = "/home/mihir/mihir_ws/src/f110_ros/pure_pursuit/best_skirk_nei_vel.csv";
        // filename_ = "/home/mihir/mihir_ws/src/f110_ros/f110_finalProject/gym_pp/data/fg_pp.csv";
       // filename_ = ("/home/mihir/mihir_ws/src/f110_ros/f110_finalProject/waypoints_data/wp_inner1.0.csv");
        L = 2.5;
        minIndex1 = minIndex2 = 0;
        waypoint_x = waypoint_y = steering_angle_new = speed_new = 0.0;
        speed = 0.0;

        // tf2_ros::Buffer tfBuffer;
        // tf2_ros::TransformListener listener(tfBuffer);

        // map_to_base_link = tfBuffer.lookupTransform("base_link", "map", ros::Time(0), ros::Duration(1.0) );
        // base_link_to_map = tfBuffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0) );
        
        get_data(waypoints);
        // print_waypoints(waypoints);
        

        pose_sub = n.subscribe("/gt_pose", 1, &PurePursuit::pose_callback, this);
        nav_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive",1);
        vis_pub = n.advertise<visualization_msgs::Marker>( "/waypoint_vis", 1 );
        // vis_array_pub = n.advertise<visualization_msgs::MarkerArray>( "/waypoint_vis_array", 1 );
        
        
    }

    void get_data(std::vector<vector<double>> &waypoints)
        {
            std::ifstream file(filename_);
            if (!file)
            {
                std::cout << "Invalid path" << "\n";
            }
            std::string line = "";
            while (getline(file, line))
            {
                std::vector<std::string> vec;
                boost::algorithm::split(vec, line, boost::is_any_of(","));
                std::vector<double> waypoint{};
                waypoint.push_back(stod(vec[0]));
                waypoint.push_back(stod(vec[1]));
                waypoint.push_back(stod(vec[2]));
                waypoint.push_back(stod(vec[3]));
                waypoint.push_back(stod(vec[4]));
                waypoints.push_back(waypoint);
            }
            file.close();
        }

    void print_waypoints(std::vector<vector<double>> &waypoints){
        for (const std::vector<double> &v : waypoints)
        {
           for ( double x : v ) std::cout << x << ' ';
           std::cout << std::endl;
        }
        cout << "############################################" << endl;
    }

    void get_marker(visualization_msgs::Marker &marker, double &waypoint_x, double &waypoint_y){
        geometry_msgs::PoseStamped way_point;
        way_point.header.frame_id = "map";
        way_point.pose.position.x = waypoint_x;
        way_point.pose.position.y = waypoint_y;
        way_point.pose.orientation.x = 0.0;
        way_point.pose.orientation.y = 0.0;
        way_point.pose.orientation.z = 0.0;
        way_point.pose.orientation.w = 1.0;

        tf2::doTransform(way_point, way_point, base_link_to_map);

        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = way_point.pose.position.x; //7.64029 
        marker.pose.position.y = way_point.pose.position.y; //9.04719 
        marker.pose.position.z = 0.0;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.a = 0.5; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

    }

    visualization_msgs::MarkerArray get_all_markers(const std::vector<vector<double>> &waypoints){
        int num_row = waypoints.size();
        visualization_msgs::MarkerArray markers_array;
        markers_array.markers.resize(num_row);
        for(int i = 0; i < num_row; i++){
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = waypoints[i][0];
            marker.pose.position.y = waypoints[i][1];
            marker.pose.position.z = 0.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 0.5; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0; 

            markers_array.markers[i] = marker; 
        }
        return markers_array;
    }

    std::vector<double> get_distance(const std::vector<vector<double>> &waypoints, const double &x_current, const double &y_current, const double &L){
        std::vector<double> distance;
        int num_row = waypoints.size();
        for(int i = 0; i < num_row; i++){
            
            // cout << sqrt(pow((x_current - waypoints[i][0]),2) + pow((y_current - waypoints[i][1]),2)) - L << endl;
            // distance.push_back(sqrt(pow((x_current - waypoints[i][0]),2) + pow((y_current - waypoints[i][1]),2)) - L);
            if(waypoints[i][0] > 0){
                distance.push_back(sqrt(pow((waypoints[i][0]),2) + pow((waypoints[i][1]),2)) - L);
            } else {distance.push_back(1000000.00);}
            // cout << "entered loop" << endl;
        }

        return distance;
    }

    void nearest_waypoints(const std::vector<vector<double>> &waypoints, std::vector<double> &distances, int &minIndex1, int &minIndex2, double &minDistance1, double &minDistance2){
        minIndex1 = -1;
        for(int i = 0; i < distances.size(); i++){
            if(distances[i] <= 0){
                if(minIndex1 == -1 || abs(distances[i]) < abs(distances[minIndex1])){
                    minIndex1 = i;
                }
            }
        }

        minIndex2 = -1;
        for(int j = 0; j < distances.size(); j++){
            if (distances[j] >= 0){
                if( j != minIndex1 && (minIndex2 == -1 || abs(distances[j]) > abs(distances[minIndex2]))){
                    minIndex2 = j;
                }
            }
        }
        minDistance1 = distances[minIndex1];
        minDistance2 = distances[minIndex2];

    }

    void get_best_waypoint(const std::vector<vector<double>> &waypoints, double &waypoint_x, double &waypoint_y, int minIndex1, int minIndex2, 
                            double minDistance1, double minDistance2, double steering_angle_new, double speed_new){
        if(minDistance1 == 0.0){
            waypoint_x = waypoints[minIndex1][0];
            waypoint_y = waypoints[minIndex1][1];
            speed_new = waypoints[minIndex1][2];
            steering_angle_new = 
        } else if(minDistance2 == 0){
            waypoint_x = waypoints[minIndex2][0];
            waypoint_y = waypoints[minIndex2][1];
        } else {
            waypoint_x = waypoints[minIndex1][0] + ((abs(minDistance1)/(abs(minDistance1) + abs(minDistance2))) * (waypoints[minIndex2][0] - waypoints[minIndex2][0]));
            waypoint_y = waypoints[minIndex1][1] + ((abs(minDistance1)/(abs(minDistance1) + abs(minDistance2))) * (waypoints[minIndex2][1] - waypoints[minIndex2][1]));
        }
    }

    std::vector<vector<double>> get_transformed_waypoints(const std::vector<vector<double>> &waypoints){
        std::vector<vector<double>> tranformed_waypoints;
        for(int i = 0; i < waypoints.size(); i++){
            std::vector<double> v;
            geometry_msgs::PoseStamped way_point;
            way_point.header.frame_id = "map";
            way_point.pose.position.x = waypoints[i][0];
            way_point.pose.position.y = waypoints[i][1];
            way_point.pose.orientation.x = 0.0;
            way_point.pose.orientation.y = 0.0;
            way_point.pose.orientation.z = 0.0;
            way_point.pose.orientation.w = 1.0;

            tf2::doTransform(way_point, way_point, map_to_base_link);
            // listener.transformPose("base_link", way_point, base_point);
            v.push_back(way_point.pose.position.x);
            v.push_back(way_point.pose.position.y);
            v.push_back(waypoints[i][2]);
            v.push_back(waypoints[i][3]);
            v.push_back(waypoints[i][4]);

            tranformed_waypoints.push_back(v);
        }

        return tranformed_waypoints;
    }

    void get_steering_angle(double &steering_angle, const double waypoint_x, const double waypoint_y){
        steering_angle = 0.6*(2*waypoint_y)/(L*L);
        if (abs(steering_angle) >= 0.41){
            steering_angle = steering_angle > 0 ? 0.41 : -0.41;
        }
    }

    double get_speed(float steering_angle){ 
    //steering_angle = clip(steering_angle, -0.44, 0.44);
        if ((abs(steering_angle) >= 0) && (abs(steering_angle) < 0.174533)){
          speed = 5.0; //2.5, 3.5, 4.5
        } else if ((abs(steering_angle) >= 0.174533) && (abs(steering_angle) < 0.349066)){ //0.349066
          speed = 3.5; //1.5, 2.5, 3.5
        } else{
          speed = 2.5; //0.5, 1.5, 2.0
        }
        return speed;
    }

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener listener(tfBuffer);

        map_to_base_link = tfBuffer.lookupTransform("base_link", "map", ros::Time(0), ros::Duration(1.0) );
        base_link_to_map = tfBuffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0) );

        tranformed_waypoints = get_transformed_waypoints(waypoints);
        // print_waypoints(tranformed_waypoints);

        markers_array = get_all_markers(waypoints);
        vis_array_pub.publish(markers_array);


    	x_current = pose_msg->pose.position.x;
    	y_current = pose_msg->pose.position.y;

    	tf::Quaternion q(
		    pose_msg->pose.orientation.x,
		    pose_msg->pose.orientation.y,
		    pose_msg->pose.orientation.z,
		    pose_msg->pose.orientation.w);

    	tf::Matrix3x3 m(q);
	    double roll, pitch, yaw;
	    m.getRPY(roll, pitch, yaw);
	    yaw_curr = yaw;
	    // ROS_INFO("x: %f, y: %f, yaw_curr: %f", x_current, y_current, yaw_curr);

        // TODO: find the current waypoint to track using methods mentioned in lecture
        distances = get_distance(tranformed_waypoints, x_current, y_current, L);
        // ROS_INFO("after distances");
        nearest_waypoints(tranformed_waypoints, distances, minIndex1, minIndex2, minDistance1, minDistance2);
        // ROS_INFO("after distances 2");
        get_best_waypoint(tranformed_waypoints, waypoint_x, waypoint_y, minIndex1, minIndex2, minDistance1, minDistance2);
        // ROS_INFO("a/fter distances 3");
        get_steering_angle(steering_angle, waypoint_x, waypoint_y);
        ROS_INFO("steering_angle: %f", steering_angle ); //waypoint_x: %f, waypoint_y: %f, //waypoint_x , waypoint_y, 
        // ROS_INFO("waypoint_x: %f, waypoint_y: %f", waypoint_x , waypoint_y );

        // TODO: transform goal point to vehicle frame of reference

        // TODO: calculate curvature/steering angle

        // TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
        
        get_marker(marker, waypoint_x, waypoint_y);
        // vis_pub.publish(marker);

        drive_msg.drive.steering_angle = steering_angle; //steering_angle
        drive_msg.drive.speed = get_speed(steering_angle); //speed
        // nav_pub.publish(drive_msg);
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pp;
    ros::spin();
    return 0;
}