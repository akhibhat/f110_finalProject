#include "gym_rrt/rrt.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>

RRT::~RRT()
{
    ROS_INFO("RRT shutting down!");
}

RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())()), tf2_listener_(tf_buffer_)
{
    folder_path_ = "/home/akhilesh/f110_ws/src/f110_finalProject/";

    filename_ = folder_path_ + "waypoints_data/wp_centre.csv";
    delimiter_ = ",";

    // Load first Map from map_server
    input_map_ = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(2)));

    if (input_map_.data.empty())
    {
        ROS_ERROR("Empty map received!");
    }

    ROS_INFO("Received first map!");

    try
    {
        tf_laser_to_map_ = tf_buffer_.lookupTransform("map", "ego_racecar/laser", ros::Time(0));
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    global_path_ = getOptimalWaypoints();
    path_num_ = 4;

    std::string file1 = folder_path_ + "waypoints_data/wp_inner2.csv";
    std::string file2 = folder_path_ + "waypoints_data/wp_outer2.csv";
    std::string file3 = folder_path_ + "waypoints_data/wp_inner3.csv";  
    std::string file4 = folder_path_ + "waypoints_data/wp_outer3.csv";

    trajectory_options_.push_back(getTrackpoints(file1));
    trajectory_options_.push_back(getTrackpoints(file2));
    trajectory_options_.push_back(getTrackpoints(file3));
    trajectory_options_.push_back(getTrackpoints(file4));

    // ROS Subscribers
    scan_sub_ = nh_.subscribe("/scan", 1, &RRT::scanCallback, this);
    pose_sub_ = nh_.subscribe("/gt_pose", 1, &RRT::poseCallback, this);

    // ROS publishers
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/dynamic_map", 1);
    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
    waypoint_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("waypoint_viz", 1);
    tree_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("tree_viz", 1);

    new_obstacles_ = {};
    clear_obstacles_count_ = 0;
    enable_rrt_star_ = true;

    // RRT parmeters
    SPEED = 1.0;
    num_pts_collision_ = 20;
    lookahead_d_ = 1.8;
    inflation_r_ = 3;
    local_lookahead_d_ = 1.0;
    goal_threshold_ = 0.4;
    max_expansion_dist_ = 1.0;
    max_iters_ = 2000;
    search_radius_ = 1.0;

    ROS_INFO("Created new RRT object");
}

std::vector<Waypoint> RRT::getOptimalWaypoints()
{
    std::ifstream file(filename_);
    if (!file)
    {
        std::cout << "Invalid path" << "\n";
    }

    std::vector<Waypoint> waypoints;

    std::string line = "";

    while(getline(file, line))
    {
        std::vector<std::string> vec;
        boost::algorithm::split(vec, line, boost::is_any_of(delimiter_));
        Waypoint waypoint{};
        waypoint.x = std::stod(vec[0]);
        waypoint.y = std::stod(vec[1]);
        waypoint.speed = 0.0;

        waypoints.push_back(waypoint);
    }

    return waypoints;
}

std::vector<Waypoint> RRT::getTrackpoints(std::string filename)
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
        waypoint.speed = 0.0;

        trajectory.push_back(waypoint);
    }

    file.close();

    return trajectory;
}

void RRT::visualize_trackpoints(double x_local, double y_local, double x_global, double y_global)
{
    visualization_msgs::Marker line;

    line.header.frame_id    = "/map";
    line.header.stamp       = ros::Time::now();
    line.lifetime           = ros::Duration(0.1);
    line.id                 = 1;
    line.ns                 = "rrt";
    line.type               = visualization_msgs::Marker::LINE_STRIP;
    line.action             = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x            = 0.05;
    line.color.r            = 0.0f;
    line.color.g            = 0.0f;
    line.color.b            = 1.0f;
    line.color.a            = 1.0f;

    geometry_msgs::Point current;
    geometry_msgs::Point local;
    geometry_msgs::Point global;

    current.x = current_x_;
    current.y = current_y_;
    current.z =  0;

    local.x = x_local;
    local.y = y_local;
    local.z = 0;

    global.x = x_global;
    global.y = y_global;
    global.z = 0;

    line.points.push_back(current);
    line.points.push_back(local);
    line.points.push_back(global);

    waypoint_viz_pub_.publish(line);
}

void RRT::visualize_tree(std::vector<Node>& tree)
{
    visualization_msgs::Marker line;

    line.header.frame_id = "/map";
    line.header.stamp = ros::Time::now();
    line.lifetime=ros::Duration(0.1);
    line.id = 1;
    line.ns = "rrt_star";
    line.type = visualization_msgs::Marker::LINE_LIST;
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.05;
    line.color.a = 1.0;
    line.color.r = 1.0;
    line.color.g = 0.0;
    line.color.b = 0.0;

    for (int i=0; i<tree.size(); i++)
    {
        geometry_msgs::Point current_node;
        current_node.x = tree[i].x;
        current_node.y = tree[i].y;
        current_node.z = 0;
        int current_node_idx = i;

        for (int j=0; j<tree.size(); j++)
        {
            if (tree[j].parent_idx == current_node_idx)
            {
                geometry_msgs::Point child_node;
                child_node.x = tree[j].x;
                child_node.y = tree[j].y;
                child_node.z = 0;
                
                line.points.push_back(current_node);
                line.points.push_back(child_node);
            }
        }
    }

    tree_viz_pub_.publish(line);
}

void RRT::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    // The scan callback, update your occupancy grid here
    // Args:
    //      scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //      None
    // Update occupancy grid
    try
    {
        tf_laser_to_map_ = tf_buffer_.lookupTransform("map", "ego_racecar/laser", ros::Time(0));
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
    double theta = scan_msg->angle_min + angle_increment*start;

    for (int i=start; i<end; i++)
    {
        const double hit = scan_msg->ranges[i];
        if (std::isnan(hit) || std::isinf(hit)) continue;

        const double x_base_link = hit*cos(theta);
        const double y_base_link = hit*sin(theta);
       
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
        theta += angle_increment;
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
        ROS_DEBUG("Obstacles cleared!");
    }

    map_pub_.publish(input_map_);
    // ROS_INFO("Map updated");
}

void RRT::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    // Pose callback is called when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //      pose_msg(*PoseStamped): pointer to incoming pose message
    // Returns:
    //      None
    current_x_ = pose_msg->pose.position.x;
    current_y_ = pose_msg->pose.position.y;

    bool bool_path_found = false;

    const auto current_pose = Waypoint(pose_msg);

    const auto waypoint = findOptGlobalWaypoint(current_pose);

    std::vector<Node> tree;

    tree.push_back(Node(pose_msg->pose.position.x, pose_msg->pose.position.y, 0.0, -1));

    int count = 0;
    // Main RRT loop
    while (count < max_iters_)
    {
        count++;

        // Sample a node
        const auto sample_node = sample();

        if (isCollided(sample_node[0], sample_node[1])) continue;

        const int nearest_node_idx = nearestIdx(tree, sample_node);

        Node new_node = steer(tree[nearest_node_idx], nearest_node_idx, sample_node);

        const auto current_node_idx = tree.size();

        if (checkCollision(tree[nearest_node_idx], new_node))
        {
            continue;
            ROS_INFO("Collision!");
        }
        else if (enable_rrt_star_)
        {
            new_node.cost = cost(tree, new_node);
            const auto neighbors = near(tree, new_node);

            std::vector<bool> is_neighbor_collided;
            int best_neighbor = new_node.parent_idx;
            
            for (int i=0; i<neighbors.size(); i++)
            {
                const int near_node_idx = neighbors[i];

                if (checkCollision(tree[near_node_idx], new_node))
                {
                    is_neighbor_collided.push_back(true);
                    continue;
                }

                is_neighbor_collided.push_back(false);

                double cost = tree[near_node_idx].cost + lineCost(tree[near_node_idx], new_node);

                if (cost < new_node.cost)
                {
                    new_node.cost = cost;
                    new_node.parent_idx = near_node_idx;
                    best_neighbor = near_node_idx;
                }
            }

            for (int i=0; i<neighbors.size(); i++)
            {
                if (is_neighbor_collided[i] || i == best_neighbor)
                {
                    continue;
                }

                if (tree[neighbors[i]].cost > new_node.cost + lineCost(new_node, tree[neighbors[i]]))
                {
                    tree[neighbors[i]].parent_idx = current_node_idx;
                }
            }
        }

        tree.push_back(new_node);

        if (isGoal(new_node, waypoint.x, waypoint.y))\
        {   
            // ROS_INFO("Inside first if");
            local_path_ = findPath(tree, new_node);
            // ROS_INFO("Path found");

            const auto local_waypoint_d = findLocalWaypoint(current_pose);
            const auto local_waypoint = local_waypoint_d.first;
            const double d = local_waypoint_d.second;

            geometry_msgs::Pose goal_waypoint;
            goal_waypoint.position.x = local_waypoint.x;
            goal_waypoint.position.y = local_waypoint.y;
            goal_waypoint.position.z = 0;
            goal_waypoint.orientation.x = 0;
            goal_waypoint.orientation.y = 0;
            goal_waypoint.orientation.z = 0;
            goal_waypoint.orientation.w = 1;

            geometry_msgs::Pose transform_waypoint;
            tf2::doTransform(goal_waypoint, transform_waypoint, tf_map_to_laser_);

            const double steering_angle = 0.5*2*(transform_waypoint.position.y)/pow(d, 2);

            ackermann_msgs::AckermannDriveStamped drive_msg;
            drive_msg.header.stamp = ros::Time::now();
            drive_msg.header.frame_id = "ego_racecar/base_link";
            drive_msg.drive.steering_angle = steering_angle;
            
            if (steering_angle > 0.41)
            {
                drive_msg.drive.steering_angle = 0.41;
            }
            else if (steering_angle < -0.41)
            {
                drive_msg.drive.steering_angle = -0.41;
            }

            if (abs(steering_angle) < 0.1745)
            {
                drive_msg.drive.speed = SPEED;
            }
            else if (abs(steering_angle) >= 0.1745 && abs(steering_angle) < 0.3491)
            {
                drive_msg.drive.speed = SPEED;
            }
            else
            {
                drive_msg.drive.speed = SPEED;
            }

            bool_path_found = true;

            drive_pub_.publish(drive_msg);

            visualize_trackpoints(goal_waypoint.position.x, goal_waypoint.position.y, waypoint.x, waypoint.y);

            // visualize_tree(tree);

            break;
        }
    }

    if (bool_path_found != true)
    {
        
        auto waypoint_options = findWaypointOptions(current_pose);

        int aliter_max_iters = 1000;

        for (int j = 0; j < waypoint_options.size(); j++)
        {   
            ROS_INFO("LOOP: %d", j);
            const auto waypoint = waypoint_options[j];
            visualize_trackpoints(waypoint.x, waypoint.y, waypoint.x, waypoint.y);


            std::vector<Node> tree;

            tree.push_back(Node(pose_msg->pose.position.x, pose_msg->pose.position.y, 0.0, -1));

            int count = 0;
            // Main RRT loop
            while (count < aliter_max_iters)
            {
                count++;

                // Sample a node
                const auto sample_node = sample();

                if (isCollided(sample_node[0], sample_node[1])) continue;

                const int nearest_node_idx = nearestIdx(tree, sample_node);

                Node new_node = steer(tree[nearest_node_idx], nearest_node_idx, sample_node);

                const auto current_node_idx = tree.size();

                if (checkCollision(tree[nearest_node_idx], new_node))
                {
                    continue;
                    ROS_INFO("Collision!");
                }
                else if (enable_rrt_star_)
                {
                    new_node.cost = cost(tree, new_node);
                    const auto neighbors = near(tree, new_node);

                    std::vector<bool> is_neighbor_collided;
                    int best_neighbor = new_node.parent_idx;
                    
                    for (int i=0; i<neighbors.size(); i++)
                    {
                        const int near_node_idx = neighbors[i];

                        if (checkCollision(tree[near_node_idx], new_node))
                        {
                            is_neighbor_collided.push_back(true);
                            continue;
                        }

                        is_neighbor_collided.push_back(false);

                        double cost = tree[near_node_idx].cost + lineCost(tree[near_node_idx], new_node);

                        if (cost < new_node.cost)
                        {
                            new_node.cost = cost;
                            new_node.parent_idx = near_node_idx;
                            best_neighbor = near_node_idx;
                        }
                    }

                    for (int i=0; i<neighbors.size(); i++)
                    {
                        if (is_neighbor_collided[i] || i == best_neighbor)
                        {
                            continue;
                        }

                        if (tree[neighbors[i]].cost > new_node.cost + lineCost(new_node, tree[neighbors[i]]))
                        {
                            tree[neighbors[i]].parent_idx = current_node_idx;
                        }
                    }
                }

                tree.push_back(new_node);

                if (isGoal(new_node, waypoint.x, waypoint.y))\
                {   
                    local_path_ = findPath(tree, new_node);

                    const auto local_waypoint_d = findLocalWaypoint(current_pose);
                    const auto local_waypoint = local_waypoint_d.first;
                    const double d = local_waypoint_d.second;

                    geometry_msgs::Pose goal_waypoint;
                    goal_waypoint.position.x = local_waypoint.x;
                    goal_waypoint.position.y = local_waypoint.y;
                    goal_waypoint.position.z = 0;
                    goal_waypoint.orientation.x = 0;
                    goal_waypoint.orientation.y = 0;
                    goal_waypoint.orientation.z = 0;
                    goal_waypoint.orientation.w = 1;

                    geometry_msgs::Pose transform_waypoint;
                    tf2::doTransform(goal_waypoint, transform_waypoint, tf_map_to_laser_);

                    const double steering_angle = 0.5*2*(transform_waypoint.position.y)/pow(d, 2);

                    ackermann_msgs::AckermannDriveStamped drive_msg;
                    drive_msg.header.stamp = ros::Time::now();
                    drive_msg.header.frame_id = "ego_racecar/base_link";
                    drive_msg.drive.steering_angle = steering_angle;
                    
                    if (steering_angle > 0.41)
                    {
                        drive_msg.drive.steering_angle = 0.41;
                    }
                    else if (steering_angle < -0.41)
                    {
                        drive_msg.drive.steering_angle = -0.41;
                    }

                    if (abs(steering_angle) < 0.1745)
                    {
                        drive_msg.drive.speed = SPEED;
                    }
                    else if (abs(steering_angle) >= 0.1745 && abs(steering_angle) < 0.3491)
                    {
                        drive_msg.drive.speed = SPEED;
                    }
                    else
                    {
                        drive_msg.drive.speed = SPEED;
                    }

                    ROS_INFO("Publishing velocity alternative %d", j);
                    bool_path_found = true;

                    drive_pub_.publish(drive_msg);
                    // ROS_INFO("Publish to drive!");

                    visualize_trackpoints(goal_waypoint.position.x, goal_waypoint.position.y, waypoint.x, waypoint.y);

                    // visualize_tree(tree);

                    break;
                }
            }

            if (bool_path_found == true)
            {
                break;
            }

        }
    }

    if (bool_path_found == false){
        ROS_INFO("NO PATH FOUND! SETTING SPEED 0");
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.drive.steering_angle = 0.0;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "ego_racecar/base_link";
        drive_msg.drive.speed = 0.0;
        drive_pub_.publish(drive_msg);
    }
}

Waypoint RRT::findOptGlobalWaypoint(const Waypoint current_pose)
{
    try
    {
        tf_map_to_laser_ = tf_buffer_.lookupTransform("ego_racecar/laser", "map", ros::Time(0));
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(0.1).sleep();
    }

    double waypoint_d = std::numeric_limits<double>::max();
    int waypoint_idx = -1;

    for (int i=0; i<global_path_.size(); i++)
    {
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

        const auto waypoint_map_idx = getMapIdx(global_path_[i].x, global_path_[i].y);
        
        if (input_map_.data[waypoint_map_idx] == 100) continue;
        
        double d = sqrt(pow(goal_waypoint.position.x, 2) + pow(goal_waypoint.position.y, 2));
        double diff = std::abs(lookahead_d_ - d);

        if (diff < waypoint_d)
        {
            waypoint_d = diff;
            waypoint_idx = i;
        }
    }

    return global_path_[waypoint_idx];
}

std::vector<Waypoint> RRT::findWaypointOptions(const Waypoint current_pose)
{
    try
    {
        tf_map_to_laser_ = tf_buffer_.lookupTransform("ego_racecar/laser", "map", ros::Time(0));
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(0.1).sleep();
    }

    std::vector<Waypoint> waypoint_options;

    for (int i=0; i<path_num_; i++)
    {
        double waypoint_d = std::numeric_limits<double>::max();
        int waypoint_idx = -1;

        for (int j=0; j<trajectory_options_[i].size(); j++)
        {
            geometry_msgs::Pose goal;
            goal.position.x = trajectory_options_[i][j].x;
            goal.position.y = trajectory_options_[i][j].y;
            goal.position.z = 0;
            goal.orientation.x = 0;
            goal.orientation.y = 0;
            goal.orientation.z = 0;
            goal.orientation.w = 1;

            tf2::doTransform(goal, goal, tf_map_to_laser_);

            if (goal.position.x < 0) continue;

            const auto waypoint_map_idx = getMapIdx(trajectory_options_[i][j].x, trajectory_options_[i][j].y);

            if (input_map_.data[waypoint_map_idx] == 100) continue;

            double d = sqrt(pow(goal.position.x, 2) + pow(goal.position.y, 2));

            double diff = std::abs(lookahead_d_ - d);

            if (diff < waypoint_d)
            {
                waypoint_d = diff;
                waypoint_idx = j;
            }
        }

        waypoint_options.push_back(trajectory_options_[i][waypoint_idx]);
    }

    return waypoint_options;
}


std::pair<Node, double> RRT::findLocalWaypoint(const Waypoint& current_pose)
{
    Node closest_point{};
    double closest_dist_current_pose = std::numeric_limits<double>::max();
    double closest_dist = std::numeric_limits<double>::max();

    for (int i=0; i<local_path_.size(); i++)
    {
        double dist = sqrt(pow(local_path_[i].x- current_pose.x, 2) + 
                            pow(local_path_[i].y - current_pose.y, 2));
        double diff = std::abs(local_lookahead_d_ - dist);

        if (diff < closest_dist)
        {
            closest_dist_current_pose = dist;
            closest_dist = diff;
            closest_point = local_path_[i];
        }
    }

    return {closest_point, closest_dist_current_pose};
}

std::vector<double> RRT::sample()
{
    // This method returns a sampled point from the free space
    // You should restrict so that it samples a small region of interest around the
    // car's current position
    // Args:
    // Returns:
    //      sampled_point(std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;
    // TODO: lookup the documentation on how to use std::mt19937 devices with a 
    // distribution
    std::uniform_real_distribution<>::param_type x_param(0, lookahead_d_);
    std::uniform_real_distribution<>::param_type y_param(-lookahead_d_, lookahead_d_);
    x_dist.param(x_param);
    y_dist.param(y_param);

    geometry_msgs::Pose sample_point;
    sample_point.position.x = x_dist(gen);
    sample_point.position.y = y_dist(gen);
    sample_point.position.z = 0;
    sample_point.orientation.x = 0;
    sample_point.orientation.y = 0;
    sample_point.orientation.z = 0;
    sample_point.orientation.w = 1;

    tf2::doTransform(sample_point, sample_point, tf_laser_to_map_);
    
    sampled_point.push_back(sample_point.position.x);
    sampled_point.push_back(sample_point.position.y);

    return sampled_point;
}

int RRT::getMapIdx(const double x_map, const double y_map)
{
    const auto x_map_idx = static_cast<int>((x_map - input_map_.info.origin.position.x)/input_map_.info.resolution);
    const auto y_map_idx = static_cast<int>((y_map - input_map_.info.origin.position.y)/input_map_.info.resolution);

    return y_map_idx * input_map_.info.width + x_map_idx;
}

std::vector<int> RRT::expandObstacles(const double x_map, const double y_map)
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



int RRT::nearestIdx(const std::vector<Node>& tree, const std::vector<double> &sampled_point)
{
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //      tree (std::vector<Node>): the current RRT tree
    //      sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //      nearest_node (int): index of the nearest node on the tree

    int nearest_node = 0;

    double min_d = std::numeric_limits<double>::max();
    for (size_t i = 0; i < tree.size(); i++)
    {
        const auto dist = sqrt(pow(tree[i].x - sampled_point[0], 2) + pow(tree[i].y - sampled_point[1], 2));

        if (dist < min_d)
        {
            nearest_node = i;
            min_d = dist;
        }
    }

    return nearest_node;
}

Node RRT::steer(const Node &nearest_node, const int nearest_node_idx, const std::vector<double> &sampled_point)
{
    // The function steer:(x,y)->z returns a point such that z is "closer" to y than x
    // The point z returned by the function steer will be such that z minimizer
    // ||z-y|| while at the same time maintaining ||z-x|| <= max_expansion_dist, for
    // a specified max_expansion_dist > 0

    // basically expand the tree towards the sample point (within a max dist)

    // Args:
    //      nearest_node (Node): nearest node on the tree to the sampled point
    //      sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //      new_node (Node): new node created from steering
    const double d = sqrt(pow(sampled_point[0]-nearest_node.x, 2) + pow(sampled_point[1]-nearest_node.y, 2));

    Node new_node{};

    if (d < max_expansion_dist_)
    {
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
    }
    else
    {
        const double theta = atan2(sampled_point[1]-nearest_node.y, sampled_point[0]-nearest_node.x);
        new_node.x = nearest_node.x + cos(theta)*max_expansion_dist_;
        new_node.y = nearest_node.y + sin(theta)*max_expansion_dist_;
    }

    new_node.parent_idx = nearest_node_idx;

    return new_node;
}

bool RRT::checkCollision(Node &nearest_node, Node &new_node)
{
    // This method returns a boolean indicating if the path between the nearest node
    // and the new node created from steering is collision free
    // Args:
    //      nearest_node (Node): nearest node on the tree to the sampled point
    //      new_node (Node): new node created from steering
    // Returns:
    //      collision (bool): true if in collision, false otherwise
    bool collision = false;

    double x_increment = (new_node.x - nearest_node.x)/num_pts_collision_;
    double y_increment = (new_node.y - nearest_node.y)/num_pts_collision_;

    double current_x = nearest_node.x;
    double current_y = nearest_node.y;

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

bool RRT::isCollided(double x_map, double y_map)
{
    // Check whether the point is in collision or not.
    const auto map_idx = getMapIdx(x_map, y_map);
    
    return (input_map_.data[map_idx] == 100);
}

bool RRT::isGoal(Node &latest_added_node, double goal_x, double goal_y)
{
    // This method checks if the latest node added to the tree is close enough
    // (defined by goal_threshold) to the goal so we can terminate the search and find
    // a path
    // Args:
    //      latest_added_node (Node): latest addition to the tree
    //      goal_x (double): x coordinate of the current goal
    //      goal_y (double): y coordinate of the current goal
    // Returns:
    //      close_enough (bool): true if node is close enough to the goal

    bool close_enough = false;

    double d = sqrt(pow(latest_added_node.x - goal_x, 2) + pow(latest_added_node.y - goal_y, 2));

    close_enough = d < goal_threshold_;
    return close_enough;
}

std::vector<Node> RRT::findPath(std::vector<Node> &tree, Node &latest_added_node)
{
    // This method traverses the tree from the node that has been determined as
    // the goal
    // Args:
    //      latest_added_node (Node): latest addition to the tree that has been
    //                                  determined to be close enough to the goal
    // Returns:
    //      path (std::vector<Node>): the vector that represents the order of the
    //                                  nodes traversed as the found path

    std::vector<Node> found_path;
    Node current_node = latest_added_node;

    while (current_node.parent_idx != -1)
    {
        Node track_point{current_node.x, current_node.y, current_node.parent_idx};
        found_path.push_back(track_point);
        current_node = tree[current_node.parent_idx];
    }
    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node)
{
    // This method returns the cost associated with a node
    // Args:
    //      tree (std::vector<Node>): current tree
    //      node (Node): the node the cost is calculated for
    // Returns:
    //      cost (double): the cost value associated with the node

    return tree[node.parent_idx].cost + lineCost(tree[node.parent_idx], node);
}

double RRT::lineCost(Node &n1, Node &n2)
{
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //      n1 (Node)
    //      n2 (Node)
    // Returns:
    //      cost (double): the cost value associated with the line

    return sqrt(pow(n1.x-n2.x,2) + pow(n1.y-n2.y,2));
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node)
{
    // This method returns the set of Nodes in the neighborhood of a node
    // Args:
    //      tree (std::vector<Node>): the current tree
    //      node (Node): node to find neighbors for
    // Returns:
    //      neighborhood (std::vector<int>): the index of node in the neighborhood

    std::vector<int> neighborhood;

    for (int i=0; i<tree.size(); i++)
    {
        const double dist = sqrt(pow(tree[i].x-node.x, 2) + pow(tree[i].y-node.y, 2));

        if (dist < search_radius_)
        {
            neighborhood.push_back(i);
        }
    }
    return neighborhood;
}
