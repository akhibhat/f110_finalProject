#!/usr/bin/env python3

import rospy
import rospkg
import dubins

from geometry_msgs.msg import Pose2D

#rospack = rospkg.RosPack()
#package_path = rospack.get_path('planner')

from planner.msg import Endpoints
from planner.msg import Path


path_pub = rospy.Publisher("path_points", Path, queue_size=10)

def goalCallback(msg):

    start_pose = (msg.start.x, msg.start.y, msg.start.theta)
    goal_pose = (msg.goal.x, msg.goal.y, msg.goal.theta)

    dubins_path = dubins.shortest_path(start_pose, goal_pose, 1.0)

    num_samples = msg.vel*msg.dt

    path_points = dubins_path.sample_many(num_samples)[0]

    dpath = Path()
    dpath.header.stamp = rospy.Time()
    dpath.header.frame_id = "map"

    dpath.path = []

    for i in range(len(path_points)):

        point = Pose2D()
        point.x = path_points[i][0]
        point.y = path_points[i][1]
        point.theta = path_points[i][2]

        dpath.path.append(point)

    path_pub.publish(dpath)

def listener():
    rospy.init_node("dubins_path", anonymous=True)

    rospy.Subscriber("waypoint", Endpoints, goalCallback)

    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == "__main__":
    listener()
