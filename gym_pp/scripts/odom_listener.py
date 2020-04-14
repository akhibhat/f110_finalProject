#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class Listener:
    def __init__(self):

        rospy.init_node("odom_listener", anonymous=True)

        self.pose_pub_ = rospy.Publisher("/gt_pose", PoseStamped, queue_size=1)
        self.odom_sub_ = rospy.Subscriber("/odom", Odometry, self.odomCallback)

        rospy.loginfo("Publishing odom as pose stamped msg")

    def odomCallback(self,odom_msg):

        pose_msg = PoseStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose = odom_msg.pose.pose

        self.pose_pub = rospy.Publisher("/gt_pose", PoseStamped, queue_size=1)

        self.pose_pub.publish(pose_msg)

if __name__ == "__main__":
    listen = Listener()

    rospy.spin()
