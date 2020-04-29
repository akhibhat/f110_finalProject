#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDriveStamped as ADS
from planner.msg import Inputs

class ExecuteMPC():
    def __init__(self):

#        self.steering = []
#        self.velocity = []

        rospy.init_node("execute_mpc", anonymous=True)

        self.input_counter = 0

        drive_topic = rospy.get_param("/drive_topic")
        input_topic = rospy.get_param("/inputs")

        self.ego_car_ = rospy.get_param("/ego_car")
        self.N_ = rospy.get_param("N")

        self.drive_pub_ = rospy.Publisher(drive_topic, ADS, queue_size=1)
        self.input_sub_ = rospy.Subscriber(input_topic, Inputs, self.inputCallback)

    def inputCallback(self, input_msg):

        if input_msg.feasible:

            self.input_counter = 0

            self.steering = input_msg.steering
            self.velocity = input_msg.speed

            self.executeControl(self.steering[self.input_counter], self.velocity[self.input_counter])

        else:
            self.input_counter = self.input_counter + 1

            if self.input_counter > self.N_:
                self.executeControl(0.0, 0.0)
                return

            rospy.loginfo("Executing input number %d from previous trajectory", self.input_counter)
            self.executeControl(self.steering[self.input_counter], self.velocity[self.input_counter])

    def executeControl(self, steer, vel):

        if steer > 0.41:
            steer = 0.41
        elif steer < -0.41:
            steer = -0.41

        drive_msg = ADS()
        drive_msg.header.stamp = rospy.Time()
        drive_msg.header.frame_id = self.ego_car_
        drive_msg.drive.speed = vel
        drive_msg.drive.steering_angle = steer

        self.drive_pub_.publish(drive_msg)

if __name__ == "__main__":
    MPC = ExecuteMPC()
    while not rospy.is_shutdown():
        rospy.spin()

