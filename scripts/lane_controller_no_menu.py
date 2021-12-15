#!/usr/bin/env python
 
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class ControlLane():
    def __init__(self):
        self.sub_lane = rospy.Subscriber('/detect/lane', Float64, self.cbFollowLane, queue_size = 1)
        self.sub_max_vel = rospy.Subscriber('/control/max_vel', Float64, self.cbGetMaxVel, queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        
        self.lastError = 0
        self.prevError = 0
        self.angular_z = 0
        self.MAX_VEL = 0.2

        rospy.on_shutdown(self.fnShutDown)

    def cbGetMaxVel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data

    def cbFollowLane(self, desired_center):

        center = desired_center.data

        error = center - 500

        Kp = 0.0005
        Kd = 0.0005

        self.angular_z += Kp * (error - self.lastError) + Kd * (error - 2 * self.prevError + self.lastError)
        self.prevError = self.lastError
        self.lastError = error
        twist = Twist()
        twist.linear.x = 0.1        
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -max(self.angular_z, -3.0) if self.angular_z < 0 else -min(self.angular_z, 3.0)
        self.pub_cmd_vel.publish(twist)

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist) 

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('control_lane')
    rospy.loginfo('lane following controller is running')
    node = ControlLane()
    node.main()
