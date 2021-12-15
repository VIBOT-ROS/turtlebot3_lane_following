#!/usr/bin/env python
 
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import sys, tty, termios

msg = """
Lane Following Menu
---------------------------
Start/Stop:

space key or s

CTRL-C to quit
"""

def getKey():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        return key

class ControlLane():
    def __init__(self):
        self.sub_lane = rospy.Subscriber('/detect/lane', Float64, self.cbFollowLane, queue_size = 1)
        self.sub_max_vel = rospy.Subscriber('/control/max_vel', Float64, self.cbGetMaxVel, queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        # print(getKey(),'1')
        self.lastError = 0
        self.prevError = 0
        self.angular_z = 0
        self.MAX_VEL = 0.2
        self.run = False
        self.status = 0
        print(msg)

        rospy.on_shutdown(self.fnShutDown)

    def cbGetMaxVel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data

    def cbFollowLane(self, desired_center):
        key = getKey()
        if key == ' ' or key == 's':
            self.run = not self.run

        if self.status == 20 :
                print(msg)
                self.status = 0
        # key = getKey()
        center = desired_center.data
        if self.run == True:
            error = center - 500

            Kp = 0.0006
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
            self.status +=  1

        elif self.run == False:
            self.stop()
            self.status +=  1

        elif key == '\x03':
            rospy.signal_shutdown('ctrl-c is pressed. Quitting now')
                  

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.stop()

    def stop(self):
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
    settings = termios.tcgetattr(sys.stdin)
    try:
        rospy.init_node('control_lane')
        rospy.loginfo('lane following controller is running')
        node = ControlLane()
        node.main()
        
    except rospy.ROSInterruptException:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        pass
