#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class epuck_forward():
    def __init__(self):
        rospy.init_node('epuck_forward',anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher('epuck2/cmd_vel',Twist,queue_size=10)
        r = rospy.Rate(10)
        move_cmd = Twist()
        move_cmd.linear.x = 0.2
        move_cmd.angular.z = 0

        while not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)
            r.sleep()

    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        epuck_forward()
    except:
        rospy.loginfo("GoForward node terminated")
