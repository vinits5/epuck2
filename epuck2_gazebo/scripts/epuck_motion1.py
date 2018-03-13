#! /usr/bin/python

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import tf
from tf.transformations import euler_from_quaternion
from std_srvs.srv import Empty

velocity_publisher = rospy.Publisher('epuck2/cmd_vel', Twist, queue_size=10)

class epuck():
    def __init__(self):
        rospy.init_node('epuck_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('epuck2/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/epuck2/odom_diffdrive', Odometry, self.callback)
        self.rate = rospy.Rate(10)

    def callback(self,data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

    def motion(self,actions):
        print('Forward: 8 and Backward: 2 and Quit: 0')
        loop = True
        i = 0
        x = input('Enter anything to start: ')
        while loop and i != len(actions):
            act = actions[i]
            i+=1
            if act == 8:
                goal_x = self.x + 0.1
                dec = True
                while dec:
                    #vel publishing.
                    vel = Twist()
                    if self.x < goal_x:
                        vel.linear.x = 0.1
                    else:
                        vel.linear.x = 0
                        dec = False
                    velocity_publisher.publish(vel)
                    self.rate.sleep()
            elif act == 2:
                goal_x = self.x - 0.1
                dec = True
                while dec:
                    #publish vel.
                    vel = Twist()
                    if self.x > goal_x:
                        vel.linear.x = -0.1
                    else:
                        vel.linear.x = 0
                        dec = False
                    velocity_publisher.publish(vel)
                    self.rate.sleep()
            elif act == 0:
                loop = False

if __name__ == '__main__':
    try:
        x = epuck()
        x.motion([8,2,8])
        reset_world = rospy.ServiceProxy('/gazebo/reset_world',Empty)
        reset_world()

    except rospy.ROSInterruptException:
        pass
