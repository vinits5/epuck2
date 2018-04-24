#! /usr/bin/python

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import tf
from tf.transformations import euler_from_quaternion
from std_srvs.srv import Empty
import time

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
        q0 = data.pose.pose.orientation.x
        q1 = data.pose.pose.orientation.y
        q2 = data.pose.pose.orientation.z
        q3 = data.pose.pose.orientation.w

	quaternion = (q0,q1,q2,q3)
	self.euler = euler_from_quaternion(quaternion)

    def orientation(self,angle):
        angle = angle*(180.0/math.pi)
        if angle >= -90:
            angle = 90 - angle
        else:
            angle = - angle - 270
        return angle

    def motion(self,xg,yg):
        loop = True

        #PID Parameters
        Kp = 1      #Proportional constant
        Ki = 0.075   #Integral constant
        Kd = 0      #Differential constant
        E = 0       #Difference of errors
        I = 0       #Sum of all errors
        ai = 0      #Previous orientation of robot
        ei = 0      #Previous error in orientation of robot
        goal = True #True if goal not reached & False if reached

        #Path points:
        path_x = []
        path_y = []

        #PID loop
        while goal:
            yi = self.y         #Current y position
            xi = self.x         #Current x position

            path_x.append(xi)
            path_y.append(yi)

            #Error Calculations
            ad = math.atan2(yg-yi,xg-xi)    #Angle from curent position to Goal
            e = ad - ai                     #Error in current and previous orientations
            e = math.atan2(math.sin(e),math.cos(e)) #Error converted in range -90 to 90

            #PID control
            E = e - ei              #Difference of previous and current error
            I = I + e               #Sum of all erros
            w = Kp*e + Ki*I + Kd*E  #Calculation of angular velocity

            #Command Velocities to robot
            vel = Twist()           #Velocity object
            if e >= 0:              #Check for left or right turn
                w = -w              #For left: -w & for right: w
            vel.angular.z = w
            vel.linear.x = 0.05
            velocity_publisher.publish(vel)

            #Loop running at 10Hz frequency.
            self.rate.sleep()

            #New positions
            yn = self.y         #New y position
            xn = self.x         #New x position
            ai = math.atan2(yn-yi,xn-xi)    #New orientation from goal
            ai = math.atan2(math.sin(ai),math.cos(ai))  #New orientation in range -90 to 90

            #Check the goal condition
            if ((xn-xg)*(xn-xg)+(yn-yg)*(yn-yg)-0.01*0.05)<0:
                print('Goal Reached!')
                vel.angular.z = 0
                vel.linear.x = 0
                velocity_publisher.publish(vel)
                goal = False

        return(path_x,path_y)

    def circular_motion(self):
        path_X = []
        path_Y = []
        y = [0,0.2,0.4,0.6,0.8,1.0]
        x2 = []
        for i in y:
            x3 = 0.25-(i-0.5)*(i-0.5)
            x2.append(x3)
        x = [math.sqrt(i) for i in x2]
        xf = []
        yf = []
        [xf.append(i) for i in x]
        [yf.append(i) for i in y]
        y.reverse()
        [yf.append(i) for i in y]
        x.reverse()
        [xf.append(-i) for i in x]
        for i in range(len(xf)):
            path_x,path_y = self.motion(xf[i],yf[i])
            path_X.append(path_x)
            path_Y.append(path_y)
        return (path_X,path_Y)

if __name__ == '__main__':
    try:
        X = epuck()
        #xg = input('Enter xg: ')
        #yg = input('Enter yg: ')
        #path_x,path_y = X.motion(xg,yg)

        x = input('Enter anything to start: ')
        #reset_world = rospy.ServiceProxy('/gazebo/reset_world',Empty)
        path_X,path_Y = X.circular_motion()
        xx = []
        yy = []
        for i in path_X:
            for j in i:
                xx.append(j)
        for i in path_Y:
            for j in i:
                yy.append(j)
        plt.plot(xx,yy)
        plt.show()
        #reset_world()

    except rospy.ROSInterruptException:
        pass
