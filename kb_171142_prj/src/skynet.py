#!/usr/bin/env python

import rospy
import math
import time
import sys
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from goal_publisher.msg import PointArray
from math import pow, sqrt, degrees, atan2, pi

class T1000:

    def __init__(self):
        rospy.init_node('skynet') #the node is initialized
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #publishes the velocity commands
        self.pos_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.get_state) # reads xyz of a Robot from ModelStates
        self.goals_sub = rospy.Subscriber('/goals', PointArray, self.get_goals) # reads xyz of the goals from PointArray
        self.las_sub = rospy.Subscriber('scan', LaserScan, self.get_laser)
        self.rate = rospy.Rate(10)

    def get_state(self, data): # get the coordinates and deflection of a robot and store them in the array
        quaternion = [data.pose[1].orientation.x, data.pose[1].orientation.y, data.pose[1].orientation.z, data.pose[1].orientation.w]
        self.cur_position = [(data.pose[1].position.x), ( data.pose[1].position.y), euler_from_quaternion(quaternion)[2]]

        #print ('position X is: ' +str(self.cur_position[0]))
        #print ('position Y is: ' +str(self.cur_position[1]))
        #print ('angle    Z is: ' +str(self.cur_position[2]) +'\n')

    def get_goals(self, PointArray): #get the coordinates of the goals and store them in the array
        self.goals = PointArray.goals

        #print('goal 1 = ' + str(self.goals[0].x) + ', ' + str(self.goals[0].y))
        #print('goal 2 = ' + str(self.goals[1].x) + ', ' + str(self.goals[1].y))
        #print('goal 3 = ' + str(self.goals[2].x) + ', ' + str(self.goals[2].y) +'\n')

    def get_laser(self, LaserScan):
        self.laser_data = LaserScan.ranges

    def e_dist(self, g_pos_x, g_pos_y): #calculates the euclidian distace between the robot and the goal point
        return sqrt(pow((g_pos_x - self.cur_position[0]), 2) + pow((g_pos_y - self.cur_position[1]), 2))

    def robot_angle(self, g_pos_x, g_pos_y): #angle between X-axis, robot,  goal
        return atan2(g_pos_y - self.cur_position[1], g_pos_x - self.cur_position[0])

    def m_line1_angle(self, dest_x, dest_y, orig_x, orig_y): #angle between X-axis, point of origin, goal
        return atan2(dest_y - orig_y, dest_x - orig_x)

    def reach_goals(self):
        time.sleep(2)
        speed = Twist()
        flag = 0
        while not rospy.is_shutdown():
            rate=rospy.Rate(10)
            if self.e_dist(self.goals[flag].x, self.goals[flag].y)<0.1:
                print('goal '+str(flag+1) +' is reached')
                flag = flag + 1
                if flag == 3:
                    speed.linear.x = 0.0
                    speed.angular.z = 0.0
                    self.vel_pub.publish(speed)
                    print('\n' +'Last goal achieved' +'\n' +'\n' +'Sara Connor is found'+'\n' +'Preparing for termination' +'\n')
                    rospy.signal_shutdown('Close node')
            else:
                self.robot_angle(self.goals[flag].x, self.goals[flag].y)
                if flag == 0:
                    self.m_line1_angle(self.goals[0].x, self.goals[0].y, 0,0)
                    if self.m_line1_angle(self.goals[0].x, self.goals[0].y, 0,0)<0:
                        self.arg = -1
                    else:
                        self.arg = 1
                    m_factor = round(self.m_line1_angle(self.goals[0].x, self.goals[0].y, 0, 0) - self.robot_angle(self.goals[0].x, self.goals[0].y), 4)
                    ang_dif = round(self.m_line1_angle(self.goals[0].x, self.goals[0].y, 0,0) - self.cur_position[2], 4)
                else:
                    self.m_line1_angle(self.goals[flag].x, self.goals[flag].y, self.goals[flag-1].x, self.goals[flag-1].y )
                    if self.m_line1_angle(self.goals[flag].x, self.goals[flag].y, self.goals[flag-1].x, self.goals[flag-1].y )<0:
                        self.arg = -1
                    else:
                        self.arg = 1
                    m_factor = round(self.m_line1_angle(self.goals[flag].x, self.goals[flag].y, self.goals[flag-1].x, self.goals[flag-1].y) - self.robot_angle(self.goals[flag].x, self.goals[flag].y), 4)
                    ang_dif = round(self.m_line1_angle(self.goals[flag].x, self.goals[flag].y, self.goals[flag-1].x, self.goals[flag-1].y) - self.cur_position[2], 4)
                print('difference is: ' +str(m_factor))
                #print('current z is: ' +str(self.cur_position[2]))

                if m_factor < -0.04 or m_factor > 0.04: #point and move towards the m-line
                    if m_factor < -0.04 and 0.1< self.cur_position[2] <= 3:
                        speed.linear.x = -round(0.01/m_factor, 4)
                        speed.angular.z = 0.3*self.arg
                    elif m_factor < -0.04 and -3 <= self.cur_position[2] <= -0.1:
                        speed.linear.x = -round(0.01/m_factor, 4)
                        speed.angular.z = -0.3*self.arg
                    elif m_factor > 0.04 and 0.1 <= self.cur_position[2] <= 3:
                        speed.linear.x = round(0.01/m_factor, 4)
                        speed.angular.z = -0.3*self.arg
                    elif m_factor > 0.04 and -3<= self.cur_position[2] <= -0.1:
                        speed.linear.x = round(0.01/m_factor, 4)
                        speed.angular.z = 0.3*self.arg
                    else:
                        if 0 < self.cur_position[2] < pi/2 or -0 > self.cur_position[2] > -pi/2:
                            speed.linear.x = 0.2
                            speed.angular.z = -self.cur_position[2]
                        elif pi > self.cur_position[2] > pi/2:
                            speed.linear.x = 0.2
                            speed.angular.z = pi - self.cur_position[2]
                        elif -pi < self.cur_position[2] < -pi/2:
                            speed.linear.x = 0.2
                            speed.angular.z = -pi - self.cur_position[2]
                else:
                    if -0.03 < ang_dif < 0.03:
                        speed.linear.x = 0.3
                        speed.angular.z = -m_factor
                    else:
                        speed.linear.x = 0.3
                        speed.angular.z = 2*ang_dif

            self.vel_pub.publish(speed)
            print ('angle    Z is: ' +str(self.cur_position[2]) +'\n')
            print('speed y ' +str(speed.angular.z))
            print('speed x ' +str(speed.linear.x))
            #print('angle (robot to m_line) ' +str(ang_dif))
            #print('the distance is ' +str(self.e_dist(self.goals[flag].x, self.goals[flag].y)))
            #print('angle (robot to goal1) is ' +str(self.robot_angle(self.goals[flag].x, self.goals[flag].y)))
            #print('angle (origin to goal1) is ' +str(self.m_line1_angle(self.goals[flag].x, self.goals[flag].y, 0,0)))
            #print('angle (origin to goal1) is ' +str(self.m_line1_angle(self.goals[flag].x, self.goals[flag].y, self.goals[flag-1].x,self.goals[flag-1].y)))
            rate.sleep()

        self.rate.sleep()

#make and run the main function
if __name__ == '__main__':
    try:
        var1 = T1000()
        var1.reach_goals()
    except rospy.ROSInterruptException:
        pass
