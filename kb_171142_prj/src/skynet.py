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


    def get_goals(self, PointArray): #get the coordinates of the goals and store them in the array
        self.goals = PointArray.goals

    def get_laser(self, LaserScan): #read the laser data
        self.laser_data = LaserScan.ranges
        self.leftfront = 0
        self.rightfront = 0
        self.left_beams_number = 0
        self.right_beams_number = 0

        for i in self.laser_data[1:90]:
            if i < 0.9:
                self.leftfront = self.laser_data[0:90].index(i)
                break
        for j in self.laser_data[270:360]:
            if j < 0.9:
                self.rightfront = self.laser_data[270:360].index(j)

        for k in self.laser_data[1:90]:
            if k < 0.9:
                self.left_beams_number +=1
        for l in self.laser_data[270:360]:
            if l < 0.9:
                self.right_beams_number +=1

    def e_dist(self, g_pos_x, g_pos_y): #calculates the euclidian distace between the robot and the goal point
        return sqrt(pow((g_pos_x - self.cur_position[0]), 2) + pow((g_pos_y - self.cur_position[1]), 2))

    def robot_angle(self, g_pos_x, g_pos_y): #angle between X-axis, robot,  goal
        return atan2(g_pos_y - self.cur_position[1], g_pos_x - self.cur_position[0])

    def m_line1_angle(self, dest_x, dest_y, orig_x, orig_y): #angle between X-axis, point of origin, goal
        return atan2(dest_y - orig_y, dest_x - orig_x)

    def reach_goals(self): #scenario run
        print('\n' +'\n' +'I need your clothes, your boots and your motorcycle' +'\n' +'\n')
        count = 0
        while count <1:
            time.sleep(2) #making short break so are callbacks are charging up
            count = count + 1
        speed = Twist() # instance of a twist
        flag = 0 # counter of goals
        while not rospy.is_shutdown():
            if self.leftfront !=0 or self.rightfront !=0: #simple algorithm for wall following
                if self.left_beams_number > self.right_beams_number:
                    if self.leftfront < 22:
                        speed.linear.x = 0.15
                        speed.angular.z = -0.3
                    else:
                        speed.linear.x = 0.25
                        speed.angular.z = 0.1

                elif self.left_beams_number < self.right_beams_number:
                    if self.rightfront > 72:
                        speed.linear.x = 0.15
                        speed.angular.z = 0.3
                    else:
                        speed.linear.x = 0.25
                        speed.angular.z = -0.1

            elif self.leftfront == 0 or self.rightfront ==0:
                if self.e_dist(self.goals[flag].x, self.goals[flag].y)<0.3:
                    print('\n' +'\n''goal '+str(flag+1) +' is reached' +'\n' +'\n')
                    flag = flag + 1
                    if flag == 3: #all goals are reached
                        speed.linear.x = 0.0
                        speed.angular.z = 0.0
                        self.vel_pub.publish(speed)
                        print('\n' +'Last goal achieved' +'\n' +'\n' +'Sara Connor is found'+'\n' +'Preparing for termination' +'\n')
                        rospy.signal_shutdown('Close node')
                else:
                    self.robot_angle(self.goals[flag].x, self.goals[flag].y)
                    if flag == 0:
                        arg_line1_angle = self.m_line1_angle(self.goals[0].x, self.goals[0].y, 0,0)

                        if pi/4 < arg_line1_angle < 3*pi/4 or -3*pi/4 < arg_line1_angle < -pi/4:
                            self.arg_search = 666
                        else:
                            self.arg_search = -666

                        if pi/2 <= arg_line1_angle <=pi or -pi <= arg_line1_angle <= -pi/2:
                            self.arg2_speed = 1
                        else:
                            self.arg2_speed = -1

                        if arg_line1_angle <= 0:
                            self.arg1_speed = -1
                        else:
                            self.arg1_speed = 1

                        m_factor = round(self.m_line1_angle(self.goals[0].x, self.goals[0].y, 0, 0) - self.robot_angle(self.goals[0].x, self.goals[0].y), 4)
                        ang_dif = round(self.m_line1_angle(self.goals[0].x, self.goals[0].y, 0,0) - self.cur_position[2], 4)
                    else:
                        arg_line1_angle = self.m_line1_angle(self.goals[flag].x, self.goals[flag].y, self.goals[flag-1].x, self.goals[flag-1].y )

                        if pi/4 < arg_line1_angle < 3*pi/4 or -3*pi/4 < arg_line1_angle < -pi/4:
                            self.arg_search = 666
                        else:
                            self.arg_search = -666

                        if pi/2 <= arg_line1_angle <=pi or -pi <= arg_line1_angle <= -pi/2:
                            self.arg2_speed = 1
                        else:
                            self.arg2_speed = -1

                        if arg_line1_angle < 0:
                            self.arg1_speed = -1
                        else:
                            self.arg1_speed = 1
                        m_factor = round(self.m_line1_angle(self.goals[flag].x, self.goals[flag].y, self.goals[flag-1].x, self.goals[flag-1].y) - self.robot_angle(self.goals[flag].x, self.goals[flag].y), 4)
                        ang_dif = round(self.m_line1_angle(self.goals[flag].x, self.goals[flag].y, self.goals[flag-1].x, self.goals[flag-1].y) - self.cur_position[2], 4)

                    if m_factor < -0.04 or m_factor > 0.04: #point and move towards the m-line
                        if self.arg_search == 666:
                            if m_factor < -0.04 and 0 <= self.cur_position[2] <= 3.1:
                                speed.linear.x = -round(0.008/m_factor, 4)
                                speed.angular.z = 0.3*self.arg1_speed
                            elif m_factor < -0.04 and -3.1 <= self.cur_position[2] <= 0:

                                speed.linear.x = -round(0.008/m_factor, 4)
                                speed.angular.z = -0.3*self.arg1_speed
                            elif m_factor > 0.04 and 0.1 <= self.cur_position[2] <= pi:

                                speed.linear.x = round(0.008/m_factor, 4)
                                speed.angular.z = -0.3*self.arg1_speed
                            elif m_factor > 0.04 and -pi <= self.cur_position[2] <= -0.1:

                                speed.linear.x = round(0.008/m_factor, 4)
                                speed.angular.z = 0.3*self.arg1_speed
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
                        elif self.arg_search == -666:
                            if m_factor < -0.04:
                                if pi/2 <= self.cur_position[2] <= pi or -pi <= self.cur_position[2] <= -1.61:
                                    speed.linear.x = -round(0.008/m_factor, 4)
                                    speed.angular.z = 0.3*self.arg2_speed
                                elif  0 <= self.cur_position[2] <= pi/2 or -1.53 <= self.cur_position[2] <= -0:
                                    speed.linear.x = -round(0.008/m_factor, 4)
                                    speed.angular.z = -0.3*self.arg2_speed
                                else:
                                    if  0 < self.cur_position[2] < pi:
                                        speed.linear.x = 0.2
                                        speed.angular.z = pi/2 - self.cur_position[2]
                                    elif  -pi < self.cur_position[2] < 0:
                                        speed.linear.x = 0.2
                                        speed.angular.z = -pi/2 - self.cur_position[2]
                            elif m_factor > 0.04:
                                if 1.61 <= self.cur_position[2]<= pi or -pi <= self.cur_position[2] <= -pi/2:
                                    speed.linear.x = round(0.008/m_factor, 4)
                                    speed.angular.z = -0.3*self.arg2_speed
                                elif 0 <= self.cur_position[2] <= 1.53 or -pi/2 <= self.cur_position[2] <= -0:
                                    speed.linear.x = round(0.008/m_factor, 4)
                                    speed.angular.z = 0.3*self.arg2_speed
                                else:
                                    if  0 < self.cur_position[2] < pi:
                                        speed.linear.x = 0.2
                                        speed.angular.z = pi/2 - self.cur_position[2]
                                    elif  -pi < self.cur_position[2] < 0:
                                        speed.linear.x = 0.2
                                        speed.angular.z = -pi/2 - self.cur_position[2]
                    else: #move along the m-line
                        if -0.03 < ang_dif < 0.03:
                            speed.linear.x = 0.3
                            speed.angular.z = -m_factor
                        else:
                            speed.linear.x = 0.3
                            speed.angular.z = 2*ang_dif


            self.vel_pub.publish(speed)


        self.rate.sleep()

#make and run the main function
if __name__ == '__main__':
    try:
        var1 = T1000() #instance of a class
        var1.reach_goals() #run main function
    except rospy.ROSInterruptException:
        pass
